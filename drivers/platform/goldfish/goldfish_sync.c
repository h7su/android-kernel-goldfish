// SPDX-License-Identifier: GPL-2.0+

/* The Goldfish sync driver is designed to provide a interface
 * between the underlying host's sync device and the kernel's
 * fence sync framework.
 *
 * The purpose of the device/driver is to enable lightweight creation and
 * signaling of timelines and fences in order to synchronize the guest with
 * host-side graphics events.
 *
 * Each time the interrupt trips, the driver may perform a sync operation.
 */

#include <linux/acpi.h>
#include <linux/dma-fence.h>
#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sync_file.h>
#include <linux/syscalls.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <uapi/linux/goldfish/goldfish_sync.h>

/**
 * struct sync_pt - sync_pt object
 * @base: base dma_fence object
 * @child_list: sync timeline child's list
 * @active_list: sync timeline active child's list
 */
struct sync_pt {
	struct dma_fence base;
	struct list_head child_list;
	struct list_head active_list;
};

/**
 * struct goldfish_sync_timeline - sync object
 * @kref:		reference count on fence.
 * @name:		name of the goldfish_sync_timeline. Useful for debugging
 * @child_list_head:	list of children sync_pts for this
 *			  goldfish_sync_timeline
 * @child_list_lock:	lock protecting @child_list_head and fence.status
 * @active_list_head:	list of active (unsignaled/errored) sync_pts
 */

struct goldfish_sync_timeline {
	struct kref		kref;
	char			name[32];

	/* protected by child_list_lock */
	u64			context;
	int			value;
	struct list_head	child_list_head;
	spinlock_t		child_list_lock;
	struct list_head	active_list_head;
};

struct goldfish_sync_timeline_obj {
	struct goldfish_sync_timeline *sync_tl;

	u32 current_time;

	/* We need to be careful about when we deallocate
	 * this |goldfish_sync_timeline_obj| struct.
	 * In order to ensure proper cleanup, we need to
	 * consider the triggered host-side wait that may
	 * still be in flight when the guest close()'s a
	 * goldfish_sync device's sync context fd (and
	 * destroys the |sync_tl| field above).
	 * The host-side wait may raise IRQ
	 * and tell the kernel to increment the timeline _after_
	 * the |sync_tl| has already been set to null.
	 *
	 * From observations on OpenGL apps and CTS tests, this
	 * happens at some very low probability upon context
	 * destruction or process close, but it does happen
	 * and it needs to be handled properly. Otherwise,
	 * if we clean up the surrounding |goldfish_sync_timeline_obj|
	 * too early, any |handle| field of any host->guest command
	 * might not even point to a null |sync_tl| field,
	 * but to garbage memory or even a reclaimed |sync_tl|.
	 * If we do not count such "pending waits" and kfree the object
	 * immediately upon |goldfish_sync_timeline_destroy|,
	 * we might get mysterous RCU stalls after running a long
	 * time because the garbage memory that is being read
	 * happens to be interpretable as a |spinlock_t| struct
	 * that is currently in the locked state.
	 *
	 * To track when to free the |goldfish_sync_timeline_obj|
	 * itself, we maintain a kref.
	 * The kref essentially counts the timeline itself plus
	 * the number of waits in flight. kref_init/kref_put
	 * are issued on
	 * |goldfish_sync_timeline_create|/|goldfish_sync_timeline_destroy|
	 * and kref_get/kref_put are issued on
	 * |goldfish_sync_fence_create|/|goldfish_sync_timeline_inc|.
	 *
	 * The timeline is destroyed after reference count
	 * reaches zero, which would happen after
	 * |goldfish_sync_timeline_destroy| and all pending
	 * |goldfish_sync_timeline_inc|'s are fulfilled.
	 *
	 * NOTE (1): We assume that |fence_create| and
	 * |timeline_inc| calls are 1:1, otherwise the kref scheme
	 * will not work. This is a valid assumption as long
	 * as the host-side virtual device implementation
	 * does not insert any timeline increments
	 * that we did not trigger from here.
	 *
	 * NOTE (2): The use of kref by itself requires no locks,
	 * but this does not mean everything works without locks.
	 * Related timeline operations do require a lock of some sort,
	 * or at least are not proven to work without it.
	 * In particualr, we assume that all the operations
	 * done on the |kref| field above are done in contexts where
	 * |global_sync_state->mutex_lock| is held. Do not
	 * remove that lock until everything is proven to work
	 * without it!!!
	 */
	struct kref kref;
};

/* The above definitions (command codes, register layout, ioctl definitions)
 * need to be in sync with the following files:
 *
 * Host-side (emulator):
 * external/qemu/android/emulation/goldfish_sync.h
 * external/qemu-android/hw/misc/goldfish_sync.c
 *
 * Guest-side (system image):
 * device/generic/goldfish-opengl/system/egl/goldfish_sync.h
 * device/generic/goldfish/ueventd.ranchu.rc
 * platform/build/target/board/generic/sepolicy/file_contexts
 */
struct goldfish_sync_hostcmd {
	/* sorted for alignment */
	u64 handle;
	u64 hostcmd_handle;
	u32 cmd;
	u32 time_arg;
};

struct goldfish_sync_guestcmd {
	u64 host_command; /* u64 for alignment */
	u64 glsync_handle;
	u64 thread_handle;
	u64 guest_timeline_handle;
};

/* The host operations are: */
enum cmd_id {
	/* Ready signal - used to mark when irq should lower */
	CMD_SYNC_READY			= 0,

	/* Create a new timeline. writes timeline handle */
	CMD_CREATE_SYNC_TIMELINE	= 1,

	/* Create a fence object. reads timeline handle and time argument.
	 * Writes fence fd to the SYNC_REG_HANDLE register.
	 */
	CMD_CREATE_SYNC_FENCE		= 2,

	/* Increments timeline. reads timeline handle and time argument */
	CMD_SYNC_TIMELINE_INC		= 3,

	/* Destroys a timeline. reads timeline handle */
	CMD_DESTROY_SYNC_TIMELINE	= 4,

	/* Starts a wait on the host with the given glsync object and
	 * sync thread handle.
	 */
	CMD_TRIGGER_HOST_WAIT		= 5,
};

/* The host register layout is: */
enum sync_reg_id {
	/* host->guest batch commands */
	SYNC_REG_BATCH_COMMAND			= 0x00,

	/* guest->host batch commands */
	SYNC_REG_BATCH_GUESTCOMMAND		= 0x04,

	/* communicate physical address of host->guest batch commands */
	SYNC_REG_BATCH_COMMAND_ADDR		= 0x08,
	SYNC_REG_BATCH_COMMAND_ADDR_HIGH	= 0x0C, /* 64-bit part */

	/* communicate physical address of guest->host commands */
	SYNC_REG_BATCH_GUESTCOMMAND_ADDR	= 0x10,
	SYNC_REG_BATCH_GUESTCOMMAND_ADDR_HIGH	= 0x14, /* 64-bit part */

	/* signals that the device has been probed */
	SYNC_REG_INIT				= 0x18,
};

#define GOLDFISH_SYNC_MAX_CMDS 32

/* The driver state: */
struct goldfish_sync_state {
	struct miscdevice miscdev;

	char __iomem *reg_base;
	int irq;

	/* Spinlock protects |to_do| / |to_do_end|. */
	spinlock_t lock;

	/* Used to generate unique names. */
	u64 id_counter;

	/* |mutex_lock| protects all concurrent access
	 * to timelines for both kernel and user space.
	 */
	struct mutex mutex_lock;

	/* Buffer holding commands issued from host. */
	struct goldfish_sync_hostcmd to_do[GOLDFISH_SYNC_MAX_CMDS];
	u32 to_do_end;

	/* Buffers for the reading or writing
	 * of individual commands. The host can directly write
	 * to |batch_hostcmd| (and then this driver immediately
	 * copies contents to |to_do|). This driver either replies
	 * through |batch_hostcmd| or simply issues a
	 * guest->host command through |batch_guestcmd|.
	 */
	struct goldfish_sync_hostcmd batch_hostcmd;
	struct goldfish_sync_guestcmd batch_guestcmd;

	/* Used to give this struct itself to a work queue
	 * function for executing actual sync commands.
	 */
	struct work_struct work_item;
};

/* The open file (per fops.open) state: */
struct goldfish_sync_context {
	struct goldfish_sync_state *sync_state;
	struct goldfish_sync_timeline_obj *timeline;
};

static struct goldfish_sync_timeline
*goldfish_dma_fence_parent(struct dma_fence *fence)
{
	return container_of(fence->lock, struct goldfish_sync_timeline,
			    child_list_lock);
}

static struct sync_pt *goldfish_sync_fence_to_sync_pt(struct dma_fence *fence)
{
	return container_of(fence, struct sync_pt, base);
}

/**
 * goldfish_sync_timeline_create_internal() - creates a sync object
 * @name:	sync_timeline name
 *
 * Creates a new sync_timeline. Returns the sync_timeline object or NULL in
 * case of error.
 */
struct goldfish_sync_timeline __must_check
*goldfish_sync_timeline_create_internal(const char *name)
{
	struct goldfish_sync_timeline *obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return NULL;

	kref_init(&obj->kref);
	obj->context = dma_fence_context_alloc(1);
	strlcpy(obj->name, name, sizeof(obj->name));
	INIT_LIST_HEAD(&obj->child_list_head);
	INIT_LIST_HEAD(&obj->active_list_head);
	spin_lock_init(&obj->child_list_lock);

	return obj;
}

static void goldfish_sync_timeline_free_internal(struct kref *kref)
{
	struct goldfish_sync_timeline *obj =
		container_of(kref, struct goldfish_sync_timeline, kref);

	kfree(obj);
}

static void
goldfish_sync_timeline_get_internal(struct goldfish_sync_timeline *obj)
{
	kref_get(&obj->kref);
}

void goldfish_sync_timeline_put_internal(struct goldfish_sync_timeline *obj)
{
	kref_put(&obj->kref, goldfish_sync_timeline_free_internal);
}

/**
 * goldfish_sync_timeline_signal() -
 * signal a status change on a goldfish_sync_timeline
 * @obj:	sync_timeline to signal
 * @inc:	num to increment on timeline->value
 *
 * A sync implementation should call this any time one of it's fences
 * has signaled or has an error condition.
 */
void goldfish_sync_timeline_signal_internal(struct goldfish_sync_timeline *obj,
					    unsigned int inc)
{
	unsigned long flags;
	struct sync_pt *pt, *next;

	spin_lock_irqsave(&obj->child_list_lock, flags);
	obj->value += inc;
	list_for_each_entry_safe(pt, next, &obj->active_list_head,
				 active_list) {
		if (dma_fence_is_signaled_locked(&pt->base))
			list_del_init(&pt->active_list);
	}
	spin_unlock_irqrestore(&obj->child_list_lock, flags);
}

static const struct dma_fence_ops goldfish_sync_timeline_fence_ops;

/**
 * goldfish_sync_pt_create_internal() - creates a sync pt
 * @parent:	fence's parent sync_timeline
 * @size:	size to allocate for this pt
 * @inc:	value of the fence
 *
 * Creates a new sync_pt as a child of @parent.  @size bytes will be
 * allocated allowing for implementation specific data to be kept after
 * the generic sync_timeline struct. Returns the sync_pt object or
 * NULL in case of error.
 */
struct sync_pt __must_check
*goldfish_sync_pt_create_internal(struct goldfish_sync_timeline *obj,
				  unsigned int value)
{
	unsigned long flags;
	struct sync_pt *pt;

	/* leak: no kfree for this kzalloc */
	pt = kzalloc(sizeof(*pt), GFP_KERNEL);
	if (!pt)
		return NULL;

	spin_lock_irqsave(&obj->child_list_lock, flags);
	goldfish_sync_timeline_get_internal(obj);
	dma_fence_init(&pt->base,
		       &goldfish_sync_timeline_fence_ops,
		       &obj->child_list_lock,
		       obj->context,
		       value);
	list_add_tail(&pt->child_list, &obj->child_list_head);
	INIT_LIST_HEAD(&pt->active_list);
	spin_unlock_irqrestore(&obj->child_list_lock, flags);

	return pt;
}

static void
goldfish_sync_pt_destroy_internal(struct goldfish_sync_timeline *obj,
				  struct sync_pt *pt)
{
	unsigned long flags;

	spin_lock_irqsave(&obj->child_list_lock, flags);
	list_del(&pt->child_list);
	dma_fence_put(&pt->base);
	goldfish_sync_timeline_put_internal(obj);
	spin_unlock_irqrestore(&obj->child_list_lock, flags);

	kfree(pt);
}

static const char
*goldfish_sync_timeline_fence_get_driver_name(struct dma_fence *fence)
{
	return "sw_sync";
}

static const char
*goldfish_sync_timeline_fence_get_timeline_name(struct dma_fence *fence)
{
	struct goldfish_sync_timeline *parent =
		goldfish_dma_fence_parent(fence);

	return parent->name;
}

static void goldfish_sync_timeline_fence_release(struct dma_fence *fence)
{
	struct sync_pt *pt = goldfish_sync_fence_to_sync_pt(fence);
	struct goldfish_sync_timeline *parent =
		goldfish_dma_fence_parent(fence);
	unsigned long flags;

	spin_lock_irqsave(fence->lock, flags);
	list_del(&pt->child_list);
	if (!list_empty(&pt->active_list))
		list_del(&pt->active_list);
	spin_unlock_irqrestore(fence->lock, flags);

	goldfish_sync_timeline_put_internal(parent);
	dma_fence_free(fence);
}

static bool goldfish_sync_timeline_fence_signaled(struct dma_fence *fence)
{
	struct goldfish_sync_timeline *parent =
		goldfish_dma_fence_parent(fence);

	return (fence->seqno > parent->value) ? false : true;
}

static bool
goldfish_sync_timeline_fence_enable_signaling(struct dma_fence *fence)
{
	struct sync_pt *pt = goldfish_sync_fence_to_sync_pt(fence);
	struct goldfish_sync_timeline *parent =
		goldfish_dma_fence_parent(fence);

	if (goldfish_sync_timeline_fence_signaled(fence))
		return false;

	list_add_tail(&pt->active_list, &parent->active_list_head);
	return true;
}

static void goldfish_sync_timeline_fence_value_str(struct dma_fence *fence,
						   char *str, int size)
{
	snprintf(str, size, "%d", fence->seqno);
}

static void goldfish_sync_timeline_fence_timeline_value_str(
				struct dma_fence *fence,
				char *str, int size)
{
	struct goldfish_sync_timeline *parent =
		goldfish_dma_fence_parent(fence);

	snprintf(str, size, "%d", parent->value);
}

static const struct dma_fence_ops goldfish_sync_timeline_fence_ops = {
	.get_driver_name = goldfish_sync_timeline_fence_get_driver_name,
	.get_timeline_name = goldfish_sync_timeline_fence_get_timeline_name,
	.enable_signaling = goldfish_sync_timeline_fence_enable_signaling,
	.signaled = goldfish_sync_timeline_fence_signaled,
	.wait = dma_fence_default_wait,
	.release = goldfish_sync_timeline_fence_release,
	.fence_value_str = goldfish_sync_timeline_fence_value_str,
	.timeline_value_str = goldfish_sync_timeline_fence_timeline_value_str,
};

/* We will call |delete_timeline_obj| when the last reference count
 * of the kref is decremented. This deletes the sync
 * timeline object along with the wrapper itself.
 */
static void delete_timeline_obj(struct kref *kref)
{
	struct goldfish_sync_timeline_obj *obj =
		container_of(kref, struct goldfish_sync_timeline_obj, kref);

	goldfish_sync_timeline_put_internal(obj->sync_tl);
	obj->sync_tl = NULL;
	kfree(obj);
}

static void gensym(char *buf, size_t size, struct goldfish_sync_state *state)
{
	snprintf(buf, size, "goldfish_sync:%s:%llu",
		 __func__, state->id_counter);
	++state->id_counter;
}

/* |goldfish_sync_timeline_create| assumes that |global_sync_state->mutex_lock|
 * is held.
 */
static struct goldfish_sync_timeline_obj __must_check
*goldfish_sync_timeline_create(struct goldfish_sync_state *sync_state)
{
	char timeline_name[32];
	struct goldfish_sync_timeline *res_sync_tl = NULL;
	struct goldfish_sync_timeline_obj *res;

	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res)
		return NULL;

	gensym(timeline_name, sizeof(timeline_name), sync_state);

	res_sync_tl = goldfish_sync_timeline_create_internal(timeline_name);
	if (!res_sync_tl) {
		kfree(res);
		return NULL;
	}

	res->sync_tl = res_sync_tl;
	res->current_time = 0;
	kref_init(&res->kref);

	return res;
}

/* |goldfish_sync_fence_create| assumes that |global_sync_state->mutex_lock|
 * is held.
 */
static int
goldfish_sync_fence_create(struct goldfish_sync_timeline_obj *obj, u32 val)
{
	int fd;
	struct sync_pt *syncpt = NULL;
	struct sync_file *sync_file_obj = NULL;
	struct goldfish_sync_timeline *tl;

	if (!obj)
		return -1;

	tl = obj->sync_tl;

	syncpt = goldfish_sync_pt_create_internal(tl, val);
	if (!syncpt)
		return -1;

	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0)
		goto err_cleanup_pt;

	sync_file_obj = sync_file_create(&syncpt->base);
	if (!sync_file_obj)
		goto err_cleanup_fd_pt;

	fd_install(fd, sync_file_obj->file);
	kref_get(&obj->kref);

	return fd;

err_cleanup_fd_pt:
	put_unused_fd(fd);

err_cleanup_pt:
	goldfish_sync_pt_destroy_internal(tl, syncpt);
	return -1;
}

/* |goldfish_sync_timeline_inc| assumes that |global_sync_state->mutex_lock|
 * is held.
 */
static void
goldfish_sync_timeline_inc(struct goldfish_sync_timeline_obj *obj, u32 inc)
{
	/* Just give up if someone else nuked the timeline.
	 * Whoever it was won't care that it doesn't get signaled.
	 */
	if (!obj)
		return;

	goldfish_sync_timeline_signal_internal(obj->sync_tl, inc);
	obj->current_time += inc;

	/* Here, we will end up deleting the timeline object if it
	 * turns out that this call was a pending increment after
	 * |goldfish_sync_timeline_destroy| was called.
	 */
	kref_put(&obj->kref, delete_timeline_obj);
}

/* |goldfish_sync_timeline_destroy| assumes
 * that |global_sync_state->mutex_lock| is held.
 */
static void
goldfish_sync_timeline_destroy(struct goldfish_sync_timeline_obj *obj)
{
	/* See description of |goldfish_sync_timeline_obj| for why we
	 * should not immediately destroy |obj|
	 */
	kref_put(&obj->kref, delete_timeline_obj);
}

static inline void
goldfish_sync_cmd_queue(struct goldfish_sync_state *sync_state,
			u32 cmd,
			u64 handle,
			u32 time_arg,
			u64 hostcmd_handle)
{
	struct goldfish_sync_hostcmd *to_add;

	WARN_ON(sync_state->to_do_end == GOLDFISH_SYNC_MAX_CMDS);

	to_add = &sync_state->to_do[sync_state->to_do_end];

	to_add->cmd = cmd;
	to_add->handle = handle;
	to_add->time_arg = time_arg;
	to_add->hostcmd_handle = hostcmd_handle;

	++sync_state->to_do_end;
}

static inline void
goldfish_sync_hostcmd_reply(struct goldfish_sync_state *sync_state,
			    u32 cmd,
			    u64 handle,
			    u32 time_arg,
			    u64 hostcmd_handle)
{
	unsigned long irq_flags;
	struct goldfish_sync_hostcmd *batch_hostcmd =
		&sync_state->batch_hostcmd;

	spin_lock_irqsave(&sync_state->lock, irq_flags);

	batch_hostcmd->cmd = cmd;
	batch_hostcmd->handle = handle;
	batch_hostcmd->time_arg = time_arg;
	batch_hostcmd->hostcmd_handle = hostcmd_handle;
	writel(0, sync_state->reg_base + SYNC_REG_BATCH_COMMAND);

	spin_unlock_irqrestore(&sync_state->lock, irq_flags);
}

static inline void
goldfish_sync_send_guestcmd(struct goldfish_sync_state *sync_state,
			    u32 cmd,
			    u64 glsync_handle,
			    u64 thread_handle,
			    u64 timeline_handle)
{
	unsigned long irq_flags;
	struct goldfish_sync_guestcmd *batch_guestcmd =
		&sync_state->batch_guestcmd;

	spin_lock_irqsave(&sync_state->lock, irq_flags);

	batch_guestcmd->host_command = cmd;
	batch_guestcmd->glsync_handle = glsync_handle;
	batch_guestcmd->thread_handle = thread_handle;
	batch_guestcmd->guest_timeline_handle = timeline_handle;
	writel(0, sync_state->reg_base + SYNC_REG_BATCH_GUESTCOMMAND);

	spin_unlock_irqrestore(&sync_state->lock, irq_flags);
}

/* |goldfish_sync_interrupt| handles IRQ raises from the virtual device.
 * In the context of OpenGL, this interrupt will fire whenever we need
 * to signal a fence fd in the guest, with the command
 * |CMD_SYNC_TIMELINE_INC|.
 * However, because this function will be called in an interrupt context,
 * it is necessary to do the actual work of signaling off of interrupt context.
 * The shared work queue is used for this purpose. At the end when
 * all pending commands are intercepted by the interrupt handler,
 * we call |schedule_work|, which will later run the actual
 * desired sync command in |goldfish_sync_work_item_fn|.
 */
static irqreturn_t
goldfish_sync_interrupt_impl(struct goldfish_sync_state *sync_state)
{
	struct goldfish_sync_hostcmd *batch_hostcmd =
			&sync_state->batch_hostcmd;

	spin_lock(&sync_state->lock);
	for (;;) {
		u32 nextcmd;
		u32 command_r;
		u64 handle_rw;
		u32 time_r;
		u64 hostcmd_handle_rw;

		readl(sync_state->reg_base + SYNC_REG_BATCH_COMMAND);
		nextcmd = batch_hostcmd->cmd;

		if (nextcmd == 0)
			break;

		command_r = nextcmd;
		handle_rw = batch_hostcmd->handle;
		time_r = batch_hostcmd->time_arg;
		hostcmd_handle_rw = batch_hostcmd->hostcmd_handle;

		goldfish_sync_cmd_queue(sync_state,
					command_r,
					handle_rw,
					time_r,
					hostcmd_handle_rw);
	}
	spin_unlock(&sync_state->lock);

	schedule_work(&sync_state->work_item);
	return IRQ_HANDLED;
}

static const struct file_operations goldfish_sync_fops;

static irqreturn_t goldfish_sync_interrupt(int irq, void *dev_id)
{
	struct goldfish_sync_state *sync_state = dev_id;

	return (sync_state->miscdev.fops == &goldfish_sync_fops) ?
		goldfish_sync_interrupt_impl(sync_state) : IRQ_NONE;
}

/* We expect that commands will come in at a slow enough rate
 * so that incoming items will not be more than
 * GOLDFISH_SYNC_MAX_CMDS.
 *
 * This is because the way the sync device is used,
 * it's only for managing buffer data transfers per frame,
 * with a sequential dependency between putting things in
 * to_do and taking them out. Once a set of commands is
 * queued up in to_do, the user of the device waits for
 * them to be processed before queuing additional commands,
 * which limits the rate at which commands come in
 * to the rate at which we take them out here.
 *
 * We also don't expect more than MAX_CMDS to be issued
 * at once; there is a correspondence between
 * which buffers need swapping to the (display / buffer queue)
 * to particular commands, and we don't expect there to be
 * enough display or buffer queues in operation at once
 * to overrun GOLDFISH_SYNC_MAX_CMDS.
 */
static u32 goldfish_sync_grab_commands(struct goldfish_sync_state *sync_state,
				       struct goldfish_sync_hostcmd *dst)
{
	u32 to_do_end;
	u32 i;
	unsigned long irq_flags;

	spin_lock_irqsave(&sync_state->lock, irq_flags);

	to_do_end = sync_state->to_do_end;
	for (i = 0; i < to_do_end; i++)
		dst[i] = sync_state->to_do[i];
	sync_state->to_do_end = 0;

	spin_unlock_irqrestore(&sync_state->lock, irq_flags);

	return to_do_end;
}

void goldfish_sync_run_hostcmd(struct goldfish_sync_state *sync_state,
			       struct goldfish_sync_hostcmd *todo)
{
	struct goldfish_sync_timeline_obj *timeline =
		(struct goldfish_sync_timeline_obj *)(uintptr_t)todo->handle;
	int sync_fence_fd;

	switch (todo->cmd) {
	case CMD_SYNC_READY:
		break;

	case CMD_CREATE_SYNC_TIMELINE:
		timeline = goldfish_sync_timeline_create(sync_state);
		WARN_ON(!timeline);
		goldfish_sync_hostcmd_reply(sync_state,
					    CMD_CREATE_SYNC_TIMELINE,
					    (uintptr_t)timeline,
					    0,
					    todo->hostcmd_handle);
		break;

	case CMD_CREATE_SYNC_FENCE:
		sync_fence_fd = goldfish_sync_fence_create(timeline,
							   todo->time_arg);
		goldfish_sync_hostcmd_reply(sync_state,
					    CMD_CREATE_SYNC_FENCE,
					    sync_fence_fd,
					    0,
					    todo->hostcmd_handle);
		break;

	case CMD_SYNC_TIMELINE_INC:
		WARN_ON(!timeline);
		if (timeline)
			goldfish_sync_timeline_inc(timeline, todo->time_arg);
		break;

	case CMD_DESTROY_SYNC_TIMELINE:
		WARN_ON(!timeline);
		if (timeline)
			goldfish_sync_timeline_destroy(timeline);
		break;
	}
}


/* |goldfish_sync_work_item_fn| does the actual work of servicing
 * host->guest sync commands. This function is triggered whenever
 * the IRQ for the goldfish sync device is raised. Once it starts
 * running, it grabs the contents of the buffer containing the
 * commands it needs to execute (there may be multiple, because
 * our IRQ is active high and not edge triggered), and then
 * runs all of them one after the other.
 */
static void goldfish_sync_work_item_fn(struct work_struct *input)
{
	struct goldfish_sync_state *sync_state =
		container_of(input, struct goldfish_sync_state, work_item);

	struct goldfish_sync_hostcmd to_run[GOLDFISH_SYNC_MAX_CMDS];
	u32 to_do_end;
	u32 i;

	mutex_lock(&sync_state->mutex_lock);

	to_do_end = goldfish_sync_grab_commands(sync_state, to_run);

	for (i = 0; i < to_do_end; i++)
		goldfish_sync_run_hostcmd(sync_state, &to_run[i]);

	mutex_unlock(&sync_state->mutex_lock);
}

/* Guest-side interface: file operations */

/* Goldfish sync context and ioctl info.
 *
 * When a sync context is created by open()-ing the goldfish sync device, we
 * create a sync context (|goldfish_sync_context|).
 *
 * Currently, the only data required to track is the sync timeline itself
 * along with the current time, which are all packed up in the
 * |goldfish_sync_timeline_obj| field. We use a |goldfish_sync_context|
 * as the filp->private_data.
 *
 * Next, when a sync context user requests that work be queued and a fence
 * fd provided, we use the |goldfish_sync_ioctl_info| struct, which holds
 * information about which host handles to touch for this particular
 * queue-work operation. We need to know about the host-side sync thread
 * and the particular host-side GLsync object. We also possibly write out
 * a file descriptor.
 */
static int goldfish_sync_open(struct inode *inode, struct file *filp)
{
	struct goldfish_sync_context *sync_context =
		kzalloc(sizeof(*sync_context), GFP_KERNEL);

	if (!sync_context)
		return -ENOMEM;

	sync_context->sync_state =
		container_of(filp->private_data,
			     struct goldfish_sync_state,
			     miscdev);
	sync_context->timeline = NULL;

	filp->private_data = sync_context;
	return 0;
}

static int goldfish_sync_release(struct inode *inode, struct file *file)
{
	struct goldfish_sync_context *sync_context = file->private_data;
	struct goldfish_sync_state *sync_state = sync_context->sync_state;

	mutex_lock(&sync_state->mutex_lock);

	if (sync_context->timeline)
		goldfish_sync_timeline_destroy(sync_context->timeline);

	mutex_unlock(&sync_state->mutex_lock);

	kfree(sync_context);
	return 0;
}

/* |goldfish_sync_ioctl| is the guest-facing interface of goldfish sync
 * and is used in conjunction with eglCreateSyncKHR to queue up the
 * actual work of waiting for the EGL sync command to complete,
 * possibly returning a fence fd to the guest.
 */
static long goldfish_sync_ioctl(struct file *file,
				unsigned int cmd,
				unsigned long arg)
{
	struct goldfish_sync_context *sync_context = file->private_data;
	struct goldfish_sync_state *sync_state = sync_context->sync_state;
	struct goldfish_sync_timeline_obj *timeline;
	struct goldfish_sync_ioctl_info ioctl_data;
	int fd_out = -1;

	switch (cmd) {
	case GOLDFISH_SYNC_IOC_QUEUE_WORK:
		if (copy_from_user(&ioctl_data,
				   (void __user *)arg,
				   sizeof(ioctl_data)))
			return -EFAULT;

		if (!ioctl_data.host_syncthread_handle_in)
			return -EFAULT;

		if (mutex_lock_interruptible(&sync_state->mutex_lock))
			return -ERESTARTSYS;

		if (!sync_context->timeline)
			sync_context->timeline =
				goldfish_sync_timeline_create(sync_state);

		if (!sync_context->timeline) {
			mutex_unlock(&sync_state->mutex_lock);
			return -ENOMEM;
		}

		timeline = sync_context->timeline;
		fd_out = goldfish_sync_fence_create(timeline,
						    timeline->current_time + 1);
		ioctl_data.fence_fd_out = fd_out;

		if (copy_to_user((void __user *)arg,
						&ioctl_data,
						sizeof(ioctl_data))) {
			sys_close(fd_out);

			/* We won't be doing an increment, kref_put immediately.
			 */
			kref_put(&timeline->kref, delete_timeline_obj);
			mutex_unlock(&sync_state->mutex_lock);
			return -EFAULT;
		}

		/* We are now about to trigger a host-side wait;
		 * accumulate on |pending_waits|.
		 */
		goldfish_sync_send_guestcmd(sync_state,
				CMD_TRIGGER_HOST_WAIT,
				ioctl_data.host_glsync_handle_in,
				ioctl_data.host_syncthread_handle_in,
				(u64)(uintptr_t)(sync_context->timeline));

		mutex_unlock(&sync_state->mutex_lock);
		return 0;

	default:
		return -ENOTTY;
	}
}

static bool setup_verify_batch_cmd_addr(char *reg_base,
					void *batch_addr,
					u32 addr_offset,
					u32 addr_offset_high)
{
	u64 batch_addr_phys;
	u64 batch_addr_phys_test_lo;
	u64 batch_addr_phys_test_hi;

	batch_addr_phys = virt_to_phys(batch_addr);
	writel(lower_32_bits(batch_addr_phys), reg_base + addr_offset);
	writel(upper_32_bits(batch_addr_phys), reg_base + addr_offset_high);

	batch_addr_phys_test_lo = readl(reg_base + addr_offset);
	batch_addr_phys_test_hi = readl(reg_base + addr_offset_high);

	batch_addr_phys = batch_addr_phys_test_lo |
		(batch_addr_phys_test_hi << 32);

	return virt_to_phys(batch_addr) == batch_addr_phys;
}

static const struct file_operations goldfish_sync_fops = {
	.owner = THIS_MODULE,
	.open = goldfish_sync_open,
	.release = goldfish_sync_release,
	.unlocked_ioctl = goldfish_sync_ioctl,
	.compat_ioctl = goldfish_sync_ioctl,
};

static void fill_miscdevice(struct miscdevice *misc)
{
	misc->name = GOLDFISH_SYNC_DEVICE_NAME;
	misc->minor = MISC_DYNAMIC_MINOR;
	misc->fops = &goldfish_sync_fops;
}

static int goldfish_sync_probe(struct platform_device *pdev)
{
	struct goldfish_sync_state *sync_state;
	struct resource *ioresource;
	int result;

	sync_state = devm_kzalloc(&pdev->dev, sizeof(*sync_state), GFP_KERNEL);
	if (!sync_state)
		return -ENOMEM;

	spin_lock_init(&sync_state->lock);
	mutex_init(&sync_state->mutex_lock);
	INIT_WORK(&sync_state->work_item, goldfish_sync_work_item_fn);

	ioresource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ioresource)
		return -ENODEV;

	sync_state->reg_base =
		devm_ioremap(&pdev->dev, ioresource->start, PAGE_SIZE);
	if (!sync_state->reg_base)
		return -ENOMEM;

	result = platform_get_irq(pdev, 0);
	if (result < 0)
		return -ENODEV;

	sync_state->irq = result;

	result = devm_request_irq(&pdev->dev,
				  sync_state->irq,
				  goldfish_sync_interrupt,
				  IRQF_SHARED,
				  pdev->name,
				  sync_state);
	if (result)
		return -ENODEV;

	if (!setup_verify_batch_cmd_addr(sync_state->reg_base,
				&sync_state->batch_hostcmd,
				SYNC_REG_BATCH_COMMAND_ADDR,
				SYNC_REG_BATCH_COMMAND_ADDR_HIGH))
		return -ENODEV;

	if (!setup_verify_batch_cmd_addr(sync_state->reg_base,
				&sync_state->batch_guestcmd,
				SYNC_REG_BATCH_GUESTCOMMAND_ADDR,
				SYNC_REG_BATCH_GUESTCOMMAND_ADDR_HIGH))
		return -ENODEV;

	fill_miscdevice(&sync_state->miscdev);
	result = misc_register(&sync_state->miscdev);
	if (result)
		return -ENODEV;

	writel(0, sync_state->reg_base + SYNC_REG_INIT);

	platform_set_drvdata(pdev, sync_state);

	return 0;
}

static int goldfish_sync_remove(struct platform_device *pdev)
{
	struct goldfish_sync_state *sync_state = platform_get_drvdata(pdev);

	misc_deregister(&sync_state->miscdev);
	return 0;
}

static const struct of_device_id goldfish_sync_of_match[] = {
	{ .compatible = "google,goldfish-sync", },
	{},
};
MODULE_DEVICE_TABLE(of, goldfish_sync_of_match);

static const struct acpi_device_id goldfish_sync_acpi_match[] = {
	{ "GFSH0006", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, goldfish_sync_acpi_match);

static struct platform_driver goldfish_sync = {
	.probe = goldfish_sync_probe,
	.remove = goldfish_sync_remove,
	.driver = {
		.name = GOLDFISH_SYNC_DEVICE_NAME,
		.of_match_table = goldfish_sync_of_match,
		.acpi_match_table = ACPI_PTR(goldfish_sync_acpi_match),
	}
};
module_platform_driver(goldfish_sync);

MODULE_AUTHOR("Google, Inc.");
MODULE_DESCRIPTION("Android QEMU Sync Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.0");
