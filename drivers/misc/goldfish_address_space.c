// SPDX-License-Identifier: GPL-2.0

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include <linux/device.h>
#include <linux/pci_regs.h>
#include <linux/pci_ids.h>
#include <linux/pci.h>

#include <uapi/linux/goldfish/goldfish_address_space.h>

MODULE_DESCRIPTION("A Goldfish driver that allocates address space ranges in "
		   "the guest to populate them later in the host. This allows "
		   "sharing host's memory with the guest.");
MODULE_AUTHOR("Roman Kiryanov <rkir@google.com>");
MODULE_LICENSE("GPLv2");

enum address_space_register_id {
	ADDRESS_SPACE_REGISTER_COMMAND = 0,
	ADDRESS_SPACE_REGISTER_STATUS = 4,
	ADDRESS_SPACE_REGISTER_GUEST_PAGE_SIZE = 8,
	ADDRESS_SPACE_REGISTER_BLOCK_SIZE_LOW = 12,
	ADDRESS_SPACE_REGISTER_BLOCK_SIZE_HIGH = 16,
	ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_LOW = 20,
	ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_HIGH = 24,
};

enum address_space_command_id {
	ADDRESS_SPACE_COMMAND_ALLOCATE_BLOCK = 1,
	ADDRESS_SPACE_COMMAND_DEALLOCATE_BLOCK = 2,
};

#define ADDRESS_SPACE_DEVICE_NAME	"goldfish_address_space"
#define ADDRESS_SPACE_PCI_VENDOR_ID	0x607D
#define ADDRESS_SPACE_PCI_DEVICE_ID	0xF153
#define ADDRESS_SPACE_MAGIC_U32		(ADDRESS_SPACE_PCI_VENDOR_ID << 16 | ADDRESS_SPACE_PCI_DEVICE_ID)
#define ADDRESS_SPACE_ALLOCATED_BLOCKS_INITIAL_CAPACITY 32

enum address_space_pci_bar_id {
	ADDRESS_SPACE_PCI_CONTROL_BAR_ID = 0,
	ADDRESS_SPACE_PCI_AREA_BAR_ID = 1,
};

struct address_space_driver_state;

struct address_space_device_state {
	u32	magic;

	struct list_head 			node;
	struct miscdevice 			miscdevice;
	struct pci_dev 				*dev;
	struct address_space_driver_state 	*driver_state;

	void __iomem 		*io_registers;
	void			*address_area;	/* to clain the address space */
	unsigned long		address_area_phys_address; /* physical address to allocate from */
	struct mutex 		registers_lock;	/* protects registers */
	wait_queue_head_t 	wake_queue;	/* to wait for the hardware */
	int			hw_done;	/* to say hw is done */
};

struct address_space_driver_state {
	struct list_head devices;	/* of struct address_space_device_state */
	struct mutex devices_lock;	/* protects devices */
	struct pci_driver pci;
};

struct address_space_block {
	u64 offset;
	u64 size;
};

struct address_space_allocated_blocks {
	struct address_space_device_state *state;

	/* a dynamic array of allocated blocks */
	struct address_space_block *blocks;
	int blocks_size;
	int blocks_capacity;
	struct mutex blocks_lock;	/* protects operations with blocks */
};

static void __iomem *address_space_register_address(void __iomem *base, int offset)
{
	return ((char __iomem *)base) + offset;
}

static void address_space_write_register(void __iomem *registers, int offset, u32 value)
{
	writel(value, address_space_register_address(registers, offset));
}

static u32 address_space_read_register(void __iomem *registers, int offset)
{
	return readl(address_space_register_address(registers, offset));
}

static int address_space_talk_to_hardware(struct address_space_device_state *state,
					  enum address_space_command_id cmd)
{
	state->hw_done = 0;

	address_space_write_register(state->io_registers,
				     ADDRESS_SPACE_REGISTER_COMMAND,
				     cmd);

	while (!state->hw_done) {
		if (wait_event_interruptible(state->wake_queue,
			state->hw_done))
			return -ERESTARTSYS;
	}

	return -address_space_read_register(state->io_registers,
					    ADDRESS_SPACE_REGISTER_STATUS);
}

static long address_space_ioctl_allocate_block_locked_impl(struct address_space_device_state *state,
							   u64 *size, u64 *offset)
{
	long res;

	address_space_write_register(state->io_registers,
				     ADDRESS_SPACE_REGISTER_BLOCK_SIZE_LOW,
				     lower_32_bits(*size));
	address_space_write_register(state->io_registers,
				     ADDRESS_SPACE_REGISTER_BLOCK_SIZE_HIGH,
				     upper_32_bits(*size));

	res = address_space_talk_to_hardware(state,
					     ADDRESS_SPACE_COMMAND_ALLOCATE_BLOCK);
	if (!res) {
		u64 low = address_space_read_register(state->io_registers,
						      ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_LOW);
		u64 high = address_space_read_register(state->io_registers,
						       ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_HIGH);
		*offset = low | (high << 32);

		low = address_space_read_register(state->io_registers,
						  ADDRESS_SPACE_REGISTER_BLOCK_SIZE_LOW);
		high = address_space_read_register(state->io_registers,
						   ADDRESS_SPACE_REGISTER_BLOCK_SIZE_HIGH);
		*size = low | (high << 32);
	}

	return res;
}

static long address_space_ioctl_unallocate_block_locked_impl(struct address_space_device_state *state,
							     u64 offset)
{
	address_space_write_register(state->io_registers,
				     ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_LOW,
				     lower_32_bits(offset));
	address_space_write_register(state->io_registers,
				     ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_HIGH,
				     upper_32_bits(offset));

	return address_space_talk_to_hardware(state,
					      ADDRESS_SPACE_COMMAND_DEALLOCATE_BLOCK);
}

static int address_space_blocks_grow_capacity(int old_capacity)
{
	BUG_ON(old_capacity < 0);

	return old_capacity + old_capacity;
}

static int address_space_blocks_insert(struct address_space_allocated_blocks *allocated_blocks,
				       u64 offset,
				       u64 size)
{
	int blocks_size;

	if (mutex_lock_interruptible(&allocated_blocks->blocks_lock))
		return -ERESTARTSYS;

	blocks_size = allocated_blocks->blocks_size;

	BUG_ON(allocated_blocks->blocks_capacity < 1);
	BUG_ON(allocated_blocks->blocks_capacity < allocated_blocks->blocks_size);
	BUG_ON(!allocated_blocks->blocks);

	if (allocated_blocks->blocks_capacity == blocks_size) {
		int new_capacity = address_space_blocks_grow_capacity(allocated_blocks->blocks_capacity);
		struct address_space_block *new_blocks =
			kcalloc(new_capacity,
				sizeof(allocated_blocks->blocks[0]),
				GFP_KERNEL);

		if (!new_blocks) {
			mutex_unlock(&allocated_blocks->blocks_lock);
			return -ENOMEM;
		}

		memcpy(new_blocks, allocated_blocks->blocks,
		       blocks_size * sizeof(allocated_blocks->blocks[0]));

		kfree(allocated_blocks->blocks);
		allocated_blocks->blocks = new_blocks;
		allocated_blocks->blocks_capacity = new_capacity;
	}

	BUG_ON(blocks_size >= allocated_blocks->blocks_capacity);

	allocated_blocks->blocks[blocks_size] =
		(struct address_space_block){ .offset = offset, .size = size };
	allocated_blocks->blocks_size = blocks_size + 1;

	mutex_unlock(&allocated_blocks->blocks_lock);
	return 0;
}

static int address_space_blocks_remove(struct address_space_allocated_blocks *allocated_blocks,
				       u64 offset)
{
	long res = -ENXIO;
	struct address_space_block *blocks;
	int blocks_size;
	int i;

	if (mutex_lock_interruptible(&allocated_blocks->blocks_lock))
		return -ERESTARTSYS;

	blocks = allocated_blocks->blocks;
	BUG_ON(!blocks);

	blocks_size = allocated_blocks->blocks_size;
	BUG_ON(blocks_size < 0);

	for (i = 0; i < blocks_size; ++i) {
		if (offset == blocks[i].offset) {
			int last = blocks_size - 1;
			if (last > i)
				blocks[i] = blocks[last];
			--allocated_blocks->blocks_size;
			res = 0;
			break;
		}
	}

	mutex_unlock(&allocated_blocks->blocks_lock);
	return res;
}

static int address_space_blocks_check_if_mine(struct address_space_allocated_blocks *allocated_blocks,
					      u64 offset,
					      u64 size)
{
	int res = -ENXIO;
	struct address_space_block *block;
	int blocks_size;

	if (mutex_lock_interruptible(&allocated_blocks->blocks_lock))
		return -ERESTARTSYS;

	block = allocated_blocks->blocks;
	BUG_ON(!block);

	blocks_size = allocated_blocks->blocks_size;
	BUG_ON(blocks_size < 0);

	for (; blocks_size > 0; --blocks_size, ++block) {
		if (block->offset == offset) {
			res = (block->size >= size) ? 0 : -EPERM;
			break;
		}
	}

	mutex_unlock(&allocated_blocks->blocks_lock);
	return res;
}

static int address_space_open(struct inode *inode, struct file *filp)
{
	struct address_space_allocated_blocks *allocated_blocks;

	allocated_blocks = kzalloc(sizeof(*allocated_blocks), GFP_KERNEL);
	if (!allocated_blocks)
		return -ENOMEM;

	allocated_blocks->state =
		container_of(filp->private_data,
			     struct address_space_device_state,
			     miscdevice);

	allocated_blocks->blocks =
		kcalloc(ADDRESS_SPACE_ALLOCATED_BLOCKS_INITIAL_CAPACITY,
			sizeof(allocated_blocks->blocks[0]),
			GFP_KERNEL);
	if (!allocated_blocks->blocks) {
		kfree(allocated_blocks);
		return -ENOMEM;
	}

	allocated_blocks->blocks_size = 0;
	allocated_blocks->blocks_capacity = ADDRESS_SPACE_ALLOCATED_BLOCKS_INITIAL_CAPACITY;
	mutex_init(&allocated_blocks->blocks_lock);

	filp->private_data = allocated_blocks;
	return 0;
}

static int address_space_release(struct inode *inode, struct file *filp)
{
	struct address_space_allocated_blocks *allocated_blocks = filp->private_data;
	struct address_space_device_state *state;
	int blocks_size;
	int i;

	BUG_ON(!allocated_blocks);
	BUG_ON(!allocated_blocks->state);
	BUG_ON(!allocated_blocks->blocks);
	BUG_ON(allocated_blocks->blocks_size < 0);

	state = allocated_blocks->state;
	blocks_size = allocated_blocks->blocks_size;

	if (mutex_lock_interruptible(&state->registers_lock))
		return -ERESTARTSYS;

	for (i = 0; i < blocks_size; ++i) {
		BUG_ON(address_space_ioctl_unallocate_block_locked_impl(state,
									allocated_blocks->blocks[i].offset));
	}

	mutex_unlock(&state->registers_lock);

	kfree(allocated_blocks->blocks);
	kfree(allocated_blocks);
	return 0;
}

static int address_space_mmap_impl(struct address_space_device_state *state,
				   size_t size,
				   struct vm_area_struct *vma)
{
	unsigned long pfn = (state->address_area_phys_address >> PAGE_SHIFT) +
		vma->vm_pgoff;

	return remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot);
}

static int address_space_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct address_space_allocated_blocks *allocated_blocks = filp->private_data;
	size_t size = PAGE_ALIGN(vma->vm_end - vma->vm_start);
	int res;

	BUG_ON(!allocated_blocks);

	res = address_space_blocks_check_if_mine(allocated_blocks,
						 vma->vm_pgoff << PAGE_SHIFT,
						 size);

	if (res)
		return res;
	else
		return address_space_mmap_impl(allocated_blocks->state, size, vma);
}

static long address_space_ioctl_allocate_block_impl(struct address_space_device_state *state,
						    struct goldfish_address_space_allocate_block *request)
{
	long res;

	if (mutex_lock_interruptible(&state->registers_lock))
		return -ERESTARTSYS;

	res = address_space_ioctl_allocate_block_locked_impl(state,
							     &request->size,
							     &request->offset);
	if (!res) {
		request->phys_addr =
			state->address_area_phys_address + request->offset;
	}

	mutex_unlock(&state->registers_lock);
	return res;
}

static void address_space_ioctl_unallocate_block_impl(struct address_space_device_state *state,
						      u64 offset)
{
	mutex_lock(&state->registers_lock);
	BUG_ON(address_space_ioctl_unallocate_block_locked_impl(state, offset));
	mutex_unlock(&state->registers_lock);
}

static long address_space_ioctl_allocate_block(struct address_space_allocated_blocks *allocated_blocks,
					       void __user *ptr)
{
	long res;
	struct address_space_device_state *state = allocated_blocks->state;
	struct goldfish_address_space_allocate_block request;

	if (copy_from_user(&request, ptr, sizeof(request)))
		return -EFAULT;

	res = address_space_ioctl_allocate_block_impl(state, &request);
	if (!res) {
		res = address_space_blocks_insert(allocated_blocks,
						  request.offset,
						  request.size);

		if (res) {
			address_space_ioctl_unallocate_block_impl(state, request.offset);
		} else if (copy_to_user(ptr, &request, sizeof(request))) {
			address_space_ioctl_unallocate_block_impl(state, request.offset);
			res = -EFAULT;
		}
	}

	return res;
}

static long address_space_ioctl_unallocate_block(struct address_space_allocated_blocks *allocated_blocks,
						 void __user *ptr)
{
	long res;
	u64 offset;

	if (copy_from_user(&offset, ptr, sizeof(offset)))
		return -EFAULT;

	res = address_space_blocks_remove(allocated_blocks, offset);
	if (!res)
		address_space_ioctl_unallocate_block_impl(allocated_blocks->state, offset);

	return res;
}

static long address_space_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct address_space_allocated_blocks *allocated_blocks = filp->private_data;

	switch (cmd) {
	case GOLDFISH_ADDRESS_SPACE_IOCTL_ALLOCATE_BLOCK:
		return address_space_ioctl_allocate_block(allocated_blocks, (void __user *)arg);

	case GOLDFISH_ADDRESS_SPACE_IOCTL_DEALLOCATE_BLOCK:
		return address_space_ioctl_unallocate_block(allocated_blocks, (void __user *)arg);

	default:
		return -ENOSYS;
	}
}

static const struct file_operations userspace_file_operations = {
	.owner = THIS_MODULE,
	.open = address_space_open,
	.release = address_space_release,
	.mmap = address_space_mmap,
	.unlocked_ioctl = address_space_ioctl,
	.compat_ioctl = address_space_ioctl,
};

static void __iomem __must_check *ioremap_pci_bar(struct pci_dev *dev, int bar_id)
{
	void __iomem *io;
	unsigned long size;
	unsigned long start = pci_resource_start(dev, bar_id);
	unsigned long end = pci_resource_end(dev, bar_id);

	if (end <= start)
		return IOMEM_ERR_PTR(-ENXIO);

	size = end - start;
	io = ioremap(start, size);
	if (!io)
		return IOMEM_ERR_PTR(-ENOMEM);

	return io;
}

static void __must_check *memremap_pci_bar(struct pci_dev *dev, int bar_id, unsigned long flags)
{
	void *mem;
	unsigned long size;
	unsigned long start = pci_resource_start(dev, bar_id);
	unsigned long end = pci_resource_end(dev, bar_id);

	if (end <= start)
		return ERR_PTR(-ENXIO);

	size = end - start;
	mem = memremap(start, size, flags);
	if (!mem)
		return ERR_PTR(-ENOMEM);

	return mem;
}

static void address_space_pci_remove_from_devices(struct address_space_device_state *state)
{
	struct list_head *i;

	mutex_lock(&state->driver_state->devices_lock);

	list_for_each(i, &state->driver_state->devices) {
		struct address_space_device_state *si =
			list_entry(i, struct address_space_device_state, node);
		if (state == si) {
			list_del(i);
			mutex_unlock(&state->driver_state->devices_lock);
			return;
		}
	}

	BUG();
}

static irqreturn_t __must_check address_space_interrupt_impl(struct address_space_device_state *state)
{
	state->hw_done = 1;
	wake_up_interruptible(&state->wake_queue);
	return IRQ_HANDLED;
}

static irqreturn_t address_space_interrupt(int irq, void *dev_id)
{
	struct address_space_device_state *state = dev_id;

	return (state->magic == ADDRESS_SPACE_MAGIC_U32)
		? address_space_interrupt_impl(state) : IRQ_NONE;
}

static void fill_miscdevice(struct miscdevice *miscdev)
{
	memset(miscdev, 0, sizeof(*miscdev));

	miscdev->minor = MISC_DYNAMIC_MINOR;
	miscdev->name = ADDRESS_SPACE_DEVICE_NAME;
	miscdev->fops = &userspace_file_operations;
}

static int __must_check create_address_space_device(struct pci_dev *dev,
						    const struct pci_device_id *id,
						    struct address_space_driver_state *driver_state)
{
	int res;
	struct address_space_device_state *state;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	res = pci_request_region(dev,
				 ADDRESS_SPACE_PCI_CONTROL_BAR_ID,
				 "Address space control");
	if (res) {
		pr_err("(bn 0x%X, sn 0x%X) failed to allocate PCI resource for BAR%d",
		       dev->bus->number,
		       dev->devfn,
		       ADDRESS_SPACE_PCI_CONTROL_BAR_ID);
		goto out_free_device_state;
	}

	res = pci_request_region(dev,
				 ADDRESS_SPACE_PCI_AREA_BAR_ID,
				 "Address space area");
	if (res) {
		pr_err("(bn 0x%X, sn 0x%X) failed to allocate PCI resource for BAR%d",
		       dev->bus->number,
		       dev->devfn,
		       ADDRESS_SPACE_PCI_AREA_BAR_ID);
		goto out_release_control_bar;
	}

	fill_miscdevice(&state->miscdevice);
	res = misc_register(&state->miscdevice);
	if (res)
		goto out_release_area_bar;

	state->io_registers = ioremap_pci_bar(dev,
					      ADDRESS_SPACE_PCI_CONTROL_BAR_ID);
	if (IS_ERR(state->io_registers)) {
		res = PTR_ERR(state->io_registers);
		goto out_misc_deregister;
	}

	state->address_area = memremap_pci_bar(dev,
					       ADDRESS_SPACE_PCI_AREA_BAR_ID,
					       MEMREMAP_WB);
	if (IS_ERR(state->address_area)) {
		res = PTR_ERR(state->address_area);
		goto out_iounmap;
	}

	state->address_area_phys_address =
		pci_resource_start(dev, ADDRESS_SPACE_PCI_AREA_BAR_ID);

	res = request_irq(dev->irq,
			  address_space_interrupt, IRQF_SHARED,
			  KBUILD_MODNAME, state);
	if (res) {
		goto out_memunmap;
	}

	address_space_write_register(state->io_registers,
				     ADDRESS_SPACE_REGISTER_GUEST_PAGE_SIZE,
				     PAGE_SIZE);

	state->magic = ADDRESS_SPACE_MAGIC_U32;
	state->dev = dev;
	state->driver_state = driver_state;
	mutex_init(&state->registers_lock);
	init_waitqueue_head(&state->wake_queue);

	list_add(&state->node, &driver_state->devices);

	pci_set_drvdata(dev, state);
	return 0;

out_memunmap:
	memunmap(state->address_area);
out_iounmap:
	iounmap(state->io_registers);
out_misc_deregister:
	misc_deregister(&state->miscdevice);
out_release_area_bar:
	pci_release_region(dev, ADDRESS_SPACE_PCI_AREA_BAR_ID);
out_release_control_bar:
	pci_release_region(dev, ADDRESS_SPACE_PCI_CONTROL_BAR_ID);
out_free_device_state:
	kzfree(state);

	return res;
}

static void destroy_address_space_device(struct address_space_device_state *state)
{
	address_space_pci_remove_from_devices(state);
	free_irq(state->dev->irq, state);
	memunmap(state->address_area);
	iounmap(state->io_registers);
	misc_deregister(&state->miscdevice);
	pci_release_region(state->dev, ADDRESS_SPACE_PCI_AREA_BAR_ID);
	pci_release_region(state->dev, ADDRESS_SPACE_PCI_CONTROL_BAR_ID);
	kfree(state);
}

static int __must_check
address_space_pci_probe_impl(struct pci_dev *dev,
			     const struct pci_device_id *id,
			     struct address_space_driver_state *driver_state)
{
	int res;
	u8 hardware_revision;

	res = pci_enable_device(dev);
	if (res)
		return res;

	res = pci_read_config_byte(dev, PCI_REVISION_ID, &hardware_revision);
	if (res)
		goto out_disable_pci;

	switch (hardware_revision) {
	case 1:
		res = create_address_space_device(dev, id, driver_state);
		break;

	default:
		res = -ENODEV;
		goto out_disable_pci;
	}

	return 0;

out_disable_pci:
	pci_disable_device(dev);

	return res;
}

static void address_space_pci_remove(struct pci_dev *dev)
{
	struct address_space_device_state *state = pci_get_drvdata(dev);

	destroy_address_space_device(state);
	pci_disable_device(dev);
}

static const struct pci_device_id address_space_pci_tbl[] = {
	{ PCI_DEVICE(ADDRESS_SPACE_PCI_VENDOR_ID,
		     ADDRESS_SPACE_PCI_DEVICE_ID), },
	{ }
};
MODULE_DEVICE_TABLE(pci, address_space_pci_tbl);

static int address_space_pci_probe(struct pci_dev *dev,
				   const struct pci_device_id *id);

static void __init fill_pci_driver(struct pci_driver *pci)
{
	pci->name = ADDRESS_SPACE_DEVICE_NAME;
	pci->id_table = address_space_pci_tbl;
	pci->probe = &address_space_pci_probe;
	pci->remove = &address_space_pci_remove;
	pci->shutdown = &address_space_pci_remove;
}

static int __must_check __init init_address_space_impl(struct address_space_driver_state *state)
{
	INIT_LIST_HEAD(&state->devices);
	mutex_init(&state->devices_lock);
	fill_pci_driver(&state->pci);

	return pci_register_driver(&state->pci);
}

static void __exit exit_address_space_impl(struct address_space_driver_state *state)
{
	BUG_ON(!list_empty(&state->devices));

	pci_unregister_driver(&state->pci);
}

static struct address_space_driver_state g_driver_state;

static int address_space_pci_probe(struct pci_dev *dev,
				   const struct pci_device_id *id)
{
	return address_space_pci_probe_impl(dev, id, &g_driver_state);
}

static int __init init_address_space(void)
{
	memset(&g_driver_state, 0, sizeof(g_driver_state));
	return init_address_space_impl(&g_driver_state);
}

static void __exit exit_address_space(void)
{
	exit_address_space_impl(&g_driver_state);
	memset(&g_driver_state, 0xCD, sizeof(g_driver_state));  // make it crash if we forgot something
}

module_init(init_address_space);
module_exit(exit_address_space);
