// SPDX-License-Identifier: GPL-2.0

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/debugfs.h>

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
	ADDRESS_SPACE_REGISTER_BLOCK_SIZE_LOW = 8,
	ADDRESS_SPACE_REGISTER_BLOCK_SIZE_HIGH = 12,
	ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_LOW = 16,
	ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_HIGH = 20,
};

enum address_space_command_id {
	ADDRESS_SPACE_COMMAND_ALLOCATE_BLOCK = 1,
	ADDRESS_SPACE_COMMAND_DEALLOCATE_BLOCK = 2,
};

#define ADDRESS_SPACE_PCI_DEVICE_NAME	"goldfish_address_space"
#define ADDRESS_SPACE_PCI_VENDOR_ID	0x607D
#define ADDRESS_SPACE_PCI_DEVICE_ID	0xF153
#define ADDRESS_SPACE_MAGIC_U32		(ADDRESS_SPACE_PCI_VENDOR_ID << 16 | ADDRESS_SPACE_PCI_DEVICE_ID)
#define ADDRESS_SPACE_GOLDFISH_DIR	"goldfish"
#define ADDRESS_SPACE_USERSPACE_ROOT	"address_space"

enum address_space_pci_bar_id {
	ADDRESS_SPACE_PCI_CONTROL_BAR_ID = 0,
	ADDRESS_SPACE_PCI_AREA_BAR_ID = 1,
};

struct address_space_driver_state;

struct address_space_device_state {
	u32 	magic;

	struct list_head 			node;
	struct dentry 				*userspace_file;
	struct pci_dev 				*dev;
	struct address_space_driver_state 	*driver_state;

	void __iomem 		*io_registers;
	void			*address_area;	/* the addresses to allocate from */
	struct mutex 		registers_lock;	/* protects registers */
	wait_queue_head_t 	wake_queue;	/* to wait for the hardware */
	int			hw_done;	/* to say hw is done */
};

struct address_space_driver_state {
	struct dentry *goldfish_dir;
	struct dentry *userspace_root;
	struct list_head devices;	/* of struct address_space_device_state */
	struct mutex devices_lock;	/* protects devices */
	struct pci_driver pci;
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

static int address_space_open(struct inode *inode, struct file *filp)
{
	struct address_space_device_state *state = inode->i_private;

	filp->private_data = state;
	return 0;
}

static int address_space_release(struct inode *inode, struct file *filp)
{
	return 0;	/* nothing here */
}

static int address_space_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct address_space_device_state *state = filp->private_data;
	size_t sz = PAGE_ALIGN(vma->vm_end - vma->vm_start);
	unsigned long pfn = (virt_to_phys(state->address_area) >> PAGE_SHIFT) +
		vma->vm_pgoff;

	return remap_pfn_range(vma, vma->vm_start, pfn, sz, vma->vm_page_prot);
}

static long address_space_ioctl_allocate_block_locked(struct address_space_device_state *state,
						      unsigned long size)
{
	int res;

	address_space_write_register(state->io_registers,
				     ADDRESS_SPACE_REGISTER_BLOCK_SIZE_LOW,
				     lower_32_bits(size));
	address_space_write_register(state->io_registers,
				     ADDRESS_SPACE_REGISTER_BLOCK_SIZE_HIGH,
				     upper_32_bits(size));

	res = address_space_talk_to_hardware(state,
					     ADDRESS_SPACE_COMMAND_ALLOCATE_BLOCK);
	if (res) {
		return res;
	} else {
		long offset_low = address_space_read_register(state->io_registers,
							      ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_LOW);
		long offset_high = address_space_read_register(state->io_registers,
							       ADDRESS_SPACE_REGISTER_BLOCK_OFFSET_HIGH);

		return (offset_high << 32) | offset_low;
	}
}

static long address_space_ioctl_allocate_block(struct address_space_device_state *state,
					       unsigned long size)
{
	int res;

	if (mutex_lock_interruptible(&state->registers_lock))
		return -ERESTARTSYS;

	res = address_space_ioctl_allocate_block_locked(state, size);

	mutex_unlock(&state->registers_lock);
	return res;
}

static long address_space_ioctl_unallocate_block_locked(struct address_space_device_state *state,
							unsigned long offset)
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

static long address_space_ioctl_unallocate_block(struct address_space_device_state *state,
						 unsigned long offset)
{
	int res;

	if (mutex_lock_interruptible(&state->registers_lock))
		return -ERESTARTSYS;

	res = address_space_ioctl_unallocate_block_locked(state, offset);

	mutex_unlock(&state->registers_lock);
	return res;
}

static long address_space_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct address_space_device_state *state = filp->private_data;

	switch (cmd) {
	case GOLDFISH_ADDRESS_SPACE_IOCTL_ALLOCATE_BLOCK:
		return address_space_ioctl_allocate_block(state, arg);

	case GOLDFISH_ADDRESS_SPACE_IOCTL_DEALLOCATE_BLOCK:
		return address_space_ioctl_unallocate_block(state, arg);

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
		return ERR_PTR(-ENXIO);

	size = end - start;
	io = ioremap(start, size);
	if (!io)
		return ERR_PTR(-ENOMEM);

	return io;
}

static void __must_check *memremap_pci_bar(struct pci_dev *dev, int bar_id, unsigned long flags)
{
	void __iomem *mem;
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

static int __must_check create_address_space_device(struct pci_dev *dev,
						    const struct pci_device_id *id,
						    struct address_space_driver_state *driver_state)
{
	int res;
	char device_filename[16];
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

	snprintf(device_filename, sizeof(device_filename), "%X", id->device);

	state->userspace_file =
		debugfs_create_file(device_filename,
				    S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH,
				    driver_state->userspace_root,
				    state,
				    &userspace_file_operations);
	if (!state->userspace_file) {
		res = -ENOENT;
		goto out_release_area_bar;
	}

	state->io_registers = ioremap_pci_bar(dev,
					      ADDRESS_SPACE_PCI_CONTROL_BAR_ID);
	if (IS_ERR(state->io_registers)) {
		res = PTR_ERR(state->io_registers);
		goto out_debugfs_remove;
	}

	state->address_area = memremap_pci_bar(dev,
					       ADDRESS_SPACE_PCI_AREA_BAR_ID,
					       MEMREMAP_WB);
	if (IS_ERR(state->address_area)) {
		res = PTR_ERR(state->address_area);
		goto out_iounmap;
	}

	res = request_irq(dev->irq,
			  address_space_interrupt, IRQF_SHARED,
			  KBUILD_MODNAME, state);
	if (res) {
		goto out_memunmap;
	}

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
out_debugfs_remove:
	debugfs_remove(state->userspace_file);
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
	debugfs_remove(state->userspace_file);
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

void __init fill_pci_driver(struct pci_driver *pci)
{
	pci->name = ADDRESS_SPACE_PCI_DEVICE_NAME;
	pci->id_table = address_space_pci_tbl;
	pci->probe = &address_space_pci_probe;
	pci->remove = &address_space_pci_remove;
	pci->shutdown = &address_space_pci_remove;
}

static int __must_check __init init_address_space_impl(struct address_space_driver_state *state)
{
	state->goldfish_dir = debugfs_create_dir(ADDRESS_SPACE_GOLDFISH_DIR,
						 NULL);
	if (!state->goldfish_dir)
		return -ENOENT;

	state->userspace_root = debugfs_create_dir(ADDRESS_SPACE_USERSPACE_ROOT,
						   state->goldfish_dir);
	if (!state->userspace_root) {
		debugfs_remove(state->goldfish_dir);
		return -ENOENT;
	}

	INIT_LIST_HEAD(&state->devices);
	mutex_init(&state->devices_lock);
	fill_pci_driver(&state->pci);

	return pci_register_driver(&state->pci);
}

static void __exit exit_address_space_impl(struct address_space_driver_state *state)
{
	BUG_ON(!list_empty(&state->devices));

	pci_unregister_driver(&state->pci);
	debugfs_remove(state->userspace_root);
	debugfs_remove(state->goldfish_dir);
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
