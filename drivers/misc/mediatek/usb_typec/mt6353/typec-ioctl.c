#include <linux/version.h>
#include "typec.h"
#include "typec-ioctl.h"

static int cdev_open(struct inode *inode, struct file *file)
{
	struct typec_hba *hba;

	hba = container_of(inode->i_cdev, struct typec_hba, cdev);
	file->private_data = hba;

	return 0;
}

static int cdev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long __cdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct typec_hba *hba = file->private_data;
	void __user *argp = (void __user *)arg;

	char wr_buf[CLI_BUF_SIZE];
	char rd_buf[CLI_BUF_SIZE] = "this is a test";

	//CC: pass arguments from user space to kernel space
	switch (cmd) {
		case IOCTL_READ:
			copy_to_user((char *) arg, rd_buf, CLI_BUF_SIZE);
			printk(KERN_DEBUG "IOCTL_READ: %s\r\n", rd_buf);
			break;
		case IOCTL_WRITE:
			copy_from_user(wr_buf, (char *) arg, CLI_BUF_SIZE);
			printk(KERN_DEBUG "IOCTL_WRITE: %s\r\n", wr_buf);

			//invoke function
			return call_function(file, wr_buf);
			break;
		default:
			return -ENOTTY;
	}

	return -ENOIOCTLCMD;
}

static long cdev_ioctl(struct file *fl, unsigned int cmd, unsigned long arg)
{
	struct typec_hba *hba = fl->private_data;
	long ret;
	int retry;
	unsigned long flags;


	if (ret = mutex_lock_interruptible(&hba->ioctl_lock))
		return ret;

	ret = __cdev_ioctl(fl, cmd, arg);

	mutex_unlock(&hba->ioctl_lock);


	return ret;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = cdev_open,
	.release = cdev_release,
	.unlocked_ioctl = cdev_ioctl,
};

#define TYPEC_MAX_ADAPTERS 1024

static unsigned int typec_cdev_major;
static DECLARE_BITMAP(typec_cdev_minor, TYPEC_MAX_ADAPTERS);

static int typec_cdev_get_minor(void)
{
	int minor;

	minor = find_first_zero_bit(typec_cdev_minor, TYPEC_MAX_ADAPTERS);
	if (minor < TYPEC_MAX_ADAPTERS)
		__set_bit(minor, typec_cdev_minor);

	return minor;
}

static void typec_cdev_release_minor(int minor)
{
	__clear_bit(minor, typec_cdev_minor);
}

static struct class *typec_cdev_class;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)

static struct attribute *typec_cdev_attrs[] = {
	NULL,
};

static const struct attribute_group typec_cdev_group = {
	.attrs = typec_cdev_attrs,
};

static const struct attribute_group *typec_cdev_groups[] = {
	&typec_cdev_group,
	NULL,
};

#else

static struct device_attribute typec_cdev_attrs[] = {
	__ATTR_NULL
};

#endif

int typec_cdev_init(struct device *parent, struct typec_hba *hba, int id)
{
	int err;
	struct device *device;
	int minor;

	//minor = typec_cdev_get_minor();
	minor = id;
	if (minor >= TYPEC_MAX_ADAPTERS)
		return -EBUSY;

	cdev_init(&hba->cdev, &fops);
	hba->cdev.owner = THIS_MODULE;

	err = cdev_add(&hba->cdev, MKDEV(typec_cdev_major, minor), 1);
	if (err < 0) {
		dev_err(parent, "cdev_add() error.\n");
		goto out_release_minor;
	}
	device = device_create(typec_cdev_class, parent,
			MKDEV(typec_cdev_major, minor), hba, "typec%u", minor);
	if (IS_ERR(device)) {
		dev_err(parent, "device_create filed\n");
		err = PTR_ERR(device);
		goto out_cdev_del;
	}

	return 0;

out_cdev_del:
	cdev_del(&hba->cdev);
out_release_minor:
	typec_cdev_release_minor(minor);

	return err;
}

void typec_cdev_remove(struct typec_hba *hba)
{
	int minor = MINOR(hba->cdev.dev);

	device_destroy(typec_cdev_class, MKDEV(typec_cdev_major, minor));
	cdev_del(&hba->cdev);
	typec_cdev_release_minor(minor);
}

int typec_cdev_module_init(void)
{
	int error;
	dev_t dev;

	error = alloc_chrdev_region(&dev, 0, TYPEC_MAX_ADAPTERS, "typec");
	if (error) {
		pr_err("failed to get a major number for adapters\n");
		return error;
	}
	typec_cdev_major = MAJOR(dev);

	typec_cdev_class = class_create(THIS_MODULE, "typec");
	if (IS_ERR(typec_cdev_class)) {
		error = PTR_ERR(typec_cdev_class);
		pr_err("failed to register with sysfs\n");
		goto out_unreg_chrdev;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
	typec_cdev_class->dev_groups = typec_cdev_groups;
#else
	typec_cdev_class->dev_attrs = typec_cdev_attrs;
#endif

	return 0;

out_unreg_chrdev:
	unregister_chrdev_region(MKDEV(typec_cdev_major, 0),
				TYPEC_MAX_ADAPTERS);

	return error;
}

void typec_cdev_module_exit(void)
{
	class_destroy(typec_cdev_class);
	unregister_chrdev_region(MKDEV(typec_cdev_major, 0),
				TYPEC_MAX_ADAPTERS);
}

