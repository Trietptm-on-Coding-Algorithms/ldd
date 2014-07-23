#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#define GLOBALMEM_SIZE 0x1000
#define MEM_CLEAR 0x1
#define GLOBALMEM_MAJOR 225
#define NOD_CNT 3

#define PRINT_PRIORITY KERN_INFO

#define DEBUG

#ifdef DEBUG
	#define dbg_printk(fmt, args...) printk(PRINT_PRIORITY "<%s: %3d> "fmt, __FUNCTION__, __LINE__, ##args)
#else
	#define dbg_printk(fmt, args...)
#endif

static int globalmem_major = GLOBALMEM_MAJOR;

struct globalmem_dev {
	struct cdev cdev;
	unsigned char mem[GLOBALMEM_SIZE];
	struct semaphore sem;
};

struct globalmem_dev *globalmem_devp;

int globalmem_open(struct inode *inode, struct file *filp)
{
	struct globalmem_dev *dev;
	dev = container_of(inode->i_cdev, struct globalmem_dev, cdev);
	filp->private_data = dev;
	return 0;
}

int globalmem_release(struct inode *globalmem_devp, struct file *filp)
{
	return 0;
}

static ssize_t globalmem_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	int ret = 0;
	struct globalmem_dev *dev = filp->private_data;

	dbg_printk("count: %u, p: %lu\n", count, p);

	if (p >= GLOBALMEM_SIZE)
		return 0;
	if (count > GLOBALMEM_SIZE - p)
		count = GLOBALMEM_SIZE - p;
	
	if (down_interruptible(&dev->sem))
		return - ERESTARTSYS;

	dbg_printk("count: %u, p: %lu\n", count, p);

	if (copy_to_user(buf, (void *)(dev->mem + p), count))
		ret = - EFAULT;
	else
	{
		*ppos += count;
		ret = count;
		dbg_printk("read %u bytes(s) from %lu\n", count, p);
	}

	up(&dev->sem);

	return ret;
}

static ssize_t globalmem_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	int ret = 0;
	struct globalmem_dev *dev = filp->private_data;

	dbg_printk("%u, %lu\n", count, p);

	if (down_interruptible(&dev->sem))
		return - ERESTARTSYS;

	if (p > GLOBALMEM_SIZE)
		return 0;
	if (count > GLOBALMEM_SIZE - p)
		count = GLOBALMEM_SIZE - p;

	if (copy_from_user(dev->mem + p, buf, count))
		ret = - EFAULT;
	else
	{
		*ppos += count;
		ret = count;
		dbg_printk("write %u bytes(s) from %lu\n", count, p);
	}
	
	up(&dev->sem);
	
	return ret;
}

static long globalmem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct globalmem_dev *dev = filp->private_data;

	switch (cmd)
	{
		case MEM_CLEAR:
			if (down_interruptible(&dev->sem))
				return - ERESTARTSYS;
			memset(dev->mem, 0, GLOBALMEM_SIZE);
			up(&dev->sem);
			break;
		default:
			return - EINVAL;
	}
	return 0;
}

static loff_t globalmem_llseek(struct file *filp, loff_t offset, int orig)
{
	loff_t ret;
	switch (orig)
	{
		case 0:	//offset relative to file start position
			if (offset < 0)
			{
				ret = - EINVAL;
				break;
			}
			if ((unsigned int)offset > GLOBALMEM_SIZE)
			{
				ret = - EINVAL;
				break;
			}
			filp->f_pos += offset;
			ret = filp->f_pos;
			break;
		case 1:	//offset relative to file current position
			if ((filp->f_pos + offset) < 0)
			{
				ret = - EINVAL;
				break;
			}
			if ((filp->f_pos + offset) > GLOBALMEM_SIZE)
			{
				ret = - EINVAL;
				break;
			}
			filp->f_pos += offset;
			ret = filp->f_pos;
			break;
		default:
			ret = - EINVAL;
	}
	return ret;
}

static const struct file_operations globalmem_fops = {
	.owner = THIS_MODULE,
	.llseek = globalmem_llseek,
	.read = globalmem_read,
	.write = globalmem_write,
	.unlocked_ioctl = globalmem_ioctl,
	.open = globalmem_open,
	.release = globalmem_release,
};

static void globalmem_setup_cdev(struct globalmem_dev *dev, int index)
{
	int err, devno;
	devno = MKDEV(globalmem_major, index);

	cdev_init(&dev->cdev, &globalmem_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);

	if (err)
		dbg_printk("Error %d adding globalmem %d", err, index);
}

int globalmem_init(void)
{
	int i, result;
	dev_t devno = MKDEV(globalmem_major, 0);

	if (globalmem_major)
		result = register_chrdev_region(devno, NOD_CNT, "globalmem");
	else
	{
		result = alloc_chrdev_region(&devno, 0, NOD_CNT, "globalmem");
		globalmem_major = MAJOR(devno);
	}
	if (result < 0)
		return result;
	
	globalmem_devp = kmalloc(NOD_CNT * sizeof(struct globalmem_dev), GFP_KERNEL);
	if (!globalmem_devp)
	{
		result = - ENOMEM;
		goto fail_malloc;
	}

	memset(globalmem_devp, 0, NOD_CNT * sizeof(struct globalmem_dev));

	for (i = 0; i < NOD_CNT; i++)
	{
		globalmem_setup_cdev(&globalmem_devp[i], i);
		sema_init(&globalmem_devp[i].sem, 1);
	}

	dbg_printk("globalmem init\n");

	return 0;

fail_malloc:
	unregister_chrdev_region(devno, 3);
	return result;
}

void globalmem_exit(void)
{
	int i;
	for (i = 0; i < NOD_CNT; i++)
		cdev_del(&(globalmem_devp[i].cdev));
	kfree(globalmem_devp);
	unregister_chrdev_region(MKDEV(globalmem_major, 0), NOD_CNT);
	dbg_printk("globalmem exit\n");
}

MODULE_AUTHOR("WANG Mohai <asicer@live.cn>");
MODULE_LICENSE("Dual BSD/GPL");

module_init(globalmem_init);
module_exit(globalmem_exit);
