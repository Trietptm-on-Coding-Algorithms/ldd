#include <linux/poll.h>
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

#define GLOBALFIFO_SIZE 0x1000
#define MEM_CLEAR 0x1
#define GLOBALFIFO_MAJOR 224
#define NOD_CNT 3

#define PRINT_PRIORITY KERN_INFO

#define DEBUG
#define SEMA

#ifdef DEBUG
	#define dbg_printk(fmt, args...) printk(PRINT_PRIORITY "<%s: %3d> "fmt, __FUNCTION__, __LINE__, ##args)
#else
	#define dbg_printk(fmt, args...)
#endif

static int globalfifo_major = GLOBALFIFO_MAJOR;

struct globalfifo_dev {
	struct cdev cdev;
	unsigned int current_len;
	unsigned char mem[GLOBALFIFO_SIZE];
	struct semaphore sem;
	wait_queue_head_t r_wait;
	wait_queue_head_t w_wait;
};

struct globalfifo_dev *globalfifo_devp;

int globalfifo_open(struct inode *inode, struct file *filp)
{
	struct globalfifo_dev *dev;
	dev = container_of(inode->i_cdev, struct globalfifo_dev, cdev);
	filp->private_data = dev;
	return 0;
}

int globalfifo_release(struct inode *globalfifo_devp, struct file *filp)
{
	return 0;
}

static ssize_t globalfifo_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	int ret;
	struct globalfifo_dev *dev = filp->private_data;
	DECLARE_WAITQUEUE(wait, current);

	dbg_printk("%u, %lu\n", count, (unsigned long)ppos);
	
	down(&dev->sem);
	add_wait_queue(&dev->r_wait, &wait);

	while (dev->current_len == 0)
	{
		//noblock start
		if (filp->f_flags & O_NONBLOCK)
		{
			ret = -EAGAIN;
			goto out1;
		}
		//noblock end

		//block start
		__set_current_state(TASK_INTERRUPTIBLE);
		up(&dev->sem);

		schedule();

		if (signal_pending(current))
		{
			ret = - ERESTARTSYS;
			goto out2;
		}

		down(&dev->sem);
		//block end
	}

	if (count > dev->current_len)
		count = dev->current_len;

	if (copy_to_user(buf, dev->mem, count))
	{
		ret = - EFAULT;
		goto out1;
	}
	else
	{
		memcpy(dev->mem, dev->mem + count, dev->current_len - count);
		dev->current_len -= count;

		dbg_printk("read %d bytes(s) from %d\n", count, dev->current_len);
		wake_up_interruptible(&dev->w_wait);	//wake up write queue
		ret = count;
	}

out1:
	up(&dev->sem);
out2:
	remove_wait_queue(&dev->w_wait, &wait);
	set_current_state(TASK_RUNNING);
	return ret;
}

static ssize_t globalfifo_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	struct globalfifo_dev *dev = filp->private_data;
	int ret;
	DECLARE_WAITQUEUE(wait, current);

	dbg_printk("%u, %lu\n", count, (unsigned long)ppos);

	down(&dev->sem);
	add_wait_queue(&dev->w_wait, &wait);

	//wait fifo not empty
	while (dev->current_len == GLOBALFIFO_SIZE)
	{
		//return if use nonblock mode
		if (filp->f_flags & O_NONBLOCK);
		{
			ret = - EAGAIN;
			goto out1;
		}

		//sleep if use block mode
		__set_current_state(TASK_INTERRUPTIBLE);
		up(&dev->sem);
		schedule();
		if (signal_pending(current))
		{
			ret = - ERESTARTSYS;
			goto out2;
		}
		down(&dev->sem);
	}

	if (count > GLOBALFIFO_SIZE - dev->current_len)
		count = GLOBALFIFO_SIZE - dev->current_len;

	if (copy_from_user(dev->mem + dev->current_len, buf, count))
	{
		ret = - EFAULT;
		goto out1;
	}
	else
	{
		dev->current_len += count;
		dbg_printk("write %d bytes(s) from %d\n", count, dev->current_len);
		wake_up_interruptible(&dev->r_wait);
		ret = count;
	}

out1:
	up(&dev->sem);
out2:
	remove_wait_queue(&dev->w_wait, &wait);
	set_current_state(TASK_RUNNING);
	return ret;
}

static long globalfifo_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct globalfifo_dev *dev = filp->private_data;

	switch (cmd)
	{
		case MEM_CLEAR:
			if (down_interruptible(&dev->sem))
			return - ERESTARTSYS;

			memset(dev->mem, 0, GLOBALFIFO_SIZE);

			up(&dev->sem);
			break;
		default:
			return - EINVAL;
	}
	return 0;
}

static loff_t globalfifo_llseek(struct file *filp, loff_t offset, int orig)
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
			
			if ((unsigned int)offset > GLOBALFIFO_SIZE)
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
			
			if ((filp->f_pos + offset) > GLOBALFIFO_SIZE)
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

static unsigned int globalfifo_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct globalfifo_dev *dev = filp->private_data;

	down(&dev->sem);

	poll_wait(filp, &dev->r_wait, wait);
	poll_wait(filp, &dev->w_wait, wait);

	if (dev->current_len != 0)
		mask |= POLLIN | POLLRDNORM;

	if (dev->current_len != GLOBALFIFO_SIZE)
		mask |= POLLOUT | POLLWRNORM;

	up(&dev->sem);
	return mask;
}

static const struct file_operations globalfifo_fops = {
	.owner = THIS_MODULE,
	.llseek = globalfifo_llseek,
	.read = globalfifo_read,
	.write = globalfifo_write,
	.poll = globalfifo_poll,
	.unlocked_ioctl = globalfifo_ioctl,
	.open = globalfifo_open,
	.release = globalfifo_release,
};

static void globalfifo_setup_cdev(struct globalfifo_dev *dev, int index)
{
	int err, devno;
	devno = MKDEV(globalfifo_major, index);

	cdev_init(&dev->cdev, &globalfifo_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);

	if (err)
		dbg_printk("Error %d adding globalfifo %d", err, index);
}

int globalfifo_init(void)
{
	int i, result;
	dev_t devno = MKDEV(globalfifo_major, 0);

	if (globalfifo_major)
		result = register_chrdev_region(devno, NOD_CNT, "globalfifo");
	else
	{
		result = alloc_chrdev_region(&devno, 0, NOD_CNT, "globalfifo");
		globalfifo_major = MAJOR(devno);
	}
	if (result < 0)
		return result;
	
	globalfifo_devp = kmalloc(NOD_CNT * sizeof(struct globalfifo_dev), GFP_KERNEL);
	if (!globalfifo_devp)
	{
		result = - ENOMEM;
		goto fail_malloc;
	}

	memset(globalfifo_devp, 0, NOD_CNT * sizeof(struct globalfifo_dev));

	for (i = 0; i < NOD_CNT; i++)
	{
		globalfifo_setup_cdev(&globalfifo_devp[i], i);
		sema_init(&globalfifo_devp[i].sem, 1);
		init_waitqueue_head(&globalfifo_devp[i].r_wait);
		init_waitqueue_head(&globalfifo_devp[i].w_wait);
	}

	dbg_printk("globalfifo init\n");

	return 0;

fail_malloc:
	unregister_chrdev_region(devno, NOD_CNT);
	return result;
}

void globalfifo_exit(void)
{
	cdev_del(&globalfifo_devp->cdev);
	kfree(globalfifo_devp);
	unregister_chrdev_region(MKDEV(globalfifo_major, 0), NOD_CNT);

	dbg_printk("globalfifo exit\n");
}

MODULE_AUTHOR("WANG Mohai <asicer@live.cn>");
MODULE_LICENSE("Dual BSD/GPL");

module_init(globalfifo_init);
module_exit(globalfifo_exit);
