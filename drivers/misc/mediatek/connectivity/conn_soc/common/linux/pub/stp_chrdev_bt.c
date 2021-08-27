/*
* Copyright (C) 2011-2014 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <asm/current.h>
#include <asm/uaccess.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/delay.h>
#if WMT_CREATE_NODE_DYNAMIC
#include <linux/device.h>
#endif
#include <linux/printk.h>

#include "stp_exp.h"
#include "wmt_exp.h"

MODULE_LICENSE("Dual BSD/GPL");


#define PFX                         "[MTK-BT] "
#define BT_LOG_DBG                  3
#define BT_LOG_INFO                 2
#define BT_LOG_WARN                 1
#define BT_LOG_ERR                  0

static unsigned int gDbgLevel = BT_LOG_WARN;

#define BT_DBG_FUNC(fmt, arg...)	\
	do { if (gDbgLevel >= BT_LOG_DBG)	\
		pr_warn(PFX "%s: "  fmt, __func__ , ##arg);	\
	} while (0)
#define BT_INFO_FUNC(fmt, arg...)	\
    do { if (gDbgLevel >= BT_LOG_INFO)	\
		pr_warn(PFX "%s: "  fmt, __func__ , ##arg);	\
	} while (0)
#define BT_WARN_FUNC(fmt, arg...)	\
	do { if (gDbgLevel >= BT_LOG_WARN)	\
		pr_err(PFX "%s: "  fmt, __func__ , ##arg);	\
	} while (0)
#define BT_ERR_FUNC(fmt, arg...)	\
	do { if (gDbgLevel >= BT_LOG_ERR)	\
		pr_err(PFX "%s: "   fmt, __func__ , ##arg);	\
	} while (0)
#define BT_TRC_FUNC(f)	\
	do { if (gDbgLevel >= BT_LOG_DBG)	\
		pr_info(PFX "<%s> <%d>\n", __func__, __LINE__);	\
	} while (0)

#define VERSION "1.0"
#define BT_DRIVER_NAME "mtk_stp_BT_chrdev"
#define BT_DEV_MAJOR 192	/* never used number */

#if 0
#define COMBO_IOC_MAGIC        0xb0
#define COMBO_IOCTL_FW_ASSERT  _IOWR(COMBO_IOC_MAGIC, 0, void*)
#define COMBO_IOCTL_BT_IC_HW_VER  _IOWR(COMBO_IOC_MAGIC, 1, int)
#define COMBO_IOCTL_BT_IC_FW_VER  _IOWR(COMBO_IOC_MAGIC, 2, int)
#define COMBO_IOC_BT_HWVER           6
#else
#define COMBO_IOCTL_FW_ASSERT        2
#define COMBO_IOCTL_BT_IC_HW_VER     3
#define COMBO_IOCTL_BT_IC_FW_VER     4
#define COMBO_IOC_BT_HWVER           5
#define COMBO_IOC_BT_SET_PSM         6
#endif

#if WMT_CREATE_NODE_DYNAMIC
struct class *bt_class = NULL;
struct device *bt_dev = NULL;
#endif

static int BT_devs = 1;		/* device count */
static int BT_major = BT_DEV_MAJOR;	/* dynamic allocation */
module_param(BT_major, uint, 0);
static struct cdev BT_cdev;

#define BT_BUFFER_SIZE 2048
static unsigned char i_buf[BT_BUFFER_SIZE];	/* input buffer of read() */
static unsigned char o_buf[BT_BUFFER_SIZE];	/* output buffer of write() */

static struct semaphore wr_mtx, rd_mtx;
static wait_queue_head_t inq;	/* read queues */
static DECLARE_WAIT_QUEUE_HEAD(BT_wq);
static int flag;
static volatile int retflag;

static void bt_cdev_rst_cb(ENUM_WMTDRV_TYPE_T src,
			   ENUM_WMTDRV_TYPE_T dst, ENUM_WMTMSG_TYPE_T type, void *buf, unsigned int sz)
{

	/*
	   To handle reset procedure please
	 */
	ENUM_WMTRSTMSG_TYPE_T rst_msg;

	BT_WARN_FUNC("sizeof(ENUM_WMTRSTMSG_TYPE_T) = %zd\n", sizeof(ENUM_WMTRSTMSG_TYPE_T));
	if (sz <= sizeof(ENUM_WMTRSTMSG_TYPE_T)) {
		memcpy((char *)&rst_msg, (char *)buf, sz);
		BT_WARN_FUNC("src = %d, dst = %d, type = %d, buf = 0x%x sz = %d, max = %d\n", src, dst, type, rst_msg,
			     sz, WMTRSTMSG_RESET_MAX);
		if ((src == WMTDRV_TYPE_WMT) && (dst == WMTDRV_TYPE_BT) && (type == WMTMSG_TYPE_RESET)) {
			if (rst_msg == WMTRSTMSG_RESET_START) {
				BT_WARN_FUNC("BT restart start!\n");
				retflag = 1;
				wake_up_interruptible(&inq);
				/*reset_start message handling */

			} else if (rst_msg == WMTRSTMSG_RESET_END) {
				BT_WARN_FUNC("BT restart end!\n");
				retflag = 2;
				wake_up_interruptible(&inq);
				/*reset_end message handling */
			}
		}
	} else {
		/*message format invalid */
		BT_WARN_FUNC("message format invalid!\n");
	}
}

void BT_event_cb(void)
{
	BT_DBG_FUNC("BT_event_cb()\n");

	flag = 1;
	wake_up(&BT_wq);

	/* finally, awake any reader */
	wake_up_interruptible(&inq);	/* blocked in read() and select() */

	return;
}

unsigned int BT_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

/* down(&wr_mtx); */
	/*
	 * The buffer is circular; it is considered full
	 * if "wp" is right behind "rp". "left" is 0 if the
	 * buffer is empty, and it is "1" if it is completely full.
	 */
	if (mtk_wcn_stp_is_rxqueue_empty(BT_TASK_INDX)) {
		poll_wait(filp, &inq, wait);

		/* empty let select sleep */
		if ((!mtk_wcn_stp_is_rxqueue_empty(BT_TASK_INDX)) || retflag)
			mask |= POLLIN | POLLRDNORM;	/* readable */
	} else {
		mask |= POLLIN | POLLRDNORM;	/* readable */
	}

	/* do we need condition? */
	mask |= POLLOUT | POLLWRNORM;	/* writable */
/* up(&wr_mtx); */
	return mask;
}

ssize_t BT_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;
	int written = 0;
	down(&wr_mtx);

	BT_DBG_FUNC("%s: count %zd pos %lld\n", __func__, count, *f_pos);
	if (retflag) {
		if (retflag == 1) {
			/* reset start */
			retval = -88;
			BT_WARN_FUNC("MT662x reset Write: start\n");
		} else if (retflag == 2) {
			/* reset end */
			retval = -99;
			BT_WARN_FUNC("MT662x reset Write: end\n");
		}
		goto OUT;
	}

	if (count > 0) {
		int copy_size;

		if (count < BT_BUFFER_SIZE) {
			copy_size = count;
		} else {
			copy_size = BT_BUFFER_SIZE;
			BT_ERR_FUNC(" count > BT_BUFFER_SIZE\n");
		}

		if (copy_from_user(&o_buf[0], &buf[0], copy_size)) {
			retval = -EFAULT;
			goto OUT;
		}
		/* printk("%02x ", val); */

		written = mtk_wcn_stp_send_data(&o_buf[0], copy_size, BT_TASK_INDX);
		if (0 == written) {
			retval = -ENOSPC;
			/*no windowspace is available, native process should not call BT_write with no delay at all */
			BT_ERR_FUNC("target packet length:%zd, write success length:%d, retval = %d.\n", count, written,
				retval);
		} else {
			retval = written;
		}

	} else {
		retval = -EFAULT;
		BT_ERR_FUNC("target packet length:%zd is not allowed, retval = %d.\n", count, retval);
	}

OUT:
	up(&wr_mtx);
	return retval;
}

ssize_t BT_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;

	down(&rd_mtx);

	BT_DBG_FUNC("BT_read(): count %zd pos %lld\n", count, *f_pos);
	if (retflag) {
		if (retflag == 1) {
			/* reset start */
			retval = -88;
			BT_WARN_FUNC("MT662x reset Read: start\n");
		} else if (retflag == 2) {
			/* reset end */
			retval = -99;
			BT_WARN_FUNC("MT662x reset Read: end\n");
		}
		goto OUT;
	}

	if (count > BT_BUFFER_SIZE) {
		count = BT_BUFFER_SIZE;
		BT_ERR_FUNC(" count > BT_BUFFER_SIZE\n");
	}
	retval = mtk_wcn_stp_receive_data(i_buf, count, BT_TASK_INDX);

	while (retval == 0) {
		/* got nothing, wait for STP's signal */

		/*If nonblocking mode, return directly O_NONBLOCK is specified during open() */
		if (filp->f_flags & O_NONBLOCK) {
			BT_DBG_FUNC("Non-blocking BT_read()\n");
			retval = -EAGAIN;
			goto OUT;
		}

		BT_DBG_FUNC("BT_read(): wait_event 1\n");
		wait_event(BT_wq, flag != 0);
		BT_DBG_FUNC("BT_read(): wait_event 2\n");
		flag = 0;
		retval = mtk_wcn_stp_receive_data(i_buf, count, BT_TASK_INDX);
		BT_DBG_FUNC("BT_read(): mtk_wcn_stp_receive_data() = %d\n", retval);
	}

	/* we got something from STP driver */
	if (copy_to_user(buf, i_buf, retval)) {
		retval = -EFAULT;
		goto OUT;
	}

OUT:
	up(&rd_mtx);
	BT_DBG_FUNC("BT_read(): retval = %d\n", retval);
	return retval;
}

/* int BT_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg) */
long BT_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	MTK_WCN_BOOL bRet = MTK_WCN_BOOL_TRUE;

	ENUM_WMTHWVER_TYPE_T hw_ver_sym = WMTHWVER_INVALID;
	BT_DBG_FUNC("BT_ioctl(): cmd (%d)\n", cmd);

	switch (cmd) {
#if 0
	case 0:		/* enable/disable STP */
		/* George: STP is controlled by WMT only */
		/* mtk_wcn_stp_enable(arg); */
		break;
#endif
	case 1:		/* send raw data */
		BT_DBG_FUNC("BT_ioctl(): disable raw data from BT dev\n");
		retval = -EINVAL;
		break;
	case COMBO_IOC_BT_HWVER:
		/*get combo hw version */
		hw_ver_sym = mtk_wcn_wmt_hwver_get();

		BT_WARN_FUNC("BT_ioctl(): get hw version = %d, sizeof(hw_ver_sym) = %zd\n", hw_ver_sym,
			     sizeof(hw_ver_sym));
		if (copy_to_user((int __user *)arg, &hw_ver_sym, sizeof(hw_ver_sym)))
			retval = -EFAULT;
		break;

	case COMBO_IOCTL_FW_ASSERT:
		/* BT trigger fw assert for debug */
		BT_WARN_FUNC("BT Set fw assert......,arg(%ld)\n", arg);
		bRet = mtk_wcn_wmt_assert(WMTDRV_TYPE_BT, arg);
		if (bRet == MTK_WCN_BOOL_TRUE) {
			BT_WARN_FUNC("BT Set fw assert OK\n");
			retval = 0;
		} else {
            BT_ERR_FUNC("BT Set fw assert Failed\n");
			retval = (-1000);
		}
		break;
	default:
		retval = -EFAULT;
        BT_ERR_FUNC("unknown cmd (%d)\n", cmd);
		break;
	}

	return retval;
}

long BT_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	BT_WARN_FUNC("cmd[0x%x]\n", cmd);
	ret = BT_unlocked_ioctl(filp, cmd, arg);

	return ret;
}

static int BT_open(struct inode *inode, struct file *file)
{
	BT_INFO_FUNC("%s: major %d minor %d (pid %d)\n", __func__, imajor(inode), iminor(inode), current->pid);

#if 1				/* GeorgeKuo: turn on function before check stp ready */
	/* turn on BT */
	if (MTK_WCN_BOOL_FALSE == mtk_wcn_wmt_func_on(WMTDRV_TYPE_BT)) {
        BT_ERR_FUNC("WMT turn on BT fail!\n");
		return -ENODEV;
	} else {
		retflag = 0;
		mtk_wcn_wmt_msgcb_reg(WMTDRV_TYPE_BT, bt_cdev_rst_cb);
		BT_INFO_FUNC("WMT register BT rst cb!\n");
	}
#endif

	if (mtk_wcn_stp_is_ready()) {
#if 0				/* GeorgeKuo: turn on function before check stp ready */
		/* turn on BT */
		if (MTK_WCN_BOOL_FALSE == mtk_wcn_wmt_func_on(WMTDRV_TYPE_BT)) {
			BT_WARN_FUNC("WMT turn on BT fail!\n");
			return -ENODEV;
		}
#endif
		mtk_wcn_stp_set_bluez(0);

		BT_INFO_FUNC("Now it's in MTK Bluetooth Mode\n");
		BT_INFO_FUNC("WMT turn on BT OK!\n");
		BT_INFO_FUNC("STP is ready!\n");

		mtk_wcn_stp_register_event_cb(BT_TASK_INDX, BT_event_cb);
		BT_INFO_FUNC("mtk_wcn_stp_register_event_cb finish\n");
	} else {
		BT_ERR_FUNC("STP is not ready\n");

		/*return error code */
		return -ENODEV;
	}

/* init_MUTEX(&wr_mtx); */
	sema_init(&wr_mtx, 1);
/* init_MUTEX(&rd_mtx); */
	sema_init(&rd_mtx, 1);
	BT_INFO_FUNC("finish\n");

	return 0;
}

static int BT_close(struct inode *inode, struct file *file)
{
	BT_INFO_FUNC("%s: major %d minor %d (pid %d)\n", __func__, imajor(inode), iminor(inode), current->pid);
	retflag = 0;
	mtk_wcn_wmt_msgcb_unreg(WMTDRV_TYPE_BT);
	mtk_wcn_stp_register_event_cb(BT_TASK_INDX, NULL);

	if (MTK_WCN_BOOL_FALSE == mtk_wcn_wmt_func_off(WMTDRV_TYPE_BT)) {
		BT_INFO_FUNC("WMT turn off BT fail!\n");
		return -EIO;	/* mostly, native programmer will not check this return value. */
	} else {
		BT_INFO_FUNC("WMT turn off BT OK!\n");
	}

	return 0;
}

const struct file_operations BT_fops = {
	.open = BT_open,
	.release = BT_close,
	.read = BT_read,
	.write = BT_write,
/* .ioctl = BT_ioctl, */
	.unlocked_ioctl = BT_unlocked_ioctl,
    .compat_ioctl = BT_compat_ioctl,
	.poll = BT_poll
};

static int BT_init(void)
{
	dev_t dev = MKDEV(BT_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;

	/*static allocate chrdev */
	alloc_ret = register_chrdev_region(dev, 1, BT_DRIVER_NAME);
	if (alloc_ret) {
		BT_ERR_FUNC("fail to register chrdev\n");
		return alloc_ret;
	}

	cdev_init(&BT_cdev, &BT_fops);
	BT_cdev.owner = THIS_MODULE;

	cdev_err = cdev_add(&BT_cdev, dev, BT_devs);
	if (cdev_err)
		goto error;

	BT_INFO_FUNC("%s driver(major %d) installed.\n", BT_DRIVER_NAME, BT_major);
#if WMT_CREATE_NODE_DYNAMIC
	bt_class = class_create(THIS_MODULE, "stpbt");
	if (IS_ERR(bt_class))
		goto error;
	bt_dev = device_create(bt_class, NULL, dev, NULL, "stpbt");
	if (IS_ERR(bt_dev))
		goto error;

#endif
	retflag = 0;
	mtk_wcn_stp_register_event_cb(BT_TASK_INDX, NULL);

	/* init wait queue */
	init_waitqueue_head(&(inq));

	return 0;

error:
#if WMT_CREATE_NODE_DYNAMIC
	if (!(IS_ERR(bt_dev)))
		device_destroy(bt_class, dev);
	if (!(IS_ERR(bt_class))) {
		class_destroy(bt_class);
		bt_class = NULL;
	}
#endif
	if (cdev_err == 0)
		cdev_del(&BT_cdev);

	if (alloc_ret == 0)
		unregister_chrdev_region(dev, BT_devs);

	return -1;
}

static void BT_exit(void)
{
	dev_t dev = MKDEV(BT_major, 0);
	retflag = 0;
	mtk_wcn_stp_register_event_cb(BT_TASK_INDX, NULL);	/* unregister event callback function */
#if WMT_CREATE_NODE_DYNAMIC
	if (bt_dev) {
		device_destroy(bt_class, dev);
		bt_dev = NULL;
	}
	if (bt_class) {
		class_destroy(bt_class);
		bt_class = NULL;
	}
#endif
	cdev_del(&BT_cdev);
	unregister_chrdev_region(dev, BT_devs);

	BT_INFO_FUNC("%s driver removed.\n", BT_DRIVER_NAME);
}

#ifdef MTK_WCN_REMOVE_KERNEL_MODULE

int mtk_wcn_stpbt_drv_init(void)
{
	return BT_init();

}
EXPORT_SYMBOL(mtk_wcn_stpbt_drv_init);

void mtk_wcn_stpbt_drv_exit(void)
{
	return BT_exit();
}
EXPORT_SYMBOL(mtk_wcn_stpbt_drv_exit);

#else

module_init(BT_init);
module_exit(BT_exit);

#endif
