/* Fingerprint Cards, Hybrid Touch sensor driver
 *
 * Copyright (c) 2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 *
 * Software license : "Dual BSD/GPL"
 * see <linux/module.h> and ./Documentation
 * for  details.
 *
*/

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include <asm/siginfo.h>
#include <asm/uaccess.h>

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/rcupdate.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/version.h>

#include <linux/of.h>
#include "fpc_irq.h"

#if 1 //add by haitao for spi ATTR control
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <mach/mt_clkmgr.h>
#include <linux/clk.h>

#define FPC_FUNC  printk(KERN_DEBUG "%s\n", __func__)
#endif

MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC IRQ driver.");

MODULE_LICENSE("Dual BSD/GPL");

/* -------------------------------------------------------------------------- */
/* platform compatibility                                                     */
/* -------------------------------------------------------------------------- */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	#include <linux/interrupt.h>
	#include <linux/irqreturn.h>
	#include <linux/of_gpio.h>

	#define SLEEP_US(delay) {usleep_range((delay), (delay)); }
#else
	#define SLEEP_US(delay) {usleep((delay)); }
#endif


/* -------------------------------------------------------------------------- */
/* global variables                                                           */
/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/* fpc data types                                                             */
/* -------------------------------------------------------------------------- */
struct fpc_irq_attribute {
	struct device_attribute attr;
	size_t offset;
};

struct fpc_irq_setup {
	pid_t dst_pid;
	int   dst_signo;
	int   enabled;
	int   test_trigger; // qqq todo: remove
	int   spi_enable_clk;
	int   wakelock;
	int   ta_opened;
	int   sessionid;
	int   deviceid;
};

typedef struct {
	int irq_gpio;
	int irq_no;
} fpc_irq_pdata_t;

typedef struct {
	struct platform_device  *plat_dev;
	struct device	  	*dev;
	struct class		*class;
	fpc_irq_pdata_t		pdata;
	struct fpc_irq_setup	setup;
	struct task_struct	*worker_thread;

//	dev_t			devno;
	struct semaphore	mutex;
	struct semaphore	sem_active;
	bool 			idle_request;
	bool 			term_request;

	wait_queue_head_t	wq_enable;
	wait_queue_head_t	wq_irq_return;
	bool			interrupt_done;
} fpc_irq_data_t;

#ifdef VENDOR_EDIT
//Haitao.Zhou@Prd.BaseDrv, 2015/04/21, Add for 

fpc_irq_data_t *fpc_irq;

#endif /* VENDOR_EDIT */



/* -------------------------------------------------------------------------- */
/* fpc_irq driver constants                                                   */
/* -------------------------------------------------------------------------- */
#define FPC_IRQ_DEV_NAME	"fpc_irq"
#define FPC_IRQ_CLASS_NAME	"fpsensor_irq"
#define FPC_IRQ_WORKER_NAME	"fpc_irq_worker"


/* -------------------------------------------------------------------------- */
/* function prototypes                                                        */
/* -------------------------------------------------------------------------- */
static int fpc_irq_init(void);

static void fpc_irq_exit(void);

static int fpc_irq_probe(struct platform_device *plat_dev);

static int fpc_irq_remove(struct platform_device *plat_dev);

static int fpc_irq_suspend(struct platform_device *plat_dev, pm_message_t state);

static int fpc_irq_resume(struct platform_device *plat_dev);

static int fpc_irq_get_of_pdata(struct platform_device *dev, fpc_irq_pdata_t *pdata);

static int fpc_irq_platform_init(fpc_irq_data_t *fpc_irq_data, fpc_irq_pdata_t *pdata);

static int fpc_irq_platform_destroy(fpc_irq_data_t *fpc_irq_data);

static int fpc_irq_create_class(fpc_irq_data_t *fpc_irq_data);

static int fpc_irq_worker_init(fpc_irq_data_t *fpc_irq_data);

static int fpc_irq_worker_init(fpc_irq_data_t *fpc_irq_data);

static int fpc_irq_worker_goto_idle(fpc_irq_data_t *fpc_irq_data);

static int fpc_irq_worker_enable(fpc_irq_data_t *fpc_irq_data);

static int fpc_irq_worker_destroy(fpc_irq_data_t *fpc_irq_data);

static int fpc_irq_manage_sysfs(fpc_irq_data_t *fpc_irq_data, bool create);

int fpc_irq_wait_for_interrupt(fpc_irq_data_t *fpc_irq_data, int timeout);

void fpc_irq_interrupt(void);

static ssize_t fpc_irq_show_attr_setup(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t fpc_irq_store_attr_setup(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count);

static int fpc_irq_worker_function(void *_fpc_irq_data);

static int fpc_irq_send_signal(struct device *dev,
				pid_t dst_pid,
				int signo,
				int payload);

static int fpc_irq_enable(fpc_irq_data_t *fpc_irq_data, int req_state);

static int fpc_irq_check_instance(const char *str_name);

/* -------------------------------------------------------------------------- */
/* External interface                                                         */
/* -------------------------------------------------------------------------- */
module_init(fpc_irq_init);
module_exit(fpc_irq_exit);

static struct platform_device *fpc_irq_platform_device;

static struct platform_driver fpc_irq_driver = {
	.driver	 = {
		.name		= FPC_IRQ_DEV_NAME,
		.owner		= THIS_MODULE,
	},
	.probe   = fpc_irq_probe,
	.remove  = fpc_irq_remove,
	.suspend = fpc_irq_suspend,
	.resume  = fpc_irq_resume
};


/* -------------------------------------------------------------------------- */
/* devfs                                                                      */
/* -------------------------------------------------------------------------- */
#define FPC_IRQ_ATTR(__grp, __field, __mode)				\
{									\
	.attr = __ATTR(__field, (__mode),				\
	fpc_irq_show_attr_##__grp,					\
	fpc_irq_store_attr_##__grp),					\
	.offset = offsetof(struct fpc_irq_##__grp, __field)		\
}

#define FPC_IRQ_DEV_ATTR(_grp, _field, _mode)				\
struct fpc_irq_attribute fpc_irq_attr_##_field =			\
					FPC_IRQ_ATTR(_grp, _field, (_mode))

#define DEVFS_SETUP_MODE (S_IWUSR|S_IWGRP|S_IWOTH|S_IRUSR|S_IRGRP|S_IROTH)

static FPC_IRQ_DEV_ATTR(setup, dst_pid,		DEVFS_SETUP_MODE);
static FPC_IRQ_DEV_ATTR(setup, dst_signo,	DEVFS_SETUP_MODE);
static FPC_IRQ_DEV_ATTR(setup, enabled,		DEVFS_SETUP_MODE);
static FPC_IRQ_DEV_ATTR(setup, test_trigger,	DEVFS_SETUP_MODE); // qqq

#if 1 //add by haitao for spi ATTR control
static FPC_IRQ_DEV_ATTR(setup, spi_enable_clk,	DEVFS_SETUP_MODE);
static FPC_IRQ_DEV_ATTR(setup, wakelock,	DEVFS_SETUP_MODE);
static FPC_IRQ_DEV_ATTR(setup, ta_opened,	DEVFS_SETUP_MODE);
static FPC_IRQ_DEV_ATTR(setup, sessionid,	DEVFS_SETUP_MODE);
static FPC_IRQ_DEV_ATTR(setup, deviceid,	DEVFS_SETUP_MODE);
#endif

static struct attribute *fpc_irq_setup_attrs[] = {
	&fpc_irq_attr_dst_pid.attr.attr,
	&fpc_irq_attr_dst_signo.attr.attr,
	&fpc_irq_attr_enabled.attr.attr,
	&fpc_irq_attr_test_trigger.attr.attr, // qqq
	&fpc_irq_attr_spi_enable_clk.attr.attr,
	&fpc_irq_attr_wakelock.attr.attr,
	&fpc_irq_attr_ta_opened.attr.attr,
	&fpc_irq_attr_sessionid.attr.attr,
	&fpc_irq_attr_deviceid.attr.attr,
	NULL
};

static const struct attribute_group fpc_irq_setup_attr_group = {
	.attrs = fpc_irq_setup_attrs,
	.name = "setup"
};

#if 1 //haitao add for SPI ATTR control

static unsigned int fpc_ta_opened = 0;

static struct wake_lock fpc_wake_lock;

static int fpc_sessionid = 0;
static int fpc_deviceid = 0;

static void enable_clk(void)
{
    FPC_FUNC;
	enable_clock(MT_CG_PERI_SPI0, "spi");
	return;
}

static void disable_clk(void)
{
    FPC_FUNC;
	disable_clock(MT_CG_PERI_SPI0, "spi");
	return;
}

#endif


/* -------------------------------------------------------------------------- */
/* function definitions                                                       */
/* -------------------------------------------------------------------------- */
static int fpc_irq_init(void)
{
	printk(KERN_INFO "%s\n", __func__);

	fpc_irq_platform_device = platform_device_register_simple(
							FPC_IRQ_DEV_NAME,
							0,
							NULL,
							0);

	if (IS_ERR(fpc_irq_platform_device))
		return PTR_ERR(fpc_irq_platform_device);
	
	return (platform_driver_register(&fpc_irq_driver) != 0)? EINVAL : 0;
}


/* -------------------------------------------------------------------------- */
static void fpc_irq_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);

	platform_driver_unregister(&fpc_irq_driver);

	platform_device_unregister(fpc_irq_platform_device);
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_probe(struct platform_device *plat_dev)
{
	int error = 0;
	fpc_irq_data_t *fpc_irq_data = NULL;

	fpc_irq_pdata_t *pdata_ptr;
	fpc_irq_pdata_t pdata_of;

	dev_info(&plat_dev->dev, "%s\n", __func__);

	wake_lock_init(&fpc_wake_lock, WAKE_LOCK_SUSPEND, "fpc_wake_lock");

	if (fpc_irq_check_instance(plat_dev->name) < 0) {
	        dev_info(&plat_dev->dev, "fpc_irq_check_instance failed in probe\n");
                //samuel removed for test
		//return 0;
        }

	fpc_irq_data = kzalloc(sizeof(*fpc_irq_data), GFP_KERNEL);

	if (!fpc_irq_data) {
		dev_err(&plat_dev->dev, "failed to allocate memory for struct fpc_irq_data\n");

		return -ENOMEM;
	}

	platform_set_drvdata(plat_dev, fpc_irq_data);

	fpc_irq_data->plat_dev = plat_dev;
	fpc_irq_data->dev = &plat_dev->dev;

	fpc_irq_data->pdata.irq_gpio = -EINVAL;
	fpc_irq_data->pdata.irq_no   = -EINVAL;
	fpc_irq =  fpc_irq_data;

	init_waitqueue_head(&fpc_irq_data->wq_enable);
	init_waitqueue_head(&fpc_irq_data->wq_irq_return);
//samuel test
//	pdata_ptr = plat_dev->dev.platform_data;
	
	//if (!pdata_ptr) {
		error = fpc_irq_get_of_pdata(plat_dev, &pdata_of);
		pdata_ptr = (error) ? NULL : &pdata_of;
//	}

	if (error)
		goto err_1;

	if (!pdata_ptr) {
		dev_err(fpc_irq_data->dev,
				"%s: dev.platform_data is NULL.\n", __func__);

		error = -EINVAL;
	}

	if (error)
		goto err_1;

	error = fpc_irq_platform_init(fpc_irq_data, pdata_ptr);
	if (error)
		goto err_1;

	error = fpc_irq_create_class(fpc_irq_data);
	if (error)
		goto err_2;

	error = fpc_irq_manage_sysfs(fpc_irq_data, true);
	if (error)
		goto err_3;

	error = fpc_irq_worker_init(fpc_irq_data);
	if (error)
		goto err_4;

	sema_init(&fpc_irq_data->mutex, 0);

	up(&fpc_irq_data->mutex);

	return 0;

err_4:
	fpc_irq_manage_sysfs(fpc_irq_data, false);
err_3:
	class_destroy(fpc_irq_data->class);
err_2:
	fpc_irq_platform_destroy(fpc_irq_data);
err_1:
	platform_set_drvdata(plat_dev, NULL);

	kfree(fpc_irq_data);

	return error;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_remove(struct platform_device *plat_dev)
{
	fpc_irq_data_t *fpc_irq_data;

	if (fpc_irq_check_instance(plat_dev->name) < 0)
		return 0;

	fpc_irq_data = platform_get_drvdata(plat_dev);

	fpc_irq_worker_destroy(fpc_irq_data);
//err_4:
	fpc_irq_manage_sysfs(fpc_irq_data, false);
//err_3:
	class_destroy(fpc_irq_data->class);
//err_2:
	fpc_irq_platform_destroy(fpc_irq_data);
//err_1:
	platform_set_drvdata(plat_dev, NULL);

	kfree(fpc_irq_data);

	return 0;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_suspend(struct platform_device *plat_dev, pm_message_t state)
{
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_resume(struct platform_device *plat_dev)
{
	printk(KERN_INFO "%s\n", __func__);
	return 0;
}


/* -------------------------------------------------------------------------- */

static int fpc_irq_get_of_pdata(struct platform_device *dev, fpc_irq_pdata_t *pdata)
{
	pdata->irq_gpio = FPC_IRQ_GPIO; 
	pdata->irq_no   = FPC_INT_IRQNO;

	return 0;
}



/* -------------------------------------------------------------------------- */
static int fpc_irq_platform_init(fpc_irq_data_t *fpc_irq_data, fpc_irq_pdata_t *pdata)
{
	int error = 0;

#ifndef VENDOR_EDIT
//Haitao.Zhou@Prd.BaseDrv, 2015/04/21 
	
	//fpc1020->irq_gpio = pdata->irq_gpio;
	
	// set irq gpio
  	error = mt_set_gpio_mode(pdata->irq_gpio, GPIO_MODE_04);
	if (error != 0) {
		printk("mt_set_gpio_mode (eint) failed.error=%d\n",error);
	}

  	error = mt_set_gpio_dir(pdata->irq_gpio, GPIO_DIR_IN);
	if (error != 0) {
		printk("mt_set_gpio_dir (eint) failed.error=%d\n",error);
	}

  	error = mt_set_gpio_pull_enable(pdata->irq_gpio, GPIO_PULL_DISABLE);
	if (error != 0) {
		printk("mt_set_gpio_pull_enable (eint) failed.error=%d\n",error);
	}

#endif /* VENDOR_EDIT */	

	mt_eint_registration(pdata->irq_no, EINTF_TRIGGER_RISING, fpc_irq_interrupt, 1);
	mt_eint_unmask(pdata->irq_no);
	
	return error;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_platform_destroy(fpc_irq_data_t *fpc_irq_data)
{
       #if 0
	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);

	if (fpc_irq_data->pdata.irq_no >= 0)
		free_irq(fpc_irq_data->pdata.irq_no, fpc_irq_data);

	if (gpio_is_valid(fpc_irq_data->pdata.irq_gpio))
		gpio_free(fpc_irq_data->pdata.irq_gpio);
	#endif

	return 0;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_create_class(fpc_irq_data_t *fpc_irq_data)
{
	int error = 0;

	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);

	fpc_irq_data->class = class_create(THIS_MODULE, FPC_IRQ_CLASS_NAME);

	if (IS_ERR(fpc_irq_data->class)) {
		dev_err(fpc_irq_data->dev, "failed to create class.\n");
		error = PTR_ERR(fpc_irq_data->class);
	}

	return error;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_worker_init(fpc_irq_data_t *fpc_irq_data)
{
	int error = 0;

	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);

	fpc_irq_data->idle_request = true;
	fpc_irq_data->term_request = false;

	sema_init(&fpc_irq_data->sem_active, 0);

	fpc_irq_data->worker_thread = kthread_run(
						fpc_irq_worker_function,
						fpc_irq_data,
						"%s", FPC_IRQ_WORKER_NAME);

	if (IS_ERR(fpc_irq_data->worker_thread)) {
		dev_err(fpc_irq_data->dev, "%s kthread_run failed.\n", __func__);
		error = (int)PTR_ERR(fpc_irq_data->worker_thread);
	}

	return error;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_worker_goto_idle(fpc_irq_data_t *fpc_irq_data)
{
	const int wait_idle_us = 100;

	fpc_irq_data->idle_request = true;

	if (down_trylock(&fpc_irq_data->sem_active) == 0) {
		dev_dbg(fpc_irq_data->dev, "%s : already idle\n", __func__);
		up (&fpc_irq_data->sem_active);
	} else {
		dev_dbg(fpc_irq_data->dev, "%s : idle_request\n", __func__);
		
		while (down_trylock(&fpc_irq_data->sem_active)) {

			fpc_irq_data->idle_request = true;
			wake_up_interruptible(&fpc_irq_data->wq_enable);

			fpc_irq_data->interrupt_done = true;
		    wake_up_interruptible(&fpc_irq_data->wq_irq_return);
		
			SLEEP_US(wait_idle_us);
		}

		dev_dbg(fpc_irq_data->dev, "%s : is idle\n", __func__);
		up (&fpc_irq_data->sem_active);
	}
	
	return 0;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_worker_enable(fpc_irq_data_t *fpc_irq_data)
{
	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);
			
	fpc_irq_data->idle_request = false;
	fpc_irq_data->interrupt_done = false;

	wake_up_interruptible(&fpc_irq_data->wq_enable);

	SLEEP_US(5000); //for making sure wakeup fpc_irq_worker_function

	return 0;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_worker_destroy(fpc_irq_data_t *fpc_irq_data)
{
	int error = 0;

	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);

	if (fpc_irq_data->worker_thread) {
	
		fpc_irq_worker_goto_idle(fpc_irq_data);

		fpc_irq_data->term_request = true;
		wake_up_interruptible(&fpc_irq_data->wq_enable);

		kthread_stop(fpc_irq_data->worker_thread);
		fpc_irq_data->worker_thread = NULL;
	}
	return error;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_manage_sysfs(fpc_irq_data_t *fpc_irq_data, bool create)
{
	int error = 0;

	if (create) {
		dev_dbg(fpc_irq_data->dev, "%s create\n", __func__);

		error = sysfs_create_group(&fpc_irq_data->dev->kobj,
					&fpc_irq_setup_attr_group);

		if (error) {
			dev_err(fpc_irq_data->dev,
				"sysf_create_group failed.\n");
			return error;
		}

	} else {
		dev_dbg(fpc_irq_data->dev, "%s remove\n", __func__);
	
		sysfs_remove_group(&fpc_irq_data->dev->kobj, &fpc_irq_setup_attr_group);
	}

	return error;
}


/* -------------------------------------------------------------------------- */
int fpc_irq_wait_for_interrupt(fpc_irq_data_t *fpc_irq_data, int timeout)
{
	int result = 0;

	if (!timeout) {
		result = wait_event_interruptible(
				fpc_irq_data->wq_irq_return,
				fpc_irq_data->interrupt_done);
	} else {
		result = wait_event_interruptible_timeout(
				fpc_irq_data->wq_irq_return,
				fpc_irq_data->interrupt_done, timeout);
	}

	if (result < 0) {
		dev_err(fpc_irq_data->dev,
			 "wait_event_interruptible interrupted by signal (%d).\n", result);

		return result;
	}

	if (result || !timeout) {
		fpc_irq_data->interrupt_done = false;
		return 0;
	}

	return -ETIMEDOUT;
}


/* -------------------------------------------------------------------------- */
void  fpc_irq_interrupt(void)
{
	fpc_irq_data_t *fpc_irq_data = fpc_irq;
	if (mt_get_gpio_in(fpc_irq_data->pdata.irq_gpio)) {
		fpc_irq_data->interrupt_done = true;
		wake_up_interruptible(&fpc_irq_data->wq_irq_return);
		return;
	}

	return;
}


/* -------------------------------------------------------------------------- */
static ssize_t fpc_irq_show_attr_setup(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	fpc_irq_data_t *fpc_irq_data = dev_get_drvdata(dev);
	struct fpc_irq_attribute *fpc_attr;
	int val = -1;

	fpc_attr = container_of(attr, struct fpc_irq_attribute, attr);

	if (fpc_attr->offset == offsetof(struct fpc_irq_setup, dst_pid))
		val = fpc_irq_data->setup.dst_pid;
	else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, dst_signo))
		val = fpc_irq_data->setup.dst_signo;
	else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, enabled))
		val = fpc_irq_data->setup.enabled;
	else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, test_trigger))
		val = fpc_irq_data->setup.test_trigger;
	else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, ta_opened))
	    val = fpc_ta_opened;
	else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, sessionid))
	    val = fpc_sessionid;
	else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, deviceid))
	    val = fpc_deviceid;
	if (val >= 0)
		return scnprintf(buf, PAGE_SIZE, "%i\n", val);

	return -ENOENT;
}

/* -------------------------------------------------------------------------- */
static ssize_t fpc_irq_store_attr_setup(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	int error = 0;
	fpc_irq_data_t *fpc_irq_data = dev_get_drvdata(dev);
	struct fpc_irq_attribute *fpc_attr;
	u64 val;
	
	error = kstrtou64(buf, 0, &val);

	fpc_attr = container_of(attr, struct fpc_irq_attribute, attr);

	if (!error) {

		if (fpc_attr->offset == offsetof(struct fpc_irq_setup, dst_pid))
			fpc_irq_data->setup.dst_pid = (pid_t)val;

		else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, dst_signo))
			fpc_irq_data->setup.dst_signo = (int)val;

		else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, enabled)) {
			fpc_irq_enable(fpc_irq_data, (int)val);
		}
		else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, test_trigger)) {

			fpc_irq_data->setup.test_trigger = (int)val;

			fpc_irq_send_signal(fpc_irq_data->dev,
						fpc_irq_data->setup.dst_pid,
						fpc_irq_data->setup.dst_signo,
						fpc_irq_data->setup.test_trigger
				      		);
		}
		else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, spi_enable_clk)){
		    if ((int)val == 0){
		        disable_clk();
		    }else{
                enable_clk();
		    }
		}
		else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, wakelock)){
            if ((int)val == 0){
                wake_unlock(&fpc_wake_lock);
                printk(KERN_DEBUG "%s, wake_unlock\n", __func__);
            }else{
                wake_lock(&fpc_wake_lock);
                printk(KERN_DEBUG "%s, wake_lock\n", __func__);
            }
		}
		else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, ta_opened)){
            fpc_ta_opened = (int)val;
            printk(KERN_DEBUG "%s, set fpc_ta_opened %d\n", __func__, fpc_ta_opened);
		}
		else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, sessionid)){
            fpc_sessionid = (int)val;
            printk(KERN_DEBUG "%s, set fpc_sessionid %d\n", __func__, fpc_sessionid);
		}
		else if (fpc_attr->offset == offsetof(struct fpc_irq_setup, deviceid)){
            fpc_deviceid = (int)val;
            printk(KERN_DEBUG "%s, set fpc_deviceid %d\n", __func__, fpc_deviceid);
		}
		else
			return -ENOENT;

		return strnlen(buf, count);
	}

	return error;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_worker_function(void *_fpc_irq_data)
{
	int status;
	const int irq_timeout_ms = 0;//1;
	fpc_irq_data_t *fpc_irq_data = _fpc_irq_data;

	while (!kthread_should_stop()) {

		up(&fpc_irq_data->sem_active);

		dev_dbg(fpc_irq_data->dev, "%s : waiting\n", __func__);

		wait_event_interruptible(fpc_irq_data->wq_enable,
				!fpc_irq_data->idle_request || fpc_irq_data->term_request);

		if (fpc_irq_data->term_request)
			continue;

		down(&fpc_irq_data->sem_active);

		if  (!fpc_irq_data->idle_request)
			dev_dbg(fpc_irq_data->dev, "%s : running\n", __func__);

		//enable_irq(fpc_irq_data->pdata.irq_no);
		mt_eint_unmask(fpc_irq_data->pdata.irq_no);

		while (!fpc_irq_data->idle_request) {
			status = fpc_irq_wait_for_interrupt(fpc_irq_data, irq_timeout_ms);

			if ((status >= 0) && (status != -ETIMEDOUT) && (!fpc_irq_data->idle_request)) {
				fpc_irq_send_signal(
						fpc_irq_data->dev,
						fpc_irq_data->setup.dst_pid,
						fpc_irq_data->setup.dst_signo,
						0);
			    printk(KERN_DEBUG "%s, fpc_irq_send_signal end\n", __func__);
			}
		}
		
		//disable_irq(fpc_irq_data->pdata.irq_no);
		mt_eint_mask(fpc_irq_data->pdata.irq_no);
	}

	dev_dbg(fpc_irq_data->dev, "%s : exit\n", __func__);

	return 0;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_send_signal(struct device *dev,
				pid_t dst_pid,
				int signo,
				int payload)
{
	int ret = 0;

	struct siginfo		info;
	struct task_struct	*dst_task;

	dev_dbg(dev, "%s\n", __func__);

	if (dst_pid == 0) {
		dev_err(dev, "%s : destination PID not set!\n", __func__);
		return -ENODEV;
	}

	memset(&info, 0, sizeof(struct siginfo));

	info.si_signo = signo;
	info.si_code  = SI_QUEUE;
	info.si_int   = payload;

	rcu_read_lock();

	dst_task = pid_task(find_pid_ns(dst_pid, &init_pid_ns), PIDTYPE_PID);

	if (dst_task == NULL)
		ret = -ENODEV;

	rcu_read_unlock();

	if (ret < 0) {
		dev_err(dev, "%s : no such PID %d\n", __func__, dst_pid);
		return ret;
	}

 	ret = send_sig_info(signo, &info, dst_task);
	if (ret < 0)
		dev_err(dev, "%s : unable to send signal\n", __func__);
	else
		dev_dbg(dev, "%s sent signal %d, with payload %d to PID:%d\n", __func__, signo, payload, dst_pid);

	return ret;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_enable(fpc_irq_data_t *fpc_irq_data, int req_state)
{
	dev_dbg(fpc_irq_data->dev, "%s\n", __func__);

	if (req_state == 0) {
		if (fpc_irq_data->setup.enabled) {
			fpc_irq_worker_goto_idle(fpc_irq_data);
			fpc_irq_data->setup.enabled = 0;
		}
	} else {
		if (fpc_irq_data->setup.enabled == 0) {
			fpc_irq_worker_enable(fpc_irq_data);
			fpc_irq_data->setup.enabled = 1;
		}
	}
	return 0;
}


/* -------------------------------------------------------------------------- */
static int fpc_irq_check_instance(const char *str_name)
{
	const char str_ref[] = FPC_IRQ_DEV_NAME;
	size_t index;

	printk(KERN_INFO "%s => %s \n", __func__, str_name);

	index = sizeof(str_ref);
	while (index) {
		--index;
       /* printk("index=%d, str_ref[index]=%d, str_name[index]=%d\n",index, str_ref[index],str_name[index]);*/
		if (str_ref[index] != str_name[index])
		    return 0;
	}
	return -EINVAL;
}

