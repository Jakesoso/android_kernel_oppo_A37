/*************************************************************
 ** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : ah1883.c
 ** Description : Hall Sensor Driver
 ** Date        : 2013-10-24 11:59
 ** Author      : Prd.SenDrv
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/ 

#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>

#include <linux/hwmsen_helper.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <cust_eint_md1.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/semaphore.h>
#include <linux/wakelock.h>

#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/hwmsensor.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <mach/mt_gpio.h>
#include <cust_eint.h>
#include <mach/eint.h>

/*----------------------------------------------------------------*/
#define HALL_DEV_NAME "hall"

//#define GPIO_HALL_1_PIN         GPIO7
//#define GPIO_HALL_EINT_MODE   4
//#define CUST_EINT_LEVEL_SENSITIVE           1
//#define CUST_EINT_POLARITY_HIGH             1
//#define CUST_EINT_HALL_1_NUM              5
//#define CUST_EINTF_TRIGGER_HIGH      4    //High Polarity and Level Sensitive
//#define CUST_EINTF_TRIGGER_LOW       8    //Low Polarity and Level Sensitive

static unsigned int is_device_suspend = 0;
static struct early_suspend hall_early_suspend;
static int hall_enable = 1;
/*----------------------------------------------------------------*/
//extern void mt_eint_unmask(unsigned int line);
//extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
//extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
//extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
//extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
//extern s32 mt_get_gpio_pull_select_chip(u32 pin);
/*----------------------------------------------------------------*/
#define HALL_INPUT

#ifdef HALL_INPUT
static struct input_dev *hall_input = NULL;
static struct workqueue_struct *hall_workqueue = NULL;
static struct work_struct hall_report_work;
#endif

#ifdef VENDOR_EDIT 	
//ziqing.guo@BasicDrv.Sensor,2015/07/03, add for leather mode
#define HALL_STATUS_PROC
#ifdef HALL_STATUS_PROC
#include <linux/proc_fs.h> 
#include <asm/uaccess.h>
static struct proc_dir_entry *halldir = NULL;
static char* hall_node_name = "hall_status";
static char* hall_enable_node_name = "hall_enable";
#endif
#endif

static unsigned int hall_signal = 0;

#ifdef HALL_INPUT
static void hall_ev_report_func(struct work_struct *work)
{
    static unsigned long last_time;

    //Lycan.Wang@Prd.BasicDrv, 2013-12-19 
    //Add for workaround the bug when another event is too fast
    if (time_before(jiffies, last_time + msecs_to_jiffies(100))) {
        printk("%s another event is too fast !\n", __func__);
        mdelay(100);
    }
    last_time = jiffies;
	hall_signal = mt_get_gpio_in(GPIO_HALL_1_PIN);
    input_report_switch(hall_input, SW_LID, !hall_signal);
    input_sync(hall_input);
	printk("%s now status is %s\n", __func__, hall_signal ? "up" : "down");

}
#endif
/*--------------------------------------------------------------*/
static irqreturn_t hall_irq_handler(int vec, void *info)
{
	printk("%s\n",__func__);
	//disable_irq_nosync(CUST_EINT_HALL_1_NUM);
	hall_signal = mt_get_gpio_in(GPIO_HALL_1_PIN);
	mt_eint_set_polarity(CUST_EINT_HALL_1_NUM, !hall_signal);		
#ifdef HALL_INPUT
    queue_work(hall_workqueue, &hall_report_work);
#endif
	//enable_irq(CUST_EINT_HALL_1_NUM);
	return IRQ_HANDLED;
}

/*----------------------------------------------------------------*/
static void hall_early_suspend_func(struct early_suspend *h)
{
    is_device_suspend = 1;
    printk("%s\n",__func__);
}
/*----------------------------------------------------------------*/
static void hall_early_resume_func(struct early_suspend *h)
{
    is_device_suspend = 0;
    printk("%s\n",__func__);
}

#ifdef VENDOR_EDIT 	
//ziqing.guo@BasicDrv.Sensor,2015/07/03, add for leather mode
#ifdef HALL_STATUS_PROC
static ssize_t hall_node_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
	char page[8]; 	
	char *p = page;	
	int len = 0; 	
	p += sprintf(p, "%d\n",  __gpio_get_value(CUST_EINT_HALL_1_NUM));	
	len = p - page;	
	if (len > *pos)		
		len -= *pos;	
	else		
		len = 0;	

	if (copy_to_user(buf,page,len < count ? len  : count))		
		return -EFAULT;	
	*pos = *pos + (len < count ? len  : count);	

	return len < count ? len  : count;
}

#if 0
static ssize_t hall_node_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{	
	char tmp[32] = {0};	
	int ret;		
	if (count > 2)		
		return -EINVAL;		
	ret = copy_from_user(tmp, buf, 32);
	
	sscanf(tmp, "%d", &(ghall->gpio_status));	
	
	return count;	
}
#endif
static struct file_operations hall_node_ctrl = {
	.read = hall_node_read,
	//.write = hall_node_write,  
};
static ssize_t hall_enable_node_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	char page[8];
	char *p = page;
	int len = 0;
	p += sprintf(p, "%d\n",  hall_enable);
	len = p - page;
	if (len > *pos)
		len -= *pos;
	else
		len = 0;

	if (copy_to_user(buf,page,len < count ? len  : count))
		return -EFAULT;
	*pos = *pos + (len < count ? len  : count);

	return len < count ? len  : count;
}


static ssize_t hall_enable_node_write(struct file *file,const char __user *buf, size_t count, loff_t *ppos)
{
	char tmp[32] = {0};
	int ret;
    int en;
	if (count > 2)
		return -EINVAL;
	ret = copy_from_user(tmp, buf, 32);

	sscanf(tmp, "%d", &en);
	if(en){
		if(hall_enable == 0){
			enable_irq(gpio_to_irq(CUST_EINT_HALL_1_NUM));
			hall_enable = 1;
			hall_signal = mt_get_gpio_in(GPIO_HALL_1_PIN);
			input_report_switch(hall_input, SW_LID, !hall_signal);
			input_sync(hall_input);
			printk("%s the first status is %s\n", __func__, hall_signal ? "up" : "down");
		}
	}else{
		if(hall_enable == 1){
			disable_irq_nosync(gpio_to_irq(CUST_EINT_HALL_1_NUM));
			hall_enable = 0;
			input_report_switch(hall_input, SW_LID, 0);
			input_sync(hall_input);
		}
	}

	return count;
}

static struct file_operations hall_enable_node_ctrl = {
	.read = hall_enable_node_read,
	.write = hall_enable_node_write,
};

#endif//HALL_STATUS_PROC
#endif

static struct miscdevice hall_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "hall",
};

static ssize_t enable_hall_show(struct device* dev,struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", hall_enable);
}

static ssize_t enable_hall_store(struct device* dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int en;
	sscanf(buf, "%d", &en);
	if(en){
		if(hall_enable == 0){
			enable_irq(gpio_to_irq(CUST_EINT_HALL_1_NUM));
			hall_enable = 1;
			hall_signal = mt_get_gpio_in(GPIO_HALL_1_PIN);
			input_report_switch(hall_input, SW_LID, !hall_signal);
			input_sync(hall_input);
			printk("%s the first status is %s\n", __func__, hall_signal ? "up" : "down");
		}
	}else{
		if(hall_enable == 1){
			disable_irq_nosync(gpio_to_irq(CUST_EINT_HALL_1_NUM));
			hall_enable = 0;
			input_report_switch(hall_input, SW_LID, 0);
			input_sync(hall_input);
		}
	}
	return count;
}

DEVICE_ATTR(enable, 0666, enable_hall_show, enable_hall_store);

static struct attribute *hall_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group hall_attribute_group = {
	.attrs = hall_attributes
};
/*----------------------------------------------------------------*/
static int hall_probe(struct platform_device *pdev)
{
    int ret;
	int irq;

    printk("%s  Enter\n",__func__);

#ifdef HALL_INPUT
    INIT_WORK(&hall_report_work, hall_ev_report_func);
    hall_input = input_allocate_device();
    if (hall_input == NULL){
        printk("%s  allocate input device fail\n",__func__);
        goto exit;
    }
    hall_input->name = HALL_DEV_NAME;

    set_bit(EV_SW, hall_input->evbit);
    set_bit(SW_LID, hall_input->swbit);

    ret = input_register_device(hall_input);
    if (ret < 0){
        printk("%s  register input fail\n",__func__);
        goto exit;
    }
#endif

    hall_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    hall_early_suspend.suspend = hall_early_suspend_func;
    hall_early_suspend.resume = hall_early_resume_func;
    register_early_suspend(&hall_early_suspend);

    //hall_signal = mt_get_gpio_pull_select_chip(GPIO_HALL_1_PIN); //default high level

    hall_signal = mt_get_gpio_in(GPIO_HALL_1_PIN); //default high level
    printk("%s  hall_signal = %d\n",__func__,hall_signal);

	input_report_switch(hall_input, SW_LID, !hall_signal);
	input_sync(hall_input);
	
//    mt_set_gpio_mode(GPIO_HALL_1_PIN, GPIO_HALL_EINT_MODE);
//    mt_set_gpio_dir(GPIO_HALL_1_PIN, GPIO_DIR_IN);
//    mt_set_gpio_pull_enable(GPIO_HALL_1_PIN, GPIO_PULL_ENABLE);
//    mt_set_gpio_pull_select(GPIO_HALL_1_PIN, GPIO_PULL_UP);
//    mt_eint_set_hw_debounce(CUST_EINT_HALL_1_NUM, 150);
//	mt_eint_set_sens(CUST_EINT_HALL_1_NUM, CUST_EINT_LEVEL_SENSITIVE);

#ifndef VENDOR_EDIT
	if (gpio_request(CUST_EINT_HALL_1_NUM, "hall_irq"))
	{
		printk("Unable to request GPIO.\n");
		goto exit;
	}
	gpio_direction_input(CUST_EINT_HALL_1_NUM);
	irq = gpio_to_irq(CUST_EINT_HALL_1_NUM);
	if (irq < 0)
	{
		printk("Unable to request gpio irq. int pin = %d, irq = %d\n", CUST_EINT_HALL_1_NUM, irq);
		gpio_free(CUST_EINT_HALL_1_NUM);
		goto exit;
	}
#endif//VENDOR_EDIT

#ifdef VENDOR_EDIT 	
//ziqing.guo@BasicDrv.Sensor,2015/07/03, add for leather mode
 #ifdef HALL_STATUS_PROC
        halldir = proc_create(hall_node_name, 0664, NULL, &hall_node_ctrl); 
        if (halldir == NULL)
        {
            printk(" create proc/%s fail\n", hall_node_name);
            goto exit;
        }
        halldir = NULL;
        halldir = proc_create(hall_enable_node_name, 0666, NULL, &hall_enable_node_ctrl);
        if (halldir == NULL)
        {
            printk(" create proc/%s fail\n", hall_enable_node_name);
            goto exit;
        }
#endif   
#endif
#ifndef VENDOR_EDIT
	mt_set_gpio_mode(GPIO_HALL_1_PIN, GPIO_MODE_04);
	//mt_eint_registration(CUST_EINT_HALL_1_NUM,EINTF_TRIGGER_LOW, hall_irq_handler, 0);

    //if (hall_signal)
	{
		//mt_eint_registration(CUST_EINT_HALL_1_NUM, CUST_EINTF_TRIGGER_LOW, hall_eint_func, 0);
		if (request_irq(irq, hall_irq_handler, IRQ_TYPE_LEVEL_LOW, "hall_ic", NULL))
		{
			printk("%s Could not allocate HALL_INT !\n", __func__);
			goto exit;
		}
	} 
   /* else {
		//mt_eint_registration(CUST_EINT_HALL_1_NUM, CUST_EINTF_TRIGGER_HIGH, hall_eint_func, 0);
		if (request_irq(CUST_EINT_HALL_1_NUM, hall_irq_handler, IRQ_TYPE_LEVEL_HIGH, "hall_ic", NULL))
		{
			printk("%s Could not allocate HALL_INT !\n", __func__);
			goto exit;
		}
	}*/
#endif//VENDOR_EDIT

#ifndef VENDOR_EDIT
	 node = of_find_compatible_node(NULL, NULL, "mediatek, HALL_1-eint");
	 if(node){
		 of_property_read_u32_array(node , "interrupts", intr, ARRAY_SIZE(intr));
		 pr_err("intr[0] = %d, intr[1]  = %d\r\n",intr[0] ,intr[1] );
		 irq = irq_of_parse_and_map(node, 0);
	 }
	 else{
		 pr_err("node not exist!\r\n");
	 }
#else//VENDOR_EDIT
	if (gpio_request(CUST_EINT_HALL_1_NUM, "hall_irq"))
	{
		printk("Unable to request GPIO.\n");
		goto exit;
	}
	gpio_direction_input(CUST_EINT_HALL_1_NUM);
	irq = gpio_to_irq(CUST_EINT_HALL_1_NUM);
	if (irq < 0)
	{
		printk("Unable to request gpio irq. int pin = %d, irq = %d\n", CUST_EINT_HALL_1_NUM, irq);
		gpio_free(CUST_EINT_HALL_1_NUM);
		goto exit;
	}
#endif//VENDOR_EDIT
	ret = request_irq(irq, (irq_handler_t)hall_irq_handler, EINTF_TRIGGER_LOW, "HALL_1-eint",  NULL);
	if (ret)
	{
		printk(KERN_EMERG"%s:request_irq fail:ret=%d\n",__func__,ret);
		goto exit;
	}
	printk(KERN_EMERG"%s:try to register misc dev\n",__func__);
	if((ret = misc_register(&hall_misc_device)))
	{
		printk(KERN_EMERG"%s:unable to register hall misc device!!\n",__func__);
		goto exit;
	}

	ret = sysfs_create_group(&hall_misc_device.this_device->kobj,
			&hall_attribute_group);
	if (ret < 0){
		printk(KERN_EMERG"%s:unable to create hall attribute file\n",__func__);
		goto exit;
	}
	 /*eint pin has been pulled down*/
    irq_set_irq_wake(irq, 1);
    printk("%s  Exit  OK\n",__func__);
    return 0;
exit:
    printk("%s  Exit  error\n",__func__);
    return -EINVAL;
}
/*----------------------------------------------------------------*/
static int hall_remove(struct platform_device *pdev)
{
    return 0;
}
/*----------------------------------------------------------------*/
static struct platform_driver hall_driver = {
    .probe = hall_probe,
    .remove = hall_remove,
    .driver = {
        .name = HALL_DEV_NAME,
    },
};

static struct platform_device hall_device = {
	.name = "hall",
	.id = -1,
};

/*----------------------------------------------------------------*/
static int __init hall_mod_init(void)
{
    int ret;
    printk("%s Enter\n",__func__);
    ret = platform_driver_register(&hall_driver);
    if (ret < 0)
        printk("%s  register driver fail !\n",__func__);

#ifdef HALL_INPUT
    hall_workqueue = create_singlethread_workqueue("hall_work_thread");
#endif
    ret = platform_device_register(&hall_device);
   if (ret < 0)
        printk("%s  register device fail !\n",__func__);
   
    printk("%s Exit\n",__func__);
    return ret;
}
/*----------------------------------------------------------------*/
static void __exit hall_mod_exit(void)
{
#ifdef HALL_INPUT
    destroy_workqueue(hall_workqueue);
#endif
    platform_driver_unregister(&hall_driver);
}
/*----------------------------------------------------------------*/
module_init(hall_mod_init);
module_exit(hall_mod_exit);

MODULE_AUTHOR("ye.zhang <zhye@oppo.com>");
MODULE_DESCRIPTION("OPPO hall driver");
MODULE_LICENSE("GPL");
