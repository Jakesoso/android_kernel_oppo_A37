/*
 * kernel/power/wakeup_reason.c
 *
 * Logs the reasons which caused the kernel to resume from
 * the suspend mode.
 *
 * Copyright (C) 2014 Google, Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/wakeup_reason.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#ifdef VENDOR_EDIT
//Wenxian.ZhEN@Prd.BaseDrv, 2016/04/15, Add for wake up source
#include <mach/mt_sleep.h>
#include <mach/mt_spm_sleep.h>
#include <mach/pcm_def.h>
#include <linux/earlysuspend.h>

#define LOG_BUF_SIZE		256 
#define PMIC_INT_WIDTH  16
#define PMIC_INT_REG_NUMBER  4
#define EINT_WIDTH  32
#define EINT_REG_NUMBER  5
int wakeup_reason_stastics_flag = 0;
extern	char wakeup_source_buf[LOG_BUF_SIZE];   
extern  u64  wakesrc_count[32];
extern u64 pmic_wakesrc_x_count[PMIC_INT_REG_NUMBER][PMIC_INT_WIDTH];
extern const char *pmic_interrupt_status_name[PMIC_INT_REG_NUMBER][PMIC_INT_WIDTH];
extern u64 eint_wakesrc_x_count[EINT_REG_NUMBER][EINT_WIDTH];
extern char * mt_eint_get_name(int index);
extern void mt_clear_wakesrc_count(void);
extern void mt_pmic_clear_wakesrc_count(void);
extern void mt_eint_clear_wakesrc_count(void);
#endif /* VENDOR_EDIT */


#define MAX_WAKEUP_REASON_IRQS 32
static int irq_list[MAX_WAKEUP_REASON_IRQS];
static int irqcount;
static bool suspend_abort;
static char abort_reason[MAX_SUSPEND_ABORT_LEN];
static struct kobject *wakeup_reason;
static DEFINE_SPINLOCK(resume_reason_lock);



static ssize_t last_resume_reason_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int irq_no, buf_offset = 0;
	struct irq_desc *desc;
	spin_lock(&resume_reason_lock);
	if (suspend_abort) {
		buf_offset = sprintf(buf, "Abort: %s", abort_reason);
	} else {
		for (irq_no = 0; irq_no < irqcount; irq_no++) {
			desc = irq_to_desc(irq_list[irq_no]);
			if (desc && desc->action && desc->action->name)
				buf_offset += sprintf(buf + buf_offset, "%d %s\n",
						irq_list[irq_no], desc->action->name);
			else
				buf_offset += sprintf(buf + buf_offset, "%d\n",
						irq_list[irq_no]);
		}
	}
	spin_unlock(&resume_reason_lock);
	return buf_offset;
}


static struct kobj_attribute resume_reason = __ATTR_RO(last_resume_reason);

#ifdef VENDOR_EDIT
//Wenxian.ZhEN@Prd.BaseDrv, 2016/04/15, Add for wake up source
static ssize_t new_resume_reason_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
		return sprintf(buf, "%s\n", wakeup_source_buf);
}

static struct kobj_attribute new_resume_reason = __ATTR_RO(new_resume_reason);



static ssize_t ap_resume_reason_stastics_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{			
	int i = 0;
	int j = 0;
	int buf_offset = 0;
	char *name;
		
	for (i = 0; i < MAX_WAKEUP_REASON_IRQS; i++) {
	if (wakesrc_count[i]) 
		{
		buf_offset += sprintf(buf + buf_offset, wakesrc_str[i]);		
		buf_offset += sprintf(buf + buf_offset,  "%s",":");
		buf_offset += sprintf(buf + buf_offset,  "%lld \n",wakesrc_count[i]);
		printk(KERN_WARNING "%s wakeup %lld times\n",wakesrc_str[i],wakesrc_count[i]);
		}
	}
	for (i = 0; i < EINT_REG_NUMBER; i++) {
		for (j = 0; j < EINT_WIDTH; j++) {
			if (eint_wakesrc_x_count[i][j] !=0)	{
				name = mt_eint_get_name(i*32 +j);
				buf_offset += sprintf(buf + buf_offset,name);		
				buf_offset += sprintf(buf + buf_offset,  "%s",":");
				buf_offset += sprintf(buf + buf_offset,  "%lld \n",eint_wakesrc_x_count[i][j]);
				printk(KERN_WARNING "%s wakeup %lld times\n",name,eint_wakesrc_x_count[i][j]);
			}
		}
	}			
	for (i = 0; i < PMIC_INT_REG_NUMBER; i++) {
		for (j = 0; j < PMIC_INT_WIDTH; j++) {
			if (pmic_wakesrc_x_count[i][j] !=0)	{
				buf_offset += sprintf(buf + buf_offset,pmic_interrupt_status_name[i][j]);		
				buf_offset += sprintf(buf + buf_offset,  "%s",":");
				buf_offset += sprintf(buf + buf_offset,  "%lld \n",pmic_wakesrc_x_count[i][j]);
				printk(KERN_WARNING "%s wakeup %lld times\n",pmic_interrupt_status_name[i][j],pmic_wakesrc_x_count[i][j]);
			}
		}
	}		
		return buf_offset;
}

static struct kobj_attribute ap_resume_reason_stastics = __ATTR_RO(ap_resume_reason_stastics);

#endif /* VENDOR_EDIT */
static struct attribute *attrs[] = {
	&resume_reason.attr,
#ifdef VENDOR_EDIT
//Wenxian.ZhEN@Prd.BaseDrv, 2016/04/15, Add for wake up source
	&new_resume_reason.attr,
	&ap_resume_reason_stastics.attr,
#endif /* VENDOR_EDIT */
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

/*
 * logs all the wake up reasons to the kernel
 * stores the irqs to expose them to the userspace via sysfs
 */
void log_wakeup_reason(int irq)
{
	struct irq_desc *desc;
	desc = irq_to_desc(irq);
	if (desc && desc->action && desc->action->name)
		printk(KERN_INFO "Resume caused by IRQ %d, %s\n", irq,
				desc->action->name);
	else
		printk(KERN_INFO "Resume caused by IRQ %d\n", irq);

	spin_lock(&resume_reason_lock);
	if (irqcount == MAX_WAKEUP_REASON_IRQS) {
		spin_unlock(&resume_reason_lock);
		printk(KERN_WARNING "Resume caused by more than %d IRQs\n",
				MAX_WAKEUP_REASON_IRQS);
		return;
	}

	irq_list[irqcount++] = irq;
	spin_unlock(&resume_reason_lock);
}

int check_wakeup_reason(int irq)
{
	int irq_no;
	int ret = false;

	spin_lock(&resume_reason_lock);
	for (irq_no = 0; irq_no < irqcount; irq_no++)
		if (irq_list[irq_no] == irq) {
			ret = true;
			break;
	}
	spin_unlock(&resume_reason_lock);
	return ret;
}

void log_suspend_abort_reason(const char *fmt, ...)
{
	va_list args;

	spin_lock(&resume_reason_lock);

	//Suspend abort reason has already been logged.
	if (suspend_abort) {
		spin_unlock(&resume_reason_lock);
		return;
	}

	suspend_abort = true;
	va_start(args, fmt);
	snprintf(abort_reason, MAX_SUSPEND_ABORT_LEN, fmt, args);
	va_end(args);
	spin_unlock(&resume_reason_lock);
}

/* Detects a suspend and clears all the previous wake up reasons*/
static int wakeup_reason_pm_event(struct notifier_block *notifier,
		unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		spin_lock(&resume_reason_lock);
		irqcount = 0;
		suspend_abort = false;
		spin_unlock(&resume_reason_lock);
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block wakeup_reason_pm_notifier_block = {
	.notifier_call = wakeup_reason_pm_event,
};
#ifdef VENDOR_EDIT
//Wenxian.ZhEN@Prd.BaseDrv, 2016/05/24, Add for wake up source
static void wakeup_reason_dev_early_suspend(struct early_suspend *h)
{
	wakeup_reason_stastics_flag = 0;
    printk(KERN_INFO "@@@@@@@@@@wakeup_reason enter early suspend@@@@@@@@@@@@@@\n");		
	mt_clear_wakesrc_count();
	mt_pmic_clear_wakesrc_count();
	mt_eint_clear_wakesrc_count();
}

static void wakeup_reason_dev_late_resume(struct early_suspend *h)
{
			
	int i = 0;
	int j = 0;
	char *name;
		
	for (i = 0; i < MAX_WAKEUP_REASON_IRQS; i++) {
	if (wakesrc_count[i]) 
		{
		printk(KERN_WARNING "%s wakeup %lld times\n",wakesrc_str[i],wakesrc_count[i]);
		}
	}
	for (i = 0; i < EINT_REG_NUMBER; i++) {
		for (j = 0; j < EINT_WIDTH; j++) {
			if (eint_wakesrc_x_count[i][j] !=0)	{
				name = mt_eint_get_name(i*32 +j);
				printk(KERN_WARNING "%s wakeup %lld times\n",name,eint_wakesrc_x_count[i][j]);
			}
		}
	}			
	for (i = 0; i < PMIC_INT_REG_NUMBER; i++) {
		for (j = 0; j < PMIC_INT_WIDTH; j++) {
			if (pmic_wakesrc_x_count[i][j] !=0)	{
				printk(KERN_WARNING "%s wakeup %lld times\n",pmic_interrupt_status_name[i][j],pmic_wakesrc_x_count[i][j]);
			}
		}
	}
	printk(KERN_INFO "@@@@@@@@@@wakeup_reason enter late resume@@@@@@@@@@@@@@\n");
	wakeup_reason_stastics_flag = 0;
}

struct early_suspend wakeup_reason_early_suspend_handler = {
    .suspend = wakeup_reason_dev_early_suspend,
    .resume = wakeup_reason_dev_late_resume,
};
#endif /* VENDOR_EDIT */

/* Initializes the sysfs parameter
 * registers the pm_event notifier
 */
int __init wakeup_reason_init(void)
{
	int retval;

	retval = register_pm_notifier(&wakeup_reason_pm_notifier_block);
	if (retval)
		printk(KERN_WARNING "[%s] failed to register PM notifier %d\n",
				__func__, retval);

	wakeup_reason = kobject_create_and_add("wakeup_reasons", kernel_kobj);
	if (!wakeup_reason) {
		printk(KERN_WARNING "[%s] failed to create a sysfs kobject\n",
				__func__);
		return 1;
	}
	retval = sysfs_create_group(wakeup_reason, &attr_group);
	if (retval) {
		kobject_put(wakeup_reason);
		printk(KERN_WARNING "[%s] failed to create a sysfs group %d\n",
				__func__, retval);
	}
#ifdef VENDOR_EDIT
//Wenxian.ZhEN@Prd.BaseDrv, 2016/05/24, Add for wake up source
	register_early_suspend(&wakeup_reason_early_suspend_handler);
	printk(KERN_INFO "wakeup_reason register_early_suspend finished\n");
#endif /* VENDOR_EDIT */
	return 0;
}

late_initcall(wakeup_reason_init);
