/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*kpd.h file path: ALPS/mediatek/kernel/include/linux */
#include <linux/kpd.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#else
#include <linux/pm_wakeup.h>
#endif
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#include <cust_eint.h>

#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2015/12/10, add for home key signal to fingerprint */
#define HOME_STATUS_SIGNAL

#ifdef HOME_STATUS_SIGNAL
#include <linux/fs.h>
static struct fasync_struct *async_queue = NULL;
#endif
#endif /*VENDOR_EDIT*/


#define KPD_NAME	"mtk-kpd"
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2016.01.29 for int
#define KPD_HOME_NAME	"mtk-kpd-home"
#define KPD_VOL_UP_NAME	"mtk-kpd-vol-up"
#define KPD_VOL_DOWN_NAME	"mtk-kpd-vol-down"
#endif/*VENDOR_EDIT*/

#define MTK_KP_WAKESOURCE	/* this is for auto set wake up source */

#ifdef CONFIG_OF
void __iomem *kp_base;
static unsigned int kp_irqnr;
#endif
struct input_dev *kpd_input_dev;
static bool kpd_suspend;
static int kpd_show_hw_keycode = 1;
static int kpd_show_register = 1;
static volatile int call_status;

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2016.01.29 for int
#define CUST_EINT_POLARITY_LOW              0
#define CUST_EINT_POLARITY_HIGH             1
#define CUST_EINT_DEBOUNCE_DISABLE          0
#define CUST_EINT_DEBOUNCE_ENABLE           1
#define CUST_EINT_EDGE_SENSITIVE            0
#define CUST_EINT_LEVEL_SENSITIVE           1

#define GPIO_VOLUMEKEY_UP               GPIO103
#define GPIO_VOLUMEKEY_DOWN             GPIO104

#define EINT_VOLUMEKEY_UP_NUM           103
#define EINT_VOLUMEKEY_DOWN_NUM     	104
#define KEY_UNPRESS                                 	(0)
#define KEY_PRESS                                       (1)

static int vol_up_press = KEY_UNPRESS;
static int vol_down_press = KEY_UNPRESS;
static unsigned int vol_up_irq = 0;
static unsigned int vol_down_irq = 0;



static irqreturn_t kpd_volumekey_up_eint(int irq, void *dev_id);
static void kpd_volumekey_up_handler(unsigned long data);
static DECLARE_TASKLET(kpd_volumekey_up_tasklet, kpd_volumekey_up_handler, 0);
static irqreturn_t kpd_volumekey_down_eint(int irq, void *dev_id);
static void kpd_volumekey_down_handler(unsigned long data);
static DECLARE_TASKLET(kpd_volumekey_down_tasklet, kpd_volumekey_down_handler, 0);

static void kpd_volumekey_up_handler(unsigned long data)
{
    					   
	input_report_key(kpd_input_dev, KEY_VOLUMEUP, !vol_up_press);  
	input_sync(kpd_input_dev);
	printk(KERN_ALERT"%s is called,vol_up_press = %d.\r\n", __func__, vol_up_press);
	
	enable_irq(vol_up_irq);
}

static irqreturn_t kpd_volumekey_up_eint(int irq, void *dev_id)
{
	disable_irq_nosync(vol_up_irq);
	vol_up_press = mt_get_gpio_in(GPIO_VOLUMEKEY_UP);
	//printk(KERN_ALERT "%s vol_up_press:%d.\n", __func__, vol_up_press);
	mt_eint_set_polarity(EINT_VOLUMEKEY_UP_NUM, !vol_up_press);
	

	tasklet_schedule(&kpd_volumekey_up_tasklet);
	return IRQ_HANDLED;
}

static void kpd_volumekey_down_handler(unsigned long data)
{
	input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, !vol_down_press);
	input_sync(kpd_input_dev);
	printk(KERN_ALERT"%s is called,vol_down_press = %d.\r\n", __func__,vol_down_press);

	enable_irq(vol_down_irq);
}


static irqreturn_t kpd_volumekey_down_eint(int irq, void *dev_id)
{
	
	disable_irq_nosync(vol_down_irq);
	vol_down_press = mt_get_gpio_in(GPIO_VOLUMEKEY_DOWN);
	//printk(KERN_ALERT "%s vol_down_press:%d.\n", __func__, vol_down_press);	
	mt_eint_set_polarity(EINT_VOLUMEKEY_DOWN_NUM, !vol_down_press);
	
	tasklet_schedule(&kpd_volumekey_down_tasklet);

	return IRQ_HANDLED;
}

#endif/*VENDOR_EDIT*/




#ifdef VENDOR_EDIT
// Jingchun.Wang@Phone.Bsp.Driver, 2015/11/10  Add for for HOME key 
extern void mt_eint_dump_status(unsigned int eint);
#ifdef GPIO_HOME_PIN
//Modified by Tong.han for if there is GPIO_HOME_PIN.then compile the below code,2016-1-15
#define GPIO_HOME               GPIO_HOME_PIN

#define EINT_HOME_NUM           CUST_EINT_HOME_KEY_NUM

static unsigned int home_irq = 0;

static irqreturn_t kpd_home_eint(int irq, void *dev_id);
static void kpd_home_handler(unsigned long data);
static DECLARE_TASKLET(kpd_home_tasklet, kpd_home_handler, 0);

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2015.12.18 for homekey
static int home_press = KEY_UNPRESS;
#endif/*VENDOR_EDIT*/

#ifdef VENDOR_EDIT//Haitao.Zhou@ProDrv.CHG,modified 2016.01.14 for send homekey signael only when backlight off
extern volatile unsigned int share_backlight_level;
#endif/*VENDOR_EDIT*/

static void kpd_home_handler(unsigned long data)
{
	input_report_key(kpd_input_dev, KEY_HOME, !home_press);
	input_sync(kpd_input_dev);
	printk(KERN_DEBUG "%s %d is called, backlight_level %u.\n", __func__, home_press, share_backlight_level);

#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2015/12/10, add for home key signal to fingerprint. */
#ifdef HOME_STATUS_SIGNAL
    if (!home_press && async_queue && !share_backlight_level) {
        printk(KERN_DEBUG "%s homekey:%d , send a signal.. \n", __func__, home_press);
        kill_fasync(&async_queue, SIGIO, POLL_IN); 
    }
#endif
#endif /*VENDOR_EDIT*/

	enable_irq(home_irq);
}


static irqreturn_t kpd_home_eint(int irq, void *dev_id)
{
	disable_irq_nosync(home_irq);
	home_press = mt_get_gpio_in(GPIO_HOME);
	printk(KERN_DEBUG "%s home_press:%d.\n", __func__, home_press);
	mt_eint_set_polarity(EINT_HOME_NUM, !home_press);
		
	tasklet_schedule(&kpd_home_tasklet);
	return IRQ_HANDLED;
}


#endif /*GPIO_HOME_PIN*/
#endif /*VENDOR_EDIT*/

#ifdef CONFIG_HAS_WAKELOCK
struct wake_lock kpd_suspend_lock;	/* For suspend usage */
#else
struct wakeup_source kpd_suspend_lock;
#endif

/*for kpd_memory_setting() function*/
static u16 kpd_keymap[KPD_NUM_KEYS];
static u16 kpd_keymap_state[KPD_NUM_MEMS];
/***********************************/

/* for slide QWERTY */
#if KPD_HAS_SLIDE_QWERTY
static void kpd_slide_handler(unsigned long data);
static DECLARE_TASKLET(kpd_slide_tasklet, kpd_slide_handler, 0);
static u8 kpd_slide_state = !KPD_SLIDE_POLARITY;
#endif
#if !defined(CONFIG_MTK_LEGACY)
struct keypad_dts_data kpd_dts_data;
#endif
/* for Power key using EINT */
#if KPD_PWRKEY_USE_EINT
static void kpd_pwrkey_handler(unsigned long data);
static DECLARE_TASKLET(kpd_pwrkey_tasklet, kpd_pwrkey_handler, 0);
#endif

/* for keymap handling */
static void kpd_keymap_handler(unsigned long data);
static DECLARE_TASKLET(kpd_keymap_tasklet, kpd_keymap_handler, 0);

/*********************************************************************/
static void kpd_memory_setting(void);

/*********************************************************************/
static int kpd_pdrv_probe(struct platform_device *pdev);
static int kpd_pdrv_remove(struct platform_device *pdev);
static int kpd_pdrv_suspend(struct platform_device *pdev, pm_message_t state);
static int kpd_pdrv_resume(struct platform_device *pdev);

#ifdef CONFIG_OF
static const struct of_device_id kpd_of_match[] = {
	{.compatible = "mediatek,KP",},
	{.compatible = "mediatek,mt6755-keypad",},
	{},
};
#endif

static struct platform_driver kpd_pdrv = {
	.probe = kpd_pdrv_probe,
	.remove = kpd_pdrv_remove,
	.suspend = kpd_pdrv_suspend,
	.resume = kpd_pdrv_resume,
	.driver = {
		   .name = KPD_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = kpd_of_match,
#endif
		   },
};

/********************************************************************/
static void kpd_memory_setting(void)
{
	kpd_init_keymap(kpd_keymap);
	kpd_init_keymap_state(kpd_keymap_state);
	return;
}

/*****************for kpd auto set wake up source*************************/

static ssize_t kpd_store_call_state(struct device_driver *ddri, const char *buf, size_t count)
{
	if (sscanf(buf, "%u", &call_status) != 1) {
		kpd_print("kpd call state: Invalid values\n");
		return -EINVAL;
	}

	switch (call_status) {
	case 1:
		kpd_print("kpd call state: Idle state!\n");
		break;
	case 2:
		kpd_print("kpd call state: ringing state!\n");
		break;
	case 3:
		kpd_print("kpd call state: active or hold state!\n");
		break;

	default:
		kpd_print("kpd call state: Invalid values\n");
		break;
	}
	return count;
}

static ssize_t kpd_show_call_state(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	res = snprintf(buf, PAGE_SIZE, "%d\n", call_status);
	return res;
}

static DRIVER_ATTR(kpd_call_state, S_IWUSR | S_IRUGO, kpd_show_call_state, kpd_store_call_state);

static struct driver_attribute *kpd_attr_list[] = {
	&driver_attr_kpd_call_state,
};

/*----------------------------------------------------------------------------*/
static int kpd_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(kpd_attr_list) / sizeof(kpd_attr_list[0]));
	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, kpd_attr_list[idx]);
		if (err) {
			kpd_info("driver_create_file (%s) = %d\n", kpd_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int kpd_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(kpd_attr_list) / sizeof(kpd_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, kpd_attr_list[idx]);

	return err;
}

/*----------------------------------------------------------------------------*/
/********************************************************************************************/
/************************************************************************************************************************************************/
/* for autotest */
#if KPD_AUTOTEST
static const u16 kpd_auto_keymap[] = {
	KEY_MENU,
	KEY_HOME, KEY_BACK,
	KEY_CALL, KEY_ENDCALL,
	KEY_VOLUMEUP, KEY_VOLUMEDOWN,
	KEY_FOCUS, KEY_CAMERA,
};
#endif
/* for AEE manual dump */
#define AEE_VOLUMEUP_BIT	0
#define AEE_VOLUMEDOWN_BIT	1
#define AEE_DELAY_TIME		15
/* enable volup + voldown was pressed 5~15 s Trigger aee manual dump */
#define AEE_ENABLE_5_15		1
static struct hrtimer aee_timer;
static unsigned long aee_pressed_keys;
static bool aee_timer_started;

#if AEE_ENABLE_5_15
#define AEE_DELAY_TIME_5S	5
static struct hrtimer aee_timer_5s;
static bool aee_timer_5s_started;
static bool flags_5s;
#endif

static inline void kpd_update_aee_state(void)
{
	if (aee_pressed_keys == ((1 << AEE_VOLUMEUP_BIT) | (1 << AEE_VOLUMEDOWN_BIT))) {
		/* if volumeup and volumedown was pressed the same time then start the time of ten seconds */
		aee_timer_started = true;

#if AEE_ENABLE_5_15
		aee_timer_5s_started = true;
		hrtimer_start(&aee_timer_5s, ktime_set(AEE_DELAY_TIME_5S, 0), HRTIMER_MODE_REL);
#endif
		hrtimer_start(&aee_timer, ktime_set(AEE_DELAY_TIME, 0), HRTIMER_MODE_REL);
		kpd_print("aee_timer started\n");
	} else {
		if (aee_timer_started) {
/*
  * hrtimer_cancel - cancel a timer and wait for the handler to finish.
  * Returns:
  *	0 when the timer was not active.
  *	1 when the timer was active.
 */
			if (hrtimer_cancel(&aee_timer)) {
				kpd_print("try to cancel hrtimer\n");
#if AEE_ENABLE_5_15
				if (flags_5s) {
					kpd_print("Pressed Volup + Voldown5s~15s then trigger aee manual dump.\n");
					aee_kernel_reminding("manual dump", "Trigger Vol Up +Vol Down 5s");
				}
#endif

			}
#if AEE_ENABLE_5_15
			flags_5s = false;
#endif
			aee_timer_started = false;
			kpd_print("aee_timer canceled\n");
		}
#if AEE_ENABLE_5_15
		if (aee_timer_5s_started) {
/*
  * hrtimer_cancel - cancel a timer and wait for the handler to finish.
  * Returns:
  *	0 when the timer was not active.
  *	1 when the timer was active.
 */
			if (hrtimer_cancel(&aee_timer_5s))
				kpd_print("try to cancel hrtimer (5s)\n");
			aee_timer_5s_started = false;
			kpd_print("aee_timer canceled (5s)\n");
		}
#endif
	}
}

static void kpd_aee_handler(u32 keycode, u16 pressed)
{
	if (pressed) {
		if (keycode == KEY_VOLUMEUP)
			__set_bit(AEE_VOLUMEUP_BIT, &aee_pressed_keys);
		else if (keycode == KEY_VOLUMEDOWN)
			__set_bit(AEE_VOLUMEDOWN_BIT, &aee_pressed_keys);
		else
			return;
		kpd_update_aee_state();
	} else {
		if (keycode == KEY_VOLUMEUP)
			__clear_bit(AEE_VOLUMEUP_BIT, &aee_pressed_keys);
		else if (keycode == KEY_VOLUMEDOWN)
			__clear_bit(AEE_VOLUMEDOWN_BIT, &aee_pressed_keys);
		else
			return;
		kpd_update_aee_state();
	}
}

static enum hrtimer_restart aee_timer_func(struct hrtimer *timer)
{
	/* kpd_info("kpd: vol up+vol down AEE manual dump!\n"); */
	/* aee_kernel_reminding("manual dump ", "Triggered by press KEY_VOLUMEUP+KEY_VOLUMEDOWN"); */
	aee_trigger_kdb();
	return HRTIMER_NORESTART;
}

#if AEE_ENABLE_5_15
static enum hrtimer_restart aee_timer_5s_func(struct hrtimer *timer)
{

	/* kpd_info("kpd: vol up+vol down AEE manual dump timer 5s !\n"); */
	flags_5s = true;
	return HRTIMER_NORESTART;
}
#endif

/************************************************************************/

#if KPD_HAS_SLIDE_QWERTY
static void kpd_slide_handler(unsigned long data)
{
	bool slid;
	u8 old_state = kpd_slide_state;

	kpd_slide_state = !kpd_slide_state;
	slid = (kpd_slide_state == !!KPD_SLIDE_POLARITY);
	/* for SW_LID, 1: lid open => slid, 0: lid shut => closed */
	input_report_switch(kpd_input_dev, SW_LID, slid);
	input_sync(kpd_input_dev);
	kpd_print("report QWERTY = %s\n", slid ? "slid" : "closed");

	if (old_state)
		mt_set_gpio_pull_select(GPIO_QWERTYSLIDE_EINT_PIN, 0);
	else
		mt_set_gpio_pull_select(GPIO_QWERTYSLIDE_EINT_PIN, 1);
	/* for detecting the return to old_state */
	mt65xx_eint_set_polarity(KPD_SLIDE_EINT, old_state);
	mt65xx_eint_unmask(KPD_SLIDE_EINT);
}

static void kpd_slide_eint_handler(void)
{
	tasklet_schedule(&kpd_slide_tasklet);
}
#endif

#if KPD_PWRKEY_USE_EINT
static void kpd_pwrkey_handler(unsigned long data)
{
	kpd_pwrkey_handler_hal(data);
}

static void kpd_pwrkey_eint_handler(void)
{
	tasklet_schedule(&kpd_pwrkey_tasklet);
}
#endif
/*********************************************************************/

/*********************************************************************/
#if KPD_PWRKEY_USE_PMIC
void kpd_pwrkey_pmic_handler(unsigned long pressed)
{
	kpd_print("Power Key generate, pressed=%ld\n", pressed);
	if (!kpd_input_dev) {
		kpd_print("KPD input device not ready\n");
		return;
	}
	kpd_pmic_pwrkey_hal(pressed);
}
#endif

void kpd_pmic_rstkey_handler(unsigned long pressed)
{
	kpd_print("PMIC reset Key generate, pressed=%ld\n", pressed);
	if (!kpd_input_dev) {
		kpd_print("KPD input device not ready\n");
		return;
	}
	kpd_pmic_rstkey_hal(pressed);
#ifdef KPD_PMIC_RSTKEY_MAP
	kpd_aee_handler(KPD_PMIC_RSTKEY_MAP, pressed);
#endif
}

/*********************************************************************/

/*********************************************************************/
static void kpd_keymap_handler(unsigned long data)
{
	int i, j;
	bool pressed;
	u16 new_state[KPD_NUM_MEMS], change, mask;
	u16 hw_keycode, linux_keycode;
	kpd_get_keymap_state(new_state);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_timeout(&kpd_suspend_lock, HZ / 2);
#else
	__pm_wakeup_event(&kpd_suspend_lock, 500);
#endif

	for (i = 0; i < KPD_NUM_MEMS; i++) {
		change = new_state[i] ^ kpd_keymap_state[i];
		if (!change)
			continue;

		for (j = 0; j < 16; j++) {
			mask = 1U << j;
			if (!(change & mask))
				continue;

			hw_keycode = (i << 4) + j;
			/* bit is 1: not pressed, 0: pressed */
			pressed = !(new_state[i] & mask);
			if (kpd_show_hw_keycode)
				kpd_print("(%s) HW keycode = %u\n", pressed ? "pressed" : "released", hw_keycode);
			BUG_ON(hw_keycode >= KPD_NUM_KEYS);
			linux_keycode = kpd_keymap[hw_keycode];
			if (unlikely(linux_keycode == 0)) {
				kpd_print("Linux keycode = 0\n");
				continue;
			}
			kpd_aee_handler(linux_keycode, pressed);

			kpd_backlight_handler(pressed, linux_keycode);
			input_report_key(kpd_input_dev, linux_keycode, pressed);
			input_sync(kpd_input_dev);
			kpd_print("report Linux keycode = %u\n", linux_keycode);
		}
	}

	memcpy(kpd_keymap_state, new_state, sizeof(new_state));
	kpd_print("save new keymap state\n");
#ifdef CONFIG_OF
	enable_irq(kp_irqnr);
#else
	enable_irq(MT_KP_IRQ_ID);
#endif
}

static irqreturn_t kpd_irq_handler(int irq, void *dev_id)
{
	/* use _nosync to avoid deadlock */
#ifdef CONFIG_OF
	disable_irq_nosync(kp_irqnr);
#else
	disable_irq_nosync(MT_KP_IRQ_ID);
#endif
	tasklet_schedule(&kpd_keymap_tasklet);
	return IRQ_HANDLED;
}

/*********************************************************************/

/*****************************************************************************************/
long kpd_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/* void __user *uarg = (void __user *)arg; */

	switch (cmd) {
#if KPD_AUTOTEST
	case PRESS_OK_KEY:	/* KPD_AUTOTEST disable auto test setting to resolve CR ALPS00464496 */
		if (test_bit(KEY_OK, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS OK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_OK, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support OK KEY!!\n");
		}
		break;
	case RELEASE_OK_KEY:
		if (test_bit(KEY_OK, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE OK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_OK, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support OK KEY!!\n");
		}
		break;
	case PRESS_MENU_KEY:
		if (test_bit(KEY_MENU, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS MENU KEY!!\n");
			input_report_key(kpd_input_dev, KEY_MENU, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support MENU KEY!!\n");
		}
		break;
	case RELEASE_MENU_KEY:
		if (test_bit(KEY_MENU, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE MENU KEY!!\n");
			input_report_key(kpd_input_dev, KEY_MENU, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support MENU KEY!!\n");
		}

		break;
	case PRESS_UP_KEY:
		if (test_bit(KEY_UP, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS UP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_UP, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support UP KEY!!\n");
		}
		break;
	case RELEASE_UP_KEY:
		if (test_bit(KEY_UP, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE UP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_UP, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support UP KEY!!\n");
		}
		break;
	case PRESS_DOWN_KEY:
		if (test_bit(KEY_DOWN, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS DOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_DOWN, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support DOWN KEY!!\n");
		}
		break;
	case RELEASE_DOWN_KEY:
		if (test_bit(KEY_DOWN, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE DOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_DOWN, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support DOWN KEY!!\n");
		}
		break;
	case PRESS_LEFT_KEY:
		if (test_bit(KEY_LEFT, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS LEFT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_LEFT, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support LEFT KEY!!\n");
		}
		break;
	case RELEASE_LEFT_KEY:
		if (test_bit(KEY_LEFT, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE LEFT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_LEFT, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support LEFT KEY!!\n");
		}
		break;

	case PRESS_RIGHT_KEY:
		if (test_bit(KEY_RIGHT, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS RIGHT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_RIGHT, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support RIGHT KEY!!\n");
		}
		break;
	case RELEASE_RIGHT_KEY:
		if (test_bit(KEY_RIGHT, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE RIGHT KEY!!\n");
			input_report_key(kpd_input_dev, KEY_RIGHT, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support RIGHT KEY!!\n");
		}
		break;
	case PRESS_HOME_KEY:
		if (test_bit(KEY_HOME, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS HOME KEY!!\n");
			input_report_key(kpd_input_dev, KEY_HOME, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support HOME KEY!!\n");
		}
		break;
	case RELEASE_HOME_KEY:
		if (test_bit(KEY_HOME, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE HOME KEY!!\n");
			input_report_key(kpd_input_dev, KEY_HOME, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support HOME KEY!!\n");
		}
		break;
	case PRESS_BACK_KEY:
		if (test_bit(KEY_BACK, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS BACK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_BACK, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support BACK KEY!!\n");
		}
		break;
	case RELEASE_BACK_KEY:
		if (test_bit(KEY_BACK, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE BACK KEY!!\n");
			input_report_key(kpd_input_dev, KEY_BACK, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support BACK KEY!!\n");
		}
		break;
	case PRESS_CALL_KEY:
		if (test_bit(KEY_CALL, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS CALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CALL, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support CALL KEY!!\n");
		}
		break;
	case RELEASE_CALL_KEY:
		if (test_bit(KEY_CALL, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE CALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CALL, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support CALL KEY!!\n");
		}
		break;

	case PRESS_ENDCALL_KEY:
		if (test_bit(KEY_ENDCALL, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS ENDCALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_ENDCALL, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support ENDCALL KEY!!\n");
		}
		break;
	case RELEASE_ENDCALL_KEY:
		if (test_bit(KEY_ENDCALL, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE ENDCALL KEY!!\n");
			input_report_key(kpd_input_dev, KEY_ENDCALL, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support ENDCALL KEY!!\n");
		}
		break;
	case PRESS_VLUP_KEY:
		if (test_bit(KEY_VOLUMEUP, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS VOLUMEUP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEUP, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support VOLUMEUP KEY!!\n");
		}
		break;
	case RELEASE_VLUP_KEY:
		if (test_bit(KEY_VOLUMEUP, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE VOLUMEUP KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEUP, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support VOLUMEUP KEY!!\n");
		}
		break;
	case PRESS_VLDOWN_KEY:
		if (test_bit(KEY_VOLUMEDOWN, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS VOLUMEDOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support VOLUMEDOWN KEY!!\n");
		}
		break;
	case RELEASE_VLDOWN_KEY:
		if (test_bit(KEY_VOLUMEDOWN, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE VOLUMEDOWN KEY!!\n");
			input_report_key(kpd_input_dev, KEY_VOLUMEDOWN, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support VOLUMEDOWN KEY!!\n");
		}
		break;
	case PRESS_FOCUS_KEY:
		if (test_bit(KEY_FOCUS, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS FOCUS KEY!!\n");
			input_report_key(kpd_input_dev, KEY_FOCUS, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support FOCUS KEY!!\n");
		}
		break;
	case RELEASE_FOCUS_KEY:
		if (test_bit(KEY_FOCUS, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE FOCUS KEY!!\n");
			input_report_key(kpd_input_dev, KEY_FOCUS, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support RELEASE KEY!!\n");
		}
		break;
	case PRESS_CAMERA_KEY:
		if (test_bit(KEY_CAMERA, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS CAMERA KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CAMERA, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support CAMERA KEY!!\n");
		}
		break;
	case RELEASE_CAMERA_KEY:
		if (test_bit(KEY_CAMERA, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE CAMERA KEY!!\n");
			input_report_key(kpd_input_dev, KEY_CAMERA, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support CAMERA KEY!!\n");
		}
		break;
	case PRESS_POWER_KEY:
		if (test_bit(KEY_POWER, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] PRESS POWER KEY!!\n");
			input_report_key(kpd_input_dev, KEY_POWER, 1);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support POWER KEY!!\n");
		}
		break;
	case RELEASE_POWER_KEY:
		if (test_bit(KEY_POWER, kpd_input_dev->keybit)) {
			kpd_print("[AUTOTEST] RELEASE POWER KEY!!\n");
			input_report_key(kpd_input_dev, KEY_POWER, 0);
			input_sync(kpd_input_dev);
		} else {
			kpd_print("[AUTOTEST] Not Support POWER KEY!!\n");
		}
		break;
#endif

	case SET_KPD_KCOL:
		kpd_auto_test_for_factorymode();	/* API 3 for kpd factory mode auto-test */
		kpd_print("[kpd_auto_test_for_factorymode] test performed!!\n");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2015/12/10, add for home key signal to fingerprint. */
#ifdef HOME_STATUS_SIGNAL

static int homekey_fasync(int fd, struct file * filp, int on) 
{
    return fasync_helper(fd, filp, on, &async_queue);
}

#endif
#endif /*VENDOR_EDIT*/


int kpd_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations kpd_dev_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = kpd_dev_ioctl,
	.open = kpd_dev_open,
#ifdef VENDOR_EDIT /* LiuPing@Phone.BSP.Sensor, 2015/12/10, add for home key signal to fingerprint */
#ifdef HOME_STATUS_SIGNAL    
    .fasync = homekey_fasync,    
#endif
#endif /*VENDOR_EDIT*/
};

/*********************************************************************/
static struct miscdevice kpd_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = KPD_NAME,
	.fops = &kpd_dev_fops,
};

static int kpd_open(struct input_dev *dev)
{
	kpd_slide_qwerty_init();	/* API 1 for kpd slide qwerty init settings */
	return 0;
}

#if !defined(CONFIG_MTK_LEGACY)
void kpd_get_dts_info(void)
{
	struct device_node *node;
	node = of_find_compatible_node(NULL, NULL, "mediatek, kpd");
	if (node) {
		of_property_read_u32(node, "kpd-key-debounce", &kpd_dts_data.kpd_key_debounce);
		of_property_read_u32(node, "kpd-sw-pwrkey", &kpd_dts_data.kpd_sw_pwrkey);
		of_property_read_u32(node, "kpd-hw-pwrkey", &kpd_dts_data.kpd_hw_pwrkey);
		of_property_read_u32(node, "kpd-sw-rstkey", &kpd_dts_data.kpd_sw_rstkey);
		of_property_read_u32(node, "kpd-hw-rstkey", &kpd_dts_data.kpd_hw_rstkey);
		of_property_read_u32(node, "kpd-use-extend-type", &kpd_dts_data.kpd_use_extend_type);
		of_property_read_u32(node, "kpd-pwrkey-eint-gpio", &kpd_dts_data.kpd_pwrkey_eint_gpio);
		of_property_read_u32(node, "kpd-pwrkey-gpio-din", &kpd_dts_data.kpd_pwrkey_gpio_din);
		of_property_read_u32(node, "kpd-hw-dl-key1", &kpd_dts_data.kpd_hw_dl_key1);
		of_property_read_u32(node, "kpd-hw-dl-key2", &kpd_dts_data.kpd_hw_dl_key2);
		of_property_read_u32(node, "kpd-hw-dl-key3", &kpd_dts_data.kpd_hw_dl_key3);
		of_property_read_u32(node, "kpd-hw-recovery-key", &kpd_dts_data.kpd_hw_recovery_key);
		of_property_read_u32(node, "kpd-hw-factory-key", &kpd_dts_data.kpd_hw_factory_key);
		of_property_read_u32_array(node, "kpd-hw-init-map", kpd_dts_data.kpd_hw_init_map,
					   ARRAY_SIZE(kpd_dts_data.kpd_hw_init_map));

		kpd_info
		    ("key-debounce = %d, sw-pwrkey = %d, hw-pwrkey = %d, hw-rstkey = %d, sw-rstkey = %d\n",
		     kpd_dts_data.kpd_key_debounce, kpd_dts_data.kpd_sw_pwrkey, kpd_dts_data.kpd_hw_pwrkey,
		     kpd_dts_data.kpd_hw_rstkey, kpd_dts_data.kpd_sw_rstkey);
	} else {
		kpd_info("[kpd]%s can't find compatible custom node\n", __func__);
	}
}
#endif


static int kpd_pdrv_probe(struct platform_device *pdev)
{

	int i, r;
	int err = 0;
	u32 intr[2] = {0,0};
	struct device_node *node = NULL;

	
	kpd_info("Keypad probe start!!!\n");
#ifdef CONFIG_OF
	kp_base = of_iomap(pdev->dev.of_node, 0);
	if (!kp_base) {
		kpd_info("KP iomap failed\n");
		return -ENODEV;
	};

	kp_irqnr = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!kp_irqnr) {
		kpd_info("KP get irqnr failed\n");
		return -ENODEV;
	}
	kpd_info("kp base: 0x%p, addr:0x%p,  kp irq: %d\n", kp_base, &kp_base, kp_irqnr);
#endif
#if defined(CONFIG_MTK_LEGACY)	/*This not need now */
#ifdef CONFIG_MTK_LDVT
	kpd_ldvt_test_init();	/* API 2 for kpd LFVT test enviroment settings */
#endif
#endif
	/* initialize and register input device (/dev/input/eventX) */
	kpd_input_dev = input_allocate_device();
	if (!kpd_input_dev)
		return -ENOMEM;

	kpd_input_dev->name = KPD_NAME;
	kpd_input_dev->id.bustype = BUS_HOST;
	kpd_input_dev->id.vendor = 0x2454;
	kpd_input_dev->id.product = 0x6500;
	kpd_input_dev->id.version = 0x0010;
	kpd_input_dev->open = kpd_open;
#if !defined(CONFIG_MTK_LEGACY)
	kpd_get_dts_info();
#endif
	/* fulfill custom settings */
	kpd_memory_setting();

	__set_bit(EV_KEY, kpd_input_dev->evbit);

#if (KPD_PWRKEY_USE_EINT || KPD_PWRKEY_USE_PMIC)
#if !defined(CONFIG_MTK_LEGACY)
	__set_bit(kpd_dts_data.kpd_sw_pwrkey, kpd_input_dev->keybit);
#else
	__set_bit(KPD_PWRKEY_MAP, kpd_input_dev->keybit);
#endif
	kpd_keymap[8] = 0;
#endif
#if !defined(CONFIG_MTK_LEGACY)
	if (!kpd_dts_data.kpd_use_extend_type) {
		for (i = 17; i < KPD_NUM_KEYS; i += 9)	/* only [8] works for Power key */
			kpd_keymap[i] = 0;
	}
#else
#if !KPD_USE_EXTEND_TYPE
	for (i = 17; i < KPD_NUM_KEYS; i += 9)	/* only [8] works for Power key */
		kpd_keymap[i] = 0;
#endif
#endif
	for (i = 0; i < KPD_NUM_KEYS; i++) {
		if (kpd_keymap[i] != 0)
			__set_bit(kpd_keymap[i], kpd_input_dev->keybit);
	}

#if KPD_AUTOTEST
	for (i = 0; i < ARRAY_SIZE(kpd_auto_keymap); i++)
		__set_bit(kpd_auto_keymap[i], kpd_input_dev->keybit);
#endif

#if KPD_HAS_SLIDE_QWERTY
	__set_bit(EV_SW, kpd_input_dev->evbit);
	__set_bit(SW_LID, kpd_input_dev->swbit);
#endif
#if !defined(CONFIG_MTK_LEGACY)
	if (kpd_dts_data.kpd_sw_rstkey)
		__set_bit(kpd_dts_data.kpd_sw_rstkey, kpd_input_dev->keybit);
#else
#ifdef KPD_PMIC_RSTKEY_MAP
	__set_bit(KPD_PMIC_RSTKEY_MAP, kpd_input_dev->keybit);
#endif
#endif
#ifdef KPD_KEY_MAP
	__set_bit(KPD_KEY_MAP, kpd_input_dev->keybit);
#endif

#ifdef CONFIG_MTK_MRDUMP_KEY
	__set_bit(KEY_RESTART, kpd_input_dev->keybit);
#endif

	kpd_input_dev->dev.parent = &pdev->dev;
	r = input_register_device(kpd_input_dev);
	if (r) {
		kpd_info("register input device failed (%d)\n", r);
		input_free_device(kpd_input_dev);
		return r;
	}

	/* register device (/dev/mt6575-kpd) */
	kpd_dev.parent = &pdev->dev;
	r = misc_register(&kpd_dev);
	if (r) {
		kpd_info("register device failed (%d)\n", r);
		input_unregister_device(kpd_input_dev);
		return r;
	}
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&kpd_suspend_lock, WAKE_LOCK_SUSPEND, "kpd wakelock");
#else
	wakeup_source_init(&kpd_suspend_lock, "kpd wakelock");
#endif

#ifndef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2016.01.29 for int
	/* register IRQ and EINT */
#if !defined(CONFIG_MTK_LEGACY)
	kpd_set_debounce(kpd_dts_data.kpd_key_debounce);
#else
	kpd_set_debounce(KPD_KEY_DEBOUNCE);
#endif
#ifdef CONFIG_OF
	r = request_irq(kp_irqnr, kpd_irq_handler, IRQF_TRIGGER_NONE, KPD_NAME, NULL);
#else
	r = request_irq(MT_KP_IRQ_ID, kpd_irq_handler, IRQF_TRIGGER_FALLING, KPD_NAME, NULL);
#endif
	if (r) {
		kpd_info("register IRQ failed (%d)\n", r);
		misc_deregister(&kpd_dev);
		input_unregister_device(kpd_input_dev);
		return r;
	}
#endif /*VENDOR_EDIT*/	

	mt_eint_register();

#ifndef KPD_EARLY_PORTING	/*add for avoid early porting build err the macro is defined in custom file */
	long_press_reboot_function_setting();	/* /API 4 for kpd long press reboot function setting */
#endif
	hrtimer_init(&aee_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aee_timer.function = aee_timer_func;

#if AEE_ENABLE_5_15
	hrtimer_init(&aee_timer_5s, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aee_timer_5s.function = aee_timer_5s_func;
#endif
	err = kpd_create_attr(&kpd_pdrv.driver);
	if (err) {
		kpd_info("create attr file fail\n");
		kpd_delete_attr(&kpd_pdrv.driver);
		return err;
	}
	kpd_info("%s Done\n", __func__);

#ifdef VENDOR_EDIT
#ifdef GPIO_HOME_PIN
//Modified by Tong.han for if there is GPIO_HOME_PIN.then compile the below code,2016-1-15
// Jingchun.Wang@Phone.Bsp.Driver, 2015/11/10  Add for for home key 
	node = of_find_compatible_node(NULL, NULL, "mediatek, HOME_KEY-eint");
	if(node){
		of_property_read_u32_array(node , "interrupts", intr, ARRAY_SIZE(intr));
		pr_err("kpd_pdrv_probe intr[0]  = %d, intr[1]  = %d\r\n",intr[0] ,intr[1] );
		home_irq = irq_of_parse_and_map(node, 0);
	}
	else{
		pr_err("kpd_pdrv_probe_irq_init node not exist!\r\n");
	}
	
	r = request_irq(home_irq, (irq_handler_t)kpd_home_eint, IRQF_TRIGGER_LOW, KPD_HOME_NAME,  NULL);
	mt_eint_dump_status(intr[0]);
	mt_eint_set_hw_debounce(intr[0], 32000);
	mt_eint_dump_status(intr[0]);
	
	__set_bit(KEY_HOME, kpd_input_dev->keybit);

#endif
#endif

	
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2016.01.29 for int 
		mt_set_gpio_mode(GPIO_VOLUMEKEY_UP, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_VOLUMEKEY_UP, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_VOLUMEKEY_UP, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_VOLUMEKEY_UP, GPIO_PULL_UP);
			
		mt_set_gpio_mode(GPIO_VOLUMEKEY_DOWN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_VOLUMEKEY_DOWN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_VOLUMEKEY_DOWN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_VOLUMEKEY_DOWN, GPIO_PULL_UP);
		
		vol_up_irq = mt_gpio_to_irq(GPIO_VOLUMEKEY_UP);
		vol_down_irq = mt_gpio_to_irq(GPIO_VOLUMEKEY_DOWN);

		request_irq(vol_up_irq, (irq_handler_t)kpd_volumekey_up_eint, IRQF_TRIGGER_LOW, KPD_VOL_UP_NAME,	NULL);
		mt_eint_set_sens(EINT_VOLUMEKEY_UP_NUM, CUST_EINT_LEVEL_SENSITIVE);
		mt_eint_set_polarity(EINT_VOLUMEKEY_UP_NUM, CUST_EINT_POLARITY_LOW);

		mt_eint_dump_status(GPIO_VOLUMEKEY_UP);
		mt_eint_set_hw_debounce(GPIO_VOLUMEKEY_UP,32000);
		mt_eint_dump_status(GPIO_VOLUMEKEY_UP);
		
		/*mt_eint_registration(EINT_VOLUMEKEY_UP_NUM, 
					CUST_EINTF_TRIGGER_LOW,
					kpd_volumekey_up_eint,
					false);*/
					
		request_irq(vol_down_irq, (irq_handler_t)kpd_volumekey_down_eint, IRQF_TRIGGER_LOW, KPD_VOL_DOWN_NAME, NULL);
		mt_eint_set_sens(EINT_VOLUMEKEY_DOWN_NUM, CUST_EINT_LEVEL_SENSITIVE);
		mt_eint_set_polarity(EINT_VOLUMEKEY_DOWN_NUM, CUST_EINT_POLARITY_LOW);
		mt_eint_dump_status(GPIO_VOLUMEKEY_DOWN);
		mt_eint_set_hw_debounce(GPIO_VOLUMEKEY_DOWN, 32000);
		mt_eint_dump_status(GPIO_VOLUMEKEY_DOWN);
		
		/*mt_eint_registration(EINT_VOLUMEKEY_DOWN_NUM, 
					CUST_EINTF_TRIGGER_LOW,
					kpd_volumekey_down_eint,
					false);*/
	
		enable_irq(vol_up_irq);
		enable_irq(vol_down_irq);
		//mt_eint_unmask(EINT_VOLUMEKEY_UP_NUM);
		//mt_eint_unmask(EINT_VOLUMEKEY_DOWN_NUM);		 
		__set_bit(KEY_VOLUMEDOWN, kpd_input_dev->keybit);
		__set_bit(KEY_VOLUMEUP, kpd_input_dev->keybit);
		__set_bit(KEY_POWER, kpd_input_dev->keybit);
#endif/*VENDOR_EDIT*/  
	return 0;
}

/* should never be called */
static int kpd_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

static int kpd_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	kpd_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		kpd_print("kpd_pdrv_suspend wake up source enable!! (%d)\n", kpd_suspend);
	} else {
		kpd_wakeup_src_setting(0);
		kpd_print("kpd_pdrv_suspend wake up source disable!! (%d)\n", kpd_suspend);
	}
#endif
	kpd_disable_backlight();
	kpd_print("suspend!! (%d)\n", kpd_suspend);
	return 0;
}

static int kpd_pdrv_resume(struct platform_device *pdev)
{
	kpd_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		kpd_print("kpd_pdrv_suspend wake up source enable!! (%d)\n", kpd_suspend);
	} else {
		kpd_print("kpd_pdrv_suspend wake up source resume!! (%d)\n", kpd_suspend);
		kpd_wakeup_src_setting(1);
	}
#endif
	kpd_print("resume!! (%d)\n", kpd_suspend);
	return 0;
}

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
static struct sb_handler kpd_sb_handler_desc = {
	.level = SB_LEVEL_DISABLE_KEYPAD,
	.plug_in = sb_kpd_enable,
	.plug_out = sb_kpd_disable,
};
#endif
#endif

static int __init kpd_mod_init(void)
{
	int r;

	r = platform_driver_register(&kpd_pdrv);
	if (r) {
		kpd_info("register driver failed (%d)\n", r);
		return r;
	}

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
	register_sb_handler(&kpd_sb_handler_desc);
#endif
#endif

	return 0;
}

/* should never be called */
static void __exit kpd_mod_exit(void)
{
}

module_init(kpd_mod_init);
module_exit(kpd_mod_exit);

module_param(kpd_show_hw_keycode, int, 0644);
module_param(kpd_show_register, int, 0644);

MODULE_AUTHOR("yucong.xiong <yucong.xiong@mediatek.com>");
MODULE_DESCRIPTION("MTK Keypad (KPD) Driver v0.4");
MODULE_LICENSE("GPL");
