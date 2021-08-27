#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#include "kd_flashlight.h"
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>



/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5
     11    12       13      14       15    16
1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif
#define GPIO_LED_EN  GPIO_TORCH_EN//(GPIO21 | 0x80000000)


/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int gDuty;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);





static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */




static int gIsTorch[18] = { 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static int gLedDuty[18] = { 0, 32, 48, 64, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };//{ 0, 32, 64, 96, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

/* current(mA) 50,141,187,234,281,375,469,563,656,750,844,938,1031,1125,1220,1313,1406,1500 */



/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

#ifdef VENDOR_EDIT
/*oppo hufeng 20151112 add for lm3642 driverIC*/
/******************************************************************************
 * local variables
******************************************************************************/
static struct i2c_client *LM3642_i2c_client;
#ifdef VENDOR_EDIT
#define LM3642_NAME "leds-LM3642"
#define LM3642_I2C_BUSNUM 2
#define I2C_REGISTER_ID            0x63
#define LM3642_I2C_ADDR           0x63
static struct i2c_board_info kd_flashligh_dev __initdata = {
	I2C_BOARD_INFO(LM3642_NAME, I2C_REGISTER_ID)
};
#endif



struct LM3642_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct LM3642_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct LM3642_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int LM3642_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int LM3642_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}



/* ========================= */




static int LM3642_chip_init(struct LM3642_chip_data *chip)
{


	return 0;
}

static int LM3642_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct LM3642_chip_data *chip;
	struct LM3642_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3642_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_ERR("LM3642 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3642_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_ERR("LM3642 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3642_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (LM3642_chip_init(chip) < 0)
		goto err_chip_init;

	LM3642_i2c_client = client;
#ifdef VENDOR_EDIT
	LM3642_i2c_client->addr = LM3642_I2C_ADDR;
#endif
	PK_DBG("LM3642 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_ERR("LM3642 probe is failed\n");
	return -ENODEV;
}

static int LM3642_remove(struct i2c_client *client)
{
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


static const struct i2c_device_id LM3642_id[] = {
	{LM3642_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id LM3642_of_match[] = {
	{.compatible = "mediatek,STROBE_MAIN"},
	{},
};
#endif

static struct i2c_driver LM3642_i2c_driver = {
	.driver = {
		   .name = LM3642_NAME,
#ifdef CONFIG_OF
		   .of_match_table = LM3642_of_match,
#endif
		   },
	.probe = LM3642_probe,
	.remove = LM3642_remove,
	.id_table = LM3642_id,
};

static int __init LM3642_init(void)
{
	PK_DBG("LM3642_init\n");
	#ifdef VENDOR_EDIT
	i2c_register_board_info(LM3642_I2C_BUSNUM, &kd_flashligh_dev, 1);
	#endif

	return i2c_add_driver(&LM3642_i2c_driver);
}

static void __exit LM3642_exit(void)
{
	i2c_del_driver(&LM3642_i2c_driver);
}


module_init(LM3642_init);
module_exit(LM3642_exit);

MODULE_DESCRIPTION("Flash driver for LM3642");
MODULE_AUTHOR("pw <hufeng@oppo.com>");
MODULE_LICENSE("GPL v2");
#endif
int LM3642_readReg(int reg)
{

	int val = 0xff;
	val = LM3642_read_reg(LM3642_i2c_client, reg);
	PK_DBG(" LM3642_read_reg reg 0x%x=0x%x \n", reg, val);
	return (int)val;
}


int FL_Enable(void)
{
	int buf[2];
	int temp1,temp2;
		buf[0] = 0x0a;
	if (gIsTorch[gDuty] == 1)
		buf[1] = 0x22;
	else
		buf[1] = 0x23;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	PK_DBG(" FL_Enable line=%d gDuty = 0x%x\n", __LINE__, gDuty);
	temp1 = LM3642_readReg(0x0a);
	temp2 = LM3642_readReg(0x0b);
	PK_DBG(" FL_dim_duty line=%d, enable value is 0x%x,flags value is 0x%x\n", __LINE__, temp1,temp2);
	return 0;
}



int FL_Disable(void)
{
	int buf[2];
	int temp1,temp2;
	buf[0] = 0x0a;
	buf[1] = 0x00;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	temp1 = LM3642_readReg(0x0a);
	temp2 = LM3642_readReg(0x0b);
	PK_DBG(" FL_dim_duty line=%d, enable value is 0x%x,flags value is 0x%x\n", __LINE__, temp1,temp2);
	/*PK_DBG(" FL_Disable line=%d\n", __LINE__);*/
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	int buf[2];
	int temp1,temp2;
	if (duty > 12)
		duty = 12;/*change 17 to 12 to limit max current to 1A */
	if (duty < 0)
		duty = 0;
	gDuty = duty;
	buf[0] = 9;
	buf[1] = gLedDuty[duty];
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	PK_DBG(" FL_dim_duty line=%d, current value is 0x%x\n", __LINE__, buf[1]);
	temp1 = LM3642_readReg(0x09);
	temp2 = LM3642_readReg(0x0b);
	PK_DBG(" FL_dim_duty line=%d, duty value is 0x%x,flags value is 0x%x\n", __LINE__, temp1,temp2);
	return 0;
}




int FL_Init(void)
{
	int buf[2];
    if(mt_set_gpio_mode(GPIO_LED_EN,GPIO_MODE_00)){PK_DBG(" set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_LED_EN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_LED_EN,1/*GPIO_OUT_ONE*/)){PK_DBG(" set gpio failed!! \n");}
	buf[0] = 0x0a;
	buf[1] = 0x00;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0] = 0x08;
	buf[1] = 0x04;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0] = 0x09;
	buf[1] = 0x1A;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	return 0;
}
#ifdef VENDOR_EDIT
/*oppo hufeng 20160615 add for checkt flashlight alive*/
int FL_check_alive()
{
	int temp = 0xff;
	int ret = 0xff;
	int buf[2];
	buf[0] = 0x0a;
	buf[1] = 0x22;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	temp = LM3642_readReg(0x0a);
	PK_DBG(" FL_check_alive line=%d, temp 0x%x\n", __LINE__, temp);
	ret = (temp==0x22)?1:0;
	return ret;
}
#endif

int FL_Uninit(void)
{
	FL_Disable();
    mt_set_gpio_mode(GPIO_LED_EN, GPIO_MODE_00);   
    mt_set_gpio_dir(GPIO_LED_EN,GPIO_DIR_OUT);  
    mt_set_gpio_out(GPIO_LED_EN, 0);
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
	/* printk(KERN_ALERT "work handler function./n"); */
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;
	if (init_flag==0){
		init_flag=1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
	/* PK_DBG
	    ("LM3642 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg); */
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;
#ifdef VENDOR_EDIT
/*oppo hufeng 20160615 add for checkt flashlight alive*/
	case FLASH_IOC_CHECK_ALIVE:
		PK_DBG("FLASH_IOC_CHECK_ALIVE: %d\n", (int)arg);
		i4RetValue = FL_check_alive();
		break;
#endif

	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;
				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_ERR(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
