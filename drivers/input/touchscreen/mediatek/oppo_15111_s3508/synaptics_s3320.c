/************************************************************************************
 ** File: - mediatek\custom\common\kernel\touchpanel\oppo\synaptics_14053.c
 ** VENDOR_EDIT
 ** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
 ** 
 ** Description: 
 **      touch panel driver for synaptics
 **      can change MAX_POINT_NUM value to support multipoint
 ** Version: 1.0
 ** Date created: 10:49:46,14/05/2012
 ** Author: Yixue.Ge@BasicDrv.TP
 ** 
 ** --------------------------- Revision History: --------------------------------
 ** 	<author>	<data>			<desc>
 ** Yongjun.Wu@BasicDrv.TP 18/05/2012 add some 12021 code
 ** Yongjun.Wu@BasicDrv.TP 19/05/2012 add Virtual key 
 ** Yongjun.Wu@BasicDrv.TP 21/05/2012 modify for Calibration
 ** Yongjun.Wu@BasicDrv.TP 08/06/2012 add firmware update
 ** Yongjun.Wu@BasicDrv.TP 12/06/2012 add Auto test
 ** Yongjun.WU@BasicDrv.TP 14/06/2012 modify for Auto test
 ************************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h> 
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/oppo_devices_list.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <mach/eint.h>
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <linux/kthread.h>

#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <mach/eint.h>
#include <cust_eint.h>

#include <linux/rtpm_prio.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#ifndef TPD_NO_GPIO 
#include "cust_gpio_usage.h"
#endif

#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <soc/oppo/device_info.h>
#include <soc/oppo/oppo_project.h>


/*------------------------------------------------Global Define--------------------------------------------*/
#define PAGESIZE 1024
#define VKNUMBER 3
#define TPD_USE_EINT

#define TPD_DEVICE "mtk-tpd-s3320"

#define SUPPORT_GESTURE
#define RESET_ONESECOND
#define SUPPORT_REPORT_COORDINATE
//#define SUPPORT_GLOVES_MODE   
#define SUPPORT_TP_SLEEP_MODE
#define DMA_SUPPORT
#define TYPE_B_PROTOCOL      //Multi-finger operation  B(Slot)  need trac id
#define TP_FW_NAME_MAX_LEN 64


//chenggang.li@BSP add for mt6752
/******************start*******************/    
#define GTP_SUPPORT_I2C_DMA   			1
#define I2C_MASTER_CLOCK                350

#define GTP_DMA_MAX_TRANSACTION_LENGTH  255       // for DMA mode



//DEFINE_MUTEX(tp_wr_access);

/******************end*******************/  

//mingqiang.guo add 
static unsigned int boot_mode = 1;
#define MAX_POINT_NUM      10//must <=10
//end

#ifdef SUPPORT_GESTURE
static atomic_t double_enable;
#define KEY_POWER 116
static int is_gesture_enable = 0;
#endif

#ifdef SUPPORT_REPORT_COORDINATE
#define LCD_WIDTH 1080
#define LCD_HEIGHT 1920
struct Coordinate {
	uint32_t x;
	uint32_t y;
};
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
static uint32_t clockwise;
static uint32_t gesture;
#endif
#ifdef DMA_SUPPORT
static int i2c_dma_write(const struct i2c_client *client, u8 addr, s32 len, u8 *txbuf);
static int i2c_dma_read(const struct i2c_client *client, u8 addr, s32 len, u8 *txbuf); 

static u8 *gprDMABuf_va = NULL;
static dma_addr_t gprDMABuf_pa = 0;

static u8 *gpwDMABuf_va = NULL;
static dma_addr_t gpwDMABuf_pa = 0;
#endif

#define CUST_EINT_TOUCH_PANEL_NUM 1

/******************for Red function*****************/
#define CONFIG_SYNAPTIC_RED
#ifdef CONFIG_SYNAPTIC_RED
#include "synaptics_redremote.h"
#endif 

/*********************for gesture*******************/
#ifdef SUPPORT_GESTURE
#define ENABLE_UNICODE  0x40
#define ENABLE_VEE      0x20
#define ENABLE_CIRCLE   0x08
#define ENABLE_SWIPE    0x02
#define ENABLE_DTAP     0x01

#define UNICODE_DETECT  0x0b
#define VEE_DETECT      0x0a
#define CIRCLE_DETECT   0x08
#define SWIPE_DETECT    0x07
#define DTAP_DETECT     0x03

// SUPPORT_GESTURE for S3203  
#define UNICODE_DETECT_S3203  0x40
#define VEE_DETECT_S3203      0x20
#define CIRCLE_DETECT_S3203   0x08
#define SWIPE_DETECT_S3203    0x02
#define DTAP_DETECT_S3203     0x01


#define UnkownGestrue       0
#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define LeftVee             4   // >
#define RightVee            5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W
#endif

/*********************for Debug LOG switch*******************/
#define TPD_ERR(a, arg...)  pr_err(TPD_DEVICE ": " a, ##arg)	
#define TPDTM_DMESG(a, arg...)  printk(TPD_DEVICE ": " a, ##arg)

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug == 1)\
		pr_err(TPD_DEVICE ": " a,##arg);\
	}while(0)	

/*---------------------------------------------Global Variable----------------------------------------------*/
static int Dot_report_down = 0;

static int baseline_ret = 0;

//static int tp_dev = 2;
static unsigned int tp_debug = 0;
static unsigned int is_suspend = 0;		//indicate the IC is in sleep mode
static int16_t delta_baseline[30][30];	
//static int report_key_point_y = 0;
static int force_update = 0;
static int tp_probe_ok = 0;
static int limit_enable = 1;
static int limit_area = 1;
static int left_x1, left_x2;
static int right_x1, right_x2;
static int in_which_area = 0;


static unsigned int touch_irq = 0;
static int finish_suspend = 0;


//15011 *#808# struct manufacture_info tp_info;
static struct synaptics_ts_data *ts_g=NULL;
static struct workqueue_struct *synaptics_wq = NULL;
static struct workqueue_struct *speedup_resume_wq = NULL;

#ifdef SUPPORT_TP_SLEEP_MODE
static atomic_t sleep_enable;
#endif

#ifdef SUPPORT_GLOVES_MODE	
static atomic_t glove_enable;
#endif 

//mingqiang.guo for charge pulg in ,open tp Finger Amplitude Thre and Finger Dbounce ,avoid charge disturb
static atomic_t charge_plug_in_flag;
//end

static int is_key_touch;
extern int is_lcd_touch;

//#define SUPPORT_LEATHER //mingqiang.guo@phone.bsp add for leather cover 
#ifdef SUPPORT_LEATHER
atomic_t leather_enable_flag;
atomic_t hall_close_flag;
#endif 

#ifdef SUPPORT_GESTURE
static uint32_t clockwise;
static uint32_t gesture;

static uint32_t gesture_upload;
static int gesture_enable = 0;		//indicate the ts suspend is over

//mingqiang.guo@phone.bsp 2015-5-22 add for get lcd modle id 
static int lcd_id;
//end
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
#endif

/*-----------------------------------------Global Registers----------------------------------------------*/
static unsigned short SynaF34DataBase;
static unsigned short SynaF34QueryBase;
static unsigned short SynaF01DataBase;
static unsigned short SynaF01CommandBase;

static unsigned short SynaF34Reflash_BlockNum;
static unsigned short SynaF34Reflash_BlockData;
static unsigned short SynaF34ReflashQuery_BootID;
static unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
static unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
static unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
static unsigned short SynaF34ReflashQuery_ConfigBlockSize;
static unsigned short SynaF34ReflashQuery_ConfigBlockCount;

static unsigned short SynaFirmwareBlockSize;
static unsigned short SynaF34_FlashControl;

static int F12_2D_QUERY_BASE;
static int F12_2D_CMD_BASE;
static int F12_2D_CTRL_BASE;
static int F12_2D_DATA_BASE;

static int F01_RMI_QUERY_BASE;
static int F01_RMI_CMD_BASE;
static int F01_RMI_CTRL_BASE;
static int F01_RMI_DATA_BASE;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F51_CUSTOM_QUERY_BASE;
static int F51_CUSTOM_CMD_BASE;
static int F51_CUSTOM_CTRL_BASE;
static int F51_CUSTOM_CTRL31;
static int F51_CUSTOM_DATA_BASE;

static int F01_RMI_QUERY11;
static int F01_RMI_DATA01;
static int F01_RMI_CMD00;
static int F01_RMI_CTRL00;
static int F01_RMI_CTRL01;
static int F01_RMI_CTRL02;// mignqiang.guo 2015/4/19 add for status bar pull down 

static int F12_2D_CTRL08;
static int F12_2D_CTRL23;
static int F12_2D_CTRL32;
static int F12_2D_DATA38;
static int F12_2D_DATA39;
static int F12_2D_CMD00;
static int F34_FLASH_CTRL00;

static int F51_CUSTOM_CTRL00;
static int F51_CUSTOM_DATA04;

//#if TP_TEST_ENABLE
static int F54_ANALOG_QUERY_BASE;//0x73
static int F54_ANALOG_COMMAND_BASE;//0x72
static int F54_ANALOG_CONTROL_BASE;//0x0d
static int F54_ANALOG_DATA_BASE;//0x00
//#endif

/***********for example of key event***********/
#ifdef KEY_USE
static int tpd_keys[VKNUMBER][5] = {
	{KEY_MENU, 90, 2050, 180, 100},
	{KEY_HOME, 500, 2050, 180, 100},
	{KEY_BACK, 855, 2050, 180, 100},
};
#endif


extern int TP_FW;
extern TP_DEV oppo_tp_dev;
static struct manufacture_info tp_info;

/*------------------------------------------Fuction Declare----------------------------------------------*/
static unsigned int all_finger_up = 0;
static int synaptics_ts_resume(struct i2c_client *client);
static int synaptics_ts_suspend(struct i2c_client *client);
static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int synaptics_tpd_button_init(struct synaptics_ts_data *ts);
static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force);
#ifdef SUPPORT_GESTURE
static int synaptics_enable_interrupt_for_gesture(struct synaptics_ts_data *ts, int enable);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif
static void speedup_synaptics_resume(struct work_struct *work);
extern void judge_key_press_all_the_time(void);
static void hard_reset_tp(struct synaptics_ts_data *ts);
int is_gesture_enable_for_lcd(void);




/*-------------------------------Using Struct----------------------------------*/
struct point_info {
	int x;
	int raw_x;
	int y;
	int raw_y;
	int z;
	int status; //mingqiang.guo@phone.bsp.tp  add for auto report (0,0)
};





struct synaptics_ts_data {
	struct mutex mutex;
	int irq;
	int irq_gpio;
	int id1_gpio;
	int id2_gpio;
	int id3_gpio;
	int reset_gpio;
	int enable2v8_gpio;
	int max_num;
	int enable_remote;
	uint32_t irq_flags;
	uint32_t max_x;
	uint32_t max_y;
	uint32_t max_y_real;
	uint32_t btn_state;
	uint32_t pre_finger_state;
	uint32_t pre_btn_state;
	struct input_dev *kpd;
	struct work_struct  work;
	struct work_struct speed_up_work;
	struct work_struct charge_write_tp_work;//mingqiang.guo 2015/8/15 add for charge driver write tp reg
	struct work_struct close_edge_product;
	//from qcom end 

	struct i2c_client *client;
	struct input_dev *input_dev;

	int use_irq;
	struct hrtimer timer;
	struct hrtimer report_timer;
	/******gesture*******/
	int i2c_suspend;
	int double_enable;
	int glove_enable;

	/********test*******/
	int i2c_device_test;

	uint32_t flags;

	struct device						*dev;
	uint32_t no_erroponit_exist;
	uint32_t finger_num;
	struct early_suspend early_suspend;
	/*******for FW update*******/
	bool suspended;
	bool loading_fw;
	char fw_name[TP_FW_NAME_MAX_LEN];
	char test_limit_name[TP_FW_NAME_MAX_LEN];
	char fw_id[12];
	char manu_name[12];
};

/* ========================================= */
/* implementation of  virtual key attribute */
/* ========================================= */
static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf){
	/* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
	return sprintf(buf,
			__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":180:2021:190:152"
			":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":540:2021:190:152"
			":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":900:2021:190:152"		 
			"\n");


}

//static int synaptics_probe(struct platform_device *pdev);


#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2
struct image_header {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_contain_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char reserved_20_2f[16];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	unsigned char ds_info[10];
	unsigned char reserved_4a_4f[6];
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

struct test_header {
	unsigned int magic1;
	unsigned int magic2;
	unsigned int withCBC;
	unsigned int array_limit_offset;
	unsigned int array_limit_size;
	unsigned int array_limitcbc_offset;
	unsigned int array_limitcbc_size;
};

static unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
		(unsigned int)ptr[1] * 0x100 +
		(unsigned int)ptr[2] * 0x10000 +
		(unsigned int)ptr[3] * 0x1000000;
}



static s32 i2c_smbus_write_byte_data_s3508(const struct i2c_client *client, u8 command, u8 value)
{
	int ret=0;

	u8 write_dyte_data = value;

	ret = i2c_dma_write(client, command, 1,&write_dyte_data); 

	return ret;
}

static s32 i2c_smbus_read_byte_data_s3508(const struct i2c_client *client, u8 command)
{
	int ret=0;

	u8 read_dyte_data ;

	ret = i2c_dma_read(client, command, 1,&read_dyte_data); 
	if (ret < 0)
		TPD_ERR("%s  error \n",__func__);
	//else
	//	TPD_DEBUG("%s  success \n",__func__);

	return read_dyte_data;

}


static s32 i2c_smbus_write_word_data_s3508(const struct i2c_client *client, u8 command,u16 value)
{
	int ret=0;

	u8 data_buf[2] ;
	data_buf[0] = value&0x00ff;
	data_buf[1] = value>>8;

	ret = i2c_dma_write(client, command, 2,data_buf); 
		
	return ret;

}

static int synaptics_rmi4_set_page(const struct i2c_client *client, u8 command, u8 value)
{
 	int ret=0;

	u8 write_dyte_data = value;

	ret = i2c_dma_write(client, command, 1,&write_dyte_data); 

	return ret;
}


//mingqiang.guo for charge pulg in ,open tp Finger Amplitude Thre and Finger Dbounce ,avoid charge disturb
static int charge_plug_status = 0 ;//0: no   1: slow  3:  quickly charge
#if 0
static void write_back_reg_status(struct synaptics_ts_data *ts)
{	
	int ret;

	TPD_ERR("%s, write back charge status  page 4  F51_CUSTOM_CTRL31=0x%x write  charge_plug_status=%d	\n",__func__,F51_CUSTOM_CTRL31,charge_plug_status);//0x0425
	if(ts==NULL)
	{
		TPD_ERR("ts==NULL\n");
		return;
	}
#ifdef SUPPORT_GESTURE
	if( (1 == atomic_read(&double_enable) ) && (gesture_enable == 1 ) ) //when suspend gesture_enable == 1 ; when gesture_enable==1 , write tp black gesture enable 
	{
		TPD_ERR(" write_back_reg_status on-off : gesture_enable == %d \n", gesture_enable);
		switch ( is_gesture_enable )
		{
			case 0:
				TPD_ERR("tp_guesture_func will be disable\n");
				ret = synaptics_enable_interrupt_for_gesture(ts_g, 0); 
				if( ret<0 )
					ret = synaptics_enable_interrupt_for_gesture(ts_g, 0); 
				ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CTRL00, 0x01); 
				if( ret < 0 ){
					TPD_ERR("write F01_RMI_CTRL00 failed\n");
				}
				break;
			case 1:
				TPD_ERR("tp_guesture_func will be enable\n");
				ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CTRL00, 0x80); 
				if( ret < 0 ){
					TPD_ERR("write F01_RMI_CTRL00 failed\n");
				}
				ret = synaptics_enable_interrupt_for_gesture(ts_g, 1); 
				if( ret<0 )
					ret = synaptics_enable_interrupt_for_gesture(ts_g, 1); 
				break;
			default:
				TPD_ERR("Please enter 0 or 1 to open or close the double-tap function\n");
		}
		
	}
#endif 

	ret = i2c_smbus_write_byte_data_s3508(ts->client, 0xff, 4);  
	if( ret < 0 ){
		TPD_ERR("%s i2c error\n",__func__);
		goto OUT;
	}

	ret = i2c_smbus_write_byte_data_s3508(ts->client, F51_CUSTOM_CTRL31&0x00ff, charge_plug_status );  
	if( ret < 0 ){
		TPD_ERR("%s i2c error\n",__func__);
		goto OUT;
	}
	
OUT :
	synaptics_rmi4_set_page(ts->client, 0xff, 0x0); 
}
#endif

int is_gesture_enable_for_lcd(void)
{
	int support_gesture_for_lcd = 0;
	
	#ifdef SUPPORT_GESTURE
		if( (1 == atomic_read(&double_enable)) ||(2 == atomic_read(&double_enable))  )
		{
			support_gesture_for_lcd = 1;
		}
		else
		{
			support_gesture_for_lcd = 0;
		}
	#else
		support_gesture_for_lcd = 0;
	#endif
	return support_gesture_for_lcd;
}



static void tpd_hw_pwron(void)
{
	/* hwPowerOn(MT6331_POWER_LDO_VGP1,VOL_2800,"CTP");

	   msleep(1); */
#if 0
	mt_set_gpio_mode(79, 0); //3v3
	mt_set_gpio_dir(79, 1);
	mt_set_gpio_out(79,1);
#endif
	msleep(1);
	//Wanghao@BSP, 2015/09/22, modify for power in mt6755
	mt_set_gpio_mode(GPIO_CTP_OPEN_PIN, 0); //3v3
	mt_set_gpio_dir(GPIO_CTP_OPEN_PIN, 1);
	mt_set_gpio_out(GPIO_CTP_OPEN_PIN, 1);
	//VIO28 may be has been powered by the system,if this, we not need power it
	//hwPowerOn(MT6331_POWER_LDO_VGP1,VOL_3000,"CTP");//i2c 1v8
	msleep(250);
}

static void tpd_hw_pwron_nosleep(void)
{
	//Wanghao@BSP, 2015/09/22, modify for power in mt6755
	mt_set_gpio_mode(GPIO_CTP_OPEN_PIN, 0); //3v3
	mt_set_gpio_dir(GPIO_CTP_OPEN_PIN, 1);
	mt_set_gpio_out(GPIO_CTP_OPEN_PIN, 1);
}



static void tpd_hw_pwroff(void)
{
	/*
	   For gesture we not need power down   
	   */
#if 0
	//hwPowerDown(MT6331_POWER_LDO_VGP4,"CTP");
	tpd_hw_pwrdown();
	msleep(1);
	//hwPowerDown(MT6331_POWER_LDO_VGP1,"CTP");
#endif

}


static void tpd_power(unsigned int on)
{
	if(on)
	{
		tpd_hw_pwron();
	}
	else
	{
		tpd_hw_pwroff();
	}
}

static int synaptics_soft_reset(struct synaptics_ts_data *ts)
{
	int ret = 0;

	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00);
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CMD_BASE, 0x01);//software reset TP
	if(ret < 0)
	{
		TPD_ERR("%s error ret=%d\n",__func__,ret);
	}
	msleep(20);
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x04);
	if(0 == limit_enable)
	{
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F51_CUSTOM_CTRL_BASE + 0x09, 0x46);
	}
	else if(1 == limit_enable)
	{
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F51_CUSTOM_CTRL_BASE + 0x09, 0x47);
	}
	synaptics_rmi4_set_page(ts_g->client, 0xff, 0x0); 
	TPD_ERR("%s !!!\n",__func__);
	
	return ret;
}

static void hard_reset_tp(struct synaptics_ts_data *ts)
{
	TPD_ERR("  i2c  error  %s \n",__func__);
	
	if(ts==NULL)
	{
		TPD_ERR("ts_g==NULL\n");
		return;
	}

//mingqiang.guo add for : can not reset when tp firmware update now 
	if(ts->loading_fw == true)
	{
		TPD_ERR(" tp firmware update now,can not reset\n ");
		return;
	}
//end 	

	//no use page_address = i2c_smbus_read_byte_data_s3508(ts_g->client, 0xff);
	//TPD_ERR(" read  page address is %d\n",page_address)	;

	//hwPowerDown(MT6331_POWER_LDO_VGP1,"CTP");//i2c 1v8
	tpd_hw_pwroff();
	
/* rst pin is NC	
	mt_set_gpio_mode(GPIO123, 0);  
  	mt_set_gpio_dir(GPIO123, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO123, GPIO_PULL_ENABLE);
  	mt_set_gpio_pull_select(GPIO123, GPIO_PULL_UP);
	mt_set_gpio_out(GPIO123, 0);
*/	
	msleep(200);
	
	//hwPowerOn(MT6331_POWER_LDO_VGP1,VOL_3000,"CTP");//i2c 1v8
	tpd_hw_pwron();
	
//rest pin is NC  mt_set_gpio_out(GPIO123, 1);
	msleep(200);

	//write_back_reg_status(ts);
	
  //no use synaptics_rmi4_set_page(ts_g->client, 0xff, page_address); //write page  address
	
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = extract_uint_le(data->checksum);
	TPD_DEBUG(" debug checksume is %x", header->checksum);
	header->bootloader_version = data->bootloader_version;
	TPD_DEBUG(" debug bootloader_version is %d\n", header->bootloader_version);

	header->firmware_size = extract_uint_le(data->firmware_size);
	TPD_DEBUG(" debug firmware_size is %x", header->firmware_size);

	header->config_size = extract_uint_le(data->config_size);
	TPD_DEBUG(" debug header->config_size is %x", header->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	TPD_DEBUG(" debug header->contains_firmware_id is %x\n", header->contains_firmware_id);
	if( header->contains_firmware_id )
		header->firmware_id = extract_uint_le(data->firmware_id);

	return;
}


#define TPD_USE_EINT

#ifdef TPD_USE_EINT
#define TOUCHPANEL_INT_PIN GPIO1
#endif

#define TPD_USE_RESET

#ifdef TPD_USE_RESET  
#define TOUCHPANEL_RESET_PIN 123
#endif

#define TP_ID_DETECT GPIO54 //0:Truly;1:TPK
#define TP_TRULY_SYNAPTICS_ID 0
#define TP_TPK_ID 1
#define TP_OFILM_ID 2
static int pre_touch[MAX_POINT_NUM];
static int synaptics_rmi4_free_fingers(struct synaptics_ts_data *ts)
{
	unsigned char ii;

#ifdef TYPE_B_PROTOCOL
	for (ii = 0; ii < ts->max_num; ii++) {
		input_mt_slot(ts->input_dev, ii);
		input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev,BTN_TOUCH, 0);
	input_report_key(ts->input_dev,BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);
#else 
	input_report_key(ts->input_dev,BTN_TOUCH, 0);
	input_report_key(ts->input_dev,BTN_TOOL_FINGER, 0);
	input_mt_sync(ts->input_dev);
	input_sync(ts->input_dev);
#endif
	//free menu and home and back key;
	input_report_key(ts->input_dev, KEY_MENU, 0);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, KEY_BACK, 0);
	input_sync(ts->input_dev);
	input_report_key(ts->input_dev, BTN_TOUCH, 0);

	is_key_touch = 0;
	is_lcd_touch = 0;

	return 0;
}


static int synaptics_read_register_map(struct synaptics_ts_data *ts)
{
	uint8_t buf[4];   
	int ret;
	memset(buf, 0, sizeof(buf));
	ret = i2c_smbus_write_byte_data_s3508( ts->client, 0xff, 0x0 ); 
	if( ret < 0 ){
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = i2c_dma_read(ts->client, 0xDD, 4, &(buf[0x0]));
	if( ret < 0 ){
		TPD_ERR("failed for page select!\n");
		return -1;
	}
	F12_2D_QUERY_BASE = buf[0];
	F12_2D_CMD_BASE = buf[1];
	F12_2D_CTRL_BASE = buf[2]; 
	F12_2D_DATA_BASE = buf[3];

	TPD_ERR("F12_2D_QUERY_BASE = %x \n \
			F12_2D_CMD_BASE   = %x \n\
			F12_2D_CTRL_BASE  = %x \n\
			F12_2D_DATA_BASE  = %x \n\
			",F12_2D_QUERY_BASE,F12_2D_CMD_BASE,F12_2D_CTRL_BASE,F12_2D_DATA_BASE);

	ret = i2c_dma_read(ts->client, 0xE3, 4, &(buf[0x0]));    
	F01_RMI_QUERY_BASE = buf[0];
	F01_RMI_CMD_BASE = buf[1];
	F01_RMI_CTRL_BASE = buf[2]; 
	F01_RMI_DATA_BASE = buf[3];
	TPD_DEBUG("F01_RMI_QUERY_BASE = %x \n\
			F01_RMI_CMD_BASE  = %x \n\
			F01_RMI_CTRL_BASE	= %x \n\
			F01_RMI_DATA_BASE	= %x \n\
			",F01_RMI_QUERY_BASE,F01_RMI_CMD_BASE,F01_RMI_CTRL_BASE,F01_RMI_DATA_BASE);


	ret = i2c_dma_read(ts->client, 0xE9, 4, &(buf[0x0]));	  
	F34_FLASH_QUERY_BASE = buf[0];
	F34_FLASH_CMD_BASE = buf[1];
	F34_FLASH_CTRL_BASE = buf[2]; 
	F34_FLASH_DATA_BASE = buf[3];
	TPD_DEBUG("F34_FLASH_QUERY_BASE = %x \n\
			F34_FLASH_CMD_BASE	= %x \n\
			F34_FLASH_CTRL_BASE	= %x \n\
			F34_FLASH_DATA_BASE	= %x \n\
			",F34_FLASH_QUERY_BASE,F34_FLASH_CMD_BASE,F34_FLASH_CTRL_BASE,F34_FLASH_DATA_BASE);


	F01_RMI_QUERY11 = F12_2D_QUERY_BASE+11;
	F01_RMI_CTRL00 = F01_RMI_CTRL_BASE;
	F01_RMI_CTRL01 = F01_RMI_CTRL_BASE + 1;
	F01_RMI_CTRL02 = F01_RMI_CTRL_BASE + 2; // mignqiang.guo 2015/4/19 add for status bar pull down 
	F01_RMI_CMD00 = F01_RMI_CMD_BASE;
	F01_RMI_DATA01 = F01_RMI_DATA_BASE + 1; 


	F12_2D_CTRL08 = F12_2D_CTRL_BASE;
	F12_2D_CTRL23 = F12_2D_CTRL_BASE + 9;
	F12_2D_CTRL32 = F12_2D_CTRL_BASE + 15;
	F12_2D_DATA38 = F12_2D_DATA_BASE + 54;
	F12_2D_DATA39 = F12_2D_DATA_BASE + 55;
	F12_2D_CMD00 = F12_2D_CMD_BASE;

	F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;

	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x4); 
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}

	ret = i2c_dma_read(ts->client, 0xE9, 4, &(buf[0x0]));		
	F51_CUSTOM_QUERY_BASE = buf[0];
	F51_CUSTOM_CMD_BASE = buf[1];
	F51_CUSTOM_CTRL_BASE = buf[2]; 
	F51_CUSTOM_CTRL31 = F51_CUSTOM_CTRL_BASE + 11;    
	F51_CUSTOM_DATA_BASE = buf[3];
	TPD_DEBUG("F51_CUSTOM_QUERY_BASE = %x \n\
			F51_CUSTOM_CMD_BASE  = %x \n\
			F51_CUSTOM_CTRL_BASE	= %x \n\
			F51_CUSTOM_DATA_BASE	= %x \n\
			",F51_CUSTOM_QUERY_BASE,F51_CUSTOM_CMD_BASE,F51_CUSTOM_CTRL_BASE,F51_CUSTOM_DATA_BASE);	

		F51_CUSTOM_CTRL00 = F51_CUSTOM_CTRL_BASE;

	F51_CUSTOM_DATA04 = F51_CUSTOM_DATA_BASE;

	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x01); 
	if (ret < 0) {
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = i2c_dma_read(ts->client, 0xE9, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];
	TPD_DEBUG("F54_QUERY_BASE = %x \n\
			F54_CMD_BASE  = %x \n\
			F54_CTRL_BASE	= %x \n\
			F54_DATA_BASE	= %x \n\
			",F54_ANALOG_QUERY_BASE,F54_ANALOG_COMMAND_BASE ,F54_ANALOG_CONTROL_BASE,F54_ANALOG_DATA_BASE);	

		ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00); 		
	return 0;
}



#ifdef SUPPORT_GESTURE
static int synaptics_enable_interrupt_for_gesture(struct synaptics_ts_data *ts, int enable)
{

	int ret;
	uint8_t status_int;
	int F12_2D_CTRL20;
	int F12_2D_CTRL27;
	unsigned char reportbuf[3];
	unsigned char val[4];
	/* page select = 0x0 */
	F12_2D_CTRL20 = F12_2D_CTRL_BASE + 0x07;
	F12_2D_CTRL27 = F12_2D_CTRL_BASE + 0x0c;
	
	TPD_DEBUG("%s is called\n", __func__);
	is_gesture_enable = enable;
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		msleep(20);
		ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0); 
		if( ret<0 )
			TPD_ERR("%s: select page failed ret = %d\n", __func__, ret);
		return -1;
	}
	ret = i2c_dma_read( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ){
		TPD_ERR("read reg F12_2D_CTRL20 failed\n");
		return -1;
	}
	ret = i2c_dma_read( ts->client, F12_2D_CTRL27, 4, &(val[0x0]) );
	val[0] = 0xef;
	ret = i2c_dma_write( ts->client, F12_2D_CTRL27, 4, &(val[0x0]) );
	if( enable )
	{
		reportbuf[2] |= 0x02 ;
		//enable_irq(touch_irq);
	}
	else
	{
		reportbuf[2] &= 0xfd ;
	}

	status_int = (ret & 0xF8) | 0x04;
	/*enable gpio wake system through intterrupt*/		
	enable_irq_wake(ts->client->irq);
	gesture = UnkownGestrue ;
	/*clear interrupt bits for previous touch*/
	TPD_DEBUG("clear interrupt bits for previous touch\n");
	ret = i2c_dma_write( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ){
		TPD_ERR("%s :Failed to write report buffer\n", __func__);
		return -1;
	}		

	return 0;	
}
#endif

//mingqiang.guo for charge pulg in ,open tp Finger Amplitude Thre and Finger Dbounce ,avoid charge disturb
int charge_plug_tp_avoid_distrub_s3320(int enable,int is_fast_charge)
{
	charge_plug_status = enable ;
	atomic_set(&charge_plug_in_flag,charge_plug_status); 
	TPD_ERR("%s is call ,charge_plug_status =%d",__func__,charge_plug_status);
	if(!is_suspend)
		queue_work(speedup_resume_wq, &ts_g->charge_write_tp_work); 

	return 0;
}

static void charge_write_tp_work_function(struct work_struct *work)
{
	int ret = 0; 

	if(ts_g == NULL) //mingqiang.guo add for AT mode .tp is not register
	{
		TPD_ERR("tp do not register\n");
		return;
	}

	
	if(ts_g->loading_fw == true)
	{
		TPD_ERR("  updtae tp firmware now,can not write tp reg \n");
		return ;
	}	

//	if(charge_plug_status == enable)
//	{
//		TPD_ERR("charge_plug_tp_avoid_distrub charge_plug_status is same %d,not operate \n",enable);
//		return -1;
//	}
//	charge_plug_status = enable ;
		
	mutex_lock(&ts_g->mutex); 
	//atomic_set(&charge_plug_in_flag,enable); 
	TPD_ERR("%s, page 4  F51_CUSTOM_CTRL31=0x%x write  charge_plug_status=%d   \n",__func__,F51_CUSTOM_CTRL31,charge_plug_status);//0x0425
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, 0xff, 4);  
	if( ret < 0 ){
		TPD_ERR(" %s i2c error\n",__func__);
		goto OUT;
	}

	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F51_CUSTOM_CTRL31&0x00ff, charge_plug_status );  
	if( ret < 0 ){
		TPD_ERR(" %s i2c error\n",__func__);
		goto OUT;
	}


	
OUT :
	synaptics_rmi4_set_page(ts_g->client, 0xff, 0x0); 
/* 	if(charge_plug_status==0 && !is_fast_charge) //can not  reset 
	{
		TPD_ERR("unplug charge, software reset tp \n");
		i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CMD_BASE, 0x01);//software reset TP
	}*/
	mutex_unlock(&ts_g->mutex);
	return ; 
}
//end 

#ifdef SUPPORT_LEATHER
int hall_interrupt_enable_glove_mode(int enable)
{
	int ret ;
	struct synaptics_ts_data *ts = ts_g;
		
	if( NULL == ts)
	{
		TPD_ERR("hall_interrupt_enable_glove_mode ts == NULL \n");	
		return -1;
	}

	atomic_set(&hall_close_flag,enable);
	
	TPD_ERR("hall_interrupt_enable_glove_mode hall_close_flag = %d\n",atomic_read(&hall_close_flag));
	if( enable == 1 )
	{
		ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_read_byte_data_s3508(ts->client, F12_2D_CTRL23); //0x001E
		//TPDTM_DMESG("enable glove  ret is %x ret|0x20 is %x\n", ret, ret|0x20);
		ret = i2c_smbus_write_byte_data_s3508(ts->client, F12_2D_CTRL23, ret | 0x20);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
	}
	else
	{
		ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_read_byte_data_s3508(ts->client, F12_2D_CTRL23);
		ret = i2c_smbus_write_byte_data_s3508(ts->client, F12_2D_CTRL23, ret & 0xdf);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}			
	}

	GLOVE_ENABLE_END:
		return ret;

}

static int oppo_leather_glove_mode_enable_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	printk("glove mode enable is: %d\n", atomic_read(&glove_enable));
	ret = sprintf(page, "%d\n", atomic_read(&glove_enable));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static int oppo_leather_glove_mode_enable_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts = ts_g;
	int ret = 0 ;
	char buf[10];

	if(!ts)
		return count;
	
	mutex_lock(&ts->mutex);
	if( count > 10 )
		goto GLOVE_ENABLE_END;
	if( copy_from_user( buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);	
		goto GLOVE_ENABLE_END;
	}
	sscanf(buf, "%d", &ret);
	
	TPD_DEBUG("tp_glove_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){	
		atomic_set(&glove_enable, ret);
		hall_interrupt_enable_glove_mode(ret);
	}	
	switch(ret){	
		case 0:	
			TPD_ERR("oppo_leather_glove_mode_enable will be disable\n");
			break;
		case 1:	
			TPD_ERR("oppo_leather_glove_mode_enable will be enable\n");
			break;		
		default:
			TPD_ERR("Please enter 0 or 1 to open or close the glove function\n");
	}
GLOVE_ENABLE_END:
	mutex_unlock(&ts->mutex);
	return count;
}

static const struct file_operations oppo_leather_glove_mode_enable_proc_fops = {
	.write = oppo_leather_glove_mode_enable_write_func,
	.read =  oppo_leather_glove_mode_enable_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#endif 

static int tp_reg_address = 0x001B;
static ssize_t tp_reg_operate_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	char buf[5];
	char sleep,interrupt;
	int gesture = atomic_read(&double_enable);

	ret = synaptics_rmi4_set_page(ts_g->client, 0xff, tp_reg_address>>8); 
	if (ret < 0) 
	{
		TPD_DEBUG("%s error %d\n",__func__,__LINE__);
		return -1;
	}

	//ret  = i2c_smbus_read_byte_data_s3508(ts_g->client,tp_reg_address & 0x00ff);
	ret = i2c_dma_read(ts_g->client, tp_reg_address & 0x00ff, 3,&buf[0]); 
	ret = i2c_dma_read(ts_g->client, 0x0e, 1, &sleep);
	ret = i2c_dma_read(ts_g->client, 0x0f, 1, &interrupt);
	
	TPD_ERR("read reg=0x%x is 0x%x\n", tp_reg_address,buf[2]);
	ret = sprintf(page, "read reg=0x%x is 0x%x, sleep = 0x%x, interrupt = 0x%x is_gesture_enable = %d double_enable = %d is_suspend = %d\n", tp_reg_address,buf[2], sleep, interrupt, is_gesture_enable,gesture,is_suspend);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}


static ssize_t tp_reg_operate_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0 ;
	char buf[50];

	int data;
	
	if (copy_from_user( buf, buffer, count)) {
		TPDTM_DMESG(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
 
	sscanf(buf, "0x%x 0x%x ", &tp_reg_address, &data);
	TPD_ERR("%s write  reg=0x%x data=0x%x \n",__func__,tp_reg_address,data);

	ret = synaptics_rmi4_set_page(ts_g->client, 0xff, tp_reg_address>>8); 
	if (ret < 0) 
	{
		TPD_DEBUG("%s error %d\n",__func__,__LINE__);
		return -1;
	}

	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,tp_reg_address & 0x00ff,data );
	if (ret < 0) 
	{
		TPD_DEBUG("%s error %d\n",__func__,__LINE__);
		return -1;
	}
	
	return count;
}

static const struct file_operations tp_reg_operate_proc_fops = {
	.write = tp_reg_operate_write_func,
	.read =  tp_reg_operate_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};




#ifdef SUPPORT_GLOVES_MODE
static int synaptics_glove_mode_enable(struct synaptics_ts_data *ts)
{

	int ret;
#if 1
	/* page select = 0x4 */
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x4); 
	if (ret < 0) 
	{
		TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for page select\n");
		return -1;
	}
	TPD_ERR("synaptics_glove_mode_enable  glove_enable = %d  F12_2D_CTRL23=0x%x \n",atomic_read(&glove_enable),F12_2D_CTRL23);
	if(1 == atomic_read(&glove_enable))
	{
		ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_read_byte_data_s3508(ts->client, F12_2D_CTRL23); //0x001E
		//TPDTM_DMESG("enable glove  ret is %x ret|0x20 is %x\n", ret, ret|0x20);
		ret = i2c_smbus_write_byte_data_s3508(ts->client, F12_2D_CTRL23, ret | 0x20);  
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
	}
	else
	{
		ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0);  
			if( ret < 0 ){
				TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}
			ret = i2c_smbus_read_byte_data_s3508(ts->client, F12_2D_CTRL23);
			ret = i2c_smbus_write_byte_data_s3508(ts->client, F12_2D_CTRL23, ret & 0xdf);  
			if( ret < 0 ){
				TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}			
	}

	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00); 
	if (ret < 0) 
	{
		TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for page select\n");
		return -1;
	}
#endif
GLOVE_ENABLE_END:
	return ret;
}
#endif 

#ifdef SUPPORT_TP_SLEEP_MODE
static int synaptics_sleep_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	mutex_lock(&ts->mutex);
	/* page select = 0x0 */
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00); 
	if (ret < 0) 
	{
		TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for page select\n");
		goto err;
	}
	if(1 == atomic_read(&sleep_enable))
	{
		/*0x00:enable glove mode,0x02:disable glove mode,*/
		TPDTM_DMESG("sleep mode enable\n");
		ret = i2c_smbus_write_byte_data_s3508(ts->client, F01_RMI_CTRL00, 0x01 ); 
		if (ret < 0) 
		{
			TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
			goto err;
		}
	}else{	
		TPDTM_DMESG("sleep mode disable\n");
		//chenggang.li@BSP add for oppo TP NO SLEEP set 1
		ret = i2c_smbus_write_byte_data_s3508(ts->client, F01_RMI_CTRL00, 0x84 ); 
		if (ret < 0) 
		{
			TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for mode select\n");
			goto err;
		}
	}
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00); 
	if (ret < 0) 
	{
		TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for page select\n");
		goto err;
	}

	ret = i2c_smbus_read_byte_data_s3508(ts->client, F01_RMI_CTRL00);
	TPD_DEBUG("after write : read from address F01_RMI_CTRL00 0x%x  = 0x%x \n",F01_RMI_CTRL00,ret);

err:
	mutex_unlock(&ts->mutex);
	return ret;
}
#endif
static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf1[11];
	int ret ;

	memset(buf1, 0 , sizeof(buf1));
	/* page select = 0x0 */
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0); 
	if(ret < 0){
		TPD_DEBUG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	ret = i2c_dma_read(ts->client, F01_RMI_QUERY_BASE+11, 8, &(buf1[0x0]));
	ret = i2c_dma_read(ts->client, F01_RMI_QUERY_BASE+19, 2, &(buf1[0x8]));
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_product_id: failed to read product info\n");
		return -1;
	}
	TPD_DEBUG("synaptics product id: %s \n",buf1);
	return 0;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;

	TPD_DEBUG("%s is called!\n",__func__);
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		TPD_ERR("init_panel failed for page select\n");
		return -1;
	}
	/*device control: normal operation, configur=1*/
	//chenggang.li @BSP change 0x80 to 0x84 , bit2:1 nosleep  bit2:0 sleep 
	ret = i2c_smbus_write_byte_data_s3508(ts->client, F01_RMI_CTRL00, 0x80); 
	if (ret < 0) {
		msleep(150);
		ret = i2c_smbus_write_byte_data_s3508(ts->client, F01_RMI_CTRL00, 0x80); 
		if( ret < 0 ){
			TPD_ERR("%s failed for mode select\n",__func__);
		}
	} 

	return ret;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	uint8_t abs_status_int;
	/* page select = 0x0 */

	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0); 
	if(ret < 0) {
		TPD_DEBUG("synaptics_enable_interrupt: select page failed ret = %d\n", ret);
		return -1;
	}

	if(enable){
		abs_status_int = 0x7f;
		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data_s3508(ts->client, F01_RMI_DATA01);
		if(ret < 0) {
			TPD_DEBUG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	}else {
		abs_status_int = 0x0;
	}
	ret = i2c_smbus_write_byte_data_s3508(ts->client, F01_RMI_CTRL01, abs_status_int);
	if(ret < 0) {
		TPD_DEBUG("synaptics_enable_interrupt: enable or disable abs \
				interrupt failed,abs_int =%d\n",abs_status_int);
		return -1;
	}
	ret = i2c_smbus_read_byte_data_s3508(ts->client, F01_RMI_CTRL00+1);
	return 0;	
}

//static void synaptics_ts_irq_handler(void);
static irqreturn_t synaptics_ts_irq_handler(unsigned irq, struct irq_desc *desc);

static void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			udelay(1);
		}
	}
}

static void int_state(struct synaptics_ts_data *ts)
{
	int ret = -1;
	unsigned char reportbuf[3];
	int F12_2D_CTRL20;

	F12_2D_CTRL20 = F12_2D_CTRL_BASE + 0x07;
	//ret = i2c_smbus_write_byte_data_s3508(ts->client, F01_RMI_CMD00, 0x01);
	ret = synaptics_soft_reset(ts_g);
	if(ret){
		TPD_ERR("int_state:cannot reset touch panel \n");
		return;
	}
	msleep(250);
	
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif

	if( is_gesture_enable == 1 ){

		ret = i2c_dma_read( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
		reportbuf[2] |= 0x02 ;
		ret = i2c_dma_write( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
		if( ret < 0 ){
			TPD_ERR("%s :Failed to write report buffer\n", __func__);
			return;
		}
	}

	synaptics_init_panel(ts);
	if( ret < 0 ){
		TPD_DEBUG("int_state: control tm1400 to sleep failed\n");
		return;
	}
	
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret){
		TPD_DEBUG("int_state:cannot  enable interrupt \n");
		return;
	}
	
}

static int pre_btn_key = 0; 
static int key_press_all_the_time = 0;
static void int_btn(struct synaptics_ts_data *ts)
{
	int ret = -1;

	int F51_CUSTOM_DATA32=0x19;

	TPD_DEBUG("int_btn ...\n");

	if( gesture_enable == 1 )
	{		
		synaptics_rmi4_set_page(ts->client, 0xff, 0x04);		
		ret = i2c_smbus_read_byte_data_s3508(ts->client, F51_CUSTOM_DATA32);			
		TPD_ERR("in gesture_enable key code =%d \n", ret);
		if( (ret & 0x07) != 0 )
		{				
			gesture_upload = DouTap;				
			input_report_key(ts->input_dev, KEY_F4, 1);				
			input_sync(ts->input_dev);				
			input_report_key(ts->input_dev, KEY_F4, 0);				
			input_sync(ts->input_dev);	
			TPD_ERR("key code =%d  button detect double tap gesture\n", ret);
		}
		synaptics_rmi4_set_page(ts->client, 0xff, 0);		
		return;

	}


	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x02); 
	if (ret < 0) {
		TPD_DEBUG("i2c_smbus_write_byte_data_s3508 failed for page select\n");
		return;
	}

	ret =i2c_smbus_read_byte_data_s3508(ts->client, 0x00);
	if(ret < 0)
	{
		TPD_DEBUG("i2c_smbus_read_byte_data_s3508 failed for button state\n");
		return;
	}
	TPD_ERR("key code 0x%x\n",ret&0x07);
	//mingqiang.guo add for two key down ,can not report long press ,resolve for  electrostatic experiment
	if((ret&0x07)!=0 ) //&& (ret&0x07)!=7 && (ret&0x07)!=6 && (ret&0x07)!=3 && (ret&0x07)!=5 )
	{
		//TPD_ERR("ts->max_x = %d report_key_point_y = %d \n",ts->max_x,report_key_point_y);
		//report_key_point_y = 2021 ; 
		pre_btn_key = ret&0x07;

  		if ( 0 == is_lcd_touch )
  		{
			if(ret&0x01)//menu
			{          
				input_report_key(ts->input_dev, KEY_MENU, 1);
				is_key_touch = 1;
			}

			if(ret&0x02)//home
			{	              
				input_report_key(ts->input_dev, KEY_HOMEPAGE, 1);
				is_key_touch = 1;
			}

			if(ret&0x04)//reback
			{				
				input_report_key(ts->input_dev, KEY_BACK, 1);
				is_key_touch = 1;
			}
  		}
	}
	else
	{
		if(pre_btn_key&0x01)//menu
		{          
			input_report_key(ts->input_dev, KEY_MENU, 0);
			is_key_touch = 0;
		}

		if(pre_btn_key&0x02)//home
		{	              
			input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
			is_key_touch = 0;
		}

		if(pre_btn_key&0x04)//reback
		{				
			input_report_key(ts->input_dev, KEY_BACK, 0);
			is_key_touch = 0;
		}
		pre_btn_key = 0;
	}

	input_sync(ts->input_dev);
	synaptics_rmi4_set_page(ts->client, 0xff, 0x0);
}


static int i2c_dma_read(const struct i2c_client *client, u8 addr, s32 len, u8 *rxbuf)
{
	int ret;
	s32 retry = 0;
	u8 buffer[1];

	struct i2c_msg msg[2] =
	{
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = buffer,
			.len = 1,
			.timing = I2C_MASTER_CLOCK
		},
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = (u8 *)gprDMABuf_pa,     
			.len = len,
			.timing = I2C_MASTER_CLOCK
		},
	};

	buffer[0] = addr & 0xFF;

	if (rxbuf == NULL){
		return -1;
	}
	TPD_DEBUG("srd dma i2c read: 0x%x, %d bytes(s)\n", addr, len);
	for (retry = 0; retry < 5; ++retry)  //old  retry < 5
	{
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
		{
			continue;
		}
		memcpy(rxbuf, gprDMABuf_va, len);
		return 0;
	}
	TPD_ERR("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);

	return ret;
}


static int i2c_dma_write(const struct i2c_client *client, u8 addr, s32 len, u8 *txbuf)
{
	int ret;
	s32 retry = 0;
	u8 *wr_buf = gpwDMABuf_va;
	struct i2c_msg msg =
	{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = (u8 *)gpwDMABuf_pa,
		.len = 1 + len,
		.timing = I2C_MASTER_CLOCK
	};
	wr_buf[0] = (u8)(addr & 0xFF);
	if (txbuf == NULL){
		return -1;
	}
	memcpy(wr_buf+1, txbuf, len);
	for (retry = 0; retry < 5; ++retry) //old  retry < 5
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
		{
			continue;
		}
		return 0;
	}
	TPD_DEBUG("Dma I2C Write Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
	
	return ret;
}


#if 0 
static int synaptics_rmi4_i2c_read(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;	
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = ts->client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};
	buf = addr & 0xFF;
	for( retry = 0; retry < 2; retry++ ){
		if( i2c_transfer(ts->client->adapter, msg, 2) == 2){
			retval = length;
			break;
		}
		msleep(20);
	}
	if( retry == 2 ){
		dev_err(&ts->client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		retval = -5;
	}
	return retval;
}
#endif

#ifdef SUPPORT_GESTURE

#ifdef SUPPORT_REPORT_COORDINATE
static void synaptics_get_coordinate_point(struct synaptics_ts_data *ts)
{
	int ret,i;
	uint8_t coordinate_buf[25] = {0};
	uint16_t trspoint = 0;

	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x4); 
	ret = i2c_dma_read(ts->client, F51_CUSTOM_DATA04, 8, &(coordinate_buf[0])); // 14045 use F51_CUSTOM_DATA05
	ret = i2c_dma_read(ts->client, F51_CUSTOM_DATA04 + 8, 8, &(coordinate_buf[8]));
	ret = i2c_dma_read(ts->client, F51_CUSTOM_DATA04 + 16, 8, &(coordinate_buf[16])); 
	ret = i2c_dma_read(ts->client, F51_CUSTOM_DATA04 + 24, 1, &(coordinate_buf[24]));
	for(i = 0; i< 23; i += 2){
		trspoint = coordinate_buf[i]|coordinate_buf[i+1] << 8;
		TPD_DEBUG("synaptics TP read coordinate_point[%d] = %d\n",i,trspoint);
	}

	TPD_DEBUG("synaptics TP coordinate_buf = 0x%x\n",coordinate_buf[24]);
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0);
	Point_start.x = (coordinate_buf[0] | (coordinate_buf[1] << 8)) * LCD_WIDTH/ (ts->max_x);
	Point_start.y = (coordinate_buf[2] | (coordinate_buf[3] << 8)) * LCD_HEIGHT/ (1920);
	Point_end.x   = (coordinate_buf[4] | (coordinate_buf[5] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_end.y   = (coordinate_buf[6] | (coordinate_buf[7] << 8)) * LCD_HEIGHT / (1920);
	Point_1st.x   = (coordinate_buf[8] | (coordinate_buf[9] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_1st.y   = (coordinate_buf[10] | (coordinate_buf[11] << 8)) * LCD_HEIGHT / (1920);
	Point_2nd.x   = (coordinate_buf[12] | (coordinate_buf[13] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_2nd.y   = (coordinate_buf[14] | (coordinate_buf[15] << 8)) * LCD_HEIGHT / (1920);
	Point_3rd.x   = (coordinate_buf[16] | (coordinate_buf[17] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_3rd.y   = (coordinate_buf[18] | (coordinate_buf[19] << 8)) * LCD_HEIGHT / (1920);
	Point_4th.x   = (coordinate_buf[20] | (coordinate_buf[21] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_4th.y   = (coordinate_buf[22] | (coordinate_buf[23] << 8)) * LCD_HEIGHT / (1920);
	TPD_DEBUG("synaptics TP (xStart,yStart)=(%d,%d),(xEnd,yEnd) = (%d,%d)\n",
			Point_start.x,Point_start.y,Point_end.x,Point_end.y);	
	TPD_DEBUG("synaptics TP (x1,y1) = (%d,%d)(x2,y2) = (%d,%d) \n \
			(x3,y3) = (%d,%d),(x4,y4) = (%d,%d)\n",
			Point_1st.x,Point_1st.x,Point_2nd.x,Point_2nd.y,
			Point_3rd.x,Point_3rd.y,Point_4th.x,Point_4th.y);
	clockwise     = (coordinate_buf[24] & 0x10) ? 1 : 
		(coordinate_buf[24] & 0x20) ? 0 : 2; // 1--clockwise, 0--anticlockwise, not circle, report 2
}
#endif


static void gesture_judge(struct synaptics_ts_data *ts)
{
	int ret = 0, regswipe;
	uint8_t gesture_buffer[10];
	static int F12_2D_DATA04;	  
	unsigned char reportbuf[3];
	int F12_2D_CTRL20;

	TPD_DEBUG("%s is called!\n",__func__); 
	F12_2D_CTRL20 = F12_2D_CTRL_BASE + 0x07;   //0x001C 
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00); 	  
	if (ret < 0) {
		TPDTM_DMESG("failed to transfer the data, ret = %d\n", ret);
	}
	F12_2D_DATA04 = 0x0007;
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00); 	    
	ret = i2c_dma_read(ts->client,  F12_2D_DATA04, 5, &(gesture_buffer[0]));	  
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x4); 
	regswipe = i2c_smbus_read_byte_data_s3508(ts->client, F51_CUSTOM_DATA04+0x18); //14045 has no WakeUpGesture Swipe Direction 
	TPD_DEBUG(" gesture_buffer[0] = 0x%x, regswipe = 0x%x,gesture_buffer[1] = 0x%x, gesture_buffer[4] = 0x%x\n", gesture_buffer[0], regswipe, gesture_buffer[1], gesture_buffer[4]);
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00); 

	//detect the gesture mode
	switch (gesture_buffer[0]) {
		case CIRCLE_DETECT:
			gesture = Circle;
			break;
		case SWIPE_DETECT:
			gesture =   (regswipe == 0x41) ? Left2RightSwip   :
				(regswipe == 0x42) ? Right2LeftSwip   :
				(regswipe == 0x44) ? Up2DownSwip      :
				(regswipe == 0x48) ? Down2UpSwip      :
				(regswipe == 0x80) ? DouSwip          :
				UnkownGestrue;
			break;
		case DTAP_DETECT:
			gesture = DouTap;
			break;
		case VEE_DETECT:
			gesture =   (gesture_buffer[2] == 0x01) ? DownVee  :
				(gesture_buffer[2] == 0x02) ? UpVee    :
				(gesture_buffer[2] == 0x04) ? RightVee :
				(gesture_buffer[2] == 0x08) ? LeftVee  : 
				UnkownGestrue;

			break;
		case UNICODE_DETECT:
			gesture =   (gesture_buffer[2] == 0x77 && gesture_buffer[3] == 0x00) ? Wgestrue :
				(gesture_buffer[2] == 0x6d && gesture_buffer[3] == 0x00) ? Mgestrue :
				UnkownGestrue;
			break;
		case 0:
			gesture = UnkownGestrue;

	}

	TPD_ERR("detect %s gesture\n", gesture == DouTap ? "double tap" :
			gesture == UpVee ? "up vee" :
			gesture == DownVee ? "down vee" :
			gesture == LeftVee ? "(>)" :
			gesture == RightVee ? "(<)" :
			gesture == Circle ? "circle" :
			gesture == DouSwip ? "(||)" :
			gesture == Left2RightSwip ? "(-->)" :
			gesture == Right2LeftSwip ? "(<--)" :
			gesture == Up2DownSwip ? "up to down |" :
			gesture == Down2UpSwip ? "down to up |" :
			gesture == Mgestrue ? "(M)" :
			gesture == Wgestrue ? "(W)" : "unknown");


	synaptics_get_coordinate_point(ts);
	if(gesture != UnkownGestrue ){
		gesture_upload = gesture;
		input_report_key(ts->input_dev, KEY_F4, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, KEY_F4, 0);
		input_sync(ts->input_dev);
	}else{

		ret = i2c_dma_read( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
		ret = reportbuf[2] & 0x20;
		if(ret == 0)
			reportbuf[2] |= 0x02 ;
		ret = i2c_dma_write( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) ); //enable gesture
		if( ret < 0 ){
			TPD_ERR("%s :Failed to write report buffer\n", __func__);
			return;
		}
	} 
}
#endif

static void int_touch(struct synaptics_ts_data *ts)
{
	static int last_report_abs_mt_width_major;
	int ret= -1,i=0;
	uint8_t buf[80];
	uint32_t finger_state = 0;
	uint8_t finger_num = 0;
	uint32_t obj_attention = 0;
	struct point_info points;
	//mingqiang.guo@phone.bsp add for dou ban Music apk can not touch
	static int touch_major_report_num  = 0;
	//end 
	
	memset(buf,0,sizeof(buf));
	points.x = 0;
	points.y = 0;
	points.z = 0;

	ret = i2c_dma_read(ts->client, F12_2D_DATA_BASE+0x03, 2, &(buf[0x0]));
	obj_attention = buf[1] << 8;
	obj_attention += buf[0];
	TPD_DEBUG("obj_attention = 0x%x  buf1 =0x%x buf0=0x%x \n",obj_attention, buf[1], buf[0]);
	
	if( (obj_attention&0x03FF) != 0 )
	{	
		/* mingqiang.guo add  2015-11-19  Start checking from the highest bit */	
		unsigned char fingers_to_process;
		fingers_to_process =  ts->max_num;
		for( i = 9;  ; i--)
		{
			if (  (obj_attention&0x03FF)>>i  || i==0 )
				break;
			else
				fingers_to_process--;	
		}
		ret = i2c_dma_read(ts->client, F12_2D_DATA_BASE, 8*fingers_to_process, &(buf[0x0]));
		if (ret < 0) 
		{
			TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
			return;
		}	

		for( i = 0; i < ts->max_num; i++ )
		{
			points.status = buf[i*8]; //mingqiang.guo@phone.bsp.tp add for auto report (0,0), point buf[0]==0 ,no point
			
			if(((obj_attention&0x03FF) >> i )&0x01   &&  ( points.status == 0 )  ) // buf[0] ==0 is wrong point ,no process
		   		continue;
			if(((obj_attention&0x03FF) >> i )&0x01   &&  ( points.status != 0 )  ) //mingqiang.guo modify for reslove dirty touch point
			{
				points.x = ((buf[i*8+2]&0x0f)<<8) | (buf[i*8+1] & 0xff);
				points.raw_x = buf[i*8+6] & 0x0f;
				points.y = ((buf[i*8+4]&0x0f)<<8) | (buf[i*8+3] & 0xff);
				points.raw_y = buf[i*8+7] & 0x0f;
				points.z = buf[i*8+5];
				//points.x = 1080 -points.x;
				//points.y = 1920 - points.y;		

				TPD_DEBUG("points.x = %d i = %d\n", points.x, i);
				//solve narrow bezel problem
				if(limit_enable)
				{
					if(points.x > left_x2 && points.x < right_x2)		//area 3
					{
						if(1 == in_which_area )
						{
							synaptics_rmi4_free_fingers(ts);
						}
						in_which_area = 3;
						TPD_DEBUG("in_which_area is %d\n", in_which_area);
					}

					if((points.x > left_x1 && points.x < left_x2) || (points.x >right_x2 && points.x < right_x1))	//area2
					{
						if(in_which_area == 1)
							in_which_area = 2;
						TPD_DEBUG("in_which_area is %d\n", in_which_area);
					}
					
					if(points.x < left_x1 || points.x > right_x1)		//area 1
					{
						if(2 == in_which_area)
						{
							in_which_area = 1;
							continue;
						}
						if(3 == in_which_area )
							continue;
						in_which_area = 1;
						TPD_DEBUG("in_which_area is %d\n", in_which_area);
					}
				}
			
#ifdef SUPPORT_LEATHER
				if (1 == atomic_read(&hall_close_flag) )  
				{
					if( (points.y > 780)  &&  (points.y < 1980)  &&  ( 6 == buf[i*8])  ) 
					{			
						TPD_DEBUG("not report point : hall_close_flag=%d  finger type=%d points:(%d,%d) \n",
						atomic_read(&hall_close_flag), buf[i*8], points.x, points.y );
						continue;
					}
				}
#endif 				

			
				pre_touch[i] = 1 ;
#ifdef TYPE_B_PROTOCOL
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 1);
#endif				
				TPD_DEBUG("down finger_num = %d  i = %d \n",finger_num,i);
				finger_num++;
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
				input_report_key(ts->input_dev,	BTN_TOOL_FINGER, 1);
				//mingqiang.guo@phone.bsp add for dou ban Music apk can not touch
				touch_major_report_num++;
				if(boot_mode == RECOVERY_BOOT)
				{
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, points.z);
					//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, (points.raw_x + points.raw_y)/2);
				}
				else
				{
					if( touch_major_report_num == 1)
					{
						 input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, (points.raw_x+points.raw_y)/2);		//add 1,ABS_MT_WIDTH_MAJOR change, in order to get action MOVE
						 last_report_abs_mt_width_major = (points.raw_x+points.raw_y)/2;
					}
					else if(!(touch_major_report_num&0x007f))
					{
						if(last_report_abs_mt_width_major == (points.raw_x+points.raw_y)/2)
						{
							last_report_abs_mt_width_major = (points.raw_x+points.raw_y)/2 + 1;
						}
						else
						{
							last_report_abs_mt_width_major = (points.raw_x+points.raw_y)/2;
						}
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, last_report_abs_mt_width_major);
					}

					//input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, max(points.raw_x ,points.raw_y) );   
        				//input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, min(points.raw_x ,points.raw_y) );
				//end 
				}

				//input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, (points.raw_x+points.raw_x)/2);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, points.x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,  points.y);
				if( Dot_report_down == 50 )
				{
					TPD_ERR("Synaptics:Down[%4d %4d %4d]\n", points.x, points.y, points.z);
					Dot_report_down = 0;
				}
				else
				{
					TPD_DEBUG("Down[%4d %4d %4d]\n", points.x, points.y, points.z);
					Dot_report_down++;					
				} 
				
			}
#ifdef TYPE_B_PROTOCOL
			else
			{			   
				//if(	pre_touch[i] ==1 )
				{
					TPD_DEBUG("up finger_num = %d  i = %d \n",finger_num,i);
					pre_touch[i] = 0 ;					 
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
				}
			}	
#endif		
		}
		input_sync(ts->input_dev);
		ts->pre_finger_state = finger_state; 
	}
	else
	{
		//mingqiang.guo@phone.bsp add for dou ban Music apk can not touch
		touch_major_report_num = 0; 
		//end 

		//synaptics_rmi4_free_fingers(ts);
		for (i = 0; i < ts->max_num; i++) 
		{
			if(	pre_touch[i] ==1 )
			{
                //TPD_DEBUG("else up finger_num = %d  i = %d \n",finger_num,i);
                pre_touch[i]=0;
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
			}
		}
		input_report_key(ts->input_dev,BTN_TOUCH, 0);
		input_report_key(ts->input_dev,BTN_TOOL_FINGER, 0);
		input_sync(ts->input_dev);
		in_which_area = 0;
	}
	
	if( finger_num )		
		is_lcd_touch = 1;
	else
		is_lcd_touch = 0;	

#ifdef SUPPORT_GESTURE
	if (is_gesture_enable == 1) {
		gesture_judge(ts);
	}
#endif	

//wanghao@phone.bsp 2016-1-7 add for judge key press all the time, can not touch up, need force cal
	//if(1 == is_lcd_touch)
		//judge_key_press_all_the_time();
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret;
	uint8_t buf[8]; 	
	uint8_t status = 0;
	uint8_t inte = 0;	
	struct synaptics_ts_data *ts = container_of(work,struct synaptics_ts_data, work);
	memset(buf, 0, sizeof(buf));
	mutex_lock(&ts->mutex);
	TPD_DEBUG("ts_work is_suspend = %d  ts->enable_remote  = %d \n",is_suspend,ts->enable_remote );
	if( is_suspend == 1 || ts->enable_remote )
		goto FREE_IRQ;
	ret = i2c_smbus_write_byte_data_s3508(ts->client, 0xff, 0x00 ); 
	//ret = i2c_smbus_read_word_data(ts->client, F01_RMI_DATA_BASE);
	ret = i2c_dma_read(ts->client, F01_RMI_DATA_BASE, 2, &(buf[0x0]));

	#if 0 //mingqiang.guo 2015-6-17 del , i2c_dma_read will retry 5 time , use time  8S , too long
	if( ret < 0 ){		
		while( (ret < 0  ) && ( i2c_err_count < 5) ){
			msleep(5);
			//ret = i2c_smbus_read_word_data(ts->client, F01_RMI_DATA_BASE);
			ret = i2c_dma_read(ts->client, F01_RMI_DATA_BASE, 2, &(buf[0x0]));		
			i2c_err_count++;
		}
		TPDTM_DMESG("Synaptic:ret == %d and try %d times\n", ret, i2c_err_count);
	}
	#endif
	if(ret < 0){
		TPD_DEBUG("%s:ret = %d\n",__func__,ret);
		goto ERR;

	}


	status = buf[0];
	inte = buf[1] & 0x7f;
	TPD_DEBUG("%s inte=0x%x status =0x%x \n",__func__,inte,status);
	if(inte & 0x2) {
		TPD_ERR("Synaptic:ret = 0x%x, TP reset detected!\n", ret);
		is_key_touch = 0;
		is_lcd_touch = 0;	
		key_press_all_the_time = 0;
		synaptics_rmi4_free_fingers(ts);
		msleep(200);
		//write_back_reg_status(ts);
		//goto FREE_IRQ;
	}
	if(status) {	
		int_state(ts);
		is_key_touch = 0;
		is_lcd_touch = 0;
		key_press_all_the_time = 0;
		synaptics_rmi4_free_fingers(ts);
		//write_back_reg_status(ts);
		goto FREE_IRQ;
	}

	if(inte&0x10 )
	{
		#ifdef SUPPORT_LEATHER
			if(0 == atomic_read(&hall_close_flag) ) 
		#endif 
			int_btn(ts); 		
	}
	 if(inte&0x04) 
	{	
		
		key_press_all_the_time = 0;
		int_touch(ts);
		
	}
FREE_IRQ:
	mutex_unlock(&ts->mutex);
	enable_irq(touch_irq);
	return;
ERR:
	hard_reset_tp(ts);
	synaptics_rmi4_free_fingers(ts);
	mutex_unlock(&ts->mutex);
	enable_irq(touch_irq);
}

#ifndef TPD_USE_EINT
static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	queue_work(synaptics_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#else

static irqreturn_t synaptics_ts_irq_handler(unsigned irq, struct irq_desc *desc)
{
	TPD_DEBUG("synaptics_ts_irq_handler\n");
	disable_irq_nosync(touch_irq);
	queue_work(synaptics_wq, &ts_g->work);
	return IRQ_HANDLED;
}
#endif

#if 0
static enum hrtimer_restart synaptics_ts_report_timer_func(struct hrtimer *report_timer)
{
	if(err_report_num >= 60)	
	{
		reset_staus = 1;
	}
	err_report_num = 0;		
	return HRTIMER_NORESTART;
}
#endif

static ssize_t tp_show(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	uint8_t ret = 0;
	char * kernel_buf;
	
	kernel_buf = kmalloc(10, GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("kmalloc error!\n");
		return 0;
	}

	enable_irq(touch_irq);
	ret = i2c_smbus_read_byte_data_s3508(ts_g->client, F01_RMI_DATA_BASE);
	if(ret < 0)
		TPDTM_DMESG("tp_show read i2c err\n");

	ret= sprintf(kernel_buf, "%d\n", tp_debug);
	ret = simple_read_from_buffer(buf, size, ppos, kernel_buf, strlen(kernel_buf));
	kfree(kernel_buf);
	return ret;
	//return sprintf(buf, " tp_debug = %d\n", tp_debug);
}

//static ssize_t store_tp(struct device_driver *ddri, char *buf, size_t count)
static ssize_t store_tp(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	int tmp = 0;
	struct synaptics_ts_data *ts = ts_g; 
	int i;
	char buffer[10];

	if(size > 2)
		return size;

	if(copy_from_user(buffer, buf, size)){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return size;
	}
	
	if (1 == sscanf(buffer, "%d", &tmp))
	{
		tp_debug = tmp;
		//mingqiang.guo 2015/6/2 add for debug 
		switch(tp_debug)
		{
			case 10: 
					for( i = 0; i < ts->max_num; i++ )
					{
						#ifdef TYPE_B_PROTOCOL
						input_mt_slot(ts->input_dev, i);
						input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 1);
						#endif				

						input_report_key(ts->input_dev, BTN_TOUCH, 1);
						input_report_key(ts->input_dev,	BTN_TOOL_FINGER, 1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 20);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 20);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 0);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 0);
					}
					input_sync(ts->input_dev);


					for( i = 0; i < ts->max_num; i++ )
					{
						input_mt_slot(ts->input_dev, i);
						input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
					}
					input_report_key(ts->input_dev,BTN_TOUCH, 0);
					input_report_key(ts->input_dev,BTN_TOOL_FINGER, 0);
					input_sync(ts->input_dev);

				break ; 

				case 11: 			
					hard_reset_tp(ts);	//only for test 

				break;
		}
		//end 
	}
	else
	{
		TPD_DEBUG("invalid content: '%s', length = %zu\n", buffer, size);
	}

	return size;
}


static ssize_t oppo_tp_suspend_read_func(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	uint8_t ret = 0;
	char * kernel_buf;
	
	kernel_buf = kmalloc(10, GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("kmalloc error!\n");
		return 0;
	}

	ret= sprintf(kernel_buf, "%d\n", finish_suspend);
	ret = simple_read_from_buffer(buf, size, ppos, kernel_buf, strlen(kernel_buf));
	kfree(kernel_buf);
	return ret;
}

static ssize_t oppo_tp_suspend_write_func(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	char buffer[5];
	int ret;

	if(size > 2)
		return size;

	if(!ts_g)
		return size;
	
	if(copy_from_user(buffer, buf, size))
	{		
		printk(KERN_INFO "%s: read proc input error.\n", __func__);		
		return size;	
	}
	
	sscanf(buffer, "%d", &ret);
	finish_suspend = ret;
	return size;
}


#define TX_NUM 17
#define RX_NUM 30
#define MIN_ROW 1800
#define MAX_ROW 4800

#define OTHERSUPPERLIMIT 200

//#if TP_TEST_ENABLE
static int synaptics_read_register_map_page1(struct synaptics_ts_data *ts)
{
	unsigned char buf[4];
	int ret;
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x1); 
	if( ret < 0 ){
		TPDTM_DMESG("i2c_smbus_write_byte_data_s3508 failed for page select\n");
		return -1;
	}
	ret = i2c_dma_read(ts->client, 0xE9, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	TPDTM_DMESG("F54_ANALOG_QUERY_BASE = 0x%x\n",F54_ANALOG_QUERY_BASE);
	F54_ANALOG_COMMAND_BASE = buf[1];
	TPDTM_DMESG("F54_ANALOG_COMMAND_BASE = 0x%x\n",F54_ANALOG_COMMAND_BASE);
	F54_ANALOG_CONTROL_BASE = buf[2];
	TPDTM_DMESG("F54_ANALOG_CONTROL_BASE = 0x%x\n",F54_ANALOG_CONTROL_BASE);
	F54_ANALOG_DATA_BASE = buf[3];
	TPDTM_DMESG("F54_ANALOG_DATA_BASE = 0x%x\n",F54_ANALOG_DATA_BASE);

	return 0;
}

static void checkCMD(void)
{
	int ret;
	int flag_err = 0;

	do {
		delay_qt_ms(30); //wait 10ms
		ret = i2c_smbus_read_byte_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE);
		flag_err++;
		TPD_DEBUG("flag_err is %d\n", flag_err);	
	}while( (ret > 0x00) && (flag_err < 30) );
	
	if( ret > 0x00 )
		TPD_ERR("checkCMD error ret is %x flag_err is %d\n", ret, flag_err);	
}
//#endif

//wanghao@phone.bsp, 2016-07-28, add for tp test step2
int  pow1_s3320(uint8_t  n)
{	
	return 1<<n;
}

static ssize_t synaptics_rmi4_baseline_show_s3508(struct device *dev, char *buf, bool savefile)	
{
	int ret = 0;
	int x,y;
	int tx_datal;
	int16_t baseline_data = 0;
	uint8_t tmp_old = 0, tmp_new = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	ssize_t num_read_chars = 0;
	//int16_t Rxdata[RX_NUM][RX_NUM] = {0};
	uint8_t buffer[9];
	uint16_t count = 0;
	int error_count = 0;
	int enable_cbc = 0;
	int16_t *baseline_data_test;
	int fd = -1;
	struct timespec   now_time;
	struct rtc_time   rtc_now_time;
	uint8_t  data_buf[64];
	mm_segment_t old_fs;
	const struct firmware *fw = NULL;
	struct test_header *ph = NULL;
	int eint_status, eint_count = 0;
	char eint_buf[64];
	//wanghao@phone.bsp, 2016-07-28, add for tp test step2
	uint8_t i,j;
	uint8_t buffer_rx[RX_NUM];
	//uint8_t buffer_tx[TX_NUM];
	int sum_line_error = 0;
	unsigned int pow_num=0;
	uint32_t CURRENT_FIRMWARE_ID = 0 ;
	uint8_t FW_buf[4];
	//end 
	
	if(!ts_g){
		num_read_chars += sprintf(&(buf[num_read_chars]), "ts_g is null\n");
		return num_read_chars;
	}
	
	ret = request_firmware(&fw, ts_g->test_limit_name, dev);	
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",ts_g->test_limit_name, ret);		
		error_count++;
		num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n",TP_FW,TP_FW);
		num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"Request firmware failed":"All test passed.");
		return num_read_chars;
	}	
	ph = (struct test_header *)(fw->data);		

	mutex_lock(&ts_g->mutex);	
	disable_irq(touch_irq);//disable_irq_nosync(ts_g->client->irq);	

	//memset(Rxdata,0,sizeof(Rxdata));
	synaptics_read_register_map_page1(ts_g);	

	if(savefile) {
		getnstimeofday(&now_time);
		rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
		sprintf(data_buf, "/sdcard/tp_testlimit_%02d%02d%02d-%02d%02d%02d.csv", 
				(rtc_now_time.tm_year+1900)%100, rtc_now_time.tm_mon+1, rtc_now_time.tm_mday,
				rtc_now_time.tm_hour+8 > 23 ? (rtc_now_time.tm_hour+8-24) : rtc_now_time.tm_hour+8, rtc_now_time.tm_min, rtc_now_time.tm_sec);

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
		if (fd < 0) {            
			TPD_ERR("Open log file '%s' failed.\n", data_buf);
			set_fs(old_fs);
		}		
	}

	//add for TP EINT direct short
	TPD_ERR("synaptics_rmi4_baseline_show_s3508  step 0:check TP EINT direct short\n");
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, 0); 	//enable tp eint pull up
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

	do{
		eint_status = mt_get_gpio_in(GPIO_CTP_EINT_PIN);
		msleep(10);
		TPD_ERR("%s eint_status = %d\n", __func__, eint_status);
		if(eint_status == 1)
			break;
		eint_count++;
	}while(eint_count < 10);

	if(eint_status == 0)
	{
		TPD_ERR("TP EINT PIN direct short!\n");
		error_count++;
		num_read_chars += sprintf(&(buf[num_read_chars]), "eint_status is low, TP EINT direct stort,\n");
		sprintf(&eint_buf[0], "eint_status is low, TP EINT direct stort,\n");
                	sys_write(fd, eint_buf, strlen(eint_buf));
		eint_count = 0;
		goto END;
	}

	//step 1:check raw capacitance.
	TPD_ERR("synaptics_rmi4_baseline_show_s3508  step 1:select report type 0x03\n");
TEST_WITH_CBC_s3508:
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data_s3508 failed \n");
		goto END;
	}

	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_CONTROL_BASE+0x15, 0x01);		//Turn off SignalClarity. ForceUpdate is required for the change to be effective
	ret = i2c_smbus_read_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27);		
	tmp_old = ret&0xff;
	ret = i2c_smbus_read_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x29);
	tmp_new = ret&0xff;
	
	if(enable_cbc){
		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x20));
		ret = synaptics_rmi4_set_page(ts_g->client, 0xff, 0x0); 
		i2c_dma_read(ts_g->client, F34_FLASH_CTRL00, 4, FW_buf); 
		CURRENT_FIRMWARE_ID = FW_buf[3];
		TPD_ERR("CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);	
		TPD_ERR("FW_buf[0] = 0x%x FW_buf[1] = 0x%x FW_buf[2] = 0x%x FW_buf[3] = 0x%x", FW_buf[0], FW_buf[1], FW_buf[2], FW_buf[3]);
		ret = synaptics_rmi4_set_page(ts_g->client, 0xff, 0x1); 
		if(CURRENT_FIRMWARE_ID < 0x28){
			ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x29,(tmp_new | 0x20));	
		}else{
			ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old | 0x20));	
		}
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
		ret = i2c_smbus_write_word_data_s3508(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
		checkCMD();
		TPD_DEBUG("Test open cbc\n");
		baseline_data_test = (uint16_t *)(fw->data + ph->array_limitcbc_offset);	
	}else{
		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xef));
		ret = synaptics_rmi4_set_page(ts_g->client, 0xff, 0x0); 
		i2c_dma_read(ts_g->client, F34_FLASH_CTRL00, 4, FW_buf); 
		CURRENT_FIRMWARE_ID = FW_buf[3];
		TPD_ERR("CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);	
		ret = synaptics_rmi4_set_page(ts_g->client, 0xff, 0x1); 
		if(CURRENT_FIRMWARE_ID < 0x28){	
			ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x29,(tmp_new & 0xdf));
		}else{
			ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old & 0xdf));	
		}
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
		ret = i2c_smbus_write_word_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04);	// force update
		checkCMD();
		baseline_data_test = (uint16_t *)(fw->data + ph->array_limit_offset);		
	}

	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD();
	TPD_DEBUG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data_s3508(ts_g->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();

	count = 0;	
	for(x = 0;x < TX_NUM; x++)  
	{
		for(y = 0; y < RX_NUM; y++) 
		{
			ret = i2c_smbus_read_byte_data_s3508(ts_g->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data_s3508(ts_g->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data = (tmp_h<<8)|tmp_l;
			if (fd >= 0){
                			sprintf(data_buf, "%d,", baseline_data);
                			sys_write(fd, data_buf, strlen(data_buf));
           		 }
			TPD_DEBUG("x = %d y = %d baseline_data is %d\n", x, y, baseline_data);
			
			//TPD_ERR("conut=%d TPD error baseline_data[%d][%d]= %d[%d,%d]\n",count,x,y,baseline_data,*(baseline_data_test+count*2),*(baseline_data_test+count*2+1));
			if((baseline_data < *(baseline_data_test+count*2)) || (baseline_data > *(baseline_data_test+count*2+1)))
			{
				TPD_ERR("TPD error baseline_data[%d][%d]= %d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),	*(baseline_data_test+count*2+1));
				num_read_chars += sprintf(&(buf[num_read_chars]), "conut=%d TPD error baseline_data[%d][%d]= %d[%d,%d]\n",count,x,y,baseline_data,*(baseline_data_test+count*2),*(baseline_data_test+count*2+1));
				error_count++;
				goto END;
			} 
			
			count++;
		}
		if (fd >= 0){
			sys_write(fd, "\n", 1);
		}
		printk("\n");
	}

	if(!enable_cbc){
		enable_cbc = 1;
		TPD_ERR("test cbc baseline again\n");
		goto TEST_WITH_CBC_s3508;
	} 

	ret = i2c_smbus_write_word_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04);
	checkCMD();

	//step 2 :check rx-rx short report
	TPD_ERR("step 2:check rx-rx short report\n" );
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CMD_BASE, 0x01);//software reset TP
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_DATA_BASE, 0x07);//select report type 7	
	ret = i2c_smbus_write_word_data_s3508(ts_g->client, F54_ANALOG_DATA_BASE+1, 0x0);	//set fifo 0
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();	
	tx_datal = i2c_dma_read(ts_g->client, F54_ANALOG_DATA_BASE+3, 7, buffer);

	//wanghao@phone.bsp, 2016-07-28, add for tp test step2
	i2c_smbus_write_byte_data_s3508(ts_g->client, 0xff, 0x3);
	i2c_dma_read(ts_g->client, 0x01, RX_NUM, buffer_rx); 		//save the used rx
	//i2c_dma_read(ts_g->client, 0x02, TX_NUM, buffer_tx); 	
	
	for(x = 0;x < 7; x++)
	{
		if(buffer[x]!=0)
		{
			for(i=0; i<8; i++)
			{
				pow_num=pow1_s3320(i);
				if((pow_num & buffer[x])!=0)
				{
					sum_line_error = 8*x+i;
					for(j=0;j<RX_NUM;j++)
					{
						if(sum_line_error ==buffer_rx[j])
						{
							error_count++;
							TPD_ERR(" step 2 :check tx-to-tx and tx-to-vdd error ,  error_line is rx =%d\n",buffer_rx[j]);
							num_read_chars += sprintf(&(buf[num_read_chars]), " step 2 :check tx-to-tx and tx-to-vdd error ,  error_line is rx =%d\n",buffer_rx[j]);
							goto END;
						}
					}

					/*for(j=0;j<TX_NUM;j++)
					{
						if(sum_line_error ==buffer_tx[j])
						{
							error_count++;
							TPD_ERR(" step 2 :check tx-to-tx and tx-to-vdd error , error_line is tx =%d\n",buffer_tx[j]);
							num_read_chars += sprintf(&(buf[num_read_chars]), " step 2 :check tx-to-tx and tx-to-vdd error , error_line is rx =%d\n",buffer_rx[j]);
							goto END;
						}	
					}*/
				}
			}
		}
	}
	num_read_chars += sprintf(buf, "1");

END:
	if (fd >= 0) {
		sys_close(fd);
		set_fs(old_fs);
	}
	release_firmware(fw);	
	num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n", TP_FW, TP_FW);
	num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);
	delay_qt_ms(60);
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, 0xff, 0x00);
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CMD00, 0x01);
	msleep(150);
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts_g);
#endif	
	synaptics_init_panel(ts_g);
	synaptics_enable_interrupt(ts_g,1);
	enable_irq(touch_irq);	
	TPDTM_DMESG("\n\nstep5 reset and open irq complete\n");
	mutex_unlock(&ts_g->mutex);
	eint_status = 0;
	return num_read_chars;	
}

//static ssize_t tp_delta_show(struct device_driver *ddri, char *buf)
static ssize_t tp_delta_show(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{    
	int ret = 0;
	int x, y;
	char * kernel_buf;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0, tmp_h = 0; 
	uint16_t count = 0;

	if(!ts_g)
		return 0;

	kernel_buf = kmalloc(4096, GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("kmalloc error!\n");
		return 0;
	}
	memset(delta_baseline, 0, sizeof(delta_baseline));
	/*disable irq when read data from IC*/	
	disable_irq(touch_irq);//disable_irq_nosync(ts_g->client->irq);	
	//mutex_lock(&ts_g->mutex);
	
	synaptics_read_register_map_page1(ts_g);
	
	//TPD_DEBUG("\nstep 2:report type2 delta image\n");	
	memset(delta_baseline, 0, sizeof(delta_baseline));
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_DATA_BASE, 0x02);//select report type 0x02
	ret = i2c_smbus_write_word_data_s3508(ts_g->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X01);//get report	
	
	checkCMD();
//	ret = i2c_smbus_read_byte_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE);
//	if(ret > 0x00){
//		TPD_ERR("checkCMD error ret is %x\n", ret);	
//		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CMD_BASE, 0x01);	//software reset
//		goto first;
//	}
//	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X01);//get report	
//	checkCMD();
//	if(ret < 0)
//		TPD_ERR("get report error ret is %d\n", ret);	
	
	count = 0;
	for( x = 0; x < TX_NUM; x++ ){
		//printk("\n[%d]", x);
		num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "\n[%3d]", x);
		for( y = 0; y < RX_NUM; y++ ){
			ret = i2c_smbus_read_byte_data_s3508(ts_g->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data_s3508(ts_g->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;       		
			//printk("%3d,", delta_baseline[x][y]);
			num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%3d ", delta_baseline[x][y]);
		}	 
	}
	delay_qt_ms(60);
	synaptics_enable_interrupt(ts_g, 1);

	//mutex_unlock(&ts_g->mutex);
	enable_irq(touch_irq); //enable_irq(ts_g->client->irq);
	
	TPD_ERR("num_read_chars = %zd , count = %zd\n", num_read_chars, size);
	num_read_chars += sprintf( &( kernel_buf[num_read_chars] ), "%s" , "\r\n" );
	ret = simple_read_from_buffer(buf, size, ppos, kernel_buf, strlen(kernel_buf));
	kfree(kernel_buf);
	
	return ret;

}

static int	synaptics_input_init(struct synaptics_ts_data *ts)
{
	int i = 0;

	TPD_DEBUG("%s is called\n",__func__);

	for( i = 0; i < ts->max_num; i++ )
		pre_touch[i] = 0;

	ts->input_dev->name = TPD_DEVICE;;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef SUPPORT_GESTURE
	set_bit(KEY_F4 , ts->input_dev->keybit);//doulbe-tap resume
#endif

	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);	

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 1080, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 1920, 0, 0);		

#ifdef TYPE_B_PROTOCOL
	ts->max_num = MAX_POINT_NUM;
	input_mt_init_slots(ts->input_dev, ts->max_num, 0);
#endif	
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_HOMEPAGE, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	input_set_drvdata(ts->input_dev, ts);

	if(input_register_device(ts->input_dev)) {
		TPD_ERR("%s: Failed to register input device\n",__func__);			
		input_unregister_device(ts->input_dev);
		input_free_device(ts->input_dev);	
		return -1;
	}
	return 0;
}
static int synaptics_device_irq_init(struct synaptics_ts_data *ts)
{
	int ret = 0;
	struct device_node *node = NULL;
	
	u32 intr[2] = {0,0};
	//wanghao 2015/10/21 for irq_registration change   
	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if(node){
		of_property_read_u32_array(node , "interrupts", intr, ARRAY_SIZE(intr));
		pr_err("synaptics_device_irq_init intr[0]  = %d, intr[1]  = %d\r\n",intr[0] ,intr[1] );
		touch_irq = irq_of_parse_and_map(node, 0);
	}
	else{
		pr_err("synaptics_device_irq_init node not exist!\r\n");
	}
	ret = request_irq(touch_irq, (irq_handler_t)synaptics_ts_irq_handler, EINTF_TRIGGER_LOW, "TOUCH_PANEL-eint",  NULL);
	if(ret){
		TPD_DEBUG("ret = %d\n", ret);
		TPD_DEBUG("synaptics_ts_probe: failed to request_irq \n");
		return -1;
	}

	ret = synaptics_enable_interrupt(ts, 1);
	if(ret) {
		TPD_DEBUG("synaptics_ts_probe: failed to enable synaptics  interrupt \n");
		free_irq(ts->client->irq, ts);
		return -1;
	}
	//synaptics_glove_mode_enable(ts);
	/*config tm1429: set report rate, sleep mode */
	ret = synaptics_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		TPD_DEBUG("synaptics_init_panel failed\n");	
	}

	return ret;
}
/*********************Added for tp FW update******************************************/
static int synatpitcs_fw_update(struct device *dev, bool force)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int ret;
	char fw_id_temp[12];
	uint8_t buf[4];
	static int call_times = 0;
	uint32_t CURRENT_FIRMWARE_ID = 0 ;

	//TPD_DEBUG("%s is called, TP get lcd id : x%x\n",__func__,lcd_id);
	TPDTM_DMESG("synatpitcs_fw_update: fw_name = %s time = %d\n", ts->fw_name, call_times);
	if(!ts->client){
		TPD_ERR("i2c client point is NULL\n");	
		return 0;
	}
	ret = request_firmware(&fw, ts->fw_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",
				ts->fw_name, ret);
		return ret;
	}
 	synaptics_rmi4_free_fingers(ts); //mingqiang.guo add for finger can not up 
	ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force); 
	if(ret < 0){
		TPD_ERR("FW update not success try again\n");
		ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
		if(ret < 0){
			TPD_ERR("FW update failed twice, quit updating process!\n");
			return ret;
		}
	}
	release_firmware(fw);	

	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0);	
	ret = i2c_dma_read(ts->client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	sprintf(fw_id_temp,"0x%x",CURRENT_FIRMWARE_ID);
	strcpy(ts->fw_id,fw_id_temp);
	if(force_update == 1){
		ret = synaptics_input_init(ts);
		if(ret < 0){
			TPD_ERR("synaptics_input_init failed!\n");			
		}
		ret = synaptics_tpd_button_init(ts);	
		if(ret < 0){
			TPD_ERR("synaptics_tpd_button_init failed!\n");			
		}
		ret = synaptics_device_irq_init(ts);
		if(ret < 0){
			TPD_ERR("synaptics_device_irq_init failed!\n");			
		}
	}else{
#ifdef SUPPORT_GLOVES_MODE
		synaptics_glove_mode_enable(ts);
#endif	
		synaptics_init_panel(ts);
		synaptics_enable_interrupt(ts,1);	
	}

	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);	
	input_sync(ts->input_dev);
	call_times++;
	return 0;
}

/*static ssize_t synaptics_update_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}*/

static ssize_t synaptics_update_fw_store(struct file *file, const char __user *buf, size_t size, loff_t *lo)
{
	struct synaptics_ts_data *ts = ts_g;
	int val = 0;
	int rc;
	char k_buf[10];

	if (size > 2)
		return -EINVAL;

	memset(k_buf, 0, 10);
	if( copy_from_user(k_buf, buf, size) ){
		printk(KERN_INFO "%s: read fw_update input error.\n", __func__);
		TPD_DEBUG("buf is %s, k_buf is %s\n",buf, k_buf);
		return size;
	}
	TPD_DEBUG("k_buf = %s", k_buf);
	rc = sscanf(k_buf, "%d", &val);
	TPD_DEBUG(TPD_DEVICE" synaptics:start update fw , k_buf = %s , rc = %d val = %d\n",k_buf, rc, val);	
	
	if (rc < 0)
		return rc;
	if(!ts->dev){
		TPD_ERR(TPD_DEVICE" dev is NULL return!!!!!\n");
		return size;		
	}
	if (tp_probe_ok==0) {
		TPD_ERR(TPD_DEVICE"probe not finished return!!!! %d %d\n", __LINE__, tp_probe_ok);
		return size;
	}
	if((val == 1)||(force_update==1))
	{
		val = 1;
	}else{
		val = 0;
	}

	if(is_suspend == 1)
		return -1;

	mutex_lock(&ts->mutex);
	if(force_update == 0){
		disable_irq(touch_irq);//	disable_irq_nosync(ts->client->irq);	
		mutex_lock(&ts->input_dev->mutex);
	}
	TPD_DEBUG("synaptics:loading_fw %x\n",ts->loading_fw);	
	if (!ts->loading_fw) {
		ts->loading_fw = true;
		synatpitcs_fw_update(ts_g->dev, val);
		ts->loading_fw = false;
	}
	if(force_update == 0){
		mutex_unlock(&ts->input_dev->mutex);	
		enable_irq(touch_irq);	//	enable_irq(ts->client->irq);
	}
	force_update = 0;
	mutex_unlock(&ts->mutex);
	return size;
}
/****************************End*************************************/
static void re_scan_PDT(struct i2c_client *client)
{
	uint8_t buf[8];

	i2c_dma_read(client, 0xE9, 6,  buf);
	SynaF34DataBase = buf[3];
	SynaF34QueryBase = buf[0];
	i2c_dma_read(client, 0xE3, 6,  buf);
	SynaF01DataBase = buf[3];
	SynaF01CommandBase = buf[1];
	i2c_dma_read(client, 0xDD, 6,  buf);
	SynaF34Reflash_BlockNum = SynaF34DataBase;
	SynaF34Reflash_BlockData = SynaF34DataBase + 1;
	SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 1;
	SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 2;
	SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +3;
	SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
	SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 3;
	i2c_dma_read(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
	SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	TPD_DEBUG("SynaFirmwareBlockSize 3310 is %d\n", SynaFirmwareBlockSize);
	SynaF34_FlashControl = SynaF34DataBase + 2;

}

static int checkFlashState(void)
{
	int ret ;
	int count = 0;
	int FLSH_STATUS = 0;
	FLSH_STATUS = SynaF34_FlashControl+1;
	//msleep(5);
	ret =  i2c_smbus_read_byte_data_s3508(ts_g->client,FLSH_STATUS);
	while ( (ret != 0x80)&&(count < 8) ) {
		msleep(3); //wait 3ms
		TPD_DEBUG(" ret is %x\n", ret);
		ret =  i2c_smbus_read_byte_data_s3508(ts_g->client,FLSH_STATUS);
		count++;
	} 
	if(count == 8)
		return 1;
	else
		return 0;
}


static int synaptics_fw_check(struct synaptics_ts_data *ts )
{
	int ret;
	uint8_t buf[4];
	uint32_t bootloader_mode;
	int max_y_ic = 0;

	if(!ts){
		TPD_ERR("%s ts is NULL\n",__func__);	
		return -1;
	}

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}

	/*read product id */
	ret = synaptics_read_product_id(ts);
	if(ret) {
		TPD_ERR("failed to read product info \n");
		return -1;
	}
	/*read max_x ,max_y*/
	ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0);
	if (ret < 0) {
		ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x0);
		if(ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data_s3508 failed for page select\n");
			return -1;
		}
	}



	//i2c_smbus_read_i2c_block_data(ts->client, F12_2D_CTRL08, 14, buf);
	i2c_dma_read(ts->client, F12_2D_CTRL08, 4, &(buf[0x0]));
	ts->max_x = ( (buf[1]<<8)&0xffff ) | (buf[0]&0xffff);       
	max_y_ic = ( (buf[3]<<8)&0xffff ) | (buf[2]&0xffff); 
	ts->max_y = max_y_ic ;
	TPDTM_DMESG("buf[0] = 0x%x, buf[1] = 0x%x,ts->max_x=%d,ts->max_y=%d\n", buf[0], buf[1],ts->max_x,ts->max_y);

	TPD_ERR("max_x = %d,max_y = %d\n",ts->max_x,max_y_ic);

	bootloader_mode = i2c_smbus_read_byte_data_s3508(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 	

	if((ts->max_x == 0)||(max_y_ic == 0)||(bootloader_mode == 0x40)) {
		TPD_ERR("Something terrible wrong \n Trying Update the Firmware again\n");		
		return -1;
	}
	TPD_DEBUG("synaptics_fw_check ok  bootloader_mode = 0x%x\n",bootloader_mode);	
	return 0;
}
static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force)
{
	uint16_t block,firmware,configuration;
	uint8_t buf[8];
	int ret,i,j;
	//	uint8_t data[0]={0};
	uint8_t bootloder_id[10];
	const uint8_t *Firmware_Data = NULL;
	const uint8_t *Config_Data = NULL;
	struct image_header_data header;
	uint32_t CURRENT_FIRMWARE_ID = 0 , FIRMWARE_ID = 0;
	struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);	

	TPD_DEBUG("%s is called\n",__func__);
	if(!client)
		return -1;

	parse_header(&header,data);	
	if((header.firmware_size + header.config_size + 0x100) > data_len) {
		TPDTM_DMESG("firmware_size + config_size + 0x100 > data_len data_len = %d \n",data_len);
		return -1;
	}

	Firmware_Data = data + 0x100;
	Config_Data = Firmware_Data + header.firmware_size;
	ret = i2c_smbus_write_byte_data_s3508(client, 0xff, 0x0);	

	ret = i2c_dma_read(client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];
	TPD_ERR("before updtate :  TP IC FIRMWARE_ID = %x , will update FIRMWARE_ID = %x\n",     
			CURRENT_FIRMWARE_ID, FIRMWARE_ID);                                               
	TPD_ERR("synaptics force is %d\n", force);                                                 
	if(!force) {                                                                               
		if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {                                               
			return 0;                                                                          
		}                                                                                      
	}                                                                                          

	re_scan_PDT(client);
	block = 16;
	TPD_DEBUG("block is %d \n",block);
	firmware = (header.firmware_size)/16;
	TPD_DEBUG("firmware is %d \n",firmware);
	configuration = (header.config_size)/16;
	TPD_DEBUG("configuration is %d \n",configuration);

	//Enter Flash
	//step1 read bootloader Id
	ret = i2c_dma_read(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));  
	TPD_DEBUG("bootloader id is %x \n",(bootloder_id[1] << 8)|bootloder_id[0]);
	ret=i2c_dma_write(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
	TPD_DEBUG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n",SynaF34_FlashControl,ret);

	//step 3 Issue programe enable
	i2c_smbus_write_byte_data_s3508(client,SynaF34_FlashControl,0x0F);
	//step 4 wait attn
	msleep(10);	
	//SynaWaitATTN();
	TPD_DEBUG("attn step 4\n");
	ret=checkFlashState();
	if(ret > 0) {
		TPD_ERR("Get in prog:The status(Image) of flashstate is %x\n",ret);
		return -1;
	}
	ret = i2c_smbus_read_byte_data_s3508(client,0x04);
	TPD_DEBUG("The status(device state) is %x\n",ret);
	ret= i2c_smbus_read_byte_data_s3508(client,F01_RMI_CTRL_BASE);
	TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
	ret= i2c_smbus_write_byte_data_s3508(client,F01_RMI_CTRL_BASE,ret&0x04);
	/********************get into prog end************/

	ret=i2c_dma_write(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
	//step 6 Re-scan PDT
	re_scan_PDT(client);
	//program
	//step1 read bootloader id
	i2c_dma_read(client,SynaF34ReflashQuery_BootID,2,buf);
	//step2 write bootloader id
	i2c_dma_write(client,SynaF34Reflash_BlockData,2,buf);
	//step 3 erase
	i2c_smbus_write_byte_data_s3508(client,SynaF34_FlashControl,0x03);
	//step 4 wait attn
	msleep(2500);
	ret = i2c_smbus_read_byte_data_s3508(client, SynaF34_FlashControl);
	if(ret != 0x00)
		msleep(2000);

	//step 5 check status, the value should be 0x80
	ret = i2c_smbus_read_byte_data_s3508(client,SynaF34_FlashControl+1);
	TPDTM_DMESG("The status(erase) is %x\n",ret);		

	//step 6 image area
	TPD_ERR("tp start update-----------------!\n");
	TPD_DEBUG("cnt %d\n",firmware);
	for(j=0; j<firmware; j++) 
	{
		//a) write block number
		buf[0]=j&0x00ff;
		buf[1]=(j&0xff00)>>8;
		i2c_dma_write(client,SynaF34Reflash_BlockNum,2,buf);
		i2c_dma_write(client,SynaF34Reflash_BlockData,16,(uint8_t *)&Firmware_Data[j*16]); 
		i2c_smbus_write_byte_data_s3508(client, SynaF34_FlashControl, 0x02);
		ret=checkFlashState();		
		if(ret > 0) {
			TPD_ERR("Firmware:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
			return -1;
		}
	}
	//step 7 configure data
	for(j=0;j<configuration;j++)
	{
		//a)
		buf[0]=j&0x00ff;
		buf[1]=(j&0xff00)>>8;
		i2c_dma_write(client,SynaF34Reflash_BlockNum,2,buf);
		//b) write data
		i2c_dma_write(client,SynaF34Reflash_BlockData,16,(uint8_t *)&Config_Data[j*16]);
		//c) issue write
		i2c_smbus_write_byte_data_s3508(client,SynaF34_FlashControl,0x06);
		//d) wait attn
		ret = checkFlashState();	
		if(ret > 0) {
			TPD_ERR("Configuration:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
			return -1;
		}
	}
	//step 1 issue software reset
	i2c_smbus_write_byte_data_s3508(client,SynaF01CommandBase,0X01);
	//step2 wait ATTN
	//delay_qt_ms(1000);
	mdelay(1500);
	synaptics_read_register_map(ts);
	//FW flash check!
	ret =synaptics_fw_check(ts);
	if(ret < 0 ) {
		TPD_ERR("Firmware self check failed\n");
		return -1;
	}
	TPD_DEBUG("Firmware self check Ok\n");

	//after update , get tp firmware id	
	ret = i2c_dma_read(client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	TP_FW = CURRENT_FIRMWARE_ID;
	oppo_tp_dev = TP_SAMSUNG_SYNAPTICS;
	TPD_ERR(TPD_DEVICE"after updtate :  TP IC FIRMWARE_ID = %x , TP_FW = %x\n", CURRENT_FIRMWARE_ID,TP_FW);     

#if 1 //mingqiang.guo 2015/6/2 add for finger can not up 
	for( i = 0; i < ts->max_num; i++ )
	{
		#ifdef TYPE_B_PROTOCOL
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 1);
		#endif				

		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_report_key(ts->input_dev,	BTN_TOOL_FINGER, 1);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 20);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 20);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 0);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 0);
	}
	input_sync(ts->input_dev);

	
	for( i = 0; i < ts->max_num; i++ )
	{
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev,MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev,BTN_TOUCH, 0);
	input_report_key(ts->input_dev,BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);

	is_key_touch = 0;
	is_lcd_touch = 0;
	
	//write_back_reg_status(ts);
#endif 

	return 0;
}
//static ssize_t tp_baseline_show(struct device_driver *ddri, char *buf)
static ssize_t tp_baseline_show(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{

	int ret = 0;
	int x,y;
	char * kernel_buf;
	ssize_t num_read_chars = 0;
	//uint8_t tmp = 0;
	uint8_t tmp_old = 0, tmp_new = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint16_t count = 0;
	int16_t baseline_data[TX_NUM][RX_NUM] = {{0}, {0}};	


	if(is_suspend == 1)
		return count;
	if(!ts_g)
		return count;

	kernel_buf = kmalloc(4096, GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("kmalloc error!\n");
		return 0;
	}

	memset(delta_baseline, 0, sizeof(delta_baseline));  
	disable_irq(touch_irq);
	synaptics_read_register_map_page1(ts_g);
	mutex_lock(&ts_g->mutex);


	TPDTM_DMESG("\nshirendong test start\n");
	TPDTM_DMESG("\nstep 1:select report type 0x03 baseline\n");
#if 1
	//step 1:check raw capacitance. 
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPD_DEBUG("read_baseline: i2c_smbus_write_byte_data_s3508 failed \n");
		//return sprintf(buf, "i2c err!");
	}


	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_CONTROL_BASE+0x15, 0x01);		//Turn off SignalClarity. ForceUpdate is required for the change to be effective
	ret = i2c_smbus_read_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27);
	tmp_old = ret&0xff;
	ret = i2c_smbus_read_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x29);
	tmp_new = ret&0xff;
	
	TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xef));
	if(TP_FW < 0xad031028){	
		TPD_ERR("0x29 TP_FW = 0x%x\n", TP_FW);
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x29,(tmp_new & 0xdf));
	}else{
		TPD_ERR("0x27 TP_FW = 0x%x\n", TP_FW);
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old & 0xdf));	
	}
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
	ret = i2c_smbus_write_word_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update 
	checkCMD();
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD();
	//TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data_s3508(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report 
	checkCMD();
	count = 0;

	for(x = 0;x < TX_NUM; x++)
	{
		TPDTM_DMESG("\n[%2d]",x);  
		num_read_chars += sprintf(kernel_buf+num_read_chars,"\n[%2d]",x);

		for(y = 0; y < RX_NUM; y++)
		{
			ret = i2c_smbus_read_byte_data_s3508(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data_s3508(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data[x][y] = (tmp_h<<8)|tmp_l;

			TPDTM_DMESG("%d,",baseline_data[x][y]);
			num_read_chars += sprintf(kernel_buf+num_read_chars,"%5d",baseline_data[x][y]);
		}
	}
#endif

	TPDTM_DMESG("\nreport type2 delta image\n");

	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
	delay_qt_ms(60);
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts_g);
#endif
	ret = synaptics_soft_reset(ts_g);
	delay_qt_ms(60);
	synaptics_enable_interrupt(ts_g,1);
	mutex_unlock(&ts_g->mutex);
	enable_irq(touch_irq);
	TPDTM_DMESG("\nreport delta image end\n");
	TPD_ERR("num_read_chars = %zd , size = %zd\n", num_read_chars, size);
	num_read_chars += sprintf( &( kernel_buf[num_read_chars] ), "%s" , "\r\n" );
	ret = simple_read_from_buffer(buf, size, ppos, kernel_buf, strlen(kernel_buf));
	kfree(kernel_buf);
	
	return ret;

}

//static ssize_t tp_baseline_show_with_cbc(struct device_driver *ddri, char *buf)
static ssize_t tp_baseline_show_with_cbc(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int ret = 0;
	int x,y;
	char * kernel_buf;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint8_t tmp_old;
	uint16_t count = 0;


	if(is_suspend == 1)
		return count;
	if(!ts_g)
		return count;

	kernel_buf = kmalloc(4096, GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("kmalloc error!\n");
		return 0;
	}

	memset(delta_baseline,0,sizeof(delta_baseline));
	/*disable irq when read data from IC*/	
	disable_irq(touch_irq);	// disable_irq_nosync(ts_g->client->irq);		
	synaptics_read_register_map_page1(ts_g);
	mutex_lock(&ts_g->mutex);
	TPDTM_DMESG("\nstep 1:select report type 0x03 baseline\n");
	//step 1:check raw capacitance.
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPDTM_DMESG("step 1: select report type 0x03 failed \n");
		//return sprintf(buf, "i2c err!");
	}
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_CONTROL_BASE+0x15, 0x01);	//Turn off SignalClarity. ForceUpdate is required for the change to be effective
	ret = i2c_smbus_read_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27);
	tmp_old = ret&0xff;
	TPDTM_DMESG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x20));
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old | 0x20));
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force update
	checkCMD();
	

	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);//Force Cal, F54_ANALOG_CMD00
	checkCMD();
	TPDTM_DMESG("Force Cal oK\n");

	ret = i2c_smbus_write_word_data_s3508(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++) {   	
		TPDTM_DMESG("\n[%2d]",x);
		num_read_chars += sprintf( kernel_buf + num_read_chars , "\n[%2d]",x);

		for(y = 0; y < RX_NUM; y++){
			ret = i2c_smbus_read_byte_data_s3508(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data_s3508(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;
			TPDTM_DMESG("%d,",delta_baseline[x][y]);	
			num_read_chars += sprintf( kernel_buf + num_read_chars , "%5d",delta_baseline[x][y]);
		}
	}

	ret = i2c_smbus_write_byte_data_s3508(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
	delay_qt_ms(60);
	ret = synaptics_soft_reset(ts_g);
	delay_qt_ms(60);
	synaptics_enable_interrupt(ts_g,1);
	enable_irq(touch_irq);	//enable_irq(ts_g->client->irq);
	TPD_ERR("num_read_chars = %zd , count = %zd\n", num_read_chars, size);
	num_read_chars += sprintf( &( kernel_buf[num_read_chars] ), "%s" , "\r\n" );
	ret = simple_read_from_buffer(buf, size, ppos, kernel_buf, strlen(kernel_buf));
	kfree(kernel_buf);
	mutex_unlock(&ts_g->mutex);
	return ret;
}



static struct kobj_attribute mtk_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.mtk-tpd",
		.mode = S_IRUGO,
	},
	.show = &cap_vk_show,
};

static struct attribute *mtk_properties_attrs[] = {
	&mtk_virtual_keys_attr.attr,
	NULL
};


static struct attribute_group mtk_properties_attr_group = {
	.attrs = mtk_properties_attrs,
};

static struct kobject *syna_properties_kobj;

static int synaptics_tpd_button_init(struct synaptics_ts_data *ts)
{
	int ret = 0;

	ts->kpd = input_allocate_device();
	if (ts->kpd == NULL) 
	{
		ret = -ENOMEM;
		TPDTM_DMESG(KERN_ERR "synaptics_tpd_button_init: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->kpd->name = TPD_DEVICE "-kpd";
	set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);
	ts->kpd->id.bustype = BUS_HOST;
	ts->kpd->id.vendor  = 0x0001;
	ts->kpd->id.product = 0x0001;
	ts->kpd->id.version = 0x0100;

	if(input_register_device(ts->kpd))
		TPD_DEBUG("input_register_device failed.(kpd)\n");
	set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);

	syna_properties_kobj = kobject_create_and_add("board_properties", NULL);
	if(syna_properties_kobj)
		ret = sysfs_create_group(syna_properties_kobj,&mtk_properties_attr_group);
	if(!syna_properties_kobj || ret)
		TPDTM_DMESG("failed to create board_properties\n");	

err_input_dev_alloc_failed:		
	return ret;

}



#ifdef SUPPORT_GESTURE
static ssize_t tp_double_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	TPD_DEBUG("double tap enable is: %d\n", atomic_read(&double_enable));
	ret = sprintf(page, "%d\n", atomic_read(&double_enable));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page)); 
	return ret;
}

static ssize_t tp_double_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{ 
	int ret = 0;
	char buf[10];
	
	if( count > 2) 
		return count;
	
	if( copy_from_user(buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	if(!ts_g)
		return count;
	
	sscanf(buf, "%d", &ret);
	TPD_ERR(" %s %d , double_enable=%d , gesture_enable=%d\n",__func__,ret,atomic_read(&double_enable),gesture_enable );

	mutex_lock(&ts_g->mutex);   
	if( (ret == 0 )||(ret == 1) ||(ret == 2) ) 
		atomic_set(&double_enable, ret);

	if( gesture_enable == 1 )//when gesture_enable==1 , write tp black gesture enable 
	{
		TPD_ERR(" on-off : gesture_enable == %d \n", gesture_enable);
		switch ( ret )
		{
			case 0:
			case 2:
				TPD_ERR("tp_guesture_func will be disable\n");
				ret = synaptics_enable_interrupt_for_gesture(ts_g, 0); 
				if( ret<0 )
					ret = synaptics_enable_interrupt_for_gesture(ts_g, 0); 
				ret = synaptics_enable_interrupt(ts_g, 0);
				if(ret < 0){
					TPD_ERR("%s: cannot disable interrupt\n", __func__);
					break;
				}
				ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CTRL00, 0x01); 
				if( ret < 0 ){
					TPD_ERR("write F01_RMI_CTRL00 failed\n");
					mutex_unlock(&ts_g->mutex);
					return -1;
				}
				is_suspend = 1;
				break;
			case 1:
				TPD_ERR("tp_guesture_func will be enable\n");
				ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CTRL00, 0x80);
				if( ret < 0 ){
					TPD_ERR("write F01_RMI_CTRL00 failed\n");
					mutex_unlock(&ts_g->mutex);
					return -1;
				}
				is_suspend = 0;
				ret = synaptics_enable_interrupt_for_gesture(ts_g, 1); 
				if( ret<0 )
					ret = synaptics_enable_interrupt_for_gesture(ts_g, 1); 
				ret = synaptics_enable_interrupt(ts_g, 1);
				if(ret < 0){
					TPD_DEBUG("%s: cannot enable interrupt\n", __func__);
					break;
				}
				break;
			default:
				TPD_ERR("Please enter 0 or 1 to open or close the double-tap function\n");
		}
		
	}
	mutex_unlock(&ts_g->mutex);

	return count;
}

static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{	
	int ret = 0;
	char page[PAGESIZE];
	ret = sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
			Point_start.x, Point_start.y, Point_end.x, Point_end.y,
			Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
			Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
			clockwise);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;	
}

/******************************start****************************/
static const struct file_operations tp_double_proc_fops = {
	.write = tp_double_write_func,
	.read =  tp_double_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations coordinate_proc_fops = {
	.read =  coordinate_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#endif

#ifdef SUPPORT_GLOVES_MODE
static int tp_glove_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	printk("glove mode enable is: %d\n", atomic_read(&glove_enable));
	ret = sprintf(page, "%d\n", atomic_read(&glove_enable));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static int tp_glove_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0 ;
	char buf[10];

	if(!ts_g)
		return count;
	
	mutex_lock(&ts_g->mutex);
	if( count > 10 )
		goto GLOVE_ENABLE_END;
	if( copy_from_user( buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);	
		goto GLOVE_ENABLE_END;
	}
	sscanf(buf, "%d", &ret);
	ts = ts_g;
	TPDTM_DMESG("tp_glove_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){	
		atomic_set(&glove_enable, ret);
		synaptics_glove_mode_enable(ts);
	}	
	switch(ret){	
		case 0:	
			TPDTM_DMESG("tp_glove_func will be disable\n");
			break;
		case 1:	
			TPDTM_DMESG("tp_glove_func will be enable\n");
			break;		
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the glove function\n");
	}
GLOVE_ENABLE_END:
	mutex_unlock(&ts_g->mutex);
	return count;
}

static const struct file_operations glove_mode_enable_proc_fops = {
	.write = tp_glove_write_func,
	.read =  tp_glove_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif 

#ifdef SUPPORT_LEATHER
static int oppo_leather_mode_enabled_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	printk("oppo_leather_mode_enabled = %d\n", atomic_read(&leather_enable_flag));
	ret = sprintf(page, "%d\n", atomic_read(&leather_enable_flag));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static int oppo_leather_mode_enabled_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts;
	int ret = 0 ;
	char buf[10];
	
	if( copy_from_user( buf, buffer, count) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);	
		return count;
	}
	sscanf(buf, "%d", &ret);

	TPD_ERR("tp_glove_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){	
		atomic_set(&leather_enable_flag, ret);
	}	

	return count;
}

static const struct file_operations oppo_leather_mode_enabled_proc_fops = {
	.write = oppo_leather_mode_enabled_write_func,
	.read =  oppo_leather_mode_enabled_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif 


#ifdef SUPPORT_TP_SLEEP_MODE

static ssize_t tp_sleep_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	printk("sleep mode enable is: %d\n", atomic_read(&sleep_enable));

	ret = i2c_smbus_read_byte_data_s3508(ts_g->client, F01_RMI_CTRL00);
	TPD_DEBUG(" read from address F01_RMI_CTRL00 0x%x  = 0x%x \n",F01_RMI_CTRL00,ret);
	ret = sprintf(page, "read from address F01_RMI_CTRL00 0x%x = 0x%x ; bit2=1(no sleep), bit2=0(sleep) \n",F01_RMI_CTRL00,ret);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;

}

static ssize_t tp_sleep_write_func(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts; 
	int ret = 0 ;
	char buf[10] = {0};
	if (count > 10) 
		return count;
	if (copy_from_user( buf, buffer, count)) {
		TPDTM_DMESG(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	sscanf(buf,"%d",&ret);
	ts = ts_g;
	TPDTM_DMESG("tp_sleep_write_func:buf = %d,ret = %d\n",*buf,ret);
	if((ret == 0 )||(ret == 1))
	{
		//  TPDTM_DMESG("tp_glove_write_func called\n");

		atomic_set(&sleep_enable,ret);

		synaptics_sleep_mode_enable(ts);		 
	}

	switch(ret)
	{	 case 0:
		TPDTM_DMESG("tp_sleep_func will be disable\n");
		break;
		case 1:
		TPDTM_DMESG("tp_sleep_func will be enable\n");
		break;
		default:TPDTM_DMESG("Please enter 0 or 1 to open or close the sleep function\n");
	}
	return count;
}

#endif

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

/*----------------------------------------------------------------------------*/
static struct i2c_driver tpd_i2c_driver = {
	.probe		= synaptics_ts_probe,
	//.remove		= __devexit_p(synaptics_ts_remove),
	//.detect         = synaptics_ts_detect,                           
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	//.address_data = &synaptics_addr_data,
	.driver = {
		//.owner    = THIS_MODULE,
		.name	= TPD_DEVICE,
	},
};

//mingqiang.guo@bsp add for tp auto test  
static ssize_t tp_baseline_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	char page[PAGESIZE];	
	if(!ts_g)		
		return baseline_ret;	
	if(baseline_ret == 0) //proc node will auto read two time, but only need tp test one time 
	{		
		count = synaptics_rmi4_baseline_show_s3508(ts_g->dev,page,1);		
		baseline_ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));		
	}
	else
	{		
		baseline_ret = 0;	
	}	
	return baseline_ret;

}

static ssize_t limit_area_show(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret =0;
	char page[PAGESIZE];

	TPD_DEBUG("limit_area is: %d\n", limit_area);
	ret = sprintf(page, "limit_area = %d left_x1 = %d right_x1 = %d left_x2 = %d right_x2 = %d\n", limit_area, left_x1, right_x1, left_x2, right_x2);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t limit_area_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[16];
	int temp;

	if(copy_from_user(buf, buffer, count))
	{
		TPDTM_DMESG(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	
	sscanf(buf, "%d", &temp);
	if(temp < 0 || temp >9)
		return count;
	
	limit_area = temp;
	left_x1 = (limit_area*1000)/100;
	right_x1 = LCD_WIDTH - left_x1;
	left_x2 = 2 * left_x1;
	right_x2 = LCD_WIDTH -2*left_x1;
	TPD_ERR("limit_area = %d left_x1 = %d right_x1 = %d left_x2 =%d right_x2 = %d\n", limit_area, left_x1, right_x1, left_x2, right_x2);
	return count;
}

static ssize_t limit_enable_show(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret =0;
	char page[16];

	TPD_DEBUG("limit_enable is: %d\n", limit_enable);
	ret = sprintf(page, "%d\n", limit_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t limit_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8];
	int temp = 0;

	if( count > 2 || ts_g == NULL) 
		return count;
	
	if(copy_from_user(buf, buffer, count))
	{
		TPDTM_DMESG(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	
	if(*buf == '1')
		temp = 1;
	else if(*buf == '0')
		temp = 0;
	TPD_ERR("temp = %d\n", temp);
	limit_enable = temp;
	queue_work(synaptics_wq, &ts_g->close_edge_product);
	return count;
}

void close_edge_protect_function_s3320(struct work_struct *work)
{
	int ret;
	
	if(ts_g == NULL)
	{
		TPD_ERR("ts_g is NULL!\n");
		return;
	}
	TPD_ERR("%s tricked!\n", __func__);	
	mutex_lock(&ts_g->mutex);
	synaptics_rmi4_set_page(ts_g->client, 0xff, 0x4); 
	if(0 == limit_enable)
	{
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F51_CUSTOM_CTRL_BASE + 0x09, 0x46);
	}
	else if(1 == limit_enable)
	{
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F51_CUSTOM_CTRL_BASE + 0x09, 0x47);
	}
	synaptics_rmi4_set_page(ts_g->client, 0xff, 0x0); 
	mutex_unlock(&ts_g->mutex);
}

static const struct file_operations tp_baseline_test_proc_fops = 
{
	.read = tp_baseline_test_read_func,
	.owner = THIS_MODULE,
};
//mingqiang.guo@bsp add for tp auto test   end 
static const struct file_operations sleep_mode_enable_proc_fops = 
{
	.read  = tp_sleep_read_func ,
	.write = tp_sleep_write_func,
	.owner = THIS_MODULE,
};

static const struct file_operations oppo_tp_delta_data_proc_fops = 
{
	.read = tp_delta_show,
	.owner = THIS_MODULE,
};

static const struct file_operations oppo_tp_baseline_image_proc_fops = 
{
	.read = tp_baseline_show,
	.owner = THIS_MODULE,
};

static const struct file_operations oppo_tp_baseline_image_with_cbc_proc_fops = 
{
	.read = tp_baseline_show_with_cbc,
	.owner = THIS_MODULE,
};

static const struct file_operations oppo_tp_debug_proc_fops = 
{
	.read = tp_show,
	.write = store_tp,
	.owner = THIS_MODULE,
};

static const struct file_operations proc_firmware_update =
{
	.write = synaptics_update_fw_store,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations proc_limit_area =
{
	.read = limit_area_show,
	.write = limit_area_write,
	.owner = THIS_MODULE,	
};


static const struct file_operations proc_limit_enable =
{
	.read = limit_enable_show,
	.write = limit_enable_write,
	.owner = THIS_MODULE,	
};

static const struct file_operations oppo_tp_suspend =
{
	.read = oppo_tp_suspend_read_func,
	.write = oppo_tp_suspend_write_func,
	.owner = THIS_MODULE,
};

static int init_synaptics_proc(struct synaptics_ts_data *ts)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_tp = NULL; 
	struct proc_dir_entry *prEntry_temp = NULL; 

	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if(prEntry_tp == NULL)
	{
		ret = -ENOMEM;
		TPD_DEBUG(KERN_INFO"init_synaptics_proc: Couldn't create TP proc entry\n");
	}
#ifdef SUPPORT_GESTURE
	prEntry_temp = proc_create( "double_tap_enable", 0666, prEntry_tp, &tp_double_proc_fops);
	if(prEntry_temp == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}	
	prEntry_temp = proc_create("coordinate", 0444, prEntry_tp, &coordinate_proc_fops);
	if(prEntry_temp == NULL){	   
		ret = -ENOMEM;	   
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
#endif	

#ifdef SUPPORT_LEATHER
	prEntry_temp = proc_create( "oppo_leather_glove_mode_enable", 0666, prEntry_tp,&oppo_leather_glove_mode_enable_proc_fops);
	if(prEntry_temp == NULL) {
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
#endif

	prEntry_temp = proc_create( "tp_reg_operate", 0666, prEntry_tp,&tp_reg_operate_proc_fops);
	if(prEntry_temp == NULL) {
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}


#ifdef SUPPORT_GLOVES_MODE
	prEntry_temp = proc_create( "glove_mode_enable", 0666, prEntry_tp,&glove_mode_enable_proc_fops);
	if(prEntry_temp == NULL) {
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
#endif 

	//mignqiang.guo@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
	prEntry_temp = proc_create( "baseline_test", 0666, prEntry_tp, &tp_baseline_test_proc_fops);
	if(prEntry_temp == NULL){
		ret = -ENOMEM;
		TPD_DEBUG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
	TPD_DEBUG(KERN_INFO"create proc\touchpanel\baseline_test proc entry ok\n");
	//mignqiang.guo@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end	

	prEntry_temp = proc_create("sleep_mode_enable", 0666, prEntry_tp, &sleep_mode_enable_proc_fops);
	if(prEntry_temp == NULL)
	{	   
		ret = -ENOMEM;	   
		TPDTM_DMESG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}

	//wanghao @BSP modified for unite the interface to /proc
	prEntry_temp = proc_create("oppo_tp_delta_data", 0644, prEntry_tp, &oppo_tp_delta_data_proc_fops);
	if(prEntry_temp == NULL)
	{            
		ret = -ENOMEM;	   
		TPDTM_DMESG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
/*	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_delta_data))
	{            
		TPD_ERR("driver_create_file Failure\n");
	}*/

/*  cat /sys/bus/i2c/drivers/mtk-tpd/baseline_test , will Request firmware failed,
	use cat proc/touchpanel/baseline_test
	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_baseline_test))
	{           
		TPD_ERR("driver_create_file Failure\n");	
	}
*/	
	TPD_DEBUG("synaptics_ts_probe: going to create files--oppo_tp_fw_update\n");
	//if(ts == NULL || ts->client == NULL)
	//	BUG();	


	prEntry_temp = proc_create("oppo_tp_baseline_image_with_cbc", 0644, prEntry_tp, &oppo_tp_baseline_image_with_cbc_proc_fops);
	if(prEntry_temp == NULL)
	{            
		ret = -ENOMEM;	   
		TPDTM_DMESG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
	
	/*if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_baseline_image_with_cbc))
	{           
		TPD_ERR("driver_create_file Failure\n");	
	}*/	 

	prEntry_temp = proc_create("oppo_tp_baseline_image", 0644, prEntry_tp, &oppo_tp_baseline_image_proc_fops);
	if(prEntry_temp == NULL)
	{            
		ret = -ENOMEM;	   
		TPDTM_DMESG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}

/*	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_baseline_image))
	{           
		TPD_ERR("driver_create_file Failure\n");
	}*/

	prEntry_temp = proc_create("oppo_tp_debug", 0644, prEntry_tp, &oppo_tp_debug_proc_fops);
	if(prEntry_temp == NULL)
	{            
		ret = -ENOMEM;	   
		TPDTM_DMESG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}

	prEntry_temp = proc_create("oppo_tp_fw_update", 0644, prEntry_tp,&proc_firmware_update);
	if(prEntry_temp == NULL)
	{            
		ret = -ENOMEM;	   
		TPDTM_DMESG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}

	prEntry_temp = proc_create("oppo_tp_suspend", 0666, prEntry_tp, &oppo_tp_suspend);
	if(prEntry_temp == NULL)
	{            
		ret = -ENOMEM;	   
		TPDTM_DMESG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}

	prEntry_temp = proc_create("oppo_tp_limit_area", 0664, prEntry_tp, &proc_limit_area);
	if(prEntry_temp == NULL)
	{            
		ret = -ENOMEM;	   
		TPDTM_DMESG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}

	prEntry_temp = proc_create("oppo_tp_limit_enable", 0664, prEntry_tp, &proc_limit_enable);
	if(prEntry_temp == NULL)
	{            
		ret = -ENOMEM;	   
		TPDTM_DMESG(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
	
	return ret;

}




static int synaptics_ts_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef CONFIG_SYNAPTIC_RED	
	struct remotepanel_data *premote_data = NULL;
#endif
	struct synaptics_ts_data *ts = NULL;
	int ret = 0;
	uint32_t CURRENT_FIRMWARE_ID = 0;
	uint8_t buf[4];
	uint8_t bootloader_mode = 0;
	u8 read_dyte_data ;
	
	//rendong.shi add 2014/09/23 for tp use
	boot_mode = get_boot_mode();

/*	TPDTM_DMESG("boot_mode = %d ............\n",boot_mode);  
	if((boot_mode == META_BOOT)  || (boot_mode == FACTORY_BOOT))
	{
		//rendong.shi@BSP, 2015/06/26, modify for mtk poweron CTP in lk/platform.c for workaround what,but we should poweroff it in factory mode
		tpd_hw_pwron();
	    msleep(1);
		//Wanghao@BSP, 2015/11/18, modify for power in mt6755
		mt_set_gpio_mode(GPIO_CTP_OPEN_PIN, 0); //3v3
		mt_set_gpio_dir(GPIO_CTP_OPEN_PIN, 1);
		mt_set_gpio_out(GPIO_CTP_OPEN_PIN, 0);
		msleep(250); 
		return 0;
	}
	// Software reset mode will be treated as normal boot
	if(boot_mode==3) boot_mode = NORMAL_BOOT;
	if(boot_mode==7) boot_mode = NORMAL_BOOT;*/


	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	memset(ts,0,sizeof(*ts));	
	
	if (ts->input_dev == NULL)
		ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("synaptics_ts_probe: Failed to allocate input device\n");
		return -1;
	}
	
#if GTP_SUPPORT_I2C_DMA	
	gprDMABuf_va = (u8 *)dma_alloc_coherent(&ts->input_dev->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gprDMABuf_pa, GFP_KERNEL);
	if(!gprDMABuf_va){
		TPD_ERR("[Error] Allocate DMA I2C Buffer failed!\n");
	}
	memset(gprDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);

	gpwDMABuf_va = (u8 *)dma_alloc_coherent(&ts->input_dev->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpwDMABuf_pa, GFP_KERNEL);
	if(!gpwDMABuf_va){
		TPD_ERR("[Error] Allocate DMA I2C Buffer failed!\n");
	}
	memset(gpwDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
#endif
	TPD_ERR("synaptics_ts_probe: enter !!!!!!!!!!!!!!\n");
#ifdef SUPPORT_GESTURE
	atomic_set(&double_enable,0);
	is_gesture_enable = 0;
#endif

//mingqiang.guo for charge pulg in ,open tp Finger Amplitude Thre and Finger Dbounce ,avoid charge disturb
	atomic_set(&charge_plug_in_flag, 0);
	is_key_touch = 0;
	is_lcd_touch = 0;
	
#ifdef SUPPORT_LEATHER	
	atomic_set(&hall_close_flag, 0);
	atomic_set(&leather_enable_flag,0);
#endif 

#ifdef SUPPORT_GLOVES_MODE
	atomic_set(&glove_enable,0);
#endif
	
	/****************start*********************/

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TPD_ERR("%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	//ret = i2c_smbus_read_byte_data_s3508(client, 0x13);
	ret = i2c_dma_read(client, 0x13, 1, &read_dyte_data); 
	if( ret < 0 ){
		TPD_ERR("%s I2c communication error\n",__func__);
		//ret = i2c_smbus_read_byte_data_s3508(client, 0x13);
		ret = i2c_dma_read(client, 0x13, 1, &read_dyte_data); 
		if( ret < 0 ){
			tpd_power(0);
			TPD_ERR("synaptics is no exist!\n");
			TPD_ERR("%s I2c communication error\n",__func__);	
			return 0;
		}
	}

	TPDTM_DMESG("boot_mode = %d ............\n",boot_mode);  
	if((boot_mode == META_BOOT)  || (boot_mode == FACTORY_BOOT))
	{
		tpd_hw_pwron();
		ret = i2c_smbus_write_byte_data_s3508(client, 0x0e, 0x01); 	//incell can't simply poweroff, tp should be in sleep mode for FTM
		ret = i2c_smbus_read_byte_data_s3508(client, 0x0e);
		TPD_ERR("TP sleep mode is %d\n", ret);
		return 0;
	}
	// Software reset mode will be treated as normal boot
	if(boot_mode==3) boot_mode = NORMAL_BOOT;
	if(boot_mode==7) boot_mode = NORMAL_BOOT;
	
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->dev = &client->dev;
	ts->loading_fw = false;
	//init mutex
	mutex_init(&ts->mutex);
	ts_g = ts;
	speedup_resume_wq = create_singlethread_workqueue("speedup_resume_wq");
	if( !speedup_resume_wq ){
		ret = -ENOMEM;
		goto exit_speedup_createworkqueue_failed;	
	}   
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	INIT_WORK(&ts->speed_up_work,speedup_synaptics_resume);
	INIT_WORK(&ts->charge_write_tp_work,charge_write_tp_work_function);//mingqiang.guo 2015/8/15 add for charge driver write tp reg 
	INIT_WORK(&ts->close_edge_product, close_edge_protect_function_s3320);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	/*disable interrupt*/
	//get_id();
	//oppo_tp_dev = TP_TPK_SYNAPTICS;
	synaptics_read_register_map(ts);
	bootloader_mode = i2c_smbus_read_byte_data_s3508(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 
	TPD_DEBUG("synaptics:before fw update,bootloader_mode = 0x%x\n", bootloader_mode);

	i2c_dma_read(ts->client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
	TPD_ERR("CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);	
	TP_FW=CURRENT_FIRMWARE_ID;	
	oppo_tp_dev = TP_SAMSUNG_SYNAPTICS;

	sprintf(ts->fw_id,"0x%x",TP_FW);
	sprintf(ts->manu_name, "TP_SAMSUNG_SYNAPTICS");
	tp_info.version = ts->fw_id;
	tp_info.manufacture = ts->manu_name;

	memset(ts_g->fw_name,TP_FW_NAME_MAX_LEN,0);
	memset(ts_g->test_limit_name,TP_FW_NAME_MAX_LEN,0);

	//Wanghao@BSP, 2015/09/22, modify for firmware update in mt6755
	strcpy(ts->fw_name,"tp/16031_FW_S3320_JDI.img");	
	strcpy(ts->test_limit_name,"tp/16031_Limit_JDI.img");
	
	TPD_DEBUG("synatpitcs_fw: fw_name = %s \n",ts->fw_name);
	register_device_proc("tp", tp_info.version, tp_info.manufacture);
	/*disable interrupt*/
	ret = synaptics_enable_interrupt(ts, 0);
	if( ret < 0 ){
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}

	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq) {
		return -ENOMEM;
		goto exit_createworkqueue_failed;	
	}   

	ret = synaptics_fw_check(ts);
	if(ret <0 || TP_FW == 0 )
	{	
		force_update = 1;
		TPD_ERR("This FW need to be updated!\n");		
	}
	else
	{	
		force_update = 0;
		ret = synaptics_input_init(ts);
		if(ret < 0){
			TPD_ERR("synaptics_input_init failed!\n");			
		}
		ret = synaptics_tpd_button_init(ts);	
		if(ret < 0){
			TPD_ERR("synaptics_tpd_button_init failed!\n");			
		}
		
		ret = synaptics_device_irq_init(ts);
		if(ret < 0){
			TPD_ERR("synaptics_device_irq_init failed!\n");			
		}
	
		ts->use_irq = 1;
			
	}		

	
	ts->loading_fw = false;
	init_synaptics_proc(ts);
	//ret = synaptics_soft_reset(ts);

	TPDTM_DMESG("synaptics_ts_probe: ok end \n");
	tp_probe_ok = 1;
	left_x1 = (limit_area * 1000)/100;
	right_x1 = LCD_WIDTH - left_x1;
	left_x2 = 2 * left_x1;
	right_x2 = LCD_WIDTH - 2 * left_x1;
#ifdef CONFIG_SYNAPTIC_RED	
	premote_data = remote_alloc_panel_data();
	if(premote_data)
	{
		premote_data->client 		= client;
		premote_data->input_dev		= ts->input_dev;
		premote_data->kpd			= ts->kpd;
		premote_data->pmutex		= &ts->mutex;
		premote_data->irq_gpio 		= ts->irq_gpio;
		premote_data->irq			= client->irq;
		premote_data->enable_remote = &(ts->enable_remote);
		register_remote_device(premote_data);

	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB  + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	//enable_irq(touch_irq);
	TPD_ERR("%s ok line:%d tp_probe_ok = %d\n",__func__,__LINE__,tp_probe_ok);
	return 0;

exit_createworkqueue_failed:		
	destroy_workqueue(synaptics_wq);
	synaptics_wq = NULL;

exit_speedup_createworkqueue_failed:
	destroy_workqueue(speedup_resume_wq);
	speedup_resume_wq = NULL;
	
err_check_functionality_failed:
	tpd_power(0);
err_alloc_data_failed:
	kfree(ts);
	ts =NULL;
	ts_g =NULL;
	return ret;

}

#if 0
static int __devexit synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	printk("%s is called\n",__func__);

#ifdef CONFIG_SYNAPTIC_RED
	unregister_remote_device();
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#ifdef TPD_USE_EINT
	disable_irq(touch_irq);
#else
	hrtimer_cancel(&ts->timer);
#endif	
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);
	tpd_hw_pwroff();
	return 0;
}
#endif

static int synaptics_ts_suspend(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	TPD_ERR("%s: is called\n", __func__);	

	if(ts->input_dev == NULL)
	{
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		return -1;
	}

	mutex_lock(&ts->mutex);
	
	gesture_enable = 1;
	TPD_ERR("gesture_enable = %d\n", gesture_enable);
	is_key_touch = 0;
	is_lcd_touch = 0;

	/***********report Up key when suspend********/	
	synaptics_rmi4_free_fingers(ts);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(ts->input_dev);	
#endif
	input_sync(ts->input_dev);

#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif

#ifdef SUPPORT_GLOVES_MODE    
	if( 1 == atomic_read(&glove_enable) ){
		/* page select = 0x4 */
		ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x4); 
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data_s3508 failed for page select\n");
			ret =  0;
			goto OUT;	
		}
		//printk("glove mode disable\n");
		ret = i2c_smbus_write_byte_data_s3508(ts->client, F51_CUSTOM_CTRL00, 0x02 ); 	
		ret = synaptics_rmi4_set_page(ts->client, 0xff, 0x00); 
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data_s3508 failed for page select\n");
			ret =  0;
			goto OUT;	
		}
	}
#endif

//mingqiang.guo for charge pulg in ,open tp Finger Amplitude Thre and Finger Dbounce ,avoid charge disturb
    if(atomic_read(&charge_plug_in_flag))
	{
		TPD_ERR("%s, page 4 F51_CUSTOM_CTRL31=0x%x write %d \n",__func__,F51_CUSTOM_CTRL31,0);//0x0424
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, 0xff, 4);  
		if( ret < 0 ){
			TPD_ERR("%s i2c error\n",__func__);
			goto OUT;
		}

		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F51_CUSTOM_CTRL31&0x00ff, 0 );  
		if( ret < 0 ){
			TPD_ERR("%s i2c error\n",__func__);
		}

		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, 0xff, 0);  
		if( ret < 0 ){
			TPD_ERR("%s i2c error\n",__func__);
		}

	}
//end 

#ifdef SUPPORT_GESTURE	
	if( 1 == atomic_read(&double_enable) ){
	   // mignqiang.guo 2015/4/19 add for status bar pull down
		ret = i2c_smbus_write_byte_data_s3508(ts_g->client, F01_RMI_CTRL02, 3);// 3*10ms =30ms  
		if( ret < 0 ){
			TPD_ERR("%s i2c error\n",__func__);
		}
		//end
		synaptics_enable_interrupt_for_gesture(ts, 1);
		TPD_ERR("synaptics:double_tap end suspend\n");
		ret =  0;
		goto OUT;
	}
#endif
	
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret){
		TPD_DEBUG("%s: cannot disable interrupt\n", __func__);
		ret =  -1;
		goto OUT;
	}
	ret = i2c_smbus_write_byte_data_s3508(ts->client, F01_RMI_CTRL00, 0x01); 
	if( ret < 0 ){
		TPD_ERR("%s: control tm1400 to sleep failed\n", __func__);
		ret =  -1;
		goto OUT;
	}
	//if(0 == atomic_read(&double_enable) )
		is_suspend = 1;
	ret = 0;

OUT:
	TPD_DEBUG("synaptics_ts_suspend is end,is_suspend = %d\n",is_suspend);
	finish_suspend = 1;
	mutex_unlock(&ts->mutex);
	return ret;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	queue_work(speedup_resume_wq, &ts_g->speed_up_work);
	return 0;
}



static void speedup_synaptics_resume(struct work_struct *work)
{
	int ret;
	struct synaptics_ts_data *ts = container_of(work,struct synaptics_ts_data, speed_up_work);


	TPD_ERR("%s \n", __func__);
	/***********report Up key when resume********/	
	mutex_lock(&ts->mutex);

	finish_suspend = 0;
	is_suspend = 0;
	is_key_touch = 0;
	is_lcd_touch = 0;
	
	is_gesture_enable = 0;
	in_which_area = 0;
#ifdef TPD_USE_EINT
	disable_irq(touch_irq);    
#endif

	if(ts->input_dev == NULL)
	{
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		goto ERR_RESUME;
	}
	gesture_enable = 0;
	/***********report Up key when suspend********/	
	synaptics_rmi4_free_fingers(ts);

	all_finger_up = 0;
	/***Reset TP ******/ 
	ret = i2c_smbus_write_byte_data_s3508(ts->client, 0xff, 0x0); 
	if( ret < 0 ){
		TPD_ERR("%s: failed for page select try again later\n", __func__);
		msleep(20);
		ret = i2c_smbus_write_byte_data_s3508(ts->client, 0xff, 0x0); 
		if( ret < 0 ){
			TPD_ERR("%s: failed for page select try again later\n", __func__);
			goto ERR_I2C;
		}
	}
	ret = i2c_smbus_read_byte_data_s3508(ts->client, F01_RMI_DATA_BASE);
	free_irq(ts->client->irq, ts);
	//ret |= i2c_smbus_write_byte_data_s3508(ts->client, F01_RMI_CMD_BASE,0x01);	//soft reset tp
	ret |= synaptics_soft_reset(ts_g);
	if(ret < 0)
		goto ERR_I2C;
	msleep(50);

#ifdef SUPPORT_GESTURE
	if( 1 == atomic_read(&double_enable))
	{
		// mignqiang.guo 2015/4/19 add for status bar pull down
		ret = i2c_smbus_write_byte_data_s3508(ts->client, F01_RMI_CTRL02, 1); //1*10ms = 30ms  
		if( ret < 0 ){
			TPD_ERR("%s i2c error\n",__func__);
		}	
		//end
		synaptics_enable_interrupt_for_gesture(ts, 0); 
	}	
#endif

#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif	

#ifdef SUPPORT_LEATHER
	TPD_ERR("hall_close_flag = %d \n",hall_close_flag);
	hall_interrupt_enable_glove_mode(atomic_read(&hall_close_flag));
#endif 

//mingqiang.guo for charge pulg in ,open tp Finger Amplitude Thre and Finger Dbounce ,avoid charge disturb
    if(atomic_read(&charge_plug_in_flag))
	{
		TPD_ERR("%s, page 4 F51_CUSTOM_CTRL31=0x%x write charge_plug_status=%d \n",__func__,F51_CUSTOM_CTRL31,charge_plug_status);//0x0424
		ret = i2c_smbus_write_byte_data_s3508(ts->client, 0xff, 4);  
		if( ret < 0 ){
			TPD_ERR("%s i2c error\n",__func__);
			goto ERR_RESUME;
		}

		ret = i2c_smbus_write_byte_data_s3508(ts->client, F51_CUSTOM_CTRL31&0x00ff,charge_plug_status);  
		if( ret < 0 ){
			TPD_ERR("%s i2c error\n",__func__);
		}

		ret = i2c_smbus_write_byte_data_s3508(ts->client, 0xff, 0);  
		if( ret < 0 ){
			TPD_ERR("%s i2c error\n",__func__);
		}

	}
//end 

	/*****Normal Init TP********/
	ret = synaptics_init_panel(ts);
	if( ret < 0 ){
		TPD_ERR("%s: TP init failed\n", __func__);
		goto ERR_I2C;
	}
	ret = request_irq(ts_g->client->irq, (irq_handler_t)synaptics_ts_irq_handler, ts->irq_flags, TPD_DEVICE, ts_g);	

	msleep(150);
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret){
		TPD_ERR("%s:can't  enable interrupt!\n", __func__);
		goto ERR_I2C;
	}

ERR_RESUME:
#ifdef TPD_USE_EINT
	enable_irq(touch_irq);    
#endif
	mutex_unlock(&ts->mutex);
	return;
ERR_I2C:
	hard_reset_tp(ts);
	#ifdef TPD_USE_EINT
	enable_irq(touch_irq);    
	#endif
	mutex_unlock(&ts->mutex);
	//queue_work(speedup_resume_wq, &ts->speed_up_work);
	return;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;

	ts = container_of(h, struct synaptics_ts_data, early_suspend);

	if(ts_g->loading_fw) //when update tp fw , can not go to sleep
		return;
		
	synaptics_ts_suspend(ts->client);

}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);

	synaptics_ts_resume(ts->client);

}
#endif



static struct i2c_board_info __initdata synaptics_i2c_info ={ I2C_BOARD_INFO(TPD_DEVICE, (0x40>>1))};
/*----------------------------------------------------------------------------*/
#if 0
static int synaptics_ts_detect(struct i2c_client *client,
		int kind, struct i2c_board_info *info)
{         
	strcpy(info->type, TPD_DEVICE);                                                         

	TPD_DEBUG("synaptics-rmi-ts driver_detect !!\n ");

	return 0;                                                                                       
}
#endif

#if 0

#define DMA_CONTROL
#ifdef DMA_CONTROL
static int remote_rmit_put_page(unsigned int address)
{
	int ret;
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, 0xff, 0x0); 
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
}

static struct i2c_client *remote_rmi4_get_i2c_client(void)
{
	return ts_g->client;
}

static struct input_dev *remote_rmi4_get_input(void)
{
	return ts_g->input_dev;
}

static int remote_rmi4_i2c_enable(bool enable)
{
	if(enable)
		enable_irq(touch_irq);
	else
		disable_irq(touch_irq);	
	return 0;   
}

static int remote_rmit_set_page(unsigned int address)
{
	int ret = 0;
	unsigned int page = ((address >> 8) & 0xFF);
	ret = i2c_smbus_write_byte_data_s3508(ts_g->client, 0xff, page); 
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	return ret;
}

static int remote_rmi4_get_irq_gpio(void)
{
	return 75;
}
#endif

#endif

//mingqiang.guo@phone.bsp 2015-5-22 add for get lcd modle id 
//int lcd_id;
#if defined(S6E3FA3_FHD_DSI_CMD)
/* liping-m@PhoneSW.Multimedia, 2016/03/18  Add for read lcm id */
extern int lcd_oled_id;
#endif
extern int atoi(const char *);
static int get_lcm_id(char *lcm_id_char)
{
	sscanf(lcm_id_char, "%x", &lcd_id);
	TPD_ERR("TP get lcd id : 0x%x ",lcd_id);
	
#if defined(S6E3FA3_FHD_DSI_CMD)
	/* liping-m@PhoneSW.Multimedia, 2016/03/18	Add for read lcm id */
	lcd_oled_id =lcd_id;
#endif

	return 1;
}
__setup("lcm_id=", get_lcm_id);
//end

#ifdef VENDOR_EDIT
// Jingchun.Wang@Phone.Bsp.Driver, 2016/01/13  Add for solve the tp init too long 
static struct delayed_work	syp_s3508_work;
#define S3508_INIT_DELAY	round_jiffies_relative(msecs_to_jiffies(250))

static void syp_s3508_update_work(struct work_struct *work)
{
	//struct delayed_work *dwork = to_delayed_work(work);

	

	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		TPD_DEBUG("unable to add i2c driver.\n");
		return;
	}	
}
#endif /*CONFIG_VENDOR_EDIT*/

/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	printk("srd tpd_driver_init driver.\n");

	if( is_project(OPPO_16031) || is_project(OPPO_16034))
	{
		TPD_ERR("tpd_IC is 3320!\n");	
	}
	else
	{
		TPD_ERR("tpd_IC is 3508!Synaptics_s3320 exit\n");	
		return 0;
	}
	
	i2c_register_board_info(0, &synaptics_i2c_info, 1);

	tpd_hw_pwron_nosleep();

	INIT_DELAYED_WORK(&syp_s3508_work, syp_s3508_update_work);  
	schedule_delayed_work(&syp_s3508_work, S3508_INIT_DELAY);

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");

