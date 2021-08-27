/*****************************************************************
* Copyright (c) 2004-2014 OPPO Mobile communication Corp.ltd.,
* VENDOR_EDIT
* Description: Source file for LCM driver IC.
* Version   : 1.0
* Date      : 2015-12-23
* Author    : liping-m@PhoneSW.Multimedia
*----------------------Revision History--------------------------
* None.
*****************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
//#include <linux/oppo_devices_list.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/upmu_common.h>
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h> 
	#include <platform/mt_pmic.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_pm_ldo.h>
    #include <mach/mt_gpio.h>
#endif
#include <cust_gpio_usage.h>
#ifndef FPGA_EARLY_PORTING
#include <cust_i2c.h>
#endif

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG, add for kpoc
volatile int OPPO_LED_ON = 1;
#endif/*VENDOR_EDIT*/

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif


//#define BL_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL
#define BL_I2C_BUSNUM  0
#define LM3630A_SLAVE_ADDR_WRITE 	0x70  //For BL (8 bit)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>
#define LCD_BL_EN (GPIO52 | 0x80000000)
#define I2C_ID_NAME "LM3630A"
static struct i2c_board_info __initdata BL_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, LM3630A_SLAVE_ADDR_WRITE>>1)};
static struct i2c_client *I2C_LM3630A = NULL;
static const struct i2c_device_id DeviceId[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};


static int BL_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int BL_remove(struct i2c_client *client);

/* liping-m@PhoneSW.Multimedia, 2016/01/22	ADD for CTA LCD video mode */
unsigned int bl3630a_esd_recovery_backlight_level = 2;

//extern OPPO_BKL_DEV oppo_bkl_dev;

static struct i2c_driver BL_I2C_driver = {
	.id_table	= DeviceId,
	.probe		= BL_probe,
	.remove		= BL_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "Black light",
	},
 
};

static int BL_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	I2C_LM3630A  = client;
	#ifndef BUILD_LK
	//hwPowerOn(MT6325_POWER_LDO_VGP3, VOL_1800, "LCD_BACKLIGHT");
	printk("srd BL_probe\n");
	#endif
    //oppo_bkl_dev = BKL_LM3630;
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG, add for kpoc
	OPPO_LED_ON = 1;
#endif/*VENDOR_EDIT*/	
    
	return 0;      
}
static int BL_remove(struct i2c_client *client)
{  	
	I2C_LM3630A = NULL;
	i2c_unregister_device(client);
	return 0;
}

//int LM3630A_Write_Byte(unsigned char addr, unsigned char value)
int LM3630A_Write_Byte(kal_uint8 addr,  kal_uint8 value)
{	
	int ret = 0;
	struct i2c_client *client = I2C_LM3630A;
	kal_uint8 write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		printk("Bl lm3630a Write I2C fail !!\n");	
	return ret ;
}
EXPORT_SYMBOL(LM3630A_Write_Byte);

int LM3630A_Read_Byte(kal_uint8 addr, char  *value, int length)
{
         int ret = 0;
         struct i2c_client *client = I2C_LM3630A;
         struct i2c_msg msg[2] = {
                   {
                            .addr = client->addr,
                            .buf = &addr,
                            .len = 1,
                   },
                   {
                            .addr = client->addr,
                            .flags = I2C_M_RD,
                            .buf = value,
                            .len = length,
                   },
         };
 
     ret = i2c_transfer(client->adapter, msg, 2);
         if(ret != 2) {
                   printk("Bl lm3630a Read I2C fail !!ret = %d\n",ret);
                   ret = -EIO;
		}else{
         	ret = 0;
		}
    return ret ;
}


EXPORT_SYMBOL(LM3630A_Read_Byte);

static int backlight_buf[] = {
		81,81,81,96,106,114,121,126,131,135,139,142,146,148,151, \
		154,156,158,160,162,164,166,167,169,170,172,173,175,176, \
		177,178,180,181,182,183,184,185,186,187,188,189,190,190, \
		191,192,193,194,194,195,196,197,197,198,199,199,200,201, \
		201,202,203,203,204,204,205,206,206,207,207,208,208,209, \
		209,210,210,211,211,212,212,213,213,213,214,214,215,215, \
		216,216,217,217,217,218,218,219,219,219,220,220,220,221, \
		221,221,222,222,223,223,223,224,224,224,225,225,225,226, \
		226,226,226,227,227,227,228,228,228,229,229,229,229,230, \
		230,230,231,231,231,231,232,232,232,232,233,233,233,234, \
		234,234,234,235,235,235,235,236,236,236,236,236,237,237, \
		237,237,238,238,238,238,239,239,239,239,239,240,240,240, \
		240,240,241,241,241,241,242,242,242,242,242,243,243,243, \
		243,243,244,244,244,244,244,244,245,245,245,245,245,246, \
		246,246,246,246,246,247,247,247,247,247,248,248,248,248, \
		248,248,249,249,249,249,249,249,250,250,250,250,250,250, \
		251,251,251,251,251,251,251,252,252,252,252,252,252,253, \
		253,253,253,253,253,253,254,254,254,254,254,254,254,255, \
		255,255,255
};

int lm3630a_setbacklight(unsigned int level)
{
   // printk("%s level is %d\n", __func__, level);

	mt_set_gpio_mode(LCD_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BL_EN, GPIO_OUT_ONE);
	
    if (level == 0) {
        LM3630A_Write_Byte(0x03,level);
	    LM3630A_Write_Byte(0x04,level);
        LM3630A_Write_Byte(0x00,0x9D);
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG, add for kpoc
		OPPO_LED_ON = 0;
#endif/*VENDOR_EDIT*/
    } else {
        LM3630A_Write_Byte(0x00,0x05);
    	LM3630A_Write_Byte(0x03,backlight_buf[level]);//Bank Level
    	LM3630A_Write_Byte(0x04,backlight_buf[level]);//Bank Level
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG, add for kpoc
		OPPO_LED_ON = 1;
#endif/*VENDOR_EDIT*/
		bl3630a_esd_recovery_backlight_level =level;
    }

    return level;
}

static int __init BL_I2C_Init(void)
{
   i2c_register_board_info(BL_I2C_BUSNUM, &BL_board_info, 1);
   i2c_add_driver(&BL_I2C_driver);
   return 0;
}

static void __exit BL_I2C_Exit(void)
{
  i2c_del_driver(&BL_I2C_driver);  
}

module_init(BL_I2C_Init);
module_exit(BL_I2C_Exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 

#else

static struct mt_i2c_t I2C_Device;

int LM3630A_Write_Byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    I2C_Device.id = BL_I2C_BUSNUM; //I2C1
    I2C_Device.addr = (LM3630A_SLAVE_ADDR_WRITE >>1);//(SlaveID >> 1);
    I2C_Device.mode = ST_MODE;
    I2C_Device.speed = 100;
    len = 2;

    ret_code = i2c_write(&I2C_Device, write_data, len);

    return ret_code;
}

int lm3630a_setbacklight(unsigned int level)
{
    //dprintf(0, "%s level is %d\n", __func__, level);

    if (level == 0) {
        LM3630A_Write_Byte(0x03,level);
	    LM3630A_Write_Byte(0x04,level);
        LM3630A_Write_Byte(0x00,0x9D);
    } else {
        LM3630A_Write_Byte(0x00,0x1D);
    	LM3630A_Write_Byte(0x03,level);//Bank Level
    	LM3630A_Write_Byte(0x04,level);//Bank Level
    }

    return level;
}

#endif
