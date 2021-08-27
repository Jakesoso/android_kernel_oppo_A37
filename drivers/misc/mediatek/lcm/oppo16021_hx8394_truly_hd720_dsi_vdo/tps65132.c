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
#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_i2c.h>
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif


//#define LCMPower_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL
#define LCMPower_I2C_BUSNUM  0
#define LCMPower_SLAVE_ADDR_WRITE	0x7C  //For LCM Power(8 bit)


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

#define I2C_ID_NAME "TPS65132"
static struct i2c_board_info __initdata tps65132_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, LCMPower_SLAVE_ADDR_WRITE>>1)};
static struct i2c_client *I2C_TPS65132 = NULL;
static const struct i2c_device_id DeviceId[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};


static int LCMPower_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int LCMPower_remove(struct i2c_client *client);


static struct i2c_driver LCMPower_I2C_driver = {
	.id_table	= DeviceId,
	.probe		= LCMPower_probe,
	.remove		= LCMPower_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "LCM Power",
	},
 
};

static int LCMPower_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	I2C_TPS65132  = client;		
	return 0;      
}
static int LCMPower_remove(struct i2c_client *client)
{  	
	I2C_TPS65132 = NULL;
	i2c_unregister_device(client);
	return 0;
}

// int LCMPower_Write_Byte(unsigned char addr, unsigned char value)
int LCMPower_Write_Byte(kal_uint8 addr, kal_uint8 value)	
{	
	int ret = 0;
	struct i2c_client *client = I2C_TPS65132;
	kal_uint8 write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		printk("Tps65132 Write I2C fail !!\n");	
	return ret ;
}
EXPORT_SYMBOL(LCMPower_Write_Byte);


static int __init LCMPower_I2C_Init(void)
{
   i2c_register_board_info(LCMPower_I2C_BUSNUM, &tps65132_board_info, 1);
   i2c_add_driver(&LCMPower_I2C_driver);
   return 0;
}

static void __exit LCMPower_I2C_Exit(void)
{
  i2c_del_driver(&LCMPower_I2C_driver);  
}

module_init(LCMPower_I2C_Init);
module_exit(LCMPower_I2C_Exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 

#else

static struct mt_i2c_t I2C_LM3630A;

int LCMPower_Write_Byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    I2C_LM3630A.id = LCMPower_I2C_BUSNUM; //I2C1
    I2C_LM3630A.addr = (LCMPower_SLAVE_ADDR_WRITE >>1);//(SlaveID >> 1);
    I2C_LM3630A.mode = ST_MODE;
    I2C_LM3630A.speed = 100;
    len = 2;

    ret_code = i2c_write(&I2C_LM3630A, write_data, len);

    dprintf(0, "%s ret_code is %d\n", __func__, ret_code);

    return ret_code;
}

#endif
