/* drivers/hwmon/mt6516/amit/apds9110.c - APDS9110 ALS/PS driver
 * 
 * Author: Lee Kai Koon <kai-koon.lee@avagotech.com>
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

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
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

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include <alsps.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE
#define APDS_DEBUG 0

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include "apds9110.h"
#include <linux/sched.h>
#include <linux/oppo_devices_list.h>
#include <soc/oppo/oppo_project.h>
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define APDS9110_DEV_NAME     "APDS9110"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_DEBUG APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args) 

#define I2C_FLAG_WRITE 0
#define I2C_FLAG_READ	1

#define HIGHLIGHT_MODE_ALS_THRESHOLD 8000

/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
long apds9110_read_ps(struct i2c_client *client, u16 *data);
int apds9110_read_als(struct i2c_client *client, u32 *data, bool calied);
int apds9110_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag);
int apds9110_calibration_ps(struct i2c_client *client);
static int apds9110_init_client(struct i2c_client *client);
static void apds9110_set_ps_threshold(struct i2c_client *client, int low_threshold, int high_threshold);
/*----------------------------------------------------------------------------*/
static struct i2c_client *apds9110_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id apds9110_i2c_id[] = {{"APDS9921",0},{}};
static struct i2c_board_info __initdata i2c_apds9110={ I2C_BOARD_INFO("APDS9921", 0x53)};
/*----------------------------------------------------------------------------*/
static int apds9110_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int apds9110_i2c_remove(struct i2c_client *client);
static int apds9110_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int apds9110_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int apds9110_i2c_resume(struct i2c_client *client);
static int apds9110_remove(void);
static int apds9110_local_init(void);

static int enable_count=0;
static int first_als_data = 0;
extern int als_only;

static int apds9110_set_psensor_threshold(struct i2c_client *client);

#ifdef VENDOR_EDIT 	
//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
static DECLARE_WAIT_QUEUE_HEAD(enable_ps);
#endif

static int apds9110_init_flag =-1; // 0<==>OK -1 <==> fail
static struct alsps_init_info apds9110_init_info = {
	.name   = "APDS9110",
	.init   = apds9110_local_init,
	.uninit = apds9110_remove,
};

static DEFINE_MUTEX(apds9110_mutex);

static struct apds9110_priv *g_apds9110_ptr = NULL;

struct PS_CALI_DATA_STRUCT
{
	int close;
	int far_away;
	int valid;
};

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static unsigned long long int_top_time = 0;
struct ps_adjust_para *apds9110_ps_adjust_para_real = NULL;

/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS = 1,
	CMC_BIT_PS  = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct apds9110_i2c_addr { /*define a series of i2c slave address*/
	u8 write_addr;
	u8 ps_thd; /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct apds9110_priv {
	struct alsps_hw *hw;
	struct i2c_client *client;
	struct work_struct eint_work;

	/*i2c address group*/
	struct apds9110_i2c_addr addr;

	/*misc*/
	u16 als_modulus;
	atomic_t i2c_retry;
	atomic_t als_suspend;
	atomic_t als_debounce; /*debounce time after enabling als*/
	atomic_t als_deb_on;   /*indicates if the debounce is on*/
	atomic_t als_deb_end;  /*the jiffies representing the end of debounce*/
	atomic_t ps_mask;      /*mask ps: always return far away*/
	atomic_t ps_debounce;  /*debounce time after enabling ps*/
	atomic_t ps_deb_on;    /*indicates if the debounce is on*/
	atomic_t ps_deb_end;   /*the jiffies representing the end of debounce*/
	atomic_t ps_suspend;

	/*data*/
	u32 als;
	u16 ps;
	u8 _align;
	u16 als_level_num;
	u16 als_value_num;
	u32 als_level[C_CUST_ALS_LEVEL-1];
	u32 als_value[C_CUST_ALS_LEVEL];
	int ps_cali;
	int cali;

	atomic_t als_cmd_val;    /*the cmd value can't be read, stored in ram*/
	atomic_t ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
	atomic_t ps_thd_val_high;/*the cmd value can't be read, stored in ram*/
	atomic_t ps_thd_val_low; /*the cmd value can't be read, stored in ram*/
	ulong enable;            /*enable mask*/
	ulong pending_intr;      /*pending interrupt*/

	int first_int_enable;
	/* kk 13-May-2015 */
	int ps_state;
	int offset;

	int gain;

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver apds9110_i2c_driver = {	
	.probe      = apds9110_i2c_probe,
	.remove     = apds9110_i2c_remove,
	.detect     = apds9110_i2c_detect,
	.suspend    = apds9110_i2c_suspend,
	.resume     = apds9110_i2c_resume,
	.id_table   = apds9110_i2c_id,
	.driver = {
		.name     = APDS9110_DEV_NAME,
	},
};

static struct apds9110_priv *apds9110_obj = NULL;
//static struct platform_driver apds9110_alsps_driver;
static int apds9110_get_ps_value(struct apds9110_priv *obj, u16 ps);
static int apds9110_get_first_ps_value(struct apds9110_priv *obj, u16 ps);


/*----------------------------------------------------------------------------*/
#if APDS_DEBUG
static int dump_count = 0;
static int dump_count_max = 20;
static int  apds9110_dump_reg(void)
{
	int i;
	int res = 0;
	u8 databuf[3];
	u8 _buff[200]={0};

	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}

	for (i = 0x00; i <= 0x27; i += 0x01){
		if(i%0xF == 0){
			strcat(_buff,"\n[psensord]####");
		}
		databuf[0]=i;
		res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x101, I2C_FLAG_READ);

		if(res < 0)
		{
			return 0;
		}

		sprintf(_buff,"%s %02x",_buff,databuf[0]);
	}

	APS_ERR("####apds9110_dump_reg start:####\n%s\n[psensord]####apds9110_dump_reg end:####\n",_buff);
	return 1;
}

static int apds9110_esd_debug(void){
	if(!apds9110_obj)
	{
		APS_ERR("%s:apds9110_obj is null!!\n",__func__);
		return 0;
	}

	apds9110_read_ps(apds9110_obj->client, &apds9110_obj->ps);
	apds9110_read_als(apds9110_obj->client, &apds9110_obj->als,true);
	APS_ERR("####%s:als:%d ps:%d\n",__func__,apds9110_obj->als,apds9110_obj->ps);
	apds9110_dump_reg();
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t apds9110_show_ps(struct device_driver *ddri, char *buf)
{
	int res;
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}
	
	if((res = apds9110_read_ps(apds9110_obj->client, &apds9110_obj->ps)))
	{
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		#ifndef VENDOR_EDIT //ziqing.guo@BasicDrv.Sensor, 2015/02/12, Modify  for factory mode 
			return scnprintf(buf, PAGE_SIZE, "0x%04X\n", apds9110_obj->ps);
		#else/* VENDOR_EDIT */
			return scnprintf(buf, PAGE_SIZE, "%d\n", apds9110_obj->ps);
		#endif/* VENDOR_EDIT */		
	}
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t apds9110_show_reg(struct device_driver *ddri, char *buf)
{
	int i;
	int res = 0;
	u8 databuf[3];
	static u8 reg_buff[PAGE_SIZE/4 - 1]={0};
	/*databuf[0]=0x9e;
	res = TMD2772_i2c_master_operate(TMD2772_obj->client, databuf, 0x101, I2C_FLAG_READ);
       return scnprintf(buf, PAGE_SIZE, "0x%04X\n", databuf[0]); */
	
	//APS_LOG("TMD2772_read_register reg:%x val:%x \n",i, databuf[0]);

	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}
	sprintf(reg_buff," ");
	for (i = 0x00; i <= 0x27; i += 0x01){
		databuf[0]=i;
		res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x101, I2C_FLAG_READ);

		if(res < 0)
		{
			//goto EXIT_INTR_ERR;
			return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
		}

		sprintf(reg_buff,"%s 0x%02x:%02x",reg_buff,i,databuf[0]);
		if((i+1)%0x8 == 0)
			strcat(reg_buff,"\n ");
	}

	return sprintf(buf,"%s\n",reg_buff);
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static ssize_t apds9110_store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	u8 databuf[2];
	u32 reg,value;
	int res;
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}

	if(2 == sscanf(buf, "%x %x", &reg,&value))
	{
		databuf[0] = reg;
		databuf[1] = value;
		res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			APS_ERR("write reg %02x err\n", databuf[0]);
		}
	}else{
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t apds9110_show_cali(struct device_driver *ddri, char *buf)
{
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", apds9110_obj->cali);
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t apds9110_store_cali(struct device_driver *ddri, const char *buf, size_t count)
{
	int calib;
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d", &calib))
	{ 
		if (calib==1)
			{
				apds9110_obj->cali=apds9110_calibration_ps(apds9110_obj->client);
			}				
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static ssize_t apds9110_show_offset(struct device_driver *ddri, char *buf)
{
		
	int res = 0;

	u8 databuf_L[2];
	u8 databuf_H[2];
	static u8 _buff[PAGE_SIZE/4 - 1]={0};

	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}

	databuf_L[0]= APDS9110_DD_PRX_CAN_0_ADDR;
	res = apds9110_i2c_master_operate(apds9110_obj->client, databuf_L, 0x101, I2C_FLAG_READ);
	
	if(res < 0)
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}	

	databuf_H[0]= APDS9110_DD_PRX_CAN_1_ADDR;
	res = apds9110_i2c_master_operate(apds9110_obj->client, databuf_H, 0x101, I2C_FLAG_READ);
	
	if(res < 0)
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}

	sprintf(_buff,"%d,%d",databuf_L[0],databuf_H[0]);

		return sprintf(buf,"%s\n",_buff);
}
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static ssize_t apds9110_store_offset(struct device_driver *ddri, const char *buf, size_t count)
{
	int res,offsetL,offsetH;
	u8 databuf[2];
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}
	
	if(2 == sscanf(buf, "%d,%d", &offsetL,&offsetH))
	{ 
       	databuf[0] = APDS9110_DD_PRX_CAN_0_ADDR;
		databuf[1] = offsetL;
		res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x2, I2C_FLAG_WRITE);//xtalk cancellation
		if (res <= 0) {
		 goto EXIT_ERR;
		}

		databuf[0] = APDS9110_DD_PRX_CAN_1_ADDR;
		databuf[1] = offsetH;
		res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x2, I2C_FLAG_WRITE);//xtalk cancellation
		if (res <= 0) {
		 goto EXIT_ERR;
		}			
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;
EXIT_ERR:
	return res;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
static ssize_t apds9110_show_gain(struct device_driver *ddri, char *buf)
{
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", apds9110_obj->gain);
}
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
static ssize_t apds9110_store_gain(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}
	
	if(1 != sscanf(buf, "%d", &apds9110_obj->gain))
	{ 				
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;
}
----------------------------------------------------------------------------*/
static ssize_t apds9110_show_threshold(struct device_driver *ddri, char *buf)
{
	int threshold_hi = 0;
	int threshold_lo = 0;
	int val_ps_cali = 0;
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}
	threshold_lo = atomic_read(&apds9110_obj->ps_thd_val_low);
	threshold_hi = atomic_read(&apds9110_obj->ps_thd_val_high);
	val_ps_cali = apds9110_obj->ps_cali;
	APS_ERR("%s low: %d, high: %d, cali: %d\n", __func__, threshold_lo,threshold_hi,val_ps_cali);
	return scnprintf(buf, PAGE_SIZE, "low: %d, high: %d, cali: %d\n",threshold_lo,threshold_hi,val_ps_cali);
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t apds9110_store_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	int threshold_hi = 0;
	int threshold_lo = 0;
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}

	if(1 != sscanf(buf, "%d,%d", &threshold_lo,&threshold_hi))
	{
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}

	APS_ERR("%s set threshold low: %d, high: %d, cali: %d\n", __func__, threshold_lo,threshold_hi,apds9110_obj->ps_cali);
	atomic_set(&apds9110_obj->ps_thd_val_low,  (threshold_lo+apds9110_obj->ps_cali));
	atomic_set(&apds9110_obj->ps_thd_val_high,  (threshold_hi+apds9110_obj->ps_cali));//need to confirm
	apds9110_set_psensor_threshold(apds9110_obj->client);

	return count;
}
#if 0
/*----------------------------------------------------------------------------*/
static ssize_t apds9110_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	int als;

	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}

	if((res = apds9110_read_als(apds9110_obj->client, &als,false)))
	{
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		#ifndef VENDOR_EDIT //ziqing.guo@BasicDrv.Sensor, 2015/02/12, Modify  for factory mode 
			return scnprintf(buf, PAGE_SIZE, "0x%08X\n", als);
		#else/* VENDOR_EDIT */
			return scnprintf(buf, PAGE_SIZE, "%d\n", als);
		#endif/* VENDOR_EDIT */		
	}
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t apds9110_show_ps_adjust_para(struct device_driver *ddri, char *buf)
{
	static char tmpbuf[500] = {0};
	int res;
	APS_FUN();
	res = prox_show_adjust_para(apds9110_ps_adjust_para_real,tmpbuf);
	if(res == -2){
		APS_ERR("apds9110_ps_adjust_para_real is null!!\n");
	}else{
		ALSPS_ERR("tmpbuf:%s",tmpbuf);
		return scnprintf(buf, PAGE_SIZE, "%s", tmpbuf);
	}
	return 1;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t apds9110_store_ps_adjust_para(struct device_driver *ddri, const char *buf, size_t count)
{
	int enable,res;
	int index,value;
	APS_FUN();
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}
	enable = test_bit(CMC_BIT_PS, &apds9110_obj->enable) ? (1) : (0);

	if(2 == sscanf(buf, "%d %d", &index,&value))
	{
		if(enable == 0){
			res = prox_set_adjust_para(apds9110_ps_adjust_para_real,index,value);
			APS_ERR("store_ps_adjust_para:indx:%d val:%d ret:%d\n",index,value,res);
		}else{
			APS_ERR("ps is enabled,can not store_ps_adjust_para\n");
		}
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t apds9110_show_tuning(struct device_driver *ddri, char *buf)
{
	u8 databuf[2];
	int res = 0;
	int freq,ps_current,pluse;
	if(!apds9110_obj){
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}
	databuf[0] = APDS9110_DD_PRX_LED_ADDR;
	res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x101, I2C_FLAG_READ);
	if (res < 0) {
		goto EXIT_ERR;
	}
	freq = databuf[0]>>4;
	ps_current = databuf[0]&0x07;
	databuf[0] = APDS9110_DD_PRX_PULSES_ADDR;
	res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x101, I2C_FLAG_READ);
	if (res < 0) {
		goto EXIT_ERR;
	}
	pluse = databuf[0];
	return sprintf(buf,"\nrun \"echo idx,value > tuning\" to change the value\n\n\
[0]freq:%d 3=60kHZ 4=70kHZ 5=80kHZ 6=90kHZ 7=100kHZ\n\
[1]current:%d 3=25mA 4=50mA 5=75mA 6=100mA 7=125mA\n\
[2]pluse:%d\n\n\
",freq,ps_current,pluse);
EXIT_ERR:
	sprintf(buf,"read err\n");
	return 1;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t apds9110_store_tuning(struct device_driver *ddri, const char *buf, size_t count)
{
	u8 databuf[2];
	int res = 0;
	int idx,value;

	if(!apds9110_obj){
		APS_ERR("apds9110_obj is null!!\n");
		return 0;
	}
	
	if(sscanf(buf,"%d,%d",&idx,&value)==2){
		APS_ERR("idx:%d value:%d\n",idx,value);
		switch(idx){
			case 0:
				databuf[0] = APDS9110_DD_PRX_LED_ADDR;
				res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x101, I2C_FLAG_READ);
				if (res < 0) {
					goto EXIT_ERR;
				}
				databuf[0] &= 0x0F;
				databuf[0] |= (u8)(value<<4) & 0x70;
				databuf[1] = databuf[0];
				databuf[0] = APDS9110_DD_PRX_LED_ADDR;
				break;
			case 1:
				databuf[0] = APDS9110_DD_PRX_LED_ADDR;
				res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x101, I2C_FLAG_READ);
				if (res < 0) {
					goto EXIT_ERR;
				}
				databuf[0] &= 0xF8;
				databuf[0] |= (u8)value & 0x07;
				databuf[1] = databuf[0];
				databuf[0] = APDS9110_DD_PRX_LED_ADDR;
				break;
			case 2:
				databuf[1] = (u8)value;
				databuf[0] = APDS9110_DD_PRX_PULSES_ADDR;
				break;
			default:
				goto EXIT_ERR;
		}
		res = apds9110_i2c_master_operate(apds9110_obj->client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res < 0) {
			goto EXIT_ERR;
		}
	}
	return count;
EXIT_ERR:
	APS_ERR("i2c err\n");
	return count;
}


/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, apds9110_show_ps,    NULL);
static DRIVER_ATTR(reg,      S_IWUSR | S_IRUGO, apds9110_show_reg,    apds9110_store_reg);
static DRIVER_ATTR(cali,      S_IWUSR | S_IRUGO, apds9110_show_cali,  apds9110_store_cali );
static DRIVER_ATTR(offset,      S_IWUSR | S_IRUGO, apds9110_show_offset,  apds9110_store_offset );
//static DRIVER_ATTR(gain_als,      S_IWUSR | S_IRUGO, apds9110_show_gain,  apds9110_store_gain);
static DRIVER_ATTR(threshold,      S_IWUSR | S_IRUGO, apds9110_show_threshold,  apds9110_store_threshold);
//static DRIVER_ATTR(als,      S_IWUSR | S_IRUGO, apds9110_show_als,    NULL);
static DRIVER_ATTR(ps_adjust_para,      S_IWUSR | S_IRUGO, apds9110_show_ps_adjust_para,  apds9110_store_ps_adjust_para );
static DRIVER_ATTR(tuning,      S_IWUSR | S_IRUGO, apds9110_show_tuning,  apds9110_store_tuning );

static struct driver_attribute *apds9110_attr_list[] = {
    &driver_attr_ps,    
    &driver_attr_reg,
    &driver_attr_cali,
    &driver_attr_offset,
	&driver_attr_threshold,
    &driver_attr_ps_adjust_para,
	&driver_attr_tuning,
};

static int apds9110_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(apds9110_attr_list)/sizeof(apds9110_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, apds9110_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", apds9110_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static int apds9110_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(apds9110_attr_list)/sizeof(apds9110_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, apds9110_attr_list[idx]);
	}
	
	return err;
}


/*------------------------i2c function for 89-------------------------------------*/
int apds9110_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;
	
	mutex_lock(&apds9110_mutex);

	switch (i2c_flag) {
		case I2C_FLAG_WRITE:
			client->addr &=I2C_MASK_FLAG;
			res = i2c_master_send(client, buf, count);
			client->addr &=I2C_MASK_FLAG;
			break;

		case I2C_FLAG_READ:
			client->addr &=I2C_MASK_FLAG;
			client->addr |=I2C_WR_FLAG;
			client->addr |=I2C_RS_FLAG;
			res = i2c_master_send(client, buf, count);
			client->addr &=I2C_MASK_FLAG;
			break;

		default:
			APS_LOG("apds9110_i2c_master_operate i2c_flag command not support!\n");
			break;
	}

	if (res <= 0) {
		goto EXIT_ERR;
	}

	mutex_unlock(&apds9110_mutex);
	return res;

EXIT_ERR:
	mutex_unlock(&apds9110_mutex);
	APS_ERR("apds9110_i2c_transfer fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
int apds9110_get_addr(struct alsps_hw *hw, struct apds9110_i2c_addr *addr)
{
	if(!hw || !addr) {
		return -EFAULT;
	}

	addr->write_addr= hw->i2c_addr[0];
	return 0;
}

/*----------------------------------------------------------------------------*/
static void apds9110_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if (hw->power_id != POWER_NONE_MACRO) {
		if (power_on == on) {
			APS_LOG("ignore power control: %d\n", on);
		} else if (on) {
			if(!hwPowerOn(hw->power_id, hw->power_vol, "APDS9110")) {
				APS_ERR("power on fails!!\n");
			}
		} else {
			if(!hwPowerDown(hw->power_id, "APDS9110")) {
				APS_ERR("power off fail!!\n");
			}
		}
	}

	power_on = on;
}


/*----------------------------------------------------------------------------*/
static long apds9110_enable_als(struct i2c_client *client, int enable)
{
	struct apds9110_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];	  
	long res = 0;

	databuf[0]= APDS9110_DD_MAIN_CTRL_ADDR;
	res = apds9110_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	
	APS_LOG("apds9110_CMM_ENABLE als value = %x\n", databuf[0]);
	
	if (enable) {
		set_bit(CMC_BIT_ALS, &obj->enable);
		enable_count++;
		APS_LOG("%s enable als[%d]\n",__func__,enable_count);

		APS_LOG("first_als_data = 1\n");

		if (test_bit(CMC_BIT_PS, &obj->enable)==0)
		{
			first_als_data = 1;
			
			databuf[1] = databuf[0]|APDS9110_DD_ALS_EN;
			databuf[0] = APDS9110_DD_MAIN_CTRL_ADDR;
			APS_LOG("apds9110_DD_MAIN_CTRL_ADDR enable als value = %x\n", databuf[1]);
			res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if (res <= 0) {
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		}
	} else {
		enable_count--;
		APS_LOG("%s disable als[%d]\n",__func__,enable_count);
		if(enable_count<=0){
			//mdelay(5);
			if (test_bit(CMC_BIT_PS, &obj->enable)==0)
			{

				databuf[0] = APDS9110_DD_ALS_MEAS_RATE_ADDR;
				databuf[1] = APDS9110_DD_ALS_MEAS_RES_16_BIT|APDS9110_DD_ALS_MEAS_RATE_25_MS;
				res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
				if (res <= 0) {
					APS_LOG("set APDS9110_DD_ALS_MEAS_RATE_ADDR i2c fail\n");
					goto EXIT_ERR;
				}

				databuf[1] = databuf[0]&~(APDS9110_DD_PRX_EN|APDS9110_DD_ALS_EN);				
				databuf[0] = APDS9110_DD_MAIN_CTRL_ADDR;
				APS_LOG("apds9110_DD_MAIN_CTRL_ADDR disable als value = %x\n", databuf[1]);
				res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
				if (res <= 0) {
					goto EXIT_ERR;
				}
			}
			clear_bit(CMC_BIT_ALS, &obj->enable);
			enable_count=0;
		}
	}

	return 0;

EXIT_ERR:
	APS_ERR("apds9110_enable_als fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static long apds9110_enable_ps(struct i2c_client *client, int enable)
{
	struct apds9110_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	long res = 0;
	//int err=0;
	u8 main_ctrl=0;
	// kk 13-May-2015
	int ps_value;

	databuf[0] = APDS9110_DD_MAIN_CTRL_ADDR;
	res = apds9110_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	//APS_LOG("APDS9110_DD_MAIN_CTRL_ADDR ps value = %x\n", databuf[0]);

	if (enable) {

		set_bit(CMC_BIT_PS, &obj->enable);
		main_ctrl = databuf[0]|APDS9110_DD_PRX_EN;//|APDS9110_DD_ALS_EN;

		if (0 == obj->hw->polling_mode_ps) {
			obj->first_int_enable = 0;
		}

		// disable interrupt
		databuf[0] = APDS9110_DD_INT_CFG_ADDR;
		databuf[1] = 0x10;

		res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			goto EXIT_ERR;
		}

		databuf[1] = main_ctrl;
		databuf[0] = APDS9110_DD_MAIN_CTRL_ADDR;
		//APS_LOG("APDS9110_DD_MAIN_CTRL_ADDR enable ps value = %x\n", databuf[1]);
		res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			goto EXIT_ERR;
		}

		//apds9110_set_ps_threshold(obj->client,150, 160);

		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));

		msleep(20);
		apds9110_read_ps(obj->client, &obj->ps);
		ps_value = apds9110_get_first_ps_value(obj, obj->ps);
		//APS_LOG("apds9110_enable_ps enable_ps ps_value=%d\n",ps_value);
		ps_report_interrupt_data(ps_value);
		APS_LOG("apds9110_enable_ps ps=%d, ps_value=%d\n",obj->ps,ps_value);

		obj->ps_state=ps_value;

		// enable interrupt
		databuf[0] = APDS9110_DD_INT_CFG_ADDR;
		databuf[1] = 0x11;

		res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			goto EXIT_ERR;
		}

		#ifdef VENDOR_EDIT 	
		//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
		wake_up(&enable_ps);
		#endif
		enable_irq(obj->hw->eint_gpio);
	} else {
		if(test_bit(CMC_BIT_ALS, &obj->enable))
			databuf[1] = databuf[0]&~APDS9110_DD_PRX_EN;			
		else				
			databuf[1] = databuf[0]&~(APDS9110_DD_PRX_EN|APDS9110_DD_ALS_EN);	
		
		databuf[0] = APDS9110_DD_MAIN_CTRL_ADDR;
		APS_LOG("APDS9110_DD_MAIN_CTRL_ADDR disable ps value = %x\n", databuf[1]);	
		res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		clear_bit(CMC_BIT_PS, &obj->enable);
		obj->cali=0;
		disable_irq_nosync(obj->hw->eint_gpio);
	}
	
	//mdelay(200);
	return 0;
	
EXIT_ERR:
	APS_ERR("apds9110_enable_ps fail\n");
	return res;
}

static void apds9110_esd_handle(struct i2c_client *client, int esd_type)
{
    struct apds9110_priv *obj = i2c_get_clientdata(client);
    int als_state =0, ps_state = 0;

    als_state =  test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
    ps_state = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);

    if(esd_type == 1)    // intflag err
    {

	 msleep(100);

        APS_ERR( "%s , esd err, reinit device.\n",__func__);
        
	 if(als_state > 0)
		{
		    apds9110_enable_als(client, 0);
        	}
        if(ps_state  > 0)
        	{
		    apds9110_enable_ps(client, 0);
		}

	if(als_state>0 || ps_state >0)
		
	{
		apds9110_init_client(client);
		msleep(100);
	}

	 if(als_state > 0)
		{
		    apds9110_enable_als(client, 1);
       	}
        if(ps_state > 0)
       	{
		    apds9110_enable_ps(client, 1);
		}        

    }
}

/*----------------------------------------------------------------------------*/
static int apds9110_check_intr(struct i2c_client *client) 
{
	// kk 13-May-2015
	struct apds9110_priv *obj = i2c_get_clientdata(client);
	int res, intp, intl;
	u8 buffer[2];
	int try=3;

	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	return 0;

	while(try--){
		buffer[0] = APDS9110_DD_MAIN_STATUS_ADDR;
		res = apds9110_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
		if (res > 0) {
			break;
		}
	}
	if (res <= 0) {
		goto EXIT_ERR;
	}

	APS_ERR("buffer0 = 0X%x\n", buffer[0] );

	res = 0;
	intp = 0;
	intl = 0;

	//if((buffer[0] == 0)||(buffer[0] == 0x20))        
		//apds9110_esd_handle(client,1);
	
	if (0 != (buffer[0] & APDS9110_DD_PRX_INT_STATUS)) {
		res = 0;
		intp = 1;
	}
	
	if(0 != (buffer[0] & APDS9110_DD_ALS_INT_STATUS)) {
		res = 0;
		intl = 1;		
	}

	// kk 13-May-2015
	if(0 == (buffer[0] & APDS9110_DD_PRX_LOGICAL_STATUS)) {
		obj->ps_state = 1; // far		
	}
	else {
		obj->ps_state = 0; // near		
	}

	return res;

EXIT_ERR:
	APS_ERR("apds9110_check_intr fail\n");
	return 1;
}

/*-----------------------------------------------------------------------------*/
void apds9110_eint_func(void)
{
	struct apds9110_priv *obj = g_apds9110_ptr;

	if (!obj) {
		return;
	}

	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}

static irqreturn_t apds9110_interrupt(int vec, void *info)
{
	struct apds9110_priv *obj = g_apds9110_ptr;
	struct alsps_hw *hw = apds9110_get_cust_alsps_hw();

	APS_FUN();

	if (!obj) {
		return IRQ_HANDLED;
	}

	disable_irq_nosync(hw->eint_gpio);

	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
	enable_irq(hw->eint_gpio);
	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
int apds9110_setup_eint(struct i2c_client *client)
{
	struct apds9110_priv *obj = i2c_get_clientdata(client);
	int err;
	int irq;

	struct alsps_hw *hw = apds9110_get_cust_alsps_hw();

	err = gpio_request(hw->eint_gpio, "apds_irq");
	if (err)
	{
		APS_ERR("Unable to request GPIO.\n");
		return -1;
	}
	gpio_direction_input(hw->eint_gpio);
	irq = gpio_to_irq(hw->eint_gpio);
	if (irq < 0)
	{
		APS_ERR("Unable to request gpio irq. int pin = %d, irq = %d, err=%d\n", hw->eint_gpio, irq, err);
		gpio_free(hw->eint_gpio);
		return -1;
	}

	if (request_irq(irq, apds9110_interrupt, IRQF_TRIGGER_FALLING, "apds9110", (void *)client))
	{
		APS_ERR("%s Could not allocate APDS9950_INT !\n", __func__);
		return -1;
	}

	irq_set_irq_wake(irq, 1);

	g_apds9110_ptr = obj;
	
	//mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	//mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	//mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	//mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	//mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	//mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, apds9110_eint_func, 0);

	//mt_eint_unmask(CUST_EINT_ALS_NUM);

	return 0;
}

/*----------------------------------------------------------------------------*/
extern ALSPS_DEV alsps_dev;

static int apds9110_init_client(struct i2c_client *client)
{
	struct apds9110_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	databuf[0] = APDS9110_DD_MAIN_CTRL_ADDR;
	databuf[1] = 0;
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9110_DD_PRX_LED_ADDR;
	#ifdef VENDOR_EDIT 	
	//ziqing.guo@BasicDrv.Sensor,2015/05/30, add for led freq
	if (alsps_dev == ALSPS_APDS_9922)
		databuf[1] = APDS9110_DD_LED_FREQ_90_KHZ|APDS9110_DD_PRX_DEFAULT_LED_CURRENT;
	else
		databuf[1] = APDS9110_DD_LED_FREQ_100_KHZ|APDS9110_DD_PRX_DEFAULT_LED_CURRENT;
	#endif
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9110_DD_PRX_PULSES_ADDR;
	if (alsps_dev == ALSPS_APDS_9922)
		databuf[1] = 28;
	else
		databuf[1] = APDS9110_DD_PRX_DEFAULT_PULSE;
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	/*databuf[0] = APDS9110_DD_PRX_MEAS_RATE_ADDR;
	//databuf[1] = 0x40|APDS9110_DD_PRX_DEFAULT_RES|APDS9110_DD_PRX_DEFAULT_MEAS_RATE;
	databuf[1] = 0x40|APDS9110_DD_PRX_DEFAULT_RES|APDS9110_DD_PRX_MEAS_RATE_6_25_MS;
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}*/

	// set quickest adc conversion time
	databuf[1] = 0x40|APDS9110_DD_PRX_DEFAULT_RES|APDS9110_DD_PRX_DEFAULT_MEAS_RATE;
	databuf[0] = APDS9110_DD_PRX_MEAS_RATE_ADDR;

	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}
/*
	databuf[0] = APDS9110_DD_ALS_MEAS_RATE_ADDR;
	databuf[1] = APDS9110_DD_ALS_DEFAULT_RES|APDS9110_DD_ALS_DEFAULT_MEAS_RATE;
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	databuf[0] = APDS9110_DD_ALS_GAIN_ADDR;
	databuf[1] = APDS9110_DD_ALS_DEFAULT_GAIN;
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}
*/
	databuf[0] = APDS9110_DD_INT_PERSISTENCE_ADDR;
	databuf[1] = APDS9110_DD_PRX_PERS_1; //lgc-2015-06-28
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}
/*
	databuf[0] = APDS9110_DD_ALS_THRES_UP_ADDR;
	databuf[1] = 0; //lgc-2015-06-28
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}
*/
	databuf[0] = APDS9110_DD_INT_CFG_ADDR;
	if (obj->hw->polling_mode_ps == 1)
		databuf[1] = 0x10;
	
	if (obj->hw->polling_mode_ps == 0)
		databuf[1] = 0x11;
	
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	if (0 == obj->hw->polling_mode_ps) {
		if (1 == ps_cali.valid) {
			databuf[0] = APDS9110_DD_PRX_THRES_LOW_0_ADDR;	
			databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
			res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if (res <= 0) {
				goto EXIT_ERR;
			}

			databuf[0] = APDS9110_DD_PRX_THRES_LOW_1_ADDR;	
			databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
			res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if (res <= 0) {
				goto EXIT_ERR;
			}
			
			databuf[0] = APDS9110_DD_PRX_THRES_UP_0_ADDR;	
			databuf[1] = (u8)(ps_cali.close & 0x00FF);
			res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if (res <= 0) {
				goto EXIT_ERR;
			}

			databuf[0] = APDS9110_DD_PRX_THRES_UP_1_ADDR;	
			databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
			res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0) {
				goto EXIT_ERR;
			}
		} else {
			apds9110_set_psensor_threshold(client);
		}
	}

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if ((res = apds9110_setup_eint(client))!=0) {
		APS_ERR("setup eint: %d\n", res);
		return res;
	}

	if ((res = apds9110_check_intr(client))) {
		APS_ERR("check/clear intr: %d\n", res);
		return res;
	}

	return APDS9110_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int apds9110_read_als(struct i2c_client *client, u32 *data, bool calied)
{	 
	u32 clr_value, als_value;	 
	u8 buffer[3];
	u16 als_time;
	u16 als_gain;
	int res = 0;
	int factor = 5;
	u32 als = 0;
	struct alsps_hw *hw = apds9110_get_cust_alsps_hw();

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	if(!apds9110_obj)
	{
		APS_ERR("apds9110_obj is null!!\n");
		return -1;
	}

	buffer[0]=APDS9110_DD_CLEAR_DATA_ADDR;
	res = apds9110_i2c_master_operate(client, buffer, 0x301, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	clr_value = buffer[0] | (buffer[1]<<8) | (buffer[2]<<16);
	//APS_LOG("clr_value=%d\n", clr_value);

	buffer[0]=APDS9110_DD_ALS_DATA_ADDR;
	res = apds9110_i2c_master_operate(client, buffer, 0x301, I2C_FLAG_READ);
	if (res <= 0) { 
		goto EXIT_ERR;
	}

	als_value = buffer[0] | (buffer[1]<<8) | (buffer[2]<<16);
	//APS_LOG("als_value=%d\n", als_value);

	buffer[0]=APDS9110_DD_ALS_MEAS_RATE_ADDR;
	res = apds9110_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	als_time = (buffer[0]>>4)&0x07;

	if (als_time == 0)
		als_time = 400;
	else if (als_time == 1)
		als_time = 200;
	else if (als_time == 2)
		als_time = 100;
	else if (als_time == 3)
		als_time = 50;
	else
		als_time = 25;

	buffer[0]=APDS9110_DD_ALS_GAIN_ADDR;
	res = apds9110_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	als_gain = buffer[0]&0x07;

	if (als_gain == 0)
		als_gain = 1;
	else if (als_gain == 1)
		als_gain = 3;
	else if (als_gain == 2)
		als_gain = 6;
	else if (als_gain == 3)
		als_gain = 9;
	else
		als_gain = 18;

	if((!hw->als_gain1)||(!hw->als_gain2)){
		hw->als_gain1 = 1;
		hw->als_gain2 = 1;
	}
	/*if(als_value<=3){
		*data = als_value;
		return 0;
	}*/

	als = ((((long)als_value*APDS9110_DD_LUX_FACTOR*factor*hw->als_gain1)<<1)/((long)als_time*als_gain*hw->als_gain2)+1)>>1;
	/*
	if(calied){
		*data = ((((long)als_value*APDS9110_DD_LUX_FACTOR*factor*hw->als_gain1*apds9110_obj->gain)<<1)/((long)als_time*als_gain*hw->als_gain2*1000)+1)>>1;
		//APS_LOG("apds9110_read_als calied   lux = %d,raw %d[%d,%d,%d,%d/%d,%d,%d,%d]\n", *data,als_value,APDS9110_DD_LUX_FACTOR,factor,hw->als_gain1,apds9110_obj->gain,als_time,als_gain,hw->als_gain2,1000);
	}else{
		*data = ((((long)als_value*APDS9110_DD_LUX_FACTOR*factor*hw->als_gain1)<<1)/((long)als_time*als_gain*hw->als_gain2)+1)>>1;
		//APS_LOG("apds9110_read_als uncalied lux = %d,raw %d[%d,%d,%d/%d,%d,%d]\n", *data,als_value,APDS9110_DD_LUX_FACTOR,factor,hw->als_gain1,als_time,als_gain,hw->als_gain2);
	}*/

	if (alsps_dev == ALSPS_APDS_9922){
		if(als>120){
			als *= 13;
			als /= 10;
		}
	}

	if(calied){
		als *= apds9110_obj->gain;
		if (alsps_dev == ALSPS_APDS_9922){
			if(apds9110_obj->gain!=1000){
				als /= 1250;
				#if 0
				if(als<=5){
					als *= 100;
					als /= 115;
				}else if(als<=9){
					als *= 100;
					als /= 85;
				}else if(als<=17){
					als *= 100;
					als /= 61;
				}else if(als<=24){
					als *= 100;
					als /= 62;
				}else if(als<=31){
					als = 38;
				}else if(als<=61){
					als *= 100;
					als /= 79;
				}else if(als<=120){
					if(als<77){
						als = 77;
					}else{
						als *= 100;
						als /= 120;
					}
				}
				#endif
			}else{
				als /= 1000;
			}
		}else{
			als /= 1000;
		}

	}

	*data=als;

	return 0;	 

EXIT_ERR:
	APS_ERR("apds9110_read_als fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int apds9110_get_als_value(struct apds9110_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	
	for (idx = 0; idx < obj->als_level_num; idx++) {
		if (als < obj->hw->als_level[idx]) {
			break;
		}
	}

	if (idx >= obj->als_value_num) {
		APS_ERR("apds9110_get_als_value exceed range\n");
		idx = obj->als_value_num - 1;
	}

	if (1 == atomic_read(&obj->als_deb_on)) {
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if (time_after(jiffies, endt)) {
			atomic_set(&obj->als_deb_on, 0);
		}

		if (1 == atomic_read(&obj->als_deb_on)) {
			invalid = 1;
		}
	}

	if (!invalid) {
#if defined(MTK_AAL_SUPPORT)
		int level_high = obj->hw->als_level[idx];
		int level_low = (idx > 0) ? obj->hw->als_level[idx-1] : 0;
		int level_diff = level_high - level_low;
		int value_high = obj->hw->als_value[idx];
		int value_low = (idx > 0) ? obj->hw->als_value[idx-1] : 0;
    int value_diff = value_high - value_low;
    int value = 0;

		if ((level_low >= level_high) || (value_low >= value_high))
			value = value_low;
		else
			value = (level_diff * value_low + (als - level_low) * value_diff + ((level_diff + 1) >> 1)) / level_diff;

		APS_DBG("ALS: %d [%d, %d] => %d [%d, %d] \n", als, level_low, level_high, value, value_low, value_high);
		
		return value;
#endif			        

		//APS_ERR("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	} else {
		//APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
long apds9110_read_ps(struct i2c_client *client, u16 *data)
{
	struct apds9110_priv *obj = i2c_get_clientdata(client);	
	u8 buffer[2];
	u16 ps_data;
	long res = 0;

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=APDS9110_DD_PRX_DATA_ADDR;
	res = apds9110_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	
	ps_data = (buffer[0] | (buffer[1]<<8))&0x7FF;
	//APS_LOG("apds9110_read_ps ps_data=%d", ps_data);
	if(ps_data < obj->ps_cali)
		*data = 0;
	else
		*data = ps_data - obj->ps_cali; // minus crosstalk?

	return 0;

EXIT_ERR:
	APS_ERR("apds9110_read_ps fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static int apds9110_get_ps_value(struct apds9110_priv *obj, u16 ps)
{
	int val;// mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	
	val = obj->ps_state;

	if (atomic_read(&obj->ps_suspend)) {
		invalid = 1;
	} else if (1 == atomic_read(&obj->ps_deb_on)) {
		unsigned long endt = atomic_read(&obj->ps_deb_end);

		if (time_after(jiffies, endt)) {
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on)) {
			invalid = 1;
		}
	}

	if (!invalid) {
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	} else {
		APS_LOG("apds9110_get_ps_value  val  = %d",val);
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
static int apds9110_get_first_ps_value(struct apds9110_priv *obj, u16 ps)
{
	int val;// mask = atomic_read(&obj->ps_mask);
	int invalid = 0;

#if 1
	if (ps_cali.valid == 1) {
		if (ps >ps_cali.close) {
			val = 0;  /*close*/
		} else if (ps < ps_cali.far_away) {
			val = 1;  /*far away*/
		} else {
			val = 1;
		}

		
	} else {
		if (ps  > atomic_read(&obj->ps_thd_val_high)) {
			val = 0;/*close*/;
		} else if (ps  < atomic_read(&obj->ps_thd_val_low)) {
			val = 1;/*far away*/
		} else {
			val = 1;
		}
		
	}
#else

	val = obj->ps_state;
#endif

	if (atomic_read(&obj->ps_suspend)) {
		invalid = 1;
	} else if (1 == atomic_read(&obj->ps_deb_on)) {
		unsigned long endt = atomic_read(&obj->ps_deb_end);

		if (time_after(jiffies, endt)) {
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on)) {
			invalid = 1;
		}
	}

	if (!invalid) {
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	} else {
		APS_LOG("apds9110_get_ps_value 222  val  = %d",val);
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
//#define DEBUG_APDS9110
static void apds9110_eint_work(struct work_struct *work)
{
	struct apds9110_priv *obj = (struct apds9110_priv *)container_of(work, struct apds9110_priv, eint_work);
	int err;
	int ps_value;
	//hwm_sensor_data sensor_data;

	if ((err = apds9110_check_intr(obj->client))) {
		APS_ERR("check intrs: %d\n", err);
		goto EXIT_ERR;
	} else {
		//get raw data
		apds9110_read_ps(obj->client, &obj->ps);
		apds9110_read_als(obj->client, &obj->als,true);
		// kk 13-May-2015
		APS_ERR("rawdata ps=%d (%d)(low=%d high=%d) als=%d - %d\n", obj->ps, obj->first_int_enable,atomic_read(&obj->ps_thd_val_low),atomic_read(&obj->ps_thd_val_high), obj->als,obj->ps_state);
		//APS_ERR("apds9110 int top half time = %lld\n", int_top_time);
			if(obj->als<HIGHLIGHT_MODE_ALS_THRESHOLD||obj->ps<900){
				ps_value = apds9110_get_ps_value(obj, obj->ps);
				ps_report_interrupt_data(ps_value);
				APS_ERR("report ps=%d\n", ps_value);
				/*
				sensor_data.values[0] = apds9110_get_ps_value(obj, obj->ps);
				sensor_data.value_divide = 1;
				sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;	
				APS_LOG("apds9110_eint_work report ps=%d\n", sensor_data.values[0] );			
				//let up layer to know
				if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data))) {
				APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
				}*/
			}
	}
	
	//enable_irq(obj->hw->eint_gpio);
	return;

EXIT_ERR:
	//enable_irq(obj->hw->eint_gpio);
	return;
}

/****************************************************************************** 
 * APDS-9921 PS Calibration Function 
******************************************************************************/
int	apds9110_calibration_ps(struct i2c_client *client)
{

    u8 databuf[3];
    u8 buffer[2];
    int res;
    int i;
    int ps_data=0;
    //int ps_threshold=0,ps_hysteresis_threshold=0;
    int ps_offset;
	//clear offset
	databuf[0] = APDS9110_DD_PRX_CAN_0_ADDR;
	databuf[1] = 0x00;
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);//xtalk cancellation
	if (res <= 0) {
		goto EXIT_ERR;
	}
	
	databuf[0] = APDS9110_DD_PRX_CAN_1_ADDR;
	databuf[1] = 0x00;
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);//xtalk cancellation
	if (res <= 0) {
		goto EXIT_ERR;
	}
	msleep(100);

	for (i=0; i<APDS9110_PS_CAL_LOOP; i++) {
		
		msleep(10);// must be greater than prx meas rate
		buffer[0]=APDS9110_DD_PRX_DATA_ADDR;
		res = apds9110_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
		if (res <= 0) {
        	goto EXIT_ERR;
		}	
		ps_data += (buffer[0] | (buffer[1]<<8))&0x7FF;
	}

	ps_data = ps_data/i;
	
	APS_ERR("APDS9110_PS_CAL pdata = %d\n", ps_data);

	if (ps_data > APDS9110_PS_CAL_CROSSTALK_HIGH&&ps_data <= 900){
		ps_offset = ps_data - APDS9110_PS_CAL_CROSSTALK_HIGH;

		databuf[0] = APDS9110_DD_PRX_CAN_0_ADDR;
		databuf[1] = (u8)(ps_offset & 0x00FF);
		res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);//xtalk cancellation
		if (res <= 0) {
		 goto EXIT_ERR;
		}

		databuf[0] = APDS9110_DD_PRX_CAN_1_ADDR;
		databuf[1] = (u8)((ps_offset & 0xFF00) >> 8);
		res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);//xtalk cancellation
		if (res <= 0) {
		 goto EXIT_ERR;
		}				
		APS_ERR("success\n");
		return 2;

	}else if(ps_data<=APDS9110_PS_CAL_CROSSTALK_HIGH){
		APS_ERR("skip\n");
		return 1;
	}

EXIT_ERR:
//	mt_eint_unmask(CUST_EINT_ALS_NUM);
	APS_ERR("apds9110_i2c_master_operate\n");
	return -1;
}
	
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int apds9110_open(struct inode *inode, struct file *file)
{
	file->private_data = apds9110_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int apds9110_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9110_set_psensor_threshold(struct i2c_client *client)
{
	struct apds9110_priv *obj = i2c_get_clientdata(client);
	
#ifdef VENDOR_EDIT
//ye.zhang@BSP.PSW, 2016-04-13, delete for set PS threshold at a time
	u8 databuf[2];
	int res = 0;

	//APS_ERR("apds9110_set_psensor_threshold function high: 0x%x, low:0x%x\n", atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));

	databuf[0] = APDS9110_DD_PRX_THRES_LOW_0_ADDR;	
	databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		return -1;
	}

	databuf[0] = APDS9110_DD_PRX_THRES_LOW_1_ADDR;
	databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) & 0xFF00) >> 8);
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		return -1;
	}

	databuf[0] = APDS9110_DD_PRX_THRES_UP_0_ADDR;
	databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_high) & 0x00FF);
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		return -1;
	}

	databuf[0] = APDS9110_DD_PRX_THRES_UP_1_ADDR;	
	databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) & 0xFF00) >> 8);;
	res = apds9110_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if (res <= 0) {
		return -1;
	}

	//APS_ERR("apds9110_set_psensor_threshold 222 function high: 0x%x, low:0x%x\n", atomic_read(&obj->ps_thd_val_high), atomic_read(&obj->ps_thd_val_low));
#else//VENDOR_EDIT
	u8 databuf[5];
	int res = 0;

	databuf[0] = APDS9110_DD_PRX_THRES_UP_0_ADDR;
	databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_high) & 0x00FF);
	databuf[2] = (u8)((atomic_read(&obj->ps_thd_val_high) & 0xFF00) >> 8);
	databuf[3] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);
	databuf[4] = (u8)((atomic_read(&obj->ps_thd_val_low) & 0xFF00) >> 8);
	res = apds9110_i2c_master_operate(client, databuf, 0x5, I2C_FLAG_WRITE);
	if (res <= 0) {
		return -1;
	}
#endif//VENDOR_EDIT
	return 0;
}

static long apds9110_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	
		void __user *arg64 = compat_ptr(arg);

		//APS_ERR( "%s cmd = 0x%04x", __FUNCTION__, cmd);
	
		if(!file->f_op || !file->f_op->unlocked_ioctl)
		{
			APS_ERR( "file->f_op OR file->f_op->unlocked_ioctl is null!\n");
			return -ENOTTY;
		}

	switch (cmd)
	{
		case COMPAT_ALSPS_SET_PS_MODE:
			ret = file->f_op->unlocked_ioctl(file, ALSPS_SET_PS_MODE, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_SET_PS_MODE is failed!\n");
				}
				break;
			break;

		case COMPAT_ALSPS_GET_PS_MODE:
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_MODE, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_GET_PS_MODE is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_PS_DATA:    
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_GET_PS_DATA is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_PS_RAW_DATA:    
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_RAW_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_GET_PS_RAW_DATA is failed!\n");
				}
				break;

		case COMPAT_ALSPS_SET_ALS_MODE:
			ret = file->f_op->unlocked_ioctl(file, ALSPS_SET_ALS_MODE, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_SET_ALS_MODE is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_ALS_MODE:
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_MODE, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_GET_ALS_MODE is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_ALS_DATA: 
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_GET_ALS_DATA is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_ALS_RAW_DATA:    
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_RAW_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_GET_ALS_RAW_DATA is failed!\n");
				}
				break;

        	case COMPAT_ALSPS_SET_PS_THRESHOLD_OPPO: //0x21 lycan add for 
            ret = file->f_op->unlocked_ioctl(file, ALSPS_SET_PS_THRESHOLD_OPPO, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_SET_PS_THRESHOLD_OPPO is failed!\n");
				}
				break;
        	case COMPAT_ALSPS_GET_CUST_PS_ADJUST_PARA: //0x22 lycan add for
          	ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_CUST_PS_ADJUST_PARA, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_GET_CUST_PS_ADJUST_PARA is failed!\n");
				}
				break;


		case COMPAT_ALSPS_GET_HIGHLIGHT_DATA: 
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_HIGHLIGHT_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_GET_HIGHLIGHT_DATA is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_WAKEUP_STATUS:			
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_WAKEUP_STATUS, (unsigned long)arg64);
				if(ret < 0)
				{
					APS_ERR( "COMPAT_ALSPS_GET_WAKEUP_STATUS is failed!\n");
				}
				break;

		default:
			 APS_ERR( "%s not supported = 0x%04x", __FUNCTION__, cmd);
			 return -ENOIOCTLCMD;
			 break;
	}

	return 0 ;
}

/*----------------------------------------------------------------------------*/
static void apds9110_set_ps_threshold(struct i2c_client *client, int low_threshold, int high_threshold)
{
    struct apds9110_priv *obj = i2c_get_clientdata(client);
    atomic_set(&obj->ps_thd_val_high,  high_threshold);
    atomic_set(&obj->ps_thd_val_low,  low_threshold);//need to confirm
    //APS_ERR("high: %d, low:%d\n",atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));
    apds9110_set_psensor_threshold(obj->client);
}

/*----------------------------------------------------------------------------*/
static long apds9110_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct apds9110_priv *obj = i2c_get_clientdata(client);
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_result;
	int ps_cali;
	int threshold[2];
	int res;
	u8 buffer[4];
	struct alsps_hw *hw = NULL;
    	struct set_ps_thd_para set_ps_thd_para;

	switch (cmd) {
		case ALSPS_SET_PS_MODE:
			if (copy_from_user(&enable, ptr, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}

			if (enable) {
				if ((err = apds9110_enable_ps(obj->client, 1))) {
					APS_ERR("enable ps fail: %ld\n", err);
					goto err_out;
				}
				
				//set_bit(CMC_BIT_PS, &obj->enable);
			} else {
				if ((err = apds9110_enable_ps(obj->client, 0))) {
					APS_ERR("disable ps fail: %ld\n", err);
					goto err_out;
				}

				//clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if (copy_to_user(ptr, &enable, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:     

			/*buffer[0] = APDS9110_DD_PRX_MEAS_RATE_ADDR;
			res = apds9110_i2c_master_operate(obj->client, buffer, 0x101, I2C_FLAG_READ);
			if (res <= 0) {
				APS_ERR("apds i2c error");
			}
			buffer[2]=buffer[0] ;
			APS_LOG("APDS9110_DD_PRX_MEAS_RATE_ADDR = 0X%x\n", buffer[2] );*/
			#if APDS_DEBUG
			dump_count++;
			if(dump_count>dump_count_max){
				apds9110_esd_debug();
				dump_count = 0;
			}
			#endif

			buffer[0] = APDS9110_DD_ALS_THRES_UP_ADDR;
			res = apds9110_i2c_master_operate(obj->client, buffer, 0x101, I2C_FLAG_READ);
			if (res <= 0) {
				APS_ERR("apds i2c error");
			}
			buffer[3]=buffer[0] ;
			#if APDS_DEBUG
			//APS_LOG("APDS9110_DD_ALS_THRES_UP_ADDR = 0X%x\n", buffer[3] );
			#endif
		      // if(buffer[2] != 0x54 || buffer[3]!=0x40)
			if(buffer[3]==0xff)  	
				apds9110_esd_handle(obj->client,1);

			if ((err = apds9110_read_ps(obj->client, &obj->ps))) {
				goto err_out;
			}
			//APS_ERR("ALSPS_GET_PS_DATA\n");
			if( ((obj->ps<atomic_read(&obj->ps_thd_val_low))&&(obj->ps_state==0))
				|| ((obj->ps>atomic_read(&obj->ps_thd_val_high))&&(obj->ps_state==1)) ){
					msleep(15);
					if ((err = apds9110_read_ps(obj->client, &obj->ps))) {
						goto err_out;
					}
					if( ((obj->ps<atomic_read(&obj->ps_thd_val_low))&&(obj->ps_state==0))
						|| ((obj->ps>atomic_read(&obj->ps_thd_val_high))&&(obj->ps_state==1)) ){
						APS_ERR("apds9110 int err ps raw:%d state:%d\n",obj->ps,obj->ps_state);
						apds9110_esd_handle(obj->client,1);
						goto err_out;
					}
			}
			dat = apds9110_get_ps_value(obj, obj->ps);
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			} 
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if ((err = apds9110_read_ps(obj->client, &obj->ps))) {
				goto err_out;
			}
			
			dat = obj->ps;

			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_SET_ALS_MODE:
			if (copy_from_user(&enable, ptr, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}

			if (enable) {
				if ((err = apds9110_enable_als(obj->client, 1))) {
					APS_ERR("enable als fail: %ld\n", err);
					goto err_out;
				}
				//set_bit(CMC_BIT_ALS, &obj->enable);
			} else {
				if ((err = apds9110_enable_als(obj->client, 0))) {
					APS_ERR("disable als fail: %ld\n", err);
					goto err_out;
				}
				//clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if (copy_to_user(ptr, &enable, sizeof(enable))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if ((err = apds9110_read_als(obj->client, &obj->als,true))) {
				goto err_out;
			}

			dat = apds9110_get_als_value(obj, obj->als);
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if ((err = apds9110_read_als(obj->client, &obj->als,true))) {
				goto err_out;
			}

			//dat = obj->als*5;
			dat = obj->als;//*obj->gain/1000;
			if (copy_to_user(ptr, &dat, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}   

			//APS_LOG("apds9110 ALSPS_GET_ALS_RAW_DATA = %d \n",dat);
			break;

		/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			if ((err = apds9110_read_ps(obj->client, &obj->ps))) {
				goto err_out;
			}

			if (obj->ps > atomic_read(&obj->ps_thd_val_high)) {
				ps_result = 0;
			} else ps_result = 1;
				
			if (copy_to_user(ptr, &ps_result, sizeof(ps_result))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_CLR_CALI:
			if (copy_from_user(&dat, ptr, sizeof(dat))) {
				err = -EFAULT;
				goto err_out;
			}

			if(dat == 0)
				obj->ps_cali = 0;
			break;

		case ALSPS_IOCTL_GET_CALI:
			ps_cali = obj->ps_cali ;
			if (copy_to_user(ptr, &ps_cali, sizeof(ps_cali))) {
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_SET_CALI:
			if (copy_from_user(&ps_cali, ptr, sizeof(ps_cali))) {
				err = -EFAULT;
				goto err_out;
			}

			obj->ps_cali = ps_cali;
			break;

		case ALSPS_SET_PS_THRESHOLD:
			if (copy_from_user(threshold, ptr, sizeof(threshold))) {
				err = -EFAULT;
				goto err_out;
			}

			//APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]);
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

			apds9110_set_psensor_threshold(obj->client);
				
			break;
				
		case ALSPS_GET_PS_THRESHOLD_HIGH:
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			//APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
			if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
				err = -EFAULT;
				goto err_out;
			}
			break;
				
		case ALSPS_GET_PS_THRESHOLD_LOW:
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			//APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
			if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
				err = -EFAULT;
				goto err_out;
			}
			break;
/*------------------------------------------------------------------------------------------*/

		case ALSPS_SET_PS_THRESHOLD_OPPO: //0x21 lycan add for 
           		if(copy_from_user(&set_ps_thd_para, ptr, sizeof(set_ps_thd_para)))
            		{
                		err = -EFAULT;
                		goto err_out;
            		}
#ifndef VENDOR_EDIT
//ye.zhang@BSP, remove for decrease log number
	     		 switch (set_ps_thd_para.algo_state) {
	      		 case PS_ADJUST_TREND_STATE : 
	                    APS_LOG("apds9110_set_ps_threshold ps average:%d state: TREND\n", set_ps_thd_para.ps_average);
	                    break;
	                case PS_ADJUST_NOTREND_STATE :
	                    APS_LOG("apds9110_set_ps_threshold ps average:%d state: NOTREND\n", set_ps_thd_para.ps_average);
	                    break;
	                case PS_ADJUST_HIGHLIGHT_STATE :
	                    APS_LOG("apds9110_set_ps_threshold ps average:%d state: HIGHLIGHT\n", set_ps_thd_para.ps_average);
	                    break;
	                case PS_ADJUST_AVOID_DIRTY_STATE : 
	                    APS_LOG("apds9110_set_ps_threshold ps average:%d state: AVOID_DIRTY\n", set_ps_thd_para.ps_average);
	                    break;
	                default:
	                    APS_LOG("apds9110_set_ps_threshold ps average:%d state: impossible\n", set_ps_thd_para.ps_average);
	                    break;
	            }
#endif//VNDOR_EDIT

	            apds9110_set_ps_threshold(obj->client,
	                    set_ps_thd_para.low_threshold, 
	                    set_ps_thd_para.high_threshold);
	            break;

	        case ALSPS_GET_CUST_PS_ADJUST_PARA: //0x22 lycan add for
	           	 APS_LOG(" %s()->case ALSPS_GET_CUST_PS_ADJUST_PARA \n", __FUNCTION__);
	            	hw = apds9110_get_cust_alsps_hw();
	            	if(copy_to_user(ptr, apds9110_ps_adjust_para_real, sizeof( struct ps_adjust_para)))
	           	 {
	               	 err = -EFAULT;
	                	goto err_out;
	            	}              
	            break;

		case ALSPS_GET_HIGHLIGHT_DATA: 
			if(obj->als<HIGHLIGHT_MODE_ALS_THRESHOLD)
				dat=0;
			/*buffer[0] = APDS9110_DD_INT_PERSISTENCE_ADDR;
			buffer[1] = APDS9110_DD_PRX_PERS_1; //lgc-2015-06-28
			res = apds9110_i2c_master_operate(client, buffer, 0x2, I2C_FLAG_WRITE);
			if (res <= 0) {
				goto  err_out;
			}*/
			else 
				dat=1;
			/*buffer[0] = APDS9110_DD_INT_PERSISTENCE_ADDR;
			buffer[1] = APDS9110_DD_PRX_PERS_5; //lgc-2015-06-28
			res = apds9110_i2c_master_operate(client, buffer, 0x2, I2C_FLAG_WRITE);
			if (res <= 0) {
				goto  err_out;
			}*/
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

#ifdef VENDOR_EDIT 	
			//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
		case ALSPS_GET_WAKEUP_STATUS:			
			wait_event_interruptible(enable_ps, test_bit(CMC_BIT_PS, &obj->enable));
			dat=1;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
           		 {
                		err = -EFAULT;
                		goto err_out;
            		}              
            		break;
#endif
		

			/*------------------------------------------------------------------------------------------*/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;
}

/*----------------------------------------------------------------------------*/
static struct file_operations apds9110_fops = {
	.owner          = THIS_MODULE,
	.open           = apds9110_open,
	.release        = apds9110_release,
	.unlocked_ioctl = apds9110_unlocked_ioctl,

#ifdef CONFIG_COMPAT
	.compat_ioctl = apds9110_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice apds9110_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "als_ps",
	.fops  = &apds9110_fops,
};

/*----------------------------------------------------------------------------*/
static int apds9110_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9110_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/
static void apds9110_early_suspend(struct early_suspend *h) 
{
	/*early_suspend is only applied for ALS*/
	struct apds9110_priv *obj = container_of(h, struct apds9110_priv, early_drv);
	//int err;

	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return;
	}
	
	/*atomic_set(&obj->als_suspend, 1);
	if (test_bit(CMC_BIT_ALS, &obj->enable)) {
		if ((err = apds9110_enable_als(obj->client, 0))) {
			APS_ERR("disable als fail: %d\n", err);
		}
	}*/
}

/*----------------------------------------------------------------------------*/
static void apds9110_late_resume(struct early_suspend *h)
{
	/*early_suspend is only applied for ALS*/
	struct apds9110_priv *obj = container_of(h, struct apds9110_priv, early_drv);
	//int err;
	
	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return;
	}

	/*atomic_set(&obj->als_suspend, 0);
	if (test_bit(CMC_BIT_ALS, &obj->enable)) {
		if ((err = apds9110_enable_als(obj->client, 1))) {
			APS_ERR("enable als fail: %d\n", err);
		}
	}*/
}

/*----------------------------------------------------------------------------*/

int apds9110_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int value;
	int err = 0;
	
	hwm_sensor_data* sensor_data;
	struct apds9110_priv *obj = (struct apds9110_priv *)self;
	
	//APS_FUN(f);
	switch (command) {
		case SENSOR_DELAY:
			if ((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			} else {	
				value = *(int *)buff_in;
				if (value) {
					if ((err = apds9110_enable_ps(obj->client, 1))) {
						APS_ERR("enable ps fail: %d\n", err);
						return -1;
					}

				} else {
					if ((err = apds9110_enable_ps(obj->client, 0))) {
						APS_ERR("disable ps fail: %d\n", err);
						return -1;
					}

				}
			}
			break;

		case SENSOR_GET_DATA:
			if ((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))) {
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			} else {
				sensor_data = (hwm_sensor_data *)buff_out;	
				apds9110_read_ps(obj->client, &obj->ps);
				APS_ERR("apds9110_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = apds9110_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;

		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int apds9110_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	int res;
	u8 buffer[4];
	u32 b[2];
	int i;
	struct apds9110_priv *obj = (struct apds9110_priv *)self;

	switch (command) {
		case SENSOR_DELAY:
			if ((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if ((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			} else {
				value = *(int *)buff_in;				
				if (value) {
					//if (!test_bit(CMC_BIT_ALS, &obj->enable))
						//{
							if ((err = apds9110_enable_als(obj->client, 1))) {
							APS_ERR("enable als fail: %d\n", err);
							return -1;
						       }
                                          	//set_bit(CMC_BIT_ALS, &obj->enable);
						//}
					
				} else {
					//if (!test_bit(CMC_BIT_PS, &obj->enable)) {
						
							if ((err = apds9110_enable_als(obj->client, 0))) {
							APS_ERR("disable als fail: %d\n", err);
							return -1;
							}
							//clear_bit(CMC_BIT_ALS, &obj->enable);	
						//}
				}
			}
			break;

		case SENSOR_GET_DATA:
			if ((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))) {
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			} else {

				buffer[0] = APDS9110_DD_ALS_THRES_UP_ADDR;
				res = apds9110_i2c_master_operate(obj->client, buffer, 0x101, I2C_FLAG_READ);
				if (res <= 0) {
					APS_ERR("apds i2c error");
				}
				buffer[3]=buffer[0] ;
				//APS_LOG("APDS9110_DD_ALS_THRES_UP_ADDR = 0X%x\n", buffer[3] );

			       //if(buffer[2] != 0x54 || buffer[3]!=0x40)  
				if(buffer[3]==0xff)   	   
					apds9110_esd_handle(obj->client,1);
				   
				sensor_data = (hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing known issue*/
				apds9110_read_als(obj->client, &obj->als,true);

				for (i = 0;i < 2;i++) {
					apds9110_read_als(obj->client, &obj->als,true);
					b[i] = obj->als;
				}

				(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
				//sensor_data->values[0] = apds9110_get_als_value(obj, obj->als);
				//sensor_data->values[0] =obj->als*5;
				sensor_data->values[0] =obj->als;//*obj->gain/1000;
				APS_ERR("apds9110 als SENSOR_GET_DATA = %d \n",sensor_data->values[0] );
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
 
			}
			break;

		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
static int apds9110_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, APDS9110_DEV_NAME);
	return 0;
}
/*
static int apds9110_als_open_report_data(int open)
{
	return 0;
}

static int apds9110_als_enable_nodata(int en)
{
	struct apds9110_priv *obj = i2c_get_clientdata(apds9110_i2c_client);
	int err = 0;
	if (en) {
		if ((err = apds9110_enable_als(obj->client, 1))) {
			APS_ERR("enable als fail: %d\n", err);
			return -1;
		}
	}else {
		if ((err = apds9110_enable_als(obj->client, 0))) {
			APS_ERR("disable als fail: %d\n", err);
			return -1;
		}
	}
	return 0;
}

static int apds9110_als_set_delay(u64 ns)
{
	return 0;
}

static int apds9110_als_get_data(int* value, int* status)
{
	int res;
	u8 buffer[4];
	u32 b[2];
	int i;
	struct apds9110_priv *obj = i2c_get_clientdata(apds9110_i2c_client);
	
	buffer[0] = APDS9110_DD_ALS_THRES_UP_ADDR;
	res = apds9110_i2c_master_operate(obj->client, buffer, 0x101, I2C_FLAG_READ);
	if (res <= 0) {
		APS_ERR("apds i2c error");
		return res;
	}
	
	buffer[3]=buffer[0];
	//APS_LOG("APDS9110_DD_ALS_THRES_UP_ADDR = 0X%x\n", buffer[3] );
	//if(buffer[2] != 0x54 || buffer[3]!=0x40)  
	if(buffer[3]==0xff){
		apds9110_esd_handle(obj->client,1);
	}	

	apds9110_read_als(obj->client, &obj->als,true);
	for (i = 0;i < 2;i++) {
		apds9110_read_als(obj->client, &obj->als,true);
		b[i] = obj->als;
	}
	(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
	if (get_project() == 15131 || get_project() == 15132 || get_project() == 15133 || get_project() == 15134)
		*value = (obj->als * 5) / 7;
	else
		*value = obj->als;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	APS_ERR("als = %d \n",*value);

	if(first_als_data){
		if((first_als_data>5) && (test_bit(CMC_BIT_PS, &obj->enable)==0)){
			buffer[0] = APDS9110_DD_ALS_MEAS_RATE_ADDR;
			buffer[1] = APDS9110_DD_ALS_MEAS_RES_18_BIT|APDS9110_DD_ALS_MEAS_RATE_100_MS;;
			apds9110_i2c_master_operate(obj->client, buffer, 0x2, I2C_FLAG_WRITE);
			APS_LOG("first_als_data reset\n");
			msleep(150);
			APS_LOG("first_als_data = 0\n");
			first_als_data = 0;
		}else{
			APS_LOG("first_als_data = %d\n",first_als_data);
			first_als_data++;
		}
	}

	return 0;
}
*/
static int apds9110_ps_open_report_data(int open)
{
	return 0;
}

static int apds9110_ps_enable_nodata(int en)
{
	int err = 0;
	struct apds9110_priv *obj = i2c_get_clientdata(apds9110_i2c_client);
	
	if (en) {
		if ((err = apds9110_enable_ps(obj->client, 1))) {
			APS_ERR("enable ps fail: %d\n", err);
			return -1;
		}
	} else {
		if ((err = apds9110_enable_ps(obj->client, 0))) {
			APS_ERR("disable ps fail: %d\n", err);
			return -1;
		}
	}
	return 0;
}

static int apds9110_ps_set_delay(u64 ns)
{
	return 0;
}

static int apds9110_ps_get_data(int* value, int* status)
{   
	struct apds9110_priv *obj = i2c_get_clientdata(apds9110_i2c_client);
	apds9110_read_ps(obj->client, &obj->ps);
	APS_ERR("apds9110_ps_get_data ps data=%d!\n",obj->ps);
	*value = apds9110_get_ps_value(obj, obj->ps);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;	
	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9110_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct apds9110_priv *obj;
	struct hwmsen_object obj_ps;//, obj_als;
	int err = 0;
/*	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};*/
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	u8 buffer[2];
	struct alsps_hw *hw = apds9110_get_cust_alsps_hw();
	//hwm_sensor_data sensor_data;

	APS_FUN();
	if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}

	#ifdef VENDOR_EDIT 	
	//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
	init_waitqueue_head(&enable_ps);
	#endif

	memset(obj, 0, sizeof(*obj));
	apds9110_obj = obj;
	obj->hw = apds9110_get_cust_alsps_hw();
	apds9110_get_addr(obj->hw, &obj->addr);

	/* for interrup work mode support */
	INIT_WORK(&obj->eint_work, apds9110_eint_work);

	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);

	obj->cali = 0;
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	obj->gain = 1000;

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	//set_bit(CMC_BIT_ALS, &obj->enable);
	//set_bit(CMC_BIT_PS, &obj->enable);

	obj->ps_cali = 0;

	apds9110_i2c_client = client;

	if (1 == obj->hw->polling_mode_ps) {
		obj_ps.polling = 1;
	} else {
		obj_ps.polling = 0;
	}
	
#ifdef VENDOR_EDIT
	//zhihong.lu@Prd.BSP.sensor,add for apds9922
	buffer[0]=APDS9110_DD_PART_ID_ADDR;
	err = apds9110_i2c_master_operate(client, buffer, 0x301, I2C_FLAG_READ);
	if (err <= 0)
	{
		APS_ERR("apds9110_device read device ID failed\n");
		goto exit_init_failed;
	}
	switch(buffer[0]){
		case 0xB3:
			alsps_dev = ALSPS_APDS_9922;
			if(hw->sec_para_supported){
				apds9110_ps_adjust_para_real=hw->p_ps_adjust_para2;
			}else{
				apds9110_ps_adjust_para_real=hw->p_ps_adjust_para;
			}
			break;
		default:
			alsps_dev = ALSPS_APDS_9921;
			apds9110_ps_adjust_para_real=hw->p_ps_adjust_para;
			break;
	}
#endif /*VENDOR_EDIT*/
	if (apds9110_ps_adjust_para_real != NULL)
	{
		atomic_set(&obj->ps_thd_val_high,  apds9110_ps_adjust_para_real->ps_thd_high_highlight);
		atomic_set(&obj->ps_thd_val_low,  apds9110_ps_adjust_para_real->ps_thd_low_highlight);
	}
	if ((err = apds9110_init_client(client))) {
		goto exit_init_failed;
	}

	APS_LOG("apds9110_init_client() OK!\n");

	if ((err = misc_register(&apds9110_device))) {
		APS_ERR("apds9110_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = apds9110_create_attr(&apds9110_init_info.platform_diver_addr->driver)))		
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
/*
	als_ctl.open_report_data= apds9110_als_open_report_data;
	als_ctl.enable_nodata = apds9110_als_enable_nodata;
	als_ctl.set_delay  = apds9110_als_set_delay;
	als_ctl.is_report_input_direct = false;
    als_ctl.is_support_batch = false;

	
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = apds9110_als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
*/
	ps_ctl.open_report_data= apds9110_ps_open_report_data;
	ps_ctl.enable_nodata = apds9110_ps_enable_nodata;
	ps_ctl.set_delay  = apds9110_ps_set_delay;
	ps_ctl.is_report_input_direct = true;
    ps_ctl.is_support_batch = false;
	
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = apds9110_ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
/*
	err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register light batch support err = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
*/	
	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register proximity batch support err = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = apds9110_early_suspend,
	obj->early_drv.resume   = apds9110_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	apds9110_init_flag = 0;

#ifdef VENDOR_EDIT
		//zhihong.lu@Prd.BSP.sensor,add for apds9922
	if (alsps_dev == ALSPS_APDS_9922)
		register_device_proc("Sensor_alsps", "apds9922", "AVAGOTECH");
	else register_device_proc("Sensor_alsps", "apds9110", "AVAGOTECH");
#endif /*VENDOR_EDIT*/

	APS_LOG("%s: OK\n", __func__);

	return 0;

//exit_hwmsen_attach_failed:
exit_sensor_obj_attach_fail:
exit_create_attr_failed:
	misc_deregister(&apds9110_device);
exit_misc_device_register_failed:
exit_init_failed:
	kfree(obj);
exit:
	apds9110_i2c_client = NULL;
// MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	apds9110_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int apds9110_i2c_remove(struct i2c_client *client)
{
	int err;	

	if((err = apds9110_delete_attr(&(apds9110_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("apds9110_delete_attr fail: %d\n", err);
	} 
	
	if ((err = misc_deregister(&apds9110_device))) {
		APS_ERR("misc_deregister fail: %d\n", err);
	}

	apds9110_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

/*----------------------------------------------------------------------------
static int apds9110_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = apds9110_get_cust_alsps_hw();

	apds9110_power(hw, 1);
	//apds9110_force[0] = hw->i2c_num;
	//apds9110_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",apds9110_force[0],apds9110_force[1]);
	if (i2c_add_driver(&apds9110_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}

	return 0;
}

----------------------------------------------------------------------------*/
static int  apds9110_local_init(void)
{
	struct alsps_hw *hw = apds9110_get_cust_alsps_hw();
	APS_FUN();
	apds9110_power(hw, 1);
	if(als_only == 0){
		APS_ERR("use als_ps device\n");
		return -1;
	}
	if (i2c_add_driver(&apds9110_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}

	if (-1 == apds9110_init_flag) {
		APS_ERR("probe init err\n");
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9110_remove(void)
{
	struct alsps_hw *hw = apds9110_get_cust_alsps_hw();

	APS_FUN();
	apds9110_power(hw, 0);
	i2c_del_driver(&apds9110_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init apds9110_init(void)
{
	//APS_FUN();
	struct alsps_hw *hw = apds9110_get_cust_alsps_hw();
	//apds9110_ps_adjust_para_real=hw->p_ps_adjust_para;
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &i2c_apds9110, 1);
	alsps_driver_add(&apds9110_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit apds9110_exit(void)
{
	APS_FUN();
	//platform_driver_unregister(&apds9110_alsps_driver);
}

/*----------------------------------------------------------------------------*/
module_init(apds9110_init);
module_exit(apds9110_exit);

/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Lee Kai Koon");
MODULE_DESCRIPTION("APDS9110 driver");
MODULE_LICENSE("GPL");
