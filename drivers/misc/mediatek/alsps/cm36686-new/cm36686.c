/* 
 * Author: yucong xiong <yucong.xion@mediatek.com>
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
#include <linux/oppo_devices_list.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include "cm36686.h"
#include <linux/sched.h>
#include <alsps.h>
#include <linux/batch.h>
#include <soc/oppo/oppo_project.h>
#ifdef CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define CM36686_DEV_NAME     "cm36686"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO 	APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  	APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR	APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO 	APS_TAG fmt, ##args)    

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

#define cm36686_PS_CAL_CROSSTALK_HIGH 1000
#define cm36686_PS_CAL_CROSSTALK_MAX 6000

//#define CAPELLA_DEBOUNCE_CANCELLATION
#if defined(CAPELLA_DEBOUNCE_CANCELLATION)
#define DEBOUNCE_LUX_LEVEL_INIT -1
#define DEBOUNCE_LUX_LEVEL_1 50
#define DEBOUNCE_LUX_LEVEL_2 200
#define DEBOUNCE_LUX_LEVEL_3 600
#define DEBOUNCE_LUX_LEVEL_4 1000
 
#define DEBOUNCE_THD_DENOMINATOR 100
#define DEBOUNCE_LEVEL_1_THD_NOMINATOR  15
#define DEBOUNCE_LEVEL_2_THD_NOMINATOR  5
#define DEBOUNCE_LEVEL_3_THD_NOMINATOR  3
#define DEBOUNCE_LEVEL_4_THD_NOMINATOR  2
#define DEBOUNCE_LEVEL_5_THD_NOMINATOR  1
 
static int curr_lux= DEBOUNCE_LUX_LEVEL_INIT;
static int curr_thd=0;
#endif//CAPELLA_DEBOUNCE_CANCELLATION

/******************************************************************************
 * extern functions
*******************************************************************************/
#ifdef CUST_EINT_ALS_TYPE
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#else
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
/*----------------------------------------------------------------------------*/
static int cm36686_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int cm36686_i2c_remove(struct i2c_client *client);
static int cm36686_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int cm36686_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int cm36686_i2c_resume(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id cm36686_i2c_id[] = {{CM36686_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_cm36686={ I2C_BOARD_INFO(CM36686_DEV_NAME, 0x60)};
static unsigned long long int_top_time = 0;
static unsigned int g_min_ps = 4095;
extern ALSPS_DEV alsps_dev;
static bool als_first_read = false;
long cm36686_read_ps(struct i2c_client *client, u16 *data);
static void cm36686_esd_handle(struct i2c_client *client, int esd_type);
static int cm36686_init_client(struct i2c_client *client);
static void cm36686_set_ps_threshold(struct i2c_client *client, int low_threshold, int high_threshold);

static int enable_count=0;

//struct alsps_hw *hw = cm36686_get_cust_alsps_hw();
struct ps_adjust_para *cm36686_ps_adjust_para_real = NULL;

#ifdef VENDOR_EDIT 	
//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
static DECLARE_WAIT_QUEUE_HEAD(enable_ps);
#endif
/*----------------------------------------------------------------------------*/
struct cm36686_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;
#ifdef CUSTOM_KERNEL_SENSORHUB
    struct work_struct init_done_work;
#endif

	/*misc*/
	u16 		als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on; 	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t 	trace;
	
	
	/*data*/
	int			als;
	u16 			ps;
	u8			_align;
	u16			als_level_num;
	u16			als_value_num;
	u32			als_level[C_CUST_ALS_LEVEL-1];
	u32			als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;
	int 		cali;
	int  i2c_err_time;
	int  esd_handle_flag;
	int  gain_als;
	
	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;
	ulong		enable; 		/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/
	
	/*early suspend*/
	#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
	#endif     
};
/*----------------------------------------------------------------------------*/

static struct i2c_driver cm36686_i2c_driver = {	
	.probe      = cm36686_i2c_probe,
	.remove     = cm36686_i2c_remove,
	.detect     = cm36686_i2c_detect,
	.suspend    = cm36686_i2c_suspend,
	.resume     = cm36686_i2c_resume,
	.id_table   = cm36686_i2c_id,
	.driver = {
		.name = CM36686_DEV_NAME,
	},
};

/*----------------------------------------------------------------------------*/
struct PS_CALI_DATA_STRUCT
{
	int close;
	int far_away;
	int valid;
};

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct i2c_client *cm36686_i2c_client = NULL;
static struct cm36686_priv *cm36686_obj = NULL;
//static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int cm36686_get_ps_value(struct cm36686_priv *obj, u16 ps);

static int cm36686_local_init(void);
static int cm36686_remove(void);
static int cm36686_init_flag =-1; // 0<==>OK -1 <==> fail
static struct alsps_init_info cm36686_init_info = {
		.name = "cm36686",
		.init = cm36686_local_init,
		.uninit = cm36686_remove,
	
};

/*----------------------------------------------------------------------------*/

static DEFINE_MUTEX(cm36686_mutex);


/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS    = 1,
	CMC_BIT_PS	   = 2,
}CMC_BIT;
/*-----------------------------CMC for debugging-------------------------------*/
typedef enum {
    CMC_TRC_ALS_DATA= 0x0001,
    CMC_TRC_PS_DATA = 0x0002,
    CMC_TRC_EINT    = 0x0004,
    CMC_TRC_IOCTL   = 0x0008,
    CMC_TRC_I2C     = 0x0010,
    CMC_TRC_CVT_ALS = 0x0020,
    CMC_TRC_CVT_PS  = 0x0040,
    CMC_TRC_DEBUG   = 0x8000,
} CMC_TRC;
/*-----------------------------------------------------------------------------*/

int CM36686_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	int res = 0;
	mutex_lock(&cm36686_mutex);
	switch(i2c_flag){	
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
	APS_LOG("CM36686_i2c_master_operate i2c_flag command not support!\n");
	break;
	}
	if(res < 0)
	{
		goto EXIT_ERR;
	}
	mutex_unlock(&cm36686_mutex);
	return res;
	EXIT_ERR:
	mutex_unlock(&cm36686_mutex);
	APS_ERR("CM36686_i2c_master_operate fail\n");
	if((obj->esd_handle_flag == 0)&&(( test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0) == 1)||( test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0) == 1)))
			cm36686_esd_handle(client, 0); // 0 mean: i2c err
	return res;
}
/*----------------------------------------------------------------------------
static void cm36686_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "CM36686")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "CM36686")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
********************************************************************/
int cm36686_enable_ps(struct i2c_client *client, int enable)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 databuf[3];
	int ps_value;

	if(enable == 1){
		set_bit(CMC_BIT_PS, &obj->enable);
		
		databuf[0] = CM36686_REG_ALS_CONF;
		res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_ALS_EXIT_ERR;
		}
		
		//APS_LOG("CM36686_REG_ALS_CONF value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

		databuf[2] = databuf[1];
		databuf[1] = databuf[0]&0xFE;		
		databuf[0] = CM36686_REG_ALS_CONF;
		
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_ALS_EXIT_ERR;
		}
		
		//APS_LOG("cm36686_enable_ps enable_ps\n");
		databuf[0]= CM36686_REG_PS_CONF3_MS;
		res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_PS_EXIT_ERR;
		}
		//APS_LOG("CM36686_REG_PS_CONF3_MS value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
		
		databuf[0]= CM36686_REG_PS_CONF1_2;
		res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_PS_EXIT_ERR;
		}
		//APS_LOG("CM36686_REG_PS_CONF1_2 value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
		databuf[2] = databuf[1];
		databuf[1] = databuf[0]&0xFE;
		
		databuf[0]= CM36686_REG_PS_CONF1_2;
		res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto ENABLE_PS_EXIT_ERR;
		}
		
		if (g_min_ps != 0 && g_min_ps + cm36686_ps_adjust_para_real->dirty_adjust_high_thd < cm36686_ps_adjust_para_real->ps_adjust_max)
			cm36686_set_ps_threshold(obj->client,g_min_ps + cm36686_ps_adjust_para_real->dirty_adjust_low_thd, g_min_ps + cm36686_ps_adjust_para_real->dirty_adjust_high_thd);
		else
			cm36686_set_ps_threshold(obj->client,cm36686_ps_adjust_para_real->ps_thd_low_highlight, cm36686_ps_adjust_para_real->ps_thd_high_highlight);
		
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		msleep(120);
		cm36686_read_ps(obj->client, &obj->ps);
		ps_value = cm36686_get_ps_value(obj, obj->ps);
		APS_LOG("cm36686_enable_ps enable_ps ps_value=%d\n",ps_value);
		ps_report_interrupt_data(ps_value);
		
		#ifdef VENDOR_EDIT 	
		//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
		wake_up(&enable_ps);
		#endif

		//mt_eint_unmask(CUST_EINT_ALS_NUM);
		enable_irq(obj->hw->eint_gpio);
		
	}else{
			APS_LOG("cm36686_enable_ps disable_ps\n");
			databuf[0]= CM36686_REG_PS_CONF1_2;
			res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
			if(res < 0)
			{
				APS_ERR("i2c_master_send function err\n");
				goto ENABLE_PS_EXIT_ERR;
			}
			
			//APS_LOG("CM36686_REG_PS_CONF1_2 value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

			databuf[2] = databuf[1];
			databuf[1] = databuf[0]|0x01;	
			databuf[0]= CM36686_REG_PS_CONF1_2;
			
			res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
			if(res < 0)
			{
				APS_ERR("i2c_master_send function err\n");
				goto ENABLE_PS_EXIT_ERR;
			}
			atomic_set(&obj->ps_deb_on, 0);

			if(!test_bit(CMC_BIT_ALS, &obj->enable))
				{	
					databuf[0] = CM36686_REG_ALS_CONF;
					res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
					if(res < 0)
					{
						APS_ERR("i2c_master_send function err\n");
						goto ENABLE_ALS_EXIT_ERR;
					}
					
					APS_LOG("CM36686_REG_ALS_CONF value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

					databuf[2] = databuf[1];
					databuf[1] = databuf[0]|0x01;
					databuf[0] = CM36686_REG_ALS_CONF;

					res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
					if(res < 0)
					{
						APS_ERR("i2c_master_send function err\n");
						goto ENABLE_ALS_EXIT_ERR;
					}
					atomic_set(&obj->als_deb_on, 0);
				}
			clear_bit(CMC_BIT_PS, &obj->enable);

			//mt_eint_mask(CUST_EINT_ALS_NUM);
			disable_irq(obj->hw->eint_gpio);

		}
	
	return 0;
	ENABLE_ALS_EXIT_ERR:
	ENABLE_PS_EXIT_ERR:
	return res;
}
/********************************************************************/
int cm36686_enable_als(struct i2c_client *client, int enable)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 databuf[3];

	if(enable == 1){
			als_first_read = true;
			set_bit(CMC_BIT_ALS, &obj->enable);
			enable_count++;
			APS_LOG("cm36686_enable_als enable_als[%d]\n",enable_count);
			if (test_bit(CMC_BIT_PS, &obj->enable)==0)
			{
				databuf[0] = CM36686_REG_ALS_CONF;
				res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
				if(res < 0)
				{
					APS_ERR("i2c_master_send function err\n");
					goto ENABLE_ALS_EXIT_ERR;
				}
				
				APS_LOG("CM36686_REG_ALS_CONF value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

				databuf[2] = databuf[1];
				databuf[1] = databuf[0]&0xFE;		
				databuf[0] = CM36686_REG_ALS_CONF;
				
				res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
				if(res < 0)
				{
					APS_ERR("i2c_master_send function err\n");
					goto ENABLE_ALS_EXIT_ERR;
				}
				atomic_set(&obj->als_deb_on, 1);
				atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			}
			
	}else{
		enable_count--;
		APS_LOG("cm36686_enable_als disable_als[%d]\n",enable_count);
		if(enable_count<=0){
			if (test_bit(CMC_BIT_PS, &obj->enable)==0)
			{
				databuf[0] = CM36686_REG_ALS_CONF;
				res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
				if(res < 0)
				{
					APS_ERR("i2c_master_send function err\n");
					goto ENABLE_ALS_EXIT_ERR;
				}
				
				APS_LOG("CM36686_REG_ALS_CONF value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

				databuf[2] = databuf[1];
				databuf[1] = databuf[0]|0x01;
				databuf[0] = CM36686_REG_ALS_CONF;

				res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
				if(res < 0)
				{
					APS_ERR("i2c_master_send function err\n");
					goto ENABLE_ALS_EXIT_ERR;
				}
				atomic_set(&obj->als_deb_on, 0);
			}
			clear_bit(CMC_BIT_ALS, &obj->enable);
			enable_count=0;
		}
	}
	return 0;
	ENABLE_ALS_EXIT_ERR:
	return res;
}
/********************************************************************/


static void cm36686_esd_handle(struct i2c_client *client, int esd_type)
{
    struct cm36686_priv *obj = i2c_get_clientdata(client);
    int als_state =0, ps_state = 0;
    int intflag_err = 0;
    //struct alsps_hw *hw = cm36686_get_cust_alsps_hw();
	
	//cm36686_power(hw, 0);//*****************  

    als_state = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0) ;
    ps_state = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0) ;

    if(esd_type == 0)    // i2c err
    {
    	obj->i2c_err_time ++;
	
       printk(KERN_ERR"%s , i2c err time = %d\n",__func__, obj->i2c_err_time); 
   }        
 
   else                // intflag err
 
   {
       intflag_err = 1;    
   }

	
if((obj->i2c_err_time > 0)||(intflag_err == 1))
	
{	
	obj->esd_handle_flag = 1;
	
	obj->i2c_err_time = 0;
	
	msleep(100);

	printk(KERN_ERR"%s , esd err, reinit device.\n",__func__);

       if(als_state > 0)	
	{
	       cm36686_enable_als(client,0);
       }
 
       if(ps_state > 0)
	
	{
		cm36686_enable_ps(client,0);
	   //cm36686_power(hw, 0);
       }

	if(als_state>0 || ps_state >0)
		
	{
		//cm36686_power(hw, 0);
   		//msleep(1000);
		//cm36686_power(hw, 1);
		cm36686_init_client(client);
		msleep(100);
	}
	
	if(als_state > 0)
	
	{
		cm36686_enable_als(client,1);	
	}
	
	if(ps_state > 0)
	
	{
		//cm36686_power(hw, 1);	
		//cm36686_init_client(client);
		cm36686_enable_ps(client,1);
	}	
	obj->esd_handle_flag = 0;
	
} 

}


long cm36686_read_ps(struct i2c_client *client, u16 *data)
{
	long res;
	u8 databuf[2];
	struct cm36686_priv *obj = i2c_get_clientdata(client);
#if 0//def CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    SCP_SENSOR_HUB_DATA_P pRsp = &req;
    CM36686_CUST_DATA *pCustData;
    int len;
#endif

#if 0//def CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_PROXIMITY;
    req.get_data_req.action = SENSOR_HUB_SET_CUST;
    
    pCustData = (CM36686_CUST_DATA *)(&req.set_cust_req.custData);

    pCustData->getPSRawData.action = CM36686_CUST_ACTION_GET_PS_RAW_DATA;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getPSRawData);
    
    res = SCP_sensorHub_req_send(&req, &len, 1);
    if (0 == res)
    {
        if (len != (offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getPSRawData)) ||
            SENSOR_HUB_SET_CUST != pRsp->rsp.action || 0 != pRsp->rsp.errCode)
        {
            APS_ERR("SCP_sensorHub_req_send failed!\n");
            goto READ_PS_EXIT_ERR;
        }

        pCustData = (CM36686_CUST_DATA *)(&pRsp->set_cust_rsp.custData);

        if (CM36686_CUST_ACTION_GET_PS_RAW_DATA != pCustData->getPSRawData.action)
        {
            APS_ERR("SCP_sensorHub_req_send failed!\n");
            goto READ_PS_EXIT_ERR;
        }

        databuf[0] = pCustData->getPSRawData.ps;
    }
    else
    {
        APS_ERR("SCP_sensorHub_req_send failed!\n");
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	databuf[0] = CM36686_REG_PS_DATA;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_PS_EXIT_ERR;
	}
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG){	
	APS_LOG("CM36686_REG_PS_DATA_1 value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	
		*data = ((databuf[1]<<8)|databuf[0]);
	//APS_LOG("CM36686_REG_PS_DATA_2 value= %d value_low = %d, value_high = %d\n",*data,databuf[0],databuf[1]);
	
	return 0;
	READ_PS_EXIT_ERR:
	return res;
}
/********************************************************************/
/********************************************************************/
long cm36686_read_intr(struct i2c_client *client, u16 *data)
{
	long res;
	u8 databuf[2];
	struct cm36686_priv *obj = i2c_get_clientdata(client);

	databuf[0] = CM36686_REG_INT_FLAG;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_PS_EXIT_ERR;
	}
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG){	
	APS_LOG("CM36686_REG_INT_FLAG value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
	}
	
		*data = ((databuf[1]<<8)|databuf[0]);

	
	return 0;
	READ_PS_EXIT_ERR:
	return res;
}
/********************************************************************/
long cm36686_read_thdl(struct i2c_client *client, u16 *data)
{
	long res;
	u8 databuf[2];
	struct cm36686_priv *obj = i2c_get_clientdata(client);

	databuf[0] = CM36686_REG_PS_THDL;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_PS_EXIT_ERR;
	}
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG){	
	APS_LOG("CM36686_REG_PS_THDL value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
	}
	
		*data = ((databuf[1]<<8)|databuf[0]);

	
	return 0;
	READ_PS_EXIT_ERR:
	return res;
}
/********************************************************************/
/********************************************************************/
long cm36686_read_thdh(struct i2c_client *client, u16 *data)
{
	long res;
	u8 databuf[2];
	struct cm36686_priv *obj = i2c_get_clientdata(client);

	databuf[0] = CM36686_REG_PS_THDH;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_PS_EXIT_ERR;
	}
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG){	
	APS_LOG("CM36686_REG_PS_THDH value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
	}
	
		*data = ((databuf[1]<<8)|databuf[0]);

	
	return 0;
	READ_PS_EXIT_ERR:
	return res;
}
/********************************************************************/
static int cm36686_read_als(struct i2c_client *client, int *data)
{
	int res;
#if 0//def CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    SCP_SENSOR_HUB_DATA_P pRsp = &req;
    CM36686_CUST_DATA *pCustData;
    int len;
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	u8 databuf[2];
	struct cm36686_priv *obj = i2c_get_clientdata(client);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#if 0//def CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_LIGHT;
    req.get_data_req.action = SENSOR_HUB_SET_CUST;
    
    pCustData = (CM36686_CUST_DATA *)(&req.set_cust_req.custData);

    pCustData->getALSRawData.action = CM36686_CUST_ACTION_GET_ALS_RAW_DATA;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getALSRawData);
    
    res = SCP_sensorHub_req_send(&req, &len, 1);
    if (0 == res)
    {
        if (len != (offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->getALSRawData)) ||
            SENSOR_HUB_SET_CUST != pRsp->rsp.action || 0 != pRsp->rsp.errCode)
        {
            APS_ERR("SCP_sensorHub_req_send failed!\n");
            goto READ_ALS_EXIT_ERR;
        }

        pCustData = (CM36686_CUST_DATA *)(&pRsp->set_cust_rsp.custData);

        if (CM36686_CUST_ACTION_GET_ALS_RAW_DATA != pCustData->getALSRawData.action)
        {
            APS_ERR("SCP_sensorHub_req_send failed!\n");
            goto READ_ALS_EXIT_ERR;
        }

        *data = pCustData->getALSRawData.als;
    }
    else
    {
        APS_ERR("SCP_sensorHub_req_send failed!\n");
    }
#else //#ifndef CUSTOM_KERNEL_SENSORHUB
	databuf[0] = CM36686_REG_ALS_DATA;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res < 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto READ_ALS_EXIT_ERR;
	}
	
	if(atomic_read(&obj->trace) & CMC_TRC_DEBUG){
		APS_LOG("CM36686_REG_ALS_DATA value: %d\n", ((databuf[1]<<8)|databuf[0]));
	}

	*data = (int)((databuf[1]<<8)|databuf[0])*3;

#endif //#ifndef CUSTOM_KERNEL_SENSORHUB
	
	return 0;
	READ_ALS_EXIT_ERR:
	return res;
}

static int cm36686_read_als_calied(struct i2c_client *client, int *data){
	int res;
	int als;
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	res = cm36686_read_als(client,&als);
	if(obj->gain_als > 0)
	{
		*data = als*obj->gain_als/1000;
		if (get_project() == 15131 || get_project() == 15132 || get_project() == 15133 || get_project() == 15134)
			*data = *data * 20 / 27;
	}else{
		*data = als;
	}
	return res;
}
/********************************************************************/
static int cm36686_get_ps_value(struct cm36686_priv *obj, u16 ps)
{
	int val;
	//int invalid = 0;
	val = 1;

	//APS_ERR("cm36686_get_ps_value ps: %d  high: %d, low:%d\n",ps,atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));

	if(ps > atomic_read(&obj->ps_thd_val_high))
	{
		val = 0;  /*close*/
	}
	else if(ps < atomic_read(&obj->ps_thd_val_low))
	{
		val = 1;  /*far away*/
	}
	
	/*if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			if(mask)
			{
				APS_DBG("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_DBG("PS:  %05d => %05d\n", ps, val);
			}
		}
		if(0 == test_bit(CMC_BIT_PS,  &obj->enable))
		{
		  //if ps is disable do not report value
		  APS_DBG("PS: not enable and do not report this value\n");
		  return -1;
		}
		else
		{
		   return val;
		}
		
	}	
	else
	{
		if(unlikely(atomic_read(&obj->trace) & CMC_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	*/
	return val;
}
/********************************************************************
static int cm36686_get_als_value(struct cm36686_priv *obj, u16 als)
{
		int idx;
		int invalid = 0;
		for(idx = 0; idx < obj->als_level_num; idx++)
		{
			if(als < obj->hw->als_level[idx])
			{
				break;
			}
		}
		if(idx >= obj->als_value_num)
		{
			APS_ERR("exceed range\n"); 
			idx = obj->als_value_num - 1;
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			unsigned long endt = atomic_read(&obj->als_deb_end);
			if(time_after(jiffies, endt))
			{
				atomic_set(&obj->als_deb_on, 0);
			}
			
			if(1 == atomic_read(&obj->als_deb_on))
			{
				invalid = 1;
			}
		}
	
		if(!invalid)
		{
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
			if (atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
			{
				APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
			}
			
			return obj->hw->als_value[idx];
		}
		else
		{
			if(atomic_read(&obj->trace) & CMC_TRC_CVT_ALS)
			{
				APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);	  
			}
			return -1;
		}

}


-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t cm36686_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
		atomic_read(&cm36686_obj->i2c_retry), atomic_read(&cm36686_obj->als_debounce), 
		atomic_read(&cm36686_obj->ps_mask), atomic_read(&cm36686_obj->ps_thd_val), atomic_read(&cm36686_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&cm36686_obj->i2c_retry, retry);
		atomic_set(&cm36686_obj->als_debounce, als_deb);
		atomic_set(&cm36686_obj->ps_mask, mask);
		atomic_set(&cm36686_obj->ps_thd_val, thres);        
		atomic_set(&cm36686_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&cm36686_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&cm36686_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	int als;
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	if((res = cm36686_read_als(cm36686_obj->client, &als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "%d\n", als);
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!cm36686_obj)
	{
		APS_ERR("cm3623_obj is null!!\n");
		return 0;
	}

	//int_gpio = mt_get_gpio_in(GPIO_ALS_EINT_PIN);

	 //cm36686_read_intr(cm36686_obj->client,&a);
	// cm36686_read_thdl(cm36686_obj->client,&a1);
	 //cm36686_read_thdh(cm36686_obj->client,&a2);
	
	if((res = cm36686_read_ps(cm36686_obj->client, &cm36686_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %zd\n", res);
	}
	else
	{
		//return snprintf(buf, PAGE_SIZE, "0x%04X  gpio:%d  a:%x\n a1:%x\n a2:%x\n", cm36686_obj->ps, int_gpio,a,a1,a2);     
		return snprintf(buf, PAGE_SIZE, "%d\n", cm36686_obj->ps);   
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_reg(struct device_driver *ddri, char *buf)
{
	int i,res = 0;
	u8 databuf[3];
	static u8 _buff[PAGE_SIZE/4 - 1]={0};

	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}


	for (i = 0x00; i <= 0x0c; i += 0x01){        
	databuf[0]=i;
	res = CM36686_i2c_master_operate(cm36686_obj->client, databuf, 0x201, I2C_FLAG_READ);
	
	if(res < 0)
	{
		//goto EXIT_INTR_ERR;
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}	

	sprintf(_buff,"%s 0x%x:L:%2x H:%2x",_buff,i,databuf[0],databuf[1]);
	if(i%0xF == 0)
		strcat(_buff,"\n");
}

	return sprintf(buf,"%s\n",_buff);
	
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	u32 code1, code2;
	int res;//int addr, cmd;
	u8 databuf[3];

	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "0x%x 0x%x", &code1, &code2))//sscanf(buf, "%x %x", &addr, &cmd)
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
	
	databuf[2] = code1 >> 8;
	databuf[1] = code1 & 0x00FF;
	databuf[0] = CM36686_REG_PS_CONF1_2;
	res = CM36686_i2c_master_operate(cm36686_obj->client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
			{
				APS_ERR("i2c_master_send function err\n");
			}
	databuf[2] = code2 >> 8;
	databuf[1] = code2 & 0x00FF;
	databuf[0] = CM36686_REG_PS_CONF3_MS;
	res = CM36686_i2c_master_operate(cm36686_obj->client, databuf, 0x3, I2C_FLAG_WRITE);
		if(res < 0)
			{
				APS_ERR("i2c_master_send function err\n");
			}		
			
	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	//u8 dat;
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	
	if(cm36686_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			cm36686_obj->hw->i2c_num, cm36686_obj->hw->power_id, cm36686_obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02X %02lX %02lX\n", 
				atomic_read(&cm36686_obj->als_cmd_val), atomic_read(&cm36686_obj->ps_cmd_val), 
				atomic_read(&cm36686_obj->ps_thd_val),cm36686_obj->enable, cm36686_obj->pending_intr);
	
	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&cm36686_obj->als_suspend), atomic_read(&cm36686_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct cm36686_priv *obj, const char* buf, size_t count, u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < cm36686_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", cm36686_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(cm36686_obj->als_level, cm36686_obj->hw->als_level, sizeof(cm36686_obj->als_level));
	}
	else if(cm36686_obj->als_level_num != read_int_from_buf(cm36686_obj, buf, count, 
			cm36686_obj->hw->als_level, cm36686_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < cm36686_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", cm36686_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(cm36686_obj->als_value, cm36686_obj->hw->als_value, sizeof(cm36686_obj->als_value));
	}
	else if(cm36686_obj->als_value_num != read_int_from_buf(cm36686_obj, buf, count, 
			cm36686_obj->hw->als_value, cm36686_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_gain_als(struct device_driver *ddri, char *buf)
{
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", cm36686_obj->gain_als);
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_gain_als(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	
	if(1 != sscanf(buf, "%d", &cm36686_obj->gain_als))
	{ 				
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;
}
static int cm36686_calibration_ps(struct i2c_client *client)
{
    u8 databuf[3];
    int res;
    int i;
    int ps_data=0;
    int ps_offset,pre_ps_offset;

	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}

	for (i=0; i<5; i++) {
		mdelay(10);// must be greater than prx meas rate
		databuf[0] = CM36686_REG_PS_DATA;
		res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto CALIB_EXIT_ERR;
		}

		if(atomic_read(&cm36686_obj->trace) & CMC_TRC_DEBUG){
			APS_LOG("CM36686_REG_ALS_DATA value: %d\n", ((databuf[1]<<8)|databuf[0]));
		}

		ps_data += (databuf[0] | (databuf[1]<<8));
	}

	ps_data = ps_data/i;

	APS_ERR("cm36686_PS_CAL pdata = %d\n", ps_data);

	if (ps_data > cm36686_PS_CAL_CROSSTALK_HIGH&&ps_data <= cm36686_PS_CAL_CROSSTALK_MAX){

		ps_offset = ps_data - cm36686_PS_CAL_CROSSTALK_HIGH;

		databuf[0] = CM36686_REG_PS_CANC;
		res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
		if(res < 0)
		{
			APS_ERR("i2c_master_send function err\n");
			goto CALIB_EXIT_ERR;
		}
		pre_ps_offset = (databuf[0] | (databuf[1]<<8));
		ps_offset += pre_ps_offset;

		APS_ERR("cm36686_PS_CAL pre_pdata = %d[0x%04x],new pdata = %d[0x%04x]\n", pre_ps_offset, pre_ps_offset, ps_data, ps_data);

		databuf[0] = CM36686_REG_PS_CANC;
		databuf[1] = (u8)(ps_offset & 0x00FF);
		databuf[2] = (u8)((ps_offset & 0xFF00)>>8);
		res = CM36686_i2c_master_operate(cm36686_obj->client, databuf, 0x3, I2C_FLAG_WRITE);//xtalk cancellation
		if (res <= 0) {
		 goto CALIB_EXIT_ERR;
		}
		return 2;
	}
	else if(ps_data<=cm36686_PS_CAL_CROSSTALK_HIGH)
	{
		return 1;
	}
CALIB_EXIT_ERR:
	APS_ERR("cm36686_calibration_ps fail\n");
	return -1;
}


/*----------------------------------------------------------------------------*/
static ssize_t cm36686_show_cali(struct device_driver *ddri, char *buf)
{
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "%d\n", cm36686_obj->cali);
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_cali(struct device_driver *ddri, const char *buf, size_t count)
{
	int calib;
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "%d", &calib))
	{
		if (calib==1)
			{
				cm36686_obj->cali=cm36686_calibration_ps(cm36686_obj->client);
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
static ssize_t cm36686_show_offset(struct device_driver *ddri, char *buf)
{
	int res = 0;
	u8 databuf[3];
	static u8 _buff[PAGE_SIZE/4 - 1]={0};

	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}

	databuf[0] = CM36686_REG_PS_CANC;//value need to confirm
	databuf[1] = 0x00;//0x00;
	databuf[2] = 0x00;
	res = CM36686_i2c_master_operate(cm36686_obj->client, databuf, 0x201, I2C_FLAG_READ);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		return 0;
	}

	APS_LOG("cm36686 ps CM36686_REG_PS_CANC command!\n");

	sprintf(_buff,"%d,%d",databuf[0],databuf[1]);

	return sprintf(buf,"%s\n",_buff);
}

/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_offset(struct device_driver *ddri, const char *buf, size_t count)
{
	int res,offsetL,offsetH;
	u8 databuf[3];
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}

	if(2 == sscanf(buf, "%d,%d", &offsetL,&offsetH))
	{
		databuf[0] = CM36686_REG_PS_CANC;
		databuf[1] = offsetL;
		databuf[2] = offsetH;
		res = CM36686_i2c_master_operate(cm36686_obj->client, databuf, 0x3, I2C_FLAG_WRITE);//xtalk cancellation
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
static ssize_t cm36686_show_ps_adjust_para(struct device_driver *ddri, char *buf)
{
	static char tmpbuf[500] = {0};
	int res;
	APS_FUN();
	res = prox_show_adjust_para(cm36686_ps_adjust_para_real,tmpbuf);
	if(res == -2){
		APS_ERR("cm36686_ps_adjust_para_real is null!!\n");
	}else{
		ALSPS_ERR("tmpbuf:%s",tmpbuf);
		return scnprintf(buf, PAGE_SIZE, "%s", tmpbuf);
	}
	return 1;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t cm36686_store_ps_adjust_para(struct device_driver *ddri, const char *buf, size_t count)
{
	int enable,res;
	int index,value;
	APS_FUN();
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return 0;
	}
	enable = test_bit(CMC_BIT_PS, &cm36686_obj->enable) ? (1) : (0);

	if(2 == sscanf(buf, "%d %d", &index,&value))
	{
		if(enable == 0){
			res = prox_set_adjust_para(cm36686_ps_adjust_para_real,index,value);
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
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, cm36686_show_als, NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, cm36686_show_ps, NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, cm36686_show_config,	cm36686_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, cm36686_show_alslv, cm36686_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, cm36686_show_alsval, cm36686_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, cm36686_show_trace,		cm36686_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, cm36686_show_status, NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, cm36686_show_send, cm36686_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, cm36686_show_recv, cm36686_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, cm36686_show_reg, NULL);
static DRIVER_ATTR(gain_als,      S_IWUSR | S_IRUGO, cm36686_show_gain_als,  cm36686_store_gain_als);
static DRIVER_ATTR(cali,      S_IWUSR | S_IRUGO, cm36686_show_cali,  cm36686_store_cali );
static DRIVER_ATTR(offset,      S_IWUSR | S_IRUGO, cm36686_show_offset,  cm36686_store_offset );
static DRIVER_ATTR(ps_adjust_para,      S_IWUSR | S_IRUGO, cm36686_show_ps_adjust_para,  cm36686_store_ps_adjust_para );
/*----------------------------------------------------------------------------*/
static struct driver_attribute *cm36686_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_reg,
    &driver_attr_gain_als,
    &driver_attr_cali,
    &driver_attr_offset,
    &driver_attr_ps_adjust_para,
};

/*----------------------------------------------------------------------------*/
static int cm36686_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(cm36686_attr_list)/sizeof(cm36686_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, cm36686_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", cm36686_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int cm36686_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(cm36686_attr_list)/sizeof(cm36686_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, cm36686_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------interrupt functions--------------------------------*/
static int intr_flag = 0;
/*----------------------------------------------------------------------------*/
#ifndef CUSTOM_KERNEL_SENSORHUB
static int cm36686_check_intr(struct i2c_client *client) 
{
	int res;
	u8 databuf[2];
	//u8 intr;
	
	databuf[0] = CM36686_REG_PS_DATA;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res<0)
	{
		APS_ERR("i2c_master_send function err res = %d\n",res);
		goto EXIT_ERR;
	}

	APS_LOG("cm36686_REG_PS_DATA value value_low = %x, value_reserve = %x\n",databuf[0],databuf[1]);
	
	databuf[0] = CM36686_REG_INT_FLAG;
	res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	if(res<0)
	{
		APS_ERR("i2c_master_send function err res = %d\n",res);
		goto EXIT_ERR;
	}
	
	APS_LOG("cm36686_REG_INT_FLAG value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
	
	if(databuf[1]&0x02)
	{
		intr_flag = 0;//for close
	}else if(databuf[1]&0x01)
	{
		intr_flag = 1;//for away
	}else{
		res = -1;
		APS_ERR("cm36686_check_intr fail databuf[1]&0x01: %d\n", res);
		goto EXIT_ERR;
	}
	
	return 0;
	EXIT_ERR:
	APS_ERR("cm36686_check_intr dev: %d\n", res);
	return res;
}
#endif //#ifndef CUSTOM_KERNEL_SENSORHUB
/*----------------------------------------------------------------------------*/
static void cm36686_eint_work(struct work_struct *work)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
    int res = 0;
    
    res = ps_report_interrupt_data(intr_flag);
    if(res != 0)
    {
        APS_ERR("cm36686_eint_work err: %d\n", res);
    }
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	struct cm36686_priv *obj = (struct cm36686_priv *)container_of(work, struct cm36686_priv, eint_work);
	int res = 0;

	APS_LOG("cm36686 interrupt top half time = %lld\n", int_top_time);

	res = cm36686_check_intr(obj->client);
	if(res != 0){
		goto EXIT_INTR_ERR;
	}else{
	       if(intr_flag!=-1){
		APS_LOG("cm36686 interrupt g_min_ps = %d \n",g_min_ps);
		APS_LOG("cm36686 interrupt value = %d  ps=%d  low=%d high=%d\n", intr_flag, obj->ps, atomic_read(&obj->ps_thd_val_low), atomic_read(&obj->ps_thd_val_high));
		res = ps_report_interrupt_data(intr_flag);	
	       }
	}
/*
#ifdef CUST_EINT_ALS_TYPE
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
*/
	enable_irq(obj->hw->eint_gpio);
	return;
	EXIT_INTR_ERR:
/*
#ifdef CUST_EINT_ALS_TYPE
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
*/
	enable_irq(obj->hw->eint_gpio);
	APS_ERR("cm36686_eint_work err: %d\n", res);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
}
/*----------------------------------------------------------------------------*/
#ifdef CUSTOM_KERNEL_SENSORHUB
static void cm36686_init_done_work(struct work_struct *work)
{
    struct cm36686_priv *obj = cm36686_obj;
    CM36686_CUST_DATA *p_cust_data;
    SCP_SENSOR_HUB_DATA data;
    int max_cust_data_size_per_packet;
    int i;
    uint sizeOfCustData;
    uint len;
    char *p = (char *)obj->hw;

    APS_FUN();

    p_cust_data = (CM36686_CUST_DATA *)data.set_cust_req.custData;
    sizeOfCustData = sizeof(*(obj->hw));
    max_cust_data_size_per_packet = sizeof(data.set_cust_req.custData) - offsetof(CM36686_SET_CUST, data);
    
    for (i=0;sizeOfCustData>0;i++)
    {
        data.set_cust_req.sensorType = ID_PROXIMITY;
        data.set_cust_req.action = SENSOR_HUB_SET_CUST;
        p_cust_data->setCust.action = CM36686_CUST_ACTION_SET_CUST;
        p_cust_data->setCust.part = i;
        
        if (sizeOfCustData > max_cust_data_size_per_packet)
        {
            len = max_cust_data_size_per_packet;
        }
        else
        {
            len = sizeOfCustData;
        }

        memcpy(p_cust_data->setCust.data, p, len);
        sizeOfCustData -= len;
        p += len;
        
        len += offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(CM36686_SET_CUST, data);
        SCP_sensorHub_req_send(&data, &len, 1);
    }

    data.set_cust_req.sensorType = ID_PROXIMITY;
    data.set_cust_req.action = SENSOR_HUB_SET_CUST;
    p_cust_data->setEintInfo.action = CM36686_CUST_ACTION_SET_EINT_INFO;
    p_cust_data->setEintInfo.gpio_mode = GPIO_ALS_EINT_PIN_M_EINT;
    p_cust_data->setEintInfo.gpio_pin = GPIO_ALS_EINT_PIN;
    p_cust_data->setEintInfo.eint_num = CUST_EINT_ALS_NUM;
    p_cust_data->setEintInfo.eint_is_deb_en = CUST_EINT_ALS_DEBOUNCE_EN;
    p_cust_data->setEintInfo.eint_type = CUST_EINT_ALS_TYPE;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(p_cust_data->setEintInfo);
    SCP_sensorHub_req_send(&data, &len, 1);

}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

static irqreturn_t cm36686_interrupt(int vec, void *info)
{
	struct cm36686_priv *obj = cm36686_obj;
	struct alsps_hw *hw = cm36686_get_cust_alsps_hw();

	if (!obj) {
		return IRQ_HANDLED;
	}

	disable_irq_nosync(hw->eint_gpio);

	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);

	return IRQ_HANDLED;
}
/*----------------------------------------------------------------------------
static void cm36686_eint_func(void)
{
	struct cm36686_priv *obj = cm36686_obj;
	if(!obj)
	{
		return;
	}	
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}
----------------------------------------------------------------------------*/
#ifdef CUSTOM_KERNEL_SENSORHUB
static int cm36686_irq_handler(void* data, uint len)
{
	struct cm36686_priv *obj = cm36686_obj;
    SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P)data;
    
	if(!obj)
	{
		return -1;
	}

    APS_ERR("len = %d, type = %d, action = %d, errCode = %d\n", len, rsp->rsp.sensorType, rsp->rsp.action, rsp->rsp.errCode);

    switch(rsp->rsp.action)
    {
        case SENSOR_HUB_NOTIFY:
            switch(rsp->notify_rsp.event)
            {
                case SCP_INIT_DONE:
                    schedule_work(&obj->init_done_work);
                    //schedule_delayed_work(&obj->init_done_work, HZ);
                    break;
                case SCP_NOTIFY:
                    if (CM36686_NOTIFY_PROXIMITY_CHANGE == rsp->notify_rsp.data[0])
                    {
                        intr_flag = rsp->notify_rsp.data[1];
                        cm36686_eint_func();
                    }
                    else
                    {
                        APS_ERR("Unknow notify");
                    }
                    break;
                default:
                    APS_ERR("Error sensor hub notify");
                    break;
            }
            break;
        default:
            APS_ERR("Error sensor hub action");
            break;
    }

    return 0;
}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
/*----------------------------------------------------------------------------*/
int cm36686_setup_eint(struct i2c_client *client)
{
#ifdef CUSTOM_KERNEL_SENSORHUB
    int err = 0;

    err = SCP_sensorHub_rsp_registration(ID_PROXIMITY, cm36686_irq_handler);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	int irq;
	int err = 0;

	struct alsps_hw *hw = cm36686_get_cust_alsps_hw();

	err = gpio_request(hw->eint_gpio, "alsps_irq");
	if (err)
	{
		printk("Unable to request GPIO.\n");
		return -1;
	}
	gpio_direction_input(hw->eint_gpio);
	irq = gpio_to_irq(hw->eint_gpio);
	if (irq < 0)
	{
		printk("Unable to request gpio irq. int pin = %d, irq = %d, err=%d\n", hw->eint_gpio, irq, err);
		gpio_free(hw->eint_gpio);
		return -1;
	}

	if (request_irq(irq, cm36686_interrupt, IRQF_TRIGGER_FALLING, "cm36686", (void *)client))
	{
		printk("%s Could not allocate cm36686_INT !\n", __func__);
		return -1;
	}

	irq_set_irq_wake(irq, 1);	

	//mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	//mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	//mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	//mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
/*
#ifdef CUST_EINT_ALS_TYPE
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, cm36686_eint_func, 0);
#else
	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, cm36686_eint_func, 0);
#endif

#ifdef CUST_EINT_ALS_TYPE
	mt_eint_mask(CUST_EINT_ALS_NUM);
#else
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
#endif
*/
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    return 0;
}
/*-------------------------------MISC device related------------------------------------------*/



/************************************************************/
static int cm36686_open(struct inode *inode, struct file *file)
{
	file->private_data = cm36686_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/
static int cm36686_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static int set_psensor_threshold(struct i2c_client *client)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB //not modified the SENSORHUB PS resolution to 12 bits
    SCP_SENSOR_HUB_DATA data;
    CM36686_CUST_DATA *pCustData;
    int len;
    int32_t ps_thd_val_low, ps_thd_val_high;

    ps_thd_val_low = atomic_read(&obj->ps_thd_val_low);
	ps_thd_val_high = atomic_read(&obj->ps_thd_val_high);


    //ps_cali would be add back in SCP side.
    ps_thd_val_low -= obj->ps_cali;
    ps_thd_val_high -= obj->ps_cali;

    data.set_cust_req.sensorType = ID_PROXIMITY;
    data.set_cust_req.action = SENSOR_HUB_SET_CUST;
    pCustData = (CM36686_CUST_DATA *)(&data.set_cust_req.custData);

    pCustData->setPSThreshold.action = CM36686_CUST_ACTION_SET_PS_THRESHODL;
    pCustData->setPSThreshold.threshold[0] = ps_thd_val_low;
    pCustData->setPSThreshold.threshold[1] = ps_thd_val_high;
    len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setPSThreshold);

    res = SCP_sensorHub_req_send(&data, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    u8 databuf[3];

	
	databuf[0] = CM36686_REG_PS_THDL;
	databuf[1] = atomic_read(&obj->ps_thd_val_low )& 0x00ff;
	databuf[2] = atomic_read(&obj->ps_thd_val_low) >> 8;//threshold value need to confirm
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		return -1;
	}
	
	databuf[0] = CM36686_REG_PS_THDH;
	databuf[1] = atomic_read(&obj->ps_thd_val_high )& 0x00ff;
	databuf[2] = atomic_read(&obj->ps_thd_val_high )>> 8;//threshold value need to confirm
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		return -1;
	}
	//APS_ERR("cm36686 set_psensor_threshold function high: %d, low:%d\n",atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	return 0;

}

/*----------------------------------------------------------------------------*/
static void cm36686_set_ps_threshold(struct i2c_client *client, int low_threshold, int high_threshold)
{
    struct cm36686_priv *obj = i2c_get_clientdata(client);
    atomic_set(&obj->ps_thd_val_high,  high_threshold);
    atomic_set(&obj->ps_thd_val_low,  low_threshold);//need to confirm
    //printk("enter 4");
    set_psensor_threshold(obj->client);
}

static long cm36686_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	
		void __user *arg64 = compat_ptr(arg);

		//printk(KERN_ERR "%s cmd = 0x%04x", __FUNCTION__, cmd);
	
		if(!file->f_op || !file->f_op->unlocked_ioctl)
		{
			printk(KERN_ERR "file->f_op OR file->f_op->unlocked_ioctl is null!\n");
			return -ENOTTY;
		}

	switch (cmd)
	{
		case COMPAT_ALSPS_SET_PS_MODE:
			ret = file->f_op->unlocked_ioctl(file, ALSPS_SET_PS_MODE, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_SET_PS_MODE is failed!\n");
				}
				break;
			break;

		case COMPAT_ALSPS_GET_PS_MODE:
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_MODE, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_GET_PS_MODE is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_PS_DATA:    
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_GET_PS_DATA is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_PS_RAW_DATA:    
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_PS_RAW_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_GET_PS_RAW_DATA is failed!\n");
				}
				break;         

		case COMPAT_ALSPS_SET_ALS_MODE:
			ret = file->f_op->unlocked_ioctl(file, ALSPS_SET_ALS_MODE, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_SET_ALS_MODE is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_ALS_MODE:
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_MODE, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_GET_ALS_MODE is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_ALS_DATA: 
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_GET_ALS_DATA is failed!\n");
				}
				break;

		case COMPAT_ALSPS_GET_ALS_RAW_DATA:    
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_ALS_RAW_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_GET_ALS_RAW_DATA is failed!\n");
				}
				break;

        	case COMPAT_ALSPS_SET_PS_THRESHOLD_OPPO: //0x21 lycan add for 
            ret = file->f_op->unlocked_ioctl(file, ALSPS_SET_PS_THRESHOLD_OPPO, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_SET_PS_THRESHOLD_OPPO is failed!\n");
				}
				break;
        	case COMPAT_ALSPS_GET_CUST_PS_ADJUST_PARA: //0x22 lycan add for
          	ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_CUST_PS_ADJUST_PARA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_GET_CUST_PS_ADJUST_PARA is failed!\n");
				}
				break;


		case COMPAT_ALSPS_GET_HIGHLIGHT_DATA: 
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_HIGHLIGHT_DATA, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_GET_HIGHLIGHT_DATA is failed!\n");
				}
				break;
		case COMPAT_ALSPS_GET_WAKEUP_STATUS:			
			ret = file->f_op->unlocked_ioctl(file, ALSPS_GET_WAKEUP_STATUS, (unsigned long)arg64);
				if(ret < 0)
				{
					printk(KERN_ERR "COMPAT_ALSPS_GET_WAKEUP_STATUS is failed!\n");
				}
				break;

		default:
			 printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			 return -ENOIOCTLCMD;
			 break;
	}

	return 0 ;  
}

/*----------------------------------------------------------------------------*/
static long cm36686_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
		struct i2c_client *client = (struct i2c_client*)file->private_data;
		struct cm36686_priv *obj = i2c_get_clientdata(client);  
		long err = 0;
		void __user *ptr = (void __user*) arg;
		int dat;
		uint32_t enable;
		int ps_result;
		int ps_cali;
		int als=0;
		int threshold[2];
		struct alsps_hw *hw = cm36686_get_cust_alsps_hw();
    		struct set_ps_thd_para set_ps_thd_para;
		int res;
		u8 databuf[2];
		//u8 test[6];

#ifdef CUSTOM_KERNEL_SENSORHUB
        SCP_SENSOR_HUB_DATA data;
        CM36686_CUST_DATA *pCustData;
        int len;

        data.set_cust_req.sensorType = ID_PROXIMITY;
        data.set_cust_req.action = SENSOR_HUB_SET_CUST;
        pCustData = (CM36686_CUST_DATA *)(&data.set_cust_req.custData);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
		
		switch (cmd)
		{
			case ALSPS_SET_PS_MODE:
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = cm36686_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %ld\n", err); 
						goto err_out;
					}
					
					//set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((err = cm36686_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %ld\n", err); 
						goto err_out;
					}
					//clear_bit(CMC_BIT_PS, &obj->enable);
				}
				break;
	
			case ALSPS_GET_PS_MODE:
				enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
				//hw = cm36686_get_cust_alsps_hw();
				//APS_LOG(" %s g_min_ps = %d \n", __FUNCTION__,g_min_ps);
				//cm36686_ps_adjust_para_real = (g_min_ps < 500) ? hw->p_ps_adjust_para:hw->p_ps_adjust_para_2;
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
	
			case ALSPS_GET_PS_DATA:    
				/*
					int res;
					u8 databuf[2];
					u8 test[6];
					//u8 intr
				*/
	
	                    /*databuf[0] = CM36686_REG_INT_FLAG;
	                    res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	                    if(res<0)
	                    {
		                     APS_ERR("cm36686 i2c_master_send function err res = %d\n",res);
	                    }
	
	                    APS_LOG("cm36686_REG_INT_FLAG value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
				test[0]=databuf[0];
				test[1]=databuf[1];

				 databuf[0] = CM36686_REG_PS_CONF1_2;
	                    res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	                    if(res<0)
	                    {
		                     APS_ERR("CM36686_REG_PS_CONF1_2 function err res = %d\n",res);
	                    }
	
	                    APS_LOG("CM36686_REG_PS_CONF1_2 value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
				test[2]=databuf[0];
				test[3]=databuf[1];*/

			      databuf[0] = CM36686_REG_PS_CONF3_MS;
	                    res = CM36686_i2c_master_operate(client, databuf, 0x201, I2C_FLAG_READ);
	                    if(res<0)
	                    {
		                     APS_ERR("CM36686_REG_PS_CONF3_MS function err res = %d\n",res);
	                    }
	
	                    //APS_LOG("CM36686_REG_PS_CONF3_MS value_low = %x, value_high = %x\n",databuf[0],databuf[1]);
				//test[4]=databuf[0];
				//test[5]=databuf[1];
	
				//if(test[0]==0&&test[1]==0&&test[2]==0&&test[3]==0&&test[4]==0&&test[5]==0)
				    if(databuf[0]==0&&databuf[1]==0)
				{
					cm36686_esd_handle(client, 1); // 1 means : intFlag err
				}
					
				if((err = cm36686_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				
				dat = cm36686_get_ps_value(obj, obj->ps);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}  				

				break;
	
			case ALSPS_GET_PS_RAW_DATA:    
				if((err = cm36686_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				
				dat = obj->ps;

				 if (obj->als < 8000){
                			if( (obj->ps != 0) &&((g_min_ps == 0) || (g_min_ps > obj->ps)))
                    				g_min_ps = obj->ps;
           			 }
				 
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}  
				break;			  
	
			case ALSPS_SET_ALS_MODE:
	
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = cm36686_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %ld\n", err); 
						goto err_out;
					}
					//set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = cm36686_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %ld\n", err); 
						goto err_out;
					}
					//clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				break;
	
			case ALSPS_GET_ALS_MODE:
				enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
	
			case ALSPS_GET_ALS_DATA: 

				//APS_ERR("cm36686 get als data enter\n"); 
				if((err = cm36686_read_als_calied(obj->client, &obj->als)))
				{
					goto err_out;
				}			
	
				dat =obj->als; //cm36686_get_als_value(obj, obj->als);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
	
			case ALSPS_GET_ALS_RAW_DATA:	
				//APS_ERR("cm36686 get als raw data enter\n"); 
				if((err = cm36686_read_als(obj->client, &als)))
				{
					goto err_out;
				}
	
				dat = als;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}

				//APS_LOG("cm36686 ALSPS_GET_ALS_RAW_DATA = %d \n",dat); 
				break;

			/*----------------------------------for factory mode test---------------------------------------*/
			case ALSPS_GET_PS_TEST_RESULT:
				if((err = cm36686_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}
				if(obj->ps > atomic_read(&obj->ps_thd_val_high))
					{
						ps_result = 0;
					}
				else	ps_result = 1;
				
				if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;

			case ALSPS_IOCTL_CLR_CALI:
				if(copy_from_user(&dat, ptr, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(dat == 0)
					obj->ps_cali = 0;

#ifdef CUSTOM_KERNEL_SENSORHUB
                pCustData->clearCali.action = CM36686_CUST_ACTION_CLR_CALI;
                len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->clearCali);
                
                err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

				break;

			case ALSPS_IOCTL_GET_CALI:
				ps_cali = obj->ps_cali ;
				if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_IOCTL_SET_CALI:
				if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
				{
					err = -EFAULT;
					goto err_out;
				}

				obj->ps_cali = ps_cali;

#ifdef CUSTOM_KERNEL_SENSORHUB
                pCustData->setCali.action = CM36686_CUST_ACTION_SET_CALI;
                pCustData->setCali.cali = ps_cali;
                len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(pCustData->setCali);
                
                err = SCP_sensorHub_req_send(&data, &len, 1);
#endif

				break;

			case ALSPS_SET_PS_THRESHOLD:
				if(copy_from_user(threshold, ptr, sizeof(threshold)))
				{
					err = -EFAULT;
					goto err_out;
				}
				//APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]); 
				atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
				atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

				set_psensor_threshold(obj->client);
				
				break;
				
			case ALSPS_GET_PS_THRESHOLD_HIGH:
				threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
				//APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]); 
				if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
				
			case ALSPS_GET_PS_THRESHOLD_LOW:
				threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
				//APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]); 
				if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
				{
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
	            	switch (set_ps_thd_para.algo_state) {
	                case PS_ADJUST_TREND_STATE : 
	                    APS_LOG("set_ps_threshold ps average:%d state: TREND\n", set_ps_thd_para.ps_average);
	                    break;
	                case PS_ADJUST_NOTREND_STATE :
	                    APS_LOG("set_ps_threshold ps average:%d state: NOTREND\n", set_ps_thd_para.ps_average);
	                    break;
	                case PS_ADJUST_HIGHLIGHT_STATE :
	                    APS_LOG("set_ps_threshold ps average:%d state: HIGHLIGHT\n", set_ps_thd_para.ps_average);
	                    break;
	                case PS_ADJUST_AVOID_DIRTY_STATE : 
	                    APS_LOG("set_ps_threshold ps average:%d state: AVOID_DIRTY\n", set_ps_thd_para.ps_average);
	                    break;
	                default:
	                    APS_LOG("set_ps_threshold ps average:%d state: impossible\n", set_ps_thd_para.ps_average);
	                    break;
	            }
			#endif//VENDOR_EDIT
	            cm36686_set_ps_threshold(obj->client,
	                    set_ps_thd_para.low_threshold, 
	                    set_ps_thd_para.high_threshold);
	            break;

	        	case ALSPS_GET_CUST_PS_ADJUST_PARA: //0x22 lycan add for
	           	 //APS_LOG(" %s()->case ALSPS_GET_CUST_PS_ADJUST_PARA  dirty_low = %d \n", __FUNCTION__,cm36686_ps_adjust_para_real->dirty_adjust_low_thd);
	            	hw = cm36686_get_cust_alsps_hw();
					if(copy_to_user(ptr, cm36686_ps_adjust_para_real, sizeof( struct ps_adjust_para)))
	           	 {
	               	 err = -EFAULT;
	                	goto err_out;
	            	}              
	            break;

			case ALSPS_GET_HIGHLIGHT_DATA: 
			if(obj->als<8000)
				dat=0;
			else 
				dat=1;
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

				
				default:
					APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
					err = -ENOIOCTLCMD;
					break;
			}
	
		err_out:
		return err;    
	}
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations cm36686_fops = {
	.owner = THIS_MODULE,
	.open = cm36686_open,
	.release = cm36686_release,
	.unlocked_ioctl = cm36686_unlocked_ioctl,

#ifdef CONFIG_COMPAT
	.compat_ioctl = cm36686_compat_ioctl,
#endif
};

static struct miscdevice cm36686_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &cm36686_fops,
};

/*--------------------------------------------------------------------------------------*/
static void cm36686_early_suspend(struct early_suspend *h)
{
		struct cm36686_priv *obj = container_of(h, struct cm36686_priv, early_drv);	
		APS_FUN();	  
	
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return;
		}
		
		atomic_set(&obj->als_suspend, 1);
		/*
		if((err = cm36686_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
		*/
}

static void cm36686_late_resume(struct early_suspend *h) 
{
		struct cm36686_priv *obj = container_of(h, struct cm36686_priv, early_drv);		  
		hwm_sensor_data sensor_data;
		memset(&sensor_data, 0, sizeof(sensor_data));
		APS_FUN();
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return;
		}
	
		atomic_set(&obj->als_suspend, 0);
		/*
		if(test_bit(CMC_BIT_ALS, &obj->enable))
		{
			if((err = cm36686_enable_als(obj->client, 1)))
			{
				APS_ERR("enable als fail: %d\n", err);		  
	
			}
		}
		*/
}
/*--------------------------------------------------------------------------------*/
static int cm36686_init_client(struct i2c_client *client)
{
	struct cm36686_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];    
	int res = 0;
	APS_FUN();
	databuf[0] = CM36686_REG_ALS_CONF;
	if(1 == obj->hw->polling_mode_als){
		databuf[1] = 0x01;
	}else{
		databuf[1] = 0x03;
	}
	databuf[2] = 0x00;
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	APS_LOG("cm36686 ps CM36686_REG_ALS_CONF command!\n");
	
	databuf[0] = CM36686_REG_PS_CONF1_2;
	databuf[1] = 0xa1;//0xad;//0x2c;//0x37;
	if(1 == obj->hw->polling_mode_ps)
	databuf[2] = 0x00;
	else
	databuf[2] = 0x03;
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	APS_LOG("cm36686 ps CM36686_REG_PS_CONF1_2 command!\n");
	
	databuf[0] = CM36686_REG_PS_CONF3_MS;
	databuf[1] = 0xa0;//0x10
	if(1 == obj->hw->polling_mode_ps)
	databuf[2] = 0x45;//LED_I=160mA
	else
	databuf[2] =0x03;//0x01;// 0x05;//LED_I=75mA
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	APS_LOG("cm36686 ps CM36686_REG_PS_CONF3_MS command!\n");
	/*
	databuf[0] = CM36686_REG_PS_CANC;//value need to confirm
	databuf[1] = 0x00;//0x00;
	databuf[2] = 0x00;
	res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		goto EXIT_ERR;
	}
	
	APS_LOG("cm36686 ps CM36686_REG_PS_CANC command!\n");*/
	
	if(0 == obj->hw->polling_mode_als){
			databuf[0] = CM36686_REG_ALS_THDH;
			databuf[1] = 0x00;
			databuf[2] = atomic_read(&obj->als_thd_val_high);
			res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				APS_ERR("i2c_master_send function err\n");
				goto EXIT_ERR;
			}
			databuf[0] = CM36686_REG_ALS_THDL;
			databuf[1] = 0x00;
			databuf[2] = atomic_read(&obj->als_thd_val_low);//threshold value need to confirm
			res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				APS_ERR("i2c_master_send function err\n");
				goto EXIT_ERR;
			}
		}
	if(0 == obj->hw->polling_mode_ps){
			databuf[0] = CM36686_REG_PS_THDL;
			databuf[1] = atomic_read(&obj->ps_thd_val_low )& 0x00ff;
			databuf[2] = atomic_read(&obj->ps_thd_val_low) >> 8;//threshold value need to confirm
			res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);

			databuf[0] = CM36686_REG_PS_THDH;
			databuf[1] = atomic_read(&obj->ps_thd_val_high )& 0x00ff;
			databuf[2] = atomic_read(&obj->ps_thd_val_high) >> 8;//threshold value need to confirm
			res = CM36686_i2c_master_operate(client, databuf, 0x3, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				APS_ERR("i2c_master_send function err\n");
				goto EXIT_ERR;
			}
		}
	res = cm36686_setup_eint(client);
	if(res!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	
	return CM36686_SUCCESS;
	
	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}
/*--------------------------------------------------------------------------------*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int als_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("cm36686_obj als enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_LIGHT;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return -1;
	}
	res=	cm36686_enable_als(cm36686_obj->client, en);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_get_data(int* value, int* status)
{
	int err = 0;
	int res;
	u8 databuf[2];
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#else
    struct cm36686_priv *obj = NULL;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_LIGHT;
    req.get_data_req.action = SENSOR_HUB_GET_DATA;
    len = sizeof(req.get_data_req);
    err = SCP_sensorHub_req_send(&req, &len, 1);
    if (err)
    {
        APS_ERR("SCP_sensorHub_req_send fail!\n");
    }
    else
    {
        *value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    if(atomic_read(&cm36686_obj->trace) & CMC_TRC_PS_DATA)
	{
        APS_LOG("value = %d\n", *value);
        //show data
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return -1;
	}
	obj = cm36686_obj;
	
	if (als_first_read) 
	{
		als_first_read = false;
		msleep(40);
	}

	databuf[0] = CM36686_REG_PS_CONF3_MS;
	res = CM36686_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
	if(res<0)
	{
		APS_ERR("CM36686_REG_PS_CONF3_MS function err res = %d\n",res);
	}

	//APS_LOG("CM36686_REG_PS_CONF3_MS value_low = %x, value_high = %x\n",databuf[0],databuf[1]);

	if(databuf[0]==0&&databuf[1]==0){
		cm36686_esd_handle(obj->client, 1);
	}

	if((err = cm36686_read_als_calied(obj->client, &obj->als)))
	{
		err = -1;
	}else{
		/*databuf[0] = CM36686_REG_ALS_CONF;
              res = CM36686_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
              if(res<0)
              {
                   APS_ERR("cm36686 i2c_master_send function err res = %d\n",res);
              }
              APS_LOG("CM36686_REG_ALS_CONF value value_low = %x, value_high = %x\n",databuf[0],databuf[1]);*/
	#if defined(CAPELLA_DEBOUNCE_CANCELLATION)
			if(curr_lux == DEBOUNCE_LUX_LEVEL_INIT )
			{
				curr_lux = obj->als;
				if( curr_lux < DEBOUNCE_LUX_LEVEL_1 )
				{
					curr_thd = (curr_lux * DEBOUNCE_LEVEL_1_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
				else if( curr_lux < DEBOUNCE_LUX_LEVEL_2 )
				{
					curr_thd = (curr_lux * DEBOUNCE_LEVEL_2_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
				else if( curr_lux < DEBOUNCE_LUX_LEVEL_3 )
				{
					curr_thd = (curr_lux * DEBOUNCE_LEVEL_3_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
				else if( curr_lux < DEBOUNCE_LUX_LEVEL_4 )
				{
					curr_thd = (curr_lux * DEBOUNCE_LEVEL_4_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
				else
				{
					curr_thd = (curr_lux * DEBOUNCE_LEVEL_5_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
			} 
			else if(obj->als > (curr_lux + curr_thd) || obj->als < (curr_lux - curr_thd))
			{
				curr_lux = obj->als;
				if( curr_lux < DEBOUNCE_LUX_LEVEL_1 )
				{
					curr_thd = (curr_lux * DEBOUNCE_LEVEL_1_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
				else if( curr_lux < DEBOUNCE_LUX_LEVEL_2 )
				{
					curr_thd = (curr_lux * DEBOUNCE_LEVEL_2_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
				else if( curr_lux < DEBOUNCE_LUX_LEVEL_3 )
				{
					curr_thd = (curr_lux * DEBOUNCE_LEVEL_3_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
				else if( curr_lux < DEBOUNCE_LUX_LEVEL_4 )
				{
					curr_thd = (curr_lux * DEBOUNCE_LEVEL_4_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
				else
				{
				   curr_thd = (curr_lux * DEBOUNCE_LEVEL_5_THD_NOMINATOR) / DEBOUNCE_THD_DENOMINATOR;
				}
			}

			*value = curr_lux;
	#else//CAPELLA_DEBOUNCE_CANCELLATION
		*value = obj->als;//cm36686_get_als_value(obj, obj->als);
	#endif//CAPELLA_DEBOUNCE_CANCELLATION
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
		//APS_LOG("als_get_data = %d\n", *value);
	}
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	return err;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int ps_enable_nodata(int en)
{
	int res = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

    APS_LOG("cm36686_obj ps enable value = %d\n", en);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.activate_req.sensorType = ID_PROXIMITY;
    req.activate_req.action = SENSOR_HUB_ACTIVATE;
    req.activate_req.enable = en;
    len = sizeof(req.activate_req);
    res = SCP_sensorHub_req_send(&req, &len, 1);
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
	if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return -1;
	}
	res=	cm36686_enable_ps(cm36686_obj->client, en);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	if(res){
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;

}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.get_data_req.sensorType = ID_PROXIMITY;
    req.get_data_req.action = SENSOR_HUB_GET_DATA;
    len = sizeof(req.get_data_req);
    err = SCP_sensorHub_req_send(&req, &len, 1);
    if (err)
    {
        APS_ERR("SCP_sensorHub_req_send fail!\n");
    }
    else
    {
        *value = req.get_data_rsp.int16_Data[0];
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

    if(atomic_read(&cm36686_obj->trace) & CMC_TRC_PS_DATA)
	{
        APS_LOG("value = %d\n", *value);
        //show data
	}
#else //#ifdef CUSTOM_KERNEL_SENSORHUB
    if(!cm36686_obj)
	{
		APS_ERR("cm36686_obj is null!!\n");
		return -1;
	}
    
    if((err = cm36686_read_ps(cm36686_obj->client, &cm36686_obj->ps)))
    {
        err = -1;;
    }
    else
    {
        *value = cm36686_get_ps_value(cm36686_obj, cm36686_obj->ps);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB
    
	return 0;
}


/*-----------------------------------i2c operations----------------------------------*/
static int cm36686_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cm36686_priv *obj;
	int i = 0;
	int err = 0;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};

	#ifdef VENDOR_EDIT 	
	//ziqing.guo@BasicDrv.Sensor,2015/04/12, add for PSD
	init_waitqueue_head(&enable_ps);
	#endif

	APS_FUN();
	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(*obj));
	cm36686_obj = obj;
	
	obj->hw = cm36686_get_cust_alsps_hw();//get custom file data struct
	
	INIT_WORK(&obj->eint_work, cm36686_eint_work);
#ifdef CUSTOM_KERNEL_SENSORHUB
    INIT_WORK(&obj->init_done_work, cm36686_init_done_work);
#endif //#ifdef CUSTOM_KERNEL_SENSORHUB

	obj->client = client;
	i2c_set_clientdata(client, obj);
	
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
	atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);
	
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	/*-----------------------------value need to be confirmed-----------------------------------------*/
	
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	//set_bit(CMC_BIT_ALS, &obj->enable);
	//set_bit(CMC_BIT_PS, &obj->enable);

	cm36686_i2c_client = client;

	for (i = 0; i < 3; i++)
	{
		if((err = cm36686_init_client(client)))
		{
			if (i >= 2)
			{
				APS_ERR("After 3 times init, still fail\n");
				goto exit_init_failed;
			}
		}
		else
			break;
	}
	APS_LOG("cm36686_init_client() OK!\n");

	if((err = misc_register(&cm36686_device)))
	{
		APS_ERR("cm36686_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	APS_LOG("cm36686_device misc_register OK!\n");

	/*------------------------cm36686 attribute file for debug--------------------------------------*/
	if((err = cm36686_create_attr(&(cm36686_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------cm36686 attribute file for debug--------------------------------------*/
	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = obj->hw->is_batch_supported_als;
#else
    als_ctl.is_support_batch = false;
#endif
	
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	
	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = true;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;
#else
    ps_ctl.is_support_batch = false;
#endif
	
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);	
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register light batch support err = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}
	
	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register proximity batch support err = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend  = cm36686_early_suspend,
	obj->early_drv.resume   = cm36686_late_resume,    
	register_early_suspend(&obj->early_drv);
	#endif

	obj->gain_als = -1;
	cm36686_init_flag =0;
	alsps_dev = ALSPS_CM_36686;
	APS_LOG("%s: OK  gpio:%d \n", __func__, mt_get_gpio_in(GPIO_ALS_EINT_PIN));
	#ifdef VENDOR_EDIT
	//ye.zhang@BSP.Sensor, 2016-01-26, add for chip information
	register_device_proc("Sensor_alsps", CM36686_DEV_NAME, "CAPELLA");
	#endif//VENDOR_EDIT
	
	return 0;

	exit_create_attr_failed:
	exit_sensor_obj_attach_fail:
	exit_misc_device_register_failed:
		misc_deregister(&cm36686_device);
	exit_init_failed:
		kfree(obj);
	exit:
	cm36686_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	cm36686_init_flag =-1;
	return err;
}

static int cm36686_i2c_remove(struct i2c_client *client)
{
	int err;	
	/*------------------------cm36686 attribute file for debug--------------------------------------*/	
	if((err = cm36686_delete_attr(&(cm36686_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("cm36686_delete_attr fail: %d\n", err);
	} 
	/*----------------------------------------------------------------------------------------*/
	
	if((err = misc_deregister(&cm36686_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
		
	cm36686_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;

}

static int cm36686_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, CM36686_DEV_NAME);
	return 0;

}

static int cm36686_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}

static int cm36686_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int cm36686_remove(void)
{
	//APS_FUN(); 
	//cm36686_power(hw, 0);//*****************  
	
	i2c_del_driver(&cm36686_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/

static int  cm36686_local_init(void)
{
	//printk("fwq loccal init+++\n");

	//cm36686_power(hw, 1);
	if(i2c_add_driver(&cm36686_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}
	if(-1 == cm36686_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}


/*----------------------------------------------------------------------------*/
static int __init cm36686_init(void)
{
	//APS_FUN();
	struct alsps_hw *hw = cm36686_get_cust_alsps_hw();
	cm36686_ps_adjust_para_real=hw->p_ps_adjust_para;
	APS_LOG("%s: i2c_number=%d, i2c_addr: 0x%x\n", __func__, hw->i2c_num, hw->i2c_addr[0]);
	i2c_register_board_info(hw->i2c_num, &i2c_cm36686, 1);
	alsps_driver_add(&cm36686_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit cm36686_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(cm36686_init);
module_exit(cm36686_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yucong xiong");
MODULE_DESCRIPTION("cm36686 driver");
MODULE_LICENSE("GPL");

