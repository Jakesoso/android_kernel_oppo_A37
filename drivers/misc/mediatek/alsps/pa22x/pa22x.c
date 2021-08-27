/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.  
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/stk3x1x.c - stk3x1x ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
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

#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/version.h> 
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/hwmsensor.h>

#include <cust_alsps.h>
#include "pa22x.h"
#include "alsps.h"

//#define POWER_NONE_MACRO MT65XX_POWER_NONE



/******************************************************************************
 * configuration
*******************************************************************************/
/**Global Variable**/
static int prevObj 	= 1;
static int intr_flag	= 1;
//static int bCal_flag	= 0;

//static int near_loop = 0;
//static int far_loop = 0;

static int als_enable	= 0;
static int ps_enable	= 0;
/*Have to set initial value of far_ps_min to be greater than factory calibration*/
static int far_ps_min = PA22_PS_OFFSET_MAX;
//static int ps_min = 255;
#define OIL_EFFECT 30
static int saturation_flag = 0; 
#define ps_ary_size 5
static const int ps_steady = ps_ary_size + 4;
static int ps_seq_far[ps_ary_size];
static int ps_seq_oil[ps_ary_size];
static int ps_seq_near[ps_ary_size];
static int oil_occurred = 0;

#define saturation_delay 100
#define sequence_dealy 15

/*Switching between window type and hysteresis type*/
enum {
	window,
	hysteresis,
};
static int int_type = window;

/*----------------------------------------------------------------------------*/
#ifdef DEVICE_PA224
#define PA22X_DEV_NAME		"pa224"
#else
#define PA22X_DEV_NAME		"pa22a"
#endif
#define PA22_DRIVER_VERSION_C		"2.0.1"
/*----------------------------------------------------------------------------*/
#define PA22X_DEBUG
#if defined(PA22X_DEBUG)
#ifdef DEVICE_PA224
#define APS_TAG		"[TXC/PS] "
#else
#define APS_TAG		"[ALS/PS] "
#endif
#define APS_FUN(f)			printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)	printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)	printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)	printk(KERN_INFO APS_TAG fmt, ##args)    
#else
#define APS_FUN(f)
#define APS_ERR(fmt, args...)
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)
#endif

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

#define TXC_ABS(x) (x) >= 0 ? (x):(x)*(-1)
#define ARRAY_SUM(arr, sum) \
do { \
	int i = 0; \
	int size = ARRAY_SIZE(arr); \
	for (i=0; i<size; i++) \
		sum += arr[i]; \
} while (0)

#define ARRAY_ABS_SUM(arr, sum) \
do { \
	int i = 0; \
	int size = ARRAY_SIZE(arr); \
	for (i=0; i<size; i++) \
		sum += TXC_ABS(arr[i]); \
} while (0)

#define IS_CLOSE(arr, close) \
do { \
	int i = 0; \
	int size = ARRAY_SIZE(arr); \
	close = 1; \
	while (i < size && close == 1) { \
		if (arr[i] >= 0) \
			i++; \
		else \
			close = 0; \
	} \
}while (0)

#define IS_AWAY(arr, away) \
do { \
	int i = 0; \
	int size = ARRAY_SIZE(arr); \
	away = 1; \
	while (i < size && away == 1) { \
		if ( arr[i] <= 0) \
			i++; \
		else \
			away = 0; \
	}  \
}while (0)


/******************************************************************************
 * extern functions
*******************************************************************************/


/*----------------------------------------------------------------------------*/
static int pa22x_init_client(struct i2c_client *client);		
static int pa22x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int pa22x_i2c_remove(struct i2c_client *client);
static int pa22x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int pa22x_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int pa22x_i2c_resume(struct i2c_client *client);

static struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

static int	pa22x_init_flag = -1;
static int  pa22x_local_init(void);
static int pa22x_local_uninit(void);
static struct alsps_init_info pa22x_init_info = {		
	.name = "pa22x",		
	.init = pa22x_local_init,		
	.uninit = pa22x_local_uninit,
};



/*----------------------------------------------------------------------------*/
static const struct i2c_device_id pa22x_i2c_id[] = {{PA22X_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_pa22x={ I2C_BOARD_INFO(PA22X_DEV_NAME,PA22_I2C_ADDRESS)};
/*----------------------------------------------------------------------------*/
struct pa22x_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;

	/* misc */
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
	atomic_t    init_done;
	struct      device_node *irq_node;
	int         irq;
	/* data */
	u16			als;
	u8 			ps;
	u8			_align;
	u16			als_level_num;
	u16			als_value_num;
	u32			als_level[C_CUST_ALS_LEVEL-1];
	u32			als_value[C_CUST_ALS_LEVEL];

	u16		als_caliADC_IR[4];
	/* Mutex */
	struct mutex	update_lock;

	/* PS Calibration */
	u8 		crosstalk; 
	u8 		crosstalk_base; 

	/* threshold */
	u8		ps_to_max;
	u8		ps_thrd_low; 
	u8		ps_thrd_high; 

	atomic_t	als_cmd_val;		/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val;			/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_high;	/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;
	ulong		enable;				/*enable mask*/
	ulong		pending_intr;		/*pending interrupt*/
	
	/* early suspend */
	#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend	early_drv;
	#endif     
};
/*----------------------------------------------------------------------------*/
static int pa22x_get_als_value(struct pa22x_priv *obj, u16 als);

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps3"},	
	{},
};
#endif

static struct i2c_driver pa22x_i2c_driver = {
.driver = {
	.name = PA22X_DEV_NAME,
	/*
#ifdef CONFIG_OF	
	.of_match_table = alsps_of_match,
#endif*/
	},
	.probe	= pa22x_i2c_probe,
	.remove	= pa22x_i2c_remove,
	.detect	= pa22x_i2c_detect,
	.suspend	= pa22x_i2c_suspend,
	.resume	= pa22x_i2c_resume,
	.id_table	= pa22x_i2c_id,
};


/*----------------------------------------------------------------------------*/

static struct i2c_client *pa22x_i2c_client = NULL;
static struct pa22x_priv *g_pa22x_ptr = NULL; 
static struct pa22x_priv *pa22x_obj = NULL;
struct platform_device *alspsPltFmDev;
/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS	= 1,
	CMC_BIT_PS	= 2,
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
// I2C Read
#if 0
static int i2c_read_reg(struct i2c_client *client,u8 reg,u8 *data)
{
  u8 reg_value[1];
  u8 databuf[2]; 
	int res = 0;
	databuf[0]= reg;
	res = i2c_master_send(client,databuf,0x1);
	if(res <= 0)
	{
		APS_ERR("i2c_master_send function err\n");
		return res;
	}
	res = i2c_master_recv(client,reg_value,0x1);
	if(res <= 0)
	{
		APS_ERR("i2c_master_recv function err\n");
		return res;
	}
	return reg_value[0];
}
// I2C Write
static int i2c_write_reg(struct i2c_client *client,u8 reg,u8 value)
{
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = reg;   
	databuf[1] = value;
	res = i2c_master_send(client,databuf,0x2);

	if (res < 0){
		return res;
		APS_ERR("i2c_master_send function err\n");
	}
		return 0;
}
#endif
/*----------------------------------------------------------------------------*/
void pa22_swap(u8 *x, u8 *y)
{
        u8 temp = *x;
        *x = *y;
        *y = temp;
}
static int pa22x_run_calibration(struct i2c_client *client)
{
	struct pa22x_priv *data = i2c_get_clientdata(client);
	int i, j;	
	int ret;
	u16 sum_of_pdata = 0;
	u8 temp_pdata[20],buftemp[2],cfg0data=0,cfg2data=0;
	unsigned int ArySize = 20;
	unsigned int cal_check_flag = 0;	
	
	APS_LOG("%s: START proximity sensor calibration\n", __func__);

RECALIBRATION:
	sum_of_pdata = 0;

	mutex_lock(&data->update_lock);
	ret = hwmsen_read_byte(client, REG_CFG0, &cfg0data);
	ret = hwmsen_read_byte(client, REG_CFG2, &cfg2data);
	
	/*PS On*/
	ret = hwmsen_write_byte(client, REG_CFG0, cfg0data | 0x02); 

	/*Set to offset mode & disable interrupt from ps*/
	ret = hwmsen_write_byte(client, REG_CFG2, cfg2data & 0x33); 

	/*Set crosstalk = 0*/
	ret = hwmsen_write_byte(client, REG_PS_OFFSET, 0x00); 	
	

	for(i = 0; i < 20; i++)
	{
		mdelay(50);
		ret = hwmsen_read_byte(client,REG_PS_DATA,temp_pdata+i);
		APS_LOG("temp_data = %d\n", temp_pdata[i]);	
	}	
	mutex_unlock(&data->update_lock);
	
	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
		for (j = i+1; j < ArySize; j++)
			if (temp_pdata[i] > temp_pdata[j])
				pa22_swap(temp_pdata + i, temp_pdata + j);	
	
	/* calculate the cross-talk using central 10 data */
	for (i = 5; i < 15; i++) 
	{
		APS_LOG("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	data->crosstalk = sum_of_pdata/10;
    	APS_LOG("%s: sum_of_pdata = %d   cross_talk = %d\n",
                        __func__, sum_of_pdata, data->crosstalk);
	

	
	/* Restore CFG2 (Normal mode) and Measure base x-talk */
	mutex_lock(&data->update_lock);
	ret = hwmsen_write_byte(client, REG_CFG0, cfg0data);
	//ret = hwmsen_write_byte(client, REG_CFG2, cfg2data | 0xC0);
	ret = hwmsen_write_byte(client, REG_CFG2, cfg2data);	
	mutex_unlock(&data->update_lock);
 	
	if (data->crosstalk > PA22_PS_OFFSET_MAX)
	{
		APS_LOG("%s: invalid calibrated data\n", __func__);

		if(cal_check_flag == 0)
		{
			APS_LOG("%s: RECALIBRATION start\n", __func__);
			cal_check_flag = 1;
			goto RECALIBRATION;
		}
		else
		{
			APS_LOG("%s: CALIBRATION FAIL -> "
                               "cross_talk is set to DEFAULT\n", __func__);
			data->crosstalk = PA22_PS_OFFSET_DEFAULT;
			//ret = hwmsen_write_byte(client, REG_PS_OFFSET, data->crosstalk);
			return -EINVAL;
         }
	}	

	APS_LOG("%s: FINISH proximity sensor calibration\n", __func__);
	/*Write x-talk info to file*/  
	buftemp[0]=data->crosstalk;
	buftemp[1]=data->crosstalk_base;

	return data->crosstalk;
}

/**********************************************************************************************/
static void pa22x_power(struct alsps_hw *hw, unsigned int on) 
{
	#if 0
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
			if(!hwPowerOn(hw->power_id, hw->power_vol, "PA22X")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "PA22X")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
	#endif
}

/********************************************************************/
int pa22x_enable_ps(struct i2c_client *client, int enable)
{
	struct pa22x_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 reg0data = 0;
	u8 reg2data = 0;
	u8 sendvalue = 0;
	hwm_sensor_data sensor_data;
	//struct file  *fop;

	mutex_lock(&obj->update_lock);
	res = hwmsen_read_byte(client, REG_CFG0, &reg0data); 
	mutex_unlock(&obj->update_lock);	

	if(res<0)
	{
		APS_ERR("i2c_read function err\n");
		return -1;
	}

	

	if(enable == 1)
	{
		APS_LOG("pa22x enable ps sensor\n");
		saturation_flag = 0;
		oil_occurred = 0;
		int_type = window;
	
		/**** SET INTERRUPT FLAG AS FAR ****/
		if(obj->hw->polling_mode_ps == 0) //0:interrupt mode
		{
			//Set initial state as far, if you use input system, remember to report
			intr_flag = 1;

			//reset far_ps_min
			far_ps_min = PA22_PS_OFFSET_MAX;
			
			//int i = 0;
			/*set initial value = 255 to prevent */
			/*for (i=0;i<ps_ary_size;i++) {
				ps_seq_far[i] = 255;
				ps_seq_oil[i] = 255;
			}*/
			
			mutex_lock(&obj->update_lock);
			//Prevent interrupt
			hwmsen_write_byte(client, REG_PS_TL, 0);
			hwmsen_write_byte(client, REG_PS_TH, 0xFF);
			//Clear interrupt
			hwmsen_read_byte(client,REG_CFG2,&reg2data);		
			reg2data=reg2data & 0xFD ; 
			hwmsen_write_byte(client,REG_CFG2,reg2data);
			//Msut reset sleep time 
			hwmsen_write_byte(client,REG_CFG3,
					(PA22_INT_TYPE << 6)| (PA22_PS_PERIOD << 3)| (PA22_ALS_PERIOD));			
			//PS On
			hwmsen_write_byte(client,REG_CFG0, reg0data | 0x02); 
			mutex_unlock(&obj->update_lock);
			//Refresh data
			msleep(50);

			obj->ps_thrd_high = 80;
			obj->ps_thrd_low = 79;
			APS_LOG("thrd high low : %d %d\n", obj->ps_thrd_high, obj->ps_thrd_low);
		}
		//Restore thresholds & CFG0 data
		mutex_lock(&obj->update_lock);
		hwmsen_write_byte(client,REG_CFG0, reg0data);
		hwmsen_write_byte(client, REG_PS_TL, obj->ps_thrd_low);
		hwmsen_write_byte(client, REG_PS_TH, obj->ps_thrd_high);
		mutex_unlock(&obj->update_lock);
		sensor_data.values[0] = intr_flag;
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;				
		if((res = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
			APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", res);
		}
		/***********************************/
		APS_LOG("CFG0 Status: %d\n",reg0data);
		//sendvalue = regdata & 0xFD;
		/* PS On */
		sendvalue = reg0data | 0x02;

		mutex_lock(&obj->update_lock);
		//Prevent interrupt
		hwmsen_write_byte(client, REG_PS_TL, 0);
		hwmsen_write_byte(client, REG_PS_TH, 0xFF);
		res=hwmsen_write_byte(client,REG_CFG0,sendvalue); 
		mutex_unlock(&obj->update_lock);

		//Refresh data
		mdelay(50);

		mutex_lock(&obj->update_lock);
		hwmsen_write_byte(client, REG_PS_TL, obj->ps_thrd_low);
		hwmsen_write_byte(client, REG_PS_TH, obj->ps_thrd_high);
		mutex_unlock(&obj->update_lock);		

		if(res<0)
		{
			APS_ERR("i2c_write function err\n");
			return res;
		}

		ps_enable = 1;
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
	}
	else
	{
		APS_LOG("pa22x disaple ps sensor\n");
		
		APS_LOG("CFG0 Status: %d\n",reg0data);
		/* PS Off */
		sendvalue = reg0data & 0xFD; 

		mutex_lock(&obj->update_lock);				
		res = hwmsen_write_byte(client, REG_CFG0, sendvalue); 
		mutex_unlock(&obj->update_lock);

		if(res<0)
		{
			APS_ERR("i2c_write function err\n");
			return res;
		}	  	
		ps_enable = 0;
		atomic_set(&obj->ps_deb_on, 0);
	}

	return 0;
}


/********************************************************************/
int pa22x_enable_als(struct i2c_client *client, int enable)
{
	struct pa22x_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 regdata = 0;
	u8 sendvalue = 0;
	// hwm_sensor_data sensor_data_als;
	
	if(enable == 1)
	{
		APS_LOG("pa22x enable als sensor\n");

		mutex_lock(&obj->update_lock);
		res=hwmsen_read_byte(client,REG_CFG0,&regdata);	
		/* If ALS using interrupt, make first interrupt */ 
		if(obj->hw->polling_mode_als == 0)
		{
			hwmsen_write_byte(client, REG_ALS_TH_MSB, 0); 
			hwmsen_write_byte(client, REG_ALS_TH_LSB, 1); 
			hwmsen_write_byte(client, REG_ALS_TL_MSB, 0); 
			hwmsen_write_byte(client, REG_ALS_TL_LSB, 0); 
		}	 
		mutex_unlock(&obj->update_lock);

		if(res<0)
		{
			APS_ERR("i2c_read function err\n");
			return -1;
		}
		else
		{
			APS_LOG("CFG0 Status: %d\n",regdata);
			/* Clear ALS enable bit */
			sendvalue=regdata & 0xFE; 
			/* Set ALS enable */
			sendvalue=sendvalue | 0x01; 
			mutex_lock(&obj->update_lock);
			res=hwmsen_write_byte(client,REG_CFG0,sendvalue); 
			mutex_unlock(&obj->update_lock);

			if(res<0)
			{
				APS_ERR("i2c_write function err\n");
				return res;
			}	  	
		}
		als_enable = 1;
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		
	}
	else
	{	
		APS_LOG("pa22x disaple als sensor\n");

		mutex_lock(&obj->update_lock);
		res=hwmsen_read_byte(client,REG_CFG0,&regdata); 
		mutex_unlock(&obj->update_lock);

		if(res<0)
		{
			APS_ERR("i2c_read function err\n");
			return res;				
		}
		else
		{
			APS_LOG("CFG0 Status: %d\n",regdata);
			/* Clear ALS enable bit */
			sendvalue=regdata & 0xFE; 
				
			mutex_lock(&obj->update_lock);
			res=hwmsen_write_byte(client,REG_CFG0,sendvalue); 
			mutex_unlock(&obj->update_lock);

			if(res<0)
			{
				APS_ERR("i2c_write function err\n");
				return res;
		    }	  	
		}
		als_enable = 0;
		atomic_set(&obj->als_deb_on, 0);
	}

	return 0;
}

/********************************************************************/
int pa22x_read_ps(struct i2c_client *client, u8 *data)
{
	int res;

	//APS_FUN(f);

	mutex_lock(&pa22x_obj->update_lock);
	res = hwmsen_read_byte(client, REG_PS_DATA, data); 
	mutex_unlock(&pa22x_obj->update_lock);

	if(res < 0)
	{
		APS_ERR("i2c_send function err\n");
	}
	//APS_LOG("PA22X_PS_DATA value = %x\n",*data);	
	return res;
}
/********************************************************************/
int pa22x_read_als(struct i2c_client *client, u16 *data)
{
	int res;
	u8 dataLSB;
	u8 dataMSB;
	u16 count;

	u16 temp_data[10];	
	unsigned int ArySize = 10;	
	int i, j;
	u32 sum_of_data = 0;
	//APS_FUN(f);

	if (ALS_USE_AVG_DATA)
	{ 
		mutex_lock(&pa22x_obj->update_lock);
		for(i = 0; i < ArySize; i++)
		{		
			res = hwmsen_read_byte(client, REG_ALS_DATA_LSB, &dataLSB); 
			res = hwmsen_read_byte(client, REG_ALS_DATA_MSB, &dataMSB);
			if(res < 0)
			{
				APS_ERR("i2c_send function err\n");
				return res;
			}
			temp_data[i] = ((dataMSB << 8) | dataLSB);
			msleep(5);	
		}
		mutex_unlock(&pa22x_obj->update_lock);

		/* data sorting */
		for (i = 0; i < ArySize - 1; i++)
			for (j = i+1; j < ArySize; j++)
				if (temp_data[i] > temp_data[j])
						pa22_swap((u8 *)(temp_data + i), (u8 *)(temp_data + j));	

		/* using central 6 data */
		for (i = 2; i < ArySize - 2; i++) 
		{
			APS_LOG("%s: temp_data = %d\n", __func__, temp_data[i]);
			sum_of_data = sum_of_data + temp_data[i];
		}	

		count = sum_of_data/6;
	}
	else
	{
		mutex_lock(&pa22x_obj->update_lock);
		res = hwmsen_read_byte(client, REG_ALS_DATA_LSB, &dataLSB); 
		res = hwmsen_read_byte(client, REG_ALS_DATA_MSB, &dataMSB);
		mutex_unlock(&pa22x_obj->update_lock);
		count = ((dataMSB << 8) | dataLSB);
	}
	
	APS_LOG("PA22X_ALS_DATA count=%d\n ",count);

	*data = count;

	return 0;
}

static void pa22x_get_ps_slope_array(int *ps_seq, int *slope, u8 ps, int arysize)
{
	int i;

	for (i=0; i<arysize-1; i++)
	{
		ps_seq[arysize-1-i] = ps_seq[arysize-1-i-1];
		if (ps_seq[arysize-1-i] == 0)
			ps_seq[arysize-1-i] = ps;
	}
	ps_seq[0] = (int)ps;

	for (i=0; i<arysize-1; i++)
	{
		slope[i] = ps_seq[i] - ps_seq[i+1];
	}
	return;
}

/**Change to near/far ****************************************************/
static int pa22x_get_ps_value(struct pa22x_priv *obj, u8 ps)
{
	int val = 0;
	int invalid = 0;
	//int mask = atomic_read(&obj->ps_mask);
	int arysize = 5;
	static int ps_seq[5];
	int slope[arysize-1];
	int i;
	static int close_loop = 0;
	static int away_loop = 0;
	static int near_ps = 20;
	int close,away;
	int sum = 0, abs_sum = 0;
	static int ps_loop = 0;

	while (ps_loop < 5)
	{
		ps_loop++;
		return prevObj;
	}

	ps_loop = 0;

	for (i=0; i<arysize-1; i++)
	{
		ps_seq[arysize-1-i] = ps_seq[arysize-1-i-1];
		if (ps_seq[arysize-1-i] == 0)
			ps_seq[arysize-1-i] = ps;
	}
	ps_seq[0] = (int)ps;

	for (i=0; i<arysize-1; i++)
	{
		slope[i] = ps_seq[i] - ps_seq[i+1];
	}


	ARRAY_SUM(slope, sum);
	ARRAY_ABS_SUM(slope, abs_sum);
	if(sum == 0 || abs_sum < 5)
	{
		close_loop = 0;
		away_loop = 0;
		//goto err_out;
	}

	//int close, away;
	IS_CLOSE(slope, close);
	IS_AWAY(slope, away);
	if(prevObj == 1)
	{
		if ( ps > 200 && close)
		{
			APS_LOG("********ps > 200********\n");
			goto NEAR;
		} 
		else if (sum > 15  && close)
		{
			APS_LOG("********sum > 15 & close********\n");
			goto NEAR;
		}
		else if (ps > 150 && abs_sum < 15)
		{
			APS_LOG("********ps > 150 & steady********\n");
			goto NEAR;	
		}
		else 
			goto err_out;
	NEAR:
			near_ps = (int)ps + 5;
			if (near_ps > 255)
				near_ps = 255;
			APS_LOG("---Close---\n");
			APS_LOG("slope : %d %d %d %d\n", slope[3], slope[2], slope[1], slope[0]);
			APS_LOG("value : %d %d %d %d %d\n", ps_seq[4], ps_seq[3], ps_seq[2], ps_seq[1], ps_seq[0]);
			APS_LOG("near_ps = %d\n", near_ps);
			val = 0;  /*close*/
			prevObj=0;
			return 0;
	}
	else if (prevObj == 0)
	{	
		if ( ps < ( (u8)(near_ps - 15)>180 ? 150 : (u8)(near_ps - 15) ) 
			 && abs_sum < 15 
		   )
		{
			APS_LOG("---Away---\n");
			APS_LOG("slope : %d %d %d %d\n", slope[3], slope[2], slope[1], slope[0]);
			APS_LOG("value : %d %d %d %d %d\n", ps_seq[4], ps_seq[3], ps_seq[2], ps_seq[1], ps_seq[0]);
			val = 1;  /*far away*/
			prevObj=1;
			return 1;
		}	
	}
	
err_out:
	//APS_LOG("---NOTHING---\n");
	//APS_LOG("slope : %d %d %d %d\n", slope[3], slope[2], slope[1], slope[0]);
	//APS_LOG("value : %d %d %d %d %d\n", ps_seq[4], ps_seq[3], ps_seq[2], ps_seq[1], ps_seq[0]);
	return prevObj;


	if(atomic_read(&obj->ps_suspend))
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
		return -1;
	}
}

/**Change to luxr************************************************/
static int pa22x_get_als_value(struct pa22x_priv *obj, u16 als)
{
	u64 lux=0;
	u8 reg0data,afsrdata,temp;	
	int res;
	mutex_lock(&pa22x_obj->update_lock);
	res = hwmsen_read_byte(pa22x_obj->client, REG_CFG0, &reg0data);
	mutex_unlock(&pa22x_obj->update_lock);
	afsrdata = reg0data>> 4;	
	lux = als * PA_ALS_CALIBRATION_LUX_TARGET /obj->als_caliADC_IR[afsrdata] ;
	APS_LOG("als:%d lux:%d afsrdata:%d\n",(int)als,(int)lux,afsrdata);
	APS_LOG("reg0data:%d\n",(int)reg0data);
	if (PA_ALS_AUTOGAIN)
	{
		if ( als > PA_ALS_MAX_COUNT && afsrdata < 3)
		{		
			afsrdata++;
			temp = (reg0data & 0x0F ) | (afsrdata << 4);
			// Write into Register
			mutex_lock(&pa22x_obj->update_lock);
			res = hwmsen_write_byte(pa22x_obj->client, REG_CFG0, temp);
			mutex_unlock(&pa22x_obj->update_lock);		
			msleep(200);
		}
		else if ( als < PA_ALS_MIN_COUNT && afsrdata > 0)
		{	
			afsrdata--;
			temp = (reg0data & 0x0F ) | (afsrdata << 4);
			// Write into Register
			mutex_lock(&pa22x_obj->update_lock);
			res = hwmsen_write_byte(pa22x_obj->client, REG_CFG0, temp);
			mutex_unlock(&pa22x_obj->update_lock);		
			msleep(200);		
		}	
	}
	APS_LOG("pa22x_get_als_value,ret = %d\n",(int)lux);
	return (int)lux;

}


/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t pa22x_show_version(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, ".H Ver: %s\n.C Ver: %s\n",PA22_DRIVER_VERSION_H,PA22_DRIVER_VERSION_C); 
	return res;    
}
static ssize_t pa22x_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
		atomic_read(&pa22x_obj->i2c_retry), atomic_read(&pa22x_obj->als_debounce), 
		atomic_read(&pa22x_obj->ps_mask), atomic_read(&pa22x_obj->ps_thd_val), atomic_read(&pa22x_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&pa22x_obj->i2c_retry, retry);
		atomic_set(&pa22x_obj->als_debounce, als_deb);
		atomic_set(&pa22x_obj->ps_mask, mask);
		atomic_set(&pa22x_obj->ps_thd_val, thres);        
		atomic_set(&pa22x_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %lu\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
#ifndef DEVICE_PA224
static ssize_t pa22x_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	if((res = pa22x_read_als(pa22x_obj->client, &pa22x_obj->als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "%d\n", pa22x_obj->als);     
	}
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	if((res = pa22x_read_ps(pa22x_obj->client, &pa22x_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %lu\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "%d\n", pa22x_obj->ps);     
	}

}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_show_reg(struct device_driver *ddri, char *buf)
{
	u8 regdata;
	int res=0;
	int count=0;
	int i=0	;
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	

	mutex_lock(&pa22x_obj->update_lock);
	for(i=0;i <19 ;i++)
	{
		res=hwmsen_read_byte(pa22x_obj->client,0x00+i,&regdata);

		if(res<0)
		{
		   break;
		}
		else
		count+=sprintf(buf+count,"[%x] = (%x)\n",0x00+i,regdata);
	}
	mutex_unlock(&pa22x_obj->update_lock);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;


	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	mutex_lock(&pa22x_obj->update_lock);		
	hwmsen_write_byte(pa22x_obj->client,addr,cmd);
	mutex_unlock(&pa22x_obj->update_lock);
	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	//u8 dat;
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
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
static ssize_t pa22x_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	
	if(pa22x_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			pa22x_obj->hw->i2c_num, pa22x_obj->hw->power_id, pa22x_obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02X %02lX %02lX\n", 
				atomic_read(&pa22x_obj->als_cmd_val), atomic_read(&pa22x_obj->ps_cmd_val), 
				atomic_read(&pa22x_obj->ps_thd_val),pa22x_obj->enable, pa22x_obj->pending_intr);
	
	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&pa22x_obj->als_suspend), atomic_read(&pa22x_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
#ifndef DEVICE_PA224
static int read_int_from_buf(struct pa22x_priv *obj, const char* buf, size_t count, u32 data[], int len)
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

static ssize_t pa22x_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < pa22x_obj->als_level_num; idx++)
	{

		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", pa22x_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(pa22x_obj->als_level, pa22x_obj->hw->als_level, sizeof(pa22x_obj->als_level));
	}
	else if(pa22x_obj->als_level_num != read_int_from_buf(pa22x_obj, buf, count, 
			pa22x_obj->hw->als_level, pa22x_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t pa22x_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < pa22x_obj->als_value_num; idx++)
	{

		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", pa22x_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}

/*----------------------------------------------------------------------------*/
static ssize_t pa22x_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(pa22x_obj->als_value, pa22x_obj->hw->als_value, sizeof(pa22x_obj->als_value));
	}
	else if(pa22x_obj->als_value_num != read_int_from_buf(pa22x_obj, buf, count, 
			pa22x_obj->hw->als_value, pa22x_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
#endif
/*---Offset At-------------------------------------------------------------------------*/
static ssize_t pa22x_show_ps_offset(struct device_driver *ddri, char *buf)
{
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", pa22x_obj->crosstalk);     

}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_set_ps_offset(struct device_driver *ddri, const char *buf, size_t count)
{
	int ret;
	ret = pa22x_run_calibration(pa22x_obj->client);
	return ret;
}
/*----------------------------------------------------------------------------*/
#if 0
static ssize_t pa22x_show_reg_add(struct device_driver *ddri, char *buf)
{return 0;}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_show_reg_value(struct device_driver *ddri, char *buf)
{return 0;}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_store_dev_init(struct device_driver *ddri, const char *buf, size_t count)
{
	int ret;
	ret = pa22x_init_client(pa22x_obj->client);
	return count;
}
/*----------------------------------------------------------------------------*/
 
static ssize_t pa22x_show_test(struct device_driver *ddri, char *buf)
{
     return sprintf(buf, "int_type = %s\n",(int_type?"hysteresis":"window"));
}
/*----------------------------------------------------------------------------*/
static ssize_t pa22x_store_test(struct device_driver *ddri, const char *buf, size_t count)
{	 
	if (int_type == window)
	{
		APS_ERR("Current type is window, change to hysteresis\n");
		int_type = hysteresis;

		/*eint mask*/
		//mt65xx_eint_mask(CUST_EINT_ALS_NUM);
		disable_irq(pa22x_obj->irq);
		
		mutex_lock(&pa22x_obj->update_lock);
		
		/*change thresholds for hysteresis*/
		hwmsen_write_byte(pa22x_obj->client, REG_PS_TH, far_ps_min+50);
		hwmsen_write_byte(pa22x_obj->client, REG_PS_TL, far_ps_min+30);
		/*change to hysteresis type*/
		hwmsen_write_byte(pa22x_obj->client, REG_CFG3, (1<<6) | (PA22_PS_PERIOD<<3));

		mutex_unlock(&pa22x_obj->update_lock);

		APS_ERR("Hysteresis type:thresholds=(%d,%d)\n", far_ps_min+40, far_ps_min+30);
		return count;
	}
	else if (int_type == hysteresis)
	{
		APS_ERR("Current type is hysteresis, change to window\n");
		int_type = window;

		/*reset all flags*/
		saturation_flag = 0;
		oil_occurred = 0;
		far_ps_min = PA22_PS_OFFSET_MAX;
		intr_flag = 0;
		
		mutex_lock(&pa22x_obj->update_lock);

		/*change thresholds for window*/
		pa22x_obj->ps_thrd_high = 80;
		pa22x_obj->ps_thrd_low = 79;
		hwmsen_write_byte(pa22x_obj->client, REG_PS_TH, 80);
		hwmsen_write_byte(pa22x_obj->client, REG_PS_TL, 79);
		/*change to window type*/
		hwmsen_write_byte(pa22x_obj->client, REG_CFG3, (0<<6) | (PA22_PS_PERIOD<<3));

		mutex_unlock(&pa22x_obj->update_lock);

		/*eint mask*/
		//mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
		enable_irq(pa22x_obj->irq);
		return count;		
	}
	return 0;
} 

/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(version,     S_IWUSR | S_IRUGO, pa22x_show_version, NULL);

static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, pa22x_show_ps, NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, pa22x_show_config,	pa22x_store_config);
#ifndef DEVICE_PA224
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, pa22x_show_alslv, pa22x_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, pa22x_show_alsval, pa22x_store_alsval);
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, pa22x_show_als, NULL);
#endif
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, pa22x_show_status, NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, pa22x_show_send, pa22x_store_send); // No func
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, pa22x_show_recv, pa22x_store_recv);    // No func
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, pa22x_show_reg, NULL);
static DRIVER_ATTR(pscalibration, S_IWUSR | S_IRUGO, pa22x_show_ps_offset,pa22x_set_ps_offset);
static DRIVER_ATTR(dev_init,S_IWUSR | S_IRUGO, NULL, pa22x_store_dev_init);
static DRIVER_ATTR(test, S_IWUSR | S_IRUGO,pa22x_show_test, pa22x_store_test);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *pa22x_attr_list[] = {
	&driver_attr_version,    
	&driver_attr_ps,    	
	&driver_attr_config,
#ifndef DEVICE_PA224	
	&driver_attr_als,	
	&driver_attr_alslv,	
	&driver_attr_alsval,
#endif	
	&driver_attr_status,
	&driver_attr_send,
	&driver_attr_recv,
	&driver_attr_reg,
	&driver_attr_pscalibration,
	&driver_attr_dev_init,
	&driver_attr_test,
};

/*----------------------------------------------------------------------------*/
static int pa22x_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(pa22x_attr_list)/sizeof(pa22x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, pa22x_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", pa22x_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int pa22x_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(pa22x_attr_list)/sizeof(pa22x_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, pa22x_attr_list[idx]);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------interrupt functions--------------------------------*/

/*----------------------------------------------------------------------------*/
static int pa22x_check_intr_als(struct i2c_client *client) 
{
	struct pa22x_priv *obj = i2c_get_clientdata(client);	
	int i = 0;
	int level = 0;
	u8 regdata = 0;	
	pa22x_read_als(client, &obj->als);
	/* Check als value now at what level */
	for(i = 1;i < obj->als_level_num; i++){
		if(obj->als <= obj->hw->als_level[i]){
			level = i;
			break;
		}						
	}
	APS_LOG("level:%d, alsht:%d, alslt:%d\n",level,
			obj->hw->als_level[level], obj->hw->als_level[level-1]);
	
	mutex_lock(&obj->update_lock);
	/* Dynamically change thresholds */	
	hwmsen_write_byte(client, REG_ALS_TL_LSB, obj->hw->als_level[level-1] & 0xFF);
	hwmsen_write_byte(client, REG_ALS_TL_MSB, obj->hw->als_level[level-1] >> 8);
	hwmsen_write_byte(client, REG_ALS_TH_LSB, obj->hw->als_level[level] & 0xFF);
	hwmsen_write_byte(client, REG_ALS_TH_MSB, obj->hw->als_level[level] >> 8);

	
	/* Clear ALS interrupt flag */
	hwmsen_read_byte(client, REG_CFG2, &regdata);
	regdata &= 0xFE;
	hwmsen_write_byte(client, REG_CFG2, regdata);

	mutex_unlock(&obj->update_lock);

	return 0;
	
}
static int pa22x_check_intr(struct i2c_client *client) 
{
	struct pa22x_priv *obj = i2c_get_clientdata(client);
	int res;
	//u8 databuf[2];
	u8 psdata=0;
	u8 cfgdata=0;
	static int far_loop = 0;

	int slope[ps_ary_size-1];	
	int sum = 0, abs_sum = 0, ps_sum = 0;	
	
	mutex_lock(&obj->update_lock);	
	res = hwmsen_read_byte(client, REG_PS_DATA, &psdata);
	mutex_unlock(&obj->update_lock);
	/* Add for soft TP effort xtalk down*/
	//if (psdata < 35)
	//	goto check_intr_exit;
	if(res<0)
	{
		APS_ERR("i2c_read function err res = %d\n",res);
		return -1;
	}
	//SUNLIGHT
	
	if (psdata == 0) 
	{
		if (intr_flag == 0){
			hwmsen_write_byte(client,REG_CFG1,
						(PA22_LED_CURR << 4)| (PA22_PS_PRST << 2) );	
		}
		saturation_flag = 1;
		if (oil_occurred && far_ps_min < PA22_PS_OFFSET_MAX)
		{
			obj->ps_thrd_high = far_ps_min + OIL_EFFECT + PA_PRX_NEAR_DIFF;
			obj->ps_thrd_low = far_ps_min + OIL_EFFECT;			
		}
		else if (!oil_occurred && far_ps_min < PA22_PS_OFFSET_MAX)
		{
			obj->ps_thrd_high = far_ps_min + PA_PRX_NEAR_DIFF;
			obj->ps_thrd_low = far_ps_min + PA_PRX_FAR_DIFF;
		}
		else if (far_ps_min == PA22_PS_OFFSET_MAX)
		{
			obj->ps_thrd_high = 80;
			obj->ps_thrd_low = 79;
		}
		msleep(saturation_delay);
		APS_LOG("Sun light!!, ht=%d, lt=%d, far_ps_min=%d\n", obj->ps_thrd_high, obj->ps_thrd_low, far_ps_min);
		intr_flag = 1;
		goto check_intr_exit;
	}
	//FARTHER AWAY
	if (psdata < obj->ps_thrd_low && intr_flag == 1)
	{

		pa22x_get_ps_slope_array(ps_seq_far, slope, psdata, ps_ary_size);
		ARRAY_SUM(ps_seq_far, ps_sum);
		ARRAY_SUM(slope, sum);
		ARRAY_ABS_SUM(slope, abs_sum);
		APS_ERR("slope : %d %d %d %d\n", slope[3], slope[2], slope[1], slope[0]);
		APS_ERR("ps_seq_far value : %d %d %d %d %d\n", ps_seq_far[4], ps_seq_far[3], ps_seq_far[2], ps_seq_far[1], ps_seq_far[0]);		
		APS_ERR("saturation_flag=%d\n", saturation_flag);
		//If saturation happened, the average ps value must be greater than (far_ps_min-10) and also  steady
		if ( (saturation_flag && ps_sum/ps_ary_size >= ( far_ps_min>10 ? (far_ps_min-10) : far_ps_min )) 
			  || !saturation_flag || (saturation_flag && far_ps_min == PA22_PS_OFFSET_MAX) ) 
		{
			//STEADY
			if (abs_sum < ps_steady) 
			{
				if (saturation_flag)
					saturation_flag = 0;				
		
				intr_flag = 1;
				oil_occurred = 0;
				far_ps_min = ps_sum/ps_ary_size;
				obj->ps_thrd_high = far_ps_min+PA_PRX_NEAR_DIFF;
				obj->ps_thrd_low = far_ps_min>5? (far_ps_min-5):1;
				hwmsen_write_byte(client, REG_CFG3, (PA22_INT_TYPE << 6)| (PA22_PS_PERIOD << 3));
				APS_ERR("FARTHER, far_ps_min %3d high low : %3d %3d\n", far_ps_min, obj->ps_thrd_high, obj->ps_thrd_low);
			}	
		}
		msleep(sequence_dealy);	
	}
	//NEAR 
	else if (psdata > obj->ps_thrd_high)
	{
		int i = 0;
		for (i=0; i<ps_ary_size; i++)
		{
			hwmsen_read_byte(client, REG_PS_DATA, (u8 *)(&(ps_seq_near[i])));
			if (i > 0)
				slope[i-1] = ps_seq_near[i]-ps_seq_near[i-1];
			mdelay(5);
		}
		//pa22x_get_ps_slope_array(ps_seq_near, slope, psdata, ps_ary_size);
		APS_ERR("slope : %d %d %d %d\n", slope[3], slope[2], slope[1], slope[0]);
		APS_ERR("near value : %d %d %d %d %d\n", ps_seq_near[4], ps_seq_near[3], ps_seq_near[2], ps_seq_near[1], ps_seq_near[0]);			
		ARRAY_ABS_SUM(slope, abs_sum);
		if ( abs_sum < ps_steady )
		{
			intr_flag = 0;
			if (psdata >= 254)
			{
				far_loop = 0;
				oil_occurred = 1;
				obj->ps_thrd_low = far_ps_min + OIL_EFFECT;
				obj->ps_thrd_high = 0xFF;
			}
			else
			{
				obj->ps_thrd_low = far_ps_min+PA_PRX_FAR_DIFF; 
				obj->ps_thrd_high = 254;//(far_ps_min + OIL_EFFECT);
			}
			APS_LOG("NER, ps %3d high low : %3d %3d\n", psdata, obj->ps_thrd_high, obj->ps_thrd_low);
		}else if (abs_sum > 20){
			/*Flicker light*/
			hwmsen_write_byte(client, REG_CFG3, (PA22_INT_TYPE << 6)| (0 << 3));
			APS_ERR("Flicker light!!!!");
		}
	}
	//FAR AWAY
	if (psdata < obj->ps_thrd_low && intr_flag == 0) 
	{
		if (oil_occurred)
		{
			far_loop++;
			pa22x_get_ps_slope_array(ps_seq_oil, slope, psdata, ps_ary_size);
			ARRAY_SUM(ps_seq_oil, ps_sum);
			ARRAY_SUM(slope, sum);
			ARRAY_ABS_SUM(slope, abs_sum);	
			APS_ERR("slope : %d %d %d %d\n", slope[3], slope[2], slope[1], slope[0]);
			APS_ERR("oil value : %d %d %d %d %d\n", ps_seq_oil[4], ps_seq_oil[3], ps_seq_oil[2], ps_seq_oil[1], ps_seq_oil[0]);	
			//STEADY
			if ( abs_sum < ps_steady || far_loop > 10) 
			{		
				intr_flag = 1;
				oil_occurred = 0;
				/*
				if (far_loop <= 10)
					far_ps_min = ps_sum/ps_ary_size;
				obj->ps_thrd_high = far_ps_min+15;
				//obj->ps_thrd_low = far_ps_min>5? (far_ps_min-backward_step):5;
				obj->ps_thrd_low = far_ps_min+8;
				*/
				
				if (far_loop <= 10){
					far_ps_min = ps_sum/ps_ary_size;
					obj->ps_thrd_high = far_ps_min+PA_PRX_NEAR_DIFF;
					obj->ps_thrd_low = far_ps_min > PA_PRX_NEAR_DIFF? (far_ps_min-5):PA_PRX_NEAR_DIFF;
				}else{
					far_ps_min = far_ps_min + PA_PRX_NEAR_DIFF;
					obj->ps_thrd_high = far_ps_min+PA_PRX_NEAR_DIFF;
					obj->ps_thrd_low = far_ps_min+5;
				}
					
				hwmsen_write_byte(client, REG_CFG3, (PA22_INT_TYPE << 6)| (PA22_PS_PERIOD << 3));
				APS_LOG("OIL to FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, obj->ps_thrd_high, obj->ps_thrd_low);		
			}
			msleep(sequence_dealy);
		}
		else
		{	
			intr_flag = 1;
			obj->ps_thrd_high = far_ps_min+PA_PRX_NEAR_DIFF;
			obj->ps_thrd_low = far_ps_min > PA_PRX_NEAR_DIFF? (far_ps_min-5):PA_PRX_NEAR_DIFF;
			APS_ERR("FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, obj->ps_thrd_high, obj->ps_thrd_low);
		}

	}

check_intr_exit:
	mutex_lock(&obj->update_lock);
	APS_ERR("ps_data:%d far_ps_min %3d high low : %3d %3d\n",psdata, far_ps_min, obj->ps_thrd_high, obj->ps_thrd_low);
	hwmsen_write_byte(client, REG_PS_TL, obj->ps_thrd_low);
	hwmsen_write_byte(client, REG_PS_TH, obj->ps_thrd_high);
	mutex_unlock(&obj->update_lock);	

	/* Clear PS INT FLAG */
	mutex_lock(&obj->update_lock);
	res = hwmsen_read_byte(client, REG_CFG2, &cfgdata);
	mutex_unlock(&obj->update_lock);

	if(res<0)
	{
		APS_ERR("i2c_read function err res = %d\n",res);
		return -1;
	}
	cfgdata = cfgdata & 0xFD ; 
	mutex_lock(&obj->update_lock);
	res = hwmsen_write_byte(client,REG_CFG2,cfgdata);
	mutex_unlock(&obj->update_lock);
  	if(res<0)
  	{		
		APS_ERR("i2c_send function err res = %d\n",res);
		return -1;
	}
//EXIT_CHECK_INTR:
	return 0;
}
/*----------------------------------------------------------------------------*/
static void pa22x_eint_work(struct work_struct *work)
{
	struct pa22x_priv *obj = (struct pa22x_priv *)container_of(work, struct pa22x_priv, eint_work);
	hwm_sensor_data sensor_data;
	int res = 0;
	u8 regdata = 0;
	int als_intr_active = 0, ps_intr_active = 0;
	memset(&sensor_data, 0, sizeof(sensor_data));

	/* Read interrput flag */
	mutex_lock(&obj->update_lock);
	hwmsen_read_byte(obj->client, REG_CFG2, &regdata);
	mutex_unlock(&obj->update_lock);
	als_intr_active = regdata  & 0x01;
	ps_intr_active = regdata & 0x02;

	if(!(obj->hw->polling_mode_ps) && ps_intr_active)
	{
		res = pa22x_check_intr(obj->client);
	
		if(res != 0){
			goto EXIT_INTR_ERR;
		}else{
			sensor_data.values[0] = intr_flag;
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;	

		}
		APS_LOG("===mlk===%s = ps  report data !!!\n",__func__);
		/*
		if((res = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
			APS_ERR("call hwmsen_get_interrupt_ps_data fail = %d\n", res);
			goto EXIT_INTR_ERR;
		}*/
		if(ps_report_interrupt_data(sensor_data.values[0]))
		{	
			APS_ERR("call ps_report_interrupt_data fail\n");
		} 
		
	}

	if(!(obj->hw->polling_mode_als) && als_intr_active)
	{
		res = pa22x_check_intr_als(obj->client);
	
		if(res != 0){
			goto EXIT_INTR_ERR;
		}else{
			sensor_data.values[0] = pa22x_get_als_value(obj, obj->als);
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;	

		}
		/*
		if((res = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data)))
		{
			APS_ERR("call hwmsen_get_interrupt_als_data fail = %d\n", res);
			goto EXIT_INTR_ERR;
		}
		*/
		if(ps_report_interrupt_data(sensor_data.values[0]))
		{	
			APS_ERR("call ps_report_interrupt_data fail\n");
		} 

	}

	
	enable_irq(obj->irq);
	return;


EXIT_INTR_ERR:
	enable_irq(obj->irq);

	APS_ERR("pa22x_eint_work err: %d\n", res);
}

/*----------------------------------------------------------------------------*/
static void pa22x_eint_func(void)
{
	struct pa22x_priv *obj = g_pa22x_ptr;
	if(!obj)
	{
		return;
	}	
	schedule_work(&obj->eint_work);
}
static irqreturn_t pa22x_eint_handler(int irq, void *desc)
{
	APS_LOG("pa22x_eint_handle\n");
	pa22x_eint_func();
	disable_irq_nosync(pa22x_obj->irq);

	return IRQ_HANDLED;
}


int pa22x_setup_eint(struct i2c_client *client)
{
	//int ret;
	u32 ints[2] = {0, 0};
/*	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;*/
	//struct stk3x1x_priv *obj = i2c_get_clientdata(client);        

	
	/*configure to GPIO function, external interrupt*/
/*
   alspsPltFmDev = get_alsps_platformdev();

	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");

	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

	}
	*/
/* eint request */
	if (pa22x_obj->irq_node) {
		of_property_read_u32_array(pa22x_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		//pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		pa22x_obj->irq = irq_of_parse_and_map(pa22x_obj->irq_node, 0);
		APS_LOG("pa22x_obj->irq = %d\n", pa22x_obj->irq);
		if (!pa22x_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if (request_irq(pa22x_obj->irq, pa22x_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		//enable_irq(pa22x_obj->irq);
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}


	return 0;
}
/*-------------------------------MISC device related------------------------------------------*/

/************************************************************/
static int pa22x_open(struct inode *inode, struct file *file)
{
	file->private_data = pa22x_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/

static int pa22x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static long pa22x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
		struct i2c_client *client = (struct i2c_client*)file->private_data;
		struct pa22x_priv *obj = i2c_get_clientdata(client);  
		long err = 0;
		void __user *ptr = (void __user*) arg;
		int dat;
		uint32_t enable;
		//int ps_result;

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
					if((err = pa22x_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %ld\n", err); 
						goto err_out;
					}

					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
					if((err = pa22x_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %ld\n", err); 
						goto err_out;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
				break;

			case ALSPS_GET_PS_MODE:
				enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_GET_PS_DATA:    
				if((err = pa22x_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}

				dat = pa22x_get_ps_value(obj, obj->ps);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_GET_PS_RAW_DATA:    
				if((err = pa22x_read_ps(obj->client, &obj->ps)))
				{
					goto err_out;
				}

				dat = obj->ps;
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
					if((err = pa22x_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %ld\n", err); 
						goto err_out;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = pa22x_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %ld\n", err); 
						goto err_out;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
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
				if((err = pa22x_read_als(obj->client, &obj->als)))
				{
					goto err_out;
				}
	
				dat = pa22x_get_als_value(obj, obj->als);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_GET_ALS_RAW_DATA:	
				if((err = pa22x_read_als(obj->client, &obj->als)))
				{
					goto err_out;
				}

				dat = obj->als;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
			#if !defined(MT6589) && !defined(MT6572)
			case ALSPS_IOCTL_CLR_CALI:
				APS_ERR("%s ALSPS_IOCTL_CLR_CALI\n", __func__);
				if(copy_from_user(&dat, ptr, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_IOCTL_GET_CALI:
				dat = obj->crosstalk ;
				APS_ERR("%s set ps_cali %x\n", __func__, dat);
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_IOCTL_SET_CALI:
				APS_ERR("%s set ps_cali %x\n", __func__, obj->crosstalk); 
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
static struct file_operations pa22x_fops = {
	.owner = THIS_MODULE,
	.open = pa22x_open,
	.release = pa22x_release,
	.unlocked_ioctl = pa22x_unlocked_ioctl,
};

static struct miscdevice pa22x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &pa22x_fops,
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
/*--------------------------------------------------------------------------------------*/
static void pa22x_early_suspend(struct early_suspend *h)
{
	struct pa22x_priv *obj = container_of(h, struct pa22x_priv, early_drv);
#ifndef DEVICE_PA224
	int err;
#endif
	APS_FUN();	  

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
#ifndef DEVICE_PA224
	atomic_set(&obj->als_suspend, 1);
		if((err = pa22x_enable_als(obj->client, 0)))
	{
		APS_ERR("disable als fail: %d\n", err);
	}
#endif
}

static void pa22x_late_resume(struct early_suspend *h) 
{
		/*early_suspend is only applied for ALS*/
		struct pa22x_priv *obj = container_of(h, struct pa22x_priv, early_drv);		  
#ifndef DEVICE_PA224
		int err;
#endif
		hwm_sensor_data sensor_data;
		memset(&sensor_data, 0, sizeof(sensor_data));
		APS_FUN();
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return;
		}

		atomic_set(&obj->als_suspend, 0);
		if(test_bit(CMC_BIT_ALS, &obj->enable))
		{
#ifndef DEVICE_PA224
			if((err = pa22x_enable_als(obj->client, 1)))
			{
				APS_ERR("enable als fail: %d\n", err);		  
	
			}
#endif
		}
}
#endif
/*--------------------------------------------------------------------------------*/
static int pa22x_init_client(struct i2c_client *client)
{
	struct pa22x_priv *obj = i2c_get_clientdata(client);
	u8 sendvalue=0;
	int res = 0;
	int intmode ;
	int i;
	// Initialize Sensor
	mutex_lock(&obj->update_lock);
	res=hwmsen_write_byte(client,REG_CFG0,
		PA22_ALS_GAIN << 4);
	
	res=hwmsen_write_byte(client,REG_CFG1,
		(PA22_LED_CURR << 4)| (PA22_PS_PRST << 2)| (PA22_ALS_PRST));

	res=hwmsen_write_byte(client,REG_CFG2,
		(PA22_PS_MODE << 6)| (PA22_PS_SET << 2));

	res=hwmsen_write_byte(client,REG_CFG3,
		(PA22_INT_TYPE << 6)| (PA22_PS_PERIOD << 3)| (PA22_ALS_PERIOD));

	res=hwmsen_write_byte(client,REG_PS_SET,0x82); 

	res=hwmsen_write_byte(client,REG_CFG4,PA22_PS_FLTFC << 4 | 0x0C); 

	obj->ps_thrd_low = PA22_PS_TH_LOW;
	obj->ps_thrd_high = PA22_PS_TH_HIGH;
	for(i=0;i<4;i++)
		obj->als_caliADC_IR[i] = als_adc_coefs_IR[i];

	/* Set ALS threshold */
	if(obj->hw->polling_mode_als == 0)
	{
		res = hwmsen_write_byte(client, REG_ALS_TH_MSB, obj->hw->als_level[1] >> 8); 
		res = hwmsen_write_byte(client, REG_ALS_TH_LSB, obj->hw->als_level[1] & 0xFF); 
		res = hwmsen_write_byte(client, REG_ALS_TL_MSB, obj->hw->als_level[0] >> 8); 
		res = hwmsen_write_byte(client, REG_ALS_TL_LSB, obj->hw->als_level[0] & 0xFF); 
	}
	else if(obj->hw->polling_mode_als == 1)
	{
		res = hwmsen_write_byte(client, REG_ALS_TH_MSB, PA22_ALS_TH_HIGH >> 8); 
		res = hwmsen_write_byte(client, REG_ALS_TH_LSB, PA22_ALS_TH_HIGH & 0xFF); 
		res = hwmsen_write_byte(client, REG_ALS_TL_MSB, PA22_ALS_TH_LOW >> 8); 
		res = hwmsen_write_byte(client, REG_ALS_TL_LSB, PA22_ALS_TH_LOW & 0xFF); 
	}

	if(obj->hw->polling_mode_ps == 0)
	{
		/* Set PS threshold */
		if(PA22_INT_TYPE == 0)
		{
			/*Window Type */	
			res=hwmsen_write_byte(client, REG_PS_TH, obj->ps_thrd_high); 
			res=hwmsen_write_byte(client, REG_PS_TL, PA22_PS_TH_MIN); 
		}
		else if(PA22_INT_TYPE == 1)
		{
			/*Hysteresis Type */
			res=hwmsen_write_byte(client, REG_PS_TH, obj->ps_thrd_high); 
			res=hwmsen_write_byte(client, REG_PS_TL, obj->ps_thrd_low); 
		}
	}
	/* Polling Setting */	
	 intmode = obj->hw->polling_mode_ps << 1 | obj->hw->polling_mode_als;
  
	res = hwmsen_read_byte(client, REG_CFG2, &sendvalue);
	/* clear interrupt flag */
	sendvalue &= 0xF0; 
  
  	switch(intmode)
	{  	
		case 0:
			/* Both Interrupt */
			sendvalue |= 0x0C; 
			res = hwmsen_write_byte(client, REG_CFG2, sendvalue); 
 		case 1:
			/* PS Interrupt */
			sendvalue |= 0x04; 
			res = hwmsen_write_byte(client, REG_CFG2, sendvalue); 
			break;

		case 2:
			/* ALS Interrupt */
			sendvalue=sendvalue | 0x00; 
			res = hwmsen_write_byte(client, REG_CFG2, sendvalue); 
			break;
		default:
			/* No Interupt */
			sendvalue |= 0x04;  
			res = hwmsen_write_byte(client, REG_CFG2, sendvalue); 
			break;  			
	}

	mutex_unlock(&obj->update_lock);

	if(res < 0)
	{
		APS_ERR("i2c_send function err\n");
		goto EXIT_ERR;
	}
		
	/* Regsit interrupt */
	res = pa22x_setup_eint(client);
	if(res!=0)
	{
		APS_ERR("PA22X setup eint: %d\n", res);
		return res;
	}

	return 0;

EXIT_ERR:
	APS_ERR("pa22x init dev fail!!!!: %d\n", res);
	return res;
}

/*
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

static int als_enable_nodata(int en)
{
	int res = 0;
	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return -1;
	}
	if(en)
	{
		if((res= pa22x_enable_als(pa22x_obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", res); 
			return -1;
		}
		set_bit(CMC_BIT_ALS, &pa22x_obj->enable);
	}	
	else
	{
		if((res = pa22x_enable_als(pa22x_obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", res); 
			return -1;
		}
		clear_bit(CMC_BIT_ALS, &pa22x_obj->enable);
	}
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
    struct pa22x_priv *obj = NULL;

	if(!pa22x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return -1;
	}
	obj = pa22x_obj;
	if((err = pa22x_read_als(obj->client, &obj->als)))
	{
		err = -1;
	}
	else
	{
		*value = pa22x_get_als_value(obj, obj->als);		
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}
*/
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

static int ps_enable_nodata(int en)
{
	int res = 0;
	

	if(!pa22x_obj)
	{
		APS_ERR("pa22x_obj is null!!\n");
		return -1;
	}

	if(en)
	{
		if((res = pa22x_enable_ps(pa22x_obj->client, 1)))
		{
			APS_ERR("enable ps fail: %d\n", res); 
			return -1;
		}
		set_bit(CMC_BIT_PS, &pa22x_obj->enable);
	}
	else
	{
		if((res = pa22x_enable_ps(pa22x_obj->client, 0)))
		{
			APS_ERR("disable ps fail: %d\n", res); 
			return -1;
		}
		clear_bit(CMC_BIT_PS, &pa22x_obj->enable);
	}

	if(res){
		APS_ERR("ps_enable_nodata is failed!!\n");
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


    if(!pa22x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return -1;
	}
    
    if((err = pa22x_read_ps(pa22x_obj->client, &pa22x_obj->ps)))
    {
        err = -1;;
    }
    else
    {
        *value = pa22x_get_ps_value(pa22x_obj, pa22x_obj->ps);
        *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    }

	return 0;
}

/*-----------------------------------i2c operations----------------------------------*/
static int pa22x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pa22x_priv *obj;
/*	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};*/
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	
	int err = 0;
	APS_LOG("==mlk=== enter  %s\n",__func__);
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	obj->hw = hw;
	pa22x_obj = obj;
	g_pa22x_ptr = obj;
	
	mutex_init(&obj->update_lock); 

	INIT_WORK(&obj->eint_work, pa22x_eint_work);

	obj->client = client;
	i2c_set_clientdata(client, obj);

	/*-----------------------------value need to be confirmed-----------------------------------------*/
	if(obj->hw==NULL){
		APS_LOG("obj hw null\n");
		goto exit;
	}
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
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);

	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
	/*-----------------------------value need to be confirmed-----------------------------------------*/

//	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
//	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	pa22x_i2c_client = client;

	if((err = pa22x_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("pa22x_init_client() OK!\n");

	if((err = misc_register(&pa22x_device)))
	{
		APS_ERR("pa22x_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	APS_LOG("pa22x_device misc_register OK!\n");

	/*------------------------sl22201001 attribute file for debug--------------------------------------*/
	if((err = pa22x_create_attr(&(pa22x_init_info.platform_diver_addr->driver))))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------s12201001 attribute file for debug--------------------------------------*/
	/*
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
	}*/
	
	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
//	ps_ctl.is_report_input_direct = true;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;
#else
    ps_ctl.is_support_batch = false;
#endif

	if(1 == obj->hw->polling_mode_ps)
	{
		ps_ctl.is_polling_mode = 1;
		ps_ctl.is_report_input_direct = false;
	}
	else
	{
		ps_ctl.is_polling_mode = 0;//PS interrupt mode
		ps_ctl.is_report_input_direct = true;
	}
	
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

	/*err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register light batch support err = %d\n", err);
	}*/
	
	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 100, 0);
	if(err)
	{
		APS_ERR("register proximity batch support err = %d\n", err);
	}
	
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend  = pa22x_early_suspend,
	obj->early_drv.resume   = pa22x_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif
	APS_LOG("==mlk== %s  is ok !",__func__);
	pa22x_init_flag = 0;
	register_device_proc("Sensor_alsps", "pa22x", "TXC");
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
		misc_deregister(&pa22x_device);
exit_init_failed:
	kfree(obj);
exit:
	pa22x_i2c_client = NULL;   
	pa22x_init_flag = -1;	
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}

static int pa22x_i2c_remove(struct i2c_client *client)
{
	int err;	
	/*------------------------pa22x attribute file for debug--------------------------------------*/	
	if((err = pa22x_delete_attr(&pa22x_i2c_driver.driver)))
	{
		APS_ERR("pa22x_delete_attr fail: %d\n", err);
	} 
	/*----------------------------------------------------------------------------------------*/
	
	if((err = misc_deregister(&pa22x_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
		
	pa22x_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;

}

static int pa22x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, PA22X_DEV_NAME);
	return 0;

}

static int pa22x_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	APS_FUN();
	return 0;
}

static int pa22x_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int pa22x_local_uninit(void)
{
	
	pa22x_power(hw, 0);
	
	i2c_del_driver(&pa22x_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/



static int pa22x_local_init(void) {

	if (hw==NULL){
		APS_ERR("get cust hw info fail\n");
		return -1;
	}
	
	pa22x_power(hw, 1); 
	if(i2c_add_driver(&pa22x_i2c_driver))	
	{		
		APS_ERR("add driver error\n");		
		return -1;	
	}
	
	if(-1 == pa22x_init_flag)    
	{      
		return -1;    
	}
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init pa22x_init(void)
{	
	const char *name = "mediatek,pa22x";	
	get_alsps_dts_func(name, hw);
	if (hw==NULL){
		APS_ERR("get cust hw info fail\n");
		return -1;
	}
	i2c_register_board_info(hw->i2c_num, &i2c_pa22x, 1);
	alsps_driver_add(&pa22x_init_info);
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit pa22x_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(pa22x_init);
module_exit(pa22x_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("TXC Corp");
MODULE_DESCRIPTION("pa22a driver");
MODULE_LICENSE("GPL");

