/* ST LIS2DS12 Accelerometer sensor driver
 *
 *
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
#include <mach/mt_gpio.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include <linux/kernel.h>
#include <mach/mt_pm_ldo.h>
#include <cust_eint.h>
#include <mach/eint.h>

#include <cust_acc.h>
#include <accel.h>
#include <step_counter.h>
#include <tilt_detector.h>
#include "lis2ds12.h"

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#define LIS2DS12_EMBEDED_FUNC 1

#if (LIS2DS12_EMBEDED_FUNC)

#if defined(CONFIG_CUSTOM_KERNEL_STEP_COUNTER)
#define LIS2DS12_STEP_COUNTER 1
#endif //CONFIG_CUSTOM_KERNEL_STEP_COUNTER

#if defined(CONFIG_CUSTOM_KERNEL_TILT_DETECTOR_SENSOR)
#define LIS2DS12_TILT_FUNC 1
#endif //CONFIG_CUSTOM_KERNEL_TILT_DETECTOR_SENSOR

static bool pedo_enable_status = false;
static bool tilt_enable_status = false;

static int LIS2DS12_IntCtrl(struct i2c_client *client, LIS2DS12_ACC_INT_ACTIVE_t int_act, LIS2DS12_ACC_INT_LATCH_CTL_t int_latch);
#endif //LIS2DS12_EMBEDED_FUNC

#ifdef VENDOR_EDIT
struct step_process_tune
{
	u64 pre_value;
	u8 new_enable;
};
static struct step_process_tune step_tune;
#endif//VENDOR_EDIT
/*---------------------------------------------------------------------------*/
#define DEBUG 0
/*----------------------------------------------------------------------------*/
#define CONFIG_LIS2DS12_LOWPASS   // apply low pass filter on output
/*----------------------------------------------------------------------------*/
#define LIS2DS12_AXIS_X          0
#define LIS2DS12_AXIS_Y          1
#define LIS2DS12_AXIS_Z          2
#define LIS2DS12_ACC_AXES_NUM    3
#define LIS2DS12_ACC_DATA_LEN    6
#define LIS2DS12_ACC_DEV_NAME    "LIS2DS12_ACC"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id lis2ds12_i2c_id[] = {{LIS2DS12_ACC_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_lis2ds12={ I2C_BOARD_INFO(LIS2DS12_ACC_DEV_NAME, 0x1d)};

/*----------------------------------------------------------------------------*/
static int lis2ds12_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lis2ds12_i2c_remove(struct i2c_client *client);

#ifndef CONFIG_HAS_EARLYSUSPEND
static int lis2ds12_acc_suspend(struct i2c_client *client, pm_message_t msg);
static int lis2ds12_acc_resume(struct i2c_client *client);
#endif

static int LIS2DS12_InitClient(struct i2c_client *client, bool enable);
static int LIS2DS12_SetPowerMode(struct i2c_client *client, bool enable);
static int LIS2DS12_SetSampleRate(struct i2c_client *client, u8 sample_rate);

/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;

/*----------------------------------------------------------------------------*/
typedef enum {
    ACCEL_TRC_FILTER  = 0x01,
    ACCEL_TRC_RAWDATA = 0x02,
    ACCEL_TRC_IOCTL   = 0x04,
    ACCEL_TRC_CALI	= 0X08,
    ACCEL_TRC_INFO	= 0X10,
    ACCEL_TRC_DATA	= 0X20,
} ACCEL_TRC;

/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};

/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};

/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LIS2DS12_ACC_AXES_NUM];
    int sum[LIS2DS12_ACC_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct lis2ds12_i2c_data {
    struct i2c_client *client;
	struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    atomic_t 				layout;
    /*misc*/
    //struct data_resolution *reso;
#ifdef LIS2DS12_EMBEDED_FUNC
    struct work_struct	eint_work;
#endif				
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s32                     cali_sw[LIS2DS12_ACC_AXES_NUM];

    /*data*/
	s32                     offset[LIS2DS12_ACC_AXES_NUM];  /*+1: for 4-byte alignment*/
    s16                     data[LIS2DS12_ACC_AXES_NUM];

	int 					sensitivity;
	int 					sample_rate;

#if defined(CONFIG_LIS2DS12_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

/*----------------------------------------------------------------------------*/
static struct i2c_driver lis2ds12_i2c_driver = {
    .driver = {
        .owner          = THIS_MODULE,
        .name           = LIS2DS12_ACC_DEV_NAME,
    },
	.probe      		= lis2ds12_i2c_probe,
	.remove    			= lis2ds12_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = lis2ds12_acc_suspend,
    .resume             = lis2ds12_acc_resume,
#endif
	.id_table = lis2ds12_i2c_id,
};

static int lis2ds12_local_init(void);
static int lis2ds12_local_uninit(void);
static int lis2ds12_local_init_common(void);

static int lis2ds12_acc_init_flag = -1;
static u64 step_num = 0;
static unsigned long lis2ds12_init_flag_test = 0; //initial state
static DEFINE_MUTEX(lis2ds12_init_mutex);

typedef enum {
	LIS2DS12_ACC = 1,
	LIS2DS12_STEP_C = 2,
	LIS2DS12_TILT = 3,
}LIS2DS12_INIT_TYPE;


static struct acc_init_info  lis2ds12_init_info = {
	.name   = LIS2DS12_ACC_DEV_NAME,
	.init   = lis2ds12_local_init,
	.uninit = lis2ds12_local_uninit,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *lis2ds12_i2c_client = NULL;
static struct lis2ds12_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static bool enable_status = false;

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[accel] "
#define GSE_FUN(f)               printk(KERN_ERR GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_ERR GSE_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)

/*----------------------------------------------------------------------------*/
static void LIS2DS12_dumpReg(struct i2c_client *client)
{
  int i=0;
  u8 addr = 0x20;
  u8 regdata=0;
  for(i=0; i<31 ; i++)
  {
    //dump all
    hwmsen_read_byte(client,addr,&regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;	
  }
}

/*----------------------------------------------------------------------------*/
static void LIS2DS12_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "LIS2DS12"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "LIS2DS12"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
}

/*----------------------------------------------------------------------------*/
static int LIS2DS12_write_rel_calibration(struct lis2ds12_i2c_data *obj, int dat[LIS2DS12_ACC_AXES_NUM])
{
    obj->cali_sw[LIS2DS12_AXIS_X] = obj->cvt.sign[LIS2DS12_AXIS_X]*dat[obj->cvt.map[LIS2DS12_AXIS_X]];
    obj->cali_sw[LIS2DS12_AXIS_Y] = obj->cvt.sign[LIS2DS12_AXIS_Y]*dat[obj->cvt.map[LIS2DS12_AXIS_Y]];
    obj->cali_sw[LIS2DS12_AXIS_Z] = obj->cvt.sign[LIS2DS12_AXIS_Z]*dat[obj->cvt.map[LIS2DS12_AXIS_Z]];
#if DEBUG		
		if(atomic_read(&obj->trace) & ACCEL_TRC_CALI)
		{
			GSE_LOG("test  (%5d, %5d, %5d) ->(%5d, %5d, %5d)->(%5d, %5d, %5d))\n", 
				obj->cvt.sign[LIS2DS12_AXIS_X],obj->cvt.sign[LIS2DS12_AXIS_Y],obj->cvt.sign[LIS2DS12_AXIS_Z],
				dat[LIS2DS12_AXIS_X], dat[LIS2DS12_AXIS_Y], dat[LIS2DS12_AXIS_Z],
				obj->cvt.map[LIS2DS12_AXIS_X],obj->cvt.map[LIS2DS12_AXIS_Y],obj->cvt.map[LIS2DS12_AXIS_Z]);
			GSE_LOG("write gyro calibration data  (%5d, %5d, %5d)\n", 
				obj->cali_sw[LIS2DS12_AXIS_X],obj->cali_sw[LIS2DS12_AXIS_Y],obj->cali_sw[LIS2DS12_AXIS_Z]);
		}
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
static int LIS2DS12_ResetCalibration(struct i2c_client *client)
{
	struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);	

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;    
}

/*----------------------------------------------------------------------------*/
static int LIS2DS12_ReadCalibration(struct i2c_client *client, int dat[LIS2DS12_ACC_AXES_NUM])
{
    struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[LIS2DS12_AXIS_X]] = obj->cvt.sign[LIS2DS12_AXIS_X]*obj->cali_sw[LIS2DS12_AXIS_X];
    dat[obj->cvt.map[LIS2DS12_AXIS_Y]] = obj->cvt.sign[LIS2DS12_AXIS_Y]*obj->cali_sw[LIS2DS12_AXIS_Y];
    dat[obj->cvt.map[LIS2DS12_AXIS_Z]] = obj->cvt.sign[LIS2DS12_AXIS_Z]*obj->cali_sw[LIS2DS12_AXIS_Z];

#if DEBUG		
		if(atomic_read(&obj->trace) & ACCEL_TRC_CALI)
		{
			GSE_LOG("Read gyro calibration data  (%5d, %5d, %5d)\n", 
				dat[LIS2DS12_AXIS_X],dat[LIS2DS12_AXIS_Y],dat[LIS2DS12_AXIS_Z]);
		}
#endif

    return 0;
}

/*----------------------------------------------------------------------------*/
static int LIS2DS12_WriteCalibration(struct i2c_client *client, int dat[LIS2DS12_ACC_AXES_NUM])
{
	struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[LIS2DS12_ACC_AXES_NUM];

	GSE_FUN();
	if(!obj || !dat)
	{
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	}
	else
	{
		cali[obj->cvt.map[LIS2DS12_AXIS_X]] = obj->cvt.sign[LIS2DS12_AXIS_X]*obj->cali_sw[LIS2DS12_AXIS_X];
		cali[obj->cvt.map[LIS2DS12_AXIS_Y]] = obj->cvt.sign[LIS2DS12_AXIS_Y]*obj->cali_sw[LIS2DS12_AXIS_Y];
		cali[obj->cvt.map[LIS2DS12_AXIS_Z]] = obj->cvt.sign[LIS2DS12_AXIS_Z]*obj->cali_sw[LIS2DS12_AXIS_Z]; 
		cali[LIS2DS12_AXIS_X] += dat[LIS2DS12_AXIS_X];
		cali[LIS2DS12_AXIS_Y] += dat[LIS2DS12_AXIS_Y];
		cali[LIS2DS12_AXIS_Z] += dat[LIS2DS12_AXIS_Z];
#if DEBUG		
		if(atomic_read(&obj->trace) & ACCEL_TRC_CALI)
		{
			GSE_LOG("write gyro calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n", 
				dat[LIS2DS12_AXIS_X], dat[LIS2DS12_AXIS_Y], dat[LIS2DS12_AXIS_Z],
				cali[LIS2DS12_AXIS_X],cali[LIS2DS12_AXIS_Y],cali[LIS2DS12_AXIS_Z]);
		}
#endif
		return LIS2DS12_write_rel_calibration(obj, cali);
	} 

	return err;
}

/*----------------------------------------------------------------------------*/
static int LIS2DS12_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	databuf[0] = LIS2DS12_FIXED_DEVID;

	res = hwmsen_read_byte(client,LIS2DS12_WHO_AM_I,databuf);
    GSE_LOG(" LIS2DS12 id: 0x%02x!\n",databuf[0]);
	if(databuf[0]!=LIS2DS12_FIXED_DEVID)
	{
		return LIS2DS12_ERR_IDENTIFICATION;
	}

	if (res < 0)
	{
		return LIS2DS12_ERR_I2C;
	}
	
	return LIS2DS12_SUCCESS;
}

#ifdef LIS2DS12_EMBEDED_FUNC
static irqreturn_t lis2ds12_interrupt(int vec, void *info)
{
	struct lis2ds12_i2c_data *priv = obj_i2c_data;
	GSE_FUN();
	if(!priv)
	{
		return IRQ_HANDLED;
	}

	schedule_work(&priv->eint_work);

	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
static int lis2ds12_setup_eint(void)
{
	struct i2c_client *client = lis2ds12_i2c_client;
	int err;
	int irq;
	struct acc_hw *hw = lis2ds12_get_cust_acc_hw();
	
	if(hw->is_eint_supported == 0){
		printk("[lis2ds12]%s:eint mode is not supported.\n",__func__);
		return LIS2DS12_SUCCESS;
	}

	err = gpio_request(hw->eint_gpio, "lis2ds12_irq");
	if (err)
	{
		printk("[lis2ds12]Unable to request GPIO.\n");
		return -1;
	}
	gpio_direction_input(hw->eint_gpio);
	irq = gpio_to_irq(hw->eint_gpio);
	if (irq < 0)
	{
		printk("[lis2ds12]Unable to request gpio irq. int pin = %d, irq = %d, err=%d\n", hw->eint_gpio, irq, err);
		gpio_free(hw->eint_gpio);
		return -1;
	}

	if (request_irq(irq, lis2ds12_interrupt, IRQF_TRIGGER_FALLING, "lis2ds12", (void *)client))
	{
		printk("%s Could not allocate APDS9950_INT !\n", __func__);
		return -1;
	}

	irq_set_irq_wake(irq, 1);

	disable_irq_nosync(hw->eint_gpio);

	return LIS2DS12_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int LIS2DS12_IntCtrl(struct i2c_client *client, LIS2DS12_ACC_INT_ACTIVE_t int_act, LIS2DS12_ACC_INT_LATCH_CTL_t int_latch)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	u8 op_reg = 0;
	GSE_FUN();

	op_reg = LIS2DS12_CTRL3;
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	//config latch int or no latch
	databuf[0] &= ~LIS2DS12_ACC_INT_LATCH_CTL_MASK;//clear 
	databuf[0] |= int_latch;

	// config high or low active
	databuf[0] &= ~LIS2DS12_ACC_INT_ACTIVE_MASK;//clear 
	databuf[0] |= int_act;

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS; 
}

/*----------------------------------------------------------------------------*/
static void LIS2DS12_EintWork(struct work_struct *work)
{
	u8 databuf[2] = {0}; 
	struct lis2ds12_i2c_data *obj = obj_i2c_data;
	struct acc_hw *hw = lis2ds12_get_cust_acc_hw();

	if(obj == NULL)
	{
		GSE_ERR("obj_i2c_data is null pointer!!\n");
		goto lis2ds12_eint_work_exit;
	}	

	if(hwmsen_read_byte(obj->client, LIS2DS12_FUNC_CK_GATE, databuf))
	{
		GSE_ERR("%s read LIS2DS12_FUNC_CK_GATE register err!\n", __func__);
		goto lis2ds12_eint_work_exit;
	}

	if(atomic_read(&obj->trace) & ACCEL_TRC_DATA)
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);		
	}

#if defined(LIS2DS12_STEP_COUNTER)

	if(LIS2DS12_FUNC_CK_GATE_SIGN_M_DET_MASK & databuf[0])
	{
		//add the action when receive the significant motion
		step_notify(TYPE_SIGNIFICANT);
	}

	if(LIS2DS12_FUNC_CK_GATE_STEP_D_MASK & databuf[0])
	{
		//add the action when receive step detection interrupt
		step_notify(TYPE_STEP_DETECTOR);
	}
#endif

#if defined(LIS2DS12_TILT_FUNC)
	if(LIS2DS12_FUNC_CK_GATE_TILT_INT_MASK & databuf[0])
	{
		//add the action when receive the tilt interrupt
		tilt_notify();
	}
#endif

lis2ds12_eint_work_exit:
	enable_irq(hw->eint_gpio);
}

/*----------------------------------------------------------------------------*/
#if defined(LIS2DS12_TILT_FUNC) //tilt detector
static struct tilt_init_info  lis2ds12_tilt_init_info;

static int LIS2DS12_Enable_Tilt_Func_On_Int(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	u8 op_reg = 0;
	struct acc_hw *hw = lis2ds12_get_cust_acc_hw();
	
	if(hw->is_eint_supported == 0){
		printk("[lis2ds12]%s:eint mode is not supported.\n",__func__);
		return LIS2DS12_SUCCESS;
	}

	op_reg = LIS2DS12_CTRL5;
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}
	
	if(enable)
	{
		databuf[0] &= ~LIS2DS12_FUNC_CTRL_TILT_MASK;//clear
		databuf[0] |= LIS2DS12_FUNC_CTRL_TILT_MASK;
	}
	else
	{
		databuf[0] &= ~LIS2DS12_FUNC_CTRL_TILT_MASK;//clear
	}

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LIS2DS12_ERR_I2C;
	}

	res = LIS2DS12_IntCtrl(client, LIS2DS12_ACC_INT_ACTIVE_LOW, LIS2DS12_ACC_INT_LATCH);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS; 
}

static int LIS2DS12_Enable_Tilt_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();
	
	if(hwmsen_read_byte(client, LIS2DS12_FUNC_CTRL, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if(enable)
	{
		databuf[0] &= ~LIS2DS12_FUNC_CTRL_TILT_MASK;//clear 
		databuf[0] |= LIS2DS12_FUNC_CTRL_TILT_MASK;			
	}
	else
	{
		databuf[0] &= ~LIS2DS12_FUNC_CTRL_TILT_MASK;//clear 		
	}

	databuf[1] = databuf[0];
	databuf[0] = LIS2DS12_FUNC_CTRL; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable tilt func register err!\n");
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS; 
}

static int LIS2DS12_enable_tilt(struct i2c_client *client, bool enable)
{
	int res = 0;
	struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);//obj_i2c_data;
	struct acc_hw *hw = lis2ds12_get_cust_acc_hw();

	GSE_FUN();
	if(enable) {
		res = LIS2DS12_SetSampleRate(client, obj->sample_rate);
		if(LIS2DS12_SUCCESS == res)
		{
			GSE_LOG(" %s set %dhz odr to acc\n", __func__, obj->sample_rate);
		}

		res = LIS2DS12_Enable_Tilt_Func(client, true);
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Enable_Tilt_Func failed!\n");
			return LIS2DS12_ERR_STATUS;
		}

		res = LIS2DS12_Enable_Tilt_Func_On_Int(client, true);  //default route to INT1 	
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Enable_Tilt_Func_On_Int failed!\n");
			return LIS2DS12_ERR_STATUS;
		}
		enable_irq(hw->eint_gpio);
	}
	else
	{
		res = LIS2DS12_Enable_Tilt_Func(client, false);
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Enable_Tilt_Func failed!\n");
			return LIS2DS12_ERR_STATUS;
		}

		res = LIS2DS12_Enable_Tilt_Func_On_Int(client, false);  //default route to INT2
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Enable_Tilt_Func_On_Int failed!\n");
			return LIS2DS12_ERR_STATUS;
		}

		if(!enable_status && !pedo_enable_status)
		{
			res = LIS2DS12_SetPowerMode(client, false);
			if(res != LIS2DS12_SUCCESS)
			{
				GSE_LOG(" LIS2DS12_SetPowerMode failed!\n");
				return LIS2DS12_ERR_STATUS;
			}
		}

		disable_irq_nosync(hw->eint_gpio);
	}

	return LIS2DS12_SUCCESS;
}

static int lis2ds12_tilt_open_report_data(int open)
{
	int res = 0;
	struct lis2ds12_i2c_data *priv = obj_i2c_data;
	
	GSE_FUN();
	if(1 == open)
	{
		tilt_enable_status = true;
		res = LIS2DS12_enable_tilt(priv->client, true);
		if(LIS2DS12_SUCCESS != res)
		{
			GSE_ERR("%s run LIS2DS12_enable_tilt to true failed!\n", __func__);
		}
	}
	else if(0 == open)
	{
		tilt_enable_status = false;
		res = LIS2DS12_enable_tilt(priv->client, false);
		if(LIS2DS12_SUCCESS != res)
		{
			GSE_ERR("%s run LIS2DS12_enable_tilt to false failed!\n", __func__);
		}
	}
	
	return res;
}

static int lis2ds12_tilt_get_data(u16 *value, int *status)
{
	return 0;
}

static int lis2ds12_tilt_local_init(void)
{
	int res = 0;

	struct tilt_control_path tilt_ctl={0};
	struct tilt_data_path tilt_data={0};

	mutex_lock(&lis2ds12_init_mutex);
	set_bit(LIS2DS12_TILT, &lis2ds12_init_flag_test);

	if((0==test_bit(LIS2DS12_ACC, &lis2ds12_init_flag_test)) \
		&& (0==test_bit(LIS2DS12_STEP_C, &lis2ds12_init_flag_test)))
	{
		res = lis2ds12_local_init_common();
		if(res < 0)
		{
			goto lis2ds12_tilt_local_init_failed;
		}
	}

	if(lis2ds12_acc_init_flag == -1)
	{
		mutex_unlock(&lis2ds12_init_mutex);
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	}
	else
	{
		tilt_ctl.open_report_data= lis2ds12_tilt_open_report_data;	
		res = tilt_register_control_path(&tilt_ctl);

		tilt_data.get_data = lis2ds12_tilt_get_data;
		res = tilt_register_data_path(&tilt_data);
	}
	mutex_unlock(&lis2ds12_init_mutex);
	return 0;

lis2ds12_tilt_local_init_failed:
	mutex_unlock(&lis2ds12_init_mutex);
	GSE_ERR("%s init failed!\n", __FUNCTION__);
	return -1;
}

static int lis2ds12_tilt_local_uninit(void)
{
	clear_bit(LIS2DS12_TILT, &lis2ds12_init_flag_test);
    return 0;
}

static struct tilt_init_info  lis2ds12_tilt_init_info = {
	.name   = "LIS2DS12_TILT",
	.init   = lis2ds12_tilt_local_init,
	.uninit = lis2ds12_tilt_local_uninit,
};
#endif

/*----------------------------------------------------------------------------*/
#if defined(LIS2DS12_STEP_COUNTER) //step counter
static struct step_c_init_info  lis2ds12_step_c_init_info;

static ssize_t show_debounce_step_num(struct device_driver *ddri, char *buf)
{
	struct lis2ds12_i2c_data *data = obj_i2c_data;
	u8 databuf[2] = {0};
	int num = -1;
	int res = 0;
	u8 CTRL2_INIT = 0;

	if(NULL == data)
	{
		GSE_LOG("%s lis2ds12_i2c_data is null!!\n", __func__);
		return -1;
	}

	if(hwmsen_read_byte(data->client, LIS2DS12_CTRL2, databuf)){
		GSE_ERR("%s read LIS2DS12_CTRL2 register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}else{
		GSE_LOG("%s read LIS2DS12_CTRL2 register: 0x%02x\n", __func__, databuf[0]);
	}
	CTRL2_INIT = databuf[0];

	// enable embedded register
	databuf[0] = LIS2DS12_CTRL2;
	databuf[1] = CTRL2_INIT|LIS2DS12_ACCESS_EMBED_REG_MASK;
	res = i2c_master_send(data->client, databuf, 0x02);
	if(res <= 0){
		GSE_ERR("%s write LIS2DS12_CTRL2 register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}

	// read embedded register
	if(hwmsen_read_byte(data->client, LIS2DS12_STEP_COUNTER_DEBOUNCE, databuf)){
		GSE_ERR("%s read LIS2DS12_STEP_COUNTER_DEBOUNCE register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}else{
		GSE_LOG("%s read LIS2DS12_STEP_COUNTER_DEBOUNCE register ok: 0x%02x\n", __func__, databuf[0]);
	}

	databuf[0] &= 0x07;
	num = (int)databuf[0];

	databuf[0] = LIS2DS12_FUNC_CTRL;
	databuf[1] = CTRL2_INIT;
	res = i2c_master_send(data->client, databuf, 0x02);
	if(res <= 0){
		GSE_ERR("%s write LIS2DS12_CTRL2 register err!\n", __func__);
			return LIS2DS12_ERR_I2C;
	}

	return sprintf(buf, "%d\n",num);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_debounce_step_num(struct device_driver *ddri, const char *buf, size_t count)
{
	int num = 0;
	struct lis2ds12_i2c_data *data = obj_i2c_data;
	u8 databuf[2] = {0};
	int res = 0;
	u8 CTRL2_INIT = 0;
	GSE_FUN();

	if(NULL == data)
	{
		GSE_LOG("%s lis2ds12_i2c_data is null!!\n", __func__);
		return count;
	}

	sscanf(buf, "%d", &num);
	GSE_LOG("%s try to set debounce_step_num:%d\n", __func__, num);

	if((num >= 0) && (num <= 7))
	{
		// store CTRL2 register
		if(hwmsen_read_byte(data->client, LIS2DS12_CTRL2, databuf)){
			GSE_ERR("%s read LIS2DS12_CTRL2 register err!\n", __func__);
			goto store_debounce_step_num_fail;
		}else{
			GSE_LOG("%s read LIS2DS12_CTRL2 register: 0x%02x\n", __func__, databuf[0]);
		}
		CTRL2_INIT = databuf[0];

		// enable embedded register
		databuf[0] = LIS2DS12_CTRL2;
		databuf[1] = CTRL2_INIT|LIS2DS12_ACCESS_EMBED_REG_MASK;
		res = i2c_master_send(data->client, databuf, 0x02);
		if(res <= 0){
			GSE_ERR("%s write LIS2DS12_CTRL2 register err!\n", __func__);
			goto store_debounce_step_num_fail;
		}

		// read embedded register
		if(hwmsen_read_byte(data->client, LIS2DS12_STEP_COUNTER_DEBOUNCE, databuf)){
			GSE_ERR("%s read LIS2DS12_STEP_COUNTER_DEBOUNCE register err!\n", __func__);
			goto store_debounce_step_num_fail;
		}
		else
		{
			GSE_LOG("%s read LIS2DS12_STEP_COUNTER_DEBOUNCE register: 0x%02x\n", __func__, databuf[0]);
		}

		databuf[0] &= 0xf8;
		databuf[0] |= (u8)num;

		// write embedded register
		databuf[1] = databuf[0];
		databuf[0] = LIS2DS12_STEP_COUNTER_DEBOUNCE;
		res = i2c_master_send(data->client, databuf, 0x02);
		if(res <= 0){
			GSE_ERR("%s write LIS2DS12_STEP_COUNTER_DEBOUNCE register err!\n", __func__);
			goto store_debounce_step_num_fail;
		}else{
			GSE_LOG("%s write LIS2DS12_STEP_COUNTER_DEBOUNCE register ok: 0x%02x\n", __func__, databuf[1]);
		}

		// disable embedded register
		databuf[0] = LIS2DS12_FUNC_CTRL;
		databuf[1] = CTRL2_INIT;
		res = i2c_master_send(data->client, databuf, 0x02);
		if(res <= 0)
		{
			GSE_ERR("%s write LIS2DS12_CTRL2 register err!\n", __func__);
			goto store_debounce_step_num_fail;
		}
	}else{
		GSE_ERR("%s invalid value = '%s'\n", __func__, buf);
	}
store_debounce_step_num_fail:
	return count;
}

static ssize_t show_threshold_debounce(struct device_driver *ddri, char *buf)
{
	struct lis2ds12_i2c_data *data = obj_i2c_data;
	u8 databuf[2] = {0};
	u8 debounce = 0;
	int res = 0;
	u8 CTRL2_INIT = 0;
	u8 threshold = 0;

	GSE_FUN();

	if(NULL == data){
		GSE_LOG("%s lis2ds12_i2c_data is null!!\n", __func__);
		return -1;
	}

	if(hwmsen_read_byte(data->client, LIS2DS12_STEP_COUNTER_MINTHS, databuf)){
		GSE_ERR("%s read LIS2DS12_STEP_COUNTER_MINTHS register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}else{
		GSE_LOG("%s read LIS2DS12_STEP_COUNTER_MINTHS register: 0x%x\n", __func__, databuf[0]);
	}
	threshold = databuf[0];

	if(hwmsen_read_byte(data->client, LIS2DS12_CTRL2, databuf)){
		GSE_ERR("%s read LIS2DS12_CTRL2 register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}else{
		GSE_LOG("%s read LIS2DS12_CTRL2 register: 0x%02x\n", __func__, databuf[0]);
	}
	CTRL2_INIT = databuf[0];

	// enable embedded register
	databuf[0] = LIS2DS12_CTRL2;
	databuf[1] = CTRL2_INIT|LIS2DS12_ACCESS_EMBED_REG_MASK;
	res = i2c_master_send(data->client, databuf, 0x02);
	if(res <= 0){
		GSE_ERR("%s write LIS2DS12_CTRL2 register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}

	// read embedded register
	if(hwmsen_read_byte(data->client, LIS2DS12_STEP_COUNTER_DEBOUNCE, databuf)){
		GSE_ERR("%s read LIS2DS12_STEP_COUNTER_DEBOUNCE register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}else{
		GSE_LOG("%s read LIS2DS12_STEP_COUNTER_DEBOUNCE register ok: 0x%02x\n", __func__, databuf[0]);
	}

	debounce = databuf[0];

	databuf[0] = LIS2DS12_FUNC_CTRL;
	databuf[1] = CTRL2_INIT;
	res = i2c_master_send(data->client, databuf, 0x02);
	if(res <= 0){
		GSE_ERR("%s write LIS2DS12_CTRL2 register err!\n", __func__);
			return LIS2DS12_ERR_I2C;
	}

	return sprintf(buf, "threshold: 0x%02x debounce: 0x%02x\n",threshold,debounce);
}
/*----------------------------------------------------------------------------*/
static int LIS2DS12_Write_PedoParameters(struct i2c_client *client, u8 pedo4g, u8 threshold, u8 debounce);

static ssize_t store_threshold_debounce(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int debounce = 0;
	unsigned int threshold = 0;
	struct lis2ds12_i2c_data *data = obj_i2c_data;
	int res = 0;
	GSE_FUN();

	if(NULL == data)
	{
		GSE_LOG("%s lis2ds12_i2c_data is null!!\n", __func__);
		return count;
	}

	sscanf(buf, "%x %x", &threshold, &debounce);
	GSE_LOG("%s: try to set threshold:0x%02x debounce:0x%02x\n", __func__, (u8)threshold, (u8)debounce);

	res = LIS2DS12_Write_PedoParameters(data->client, 1, threshold, debounce);// set threshold to a certain value here
	if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Write_PedoParameters failed!\n");
			goto store_threshold_debounce_fail;
		}
store_threshold_debounce_fail:
	return count;
}

static DRIVER_ATTR(debounce_step_num,      S_IRUGO | S_IWUGO, show_debounce_step_num, store_debounce_step_num);
static DRIVER_ATTR(threshold_debounce,      S_IRUGO | S_IWUGO, show_threshold_debounce, store_threshold_debounce);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *lis2ds12_step_c_attr_list[] = {
	&driver_attr_debounce_step_num,     /*debounce step num*/
	&driver_attr_threshold_debounce,     /*debounce step num*/
};
/*----------------------------------------------------------------------------*/
static int lis2ds12_step_c_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(lis2ds12_step_c_attr_list)/sizeof(lis2ds12_step_c_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver,  lis2ds12_step_c_attr_list[idx])))
		{
			GSE_ERR("driver_create_file (%s) = %d\n",  lis2ds12_step_c_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int lis2ds12_step_c_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof( lis2ds12_step_c_attr_list)/sizeof( lis2ds12_step_c_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver,  lis2ds12_step_c_attr_list[idx]);
	}
	return err;
}

static int LIS2DS12_Enable_Pedometer_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();

	if(hwmsen_read_byte(client, LIS2DS12_FUNC_CTRL, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if(enable)
	{
		databuf[0] &= ~LIS2DS12_FUNC_CTRL_STEP_CNT_MASK;//clear 
		databuf[0] |= LIS2DS12_FUNC_CTRL_STEP_CNT_MASK;
	}
	else
	{
		databuf[0] &= ~LIS2DS12_FUNC_CTRL_STEP_CNT_MASK;//clear 
	}

	databuf[1] = databuf[0];
	databuf[0] = LIS2DS12_FUNC_CTRL;
	res = i2c_master_send(client, databuf, 0x02);
	if(res < 0)
	{
		GSE_ERR("write enable pedometer func register err!\n");
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS;    
}

static int LIS2DS12_Enable_Pedometer_Func_On_Int(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0;
	u8 op_reg = 0;
	struct acc_hw *hw = lis2ds12_get_cust_acc_hw();
	
	if(hw->is_eint_supported == 0){
		printk("[lis2ds12]%s:eint mode is not supported.\n",__func__);
		return LIS2DS12_SUCCESS;
	}

	op_reg = LIS2DS12_CTRL5;
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read data format register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if(enable)
	{
		databuf[0] &= ~LIS2DS12_INT2_STEP_DET_MASK;//clear
		databuf[0] |= LIS2DS12_INT2_STEP_DET_MASK;
	}
	else
	{
		databuf[0] &= ~LIS2DS12_INT2_STEP_DET_MASK;//clear
	}

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable step detector func register err!\n");
		return LIS2DS12_ERR_I2C;
	}

	res = LIS2DS12_IntCtrl(client, LIS2DS12_ACC_INT_ACTIVE_LOW, LIS2DS12_ACC_INT_LATCH);
	if(res < 0)
	{
		GSE_ERR("write enable step detector func register err!\n");
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS;
}


static int LIS2DS12_Write_PedoParameters(struct i2c_client *client, u8 pedo4g, u8 threshold, u8 debounce)
{
	u8 databuf[2] = {0};
	int res = 0;
	u8 CTRL2_INIT = 0;
	GSE_FUN();

	// read step counter threshold value
	if(hwmsen_read_byte(client, LIS2DS12_STEP_COUNTER_MINTHS, databuf))
	{
		GSE_ERR("%s read LIS2DS12_STEP_COUNTER_MINTHS register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read LIS2DS12_STEP_COUNTER_MINTHS register: 0x%x\n", __func__, databuf[0]);
	}

	// setup Pedo 4g mode, need to set the full-scale to 4g.
	if (pedo4g)
	{
		databuf[0] &= ~LIS2DS12_STEP_CNT_4G_MASK; // clear
		databuf[0] |= LIS2DS12_STEP_CNT_4G_MASK; // 4g mode, 1LSB = 32mg
	}
	else
	{
		databuf[0] &= ~LIS2DS12_STEP_CNT_4G_MASK; // 2g mode, 1LSB = 16mg
	}

	// write the new threshold value
	databuf[0] &= ~LIS2DS12_STEP_CNT_THRESHOLD_MASK; // clear
	databuf[0] |= (threshold & LIS2DS12_STEP_CNT_THRESHOLD_MASK);

	databuf[1] = databuf[0];
	databuf[0] = LIS2DS12_STEP_COUNTER_MINTHS;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LIS2DS12_STEP_COUNTER_MINTHS register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}

	// store CTRL2 register
	if(hwmsen_read_byte(client, LIS2DS12_CTRL2, databuf))
	{
		GSE_ERR("%s read LIS2DS12_CTRL2 register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	CTRL2_INIT = databuf[0];

	// enable embedded register
	databuf[0] = LIS2DS12_CTRL2;
	databuf[1] = CTRL2_INIT|LIS2DS12_ACCESS_EMBED_REG_MASK;
	res = i2c_master_send(client, databuf, 0x02);
	if(res <= 0)
	{
		GSE_ERR("%s write LIS2DS12_CTRL2 register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}

	// read embedded register
	if(hwmsen_read_byte(client, LIS2DS12_STEP_COUNTER_DEBOUNCE, databuf))
	{
		GSE_ERR("%s read LIS2DS12_STEP_COUNTER_DEBOUNCE register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}

	databuf[0] &= 0x00;//~LIS2DS12_STEP_CNT_DEBOUNCE_MASK; // clear
	databuf[0] |= (debounce & LIS2DS12_STEP_CNT_DEBOUNCE_MASK);

	// write embedded register
	databuf[1] = databuf[0];
	databuf[0] = LIS2DS12_STEP_COUNTER_DEBOUNCE;
	res = i2c_master_send(client, databuf, 0x02);
	if(res <= 0)
	{
		GSE_ERR("%s write LIS2DS12_STEP_COUNTER_DEBOUNCE register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}

	// disable embedded register
	databuf[0] = LIS2DS12_FUNC_CTRL;
	databuf[1] = CTRL2_INIT;
	res = i2c_master_send(client, databuf, 0x02);
	if(res <= 0)
	{
		GSE_ERR("%s write LIS2DS12_CTRL2 register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS; 
}

static int LIS2DS12_Reset_Pedo_Data(struct i2c_client *client)
{
	u8 databuf[2] = {0};
	int res = 0;
	GSE_FUN();

	if(hwmsen_read_byte(client, LIS2DS12_STEP_COUNTER_MINTHS, databuf))
	{
		GSE_ERR("%s read LIS2DS12_STEP_COUNTER_MINTHS register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc LIS2DS12_STEP_COUNTER_MINTHS data format register: 0x%x\n", __func__, databuf[0]);
	}

	databuf[0] &= ~LIS2DS12_STEP_CNT_RST_MASK;//clear 
	databuf[0] |= LIS2DS12_STEP_CNT_RST_MASK;

	databuf[1] = databuf[0];
	databuf[0] = LIS2DS12_STEP_COUNTER_MINTHS;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LIS2DS12_STEP_COUNTER_MINTHS register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS;
}

static int LIS2DS12_enable_pedo(struct i2c_client *client, bool enable)
{
	int res = 0;
	struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);

	if(true == enable)
	{
		res = LIS2DS12_SetSampleRate(client, obj->sample_rate);
		if(LIS2DS12_SUCCESS == res)
		{
			GSE_LOG(" %s set %dhz odr to acc\n", __func__, obj->sample_rate);
		}
		//enable tilt feature and pedometer feature
		res = LIS2DS12_Enable_Pedometer_Func(client, true);
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Enable_Pedometer_Func failed!\n");
			return LIS2DS12_ERR_STATUS;
		}
		res = LIS2DS12_Enable_Pedometer_Func_On_Int(client, true);
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Enable_Pedometer_Func_On_Int failed!\n");
			return LIS2DS12_ERR_STATUS;
		}
		#if 0
		res = LIS2DS12_Write_PedoParameters(client, 1, 0x0e, 0x5f);// set threshold to a certain value here
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Write_PedoParameters failed!\n");
			return LIS2DS12_ERR_STATUS;
		}
		#endif
		/*res = LIS2DS12_Reset_Pedo_Data(client);
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Reset_Pedo_Data failed!\n");
			return LIS2DS12_ERR_STATUS;
		}*/
	}
	else
	{
		res = LIS2DS12_Enable_Pedometer_Func(client, false);
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Enable_Pedometer_Func failed at disable pedo!\n");
			return LIS2DS12_ERR_STATUS;
		}

		res = LIS2DS12_Enable_Pedometer_Func_On_Int(client, false);
		if(res != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Enable_Pedometer_Func_On_Int failed!\n");
			return LIS2DS12_ERR_STATUS;
		}

		//do not turn off the func
		if(!enable_status && !tilt_enable_status)
		{
			res = LIS2DS12_SetPowerMode(client, false);
			if(res != LIS2DS12_SUCCESS)
			{
				GSE_LOG(" LIS2DS12_SetPowerMode failed at disable pedo!\n");
				return LIS2DS12_ERR_STATUS;
			}
		}
	}

	return LIS2DS12_SUCCESS;
}

static int LIS2DS12_Get_Pedo_DataReg(struct i2c_client *client, u16 *Value)
{
	u8 databuf[2] = {0};

	if(hwmsen_read_block(client, LIS2DS12_STEP_COUNTER_L, databuf, 2))
	{
		GSE_ERR("LIS2DS12 read acc data  error\n");
		return -2;
	}

	*Value = (databuf[1]<<8)|databuf[0];	
	GSE_ERR("::step num = %d\n",*Value);
	return LIS2DS12_SUCCESS;
}

static int lis2ds12_step_c_open_report_data(int open)
{
	return LIS2DS12_SUCCESS;
}

static int lis2ds12_step_c_enable_nodata(int en)
{
	int res =0;
	int value = en;
	int err = 0;
	struct lis2ds12_i2c_data *priv = obj_i2c_data;
	GSE_FUN();

	if(priv == NULL)
	{
		GSE_ERR("%s obj_i2c_data is NULL!\n", __func__);
		return -1;
	}

	if(value == 1)
	{
		pedo_enable_status = true;
	#ifdef VENDOR_EDIT
		step_tune.new_enable = 1;
	#endif//VENDOR_EDIT
		res = LIS2DS12_enable_pedo(priv->client, true);
		if(LIS2DS12_SUCCESS != res)
		{
			GSE_LOG("LIS2DS12_enable_pedo failed at open action!\n");
			return res;
		}
	}
	else
	{
		pedo_enable_status = false;
		res = LIS2DS12_enable_pedo(priv->client, false);
		if(LIS2DS12_SUCCESS != res)
		{
			GSE_LOG("LIS2DS12_enable_pedo failed at close action!\n");
			return res;
		}
	}

	GSE_LOG("lis2ds12_step_c_enable_nodata OK!\n");
    return err;
}

static int lis2ds12_step_c_enable_step_detect(int en)
{
	return lis2ds12_step_c_enable_nodata(en);
}

static int lis2ds12_step_c_set_delay(u64 delay)
{
	return 0;
}

static int lis2ds12_step_c_get_data(u64 *value, int *status)
{
	int err = 0;
	u16 pedo_data = 0;

	struct lis2ds12_i2c_data *priv = obj_i2c_data;
	err = LIS2DS12_Get_Pedo_DataReg(priv->client, &pedo_data);

	*value = (u64)pedo_data + step_num;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

#ifndef VENDOR_EDIT
	printk(KERN_EMERG"zhye::pre_value=%llu  new_value=%llu\n",step_tune.pre_value,*value);
	if ((step_tune.new_enable == 1) && (step_tune.pre_value > 0) && (step_tune.pre_value == *value))
	{
		*value = step_tune.pre_value - 1;
	}
	step_tune.new_enable = 0;
	
	if (step_tune.pre_value != *value)
		step_tune.pre_value = *value;
#endif//VENDOR_EDIT

	if(pedo_data >= 60000){
		err = LIS2DS12_Reset_Pedo_Data(priv->client);
		if(err != LIS2DS12_SUCCESS)
		{
			GSE_LOG(" LIS2DS12_Reset_Pedo_Data failed!\n");
			return LIS2DS12_ERR_STATUS;
		}
		step_num += pedo_data;
	}

	return err;
}

static int lis2ds12_step_c_get_data_step_d(u64 *value, int *status)
{
	return 0;
}

static int lis2ds12_step_c_get_data_significant(u64 *value, int *status)
{
	return 0;
}

static int LIS2DS12_Enable_SigMotion_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();
	
	if(hwmsen_read_byte(client, LIS2DS12_FUNC_CTRL, databuf))
	{
		GSE_ERR("%s read LIS2DS12_FUNC_CTRL register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}

	if(enable)
	{
		databuf[0] &= ~LIS2DS12_FUNC_CTRL_SIGN_MOT_MASK;//clear 
		databuf[0] |= LIS2DS12_FUNC_CTRL_SIGN_MOT_MASK;
	}
	else
	{
		databuf[0] &= ~LIS2DS12_FUNC_CTRL_SIGN_MOT_MASK;//clear 		
	}

	databuf[1] = databuf[0];
	databuf[0] = LIS2DS12_FUNC_CTRL; 	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("%s write LIS2DS12_FUNC_CTRL register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS;    
}

static int LIS2DS12_Enable_SigMotion_Func_On_Int(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	u8 op_reg = 0;
	struct acc_hw *hw = lis2ds12_get_cust_acc_hw();
	
	if(hw->is_eint_supported == 0){
		printk("[lis2ds12]%s:eint mode is not supported.\n",__func__);
		return LIS2DS12_SUCCESS;
	}

	// Config interrupt for significant motion
	op_reg = LIS2DS12_CTRL5;
	if(hwmsen_read_byte(client, op_reg, databuf))
	{
		GSE_ERR("%s read LIS2DS12_CTRL5 register err!\n", __func__);
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc LIS2DS12_CTRL5 register: 0x%x\n", databuf[0]);
	}

	if(enable)
	{
		databuf[0] &= ~LIS2DS12_INT2_SIG_MOT_DET_MASK;//clear 
		databuf[0] |= LIS2DS12_INT2_SIG_MOT_DET_MASK;			
	}
	else
	{
		databuf[0] &= ~LIS2DS12_INT2_SIG_MOT_DET_MASK;//clear 
	}

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if(res < 0)
	{
		GSE_ERR("write enable sig_mot INT err!\n");
		return LIS2DS12_ERR_I2C;
	}

	res = LIS2DS12_IntCtrl(client, LIS2DS12_ACC_INT_ACTIVE_LOW, LIS2DS12_ACC_INT_LATCH);
	if(res < 0)
	{
		GSE_ERR("write enable sig_mot active low err!\n");
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS; 
}


static int lis2ds12_step_c_enable_significant(int en)
{
	int res =0;
	struct lis2ds12_i2c_data *priv = obj_i2c_data;
	struct acc_hw *hw = lis2ds12_get_cust_acc_hw();

	if(en)
	{
		pedo_enable_status = true;
		res = LIS2DS12_SetSampleRate(priv->client, priv->sample_rate);
		if(LIS2DS12_SUCCESS != res)
		{
			GSE_ERR("%s run LIS2DS12_SetSampleRate to fail!\n", __func__);
			return LIS2DS12_ERR_STATUS;
		}
		res = LIS2DS12_Enable_SigMotion_Func(priv->client, true); //default route to INT2
		if(LIS2DS12_SUCCESS != res)
		{
			GSE_ERR("%s run LIS2DS12_Enable_SigMotion_Func to fail!\n", __func__);
			return LIS2DS12_ERR_STATUS;
		}
		res = LIS2DS12_Enable_SigMotion_Func_On_Int(priv->client, true); //default route to INT2
		if(LIS2DS12_SUCCESS != res)
		{
			GSE_ERR("%s run LIS2DS12_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
			return LIS2DS12_ERR_STATUS;
		}

		enable_irq(hw->eint_gpio);
	}
	else
	{
		pedo_enable_status = false;
		res = LIS2DS12_Enable_SigMotion_Func(priv->client, false);
		if(LIS2DS12_SUCCESS != res)
		{
			GSE_ERR("%s run LIS2DS12_Enable_SigMotion_Func to fail!\n", __func__);
			return LIS2DS12_ERR_STATUS;
		}
		res = LIS2DS12_Enable_SigMotion_Func_On_Int(priv->client, false);
		if(LIS2DS12_SUCCESS != res)
		{
			GSE_ERR("%s run LIS2DS12_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
			return LIS2DS12_ERR_STATUS;
		}
		if(!enable_status && !tilt_enable_status)
		{
			res = LIS2DS12_SetPowerMode(priv->client, false);
			if(LIS2DS12_SUCCESS != res)
			{
				GSE_ERR("%s run LIS2DS12_SetPowerMode to fail!\n", __func__);
				return LIS2DS12_ERR_STATUS;
			}
		}

		disable_irq_nosync(hw->eint_gpio);
	}

	return res;
}

static int lis2ds12_step_c_local_init(void)
{
	int res = 0;

	struct step_c_control_path step_ctl={0};
	struct step_c_data_path step_data={0};	
	
	mutex_lock(&lis2ds12_init_mutex);
		
	set_bit(LIS2DS12_STEP_C, &lis2ds12_init_flag_test);
	
	if((0==test_bit(LIS2DS12_ACC, &lis2ds12_init_flag_test)) \
		&& (0 == test_bit(LIS2DS12_TILT, &lis2ds12_init_flag_test)))
	{
		res = lis2ds12_local_init_common();
		if(res < 0)
		{
			goto lis2ds12_step_c_local_init_failed;
		}
			
	}

	if(lis2ds12_acc_init_flag == -1)
	{
		mutex_unlock(&lis2ds12_init_mutex);
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	}
	else
	{
		res = lis2ds12_step_c_create_attr(&(lis2ds12_step_c_init_info.platform_diver_addr->driver));
		if(res)
		{
			 GSE_ERR("lis2ds12_step_c_create_attr err\n");
			goto lis2ds12_step_c_creat_attr_failed;
		}
		step_ctl.open_report_data= lis2ds12_step_c_open_report_data;
		step_ctl.enable_nodata = lis2ds12_step_c_enable_nodata;
		step_ctl.enable_step_detect  = lis2ds12_step_c_enable_step_detect;
		step_ctl.set_delay = lis2ds12_step_c_set_delay;
		step_ctl.is_report_input_direct = false;
		step_ctl.is_support_batch = false;
		step_ctl.enable_significant = lis2ds12_step_c_enable_significant;

		res = step_c_register_control_path(&step_ctl);
		if(res)
		{
			 GSE_ERR("register step counter control path err\n");
			goto lis2ds12_step_c_local_init_failed;
		}

		step_data.get_data = lis2ds12_step_c_get_data;
		step_data.get_data_step_d = lis2ds12_step_c_get_data_step_d;
		step_data.get_data_significant = lis2ds12_step_c_get_data_significant;

		step_data.vender_div = 1;
		res = step_c_register_data_path(&step_data);
		if(res)
		{
			GSE_ERR("register step counter data path err= %d\n", res);
			goto lis2ds12_step_c_local_init_failed;
		}
	}
	mutex_unlock(&lis2ds12_init_mutex);
	return 0;
	
lis2ds12_step_c_local_init_failed:
	lis2ds12_step_c_delete_attr(&(lis2ds12_step_c_init_info.platform_diver_addr->driver));
lis2ds12_step_c_creat_attr_failed:
	mutex_unlock(&lis2ds12_init_mutex);
	GSE_ERR("%s init failed!\n", __FUNCTION__);
	return res;

}
static int lis2ds12_step_c_local_uninit(void)
{
	clear_bit(LIS2DS12_STEP_C, &lis2ds12_init_flag_test);
    return 0;
}

static struct step_c_init_info  lis2ds12_step_c_init_info = {
	.name   = "LIS2DS12_STEP_C",
	.init   = lis2ds12_step_c_local_init,
	.uninit = lis2ds12_step_c_local_uninit,
};
#endif

#endif //LIS2DS12_EMBEDED_FUNC

static int LIS2DS12_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);//obj_i2c_data;

	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return LIS2DS12_SUCCESS;
	}

	if(hwmsen_read_byte(client, LIS2DS12_CTRL1, databuf))
	{
		GSE_ERR("read lis2ds12 power ctl register err!\n");
		return LIS2DS12_ERR_I2C;
	}
	GSE_LOG("LIS2DS12_CTRL1:databuf[0] =  %x!\n", databuf[0]);


	if(true == enable)
	{
		databuf[0] &= ~LIS2DS12_ACC_ODR_MASK;//clear lis2ds12 gyro ODR bits
		databuf[0] |= obj->sample_rate;//LIS2DS12_ACC_ODR_100HZ; //default set 100HZ for LIS2DS12 acc
	}
	else
	{
		// do nothing
		databuf[0] &= ~LIS2DS12_ACC_ODR_MASK;//clear lis2ds12 acc ODR bits
		databuf[0] |= LIS2DS12_ACC_ODR_POWER_DOWN;
	}
	databuf[1] = databuf[0];
	databuf[0] = LIS2DS12_CTRL1;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_LOG("LIS2DS12 set power mode: ODR 100hz failed!\n");
		return LIS2DS12_ERR_I2C;
	}	
	else
	{
		GSE_LOG("set LIS2DS12 gyro power mode:ODR 100HZ ok %d!\n", enable);
	}	

	sensor_power = enable;

	return LIS2DS12_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int LIS2DS12_SetFullScale(struct i2c_client *client, u8 acc_fs)
{
	u8 databuf[2] = {0};    
	int res = 0;
	struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);
	
	GSE_FUN(); 
		
	if(hwmsen_read_byte(client, LIS2DS12_CTRL1, databuf))
	{
		GSE_ERR("read LIS2DS12_CTRL1 err!\n");
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LIS2DS12_CTRL1 register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LIS2DS12_ACC_RANGE_MASK;//clear 
	databuf[0] |= acc_fs;
	
	databuf[1] = databuf[0];
	databuf[0] = LIS2DS12_CTRL1; 
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write full scale register err!\n");
		return LIS2DS12_ERR_I2C;
	}
	switch(acc_fs)
	{
		case LIS2DS12_ACC_RANGE_2g:
			obj->sensitivity = LIS2DS12_ACC_SENSITIVITY_2G;
			break;
		case LIS2DS12_ACC_RANGE_4g:
			obj->sensitivity = LIS2DS12_ACC_SENSITIVITY_4G;
			break;
		case LIS2DS12_ACC_RANGE_8g:
			obj->sensitivity = LIS2DS12_ACC_SENSITIVITY_8G;
			break;
		case LIS2DS12_ACC_RANGE_16g:
			obj->sensitivity = LIS2DS12_ACC_SENSITIVITY_16G;
			break;
		default:
			obj->sensitivity = LIS2DS12_ACC_SENSITIVITY_2G;
			break;
	}

	return LIS2DS12_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
// set the acc sample rate
static int LIS2DS12_SetSampleRate(struct i2c_client *client, u8 sample_rate)
{
	u8 databuf[2] = {0}; 
	int res = 0;
	GSE_FUN();

	res = LIS2DS12_SetPowerMode(client, true);	//set Sample Rate will enable power and should changed power status
	if(res != LIS2DS12_SUCCESS)	
	{
		return res;
	}

	if(hwmsen_read_byte(client, LIS2DS12_CTRL1, databuf))
	{
		GSE_ERR("read acc data format register err!\n");
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LIS2DS12_ACC_ODR_MASK;//clear 
	databuf[0] |= sample_rate;
	
	databuf[1] = databuf[0];
	databuf[0] = LIS2DS12_CTRL1; 
		
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("write sample rate register err!\n");
		return LIS2DS12_ERR_I2C;
	}

	return LIS2DS12_SUCCESS;    
}

static int LIS2DS12_ReadRawData(struct i2c_client *client, s16 data[LIS2DS12_ACC_AXES_NUM])
{
	int err = 0;
	u8 databuf[LIS2DS12_ACC_DATA_LEN] = {0};

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else
	{
		if(hwmsen_read_block(client, LIS2DS12_OUTX_L_XL|AUTO_INCREMENT, databuf, LIS2DS12_ACC_DATA_LEN))
		{
			GSE_ERR("LIS2DS12 read acc data  error\n");
			return -2;
		}
		else
		{
		#if 0
			data[LIS2DS12_AXIS_X] = (s16)((databuf[LIS2DS12_AXIS_X*2+1] << 8) | (databuf[LIS2DS12_AXIS_X*2]));
			data[LIS2DS12_AXIS_Y] = (s16)((databuf[LIS2DS12_AXIS_Y*2+1] << 8) | (databuf[LIS2DS12_AXIS_Y*2]));
			data[LIS2DS12_AXIS_Z] = (s16)((databuf[LIS2DS12_AXIS_Z*2+1] << 8) | (databuf[LIS2DS12_AXIS_Z*2]));
		#else // for 14bit resolution
			data[LIS2DS12_AXIS_X] = (s16)((s16)(((databuf[LIS2DS12_AXIS_X*2+1] << 8) | databuf[LIS2DS12_AXIS_X*2])) >> 2);
			data[LIS2DS12_AXIS_Y] = (s16)((s16)(((databuf[LIS2DS12_AXIS_Y*2+1] << 8) | databuf[LIS2DS12_AXIS_Y*2])) >> 2);
			data[LIS2DS12_AXIS_Z] = (s16)((s16)(((databuf[LIS2DS12_AXIS_Z*2+1] << 8) | databuf[LIS2DS12_AXIS_Z*2])) >> 2);
		#endif
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int LIS2DS12_ReadData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lis2ds12_i2c_data *obj = (struct lis2ds12_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[LIS2DS12_BUFSIZE];
	int acc[LIS2DS12_ACC_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*LIS2DS12_BUFSIZE);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == false)
	{
		res = LIS2DS12_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on lis2ds12 error %d!\n", res);
		}
		msleep(20);
	}

	res = LIS2DS12_ReadRawData(client, obj->data);
	if(res < 0)
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		obj->data[LIS2DS12_AXIS_X] = (long)(obj->data[LIS2DS12_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000); //NTC
		obj->data[LIS2DS12_AXIS_Y] = (long)(obj->data[LIS2DS12_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		obj->data[LIS2DS12_AXIS_Z] = (long)(obj->data[LIS2DS12_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		
		obj->data[LIS2DS12_AXIS_X] += obj->cali_sw[LIS2DS12_AXIS_X];
		obj->data[LIS2DS12_AXIS_Y] += obj->cali_sw[LIS2DS12_AXIS_Y];
		obj->data[LIS2DS12_AXIS_Z] += obj->cali_sw[LIS2DS12_AXIS_Z];
		
		/*remap coordinate*/
		acc[obj->cvt.map[LIS2DS12_AXIS_X]] = obj->cvt.sign[LIS2DS12_AXIS_X]*obj->data[LIS2DS12_AXIS_X];
		acc[obj->cvt.map[LIS2DS12_AXIS_Y]] = obj->cvt.sign[LIS2DS12_AXIS_Y]*obj->data[LIS2DS12_AXIS_Y];
		acc[obj->cvt.map[LIS2DS12_AXIS_Z]] = obj->cvt.sign[LIS2DS12_AXIS_Z]*obj->data[LIS2DS12_AXIS_Z];

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[LIS2DS12_AXIS_X], acc[LIS2DS12_AXIS_Y], acc[LIS2DS12_AXIS_Z]);

		//Out put the mg
		/*
		acc[LIS2DS12_AXIS_X] = acc[LIS2DS12_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LIS2DS12_AXIS_Y] = acc[LIS2DS12_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LIS2DS12_AXIS_Z] = acc[LIS2DS12_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		*/

		sprintf(buf, "%04x %04x %04x", acc[LIS2DS12_AXIS_X], acc[LIS2DS12_AXIS_Y], acc[LIS2DS12_AXIS_Z]);
	
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			//GSE_LOG("gsensor data: %s!\n", buf);
			GSE_LOG("raw data:obj->data:%04x %04x %04x\n", obj->data[LIS2DS12_AXIS_X], obj->data[LIS2DS12_AXIS_Y], obj->data[LIS2DS12_AXIS_Z]);
			GSE_LOG("acc:%04x %04x %04x\n", acc[LIS2DS12_AXIS_X], acc[LIS2DS12_AXIS_Y], acc[LIS2DS12_AXIS_Z]);
			//LIS2DS12_dumpReg(client);
		}
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int LIS2DS12_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "LIS2DS12 Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2ds12_i2c_client;
	char strbuf[LIS2DS12_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LIS2DS12_ReadChipInfo(client, strbuf, LIS2DS12_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2ds12_i2c_client;
	char strbuf[LIS2DS12_BUFSIZE];
	int x,y,z;
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LIS2DS12_ReadData(client, strbuf, LIS2DS12_BUFSIZE);
	sscanf(strbuf, "%x %x %x", &x, &y, &z);	
	return snprintf(buf, PAGE_SIZE, "%d, %d, %d\n", x,y,z);            
}
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2ds12_i2c_client;
	s16 data[LIS2DS12_ACC_AXES_NUM] = {0};
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LIS2DS12_ReadRawData(client, data);
	return snprintf(buf, PAGE_SIZE, "%x,%x,%x\n", data[0],data[1],data[2]);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lis2ds12_i2c_data *obj = obj_i2c_data;
	
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lis2ds12_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	
	return count;    
}
static ssize_t show_chipinit_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lis2ds12_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lis2ds12_i2c_data *obj = obj_i2c_data;

	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}
	
	LIS2DS12_InitClient(obj->client, true);
	LIS2DS12_dumpReg(obj->client);
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct lis2ds12_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->sensitivity, obj->hw->power_id, obj->hw->power_vol);   
	LIS2DS12_dumpReg(obj->client);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct lis2ds12_i2c_data *data = obj_i2c_data;
	if(NULL == data)
	{
		printk(KERN_ERR "lis2ds12_i2c_data is null!!\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int layout = 0;
	struct lis2ds12_i2c_data *data = obj_i2c_data;

	if(NULL == data)
	{
		printk(KERN_ERR "lis2ds12_i2c_data is null!!\n");
		return count;
	}

	

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensorrawdata,           S_IRUGO, show_sensorrawdata_value,    NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(trace,      S_IWUGO | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(chipinit,      S_IWUGO | S_IRUGO, show_chipinit_value,         store_chipinit_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *LIS2DS12_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/	
	&driver_attr_sensorrawdata,   /*dump sensor raw data*/	
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,  
	&driver_attr_chipinit,
	&driver_attr_layout,
};
/*----------------------------------------------------------------------------*/
static int lis2ds12_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(LIS2DS12_attr_list)/sizeof(LIS2DS12_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver,  LIS2DS12_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n",  LIS2DS12_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int lis2ds12_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof( LIS2DS12_attr_list)/sizeof( LIS2DS12_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver,  LIS2DS12_attr_list[idx]);
	}
	return err;
}

static int LIS2DS12_Set_RegInc(struct i2c_client *client, bool inc)
{
	u8 databuf[2] = {0};    
	int res = 0;
	//GSE_FUN(); 
	
	if(hwmsen_read_byte(client, LIS2DS12_CTRL2, databuf))
	{
		GSE_ERR("read LIS2DS12_CTRL2 err!\n");
		return LIS2DS12_ERR_I2C;
	}
	else
	{
		GSE_LOG("read  LIS2DS12_CTRL2 register: 0x%02x\n", databuf[0]);
	}
	if(inc)
	{
		databuf[0] |= LIS2DS12_CTRL2_IF_ADD_INC;
		
		databuf[1] = databuf[0];
		databuf[0] = LIS2DS12_CTRL2; 
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			GSE_ERR("write full scale register err!\n");
			return LIS2DS12_ERR_I2C;
		}
	}

	return LIS2DS12_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int LIS2DS12_InitClient(struct i2c_client *client, bool enable)
{
	struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	GSE_FUN();
    GSE_LOG(" lis2ds12 addr %x!\n", client->addr);
	res = LIS2DS12_CheckDeviceID(client);
	if(res != LIS2DS12_SUCCESS)
	{
		return res;
	}

	res = LIS2DS12_Set_RegInc(client, true);
	if(res != LIS2DS12_SUCCESS)
	{
		return res;
	}

	res = LIS2DS12_SetFullScale(client, LIS2DS12_ACC_RANGE_4g);//we have only this choice
	if(res != LIS2DS12_SUCCESS) 
	{
		return res;
	}

	res = LIS2DS12_SetSampleRate(client, obj->sample_rate);
	if(res != LIS2DS12_SUCCESS ) 
	{
		return res;
	}

	res = LIS2DS12_Write_PedoParameters(client, 1, 0x0e, 0x5f);// set threshold to a certain value here
	if(res != LIS2DS12_SUCCESS)
	{
		GSE_LOG(" LIS2DS12_Write_PedoParameters failed!\n");
		return LIS2DS12_ERR_STATUS;
	}	

	res = LIS2DS12_SetPowerMode(client, enable);
	if(res != LIS2DS12_SUCCESS)
	{
		return res;
	}

	GSE_LOG("LIS2DS12_InitClient OK!\n");

#ifdef CONFIG_LIS2DS12_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return LIS2DS12_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lis2ds12_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
	
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int lis2ds12_enable_nodata(int en)
{
	int value = en;
	int err = 0;
	struct lis2ds12_i2c_data *priv = obj_i2c_data;

	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	if(value == 1)
	{
		enable_status = true;
	}
	else
	{
		enable_status = false;
		priv->sample_rate = LIS2DS12_ACC_ODR_100HZ; //default rate
	}
	GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);
	
	if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
	{
		GSE_LOG("Gsensor device have updated!\n");
	}
#if (LIS2DS12_EMBEDED_FUNC)
	else if(!pedo_enable_status && !tilt_enable_status)
	{
		err = LIS2DS12_SetPowerMode( priv->client, enable_status);					
	}
#endif
    GSE_LOG("%s OK!\n",__FUNCTION__);
    return err;
}

static int lis2ds12_set_delay(u64 ns)
{
    int value =0;
	int err = 0;
	int sample_delay;
	struct lis2ds12_i2c_data *priv = obj_i2c_data;

    	//value = (int)(ns/1000/1000);
    	value = (int)(ns/1000000);
	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
					
	if(value <= 5)
	{
		sample_delay = LIS2DS12_ACC_ODR_200HZ;
	}
	else if(value <= 10)
	{
		sample_delay = LIS2DS12_ACC_ODR_100HZ;
	}
	else
	{
		sample_delay = LIS2DS12_ACC_ODR_50HZ;
	}
	priv->sample_rate = sample_delay;
	err = LIS2DS12_SetSampleRate(priv->client, sample_delay);
	if(err != LIS2DS12_SUCCESS ) 
	{
		GSE_ERR("Set delay parameter error!\n");
	}

	if(value >= 50)
	{
		atomic_set(&priv->filter, 0);
	}
	else
	{					
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[LIS2DS12_AXIS_X] = 0;
		priv->fir.sum[LIS2DS12_AXIS_Y] = 0;
		priv->fir.sum[LIS2DS12_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	}

    GSE_LOG("%s (%d), chip only use 1024HZ \n",__FUNCTION__, value);
    return 0;
}

static int lis2ds12_get_data(int* x ,int* y,int* z, int* status)
{
    char buff[LIS2DS12_BUFSIZE];
	struct lis2ds12_i2c_data *priv = obj_i2c_data;
		
	if(priv == NULL)
	{
		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	if(atomic_read(&priv->trace) & ACCEL_TRC_DATA)
	{
		GSE_LOG("%s (%d),  \n",__FUNCTION__,__LINE__);
	}
	memset(buff, 0, sizeof(buff));
	LIS2DS12_ReadData(priv->client, buff, LIS2DS12_BUFSIZE);
	
	sscanf(buff, "%x %x %x", x, y, z);				
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;				

    return 0;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int lis2ds12_open(struct inode *inode, struct file *file)
{
	file->private_data = lis2ds12_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int lis2ds12_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long lis2ds12_acc_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct lis2ds12_i2c_data *obj = (struct lis2ds12_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[LIS2DS12_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			LIS2DS12_ReadChipInfo(client, strbuf, LIS2DS12_BUFSIZE);
			
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			LIS2DS12_ReadData(client, strbuf, LIS2DS12_BUFSIZE);
			
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			

			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			LIS2DS12_ReadRawData(client, (s16 *)strbuf);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
			#if 0
				cali[LIS2DS12_AXIS_X] = (s64)(sensor_data.x) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000); //NTC
				cali[LIS2DS12_AXIS_Y] = (s64)(sensor_data.y) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
				cali[LIS2DS12_AXIS_Z] = (s64)(sensor_data.z) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
			#else
				cali[LIS2DS12_AXIS_X] = (s64)(sensor_data.x);
				cali[LIS2DS12_AXIS_Y] = (s64)(sensor_data.y);
				cali[LIS2DS12_AXIS_Z] = (s64)(sensor_data.z);
			#endif
				err = LIS2DS12_WriteCalibration(client, cali);
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = LIS2DS12_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			err = LIS2DS12_ReadCalibration(client, cali);
			if(err < 0)
			{
				break;
			}
					
		#if 0
			sensor_data.x = (s64)(cali[LIS2DS12_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000); //NTC
			sensor_data.y = (s64)(cali[LIS2DS12_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
			sensor_data.z = (s64)(cali[LIS2DS12_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		#else
			sensor_data.x = (s64)(cali[LIS2DS12_AXIS_X]);
			sensor_data.y = (s64)(cali[LIS2DS12_AXIS_Y]);
			sensor_data.z = (s64)(cali[LIS2DS12_AXIS_Z]);
		#endif
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		
		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

#ifdef CONFIG_COMPAT
static long lis2ds12_acc_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
		        return err;
		    }
			break;
			
        case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
		        return err;
		    }
			break;
			
        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
		        return err;
		    }
			break;
			
        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
		        return err;
		    }
			break;

        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;

    }

    return err;
}
#endif

/*----------------------------------------------------------------------------*/
static struct file_operations lis2ds12_acc_fops = {
	.owner = THIS_MODULE,
	.open = lis2ds12_open,
	.release = lis2ds12_release,
	.unlocked_ioctl = lis2ds12_acc_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = lis2ds12_acc_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice lis2ds12_acc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &lis2ds12_acc_fops,
};

/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
static int lis2ds12_acc_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);    
	GSE_FUN();
	int err = 0;
	
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);

		err = LIS2DS12_SetPowerMode(obj->client, false);
		if(err)
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}
		
		sensor_power = false;
		
		LIS2DS12_power(obj->hw, 0);

	}
	return err;
}

static int lis2ds12_acc_resume(struct i2c_client *client)
{
	struct lis2ds12_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -1;
	}

	LIS2DS12_power(obj->hw, 1);
	err = LIS2DS12_SetPowerMode(obj->client, enable_status);
	if(err)
	{
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return err ;        
	}
	atomic_set(&obj->suspend, 0);  

	return 0;
}
#else /* CONFIG_HAS_EARLY_SUSPEND is defined */
/*----------------------------------------------------------------------------*/
static void lis2ds12_early_suspend(struct early_suspend *h) 
{
	return;
}

/*----------------------------------------------------------------------------*/
static void lis2ds12_late_resume(struct early_suspend *h)
{
	return;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

/*----------------------------------------------------------------------------*/
static int lis2ds12_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct lis2ds12_i2c_data *obj;

	int err = 0;

	GSE_FUN();
    
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct lis2ds12_i2c_data));

#ifdef LIS2DS12_EMBEDED_FUNC
	INIT_WORK(&obj->eint_work, LIS2DS12_EintWork);
#endif
	obj->hw = lis2ds12_get_cust_acc_hw(); 
	obj->sample_rate = LIS2DS12_ACC_ODR_100HZ;
	
	atomic_set(&obj->layout, obj->hw->direction);
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if(err)
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit_kfree;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
	LIS2DS12_Reset_Pedo_Data(client);//reset step counter to avoid can't clear previous data in chip

	lis2ds12_i2c_client = new_client;	
	err = LIS2DS12_InitClient(new_client, false);
	if(err)
	{
		goto exit_init_failed;
	}
	
	err = misc_register(&lis2ds12_acc_device);
	if(err)
	{
		GSE_ERR("lis2ds12_acc_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = lis2ds12_early_suspend,
	obj->early_drv.resume   = lis2ds12_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif
#ifdef LIS2DS12_EMBEDED_FUNC
	err = lis2ds12_setup_eint();
	if(err){
		GSE_ERR("lis2ds12 setup interrupt failed!\n");
		goto exit_misc_device_register_failed;
	}
#endif
	lis2ds12_acc_init_flag = 0;

#ifdef VENDOR_EDIT
	step_tune.pre_value = 0;
	step_tune.new_enable = 0;
#endif//VENDOR_EDIT
	register_device_proc("Sensor_accel", "lis2ds12", "ST");

	GSE_LOG("%s: OK\n", __func__);    
	return 0;

exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	lis2ds12_acc_init_flag = -1;
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int lis2ds12_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	if(test_bit(LIS2DS12_ACC, &lis2ds12_init_flag_test))
	{
		err = lis2ds12_delete_attr(&(lis2ds12_init_info.platform_diver_addr->driver));
	}
	lis2ds12_acc_init_flag = -1;

	err = misc_deregister(&lis2ds12_acc_device);
	if(err)
	{
		GSE_ERR("misc_deregister lis2ds12_device fail: %d\n", err);
	}

	lis2ds12_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int lis2ds12_local_init_common(void)
{
	struct acc_hw *accel_hw = lis2ds12_get_cust_acc_hw();
	GSE_FUN();
	
	LIS2DS12_power(accel_hw, 1);
	
	if(i2c_add_driver(&lis2ds12_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int lis2ds12_local_init(void)
{
	int res = 0;
	struct acc_control_path ctl={0};
    struct acc_data_path data={0};
	struct lis2ds12_i2c_data *obj = NULL;
	
	GSE_FUN();

	mutex_lock(&lis2ds12_init_mutex);
	
	set_bit(LIS2DS12_ACC, &lis2ds12_init_flag_test);

	if((0==test_bit(LIS2DS12_STEP_C, &lis2ds12_init_flag_test)) \
		&& (0 == test_bit(LIS2DS12_TILT, &lis2ds12_init_flag_test)))
	{
		res = lis2ds12_local_init_common();
		if(res < 0)
		{
			goto lis2ds12_local_init_failed;
		}
		
	}

	if(lis2ds12_acc_init_flag == -1)
	{
		mutex_unlock(&lis2ds12_init_mutex);
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	}
	else
	{
		obj = obj_i2c_data;
		if(NULL == obj)
		{
			GSE_ERR("i2c_data obj is null!!\n");
			goto lis2ds12_local_init_failed;
		}
		
		res = lis2ds12_create_attr(&(lis2ds12_init_info.platform_diver_addr->driver));
		if(res < 0)
		{
			goto lis2ds12_local_init_failed;
		}
		ctl.open_report_data= lis2ds12_open_report_data;
	    ctl.enable_nodata = lis2ds12_enable_nodata;
	    ctl.set_delay  = lis2ds12_set_delay;
	    ctl.is_report_input_direct = false;
	    ctl.is_support_batch = obj->hw->is_batch_supported;

	    res = acc_register_control_path(&ctl);
	    if(res)
	    {
	         GSE_ERR("register acc control path err\n");
			 goto lis2ds12_local_init_failed;

	    }

	    data.get_data = lis2ds12_get_data;
	    data.vender_div = 1000;
	    res = acc_register_data_path(&data);
	    if(res)
	    {
	        GSE_ERR("register acc data path err= %d\n", res);
			goto lis2ds12_local_init_failed;

	    }
	}

	mutex_unlock(&lis2ds12_init_mutex);
	return 0;
lis2ds12_local_init_failed:
	GSE_ERR("%s init failed\n", __FUNCTION__);
	mutex_unlock(&lis2ds12_init_mutex);
	return res;
}

/*----------------------------------------------------------------------------*/
static int lis2ds12_local_uninit(void)
{
	struct acc_hw *accel_hw = lis2ds12_get_cust_acc_hw();
	clear_bit(LIS2DS12_ACC, &lis2ds12_init_flag_test);

    //GSE_FUN();
    LIS2DS12_power(accel_hw, 0);
    i2c_del_driver(&lis2ds12_i2c_driver);

    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init lis2ds12_init(void)
{
	struct acc_hw *hw = lis2ds12_get_cust_acc_hw();
	GSE_FUN();
    printk("%s: i2c_number=%d\n", __func__,hw->i2c_num);
    i2c_register_board_info(hw->i2c_num, &i2c_lis2ds12, 1);

	acc_driver_add(&lis2ds12_init_info);

#if defined(LIS2DS12_STEP_COUNTER) //step counter
	step_c_driver_add(&lis2ds12_step_c_init_info);
#endif

#if defined(LIS2DS12_TILT_FUNC)
	tilt_driver_add(&lis2ds12_tilt_init_info);
#endif

	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit lis2ds12_exit(void)
{
	GSE_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(lis2ds12_init);
module_exit(lis2ds12_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LIS2DS12 Accelerometer New Arch");
MODULE_AUTHOR("xj.wang@mediatek.com, jay.huangfu@st.com");
/*----------------------------------------------------------------- LIS2DS12 ------------------------------------------------------------------*/
