/* ST LSM6DS3 Accelerometer and Gyroscope sensor driver
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

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "lsm6ds3_gy.h"
#include <linux/hwmsen_helper.h>
#include <linux/kernel.h>
#include <mach/mt_pm_ldo.h>
#include <cust_gyro.h>
#include <linux/oppo_devices_list.h>
#include <linux/dma-mapping.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#define LSM6DS3_GYRO_NEW_ARCH		/*kk and L compatialbe*/

#ifdef LSM6DS3_GYRO_NEW_ARCH
#include <gyroscope.h>
#endif

/*---------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_LSM6DS3_LOWPASS   /*apply low pass filter on output*/
/*----------------------------------------------------------------------------*/
#define LSM6DS3_AXIS_X          0
#define LSM6DS3_AXIS_Y          1
#define LSM6DS3_AXIS_Z          2

#define LSM6DS3_GYRO_AXES_NUM       3
#define LSM6DS3_GYRO_DATA_LEN       6
#define LSM6DS3_GYRO_DEV_NAME        "LSM6DS3_GYRO"


#define FIFO_LEN 450
#define I2C_MASTER_CLOCK 300
static u8*		I2CDMABuf_va = NULL;
static u32*		I2CDMABuf_pa = NULL;
struct input_dev *input_dev;
static int FIFOinit = 0;
static int FIFOtestResult = 0;
static int firstFIFOdata = 1;
static int firstresetdata = 0;
//static char fifobuf[FIFO_LEN*5] = {0};
static int fifodata[FIFO_LEN/18*35] = {0};
static u32 pretimestamp = 0;
static int cali_res = -1;

extern GYRO_DEV gyro_dev;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id lsm6ds3_gyro_i2c_id[] = {{LSM6DS3_GYRO_DEV_NAME, 0}, {} };
static struct i2c_board_info __initdata i2c_lsm6ds3_gyro = { I2C_BOARD_INFO(LSM6DS3_GYRO_DEV_NAME, 0x34)}; /*0xD4>>1 is right address*/

/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lsm6ds3_gyro_i2c_remove(struct i2c_client *client);
static int LSM6DS3_gyro_init_client(struct i2c_client *client, bool enable);

#ifndef CONFIG_HAS_EARLYSUSPEND
static int lsm6ds3_gyro_resume(struct i2c_client *client);
static int lsm6ds3_gyro_suspend(struct i2c_client *client, pm_message_t msg);
#endif
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
    GYRO_TRC_FILTER  = 0x01,
    GYRO_TRC_RAWDATA = 0x02,
    GYRO_TRC_IOCTL   = 0x04,
    GYRO_TRC_CALI	= 0X08,
    GYRO_TRC_INFO	= 0X10,
    GYRO_TRC_DATA	= 0X20,
} GYRO_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int					sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/

struct gyro_data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LSM6DS3_GYRO_AXES_NUM];
    int sum[LSM6DS3_GYRO_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct lsm6ds3_gyro_i2c_data {
    struct i2c_client *client;
    struct gyro_hw *hw;
    struct hwmsen_convert   cvt;
    atomic_t    layout;

    /*misc*/
    /*struct data_resolution *reso;*/
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s32                     cali_sw[LSM6DS3_GYRO_AXES_NUM+1];
	s32						fifo_cali_sw[LSM6DS3_GYRO_AXES_NUM+1];

    /*data*/

    s8                      offset[LSM6DS3_GYRO_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s32                     data[LSM6DS3_GYRO_AXES_NUM+1];
	int 					sensitivity;

#if defined(CONFIG_LSM6DS3_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct gyro_data_filter      fir;
#endif
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver lsm6ds3_gyro_i2c_driver = {
    .driver = {
		.owner			= THIS_MODULE,
		.name			= LSM6DS3_GYRO_DEV_NAME,
    },
	.probe				= lsm6ds3_gyro_i2c_probe,
	.remove				= lsm6ds3_gyro_i2c_remove,

#if !defined(CONFIG_HAS_EARLYSUSPEND)
    .suspend			= lsm6ds3_gyro_suspend,
    .resume				= lsm6ds3_gyro_resume,
#endif
	.id_table = lsm6ds3_gyro_i2c_id,
};
#ifdef LSM6DS3_GYRO_NEW_ARCH
static int lsm6ds3_gyro_local_init(struct platform_device *pdev);
static int lsm6ds3_gyro_local_uninit(void);
static int lsm6ds3_gyro_init_flag = -1;
static struct gyro_init_info  lsm6ds3_gyro_init_info = {

	.name	= LSM6DS3_GYRO_DEV_NAME,
	.init	= lsm6ds3_gyro_local_init,
	.uninit	= lsm6ds3_gyro_local_uninit,
};
#endif
/*----------------------------------------------------------------------------*/
static struct i2c_client *lsm6ds3_i2c_client;	/*initial in module_init*/

static struct lsm6ds3_gyro_i2c_data *obj_i2c_data;	/*initial in module_init*/
static bool sensor_power;	/*initial in module_init*/
static bool enable_status;	/*initial in module_init*/

/*----------------------------------------------------------------------------*/
//#define GYRO_TAG                  "[Gyroscope] "

//#define GYRO_FUN(f)               printk(KERN_INFO "[Gyroscope] %s\n", __FUNCTION__)
//#define GYRO_ERR(fmt, args...)    printk(KERN_ERR GYRO_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
//#define GYRO_LOG(fmt, args...)    printk(KERN_INFO GYRO_TAG fmt, ##args)


/*----------------------------------------------------------------------------

static void LSM6DS3_dumpReg(struct i2c_client *client)
{
	int i = 0;
	u8 addr = 0x10;
	u8 regdata = 0;
	for (i = 0; i < 25 ; i++) {

		hwmsen_read_byte(client, addr, &regdata);
		HWM_LOG("Reg addr=%x regdata=%x\n", addr, regdata);
		addr++;
	}
}

--------------------gyroscopy power control function----------------------------------*/
static void LSM6DS3_power(struct gyro_hw *hw, unsigned int on)
{
	static unsigned int power_on;	/*default = 0;*/

	if (hw->power_id != POWER_NONE_MACRO) {		/* have externel LDO*/

		GYRO_LOG("power %s\n", on ? "on" : "off");
		if (power_on == on) {	/* power status not change*/

			GYRO_LOG("ignore power control: %d\n", on);
		} else if (on) {	/* power on*/

			if (!hwPowerOn(hw->power_id, hw->power_vol, "LSM6DS3")) {

				GYRO_ERR("power on fails!!\n");
			}
		} else {	/* power off*/

			if (!hwPowerDown(hw->power_id, "LSM6DS3")) {

				GYRO_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int LSM6DS3_gyro_write_rel_calibration(struct lsm6ds3_gyro_i2c_data *obj, int dat[LSM6DS3_GYRO_AXES_NUM])
{
	obj->cali_sw[LSM6DS3_AXIS_X] = obj->cvt.sign[LSM6DS3_AXIS_X] * dat[obj->cvt.map[LSM6DS3_AXIS_X]];
	obj->cali_sw[LSM6DS3_AXIS_Y] = obj->cvt.sign[LSM6DS3_AXIS_Y] * dat[obj->cvt.map[LSM6DS3_AXIS_Y]];
	obj->cali_sw[LSM6DS3_AXIS_Z] = obj->cvt.sign[LSM6DS3_AXIS_Z] * dat[obj->cvt.map[LSM6DS3_AXIS_Z]];
#if DEBUG
		if (atomic_read(&obj->trace) & GYRO_TRC_CALI) {

			GYRO_LOG("test  (%5d, %5d, %5d) ->(%5d, %5d, %5d)->(%5d, %5d, %5d))\n",
				obj->cvt.sign[LSM6DS3_AXIS_X], obj->cvt.sign[LSM6DS3_AXIS_Y], obj->cvt.sign[LSM6DS3_AXIS_Z],
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z],
				obj->cvt.map[LSM6DS3_AXIS_X], obj->cvt.map[LSM6DS3_AXIS_Y], obj->cvt.map[LSM6DS3_AXIS_Z]);
			GYRO_LOG("write gyro calibration data  (%5d, %5d, %5d)\n",
				obj->cali_sw[LSM6DS3_AXIS_X], obj->cali_sw[LSM6DS3_AXIS_Y], obj->cali_sw[LSM6DS3_AXIS_Z]);
		}
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_gyro_ResetCalibration(struct i2c_client *client)
{
	struct lsm6ds3_gyro_i2c_data *obj = i2c_get_clientdata(client);

	memset(obj->cali_sw, 0x0000, sizeof(obj->cali_sw));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_gyro_ReadCalibration(struct i2c_client *client, int dat[LSM6DS3_GYRO_AXES_NUM])
{
    struct lsm6ds3_gyro_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->cali_sw[LSM6DS3_AXIS_X];
    dat[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->cali_sw[LSM6DS3_AXIS_Y];
    dat[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->cali_sw[LSM6DS3_AXIS_Z];

#if DEBUG
		if (atomic_read(&obj->trace) & GYRO_TRC_CALI) {

			GYRO_LOG("Read gyro calibration data  (%5d, %5d, %5d)\n",
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z]);
		}
#endif

    return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int LSM6DS3_gyro_WriteCalibration(struct i2c_client *client, int dat[LSM6DS3_GYRO_AXES_NUM])
{
	struct lsm6ds3_gyro_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[LSM6DS3_GYRO_AXES_NUM];


	GYRO_FUN();
	if (!obj || !dat) {

		GYRO_ERR("null ptr!!\n");
		return -EINVAL;
	} else {

		cali[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X] * obj->cali_sw[LSM6DS3_AXIS_X];
		cali[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y] * obj->cali_sw[LSM6DS3_AXIS_Y];
		cali[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z] * obj->cali_sw[LSM6DS3_AXIS_Z];
		cali[LSM6DS3_AXIS_X] += dat[LSM6DS3_AXIS_X];
		cali[LSM6DS3_AXIS_Y] += dat[LSM6DS3_AXIS_Y];
		cali[LSM6DS3_AXIS_Z] += dat[LSM6DS3_AXIS_Z];
#if DEBUG
		if (atomic_read(&obj->trace) & GYRO_TRC_CALI) {

			GYRO_LOG("write gyro calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n",
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z],
				cali[LSM6DS3_AXIS_X], cali[LSM6DS3_AXIS_Y], cali[LSM6DS3_AXIS_Z]);
		}
#endif
		return LSM6DS3_gyro_write_rel_calibration(obj, cali);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int LSM6DS3_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	databuf[0] = LSM6DS3_FIXED_DEVID;

	res = hwmsen_read_byte(client, LSM6DS3_WHO_AM_I, databuf);
    GYRO_LOG(" LSM6DS3  id %x!\n", databuf[0]);
	if (databuf[0] != LSM6DS3_FIXED_DEVID) {

		return LSM6DS3_ERR_IDENTIFICATION;
	}

	if (res < 0) {

		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

/*/----------------------------------------------------------------------------/*/
static int LSM6DS3_gyro_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0;

	if (enable == sensor_power) {

		GYRO_LOG("Sensor power status is newest!\n");
		return LSM6DS3_SUCCESS;
	}

	if (hwmsen_read_byte(client, LSM6DS3_CTRL2_G, databuf)) {

		GYRO_ERR("read lsm6ds3 power ctl register err!\n");
		return LSM6DS3_ERR_I2C;
	}


	if (true == enable) {

		databuf[0] &= ~LSM6DS3_GYRO_ODR_MASK;/*clear lsm6ds3 gyro ODR bits*/
		databuf[0] |= LSM6DS3_GYRO_ODR_104HZ; /*default set 100HZ for LSM6DS3 gyro*/


	} else {

		/* do nothing*/
		databuf[0] &= ~LSM6DS3_GYRO_ODR_MASK;/*clear lsm6ds3 gyro ODR bits*/
		databuf[0] |= LSM6DS3_GYRO_ODR_POWER_DOWN; /*POWER DOWN*/
	}
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL2_G;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GYRO_LOG("LSM6DS3 set power mode: ODR 100hz failed!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GYRO_LOG("set LSM6DS3 gyro power mode:ODR 100HZ ok %d!\n", enable);
	}


	sensor_power = enable;

	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_Set_RegInc(struct i2c_client *client, bool inc)
{
	u8 databuf[2] = {0};
	int res = 0;
	/*GYRO_FUN();     */

	if (hwmsen_read_byte(client, LSM6DS3_CTRL3_C, databuf)) {

		GYRO_ERR("read LSM6DS3_CTRL1_XL err!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GYRO_LOG("read  LSM6DS3_CTRL1_XL register: 0x%x\n", databuf[0]);
	}
	if (inc) {

		databuf[0] |= LSM6DS3_CTRL3_C_IFINC;

		databuf[1] = databuf[0];
		databuf[0] = LSM6DS3_CTRL3_C;


		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {

			GYRO_ERR("write full scale register err!\n");
			return LSM6DS3_ERR_I2C;
		}
	}
	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_gyro_SetFullScale(struct i2c_client *client, u8 gyro_fs)
{
	u8 databuf[2] = {0};
	int res = 0;
	GYRO_FUN();

	if (hwmsen_read_byte(client, LSM6DS3_CTRL2_G, databuf)) {

		GYRO_ERR("read LSM6DS3_CTRL2_G err!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GYRO_LOG("read  LSM6DS3_CTRL2_G register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_GYRO_RANGE_MASK;/*clear */
	databuf[0] |= gyro_fs;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL2_G;


	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GYRO_ERR("write full scale register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	return LSM6DS3_SUCCESS;
}

/*----------------------------------------------------------------------------*/

/* set the gyro sample rate*/
static int LSM6DS3_gyro_SetSampleRate(struct i2c_client *client, u8 sample_rate)
{
	u8 databuf[2] = {0};
	int res = 0;
	GYRO_FUN();

	res = LSM6DS3_gyro_SetPowerMode(client, true);	/*set Sample Rate will enable power and should changed power status*/
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	if (hwmsen_read_byte(client, LSM6DS3_CTRL2_G, databuf)) {

		GYRO_ERR("read gyro data format register err!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GYRO_LOG("read  gyro data format register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_GYRO_ODR_MASK;/*clear */
	databuf[0] |= sample_rate;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL2_G;


	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GYRO_ERR("write sample rate register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_ReadGyroData(struct i2c_client *client, char *buf, int bufsize)
{
	char databuf[6];
	int data[3];
	struct lsm6ds3_gyro_i2c_data *obj = i2c_get_clientdata(client);

	if (sensor_power == false) {

		LSM6DS3_gyro_SetPowerMode(client, true);
	}

	if (hwmsen_read_block(client, LSM6DS3_OUTX_L_G, databuf, 6)) {

		GYRO_ERR("LSM6DS3 read gyroscope data  error\n");
		return -2;
	} else {

		obj->data[LSM6DS3_AXIS_X] = (s16)((databuf[LSM6DS3_AXIS_X*2+1] << 8) | (databuf[LSM6DS3_AXIS_X*2]));
		obj->data[LSM6DS3_AXIS_Y] = (s16)((databuf[LSM6DS3_AXIS_Y*2+1] << 8) | (databuf[LSM6DS3_AXIS_Y*2]));
		obj->data[LSM6DS3_AXIS_Z] = (s16)((databuf[LSM6DS3_AXIS_Z*2+1] << 8) | (databuf[LSM6DS3_AXIS_Z*2]));

#if DEBUG
		if (atomic_read(&obj->trace) & GYRO_TRC_RAWDATA) {

			GYRO_LOG("read gyro register: %x, %x, %x, %x, %x, %x",
				databuf[0], databuf[1], databuf[2], databuf[3], databuf[4], databuf[5]);
			GYRO_LOG("get gyro raw data (0x%08X, 0x%08X, 0x%08X) -> (%5d, %5d, %5d)\n",
				obj->data[LSM6DS3_AXIS_X], obj->data[LSM6DS3_AXIS_Y], obj->data[LSM6DS3_AXIS_Z],
				obj->data[LSM6DS3_AXIS_X], obj->data[LSM6DS3_AXIS_Y], obj->data[LSM6DS3_AXIS_Z]);
			GYRO_LOG("get gyro cali data (%5d, %5d, %5d)\n",
				obj->cali_sw[LSM6DS3_AXIS_X], obj->cali_sw[LSM6DS3_AXIS_Y], obj->cali_sw[LSM6DS3_AXIS_Z]);
		}
#endif
#if 1
		obj->data[LSM6DS3_AXIS_X] += obj->cali_sw[LSM6DS3_AXIS_X];
		obj->data[LSM6DS3_AXIS_Y] += obj->cali_sw[LSM6DS3_AXIS_Y];
		obj->data[LSM6DS3_AXIS_Z] += obj->cali_sw[LSM6DS3_AXIS_Z];
	#if 1
		obj->data[LSM6DS3_AXIS_X] = (long)(obj->data[LSM6DS3_AXIS_X]) * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142;
		obj->data[LSM6DS3_AXIS_Y] = (long)(obj->data[LSM6DS3_AXIS_Y]) * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142;
		obj->data[LSM6DS3_AXIS_Z] = (long)(obj->data[LSM6DS3_AXIS_Z]) * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142;
	#else
			/*report degree/s */
		obj->data[LSM6DS3_AXIS_X] = (long)(obj->data[LSM6DS3_AXIS_X])*LSM6DS3_GYRO_SENSITIVITY_2000DPS/1000/1000;
		obj->data[LSM6DS3_AXIS_Y] = (long)(obj->data[LSM6DS3_AXIS_Y])*LSM6DS3_GYRO_SENSITIVITY_2000DPS/1000/1000;
		obj->data[LSM6DS3_AXIS_Z] = (long)(obj->data[LSM6DS3_AXIS_Z])*LSM6DS3_GYRO_SENSITIVITY_2000DPS/1000/1000;
	#endif

		/*remap coordinate*/
		data[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X] * obj->data[LSM6DS3_AXIS_X];
		data[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y] * obj->data[LSM6DS3_AXIS_Y];
		data[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z] * obj->data[LSM6DS3_AXIS_Z];
#else
		data[LSM6DS3_AXIS_X] = (s64)(data[LSM6DS3_AXIS_X]) * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142/(180*1000*1000);
		data[LSM6DS3_AXIS_Y] = (s64)(data[LSM6DS3_AXIS_Y]) * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142/(180*1000*1000);
		data[LSM6DS3_AXIS_Z] = (s64)(data[LSM6DS3_AXIS_Z]) * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142/(180*1000*1000);
#endif
	}

	sprintf(buf, "%x %x %x", data[LSM6DS3_AXIS_X], data[LSM6DS3_AXIS_Y], data[LSM6DS3_AXIS_Z]);

#if DEBUG
	if (atomic_read(&obj->trace) & GYRO_TRC_DATA) {

		GYRO_LOG("get gyro data packet:[%d %d %d]\n", data[0], data[1], data[2]);
	}
#endif

	return 0;

}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8)*10);

	if ((NULL == buf) || (bufsize <= 30)) {

		return -1;
	}

	if (NULL == client) {

		*buf = 0;
		return -2;
	}

	sprintf(buf, "LSM6DS3 Chip");
	return 0;
}

static int LSM6DS3_i2c_dma_read(struct i2c_client *client, u8 addr, s32 len, u8 *rxbuf)
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
			.buf = (u8 *)I2CDMABuf_pa,
			.len = len,
			.timing = I2C_MASTER_CLOCK
		},
	};

	//mutex_lock(&tp_wr_access);
	//buffer[0] = (addr >> 8) & 0xFF;
	buffer[0] = addr & 0xFF;

	if (rxbuf == NULL){
		//mutex_unlock(&tp_wr_access);
		return -1;
	}
	//GYRO_ERR("srd dma i2c read: 0x%x, %d bytes(s)\n", addr, len);
	for (retry = 0; retry < 5; ++retry)  //old  retry < 5
	{
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
		{
			continue;
		}
		memcpy(rxbuf, I2CDMABuf_va, len);
		//mutex_unlock(&tp_wr_access);
		return 0;
	}
	GYRO_ERR("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);

	//mutex_unlock(&tp_wr_access);
	return ret;
}

/*----------------------------------------------------------------------------*/

static int LSM6DS3_initFIFO(struct i2c_client *client)
{
	u8 databuf[2];
	int i;
	int res;
	struct lsm6ds3_gyro_i2c_data *obj = i2c_get_clientdata(client);
	u8 regInitArray[][2] = {
	// reg   val
	{0x10, 0x58}, //Set Accel data range ±4g, and Set Accel ODR to 208Hz
	{0x11, 0x52}, //Set Gyro data range ±125dps, and Set Gyro ODR to 208Hz

	//{0x06, 0xFC}, //Set FIFO size
	//{0x13, 0x01}, //Set limit fifo size

	{0x5C, 0x10}, //Set Timestamp resolution to 25 us
	{0x07, 0x80}, //Enable Timestamp in FIFO
	{0x58, 0x80}, //Enable Timestamp Counter

	{0x08, 0x09}, //Enable Accel and Gyro Samples in FIFO
	{0x09, 0x08}, //Enable timestamp in FIFO

	{0x0A, 0x28}, //Set FIFO ODR 208Hz, and Set FIFO in bypass mode
	//{0x0A, 0x29}, //Set FIFO ODR 208Hz, and Set FIFO in FIFO mode

	};

	GYRO_FUN();

	if (input_dev == NULL)
		input_dev = input_allocate_device();
	if (input_dev == NULL) {
		GYRO_ERR("input_allocate_device failed!\n");
		return -ENOMEM;
	}
	I2CDMABuf_va = (u8 *)dma_alloc_coherent(&input_dev->dev, 9000, (dma_addr_t *)&I2CDMABuf_pa, GFP_KERNEL);
	if(!I2CDMABuf_va) {
		GYRO_ERR("Allocate DMA I2C Buffer failed!\n");
	}
	if(!I2CDMABuf_va) {
		GYRO_ERR("Allocate DMA I2C Buffer failed!\n");
		return -ENOMEM;
	}

	if (sensor_power == false) {
		LSM6DS3_gyro_SetPowerMode(obj->client, true);
	}

	// init registers
	for(i=0; i<(sizeof(regInitArray)/2); i++) {
		databuf[0] = regInitArray[i][0];
		databuf[1] = regInitArray[i][1];
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			GYRO_ERR("write fifo register 0x%02x err!\n",databuf[0]);
			return LSM6DS3_ERR_I2C;
		}
	}
	FIFOinit = 1;
	return 0;
}

static int LSM6DS3_enableFIFO(struct i2c_client *client,int en)
{
	u8 regbuf[2];
	int res;
	GYRO_ERR("%d\n",en);
	regbuf[0] = 0x0A;
	if(!FIFOinit){
		return -2;
	}
	if(en){
		regbuf[1] = 0x29;
	}else{
		regbuf[1] = 0x28;
	}
	res = i2c_master_send(client, regbuf, 0x2);
	if (res <= 0) {
		GYRO_ERR("write fifo register 0x%02x err!\n",regbuf[0]);
		return LSM6DS3_ERR_I2C;
	}

	return 0;
}

static int LSM6DS3_resetTimeStamp(struct i2c_client *client)
{
	u8 regbuf[2];
	int res;
	regbuf[0] = 0x42;
	regbuf[1] = 0xAA;
	res = i2c_master_send(client, regbuf, 0x2);
	if (res <= 0) {
		GYRO_ERR("reset the timestamp register 0x%02x err!\n",regbuf[0]);
		return LSM6DS3_ERR_I2C;
	}
	GYRO_ERR("reset the timestamp\n");
	return 0;
}

static int LSM6DS3_destroyFIFO(struct i2c_client *client)
{
	int res;
	u8 databuf[2];
	int i;
	u8 regRstArray[][2] = {
	//reg   val
	{0x07, 0x00}, //Disable Timestamp in FIFO
	{0x58, 0x00}, //Disable Timestamp Counter

	{0x09, 0x00}, //Disable timestamp in FIFO
	{0x10, 0x38}, //Set Accel data range ±4g, and Set Accel ODR to 52Hz
	{0x11, 0x4C}, //Set Gyro data range ±2000dps, and Set Gyro ODR to 104Hz
	};
	GYRO_FUN();
	res = LSM6DS3_enableFIFO(client,0);
	for(i=0; i<(sizeof(regRstArray)/2); i++) {
		databuf[0] = regRstArray[i][0];
		databuf[1] = regRstArray[i][1];
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			GYRO_ERR("write fifo register 0x%02x err!\n",databuf[0]);
			return LSM6DS3_ERR_I2C;
		}
	}
	if(I2CDMABuf_va) {
		dma_free_coherent(&input_dev->dev, 9000, I2CDMABuf_va, (dma_addr_t)I2CDMABuf_pa);
		I2CDMABuf_va = NULL;
		I2CDMABuf_pa = 0;
	}
	input_free_device(input_dev);
	input_dev = NULL;
	FIFOinit = 0;
	FIFOtestResult = 0;
	firstFIFOdata = 1;
	return 0;
}
static int LSM6DS3_rstFIFOdata(struct i2c_client *client)
{
	int res=0;
	GYRO_ERR("reset\n");
	//full = 1; // FIFO is full, you can do something.
	//LSM6DS3_resetTimeStamp(client);
	res += LSM6DS3_enableFIFO(client,0);
	res += LSM6DS3_enableFIFO(client,1);
	firstresetdata = 1;
	mdelay(12);
	return res;
}

static int LSM6DS3_readfromFIFO(struct i2c_client *client,int *data,int req_len)
{
	static u8 databuf[FIFO_LEN] = {0};
	u8 regbuf[2] = {0};
	//u8 tmpdatabuf[18] = {0};
	int i=0,j=0,res;
	u32 timestamp = 0;
	long gyADC[3]= {0};
	s16 accDATA[3] = {0};
	s16 gyroDATA[3] = {0};
	//u8 testbuf[6] = {0};
	long xlADC[3]= {0};
	int length;
	int difftime;
	struct lsm6ds3_gyro_i2c_data *obj = i2c_get_clientdata(client);

	GYRO_FUN();
	if(!FIFOinit){
		GYRO_ERR("fifo not init\n");
		return -1;
	}
	if(atomic_read(&obj->suspend)==1){
		return 0;
	}
	if(firstFIFOdata){
		GYRO_ERR("fifo read first data\n");
		res = LSM6DS3_enableFIFO(client,1);
		if(res<0){
			return res;
		}
		mdelay(12);
	}
	if (hwmsen_read_block(client, 0x3A, regbuf, 2)) {
		GYRO_ERR("LSM6DS3 read gyroscope reg 0x3A error\n");
		return -2;
	} else {
		length = ((int)regbuf[1]&0x0F)<<8 | (int)regbuf[0];
		if(length>0){
			length -= 1; //avoid the fake word
		}
		length <<= 1; // convert word to byte
		GYRO_ERR("LSM6DS3 has fifo data %d byte\n",length);
		if((regbuf[1] & 0x20)||(length>=FIFO_LEN)){
			GYRO_ERR("LSM6DS3 fifo data is full\n");
			LSM6DS3_rstFIFOdata(client);
			return 0;
		}
		length -= (length%18); //to make length = 18*n

		if(length>0){
			LSM6DS3_i2c_dma_read(client, 0x3E, length, databuf);
		}
		for(i=0; i<length; i+=18) {
			accDATA[LSM6DS3_AXIS_X] = (s16)((databuf[i+7]<<8) | databuf[i+6]);
			accDATA[LSM6DS3_AXIS_Y] = (s16)((databuf[i+9]<<8) | databuf[i+8]);
			accDATA[LSM6DS3_AXIS_Z] = (s16)((databuf[i+11]<<8) | databuf[i+10]);

			gyroDATA[LSM6DS3_AXIS_X] = (s16)((databuf[i+1]<<8) | databuf[i+0]) + obj->fifo_cali_sw[LSM6DS3_AXIS_X];
			gyroDATA[LSM6DS3_AXIS_Y] = (s16)((databuf[i+3]<<8) | databuf[i+2]) + obj->fifo_cali_sw[LSM6DS3_AXIS_Y];
			gyroDATA[LSM6DS3_AXIS_Z] = (s16)((databuf[i+5]<<8) | databuf[i+4]) + obj->fifo_cali_sw[LSM6DS3_AXIS_Z];

			gyADC[obj->cvt.map[LSM6DS3_AXIS_X]] = (long)gyroDATA[LSM6DS3_AXIS_X]*obj->cvt.sign[LSM6DS3_AXIS_X]; //1000000
			gyADC[obj->cvt.map[LSM6DS3_AXIS_Y]] = (long)gyroDATA[LSM6DS3_AXIS_Y]*obj->cvt.sign[LSM6DS3_AXIS_Y]; //
			gyADC[obj->cvt.map[LSM6DS3_AXIS_Z]] = (long)gyroDATA[LSM6DS3_AXIS_Z]*obj->cvt.sign[LSM6DS3_AXIS_Z]; //

			xlADC[obj->cvt.map[LSM6DS3_AXIS_X]] = (long)accDATA[LSM6DS3_AXIS_X]*obj->cvt.sign[LSM6DS3_AXIS_X];
			xlADC[obj->cvt.map[LSM6DS3_AXIS_Y]] = (long)accDATA[LSM6DS3_AXIS_Y]*obj->cvt.sign[LSM6DS3_AXIS_Y];
			xlADC[obj->cvt.map[LSM6DS3_AXIS_Z]] = (long)accDATA[LSM6DS3_AXIS_Z]*obj->cvt.sign[LSM6DS3_AXIS_Z];


			//Timestamp
			timestamp = (u32)((databuf[i+13]<<16) | (databuf[i+12]<<8) | databuf[i+15]);
			difftime = timestamp-pretimestamp;
			GYRO_ERR("length: %d, %d %d %d %d %d %d %d %d\r\n", length,
					(int)gyADC[LSM6DS3_AXIS_X], (int)gyADC[LSM6DS3_AXIS_Y], (int)gyADC[LSM6DS3_AXIS_Z],
					(int)xlADC[LSM6DS3_AXIS_X], (int)xlADC[LSM6DS3_AXIS_Y], (int)xlADC[LSM6DS3_AXIS_Z], timestamp,difftime);

			if((firstresetdata==0)&&(firstFIFOdata==0)){
				if((difftime<180) || (difftime>210)){
					GYRO_ERR("LSM6DS3 fifo data is err\n");
					LSM6DS3_rstFIFOdata(client);
					return 0;
				}
			}

			if(firstFIFOdata){
				firstFIFOdata = 0;
			}

			if(firstresetdata){
				firstresetdata = 0;
				GYRO_ERR("discard the first reset fifo data\n");
				//return 0;
			}

			pretimestamp = timestamp;
			data[j] = gyADC[LSM6DS3_AXIS_X];
			data[j+1] = gyADC[LSM6DS3_AXIS_Y];
			data[j+2] = gyADC[LSM6DS3_AXIS_Z];
			data[j+3] = xlADC[LSM6DS3_AXIS_X];
			data[j+4] = xlADC[LSM6DS3_AXIS_Y];
			data[j+5] = xlADC[LSM6DS3_AXIS_Z];
			data[j+6] = timestamp;
			j+=7;

		}
		//sprintf(data, "%s",fifobuf);
		if(timestamp > 15000000) { // reset the timestamp register
			LSM6DS3_resetTimeStamp(client);
		}
	}
	return length/18;
}

static int lsm6ds3_do_cali(struct i2c_client *client, int *cali_sw){
	int x,y,z;
	int tmp_avg_x=0,tmp_avg_y=0,tmp_avg_z=0;
	int tmp_sum_x=0,tmp_sum_y=0,tmp_sum_z=0;
	int skip_count=0;
	int i,res=0;
	u8 databuf[6] = {0};
	int sample_count = 30;

	GYRO_FUN();
	if (NULL == client) {
		cali_res = -2;
		GYRO_ERR("i2c client is null!!\n");
		return -1;
	}
	if (sensor_power == false) {
		LSM6DS3_gyro_SetPowerMode(client, true);
	}
	msleep(500);

	for(i=0;i<sample_count;){
		msleep(30);
		res += hwmsen_read_block(client, LSM6DS3_OUTX_L_G, databuf, 6);
		x = (s16)((databuf[LSM6DS3_AXIS_X*2+1] << 8) | (databuf[LSM6DS3_AXIS_X*2]));
		y = (s16)((databuf[LSM6DS3_AXIS_Y*2+1] << 8) | (databuf[LSM6DS3_AXIS_Y*2]));
		z = (s16)((databuf[LSM6DS3_AXIS_Z*2+1] << 8) | (databuf[LSM6DS3_AXIS_Z*2]));
		GYRO_ERR("cali data [%d]%d,%d,%d\n",i,x,y,z);
		if(((x-tmp_avg_x<-50)||(x-tmp_avg_x>50)||(y-tmp_avg_y<-50)||(y-tmp_avg_y>50)||(z-tmp_avg_z<-50)||(z-tmp_avg_z>50))&&(i>0)){
			skip_count++;
			GYRO_ERR("err data [%d]%d,%d,%d avg:%d,%d,%d,skip\n",skip_count,x,y,z,tmp_avg_x,tmp_avg_y,tmp_avg_z);
			if(skip_count>2){
				cali_res = -100;
				GYRO_ERR("data vary fail!!\n");
				return -1;
			}
			continue;
		}else{
			i++;
			tmp_sum_x += x;
			tmp_avg_x = tmp_sum_x/i;
			tmp_sum_y += y;
			tmp_avg_y = tmp_sum_y/i;
			tmp_sum_z += z;
			tmp_avg_z = tmp_sum_z/i;
		}
	}
	if(res<0){
		cali_res = -2;
		GYRO_ERR("fail!!\n");
		return -1;
	}else{
		cali_sw[LSM6DS3_AXIS_X]=(-1)*tmp_avg_x;
		cali_sw[LSM6DS3_AXIS_Y]=(-1)*tmp_avg_y;
		cali_sw[LSM6DS3_AXIS_Z]=(-1)*tmp_avg_z;
		cali_res = 0;
		GYRO_ERR("cali result %d,%d,%d\n",cali_sw[0],cali_sw[1],cali_sw[2]);
	}
	return 0;
}
static int int_to_byte(int *intbuf,unsigned char *charbuf,int length)
{
	int i=0,j;
	int index;
	for(i=0;i<length;i++){
		index = i*4;
		for(j=0;j<4;j++){
			charbuf[index+j] = (unsigned char)((intbuf[i]>>(j*8))&0x000000FF);
		}
		GYRO_ERR("%08x: %02x %02x %02x %02x\n",intbuf[i],charbuf[index+3],charbuf[index+2],charbuf[index+1],charbuf[index]);
	}
	return 0;
}
static ssize_t show_FIFOdata(struct device_driver *ddri, char *buf)
{
	static unsigned char bytebuf[FIFO_LEN]={0};

	int_to_byte(fifodata,bytebuf,FIFO_LEN/4);
	//GYRO_ERR("bytebuf:%s\n", bytebuf);
	return sprintf(buf, "%s\n",bytebuf);
}

static ssize_t show_FIFOtest(struct device_driver *ddri, char *buf)
{
	struct lsm6ds3_gyro_i2c_data *data = obj_i2c_data;
	if(FIFOtestResult<0){
		return sprintf(buf, "%d\n",FIFOtestResult);
	}else{
		if(FIFOinit){
			return sprintf(buf, "%d\n",LSM6DS3_readfromFIFO(data->client,fifodata,FIFO_LEN));
		}else{
			return sprintf(buf, "FIFO not init\n");
		}
	}
	return sprintf(buf, "%d\n",LSM6DS3_readfromFIFO(data->client,fifodata,FIFO_LEN));
}
/*----------------------------------------------------------------------------*/
static ssize_t store_FIFOtest(struct device_driver *ddri, const char *buf, size_t count)
{
	int en = 0;
	int try = 100;
	//int i;
	int length;
	struct lsm6ds3_gyro_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GYRO_ERR("i2c_data obj is null!!\n");
		return count;
	}

	GYRO_FUN();

	if (sscanf(buf, "%d", &en)) {
		GYRO_ERR("%d\n", en);

		switch(en){
			case 1:
				if(!FIFOinit){
					FIFOtestResult += LSM6DS3_initFIFO(data->client);
				}
				break;
			case 2:
				while(try--){
					length = LSM6DS3_readfromFIFO(data->client,fifodata,FIFO_LEN);
					/*for(i=0;i<length*7;i+=7){
						GYRO_ERR("%d %d %d %d %d %d %d\n",fifodata[i], fifodata[i+1], fifodata[i+2],
								fifodata[i+3], fifodata[i+4], fifodata[i+5], fifodata[i+6]);
					}*/
					msleep(20);
				}
			case 3:
				if(FIFOinit){
					FIFOtestResult += LSM6DS3_destroyFIFO(data->client);
				}
				break;
			case 4:
				if(!FIFOinit){
					FIFOtestResult += LSM6DS3_initFIFO(data->client);
					msleep(30);
				}
				lsm6ds3_do_cali(data->client,data->fifo_cali_sw);
				while(try--){
					length = LSM6DS3_readfromFIFO(data->client,fifodata,FIFO_LEN);
					msleep(20);
				}
				if(FIFOinit){
					FIFOtestResult += LSM6DS3_destroyFIFO(data->client);
				}
				break;
			default:
				FIFOtestResult = -1;
				break;
		}
	} else {
		GYRO_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	char strbuf[LSM6DS3_BUFSIZE];
	if (NULL == client) {

		GYRO_ERR("i2c client is null!!\n");
		return 0;
	}

	LSM6DS3_ReadChipInfo(client, strbuf, LSM6DS3_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	char strbuf[LSM6DS3_BUFSIZE];

	if (NULL == client) {

		GYRO_ERR("i2c client is null!!\n");
		return 0;
	}

	LSM6DS3_ReadGyroData(client, strbuf, LSM6DS3_BUFSIZE);

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6ds3_gyro_i2c_data *obj = obj_i2c_data;
	if (obj == NULL) {

		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_gyro_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL) {

		GYRO_ERR("i2c_data obj is null!!\n");
		return count;
	}

	if (1 == sscanf(buf, "0x%x", &trace)) {

		atomic_set(&obj->trace, trace);
	} else {

		GYRO_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct lsm6ds3_gyro_i2c_data *obj = obj_i2c_data;
	if (obj == NULL) {

		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw) {

		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);
	} else {

		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;
}

static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6ds3_gyro_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GYRO_ERR("i2c_data obj is null!!\n");
		return -1;
	}

	res = sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction, atomic_read(&data->layout), data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int layout = 0;
	struct lsm6ds3_gyro_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GYRO_ERR("i2c_data obj is null!!\n");
		return count;
	}

	if (1 == sscanf(buf, "%d", &layout)) {

		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt)) {

			GYRO_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(data->hw->direction, &data->cvt)) {
			GYRO_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		} else {

			GYRO_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else {

		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}

static ssize_t show_odr(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	u8 databuf[2];
	u8 odr;
	struct lsm6ds3_gyro_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GYRO_ERR("i2c_data obj is null!!\n");
		return -1;
	}

	if (hwmsen_read_byte(data->client, LSM6DS3_CTRL2_G, databuf)) {
		GYRO_ERR("read gyro data format register err!\n");
		return LSM6DS3_ERR_I2C;
	} else {
		GYRO_LOG("read  gyro data format register: 0x%x\n", databuf[0]);
	}

	odr = databuf[0]&LSM6DS3_GYRO_ODR_MASK;
	switch(odr){
		case LSM6DS3_GYRO_ODR_13HZ:
			res = sprintf(buf, "0:13HZ\n1:104HZ\n2:208HZ\n3:1660HZ\n4:other\ncurrent:LSM6DS3_GYRO_ODR_13HZ\n");
			break;
		case LSM6DS3_GYRO_ODR_104HZ:
			res = sprintf(buf, "0:13HZ\n1:104HZ\n2:208HZ\n3:1660HZ\n4:other\ncurrent:LSM6DS3_GYRO_ODR_104HZ\n");
			break;
		case LSM6DS3_GYRO_ODR_208HZ:
			res = sprintf(buf, "0:13HZ\n1:104HZ\n2:208HZ\n3:1660HZ\n4:other\ncurrent:LSM6DS3_GYRO_ODR_208HZ\n");
			break;
		case LSM6DS3_GYRO_ODR_1660HZ:
			res = sprintf(buf, "0:13HZ\n1:104HZ\n2:208HZ\n3:1660HZ\n4:other\ncurrent:LSM6DS3_GYRO_ODR_1660HZ\n");
			break;
		default:
			res = sprintf(buf, "0:13HZ\n1:104HZ\n2:208HZ\n3:1660HZ\n4:other\ncurrent:other 0x%02x\n",odr);
			break;
	}

	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_odr(struct device_driver *ddri, const char *buf, size_t count)
{
	int odr_sel = 0;
	u8 odr = LSM6DS3_GYRO_ODR_104HZ;
	struct lsm6ds3_gyro_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GYRO_ERR("i2c_data obj is null!!\n");
		return count;
	}

	if (1 == sscanf(buf, "%d", &odr_sel)) {
		switch(odr_sel){
			case 0:
				odr = LSM6DS3_GYRO_ODR_13HZ;
				break;
			case 1:
				odr = LSM6DS3_GYRO_ODR_104HZ;
				break;
			case 2:
				odr = LSM6DS3_GYRO_ODR_208HZ;
				break;
			case 3:
				odr = LSM6DS3_GYRO_ODR_1660HZ;
				break;
			default:
				odr = LSM6DS3_GYRO_ODR_104HZ;
				break;
		}

		LSM6DS3_gyro_SetSampleRate(data->client, odr);
	} else {

		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}

static ssize_t show_gyro_cali(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n",cali_res);;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_gyro_cali(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_gyro_i2c_data *obj = obj_i2c_data;

	GYRO_FUN();
	if (NULL == obj) {
		GYRO_ERR("i2c obj is null!!\n");
		return count;
	}

	lsm6ds3_do_cali(obj->client,obj->cali_sw);

	return count;
}

static ssize_t show_gyro_offset(struct device_driver *ddri, char *buf)
{
	struct lsm6ds3_gyro_i2c_data *obj = obj_i2c_data;

	GYRO_FUN();
	if (NULL == obj) {
		GYRO_ERR("i2c obj is null!!\n");
		return 0;
	}

	return sprintf(buf, "%d %d %d\n",obj->cali_sw[LSM6DS3_AXIS_X],obj->cali_sw[LSM6DS3_AXIS_Y],obj->cali_sw[LSM6DS3_AXIS_Z]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_gyro_offset(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_gyro_i2c_data *obj = obj_i2c_data;
	int x,y,z;
	GYRO_FUN();
	if (NULL == obj) {
		GYRO_ERR("i2c obj is null!!\n");
		return count;
	}
	if(sscanf(buf, "%d %d %d", &x, &y, &z)==3){
		obj->cali_sw[LSM6DS3_AXIS_X]=x;
		obj->cali_sw[LSM6DS3_AXIS_Y]=y;
		obj->cali_sw[LSM6DS3_AXIS_Z]=z;
		GYRO_ERR("store offset %d,%d,%d\n",x,y,z);
	}else{
		GYRO_ERR("invalid input\n");
	}
	return count;
}

/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(trace,      S_IWUGO | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(odr,     S_IRUGO | S_IWUSR, show_odr, store_odr);
static DRIVER_ATTR(FIFOtest,     S_IRUGO | S_IWUSR, show_FIFOtest, store_FIFOtest);
static DRIVER_ATTR(cali,     S_IRUGO | S_IWUSR, show_gyro_cali, store_gyro_cali);
static DRIVER_ATTR(offset,     S_IRUGO | S_IWUSR, show_gyro_offset, store_gyro_offset);
static DRIVER_ATTR(FIFOdata,               S_IRUGO, show_FIFOdata,        NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *LSM6DS3_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_layout,
	&driver_attr_odr,
	&driver_attr_FIFOtest,
	&driver_attr_FIFOdata,
	&driver_attr_cali,
	&driver_attr_offset,
};
/*----------------------------------------------------------------------------*/
static int lsm6ds3_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(LSM6DS3_attr_list)/sizeof(LSM6DS3_attr_list[0]));
	if (driver == NULL) {

		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, LSM6DS3_attr_list[idx]);
		if (0 != err) {

			GYRO_ERR("driver_create_file (%s) = %d\n", LSM6DS3_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(LSM6DS3_attr_list)/sizeof(LSM6DS3_attr_list[0]));

	if (driver == NULL) {

		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {

		driver_remove_file(driver, LSM6DS3_attr_list[idx]);
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_gyro_init_client(struct i2c_client *client, bool enable)
{
	struct lsm6ds3_gyro_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	GYRO_LOG("%s lsm6ds3 addr %x!\n", __FUNCTION__, client->addr);

	res = LSM6DS3_CheckDeviceID(client);
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	res = LSM6DS3_Set_RegInc(client, true);
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	res = LSM6DS3_gyro_SetFullScale(client, LSM6DS3_GYRO_RANGE_2000DPS);/*we have only this choice*/
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}


	res = LSM6DS3_gyro_SetSampleRate(client, LSM6DS3_GYRO_ODR_104HZ);
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}
	res = LSM6DS3_gyro_SetPowerMode(client, enable);
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	GYRO_LOG("LSM6DS3_gyro_init_client OK!\n");
	/*acc setting*/


#ifdef CONFIG_LSM6DS3_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
#ifdef LSM6DS3_GYRO_NEW_ARCH
static int lsm6ds3_gyro_open_report_data(int open)
{
    /*should queuq work to report event if  is_report_input_direct=true*/
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL*/

static int lsm6ds3_gyro_enable_nodata(int en)
{
	int value = en;
	int err = 0;
	struct lsm6ds3_gyro_i2c_data *priv = obj_i2c_data;

	if (priv == NULL) {

		GYRO_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	if (value == 1) {

		enable_status = true;
	} else {

		enable_status = false;
	}
	GYRO_LOG("enable value=%d, sensor_power =%d\n", value, sensor_power);
	if (((value == 0) && (sensor_power == false)) || ((value == 1) && (sensor_power == true))) {

		GYRO_LOG("Gsensor device have updated!\n");
	} else {

		err = LSM6DS3_gyro_SetPowerMode(priv->client, enable_status);
	}

    GYRO_LOG("lsm6ds3_gyro_enable_nodata OK!\n");
    return err;
}

static int lsm6ds3_gyro_set_delay(u64 ns)
{
    int value = 0;

	struct lsm6ds3_gyro_i2c_data *priv = obj_i2c_data;

	if (priv == NULL) {

		GYRO_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	value = (int)ns/1000/1000;

    GYRO_LOG("lsm6ds3_gyro_set_delay (%d), chip only use 1024HZ \n", value);
    return 0;
}

static int lsm6ds3_gyro_get_data(int *x, int *y, int *z, int *status)
{
    char buff[LSM6DS3_BUFSIZE];
	struct lsm6ds3_gyro_i2c_data *priv = obj_i2c_data;

	if (priv == NULL) {

		GYRO_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	if (atomic_read(&priv->trace) & GYRO_TRC_DATA) {

		GYRO_LOG("%s (%d),	\n", __FUNCTION__, __LINE__);
	}
	memset(buff, 0, sizeof(buff));
	LSM6DS3_ReadGyroData(priv->client, buff, LSM6DS3_BUFSIZE);

	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return 0;
}
#endif

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int lsm6ds3_open(struct inode *inode, struct file *file)
{
	file->private_data = lsm6ds3_i2c_client;

	if (file->private_data == NULL) {

		GYRO_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_COMPAT
static long lsm6ds3_gyro_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GYROSCOPE_IOCTL_INIT:
			 if (arg32 == NULL) {

				 GYRO_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_INIT,
							(unsigned long)arg32);
			 if (ret) {
				GYRO_ERR("GYROSCOPE_IOCTL_INIT unlocked_ioctl failed.");
				return ret;
			 }

			 break;

	case COMPAT_GYROSCOPE_IOCTL_SET_CALI:
			 if (arg32 == NULL) {

				 GYRO_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_SET_CALI,
							(unsigned long)arg32);
			 if (ret) {
				GYRO_ERR("GYROSCOPE_IOCTL_SET_CALI unlocked_ioctl failed.");
				return ret;
			 }

			 break;

	case COMPAT_GYROSCOPE_IOCTL_CLR_CALI:
			 if (arg32 == NULL) {

				 GYRO_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_CLR_CALI,
							(unsigned long)arg32);
			 if (ret) {
				GYRO_ERR("GYROSCOPE_IOCTL_CLR_CALI unlocked_ioctl failed.");
				return ret;
			 }

			 break;

	case COMPAT_GYROSCOPE_IOCTL_GET_CALI:
			 if (arg32 == NULL) {

				 GYRO_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_GET_CALI,
							(unsigned long)arg32);
			 if (ret) {
				GYRO_ERR("GYROSCOPE_IOCTL_GET_CALI unlocked_ioctl failed.");
				return ret;
			 }

			 break;

	case COMPAT_GYROSCOPE_IOCTL_READ_VENDORDIV:
			 if (arg32 == NULL) {

				 GYRO_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_READ_VENDORDIV,
							(unsigned long)arg32);
			 if (ret) {
				GYRO_ERR("GYROSCOPE_IOCTL_READ_VENDORDIV unlocked_ioctl failed.\n");
				return ret;
			 }

			break;

	case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA:
			 if (arg32 == NULL) {

				 GYRO_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_READ_SENSORDATA,
							(unsigned long)arg32);
			 if (ret) {
				GYRO_ERR("COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA unlocked_ioctl failed.\n");
				return ret;
			 }

			break;
	case COMPAT_GYROSCOPE_IOCTL_FIFO_INIT:
			 if (arg32 == NULL) {

				 GYRO_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_FIFO_INIT,
							(unsigned long)arg32);
			 if (ret) {
				GYRO_ERR("COMPAT_GYROSCOPE_IOCTL_FIFO_INIT unlocked_ioctl failed.\n");
				return ret;
			 }

			 break;

	case COMPAT_GYROSCOPE_IOCTL_GET_FIFODATA:
			 if (arg32 == NULL) {

				 GYRO_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_GET_FIFODATA,
							(unsigned long)arg32);
			 if (ret) {
				GYRO_ERR("COMPAT_GYROSCOPE_IOCTL_GET_FIFODATA unlocked_ioctl failed.\n");
				return ret;
			 }

			 break;

	case COMPAT_GYROSCOPE_IOCTL_GET_FIFOLENGTH:
			 if (arg32 == NULL) {

				 GYRO_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 ret = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_GET_FIFOLENGTH,
							(unsigned long)arg32);
			 if (ret) {
				GYRO_ERR("GYROSCOPE_IOCTL_GET_FIFOLENGTH unlocked_ioctl failed.");
				return ret;
			 }

			 break;

	default:
			 printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			 return -ENOIOCTLCMD;
			 break;
	}
	return ret;
}
#endif

static long lsm6ds3_gyro_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;

	char strbuf[LSM6DS3_BUFSIZE] = {0};
	void __user *data;
	int err = 0;
	int copy_cnt = 0;
	SENSOR_DATA sensor_data;
	int cali[3] = {0};
	int smtRes = 0;
	int vendordiv;
	uint32_t fifoinitcmd;
	int length;
	/*GYRO_FUN();*/

	if (_IOC_DIR(cmd) & _IOC_READ) {

		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {

		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err) {

		GYRO_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {

	case GYROSCOPE_IOCTL_INIT:
			LSM6DS3_gyro_init_client(client, false);
			break;

	case GYROSCOPE_IOCTL_SMT_DATA:
			data = (void __user *) arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}

			GYRO_LOG("IOCTL smtRes: %d!\n", smtRes);
			copy_cnt = copy_to_user(data, &smtRes,  sizeof(smtRes));

			if (copy_cnt) {

				err = -EFAULT;
				GYRO_ERR("copy gyro data to user failed!\n");
				break;
			}
			GYRO_LOG("copy gyro data to user OK: %d!\n", copy_cnt);
			break;

	case GYROSCOPE_IOCTL_READ_VENDORDIV:
			data = (void __user *) arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}
			vendordiv = DEGREE_TO_RAD;
			GYRO_LOG("IOCTL VENDORDIV: %d!\n", vendordiv);
			copy_cnt = copy_to_user(data, &vendordiv,  sizeof(vendordiv));

			if (copy_cnt) {

				err = -EFAULT;
				GYRO_ERR("copy gyro vendordiv to user failed!\n");
				break;
			}
			GYRO_LOG("copy gyro vendordiv to user OK: %d!\n", copy_cnt);
			break;

	case GYROSCOPE_IOCTL_READ_SENSORDATA:
			data = (void __user *)arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}

			LSM6DS3_ReadGyroData(client, strbuf, LSM6DS3_BUFSIZE);
			if (copy_to_user(data, strbuf, sizeof(strbuf))) {
				err = -EFAULT;
				break;
			}
			break;

	case GYROSCOPE_IOCTL_SET_CALI:
			data = (void __user *)arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
				err = -EFAULT;
				break;
			} else {
				cali[LSM6DS3_AXIS_X] = (s64)(sensor_data.x);/* * 180*1000*1000/(LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142);*/
				cali[LSM6DS3_AXIS_Y] = (s64)(sensor_data.y);/* * 180*1000*1000/(LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142);*/
				cali[LSM6DS3_AXIS_Z] = (s64)(sensor_data.z);/* * 180*1000*1000/(LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142); 			*/
				err = LSM6DS3_gyro_WriteCalibration(client, cali);
			}
			break;

	case GYROSCOPE_IOCTL_CLR_CALI:
			err = LSM6DS3_gyro_ResetCalibration(client);
			break;

	case GYROSCOPE_IOCTL_GET_CALI:
			data = (void __user *)arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			err = LSM6DS3_gyro_ReadCalibration(client, cali);
			if (err) {
				break;
			}

			sensor_data.x = (s64)(cali[LSM6DS3_AXIS_X]);/* * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142/(180*1000*1000);*/
			sensor_data.y = (s64)(cali[LSM6DS3_AXIS_Y]);/* * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142/(180*1000*1000);*/
			sensor_data.z = (s64)(cali[LSM6DS3_AXIS_Z]);/* * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142/(180*1000*1000); */

			if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
				err = -EFAULT;
				break;
			}
			break;

	case GYROSCOPE_IOCTL_FIFO_INIT:
			data = (void __user *)arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&fifoinitcmd, data, sizeof(fifoinitcmd))) {
				err = -EFAULT;
				GYRO_ERR("GYROSCOPE_IOCTL_FIFO_INIT copy_from_user failed.%d\n",fifoinitcmd);
			} else {
				if(fifoinitcmd){
					err = LSM6DS3_initFIFO(client);
				}else{
					err = LSM6DS3_destroyFIFO(client);
				}
				GYRO_ERR("GYROSCOPE_IOCTL_FIFO_INIT cmd:%d err:%d\n",fifoinitcmd,err);
			}
			break;

	case GYROSCOPE_IOCTL_GET_FIFOLENGTH:
			data = (void __user *)arg;
			if (data == NULL) {
				err = -EINVAL;
				GYRO_ERR("GYROSCOPE_IOCTL_GET_FIFOLENGTH data is null\n");
				break;
			}
			length = LSM6DS3_readfromFIFO(client,fifodata,FIFO_LEN);

			if (copy_to_user(data, &length, sizeof(length))) {
				err = -EFAULT;
				GYRO_ERR("GYROSCOPE_IOCTL_GET_FIFOLENGTH copy_to_user fail,len:%d\n",length);
			}
			break;

	case GYROSCOPE_IOCTL_GET_FIFODATA:
			data = (void __user *)arg;
			if (data == NULL) {
				err = -EINVAL;
				break;
			}
			if (copy_to_user(data, fifodata, sizeof(fifodata))) {
				err = -EFAULT;
				break;
			}
			break;

	default:
			GYRO_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
	}
	return err;
}

#if 1
/*----------------------------------------------------------------------------*/
static struct file_operations lsm6ds3_gyro_fops = {
	.owner = THIS_MODULE,
	.open = lsm6ds3_open,
	.release = lsm6ds3_release,
	.unlocked_ioctl = lsm6ds3_gyro_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = lsm6ds3_gyro_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice lsm6ds3_gyro_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gyroscope",
	.fops = &lsm6ds3_gyro_fops,
};
#endif

/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct lsm6ds3_gyro_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GYRO_FUN();

	if (msg.event == PM_EVENT_SUSPEND) {

		if (obj == NULL) {

			GYRO_ERR("null pointer!!\n");
			return -1;
		}
		atomic_set(&obj->suspend, 1);
		err = LSM6DS3_gyro_SetPowerMode(obj->client, false);
		if (err) {

			GYRO_ERR("write power control fail!!\n");
			return err;
		}

		sensor_power = false;

		LSM6DS3_power(obj->hw, 0);

	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_resume(struct i2c_client *client)
{
	struct lsm6ds3_gyro_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	GYRO_FUN();

	if (obj == NULL) {

		GYRO_ERR("null pointer!!\n");
		return -EINVAL;
	}

	LSM6DS3_power(obj->hw, 1);

	err = LSM6DS3_gyro_SetPowerMode(obj->client, enable_status);
	if (err) {

		GYRO_ERR("initialize client fail! err code %d!\n", err);
		return err;
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void lsm6ds3_gyro_early_suspend(struct early_suspend *h)
{
	struct lsm6ds3_gyro_i2c_data *obj = container_of(h, struct lsm6ds3_gyro_i2c_data, early_drv);
	int err;
	GYRO_FUN();

	if (obj == NULL) {

		GYRO_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1);
	err = LSM6DS3_gyro_SetPowerMode(obj->client, false);
	if (err) {

		GYRO_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;

	LSM6DS3_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void lsm6ds3_gyro_late_resume(struct early_suspend *h)
{
	struct lsm6ds3_gyro_i2c_data *obj = container_of(h, struct lsm6ds3_gyro_i2c_data, early_drv);
	int err;
	GYRO_FUN();

	if (obj == NULL) {

		GYRO_ERR("null pointer!!\n");
		return;
	}

	LSM6DS3_power(obj->hw, 1);
	err = LSM6DS3_gyro_SetPowerMode(obj->client, enable_status);
	if (err) {

		GYRO_ERR("initialize client fail! err code %d!\n", err);
		return;
	}
	atomic_set(&obj->suspend, 0);
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct lsm6ds3_gyro_i2c_data *obj;

#ifdef LSM6DS3_GYRO_NEW_ARCH
	struct gyro_control_path ctl = {0};
    struct gyro_data_path data = {0};
#else
	struct hwmsen_object gyro_sobj;
#endif
	int err = 0;
	GYRO_FUN();
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);

	if (!obj) {

		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct lsm6ds3_gyro_i2c_data));

	obj->hw =lsm6ds3_get_cust_gyro_hw();
	atomic_set(&obj->layout, obj->hw->direction);
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {

		GYRO_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	client->addr = 0xD4 >> 1;
	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	lsm6ds3_i2c_client = new_client;
	err = LSM6DS3_gyro_init_client(new_client, false);
	if (err) {

		goto exit_init_failed;
	}

#if 1
	err = misc_register(&lsm6ds3_gyro_device);
	if (err) {

		GYRO_ERR("lsm6ds3_gyro_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}
#endif

#ifdef LSM6DS3_GYRO_NEW_ARCH
	err = lsm6ds3_create_attr(&(lsm6ds3_gyro_init_info.platform_diver_addr->driver));
#else
	err = lsm6ds3_create_attr(&lsm6ds3_driver.driver);
#endif
	if (err) {

		GYRO_ERR("lsm6ds3 create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

    ctl.open_report_data = lsm6ds3_gyro_open_report_data;
    ctl.enable_nodata = lsm6ds3_gyro_enable_nodata;
    ctl.set_delay  = lsm6ds3_gyro_set_delay;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = obj->hw->is_batch_supported;

    err = gyro_register_control_path(&ctl);
    if (err) {

		GYRO_ERR("register acc control path err\n");
		goto exit_kfree;
	}

    data.get_data = lsm6ds3_gyro_get_data;
    data.vender_div = DEGREE_TO_RAD;
    err = gyro_register_data_path(&data);
    if (err) {

		GYRO_ERR("register acc data path err= %d\n", err);
		goto exit_kfree;
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = lsm6ds3_gyro_early_suspend,
	obj->early_drv.resume   = lsm6ds3_gyro_late_resume,
	register_early_suspend(&obj->early_drv);
#endif
#ifdef LSM6DS3_GYRO_NEW_ARCH
	lsm6ds3_gyro_init_flag = 0;
#endif
	gyro_dev = GYRO_LSM6DS3;
	register_device_proc("Sensor_gyro", "lsm6ds3", "ST");
	GYRO_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&lsm6ds3_gyro_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	/*i2c_detach_client(new_client);*/
	exit_kfree:
	kfree(obj);
	exit:
#ifdef LSM6DS3_GYRO_NEW_ARCH
	lsm6ds3_gyro_init_flag = -1;
#endif
	GYRO_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = lsm6ds3_delete_attr(&(lsm6ds3_gyro_init_info.platform_diver_addr->driver));

	if (err) {

		GYRO_ERR("lsm6ds3_gyro_i2c_remove fail: %d\n", err);
	}

	#if 1
	err = misc_deregister(&lsm6ds3_gyro_device);
	if (err) {

		GYRO_ERR("misc_deregister lsm6ds3_gyro_device fail: %d\n", err);
	}
	#endif

	lsm6ds3_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_local_init(struct platform_device *pdev)
{
	struct gyro_hw *gy_hw =lsm6ds3_get_cust_gyro_hw();
	GYRO_FUN();

	LSM6DS3_power(gy_hw, 1);

	if (i2c_add_driver(&lsm6ds3_gyro_i2c_driver)) {

		GYRO_ERR("add driver error\n");
		return -1;
	}
	if (lsm6ds3_gyro_init_flag == -1) {

		GYRO_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	}

	return 0;
}
static int lsm6ds3_gyro_local_uninit(void)
{
    struct gyro_hw *gy_hw =lsm6ds3_get_cust_gyro_hw();

    GYRO_FUN();

    LSM6DS3_power(gy_hw, 0);
    i2c_del_driver(&lsm6ds3_gyro_i2c_driver);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init lsm6ds3_gyro_init(void)
{
	/*GYRO_FUN();*/
	struct gyro_hw *hw = lsm6ds3_get_cust_gyro_hw();
    GYRO_LOG("%s: i2c_number=%d\n", __func__, hw->i2c_num);
    i2c_register_board_info(hw->i2c_num, &i2c_lsm6ds3_gyro, 1);

	lsm6ds3_i2c_client = NULL;	/*initial in module_init*/

	obj_i2c_data = NULL;	/*initial in module_init*/
	sensor_power = false;	/*initial in module_init*/
	enable_status = false;	/*initial in module_init*/

    gyro_driver_add(&lsm6ds3_gyro_init_info);

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit lsm6ds3_gyro_exit(void)
{
	GYRO_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(lsm6ds3_gyro_init);
module_exit(lsm6ds3_gyro_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LSM6DS3 Accelerometer and gyroscope driver");
MODULE_AUTHOR("xj.wang@mediatek.com, darren.han@st.com");






/*----------------------------------------------------------------- LSM6DS3 ------------------------------------------------------------------*/
