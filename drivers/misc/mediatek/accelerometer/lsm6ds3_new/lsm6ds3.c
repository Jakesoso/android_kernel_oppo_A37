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
#include "lsm6ds3.h"
#include <linux/hwmsen_helper.h>
#include <linux/kernel.h>
#include <mach/mt_pm_ldo.h>
#include <cust_eint.h>
#include <mach/eint.h>

#include <cust_acc.h>
#include <accel.h>
#include <step_counter.h>
#include <tilt_detector.h>


#define POWER_NONE_MACRO MT65XX_POWER_NONE

#define LSM6DS3_EMBEDED_FUNC 1

#if (LSM6DS3_EMBEDED_FUNC)

#if defined(CONFIG_CUSTOM_KERNEL_STEP_COUNTER)
#define LSM6DS3_STEP_COUNTER 1
#endif //CONFIG_CUSTOM_KERNEL_STEP_COUNTER

#if defined(CONFIG_CUSTOM_KERNEL_TILT_DETECTOR_SENSOR)
#define LSM6DS3_TILT_FUNC
#endif

#endif

/*---------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_LSM6DS3_LOWPASS   /*apply low pass filter on output*/
/*----------------------------------------------------------------------------*/
#define LSM6DS3_AXIS_X          0
#define LSM6DS3_AXIS_Y          1
#define LSM6DS3_AXIS_Z          2
#define LSM6DS3_ACC_AXES_NUM        3
#define LSM6DS3_GYRO_AXES_NUM       3
#define LSM6DS3_ACC_DATA_LEN        6
#define LSM6DS3_GYRO_DATA_LEN       6
#define LSM6DS3_ACC_DEV_NAME        "LSM6DS3_ACCEL"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id lsm6ds3_i2c_id[] = {{LSM6DS3_ACC_DEV_NAME, 0}, { } };
static struct i2c_board_info __initdata i2c_lsm6ds3 = { I2C_BOARD_INFO(LSM6DS3_ACC_DEV_NAME, (0xD4>>1))};


/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lsm6ds3_i2c_remove(struct i2c_client *client);
/*static int lsm6ds3_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);*/

#ifndef LSM6DS3_USE_EARLYSUSPEND

static int lsm6ds3_acc_suspend(struct i2c_client *client, pm_message_t msg);
static int lsm6ds3_acc_resume(struct i2c_client *client);
#endif

static int LSM6DS3_init_client(struct i2c_client *client, bool enable);
static int LSM6DS3_acc_SetPowerMode(struct i2c_client *client, bool enable);
static int LSM6DS3_acc_SetSampleRate(struct i2c_client *client, u8 sample_rate);
#ifdef LSM6DS3_STEP_COUNTER
static u64 pre_step = 0;
static int step_poll_count = 0;
static bool first_data_after_reset = false;
static int detector_step = -1;
static int aim_detector_step = 0;
static int lsm6ds3_report_detector(void);
static int LSM6DS3_Reset_Pedo_Data(struct i2c_client *client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t newValue);
static void report_single_detector_event(struct work_struct *work);
static DECLARE_DELAYED_WORK(report_single_detector_worker, report_single_detector_event);
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
    s16 raw[C_MAX_FIR_LENGTH][LSM6DS3_ACC_AXES_NUM];
    int sum[LSM6DS3_ACC_AXES_NUM];
    int num;
    int idx;
};
struct gyro_data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LSM6DS3_GYRO_AXES_NUM];
    int sum[LSM6DS3_GYRO_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct lsm6ds3_i2c_data {
    struct i2c_client *client;
	struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    atomic_t 				layout;
    /*misc*/
#ifdef LSM6DS3_EMBEDED_FUNC
    struct work_struct	eint_work;
#endif
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t				filter;
    s16                     cali_sw[LSM6DS3_GYRO_AXES_NUM+1];

    /*data*/
	s8                      offset[LSM6DS3_ACC_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[LSM6DS3_ACC_AXES_NUM+1];

	int 					sensitivity;
	int 					sample_rate;

#if defined(CONFIG_LSM6DS3_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif
    /*early suspend*/
#if defined(LSM6DS3_USE_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver lsm6ds3_i2c_driver = {
    .driver = {
		.owner			= THIS_MODULE,
		.name			= LSM6DS3_ACC_DEV_NAME,
    },
	.probe				= lsm6ds3_i2c_probe,
	.remove				= lsm6ds3_i2c_remove,
#if !defined(LSM6DS3_USE_EARLYSUSPEND)
	.suspend			= lsm6ds3_acc_suspend,
	.resume				= lsm6ds3_acc_resume,
#endif
	.id_table = lsm6ds3_i2c_id,
};

static int lsm6ds3_local_init(void);
static int lsm6ds3_local_uninit(void);
static int lsm6ds3_local_init_common(void);
static int LSM6DS3_W_Open_RAM_Page(struct i2c_client *client, LSM6DS3_ACC_GYRO_RAM_PAGE_t newValue);

static int lsm6ds3_acc_init_flag = -1;			/*initial in module_init     = -1;*/
static u64 step_base_num = 0;
static unsigned long lsm6ds3_init_flag_test = 0;	/*nitial in module_init    = 0; initial state*/
static DEFINE_MUTEX(lsm6ds3_init_mutex);

typedef enum {
	LSM6DS3_ACC = 1,
	LSM6DS3_STEP_C = 2,
	LSM6DS3_TILT = 3,
} LSM6DS3_INIT_TYPE;

static struct acc_init_info  lsm6ds3_init_info = {
	.name   = LSM6DS3_ACC_DEV_NAME,
	.init   = lsm6ds3_local_init,
	.uninit = lsm6ds3_local_uninit,
};

/*----------------------------------------------------------------------------*/

static struct lsm6ds3_i2c_data *obj_i2c_data = NULL;	/*initial in module_init      = NULL;*/
static struct i2c_client *lsm6ds3_i2c_client = NULL;	/*initial in module_init      = NULL;*/
static bool sensor_power = false;				/*initial in module_init      = false;*/
static bool enable_status = false;				/*initial in module_init      = false;*/
static bool pedo_enable_status = false;			/*initial in module_init      = false;*/
static bool pedo_enable_sig_status = false;
static int detector_enable_status = 0;
static int pedo_enable_count=0;
static bool tilt_enable_status = false;			/*initial in module_init      = false;*/


/*----------------------------------------------------------------------------*/

#define GSE_TAG                  "[accel] "

#define GSE_FUN(f)               printk(KERN_ERR GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_ERR GSE_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)

/*----------------------------------------------------------------------------*/

static void LSM6DS3_dumpReg(struct i2c_client *client)
{
	int i = 0;
	u8 addr = 0x10;
	u8 regdata = 0;

	for (i = 0; i < 25; i++) {
	/*dump all*/
		hwmsen_read_byte(client, addr, &regdata);
		HWM_LOG("Reg addr=%x regdata=%x\n", addr, regdata);
		addr++;
	}
}

static void LSM6DS3_power(struct acc_hw *hw, unsigned int on)
{
	static unsigned int power_on;	/* default= 0;*/

	if (hw->power_id != POWER_NONE_MACRO) {		/* have externel LDO*/

		GSE_LOG("power %s\n", on ? "on" : "off");
		if (power_on == on) {	/*power status not change*/

			GSE_LOG("ignore power control: %d\n", on);
		} else if (on) {	/* power on*/

			if (!hwPowerOn(hw->power_id, hw->power_vol, "LSM6DS3"))	{

				GSE_ERR("power on fails!!\n");
			}
		} else {	/* power off*/

			if (!hwPowerDown(hw->power_id, "LSM6DS3")) {

				GSE_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/

static int LSM6DS3_acc_write_rel_calibration(struct lsm6ds3_i2c_data *obj, int dat[LSM6DS3_GYRO_AXES_NUM])
{
    obj->cali_sw[LSM6DS3_AXIS_X] = obj->cvt.sign[LSM6DS3_AXIS_X]*dat[obj->cvt.map[LSM6DS3_AXIS_X]];
    obj->cali_sw[LSM6DS3_AXIS_Y] = obj->cvt.sign[LSM6DS3_AXIS_Y]*dat[obj->cvt.map[LSM6DS3_AXIS_Y]];
    obj->cali_sw[LSM6DS3_AXIS_Z] = obj->cvt.sign[LSM6DS3_AXIS_Z]*dat[obj->cvt.map[LSM6DS3_AXIS_Z]];
#if DEBUG
		if (atomic_read(&obj->trace) & ACCEL_TRC_CALI) {

			GSE_LOG("test  (%5d, %5d, %5d) ->(%5d, %5d, %5d)->(%5d, %5d, %5d))\n",
				obj->cvt.sign[LSM6DS3_AXIS_X], obj->cvt.sign[LSM6DS3_AXIS_Y], obj->cvt.sign[LSM6DS3_AXIS_Z],
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z],
				obj->cvt.map[LSM6DS3_AXIS_X], obj->cvt.map[LSM6DS3_AXIS_Y], obj->cvt.map[LSM6DS3_AXIS_Z]);
			GSE_LOG("write gyro calibration data  (%5d, %5d, %5d)\n",
				obj->cali_sw[LSM6DS3_AXIS_X], obj->cali_sw[LSM6DS3_AXIS_Y], obj->cali_sw[LSM6DS3_AXIS_Z]);
		}
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_ResetCalibration(struct i2c_client *client)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_ReadCalibration(struct i2c_client *client, int dat[LSM6DS3_GYRO_AXES_NUM])
{
    struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->cali_sw[LSM6DS3_AXIS_X];
    dat[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->cali_sw[LSM6DS3_AXIS_Y];
    dat[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->cali_sw[LSM6DS3_AXIS_Z];

#if DEBUG
		if (atomic_read(&obj->trace) & ACCEL_TRC_CALI) {

			GSE_LOG("Read gyro calibration data  (%5d, %5d, %5d)\n",
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z]);
		}
#endif

	return 0;
}
/*----------------------------------------------------------------------------*/

static int LSM6DS3_acc_WriteCalibration(struct i2c_client *client, int dat[LSM6DS3_GYRO_AXES_NUM])
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[LSM6DS3_GYRO_AXES_NUM];

	GSE_FUN();
	if (!obj || !dat) {

		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	} else {

		cali[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X] * obj->cali_sw[LSM6DS3_AXIS_X];
		cali[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y] * obj->cali_sw[LSM6DS3_AXIS_Y];
		cali[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z] * obj->cali_sw[LSM6DS3_AXIS_Z];
		cali[LSM6DS3_AXIS_X] += dat[LSM6DS3_AXIS_X];
		cali[LSM6DS3_AXIS_Y] += dat[LSM6DS3_AXIS_Y];
		cali[LSM6DS3_AXIS_Z] += dat[LSM6DS3_AXIS_Z];
#if DEBUG
		if (atomic_read(&obj->trace) & ACCEL_TRC_CALI) {

			GSE_LOG("write gyro calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n",
				dat[LSM6DS3_AXIS_X], dat[LSM6DS3_AXIS_Y], dat[LSM6DS3_AXIS_Z],
				cali[LSM6DS3_AXIS_X], cali[LSM6DS3_AXIS_Y], cali[LSM6DS3_AXIS_Z]);
		}
#endif
		return LSM6DS3_acc_write_rel_calibration(obj, cali);
	}

	return err;
}
//add by zhihong.lu@BSP.sensor
static int lsm6ds3_set_reg_bit(u8 reg,u8 bit,u8 value,bool isEmbedReg){
	u8 databuf[2];
	int res = 0;
	struct i2c_client *client = lsm6ds3_i2c_client;

	if(client == NULL){
		GSE_ERR("client is null\n");
		return LSM6DS3_ERR_STATUS;
	}
	if(isEmbedReg){
		// enable embedded register
		res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_ENABLED);
		if(res < 0)
		{
			return LSM6DS3_ERR_I2C;
		}
	}

	if(hwmsen_read_byte(client, reg, databuf)){
		GSE_ERR("read register %x err!\n", reg);
		goto EXIT_FAIL;
	}
	databuf[0] &= ~bit;//clear bit
	databuf[0] |= (value&bit);
	databuf[1] = databuf[0];
	databuf[0] = reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {
		GSE_ERR("write register %x err!\n",reg);
		goto EXIT_FAIL;
	}

	if(isEmbedReg){
		// disable embedded register
		res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_DISABLED);
		if(res < 0)
		{
			return LSM6DS3_ERR_I2C;
		}
	}
	return LSM6DS3_SUCCESS;
EXIT_FAIL:
	if(isEmbedReg){
		// disable embedded register
		LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_DISABLED);
	}
	return LSM6DS3_ERR_STATUS;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	databuf[0] = LSM6DS3_FIXED_DEVID;

	res = hwmsen_read_byte(client, LSM6DS3_WHO_AM_I, databuf);
	GSE_LOG(" LSM6DS3  id %x!\n", databuf[0]);
	if (databuf[0] != LSM6DS3_FIXED_DEVID) {

		return LSM6DS3_ERR_IDENTIFICATION;
	}

	if (res < 0) {

		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/

#ifdef LSM6DS3_EMBEDED_FUNC
static int LSM6DS3_acc_Enable_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_FUNC_EN_t newValue)
{
	u8 databuf[2] = {0};
	int res = 0;
	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DS3_CTRL10_C, databuf)) {

		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_ACC_GYRO_FUNC_EN_MASK;/*clear */
	databuf[0] |= newValue;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL10_C;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}
static int irq_enable_count = 1;
static int lsm6ds3_enable_irq(bool en){
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	if(hw->is_eint_supported == 0){
		return 0;
	}
	if(en){
		if(irq_enable_count==0){
			enable_irq(hw->eint_gpio);
			if(hw->is_eint2_connected != 0){
				enable_irq(hw->eint2_gpio);
			}
		}
		irq_enable_count++;
		GSE_ERR("enable %d\n", irq_enable_count);
	}else{
		irq_enable_count--;
		GSE_ERR("disable %d\n", irq_enable_count);
		if(irq_enable_count<=0){
			disable_irq_nosync(hw->eint_gpio);
			if(hw->is_eint2_connected != 0){
				disable_irq_nosync(hw->eint2_gpio);
			}
			irq_enable_count=0;
		}
	}
	return 0;
}

static irqreturn_t lsm6ds3_interrupt(int vec, void *info)
{
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	//GSE_FUN();
	if(!priv)
	{
		return IRQ_HANDLED;
	}
	disable_irq_nosync(hw->eint_gpio);
	if(hw->is_eint2_connected != 0){
		disable_irq_nosync(hw->eint2_gpio);
	}
	schedule_work(&priv->eint_work);
	enable_irq(hw->eint_gpio);
	if(hw->is_eint2_connected != 0){
		enable_irq(hw->eint2_gpio);
	}
	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_Int_Ctrl(struct i2c_client *client, LSM6DS3_ACC_GYRO_INT_ACTIVE_t int_act, LSM6DS3_ACC_GYRO_INT_LATCH_CTL_t int_latch)
{
	u8 databuf[2] = {0};
	int res = 0;
	u8 op_reg = 0;
	GSE_FUN();

	/*config latch int or no latch*/
	op_reg = LSM6DS3_TAP_CFG;
	if (hwmsen_read_byte(client, op_reg, databuf)) {

		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  LSM6DS3_TAP_CFG register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_LATCH_CTL_MASK;/*clear */
	databuf[0] |= int_latch;

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {

		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	/* config high or low active*/
	op_reg = LSM6DS3_CTRL3_C;
	if (hwmsen_read_byte(client, op_reg, databuf)) {

		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  LSM6DS3_CTRL3_C register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_ACTIVE_MASK;/*clear */
	databuf[0] |= int_act;

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

/*----------------------------------------------------------------------------*/

static int lsm6ds3_setup_eint(void)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	int err;
	int irq,irq2;
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	
	if(hw->is_eint_supported == 0){
		GSE_ERR("eint mode is not supported.\n");
		return LSM6DS3_SUCCESS;
	}
	mt_set_gpio_mode(hw->eint_gpio, 0);
	mt_set_gpio_dir(hw->eint_gpio, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(hw->eint_gpio, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(hw->eint_gpio, GPIO_PULL_DOWN);
	err = gpio_request(hw->eint_gpio, "lsm6ds3_irq");
	if (err){
		GSE_ERR("Unable to request GPIO.\n");
		return -1;
	}

	if(hw->is_eint2_connected != 0){
		//gpio_direction_input(hw->eint_gpio);
		//to avoid leak current
		mt_set_gpio_mode(hw->eint2_gpio, 0);
		mt_set_gpio_dir(hw->eint2_gpio, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(hw->eint2_gpio, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(hw->eint2_gpio, GPIO_PULL_DOWN);
		err = gpio_request(hw->eint2_gpio, "lsm6ds3_irq2");
		if (err){
			GSE_ERR("Unable to request GPIO2.\n");
			return -1;
		}
	}
	//gpio_direction_input(hw->eint2_gpio);

	err = LSM6DS3_Int_Ctrl(client, LSM6DS3_ACC_GYRO_INT_ACTIVE_HIGH, LSM6DS3_ACC_GYRO_INT_LATCH);
	if (err < 0) {

		GSE_ERR("LSM6DS3_Int_Ctrl err!\n");
		goto EXIT_FAIL;
	}

	irq = gpio_to_irq(hw->eint_gpio);
	if (irq < 0){
		GSE_ERR("Unable to request gpio irq. int pin = %d, irq = %d\n", hw->eint_gpio, irq);
		goto EXIT_FAIL;
	}
	if (request_irq(irq, lsm6ds3_interrupt, IRQF_TRIGGER_RISING, "lsm6ds3_irq1", (void *)client)){
		GSE_ERR("Could not allocate APDS9950_INT !\n");
		goto EXIT_FAIL;
	}
	irq_set_irq_wake(irq, 1);

	if(hw->is_eint2_connected != 0){
		irq2 = gpio_to_irq(hw->eint2_gpio);
		if (irq2 < 0){
			GSE_ERR("Unable to request gpio irq2. int pin = %d, irq2 = %d, err=%d\n", hw->eint_gpio, irq2, err);
			goto EXIT_FAIL;
		}
		if (request_irq(irq2, lsm6ds3_interrupt, IRQF_TRIGGER_RISING, "lsm6ds3_irq2", (void *)client)){
			GSE_ERR("Could not allocate APDS9950_INT !\n");
			goto EXIT_FAIL;
		}
		irq_set_irq_wake(irq2, 1);
	}

	lsm6ds3_enable_irq(0);

	return LSM6DS3_SUCCESS;
EXIT_FAIL:
	gpio_free(hw->eint_gpio);
	return -1;
}

static void lsm6ds3_eint_work(struct work_struct *work)
{
	u8 databuf[2] = {0};
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	int err;
	//struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();

	if (obj == NULL) {

		GSE_ERR("obj_i2c_data is null pointer!!\n");
		goto lsm6ds3_eint_work_exit;
	}

	if (hwmsen_read_byte(obj->client, LSM6DS3_FUNC_SRC, databuf)) {

		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		goto lsm6ds3_eint_work_exit;
	}

	if (atomic_read(&obj->trace) & ACCEL_TRC_DATA) {
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
#ifdef LSM6DS3_STEP_COUNTER
	if (LSM6DS3_SIGNICANT_MOTION_INT_STATUS & databuf[0]) {
		/*add the action when receive the significant motion*/
		step_notify(TYPE_SIGNIFICANT);
	}
	if(LSM6DS3_STEP_OVERFLOW_INT_STATUS & databuf[0]){
		GSE_ERR("the step_counter is overflow\n");
		if(detector_enable_status){
			lsm6ds3_report_detector();
		}
		err = LSM6DS3_Reset_Pedo_Data(obj->client,LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);
		if(err != LSM6DS3_SUCCESS)
		{
			goto lsm6ds3_eint_work_exit;
		}
		step_base_num += 65536;
		first_data_after_reset = true;
	}
	if ((LSM6DS3_STEP_DETECT_INT_STATUS|LSM6DS3_STEP_DELTA_INT_STATUS) & databuf[0]) {
		lsm6ds3_report_detector();
	}
#endif

#ifdef LSM6DS3_TILT_FUNC
	if (LSM6DS3_TILT_INT_STATUS & databuf[0]) {

		/*add the action when receive the tilt interrupt*/
		tilt_notify();
	}
#endif

lsm6ds3_eint_work_exit:
	return;
}

#if defined(LSM6DS3_TILT_FUNC)
static struct tilt_init_info  lsm6ds3_tilt_init_info;
static int LSM6DS3_Enable_Tilt_Func_On_Int(struct i2c_client *client, LSM6DS3_ACC_GYRO_ROUNT_INT_t tilt_int, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0;
	u8 op_reg = 0;
	GSE_FUN();

	if (LSM6DS3_ACC_GYRO_INT1 == tilt_int) {

		op_reg = LSM6DS3_MD1_CFG;
	} else if (LSM6DS3_ACC_GYRO_INT2 == tilt_int) {

		op_reg = LSM6DS3_MD2_CFG;
	}

	if (hwmsen_read_byte(client, op_reg, databuf)) {

		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if (enable) {

		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_TILT_MASK;	/*clear */
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_TILT_ENABLED;
	} else {

		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_TILT_MASK;	/*clear */
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_TILT_DISABLED;
	}

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {

		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	res = LSM6DS3_Int_Ctrl(client, LSM6DS3_ACC_GYRO_INT_ACTIVE_HIGH, LSM6DS3_ACC_GYRO_INT_LATCH);
	if (res < 0) {

		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}
static int LSM6DS3_Enable_Tilt_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0;
	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DS3_TAP_CFG, databuf)) {

		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if (enable) {

		databuf[0] &= ~LSM6DS3_TILT_EN_MASK;	/*clear */
		databuf[0] |= LSM6DS3_ACC_GYRO_TILT_EN_ENABLED;
	} else {

		databuf[0] &= ~LSM6DS3_TILT_EN_MASK;	/*clear*/
		databuf[0] |= LSM6DS3_ACC_GYRO_TILT_EN_DISABLED;
	}

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_TAP_CFG;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {

		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}
static int LSM6DS3_enable_tilt(struct i2c_client *client, bool enable)
{
	int res = 0;
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();

	if (enable) {

		/*/set ODR to 26 hz
		//res = LSM6DS3_acc_SetSampleRate(client, LSM6DS3_ACC_ODR_26HZ);*/
		res = LSM6DS3_acc_SetSampleRate(client, obj->sample_rate);
		if (LSM6DS3_SUCCESS == res) {

			GSE_LOG(" %s set 26hz odr to acc\n", __func__);
		}

		res = LSM6DS3_Enable_Tilt_Func(client, enable);
		if (res != LSM6DS3_SUCCESS) {

			GSE_LOG(" LSM6DS3_Enable_Tilt_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}

		res = LSM6DS3_acc_Enable_Func(client, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);
		if (res != LSM6DS3_SUCCESS) {

			GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}

		res = LSM6DS3_Enable_Tilt_Func_On_Int(client, LSM6DS3_ACC_GYRO_INT1, true);  /*default route to INT1 	*/
		if (res != LSM6DS3_SUCCESS) {

			GSE_LOG(" LSM6DS3_Enable_Tilt_Func_On_Int failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		lsm6ds3_enable_irq(1);
	} else {

		res = LSM6DS3_Enable_Tilt_Func(client, enable);
		if (res != LSM6DS3_SUCCESS) {

			GSE_LOG(" LSM6DS3_Enable_Tilt_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
		if (!enable_status && !pedo_enable_status && !pedo_enable_sig_status) {

			res = LSM6DS3_acc_SetPowerMode(client, false);
			if (res != LSM6DS3_SUCCESS) {

				GSE_LOG(" LSM6DS3_acc_SetPowerMode failed!\n");
				return LSM6DS3_ERR_STATUS;
			}
		}

		lsm6ds3_enable_irq(0);
	}

	return LSM6DS3_SUCCESS;
}

static int lsm6ds3_tilt_open_report_data(int open)
{
	int res = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	if (1 == open) {

		tilt_enable_status = true;
		res = LSM6DS3_enable_tilt(priv->client, true);
		if (LSM6DS3_SUCCESS != res) {

			GSE_ERR("%s run LSM6DS3_enable_tilt to true failed!\n", __func__);
		}
	} else if (0 == open) {

		tilt_enable_status = false;
		res = LSM6DS3_enable_tilt(priv->client, false);
		if (LSM6DS3_SUCCESS != res) {

			GSE_ERR("%s run LSM6DS3_enable_tilt to false failed!\n", __func__);
		}
	}

	return res;
}
static int lsm6ds3_tilt_get_data(u16 *value, int *status)
{
	return 0;
}

static int lsm6ds3_tilt_local_init(void)
{
	int res = 0;

	struct tilt_control_path tilt_ctl = {0};
	struct tilt_data_path tilt_data = {0};

	mutex_lock(&lsm6ds3_init_mutex);
	set_bit(LSM6DS3_TILT, &lsm6ds3_init_flag_test);

	if ((0 == test_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test)) \
		&& (0 == test_bit(LSM6DS3_STEP_C, &lsm6ds3_init_flag_test))) {

		res = lsm6ds3_local_init_common();
		if (res < 0) {

			goto lsm6ds3_tilt_local_init_failed;
		}
	}

	if (lsm6ds3_acc_init_flag == -1) {

		mutex_unlock(&lsm6ds3_init_mutex);
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	} else {

		res = lsm6ds3_setup_eint();
		tilt_ctl.open_report_data = lsm6ds3_tilt_open_report_data;
		res = tilt_register_control_path(&tilt_ctl);

		tilt_data.get_data = lsm6ds3_tilt_get_data;
		res = tilt_register_data_path(&tilt_data);
	}
	mutex_unlock(&lsm6ds3_init_mutex);
	return 0;

lsm6ds3_tilt_local_init_failed:
	mutex_unlock(&lsm6ds3_init_mutex);
	GSE_ERR("%s init failed!\n", __FUNCTION__);
	return -1;
}
static int lsm6ds3_tilt_local_uninit(void)
{
	clear_bit(LSM6DS3_TILT, &lsm6ds3_init_flag_test);
    return 0;
}

static struct tilt_init_info  lsm6ds3_tilt_init_info = {
	.name   = "LSM6DS3_TILT",
	.init   = lsm6ds3_tilt_local_init,
	.uninit = lsm6ds3_tilt_local_uninit,
};
#endif

#ifdef LSM6DS3_STEP_COUNTER
static struct step_c_init_info  lsm6ds3_step_c_init_info;
static int LSM6DS3_acc_SetFullScale(struct i2c_client *client, u8 acc_fs);
static ssize_t show_debounce_step_num(struct device_driver *ddri, char *buf)
{
	int num = -1;
	u8 databuf[2] = {0};
	int res = 0;
	struct i2c_client *client = lsm6ds3_i2c_client;
	GSE_FUN();
	// enable embedded register

	res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_ENABLED);
	if(res < 0)
	{
		GSE_ERR("%s LSM6DS3_W_Open_RAM_Page err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	// read embedded register
	if(hwmsen_read_byte(client, LSM6DS3_STEP_COUNTER_DEBOUNCE, databuf))
	{
		GSE_ERR("%s read LSM6DS3_STEP_COUNTER_DEBOUNCE register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}

	databuf[0] &= 0x07;
	num = (int)databuf[0];

	// disable embedded register
	res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_DISABLED);
	if(res < 0)
	{
		GSE_ERR("%s LSM6DS3_W_Open_RAM_Page err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return sprintf(buf, "%d\n",num);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_debounce_step_num(struct device_driver *ddri, const char *buf, size_t count)
{
	int num = 0;

	sscanf(buf, "%d", &num);
	GSE_LOG("%s try to set debounce_step_num:%d\n", __func__, num);

	if((num >= 0) && (num <= 7))
	{
		if (lsm6ds3_set_reg_bit(LSM6DS3_STEP_COUNTER_DEBOUNCE,0x07,(u8)num,true)) {
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
	u8 databuf[2] = {0};
	u8 debounce = 0;
	int res = 0;
	u8 threshold = 0;
	struct i2c_client *client = lsm6ds3_i2c_client;

	GSE_FUN();
	// enable embedded register
	res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_ENABLED);
	if(res < 0)
	{
		GSE_ERR("%s LSM6DS3_W_Open_RAM_Page err:%d!\n", __func__,res);
		return LSM6DS3_ERR_I2C;
	}
	// read step counter threshold value
	if(hwmsen_read_byte(client, LSM6DS3_STEP_COUNTER_MINTHS, databuf))
	{
		GSE_ERR("%s read LSM6DS3_STEP_COUNTER_MINTHS register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read LSM6DS3_STEP_COUNTER_MINTHS register: 0x%x\n", __func__, databuf[0]);
	}

	threshold = databuf[0];

	// read embedded register
	if(hwmsen_read_byte(client, LSM6DS3_STEP_COUNTER_DEBOUNCE, databuf))
	{
		GSE_ERR("%s read LSM6DS3_STEP_COUNTER_DEBOUNCE register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	debounce = databuf[0];

	// disable embedded register
	res = LSM6DS3_W_Open_RAM_Page(client, LSM6DS3_ACC_GYRO_RAM_PAGE_DISABLED);
	if(res < 0)
	{
		GSE_ERR("%s LSM6DS3_W_Open_RAM_Page err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return sprintf(buf, "threshold: 0x%02x debounce: 0x%02x\n",threshold,debounce);
}
/*----------------------------------------------------------------------------*/
static int LSM6DS3_Write_PedoParameters(struct i2c_client *client, u8 pedo4g, u8 threshold, u8 debounce);

static ssize_t store_threshold_debounce(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned int debounce = 0;
	unsigned int threshold = 0;
	struct i2c_client *client = lsm6ds3_i2c_client;
	int res = 0;
	GSE_FUN();

	sscanf(buf, "%x %x", &threshold, &debounce);
	GSE_LOG("%s: try to set threshold:0x%02x debounce:0x%02x\n", __func__, (u8)threshold, (u8)debounce);

	res = LSM6DS3_Write_PedoParameters(client, 1, threshold, debounce);// set threshold to a certain value here
	if(res != LSM6DS3_SUCCESS)
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
static struct driver_attribute *lsm6ds3_step_c_attr_list[] = {
	&driver_attr_debounce_step_num,     /*debounce step num*/
	&driver_attr_threshold_debounce,     /*debounce step num*/
};
/*----------------------------------------------------------------------------*/
static int lsm6ds3_step_c_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(lsm6ds3_step_c_attr_list)/sizeof(lsm6ds3_step_c_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver,  lsm6ds3_step_c_attr_list[idx])))
		{
			GSE_ERR("driver_create_file (%s) = %d\n",  lsm6ds3_step_c_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_step_c_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof( lsm6ds3_step_c_attr_list)/sizeof( lsm6ds3_step_c_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver,  lsm6ds3_step_c_attr_list[idx]);
	}
	return err;
}

static int LSM6DS3_W_Open_RAM_Page(struct i2c_client *client, LSM6DS3_ACC_GYRO_RAM_PAGE_t newValue)
{
	u8 databuf[2] = {0};
	int res = 0;
	GSE_FUN();
	//before access the ram register,must power down the sensor
	if((newValue==LSM6DS3_ACC_GYRO_RAM_PAGE_ENABLED)&&(sensor_power==true)){
		LSM6DS3_acc_Enable_Func(client, LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED);
		GSE_ERR("disable pedo before RAM_ACCESS\n");
	}
	msleep(10);
	if (hwmsen_read_byte(client, LSM6DS3_RAM_ACCESS, databuf)) {
		GSE_ERR("%s read LSM6DS3_RAM_ACCESS register err!\n", __func__);
		goto EXIT_FAIL;
	} else {
		GSE_LOG("read LSM6DS3_RAM_ACCESS register: 0x%02x\n", databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_RAM_PAGE_MASK;/*clear */
	databuf[0] |= newValue;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_RAM_ACCESS;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("%s write LSM6DS3_RAM_ACCESS register err!\n", __func__);
		goto EXIT_FAIL;
	}
	if((newValue==LSM6DS3_ACC_GYRO_RAM_PAGE_DISABLED)&&(sensor_power==true)){
		LSM6DS3_acc_Enable_Func(client, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);
		GSE_ERR("enable pedo after RAM_ACCESS\n");
	}
	return LSM6DS3_SUCCESS;
EXIT_FAIL:
	if(sensor_power==true){
		LSM6DS3_acc_Enable_Func(client, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);
		GSE_ERR("enable pedo after RAM_ACCESS ERR\n");
	}
	return LSM6DS3_ERR_I2C;
}

static int LSM6DS3_Init_Pedometer(struct i2c_client *client)
{
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();

	if(hw->is_eint2_connected == 0){
		GSE_ERR("eint2 is not connected\n");
		//route INT2 to INT1
		if (lsm6ds3_set_reg_bit(LSM6DS3_CTRL4_C,0x20,0x20,false)) {
			goto INIT_FAIL;
		}
	}

	//DELTA time = 0xbb * 1.6384s
	if (lsm6ds3_set_reg_bit(LSM6DS3_STEP_COUNT_DELTA,0xff,0xbb,true)) {
		goto INIT_FAIL;
	}

	GSE_ERR("success\n");
	return LSM6DS3_SUCCESS;
INIT_FAIL:
	GSE_ERR("fail\n");
	return LSM6DS3_ERR_STATUS;
}

static int LSM6DS3_Enable_Pedometer_Int_ByTime(bool enable)
{
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	GSE_FUN();

	if(hw->is_eint_supported == 0){
		GSE_ERR("eint is not support\n");
		return 0;
	}

	if(enable){
		if(lsm6ds3_set_reg_bit(LSM6DS3_WAKE_UP_DUR,0x10,0x00,false)){
			goto EXIT_FAIL;
		}
		if(lsm6ds3_set_reg_bit(LSM6DS3_TAP_CFG,0x80,0x80,false)){
			goto EXIT_FAIL;
		}
		if(lsm6ds3_set_reg_bit(LSM6DS3_INT2_CTRL,0x80,0x80,false)){
			goto EXIT_FAIL;
		}
	}else{
		if(lsm6ds3_set_reg_bit(LSM6DS3_INT2_CTRL,0x80,0x00,false)){
			goto EXIT_FAIL;
		}
	}

	return LSM6DS3_SUCCESS;
EXIT_FAIL:
	return LSM6DS3_ERR_STATUS;
}

static int LSM6DS3_acc_Enable_Pedometer_Func(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0;
	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DS3_TAP_CFG, databuf)) {

		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if (enable) {

		databuf[0] &= ~LSM6DS3_PEDO_EN_MASK;	/*clear */
		databuf[0] |= LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED;
	} else {

		databuf[0] &= ~LSM6DS3_PEDO_EN_MASK;	/*clear */
		databuf[0] |= LSM6DS3_ACC_GYRO_PEDO_EN_DISABLED;
	}

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_TAP_CFG;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {

		GSE_ERR("write enable pedometer func register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

static int lsm6ds3_Enable_Pedometer_Func_On_Int(struct i2c_client *client, bool enable)
{
	int res = 0;
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	GSE_FUN();

	if(hw->is_eint_supported == 0){
		GSE_ERR("eint is not support\n");
		return 0;
	}

	if (enable) {
		if(lsm6ds3_set_reg_bit(LSM6DS3_INT1_CTRL,LSM6DS3_ACC_GYRO_INT_PEDO_MASK,LSM6DS3_ACC_GYRO_INT_PEDO_ENABLED,false)){
			return LSM6DS3_ERR_STATUS;
		}
	} else {
		if(lsm6ds3_set_reg_bit(LSM6DS3_INT1_CTRL,LSM6DS3_ACC_GYRO_INT_PEDO_MASK,LSM6DS3_ACC_GYRO_INT_PEDO_DISABLED,false)){
			return LSM6DS3_ERR_STATUS;
		}
	}

	res = LSM6DS3_Int_Ctrl(client, LSM6DS3_ACC_GYRO_INT_ACTIVE_HIGH, LSM6DS3_ACC_GYRO_INT_LATCH);
	if (res < 0) {
		GSE_ERR("write enable pedo func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_Write_PedoParameters(struct i2c_client *client, u8 pedo4g, u8 threshold, u8 debounce)
{
	u8 value;
	u8 mask;
	GSE_FUN();
	mask = LSM6DS3_STEP_CNT_4G_MASK | LSM6DS3_STEP_CNT_THRESHOLD_MASK;
	if(pedo4g){
		value = LSM6DS3_STEP_CNT_4G_MASK | (threshold & LSM6DS3_STEP_CNT_THRESHOLD_MASK);
	}else{
		value = (threshold & LSM6DS3_STEP_CNT_THRESHOLD_MASK);
	}
	if(lsm6ds3_set_reg_bit(LSM6DS3_STEP_COUNTER_MINTHS,mask,value,true)){
		goto SET_FAIL;
	}

	if(lsm6ds3_set_reg_bit(LSM6DS3_STEP_COUNTER_DEBOUNCE,0xFF,(debounce & LSM6DS3_STEP_CNT_DEBOUNCE_MASK),true)){
		goto SET_FAIL;
	}
	GSE_ERR("success\n");
	return LSM6DS3_SUCCESS;
SET_FAIL:
	GSE_ERR("fail\n");
	return LSM6DS3_ERR_STATUS;
}
static int LSM6DS3_Reset_Pedo_Data(struct i2c_client *client, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_t newValue)
{

	if(lsm6ds3_set_reg_bit(LSM6DS3_CTRL10_C,LSM6DS3_PEDO_RST_STEP_MASK,newValue,false)){
		goto RESET_FAIL;
	}
	if(lsm6ds3_set_reg_bit(LSM6DS3_CTRL10_C,LSM6DS3_PEDO_RST_STEP_MASK,0x00,false)){
		goto RESET_FAIL;
	}
	msleep(20);
	GSE_LOG(" LSM6DS3_Reset_Pedo_Data success!\n");
	return LSM6DS3_SUCCESS;
RESET_FAIL:
	GSE_LOG(" LSM6DS3_Reset_Pedo_Data failed!\n");
	return LSM6DS3_ERR_STATUS;
}

static int LSM6DS3_enable_pedo(struct i2c_client *client, bool enable)
{

	int res = 0;
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);

	if (true == enable) {

		/*/software reset
		//set ODR to 26 hz
		//res = LSM6DS3_acc_SetSampleRate(client, LSM6DS3_ACC_ODR_26HZ);*/
		res = LSM6DS3_acc_SetSampleRate(client, obj->sample_rate);
		if (LSM6DS3_SUCCESS == res) {

			GSE_LOG(" %s set 26hz odr to acc\n", __func__);
		}
		/*enable tilt feature and pedometer feature*/
		res = LSM6DS3_acc_Enable_Pedometer_Func(client, enable);
		if (res != LSM6DS3_SUCCESS) {

			GSE_LOG(" LSM6DS3_acc_Enable_Pedometer_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}

		res = LSM6DS3_acc_Enable_Func(client, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);
		if (res != LSM6DS3_SUCCESS) {

			GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
	} else {

		res = LSM6DS3_acc_Enable_Pedometer_Func(client, enable);
		if (res != LSM6DS3_SUCCESS) {

			GSE_LOG(" LSM6DS3_acc_Enable_Func failed at disable pedo!\n");
			return LSM6DS3_ERR_STATUS;
		}
		/*do not turn off the func*/
		if (!enable_status && !tilt_enable_status) {

			res = LSM6DS3_acc_SetPowerMode(client, false);
			if (res != LSM6DS3_SUCCESS) {

				GSE_LOG(" LSM6DS3_acc_SetPowerMode failed at disable pedo!\n");
				return LSM6DS3_ERR_STATUS;
			}
		}
	}
	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_Get_Pedo_DataReg(struct i2c_client *client, u16 *Value)
{
	u8 databuf[2] = {0};
	//GSE_FUN();

	if (hwmsen_read_block(client, LSM6DS3_STEP_COUNTER_L, databuf, 2)) {

		GSE_ERR("LSM6DS3 read pedo data  error\n");
		return -2;
	}

	*Value = (databuf[1]<<8)|databuf[0];

	return LSM6DS3_SUCCESS;
}

static int lsm6ds3_step_c_open_report_data(int open)
{
	return LSM6DS3_SUCCESS;
}

static int lsm6ds3_step_c_enable_nodata(int en)
{
	int res = 0;
	int value = en;
	int err = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	if (priv == NULL) {

		GSE_ERR("%s obj_i2c_data is NULL!\n", __func__);
		return -1;
	}

	if (value == 1) {
		pedo_enable_count++;
		GSE_ERR("enable %d\n",pedo_enable_count);
		if(!pedo_enable_status){
			res = LSM6DS3_enable_pedo(priv->client, true);
			if (LSM6DS3_SUCCESS != res) {
				GSE_LOG("LSM6DS3_enable_pedo failed at open action!\n");
				return res;
			}
			//set overflow int
			lsm6ds3_set_reg_bit(LSM6DS3_INT2_CTRL,0x40,0x40,false);
			lsm6ds3_enable_irq(1);
			pedo_enable_status = true;
		}else{
			GSE_LOG("lsm6ds3_step_c already enable!\n");
		}
	} else {
		pedo_enable_count--;
		GSE_ERR("disable %d\n",pedo_enable_count);
		if(pedo_enable_count<=0){
			if(pedo_enable_status){
				if(detector_enable_status==0){
					lsm6ds3_set_reg_bit(LSM6DS3_INT2_CTRL,0x40,0x00,false);
					lsm6ds3_enable_irq(0);
					pedo_enable_status = false;
					res = LSM6DS3_enable_pedo(priv->client, false);
					if (LSM6DS3_SUCCESS != res) {
						GSE_LOG("LSM6DS3_enable_pedo failed at close action!\n");
						return res;
					}
				}
			}else{
				GSE_LOG("lsm6ds3_step_c already disable!\n");
			}
			pedo_enable_count=0;
		}

	}

	//GSE_LOG("lsm6ds3_step_c_enable_nodata %d OK!\n",value);
    return err;
}
static int lsm6ds3_step_c_enable_step_detect(int en)
{
	//struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	int res = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	GSE_ERR("%s %d enter\n", __func__,en);
	if (priv == NULL) {

		GSE_ERR("%s obj_i2c_data is NULL!\n", __func__);
		return -1;
	}

	detector_enable_status = en;
	res = lsm6ds3_step_c_enable_nodata(en);
	if (LSM6DS3_SUCCESS != res) {
		GSE_LOG("%s failed at open action!\n",__func__);
		return res;
	}
	if(en){
		detector_step=-1;
		res = lsm6ds3_Enable_Pedometer_Func_On_Int(priv->client, true);
		if (LSM6DS3_SUCCESS != res) {
			GSE_LOG("%s failed at open action!\n",__func__);
			return res;
		}
		lsm6ds3_report_detector();
	}else{
		res = lsm6ds3_Enable_Pedometer_Func_On_Int(priv->client, false);
		if (LSM6DS3_SUCCESS != res) {
			GSE_LOG("%s failed at open action!\n",__func__);
			return res;
		}
	}

	GSE_LOG("%s %d OK!\n",__func__,en);
	return 0;
}

static int lsm6ds3_step_c_set_delay(u64 delay)
{

	return 0;
}

static int lsm6ds3_step_c_get_data(u64 *value, int *status)
{
	int err = 0;
	u16 pedo_data = 0;
	u16 pedo_data_1 = 0;
	u16 pedo_data_2 = 0;
	u64 total_step;
	int diff = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	
	//GSE_FUN();

	if(!pedo_enable_status){
		GSE_LOG(" LSM6DS3 step_counter is not enabled!\n");
		lsm6ds3_step_c_enable_nodata(1);
		return LSM6DS3_ERR_STATUS;
	}

	if(atomic_read(&priv->suspend)){
		GSE_LOG(" LSM6DS3 step_counter is suspend!\n");
		return LSM6DS3_ERR_STATUS;
	}

	err = LSM6DS3_Get_Pedo_DataReg(priv->client, &pedo_data_1);
	if(err != LSM6DS3_SUCCESS)
	{
		GSE_LOG(" LSM6DS3_Get_Pedo_DataReg 1st failed!\n");
		return LSM6DS3_ERR_STATUS;
	}
	msleep(50);
	err = LSM6DS3_Get_Pedo_DataReg(priv->client, &pedo_data_2);
	if(err != LSM6DS3_SUCCESS)
	{
		GSE_LOG(" LSM6DS3_Get_Pedo_DataReg 2nd failed!\n");
		return LSM6DS3_ERR_STATUS;
	}
	
	diff = pedo_data_2 - pedo_data_1;
	if((diff <0) || (diff >10)){
		GSE_LOG(" LSM6DS3_Get_Pedo_DataReg data error!\n");
		return LSM6DS3_ERR_STATUS;
	}
	
	pedo_data = pedo_data_2;

	//wait for the data be reset
	if(first_data_after_reset){
		if(pedo_data >= 1000){
			GSE_LOG("step counter first_data_after_reset:%d[0x%04x]\n",pedo_data,pedo_data);
			pedo_data = 0;
		}else{
			first_data_after_reset = false;
		}
	}
#if 0
	//to avoid the error large data
	diff = (int)((u64)pedo_data+ step_base_num - pre_step);
	diff = diff<0?diff*(-1):diff;

	if(diff>20000){
		msleep(20);
		GSE_LOG("the data may be error:diff[%d] pedo[%d] base[%lld] pre[%lld]\n",diff,pedo_data,step_base_num,pre_step);
		err = LSM6DS3_Get_Pedo_DataReg(priv->client, &pedo_data);
		if(err != LSM6DS3_SUCCESS)
		{
			return LSM6DS3_ERR_STATUS;
		}
		diff = (int)((u64)pedo_data+ step_base_num - pre_step);
		diff = diff<0?diff*(-1):diff;
		if(diff>20000){
			msleep(20);
			GSE_LOG("the data may be error again:diff[%d] pedo[%d] base[%lld] pre[%lld]\n",diff,pedo_data,step_base_num,pre_step);
			err = LSM6DS3_Reset_Pedo_Data(priv->client,LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);
			if(err != LSM6DS3_SUCCESS)
			{
				return LSM6DS3_ERR_STATUS;
			}
			step_base_num = pre_step;
			first_data_after_reset = true;
			return LSM6DS3_ERR_STATUS;
		}
	}
#endif

	total_step = (u64)pedo_data + step_base_num;

	*value = total_step;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	step_poll_count++;

	if(total_step != pre_step){
		GSE_LOG("step changed! [%d]%lld = %lld + %d,pre: %lld\n",step_poll_count,total_step,step_base_num,pedo_data,pre_step);
		pre_step = total_step;
	}

	if(step_poll_count>=50){
		GSE_LOG("step poll %d times:%lld = %lld + %d,pre: %lld\n",step_poll_count,total_step,step_base_num,pedo_data,pre_step);
		step_poll_count=0;
	}

	if(hw->is_eint_supported == 0){
		if(pedo_data >= 60000){
			err = LSM6DS3_Reset_Pedo_Data(priv->client,LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);
			if(err != LSM6DS3_SUCCESS)
			{
				return LSM6DS3_ERR_STATUS;
			}
			step_base_num += pedo_data;
			first_data_after_reset = true;
		}
	}

	return err;
}
static int lsm6ds3_step_c_get_data_step_d(u64 *value, int *status)
{
	return 0;
}
static int lsm6ds3_step_c_get_data_significant(u64 *value, int *status)
{
	return 0;
}

static void report_single_detector_event(struct work_struct *work)
{
	int i=0;
	for(i=0;i<3;i++){
		if(aim_detector_step-detector_step>0){
			step_notify(TYPE_STEP_DETECTOR);
			detector_step++;
			//GSE_ERR("%d\n",detector_step);
			mdelay(1);
		}
	}

	if(aim_detector_step-detector_step>0){
		schedule_delayed_work(&report_single_detector_worker,msecs_to_jiffies(10));
	}
}

static int lsm6ds3_report_detector(void){
	int err;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	u16 tmpstep;
	int diff=0;
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();

	if(hw->is_eint_supported == 0){
		GSE_ERR("eint is not support\n");
		return 0;
	}
	//to wait the wakeup from suspend
	//msleep(50);

	err = LSM6DS3_Get_Pedo_DataReg(priv->client, &tmpstep);
	if(err != LSM6DS3_SUCCESS)
	{
		GSE_LOG(" LSM6DS3_Get_Pedo_DataReg failed!\n");
		return LSM6DS3_ERR_STATUS;
	}
	aim_detector_step = tmpstep;
	if((aim_detector_step-detector_step<0)||(detector_step<0)){
		detector_step=aim_detector_step;
	}
	diff = aim_detector_step-detector_step;
	GSE_ERR("pre %d new %d diff %d\n", detector_step,aim_detector_step,diff);
	if(diff>0){
		//step_notify(TYPE_STEP_DETECTOR);
		//detector_step++;
		schedule_delayed_work(&report_single_detector_worker,0);
		//GSE_ERR("report %d\n", i);
	}
	//detector_step=tmpstep;

	return 0;
}

static int LSM6DS3_Enable_SigMotion_Func(struct i2c_client *client, LSM6DS3_ACC_GYRO_SIGN_MOT_t newValue)
{
	u8 databuf[2] = {0};
	int res = 0;
	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DS3_CTRL10_C, databuf)) {

		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] &= ~LSM6DS3_ACC_GYRO_SIGN_MOT_MASK;/*clear */
	databuf[0] |= newValue;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL10_C;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_Enable_SigMotion_Func_On_Int(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0;
	u8 op_reg = 0;

	LSM6DS3_ACC_GYRO_FUNC_EN_t func_enable;
	LSM6DS3_ACC_GYRO_SIGN_MOT_t sigm_enable;
	GSE_FUN();

	if (enable) {

		func_enable = LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED;
		sigm_enable = LSM6DS3_ACC_GYRO_SIGN_MOT_ENABLED;

		res = LSM6DS3_acc_Enable_Func(client, func_enable);
		if (res != LSM6DS3_SUCCESS) {

			GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
			return LSM6DS3_ERR_STATUS;
		}
	} else {

		/*func_enable = LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED;*/
		sigm_enable = LSM6DS3_ACC_GYRO_SIGN_MOT_DISABLED;
	}

	res = LSM6DS3_Enable_SigMotion_Func(client, sigm_enable);
	if (res != LSM6DS3_SUCCESS) {

		GSE_LOG(" LSM6DS3_acc_Enable_Func failed!\n");
		return LSM6DS3_ERR_STATUS;
	}

	/*Config interrupt for significant motion*/

	op_reg = LSM6DS3_INT1_CTRL;

	if (hwmsen_read_byte(client, op_reg, databuf)) {

		GSE_ERR("%s read data format register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);
	}

	if (enable) {

		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_SIGN_MOT_MASK;		/*clear */
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_SIGN_MOT_ENABLED;
	} else {

		databuf[0] &= ~LSM6DS3_ACC_GYRO_INT_SIGN_MOT_MASK;		/*clear */
		databuf[0] |= LSM6DS3_ACC_GYRO_INT_SIGN_MOT_DISABLED;
	}

	databuf[1] = databuf[0];
	databuf[0] = op_reg;
	res = i2c_master_send(client, databuf, 0x2);
	if (res < 0) {

		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	res = LSM6DS3_Int_Ctrl(client, LSM6DS3_ACC_GYRO_INT_ACTIVE_HIGH, LSM6DS3_ACC_GYRO_INT_LATCH);
	if (res < 0) {

		GSE_ERR("write enable tilt func register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_Set_SigMotion_Threshold(struct i2c_client *client, u8 SigMotion_Threshold)
{
	u8 databuf[2] = {0};
	int res = 0;
	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DS3_FUNC_CFG_ACCESS, databuf)) {

		GSE_ERR("%s read LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	}
	databuf[0] = 0x80;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_FUNC_CFG_ACCESS;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	databuf[1] = SigMotion_Threshold;
	databuf[0] = LSM6DS3_SM_THS;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}

	databuf[1] = 0x00;
	databuf[0] = LSM6DS3_FUNC_CFG_ACCESS;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GSE_ERR("%s write LSM6DS3_CTRL10_C register err!\n", __func__);
		return LSM6DS3_ERR_I2C;
	}
	return LSM6DS3_SUCCESS;
}

static int lsm6ds3_step_c_enable_significant(int en)
{
	int res = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	//struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();

	if (1 == en) {

		pedo_enable_sig_status = true;
		res = LSM6DS3_Set_SigMotion_Threshold(priv->client, 0x08);
		if (LSM6DS3_SUCCESS != res) {

			GSE_ERR("%s run LSM6DS3_Set_SigMotion_Threshold to fail!\n", __func__);
		}
		/*res = LSM6DS3_acc_SetSampleRate(priv->client, LSM6DS3_ACC_ODR_26HZ);*/
		res = LSM6DS3_acc_SetSampleRate(priv->client, priv->sample_rate);
		if (LSM6DS3_SUCCESS != res) {

			GSE_ERR("%s run LSM6DS3_Set_SigMotion_Threshold to fail!\n", __func__);
		}
		res = LSM6DS3_Enable_SigMotion_Func_On_Int(priv->client, true); /*default route to INT2*/
		if (LSM6DS3_SUCCESS != res) {

			GSE_ERR("%s run LSM6DS3_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
		}

		res = LSM6DS3_acc_SetFullScale(priv->client, LSM6DS3_ACC_RANGE_4g);
		if (LSM6DS3_SUCCESS != res) {

			GSE_ERR("%s run LSM6DS3_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
		}

		lsm6ds3_enable_irq(1);

	} else if (0 == en) {

		pedo_enable_sig_status = false;
		res = LSM6DS3_Enable_SigMotion_Func_On_Int(priv->client, false);
		if (LSM6DS3_SUCCESS != res) {

			GSE_ERR("%s run LSM6DS3_Enable_SigMotion_Func_On_Int to fail!\n", __func__);
		}
		if (!enable_status && !tilt_enable_status && !pedo_enable_status) {

			res = LSM6DS3_acc_SetPowerMode(priv->client, false);
			if (LSM6DS3_SUCCESS != res) {

				GSE_ERR("%s run LSM6DS3_acc_SetPowerMode to fail!\n", __func__);
			}
		}
		lsm6ds3_enable_irq(0);
	}

	return res;
}

static int lsm6ds3_step_c_local_init(void)
{
	int res = 0;

	struct step_c_control_path step_ctl = {0};
	struct step_c_data_path step_data = {0};

	mutex_lock(&lsm6ds3_init_mutex);

	set_bit(LSM6DS3_STEP_C, &lsm6ds3_init_flag_test);

	if ((0 == test_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test)) \
		&& (0 == test_bit(LSM6DS3_TILT, &lsm6ds3_init_flag_test))) {

		res = lsm6ds3_local_init_common();
		if (res < 0) {

			goto lsm6ds3_step_c_local_init_failed;
		}

	}

	if (lsm6ds3_acc_init_flag == -1) {

		mutex_unlock(&lsm6ds3_init_mutex);
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	} else {
		res = lsm6ds3_step_c_create_attr(&(lsm6ds3_step_c_init_info.platform_diver_addr->driver));
		if(res)
		{
			 GSE_ERR("lsm6ds3_step_c_create_attr err\n");
			goto lsm6ds3_step_c_creat_attr_failed;
		}

		step_ctl.open_report_data = lsm6ds3_step_c_open_report_data;
		step_ctl.enable_nodata = lsm6ds3_step_c_enable_nodata;
		step_ctl.enable_step_detect  = lsm6ds3_step_c_enable_step_detect;
		step_ctl.set_delay = lsm6ds3_step_c_set_delay;
		step_ctl.is_report_input_direct = false;
		step_ctl.is_support_batch = false;
		step_ctl.enable_significant = lsm6ds3_step_c_enable_significant;

		res = step_c_register_control_path(&step_ctl);
		if (res) {

			 GSE_ERR("register step counter control path err\n");
			goto lsm6ds3_step_c_local_init_failed;
		}

		step_data.get_data = lsm6ds3_step_c_get_data;
		step_data.get_data_step_d = lsm6ds3_step_c_get_data_step_d;
		step_data.get_data_significant = lsm6ds3_step_c_get_data_significant;

		step_data.vender_div = 1;
		res = step_c_register_data_path(&step_data);
		if (res) {

			GSE_ERR("register step counter data path err= %d\n", res);
			goto lsm6ds3_step_c_local_init_failed;
		}

	}
	mutex_unlock(&lsm6ds3_init_mutex);
	return 0;

lsm6ds3_step_c_local_init_failed:
	lsm6ds3_step_c_delete_attr(&(lsm6ds3_step_c_init_info.platform_diver_addr->driver));
lsm6ds3_step_c_creat_attr_failed:
	mutex_unlock(&lsm6ds3_init_mutex);
	GSE_ERR("%s init failed!\n", __FUNCTION__);
	return res;

}
static int lsm6ds3_step_c_local_uninit(void)
{
	clear_bit(LSM6DS3_STEP_C, &lsm6ds3_init_flag_test);
    return 0;
}

static struct step_c_init_info  lsm6ds3_step_c_init_info = {
	.name   = "LSM6DS3_STEP_C",
	.init   = lsm6ds3_step_c_local_init,
	.uninit = lsm6ds3_step_c_local_uninit,
};
#endif
#endif //LSM6DS3_EMBEDED_FUNC

static int LSM6DS3_acc_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};
	int res = 0;
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);

	if (enable == sensor_power) {

		GSE_LOG("Sensor power status is newest!\n");
		return LSM6DS3_SUCCESS;
	}

	if (hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf)) {

		GSE_ERR("read lsm6ds3 power ctl register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	GSE_LOG("LSM6DS3_CTRL1_XL:databuf[0] =  %x!\n", databuf[0]);


	if (true == enable) {

		databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;/*clear lsm6ds3 gyro ODR bits*/
		databuf[0] |= obj->sample_rate;/*LSM6DS3_ACC_ODR_104HZ; //default set 100HZ for LSM6DS3 acc*/
	} else {

		databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;/*clear lsm6ds3 acc ODR bits*/
		databuf[0] |= LSM6DS3_ACC_ODR_POWER_DOWN;
	}
	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL1_XL;
	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GSE_LOG("LSM6DS3 set power mode: ODR 100hz failed!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("set LSM6DS3 gyro power mode:ODR 100HZ ok %d!\n", enable);
	}

	sensor_power = enable;

	return LSM6DS3_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_acc_SetFullScale(struct i2c_client *client, u8 acc_fs)
{
	u8 databuf[2] = {0};
	int res = 0;
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);

	GSE_FUN();

	if (hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf)) {

		GSE_ERR("read LSM6DS3_CTRL1_XL err!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  LSM6DS3_CTRL1_XL register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_RANGE_MASK;	/*clear */
	databuf[0] |= acc_fs;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL1_XL;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GSE_ERR("write full scale register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	switch (acc_fs) {

	case LSM6DS3_ACC_RANGE_2g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_2G;
			break;
	case LSM6DS3_ACC_RANGE_4g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_4G;
			break;
	case LSM6DS3_ACC_RANGE_8g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_8G;
			break;
	case LSM6DS3_ACC_RANGE_16g:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_16G;
			break;
	default:
			obj->sensitivity = LSM6DS3_ACC_SENSITIVITY_2G;
			break;
	}

	if (hwmsen_read_byte(client, LSM6DS3_CTRL9_XL, databuf)) {

		GSE_ERR("read LSM6DS3_CTRL9_XL err!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  LSM6DS3_CTRL9_XL register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_ENABLE_AXIS_MASK;	/*clear */
	databuf[0] |= LSM6DS3_ACC_ENABLE_AXIS_X | LSM6DS3_ACC_ENABLE_AXIS_Y | LSM6DS3_ACC_ENABLE_AXIS_Z;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL9_XL;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {

		GSE_ERR("write full scale register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/* set the acc sample rate*/
static int LSM6DS3_acc_SetSampleRate(struct i2c_client *client, u8 sample_rate)
{
	u8 databuf[2] = {0};
	int res = 0;
	GSE_FUN();

	res = LSM6DS3_acc_SetPowerMode(client, true);	/*set Sample Rate will enable power and should changed power status*/
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	if (hwmsen_read_byte(client, LSM6DS3_CTRL1_XL, databuf)) {

		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	} else {
		GSE_LOG("read  LSM6DS3_CTRL1_XL register: 0x%x\n", databuf[0]);
	}

	databuf[0] &= ~LSM6DS3_ACC_ODR_MASK;	/*clear*/
	databuf[0] |= sample_rate;

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL1_XL;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("write sample rate register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_ReadAccRawData(struct i2c_client *client, s16 data[LSM6DS3_ACC_AXES_NUM])
{
	int err = 0;
	char databuf[6] = {0};

	if (NULL == client) {

		err = -EINVAL;
	} else {

		if (hwmsen_read_block(client, LSM6DS3_OUTX_L_XL, databuf, 6)) {

			GSE_ERR("LSM6DS3 read acc data  error\n");
			return -2;
		} else {
			data[LSM6DS3_AXIS_X] = (s16)((databuf[LSM6DS3_AXIS_X*2+1] << 8) | (databuf[LSM6DS3_AXIS_X*2]));
			data[LSM6DS3_AXIS_Y] = (s16)((databuf[LSM6DS3_AXIS_Y*2+1] << 8) | (databuf[LSM6DS3_AXIS_Y*2]));
			data[LSM6DS3_AXIS_Z] = (s16)((databuf[LSM6DS3_AXIS_Z*2+1] << 8) | (databuf[LSM6DS3_AXIS_Z*2]));
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int LSM6DS3_ReadAccData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lsm6ds3_i2c_data *obj = (struct lsm6ds3_i2c_data *)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LSM6DS3_ACC_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if (NULL == buf) {

		return -1;
	}
	if (NULL == client) {

		*buf = 0;
		return -2;
	}

	if (sensor_power == false) {

		res = LSM6DS3_acc_SetPowerMode(client, true);
		if (res) {

			GSE_ERR("Power on lsm6ds3 error %d!\n", res);
		}
		msleep(20);
	}

	res = LSM6DS3_ReadAccRawData(client, obj->data);
	if (res < 0) {

		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	} else {
		obj->data[LSM6DS3_AXIS_X] = (long)(obj->data[LSM6DS3_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		obj->data[LSM6DS3_AXIS_Y] = (long)(obj->data[LSM6DS3_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		obj->data[LSM6DS3_AXIS_Z] = (long)(obj->data[LSM6DS3_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);

		obj->data[LSM6DS3_AXIS_X] += obj->cali_sw[LSM6DS3_AXIS_X];
		obj->data[LSM6DS3_AXIS_Y] += obj->cali_sw[LSM6DS3_AXIS_Y];
		obj->data[LSM6DS3_AXIS_Z] += obj->cali_sw[LSM6DS3_AXIS_Z];

		/*remap coordinate*/
		acc[obj->cvt.map[LSM6DS3_AXIS_X]] = obj->cvt.sign[LSM6DS3_AXIS_X]*obj->data[LSM6DS3_AXIS_X];
		acc[obj->cvt.map[LSM6DS3_AXIS_Y]] = obj->cvt.sign[LSM6DS3_AXIS_Y]*obj->data[LSM6DS3_AXIS_Y];
		acc[obj->cvt.map[LSM6DS3_AXIS_Z]] = obj->cvt.sign[LSM6DS3_AXIS_Z]*obj->data[LSM6DS3_AXIS_Z];

		/*//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);

		//Out put the mg

		acc[LSM6DS3_AXIS_X] = acc[LSM6DS3_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LSM6DS3_AXIS_Y] = acc[LSM6DS3_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LSM6DS3_AXIS_Z] = acc[LSM6DS3_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		*/

		sprintf(buf, "%04x %04x %04x", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);

		if (atomic_read(&obj->trace) & ADX_TRC_IOCTL) { /*atomic_read(&obj->trace) & ADX_TRC_IOCTL*/

			/*GSE_LOG("gsensor data: %s!\n", buf);*/
			GSE_LOG("raw data:obj->data:%04x %04x %04x\n", obj->data[LSM6DS3_AXIS_X], obj->data[LSM6DS3_AXIS_Y], obj->data[LSM6DS3_AXIS_Z]);
			GSE_LOG("acc:%04x %04x %04x\n", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);

			/*LSM6DS3_dumpReg(client);*/
		}
	}

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

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	char strbuf[LSM6DS3_BUFSIZE];
	if (NULL == client) {

		GSE_ERR("i2c client is null!!\n");
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
	int x, y, z;

	if (NULL == client) {

		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	LSM6DS3_ReadAccData(client, strbuf, LSM6DS3_BUFSIZE);
	sscanf(strbuf, "%x %x %x", &x, &y, &z);
	return snprintf(buf, PAGE_SIZE, "%d, %d, %d\n", x, y, z);
}
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lsm6ds3_i2c_client;
	s16 data[LSM6DS3_ACC_AXES_NUM] = {0};

	if (NULL == client) {

		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	LSM6DS3_ReadAccRawData(client, data);
	return snprintf(buf, PAGE_SIZE, "%x,%x,%x\n", data[0], data[1], data[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {

		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL) {

		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}

	if (1 == sscanf(buf, "0x%x", &trace)) {

		atomic_set(&obj->trace, trace);
	} else {

		GSE_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}

	return count;
}
static ssize_t show_chipinit_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	if (obj == NULL) {

		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {

		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}

	LSM6DS3_init_client(obj->client, true);
	LSM6DS3_dumpReg(obj->client);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct lsm6ds3_i2c_data *obj = obj_i2c_data;
	if (obj == NULL) {

		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw) {

		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n",
			obj->hw->i2c_num, obj->hw->direction, obj->sensitivity, obj->hw->power_id, obj->hw->power_vol);
	LSM6DS3_dumpReg(obj->client);
	} else {

		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;
}
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct lsm6ds3_i2c_data *data = obj_i2c_data;
	if (NULL == data) {

		printk(KERN_ERR "lsm6ds3_i2c_data is null!!\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction, atomic_read(&data->layout), data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int layout = 0;
	struct lsm6ds3_i2c_data *data = obj_i2c_data;

	if (NULL == data) {

		printk(KERN_ERR "lsm6ds3_i2c_data is null!!\n");
		return count;
	}



	if (1 == sscanf(buf, "%d", &layout)) {

		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt)) {

			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(data->hw->direction, &data->cvt)) {

			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		} else {

			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
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
	struct lsm6ds3_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GSE_ERR("i2c_data obj is null!!\n");
		return -1;
	}

	if (hwmsen_read_byte(data->client, LSM6DS3_CTRL1_XL, databuf)) {

		GSE_ERR("read acc data format register err!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  LSM6DS3_CTRL1_XL register: 0x%x\n", databuf[0]);
	}

	odr = databuf[0]&LSM6DS3_ACC_ODR_MASK;
	switch(odr){
		case LSM6DS3_ACC_ODR_13HZ:
			res = sprintf(buf, "0:13HZ\n1:104HZ\n2:208HZ\n3:1660HZ\n4:other\ncurrent:LSM6DS3_ACC_ODR_13HZ\n");
			break;
		case LSM6DS3_ACC_ODR_104HZ:
			res = sprintf(buf, "0:13HZ\n1:104HZ\n2:208HZ\n3:1660HZ\n4:other\ncurrent:LSM6DS3_ACC_ODR_104HZ\n");
			break;
		case LSM6DS3_ACC_ODR_208HZ:
			res = sprintf(buf, "0:13HZ\n1:104HZ\n2:208HZ\n3:1660HZ\n4:other\ncurrent:LSM6DS3_ACC_ODR_208HZ\n");
			break;
		case LSM6DS3_ACC_ODR_1660HZ:
			res = sprintf(buf, "0:13HZ\n1:104HZ\n2:208HZ\n3:1660HZ\n4:other\ncurrent:LSM6DS3_ACC_ODR_1660HZ\n");
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
	u8 odr = LSM6DS3_ACC_ODR_104HZ;
	struct lsm6ds3_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}

	if (1 == sscanf(buf, "%d", &odr_sel)) {
		switch(odr_sel){
			case 0:
				odr = LSM6DS3_ACC_ODR_13HZ;
				break;
			case 1:
				odr = LSM6DS3_ACC_ODR_104HZ;
				break;
			case 2:
				odr = LSM6DS3_ACC_ODR_208HZ;
				break;
			case 3:
				odr = LSM6DS3_ACC_ODR_1660HZ;
				break;
			default:
				odr = LSM6DS3_ACC_ODR_104HZ;
				break;
		}

		LSM6DS3_acc_SetSampleRate(data->client, odr);
	} else {

		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}

static u8 dump_reg = 0;
static ssize_t show_reg(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	u8 databuf[2];
	struct lsm6ds3_i2c_data *data = obj_i2c_data;

	if (NULL == data) {
		GSE_ERR("i2c_data obj is null!!\n");
		return -1;
	}

	if (hwmsen_read_byte(data->client, dump_reg, databuf)) {
		GSE_ERR("read acc data format register %02x err!\n",dump_reg);
		res = sprintf(buf, "-1");
	} else {
		res = sprintf(buf, "reg 0x%02x = 0x%02x\n",dump_reg,databuf[0]);
	}
	return res;
}
/*----------------------------------------------------------------------------*/

static ssize_t store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6ds3_i2c_data *data = obj_i2c_data;
	int addr = 0;
	int value = 0;
	u8 databuf[2] = {0};
	int num = 0;
	int res = 0;

	if (NULL == data) {
		GSE_ERR("i2c_data obj is null!!\n");
		return count;
	}
	num = sscanf(buf, "%x %x", &addr,&value);

	switch(num){
		case 1:
			dump_reg = addr;
			break;
		case 2:
			dump_reg = (u8)addr;
			databuf[1] = (u8)value;
			databuf[0] = (u8)dump_reg;

			res = i2c_master_send(data->client, databuf, 0x2);
			if (res <= 0) {
				GSE_ERR("write register 0x%02x err!\n",databuf[0]);
			}
			break;
		default:
			GSE_ERR("invalid input num!\n");
			break;
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
static DRIVER_ATTR(odr,      S_IRUGO | S_IWUSR, show_odr, store_odr);
static DRIVER_ATTR(reg,      S_IRUGO | S_IWUSR, show_reg, store_reg);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *LSM6DS3_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_sensorrawdata,   /*dump sensor raw data*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_chipinit,
	&driver_attr_layout,
	&driver_attr_odr,
	&driver_attr_reg,
};
/*----------------------------------------------------------------------------*/
static int lsm6ds3_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(LSM6DS3_attr_list)/sizeof(LSM6DS3_attr_list[0]));
	if (driver == NULL) {

		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++)	{
		err = driver_create_file(driver, LSM6DS3_attr_list[idx]);
		if (0 != err) {

			GSE_ERR("driver_create_file (%s) = %d\n", LSM6DS3_attr_list[idx]->attr.name, err);
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

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver,  LSM6DS3_attr_list[idx]);
	}
	return err;
}
static int LSM6DS3_Set_RegInc(struct i2c_client *client, bool inc)
{
	u8 databuf[2] = {0};
	int res = 0;
	/*GSE_FUN();     */

	if (hwmsen_read_byte(client, LSM6DS3_CTRL3_C, databuf)) {

		GSE_ERR("read LSM6DS3_CTRL3_XL err!\n");
		return LSM6DS3_ERR_I2C;
	} else {

		GSE_LOG("read  LSM6DS3_CTRL3_C register: 0x%x\n", databuf[0]);
	}
	if (inc) {

		databuf[0] |= LSM6DS3_CTRL3_C_IFINC;

		databuf[1] = databuf[0];
		databuf[0] = LSM6DS3_CTRL3_C;

		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {

			GSE_ERR("write full scale register err!\n");
			return LSM6DS3_ERR_I2C;
		}
	}

	return LSM6DS3_SUCCESS;
}

static int LSM6DS3_Set_Performance(struct i2c_client *client, int en)
{
	u8 databuf[2] = {0};
	int res = 0;
	/*GSE_FUN();     */

	if (hwmsen_read_byte(client, LSM6DS3_CTRL6_C, databuf)) {

		GSE_ERR("read LSM6DS3_CTRL6_C err!\n");
		return LSM6DS3_ERR_I2C;
	} else {
		GSE_LOG("read  LSM6DS3_CTRL6_C register: 0x%x\n", databuf[0]);
	}
	if (en) {
		databuf[0] &= ~LSM6DS3_ACC_HIPERF_MASK;
	}else{
		databuf[0] |= LSM6DS3_ACC_HIPERF_MASK;
	}

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL6_C;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("write LSM6DS3_CTRL6_C register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	if (hwmsen_read_byte(client, LSM6DS3_CTRL7_G, databuf)) {

		GSE_ERR("read LSM6DS3_CTRL6_C err!\n");
		return LSM6DS3_ERR_I2C;
	} else {
		GSE_LOG("read  LSM6DS3_CTRL6_C register: 0x%x\n", databuf[0]);
	}
	if (en) {
		databuf[0] &= ~LSM6DS3_GYRO_HIPERF_MASK;
	}else{
		databuf[0] |= LSM6DS3_GYRO_HIPERF_MASK;
	}

	databuf[1] = databuf[0];
	databuf[0] = LSM6DS3_CTRL7_G;

	res = i2c_master_send(client, databuf, 0x2);
	if (res <= 0) {
		GSE_ERR("write LSM6DS3_CTRL7_G register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int LSM6DS3_init_client(struct i2c_client *client, bool enable)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	GSE_FUN();
    GSE_LOG(" lsm6ds3 addr %x!\n", client->addr);
	res = LSM6DS3_CheckDeviceID(client);
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	res = LSM6DS3_Set_RegInc(client, true);
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	res = LSM6DS3_acc_SetFullScale(client, LSM6DS3_ACC_RANGE_4g);/*we have only this choice*/
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	/*res = LSM6DS3_acc_SetSampleRate(client, LSM6DS3_ACC_ODR_104HZ);*/
	res = LSM6DS3_acc_SetSampleRate(client, obj->sample_rate);
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	res = LSM6DS3_acc_SetPowerMode(client, enable);
	if (res != LSM6DS3_SUCCESS) {

		return res;
	}

	res = LSM6DS3_Set_Performance(client, 1);
	if (res != LSM6DS3_SUCCESS) {
		return res;
	}

	GSE_LOG("LSM6DS3_init_client OK!\n");
	/*acc setting*/

#ifdef CONFIG_LSM6DS3_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_open_report_data(int open)
{
    /*should queuq work to report event if  is_report_input_direct=true*/

    return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL*/

static int lsm6ds3_enable_nodata(int en)
{
	int value = en;
	int err = 0;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	if (priv == NULL) {

		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	if (value == 1) {

		enable_status = true;
	} else {

		enable_status = false;
		priv->sample_rate = LSM6DS3_ACC_ODR_52HZ; /*default rate*/
	}
	GSE_LOG("enable value=%d, sensor_power =%d\n", value, sensor_power);

	if (((value == 0) && (sensor_power == false)) || ((value == 1) && (sensor_power == true))) {

		GSE_LOG("Gsensor device have updated!\n");
	}
#if (LSM6DS3_EMBEDED_FUNC)
	else if (!pedo_enable_status && !tilt_enable_status  && !pedo_enable_sig_status) {

		err = LSM6DS3_acc_SetPowerMode(priv->client, enable_status);
	}
#endif
    GSE_LOG("%s OK!\n", __FUNCTION__);
    return err;
}

static int lsm6ds3_set_delay(u64 ns)
{
    int value = 0;
	int err = 0;
	int sample_delay;
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;

	if (priv == NULL) {

		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	value = (int)ns/1000/1000;

	if (value <= 5) {

		sample_delay = LSM6DS3_ACC_ODR_208HZ;
	} else if (value <= 10) {

		sample_delay = LSM6DS3_ACC_ODR_104HZ;
	} else {

		sample_delay = LSM6DS3_ACC_ODR_52HZ;
	}
	priv->sample_rate = sample_delay;
	err = LSM6DS3_acc_SetSampleRate(priv->client, sample_delay);
	if (err != LSM6DS3_SUCCESS) {

		GSE_ERR("Set delay parameter error!\n");
	}

	if (value >= 50) {

		atomic_set(&priv->filter, 0);
	} else {
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[LSM6DS3_AXIS_X] = 0;
		priv->fir.sum[LSM6DS3_AXIS_Y] = 0;
		priv->fir.sum[LSM6DS3_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	}

    GSE_LOG("%s (%d), chip only use 1024HZ \n", __FUNCTION__, value);
    return 0;
}

static int lsm6ds3_get_data(int *x, int *y, int *z, int *status)
{
    char buff[LSM6DS3_BUFSIZE];
	struct lsm6ds3_i2c_data *priv = obj_i2c_data;
	
	//GSE_FUN();

	if (priv == NULL) {

		GSE_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	if (atomic_read(&priv->trace) & ACCEL_TRC_DATA) {

		GSE_LOG("%s (%d),  \n", __FUNCTION__, __LINE__);
	}
	memset(buff, 0, sizeof(buff));
	LSM6DS3_ReadAccData(priv->client, buff, LSM6DS3_BUFSIZE);

	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return 0;
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int lsm6ds3_open(struct inode *inode, struct file *file)
{
	file->private_data = lsm6ds3_i2c_client;

	if (file->private_data == NULL) {

		GSE_ERR("null pointer!!\n");
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
static long lsm6ds3_acc_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct lsm6ds3_i2c_data *obj = (struct lsm6ds3_i2c_data *)i2c_get_clientdata(client);
	char strbuf[LSM6DS3_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];

	/*GSE_FUN(f);*/
	if (_IOC_DIR(cmd) & _IOC_READ) {

		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {

		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (err) {

		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {

	case GSENSOR_IOCTL_INIT:
			break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}

			LSM6DS3_ReadChipInfo(client, strbuf, LSM6DS3_BUFSIZE);

			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {

				err = -EFAULT;
				break;
			}
			break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}

			LSM6DS3_ReadAccData(client, strbuf, LSM6DS3_BUFSIZE);

			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {

				err = -EFAULT;
				break;
			}
			break;

	case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}

			break;

	case GSENSOR_IOCTL_READ_OFFSET:
			data = (void __user *) arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}

			break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}

			LSM6DS3_ReadAccRawData(client, (s16 *)strbuf);
			if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {

				err = -EFAULT;
				break;
			}
			break;

	case GSENSOR_IOCTL_SET_CALI:
			data = (void __user *)arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}
			if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {

				err = -EFAULT;
				break;
			}
			if (atomic_read(&obj->suspend)) {

				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			} else {

		#if 0
			cali[LSM6DS3_AXIS_X] = (s64)(sensor_data.x) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
			cali[LSM6DS3_AXIS_Y] = (s64)(sensor_data.y) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
			cali[LSM6DS3_AXIS_Z] = (s64)(sensor_data.z) * 1000*1000/(obj->sensitivity*GRAVITY_EARTH_1000);
		#else
			cali[LSM6DS3_AXIS_X] = (s64)(sensor_data.x);
			cali[LSM6DS3_AXIS_Y] = (s64)(sensor_data.y);
			cali[LSM6DS3_AXIS_Z] = (s64)(sensor_data.z);
		#endif
				err = LSM6DS3_acc_WriteCalibration(client, cali);
			}
			break;

	case GSENSOR_IOCTL_CLR_CALI:
			err = LSM6DS3_acc_ResetCalibration(client);
			break;

	case GSENSOR_IOCTL_GET_CALI:
			data = (void __user *)arg;
			if (data == NULL) {

				err = -EINVAL;
				break;
			}
			err = LSM6DS3_acc_ReadCalibration(client, cali);
			if (err < 0) {

				break;
			}

		#if 0
			sensor_data.x = (s64)(cali[LSM6DS3_AXIS_X]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
			sensor_data.y = (s64)(cali[LSM6DS3_AXIS_Y]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
			sensor_data.z = (s64)(cali[LSM6DS3_AXIS_Z]) * obj->sensitivity*GRAVITY_EARTH_1000/(1000*1000);
		#else
			sensor_data.x = (s64)(cali[LSM6DS3_AXIS_X]);
			sensor_data.y = (s64)(cali[LSM6DS3_AXIS_Y]);
			sensor_data.z = (s64)(cali[LSM6DS3_AXIS_Z]);
		#endif
			if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {

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
static long lsm6ds3_acc_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {

	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
			if (arg32 == NULL) {

				err = -EINVAL;
				break;
			}

		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
			if (err) {
				GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
				return err;
		    }
			break;

	case COMPAT_GSENSOR_IOCTL_SET_CALI:
			if (arg32 == NULL) {

				err = -EINVAL;
				break;
			}

			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
			if (err) {
				GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
				return err;
			}
			break;

	case COMPAT_GSENSOR_IOCTL_GET_CALI:
			if (arg32 == NULL) {

				err = -EINVAL;
				break;
			}

		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
			if (err) {
				GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
				return err;
			}
			break;

	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
			if (arg32 == NULL) {

				err = -EINVAL;
				break;
			}

			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
			if (err) {
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
static struct file_operations lsm6ds3_acc_fops = {
	.owner = THIS_MODULE,
	.open = lsm6ds3_open,
	.release = lsm6ds3_release,
	.unlocked_ioctl = lsm6ds3_acc_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = lsm6ds3_acc_compat_ioctl,
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice lsm6ds3_acc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &lsm6ds3_acc_fops,
};

/*----------------------------------------------------------------------------*/
#ifndef LSM6DS3_USE_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	GSE_FUN();

	if (obj == NULL) {

		GSE_ERR("null pointer!!\n");
		return -1;
	}
	atomic_set(&obj->suspend, 1);

	if(detector_enable_status){
		lsm6ds3_Enable_Pedometer_Func_On_Int(obj->client, false);
		LSM6DS3_Enable_Pedometer_Int_ByTime(true);
	}

	if (pedo_enable_status  || tilt_enable_status || pedo_enable_sig_status) {
		LSM6DS3_acc_SetSampleRate(obj->client, LSM6DS3_ACC_ODR_26HZ);
		return 0;
	}
	err = LSM6DS3_acc_SetPowerMode(obj->client, false);
	if (err) {
		GSE_ERR("write power control fail!!\n");
		return -1;
	}

	sensor_power = false;

	LSM6DS3_power(obj->hw, 0);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_resume(struct i2c_client *client)
{
	struct lsm6ds3_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	//struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return -1;
	}

	atomic_set(&obj->suspend, 0);
	if(detector_enable_status){
		LSM6DS3_Enable_Pedometer_Int_ByTime(false);
		//lsm6ds3_report_detector();
		lsm6ds3_Enable_Pedometer_Func_On_Int(obj->client, true);
	}

	if (pedo_enable_status  || tilt_enable_status || pedo_enable_sig_status) {
		LSM6DS3_acc_SetSampleRate(obj->client, obj->sample_rate);
		return 0;
	}

	LSM6DS3_power(obj->hw, 1);
	err = LSM6DS3_acc_SetPowerMode(obj->client, enable_status);

	if (err) {
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return -1;
	}
    return 0;
}
/*----------------------------------------------------------------------------*/
#else /*LSM6DS3_USE_EARLYSUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void lsm6ds3_early_suspend(struct early_suspend *h)
{
	struct lsm6ds3_i2c_data *obj = container_of(h, struct lsm6ds3_i2c_data, early_drv);
	int err;
	//struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	GSE_FUN();

	if (obj == NULL) {

		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1);

	if(detector_enable_status){
		lsm6ds3_Enable_Pedometer_Func_On_Int(obj->client, false);
		LSM6DS3_Enable_Pedometer_Int_ByTime(true);
	}

	if (pedo_enable_status  || tilt_enable_status || pedo_enable_sig_status) {
		LSM6DS3_acc_SetSampleRate(obj->client, LSM6DS3_ACC_ODR_26HZ);
		return;
	}
	err = LSM6DS3_acc_SetPowerMode(obj->client, false);
	if (err) {
		GSE_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;

	LSM6DS3_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void lsm6ds3_late_resume(struct early_suspend *h)
{
	struct lsm6ds3_i2c_data *obj = container_of(h, struct lsm6ds3_i2c_data, early_drv);
	int err;
	//struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->suspend, 0);
	if(detector_enable_status){
		LSM6DS3_Enable_Pedometer_Int_ByTime(false);
		//lsm6ds3_report_detector();
		lsm6ds3_Enable_Pedometer_Func_On_Int(obj->client, true);
	}

	if (pedo_enable_status  || tilt_enable_status || pedo_enable_sig_status) {
		LSM6DS3_acc_SetSampleRate(obj->client, obj->sample_rate);
		return;
	}

	LSM6DS3_power(obj->hw, 1);

	err = LSM6DS3_acc_SetPowerMode(obj->client, enable_status);

	if (err) {
		GSE_ERR("initialize client fail! err code %d!\n", err);
		return;
	}
}
#endif /*LSM6DS3_USE_EARLYSUSPEND*/

/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct lsm6ds3_i2c_data *obj;

	int err = 0;

	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct lsm6ds3_i2c_data));

#ifdef LSM6DS3_EMBEDED_FUNC
	INIT_WORK(&obj->eint_work, lsm6ds3_eint_work);
#endif
	obj->hw = lsm6ds3_get_cust_acc_hw();
	obj->sample_rate = LSM6DS3_ACC_ODR_52HZ;

	atomic_set(&obj->layout, obj->hw->direction);
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {

		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit_kfree;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	lsm6ds3_i2c_client = new_client;
	err = LSM6DS3_init_client(new_client, false);
	if (err) {

		goto exit_init_failed;
	}

#if (LSM6DS3_STEP_COUNTER)
	err = LSM6DS3_Init_Pedometer(lsm6ds3_i2c_client);
	if(err)
	{
		GSE_LOG("LSM6DS3_Init_Pedometer failed!\n");
	}
	err = LSM6DS3_Write_PedoParameters(lsm6ds3_i2c_client, 1, 0x0d, 0x4f);// set threshold to a certain value here
	if(err)
	{
		GSE_LOG(" LIS2DS12_Write_PedoParameters failed!\n");
	}
	//try to reset pedo data to 0 after reboot
	LSM6DS3_Reset_Pedo_Data(lsm6ds3_i2c_client,LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED);
#endif

	err = misc_register(&lsm6ds3_acc_device);
	if (err) {

		GSE_ERR("lsm6ds3_acc_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}

#ifdef LSM6DS3_USE_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = lsm6ds3_early_suspend,
	obj->early_drv.resume   = lsm6ds3_late_resume,
	register_early_suspend(&obj->early_drv);
#endif
#ifdef LSM6DS3_EMBEDED_FUNC
	err = lsm6ds3_setup_eint();
	if(err){
		GSE_ERR("lsm6ds3 setup interrupt failed!\n");
		goto exit_misc_device_register_failed;
	}
#endif
	lsm6ds3_acc_init_flag = 0;
	register_device_proc("Sensor_accel", "lsm6ds3", "ST");
	GSE_LOG("%s: OK\n", __func__);
	return 0;

exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	lsm6ds3_acc_init_flag = -1;
	GSE_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	if (test_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test)) {

		err = lsm6ds3_delete_attr(&(lsm6ds3_init_info.platform_diver_addr->driver));
	}
	lsm6ds3_acc_init_flag = -1;

	err = misc_deregister(&lsm6ds3_acc_device);
	if (err) {

		GSE_ERR("misc_deregister lsm6ds3_gyro_device fail: %d\n", err);
	}

	lsm6ds3_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_local_init_common(void)
{
	struct acc_hw *accel_hw = lsm6ds3_get_cust_acc_hw();
	GSE_FUN();

	LSM6DS3_power(accel_hw, 1);

	if (i2c_add_driver(&lsm6ds3_i2c_driver)) {

		GSE_ERR("add driver error\n");
		return -1;
	}

	return 0;
}

static int lsm6ds3_local_init(void)
{
	int res = 0;
	struct acc_control_path ctl = {0};
    struct acc_data_path data = {0};
	struct lsm6ds3_i2c_data *obj = NULL;
	
	GSE_FUN();
	mutex_lock(&lsm6ds3_init_mutex);

	set_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test);

	if ((0 == test_bit(LSM6DS3_STEP_C, &lsm6ds3_init_flag_test)) \
		&& (0 == test_bit(LSM6DS3_TILT, &lsm6ds3_init_flag_test)))
	{
		res = lsm6ds3_local_init_common();
		if (res < 0) {

			goto lsm6ds3_local_init_failed;
		}
	}

	if (lsm6ds3_acc_init_flag == -1) {

		mutex_unlock(&lsm6ds3_init_mutex);
		GSE_ERR("%s init failed!\n", __FUNCTION__);
		return -1;
	} else {

		obj = obj_i2c_data;
		if (NULL == obj) {

			GSE_ERR("i2c_data obj is null!!\n");
			goto lsm6ds3_local_init_failed;
		}

		res = lsm6ds3_create_attr(&(lsm6ds3_init_info.platform_diver_addr->driver));
		if (res < 0) {

			goto lsm6ds3_local_init_failed;
		}
		ctl.open_report_data = lsm6ds3_open_report_data;
	    ctl.enable_nodata = lsm6ds3_enable_nodata;
	    ctl.set_delay  = lsm6ds3_set_delay;
	    ctl.is_report_input_direct = false;
	    ctl.is_support_batch = obj->hw->is_batch_supported;

	    res = acc_register_control_path(&ctl);
	    if (res) {

			GSE_ERR("register acc control path err\n");
			goto lsm6ds3_local_init_failed;

	    }

	    data.get_data = lsm6ds3_get_data;
	    data.vender_div = 1000;
	    res = acc_register_data_path(&data);
	    if (res) {
			GSE_ERR("register acc data path err= %d\n", res);
			goto lsm6ds3_local_init_failed;

	    }
	}
	mutex_unlock(&lsm6ds3_init_mutex);
	return 0;
lsm6ds3_local_init_failed:
	GSE_ERR("%s init failed\n", __FUNCTION__);
	mutex_unlock(&lsm6ds3_init_mutex);
	return res;

}
static int lsm6ds3_local_uninit(void)
{
	struct acc_hw *accel_hw = lsm6ds3_get_cust_acc_hw();
	clear_bit(LSM6DS3_ACC, &lsm6ds3_init_flag_test);

    /*GSE_FUN();    */
    LSM6DS3_power(accel_hw, 0);
    i2c_del_driver(&lsm6ds3_i2c_driver);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init lsm6ds3_init(void)
{
	struct acc_hw *hw = lsm6ds3_get_cust_acc_hw();
	GSE_FUN();
    //GSE_LOG("%s: i2c_number=%d\n", __func__, hw->i2c_num);
    i2c_register_board_info(hw->i2c_num, &i2c_lsm6ds3, 1);

	acc_driver_add(&lsm6ds3_init_info);

#ifdef LSM6DS3_STEP_COUNTER /*step counter*/
	step_c_driver_add(&lsm6ds3_step_c_init_info); /*step counter*/
#endif
#ifdef LSM6DS3_TILT_FUNC
	tilt_driver_add(&lsm6ds3_tilt_init_info);
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit lsm6ds3_exit(void)
{
	GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(lsm6ds3_init);
module_exit(lsm6ds3_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LSM6DS3 Accelerometer");
MODULE_AUTHOR("xj.wang@mediatek.com, darren.han@st.com");






/*----------------------------------------------------------------- LSM6DS3 ------------------------------------------------------------------*/
