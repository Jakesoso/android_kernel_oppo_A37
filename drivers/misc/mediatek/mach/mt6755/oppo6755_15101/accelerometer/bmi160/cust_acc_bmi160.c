/*************************************************************
 ** Copyright (C), 2008-2013, OPPO Mobile Comm Corp., Ltd
 ** VENDOR_EDIT
 ** File : cust_acc_lis3dh.c
 ** Description : ACC Driver Custom Para
 ** Date : 2013-12-05 15:56
 ** Author : Prd.SenDrv
 **
 ** ------------------ Revision History: ---------------------
 ** <author> <date> <desc>
 ** Prd.SenDrv 2013/12/05 NULL
 *************************************************************/
#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>

#ifdef VENDOR_EDIT//Shaoyu.Huang@Prd.BasicDrv.Sensor,add 2012/5/23 for gsensor power
#include <linux/delay.h>
#include <mach/mt_gpio.h>
static int power(struct acc_hw *hw, unsigned int on, char *devname)
{
	static unsigned int status = 0;
	if (on == status){
		return 0;
	}
	if (on){
		//hwPowerOn(MT6325_POWER_LDO_VGP3, VOL_1800, devname);
		//hwPowerOn(MT6325_POWER_LDO_VIO28, VOL_1800, devname);
		msleep(1);
	}
	else{
		//hwPowerDown(MT6325_POWER_LDO_VIO28,  devname);
		//hwPowerDown(MT6325_POWER_LDO_VGP3,  devname);
		msleep(1);
	}

	status = on;
	return 0;
}
#endif/*VENDOR_EDIT*/
/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = 4,
    .direction = 5,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
#ifdef VENDOR_EDIT//Shaoyu.Huang@Prd.BasicDrv.Sensor, add 2012/5/23 for gsensor power
    .power = power,
#endif/*VENDOR_EDIT*/
#ifdef VENDOR_EDIT//zhihong.lu@BSP.sensor
    .is_eint_supported = 0,
    .eint_gpio = 64,//int2 = gpio64
#endif
};
/*---------------------------------------------------------------------------*/
struct acc_hw* bmi160_get_cust_acc_hw(void)
{
    return &cust_acc_hw;
}
