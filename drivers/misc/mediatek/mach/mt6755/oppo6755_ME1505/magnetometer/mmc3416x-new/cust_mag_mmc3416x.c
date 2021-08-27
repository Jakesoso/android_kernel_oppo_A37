/*************************************************************
 ** Copyright (C), 2008-2013, OPPO Mobile Comm Corp., Ltd
 ** VENDOR_EDIT
 ** File : cust_mag.c
 ** Description : MAG Driver Custom Para
 ** Date : 2013-12-05 15:56
 ** Author : Prd.SenDrv
 **
 ** ------------------ Revision History: ---------------------
 ** <author> <date> <desc>
 ** Prd.SenDrv 2013/12/05 NULL
 *************************************************************/ 
#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>

static struct mag_hw cust_mag_hw = {
    .i2c_num = 4,//i2c-0
    .direction = 0,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
};

struct mag_hw* mmc3416x_get_cust_mag_hw(void) 
{
    return &cust_mag_hw;
}


