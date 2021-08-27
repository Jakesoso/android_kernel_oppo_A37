#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#include <cust_alsps.h>

static struct ps_adjust_para cust_ps_adjust_para_apds9921 = {
	.ps_up = 30,//60,
	.ps_thd_low_notrend = 60,
	.ps_thd_high_notrend = 80,
	.ps_thd_low_trend =20,// 40,
	.ps_thd_high_trend =30,// 60,
	.ps_thd_low_highlight =800,// 600,
	.ps_thd_high_highlight = 850,//650,
	.ps_adjust_min = 0,
	.ps_adjust_max = 850,
	.highlight_limit = 8000,
	.sampling_time = 60,
	.sampling_count = 5,
	.dirty_adjust_limit = 900,
	.dirty_adjust_low_thd = 140,//150,
	.dirty_adjust_high_thd =180,// 300,
};
#ifdef VENDOR_EDIT
//zhihong.lu@Prd.BSP.sensor,2016/4/15,add for apds9922
static struct ps_adjust_para cust_ps_adjust_para_apds9922 = {
	.ps_up = 30,//60,
	.ps_thd_low_notrend = 50,
	.ps_thd_high_notrend = 60,
	.ps_thd_low_trend =20,// 40,
	.ps_thd_high_trend =30,// 60,
	.ps_thd_low_highlight =800,// 600,
	.ps_thd_high_highlight = 850,//650,
	.ps_adjust_min = 0,
	.ps_adjust_max = 850,
	.highlight_limit = 8000,
	.sampling_time = 60,
	.sampling_count = 5,
	.dirty_adjust_limit = 900,
	.dirty_adjust_low_thd = 140,//150,
	.dirty_adjust_high_thd =180,// 300,
};
#endif /*VENDOR_EDIT*/
static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 1,
	.eint_gpio = 6,
	.als_gain1 = 2,
	.als_gain2 = 3,
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    //.als_level  = { 0,  1,  1,   7,  15,  15,  100, 1000, 2000,  3000,  6000, 10000, 14000, 18000, 20000},
    //.als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
    /* MTK: modified to support AAL */
    .als_level  = {0, 194, 450, 749, 1705, 3845, 4230, 7034, 12907, 16034, 19011, 26895, 32956, 32956, 65535},
    .als_value  = {0, 133, 301, 500, 1002, 2003, 3002, 5003, 8005, 10010, 12010, 16000, 20000, 20000, 20000, 20000},
   	.ps_threshold_high = 400,
    .ps_threshold_low = 350,
   	.p_ps_adjust_para = &cust_ps_adjust_para_apds9921,
	.sec_para_supported = 1,
	.p_ps_adjust_para2 = &cust_ps_adjust_para_apds9922,
};
struct alsps_hw *apds9921_get_cust_alsps_hw(void) {
    cust_alsps_hw.p_ps_adjust_para = &cust_ps_adjust_para_apds9921;
    return &cust_alsps_hw;

}


