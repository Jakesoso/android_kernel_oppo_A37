#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct ps_adjust_para cust_ps_adjust_para = {
    .ps_up = 70,//60,
    .ps_thd_low_notrend = 180,
    .ps_thd_high_notrend = 200,
    .ps_thd_low_trend = 50,//40,
    .ps_thd_high_trend = 70,//60,
    .ps_thd_low_highlight = 3000,//1900,//900,
    .ps_thd_high_highlight = 3200,//2000,//1000,
    .ps_adjust_min = 0,
    .ps_adjust_max = 3200,//1200,
    .highlight_limit = 8000,
    .sampling_time = 60, //Unit:ms
    .sampling_count = 6,
    .dirty_adjust_limit =3000,// 1200,
    .dirty_adjust_low_thd = 700,//2000,//200,//120,
    .dirty_adjust_high_thd = 800,//2200,//300,//200,
};
/*
static struct ps_adjust_para cust_ps_adjust_para_cm36686_2 = {
    .ps_up = 80,//60,
    .ps_thd_low_notrend = 280,
    .ps_thd_high_notrend = 300,
    .ps_thd_low_trend = 60,//40,
    .ps_thd_high_trend = 80,//60,
    .ps_thd_low_highlight = 2400,//900,
    .ps_thd_high_highlight = 2500,//1000,
    .ps_adjust_min = 0,
    .ps_adjust_max = 3800,//1200,
    .highlight_limit = 8000,
    .sampling_time = 60, //Unit:ms
    .sampling_count = 6,
    .dirty_adjust_limit =2200,// 1200,
    .dirty_adjust_low_thd = 1000,//2000,//200,//120,
    .dirty_adjust_high_thd =1200,// 2200,//300,//200,

};*/

static struct alsps_hw cust_alsps_hw = {
	.i2c_num    = 4,
	.eint_gpio = 6,
	.polling_mode_ps =0,
	.polling_mode_als =1,
	.power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
	.power_vol  = VOL_DEFAULT,          /*LDO is not used*/
      //power_id   =MT6331_POWER_LDO_VIO28,
       //ower_vol  = VOL_2800,   
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    //.als_level  = { 0,  1,  1,   7,  15,  15,  100, 1000, 2000,  3000,  6000, 10000, 14000, 18000, 20000},
    //.als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
     .als_level  = {10, 160, 225, 320, 640, 1280, 2600, 5800, 8000, 10240,12000,14000,16000,18000,20000},	/* als_code */
    .als_value  = {0, 10, 40, 65, 90, 145, 225, 300, 550, 930, 1250, 1700, 2600, 5120, 7680, 10240},    /* lux */
    .ps_threshold_high = 1500,
    .ps_threshold_low = 1400,
    .p_ps_adjust_para = &cust_ps_adjust_para,
   // .p_ps_adjust_para_2 = &cust_ps_adjust_para_cm36686_2,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    cust_alsps_hw.p_ps_adjust_para = &cust_ps_adjust_para;
   // cust_alsps_hw.p_ps_adjust_para_2 = &cust_ps_adjust_para_cm36686_2;
    return &cust_alsps_hw;
}

