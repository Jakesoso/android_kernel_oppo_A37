#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include <soc/oppo/oppo_project.h>

//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int mtkfb_set_backlight_level(unsigned int level);
extern int disp_bls_set_backlight(unsigned int level);



#ifdef VENDOR_EDIT
/* liping-m@PhoneSW.Multimedia, 2015/10/14  Add for backlight control use panel's dcs command */
extern int lcm_set_backlight(unsigned int level);
#endif /*VENDOR_EDIT*/

#ifdef VENDOR_EDIT
/* liping-m@PhoneSW.Multimedia, 2016/03/15  Add for backlight use lm3630a */
extern int lm3630a_setbacklight(unsigned int level);
extern int lm3697_setbacklight(unsigned int level);
extern unsigned int is_lm3697_backlight(void);
#endif /*VENDOR_EDIT*/

// Only support 64 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT 64
// Support 256 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256

// Configure the support type "BACKLIGHT_LEVEL_PWM_256_SUPPORT" or "BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT" !!
#define BACKLIGHT_LEVEL_PWM_MODE_CONFIG BACKLIGHT_LEVEL_PWM_256_SUPPORT

unsigned int Cust_GetBacklightLevelSupport_byPWM(void)
{
	return BACKLIGHT_LEVEL_PWM_MODE_CONFIG;
}

unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;

    mapped_level = level;

	return mapped_level;
}

//#ifdef VENDOR_EDIT
/* Xinqin.Yang@PhoneSW.Multimedia, 2015/02/08  Add for backlight control use panel's dcs command */
int cust_set_backlight(int level, int div)
{
    //printk("%s YXQ level is %d\n", __func__, level);

    //lcm_set_backlight(level);
//liping-m@PhoneSW.Multimedia, 2016/08/01 modify backlight 80% to JDI panel CTTEST BUGid834171
#ifdef OPPO_CTTEST_FLAG
if(is_project(OPPO_16031)||is_project(OPPO_16034)){
	level = (int)((level * 4)/5);
	printk("%s level is %d\n", __func__, level);
}
#endif
 #if defined(OPPO_JDI_R63452_FHD1080_DSI_CMD)
	if(is_project(OPPO_16031)||is_project(OPPO_16034)){
	   if(is_lm3697_backlight()){
    	   lm3697_setbacklight(level);
        }else{
     	   lm3630a_setbacklight(level);
        }
    }else{
    	mtkfb_set_backlight_level(level);
	}
 #else
 	mtkfb_set_backlight_level(level);
 #endif

    return 0;
}
//#endif /*VENDOR_EDIT*/
/*

 * To explain How to set these para for cust_led_list[] of led/backlight
 * "name" para: led or backlight
 * "mode" para:which mode for led/backlight
 *	such as:
 *			MT65XX_LED_MODE_NONE,
 *			MT65XX_LED_MODE_PWM,
 *			MT65XX_LED_MODE_GPIO,
 *			MT65XX_LED_MODE_PMIC,
 *			MT65XX_LED_MODE_CUST_LCM,
 *			MT65XX_LED_MODE_CUST_BLS_PWM
 *
 *"data" para: control methord for led/backlight
 *   such as:
 *			MT65XX_LED_PMIC_LCD_ISINK=0,
 *			MT65XX_LED_PMIC_NLED_ISINK0,
 *			MT65XX_LED_PMIC_NLED_ISINK1,
 *			MT65XX_LED_PMIC_NLED_ISINK2,
 *			MT65XX_LED_PMIC_NLED_ISINK3
 *
 *"PWM_config" para:PWM(AP side Or BLS module), by default setting{0,0,0,0,0} Or {0}
 *struct PWM_config {
 *  int clock_source;
 *  int div;
 *  int low_duration;
 *  int High_duration;
 *  BOOL pmic_pad;//AP side PWM pin in PMIC chip (only 89 needs confirm); 1:yes 0:no(default)
 *};
 *-------------------------------------------------------------------------------------------
 *   for AP PWM setting as follow:
 *1.	 PWM config data
 *  clock_source: clock source frequency, can be 0/1
 *  div: clock division, can be any value within 0~7 (i.e. 1/2^(div) = /1, /2, /4, /8, /16, /32, /64, /128)
 *  low_duration: only for BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT
 *  High_duration: only for BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT
 *
 *2.	 PWM freq.
 * If BACKLIGHT_LEVEL_PWM_MODE_CONFIG = BACKLIGHT_LEVEL_PWM_256_SUPPORT,
 *	 PWM freq. = clock source / 2^(div) / 256
 *
 * If BACKLIGHT_LEVEL_PWM_MODE_CONFIG = BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT,
 *	 PWM freq. = clock source / 2^(div) / [(High_duration+1)(Level')+(low_duration+1)(64 - Level')]
 *	           = clock source / 2^(div) / [(High_duration+1)*64]     (when low_duration = High_duration)
 *Clock source:
 *	 0: block clock/1625 = 26M/1625 = 16K (MT6571)
 *	 1: block clock = 26M (MT6571)
 *Div: 0~7
 *
 *For example, in MT6571, PWM_config = {1,1,0,0,0}
 *	 ==> PWM freq. = 26M/2^1/256 	 =	50.78 KHz ( when BACKLIGHT_LEVEL_PWM_256_SUPPORT )
 *	 ==> PWM freq. = 26M/2^1/(0+1)*64 = 203.13 KHz ( when BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT )
 *-------------------------------------------------------------------------------------------
 *   for BLS PWM setting as follow:
 *1.	 PWM config data
 *	 clock_source: clock source frequency, can be 0/1/2/3
 *	 div: clock division, can be any value within 0~1023
 *	 low_duration: non-use
 *	 High_duration: non-use
 *	 pmic_pad: non-use
 *
 *2.	 PWM freq.= clock source / (div + 1) /1024
 *Clock source:
 *	 0: 26 MHz
 *	 1: 104 MHz
 *	 2: 124.8 MHz
 *	 3: 156 MHz
 *Div: 0~1023
 *
 *By default, clock_source = 0 and div = 0 => PWM freq. = 26 KHz
 *-------------------------------------------------------------------------------------------
 */
static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
#ifdef VENDOR_EDIT
	//rendong.shi@Basic.drv, 2013/8/1, Add for 13059 keypadlight
	{"white",             MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
#endif
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
#ifndef VENDOR_EDIT
	//wanghao-m@oppo.com, 2015-11-04 Modify for button-backlight control
	{"button-backlight",  MT65XX_LED_MODE_NONE, -1,{0}},
#else
	{"button-backlight", MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1, {0}},
#endif

#ifndef VENDOR_EDIT
	/* liping-m@PhoneSW.Multimedia, 2015/10/14	Modify for backlight control use panel's dcs command */
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (long)disp_bls_set_backlight, {3, 0, 0, 0, 0} },
#else /*VENDOR_EDIT*/
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_LCM, (long)cust_set_backlight, {3, 0, 0, 0, 0} },
#endif /*VENDOR_EDIT*/

};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

