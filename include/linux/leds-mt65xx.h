/******************************************************************************
 * mt65xx_leds.h
 *
 * Copyright 2010 MediaTek Co.,Ltd.
 *
 ******************************************************************************/
#ifndef _MT65XX_LEDS_H
#define _MT65XX_LEDS_H

#include <linux/leds.h>
#include <cust_leds.h>

#ifdef CONFIG_MTK_LEDS
extern int mt65xx_leds_brightness_set(enum mt65xx_led_type type, enum led_brightness value);
extern int backlight_brightness_set(int level);
#else
static inline int mt65xx_leds_brightness_set(enum mt65xx_led_type type, enum led_brightness value){return 0;}
static inline int backlight_brightness_set(int level){return 0;}
#endif

#endif

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2015.9.30 for charging
extern void mt_step_set_pmic(int step);
extern void mt_duty_set_pmic(int duty);
extern int  mt_step_get_pmic(void);
extern int mt_duty_get_pmic(void);
#endif /* VENDOR_EDIT */
