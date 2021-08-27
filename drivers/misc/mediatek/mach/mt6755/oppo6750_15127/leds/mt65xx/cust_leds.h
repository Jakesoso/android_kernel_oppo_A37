#ifndef _CUST_LEDS_H
#define _CUST_LEDS_H

#include <mach/mt_typedefs.h>
enum mt65xx_led_type
{
	MT65XX_LED_TYPE_RED = 0,
	MT65XX_LED_TYPE_GREEN,
	MT65XX_LED_TYPE_BLUE,
	
	#ifdef VENDOR_EDIT
	//rendong.shi@BasicDrv.led modify 2013/09/27 for breathled
	MT65XX_LED_TYPE_WHITE,
	#endif
	
	MT65XX_LED_TYPE_JOGBALL,
	MT65XX_LED_TYPE_KEYBOARD,
	MT65XX_LED_TYPE_BUTTON,	
	MT65XX_LED_TYPE_LCD,
	MT65XX_LED_TYPE_TOTAL,
};



#endif

