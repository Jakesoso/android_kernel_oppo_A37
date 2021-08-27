/* 
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Version: 1.92.1.01
 *	-Add numerator and denominator value to transfer adc to lux
 *	-Add flag to identify whether update continous lux value or discrete lux value defined in cust_alsps.c
 * Version: 1.92.1.02
 *    -pa22x00001.h -> pa22x.h
 * Version: 1.92.1.03
 *    -add ALS_USE_AVG_DATA
 * Version: 1.92.1.04
 *    -add ALS Interrupt
 * Version: 1.0.1.01
 *	  -Combine 1.92.1.04 & 1.89.1.04 
 */
/*
 * Definitions for TXC PA22X series als/ps sensor chip.
 */
#ifndef __PA22X_H__
#define __PA22X_H__

#include <linux/ioctl.h>

#define PA22_DRIVER_VERSION_H	"2.0.1"


/* DEVICE can change */ 
//#define DEVICE_PA22A
#define	DEVICE_PA224	
/*pa22x als/ps Default*/  
#define PA22_I2C_ADDRESS		0x1E  	/* 7 bit Address */
/*-----------------------------------------------------------------------------*/
#define PA22_ALS_TH_HIGH		65535
#define PA22_ALS_TH_LOW		0
/*-----------------------------------------------------------------------------*/
#define PA22_PS_TH_HIGH		40
#define PA22_PS_TH_LOW			25
#define PA22_PS_TH_MIN		  	0		/* Minimun value */
#define PA22_PS_TH_MAX 			255		/* 8 bit MAX */
/*-----------------------------------------------------------------------------*/
#define PA22_PS_TH_BASE_HIGH 		20
#define PA22_PS_TH_BASE_LOW		18
#define PA22_PS_TH_HIGH_MINIMUM	40
#define PA22_PS_TH_INTERVAL			15
/*-----------------------------------------------------------------------------*/
#define PA22_PS_OFFSET_DEFAULT	0	 	/* for X-talk cannceling */
#define PA22_PS_OFFSET_EXTRA		1
#define PA22_PS_OFFSET_MAX			150
#define PA22_PS_OFFSET_MIN			0
#define PA22_FAST_CAL					1
#define PA22_FAST_CAL_ONCE			0

/* pa22x als/ps parameter setting */
#define PA22_ALS_GAIN		3 	/* 0:x1 | 1:x8 | 2:x16 | 3:x80 */
#define PA22_LED_CURR		6 	/* 4:15mA | 5:12mA | 6:10mA | 7:7mA*/

#define PA22_PS_PRST		3 /* 0:1point | 1:2points | 2:4points | 3:8points (for INT) */
#define PA22_ALS_PRST		0	/* 0:1point | 1:2points | 2:4points | 3:8points (for INT) */

#define PA22_PS_SET			1	/* 0:ALS only | 1:PS only | 3:BOTH */
#define PA22_PS_MODE			0	/* 0:OFFSET |1:NORMAL */

#define PA22_INT_TYPE			0 	/* 0:Window type | 1:Hysteresis type for Auto Clear flag */
#define PA22_PS_PERIOD		1	/* 0:6.25 ms | 1:12.5 ms | 2:25 ms | 3:50 ms | 4:100 ms | 5:200 ms | 6:400 ms | 7:800 ms */
#define PA22_ALS_PERIOD	0	/* 0:0 ms | 1:100 ms | 2:300 ms | 3:700 ms | 4:1500 ms  */
#define PA22_PS_FLTFC			0	/* 0~4 */

/*pa22x als/ps sensor register map*/
#define REG_CFG0 				0X00		/* ALS_GAIN(D5-4) | PS_ON(D1) | ALS_ON(D0) */
#define REG_CFG1 				0X01 	/* LED_CURR(D6-4) | PS_PRST(D3-2) | ALS_PRST(D1-0) */
#define REG_CFG2 				0X02 	/* PS_MODE(D6) | CLEAR(D4) | INT_SET(D3-2) | PS_INT(D1) | ALS_INT(D0) */
#define REG_CFG3				0X03		/* INT_TYPE(D6) | PS_PERIOD(D5-3) | ALS_PERIOD(D2-0) */
#define REG_ALS_TL_LSB		0X04		/* ALS Threshold Low LSB */
#define REG_ALS_TL_MSB		0X05		/* ALS Threshold Low MSB */
#define REG_ALS_TH_LSB		0X06		/* ALS Threshold high LSB */
#define REG_ALS_TH_MSB		0X07		/* ALS Threshold high MSB */
#define REG_PS_TL				0X08		/* PS Threshold Low */
#define REG_PS_TH				0X0A	/* PS Threshold High */
#define REG_ALS_DATA_LSB	0X0B	/* ALS DATA LSB */
#define REG_ALS_DATA_MSB	0X0C	/* ALS DATA MSB */
#define REG_PS_DATA			0X0E		/* PS DATA */
#define REG_PS_OFFSET		0X10		/* TBD */
#define REG_PS_SET				0X11		/* 0x82 */
#define REG_CFG4				0x12		/* Typical = 0x0C */
       
	   
#define PA_PRX_FAR_DIFF          7                       /* Orinal recommand is 5 , you can adjust far distance */
#define PA_PRX_NEAR_DIFF         14                      /* Orinal recommand is 15, you can adjust near distance */	   

/* ALS Using average data */
#define ALS_USE_AVG_DATA 0

static u16 als_adc_coefs_IR[] = {
	8700,
	1020,
	265,
	52
};
/* ALS: Update continuous lux or use discrete value defined in cust_alsps.c */
#define PA22_ALS_ADC_TO_LUX_USE_LEVEL	0
#define PA_ALS_CALIBRATION_LUX_TARGET 400
#define PA_ALS_MAX_COUNT 60000
#define PA_ALS_MIN_COUNT 5000

#define PA_ALS_AUTOGAIN	0
/* Interrupt step */
#define forward_step 	25
#define backward_step 	5
#endif

