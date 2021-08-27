/******************************************************************************
 * MODULE       : rohm_bh1745_i2c.h
 * FUNCTION     : Driver header for BH1745 Ambient Light Sensor(RGB) IC
 * AUTHOR       : Shengfan Wen
 * PROGRAMMED   : Software Development & Consulting, ArcharMind
 * MODIFICATION : Modified by Shengfan Wen, DEC/18/2015
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2015 - ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor,Boston, MA  02110-1301, USA.
 *****************************************************************************/
#ifndef _ROHM_BH1745_I2C_H_
#define _ROHM_BH1745_I2C_H_

#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
#include <linux/ioctl.h> /* For _IOR */
#endif

/*-----------------------------------------------------*/

#define BH1745_DGB_SWITCH         // debug switch
#define  BH1745_TAG             "[ALS/PS]BH1745"

#ifdef BH1745_DGB_SWITCH
#define BH1745_DEBUG   1
#else
#define BH1745_DEBUG   0
#endif

#define  BH1745_ERR(f, a...)        do {printk(KERN_ERR BH1745_TAG "ERROR (%s(), %d):"   f, __func__,  __LINE__, ## a);} while (0)
#define  BH1745_WARNING(f, a...)    do {printk(KERN_WARNING BH1745_TAG "(%s(), %d):"     f, __func__,  __LINE__, ## a);} while (0)

#if BH1745_DEBUG
#define  BH1745_FUN()               do {printk(KERN_ERR BH1745_TAG "(%s(), %d)",         __func__,  __LINE__);} while (0)
#define  BH1745_INFO(f, a...)       do {printk(KERN_INFO BH1745_TAG "INFO (%s(), %d):"   f, __func__,  __LINE__, ## a);} while (0)
#define  BH1745_DBG(f, a...)        do {printk(KERN_DEBUG BH1745_TAG "DEBUG (%s(), %d):" f, __func__,  __LINE__, ## a);} while (0)
#else
#define  BH1745_FUN()   do {} while (0)
#define  BH1745_INFO(f, a...)   do {} while (0)
#define  BH1745_DBG(f, a...)   do {} while (0)
#endif
/*-----------------------------------------------------*/


#define FOPEN_MAX_TIME         (1)

#define BH1745_DRIVER_VER      ("alpha.1.0")
#define SM_TIME_UNIT           (1000)
#define MN_TIME_UNIT           (1000000)
#define MASK_CHAR              (0xFF)
#define CLR_LOW2BIT            (0xFC)
#define UNRELATEDNESS          (0xFF)

//#define _RGB_BIG_ENDIAN_       (1)
#ifdef _RGB_BIG_ENDIAN_
#define CONVERT_TO_BE(value) ((((value) >> 8) & 0xFF) | (((value) << 8) & 0xFF00))
#else
#define CONVERT_TO_BE(value) (value)
#endif

#define FIX_UNDER_MEASURE_TIME (0xC)
#define ROHM_RGB_MAX           (65535)
/************ define register for IC ************/
/* BH1745 REGSTER */
#define REG_SYSTEMCONTROL      (0x40)
#define REG_MODECONTROL1       (0x41)
#define REG_MODECONTROL2       (0x42)
#define REG_MODECONTROL3       (0x44)
#define REG_RED_DATA           (0x50)
#define REG_GREEN_DATA         (0x52)
#define REG_BLUE_DATA          (0x54)
#define REG_CLEAR_DATA         (0x56)
#define REG_INTERRUPT          (0x60)
#define REG_INT_PERSISTENCE    (0x61)
#define REG_THRED_HIGH         (0x62)
#define REG_THRED_LOW          (0x64)
#define REG_MANUFACT_ID        (0x92)
#define MANUFACT_VALUE         (0xE0)

/************ define parameter for register ************/
/* REG_SYSTEMCONTROL(0x40) */
#define SW_RESET               (1 << 7)
#define INT_RESET              (1 << 6)

/* REG_MODECONTROL1(0x41) */
#define MEASURE_160MS          (0x00)
#define MEASURE_320MS          (0x01)
#define MEASURE_640MS          (0x02)
#define MEASURE_1280MS         (0x03)
#define MEASURE_2560MS         (0x04)
#define MEASUREMENT_MAX        (0x05)

/* REG_MODECONTROL2(0x42) */
#define ADC_GAIN_X1            (0x00)
#define ADC_GAIN_X2            (0x01)
#define ADC_GAIN_X16           (0x02)
#define RGBC_EN_ON             (1 << 4)  /* RGBC measurement is active */
#define RGBC_EN_OFF            (0 << 4)  /* RGBC measurement is inactive and becomes power down */
#define RGBC_VALID_HIGH        (1 << 7)

/************ definition to dependent on sensor IC ************/
#define FOPEN_MAX_TIME         (1)
#define BH1745_I2C_NAME        ("bh1745_i2c")
#define INPUTDEV_NAME          ("bh1745_inputdev")
#define BH1745_I2C_ADDRESS     (0x39) //7 bits slave address 011 1001

/************ set initial parameter to IC ************/
#define RGB_SET_MODE_CONTROL1  (MEASURE_320MS)
#define RGB_SET_MODE_CONTROL2  (ADC_GAIN_X16 | RGBC_EN_OFF)

#define RGB_SET_MIN_DELAY_TIME (100 * MN_TIME_UNIT)

/* Interface parameter of file system */
#define RGB_DISABLE            (0)
#define RGB_ENABLE             (1)

/************ define register for IC ************/
//system define
#define RGB_RED                (ABS_X)
#define RGB_GREEN              (ABS_Y)
#define RGB_BLUE               (ABS_Z)
#define RGB_CLEAR              (ABS_RX)

/************ define for dts *******************/
#define TP_MODULE_COUNT             "bh1745,tp_moudle_count"
#define TP_MODULE_COUNT_DEFAULT     (3)
#define TP_LUX_PARAMETER_PREFIX     "bh1745,lux"
#define TP_VALUE_LENGTH             (16)
#define TP_COLOR_PARAMETER_PREFIX   "bh1745,color_temperature"

/************ define for char device **********/
#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)
#define ROHM_CHAR_NAME          "rohm"
#define ROHM                    'r'
struct color_parameter;
#define ROHM_GET_PARAMETER      _IOR(ROHM, 0x01, struct color_parameter)
#endif

/*************define for lux calculation ************/
#define JUDGE_FIXED_COEF        (1000)
#define CUT_UNIT                (1000)
#define TRANS                   (16)

/************ typedef struct ************/
/* structure to read data value from sensor */
typedef struct {
    unsigned short red;         /* data value of red data from sensor */
    unsigned short green;       /* data value of green data from sensor */
    unsigned short blue;        /* data value of blue data from sensor */
    unsigned short clear;       /* data value of clear data from sensor */
} READ_DATA_ARG;

/* structure to read data value from sensor */
typedef struct {
    unsigned long lux;          /* data value of lux data from sensor */
    unsigned long color_temp;   /* data value of temperature data from sensor */
} CALC_DATA_ARG;

/* structure to set initial value to sensor */
typedef struct {
    unsigned char  mode1_ctl;    /* value of Mode control1 reg */
    unsigned char  mode2_ctl;    /* value of Mode control2 reg */
} INIT_ARG;

/* structure to activate sensor */
typedef struct {
    unsigned char  power;  /* value of setting power state */
    unsigned char  mode;   /* value of setting power mode */
    unsigned short timing; /* value of setting measure time */
} DATA_STATE;

/* structure to read register value from sensor */
typedef struct {
    unsigned char adr_reg; /* start register value */
    unsigned char *addr;   /* address to save value which read from sensor */
    unsigned char size;    /* length to read */
} GENREAD_ARG;

/* Color type */
typedef enum {
    GOLD = 0,
    WHITE,
    BLACK,
} COLOR_T;

/* structure of one group/color parameter for lux calculation*/
typedef struct {
    u32 judge;
    u32 red[2];
    u32 green[2];
    u32 blue[2];
} LUX_PARAMETER;

/* structure of one tp module parameter for lux calculation */
typedef struct {
    u32           module_id;
    LUX_PARAMETER gold_parameter;
    LUX_PARAMETER white_parameter;
    LUX_PARAMETER black_parameter;
} CALC_LUX_PARAMETER;

#if defined(CONFIG_MTK_ROHM_BH1745_COLOR_TEMPERATURE)

/* structure of one group/color parameter for color temperature calculation */
typedef struct color_parameter {
    char judge[1][TP_VALUE_LENGTH];
    char beff[3][TP_VALUE_LENGTH];
    char color[4][TP_VALUE_LENGTH];
    char cct_eff[4][TP_VALUE_LENGTH];
} COLOR_PARAMETER;

/* structure of one tp module parameter for color temperature calculation */
typedef struct {
    char            module_id[1][TP_VALUE_LENGTH];
    COLOR_PARAMETER gold_parameter;
    COLOR_PARAMETER white_parameter;
    COLOR_PARAMETER black_parameter;
} CALC_COLOR_PARAMETER;



#endif

#endif /* _ROHM_BH1745_I2C_H_ */
