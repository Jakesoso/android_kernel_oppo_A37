#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <mach/mt_typedefs.h>


#include "kd_camera_hw.h"


#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"




/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(PFX fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG printk
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
		do {    \
		   pr_debug(PFX fmt, ##arg); \
		} while (0)
#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif


#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4


extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);

enum{
	HW_VERSION__UNKNOWN,
	HW_VERSION__T0, 	//1200mV
	HW_VERSION__EVT, 	//1030 mV
	HW_VERSION__FDD, 	//820 mV
	HW_VERSION__DVT, 	//680 mV
	HW_VERSION__PVT, 	//540mV
}; 

extern int pcb_hw_version;
static int hw_version = HW_VERSION__UNKNOWN;

u32 pinSetIdx = 0;		/* default main sensor */
u32 pinSet[3][8] = {
	/* for main sensor */
	{CAMERA_CMRST_PIN,
	 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
	 GPIO_OUT_ONE,		/* ON state */
	 GPIO_OUT_ZERO,		/* OFF state */
	 CAMERA_CMPDN_PIN,
	 CAMERA_CMPDN_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 },
	/* for sub sensor */
	{CAMERA_CMRST1_PIN,
	 CAMERA_CMRST1_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 CAMERA_CMPDN1_PIN,
	 CAMERA_CMPDN1_PIN_M_GPIO,
	 GPIO_OUT_ONE,
	 GPIO_OUT_ZERO,
	 },
};
#ifndef CONFIG_MTK_LEGACY
#define CUST_AVDD AVDD - AVDD
#define CUST_DVDD DVDD - AVDD
#define CUST_DOVDD DOVDD - AVDD
#define CUST_AFVDD AFVDD - AVDD
#define CUST_SUB_DVDD SUB_DVDD - AVDD
#define CUST_MAIN2_DVDD MAIN2_DVDD - AVDD
#endif


PowerCust PowerCustList = {
	{
	 {CAMERA_MAIN_AVDD_T0, CAMERA_MAIN_AVDD_M_GPIO, Vol_High},	/* for AVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for DVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for DOVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for AFVDD; */
	 {GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for SUB_DVDD; */
/* {GPIO_SUPPORTED, GPIO_MODE_GPIO, Vol_Low}, */
	 }
};



PowerUp PowerOnList = {
	{
     //imx298
	  {SENSOR_DRVNAME_IMX298_MIPI_RAW,
	  {
	   {RST,   Vol_Low,  5},
	   {SensorMCLK, Vol_High, 5},
	   
	   {AVDD, Vol_2800, 5},
	   {DOVDD, Vol_1800, 5},
	   {DVDD, Vol_1100, 5},
	   {AFVDD, Vol_2800, 5},
	   {RST, Vol_High, 5}
	   },
	  },
     //imx298
	  {SENSOR_DRVNAME_IMX298SUNNY_MIPI_RAW,
	  {
	   {RST,   Vol_Low,  5},
	   {SensorMCLK, Vol_High, 5},
	   
	   {AVDD, Vol_2800, 5},
	   {DOVDD, Vol_1800, 5},
	   {DVDD, Vol_1100, 5},
	   {AFVDD, Vol_2800, 5},
	   {RST, Vol_High, 5}
	   },
	  },
	     //imx298
	  {SENSOR_DRVNAME_IMX298SEMCO_MIPI_RAW,
	  {
	   {RST,   Vol_Low,  5},
	   {SensorMCLK, Vol_High, 5},
	   
	   {AVDD, Vol_2800, 5},
	   {DOVDD, Vol_1800, 5},
	   {DVDD, Vol_1100, 5},
	   {AFVDD, Vol_2800, 5},
	   {RST, Vol_High, 5}
	   },
	  },
	  //s5k3p3sx
	  {SENSOR_DRVNAME_S5K3P3SX_MIPI_RAW,
	  {
	   {RST, Vol_Low, 5},
	   {SensorMCLK, Vol_High, 5},
	   {AVDD, Vol_3000, 5},
	   {DOVDD, Vol_1800, 5},
	   {DVDD, Vol_1200, 5},
	   {RST, Vol_High, 5}
	   },
	  },
	  //s5k3p3sp
	  {SENSOR_DRVNAME_S5K3P3SP_MIPI_RAW,
	  {
	   {RST, Vol_Low, 5},
	   {SensorMCLK, Vol_High, 5},
	   {AVDD, Vol_3000, 5},
	   {DOVDD, Vol_1800, 5},
	   {DVDD, Vol_1200, 5},
	   {RST, Vol_High, 5}
	   },
	  },
	 /* add new sensor before this line */
	 {NULL,},
	 }
};

PowerUp PowerDownList = {
	{

     //imx298
	 {SENSOR_DRVNAME_IMX298_MIPI_RAW,
     {
		{RST,   Vol_Low,  5},
		{AFVDD, Vol_2800, 5},
		{DVDD,	Vol_1100, 5},
		{DOVDD, Vol_1800, 5},
		{AVDD,	Vol_2800, 5},
		{SensorMCLK,Vol_Low, 0}
     	},
	  }, 
     //imx298
	 {SENSOR_DRVNAME_IMX298SUNNY_MIPI_RAW,
     {
		{RST,   Vol_Low,  5},
		{AFVDD, Vol_2800, 5},
		{DVDD,	Vol_1100, 5},
		{DOVDD, Vol_1800, 5},
		{AVDD,	Vol_2800, 5},
		{SensorMCLK,Vol_Low, 0}
     	},
	  },
     //imx298
	 {SENSOR_DRVNAME_IMX298SEMCO_MIPI_RAW,
     {
		{RST,   Vol_Low,  5},
		{AFVDD, Vol_2800, 5},
		{DVDD,	Vol_1100, 5},
		{DOVDD, Vol_1800, 5},
		{AVDD,	Vol_2800, 5},
		{SensorMCLK,Vol_Low, 0}
     	},
	  },
	  //s5k3p3sx
	  {SENSOR_DRVNAME_S5K3P3SX_MIPI_RAW,
	  {
		{RST, Vol_Low, 5},
		{DVDD, Vol_1200, 0},
		{DOVDD, Vol_1800, 5},
		{AVDD, Vol_3000, 0},
		{SensorMCLK,Vol_Low, 0}
	   },
	  },
	  //s5k3p3sp
	  {SENSOR_DRVNAME_S5K3P3SP_MIPI_RAW,
	  {
		{RST, Vol_Low, 5},
		{DVDD, Vol_1200, 0},
		{DOVDD, Vol_1800, 5},
		{AVDD, Vol_3000, 0},
		{SensorMCLK,Vol_Low, 0}
	   },
	  },
	 /* add new sensor before this line */
	 {NULL,},
	 }
};


BOOL hwpoweron(PowerInformation pwInfo, char *mode_name)
{
    hw_version = pcb_hw_version;
	if (pwInfo.PowerType == AVDD) {
		PK_DBG("[%s] [CAMERA SENSOR] AVDD HW_VERSON = %d\n", __func__,hw_version);
		if(pinSetIdx == 1){
            PK_DBG("[%s] [CAMERA SENSOR] set AVDD gpio82 high\n", __func__);
			if(mt_set_gpio_mode(CAMERA_SUB_AVDD, GPIO_MODE_00)){PK_DBG("set gpio82 mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_SUB_AVDD,  GPIO_DIR_OUT)){PK_DBG("set gpio82 dir failed!! \n");}
			if(mt_set_gpio_out(CAMERA_SUB_AVDD,  Vol_High)){PK_DBG("set gpio82 failed!! \n");}	
		}else {
            if(hw_version == HW_VERSION__T0){
                PK_DBG("[%s] [CAMERA SENSOR] T0 set AVDD gpio60 high\n", __func__);
                if(mt_set_gpio_mode(CAMERA_MAIN_AVDD_T0, GPIO_MODE_00)){PK_DBG("set gpio60 mode failed!! \n");}
			    if(mt_set_gpio_dir(CAMERA_MAIN_AVDD_T0,  GPIO_DIR_OUT)){PK_DBG("set gpio60 dir failed!! \n");}
			    if(mt_set_gpio_out(CAMERA_MAIN_AVDD_T0,  Vol_High)){PK_DBG("set gpio60 failed!! \n");}
            }
            else {
                PK_DBG("[%s] [CAMERA SENSOR] EVT set AVDD gpio63 high\n", __func__);
                if(mt_set_gpio_mode(CAMERA_MAIN_AVDD_EVT, GPIO_MODE_00)){PK_DBG("set gpio63 mode failed!! \n");}
			    if(mt_set_gpio_dir(CAMERA_MAIN_AVDD_EVT,  GPIO_DIR_OUT)){PK_DBG("set gpio63 dir failed!! \n");}
			    if(mt_set_gpio_out(CAMERA_MAIN_AVDD_EVT,  Vol_High)){PK_DBG("set gpio63 failed!! \n");}
            }
		}
		

	} else if (pwInfo.PowerType == DVDD) {
		PK_DBG("[%s] [CAMERA SENSOR] DVDD \n", __func__);
		if (PowerCustList.PowerCustInfo[1].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerOn(pwInfo.PowerType, pwInfo.Voltage * 1000, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[1].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == DOVDD) {
	     PK_DBG("[%s] [CAMERA SENSOR] DOVDD \n", __func__);
		if (PowerCustList.PowerCustInfo[2].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerOn(pwInfo.PowerType, pwInfo.Voltage * 1000, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[2].Gpio_Pin,
			     PowerCustList.PowerCustInfo[2].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[2].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[2].Gpio_Pin,
			     PowerCustList.PowerCustInfo[2].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == AFVDD) {
		PK_DBG("[%s] [CAMERA SENSOR] AFVDD \n", __func__);
		#if 1
		//PK_DBG("[CAMERA SENSOR] Skip AFVDD setting\n");
		if (PowerCustList.PowerCustInfo[3].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerOn(pwInfo.PowerType, pwInfo.Voltage * 1000, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}

			if (PowerCustList.PowerCustInfo[4].Gpio_Pin != GPIO_UNSUPPORTED) {
				mdelay(5);
				if (mt_set_gpio_mode
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
					PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
				}
				if (mt_set_gpio_dir
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
				}
				if (mt_set_gpio_out
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Voltage)) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
				}
			}
		}
		#endif
	} else if (pwInfo.PowerType == PDN) {
		PK_DBG("[%s] [CAMERA SENSOR] PDN \n", __func__);
		#if 0
		if(pinSetIdx == 0){
			if(mt_set_gpio_mode(CAMERA_CMPDN_PIN, GPIO_MODE_00)){PK_DBG("set PDN mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_CMPDN_PIN,  GPIO_DIR_OUT)){PK_DBG("set PDN dir failed!! \n");}
			if(mt_set_gpio_out(CAMERA_CMPDN_PIN,  Vol_Low)){PK_DBG("set PDN failed!! \n");}
			mdelay(5);
			if(mt_set_gpio_out(CAMERA_CMPDN_PIN,  Vol_High)){PK_DBG("set PDN failed!! \n");}
		}else if(pinSetIdx == 1){
			if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN, GPIO_MODE_00)){PK_DBG("set Sub PDN mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,  GPIO_DIR_OUT)){PK_DBG("set Sub PDN dir failed!! \n");}
			if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,  Vol_Low)){PK_DBG("set Sub PDN failed!! \n");}
			mdelay(5);
			if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,  Vol_High)){PK_DBG("set PDN failed!! \n");}
		}
		#endif

	} else if (pwInfo.PowerType == RST) {
		PK_DBG("[%s] [CAMERA SENSOR] RST Voltage=%d\n", __func__, pwInfo.Voltage);
		if (pinSetIdx == 0) {
			if(mt_set_gpio_mode(CAMERA_CMRST_PIN, GPIO_MODE_00)){PK_DBG("set Main Rst mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_CMRST_PIN,  GPIO_DIR_OUT)){PK_DBG("set Main Rst dir failed!! \n");}
			if(pwInfo.Voltage == Vol_High){
				if(mt_set_gpio_out(CAMERA_CMRST_PIN,  Vol_High)){PK_DBG("set Main Rst failed!! \n");}
			}else{
				if(mt_set_gpio_out(CAMERA_CMRST_PIN,  Vol_Low)){PK_DBG("set Main Rst failed!! \n");}
			}
		} else if (pinSetIdx == 1) {
			if(mt_set_gpio_mode(CAMERA_CMRST1_PIN, GPIO_MODE_00)){PK_DBG("set Sub Rst mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_CMRST1_PIN,  GPIO_DIR_OUT)){PK_DBG("set Sub Rst dir failed!! \n");}
			if(pwInfo.Voltage == Vol_High){
				if(mt_set_gpio_out(CAMERA_CMRST1_PIN,  Vol_High)){PK_DBG("set Sub Rst failed!! \n");}
			}else{
				if(mt_set_gpio_out(CAMERA_CMRST1_PIN,  Vol_Low)){PK_DBG("set Sub Rst failed!! \n");}
			}	
		}

	} else if (pwInfo.PowerType == SensorMCLK) {
		if (pinSetIdx == 0) {
			/* PK_DBG("Sensor MCLK1 On"); */
			ISP_MCLK1_EN(TRUE);
		} else if (pinSetIdx == 1) {
			/* PK_DBG("Sensor MCLK2 On"); */
			ISP_MCLK2_EN(TRUE);
		}
	} else {
	}
	if (pwInfo.Delay > 0)
		mdelay(pwInfo.Delay);
	return TRUE;
}



BOOL hwpowerdown(PowerInformation pwInfo, char *mode_name)
{
    hw_version = pcb_hw_version;
	if (pwInfo.PowerType == AVDD) {
		PK_DBG("[%s] [CAMERA SENSOR] AVDD HW_VERSON = %d\n", __func__,hw_version);
		if(pinSetIdx == 1){
            PK_DBG("[%s] [CAMERA SENSOR] set AVDD gpio82 low\n", __func__);
			if(mt_set_gpio_mode(CAMERA_SUB_AVDD, GPIO_MODE_00)){PK_DBG("set gpio82 mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_SUB_AVDD,  GPIO_DIR_OUT)){PK_DBG("set gpio82 dir failed!! \n");}
			if(mt_set_gpio_out(CAMERA_SUB_AVDD,  Vol_Low)){PK_DBG("set gpio82 failed!! \n");}	
		} else {
            if(hw_version == HW_VERSION__T0){
                PK_DBG("[%s] [CAMERA SENSOR] T0 set AVDD gpio60 low\n", __func__);
                if(mt_set_gpio_mode(CAMERA_MAIN_AVDD_T0, GPIO_MODE_00)){PK_DBG("set gpio60 mode failed!! \n");}
			    if(mt_set_gpio_dir(CAMERA_MAIN_AVDD_T0,  GPIO_DIR_OUT)){PK_DBG("set gpio60 dir failed!! \n");}
			    if(mt_set_gpio_out(CAMERA_MAIN_AVDD_T0,  Vol_Low)){PK_DBG("set gpio60 failed!! \n");}
            }
            else {
                PK_DBG("[%s] [CAMERA SENSOR] EVT set AVDD gpio63 low\n", __func__);
                if(mt_set_gpio_mode(CAMERA_MAIN_AVDD_EVT, GPIO_MODE_00)){PK_DBG("set gpio63 mode failed!! \n");}
			    if(mt_set_gpio_dir(CAMERA_MAIN_AVDD_EVT,  GPIO_DIR_OUT)){PK_DBG("set gpio63 dir failed!! \n");}
			    if(mt_set_gpio_out(CAMERA_MAIN_AVDD_EVT,  Vol_Low)){PK_DBG("set gpio63 failed!! \n");}
            }
		}
		

	} else if (pwInfo.PowerType == DVDD) {
		PK_DBG("[%s] [CAMERA SENSOR] DVDD \n", __func__);
		if (PowerCustList.PowerCustInfo[1].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerDown(pwInfo.PowerType, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[1].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[1].Gpio_Pin,
			     PowerCustList.PowerCustInfo[1].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == DOVDD) {
		PK_DBG("[%s] [CAMERA SENSOR] DOVDD \n", __func__);
		if (PowerCustList.PowerCustInfo[2].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerDown(pwInfo.PowerType, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[2].Gpio_Pin,
			     PowerCustList.PowerCustInfo[2].Gpio_Mode)) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[2].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[2].Gpio_Pin,
			     PowerCustList.PowerCustInfo[2].Voltage)) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == AFVDD) {
		PK_DBG("[CAMERA SENSOR] Skip AFVDD setting\n");
		if (PowerCustList.PowerCustInfo[3].Gpio_Pin == GPIO_UNSUPPORTED) {
			if (TRUE != hwPowerDown(pwInfo.PowerType, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		} else {
			if (mt_set_gpio_mode
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
				PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
			}
			if (mt_set_gpio_dir(PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
			}
			if (mt_set_gpio_out
			    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
			     PowerCustList.PowerCustInfo[3].Voltage)) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}

			if (PowerCustList.PowerCustInfo[4].Gpio_Pin != GPIO_UNSUPPORTED) {
				mdelay(5);
				if (mt_set_gpio_mode
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Gpio_Mode)) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
				}
				if (mt_set_gpio_dir
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin, GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
				}
				if (mt_set_gpio_out
				    (PowerCustList.PowerCustInfo[3].Gpio_Pin,
				     PowerCustList.PowerCustInfo[3].Voltage)) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			}
		}
	} else if (pwInfo.PowerType == PDN) {
		PK_DBG("[%s] [CAMERA SENSOR] PDN \n", __func__);
		
#if 1
		if(pinSetIdx == 0){
			if(mt_set_gpio_mode(CAMERA_CMPDN_PIN, GPIO_MODE_00)){PK_DBG("set PDN mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_CMPDN_PIN,  GPIO_DIR_OUT)){PK_DBG("set PDN dir failed!! \n");}
			if(pwInfo.Voltage == Vol_High){
				if(mt_set_gpio_out(CAMERA_CMPDN_PIN,  Vol_High)){PK_DBG("set PDN Hight failed!! \n");}
			}else{
				if(mt_set_gpio_out(CAMERA_CMPDN_PIN,  Vol_Low)){PK_DBG("set PDN Low failed!! \n");}
			}	
		}else if(pinSetIdx == 1){
			if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN, GPIO_MODE_00)){PK_DBG("set Sub PDN mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,  GPIO_DIR_OUT)){PK_DBG("set Sub PDN dir failed!! \n");}
			if(pwInfo.Voltage == Vol_High){
				if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,  Vol_High)){PK_DBG("set Sub PDN High failed!! \n");}
			}else{
				if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,  Vol_Low)){PK_DBG("set Sub PDN Low failed!! \n");}
			}
		}
#endif
		#if 0
		if (mt_set_gpio_mode
		    (pinSet[pinSetIdx][IDX_PS_CMPDN],
		     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
			PK_DBG("[CAMERA LENS] set gpio mode failed!!\n");
		}
		if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
			PK_DBG("[CAMERA LENS] set gpio dir failed!!\n");
		}
		if (pwInfo.Voltage == Vol_High) {
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMPDN],
			     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}
			msleep(1);
		} else {
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMPDN],
			     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {
				PK_DBG("[CAMERA LENS] set gpio failed!!\n");
			}
			msleep(1);
		}
		#endif
	} else if (pwInfo.PowerType == RST) {
		PK_DBG("[%s] [CAMERA SENSOR] RST Voltage=%d\n", __func__, pwInfo.Voltage);
		if (pinSetIdx == 0) {
			if(mt_set_gpio_mode(CAMERA_CMRST_PIN, GPIO_MODE_00)){PK_DBG("set Main Rst mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_CMRST_PIN,  GPIO_DIR_OUT)){PK_DBG("set Main Rst dir failed!! \n");}
			if(pwInfo.Voltage == Vol_High){
				if(mt_set_gpio_out(CAMERA_CMRST_PIN,  Vol_High)){PK_DBG("set Main Rst High failed!! \n");}
			}else{
				if(mt_set_gpio_out(CAMERA_CMRST_PIN,  Vol_Low)){PK_DBG("set Main Rst Low failed!! \n");}
			}
		} else if (pinSetIdx == 1) {
			if(mt_set_gpio_mode(CAMERA_CMRST1_PIN, GPIO_MODE_00)){PK_DBG("set Sub Rst mode failed!! \n");}
			if(mt_set_gpio_dir(CAMERA_CMRST1_PIN,  GPIO_DIR_OUT)){PK_DBG("set Sub Rst dir failed!! \n");}
			if(pwInfo.Voltage == Vol_High){
				if(mt_set_gpio_out(CAMERA_CMRST1_PIN,  Vol_High)){PK_DBG("set Sub Rst High failed!! \n");}
			}else{
				if(mt_set_gpio_out(CAMERA_CMRST1_PIN,  Vol_Low)){PK_DBG("set Sub Rst Low failed!! \n");}
			}	
		}		
#if 0		
		if (pinSetIdx == 0) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
			if (mt_set_gpio_mode
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
			}
			if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
			}
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}
			if (pwInfo.Voltage == Vol_High) {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			} else {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			}
#else
			if (mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
			}
			if (pwInfo.Voltage == Vol_High) {
				if (mt6306_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
				}
			} else {
				if (mt6306_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
					PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
				}
			}
#endif
		} else if (pinSetIdx == 1) {
			if (mt_set_gpio_mode
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
				PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
			}
			if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
				PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
			}
			if (mt_set_gpio_out
			    (pinSet[pinSetIdx][IDX_PS_CMRST],
			     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
				PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
			}
			if (pwInfo.Voltage == Vol_High) {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			} else {
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMRST],
				     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!!\n");
				}
			}
		}
#endif

	} else if (pwInfo.PowerType == SensorMCLK) {
		if (pinSetIdx == 0) {
			ISP_MCLK1_EN(FALSE);
		} else if (pinSetIdx == 1) {
			ISP_MCLK2_EN(FALSE);
		}
	} else {
	}
	return TRUE;
}



#if 0
int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On,
		       char *mode_name)
{

	int pwListIdx, pwIdx;
	BOOL sensorInPowerList = KAL_FALSE;

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx) {
		pinSetIdx = 0;
	} else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
		pinSetIdx = 1;
	} else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
		pinSetIdx = 2;
	}
	if(strcmp(currSensorName, "s5k3p3sxmipiraw") == 0){
		pinSetIdx = 1;
	}
	else if(strcmp(currSensorName, "s5k3l8mipiraw") == 0){
		pinSetIdx = 0;
	}
	else {
		pinSetIdx = 2;
	}
	/* power ON */
	if (On) {
		PK_DBG("kdCISModulePowerOn -on:currSensorName=%s  pinSetIdx=%d\n", currSensorName, pinSetIdx);
	
		for (pwListIdx = 0; pwListIdx < 16; pwListIdx++) {
			if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (0 ==
				strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,
				       currSensorName))) {
				PK_DBG("sensorIdx:%d\n", SensorIdx);

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 0; pwIdx < 10; pwIdx++) {
					if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
					    PowerType != VDD_None) {
						if (hwpoweron
						    (PowerOnList.PowerSeq[pwListIdx].
						     PowerInfo[pwIdx], mode_name) == FALSE)
							goto _kdCISModulePowerOn_exit_;
					} else {
						/* PK_DBG("pwIdx=%d\n", pwIdx); */
						break;
					}
				}
				break;
			} else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			} else {
			}
		}

		/* Temp solution: default power on/off sequence */
		if (KAL_FALSE == sensorInPowerList) {
			PK_DBG("Default power on sequence");

			if (pinSetIdx == 0) {
				ISP_MCLK1_EN(1);
			} else if (pinSetIdx == 1) {
				ISP_MCLK2_EN(1);
			}
			/* First Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				}
			}

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (0 == pinSetIdx) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#else
					if (mt6306_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt6306_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#endif
				} else {
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
				}
			}
			/* VCAM_IO */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d\n",
				     CAMERA_POWER_VCAM_D2);
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_A */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n",
				     CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d\n",
				     CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}
			/* AF_VCC */
			if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800*1000, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d\n",
				     CAMERA_POWER_VCAM_A2);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(5);

			/* enable active sensor */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON])) {
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				}
			}

			mdelay(1);

			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (0 == pinSetIdx) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#else
					if (mt6306_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt6306_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#endif

				} else {
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
				}
			}




		}
		/*
		   if(pinSetIdx==0)
		   for(;;)
		   {}
		 */
		/*
		   if(pinSetIdx==1)
		   for(;;)
		   {}
		 */
	} else {		/* power OFF */
		for (pwListIdx = 0; pwListIdx < 16; pwListIdx++) {
			if (currSensorName && (PowerDownList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (0 ==
				strcmp(PowerDownList.PowerSeq[pwListIdx].SensorName,
				       currSensorName))) {
				PK_DBG("kdCISModulePowerOn get in---\n");
				PK_DBG("sensorIdx:%d\n", SensorIdx);

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 9; pwIdx >= 0; pwIdx--) {
					if (PowerDownList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
					    PowerType != VDD_None) {
						if (hwpowerdown
						    (PowerDownList.PowerSeq[pwListIdx].
						     PowerInfo[pwIdx], mode_name) == FALSE)
							goto _kdCISModulePowerOn_exit_;
						if (pwIdx > 0) {
							if (PowerDownList.PowerSeq[pwListIdx].
							    PowerInfo[pwIdx - 1].Delay > 0)
								mdelay(PowerDownList.
								       PowerSeq[pwListIdx].
								       PowerInfo[pwIdx - 1].Delay);
						}
					} else {
						PK_DBG("pwIdx=%d\n", pwIdx);
					}
				}
			} else if (PowerDownList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			} else {
			}
		}

		/* Temp solution: default power on/off sequence */
		if (KAL_FALSE == sensorInPowerList) {
			PK_DBG("Default power off sequence");

			if (pinSetIdx == 0) {
				ISP_MCLK1_EN(0);
			} else if (pinSetIdx == 1) {
				ISP_MCLK2_EN(0);
			}
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				if (mt_set_gpio_mode
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_MODE])) {
					PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN], GPIO_DIR_OUT)) {
					PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");
				}
				if (mt_set_gpio_out
				    (pinSet[pinSetIdx][IDX_PS_CMPDN],
				     pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF])) {
					PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");
				}
			}


			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
				if (0 == pinSetIdx) {
#ifndef CONFIG_MTK_MT6306_SUPPORT
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#else
					if (mt6306_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt6306_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
#endif
				} else {
					if (mt_set_gpio_mode
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_MODE])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");
					}
					if (mt_set_gpio_dir
					    (pinSet[pinSetIdx][IDX_PS_CMRST], GPIO_DIR_OUT)) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");
					}
					if (mt_set_gpio_out
					    (pinSet[pinSetIdx][IDX_PS_CMRST],
					     pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF])) {
						PK_DBG
						    ("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");
					}
				}
			}


			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d\n",
				     CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_A */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d)\n",
				     CAMERA_POWER_VCAM_A);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_IO */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n",
				     CAMERA_POWER_VCAM_D2);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
			/* AF_VCC */
			if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2, mode_name)) {
				PK_DBG
				    ("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d\n",
				     CAMERA_POWER_VCAM_A2);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}







		}
	}			/*  */

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}
#else
int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
	int pwListIdx,pwIdx;
    BOOL sensorInPowerList = KAL_FALSE;

	PK_DBG("%s: Enter \n", __func__);

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
	{
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
	{
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
	{
        pinSetIdx = 2;
    }
	/*if(strcmp(currSensorName, "s5k3p3sxmipiraw") == 0){
		pinSetIdx = 1;
	}
	else if(strcmp(currSensorName, "s5k3l8mipiraw") == 0){
		pinSetIdx = 0;
	}
	else {
		pinSetIdx = 2;
	}*/
    //power ON
    if (On)
	{
		PK_DBG("[Power ON] currSensorName=%s, pinSetIdx=%d\n",currSensorName,pinSetIdx);

		for(pwListIdx=0; pwListIdx<16; pwListIdx++)
		{
			if(currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
			{
                sensorInPowerList = KAL_TRUE;

				for(pwIdx=0; pwIdx<10; pwIdx++)
				{
					if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType != VDD_None)
					{
						if(hwpoweron(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx],mode_name) == FALSE)
							goto _kdCISModulePowerOn_exit_;
					}
					else
					{
						PK_DBG("pwIdx=%d \n",pwIdx);
						break;
					}
				}
				break;
			}
			else if(PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
			{
				break;
			}
			else{}
		}

        // Temp solution: default power on/off sequence
        if(KAL_FALSE == sensorInPowerList)
        {
            PK_DBG("Default power on sequence");

            if(pinSetIdx == 0 )
			{
                ISP_MCLK1_EN(1);
            }
            else if (pinSetIdx == 1)
			{
                ISP_MCLK2_EN(1);
            }

            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
			{
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
			{
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("set gpio failed!! (CMRST)\n");}
            }

            //VCAM_IO
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800, mode_name))
            {
                PK_DBG("Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_D2);
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_A
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
            {
                 PK_DBG("Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }

             //AF_VCC
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            {
                PK_DBG("Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_A2);
                goto _kdCISModulePowerOn_exit_;
            }

            mdelay(5);

            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
			{
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("set gpio failed!! (CMPDN)\n");}
            }

            mdelay(1);

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
			{
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("set gpio failed!! (CMRST)\n");}
            }
        }
	}
    else
	{
		//power OFF
		PK_DBG("[Power OFF] currSensorName=%s, pinSetIdx=%d\n",currSensorName,pinSetIdx);

		for(pwListIdx=0 ; pwListIdx<16; pwListIdx++)
		{
			#ifndef VENDOR_EDIT
			/* OPPO 2015-04-11 liubin modify for camera power down sequence */
			if(currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
			{
                sensorInPowerList = KAL_TRUE;

				for(pwIdx=9;pwIdx>=0;pwIdx--)
				{
					if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType != VDD_None)
					{
						if(hwpowerdown(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx],mode_name) == FALSE)
							goto _kdCISModulePowerOn_exit_;
						if(pwIdx > 0)
						{
							if(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx-1].Delay > 0)
								mdelay(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx-1].Delay);
						}
					}
					else
					{
						PK_DBG("pwIdx=%d \n",pwIdx);
					}
				}
			}
			else if(PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
			{
				break;
			}
			#else
			if(currSensorName && (PowerDownList.PowerSeq[pwListIdx].SensorName!=NULL) && (0 == strcmp(PowerDownList.PowerSeq[pwListIdx].SensorName,currSensorName)))
			{
                sensorInPowerList = KAL_TRUE;

				for(pwIdx=0;pwIdx<10;pwIdx++)
				{
					if(PowerDownList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType != VDD_None)
					{
						if(hwpowerdown(PowerDownList.PowerSeq[pwListIdx].PowerInfo[pwIdx],mode_name) == FALSE)
							goto _kdCISModulePowerOn_exit_;
						if(PowerDownList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Delay > 0)
							mdelay(PowerDownList.PowerSeq[pwListIdx].PowerInfo[pwIdx].Delay);
					}
					else
					{
						PK_DBG("pwIdx=%d \n",pwIdx);
					}
				}
			}
			else if(PowerDownList.PowerSeq[pwListIdx].SensorName == NULL)
			{
				break;
			}
			#endif
			else{}
		}

        // Temp solution: default power on/off sequence
        if(KAL_FALSE == sensorInPowerList)
        {
            PK_DBG("Default power off sequence");

            if(pinSetIdx == 0)
            {
                ISP_MCLK1_EN(0);
            }
            else if (pinSetIdx == 1)
            {
                ISP_MCLK2_EN(0);
            }

            //Set Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN] && pinSetIdx == 1)
            {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("set gpio failed!! (CMPDN)\n");}
            }

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("set gpio failed!! (CMRST)\n");}
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
            {
                PK_DBG("Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_A
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
            {
                PK_DBG("Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //VCAM_IO
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name))
            {
                PK_DBG("Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_D2);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

            //AF_VCC
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            {
                PK_DBG("Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_A2);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
        }
    }//

    PK_DBG("%s: Exit \n", __func__);

	return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

#endif

EXPORT_SYMBOL(kdCISModulePowerOn);

#ifdef VENDOR_EDIT	
/*jindian.guan 2015/11/13 add for io power*/
int kdVIOPowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
    u32 pinSetIdx = 0;

    if (On) {
        if (TRUE != hwPowerOn(DOVDD, Vol_1800 * 1000, mode_name)) {
            PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            return FALSE;
        }
    }
    else{
        if (TRUE != hwPowerDown(DOVDD, mode_name)) {
            PK_DBG("[CAMERA SENSOR] Fail to disable digital power\n");
            return FALSE;
        }
    }
    return TRUE;
}
EXPORT_SYMBOL(kdVIOPowerOn);
#endif /*VENDOR_EDIT*/




/* !-- */
/*  */
