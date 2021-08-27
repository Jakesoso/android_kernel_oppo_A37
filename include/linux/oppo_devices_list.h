#ifndef __OPPO_DEVICES_LIST_H__
#define __OPPO_DEVICES_LIST_H__

typedef enum
{
    LCD_NONE = 0,
    LCD_HITACHI,
    LCD_HITACHI_VIDEO,
    LCD_TRULY,
	LCD_BYD,
	LCD_JDI,
	LCD_TRULY_OLD,
	LCD_TRULY_NEW,
	LCD_NOVTEK,
	LCD_TIANMA,
	LCD_TRULY_COMMAND,
	LCD_TRULY_VIDEO,
	LCD_BYD_COMMAND,
	LCD_BYD_VIDEO,
	LCD_TRULY_LG_NT35521,
	LCD_BYD_JDI_NT35521,
	LCD_TRULY_LG_HX8394,
	LCD_TRULY_JDI_NT35521,
	/* Xinqin.Yang@PhoneSW.Multimedia, 2014/09/20  Add for 14053 JDI panel begin*/
    LCD_JDI_R63417_VIDEO,
    /* Xinqin.Yang@PhoneSW.Multimedia, 2014/09/20  Add for 14053 JDI panel end*/
    /* Xinqin.Yang@PhoneSW.Multimedia, 2015/02/08  Add for 15011 SAMSUNG panel begin*/
    LCD_SAMSUNG_S6E3FA3_CMD,
    /* Xinqin.Yang@PhoneSW.Multimedia, 2014/09/20  Add for 15011 SAMSUNG panel end*/
    /* liuyan@Onlinerd.driver, 2014/11/28  Add for add for 14007 sansung panel */
    LCD_OLED_S6E3FA2_CMD,
    /*CONFIG_VENDOR_EDIT*/
	/* liping-m@PhoneSW.Multimedia, 2015/11/26  Add for add for sansung new panel */
    LCD_SAMSUNG_EA8064T_VIDEO,
    LCD_SAMSUNG_EA8064T_CMD,
    /*CONFIG_VENDOR_EDIT*/
	/* liping-m@PhoneSW.Multimedia, 2016/02/18  Add for add for BOE,TRULY,tianma new panel */
	LCD_BOE_ILI9881C_VIDEO,
	LCD_TRULY_NT35521S_VIDEO,
	LCD_TIANMA_NT35521S_VIDEO,
    LCD_OTHRE
} LCD_DEV;

typedef enum
{
    TP_NONE = 0,
    TP_ALPS,
    TP_TRULY,
    TP_YOUNGFAST,
	TP_NITTO,
	TP_OIKE,
	TP_OFILM,
	TP_TPK,
	TP_JDI,
    TP_OFILM_WHITE,
	TP_OFILM_BLACK,
	TP_TPK_WHITE,
	TP_TPK_BLACK,
	TP_TPK_GOODIX,
	TP_TPK_SYNAPTICS,
	TP_TRULY_GOODIX,
	TP_TRULY_SYNAPTICS,
	TP_SAMSUNG_SYNAPTICS,
    TP_OTHER

} TP_DEV;

typedef enum
{
    CAMERA_BACK_NONE = 0,
    CAMERA_BACK_OV5650MIPI,
    CAMERA_BACK_OV5647,
    CAMERA_BACK_OV5647AC,
    CAMERA_BACK_S5K4E5YA,
	//lvxj@MutimediaDrv.camsensor, 2012/08/31, add for 12021 test mode
    CAMERA_BACK_IMX105MIPI,
    CAMERA_BACK_VD6803A,
    CAMERA_BACK_MS2R,
    //zhengrong.zhang@CameraDrv, 2013/09/24, add for 13059 test mode
    CAMERA_BACK_IMX179,
    //xianglie.liu@CameraDrv, 2013/12/24, add for 13085 factory mode
    CAMERA_BACK_OV5648,
    //bin.liu@CameraDrv, 2014/11/18, add for 14053 test mode
    CAMERA_BACK_IMX214,
    /*oppo hufeng 20150515 add for device test*/
    CAMERA_BACK_IMX278,
    //zhangkw add
    CAMERA_BACK_S5K3M2,
    //hufeng add
    CAMERA_BACK_S5K3L8,
    CAMERA_BACK_OTHER
} CAMERA_BACK_DEV;

typedef enum
{
    CAMERA_FRONT_NONE = 0,
    CAMERA_FRONT_mt9d115,
    CAMERA_FRONT_s5k5bbgx,
	//lvxj@MutimediaDrv.camsensor, 2012/08/31, add for 12021 test mode
    CAMERA_FRONT_ov7675,
    CAMERA_FRONT_hi704,
    CAMERA_FRONT_HI253,
    //zhengrong.zhang@CameraDrv, 2013/09/24, add for 13059 test mode
    CAMERA_FRONT_HI256,
    CAMERA_FRONT_OV5647,
     //zhangkw@CameraDrv.camera, 2013/06/08, add for 13003 test mode
    CAMERA_FRONT_OV5693,
    // lingjianing@CameraDrv, 2013/10/28, add for 13065 test mode
    CAMERA_FRONT_OV5648,
    //zhangkw add
    CAMERA_FRONT_OV8858,
    //hufeng add
    CAMERA_BACK_S5K3P3SX,
    CAMERA_FRONT_OTHER
} CAMERA_FRONT_DEV;

typedef enum
{
    MCP_NONE = 0,
    MCP_KMSJS000KM_B308,
    MCP_H9DP32A4JJACGR_KEM,
    MCP_KMKJS000VM_B309,
    MCP_KMNJS000ZM_B205,
    MCP_H9TP32A4GDMCPR_KDM,
    MCP_KMKJS000VM_B604,
    MCP_KMK3U000VM_B410,
	MCP_H9TP17A8JDACNR_KGM,
	MCP_TYD0HH251623RC,
	MCP_SD5C28B_16G,
	MCP_H9TP32A8JDMCPR_KGM,
	MCP_H9TP32A4GDBCPR_KGM,
	MCP_TYC0FH121597RA,
} MCP_DEV;

typedef enum
{
	ALSPS_NONE = 0,
	ALSPS_STK31XX,
	ALSPS_STK3X1X,
	ALSPS_TAOS_277X,
	ALSPS_LITE_558,
	ALSPS_GP2AAP052A,
	ALSPS_TAOS_TMG399X,
	ALSPS_TMD_27723,
	ALSPS_CM_36686,
	ALSPS_APDS_9921,
	ALSPS_APDS_9922,
}ALSPS_DEV;

typedef enum
{
	GYRO_NONE = 0,
	GYRO_MISS,
	GYRO_MMC_PG,
	GYRO_MPU6050C,
	GYRO_L3GD20,
	//zhihong.lu@BSP.sensor
	GYRO_LSM6DS3,
}GYRO_DEV;

typedef enum
{
    BKL_NONE,
    BKL_LM3580,
	BKL_LM3630,
} OPPO_BKL_DEV;

#ifdef VENDOR_EDIT
//Fuchun.Liao@Mobile.BSP.CHG 2016-01-26 add for charger device_list
typedef enum
{
	CHARGER_NONE,
    CHARGER_BQ24196,
} CHARGER_DEV;

typedef enum
{
	BMS_NONE,
    BMS_BQ27541,
    BMS_BQ27411,
} BMS_DEV;

typedef enum
{
	VOOC_NONE,
    VOOC_STM8S,
    VOOC_PIC16F,
} VOOC_DEV;
#endif	//VENDOR_EDIT


#endif
