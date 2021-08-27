/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 hi546mipiraw_sensor.c
 *  
 * Project:
 * --------
 *	 ALPS MT6735
 *	20150820: mirror flip not setting,so preview fuction can can use it ,to resolv it.
  *	20150914: 送?完了fae 要求修改setting，重新合setting.
  -------------------------------
  @DateTime:    20151021163817
  modify the winsize info,last time foroget to modity it.
 * Description:
 -------------------------------
 @DateTime:    20150925173104 move to mt6735
 modify winsizeinfo,add pip setting ,add high speed video setting-------------------------------
 @DateTime:    20151119155951
 add group hold fuction at shutter gain setting, for ae peak
 -------------------------------
 @DateTime:    20151123105620
 modify the marge and min shutter 4->6
 * ------------
 *	 Source code of Sensor driver
 *
 *	PengtaoFan

 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi546_mipi_raw_Sensor.h"

 #define LOG_INF LOG_INF_NEW

/****************************Modify Following Strings for Debug****************************/ 
#define PFX "hi546"
#define LOG_INF_NEW(format, args...)    pr_err(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF_LOD(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
#define LOG_1 LOG_INF("hi546,MIPI 2LANE\n")
#define SENSORDB LOG_INF
/****************************   Modify end    *******************************************/
static void capture_setting(kal_uint16 currefps);

#define Hi546_OTP_FUNCTION 1
#if Hi546_OTP_FUNCTION //Hi-546 OPT
void HI546_Sensor_update_wb_gain(kal_uint32 r_gain, kal_uint32 g_gain,kal_uint32 b_gain);
kal_uint16 HI546_Sensor_OTP_read(kal_uint16 otp_addr);
void HI546_Sensor_OTP_read_Continuous(kal_uint16 start_addr, kal_uint16 end_addr, kal_uint16 *data);
void HI546_Sensor_calc_wbdata(void);
#endif //Hi-546 OTP

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = HI546_SENSOR_ID,		//Sensor ID Value: 0x30C8//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0x55e2a82f,		//checksum value for Camera Auto Test

	.pre = {
		.pclk = 176000000,		//record different mode's pclk
		.linelength  = 2816,		//record different mode's linelength
		.framelength = 2049,		//record different mode's framelength
		.startx= 0,			//record different mode's startx of grabwindow
		.starty = 0,			//record different mode's starty of grabwindow
		.grabwindow_width  = 1296,	//record different mode's width of grabwindow
		.grabwindow_height = 972,	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 176000000,		//record different mode's pclk
		.linelength  = 2816,		//record different mode's linelength
		.framelength = 2049,		//record different mode's framelength
		.startx = 0,			//record different mode's startx of grabwindow
		.starty = 0,			//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,	//record different mode's width of grabwindow
		.grabwindow_height = 1944,	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap1 = {				//capture for PIP 30ps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 176000000,		//record different mode's pclk
		.linelength  = 2816,		//record different mode's linelength
		.framelength = 2049,		//record different mode's framelength
		.startx = 0,			//record different mode's startx of grabwindow
		.starty = 0,			//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,	//record different mode's width of grabwindow
		.grabwindow_height = 1944,	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 150,	
	},
	.normal_video = {
		.pclk = 176000000,		//record different mode's pclk
		.linelength  = 2816,		//record different mode's linelength
		.framelength = 2049, 		//record different mode's framelength
		.startx = 0,			//record different mode's startx of grabwindow
		.starty = 0,			//record different mode's starty of grabwindow
		.grabwindow_width  = 2592,	//record different mode's width of grabwindow
		.grabwindow_height = 1944,	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.hs_video = {
		.pclk = 176000000,		//record different mode's pclk
		.linelength  = 2816,		//record different mode's linelength
		.framelength = 520, 		//record different mode's framelength
		.startx = 0,			//record different mode's startx of grabwindow
		.starty = 0,			//record different mode's starty of grabwindow
		.grabwindow_width  = 640,	//record different mode's width of grabwindow
		.grabwindow_height = 480,	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 19,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,	
	},
	.slim_video = {
		.pclk = 176000000,		//record different mode's pclk
		.linelength  = 2816,		//record different mode's linelength
		.framelength = 2083,		//record different mode's framelength
		.startx= 0,			//record different mode's startx of grabwindow
		.starty= 0,			//record different mode's starty of grabwindow
		.grabwindow_width  = 1280,	//record different mode's width of grabwindow
		.grabwindow_height = 720,	//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.margin = 6,			//sensor framelength & shutter margin
	.min_shutter = 6,		//min shutter
	.max_frame_length = 0x7FFF,//REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num ,don't support Slow motion
	
	.cap_delay_frame = 3,		//enter capture delay frame num
	.pre_delay_frame = 3, 		//enter preview delay frame num
	.video_delay_frame = 3,		//enter video delay frame num
	.hs_video_delay_frame = 3,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,//enter slim video delay frame num
	.isp_driving_current = ISP_DRIVING_4MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//SENSOR_OUTPUT_FORMAT_RAW_Gb, //SENSOR_OUTPUT_FORMAT_RAW_Gr,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
	.i2c_addr_table = {0x40, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
	.i2c_speed = 400, // i2c read/write speed
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x200,					//current shutter
	.gain = 0x200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x40,//record current sensor's i2c write id
};


/* Sensor output window information*/

static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{
 { 2592, 1944,	  0,	0, 2592, 1944, 1296,  972,   0, 0, 1296,  972,   0, 0, 1296,  972}, // Preview
 { 2592, 1944,	  0,  	0, 2592, 1944, 2592, 1944,   0,	0, 2592, 1944, 	 0, 0, 2592, 1944}, // capture 
 { 2592, 1944,	  0,  	0, 2592, 1944, 2592, 1944,   0,	0, 2592, 1944, 	 0, 0, 2592, 1944}, // video 
 { 2592, 1944,	  16,  12, 2592, 1944, 2560, 1920,   0,	0,  640,  480, 	 0, 0,  640,  480}, // hight speed video
 { 2592, 1944,	  16, 262, 2592, 1944, 2560, 1440,   0,	0, 1280,  720, 	 0, 0, 1280,  720}, // slim
};


static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xFF00)|((get_byte>>8)&0x00FF);
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};

    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static void set_dummy()
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF);	  
	write_cmos_sensor(0x0008, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */

#if Hi546_OTP_FUNCTION //Hi-546 OPT fuction begin
static void HI546_OTP_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	kdSetI2CSpeed(400); 
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);

}
/* need to modify */
void HI546OTPSetting(void)
{
	LOG_INF("%s enter\n",__func__);
	HI546_OTP_write_cmos_sensor(0x0A02, 0x01); //Fast sleep on
	HI546_OTP_write_cmos_sensor(0x0A00, 0x00);//stand by on
	mdelay(10);
	HI546_OTP_write_cmos_sensor(0x0f02, 0x00);//pll disable
	HI546_OTP_write_cmos_sensor(0x011a, 0x01);//CP TRIM_H
	HI546_OTP_write_cmos_sensor(0x011b, 0x09);//IPGM TRIM_H
	HI546_OTP_write_cmos_sensor(0x0d04, 0x01);//Fsync(OTP busy)Output Enable 
	HI546_OTP_write_cmos_sensor(0x0d00, 0x07);//Fsync(OTP busy)Output Drivability
	HI546_OTP_write_cmos_sensor(0x003e, 0x10);//OTP r/w mode
	HI546_OTP_write_cmos_sensor(0x0a00, 0x01);//standby off

	LOG_INF("%s exit\n",__func__);
}

/* need to modify */
kal_uint16 HI546_Sensor_OTP_read(kal_uint16 otp_addr)
{
	kal_uint16 i, data;
	i = otp_addr;

    HI546_OTP_write_cmos_sensor(0x010a, (i >> 8) & 0xFF); //start address H
    HI546_OTP_write_cmos_sensor(0x010b, i & 0xFF); //start address L
    HI546_OTP_write_cmos_sensor(0x0102, 0x01); //read enable
    data = read_cmos_sensor_byte(0x0108); //OTP data read
    //HI546_OTP_write_cmos_sensor(0x003e, 0x00); //complete
    return data;
}

/* need to modify */
void HI546_Sensor_OTP_read_Continuous(kal_uint16 start_addr, kal_uint16 end_addr, kal_uint16 *data)
{
    kal_uint16 i = 0;

    HI546_OTP_write_cmos_sensor(0x010a, (start_addr >> 8) & 0xFF); //start address H
    HI546_OTP_write_cmos_sensor(0x010b, start_addr & 0xFF); //start address L
    HI546_OTP_write_cmos_sensor(0x0102, 0x01); //read enable

	for (i=0; i<=end_addr-start_addr; i++)
	{
        data[i] = read_cmos_sensor_byte(0x0108); //OTP data read
	}

}


/* need to modify */
void Hi546_Sensor_OTP_info(void)
{
	uint16_t ModuleHouseID = 0;
    uint16_t AFFF = 0;
    uint16_t Year = 0;
    uint16_t Month = 0;
    uint16_t Day = 0;
    uint16_t SensorID = 0;
    uint16_t LensID = 0;
    uint16_t VCMID = 0;
    uint16_t DriverID = 0;
    uint16_t FNoID = 0;
    uint16_t info_flag = 0;
    uint16_t infocheck = 0;
    uint16_t checksum = 0;
    uint16_t data[17]={0,};

    info_flag = HI546_Sensor_OTP_read(0x0401);

    LOG_INF("%s enter\n",__func__);
    LOG_INF("%s,info_flag = 0x%x\n",__func__,info_flag);
   
    switch(info_flag)
    {
        case 0x01 : HI546_Sensor_OTP_read_Continuous(0x0402, 0x0412, data);
            break ; 
        case 0x13 : HI546_Sensor_OTP_read_Continuous(0x0413, 0x0423, data);
            break ; 
        case 0x37 : HI546_Sensor_OTP_read_Continuous(0x0424, 0x0434, data);
            break ; 
        default : LOG_INF("HI546_Sensor: info_flag error value: %x \n ",info_flag);
            break ; 
    }

    ModuleHouseID	= data[0];
	AFFF			= data[1];
	Year			= data[2];
	Month			= data[3];
	Day				= data[4];
	SensorID		= data[5];
	LensID			= data[6];
	VCMID			= data[7];
	DriverID		= data[8];
	FNoID			= data[9];
	infocheck		= data[16];
  
	checksum = (ModuleHouseID + AFFF + Year + Month + Day + SensorID + LensID + VCMID + DriverID + FNoID ) % 255+1;
  
    if (checksum == infocheck)
    {
        LOG_INF("HI546_Sensor: Module information checksum PASS\n ");
    }
    else
    {
        LOG_INF("HI546_Sensor: Module information checksum Fail\n ");
    }

    LOG_INF("ModuleHouseID = 0x%x \n", ModuleHouseID);
    LOG_INF("AFFF = 0x%x \n", AFFF);
    LOG_INF("Year = 0x%x, Month = 0x%x, Day = 0x%x\n", Year, Month, Day);
    LOG_INF("SensorID = 0x%x \n", SensorID);
    LOG_INF("LensID = 0x%x \n", LensID);
    LOG_INF("VCMID = 0x%x \n", VCMID);
    LOG_INF("DriverID = 0x%x \n", DriverID);
    LOG_INF("FNoID = 0x%x \n", FNoID);
    LOG_INF("checksum = 0x%x \n", checksum);

    LOG_INF("%s exit\n",__func__);
}


void HI546_Sensor_update_wb_gain(kal_uint32 r_gain, kal_uint32 g_gain,kal_uint32 b_gain)
{
	kal_int16 temp;

    printk("%s,r_gain = 0x%x, b_gain = 0x%x\n", __func__,r_gain, b_gain);

    HI546_OTP_write_cmos_sensor(0x0078, g_gain >> 8); //gr_gain
    HI546_OTP_write_cmos_sensor(0x0079, g_gain & 0xFF); //gr_gain
    
    HI546_OTP_write_cmos_sensor(0x007a, g_gain >> 8); //gb_gain
    HI546_OTP_write_cmos_sensor(0x007b, g_gain & 0xFF); //gb_gain

    HI546_OTP_write_cmos_sensor(0x007c, r_gain >> 8); //r_gain
    HI546_OTP_write_cmos_sensor(0x007d, r_gain & 0xFF); //r_gain
    
    HI546_OTP_write_cmos_sensor(0x007e, b_gain >> 8); //b_gain
    HI546_OTP_write_cmos_sensor(0x007f, b_gain & 0xFF); //b_gain

    temp = read_cmos_sensor_byte(0x0a05) | 0x08;
    printk("%s, = 0x%x\n", __func__,temp);
    HI546_OTP_write_cmos_sensor(0x0a05, temp); //Digital Gain enable
}

void HI546_Sensor_calc_wbdata(void)
{
	int i = 0;
	uint16_t wbcheck = 0;
    uint16_t checksum = 0;
    uint16_t wb_flag = 0;
    uint16_t r_gain = 0x100;
    uint16_t b_gain = 0x100;
    uint16_t g_gain = 0x100;
    uint16_t rc = 0;
    uint16_t RG_ratio_unit = 0;
    uint16_t BG_ratio_unit = 0;
    uint16_t RG_ratio_light_source = 0;
    uint16_t BG_ratio_light_source = 0;
    uint16_t RG_ratio_golden = 0; 
    uint16_t BG_ratio_golden = 0;
    uint16_t RG_5100k_light_source = 0;
    uint16_t BG_5100k_light_source = 0;
    uint32_t temp = 0;
    uint16_t data[30]={0,};
  
	LOG_INF("%s enter\n",__func__);
	wb_flag = HI546_Sensor_OTP_read(0x0435);

	printk("wb_flag = 0x%x %s\n",wb_flag,__func__);

    switch(wb_flag)
    {
    case 0x01 : HI546_Sensor_OTP_read_Continuous(0x0436, 0x0453, data); rc = 1;
        break ;

    case 0x13 : HI546_Sensor_OTP_read_Continuous(0x0454, 0x0471, data); rc = 1;
        break ;

    case 0x37 : HI546_Sensor_OTP_read_Continuous(0x0472, 0x048f, data); rc = 1;
        break ;

    default:
		    LOG_INF("HI546_Sensor: wb_flag error value: 0x%x\n ",wb_flag); rc = 0; // Read Fail 
	   	break;        

    }

    if(rc) // if Flag register read pass 
    {
        RG_ratio_unit = (data[0] << 8) | (data[1] & 0xFF);
        BG_ratio_unit = (data[2] << 8) | (data[3] & 0xFF);

        RG_ratio_golden = (data[6] << 8) | (data[7] & 0xFF);
        BG_ratio_golden = (data[8] << 8) | (data[9] & 0xFF);

        RG_5100k_light_source = (data[12] << 8) | (data[13] & 0xFF);
        BG_5100k_light_source = (data[14] << 8) | (data[15] & 0xFF);
        
        
        wbcheck = data[29];
        
        for(i=0;i<29;i++)
    	{
    	    checksum += data[i];
            LOG_INF("HI546_Sensor: data[%d]is 0x%x \n",i,data[i]);
    	}
    	checksum = (checksum % 255+1); // Add 

    	if (checksum == wbcheck)
    	{
    		LOG_INF("HI546_Sensor: WB checksum PASS\n ");
            rc = 1;// Read Pass 
        }
    	else
        {
    	    LOG_INF("HI546_Sensor: WB checksum Fail\n ");
            rc = 0;// Read Fail 
        }
     }
    
        LOG_INF("HI546 RG_ratio_unit = 0x%x, BG_ratio_unit = 0x%x, RG_ratio_golden = 0x%x, BG_ratio_golden = 0x%x, wbcheck = 0x%x,checksum =0x%x \n",\
    		RG_ratio_unit, BG_ratio_unit, RG_ratio_golden, BG_ratio_golden, wbcheck,checksum);

    if(RG_5100k_light_source != 0 && 
        RG_5100k_light_source != 0xffff &&
        BG_5100k_light_source != 0 &&
        BG_5100k_light_source != 0xffff)
    {
        temp = RG_ratio_unit*1000;
        RG_ratio_light_source = temp/RG_5100k_light_source;
        temp = BG_ratio_unit*1000;
        BG_ratio_light_source = temp/BG_5100k_light_source;
        LOG_INF("HI546 RG_ratio_light = 0x%x, BG_ratio_light = 0x%x \n",RG_ratio_light_source,BG_ratio_light_source);

        RG_ratio_unit = RG_ratio_light_source;
        BG_ratio_unit = BG_ratio_light_source;
    }
    if(rc) // if Checksum is pass. 
    {
        r_gain = (0x100 * RG_ratio_golden / RG_ratio_unit);
    	b_gain = (0x100 * BG_ratio_golden / BG_ratio_unit);
    	g_gain = 0x100;

    	if (r_gain < b_gain) {
    		if(r_gain < 0x100) {
    			b_gain =0x100 *  b_gain / r_gain;
    			g_gain =0x100 *  g_gain / r_gain;
    			r_gain = 0x100;
    		}
    	} else {
    		if (b_gain < 0x100) {
    			r_gain = 0x100 * r_gain / b_gain;
    			g_gain = 0x100 * g_gain / b_gain;
    			b_gain = 0x100;
    		}
    	}
    }
    else // Set to x1 Gain 
    {
        r_gain = 0x100;
        b_gain = 0x100;
        g_gain = 0x100;
    }
    
	HI546_OTP_write_cmos_sensor(0x0a00, 0x00); //sleep On
    mdelay(10);
    HI546_OTP_write_cmos_sensor(0x003e, 0x00); //OTP mode off
    HI546_OTP_write_cmos_sensor(0x0a00, 0x01); //sleep Off

    LOG_INF("HI546 Before apply otp G_gain = 0x%x, R_gain = 0x%x, B_gain = 0x%x \n",\
	(read_cmos_sensor_byte(0x0078) << 8) | (read_cmos_sensor_byte(0x0079) & 0xFF), (read_cmos_sensor_byte(0x007c) << 8) | (read_cmos_sensor_byte(0x007d) & 0xFF), (read_cmos_sensor_byte(0x007e) << 8) | (read_cmos_sensor_byte(0x007f) & 0xFF));
    
    HI546_Sensor_update_wb_gain(r_gain, g_gain,b_gain);

    
    LOG_INF("HI546 after apply otp G_gain = 0x%x, R_gain = 0x%x, B_gain = 0x%x \n",\
	(read_cmos_sensor_byte(0x0078) << 8) | (read_cmos_sensor_byte(0x0079) & 0xFF), (read_cmos_sensor_byte(0x007c) << 8) | (read_cmos_sensor_byte(0x007d) & 0xFF), (read_cmos_sensor_byte(0x007e) << 8) | (read_cmos_sensor_byte(0x007f) & 0xFF));
}
#endif //Hi-546 OTP function finish


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable(%d) \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */






/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	//write_shutter(shutter);
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor_byte(0x0073, ((shutter & 0xFF0000) >> 16));
	write_cmos_sensor(0x0074, shutter & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	//gain = 64 = 1x real gain.
	reg_gain = gain / 4 - 16;
	//reg_gain = reg_gain & 0xFFFF;
	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	LOG_INF("set_gain %d \n", gain);
  //gain = 64 = 1x real gain.
	kal_uint16 reg_gain;
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;		 
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor_byte(0x0077, (reg_gain&0xFF)); 
	return gain;
}	/*	set_gain  */

//[TODO]ihdr_write_shutter_gain not support for hi546
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
}


//[TODO]
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror; 
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x000e,0X0000); //GR
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x000e,0X0100); //R
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x000e,0X0200); //B	
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x000e,0X0300); //GB
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/
static void sensor_init(void)
{
	write_cmos_sensor(0x0e00, 0x0102);
	write_cmos_sensor(0x0e02, 0x0102);
	write_cmos_sensor(0x0e0c, 0x0100);
	write_cmos_sensor(0x2000, 0x4031);
	write_cmos_sensor(0x2002, 0x8400);
	write_cmos_sensor(0x2004, 0x12b0);
	write_cmos_sensor(0x2006, 0xe250);
	write_cmos_sensor(0x2008, 0x12b0);
	write_cmos_sensor(0x200a, 0xe26e);
	write_cmos_sensor(0x200c, 0x93d2);
	write_cmos_sensor(0x200e, 0x00bd);
	write_cmos_sensor(0x2010, 0x247b);
	write_cmos_sensor(0x2012, 0x0900);
	write_cmos_sensor(0x2014, 0x7312);
	write_cmos_sensor(0x2016, 0x43d2);
	write_cmos_sensor(0x2018, 0x00bd);
	write_cmos_sensor(0x201a, 0x12b0);
	write_cmos_sensor(0x201c, 0xe556);
	write_cmos_sensor(0x201e, 0x12b0);
	write_cmos_sensor(0x2020, 0xe594);
	write_cmos_sensor(0x2022, 0x12b0);
	write_cmos_sensor(0x2024, 0xe7aa);
	write_cmos_sensor(0x2026, 0x12b0);
	write_cmos_sensor(0x2028, 0xe288);
	write_cmos_sensor(0x202a, 0x12b0);
	write_cmos_sensor(0x202c, 0xe5ae);
	write_cmos_sensor(0x202e, 0x12b0);
	write_cmos_sensor(0x2030, 0xe70e);
	write_cmos_sensor(0x2032, 0xb3a2);
	write_cmos_sensor(0x2034, 0x0a84);
	write_cmos_sensor(0x2036, 0x2402);
	write_cmos_sensor(0x2038, 0x12b0);
	write_cmos_sensor(0x203a, 0xe8fa);
	write_cmos_sensor(0x203c, 0x43a2);
	write_cmos_sensor(0x203e, 0x7320);
	write_cmos_sensor(0x2040, 0x4382);
	write_cmos_sensor(0x2042, 0x7322);
	write_cmos_sensor(0x2044, 0x4392);
	write_cmos_sensor(0x2046, 0x7326);
	write_cmos_sensor(0x2048, 0x12b0);
	write_cmos_sensor(0x204a, 0xfa6e);
	write_cmos_sensor(0x204c, 0x12b0);
	write_cmos_sensor(0x204e, 0xe3f6);
	write_cmos_sensor(0x2050, 0x12b0);
	write_cmos_sensor(0x2052, 0xe45e);
	write_cmos_sensor(0x2054, 0x12b0);
	write_cmos_sensor(0x2056, 0xe4a4);
	write_cmos_sensor(0x2058, 0x12b0);
	write_cmos_sensor(0x205a, 0xe4f6);
	write_cmos_sensor(0x205c, 0x12b0);
	write_cmos_sensor(0x205e, 0xe5fc);
	write_cmos_sensor(0x2060, 0x12b0);
	write_cmos_sensor(0x2062, 0xfa3c);
	write_cmos_sensor(0x2064, 0x12b0);
	write_cmos_sensor(0x2066, 0xe654);
	write_cmos_sensor(0x2068, 0x4392);
	write_cmos_sensor(0x206a, 0x731c);
	write_cmos_sensor(0x206c, 0x12b0);
	write_cmos_sensor(0x206e, 0xe984);
	write_cmos_sensor(0x2070, 0x12b0);
	write_cmos_sensor(0x2072, 0xe512);
	write_cmos_sensor(0x2074, 0x93c2);
	write_cmos_sensor(0x2076, 0x8294);
	write_cmos_sensor(0x2078, 0x2009);
	write_cmos_sensor(0x207a, 0x0b00);
	write_cmos_sensor(0x207c, 0x7302);
	write_cmos_sensor(0x207e, 0x0258);
	write_cmos_sensor(0x2080, 0x4382);
	write_cmos_sensor(0x2082, 0x7902);
	write_cmos_sensor(0x2084, 0x0900);
	write_cmos_sensor(0x2086, 0x7308);
	write_cmos_sensor(0x2088, 0x12b0);
	write_cmos_sensor(0x208a, 0xfa6e);
	write_cmos_sensor(0x208c, 0x12b0);
	write_cmos_sensor(0x208e, 0xfa0e);
	write_cmos_sensor(0x2090, 0x12b0);
	write_cmos_sensor(0x2092, 0xe32e);
	write_cmos_sensor(0x2094, 0x0900);
	write_cmos_sensor(0x2096, 0x7328);
	write_cmos_sensor(0x2098, 0x4292);
	write_cmos_sensor(0x209a, 0x8292);
	write_cmos_sensor(0x209c, 0x7114);
	write_cmos_sensor(0x209e, 0x421f);
	write_cmos_sensor(0x20a0, 0x7316);
	write_cmos_sensor(0x20a2, 0xc312);
	write_cmos_sensor(0x20a4, 0x100f);
	write_cmos_sensor(0x20a6, 0x821f);
	write_cmos_sensor(0x20a8, 0x8298);
	write_cmos_sensor(0x20aa, 0x4f82);
	write_cmos_sensor(0x20ac, 0x7334);
	write_cmos_sensor(0x20ae, 0x0f00);
	write_cmos_sensor(0x20b0, 0x7302);
	write_cmos_sensor(0x20b2, 0x12b0);
	write_cmos_sensor(0x20b4, 0xe5ea);
	write_cmos_sensor(0x20b6, 0x9392);
	write_cmos_sensor(0x20b8, 0x7114);
	write_cmos_sensor(0x20ba, 0x2011);
	write_cmos_sensor(0x20bc, 0x0b00);
	write_cmos_sensor(0x20be, 0x7302);
	write_cmos_sensor(0x20c0, 0x0258);
	write_cmos_sensor(0x20c2, 0x4382);
	write_cmos_sensor(0x20c4, 0x7902);
	write_cmos_sensor(0x20c6, 0x0800);
	write_cmos_sensor(0x20c8, 0x7118);
	write_cmos_sensor(0x20ca, 0x12b0);
	write_cmos_sensor(0x20cc, 0xea60);
	write_cmos_sensor(0x20ce, 0x0900);
	write_cmos_sensor(0x20d0, 0x7112);
	write_cmos_sensor(0x20d2, 0x12b0);
	write_cmos_sensor(0x20d4, 0xe3a4);
	write_cmos_sensor(0x20d6, 0x0b00);
	write_cmos_sensor(0x20d8, 0x7302);
	write_cmos_sensor(0x20da, 0x0036);
	write_cmos_sensor(0x20dc, 0x3fec);
	write_cmos_sensor(0x20de, 0x0b00);
	write_cmos_sensor(0x20e0, 0x7302);
	write_cmos_sensor(0x20e2, 0x0036);
	write_cmos_sensor(0x20e4, 0x4392);
	write_cmos_sensor(0x20e6, 0x7902);
	write_cmos_sensor(0x20e8, 0x4292);
	write_cmos_sensor(0x20ea, 0x7100);
	write_cmos_sensor(0x20ec, 0x82be);
	write_cmos_sensor(0x20ee, 0x12b0);
	write_cmos_sensor(0x20f0, 0xe67c);
	write_cmos_sensor(0x20f2, 0x12b0);
	write_cmos_sensor(0x20f4, 0xe9c6);
	write_cmos_sensor(0x20f6, 0x12b0);
	write_cmos_sensor(0x20f8, 0xea0c);
	write_cmos_sensor(0x20fa, 0x12b0);
	write_cmos_sensor(0x20fc, 0xe3a4);
	write_cmos_sensor(0x20fe, 0x930f);
	write_cmos_sensor(0x2100, 0x27da);
	write_cmos_sensor(0x2102, 0x43c2);
	write_cmos_sensor(0x2104, 0x8294);
	write_cmos_sensor(0x2106, 0x3fb4);
	write_cmos_sensor(0x2108, 0x12b0);
	write_cmos_sensor(0x210a, 0xe52e);
	write_cmos_sensor(0x210c, 0x3f82);
	write_cmos_sensor(0x210e, 0x4030);
	write_cmos_sensor(0x2110, 0xfb06);
	write_cmos_sensor(0x2112, 0x7400);
	write_cmos_sensor(0x2114, 0x178f);
	write_cmos_sensor(0x2116, 0x0242);
	write_cmos_sensor(0x2118, 0x04db);
	write_cmos_sensor(0x211a, 0x025b);
	write_cmos_sensor(0x211c, 0x7006);
	write_cmos_sensor(0x211e, 0x1742);
	write_cmos_sensor(0x2120, 0x005a);
	write_cmos_sensor(0x2122, 0x025a);
	write_cmos_sensor(0x2124, 0x700f);
	write_cmos_sensor(0x2126, 0x0fd0);
	write_cmos_sensor(0x2128, 0x0047);
	write_cmos_sensor(0x212a, 0x2118);
	write_cmos_sensor(0x212c, 0x0041);
	write_cmos_sensor(0x212e, 0x0245);
	write_cmos_sensor(0x2130, 0x0006);
	write_cmos_sensor(0x2132, 0x0181);
	write_cmos_sensor(0x2134, 0x144d);
	write_cmos_sensor(0x2136, 0x204f);
	write_cmos_sensor(0x2138, 0x7001);
	write_cmos_sensor(0x213a, 0x0fcb);
	write_cmos_sensor(0x213c, 0x00cc);
	write_cmos_sensor(0x213e, 0x7000);
	write_cmos_sensor(0x2140, 0x144d);
	write_cmos_sensor(0x2142, 0x0024);
	write_cmos_sensor(0x2144, 0x040e);
	write_cmos_sensor(0x2146, 0x299f);
	write_cmos_sensor(0x2148, 0x279c);
	write_cmos_sensor(0x214a, 0x0025);
	write_cmos_sensor(0x214c, 0x2063);
	write_cmos_sensor(0x214e, 0x0e5f);
	write_cmos_sensor(0x2150, 0x195f);
	write_cmos_sensor(0x2152, 0x207b);
	write_cmos_sensor(0x2154, 0x20e4);
	write_cmos_sensor(0x2156, 0x5000);
	write_cmos_sensor(0x2158, 0x0005);
	write_cmos_sensor(0x215a, 0x0000);
	write_cmos_sensor(0x215c, 0x01dd);
	write_cmos_sensor(0x215e, 0x025c);
	write_cmos_sensor(0x2160, 0x00c0);
	write_cmos_sensor(0x2162, 0x0005);
	write_cmos_sensor(0x2164, 0x0006);
	write_cmos_sensor(0x2166, 0x0adb);
	write_cmos_sensor(0x2168, 0x025b);
	write_cmos_sensor(0x216a, 0x025a);
	write_cmos_sensor(0x216c, 0x025a);
	write_cmos_sensor(0x216e, 0x7001);
	write_cmos_sensor(0x2170, 0x2f9c);
	write_cmos_sensor(0x2172, 0x20e8);
	write_cmos_sensor(0x2174, 0x2063);
	write_cmos_sensor(0x2176, 0x707b);
	write_cmos_sensor(0x2178, 0x0fdf);
	write_cmos_sensor(0x217a, 0x81b8);
	write_cmos_sensor(0x217c, 0x5040);
	write_cmos_sensor(0x217e, 0x0025);
	write_cmos_sensor(0x2180, 0x5060);
	write_cmos_sensor(0x2182, 0x3143);
	write_cmos_sensor(0x2184, 0x5081);
	write_cmos_sensor(0x2186, 0x025e);
	write_cmos_sensor(0x2188, 0x7800);
	write_cmos_sensor(0x218a, 0x7400);
	write_cmos_sensor(0x218c, 0x178f);
	write_cmos_sensor(0x218e, 0x0242);
	write_cmos_sensor(0x2190, 0x04db);
	write_cmos_sensor(0x2192, 0x025b);
	write_cmos_sensor(0x2194, 0x7006);
	write_cmos_sensor(0x2196, 0x1742);
	write_cmos_sensor(0x2198, 0x005a);
	write_cmos_sensor(0x219a, 0x025a);
	write_cmos_sensor(0x219c, 0x0dd0);
	write_cmos_sensor(0x219e, 0x0047);
	write_cmos_sensor(0x21a0, 0x2118);
	write_cmos_sensor(0x21a2, 0x0041);
	write_cmos_sensor(0x21a4, 0x0245);
	write_cmos_sensor(0x21a6, 0x0006);
	write_cmos_sensor(0x21a8, 0x0181);
	write_cmos_sensor(0x21aa, 0x144d);
	write_cmos_sensor(0x21ac, 0x204f);
	write_cmos_sensor(0x21ae, 0x7001);
	write_cmos_sensor(0x21b0, 0x0fcb);
	write_cmos_sensor(0x21b2, 0x00cc);
	write_cmos_sensor(0x21b4, 0x7000);
	write_cmos_sensor(0x21b6, 0x144d);
	write_cmos_sensor(0x21b8, 0x0024);
	write_cmos_sensor(0x21ba, 0x040e);
	write_cmos_sensor(0x21bc, 0x299f);
	write_cmos_sensor(0x21be, 0x279c);
	write_cmos_sensor(0x21c0, 0x0025);
	write_cmos_sensor(0x21c2, 0x2063);
	write_cmos_sensor(0x21c4, 0x0e5f);
	write_cmos_sensor(0x21c6, 0x195f);
	write_cmos_sensor(0x21c8, 0x207b);
	write_cmos_sensor(0x21ca, 0x20e4);
	write_cmos_sensor(0x21cc, 0x50a0);
	write_cmos_sensor(0x21ce, 0x0005);
	write_cmos_sensor(0x21d0, 0x0000);
	write_cmos_sensor(0x21d2, 0x01dd);
	write_cmos_sensor(0x21d4, 0x025c);
	write_cmos_sensor(0x21d6, 0x00c0);
	write_cmos_sensor(0x21d8, 0x0005);
	write_cmos_sensor(0x21da, 0x0006);
	write_cmos_sensor(0x21dc, 0x0adb);
	write_cmos_sensor(0x21de, 0x025b);
	write_cmos_sensor(0x21e0, 0x025a);
	write_cmos_sensor(0x21e2, 0x025a);
	write_cmos_sensor(0x21e4, 0x7001);
	write_cmos_sensor(0x21e6, 0x2f9c);
	write_cmos_sensor(0x21e8, 0x20e8);
	write_cmos_sensor(0x21ea, 0x2063);
	write_cmos_sensor(0x21ec, 0x707b);
	write_cmos_sensor(0x21ee, 0x0fdf);
	write_cmos_sensor(0x21f0, 0x86b8);
	write_cmos_sensor(0x21f2, 0x50e0);
	write_cmos_sensor(0x21f4, 0x0025);
	write_cmos_sensor(0x21f6, 0x5100);
	write_cmos_sensor(0x21f8, 0x3143);
	write_cmos_sensor(0x21fa, 0x5121);
	write_cmos_sensor(0x21fc, 0x7800);
	write_cmos_sensor(0x21fe, 0x3140);
	write_cmos_sensor(0x2200, 0x01c4);
	write_cmos_sensor(0x2202, 0x01c1);
	write_cmos_sensor(0x2204, 0x01c0);
	write_cmos_sensor(0x2206, 0x01c4);
	write_cmos_sensor(0x2208, 0x2700);
	write_cmos_sensor(0x220a, 0x3d40);
	write_cmos_sensor(0x220c, 0x7800);
	write_cmos_sensor(0x220e, 0x40b2);
	write_cmos_sensor(0x2210, 0x0064);
	write_cmos_sensor(0x2212, 0x829c);
	write_cmos_sensor(0x2214, 0x40b2);
	write_cmos_sensor(0x2216, 0x0258);
	write_cmos_sensor(0x2218, 0x829e);
	write_cmos_sensor(0x221a, 0x93c2);
	write_cmos_sensor(0x221c, 0x00cc);
	write_cmos_sensor(0x221e, 0x2407);
	write_cmos_sensor(0x2220, 0x40b2);
	write_cmos_sensor(0x2222, 0x02e5);
	write_cmos_sensor(0x2224, 0x82a0);
	write_cmos_sensor(0x2226, 0x40b2);
	write_cmos_sensor(0x2228, 0x04da);
	write_cmos_sensor(0x222a, 0x82a2);
	write_cmos_sensor(0x222c, 0x4130);
	write_cmos_sensor(0x222e, 0x40b2);
	write_cmos_sensor(0x2230, 0x0324);
	write_cmos_sensor(0x2232, 0x82a0);
	write_cmos_sensor(0x2234, 0x40b2);
	write_cmos_sensor(0x2236, 0x051c);
	write_cmos_sensor(0x2238, 0x82a2);
	write_cmos_sensor(0x223a, 0x4130);
	write_cmos_sensor(0x223c, 0xb3e2);
	write_cmos_sensor(0x223e, 0x00dd);
	write_cmos_sensor(0x2240, 0x240b);
	write_cmos_sensor(0x2242, 0x93c2);
	write_cmos_sensor(0x2244, 0x00cc);
	write_cmos_sensor(0x2246, 0x2404);
	write_cmos_sensor(0x2248, 0x4292);
	write_cmos_sensor(0x224a, 0x806a);
	write_cmos_sensor(0x224c, 0x82c0);
	write_cmos_sensor(0x224e, 0x4130);
	write_cmos_sensor(0x2250, 0x4292);
	write_cmos_sensor(0x2252, 0x8068);
	write_cmos_sensor(0x2254, 0x82c0);
	write_cmos_sensor(0x2256, 0x4130);
	write_cmos_sensor(0x2258, 0x93c2);
	write_cmos_sensor(0x225a, 0x00cc);
	write_cmos_sensor(0x225c, 0x2404);
	write_cmos_sensor(0x225e, 0x40b2);
	write_cmos_sensor(0x2260, 0xf98a);
	write_cmos_sensor(0x2262, 0x82c0);
	write_cmos_sensor(0x2264, 0x4130);
	write_cmos_sensor(0x2266, 0x40b2);
	write_cmos_sensor(0x2268, 0xf912);
	write_cmos_sensor(0x226a, 0x82c0);
	write_cmos_sensor(0x226c, 0x4130);
	write_cmos_sensor(0x226e, 0x12b0);
	write_cmos_sensor(0x2270, 0xedd8);
	write_cmos_sensor(0x2272, 0x12b0);
	write_cmos_sensor(0x2274, 0xee2a);
	write_cmos_sensor(0x2276, 0x12b0);
	write_cmos_sensor(0x2278, 0xee7a);
	write_cmos_sensor(0x227a, 0x12b0);
	write_cmos_sensor(0x227c, 0xf1fa);
	write_cmos_sensor(0x227e, 0x12b0);
	write_cmos_sensor(0x2280, 0xeeb8);
	write_cmos_sensor(0x2282, 0x12b0);
	write_cmos_sensor(0x2284, 0xeed4);
	write_cmos_sensor(0x2286, 0x40b2);
	write_cmos_sensor(0x2288, 0x003c);
	write_cmos_sensor(0x228a, 0x7814);
	write_cmos_sensor(0x228c, 0x93c2);
	write_cmos_sensor(0x228e, 0x00cd);
	write_cmos_sensor(0x2290, 0x2413);
	write_cmos_sensor(0x2292, 0x40b2);
	write_cmos_sensor(0x2294, 0x0c01);
	write_cmos_sensor(0x2296, 0x7500);
	write_cmos_sensor(0x2298, 0x40b2);
	write_cmos_sensor(0x229a, 0x080b);
	write_cmos_sensor(0x229c, 0x7502);
	write_cmos_sensor(0x229e, 0x40b2);
	write_cmos_sensor(0x22a0, 0x080f);
	write_cmos_sensor(0x22a2, 0x7504);
	write_cmos_sensor(0x22a4, 0x40b2);
	write_cmos_sensor(0x22a6, 0x500b);
	write_cmos_sensor(0x22a8, 0x7506);
	write_cmos_sensor(0x22aa, 0x40b2);
	write_cmos_sensor(0x22ac, 0x0802);
	write_cmos_sensor(0x22ae, 0x7508);
	write_cmos_sensor(0x22b0, 0x40b2);
	write_cmos_sensor(0x22b2, 0x0800);
	write_cmos_sensor(0x22b4, 0x750a);
	write_cmos_sensor(0x22b6, 0x3c1e);
	write_cmos_sensor(0x22b8, 0x40b2);
	write_cmos_sensor(0x22ba, 0x0c01);
	write_cmos_sensor(0x22bc, 0x7500);
	write_cmos_sensor(0x22be, 0x40b2);
	write_cmos_sensor(0x22c0, 0x080b);
	write_cmos_sensor(0x22c2, 0x7502);
	write_cmos_sensor(0x22c4, 0x40b2);
	write_cmos_sensor(0x22c6, 0x080f);
	write_cmos_sensor(0x22c8, 0x7504);
	write_cmos_sensor(0x22ca, 0x40b2);
	write_cmos_sensor(0x22cc, 0x300b);
	write_cmos_sensor(0x22ce, 0x7506);
	write_cmos_sensor(0x22d0, 0x40b2);
	write_cmos_sensor(0x22d2, 0x0809);
	write_cmos_sensor(0x22d4, 0x7508);
	write_cmos_sensor(0x22d6, 0x40b2);
	write_cmos_sensor(0x22d8, 0x080d);
	write_cmos_sensor(0x22da, 0x750a);
	write_cmos_sensor(0x22dc, 0x40b2);
	write_cmos_sensor(0x22de, 0x5809);
	write_cmos_sensor(0x22e0, 0x750c);
	write_cmos_sensor(0x22e2, 0x40b2);
	write_cmos_sensor(0x22e4, 0x080b);
	write_cmos_sensor(0x22e6, 0x750e);
	write_cmos_sensor(0x22e8, 0x40b2);
	write_cmos_sensor(0x22ea, 0x0802);
	write_cmos_sensor(0x22ec, 0x7510);
	write_cmos_sensor(0x22ee, 0x40b2);
	write_cmos_sensor(0x22f0, 0x0800);
	write_cmos_sensor(0x22f2, 0x7512);
	write_cmos_sensor(0x22f4, 0x12b0);
	write_cmos_sensor(0x22f6, 0xef48);
	write_cmos_sensor(0x22f8, 0x12b0);
	write_cmos_sensor(0x22fa, 0xf0e0);
	write_cmos_sensor(0x22fc, 0x12b0);
	write_cmos_sensor(0x22fe, 0xf1aa);
	write_cmos_sensor(0x2300, 0xd392);
	write_cmos_sensor(0x2302, 0x7102);
	write_cmos_sensor(0x2304, 0x4130);
	write_cmos_sensor(0x2306, 0xdf02);
	write_cmos_sensor(0x2308, 0x3ffe);
	write_cmos_sensor(0x230a, 0x430e);
	write_cmos_sensor(0x230c, 0x930a);
	write_cmos_sensor(0x230e, 0x2407);
	write_cmos_sensor(0x2310, 0xc312);
	write_cmos_sensor(0x2312, 0x100c);
	write_cmos_sensor(0x2314, 0x2801);
	write_cmos_sensor(0x2316, 0x5a0e);
	write_cmos_sensor(0x2318, 0x5a0a);
	write_cmos_sensor(0x231a, 0x930c);
	write_cmos_sensor(0x231c, 0x23f7);
	write_cmos_sensor(0x231e, 0x4130);
	write_cmos_sensor(0x2320, 0x430e);
	write_cmos_sensor(0x2322, 0x430f);
	write_cmos_sensor(0x2324, 0x3c08);
	write_cmos_sensor(0x2326, 0xc312);
	write_cmos_sensor(0x2328, 0x100d);
	write_cmos_sensor(0x232a, 0x100c);
	write_cmos_sensor(0x232c, 0x2802);
	write_cmos_sensor(0x232e, 0x5a0e);
	write_cmos_sensor(0x2330, 0x6b0f);
	write_cmos_sensor(0x2332, 0x5a0a);
	write_cmos_sensor(0x2334, 0x6b0b);
	write_cmos_sensor(0x2336, 0x930c);
	write_cmos_sensor(0x2338, 0x23f6);
	write_cmos_sensor(0x233a, 0x930d);
	write_cmos_sensor(0x233c, 0x23f4);
	write_cmos_sensor(0x233e, 0x4130);
	write_cmos_sensor(0x2340, 0xee0e);
	write_cmos_sensor(0x2342, 0x403b);
	write_cmos_sensor(0x2344, 0x0011);
	write_cmos_sensor(0x2346, 0x3c05);
	write_cmos_sensor(0x2348, 0x100d);
	write_cmos_sensor(0x234a, 0x6e0e);
	write_cmos_sensor(0x234c, 0x9a0e);
	write_cmos_sensor(0x234e, 0x2801);
	write_cmos_sensor(0x2350, 0x8a0e);
	write_cmos_sensor(0x2352, 0x6c0c);
	write_cmos_sensor(0x2354, 0x6d0d);
	write_cmos_sensor(0x2356, 0x831b);
	write_cmos_sensor(0x2358, 0x23f7);
	write_cmos_sensor(0x235a, 0x4130);
	write_cmos_sensor(0x235c, 0xef0f);
	write_cmos_sensor(0x235e, 0xee0e);
	write_cmos_sensor(0x2360, 0x4039);
	write_cmos_sensor(0x2362, 0x0021);
	write_cmos_sensor(0x2364, 0x3c0a);
	write_cmos_sensor(0x2366, 0x1008);
	write_cmos_sensor(0x2368, 0x6e0e);
	write_cmos_sensor(0x236a, 0x6f0f);
	write_cmos_sensor(0x236c, 0x9b0f);
	write_cmos_sensor(0x236e, 0x2805);
	write_cmos_sensor(0x2370, 0x2002);
	write_cmos_sensor(0x2372, 0x9a0e);
	write_cmos_sensor(0x2374, 0x2802);
	write_cmos_sensor(0x2376, 0x8a0e);
	write_cmos_sensor(0x2378, 0x7b0f);
	write_cmos_sensor(0x237a, 0x6c0c);
	write_cmos_sensor(0x237c, 0x6d0d);
	write_cmos_sensor(0x237e, 0x6808);
	write_cmos_sensor(0x2380, 0x8319);
	write_cmos_sensor(0x2382, 0x23f1);
	write_cmos_sensor(0x2384, 0x4130);
	write_cmos_sensor(0x2386, 0x7e00);
	write_cmos_sensor(0x27fe, 0xe02a);
	write_cmos_sensor(0x3000, 0xc4f8);
	write_cmos_sensor(0x3002, 0x61fd);
	write_cmos_sensor(0x3004, 0xc1c4);
	write_cmos_sensor(0x3006, 0x2051);
	write_cmos_sensor(0x3008, 0xc4b0);
	write_cmos_sensor(0x300a, 0x51cd);
	write_cmos_sensor(0x300c, 0x8413);
	write_cmos_sensor(0x300e, 0x503c);
	write_cmos_sensor(0x3010, 0x0440);
	write_cmos_sensor(0x3012, 0x403c);
	write_cmos_sensor(0x3014, 0xc4f8);
	write_cmos_sensor(0x3016, 0x61fd);
	write_cmos_sensor(0x3018, 0xc1c4);
	write_cmos_sensor(0x301a, 0x2051);
	write_cmos_sensor(0x301c, 0xc4b0);
	write_cmos_sensor(0x301e, 0x51cd);
	write_cmos_sensor(0x3020, 0x8413);
	write_cmos_sensor(0x3022, 0x503c);
	write_cmos_sensor(0x3024, 0x0440);
	write_cmos_sensor(0x3026, 0x003c);

	write_cmos_sensor(0x0b00, 0x0000);
	write_cmos_sensor(0x0b02, 0x0045);
	write_cmos_sensor(0x0b04, 0xb405);
	write_cmos_sensor(0x0b06, 0xe403);
	write_cmos_sensor(0x0b08, 0xa081);
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0b0c, 0xf814);
	write_cmos_sensor(0x0b0e, 0xc618);
	write_cmos_sensor(0x0b10, 0xd028);
	write_cmos_sensor(0x0b12, 0x002c);
	write_cmos_sensor(0x0b14, 0x1468);
	write_cmos_sensor(0x0b16, 0x0109);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7067);
	write_cmos_sensor(0x0954, 0x0009);
	write_cmos_sensor(0x0956, 0x0000);
	write_cmos_sensor(0x0958, 0xba80);
	write_cmos_sensor(0x095a, 0x0000);
	write_cmos_sensor(0x0c00, 0x1110);
	write_cmos_sensor(0x0c02, 0x0011);
	write_cmos_sensor(0x0c04, 0x0000);
	write_cmos_sensor(0x0c06, 0x0200);
	write_cmos_sensor(0x0c10, 0x0040);
	write_cmos_sensor(0x0c12, 0x0040);
	write_cmos_sensor(0x0c14, 0x0040);
	write_cmos_sensor(0x0c16, 0x0040);
	write_cmos_sensor(0x0a10, 0x4000);
	write_cmos_sensor(0x006c, 0x0200);
	write_cmos_sensor(0x005e, 0x0200);
	write_cmos_sensor(0x000e, 0x0100);
	write_cmos_sensor(0x0e0a, 0x0001);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100);
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0202);
	write_cmos_sensor(0x0012, 0x000e);
	write_cmos_sensor(0x0018, 0x0a31);
	write_cmos_sensor(0x0022, 0x0004);
	write_cmos_sensor(0x0028, 0x0013);
	write_cmos_sensor(0x0024, 0x002c);
	write_cmos_sensor(0x002a, 0x0031);
	write_cmos_sensor(0x0026, 0x0034);
	write_cmos_sensor(0x002c, 0x07cb);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0006, 0x07bc);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0a20);
	write_cmos_sensor(0x0a14, 0x0798);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0074, 0x07ba);
	write_cmos_sensor(0x0070, 0x0407);
	write_cmos_sensor(0x0002, 0x0000);
	write_cmos_sensor(0x0a02, 0x0100);
	write_cmos_sensor(0x0a24, 0x0100);
	write_cmos_sensor(0x0046, 0x0000);
	write_cmos_sensor(0x0076, 0x0000);
	write_cmos_sensor(0x0122, 0x0300);
	write_cmos_sensor(0x015a, 0xff08);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x005c, 0x0100);
	write_cmos_sensor(0x0a1a, 0x0800);
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc10c);
	write_cmos_sensor(0x0916, 0x061d);
	write_cmos_sensor(0x091c, 0x0e09);
	write_cmos_sensor(0x0918, 0x0307);
	write_cmos_sensor(0x091a, 0x0c0c);
	write_cmos_sensor(0x091e, 0x0a00);
	write_cmos_sensor(0x090c, 0x0fdc);
	write_cmos_sensor(0x090e, 0x002d);
	write_cmos_sensor(0x0a04, 0x014a);
	write_cmos_sensor(0x0914, 0xc10a);
	write_cmos_sensor(0x0916, 0x071f);
	write_cmos_sensor(0x0918, 0x0408);
	write_cmos_sensor(0x091a, 0x0c0d);
	write_cmos_sensor(0x091c, 0x0f09);
}

static void preview_setting(void)
{
	//Sensor Information////////////////////////////
	//Sensor	  : Hi-546
	//Date		  : 2016-11-02
	//Customer        : MTK_validation
	//Image size	  : 1296x972
	//MCLK		  : 24MHz
	//MIPI speed(Mbps): 440Mbps x 2Lane
	//Frame Length	  : 2049
	//Line Length 	  : 2816
	//Max Fps 	  : 30.5fps
	//Pixel order 	  : Green 1st (=GB)
	//X/Y-flip	  : X-flip
	//BLC offset	  : 64code
	////////////////////////////////////////////////
	write_cmos_sensor(0x0a00, 0x0000);
	write_cmos_sensor(0x0b0a, 0x8255);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7167);
	write_cmos_sensor(0x0958, 0xba80);
	write_cmos_sensor(0x0c00, 0x1110);
	write_cmos_sensor(0x0c02, 0x0011);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x000c, 0x0122);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0404);
	write_cmos_sensor(0x0012, 0x000c);
	write_cmos_sensor(0x0018, 0x0a33);
	write_cmos_sensor(0x0024, 0x0026);
	write_cmos_sensor(0x002a, 0x002f);
	write_cmos_sensor(0x0026, 0x0034);
	write_cmos_sensor(0x002c, 0x07cb);
	write_cmos_sensor(0x002e, 0x3311);
	write_cmos_sensor(0x0030, 0x3311);
	write_cmos_sensor(0x0032, 0x3311);
	write_cmos_sensor(0x0006, 0x0801);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0510);
	write_cmos_sensor(0x0a14, 0x03cc);
	write_cmos_sensor(0x0074, 0x07ff);
	write_cmos_sensor(0x0070, 0x0407);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x090e, 0x0010);
	write_cmos_sensor(0x090c, 0x09c0);
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc106);
	write_cmos_sensor(0x0916, 0x040e);
	write_cmos_sensor(0x0918, 0x0304);
	write_cmos_sensor(0x091a, 0x0709);
	write_cmos_sensor(0x091c, 0x0e06);
	write_cmos_sensor(0x091e, 0x0300);
	write_cmos_sensor(0x0a00, 0x0100);
}
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	//Sensor Information////////////////////////////
	//Sensor	  : Hi-546
	//Date		  : 2016-11-02
	//Customer        : MTK_validation
	//Image size	  : 2592x1944
	//MCLK		  : 24MHz
	//MIPI speed(Mbps): 880Mbps x 2Lane
	//Frame Length	  : 2049
	//Line Length 	  : 2816
	//Max Fps 	  : 30.5fps
	//Pixel order 	  : Green 1st (=GB)
	//X/Y-flip	  : X-flip
	//BLC offset	  : 64code
	////////////////////////////////////////////////
	write_cmos_sensor(0x0a00, 0x0000);
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7067);
	write_cmos_sensor(0x0958, 0xba80);
	write_cmos_sensor(0x0c00, 0x1110);
	write_cmos_sensor(0x0c02, 0x0011);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0202);
	write_cmos_sensor(0x0012, 0x000e);
	write_cmos_sensor(0x0018, 0x0a31);
	write_cmos_sensor(0x0024, 0x002c);
	write_cmos_sensor(0x002a, 0x0031);
	write_cmos_sensor(0x0026, 0x0034);
	write_cmos_sensor(0x002c, 0x07cb);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x1111);
	write_cmos_sensor(0x0006, 0x0801);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0a20);
	write_cmos_sensor(0x0a14, 0x0798);
	write_cmos_sensor(0x0074, 0x07ff);
	write_cmos_sensor(0x0070, 0x0407);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x014a);
	write_cmos_sensor(0x090c, 0x0fdc);
	write_cmos_sensor(0x090e, 0x002d);
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc10a);
	write_cmos_sensor(0x0916, 0x071f);
	write_cmos_sensor(0x0918, 0x0408);
	write_cmos_sensor(0x091a, 0x0c0d);
	write_cmos_sensor(0x091c, 0x0f09);
	write_cmos_sensor(0x091e, 0x0a00);
	write_cmos_sensor(0x0a00, 0x0100);						   
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	capture_setting(currefps);
}

static void hs_video_setting()
{
	LOG_INF("E\n hs_video_setting");
	//Sensor Information////////////////////////////
	//Sensor	  : Hi-546
	//Date		  : 2016-11-02
	//Customer        : MTK_validation
	//Image size	  : 640x480
	//MCLK		  : 24MHz
	//MIPI speed(Mbps): 220Mbps x 2Lane
	//Frame Length	  : 520
	//Line Length 	  : 2816
	//Max Fps 	  : 120.19fps
	//Pixel order 	  : Green 1st (=GB)
	//X/Y-flip	  : X-flip
	//BLC offset	  : 64code
	////////////////////////////////////////////////
	write_cmos_sensor(0x0a00, 0x0000);
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7267);
	write_cmos_sensor(0x0958, 0xba80);
	write_cmos_sensor(0x0c00, 0x1110);
	write_cmos_sensor(0x0c02, 0x0011);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0208);
	write_cmos_sensor(0x0012, 0x0018);
	write_cmos_sensor(0x0018, 0x0a27);
	write_cmos_sensor(0x0024, 0x0032);
	write_cmos_sensor(0x002a, 0x0037);
	write_cmos_sensor(0x0026, 0x0040);
	write_cmos_sensor(0x002c, 0x07bf);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x7711);
	write_cmos_sensor(0x0006, 0x0208);
	write_cmos_sensor(0x0a22, 0x0100);
	write_cmos_sensor(0x0a12, 0x0280);
	write_cmos_sensor(0x0a14, 0x01e0);
	write_cmos_sensor(0x0074, 0x0206);
	write_cmos_sensor(0x0070, 0x0407);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x090c, 0x0270);
	write_cmos_sensor(0x090e, 0x000c);
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc103);
	write_cmos_sensor(0x0916, 0x0207);
	write_cmos_sensor(0x0918, 0x0302);
	write_cmos_sensor(0x091a, 0x0406);
	write_cmos_sensor(0x091c, 0x0903);
	write_cmos_sensor(0x091e, 0x0200);
	write_cmos_sensor(0x0a00, 0x0100); //stream on
}


static void slim_video_setting(void)
{
	LOG_INF("E\n slim_video_setting");
	//Sensor Information////////////////////////////
	//Sensor	  : Hi-546
	//Date		  : 2016-11-02
	//Customer        : MTK_validation
	//Image size	  : 1280x720
	//MCLK		  : 24MHz
	//MIPI speed(Mbps): 440Mbps x 2Lane
	//Frame Length	  : 2083
	//Line Length 	  : 2816
	//Max Fps 	  : 30.0fps
	//Pixel order 	  : Green 1st (=GB)
	//X/Y-flip	  : X-flip
	//BLC offset	  : 64code
	////////////////////////////////////////////////
	write_cmos_sensor(0x0a00, 0x0000);
	write_cmos_sensor(0x0b0a, 0x8255);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7167);
	write_cmos_sensor(0x0958, 0xba80);
	write_cmos_sensor(0x0c00, 0x1110);
	write_cmos_sensor(0x0c02, 0x0011);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x000c, 0x0122);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0404);
	write_cmos_sensor(0x0012, 0x001c);
	write_cmos_sensor(0x0018, 0x0a23);
	write_cmos_sensor(0x0024, 0x0122);
	write_cmos_sensor(0x002a, 0x012b);
	write_cmos_sensor(0x0026, 0x0130);
	write_cmos_sensor(0x002c, 0x06cf);
	write_cmos_sensor(0x002e, 0x3311);
	write_cmos_sensor(0x0030, 0x3311);
	write_cmos_sensor(0x0032, 0x3311);
	write_cmos_sensor(0x0006, 0x0823);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0500);
	write_cmos_sensor(0x0a14, 0x02d0);
	write_cmos_sensor(0x0074, 0x0821);
	write_cmos_sensor(0x0070, 0x0407);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x090e, 0x0010);
	write_cmos_sensor(0x090c, 0x09c0);

	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc106);
	write_cmos_sensor(0x0916, 0x040e);
	write_cmos_sensor(0x0918, 0x0304);
	write_cmos_sensor(0x091a, 0x0709);
	write_cmos_sensor(0x091c, 0x0e06);
	write_cmos_sensor(0x091e, 0x0300);
	write_cmos_sensor(0x0a00, 0x0100);
}


static kal_uint32 return_sensor_id(void)
{
    return read_cmos_sensor(0x0f16);
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {

				LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);	
                return ERROR_NONE;
            }
			LOG_INF("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);	
            retry--;
        } while(retry > 0);
        i++;
        retry = 1;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    //const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();
//	set_mirror_flip(IMAGE_HV_MIRROR);
#if Hi546_OTP_FUNCTION
	HI546OTPSetting();
	Hi546_Sensor_OTP_info();
	HI546_Sensor_calc_wbdata();
#endif
    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}   /*  open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		LOG_INF("capture30fps: use cap30FPS's setting: %d fps!\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 
	else  
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		LOG_INF("cap115fps: use cap1's setting: %d fps!\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else  { //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		LOG_INF("Warning:=== current_fps %d fps is not support, so use cap1's setting\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps); 
//	set_mirror_flip(IMAGE_H_MIRROR);	
//	mdelay(10);

#if 0
	if(imgsensor.test_pattern == KAL_TRUE)
	{
		 write_cmos_sensor(0x0a04, 0x0141);
		 write_cmos_sensor(0x0200, 0x0001);
		 write_cmos_sensor(0x0206, 0x000a);
		 write_cmos_sensor(0x0208, 0x0a0a);
		 write_cmos_sensor(0x020a, 0x000a);
		 write_cmos_sensor(0x020c, 0x0a0a);
  	}
#endif

	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	//set_mirror_flip(IMAGE_NORMAL);	
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	//set_mirror_flip(IMAGE_NORMAL);
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	//set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}


static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;


	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;
	sensor_info->PDAF_Support = 0;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 
			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			//set_dummy();
			spin_unlock(&imgsensor_drv_lock);
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			//set_dummy();
			spin_unlock(&imgsensor_drv_lock);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:	
			if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			//set_dummy();
			spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			//set_dummy();
			spin_unlock(&imgsensor_drv_lock);
			}
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			//set_dummy();
			spin_unlock(&imgsensor_drv_lock);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			//set_dummy();
			spin_unlock(&imgsensor_drv_lock);
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			//set_dummy();
			spin_unlock(&imgsensor_drv_lock);
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
#if 0
		 write_cmos_sensor(0x0a04, 0x0141);
		 write_cmos_sensor(0x0200, 0x0001);
		 write_cmos_sensor(0x0206, 0x000a);
		 write_cmos_sensor(0x0208, 0x0a0a);
		 write_cmos_sensor(0x020a, 0x000a);
		 write_cmos_sensor(0x020c, 0x0a0a);
#endif
		write_cmos_sensor(0x0a04, 0x0143);
		write_cmos_sensor(0x0200, 0x0002);
		 LOG_INF(">>crc enable>> reg: %d\n", enable);
	} else {
#if 0
		 write_cmos_sensor(0x0a04, 0x0140);
		 write_cmos_sensor(0x0200, 0x0000);
#endif
		write_cmos_sensor(0x0a04, 0x0142);
		write_cmos_sensor(0x020a, 0x0000);
	}	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
	SET_PD_BLOCK_INFO_T *PDAFinfo;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
			break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */


static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 HI546_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	hi546_MIPI_RAW_SensorInit	*/



