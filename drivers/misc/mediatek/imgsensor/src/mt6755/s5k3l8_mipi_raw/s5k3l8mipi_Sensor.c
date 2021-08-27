/*****************************************************************************
 *
 * Filename:
 * ---------
 *   S5K3L8mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description: 
 * ------------
 *	 Source code of Sensor driver
 *
 *
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
#include "s5k3l8mipi_Sensor.h"
#include <mach/mt_boot_common.h>
#ifdef VENDOR_EDIT
/* Add by LiuBin for register device info at 20160616 */
#define DEVICE_VERSION_S5K3L8    "s5k3l8"
extern void register_imgsensor_deviceinfo(char *name, char *version, u8 module_id);
static uint8_t deviceInfo_register_value = 0x00; 
#endif

#ifdef VENDOR_EDIT //lanhe add
#define USE_KTHREAD
#ifdef USE_KTHREAD
#include <linux/semaphore.h>
#include <linux/kthread.h>
static struct semaphore vendor_sem;
static struct semaphore close_sem;
static volatile bool ms_work_mode = false;
static struct task_struct *vendor_init_task = NULL;
static kal_bool vendor_kthread = KAL_FALSE;
static int vendor_init_kthread(void *data);
static kal_uint8 hw_sensor_mode = IMGSENSOR_MODE_INIT;
static void normal_capture_setting(kal_bool stream_on);
/*oppo hufeng 20160523 add to speed up open camera time when first open after power on phone*/
extern void read_otp_static_array();
/*oppo hufeng 20160107 add to solve eeprom and sensor conflict*/
extern struct mutex sensor_eeprom_lock;
#endif
#endif


#define PFX "S5K3L8_camera_sensor"
#define LOG_1 LOG_INF("S5K3L8,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview revise to capture size; video 4192*3104@30fps,1260Mbps/lane; capture 13M@30fps,1260Mbps/lane\n")
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
#define LOGE(format, args...)   xlog_printk(ANDROID_LOG_ERROR, PFX, "[%s] " format, __FUNCTION__, ##args)
#define BPC_OFF 

static bool bNeedSetNormalMode = KAL_FALSE;
static DEFINE_SPINLOCK(imgsensor_drv_lock);
static char pdaf_data[2048] = {0};
static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = S5K3L8_SENSOR_ID,
	.checksum_value = 0x49c09f86,
	.pre = {
		.pclk = 560000000,				
		.linelength = 5808,				
		.framelength =3206, //3168,			
		.startx = 0,					
		.starty = 0,					
		.grabwindow_width = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 560000000,
		.linelength =5808,
		.framelength = 3206,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,//5334,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
		},
	.cap1 = {
#ifdef VENDOR_EDIT
/*oppo hufeng 20160521 add for cts*/
		.pclk = 560000000,
		.linelength =5808,
		.framelength = 3206,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,//5334,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
#else
		.pclk = 400000000,
		.linelength = 5808,
		.framelength = 4589,
		.startx = 0,
		.starty = 0,
		.grabwindow_width =4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 150,
#endif
	},
#ifndef VENDOR_EDIT
/*zhaozhengtao modify 2016-03-21*/
	.normal_video = {
		.pclk = 560000000,
		.linelength = 5808,
		.framelength = 3206,
		.startx = 0,
		.starty = 0,
		.grabwindow_width =  1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
#else
	.normal_video = {
		.pclk = 560000000,				
		.linelength = 5808,				
		.framelength =3206, //3168,			
		.startx = 0,					
		.starty = 0,					
		.grabwindow_width = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
#endif
	.hs_video = {
		.pclk = 560000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 803,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 640,		//record different mode's width of grabwindow
		.grabwindow_height = 480,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,	
	},
	.slim_video = {
		.pclk = 560000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 803,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 720,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,	
	},
	.custom1 = {
		.pclk = 560000000,
		.linelength =5808,
		.framelength = 3206,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,//5334,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
    .custom2 = {
		.pclk = 440000000,				//record different mode's pclk
		.linelength = 4592,				//record different mode's linelength
		.framelength =3188, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1552,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
    .custom3 = {
		.pclk = 440000000,				//record different mode's pclk
		.linelength = 4592,				//record different mode's linelength
		.framelength =3188, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1552,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
    .custom4 = {
		.pclk = 440000000,				//record different mode's pclk
		.linelength = 4592,				//record different mode's linelength
		.framelength =3188, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1552,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
    .custom5 = {
		.pclk = 440000000,				//record different mode's pclk
		.linelength = 4592,				//record different mode's linelength
		.framelength =3188, //3168,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 2096,		//record different mode's width of grabwindow
		.grabwindow_height = 1552,		//record different mode's height of grabwindow

		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.margin = 5,
	.min_shutter = 4,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame =2,// 1,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3,//2  guanjd modify for slow shutter
	.pre_delay_frame = 2,  
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2, 
    .custom3_delay_frame = 2, 
    .custom4_delay_frame = 2, 
    .custom5_delay_frame = 2,
	
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x5A, 0x5B, 0xff},
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
	.i2c_write_id = 0x5A,
};


/* Sensor output window information*/
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] =	 
{
 { 4208, 3120,	  0,  	0, 4208, 3120, 2104, 1560,   0,	0, 2104, 1560, 	 0, 0, 2104, 1560}, // Preview 
 { 4208, 3120,	  0,  	0, 4208, 3120, 4208, 3120,   0,	0, 4208, 3120, 	 0, 0, 4208, 3120}, // capture 
#ifndef VENDOR_EDIT
/*zhaozhengtao modify 2016-03-21*/
 { 4208, 3120,	  0,  	0, 4208, 3120, 1920, 1080,   0,	0, 1920, 1080, 	 0, 0, 1920, 1080}, // video 
#else
 { 4208, 3120,	  0,  	0, 4208, 3120, 2104, 1560,   0,	0, 2104, 1560, 	 0, 0, 2104, 1560}, // Preview 
#endif
 { 4208, 3120,	184,  120, 3840, 2880,  640,  480,   0,	0,  640,  480, 	 0, 0,  640,  480}, //hight speed video
 { 4208, 3120,	184,  480, 3840, 2160, 1280,  720,   0,	0, 1280,  720, 	 0, 0, 1280,  720},// slim video 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom1 (defaultuse preview) 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom2 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom3 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom4 
 { 4192, 3104,	  0,  0, 4192, 3104, 2096,  1552, 0000, 0000, 2096, 1552, 0,	0, 2096,  1552},// Custom5 
};

 static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
 //for 3l8
{
 
    .i4OffsetX = 28,
 
    .i4OffsetY = 31,
 
    .i4PitchX = 64,
 
    .i4PitchY = 64,
 
    .i4PairNum =16,
 
    .i4SubBlkW =16,
 
    .i4SubBlkH =16, 
 
    .i4PosL = {{28,31},{80,31},{44,35},{64,35},{32,51},{76,51},{48,55},{60,55},{48,63},{60,63},{32,67},{76,67},{44,83},{64,83},{28,87},{80,87}},
 
    .i4PosR = {{28,35},{80,35},{44,39},{64,39},{32,47},{76,47},{48,51},{60,51},{48,67},{60,67},{32,71},{76,71},{44,79},{64,79},{28,83},{80,83}},
 
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}
#if 0
static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}
#endif
static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	 LOG_INF("dummyline = %d, dummypixels = %d ", imgsensor.dummy_line, imgsensor.dummy_pixel);
    //write_cmos_sensor_8(0x0104, 0x01); 
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0342, imgsensor.line_length);
   // write_cmos_sensor_8(0x0104, 0x00); 
  
}	/*	set_dummy  */
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
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



static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
    //kal_uint32 frame_length = 0;
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)       
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	//shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
    } else {
        // Extend frame length
        //write_cmos_sensor_8(0x0104,0x01);
        write_cmos_sensor(0x0340, imgsensor.frame_length);
        //write_cmos_sensor_8(0x0104,0x00);
    }

#ifdef VENDOR_EDIT
//iAudi modify for slow shutter
    if(bNeedSetNormalMode){
        LOG_INF("change to normal mode\n");
        write_cmos_sensor(0x6028,0x4000);
        if(imgsensor_info.module_id==0x04)
        	write_cmos_sensor(0x0100,0x0003);  // stream Off 
        else
        	write_cmos_sensor(0x0100,0x0000);  // stream Off 	
        write_cmos_sensor(0x0306,0x00AF);  //177 main pll M
        write_cmos_sensor(0x0302,0x0001); //vtpix 
        write_cmos_sensor(0x030E,0x0119); //281 mipi pll M
        write_cmos_sensor(0x3008,0x0000); // mipi s-divider
        write_cmos_sensor(0x31e4,0x02BC);	//#senAnalog_AIG_DefCclkFreqMhz 0058   //default : 02BC //test tg
        write_cmos_sensor(0x0104, 0x0100);  //GP-on
        write_cmos_sensor(0x0202,0x0200);  //CintR
        write_cmos_sensor(0x0342,0x16B0);
        write_cmos_sensor(0x0340,0x0CB2);
        write_cmos_sensor(0x0104, 0x0000);  //GP-off
        write_cmos_sensor(0x316A,0x00A0);  //OUTIF threshold
        write_cmos_sensor(0x304A,0x0000); 
		
        if(imgsensor_info.module_id==0x04)
        	write_cmos_sensor(0x0100,0x0103);  
        else
        	write_cmos_sensor(0x0100,0x0100);  
        bNeedSetNormalMode = KAL_FALSE;
    }
    LOG_INF("frame_length is %d   shutter=%d\n",imgsensor.frame_length, shutter);

    if(shutter>65520){
		if(shutter > 2096800)
			shutter = 2096000;
		write_cmos_sensor(0x6028, 0x4000);
		write_cmos_sensor(0x602A, 0x0100);
		write_cmos_sensor(0x6F12, 0x0000);
			
		write_cmos_sensor(0x0306, 0x007D);
		write_cmos_sensor(0x0302, 0x0008);
		write_cmos_sensor(0x030E, 0x0080);
		write_cmos_sensor(0x3008, 0x0002);

		write_cmos_sensor(0x31E4, 0x0058);

		if(shutter > 65520&&shutter < 131100){
			LOG_INF("shutter 1s\n");
			write_cmos_sensor(0x0104, 0x0100);  //GP-on
			write_cmos_sensor(0x0342, 0x3F86);
			write_cmos_sensor(0x0200, 0x00c6);
		    write_cmos_sensor(0x0202, 0x0c03);
			write_cmos_sensor(0x0104, 0x0000);  //GP-off
		    write_cmos_sensor(0x316A, 0x0000);
		    
			if(imgsensor_info.module_id==0x04)
		    	write_cmos_sensor(0x0100,0x0103);  
			else
		    	write_cmos_sensor(0x0100,0x0100);  
		}
		else if(shutter > 131100&&shutter < 262100){
			LOG_INF("shutter 2s\n");
			write_cmos_sensor(0x0104, 0x0100);  //GP-on
			write_cmos_sensor(0x0342, 0x3F86);
			write_cmos_sensor(0x0200, 0x00c6);
		    write_cmos_sensor(0x0202, 0x1806);
			write_cmos_sensor(0x0104, 0x0000);  //GP-off
		    write_cmos_sensor(0x316A, 0x0000);
		    
			if(imgsensor_info.module_id==0x04)
		    	write_cmos_sensor(0x0100,0x0103);  
			else
		    	write_cmos_sensor(0x0100,0x0100); 
		}
		else if(shutter > 262100&&shutter <524200){
			LOG_INF("shutter 4s\n");
			write_cmos_sensor(0x0104, 0x0100);  //GP-on
			write_cmos_sensor(0x0340, 0x301c);
			write_cmos_sensor(0x0342, 0x3F86);
			write_cmos_sensor(0x0200, 0x00c6);
		    write_cmos_sensor(0x0202, 0x300c);
			write_cmos_sensor(0x0104, 0x0000);  //GP-off
		    write_cmos_sensor(0x316A, 0x0000);
		    
			if(imgsensor_info.module_id==0x04)
		    	write_cmos_sensor(0x0100,0x0103);  
			else
		    	write_cmos_sensor(0x0100,0x0100); 
		}
		else if(shutter > 524200&&shutter <1048400){
			LOG_INF("shutter 8s\n");
			write_cmos_sensor(0x0104, 0x0100);  //GP-on
			write_cmos_sensor(0x0340, 0x6026);
			write_cmos_sensor(0x0342, 0x3F86);
			write_cmos_sensor(0x0200, 0x00c6);
		    write_cmos_sensor(0x0202, 0x6016);
			write_cmos_sensor(0x0104, 0x0000);  //GP-off
		    write_cmos_sensor(0x316A, 0x0000);
		    
			if(imgsensor_info.module_id==0x04)
		    	write_cmos_sensor(0x0100,0x0103);  
			else
		    	write_cmos_sensor(0x0100,0x0100); 
		}
		else if(shutter > 1048400&&shutter <2096800){
			LOG_INF("shutter 16s\n");
			write_cmos_sensor(0x0104, 0x0100);  //GP-on
			write_cmos_sensor(0x0340, 0xc03b);
			write_cmos_sensor(0x0342, 0x3F86);
			write_cmos_sensor(0x0200, 0x00c6);
		    write_cmos_sensor(0x0202, 0xc02b);
			write_cmos_sensor(0x0104, 0x0000);  //GP-off
			write_cmos_sensor(0x316A, 0x0000);
		    
			if(imgsensor_info.module_id==0x04)
		    	write_cmos_sensor(0x0100,0x0103);  
			else
		    	write_cmos_sensor(0x0100,0x0100); 
		}
		/*else{
			LOG_INF("shutter default\n");
			write_cmos_sensor(0x6028, 0x4000);
			write_cmos_sensor(0x0100, 0x0000);
			write_cmos_sensor(0x0306, 0x00AF);//c6c
			write_cmos_sensor(0x0302, 0x0001);
			write_cmos_sensor(0x030E, 0x0119);
			write_cmos_sensor(0x3008, 0x0000);
				
			write_cmos_sensor(0x31E4, 0x02BC);
			write_cmos_sensor(0x0104, 0x0100);	
			write_cmos_sensor(0x0202, 0x0200);
			write_cmos_sensor(0x0342, 0x16B0);
			write_cmos_sensor(0x0340, 0x0CB2);
			write_cmos_sensor(0x316A, 0x00A0);
				
			write_cmos_sensor(0x304A, 0x0000);
			write_cmos_sensor(0x0104, 0x0000);
			write_cmos_sensor(0x0100,0x0100);
		}*/
		//write_cmos_sensor(0x0100,0x0100);
		bNeedSetNormalMode = KAL_TRUE;

    }
    else{
    	write_cmos_sensor(0x0340, imgsensor.frame_length);
    	write_cmos_sensor(0x0202, shutter);
    }
#else
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0202, shutter);

#endif
    //write_cmos_sensor_8(0x0104,0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
}	/*	write_shutter  */


#if 0
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
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	write_shutter(shutter);
}	/*	set_shutter */
#endif


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint16 reg_gain = 0x0;
    
    reg_gain = gain/2;
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
	  kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X  */
    /* [4:9] = M meams M X       */
    /* Total gain = M + N /16 X   */

    //
    if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
        LOG_INF("Error gain setting");

        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 32 * BASEGAIN)
            gain = 32 * BASEGAIN;        
    }
 
    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain; 
    spin_unlock(&imgsensor_drv_lock);
   // LOG_INF("gain = %d , reg_gain = 0x%x ", gain, reg_gain);

    //write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor_8(0x0204,(reg_gain>>8));
    write_cmos_sensor_8(0x0205,(reg_gain&0xff));
    //write_cmos_sensor_8(0x0104, 0x00);
    
    return gain;
}	/*	set_gain  */
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d", image_mirror);

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
            write_cmos_sensor(0x0100,0x0100);   // Gr
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor(0x0100,0x0101);
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor(0x0100,0x0102);
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor(0x0100,0x0103);//Gb
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
static void sensor_init(void)
{
  //s 2015/08/27, iAudi
  LOG_INF("-- sensor_init, chip id = s5k3l8\n");
	write_cmos_sensor(0x6028, 0x4000); 
	write_cmos_sensor(0x6214, 0xFFFF);
	write_cmos_sensor(0x6216, 0xFFFF);
	write_cmos_sensor(0x6218, 0x0000);
	write_cmos_sensor(0x621A, 0x0000);

	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x2450);
	write_cmos_sensor(0x6F12, 0x0448);
	write_cmos_sensor(0x6F12, 0x0349);
	write_cmos_sensor(0x6F12, 0x0160);
	write_cmos_sensor(0x6F12, 0xC26A);
	write_cmos_sensor(0x6F12, 0x511A);
	write_cmos_sensor(0x6F12, 0x8180);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x48B8);
	write_cmos_sensor(0x6F12, 0x2000); 
	write_cmos_sensor(0x6F12, 0x2588); 
	write_cmos_sensor(0x6F12, 0x2000); 
	write_cmos_sensor(0x6F12, 0x16C0); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x10B5); 
	write_cmos_sensor(0x6F12, 0x00F0); 
	write_cmos_sensor(0x6F12, 0x5DF8); 
	write_cmos_sensor(0x6F12, 0x2748); 
	write_cmos_sensor(0x6F12, 0x4078); 
	write_cmos_sensor(0x6F12, 0x0028); 
	write_cmos_sensor(0x6F12, 0x0AD0); 
	write_cmos_sensor(0x6F12, 0x00F0); 
	write_cmos_sensor(0x6F12, 0x5CF8); 
	write_cmos_sensor(0x6F12, 0x2549); 
	write_cmos_sensor(0x6F12, 0xB1F8); 
	write_cmos_sensor(0x6F12, 0x1403); 
	write_cmos_sensor(0x6F12, 0x4200); 
	write_cmos_sensor(0x6F12, 0x2448); 
	write_cmos_sensor(0x6F12, 0x4282); 
	write_cmos_sensor(0x6F12, 0x91F8); 
	write_cmos_sensor(0x6F12, 0x9610); 
	write_cmos_sensor(0x6F12, 0x4187); 
	write_cmos_sensor(0x6F12, 0x10BD); 
	write_cmos_sensor(0x6F12, 0x70B5); 
	write_cmos_sensor(0x6F12, 0x0446); 
	write_cmos_sensor(0x6F12, 0x2148); 
	write_cmos_sensor(0x6F12, 0x0022); 
	write_cmos_sensor(0x6F12, 0x4068); 
	write_cmos_sensor(0x6F12, 0x86B2); 
	write_cmos_sensor(0x6F12, 0x050C); 
	write_cmos_sensor(0x6F12, 0x3146); 
	write_cmos_sensor(0x6F12, 0x2846); 
	write_cmos_sensor(0x6F12, 0x00F0); 
	write_cmos_sensor(0x6F12, 0x4CF8); 
	write_cmos_sensor(0x6F12, 0x2046); 
	write_cmos_sensor(0x6F12, 0x00F0); 
	write_cmos_sensor(0x6F12, 0x4EF8); 
	write_cmos_sensor(0x6F12, 0x14F8);  
	write_cmos_sensor(0x6F12, 0x680F); 
	write_cmos_sensor(0x6F12, 0x6178); 
	write_cmos_sensor(0x6F12, 0x40EA); 
	write_cmos_sensor(0x6F12, 0x4100); 
	write_cmos_sensor(0x6F12, 0x1749); 
	write_cmos_sensor(0x6F12, 0xC886); 
	write_cmos_sensor(0x6F12, 0x1848); 
	write_cmos_sensor(0x6F12, 0x2278); 
	write_cmos_sensor(0x6F12, 0x007C); 
	write_cmos_sensor(0x6F12, 0x4240); 
	write_cmos_sensor(0x6F12, 0x1348); 
	write_cmos_sensor(0x6F12, 0xA230); 
	write_cmos_sensor(0x6F12, 0x8378); 
	write_cmos_sensor(0x6F12, 0x43EA); 
	write_cmos_sensor(0x6F12, 0xC202); 
	write_cmos_sensor(0x6F12, 0x0378); 
	write_cmos_sensor(0x6F12, 0x4078); 
	write_cmos_sensor(0x6F12, 0x9B00); 
	write_cmos_sensor(0x6F12, 0x43EA); 
	write_cmos_sensor(0x6F12, 0x4000); 
	write_cmos_sensor(0x6F12, 0x0243); 
	write_cmos_sensor(0x6F12, 0xD0B2); 
	write_cmos_sensor(0x6F12, 0x0882); 
	write_cmos_sensor(0x6F12, 0x3146); 
	write_cmos_sensor(0x6F12, 0x2846); 
	write_cmos_sensor(0x6F12, 0xBDE8); 
	write_cmos_sensor(0x6F12, 0x7040); 
	write_cmos_sensor(0x6F12, 0x0122); 
	write_cmos_sensor(0x6F12, 0x00F0); 
	write_cmos_sensor(0x6F12, 0x2AB8); 
	write_cmos_sensor(0x6F12, 0x10B5); 
	write_cmos_sensor(0x6F12, 0x0022); 
	write_cmos_sensor(0x6F12, 0xAFF2); 
	write_cmos_sensor(0x6F12, 0x8701); 
	write_cmos_sensor(0x6F12, 0x0B48); 
	write_cmos_sensor(0x6F12, 0x00F0); 
	write_cmos_sensor(0x6F12, 0x2DF8); 
	write_cmos_sensor(0x6F12, 0x084C); 
	write_cmos_sensor(0x6F12, 0x0022); 
	write_cmos_sensor(0x6F12, 0xAFF2); 
	write_cmos_sensor(0x6F12, 0x6D01); 
	write_cmos_sensor(0x6F12, 0x2060); 
	write_cmos_sensor(0x6F12, 0x0848); 
	write_cmos_sensor(0x6F12, 0x00F0); 
	write_cmos_sensor(0x6F12, 0x25F8); 
	write_cmos_sensor(0x6F12, 0x6060); 
	write_cmos_sensor(0x6F12, 0x10BD); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x2000); 
	write_cmos_sensor(0x6F12, 0x0550); 
	write_cmos_sensor(0x6F12, 0x2000); 
	write_cmos_sensor(0x6F12, 0x0C60); 
	write_cmos_sensor(0x6F12, 0x4000); 
	write_cmos_sensor(0x6F12, 0xD000); 
	write_cmos_sensor(0x6F12, 0x2000); 
	write_cmos_sensor(0x6F12, 0x2580); 
	write_cmos_sensor(0x6F12, 0x2000); 
	write_cmos_sensor(0x6F12, 0x16F0); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x2221); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x2249); 
	write_cmos_sensor(0x6F12, 0x42F2); 
	write_cmos_sensor(0x6F12, 0x351C); 
	write_cmos_sensor(0x6F12, 0xC0F2); 
	write_cmos_sensor(0x6F12, 0x000C); 
	write_cmos_sensor(0x6F12, 0x6047); 
	write_cmos_sensor(0x6F12, 0x42F2); 
	write_cmos_sensor(0x6F12, 0xE11C); 
	write_cmos_sensor(0x6F12, 0xC0F2); 
	write_cmos_sensor(0x6F12, 0x000C); 
	write_cmos_sensor(0x6F12, 0x6047); 
	write_cmos_sensor(0x6F12, 0x40F2); 
	write_cmos_sensor(0x6F12, 0x077C); 
	write_cmos_sensor(0x6F12, 0xC0F2); 
	write_cmos_sensor(0x6F12, 0x000C); 
	write_cmos_sensor(0x6F12, 0x6047); 
	write_cmos_sensor(0x6F12, 0x42F2); 
	write_cmos_sensor(0x6F12, 0x492C); 
	write_cmos_sensor(0x6F12, 0xC0F2); 
	write_cmos_sensor(0x6F12, 0x000C); 
	write_cmos_sensor(0x6F12, 0x6047); 
	write_cmos_sensor(0x6F12, 0x4BF2); 
	write_cmos_sensor(0x6F12, 0x453C); 
	write_cmos_sensor(0x6F12, 0xC0F2); 
	write_cmos_sensor(0x6F12, 0x000C); 
	write_cmos_sensor(0x6F12, 0x6047); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x30C8); 
	write_cmos_sensor(0x6F12, 0x0157); 
	write_cmos_sensor(0x6F12, 0x0000); 
	write_cmos_sensor(0x6F12, 0x0003); 
	                               
	//                               
	write_cmos_sensor(0x6028, 0x2000);                 
	write_cmos_sensor(0x602A, 0x1082);                 
	write_cmos_sensor(0x6F12, 0x8010);                 
	write_cmos_sensor(0x6028, 0x4000);                 
	write_cmos_sensor(0x31CE, 0x0001);                 
	write_cmos_sensor(0x0200, 0x00C6);                 
	write_cmos_sensor(0x3734, 0x0010);                 
	write_cmos_sensor(0x3736, 0x0001);                 
	write_cmos_sensor(0x3738, 0x0001);                  
	write_cmos_sensor(0x37CC, 0x0001);                  
	write_cmos_sensor(0x3744, 0x0100);                  
	write_cmos_sensor(0x3762, 0x0105);                  
	write_cmos_sensor(0x3764, 0x0105);                  
	write_cmos_sensor(0x376A, 0x00F0);                  
	write_cmos_sensor(0x344A, 0x000F);                  
	write_cmos_sensor(0x344C, 0x003D);                  
	write_cmos_sensor(0xF460, 0x0030);                  
	write_cmos_sensor(0xF414, 0x24C2);                  
	write_cmos_sensor(0xF416, 0x0183);                  
	write_cmos_sensor(0xF468, 0x0405);                  
	write_cmos_sensor(0x3424, 0x0A07);                  
	write_cmos_sensor(0x3426, 0x0F07);                  
	write_cmos_sensor(0x3428, 0x0F07);                  
	write_cmos_sensor(0x341E, 0x0804);                  
	write_cmos_sensor(0x3420, 0x0C0C);                  
	write_cmos_sensor(0x3422, 0x2D2D);                  
	write_cmos_sensor(0xF462, 0x003A);                  
	write_cmos_sensor(0x3450, 0x0010);                  
	write_cmos_sensor(0x3452, 0x0010);                   
	write_cmos_sensor(0xF446, 0x0020);                   
	write_cmos_sensor(0xF44E, 0x000C);                   
	write_cmos_sensor(0x31FA, 0x0007);                   
	write_cmos_sensor(0x31FC, 0x0161);                   
	write_cmos_sensor(0x31FE, 0x0009);                   
	write_cmos_sensor(0x3200, 0x000C);                   
	write_cmos_sensor(0x3202, 0x007F);                   
	write_cmos_sensor(0x3204, 0x00A2);                   
	write_cmos_sensor(0x3206, 0x007D);                   
	write_cmos_sensor(0x3208, 0x00A4);                   
	write_cmos_sensor(0x3334, 0x00A7);                   
	write_cmos_sensor(0x3336, 0x00A5);                   
	write_cmos_sensor(0x3338, 0x0033);                   
	write_cmos_sensor(0x333A, 0x0006);                   
	write_cmos_sensor(0x333C, 0x009F);                   
	write_cmos_sensor(0x333E, 0x008C);                   
	write_cmos_sensor(0x3340, 0x002D);                   
	write_cmos_sensor(0x3342, 0x000A);                   
	write_cmos_sensor(0x3344, 0x002F);                   
	write_cmos_sensor(0x3346, 0x0008);                   
	write_cmos_sensor(0x3348, 0x009F);                   
	write_cmos_sensor(0x334A, 0x008C);                   
	write_cmos_sensor(0x334C, 0x002D);                   
	write_cmos_sensor(0x334E, 0x000A);                   
	write_cmos_sensor(0x3350, 0x000A);                   
	write_cmos_sensor(0x320A, 0x007B);                   
	write_cmos_sensor(0x320C, 0x0161);                   
	write_cmos_sensor(0x320E, 0x007F);                   
	write_cmos_sensor(0x3210, 0x015F);                   
	write_cmos_sensor(0x3212, 0x007B);                   
	write_cmos_sensor(0x3214, 0x00B0);                   
	write_cmos_sensor(0x3216, 0x0009);                   
	write_cmos_sensor(0x3218, 0x0038);                   
	write_cmos_sensor(0x321A, 0x0009);                   
	write_cmos_sensor(0x321C, 0x0031);                   
	write_cmos_sensor(0x321E, 0x0009);                   
	write_cmos_sensor(0x3220, 0x0038);                   
	write_cmos_sensor(0x3222, 0x0009);                   
	write_cmos_sensor(0x3224, 0x007B);                   
	write_cmos_sensor(0x3226, 0x0001);                   
	write_cmos_sensor(0x3228, 0x0010);                   
	write_cmos_sensor(0x322A, 0x00A2);                   
	write_cmos_sensor(0x322C, 0x00B1);                   
	write_cmos_sensor(0x322E, 0x0002);                   
	write_cmos_sensor(0x3230, 0x015D);                   
	write_cmos_sensor(0x3232, 0x0001);                   
	write_cmos_sensor(0x3234, 0x015D);                   
	write_cmos_sensor(0x3236, 0x0001);                   
	write_cmos_sensor(0x3238, 0x000B);                   
	write_cmos_sensor(0x323A, 0x0016);                   
	write_cmos_sensor(0x323C, 0x000D);                   
	write_cmos_sensor(0x323E, 0x001C);                   
	write_cmos_sensor(0x3240, 0x000D);                   
	write_cmos_sensor(0x3242, 0x0054);                   
	write_cmos_sensor(0x3244, 0x007B);                   
	write_cmos_sensor(0x3246, 0x00CC);                  
	write_cmos_sensor(0x3248, 0x015D);                  
	write_cmos_sensor(0x324A, 0x007E);                  
	write_cmos_sensor(0x324C, 0x0095);                  
	write_cmos_sensor(0x324E, 0x0085);                  
	write_cmos_sensor(0x3250, 0x009D);                  
	write_cmos_sensor(0x3252, 0x008D);                  
	write_cmos_sensor(0x3254, 0x009D);                  
	write_cmos_sensor(0x3256, 0x007E);                  
	write_cmos_sensor(0x3258, 0x0080);                  
	write_cmos_sensor(0x325A, 0x0001);                  
	write_cmos_sensor(0x325C, 0x0005);                  
	write_cmos_sensor(0x325E, 0x0085);                  
	write_cmos_sensor(0x3260, 0x009D);                
	write_cmos_sensor(0x3262, 0x0001);                  
	write_cmos_sensor(0x3264, 0x0005);                  
	write_cmos_sensor(0x3266, 0x007E);                  
	write_cmos_sensor(0x3268, 0x0080);                  
	write_cmos_sensor(0x326A, 0x0053);                  
	write_cmos_sensor(0x326C, 0x007D);                  
	write_cmos_sensor(0x326E, 0x00CB);                  
	write_cmos_sensor(0x3270, 0x015E);                  
	write_cmos_sensor(0x3272, 0x0001);                  
	write_cmos_sensor(0x3274, 0x0005);                  
	write_cmos_sensor(0x3276, 0x0009);                  
	write_cmos_sensor(0x3278, 0x000C);                  
	write_cmos_sensor(0x327A, 0x007E);                  
	write_cmos_sensor(0x327C, 0x0098);                  
	write_cmos_sensor(0x327E, 0x0009);                   
	write_cmos_sensor(0x3280, 0x000C);                   
	write_cmos_sensor(0x3282, 0x007E);                   
	write_cmos_sensor(0x3284, 0x0080);                   
	write_cmos_sensor(0x3286, 0x0044);                   
	write_cmos_sensor(0x3288, 0x0163);                   
	write_cmos_sensor(0x328A, 0x0045);                   
	write_cmos_sensor(0x328C, 0x0047);                   
	write_cmos_sensor(0x328E, 0x007D);                   
	write_cmos_sensor(0x3290, 0x0080);                   
	write_cmos_sensor(0x3292, 0x015F);                   
	write_cmos_sensor(0x3294, 0x0162);                   
	write_cmos_sensor(0x3296, 0x007D);                   
	write_cmos_sensor(0x3298, 0x0000);                   
	write_cmos_sensor(0x329A, 0x0000);                   
	write_cmos_sensor(0x329C, 0x0000);                   
	write_cmos_sensor(0x329E, 0x0000);                   
	write_cmos_sensor(0x32A0, 0x0008);                   
	write_cmos_sensor(0x32A2, 0x0010);                   
	write_cmos_sensor(0x32A4, 0x0018);                   
	write_cmos_sensor(0x32A6, 0x0020);                   
	write_cmos_sensor(0x32A8, 0x0000);                   
	write_cmos_sensor(0x32AA, 0x0008);                   
	write_cmos_sensor(0x32AC, 0x0010);                   
	write_cmos_sensor(0x32AE, 0x0018);                   
	write_cmos_sensor(0x32B0, 0x0020);                   
	write_cmos_sensor(0x32B2, 0x0020);                   
	write_cmos_sensor(0x32B4, 0x0020);                   
	write_cmos_sensor(0x32B6, 0x0020);                   
	write_cmos_sensor(0x32B8, 0x0000);                   
	write_cmos_sensor(0x32BA, 0x0000);                   
	write_cmos_sensor(0x32BC, 0x0000);                   
	write_cmos_sensor(0x32BE, 0x0000);                   
	write_cmos_sensor(0x32C0, 0x0000);                   
	write_cmos_sensor(0x32C2, 0x0000);                   
	write_cmos_sensor(0x32C4, 0x0000);                   
	write_cmos_sensor(0x32C6, 0x0000);                   
	write_cmos_sensor(0x32C8, 0x0000);                   
	write_cmos_sensor(0x32CA, 0x0000);                   
	write_cmos_sensor(0x32CC, 0x0000);                   
	write_cmos_sensor(0x32CE, 0x0000);                   
	write_cmos_sensor(0x32D0, 0x0000);                   
	write_cmos_sensor(0x32D2, 0x0000);                   
	write_cmos_sensor(0x32D4, 0x0000);                   
	write_cmos_sensor(0x32D6, 0x0000);                   
	write_cmos_sensor(0x32D8, 0x0000);                   
	write_cmos_sensor(0x32DA, 0x0000);                   
	write_cmos_sensor(0x32DC, 0x0000);                   
	write_cmos_sensor(0x32DE, 0x0000);                   
	write_cmos_sensor(0x32E0, 0x0000);                   
	write_cmos_sensor(0x32E2, 0x0000);                   
	write_cmos_sensor(0x32E4, 0x0000);                   
	write_cmos_sensor(0x32E6, 0x0000);                   
	write_cmos_sensor(0x32E8, 0x0000);                   
	write_cmos_sensor(0x32EA, 0x0000);                   
	write_cmos_sensor(0x32EC, 0x0000);                   
	write_cmos_sensor(0x32EE, 0x0000);                
	write_cmos_sensor(0x32F0, 0x0000);                
	write_cmos_sensor(0x32F2, 0x0000);                
	write_cmos_sensor(0x32F4, 0x000A);                
	write_cmos_sensor(0x32F6, 0x0002);                
	write_cmos_sensor(0x32F8, 0x0008);                
	write_cmos_sensor(0x32FA, 0x0010);                
	write_cmos_sensor(0x32FC, 0x0020);                
	write_cmos_sensor(0x32FE, 0x0028);                
	write_cmos_sensor(0x3300, 0x0038);                
	write_cmos_sensor(0x3302, 0x0040);                
	write_cmos_sensor(0x3304, 0x0050);                
	write_cmos_sensor(0x3306, 0x0058);                
	write_cmos_sensor(0x3308, 0x0068);                
	write_cmos_sensor(0x330A, 0x0070);                
	write_cmos_sensor(0x330C, 0x0080);                
	write_cmos_sensor(0x330E, 0x0088);                
	write_cmos_sensor(0x3310, 0x0098);                
	write_cmos_sensor(0x3312, 0x00A0);                
	write_cmos_sensor(0x3314, 0x00B0);                
	write_cmos_sensor(0x3316, 0x00B8);                
	write_cmos_sensor(0x3318, 0x00C8);                
	write_cmos_sensor(0x331A, 0x00D0);                
	write_cmos_sensor(0x331C, 0x00E0);                
	write_cmos_sensor(0x331E, 0x00E8);                
	write_cmos_sensor(0x3320, 0x0017);                
	write_cmos_sensor(0x3322, 0x002F);                
	write_cmos_sensor(0x3324, 0x0047);                
	write_cmos_sensor(0x3326, 0x005F);                  
	write_cmos_sensor(0x3328, 0x0077);                  
	write_cmos_sensor(0x332A, 0x008F);                  
	write_cmos_sensor(0x332C, 0x00A7);                  
	write_cmos_sensor(0x332E, 0x00BF);                  
	write_cmos_sensor(0x3330, 0x00D7);                  
	write_cmos_sensor(0x3332, 0x00EF);                  
	write_cmos_sensor(0x3352, 0x00A5);                  
	write_cmos_sensor(0x3354, 0x00AF);                  
	write_cmos_sensor(0x3356, 0x0187);                  
	write_cmos_sensor(0x3358, 0x0000);                  
	write_cmos_sensor(0x335A, 0x009E);                  
	write_cmos_sensor(0x335C, 0x016B);                  
	write_cmos_sensor(0x335E, 0x0015);                  
	write_cmos_sensor(0x3360, 0x00A5);                  
	write_cmos_sensor(0x3362, 0x00AF);                  
	write_cmos_sensor(0x3364, 0x01FB);                  
	write_cmos_sensor(0x3366, 0x0000);                  
	write_cmos_sensor(0x3368, 0x009E);                  
	write_cmos_sensor(0x336A, 0x016B);                  
	write_cmos_sensor(0x336C, 0x0015);                  
	write_cmos_sensor(0x336E, 0x00A5);                  
	write_cmos_sensor(0x3370, 0x00A6);                  
	write_cmos_sensor(0x3372, 0x0187);                  
	write_cmos_sensor(0x3374, 0x0000);                  
	write_cmos_sensor(0x3376, 0x009E);                  
	write_cmos_sensor(0x3378, 0x016B);                  
	write_cmos_sensor(0x337A, 0x0015);                  
	write_cmos_sensor(0x337C, 0x00A5);                    
	write_cmos_sensor(0x337E, 0x00A6);                    
	write_cmos_sensor(0x3380, 0x01FB);                    
	write_cmos_sensor(0x3382, 0x0000);                    
	write_cmos_sensor(0x3384, 0x009E);                    
	write_cmos_sensor(0x3386, 0x016B);                    
	write_cmos_sensor(0x3388, 0x0015);                    
	write_cmos_sensor(0x319A, 0x0005);                    
	write_cmos_sensor(0x1006, 0x0005);                    
	write_cmos_sensor(0x3416, 0x0001);                    
	write_cmos_sensor(0x308C, 0x0008);                    
	write_cmos_sensor(0x307C, 0x0240);                    
	write_cmos_sensor(0x375E, 0x0050);                    
	//write_cmos_sensor(0x316A, 0x00A0);                    
	write_cmos_sensor(0x31CE, 0x0101);                    
	write_cmos_sensor(0x374E, 0x0007);                    
	write_cmos_sensor(0x3460, 0x0001);
	write_cmos_sensor(0x3052, 0x0002);
	write_cmos_sensor(0x3058, 0x0100);//0x0001//samsung modify for MIPI
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x108A);
	write_cmos_sensor(0x6F12, 0x0359);
	write_cmos_sensor(0x6F12, 0x0100);
	//samsung modify for MIPI ++++
	write_cmos_sensor(0x6028, 0x4000);	// For long-shutter
	write_cmos_sensor(0x1124, 0x4100);
	write_cmos_sensor(0x1126, 0x0000);
	write_cmos_sensor(0x112C, 0x4100);
	write_cmos_sensor(0x112E, 0x0000);
	
	write_cmos_sensor(0xB0C8, 0x0002);	// MIPI VOD level control (Minimum)
																			// 0:240mV(Default), 1:250mV, 2:200mV, 3:220mV
	//samsung modify for MIPI----
	write_cmos_sensor(0x6028, 0x4000);	// For firstRow defect
	write_cmos_sensor(0xF468, 0x4005);
	write_cmos_sensor(0x3442, 0x0100);
	//control continue mode use B0A0 is better
	#ifdef NONCONTINUEMODE
	write_cmos_sensor_8(0xB0A0, 0x7C);	//non continue mode
	#else
	write_cmos_sensor_8(0xB0A0, 0x7D);	//continue mode
	#endif
	write_cmos_sensor(0x0100, 0x0000);

}	/*	sensor_init  */
/**********************************************************************************************************************/
//$MIPI[Width:2096,Height:1552,Format:RAW10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:1260,useEmbData:0]
//$MV1[MCLK:24,Width:2096,Height:1552,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1260,pvi_pclk_inverse:0]
//=====================================================
// 3M2XXM
// 2X2 Binning Normal Mode
// X_output size : 2096
// Y_output size : 1552
// Frame_rate : 30.06 fps
// Output_format RAW 10
// Output_lanes 4
// Output_clock_mhz : 1272 Mhz
// System_clock_mhz : 440 Mhz
// Input_clock_mhz : 24 Mhz
// TnP R651
//=====================================================
/**********************************************************************************************************************/
static void preview_setting(void)
{
    // s 2015/08/27, iAudi
	LOG_INF("preview_setting, chip_id: s5k3l8\n");

    write_cmos_sensor(0x6028,0x4000);
    write_cmos_sensor(0x0100,0x0000);
	
    write_cmos_sensor(0x6028,0x0200);
    write_cmos_sensor(0x602A,0x0F74);
    write_cmos_sensor(0x6F12,0x0040);
    write_cmos_sensor(0x6F12,0x0040);
    write_cmos_sensor(0x6028,0x4000);
    write_cmos_sensor(0x0344,0x0008);
    write_cmos_sensor(0x0346,0x0008);
    write_cmos_sensor(0x0348,0x1077);
    write_cmos_sensor(0x034A,0x0C37);
    write_cmos_sensor(0x034C,0x0838);
    write_cmos_sensor(0x034E,0x0618);   //1560
    write_cmos_sensor(0x0900,0x0112);
    write_cmos_sensor(0x0380,0x0001);
    write_cmos_sensor(0x0382,0x0001);
    write_cmos_sensor(0x0384,0x0001);
    write_cmos_sensor(0x0386,0x0003);
    write_cmos_sensor(0x0400,0x0001);
    write_cmos_sensor(0x0404,0x0020);
    write_cmos_sensor(0x0114,0x0300);
    write_cmos_sensor(0x1100,0x0002);
    write_cmos_sensor(0x0136,0x1800);
    write_cmos_sensor(0x0304,0x0006);  //6
    write_cmos_sensor(0x0306,0x00AF);
    write_cmos_sensor(0x0302,0x0001);
    write_cmos_sensor(0x0300,0x0005);
    write_cmos_sensor(0x030C,0x0006);
    write_cmos_sensor(0x030E,0x0119);  //281
    write_cmos_sensor(0x030A,0x0001);
    write_cmos_sensor(0x0308,0x0008);
    write_cmos_sensor(0x0342,0x16B0);
    write_cmos_sensor(0x0340,0x0C86);
    write_cmos_sensor(0x0202,0x0200);
    write_cmos_sensor(0x0200,0x00C6);
    write_cmos_sensor(0x0B04,0x0101);
    write_cmos_sensor(0x0B08,0x0000);
    write_cmos_sensor(0x0B00,0x0007);
    write_cmos_sensor(0x316A,0x00A0);
    if(imgsensor_info.module_id==0x04)
    	write_cmos_sensor(0x0100,0x0103);  
    else
    	write_cmos_sensor(0x0100,0x0100);  	
	//e, 2015/08/27, iAudi
}	/*	preview_setting  */
/**********************************************
*copy from preview setting set 1/4 bining*****
**
*********************************************/
static void video_setting(void)
{
    LOG_INF("preview_setting, chip_id: s5k3l8\n");
    //s, 2015/08/27, iAudi
    //Page Pointer HW 
    write_cmos_sensor(0x6028, 0x4000);
    write_cmos_sensor(0x0100, 0x0000);
									
    //Mode								
    write_cmos_sensor(0x6028, 0x2000);
    write_cmos_sensor(0x602A, 0x0F74);
    write_cmos_sensor(0x6F12, 0x0040);
    write_cmos_sensor(0x6F12, 0x0040);
    write_cmos_sensor(0x6028, 0x4000);
    write_cmos_sensor(0x0344, 0x00C0);
    write_cmos_sensor(0x0346, 0x01E8);
    write_cmos_sensor(0x0348, 0x0FBF);
    write_cmos_sensor(0x034A, 0x0A57);
    write_cmos_sensor(0x034C, 0x0780);
    write_cmos_sensor(0x034E, 0x0438);
    write_cmos_sensor(0x0900, 0x0112);
    write_cmos_sensor(0x0380, 0x0001);
    write_cmos_sensor(0x0382, 0x0001);
    write_cmos_sensor(0x0384, 0x0001);
    write_cmos_sensor(0x0386, 0x0003);
    write_cmos_sensor(0x0400, 0x0001);
    write_cmos_sensor(0x0404, 0x0020);
    write_cmos_sensor(0x0114, 0x0300);
    write_cmos_sensor(0x0110, 0x0002);
    write_cmos_sensor(0x0136, 0x1800);
    write_cmos_sensor(0x0304, 0x0006);
    write_cmos_sensor(0x0306, 0x00AF);
    write_cmos_sensor(0x0302, 0x0001);
    write_cmos_sensor(0x0300, 0x0005);
    write_cmos_sensor(0x030C, 0x0006);
    write_cmos_sensor(0x030E, 0x0119);
    write_cmos_sensor(0x030A, 0x0001);
    write_cmos_sensor(0x0308, 0x0008);
    write_cmos_sensor(0x0342, 0x16B0);
    write_cmos_sensor(0x0340, 0x0C86);
    write_cmos_sensor(0x0202, 0x0200);
    write_cmos_sensor(0x0200, 0x00C6);
    write_cmos_sensor(0x0B04, 0x0101);
    write_cmos_sensor(0x0B08, 0x0000);
    write_cmos_sensor(0x0B00, 0x0007);
    write_cmos_sensor(0x316A, 0x00A0);
																
    write_cmos_sensor(0x0100, 0x0100);
    //e, 2015/08/27, iAudi
}

/**********************************************************************************************************************/
//$MIPI[Width:4192,Height:3104,Format:RAW10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:1260,useEmbData:0]
//$MV1[MCLK:24,Width:4192,Height:3104,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1260,pvi_pclk_inverse:0]

//=====================================================
// 3M2XXM
// Full Resolution normal Mode
// X_output size : 4192
// Y_output size : 3104
// Frame_rate : 30 fps
// Output_format RAW 10
// Output_lanes 4
// Output_clock_mhz : 1272 Mhz
// System_clock_mhz : 440 Mhz , VT_PIX_clk : 88Mhz
// Input_clock_mhz : 24 Mhz
// TnP R651
//=====================================================
//200
/**********************************************************************************************************************/
static void long_capture_setting(void)
{
	LOG_INF("-- long_capture_setting\n");
		//Page pointer HW
    write_cmos_sensor(0x6028,0x4000);
    write_cmos_sensor(0x0100,0x0000);

	//Mode Setting
    write_cmos_sensor(0x6028,0x2000);
    write_cmos_sensor(0x602A,0x0F74);
    write_cmos_sensor(0x6F12,0x0040);   //64
    write_cmos_sensor(0x6F12,0x0040);   //64
    write_cmos_sensor(0x6028,0x4000);
    write_cmos_sensor(0x0344,0x0008);
    write_cmos_sensor(0x0346,0x0008);
    write_cmos_sensor(0x0348,0x1077);   // 4215
    write_cmos_sensor(0x034A,0x0C37);   // 3127
    write_cmos_sensor(0x034C,0x1070);   // 4208
    write_cmos_sensor(0x034E,0x0C30);   // 3120
    write_cmos_sensor(0x0900,0x0011);
    write_cmos_sensor(0x0380,0x0001);
    write_cmos_sensor(0x0382,0x0001);
    write_cmos_sensor(0x0384,0x0001);
    write_cmos_sensor(0x0386,0x0001);
    write_cmos_sensor(0x0400,0x0000);
    write_cmos_sensor(0x0404,0x0010);
    write_cmos_sensor(0x0114,0x0300);
    write_cmos_sensor(0x0110,0x0002);
    write_cmos_sensor(0x0136,0x1800);   // 24MHz
    write_cmos_sensor(0x0304,0x0006);   //6
    write_cmos_sensor(0x0306,0x00AF);   // 175
    write_cmos_sensor(0x0302,0x0001);   // 1
    write_cmos_sensor(0x0300,0x0005);   // 5
    write_cmos_sensor(0x030C,0x0006);   // 6
    write_cmos_sensor(0x030E,0x0119);   // 281
    write_cmos_sensor(0x030A,0x0001);   // 1
    write_cmos_sensor(0x0308,0x0008);   // 8
    write_cmos_sensor(0x0342,0x16B0);   // 5808
    write_cmos_sensor(0x0340,0x0C86);   // 3206
    write_cmos_sensor(0x0202,0x0200);   // 512
    write_cmos_sensor(0x0200,0x00C6);   // 198
    write_cmos_sensor(0x0B04,0x0101);   // BPC_ON
    write_cmos_sensor(0x0B08,0x0000);   // BPC_OFF
    write_cmos_sensor(0x0B00,0x0007);   // LSC_OFF
    write_cmos_sensor(0x316A,0x00A0);   // OUTIF threshold 
    
  	write_cmos_sensor_8(0x0100,0x01);
}

static void normal_capture_setting(kal_bool stream_on)
{
		//iAudi 2015/08/26
	LOG_INF( "-- normal_capture_setting, chip_id = 0x30C8  stream_on=%d\n", stream_on);
	//Page pointer HW
    write_cmos_sensor(0x6028,0x4000);
    if(imgsensor_info.module_id==0x04)
    	write_cmos_sensor(0x0100,0x0003);  
    else
    	write_cmos_sensor(0x0100,0x0000);
	//Mode Setting
    write_cmos_sensor(0x6028,0x2000);
    write_cmos_sensor(0x602A,0x0F74);
    write_cmos_sensor(0x6F12,0x0040);   //64
    write_cmos_sensor(0x6F12,0x0040);   //64
    write_cmos_sensor(0x6028,0x4000);
    write_cmos_sensor(0x0344,0x0008);
    write_cmos_sensor(0x0346,0x0008);
    write_cmos_sensor(0x0348,0x1077);   // 4215
    write_cmos_sensor(0x034A,0x0C37);   // 3127
    write_cmos_sensor(0x034C,0x1070);   // 4208
    write_cmos_sensor(0x034E,0x0C30);   // 3120
    write_cmos_sensor(0x0900,0x0011);
    write_cmos_sensor(0x0380,0x0001);
    write_cmos_sensor(0x0382,0x0001);
    write_cmos_sensor(0x0384,0x0001);
    write_cmos_sensor(0x0386,0x0001);
    write_cmos_sensor(0x0400,0x0000);
    write_cmos_sensor(0x0404,0x0010);
    write_cmos_sensor(0x0114,0x0300);
    write_cmos_sensor(0x0110,0x0002);
    write_cmos_sensor(0x0136,0x1800);   // 24MHz
    write_cmos_sensor(0x0304,0x0006);   //6
    write_cmos_sensor(0x0306,0x00AF);   // 175
    write_cmos_sensor(0x0302,0x0001);   // 1
    write_cmos_sensor(0x0300,0x0005);   // 5
    write_cmos_sensor(0x030C,0x0006);   // 6
    write_cmos_sensor(0x030E,0x0119);   // 281
    write_cmos_sensor(0x030A,0x0001);   // 1
    write_cmos_sensor(0x0308,0x0008);   // 8
    write_cmos_sensor(0x0342,0x16B0);   // 5808
    write_cmos_sensor(0x0340,0x0C86);   // 3206
    write_cmos_sensor(0x0202,0x0200);   // 512
    write_cmos_sensor(0x0200,0x00C6);   // 198
    write_cmos_sensor(0x0B04,0x0101);   // BPC_ON
    write_cmos_sensor(0x0B08,0x0000);   // BPC_OFF
    write_cmos_sensor(0x0B00,0x0007);   // LSC_OFF
    write_cmos_sensor(0x316A,0x00A0);   // OUTIF threshold 
    // iAudi 
  if(stream_on)
  	write_cmos_sensor_8(0x0100,0x01);
// e bruce, 2014/12/01	
}

/**********************************************************************************************************************/
//$MIPI[Width:4192,Height:3104,Format:RAW10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:1260,useEmbData:0]
//$MV1[MCLK:24,Width:4192,Height:3104,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1260,pvi_pclk_inverse:0]
//=====================================================
// 3M2XXM
// Full Resolution normal Mode
// X_output size : 4192
// Y_output size : 3104
// Frame_rate : 24.07 fps
// Output_format RAW 10
// Output_lanes 4
// Output_clock_mhz : 1272 Mhz
// System_clock_mhz : 352 Mhz , VT_PIX_clk : 88Mhz
// Input_clock_mhz : 24 Mhz
// TnP R651
//=====================================================
/**********************************************************************************************************************/
static void pip_capture_setting(void)
{
	// s bruce, 2014/12/01
	LOG_INF( "--pip_capture_setting, chip_id = 0xD101\n");
}

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
#ifdef VENDOR_EDIT
/*oppo hufeng 20160521 add for cts*/
	normal_capture_setting(1);
#else
	if(currefps==300)
		normal_capture_setting(1);
	else if(currefps==240) // PIP
		pip_capture_setting();
	else
		pip_capture_setting();
#endif
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
#ifndef VENDOR_EDIT
/*zhaozhengtao modify 2016-03-21*/
	video_setting();
#else
    preview_setting(); //modify by liubin for cts test 20150402
#endif
}
/**********************************************************************************************************************/
//$MIPI[Width:688,Height:512,Format:RAW10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:1260,useEmbData:0]
//$MV1[MCLK:24,Width:688,Height:512,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1260,pvi_pclk_inverse:0]

//=====================================================
// 3M2XXM
// 6X6 Binning Normal Mode
// X_output size : 688
// Y_output size : 512
// Frame_rate : 120.07 fps
// Output_format RAW 10
// Output_lanes 4
// Output_clock_mhz : 1272 Mhz
// System_clock_mhz : 440 Mhz
// Input_clock_mhz : 24 Mhz
// TnP R651
//=====================================================
/**********************************************************************************************************************/
static void hs_video_setting(void)
{
	LOG_INF("--hs_video_setting\n");

	//$MV1[MCLK:24,Width:640,Height:480,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
	write_cmos_sensor(0x0100, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0F74);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x00C0);
	write_cmos_sensor(0x0346, 0x0080);
	write_cmos_sensor(0x0348, 0x0FBF);
	write_cmos_sensor(0x034A, 0x0BBF);
	write_cmos_sensor(0x034C, 0x0280);
	write_cmos_sensor(0x034E, 0x01E0);
	write_cmos_sensor(0x0900, 0x0116);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x000B);
	write_cmos_sensor(0x0400, 0x0001);
	write_cmos_sensor(0x0404, 0x0060);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x0002);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x0306, 0x00AF);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0300, 0x0005);
	write_cmos_sensor(0x030C, 0x0006);
	write_cmos_sensor(0x030E, 0x0119);
	write_cmos_sensor(0x030A, 0x0001);
	write_cmos_sensor(0x0308, 0x0008);
	write_cmos_sensor(0x0342, 0x16B0);
	write_cmos_sensor(0x0340, 0x0323);
	write_cmos_sensor(0x0202, 0x0200);
	write_cmos_sensor(0x0200, 0x00C6);
	write_cmos_sensor(0x0B04, 0x0101);
	write_cmos_sensor(0x0B08, 0x0000);
	write_cmos_sensor(0x0B00, 0x0007);
	write_cmos_sensor(0x316A, 0x00A0);
	write_cmos_sensor(0x0100, 0x0100);
	
}
#if 0
/**********************************************************************************************************************/
//$MIPI[Width:1280,Height:720,Format:RAW10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4,DataRate:1260,useEmbData:0]
//$MV1[MCLK:24,Width:1280,Height:720,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1260,pvi_pclk_inverse:0]
//=====================================================
// 3M2XXM
// 3X3 Binning Normal Mode
// X_output size : 1280
// Y_output size : 720
// Frame_rate : 30.06 fps
// Output_format RAW 10
// Output_lanes 4
// Output_clock_mhz : 1272 Mhz
// System_clock_mhz : 440 Mhz
// Input_clock_mhz : 24 Mhz
// TnP R651
//=====================================================
/**********************************************************************************************************************/
static void slim_video_setting(void)
{
	// s bruce, 2014/12/01
	LOG_INF("--slim_video_setting. chip_id = 0xD101");
	write_cmos_sensor(0x6028,0x2000);
	write_cmos_sensor(0x602A,0x14F0);
	write_cmos_sensor(0x6F12,0x0040);
	write_cmos_sensor(0x6F12,0x0040);
	write_cmos_sensor(0x6028,0x4000);
	write_cmos_sensor(0x0344,0x00BC);
	write_cmos_sensor(0x0346,0x01DC);
	write_cmos_sensor(0x0348,0x0FC3);
	write_cmos_sensor(0x034A,0x0A63);
	write_cmos_sensor(0x034C,0x0500);
	write_cmos_sensor(0x034E,0x02D0);
	write_cmos_sensor_8(0x0901,0x13);
	write_cmos_sensor(0x0380,0x0001);
	write_cmos_sensor(0x0382,0x0001);
	write_cmos_sensor(0x0384,0x0001);
	write_cmos_sensor(0x0386,0x0005);
	write_cmos_sensor(0x0400,0x0001);
	write_cmos_sensor(0x0404,0x0030);
	write_cmos_sensor_8(0x0114,0x03);
	write_cmos_sensor_8(0x0111,0x02);
	write_cmos_sensor(0x112C,0x0000);
	write_cmos_sensor(0x112E,0x0000);
	write_cmos_sensor(0x0136,0x1800);
	write_cmos_sensor(0x0304,0x0006);
	write_cmos_sensor(0x0306,0x006E);
	write_cmos_sensor(0x0302,0x0001);
	write_cmos_sensor(0x0300,0x0004);
	write_cmos_sensor(0x030C,0x0004);
	write_cmos_sensor(0x030E,0x006A);
	write_cmos_sensor(0x030A,0x0001);
	write_cmos_sensor(0x0308,0x0008);
	write_cmos_sensor(0x0342,0x11F0);
	write_cmos_sensor(0x0340,0x0C74);
	write_cmos_sensor(0x0202,0x0200);
	write_cmos_sensor(0x0200,0x0400);
	write_cmos_sensor_8(0x0B05,0x01);
	write_cmos_sensor_8(0x0B08,0x00);
	write_cmos_sensor_8(0x0B00,0x00);
	write_cmos_sensor_8(0x3B3C,0x01);
	write_cmos_sensor(0x3B34,0x3030);
	write_cmos_sensor(0x3B36,0x3030);
	write_cmos_sensor(0x3B38,0x3030);
	write_cmos_sensor(0x3B3A,0x3030);
	write_cmos_sensor(0x306A,0x0068);

	write_cmos_sensor_8(0x0100,0x01);
	write_cmos_sensor_8(0x0100,0x01);		
}
#endif
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
    kal_uint8 retry = 1;
	char data[1] = {0};
    
    /*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
			write_cmos_sensor(0x602C,0x4000);
			write_cmos_sensor(0x602E,0x0000);
			*sensor_id = read_cmos_sensor(0x6F12);
			//*sensor_id = imgsensor_info.sensor_id;
            if (*sensor_id == imgsensor_info.sensor_id) {   
				selective_read_eeprom_3l8(0,data);
				imgsensor_info.module_id = data[0];
#ifdef VENDOR_EDIT
/* Add by LiuBin for register device info at 20160616 */
                if(deviceInfo_register_value==0x00){
                    register_imgsensor_deviceinfo("Cam_b", DEVICE_VERSION_S5K3L8, imgsensor_info.module_id);
                    deviceInfo_register_value=0x01;
                }
/*oppo hufeng 20160523 add to speed up open camera time when first open after power on phone*/
				read_otp_static_array();
#endif
                printk("i2c write id: 0x%x, sensor id: 0x%x module_id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id,data[0]);
				
                if(imgsensor_info.module_id==0x04)
                	imgsensor_info.sensor_output_dataformat=SENSOR_OUTPUT_FORMAT_RAW_Gb;
				
                return ERROR_NONE;
            }   
            LOG_INF("[s5k3l8 get_imgsensor_id] Read sensor id fail, write id: 0x%x, sensor id = 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
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
#ifndef VENDOR_EDIT //lanhe add
#ifdef USE_KTHREAD
    if(!vendor_kthread)
    {
    	sema_init(&vendor_sem, 0);	
    	sema_init(&close_sem,0);
    	vendor_init_task = kthread_create(vendor_init_kthread, NULL, "vendor_kthread");
    	LOG_INF("create thread and wake up thread\n");
    	wake_up_process(vendor_init_task);
    	vendor_kthread = KAL_TRUE;
    }
#endif
#endif 
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
	kal_uint16 sensor_id = 0; 
	LOG_1;
	LOG_2;
	
#ifndef VENDOR_EDIT //lanhe add
#ifdef USE_KTHREAD
    if(!vendor_kthread)
    {
    	sema_init(&vendor_sem, 0);	
    	sema_init(&close_sem,0);
    	vendor_init_task = kthread_create(vendor_init_kthread, NULL, "vendor_kthread"); 
    	wake_up_process(vendor_init_task);
    	vendor_kthread = KAL_TRUE;
    }
#endif
#endif
	//sensor have two i2c address 0x5a 0x5b & 0x21 0x20, we should detect the module used i2c address
	    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
			write_cmos_sensor(0x602C,0x4000);
			write_cmos_sensor(0x602E,0x0000);
            sensor_id =  read_cmos_sensor(0x6F12);
			//sensor_id = imgsensor_info.sensor_id;
            if (sensor_id == imgsensor_info.sensor_id) {                
                LOG_INF("[s5k3l8 open] i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);   
                break;
            }   
            LOG_INF("[s5k3l8 open] Read sensor id fail, write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
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
#ifndef VENDOR_EDIT //lanhe add
    sensor_init();
#else
    if(!vendor_init_task){
    	LOG_INF("sensor_init, sensor id: 0x%x\n", sensor_id);
    	sensor_init();
    }else{
    	LOG_INF("vendor_sem, sensor id: 0x%x\n", sensor_id);
    	sema_init(&close_sem,0);
    	up(&vendor_sem);
    }
#endif
	
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



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
	if(bNeedSetNormalMode){
		LOG_INF("change to normal mode\n");
		write_cmos_sensor(0x6028, 0x4000);
		write_cmos_sensor(0x0100, 0x0000);
		write_cmos_sensor(0x0306, 0x00AF);//c6c
		write_cmos_sensor(0x0302, 0x0001);
		write_cmos_sensor(0x030E, 0x0119);
		write_cmos_sensor(0x3008, 0x0000);
						
		write_cmos_sensor(0x31E4, 0x02BC);
		write_cmos_sensor(0x0104, 0x0100);	
		write_cmos_sensor(0x0202, 0x0200);
		write_cmos_sensor(0x0342, 0x16B0);
		write_cmos_sensor(0x0340, 0x0CB2);
		write_cmos_sensor(0x316A, 0x00A0);
						
		write_cmos_sensor(0x304A, 0x0000);
		write_cmos_sensor(0x0104, 0x0000);
		write_cmos_sensor(0x0100,0x0100);
		bNeedSetNormalMode = KAL_FALSE;
	}
	LOG_INF("E\n");
#ifdef VENDOR_EDIT //lanhe add
    if(ms_work_mode)
    	down(&close_sem);
    hw_sensor_mode = IMGSENSOR_MODE_INIT;
#endif

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
	
#ifdef VENDOR_EDIT //lanhe add
    while(ms_work_mode){
    	mdelay(10);
    }
    hw_sensor_mode = IMGSENSOR_MODE_PREVIEW;
#endif
	preview_setting(); //modify by liubin for cts test 20150402
	//normal_capture_setting();//oppo customize
	
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
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
    {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
    {
		if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);   
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 

	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
#ifndef VENDOR_EDIT //lanhe add
    capture_setting(imgsensor.current_fps); 
#else
    while(ms_work_mode){
    	mdelay(10);
    }
    if(hw_sensor_mode != IMGSENSOR_MODE_CAPTURE){
    	capture_setting(imgsensor.current_fps); 
    }else{
    	write_cmos_sensor_8(0x0100,0x01);
    }
#endif

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
#ifdef VENDOR_EDIT //lanhe add
    while(ms_work_mode){
    	mdelay(10);
    }
    hw_sensor_mode = IMGSENSOR_MODE_VIDEO;
#endif
	normal_video_setting(imgsensor.current_fps);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
#ifdef VENDOR_EDIT //lanhe add
    while(ms_work_mode){
    	mdelay(10);
    }
    hw_sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
#endif	
	hs_video_setting();
	return ERROR_NONE;
}	/*	hs_video   */


static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
#ifdef VENDOR_EDIT //lanhe add
    hw_sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
#endif
	//set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}

	
/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
*   This function start the sensor Custom1.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    //preview_setting();
#ifdef VENDOR_EDIT //lanhe add
    hw_sensor_mode = IMGSENSOR_MODE_CUSTOM1;
#endif
    long_capture_setting();
    return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	// preview_setting();
	 normal_capture_setting(1);//oppo customize
    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	// preview_setting();
	 normal_capture_setting(1);//oppo customize
    return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	// preview_setting();
	 normal_capture_setting(1);//oppo customize
    return ERROR_NONE;
}   /*  Custom4   */


static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	// preview_setting();
	 normal_capture_setting(1);//oppo customize
    return ERROR_NONE;
}   /*  Custom5   */
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E");
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
    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;

    sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;

    sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height     = imgsensor_info.custom4.grabwindow_height;

    sensor_resolution->SensorCustom5Width  = imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height     = imgsensor_info.custom5.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d", scenario_id);

	
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
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame; 
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame; 
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame; 
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame; 
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame; 

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	/*modify by liubin at 20150326*/
	if (get_boot_mode() == FACTORY_BOOT)
		sensor_info->PDAF_Support = 0;
	else
		sensor_info->PDAF_Support = 1;
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
        case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx; 
            sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;   
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

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
	LOG_INF("%s:scenario_id = %d", __func__,scenario_id);
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
        case MSDK_SCENARIO_ID_CUSTOM1:
            Custom1(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            Custom3(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            Custom4(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            Custom5(image_window, sensor_config_data); // Custom1
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
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) 	  
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
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			//set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();            
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
           // set_dummy();            
            break; 
        case MSDK_SCENARIO_ID_CUSTOM3:
            frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();            
            break; 
        case MSDK_SCENARIO_ID_CUSTOM4:
            frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();            
            break; 
        case MSDK_SCENARIO_ID_CUSTOM5:
            frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();			
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();	
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
        case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *framerate = imgsensor_info.custom3.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *framerate = imgsensor_info.custom4.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *framerate = imgsensor_info.custom5.max_framerate;
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
        write_cmos_sensor(0x0600, 0x0002);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0600, 0x0000);
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

    SET_PD_BLOCK_INFO_T *PDAFinfo;	
	
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
    
	unsigned long long *feature_data=(unsigned long long *) feature_para;
	//UINT32 *feature_return_para = (UINT32 *) feature_para;
    	
 
	LOG_INF("%s:feature_id = %d", __func__,feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:	 
			LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;		   
		case SENSOR_FEATURE_SET_ESHUTTER:
			write_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		case SENSOR_FEATURE_SET_GAIN:		
			set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			if((sensor_reg_data->RegData>>8)>0)
			   write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			else
				write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
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
			//get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(*(feature_data+1)));
			get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_GET_PDAF_DATA:	
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			//zhangkw del
			//read_3m2_eeprom((kal_uint16 )(*feature_data_32),(char*)(uintptr_t)(*(feature_data_32+1)),(kal_uint32)(*(feature_data_32+2)));
			read_3l8_eeprom(0x1400,(char*)(uintptr_t)(*(feature_data+1)),0x1A67-0x1400);
			//memcpy((char*)(uintptr_t)(*(feature_data+1)),pdaf_data,(kal_uint32)(*(feature_data+2)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing			 
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;							 
			break;				
		case SENSOR_FEATURE_SET_FRAMERATE:
			LOG_INF("current fps :%d\n", *feature_data);
			spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);	
			break;
		case SENSOR_FEATURE_SET_HDR:
			//LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_16);
			LOG_INF("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = (BOOL)*feature_data_16;
            imgsensor.ihdr_en = KAL_FALSE;
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
		case SENSOR_FEATURE_GET_PDAF_INFO:
			LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT32)*feature_data);
			PDAFinfo= (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		
			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW://oppo customize
					memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
				//case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					break;
			}
			break;
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
            LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT32)*feature_data);
            //PDAF capacity enable or not, 2p8 only full size support PDAF
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
                    break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; // video & capture use same setting
                    break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
				default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
					break;
			}
            break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR is no support");
		break;
		default:
			break;
	}
	return ERROR_NONE;
}	/*	feature_control()  */

#ifdef VENDOR_EDIT //lanhe add
#ifdef USE_KTHREAD
static int vendor_init_kthread(void *data)
{
		//int hpd_result = 0;
        struct sched_param param = { .sched_priority = 30 };
        sched_setscheduler(current, SCHED_RR, &param);
		sema_init(&vendor_sem, 0);	
        for( ;; ) 
        {
			down(&vendor_sem);
			sema_init(&close_sem, 0);
			ms_work_mode = true;
			printk("%s:E",__func__);
			//do async sensor ctl
			/*oppo hufeng 20160107 add to solve eeprom and sensor conflict*/
			mutex_lock(&sensor_eeprom_lock);
		       sensor_init();
			normal_capture_setting(KAL_FALSE);
			/*oppo hufeng 20160107 add to solve eeprom and sensor conflict*/
			mutex_unlock(&sensor_eeprom_lock);
			hw_sensor_mode = IMGSENSOR_MODE_CAPTURE;
			printk("%s:X",__func__);
			ms_work_mode = false;
			up(&close_sem);
        }

        return 0;
}
#endif
#endif


static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K3L8_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	S5K3L8_MIPI_RAW_SensorInit	*/
