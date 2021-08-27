/*
NOTE:
The modification is appended to initialization of image sensor. 
After sensor initialization, use the function
bool otp_update_wb(unsigned char golden_rg, unsigned char golden_bg)
and
bool otp_update_lenc(void)
and
then the calibration of AWB & LSC & BLC will be applied. 
After finishing the OTP written, we will provide you the typical value of golden sample.
*/

#include <linux/videodev2.h>    
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
//#include <asm/system.h>

#include <linux/xlog.h>

#include <mach/mt_boot.h>
#include <linux/slab.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov8865rmipiraw_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "OV8865ROTP"
#define LOG_1 SENSORDB("OV8865R,MIPI CAM\n")
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
/****************************   Modify end    *******************************************/

//#include "ov8865rmipiraw_Camera_Sensor_para.h"
//#include "ov8865rmipiraw_CameraCustomized.h"

//extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
//#define write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV8865RMIPI_WRITE_ID)
//extern kal_uint16 read_cmos_sensor(kal_uint32 addr);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId); //add liuchrg
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId); //add liuchrg

static void DPCFuncEnable(void);
static void DPCFuncDisable(void);
static void otp_apply_lenc(void);
static bool CheckOtpValid(void);

//#define SUPPORT_FLOATING
#define USHORT        unsigned short
#define BYTE          unsigned char


#define GAIN_RH_ADDR          0x5018
#define GAIN_RL_ADDR          0x5019
#define GAIN_GH_ADDR          0x501A
#define GAIN_GL_ADDR          0x501B
#define GAIN_BH_ADDR          0x501C
#define GAIN_BL_ADDR          0x501D

#define GAIN_DEFAULT_VALUE    0x0400 // 1x gain

#ifndef OPPO_CMCC_TEST
/*lingjianing 2016-05-25 modify for CMCC test*/
#define TrulyGoldenRG         0xF4
#define TrulyGoldenBG         0x11D
#else
#define TrulyGoldenRG         0xFA//0xF5
#define TrulyGoldenBG         0x125//0x123
#endif

#define OfilmGoldenRG         0x118
#define OfilmGoldenBG         0x147

#define QtechGoldenRG         0xF8
#define QtechGoldenBG         0x120


#define OTP_MID               0x02

// R/G and B/G of current camera module
static unsigned short rg_ratio = 0;
static unsigned short bg_ratio = 0;

static char AF[5]={0};


enum OV8865R_OTP_ADDR
{
	OV8865R_ADDR_OTP_WRITE_CTRL = 0x3D80,	
	OV8865R_ADDR_OTP_READ_CTRL = 0x3D81,	
	OV8865R_ADDR_OTP_MODE_CTRL = 0x3D84,	
	
	OV8865R_ADDR_OTP_START_H = 0x3D88,	
	OV8865R_ADDR_OTP_START_L = 0x3D89,	
	OV8865R_ADDR_OTP_END_H = 0x3D8A,	
	OV8865R_ADDR_OTP_END_L = 0x3D8B,	
};

enum OV8865R_OTP_DATA
{
	OV8865R_DATA_WRITE_ENABLE = 0x01,
	OV8865R_DATA_WRITE_DISABLE = 0x00,
	
	OV8865R_DATA_READ_ENABLE = 0x01,
	OV8865R_DATA_READ_DISABLE = 0x00,
	
	OV8865R_DATA_MODE_WRITE = 0x40,
	OV8865R_DATA_MODE_READ = 0xC0,
};

#define OTP_OFFSET 0 //240

enum OV8865R_REG_ADDR
{	
	OV8865R_ADDR_LSC_CTRL = 0x5000,	
	OV8865R_ADDR_LSC_DATA = 0x5800,	
	
	OV8865R_ADDR_OTP_DATA = 0x7010 + OTP_OFFSET,

	OV8865R_ADDR_OTP_BASIC_FLAG = 0x7010 + OTP_OFFSET,	
	OV8865R_ADDR_OTP_BASIC_DATA = 0x7011 + OTP_OFFSET,
	
	OV8865R_ADDR_OTP_BASIC_3000k_FLAG = 0x703E + OTP_OFFSET,	

	OV8865R_ADDR_OTP_LENC_FLAG = 0x7070 + OTP_OFFSET,	
	OV8865R_ADDR_OTP_LENC_DATA = 0x7071 + OTP_OFFSET,
};

enum OV8865R_SEGMENT_FLAG
{
	OV8865R_SEGMENT_FLAG_EMPTY   = 0x00,
	OV8865R_OFILM_FLAG_VALID     = 0x01,
	OV8865R_SEGMENT_FLAG_VALID   = 0x40,
	OV8865R_SEGMENT_FLAG_INVALID = 0xc0,
};

enum OV8865R_OTP_SIZE
{
	OV8865R_OTP_USAGE_SIZE = 704,

	OV8865R_OTP_GROUP_SIZE = 112,
	OV8865R_BASIC_GROUP_SIZE = 48,
	OV8865R_LENC_GROUP_SIZE = 64,

	OV8865R_BASIC_SEGMENT_SIZE = 48,
	OV8865R_LENC_SEGMENT_SIZE = 64,

	OV8865R_LSC_SIZE = 62,
};

typedef union OV8865R_OtpGroupUnion
{
  BYTE data[82];
    struct
    {
        union
		{
			BYTE dataBasicInfo[9];
			
			struct
			{
			    //BYTE flags;
    			BYTE Module_ID;
                BYTE Lens_ID;
                BYTE Vcm_ID;
                BYTE DriverIc_ID;
                BYTE Sensor_ID;
                BYTE year;
                BYTE month;
                BYTE day;
                BYTE infochecksum;
            };
        };

       	union
		{
			BYTE dataBasicWB[6];
			
			struct
			{
    			BYTE RG_MSB;
                BYTE BG_MSB;
                BYTE Light_RG_MSB;
                BYTE Light_BG_MSB;
                BYTE AWB_LSB;
                BYTE awbchecksum;
            };
        };
    

       	union
		{
			BYTE dataBasicAF[4];
			
			struct
			{
    			BYTE VCM_START_MSB;
                BYTE VCM_END_MSB;
                BYTE VCM_LSB;
                BYTE afchecksum;
            };
        };

        union
		{
			BYTE dataLenc[63];
			
			struct
			{
				BYTE lenc[62];	
                BYTE lencchecksum;
			};
		};
        
    };
  
} OV8865R_OtpData;

OV8865R_OtpData m_readGroup;


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, 0x20);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, 0x20);
}

static void i2c_write(USHORT reg, USHORT val)
{
	write_cmos_sensor(reg, val);
}

static void i2c_read(USHORT reg, USHORT *pval)
{
	*pval = read_cmos_sensor(reg);	
}

static void ReadBegin(void)
{
	DPCFuncDisable();
}

static void ReadEnd(void)
{
	DPCFuncEnable();
}

static void ReadEnable(void)
{
	i2c_write(OV8865R_ADDR_OTP_READ_CTRL, OV8865R_DATA_READ_ENABLE);
	mdelay(20);
}

static void ReadDisable(void)
{
	i2c_write(OV8865R_ADDR_OTP_READ_CTRL, OV8865R_DATA_READ_DISABLE);
}

static void ReadMultiData(USHORT addr, BYTE* data, int cnt)
{
    int i,j;
	ReadBegin();

	i2c_write(OV8865R_ADDR_OTP_MODE_CTRL, OV8865R_DATA_MODE_READ);
	i2c_write(OV8865R_ADDR_OTP_START_H, addr>>8);
	i2c_write(OV8865R_ADDR_OTP_START_L, addr&0xFf);
	i2c_write(OV8865R_ADDR_OTP_END_H, (addr+cnt-1)>>8);
	i2c_write(OV8865R_ADDR_OTP_END_L, (addr+cnt-1)&0xFF);
	
	ReadEnable();
	
	for (i=0; i<cnt; i++)
	{
		USHORT tmp = 0xFF;
		i2c_read(addr+i, &tmp);
		data[i] = (BYTE)(tmp & 0xFF);
	}
	
	ReadDisable();
		
	for (j=0; j<cnt; j++)
	{
		i2c_write(addr+j, 0);
	}

	ReadEnd();
}

static void DPCFuncEnable(void)
{
	USHORT ctrl;
	i2c_read(0x5002, &ctrl);
	i2c_write(0x5002, ctrl | 0x08);
	mdelay(10);
}

static void DPCFuncDisable(void)
{
	USHORT ctrl;
	i2c_read(0x5002, &ctrl);
	i2c_write(0x5002, ctrl & (~0x08));
}

#ifdef SUPPORT_FLOATING //Use this if support floating point values
/*******************************************************************************
* Function    :  otp_apply_wb
* Description :  Calcualte and apply R, G, B gain to module
* Parameters  :  [in] golden_rg : R/G of golden camera module
                 [in] golden_bg : B/G of golden camera module
* Return      :  1, success; 0, fail
*******************************************************************************/	
static bool otp_apply_wb(unsigned short golden_rg, unsigned short golden_bg)
{
	unsigned short gain_r = GAIN_DEFAULT_VALUE;
	unsigned short gain_g = GAIN_DEFAULT_VALUE;
	unsigned short gain_b = GAIN_DEFAULT_VALUE;

	double ratio_r, ratio_g, ratio_b;
	double cmp_rg, cmp_bg;

	if (!golden_rg || !golden_bg)
	{
		LOG_INF("golden_rg / golden_bg can not be zero\n");
		return 0;
	}

	// Calcualte R, G, B gain of current module from R/G, B/G of golden module
        // and R/G, B/G of current module
	cmp_rg = 1.0 * rg_ratio / golden_rg;
	cmp_bg = 1.0 * bg_ratio / golden_bg;

	if ((cmp_rg<1) && (cmp_bg<1))
	{
		// R/G < R/G golden, B/G < B/G golden
		ratio_g = 1;
		ratio_r = 1 / cmp_rg;
		ratio_b = 1 / cmp_bg;
	}
	else if (cmp_rg > cmp_bg)
	{
		// R/G >= R/G golden, B/G < B/G golden
		// R/G >= R/G golden, B/G >= B/G golden
		ratio_r = 1;
		ratio_g = cmp_rg;
		ratio_b = cmp_rg / cmp_bg;
	}
	else
	{
		// B/G >= B/G golden, R/G < R/G golden
		// B/G >= B/G golden, R/G >= R/G golden
		ratio_b = 1;
		ratio_g = cmp_bg;
		ratio_r = cmp_bg / cmp_rg;
	}

	// write sensor wb gain to registers
	// 0x0400 = 1x gain
	if (ratio_r != 1)
	{
		gain_r = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_r);
		write_cmos_sensor(GAIN_RH_ADDR, gain_r >> 6);
		write_cmos_sensor(GAIN_RL_ADDR, gain_r & 0x003f);
	}

	if (ratio_g != 1)
	{
		gain_g = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_g);
		write_cmos_sensor(GAIN_GH_ADDR, gain_g >> 6);
		write_cmos_sensor(GAIN_GL_ADDR, gain_g & 0x003f);
	}

	if (ratio_b != 1)
	{
		gain_b = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_b);
		write_cmos_sensor(GAIN_BH_ADDR, gain_b >> 6);
		write_cmos_sensor(GAIN_BL_ADDR, gain_b & 0x003f);
	}

	LOG_INF("cmp_rg=%f, cmp_bg=%f\n", cmp_rg, cmp_bg);
	LOG_INF("ratio_r=%f, ratio_g=%f, ratio_b=%f\n", ratio_r, ratio_g, ratio_b);
	LOG_INF("gain_r=0x%x, gain_g=0x%x, gain_b=0x%x\n", gain_r, gain_g, gain_b);
	return 1;
}

#else //Use this if not support floating point values

#define OTP_MULTIPLE_FAC	10000
static bool otp_apply_wb(unsigned short golden_rg, unsigned short golden_bg)
{
	unsigned short gain_r = GAIN_DEFAULT_VALUE;
	unsigned short gain_g = GAIN_DEFAULT_VALUE;
	unsigned short gain_b = GAIN_DEFAULT_VALUE;

	unsigned short ratio_r, ratio_g, ratio_b;
	unsigned short cmp_rg, cmp_bg;

	if (!golden_rg || !golden_bg)
	{
		LOG_INF("golden_rg / golden_bg can not be zero\n");
		return 0;
	}

	// Calcualte R, G, B gain of current module from R/G, B/G of golden module
    // and R/G, B/G of current module
	cmp_rg = OTP_MULTIPLE_FAC * rg_ratio / golden_rg;
	cmp_bg = OTP_MULTIPLE_FAC * bg_ratio / golden_bg;

	if ((cmp_rg < 1 * OTP_MULTIPLE_FAC) && (cmp_bg < 1 * OTP_MULTIPLE_FAC))
	{
		// R/G < R/G golden, B/G < B/G golden
		ratio_g = 1 * OTP_MULTIPLE_FAC;
		ratio_r = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_rg;
		ratio_b = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_bg;
	}
	else if (cmp_rg > cmp_bg)
	{
		// R/G >= R/G golden, B/G < B/G golden
		// R/G >= R/G golden, B/G >= B/G golden
		ratio_r = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_rg;
		ratio_b = OTP_MULTIPLE_FAC * cmp_rg / cmp_bg;
	}
	else
	{
		// B/G >= B/G golden, R/G < R/G golden
		// B/G >= B/G golden, R/G >= R/G golden
		ratio_b = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_bg;
		ratio_r = OTP_MULTIPLE_FAC * cmp_bg / cmp_rg;
	}

	// write sensor wb gain to registers
	// 0x0400 = 1x gain
	if (ratio_r != 1 * OTP_MULTIPLE_FAC)
	{
		gain_r = GAIN_DEFAULT_VALUE * ratio_r / OTP_MULTIPLE_FAC;
		write_cmos_sensor(GAIN_RH_ADDR, gain_r >> 6);
		write_cmos_sensor(GAIN_RL_ADDR, gain_r & 0x003f);
	}

	if (ratio_g != 1 * OTP_MULTIPLE_FAC)
	{
		gain_g = GAIN_DEFAULT_VALUE * ratio_g / OTP_MULTIPLE_FAC;
		write_cmos_sensor(GAIN_GH_ADDR, gain_g >> 6);
		write_cmos_sensor(GAIN_GL_ADDR, gain_g & 0x003f);
	}

	if (ratio_b != 1 * OTP_MULTIPLE_FAC)
	{
		gain_b = GAIN_DEFAULT_VALUE * ratio_b / OTP_MULTIPLE_FAC;
		write_cmos_sensor(GAIN_BH_ADDR, gain_b >> 6);
		write_cmos_sensor(GAIN_BL_ADDR, gain_b & 0x003f);
	}
    LOG_INF("rg_ratio=0x%x, bg_ratio=0x%x\n", rg_ratio, bg_ratio);
	LOG_INF("cmp_rg=%d, cmp_bg=%d\n", cmp_rg, cmp_bg);
	LOG_INF("ratio_r=%d, ratio_g=%d, ratio_b=%d\n", ratio_r, ratio_g, ratio_b);
	LOG_INF("gain_r=0x%x, gain_g=0x%x, gain_b=0x%x\n", gain_r, gain_g, gain_b);
	return 1;
}
#endif /* SUPPORT_FLOATING */


static bool CheckOtpValid(void)
{	
    int validGroup  = 0;
	int validGroup_wb = 0;
    int validGroup_af = 0; 
	int validGroup_lsc = 0;
	int i;
    BYTE flag_info,flag_wb,flag_af,flag_lsc;
	
	
		ReadMultiData(0x7010, &flag_info, 1);
		LOG_INF("Basic Group flag %x", flag_info);
		if (0x40==(flag_info&0xc0))
		{
		    ReadMultiData(0x7011, &m_readGroup.dataBasicInfo[0], 9);
			LOG_INF("read Basic info Group 1");
		}
		else if (0x10==(flag_info&0x30))
		{
		    ReadMultiData(0x701a, &m_readGroup.dataBasicInfo[0], 9);
			LOG_INF("read Basic info Group 2");
		}
		else if (0x04==(flag_info&0x0c))
		{
		    ReadMultiData(0x7023, &m_readGroup.dataBasicInfo[0], 9);
			LOG_INF("read Basic info Group 3");
		}
		else
		{
			LOG_INF("Basic Group info Invalid");
            validGroup = -1;
		}


        ReadMultiData(0x702c, &flag_wb, 1);
        LOG_INF("wb Group flag %x", flag_wb);
        if (0x40==(flag_wb&0xc0))
        {
            ReadMultiData(0x702d, &m_readGroup.dataBasicWB[0], 6);
            LOG_INF("read wb info Group 1");
        }
        else if (0x10==(flag_wb&0x30))
        {
            ReadMultiData(0x7033, &m_readGroup.dataBasicWB[0], 6);
            LOG_INF("read wb info Group 2");
        }
        else if (0x04==(flag_wb&0x0c))
        {
            ReadMultiData(0x7039, &m_readGroup.dataBasicWB[0], 6);
            LOG_INF("read wb info Group 3");
        }
        else
        {
            LOG_INF("Basic wb info Invalid");
            validGroup_wb = -1;
        }


        ReadMultiData(0x7040, &flag_af, 1);
        LOG_INF("AF Group flag %x", flag_af);
        if (0x40==(flag_af&0xc0))
        {
            ReadMultiData(0x7041, &m_readGroup.dataBasicAF[0], 4);
            LOG_INF("read AF info Group 1");
        }
        else if (0x10==(flag_af&0x30))
        {
            ReadMultiData(0x7045, &m_readGroup.dataBasicAF[0], 4);
            LOG_INF("read AF info Group 2");
        }
        else if (0x04==(flag_af&0x0c))
        {
            ReadMultiData(0x7049, &m_readGroup.dataBasicAF[0], 4);
            LOG_INF("read AF info Group 3");
        }
        else
        {
            LOG_INF("Basic AF info Invalid");
            validGroup_af = -1;
        }

        ReadMultiData(0x704D, &flag_lsc, 1);
        LOG_INF("LSC Group flag %x", flag_lsc);
        if (0x40==(flag_lsc&0xc0))
        {
            ReadMultiData(0x704E, &m_readGroup.dataLenc[0], 63);
            LOG_INF("read LSC info Group 1");
        }
        else if (0x10==(flag_lsc&0x30))
        {
            ReadMultiData(0x708D, &m_readGroup.dataLenc[0], 63);
            LOG_INF("read LSC info Group 2");
        }
        else if (0x04==(flag_lsc&0x0c))
        {
            ReadMultiData(0x70CC, &m_readGroup.dataLenc[0], 63);
            LOG_INF("read LSC info Group 3");
        }
        else
        {
            LOG_INF("Basic LSC info Invalid");
            validGroup_af = -1;
        }

	if (validGroup == -1 ||validGroup_wb == -1||validGroup_af == -1|| validGroup_lsc == -1)
	{
		return 0;
	}
    LOG_INF("MID = %x,LenID = %x,VcmID = %x,DriverIc = %x,SensorID = %x",
            m_readGroup.Module_ID,m_readGroup.Lens_ID,m_readGroup.Vcm_ID,
            m_readGroup.DriverIc_ID,m_readGroup.Sensor_ID);

	return 1;
}

int ov8865r_get_otp_af(char * af)
{
    if (NULL==af) {
        return ERROR_INVALID_PARA;
    }
  	if (((AF[0]<<2)|AF[1]+110)>=((AF[2]<<2)|AF[3])) {
        LOG_INF("read SEMCO module af otp failed,set inf0xA7,mcro0x1FC\n");
        AF[0]=0x00;
        AF[1]=0xD8;
        AF[2]=0x01;
        AF[3]=0xCC;
   }
   memcpy(af,AF,5);
   return ERROR_NONE;
}

static bool ov8865r_read_otp_af(void)
{
    //int temp;
    AF[0] = m_readGroup.VCM_START_MSB;
    AF[1] = (m_readGroup.VCM_LSB&0xc0)>>6;
    AF[2] = m_readGroup.VCM_END_MSB;
    AF[3] = (m_readGroup.VCM_LSB&0x30)>>4;
    AF[4] = m_readGroup.Module_ID;
    LOG_INF("VCM_START_MSB = %x,VCM_END_MSB =%x,VCM_LSB =%x",m_readGroup.VCM_START_MSB,m_readGroup.VCM_END_MSB,m_readGroup.VCM_LSB);
	LOG_INF("ov8865r af data:inf_h=0x%x, inf_l=0x%x,macro_h=0x%x,macro_l=0x%x,vendor id =%x\n",AF[0],AF[1],AF[2],AF[3],AF[4]);
    if(AF[0]==0&&AF[1]==0&&AF[2]==0&&AF[3]==0){
	   LOG_INF("read AF otp fail");
	   return 0;
	}
 
    return 1;
}


/*******************************************************************************
* Function    :  otp_apply_lenc
* Description :  Apply lens correction setting to module
* Parameters  :  none
* Return      :  none
*******************************************************************************/	
static void otp_apply_lenc(void)
{
	// write lens correction setting to registers

	unsigned char i,temp;
	LOG_INF("apply lenc setting\n");
	for (i=0; i<OV8865R_LSC_SIZE; i++)
	{
		write_cmos_sensor(OV8865R_ADDR_LSC_DATA+i, m_readGroup.lenc[i]);
		LOG_INF("otp_apply_lenc:0x%x, 0x%x\n", OV8865R_ADDR_LSC_DATA+i, m_readGroup.lenc[i]);
	}
	
	//Enable LSC
	temp = read_cmos_sensor(OV8865R_ADDR_LSC_CTRL);
	temp = temp | 0x80;
	write_cmos_sensor(OV8865R_ADDR_LSC_CTRL, temp);
	LOG_INF("otp_apply_lenc:0x%x\n", temp);
}

/*******************************************************************************
* Function    :  otp_update_wb
* Description :  Update white balance settings from OTP
* Parameters  :  [in] golden_rg : R/G of golden camera module
                 [in] golden_bg : B/G of golden camera module
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool ov8865r_otp_update(void) 
{
	unsigned short golden_rg, golden_bg;
	unsigned short rg, bg;
    unsigned int   irg,irb;
	
	if (CheckOtpValid() == 0)
	{
			LOG_INF("otp update failed\n");
			return 0;
	}
	
	LOG_INF("start wb update\n");	
	
	rg = (USHORT)m_readGroup.RG_MSB * 4 + ((m_readGroup.AWB_LSB&0xc0) >> 6);
	bg = (USHORT)m_readGroup.BG_MSB * 4 + ((m_readGroup.AWB_LSB&0x30) >> 4);
	
	rg_ratio = rg;
	bg_ratio = bg;

    LOG_INF("r/g = %x, b/g = %x\n",rg_ratio,bg_ratio);
	if(m_readGroup.Module_ID == 0x02)
	{
        golden_rg = TrulyGoldenRG;
        golden_bg = TrulyGoldenBG;
        LOG_INF("it is truely module\n");
	}
	else if(m_readGroup.Module_ID == 0x06)
	{
        golden_rg = OfilmGoldenRG;
        golden_bg = OfilmGoldenBG;
        LOG_INF("it is ofilm module\n");
	}
    else if(m_readGroup.Module_ID == 0x05)
	{
        golden_rg = QtechGoldenRG;
        golden_bg = QtechGoldenBG;
        LOG_INF("it is QTECH module\n");
	}
	else
	{
        golden_rg = 0;
        golden_bg = 0;
	}
    ov8865r_read_otp_af();
	otp_apply_wb(golden_rg, golden_bg);
	
	LOG_INF("wb update finished\n");

	LOG_INF("start lenc update\n");
	otp_apply_lenc();
	LOG_INF("lenc update finished\n");

	return 1;
}

#ifdef VENDOR_EDIT
/* Add by LiuBin for register device info at 20160616 */
unsigned char get_module_id_ov8865r()
{
    unsigned char module_id = 0;
    write_cmos_sensor(0x0103,0x01);// ; software reset
	mdelay(10);
	write_cmos_sensor(0x0100,0x01);// ; wake up, streaming
    if (1 == CheckOtpValid())
    {
        module_id = m_readGroup.Module_ID;
    }
    printk("ov8865r module id = 0x%x \n", module_id);
    return module_id;
}
#endif
