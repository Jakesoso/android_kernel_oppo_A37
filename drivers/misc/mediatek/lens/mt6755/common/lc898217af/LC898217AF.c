/*
 * TDK tvclb820lba voice coil motor driver IC LC898217.
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include "lens_info.h"
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include	"Af.h"
#include "AfSTMV.h"

#define E2PROM_WRITE_ID 0xA0
#define E2PROM_PRIMAX_WRITE_ID 0xA0
#define AF_DRVNAME "LC898217AF_DRV"
#define AF_I2C_SLAVE_ADDR        0xE4

#define	Min_Pos		0
#define Max_Pos		1023

signed short Hall_Max = 0x0000;	/* Please read INF position from EEPROM or OTP */
signed short Hall_Min = 0x0000;	/* Please read MACRO position from EEPROM or OTP */
#define TIME_OUT 1000
#define TIMEOUT_ERROR 10

#define LC898217_DRVNAME "LC898217AF"

#define AF_DEBUG
#ifdef AF_DEBUG
#define LC898217DB(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LC898217DB(format, args...)
#endif

static spinlock_t* g_LC898217_SpinLock;

static struct i2c_client *g_pstLC898217_I2Cclient;

static int*  g_s4LC898217_Opened;
static long g_i4MotorStatus;
static unsigned long g_u4LC898217_INF;
static unsigned long g_u4LC898217_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

/*******************************************************************************
* WriteRegI2C
********************************************************************************/
int LC898217_WriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId)
{
	int i4RetValue = 0;
	int retry = 3;

	spin_lock(g_LC898217_SpinLock);
	g_pstLC898217_I2Cclient->addr = (i2cId >> 1);
	g_pstLC898217_I2Cclient->ext_flag =
	    (g_pstLC898217_I2Cclient->ext_flag) & (~I2C_DMA_FLAG);
	spin_unlock(g_LC898217_SpinLock);

	do {
		i4RetValue =
		    i2c_master_send(g_pstLC898217_I2Cclient, a_pSendData, a_sizeSendData);
		if (i4RetValue != a_sizeSendData) {
			LC898217DB("[LC898217] I2C send failed!!, Addr = 0x%x, Data = 0x%x\n",
				     a_pSendData[0], a_pSendData[1]);
		} else {
			break;
		}
		udelay(50);
	} while ((retry--) > 0);

	return 0;
}

/*******************************************************************************
* ReadRegI2C
********************************************************************************/
int LC898217_ReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,
			  u16 a_sizeRecvData, u16 i2cId)
{
	int i4RetValue = 0;

	spin_lock(g_LC898217_SpinLock);
	g_pstLC898217_I2Cclient->addr = (i2cId >> 1);
	g_pstLC898217_I2Cclient->ext_flag =
	    (g_pstLC898217_I2Cclient->ext_flag) & (~I2C_DMA_FLAG);
	spin_unlock(g_LC898217_SpinLock);

	i4RetValue = i2c_master_send(g_pstLC898217_I2Cclient, a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		LC898217DB("[LC898217] I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstLC898217_I2Cclient, (u8 *) a_pRecvData, a_sizeRecvData);
	if (i4RetValue != a_sizeRecvData) {
		LC898217DB("[LC898217] I2C read failed!!\n");
		return -1;
	}

	return 0;
}



void E2PROMReadA(unsigned short addr, u8 *data)
{
	int ret = 0;

	u8 puSendCmd[2] = { (u8) (addr >> 8), (u8) (addr & 0xFF) };

	ret = LC898217_ReadRegI2C(puSendCmd, sizeof(puSendCmd), data, 1, E2PROM_WRITE_ID);
	if (ret < 0)
		LC898217DB("[LC898217] I2C read e2prom failed!!\n");

	return;
}
void RegReadA(unsigned short addr, unsigned char *data)
{
	/* To call your IIC function here */
	u8 puSendCmd[1] = { (u8) (addr & 0xFF) };

	LC898217_ReadRegI2C(puSendCmd, sizeof(puSendCmd), data, 1, AF_I2C_SLAVE_ADDR);
}
void RegWriteA(unsigned short addr, unsigned char data)
{
	/* To call your IIC function here */
	u8 puSendCmd[2] = { (u8) (addr & 0xFF), (u8) (data & 0xFF) };

	LC898217_WriteRegI2C(puSendCmd, sizeof(puSendCmd), AF_I2C_SLAVE_ADDR);
}
unsigned char CheckCver( void )
{
	unsigned char UcLsiVer;

	RegReadA( CVER, &UcLsiVer );

	return( UcLsiVer == 0x71 ) ? SUCCESS : FAILURE ;
}
unsigned char	WakeUpCheck(unsigned char UcRescailMode)
{
	unsigned char	UcStatus, UcReadDat ;
	unsigned int	i ;
	LC898217DB("wakeupcheck enter!\n");	
	//I2C communication check if necessary	
	UcStatus	= CheckCver() ;
	if( UcStatus != FAILURE ) {
		for( i = 0 ; i < 3000 ; i++  ) {
			RegReadA( FUNCRSLT1, &UcReadDat ) ;
			if(UcRescailMode == RESCAILING){
				if( UcReadDat == 0x01 )             //Rescailing Mode
					break ;
			}
			else
			{
				if( !UcReadDat )
					break ;			
			}
		}

		if( i == 3000 ) {
			UcStatus	= DOWNLOAD_ERROR ;
		}
	}
		
	RegWriteA( PINC, 0x02 ) ;
	RegWriteA( TESTC, 0x20 ) ;
	LC898217DB("wakeupcheck exit!\n");

	
	return( UcStatus ) ;
}
void RescailEn( void )
{
	RegWriteA(FUNCRUN2	, 0x02 );  //0xA1 02h, After Calibration Data Writing, Rescailing Run 
	msleep(1) ;//WAIT	1ms
}
unsigned char lc898217_Init( void )
{
	unsigned char	UcSts ;
	//unsigned char	UcDownloadMode = AUTO_DOWNLOAD;	
	//unsigned char	UcMode = RESCAILING ;
	LC898217DB("lc898217_Init enter!\n");
	
	//After Power On
	msleep(1); //Wait 8ms, Make sure VDD Stabile,
		   //This is one example of time. Please adjust wait time to fit your system.
		   //And change WitTim() to your system delay function.
	 
	UcSts = WakeUpCheck(RESCAILING);
		if (UcSts != SUCCESS)
			return FAILURE;	

	RescailEn(); //A1h 02h setting
	LC898217DB("lc898217_Init exit!\n");		
	return SUCCESS;	
}
unsigned char Stmv217( unsigned char DH, unsigned char DL )
{
	unsigned char	UcConvCheck;
	unsigned char	UcSetTime;
	unsigned char	UcCount;	
	
	//AF Operation
	RegWriteA(TARGETH	, DH );  //0x84, DH 0x0X
	RegWriteA(TARGETL	, DL );  //0x85, DL 0xXX
	msleep(5) ;//WAIT	5ms
	UcCount = 0;
	do
	{
		RegReadA(SRVSTATE1, &UcConvCheck); 	//0xB0
		UcConvCheck = UcConvCheck & 0x80;  	//bit7 == 0 or not
		RegReadA(TGTCNVTIM, &UcSetTime); 	//0xB2 Settle Time check Settle Time Calculate =
							// ( B2h Read Value x 0.171mS ) - 2.731mS Condition:
							//EEPROM 55h(THD1) = 0x20
							//82h:bit2-0( LOCCNVCNT ) = 100b
		msleep(2) ;//WAIT 2ms
		UcCount++;
		if(UcCount == TIME_OUT)	{
			return TIMEOUT_ERROR;
		}
	}while(UcConvCheck); 	
	
	return SUCCESS;
}
unsigned char SetPosition(unsigned long UsPosition)
{
    unsigned char UcPosH;
    unsigned char UcPosL;
    unsigned char UcAFSts;
    //unsigned char UcMaxPosition = 1023;  

    //UsPosition = 1023 - UsPosition; //Only SI1330C, SI1331C

    UcPosH = (unsigned char)(UsPosition >> 8);
    //UcPosH = UcPosH |0x40;
    UcPosL = (unsigned char)(UsPosition & 0x00FF);    

    UcAFSts = Stmv217( UcPosH, UcPosL );   
    if (UcAFSts != SUCCESS)
	return FAILURE;	

    return  UcAFSts; 
}

static int getLC898217Info(__user stAF_MotorInfo * pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4LC898217_MACRO;
	stMotorInfo.u4InfPosition = g_u4LC898217_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_s4LC898217_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;


	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LC898217DB("[LC898217] copy to user failed when getting motor information\n");

	return 0;
}

#ifdef LensdrvCM3
static int getLC898217META(__user stFM50AF_MotorMETAInfo * pstMotorMETAInfo)
{
	stLC898217_MotorMETAInfo stMotorMETAInfo;

	stMotorMETAInfo.Aperture = 2.8;	/* fn */
	stMotorMETAInfo.Facing = 1;
	stMotorMETAInfo.FilterDensity = 1;	/* X */
	stMotorMETAInfo.FocalDistance = 1.0;	/* diopters */
	stMotorMETAInfo.FocalLength = 34.0;	/* mm */
	stMotorMETAInfo.FocusRange = 1.0;	/* diopters */
	stMotorMETAInfo.InfoAvalibleApertures = 2.8;
	stMotorMETAInfo.InfoAvalibleFilterDensity = 1;
	stMotorMETAInfo.InfoAvalibleFocalLength = 34.0;
	stMotorMETAInfo.InfoAvalibleHypeDistance = 1.0;
	stMotorMETAInfo.InfoAvalibleMinFocusDistance = 1.0;
	stMotorMETAInfo.InfoAvalibleOptStabilization = 0;
	stMotorMETAInfo.OpticalAxisAng[0] = 0.0;
	stMotorMETAInfo.OpticalAxisAng[1] = 0.0;
	stMotorMETAInfo.Position[0] = 0.0;
	stMotorMETAInfo.Position[1] = 0.0;
	stMotorMETAInfo.Position[2] = 0.0;
	stMotorMETAInfo.State = 0;
	stMotorMETAInfo.u4OIS_Mode = 0;

	if (copy_to_user(pstMotorMETAInfo, &stMotorMETAInfo, sizeof(stLC898217_MotorMETAInfo)))
		LC898217DB("[LC898217] copy to user failed when getting motor information\n");

	return 0;
}
#endif

unsigned short AF_convert(int position)
{
#if 0				/* 1: INF -> Macro =  0x8001 -> 0x7FFF */
	return (((position - Min_Pos) * (unsigned short)(Hall_Max - Hall_Min) / (Max_Pos -
										 Min_Pos)) +
		Hall_Min) & 0xFFFF;
#else	/* k55v2_64_stereo */			/* 0: INF -> Macro =  0x7FFF -> 0x8001 */
	return (((Max_Pos - position) * (unsigned short)(Hall_Max - Hall_Min) / (Max_Pos -
										 Min_Pos)) +
		Hall_Min) & 0xFFFF;
#endif
}

static int moveLC898217(unsigned long a_u4Position)
{
	if (*g_s4LC898217_Opened==1){
		lc898217_Init();
		spin_lock(g_LC898217_SpinLock);
		*g_s4LC898217_Opened=2;
		spin_unlock(g_LC898217_SpinLock);
	}
	LC898217DB("=====LC898217:==hall_max:0x%x==hall_min:0x%x==position:%lu====\n", Hall_Max,
	       Hall_Min, a_u4Position);
	SetPosition(a_u4Position);	/* Move to Target Position */

	spin_lock(g_LC898217_SpinLock);
	g_u4CurrPosition = (unsigned long)a_u4Position;
	spin_unlock(g_LC898217_SpinLock);
	return 0;
}

static int setLC898217Inf(unsigned long a_u4Position)
{
	spin_lock(g_LC898217_SpinLock);
	g_u4LC898217_INF = a_u4Position;
	spin_unlock(g_LC898217_SpinLock);
	return 0;
}

static int setLC898217Macro(unsigned long a_u4Position)
{
	spin_lock(g_LC898217_SpinLock);
	g_u4LC898217_MACRO = a_u4Position;
	spin_unlock(g_LC898217_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long LC898217AF_Ioctl(struct file *a_pstFile,
			     unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getLC898217Info((__user stAF_MotorInfo *) (a_u4Param));
		break;
#ifdef LensdrvCM3
/*
	case LC898212AFIOC_G_MOTORMETAINFO:
		i4RetValue = getLC898217META((__user stLC898212AF_MotorMETAInfo *) compat_ptr(a_u4Param));
		break;*/
#endif
	case AFIOC_T_MOVETO:
		i4RetValue = moveLC898217((compat_uint_t) a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setLC898217Inf((compat_uint_t) a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setLC898217Macro((compat_uint_t) a_u4Param);
		break;

	default:
		LC898217DB("[LC898217] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int LC898217AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LC898217DB("[LC898217] LC898217_Release - Start\n");

	if (*g_s4LC898217_Opened) {
		spin_lock(g_LC898217_SpinLock);
		*g_s4LC898217_Opened = 0;
		spin_unlock(g_LC898217_SpinLock);

	}

	LC898217DB("[LC898217] LC898217_Release - End\n");

	return 0;
}
void LC898217AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstLC898217_I2Cclient = pstAF_I2Cclient;
	g_LC898217_SpinLock = pAF_SpinLock;
	g_s4LC898217_Opened = pAF_Opened;
}

