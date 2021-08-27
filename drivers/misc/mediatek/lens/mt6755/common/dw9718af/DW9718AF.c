/*
 * DW9718AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "DW9718AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif
#ifdef VENDOR_EDIT
/*oppo hufeng 20151111 add for dw9800*/
typedef enum manu_id
{
 IC_9718 = 0,
 IC_9800,
 IC_MAX,
}MID;
static MID ic_diver = IC_MAX;
#endif

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[1] = { (char)(a_u2Addr) };
/*oppo hufeng 20150930 add for i2c transfer*/
	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
/*oppo hufeng add end*/
	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puReadCmd, 1);
/*oppo hufeng change 2 to 1, as just send one byte data*/
	if (i4RetValue != 1) {
		LOG_INF(" I2C write failed!!\n");
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, (char *)a_puBuff, 1);
	if (i4RetValue != 1) {
		LOG_INF(" I2C read failed!!\n");
		return -1;
	}

	return 0;
}

static u8 read_data(u8 addr)
{
	u8 get_byte = 0;
	i2c_read(addr, &get_byte);

	return get_byte;
}

static int s4DW9718AF_ReadReg(unsigned short *a_pu2Result)
{
#ifdef VENDOR_EDIT
/*oppo hufeng 20151111 add for dw9800*/
	if(ic_diver == IC_9718)
		*a_pu2Result = (read_data(0x02) << 8) + (read_data(0x03) & 0xff);
	if(ic_diver == IC_9800)
		*a_pu2Result = (read_data(0x03) << 8) + (read_data(0x04)&0xff);
#else
	*a_pu2Result = (read_data(0x02) << 8) + (read_data(0x03) & 0xff);
#endif
	return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;
#ifdef VENDOR_EDIT
/*oppo hufeng 20151111 add for dw9800*/
	char puSendCmd[3];
	if(ic_diver == IC_9718){
		puSendCmd[0] = 0x02;
	}
	if(ic_diver == IC_9800){
		puSendCmd[0] = 0x03;
	}
	puSendCmd[1] = (char)(a_u2Data >> 8);
	puSendCmd[2] = (char)(a_u2Data & 0xFF); 
#else
	char puSendCmd[3] = { 0x02, (char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF) };
#endif
	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	g_pstAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 3);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static inline int getAFInfo(__user stAF_MotorInfo * pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;
	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

#ifndef VENDOR_EDIT
/*oppo hufeng add 20151102 for dw9800af*/
static void initdrv(void)
{
	char puSendCmd2[2] = { 0x01, 0x39 };
	char puSendCmd3[2] = { 0x05, 0x65 };
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
}


#else
//ic_driver = IC_9718
static void initdrvDW9718AF(void)
{
#if 0  //modify by tower from mtkxuyan 20151218
	char puSendCmd2[2] = { 0x01, 0x39 };
	char puSendCmd3[2] = { 0x05, 0x65 };
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
#else
    char puSendCmd1[2] = { 0x02, 0x02 };
    char puSendCmd2[2] = { 0x06, 0x40 };
    char puSendCmd3[2] = { 0x07, 0x6a };
    i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2);
    i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
    i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
#endif
}
//ic_diver = IC_9800;
static void initdrvDW9800AF(void)
{
	char puSendCmd2[2] = {0x02,0x02};
	char puSendCmd3[2] = {0x06,0x40};
	char puSendCmd4[2] = {0x07,0x6a};
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
	i2c_master_send(g_pstAF_I2Cclient, puSendCmd4, 2);
}
static void initdrv(void)
{
	u8 vcm_id = 0xAA;
	vcm_id = read_data(0x00);
	LOG_INF("vcm_id is %d\n", vcm_id);
	if(vcm_id < 0)
		return vcm_id;
	if(vcm_id == 0){
		ic_diver = IC_9718;
		initdrvDW9718AF();
	}
	if(vcm_id == 0xF2){
		ic_diver = IC_9800;
		initdrvDW9800AF();
	}
}
#endif
static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;
		initdrv();
		ret = s4DW9718AF_ReadReg(&InitPos);

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	/*LOG_INF("move [curr] %ld [target] %ld\n", g_u4CurrPosition, g_u4TargetPosition);*/


	if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
	}

	return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}
#ifdef VENDOR_EDIT
/*oppo hufeng 20160531 add for lens alive test*/
static inline int checkMotorAlive()
{
	int ret = -1;
	unsigned short InitPos = 0xffff;
	unsigned short targetPos = 200;
	initdrv();
	s4AF_WriteReg((unsigned short)targetPos);
	ret = s4DW9718AF_ReadReg(&InitPos);
	LOG_INF("Init Pos %6d\n", InitPos);
	if (ret == 0)
	{
		LOG_INF("Init Pos %6d\n", InitPos);
		return 1;
	}
	return 0;
}
#endif
/* ////////////////////////////////////////////////////////////// */
long DW9718AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;
#ifdef VENDOR_EDIT
/*oppo hufeng 20160531 add for lens alive test*/
	case AFIOC_T_CHECKALIVE:	
		i4RetValue = checkMotorAlive();
		break;
#endif
	default:
		LOG_INF("No CMD\n");
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
int DW9718AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2)
		LOG_INF("Wait\n");

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

void DW9718AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}
