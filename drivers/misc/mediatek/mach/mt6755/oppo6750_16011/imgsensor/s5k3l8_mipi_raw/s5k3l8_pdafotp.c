#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/xlog.h>


#define PFX "s5k3l8_pdafotp"
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_ERROR   , PFX, "[%s] " format, __FUNCTION__, ##args)

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#ifdef VENDOR_EDIT
//jindian.guan 2015/04/15 add proc file for pdaf
#include <linux/proc_fs.h>
#endif
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define S5K3L8_EEPROM_READ_ID  0xA0
#define S5K3L8_EEPROM_WRITE_ID   0xA1
#define S5K3L8_I2C_SPEED        100  
#define S5K3L8_MAX_OFFSET		0x1FFF
#ifdef VENDOR_EDIT
//jindian.guan 2015/04/15 add proc file for pdaf
#define PDAF_SUPPORT   0x02
#define PDAF_NOT_SUPPORT   0x00
#define PDAF_OTP_OK    0x01
#define PDAF_OTP_FAIL    0x00
#endif

#define DATA_SIZE 8192//2048
BYTE s5k3l8_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;
#ifdef VENDOR_EDIT
//jindian.guan 2015/04/15 add proc file for pdaf
static int pdaf_info = PDAF_SUPPORT|PDAF_OTP_FAIL;
static int eeprom_info = 0;
#endif
#define PROC1_DATA_SIZE		496
#define PROC2_DATA_SIZE		806
#define PROC3_DATA_SIZE		102

#ifdef VENDOR_EDIT
//jindian.guan 2015/04/15 add proc file for pdaf
static ssize_t PDAF_READ(struct file *filp, char __user *buff,
                        	size_t len, loff_t *data)
{
    char value[2] = {0};

    snprintf(value, sizeof(value), "%d", pdaf_info);
    return simple_read_from_buffer(buff, len, data, value,1);
}
static ssize_t PDAF_WRITE( struct file *file, const char __user *buffer, size_t count,loff_t *data)
{
    return 0;
}

static const struct file_operations pdaf_proc_fops = {
	.owner = THIS_MODULE,
	.read = PDAF_READ,
	.write = PDAF_WRITE,
};

static int pdaf_proc_init(void)
{
	int ret=0;
	struct proc_dir_entry *proc_entry = proc_create_data( "pdaf_info", 0666, NULL,&pdaf_proc_fops, NULL);
	if (proc_entry == NULL)
	{
		ret = -ENOMEM;
	  	LOG_INF("Error! Couldn't create pdaf_info proc entry\n");
	}
	return ret;
}

static ssize_t EEPROM_READ(struct file *filp, char __user *buff,size_t len, loff_t *data)
{
    char value[2] = {0};

    snprintf(value, sizeof(value), "%d", eeprom_info);
    return simple_read_from_buffer(buff, len, data, value,1);
}
static ssize_t EEPROM_WRITE( struct file *file, const char __user *buffer, size_t count,loff_t *data)
{
    return 0;
}

static const struct file_operations eeprom_proc_fops = {
	.owner = THIS_MODULE,
	.read = EEPROM_READ,
	.write = EEPROM_WRITE,
};

static int eeprom_proc_init(void)
{
	int ret=0;
	struct proc_dir_entry *proc_entry = proc_create_data( "s5k3l8_eeprom_info", 0666, NULL,&eeprom_proc_fops, NULL);
	if (proc_entry == NULL)
	{
		ret = -ENOMEM;
		LOG_INF("Error! Couldn't create eeprom_info proc entry\n");
	}
	return ret;
}

#endif
bool selective_read_eeprom_3l8(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > S5K3L8_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(S5K3L8_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, S5K3L8_EEPROM_READ_ID)<0)
		return false;
    return true;
}

static bool _read_3l8_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom_3l8(offset, &data[i])){
			return false;
		}
		//LOG_INF("read_eeprom 0x%0x %x\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

/*
proc1
data 0x1400--0x15EF
valid flag 0x15F0
checksum 0x15F1

proc2
data 0x1600--0x1925
valid flag 0x1926
checksum 0x1927

proc3
data 0x1A00--0x1A65
valid flag 0x1A66
checksum 0x1A67
*/
bool read_3l8_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size){
	int i = 0;
	int proc1_flag = 0;
	int proc2_flag = 0;
	int proc3_flag = 0;
    BYTE num = 0;
#ifndef VENDOR_EDIT
//jindian.guan 2015/04/15 add proc file for pdaf
	pdaf_proc_init();
	eeprom_proc_init();
#endif
	LOG_INF("read 3l8 eeprom, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_3l8_eeprom(addr, s5k3l8_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}

	proc1_flag = s5k3l8_eeprom_data[0x15F0-0x1400];
	proc2_flag = s5k3l8_eeprom_data[0x1926-0x1400];
	proc3_flag = s5k3l8_eeprom_data[0x1A66-0x1400];

	LOG_INF("proc1_flag = %d, proc2_flag = %d, proc3_flag = %d\n", proc1_flag, proc2_flag, proc3_flag);

	if((proc1_flag != 1) || (proc2_flag != 1) || (proc3_flag != 1))
	{
		LOG_INF("3M2 PDAF eeprom data invalid\n");
		return false;
	}
#ifndef VENDOR_EDIT
//jindian.guan 2015/04/15 add proc file for pdaf
    pdaf_info = pdaf_info|PDAF_OTP_OK;

    if(selective_read_eeprom_3l8(0x0030,&num))
    {
        eeprom_info = num;
    }
#endif
	// proc1 data
	for(i=0; i<496; i++)
	{
		data[i] = s5k3l8_eeprom_data[i];
		LOG_INF("data[%d] = %x\n", i, data[i]);
	}

	// proc2 data
	for(i=496; i<1302; i++)
	{
		data[i] = s5k3l8_eeprom_data[i+16];
		LOG_INF("data[%d] = %x\n", i, data[i]);
	}

	// proc3 data
	for(i=1302; i<1639; i++)
	{
		data[i] = s5k3l8_eeprom_data[i+16+218];
		LOG_INF("data[%d] = %x\n", i, data[i]);
	}	

	#if 0
	for(i=0; i<1404; i++)
	{
		LOG_INF("data[%d] = %d\n", i, data[i]);
	}
	#endif
	
    return true;
}


