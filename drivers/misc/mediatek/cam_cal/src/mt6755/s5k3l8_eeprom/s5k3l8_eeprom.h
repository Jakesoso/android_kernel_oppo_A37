/*****************************************************************************
 *
 * Filename:
 * ---------
 *   catc24c16.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 *   LukeHu
 *
 *============================================================================*/
#ifndef __S5K3L8_EEPROM_H
#define __S5K3L8_EEPROM_H

#define CAM_CAL_DEV_MAJOR_NUMBER 226

/* CAM_CAL READ/WRITE ID */
#define S5K3L8_DEVICE_ID							0xA0
#define CUR_EEPROM_DEVICE_ID                                    S5K3L8_DEVICE_ID
/*#define I2C_UNIT_SIZE                                  1 //in byte*/
/*#define OTP_START_ADDR                            0x0A04*/
/*#define OTP_SIZE                                      24*/
#ifdef VENDOR_EDIT
/*oppo hufeng 20160523 add to speed up open camera time when first open after power on phone*/
#define SHAREDDATAADDR 0x0000
#define BLANKDATAADDR   0x0048
#define OISDATAADDR       0x0100
#define QCOMDATAADDR    0x0200
#define MTKLSCDATAADDR    0x0C00
#define MTKPDAFDATAADDR  0x1400
#define EEPROMENDADDR      0x1A67

#define SHAREDDATALEN (BLANKDATAADDR-SHAREDDATAADDR)
#define BLANKDATALEN   (OISDATAADDR-BLANKDATAADDR)
#define OISDATALEN       (QCOMDATAADDR-OISDATAADDR)
#define QCOMDATALEN    (MTKLSCDATAADDR-QCOMDATAADDR)
#define MTKLSCDATALEN    (MTKPDAFDATAADDR-MTKLSCDATAADDR)
#define MTKPDAFDATALEN    (EEPROMENDADDR-MTKPDAFDATAADDR+1)

#define SHAREDDATAFALG   1
#define BLANKDATAFLAG      0
#define OISDATAFLAG           0
#define QCOMDATAFLAG       0
#define MTKLSCDATAFLAG    1
#define MTKPDAFDATAFLAG  1
typedef enum
{
	SharedData = 0,
	BlankData,
	OisData,
	QcomData,
	MtkLscData,
	MtkPdafData,
	MaxMapNum,
}eEepromMap;
#endif
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData,
u16 a_sizeRecvData, u16 i2cId);



#endif /* __CAM_CAL_H */

