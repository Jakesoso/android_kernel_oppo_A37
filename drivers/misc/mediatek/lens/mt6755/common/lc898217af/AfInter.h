#ifndef	__AFINITER__
#define __AFINITER__

extern int LC898217_WriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int LC898217_ReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,
				 u16 a_sizeRecvData, u16 i2cId);

#endif
