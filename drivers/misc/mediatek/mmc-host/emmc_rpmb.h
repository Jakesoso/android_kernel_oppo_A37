


#ifndef _EMMC_RPMB_H
#define _EMMC_RPMB_H

#include <linux/mmc/ioctl.h>
#include <linux/mmc/card.h>
//#ifdef VENDOR_EDIT //Haitao.Zhou@Prd.BaseDrv for RPMB patch 
#include <linux/ioctl.h>
//#endif

extern struct msdc_host *mtk_msdc_host[];

/************************************************************************
 *
 * RPMB IOCTL interface.
 *
 ***********************************************************************/
 //#ifdef VENDOR_EDIT //Haitao.Zhou@Prd.BaseDrv for RPMB patch 
#define RPMB_MAGIC               'p'
#define RPMB_IOCTL_PROGRAM_KEY   _IOW(RPMB_MAGIC, 1, int)
#define RPMB_IOCTL_WRITE_DATA    _IOW(RPMB_MAGIC, 2, int)
#define RPMB_IOCTL_READ_DATA     _IOWR(RPMB_MAGIC, 3, int)
#define RPMB_IOCTL_GET_RPMB_SIZE _IOR(RPMB_MAGIC, 4, int)
#define RPMB_IOCTL_SET_USER_STAT _IOW(RPMB_MAGIC, 5, int)
#define RPMB_IOCTL_GET_USER_STAT _IOWR(RPMB_MAGIC, 6, int)
//#endif
 
#if (defined(CONFIG_MICROTRUST_TZ_DRIVER))

#define RPMB_MULTI_BLOCK_ACCESS 1

#if RPMB_MULTI_BLOCK_ACCESS
#define MAX_RPMB_TRANSFER_BLK 16
/* 8KB(16blks) per requests */
#define MAX_RPMB_REQUEST_SIZE (512*MAX_RPMB_TRANSFER_BLK)
#else
#define MAX_RPMB_TRANSFER_BLK 1
/* 512B(1blks) per requests */
#define MAX_RPMB_REQUEST_SIZE (512*MAX_RPMB_TRANSFER_BLK)
#endif

//#ifdef VENDOR_EDIT //Haitao.Zhou@Prd.BaseDrv for RPMB patch 
#define RPMB_IOCTL_SOTER_WRITE_DATA   _IOWR(RPMB_MAGIC, 7, int)
#define RPMB_IOCTL_SOTER_READ_DATA    _IOWR(RPMB_MAGIC, 8, int)
#define RPMB_IOCTL_SOTER_GET_CNT      _IOWR(RPMB_MAGIC, 9, int)
//#endif

struct rpmb_infor {
	unsigned int size;
	unsigned char *data_frame;
};
#endif

struct rpmb_ioc_param {
	unsigned char *key;
	unsigned char *data;
	unsigned int  data_len;
	unsigned short addr;
	unsigned char *hmac;
	unsigned int hmac_len;
};
/***********************************************************************/


#define RPMB_SZ_STUFF 196
#define RPMB_SZ_MAC   32
#define RPMB_SZ_DATA  256
#define RPMB_SZ_NONCE 16

struct s_rpmb {
	unsigned char stuff[RPMB_SZ_STUFF];
	unsigned char mac[RPMB_SZ_MAC];
	unsigned char data[RPMB_SZ_DATA];
	unsigned char nonce[RPMB_SZ_NONCE];
	unsigned int write_counter;
	unsigned short address;
	unsigned short block_count;
	unsigned short result;
	unsigned short request;
};

enum {
	RPMB_SUCCESS = 0,
	RPMB_HMAC_ERROR,
	RPMB_RESULT_ERROR,
	RPMB_WC_ERROR,
	RPMB_NONCE_ERROR,
	RPMB_ALLOC_ERROR,
	RPMB_TRANSFER_NOT_COMPLETE,
};

#define RPMB_PROGRAM_KEY       1       /* Program RPMB Authentication Key */
#define RPMB_GET_WRITE_COUNTER 2       /* Read RPMB write counter */
#define RPMB_WRITE_DATA		   3	   /* Write data to RPMB partition */
#define RPMB_READ_DATA         4       /* Read data from RPMB partition */
#define RPMB_RESULT_READ       5       /* Read result request */
#define RPMB_REQ               1       /* RPMB request mark */
#define RPMB_RESP              (1 << 1)/* RPMB response mark */
#define RPMB_AVALIABLE_SECTORS 8       /* 4K page size */

#define RPMB_TYPE_BEG          510
#define RPMB_RES_BEG           508
#define RPMB_BLKS_BEG          506
#define RPMB_ADDR_BEG          504
#define RPMB_WCOUNTER_BEG      500

#define RPMB_NONCE_BEG         484
#define RPMB_DATA_BEG          228
#define RPMB_MAC_BEG           196

struct emmc_rpmb_req {
	__u16 type;                     /* RPMB request type */
	__u16 *result;                  /* response or request result */
	__u16 blk_cnt;                  /* Number of blocks(half sector 256B) */
	__u16 addr;                     /* data address */
	__u32 *wc;                      /* write counter */
	__u8 *nonce;                    /* Ramdom number */
	__u8 *data;                     /* Buffer of the user data */
	__u8 *mac;                      /* Message Authentication Code */
	__u8 *data_frame;
};

//#ifdef VENDOR_EDIT //Haitao.Zhou@Prd.BaseDrv for RPMB patch 
#define MAX_RPMB_USER_NUMBER 4

typedef enum {
	RPMB_USER_ID_BASE = 0xAD,
	RPMB_USER_ID_0 = RPMB_USER_ID_BASE,
	RPMB_USER_ID_1,
	RPMB_USER_ID_2,
	RPMB_USER_ID_3,
	RPMB_USER_ID_END = RPMB_USER_ID_BASE + MAX_RPMB_USER_NUMBER,
} rpmb_uid_t;

struct rpmb_user {
	u32 uid;
	u32 pid;
	u32 sessionid;
};
//#endif

int mmc_rpmb_set_key(struct mmc_card *card, void *key);
int mmc_rpmb_read(struct mmc_card *card, u8 *buf, u16 blk, u16 cnt, void *key);
int mmc_rpmb_write(struct mmc_card *card, u8 *buf, u16 blk, u16 cnt, void *key);

extern void emmc_rpmb_set_host(void *mmc_host);


#endif
