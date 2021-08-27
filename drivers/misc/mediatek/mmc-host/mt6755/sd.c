#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/blkdev.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/mman.h>
#include <linux/pagemap.h>
#include <linux/highmem.h>
#include <linux/printk.h>
#include <asm/page.h>
#include <mach/dma.h>
#include <mach/board.h>
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <mach/mt_chip.h>
#include <core/mmc_ops.h>
#include <core/core.h>
#include <card/queue.h>
#include "mt_sd.h"
#include "dbg.h"
#include "msdc_tune.h"
#include "autok.h"
#include "autok_dvfs.h"

#ifdef CONFIG_MMC_FFU
#include <linux/mmc/ffu.h>
#endif

#ifdef MTK_MSDC_BRINGUP_DEBUG
#include <mach/mt_pmic_wrap.h>
#endif

#ifdef MTK_MSDC_USE_CACHE
#include <mach/mt_boot.h>
#endif

#include <linux/proc_fs.h>
#include <mach/emi_mpu.h>
#include <mach/memory.h>
#ifdef CONFIG_MTK_AEE_FEATURE
#include <linux/aee.h>
#endif

#ifdef CONFIG_MTK_HIBERNATION
#include "mach/mtk_hibernate_dpm.h"
#endif

#if 1 /*#ifndef FPGA_PLATFORM*/
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>
#endif
#include <mach/partition.h>

#include <mach/power_loss_test.h>

#define CAPACITY_2G             (2 * 1024 * 1024 * 1024ULL)

#ifdef CONFIG_MTK_EMMC_SUPPORT
u32 g_emmc_mode_switch = 0;
EXPORT_SYMBOL(g_emmc_mode_switch);
#endif

#ifdef MTK_MSDC_USE_CACHE
#define MSDC_MAX_FLUSH_COUNT    (3)
#define CACHE_UN_FLUSHED        (0)
#define CACHE_FLUSHED           (1)
static unsigned int g_cache_status = CACHE_UN_FLUSHED;
static unsigned long long g_flush_data_size;
static unsigned int g_flush_error_count;
static int g_flush_error_happend;
#endif

unsigned long long msdc_print_start_time;
unsigned long long msdc_print_end_time;
unsigned int print_nums;

#if (MSDC_DATA1_INT == 1)
static u16 u_sdio_irq_counter;
static u16 u_msdc_irq_counter;
/*static int int_sdio_irq_enable;*/
#endif

struct msdc_host *ghost;
int src_clk_control;

bool emmc_sleep_failed;
static int emmc_do_sleep_awake;
bool sdio_lock_dvfs = 0;

#if defined(FEATURE_MET_MMC_INDEX)
static unsigned int met_mmc_bdnum;
#endif

#define DRV_NAME                "mtk-msdc"

#define MSDC_COOKIE_PIO         (1<<0)
#define MSDC_COOKIE_ASYNC       (1<<1)

#define msdc_use_async(x)       (x & MSDC_COOKIE_ASYNC)
#define msdc_use_async_dma(x)   (msdc_use_async(x) && (!(x & MSDC_COOKIE_PIO)))
#define msdc_use_async_pio(x)   (msdc_use_async(x) && ((x & MSDC_COOKIE_PIO)))

#define HOST_MAX_BLKSZ          (2048)

#define MSDC_OCR_AVAIL          (MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 \
				| MMC_VDD_31_32 | MMC_VDD_32_33)
/* #define MSDC_OCR_AVAIL         (MMC_VDD_32_33 | MMC_VDD_33_34) */

#define DEFAULT_DEBOUNCE        (8)     /* 8 cycles */
#define DEFAULT_DTOC            (3)     /* data timeout counter.
					1048576 * 3 sclk. */

#define MAX_DMA_CNT             (64 * 1024 - 512)
				/* a WIFI transaction may be 50K */
#define MAX_DMA_CNT_SDIO        (0xFFFFFFFF - 255)
				/* a LTE  transaction may be 128K */

#define MAX_HW_SGMTS            (MAX_BD_NUM)
#define MAX_PHY_SGMTS           (MAX_BD_NUM)
#define MAX_SGMT_SZ             (MAX_DMA_CNT)
#define MAX_SGMT_SZ_SDIO        (MAX_DMA_CNT_SDIO)

static unsigned int cd_irq;

struct msdc_host *mtk_msdc_host[] = { NULL, NULL, NULL, NULL};
int g_dma_debug[HOST_MAX_NUM] = { 0, 0, 0, 0};
u32 latest_int_status[HOST_MAX_NUM] = { 0, 0, 0, 0};

transfer_mode msdc_latest_transfer_mode[HOST_MAX_NUM] = {
	/* 0 for PIO; 1 for DMA; 2 for nothing */
	TRAN_MOD_NUM,
	TRAN_MOD_NUM,
	TRAN_MOD_NUM,
	TRAN_MOD_NUM,
};

operation_type msdc_latest_op[HOST_MAX_NUM] = {
	/* 0 for read; 1 for write; 2 for nothing */
	OPER_TYPE_NUM,
	OPER_TYPE_NUM,
	OPER_TYPE_NUM,
	OPER_TYPE_NUM,
};

/* for debug zone */
unsigned int sd_debug_zone[HOST_MAX_NUM] = {
	0,
	0,
	0,
	0,
};

/* mode select */
#ifndef VENDOR_EDIT
// Jingchun.Wang@Phone.Bsp.Driver, 2016/05/17  Modify for use DMA mode instead of PIO mode 
u32 dma_size[HOST_MAX_NUM] = {
	512,
	512,
	512,
	512,
};
#else /*VENDOR_EDIT*/
u32 dma_size[HOST_MAX_NUM] = {
	512,
	8,
	512,
	512,
};
#endif /*VENDOR_EDIT*/

msdc_mode drv_mode[HOST_MAX_NUM] = {
	MODE_SIZE_DEP, /* using DMA or not depend on the size */
	MODE_SIZE_DEP,
	MODE_SIZE_DEP,
	MODE_SIZE_DEP,
};

u8 msdc_clock_src[HOST_MAX_NUM] = {
	0,
	0,
	0,
	0,
};

u32 msdc_host_mode[HOST_MAX_NUM] = {
	0,
	0,
	0,
	0,
};

u32 msdc_host_mode2[HOST_MAX_NUM] = {
	0,
	0,
	0,
	0,
};

int msdc_rsp[] = {
	0,                      /* RESP_NONE */
	1,                      /* RESP_R1 */
	2,                      /* RESP_R2 */
	3,                      /* RESP_R3 */
	4,                      /* RESP_R4 */
	1,                      /* RESP_R5 */
	1,                      /* RESP_R6 */
	1,                      /* RESP_R7 */
	7,                      /* RESP_R1b */
};

/* For Inhanced DMA */
#define msdc_init_gpd_ex(gpd, extlen, cmd, arg, blknum) \
	do { \
		((gpd_t *)gpd)->extlen = extlen; \
		((gpd_t *)gpd)->cmd    = cmd; \
		((gpd_t *)gpd)->arg    = arg; \
		((gpd_t *)gpd)->blknum = blknum; \
	} while (0)

#define msdc_init_bd(bd, blkpad, dwpad, dptr, dlen) \
	do { \
		BUG_ON(dlen > 0xFFFFUL); \
		((bd_t *)bd)->blkpad = blkpad; \
		((bd_t *)bd)->dwpad = dwpad; \
		((bd_t *)bd)->ptr = (u32)dptr; \
		((bd_t *)bd)->buflen = dlen; \
	} while (0)

#ifdef CONFIG_NEED_SG_DMA_LENGTH
#define msdc_sg_len(sg, dma)    ((dma) ? (sg)->dma_length : (sg)->length)
#else
#define msdc_sg_len(sg, dma)    sg_dma_len(sg)
#endif

#define msdc_dma_on()           MSDC_CLR_BIT32(MSDC_CFG, MSDC_CFG_PIO)
#define msdc_dma_off()          MSDC_SET_BIT32(MSDC_CFG, MSDC_CFG_PIO)
#define msdc_dma_status()       ((MSDC_READ32(MSDC_CFG) & MSDC_CFG_PIO) >> 3)

#define pr_reg(OFFSET, REG)     \
	pr_err("%d R[%x]=0x%.8x", id, OFFSET, MSDC_READ32(REG));

void msdc_dump_register_core(u32 id, void __iomem *base)
{
	pr_reg(OFFSET_MSDC_CFG,                 MSDC_CFG);
	pr_reg(OFFSET_MSDC_IOCON,               MSDC_IOCON);
	pr_reg(OFFSET_MSDC_PS,                  MSDC_PS);
	pr_reg(OFFSET_MSDC_INT,                 MSDC_INT);
	pr_reg(OFFSET_MSDC_INTEN,               MSDC_INTEN);
	pr_reg(OFFSET_MSDC_FIFOCS,              MSDC_FIFOCS);
	pr_err("R[%x]=not read", OFFSET_MSDC_TXDATA);
	pr_err("R[%x]=not read", OFFSET_MSDC_RXDATA);
	pr_reg(OFFSET_SDC_CFG,                  SDC_CFG);
	pr_reg(OFFSET_SDC_CMD,                  SDC_CMD);
	pr_reg(OFFSET_SDC_ARG,                  SDC_ARG);
	pr_reg(OFFSET_SDC_STS,                  SDC_STS);
	pr_reg(OFFSET_SDC_RESP0,                SDC_RESP0);
	pr_reg(OFFSET_SDC_RESP1,                SDC_RESP1);
	pr_reg(OFFSET_SDC_RESP2,                SDC_RESP2);
	pr_reg(OFFSET_SDC_RESP3,                SDC_RESP3);
	pr_reg(OFFSET_SDC_BLK_NUM,              SDC_BLK_NUM);
	pr_reg(OFFSET_SDC_VOL_CHG,              SDC_VOL_CHG);
	pr_reg(OFFSET_SDC_CSTS,                 SDC_CSTS);
	pr_reg(OFFSET_SDC_CSTS_EN,              SDC_CSTS_EN);
	pr_reg(OFFSET_SDC_DCRC_STS,             SDC_DCRC_STS);
	pr_reg(OFFSET_EMMC_CFG0,                EMMC_CFG0);
	pr_reg(OFFSET_EMMC_CFG1,                EMMC_CFG1);
	pr_reg(OFFSET_EMMC_STS,                 EMMC_STS);
	pr_reg(OFFSET_EMMC_IOCON,               EMMC_IOCON);
	pr_reg(OFFSET_SDC_ACMD_RESP,            SDC_ACMD_RESP);
	pr_reg(OFFSET_SDC_ACMD19_TRG,           SDC_ACMD19_TRG);
	pr_reg(OFFSET_SDC_ACMD19_STS,           SDC_ACMD19_STS);
	pr_reg(OFFSET_MSDC_DMA_SA_HIGH,         MSDC_DMA_SA_HIGH);
	pr_reg(OFFSET_MSDC_DMA_SA,              MSDC_DMA_SA);
	pr_reg(OFFSET_MSDC_DMA_CA,              MSDC_DMA_CA);
	pr_reg(OFFSET_MSDC_DMA_CTRL,            MSDC_DMA_CTRL);
	pr_reg(OFFSET_MSDC_DMA_CFG,             MSDC_DMA_CFG);
	pr_reg(OFFSET_MSDC_DMA_LEN,             MSDC_DMA_LEN);
	pr_reg(OFFSET_MSDC_DBG_SEL,             MSDC_DBG_SEL);
	pr_reg(OFFSET_MSDC_DBG_OUT,             MSDC_DBG_OUT);
	pr_reg(OFFSET_MSDC_PATCH_BIT0,          MSDC_PATCH_BIT0);
	pr_reg(OFFSET_MSDC_PATCH_BIT1,          MSDC_PATCH_BIT1);
	pr_reg(OFFSET_MSDC_PATCH_BIT2,          MSDC_PATCH_BIT2);

	if ((id != 2) && (id != 3))
		goto skip_sdio_tune_reg;

	pr_reg(OFFSET_DAT0_TUNE_CRC,            DAT0_TUNE_CRC);
	pr_reg(OFFSET_DAT0_TUNE_CRC,            DAT1_TUNE_CRC);
	pr_reg(OFFSET_DAT0_TUNE_CRC,            DAT2_TUNE_CRC);
	pr_reg(OFFSET_DAT0_TUNE_CRC,            DAT3_TUNE_CRC);
	pr_reg(OFFSET_CMD_TUNE_CRC,             CMD_TUNE_CRC);
	pr_reg(OFFSET_SDIO_TUNE_WIND,           SDIO_TUNE_WIND);

skip_sdio_tune_reg:
	pr_reg(OFFSET_MSDC_PAD_TUNE0,           MSDC_PAD_TUNE0);
	pr_reg(OFFSET_MSDC_PAD_TUNE1,           MSDC_PAD_TUNE1);
	pr_reg(OFFSET_MSDC_DAT_RDDLY0,          MSDC_DAT_RDDLY0);
	pr_reg(OFFSET_MSDC_DAT_RDDLY1,          MSDC_DAT_RDDLY1);
	pr_reg(OFFSET_MSDC_DAT_RDDLY2,          MSDC_DAT_RDDLY2);
	pr_reg(OFFSET_MSDC_DAT_RDDLY3,          MSDC_DAT_RDDLY3);
	pr_reg(OFFSET_MSDC_HW_DBG,              MSDC_HW_DBG);
	pr_reg(OFFSET_MSDC_VERSION,             MSDC_VERSION);

	if (id != 0)
		goto skip_emmc50_reg;

	pr_reg(OFFSET_EMMC50_PAD_DS_TUNE,       EMMC50_PAD_DS_TUNE);
	pr_reg(OFFSET_EMMC50_PAD_CMD_TUNE,      EMMC50_PAD_CMD_TUNE);
	pr_reg(OFFSET_EMMC50_PAD_DAT01_TUNE,    EMMC50_PAD_DAT01_TUNE);
	pr_reg(OFFSET_EMMC50_PAD_DAT23_TUNE,    EMMC50_PAD_DAT23_TUNE);
	pr_reg(OFFSET_EMMC50_PAD_DAT45_TUNE,    EMMC50_PAD_DAT45_TUNE);
	pr_reg(OFFSET_EMMC50_PAD_DAT67_TUNE,    EMMC50_PAD_DAT67_TUNE);
	pr_reg(OFFSET_EMMC51_CFG0,              EMMC51_CFG0);
	pr_reg(OFFSET_EMMC50_CFG0,              EMMC50_CFG0);
	pr_reg(OFFSET_EMMC50_CFG1,              EMMC50_CFG1);
	pr_reg(OFFSET_EMMC50_CFG2,              EMMC50_CFG2);
	pr_reg(OFFSET_EMMC50_CFG3,              EMMC50_CFG3);
	pr_reg(OFFSET_EMMC50_CFG4,              EMMC50_CFG4);

skip_emmc50_reg:
	return;
}

void msdc_dump_register(struct msdc_host *host)
{
	void __iomem *base = host->base;

	msdc_dump_register_core(host->id, base);
}

void msdc_dump_dbg_register_core(u32 id, void __iomem *base)
{
	u32 i;

	for (i = 0; i <= 0x27; i++) {
		MSDC_WRITE32(MSDC_DBG_SEL, i);
		SIMPLE_INIT_MSG("SEL:r[%x]=0x%x, OUT:r[%x]=0x%x",
			OFFSET_MSDC_DBG_SEL, i,
			OFFSET_MSDC_DBG_OUT, MSDC_READ32(MSDC_DBG_OUT));
	}

	MSDC_WRITE32(MSDC_DBG_SEL, 0);
}

static void msdc_dump_dbg_register(struct msdc_host *host)
{
	void __iomem *base = host->base;

	msdc_dump_dbg_register_core(host->id, base);
}

void msdc_dump_info(u32 id)
{
	struct msdc_host *host = mtk_msdc_host[id];
	void __iomem *base;

	if (host == NULL) {
		pr_err("msdc host<%d> null\r\n", id);
		return;
	}

	if (host->async_tuning_in_progress || host->legacy_tuning_in_progress)
		return;

	/* card is removed. */
	if ((host->hw->flags & MSDC_REMOVABLE) && mt_get_gpio_in(GPIO_MSDC1_INSI))
		return;

	base = host->base;

	/* 1: dump msdc hw register */
	msdc_dump_register(host);
	INIT_MSG("latest_INT_status<0x%.8x>", latest_int_status[id]);

	/* 2: check msdc clock gate and clock source */
	mdelay(10);
	msdc_dump_clock_sts();

	/* 3: check msdc pmic ldo */
	msdc_dump_ldo_sts(host);

	/* 4: check msdc pad control */
	msdc_dump_padctl(host);

	/* 5: For designer */
	mdelay(10);
	msdc_dump_dbg_register(host);
}

/*
 * for AHB read / write debug
 * return DMA status.
 */
int msdc_get_dma_status(int host_id)
{
	int result = -1;

	if (host_id < 0 || host_id >= HOST_MAX_NUM) {
		pr_err("[%s] failed to get dma status, invalid host_id %d\n",
			__func__, host_id);
	} else if (msdc_latest_transfer_mode[host_id] == TRAN_MOD_DMA) {
		if (msdc_latest_op[host_id] == OPER_TYPE_READ)
			return 1;       /* DMA read */
		else if (msdc_latest_op[host_id] == OPER_TYPE_WRITE)
			return 2;       /* DMA write */
	} else if (msdc_latest_transfer_mode[host_id] == TRAN_MOD_PIO) {
		return 0;               /* PIO mode */
	}

	return result;
}
EXPORT_SYMBOL(msdc_get_dma_status);

void msdc_clr_fifo(unsigned int id)
{
	int retry = 3, cnt = 1000;
	void __iomem *base;
	if (id < 0 || id >= HOST_MAX_NUM)
		return;
	base = mtk_msdc_host[id]->base;

	if (MSDC_READ32(MSDC_DMA_CFG) & MSDC_DMA_CFG_STS) {
		pr_err("<<<WARN>>>: msdc%d, clear FIFO when DMA active, MSDC_DMA_CFG=0x%x\n",
			id, MSDC_READ32(MSDC_DMA_CFG));
		show_stack(current, NULL);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_STOP, 1);
		msdc_retry((MSDC_READ32(MSDC_DMA_CFG) & MSDC_DMA_CFG_STS),
			retry, cnt, id);
		if (retry == 0) {
			pr_err("<<<WARN>>>: msdc%d, faield to stop DMA before clear FIFO, MSDC_DMA_CFG=0x%x\n",
				id, MSDC_READ32(MSDC_DMA_CFG));
			return;
		}
	}

	retry = 3;
	cnt = 1000;
	MSDC_SET_BIT32(MSDC_FIFOCS, MSDC_FIFOCS_CLR);
	msdc_retry(MSDC_READ32(MSDC_FIFOCS) & MSDC_FIFOCS_CLR, retry, cnt, id);
}

int msdc_clk_stable(struct msdc_host *host, u32 mode, u32 div,
	u32 hs400_div_dis)
{
	void __iomem *base = host->base;
	int retry = 0;
	int cnt = 1000;
	int retry_cnt = 1;

#if defined(CFG_DEV_MSDC3)
	/*FIXME: check function instead of host->id*/
	/*MSDC3 is dedicated for C2K, need special clock setting*/
	if (host->id == 3) {
		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_CKDIV, 0);
		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD, 1);
		MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_SDR104CKS, 1);
		return 0;
	}
#endif

	do {
		retry = 3;
		MSDC_SET_FIELD(MSDC_CFG,
			MSDC_CFG_CKMOD_HS400 | MSDC_CFG_CKMOD | MSDC_CFG_CKDIV,
			(hs400_div_dis << 14) | (mode << 12) |
				((div + retry_cnt) % 0xfff));
		/* MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD, mode); */
		msdc_retry(!(MSDC_READ32(MSDC_CFG) & MSDC_CFG_CKSTB), retry,
			cnt, host->id);
		if (retry == 0) {
			pr_err("msdc%d on clock failed ===> retry twice\n",
				host->id);

			msdc_clk_disable(host);
			msdc_clk_enable(host);
		}
		retry = 3;
		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_CKDIV, div);
		msdc_retry(!(MSDC_READ32(MSDC_CFG) & MSDC_CFG_CKSTB), retry,
			cnt, host->id);
		if (retry == 0)
			msdc_dump_info(host->id);
		msdc_reset_hw(host->id);
		if (retry_cnt == 2)
			break;
		retry_cnt++;
	} while (!retry);

	return 0;
}

#define msdc_irq_save(val) \
	do { \
		val = MSDC_READ32(MSDC_INTEN); \
		MSDC_CLR_BIT32(MSDC_INTEN, val); \
	} while (0)

#define msdc_irq_restore(val) \
	MSDC_SET_BIT32(MSDC_INTEN, val); \

/* set start bit of data sampling */
void msdc_set_startbit(struct msdc_host *host, u8 start_bit)
{
	void __iomem *base = host->base;

	/* set start bit */
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_START_BIT, start_bit);
	/* ERR_MSG("finished, start_bit=%d\n", start_bit); */
}

/* set the edge of data sampling */
void msdc_set_smpl(struct msdc_host *host, u8 HS400, u8 mode, u8 type, u8 *edge)
{
	void __iomem *base = host->base;
	int i = 0;

	switch (type) {
	case TYPE_CMD_RESP_EDGE:
		if (HS400) {
			MSDC_SET_FIELD(EMMC50_CFG0,
				MSDC_EMMC50_CFG_PADCMD_LATCHCK, 0);
			MSDC_SET_FIELD(EMMC50_CFG0,
				MSDC_EMMC50_CFG_CMD_RESP_SEL, 0);
		}

		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
#if 0
			if (HS400) {
				MSDC_SET_FIELD(EMMC50_CFG0,
					MSDC_EMMC50_CFG_CMD_EDGE_SEL, mode);
			} else {
				MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_RSPL,
					mode);
			}
#else
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_RSPL, mode);
#endif
		} else {
			ERR_MSG("invalid resp parameter: HS400=%d, type=%d, "
				"mode=%d\n", HS400, type, mode);
		}
		break;
	case TYPE_WRITE_CRC_EDGE:
		if (HS400) {
			/*latch write crc status at DS pin*/
			MSDC_SET_FIELD(EMMC50_CFG0,
				MSDC_EMMC50_CFG_CRC_STS_SEL, 1);
		} else {
			/*latch write crc status at CLK pin*/
			MSDC_SET_FIELD(EMMC50_CFG0,
				MSDC_EMMC50_CFG_CRC_STS_SEL, 0);
		}
		#if (TUNE_METHOD == DATA_TUNE)
		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
			if (HS400) {
				MSDC_SET_FIELD(EMMC50_CFG0,
					MSDC_EMMC50_CFG_CRC_STS_EDGE, mode);
			} else {
				MSDC_SET_FIELD(MSDC_PATCH_BIT2,
					MSDC_PB2_CFGCRCSTSEDGE, mode);
			}
		} else if ((mode == MSDC_SMPL_SEPERATE) &&
			   (edge != NULL) &&
			   (sizeof(edge) == 8)) {
			pr_err("Shall not enter here\n");

		} else {
			ERR_MSG("invalid crc parameter: HS400=%d, type=%d, "
				"mode=%d\n", HS400, type, mode);
		}
		#elif (TUNE_METHOD == CLOCK_TUNE)
		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
			if (HS400) {
				MSDC_SET_FIELD(EMMC50_CFG0,
					MSDC_EMMC50_CFG_CRC_STS_EDGE, mode);
			} else {
				MSDC_SET_FIELD(MSDC_IOCON,
					MSDC_IOCON_W_D_SMPL_SEL, 0);
				MSDC_SET_FIELD(MSDC_IOCON,
					MSDC_IOCON_W_D_SMPL, mode);
			}
		} else if ((mode == MSDC_SMPL_SEPERATE) &&
			   !HS400 &&
			   (edge != NULL)) {
			/*only dat0 is for write crc status.*/
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_W_D0SPL,
				edge[0]);
		} else {
			ERR_MSG("invalid crc parameter: HS400=%d, type=%d, "
				"mode=%d\n", HS400, type, mode);
		}
		#endif
		break;

	case TYPE_READ_DATA_EDGE:
		if (HS400) {
			/*for HS400, start bit is output on both edge*/
			msdc_set_startbit(host, START_AT_RISING_AND_FALLING);
		} else {
			/*for the other modes, start bit is only output on
			 rising edge; but DDR50 can try falling edge
			 if error casued by pad delay*/
			msdc_set_startbit(host, START_AT_RISING);
		}

		#if (TUNE_METHOD == DATA_TUNE)
		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_R_D_SMPL_SEL, 0);
			MSDC_SET_FIELD(MSDC_PATCH_BIT0,
				MSDC_PB0_RD_DAT_SEL, mode);
		} else if ((mode == MSDC_SMPL_SEPERATE) &&
			   (edge != NULL) &&
			   (sizeof(edge) == 8)) {
			pr_err("Shall not enter here\n");
		} else {
			ERR_MSG("invalid read parameter: HS400=%d, type=%d, "
				"mode=%d\n", HS400, type, mode);
		}

		#elif (TUNE_METHOD == CLOCK_TUNE)
		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_R_D_SMPL_SEL, 0);
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_R_D_SMPL, mode);

		} else if ((mode == MSDC_SMPL_SEPERATE) &&
			   (edge != NULL) &&
			   (sizeof(edge) == 8)) {
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_R_D_SMPL_SEL, 1);
			for (i = 0; i < 8; i++) {
				MSDC_SET_FIELD(MSDC_IOCON,
					(MSDC_IOCON_R_D0SPL << i), edge[i]);
			}
		} else {
			ERR_MSG("invalid read parameter: HS400=%d, type=%d, "
				"mode=%d\n", HS400, type, mode);
		}
		#endif
		break;

	case TYPE_WRITE_DATA_EDGE:
		/*latch write crc status at CLK pin*/
		MSDC_SET_FIELD(EMMC50_CFG0, MSDC_EMMC50_CFG_CRC_STS_SEL, 0);

		if (mode == MSDC_SMPL_RISING || mode == MSDC_SMPL_FALLING) {
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_W_D_SMPL_SEL, 0);
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_W_D_SMPL, mode);
		} else if ((mode == MSDC_SMPL_SEPERATE) &&
			   (edge != NULL) &&
			   (sizeof(edge) >= 4)) {
			MSDC_SET_FIELD(MSDC_IOCON, MSDC_IOCON_W_D_SMPL_SEL, 1);
			for (i = 0; i < 4; i++) {
				/*dat0~4 is for SDIO card*/
				MSDC_SET_FIELD(MSDC_IOCON,
					(MSDC_IOCON_W_D0SPL << i), edge[i]);
			}
		} else {
			ERR_MSG("invalid write parameter: HS400=%d, type=%d, "
				"mode=%d\n", HS400, type, mode);
		}
		break;
	default:
		ERR_MSG("invalid parameter: HS400=%d, type=%d, mode=%d\n",
			HS400, type, mode);
		break;
	}
	/*pr_err("finished, HS400=%d, type=%d, mode=%d\n", HS400, type, mode);*/

}

void msdc_set_smpl_all(struct msdc_host *host, u8 HS400, u8 ddr)
{
	struct msdc_hw *hw = host->hw;

	msdc_set_smpl(host, HS400, hw->cmd_edge, TYPE_CMD_RESP_EDGE, NULL);
	if (ddr == 0)
		msdc_set_smpl(host, HS400, hw->rdata_edge,
			TYPE_READ_DATA_EDGE, NULL);
	else
		msdc_set_smpl(host, HS400, 0, TYPE_READ_DATA_EDGE, NULL);
	msdc_set_smpl(host, HS400, hw->wdata_edge, TYPE_WRITE_CRC_EDGE, NULL);
}

/*sd card change voltage wait time= (1/freq) * SDC_VOL_CHG_CNT(default 0x145)*/
#define msdc_set_vol_change_wait_count(count) \
	MSDC_SET_FIELD(SDC_VOL_CHG, SDC_VOL_CHG_CNT, (count));

static void msdc_clksrc_onoff(struct msdc_host *host, u32 on)
{
	void __iomem *base = host->base;
	u32 div, mode, hs400_div_dis;
	if ((on) && (0 == host->core_clkon)) {
		#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		/*
		msdc_power_DL_CL_control(host, MSDC_POWER_DL_CL_FOR_HAS_LOAD);
		*/
		#endif

		msdc_clk_enable(host);

		host->core_clkon = 1;
		udelay(10);

		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_MODE, MSDC_SDMMC);

		MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD, mode);
		MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_CKDIV, div);
		MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD_HS400, hs400_div_dis);
		msdc_clk_stable(host, mode, div, hs400_div_dis);

	} else if ((!on)
		&& (!((host->hw->flags & MSDC_SDIO_IRQ) && src_clk_control))) {
		if (host->core_clkon == 1) {

			MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_MODE, MSDC_MS);

			msdc_clk_disable(host);

			host->core_clkon = 0;

			#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
			/* Bug: We cannot know if device is performning internal
			        operation */
			/*
			msdc_power_DL_CL_control(host,
				MSDC_POWER_DL_CL_FOR_NO_LOAD);
			*/
			#endif
		}
	}
}

/*host doesn't need the clock on*/
void msdc_gate_clock(struct msdc_host *host, int delay)
{
	unsigned long flags;
	unsigned int suspend;

	/*Use delay<0 for suspend purpose*/
	if (delay < 0) {
		suspend = 1;
		delay = 0;
	} else {
		suspend = 0;
	}

	spin_lock_irqsave(&host->clk_gate_lock, flags);
	if ((suspend == 0) && (host->clk_gate_count > 0))
		host->clk_gate_count--;
	if (delay) {
		mod_timer(&host->timer, jiffies + CLK_TIMEOUT);
		N_MSG(CLK, "[%s]: msdc%d, clk_gate_count=%d, delay=%d",
			__func__, host->id, host->clk_gate_count, delay);
	} else if (host->clk_gate_count == 0) {
		del_timer(&host->timer);
		msdc_clksrc_onoff(host, 0);
		N_MSG(CLK, "[%s]: msdc%d, clock gated done",
			__func__, host->id);
	} else {
		if (is_card_sdio(host))
			host->error = -EBUSY;
		ERR_MSG("[%s]: msdc%d, clock is needed, clk_gate_count=%d",
			__func__, host->id, host->clk_gate_count);
	}
	spin_unlock_irqrestore(&host->clk_gate_lock, flags);
}

/* host does need the clock on */
void msdc_ungate_clock(struct msdc_host *host)
{
	unsigned long flags;
	spin_lock_irqsave(&host->clk_gate_lock, flags);
	host->clk_gate_count++;
	N_MSG(CLK, "[%s]: msdc%d, clk_gate_count=%d", __func__, host->id,
		host->clk_gate_count);
	if (host->clk_gate_count == 1)
		msdc_clksrc_onoff(host, 1);
	spin_unlock_irqrestore(&host->clk_gate_lock, flags);
}

#if 0
static void msdc_dump_card_status(struct msdc_host *host, u32 status)
{
	static char *state[] = {
		"Idle",         /* 0 */
		"Ready",        /* 1 */
		"Ident",        /* 2 */
		"Stby",         /* 3 */
		"Tran",         /* 4 */
		"Data",         /* 5 */
		"Rcv",          /* 6 */
		"Prg",          /* 7 */
		"Dis",          /* 8 */
		"Reserved",     /* 9 */
		"Reserved",     /* 10 */
		"Reserved",     /* 11 */
		"Reserved",     /* 12 */
		"Reserved",     /* 13 */
		"Reserved",     /* 14 */
		"I/O mode",     /* 15 */
	};
	if (status & R1_OUT_OF_RANGE)
		N_MSG(RSP, "[CARD_STATUS] Out of Range");
	if (status & R1_ADDRESS_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Address Error");
	if (status & R1_BLOCK_LEN_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Block Len Error");
	if (status & R1_ERASE_SEQ_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Erase Seq Error");
	if (status & R1_ERASE_PARAM)
		N_MSG(RSP, "[CARD_STATUS] Erase Param");
	if (status & R1_WP_VIOLATION)
		N_MSG(RSP, "[CARD_STATUS] WP Violation");
	if (status & R1_CARD_IS_LOCKED)
		N_MSG(RSP, "[CARD_STATUS] Card is Locked");
	if (status & R1_LOCK_UNLOCK_FAILED)
		N_MSG(RSP, "[CARD_STATUS] Lock/Unlock Failed");
	if (status & R1_COM_CRC_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Command CRC Error");
	if (status & R1_ILLEGAL_COMMAND)
		N_MSG(RSP, "[CARD_STATUS] Illegal Command");
	if (status & R1_CARD_ECC_FAILED)
		N_MSG(RSP, "[CARD_STATUS] Card ECC Failed");
	if (status & R1_CC_ERROR)
		N_MSG(RSP, "[CARD_STATUS] CC Error");
	if (status & R1_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Error");
	if (status & R1_UNDERRUN)
		N_MSG(RSP, "[CARD_STATUS] Underrun");
	if (status & R1_OVERRUN)
		N_MSG(RSP, "[CARD_STATUS] Overrun");
	if (status & R1_CID_CSD_OVERWRITE)
		N_MSG(RSP, "[CARD_STATUS] CID/CSD Overwrite");
	if (status & R1_WP_ERASE_SKIP)
		N_MSG(RSP, "[CARD_STATUS] WP Eraser Skip");
	if (status & R1_CARD_ECC_DISABLED)
		N_MSG(RSP, "[CARD_STATUS] Card ECC Disabled");
	if (status & R1_ERASE_RESET)
		N_MSG(RSP, "[CARD_STATUS] Erase Reset");
	if ((status & R1_READY_FOR_DATA) == 0)
		N_MSG(RSP, "[CARD_STATUS] Not Ready for Data");
	if (status & R1_SWITCH_ERROR)
		N_MSG(RSP, "[CARD_STATUS] Switch error");
	if (status & R1_APP_CMD)
		N_MSG(RSP, "[CARD_STATUS] App Command");

	N_MSG(RSP, "[CARD_STATUS] '%s' State", state[R1_CURRENT_STATE(status)]);
}
#endif

static void msdc_set_timeout(struct msdc_host *host, u32 ns, u32 clks)
{
	void __iomem *base = host->base;
	u32 timeout, clk_ns;
	u32 mode = 0;

	host->timeout_ns = ns;
	host->timeout_clks = clks;
	if (host->sclk == 0) {
		timeout = 0;
	} else {
		clk_ns  = 1000000000UL / host->sclk;
		timeout = (ns + clk_ns - 1) / clk_ns + clks;
		/* in 1048576 sclk cycle unit */
		timeout = (timeout + (1 << 20) - 1) >> 20;
		MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD, mode);
		/*DDR mode will double the clk cycles for data timeout*/
		timeout = mode >= 2 ? timeout * 2 : timeout;
		timeout = timeout > 1 ? timeout - 1 : 0;
		timeout = timeout > 255 ? 255 : timeout;
	}
	MSDC_SET_FIELD(SDC_CFG, SDC_CFG_DTOC, timeout);

	N_MSG(OPS, "msdc%d, Set read data timeout: %dns %dclks -> "
		"%d x 1048576 cycles, mode:%d, clk_freq=%dKHz\n",
		host->id, ns, clks, timeout + 1, mode, (host->sclk / 1000));
}

#ifdef CONFIG_EMMC_50_FEATURE
#define msdc_set_caps_speed(mmc, hw) \
	if (hw->flags & MSDC_HIGHSPEED) \
		mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED; \
	if (hw->flags & MSDC_UHS1) { \
		mmc->caps |= MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 | \
				   MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR104; \
		mmc->caps2 |= MMC_CAP2_HS200_1_8V_SDR; \
	} \
	if (hw->flags & MSDC_HS400) \
		mmc->caps2 |= MMC_CAP2_HS400_1_8V_DDR; \
	if (hw->flags & MSDC_DDR) \
		mmc->caps |= MMC_CAP_1_8V_DDR; /* MMC_CAP_UHS_DDR50 jade disabled. */
#else
#define msdc_set_caps_speed(mmc, hw) \
	if (hw->flags & MSDC_HIGHSPEED) \
		mmc->caps | = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED; \
	if (hw->flags & MSDC_UHS1) { \
		mmc->caps |= MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 | \
				   MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR104; \
		mmc->caps2 |= MMC_CAP2_HS200_1_8V_SDR; \
	} \
	if (hw->flags & MSDC_DDR) \
		mmc->caps |= MMC_CAP_1_8V_DDR; /* MMC_CAP_UHS_DDR50 jade disabled. */
#endif


/* msdc_eirq_sdio() will be called when EIRQ(for WIFI) */
static void msdc_eirq_sdio(void *data)
{
	struct msdc_host *host = (struct msdc_host *)data;

	N_MSG(INT, "SDIO EINT");
#ifdef SDIO_ERROR_BYPASS
	if (host->sdio_error != -EIO) {
#endif
		mmc_signal_sdio_irq(host->mmc);
#ifdef SDIO_ERROR_BYPASS
	}
#endif
}

/* msdc_eirq_cd will not be used!  We do not use EINT for card detection. */
static void msdc_eirq_cd(void *data)
{
	struct msdc_host *host = (struct msdc_host *)data;

	N_MSG(INT, "CD EINT");

	tasklet_hi_schedule(&host->card_tasklet);
}

/* detect cd interrupt */
static void msdc_tasklet_card(unsigned long arg)
{
	struct msdc_host *host = (struct msdc_host *)arg;
	struct msdc_hw *hw = host->hw;
	/* unsigned long flags; */
	/* void __iomem *base = host->base; */
	u32 inserted;
	/* u32 status = 0; */

	if (hw->get_cd_status) {        /* NULL */
		inserted = hw->get_cd_status();
	} else {
		/* status = MSDC_READ32(MSDC_PS); */
		inserted = (host->sd_cd_polarity == hw->cd_level) ? 0 : 1;
	}
	if (host->block_bad_card) {
		inserted = 0;
		if (host->mmc->card)
			mmc_card_set_removed(host->mmc->card);
		ERR_MSG("remove bad SD card");
	} else {
		ERR_MSG("card found<%s>", inserted ? "inserted" : "removed");
	}

	host->card_inserted = inserted;
	host->mmc->f_max = HOST_MAX_MCLK;
	host->first_tune_done = 0;
	#if 0
	host->hw->cmd_edge = 0; /* new card tuning from 0*/
	host->hw->rdata_edge = 0;
	host->hw->wdata_edge = 0;
	#endif
	
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2016.01.12 for fat 733813	
	if (host->hw->host_function == MSDC_SD)
		host->hw->cmd_edge = 0;
#endif /* VENDOR_EDIT */

	msdc_set_caps_speed(host->mmc, hw);

	msdc_host_mode[host->id] = host->mmc->caps;
	msdc_host_mode2[host->id] = host->mmc->caps2;

	if ((hw->flags & MSDC_CD_PIN_EN) && inserted) {
		msdc_reset_pwr_cycle_counter(host);
		msdc_reset_crc_tune_counter(host, all_counter);
		msdc_reset_tmo_tune_counter(host, all_counter);
		host->is_sd_autok_done = 0;
	}

	ERR_MSG("host->suspend(%d)", host->suspend);
	if (!host->suspend && host->sd_cd_insert_work)
		mmc_detect_change(host->mmc, msecs_to_jiffies(200));
	ERR_MSG("insert_workqueue(%d)", host->sd_cd_insert_work);
}


volatile int sdio_autok_processed = 0;

void msdc_set_mclk(struct msdc_host *host, unsigned char timing, u32 hz)
{
	struct msdc_hw *hw = host->hw;
	void __iomem *base = host->base;
	u32 mode;
	u32 flags;
	u32 div;
	u32 sclk;
	u32 hclk = host->hclk;
	u32 hs400_div_dis = 0; /* FOR MSDC_CFG.HS400CKMOD */
	u8  clksrc = hw->clk_src;

	if (!hz) { /* set mmc system clock to 0*/
		pr_err("msdc%d -> set mclk to 0", host->id);
		if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ)) {
			host->saved_para.hz = hz;
#ifdef SDIO_ERROR_BYPASS
			host->sdio_error = 0;
#endif
		}
		host->mclk = 0;
		msdc_reset_hw(host->id);
		return;
	}

	msdc_irq_save(flags);

	if (timing == MMC_TIMING_MMC_HS400) {
		mode = 0x3; /* HS400 mode */
		/* FIX ME: change to use hclk */
		#if 1
		if (clksrc == MSDC50_CLKSRC_400MHZ) {
		#else
		if (hz >= hclk) {
		#endif
			hs400_div_dis = 1;
			div = 0;
			sclk = hclk/2;
		} else {
			hs400_div_dis = 0;
			if (hz >= (hclk >> 2)) {
				div  = 0;         /* mean div = 1/4 */
				sclk = hclk >> 2; /* sclk = clk / 4 */
			} else {
				div  = (hclk + ((hz << 2) - 1)) / (hz << 2);
				sclk = (hclk >> 2) / div;
				div  = (div >> 1);
			}
		}
	} else if (timing == MMC_TIMING_UHS_DDR50) {
		mode = 0x2; /* ddr mode and use divisor */
		if (hz >= (hclk >> 2)) {
			div  = 0;         /* mean div = 1/4 */
			sclk = hclk >> 2; /* sclk = clk / 4 */
		} else {
			div  = (hclk + ((hz << 2) - 1)) / (hz << 2);
			sclk = (hclk >> 2) / div;
			div  = (div >> 1);
		}
#if !defined(FPGA_PLATFORM)
	} else if (hz >= hclk) {
		mode = 0x1; /* no divisor */
		div  = 0;
		sclk = hclk;
#endif
	} else {
		mode = 0x0; /* use divisor */
		if (hz >= (hclk >> 1)) {
			div  = 0;         /* mean div = 1/2 */
			sclk = hclk >> 1; /* sclk = clk / 2 */
		} else {
			div  = (hclk + ((hz << 2) - 1)) / (hz << 2);
			sclk = (hclk >> 2) / div;
		}
	}

	msdc_clk_stable(host, mode, div, hs400_div_dis);

	host->sclk = sclk;
	host->mclk = hz;

	/* need because clk changed.*/
	msdc_set_timeout(host, host->timeout_ns, host->timeout_clks);

	if (mode == 0x3)
		msdc_set_smpl_all(host, 1, 0);
	else
		msdc_set_smpl_all(host, 0, ((mode == 2) ? 1 : 0));

	pr_err("msdc%d -> !!! Set<%dKHz> Source<%dKHz> -> sclk<%dKHz> timing<%d> mode<%d> div<%d> hs400_div_dis<%d>",
		host->id, hz/1000, hclk/1000, sclk/1000, (int)timing, mode, div,
		hs400_div_dis);

	msdc_irq_restore(flags);
}

void msdc_send_stop(struct msdc_host *host)
{
	struct mmc_command stop = {0};
	struct mmc_request mrq = {0};
	u32 err = -1;

	stop.opcode = MMC_STOP_TRANSMISSION;
	stop.arg = 0;
	stop.flags = MMC_RSP_R1B | MMC_CMD_AC;

	mrq.cmd = &stop;
	stop.mrq = &mrq;
	stop.data = NULL;

	err = msdc_do_command(host, &stop, 0, CMD_TIMEOUT);
}

static int msdc_app_cmd(struct mmc_host *mmc, struct msdc_host *host)
{
	struct mmc_command cmd = { 0 };
	struct mmc_request mrq = { 0 };
	u32 err = -1;

	cmd.opcode = MMC_APP_CMD;
	cmd.arg = host->app_cmd_arg;    /* meet mmc->card is null when ACMD6 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;

	mrq.cmd = &cmd;
	cmd.mrq = &mrq;
	cmd.data = NULL;

	err = msdc_do_command(host, &cmd, 0, CMD_TIMEOUT);
	return err;
}

int msdc_get_card_status(struct mmc_host *mmc, struct msdc_host *host,
	u32 *status)
{
	struct mmc_command cmd;
	struct mmc_request mrq;
	u32 err;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SEND_STATUS;   /* CMD13 */
	cmd.arg = host->app_cmd_arg;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;

	memset(&mrq, 0, sizeof(struct mmc_request));
	mrq.cmd = &cmd;
	cmd.mrq = &mrq;
	cmd.data = NULL;

	/* tune until CMD13 pass. */
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (host->mmc->card->ext_csd.cmdq_mode_en)
		err = msdc_do_cmdq_command(host, &cmd, 0, CMD_TIMEOUT);
	else
#endif
	err = msdc_do_command(host, &cmd, 0, CMD_TIMEOUT);

	if (status)
		*status = cmd.resp[0];

	return err;
}

#if 0
static void msdc_remove_card(struct work_struct *work)
{
	struct msdc_host *host = container_of(work, struct msdc_host,
		remove_card.work);

	BUG_ON(!host);
	BUG_ON(!(host->mmc));
	ERR_MSG("Need remove card");
	if (host->mmc->card) {
		if (mmc_card_present(host->mmc->card)) {
			ERR_MSG("1.remove card");
			mmc_remove_card(host->mmc->card);
		} else {
			ERR_MSG("card was not present can not remove");
			host->block_bad_card = 0;
			host->card_inserted = 1;
			host->mmc->card->state &= ~MMC_CARD_REMOVED;
			return;
		}
		mmc_claim_host(host->mmc);
		ERR_MSG("2.detach bus");
		host->mmc->card = NULL;
		mmc_detach_bus(host->mmc);
		ERR_MSG("3.Power off");
		mmc_power_off(host->mmc);
		ERR_MSG("4.Gate clock");
		msdc_gate_clock(host, 0);
		mmc_release_host(host->mmc);
	}
	ERR_MSG("Card removed");
}
#endif
int msdc_reinit(struct msdc_host *host)
{
	struct mmc_host *mmc;
	struct mmc_card *card;
	/*struct mmc_request *mrq;*/
	int ret = -1;
	u32 err = 0;
	u32 status = 0;
	unsigned long tmo = 12;
	/*u32 state = 0;*/
	if (!host) {
		ERR_MSG("msdc_host is NULL");
		return -1;
	}
	mmc = host->mmc;
	if (!mmc) {
		ERR_MSG("mmc is NULL");
		return -1;
	}

	card = mmc->card;
	if (card == NULL)
		ERR_MSG("mmc->card is NULL");
	if (host->block_bad_card)
		ERR_MSG("Need block this bad SD card from re-initialization");

#ifdef CONFIG_MTK_EMMC_SUPPORT
	if (host->hw->host_function == MSDC_EMMC)
		return -1;
#endif
	if (host->hw->host_function != MSDC_SD)
		goto skip_reinit2;

	if ((host->hw->flags & MSDC_CD_PIN_EN) || (host->block_bad_card != 0))
		goto skip_reinit1;

	/* power cycle */
	ERR_MSG("SD card Re-Init!");
	mmc_claim_host(host->mmc);
	ERR_MSG("SD card Re-Init get host!");
	spin_lock(&host->lock);
	ERR_MSG("SD card Re-Init get lock!");
	msdc_clksrc_onoff(host, 1);
	if (host->app_cmd_arg) {
		while ((err = msdc_get_card_status(mmc, host, &status))) {
			ERR_MSG("SD card Re-Init in get card status!err(%d)",
				err);
			if (err == (unsigned int)-EIO) {
				if (msdc_tune_cmdrsp(host)) {
					ERR_MSG("update cmd para failed");
					break;
				}
			} else {
				break;
			}
		}
		if (err == 0) {
			if (status == 0) {
				msdc_dump_info(host->id);
			} else {
				msdc_clksrc_onoff(host, 0);
				spin_unlock(&host->lock);
				mmc_release_host(host->mmc);
				ERR_MSG("SD Card is ready");
				return 0;
			}
		}
	}
	msdc_clksrc_onoff(host, 0);
	ERR_MSG("Reinit start..");
	mmc->ios.clock = HOST_MIN_MCLK;
	mmc->ios.bus_width = MMC_BUS_WIDTH_1;
	mmc->ios.timing = MMC_TIMING_LEGACY;
	host->card_inserted = 1;
	msdc_clksrc_onoff(host, 1);
	msdc_set_mclk(host, MMC_TIMING_LEGACY, HOST_MIN_MCLK);
	msdc_clksrc_onoff(host, 0);
	spin_unlock(&host->lock);
	mmc_release_host(host->mmc);
	if (host->mmc->card) {
		mmc_remove_card(host->mmc->card);
		host->mmc->card = NULL;
		mmc_claim_host(host->mmc);
		mmc_detach_bus(host->mmc);
		mmc_release_host(host->mmc);
	}
	mmc_power_off(host->mmc);
	mmc_detect_change(host->mmc, 0);
	while (tmo) {
		if (host->mmc->card && mmc_card_present(host->mmc->card)) {
			ret = 0;
			break;
		}
		msleep(50);
		tmo--;
	}
	ERR_MSG("Reinit %s", ret == 0 ? "success" : "fail");

skip_reinit1:
	if ((host->hw->flags & MSDC_CD_PIN_EN) && (host->mmc->card)
		&& mmc_card_present(host->mmc->card)
		&& (!mmc_card_removed(host->mmc->card))
		&& (host->block_bad_card == 0))
		ret = 0;
skip_reinit2:
	return ret;
}

static u32 msdc_polling_idle(struct msdc_host *host)
{
	struct mmc_host *mmc = host->mmc;
	u32 status = 0;
	u32 state = 0;
	u32 err = 0;
	unsigned long tmo = jiffies + POLLING_BUSY;

	while (state != 4) {    /* until status to "tran" */
		while ((err = msdc_get_card_status(mmc, host, &status))) {
			ERR_MSG("CMD13 ERR<%d>", err);
			if (err != (unsigned int)-EIO) {
				return msdc_power_tuning(host);
			} else if (msdc_tune_cmdrsp(host)) {
				ERR_MSG("update cmd para failed");
				return 1;
			}
		}

		state = R1_CURRENT_STATE(status);
		/* ERR_MSG("check card state<%d>", state); */
		if (state == 5 || state == 6) {
			ERR_MSG("state<%d> need cmd12 to stop", state);
			msdc_send_stop(host);   /* don't tuning */
		} else if (state == 7) {        /* busy in programing */
			ERR_MSG("state<%d> card is busy", state);
			spin_unlock(&host->lock);
			msleep(100);
			spin_lock(&host->lock);
		} else if (state != 4) {
			ERR_MSG("state<%d> ??? ", state);
			return msdc_power_tuning(host);
		}

		if (time_after(jiffies, tmo)) {
			ERR_MSG("abort timeout. Do power cycle");
			return msdc_power_tuning(host);
		}
	}
	return 0;
}

void msdc_pin_reset(struct msdc_host *host, int mode, int force_reset)
{
	struct msdc_hw *hw = (struct msdc_hw *)host->hw;
	void __iomem *base = host->base;
	int pull = (mode == MSDC_PIN_PULL_UP) ? MSDC_GPIO_PULL_UP :
		MSDC_GPIO_PULL_DOWN;

	/* Config reset pin */
	if ((hw->flags & MSDC_RST_PIN_EN) || force_reset) {
		if (hw->config_gpio_pin)        /* NULL */
			hw->config_gpio_pin(MSDC_RST_PIN, pull);

		if (mode == MSDC_PIN_PULL_UP)
			MSDC_CLR_BIT32(EMMC_IOCON, EMMC_IOCON_BOOTRST);
		else
			MSDC_SET_BIT32(EMMC_IOCON, EMMC_IOCON_BOOTRST);
	}
}

static void msdc_set_power_mode(struct msdc_host *host, u8 mode)
{
	u32 val;

	N_MSG(CFG, "Set power mode(%d)", mode);
	if (host->power_mode == MMC_POWER_OFF && mode != MMC_POWER_OFF) {
		MVG_EMMC_RESET();
		msdc_pin_reset(host, MSDC_PIN_PULL_UP, 0);
		msdc_pin_config(host, MSDC_PIN_PULL_UP);

		if (host->power_control)
			host->power_control(host, 1);

		mdelay(10);

		if (host->id == 1) {
			pmic_read_interface(REG_VMCH_OC_STATUS,
					    &val,
					    MASK_VMCH_OC_STATUS,
					    SHIFT_VMCH_OC_STATUS
					    );

			if (val) {
				pr_err("msdc1 OC status = %x\n", val);
				host->power_control(host, 0);
			}
		}

	} else if (host->power_mode != MMC_POWER_OFF && mode == MMC_POWER_OFF) {

		if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ)) {
			msdc_pin_config(host, MSDC_PIN_PULL_UP);
		} else {

			if (host->power_control)
				host->power_control(host, 0);

			msdc_pin_config(host, MSDC_PIN_PULL_DOWN);
		}
		mdelay(20);
		msdc_pin_reset(host, MSDC_PIN_PULL_DOWN, 0);
	}
	host->power_mode = mode;
}

/* FIX ME: check if can be remved*/
typedef enum MMC_STATE_TAG {
	MMC_IDLE_STATE,
	MMC_READ_STATE,
	MMC_IDENT_STATE,
	MMC_STBY_STATE,
	MMC_TRAN_STATE,
	MMC_DATA_STATE,
	MMC_RCV_STATE,
	MMC_PRG_STATE,
	MMC_DIS_STATE,
	MMC_BTST_STATE,
	MMC_SLP_STATE,
	MMC_RESERVED1_STATE,
	MMC_RESERVED2_STATE,
	MMC_RESERVED3_STATE,
	MMC_RESERVED4_STATE,
	MMC_RESERVED5_STATE,
} MMC_STATE_T;

#ifdef CONFIG_PM
static void msdc_pm(pm_message_t state, void *data)
{
	struct msdc_host *host = (struct msdc_host *)data;
	void __iomem *base = host->base;

	int evt = state.event;

	msdc_ungate_clock(host);
	if (host->hw->host_function == MSDC_EMMC)
		emmc_do_sleep_awake = 1;

	if (evt == PM_EVENT_SUSPEND || evt == PM_EVENT_USER_SUSPEND) {
		if (host->suspend)
			goto end;

		if (evt == PM_EVENT_SUSPEND &&
		     host->power_mode == MMC_POWER_OFF)
			goto end;

		host->suspend = 1;
		host->pm_state = state;

		pr_err("msdc%d -> %s Suspend", host->id,
			evt == PM_EVENT_SUSPEND ? "PM" : "USR");
		if (host->hw->flags & MSDC_SYS_SUSPEND) {
#ifdef CONFIG_MTK_EMMC_SUPPORT
			if (host->hw->host_function == MSDC_EMMC &&
			    host->mmc->card &&
			    mmc_card_mmc(host->mmc->card)) {
				if (g_emmc_mode_switch == 0) {
					host->mmc->pm_flags |=
						MMC_PM_KEEP_POWER;
				} else {
					host->mmc->pm_flags &=
						(~MMC_PM_KEEP_POWER);
				}
			}
#else
			if (host->hw->host_function == MSDC_EMMC &&
			    host->mmc->card &&
			    mmc_card_mmc(host->mmc->card))
				host->mmc->pm_flags |= MMC_PM_KEEP_POWER;
#endif
			emmc_sleep_failed = 0;
			(void)mmc_suspend_host(host->mmc);

#ifdef CONFIG_MTK_EMMC_SUPPORT
			if ((g_emmc_mode_switch == 0) &&
			     host->hw->host_function == MSDC_EMMC &&
			     host->mmc->card &&
			     mmc_card_mmc(host->mmc->card)) {
				msdc_save_timing_setting(host, 0, 1, 0, 0, 0);
				msdc_set_power_mode(host, MMC_POWER_OFF);
			}
#else
			if (host->hw->host_function == MSDC_EMMC &&
			    host->mmc->card &&
			    mmc_card_mmc(host->mmc->card)) {
				msdc_save_timing_setting(host, 0, 1, 0, 0, 0);
				msdc_set_power_mode(host, MMC_POWER_OFF);
			}
#endif

			msdc_set_tdsel(host, 1, 0);

		} else {
			host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
			mmc_remove_host(host->mmc);
		}
	} else if (evt == PM_EVENT_RESUME || evt == PM_EVENT_USER_RESUME) {
		if (!host->suspend)
			goto end;

		if (evt == PM_EVENT_RESUME
			&& host->pm_state.event == PM_EVENT_USER_SUSPEND) {
			ERR_MSG("PM Resume when in USR Suspend");
			goto end;
		}

		host->suspend = 0;
		host->pm_state = state;

		pr_err("msdc%d -> %s Resume", host->id,
			evt == PM_EVENT_RESUME ? "PM" : "USR");

		if (!(host->hw->flags & MSDC_SYS_SUSPEND)) {
			host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
			mmc_add_host(host->mmc);
			goto end;
		}

		/* Begin for host->hw->flags & MSDC_SYS_SUSPEND*/
		msdc_set_tdsel(host, 1, 0);

#ifdef CONFIG_MTK_EMMC_SUPPORT
		if ((g_emmc_mode_switch == 0) &&
		    host->hw->host_function == MSDC_EMMC &&
		    host->mmc->card &&
		    mmc_card_mmc(host->mmc->card)) {
			msdc_reset_hw(host->id);
			msdc_set_power_mode(host, MMC_POWER_ON);
			msdc_restore_timing_setting(host);
			if (emmc_sleep_failed) {
				msdc_pin_reset(host, MSDC_PIN_PULL_DOWN, 1);
				msdc_pin_reset(host, MSDC_PIN_PULL_UP, 1);
				mdelay(200);
				mmc_card_clr_sleep(host->mmc->card);
				host->mmc->pm_flags &= ~MMC_PM_KEEP_POWER;
			}
		}
#else
		if (host->hw->host_function == MSDC_EMMC &&
		    host->mmc->card && mmc_card_mmc(host->mmc->card)) {
			msdc_reset_hw(host->id);
			msdc_set_power_mode(host, MMC_POWER_ON);
			msdc_restore_timing_setting(host);
			if (emmc_sleep_failed) {
				msdc_pin_reset(host, MSDC_PIN_PULL_DOWN, 1);
				msdc_pin_reset(host, MSDC_PIN_PULL_UP, 1);
				mdelay(200);
				mmc_card_clr_sleep(host->mmc->card);
				host->mmc->pm_flags &= ~MMC_PM_KEEP_POWER;
			}
		}
#endif
		(void)mmc_resume_host(host->mmc);
		if ((host->hw->host_function == MSDC_EMMC)
		 && emmc_sleep_failed)
			emmc_sleep_failed = 0;
		/* End for host->hw->flags & MSDC_SYS_SUSPEND*/
	}

end:
#ifdef SDIO_ERROR_BYPASS
	if (is_card_sdio(host))
		host->sdio_error = 0;
#endif
	if ((evt == PM_EVENT_SUSPEND) || (evt == PM_EVENT_USER_SUSPEND)) {
		if ((host->hw->host_function == MSDC_SDIO) &&
		    (evt == PM_EVENT_USER_SUSPEND)) {
			pr_err("msdc%d -> MSDC Device Request Suspend",
				host->id);
		}
		msdc_gate_clock(host, 0);
	} else {
		msdc_gate_clock(host, 1);
	}
	if (host->hw->host_function == MSDC_EMMC)
		emmc_do_sleep_awake = 0;
}
#endif

struct msdc_host *msdc_get_host(int host_function, bool boot, bool secondary)
{
	int host_index = 0;
	struct msdc_host *host = NULL, *host2;

	for (; host_index < HOST_MAX_NUM; ++host_index) {
		host2 = mtk_msdc_host[host_index];
		if (!host2)
			continue;
		if ((host_function == host2->hw->host_function) &&
		    (boot == host2->hw->boot)) {
			host = host2;
			break;
		}
	}
	if (secondary && (host_function == MSDC_SD))
		host = mtk_msdc_host[2];
	if (host == NULL) {
		pr_err("[MSDC] This host(<host_function:%d> <boot:%d><secondary:%d>) isn't in MSDC host config list",
			 host_function, boot, secondary);
		/* BUG(); */
	}

	return host;
}
EXPORT_SYMBOL(msdc_get_host);

#ifdef CONFIG_MTK_EMMC_SUPPORT

int msdc_switch_part(struct msdc_host *host, char part_id)
{
	int ret = 0;
	struct mmc_card *card;

	if ((host != NULL) && (host->mmc != NULL) && (host->mmc->card != NULL))
		card = host->mmc->card;
	else
		return -ENOMEDIUM;

	if (mmc_card_mmc(card)) {
		u8 part_config = card->ext_csd.part_config;

		part_config &= ~EXT_CSD_PART_CONFIG_ACC_MASK;
		part_config |= part_id;

		ret = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_PART_CONFIG, part_config,
			card->ext_csd.part_time);
		if (ret)
			return ret;

		card->ext_csd.part_config = part_config;
	}

	return ret;
}

#ifdef MTK_MSDC_USE_CACHE
static int msdc_cache_ctrl(struct msdc_host *host, unsigned int enable,
	u32 *status)
{
	struct mmc_command cmd;
	struct mmc_request mrq;
	u32 err;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SWITCH;        /* CMD6 */
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24)
		| (EXT_CSD_CACHE_CTRL << 16) | (!!enable << 8)
		| EXT_CSD_CMD_SET_NORMAL;
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

	memset(&mrq, 0, sizeof(struct mmc_request));
	mrq.cmd = &cmd;
	cmd.mrq = &mrq;
	cmd.data = NULL;

	ERR_MSG("do disable Cache, cmd=0x%x, arg=0x%x\n", cmd.opcode, cmd.arg);
	/* tune until CMD13 pass. */
	err = msdc_do_command(host, &cmd, 0, CMD_TIMEOUT);

	if (status)
		*status = cmd.resp[0];
	if (!err) {
		host->mmc->card->ext_csd.cache_ctrl = !!enable;
		host->autocmd |= MSDC_AUTOCMD23;
		N_MSG(CHE, "enable AUTO_CMD23 because Cache feature is "
			"disabled\n");
	}

	return err;
}

static void msdc_update_cache_flush_status(struct msdc_host *host,
	struct mmc_request *mrq, struct mmc_data *data,
	u32 l_bypass_flush)
{
	struct mmc_command *cmd = mrq->cmd;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	struct mmc_command *sbc;
	unsigned int task_id;
#endif

	if (!check_mmc_cache_ctrl(host->mmc->card))
		return;

	if (check_mmc_cmd2425(cmd->opcode)) {
		if ((host->error == 0)
		 && mrq->sbc
		 && (((mrq->sbc->arg >> 24) & 0x1) ||
		     ((mrq->sbc->arg >> 31) & 0x1))) {
			/* if reliable write, or force prg write succeed,
			   do set cache flushed status */
			if (g_cache_status == CACHE_UN_FLUSHED) {
				g_cache_status = CACHE_FLUSHED;
				N_MSG(CHE, "reliable/force prg write happend, "
					"update g_cache_status = %d, "
					"g_flush_data_size=%lld",
					g_cache_status, g_flush_data_size);
				g_flush_data_size = 0;
			}
		} else if (host->error == 0) {
			/* if normal write succee,
			   do clear the cache flushed status */
			if (g_cache_status == CACHE_FLUSHED) {
				g_cache_status = CACHE_UN_FLUSHED;
				N_MSG(CHE, "normal write happend, "
					"update g_cache_status = %d",
					g_cache_status);
			}
			g_flush_data_size += data->blocks;
		} else if (host->error) {
			g_flush_data_size += data->blocks;
			ERR_MSG("write error happend, g_flush_data_size=%lld",
				g_flush_data_size);
		}
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	} else if (cmd->opcode == MMC_WRITE_REQUESTED_QUEUE) {
		if (host->error == 0) {
			task_id = (cmd->arg >> 16) & 0x1f;
			sbc = host->mmc->areq_que[task_id]->mrq_que->sbc;
			if (sbc	&& (((sbc->arg >> 24) & 0x1) ||
				((sbc->arg >> 31) & 0x1))) {
				/* if reliable write, or force prg write succeed,
				do set cache flushed status */
				if (g_cache_status == CACHE_UN_FLUSHED) {
					g_cache_status = CACHE_FLUSHED;
					N_MSG(CHE, "reliable/force prg write happend, update g_cache_status = %d",
						g_cache_status);
					N_MSG(CHE, "reliable/force prg write happend, update g_flush_data_size=%lld",
						g_flush_data_size);
				   g_flush_data_size = 0;
				}
			} else {
				/* if normal write succee,
				   do clear the cache flushed status */
				if (g_cache_status == CACHE_FLUSHED) {
					g_cache_status = CACHE_UN_FLUSHED;
					N_MSG(CHE, "normal write happend, update g_cache_status = %d",
						g_cache_status);
				}
				g_flush_data_size += data->blocks;
			}
		} else if (host->error) {
			g_flush_data_size += data->blocks;
			ERR_MSG("write error happend, g_flush_data_size=%lld",
				g_flush_data_size);
		}
#endif
	} else if (l_bypass_flush == 0) {
		if (host->error == 0) {
			/* if flush cache of emmc device successfully,
			   do set the cache flushed status */
			g_cache_status = CACHE_FLUSHED;
			N_MSG(CHE, "flush happend, update g_cache_status = %d,"
				" g_flush_data_size=%lld",
				g_cache_status, g_flush_data_size);
			g_flush_data_size = 0;
		} else {
			g_flush_error_happend = 1;
		}
	}
}

void msdc_check_cache_flush_error(struct msdc_host *host,
	struct mmc_command *cmd)
{
	if (g_flush_error_happend &&
	    check_mmc_cache_ctrl(host->mmc->card) &&
	    check_mmc_cache_flush_cmd(cmd)) {
		g_flush_error_count++;
		g_flush_error_happend = 0;
		ERR_MSG("the %d time flush error happned, "
			"g_flush_data_size=%lld",
			g_flush_error_count, g_flush_data_size);
		if (g_flush_error_count >= MSDC_MAX_FLUSH_COUNT) {
			if (!msdc_cache_ctrl(host, 0, NULL))
				host->mmc->caps2 &= ~MMC_CAP2_CACHE_CTRL;
			ERR_MSG("flush failed %d times, exceed max flush "
				"error count %d, disable cache feature",
				g_flush_error_count, MSDC_MAX_FLUSH_COUNT);
		}
	}
}
#endif
#endif

/*--------------------------------------------------------------------------*/
/* mmc_host_ops members                                                     */
/*--------------------------------------------------------------------------*/
static u32 wints_cmd = MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO |
		       MSDC_INT_ACMDRDY | MSDC_INT_ACMDCRCERR |
		       MSDC_INT_ACMDTMO;
static unsigned int msdc_command_start(struct msdc_host   *host,
	struct mmc_command *cmd,
	int                 tune,   /* not used */
	unsigned long       timeout)
{
	void __iomem *base = host->base;
	u32 opcode = cmd->opcode;
	u32 rawcmd;
	u32 rawarg;
	u32 resp;
	unsigned long tmo;
	struct mmc_command *sbc = NULL;
	char *str;
	
	//#ifdef VENDOR_EDIT //Haitao.Zhou@Prd.BaseDrv for RPMB patch 
	if (host->data && host->data->mrq && host->data->mrq->sbc && (host->autocmd & MSDC_AUTOCMD23))
		sbc = host->data->mrq->sbc;
	//#endif
	
	/* Protocol layer does not provide response type, but our hardware needs
	 * to know exact type, not just size!
	 */
	switch (opcode) {
	case MMC_SEND_OP_COND:
	case SD_APP_OP_COND:
		resp = RESP_R3;
		break;
	case MMC_SET_RELATIVE_ADDR:
	/*case SD_SEND_RELATIVE_ADDR:*/
		/* Since SD_SEND_RELATIVE_ADDR=MMC_SET_RELATIVE_ADDR=3,
		   only one is allowed in switch case.*/
		resp = (mmc_cmd_type(cmd) == MMC_CMD_BCR) ? RESP_R6 : RESP_R1;
		break;
	case MMC_FAST_IO:
		resp = RESP_R4;
		break;
	case MMC_GO_IRQ_STATE:
		resp = RESP_R5;
		break;
	case MMC_SELECT_CARD:
		resp = (cmd->arg != 0) ? RESP_R1 : RESP_NONE;
		host->app_cmd_arg = cmd->arg;
		pr_warn("msdc%d select card<0x%.8x>", host->id, cmd->arg);
		break;
	case SD_IO_RW_DIRECT:
	case SD_IO_RW_EXTENDED:
		/* SDIO workaround. */
		resp = RESP_R1;
		break;
	case SD_SEND_IF_COND:
		if (mmc_cmd_type(cmd) == MMC_CMD_BCR)
			resp = RESP_R1;
		break;
	default:
		switch (mmc_resp_type(cmd)) {
		case MMC_RSP_R1:
			resp = RESP_R1;
			break;
		case MMC_RSP_R1B:
			resp = RESP_R1B;
			break;
		case MMC_RSP_R2:
			resp = RESP_R2;
			break;
		case MMC_RSP_R3:
			resp = RESP_R3;
			break;
		case MMC_RSP_NONE:
		default:
			resp = RESP_NONE;
			break;
		}
	}

	cmd->error = 0;
	/* rawcmd :
	 * vol_swt << 30 | auto_cmd << 28 | blklen << 16 | go_irq << 15 |
	 * stop << 14 | rw << 13 | dtype << 11 | rsptyp << 7 | brk << 6 |
	 * opcode
	 */

	rawcmd = opcode | msdc_rsp[resp] << 7 | host->blksz << 16;

	switch (opcode) {
	case MMC_READ_MULTIPLE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
		rawcmd |= (2 << 11);
		if (opcode == MMC_WRITE_MULTIPLE_BLOCK)
			rawcmd |= (1 << 13);
		if (host->autocmd & MSDC_AUTOCMD12) {
			rawcmd |= (1 << 28);
			N_MSG(CMD, "AUTOCMD12 is set, addr<0x%x>", cmd->arg);
#ifdef MTK_MSDC_USE_CMD23
		} else if ((host->autocmd & MSDC_AUTOCMD23)) {
			unsigned int reg_blk_num;
			rawcmd |= (1 << 29);
			if (sbc) {
				/* if block number is greater than 0xFFFF,
				   CMD23 arg will fail to set it.*/
				reg_blk_num = MSDC_READ32(SDC_BLK_NUM);
				if (reg_blk_num != (sbc->arg & 0xFFFF))
					pr_err("msdc%d: acmd23 arg(0x%x) fail to match block num(0x%x), SDC_BLK_NUM(0x%x)\n",
						host->id, sbc->arg,
						host->mrq->cmd->data->blocks,
						reg_blk_num);
				else
					MSDC_WRITE32(SDC_BLK_NUM, sbc->arg);
				N_MSG(CMD, "AUTOCMD23 addr<0x%x>, arg<0x%x> ",
					cmd->arg, sbc->arg);
			}
#endif /* end of MTK_MSDC_USE_CMD23 */
		}
		break;

	case MMC_READ_SINGLE_BLOCK:
	case MMC_SEND_TUNING_BLOCK:
	case MMC_SEND_TUNING_BLOCK_HS200:
		rawcmd |= (1 << 11);
		break;
	case MMC_WRITE_BLOCK:
		rawcmd |= ((1 << 11) | (1 << 13));
#ifdef MTK_MSDC_USE_CACHE
		if (check_mmc_cache_ctrl(host->mmc->card))
			ERR_MSG("[Warning]: Single write(0x%x) happend "
				"when cache enabled", cmd->arg);
#endif
		break;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	case MMC_READ_REQUESTED_QUEUE:
		rawcmd |= (2 << 11);
		break;
	case MMC_WRITE_REQUESTED_QUEUE:
		rawcmd |= ((2 << 11) | (1 << 13));
		break;
	case MMC_CMDQ_TASK_MGMT:
		break;
#endif
	case SD_IO_RW_EXTENDED:
		if (cmd->data->flags & MMC_DATA_WRITE)
			rawcmd |= (1 << 13);
		if (cmd->data->blocks > 1)
			rawcmd |= (2 << 11);
		else
			rawcmd |= (1 << 11);
		break;
	case SD_IO_RW_DIRECT:
		if (cmd->flags == (unsigned int)-1)
			rawcmd |= (1 << 14);
		break;
	case SD_SWITCH_VOLTAGE:
		rawcmd |= (1 << 30);
		break;
	case SD_APP_SEND_SCR:
	case SD_APP_SEND_NUM_WR_BLKS:
		rawcmd |= (1 << 11);
		break;
	case SD_SWITCH:
	case SD_APP_SD_STATUS:
	case MMC_SEND_EXT_CSD:
		if (mmc_cmd_type(cmd) == MMC_CMD_ADTC)
			rawcmd |= (1 << 11);
		break;
	case MMC_STOP_TRANSMISSION:
		rawcmd |= (1 << 14);
		rawcmd &= ~(0x0FFF << 16);
		break;
	}

	N_MSG(CMD, "CMD<%d><0x%.8x> Arg<0x%.8x>", opcode, rawcmd, cmd->arg);

	tmo = jiffies + timeout;

	if (opcode == MMC_SEND_STATUS) {
		for (;;) {
			if (!sdc_is_cmd_busy())
				break;
			if (time_after(jiffies, tmo)) {
				str = "cmd_busy";
				goto err;
			}
		}
	} else {
		for (;;) {
			if (!sdc_is_busy())
				break;
			if (time_after(jiffies, tmo)) {
				str = "sdc_busy";
				goto err;
			}
		}
	}

	/* BUG_ON(in_interrupt()); */
	host->cmd = cmd;
	host->cmd_rsp = resp;

	/* use polling way */
	MSDC_CLR_BIT32(MSDC_INTEN, wints_cmd);
	rawarg = cmd->arg;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	dbg_add_host_log(host->mmc, 0, cmd->opcode, cmd->arg);
#endif

	sdc_send_cmd(rawcmd, rawarg);

/*end:*/
	/* irq too fast, then cmd->error has value,
	   and don't call msdc_command_resp, don't tune. */
	return 0;

err:
	ERR_MSG("XXX %s timeout: before CMD<%d>", str, opcode);
	cmd->error = (unsigned int)-ETIMEDOUT;
	msdc_dump_register(host);
	msdc_reset_hw(host->id);
	return cmd->error;

}

static u32 msdc_command_resp_polling(struct msdc_host *host,
	struct mmc_command *cmd,
	int                 tune,
	unsigned long       timeout)
{
	void __iomem *base = host->base;
	u32 intsts;
	u32 resp;
	unsigned long tmo;
	/* struct mmc_data   *data = host->data; */
	u32 cmdsts = MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO;
#ifdef MTK_MSDC_USE_CMD23
	struct mmc_command *sbc = NULL;
#endif

#ifdef MTK_MSDC_USE_CMD23
	if (host->autocmd & MSDC_AUTOCMD23) {
		if (host->data && host->data->mrq && host->data->mrq->sbc)
			sbc = host->data->mrq->sbc;

		/* autocmd interupt disabled, used polling way */
		cmdsts |= MSDC_INT_ACMDCRCERR | MSDC_INT_ACMDTMO;
	}
#endif

	resp = host->cmd_rsp;

	/*polling */
	tmo = jiffies + timeout;
	while (1) {
		intsts = MSDC_READ32(MSDC_INT);
		if ((intsts & cmdsts) != 0) {
			/* clear all int flag */
#ifdef MTK_MSDC_USE_CMD23
			/* need clear autocmd23 comand ready interrupt */
			intsts &= (cmdsts | MSDC_INT_ACMDRDY);
#else
			intsts &= cmdsts;
#endif
			MSDC_WRITE32(MSDC_INT, intsts);
			break;
		}

		if (time_after(jiffies, tmo)) {
			pr_err("[%s]: msdc%d CMD<%d> polling_for_completion timeout ARG<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
			cmd->error = (unsigned int)-ETIMEDOUT;
			host->sw_timeout++;
			msdc_dump_info(host->id);
			msdc_reset_hw(host->id);
			goto out;
		}
	}

#ifdef MTK_MSDC_ERROR_TUNE_DEBUG
	msdc_error_tune_debug1(host, cmd, sbc, &intsts);
#endif

	/* command interrupts */
	if  (!(intsts & cmdsts))
		goto out;

#ifdef MTK_MSDC_USE_CMD23
	if (intsts & (MSDC_INT_CMDRDY | MSDC_INT_ACMD19_DONE)) {
#else
	if (intsts & (MSDC_INT_CMDRDY | MSDC_INT_ACMD19_DONE
		| MSDC_INT_ACMDRDY)) {
#endif
		u32 *rsp = NULL;
		rsp = &cmd->resp[0];
		switch (host->cmd_rsp) {
		case RESP_NONE:
			break;
		case RESP_R2:
			*rsp++ = MSDC_READ32(SDC_RESP3);
			*rsp++ = MSDC_READ32(SDC_RESP2);
			*rsp++ = MSDC_READ32(SDC_RESP1);
			*rsp++ = MSDC_READ32(SDC_RESP0);
			break;
		default: /* Response types 1, 3, 4, 5, 6, 7(1b) */
			*rsp = MSDC_READ32(SDC_RESP0);
			/* workaround for latch error */
			if (((cmd->opcode == 13) || (cmd->opcode == 25))
			 && (*rsp & R1_OUT_OF_RANGE)
			 && (host->hw->host_function != MSDC_SDIO)) {
				pr_err("[%s]: msdc%d XXX CMD<%d> resp<0x%.8x>,bit31=1,force make crc error\n",
					__func__, host->id, cmd->opcode, *rsp);
				cmd->error = (unsigned int)-EIO;
			}
			break;
		}
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		dbg_add_host_log(host->mmc, 1, cmd->opcode, cmd->resp[0]);
#endif
	} else if (intsts & MSDC_INT_RSPCRCERR) {
		cmd->error = (unsigned int)-EIO;
		if ((cmd->opcode != 19) && (cmd->opcode != 21))
			pr_err("[%s]: msdc%d CMD<%d> MSDC_INT_RSPCRCERR Arg<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
		if (((MMC_RSP_R1B == mmc_resp_type(cmd)) || (cmd->opcode == 13))
			&& (host->hw->host_function != MSDC_SDIO)) {
			pr_err("[%s]: msdc%d CMD<%d> ARG<0x%.8X> is R1B, CRC not reset hw\n",
				__func__, host->id, cmd->opcode, cmd->arg);
		} else {
			msdc_reset_hw(host->id);
		}
	} else if (intsts & MSDC_INT_CMDTMO) {
		cmd->error = (unsigned int)-ETIMEDOUT;
		if ((cmd->opcode != 19) && (cmd->opcode != 21))
			pr_err("[%s]: msdc%d CMD<%d> MSDC_INT_CMDTMO Arg<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
		if ((cmd->opcode != 52) && (cmd->opcode != 8) &&
		    (cmd->opcode != 5) && (cmd->opcode != 55) &&
		    (cmd->opcode != 19) && (cmd->opcode != 21) &&
		    (cmd->opcode != 1)) {
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
			mmc_cmd_dump(host->mmc);
#endif
			msdc_dump_info(host->id);
		}
		if ((cmd->opcode == 5) && emmc_do_sleep_awake)
			msdc_dump_info(host->id);

		if (((MMC_RSP_R1B == mmc_resp_type(cmd)) || (cmd->opcode == 13))
			&& (host->hw->host_function != MSDC_SDIO)) {
			pr_err("[%s]: msdc%d XXX CMD<%d> ARG<0x%.8X> is R1B, TMO not reset hw\n",
				__func__, host->id, cmd->opcode, cmd->arg);
		} else {
			msdc_reset_hw(host->id);
		}
	}
#ifdef MTK_MSDC_USE_CMD23
	if ((sbc != NULL) && (host->autocmd & MSDC_AUTOCMD23)) {
		if (intsts & MSDC_INT_ACMDRDY) {
			u32 *arsp = &sbc->resp[0];
			*arsp = MSDC_READ32(SDC_ACMD_RESP);
		} else if (intsts & MSDC_INT_ACMDCRCERR) {
			pr_err("[%s]: msdc%d, autocmd23 crc error\n",
				__func__, host->id);
			sbc->error = (unsigned int)-EIO;
			/* record the error info in current cmd struct */
			cmd->error = (unsigned int)-EIO;
			/* host->error |= REQ_CMD23_EIO; */
			msdc_reset_hw(host->id);
		} else if (intsts & MSDC_INT_ACMDTMO) {
			pr_err("[%s]: msdc%d, autocmd23 tmo error\n",
				__func__, host->id);
			sbc->error = (unsigned int)-ETIMEDOUT;
			/* record the error info in current cmd struct */
			cmd->error = (unsigned int)-ETIMEDOUT;
			msdc_dump_info(host->id);
			/* host->error |= REQ_CMD23_TMO; */
			msdc_reset_hw(host->id);
		}
	}
#endif /* end of MTK_MSDC_USE_CMD23 */

 out:
	host->cmd = NULL;

	return cmd->error;
}

unsigned int msdc_do_command(struct msdc_host *host,
	struct mmc_command *cmd,
	int tune,
	unsigned long timeout)
{
	MVG_EMMC_DECLARE_INT32(delay_ns);
	MVG_EMMC_DECLARE_INT32(delay_us);
	MVG_EMMC_DECLARE_INT32(delay_ms);

	if ((cmd->opcode == MMC_GO_IDLE_STATE) &&
	    (host->hw->host_function == MSDC_SD)) {
		mdelay(10);
	}

	MVG_EMMC_ERASE_MATCH(host, (u64)cmd->arg, delay_ms, delay_us,
		delay_ns, cmd->opcode);

	if (msdc_command_start(host, cmd, tune, timeout))
		goto end;

	MVG_EMMC_ERASE_RESET(delay_ms, delay_us, cmd->opcode);

	if (msdc_command_resp_polling(host, cmd, tune, timeout))
		goto end;
 end:

	N_MSG(CMD, "        return<%d> resp<0x%.8x>", cmd->error, cmd->resp[0]);
	return cmd->error;
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static unsigned int msdc_cmdq_command_start(struct msdc_host *host,
	struct mmc_command *cmd,
	int tune,   /* not used */
	unsigned long timeout)
{
	void __iomem *base = host->base;
	u32 opcode = cmd->opcode;
	u32 rawarg;
	u32 resp;
	unsigned long tmo;
	u32 wints_cq_cmd = MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO;

	switch (opcode) {
	case MMC_SET_QUEUE_CONTEXT:
	case MMC_QUEUE_READ_ADDRESS:
	case MMC_SEND_STATUS:
		break;
	default:
		pr_err("[%s]: ERROR, only CMD44/CMD45/CMD13 can issue\n",
			__func__);
		break;
	}

	resp = RESP_R1;
	cmd->error = 0;
	N_MSG(CMD, "CMD<%d>        Arg<0x%.8x>", opcode, cmd->arg);

	tmo = jiffies + timeout;

	for (;;) {
		if (!sdc_is_cmd_busy())
			break;

		if (time_after(jiffies, tmo)) {
			ERR_MSG("[%s]: XXX cmd_busy timeout: before CMD<%d>",
				__func__ , opcode);
			cmd->error = (unsigned int)-ETIMEDOUT;
			msdc_reset_hw(host->id);
			return cmd->error;
		}
	}

	host->cmd	  = cmd;
	host->cmd_rsp = resp;

	/* use polling way */
	MSDC_CLR_BIT32(MSDC_INTEN, wints_cq_cmd);
	rawarg = cmd->arg;

	dbg_add_host_log(host->mmc, 0, cmd->opcode, cmd->arg);
	sdc_send_cmdq_cmd(opcode, rawarg);

	return 0;
}

static unsigned int msdc_cmdq_command_resp_polling(struct msdc_host *host,
	struct mmc_command *cmd,
	int tune,
	unsigned long timeout)
{
	void __iomem *base = host->base;
	u32 intsts;
	u32 resp;
	unsigned long tmo;
	u32 cmdsts = MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO;

	resp = host->cmd_rsp;

	/*polling*/
	tmo = jiffies + timeout;
	while (1) {
		intsts = MSDC_READ32(MSDC_INT);
		if ((intsts & cmdsts) != 0) {
			/* clear all int flag */
			intsts &= cmdsts;
			MSDC_WRITE32(MSDC_INT, intsts);
			break;
		}

		if (time_after(jiffies, tmo)) {
			pr_err("[%s]: msdc%d CMD<%d> polling_for_completion timeout ARG<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
			cmd->error = (unsigned int)-ETIMEDOUT;
			host->sw_timeout++;
			msdc_dump_info(host->id);
			/*msdc_reset_hw(host->id);*/
			goto out;
		}
	}

	/* command interrupts */
	if (intsts & cmdsts) {
		if (intsts & MSDC_INT_CMDRDY) {
			u32 *rsp = NULL;
			rsp = &cmd->resp[0];
			switch (host->cmd_rsp) {
			case RESP_NONE:
				break;
			case RESP_R2:
				*rsp++ = MSDC_READ32(SDC_RESP3);
				*rsp++ = MSDC_READ32(SDC_RESP2);
				*rsp++ = MSDC_READ32(SDC_RESP1);
				*rsp++ = MSDC_READ32(SDC_RESP0);
				break;
			default: /* Response types 1, 3, 4, 5, 6, 7(1b) */
				*rsp = MSDC_READ32(SDC_RESP0);
				break;
			}
			dbg_add_host_log(host->mmc, 1, cmd->opcode, cmd->resp[0]);
		} else if (intsts & MSDC_INT_RSPCRCERR) {
			cmd->error = (unsigned int)-EIO;
			pr_err("[%s]: msdc%d XXX CMD<%d> MSDC_INT_RSPCRCERR Arg<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
			msdc_dump_info(host->id);
			/*msdc_reset_hw(host->id);*/
		} else if (intsts & MSDC_INT_CMDTMO) {
			cmd->error = (unsigned int)-ETIMEDOUT;
			pr_err("[%s]: msdc%d XXX CMD<%d> MSDC_INT_CMDTMO Arg<0x%.8x>",
				__func__, host->id, cmd->opcode, cmd->arg);
			mmc_cmd_dump(host->mmc);
			msdc_dump_info(host->id);
			/*msdc_reset_hw(host->id);*/
		}
	}
out:
	host->cmd = NULL;
	MSDC_SET_FIELD(EMMC51_CFG0, MSDC_EMMC51_CFG_CMDQEN, (0));
	return cmd->error;
}

/* do command queue command
	- CMD44, CMD45, CMD13 - QSR
	Use another register set */
unsigned int msdc_do_cmdq_command(struct msdc_host *host,
	struct mmc_command *cmd,
	int tune,
	unsigned long timeout)
{
	if (msdc_cmdq_command_start(host, cmd, tune, timeout))
		goto end;

	if (msdc_cmdq_command_resp_polling(host, cmd, tune, timeout))
		goto end;
end:

	N_MSG(CMD, "		return<%d> resp<0x%.8x>", cmd->error, cmd->resp[0]);
	return cmd->error;
}
#endif

/* The abort condition when PIO read/write
   tmo:
*/
static int msdc_pio_abort(struct msdc_host *host, struct mmc_data *data,
	unsigned long tmo)
{
	int  ret = 0;
	void __iomem *base = host->base;

	if (atomic_read(&host->abort))
		ret = 1;

	if (time_after(jiffies, tmo)) {
		data->error = (unsigned int)-ETIMEDOUT;
		ERR_MSG("XXX PIO Data Timeout: CMD<%d>",
			host->mrq->cmd->opcode);
		msdc_dump_info(host->id);
		ret = 1;
	}

	if (ret) {
		msdc_reset_hw(host->id);
		ERR_MSG("msdc pio find abort");
	}
	return ret;
}

/*
   Need to add a timeout, or WDT timeout, system reboot.
*/
/* pio mode data read/write */
#define COMBINE_HM
int msdc_pio_read(struct msdc_host *host, struct mmc_data *data)
{
	struct scatterlist *sg = data->sg;
	void __iomem *base = host->base;
	u32 num = data->sg_len;
	u32 *ptr;
	u8 *u8ptr;
	u32 left = 0;
	u32 count, size = 0;
	u32 wints = MSDC_INTEN_DATTMO | MSDC_INTEN_DATCRCERR
		| MSDC_INTEN_XFER_COMPL;
	u32 ints = 0;
	bool get_xfer_done = 0;
	unsigned long tmo = jiffies + DAT_TIMEOUT;
	struct page *hmpage = NULL;
	int i = 0, subpage = 0, totalpages = 0;
	int flag = 0;
	ulong kaddr[DIV_ROUND_UP(MAX_SGMT_SZ, PAGE_SIZE)];

	BUG_ON(sg == NULL);
	/*MSDC_CLR_BIT32(MSDC_INTEN, wints);*/
	while (1) {
		if (!get_xfer_done) {
			ints = MSDC_READ32(MSDC_INT);
			latest_int_status[host->id] = ints;
			ints &= wints;
			MSDC_WRITE32(MSDC_INT, ints);
		}
		if (ints & MSDC_INT_DATTMO) {
			data->error = (unsigned int)-ETIMEDOUT;
			msdc_dump_info(host->id);
			msdc_reset_hw(host->id);
			break;
		} else if (ints & MSDC_INT_DATCRCERR) {
			data->error = (unsigned int)-EIO;
			/* msdc_dump_info(host->id); */
			msdc_reset_hw(host->id);
			/* left = msdc_sg_len(sg, host->dma_xfer); */
			/* ptr = sg_virt(sg); */
			break;
		} else if (ints & MSDC_INT_XFER_COMPL) {
			get_xfer_done = 1;
		}
		if (get_xfer_done && (num == 0) && (left == 0))
			break;
		if (msdc_pio_abort(host, data, tmo))
			goto end;
		if ((num == 0) && (left == 0))
			continue;
		left = msdc_sg_len(sg, host->dma_xfer);
		ptr = sg_virt(sg);
		flag = 0;

		if  ((ptr != NULL) &&
		     !(PageHighMem((struct page *)(sg->page_link & ~0x3))))
			#ifndef COMBINE_HM
			goto check_fifo2;
			#else
			goto check_fifo1;
			#endif

		hmpage = (struct page *)(sg->page_link & ~0x3);
		totalpages = DIV_ROUND_UP((left + sg->offset), PAGE_SIZE);
		subpage = (left + sg->offset) % PAGE_SIZE;

		if (subpage != 0 || (sg->offset != 0))
			N_MSG(OPS, "msdc%d: read size or start not align %x,"
				"%x, hmpage %lx,sg offset %x\n", host->id,
				subpage, left, (ulong)hmpage, sg->offset);

		for (i = 0; i < totalpages; i++) {
			kaddr[i] = (ulong) kmap(hmpage + i);
			if ((i > 0) && ((kaddr[i] - kaddr[i - 1]) != PAGE_SIZE))
				flag = 1;
			if (!kaddr[i])
				ERR_MSG("msdc0:kmap failed %lx", kaddr[i]);
		}

		ptr = sg_virt(sg);

		if (ptr == NULL)
			ERR_MSG("msdc0:sg_virt %p", ptr);

		if (flag == 0)
			#ifndef COMBINE_HM
			goto check_fifo2;
			#else
			goto check_fifo1;
			#endif

		/* High memory and more than 1 va address va
		   and not continous */
		/* pr_err("msdc0: kmap not continous %x %x %x\n",
			left,kaddr[i],kaddr[i-1]); */
		for (i = 0; i < totalpages; i++) {
			left = PAGE_SIZE;
			ptr = (u32 *) kaddr[i];

			if (i == 0) {
				left = PAGE_SIZE - sg->offset;
				ptr = (u32 *) (kaddr[i] + sg->offset);
			}
			if ((subpage != 0) && (i == (totalpages-1)))
				left = subpage;

#ifndef COMBINE_HM
check_fifo1:
			if (left == 0)
				continue;
#else
check_fifo1:
			if ((flag == 1) && (left == 0))
				continue;
			else if ((flag == 0) && (left == 0))
				goto check_fifo_end;
#endif

			if ((msdc_rxfifocnt() >= MSDC_FIFO_THD) &&
			    (left >= MSDC_FIFO_THD)) {
				count = MSDC_FIFO_THD >> 2;
				do {
#ifdef MTK_MSDC_DUMP_FIFO
					pr_debug("0x%x ", msdc_fifo_read32());
#else
					*ptr++ = msdc_fifo_read32();
#endif
				} while (--count);
				left -= MSDC_FIFO_THD;
			} else if ((left < MSDC_FIFO_THD) &&
				    msdc_rxfifocnt() >= left) {
				while (left > 3) {
#ifdef MTK_MSDC_DUMP_FIFO
					pr_debug("0x%x ", msdc_fifo_read32());
#else
					*ptr++ = msdc_fifo_read32();
#endif
					left -= 4;
				}

				u8ptr = (u8 *) ptr;
				while (left) {
#ifdef MTK_MSDC_DUMP_FIFO
					pr_debug("0x%x ", msdc_fifo_read8());
#else
					*u8ptr++ = msdc_fifo_read8();
#endif
					left--;
				}
			} else {
				ints = MSDC_READ32(MSDC_INT);
				latest_int_status[host->id] = ints;

				if (ints & MSDC_INT_DATCRCERR) {
					ERR_MSG("[msdc%d] DAT CRC error (0x%x),"
						" Left DAT: %d bytes\n",
						host->id, ints, left);
					data->error = (unsigned int)-EIO;
				} else if (ints & MSDC_INT_DATTMO) {
					ERR_MSG("[msdc%d] DAT TMO error (0x%x),"
						" Left DAT: %d bytes\n",
						host->id, ints, left);
					data->error = (unsigned int)-ETIMEDOUT;
				} else {
					goto skip_msdc_dump_and_reset1;
				}

				if (ints & MSDC_INT_DATTMO)
					msdc_dump_info(host->id);

				MSDC_WRITE32(MSDC_INT, ints);
				msdc_reset_hw(host->id);
				goto end;
			}

skip_msdc_dump_and_reset1:
			if (msdc_pio_abort(host, data, tmo))
				goto end;

			goto check_fifo1;
		}

check_fifo_end:
		if (hmpage != NULL) {
			/* pr_err("read msdc0:unmap %x\n", hmpage); */
			for (i = 0; i < totalpages; i++)
				kunmap(hmpage + i);

			hmpage = NULL;
		}
		size += msdc_sg_len(sg, host->dma_xfer);
		sg = sg_next(sg);
		num--;
	}
 end:
	if (hmpage != NULL) {
		for (i = 0; i < totalpages; i++)
			kunmap(hmpage + i);
		/* pr_err("msdc0 read unmap:\n"); */
	}
	data->bytes_xfered += size;
	N_MSG(FIO, "        PIO Read<%d>bytes", size);

	/* MSDC_CLR_BIT32(MSDC_INTEN, wints); */
	if (data->error)
		ERR_MSG("read pio data->error<%d> left<%d> size<%d>",
			data->error, left, size);
	return data->error;
}

/* please make sure won't using PIO when size >= 512
   which means, memory card block read/write won't using pio
   then don't need to handle the CMD12 when data error.
*/
int msdc_pio_write(struct msdc_host *host, struct mmc_data *data)
{
	void __iomem *base = host->base;
	struct scatterlist *sg = data->sg;
	u32 num = data->sg_len;
	u32 *ptr;
	u8 *u8ptr;
	u32 left = 0;
	u32 count, size = 0;
	u32 wints = MSDC_INTEN_DATTMO | MSDC_INTEN_DATCRCERR
		| MSDC_INTEN_XFER_COMPL;
	bool get_xfer_done = 0;
	unsigned long tmo = jiffies + DAT_TIMEOUT;
	u32 ints = 0;
	struct page *hmpage = NULL;
	int i = 0, totalpages = 0;
	int flag, subpage = 0;
	ulong kaddr[DIV_ROUND_UP(MAX_SGMT_SZ, PAGE_SIZE)];

	/* MSDC_CLR_BIT32(MSDC_INTEN, wints); */
	while (1) {
		if (!get_xfer_done) {
			ints = MSDC_READ32(MSDC_INT);
			latest_int_status[host->id] = ints;
			ints &= wints;
			MSDC_WRITE32(MSDC_INT, ints);
		}
		if (ints & MSDC_INT_DATTMO) {
			data->error = (unsigned int)-ETIMEDOUT;
			msdc_dump_info(host->id);
			msdc_reset_hw(host->id);
			break;
		} else if (ints & MSDC_INT_DATCRCERR) {
			data->error = (unsigned int)-EIO;
			/* msdc_dump_info(host->id); */
			msdc_reset_hw(host->id);
			break;
		} else if (ints & MSDC_INT_XFER_COMPL) {
			get_xfer_done = 1;
			if ((num == 0) && (left == 0))
				break;
		}
		if (msdc_pio_abort(host, data, tmo))
			goto end;
		if ((num == 0) && (left == 0))
			continue;
		left = msdc_sg_len(sg, host->dma_xfer);
		ptr = sg_virt(sg);

		flag = 0;

		/* High memory must kmap, if already mapped,
		   only add counter */
		if  ((ptr != NULL) &&
		     !(PageHighMem((struct page *)(sg->page_link & ~0x3))))
			#ifndef COMBINE_HM
			goto check_fifo2;
			#else
			goto check_fifo1;
			#endif

		hmpage = (struct page *)(sg->page_link & ~0x3);
		totalpages = DIV_ROUND_UP(left + sg->offset, PAGE_SIZE);
		subpage = (left + sg->offset) % PAGE_SIZE;

		if ((subpage != 0) || (sg->offset != 0))
			N_MSG(OPS, "msdc%d: write size or start not align %x,"
				"%x, hmpage %lx,sg offset %x\n", host->id,
				subpage, left, (ulong)hmpage, sg->offset);

		/* Kmap all need pages, */
		for (i = 0; i < totalpages; i++) {
			kaddr[i] = (ulong) kmap(hmpage + i);
			if ((i > 0) && ((kaddr[i] - kaddr[i - 1]) != PAGE_SIZE))
				flag = 1;
			if (!kaddr[i])
				ERR_MSG("msdc0:kmap failed %lx\n", kaddr[i]);
		}

		ptr = sg_virt(sg);

		if (ptr == NULL)
			ERR_MSG("msdc0:write sg_virt %p\n", ptr);

		if (flag == 0)
			#ifndef COMBINE_HM
			goto check_fifo2;
			#else
			goto check_fifo1;
			#endif

		/* High memory and more than 1 va address va
		   may be not continous */
		/*pr_err(ERR "msdc0:w kmap not continous %x %x %x\n",
			left, kaddr[i], kaddr[i-1]);*/
		for (i = 0; i < totalpages; i++) {
			left = PAGE_SIZE;
			ptr = (u32 *) kaddr[i];

			if (i == 0) {
				left = PAGE_SIZE - sg->offset;
				ptr = (u32 *) (kaddr[i] + sg->offset);
			}
			if (subpage != 0 && (i == (totalpages - 1)))
				left = subpage;

#ifndef COMBINE_HM
check_fifo1:
			if (left == 0)
				continue;
#else
check_fifo1:
			if ((flag == 1) && (left == 0))
				continue;
			else if ((flag == 0) && (left == 0))
				goto check_fifo_end;
#endif

			if (left >= MSDC_FIFO_SZ && msdc_txfifocnt() == 0) {
				count = MSDC_FIFO_SZ >> 2;
				do {
					msdc_fifo_write32(*ptr);
					ptr++;
				} while (--count);
				left -= MSDC_FIFO_SZ;
			} else if (left < MSDC_FIFO_SZ &&
				   msdc_txfifocnt() == 0) {
				while (left > 3) {
					msdc_fifo_write32(*ptr);
					ptr++;
					left -= 4;
				}
				u8ptr = (u8 *) ptr;
				while (left) {
					msdc_fifo_write8(*u8ptr);
					u8ptr++;
					left--;
				}
			} else {
				ints = MSDC_READ32(MSDC_INT);
				latest_int_status[host->id] = ints;

				if (ints & MSDC_INT_DATCRCERR) {
					ERR_MSG("[msdc%d] DAT CRC error (0x%x),"
						" Left DAT: %d bytes\n",
						host->id, ints, left);
					data->error = (unsigned int)-EIO;
				} else if (ints & MSDC_INT_DATTMO) {
					ERR_MSG("[msdc%d] DAT TMO error (0x%x),"
						" Left DAT: %d bytes\n",
						host->id, ints, left);
					data->error = (unsigned int)-ETIMEDOUT;
				} else {
					goto skip_msdc_dump_and_reset1;
				}

				msdc_dump_info(host->id);

				MSDC_WRITE32(MSDC_INT, ints);
				msdc_reset_hw(host->id);
				goto end;
			}

skip_msdc_dump_and_reset1:
			if (msdc_pio_abort(host, data, tmo))
				goto end;

			goto check_fifo1;
		}

check_fifo_end:
		if (hmpage != NULL) {
			for (i = 0; i < totalpages; i++)
				kunmap(hmpage + i);

			hmpage = NULL;

		}
		size += msdc_sg_len(sg, host->dma_xfer);
		sg = sg_next(sg);
		num--;
	}
 end:
	if (hmpage != NULL) {
		for (i = 0; i < totalpages; i++)
			kunmap(hmpage + i);
		pr_err("msdc0 write unmap 0x%x:\n", left);
	}
	data->bytes_xfered += size;
	N_MSG(FIO, "        PIO Write<%d>bytes", size);

	if (data->error)
		ERR_MSG("write pio data->error<%d> left<%d> size<%d>",
			data->error, left, size);

	/*MSDC_CLR_BIT32(MSDC_INTEN, wints);*/
	return data->error;
}

static void msdc_dma_start(struct msdc_host *host)
{
	void __iomem *base = host->base;
	u32 wints = MSDC_INTEN_XFER_COMPL | MSDC_INTEN_DATTMO
		| MSDC_INTEN_DATCRCERR;

	if (host->autocmd & MSDC_AUTOCMD12)
		wints |= MSDC_INT_ACMDCRCERR | MSDC_INT_ACMDTMO
			| MSDC_INT_ACMDRDY;
	MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_START, 1);

	MSDC_SET_BIT32(MSDC_INTEN, wints);

	N_MSG(DMA, "DMA start");
	if (host->data && (host->data->flags & MMC_DATA_WRITE) &&
	    host->hw->host_function == MSDC_SD) {
		host->write_timeout_ms = min_t(u32, max_t(u32,
			host->data->blocks * 500,
			host->data->timeout_ns / 1000000), 270 * 1000);
		schedule_delayed_work(&host->write_timeout,
			msecs_to_jiffies(host->write_timeout_ms));
		N_MSG(DMA, "DMA Data Busy Timeout:%u ms, schedule_delayed_work",
			host->write_timeout_ms);
	}
}

static void msdc_dma_stop(struct msdc_host *host)
{
	void __iomem *base = host->base;
	int retry = 500;
	int count = 1000;
	u32 wints = MSDC_INTEN_XFER_COMPL | MSDC_INTEN_DATTMO
		| MSDC_INTEN_DATCRCERR;

	/* Clear DMA data busy timeout */
	if (host->data && (host->data->flags & MMC_DATA_WRITE) &&
	    host->hw->host_function == MSDC_SD) {
		cancel_delayed_work(&host->write_timeout);
		N_MSG(DMA, "DMA Data Busy Timeout:%u ms, cancel_delayed_work",
			host->write_timeout_ms);
		host->write_timeout_ms = 0; /* clear timeout */
	}

	/* handle autocmd12 error in msdc_irq */
	if (host->autocmd & MSDC_AUTOCMD12)
		wints |= MSDC_INT_ACMDCRCERR | MSDC_INT_ACMDTMO
			| MSDC_INT_ACMDRDY;
	N_MSG(DMA, "DMA status: 0x%.8x", MSDC_READ32(MSDC_DMA_CFG));
	/*while (MSDC_READ32(MSDC_DMA_CFG) & MSDC_DMA_CFG_STS);*/

	MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_STOP, 1);
	msdc_retry((MSDC_READ32(MSDC_DMA_CFG) & MSDC_DMA_CFG_STS), retry,
		count, host->id);
	if (retry == 0)
		msdc_polling_axi_status(__LINE__, 1);

	MSDC_CLR_BIT32(MSDC_INTEN, wints); /* Not just xfer_comp */

	N_MSG(DMA, "DMA stop");
}

/* calc checksum */
static u8 msdc_dma_calcs(u8 *buf, u32 len)
{
	u32 i, sum = 0;
	for (i = 0; i < len; i++)
		sum += buf[i];
	return 0xFF - (u8) sum;
}

/* gpd bd setup + dma registers */
static int msdc_dma_config(struct msdc_host *host, struct msdc_dma *dma)
{
	void __iomem *base = host->base;
	u32 sglen = dma->sglen;
	u32 j, num, bdlen;
	dma_addr_t dma_address;
	u32 dma_len;
	u8  blkpad, dwpad, chksum;
	struct scatterlist *sg = dma->sg;
	gpd_t *gpd;
	bd_t *bd, vbd = {0};

	switch (dma->mode) {
	case MSDC_MODE_DMA_BASIC:
#if defined(FEATURE_MET_MMC_INDEX)
		met_mmc_bdnum = 1;
#endif

		if (host->hw->host_function == MSDC_SDIO)
			BUG_ON(dma->xfersz > 0xFFFFFFFF);
		else
			BUG_ON(dma->xfersz > 65535);

		BUG_ON(dma->sglen != 1);
		dma_address = sg_dma_address(sg);
		dma_len = msdc_sg_len(sg, host->dma_xfer);

		N_MSG(DMA, "DMA BASIC mode dma_len<%x> dma_address<%llx>",
			dma_len, (u64)dma_address);

		MSDC_WRITE32(MSDC_DMA_SA, dma_address);

		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_LASTBUF, 1);
		MSDC_WRITE32(MSDC_DMA_LEN, dma_len);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_BRUSTSZ,
			dma->burstsz);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_MODE, 0);
		break;

	case MSDC_MODE_DMA_DESC:
		blkpad = (dma->flags & DMA_FLAG_PAD_BLOCK) ? 1 : 0;
		dwpad  = (dma->flags & DMA_FLAG_PAD_DWORD) ? 1 : 0;
		chksum = (dma->flags & DMA_FLAG_EN_CHKSUM) ? 1 : 0;

		/* calculate the required number of gpd */
		num = (sglen + MAX_BD_PER_GPD - 1) / MAX_BD_PER_GPD;
		BUG_ON(num != 1);

		gpd = dma->gpd;
		bd  = dma->bd;
		bdlen = sglen;

#if defined(FEATURE_MET_MMC_INDEX)
		met_mmc_bdnum = bdlen;
#endif

		/* modify gpd */
		gpd->hwo = 1;   /* hw will clear it */
		gpd->bdp = 1;
		gpd->chksum = 0;        /* need to clear first. */
		gpd->chksum = (chksum ? msdc_dma_calcs((u8 *) gpd, 16) : 0);

		/* modify bd */
		for (j = 0; j < bdlen; j++) {
#ifdef MSDC_DMA_VIOLATION_DEBUG
			if (g_dma_debug[host->id] &&
			    (msdc_latest_op[host->id] == OPER_TYPE_READ)) {
				pr_debug("[%s] msdc%d do write 0x10000\n",
					__func__, host->id);
				dma_address = 0x10000;
			} else {
				dma_address = sg_dma_address(sg);
			}
#else
			dma_address = sg_dma_address(sg);
#endif

			dma_len = msdc_sg_len(sg, host->dma_xfer);

			N_MSG(DMA, "DESC DMA len<%x> dma_address<%llx>",
				dma_len, (u64)dma_address);

			memcpy(&vbd, &bd[j], sizeof(bd_t));

			msdc_init_bd(&vbd, blkpad, dwpad, dma_address,
				dma_len);

			if (j == bdlen - 1)
				vbd.eol = 1;  /* the last bd */
			else
				vbd.eol = 0;

			/* checksume need to clear first */
			vbd.chksum = 0;
			vbd.chksum = (chksum ?
				msdc_dma_calcs((u8 *) (&vbd), 16) : 0);

			memcpy(&bd[j], &vbd, sizeof(bd_t));

			sg++;
		}
#ifdef MSDC_DMA_VIOLATION_DEBUG
		if (g_dma_debug[host->id] &&
		    (msdc_latest_op[host->id] == OPER_TYPE_READ))
			g_dma_debug[host->id] = 0;
#endif

		dma->used_gpd += 2;
		dma->used_bd += bdlen;

		MSDC_SET_FIELD(MSDC_DMA_CFG, MSDC_DMA_CFG_DECSEN, chksum);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_BRUSTSZ,
			dma->burstsz);
		MSDC_SET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_MODE, 1);

		MSDC_WRITE32(MSDC_DMA_SA, (u32) dma->gpd_addr);
		break;

	default:
		break;
	}

	N_MSG(DMA, "DMA_CTRL = 0x%x", MSDC_READ32(MSDC_DMA_CTRL));
	N_MSG(DMA, "DMA_CFG  = 0x%x", MSDC_READ32(MSDC_DMA_CFG));
	N_MSG(DMA, "DMA_SA   = 0x%x", MSDC_READ32(MSDC_DMA_SA));

	return 0;
}

static void msdc_dma_setup(struct msdc_host *host, struct msdc_dma *dma,
	struct scatterlist *sg, unsigned int sglen)
{
	u32 max_dma_len = 0;
	BUG_ON(sglen > MAX_BD_NUM);     /* not support currently */

	dma->sg = sg;
	dma->flags = DMA_FLAG_EN_CHKSUM;
	/* dma->flags = DMA_FLAG_NONE; */ /* CHECKME */
	dma->sglen = sglen;
	dma->xfersz = host->xfer_size;
	dma->burstsz = MSDC_BRUST_64B;

	if (host->hw->host_function == MSDC_SDIO)
		max_dma_len = MAX_DMA_CNT_SDIO;
	else
		max_dma_len = MAX_DMA_CNT;

	if (sglen == 1 &&
	     msdc_sg_len(sg, host->dma_xfer) <= max_dma_len)
		dma->mode = MSDC_MODE_DMA_BASIC;
	else
		dma->mode = MSDC_MODE_DMA_DESC;

	N_MSG(DMA, "DMA mode<%d> sglen<%d> xfersz<%d>", dma->mode, dma->sglen,
		dma->xfersz);

	msdc_dma_config(host, dma);
}

static void msdc_dma_clear(struct msdc_host *host)
{
	void __iomem *base = host->base;
	host->data = NULL;
	host->mrq = NULL;
	host->dma_xfer = 0;
	msdc_dma_off();
	host->dma.used_bd = 0;
	host->dma.used_gpd = 0;
	host->blksz = 0;
}

/* set block number before send command */
static void msdc_set_blknum(struct msdc_host *host, u32 blknum)
{
	void __iomem *base = host->base;

	MSDC_WRITE32(SDC_BLK_NUM, blknum);
}

#define REQ_CMD_EIO  (0x1 << 0)
#define REQ_CMD_TMO  (0x1 << 1)
#define REQ_DAT_ERR  (0x1 << 2)
#define REQ_STOP_EIO (0x1 << 3)
#define REQ_STOP_TMO (0x1 << 4)
#define REQ_CMD23_EIO (0x1 << 5)
#define REQ_CMD23_TMO (0x1 << 6)

static void msdc_log_cmd(struct msdc_host *host, struct mmc_command *cmd,
	struct mmc_data *data)
{
	N_MSG(OPS, "CMD<%d> data<%s %s> blksz<%d> block<%d> error<%d>",
		cmd->opcode, (host->dma_xfer ? "dma" : "pio"),
		((data->flags & MMC_DATA_READ) ? "read " : "write"),
		data->blksz, data->blocks, data->error);

	if (!(is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ))) {
		if (!check_mmc_cmd2425(cmd->opcode) &&
		    !check_mmc_cmd1718(cmd->opcode)) {
			N_MSG(NRW, "CMD<%3d> arg<0x%8x> Resp<0x%8x> "
				"data<%s> size<%d>",
				cmd->opcode, cmd->arg, cmd->resp[0],
				((data->flags & MMC_DATA_READ)
					? "read " : "write"),
				data->blksz * data->blocks);
		} else if (cmd->opcode != 13) { /* by pass CMD13 */
			N_MSG(NRW, "CMD<%3d> arg<0x%8x> resp<%8x %8x "
				"%8x %8x>", cmd->opcode,
				cmd->arg, cmd->resp[0], cmd->resp[1],
				cmd->resp[2], cmd->resp[3]);
		} else {
			N_MSG(RW, "CMD<%3d> arg<0x%8x> Resp<0x%8x> "
				"block<%d>", cmd->opcode,
				cmd->arg, cmd->resp[0], data->blocks);
		}
	}
}

void msdc_sdio_restore_after_resume(struct msdc_host *host)
{
	void __iomem *base = host->base;

	if (host->saved_para.hz) {
		if ((host->saved_para.suspend_flag)
		 || ((host->saved_para.msdc_cfg != 0) &&
		     ((host->saved_para.msdc_cfg&0xFFFFFF9F) !=
		      (MSDC_READ32(MSDC_CFG)&0xFFFFFF9F)))) {
			ERR_MSG("msdc resume[ns] cur_cfg=%x, save_cfg=%x"
				", cur_hz=%d, save_hz=%d",
				MSDC_READ32(MSDC_CFG),
				host->saved_para.msdc_cfg, host->mclk,
				host->saved_para.hz);
			host->saved_para.suspend_flag = 0;
			msdc_restore_timing_setting(host);
		}
	}
}

int msdc_if_send_stop(struct msdc_host *host,
	struct mmc_command *cmd, struct mmc_data *data)
{
	if (!data || !data->stop)
		return 0;

	if ((cmd->error != 0)
	 || (data->error != 0)
	 || !(host->autocmd & MSDC_AUTOCMD12)
	 || !(check_mmc_cmd1825(cmd->opcode))) {
		if (msdc_do_command(host, data->stop, 0, CMD_TIMEOUT) != 0)
			return 1;
	}

	return 0;
}

void msdc_if_set_err(struct msdc_host *host, struct mmc_request *mrq,
	struct mmc_command *cmd)
{
	if (mrq->cmd->error == (unsigned int)-EIO) {
		if (((cmd->opcode == MMC_SELECT_CARD) ||
		     (cmd->opcode == MMC_SLEEP_AWAKE))
		 && ((host->hw->host_function == MSDC_EMMC) ||
		     (host->hw->host_function == MSDC_SD))) {
			/* should be deleted in new platform,
			   as the state verify function has applied*/
			mrq->cmd->error = 0x0;
		} else {
			host->error |= REQ_CMD_EIO;
		}
	}
	if (mrq->cmd->error == (unsigned int)-ETIMEDOUT)
		host->error |= REQ_CMD_TMO;
	if (mrq->data && (mrq->data->error))
		host->error |= REQ_DAT_ERR;
	if (mrq->stop && (mrq->stop->error == (unsigned int)-EIO))
		host->error |= REQ_STOP_EIO;
	if (mrq->stop && (mrq->stop->error == (unsigned int)-ETIMEDOUT))
		host->error |= REQ_STOP_TMO;
}

int msdc_rw_cmd_dma(struct mmc_host *mmc, struct mmc_command *cmd,
	struct mmc_data *data, struct mmc_request *mrq, int tune)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	int map_sg = 0;
	int dir;

	msdc_dma_on();  /* enable DMA mode first!! */

	init_completion(&host->xfer_done);

	if (msdc_command_start(host, cmd, 0, CMD_TIMEOUT) != 0)
		return -1;

	if (tune == 0) {
		dir = data->flags & MMC_DATA_READ ?
			DMA_FROM_DEVICE : DMA_TO_DEVICE;
		(void)dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len, dir);
		map_sg = 1;
	}

	/* then wait command done */
	if (msdc_command_resp_polling(host, cmd, 0, CMD_TIMEOUT) != 0)
		return -2;

	/* for read, the data coming too fast, then CRC error
	   start DMA no business with CRC. */
	msdc_dma_setup(host, &host->dma, data->sg, data->sg_len);
	msdc_dma_start(host);

	spin_unlock(&host->lock);
	if (!wait_for_completion_timeout(&host->xfer_done, DAT_TIMEOUT)) {
		ERR_MSG("XXX CMD<%d> ARG<0x%x> wait xfer_done<%d> timeout!!",
			cmd->opcode, cmd->arg, data->blocks * data->blksz);

		host->sw_timeout++;

		msdc_dump_info(host->id);
		data->error = (unsigned int)-ETIMEDOUT;
		msdc_reset(host->id);
	}
	spin_lock(&host->lock);

	msdc_dma_stop(host);

	if (((host->autocmd & MSDC_AUTOCMD12) && mrq->stop && mrq->stop->error)
	 || (mrq->data && mrq->data->error)
	 || (mrq->sbc && (mrq->sbc->error != 0) &&
	    (host->autocmd & MSDC_AUTOCMD23))) {
		msdc_clr_fifo(host->id);
		msdc_clr_int();
	}

	if (tune)
		return 0;
	else
		return map_sg;
}

#define PREPARE_NON_ASYNC       0
#define PREPARE_ASYNC           1
#define PREPARE_TUNE            2
int msdc_do_request_prepare(struct msdc_host *host,
	struct mmc_request *mrq,
	struct mmc_command *cmd,
	struct mmc_data *data,
	u32 *l_force_prg,
	u32 *l_bypass_flush,
	int prepare_case)
{
	void __iomem *base = host->base;

	if (prepare_case != PREPARE_TUNE)
		host->error = 0;

	atomic_set(&host->abort, 0);

#ifndef CONFIG_MTK_EMMC_CQ_SUPPORT
	/* check msdc work ok: RX/TX fifocnt must be zero after last request
	 * if find abnormal, try to reset msdc first
	 */
	if (msdc_txfifocnt() || msdc_rxfifocnt()) {
		pr_err("[SD%d] register abnormal,please check!\n", host->id);
		msdc_reset_hw(host->id);
	}
#endif

	if ((prepare_case == PREPARE_NON_ASYNC) && !data) {

#ifdef MTK_MSDC_USE_CACHE
		if ((host->hw->host_function == MSDC_EMMC) &&
		    check_mmc_cache_flush_cmd(cmd)) {
			if (g_cache_status == CACHE_FLUSHED) {
				N_MSG(CHE, "bypass flush command, "
					"g_cache_status=%d",
					g_cache_status);
				*l_bypass_flush = 1;
				return 1;
			} else {
				*l_bypass_flush = 0;
			}
		}
#endif

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (check_mmc_cmd13_sqs(cmd)) {
			if (msdc_do_cmdq_command(host, cmd, 0, CMD_TIMEOUT) != 0)
				return 1;
		} else {
#endif
		if (msdc_do_command(host, cmd, 0, CMD_TIMEOUT) != 0)
			return 1;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		}
#endif

		return 1;
	}

	BUG_ON(data->blksz > HOST_MAX_BLKSZ);

	data->error = 0;
	msdc_latest_op[host->id] = (data->flags & MMC_DATA_READ)
		? OPER_TYPE_READ : OPER_TYPE_WRITE;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	/* if CMDQ CMD13 QSR, host->data may be data of mrq - CMD46,47 */
	if (!check_mmc_cmd13_sqs(cmd))
		host->data = data;
#else
	host->data = data;
#endif

	host->xfer_size = data->blocks * data->blksz;
	host->blksz = data->blksz;
	if (prepare_case != PREPARE_NON_ASYNC) {
		host->dma_xfer = 1;
	} else {
		/* deside the transfer mode */
		if (drv_mode[host->id] == MODE_PIO) {
			host->dma_xfer = 0;
			msdc_latest_transfer_mode[host->id] = TRAN_MOD_PIO;
		} else if (drv_mode[host->id] == MODE_DMA) {
			host->dma_xfer = 1;
			msdc_latest_transfer_mode[host->id] = TRAN_MOD_DMA;
		} else if (drv_mode[host->id] == MODE_SIZE_DEP) {
			host->dma_xfer = (host->xfer_size >= dma_size[host->id])
				? 1 : 0;
			msdc_latest_transfer_mode[host->id] =
				host->dma_xfer ? TRAN_MOD_DMA : TRAN_MOD_PIO;
		}
	}

	if (data->flags & MMC_DATA_READ) {
		if ((host->timeout_ns != data->timeout_ns) ||
		    (host->timeout_clks != data->timeout_clks)) {
			msdc_set_timeout(host, data->timeout_ns,
				data->timeout_clks);
		}
	}

	msdc_set_blknum(host, data->blocks);
	/* msdc_clr_fifo();  */ /* no need */

#ifdef MTK_MSDC_USE_CACHE
	/* Currently, tuning does not use CMD23, so force programming
	   cannot be applied */
	if (prepare_case != PREPARE_TUNE
	 && check_mmc_cache_ctrl(host->mmc->card)
	 && check_mmc_cmd2425(cmd->opcode))
		*l_force_prg = !msdc_can_apply_cache(cmd->arg, data->blocks);
#endif

	return 0;
}

int msdc_do_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	u32 l_autocmd23_is_set = 0;
#ifdef MTK_MSDC_USE_CMD23
	u32 l_card_no_cmd23 = 0;
#endif
#ifdef MTK_MSDC_USE_CACHE
	u32 l_force_prg = 0;
	/* 0: flush need, 1: flush bypass, 2: not switch cmd*/
	u32 l_bypass_flush = 2;
#endif
	void __iomem *base = host->base;
	/* u32 intsts = 0; */
	unsigned int left = 0;
	int read = 1, dir = DMA_FROM_DEVICE;
	u32 map_sg = 0;
	unsigned long pio_tmo;

	if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ))
		msdc_sdio_restore_after_resume(host);

#if (MSDC_DATA1_INT == 1)
	if (host->hw->flags & MSDC_SDIO_IRQ) {
		/*if ((!u_sdio_irq_counter) && (!u_msdc_irq_counter))*/
		/*ERR_MSG("u_sdio_irq_counter=%d, u_msdc_irq_counter=%d  "
			"int_sdio_irq_enable=%d SDC_CFG=%x MSDC_INTEN=%x "
			"MSDC_INT=%x MSDC_PATCH_BIT0=%x",
			u_sdio_irq_counter, u_msdc_irq_counter,
			int_sdio_irq_enable, MSDC_READ32(SDC_CFG),
			MSDC_READ32(MSDC_INTEN), MSDC_READ32(MSDC_INT),
			MSDC_READ32(MSDC_PATCH_BIT0));*/
		if ((u_sdio_irq_counter > 0) && ((u_sdio_irq_counter%800) == 0))
			ERR_MSG("sdio_irq=%d, msdc_irq=%d  SDC_CFG=%x "
				"MSDC_INTEN=%x MSDC_INT=%x ",
				u_sdio_irq_counter, u_msdc_irq_counter,
				MSDC_READ32(SDC_CFG), MSDC_READ32(MSDC_INTEN),
				MSDC_READ32(MSDC_INT));
	}
#endif

	BUG_ON(mmc == NULL);
	BUG_ON(mrq == NULL);

	cmd = mrq->cmd;
	data = mrq->cmd->data;

#ifdef MTK_MSDC_USE_CACHE
	if (msdc_do_request_prepare(host, mrq, cmd, data, &l_force_prg,
		&l_bypass_flush, PREPARE_NON_ASYNC))
#else
	if (msdc_do_request_prepare(host, mrq, cmd, data, NULL,
		NULL, PREPARE_NON_ASYNC))
#endif
		goto done;

#ifdef MTK_MSDC_USE_CMD23
	if (0 == (host->autocmd & MSDC_AUTOCMD23)) {
		/* start the cmd23 first,
		   mrq->sbc is NULL with single r/w */
		if (mrq->sbc) {
			host->autocmd &= ~MSDC_AUTOCMD12;

			if (host->hw->host_function == MSDC_EMMC) {
#ifdef MTK_MSDC_USE_CACHE
				if (!((mrq->sbc->arg >> 31) & 0x1) &&
				    l_force_prg)
					mrq->sbc->arg |= (1 << 24);
#endif
			}

			if (msdc_command_start(host, mrq->sbc, 0,
				CMD_TIMEOUT) != 0)
				goto done;

			/* then wait command done */
			if (msdc_command_resp_polling(host, mrq->sbc, 0,
				CMD_TIMEOUT) != 0) {
				goto stop;
			}
		} else {
			/* some sd card may not support cmd23,
			 * some emmc card have problem with cmd23,
			   so use cmd12 here */
			if (host->hw->host_function != MSDC_SDIO)
				host->autocmd |= MSDC_AUTOCMD12;
		}
	} else {
		/* enable auto cmd23 */
		if (mrq->sbc) {
			host->autocmd &= ~MSDC_AUTOCMD12;
			if (host->hw->host_function == MSDC_EMMC) {
#ifdef MTK_MSDC_USE_CACHE
				if (!((mrq->sbc->arg >> 31) & 0x1) &&
				    l_force_prg)
					mrq->sbc->arg |= (1 << 24);
#endif
			}
		} else {
			/* some sd card may not support cmd23,
			 * some emmc card have problem with cmd23,
			   so use cmd12 here */
			if (host->hw->host_function != MSDC_SDIO) {
				host->autocmd &= ~MSDC_AUTOCMD23;
				host->autocmd |= MSDC_AUTOCMD12;
				l_card_no_cmd23 = 1;
			}
		}
	}
#endif /* end of MTK_MSDC_USE_CMD23 */

	read = data->flags & MMC_DATA_READ ? 1 : 0;
	if (host->dma_xfer) {
		map_sg = msdc_rw_cmd_dma(mmc, cmd, data, mrq, 0);
		if (map_sg == -1)
			goto done;
		else if (map_sg == -2)
			goto stop;

	} else {
		/* Turn off dma */
		if (is_card_sdio(host)) {
			msdc_reset_hw(host->id);
			msdc_dma_off();
			data->error = 0;
		}
		/* Firstly: send command */
		host->autocmd &= ~MSDC_AUTOCMD12;

		l_autocmd23_is_set = 0;
		if (host->autocmd & MSDC_AUTOCMD23) {
			l_autocmd23_is_set = 1;
			host->autocmd &= ~MSDC_AUTOCMD23;
		}

		host->dma_xfer = 0;
		if (msdc_do_command(host, cmd, 0, CMD_TIMEOUT) != 0)
			goto stop;

		/* Secondly: pio data phase */
		if (read) {
#ifdef MTK_MSDC_DUMP_FIFO
			pr_debug("[%s]: start pio read\n", __func__);
#endif
			if (msdc_pio_read(host, data)) {
				msdc_gate_clock(host, 0);
				msdc_ungate_clock(host);
				goto stop;      /* need cmd12 */
			}
		} else {
#ifdef MTK_MSDC_DUMP_FIFO
			pr_debug("[%s]: start pio write\n", __func__);
#endif
			if (msdc_pio_write(host, data)) {
				msdc_gate_clock(host, 0);
				msdc_ungate_clock(host);
				goto stop;
			}

			/* For write case: make sure contents in fifo
			   flushed to device */

			pio_tmo = jiffies + DAT_TIMEOUT;
			while (1) {
				left = msdc_txfifocnt();
				if (left == 0)
					break;

				if (msdc_pio_abort(host, data, pio_tmo))
					break;
			}
		}
	}

stop:
	/* pio mode will disable autocmd23 */
	if (l_autocmd23_is_set == 1) {
		l_autocmd23_is_set = 0;
		host->autocmd |= MSDC_AUTOCMD23;
	}

#ifndef MTK_MSDC_USE_CMD23
	/* Last: stop transfer */
	if (msdc_if_send_stop(host, cmd, data))
		goto done;

#else

	if (host->hw->host_function == MSDC_EMMC) {
		/* multi r/w with no cmd23 and no autocmd12,
		   need send cmd12 manual */
		/* if PIO mode and autocmd23 enable, cmd12 need send,
		   because autocmd23 is disable under PIO */
		if (!check_mmc_cmd2425(cmd->opcode))
			goto done;
		if (((mrq->sbc == NULL) &&
		     !(host->autocmd & MSDC_AUTOCMD12))
		 || (!host->dma_xfer && mrq->sbc &&
		     (host->autocmd & MSDC_AUTOCMD23))) {
			if (msdc_do_command(host, data->stop, 0,
				CMD_TIMEOUT) != 0)
				goto done;
		}
	} else {
		if (msdc_if_send_stop(host, cmd, data))
			goto done;
	}
#endif
done:

#ifdef MTK_MSDC_USE_CMD23
	/* for msdc use cmd23, but card not supported(sbc is NULL),
	   need enable autocmd23 for next request */
	if (1 == l_card_no_cmd23) {
		if (host->hw->host_function != MSDC_SDIO) {
			host->autocmd |= MSDC_AUTOCMD23;
			host->autocmd &= ~MSDC_AUTOCMD12;
			l_card_no_cmd23 = 0;
		}
	}
#endif

	if (data != NULL) {
		host->data = NULL;

		/* end read MBR */
		if (host->dma_xfer != 0) {
			host->dma_xfer = 0;
			msdc_dma_off();
			host->dma.used_bd = 0;
			host->dma.used_gpd = 0;
			if (map_sg == 1) {
				/*if (data->error == 0) {
					int retry = 3;
					int count = 1000;
					msdc_retry(host->dma.gpd->hwo, retry,
						count, host->id);
				} */
				dma_unmap_sg(mmc_dev(mmc), data->sg,
					data->sg_len, dir);
			}
		}

		host->blksz = 0;
		msdc_log_cmd(host, cmd, data);
	}

	if (mrq->cmd->error == (unsigned int)-EIO) {
		if (((cmd->opcode == MMC_SELECT_CARD) ||
		     (cmd->opcode == MMC_SLEEP_AWAKE))
		 && ((host->hw->host_function == MSDC_EMMC) ||
		     (host->hw->host_function == MSDC_SD))) {
			/* should be deleted in new platform,
			   as the state verify function has applied*/
			mrq->cmd->error = 0x0;
		} else {
			host->error |= REQ_CMD_EIO;

			if (mrq->cmd->opcode == SD_IO_RW_EXTENDED)
				sdio_tune_flag |= 0x1;
		}
	}

	if (mrq->cmd->error == (unsigned int)-ETIMEDOUT) {
		if ((mrq->cmd->opcode == MMC_SLEEP_AWAKE) &&
		    emmc_do_sleep_awake) {
			emmc_sleep_failed = 1;
			if (mrq->cmd->arg & (1<<15)) {
				mrq->cmd->error = 0x0;
				pr_err("eMMC sleep CMD5 TMO will reinit\n");
			} else {
				host->error |= REQ_CMD_TMO;
			}
		} else {
			host->error |= REQ_CMD_TMO;
		}
	}

	if (mrq->data && mrq->data->error) {
		host->error |= REQ_DAT_ERR;
		sdio_tune_flag |= 0x10;

		if (mrq->data->flags & MMC_DATA_READ)
			sdio_tune_flag |= 0x80;
		else
			sdio_tune_flag |= 0x40;
	}

#ifdef MTK_MSDC_USE_CMD23
	if (mrq->sbc && (mrq->sbc->error == (unsigned int)-EIO))
		host->error |= REQ_CMD_EIO;
	if (mrq->sbc && (mrq->sbc->error == (unsigned int)-ETIMEDOUT)) {
#ifdef CONFIG_MTK_AEE_FEATURE
		aee_kernel_warning_api(__FILE__, __LINE__,
			DB_OPT_NE_JBT_TRACES|DB_OPT_DISPLAY_HANG_DUMP,
			"\n@eMMC FATAL ERROR@\n", "eMMC fatal error ");
#endif
		host->error |= REQ_CMD_TMO;
	}
#endif

	if (mrq->stop && (mrq->stop->error == (unsigned int)-EIO))
		host->error |= REQ_STOP_EIO;
	if (mrq->stop && (mrq->stop->error == (unsigned int)-ETIMEDOUT))
		host->error |= REQ_STOP_TMO;

#ifdef SDIO_ERROR_BYPASS
	if (is_card_sdio(host) && !host->error)
		host->sdio_error = 0;
#endif

#ifdef MTK_MSDC_USE_CACHE
	msdc_update_cache_flush_status(host, mrq, data, l_bypass_flush);
#endif

	return host->error;
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static int msdc_do_discard_task_cq(struct mmc_host *mmc,
	struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 task_id;

	task_id = (mrq->sbc->arg >> 16) & 0x1f;
	memset(&mmc->deq_cmd, 0, sizeof(struct mmc_command));
	mmc->deq_cmd.opcode = MMC_CMDQ_TASK_MGMT;
	mmc->deq_cmd.arg = 2 | (task_id << 16);
	mmc->deq_cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1B | MMC_CMD_AC;
	mmc->deq_cmd.data = NULL;
	msdc_do_command(host, &mmc->deq_cmd, 0, CMD_TIMEOUT);

	pr_debug("[%s]: msdc%d, discard task id %d, CMD<%d> arg<0x%08x> rsp<0x%08x>",
		__func__, host->id, task_id, mmc->deq_cmd.opcode, mmc->deq_cmd.arg, mmc->deq_cmd.resp[0]);

	return mmc->deq_cmd.error;
}

static int msdc_do_request_cq(struct mmc_host *mmc,
	struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
#ifdef MTK_MSDC_USE_CACHE
	u32 l_force_prg = 0;
#endif

	BUG_ON(mmc == NULL);
	BUG_ON(mrq == NULL);
	BUG_ON(mrq->data);

	host->error = 0;
	atomic_set(&host->abort, 0);

	cmd  = mrq->sbc;
	data = mrq->data;

	mrq->sbc->error = 0;
	mrq->cmd->error = 0;

#ifdef MTK_MSDC_USE_CACHE
	/* check cache enabled, write direction */
	if (check_mmc_cache_ctrl(host->mmc->card) &&
		!((cmd->arg >> 30) & 0x1)) {
		l_force_prg = !msdc_can_apply_cache(mrq->cmd->arg, cmd->arg & 0xffff);
		/* check not reliable write */
		if (!((cmd->arg >> 31) & 0x1) &&
			l_force_prg)
			cmd->arg |= (1 << 24);
	}
#endif

	if (msdc_do_cmdq_command(host, cmd, 0, CMD_TIMEOUT) != 0)
		goto done1;

done1:
	if (cmd->error == (unsigned int)-EIO)
		host->error |= REQ_CMD_EIO;
	else if (cmd->error == (unsigned int)-ETIMEDOUT)
		host->error |= REQ_CMD_TMO;

	cmd  = mrq->cmd;
	data = mrq->cmd->data;

	if (msdc_do_cmdq_command(host, cmd, 0, CMD_TIMEOUT) != 0)
		goto done2;

done2:
	if (cmd->error == (unsigned int)-EIO)
		host->error |= REQ_CMD_EIO;
	else if (cmd->error == (unsigned int)-ETIMEDOUT)
		host->error |= REQ_CMD_TMO;

	return host->error;
}

static int tune_cmdq_cmdrsp(struct mmc_host *mmc,
	struct mmc_request *mrq, int *retry)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	unsigned long polling_tmo = 0;

	u32 err = 0, status = 0;

	do {
		err = msdc_get_card_status(mmc, host, &status);
		if (err) {
			/* wait for transfer done */
			if (!atomic_read(&mmc->cq_tuning_now)) {
				polling_tmo = jiffies + 10 * HZ;
				pr_err("msdc%d waiting data transfer done\n",
						host->id);
				while (mmc->is_data_dma) {
					if (time_after(jiffies, polling_tmo)) {
						ERR_MSG("waiting data transfer done TMO");
						msdc_dump_info(host->id);
						msdc_dma_stop(host);
						msdc_dma_clear(host);
						msdc_reset_hw(host->id);
						return -1;
					}
				}
			}

			ERR_MSG("get card status, err = %d", err);
#ifdef MSDC_AUTOK_ON_ERROR
			if (msdc_execute_tuning(mmc, MMC_SEND_STATUS)) {
				ERR_MSG("failed to updata cmd para");
				return 1;
			}
#else
			if (msdc_tune_cmdrsp(host)) {
				ERR_MSG("failed to updata cmd para");
				return 1;
			}
#endif
			continue;
		}

		if (status & (1 << 22)) {
			/* illegal command */
			(*retry)--;
			ERR_MSG("status = %x, illegal command, retry = %d",
				status, *retry);
			if ((mrq->cmd->error || mrq->sbc->error) && *retry)
				return 0;
			else
				return 1;
		} else {
			ERR_MSG("status = %x, discard task, re-send command",
				status);
			err = msdc_do_discard_task_cq(mmc, mrq);
			if (err == (unsigned int)-EIO)
				continue;
			else
				break;
		}
	} while (err);

	/* wait for transfer done */
	if (!atomic_read(&mmc->cq_tuning_now)) {
		polling_tmo = jiffies + 10 * HZ;
		pr_err("msdc%d waiting data transfer done\n", host->id);
		while (mmc->is_data_dma) {
			if (time_after(jiffies, polling_tmo)) {
				ERR_MSG("waiting data transfer done TMO");
				msdc_dump_info(host->id);
				msdc_dma_stop(host);
				msdc_dma_clear(host);
				msdc_reset_hw(host->id);
				return -1;
			}
		}
	}
	if (msdc_execute_tuning(mmc, MMC_SEND_STATUS)) {
		pr_err("msdc%d autok failed\n", host->id);
		return 1;
	}

	return 0;
}

static int tune_cmdq_data(struct mmc_host *mmc,
	struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret;

	if (mrq->cmd && (mrq->cmd->error == (unsigned int)-EIO)) {
		ret = msdc_tune_cmdrsp(host);
	} else if (mrq->data && (mrq->data->error == (unsigned int)-EIO)) {
		if (host->timing == MMC_TIMING_MMC_HS400) {
			ret = emmc_hs400_tune_rw(host);
		} else if (host->timing == MMC_TIMING_MMC_HS200) {
			if (mrq->data->flags & MMC_DATA_READ)
				ret = msdc_tune_read(host);
			else
				ret = msdc_tune_write(host);
		}
	}

	return ret;
}
#endif

static int msdc_tune_rw_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	int ret;

#ifdef MTK_MSDC_USE_CMD23
	u32 l_autocmd23_is_set = 0;
#endif

	BUG_ON(mmc == NULL);
	BUG_ON(mrq == NULL);

	cmd = mrq->cmd;
	data = mrq->cmd->data;

	msdc_do_request_prepare(host, mrq, cmd, data, NULL,
		NULL, PREPARE_TUNE);

	if (host->hw->host_function != MSDC_SDIO) {
		host->autocmd |= MSDC_AUTOCMD12;

#ifdef MTK_MSDC_USE_CMD23
		/* disable autocmd23 in error tuning flow */
		l_autocmd23_is_set = 0;
		if (host->autocmd & MSDC_AUTOCMD23) {
			l_autocmd23_is_set = 1;
			host->autocmd &= ~MSDC_AUTOCMD23;
		}
#endif
	}

	ret = msdc_rw_cmd_dma(mmc, cmd, data, mrq, 1);
	if (ret == -1)
		goto done;
	else if (ret == -2)
		goto stop;

stop:
	/* Last: stop transfer */
	if (msdc_if_send_stop(host, cmd, data))
		goto done;

done:
	msdc_dma_clear(host);
	host->mrq = mrq; /* check this can be removed */

	msdc_log_cmd(host, cmd, data);

	host->error = 0;

	msdc_if_set_err(host, mrq, cmd);

#ifdef MTK_MSDC_USE_CMD23
	if (l_autocmd23_is_set == 1) {
		/* restore the value */
		host->autocmd |= MSDC_AUTOCMD23;
	}
#endif
	return host->error;
}

static void msdc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq,
	bool is_first_req)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data;
	struct mmc_command *cmd = mrq->cmd;
	int read = 1, dir = DMA_FROM_DEVICE;
	BUG_ON(!cmd);
	data = mrq->data;

	if (!data)
		return;

	data->host_cookie = MSDC_COOKIE_ASYNC;
	if (check_mmc_cmd1718(cmd->opcode) ||
	    check_mmc_cmd2425(cmd->opcode)) {
		host->xfer_size = data->blocks * data->blksz;
		read = data->flags & MMC_DATA_READ ? 1 : 0;
		if (drv_mode[host->id] == MODE_PIO) {
			data->host_cookie |= MSDC_COOKIE_PIO;
			msdc_latest_transfer_mode[host->id] = TRAN_MOD_PIO;
		} else if (drv_mode[host->id] == MODE_DMA) {
			msdc_latest_transfer_mode[host->id] = TRAN_MOD_DMA;
		} else if (drv_mode[host->id] == MODE_SIZE_DEP) {
			if (host->xfer_size < dma_size[host->id]) {
				data->host_cookie |= MSDC_COOKIE_PIO;
				msdc_latest_transfer_mode[host->id] =
					TRAN_MOD_PIO;
			} else {
				msdc_latest_transfer_mode[host->id] =
					TRAN_MOD_DMA;
			}
		}
		if (msdc_use_async_dma(data->host_cookie)) {
			dir = read ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
			(void)dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len,
				dir);
		}
		N_MSG(OPS, "CMD<%d> ARG<0x%x>data<%s %s> blksz<%d> block<%d> "
			"error<%d>",
			mrq->cmd->opcode, mrq->cmd->arg,
			(data->host_cookie ? "dma" : "pio"),
			(read ? "read " : "write"), data->blksz,
			data->blocks, data->error);
	}
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	else if (data && check_mmc_cmd4647(cmd->opcode)) {
		read = data->flags & MMC_DATA_READ ? 1 : 0;
		dir = read ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
		(void)dma_map_sg(mmc_dev(mmc), data->sg, data->sg_len, dir);
	}
#endif
}

static void msdc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
	int err)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data;
	/* struct mmc_command *cmd = mrq->cmd; */
	int dir = DMA_FROM_DEVICE;
	data = mrq->data;
	if (data && (msdc_use_async_dma(data->host_cookie))) {
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		if (!mmc->card->ext_csd.cmdq_mode_en)
#endif
			host->xfer_size = data->blocks * data->blksz;
		dir = data->flags & MMC_DATA_READ ?
			DMA_FROM_DEVICE : DMA_TO_DEVICE;
		dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len, dir);
		data->host_cookie = 0;
		N_MSG(OPS, "CMD<%d> ARG<0x%x> blksz<%d> block<%d> error<%d>",
			mrq->cmd->opcode, mrq->cmd->arg, data->blksz,
			data->blocks, data->error);
	}
	data->host_cookie = 0;

	return;
}

static int msdc_do_request_async(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	void __iomem *base = host->base;

#ifdef MTK_MSDC_USE_CMD23
	u32 l_card_no_cmd23 = 0;
#endif

#ifdef MTK_MSDC_USE_CACHE
	u32 l_force_prg = 0;
#endif
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	u32 task_id;
#endif

	MVG_EMMC_DECLARE_INT32(delay_ns);
	MVG_EMMC_DECLARE_INT32(delay_us);
	MVG_EMMC_DECLARE_INT32(delay_ms);

	BUG_ON(mmc == NULL);
	BUG_ON(mrq == NULL);

	if (!is_card_present(host) || host->power_mode == MMC_POWER_OFF) {
		ERR_MSG("cmd<%d> arg<0x%x> card<%d> power<%d>",
			mrq->cmd->opcode, mrq->cmd->arg,
			is_card_present(host), host->power_mode);
		mrq->cmd->error = (unsigned int)-ENOMEDIUM;
		if (mrq->done)
			mrq->done(mrq); /* call done directly. */
		return 0;
	}
	msdc_ungate_clock(host);
	/*#if defined(MSDC_AUTOK_ON_ERROR)*/
	host->async_tuning_in_progress = false;
	/*#endif*/

	spin_lock(&host->lock);

	cmd = mrq->cmd;
	data = mrq->cmd->data;

	host->mrq = mrq;

#ifdef MTK_MSDC_USE_CACHE
	if (msdc_do_request_prepare(host, mrq, cmd, data, &l_force_prg,
		NULL, PREPARE_ASYNC))
		goto done;
#else
	if (msdc_do_request_prepare(host, mrq, cmd, data, NULL,
		NULL, PREPARE_ASYNC))
		goto done;
#endif

#ifdef MTK_MSDC_USE_CMD23
	/* start the cmd23 first */
	if (mrq->sbc) {
		host->autocmd &= ~MSDC_AUTOCMD12;

		if (host->hw->host_function == MSDC_EMMC) {
#ifdef MTK_MSDC_USE_CACHE
			if (l_force_prg && !((mrq->sbc->arg >> 31) & 0x1))
				mrq->sbc->arg |= (1 << 24);
#endif
		}

		if (0 == (host->autocmd & MSDC_AUTOCMD23)) {
			if (msdc_command_start(host, mrq->sbc, 0,
				CMD_TIMEOUT) != 0)
				goto done;

			/* then wait command done */
			if (msdc_command_resp_polling(host, mrq->sbc, 0,
				CMD_TIMEOUT) != 0) {
				goto stop;
			}
		}
	} else {
		/* some sd card may not support cmd23,
		 * some emmc card have problem with cmd23,
		   so use cmd12 here */
		if (host->hw->host_function != MSDC_SDIO) {
			host->autocmd |= MSDC_AUTOCMD12;
			if (0 != (host->autocmd & MSDC_AUTOCMD23)) {
				host->autocmd &= ~MSDC_AUTOCMD23;
				l_card_no_cmd23 = 1;
			}
		}
	}

#else
	/* start the command first*/
	if (host->hw->host_function != MSDC_SDIO)
		host->autocmd |= MSDC_AUTOCMD12;
#endif /* end of MTK_MSDC_USE_CMD23 */

	msdc_dma_on();          /* enable DMA mode first!! */
	/* init_completion(&host->xfer_done); */

	if (msdc_command_start(host, cmd, 0, CMD_TIMEOUT) != 0)
		goto done;

	/* then wait command done */
	if (msdc_command_resp_polling(host, cmd, 0, CMD_TIMEOUT) != 0)
		goto stop;

	/* for read, the data coming too fast, then CRC error
	   start DMA no business with CRC. */
	msdc_dma_setup(host, &host->dma, data->sg, data->sg_len);

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	mmc->is_data_dma = 1;
#endif

	msdc_dma_start(host);
	/*ERR_MSG("0.Power cycle enable(%d)",host->power_cycle_enable);*/

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (check_mmc_cmd4647(cmd->opcode)) {
		task_id = (cmd->arg >> 16) & 0x1f;
		MVG_EMMC_WRITE_MATCH(host,
			(u64)mmc->areq_que[task_id]->mrq_que->cmd->arg,
			delay_ms, delay_us, delay_ns,
			cmd->opcode, host->xfer_size);
	} else
#endif
	MVG_EMMC_WRITE_MATCH(host, (u64)cmd->arg, delay_ms, delay_us, delay_ns,
		cmd->opcode, host->xfer_size);

	spin_unlock(&host->lock);

#if defined(FEATURE_MET_MMC_INDEX)
	met_mmc_issue(host->mmc, host->mrq);
#endif

#ifdef MTK_MSDC_USE_CMD23
	/* for msdc use cmd23, but card not supported(sbc is NULL),
	   need enable autocmd23 for next request */
	if (1 == l_card_no_cmd23) {
		if (host->hw->host_function != MSDC_SDIO) {
			host->autocmd |= MSDC_AUTOCMD23;
			host->autocmd &= ~MSDC_AUTOCMD12;
			l_card_no_cmd23 = 0;
		}
	}
#endif

#ifdef MTK_MSDC_USE_CACHE
	msdc_update_cache_flush_status(host, mrq, data, 1);
#endif

	return 0;


stop:
#ifndef MTK_MSDC_USE_CMD23
	/* Last: stop transfer */
	if (msdc_if_send_stop(host, cmd, data))
		goto done;
#else

	if (host->hw->host_function == MSDC_EMMC) {
		/* error handle will do msdc_abort_data() */
	} else {
		if (msdc_if_send_stop(host, cmd, data))
			goto done;
	}
#endif

done:
#ifdef MTK_MSDC_USE_CMD23
	/* for msdc use cmd23, but card not supported(sbc is NULL),
	   need enable autocmd23 for next request */
	if (1 == l_card_no_cmd23) {
		if (host->hw->host_function != MSDC_SDIO) {
			host->autocmd |= MSDC_AUTOCMD23;
			host->autocmd &= ~MSDC_AUTOCMD12;
			l_card_no_cmd23 = 0;
		}
	}
#endif

	msdc_dma_clear(host);

	msdc_log_cmd(host, cmd, data);

#ifdef MTK_MSDC_USE_CMD23
	if (mrq->sbc && (mrq->sbc->error == (unsigned int)-EIO))
		host->error |= REQ_CMD_EIO;
	if (mrq->sbc && (mrq->sbc->error == (unsigned int)-ETIMEDOUT)) {
#ifdef CONFIG_MTK_AEE_FEATURE
		aee_kernel_warning_api(__FILE__, __LINE__,
			DB_OPT_NE_JBT_TRACES|DB_OPT_DISPLAY_HANG_DUMP,
			"\n@eMMC FATAL ERROR@\n", "eMMC fatal error ");
#endif
		host->error |= REQ_CMD_TMO;
	}
#endif

	msdc_if_set_err(host, mrq, cmd);

#ifdef MTK_MSDC_USE_CACHE
	msdc_update_cache_flush_status(host, mrq, data, 1);
/*end2:*/
#endif

	if (!host->async_tuning_in_progress
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		&& !host->mmc->card->ext_csd.cmdq_mode_en
#endif
		) {
		if ((data && data->error)
		 || (cmd && (cmd->error == (unsigned int)-EIO))) {
			host->async_tuning_done = false;
		}
	}

	/* if cmd send error occur, dma not start yet, just call done here,
	   msdc_tune_async_request() will apply  */
	if (mrq->done)
		mrq->done(mrq);

	msdc_gate_clock(host, 1);
	spin_unlock(&host->lock);
	return host->error;
}

/* #define TUNE_FLOW_TEST */
#ifdef TUNE_FLOW_TEST
static void msdc_reset_para(struct msdc_host *host)
{
	void __iomem *base = host->base;
	u32 dsmpl, rsmpl, clkmode;
	int hs400 = 0;

	/* because we have a card, which must work at dsmpl<0> and rsmpl<0> */

	MSDC_GET_FIELD(MSDC_IOCON, MSDC_IOCON_R_D_SMPL, dsmpl);
	MSDC_GET_FIELD(MSDC_IOCON, MSDC_IOCON_RSPL, rsmpl);
	MSDC_GET_FIELD(MSDC_CFG, MSDC_CFG_CKMOD, clkmode);
	hs400 = (clkmode == 3) ? 1 : 0;

	if (dsmpl == 0) {
		msdc_set_smpl(host, hs400, 1, TYPE_READ_DATA_EDGE, NULL);
		ERR_MSG("set dspl<0>");
		MSDC_SET_FIELD(MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_CMDRDLY, 0);
	}

	if (rsmpl == 0) {
		msdc_set_smpl(host, hs400, 1, TYPE_CMD_RESP_EDGE, NULL);
		ERR_MSG("set rspl<0>");
		MSDC_WRITE32(MSDC_DAT_RDDLY0, 0);
		MSDC_SET_FIELD(MSDC_PAD_TUNE0, MSDC_PAD_TUNE0_DATWRDLY, 0);
	}
}
#endif

static void msdc_dump_trans_error(struct msdc_host   *host,
	struct mmc_command *cmd,
	struct mmc_data    *data,
	struct mmc_command *stop,
	struct mmc_command *sbc)
{
	if ((cmd->opcode == 52) && (cmd->arg == 0xc00))
		return;
	if ((cmd->opcode == 52) && (cmd->arg == 0x80000c08))
		return;

	if (!(is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ))) {
		/* by pass the SDIO CMD TO for SD/eMMC */
		if ((host->hw->host_function == MSDC_SD) &&
		    (cmd->opcode == 5))
			return;
	} else {
		if (cmd->opcode == 8)
			return;
	}

	ERR_MSG("XXX CMD<%d><0x%x> Error<%d> Resp<0x%x>", cmd->opcode, cmd->arg,
		cmd->error, cmd->resp[0]);

	if (data) {
		ERR_MSG("XXX DAT block<%d> Error<%d>", data->blocks,
			data->error);
	}
	if (stop) {
		ERR_MSG("XXX STOP<%d><0x%x> Error<%d> Resp<0x%x>",
			stop->opcode, stop->arg, stop->error, stop->resp[0]);
	}

	if (sbc) {
		ERR_MSG("XXX SBC<%d><0x%x> Error<%d> Resp<0x%x>",
			sbc->opcode, sbc->arg, sbc->error, sbc->resp[0]);
	}

	if ((host->hw->host_function == MSDC_SD)
	 && (host->sclk > 100000000)
	 && (data)
	 && (data->error != (unsigned int)-ETIMEDOUT)) {
		if ((data->flags & MMC_DATA_WRITE) &&
		    (host->write_timeout_uhs104))
			host->write_timeout_uhs104 = 0;
		if ((data->flags & MMC_DATA_READ) &&
		    (host->read_timeout_uhs104))
			host->read_timeout_uhs104 = 0;
	}

	if ((host->hw->host_function == MSDC_EMMC) &&
	    (data) &&
	    (data->error != (unsigned int)-ETIMEDOUT)) {
		if ((data->flags & MMC_DATA_WRITE) &&
		    (host->write_timeout_emmc))
			host->write_timeout_emmc = 0;
		if ((data->flags & MMC_DATA_READ) &&
		    (host->read_timeout_emmc))
			host->read_timeout_emmc = 0;
	}
#ifdef SDIO_ERROR_BYPASS
	if (is_card_sdio(host) &&
	    (host->sdio_error != -EIO) &&
	    (cmd->opcode == 53) &&
	    (msdc_sg_len(data->sg, host->dma_xfer) > 4)) {
		host->sdio_error = -EIO;
		ERR_MSG("XXX SDIO Error ByPass");
	}
#endif
}

static bool msdc_check_written_data(struct mmc_host *mmc,
	struct mmc_request *mrq)
{
	bool ret = 0;
	u32 result = 0;
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_card *card;
	if (msdc_use_async_pio(mrq->data->host_cookie))
		return 0;

	if (!is_card_present(host) ||
	    host->power_mode == MMC_POWER_OFF) {
		ERR_MSG("cmd<%d> arg<0x%x> card<%d> power<%d>",
			mrq->cmd->opcode, mrq->cmd->arg,
			is_card_present(host), host->power_mode);
		mrq->cmd->error = (unsigned int)-ENOMEDIUM;
		return 0;
	}
	if (mmc->card)
		card = mmc->card;
	else
		return 0;

	if ((host->hw->host_function == MSDC_SD) &&
	     (host->sclk > 100000000) &&
	     mmc_card_sd(card) &&
	     (mrq->data) &&
	     (mrq->data->flags & MMC_DATA_WRITE) &&
	     (host->error == 0)) {
		msdc_ungate_clock(host);
		spin_lock(&host->lock);
		if (msdc_polling_idle(host)) {
			spin_unlock(&host->lock);
			goto out;
		}
		spin_unlock(&host->lock);
		result = __mmc_sd_num_wr_blocks(card);
		if ((result != mrq->data->blocks) &&
		    (is_card_present(host)) &&
		    (host->power_mode == MMC_POWER_ON)) {
			mrq->data->error = (unsigned int)-EIO;
			host->error |= REQ_DAT_ERR;
			ERR_MSG("written data<%d> blocks isn't equal to "
				"request data blocks<%d>",
				result, mrq->data->blocks);
			ret = 1;
		}
out:
		msdc_gate_clock(host, 1);
	}
	return ret;
}

static void msdc_do_request_with_retry(struct msdc_host *host,
	struct mmc_request *mrq,
	struct mmc_command *cmd,
	struct mmc_data *data,
	struct mmc_command *stop,
	struct mmc_command *sbc,
	int async)
{
	do {
		if (!async) {
			if (!msdc_do_request(host->mmc, mrq))
				break;
		}
		/* there is some error*/
		/* becasue ISR executing time will be monitor,
		   try to dump info here.*/
		if (cmd->opcode != 19)
			msdc_dump_trans_error(host, cmd, data, stop, sbc);

		if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ)) {
			/* sdio will tune*/
			return;
		}

		if (host->legacy_tuning_in_progress)
			return;

		#ifdef MSDC_AUTOK_ON_ERROR /*define as 0 if runtime tune is used */
		if (host->mmc->ios.timing != MMC_TIMING_LEGACY
		 && host->mmc->ios.timing != MMC_TIMING_SD_HS
		 && host->mmc->ios.timing != MMC_TIMING_UHS_DDR50) {
			if (!host->legacy_tuning_in_progress) {
				if ((cmd && (cmd->error == (unsigned int)-EIO))
				 || (sbc && (sbc->error == (unsigned int)-EIO))
				 || (stop && (stop->error == (unsigned int)-EIO))
				 || (data && data->error)) {
					host->legacy_tuning_done = false;
				}

				return;
			}
		}
		#endif

#ifdef MTK_MSDC_USE_CMD23
		if ((sbc != NULL) &&
		    (sbc->error == (unsigned int)-ETIMEDOUT)) {
			if (check_mmc_cmd1825(cmd->opcode)) {
				/* not tuning, go out directly */
				pr_err("===[%s:%d]==cmd23 timeout==\n",
					__func__, __LINE__);
				return;
			}
		}
#endif

		if (msdc_crc_tune(host, cmd, data, stop, sbc))
			return;

		/* CMD TO -> not tuning */
		if (!async) {
			if (cmd->error == (unsigned int)-ETIMEDOUT &&
			    !check_mmc_cmd2425(cmd->opcode) &&
			    !check_mmc_cmd1718(cmd->opcode))
				return;
		}

		if (cmd->error == (unsigned int)-ENOMEDIUM)
			return;

		if (msdc_data_timeout_tune(host, data))
			return;

		/* clear the error condition. */
		cmd->error = 0;
		if (data)
			data->error = 0;
		if (stop)
			stop->error = 0;

#ifdef MTK_MSDC_USE_CMD23
		if (sbc)
			sbc->error = 0;
#endif

		/* check if an app commmand. */
		if (!async) {
			if (host->app_cmd) {
				while (msdc_app_cmd(host->mmc, host)) {
					if (msdc_tune_cmdrsp(host)) {
						ERR_MSG("failed to updata cmd"
							" para for app");
						return;
					}
				}
			}
		}

		if (async)
			host->sw_timeout = 0;
		if (!is_card_present(host))
			return;

		if (async) {
			if  (!msdc_tune_rw_request(host->mmc, mrq))
				break;
		}
	} while (1);

	if (async) {
		if ((host->rwcmd_time_tune)
		 && (check_mmc_cmd1718(cmd->opcode) ||
		     check_mmc_cmd2425(cmd->opcode))) {
			host->rwcmd_time_tune = 0;
			ERR_MSG("RW cmd recover");
			msdc_dump_trans_error(host, cmd, data, stop, sbc);
		}
	}
	if ((host->read_time_tune)
	 && check_mmc_cmd1718(cmd->opcode)) {
		host->read_time_tune = 0;
		ERR_MSG("Read recover");
		msdc_dump_trans_error(host, cmd, data, stop, sbc);
	}
	if ((host->write_time_tune)
	 && check_mmc_cmd2425(cmd->opcode)) {
		host->write_time_tune = 0;
		ERR_MSG("Write recover");
		msdc_dump_trans_error(host, cmd, data, stop, sbc);
	}

	if (async)
		host->power_cycle_enable = 1;
	host->sw_timeout = 0;

	if (host->hw->host_function == MSDC_SD)
		host->continuous_fail_request_count = 0;
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static int msdc_do_cmdq_request_with_retry(struct msdc_host *host,
	struct mmc_request *mrq)
{
	struct mmc_host *mmc;
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_command *stop = NULL;
	int ret = 0, retry;

	mmc = host->mmc;
	cmd = mrq->cmd;
	data = mrq->cmd->data;
	if (data)
		stop = data->stop;

	retry = 5;
	while (msdc_do_request_cq(mmc, mrq)) {
		msdc_dump_trans_error(host, cmd, data, stop, mrq->sbc);
		if ((cmd->error == (unsigned int)-EIO) ||
			(cmd->error == (unsigned int)-ETIMEDOUT) ||
			(mrq->sbc->error == (unsigned int)-EIO) ||
			(mrq->sbc->error == (unsigned int)-ETIMEDOUT)) {
			ret = tune_cmdq_cmdrsp(mmc, mrq, &retry);
			if (ret)
				return ret;
		} else {
			ERR_MSG("CMD44 and CMD45 error - error %d %d",
				mrq->sbc->error, cmd->error);
			break;
		}
	}

	return ret;
}
#endif

/* ops.request */
static void msdc_ops_request_legacy(struct mmc_host *mmc,
	struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_command *stop = NULL;
	struct mmc_command *sbc = NULL;
	/* === for sdio profile === */
	u32 old_H32 = 0, old_L32 = 0, new_H32 = 0, new_L32 = 0;
	u32 ticks = 0, opcode = 0, sizes = 0, bRx = 0;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	int ret;
#endif

	msdc_reset_crc_tune_counter(host, all_counter);
#ifndef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (host->mrq) {
		ERR_MSG("XXX host->mrq<0x%p> cmd<%d>arg<0x%x>", host->mrq,
			host->mrq->cmd->opcode, host->mrq->cmd->arg);
#ifdef VENDOR_EDIT
// Jingchun.Wang@Phone.Bsp.Driver, 2016/04/11  Modify for bad card 
			ERR_MSG("Esso ignore bug on msdc_ops_request_legacy\n");
#else
			BUG();
#endif /*VENDOR_EDIT*/
	}
#endif

	if (!is_card_present(host) || host->power_mode == MMC_POWER_OFF) {
		ERR_MSG("cmd<%d> arg<0x%x> card<%d> power<%d>",
			mrq->cmd->opcode, mrq->cmd->arg,
			is_card_present(host), host->power_mode);
		mrq->cmd->error = (unsigned int)-ENOMEDIUM;

#if 1
		if (mrq->done)
			mrq->done(mrq); /* call done directly. */
#else
		mrq->cmd->retries = 0;  /* please don't retry. */
		mmc_request_done(mmc, mrq);
#endif

		return;
	}

	/* start to process */
	spin_lock(&host->lock);
	host->power_cycle_enable = 1;

	cmd = mrq->cmd;
	data = mrq->cmd->data;
	if (data)
		stop = data->stop;

#ifdef MTK_MSDC_USE_CMD23
	if (data)
		sbc = mrq->sbc;
#endif

	msdc_ungate_clock(host);  /* set sw flag */

	if (sdio_pro_enable) {
		/*=== for sdio profile ===*/
		if (mrq->cmd->opcode == 52 || mrq->cmd->opcode == 53)
			; /*GPT_GetCounter64(&old_L32, &old_H32);*/
	}

#ifndef CONFIG_MTK_EMMC_CQ_SUPPORT
	host->mrq = mrq;
#endif

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (check_mmc_cmd44(mrq->sbc)) {
		ret = msdc_do_cmdq_request_with_retry(host, mrq);
		if (ret)
			goto out;
	} else {
		/* only CMD0/CMD12/CMD13 can be send
			when non-empty queue @ CMDQ on */
		if (mmc->card && mmc->card->ext_csd.cmdq_mode_en
			&& atomic_read(&mmc->areq_cnt)
			&& !check_mmc_cmd01213(cmd->opcode)
			&& !check_mmc_cmd48(cmd->opcode)) {
			ERR_MSG("[%s][WARNING] CMDQ on, sending CMD%d\n",
				__func__, cmd->opcode);
		}
		if (!check_mmc_cmd13_sqs(mrq->cmd))
			host->mrq = mrq;
#endif
	msdc_do_request_with_retry(host, mrq, cmd, data, stop, sbc, 0);

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	}
out:
#endif
	msdc_reset_crc_tune_counter(host, all_counter);
#ifdef MTK_MSDC_USE_CACHE
	msdc_check_cache_flush_error(host, cmd);
#endif

#ifdef TUNE_FLOW_TEST
	if (!is_card_sdio(host))
		msdc_reset_para(host);
#endif

	/* ==== when request done, check if app_cmd ==== */
	if (mrq->cmd->opcode == MMC_APP_CMD) {
		host->app_cmd = 1;
		host->app_cmd_arg = mrq->cmd->arg;      /* save the RCA */
	} else {
		host->app_cmd = 0;
		/* host->app_cmd_arg = 0; */
	}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (!(check_mmc_cmd13_sqs(mrq->cmd)
		|| check_mmc_cmd44(mrq->sbc))) {
		/* if not CMDQ CMD44/45 or CMD13, follow orignal flow to clear host->mrq
		if it's CMD44/45 or CMD13 QSR, host->mrq may be CMD46,47 */
		host->mrq = NULL;
	}
#else
	host->mrq = NULL;
#endif

	/* === for sdio profile === */
	if (sdio_pro_enable) {
		if (mrq->cmd->opcode == 52 || mrq->cmd->opcode == 53) {
			/* GPT_GetCounter64(&new_L32, &new_H32); */
			ticks = msdc_time_calc(old_L32, old_H32,
				new_L32, new_H32);

			opcode = mrq->cmd->opcode;
			if (mrq->cmd->data) {
				sizes = mrq->cmd->data->blocks *
					mrq->cmd->data->blksz;
				bRx = mrq->cmd->data->flags & MMC_DATA_READ ?
					1 : 0;
			} else {
				bRx = mrq->cmd->arg & 0x80000000 ? 1 : 0;
			}

			if (!mrq->cmd->error)
				msdc_performance(opcode, sizes, bRx, ticks);
		}
	}

	msdc_gate_clock(host, 1);       /* clear flag. */
	spin_unlock(&host->lock);

	mmc_request_done(mmc, mrq);

	return;
}


static void msdc_tune_async_request(struct mmc_host *mmc,
	struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_command *stop = NULL;
	struct mmc_command *sbc = NULL;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (mmc->card->ext_csd.cmdq_mode_en == 1
		&& (atomic_read(&mmc->cq_tuning_now) == 1)) {
		tune_cmdq_data(mmc, mrq);
		return;
	}
#endif

	/* msdc_reset_crc_tune_counter(host,all_counter) */
	if (host->mrq) {
#ifdef CONFIG_MTK_AEE_FEATURE
		aee_kernel_warning("MSDC",
				   "MSDC request not clear.\n host "
				   "attached<0x%p> current<0x%p>.\n",
				   host->mrq, mrq);
#else
		WARN_ON(host->mrq);
#endif
		ERR_MSG("XXX host->mrq<0x%p> cmd<%d>arg<0x%x>", host->mrq,
			host->mrq->cmd->opcode, host->mrq->cmd->arg);
		if (host->mrq->data) {
			ERR_MSG("XXX request data size<%d>",
				host->mrq->data->blocks *
				host->mrq->data->blksz);
			ERR_MSG("XXX request attach to host "
				"force data timeout and retry");
			host->mrq->data->error = (unsigned int)-ETIMEDOUT;
		} else {
			ERR_MSG("XXX request attach to host "
				"force cmd timeout and retry");
			host->mrq->cmd->error = (unsigned int)-ETIMEDOUT;
		}
		ERR_MSG("XXX current request <0x%p> cmd<%d>arg<0x%x>",
			mrq, mrq->cmd->opcode, mrq->cmd->arg);
		if (mrq->data)
			ERR_MSG("XXX current request data size<%d>",
				mrq->data->blocks * mrq->data->blksz);
	}

	if (!is_card_present(host) || host->power_mode == MMC_POWER_OFF) {
		ERR_MSG("cmd<%d> arg<0x%x> card<%d> power<%d>",
			mrq->cmd->opcode, mrq->cmd->arg,
			is_card_present(host), host->power_mode);
		mrq->cmd->error = (unsigned int)-ENOMEDIUM;
		/* mrq->done(mrq);*/
		return;
	}

	cmd = mrq->cmd;
	data = mrq->cmd->data;
	if (msdc_use_async_pio(mrq->data->host_cookie))
		return;
	if (data)
		stop = data->stop;
#ifdef MTK_MSDC_USE_CMD23
	if (data)
		sbc = mrq->sbc;
#endif

#ifdef MTK_MSDC_USE_CMD23
	if ((cmd->error == 0)
	 && (data && data->error == 0)
	 && (!stop || stop->error == 0)
	 && ((sbc == NULL) || (sbc && sbc->error == 0))) {
#else
	if ((cmd->error == 0)
	 && (data && data->error == 0)
	 && (!stop || stop->error == 0)) {
#endif
		if (check_mmc_cmd1718(cmd->opcode))
			host->read_time_tune = 0;
		if (check_mmc_cmd2425(cmd->opcode))
			host->write_time_tune = 0;
		host->rwcmd_time_tune = 0;
		host->power_cycle_enable = 1;
		return;
	}
	/* start to process */
	spin_lock(&host->lock);

	msdc_ungate_clock(host);        /* set sw flag */
	/*#if defined(MSDC_AUTOK_ON_ERROR)*/
	host->async_tuning_in_progress = true;
	/*#endif*/
	host->mrq = mrq;

	msdc_do_request_with_retry(host, mrq, cmd, data, stop, sbc, 1);

	if (host->sclk <= 50000000
/*
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	 && (host->timing != MMC_TIMING_MMC_DDR52)
#endif
*/
	 && (host->timing != MMC_TIMING_UHS_DDR50))
		host->sd_30_busy = 0;
	msdc_reset_crc_tune_counter(host, all_counter);
	host->mrq = NULL;
	msdc_gate_clock(host, 1);       /* clear flag. */
	/*#if defined(MSDC_AUTOK_ON_ERROR)*/
	host->async_tuning_in_progress = false;
	/*#endif*/
	spin_unlock(&host->lock);

	/* mmc_request_done(mmc, mrq); */
	return;
}

int msdc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);

	host->legacy_tuning_in_progress = true;

	msdc_init_tune_path(host, mmc->ios.timing);

	msdc_ungate_clock(host);

	#if !defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_force_vcore_pwm(true);
	#endif

	if (host->hw->host_function == MSDC_SD) {
		
		if (mmc->ios.timing == MMC_TIMING_UHS_SDR104 ||
		    mmc->ios.timing == MMC_TIMING_UHS_SDR50) {
		    	if (host->is_sd_autok_done == 0) {
				pr_err("[AUTOK]SD Autok\n");
				autok_execute_tuning(host, NULL);
				host->is_sd_autok_done = 1;
			}
			else {
				autok_init_sdr104(host);
				autok_tuning_parameter_init(host, sd1_autok_res);
			}
		}
	} else if (host->hw->host_function == MSDC_EMMC) {
		#ifdef MSDC_HQA
		msdc_HQA_set_vcore(host);
		#endif

		if (mmc->ios.timing == MMC_TIMING_MMC_HS200) {
			if (opcode == MMC_SEND_STATUS) {
				pr_err("[AUTOK]eMMC HS200 Tune CMD only\n");
				hs200_execute_tuning_cmd(host, NULL);
			} else {
				pr_err("[AUTOK]eMMC HS200 Tune\n");
				hs200_execute_tuning(host, NULL);
			}
		} else if (mmc->ios.timing == MMC_TIMING_MMC_HS400) {
			if (opcode == MMC_SEND_STATUS) {
				pr_err("[AUTOK]eMMC HS400 Tune CMD only\n");
				hs400_execute_tuning_cmd(host, NULL);
			} else {
				pr_err("[AUTOK]eMMC HS400 Tune\n");
				hs400_execute_tuning(host, NULL);
			}
		}
	} else if (host->hw->host_function == MSDC_SDIO) {
		/* Default autok result is not exist, always excute tuning */
		if (sdio_autok_res_apply(host, AUTOK_VCORE_HIGH) != 0) {
			pr_err("sdio autok result not exist!, excute tuning\n");
			if (host->is_autok_done == 0) {
				pr_err("[AUTOK]SDIO SDR104 Tune\n");
				/* Wait DFVS ready for excute autok here */
				sdio_autok_wait_dvfs_ready();

				/* Performance mode, return 0 pass */
				if (vcorefs_request_dvfs_opp(KIR_AUTOK_SDIO, OPPI_PERF) != 0)
					pr_err("vcorefs_request_dvfs_opp@OPPI_PERF fail!\n");
				autok_execute_tuning(host, sdio_autok_res[AUTOK_VCORE_HIGH]);

			#ifdef SDIO_FIX_VCORE_CONDITIONAL
				/* Low power mode, return 0 pass */
				if (vcorefs_request_dvfs_opp(KIR_AUTOK_SDIO, OPPI_LOW_PWR) != 0)
					pr_err("vcorefs_request_dvfs_opp@OPPI_PERF fail!\n");
				autok_execute_tuning(host, sdio_autok_res[AUTOK_VCORE_LOW]);

				if (autok_res_check(sdio_autok_res[AUTOK_VCORE_HIGH], sdio_autok_res[AUTOK_VCORE_LOW]) == 0) {
					pr_err("[AUTOK] No need change para when dvfs\n");
				} else {
					pr_err("[AUTOK] Need change para when dvfs or lock dvfs\n");
					sdio_lock_dvfs = 1;
				}
			#else
				/* SDIO E2 */
				if (sdio_version(host) == 2) {
					sdio_set_vcorefs_sram(AUTOK_VCORE_HIGH, 0, host);
					/* Low power mode, return 0 pass */
					if (vcorefs_request_dvfs_opp(KIR_AUTOK_SDIO, OPPI_LOW_PWR) != 0)
						pr_err("vcorefs_request_dvfs_opp@OPPI_PERF fail!\n");
					autok_execute_tuning(host, sdio_autok_res[AUTOK_VCORE_LOW]);
					sdio_set_vcorefs_sram(AUTOK_VCORE_LOW, 1, host);
				}
			#endif

				/* Un-request, return 0 pass */
				if (vcorefs_request_dvfs_opp(KIR_AUTOK_SDIO, OPPI_UNREQ) != 0)
					pr_err("vcorefs_request_dvfs_opp@OPPI_UNREQ fail!\n");

				host->is_autok_done = 1;
				complete(&host->autok_done);
			} else {
				autok_init_sdr104(host);
				autok_tuning_parameter_init(host, sdio_autok_res[AUTOK_VCORE_HIGH]);
			}
		} else {
			autok_init_sdr104(host);

		#ifdef SDIO_FIX_VCORE_CONDITIONAL
			if (autok_res_check(sdio_autok_res[AUTOK_VCORE_HIGH], sdio_autok_res[AUTOK_VCORE_LOW]) == 0) {
				pr_err("[AUTOK] No need change para when dvfs\n");
			} else {
				pr_err("[AUTOK] Need change para when dvfs or lock dvfs\n");
				sdio_lock_dvfs = 1;
			}
		#else
			/* SDIO E2 */
			if (sdio_version(host) == 2) {
				sdio_set_vcorefs_sram(AUTOK_VCORE_HIGH, 0, host);
				if (sdio_autok_res_apply(host, AUTOK_VCORE_LOW) == 0)
					sdio_set_vcorefs_sram(AUTOK_VCORE_LOW, 1, host);
			}
		#endif

			if (host->is_autok_done == 0) {
				host->is_autok_done = 1;
				complete(&host->autok_done);
			}
		}
	}
	host->legacy_tuning_in_progress = false;
	host->legacy_tuning_done = true;
	host->first_tune_done = 1;

	#if !defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_force_vcore_pwm(false);
	#endif

	msdc_gate_clock(host, 0);

	return 0;
}

static void msdc_ops_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmc_data *data;
	int host_cookie = 0;
	struct msdc_host *host = mmc_priv(mmc);
	BUG_ON(mmc == NULL);
	BUG_ON(mrq == NULL);
#ifndef CONFIG_HAS_EARLYSUSPEND
	if ((host->hw->host_function == MSDC_SDIO) &&
	    !(host->trans_lock.active))
		__pm_stay_awake(&host->trans_lock);
#else
	if ((host->hw->host_function == MSDC_SDIO) &&
	    !wake_lock_active(&host->trans_lock))
		wake_lock(&host->trans_lock);
#endif

	/* 6630 in msdc2 and SDIO need lock dvfs */
	if ((host->id == 2) && (sdio_lock_dvfs == 1)) {
		sdio_set_vcore_performance(host, 1);
	}

	data = mrq->data;
	if (data)
		host_cookie = data->host_cookie;
	/* Asyn only support  DMA and asyc CMD flow */
	if (msdc_use_async_dma(host_cookie)) {
		#if defined(MSDC_AUTOK_ON_ERROR)
		if (!host->async_tuning_in_progress &&
		    !host->async_tuning_done) {
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
			if (host->mmc->card->ext_csd.cmdq_mode_en)
				ERR_MSG("[%s][ERROR] CMDQ on, not support async tuning",
					__func__);
#endif
			if (mmc->ios.timing == MMC_TIMING_UHS_SDR104) {
				msdc_execute_tuning(mmc,
					MMC_SEND_TUNING_BLOCK);
			} else if (mmc->ios.timing == MMC_TIMING_MMC_HS200) {
				msdc_execute_tuning(mmc,
					MMC_SEND_TUNING_BLOCK_HS200);
			} else if (mmc->ios.timing == MMC_TIMING_MMC_HS400) {
				msdc_execute_tuning(mmc,
					MMC_SEND_TUNING_BLOCK_HS200);
			}
			host->async_tuning_in_progress = false;
			host->async_tuning_done = true;
		}
		#endif
		msdc_do_request_async(mmc, mrq);
	} else {
		if (!host->legacy_tuning_in_progress
		 && !host->legacy_tuning_done) {
			if (mmc->ios.timing == MMC_TIMING_UHS_SDR104) {
				msdc_execute_tuning(mmc,
					MMC_SEND_TUNING_BLOCK);
			} else if (mmc->ios.timing == MMC_TIMING_MMC_HS200) {
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
				if (host->error == REQ_CMD_EIO) {
					msdc_execute_tuning(mmc, MMC_SEND_STATUS);
					host->error &= ~REQ_CMD_EIO;
				} else
#endif
				msdc_execute_tuning(mmc,
					MMC_SEND_TUNING_BLOCK_HS200);
			} else if (mmc->ios.timing == MMC_TIMING_MMC_HS400) {
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
				if (host->error == REQ_CMD_EIO) {
					msdc_execute_tuning(mmc, MMC_SEND_STATUS);
					host->error &= ~REQ_CMD_EIO;
				} else
#endif
				msdc_execute_tuning(mmc,
					MMC_SEND_TUNING_BLOCK_HS200);
			}
		}
		msdc_ops_request_legacy(mmc, mrq);
	}

	/* 6630 in msdc2 and SDIO need lock dvfs */
	if ((host->id == 2) && (sdio_lock_dvfs == 1)) {
		sdio_set_vcore_performance(host, 0);
	}

#ifndef CONFIG_HAS_EARLYSUSPEND
	if ((host->hw->host_function == MSDC_SDIO) && (host->trans_lock.active))
		__pm_relax(&host->trans_lock);
#else
	if ((host->hw->host_function == MSDC_SDIO) &&
	    wake_lock_active(&host->trans_lock))
		wake_unlock(&host->trans_lock);
#endif

	return;
}


/* called by ops.set_ios */
static void msdc_set_buswidth(struct msdc_host *host, u32 width)
{
	void __iomem *base = host->base;
	u32 val = MSDC_READ32(SDC_CFG);

	val &= ~SDC_CFG_BUSWIDTH;

	switch (width) {
	default:
	case MMC_BUS_WIDTH_1:
		width = 1;
		val |= (MSDC_BUS_1BITS << 16);
		break;
	case MMC_BUS_WIDTH_4:
		val |= (MSDC_BUS_4BITS << 16);
		break;
	case MMC_BUS_WIDTH_8:
		val |= (MSDC_BUS_8BITS << 16);
		break;
	}

	MSDC_WRITE32(SDC_CFG, val);

	N_MSG(CFG, "Bus Width = %d", width);
}

static void msdc_reconfig_mode(struct mmc_host *mmc,
	struct msdc_host *host)
{
	struct mmc_card *card = mmc->card;

	#ifdef CONFIG_EMMC_50_FEATURE
	if (!(mmc->caps2 & MMC_CAP2_HS200_1_8V_SDR) &&
	     !(mmc->caps2 & MMC_CAP2_HS400_1_8V_DDR)) {
	#else
	if (!(mmc->caps2 & MMC_CAP2_HS200_1_8V_SDR)) {
	#endif
		mmc->f_max = 50000000;
		if (card && card->ext_csd.hs_max_dtr > 52000000) {
			card->state &=
				~(MMC_STATE_HIGHSPEED_200 |
				  MMC_STATE_HIGHSPEED_400);
			if (card->ext_csd.raw_card_type
			  & EXT_CSD_CARD_TYPE_MASK
			  & EXT_CSD_CARD_TYPE_DDR_1_8V)
				card->ext_csd.card_type |=
				    EXT_CSD_CARD_TYPE_DDR_1_8V;
		}
	} else {
		mmc->f_max = HOST_MAX_MCLK;
		if (card && card->ext_csd.hs_max_dtr > 52000000)
			card->ext_csd.card_type &= ~EXT_CSD_CARD_TYPE_DDR_1_8V;
	}
	pr_err("[%s]: msdc%d, ext_csd.card_type=0x%x\n",
		__func__, host->id, mmc->card->ext_csd.card_type);
}

/* called by msdc_drv_probe */
static void msdc_init_hw(struct msdc_host *host)
{
	void __iomem *base = host->base;
	struct msdc_hw *hw = host->hw;

	/* Power on */
	/*msdc_pin_reset(host, MSDC_PIN_PULL_UP, 0);*/

	msdc_ungate_clock(host);

	/* Configure to MMC/SD mode */
	MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_MODE, MSDC_SDMMC);

	/* Reset */
	msdc_reset_hw(host->id);

	/* Disable card detection */
	MSDC_CLR_BIT32(MSDC_PS, MSDC_PS_CDEN);

	/* Disable and clear all interrupts */
	MSDC_CLR_BIT32(MSDC_INTEN, MSDC_READ32(MSDC_INTEN));
	MSDC_WRITE32(MSDC_INT, MSDC_READ32(MSDC_INT));

	/* reset tuning parameter */
	msdc_init_tune_setting(host);

	/* for safety, should clear SDC_CFG.SDIO_INT_DET_EN & set SDC_CFG.SDIO
	   in pre-loader,uboot,kernel drivers.
	   SDC_CFG.SDIO_INT_DET_EN will be only set when kernel driver wants
	   to use SDIO bus interrupt */
	/* Enable SDIO mode. it's must otherwise sdio cmd5 failed */
	MSDC_SET_BIT32(SDC_CFG, SDC_CFG_SDIO);

	/* disable detect SDIO device interupt function */
	if (host->hw->flags & MSDC_SDIO_IRQ) {
		ghost = host;
		/* enable sdio detection */
		MSDC_SET_BIT32(SDC_CFG, SDC_CFG_SDIOIDE);
	} else {
		MSDC_CLR_BIT32(SDC_CFG, SDC_CFG_SDIOIDE);
	}

	msdc_set_smt(host, 1);
	msdc_set_driving(host, hw, 0);
	/*msdc_set_pin_mode(host);*/
	/*msdc_set_ies(host, 1);*/

	INIT_MSG("msdc drving<clk %d,cmd %d,dat %d>",
		hw->clk_drv, hw->cmd_drv, hw->dat_drv);

	/* write crc timeout detection */
	MSDC_SET_FIELD(MSDC_PATCH_BIT0, MSDC_PB0_DETWR_CRCTMO, 1);

	/* Configure to default data timeout */
	MSDC_SET_FIELD(SDC_CFG, SDC_CFG_DTOC, DEFAULT_DTOC);

	msdc_set_buswidth(host, MMC_BUS_WIDTH_1);

	msdc_gate_clock(host, 1);

	N_MSG(FUC, "init hardware done!");
}

/* ops.set_ios */
static void msdc_ops_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct msdc_hw *hw = host->hw;

	spin_lock(&host->lock);

	msdc_ungate_clock(host);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
	case MMC_POWER_UP:
		spin_unlock(&host->lock);
		if (ios->power_mode == MMC_POWER_UP &&
		    host->power_mode == MMC_POWER_OFF)
			msdc_init_hw(host);
		msdc_set_power_mode(host, ios->power_mode);
		spin_lock(&host->lock);
		break;
	case MMC_POWER_ON:
		host->power_mode = MMC_POWER_ON;
		break;
	default:
		break;
	}

	msdc_set_buswidth(host, ios->bus_width);

	if ((msdc_host_mode[host->id] != mmc->caps) ||
	    (msdc_host_mode2[host->id] != mmc->caps2)) {
		mmc->caps = msdc_host_mode[host->id];
		mmc->caps2 = msdc_host_mode2[host->id];
#ifdef CONFIG_MTK_EMMC_SUPPORT
		if (1 == g_emmc_mode_switch)
			msdc_reconfig_mode(mmc, host);
#endif

		msdc_init_tune_setting(host);

	}

	if (msdc_clock_src[host->id] != hw->clk_src) {
		hw->clk_src = msdc_clock_src[host->id];
		msdc_select_clksrc(host, hw->clk_src);
	}

	if (host->mclk != ios->clock || host->timing != ios->timing) {
		/* not change when clock Freq.
		   not changed state need set clock*/
		if (ios->clock > 100000000)
			msdc_set_driving(host, hw, 1);

		msdc_ios_tune_setting(mmc, ios);
		msdc_set_mclk(host, ios->timing, ios->clock);
		host->timing = ios->timing;
	}

	msdc_gate_clock(host, 1);
	spin_unlock(&host->lock);
}

/* ops.get_ro */
static int msdc_ops_get_ro(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	unsigned long flags;
	int ro = 0;

	spin_lock_irqsave(&host->lock, flags);
	msdc_ungate_clock(host);
	if (host->hw->flags & MSDC_WP_PIN_EN)
		ro = (MSDC_READ32(MSDC_PS) >> 31);
	msdc_gate_clock(host, 1);
	spin_unlock_irqrestore(&host->lock, flags);
	return ro;
}

/* ops.get_cd */
static int msdc_ops_get_cd(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base;
	unsigned long flags;

	base = host->base;
	spin_lock_irqsave(&host->lock, flags);

	/* for sdio, depends on USER_RESUME */
	if (is_card_sdio(host)) {
		if (!(host->hw->flags & MSDC_SDIO_IRQ)) {
			host->card_inserted =
				(host->pm_state.event == PM_EVENT_USER_RESUME) ?
					1 : 0;
			goto end;
		}
	}

	/* for emmc, MSDC_REMOVABLE not set, always return 1 */
	if (!(host->hw->flags & MSDC_REMOVABLE)) {
		host->card_inserted = 1;
		goto end;
	}

	if (host->hw->flags & MSDC_CD_PIN_EN) {
		/* for card, MSDC_CD_PIN_EN set */
		host->card_inserted =
			(host->sd_cd_polarity == host->hw->cd_level) ? 0 : 1;
	} else {
		host->card_inserted = 1;
	}

	/* host->card_inserted = 1; */
	if (host->hw->host_function == MSDC_SD && host->block_bad_card)
		host->card_inserted = 0;
 end:
	INIT_MSG("Card insert<%d> Block bad card<%d>",
		host->card_inserted, host->block_bad_card);
	spin_unlock_irqrestore(&host->lock, flags);
	return host->card_inserted;
}

/* ops.enable_sdio_irq */
static void msdc_ops_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct msdc_hw *hw = host->hw;
	void __iomem *base = host->base;
	unsigned long flags;

	if (hw->flags & MSDC_EXT_SDIO_IRQ) {    /* yes for sdio */
		if (enable)
			hw->enable_sdio_eirq(); /* combo_sdio_enable_eirq */
		else
			hw->disable_sdio_eirq(); /* combo_sdio_disable_eirq */
	} else if (hw->flags & MSDC_SDIO_IRQ) {

#if (MSDC_DATA1_INT == 1)
		spin_lock_irqsave(&host->sdio_irq_lock, flags);

		if (enable) {
			while (1) {
				MSDC_SET_BIT32(MSDC_INTEN, MSDC_INT_SDIOIRQ);
				pr_debug("@#0x%08x @e >%d<\n",
					(MSDC_READ32(MSDC_INTEN)),
					host->mmc->sdio_irq_pending);
				if ((MSDC_READ32(MSDC_INTEN) & MSDC_INT_SDIOIRQ)
					== 0) {
					pr_debug("Should never ever get into this >%d<\n",
						host->mmc->sdio_irq_pending);
				} else {
					break;
				}
			}
		} else {
			MSDC_CLR_BIT32(MSDC_INTEN, MSDC_INT_SDIOIRQ);
			pr_debug("@#0x%08x @d\n", (MSDC_READ32(MSDC_INTEN)));
		}

		spin_unlock_irqrestore(&host->sdio_irq_lock, flags);
#endif
	}
}

static int msdc_ops_switch_volt(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	int err = 0;
	u32 timeout = 100;
	u32 retry = 10;
	u32 status;
	u32 sclk = host->sclk;
	u32 i;

	if (host->hw->host_function == MSDC_EMMC)
		return 0;

	if (ios->signal_voltage != MMC_SIGNAL_VOLTAGE_330) {
		/* make sure SDC is not busy (TBC) */
		/* WAIT_COND(!SDC_IS_BUSY(), timeout, timeout); */
		err = (unsigned int)-EIO;
		msdc_retry(sdc_is_busy(), retry, timeout, host->id);
		if (timeout == 0 && retry == 0) {
			err = (unsigned int)-ETIMEDOUT;
			goto out;
		}

		/* pull up disabled in CMD and DAT[3:0]
		   to allow card drives them to low */
		/* check if CMD/DATA lines both 0 */
		if ((MSDC_READ32(MSDC_PS) & ((1 << 24) | (0xF << 16))) == 0) {
			/* pull up disabled in CMD and DAT[3:0] */
			msdc_pin_config(host, MSDC_PIN_PULL_NONE);

			if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {

				if (host->power_switch)
					host->power_switch(host, 1);

			}
			/* wait at least 5ms for card to switch to 1.8v signal*/
			mdelay(10);

			/* config clock to 10~12MHz mode for
			   volt switch detection by host. */

			/*For FPGA 13MHz clock, this not work*/
			msdc_set_mclk(host, MMC_TIMING_LEGACY, 260000);

			/* pull up enabled in CMD and DAT[3:0] */
			msdc_pin_config(host, MSDC_PIN_PULL_UP);
			mdelay(105);

			/* start to detect volt change
			   by providing 1.8v signal to card */
			MSDC_SET_BIT32(MSDC_CFG, MSDC_CFG_BV18SDT);

			/* wait at max. 1ms */
			mdelay(1);
			/* ERR_MSG("before read status"); */

			for (i = 0; i < 5000; i++) {
				status = MSDC_READ32(MSDC_CFG);
				if (!(status & MSDC_CFG_BV18SDT))
					break;

				mdelay(1);
			}

			if (i >= 5000) {
				pr_err("msdc1, voltage switch timeout!!\n");
				msdc_dump_info(1);
				BUG_ON(1);
			}


			if (status & MSDC_CFG_BV18PSS)
				err = 0;
			/* ERR_MSG("msdc V1800 status (0x%x),err(%d)",
				status,err); */
			/* config clock back to init clk freq. */
			msdc_set_mclk(host, MMC_TIMING_LEGACY, sclk);
		}
	}
 out:

	return err;
}

/* async way: mmc_start_req() ->  __mmc_start_req() -> mmc_start_request()
		-> request() -> mmc_wait_for_req_done() -> msdc_ops_stop()
		-> mmc_post_req()
 * legacy way: mmc_wait_for_req() -> __mmc_start_req -> mmc_start_request()
		-> request()
 * msdc_send_stop() just for async way. for when to trigger stop cmd(arg=0):
 * 1 aysnc way but used pio mode will call msdc_ops_request_legacy(),
	and pio mode will disable autocmd12,
	so cmd12 will send in msdc_do_request()
 * 2 aysnc way with non-cmd23 mode: if host does not use autocmd12,
	sw need send cmd12 after mrq->completion (polling in mmc_core.c) is done
 * 3 aysnc way with cmd23 mode: no need to send cmd12 here
 * sd card will not enable cmd23 */
static void msdc_ops_stop(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 err = -1;

	if (msdc_use_async_pio(mrq->data->host_cookie))
		return;

	if (host->hw->host_function != MSDC_SDIO) {
		if (!mrq->stop)
			return;

#ifndef MTK_MSDC_USE_CMD23
		if ((host->autocmd & MSDC_AUTOCMD12))
			return;
#else
		if (host->hw->host_function == MSDC_EMMC) {
			/* if transfer error occur,
			   cmd12 will send under msdc_abort_data() */
			if (mrq->sbc)
				return;
		} else {
			if ((host->autocmd & MSDC_AUTOCMD12))
				return;
		}
#endif                          /* end of MTK_MSDC_USE_CMD23 */

		N_MSG(OPS, "MSDC Stop for non-autocmd12 host->error(%d) "
			"host->autocmd(%d)",
			host->error, host->autocmd);
		err = msdc_do_command(host, mrq->stop, 0, CMD_TIMEOUT);
		if (err) {
			if (mrq->stop->error == (unsigned int)-EIO)
				host->error |= REQ_STOP_EIO;
			if (mrq->stop->error == (unsigned int)-ETIMEDOUT)
				host->error |= REQ_STOP_TMO;
		}
	}
}

/* Add this function to check if no interrupt back after write.         *
 * It may occur when write crc revice, but busy over data->timeout_ns   */
static void msdc_check_write_timeout(struct work_struct *work)
{
	struct msdc_host *host =
		container_of(work, struct msdc_host, write_timeout.work);
	void __iomem *base = host->base;
	struct mmc_data  *data = host->data;
	struct mmc_request *mrq = host->mrq;
	struct mmc_host *mmc = host->mmc;

	u32 status = 0;
	u32 state = 0;
	u32 err = 0;
	unsigned long tmo;

	if (!data || !mrq || !mmc)
		return;

	pr_err("[%s]: XXX DMA Data Write Busy Timeout: %u ms, CMD<%d>",
		__func__, host->write_timeout_ms, mrq->cmd->opcode);

	if ((msdc_use_async_dma(data->host_cookie)) &&
	    (!host->async_tuning_in_progress)) {
		msdc_dump_info(host->id);

		msdc_dma_stop(host);
		msdc_dma_clear(host);
		msdc_reset_hw(host->id);

		tmo = jiffies + POLLING_BUSY;

		/* check the card state, try to bring back to trans state */
		spin_lock(&host->lock);
		do {
			/* if anything wrong, let block driver do error
			   handling. */
			err = msdc_get_card_status(mmc, host, &status);
			if (err) {
				ERR_MSG("CMD13 ERR<%d>", err);
				break;
			}

			state = R1_CURRENT_STATE(status);
			ERR_MSG("check card state<%d>", state);
			if (state == R1_STATE_DATA || state == R1_STATE_RCV) {
				ERR_MSG("state<%d> need cmd12 to stop", state);
				msdc_send_stop(host);
			} else if (state == R1_STATE_PRG) {
				ERR_MSG("state<%d> card is busy", state);
				spin_unlock(&host->lock);
				msleep(100);
				spin_lock(&host->lock);
			}

			if (time_after(jiffies, tmo)) {
				ERR_MSG("abort timeout and stuck in %d state,"
					"remove such bad card!" , state);
				spin_unlock(&host->lock);
				msdc_set_bad_card_and_remove(host);
				spin_lock(&host->lock);
				break;
			}
		} while (state != R1_STATE_TRAN);
		spin_unlock(&host->lock);

		data->error = (unsigned int)-ETIMEDOUT;
		host->sw_timeout++;

		if (mrq->done)
			mrq->done(mrq);

		msdc_gate_clock(host, 1);
		host->error |= REQ_DAT_ERR;
	} else {
		/* do nothing, since legacy mode or async tuning
		   have it own timeout. */
		/* complete(&host->xfer_done); */
	}
}

static void msdc_dma_error_reset(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	void __iomem *base = host->base;
	struct mmc_data *data = host->data;
	if (data && host->dma_xfer &&
	    (msdc_use_async_dma(data->host_cookie)) &&
	    (!host->async_tuning_in_progress)) {
		host->sw_timeout++;
		host->error |= REQ_DAT_ERR;
		msdc_dump_info(host->id);
		msdc_reset(host->id);
		msdc_dma_stop(host);
		msdc_clr_fifo(host->id);
		msdc_clr_int();
		msdc_dma_clear(host);
		msdc_gate_clock(host, 1);
	}
}

static struct mmc_host_ops mt_msdc_ops = {
	.post_req                      = msdc_post_req,
	.pre_req                       = msdc_pre_req,
	.request                       = msdc_ops_request,
	.tuning                        = msdc_tune_async_request,
	.set_ios                       = msdc_ops_set_ios,
	.get_ro                        = msdc_ops_get_ro,
	.get_cd                        = msdc_ops_get_cd,
	.enable_sdio_irq               = msdc_ops_enable_sdio_irq,
	.start_signal_voltage_switch   = msdc_ops_switch_volt,
	.send_stop                     = msdc_ops_stop,
	.dma_error_reset               = msdc_dma_error_reset,
	.check_written_data            = msdc_check_written_data,
	.execute_tuning                = msdc_execute_tuning,
};

/*--------------------------------------------------------------------------*/
/* interrupt handler                 */
/*--------------------------------------------------------------------------*/
/*static __tcmfunc irqreturn_t msdc_irq(int irq, void *dev_id)*/
#ifndef FPGA_PLATFORM
static irqreturn_t msdc1_eint_handler(void)
{
	struct msdc_host *host = mtk_msdc_host[1];
	int got_bad_card = 0;
	unsigned long flags;

	spin_lock_irqsave(&host->remove_bad_card, flags);
	if (host->hw->cd_level ^ host->sd_cd_polarity) {
		got_bad_card = host->block_bad_card;
		host->card_inserted = 0;
		if (host->mmc && host->mmc->card)
			mmc_card_set_removed(host->mmc->card);
	}
	host->sd_cd_polarity = (~(host->sd_cd_polarity))&0x1;
	spin_unlock_irqrestore(&host->remove_bad_card, flags);
	host->block_bad_card = 0;

	if (host->sd_cd_polarity == 0)
		irq_set_irq_type(cd_irq, IRQ_TYPE_LEVEL_LOW);
	else
		irq_set_irq_type(cd_irq, IRQ_TYPE_LEVEL_HIGH);

	if (got_bad_card == 0)
		tasklet_hi_schedule(&host->card_tasklet);
	pr_err("SD card %s(%x:%x)",
		(host->hw->cd_level ^ host->sd_cd_polarity) ? "insert":"remove",
		host->hw->cd_level, host->sd_cd_polarity);

	return IRQ_HANDLED;
}
#endif

#if defined(FEATURE_MET_MMC_INDEX)
extern void met_mmc_dma_stop(struct mmc_host *host, u32 lba, unsigned int len,
	u32 opcode, unsigned int bd_num);
#endif

static void msdc_irq_data_complete(struct msdc_host *host,
	struct mmc_data *data, int error)
{
	void __iomem *base = host->base;
	struct mmc_request *mrq;

	if ((msdc_use_async_dma(data->host_cookie)) &&
	    (!host->async_tuning_in_progress)) {
		msdc_dma_stop(host);
		if (error) {
			msdc_clr_fifo(host->id);
			msdc_clr_int();
		}
		mrq = host->mrq;
		msdc_dma_clear(host);
		if (mrq->done)
			mrq->done(mrq);
		msdc_gate_clock(host, 1);
		if (!error)
			host->error &= ~REQ_DAT_ERR;
		else
			host->error |= REQ_DAT_ERR;
	} else {
		complete(&host->xfer_done);
	}

#if defined(FEATURE_MET_MMC_INDEX)
	if ((data->mrq != NULL) && (data->mrq->cmd != NULL)) {
		met_mmc_dma_stop(host->mmc, data->mrq->cmd->arg,
			data->blocks, data->mrq->cmd->opcode, met_mmc_bdnum);
	}
#endif
}

static irqreturn_t msdc_irq(int irq, void *dev_id)
{
	struct msdc_host *host = (struct msdc_host *)dev_id;
	struct mmc_data *data = host->data;
	struct mmc_command *cmd = host->cmd;
	struct mmc_command *stop = NULL;
	/*struct mmc_request *mrq = NULL;*/
	void __iomem *base = host->base;

	u32 cmdsts = MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO | MSDC_INT_CMDRDY |
		     MSDC_INT_ACMDCRCERR | MSDC_INT_ACMDTMO | MSDC_INT_ACMDRDY |
		     MSDC_INT_ACMD19_DONE;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	u32 cmdqsts = MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO | MSDC_INT_CMDRDY;
#endif
	u32 datsts = MSDC_INT_DATCRCERR | MSDC_INT_DATTMO;
	u32 intsts, inten;

	if (host->hw->flags & MSDC_SDIO_IRQ)
		spin_lock(&host->sdio_irq_lock);


	if (host->core_clkon == 0) {
		msdc_clk_enable(host);
		host->core_clkon = 1;
		MSDC_SET_FIELD(MSDC_CFG, MSDC_CFG_MODE, MSDC_SDMMC);
	}
	intsts = MSDC_READ32(MSDC_INT);

	latest_int_status[host->id] = intsts;
	inten = MSDC_READ32(MSDC_INTEN);
#if (MSDC_DATA1_INT == 1)
	if (host->hw->flags & MSDC_SDIO_IRQ) {
		intsts &= inten;
	} else
#endif
	{
		inten &= intsts;
	}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	/* don't clear command related interrupt bits,
		these will be cleared by command polling response */
	intsts &= ~cmdqsts;
#endif

	MSDC_WRITE32(MSDC_INT, intsts); /* clear interrupts */

	/* MSG will cause fatal error */
#if 0
	/* card change interrupt */
	if (intsts & MSDC_INT_CDSC) {
		IRQ_MSG("MSDC_INT_CDSC irq<0x%.8x>", intsts);
		tasklet_hi_schedule(&host->card_tasklet);
		/* tuning when plug card ? */
	}
#endif

	/* sdio interrupt */
	if (host->hw->flags & MSDC_SDIO_IRQ) {
		spin_unlock(&host->sdio_irq_lock);

		#if (MSDC_DATA1_INT == 1)
		if (intsts & MSDC_INT_SDIOIRQ)
			mmc_signal_sdio_irq(host->mmc);
		#endif
	}

	/* transfer complete interrupt */
	if (data == NULL)
		goto skip_data_interrupts;

#ifdef MTK_MSDC_ERROR_TUNE_DEBUG
	msdc_error_tune_debug2(host, stop, &intsts);
#endif

	stop = data->stop;
#if (MSDC_DATA1_INT == 1)
	if ((host->hw->flags & MSDC_SDIO_IRQ) &&
	    (intsts & MSDC_INT_XFER_COMPL)) {
		goto done;
	} else
#endif
	{
		if (host->hw->host_function == MSDC_SD)
			host->continuous_fail_request_count = 0;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		host->mmc->is_data_dma = 0;
#endif
		if (inten & MSDC_INT_XFER_COMPL)
			goto done;
	}

	if (intsts & datsts) {
		/* do basic reset, or stop command will sdc_busy */
		if (intsts & MSDC_INT_DATTMO)
			msdc_dump_info(host->id);

		if (host->dma_xfer)
			msdc_reset(host->id);
		else
			msdc_reset_hw(host->id);

		atomic_set(&host->abort, 1);    /* For PIO mode exit */

		if (intsts & MSDC_INT_DATTMO) {
			data->error = (unsigned int)-ETIMEDOUT;
			ERR_MSG("XXX CMD<%d> Arg<0x%.8x> MSDC_INT_DATTMO",
				host->mrq->cmd->opcode, host->mrq->cmd->arg);
		} else if (intsts & MSDC_INT_DATCRCERR) {
			data->error = (unsigned int)-EIO;
			ERR_MSG("XXX CMD<%d> Arg<0x%.8x> MSDC_INT_DATCRCERR, "
				"SDC_DCRC_STS<0x%x>",
				host->mrq->cmd->opcode, host->mrq->cmd->arg,
				MSDC_READ32(SDC_DCRC_STS));
		}

		goto tune;
	}
	if ((stop != NULL) &&
	    (host->autocmd & MSDC_AUTOCMD12) &&
	    (intsts & cmdsts)) {
		if (intsts & MSDC_INT_ACMDRDY) {
			u32 *arsp = &stop->resp[0];
			*arsp = MSDC_READ32(SDC_ACMD_RESP);
		} else if (intsts & MSDC_INT_ACMDCRCERR) {
			stop->error = (unsigned int)-EIO;
			host->error |= REQ_STOP_EIO;
			if (host->dma_xfer)
				msdc_reset(host->id);
			else
				msdc_reset_hw(host->id);
		} else if (intsts & MSDC_INT_ACMDTMO) {
			stop->error = (unsigned int)-ETIMEDOUT;
			host->error |= REQ_STOP_TMO;
			if (host->dma_xfer)
				msdc_reset(host->id);
			else
				msdc_reset_hw(host->id);
		}
		if ((intsts & MSDC_INT_ACMDCRCERR) ||
		    (intsts & MSDC_INT_ACMDTMO)) {
			goto skip_cmd_interrupts;
		}
	}

skip_data_interrupts:

	/* command interrupts */
	if ((cmd == NULL) || !(intsts & cmdsts))
		goto skip_cmd_interrupts;

#ifdef MTK_MSDC_ERROR_TUNE_DEBUG
	msdc_error_tune_debug3(host, cmd, &intsts);
#endif

#ifndef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (intsts & MSDC_INT_CMDRDY) {
		u32 *rsp = NULL;
		rsp = &cmd->resp[0];
		switch (host->cmd_rsp) {
		case RESP_NONE:
			break;
		case RESP_R2:
			*rsp++ = MSDC_READ32(SDC_RESP3);
			*rsp++ = MSDC_READ32(SDC_RESP2);
			*rsp++ = MSDC_READ32(SDC_RESP1);
			*rsp++ = MSDC_READ32(SDC_RESP0);
			break;
		default: /* Response types 1, 3, 4, 5, 6, 7(1b) */
			*rsp = MSDC_READ32(SDC_RESP0);
			break;
		}

		if (host->hw->host_function == MSDC_SD)
			host->continuous_fail_request_count = 0;
	} else if (intsts & MSDC_INT_RSPCRCERR) {
		cmd->error = (unsigned int)-EIO;
		ERR_MSG("XXX CMD<%d> MSDC_INT_RSPCRCERR Arg<0x%.8x>",
			cmd->opcode, cmd->arg);
		msdc_reset_hw(host->id);
	} else if (intsts & MSDC_INT_CMDTMO) {
		cmd->error = (unsigned int)-ETIMEDOUT;
		ERR_MSG("XXX CMD<%d> MSDC_INT_CMDTMO Arg<0x%.8x>",
			cmd->opcode, cmd->arg);
		msdc_reset_hw(host->id);
	}
	if (intsts & (MSDC_INT_CMDRDY | MSDC_INT_RSPCRCERR | MSDC_INT_CMDTMO))
		complete(&host->cmd_done);
#endif

skip_cmd_interrupts:
	/* mmc irq interrupts */
	if (intsts & MSDC_INT_MMCIRQ) {
		/* pr_debug("msdc[%d] MMCIRQ: SDC_CSTS=0x%.8x\r\n",
			host->id, MSDC_READ32(SDC_CSTS)); */
	}

	if (!host->async_tuning_in_progress
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		&& (!host->mmc->card ||
			!host->mmc->card->ext_csd.cmdq_mode_en)
#endif
		) {
		if (cmd && (cmd->error == (unsigned int)-EIO)) {
			host->async_tuning_done = false;
		}
	}

	if (host->dma_xfer)
		msdc_irq_data_complete(host, data, 1);

	latest_int_status[host->id] = 0;
	return IRQ_HANDLED;

done:   /* Finished data transfer */
	data->bytes_xfered = host->dma.xfersz;
	msdc_irq_data_complete(host, data, 0);
	return IRQ_HANDLED;

tune:   /* DMA DATA transfer crc error */
	/* PIO mode can't do complete, because not init */
	if (!host->async_tuning_in_progress
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
		&& (!host->mmc->card ||
			!host->mmc->card->ext_csd.cmdq_mode_en)
#endif
		) {
		if ((data && data->error)
		 || (cmd && (cmd->error == (unsigned int)-EIO))) {
			host->async_tuning_done = false;
		}
	}

	if (host->dma_xfer)
		msdc_irq_data_complete(host, data, 1);

	return IRQ_HANDLED;
}

/*--------------------------------------------------------------------------*/
/* platform_driver members                                                  */
/*--------------------------------------------------------------------------*/
/* called by msdc_drv_probe/remove */
static void msdc_enable_cd_irq(struct msdc_host *host, int enable)
{
#ifndef FPGA_PLATFORM
	struct msdc_hw *hw = host->hw;

	unsigned int gpiopin, debounce;
	unsigned int ints[2] = {0, 0};
	unsigned long flags;

	void __iomem *base = host->base;
	MSDC_CLR_BIT32(MSDC_PS, MSDC_PS_CDEN);
	MSDC_CLR_BIT32(MSDC_INTEN, MSDC_INTEN_CDSC);
	MSDC_CLR_BIT32(SDC_CFG, SDC_CFG_INSWKUP);

	if ((eint_node == NULL) || (cd_irq == 0))
		return;

	if (enable) {
		/* get gpio pin & debounce time */
		of_property_read_u32_array(eint_node, "debounce", ints,
			ARRAY_SIZE(ints));

		/* set debounce */
		gpiopin = ints[GPIOPIN];
		debounce = ints[DEBOUNCE];
		mt_gpio_set_debounce(gpiopin, debounce);

		if (hw->cd_level) {
			host->sd_cd_polarity = 1;
			flags = IRQF_TRIGGER_HIGH;
		} else {
			host->sd_cd_polarity = 0;
			flags = IRQF_TRIGGER_LOW;
		}
		/* request irq for eint (either way) */
		if (request_irq(cd_irq, (irq_handler_t)msdc1_eint_handler,
				flags, "MSDC1_INS-eint", NULL))
			pr_err("MSDC1 IRQ LINE NOT AVAILABLE!!\n");
	} else {
		disable_irq(cd_irq);
	}
#else
	struct msdc_hw *hw = host->hw;
	void __iomem *base = host->base;

	/* for sdio, not set */
	if ((hw->flags & MSDC_CD_PIN_EN) == 0) {
		MSDC_CLR_BIT32(MSDC_PS, MSDC_PS_CDEN);
		MSDC_CLR_BIT32(MSDC_INTEN, MSDC_INTEN_CDSC);
		MSDC_CLR_BIT32(SDC_CFG, SDC_CFG_INSWKUP);
		return;
	}

	N_MSG(CFG, "CD IRQ Eanable(%d)", enable);

	if (enable) {
		if (hw->enable_cd_eirq) { /* not set, never enter */
			hw->enable_cd_eirq();
		} else {
			/* card detection circuit relies on core power
			*  so that core power shouldn't be turned off.
			*  Here adds a reference count to keep core power alive.
			*/
			if (hw->config_gpio_pin) /* NULL */
				hw->config_gpio_pin(MSDC_CD_PIN,
					MSDC_GPIO_PULL_UP);

			MSDC_SET_FIELD(MSDC_PS, MSDC_PS_CDDEBOUNCE,
				DEFAULT_DEBOUNCE);
			MSDC_SET_BIT32(MSDC_PS, MSDC_PS_CDEN);
			MSDC_SET_BIT32(MSDC_INTEN, MSDC_INTEN_CDSC);
			MSDC_SET_BIT32(SDC_CFG, SDC_CFG_INSWKUP);
		}
	} else {
		if (hw->disable_cd_eirq) {
			hw->disable_cd_eirq();
		} else {
			if (hw->config_gpio_pin) /* NULL */
				hw->config_gpio_pin(MSDC_CD_PIN,
					MSDC_GPIO_PULL_DOWN);

			MSDC_CLR_BIT32(SDC_CFG, SDC_CFG_INSWKUP);
			MSDC_CLR_BIT32(MSDC_PS, MSDC_PS_CDEN);
			MSDC_CLR_BIT32(MSDC_INTEN, MSDC_INTEN_CDSC);

			/* Decreases a reference count to core power since card
			* detection circuit is shutdown.
			*/
		}
	}
#endif
}

void msdc_dump_gpd_bd(int id)
{
	struct msdc_host *host;
	int i = 0;
	gpd_t *gpd;
	bd_t  *bd;

	if (id < 0 || id >= HOST_MAX_NUM)
		pr_err("[%s]: invalide host id: %d\n", __func__, id);

	host = mtk_msdc_host[id];
	if (host == NULL) {
		pr_err("[%s]: host0 or host0->dma is NULL\n", __func__);
		return;
	}
	gpd = host->dma.gpd;
	bd  = host->dma.bd;

	pr_err("================MSDC GPD INFO ==================\n");
	if (gpd == NULL) {
		pr_err("GPD is NULL\n");
	} else {
		pr_err("gpd addr:0x%lx\n", (ulong)(host->dma.gpd_addr));
		pr_err("hwo:0x%x, bdp:0x%x, rsv0:0x%x, chksum:0x%x,intr:0x%x,rsv1:0x%x,nexth4:0x%x,ptrh4:0x%x\n",
			gpd->hwo, gpd->bdp, gpd->rsv0, gpd->chksum,
			gpd->intr, gpd->rsv1, (unsigned int)gpd->nexth4,
			(unsigned int)gpd->ptrh4);
		pr_err("next:0x%x, ptr:0x%x, buflen:0x%x, extlen:0x%x, arg:0x%x,blknum:0x%x,cmd:0x%x\n",
			(unsigned int)gpd->next, (unsigned int)gpd->ptr,
			gpd->buflen, gpd->extlen, gpd->arg, gpd->blknum,
			gpd->cmd);
	}
	pr_err("================MSDC BD INFO ===================\n");
	if (bd == NULL) {
		pr_err("BD is NULL\n");
	} else {
		pr_err("bd addr:0x%lx\n", (ulong)(host->dma.bd_addr));
		for (i = 0; i < host->dma.sglen; i++) {
			pr_err("the %d BD\n", i);
			pr_err("        eol:0x%x, rsv0:0x%x, chksum:0x%x, rsv1:0x%x,blkpad:0x%x,dwpad:0x%x,rsv2:0x%x\n",
				bd->eol, bd->rsv0, bd->chksum, bd->rsv1,
				bd->blkpad, bd->dwpad, bd->rsv2);
			pr_err("        nexth4:0x%x, ptrh4:0x%x, next:0x%x, ptr:0x%x, buflen:0x%x, rsv3:0x%x\n",
				(unsigned int)bd->nexth4,
				(unsigned int)bd->ptrh4, (unsigned int)bd->next,
				(unsigned int)bd->ptr, bd->buflen, bd->rsv3);
		}
	}
}

/* init gpd and bd list in msdc_drv_probe */
static void msdc_init_gpd_bd(struct msdc_host *host, struct msdc_dma *dma)
{
	gpd_t *gpd = dma->gpd;
	bd_t *bd = dma->bd;
	bd_t *ptr, *prev;

	/* we just support one gpd */
	int bdlen = MAX_BD_PER_GPD;

	/* init the 2 gpd */
	memset(gpd, 0, sizeof(gpd_t) * 2);
	gpd->next = (u32)dma->gpd_addr + sizeof(gpd_t);

	/* gpd->intr = 0; */
	gpd->bdp = 1;           /* hwo, cs, bd pointer */
	/* gpd->ptr  = (void*)virt_to_phys(bd); */
	gpd->ptr = (u32)dma->bd_addr; /* physical address */

	memset(bd, 0, sizeof(bd_t) * bdlen);
	ptr = bd + bdlen - 1;
	while (ptr != bd) {
		prev = ptr - 1;
		prev->next = ((u32)dma->bd_addr + sizeof(bd_t) * (ptr - bd));
		ptr = prev;
	}
}

#ifdef MTK_MSDC_FLUSH_BY_CLK_GATE
static void msdc_tasklet_flush_cache(unsigned long arg)
{
	struct msdc_host *host = (struct msdc_host *)arg;

	if (host->mmc->card) {
		mmc_claim_host(host->mmc);
		mmc_flush_cache(host->mmc->card);
		mmc_release_host(host->mmc);
	}

	return;
}
#endif

/* This is called by run_timer_softirq */
static void msdc_timer_pm(unsigned long data)
{
	struct msdc_host *host = (struct msdc_host *)data;
	unsigned long flags;

	spin_lock_irqsave(&host->clk_gate_lock, flags);
	if (host->clk_gate_count == 0) {
		msdc_clksrc_onoff(host, 0);
		N_MSG(CLK, "time out, dsiable clock, clk_gate_count=%d",
			host->clk_gate_count);
	}
#ifdef MTK_MSDC_FLUSH_BY_CLK_GATE
	if (check_mmc_cache_ctrl(mmc->card))
		tasklet_hi_schedule(&host->flush_cache_tasklet);
#endif

	spin_unlock_irqrestore(&host->clk_gate_lock, flags);
}

/* FIX ME : consider if this function can be moved to msdc_io.c */
static void msdc_set_host_power_control(struct msdc_host *host)
{
	u32 val;

	switch (host->id) {
	case 0:
		if (MSDC_EMMC == host->hw->host_function)
			host->power_control = msdc_emmc_power;
		break;
	case 1:
		if (MSDC_SD == host->hw->host_function) {
			host->power_control = msdc_sd_power;
			host->power_switch = msdc_sd_power_switch;
		}

		#if !defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		/* VMCH calibration default change to 0mv. We use 3.0V */
		pmic_read_interface(REG_VMCH_VOSEL_CAL, &val,
			MASK_VMCH_VOSEL_CAL,
			SHIFT_VMCH_VOSEL_CAL
			);

		pr_err("msdc1, 0xACE=%x\n", val);

		if (5 > val)
			val = 0x20 + val - 5;
		else
			val = val - 5;

		pmic_config_interface(REG_VMCH_VOSEL_CAL, val,
			MASK_VMCH_VOSEL_CAL,
			SHIFT_VMCH_VOSEL_CAL
			);
		pr_err("msdc1, 0xACE=%x\n", val);

		/* VMC calibration default not +100mv. We use 3.0V */
		pmic_read_interface(REG_VMC_VOSEL_CAL, &val,
			MASK_VMC_VOSEL_CAL,
			SHIFT_VMC_VOSEL_CAL
			);

		pr_err("msdc1, 0xAE2=%x\n", val);

		if (0x1b > val)
			val = 0x20 + val - 0x1b;
		else
			val = val - 0x1b;

		pr_err("msdc1, 0xAE2=%x\n", val);

		host->vmc_cal_default = val;
		pmic_config_interface(REG_VMC_VOSEL_CAL, val,
			MASK_VMC_VOSEL_CAL,
			SHIFT_VMC_VOSEL_CAL
			);
		#else
		pmic_read_interface(REG_VMC_VOSEL_CAL, &val,
			MASK_VMC_VOSEL_CAL,
			SHIFT_VMC_VOSEL_CAL
			);

		host->vmc_cal_default = val;
		#endif

		break;
	case 2:
		if (MSDC_SDIO == host->hw->host_function)
			host->power_control = msdc_sdio_power;

		break;
	default:
		return;
	}

	if (host->power_control == NULL) {
		ERR_MSG("Host function defination error for msdc%d", host->id);
		BUG();
	}
}

void SRC_trigger_signal(int i_on)
{
	if ((ghost != NULL) && (ghost->hw->flags & MSDC_SDIO_IRQ)) {
		pr_debug("msdc2 SRC_trigger_signal %d\n", i_on);
		src_clk_control = i_on;
		if (src_clk_control) {
			msdc_clksrc_onoff(ghost, 1);
			/* mb(); */
			if (ghost->mmc->sdio_irq_thread &&
			    (atomic_read(&ghost->mmc->sdio_irq_thread_abort)
				== 0)) {/* if (ghost->mmc->sdio_irq_thread) */
				mmc_signal_sdio_irq(ghost->mmc);
				if (u_msdc_irq_counter < 3)
					pr_debug("msdc2 SRC_trigger_signal mmc_signal_sdio_irq\n");
			}
			/* pr_debug("msdc2 SRC_trigger_signal ghost->id=%d\n",
				ghost->id); */
		}
	}

}
EXPORT_SYMBOL(SRC_trigger_signal);

#ifdef CONFIG_MTK_HIBERNATION
int msdc_drv_pm_restore_noirq(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mmc_host *mmc = NULL;
	struct msdc_host *host = NULL;
	u32 l_polarity = 0;
	BUG_ON(pdev == NULL);
	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);
	if (host->hw->host_function == MSDC_SD) {
		if ((host->id == 1) && (host->hw->flags & MSDC_CD_PIN_EN)) {
			l_polarity = mt_eint_get_polarity_external(cd_irq);
			if (l_polarity == MT_POLARITY_LOW)
				host->sd_cd_polarity = 0;
			else
				host->sd_cd_polarity = 1;

			if (!(host->hw->cd_level ^ host->sd_cd_polarity) &&
			     host->mmc->card) {
				mmc_card_set_removed(host->mmc->card);
				host->card_inserted = 0;
			}
		} else if ((host->id == 2) &&
			   (host->hw->flags & MSDC_CD_PIN_EN)) {
			/* sdio need handle here */
		}
		host->block_bad_card = 0;
	}
	return 0;
}
#endif

/* called by msdc_drv_remove */
static void msdc_deinit_hw(struct msdc_host *host)
{
	void __iomem *base = host->base;

	/* Disable and clear all interrupts */
	MSDC_CLR_BIT32(MSDC_INTEN, MSDC_READ32(MSDC_INTEN));
	MSDC_WRITE32(MSDC_INT, MSDC_READ32(MSDC_INT));

	/* Disable card detection */
	msdc_enable_cd_irq(host, 0);
	/* make sure power down */
	msdc_set_power_mode(host, MMC_POWER_OFF);
}

static int msdc_drv_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct msdc_host *host;
	struct msdc_hw *hw = NULL;
	void __iomem *base;
	u32 *hclks;
	unsigned int irq;
	int ret;

#ifdef MTK_MSDC_USE_CACHE
	BOOTMODE boot_mode;
#endif

	ret = msdc_dt_init(pdev, &cd_irq, &base, &irq, &hw);
	if (ret)
		return ret;

	/* Allocate MMC host for this device */
	mmc = mmc_alloc_host(sizeof(struct msdc_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	/* Set host parameters to mmc */
	mmc->ops        = &mt_msdc_ops;
	mmc->f_min      = HOST_MIN_MCLK;
	mmc->f_max      = HOST_MAX_MCLK;
	mmc->ocr_avail  = MSDC_OCR_AVAIL;

	/* For sd card: MSDC_SYS_SUSPEND | MSDC_WP_PIN_EN | MSDC_CD_PIN_EN |
			MSDC_REMOVABLE | MSDC_HIGHSPEED,
	   For sdio   : MSDC_EXT_SDIO_IRQ | MSDC_HIGHSPEED */
	if (hw->data_pins == 4)
		mmc->caps  |= MMC_CAP_4_BIT_DATA;
	else if (hw->data_pins == 8)
		mmc->caps  |= MMC_CAP_8_BIT_DATA | MMC_CAP_4_BIT_DATA;

	if ((hw->flags & MSDC_SDIO_IRQ) || (hw->flags & MSDC_EXT_SDIO_IRQ))
		mmc->caps |= MMC_CAP_SDIO_IRQ;  /* yes for sdio */

	msdc_set_caps_speed(mmc, hw);

	if (!(hw->flags & MSDC_REMOVABLE))
		mmc->caps |= MMC_CAP_NONREMOVABLE;

#ifdef MTK_MSDC_USE_CMD23
	if (hw->host_function == MSDC_EMMC)
		mmc->caps |= MMC_CAP_ERASE | MMC_CAP_WAIT_WHILE_BUSY |
			MMC_CAP_CMD23;
	else
		mmc->caps |= MMC_CAP_ERASE | MMC_CAP_WAIT_WHILE_BUSY;
#else
	mmc->caps |= MMC_CAP_ERASE | MMC_CAP_WAIT_WHILE_BUSY;
#endif

#ifdef MTK_MSDC_USE_CACHE
	/* only enable emmc cache feature for normal boot up,
	   alarm boot up, and sw reboot*/
	boot_mode = get_boot_mode();

	if ((hw->host_function == MSDC_EMMC)
	 && (hw->flags & MSDC_CACHE)
	 && ((boot_mode == NORMAL_BOOT) ||
	     (boot_mode == ALARM_BOOT) ||
	     (boot_mode == SW_REBOOT)))
		mmc->caps2 |= MMC_CAP2_CACHE_CTRL;
#endif

	/* MMC core transfer sizes tunable parameters */
	mmc->max_segs = MAX_HW_SGMTS;
	/*mmc->max_phys_segs = MAX_PHY_SGMTS;*/
	if (hw->host_function == MSDC_SDIO)
		mmc->max_seg_size  = MAX_SGMT_SZ_SDIO;
	else
		mmc->max_seg_size  = MAX_SGMT_SZ;
	mmc->max_blk_size  = HOST_MAX_BLKSZ;
	mmc->max_req_size  = MAX_REQ_SZ;
	mmc->max_blk_count = MAX_REQ_SZ / 512; /*mmc->max_req_size;*/

	hclks = msdc_get_hclks(pdev->id);

	host                    = mmc_priv(mmc);
	host->hw                = hw;
	host->mmc               = mmc;  /* msdc_check_init_done() need */
	host->id                = pdev->id;
	host->error             = 0;
	host->irq               = irq;
	host->base              = base;
	host->mclk              = 0;    /* request clock of mmc */
	host->hclk              = hclks[hw->clk_src];
					/* clocksource to msdc */
	host->sclk              = 0;    /* sd/sdio/emmc bus clock */
	host->pm_state          = PMSG_RESUME;
	host->suspend           = 0;

	INIT_DELAYED_WORK(&(host->set_vcore_workq), sdio_unreq_vcore);

	init_completion(&host->autok_done);
	host->is_autok_done     = 0;

	host->core_clkon        = 0;
	host->clk_gate_count    = 0;
	host->core_power        = 0;
	host->power_mode        = MMC_POWER_OFF;
	host->power_control     = NULL;
	host->power_switch      = NULL;

	host->dma_mask          = DMA_BIT_MASK(33);
	mmc_dev(mmc)->dma_mask  = &host->dma_mask;

#ifndef CONFIG_MTK_CLKMGR
	if (msdc_get_ccf_clk_pointer(pdev, host))
		return 1;
#endif

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2016.01.12 for fat 733813
	if (host->hw->host_function == MSDC_SD)
		host->hw->cmd_edge = 0;
#endif /* VENDOR_EDIT */

	msdc_set_host_power_control(host);
	if ((host->hw->host_function == MSDC_SD) &&
	    (host->hw->flags & MSDC_CD_PIN_EN)) {
		/* Since SD card power is default on,
		   it shall be turned off so that removalbe card slot won't
		    keep power when there is no card plugged */
		msdc_sd_power(host, 1); /* turn on first to match HW/SW state*/
		msdc_sd_power(host, 0);
	}

	if (host->hw->host_function == MSDC_EMMC &&
	    !(host->hw->flags & MSDC_UHS1))
		host->mmc->f_max = 50000000;
	/* host->card_inserted = hw->flags & MSDC_REMOVABLE ? 0 : 1; */
	host->timeout_ns = 0;
	host->timeout_clks = DEFAULT_DTOC * 1048576;

#ifndef MTK_MSDC_USE_CMD23
	if (host->hw->host_function != MSDC_SDIO)
		host->autocmd |= MSDC_AUTOCMD12;
	else
		host->autocmd &= ~MSDC_AUTOCMD12;
#else
	if (host->hw->host_function == MSDC_EMMC) {
		host->autocmd &= ~MSDC_AUTOCMD12;

#if (1 == MSDC_USE_AUTO_CMD23)
		host->autocmd |= MSDC_AUTOCMD23;
#endif

	} else if (host->hw->host_function == MSDC_SD) {
		host->autocmd |= MSDC_AUTOCMD12;
	} else {
		host->autocmd &= ~MSDC_AUTOCMD12;
	}
#endif  /* end of MTK_MSDC_USE_CMD23 */

	host->mrq = NULL;

	host->dma.used_gpd = 0;
	host->dma.used_bd = 0;

	/* using dma_alloc_coherent *//* todo: using 1, for all 4 slots */
	host->dma.gpd =
		dma_alloc_coherent(&pdev->dev, MAX_GPD_NUM * sizeof(gpd_t),
			&host->dma.gpd_addr, GFP_KERNEL);
	host->dma.bd =
		dma_alloc_coherent(&pdev->dev, MAX_BD_NUM * sizeof(bd_t),
			&host->dma.bd_addr, GFP_KERNEL);
	BUG_ON((!host->dma.gpd) || (!host->dma.bd));
	msdc_init_gpd_bd(host, &host->dma);
	msdc_clock_src[host->id] = hw->clk_src;
	msdc_host_mode[host->id] = mmc->caps;
	msdc_host_mode2[host->id] = mmc->caps2;
	/*for emmc */
	mtk_msdc_host[pdev->id] = host;
	host->write_timeout_uhs104 = 0;
	host->write_timeout_emmc = 0;
	host->read_timeout_uhs104 = 0;
	host->read_timeout_emmc = 0;
	host->sw_timeout = 0;
	host->async_tuning_in_progress = false;
	host->async_tuning_done = true;
	host->legacy_tuning_in_progress = false;
	host->legacy_tuning_done = true;
	host->timing = 0;
	host->sd_cd_insert_work = 0;
	host->block_bad_card = 0;
	host->sd_30_busy = 0;
	msdc_reset_tmo_tune_counter(host, all_counter);
	msdc_reset_pwr_cycle_counter(host);

	if (host->hw->host_function == MSDC_SDIO) {
#ifndef CONFIG_HAS_EARLYSUSPEND
		wakeup_source_init(&host->trans_lock, "MSDC Transfer Lock");
#else
		wake_lock_init(&host->trans_lock, WAKE_LOCK_SUSPEND,
			"MSDC Transfer Lock");
#endif
	}
	tasklet_init(&host->card_tasklet, msdc_tasklet_card, (ulong) host);
#ifdef MTK_MSDC_FLUSH_BY_CLK_GATE
	if (host->mmc->caps2 & MMC_CAP2_CACHE_CTRL)
		tasklet_init(&host->flush_cache_tasklet,
			msdc_tasklet_flush_cache, (ulong) host);
#endif
	/*INIT_DELAYED_WORK(&host->remove_card, msdc_remove_card);*/
	INIT_DELAYED_WORK(&host->write_timeout, msdc_check_write_timeout);
	spin_lock_init(&host->lock);
	spin_lock_init(&host->clk_gate_lock);
	spin_lock_init(&host->remove_bad_card);
	spin_lock_init(&host->sdio_irq_lock);
	/* init dynamtic timer */
	init_timer(&host->timer);
	/*host->timer.expires = jiffies + HZ;*/
	host->timer.function = msdc_timer_pm;
	host->timer.data = (unsigned long)host;

	ret = request_irq(irq, msdc_irq, IRQF_TRIGGER_NONE, DRV_NAME, host);
	if (ret)
		goto release;

	MVG_EMMC_SETUP(host);

	if (hw->flags & MSDC_CD_PIN_EN) {       /* not set for sdio */
		if (hw->request_cd_eirq) {
			/* msdc_eirq_cd will not be used! */
			hw->request_cd_eirq(msdc_eirq_cd, (void *)host);
		}
	}

	if (hw->request_sdio_eirq)
		/* set to combo_sdio_request_eirq() for WIFI */
		/* msdc_eirq_sdio() will be called when EIRQ */
		hw->request_sdio_eirq(msdc_eirq_sdio, (void *)host);

#ifdef CONFIG_PM
	if (hw->register_pm) {/* only for sdio */
		/* function pointer to combo_sdio_register_pm() */
		hw->register_pm(msdc_pm, (void *)host);
		if (hw->flags & MSDC_SYS_SUSPEND) {
			/* will not set for WIFI */
			ERR_MSG("MSDC_SYS_SUSPEND and register_pm both set");
		}
		/* pm not controlled by system but by client. */
		mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
	}
#endif

	platform_set_drvdata(pdev, mmc);

#ifdef CONFIG_MTK_HIBERNATION
	if (pdev->id == 1)
		register_swsusp_restore_noirq_func(ID_M_MSDC,
			msdc_drv_pm_restore_noirq, &(pdev->dev));
#endif

	/* Config card detection pin and enable interrupts */
	if (hw->flags & MSDC_CD_PIN_EN) {
		/* set for card */
		msdc_enable_cd_irq(host, 1);
	}

	ret = mmc_add_host(mmc);
	if (ret)
		goto free_irq;

	/* if (hw->flags & MSDC_CD_PIN_EN) */
	host->sd_cd_insert_work = 1;

#ifdef DEBUG_TEST_FOR_SIGNAL
	/* use EINT1 for trigger signal */
	/* need to remove gpio warning log at
	 * mediatek/kernel/include/mach/mt_gpio_core.h
	 * mediatek/platform/{project}/kernel/drivers/gpio/mt_gpio_affix.c */
	mt_set_gpio_mode(1, GPIO_MODE_00);
	mt_set_gpio_dir(1, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(1, 1);

	mt_set_gpio_out(1, 0);  /* 1-high, 0-low */
#endif

#ifdef MTK_MSDC_BRINGUP_DEBUG

	pr_debug("[%s]: msdc%d, mmc->caps=0x%x, mmc->caps2=0x%x\n",
		__func__, host->id, mmc->caps, mmc->caps2);
	msdc_dump_clock_sts();
#endif

#ifdef FPGA_PLATFORM
#if 0 /*def CONFIG_MTK_EMMC_SUPPORT*/
	pr_debug("[%s]: waiting emmc init complete\n", __func__);
	host->mmc->card_init_wait(host->mmc);
	pr_debug("[%s]: start read write compare test\n", __func__);
	multi_rw_compare(0, 0x200, 0xf, MMC_TYPE_MMC);
	pr_debug("[%s]: finish read write compare test\n", __func__);
#endif
#endif

	return 0;

free_irq:
	free_irq(irq, host);
	pr_err("[%s]: msdc%d init fail free irq!\n", __func__, host->id);
release:
	platform_set_drvdata(pdev, NULL);
	msdc_deinit_hw(host);
	pr_err("[%s]: msdc%d init fail release!\n", __func__, host->id);

	tasklet_kill(&host->card_tasklet);
#ifdef MTK_MSDC_FLUSH_BY_CLK_GATE
	if (host->mmc->caps2 & MMC_CAP2_CACHE_CTRL)
		tasklet_kill(&host->flush_cache_tasklet);
#endif

	mmc_free_host(mmc);

	return ret;
}

/* 4 device share one driver, using "drvdata" to show difference */
static int msdc_drv_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct msdc_host *host;
	struct resource *mem;

	mmc = platform_get_drvdata(pdev);
	BUG_ON(!mmc);

	host = mmc_priv(mmc);
	BUG_ON(!host);

	ERR_MSG("msdc_drv_remove");
#ifndef CONFIG_MTK_CLKMGR
	/* clock unprepare */
	if (host->clock_control)
		clk_unprepare(host->clock_control);

#endif
	platform_set_drvdata(pdev, NULL);
	mmc_remove_host(host->mmc);
	msdc_deinit_hw(host);

	tasklet_kill(&host->card_tasklet);
#ifdef MTK_MSDC_FLUSH_BY_CLK_GATE
	if ((host->hw->host_function == MSDC_EMMC) &&
	    (host->mmc->caps2 & MMC_CAP2_CACHE_CTRL))
		tasklet_kill(&host->flush_cache_tasklet);
#endif
	free_irq(host->irq, host);

	dma_free_coherent(NULL, MAX_GPD_NUM * sizeof(gpd_t),
		host->dma.gpd, host->dma.gpd_addr);
	dma_free_coherent(NULL, MAX_BD_NUM * sizeof(bd_t),
		host->dma.bd, host->dma.bd_addr);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (mem)
		release_mem_region(mem->start, mem->end - mem->start + 1);

	mmc_free_host(host->mmc);

	return 0;
}

#ifdef CONFIG_PM
static int msdc_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct msdc_host *host;
	void __iomem *base;

	if (mmc == NULL)
		return 0;

	host = mmc_priv(mmc);
	base = host->base;

	if ((host->hw->host_function == MSDC_SD) &&
	    (host->card_inserted == 1) &&
	    (g_msdc1_io == 0)) {
		/*bad sd because power is off, but sd is still inserted*/
		host->block_bad_card = 1;
	}

	if (state.event == PM_EVENT_SUSPEND) {
		if  (host->hw->flags & MSDC_SYS_SUSPEND) {
			/* will set for card */
			msdc_pm(state, (void *)host);
		} else {
			/* WIFI slot should be off when enter suspend */
			msdc_gate_clock(host, -1);
			if (host->error == -EBUSY) {
				ret = host->error;
				host->error = 0;
			}
		}
	}

	if (is_card_sdio(host) || (host->hw->flags & MSDC_SDIO_IRQ)) {
		if (host->clk_gate_count > 0) {
			host->error = 0;
			return -EBUSY;
		}
		if (host->saved_para.suspend_flag == 0) {
			host->saved_para.hz = host->mclk;
			if (host->saved_para.hz) {
				/*FIX ME: check if can be moved as
				  msdc_save_sdio_setting, similar to
				  msdc_save_emmc_setting */
				host->saved_para.suspend_flag = 1;
				/* mb(); */
				msdc_ungate_clock(host);
				msdc_save_timing_setting(host, 0, 0, 1, 0, 0);
				msdc_gate_clock(host, 0);
				if (host->error == -EBUSY) {
					ret = host->error;
					host->error = 0;
				}
			}
			ERR_MSG("msdc suspend cur_cfg=%x, save_cfg=%x,"
				" cur_hz=%d",
				MSDC_READ32(MSDC_CFG),
				host->saved_para.msdc_cfg, host->mclk);
		}
	}
	return ret;
}

static int msdc_drv_resume(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct msdc_host *host = mmc_priv(mmc);
	struct pm_message state;

	if (host->hw->flags & MSDC_SDIO_IRQ)
		pr_err("msdc msdc_drv_resume\n");
	state.event = PM_EVENT_RESUME;
	if (mmc && (host->hw->flags & MSDC_SYS_SUSPEND)) {
		/* will set for card;
		   WIFI not controller by PM */
		msdc_pm(state, (void *)host);
	}

	return 0;
}
#endif

static struct platform_driver mt_msdc_driver = {
	.probe = msdc_drv_probe,
	.remove = msdc_drv_remove,
#ifdef CONFIG_PM
	.suspend = msdc_drv_suspend,
	.resume = msdc_drv_resume,
#endif
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msdc_of_ids,
	},
};

/*--------------------------------------------------------------------------*/
/* module init/exit                                                         */
/*--------------------------------------------------------------------------*/
static int __init mt_msdc_init(void)
{
	int ret;

	ret = platform_driver_register(&mt_msdc_driver);
	if (ret) {
		pr_err(DRV_NAME ": Can't register driver");
		return ret;
	}

#if defined(CONFIG_MTK_EMMC_SUPPORT) && defined(CONFIG_PROC_FS)
	msdc_proc_emmc_create();
#endif

	pr_debug(DRV_NAME ": MediaTek MSDC Driver\n");

	msdc_debug_proc_init();

	return 0;
}

static void __exit mt_msdc_exit(void)
{
	platform_driver_unregister(&mt_msdc_driver);

#ifdef CONFIG_MTK_HIBERNATION
	unregister_swsusp_restore_noirq_func(ID_M_MSDC);
#endif
}

module_init(mt_msdc_init);
module_exit(mt_msdc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek SD/MMC Card Driver");
#ifdef CONFIG_MTK_EMMC_SUPPORT
/*EXPORT_SYMBOL(ext_csd);*/
#endif
EXPORT_SYMBOL(mtk_msdc_host);
