#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>
#include <mach/board.h>
#include <linux/seq_file.h>
#include <mach/mt_gpt.h>
#include <asm/io.h>
/* for fpga early porting */
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/sd_misc.h>
#include <linux/scatterlist.h>
#include <mach/mt_typedefs.h>
/* end for fpga early porting */
#include "mt_sd.h"
#include "dbg.h"
#ifndef FPGA_PLATFORM
#include <mach/mt_clkmgr.h>
#include <mach/upmu_common.h>
#endif
#ifdef MTK_MSDC_BRINGUP_DEBUG
#include <mach/mt_pmic_wrap.h>
#endif
#include "autok_dvfs.h"

#ifdef MTK_IO_PERFORMANCE_DEBUG
unsigned int g_mtk_mmc_perf_dbg = 0;
unsigned int g_mtk_mmc_dbg_range = 0;
unsigned int g_dbg_range_start = 0;
unsigned int g_dbg_range_end = 0;
unsigned int g_mtk_mmc_dbg_flag = 0;
unsigned int g_dbg_req_count = 0;
unsigned int g_dbg_raw_count = 0;
unsigned int g_dbg_write_count = 0;
unsigned int g_dbg_raw_count_old = 0;
unsigned int g_mtk_mmc_clear = 0;
int g_check_read_write = 0;
int g_i = 0;
unsigned long long g_req_buf[4000][30] = { {0} };
unsigned long long g_req_write_buf[4000][30] = { {0} };
unsigned long long g_req_write_count[4000] = { 0 };

unsigned long long g_mmcqd_buf[400][300] = { {0} };

char *g_time_mark[] = {
	"--start fetch request",
	"--end fetch request",
	"--start dma map this request",
	"--end dma map this request",
	"--start request",
	"--DMA start",
	"--DMA transfer done",
	"--start dma unmap request",
	"--end dma unmap request",
	"--end of request",
};

char *g_time_mark_vfs_write[] = {
	"--in vfs_write",
	"--before generic_segment_checks",
	"--after generic_segment_checks",
	"--after vfs_check_frozen",
	"--after generic_write_checks",
	"--after file_remove_suid",
	"--after file_update_time",
	"--after generic_file_direct_write",
	"--after generic_file_buffered_write",
	"--after filemap_write_and_wait_range",
	"--after invalidate_mapping_pages",
	"--after 2nd generic_file_buffered_write",
	"--before generic_write_sync",
	"--after generic_write_sync",
	"--out vfs_write"
};
#endif

/* for get transfer time with each trunk size, default not open */
#ifdef MTK_MMC_PERFORMANCE_TEST
unsigned int g_mtk_mmc_perf_test = 0;
#endif

/* for a type command, e.g. CMD53, 2 blocks */
struct cmd_profile {
	u32 max_tc;             /* Max tick count */
	u32 min_tc;
	u32 tot_tc;             /* total tick count */
	u32 tot_bytes;
	u32 count;              /* the counts of the command */
};

/* dump when total_tc and total_bytes */
struct sdio_profile {
	u32 total_tc;           /* total tick count of CMD52 and CMD53 */
	u32 total_tx_bytes;     /* total bytes of CMD53 Tx */
	u32 total_rx_bytes;     /* total bytes of CMD53 Rx */

	/*CMD52 */
	struct cmd_profile cmd52_tx;
	struct cmd_profile cmd52_rx;

	/*CMD53 in byte unit */
	struct cmd_profile cmd53_tx_byte[512];
	struct cmd_profile cmd53_rx_byte[512];

	/*CMD53 in block unit */
	struct cmd_profile cmd53_tx_blk[100];
	struct cmd_profile cmd53_rx_blk[100];
};

static char cmd_buf[256];

int sdio_cd_result = 1;

/* for driver profile */
#define TICKS_ONE_MS  (13000)
u32 gpt_enable = 0;
u32 sdio_pro_enable = 0;	/* make sure gpt is enabled */
static unsigned long long sdio_pro_time = 30;	/* no more than 30s */
static unsigned long long sdio_profiling_start;
struct sdio_profile sdio_perfomance = { 0 };

/*#define MTK_MSDC_ERROR_TUNE_DEBUG*/

#ifdef MSDC_DMA_ADDR_DEBUG
struct dma_addr msdc_latest_dma_address[MAX_BD_PER_GPD];

struct dma_addr *msdc_get_dma_address(int host_id)
{
	bd_t *bd;
	int i = 0;
	int mode = -1;
	struct msdc_host *host;
	void __iomem *base;
	if (host_id < 0 || host_id >= HOST_MAX_NUM) {
		pr_err("[%s] invalid host_id %d\n", __func__, host_id);
		return NULL;
	}

	if (!mtk_msdc_host[host_id]) {
		pr_err("[%s] msdc%d does not exist\n", __func__, host_id);
		return NULL;
	}

	host = mtk_msdc_host[host_id];
	base = host->base;
	/* spin_lock(&host->lock); */
	MSDC_GET_FIELD(MSDC_DMA_CTRL, MSDC_DMA_CTRL_MODE, mode);
	if (mode == 1) {
		pr_crit("Desc.DMA\n");
		bd = host->dma.bd;
		i = 0;
		while (i < MAX_BD_PER_GPD) {
			msdc_latest_dma_address[i].start_address =
				(u32) bd[i].ptr;
			msdc_latest_dma_address[i].size = bd[i].buflen;
			msdc_latest_dma_address[i].end = bd[i].eol;
			if (i > 0)
				msdc_latest_dma_address[i - 1].next =
					&msdc_latest_dma_address[i];

			if (bd[i].eol)
				break;
			i++;
		}
	} else if (mode == 0) {
		pr_crit("Basic DMA\n");
		msdc_latest_dma_address[0].start_address =
			MSDC_READ32(MSDC_DMA_SA);
		msdc_latest_dma_address[0].size = MSDC_READ32(MSDC_DMA_LEN);
		msdc_latest_dma_address[0].end = 1;
	}
	/* spin_unlock(&host->lock); */

	return msdc_latest_dma_address;

}
EXPORT_SYMBOL(msdc_get_dma_address);

static void msdc_init_dma_latest_address(void)
{
	struct dma_addr *ptr, *prev;
	int bdlen = MAX_BD_PER_GPD;

	memset(msdc_latest_dma_address, 0, sizeof(struct dma_addr) * bdlen);
	ptr = msdc_latest_dma_address + bdlen - 1;
	while (ptr != msdc_latest_dma_address) {
		prev = ptr - 1;
		prev->next = (void *)(msdc_latest_dma_address
			+ sizeof(struct dma_addr)
			* (ptr - msdc_latest_dma_address));
		ptr = prev;
	}

}
#endif

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static unsigned int printk_cpu_test = UINT_MAX;
struct timeval cur_tv;
struct timeval prev_tv, curr_tv;

void dbg_add_host_log(struct mmc_host *host, int type, int cmd, int arg)
{
	unsigned long long t;
	unsigned long long nanosec_rem;
	unsigned long flags;

	spin_lock_irqsave(&host->cmd_dump_lock, flags);
	t = cpu_clock(printk_cpu_test);
	nanosec_rem = do_div(t, 1000000000)/1000;
	do_gettimeofday(&cur_tv);
	host->dbg_run_host_log_dat[host->dbg_host_cnt].time_sec = t;
	host->dbg_run_host_log_dat[host->dbg_host_cnt].time_usec = nanosec_rem;
	host->dbg_run_host_log_dat[host->dbg_host_cnt].type = type;
	host->dbg_run_host_log_dat[host->dbg_host_cnt].cmd = cmd;
	host->dbg_run_host_log_dat[host->dbg_host_cnt].arg = arg;
	host->dbg_host_cnt++;
	if (host->dbg_host_cnt >= dbg_max_cnt)
		host->dbg_host_cnt = 0;
	spin_unlock_irqrestore(&host->cmd_dump_lock, flags);
}

void mmc_cmd_dump(struct mmc_host *host)
{
	int i;
	int tag = -1;
	unsigned long flags;
	pr_err("-------------------------------------------------------------------------------\n");
	spin_lock_irqsave(&host->cmd_dump_lock, flags);
	for (i = host->dbg_host_cnt; i < dbg_max_cnt; i++) {
		if (host->dbg_run_host_log_dat[i].cmd == 44
			&& (host->dbg_run_host_log_dat[i].type == 0)) {
			tag = (host->dbg_run_host_log_dat[i].arg >> 16) & 0xf;
			pr_err("%d [%5llu.%06llu]%2d %2d 0x%08x tag=%d type=%s\n", i,
				host->dbg_run_host_log_dat[i].time_sec,
				host->dbg_run_host_log_dat[i].time_usec,
				host->dbg_run_host_log_dat[i].type,
				host->dbg_run_host_log_dat[i].cmd,
				host->dbg_run_host_log_dat[i].arg, tag,
				((host->dbg_run_host_log_dat[i].arg >> 30) & 0x1) ? "read" : "write");
		} else if ((host->dbg_run_host_log_dat[i].cmd == 46
			|| host->dbg_run_host_log_dat[i].cmd == 47)
			&& !host->dbg_run_host_log_dat[i].type) {
			tag = (host->dbg_run_host_log_dat[i].arg >> 16) & 0xf;
			pr_err("%d [%5llu.%06llu]%2d %2d 0x%08x tag=%d\n", i,
				host->dbg_run_host_log_dat[i].time_sec,
				host->dbg_run_host_log_dat[i].time_usec,
				host->dbg_run_host_log_dat[i].type,
				host->dbg_run_host_log_dat[i].cmd,
				host->dbg_run_host_log_dat[i].arg, tag);
		} else
			pr_err("%d [%5llu.%06llu]%2d %2d 0x%08x\n", i,
			host->dbg_run_host_log_dat[i].time_sec,
			host->dbg_run_host_log_dat[i].time_usec,
			host->dbg_run_host_log_dat[i].type,
			host->dbg_run_host_log_dat[i].cmd,
			host->dbg_run_host_log_dat[i].arg);
	}

	for (i = 0; i < host->dbg_host_cnt; i++) {
		if (host->dbg_run_host_log_dat[i].cmd == 44
			&& !host->dbg_run_host_log_dat[i].type) {
			tag = (host->dbg_run_host_log_dat[i].arg >> 16) & 0xf;
			pr_err("%d [%5llu.%06llu]%2d %2d 0x%08x tag=%d type=%s\n", i,
				host->dbg_run_host_log_dat[i].time_sec,
				host->dbg_run_host_log_dat[i].time_usec,
				host->dbg_run_host_log_dat[i].type,
				host->dbg_run_host_log_dat[i].cmd,
				host->dbg_run_host_log_dat[i].arg, tag,
				((host->dbg_run_host_log_dat[i].arg >> 30) & 0x1) ? "read" : "write");
		} else if ((host->dbg_run_host_log_dat[i].cmd == 46
			|| host->dbg_run_host_log_dat[i].cmd == 47)
			&& !host->dbg_run_host_log_dat[i].type) {
			tag = (host->dbg_run_host_log_dat[i].arg >> 16) & 0xf;
			pr_err("%d [%5llu.%06llu]%2d %2d 0x%08x tag=%d\n", i,
				host->dbg_run_host_log_dat[i].time_sec,
				host->dbg_run_host_log_dat[i].time_usec,
				host->dbg_run_host_log_dat[i].type,
				host->dbg_run_host_log_dat[i].cmd,
				host->dbg_run_host_log_dat[i].arg, tag);
		} else
			pr_err("%d [%5llu.%06llu]%2d %2d 0x%08x\n", i,
			host->dbg_run_host_log_dat[i].time_sec,
			host->dbg_run_host_log_dat[i].time_usec,
			host->dbg_run_host_log_dat[i].type,
			host->dbg_run_host_log_dat[i].cmd,
			host->dbg_run_host_log_dat[i].arg);
	}
	spin_unlock_irqrestore(&host->cmd_dump_lock, flags);
}
#endif

void msdc_cmdq_status_print(struct msdc_host *host)
{
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	struct mmc_host *mmc = host->mmc;
	if (!mmc)
		return;

	pr_err("===============================\n");
	pr_err("cmdq support : %s\n",
		mmc->card->ext_csd.cmdq_support ? "yes":"no");
	pr_err("cmdq mode    : %s\n",
		mmc->card->ext_csd.cmdq_mode_en ? "enable" : "disable");
	pr_err("cmdq depth   : %d\n",
		mmc->card->ext_csd.cmdq_depth);
	pr_err("===============================\n");
	pr_err("task_id_index: %08lx\n",
		mmc->task_id_index);
	pr_err("cq_cmd       : %d\n",
		atomic_read(&mmc->cq_cmd));
	pr_err("cq_wait_rdy  : %d\n",
		atomic_read(&mmc->cq_wait_rdy));
	pr_err("cq_tuning_now: %d\n",
		atomic_read(&mmc->cq_tuning_now));

#else
	pr_err("driver not supported\n");
#endif
}

void msdc_set_host_mode_speed(int id, int spd_mode)
{
	switch (spd_mode) {
	case CAPS_SPEED_NULL:
		break;
	case SDHC_HIGHSPEED:
		msdc_host_mode[id] |=
			MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;
		msdc_host_mode[id] &=
			(~MMC_CAP_UHS_SDR12) & (~MMC_CAP_UHS_SDR25) &
			(~MMC_CAP_UHS_SDR50) & (~MMC_CAP_UHS_DDR50) &
			(~MMC_CAP_1_8V_DDR) & (~MMC_CAP_UHS_SDR104);
#ifdef CONFIG_EMMC_50_FEATURE
		msdc_host_mode2[id] &=
			(~MMC_CAP2_HS200_1_8V_SDR) &
			(~MMC_CAP2_HS400_1_8V_DDR);
#else
		msdc_host_mode2[id] &= (~MMC_CAP2_HS200_1_8V_SDR);
#endif
		pr_err("[SD_Debug]host will support Highspeed\n");
		break;
	case UHS_SDR12:
		msdc_host_mode[id] |= MMC_CAP_UHS_SDR12;
		msdc_host_mode[id] &=
			(~MMC_CAP_UHS_SDR25) & (~MMC_CAP_UHS_SDR50) &
			(~MMC_CAP_UHS_DDR50) & (~MMC_CAP_1_8V_DDR) &
			(~MMC_CAP_UHS_SDR104);
#ifdef CONFIG_EMMC_50_FEATURE
		msdc_host_mode2[id] &=
			(~MMC_CAP2_HS200_1_8V_SDR) &
			(~MMC_CAP2_HS400_1_8V_DDR);
#else
		msdc_host_mode2[id] &= (~MMC_CAP2_HS200_1_8V_SDR);
#endif
		pr_err("[SD_Debug]host will support UHS-SDR12\n");
		break;
	case UHS_SDR25:
		msdc_host_mode[id] |=
			MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25;
		msdc_host_mode[id] &=
			(~MMC_CAP_UHS_SDR50) & (~MMC_CAP_UHS_DDR50) &
			(~MMC_CAP_1_8V_DDR) & (~MMC_CAP_UHS_SDR104);
#ifdef CONFIG_EMMC_50_FEATURE
		msdc_host_mode2[id] &=
			(~MMC_CAP2_HS200_1_8V_SDR) &
			(~MMC_CAP2_HS400_1_8V_DDR);
#else
		msdc_host_mode2[id] &= (~MMC_CAP2_HS200_1_8V_SDR);
#endif
		pr_err("[SD_Debug]host will support UHS-SDR25\n");
		break;
	case UHS_SDR50:
		msdc_host_mode[id] |=
			MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
			MMC_CAP_UHS_SDR50;
		msdc_host_mode[id] &=
			(~MMC_CAP_UHS_DDR50) & (~MMC_CAP_1_8V_DDR) &
			(~MMC_CAP_UHS_SDR104);
#ifdef CONFIG_EMMC_50_FEATURE
		msdc_host_mode2[id] &=
			(~MMC_CAP2_HS200_1_8V_SDR) &
			(~MMC_CAP2_HS400_1_8V_DDR);
#else
		msdc_host_mode2[id] &= (~MMC_CAP2_HS200_1_8V_SDR);
#endif
		pr_err("[SD_Debug]host will support UHS-SDR50\n");
		break;
	case UHS_SDR104:
		msdc_host_mode[id] |=
			MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
			MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50 |
			MMC_CAP_1_8V_DDR | MMC_CAP_UHS_SDR104;
		msdc_host_mode2[id] |= MMC_CAP2_HS200_1_8V_SDR;
#ifdef CONFIG_EMMC_50_FEATURE
		msdc_host_mode2[id] &= (~MMC_CAP2_HS400_1_8V_DDR);
#endif
		pr_err("[SD_Debug]host will support UHS-SDR104\n");
		break;
	case UHS_DDR50:
		msdc_host_mode[id] |=
			MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
			MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR;

		msdc_host_mode[id] &=
			(~MMC_CAP_UHS_SDR50) & (~MMC_CAP_UHS_SDR104);
#ifdef CONFIG_EMMC_50_FEATURE
		msdc_host_mode2[id] &=
			(~MMC_CAP2_HS200_1_8V_SDR) &
			(~MMC_CAP2_HS400_1_8V_DDR);
#else
		msdc_host_mode2[id] &= (~MMC_CAP2_HS200_1_8V_SDR);
#endif

		pr_err("[SD_Debug]host will support UHS-DDR50\n");
		break;
#ifdef CONFIG_EMMC_50_FEATURE
	case EMMC_HS400:
		msdc_host_mode[id] |=
			MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
			MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_DDR50 |
			MMC_CAP_1_8V_DDR | MMC_CAP_UHS_SDR104;
		msdc_host_mode2[id] |=
			MMC_CAP2_HS200_1_8V_SDR |
			MMC_CAP2_HS400_1_8V_DDR;
		pr_err("[SD_Debug]host will support EMMC_HS400\n");
		break;
#endif
	default:
		pr_err("[SD_Debug]invalid sd30_mode:%d\n",
			spd_mode);
		break;
	}
}

void msdc_set_host_mode_driver_type(int id, int driver_type)
{
	switch (driver_type) {
	case DRIVER_TYPE_A:
		msdc_host_mode[id] |= MMC_CAP_DRIVER_TYPE_A;
		msdc_host_mode[id] &=
			(~MMC_CAP_DRIVER_TYPE_C) & (~MMC_CAP_DRIVER_TYPE_D);
		pr_debug("[SD_Debug]host will support DRIVING TYPE A\n");
		break;
	case DRIVER_TYPE_B:
		msdc_host_mode[id] &= (~MMC_CAP_DRIVER_TYPE_A) &
			(~MMC_CAP_DRIVER_TYPE_C) & (~MMC_CAP_DRIVER_TYPE_D);
		pr_debug("[SD_Debug]host will support DRIVING TYPE B\n");
		break;
	case DRIVER_TYPE_C:
		msdc_host_mode[id] |= MMC_CAP_DRIVER_TYPE_C;
		msdc_host_mode[id] &=
			(~MMC_CAP_DRIVER_TYPE_A) & (~MMC_CAP_DRIVER_TYPE_D);
		pr_debug("[SD_Debug]host will support DRIVING TYPE C\n");
		break;
	case DRIVER_TYPE_D:
		msdc_host_mode[id] |= MMC_CAP_DRIVER_TYPE_D;
		msdc_host_mode[id] &=
			(~MMC_CAP_DRIVER_TYPE_A) & (~MMC_CAP_DRIVER_TYPE_C);
		pr_debug("[SD_Debug]host will support DRIVING TYPE D\n");
		break;
	default:
		pr_debug("[SD_Debug]invalid sd30_drive:%d\n",
			driver_type);
		break;
	}
}

static void msdc_set_field(void __iomem *address, unsigned int start_bit,
	unsigned int len, unsigned int value)
{
	unsigned long field;
	if (start_bit > 31 || start_bit < 0 || len > 32 || len <= 0) {
		pr_err("[SD_Debug]invalid reg field range or length\n");
	} else {
		field = ((1 << len) - 1) << start_bit;
		value &= (1 << len) - 1;
		pr_err("[SD_Debug]Original:0x%p (0x%x)\n",
			address, MSDC_READ32(address));
		MSDC_SET_FIELD(address, field, value);
		pr_err("[SD_Debug]Modified:0x%p (0x%x)\n",
			address, MSDC_READ32(address));
	}
}

static void msdc_get_field(void __iomem *address, unsigned int start_bit,
	unsigned int len, unsigned int value)
{
	unsigned long field;
	if (start_bit > 31 || start_bit < 0 || len > 32 || len <= 0) {
		pr_err("[SD_Debug]invalid reg field range or length\n");
	} else {
		field = ((1 << len) - 1) << start_bit;
		MSDC_GET_FIELD(address, field, value);
		pr_err("[SD_Debug]Reg:0x%p start_bit(%d)len(%d)(0x%x)\n",
			address, start_bit, len, value);
	}
}

static void msdc_init_gpt(void)
{
#if 0
	GPT_CONFIG config;

	config.num = GPT6;
	config.mode = GPT_FREE_RUN;
	config.clkSrc = GPT_CLK_SRC_SYS;
	config.clkDiv = GPT_CLK_DIV_1;	/* 13MHz GPT6 */

	if (GPT_Config(config) == FALSE)
		return;

	GPT_Start(GPT6);
#endif
}

u32 msdc_time_calc(u32 old_L32, u32 old_H32, u32 new_L32, u32 new_H32)
{
	u32 ret = 0;

	if (new_H32 == old_H32) {
		ret = new_L32 - old_L32;
	} else if (new_H32 == (old_H32 + 1)) {
		if (new_L32 > old_L32) {
			pr_debug("msdc old_L<0x%x> new_L<0x%x>\n",
				old_L32, new_L32);
		}
		ret = (0xffffffff - old_L32);
		ret += new_L32;
	} else {
		pr_debug("msdc old_H<0x%x> new_H<0x%x>\n",
			old_H32, new_H32);
	}

	return ret;
}

void msdc_sdio_profile(struct sdio_profile *result)
{
	struct cmd_profile *cmd;
	u32 i;

	pr_err("sdio === performance dump ===\n");
	pr_err("sdio === total execute tick<%d> time<%dms> Tx<%dB> Rx<%dB>\n",
		result->total_tc, result->total_tc / TICKS_ONE_MS,
		result->total_tx_bytes, result->total_rx_bytes);

	/* CMD52 Dump */
	cmd = &result->cmd52_rx;
	pr_err("sdio === CMD52 Rx <%d>times tick<%d> Max<%d> Min<%d> Aver<%d>\n",
		cmd->count, cmd->tot_tc,
		cmd->max_tc, cmd->min_tc, cmd->tot_tc / cmd->count);
	cmd = &result->cmd52_tx;
	pr_err("sdio === CMD52 Tx <%d>times tick<%d> Max<%d> Min<%d> Aver<%d>\n",
		cmd->count, cmd->tot_tc,
		cmd->max_tc, cmd->min_tc, cmd->tot_tc / cmd->count);

	/* CMD53 Rx bytes + block mode */
	for (i = 0; i < 512; i++) {
		cmd = &result->cmd53_rx_byte[i];
		if (cmd->count == 0)
			continue;
		pr_err("sdio<%6d><%3dB>_Rx_<%9d><%9d><%6d><%6d>_<%9dB><%2dM>\n",
			cmd->count, i, cmd->tot_tc, cmd->max_tc, cmd->min_tc,
			cmd->tot_tc / cmd->count, cmd->tot_bytes,
			(cmd->tot_bytes / 10) * 13 / (cmd->tot_tc / 10));
	}
	for (i = 0; i < 100; i++) {
		cmd = &result->cmd53_rx_blk[i];
		if (cmd->count == 0)
			continue;
		pr_err("sdio<%6d><%3d>B_Rx_<%9d><%9d><%6d><%6d>_<%9dB><%2dM>\n",
			cmd->count, i, cmd->tot_tc, cmd->max_tc, cmd->min_tc,
			cmd->tot_tc / cmd->count, cmd->tot_bytes,
			(cmd->tot_bytes / 10) * 13 / (cmd->tot_tc / 10));
	}

	/* CMD53 Tx bytes + block mode */
	for (i = 0; i < 512; i++) {
		cmd = &result->cmd53_tx_byte[i];
		if (cmd->count == 0)
			continue;
		pr_err("sdio<%6d><%3dB>_Tx_<%9d><%9d><%6d><%6d>_<%9dB><%2dM>\n",
			 cmd->count, i, cmd->tot_tc, cmd->max_tc, cmd->min_tc,
			 cmd->tot_tc / cmd->count, cmd->tot_bytes,
			 (cmd->tot_bytes / 10) * 13 / (cmd->tot_tc / 10));
	}
	for (i = 0; i < 100; i++) {
		cmd = &result->cmd53_tx_blk[i];
		if (cmd->count == 0)
			continue;
		pr_err("sdio<%6d><%3d>B_Tx_<%9d><%9d><%6d><%6d>_<%9dB><%2dM>\n",
			 cmd->count, i, cmd->tot_tc, cmd->max_tc, cmd->min_tc,
			 cmd->tot_tc / cmd->count, cmd->tot_bytes,
			 (cmd->tot_bytes / 10) * 13 / (cmd->tot_tc / 10));
	}

	pr_err("sdio === performance dump done ===\n");
}

/* ========= sdio command table =========== */
void msdc_performance(u32 opcode, u32 sizes, u32 bRx, u32 ticks)
{
	struct sdio_profile *result = &sdio_perfomance;
	struct cmd_profile *cmd;
	u32 block;
	long long endtime;

	if (sdio_pro_enable == 0)
		return;

	if (opcode == 52) {
		cmd = bRx ? &result->cmd52_rx : &result->cmd52_tx;
	} else if (opcode == 53) {
		if (sizes < 512) {
			cmd = bRx ? &result->cmd53_rx_byte[sizes] :
				&result->cmd53_tx_byte[sizes];
		} else {
			block = sizes / 512;
			if (block >= 99) {
				pr_err("cmd53 error blocks\n");
				while (1)
					;
			}
			cmd = bRx ? &result->cmd53_rx_blk[block] :
				&result->cmd53_tx_blk[block];
		}
	} else {
		return;
	}

	/* update the members */
	if (ticks > cmd->max_tc)
		cmd->max_tc = ticks;

	if (cmd->min_tc == 0 || ticks < cmd->min_tc)
		cmd->min_tc = ticks;

	cmd->tot_tc += ticks;
	cmd->tot_bytes += sizes;
	cmd->count++;

	if (bRx)
		result->total_rx_bytes += sizes;
	else
		result->total_tx_bytes += sizes;

	result->total_tc += ticks;
#if 0
	/* dump when total_tc > 30s */
	if (result->total_tc >= sdio_pro_time * TICKS_ONE_MS * 1000) {
		msdc_sdio_profile(result);
		memset(result, 0, sizeof(struct sdio_profile));
	}
#endif

	endtime = sched_clock();
	if ((endtime - sdio_profiling_start) >=
		sdio_pro_time * 1000000000) {
		msdc_sdio_profile(result);
		memset(result, 0, sizeof(struct sdio_profile));
		sdio_profiling_start = endtime;
	}


}

#define COMPARE_ADDRESS_MMC             0x402000
#define COMPARE_ADDRESS_SD              0x2000
#define COMPARE_ADDRESS_SDIO            0x0
#define COMPARE_ADDRESS_SD_COMBO        0x2000

#define MSDC_MULTI_BUF_LEN  (4*4*1024) /*16KB write/read/compare*/

static DEFINE_MUTEX(sd_lock);
static DEFINE_MUTEX(emmc_lock);

u8 read_write_state = 0;	/* 0:stop, 1:read, 2:write */

static u8 wData_emmc[16] = {
	0x67, 0x45, 0x23, 0x01,
	0xef, 0xcd, 0xab, 0x89,
	0xce, 0x8a, 0x46, 0x02,
	0xde, 0x9b, 0x57, 0x13
};

static u8 wData_sd[200] = {
	0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,    /*worst1*/
	0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,
	0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,

	0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,    /*worst2*/
	0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,
	0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00,

	0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff,    /*worst3*/
	0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00,
	0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,

	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,    /*worst4*/
	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,

	0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55,    /*worst5*/
	0x80, 0x80, 0x80, 0x7f, 0x80, 0x80, 0x80, 0x7f,
	0x7f, 0x7f, 0x80, 0x7f, 0x7f, 0x7f, 0x40, 0x40,
	0x04, 0xfb, 0x04, 0x04, 0x04, 0xfb, 0xfb, 0xfb,
	0x04, 0xfb, 0xfb, 0xfb, 0x02, 0x02, 0x02, 0xfd,
	0x02, 0x02, 0x02, 0xfd, 0xfd, 0xfd, 0x02, 0xfd,
	0xfd, 0xfd, 0x01, 0x01, 0x01, 0xfe, 0x01, 0x01,
	0x01, 0xfe, 0xfe, 0xfe, 0x01, 0xfe, 0xfe, 0xfe,
	0x80, 0x80, 0x80, 0x7f, 0x80, 0x80, 0x80, 0x7f,
	0x7f, 0x7f, 0x80, 0x7f, 0x7f, 0x7f, 0x40, 0x40,
	0x40, 0x40, 0x80, 0x7f, 0x7f, 0x7f, 0x40, 0x40,
	0x20, 0xdf, 0x20, 0x20, 0x20, 0xdf, 0xdf, 0xdf,
	0x10, 0x10, 0x10, 0xef, 0xef, 0x10, 0xef, 0xef,
};

/*
  * @read, bit0: 1:read/0:write; bit1: 0:compare/1:not compare
*/
static int multi_rw_compare_core(int host_num, int read, uint address,
	uint type)
{
	struct scatterlist msdc_sg;
	struct mmc_data msdc_data;
	struct mmc_command msdc_cmd;
	struct mmc_command msdc_stop;

	u32 *multi_rwbuf = NULL;
	u8 *wPtr = NULL, *rPtr = NULL;

	struct mmc_request msdc_mrq;
	struct msdc_host *host_ctl;
	struct mmc_host *mmc;
	int result = 0, forIndex = 0;

	u8 wData_len;
	u8 *wData;

	if (type == MMC_TYPE_MMC) {
		wData = wData_emmc;
		wData_len = 16;
	} else if (type == MMC_TYPE_SD) {
		wData = wData_sd;
		wData_len = 200;
	} else
		return -1;

	if (host_num >= HOST_MAX_NUM || host_num < 0) {
		pr_err("[%s]invalid host id: %d\n", __func__, host_num);
		return -1;
	}

	/*allock memory for test buf*/
	multi_rwbuf = kzalloc((MSDC_MULTI_BUF_LEN), GFP_KERNEL);
	if (multi_rwbuf == NULL) {
		pr_err("[%s]alloc memory error!!! L<%d>\n", __func__, __LINE__);
		result = -1;
		goto free;
	}
	rPtr = wPtr = (u8 *)multi_rwbuf;

	host_ctl = mtk_msdc_host[host_num];
	mmc = host_ctl->mmc;
	if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card) {
		pr_err(" No card initialized in host[%d]\n", host_num);
		result = -1;
		goto free;
	}

	if (!is_card_present(host_ctl)) {
		pr_err("  [%s]: card is removed!\n", __func__);
		result = -1;
		goto free;
	}

	mmc_claim_host(mmc);

#ifdef CONFIG_MTK_EMMC_SUPPORT
	if (!g_ett_tune && (host_ctl->hw->host_function == MSDC_EMMC))
		msdc_switch_part(host_ctl, 0);
#endif

	memset(&msdc_data, 0, sizeof(struct mmc_data));
	memset(&msdc_mrq, 0, sizeof(struct mmc_request));
	memset(&msdc_cmd, 0, sizeof(struct mmc_command));
	memset(&msdc_stop, 0, sizeof(struct mmc_command));

	msdc_mrq.cmd = &msdc_cmd;
	msdc_mrq.data = &msdc_data;
	msdc_data.blocks = MSDC_MULTI_BUF_LEN / 512;

	if (read) {
		/* init read command */
		msdc_data.flags = MMC_DATA_READ;
		msdc_cmd.opcode = MMC_READ_MULTIPLE_BLOCK;
	} else {
		/* init write command */
		msdc_data.flags = MMC_DATA_WRITE;
		msdc_cmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
		/* init write buffer */
		for (forIndex = 0; forIndex < MSDC_MULTI_BUF_LEN; forIndex++)
			*(wPtr + forIndex) = wData[forIndex % wData_len];
	}

	msdc_cmd.arg = address;

	BUG_ON(!host_ctl->mmc->card);

	msdc_stop.opcode = MMC_STOP_TRANSMISSION;
	msdc_stop.arg = 0;
	msdc_stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	msdc_data.stop = &msdc_stop;

	if (!mmc_card_blockaddr(host_ctl->mmc->card)) {
		/* pr_err("this device use byte address!!\n"); */
		msdc_cmd.arg <<= 9;
	}
	msdc_cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	msdc_data.blksz = 512;
	msdc_data.sg = &msdc_sg;
	msdc_data.sg_len = 1;

	sg_init_one(&msdc_sg, multi_rwbuf, MSDC_MULTI_BUF_LEN);

	mmc_set_data_timeout(&msdc_data, mmc->card);
	mmc_wait_for_req(mmc, &msdc_mrq);
	/* compare */
	if ((!read || g_ett_tune) && (type == MMC_TYPE_MMC))
		goto skip_check;
	if ((!read) && (type == MMC_TYPE_SD))
		goto skip_check;
	for (forIndex = 0; forIndex < MSDC_MULTI_BUF_LEN; forIndex++) {
		/*
		pr_err("index[%d]\tW_buffer[0x%x]\tR_buffer[0x%x]\t\n",
			forIndex, wData[forIndex%wData_len], rPtr[forIndex]);
		*/
		if (rPtr[forIndex] != wData[forIndex % wData_len]) {
			pr_err("index[%d]\tW_buffer[0x%x]\tR_buffer[0x%x]\tfailed\n",
				forIndex, wData[forIndex % wData_len],
				rPtr[forIndex]);
			result = -1;
		}
	}

skip_check:
	mmc_release_host(host_ctl->mmc);

	if (msdc_cmd.error)
		result = msdc_cmd.error;

	if (msdc_data.error)
		result = msdc_data.error;
	else
		result = 0;

free:
	kfree(multi_rwbuf);

	return result;
}

int multi_rw_compare(int host_num, uint address, int count, uint type)
{
	int i = 0, j = 0;
	int error = 0;
	struct mutex *rw_mutex;

	if (type == MMC_TYPE_SD)
		rw_mutex = &sd_lock;
	else if (type == MMC_TYPE_MMC)
		rw_mutex = &emmc_lock;
	else
		return 0;

	for (i = 0; i < count; i++) {
		/* pr_err("== cpu[%d] pid[%d]: start %d time compare ==\n",
			task_cpu(current), current->pid, i); */

		mutex_lock(rw_mutex);
		/* write */
		error = multi_rw_compare_core(host_num, 0, address, type);
		if (error) {
			pr_err("[%s]: failed to write data, error=%d\n",
				__func__, error);
			mutex_unlock(rw_mutex);
			break;
		}

		/* read */
		for (j = 0; j < 1; j++) {
			error = multi_rw_compare_core(host_num, 1, address,
				type);
			if (error) {
				pr_err("[%s]: failed to read data, error=%d\n",
					__func__, error);
				break;
			}
		}
		pr_err("== cpu[%d] pid[%d]: %s %d time compare ==\n",
			task_cpu(current), current->pid,
			(error ? "FAILED" : "FINISH"), i);

		mutex_unlock(rw_mutex);
	}

	if (i == count)
		pr_err("pid[%d]: success to compare data for %d times\n",
			current->pid, count);

	return error;
}

#define MAX_THREAD_NUM_FOR_SMP 20

/* make the test can run on 4GB card */
static uint smp_address_on_sd[MAX_THREAD_NUM_FOR_SMP] = {
	0x2000,
	0x80000,
	0x100000,
	0x180000,
	0x200000,		/* 1GB */
	0x202000,
	0x280000,
	0x300000,
	0x380000,
	0x400000,		/* 2GB */
	0x402000,
	0x480000,
	0x500000,
	0x580000,
	0x600000,
	0x602000,		/* 3GB */
	0x660000,		/* real total size of 4GB sd card is < 4GB */
	0x680000,
	0x6a0000,
	0x6b0000,
};

/* cause the system run on the emmc storage,
 * so do not to access the first 2GB region */
static uint smp_address_on_mmc[MAX_THREAD_NUM_FOR_SMP] = {
	0x402000,
	0x410000,
	0x520000,
	0x530000,
	0x640000,
	0x452000,
	0x460000,
	0x470000,
	0x480000,
	0x490000,
	0x4a2000,
	0x4b0000,
	0x5c0000,
	0x5d0000,
	0x6e0000,
	0x602000,
	0x660000,		/* real total size of 4GB sd card is < 4GB */
	0x680000,
	0x6a0000,
	0x6b0000,
};

struct write_read_data {
	int host_id;		/* target host you want to do SMP test on. */
	uint start_address;	/* Address of memcard you want to write/read */
	int count;		/* times you want to do read after write */
};

static struct write_read_data wr_data[HOST_MAX_NUM][MAX_THREAD_NUM_FOR_SMP];
/*
 * 2012-03-25
 * the SMP thread function
 * do read after write the memory card, and bit by bit comparison
 */
static int write_read_thread(void *ptr)
{
	struct write_read_data *data = (struct write_read_data *)ptr;

	if (1 == data->host_id) {
		pr_err("sd thread\n");
		multi_rw_compare(data->host_id, data->start_address,
			data->count, MMC_TYPE_SD);
	} else if (0 == data->host_id) {
		pr_err("emmc thread\n");
		multi_rw_compare(data->host_id, data->start_address,
			data->count, MMC_TYPE_MMC);
	}
	return 0;
}

/*
 * 2012-03-25
 * function:         do SMP test on all MSDC hosts
 * thread_num:       number of thread to be triggerred on this host.
 * count:            times you want to do read after writein each thread.
 * multi_address:    whether do read/write the same/different address of
 *                       the memory card in each thread.
 */
static int smp_test_on_hosts(int thread_num, int host_id, int count,
	int multi_address)
{
	int i = 0, j = 0;
	int id_start, id_end;
	int ret = 0;
	uint start_address, type;
	char thread_name[128];
	struct msdc_host *host_ctl;

	pr_err("=======================[%s] start ===========================\n\n",
		__func__);
	pr_err(" each host run %d thread, each thread run %d RW comparison\n",
		thread_num, count);
	if (thread_num > MAX_THREAD_NUM_FOR_SMP) {
		pr_err(" too much thread for SMP test, thread_num=%d\n",
			thread_num);
		ret = -1;
		goto out;
	}
	if (host_id > HOST_MAX_NUM || host_id < 0) {
		pr_err(" Invalid host id %d\n", host_id);
		ret = -1;
		goto out;
	}
	if (host_id == HOST_MAX_NUM) {
		id_start = 0;
		id_end = HOST_MAX_NUM;
	} else {
		id_start = host_id;
		id_end = host_id+1;
	}


	for (i = id_start; i < id_end; i++) {
		host_ctl = mtk_msdc_host[i];
		if (!host_ctl || !host_ctl->mmc || !host_ctl->mmc->card) {
			pr_err(" MSDC[%d], no card is initialized\n", i);
			continue;
		}
		type = host_ctl->mmc->card->type;
		if (type == MMC_TYPE_MMC) {
			if (!multi_address)
				start_address = COMPARE_ADDRESS_MMC;
			else
				start_address = smp_address_on_mmc[i];
		} else if (type == MMC_TYPE_SD) {
			if (!multi_address)
				start_address = COMPARE_ADDRESS_MMC;
			else
				start_address = smp_address_on_sd[i];
		} else if (type == MMC_TYPE_SDIO) {
			pr_err(" MSDC[%d], SDIO:\n", i);
			pr_err("   please manually run wifi application instead of write/read SDIO card\n");
			continue;
		} else {
			pr_err(" MSDC[%d], unkwonn card type\n ", i);
			continue;
		}
		for (j = 0; j < thread_num; j++) {
			wr_data[i][j].host_id = i;
			wr_data[i][j].count = count;
			wr_data[i][j].start_address = start_address;
			sprintf(thread_name, "msdc_H%d_T%d", i, j);
			kthread_run(write_read_thread, &wr_data[i][j],
				thread_name);
			pr_err("	start thread: %s, at address: 0x%x\n",
				 thread_name, wr_data[i][j].start_address);
		}
	}
 out:
	pr_err("=======================[%s] end ===========================\n\n",
		__func__);
	return ret;
}

static int msdc_help_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, "\n===============[msdc_help]================\n");

	seq_printf(m, "\n   LOG control:        echo %x [host_id] [debug_zone] > msdc_debug\n",
		SD_TOOL_ZONE);
	seq_printf(m, "          [debug_zone] DMA:0x%x, CMD:0x%x, RSP:0x%x, INT:0x%x, CFG:0x%x, FUC:0x%x,\n",
		DBG_EVT_DMA, DBG_EVT_CMD, DBG_EVT_RSP, DBG_EVT_INT, DBG_EVT_CFG,
		DBG_EVT_FUC);
	seq_printf(m, "                        OPS:0x%x, FIO:0x%x, WRN:0x%x, PWR:0x%x, CLK:0x%x, RW:0x%x, NRW:0x%x, CHE:0x%x\n",
		DBG_EVT_OPS, DBG_EVT_FIO, DBG_EVT_WRN, DBG_EVT_PWR, DBG_EVT_CLK,
		DBG_EVT_RW, DBG_EVT_NRW, DBG_EVT_CHE);
	seq_puts(m, "\n   DMA mode:\n");
	seq_printf(m, "   *set DMA mode:   echo %x 0 [host_id] [dma_mode] [dma_size] > msdc_debug\n",
		SD_TOOL_DMA_SIZE);
	seq_printf(m, "   *get DMA mode:   echo %x 1 [host_id] > msdc_debug\n",
		SD_TOOL_DMA_SIZE);
	seq_puts(m, "        [dma_mode]   0:PIO, 1:DMA, 2:SIZE_DEP\n");
	seq_puts(m, "        [dma_size]   valid for SIZE_DEP mode, the min size can trigger the DMA mode\n");
	seq_printf(m, "\n   SDIO profile:  echo %x [enable] [time] > msdc_debug\n",
		SD_TOOL_SDIO_PROFILE);
	seq_puts(m, "\n   CLOCK control:\n");
	seq_printf(m, "   *set clk src:       echo %x 0 [host_id] [clk_src] > msdc_debug\n",
		SD_TOOL_CLK_SRC_SELECT);
	seq_printf(m, "   *get clk src:       echo %x 1 [host_id] > msdc_debug\n",
		SD_TOOL_CLK_SRC_SELECT);
	seq_puts(m, "      [clk_src]       msdc0: 0:26M, 1:800M, 2:400M, 3:200M, 4:182M, 5:136M, 6:156M, 7:48M, 8:91M\n");
	seq_puts(m, "	  [clk_src]  msdc1/2/3: 0:26M, 1:208M, 2:200M, 3:182M, 4:182M, 5:136M, 6:156M, 7:48M, 8:91M\n");
	seq_puts(m, "\n   REGISTER control:\n");
	seq_printf(m, "        write register:    echo %x 0 [host_id] [register_offset] [value] > msdc_debug\n",
		SD_TOOL_REG_ACCESS);
	seq_printf(m, "        read register:     echo %x 1 [host_id] [register_offset] > msdc_debug\n",
		SD_TOOL_REG_ACCESS);
	seq_printf(m, "        write mask:        echo %x 2 [host_id] [register_offset] [start_bit] [len] [value] > msdc_debug\n",
		SD_TOOL_REG_ACCESS);
	seq_printf(m, "        read mask:         echo %x 3 [host_id] [register_offset] [start_bit] [len] > msdc_debug\n",
		SD_TOOL_REG_ACCESS);
	seq_printf(m, "     dump all:          echo %x 4 [host_id] > msdc_debug\n",
		SD_TOOL_REG_ACCESS);
	seq_puts(m, "\n   DRVING control:\n");
	seq_printf(m, "        set driving:       echo %x [host_id] [clk_drv] [cmd_drv] [dat_drv] [rst_drv] [ds_drv] [voltage] > msdc_debug\n",
		SD_TOOL_SET_DRIVING);
	seq_puts(m, "            [voltage]        0x18:18v, 0x33:33v\n");
	seq_puts(m, "\n   DESENSE control:\n");
	seq_printf(m, "          write register:    echo %x 0 [value] > msdc_debug\n",
		SD_TOOL_DESENSE);
	seq_printf(m, "          read register:     echo %x 1 > msdc_debug\n",
		SD_TOOL_DESENSE);
	seq_printf(m, "          write mask:        echo %x 2 [start_bit] [len] [value] > msdc_debug\n",
		SD_TOOL_DESENSE);
	seq_printf(m, "          read mask:         echo %x 3 [start_bit] [len] > msdc_debug\n",
		SD_TOOL_DESENSE);
	seq_printf(m, "\n   RW_COMPARE test:       echo %x [host_id] [compare_count] > msdc_debug\n",
		RW_BIT_BY_BIT_COMPARE);
	seq_puts(m, "          [compare_count]    how many time you want to \"write=>read=>compare\"\n");
	seq_printf(m, "\n   SMP_ON_ONE_HOST test:  echo %x [host_id] [thread_num] [compare_count] [multi_address] > msdc_debug\n",
		SMP_TEST_ON_ONE_HOST);
	seq_puts(m, "          [thread_num]       how many R/W comparision thread you want to run at host_id\n");
	seq_puts(m, "          [compare_count]    how many time you want to \"write=>read=>compare\" in each thread\n");
	seq_puts(m, "          [multi_address]    whether read/write different address in each thread, 0:No, 1:Yes\n");
	seq_printf(m, "\n   SMP_ON_ALL_HOST test:  echo %x [thread_num] [compare_count] [multi_address] > msdc_debug\n",
		SMP_TEST_ON_ALL_HOST);
	seq_puts(m, "          [thread_num]       how many R/W comparision thread you want to run at each host\n");
	seq_puts(m, "          [compare_count]    how many time you want to \"write=>read=>compare\" in each thread\n");
	seq_puts(m, "          [multi_address]    whether read/write different address in each thread, 0:No, 1:Yes\n");
	seq_puts(m, "\n   SPEED_MODE control:\n");
	seq_printf(m, "          set speed mode:    echo %x 0 [host_id] [speed_mode] [driver_type] [max_current] [power_control] > msdc_debug\n",
		SD_TOOL_MSDC_HOST_MODE);
	seq_printf(m, "          get speed mode:    echo %x 1 [host_id]\n",
		SD_TOOL_MSDC_HOST_MODE);
	seq_puts(m, "            [speed_mode]       ff:N/A,  0:HS,      1:SDR12,   2:SDR25,   3:SDR:50,  4:SDR104,  5:DDR, 6:HS400\n");
	seq_puts(m, "            [driver_type]      ff:N/A,  0: type A, 1:type B,  2:type C,  3:type D\n");
	seq_puts(m, "            [max_current]      ff:N/A,  0:200mA,  1:400mA,   2:600mA,   3:800mA\n");
	seq_puts(m, "            [power_control]    ff:N/A,  0:disable, 1:enable\n");
	seq_printf(m, "\n   DMA viloation:         echo %x [host_id] [ops]> msdc_debug\n",
		SD_TOOL_DMA_STATUS);
	seq_puts(m, "          [ops]              0:get latest dma address, 1:start violation test\n");
	seq_printf(m, "\n   SET Slew Rate:         echo %x [host_id] [clk] [cmd] [dat] [rst] [ds]> msdc_debug\n",
		SD_TOOL_ENABLE_SLEW_RATE);
	seq_puts(m, "\n   TD/RD SEL:\n");
	seq_printf(m, "          set rdsel:             echo %x [host_id] 0 [value] > msdc_debug\n",
		SD_TOOL_SET_RDTDSEL);
	seq_printf(m, "          set tdsel:             echo %x [host_id] 1 [value] > msdc_debug\n",
		SD_TOOL_SET_RDTDSEL);
	seq_printf(m, "          get tdsel/rdsel:       echo %x [host_id] 2 > msdc_debug\n",
		SD_TOOL_SET_RDTDSEL);
	seq_puts(m, "            [value]              rdsel: 0x0<<4 ~ 0x3f<<4,    tdsel: 0x0~0xf\n");
	seq_printf(m, "\n   EMMC/SD RW test:       echo %x [host_id] [mode] > msdc_debug\n",
		MSDC_READ_WRITE);
	seq_puts(m, "          [mode]               mode 0:stop, 1:read, 2:write\n");
	seq_printf(m, "\n   Error tune debug:       echo %x [host_id] [cmd_id] [arg] [error_type] [count] > msdc_debug\n",
		MMC_ERROR_TUNE);
	seq_puts(m, "            [cmd_id]           0: CMD0, 1: CMD1, 2: CMD2......\n");
	seq_puts(m, "            [arg]              for CMD6, arg means ext_csd index......\n");
	seq_puts(m, "            [error]            0: disable error tune debug, 1: cmd timeout, 2: cmd crc, 4: dat timeout, 8: dat crc, 16: acmd timeout, 32: acmd crc\n");
	seq_puts(m, "            [count]            error count\n");
#ifdef MTK_MSDC_USE_CACHE
	seq_printf(m, "\n   eMMC Cache Control: echo %x [host_id] [action_id] > /proc/msdc_debug\n",
		MMC_EDC_EMMC_CACHE);
	seq_puts(m, "            [action_id]        0:Disable cache 1:Enable cache 2:check cache status\n");
#endif
	seq_printf(m, "\n   eMMC Dump GPD/BD:      echo %x [host_id] > /proc/msdc_debug\n",
		MMC_DUMP_GPD);
	seq_printf(m, "\n   eMMC ETT Tune:         echo %x [type] [start_voltage], [end_voltage] > /proc/msdc_debug\n",
		MMC_ETT_TUNE);
	seq_puts(m, "            [type]             0:tune cmd  1:tune read  2:tune write  3:tune HS400\n");
	seq_puts(m, "            [start_voltage]    ?mV\n");
	seq_puts(m, "            [end_voltage]      ?mV, we try ETT from higher voltage to lower voltage\n");
	seq_printf(m, "\n   CRC Stress Test:       echo %x [action_id]> /proc/msdc_debug\n",
		MMC_CRC_STRESS);
	seq_puts(m, "            [action_id]        0:disable 1:enable\n");
	seq_printf(m, "\n   Enable AXI Modules:    echo %x [action_id][module_id]> /proc/msdc_debug\n",
		ENABLE_AXI_MODULE);
	seq_puts(m, "            [action_id]        0:disable 1:enable\n");
	seq_puts(m, "            [module_id]        0:NFI  1:MSDC1  2:USB  3:PERI  4:AUDIO  5:ALL\n");

	seq_printf(m, "\n   SDIO AutoK Result :    echo %x [host_id][vcore][rw]> /proc/msdc_debug\n",
		SDIO_AUTOK_RESULT);
	seq_puts(m, "            [host_id]  2:sdio\n");
	seq_puts(m, "            [vcore]    0:low  1:high\n");
	seq_puts(m, "            [rw]       0:read  1:write\n");

	seq_puts(m, "\n   NOTE: All input data is Hex number!\n");

	seq_puts(m, "\n=============================================\n\n");

	return 0;
}

void msdc_hw_parameter_debug(struct msdc_hw *hw, struct seq_file *m, void *v)
{
	unsigned int i;
	seq_printf(m, "hw->clk_src = %x\n", hw->clk_src);
	seq_printf(m, "hw->cmd_edge = %x\n", hw->cmd_edge);
	seq_printf(m, "hw->rdata_edge = %x\n", hw->rdata_edge);
	seq_printf(m, "hw->wdata_edge = %x\n", hw->wdata_edge);
	seq_printf(m, "hw->clk_drv = %x\n", hw->clk_drv);
	seq_printf(m, "hw->cmd_drv = %x\n", hw->cmd_drv);
	seq_printf(m, "hw->dat_drv = %x\n", hw->dat_drv);
	seq_printf(m, "hw->rst_drv = %x\n", hw->rst_drv);
	seq_printf(m, "hw->ds_drv = %x\n", hw->ds_drv);
	seq_printf(m, "hw->data_pins = %x\n", (unsigned int)hw->data_pins);
	seq_printf(m, "hw->data_offset = %x\n", (unsigned int)hw->data_offset);
	seq_printf(m, "hw->flags = %x\n", (unsigned int)hw->flags);
	seq_printf(m, "hw->dat0rddly = %x\n", hw->dat0rddly);
	seq_printf(m, "hw->dat1rddly = %x\n", hw->dat1rddly);
	seq_printf(m, "hw->dat2rddly = %x\n", hw->dat2rddly);
	seq_printf(m, "hw->dat3rddly = %x\n", hw->dat3rddly);
	seq_printf(m, "hw->dat4rddly = %x\n", hw->dat4rddly);
	seq_printf(m, "hw->dat5rddly = %x\n", hw->dat5rddly);
	seq_printf(m, "hw->dat6rddly = %x\n", hw->dat6rddly);
	seq_printf(m, "hw->dat7rddly = %x\n", hw->dat7rddly);
	seq_printf(m, "hw->datwrddly = %x\n", hw->datwrddly);
	seq_printf(m, "hw->cmdrrddly = %x\n", hw->cmdrrddly);
	seq_printf(m, "hw->cmdrddly = %x\n", hw->cmdrddly);
	seq_printf(m, "hw->ett_count = %x\n", hw->ett_count);
	/*seq_printf(m,"hw->ett_settings  = %x\n", hw->ett_settings);*/
	seq_printf(m, "hw->host_function = %x\n", (u32)hw->host_function);
	seq_printf(m, "hw->boot = %x\n", hw->boot);

	for (i = 0; i < hw->ett_count; i++) {
		seq_printf(m, "msdc0_ett_settings[%d]: %x, %x, %x, %x\n", i,
			hw->ett_settings[i].speed_mode,
			hw->ett_settings[i].reg_addr,
			hw->ett_settings[i].reg_offset,
			hw->ett_settings[i].value);
	}

}
/* ========== driver proc interface =========== */
static int msdc_debug_proc_show(struct seq_file *m, void *v)
{

	seq_puts(m, "\n=========================================\n");

	seq_puts(m, "Index<0> + Id + Zone\n");
	seq_puts(m, "-> PWR<9> WRN<8> | FIO<7> OPS<6> FUN<5> CFG<4> | INT<3> RSP<2> CMD<1> DMA<0>\n");
	seq_puts(m, "-> echo 0 3 0x3ff >msdc_bebug -> host[3] debug zone set to 0x3ff\n");
	seq_printf(m, "-> MSDC[0] Zone: 0x%.8x\n", sd_debug_zone[0]);
	seq_printf(m, "-> MSDC[1] Zone: 0x%.8x\n", sd_debug_zone[1]);

	seq_printf(m, "-> MSDC[2] Zone: 0x%.8x\n", sd_debug_zone[2]);
	seq_printf(m, "-> MSDC[3] Zone: 0x%.8x\n", sd_debug_zone[3]);

	seq_puts(m, "Index<1> + ID:4|Mode:4 + DMA_SIZE\n");
	seq_puts(m, "-> 0)PIO 1)DMA 2)SIZE\n");
	seq_puts(m, "-> echo 1 22 0x200 >msdc_bebug -> host[2] size mode, dma when >= 512\n");
	seq_printf(m, "-> MSDC[0] mode<%d> size<%d>\n",
		drv_mode[0], dma_size[0]);
	seq_printf(m, "-> MSDC[1] mode<%d> size<%d>\n",
		drv_mode[1], dma_size[1]);

	seq_printf(m, "-> MSDC[2] mode<%d> size<%d>\n",
		drv_mode[2], dma_size[2]);
	seq_printf(m, "-> MSDC[3] mode<%d> size<%d>\n",
		drv_mode[3], dma_size[3]);

	seq_puts(m, "Index<3> + SDIO_PROFILE + TIME\n");
	seq_puts(m, "-> echo 3 1 0x1E > msdc_bebug -> enable sdio_profile, 30s\n");
	seq_printf(m, "-> SDIO_PROFILE<%d> TIME<%llu s>\n",
		sdio_pro_enable, sdio_pro_time);
	seq_printf(m, "-> Clokc SRC of Host[0]<%d>\n", msdc_clock_src[0]);
	seq_printf(m, "-> Clokc SRC of Host[1]<%d>\n", msdc_clock_src[1]);
	seq_printf(m, "-> Clokc SRC of Host[2]<%d>\n", msdc_clock_src[2]);
	seq_printf(m, "-> Clokc SRC of Host[3]<%d>\n", msdc_clock_src[3]);
	seq_puts(m, "=========================================\n\n");
	seq_puts(m, "Index<4> msdc0 hw parameter and ett settings:\n");
	msdc_hw_parameter_debug(&msdc0_hw, m, v);
#if defined(CFG_DEV_MSDC1)
	seq_puts(m, "Index<5> msdc1 hw parameter:\n");
	msdc_hw_parameter_debug(&msdc1_hw, m, v);
#endif

	return 0;
}

/*
  *data: bit0~4:id, bit4~7: mode
*/
static int rwThread(void *data)
{
	int error, i = 0;
	ulong p = (ulong) data;
	int id = p & 0x3;
	int mode = (p >> 4) & 0x3;
	int read;
	uint type, address;

	pr_err("[****SD_rwThread****]id=%d, mode=%d\n", id, mode);

	if (mode == 1)
		read = 1;
	else if (mode == 2)
		read = 0;
	else
		return -1;

	while (read_write_state != 0) {
		if (read_write_state == 1)
			p = 0x3;
		else if (read_write_state == 2)
			p = 0;

		#ifdef CONFIG_MTK_EMMC_SUPPORT
		if (id == 0) {
			type = MMC_TYPE_MMC;
			address = COMPARE_ADDRESS_MMC;
		} else
		#endif
		if (id < HOST_MAX_NUM) {
			type = MMC_TYPE_SD;
			address = COMPARE_ADDRESS_SD;
		}

		error = multi_rw_compare_core(id, read, address, type);
		if (error) {
			pr_err("[%s]: failed data id0, error=%d\n",
				__func__, error);
				break;
		}

		i++;
		if (i == 10000) {
			pr_err("[***rwThread %s***]",
				read_write_state == 1 ? "read" : "write");
			i = 0;
		}
	}
	pr_err("[SD_Debug]rwThread exit\n");
	return 0;
}

#ifdef MTK_MSDC_USE_CACHE
static int msdc_check_emmc_cache_status(struct msdc_host *host)
{
	struct mmc_card *card = host->mmc->card;
	msdc_get_cache_region_func(host);
	mmc_claim_host(host->mmc);
	if (card && !mmc_card_mmc(card)) {
		pr_err("host:%d is not a eMMC card\n", host->id);
		goto exit;
	} else {
		if (0 == host->mmc->card->ext_csd.cache_size) {
			pr_err("card don't support cache feature\n");
			goto unsupport;
		} else {
			pr_err("card cache size:%dKB\n",
				host->mmc->card->ext_csd.cache_size/8);
		}
	}
	if (host->mmc->card->ext_csd.cache_ctrl)
		pr_err("Current Cache status: Enable\n");
	else
		pr_err("Current Cache status: Disable\n");
	mmc_release_host(host->mmc);

	return host->mmc->card->ext_csd.cache_ctrl;

exit:
	return -2;
unsupport:
	return -1;
}

static int msdc_enable_emmc_cache(struct msdc_host *host, int enable)
{
	u32 err;
	u8 c_ctrl;
	struct mmc_card *card = host->mmc->card;
	mmc_claim_host(host->mmc);
	if (card && !mmc_card_mmc(card)) {
		pr_err("host:%d is not a eMMC card\n", host->id);
		goto exit;
	}
	msdc_get_cache_region_func(host);

	err = msdc_check_emmc_cache_status(host);

	if (err < 0)
		goto exit;
	c_ctrl = host->mmc->card->ext_csd.cache_ctrl;

	if (c_ctrl && enable) {
		pr_err("cache already enabled, don't need enable it\n");
	} else if (c_ctrl && !enable) {
		err = mmc_cache_ctrl(host->mmc, enable);
		if (err) {
			pr_warn("%s: Cache is supported, but failed to turn off (%d)\n",
				mmc_hostname(host->mmc), err);
		} else {
			pr_err("disable cache successfully\n");
			host->mmc->caps2 &= ~MMC_CAP2_CACHE_CTRL;
		}
	} else if (!c_ctrl && enable) {
		host->mmc->caps2 |= MMC_CAP2_CACHE_CTRL;
		err = mmc_cache_ctrl(host->mmc, enable);
		if (err) {
			pr_warn("%s: Cache is supported, but failed to turn on (%d)\n",
				mmc_hostname(host->mmc), err);
		} else {
			pr_err("enable cache successfully\n");
		}
	} else if (!c_ctrl && !enable) {
		pr_err("cache has already been in disable status, don't need disable it\n");
	}

	mmc_release_host(host->mmc);

	return 0;
exit:
	return -1;
}
#endif

#ifdef MTK_MSDC_ERROR_TUNE_DEBUG
#define MTK_MSDC_ERROR_NONE	(0)
#define MTK_MSDC_ERROR_CMD_TMO	(0x1)
#define MTK_MSDC_ERROR_CMD_CRC	(0x1 << 1)
#define MTK_MSDC_ERROR_DAT_TMO	(0x1 << 2)
#define MTK_MSDC_ERROR_DAT_CRC	(0x1 << 3)
#define MTK_MSDC_ERROR_ACMD_TMO	(0x1 << 4)
#define MTK_MSDC_ERROR_ACMD_CRC	(0x1 << 5)
unsigned int g_err_tune_dbg_count = 0;
unsigned int g_err_tune_dbg_host = 0;
unsigned int g_err_tune_dbg_cmd = 0;
unsigned int g_err_tune_dbg_arg = 0;
unsigned int g_err_tune_dbg_error = MTK_MSDC_ERROR_NONE;

static void msdc_error_tune_debug_print(int p1, int p2, int p3, int p4, int p5)
{
	g_err_tune_dbg_host = p1;
	g_err_tune_dbg_cmd = p2;
	g_err_tune_dbg_arg = p3;
	g_err_tune_dbg_error = p4;
	g_err_tune_dbg_count = p5;
	if (g_err_tune_dbg_count &&
	    (g_err_tune_dbg_error != MTK_MSDC_ERROR_NONE)) {
		pr_err("===================MSDC error debug start =======================\n");
		pr_err("host:%d, cmd=%d, arg=%d, error=%d, count=%d\n",
			g_err_tune_dbg_host, g_err_tune_dbg_cmd,
			g_err_tune_dbg_arg, g_err_tune_dbg_error,
			g_err_tune_dbg_count);
	} else {
		g_err_tune_dbg_host = 0;
		g_err_tune_dbg_cmd = 0;
		g_err_tune_dbg_arg = 0;
		g_err_tune_dbg_error = MTK_MSDC_ERROR_NONE;
		g_err_tune_dbg_count = 0;
		pr_err("host:%d, cmd=%d, arg=%d, error=%d, count=%d\n",
			g_err_tune_dbg_host, g_err_tune_dbg_cmd,
			g_err_tune_dbg_arg, g_err_tune_dbg_error,
			g_err_tune_dbg_count);
		pr_err("=====================MSDC error debug end =======================\n");
	}
}

void msdc_error_tune_debug1(struct msdc_host *host, struct mmc_command *cmd,
	struct mmc_command *sbc, u32 *intsts)
{
	u32 opcode = cmd->opcode;

	if (!g_err_tune_dbg_error ||
	    (g_err_tune_dbg_count <= 0) ||
	    (g_err_tune_dbg_host != host->id))
		return;

	if (g_err_tune_dbg_cmd == opcode) {
		if ((opcode != MMC_SWITCH)
		 || ((opcode == MMC_SWITCH) &&
		     (g_err_tune_dbg_arg == ((cmd->arg >> 16) & 0xff)))) {
			if (g_err_tune_dbg_error & MTK_MSDC_ERROR_CMD_TMO) {
				*intsts = MSDC_INT_CMDTMO;
				g_err_tune_dbg_count--;
			} else if (g_err_tune_dbg_error
				 & MTK_MSDC_ERROR_CMD_CRC) {
				*intsts = MSDC_INT_RSPCRCERR;
				g_err_tune_dbg_count--;
			}
			pr_err("[%s]: got the error cmd:%d, arg=%d, dbg error=%d, cmd->error=%d, count=%d\n",
				__func__, g_err_tune_dbg_cmd,
				g_err_tune_dbg_arg, g_err_tune_dbg_error,
				cmd->error, g_err_tune_dbg_count);
		}
	}

#ifdef MTK_MSDC_USE_CMD23
	if ((g_err_tune_dbg_cmd == MMC_SET_BLOCK_COUNT) &&
	    sbc &&
	    (host->autocmd & MSDC_AUTOCMD23)) {
		if (g_err_tune_dbg_error & MTK_MSDC_ERROR_ACMD_TMO) {
			*intsts = MSDC_INT_ACMDTMO;
		} else if (g_err_tune_dbg_error
			& MTK_MSDC_ERROR_ACMD_CRC) {
			*intsts = MSDC_INT_ACMDCRCERR;
		} else {
			return;
		}
		pr_err("[%s]: got the error cmd:%d, dbg error=%d, sbc->error=%d, count=%d\n",
			__func__, g_err_tune_dbg_cmd, g_err_tune_dbg_error,
			sbc->error, g_err_tune_dbg_count);
	}
#endif

}

void msdc_error_tune_debug2(struct msdc_host *host, struct mmc_command *stop,
	u32 *intsts)
{
	void __iomem *base = host->base;

	if (!g_err_tune_dbg_error ||
	    (g_err_tune_dbg_count <= 0) ||
	    (g_err_tune_dbg_host != host->id))
		return;

	if (g_err_tune_dbg_cmd == (MSDC_READ32(SDC_CMD) & 0x3f)) {
		if (g_err_tune_dbg_error & MTK_MSDC_ERROR_DAT_TMO) {
			*intsts = MSDC_INT_DATTMO;
			g_err_tune_dbg_count--;
		} else if (g_err_tune_dbg_error & MTK_MSDC_ERROR_DAT_CRC) {
			*intsts = MSDC_INT_DATCRCERR;
			g_err_tune_dbg_count--;
		}
		pr_err("[%s]: got the error cmd:%d, dbg error 0x%x, data->error=%d, count=%d\n",
			__func__, g_err_tune_dbg_cmd, g_err_tune_dbg_error,
			host->data->error, g_err_tune_dbg_count);
	}
	if ((g_err_tune_dbg_cmd == MMC_STOP_TRANSMISSION) &&
	    stop &&
	    (host->autocmd & MSDC_AUTOCMD12)) {
		if (g_err_tune_dbg_error & MTK_MSDC_ERROR_ACMD_TMO) {
			*intsts = MSDC_INT_ACMDTMO;
			g_err_tune_dbg_count--;
		} else if (g_err_tune_dbg_error & MTK_MSDC_ERROR_ACMD_CRC) {
			*intsts = MSDC_INT_ACMDCRCERR;
			g_err_tune_dbg_count--;
		}
		pr_err("[%s]: got the error cmd:%d, dbg error 0x%x, stop->error=%d, host->error=%d, count=%d\n",
			__func__, g_err_tune_dbg_cmd, g_err_tune_dbg_error,
			stop->error, host->error, g_err_tune_dbg_count);
	}
}

void msdc_error_tune_debug3(struct msdc_host *host, struct mmc_command *cmd,
	u32 *intsts)
{
	if (!g_err_tune_dbg_error ||
	    (g_err_tune_dbg_count <= 0) ||
	    (g_err_tune_dbg_host != host->id))
		return;

	if (g_err_tune_dbg_cmd != cmd->opcode)
		return;

	if ((g_err_tune_dbg_cmd != MMC_SWITCH)
	 || ((g_err_tune_dbg_cmd == MMC_SWITCH) &&
	     (g_err_tune_dbg_arg == ((cmd->arg >> 16) & 0xff)))) {
		if (g_err_tune_dbg_error & MTK_MSDC_ERROR_CMD_TMO) {
			*intsts = MSDC_INT_CMDTMO;
			g_err_tune_dbg_count--;
		} else if (g_err_tune_dbg_error & MTK_MSDC_ERROR_CMD_CRC) {
			*intsts = MSDC_INT_RSPCRCERR;
			g_err_tune_dbg_count--;
		}
		pr_err("[%s]: got the error cmd:%d, arg=%d, dbg error=%d, cmd->error=%d, count=%d\n",
			__func__, g_err_tune_dbg_cmd, g_err_tune_dbg_arg,
			g_err_tune_dbg_error, cmd->error, g_err_tune_dbg_count);
	}
}
#endif /*ifdef MTK_MSDC_ERROR_TUNE_DEBUG*/

static ssize_t msdc_debug_proc_write(struct file *file, const char *buf,
	size_t count, loff_t *data)
{
	int ret = 0;
	int cmd, p1, p2, p3, p4, p5, p6, p7 = 0;
	int id, zone, vcore;
	int mode, size;
	int thread_num, compare_count, multi_address;
	void __iomem *base = NULL;
	ulong data_for_wr;
	unsigned int offset = 0;
	unsigned int reg_value;
	SD3_MODE spd_mode = CAPS_SPEED_NULL;
	SD3_DRIVE drv_type = CAPS_DRIVE_NULL;
	struct msdc_host *host = NULL;
#ifdef MSDC_DMA_ADDR_DEBUG
	struct dma_addr *dma_address, *p_dma_address;
#endif
	int dma_status;
	struct task_struct *rw_thread = NULL;
	int sscanf_num;
	u8 *res;

	if (count == 0)
		return -1;
	if (count > 255)
		count = 255;
	ret = copy_from_user(cmd_buf, buf, count);
	if (ret < 0)
		return -1;

	cmd_buf[count] = '\0';
	pr_err("[SD_Debug]msdc Write %s\n", cmd_buf);

	sscanf_num = sscanf(cmd_buf, "%x %x %x %x %x %x %x %x", &cmd, &p1, &p2,
		&p3, &p4, &p5, &p6, &p7);
	if (sscanf_num < 1)
		return count;

	if (cmd == SD_TOOL_ZONE) {
		id = p1;
		zone = p2;	/* zone &= 0x3ff; */
		pr_err("[SD_Debug]msdc host_<%d> zone<0x%.8x>\n", id, zone);
		if (id > HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		if (id == HOST_MAX_NUM) {
			sd_debug_zone[0] = sd_debug_zone[1] = zone;
			sd_debug_zone[2] = zone;
			sd_debug_zone[3] = zone;
		} else {
			sd_debug_zone[id] = zone;
		}
	} else if (cmd == SD_TOOL_DMA_SIZE) {
		id = p2;
		mode = p3;
		size = p4;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		if (p1 == 0) {
			drv_mode[id] = mode;
			dma_size[id] = size;
		} else {
			pr_err("-> MSDC[%d] mode<%d> size<%d>\n",
				 id, drv_mode[id], dma_size[id]);
		}
	} else if (cmd == SD_TOOL_CLK_SRC_SELECT) {
		id = p2;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		if (p1 == 0) {
			if (p3 >= 0 && p3 < CLK_SRC_MAX_NUM) {
				msdc_clock_src[id] = p3;
				pr_err("[SD_Debug]msdc%d's clk source changed to %d\n",
					id, msdc_clock_src[id]);
				pr_err("[SD_Debug]to enable the above settings, suspend and resume the phone again\n");
			} else {
				pr_err("[SD_Debug] invalide clock src id:%d, check /proc/msdc_help\n",
					p3);
			}
		} else if (p1 == 1) {
			pr_err("[SD_Debug]msdc%d's pll source is %d\n",
				id, msdc_clock_src[id]);
		}
	} else if (cmd == SD_TOOL_REG_ACCESS) {
		id = p2;
		offset = (unsigned int)p3;

		if (id >= HOST_MAX_NUM || id < 0 || mtk_msdc_host[id] == NULL)
			goto invalid_host_id;

		host = mtk_msdc_host[id];
		base = host->base;
		if ((offset == 0x18 || offset == 0x1C) && p1 != 4) {
			pr_err("[SD_Debug]Err: Accessing TXDATA and RXDATA is forbidden\n");
			return count;
		}

		msdc_clk_enable(host);

		if (p1 == 0) {
			reg_value = p4;
			pr_err("[SD_Debug][MSDC Reg]Original:0x%p+0x%x (0x%x)\n",
				base, offset, MSDC_READ32(base + offset));
			MSDC_WRITE32(base + offset, reg_value);
			pr_err("[SD_Debug][MSDC Reg]Modified:0x%p+0x%x (0x%x)\n",
				base, offset, MSDC_READ32(base + offset));
		} else if (p1 == 1) {
			pr_err("[SD_Debug][MSDC Reg]Reg:0x%p+0x%x (0x%x)\n",
					base, offset,
					MSDC_READ32(base + offset));
		} else if (p1 == 2) {
			msdc_set_field(base + offset, p4, p5, p6);
		} else if (p1 == 3) {
			msdc_get_field(base + offset, p4, p5, p6);
		} else if (p1 == 4) {
			msdc_dump_info(host->id);
		} else if (p1 == 5) {
			msdc_dump_info(host->id);
		}

		msdc_clk_disable(host);
	} else if (cmd == SD_TOOL_SET_DRIVING) {
		id = p1;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		host = mtk_msdc_host[id];
		if ((unsigned char)p2 > 7 || (unsigned char)p3 > 7 ||
		    (unsigned char)p4 > 7 || (unsigned char)p5 > 7 ||
		    (unsigned char)p6 > 7) {
			pr_err("[SD_Debug]Some drving value was invalid(invalid:0~7)\n");
		} else {
			if (p7 == 0x33) {
				host->hw->clk_drv = (unsigned char)p2;
				host->hw->cmd_drv = (unsigned char)p3;
				host->hw->dat_drv = (unsigned char)p4;
				host->hw->rst_drv = (unsigned char)p5;
				host->hw->ds_drv = (unsigned char)p6;
				msdc_set_driving(host, host->hw, 0);
			} else if (p7 == 0x18) {
				host->hw->clk_drv_sd_18 = (unsigned char)p2;
				host->hw->cmd_drv_sd_18 = (unsigned char)p3;
				host->hw->dat_drv_sd_18 = (unsigned char)p4;
				msdc_set_driving(host, host->hw, 1);
			}
			pr_err("[SD_Debug]clk_drv=%d, cmd_drv=%d, dat_drv=%d, rst_drv=%d, ds_drv=%d\n",
				p2, p3, p4, p5, p6);
		}
	} else if (cmd == SD_TOOL_ENABLE_SLEW_RATE) {
		id = p1;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		host = mtk_msdc_host[id];
		if ((unsigned char)p2 > 1 || (unsigned char)p3 > 1
		 || (unsigned char)p4 > 1 || (unsigned char)p5 > 1
		 || (unsigned char)p6 > 1) {
			pr_err("[SD_Debug]Some sr value was invalid(correct:0(disable),1(enable))\n");
		} else {
			msdc_set_sr(host, p2, p3, p4, p5, p6);
			pr_err("[SD_Debug]msdc%d, clk_sr=%d, cmd_sr=%d, dat_sr=%d, rst_sr=%d, ds_sr=%d\n",
				id, p2, p3, p4, p5, p6);
		}
	} else if (cmd == SD_TOOL_SET_RDTDSEL) {
		id = p1;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		host = mtk_msdc_host[id];
		if ((p2 < 0) || (p2 > 2)) {
			pr_err("[SD_Debug]invalid option ( set rd:0, set td:1, get td/rd: 2)\n");
		} else if ((p2 == 0 && (unsigned char)p3 > 0x3F)
			|| (p2 == 1 && (unsigned char)p3 > 0xF)) {
			pr_err("[SD_Debug]Some rd/td value was invalid (rd mask:(0x3F << 4),td mask:(0xF << 0))\n");
		} else {
			if (p2 == 0) {
				msdc_set_rdsel_dbg(host, p3);
				pr_err("[SD_Debug]msdc%d, set rd=%d\n",
					id, p3);
			} else if (p2 == 1) { /* set td:1 */
				msdc_set_tdsel_dbg(host, p3);
				pr_err("[SD_Debug]msdc%d, set td=%d\n",
					id, p3);
			} else if (p2 == 2) { /* get td/rd:2 */
				msdc_get_rdsel_dbg(host, &p3); /* get rd */
				msdc_get_tdsel_dbg(host, &p4); /* get td */
				pr_err("[SD_Debug]msdc%d, rd : 0x%x, td : 0x%x\n",
					id, p3, p4);
			}
		}
	} else if (cmd == SD_TOOL_ENABLE_SMT) {
		id = p1;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		host = mtk_msdc_host[id];
		msdc_set_smt(host, p2);
		pr_err("[SD_Debug]smt=%d\n", p2);
	} else if (cmd == RW_BIT_BY_BIT_COMPARE) {
		id = p1;
		compare_count = p2;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		if (compare_count < 0) {
			pr_err("[SD_Debug]: bad compare count: %d\n",
				compare_count);
			return count;
		}

		if (id == 0) { /* for msdc0 */
#ifdef CONFIG_MTK_EMMC_SUPPORT
			multi_rw_compare(0, COMPARE_ADDRESS_MMC,
				compare_count, MMC_TYPE_MMC);
			/*  test the address 0 of eMMC */
#else
			multi_rw_compare(0, COMPARE_ADDRESS_SD,
				compare_count, MMC_TYPE_SD);
			/* test a larger address of SD card */
#endif
		} else {
			multi_rw_compare(id, COMPARE_ADDRESS_SD,
				compare_count, MMC_TYPE_SD);
		}
	} else if (cmd == MSDC_READ_WRITE) {
		id = p1;
		mode = p2;	/* 0:stop, 1:read, 2:write */
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		if (mode > 2 || mode < 0) {
			pr_err("[SD_Debug]: bad mode: %d\n", mode);
			return count;
		}
		if (mode == read_write_state) {
			pr_err("[SD_Debug]: same operation mode=%d.\n",
				read_write_state);
			return count;
		}
		if (mode == 1 && read_write_state == 2) {
			pr_err("[SD_Debug]: cannot read in write state, please stop first\n");
			return count;
		}
		if (mode == 2 && read_write_state == 1) {
			pr_err("[SD_Debug]: cannot write in read state, please stop first\n");
			return count;
		}
		read_write_state = mode;

		pr_err("[SD_Debug]: host id: %d, mode: %d.\n", id, mode);
		if ((mode == 0) && (rw_thread)) {
			kthread_stop(rw_thread);
			pr_err("[SD_Debug]: stop read/write thread\n");
		} else {
			pr_err("[SD_Debug]: start read/write thread\n");
			data_for_wr = (id & 0x3) | ((mode & 0x3) << 4);
			rw_thread = kthread_create(rwThread,
				(void *)data_for_wr, "msdc_rw_thread");
			wake_up_process(rw_thread);
		}
	} else if (cmd == SD_TOOL_MSDC_HOST_MODE) {
		id = p2;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		if (p1 != 0) {
			pr_err("[SD_Debug]msdc[%d] supports:\n", id);
			pr_err("[SD_Debug]      speed mode: ");
			if (msdc_host_mode[id] &
				(MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED))
				pr_err("HS, ");
			if (msdc_host_mode[id] & MMC_CAP_UHS_SDR12)
				pr_err("SDR12, ");
			if (msdc_host_mode[id] & MMC_CAP_UHS_SDR25)
				pr_err("SDR25, ");
			if (msdc_host_mode[id] & MMC_CAP_UHS_SDR50)
				pr_err("SDR50, ");
			if (msdc_host_mode[id] & MMC_CAP_UHS_SDR104)
				pr_err("SDR104, ");
			if (msdc_host_mode[id] & MMC_CAP_UHS_DDR50)
				pr_err("DDR50");
			if (msdc_host_mode2[id] &
				MMC_CAP2_HS200_1_8V_SDR)
				pr_err("HS200, ");
			if (msdc_host_mode2[id] &
				MMC_CAP2_HS400_1_8V_DDR)
				pr_err("HS400 ");
			pr_err("\n");

			pr_err("[SD_Debug]      driver_type: ");
			if (msdc_host_mode[id] & MMC_CAP_DRIVER_TYPE_A)
				pr_err("A, ");
			pr_err("B, ");
			if (msdc_host_mode[id] & MMC_CAP_DRIVER_TYPE_C)
				pr_err("C, ");
			if (msdc_host_mode[id] & MMC_CAP_DRIVER_TYPE_D)
				pr_err("D");
			pr_err("\n");

			goto out;
		}

		if (p3 <= UHS_DDR50 && p3 >= SDHC_HIGHSPEED)
			spd_mode = p3;
		if (p4 <= DRIVER_TYPE_D && p4 >= DRIVER_TYPE_A)
			drv_type = p4;

		msdc_set_host_mode_speed(id, (int)spd_mode);

		msdc_set_host_mode_driver_type(id, (int)drv_type);

		pr_err("[SD_Debug]to enable the above settings, please suspend and resume the phone again\n");
	} else if (cmd == SD_TOOL_DMA_STATUS) {
		id = p1;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		if (p2 == 0) {
			char *str[] = {
				"No data transaction or the device is "
					"not present until now\n",
				"DMA mode is disabled Now\n",
				"Write from SD to DRAM in DMA mode\n",
				"Write from DRAM to SD in DMA mode\n"
			};
			dma_status = msdc_get_dma_status(id);
			pr_err(">>>> msdc%d: dma_status=%d, ", id, dma_status);
			pr_err("%s", str[dma_status+1]);
			if (dma_status == -1)
				goto out;
#ifdef MSDC_DMA_ADDR_DEBUG
			dma_address = msdc_get_dma_address(id);
			if (dma_address) {
				pr_err(">>>> msdc%d:\n", id);
				p_dma_address = dma_address;
				while (p_dma_address) {
					pr_err(">>>>     addr=0x%x, size=%d\n",
						p_dma_address->start_address,
						p_dma_address->size);
					if (p_dma_address->end)
						break;
					p_dma_address = p_dma_address->next;
				}
			} else {
				pr_err(">>>> msdc%d: BD count=0\n", id);
			}
#else
			pr_err("please enable MSDC_DMA_ADDR_DEBUG at mt_sd.h if you want dump dma address\n");
#endif
		} else if (p2 == 1) {
			pr_err(">>>> msdc%d: start dma violation test\n", id);
			g_dma_debug[id] = 1;
			multi_rw_compare(id, COMPARE_ADDRESS_SD, 3,
				MMC_TYPE_SD);
		}
	}

#ifdef MTK_MSDC_USE_CACHE
	else if (cmd == MMC_EDC_EMMC_CACHE) {
		pr_err("==== MSDC Cache Feature Test ====\n");
		id = p1;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;

		host = mtk_msdc_host[id];
		switch (p2) {
		case 0:
			msdc_enable_emmc_cache(host, 0);
			break;
		case 1:
			msdc_enable_emmc_cache(host, 1);
			break;
		case 2:
			msdc_check_emmc_cache_status(host);
			break;
		default:
			pr_err("ERROR:3rd parameter is wrong, please see the msdc_help\n");
			break;
		}
	}
#endif
	else if (cmd == MMC_DUMP_GPD) {
		pr_err("==== MSDC DUMP GPD/BD ====\n");
		id = p1;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		else
			msdc_dump_gpd_bd(id);
	} else if (cmd == SD_TOOL_SDIO_PROFILE) {
		if (p1 == 1) {	/* enable profile */
			if (gpt_enable == 0) {
				msdc_init_gpt();
				gpt_enable = 1;
			}
			sdio_pro_enable = 1;
			if (p2 == 0)
				p2 = 1;
			if (p2 >= 30)
				p2 = 30;
			sdio_pro_time = p2;
		} else if (p1 == 0) {
			/* todo */
			sdio_pro_enable = 0;
		}
	} else if (cmd == SMP_TEST_ON_ONE_HOST) {
		id = p1;
		thread_num = p2;
		compare_count = p3;
		multi_address = p4;
		smp_test_on_hosts(thread_num, id, compare_count, multi_address);
	} else if (cmd == SMP_TEST_ON_ALL_HOST) {
		thread_num = p1;
		compare_count = p2;
		multi_address = p3;
		smp_test_on_hosts(thread_num, HOST_MAX_NUM, compare_count,
			multi_address);
	} else if (cmd == MMC_REGISTER_READ) {
		pr_err("p1 = 0x%x\n", p1);
		pr_err("regiser: 0x%x = 0x%x\n", p1, MSDC_READ32((ulong) p1));
	}
#ifdef MTK_IO_PERFORMANCE_DEBUG
	else if (cmd == MMC_PERF_DEBUG) {
		/* 1 enable; 0 disable */
		g_mtk_mmc_perf_dbg = p1;
		g_mtk_mmc_dbg_range = p2;

		if (2 == g_mtk_mmc_dbg_range) {
			g_dbg_range_start = p3;
			g_dbg_range_end = p3 + p4;
			g_check_read_write = p5;
		}
		pr_err("g_mtk_mmc_perf_dbg = 0x%x, g_mtk_mmc_dbg_range = 0x%x, start = 0x%x, end = 0x%x\n",
			g_mtk_mmc_perf_dbg, g_mtk_mmc_dbg_range,
			g_dbg_range_start, g_dbg_range_end);
	} else if (cmd == MMC_PERF_DEBUG_PRINT) {
		int i, j, k, num = 0;

		if (p1 == 0) {
			g_mtk_mmc_clear = 0;
			return count;
		}
		pr_err("msdc g_dbg_req_count<%d>\n", g_dbg_req_count);
		for (i = 1; i <= g_dbg_req_count; i++) {
			pr_err("analysis: %s 0x%x %d block, PGh %d\n",
				(g_check_read_write == 18 ? "read" : "write"),
				(unsigned int)g_mmcqd_buf[i][298],
				(unsigned int)g_mmcqd_buf[i][299],
				(unsigned int)(g_mmcqd_buf[i][297] * 2));
			if (g_check_read_write == 18) {
				for (j = 1; j <= g_mmcqd_buf[i][296] * 2;
					j++) {
					pr_err("page %d:\n", num+1);
					for (k = 0; k < 5; k++) {
						pr_err("%d %llu\n",
							k, g_req_buf[num][k]);
					}
					num += 1;
				}
			}
			pr_err("----------------------------------\n");
			for (j = 0; j < sizeof(g_time_mark)/sizeof(char *);
				j++) {
				pr_err("%d. %llu %s\n", j, g_mmcqd_buf[i][j],
					g_time_mark[j]);
			}
			pr_err("==================================\n");
		}
		if (g_check_read_write == 25) {
			pr_err("msdc g_dbg_write_count<%d>\n",
				g_dbg_write_count);
			for (i = 1; i <= g_dbg_write_count; i++) {
				pr_err("**********************************\n");
				pr_err("write count: %llu\n",
					g_req_write_count[i]);
				for (j = 0; j < sizeof(g_time_mark_vfs_write)/
					sizeof(char *); j++) {
					pr_err("%d. %llu %s\n", j,
						g_req_write_buf[i][j],
						g_time_mark_vfs_write[j]);
				}
			}
			pr_err("**********************************\n");
		}
		g_mtk_mmc_clear = 0;
	}
#endif

#ifdef MTK_MMC_PERFORMANCE_TEST
	else if (cmd == MMC_PERF_TEST) {
		/* 1 enable; 0 disable */
		g_mtk_mmc_perf_test = p1;
	}
#endif

#ifdef MTK_MSDC_ERROR_TUNE_DEBUG
	else if (cmd == MMC_ERROR_TUNE)
		msdc_error_tune_debug_print(p1, p2, p3, p4, p5);
#endif

	else if (cmd == ENABLE_AXI_MODULE) {
		char *enable_str = (p1 ? "enable" : "disable");
		pr_err("==== %s AXI MODULE ====\n", enable_str);
		if (p2 == 0 || p2 == 5) {
			pr_err("%s %s transaction on AXI bus\n",
				enable_str, "NFI");
			/* NFI_SW_RST */
			MSDC_SET_FIELD(pericfg_reg_base, (0x1 << 14),
				(p1 ? 0x0 : 0x1));
		}
		if (p2 == 1 || p2 == 5) {
			pr_err("%s %s transaction on AXI bus\n",
				enable_str, "SD");
			/* MSDC1_SW_RST */
			MSDC_SET_FIELD(pericfg_reg_base, (0x1 << 20),
				(p1 ? 0x0 : 0x1));
		}
		if (p2 == 2 || p2 == 5) {
			pr_err("%s %s transaction on AXI bus\n",
				enable_str, "USB");
			/* USB_SW_RST */
			MSDC_SET_FIELD(pericfg_reg_base, (0x1 << 28),
				(p1 ? 0x0 : 0x1));
		}
		if (p2 == 3 || p2 == 5) {
			pr_err("%s %s transaction on AXI bus\n",
				enable_str, "PERI");
			/* PERI_AXI */
			MSDC_SET_FIELD(pericfg_reg_base + 0x210, (0x3 << 8),
				(p1 ? 0x3 : 0x2));
		}
		if (p2 == 4 || p2 == 5) {
			pr_err("%s %s transaction on AXI bus\n",
				enable_str, "AUDIO");
			/* AUDIO_RST */
			MSDC_SET_FIELD(infracfg_ao_reg_base + 0x40, (0x1 << 5),
				(p1 ? 0x0 : 0x1));
		}
		pr_err("disable AXI modules, reg[0x10003000]=0x%x, reg[0x10003210]=0x%x, reg[0x10001040]=0x%x\n",
			MSDC_READ32(pericfg_reg_base),
			MSDC_READ32(pericfg_reg_base + 0x210),
			MSDC_READ32(infracfg_ao_reg_base + 0x40));
	} else if (cmd == MMC_CRC_STRESS) {
		pr_err("==== CRC Stress Test ====\n");
		if (0 == p1) {
			g_reset_tune = 0;
		} else {
			g_reset_tune = 1;
			base = mtk_msdc_host[0]->base;
			MSDC_SET_FIELD(EMMC50_PAD_DS_TUNE,
				MSDC_EMMC50_PAD_DS_TUNE_DLY1, 0x1c);
			MSDC_SET_FIELD(EMMC50_PAD_DS_TUNE,
				MSDC_EMMC50_PAD_DS_TUNE_DLY3, 0xe);
		}
	} else if (cmd == SDIO_AUTOK_RESULT) {
		id = p1;
		vcore = p2;
		mode = p3;
		host = mtk_msdc_host[id];
		base = host->base;
		pr_err("[****AutoK test****]msdc host_id<%d> vcore<%d> mode<%d>\n", id, vcore, mode);

		vcore = (vcore <= 0) ? AUTOK_VCORE_LOW : AUTOK_VCORE_HIGH;
		res = sdio_autok_res[vcore];

		if (mode == 0) {
			if (sdio_autok_res_apply(host, vcore) != 0)
				pr_err("sdio autok result not exist!\n");
		} else if (mode == 1) {
			sdio_autok_res_save(host, vcore, res);
		} else if (mode == 2) {
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_IOCON, MSDC_READ32(MSDC_IOCON));
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_PATCH_BIT0, MSDC_READ32(MSDC_PATCH_BIT0));
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_PATCH_BIT1, MSDC_READ32(MSDC_PATCH_BIT1));
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_PATCH_BIT2, MSDC_READ32(MSDC_PATCH_BIT2));
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_PAD_TUNE0, MSDC_READ32(MSDC_PAD_TUNE0));
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_PAD_TUNE1, MSDC_READ32(MSDC_PAD_TUNE1));
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_DAT_RDDLY0, MSDC_READ32(MSDC_DAT_RDDLY0));
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_DAT_RDDLY1, MSDC_READ32(MSDC_DAT_RDDLY1));
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_DAT_RDDLY2, MSDC_READ32(MSDC_DAT_RDDLY2));
			pr_err("%d R[%x]=0x%.8x\n", id, OFFSET_MSDC_DAT_RDDLY3, MSDC_READ32(MSDC_DAT_RDDLY3));
		}
	} else if (cmd == MMC_CMDQ_STATUS) {
		pr_err("==== eMMC CMDQ Feature ====\n");
		id = p1;
		if (id >= HOST_MAX_NUM || id < 0)
			goto invalid_host_id;
		host = mtk_msdc_host[id];
		msdc_cmdq_status_print(host);
	} else if (cmd == DO_AUTOK_OFFLINE_TUNE_TX) {
		host = mtk_msdc_host[p1]; /*p1 = id */
		mmc_claim_host(host->mmc);
		#if 0
		do_autok_offline_tune_tx = 1;
		host->mmc->ops->execute_tuning(host->mmc,
			MMC_SEND_TUNING_BLOCK_HS200);
		do_autok_offline_tune_tx = 0;
		#else
		autok_offline_tuning_TX(host);
		#endif
		mmc_release_host(host->mmc);
	}

out:
	return count;

invalid_host_id:
	pr_err("[SD_Debug]invalid host id: %d\n", id);
	return count;
}

static int msdc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, msdc_debug_proc_show, inode->i_private);
}

static const struct file_operations msdc_proc_fops = {
	.open = msdc_proc_open,
	.write = msdc_debug_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msdc_help_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, msdc_help_proc_show, inode->i_private);
}
static const struct file_operations msdc_help_fops = {
	.open = msdc_help_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifdef MSDC_HQA
u32 sdio_vio18_flag, sdio_vcore1_flag, sdio_vcore2_flag;
u32 vio18_reg, vcore1_reg, vcore2_reg;

static ssize_t msdc_voltage_proc_write(struct file *file,
	const char __user *buf, size_t count, loff_t *data)
{
	int ret;
	int sscanf_num;

	ret = copy_from_user(cmd_buf, buf, count);
	if (ret < 0)
		return -1;

	cmd_buf[count] = '\0';
	pr_err("[SD_Debug]msdc Write %s\n", cmd_buf);

	sscanf_num = sscanf(cmd_buf, "%d %d %d", &sdio_vio18_flag,
		&sdio_vcore1_flag, &sdio_vcore2_flag);

	if (sscanf_num < 3)
		return -1;

	if (sdio_vio18_flag >= 1730 && sdio_vio18_flag <= 1880) {
		/*0.01V per step*/
		if (sdio_vio18_flag > 1800)
			vio18_reg = 8-(sdio_vio18_flag-1800)/10;
		else
			vio18_reg = (1800-sdio_vio18_flag)/10;
		pmic_config_interface(REG_VIO18_CAL, vio18_reg,
			VIO18_CAL_MASK, VIO18_CAL_SHIFT);
	}

	/*Vcore2 is VCORE_AO*/
	if (sdio_vcore2_flag > 900 && sdio_vcore2_flag < 1200) {
		/*0.00625V per step*/
		/*Originally divied by 12.5, to avoid floating-point division,
		 amplify numerator and denominator by 4*/
		vcore2_reg = ((sdio_vcore2_flag-600)<<2)/25;
		pmic_config_interface(REG_VCORE_VOSEL_SW, vcore2_reg,
			VCORE_VOSEL_SW_MASK, VCORE_VOSEL_SW_SHIFT);
		pmic_config_interface(REG_VCORE_VOSEL_HW, vcore2_reg,
			VCORE_VOSEL_HW_MASK, VCORE_VOSEL_HW_SHIFT);
	}

	return count;
}

static int msdc_voltage_flag_proc_read_show(struct seq_file *m, void *data)
{
	seq_printf(m, "vio18: 0x%d 0x%X\n", sdio_vio18_flag, vio18_reg);
	seq_printf(m, "vcore1: 0x%d 0x%X\n", sdio_vcore1_flag,  vcore1_reg);
	seq_printf(m, "vcore2: 0x%d 0x%X\n", sdio_vcore2_flag, vcore2_reg);
	return 0;
}

static int msdc_voltage_flag_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, msdc_voltage_flag_proc_read_show,
		inode->i_private);
}

static const struct file_operations msdc_voltage_flag_fops = {
	.open = msdc_voltage_flag_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = msdc_voltage_proc_write,
};
#endif

#define PERM
#ifndef USER_BUILD_KERNEL
#define PROC_PERM		0660
#else
#define PROC_PERM		0440
#endif

int msdc_debug_proc_init(void)
{
	struct proc_dir_entry *prEntry;
#ifdef MSDC_HQA
	struct proc_dir_entry *voltage_flag;
#endif

	prEntry = proc_create("msdc_debug", 0660, NULL, &msdc_proc_fops);

	if (prEntry) {
		pr_err("[%s]: create /proc/msdc_debug\n", __func__);
		proc_set_user(prEntry, 0, 1001);
	} else {
		pr_err("[%s]: failed to create /proc/msdc_debug\n", __func__);
	}

	prEntry = proc_create("msdc_help", PROC_PERM, NULL, &msdc_help_fops);

	if (prEntry)
		pr_err("[%s]: create /proc/msdc_help\n", __func__);
	else
		pr_err("[%s]: failed to create /proc/msdc_help\n", __func__);

#ifdef MSDC_HQA
	voltage_flag = proc_create("msdc_voltage_flag", 0660, NULL,
		&msdc_voltage_flag_fops);

	if (voltage_flag) {
		proc_set_user(voltage_flag, 0, 1001);
		pr_err("[%s]: successfully create /proc/msdc_voltage_flag\n",
			__func__);
	} else {
		pr_err("[%s]: failed to create /proc/msdc_voltage_flag\n",
			__func__);
	}
#endif

#ifdef MSDC_DMA_ADDR_DEBUG
	msdc_init_dma_latest_address();
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(msdc_debug_proc_init);

