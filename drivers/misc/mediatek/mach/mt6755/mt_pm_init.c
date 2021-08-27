#include <linux/pm.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/types.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include "mach/irqs.h"
#include "mach/sync_write.h"
#include "mach/mt_reg_base.h"
#include "mach/mt_typedefs.h"
#include "mach/mt_spm.h"
#include "mach/mt_sleep.h"
#ifndef CONFIG_MTK_FPGA
#include "mach/mt_dcm.h"
#include "mach/mt_clkmgr.h"
#include "mach/mt_cpufreq.h"
#include "mach/mt_gpufreq.h"
#include "mach/mt_dormant.h"
#include "mach/mt_cpuidle.h"
#include "mach/mt_clkbuf_ctl.h"
#endif


#define pminit_write(addr, val)         mt_reg_sync_writel((val), ((void *)(addr)))
#define pminit_read(addr)               __raw_readl(IOMEM(addr))

extern int mt_clkmgr_init(void);
#ifndef CONFIG_MTK_FPGA
extern void mt_idle_init(void);
extern void mt_power_off(void);
extern void mt_dcm_init(void);
#endif

#define TOPCK_LDVT

#ifdef TOPCK_LDVT
/***************************
*For TOPCKGen Meter LDVT Test
****************************/
unsigned int ckgen_meter(int val)
{
#if 0
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	pminit_write(CLK_DBG_CFG, (val << 8) | 0x01);	/* sel ckgen_cksw and enable freq meter sel ckgen */

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF) | (0x07 << 24));	/* select divider */

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);
	pminit_write(CLK26CALI_1, 0x00ff0000);	/*  */

	clk26cali_0 = DRV_Reg32(CLK26CALI_0);
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	output = (((temp * 26000)) / 256) * 8;	/* Khz */

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);
	pminit_write(CLK26CALI_0, clk26cali_0);
	pminit_write(CLK26CALI_1, clk26cali_1);

	pr_debug("freq = %d\n", output);

	if (i > 10)
		return 0;
	else
		return output;
#else
	return 0;
#endif
}

unsigned int abist_meter(int val)
{
#if 0
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	pminit_write(CLK_DBG_CFG, val << 16);	/* sel abist_cksw and enable freq meter sel abist */

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	if (val == 16)
		DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF) | (0x1 << 24));	/* select divider */
	else
		DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF) | (0x7 << 24));	/* select divider */

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);
	if (val == 16)
		pminit_write(CLK26CALI_1, 0x000f0000);	/*  */
	else
		pminit_write(CLK26CALI_1, 0x00ff0000);	/*  */


	clk26cali_0 = DRV_Reg32(CLK26CALI_0);
	if (val == 16) {
		pminit_write(CLK26CALI_0, 0x1100);
		pminit_write(CLK26CALI_0, 0x1110);
	} else {
		pminit_write(CLK26CALI_0, 0x1000);
		pminit_write(CLK26CALI_0, 0x1010);
	}

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	if (val == 16)
		output = (((16 * 26000)) / temp) * 2;	/* Khz */
	else
		output = (((temp * 26000)) / 256) * 8;	/* Khz */

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);
	pminit_write(CLK26CALI_0, clk26cali_0);
	pminit_write(CLK26CALI_1, clk26cali_1);

	pr_debug("measure= %d, freq = %d\n", temp, output);

	if (i > 10)
		return 0;
	else
		return output;
#else
	return 0;
#endif
}

const char *ckgen_array[] = {
	"hd_faxi_ck", "hf_fddrphycfg_ck", "hf_fpwm_ck",
	"hf_fvdec_ck", "hf_fmm_ck", "hf_fcamtg_ck",
	"hf_fuart_ck", "hf_fspi_ck", "hf_fmsdc50_0_hclk_ck",
	"hf_fmsdc50_0_ck", "hf_fmsdc30_1_ck", "hf_fmsdc30_2_ck",
	"hf_fmsdc30_3_ck", "hf_faudio_ck", "hf_faud_intbus_ck",
	"hf_fpmicspi_ck", "hf_fscp_ck", "hf_fatb_ck",
	"hf_fmjc_ck", "hf_fdpi0_ck", "hf_faud_1_ck",
	"hf_faud_2_ck", "hf_fscam_ck", "hf_fmfg_ck",
	"mem_clkmux_ck", "mem_dcm_ck"
};

static int ckgen_meter_read(struct seq_file *m, void *v)
{
	int i;

	for (i = 1; i < 27; i++)
		seq_printf(m, "%s: %d\n", ckgen_array[i - 1], ckgen_meter(i));

	return 0;
}

static int ckgen_meter_write(struct file *file, const char __user *buffer,
			     size_t count, loff_t *data)
{
	char desc[128];
	int len = 0;
	int val;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &val) == 1) {
		pr_debug("ckgen_meter %d is %d\n", val, ckgen_meter(val));
	}
	return count;
}


static int abist_meter_read(struct seq_file *m, void *v)
{
	int i;

	for (i = 2; i < 26; i++)
		seq_printf(m, "%d\n", abist_meter(i));

	return 0;
}

static int abist_meter_write(struct file *file, const char __user *buffer,
			     size_t count, loff_t *data)
{
	char desc[128];
	int len = 0;
	int val;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d", &val) == 1) {
		pr_debug("abist_meter %d is %d\n", val, abist_meter(val));
	}
	return count;
}

static int proc_abist_meter_open(struct inode *inode, struct file *file)
{
	return single_open(file, abist_meter_read, NULL);
}

static const struct file_operations abist_meter_fops = {
	.owner = THIS_MODULE,
	.open = proc_abist_meter_open,
	.read = seq_read,
	.write = abist_meter_write,
};

static int proc_ckgen_meter_open(struct inode *inode, struct file *file)
{
	return single_open(file, ckgen_meter_read, NULL);
}

static const struct file_operations ckgen_meter_fops = {
	.owner = THIS_MODULE,
	.open = proc_ckgen_meter_open,
	.read = seq_read,
	.write = ckgen_meter_write,
};

#endif

/*********************************************************************
 * FUNCTION DEFINATIONS
 ********************************************************************/

unsigned int mt_get_emi_freq(void)
{
#if 0
	int output = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	pminit_write(CLK_DBG_CFG, 0x1901);	/* sel ckgen_cksw and enable freq meter sel ckgen */

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF) | (0x07 << 24));	/* select divider */

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);
	pminit_write(CLK26CALI_1, 0x00ff0000);	/*  */

	clk26cali_0 = DRV_Reg32(CLK26CALI_0);
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		pr_debug("wait for frequency meter finish, CLK26CALI = 0x%x\n",
			 DRV_Reg32(CLK26CALI_0));
		/* mdelay(10); */
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	output = (((temp * 26000)) / 256) * 8;	/* Khz */

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);
	pminit_write(CLK26CALI_0, clk26cali_0);
	pminit_write(CLK26CALI_1, clk26cali_1);

	/* print("CLK26CALI = 0x%x, bus frequency = %d Khz\n", temp, output); */

	return output;
#else
	return 0;
#endif
}
EXPORT_SYMBOL(mt_get_emi_freq);

unsigned int mt_get_bus_freq(void)
{
#if 0
	int output = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	pminit_write(CLK_DBG_CFG, 0x101);	/* sel ckgen_cksw and enable freq meter sel ckgen */

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF) | (0x07 << 24));	/* select divider */

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);
	pminit_write(CLK26CALI_1, 0x00ff0000);	/*  */

	clk26cali_0 = DRV_Reg32(CLK26CALI_0);
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		pr_debug("wait for frequency meter finish, CLK26CALI = 0x%x\n",
			 DRV_Reg32(CLK26CALI_0));
		/* mdelay(10); */
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	output = (((temp * 26000)) / 256) * 8;	/* Khz */

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);
	pminit_write(CLK26CALI_0, clk26cali_0);
	pminit_write(CLK26CALI_1, clk26cali_1);

	/* print("CLK26CALI = 0x%x, bus frequency = %d Khz\n", temp, output); */

	return output;
#else
	return 0;
#endif
}
EXPORT_SYMBOL(mt_get_bus_freq);

unsigned int mt_get_cpu_freq(void)
{
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	DRV_WriteReg32(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFFFFE) | (41 << 16));

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF));

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);

	DRV_WriteReg32(CLK26CALI_0, 0x1000);
	DRV_WriteReg32(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	output = ((temp * 26000)) / 1024;

	DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
	DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
	DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
	DRV_WriteReg32(CLK26CALI_1, clk26cali_1);

	if (i > 10)
		return 0;
	else
		return output;
}
EXPORT_SYMBOL(mt_get_cpu_freq);
unsigned int mt_get_bigcpu_freq(void)
{
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	DRV_WriteReg32(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFFFFE) | (40 << 16));

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF));

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);



	DRV_WriteReg32(CLK26CALI_0, 0x1000);
	DRV_WriteReg32(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	output = ((temp * 26000)) / 1024;

	DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
	DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
	DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
	DRV_WriteReg32(CLK26CALI_1, clk26cali_1);






	if (i > 10)
		return 0;
	else
		return output;
}
EXPORT_SYMBOL(mt_get_bigcpu_freq);


unsigned int mt_get_abist_meter(unsigned int id)
{
	int output = 0, i = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	DRV_WriteReg32(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFFFFE) | (id << 16));

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF));

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);

	DRV_WriteReg32(CLK26CALI_0, 0x1000);
	DRV_WriteReg32(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		mdelay(10);
		i++;
		if (i > 10)
			break;
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	output = ((temp * 26000)) / 1024;

	DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
	DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
	DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
	DRV_WriteReg32(CLK26CALI_1, clk26cali_1);

	if (i > 10)
		return 0;
	else
		return output;
}
EXPORT_SYMBOL(mt_get_abist_meter);
unsigned int mt_get_mmclk_freq(void)
{
#if 0
	int output = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	pminit_write(CLK_DBG_CFG, 0x501);	/* sel ckgen_cksw and enable freq meter sel ckgen */

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF) | (0x07 << 24));	/* select divider */

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);
	pminit_write(CLK26CALI_1, 0x00ff0000);	/*  */

	clk26cali_0 = DRV_Reg32(CLK26CALI_0);
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		pr_debug("wait for frequency meter finish, CLK26CALI = 0x%x\n",
			 DRV_Reg32(CLK26CALI_0));
		/* mdelay(10); */
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	output = (((temp * 26000)) / 256) * 8;	/* Khz */

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);
	pminit_write(CLK26CALI_0, clk26cali_0);
	pminit_write(CLK26CALI_1, clk26cali_1);

	pr_debug("CLK26CALI = 0x%x, bus frequency = %d Khz\n", temp, output);

	return output;
#else
	return 0;
#endif
}
EXPORT_SYMBOL(mt_get_mmclk_freq);

unsigned int mt_get_mfgclk_freq(void)
{
#if 0
	int output = 0;
	unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

	clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
	pminit_write(CLK_DBG_CFG, 0x1801);	/* sel ckgen_cksw and enable freq meter sel ckgen */

	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
	DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF) | (0x07 << 24));	/* select divider */

	clk26cali_1 = DRV_Reg32(CLK26CALI_1);
	pminit_write(CLK26CALI_1, 0x00ff0000);	/*  */

	clk26cali_0 = DRV_Reg32(CLK26CALI_0);
	pminit_write(CLK26CALI_0, 0x1000);
	pminit_write(CLK26CALI_0, 0x1010);

	/* wait frequency meter finish */
	while (DRV_Reg32(CLK26CALI_0) & 0x10) {
		pr_debug("wait for frequency meter finish, CLK26CALI = 0x%x\n",
			 DRV_Reg32(CLK26CALI_0));
		/* mdelay(10); */
	}

	temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

	output = (((temp * 26000)) / 256) * 8;	/* Khz */

	pminit_write(CLK_DBG_CFG, clk_dbg_cfg);
	pminit_write(CLK_MISC_CFG_0, clk_misc_cfg_0);
	pminit_write(CLK26CALI_0, clk26cali_0);
	pminit_write(CLK26CALI_1, clk26cali_1);

	/* print("CLK26CALI = 0x%x, bus frequency = %d Khz\n", temp, output); */

	return output;
#else
	return 0;
#endif
}
EXPORT_SYMBOL(mt_get_mfgclk_freq);

static int cpu_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_cpu_freq());
	return 0;
}

static int bigcpu_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_bigcpu_freq());
	return 0;
}

static int mmpll_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_abist_meter(25));
	return 0;
}

static int syspll_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_abist_meter(23));
	return 0;
}

static int univpll_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_abist_meter(24));
	return 0;
}

static int vencpll_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_abist_meter(27));
	return 0;
}

static int emi_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_emi_freq());
	return 0;
}

static int bus_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_bus_freq());
	return 0;
}

static int mmclk_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_mmclk_freq());
	return 0;
}

static int mfgclk_speed_dump_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", mt_get_mfgclk_freq());
	return 0;
}

static int proc_cpu_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_speed_dump_read, NULL);
}

static const struct file_operations cpu_fops = {
	.owner = THIS_MODULE,
	.open = proc_cpu_open,
	.read = seq_read,
};

static int proc_bigcpu_open(struct inode *inode, struct file *file)
{
	return single_open(file, bigcpu_speed_dump_read, NULL);
}

static const struct file_operations bigcpu_fops = {
	.owner = THIS_MODULE,
	.open = proc_bigcpu_open,
	.read = seq_read,
};

static int proc_mmpll_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmpll_speed_dump_read, NULL);
}

static const struct file_operations mm_fops = {
	.owner = THIS_MODULE,
	.open = proc_mmpll_open,
	.read = seq_read,
};

static int proc_vencpll_open(struct inode *inode, struct file *file)
{
	return single_open(file, vencpll_speed_dump_read, NULL);
}

static const struct file_operations venc_fops = {
	.owner = THIS_MODULE,
	.open = proc_vencpll_open,
	.read = seq_read,
};

static int proc_syspll_open(struct inode *inode, struct file *file)
{
	return single_open(file, syspll_speed_dump_read, NULL);
}

static const struct file_operations sys_fops = {
	.owner = THIS_MODULE,
	.open = proc_syspll_open,
	.read = seq_read,
};

static int proc_univpll_open(struct inode *inode, struct file *file)
{
	return single_open(file, univpll_speed_dump_read, NULL);
}

static const struct file_operations univ_fops = {
	.owner = THIS_MODULE,
	.open = proc_univpll_open,
	.read = seq_read,
};

static int proc_emi_open(struct inode *inode, struct file *file)
{
	return single_open(file, emi_speed_dump_read, NULL);
}

static const struct file_operations emi_fops = {
	.owner = THIS_MODULE,
	.open = proc_emi_open,
	.read = seq_read,
};

static int proc_bus_open(struct inode *inode, struct file *file)
{
	return single_open(file, bus_speed_dump_read, NULL);
}

static const struct file_operations bus_fops = {
	.owner = THIS_MODULE,
	.open = proc_bus_open,
	.read = seq_read,
};

static int proc_mmclk_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmclk_speed_dump_read, NULL);
}

static const struct file_operations mmclk_fops = {
	.owner = THIS_MODULE,
	.open = proc_mmclk_open,
	.read = seq_read,
};

static int proc_mfgclk_open(struct inode *inode, struct file *file)
{
	return single_open(file, mfgclk_speed_dump_read, NULL);
}

static const struct file_operations mfgclk_fops = {
	.owner = THIS_MODULE,
	.open = proc_mfgclk_open,
	.read = seq_read,
};



static int __init mt_power_management_init(void)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *pm_init_dir = NULL;

#ifndef CONFIG_MTK_FPGA
	pm_power_off = mt_power_off;
#endif

#if !defined(CONFIG_FPGA_CA7)
	/* FIXME: for FPGA early porting */
#if 0
	pr_debug("Power/PM_INIT" "Bus Frequency = %d KHz\n", mt_get_bus_freq());
#endif

	/* CPU Dormant Driver Init */
	mt_cpu_dormant_init();

	/* SPM driver init */
	spm_module_init();

	/* Sleep driver init (for suspend) */
	slp_module_init();

#ifndef CONFIG_MTK_FPGA
	mt_clkmgr_init();
#endif

	/* mt_pm_log_init(); // power management log init */

	/* FIXME: for FPGA early porting */

/* mt_dcm_init(); // dynamic clock management init */


	pm_init_dir = proc_mkdir("pm_init", NULL);
	if (!pm_init_dir) {
		pr_err("[%s]: mkdir /proc/pm_init failed\n", __func__);
	} else {
		entry = proc_create("cpu_speed_dump", S_IRUGO, pm_init_dir, &cpu_fops);

		entry = proc_create("bigcpu_speed_dump", S_IRUGO, pm_init_dir, &bigcpu_fops);
		entry = proc_create("mm_speed_dump", S_IRUGO, pm_init_dir, &mm_fops);
		entry = proc_create("venc_speed_dump", S_IRUGO, pm_init_dir, &venc_fops);
		entry = proc_create("sys_speed_dump", S_IRUGO, pm_init_dir, &sys_fops);
		entry = proc_create("univ_speed_dump", S_IRUGO, pm_init_dir, &univ_fops);


		entry = proc_create("emi_speed_dump", S_IRUGO, pm_init_dir, &emi_fops);

		entry = proc_create("bus_speed_dump", S_IRUGO, pm_init_dir, &bus_fops);

		entry = proc_create("mmclk_speed_dump", S_IRUGO, pm_init_dir, &mmclk_fops);

		entry = proc_create("mfgclk_speed_dump", S_IRUGO, pm_init_dir, &mfgclk_fops);
#ifdef TOPCK_LDVT
		entry =
		    proc_create("abist_meter_test", S_IRUGO | S_IWUSR, pm_init_dir,
				&abist_meter_fops);
		entry =
		    proc_create("ckgen_meter_test", S_IRUGO | S_IWUSR, pm_init_dir,
				&ckgen_meter_fops);
#endif
	}

#endif

	return 0;
}
arch_initcall(mt_power_management_init);


#if !defined(MT_DORMANT_UT)
static int __init mt_pm_late_init(void)
{
#ifndef CONFIG_MTK_FPGA
	mt_idle_init();
	clk_buf_init();
#endif
	return 0;
}
late_initcall(mt_pm_late_init);
#endif				/* #if !defined (MT_DORMANT_UT) */


MODULE_DESCRIPTION("MTK Power Management Init Driver");
MODULE_LICENSE("GPL");
