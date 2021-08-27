/*
* Copyright (c) 2015 MediaTek Inc.
* Author: Cheng-En Chung <cheng-en.chung@mediatek.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <asm/irqflags.h>
#include <asm/neon.h>
#include <asm/psci.h>
#include <asm/suspend.h>
#include <asm/setup.h>

#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_fdt.h>

#include <mach/mt_irq.h>
#include <mach/sync_write.h>

#include "mt_cpuidle.h"
#include "mt_spm_reg.h"
#include "mt_spm.h"

#ifdef CONFIG_MTK_RAM_CONSOLE
#include <mach/mt_secure_api.h>
#endif


#define TAG "[Power-Dormant] "

#define dormant_err(fmt, args...)	pr_emerg(TAG fmt, ##args)
#define dormant_warn(fmt, args...)	pr_warn(TAG fmt, ##args)
#define dormant_dbg(fmt, args...)	pr_debug(TAG fmt, ##args)


static unsigned long gic_id_base;

static unsigned int kp_irq_bit;
static unsigned int conn_wdt_irq_bit;
static unsigned int lowbattery_irq_bit;
static unsigned int md1_wdt_bit;
static unsigned int c2k_wdt_bit;


#define MAX_CORES 4
#define MAX_CLUSTER 2

#define GIC_NODE		"mtk,mt-gic"

#define KP_NODE			"mediatek,KP"
#define CONSYS_NODE		"mediatek,mt6755-consys"
#define AUXADC_NODE		"mediatek,AUXADC"
#define MDCLDMA_NODE		"mediatek,mdcldma"
#define MDC2K_NODE		"mediatek,ap2c2k_ccif"

#define DMT_GIC_DIST_BASE	(gic_id_base)

#define DMT_KP_IRQ_BIT		(kp_irq_bit)
#define DMT_CONN_WDT_IRQ_BIT	(conn_wdt_irq_bit)
#define DMT_LOWBATTERY_IRQ_BIT	(lowbattery_irq_bit)
#define DMT_MD1_WDT_BIT		(md1_wdt_bit)
#define DMT_C2K_WDT_BIT		(c2k_wdt_bit)

#define reg_read(addr)		__raw_readl(IOMEM(addr))
#define reg_write(addr, val)	mt_reg_sync_writel(val, addr)

#define read_cntpct()					\
	({						\
		register u64 cntpct;			\
		__asm__ __volatile__(			\
			"MRS	%0, CNTPCT_EL0\n\t"	\
			: "=r"(cntpct)			\
			:				\
			: "memory");			\
		cntpct;					\
	})

#define read_cntpctl()					\
	({						\
		register u32 cntpctl;			\
		__asm__ __volatile__(			\
			"MRS	%0, CNTP_CTL_EL0\n\t"	\
			: "=r"(cntpctl)			\
			:				\
			: "memory");			\
		cntpctl;				\
	})

#define write_cntpctl(cntpctl)				\
	do {						\
		register u32 t = (u32)cntpctl;		\
		__asm__ __volatile__(			\
			"MSR	CNTP_CTL_EL0, %0\n\t"	\
			:				\
			: "r"(t));			\
	} while (0)


struct core_context {
	volatile u64 timestamp[5];
	unsigned long timer_data[8];
};

struct cluster_context {
	struct core_context core[MAX_CORES] ____cacheline_aligned;
	unsigned long dbg_data[40];
};

struct system_context {
	struct cluster_context cluster[MAX_CLUSTER];
};

struct system_context dormant_data[1];
static int mt_dormant_initialized;


#define SPM_CORE_ID() core_idx()
#define SPM_IS_CPU_IRQ_OCCUR(core_id)					\
	({								\
		(!!(spm_read(SPM_WAKEUP_MISC) & ((0x101<<(core_id)))));	\
	})

#ifdef CONFIG_MTK_RAM_CONSOLE
#define DORMANT_LOG(cid, pattern) (sleep_aee_rec_cpu_dormant_va[cid] = pattern)
#else
#define DORMANT_LOG(cid, pattern)
#endif

#define read_mpidr()							\
	({								\
		register u64 ret;					\
		__asm__ __volatile__ (					\
			"MRS	%0, MPIDR_EL1\n\t"			\
			: "=r"(ret));					\
		ret;							\
	})

#define read_midr()							\
	({								\
		register u32 ret;					\
		__asm__ __volatile__ (					\
			"MRS	%0, MIDR_EL1\n\t"			\
			: "=r"(ret));					\
		ret;							\
	})


#define cpu_id()							\
	({								\
		(read_mpidr() & 0x0ff);					\
	})

#define cluster_id()							\
	({								\
		((read_mpidr() >> 8) & 0x0ff);				\
	})

#define core_idx()							\
	({								\
		int mpidr = read_mpidr();				\
		(((mpidr & (0x0ff << 8)) >> 6) | (mpidr & 0xff));	\
	})

inline int read_id(int *cpu_id, int *cluster_id)
{
	int mpidr = read_mpidr();

	*cpu_id = mpidr & 0x0f;
	*cluster_id = (mpidr >> 8) & 0x0f;

	return mpidr;
}

#define system_cluster(system, clusterid)	(&((struct system_context *)system)->cluster[clusterid])
#define cluster_core(cluster, cpuid)		(&((struct cluster_context *)cluster)->core[cpuid])

void *_get_data(int core_or_cluster)
{
	int cpuid, clusterid;
	struct cluster_context *cluster;
	struct core_context *core;

	read_id(&cpuid, &clusterid);

	cluster = system_cluster(dormant_data, clusterid);
	if (core_or_cluster == 1)
		return (void *)cluster;

	core = cluster_core(cluster, cpuid);

	return (void *)core;
}

#define GET_CORE_DATA()		((struct core_context *)_get_data(0))
#define GET_CLUSTER_DATA()	((struct cluster_context *)_get_data(1))

unsigned int *mt_save_generic_timer(unsigned int *container, int sw)
{
	__asm__ __volatile__ (
		"MRS	x3, CNTKCTL_EL1\n\t"
		"STR	x3, [%0, #0]\n\t"
		"MRS	x2, CNTP_CTL_EL0\n\t"
		"MRS	x3, CNTP_TVAL_EL0\n\t"
		"STP	x2, x3, [%0, #8]\n\t"
		"MRS	x2, CNTV_CTL_EL0\n\t"
		"MRS	x3, CNTV_TVAL_EL0\n\t"
		"STP	x2, x3, [%0, #24]!\n\t"
		: "+r"(container)
		: "r"(sw)
		: "r2", "r3");

	return container;
}

void mt_restore_generic_timer(unsigned int *container, int sw)
{
	__asm__ __volatile__ (
		"LDR	x3, [%0, #0]\n\t"
		"MSR	CNTKCTL_EL1, x3\n\t"
		"LDP	x2, x3, [%0, #8]\n\t"
		"MSR	CNTP_CTL_EL0, x2\n\t"
		"MSR	CNTP_TVAL_EL0, x3\n\t"
		"LDP	x2, x3, [%0, #24]\n\t"
		"MSR	CNTV_CTL_EL0, x2\n\t"
		"MSR	CNTV_TVAL_EL0, x3\n\t"
		:
		: "r"(container), "r"(sw)
		: "r2", "r3");
}

void stop_generic_timer(void)
{
	write_cntpctl(read_cntpctl() & ~1);
}

void start_generic_timer(void)
{
	write_cntpctl(read_cntpctl() | 1);
}

struct set_and_clear_regs {
	volatile unsigned int set[32], clear[32];
};

struct interrupt_distributor {
	volatile unsigned int control;			/* 0x000 */
	const unsigned int controller_type;
	const unsigned int implementer;
	const char padding1[116];
	volatile unsigned int security[32];		/* 0x080 */
	struct set_and_clear_regs enable;		/* 0x100 */
	struct set_and_clear_regs pending;		/* 0x200 */
	struct set_and_clear_regs active;		/* 0x300 */
	volatile unsigned int priority[256];		/* 0x400 */
	volatile unsigned int target[256];		/* 0x800 */
	volatile unsigned int configuration[64];	/* 0xC00 */
	const char padding3[256];			/* 0xD00 */
	volatile unsigned int non_security_access_control[64]; /* 0xE00 */
	volatile unsigned int software_interrupt;	/* 0xF00 */
	volatile unsigned int sgi_clr_pending[4];	/* 0xF10 */
	volatile unsigned int sgi_set_pending[4];	/* 0xF20 */
	const char padding4[176];

	unsigned const int peripheral_id[4];		/* 0xFE0 */
	unsigned const int primecell_id[4];		/* 0xFF0 */
};

static void restore_gic_spm_irq(unsigned long gic_distributor_address)
{
	struct interrupt_distributor *id = (struct interrupt_distributor *) gic_distributor_address;
	unsigned int backup;
	int i, j;

	backup = id->control;
	id->control = 0;

	/* Set the pending bit for spm wakeup source that is edge triggerd */
	if (reg_read(SPM_WAKEUP_STA) & WAKE_SRC_R12_KP_IRQ_B) {
		i = DMT_KP_IRQ_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_KP_IRQ_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}
	if (reg_read(SPM_WAKEUP_STA) & WAKE_SRC_R12_CONN_WDT_IRQ_B) {
		i = DMT_CONN_WDT_IRQ_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_CONN_WDT_IRQ_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}
	if (reg_read(SPM_WAKEUP_STA) & WAKE_SRC_R12_LOWBATTERY_IRQ_B) {
		i = DMT_LOWBATTERY_IRQ_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_LOWBATTERY_IRQ_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}
	if (reg_read(SPM_WAKEUP_STA) & WAKE_SRC_R12_MD1_WDT_B) {
		i = DMT_MD1_WDT_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_MD1_WDT_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}
	if (reg_read(SPM_WAKEUP_STA) & WAKE_SRC_R12_C2K_WDT_IRQ_B) {
		i = DMT_C2K_WDT_BIT / GIC_PRIVATE_SIGNALS;
		j = DMT_C2K_WDT_BIT % GIC_PRIVATE_SIGNALS;
		id->pending.set[i] |= (1 << j);
	}

	id->control = backup;
}

unsigned int __weak *mt_save_dbg_regs(unsigned int *p, unsigned int cpuid)
{
	return p;
}
void __weak mt_restore_dbg_regs(unsigned int *p, unsigned int cpuid) { }
void __weak mt_copy_dbg_regs(int to, int from) { }
void __weak mt_save_banked_registers(unsigned int *container) { }
void __weak mt_restore_banked_registers(unsigned int *container) { }

void mt_cpu_save(void)
{
	struct core_context *core;
	struct cluster_context *cluster;
	unsigned int *ret;
	unsigned int sleep_sta;
	int cpuid, clusterid;

	read_id(&cpuid, &clusterid);

	core = GET_CORE_DATA();

	ret = mt_save_generic_timer((unsigned int *)core->timer_data, 0x0);
	stop_generic_timer();

	if (clusterid == 0)
		sleep_sta = (spm_read(CPU_IDLE_STA) >> 10) & 0x0f;
	else
		sleep_sta = (spm_read(CPU_IDLE_STA) >> 14) & 0x0f;

	if ((sleep_sta | (1 << cpuid)) == 0x0f) { /* last core */
		cluster = GET_CLUSTER_DATA();
		ret = mt_save_dbg_regs((unsigned int *)cluster->dbg_data, cpuid + (clusterid * 4));
	}
}

void mt_cpu_restore(void)
{
	struct core_context *core;
	struct cluster_context *cluster;
	unsigned int sleep_sta;
	int cpuid, clusterid;

	read_id(&cpuid, &clusterid);

	core = GET_CORE_DATA();

	if (clusterid == 0)
		sleep_sta = (spm_read(CPU_IDLE_STA) >> 10) & 0x0f;
	else
		sleep_sta = (spm_read(CPU_IDLE_STA) >> 14) & 0x0f;

	sleep_sta = (sleep_sta | (1 << cpuid));

	if (sleep_sta == 0x0f) { /* first core */
		cluster = GET_CLUSTER_DATA();
		mt_restore_dbg_regs((unsigned int *)cluster->dbg_data, cpuid + (clusterid * 4));
	} else {
		int any = __builtin_ffs(~sleep_sta) - 1;

		mt_copy_dbg_regs(cpuid + (clusterid * 4), any + (clusterid * 4));
	}

	mt_restore_generic_timer((unsigned int *)core->timer_data, 0x0);
}

void mt_platform_save_context(int flags)
{
	mt_cpu_save();
}

void mt_platform_restore_context(int flags)
{
	mt_cpu_restore();

	if (IS_DORMANT_GIC_OFF(flags))
		restore_gic_spm_irq(DMT_GIC_DIST_BASE);
}

int mt_cpu_dormant_psci(unsigned long flags)
{
        int ret = 1;
	int cpuid, clusterid;

        struct psci_power_state pps = {
		.type = PSCI_POWER_STATE_TYPE_POWER_DOWN,
		.affinity_level = 0,
        };

	if (unlikely(IS_DORMANT_SNOOP_OFF(flags))) {
		pps.affinity_level = 1;
	}

	read_id(&cpuid, &clusterid);

        if (psci_ops.cpu_suspend) {
		DORMANT_LOG(clusterid * MAX_CORES + cpuid, 0x203);
                ret = psci_ops.cpu_suspend(pps, virt_to_phys(cpu_resume));
		DORMANT_LOG(clusterid * MAX_CORES + cpuid, 0x204);

                if (!ret) { //fixme
                        ret = 1;
                }
        }

        return ret;
}

static int mt_cpu_dormant_abort(unsigned long index)
{
	start_generic_timer();

	return 0;
}

int mt_cpu_dormant(unsigned long flags)
{
	int ret, i;
	int cpuid, clusterid;

	if (!mt_dormant_initialized)
		return MT_CPU_DORMANT_BYPASS;

	read_id(&cpuid, &clusterid);

	DORMANT_LOG(clusterid * MAX_CORES + cpuid, 0x101);

	BUG_ON(!irqs_disabled());

	/* to mark as cpu clobs vfp register.*/
	kernel_neon_begin();

	/* dormant break */
	if (IS_DORMANT_BREAK_CHECK(flags) && SPM_IS_CPU_IRQ_OCCUR(SPM_CORE_ID())) {
		ret = MT_CPU_DORMANT_BREAK_V(IRQ_PENDING_1);
		goto dormant_exit;
	}

	mt_platform_save_context(flags);

	DORMANT_LOG(clusterid * MAX_CORES + cpuid, 0x102);

	/* dormant break */
	if (IS_DORMANT_BREAK_CHECK(flags) && SPM_IS_CPU_IRQ_OCCUR(SPM_CORE_ID())) {
		mt_cpu_dormant_abort(flags);
		ret = MT_CPU_DORMANT_BREAK_V(IRQ_PENDING_2);
		goto dormant_exit;
	}

	DORMANT_LOG(clusterid * MAX_CORES + cpuid, 0x103);

	ret = cpu_suspend(flags);

	DORMANT_LOG(clusterid * MAX_CORES + cpuid, 0x601);

	switch (ret) {
	case 0: /* back from dormant reset */
		mt_platform_restore_context(flags);
		ret = MT_CPU_DORMANT_RESET;
		break;

	case 1: /* back from dormant abort, */
		mt_cpu_dormant_abort(flags);
		ret = MT_CPU_DORMANT_ABORT;
		break;
	case 2:
		mt_cpu_dormant_abort(flags);
		ret = MT_CPU_DORMANT_BREAK_V(IRQ_PENDING_3);
		break;
	default: /* back from dormant break, do nothing for return */
		dormant_dbg("EOPNOTSUPP\n");
		break;
	}

	DORMANT_LOG(clusterid * MAX_CORES + cpuid, 0x602);

	local_fiq_enable();

dormant_exit:

	kernel_neon_end();

	for (i = 0; i < MAX_CORES * MAX_CLUSTER; i++)
		DORMANT_LOG(i, 0);

	return ret & 0x0ff;
}

static int mt_dormant_dts_map(void)
{
	struct device_node *node;
	u32 kp_interrupt[3];
	u32 consys_interrupt[6];
	u32 auxadc_interrupt[3];
	u32 mdcldma_interrupt[9];
	u32 mdc2k_interrupt[6];

	node = of_find_compatible_node(NULL, NULL, GIC_NODE);
	if (!node) {
		dormant_err("error: cannot find node " GIC_NODE);
		BUG();
	}
	gic_id_base = (unsigned long)of_iomap(node, 0);
	if (!gic_id_base) {
		dormant_err("error: cannot iomap " GIC_NODE);
		BUG();
	}
	of_node_put(node);

	node = of_find_compatible_node(NULL, NULL, KP_NODE);
	if (!node) {
		dormant_err("error: cannot find node " KP_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts", kp_interrupt, ARRAY_SIZE(kp_interrupt))) {
		dormant_err("error: cannot property_read " KP_NODE);
		BUG();
	}
	kp_irq_bit = ((1 - kp_interrupt[0]) << 5) + kp_interrupt[1]; /* irq[0] = 0 => spi */
	of_node_put(node);
	dormant_dbg("kp_irq_bit = %u\n", kp_irq_bit);

	node = of_find_compatible_node(NULL, NULL, CONSYS_NODE);
	if (!node) {
		dormant_err("error: cannot find node " CONSYS_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts",
				       consys_interrupt, ARRAY_SIZE(consys_interrupt))) {
		dormant_err("error: cannot property_read " CONSYS_NODE);
		BUG();
	}
	conn_wdt_irq_bit = ((1 - consys_interrupt[3]) << 5) + consys_interrupt[4]; /* irq[0] = 0 => spi */
	of_node_put(node);
	dormant_dbg("conn_wdt_irq_bit = %u\n", conn_wdt_irq_bit);

	node = of_find_compatible_node(NULL, NULL, AUXADC_NODE);
	if (!node) {
		dormant_err("error: cannot find node " AUXADC_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts",
				       auxadc_interrupt, ARRAY_SIZE(auxadc_interrupt))) {
		dormant_err("error: cannot property_read " AUXADC_NODE);
		BUG();
	}
	lowbattery_irq_bit = ((1 - auxadc_interrupt[0]) << 5) + auxadc_interrupt[1]; /* irq[0] = 0 => spi */
	of_node_put(node);
	dormant_dbg("lowbattery_irq_bit = %u\n", lowbattery_irq_bit);

	node = of_find_compatible_node(NULL, NULL, MDCLDMA_NODE);
	if (!node) {
		dormant_err("error: cannot find node " MDCLDMA_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts",
				       mdcldma_interrupt, ARRAY_SIZE(mdcldma_interrupt))) {
		dormant_err("error: cannot property_read " MDCLDMA_NODE);
		BUG();
	}
	md1_wdt_bit = ((1 - mdcldma_interrupt[6]) << 5) + mdcldma_interrupt[7]; /* irq[0] = 0 => spi */
	of_node_put(node);
	dormant_dbg("md1_wdt_bit = %u\n", md1_wdt_bit);

	node = of_find_compatible_node(NULL, NULL, MDC2K_NODE);
	if (!node) {
		dormant_err("error: cannot find node " MDC2K_NODE);
		BUG();
	}
	if (of_property_read_u32_array(node, "interrupts",
				       mdc2k_interrupt, ARRAY_SIZE(mdc2k_interrupt))) {
		dormant_err("error: cannot property_read " MDC2K_NODE);
		BUG();
	}
	c2k_wdt_bit = ((1 - mdc2k_interrupt[3]) << 5) + mdc2k_interrupt[4]; /* irq[0] = 0 => spi */
	of_node_put(node);
	dormant_dbg("c2k_wdt_bit = %u\n", c2k_wdt_bit);

	return 0;
}

dram_info_t *g_dram_dummy_read = NULL;

static int dt_scan_dram_info(unsigned long node, const char *uname, int depth, void *data)
{
	char *type = of_get_flat_dt_prop(node, "device_type", NULL);
	__be32 *reg, *endp;
	unsigned long l;

	/* We are scanning "memory" nodes only */
	if (type == NULL) {
		/*
		* The longtrail doesn't have a device_type on the
		* /memory node, so look for the node called /memory@0.
		*/
		if (depth != 1 || strcmp(uname, "memory@0") != 0)
			return 0;
	} else if (strcmp(type, "memory") != 0)
		return 0;

	reg = of_get_flat_dt_prop(node, "reg", &l);
	if (reg == NULL)
		return 0;

	endp = reg + (l / sizeof(__be32));
	if (node) {
		/* orig_dram_info */
		g_dram_dummy_read = (dram_info_t *)of_get_flat_dt_prop(node, "orig_dram_info", NULL);
	}

	pr_warn("[DRAMC] dram info dram rank number = %d\n", g_dram_dummy_read->rank_num);
	pr_warn("[DRAMC] dram info dram rank0 base = 0x%llx\n", g_dram_dummy_read->rank_info[0].start);
	pr_warn("[DRAMC] dram info dram rank1 base = 0x%llx\n", g_dram_dummy_read->rank_info[0].start + g_dram_dummy_read->rank_info[0].size);

	return node;
}

int mt_cpu_dormant_init(void)
{
	int cpuid, clusterid;

	read_id(&cpuid, &clusterid);

	if (mt_dormant_initialized == 1)
		return MT_CPU_DORMANT_BYPASS;

	mt_dormant_dts_map();

#ifdef CONFIG_MTK_RAM_CONSOLE
	sleep_aee_rec_cpu_dormant_va = aee_rr_rec_cpu_dormant();
	sleep_aee_rec_cpu_dormant = aee_rr_rec_cpu_dormant_pa();

	BUG_ON(!sleep_aee_rec_cpu_dormant_va || !sleep_aee_rec_cpu_dormant);

	kernel_smc_msg(0, 2, (phys_addr_t)sleep_aee_rec_cpu_dormant);

	dormant_dbg("init aee_rec_cpu_dormant: va:%p pa:%p\n",
		    sleep_aee_rec_cpu_dormant_va, sleep_aee_rec_cpu_dormant);
#endif

	if (of_scan_flat_dt(dt_scan_dram_info, NULL) > 0) {
		kernel_smc_msg(0, 3, g_dram_dummy_read->rank_info[0].start);
		kernel_smc_msg(0, 4, g_dram_dummy_read->rank_info[0].start + g_dram_dummy_read->rank_info[0].size);

		kernel_smc_msg(1, 1, 0);
	} else {
		pr_err("[DRAMC]can't find dt_scan_dram_info\n");
	}

	mt_dormant_initialized = 1;

	return 0;
}
