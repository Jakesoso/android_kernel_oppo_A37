#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/of_fdt.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include <mach/irqs.h>
#include <mach/mt_cirq.h>
#include <mach/mt_spm_idle.h>
#include <mach/mt_cpuidle.h>
#ifdef CONFIG_MTK_WD_KICKER
#include <mach/wd_api.h>
#endif
#include <mach/mt_gpt.h>
#include <mach/mt_cpufreq.h>
#include <mach/mt_ccci_common.h>
#include <mach/md32_helper.h>
#include <mach/mt_spm_misc.h>
#include <mach/upmu_common.h>
#include <mach/mt_clkbuf_ctl.h>

#include "mt_spm_internal.h"
#include "mt_spm_pmic_wrap.h"


/**************************************
 * only for internal debug
 **************************************/
#define DPIDLE_TAG     "[DP] "
#define dpidle_dbg(fmt, args...)	pr_debug(DPIDLE_TAG fmt, ##args)

#define SPM_PWAKE_EN            0
#ifdef VENDOR_EDIT
//Modified by Tong.han@Bsp.Group.boot added for disable Jtag reset sig during WD check ,2016-4-17
#define SPM_BYPASS_SYSPWREQ         1	/* JTAG is used */
#else
#define SPM_BYPASS_SYSPWREQ         0	/* JTAG is used */
#endif/*VENDOR_EDIT*/

#if defined(CONFIG_MICROTRUST_TEE_SUPPORT)
#define WAKE_SRC_FOR_DPIDLE             \
	(WAKE_SRC_R12_KP_IRQ_B |            \
	WAKE_SRC_R12_PCM_TIMER |            \
	WAKE_SRC_R12_APXGPT1_EVENT_B |      \
	WAKE_SRC_R12_CONN2AP_SPM_WAKEUP_B | \
	WAKE_SRC_R12_EINT_EVENT_B |         \
	WAKE_SRC_R12_CONN_WDT_IRQ_B |       \
	WAKE_SRC_R12_CCIF0_EVENT_B |        \
	WAKE_SRC_R12_USB_CDSC_B |           \
	WAKE_SRC_R12_USB_POWERDWN_B |       \
	WAKE_SRC_R12_C2K_WDT_IRQ_B |        \
	WAKE_SRC_R12_EINT_EVENT_SECURE_B |  \
	WAKE_SRC_R12_CCIF1_EVENT_B |        \
	WAKE_SRC_R12_AFE_IRQ_MCU_B |        \
	WAKE_SRC_R12_SYS_CIRQ_IRQ_B |       \
	WAKE_SRC_R12_MD1_WDT_B |            \
	WAKE_SRC_R12_CLDMA_EVENT_B)
#else
#define WAKE_SRC_FOR_DPIDLE             \
	(WAKE_SRC_R12_KP_IRQ_B |            \
	WAKE_SRC_R12_PCM_TIMER |            \
	WAKE_SRC_R12_APXGPT1_EVENT_B |      \
	WAKE_SRC_R12_CONN2AP_SPM_WAKEUP_B | \
	WAKE_SRC_R12_EINT_EVENT_B |         \
	WAKE_SRC_R12_CONN_WDT_IRQ_B |       \
	WAKE_SRC_R12_CCIF0_EVENT_B |        \
	WAKE_SRC_R12_USB_CDSC_B |           \
	WAKE_SRC_R12_USB_POWERDWN_B |       \
	WAKE_SRC_R12_C2K_WDT_IRQ_B |        \
	WAKE_SRC_R12_EINT_EVENT_SECURE_B |  \
	WAKE_SRC_R12_CCIF1_EVENT_B |        \
	WAKE_SRC_R12_AFE_IRQ_MCU_B |        \
	WAKE_SRC_R12_SYS_CIRQ_IRQ_B |       \
	WAKE_SRC_R12_MD1_WDT_B |            \
	WAKE_SRC_R12_CLDMA_EVENT_B |        \
	WAKE_SRC_R12_SEJ_WDT_GPT_B)
#endif

#define WAKE_SRC_FOR_MD32  0

#define I2C_CHANNEL 2

#define spm_is_wakesrc_invalid(wakesrc)     (!!((u32)(wakesrc) & 0xc0003803))

#define CA70_BUS_CONFIG          0xF020002C  /* (CA7MCUCFG_BASE + 0x1C) - 0x1020011c */
#define CA71_BUS_CONFIG          0xF020022C  /* (CA7MCUCFG_BASE + 0x1C) - 0x1020011c */

#ifdef CONFIG_MTK_RAM_CONSOLE
#define SPM_AEE_RR_REC 1
#else
#define SPM_AEE_RR_REC 0
#endif
#define SPM_USE_TWAM_DEBUG	0

#define	DPIDLE_LOG_PRINT_TIMEOUT_CRITERIA	20
#define	DPIDLE_LOG_DISCARD_CRITERIA			5000	/* ms */

#define reg_read(addr)         __raw_readl(IOMEM(addr))

#if defined(CONFIG_OF)
#define MCUCFG_NODE "mediatek,MCUCFG"
static unsigned long mcucfg_base;
static unsigned long mcucfg_phys_base;
#undef MCUCFG_BASE
#define MCUCFG_BASE (mcucfg_base)

static void __iomem *apmixedsys_base;

#define APMIXED_REG(ofs)	(apmixedsys_base + ofs)

#define AP_PLL_CON0			APMIXED_REG(0x0)

#else /* #if defined (CONFIG_OF) */
#undef MCUCFG_BASE
#define MCUCFG_BASE 0xF0200000 /* 0x1020_0000 */

#endif /* #if defined (CONFIG_OF) */

/* MCUCFG registers */
#define MP0_AXI_CONFIG		(MCUCFG_BASE + 0x2C)
#define MP0_AXI_CONFIG_PHYS	(mcucfg_phys_base + 0x2C)
#define MP1_AXI_CONFIG		(MCUCFG_BASE + 0x22C)
#define MP1_AXI_CONFIG_PHYS	(mcucfg_phys_base + 0x22C)
#define ACINACTM		(1 << 4)

#if defined(CONFIG_ARM_PSCI) || defined(CONFIG_MTK_PSCI)
#include <mach/mt_secure_api.h>
#define MCUSYS_SMC_WRITE(addr, val)  mcusys_smc_write_phy(addr##_PHYS, val)
#else
#define MCUSYS_SMC_WRITE(addr, val)  mcusys_smc_write(addr, val)
#endif

#if SPM_AEE_RR_REC
enum spm_deepidle_step {
	SPM_DEEPIDLE_ENTER = 0x00000001,
	SPM_DEEPIDLE_ENTER_UART_SLEEP = 0x00000003,
	SPM_DEEPIDLE_ENTER_WFI = 0x000000ff,
	SPM_DEEPIDLE_LEAVE_WFI = 0x000001ff,
	SPM_DEEPIDLE_ENTER_UART_AWAKE = 0x000003ff,
	SPM_DEEPIDLE_LEAVE = 0x000007ff
};
#endif

/* #define PMIC_CLK_SRC_BY_SRCCLKEN_IN1 */

/* please put firmware to vendor/mediatek/proprietary/hardware/spm/mtxxxx/ */
#if 0
/**********************************************************
 * PCM code for deep idle
 **********************************************************/
static const u32 dpidle_binary[] = {
	0x81f48407, 0x80328400, 0x80318400, 0xe8208000, 0x10006354, 0xffff1fff,
	0xe8208000, 0x10001108, 0x00000000, 0x1b80001f, 0x20000034, 0xe8208000,
	0x10006b04, 0x00000000, 0xc28032c0, 0x1290041f, 0x1b00001f, 0x7ffcf7ff,
	0xf0000000, 0x17c07c1f, 0x1b00001f, 0x3ffce7ff, 0x1b80001f, 0x20000004,
	0xd820038c, 0x17c07c1f, 0xd0000620, 0x17c07c1f, 0xe8208000, 0x10001108,
	0x00000002, 0x1b80001f, 0x20000034, 0xe8208000, 0x10006354, 0xffffffff,
	0xe8208000, 0x10006b04, 0x00000001, 0xc28032c0, 0x1290841f, 0xa0118400,
	0xa0128400, 0xe8208000, 0x10006b04, 0x00000004, 0x1b00001f, 0x3ffcefff,
	0xa1d48407, 0xf0000000, 0x17c07c1f, 0x81489801, 0xd80007c5, 0x17c07c1f,
	0x81419801, 0xd80007c5, 0x17c07c1f, 0x1a00001f, 0x10006604, 0xe2200004,
	0xc0c033c0, 0x10807c1f, 0x81411801, 0xd80009e5, 0x17c07c1f, 0x18c0001f,
	0x1000f644, 0x1910001f, 0x1000f644, 0xa1140404, 0xe0c00004, 0x1b80001f,
	0x20000208, 0x18c0001f, 0x10006240, 0xe0e00016, 0xe0e0001e, 0xe0e0000e,
	0xe0e0000f, 0x81481801, 0xd8200d25, 0x17c07c1f, 0x18c0001f, 0x10004828,
	0x1910001f, 0x10004828, 0x89000004, 0x3fffffff, 0xe0c00004, 0x18c0001f,
	0x100041dc, 0x1910001f, 0x100041dc, 0x89000004, 0x3fffffff, 0xe0c00004,
	0x18c0001f, 0x1000f63c, 0x1910001f, 0x1000f63c, 0x89000004, 0xfffffff9,
	0xe0c00004, 0xc28032c0, 0x1294841f, 0x803e0400, 0x1b80001f, 0x20000050,
	0x803e8400, 0x803f0400, 0x803f8400, 0x1b80001f, 0x20000208, 0x803d0400,
	0x1b80001f, 0x20000034, 0x80380400, 0xa01d8400, 0x1b80001f, 0x20000034,
	0x803d8400, 0x803b0400, 0x1b80001f, 0x20000158, 0x81481801, 0xd8201005,
	0x17c07c1f, 0xa01d8400, 0x80340400, 0x81481801, 0xd8201145, 0x17c07c1f,
	0x18c0001f, 0x1000f698, 0x1910001f, 0x1000f698, 0xa1120404, 0xe0c00004,
	0x80310400, 0x81481801, 0xd8201265, 0x17c07c1f, 0xe8208000, 0x10000044,
	0x00000200, 0xd00012c0, 0x17c07c1f, 0xe8208000, 0x10000044, 0x00000100,
	0xe8208000, 0x10000004, 0x00000002, 0x1b80001f, 0x20000068, 0x1b80001f,
	0x2000000a, 0x81481801, 0xd82017a5, 0x17c07c1f, 0x18c0001f, 0x1000f640,
	0x1910001f, 0x1000f640, 0x81200404, 0xe0c00004, 0xa1000404, 0xe0c00004,
	0x18c0001f, 0x10004828, 0x1910001f, 0x10004828, 0xa9000004, 0xc0000000,
	0xe0c00004, 0x18c0001f, 0x100041dc, 0x1910001f, 0x100041dc, 0xa9000004,
	0xc0000000, 0xe0c00004, 0x18c0001f, 0x1000f63c, 0x1910001f, 0x1000f63c,
	0xa9000004, 0x00000006, 0xe0c00004, 0x18c0001f, 0x10006240, 0xe0e0000d,
	0x81fa0407, 0x18c0001f, 0x100040f4, 0x1910001f, 0x100040f4, 0xa11c8404,
	0xe0c00004, 0x813c8404, 0xe0c00004, 0x1b80001f, 0x20000100, 0x81f08407,
	0xe8208000, 0x10006354, 0xfff01b47, 0xa1d80407, 0xa1dc0407, 0xa1de8407,
	0xa1df0407, 0xc28032c0, 0x1291041f, 0x1b00001f, 0xbffce7ff, 0xf0000000,
	0x17c07c1f, 0x1b80001f, 0x20000fdf, 0x1a50001f, 0x10006608, 0x80c9a401,
	0x810aa401, 0x10918c1f, 0xa0939002, 0x80ca2401, 0x810ba401, 0xa09c0c02,
	0xa0979002, 0x8080080d, 0xd8201e42, 0x17c07c1f, 0x1b00001f, 0x3ffce7ff,
	0x1b80001f, 0x20000004, 0xd80027ec, 0x17c07c1f, 0x1b00001f, 0xbffce7ff,
	0xd00027e0, 0x17c07c1f, 0x81f80407, 0x81fc0407, 0x81fe8407, 0x81ff0407,
	0x1880001f, 0x10006320, 0xc0c02b60, 0xe080000f, 0xd8001d03, 0x17c07c1f,
	0xe080001f, 0xa1da0407, 0x81481801, 0xd82020c5, 0x17c07c1f, 0xe8208000,
	0x10000048, 0x00000300, 0xd0002120, 0x17c07c1f, 0xe8208000, 0x10000048,
	0x00000100, 0xe8208000, 0x10000004, 0x00000002, 0x1b80001f, 0x20000068,
	0xa0110400, 0x81481801, 0xd8202325, 0x17c07c1f, 0x18c0001f, 0x1000f698,
	0x1910001f, 0x1000f698, 0x81320404, 0xe0c00004, 0x803d8400, 0xa0140400,
	0xa01b0400, 0xa0180400, 0xa01d0400, 0xa01f8400, 0xa01f0400, 0xa01e8400,
	0xa01e0400, 0x1b80001f, 0x20000104, 0x81411801, 0xd8002605, 0x17c07c1f,
	0x18c0001f, 0x10006240, 0xc0c02aa0, 0x17c07c1f, 0x18c0001f, 0x1000f644,
	0x1910001f, 0x1000f644, 0x81340404, 0xe0c00004, 0x81489801, 0xd8002765,
	0x17c07c1f, 0x81419801, 0xd8002765, 0x17c07c1f, 0x1a00001f, 0x10006604,
	0xe2200005, 0xc0c033c0, 0x10807c1f, 0xc28032c0, 0x1291841f, 0x1b00001f,
	0x7ffcf7ff, 0xf0000000, 0x17c07c1f, 0x1900001f, 0x10006830, 0xe1000003,
	0xf0000000, 0x17c07c1f, 0xe0f07f16, 0x1380201f, 0xe0f07f1e, 0x1380201f,
	0xe0f07f0e, 0x1b80001f, 0x20000104, 0xe0f07f0c, 0xe0f07f0d, 0xe0f07e0d,
	0xe0f07c0d, 0xe0f0780d, 0xe0f0700d, 0xf0000000, 0x17c07c1f, 0xe0f07f0d,
	0xe0f07f0f, 0xe0f07f1e, 0xe0f07f12, 0xf0000000, 0x17c07c1f, 0xa1d08407,
	0x1b80001f, 0x20000080, 0x80eab401, 0x1a00001f, 0x10006814, 0xe2000003,
	0xf0000000, 0x17c07c1f, 0x81429801, 0xd8002e45, 0x17c07c1f, 0x18c0001f,
	0x65930005, 0x1900001f, 0x10006830, 0xe1000003, 0xe8208000, 0x10006834,
	0x00000000, 0xe8208000, 0x10006834, 0x00000001, 0xf0000000, 0x17c07c1f,
	0xa1d10407, 0x1b80001f, 0x20000020, 0xf0000000, 0x17c07c1f, 0xa1d00407,
	0x1b80001f, 0x20000100, 0x80ea3401, 0x1a00001f, 0x10006814, 0xe2000003,
	0xf0000000, 0x17c07c1f, 0xe0e0000f, 0xe0e0000e, 0xe0e0001e, 0xe0e00012,
	0xf0000000, 0x17c07c1f, 0xd800324a, 0x17c07c1f, 0xe0e00016, 0xe0e0001e,
	0x1380201f, 0xe0e0001f, 0xe0e0001d, 0xe0e0000d, 0xd0003280, 0x17c07c1f,
	0xe0e03301, 0xe0e03101, 0xf0000000, 0x17c07c1f, 0x18c0001f, 0x10006b6c,
	0x1910001f, 0x10006b6c, 0xa1002804, 0xe0c00004, 0xf0000000, 0x17c07c1f,
	0x18d0001f, 0x10006604, 0x10cf8c1f, 0xd82033c3, 0x17c07c1f, 0xf0000000,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f, 0x17c07c1f,
	0x17c07c1f, 0x17c07c1f, 0x1840001f, 0x00000001, 0x1990001f, 0x10006b08,
	0xe8208000, 0x10006b6c, 0x00000000, 0x1b00001f, 0x2ffce7ff, 0x1b80001f,
	0x500f0000, 0xe8208000, 0x10006354, 0xfff01b47, 0xc0c02e80, 0x81401801,
	0xd8004765, 0x17c07c1f, 0x81f60407, 0x18c0001f, 0x10006200, 0xc0c063c0,
	0x12807c1f, 0xe8208000, 0x1000625c, 0x00000001, 0x1890001f, 0x1000625c,
	0x81040801, 0xd8204344, 0x17c07c1f, 0xc0c063c0, 0x1280041f, 0x18c0001f,
	0x10006208, 0xc0c063c0, 0x12807c1f, 0x1b80001f, 0x20000003, 0xe8208000,
	0x10006248, 0x00000000, 0x1890001f, 0x10006248, 0x81040801, 0xd8004544,
	0x17c07c1f, 0xc0c063c0, 0x1280041f, 0x18c0001f, 0x10006290, 0xe0e0004f,
	0xc0c063c0, 0x1280041f, 0xe8208000, 0x10006404, 0x00003101, 0xc28032c0,
	0x1292041f, 0x1b00001f, 0x2ffce7ff, 0x1b80001f, 0x30000004, 0x8880000c,
	0x2ffce7ff, 0xd8005dc2, 0x17c07c1f, 0xe8208000, 0x10006294, 0x0003ffff,
	0x18c0001f, 0x10006294, 0xe0e03fff, 0xe0e003ff, 0x81449801, 0xd8004c65,
	0x17c07c1f, 0x1a00001f, 0x10006604, 0xe2200006, 0xc0c033c0, 0x17c07c1f,
	0x1a00001f, 0x10006604, 0x81491801, 0xd8004c05, 0x17c07c1f, 0xc28032c0,
	0x1294041f, 0xe2200003, 0xc0c06820, 0x12807c1f, 0xc0c033c0, 0x17c07c1f,
	0xd0004c60, 0x17c07c1f, 0xe2200003, 0xc0c033c0, 0x17c07c1f, 0x81489801,
	0xd8204f45, 0x17c07c1f, 0x81419801, 0xd8004f45, 0x17c07c1f, 0x1a00001f,
	0x10006604, 0xe2200000, 0xc0c033c0, 0x17c07c1f, 0xc0c06c00, 0x17c07c1f,
	0xe2200001, 0xc0c033c0, 0x17c07c1f, 0xc0c06c00, 0x17c07c1f, 0xe2200005,
	0xc0c033c0, 0x17c07c1f, 0xc0c06c00, 0x17c07c1f, 0xc0c06760, 0x17c07c1f,
	0xa1d38407, 0xa1d98407, 0xa0108400, 0xa0120400, 0xa0148400, 0xa0150400,
	0xa0158400, 0xa01b8400, 0xa01c0400, 0xa01c8400, 0xa0188400, 0xa0190400,
	0xa0198400, 0xe8208000, 0x10006310, 0x0b1600f8, 0x1b00001f, 0xbffce7ff,
	0x1b80001f, 0x90100000, 0x1240301f, 0x80c28001, 0xc8c00003, 0x17c07c1f,
	0x80c10001, 0xc8c00663, 0x17c07c1f, 0x1b00001f, 0x2ffce7ff, 0x18c0001f,
	0x10006294, 0xe0e007fe, 0xe0e00ffc, 0xe0e01ff8, 0xe0e03ff0, 0xe0e03fe0,
	0xe0e03fc0, 0x1b80001f, 0x20000020, 0xe8208000, 0x10006294, 0x0003ffc0,
	0xe8208000, 0x10006294, 0x0003fc00, 0x80388400, 0x80390400, 0x80398400,
	0x1b80001f, 0x20000300, 0x803b8400, 0x803c0400, 0x803c8400, 0x1b80001f,
	0x20000300, 0x80348400, 0x80350400, 0x80358400, 0x1b80001f, 0x20000104,
	0x80308400, 0x80320400, 0x81f38407, 0x81f98407, 0x81f90407, 0x81f40407,
	0x81489801, 0xd8205aa5, 0x17c07c1f, 0x81419801, 0xd8005aa5, 0x17c07c1f,
	0x1a00001f, 0x10006604, 0xe2200001, 0xc0c033c0, 0x17c07c1f, 0xc0c06c00,
	0x17c07c1f, 0xe2200000, 0xc0c033c0, 0x17c07c1f, 0xc0c06c00, 0x17c07c1f,
	0xe2200004, 0xc0c033c0, 0x17c07c1f, 0xc0c06c00, 0x17c07c1f, 0x81449801,
	0xd8005dc5, 0x17c07c1f, 0x1a00001f, 0x10006604, 0x81491801, 0xd8005c85,
	0x17c07c1f, 0xe2200002, 0xc0c06820, 0x1280041f, 0xc0c033c0, 0x17c07c1f,
	0xd0005ce0, 0x17c07c1f, 0xe2200002, 0xc0c033c0, 0x17c07c1f, 0x1a00001f,
	0x10006604, 0xe2200007, 0xc0c033c0, 0x17c07c1f, 0x1b80001f, 0x200016a8,
	0x81401801, 0xd8006325, 0x17c07c1f, 0xe8208000, 0x10006404, 0x00000101,
	0x18c0001f, 0x10006290, 0x1212841f, 0xc0c06540, 0x12807c1f, 0xc0c06540,
	0x1280041f, 0x18c0001f, 0x10006208, 0x1212841f, 0xc0c06540, 0x12807c1f,
	0xe8208000, 0x10006248, 0x00000001, 0x1890001f, 0x10006248, 0x81040801,
	0xd8206064, 0x17c07c1f, 0xc0c06540, 0x1280041f, 0x18c0001f, 0x10006200,
	0x1212841f, 0xc0c06540, 0x12807c1f, 0xe8208000, 0x1000625c, 0x00000000,
	0x1890001f, 0x1000625c, 0x81040801, 0xd8006244, 0x17c07c1f, 0xc0c06540,
	0x1280041f, 0x19c0001f, 0x61415820, 0x1ac0001f, 0x55aa55aa, 0xf0000000,
	0xd800646a, 0x17c07c1f, 0xe2e0004f, 0xe2e0006f, 0xe2e0002f, 0xd820650a,
	0x17c07c1f, 0xe2e0002e, 0xe2e0003e, 0xe2e00032, 0xf0000000, 0x17c07c1f,
	0xd800660a, 0x17c07c1f, 0xe2e00036, 0xe2e0003e, 0x1380201f, 0xe2e0003c,
	0xd820672a, 0x17c07c1f, 0x1380201f, 0xe2e0007c, 0x1b80001f, 0x20000003,
	0xe2e0005c, 0xe2e0004c, 0xe2e0004d, 0xf0000000, 0x17c07c1f, 0xa1d40407,
	0x1391841f, 0xa1d90407, 0x1392841f, 0xf0000000, 0x17c07c1f, 0xe8208000,
	0x10059c14, 0x00000002, 0xe8208000, 0x10059c20, 0x00000001, 0xe8208000,
	0x10059c04, 0x000000d6, 0x1a00001f, 0x10059c00, 0xd8206aea, 0x17c07c1f,
	0xe2200088, 0xe2200002, 0xe8208000, 0x10059c24, 0x00000001, 0x1b80001f,
	0x20000158, 0xd0006bc0, 0x17c07c1f, 0xe2200088, 0xe2200000, 0xe8208000,
	0x10059c24, 0x00000001, 0x1b80001f, 0x20000158, 0xf0000000, 0x17c07c1f,
	0x1880001f, 0x0000001d, 0x814a1801, 0xd8006e65, 0x17c07c1f, 0x81499801,
	0xd8006f65, 0x17c07c1f, 0x814a9801, 0xd8007065, 0x17c07c1f, 0x18d0001f,
	0x40000000, 0x18d0001f, 0x40000000, 0xd8006d62, 0x00a00402, 0xd0007160,
	0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f, 0x80000000, 0xd8006e62,
	0x00a00402, 0xd0007160, 0x17c07c1f, 0x18d0001f, 0x40000000, 0x18d0001f,
	0x60000000, 0xd8006f62, 0x00a00402, 0xd0007160, 0x17c07c1f, 0x18d0001f,
	0x40000000, 0x18d0001f, 0xc0000000, 0xd8007062, 0x00a00402, 0xd0007160,
	0x17c07c1f, 0xf0000000, 0x17c07c1f
};
static struct pcm_desc dpidle_pcm = {
	.version	= "pcm_deepidle_v19.15_20140731-exp1-no_6311_low_power_mode",
	.base		= dpidle_binary,
	.size		= 909,
	.sess		= 2,
	.replace	= 0,
	.vec0		= EVENT_VEC(11, 1, 0, 0),	/* FUNC_26M_WAKEUP */
	.vec1		= EVENT_VEC(12, 1, 0, 20),	/* FUNC_26M_SLEEP */
	.vec2		= EVENT_VEC(30, 1, 0, 51),	/* FUNC_APSRC_WAKEUP */
	.vec3		= EVENT_VEC(31, 1, 0, 217),	/* FUNC_APSRC_SLEEP */
};
#endif

static struct pwr_ctrl dpidle_ctrl = {
	.wake_src			= WAKE_SRC_FOR_DPIDLE,
	.wake_src_md32		= WAKE_SRC_FOR_MD32,
	.r0_ctrl_en			= 1,
	.r7_ctrl_en			= 1,
	.infra_dcm_lock		= 1,
	.wfi_op				= WFI_OP_AND,

	/* SPM_AP_STANDBY_CON */
	.mp0top_idle_mask = 0,
	.mp1top_idle_mask = 0,
	.mcusys_idle_mask = 0,
	.md_ddr_dbc_en = 0,
	.md1_req_mask_b = 1,
	.md2_req_mask_b = 0,
	.scp_req_mask_b = 0,
	.lte_mask_b = 0,
	.md_apsrc1_sel = 0,
	.md_apsrc0_sel = 0,
	.conn_mask_b = 1,
	.conn_apsrc_sel = 0,

	/* SPM_SRC_REQ */
	.spm_apsrc_req = 0,
	.spm_f26m_req = 0,
	.spm_lte_req = 0,
	.spm_infra_req = 0,
	.spm_vrf18_req = 0,
	.spm_dvfs_req = 0,
	.spm_dvfs_force_down = 0,
	.spm_ddren_req = 0,
	.cpu_md_dvfs_sop_force_on = 0,

	/* SPM_SRC_MASK */
	.ccif0_to_md_mask_b = 1,
	.ccif0_to_ap_mask_b = 1,
	.ccif1_to_md_mask_b = 1,
	.ccif1_to_ap_mask_b = 1,
	.ccifmd_md1_event_mask_b = 1,
	.ccifmd_md2_event_mask_b = 1,
	.vsync_mask_b = 0,	/* 5bit */
	.md_srcclkena_0_infra_mask_b = 0,
	.md_srcclkena_1_infra_mask_b = 0,
	.conn_srcclkena_infra_mask_b = 0,
	.md32_srcclkena_infra_mask_b = 0,
	.srcclkeni_infra_mask_b = 0,
	.md_apsrcreq_0_infra_mask_b = 1,
	.md_apsrcreq_1_infra_mask_b = 0,
	.conn_apsrcreq_infra_mask_b = 1,
	.md32_apsrcreq_infra_mask_b = 0,
	.md_ddr_en_0_mask_b = 1,
	.md_ddr_en_1_mask_b = 0,
	.md_vrf18_req_0_mask_b = 1,
	.md_vrf18_req_1_mask_b = 0,
	.emi_bw_dvfs_req_mask = 1,
	.md_srcclkena_0_dvfs_req_mask_b = 0,
	.md_srcclkena_1_dvfs_req_mask_b = 0,
	.conn_srcclkena_dvfs_req_mask_b = 0,

	/* SPM_SRC2_MASK */
	.dvfs_halt_mask_b = 0x1f,	/* 5bit */
	.vdec_req_mask_b = 0,
	.gce_req_mask_b = 0,
	.cpu_md_dvfs_erq_merge_mask_b = 0,
	.md1_ddr_en_dvfs_halt_mask_b = 0,
	.md2_ddr_en_dvfs_halt_mask_b = 0,
	.vsync_dvfs_halt_mask_b = 0,	/* 5bit */
	.conn_ddr_en_mask_b = 1,
	.disp_req_mask_b = 0,
	.disp1_req_mask_b = 0,
	.mfg_req_mask_b = 0,
	.c2k_ps_rccif_wake_mask_b = 1,
	.c2k_l1_rccif_wake_mask_b = 1,
	.ps_c2k_rccif_wake_mask_b = 1,
	.l1_c2k_rccif_wake_mask_b = 1,
	.sdio_on_dvfs_req_mask_b = 0,
	.emi_boost_dvfs_req_mask_b = 0,
	.cpu_md_emi_dvfs_req_prot_dis = 0,

	/* SPM_CLK_CON */
	.srclkenai_mask = 1,

	.mp1_cpu0_wfi_en	= 1,
	.mp1_cpu1_wfi_en	= 1,
	.mp1_cpu2_wfi_en	= 1,
	.mp1_cpu3_wfi_en	= 1,
	.mp0_cpu0_wfi_en	= 1,
	.mp0_cpu1_wfi_en	= 1,
	.mp0_cpu2_wfi_en	= 1,
	.mp0_cpu3_wfi_en	= 1,

#if SPM_BYPASS_SYSPWREQ
	.syspwreq_mask = 1,
#endif
};

struct spm_lp_scen __spm_dpidle = {
/*	.pcmdesc	= &dpidle_pcm, */
	.pwrctrl	= &dpidle_ctrl,
};

static unsigned int dpidle_log_discard_cnt;
static unsigned int dpidle_log_print_prev_time;

static long int idle_get_current_time_ms(void)
{
	struct timeval t;
	do_gettimeofday(&t);
	return ((t.tv_sec & 0xFFF) * 1000000 + t.tv_usec) / 1000;
}

static void spm_trigger_wfi_for_dpidle(struct pwr_ctrl *pwrctrl)
{
	u32 v0, v1;

	if (is_cpu_pdn(pwrctrl->pcm_flags))
		mt_cpu_dormant(CPU_DEEPIDLE_MODE);
	else {
		/* backup MPx_AXI_CONFIG */
		v0 = reg_read(MP0_AXI_CONFIG);
		v1 = reg_read(MP1_AXI_CONFIG);

		/* disable snoop function */
		MCUSYS_SMC_WRITE(MP0_AXI_CONFIG, v0 | ACINACTM);
		MCUSYS_SMC_WRITE(MP1_AXI_CONFIG, v1 | ACINACTM);

		dpidle_dbg("enter legacy WFI, MP0_AXI_CONFIG=0x%x, MP1_AXI_CONFIG=0x%x\n",
			   reg_read(MP0_AXI_CONFIG), reg_read(MP1_AXI_CONFIG));

		wfi_with_sync();

		/* restore MP0_AXI_CONFIG */
		MCUSYS_SMC_WRITE(MP0_AXI_CONFIG, v0);
		MCUSYS_SMC_WRITE(MP1_AXI_CONFIG, v1);

		dpidle_dbg("exit legacy WFI, MP0_AXI_CONFIG=0x%x, MP1_AXI_CONFIG=0x%x\n",
			   reg_read(MP0_AXI_CONFIG), reg_read(MP1_AXI_CONFIG));
	}
}

/*
 * wakesrc: WAKE_SRC_XXX
 * enable : enable or disable @wakesrc
 * replace: if true, will replace the default setting
 */
int spm_set_dpidle_wakesrc(u32 wakesrc, bool enable, bool replace)
{
	unsigned long flags;

	if (spm_is_wakesrc_invalid(wakesrc))
		return -EINVAL;

	spin_lock_irqsave(&__spm_lock, flags);
	if (enable) {
		if (replace)
			__spm_dpidle.pwrctrl->wake_src = wakesrc;
		else
			__spm_dpidle.pwrctrl->wake_src |= wakesrc;
	} else {
		if (replace)
			__spm_dpidle.pwrctrl->wake_src = 0;
		else
			__spm_dpidle.pwrctrl->wake_src &= ~wakesrc;
	}
	spin_unlock_irqrestore(&__spm_lock, flags);

	return 0;
}

static wake_reason_t spm_output_wake_reason(struct wake_status *wakesta, const struct pcm_desc *pcmdesc, u32 dump_log)
{
	wake_reason_t wr = WR_NONE;
	unsigned long int dpidle_log_print_curr_time = 0;
	bool log_print = false;

	if (dump_log == DEEPIDLE_LOG_FULL) {
		wr = __spm_output_wake_reason(wakesta, pcmdesc, false);
	} else if (dump_log == DEEPIDLE_LOG_REDUCED) {
		/* Determine print SPM log or not */
		dpidle_log_print_curr_time = idle_get_current_time_ms();

		if (wakesta->assert_pc != 0)
			log_print = true;
		/* Not wakeup by GPT */
		else if ((wakesta->r12 & (0x1 << 4)) == 0)
			log_print = true;
		else if (wakesta->timer_out <= DPIDLE_LOG_PRINT_TIMEOUT_CRITERIA)
			log_print = true;
		else if ((dpidle_log_print_curr_time - dpidle_log_print_prev_time) > DPIDLE_LOG_DISCARD_CRITERIA)
			log_print = true;

		/* Print SPM log */
		if (log_print == true) {
			dpidle_dbg("dpidle_log_discard_cnt = %d\n", dpidle_log_discard_cnt);
			wr = __spm_output_wake_reason(wakesta, pcmdesc, false);

			dpidle_log_print_prev_time = dpidle_log_print_curr_time;
			dpidle_log_discard_cnt = 0;
		} else {
			dpidle_log_discard_cnt++;

			wr = WR_NONE;
		}
	}

#ifdef CONFIG_MTK_ECCCI_DRIVER
	if (wakesta->r12 & WAKE_SRC_R12_CLDMA_EVENT_B)
		exec_ccci_kern_func_by_md_id(0, ID_GET_MD_WAKEUP_SRC, NULL, 0);
#endif

	return wr;
}

static u32 ap_pll_con0_val;

static void spm_dpidle_pre_process(void)
{
	u32 value = 0;

	__spm_pmic_pg_force_on();

	spm_pmic_power_mode(PMIC_PWR_DEEPIDLE, 0, 0);

	spm_bypass_boost_gpio_set();

	/* Update PMIC wrap table: deepidle */
	value = 0;
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_read_interface_nolock(PMIC_LDO_VSRAM_PROC_VOSEL_ON_ADDR,
								&value,
								PMIC_LDO_VSRAM_PROC_VOSEL_ON_MASK,
								PMIC_LDO_VSRAM_PROC_VOSEL_ON_SHIFT);
#else
	pmic_read_interface_nolock(MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON_ADDR,
								&value,
								MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON_MASK,
								MT6351_PMIC_BUCK_VSRAM_PROC_VOSEL_ON_SHIFT);
#endif
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE, IDX_DI_VSRAM_NORMAL, value);

	value = 0;
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_read_interface_nolock(PMIC_RG_SRCLKEN_IN2_EN_ADDR, &value, 0x037F, 0);
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE,
								IDX_DI_SRCCLKEN_IN2_NORMAL,
								value | (1 << PMIC_RG_SRCLKEN_IN2_EN_SHIFT));
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE,
								IDX_DI_SRCCLKEN_IN2_SLEEP,
								value & ~(1 << PMIC_RG_SRCLKEN_IN2_EN_SHIFT));
#else
	pmic_read_interface_nolock(MT6351_TOP_CON, &value, 0x037F, 0);
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE,
								IDX_DI_SRCCLKEN_IN2_NORMAL,
								value | (1 << MT6351_PMIC_RG_SRCLKEN_IN2_EN_SHIFT));
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE,
								IDX_DI_SRCCLKEN_IN2_SLEEP,
								value & ~(1 << MT6351_PMIC_RG_SRCLKEN_IN2_EN_SHIFT));
#endif

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_read_interface_nolock(PMIC_BUCK_VPROC_VOSEL_ON_ADDR,
					&value,
					PMIC_BUCK_VPROC_VOSEL_ON_MASK,
					PMIC_BUCK_VPROC_VOSEL_ON_SHIFT);
	mt_spm_pmic_wrap_set_cmd(PMIC_WRAP_PHASE_DEEPIDLE, IDX_DI_VPROC_NORMAL, value);
#else
	/* nothing */
#endif

	/* set PMIC WRAP table for deepidle power control */
	mt_spm_pmic_wrap_set_phase(PMIC_WRAP_PHASE_DEEPIDLE);

	/* Do more low power setting when MD1/C2K/CONN off */
	if (is_md_c2k_conn_power_off()) {
		__spm_bsi_top_init_setting();

		__spm_backup_pmic_ck_pdn();

		/* disable 26M clks: MIPID, MIPIC1, MIPIC0, MDPLLGP, SSUSB */
		ap_pll_con0_val = spm_read(AP_PLL_CON0);
		spm_write(AP_PLL_CON0, ap_pll_con0_val & (~0x18D0));
#if defined(PMIC_CLK_SRC_BY_SRCCLKEN_IN1)
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_config_interface_nolock(MT6353_BUCK_ALL_CON2, 0x111, 0x3FF, 0);
#else
		pmic_config_interface_nolock(MT6351_BUCK_ALL_CON2, 0x111, 0x3FF, 0);
#endif
#endif
		clk_buf_control_bblpm(true);
	}
}

static void spm_dpidle_post_process(void)
{
	/* Do more low power setting when MD1/C2K/CONN off */
	if (is_md_c2k_conn_power_off()) {
		clk_buf_control_bblpm(false);

#if defined(PMIC_CLK_SRC_BY_SRCCLKEN_IN1)
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		pmic_config_interface_nolock(MT6353_BUCK_ALL_CON2, 0x0, 0x3FF, 0);
#else
		pmic_config_interface_nolock(MT6351_BUCK_ALL_CON2, 0x0, 0x3FF, 0);
#endif
#endif
		/* Enable 26M clks: MIPID, MIPIC1, MIPIC0, MDPLLGP, SSUSB */
		spm_write(AP_PLL_CON0, ap_pll_con0_val);

		__spm_restore_pmic_ck_pdn();
	}

	/* set PMIC WRAP table for normal power control */
	mt_spm_pmic_wrap_set_phase(PMIC_WRAP_PHASE_NORMAL);

	__spm_pmic_pg_force_off();
}

#if 0
#ifdef CONFIG_OF
static int dt_scan_memory(unsigned long node, const char *uname, int depth, void *data)
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
	*(unsigned long *)data = node;
	return node;
}
#endif
#endif

wake_reason_t spm_go_to_dpidle(u32 spm_flags, u32 spm_data, u32 dump_log)
{
	struct wake_status wakesta;
	unsigned long flags;
	struct mtk_irq_mask mask;
	wake_reason_t wr = WR_NONE;
	/* struct pcm_desc *pcmdesc = __spm_dpidle.pcmdesc; */
	struct pcm_desc *pcmdesc = NULL;
	struct pwr_ctrl *pwrctrl = __spm_dpidle.pwrctrl;
	u32 cpu = spm_data;

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(SPM_DEEPIDLE_ENTER);
#endif

	if (dyna_load_pcm[DYNA_LOAD_PCM_DEEPIDLE + cpu / 4].ready)
		pcmdesc = &(dyna_load_pcm[DYNA_LOAD_PCM_DEEPIDLE + cpu / 4].desc);
	else
		BUG();

	update_pwrctrl_pcm_flags(&spm_flags);
	set_pwrctrl_pcm_flags(pwrctrl, spm_flags);

	spm_dpidle_before_wfi(cpu);

	lockdep_off();
	spin_lock_irqsave(&__spm_lock, flags);
	mt_irq_mask_all(&mask);
	mt_irq_unmask_for_sleep(SPM_IRQ0_ID);
	mt_irq_unmask_for_sleep(196);
	mt_cirq_clone_gic();
	mt_cirq_enable();

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(SPM_DEEPIDLE_ENTER_UART_SLEEP);
#endif

	if (request_uart_to_sleep()) {
		wr = WR_UART_BUSY;
		goto RESTORE_IRQ;
	}

	__spm_reset_and_init_pcm(pcmdesc);

	__spm_kick_im_to_fetch(pcmdesc);

	__spm_init_pcm_register();

	__spm_init_event_vector(pcmdesc);

	__spm_check_md_pdn_power_control(pwrctrl);

	__spm_sync_vcore_dvfs_power_control(pwrctrl, __spm_vcore_dvfs.pwrctrl);

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	spm_dpidle_pre_process();

	__spm_kick_pcm_to_run(pwrctrl);

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(SPM_DEEPIDLE_ENTER_WFI);
#endif

#ifdef SPM_DEEPIDLE_PROFILE_TIME
	gpt_get_cnt(SPM_PROFILE_APXGPT, &dpidle_profile[1]);
#endif

	spm_trigger_wfi_for_dpidle(pwrctrl);

#ifdef SPM_DEEPIDLE_PROFILE_TIME
	gpt_get_cnt(SPM_PROFILE_APXGPT, &dpidle_profile[2]);
#endif

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(SPM_DEEPIDLE_LEAVE_WFI);
#endif

	spm_dpidle_post_process();

	__spm_get_wakeup_status(&wakesta);

	__spm_clean_after_wakeup();

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(SPM_DEEPIDLE_ENTER_UART_AWAKE);
#endif

	request_uart_to_wakeup();

	wr = spm_output_wake_reason(&wakesta, pcmdesc, dump_log);

RESTORE_IRQ:
	mt_cirq_flush();
	mt_cirq_disable();
	mt_irq_mask_restore(&mask);
	spin_unlock_irqrestore(&__spm_lock, flags);
	lockdep_on();
	spm_dpidle_after_wfi(cpu, wakesta.debug_flag);

#if SPM_AEE_RR_REC
	aee_rr_rec_deepidle_val(0);
#endif

	return wr;
}

/*
 * cpu_pdn:
 *    true  = CPU dormant
 *    false = CPU standby
 * pwrlevel:
 *    0 = AXI is off
 *    1 = AXI is 26M
 * pwake_time:
 *    >= 0  = specific wakeup period
 */
wake_reason_t spm_go_to_sleep_dpidle(u32 spm_flags, u32 spm_data)
{
	u32 sec = 0;
	u32 dpidle_timer_val = 0;
	u32 dpidle_wake_src = 0;
	struct wake_status wakesta;
	unsigned long flags;
	struct mtk_irq_mask mask;
#ifdef CONFIG_MTK_WD_KICKER
	struct wd_api *wd_api;
	int wd_ret;
#endif
	static wake_reason_t last_wr = WR_NONE;
	/* struct pcm_desc *pcmdesc = __spm_dpidle.pcmdesc; */
	struct pcm_desc *pcmdesc = NULL;
	struct pwr_ctrl *pwrctrl = __spm_dpidle.pwrctrl;
	int cpu = smp_processor_id();

	if (dyna_load_pcm[DYNA_LOAD_PCM_DEEPIDLE + cpu / 4].ready)
		pcmdesc = &(dyna_load_pcm[DYNA_LOAD_PCM_DEEPIDLE + cpu / 4].desc);
	else
		BUG();

	/* backup original dpidle setting */
	dpidle_timer_val = pwrctrl->timer_val;
	dpidle_wake_src = pwrctrl->wake_src;

	update_pwrctrl_pcm_flags(&spm_flags);
	set_pwrctrl_pcm_flags(pwrctrl, spm_flags);

	spm_dpidle_before_wfi(cpu);

#if SPM_PWAKE_EN
	sec = spm_get_wake_period(-1 /* FIXME */, last_wr);
#endif
	pwrctrl->timer_val = sec * 32768;

	pwrctrl->wake_src = spm_get_sleep_wakesrc();
	pwrctrl->wake_src |= WAKE_SRC_R12_SYS_CIRQ_IRQ_B;

#ifdef CONFIG_MTK_WD_KICKER
	wd_ret = get_wd_api(&wd_api);
	if (!wd_ret)
		wd_api->wd_suspend_notify();
#endif

	spin_lock_irqsave(&__spm_lock, flags);
	mt_irq_mask_all(&mask);
	mt_irq_unmask_for_sleep(SPM_IRQ0_ID);
	mt_irq_unmask_for_sleep(196);
	mt_cirq_clone_gic();
	mt_cirq_enable();

	spm_crit2("sleep_deepidle, sec = %u, wakesrc = 0x%x [%u]\n",
			  sec, pwrctrl->wake_src, is_cpu_pdn(pwrctrl->pcm_flags));

	if (request_uart_to_sleep()) {
		last_wr = WR_UART_BUSY;
		goto RESTORE_IRQ;
	}

	__spm_reset_and_init_pcm(pcmdesc);

	__spm_kick_im_to_fetch(pcmdesc);

	__spm_init_pcm_register();

	__spm_init_event_vector(pcmdesc);

	__spm_sync_vcore_dvfs_power_control(pwrctrl, __spm_vcore_dvfs.pwrctrl);

	__spm_set_power_control(pwrctrl);

	__spm_set_wakeup_event(pwrctrl);

	spm_dpidle_pre_process();

	__spm_kick_pcm_to_run(pwrctrl);

	spm_trigger_wfi_for_dpidle(pwrctrl);

	spm_dpidle_post_process();

	__spm_get_wakeup_status(&wakesta);

	__spm_clean_after_wakeup();

	request_uart_to_wakeup();

	last_wr = __spm_output_wake_reason(&wakesta, pcmdesc, true);

RESTORE_IRQ:
	mt_cirq_flush();
	mt_cirq_disable();
	mt_irq_mask_restore(&mask);
	spin_unlock_irqrestore(&__spm_lock, flags);

#ifdef CONFIG_MTK_WD_KICKER
	if (!wd_ret)
		wd_api->wd_resume_notify();
#endif

	spm_dpidle_after_wfi(cpu, wakesta.debug_flag);

	/* restore original dpidle setting */
	pwrctrl->timer_val = dpidle_timer_val;
	pwrctrl->wake_src = dpidle_wake_src;

	return last_wr;
}

static int __init get_base_from_node(
	const char *cmp, void __iomem **pbase, int idx)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, cmp);

	if (!node) {
		pr_err("node '%s' not found!\n", cmp);
		return -1;
	}

	*pbase = of_iomap(node, idx);

	return 0;
}

void spm_deepidle_init(void)
{
#if defined(CONFIG_OF)
	struct device_node *node;
	struct resource r;

	/* mcucfg */
	node = of_find_compatible_node(NULL, NULL, MCUCFG_NODE);
	if (!node) {
		pr_err("error: cannot find node " MCUCFG_NODE);
		goto mcucfg_exit;
	}
	if (of_address_to_resource(node, 0, &r)) {
		pr_err("error: cannot get phys addr" MCUCFG_NODE);
		goto mcucfg_exit;
	}
	mcucfg_phys_base = r.start;

	mcucfg_base = (unsigned long)of_iomap(node, 0);
	if (!mcucfg_base) {
		pr_err("error: cannot iomap " MCUCFG_NODE);
		goto mcucfg_exit;
	}

	dpidle_dbg("mcucfg_base = 0x%u\n", (unsigned int)mcucfg_base);

	get_base_from_node("mediatek,APMIXED", &apmixedsys_base, 0);

mcucfg_exit:

	dpidle_dbg("spm_deepidle_init\n");
#endif
}

MODULE_DESCRIPTION("SPM-DPIdle Driver v0.1");
