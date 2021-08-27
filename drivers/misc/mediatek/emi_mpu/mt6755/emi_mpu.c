#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/list.h>
#ifdef CONFIG_MTK_AEE_FEATURE
#include <linux/aee.h>
#endif
#include <linux/timer.h>
#include <linux/workqueue.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_device_apc.h>
#include <mach/sync_write.h>
#include <mach/irqs.h>
#include <mach/dma.h>
#include <mach/memory.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <mach/emi_mpu.h>
#include <mach/mt_ccci_common.h>

void __iomem *EMI_BASE_ADDR = NULL;
static int Violation_PortID = MASTER_ALL;


#define ENABLE_EMI_CHKER
#define ENABLE_EMI_WATCH_POINT

#define MAX_EMI_MPU_STORE_CMD_LEN 128
//#define ABORT_EMI_BUS_INTERFACE 0x00200000 //DEVAPC0_D0_VIO_STA_0, idx:21
//#define ABORT_EMI			   0x00000001 //DEVAPC0_D0_VIO_STA_3, idx:0
#define TIMEOUT 100
#define AXI_VIO_MONITOR_TIME	(1 * HZ)

static struct work_struct emi_mpu_work;
static struct workqueue_struct * emi_mpu_workqueue = NULL;
static unsigned int enable_4gb;
static unsigned int vio_addr;
static unsigned int emi_physical_offset;

struct mst_tbl_entry {
	u32 master;
	u32 port;
	u32 id_mask;
	u32 id_val;
	char *note;
	char *name;
};

struct emi_mpu_notifier_block {
	struct list_head list;
	emi_mpu_notifier notifier;
};

static const struct mst_tbl_entry mst_tbl[] = {
	/* apmcu */
	{ .master = MST_ID_APMCU_0,  .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000000000100, .note = "CA7LL: Core nn system domain store exclusive", .name = "CA7" },
	{ .master = MST_ID_APMCU_1,  .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000000100100, .note = "CA7LL: Core nn barrier", .name = "CA7" },
	{ .master = MST_ID_APMCU_2,  .port = 0x0, .id_mask = 0b111111111111, .id_val = 0b000001001100, .note = "CA7LL: SCU generated barrier", .name = "CA7" },
	{ .master = MST_ID_APMCU_3,  .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000001100100, .note = "CA7LL: Core nn non-re-orderable device write", .name = "CA7" },
	{ .master = MST_ID_APMCU_4,  .port = 0x0, .id_mask = 0b111110000111, .id_val = 0b000010000100, .note = "CA7LL: Write to normal memory, or re-orderable device memory", .name = "CA7" },
	{ .master = MST_ID_APMCU_5,  .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000000000100, .note = "CA7LL: Core nn exclusive read or non-reorderable device read", .name = "CA7" },
	{ .master = MST_ID_APMCU_6,  .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000000100100, .note = "CA7LL: Core nn barrier", .name = "CA7" },
	{ .master = MST_ID_APMCU_7,  .port = 0x0, .id_mask = 0b111111111111, .id_val = 0b000001001100, .note = "CA7LL: SCU generated barrier or DVM complete", .name = "CA7" },
	{ .master = MST_ID_APMCU_8,  .port = 0x0, .id_mask = 0b111110011111, .id_val = 0b000010001100, .note = "CA7LL: ACP read", .name = "CA7"},
	{ .master = MST_ID_APMCU_9,  .port = 0x0, .id_mask = 0b111100000111, .id_val = 0b000100000100, .note = "CA7LL: Core nn read", .name = "CA7" },
	{ .master = MST_ID_APMCU_10, .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000000000011, .note = "CA7L: Core nn system domain store exclusive", .name = "CA7" },
	{ .master = MST_ID_APMCU_11, .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000000100011, .note = "CA7L: Core nn barrier", .name = "CA7" },
	{ .master = MST_ID_APMCU_12, .port = 0x0, .id_mask = 0b111111111111, .id_val = 0b000001001011, .note = "CA7L: SCU generated barrier", .name = "CA7" },
	{ .master = MST_ID_APMCU_13, .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000001100011, .note = "CA7L: Core nn non-re-orderable device write", .name = "CA7" },
	{ .master = MST_ID_APMCU_14, .port = 0x0, .id_mask = 0b111110000111, .id_val = 0b000010000011, .note = "CA7L: Write to normal memory, or re-orderable device memory", .name = "CA7" },
	{ .master = MST_ID_APMCU_15, .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000000000011, .note = "CA7L: Core nn exclusive read or non-reorderable device read", .name = "CA7" },
	{ .master = MST_ID_APMCU_16, .port = 0x0, .id_mask = 0b111111100111, .id_val = 0b000000100011, .note = "CA7L: Core nn barrier", .name = "CA7" },
	{ .master = MST_ID_APMCU_17, .port = 0x0, .id_mask = 0b111111111111, .id_val = 0b000001001011, .note = "CA7L: SCU generated barrier or DVM complete", .name = "CA7" },
	{ .master = MST_ID_APMCU_18, .port = 0x0, .id_mask = 0b111110011111, .id_val = 0b000010001011, .note = "CA7L: ACP read", .name = "CA7" },
	{ .master = MST_ID_APMCU_19, .port = 0x0, .id_mask = 0b111100000111, .id_val = 0b000100000011, .note = "CA7L: Core nn read", .name = "CA7" },

	/* MM */
	{ .master = MST_ID_MM_0, .port = 0x1, .id_mask = 0b111110000000, .id_val = 0b000000000000, .note = "smi_larb0", .name = "smi_larb0" },
	{ .master = MST_ID_MM_1, .port = 0x1, .id_mask = 0b111110000000, .id_val = 0b000010000000, .note = "smi_larb1", .name = "smi_larb1" },
	{ .master = MST_ID_MM_2, .port = 0x1, .id_mask = 0b111110000000, .id_val = 0b000100000000, .note = "smi_larb2", .name = "smi_larb2" },
	{ .master = MST_ID_MM_3, .port = 0x1, .id_mask = 0b111110000000, .id_val = 0b000110000000, .note = "smi_larb3", .name = "smi_larb3" },
	{ .master = MST_ID_MM_4, .port = 0x1, .id_mask = 0b111111111110, .id_val = 0b001111111100, .note = "MM IOMMU Internal Used", .name = "MM IOMMU Internal Used" },

	/* Periperal */
	{ .master = MST_ID_PERI_0,  .port = 0x2, .id_mask = 0b111111110011, .id_val = 0b000000000000, .note = "MSDC0", .name = "l_MSDC0" },
	{ .master = MST_ID_PERI_1,  .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000000010, .note = "PWM", .name = "l_PWM" },
	{ .master = MST_ID_PERI_2,  .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000000110, .note = "MSDC1", .name = "l_MSDC1" },
	{ .master = MST_ID_PERI_3,  .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000001010, .note = "MSDC2", .name = "MSDC2" },
	{ .master = MST_ID_PERI_4,  .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000001110, .note = "SPI0", .name = "l_SPI0" },
	{ .master = MST_ID_PERI_5,  .port = 0x2, .id_mask = 0b111111011111, .id_val = 0b000000000001, .note = "Debug Top", .name = "Debug Top" },
	{ .master = MST_ID_PERI_6,  .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000000101, .note = "AUDIO", .name = "AUDIO" },
	{ .master = MST_ID_PERI_7,  .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000100000101, .note = "MSDC3", .name = "MSDC3" },
	{ .master = MST_ID_PERI_8,  .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b001000000101, .note = "SPI1", .name = "SPI1" },
	{ .master = MST_ID_PERI_9,  .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b001100000101, .note = "USB2.0", .name = "USB2.0" },
	{ .master = MST_ID_PERI_10, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000100100101, .note = "SPM1_SPM2", .name = "SPM1_SPM2" },
	{ .master = MST_ID_PERI_11, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b001100100101, .note = "THERM", .name = "THERM" },
	{ .master = MST_ID_PERI_12, .port = 0x2, .id_mask = 0b111011111111, .id_val = 0b000001000101, .note = "U3", .name = "U3" },
	{ .master = MST_ID_PERI_13, .port = 0x2, .id_mask = 0b111011111111, .id_val = 0b000001100101, .note = "DMA", .name = "DMA" },
	{ .master = MST_ID_PERI_14, .port = 0x2, .id_mask = 0b100111111111, .id_val = 0b000010000101, .note = "MSDC0", .name = "l_MSDC0" },
	{ .master = MST_ID_PERI_15, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000110000101, .note = "PWM", .name = "l_PWM" },
	{ .master = MST_ID_PERI_16, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b001110000101, .note = "MSDC1", .name = "l_MSDC1" },
	{ .master = MST_ID_PERI_17, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b010110000101, .note = "MSDC2", .name = "MSDC2" },
	{ .master = MST_ID_PERI_18, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b011110000101, .note = "SPI0", .name = "l_SPI0" },
	{ .master = MST_ID_PERI_19, .port = 0x2, .id_mask = 0b111111011111, .id_val = 0b000000001001, .note = "CONNSYS", .name = "l_CONNSYS" },
	{ .master = MST_ID_PERI_20, .port = 0x2, .id_mask = 0b111111011111, .id_val = 0b000000001101, .note = "GCPU_M", .name = "GCPU_M" },
	{ .master = MST_ID_PERI_21, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000010001, .note = "CLDMA_AP", .name = "CLDMA_AP" },
	{ .master = MST_ID_PERI_22, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000110001, .note = "CLDMA_MD", .name = "CLDMA_MD" },
	{ .master = MST_ID_PERI_23, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000000011, .note = "GCE", .name = "l_GCE" },
	{ .master = MST_ID_PERI_24, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000000111, .note = "GDMA", .name = "GDMA" },
	{ .master = MST_ID_PERI_25, .port = 0x2, .id_mask = 0b111111111111, .id_val = 0b000000001011, .note = "GCE_prefetch", .name = "GCE_prefetch" },

	/* Modem MCU*/
	{ .master = MST_ID_MDMCU_0,    .port = 0x3, .id_mask = 0b111100001111, .id_val = 0b000000000000, .note = "MDMCU", .name = "MDMCU" },
	{ .master = MST_ID_MDMCU_1,    .port = 0x3, .id_mask = 0b111100001111, .id_val = 0b000000000100, .note = "MDMCU PREFETCH", .name = "MDMCU" },
	{ .master = MST_ID_MDMCU_2,    .port = 0x3, .id_mask = 0b111100001111, .id_val = 0b000000001000, .note = "MDMCU ROW3 NO TITLE", .name = "MDMCU" },
	{ .master = MST_ID_MD_L1MCU_0, .port = 0x3, .id_mask = 0b111100001111, .id_val = 0b000000000001, .note = "L1MCU", .name = "L1MCU" },
	{ .master = MST_ID_MD_L1MCU_1, .port = 0x3, .id_mask = 0b111100001111, .id_val = 0b000000000101, .note = "L1MCU PREFETCH", .name = "L1MCU" },
	{ .master = MST_ID_MD_L1MCU_2, .port = 0x3, .id_mask = 0b111100001111, .id_val = 0b000000001001, .note = "L1MCU ROW3 NO TITLE", .name = "L1MCU" },
	{ .master = MST_ID_C2KMCU_0,   .port = 0x3, .id_mask = 0b000000000011, .id_val = 0b000000000010, .note = "C2KMCU ARM D port", .name = "C2KMCU" },

	/* Modem HW (2G/3G/LTE) */
	{ .master = MST_ID_MDHW_0,  .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b000000000000, .note = "AP2MDREG", .name = "MDHWMIX_AP2MDREG" },
	{ .master = MST_ID_MDHW_1,  .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b000000000001, .note = "MDDBGSYS", .name = "MDHWMIX_MDDBGSYS" },
	{ .master = MST_ID_MDHW_2,  .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b000000000010, .note = "PSMCU2REG", .name = "MDHWMIX_PSMCU2REG" },
	{ .master = MST_ID_MDHW_3,  .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b000100000000, .note = "MDGDMA", .name = "MDHWMIX_MDGDMA" },
	{ .master = MST_ID_MDHW_4,  .port = 0x4, .id_mask = 0b111111111110, .id_val = 0b001000000000, .note = "HSPAL2", .name = "MDHWMIX_HSPAL2" },
	{ .master = MST_ID_MDHW_5,  .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001111000000, .note = "Rake_MD2G", .name = "MDHWMIX_Rake_MD2G" },
	{ .master = MST_ID_MDHW_6,  .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001101000000, .note = "DFE_B4", .name = "MDHWMIX_DFE" },
	{ .master = MST_ID_MDHW_7,  .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001101000001, .note = "DFE_DBG", .name = "MDHWMIX_DFE" },
	{ .master = MST_ID_MDHW_8,  .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001101000010, .note = "DFE_DBG1", .name = "MDHWMIX_DFE" },
	{ .master = MST_ID_MDHW_9,  .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001101000011, .note = "DFE_SHM", .name = "MDHWMIX_DFE" },
	{ .master = MST_ID_MDHW_10, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001110000000, .note = "RXBRP_HRQ_WR", .name = "MDHWMIX_RXBRP" },
	{ .master = MST_ID_MDHW_11, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001110000100, .note = "RXBRP_HRQ_WR1", .name = "MDHWMIX_RXBRP" },
	{ .master = MST_ID_MDHW_12, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001110000001, .note = "RXBRP_HRQ_RD", .name = "MDHWMIX_RXBRP" },
	{ .master = MST_ID_MDHW_13, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001110000101, .note = "RXBRP_HRQ_RD1", .name = "MDHWMIX_RXBRP" },
	{ .master = MST_ID_MDHW_14, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001110000010, .note = "RXBRP_HRQ_DBG", .name = "MDHWMIX_RXBRP" },
	{ .master = MST_ID_MDHW_15, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001110000110, .note = "RXBRP_HRQ_OTH", .name = "MDHWMIX_RXBRP" },
	{ .master = MST_ID_MDHW_16, .port = 0x4, .id_mask = 0b111111000111, .id_val = 0b001100000000, .note = "L1MCU", .name = "MDHWMIX_L1MCU" },
	{ .master = MST_ID_MDHW_17, .port = 0x4, .id_mask = 0b111111111011, .id_val = 0b001100000001, .note = "MDL1_GDMA", .name = "MDHWMIX_MDL1_GDMA" },
	{ .master = MST_ID_MDHW_18, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b001100000010, .note = "MDL1_ABM", .name = "MDHWMIX_MDL1_ABM" },
	{ .master = MST_ID_MDHW_19, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b010000000000, .note = "ARM7", .name = "MDHWMIX_ARM7" },
	{ .master = MST_ID_MDHW_20, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b010100000000, .note = "PS_PERI", .name = "MDHWMIX_PS_PERI" },
	{ .master = MST_ID_MDHW_21, .port = 0x4, .id_mask = 0b111111111110, .id_val = 0b011000000000, .note = "LTEL2DMA", .name = "MDHWMIX_LTEL2DMA" },
	{ .master = MST_ID_MDHW_22, .port = 0x4, .id_mask = 0b111111111110, .id_val = 0b011100000000, .note = "SOE_TRACE", .name = "MDHWMIX_SOE_TRACE" },

	{ .master = MST_ID_LTE_0,  .port = 0x4, .id_mask = 0b111111111011, .id_val = 0b100000000000, .note = "CMP_DSPLOG", .name = "MDHWLTE_DSPLOG" },
	{ .master = MST_ID_LTE_1,  .port = 0x4, .id_mask = 0b111111111011, .id_val = 0b100000000000, .note = "CMP_DSPBTDMA", .name = "MDHWLTE_CMP_DSPBTDMA" },
	{ .master = MST_ID_LTE_2,  .port = 0x4, .id_mask = 0b111111111011, .id_val = 0b100000000010, .note = "CMP_CNWDMA", .name = "MDHWLTE_CMP_CNWDMA" },
	{ .master = MST_ID_LTE_3,  .port = 0x4, .id_mask = 0b111111111101, .id_val = 0b100000010000, .note = "CS", .name = "MDHWLTE_CS" },
	{ .master = MST_ID_LTE_4,  .port = 0x4, .id_mask = 0b111111111011, .id_val = 0b100000100000, .note = "IMC_RXTDB", .name = "MDHWLTE_IMC" },
	{ .master = MST_ID_LTE_5,  .port = 0x4, .id_mask = 0b111111110111, .id_val = 0b100000100010, .note = "IMC_DSPLOG", .name = "MDHWLTE_DSPLOG" },
	{ .master = MST_ID_LTE_6,  .port = 0x4, .id_mask = 0b111111110111, .id_val = 0b100000100010, .note = "IMC_DSPBTDMA", .name = "MDHWLTE_IMC" },
	{ .master = MST_ID_LTE_7,  .port = 0x4, .id_mask = 0b111111110111, .id_val = 0b100000100110, .note = "IMC_CNWDMA", .name = "MDHWLTE_IMC" },
	{ .master = MST_ID_LTE_8,  .port = 0x4, .id_mask = 0b111111111011, .id_val = 0b100000110000, .note = "ICC_DSPLOG", .name = "MDHWLTE_DSPLOG" },
	{ .master = MST_ID_LTE_9,  .port = 0x4, .id_mask = 0b111111111011, .id_val = 0b100000110000, .note = "ICC_DSPBTDMA", .name = "MDHWLTE_ICC" },
	{ .master = MST_ID_LTE_10, .port = 0x4, .id_mask = 0b111111111011, .id_val = 0b100000110010, .note = "ICC_CNWDMA", .name = "MDHWLTE_ICC" },
	{ .master = MST_ID_LTE_11, .port = 0x4, .id_mask = 0b111111111101, .id_val = 0b100001000000, .note = "RXDMP", .name = "MDHWLTE_RXDMP" },
	{ .master = MST_ID_LTE_12, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b100000000001, .note = "cs id0", .name = "MDHWLTE_CS" },
	{ .master = MST_ID_LTE_13, .port = 0x4, .id_mask = 0b111111111111, .id_val = 0b100000000011, .note = "cs id1", .name = "MDHWLTE_CS" },

	/* MFG */
	{ .master = MST_ID_MFG_0, .port = 0x5, .id_mask = 0b111111000000, .id_val = 0b000000000000, .note = "MFG", .name = "l_MFG" },

};

//struct list_head emi_mpu_notifier_list[NR_MST];
static const char *UNKNOWN_MASTER = "l_unknown";
static spinlock_t emi_mpu_lock;

#ifdef ENABLE_EMI_CHKER
struct timer_list emi_axi_vio_timer;
#endif

char *smi_larb0_port[17] = {"disp_ovl_0", "disp_rdma_0", "disp_wdma_0", "disp_ovl_1", "disp_rdma_1", "disp_wdma_1", "ufod_rdma0", "ufod_rdma1", "ufod_rdma2", "ufod_rdma3", "ufod_rdma4", "ufod_rdma5", "ufod_rdma6", "ufod_rdma7", "mdp_rdma", "mdp_wdma", "mdp_wrot"};
char *smi_larb1_port[9] =  {"hw_vdec_mc_ext", "hw_vdec_pp_ext", "hw_vdec_vld_ext", "hw_vdec_avc_mv_ext" , "hw_vdec_pred_rd_ext", "hw_vdec_pred_wr_ext", "hw_vdec_ppwarp_ext" };
char *smi_larb2_port[21] = {"cam_imgo", "cam_rrzo", "cam_aao", "cam_esfko", "cam_imgo_s", "cam_isci", "cam_isci_d", "cam_bpci", "cam_bpci_d", "cam_ufdi",
							"cam_imgi", "cam_img2o", "cam_img3o", "cam_vipi", "cam_vip2i", "cam_vip3i", "cam_icei" , "cam_rb" , "cam_rp" , "cam_wr"
						   };
char *smi_larb3_port[19] = {"venc_rcpu", "venc_rec", "venc_bsdma", "venc_sv_comv", "vend_rd_comv", "jpgenc_rdma", "jpgenc_bsdma", "jpgdec_wdma", "jpgdec_bsdma", "venc_cur_luma", "venc_cur_chroma", "venc_ref_luma", "vend_ref_chroma", "redmc_wdma", "venc_nbm_rdma", "venc_nbm_wdma"};
char *smi_larb4_port[4] = {"mjc_mv_rd", "mjc_mv_wr", "mjc_dma_rd", "mjc_dma_wr"};

static int __match_id(u32 axi_id, int tbl_idx, u32 port_ID)
{
	u32 mm_larb;
	u32 smi_port;

	if (((axi_id & mst_tbl[tbl_idx].id_mask) == mst_tbl[tbl_idx].id_val) && (port_ID == mst_tbl[tbl_idx].port)) {
		switch(port_ID) {
		case 0: /* ARM */
		case 2:	/* Peripheral */
		case 3:	/* MDMCU ,C2K MCU */
		case 4:	/* MD HW ,LTE */
		case 5:	/* MFG */
			pr_err("Violation master name is %s.\n", mst_tbl[tbl_idx].name);
			break;
		case 1:
			 /*MM*/ mm_larb = axi_id >> 7;
			smi_port = (axi_id & 0x7F) >> 2;
			if(mm_larb == 0x0) {
				if(smi_port >= ARRAY_SIZE(smi_larb0_port)) {
					pr_err("[EMI MPU ERROR] Invalidate master ID! lookup smi table failed!\n");
					return 0;
				}
				pr_err("Violation master name is %s (%s).\n", mst_tbl[tbl_idx].name, smi_larb0_port[smi_port]);
			} else if(mm_larb == 0x1) {
				if(smi_port >= ARRAY_SIZE(smi_larb1_port)) {
					pr_err("[EMI MPU ERROR] Invalidate master ID! lookup smi table failed!\n");
					return 0;
				}
				pr_err("Violation master name is %s (%s).\n", mst_tbl[tbl_idx].name, smi_larb1_port[smi_port]);
			} else if(mm_larb == 0x2) {
				if(smi_port >= ARRAY_SIZE(smi_larb2_port)) {
					pr_err("[EMI MPU ERROR] Invalidate master ID! lookup smi table failed!\n");
					return 0;
				}
				pr_err("Violation master name is %s (%s).\n", mst_tbl[tbl_idx].name, smi_larb2_port[smi_port]);
			} else if (mm_larb == 0x3) {	/* smi_larb3 */
				if (smi_port >= ARRAY_SIZE(smi_larb3_port)) {
					pr_err("[EMI MPU ERROR] Invalidate master ID! lookup smi table failed!\n");
					return 0;
				}
				pr_err("Violation master name is %s (%s).\n", mst_tbl[tbl_idx].name,
				       smi_larb3_port[smi_port]);
			} else {	/*MM IOMMU Internal Used */
				pr_err("Violation master name is %s.\n", mst_tbl[tbl_idx].name);
			}
			break;
		default:
			pr_err("[EMI MPU ERROR] Invalidate port ID! lookup bus ID table failed!\n");
			break;
		}
		return 1;
	} else {
		return 0;
	}
}

#if 0
static u32 __id2mst(u32 id)
{
	int i;
	u32 axi_ID;
	u32 port_ID;

	axi_ID = (id >> 3) & 0x000001FFF;
	port_ID = id & 0x00000007;

	pr_err("[EMI MPU] axi_id = %x, port_id = %x\n", axi_ID, port_ID);

	for (i = 0; i < ARRAY_SIZE(mst_tbl); i++) {
		if (__match_id(axi_ID, i, port_ID)) {
			return mst_tbl[i].master;
		}
	}
	return MST_INVALID;
}
#endif

static int isetocunt;
static void emi_mpu_set_violation_port(int portID)
{
	if (isetocunt == 0) {
			Violation_PortID = portID;
			isetocunt = 1;
		}
	else if (isetocunt == 2) {
			isetocunt = 0;
		}
}

int emi_mpu_get_violation_port(void)
{
	int ret;

	pr_err("[EMI MPU] emi_mpu_get_violation_port \n");

	if (isetocunt == 0) {
		pr_err("[EMI MPU] EMI did not set Port ID \n");
		isetocunt = 2;
		return MASTER_ALL;
	}

	ret = Violation_PortID;
	isetocunt = 0;

	if (ret >= MASTER_ALL) {
		pr_err("[EMI MPU] Port ID: %d is an invalid Port ID\n", ret);
	} else {
		Violation_PortID = MASTER_ALL;
	}

	return ret;
}

static char *__id2name(u32 id)
{
	int i;
	u32 axi_ID;
	u32 port_ID;

	axi_ID = (id >> 3) & 0x00000FFF;
	port_ID = id & 0x00000007;
	emi_mpu_set_violation_port(port_ID);
	pr_err("[EMI MPU] axi_id = %x, port_id = %x\n", axi_ID, port_ID);

	for (i = 0; i < ARRAY_SIZE(mst_tbl); i++) {
		if (__match_id(axi_ID, i, port_ID)) {
			return mst_tbl[i].name;
		}
	}

	return (char *)UNKNOWN_MASTER;
}
#define NR_REGIONS  18
static void __clear_emi_mpu_vio(unsigned int first)
{
	u32 dbg_s, dbg_t;
	int i;
	/* clear violation status */

	writel(0x00FFF3FF, EMI_MPUP);
	writel(0x00FFF3FF, EMI_MPUQ);
	writel(0x00FFF3FF, EMI_MPUR);
	writel(0x00FFF3FF, EMI_MPUY);
	writel(0x00FFF3FF, EMI_MPUP2);
	writel(0x00FFF3FF, EMI_MPUQ2);
	writel(0x00FFF3FF, EMI_MPUR2);
	mt_reg_sync_writel(0x00FFF3FF, EMI_MPUY2);

	/* clear debug info */
	mt_reg_sync_writel(0x80000000 , EMI_MPUS);
	dbg_s = readl(IOMEM(EMI_MPUS));
	dbg_t = readl(IOMEM(EMI_MPUT));

	if(first){
		/* clear the permission setting of MPU*/
		for(i = 0; i < NR_REGIONS; i++) {
			if(i < 8) {
				mt_emi_reg_write(0x0, EMI_MPUA + i * 8);
			} else if ((i > 7) && (i < 16)) {
				mt_emi_reg_write(0x0, EMI_MPUA2 + (i - 8) * 8);
			} else {
				mt_emi_reg_write(0x0, EMI_MPUA3 + (i - 16) * 8);
			}
		}

		for(i = 0; i < NR_REGIONS; i++) {
			if(i < 8) {
				mt_emi_reg_write(0x0, EMI_MPUI + i * 4);
			} else if ((i > 7) && (i < 16)) {
				mt_emi_reg_write(0x0, EMI_MPUI2 + (i - 8) * 4);
			} else {
				mt_emi_reg_write(0x0, EMI_MPUI3 + (i - 16) * 8);
			}
		}
	}

	if (dbg_s) {
		pr_err("Fail to clear EMI MPU violation\n");
		pr_err("EMI_MPUS = %x, EMI_MPUT = %x", dbg_s, dbg_t);
	}
}

static int mpu_check_violation(void)
{
	u32 dbg_s, dbg_t, dbg_pqry;
	u32 master_ID, domain_ID, wr_vio;
	s32 region;
	char *master_name;

	dbg_s = readl(IOMEM(EMI_MPUS));
	dbg_t = readl(IOMEM(EMI_MPUT));

	pr_alert("Clear status.\n");

	master_ID = (dbg_s & 0x00007FFF);
	domain_ID = (dbg_s >> 21) & 0x00000007;
	wr_vio = (dbg_s >> 28) & 0x00000003;
	region = (dbg_s >> 16) & 0x1F;

	switch (domain_ID) {
	case 0:
		dbg_pqry = readl(IOMEM(EMI_MPUP));
		break;
	case 1:
		dbg_pqry = readl(IOMEM(EMI_MPUQ));
		break;
	case 2:
		dbg_pqry = readl(IOMEM(EMI_MPUR));
		break;
	case 3:
		dbg_pqry = readl(IOMEM(EMI_MPUY));
		break;
	case 4:
		dbg_pqry = readl(IOMEM(EMI_MPUP2));
		break;
	case 5:
		dbg_pqry = readl(IOMEM(EMI_MPUQ2));
		break;
	case 6:
		dbg_pqry = readl(IOMEM(EMI_MPUR2));
		break;
	case 7:
		dbg_pqry = readl(IOMEM(EMI_MPUY2));
		break;
	default:
		dbg_pqry = 0;
		break;
	}

	/*TBD: print the abort region*/

	pr_err("EMI MPU violation.\n");
	pr_err("[EMI MPU] Debug info start ----------------------------------------\n");

	pr_err("EMI_MPUS = %x, EMI_MPUT = %x.\n", dbg_s, dbg_t);
	pr_err("Current process is \"%s \" (pid: %i).\n", current->comm, current->pid);
	pr_err("Violation address is 0x%x.\n", dbg_t + emi_physical_offset);
	pr_err("Violation master ID is 0x%x.\n", master_ID);
	/*print out the murderer name*/
	master_name = __id2name(master_ID);
	pr_err("Violation domain ID is 0x%x.\n", domain_ID);
	pr_err("%s violation.\n", (wr_vio == 1)? "Write": "Read");
	pr_err("Corrupted region is %d\n\r", region);
	if ((region == 0) || (region == 1)) { /* add log to debug a strange issue */
		pr_err("EMI_CHKER=%x, EMI_CHKER_TYPE=%x, EMI_CHKER_ADR=%x\n", readl(IOMEM(EMI_CHKER)),
			readl(IOMEM(EMI_CHKER_TYPE)), readl(IOMEM(EMI_CHKER_ADR)));
		pr_err("EMI_MPUA=%x, EMI_MPUB=%x, EMI_MPUC=%x\n", mt_emi_reg_read(EMI_MPUA),
			mt_emi_reg_read(EMI_MPUB), mt_emi_reg_read(EMI_MPUC));
		pr_err("EMI_MPUD=%x, EMI_MPUE=%x, EMI_MPUF=%x\n", mt_emi_reg_read(EMI_MPUD),
			mt_emi_reg_read(EMI_MPUE), mt_emi_reg_read(EMI_MPUF));
		pr_err("EMI_MPUG=%x, EMI_MPUH=%x, EMI_MPUA2=%x\n", mt_emi_reg_read(EMI_MPUG),
			mt_emi_reg_read(EMI_MPUH), mt_emi_reg_read(EMI_MPUA2));
		pr_err("EMI_MPUB2=%x, EMI_MPUC2=%x, EMI_MPUD2=%x\n", mt_emi_reg_read(EMI_MPUB2),
			mt_emi_reg_read(EMI_MPUC2), mt_emi_reg_read(EMI_MPUD2));
		pr_err("EMI_MPUE2=%x, EMI_MPUF2=%x, EMI_MPUG2=%x\n", mt_emi_reg_read(EMI_MPUE2),
			mt_emi_reg_read(EMI_MPUF2), mt_emi_reg_read(EMI_MPUG2));
		pr_err("EMI_MPUH2=%x, EMI_MPUA3=%x, EMI_MPUB3=%x\n", mt_emi_reg_read(EMI_MPUH2), mt_emi_reg_read(EMI_MPUA3), mt_emi_reg_read(EMI_MPUB3));
		pr_err("EMI_MPUC3=%x, EMI_MPUD3=%x, EMI_MPUI=%x\n", mt_emi_reg_read(EMI_MPUC3), mt_emi_reg_read(EMI_MPUD3), mt_emi_reg_read(EMI_MPUI));
		pr_err("EMI_MPUI_2ND=%x, EMI_MPUJ=%x, EMI_MPUJ_2ND=%x\n", mt_emi_reg_read(EMI_MPUI_2ND), mt_emi_reg_read(EMI_MPUJ), mt_emi_reg_read(EMI_MPUJ_2ND));
		pr_err("EMI_MPUK=%x, EMI_MPUK_2ND=%x, EMI_MPUL=%x\n", mt_emi_reg_read(EMI_MPUK), mt_emi_reg_read(EMI_MPUK_2ND), mt_emi_reg_read(EMI_MPUL));
		pr_err("EMI_MPUL_2ND=%x, EMI_MPUI2=%x, EMI_MPUI2_2ND=%x\n", mt_emi_reg_read(EMI_MPUL_2ND), mt_emi_reg_read(EMI_MPUI2), mt_emi_reg_read(EMI_MPUI2_2ND));
		pr_err("EMI_MPUJ2=%x, EMI_MPUJ2_2ND=%x, EMI_MPUK2=%x\n", mt_emi_reg_read(EMI_MPUJ2), mt_emi_reg_read(EMI_MPUJ2_2ND), mt_emi_reg_read(EMI_MPUK2));
		pr_err("EMI_MPUK2_2ND=%x, EMI_MPUL2=%x, EMI_MPUL2_2ND=%x\n", mt_emi_reg_read(EMI_MPUK2_2ND), mt_emi_reg_read(EMI_MPUL2), mt_emi_reg_read(EMI_MPUL2_2ND));
		pr_err("EMI_MPUI3=%x, EMI_MPUJ3=%x, EMI_MPUK3=%x\n", mt_emi_reg_read(EMI_MPUI3), mt_emi_reg_read(EMI_MPUJ3), mt_emi_reg_read(EMI_MPUK3));
		pr_err("EMI_MPUL3=%x, EMI_MPUM=%x, EMI_MPUN=%x\n", mt_emi_reg_read(EMI_MPUL3), mt_emi_reg_read(EMI_MPUM), mt_emi_reg_read(EMI_MPUN));
		pr_err("EMI_MPUO=%x, EMI_MPUU=%x, EMI_MPUM2=%x\n", mt_emi_reg_read(EMI_MPUO), mt_emi_reg_read(EMI_MPUU), mt_emi_reg_read(EMI_MPUM2));
		pr_err("EMI_MPUN2=%x, EMI_MPUO2=%x, EMI_MPUU2=%x\n", mt_emi_reg_read(EMI_MPUN2), mt_emi_reg_read(EMI_MPUO2), mt_emi_reg_read(EMI_MPUU2));
		pr_err("EMI_MPUV=%x, EMI_MPUW=%x, EMI_MPUX=%x\n", mt_emi_reg_read(EMI_MPUV), mt_emi_reg_read(EMI_MPUW), mt_emi_reg_read(EMI_MPUX));
	}
	if (dbg_pqry & OOR_VIO) {
		pr_err("Out of range violation.\n");
	}
	pr_err("[EMI MPU] Debug info end------------------------------------------\n");

#ifdef CONFIG_MTK_AEE_FEATURE
	if (wr_vio != 0) {
		#ifndef CONFIG_MT_ENG_BUILD /* EMI violation is relative to MD */
			if (((master_ID & 0x7) == MASTER_MDMCU) || ((master_ID & 0x7) == MASTER_MDHW)) {
					int md_id = 0;
					exec_ccci_kern_func_by_md_id(md_id, ID_FORCE_MD_ASSERT, NULL, 0);
					pr_err("[EMI MPU] MPU violation trigger MD\n");
				}
		#endif
		aee_kernel_exception("EMI MPU",
		"%sEMI_MPUS = 0x%x,EMI_MPUT = 0x%x\n \
CHKER = 0x%x,CHKER_TYPE = 0x%x,CHKER_ADR = 0x%x\n \
MPUA = 0x%x, MPUI = 0x%x, MPUB = 0x%x, MPUJ = 0x%x, MPUC = 0x%x, MPUK = 0x%x\n \
MPUD = 0x%x, MPUL = 0x%x, MPUE = 0x%x, MPUI_2ND = 0x%x\n%s%s\n",
				     "EMI MPU violation.\n",
				     dbg_s,
				     dbg_t+emi_physical_offset,
				     readl(IOMEM(EMI_CHKER)), readl(IOMEM(EMI_CHKER_TYPE)), readl(IOMEM(EMI_CHKER_ADR)),
					 mt_emi_reg_read(EMI_MPUA), mt_emi_reg_read(EMI_MPUI), mt_emi_reg_read(EMI_MPUB),
					 mt_emi_reg_read(EMI_MPUJ), mt_emi_reg_read(EMI_MPUC), mt_emi_reg_read(EMI_MPUK),
					 mt_emi_reg_read(EMI_MPUD), mt_emi_reg_read(EMI_MPUL), mt_emi_reg_read(EMI_MPUE),
					 mt_emi_reg_read(EMI_MPUI_2ND),
				     "CRDISPATCH_KEY:EMI MPU Violation Issue/",
				     master_name);
		}
#endif

	__clear_emi_mpu_vio(0);
	mt_devapc_clear_emi_violation();
	vio_addr = dbg_t + emi_physical_offset;
	return 0;

}

/*EMI MPU violation handler*/
static irqreturn_t mpu_violation_irq(int irq, void *dev_id)
{
	int res, res1;

	/* Need DEVAPC owner porting */
	res = mt_devapc_check_emi_violation();
	if (!res)
		pr_err("it's a mt_devapc_emi_violation.\n");

	res1 = mt_devapc_check_emi_mpu_violation();
	if (!res1) {
		    pr_err("EMI MPU APB violation.\n");
			pr_err("[EMI MPU] Debug info start ----------------------------------------\n");
			pr_err("EMI_MPUW=%x \n", mt_emi_reg_read(EMI_MPUW));
			pr_err("EMI_MPUV=%x \n", mt_emi_reg_read(EMI_MPUV));
			pr_err("EMI_MPUX=%x \n", mt_emi_reg_read(EMI_MPUX));
			pr_err("Violation address=%x \n", mt_emi_reg_read(EMI_MPUW) & 0xFFFF);
			pr_err("Violation domain=%x \n", (mt_emi_reg_read(EMI_MPUW)>>16) & 0x7);

			if ((mt_emi_reg_read(EMI_MPUV)>>28) & 0x1)
				pr_err("it's a Write violation \n");
			else if ((mt_emi_reg_read(EMI_MPUV)>>29) & 0x1)
				pr_err("it's a Read violation \n");

			pr_err("[EMI MPU] Debug info end------------------------------------------\n");
			mt_emi_reg_write(mt_emi_reg_read(EMI_MPUX)|0x80000000, EMI_MPUX);
			mt_devapc_clear_emi_mpu_violation();
			return IRQ_HANDLED;
		}

	if (res && res1)
		return IRQ_NONE;

	pr_info("It's a MPU violation.\n");
	mpu_check_violation();
	return IRQ_HANDLED;
}

#if defined(CONFIG_MTKPASR)
/* Acquire DRAM Setting for PASR/DPD */
void acquire_dram_setting(struct basic_dram_setting *pasrdpd)
{
	int ch_nr = MAX_CHANNELS;
	unsigned int emi_cona, emi_conh, col_bit, row_bit;
	unsigned int ch0_rank0_size, ch0_rank1_size, ch1_rank0_size, ch1_rank1_size;

	pasrdpd->channel_nr = ch_nr;

	emi_cona = readl(IOMEM(EMI_CONA));
	emi_conh = readl(IOMEM(EMI_CONH));

	ch0_rank0_size = (emi_conh >> 16) & 0xf;
	ch0_rank1_size = (emi_conh >> 20) & 0xf;
	ch1_rank0_size = (emi_conh >> 24) & 0xf;
	ch1_rank1_size = (emi_conh >> 28) & 0xf;

	{
		pasrdpd->channel[0].rank[0].valid_rank = true;

		if (ch0_rank0_size == 0) {
			col_bit = ((emi_cona >> 4) & 0x03) + 9;
			row_bit = ((emi_cona >> 12) & 0x03) + 13;
			pasrdpd->channel[0].rank[0].rank_size = (1 << (row_bit + col_bit)) >> 22;
			pasrdpd->channel[0].rank[0].segment_nr = 8;
		} else {
			pasrdpd->channel[0].rank[0].rank_size = (ch0_rank0_size * 2);
			pasrdpd->channel[0].rank[0].segment_nr = 6;
		}

		if (0 != (emi_cona & (1 << 17))) {
			pasrdpd->channel[0].rank[1].valid_rank = true;

			if (ch0_rank1_size == 0) {
				col_bit = ((emi_cona >> 6) & 0x03) + 9;
				row_bit = ((emi_cona >> 14) & 0x03) + 13;
				pasrdpd->channel[0].rank[1].rank_size = (1 << (row_bit + col_bit)) >> 22;
				pasrdpd->channel[0].rank[1].segment_nr = 8;
			} else {
				pasrdpd->channel[0].rank[1].rank_size = (ch0_rank1_size * 2);
				pasrdpd->channel[0].rank[1].segment_nr = 6;
			}
		} else {
			pasrdpd->channel[0].rank[1].valid_rank = false;
			pasrdpd->channel[0].rank[1].segment_nr = 0;
			pasrdpd->channel[0].rank[1].rank_size = 0;
		}
	}

	if (0 != (emi_cona & 0x01)) {

		pasrdpd->channel[1].rank[0].valid_rank = true;

		if (ch1_rank0_size == 0) {
			col_bit = ((emi_cona >> 20) & 0x03) + 9;
			row_bit = ((emi_cona >> 28) & 0x03) + 13;
			pasrdpd->channel[1].rank[0].rank_size = (1 << (row_bit + col_bit)) >> 22;
			pasrdpd->channel[1].rank[0].segment_nr = 8;
		} else {
			pasrdpd->channel[1].rank[0].rank_size = (ch1_rank0_size * 2);
			pasrdpd->channel[1].rank[0].segment_nr = 6;
		}

		if (0 != (emi_cona & (1 << 16))) {
			pasrdpd->channel[1].rank[1].valid_rank = true;

			if (ch1_rank1_size == 0) {
				col_bit = ((emi_cona >> 22) & 0x03) + 9;
				row_bit = ((emi_cona >> 30) & 0x03) + 13;
				pasrdpd->channel[1].rank[1].rank_size = (1 << (row_bit + col_bit)) >> 22;
				pasrdpd->channel[1].rank[1].segment_nr = 8;
			} else {
				pasrdpd->channel[1].rank[1].rank_size = (ch1_rank1_size * 2);
				pasrdpd->channel[1].rank[1].segment_nr = 6;
			}
		} else {
			pasrdpd->channel[1].rank[1].valid_rank = false;
			pasrdpd->channel[1].rank[1].segment_nr = 0;
			pasrdpd->channel[1].rank[1].rank_size = 0;
		}
	} else {
		pasrdpd->channel[1].rank[0].valid_rank = false;
		pasrdpd->channel[1].rank[0].segment_nr = 0;
		pasrdpd->channel[1].rank[0].rank_size = 0;

		pasrdpd->channel[1].rank[1].valid_rank = false;
		pasrdpd->channel[1].rank[1].segment_nr = 0;
		pasrdpd->channel[1].rank[1].rank_size = 0;
	}

	return;
}
#endif

/*
 * emi_mpu_set_region_protection: protect a region.
 * @start: start address of the region
 * @end: end address of the region
 * @region: EMI MPU region id
 * @access_permission: EMI MPU access permission
 * Return 0 for success, otherwise negative status code.
 */
int emi_mpu_set_region_protection(unsigned long long start, unsigned long long end, int region, unsigned int access_permission)
{
	int ret = 0;
	unsigned long flags;

	access_permission = access_permission & 0xFFFFFF;
	access_permission = access_permission | ((region & 0x1F)<<27);
	spin_lock_irqsave(&emi_mpu_lock, flags);
	mt_emi_mpu_set_region_protection(start, end, access_permission);
	spin_unlock_irqrestore(&emi_mpu_lock, flags);

	return ret;
}

static ssize_t emi_mpu_show(struct device_driver *driver, char *buf)
{
	char *ptr = buf;
	unsigned int start, end;
	unsigned int reg_value;
	unsigned int d0, d1, d2, d3, d4, d5, d6, d7;
	static const char *permission[7] = {
		"No",
		"SEC_RW",
		"SEC_RW_NSEC R",
		"SEC_RW_NSEC_W",
		"SEC_R_NSEC_R",
		"For",
		"SEC_R_NSEC_RW"
	};

	reg_value = mt_emi_reg_read(EMI_MPUA);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R0 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUB);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R1 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUC);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R2 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUD);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R3 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUE);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R4 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUF);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R5 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUG);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R6 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUH);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R7 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUA2);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R8 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUB2);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R9 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUC2);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R10 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUD2);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R11 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUE2);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R12 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUF2);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R13 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUG2);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R14 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUH2);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R15 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUA3);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R16 --> 0x%x to 0x%x\n", start, end);

	reg_value = mt_emi_reg_read(EMI_MPUB3);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R17 --> 0x%x to 0x%x\n", start, end);
	
	reg_value = mt_emi_reg_read(EMI_MPUC3);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R18 --> 0x%x to 0x%x\n", start, end);
	
	reg_value = mt_emi_reg_read(EMI_MPUD3);
	start = ((reg_value >> 16) << 16) + emi_physical_offset;
	end = ((reg_value & 0xFFFF) << 16) + emi_physical_offset + 0xFFFF;
	ptr += sprintf(ptr, "R19 --> 0x%x to 0x%x\n", start, end);

	ptr += sprintf (ptr, "\n");

	reg_value = mt_emi_reg_read(EMI_MPUI);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R0 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R0 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

  reg_value = mt_emi_reg_read(EMI_MPUJ);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R1 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R1 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUK);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R2 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R2 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUL);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R3 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R3 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);


	reg_value = mt_emi_reg_read(EMI_MPUI_2ND);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R4 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R4 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);


	reg_value = mt_emi_reg_read(EMI_MPUJ_2ND);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R5 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R5 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUK_2ND);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R6 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R6 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUL_2ND);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R7 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R7 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUI2);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R8 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R8 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUJ2);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R9 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R9 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUK2);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R10 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R10 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUL2);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R11 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R11 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);


	reg_value = mt_emi_reg_read(EMI_MPUI2_2ND);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R12 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R12 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);


	reg_value = mt_emi_reg_read(EMI_MPUJ2_2ND);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R13 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R13 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUK2_2ND);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R14 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R14 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUL2_2ND);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R15 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R15 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUI3);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R16 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R16 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	reg_value = mt_emi_reg_read(EMI_MPUJ3);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R17 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R17 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);
	
	reg_value = mt_emi_reg_read(EMI_MPUK3);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R18 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R18 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);
	
	reg_value = mt_emi_reg_read(EMI_MPUL3);
	d0 = (reg_value & 0x7);
	d1 = (reg_value >> 3) & 0x7;
	d2 = (reg_value >> 6) & 0x7;
	d3 = (reg_value >> 9) & 0x7;
	d4 = (reg_value >> 12) & 0x7;
	d5 = (reg_value >> 15) & 0x7;
	d6 = (reg_value >> 18) & 0x7;
	d7 = (reg_value >> 21) & 0x7;
	ptr += sprintf(ptr, "R19 --> d0 = %s, d1 = %s, d2 = %s, d3 = %s\n", permission[d0],  permission[d1],  permission[d2], permission[d3]);
	ptr += sprintf(ptr, "R19 --> d4 = %s, d5 = %s, d6 = %s, d7 = %s\n", permission[d4],  permission[d5],  permission[d6], permission[d7]);

	return strlen(buf);
}

static ssize_t emi_mpu_store(struct device_driver *driver, const char *buf, size_t count)
{
	int i;
	unsigned int start_addr;
	unsigned int end_addr;
	unsigned int region;
	unsigned int access_permission;
	char *command;
	char *ptr;
	char *token [5];
	ssize_t ret;

	if ((strlen(buf) + 1) > MAX_EMI_MPU_STORE_CMD_LEN) {
		pr_err("emi_mpu_store command overflow.");
		return count;
	}
	pr_err("emi_mpu_store: %s\n", buf);

	command = kmalloc((size_t) MAX_EMI_MPU_STORE_CMD_LEN, GFP_KERNEL);
	if (!command)
		return count;

	strcpy(command, buf);
	ptr = (char *)buf;

	if (!strncmp(buf, EN_MPU_STR, strlen(EN_MPU_STR))) {
		i = 0;
		while (ptr != NULL) {
			ptr = strsep(&command, " ");
			token[i] = ptr;
			pr_devel("token[%d] = %s\n", i, token[i]);
			i++;
		}
		for (i = 0; i < 5; i++)
			pr_devel("token[%d] = %s\n", i, token[i]);

		/* kernel standardization
		start_addr = simple_strtoul(token[1], &token[1], 16);
		end_addr = simple_strtoul(token[2], &token[2], 16);
		region = simple_strtoul(token[3], &token[3], 16);
		access_permission = simple_strtoul(token[4], &token[4], 16);
		*/
		ret = kstrtoul(token[1], 16, (unsigned long *)&start_addr);
		if (ret != 0)
			pr_err("kstrtoul fails to parse start_addr");
		ret = kstrtoul(token[2], 16, (unsigned long *)&end_addr);
		if (ret != 0)
			pr_err("kstrtoul fails to parse end_addr");
		ret = kstrtoul(token[3], 16, (unsigned long *)&region);
		if (ret != 0)
			pr_err("kstrtoul fails to parse region");
		ret = kstrtoul(token[4], 16, (unsigned long *)&access_permission);
		if (ret != 0)
			pr_err("kstrtoul fails to parse access_permission");
		emi_mpu_set_region_protection(start_addr, end_addr, region, access_permission);
		pr_err("Set EMI_MPU: start: 0x%x, end: 0x%x, region: %d, permission: 0x%x.\n",
		       start_addr, end_addr, region, access_permission);
	} else if (!strncmp(buf, DIS_MPU_STR, strlen(DIS_MPU_STR))) {
		i = 0;
		while (ptr != NULL) {
			ptr = strsep(&command, " ");
			token[i] = ptr;
			pr_devel("token[%d] = %s\n", i, token[i]);
			i++;
		}
		for (i = 0; i < 5; i++)
			pr_devel("token[%d] = %s\n", i, token[i]);
		/* kernel standardization
		start_addr = simple_strtoul(token[1], &token[1], 16);
		end_addr = simple_strtoul(token[2], &token[2], 16);
		region = simple_strtoul(token[3], &token[3], 16);
		*/
		ret = kstrtoul(token[1], 16, (unsigned long *)&start_addr);
		if (ret != 0)
			pr_err("kstrtoul fails to parse start_addr");
		ret = kstrtoul(token[2], 16, (unsigned long *)&end_addr);
		if (ret != 0)
			pr_err("kstrtoul fails to parse end_addr");
		ret = kstrtoul(token[3], 16, (unsigned long *)&region);
		if (ret != 0)
			pr_err("kstrtoul fails to parse region");

		emi_mpu_set_region_protection(0x0, 0x0, region,
					      SET_ACCESS_PERMISSON(NO_PROTECTION, NO_PROTECTION,
								   NO_PROTECTION, NO_PROTECTION,
								   NO_PROTECTION, NO_PROTECTION,
								   NO_PROTECTION, NO_PROTECTION));
		pr_err("set EMI MPU: start: 0x%x, end: 0x%x, region: %d, permission: 0x%x\n", 0, 0,
		       region, SET_ACCESS_PERMISSON(NO_PROTECTION, NO_PROTECTION, NO_PROTECTION,
						    NO_PROTECTION, NO_PROTECTION, NO_PROTECTION,
						    NO_PROTECTION, NO_PROTECTION));
	} else {
		pr_err("Unknown emi_mpu command.\n");
	}

	kfree(command);

	return count;
}

DRIVER_ATTR(mpu_config, 0644, emi_mpu_show, emi_mpu_store);

void mtk_search_full_pgtab(void)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	unsigned long addr;
#ifndef CONFIG_ARM_LPAE
	pte_t *pte;
	unsigned long addr_2nd, addr_2nd_end;
#endif
	unsigned int v_addr = vio_addr;

	/*FIXME: testing*/
	//vio_addr = 0x9DE0D000;

	for(addr=0xC0000000; addr<0xFFF00000; addr+=0x100000) {
		pgd = pgd_offset(&init_mm, addr);
		if (pgd_none(*pgd) || !pgd_present(*pgd)) {
			continue;
		}

		pud = pud_offset(pgd, addr);
		if (pud_none(*pud) || !pud_present(*pud)) {
			continue;
		}

		pmd = pmd_offset(pud, addr);
		if (pmd_none(*pmd) || !pmd_present(*pmd)) {
			continue;
		}

		//printk("[EMI MPU] ============= addr = %x\n", addr);

#ifndef CONFIG_ARM_LPAE
		if ((pmd_val(*pmd) & PMD_TYPE_MASK) == PMD_TYPE_TABLE) {
			/* Page table entry*/
			//printk("[EMI MPU] 2nd Entry pmd: %lx, *pmd = %lx\n", (unsigned long)(pmd), (unsigned long)pmd_val(*(pmd)));
			addr_2nd = addr;
			addr_2nd_end = addr_2nd + 0x100000;
			for(; addr_2nd<(addr_2nd_end); addr_2nd+=0x1000) {
				pte = pte_offset_map(pmd, addr_2nd);
				//printk("[EMI MPU] pmd: %x, pte: %x, *pte = %x, addr_2nd = 0x%x, addr_2nd_end = 0x%x\n", (unsigned long)(pmd), (unsigned long)(pte), (unsigned long)pte_val(*(pte)), addr_2nd, addr_2nd_end);
				if(((unsigned long)v_addr & PAGE_MASK) == ((unsigned long)pte_val(*(pte)) & PAGE_MASK)) {
					pr_err("[EMI MPU] Find page entry section at pte: %lx. violation address = 0x%x\n", (unsigned long)(pte), v_addr);
					return;
				}
			}
		} else {
			//printk("[EMI MPU] Section pmd: %x, addr = 0x%x\n", (unsigned long)(pmd), addr);
			/* Section */
			//if(v_addr>>20 == (unsigned long)pmd_val(*(pmd))>>20)
			if(((unsigned long)pmd_val(*(pmd)) & SECTION_MASK) == ((unsigned long)v_addr & SECTION_MASK)) {
				pr_err("[EMI MPU] Find page entry section at pmd: %lx. violation address = 0x%x\n", (unsigned long)(pmd), v_addr);
				return;
			}
		}
#else
		/* TBD */
#endif
	}
	pr_err("[EMI MPU] ****** Can not find page table entry! violation address = 0x%x ******\n", v_addr);

	return;
}

void emi_mpu_work_callback(struct work_struct *work)
{
	pr_err("[EMI MPU] Enter EMI MPU workqueue!\n");
	mtk_search_full_pgtab();
	pr_err("[EMI MPU] Exit EMI MPU workqueue!\n");
}

static ssize_t pgt_scan_show(struct device_driver *driver, char *buf)
{
	return 0;
}

static ssize_t pgt_scan_store(struct device_driver *driver, const char *buf, size_t count)
{
	unsigned int value;
	unsigned int ret;

	if (unlikely(sscanf(buf, "%u", &value) != 1))
		return -EINVAL;

	if(value == 1) {
		ret = queue_work(emi_mpu_workqueue, &emi_mpu_work);
		if(!ret) {
			pr_debug("[EMI MPU] submit workqueue failed, ret = %d\n", ret);
		}
	}

	return count;
}
DRIVER_ATTR(pgt_scan, 0644, pgt_scan_show, pgt_scan_store);

#ifdef ENABLE_EMI_CHKER
static void emi_axi_set_chker(const unsigned int setting)
{
	int value;

	value = readl(IOMEM(EMI_CHKER));
	value &= ~(0x7 << 16);
	value |= (setting);

	mt_reg_sync_writel(value, EMI_CHKER);
}

static void emi_axi_set_master(const unsigned int setting)
{
	int value;

	value = readl(IOMEM(EMI_CHKER));
	value &= ~(0x0F << AXI_NON_ALIGN_CHK_MST);
	value |= (setting & 0xF) << AXI_NON_ALIGN_CHK_MST;

	mt_reg_sync_writel(value, EMI_CHKER);
}

static void emi_axi_dump_info(int aee_ke_en)
{
	int value, master_ID;
	char *master_name;

	value = readl(IOMEM(EMI_CHKER));
	master_ID = (value & 0x0000FFFF);

	if (value & 0x0000FFFF) {
		pr_err("AXI violation.\n");
		pr_err("[EMI MPU AXI] Debug info start ----------------------------------------\n");

		pr_err("EMI_CHKER = %x.\n", value);
		pr_err("Violation address is 0x%x.\n", readl(IOMEM(EMI_CHKER_ADR)));
		pr_err("Violation master ID is 0x%x.\n", master_ID);
		pr_err("Violation type is: AXI_ADR_CHK_EN(%d), AXI_LOCK_CHK_EN(%d), AXI_NON_ALIGN_CHK_EN(%d).\n",
			   (value & (1 << AXI_ADR_VIO)) ? 1 : 0, (value & (1 << AXI_LOCK_ISSUE)) ? 1 : 0, (value & (1 << AXI_NON_ALIGN_ISSUE)) ? 1 : 0);
		pr_err("%s violation.\n", (value & (1 << AXI_VIO_WR))? "Write": "Read");

		pr_err("[EMI MPU AXI] Debug info end ----------------------------------------\n");

		master_name = __id2name(master_ID);
#ifdef CONFIG_MTK_AEE_FEATURE
		if (aee_ke_en)
			aee_kernel_exception("EMI MPU AXI", "AXI violation.\nEMI_CHKER = 0x%x\nCRDISPATCH_KEY:EMI MPU Violation Issue/%s\n", value, master_name);
#endif
		// clear AXI checker status
		mt_reg_sync_writel((1 << AXI_VIO_CLR) | readl(IOMEM(EMI_CHKER)), EMI_CHKER);
	}
}

static void emi_axi_vio_timer_func(unsigned long a)
{
	emi_axi_dump_info(1);

	mod_timer(&emi_axi_vio_timer, jiffies + AXI_VIO_MONITOR_TIME);
}

static ssize_t emi_axi_vio_show(struct device_driver *driver, char *buf)
{
	int value;

	value = readl(IOMEM(EMI_CHKER));

	emi_axi_dump_info(0);

	return snprintf(buf, PAGE_SIZE, "AXI vio setting is: ADR_CHK_EN %s, LOCK_CHK_EN %s, NON_ALIGN_CHK_EN %s\n", (value & (1 << AXI_ADR_CHK_EN)) ? "ON" : "OFF",
					(value & (1 << AXI_LOCK_CHK_EN)) ? "ON" : "OFF",
					(value & (1 << AXI_NON_ALIGN_CHK_EN)) ? "ON" : "OFF");
}

ssize_t emi_axi_vio_store(struct device_driver *driver, const char *buf, size_t count)
{
	int value;
	int cpu = 0;	//assign timer to CPU0 to avoid CPU plug-out and timer will be unavailable

	value = readl(IOMEM(EMI_CHKER));

	if (!strncmp(buf, "ADR_CHK_ON", strlen("ADR_CHK_ON"))) {
		emi_axi_set_chker(1 << AXI_ADR_CHK_EN);
		add_timer_on(&emi_axi_vio_timer, cpu);
	} else if (!strncmp(buf, "LOCK_CHK_ON", strlen("LOCK_CHK_ON"))) {
		emi_axi_set_chker(1 << AXI_LOCK_CHK_EN);
		add_timer_on(&emi_axi_vio_timer, cpu);
	} else if (!strncmp(buf, "NON_ALIGN_CHK_ON", strlen("NON_ALIGN_CHK_ON"))) {
		emi_axi_set_chker(1 << AXI_NON_ALIGN_CHK_EN);
		add_timer_on(&emi_axi_vio_timer, cpu);
	} else if (!strncmp(buf, "OFF", strlen("OFF"))) {
		emi_axi_set_chker(0);
		del_timer(&emi_axi_vio_timer);
	} else {
		pr_err("invalid setting\n");
	}

	return count;
}

DRIVER_ATTR(emi_axi_vio,	0644, emi_axi_vio_show,	 emi_axi_vio_store);

#endif //#ifdef ENABLE_EMI_CHKER

#ifdef ENABLE_EMI_WATCH_POINT
static void emi_wp_set_address(unsigned int address)
{
	mt_reg_sync_writel(address - emi_physical_offset, EMI_WP_ADR);
}

static void emi_wp_set_range(unsigned int range)  // 2^ range bytes
{
	unsigned int value;

	value = readl(IOMEM(EMI_WP_CTRL));
	value = (value & (~EMI_WP_RANGE)) | range;
	mt_reg_sync_writel(value, EMI_WP_CTRL);
}

static void emi_wp_set_monitor_type(unsigned int type)
{
	unsigned int value;

	value = readl(IOMEM(EMI_WP_CTRL));
	value = (value & (~EMI_WP_RW_MONITOR)) | (type << EMI_WP_RW_MONITOR_SHIFT);
	mt_reg_sync_writel(value, EMI_WP_CTRL);
}

#if 0
static void emi_wp_set_rw_disable(unsigned int type)
{
	unsigned int value;

	value = readl(IOMEM(EMI_WP_CTRL));
	value = (value & (~EMI_WP_RW_DISABLE)) | (type << EMI_WP_RW_DISABLE_SHIFT);
	mt_reg_sync_writel(value, EMI_WP_CTRL);
}
#endif

static void emi_wp_enable(int enable)
{
	unsigned int value;

	/* Enable WP */
	value = readl(IOMEM(EMI_CHKER));
	value = (value & ~(1 << EMI_WP_ENABLE_SHIFT)) | (enable << EMI_WP_ENABLE_SHIFT);
	mt_reg_sync_writel(value, EMI_CHKER);
}

static void emi_wp_slave_error_enable(unsigned int enable)
{
	unsigned int value;

	value = readl(IOMEM(EMI_WP_CTRL));
	value = (value & ~(1 << EMI_WP_SLVERR_SHIFT)) | (enable << EMI_WP_SLVERR_SHIFT);
	mt_reg_sync_writel(value, EMI_WP_CTRL);
}

static void emi_wp_int_enable(unsigned int enable)
{
	unsigned int value;

	value = readl(IOMEM(EMI_WP_CTRL));
	value = (value & ~(1 << EMI_WP_INT_SHIFT)) | (enable << EMI_WP_INT_SHIFT);
	mt_reg_sync_writel(value, EMI_WP_CTRL);
}

static void emi_wp_clr_status(void)
{
	unsigned int value;
	int result;

	value = readl(IOMEM(EMI_CHKER));
	value |= 1 << EMI_WP_VIO_CLR_SHIFT;
	mt_reg_sync_writel(value, EMI_CHKER);

	result = readl(IOMEM(EMI_CHKER)) & EMI_WP_AXI_ID;
	result |= readl(IOMEM(EMI_CHKER_TYPE));
	result |= readl(IOMEM(EMI_CHKER_ADR));

	if (result)
		pr_err("[EMI_WP] Clear WP status fail!!!!!!!!!!!!!!\n");
}

void emi_wp_get_status(void)
{
	unsigned int value,master_ID;
	char *master_name;

	value = readl(IOMEM(EMI_CHKER));

	if ((value & 0x80000000) == 0) {
		pr_err("[EMI_WP] No watch point hit\n");
		return;
	}

	master_ID = (value & EMI_WP_AXI_ID);
	pr_err("[EMI_WP] Violation master ID is 0x%x.\n", master_ID);
	pr_err("[EMI_WP] Violation Address is : 0x%X\n", readl(IOMEM(EMI_CHKER_ADR))+ emi_physical_offset);

	master_name = __id2name(master_ID);
	pr_err("[EMI_WP] EMI_CHKER = 0x%x, module is %s.\n", value, master_name);

	value = readl(IOMEM(EMI_CHKER_TYPE));
	pr_err("[EMI_WP] Transaction Type is : %d beat, %d byte, %s burst type (0x%X)\n", (value & 0xF)+1, 1 << ((value >> 4) & 0x7), (value >> 7 & 1) ? "INCR": "WRAP", value);

	emi_wp_clr_status();
}

static int emi_wp_set(unsigned int enable, unsigned int address, unsigned int range, unsigned int rw)
{
	if (address < emi_physical_offset) {
		pr_err("[EMI_WP] Address error, you can't set address less than 0x%X\n", emi_physical_offset);
		return -1;
	}
	if (range < 4 || range > 32) {
		pr_err("[EMI_WP] Range error, you can't set range less than 16 bytes and more than 4G bytes\n");
		return -1;
	}

	emi_wp_set_monitor_type(rw);
	emi_wp_set_address(address);
	emi_wp_set_range(range);
	emi_wp_slave_error_enable(1);
	emi_wp_int_enable(0);
	emi_wp_enable(enable);

	return 0;
}

static ssize_t emi_wp_vio_show(struct device_driver *driver, char *buf)
{
	unsigned int value,master_ID, type, vio_addr;
	char *master_name;
	char *ptr = buf;

	value = readl(IOMEM(EMI_CHKER));

	if ((value & 0x80000000) == 0) {
		return snprintf(buf, PAGE_SIZE, "[EMI_WP] No watch point hit \n");
	}

	master_ID = (value & EMI_WP_AXI_ID);
	master_name = __id2name(master_ID);

	type = readl(IOMEM(EMI_CHKER_TYPE));
	vio_addr = readl(IOMEM(EMI_CHKER_ADR))+ emi_physical_offset;
	emi_wp_clr_status();
	ptr += snprintf(ptr, PAGE_SIZE, "[EMI WP] vio setting is: CHKER 0x%X, ",
			value);

	ptr += snprintf(ptr, PAGE_SIZE, "module is %s, Address is : 0x%X," ,
			master_name,
			vio_addr);

	ptr += snprintf(ptr, PAGE_SIZE, "Transaction Type is : %d beat, ",
			(type & 0xF) + 1);

	ptr += snprintf(ptr, PAGE_SIZE, "%d byte, %s burst type (0x%X)\n",
			1 << ((type >> 4) & 0x7),
			(type >> 7 & 1) ? "INCR" : "WRAP",
			type);
	return ptr - buf;
}

ssize_t emi_wp_vio_store(struct device_driver *driver, const char *buf, size_t count)
{
	int i;
	unsigned int wp_addr;
	unsigned int range, start_addr, end_addr;
	unsigned int rw;
	char *command;
	char *ptr;
	char *token [5];
	int err = 0;

	if ((strlen(buf) + 1) > MAX_EMI_MPU_STORE_CMD_LEN) {
		pr_err("emi_wp_store command overflow.");
		return count;
	}
	pr_err("emi_wp_store: %s\n", buf);

	command = kmalloc((size_t)MAX_EMI_MPU_STORE_CMD_LEN, GFP_KERNEL);
	if (!command) {
		return count;
	}
	strcpy(command, buf);
	ptr = (char *)buf;

	if (!strncmp(buf, EN_WP_STR, strlen(EN_WP_STR))) {
		i = 0;
		while (ptr != NULL) {
			ptr = strsep(&command, " ");
			token[i] = ptr;
			pr_debug("token[%d] = %s\n", i, token[i]);
			i++;
		}
		for (i = 0; i < 4; i++)
			pr_debug("token[%d] = %s\n", i, token[i]);

		err += kstrtoul(token[1], 16, (unsigned long *)&wp_addr);
		err += kstrtoul(token[2], 16, (unsigned long *)&range);
		err += kstrtoul(token[3], 16, (unsigned long *)&rw);
		if (err)
			goto out;

		emi_wp_set(1, wp_addr, range, rw);

		start_addr = (wp_addr >> range) << range;
		end_addr = start_addr + (1 << range)     - 1;
		pr_err("Set EMI_WP: address: 0x%x, range:%d, start addr: 0x%x, end addr: 0x%x,  rw: %d .\n",
		       wp_addr, range, start_addr, end_addr, rw);
	} else if (!strncmp(buf, DIS_WP_STR, strlen(DIS_WP_STR))) {
		i = 0;
		while (ptr != NULL) {
			ptr = strsep(&command, " ");
			token[i] = ptr;
			pr_debug("token[%d] = %s\n", i, token[i]);
			i++;
		}
		for (i = 0; i < 4; i++)
			pr_debug("token[%d] = %s\n", i, token[i]);


		err += kstrtoul(token[1], 16, (unsigned long *)&wp_addr);
		err += kstrtoul(token[2], 16, (unsigned long *)&range);
		err += kstrtoul(token[3], 16, (unsigned long *)&rw);
		if (err)
			goto out;

		emi_wp_set(0, 0x40000000, 4, 2);
		pr_err("disable EMI WP\n");
	} else {
		pr_err("Unknown emi_wp command.\n");
	}

out:
	kfree(command);

	return count;
}


DRIVER_ATTR(emi_wp_vio, 0644, emi_wp_vio_show, emi_wp_vio_store);
#endif //#ifdef ENABLE_EMI_WATCH_POINT

extern phys_addr_t get_max_DRAM_size(void);
#define AP_REGION_ID   19
static void protect_ap_region(void)
{

	unsigned int ap_mem_mpu_id, ap_mem_mpu_attr;
	unsigned long long kernel_base;
	phys_addr_t dram_size;

	kernel_base = emi_physical_offset;
	dram_size = get_max_DRAM_size();

	ap_mem_mpu_id = AP_REGION_ID;
	ap_mem_mpu_attr = SET_ACCESS_PERMISSON(FORBIDDEN, NO_PROTECTION, FORBIDDEN, NO_PROTECTION, FORBIDDEN, FORBIDDEN, FORBIDDEN, NO_PROTECTION);

	emi_mpu_set_region_protection(kernel_base, (kernel_base+dram_size-1), ap_mem_mpu_id, ap_mem_mpu_attr);

}

static struct platform_driver emi_mpu_ctrl = {
	.driver = {
		.name = "emi_mpu_ctrl",
		.bus = &platform_bus_type,
		.owner = THIS_MODULE,
	},
	.id_table = NULL,
};

static int __init emi_mpu_mod_init(void)
{
	int ret;
	struct device_node *node;
	unsigned int mpu_irq;
	void __iomem *INFRASYS_BASE_ADDR = NULL;

	pr_info("Initialize EMI MPU.\n");

	isetocunt = 0;

	/* DTS version */
	if (EMI_BASE_ADDR == NULL) {
		node = of_find_compatible_node(NULL, NULL, "mediatek,EMI");
		if (node) {
			EMI_BASE_ADDR = of_iomap(node, 0);
			pr_err("get EMI_BASE_ADDR @ %p\n", EMI_BASE_ADDR);
		} else {
			pr_err("can't find compatible node\n");
			return -1;
		}
	}

	node = of_find_compatible_node(NULL, NULL, "mediatek,devapc");
	if (node) {
		mpu_irq = irq_of_parse_and_map(node, 0);
		pr_notice("get EMI_MPU irq = %d\n", mpu_irq);
	} else {
		pr_err("can't find compatible node\n");
		return -1;
	}

	{ /* get the information for 4G mode */
		node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-infrasys");
		if (node) {
			INFRASYS_BASE_ADDR = of_iomap(node, 0);
			pr_err("get INFRASYS_BASE_ADDR @ %p\n", INFRASYS_BASE_ADDR);
		} else {
			pr_err("INFRASYS_BASE_ADDR can't find compatible node\n");
			return -1;
		}
	}

	spin_lock_init(&emi_mpu_lock);

	pr_err("[EMI MPU] EMI_MPUP = 0x%x\n", readl(IOMEM(EMI_MPUP)));
	pr_err("[EMI MPU] EMI_MPUQ = 0x%x\n", readl(IOMEM(EMI_MPUQ)));
	pr_err("[EMI MPU] EMI_MPUR = 0x%x\n", readl(IOMEM(EMI_MPUR)));
	pr_err("[EMI MPU] EMI_MPUY = 0x%x\n", readl(IOMEM(EMI_MPUY)));
	pr_err("[EMI MPU] EMI_MPUP2 = 0x%x\n", readl(IOMEM(EMI_MPUP2)));
	pr_err("[EMI MPU] EMI_MPUQ2 = 0x%x\n", readl(IOMEM(EMI_MPUQ2)));
	pr_err("[EMI MPU] EMI_MPUR2 = 0x%x\n", readl(IOMEM(EMI_MPUR2)));
	pr_err("[EMI MPU] EMI_MPUY2 = 0x%x\n", readl(IOMEM(EMI_MPUY2)));

	pr_err("[EMI MPU] EMI_MPUS = 0x%x\n", readl(IOMEM(EMI_MPUS)));
	pr_err("[EMI MPU] EMI_MPUT = 0x%x\n", readl(IOMEM(EMI_MPUT)));

	pr_err("[EMI MPU] EMI_WP_ADR = 0x%x\n", readl(IOMEM(EMI_WP_ADR)));
	pr_err("[EMI MPU] EMI_WP_CTRL = 0x%x\n", readl(IOMEM(EMI_WP_CTRL)));
	pr_err("[EMI MPU] EMI_CHKER = 0x%x\n", readl(IOMEM(EMI_CHKER)));
	pr_err("[EMI MPU] EMI_CHKER_TYPE = 0x%x\n", readl(IOMEM(EMI_CHKER_TYPE)));
	pr_err("[EMI MPU] EMI_CHKER_ADR = 0x%x\n", readl(IOMEM(EMI_CHKER_ADR)));

	pr_err("[EMI MPU] EMI_MPUW = 0x%x\n", mt_emi_reg_read(EMI_MPUW));
	pr_err("[EMI MPU] EMI_MPUV = 0x%x\n", mt_emi_reg_read(EMI_MPUV));
	pr_err("[EMI MPU] EMI_MPUX = 0x%x\n", mt_emi_reg_read(EMI_MPUX));

	if (readl(IOMEM(EMI_MPUS))) {
		pr_err("[EMI MPU] get MPU violation in driver init\n");
		mt_devapc_emi_initial();
		mpu_check_violation();
	} else {
		__clear_emi_mpu_vio(0);
		/* Set Device APC initialization for EMI-MPU. */
		mt_devapc_emi_initial();
	}

	pr_err("[EMI MPU] INFRASYS_BASE_ADDR+0xF00 = 0x%x\n", readl(IOMEM(INFRASYS_BASE_ADDR + 0xF00)));

	/* if (0 == enable_4G()) */
	if ((readl(IOMEM(INFRASYS_BASE_ADDR + 0xF00)) & 0x2000) == 0) {
		emi_physical_offset = 0x40000000;
		pr_err("[EMI MPU] Not 4G mode\n");
	} else { //enable 4G mode
		emi_physical_offset = 0;
		pr_err("[EMI MPU] 4G mode\n");
	}
	/*
	 * NoteXXX: Interrupts of violation (including SPC in SMI, or EMI MPU)
	 *		  are triggered by the device APC.
	 *		  Need to share the interrupt with the SPC driver.
	 */

	ret = request_irq(mpu_irq, (irq_handler_t)mpu_violation_irq, IRQF_TRIGGER_LOW | IRQF_SHARED, "mt_emi_mpu", &emi_mpu_ctrl);
	if (ret != 0) {
		pr_err("Fail to request EMI_MPU interrupt. Error = %d.\n", ret);
		return ret;
	}

	protect_ap_region();

#ifdef ENABLE_EMI_CHKER
	/* AXI violation monitor setting and timer function create */
	mt_reg_sync_writel((1 << AXI_VIO_CLR) | readl(IOMEM(EMI_CHKER)), EMI_CHKER);
	emi_axi_set_master(MASTER_ALL);
	init_timer(&emi_axi_vio_timer);
	emi_axi_vio_timer.expires = jiffies + AXI_VIO_MONITOR_TIME;
	emi_axi_vio_timer.function = &emi_axi_vio_timer_func;
	emi_axi_vio_timer.data = ((unsigned long) 0 );
#endif //#ifdef ENABLE_EMI_CHKER

#if !defined(USER_BUILD_KERNEL)
#ifdef ENABLE_EMI_CHKER
	/* Enable AXI 4KB boundary violation monitor timer */
	//emi_axi_set_chker(1 << AXI_ADR_CHK_EN);
	//add_timer_on(&emi_axi_vio_timer, 0);
#endif

	/* register driver and create sysfs files */
	ret = platform_driver_register(&emi_mpu_ctrl);
	if (ret) {
		pr_err("Fail to register EMI_MPU driver.\n");
	}
	ret = driver_create_file(&emi_mpu_ctrl.driver, &driver_attr_mpu_config);
	if (ret) {
		pr_err("Fail to create MPU config sysfs file.\n");
	}
#ifdef ENABLE_EMI_CHKER
	ret = driver_create_file(&emi_mpu_ctrl.driver, &driver_attr_emi_axi_vio);
	if (ret) {
		pr_err("Fail to create AXI violation monitor sysfs file.\n");
	}
#endif
	ret = driver_create_file(&emi_mpu_ctrl.driver, &driver_attr_pgt_scan);
	if (ret) {
		pr_err("Fail to create pgt scan sysfs file.\n");
	}
#ifdef ENABLE_EMI_WATCH_POINT
	ret = driver_create_file(&emi_mpu_ctrl.driver, &driver_attr_emi_wp_vio);
	if (ret) {
		pr_err("Fail to create WP violation monitor sysfs file.\n");
	}
#endif
#endif

	//atomic_notifier_chain_register(&panic_notifier_list, &emi_mpu_blk);

	/* Create a workqueue to search pagetable entry */
	emi_mpu_workqueue = create_singlethread_workqueue("emi_mpu");
	INIT_WORK(&emi_mpu_work, emi_mpu_work_callback);
	/*Init for testing*/
	//emi_mpu_set_region_protection(0x9c000000,
	//							  0x9d7fffff, /*MD_IMG_REGION_LEN*/
	//									0,	/*region*/
	//									SET_ACCESS_PERMISSON(SEC_R_NSEC_R, SEC_R_NSEC_R, SEC_R_NSEC_R, SEC_R_NSEC_R));


	return 0;
}

static void __exit emi_mpu_mod_exit(void)
{
}

module_init(emi_mpu_mod_init);
module_exit(emi_mpu_mod_exit);

EXPORT_SYMBOL(emi_mpu_set_region_protection);
//EXPORT_SYMBOL(start_mm_mau_protect);


unsigned int enable_4G(void)
{
	return enable_4gb;
}


static int __init dram_4gb_init(void)
{
	void __iomem *INFRA_BASE_ADDR = NULL;
	void __iomem *PERISYS_BASE_ADDR = NULL;
	struct device_node *node;
	unsigned int infra_4g_sp, perisis_4g_sp;

	node = of_find_compatible_node(NULL, NULL, "mediatek,infracfg_ao");

	if (!node)
		pr_err("find INFRACFG_AO node failed\n");

	INFRA_BASE_ADDR = of_iomap(node, 0);

	if (!INFRA_BASE_ADDR)
		pr_err("INFRACFG_AO ioremap failed\n");

	node = of_find_compatible_node(NULL, NULL, "mediatek,pericfg");

	if (!node)
		pr_err("find PERICFG node failed\n");

	PERISYS_BASE_ADDR = of_iomap(node, 0);

	if (!PERISYS_BASE_ADDR)
		pr_err("PERISYS_BASE_ADDR ioremap failed\n");

	infra_4g_sp = readl(IOMEM(INFRA_BASE_ADDR + 0xf00)) & (1 << 13);
	perisis_4g_sp = readl(IOMEM(PERISYS_BASE_ADDR + 0x208)) & (1 << 15);

	pr_err("infra = 0x%x   perisis = 0x%x   result = %d\n",
	       infra_4g_sp,
	       perisis_4g_sp,
	       (infra_4g_sp && perisis_4g_sp));

	if (infra_4g_sp && perisis_4g_sp) {
		enable_4gb = 1;
		pr_err("[EMI MPU] 4G mode\n");
	} else {
		enable_4gb = 0;
		pr_err("[EMI MPU] Not 4G mode\n");
	}

	return 0;

}
early_initcall(dram_4gb_init);


