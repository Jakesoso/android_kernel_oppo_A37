#ifndef ECCCI_INTERNAL_OPTION
#define ECCCI_INTERNAL_OPTION

#include <mach/mt_reg_base.h>

/*================================================ */
/*Bool option part*/
/*================================================*/
/*#define CCCI_STATISTIC*/
#define FEATURE_GET_MD_GPIO_NUM
#define FEATURE_GET_MD_GPIO_VAL
#define FEATURE_GET_MD_ADC_NUM
#define FEATURE_GET_MD_ADC_VAL
#define FEATURE_GET_MD_EINT_ATTR
#if defined(FEATURE_GET_MD_EINT_ATTR) && !defined(CONFIG_MTK_LEGACY)
#define FEATURE_GET_MD_EINT_ATTR_DTS
#endif

#define FEATURE_GET_MD_BAT_VOL
#define FEATURE_PM_IPO_H
/*#define FEATURE_DFO_EN*/
#define FEATURE_SEQ_CHECK_EN
#define FEATURE_POLL_MD_EN

#define FEATURE_DHL_LOG_EN
#define FEATURE_MD1MD3_SHARE_MEM

#if 0 /*DEPRECATED */
#define FEATURE_GET_TD_EINT_NUM
#define FEATURE_GET_DRAM_TYPE_CLK
#endif
//#ifdef VENDOR_EDIT
//pw@Network.Sim,2016-6-3,Revert the VPP-DATA card issue's modify ,to resolve the NFC issue : 811157
/*
#ifdef CONFIG_NFC_MT6605
#define FEATURE_INFORM_NFC_VSIM_CHANGE
#endif
*/
//#endif /* VENDOR_EDIT */
#define ENABLE_DRAM_API
#define ENABLE_MEM_REMAP_HW
/*#define ENABLE_CHIP_VER_CHECK*/
/*#define ENABLE_2G_3G_CHECK*/
/*#define ENABLE_MD_WDT_DBG*/
#define ENABLE_CLDMA_AP_SIDE
#define ENABLE_MD_POWER_OFF_CHECK

#ifdef CONFIG_MTK_CONN_MD
#define FEATURE_CONN_MD_EXP_EN
#endif

#define FEATURE_USING_4G_MEMORY_API
/*#define FEATURE_LOW_BATTERY_SUPPORT disable for customer complaint*/
#ifdef CONFIG_MTK_FPGA
#define FEATURE_FPGA_PORTING
#else
#define FEATURE_RF_CLK_BUF
#define ENABLE_32K_CLK_LESS
#define FEATURE_MD_GET_CLIB_TIME
#define FEATURE_C2K_ALWAYS_ON
#define FEATURE_DBM_SUPPORT

#define ENABLE_EMI_PROTECTION
#ifdef ENABLE_EMI_PROTECTION
#define SET_EMI_STEP_BY_STAGE
/* #define SET_AP_MPU_REGION */ /*no need set ap region */
#endif

#endif
/* #define DISABLE_MD_WDT_PROCESS */ /* enable wdt after bringup */
#define NO_POWER_OFF_ON_STARTMD
#define NO_START_ON_SUSPEND_RESUME
#define MD_CACHE_TO_NONECACHE
#define MD_UMOLY_EE_SUPPORT
#define TEST_MESSAGE_FOR_BRINGUP

#define CCCI_SMEM_DUMP_SIZE      4096/* smem size we dump when EE */
#define CCCI_SMEM_SIZE_EXCEPTION 0x10000/* exception smem total size */
#define CCCI_SMEM_SIZE_RUNTIME_AP 	0x800/* AP runtime data size */
#define CCCI_SMEM_SIZE_RUNTIME_MD 	0x800/* MD runtime data size */
#define CCCI_SMEM_SIZE_RUNTIME	(CCCI_SMEM_SIZE_RUNTIME_AP+CCCI_SMEM_SIZE_RUNTIME_MD)
#define CCCI_SMEM_OFFSET_EXREC 2048/* where the exception record begain in smem */
#define CCCC_SMEM_CCIF_SRAM_SIZE 16
#define CCCI_SMEM_OFFSET_CCIF_SRAM (CCCI_SMEM_OFFSET_EXREC+1024-CCCC_SMEM_CCIF_SRAM_SIZE)
#define CCCI_SMEM_OFFSET_EPON 0xC64
#define CCCI_SMEM_OFFSET_EPON_UMOLY 0x1830
#define CCCI_SMEM_OFFSET_SEQERR 0x34
#define CCCI_SMEM_OFFSET_CCCI_DEBUG 0 /* where the MD CCCI debug info begain in smem */
#define CCCI_SMEM_CCCI_DEBUG_SIZE 2048 /* MD CCCI debug info size */
#define CCCI_SMEM_OFFSET_MDSS_DEBUG 2048 /* where the MD SS debug info begain in smem */
#define CCCI_SMEM_MDSS_DEBUG_SIZE_UMOLY  8192 /* MD SS debug info size for MD1 */
#define CCCI_SMEM_MDSS_DEBUG_SIZE 2048 /* MD SS debug info size except MD1 */
#define CCCI_SMEM_SLEEP_MODE_DBG_SIZE 1024 /* MD sleep mode debug info section in smem tail */
#define CCCI_SMEM_SLEEP_MODE_DBG_DUMP 512 /* only dump first 512bytes in sleep mode info */
#define CCCI_SMEM_MD1_DBM_OFFSET (64*1024-16*3-8)
#define CCCI_SMEM_MD3_DBM_OFFSET (CCCI_SMEM_SIZE_EXCEPTION-16*3-CCCI_SMEM_DBM_GUARD_SIZE)
#define CCCI_SMEM_DBM_GUARD_SIZE (8)

/*================================================ */
/*Configure value option part*/
/*================================================*/
#define AP_PLATFORM_INFO    "MT6755E1"
#define CCCI_MTU            (3584-128)
#define CCCI_NET_MTU        (1500)
#define SKB_POOL_SIZE_4K    (256)	/*2*MD */
#define SKB_POOL_SIZE_1_5K  (256)	/*2*MD */
#define SKB_POOL_SIZE_16    (64)	/*2*MD */
#define BM_POOL_SIZE        (SKB_POOL_SIZE_4K+SKB_POOL_SIZE_1_5K+SKB_POOL_SIZE_16)
#define RELOAD_TH            3	/*reload pool if pool size dropped below 1/RELOAD_TH */
#define MD_HEADER_VER_NO    (3)
#define MEM_LAY_OUT_VER     (1)
#define AP_MD_HS_V2          2	/*handshake version*/

#define CCCI_MEM_ALIGN      (SZ_32M)
#define CCCI_SMEM_ALIGN_MD1 (0x200000)	/*2M */
#define CCCI_SMEM_ALIGN_MD2 (0x200000)	/*2M */

#define CURR_SEC_CCCI_SYNC_VER (1)	/*Note: must sync with sec lib, if ccci and sec has dependency change */
#define CCCI_DRIVER_VER     0x20110118

#define IPC_L4C_MSG_ID_LEN   (0x40)
#ifndef VENDOR_EDIT
// Jingchun.Wang@Phone.Bsp.Driver, 2016/01/08  Delete for del logs 
#define CCCI_LOG_LEVEL  (0)
#else
#define CCCI_LOG_LEVEL	(5)
#endif /*VENDOR_EDIT*/
#endif
