#ifndef _MT_PMIC_UPMU_SW_MT6353_H_
#define _MT_PMIC_UPMU_SW_MT6353_H_
#include <mach/upmu_hw.h>
#include <mach/mt_chip.h>

#define AUXADC_SUPPORT_IMM_CURRENT_MODE
//#define BATTERY_DTS_SUPPORT
#define BATTERY_SW_INIT
/* #define RBAT_PULL_UP_VOLT_BY_BIF */

/*==============================================================================
 * Low battery level define
 * ============================================================================== */
typedef enum LOW_BATTERY_LEVEL_TAG {
	LOW_BATTERY_LEVEL_0 = 0,
	LOW_BATTERY_LEVEL_1 = 1,
	LOW_BATTERY_LEVEL_2 = 2
} LOW_BATTERY_LEVEL;

typedef enum LOW_BATTERY_PRIO_TAG {
	LOW_BATTERY_PRIO_CPU_B = 0,
	LOW_BATTERY_PRIO_CPU_L = 1,
	LOW_BATTERY_PRIO_GPU = 2,
	LOW_BATTERY_PRIO_MD = 3,
	LOW_BATTERY_PRIO_MD5 = 4,
	LOW_BATTERY_PRIO_FLASHLIGHT = 5,
	LOW_BATTERY_PRIO_VIDEO = 6,
	LOW_BATTERY_PRIO_WIFI = 7,
	LOW_BATTERY_PRIO_BACKLIGHT = 8
} LOW_BATTERY_PRIO;

extern void (*low_battery_callback)(LOW_BATTERY_LEVEL);
extern void register_low_battery_notify(void (*low_battery_callback) (LOW_BATTERY_LEVEL),
					LOW_BATTERY_PRIO prio_val);


/* ==============================================================================
 * Battery OC level define
 * ============================================================================== */
typedef enum BATTERY_OC_LEVEL_TAG {
	BATTERY_OC_LEVEL_0 = 0,
	BATTERY_OC_LEVEL_1 = 1
} BATTERY_OC_LEVEL;

typedef enum BATTERY_OC_PRIO_TAG {
	BATTERY_OC_PRIO_CPU_B = 0,
	BATTERY_OC_PRIO_CPU_L = 1,
	BATTERY_OC_PRIO_GPU = 2
} BATTERY_OC_PRIO;

extern void (*battery_oc_callback)(BATTERY_OC_LEVEL);
extern void register_battery_oc_notify(void (*battery_oc_callback) (BATTERY_OC_LEVEL),
				       BATTERY_OC_PRIO prio_val);

/*==============================================================================
 * Battery percent define
 *==============================================================================*/
typedef enum BATTERY_PERCENT_LEVEL_TAG {
	BATTERY_PERCENT_LEVEL_0 = 0,
	BATTERY_PERCENT_LEVEL_1 = 1
} BATTERY_PERCENT_LEVEL;

typedef enum BATTERY_PERCENT_PRIO_TAG {
	BATTERY_PERCENT_PRIO_CPU_B = 0,
	BATTERY_PERCENT_PRIO_CPU_L = 1,
	BATTERY_PERCENT_PRIO_GPU = 2,
	BATTERY_PERCENT_PRIO_MD = 3,
	BATTERY_PERCENT_PRIO_MD5 = 4,
	BATTERY_PERCENT_PRIO_FLASHLIGHT = 5,
	BATTERY_PERCENT_PRIO_VIDEO = 6,
	BATTERY_PERCENT_PRIO_WIFI = 7,
	BATTERY_PERCENT_PRIO_BACKLIGHT = 8
} BATTERY_PERCENT_PRIO;

extern void (*battery_percent_callback)(BATTERY_PERCENT_LEVEL);
extern void
register_battery_percent_notify(void (*battery_percent_callback) (BATTERY_PERCENT_LEVEL),
				BATTERY_PERCENT_PRIO prio_val);

/* ADC Channel Number */
typedef enum {
	PMIC_AUX_BATSNS_AP =		0x000,
	PMIC_AUX_ISENSE_AP,
	PMIC_AUX_VCDT_AP,
	PMIC_AUX_BATON_AP, /* BATON/BIF */
	PMIC_AUX_CH4,
	PMIC_AUX_VACCDET_AP,
	PMIC_AUX_CH6,
	PMIC_AUX_TSX,
	PMIC_AUX_CH8,
	PMIC_AUX_CH9,
	PMIC_AUX_CH10,
	PMIC_AUX_CH11,
	PMIC_AUX_CH12,
	PMIC_AUX_CH13,
	PMIC_AUX_CH14,
	PMIC_AUX_CH15,
	PMIC_AUX_CH16,
	PMIC_AUX_CH4_DCXO = 0x10000004,
} pmic_adc_ch_list_enum;

/*==============================================================================
 * DLPT define
 *==============================================================================*/
typedef enum DLPT_PRIO_TAG {
	DLPT_PRIO_PBM = 0,
	DLPT_PRIO_CPU_B = 1,
	DLPT_PRIO_CPU_L = 2,
	DLPT_PRIO_GPU = 3,
	DLPT_PRIO_MD = 4,
	DLPT_PRIO_MD5 = 5,
	DLPT_PRIO_FLASHLIGHT = 6,
	DLPT_PRIO_VIDEO = 7,
	DLPT_PRIO_WIFI = 8,
	DLPT_PRIO_BACKLIGHT = 9
} DLPT_PRIO;

extern void (*dlpt_callback)(unsigned int);
extern void register_dlpt_notify(void (*dlpt_callback)(unsigned int), DLPT_PRIO prio_val);
extern const PMU_FLAG_TABLE_ENTRY pmu_flags_table[];

extern unsigned short is_battery_remove;
extern unsigned short is_wdt_reboot_pmic;
extern unsigned short is_wdt_reboot_pmic_chk;
extern unsigned int g_pmic_pad_vbif28_vol;

/*==============================================================================
// PMIC LDO define
//==============================================================================*/

/*==============================================================================
// PMIC auxadc define
//==============================================================================*/
extern signed int g_I_SENSE_offset;
extern void pmic_auxadc_init(void);
extern void pmic_auxadc_lock(void);
extern void pmic_auxadc_unlock(void);
extern void mt_power_off(void);
/*==============================================================================
// PMIC fg define
//==============================================================================*/
extern unsigned int bat_get_ui_percentage(void);
extern signed int fgauge_read_v_by_d(int d_val);
extern signed int fgauge_read_r_bat_by_v(signed int voltage);
extern signed int fgauge_read_IM_current(void *data);
extern void kpd_pwrkey_pmic_handler(unsigned long pressed);
extern void kpd_pmic_rstkey_handler(unsigned long pressed);
extern int is_mt6311_sw_ready(void);
extern int is_mt6311_exist(void);
extern int get_mt6311_i2c_ch_num(void);
extern bool crystal_exist_status(void);
#if defined CONFIG_MTK_LEGACY
extern void pmu_drv_tool_customization_init(void);
#endif
extern int batt_init_cust_data(void);
extern void PMIC_INIT_SETTING_V1(void);
extern chip_sw_ver_t mt_get_chip_sw_ver(void);

#endif /* _MT_PMIC_UPMU_SW_MT6353_H_ */

