#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include <asm/uaccess.h>
#include <mach/mt_typedefs.h>
#include <mach/hardware.h>
#include <mach/mt_boot.h>
#include <mach/mt_boot_reason.h>

#include <mach/battery_common.h>
#include <mach/battery_meter.h>
#include <mach/battery_meter_hal.h>
#include "cust_battery_meter.h"
#ifndef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2015.9.30 for charging
//#include "cust_battery_meter_table.h"
#else/*VENDOR_EDIT*/
#include "cust_battery_meter_table_meter.h"
#endif/*VENDOR_EDIT*/
#include "cust_pmic.h"
#include "mach/mtk_rtc.h"

#include <mach/upmu_common.h>


/* ============================================================ // */
/* define */
/* ============================================================ // */
#define PROFILE_SIZE 4

static DEFINE_MUTEX(FGADC_mutex);

int Enable_FGADC_LOG = 1;

/* ============================================================ // */
/* global variable */
/* ============================================================ // */
BATTERY_METER_CONTROL battery_meter_ctrl = NULL;

/* static struct proc_dir_entry *proc_entry_fgadc; */
static char proc_fgadc_data[32];

kal_bool gFG_Is_Charging = KAL_FALSE;
kal_int32 g_auxadc_solution = 0;
U32 g_spm_timer = 600;
BOOL bat_spm_timeout = false;
U32 _g_bat_sleep_total_time = NORMAL_WAKEUP_PERIOD;
#ifdef MTK_ENABLE_AGING_ALGORITHM
U32 suspend_time = 0;
#endif
kal_int32 g_booting_vbat = 0;
static U32 temperature_change = 1;

#if defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
static kal_int32 g_currentfactor = 100;
static kal_bool g_USE_UI_SOC = KAL_TRUE;
#if defined(CUST_SYSTEM_OFF_VOLTAGE)
#define SYSTEM_OFF_VOLTAGE CUST_SYSTEM_OFF_VOLTAGE
#endif
#endif

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // PMIC AUXADC Related Variable */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
int g_R_BAT_SENSE;		/* R_BAT_SENSE; */
int g_R_I_SENSE;		/* R_I_SENSE; */
int g_R_CHARGER_1;		/* R_CHARGER_1; */
int g_R_CHARGER_2;		/* R_CHARGER_2; */

int fg_qmax_update_for_aging_flag = 1;

/* HW FG */
kal_int32 gFG_DOD0 = 0;
kal_int32 gFG_DOD1 = 0;
kal_int32 gFG_columb = 0;
kal_int32 gFG_voltage = 0;
kal_int32 gFG_current = 0;
kal_int32 gFG_capacity = 0;
kal_int32 gFG_capacity_by_c = 0;
kal_int32 gFG_capacity_by_c_init = 0;
kal_int32 gFG_capacity_by_v = 0;
kal_int32 gFG_capacity_by_v_init = 0;
kal_int32 gFG_temp = 100;
kal_int32 gFG_resistance_bat = 0;
kal_int32 gFG_compensate_value = 0;
kal_int32 gFG_ori_voltage = 0;
kal_int32 gFG_BATT_CAPACITY = 0;
kal_int32 gFG_voltage_init = 0;
kal_int32 gFG_current_auto_detect_R_fg_total = 0;
kal_int32 gFG_current_auto_detect_R_fg_count = 0;
kal_int32 gFG_current_auto_detect_R_fg_result = 0;
kal_int32 gFG_15_vlot = 3700;
kal_int32 gFG_BATT_CAPACITY_init_high_current = 1200;
kal_int32 gFG_BATT_CAPACITY_aging = 1200;

/* voltage mode */
kal_int32 gfg_percent_check_point = 50;
kal_int32 volt_mode_update_timer = 0;
kal_int32 volt_mode_update_time_out = 6;	/* 1mins */

/* EM */
kal_int32 g_fg_dbg_bat_volt = 0;
kal_int32 g_fg_dbg_bat_current = 0;
kal_int32 g_fg_dbg_bat_zcv = 0;
kal_int32 g_fg_dbg_bat_temp = 0;
kal_int32 g_fg_dbg_bat_r = 0;
kal_int32 g_fg_dbg_bat_car = 0;
kal_int32 g_fg_dbg_bat_qmax = 0;
kal_int32 g_fg_dbg_d0 = 0;
kal_int32 g_fg_dbg_d1 = 0;
kal_int32 g_fg_dbg_percentage = 0;
kal_int32 g_fg_dbg_percentage_fg = 0;
kal_int32 g_fg_dbg_percentage_voltmode = 0;

kal_int32 FGvbatVoltageBuffer[FG_VBAT_AVERAGE_SIZE];
kal_int32 FGbatteryIndex = 0;
kal_int32 FGbatteryVoltageSum = 0;
kal_int32 gFG_voltage_AVG = 0;
kal_int32 gFG_vbat_offset = 0;
#ifdef Q_MAX_BY_CURRENT
kal_int32 FGCurrentBuffer[FG_CURRENT_AVERAGE_SIZE];
kal_int32 FGCurrentIndex = 0;
kal_int32 FGCurrentSum = 0;
kal_int32 gFG_current_AVG = 0;
#endif
kal_int32 g_tracking_point;	/* CUST_TRACKING_POINT; */
kal_int32 g_rtc_fg_soc = 0;
kal_int32 g_I_SENSE_offset = 0;

/* SW FG */
kal_int32 oam_v_ocv_init = 0;
kal_int32 oam_v_ocv_1 = 0;
kal_int32 oam_v_ocv_2 = 0;
kal_int32 oam_r_1 = 0;
kal_int32 oam_r_2 = 0;
kal_int32 oam_d0 = 0;
kal_int32 oam_i_ori = 0;
kal_int32 oam_i_1 = 0;
kal_int32 oam_i_2 = 0;
kal_int32 oam_car_1 = 0;
kal_int32 oam_car_2 = 0;
kal_int32 oam_d_1 = 1;
kal_int32 oam_d_2 = 1;
kal_int32 oam_d_3 = 1;
kal_int32 oam_d_3_pre = 0;
kal_int32 oam_d_4 = 0;
kal_int32 oam_d_4_pre = 0;
kal_int32 oam_d_5 = 0;
kal_int32 oam_init_i = 0;
kal_int32 oam_run_i = 0;
kal_int32 d5_count = 0;
kal_int32 d5_count_time = 60;
kal_int32 d5_count_time_rate = 1;
kal_int32 g_d_hw_ocv = 0;
kal_int32 g_vol_bat_hw_ocv = 0;
kal_int32 g_hw_ocv_before_sleep = 0;
struct timespec g_rtc_time_before_sleep, xts_before_sleep;
kal_int32 g_sw_vbat_temp = 0;
struct timespec last_oam_run_time;

/* aging mechanism */
#ifdef MTK_ENABLE_AGING_ALGORITHM

static kal_int32 aging_ocv_1;
static kal_int32 aging_ocv_2;
static kal_int32 aging_car_1;
static kal_int32 aging_car_2;
static kal_int32 aging_dod_1;
static kal_int32 aging_dod_2;
#ifdef MD_SLEEP_CURRENT_CHECK
static kal_int32 columb_before_sleep = 0x123456;
#endif
/* static time_t aging_resume_time_1 = 0; */
/* static time_t aging_resume_time_2 = 0; */

#ifndef SELF_DISCHARGE_CHECK_THRESHOLD
#define SELF_DISCHARGE_CHECK_THRESHOLD 10
#endif

#ifndef OCV_RECOVER_TIME
#define OCV_RECOVER_TIME 2100
#endif

#ifndef DOD1_ABOVE_THRESHOLD
#define DOD1_ABOVE_THRESHOLD 30
#endif

#ifndef DOD2_BELOW_THRESHOLD
#define DOD2_BELOW_THRESHOLD 70
#endif

#ifndef MIN_DOD_DIFF_THRESHOLD
#define MIN_DOD_DIFF_THRESHOLD 60
#endif

#ifndef MIN_AGING_FACTOR
#define MIN_AGING_FACTOR 90
#endif

#endif				/* aging mechanism */

/* battery info */
#ifdef MTK_BATTERY_LIFETIME_DATA_SUPPORT

kal_int32 gFG_battery_cycle = 0;
kal_int32 gFG_aging_factor = 100;
kal_int32 gFG_columb_sum = 0;
kal_int32 gFG_pre_columb_count = 0;

kal_int32 gFG_max_voltage = 0;
kal_int32 gFG_min_voltage = 10000;
kal_int32 gFG_max_current = 0;
kal_int32 gFG_min_current = 0;
kal_int32 gFG_max_temperature = -20;
kal_int32 gFG_min_temperature = 100;

#endif				/* battery info */

/*extern char *saved_command_line;*/
/* Temperature window size */
#define TEMP_AVERAGE_SIZE	30

kal_bool gFG_Is_offset_init = KAL_FALSE;

#ifdef MTK_MULTI_BAT_PROFILE_SUPPORT
/*extern int IMM_GetOneChannelValue_Cali(int Channel, int *voltage);*/
kal_uint32 g_fg_battery_id = 0;

#ifdef MTK_GET_BATTERY_ID_BY_AUXADC
void fgauge_get_profile_id(void)
{
	int id_volt = 0;
	int id = 0;
	int ret = 0;

	ret = IMM_GetOneChannelValue_Cali(BATTERY_ID_CHANNEL_NUM, &id_volt);
	if (ret != 0)
		bm_print(BM_LOG_CRTI, "[fgauge_get_profile_id]id_volt read fail\n");
	else
		bm_print(BM_LOG_CRTI, "[fgauge_get_profile_id]id_volt = %d\n", id_volt);

	if ((sizeof(g_battery_id_voltage) / sizeof(kal_int32)) != TOTAL_BATTERY_NUMBER) {
		bm_print(BM_LOG_CRTI, "[fgauge_get_profile_id]error! voltage range incorrect!\n");
		return;
	}

	for (id = 0; id < TOTAL_BATTERY_NUMBER; id++) {
		if (id_volt < g_battery_id_voltage[id]) {
			g_fg_battery_id = id;
			break;
		} else if (g_battery_id_voltage[id] == -1) {
			g_fg_battery_id = TOTAL_BATTERY_NUMBER - 1;
		}
	}

	bm_print(BM_LOG_CRTI, "[fgauge_get_profile_id]Battery id (%d)\n", g_fg_battery_id);
}
#elif defined(MTK_GET_BATTERY_ID_BY_GPIO)
void fgauge_get_profile_id(void)
{
	g_fg_battery_id = 0;
}
#else
void fgauge_get_profile_id(void)
{
	g_fg_battery_id = 0;
}
#endif
#endif

/* ============================================================ // */
/* function prototype */
/* ============================================================ // */

/* ============================================================ // */
/* extern variable */
/* ============================================================ // */

/* ============================================================ // */
/* extern function */
/* ============================================================ // */
/* extern int get_rtc_spare_fg_value(void); */
/* extern unsigned long rtc_read_hw_time(void); */


struct battery_custom_data batt_cust_data;
struct battery_meter_custom_data batt_meter_cust_data;


int __batt_init_cust_data_from_cust_header(void)
{
	/* cust_charging.h */
	/* stop charging while in talking mode */
#if defined(STOP_CHARGING_IN_TAKLING)
	batt_cust_data.stop_charging_in_takling = 1;
#else				/* #if defined(STOP_CHARGING_IN_TAKLING) */
	batt_cust_data.stop_charging_in_takling = 0;
#endif				/* #if defined(STOP_CHARGING_IN_TAKLING) */

#if defined(TALKING_RECHARGE_VOLTAGE)
	batt_cust_data.talking_recharge_voltage = TALKING_RECHARGE_VOLTAGE;
#endif

#if defined(TALKING_SYNC_TIME)
	batt_cust_data.talking_sync_time = TALKING_SYNC_TIME;
#endif

	/* Battery Temperature Protection */
#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
	batt_cust_data.mtk_temperature_recharge_support = 1;
#else				/* #if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT) */
	batt_cust_data.mtk_temperature_recharge_support = 0;
#endif				/* #if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT) */

#if defined(MAX_CHARGE_TEMPERATURE)
	batt_cust_data.max_charge_temperature = MAX_CHARGE_TEMPERATURE;
#endif

#if defined(MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE)
	batt_cust_data.max_charge_temperature_minus_x_degree =
	    MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE;
#endif

#if defined(MIN_CHARGE_TEMPERATURE)
	batt_cust_data.min_charge_temperature = MIN_CHARGE_TEMPERATURE;
#endif

#if defined(MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE)
	batt_cust_data.min_charge_temperature_plus_x_degree = MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE;
#endif

#if defined(ERR_CHARGE_TEMPERATURE)
	batt_cust_data.err_charge_temperature = ERR_CHARGE_TEMPERATURE;
#endif

	/* Linear Charging Threshold */
#if defined(V_PRE2CC_THRES)
	batt_cust_data.v_pre2cc_thres = V_PRE2CC_THRES;
#endif
#if defined(V_CC2TOPOFF_THRES)
	batt_cust_data.v_cc2topoff_thres = V_CC2TOPOFF_THRES;
#endif
#if defined(RECHARGING_VOLTAGE)
	batt_cust_data.recharging_voltage = RECHARGING_VOLTAGE;
#endif
#if defined(CHARGING_FULL_CURRENT)
	batt_cust_data.charging_full_current = CHARGING_FULL_CURRENT;
#endif

	/* Charging Current Setting */
#if defined(CONFIG_USB_IF)
	batt_cust_data.config_usb_if = 1;
#else				/* #if defined(CONFIG_USB_IF) */
	batt_cust_data.config_usb_if = 0;
#endif				/* #if defined(CONFIG_USB_IF) */

#if defined(USB_CHARGER_CURRENT_SUSPEND)
	batt_cust_data.usb_charger_current_suspend = USB_CHARGER_CURRENT_SUSPEND;
#endif
#if defined(USB_CHARGER_CURRENT_UNCONFIGURED)
	batt_cust_data.usb_charger_current_unconfigured = USB_CHARGER_CURRENT_UNCONFIGURED;
#endif
#if defined(USB_CHARGER_CURRENT_CONFIGURED)
	batt_cust_data.usb_charger_current_configured = USB_CHARGER_CURRENT_CONFIGURED;
#endif
#if defined(USB_CHARGER_CURRENT)
	batt_cust_data.usb_charger_current = USB_CHARGER_CURRENT;
#endif
#if defined(AC_CHARGER_CURRENT)
	batt_cust_data.ac_charger_current = AC_CHARGER_CURRENT;
#endif
#if defined(NON_STD_AC_CHARGER_CURRENT)
	batt_cust_data.non_std_ac_charger_current = NON_STD_AC_CHARGER_CURRENT;
#endif
#if defined(CHARGING_HOST_CHARGER_CURRENT)
	batt_cust_data.charging_host_charger_current = CHARGING_HOST_CHARGER_CURRENT;
#endif
#if defined(APPLE_0_5A_CHARGER_CURRENT)
	batt_cust_data.apple_0_5a_charger_current = APPLE_0_5A_CHARGER_CURRENT;
#endif
#if defined(APPLE_1_0A_CHARGER_CURRENT)
	batt_cust_data.apple_1_0a_charger_current = APPLE_1_0A_CHARGER_CURRENT;
#endif
#if defined(APPLE_2_1A_CHARGER_CURRENT)
	batt_cust_data.apple_2_1a_charger_current = APPLE_2_1A_CHARGER_CURRENT;
#endif

	/* Precise Tunning
	   batt_cust_data.battery_average_data_number =
	   BATTERY_AVERAGE_DATA_NUMBER;
	   batt_cust_data.battery_average_size = BATTERY_AVERAGE_SIZE;
	 */


	/* charger error check */
#if defined(BAT_LOW_TEMP_PROTECT_ENABLE)
	batt_cust_data.bat_low_temp_protect_enable = 1;
#else				/* #if defined(BAT_LOW_TEMP_PROTECT_ENABLE) */
	batt_cust_data.bat_low_temp_protect_enable = 0;
#endif				/* #if defined(BAT_LOW_TEMP_PROTECT_ENABLE) */

#if defined(V_CHARGER_ENABLE)
	batt_cust_data.v_charger_enable = V_CHARGER_ENABLE;
#endif
#if defined(V_CHARGER_MAX)
	batt_cust_data.v_charger_max = V_CHARGER_MAX;
#endif
#if defined(V_CHARGER_MIN)
	batt_cust_data.v_charger_min = V_CHARGER_MIN;
#endif


	/* Tracking TIME */
#if defined(ONEHUNDRED_PERCENT_TRACKING_TIME)
	batt_cust_data.onehundred_percent_tracking_time = ONEHUNDRED_PERCENT_TRACKING_TIME;
#endif
#if defined(NPERCENT_TRACKING_TIME)
	batt_cust_data.npercent_tracking_time = NPERCENT_TRACKING_TIME;
#endif
#if defined(SYNC_TO_REAL_TRACKING_TIME)
	batt_cust_data.sync_to_real_tracking_time = SYNC_TO_REAL_TRACKING_TIME;
#endif
#if defined(V_0PERCENT_TRACKING)
	batt_cust_data.v_0percent_tracking = V_0PERCENT_TRACKING;
#endif

	/* High battery support */
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
	batt_cust_data.high_battery_voltage_support = 1;
#else				/* #if defined(HIGH_BATTERY_VOLTAGE_SUPPORT) */
	batt_cust_data.high_battery_voltage_support = 0;
#endif				/* #if defined(HIGH_BATTERY_VOLTAGE_SUPPORT) */

	return 0;
}


#ifdef CONFIG_OF
int __batt_init_cust_data_from_dt(void)
{
	/* struct device_node *np = dev->dev.of_node; */
	struct device_node *np;
	u32 val;
	/* char *path = "/BAT_METTER"; */
	char *path = "/bus/BAT_METTER";
	struct device_node *dt_node;
	const u32 *property;
	int len;

	np = of_find_node_by_path(path);
	if (!np) {
		/* printk(KERN_ERR "(E) Failed to find device-tree node: %s\n", path); */
		battery_log(BAT_LOG_CRTI, "Failed to find device-tree node: %s\n", path);
		return -ENODEV;
	}

	if (of_property_read_u32(np, "stop_charging_in_takling", &val) == 0) {
		batt_cust_data.stop_charging_in_takling = (int)val;
		battery_log(BAT_LOG_CRTI, "Get stop_charging_in_takling: %d\n",
			    batt_cust_data.stop_charging_in_takling);
	} else {
		battery_log(BAT_LOG_CRTI, "Get stop_charging_in_takling failed\n");
	}

	if (of_property_read_u32(np, "talking_recharge_voltage", &val) == 0) {
		batt_cust_data.talking_recharge_voltage = (int)val;
		battery_log(BAT_LOG_CRTI, "Get talking_recharge_voltage: %d\n",
			    batt_cust_data.talking_recharge_voltage);
	} else {
		battery_log(BAT_LOG_CRTI, "Get talking_recharge_voltage failed\n");
	}

	if (of_property_read_u32(np, "talking_sync_time", &val) == 0) {
		batt_cust_data.talking_sync_time = (int)val;
		battery_log(BAT_LOG_CRTI, "Get talking_sync_time: %d\n",
			    batt_cust_data.talking_sync_time);
	} else {
		battery_log(BAT_LOG_CRTI, "Get talking_sync_time failed\n");
	}

	if (of_property_read_u32(np, "mtk_temperature_recharge_support", &val) == 0) {
		batt_cust_data.mtk_temperature_recharge_support = (int)val;
		battery_log(BAT_LOG_CRTI, "Get mtk_temperature_recharge_support: %d\n",
			    batt_cust_data.mtk_temperature_recharge_support);
	} else {
		battery_log(BAT_LOG_CRTI, "Get mtk_temperature_recharge_support failed\n");
	}

	if (of_property_read_u32(np, "max_charge_temperature", &val) == 0) {
		batt_cust_data.max_charge_temperature = (int)val;
		battery_log(BAT_LOG_CRTI, "Get max_charge_temperature: %d\n",
			    batt_cust_data.max_charge_temperature);
	} else {
		battery_log(BAT_LOG_CRTI, "Get max_charge_temperature failed\n");
	}

	if (of_property_read_u32(np, "max_charge_temperature_minus_x_degree", &val) == 0) {
		batt_cust_data.max_charge_temperature_minus_x_degree = (int)val;
		battery_log(BAT_LOG_CRTI, "Get max_charge_temperature_minus_x_degree: %d\n",
			    batt_cust_data.max_charge_temperature_minus_x_degree);
	} else {
		battery_log(BAT_LOG_CRTI, "Get max_charge_temperature_minus_x_degree failed\n");
	}

	if (of_property_read_u32(np, "min_charge_temperature", &val) == 0) {
		batt_cust_data.min_charge_temperature = (int)val;
		battery_log(BAT_LOG_CRTI, "Get min_charge_temperature: %d\n",
			    batt_cust_data.min_charge_temperature);
	} else {
		battery_log(BAT_LOG_CRTI, "Get min_charge_temperature failed\n");
	}

	if (of_property_read_u32(np, "min_charge_temperature_plus_x_degree", &val) == 0) {
		batt_cust_data.min_charge_temperature_plus_x_degree = (int)val;
		battery_log(BAT_LOG_CRTI, "Get min_charge_temperature_plus_x_degree: %d\n",
			    batt_cust_data.min_charge_temperature_plus_x_degree);
	} else {
		battery_log(BAT_LOG_CRTI, "Get min_charge_temperature_plus_x_degree failed\n");
	}

	if (of_property_read_u32(np, "err_charge_temperature", &val) == 0) {
		batt_cust_data.err_charge_temperature = (int)val;
		battery_log(BAT_LOG_CRTI, "Get err_charge_temperature: %d\n",
			    batt_cust_data.err_charge_temperature);
	} else {
		battery_log(BAT_LOG_CRTI, "Get err_charge_temperature failed\n");
	}

	if (of_property_read_u32(np, "v_pre2cc_thres", &val) == 0) {
		batt_cust_data.v_pre2cc_thres = (int)val;
		battery_log(BAT_LOG_CRTI, "Get v_pre2cc_thres: %d\n",
			    batt_cust_data.v_pre2cc_thres);
	} else {
		battery_log(BAT_LOG_CRTI, "Get v_pre2cc_thres failed\n");
	}

	if (of_property_read_u32(np, "v_cc2topoff_thres", &val) == 0) {
		batt_cust_data.v_cc2topoff_thres = (int)val;
		battery_log(BAT_LOG_CRTI, "Get v_cc2topoff_thres: %d\n",
			    batt_cust_data.v_cc2topoff_thres);
	} else {
		battery_log(BAT_LOG_CRTI, "Get v_cc2topoff_thres failed\n");
	}

	if (of_property_read_u32(np, "recharging_voltage", &val) == 0) {
		batt_cust_data.recharging_voltage = (int)val;
		battery_log(BAT_LOG_CRTI, "Get recharging_voltage: %d\n",
			    batt_cust_data.recharging_voltage);
	} else {
		battery_log(BAT_LOG_CRTI, "Get recharging_voltage failed\n");
	}

	if (of_property_read_u32(np, "charging_full_current", &val) == 0) {
		batt_cust_data.charging_full_current = (int)val;
		battery_log(BAT_LOG_CRTI, "Get charging_full_current: %d\n",
			    batt_cust_data.charging_full_current);
	} else {
		battery_log(BAT_LOG_CRTI, "Get charging_full_current failed\n");
	}

	if (of_property_read_u32(np, "config_usb_if", &val) == 0) {
		batt_cust_data.config_usb_if = (int)val;
		battery_log(BAT_LOG_CRTI, "Get config_usb_if: %d\n", batt_cust_data.config_usb_if);
	} else {
		battery_log(BAT_LOG_CRTI, "Get config_usb_if failed\n");
	}

	if (of_property_read_u32(np, "usb_charger_current_suspend", &val) == 0) {
		batt_cust_data.usb_charger_current_suspend = (int)val;
		battery_log(BAT_LOG_CRTI, "Get usb_charger_current_suspend: %d\n",
			    batt_cust_data.usb_charger_current_suspend);
	} else {
		battery_log(BAT_LOG_CRTI, "Get usb_charger_current_suspend failed\n");
	}

	if (of_property_read_u32(np, "usb_charger_current_unconfigured", &val) == 0) {
		batt_cust_data.usb_charger_current_unconfigured = (int)val;
		battery_log(BAT_LOG_CRTI, "Get usb_charger_current_unconfigured: %d\n",
			    batt_cust_data.usb_charger_current_unconfigured);
	} else {
		battery_log(BAT_LOG_CRTI, "Get usb_charger_current_unconfigured failed\n");
	}

	if (of_property_read_u32(np, "usb_charger_current_configured", &val) == 0) {
		batt_cust_data.usb_charger_current_configured = (int)val;
		battery_log(BAT_LOG_CRTI, "Get usb_charger_current_configured: %d\n",
			    batt_cust_data.usb_charger_current_configured);
	} else {
		battery_log(BAT_LOG_CRTI, "Get usb_charger_current_configured failed\n");
	}

	if (of_property_read_u32(np, "usb_charger_current", &val) == 0) {
		batt_cust_data.usb_charger_current = (int)val;
		battery_log(BAT_LOG_CRTI, "Get usb_charger_current: %d\n",
			    batt_cust_data.usb_charger_current);
	} else {
		battery_log(BAT_LOG_CRTI, "Get usb_charger_current failed\n");
	}

	if (of_property_read_u32(np, "ac_charger_current", &val) == 0) {
		batt_cust_data.ac_charger_current = (int)val;
		battery_log(BAT_LOG_CRTI, "Get ac_charger_current: %d\n",
			    batt_cust_data.ac_charger_current);
	} else {
		battery_log(BAT_LOG_CRTI, "Get ac_charger_current failed\n");
	}

	if (of_property_read_u32(np, "non_std_ac_charger_current", &val) == 0) {
		batt_cust_data.non_std_ac_charger_current = (int)val;
		battery_log(BAT_LOG_CRTI, "Get non_std_ac_charger_current: %d\n",
			    batt_cust_data.non_std_ac_charger_current);
	} else {
		battery_log(BAT_LOG_CRTI, "Get non_std_ac_charger_current failed\n");
	}

	if (of_property_read_u32(np, "charging_host_charger_current", &val) == 0) {
		batt_cust_data.charging_host_charger_current = (int)val;
		battery_log(BAT_LOG_CRTI, "Get charging_host_charger_current: %d\n",
			    batt_cust_data.charging_host_charger_current);
	} else {
		battery_log(BAT_LOG_CRTI, "Get charging_host_charger_current failed\n");
	}

	if (of_property_read_u32(np, "apple_0_5a_charger_current", &val) == 0) {
		batt_cust_data.apple_0_5a_charger_current = (int)val;
		battery_log(BAT_LOG_CRTI, "Get apple_0_5a_charger_current: %d\n",
			    batt_cust_data.apple_0_5a_charger_current);
	} else {
		battery_log(BAT_LOG_CRTI, "Get apple_0_5a_charger_current failed\n");
	}

	if (of_property_read_u32(np, "apple_1_0a_charger_current", &val) == 0) {
		batt_cust_data.apple_1_0a_charger_current = (int)val;
		battery_log(BAT_LOG_CRTI, "Get apple_1_0a_charger_current: %d\n",
			    batt_cust_data.apple_1_0a_charger_current);
	} else {
		battery_log(BAT_LOG_CRTI, "Get apple_1_0a_charger_current failed\n");
	}

	if (of_property_read_u32(np, "apple_2_1a_charger_current", &val) == 0) {
		batt_cust_data.apple_2_1a_charger_current = (int)val;
		battery_log(BAT_LOG_CRTI, "Get apple_2_1a_charger_current: %d\n",
			    batt_cust_data.apple_2_1a_charger_current);
	} else {
		battery_log(BAT_LOG_CRTI, "Get apple_2_1a_charger_current failed\n");
	}


	if (of_property_read_u32(np, "bat_low_temp_protect_enable", &val) == 0) {
		batt_cust_data.bat_low_temp_protect_enable = (int)val;
		battery_log(BAT_LOG_CRTI, "Get bat_low_temp_protect_enable: %d\n",
			    batt_cust_data.bat_low_temp_protect_enable);
	} else {
		battery_log(BAT_LOG_CRTI, "Get bat_low_temp_protect_enable failed\n");
	}

	if (of_property_read_u32(np, "v_charger_enable", &val) == 0) {
		batt_cust_data.v_charger_enable = (int)val;
		battery_log(BAT_LOG_CRTI, "Get v_charger_enable: %d\n",
			    batt_cust_data.v_charger_enable);
	} else {
		battery_log(BAT_LOG_CRTI, "Get v_charger_enable failed\n");
	}

	if (of_property_read_u32(np, "v_charger_max", &val) == 0) {
		batt_cust_data.v_charger_max = (int)val;
		battery_log(BAT_LOG_CRTI, "Get v_charger_max: %d\n", batt_cust_data.v_charger_max);
	} else {
		battery_log(BAT_LOG_CRTI, "Get v_charger_max failed\n");
	}

	if (of_property_read_u32(np, "v_charger_min", &val) == 0) {
		batt_cust_data.v_charger_min = (int)val;
		battery_log(BAT_LOG_CRTI, "Get v_charger_min: %d\n", batt_cust_data.v_charger_min);
	} else {
		battery_log(BAT_LOG_CRTI, "Get v_charger_min failed\n");
	}

	if (of_property_read_u32(np, "onehundred_percent_tracking_time", &val) == 0) {
		batt_cust_data.onehundred_percent_tracking_time = (int)val;
		battery_log(BAT_LOG_CRTI, "Get onehundred_percent_tracking_time: %d\n",
			    batt_cust_data.onehundred_percent_tracking_time);
	} else {
		battery_log(BAT_LOG_CRTI, "Get onehundred_percent_tracking_time failed\n");
	}

	if (of_property_read_u32(np, "npercent_tracking_time", &val) == 0) {
		batt_cust_data.npercent_tracking_time = (int)val;
		battery_log(BAT_LOG_CRTI, "Get npercent_tracking_time: %d\n",
			    batt_cust_data.npercent_tracking_time);
	} else {
		battery_log(BAT_LOG_CRTI, "Get npercent_tracking_time failed\n");
	}

	if (of_property_read_u32(np, "sync_to_real_tracking_time", &val) == 0) {
		batt_cust_data.sync_to_real_tracking_time = (int)val;
		battery_log(BAT_LOG_CRTI, "Get sync_to_real_tracking_time: %d\n",
			    batt_cust_data.sync_to_real_tracking_time);
	} else {
		battery_log(BAT_LOG_CRTI, "Get sync_to_real_tracking_time failed\n");
	}

	if (of_property_read_u32(np, "v_0percent_tracking", &val) == 0) {
		batt_cust_data.v_0percent_tracking = (int)val;
		battery_log(BAT_LOG_CRTI, "Get v_0percent_tracking: %d\n",
			    batt_cust_data.v_0percent_tracking);
	} else {
		battery_log(BAT_LOG_CRTI, "Get v_0percent_tracking failed\n");
	}


	if (of_property_read_u32(np, "high_battery_voltage_support", &val) == 0) {
		batt_cust_data.high_battery_voltage_support = (int)val;
		battery_log(BAT_LOG_CRTI, "Get high_battery_voltage_support: %d\n",
			    batt_cust_data.high_battery_voltage_support);
	} else {
		battery_log(BAT_LOG_CRTI, "Get high_battery_voltage_support failed\n");
	}

	of_node_put(np);
	return 0;
}
#endif
int __batt_meter_init_cust_data_from_cust_header(void)
{
	battery_log(BAT_LOG_CRTI, "__batt_meter_init_cust_data_from_cust_header\n");

	/* cust_battery_meter_table.h */
#if (BAT_NTC_10 == 1)
	batt_meter_cust_data.bat_ntc = 10;
#elif (BAT_NTC_47 == 1)
	batt_meter_cust_data.bat_ntc = 47;
#endif

#if defined(RBAT_PULL_UP_R)
	batt_meter_cust_data.rbat_pull_up_r = RBAT_PULL_UP_R;
#endif
#if defined(RBAT_PULL_UP_VOLT)
	batt_meter_cust_data.rbat_pull_up_volt = RBAT_PULL_UP_VOLT;
#endif


	/* cust_battery_meter.h */

	/* ADC resister */
#if defined(R_BAT_SENSE)
	batt_meter_cust_data.r_bat_sense = R_BAT_SENSE;
	g_R_BAT_SENSE = R_BAT_SENSE;
#endif
#if defined(R_I_SENSE)
	batt_meter_cust_data.r_i_sense = R_I_SENSE;
	g_R_I_SENSE = R_I_SENSE;
#endif
#if defined(R_CHARGER_1)
	batt_meter_cust_data.r_charger_1 = R_CHARGER_1;
	g_R_CHARGER_1 = R_CHARGER_1;
#endif
#if defined(R_CHARGER_2)
	batt_meter_cust_data.r_charger_2 = R_CHARGER_2;
	g_R_CHARGER_2 = R_CHARGER_2;
#endif

#if defined(TEMPERATURE_T0)
	batt_meter_cust_data.temperature_t0 = TEMPERATURE_T0;
#endif
#if defined(TEMPERATURE_T1)
	batt_meter_cust_data.temperature_t1 = TEMPERATURE_T1;
#endif
#if defined(TEMPERATURE_T2)
	batt_meter_cust_data.temperature_t2 = TEMPERATURE_T2;
#endif
#if defined(TEMPERATURE_T3)
	batt_meter_cust_data.temperature_t3 = TEMPERATURE_T3;
#endif
#if defined(TEMPERATURE_T)
	batt_meter_cust_data.temperature_t = TEMPERATURE_T;
#endif
#if defined(FG_METER_RESISTANCE)
	batt_meter_cust_data.fg_meter_resistance = FG_METER_RESISTANCE;
#endif


	/* Qmax for battery  */
#if defined(Q_MAX_POS_50)
	batt_meter_cust_data.q_max_pos_50 = Q_MAX_POS_50;
#endif
#if defined(Q_MAX_POS_25)
	batt_meter_cust_data.q_max_pos_25 = Q_MAX_POS_25;
#endif
#if defined(Q_MAX_POS_0)
	batt_meter_cust_data.q_max_pos_0 = Q_MAX_POS_0;
#endif
#if defined(Q_MAX_NEG_10)
	batt_meter_cust_data.q_max_neg_10 = Q_MAX_NEG_10;
#endif
#if defined(Q_MAX_POS_50_H_CURRENT)
	batt_meter_cust_data.q_max_pos_50_h_current = Q_MAX_POS_50_H_CURRENT;
#endif
#if defined(Q_MAX_POS_25_H_CURRENT)
	batt_meter_cust_data.q_max_pos_25_h_current = Q_MAX_POS_25_H_CURRENT;
#endif
#if defined(Q_MAX_POS_0_H_CURRENT)
	batt_meter_cust_data.q_max_pos_0_h_current = Q_MAX_POS_0_H_CURRENT;
#endif
#if defined(Q_MAX_NEG_10_H_CURRENT)
	batt_meter_cust_data.q_max_neg_10_h_current = Q_MAX_NEG_10_H_CURRENT;
#endif
#if defined(OAM_D5)
	batt_meter_cust_data.oam_d5 = OAM_D5;	/* 1 : D5,   0: D2 */
#endif


#if defined(CHANGE_TRACKING_POINT)
	batt_meter_cust_data.change_tracking_point = 1;
#else				/* #if defined(CHANGE_TRACKING_POINT) */
	batt_meter_cust_data.change_tracking_point = 0;
#endif				/* #if defined(CHANGE_TRACKING_POINT) */


#if defined(CUST_TRACKING_POINT)
	batt_meter_cust_data.cust_tracking_point = CUST_TRACKING_POINT;
	g_tracking_point = CUST_TRACKING_POINT;
#endif
#if defined(CUST_R_SENSE)
	batt_meter_cust_data.cust_r_sense = CUST_R_SENSE;
#endif
#if defined(CUST_HW_CC)
	batt_meter_cust_data.cust_hw_cc = CUST_HW_CC;
#endif
#if defined(AGING_TUNING_VALUE)
	batt_meter_cust_data.aging_tuning_value = AGING_TUNING_VALUE;
#endif
#if defined(CUST_R_FG_OFFSET)
	batt_meter_cust_data.cust_r_fg_offset = CUST_R_FG_OFFSET;
#endif
#if defined(OCV_BOARD_COMPESATE)
	batt_meter_cust_data.ocv_board_compesate = OCV_BOARD_COMPESATE;
#endif
#if defined(R_FG_BOARD_BASE)
	batt_meter_cust_data.r_fg_board_base = R_FG_BOARD_BASE;
#endif
#if defined(R_FG_BOARD_SLOPE)
	batt_meter_cust_data.r_fg_board_slope = R_FG_BOARD_SLOPE;
#endif
#if defined(CAR_TUNE_VALUE)
	batt_meter_cust_data.car_tune_value = CAR_TUNE_VALUE;
#endif

	/* HW Fuel gague  */
#if defined(CURRENT_DETECT_R_FG)
	batt_meter_cust_data.current_detect_r_fg = CURRENT_DETECT_R_FG;
#endif
#if defined(MinErrorOffset)
	batt_meter_cust_data.minerroroffset = MinErrorOffset;
#endif
#if defined(FG_VBAT_AVERAGE_SIZE)
	batt_meter_cust_data.fg_vbat_average_size = FG_VBAT_AVERAGE_SIZE;
#endif
#if defined(R_FG_VALUE)
	batt_meter_cust_data.r_fg_value = R_FG_VALUE;
#endif
#if defined(CUST_POWERON_DELTA_CAPACITY_TOLRANCE)
	batt_meter_cust_data.cust_poweron_delta_capacity_tolrance =
	    CUST_POWERON_DELTA_CAPACITY_TOLRANCE;
#endif
#if defined(CUST_POWERON_LOW_CAPACITY_TOLRANCE)
	batt_meter_cust_data.cust_poweron_low_capacity_tolrance =
	    CUST_POWERON_LOW_CAPACITY_TOLRANCE;
#endif
#if defined(CUST_POWERON_MAX_VBAT_TOLRANCE)
	batt_meter_cust_data.cust_poweron_max_vbat_tolrance = CUST_POWERON_MAX_VBAT_TOLRANCE;
#endif
#if defined(CUST_POWERON_DELTA_VBAT_TOLRANCE)
	batt_meter_cust_data.cust_poweron_delta_vbat_tolrance = CUST_POWERON_DELTA_VBAT_TOLRANCE;
#endif
#if defined(CUST_POWERON_DELTA_HW_SW_OCV_CAPACITY_TOLRANCE)
	batt_meter_cust_data.cust_poweron_delta_hw_sw_ocv_capacity_tolrance =
	    CUST_POWERON_DELTA_HW_SW_OCV_CAPACITY_TOLRANCE;
#endif

#if defined(FIXED_TBAT_25)
	batt_meter_cust_data.fixed_tbat_25 = 1;
#else				/* #if defined(FIXED_TBAT_25) */
	batt_meter_cust_data.fixed_tbat_25 = 0;
#endif				/* #if defined(FIXED_TBAT_25) */

	/* Dynamic change wake up period of battery thread when suspend */
#if defined(VBAT_NORMAL_WAKEUP)
	batt_meter_cust_data.vbat_normal_wakeup = VBAT_NORMAL_WAKEUP;
#endif
#if defined(VBAT_LOW_POWER_WAKEUP)
	batt_meter_cust_data.vbat_low_power_wakeup = VBAT_LOW_POWER_WAKEUP;
#endif
#if defined(NORMAL_WAKEUP_PERIOD)
	batt_meter_cust_data.normal_wakeup_period = NORMAL_WAKEUP_PERIOD;
	_g_bat_sleep_total_time = NORMAL_WAKEUP_PERIOD;
#endif
#if defined(LOW_POWER_WAKEUP_PERIOD)
	batt_meter_cust_data.low_power_wakeup_period = LOW_POWER_WAKEUP_PERIOD;
#endif
#if defined(CLOSE_POWEROFF_WAKEUP_PERIOD)
	batt_meter_cust_data.close_poweroff_wakeup_period = CLOSE_POWEROFF_WAKEUP_PERIOD;
#endif

	return 0;
}

#ifdef CONFIG_OF
int __batt_meter_init_cust_data_from_dt(void)
{
	struct device_node *np;
	u32 val;
	/* char *path = "/BAT_METTER"; */
	char *path = "/bus/BAT_METTER";
	/*struct device_node *dt_node;
	   const u32 *property;
	   int len; */
	int num, idx, saddles;
	unsigned int addr;



	np = of_find_node_by_path(path);
	if (!np) {
		/* printk(KERN_ERR "(E) Failed to find device-tree node: %s\n", path); */
		battery_log(BAT_LOG_CRTI, "Failed to find device-tree node: %s\n", path);
		return -ENODEV;
	}


	if (of_property_read_u32(np, "rbat_pull_up_r", &val) == 0) {
		batt_meter_cust_data.rbat_pull_up_r = (int)val;
		battery_log(BAT_LOG_CRTI, "Get rbat_pull_up_r: %d\n",
			    batt_meter_cust_data.rbat_pull_up_r);
	} else {
		battery_log(BAT_LOG_CRTI, "Get rbat_pull_up_r failed\n");
	}

	if (of_property_read_u32(np, "rbat_pull_up_volt", &val) == 0) {
		batt_meter_cust_data.rbat_pull_up_volt = (int)val;
		battery_log(BAT_LOG_CRTI, "Get rbat_pull_up_volt: %d\n",
			    batt_meter_cust_data.rbat_pull_up_volt);
	} else {
		battery_log(BAT_LOG_CRTI, "Get rbat_pull_up_volt failed\n");
	}



	if (of_property_read_u32(np, "batt_temperature_table_num", &val) == 0) {
		num = (int)val;
		battery_log(BAT_LOG_CRTI, "Get batt_temperature_table_num: %d\n", num);
	} else {
		battery_log(BAT_LOG_CRTI, "Get batt_temperature_table_num failed\n");
	}

	saddles = fgauge_get_saddles();
	idx = 0;
	while (!of_property_read_u32_index(np, "batt_temperature_table", idx, &addr)) {
		idx++;
		if (!of_property_read_u32_index(np, "batt_temperature_table", idx, &val)) {
			battery_log(BAT_LOG_CRTI, "batt_temperature_table: addr: %d, val: %d\n",
				    addr, val);
		}
		idx++;
		Batt_Temperature_Table[idx / 2].BatteryTemp = addr;
		Batt_Temperature_Table[idx / 2].TemperatureR = val;

		if (idx >= saddles * 2)
			break;
	}




	if (of_property_read_u32(np, "battery_profile_t0_num", &val) == 0) {
		num = (int)val;
		battery_log(BAT_LOG_CRTI, "Get battery_profile_t0_num: %d\n", num);
	} else {
		battery_log(BAT_LOG_CRTI, "Get battery_profile_t0_num failed\n");
	}

	saddles = fgauge_get_saddles();
	idx = 0;
	while (!of_property_read_u32_index(np, "battery_profile_t0", idx, &addr)) {
		idx++;
		if (!of_property_read_u32_index(np, "battery_profile_t0", idx, &val)) {
			battery_log(BAT_LOG_CRTI, "battery_profile_t0: addr: %d, val: %d\n", addr,
				    val);
		}
		idx++;
		battery_profile_t0[idx / 2].percentage = addr;
		battery_profile_t0[idx / 2].voltage = val;

		if (idx >= saddles * 2)
			break;
	}


	if (of_property_read_u32(np, "battery_profile_t1_num", &val) == 0) {
		num = (int)val;
		battery_log(BAT_LOG_CRTI, "Get battery_profile_t1_num: %d\n", num);
	} else {
		battery_log(BAT_LOG_CRTI, "Get battery_profile_t1_num failed\n");
	}

	saddles = fgauge_get_saddles();
	idx = 0;
	while (!of_property_read_u32_index(np, "battery_profile_t1", idx, &addr)) {
		idx++;
		if (!of_property_read_u32_index(np, "battery_profile_t1", idx, &val)) {
			battery_log(BAT_LOG_CRTI, "battery_profile_t1: addr: %d, val: %d\n", addr,
				    val);
		}
		idx++;
		battery_profile_t1[idx / 2].percentage = addr;
		battery_profile_t1[idx / 2].voltage = val;

		if (idx >= saddles * 2)
			break;
	}

	if (of_property_read_u32(np, "battery_profile_t2_num", &val) == 0) {
		num = (int)val;
		battery_log(BAT_LOG_CRTI, "Get battery_profile_t2_num: %d\n", num);
	} else {
		battery_log(BAT_LOG_CRTI, "Get battery_profile_t2_num failed\n");
	}

	saddles = fgauge_get_saddles();
	idx = 0;
	while (!of_property_read_u32_index(np, "battery_profile_t2", idx, &addr)) {
		idx++;
		if (!of_property_read_u32_index(np, "battery_profile_t2", idx, &val)) {
			battery_log(BAT_LOG_CRTI, "battery_profile_t0: addr: %d, val: %d\n", addr,
				    val);
		}
		idx++;
		battery_profile_t2[idx / 2].percentage = addr;
		battery_profile_t2[idx / 2].voltage = val;

		if (idx >= saddles * 2)
			break;
	}

	if (of_property_read_u32(np, "battery_profile_t3_num", &val) == 0) {
		num = (int)val;
		battery_log(BAT_LOG_CRTI, "Get battery_profile_t3_num: %d\n", num);
	} else {
		battery_log(BAT_LOG_CRTI, "Get battery_profile_t3_num failed\n");
	}

	saddles = fgauge_get_saddles();
	idx = 0;
	while (!of_property_read_u32_index(np, "battery_profile_t3", idx, &addr)) {
		idx++;
		if (!of_property_read_u32_index(np, "battery_profile_t3", idx, &val)) {
			battery_log(BAT_LOG_CRTI, "battery_profile_t3: addr: %d, val: %d\n", addr,
				    val);
		}
		idx++;
		battery_profile_t0[idx / 2].percentage = addr;
		battery_profile_t0[idx / 2].voltage = val;

		if (idx >= saddles * 2)
			break;
	}

	if (of_property_read_u32(np, "r_profile_t0_num", &val) == 0) {
		num = (int)val;
		battery_log(BAT_LOG_CRTI, "Get r_profile_t0_num: %d\n", num);
	} else {
		battery_log(BAT_LOG_CRTI, "Get r_profile_t0_num failed\n");
	}

	saddles = fgauge_get_saddles();
	idx = 0;
	while (!of_property_read_u32_index(np, "r_profile_t0", idx, &addr)) {
		idx++;
		if (!of_property_read_u32_index(np, "r_profile_t0", idx, &val))
			battery_log(BAT_LOG_CRTI, "r_profile_t0: addr: %d, val: %d\n", addr, val);

		idx++;
		r_profile_t0[idx / 2].resistance = addr;
		r_profile_t0[idx / 2].voltage = val;

		if (idx >= saddles * 2)
			break;
	}

	if (of_property_read_u32(np, "r_profile_t1_num", &val) == 0) {
		num = (int)val;
		battery_log(BAT_LOG_CRTI, "Get r_profile_t1_num: %d\n", num);
	} else {
		battery_log(BAT_LOG_CRTI, "Get r_profile_t1_num failed\n");
	}

	saddles = fgauge_get_saddles();
	idx = 0;
	while (!of_property_read_u32_index(np, "r_profile_t1", idx, &addr)) {
		idx++;
		if (!of_property_read_u32_index(np, "r_profile_t1", idx, &val))
			battery_log(BAT_LOG_CRTI, "r_profile_t1: addr: %d, val: %d\n", addr, val);

		idx++;
		r_profile_t1[idx / 2].resistance = addr;
		r_profile_t1[idx / 2].voltage = val;

		if (idx >= saddles * 2)
			break;
	}

	if (of_property_read_u32(np, "r_profile_t2_num", &val) == 0) {
		num = (int)val;
		battery_log(BAT_LOG_CRTI, "Get r_profile_t2_num: %d\n", num);
	} else {
		battery_log(BAT_LOG_CRTI, "Get r_profile_t2_num failed\n");
	}

	saddles = fgauge_get_saddles();
	idx = 0;
	while (!of_property_read_u32_index(np, "r_profile_t2", idx, &addr)) {
		idx++;
		if (!of_property_read_u32_index(np, "r_profile_t2", idx, &val))
			battery_log(BAT_LOG_CRTI, "r_profile_t2: addr: %d, val: %d\n", addr, val);

		idx++;
		r_profile_t2[idx / 2].resistance = addr;
		r_profile_t2[idx / 2].voltage = val;

		if (idx >= saddles * 2)
			break;
	}

	if (of_property_read_u32(np, "r_profile_t3_num", &val) == 0) {
		num = (int)val;
		battery_log(BAT_LOG_CRTI, "Get r_profile_t3_num: %d\n", num);
	} else {
		battery_log(BAT_LOG_CRTI, "Get r_profile_t3_num failed\n");
	}

	saddles = fgauge_get_saddles();
	idx = 0;
	while (!of_property_read_u32_index(np, "r_profile_t3", idx, &addr)) {
		idx++;
		if (!of_property_read_u32_index(np, "r_profile_t3", idx, &val))
			battery_log(BAT_LOG_CRTI, "r_profile_t3: addr: %d, val: %d\n", addr, val);

		idx++;
		r_profile_t3[idx / 2].resistance = addr;
		r_profile_t3[idx / 2].voltage = val;

		if (idx >= saddles * 2)
			break;
	}



	if (of_property_read_u32(np, "r_bat_sense", &val) == 0) {
		batt_meter_cust_data.r_bat_sense = (int)val;
		bm_print(BM_LOG_CRTI, "Get r_bat_sense: %d\n", batt_meter_cust_data.r_bat_sense);
	} else {
		bm_print(BM_LOG_CRTI, "Get r_bat_sense failed\n");
	}

	if (of_property_read_u32(np, "r_i_sense", &val) == 0) {
		batt_meter_cust_data.r_i_sense = (int)val;
		bm_print(BM_LOG_CRTI, "Get r_i_sense: %d\n", batt_meter_cust_data.r_i_sense);
	} else {
		bm_print(BM_LOG_CRTI, "Get r_i_sense failed\n");
	}

	if (of_property_read_u32(np, "r_charger_1", &val) == 0) {
		batt_meter_cust_data.r_charger_1 = (int)val;
		bm_print(BM_LOG_CRTI, "Get r_charger_1: %d\n", batt_meter_cust_data.r_charger_1);
	} else {
		bm_print(BM_LOG_CRTI, "Get r_charger_1 failed\n");
	}

	if (of_property_read_u32(np, "r_charger_2", &val) == 0) {
		batt_meter_cust_data.r_charger_2 = (int)val;
		bm_print(BM_LOG_CRTI, "Get r_charger_2: %d\n", batt_meter_cust_data.r_charger_2);
	} else {
		bm_print(BM_LOG_CRTI, "Get r_charger_2 failed\n");
	}

	if (of_property_read_u32(np, "temperature_t0", &val) == 0) {
		batt_meter_cust_data.temperature_t0 = (int)val;
		bm_print(BM_LOG_CRTI, "Get temperature_t0: %d\n",
			 batt_meter_cust_data.temperature_t0);
	} else {
		bm_print(BM_LOG_CRTI, "Get temperature_t0 failed\n");
	}

	if (of_property_read_u32(np, "temperature_t1", &val) == 0) {
		batt_meter_cust_data.temperature_t1 = (int)val;
		bm_print(BM_LOG_CRTI, "Get temperature_t1: %d\n",
			 batt_meter_cust_data.temperature_t1);
	} else {
		bm_print(BM_LOG_CRTI, "Get temperature_t1 failed\n");
	}

	if (of_property_read_u32(np, "temperature_t2", &val) == 0) {
		batt_meter_cust_data.temperature_t2 = (int)val;
		bm_print(BM_LOG_CRTI, "Get temperature_t2: %d\n",
			 batt_meter_cust_data.temperature_t2);
	} else {
		bm_print(BM_LOG_CRTI, "Get temperature_t2 failed\n");
	}

	if (of_property_read_u32(np, "temperature_t3", &val) == 0) {
		batt_meter_cust_data.temperature_t3 = (int)val;
		bm_print(BM_LOG_CRTI, "Get temperature_t3: %d\n",
			 batt_meter_cust_data.temperature_t3);
	} else {
		bm_print(BM_LOG_CRTI, "Get temperature_t3 failed\n");
	}

	if (of_property_read_u32(np, "temperature_t", &val) == 0) {
		batt_meter_cust_data.temperature_t = (int)val;
		bm_print(BM_LOG_CRTI, "Get temperature_t: %d\n",
			 batt_meter_cust_data.temperature_t);
	} else {
		bm_print(BM_LOG_CRTI, "Get temperature_t failed\n");
	}

	if (of_property_read_u32(np, "fg_meter_resistance", &val) == 0) {
		batt_meter_cust_data.fg_meter_resistance = (int)val;
		bm_print(BM_LOG_CRTI, "Get fg_meter_resistance: %d\n",
			 batt_meter_cust_data.fg_meter_resistance);
	} else {
		bm_print(BM_LOG_CRTI, "Get fg_meter_resistance failed\n");
	}

	if (of_property_read_u32(np, "q_max_pos_50", &val) == 0) {
		batt_meter_cust_data.q_max_pos_50 = (int)val;
		bm_print(BM_LOG_CRTI, "Get q_max_pos_50: %d\n", batt_meter_cust_data.q_max_pos_50);
	} else {
		bm_print(BM_LOG_CRTI, "Get q_max_pos_50 failed\n");
	}

	if (of_property_read_u32(np, "q_max_pos_25", &val) == 0) {
		batt_meter_cust_data.q_max_pos_25 = (int)val;
		bm_print(BM_LOG_CRTI, "Get q_max_pos_25: %d\n", batt_meter_cust_data.q_max_pos_25);
	} else {
		bm_print(BM_LOG_CRTI, "Get q_max_pos_25 failed\n");
	}

	if (of_property_read_u32(np, "q_max_pos_0", &val) == 0) {
		batt_meter_cust_data.q_max_pos_0 = (int)val;
		bm_print(BM_LOG_CRTI, "Get q_max_pos_0: %d\n", batt_meter_cust_data.q_max_pos_0);
	} else {
		bm_print(BM_LOG_CRTI, "Get q_max_pos_0 failed\n");
	}

	if (of_property_read_u32(np, "q_max_neg_10", &val) == 0) {
		batt_meter_cust_data.q_max_neg_10 = (int)val;
		bm_print(BM_LOG_CRTI, "Get q_max_neg_10: %d\n", batt_meter_cust_data.q_max_neg_10);
	} else {
		bm_print(BM_LOG_CRTI, "Get q_max_neg_10 failed\n");
	}

	if (of_property_read_u32(np, "q_max_pos_50_h_current", &val) == 0) {
		batt_meter_cust_data.q_max_pos_50_h_current = (int)val;
		bm_print(BM_LOG_CRTI, "Get q_max_pos_50_h_current: %d\n",
			 batt_meter_cust_data.q_max_pos_50_h_current);
	} else {
		bm_print(BM_LOG_CRTI, "Get q_max_pos_50_h_current failed\n");
	}

	if (of_property_read_u32(np, "q_max_pos_25_h_current", &val) == 0) {
		batt_meter_cust_data.q_max_pos_25_h_current = (int)val;
		bm_print(BM_LOG_CRTI, "Get q_max_pos_25_h_current: %d\n",
			 batt_meter_cust_data.q_max_pos_25_h_current);
	} else {
		bm_print(BM_LOG_CRTI, "Get q_max_pos_25_h_current failed\n");
	}

	if (of_property_read_u32(np, "q_max_pos_0_h_current", &val) == 0) {
		batt_meter_cust_data.q_max_pos_0_h_current = (int)val;
		bm_print(BM_LOG_CRTI, "Get q_max_pos_0_h_current: %d\n",
			 batt_meter_cust_data.q_max_pos_0_h_current);
	} else {
		bm_print(BM_LOG_CRTI, "Get q_max_pos_0_h_current failed\n");
	}

	if (of_property_read_u32(np, "q_max_neg_10_h_current", &val) == 0) {
		batt_meter_cust_data.q_max_neg_10_h_current = (int)val;
		bm_print(BM_LOG_CRTI, "Get q_max_neg_10_h_current: %d\n",
			 batt_meter_cust_data.q_max_neg_10_h_current);
	} else {
		bm_print(BM_LOG_CRTI, "Get q_max_neg_10_h_current failed\n");
	}

	if (of_property_read_u32(np, "oam_d5", &val) == 0) {
		batt_meter_cust_data.oam_d5 = (int)val;
		bm_print(BM_LOG_CRTI, "Get oam_d5: %d\n", batt_meter_cust_data.oam_d5);
	} else {
		bm_print(BM_LOG_CRTI, "Get oam_d5 failed\n");
	}

	if (of_property_read_u32(np, "change_tracking_point", &val) == 0) {
		batt_meter_cust_data.change_tracking_point = (int)val;
		bm_print(BM_LOG_CRTI, "Get change_tracking_point: %d\n",
			 batt_meter_cust_data.change_tracking_point);
	} else {
		bm_print(BM_LOG_CRTI, "Get change_tracking_point failed\n");
	}

	if (of_property_read_u32(np, "cust_tracking_point", &val) == 0) {
		batt_meter_cust_data.cust_tracking_point = (int)val;
		bm_print(BM_LOG_CRTI, "Get cust_tracking_point: %d\n",
			 batt_meter_cust_data.cust_tracking_point);
	} else {
		bm_print(BM_LOG_CRTI, "Get cust_tracking_point failed\n");
	}

	if (of_property_read_u32(np, "cust_r_sense", &val) == 0) {
		batt_meter_cust_data.cust_r_sense = (int)val;
		bm_print(BM_LOG_CRTI, "Get cust_r_sense: %d\n", batt_meter_cust_data.cust_r_sense);
	} else {
		bm_print(BM_LOG_CRTI, "Get cust_r_sense failed\n");
	}

	if (of_property_read_u32(np, "cust_hw_cc", &val) == 0) {
		batt_meter_cust_data.cust_hw_cc = (int)val;
		bm_print(BM_LOG_CRTI, "Get cust_hw_cc: %d\n", batt_meter_cust_data.cust_hw_cc);
	} else {
		bm_print(BM_LOG_CRTI, "Get cust_hw_cc failed\n");
	}

	if (of_property_read_u32(np, "aging_tuning_value", &val) == 0) {
		batt_meter_cust_data.aging_tuning_value = (int)val;
		bm_print(BM_LOG_CRTI, "Get aging_tuning_value: %d\n",
			 batt_meter_cust_data.aging_tuning_value);
	} else {
		bm_print(BM_LOG_CRTI, "Get aging_tuning_value failed\n");
	}

	if (of_property_read_u32(np, "cust_r_fg_offset", &val) == 0) {
		batt_meter_cust_data.cust_r_fg_offset = (int)val;
		bm_print(BM_LOG_CRTI, "Get cust_r_fg_offset: %d\n",
			 batt_meter_cust_data.cust_r_fg_offset);
	} else {
		bm_print(BM_LOG_CRTI, "Get cust_r_fg_offset failed\n");
	}

	if (of_property_read_u32(np, "ocv_board_compesate", &val) == 0) {
		batt_meter_cust_data.ocv_board_compesate = (int)val;
		bm_print(BM_LOG_CRTI, "Get ocv_board_compesate: %d\n",
			 batt_meter_cust_data.ocv_board_compesate);
	} else {
		bm_print(BM_LOG_CRTI, "Get ocv_board_compesate failed\n");
	}

	if (of_property_read_u32(np, "r_fg_board_base", &val) == 0) {
		batt_meter_cust_data.r_fg_board_base = (int)val;
		bm_print(BM_LOG_CRTI, "Get r_fg_board_base: %d\n",
			 batt_meter_cust_data.r_fg_board_base);
	} else {
		bm_print(BM_LOG_CRTI, "Get r_fg_board_base failed\n");
	}

	if (of_property_read_u32(np, "r_fg_board_slope", &val) == 0) {
		batt_meter_cust_data.r_fg_board_slope = (int)val;
		bm_print(BM_LOG_CRTI, "Get r_fg_board_slope: %d\n",
			 batt_meter_cust_data.r_fg_board_slope);
	} else {
		bm_print(BM_LOG_CRTI, "Get r_fg_board_slope failed\n");
	}

	if (of_property_read_u32(np, "car_tune_value", &val) == 0) {
		batt_meter_cust_data.car_tune_value = (int)val;
		bm_print(BM_LOG_CRTI, "Get car_tune_value: %d\n",
			 batt_meter_cust_data.car_tune_value);
	} else {
		bm_print(BM_LOG_CRTI, "Get car_tune_value failed\n");
	}

	if (of_property_read_u32(np, "current_detect_r_fg", &val) == 0) {
		batt_meter_cust_data.current_detect_r_fg = (int)val;
		bm_print(BM_LOG_CRTI, "Get current_detect_r_fg: %d\n",
			 batt_meter_cust_data.current_detect_r_fg);
	} else {
		bm_print(BM_LOG_CRTI, "Get current_detect_r_fg failed\n");
	}

	if (of_property_read_u32(np, "minerroroffset", &val) == 0) {
		batt_meter_cust_data.minerroroffset = (int)val;
		bm_print(BM_LOG_CRTI, "Get minerroroffset: %d\n",
			 batt_meter_cust_data.minerroroffset);
	} else {
		bm_print(BM_LOG_CRTI, "Get minerroroffset failed\n");
	}

	if (of_property_read_u32(np, "fg_vbat_average_size", &val) == 0) {
		batt_meter_cust_data.fg_vbat_average_size = (int)val;
		bm_print(BM_LOG_CRTI, "Get fg_vbat_average_size: %d\n",
			 batt_meter_cust_data.fg_vbat_average_size);
	} else {
		bm_print(BM_LOG_CRTI, "Get fg_vbat_average_size failed\n");
	}

	if (of_property_read_u32(np, "r_fg_value", &val) == 0) {
		batt_meter_cust_data.r_fg_value = (int)val;
		bm_print(BM_LOG_CRTI, "Get r_fg_value: %d\n", batt_meter_cust_data.r_fg_value);
	} else {
		bm_print(BM_LOG_CRTI, "Get r_fg_value failed\n");
	}

	if (of_property_read_u32(np, "cust_poweron_delta_capacity_tolrance", &val) == 0) {
		batt_meter_cust_data.cust_poweron_delta_capacity_tolrance = (int)val;
		bm_print(BM_LOG_CRTI, "Get cust_poweron_delta_capacity_tolrance: %d\n",
			 batt_meter_cust_data.cust_poweron_delta_capacity_tolrance);
	} else {
		bm_print(BM_LOG_CRTI, "Get cust_poweron_delta_capacity_tolrance failed\n");
	}

	if (of_property_read_u32(np, "cust_poweron_low_capacity_tolrance", &val) == 0) {
		batt_meter_cust_data.cust_poweron_low_capacity_tolrance = (int)val;
		bm_print(BM_LOG_CRTI, "Get cust_poweron_low_capacity_tolrance: %d\n",
			 batt_meter_cust_data.cust_poweron_low_capacity_tolrance);
	} else {
		bm_print(BM_LOG_CRTI, "Get cust_poweron_low_capacity_tolrance failed\n");
	}

	if (of_property_read_u32(np, "cust_poweron_max_vbat_tolrance", &val) == 0) {
		batt_meter_cust_data.cust_poweron_max_vbat_tolrance = (int)val;
		bm_print(BM_LOG_CRTI, "Get cust_poweron_max_vbat_tolrance: %d\n",
			 batt_meter_cust_data.cust_poweron_max_vbat_tolrance);
	} else {
		bm_print(BM_LOG_CRTI, "Get cust_poweron_max_vbat_tolrance failed\n");
	}

	if (of_property_read_u32(np, "cust_poweron_delta_vbat_tolrance", &val) == 0) {
		batt_meter_cust_data.cust_poweron_delta_vbat_tolrance = (int)val;
		bm_print(BM_LOG_CRTI, "Get cust_poweron_delta_vbat_tolrance: %d\n",
			 batt_meter_cust_data.cust_poweron_delta_vbat_tolrance);
	} else {
		bm_print(BM_LOG_CRTI, "Get cust_poweron_delta_vbat_tolrance failed\n");
	}

	if (of_property_read_u32(np, "cust_poweron_delta_hw_sw_ocv_capacity_tolrance", &val) == 0) {
		batt_meter_cust_data.cust_poweron_delta_hw_sw_ocv_capacity_tolrance = (int)val;
		bm_print(BM_LOG_CRTI, "Get cust_poweron_delta_hw_sw_ocv_capacity_tolrance: %d\n",
			 batt_meter_cust_data.cust_poweron_delta_hw_sw_ocv_capacity_tolrance);
	} else {
		bm_print(BM_LOG_CRTI,
			 "Get cust_poweron_delta_hw_sw_ocv_capacity_tolrance failed\n");
	}

	if (of_property_read_u32(np, "fixed_tbat_25", &val) == 0) {
		batt_meter_cust_data.fixed_tbat_25 = (int)val;
		bm_print(BM_LOG_CRTI, "Get fixed_tbat_25: %d\n",
			 batt_meter_cust_data.fixed_tbat_25);
	} else {
		bm_print(BM_LOG_CRTI, "Get fixed_tbat_25 failed\n");
	}

	if (of_property_read_u32(np, "vbat_normal_wakeup", &val) == 0) {
		batt_meter_cust_data.vbat_normal_wakeup = (int)val;
		bm_print(BM_LOG_CRTI, "Get vbat_normal_wakeup: %d\n",
			 batt_meter_cust_data.vbat_normal_wakeup);
	} else {
		bm_print(BM_LOG_CRTI, "Get vbat_normal_wakeup failed\n");
	}

	if (of_property_read_u32(np, "vbat_low_power_wakeup", &val) == 0) {
		batt_meter_cust_data.vbat_low_power_wakeup = (int)val;
		bm_print(BM_LOG_CRTI, "Get vbat_low_power_wakeup: %d\n",
			 batt_meter_cust_data.vbat_low_power_wakeup);
	} else {
		bm_print(BM_LOG_CRTI, "Get vbat_low_power_wakeup failed\n");
	}

	if (of_property_read_u32(np, "normal_wakeup_period", &val) == 0) {
		batt_meter_cust_data.normal_wakeup_period = (int)val;
		bm_print(BM_LOG_CRTI, "Get normal_wakeup_period: %d\n",
			 batt_meter_cust_data.normal_wakeup_period);
	} else {
		bm_print(BM_LOG_CRTI, "Get normal_wakeup_period failed\n");
	}

	if (of_property_read_u32(np, "low_power_wakeup_period", &val) == 0) {
		batt_meter_cust_data.low_power_wakeup_period = (int)val;
		bm_print(BM_LOG_CRTI, "Get low_power_wakeup_period: %d\n",
			 batt_meter_cust_data.low_power_wakeup_period);
	} else {
		bm_print(BM_LOG_CRTI, "Get low_power_wakeup_period failed\n");
	}

	if (of_property_read_u32(np, "close_poweroff_wakeup_period", &val) == 0) {
		batt_meter_cust_data.close_poweroff_wakeup_period = (int)val;
		bm_print(BM_LOG_CRTI, "Get close_poweroff_wakeup_period: %d\n",
			 batt_meter_cust_data.close_poweroff_wakeup_period);
	} else {
		bm_print(BM_LOG_CRTI, "Get close_poweroff_wakeup_period failed\n");
	}
	of_node_put(np);

	return 0;
}
#endif

int batt_init_cust_data(void)
{
	static int init_done;

	if (init_done == 1)
		return;
	init_done = 1;

#if defined(BATTERY_DTS_SUPPORT)
#ifdef CONFIG_OF
	bm_print(BM_LOG_CRTI, "battery custom init by DTS\n");
	__batt_init_cust_data_from_cust_header();
	__batt_meter_init_cust_data_from_cust_header();
	__batt_init_cust_data_from_dt();
	__batt_meter_init_cust_data_from_dt();
#else				/* #ifdef CONFIG_OF */
	bm_print(BM_LOG_CRTI, "battery custom init\n");
	__batt_init_cust_data_from_cust_header();
	__batt_meter_init_cust_data_from_cust_header();
#endif				/* #ifdef CONFIG_OF */
#else
	bm_print(BM_LOG_CRTI, "battery custom init2\n");
	__batt_init_cust_data_from_cust_header();
	__batt_meter_init_cust_data_from_cust_header();
#endif

	return 0;
}





/* ============================================================ // */
int get_r_fg_value(void)
{
	return batt_meter_cust_data.r_fg_value + batt_meter_cust_data.cust_r_fg_offset;
}

#ifdef MTK_MULTI_BAT_PROFILE_SUPPORT
int BattThermistorConverTemp(int Res)
{
	int i = 0;
	int RES1 = 0, RES2 = 0;
	int TBatt_Value = -200, TMP1 = 0, TMP2 = 0;

	BATT_TEMPERATURE *batt_temperature_table = &Batt_Temperature_Table[g_fg_battery_id];
	if (Res >= batt_temperature_table[0].TemperatureR) {
		TBatt_Value = -20;
	} else if (Res <= batt_temperature_table[16].TemperatureR) {
		TBatt_Value = 60;
	} else {
		RES1 = batt_temperature_table[0].TemperatureR;
		TMP1 = batt_temperature_table[0].BatteryTemp;

		for (i = 0; i <= 16; i++) {
			if (Res >= batt_temperature_table[i].TemperatureR) {
				RES2 = batt_temperature_table[i].TemperatureR;
				TMP2 = batt_temperature_table[i].BatteryTemp;
				break;
			} else {
				RES1 = batt_temperature_table[i].TemperatureR;
				TMP1 = batt_temperature_table[i].BatteryTemp;
			}
		}

		TBatt_Value = (((Res - RES2) * TMP1) + ((RES1 - Res) * TMP2)) / (RES1 - RES2);
	}

	return TBatt_Value;
}

kal_int32 fgauge_get_Q_max(kal_int16 temperature)
{
	kal_int32 ret_Q_max = 0;
	kal_int32 low_temperature = 0, high_temperature = 0;
	kal_int32 low_Q_max = 0, high_Q_max = 0;

	if (temperature <= batt_meter_cust_data.temperature_t1) {
		low_temperature = (-10);
		low_Q_max = g_Q_MAX_NEG_10[g_fg_battery_id];
		high_temperature = batt_meter_cust_data.temperature_t1;
		high_Q_max = g_Q_MAX_POS_0[g_fg_battery_id];

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else if (temperature <= batt_meter_cust_data.temperature_t2) {
		low_temperature = batt_meter_cust_data.temperature_t1;
		low_Q_max = g_Q_MAX_POS_0[g_fg_battery_id];
		high_temperature = batt_meter_cust_data.temperature_t2;
		high_Q_max = g_Q_MAX_POS_25[g_fg_battery_id];

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else {
		low_temperature = batt_meter_cust_data.temperature_t2;
		low_Q_max = g_Q_MAX_POS_25[g_fg_battery_id];
		high_temperature = batt_meter_cust_data.temperature_t3;
		high_Q_max = g_Q_MAX_POS_50[g_fg_battery_id];

		if (temperature > high_temperature)
			temperature = high_temperature;

	}

	ret_Q_max = low_Q_max + (((temperature - low_temperature) * (high_Q_max - low_Q_max)
				 ) / (high_temperature - low_temperature)
	    );

	bm_print(BM_LOG_FULL, "[fgauge_get_Q_max] Q_max = %d\r\n", ret_Q_max);

	return ret_Q_max;
}


kal_int32 fgauge_get_Q_max_high_current(kal_int16 temperature)
{
	kal_int32 ret_Q_max = 0;
	kal_int32 low_temperature = 0, high_temperature = 0;
	kal_int32 low_Q_max = 0, high_Q_max = 0;

	if (temperature <= batt_meter_cust_data.temperature_t1) {
		low_temperature = (-10);
		low_Q_max = g_Q_MAX_NEG_10_H_CURRENT[g_fg_battery_id];
		high_temperature = batt_meter_cust_data.temperature_t1;
		high_Q_max = g_Q_MAX_POS_0_H_CURRENT[g_fg_battery_id];

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else if (temperature <= batt_meter_cust_data.temperature_t2) {
		low_temperature = batt_meter_cust_data.temperature_t1;
		low_Q_max = g_Q_MAX_POS_0_H_CURRENT[g_fg_battery_id];
		high_temperature = batt_meter_cust_data.temperature_t2;
		high_Q_max = g_Q_MAX_POS_25_H_CURRENT[g_fg_battery_id];

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else {
		low_temperature = batt_meter_cust_data.temperature_t2;
		low_Q_max = g_Q_MAX_POS_25_H_CURRENT[g_fg_battery_id];
		high_temperature = batt_meter_cust_data.temperature_t3;
		high_Q_max = g_Q_MAX_POS_50_H_CURRENT[g_fg_battery_id];

		if (temperature > high_temperature)
			temperature = high_temperature;

	}

	ret_Q_max = low_Q_max + (((temperature - low_temperature) * (high_Q_max - low_Q_max)
				 ) / (high_temperature - low_temperature)
	    );

	bm_print(BM_LOG_FULL, "[fgauge_get_Q_max_high_current] Q_max = %d\r\n", ret_Q_max);

	return ret_Q_max;
}

#else

int BattThermistorConverTemp(int Res)
{
	int i = 0;
	int RES1 = 0, RES2 = 0;
	int TBatt_Value = -200, TMP1 = 0, TMP2 = 0;

	if (Res >= Batt_Temperature_Table[0].TemperatureR) {
		TBatt_Value = -20;
	} else if (Res <= Batt_Temperature_Table[16].TemperatureR) {
		TBatt_Value = 60;
	} else {
		RES1 = Batt_Temperature_Table[0].TemperatureR;
		TMP1 = Batt_Temperature_Table[0].BatteryTemp;

		for (i = 0; i <= 16; i++) {
			if (Res >= Batt_Temperature_Table[i].TemperatureR) {
				RES2 = Batt_Temperature_Table[i].TemperatureR;
				TMP2 = Batt_Temperature_Table[i].BatteryTemp;
				break;
			} else {
				RES1 = Batt_Temperature_Table[i].TemperatureR;
				TMP1 = Batt_Temperature_Table[i].BatteryTemp;
			}
		}

		TBatt_Value = (((Res - RES2) * TMP1) + ((RES1 - Res) * TMP2)) / (RES1 - RES2);
	}

	return TBatt_Value;
}

kal_int32 fgauge_get_Q_max(kal_int16 temperature)
{
	kal_int32 ret_Q_max = 0;
	kal_int32 low_temperature = 0, high_temperature = 0;
	kal_int32 low_Q_max = 0, high_Q_max = 0;

	if (temperature <= batt_meter_cust_data.temperature_t1) {
		low_temperature = (-10);
		low_Q_max = batt_meter_cust_data.q_max_neg_10;
		high_temperature = batt_meter_cust_data.temperature_t1;
		high_Q_max = batt_meter_cust_data.q_max_pos_0;

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else if (temperature <= batt_meter_cust_data.temperature_t2) {
		low_temperature = batt_meter_cust_data.temperature_t1;
		low_Q_max = batt_meter_cust_data.q_max_pos_0;
		high_temperature = batt_meter_cust_data.temperature_t2;
		high_Q_max = batt_meter_cust_data.q_max_pos_25;

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else {
		low_temperature = batt_meter_cust_data.temperature_t2;
		low_Q_max = batt_meter_cust_data.q_max_pos_25;
		high_temperature = batt_meter_cust_data.temperature_t3;
		high_Q_max = batt_meter_cust_data.q_max_pos_50;

		if (temperature > high_temperature)
			temperature = high_temperature;

	}

	ret_Q_max = low_Q_max + (((temperature - low_temperature) * (high_Q_max - low_Q_max)
				 ) / (high_temperature - low_temperature)
	    );

	bm_print(BM_LOG_FULL, "[fgauge_get_Q_max] Q_max = %d\r\n", ret_Q_max);

	return ret_Q_max;
}


kal_int32 fgauge_get_Q_max_high_current(kal_int16 temperature)
{
	kal_int32 ret_Q_max = 0;
	kal_int32 low_temperature = 0, high_temperature = 0;
	kal_int32 low_Q_max = 0, high_Q_max = 0;

	if (temperature <= batt_meter_cust_data.temperature_t1) {
		low_temperature = (-10);
		low_Q_max = batt_meter_cust_data.q_max_neg_10_h_current;
		high_temperature = batt_meter_cust_data.temperature_t1;
		high_Q_max = batt_meter_cust_data.q_max_pos_0_h_current;

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else if (temperature <= batt_meter_cust_data.temperature_t2) {
		low_temperature = batt_meter_cust_data.temperature_t1;
		low_Q_max = batt_meter_cust_data.q_max_pos_0_h_current;
		high_temperature = batt_meter_cust_data.temperature_t2;
		high_Q_max = batt_meter_cust_data.q_max_pos_25_h_current;

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else {
		low_temperature = batt_meter_cust_data.temperature_t2;
		low_Q_max = batt_meter_cust_data.q_max_pos_25_h_current;
		high_temperature = batt_meter_cust_data.temperature_t3;
		high_Q_max = batt_meter_cust_data.q_max_pos_50_h_current;

		if (temperature > high_temperature)
			temperature = high_temperature;

	}

	ret_Q_max = low_Q_max + (((temperature - low_temperature) * (high_Q_max - low_Q_max)
				 ) / (high_temperature - low_temperature)
	    );

	bm_print(BM_LOG_FULL, "[fgauge_get_Q_max_high_current] Q_max = %d\r\n", ret_Q_max);

	return ret_Q_max;
}

#endif

int BattVoltToTemp(int dwVolt)
{
	kal_int64 TRes_temp;
	kal_int64 TRes;
	int sBaTTMP = -100;

	/* TRes_temp = ((kal_int64)RBAT_PULL_UP_R*(kal_int64)dwVolt) / (RBAT_PULL_UP_VOLT-dwVolt); */
	/* TRes = (TRes_temp * (kal_int64)RBAT_PULL_DOWN_R)/((kal_int64)RBAT_PULL_DOWN_R - TRes_temp); */

	TRes_temp = (batt_meter_cust_data.rbat_pull_up_r * (kal_int64) dwVolt);
#ifdef RBAT_PULL_UP_VOLT_BY_BIF
	do_div(TRes_temp, (pmic_get_vbif28_volt() - dwVolt));
	/* bm_print(BM_LOG_CRTI, "[RBAT_PULL_UP_VOLT_BY_BIF] vbif28:%d \n",pmic_get_vbif28_volt()); */
#else
	do_div(TRes_temp, (batt_meter_cust_data.rbat_pull_up_volt - dwVolt));
#endif

#ifdef RBAT_PULL_DOWN_R
	TRes = (TRes_temp * RBAT_PULL_DOWN_R);
	do_div(TRes, abs(RBAT_PULL_DOWN_R - TRes_temp));
#else
	TRes = TRes_temp;
#endif

	/* convert register to temperature */
	sBaTTMP = BattThermistorConverTemp((int)TRes);

	return sBaTTMP;
}

int force_get_tbat(kal_bool update)
{
#if defined(CONFIG_POWER_EXT) || defined(FIXED_TBAT_25)
	bm_print(BM_LOG_CRTI, "[force_get_tbat] fixed TBAT=25 t\n");
	return 25;
#else
	int bat_temperature_volt = 0;
	int bat_temperature_val = 0;
	static int pre_bat_temperature_val = -1;
	int fg_r_value = 0;
	kal_int32 fg_current_temp = 0;
	kal_bool fg_current_state = KAL_FALSE;
	int bat_temperature_volt_temp = 0;
	int ret = 0;

	if (update == KAL_TRUE || pre_bat_temperature_val == -1) {
		/* Get V_BAT_Temperature */
		bat_temperature_volt = 2;
		ret =
		    battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_BAT_TEMP, &bat_temperature_volt);

		if (bat_temperature_volt != 0) {
#if defined(SOC_BY_HW_FG)
			fg_r_value = get_r_fg_value();

			ret =
			    battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT,
					       &fg_current_temp);
			ret =
			    battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT_SIGN,
					       &fg_current_state);
			fg_current_temp = fg_current_temp / 10;

			if (fg_current_state == KAL_TRUE) {
				bat_temperature_volt_temp = bat_temperature_volt;
				bat_temperature_volt =
				    bat_temperature_volt - ((fg_current_temp * fg_r_value) / 1000);
			} else {
				bat_temperature_volt_temp = bat_temperature_volt;
				bat_temperature_volt =
				    bat_temperature_volt + ((fg_current_temp * fg_r_value) / 1000);
			}
#endif

			bat_temperature_val = BattVoltToTemp(bat_temperature_volt);
		}
#ifdef CONFIG_MTK_BIF_SUPPORT
		battery_charging_control(CHARGING_CMD_GET_BIF_TBAT, &bat_temperature_val);
#endif
		bm_print(BM_LOG_CRTI, "[force_get_tbat] %d,%d,%d,%d,%d,%d\n",
			 bat_temperature_volt_temp, bat_temperature_volt, fg_current_state,
			 fg_current_temp, fg_r_value, bat_temperature_val);
		pre_bat_temperature_val = bat_temperature_val;
	} else {
		bat_temperature_val = pre_bat_temperature_val;
	}
	return bat_temperature_val;
#endif
}
EXPORT_SYMBOL(force_get_tbat);

#ifdef MTK_MULTI_BAT_PROFILE_SUPPORT
int fgauge_get_saddles(void)
{
	return sizeof(battery_profile_temperature) / sizeof(BATTERY_PROFILE_STRUC);
}

int fgauge_get_saddles_r_table(void)
{
	return sizeof(r_profile_temperature) / sizeof(R_PROFILE_STRUC);
}

BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature)
{
	switch (temperature) {
	case batt_meter_cust_data.temperature_t0:
		return &battery_profile_t0[g_fg_battery_id][0];
		break;
	case batt_meter_cust_data.temperature_t1:
		return &battery_profile_t1[g_fg_battery_id][0];
		break;
	case batt_meter_cust_data.temperature_t2:
		return &battery_profile_t2[g_fg_battery_id][0];
		break;
	case batt_meter_cust_data.temperature_t3:
		return &battery_profile_t3[g_fg_battery_id][0];
		break;
	case batt_meter_cust_data.temperature_t:
		return &battery_profile_temperature[0];
		break;
	default:
		return NULL;
		break;
	}
}

R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature)
{
	switch (temperature) {
	case batt_meter_cust_data.temperature_t0:
		return &r_profile_t0[g_fg_battery_id][0];
		break;
	case batt_meter_cust_data.temperature_t1:
		return &r_profile_t1[g_fg_battery_id][0];
		break;
	case batt_meter_cust_data.temperature_t2:
		return &r_profile_t2[g_fg_battery_id][0];
		break;
	case batt_meter_cust_data.temperature_t3:
		return &r_profile_t3[g_fg_battery_id][0];
		break;
	case batt_meter_cust_data.temperature_t:
		return &r_profile_temperature[0];
		break;
	default:
		return NULL;
		break;
	}
}
#else
int fgauge_get_saddles(void)
{
	return sizeof(battery_profile_t2) / sizeof(BATTERY_PROFILE_STRUC);
}

int fgauge_get_saddles_r_table(void)
{
	return sizeof(r_profile_t2) / sizeof(R_PROFILE_STRUC);
}

BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature)
{
	if (temperature == batt_meter_cust_data.temperature_t0)
		return &battery_profile_t0[0];

	if (temperature == batt_meter_cust_data.temperature_t1)
		return &battery_profile_t1[0];

	if (temperature == batt_meter_cust_data.temperature_t2)
		return &battery_profile_t2[0];

	if (temperature == batt_meter_cust_data.temperature_t3)
		return &battery_profile_t3[0];

	if (temperature == batt_meter_cust_data.temperature_t)
		return &battery_profile_temperature[0];


	return NULL;

}

R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature)
{
	if (temperature == batt_meter_cust_data.temperature_t0)
		return &r_profile_t0[0];

	if (temperature == batt_meter_cust_data.temperature_t1)
		return &r_profile_t1[0];

	if (temperature == batt_meter_cust_data.temperature_t2)
		return &r_profile_t2[0];

	if (temperature == batt_meter_cust_data.temperature_t3)
		return &r_profile_t3[0];

	if (temperature == batt_meter_cust_data.temperature_t)
		return &r_profile_temperature[0];

	return NULL;
}
#endif

kal_int32 fgauge_read_capacity_by_v(kal_int32 voltage)
{
	int i = 0, saddles = 0;
	BATTERY_PROFILE_STRUC_P profile_p;
	kal_int32 ret_percent = 0;

	profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t);
	if (profile_p == NULL) {
		bm_print(BM_LOG_CRTI, "[FGADC] fgauge get ZCV profile : fail !\r\n");
		return 100;
	}

	saddles = fgauge_get_saddles();

	if (voltage > (profile_p + 0)->voltage)
		return 100;	/* battery capacity, not dod */

	if (voltage < (profile_p + saddles - 1)->voltage)
		return 0;	/* battery capacity, not dod */


	for (i = 0; i < saddles - 1; i++) {
		if ((voltage <= (profile_p + i)->voltage)
		    && (voltage >= (profile_p + i + 1)->voltage)) {
			ret_percent =
			    (profile_p + i)->percentage +
			    (((((profile_p + i)->voltage) -
			       voltage) * (((profile_p + i + 1)->percentage) -
					   ((profile_p + i)->percentage))
			     ) / (((profile_p + i)->voltage) - ((profile_p + i + 1)->voltage))
			    );

			break;
		}

	}
	ret_percent = 100 - ret_percent;

	return ret_percent;
}

kal_int32 fgauge_read_v_by_capacity(int bat_capacity)
{
	int i = 0, saddles = 0;
	BATTERY_PROFILE_STRUC_P profile_p;
	kal_int32 ret_volt = 0;

	profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t);
	if (profile_p == NULL) {
		bm_print(BM_LOG_CRTI,
			 "[fgauge_read_v_by_capacity] fgauge get ZCV profile : fail !\r\n");
		return 3700;
	}

	saddles = fgauge_get_saddles();

	if (bat_capacity < (profile_p + 0)->percentage)
		return 3700;

	if (bat_capacity > (profile_p + saddles - 1)->percentage)
		return 3700;


	for (i = 0; i < saddles - 1; i++) {
		if ((bat_capacity >= (profile_p + i)->percentage)
		    && (bat_capacity <= (profile_p + i + 1)->percentage)) {
			ret_volt =
			    (profile_p + i)->voltage -
			    (((bat_capacity -
			       ((profile_p + i)->percentage)) * (((profile_p + i)->voltage) -
								 ((profile_p + i + 1)->voltage))
			     ) / (((profile_p + i + 1)->percentage) - ((profile_p + i)->percentage))
			    );

			break;
		}
	}

	return ret_volt;
}

kal_int32 fgauge_read_d_by_v(kal_int32 volt_bat)
{
	int i = 0, saddles = 0;
	BATTERY_PROFILE_STRUC_P profile_p;
	kal_int32 ret_d = 0;

	profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t);
	if (profile_p == NULL) {
		bm_print(BM_LOG_CRTI, "[FGADC] fgauge get ZCV profile : fail !\r\n");
		return 100;
	}

	saddles = fgauge_get_saddles();

	if (volt_bat > (profile_p + 0)->voltage)
		return 0;

	if (volt_bat < (profile_p + saddles - 1)->voltage)
		return 100;


	for (i = 0; i < saddles - 1; i++) {
		if ((volt_bat <= (profile_p + i)->voltage)
		    && (volt_bat >= (profile_p + i + 1)->voltage)) {
			ret_d =
			    (profile_p + i)->percentage +
			    (((((profile_p + i)->voltage) -
			       volt_bat) * (((profile_p + i + 1)->percentage) -
					    ((profile_p + i)->percentage))
			     ) / (((profile_p + i)->voltage) - ((profile_p + i + 1)->voltage))
			    );

			break;
		}

	}

	return ret_d;
}

kal_int32 fgauge_read_v_by_d(int d_val)
{
	int i = 0, saddles = 0;
	BATTERY_PROFILE_STRUC_P profile_p;
	kal_int32 ret_volt = 0;

	profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t);
	if (profile_p == NULL) {
		bm_print(BM_LOG_CRTI,
			 "[fgauge_read_v_by_capacity] fgauge get ZCV profile : fail !\r\n");
		return 3700;
	}

	saddles = fgauge_get_saddles();

	if (d_val < (profile_p + 0)->percentage)
		return 3700;

	if (d_val > (profile_p + saddles - 1)->percentage)
		return 3700;


	for (i = 0; i < saddles - 1; i++) {
		if ((d_val >= (profile_p + i)->percentage)
		    && (d_val <= (profile_p + i + 1)->percentage)) {
			ret_volt =
			    (profile_p + i)->voltage -
			    (((d_val -
			       ((profile_p + i)->percentage)) * (((profile_p + i)->voltage) -
								 ((profile_p + i + 1)->voltage))
			     ) / (((profile_p + i + 1)->percentage) - ((profile_p + i)->percentage))
			    );

			break;
		}
	}

	return ret_volt;
}

kal_int32 fgauge_read_r_bat_by_v(kal_int32 voltage)
{
	int i = 0, saddles = 0;
	R_PROFILE_STRUC_P profile_p;
	kal_int32 ret_r = 0;

	profile_p = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t);
	if (profile_p == NULL) {
		bm_print(BM_LOG_CRTI, "[FGADC] fgauge get R-Table profile : fail !\r\n");
		return (profile_p + 0)->resistance;
	}

	saddles = fgauge_get_saddles_r_table();

	if (voltage > (profile_p + 0)->voltage)
		return (profile_p + 0)->resistance;

	if (voltage < (profile_p + saddles - 1)->voltage)
		return (profile_p + saddles - 1)->resistance;


	for (i = 0; i < saddles - 1; i++) {
		if ((voltage <= (profile_p + i)->voltage)
		    && (voltage >= (profile_p + i + 1)->voltage)) {
			ret_r =
			    (profile_p + i)->resistance +
			    (((((profile_p + i)->voltage) -
			       voltage) * (((profile_p + i + 1)->resistance) -
					   ((profile_p + i)->resistance))
			     ) / (((profile_p + i)->voltage) - ((profile_p + i + 1)->voltage))
			    );
			break;
		}
	}

	return ret_r;
}

void fgauge_construct_battery_profile_init(void)
{
	BATTERY_PROFILE_STRUC_P temp_profile_p, profile_p[PROFILE_SIZE];
	int i, j, saddles, profile_index;
	kal_int32 low_p = 0, high_p = 0, now_p = 0, low_vol = 0, high_vol = 0;

	profile_p[0] = fgauge_get_profile(batt_meter_cust_data.temperature_t0);
	profile_p[1] = fgauge_get_profile(batt_meter_cust_data.temperature_t1);
	profile_p[2] = fgauge_get_profile(batt_meter_cust_data.temperature_t2);
	profile_p[3] = fgauge_get_profile(batt_meter_cust_data.temperature_t3);
	saddles = fgauge_get_saddles();
	temp_profile_p =
	    (BATTERY_PROFILE_STRUC_P) kmalloc(51 * sizeof(*temp_profile_p), GFP_KERNEL);
	memset(temp_profile_p, 0, 51 * sizeof(*temp_profile_p));
	for (i = 0; i < PROFILE_SIZE; i++) {
		profile_index = 0;
		for (j = 0; j * 2 <= 100; j++) {
			while (profile_index < saddles && profile_index >= 0) {
				if (((profile_p[i] + profile_index)->percentage) < j * 2) {
					profile_index++;
					continue;
				} else if (((profile_p[i] + profile_index)->percentage) == j * 2) {
					(temp_profile_p + j)->voltage =
					    (profile_p[i] + profile_index)->voltage;
					(temp_profile_p + j)->percentage =
					    (profile_p[i] + profile_index)->percentage;
					break;
				}
				low_p = (profile_p[i] + profile_index - 1)->percentage;
				high_p = (profile_p[i] + profile_index)->percentage;
				now_p = j * 2;
				low_vol = (profile_p[i] + profile_index)->voltage;
				high_vol = (profile_p[i] + profile_index - 1)->voltage;
				(temp_profile_p + j)->voltage =
				    (low_vol * 1000 +
				     ((high_vol - low_vol) * 1000 * (now_p - low_p) / (high_p -
										       low_p))) /
				    1000;
				(temp_profile_p + j)->percentage = j * 2;

				break;
			}
			bm_print(BM_LOG_CRTI, "new battery_profile[%d,%d] <%d,%d>\n", i, j,
				 (temp_profile_p + j)->percentage, (temp_profile_p + j)->voltage);
		}

		for (j = 0; j * 2 <= 100; j++) {
			(profile_p[i] + j)->voltage = (temp_profile_p + j)->voltage;
			(profile_p[i] + j)->percentage = (temp_profile_p + j)->percentage;
		}
	}
	kfree(temp_profile_p);
}

void fgauge_construct_battery_profile(kal_int32 temperature, BATTERY_PROFILE_STRUC_P temp_profile_p)
{
	BATTERY_PROFILE_STRUC_P low_profile_p, high_profile_p;
	kal_int32 low_temperature, high_temperature;
	int i, saddles;
	kal_int32 temp_v_1 = 0, temp_v_2 = 0;

	if (temperature <= batt_meter_cust_data.temperature_t1) {
		low_profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t0);
		high_profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t1);
		low_temperature = (-10);
		high_temperature = batt_meter_cust_data.temperature_t1;

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else if (temperature <= batt_meter_cust_data.temperature_t2) {
		low_profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t1);
		high_profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t2);
		low_temperature = batt_meter_cust_data.temperature_t1;
		high_temperature = batt_meter_cust_data.temperature_t2;

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else {
		low_profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t2);
		high_profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t3);
		low_temperature = batt_meter_cust_data.temperature_t2;
		high_temperature = batt_meter_cust_data.temperature_t3;

		if (temperature > high_temperature)
			temperature = high_temperature;

	}

	saddles = fgauge_get_saddles();

	for (i = 0; i < saddles; i++) {
		if (((high_profile_p + i)->voltage) > ((low_profile_p + i)->voltage)) {
			temp_v_1 = (high_profile_p + i)->voltage;
			temp_v_2 = (low_profile_p + i)->voltage;

			(temp_profile_p + i)->voltage = temp_v_2 +
			    (((temperature - low_temperature) * (temp_v_1 - temp_v_2)
			     ) / (high_temperature - low_temperature)
			    );
		} else {
			temp_v_1 = (low_profile_p + i)->voltage;
			temp_v_2 = (high_profile_p + i)->voltage;

			(temp_profile_p + i)->voltage = temp_v_2 +
			    (((high_temperature - temperature) * (temp_v_1 - temp_v_2)
			     ) / (high_temperature - low_temperature)
			    );
		}

		(temp_profile_p + i)->percentage = (high_profile_p + i)->percentage;
#if 0
		(temp_profile_p + i)->voltage = temp_v_2 +
		    (((temperature - low_temperature) * (temp_v_1 - temp_v_2)
		     ) / (high_temperature - low_temperature)
		    );
#endif
	}


	/* Dumpt new battery profile
	for (i = 0; i < saddles; i++) {
		bm_print(BM_LOG_CRTI, "<DOD,Voltage> at %d = <%d,%d>\r\n",
			 temperature, (temp_profile_p + i)->percentage,
			 (temp_profile_p + i)->voltage);
	} */

}

void fgauge_construct_r_table_profile(kal_int32 temperature, R_PROFILE_STRUC_P temp_profile_p)
{
	R_PROFILE_STRUC_P low_profile_p, high_profile_p;
	kal_int32 low_temperature, high_temperature;
	int i, saddles;
	kal_int32 temp_v_1 = 0, temp_v_2 = 0;
	kal_int32 temp_r_1 = 0, temp_r_2 = 0;

	if (temperature <= batt_meter_cust_data.temperature_t1) {
		low_profile_p = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t0);
		high_profile_p = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t1);
		low_temperature = (-10);
		high_temperature = batt_meter_cust_data.temperature_t1;

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else if (temperature <= batt_meter_cust_data.temperature_t2) {
		low_profile_p = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t1);
		high_profile_p = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t2);
		low_temperature = batt_meter_cust_data.temperature_t1;
		high_temperature = batt_meter_cust_data.temperature_t2;

		if (temperature < low_temperature)
			temperature = low_temperature;

	} else {
		low_profile_p = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t2);
		high_profile_p = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t3);
		low_temperature = batt_meter_cust_data.temperature_t2;
		high_temperature = batt_meter_cust_data.temperature_t3;

		if (temperature > high_temperature)
			temperature = high_temperature;

	}

	saddles = fgauge_get_saddles_r_table();

	/* Interpolation for V_BAT */
	for (i = 0; i < saddles; i++) {
		if (((high_profile_p + i)->voltage) > ((low_profile_p + i)->voltage)) {
			temp_v_1 = (high_profile_p + i)->voltage;
			temp_v_2 = (low_profile_p + i)->voltage;

			(temp_profile_p + i)->voltage = temp_v_2 +
			    (((temperature - low_temperature) * (temp_v_1 - temp_v_2)
			     ) / (high_temperature - low_temperature)
			    );
		} else {
			temp_v_1 = (low_profile_p + i)->voltage;
			temp_v_2 = (high_profile_p + i)->voltage;

			(temp_profile_p + i)->voltage = temp_v_2 +
			    (((high_temperature - temperature) * (temp_v_1 - temp_v_2)
			     ) / (high_temperature - low_temperature)
			    );
		}

#if 0
		/* (temp_profile_p + i)->resistance = (high_profile_p + i)->resistance; */

		(temp_profile_p + i)->voltage = temp_v_2 +
		    (((temperature - low_temperature) * (temp_v_1 - temp_v_2)
		     ) / (high_temperature - low_temperature)
		    );
#endif
	}

	/* Interpolation for R_BAT */
	for (i = 0; i < saddles; i++) {
		if (((high_profile_p + i)->resistance) > ((low_profile_p + i)->resistance)) {
			temp_r_1 = (high_profile_p + i)->resistance;
			temp_r_2 = (low_profile_p + i)->resistance;

			(temp_profile_p + i)->resistance = temp_r_2 +
			    (((temperature - low_temperature) * (temp_r_1 - temp_r_2)
			     ) / (high_temperature - low_temperature)
			    );
		} else {
			temp_r_1 = (low_profile_p + i)->resistance;
			temp_r_2 = (high_profile_p + i)->resistance;

			(temp_profile_p + i)->resistance = temp_r_2 +
			    (((high_temperature - temperature) * (temp_r_1 - temp_r_2)
			     ) / (high_temperature - low_temperature)
			    );
		}

#if 0
		/* (temp_profile_p + i)->voltage = (high_profile_p + i)->voltage; */

		(temp_profile_p + i)->resistance = temp_r_2 +
		    (((temperature - low_temperature) * (temp_r_1 - temp_r_2)
		     ) / (high_temperature - low_temperature)
		    );
#endif
	}

	/* Dumpt new r-table profile
	for (i = 0; i < saddles; i++) {
		bm_print(BM_LOG_CRTI, "<Rbat,VBAT> at %d = <%d,%d>\r\n",
			 temperature, (temp_profile_p + i)->resistance,
			 (temp_profile_p + i)->voltage);
	} */

}

void fgauge_construct_table_by_temp(void)
{
#if defined(CONFIG_POWER_EXT)
#else
	kal_uint32 i;
	static kal_int32 init_temp = KAL_TRUE;
	static kal_int32 curr_temp, last_temp, avg_temp;
	static kal_int32 battTempBuffer[TEMP_AVERAGE_SIZE];
	static kal_int32 temperature_sum;
	static kal_uint8 tempIndex;

	curr_temp = battery_meter_get_battery_temperature();

	/* Temperature window init */
	if (init_temp == KAL_TRUE) {
		for (i = 0; i < TEMP_AVERAGE_SIZE; i++)
			battTempBuffer[i] = curr_temp;

		last_temp = curr_temp;
		temperature_sum = curr_temp * TEMP_AVERAGE_SIZE;
		init_temp = KAL_FALSE;
	}
	/* Temperature sliding window */
	temperature_sum -= battTempBuffer[tempIndex];
	temperature_sum += curr_temp;
	battTempBuffer[tempIndex] = curr_temp;
	avg_temp = (temperature_sum) / TEMP_AVERAGE_SIZE;

	if (avg_temp != last_temp) {
		bm_print(BM_LOG_FULL,
			 "[fgauge_construct_table_by_temp] reconstruct table by temperature change from (%d) to (%d)\r\n",
			 last_temp, avg_temp);
		fgauge_construct_r_table_profile(curr_temp,
		fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t));
		fgauge_construct_battery_profile(curr_temp, fgauge_get_profile(batt_meter_cust_data.temperature_t));
		last_temp = avg_temp;
		temperature_change = 1;
		#if defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
		gFG_BATT_CAPACITY = fgauge_get_Q_max(curr_temp);
		gFG_BATT_CAPACITY_aging = gFG_BATT_CAPACITY;
		bm_print(BM_LOG_CRTI, "[%s]gFG_BATT_CAPACITY: %d\n", __func__, gFG_BATT_CAPACITY);
		#endif
	}

	tempIndex = (tempIndex + 1) % TEMP_AVERAGE_SIZE;

#endif
}

#if defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
/*
	ZCV table is created by 600mA loading.
	Here we calculate average current and get a factor based on 600mA.
*/
void fgauge_get_current_factor(void)
{
#if defined(CONFIG_POWER_EXT)
#else
	kal_uint32 i;
	static kal_int32 init_current = KAL_TRUE;
	static kal_int32 inst_current, avg_current;
	static kal_int32 battCurrentBuffer[TEMP_AVERAGE_SIZE];
	static kal_int32 current_sum;
	static kal_uint8 tempcurrentIndex;

	if (KAL_TRUE == gFG_Is_Charging) {
		init_current = KAL_TRUE;
		g_currentfactor = 100;
		bm_print(BM_LOG_CRTI, "[fgauge_get_current_factor] Charging!!\r\n");
		return;
	}

#if defined(SOC_BY_HW_FG)
	inst_current = gFG_current;
#elif #defined(SOC_BY_SW_FG)
#if (OAM_D5 == 1)
	inst_current = oam_i_1;
#else
	inst_current = oam_i_2;
#endif
#endif

	if (init_current == KAL_TRUE) {
		for (i = 0; i < TEMP_AVERAGE_SIZE; i++)
			battCurrentBuffer[i] = inst_current;

		current_sum = inst_current * TEMP_AVERAGE_SIZE;
		init_current = KAL_FALSE;
	}

	/* current sliding window */
	current_sum -= battCurrentBuffer[tempcurrentIndex];
	current_sum += inst_current;
	battCurrentBuffer[tempcurrentIndex] = inst_current;
	avg_current = (current_sum) / TEMP_AVERAGE_SIZE;

	g_currentfactor = avg_current * 100 / CV_CURRENT;	/* calculate factor by 600ma */

	bm_print(BM_LOG_CRTI, "[fgauge_get_current_factor] %d,%d,%d,%d\r\n",
		 inst_current, avg_current, g_currentfactor, gFG_Is_Charging);

	tempcurrentIndex = (tempcurrentIndex + 1) % TEMP_AVERAGE_SIZE;
#endif
}

/*
	ZCV table has battery OCV-to-resistance information.
	Based on a given discharging current value, we can get a new estimated Qmax.
	Qmax is defined as OCV -I*R < power off voltage.
	Default power off voltage is 3400mV.
*/

kal_int32 fgauge_get_Q_max_high_current_by_current(kal_int32 i_current, kal_int16 val_temp)
{
	kal_int32 ret_Q_max = 0;
	kal_int32 iIndex = 0, saddles = 0;
	kal_int32 OCV_temp = 0, Rbat_temp = 0, V_drop = 0;
	R_PROFILE_STRUC_P p_profile_r;
	BATTERY_PROFILE_STRUC_P p_profile_battery;
	kal_int32 threshold = SYSTEM_OFF_VOLTAGE;
	/* for Qmax initialization */
	ret_Q_max = fgauge_get_Q_max_high_current(val_temp);

	/* get Rbat and OCV table of the current temperature */
	p_profile_r = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t);
	p_profile_battery = fgauge_get_profile(batt_meter_cust_data.temperature_t);
	if (p_profile_r == NULL || p_profile_battery == NULL) {
		bm_print(BM_LOG_CRTI, "get R-Table profile/OCV table profile : fail !\r\n");
		return ret_Q_max;
	}

	if (0 == p_profile_r->resistance || 0 == p_profile_battery->voltage) {
		bm_print(BM_LOG_CRTI, "get R-Table profile/OCV table profile : not ready !\r\n");
		return ret_Q_max;
	}

	saddles = fgauge_get_saddles();

	/* get Qmax in current temperature (>3.4) */
	for (iIndex = 0; iIndex < saddles - 1; iIndex++) {
		OCV_temp = (p_profile_battery + iIndex)->voltage;
		Rbat_temp = (p_profile_r + iIndex)->resistance;
		V_drop = (i_current * Rbat_temp) / 10000;

		if (OCV_temp - V_drop < threshold) {
			if (iIndex <= 1)
				ret_Q_max = STEP_OF_QMAX;
			else
				ret_Q_max = (iIndex - 1) * STEP_OF_QMAX;
			break;
		}
	}

	bm_print(BM_LOG_CRTI, "[fgauge_get_Q_max_by_current] %d,%d,%d,%d,%d\r\n",
		 i_current, iIndex, OCV_temp, Rbat_temp, ret_Q_max);

	return ret_Q_max;
}
#endif

void fg_qmax_update_for_aging(void)
{
#if defined(CONFIG_POWER_EXT)
#else
	kal_bool hw_charging_done = bat_is_charging_full();

	if (hw_charging_done == KAL_TRUE) {	/* charging full, g_HW_Charging_Done == 1 */
		if (gFG_DOD0 > 85) {
			if (gFG_columb < 0)
				gFG_columb = gFG_columb - gFG_columb * 2;	/* absolute value */

			gFG_BATT_CAPACITY_aging =
			    (((gFG_columb * 1000) + (5 * gFG_DOD0)) / gFG_DOD0) / 10;

			/* tuning */
			gFG_BATT_CAPACITY_aging =
			    (gFG_BATT_CAPACITY_aging * 100) /
			    batt_meter_cust_data.aging_tuning_value;

			if (gFG_BATT_CAPACITY_aging == 0) {
				gFG_BATT_CAPACITY_aging =
				    fgauge_get_Q_max(battery_meter_get_battery_temperature());
				bm_print(BM_LOG_CRTI,
					 "[fg_qmax_update_for_aging] error, restore gFG_BATT_CAPACITY_aging (%d)\n",
					 gFG_BATT_CAPACITY_aging);
			}

			bm_print(BM_LOG_CRTI,
				 "[fg_qmax_update_for_aging] need update : gFG_columb=%d, gFG_DOD0=%d, new_qmax=%d\r\n",
				 gFG_columb, gFG_DOD0, gFG_BATT_CAPACITY_aging);
		} else {
			bm_print(BM_LOG_CRTI,
				 "[fg_qmax_update_for_aging] no update : gFG_columb=%d, gFG_DOD0=%d, new_qmax=%d\r\n",
				 gFG_columb, gFG_DOD0, gFG_BATT_CAPACITY_aging);
		}
	} else {
		bm_print(BM_LOG_CRTI, "[fg_qmax_update_for_aging] hw_charging_done=%d\r\n",
			 hw_charging_done);
	}
#endif
}


#if defined(SW_OAM_INIT_V2)
char bootbuf[100];
void sw_oam_init_v2(void)
{
	int ret = 0;
	int plugout_status = 0;
	int type = 0;

	/* use get_hw_ocv----------------------------------------------------------------- */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_OCV, &gFG_voltage);
	gFG_capacity_by_v = fgauge_read_capacity_by_v(gFG_voltage);

#if defined(CONFIG_POWER_EXT)
	g_rtc_fg_soc = gFG_capacity_by_v;
#else
	g_rtc_fg_soc = get_rtc_spare_fg_value();
#endif

	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_BATTERY_PLUG_STATUS, &plugout_status);

	if (plugout_status == 0 && bat_is_charger_exist() == KAL_FALSE) {
		if (g_rtc_fg_soc == 0) {
			gFG_capacity_by_v = gFG_capacity_by_v_init;	/* g_booting_vbat */
			type = 1;
		} else {
			gFG_capacity_by_v = g_rtc_fg_soc;
			type = 2;
		}
	} else {
		if ((abs(gFG_capacity_by_v - g_rtc_fg_soc) >
		     batt_meter_cust_data.cust_poweron_delta_capacity_tolrance)
		    && (abs(gFG_capacity_by_v - gFG_capacity_by_v_init) <
			abs(gFG_capacity_by_v_init - g_rtc_fg_soc))) {
			if (abs(gFG_capacity_by_v - gFG_capacity_by_v_init) >
			    batt_meter_cust_data.cust_poweron_delta_hw_sw_ocv_capacity_tolrance) {
				gFG_capacity_by_v = gFG_capacity_by_v_init;
				type = 3;
			} else {
				/* use hw ocv; */
				type = 4;
			}

		} else {
			if ((abs(g_rtc_fg_soc - gFG_capacity_by_v_init) >
			     batt_meter_cust_data.cust_poweron_delta_hw_sw_ocv_capacity_tolrance)
			    || g_rtc_fg_soc == 0) {
				gFG_capacity_by_v = gFG_capacity_by_v_init;
				type = 5;
			} else {
				gFG_capacity_by_v = g_rtc_fg_soc;
				type = 6;
			}
		}
	}


	bm_print(BM_LOG_CRTI,
		 "[sw_oam_init_v2] swocv:%d(%d) hwocv:%d(%d) rtc:%d plugout_status=%d chr:%d type:%d f:%d %d %d\n",
		 g_booting_vbat, gFG_capacity_by_v_init, gFG_voltage, gFG_capacity_by_v,
		 g_rtc_fg_soc, plugout_status, bat_is_charger_exist(), type, gFG_capacity_by_v,
		 batt_meter_cust_data.cust_poweron_delta_capacity_tolrance,
		 batt_meter_cust_data.cust_poweron_delta_hw_sw_ocv_capacity_tolrance);

	sprintf(bootbuf,
		"[sw_oam_init_v2] swocv:%d(%d) hwocv:%d(%d) rtc:%d plugout_status=%d chr:%d type:%d f:%d %d %d\n",
		g_booting_vbat, gFG_capacity_by_v_init, gFG_voltage, gFG_capacity_by_v,
		g_rtc_fg_soc, plugout_status, bat_is_charger_exist(), type, gFG_capacity_by_v,
		batt_meter_cust_data.cust_poweron_delta_capacity_tolrance,
		batt_meter_cust_data.cust_poweron_delta_hw_sw_ocv_capacity_tolrance);
}
#endif

void dod_init(void)
{
#if defined(SOC_BY_HW_FG)
	int ret = 0;

#if defined(IS_BATTERY_REMOVE_BY_PMIC)
	kal_int32 gFG_capacity_by_sw_ocv = gFG_capacity_by_v;
#endif				/* #if defined(IS_BATTERY_REMOVE_BY_PMIC) */

	/* use get_hw_ocv----------------------------------------------------------------- */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_OCV, &gFG_voltage);
	gFG_capacity_by_v = fgauge_read_capacity_by_v(gFG_voltage);

	bm_print(BM_LOG_CRTI, "[FGADC] get_hw_ocv=%d, HW_SOC=%d, SW_SOC = %d\n",
		 gFG_voltage, gFG_capacity_by_v, gFG_capacity_by_v_init);
#if defined(EXTERNAL_SWCHR_SUPPORT)
	/* compare with hw_ocv & sw_ocv, check if less than or equal to 5% tolerance */
	if ((abs(gFG_capacity_by_v_init - gFG_capacity_by_v) > 5)
	    && (bat_is_charger_exist() == KAL_TRUE)) {
		gFG_capacity_by_v = gFG_capacity_by_v_init;
	}
#endif
#if defined(HW_FG_FORCE_USE_SW_OCV)
	gFG_capacity_by_v = gFG_capacity_by_v_init;
	bm_print(BM_LOG_CRTI, "[FGADC] HW_FG_FORCE_USE_SW_OCV : HW_SOC=%d, SW_SOC = %d\n",
		 gFG_capacity_by_v, gFG_capacity_by_v_init);
#endif
	/* ------------------------------------------------------------------------------- */
#endif

#if defined(CONFIG_POWER_EXT)
	g_rtc_fg_soc = gFG_capacity_by_v;
#else
	g_rtc_fg_soc = get_rtc_spare_fg_value();
#endif


#if defined(IS_BATTERY_REMOVE_BY_PMIC)
	if (is_battery_remove_pmic() == 0 && (g_rtc_fg_soc != 0)) {
		bm_print(BM_LOG_CRTI, "[FGADC]is_battery_remove()==0 , use rtc_fg_soc%d\n",
			 g_rtc_fg_soc);
		gFG_capacity_by_v = g_rtc_fg_soc;
	} else {

#if defined(INIT_SOC_BY_SW_SOC)
if (((g_rtc_fg_soc != 0)
		     &&
		     (((abs(g_rtc_fg_soc - gFG_capacity_by_v)) <=
		       batt_meter_cust_data.cust_poweron_delta_capacity_tolrance)
		      || (abs(gFG_capacity_by_v_init - g_rtc_fg_soc) <
			  abs(gFG_capacity_by_v - gFG_capacity_by_v_init))))
		    || ((g_rtc_fg_soc != 0)
			&& (g_boot_reason == BR_WDT_BY_PASS_PWK || g_boot_reason == BR_WDT
			    || g_boot_reason == BR_TOOL_BY_PASS_PWK
			    || g_boot_reason == BR_2SEC_REBOOT || g_boot_mode == RECOVERY_BOOT)))
#else
if (((g_rtc_fg_soc != 0)
		     &&
		     (((abs(g_rtc_fg_soc - gFG_capacity_by_v)) <
		       batt_meter_cust_data.cust_poweron_delta_capacity_tolrance))
		     &&
		     ((gFG_capacity_by_v > batt_meter_cust_data.cust_poweron_low_capacity_tolrance
		       || bat_is_charger_exist() == KAL_TRUE)))
		    || ((g_rtc_fg_soc != 0)
			&& (g_boot_reason == BR_WDT_BY_PASS_PWK || g_boot_reason == BR_WDT
			    || g_boot_reason == BR_TOOL_BY_PASS_PWK
			    || g_boot_reason == BR_2SEC_REBOOT || g_boot_mode == RECOVERY_BOOT)))
#endif
		{
			gFG_capacity_by_v = g_rtc_fg_soc;
		} else {
			if (abs(gFG_capacity_by_v - gFG_capacity_by_sw_ocv) >
			    batt_meter_cust_data.cust_poweron_delta_hw_sw_ocv_capacity_tolrance) {
				bm_print(BM_LOG_CRTI,
					 "[FGADC] gFG_capacity_by_v=%d, gFG_capacity_by_sw_ocv=%d use SWOCV\n",
					 gFG_capacity_by_v, gFG_capacity_by_sw_ocv);
				gFG_capacity_by_v = gFG_capacity_by_sw_ocv;
			} else {
				bm_print(BM_LOG_CRTI,
					 "[FGADC] gFG_capacity_by_v=%d, gFG_capacity_by_sw_ocv=%d use HWOCV\n",
					 gFG_capacity_by_v, gFG_capacity_by_sw_ocv);
			}
		}

	}

#else

#if defined(SOC_BY_HW_FG)
#if defined(INIT_SOC_BY_SW_SOC)
if (((g_rtc_fg_soc != 0)
	     &&
	     (((abs(g_rtc_fg_soc - gFG_capacity_by_v)) <=
	       batt_meter_cust_data.cust_poweron_delta_capacity_tolrance)
	      || (abs(gFG_capacity_by_v_init - g_rtc_fg_soc) <
		  abs(gFG_capacity_by_v - gFG_capacity_by_v_init))))
	    || ((g_rtc_fg_soc != 0)
		&& (g_boot_reason == BR_WDT_BY_PASS_PWK || g_boot_reason == BR_WDT
		    || g_boot_reason == BR_TOOL_BY_PASS_PWK || g_boot_reason == BR_2SEC_REBOOT
		    || g_boot_mode == RECOVERY_BOOT)))
#else
if (((g_rtc_fg_soc != 0)
	     &&
	     (((abs(g_rtc_fg_soc - gFG_capacity_by_v)) <
	       batt_meter_cust_data.cust_poweron_delta_capacity_tolrance))
	     &&
	     ((gFG_capacity_by_v > batt_meter_cust_data.cust_poweron_low_capacity_tolrance
	       || bat_is_charger_exist() == KAL_TRUE)))
	    || ((g_rtc_fg_soc != 0)
		&& (g_boot_reason == BR_WDT_BY_PASS_PWK || g_boot_reason == BR_WDT
		    || g_boot_reason == BR_TOOL_BY_PASS_PWK || g_boot_reason == BR_2SEC_REBOOT
		    || g_boot_mode == RECOVERY_BOOT)))
#endif
	{
		gFG_capacity_by_v = g_rtc_fg_soc;
	}
#elif defined(SOC_BY_SW_FG)
	if (((g_rtc_fg_soc != 0)
	     &&
	     (((abs(g_rtc_fg_soc - gFG_capacity_by_v)) <
	       batt_meter_cust_data.cust_poweron_delta_capacity_tolrance)
	      || (abs(g_rtc_fg_soc - g_booting_vbat) <
		  batt_meter_cust_data.cust_poweron_delta_capacity_tolrance))
	     &&
	     ((gFG_capacity_by_v > batt_meter_cust_data.cust_poweron_low_capacity_tolrance
	       || bat_is_charger_exist() == KAL_TRUE)))
	    || ((g_rtc_fg_soc != 0)
		&& (g_boot_reason == BR_WDT_BY_PASS_PWK || g_boot_reason == BR_WDT
		    || g_boot_reason == BR_TOOL_BY_PASS_PWK || g_boot_reason == BR_2SEC_REBOOT
		    || g_boot_mode == RECOVERY_BOOT))) {
		gFG_capacity_by_v = g_rtc_fg_soc;
	}
#endif
#endif

#if defined(SW_OAM_INIT_V2)
	sw_oam_init_v2();
#endif

	bm_print(BM_LOG_CRTI, "[FGADC] g_rtc_fg_soc=%d, gFG_capacity_by_v=%d\n",
		 g_rtc_fg_soc, gFG_capacity_by_v);

	if (gFG_capacity_by_v == 0 && bat_is_charger_exist() == KAL_TRUE) {
		gFG_capacity_by_v = 1;

		bm_print(BM_LOG_CRTI, "[FGADC] gFG_capacity_by_v=%d\n", gFG_capacity_by_v);
	}
	gFG_capacity = gFG_capacity_by_v;
	gFG_capacity_by_c_init = gFG_capacity;
	gFG_capacity_by_c = gFG_capacity;

	gFG_DOD0 = 100 - gFG_capacity;
	gFG_DOD1 = gFG_DOD0;

	gfg_percent_check_point = gFG_capacity;

	if (batt_meter_cust_data.change_tracking_point) {
		gFG_15_vlot = fgauge_read_v_by_capacity((100 - g_tracking_point));
		bm_print(BM_LOG_CRTI, "[FGADC] gFG_15_vlot = %dmV\n", gFG_15_vlot);
	} else {
		/* gFG_15_vlot = fgauge_read_v_by_capacity(86); //14% */
		gFG_15_vlot = fgauge_read_v_by_capacity((100 - g_tracking_point));
		bm_print(BM_LOG_CRTI, "[FGADC] gFG_15_vlot = %dmV\n", gFG_15_vlot);
		if ((gFG_15_vlot > 3800) || (gFG_15_vlot < 3600)) {
			bm_print(BM_LOG_CRTI, "[FGADC] gFG_15_vlot(%d) over range, reset to 3700\n",
				 gFG_15_vlot);
			gFG_15_vlot = 3700;
		}
	}
}

/* ============================================================ // SW FG */
kal_int32 mtk_imp_tracking(kal_int32 ori_voltage, kal_int32 ori_current, kal_int32 recursion_time)
{
	kal_int32 ret_compensate_value = 0;
	kal_int32 temp_voltage_1 = ori_voltage;
	kal_int32 temp_voltage_2 = temp_voltage_1;
	int i = 0;

	for (i = 0; i < recursion_time; i++) {
		gFG_resistance_bat = fgauge_read_r_bat_by_v(temp_voltage_2);
		ret_compensate_value =
		    ((ori_current) * (gFG_resistance_bat + batt_meter_cust_data.r_fg_value)) / 1000;
		ret_compensate_value = (ret_compensate_value + (10 / 2)) / 10;
		temp_voltage_2 = temp_voltage_1 + ret_compensate_value;

		bm_print(BM_LOG_FULL,
			 "[mtk_imp_tracking] temp_voltage_2=%d,temp_voltage_1=%d,ret_compensate_value=%d,gFG_resistance_bat=%d\n",
			 temp_voltage_2, temp_voltage_1, ret_compensate_value, gFG_resistance_bat);
	}

	gFG_resistance_bat = fgauge_read_r_bat_by_v(temp_voltage_2);
	ret_compensate_value =
	    ((ori_current) *
	     (gFG_resistance_bat + batt_meter_cust_data.r_fg_value +
	      batt_meter_cust_data.fg_meter_resistance)) / 1000;
	ret_compensate_value = (ret_compensate_value + (10 / 2)) / 10;

	gFG_compensate_value = ret_compensate_value;

	bm_print(BM_LOG_FULL,
		 "[mtk_imp_tracking] temp_voltage_2=%d,temp_voltage_1=%d,ret_compensate_value=%d,gFG_resistance_bat=%d\n",
		 temp_voltage_2, temp_voltage_1, ret_compensate_value, gFG_resistance_bat);

	return ret_compensate_value;
}

void oam_init(void)
{
	int ret = 0;
	kal_int32 vbat_capacity = 0;
	kal_bool charging_enable = KAL_FALSE;

	/*stop charging for vbat measurement */
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	msleep(50);

	g_booting_vbat = 5;	/* set avg times */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_OCV, &gFG_voltage);
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_BAT_SENSE, &g_booting_vbat);


	gFG_capacity_by_v = fgauge_read_capacity_by_v(gFG_voltage);
	vbat_capacity = fgauge_read_capacity_by_v(g_booting_vbat);

	if (bat_is_charger_exist() == KAL_TRUE) {
		bm_print(BM_LOG_CRTI, "[oam_init_inf] gFG_capacity_by_v=%d, vbat_capacity=%d,\n",
			 gFG_capacity_by_v, vbat_capacity);

		/* to avoid plug in cable without battery, then plug in battery to make hw soc = 100% */
		/* if the difference bwtween ZCV and vbat is too large, using vbat instead ZCV */
		if (((gFG_capacity_by_v == 100)
		     && (vbat_capacity < batt_meter_cust_data.cust_poweron_max_vbat_tolrance))
		    || (abs(gFG_capacity_by_v - vbat_capacity) >
			batt_meter_cust_data.cust_poweron_delta_vbat_tolrance)) {
			bm_print(BM_LOG_CRTI,
				 "[oam_init] fg_vbat=(%d), vbat=(%d), set fg_vat as vat\n",
				 gFG_voltage, g_booting_vbat);

			gFG_voltage = g_booting_vbat;
			gFG_capacity_by_v = vbat_capacity;
		}
	}

	gFG_capacity_by_v_init = gFG_capacity_by_v;

	dod_init();

	gFG_BATT_CAPACITY_aging = fgauge_get_Q_max(force_get_tbat(KAL_FALSE));

	/* oam_v_ocv_1 = gFG_voltage; */
	/* oam_v_ocv_2 = gFG_voltage; */


	oam_v_ocv_init = fgauge_read_v_by_d(gFG_DOD0);
	oam_v_ocv_2 = oam_v_ocv_1 = oam_v_ocv_init;
	g_vol_bat_hw_ocv = gFG_voltage;

	/* vbat = 5; //set avg times */
	/* ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_BAT_SENSE, &vbat); */
	/* oam_r_1 = fgauge_read_r_bat_by_v(vbat); */
	oam_r_1 = fgauge_read_r_bat_by_v(gFG_voltage);
	oam_r_2 = oam_r_1;

	oam_d0 = gFG_DOD0;
	oam_d_5 = oam_d0;
	oam_i_ori = gFG_current;
	g_d_hw_ocv = oam_d0;

	if (oam_init_i == 0) {
		bm_print(BM_LOG_CRTI,
			 "[oam_init] oam_v_ocv_1,oam_v_ocv_2,oam_r_1,oam_r_2,oam_d0,oam_i_ori\n");
		oam_init_i = 1;
	}

	bm_print(BM_LOG_CRTI, "[oam_init] %d,%d,%d,%d,%d,%d\n",
		 oam_v_ocv_1, oam_v_ocv_2, oam_r_1, oam_r_2, oam_d0, oam_i_ori);

	bm_print(BM_LOG_CRTI, "[oam_init_inf] hw_OCV, hw_D0, RTC, D0, oam_OCV_init, tbat\n");
	bm_print(BM_LOG_CRTI,
		 "[oam_run_inf] oam_OCV1, oam_OCV2, vbat, I1, I2, R1, R2, Car1, Car2,qmax, tbat\n");
	bm_print(BM_LOG_CRTI, "[oam_result_inf] D1, D2, D3, D4, D5, UI_SOC\n");


	bm_print(BM_LOG_CRTI, "[oam_init_inf] %d, %d, %d, %d, %d, %d\n",
		 gFG_voltage, (100 - fgauge_read_capacity_by_v(gFG_voltage)), g_rtc_fg_soc,
		 gFG_DOD0, oam_v_ocv_init, force_get_tbat(KAL_FALSE));

}


void oam_run(void)
{
	int vol_bat = 0;
	/* int vol_bat_hw_ocv=0; */
	/* int d_hw_ocv=0; */
	int charging_current = 0;
	int ret = 0;
	/* kal_uint32 now_time; */
	struct timespec now_time;
	kal_int32 delta_time = 0;

	/* now_time = rtc_read_hw_time(); */
	#if !defined(FG_SELF_DISCHARGE_CHECK)
	get_monotonic_boottime(&now_time);
	#else
	getrawmonotonic(&now_time);
	#endif

	/* delta_time = now_time - last_oam_run_time; */
	delta_time = now_time.tv_sec - last_oam_run_time.tv_sec;

	bm_print(BM_LOG_CRTI, "[oam_run_time] delta time=%d\n", delta_time);

#if defined(SW_OAM_INIT_V2)
	printk(bootbuf);
#endif

	last_oam_run_time = now_time;

	/* Reconstruct table if temp changed; */
	fgauge_construct_table_by_temp();

#if defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
	fgauge_get_current_factor();
#endif

	vol_bat = 15;		/* set avg times */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_BAT_SENSE, &vol_bat);

	/* ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_OCV, &vol_bat_hw_ocv); */
	/* d_hw_ocv = fgauge_read_d_by_v(vol_bat_hw_ocv); */

	oam_i_1 = (((oam_v_ocv_1 - vol_bat) * 1000) * 10) / oam_r_1;	/* 0.1mA */
	oam_i_2 = (((oam_v_ocv_2 - vol_bat) * 1000) * 10) / oam_r_2;	/* 0.1mA */

	oam_car_1 = (oam_i_1 * delta_time / 3600) + oam_car_1;	/* 0.1mAh */
	oam_car_2 = (oam_i_2 * delta_time / 3600) + oam_car_2;	/* 0.1mAh */

	oam_d_1 = oam_d0 + (oam_car_1 * 100 / 10) / gFG_BATT_CAPACITY_aging;
	if (oam_d_1 < 0)
		oam_d_1 = 0;
	if (oam_d_1 > 100)
		oam_d_1 = 100;

	oam_d_2 = oam_d0 + (oam_car_2 * 100 / 10) / gFG_BATT_CAPACITY_aging;
	if (oam_d_2 < 0)
		oam_d_2 = 0;
	if (oam_d_2 > 100)
		oam_d_2 = 100;

	oam_v_ocv_1 = vol_bat + mtk_imp_tracking(vol_bat, oam_i_2, 5);

	oam_d_3 = fgauge_read_d_by_v(oam_v_ocv_1);
	if (oam_d_3 < 0)
		oam_d_3 = 0;
	if (oam_d_3 > 100)
		oam_d_3 = 100;

	oam_r_1 = fgauge_read_r_bat_by_v(oam_v_ocv_1);

	oam_v_ocv_2 = fgauge_read_v_by_d(oam_d_2);
	oam_r_2 = fgauge_read_r_bat_by_v(oam_v_ocv_2);

#if 0
	oam_d_4 = (oam_d_2 + oam_d_3) / 2;
#else
	oam_d_4 = oam_d_3;
#endif

	gFG_columb = oam_car_2 / 10;	/* mAh */

	if ((oam_i_1 < 0) || (oam_i_2 < 0))
		gFG_Is_Charging = KAL_TRUE;
	else
		gFG_Is_Charging = KAL_FALSE;

#if 0
	if (gFG_Is_Charging == KAL_FALSE) {
		d5_count_time = 60;
	} else {
		charging_current = get_charging_setting_current();
		charging_current = charging_current / 100;
		d5_count_time_rate =
		    (((gFG_BATT_CAPACITY_aging * 60 * 60 / 100 / (charging_current - 50)) * 10) +
		     5) / 10;

		if (d5_count_time_rate < 1)
			d5_count_time_rate = 1;

		d5_count_time = d5_count_time_rate;
	}
#else
	d5_count_time = 60;
#endif
	d5_count = d5_count + delta_time;
	if (d5_count >= d5_count_time) {
		if (gFG_Is_Charging == KAL_FALSE) {
			if (oam_d_3 > oam_d_5)
				oam_d_5 = oam_d_5 + 1;
			else
				if (oam_d_4 > oam_d_5)
					oam_d_5 = oam_d_5 + 1;


		} else {
			if (oam_d_5 > oam_d_3)
				oam_d_5 = oam_d_5 - 1;
			else
				if (oam_d_4 < oam_d_5)
					oam_d_5 = oam_d_5 - 1;


		}
		d5_count = 0;
		oam_d_3_pre = oam_d_3;
		oam_d_4_pre = oam_d_4;
	}

	bm_print(BM_LOG_CRTI, "[oam_run] %d,%d,%d,%d,%d,%d,%d,%d\n",
		 d5_count, d5_count_time, oam_d_3_pre, oam_d_3, oam_d_4_pre, oam_d_4, oam_d_5,
		 charging_current);

	if (oam_run_i == 0) {
		bm_print(BM_LOG_FULL,
			 "[oam_run] oam_i_1,oam_i_2,oam_car_1,oam_car_2,oam_d_1,oam_d_2,oam_v_ocv_1,oam_d_3,oam_r_1,oam_v_ocv_2,oam_r_2,vol_bat,g_vol_bat_hw_ocv,g_d_hw_ocv\n");
		oam_run_i = 1;
	}

	bm_print(BM_LOG_FULL, "[oam_run] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		 oam_i_1, oam_i_2, oam_car_1, oam_car_2, oam_d_1, oam_d_2, oam_v_ocv_1, oam_d_3,
		 oam_r_1, oam_v_ocv_2, oam_r_2, vol_bat, g_vol_bat_hw_ocv, g_d_hw_ocv);

	bm_print(BM_LOG_FULL, "[oam_total] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		 gFG_capacity_by_c, gFG_capacity_by_v, gfg_percent_check_point,
		 oam_d_1, oam_d_2, oam_d_3, oam_d_4, oam_d_5, gFG_capacity_by_c_init, g_d_hw_ocv);

	bm_print(BM_LOG_CRTI, "[oam_total_s] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", gFG_capacity_by_c,	/* 1 */
		 gFG_capacity_by_v,	/* 2 */
		 gfg_percent_check_point,	/* 3 */
		 (100 - oam_d_1),	/* 4 */
		 (100 - oam_d_2),	/* 5 */
		 (100 - oam_d_3),	/* 6 */
		 (100 - oam_d_4),	/* 9 */
		 (100 - oam_d_5),	/* 10 */
		 gFG_capacity_by_c_init,	/* 7 */
		 (100 - g_d_hw_ocv)	/* 8 */
	    );

	bm_print(BM_LOG_FULL, "[oam_total_s_err] %d,%d,%d,%d,%d,%d,%d\n",
		 (gFG_capacity_by_c - gFG_capacity_by_v),
		 (gFG_capacity_by_c - gfg_percent_check_point),
		 (gFG_capacity_by_c - (100 - oam_d_1)),
		 (gFG_capacity_by_c - (100 - oam_d_2)),
		 (gFG_capacity_by_c - (100 - oam_d_3)),
		 (gFG_capacity_by_c - (100 - oam_d_4)), (gFG_capacity_by_c - (100 - oam_d_5))
	    );

	bm_print(BM_LOG_CRTI, "[oam_init_inf] %d, %d, %d, %d, %d, %d\n",
		 gFG_voltage, (100 - fgauge_read_capacity_by_v(gFG_voltage)), g_rtc_fg_soc,
		 gFG_DOD0, oam_v_ocv_init, force_get_tbat(KAL_FALSE));

	bm_print(BM_LOG_CRTI, "[oam_run_inf] %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
		 oam_v_ocv_1, oam_v_ocv_2, vol_bat, oam_i_1, oam_i_2, oam_r_1, oam_r_2, oam_car_1,
		 oam_car_2, gFG_BATT_CAPACITY_aging, force_get_tbat(KAL_FALSE), oam_d0);

	bm_print(BM_LOG_CRTI, "[oam_result_inf] %d, %d, %d, %d, %d, %d\n",
		 oam_d_1, oam_d_2, oam_d_3, oam_d_4, oam_d_5, BMT_status.UI_SOC);
}

/* ============================================================ // */



void table_init(void)
{
	BATTERY_PROFILE_STRUC_P profile_p;
	R_PROFILE_STRUC_P profile_p_r_table;

	int temperature = force_get_tbat(KAL_FALSE);

	/* Re-constructure r-table profile according to current temperature */
	profile_p_r_table = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t);
	if (profile_p_r_table == NULL) {
		bm_print(BM_LOG_CRTI,
			 "[FGADC] fgauge_get_profile_r_table : create table fail !\r\n");
	}
	fgauge_construct_r_table_profile(temperature, profile_p_r_table);

	/* Re-constructure battery profile according to current temperature */
	profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t);
	if (profile_p == NULL)
		bm_print(BM_LOG_CRTI, "[FGADC] fgauge_get_profile : create table fail !\r\n");

	fgauge_construct_battery_profile(temperature, profile_p);
}

kal_int32 auxadc_algo_run(void)
{
	kal_int32 val = 0;

	gFG_voltage = battery_meter_get_battery_voltage(KAL_FALSE);
	val = fgauge_read_capacity_by_v(gFG_voltage);

	bm_print(BM_LOG_CRTI, "[auxadc_algo_run] %d,%d\n", gFG_voltage, val);

	return val;
}

#if defined(SOC_BY_HW_FG)
void update_fg_dbg_tool_value(void)
{
	g_fg_dbg_bat_volt = gFG_voltage_init;

	if (gFG_Is_Charging == KAL_TRUE)
		g_fg_dbg_bat_current = 1 - gFG_current - 1;
	else
		g_fg_dbg_bat_current = gFG_current;

	g_fg_dbg_bat_zcv = gFG_voltage;

	g_fg_dbg_bat_temp = gFG_temp;

	g_fg_dbg_bat_r = gFG_resistance_bat;

	g_fg_dbg_bat_car = gFG_columb;

	g_fg_dbg_bat_qmax = gFG_BATT_CAPACITY_aging;

	g_fg_dbg_d0 = gFG_DOD0;

	g_fg_dbg_d1 = gFG_DOD1;

	g_fg_dbg_percentage = bat_get_ui_percentage();

	g_fg_dbg_percentage_fg = gFG_capacity_by_c;

	g_fg_dbg_percentage_voltmode = gfg_percent_check_point;
}

kal_int32 fgauge_compensate_battery_voltage(kal_int32 ori_voltage)
{
	kal_int32 ret_compensate_value = 0;

	gFG_ori_voltage = ori_voltage;
	gFG_resistance_bat = fgauge_read_r_bat_by_v(ori_voltage);	/* Ohm */
	ret_compensate_value =
	    (gFG_current * (gFG_resistance_bat + batt_meter_cust_data.r_fg_value)) / 1000;
	ret_compensate_value = (ret_compensate_value + (10 / 2)) / 10;

	if (gFG_Is_Charging == KAL_TRUE)
		ret_compensate_value = ret_compensate_value - (ret_compensate_value * 2);


	gFG_compensate_value = ret_compensate_value;

	bm_print(BM_LOG_FULL,
		 "[CompensateVoltage] Ori_voltage:%d, compensate_value:%d, gFG_resistance_bat:%d, gFG_current:%d\r\n",
		 ori_voltage, ret_compensate_value, gFG_resistance_bat, gFG_current);

	return ret_compensate_value;
}

kal_int32 fgauge_compensate_battery_voltage_recursion(kal_int32 ori_voltage,
						      kal_int32 recursion_time)
{
	kal_int32 ret_compensate_value = 0;
	kal_int32 temp_voltage_1 = ori_voltage;
	kal_int32 temp_voltage_2 = temp_voltage_1;
	int i = 0;

	for (i = 0; i < recursion_time; i++) {
		gFG_resistance_bat = fgauge_read_r_bat_by_v(temp_voltage_2);	/* Ohm */
		ret_compensate_value =
		    (gFG_current * (gFG_resistance_bat + batt_meter_cust_data.r_fg_value)) / 1000;
		ret_compensate_value = (ret_compensate_value + (10 / 2)) / 10;

		if (gFG_Is_Charging == KAL_TRUE)
			ret_compensate_value = ret_compensate_value - (ret_compensate_value * 2);

		temp_voltage_2 = temp_voltage_1 + ret_compensate_value;

		bm_print(BM_LOG_FULL,
			 "[fgauge_compensate_battery_voltage_recursion] %d,%d,%d,%d\r\n",
			 temp_voltage_1, temp_voltage_2, gFG_resistance_bat, ret_compensate_value);
	}

	gFG_resistance_bat = fgauge_read_r_bat_by_v(temp_voltage_2);	/* Ohm */
	ret_compensate_value =
	    (gFG_current *
	     (gFG_resistance_bat + batt_meter_cust_data.r_fg_value +
	      batt_meter_cust_data.fg_meter_resistance)) / 1000;
	ret_compensate_value = (ret_compensate_value + (10 / 2)) / 10;

	if (gFG_Is_Charging == KAL_TRUE)
		ret_compensate_value = ret_compensate_value - (ret_compensate_value * 2);


	gFG_compensate_value = ret_compensate_value;

	bm_print(BM_LOG_FULL, "[fgauge_compensate_battery_voltage_recursion] %d,%d,%d,%d\r\n",
		 temp_voltage_1, temp_voltage_2, gFG_resistance_bat, ret_compensate_value);

	return ret_compensate_value;
}


kal_int32 fgauge_get_dod0(kal_int32 voltage, kal_int32 temperature, kal_bool bOcv)
{
	kal_int32 dod0 = 0;
	int i = 0, saddles = 0, jj = 0;
	BATTERY_PROFILE_STRUC_P profile_p;
	R_PROFILE_STRUC_P profile_p_r_table;
	int ret = 0;

/* R-Table (First Time) */
	/* Re-constructure r-table profile according to current temperature */
	profile_p_r_table = fgauge_get_profile_r_table(batt_meter_cust_data.temperature_t);
	if (profile_p_r_table == NULL) {
		bm_print(BM_LOG_CRTI,
			 "[FGADC] fgauge_get_profile_r_table : create table fail !\r\n");
	}
	fgauge_construct_r_table_profile(temperature, profile_p_r_table);

	/* Re-constructure battery profile according to current temperature */
	profile_p = fgauge_get_profile(batt_meter_cust_data.temperature_t);
	if (profile_p == NULL) {
		bm_print(BM_LOG_CRTI, "[FGADC] fgauge_get_profile : create table fail !\r\n");
		return 100;
	}
	fgauge_construct_battery_profile(temperature, profile_p);

	/* Get total saddle points from the battery profile */
	saddles = fgauge_get_saddles();

	/* If the input voltage is not OCV, compensate to ZCV due to battery loading */
	/* Compasate battery voltage from current battery voltage */
	jj = 0;
	if (bOcv == KAL_FALSE) {
		while (gFG_current == 0) {
			ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &gFG_current);
			if (jj > 10)
				break;
			jj++;
		}
		/* voltage = voltage + fgauge_compensate_battery_voltage(voltage); //mV */
		voltage = voltage + fgauge_compensate_battery_voltage_recursion(voltage, 5);	/* mV */
		bm_print(BM_LOG_CRTI, "[FGADC] compensate_battery_voltage, voltage=%d\r\n",
			 voltage);
	}
	/* If battery voltage is less then mimimum profile voltage, then return 100 */
	/* If battery voltage is greater then maximum profile voltage, then return 0 */
	if (voltage > (profile_p + 0)->voltage)
		return 0;

	if (voltage < (profile_p + saddles - 1)->voltage)
		return 100;

	/* get DOD0 according to current temperature */
	for (i = 0; i < saddles - 1; i++) {
		if ((voltage <= (profile_p + i)->voltage)
		    && (voltage >= (profile_p + i + 1)->voltage)) {
			dod0 =
			    (profile_p + i)->percentage +
			    (((((profile_p + i)->voltage) -
			       voltage) * (((profile_p + i + 1)->percentage) -
					   ((profile_p + i)->percentage))
			     ) / (((profile_p + i)->voltage) - ((profile_p + i + 1)->voltage))
			    );

			break;
		}
	}

	return dod0;
}


kal_int32 fgauge_update_dod(void)
{
	kal_int32 FG_dod_1 = 0;
	int adjust_coulomb_counter = batt_meter_cust_data.car_tune_value;
#ifdef Q_MAX_BY_CURRENT
	kal_int32 C_0mA = 0;
	kal_int32 C_400mA = 0;
	kal_int32 C_FGCurrent = 0;
#endif

	if (gFG_DOD0 > 100) {
		gFG_DOD0 = 100;
		bm_print(BM_LOG_FULL, "[fgauge_update_dod] gFG_DOD0 set to 100, gFG_columb=%d\r\n",
			 gFG_columb);
	} else if (gFG_DOD0 < 0) {
		gFG_DOD0 = 0;
		bm_print(BM_LOG_FULL, "[fgauge_update_dod] gFG_DOD0 set to 0, gFG_columb=%d\r\n",
			 gFG_columb);
	} else {
	}

	gFG_temp = force_get_tbat(KAL_FALSE);

	if (temperature_change == 1) {
		gFG_BATT_CAPACITY = fgauge_get_Q_max(gFG_temp);
		bm_print(BM_LOG_CRTI,
			 "[fgauge_update_dod] gFG_BATT_CAPACITY=%d, gFG_BATT_CAPACITY_aging=%d, gFG_BATT_CAPACITY_init_high_current=%d\r\n",
			 gFG_BATT_CAPACITY, gFG_BATT_CAPACITY_aging,
			 gFG_BATT_CAPACITY_init_high_current);
		temperature_change = 0;
	}
#if 0
	C_0mA = fgauge_get_Q_max(gFG_temp);
	C_400mA = fgauge_get_Q_max_high_current(gFG_temp);
	C_FGCurrent = C_0mA - (C_0mA - C_400mA) * gFG_current_AVG / 4000;
	if (C_FGCurrent != 0)
		FG_dod_1 =
		    gFG_DOD0 - ((gFG_columb * 100) / gFG_BATT_CAPACITY_aging) * C_0mA / C_FGCurrent;

	bm_print(BM_LOG_CRTI,
		 "[fgauge_update_dod] FG_dod_1=%d, adjust_coulomb_counter=%d, gFG_columb=%d, gFG_DOD0=%d, gFG_temp=%d, gFG_BATT_CAPACITY=%d, C_0mA=%d, C_400mA=%d, C_FGCurrent=%d, gFG_current_AVG=%d\n",
		 FG_dod_1, adjust_coulomb_counter, gFG_columb, gFG_DOD0, gFG_temp,
		 gFG_BATT_CAPACITY, C_0mA, C_400mA, C_FGCurrent, gFG_current_AVG);
#else
	FG_dod_1 = gFG_DOD0 - ((gFG_columb * 100) / gFG_BATT_CAPACITY_aging);

	bm_print(BM_LOG_FULL,
		 "[fgauge_update_dod] FG_dod_1=%d, adjust_coulomb_counter=%d, gFG_columb=%d, gFG_DOD0=%d, gFG_temp=%d, gFG_BATT_CAPACITY=%d  %d\r\n",
		 FG_dod_1, adjust_coulomb_counter, gFG_columb, gFG_DOD0, gFG_temp,
		 gFG_BATT_CAPACITY, gFG_BATT_CAPACITY_aging);
#endif
	if (FG_dod_1 > 100) {
		FG_dod_1 = 100;
		bm_print(BM_LOG_FULL, "[fgauge_update_dod] FG_dod_1 set to 100, gFG_columb=%d\r\n",
			 gFG_columb);
	} else if (FG_dod_1 < 0) {
		FG_dod_1 = 0;
		bm_print(BM_LOG_FULL, "[fgauge_update_dod] FG_dod_1 set to 0, gFG_columb=%d\r\n",
			 gFG_columb);
	} else {
	}

	return FG_dod_1;
}


kal_int32 fgauge_read_capacity(kal_int32 type)
{
	kal_int32 voltage;
	kal_int32 temperature;
	kal_int32 dvalue = 0;
	kal_int32 temp_val = 0;

	if (type == 0) {	/* for initialization */
		/* Use voltage to calculate capacity */
		voltage = battery_meter_get_battery_voltage(KAL_TRUE);	/* in unit of mV */
		temperature = force_get_tbat(KAL_FALSE);
		dvalue = fgauge_get_dod0(voltage, temperature, KAL_FALSE);	/* need compensate vbat */
	} else {
		/* Use DOD0 and columb counter to calculate capacity */
		dvalue = fgauge_update_dod();	/* DOD1 = DOD0 + (-CAR)/Qmax */
	}

	gFG_DOD1 = dvalue;

	temp_val = dvalue;
	dvalue = 100 - temp_val;

	if (dvalue <= 1) {
		dvalue = 1;
		bm_print(BM_LOG_FULL, "[fgauge_read_capacity] dvalue<=1 and set dvalue=1 !!\r\n");
	}

	return dvalue;
}


void fg_voltage_mode(void)
{
#if defined(CONFIG_POWER_EXT)
#else
	if (bat_is_charger_exist() == KAL_TRUE) {
		/* SOC only UP when charging */
		if (gFG_capacity_by_v > gfg_percent_check_point)
			gfg_percent_check_point++;

	} else {
		/* SOC only Done when dis-charging */
		if (gFG_capacity_by_v < gfg_percent_check_point)
			gfg_percent_check_point--;

	}

	bm_print(BM_LOG_FULL,
		 "[FGADC_VoltageMothod] gFG_capacity_by_v=%d,gfg_percent_check_point=%d\r\n",
		 gFG_capacity_by_v, gfg_percent_check_point);
#endif
}


void fgauge_algo_run(void)
{
	int i = 0;
	int ret = 0;
#ifdef MTK_BATTERY_LIFETIME_DATA_SUPPORT
	int columb_delta = 0;
	int charge_current = 0;
#endif

	/* Reconstruct table if temp changed; */
	fgauge_construct_table_by_temp();

/* 1. Get Raw Data */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &gFG_current);
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT_SIGN, &gFG_Is_Charging);

	gFG_voltage = battery_meter_get_battery_voltage(KAL_FALSE);
	gFG_voltage_init = gFG_voltage;
	gFG_voltage = gFG_voltage + fgauge_compensate_battery_voltage_recursion(gFG_voltage, 5);	/* mV */
	gFG_voltage = gFG_voltage + batt_meter_cust_data.ocv_board_compesate;

#if defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
	fgauge_get_current_factor();
#endif

	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CAR, &gFG_columb);

#ifdef MTK_BATTERY_LIFETIME_DATA_SUPPORT
	if (gFG_Is_Charging) {
		charge_current -= gFG_current;
		if (charge_current < gFG_min_current)
			gFG_min_current = charge_current;
	} else {
		if (gFG_current > gFG_max_current)
			gFG_max_current = gFG_current;
	}

	columb_delta = gFG_pre_columb_count - gFG_columb;

	if (columb_delta < 0)
		columb_delta = columb_delta - 2 * columb_delta;	/* absolute value */

	gFG_pre_columb_count = gFG_columb;
	gFG_columb_sum += columb_delta;

	/* should we use gFG_BATT_CAPACITY or gFG_BATT_CAPACITY_aging ?? */
	if (gFG_columb_sum >= 2 * gFG_BATT_CAPACITY_aging) {
		gFG_battery_cycle++;
		gFG_columb_sum -= 2 * gFG_BATT_CAPACITY_aging;
		bm_print(BM_LOG_CRTI, "Update battery cycle count to %d. \r\n", gFG_battery_cycle);
	}
	bm_print(BM_LOG_FULL, "@@@ bat cycle count %d, columb sum %d. \r\n", gFG_battery_cycle,
		 gFG_columb_sum);
#endif

	/* add by willcai 2014-12-18 begin */
	if (BMT_status.charger_exist == KAL_FALSE) {
		if (gFG_Is_offset_init == KAL_FALSE) {
			for (i = 0; i < batt_meter_cust_data.fg_vbat_average_size; i++)
				FGvbatVoltageBuffer[i] = gFG_voltage;


			FGbatteryVoltageSum =
			    gFG_voltage * batt_meter_cust_data.fg_vbat_average_size;
			gFG_voltage_AVG = gFG_voltage;
			gFG_Is_offset_init = KAL_TRUE;
		}
/* 1.1 Average FG_voltage */
    /**************** Averaging : START ****************/
		if (gFG_voltage >= gFG_voltage_AVG)
			gFG_vbat_offset = (gFG_voltage - gFG_voltage_AVG);
		else
			gFG_vbat_offset = (gFG_voltage_AVG - gFG_voltage);


		if (gFG_vbat_offset <= batt_meter_cust_data.minerroroffset) {
			FGbatteryVoltageSum -= FGvbatVoltageBuffer[FGbatteryIndex];
			FGbatteryVoltageSum += gFG_voltage;
			FGvbatVoltageBuffer[FGbatteryIndex] = gFG_voltage;

			gFG_voltage_AVG =
			    FGbatteryVoltageSum / batt_meter_cust_data.fg_vbat_average_size;
			gFG_voltage = gFG_voltage_AVG;

			FGbatteryIndex++;
			if (FGbatteryIndex >= batt_meter_cust_data.fg_vbat_average_size)
				FGbatteryIndex = 0;

			bm_print(BM_LOG_FULL, "[FG_BUFFER] ");
			for (i = 0; i < batt_meter_cust_data.fg_vbat_average_size; i++)
				bm_print(BM_LOG_FULL, "%d,", FGvbatVoltageBuffer[i]);

			bm_print(BM_LOG_FULL, "\r\n");
		} else {
			bm_print(BM_LOG_FULL, "[FG] Over MinErrorOffset:V=%d,Avg_V=%d, ",
				 gFG_voltage, gFG_voltage_AVG);

			gFG_voltage = gFG_voltage_AVG;

			bm_print(BM_LOG_FULL, "Avg_V need write back to V : V=%d,Avg_V=%d.\r\n",
				 gFG_voltage, gFG_voltage_AVG);
		}
	} else
		gFG_Is_offset_init = KAL_FALSE;

#ifdef Q_MAX_BY_CURRENT
/* 1.2 Average FG_current */
    /**************** Averaging : START ****************/
	if (gFG_current_AVG == 0) {
		for (i = 0; i < FG_CURRENT_AVERAGE_SIZE; i++)
			FGCurrentBuffer[i] = gFG_current;


		FGCurrentSum = gFG_current * FG_CURRENT_AVERAGE_SIZE;
		gFG_current_AVG = gFG_current;
	} else {
		FGCurrentSum -= FGCurrentBuffer[FGCurrentIndex];
		FGCurrentSum += gFG_current;
		FGCurrentBuffer[FGCurrentIndex] = gFG_current;

		gFG_current_AVG = FGCurrentSum / FG_CURRENT_AVERAGE_SIZE;

		FGCurrentIndex++;
		if (FGCurrentIndex >= FG_CURRENT_AVERAGE_SIZE)
			FGCurrentIndex = 0;

		bm_print(BM_LOG_FULL, "[FG_BUFFER] ");
		for (i = 0; i < FG_CURRENT_AVERAGE_SIZE; i++)
			bm_print(BM_LOG_FULL, "%d,", FGCurrentBuffer[i]);

		bm_print(BM_LOG_FULL, "\n");
	}
#endif
/* 2. Calculate battery capacity by VBAT */
	gFG_capacity_by_v = fgauge_read_capacity_by_v(gFG_voltage);

/* 3. Calculate battery capacity by Coulomb Counter */
	gFG_capacity_by_c = fgauge_read_capacity(1);

/* 4. voltage mode */
	if (volt_mode_update_timer >= volt_mode_update_time_out) {
		volt_mode_update_timer = 0;

		fg_voltage_mode();
	} else {
		volt_mode_update_timer++;
	}

/* 5. Logging */
	bm_print(BM_LOG_CRTI,
		 "[FGADC] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
		 gFG_Is_Charging, gFG_current, gFG_columb, gFG_voltage, gFG_capacity_by_v,
		 gFG_capacity_by_c, gFG_capacity_by_c_init, gFG_BATT_CAPACITY,
		 gFG_BATT_CAPACITY_aging, gFG_compensate_value, gFG_ori_voltage,
		 batt_meter_cust_data.ocv_board_compesate, batt_meter_cust_data.r_fg_board_slope,
		 gFG_voltage_init, batt_meter_cust_data.minerroroffset, gFG_DOD0, gFG_DOD1,
		 batt_meter_cust_data.car_tune_value, batt_meter_cust_data.aging_tuning_value);
	update_fg_dbg_tool_value();
}

void fgauge_algo_run_init(void)
{
	int i = 0;
	int ret = 0;

#ifdef INIT_SOC_BY_SW_SOC
	kal_bool charging_enable = KAL_FALSE;
#if defined(CONFIG_MTK_KERNEL_POWER_OFF_CHARGING) && !defined(SWCHR_POWER_PATH)
	if (LOW_POWER_OFF_CHARGING_BOOT != g_boot_mode)
#endif
		/*stop charging for vbat measurement */
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	msleep(50);
#endif
/* 1. Get Raw Data */
	gFG_voltage = battery_meter_get_battery_voltage(KAL_TRUE);
	gFG_voltage_init = gFG_voltage;
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &gFG_current);
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT_SIGN, &gFG_Is_Charging);

	gFG_voltage = gFG_voltage + fgauge_compensate_battery_voltage_recursion(gFG_voltage, 5);	/* mV */
	gFG_voltage = gFG_voltage + batt_meter_cust_data.ocv_board_compesate;

	bm_print(BM_LOG_CRTI, "[FGADC] SWOCV : %d,%d,%d,%d,%d,%d\n",
		 gFG_voltage_init, gFG_voltage, gFG_current, gFG_Is_Charging, gFG_resistance_bat,
		 gFG_compensate_value);
#ifdef INIT_SOC_BY_SW_SOC
	charging_enable = KAL_TRUE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
#endif
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CAR, &gFG_columb);

/* 1.1 Average FG_voltage */
	for (i = 0; i < batt_meter_cust_data.fg_vbat_average_size; i++)
		FGvbatVoltageBuffer[i] = gFG_voltage;


	FGbatteryVoltageSum = gFG_voltage * batt_meter_cust_data.fg_vbat_average_size;
	gFG_voltage_AVG = gFG_voltage;

#ifdef Q_MAX_BY_CURRENT
/* 1.2 Average FG_current */
	for (i = 0; i < FG_CURRENT_AVERAGE_SIZE; i++)
		FGCurrentBuffer[i] = gFG_current;


	FGCurrentSum = gFG_current * FG_CURRENT_AVERAGE_SIZE;
	gFG_current_AVG = gFG_current;
#endif

/* 2. Calculate battery capacity by VBAT */
	gFG_capacity_by_v = fgauge_read_capacity_by_v(gFG_voltage);
	gFG_capacity_by_v_init = gFG_capacity_by_v;

/* 3. Calculate battery capacity by Coulomb Counter */
	gFG_capacity_by_c = fgauge_read_capacity(1);

/* 4. update DOD0 */

	dod_init();

	gFG_current_auto_detect_R_fg_count = 0;

	for (i = 0; i < 10; i++) {
		ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &gFG_current);

		gFG_current_auto_detect_R_fg_total += gFG_current;
		gFG_current_auto_detect_R_fg_count++;
	}

	/* double check */
	if (gFG_current_auto_detect_R_fg_total <= 0) {
		bm_print(BM_LOG_CRTI, "gFG_current_auto_detect_R_fg_total=0, need double check\n");

		gFG_current_auto_detect_R_fg_count = 0;

		for (i = 0; i < 10; i++) {
			ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &gFG_current);

			gFG_current_auto_detect_R_fg_total += gFG_current;
			gFG_current_auto_detect_R_fg_count++;
		}
	}

	gFG_current_auto_detect_R_fg_result =
	    gFG_current_auto_detect_R_fg_total / gFG_current_auto_detect_R_fg_count;
#if !defined(DISABLE_RFG_EXIST_CHECK)
	if (gFG_current_auto_detect_R_fg_result <= batt_meter_cust_data.current_detect_r_fg) {
		g_auxadc_solution = 1;

		bm_print(BM_LOG_CRTI,
			 "[FGADC] Detect NO Rfg, use AUXADC report. (%d=%d/%d)(%d)\r\n",
			 gFG_current_auto_detect_R_fg_result, gFG_current_auto_detect_R_fg_total,
			 gFG_current_auto_detect_R_fg_count, g_auxadc_solution);
	} else {
		if (g_auxadc_solution == 0) {
			g_auxadc_solution = 0;

			bm_print(BM_LOG_CRTI,
				 "[FGADC] Detect Rfg, use FG report. (%d=%d/%d)(%d)\r\n",
				 gFG_current_auto_detect_R_fg_result,
				 gFG_current_auto_detect_R_fg_total,
				 gFG_current_auto_detect_R_fg_count, g_auxadc_solution);
		} else {
			bm_print(BM_LOG_CRTI,
				 "[FGADC] Detect Rfg, but use AUXADC report. due to g_auxadc_solution=%d \r\n",
				 g_auxadc_solution);
		}
	}
#endif
/* 5. Logging */
	bm_print(BM_LOG_CRTI,
		 "[FGADC] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
		 gFG_Is_Charging, gFG_current, gFG_columb, gFG_voltage, gFG_capacity_by_v,
		 gFG_capacity_by_c, gFG_capacity_by_c_init, gFG_BATT_CAPACITY,
		 gFG_BATT_CAPACITY_aging, gFG_compensate_value, gFG_ori_voltage,
		 batt_meter_cust_data.ocv_board_compesate, batt_meter_cust_data.r_fg_board_slope,
		 gFG_voltage_init, batt_meter_cust_data.minerroroffset, gFG_DOD0, gFG_DOD1,
		 batt_meter_cust_data.car_tune_value, batt_meter_cust_data.aging_tuning_value);
	update_fg_dbg_tool_value();
}


#ifdef FG_BAT_INT
kal_uint8 reset_fg_bat_int = KAL_TRUE;
void fg_bat_int_handler(void)
{
	reset_fg_bat_int = KAL_TRUE;
	wake_up_bat2();
}
#endif

void fgauge_initialization(void)
{
#if defined(CONFIG_POWER_EXT)
#else
	int i = 0;
	kal_uint32 ret = 0;

	/* gFG_BATT_CAPACITY_init_high_current = fgauge_get_Q_max_high_current(25); */
	/* gFG_BATT_CAPACITY_aging = fgauge_get_Q_max(25); */

	/* 1. HW initialization */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_HW_FG_INIT, NULL);

	/* 2. SW algorithm initialization */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_OCV, &gFG_voltage);

	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &gFG_current);
	i = 0;
	while (gFG_current == 0) {
		ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &gFG_current);
		if (i > 10) {
			bm_print(BM_LOG_CRTI, "[fgauge_initialization] gFG_current == 0\n");
			break;
		}
		i++;
	}

	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CAR, &gFG_columb);
#if !defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
	fgauge_construct_battery_profile_init();
#endif
	gFG_temp = force_get_tbat(KAL_FALSE);
	gFG_capacity = fgauge_read_capacity(0);

	gFG_capacity_by_c_init = gFG_capacity;
	gFG_capacity_by_c = gFG_capacity;
	gFG_capacity_by_v = gFG_capacity;

	gFG_DOD0 = 100 - gFG_capacity;
	bm_print(BM_LOG_CRTI, "[fgauge_initialization] gFG_DOD0 =%d %d\n", gFG_DOD0, gFG_capacity);

	gFG_BATT_CAPACITY = fgauge_get_Q_max(gFG_temp);

	gFG_BATT_CAPACITY_init_high_current = fgauge_get_Q_max_high_current(gFG_temp);
	gFG_BATT_CAPACITY_aging = fgauge_get_Q_max(gFG_temp);

	ret = battery_meter_ctrl(BATTERY_METER_CMD_DUMP_REGISTER, NULL);

	bm_print(BM_LOG_CRTI,
		 "[fgauge_initialization] Done HW_OCV:%d FG_Current:%d FG_CAR:%d tmp=%d capacity=%d Qmax=%d\n",
		 gFG_voltage, gFG_current, gFG_columb, gFG_temp, gFG_capacity, gFG_BATT_CAPACITY);

#if defined(FG_BAT_INT)
	pmic_register_interrupt_callback(41, fg_bat_int_handler);
	pmic_register_interrupt_callback(40, fg_bat_int_handler);
#endif
#endif
}
#endif

kal_int32 get_dynamic_period(int first_use, int first_wakeup_time, int battery_capacity_level)
{
#if defined(CONFIG_POWER_EXT)

	return first_wakeup_time;

#elif defined(SOC_BY_AUXADC) ||  defined(SOC_BY_SW_FG)

	kal_int32 vbat_val = 0;

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return batt_meter_cust_data.normal_wakeup_period;
#endif

	vbat_val = g_sw_vbat_temp;

	/* change wake up period when system suspend. */
	if (vbat_val > batt_meter_cust_data.vbat_normal_wakeup)	/* 3.6v */
		g_spm_timer = batt_meter_cust_data.normal_wakeup_period;	/* 90 min */
	else if (vbat_val > batt_meter_cust_data.vbat_low_power_wakeup)	/* 3.5v */
		g_spm_timer = batt_meter_cust_data.low_power_wakeup_period;	/* 5 min */
	else
		g_spm_timer = batt_meter_cust_data.close_poweroff_wakeup_period;	/* 0.5 min */



	bm_print(BM_LOG_CRTI, "vbat_val=%d, g_spm_timer=%d\n", vbat_val, g_spm_timer);

	return g_spm_timer;
#else

	kal_int32 car_instant = 0;
	kal_int32 current_instant = 0;
	static kal_int32 car_sleep = 0x12345678;
	kal_int32 car_wakeup = 0;
	static kal_int32 last_time;

	kal_int32 ret_val = -1;
	int check_fglog = 0;
	kal_int32 I_sleep = 0;
	kal_int32 new_time = 0;
	kal_int32 vbat_val = 0;
	int ret = 0;

	check_fglog = Enable_FGADC_LOG;
	/*if (check_fglog == 0)*/
		/* Enable_FGADC_LOG=1; */



	vbat_val = g_sw_vbat_temp;

	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &current_instant);

	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CAR, &car_instant);

	/*if (check_fglog == 0)*/
		/* Enable_FGADC_LOG=0; */

	if (car_instant < 0)
		car_instant = car_instant - (car_instant * 2);


	if (vbat_val > batt_meter_cust_data.vbat_normal_wakeup) {	/* 3.6v */
		car_wakeup = car_instant;

		if (last_time == 0)
			last_time = 1;

		if (car_sleep > car_wakeup || car_sleep == 0x12345678) {
			car_sleep = car_wakeup;
			bm_print(BM_LOG_CRTI, "[get_dynamic_period] reset car_sleep\n");
		}

		I_sleep = ((car_wakeup - car_sleep) * 3600) / last_time;	/* unit: second */

		if (I_sleep == 0) {
			/*if (check_fglog == 0)*/
				/* Enable_FGADC_LOG=1; */


			ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &I_sleep);

			I_sleep = I_sleep / 10;
			/*if (check_fglog == 0)*/
				/* Enable_FGADC_LOG=0; */

		}

		if (I_sleep == 0) {
			new_time = first_wakeup_time;
		} else {
			new_time =
			    ((gFG_BATT_CAPACITY * battery_capacity_level * 3600) / 100) / I_sleep;
		}
		ret_val = new_time;

		if (ret_val == 0)
			ret_val = first_wakeup_time;

		bm_print(BM_LOG_CRTI,
			 "[get_dynamic_period] car_instant=%d, car_wakeup=%d, car_sleep=%d, I_sleep=%d, gFG_BATT_CAPACITY=%d, last_time=%d, new_time=%d\r\n",
			 car_instant, car_wakeup, car_sleep, I_sleep, gFG_BATT_CAPACITY, last_time,
			 new_time);

		/* update parameter */
		car_sleep = car_wakeup;
		last_time = ret_val;
		g_spm_timer = ret_val;
	} else if (vbat_val > batt_meter_cust_data.vbat_low_power_wakeup) {	/* 3.5v */
		g_spm_timer = batt_meter_cust_data.low_power_wakeup_period;	/* 5 min */
	} else {
		g_spm_timer = batt_meter_cust_data.close_poweroff_wakeup_period;	/* 0.5 min */
	}

	bm_print(BM_LOG_CRTI, "vbat_val=%d, g_spm_timer=%d\n", vbat_val, g_spm_timer);
	return g_spm_timer;

#endif
}

/* ============================================================ // */
kal_int32 battery_meter_get_battery_voltage(kal_bool update)
{
	int ret = 0;
	int val = 5;
	static int pre_val = -1;

	if (update == KAL_TRUE || pre_val == -1) {
		val = 5;	/* set avg times */
		ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_BAT_SENSE, &val);
		pre_val = val;
	} else {
		val = pre_val;
	}
	g_sw_vbat_temp = val;

#ifdef MTK_BATTERY_LIFETIME_DATA_SUPPORT
	if (g_sw_vbat_temp > gFG_max_voltage)
		gFG_max_voltage = g_sw_vbat_temp;


	if (g_sw_vbat_temp < gFG_min_voltage)
		gFG_min_voltage = g_sw_vbat_temp;

#endif

	return val;
}

kal_int32 battery_meter_get_charging_current_imm(void)
{
#ifdef AUXADC_SUPPORT_IMM_CURRENT_MODE
	return PMIC_IMM_GetCurrent();
#else
	int ret;
	kal_int32 ADC_I_SENSE = 1;	/* 1 measure time */
	kal_int32 ADC_BAT_SENSE = 1;	/* 1 measure time */
	int ICharging = 0;

	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_BAT_SENSE, &ADC_BAT_SENSE);
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_I_SENSE, &ADC_I_SENSE);

	ICharging =
	    (ADC_I_SENSE - ADC_BAT_SENSE +
	     g_I_SENSE_offset) * 1000 / batt_meter_cust_data.cust_r_sense;
	return ICharging;
#endif
}

kal_int32 battery_meter_get_charging_current(void)
{
#ifdef DISABLE_CHARGING_CURRENT_MEASURE
	return 0;
#elif !defined(EXTERNAL_SWCHR_SUPPORT)
	kal_int32 ADC_BAT_SENSE_tmp[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	kal_int32 ADC_BAT_SENSE_sum = 0;
	kal_int32 ADC_BAT_SENSE = 0;
	kal_int32 ADC_I_SENSE_tmp[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	kal_int32 ADC_I_SENSE_sum = 0;
	kal_int32 ADC_I_SENSE = 0;
	int repeat = 20;
	int i = 0;
	int j = 0;
	kal_int32 temp = 0;
	int ICharging = 0;
	int ret = 0;
	int val = 1;

	for (i = 0; i < repeat; i++) {
		val = 1;	/* set avg times */
		ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_BAT_SENSE, &val);
		ADC_BAT_SENSE_tmp[i] = val;

		val = 1;	/* set avg times */
		ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_I_SENSE, &val);
		ADC_I_SENSE_tmp[i] = val;

		ADC_BAT_SENSE_sum += ADC_BAT_SENSE_tmp[i];
		ADC_I_SENSE_sum += ADC_I_SENSE_tmp[i];
	}

	/* sorting    BAT_SENSE */
	for (i = 0; i < repeat; i++) {
		for (j = i; j < repeat; j++) {
			if (ADC_BAT_SENSE_tmp[j] < ADC_BAT_SENSE_tmp[i]) {
				temp = ADC_BAT_SENSE_tmp[j];
				ADC_BAT_SENSE_tmp[j] = ADC_BAT_SENSE_tmp[i];
				ADC_BAT_SENSE_tmp[i] = temp;
			}
		}
	}

	bm_print(BM_LOG_FULL, "[g_Get_I_Charging:BAT_SENSE]\r\n");
	for (i = 0; i < repeat; i++)
		bm_print(BM_LOG_FULL, "%d,", ADC_BAT_SENSE_tmp[i]);

	bm_print(BM_LOG_FULL, "\r\n");

	/* sorting    I_SENSE */
	for (i = 0; i < repeat; i++) {
		for (j = i; j < repeat; j++) {
			if (ADC_I_SENSE_tmp[j] < ADC_I_SENSE_tmp[i]) {
				temp = ADC_I_SENSE_tmp[j];
				ADC_I_SENSE_tmp[j] = ADC_I_SENSE_tmp[i];
				ADC_I_SENSE_tmp[i] = temp;
			}
		}
	}

	bm_print(BM_LOG_FULL, "[g_Get_I_Charging:I_SENSE]\r\n");
	for (i = 0; i < repeat; i++)
		bm_print(BM_LOG_FULL, "%d,", ADC_I_SENSE_tmp[i]);

	bm_print(BM_LOG_FULL, "\r\n");

	ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[0];
	ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[1];
	ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[18];
	ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[19];
	ADC_BAT_SENSE = ADC_BAT_SENSE_sum / (repeat - 4);

	bm_print(BM_LOG_FULL, "[g_Get_I_Charging] ADC_BAT_SENSE=%d\r\n", ADC_BAT_SENSE);

	ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[0];
	ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[1];
	ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[18];
	ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[19];
	ADC_I_SENSE = ADC_I_SENSE_sum / (repeat - 4);

	bm_print(BM_LOG_FULL, "[g_Get_I_Charging] ADC_I_SENSE(Before)=%d\r\n", ADC_I_SENSE);


	bm_print(BM_LOG_FULL, "[g_Get_I_Charging] ADC_I_SENSE(After)=%d\r\n", ADC_I_SENSE);

	if (ADC_I_SENSE > ADC_BAT_SENSE) {
		ICharging =
		    (ADC_I_SENSE - ADC_BAT_SENSE +
		     g_I_SENSE_offset) * 1000 / batt_meter_cust_data.cust_r_sense;
	} else {
		ICharging = 0;
	}

	return ICharging;
#else
	return 0;
#endif
}

kal_int32 battery_meter_get_battery_current(void)
{
	int ret = 0;
	kal_int32 val = 0;

	if (g_auxadc_solution == 1)
		val = oam_i_2;
	else
		ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &val);

	return val;
}

kal_bool battery_meter_get_battery_current_sign(void)
{
	int ret = 0;
	kal_bool val = 0;

	if (g_auxadc_solution == 1)
		val = 0;	/* discharging */
	else
		ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT_SIGN, &val);

	return val;
}

kal_int32 battery_meter_get_car(void)
{
	int ret = 0;
	kal_int32 val = 0;

	if (g_auxadc_solution == 1)
		val = oam_car_2;
	else
		ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CAR, &val);

	return val;
}

kal_int32 battery_meter_get_battery_temperature(void)
{
#ifdef MTK_BATTERY_LIFETIME_DATA_SUPPORT
	kal_int32 batt_temp = force_get_tbat(KAL_TRUE);

	if (batt_temp > gFG_max_temperature)
		gFG_max_temperature = batt_temp;
	if (batt_temp < gFG_min_temperature)
		gFG_min_temperature = batt_temp;

	return batt_temp;
#else
	return force_get_tbat(KAL_TRUE);
#endif
}

kal_int32 battery_meter_get_charger_voltage(void)
{
	int ret = 0;
	int val = 0;

	val = 5;		/* set avg times */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_CHARGER, &val);

	/* val = (((R_CHARGER_1+R_CHARGER_2)*100*val)/R_CHARGER_2)/100; */
	return val;
}

#if defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
void battery_meter_set_reset_soc(kal_bool bUSE_UI_SOC)
{
	g_USE_UI_SOC = bUSE_UI_SOC;
}

kal_int32 battery_meter_get_battery_soc(void)
{
#if defined(SOC_BY_HW_FG)
		return gFG_capacity_by_c;
#elif defined(SOC_BY_SW_FG)
	#if (OAM_D5 == 1)
		return 100-oam_d_5;
	#else
		return 100-oam_d_2;
	#endif
#else
		return 50;
#endif
}

/* Here we compensate D1 by a factor from Qmax with loading. */
kal_int32 battery_meter_trans_battery_percentage(kal_bool d_val)
{
	kal_int32 d_val_before = 0;
	kal_int32 temp_val = 0;
	kal_int32 C_0mA = 0;
	kal_int32 C_600mA = 0;
	kal_int32 C_current = 0;
	kal_int32 i_avg_current = 0;

	d_val_before = d_val;
	temp_val = battery_meter_get_battery_temperature();
	C_0mA = fgauge_get_Q_max(temp_val);

	/* discharging and current > 600ma */
	i_avg_current = g_currentfactor * CV_CURRENT / 100;
	if (KAL_FALSE == gFG_Is_Charging && g_currentfactor > 100) {
		C_600mA = fgauge_get_Q_max_high_current(temp_val);
		C_current = fgauge_get_Q_max_high_current_by_current(i_avg_current, temp_val);
		if (C_current < C_600mA)
			C_600mA = C_current;
	} else
		C_600mA = fgauge_get_Q_max_high_current(temp_val);

	if (C_0mA > C_600mA)
		d_val = d_val + (((C_0mA - C_600mA) * (d_val)) / C_600mA);

	if (d_val > 100)
		d_val = 100;

	bm_print(BM_LOG_CRTI, "[battery_meter_trans_battery_percentage] %d,%d,%d,%d,%d,%d\r\n",
		 temp_val, C_0mA, C_600mA, d_val_before, d_val, g_currentfactor);

	return d_val;
}
#endif

#if defined(FG_BAT_INT)
kal_int32 battery_meter_set_columb_interrupt(kal_uint32 val)
{
	battery_log(BAT_LOG_FULL, "battery_meter_set_columb_interrupt=%d\n", val);
	battery_meter_ctrl(BATTERY_METER_CMD_SET_COLUMB_INTERRUPT, &val);
}
#endif				/* #if defined(FG_BAT_INT) */

kal_int32 battery_meter_get_battery_percentage(void)
{
#if defined(CONFIG_POWER_EXT)
	return 50;
#else

	if (bat_is_charger_exist() == KAL_FALSE)
		fg_qmax_update_for_aging_flag = 1;

#if defined(SOC_BY_AUXADC)
	return auxadc_algo_run();
#endif

#if defined(SOC_BY_HW_FG)
	if (g_auxadc_solution == 1) {
		return auxadc_algo_run();
	} else {
		fgauge_algo_run();
#if !defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
		return gFG_capacity_by_c;	/* hw fg, //return gfg_percent_check_point; // voltage mode */
#else
		/* We keep gFG_capacity_by_c as capacity before compensation */
		/* Compensated capacity is returned for UI SOC tracking */
		return 100 - battery_meter_trans_battery_percentage(100 - gFG_capacity_by_c);
#endif
	}
#endif

#if defined(SOC_BY_SW_FG)
	oam_run();
#if !defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
#if (OAM_D5 == 1)
	return 100 - oam_d_5;
#else
	return 100 - oam_d_2;
#endif
#else
#if (OAM_D5 == 1)
	return 100 - battery_meter_trans_battery_percentage(oam_d_5);
#else
	return 100 - battery_meter_trans_battery_percentage(oam_d_2);
#endif
#endif
#endif

#endif
}


kal_int32 battery_meter_initial(void)
{
#if defined(CONFIG_POWER_EXT)
	return 0;
#else
	static kal_bool meter_initilized = KAL_FALSE;

	mutex_lock(&FGADC_mutex);
	if (meter_initilized == KAL_FALSE) {
#ifdef MTK_MULTI_BAT_PROFILE_SUPPORT
		fgauge_get_profile_id();
#endif

#if defined(SOC_BY_AUXADC)
		g_auxadc_solution = 1;
		table_init();
		bm_print(BM_LOG_CRTI, "[battery_meter_initial] SOC_BY_AUXADC done\n");
#endif

#if defined(SOC_BY_HW_FG)
		fgauge_initialization();
		fgauge_algo_run_init();
		bm_print(BM_LOG_CRTI, "[battery_meter_initial] SOC_BY_HW_FG done\n");
#endif

#if defined(SOC_BY_SW_FG)
		g_auxadc_solution = 1;
		table_init();
		oam_init();
		bm_print(BM_LOG_CRTI, "[battery_meter_initial] SOC_BY_SW_FG done\n");
#endif

		meter_initilized = KAL_TRUE;
	}
	mutex_unlock(&FGADC_mutex);
	return 0;
#endif
}

void reset_parameter_car(void)
{
#if defined(SOC_BY_HW_FG)
	int ret = 0;
	ret = battery_meter_ctrl(BATTERY_METER_CMD_HW_RESET, NULL);
	gFG_columb = 0;

#ifdef MTK_BATTERY_LIFETIME_DATA_SUPPORT
	gFG_pre_columb_count = 0;
#endif

#ifdef MTK_ENABLE_AGING_ALGORITHM
	aging_ocv_1 = 0;
	aging_ocv_2 = 0;
#ifdef MD_SLEEP_CURRENT_CHECK
	columb_before_sleep = 0x123456;
#endif
#endif

#endif

#if defined(SOC_BY_SW_FG)
	oam_car_1 = 0;
	oam_car_2 = 0;
	gFG_columb = 0;
#endif
}

void reset_parameter_dod_change(void)
{
#if defined(SOC_BY_HW_FG)
	bm_print(BM_LOG_CRTI, "[FGADC] Update DOD0(%d) by %d \r\n", gFG_DOD0, gFG_DOD1);
	gFG_DOD0 = gFG_DOD1;
#endif

#if defined(SOC_BY_SW_FG)
	bm_print(BM_LOG_CRTI, "[FGADC] Update oam_d0(%d) by %d \r\n", oam_d0, oam_d_5);
	oam_d0 = oam_d_5;
	gFG_DOD0 = oam_d0;
	oam_d_1 = oam_d_5;
	oam_d_2 = oam_d_5;
	oam_d_3 = oam_d_5;
	oam_d_4 = oam_d_5;
#endif
}

void reset_parameter_dod_full(kal_uint32 ui_percentage)
{
#if defined(SOC_BY_HW_FG)
	bm_print(BM_LOG_CRTI, "[battery_meter_reset]1 DOD0=%d,DOD1=%d,ui=%d\n", gFG_DOD0, gFG_DOD1,
		 ui_percentage);
	gFG_DOD0 = 100 - ui_percentage;
	gFG_DOD1 = gFG_DOD0;
	bm_print(BM_LOG_CRTI, "[battery_meter_reset]2 DOD0=%d,DOD1=%d,ui=%d\n", gFG_DOD0, gFG_DOD1,
		 ui_percentage);
#endif

#if defined(SOC_BY_SW_FG)
	bm_print(BM_LOG_CRTI, "[battery_meter_reset]1 oam_d0=%d,oam_d_5=%d,ui=%d\n", oam_d0,
		 oam_d_5, ui_percentage);
	oam_d0 = 100 - ui_percentage;
	gFG_DOD0 = oam_d0;
	gFG_DOD1 = oam_d0;
	oam_d_1 = oam_d0;
	oam_d_2 = oam_d0;
	oam_d_3 = oam_d0;
	oam_d_4 = oam_d0;
	oam_d_5 = oam_d0;
	bm_print(BM_LOG_CRTI, "[battery_meter_reset]2 oam_d0=%d,oam_d_5=%d,ui=%d\n", oam_d0,
		 oam_d_5, ui_percentage);
#endif
}

kal_int32 battery_meter_reset(void)
{
#if defined(CONFIG_POWER_EXT)
	return 0;
#else
	kal_uint32 ui_percentage = bat_get_ui_percentage();

#if defined(CUST_CAPACITY_OCV2CV_TRANSFORM)
	if (KAL_FALSE == g_USE_UI_SOC) {
		ui_percentage = battery_meter_get_battery_soc();
		g_USE_UI_SOC = KAL_TRUE;
		bm_print(BM_LOG_FULL, "[CUST_CAPACITY_OCV2CV_TRANSFORM]Use Battery SOC: %d\n",
			 ui_percentage);
	}
#endif

	reset_parameter_car();
	reset_parameter_dod_full(ui_percentage);

	return 0;
#endif
}

kal_int32 battery_meter_sync(kal_int32 bat_i_sense_offset)
{
#if defined(CONFIG_POWER_EXT)
	return 0;
#else
	g_I_SENSE_offset = bat_i_sense_offset;
	return 0;
#endif
}

kal_int32 battery_meter_get_battery_zcv(void)
{
#if defined(CONFIG_POWER_EXT)
	return 3987;
#else
	return gFG_voltage;
#endif
}

kal_int32 battery_meter_get_battery_nPercent_zcv(void)
{
#if defined(CONFIG_POWER_EXT)
	return 3700;
#else
	return gFG_15_vlot;	/* 15% zcv,  15% can be customized by 100-g_tracking_point */
#endif
}

kal_int32 battery_meter_get_battery_nPercent_UI_SOC(void)
{
#if defined(CONFIG_POWER_EXT)
	return 15;
#else
	return g_tracking_point;	/* tracking point */
#endif
}

kal_int32 battery_meter_get_tempR(kal_int32 dwVolt)
{
#if defined(CONFIG_POWER_EXT)
	return 0;
#else
	int TRes;

	TRes =
	    (batt_meter_cust_data.rbat_pull_up_r * dwVolt) /
	    (batt_meter_cust_data.rbat_pull_up_volt - dwVolt);

	return TRes;
#endif
}

kal_int32 battery_meter_get_tempV(void)
{
#if defined(CONFIG_POWER_EXT)
	return 0;
#else
	int ret = 0;
	int val = 0;

	val = 1;		/* set avg times */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_BAT_TEMP, &val);
	return val;
#endif
}

kal_int32 battery_meter_get_VSense(void)
{
#if defined(CONFIG_POWER_EXT)
	return 0;
#else
	int ret = 0;
	int val = 0;

	val = 1;		/* set avg times */
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_ADC_V_I_SENSE, &val);
	return val;
#endif
}

/* ============================================================ // */
static ssize_t fgadc_log_write(struct file *filp, const char __user *buff,
			       size_t len, loff_t *data)
{
	if (copy_from_user(&proc_fgadc_data, buff, len)) {
		bm_print(BM_LOG_CRTI, "fgadc_log_write error.\n");
		return -EFAULT;
	}

	if (proc_fgadc_data[0] == '1') {
		bm_print(BM_LOG_CRTI, "enable FGADC driver log system\n");
		Enable_FGADC_LOG = 1;
	} else if (proc_fgadc_data[0] == '2') {
		bm_print(BM_LOG_CRTI, "enable FGADC driver log system:2\n");
		Enable_FGADC_LOG = 2;
	} else {
		bm_print(BM_LOG_CRTI, "Disable FGADC driver log system\n");
		Enable_FGADC_LOG = 0;
	}

	return len;
}

static const struct file_operations fgadc_proc_fops = {
	.write = fgadc_log_write,
};

int init_proc_log_fg(void)
{
	int ret = 0;

#if 1
	proc_create("fgadc_log", 0644, NULL, &fgadc_proc_fops);
	bm_print(BM_LOG_CRTI, "proc_create fgadc_proc_fops\n");
#else
	proc_entry_fgadc = create_proc_entry("fgadc_log", 0644, NULL);

	if (proc_entry_fgadc == NULL) {
		ret = -ENOMEM;
		bm_print(BM_LOG_CRTI, "init_proc_log_fg: Couldn't create proc entry\n");
	} else {
		proc_entry_fgadc->write_proc = fgadc_log_write;
		bm_print(BM_LOG_CRTI, "init_proc_log_fg loaded.\n");
	}
#endif

	return ret;
}

#ifdef MTK_BATTERY_LIFETIME_DATA_SUPPORT

/* ============================================================ // */

#ifdef CUSTOM_BATTERY_CYCLE_AGING_DATA

kal_int32 get_battery_aging_factor(kal_int32 cycle)
{
	kal_int32 i, f1, f2, c1, c2;
	kal_int32 saddles;

	saddles = sizeof(battery_aging_table) / sizeof(BATTERY_CYCLE_STRUC);

	for (i = 0; i < saddles; i++) {
		if (battery_aging_table[i].cycle == cycle)
			return battery_aging_table[i].aging_factor;


		if (battery_aging_table[i].cycle > cycle) {
			if (i == 0)
				return 100;


			if (battery_aging_table[i].aging_factor >
			    battery_aging_table[i - 1].aging_factor) {
				f1 = battery_aging_table[i].aging_factor;
				f2 = battery_aging_table[i - 1].aging_factor;
				c1 = battery_aging_table[i].cycle;
				c2 = battery_aging_table[i - 1].cycle;
				return f2 + ((cycle - c2) * (f1 - f2)) / (c1 - c2);
			} else {
				f1 = battery_aging_table[i - 1].aging_factor;
				f2 = battery_aging_table[i].aging_factor;
				c1 = battery_aging_table[i].cycle;
				c2 = battery_aging_table[i - 1].cycle;
				return f2 + ((cycle - c2) * (f1 - f2)) / (c1 - c2);
			}
		}
	}

	return battery_aging_table[saddles - 1].aging_factor;
}

#endif

static ssize_t show_FG_Battery_Cycle(struct device *dev, struct device_attribute *attr, char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] gFG_battery_cycle  : %d\n", gFG_battery_cycle);
	return sprintf(buf, "%d\n", gFG_battery_cycle);
}

static ssize_t store_FG_Battery_Cycle(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	kal_int32 cycle;

#ifdef CUSTOM_BATTERY_CYCLE_AGING_DATA
	kal_int32 aging_capacity;
	kal_int32 factor;
#endif

	if (1 == sscanf(buf, "%d", &cycle)) {
		bm_print(BM_LOG_CRTI, "[FG] update battery cycle count: %d\n", cycle);
		gFG_battery_cycle = cycle;

#ifdef CUSTOM_BATTERY_CYCLE_AGING_DATA
		/* perform cycle aging calculation */

		factor = get_battery_aging_factor(gFG_battery_cycle);
		if (factor > 0 && factor < 100) {
			bm_print(BM_LOG_CRTI, "[FG] cycle count to aging factor %d\n", factor);
			aging_capacity = gFG_BATT_CAPACITY * factor / 100;
			if (aging_capacity < gFG_BATT_CAPACITY_aging) {
				bm_print(BM_LOG_CRTI, "[FG] update gFG_BATT_CAPACITY_aging to %d\n",
					 aging_capacity);
				gFG_BATT_CAPACITY_aging = aging_capacity;
			}
		}
#endif
	} else {
		bm_print(BM_LOG_CRTI, "[FG] format error!\n");
	}
	return size;
}

static DEVICE_ATTR(FG_Battery_Cycle, 0664, show_FG_Battery_Cycle, store_FG_Battery_Cycle);

/* ------------------------------------------------------------------------------------------- */

static ssize_t show_FG_Max_Battery_Voltage(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] gFG_max_voltage  : %d\n", gFG_max_voltage);
	return sprintf(buf, "%d\n", gFG_max_voltage);
}

static ssize_t store_FG_Max_Battery_Voltage(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	kal_int32 voltage;
	if (1 == sscanf(buf, "%d", &voltage)) {
		if (voltage > gFG_max_voltage) {
			bm_print(BM_LOG_CRTI, "[FG] update battery max voltage: %d\n", voltage);
			gFG_max_voltage = voltage;
		}
	} else {
		bm_print(BM_LOG_CRTI, "[FG] format error!\n");
	}
	return size;
}

static DEVICE_ATTR(FG_Max_Battery_Voltage, 0664, show_FG_Max_Battery_Voltage,
		   store_FG_Max_Battery_Voltage);

/* ------------------------------------------------------------------------------------------- */

static ssize_t show_FG_Min_Battery_Voltage(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] gFG_min_voltage  : %d\n", gFG_min_voltage);
	return sprintf(buf, "%d\n", gFG_min_voltage);
}

static ssize_t store_FG_Min_Battery_Voltage(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	kal_int32 voltage;
	if (1 == sscanf(buf, "%d", &voltage)) {
		if (voltage < gFG_min_voltage) {
			bm_print(BM_LOG_CRTI, "[FG] update battery min voltage: %d\n", voltage);
			gFG_min_voltage = voltage;
		}
	} else {
		bm_print(BM_LOG_CRTI, "[FG] format error!\n");
	}
	return size;
}

static DEVICE_ATTR(FG_Min_Battery_Voltage, 0664, show_FG_Min_Battery_Voltage,
		   store_FG_Min_Battery_Voltage);

/* ------------------------------------------------------------------------------------------- */

static ssize_t show_FG_Max_Battery_Current(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] gFG_max_current  : %d\n", gFG_max_current);
	return sprintf(buf, "%d\n", gFG_max_current);
}

static ssize_t store_FG_Max_Battery_Current(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	kal_int32 bat_current;
	if (1 == sscanf(buf, "%d", &bat_current)) {
		if (bat_current > gFG_max_current) {
			bm_print(BM_LOG_CRTI, "[FG] update battery max current: %d\n", bat_current);
			gFG_max_current = bat_current;
		}
	} else {
		bm_print(BM_LOG_CRTI, "[FG] format error!\n");
	}
	return size;
}

static DEVICE_ATTR(FG_Max_Battery_Current, 0664, show_FG_Max_Battery_Current,
		   store_FG_Max_Battery_Current);

/* ------------------------------------------------------------------------------------------- */

static ssize_t show_FG_Min_Battery_Current(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] gFG_min_current  : %d\n", gFG_min_current);
	return sprintf(buf, "%d\n", gFG_min_current);
}

static ssize_t store_FG_Min_Battery_Current(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	kal_int32 bat_current;
	if (1 == sscanf(buf, "%d", &bat_current)) {
		if (bat_current < gFG_min_current) {
			bm_print(BM_LOG_CRTI, "[FG] update battery min current: %d\n", bat_current);
			gFG_min_current = bat_current;
		}
	} else {
		bm_print(BM_LOG_CRTI, "[FG] format error!\n");
	}
	return size;
}

static DEVICE_ATTR(FG_Min_Battery_Current, 0664, show_FG_Min_Battery_Current,
		   store_FG_Min_Battery_Current);

/* ------------------------------------------------------------------------------------------- */

static ssize_t show_FG_Max_Battery_Temperature(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] gFG_max_temperature  : %d\n", gFG_max_temperature);
	return sprintf(buf, "%d\n", gFG_max_temperature);
}

static ssize_t store_FG_Max_Battery_Temperature(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	kal_int32 temp;
	if (1 == sscanf(buf, "%d", &temp)) {
		if (temp > gFG_max_temperature) {
			bm_print(BM_LOG_CRTI, "[FG] update battery max temp: %d\n", temp);
			gFG_max_temperature = temp;
		}
	} else {
		bm_print(BM_LOG_CRTI, "[FG] format error!\n");
	}
	return size;
}

static DEVICE_ATTR(FG_Max_Battery_Temperature, 0664, show_FG_Max_Battery_Temperature,
		   store_FG_Max_Battery_Temperature);

/* ------------------------------------------------------------------------------------------- */

static ssize_t show_FG_Min_Battery_Temperature(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] gFG_min_temperature  : %d\n", gFG_min_temperature);
	return sprintf(buf, "%d\n", gFG_min_temperature);
}

static ssize_t store_FG_Min_Battery_Temperature(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	kal_int32 temp;
	if (1 == sscanf(buf, "%d", &temp)) {
		if (temp < gFG_min_temperature) {
			bm_print(BM_LOG_CRTI, "[FG] update battery min temp: %d\n", temp);
			gFG_min_temperature = temp;
		}
	} else {
		bm_print(BM_LOG_CRTI, "[FG] format error!\n");
	}
	return size;
}

static DEVICE_ATTR(FG_Min_Battery_Temperature, 0664, show_FG_Min_Battery_Temperature,
		   store_FG_Min_Battery_Temperature);

/* ------------------------------------------------------------------------------------------- */

static ssize_t show_FG_Aging_Factor(struct device *dev, struct device_attribute *attr, char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] gFG_aging_factor  : %d\n", gFG_aging_factor);
	return sprintf(buf, "%d\n", gFG_aging_factor);
}

static ssize_t store_FG_Aging_Factor(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	kal_int32 factor;
	kal_int32 aging_capacity;

	if (1 == sscanf(buf, "%d", &factor)) {
		if (factor <= 100 && factor >= 0) {
			bm_print(BM_LOG_CRTI,
				 "[FG] update battery aging factor: old(%d), new(%d)\n",
				 gFG_aging_factor, factor);

			gFG_aging_factor = factor;

			if (gFG_aging_factor != 100) {
				aging_capacity = gFG_BATT_CAPACITY * gFG_aging_factor / 100;
				if (aging_capacity < gFG_BATT_CAPACITY_aging) {
					bm_print(BM_LOG_CRTI,
						 "[FG] update gFG_BATT_CAPACITY_aging to %d\n",
						 aging_capacity);
					gFG_BATT_CAPACITY_aging = aging_capacity;
				}
			}
		}
	} else {
		bm_print(BM_LOG_CRTI, "[FG] format error!\n");
	}

	return size;
}

static DEVICE_ATTR(FG_Aging_Factor, 0664, show_FG_Aging_Factor, store_FG_Aging_Factor);

/* ------------------------------------------------------------------------------------------- */

#endif

/* ============================================================ // */
static ssize_t show_FG_Current(struct device *dev, struct device_attribute *attr, char *buf)
{
	kal_int32 ret = 0;
	kal_int32 fg_current_inout_battery = 0;
	kal_int32 val = 0;
	kal_bool is_charging = 0;

	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &val);
	ret = battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CURRENT_SIGN, &is_charging);

	if (is_charging == KAL_TRUE)
		fg_current_inout_battery = 0 - val;
	else
		fg_current_inout_battery = val;


	bm_print(BM_LOG_CRTI, "[FG] gFG_current_inout_battery : %d\n", fg_current_inout_battery);
	return sprintf(buf, "%d\n", fg_current_inout_battery);
}

static ssize_t store_FG_Current(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_Current, 0664, show_FG_Current, store_FG_Current);

/* ============================================================ // */
static ssize_t show_FG_g_fg_dbg_bat_volt(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_bat_volt : %d\n", g_fg_dbg_bat_volt);
	return sprintf(buf, "%d\n", g_fg_dbg_bat_volt);
}

static ssize_t store_FG_g_fg_dbg_bat_volt(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_bat_volt, 0664, show_FG_g_fg_dbg_bat_volt,
		   store_FG_g_fg_dbg_bat_volt);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_bat_current(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_bat_current : %d\n", g_fg_dbg_bat_current);
	return sprintf(buf, "%d\n", g_fg_dbg_bat_current);
}

static ssize_t store_FG_g_fg_dbg_bat_current(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_bat_current, 0664, show_FG_g_fg_dbg_bat_current,
		   store_FG_g_fg_dbg_bat_current);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_bat_zcv(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_bat_zcv : %d\n", g_fg_dbg_bat_zcv);
	return sprintf(buf, "%d\n", g_fg_dbg_bat_zcv);
}

static ssize_t store_FG_g_fg_dbg_bat_zcv(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_bat_zcv, 0664, show_FG_g_fg_dbg_bat_zcv, store_FG_g_fg_dbg_bat_zcv);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_bat_temp(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_bat_temp : %d\n", g_fg_dbg_bat_temp);
	return sprintf(buf, "%d\n", g_fg_dbg_bat_temp);
}

static ssize_t store_FG_g_fg_dbg_bat_temp(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_bat_temp, 0664, show_FG_g_fg_dbg_bat_temp,
		   store_FG_g_fg_dbg_bat_temp);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_bat_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_bat_r : %d\n", g_fg_dbg_bat_r);
	return sprintf(buf, "%d\n", g_fg_dbg_bat_r);
}

static ssize_t store_FG_g_fg_dbg_bat_r(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_bat_r, 0664, show_FG_g_fg_dbg_bat_r, store_FG_g_fg_dbg_bat_r);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_bat_car(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_bat_car : %d\n", g_fg_dbg_bat_car);
	return sprintf(buf, "%d\n", g_fg_dbg_bat_car);
}

static ssize_t store_FG_g_fg_dbg_bat_car(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_bat_car, 0664, show_FG_g_fg_dbg_bat_car, store_FG_g_fg_dbg_bat_car);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_bat_qmax(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_bat_qmax : %d\n", g_fg_dbg_bat_qmax);
	return sprintf(buf, "%d\n", g_fg_dbg_bat_qmax);
}

static ssize_t store_FG_g_fg_dbg_bat_qmax(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_bat_qmax, 0664, show_FG_g_fg_dbg_bat_qmax,
		   store_FG_g_fg_dbg_bat_qmax);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_d0(struct device *dev, struct device_attribute *attr, char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_d0 : %d\n", g_fg_dbg_d0);
	return sprintf(buf, "%d\n", g_fg_dbg_d0);
}

static ssize_t store_FG_g_fg_dbg_d0(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_d0, 0664, show_FG_g_fg_dbg_d0, store_FG_g_fg_dbg_d0);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_d1(struct device *dev, struct device_attribute *attr, char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_d1 : %d\n", g_fg_dbg_d1);
	return sprintf(buf, "%d\n", g_fg_dbg_d1);
}

static ssize_t store_FG_g_fg_dbg_d1(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_d1, 0664, show_FG_g_fg_dbg_d1, store_FG_g_fg_dbg_d1);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_percentage(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_percentage : %d\n", g_fg_dbg_percentage);
	return sprintf(buf, "%d\n", g_fg_dbg_percentage);
}

static ssize_t store_FG_g_fg_dbg_percentage(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_percentage, 0664, show_FG_g_fg_dbg_percentage,
		   store_FG_g_fg_dbg_percentage);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_percentage_fg(struct device *dev, struct device_attribute *attr,
					      char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_percentage_fg : %d\n", g_fg_dbg_percentage_fg);
	return sprintf(buf, "%d\n", g_fg_dbg_percentage_fg);
}

static ssize_t store_FG_g_fg_dbg_percentage_fg(struct device *dev, struct device_attribute *attr,
					       const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_percentage_fg, 0664, show_FG_g_fg_dbg_percentage_fg,
		   store_FG_g_fg_dbg_percentage_fg);
/* ------------------------------------------------------------------------------------------- */
static ssize_t show_FG_g_fg_dbg_percentage_voltmode(struct device *dev,
						    struct device_attribute *attr, char *buf)
{
	bm_print(BM_LOG_CRTI, "[FG] g_fg_dbg_percentage_voltmode : %d\n",
		 g_fg_dbg_percentage_voltmode);
	return sprintf(buf, "%d\n", g_fg_dbg_percentage_voltmode);
}

static ssize_t store_FG_g_fg_dbg_percentage_voltmode(struct device *dev,
						     struct device_attribute *attr, const char *buf,
						     size_t size)
{
	return size;
}

static DEVICE_ATTR(FG_g_fg_dbg_percentage_voltmode, 0664, show_FG_g_fg_dbg_percentage_voltmode,
		   store_FG_g_fg_dbg_percentage_voltmode);

/* ============================================================ // */
static int battery_meter_probe(struct platform_device *dev)
{
	int ret_device_file = 0;
#if defined(CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
	char *temp_strptr;
#endif
	battery_meter_ctrl = bm_ctrl_cmd;

	bm_print(BM_LOG_CRTI, "[battery_meter_probe] probe\n");

	batt_init_cust_data();

	/* select battery meter control method */
	battery_meter_ctrl = bm_ctrl_cmd;
#if defined(CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
	if (g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT
	    || g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) {
		temp_strptr =
		    kzalloc(strlen(saved_command_line) + strlen(" androidboot.mode=charger") + 1,
			    GFP_KERNEL);
		strcpy(temp_strptr, saved_command_line);
		strcat(temp_strptr, " androidboot.mode=charger");
		saved_command_line = temp_strptr;
	}
#endif
	/* LOG System Set */
	init_proc_log_fg();

	/* last_oam_run_time = rtc_read_hw_time(); */
	#if !defined(FG_SELF_DISCHARGE_CHECK)
	get_monotonic_boottime(&last_oam_run_time);
	#else
	getrawmonotonic(&last_oam_run_time);
	#endif
	/* Create File For FG UI DEBUG */
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Current);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_bat_volt);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_bat_current);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_bat_zcv);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_bat_temp);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_bat_r);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_bat_car);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_bat_qmax);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_d0);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_d1);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_percentage);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_percentage_fg);
	ret_device_file =
	    device_create_file(&(dev->dev), &dev_attr_FG_g_fg_dbg_percentage_voltmode);

#ifdef MTK_BATTERY_LIFETIME_DATA_SUPPORT
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Battery_Cycle);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Aging_Factor);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Max_Battery_Voltage);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Min_Battery_Voltage);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Max_Battery_Current);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Min_Battery_Current);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Max_Battery_Temperature);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Min_Battery_Temperature);
#endif

	return 0;
}

static int battery_meter_remove(struct platform_device *dev)
{
	bm_print(BM_LOG_CRTI, "[battery_meter_remove]\n");
	return 0;
}

static void battery_meter_shutdown(struct platform_device *dev)
{
	bm_print(BM_LOG_CRTI, "[battery_meter_shutdown]\n");
}

static int battery_meter_suspend(struct platform_device *dev, pm_message_t state)
{
#if defined(CONFIG_POWER_EXT)

#elif defined(SOC_BY_SW_FG) || defined(SOC_BY_HW_FG)
	struct timespec xts, tom;
#endif

#if defined(FG_BAT_INT)
#if defined(CONFIG_POWER_EXT)
#elif defined(SOC_BY_HW_FG)
	if (reset_fg_bat_int == KAL_TRUE) {
		battery_meter_set_columb_interrupt(gFG_BATT_CAPACITY / 100);
		reset_fg_bat_int = KAL_FALSE;
	} else {
		battery_meter_set_columb_interrupt(0x1ffff);
	}
#endif
#endif				/* #if defined(FG_BAT_INT) */

	/* -- hibernation path */
	if (state.event == PM_EVENT_FREEZE) {
		pr_warn("[%s] %p:%p\n", __func__, battery_meter_ctrl, &bm_ctrl_cmd);
		battery_meter_ctrl = bm_ctrl_cmd;
	}
	/* -- end of hibernation path */
#if defined(CONFIG_POWER_EXT)

#elif defined(SOC_BY_SW_FG) || defined(SOC_BY_HW_FG)
	{
#ifdef MTK_POWER_EXT_DETECT
		if (KAL_TRUE == bat_is_ext_power())
			return 0;
#endif
		get_xtime_and_monotonic_and_sleep_offset(&xts_before_sleep, &tom,
							 &g_rtc_time_before_sleep);
		#if !defined(FG_SELF_DISCHARGE_CHECK)
		if (_g_bat_sleep_total_time < g_spm_timer)
			return 0;

		_g_bat_sleep_total_time = 0;
		#else
		if (_g_bat_sleep_total_time >= g_spm_timer)
			_g_bat_sleep_total_time = 0;
		#endif		/* #if !defined(FG_SELF_DISCHARGE_CHECK) */

		battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_OCV, &g_hw_ocv_before_sleep);
	}
#endif
	bm_print(BM_LOG_CRTI, "[battery_meter_suspend]\n");
	return 0;
}

#if defined(SOC_BY_HW_FG)
#ifdef MTK_ENABLE_AGING_ALGORITHM
static void battery_aging_check(void)
{
	kal_int32 hw_ocv_after_sleep;
	struct timespec xts, tom, sleeptime;
	kal_int32 vbat;
	kal_int32 qmax_aging = 0;
	kal_int32 dod_gap = 10;
	kal_int32 columb_after_sleep = 0;
#if defined(MD_SLEEP_CURRENT_CHECK)
	kal_int32 DOD_hwocv;
	kal_int32 DOD_now;
	kal_int32 suspend_current = 0;
#endif

	battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_OCV, &hw_ocv_after_sleep);
	vbat = battery_meter_get_battery_voltage(KAL_TRUE);
	bm_print(BM_LOG_CRTI, "@@@ HW_OCV_D3=%d, HW_OCV_D1=%d, VBAT=%d\n", hw_ocv_after_sleep,
		 g_hw_ocv_before_sleep, vbat);

	/* gauge correct */
	battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CAR, &columb_after_sleep);
	/* update columb counter to get DOD_now. */

	get_xtime_and_monotonic_and_sleep_offset(&xts, &tom, &sleeptime);
	suspend_time += abs(xts.tv_sec - xts_before_sleep.tv_sec);
	_g_bat_sleep_total_time += abs(xts.tv_sec - xts_before_sleep.tv_sec);
#if defined(MD_SLEEP_CURRENT_CHECK)
	bm_print(BM_LOG_CRTI, "sleeptime=(%d)s, car_be = %d, car_af = %d\n", suspend_time,
		 columb_before_sleep, columb_after_sleep);
	if (columb_before_sleep == 0x123456) {
		columb_before_sleep = columb_after_sleep;
		suspend_time = 0;
		return;
	}
	if (hw_ocv_after_sleep != g_hw_ocv_before_sleep) {
		if (suspend_time > OCV_RECOVER_TIME) {	/* 35 mins */
			suspend_current =
			    abs(columb_after_sleep - columb_before_sleep) * 3600 / suspend_time;
			bm_print(BM_LOG_CRTI,
				 "[aging check]sleeptime = %d, HW_OCV_D3=%d, car_be = %d, car_af = %d, suspend cur = %d ",
				 suspend_time, hw_ocv_after_sleep, columb_before_sleep,
				 columb_after_sleep, suspend_current);
			if (suspend_current < 10) {	/* 10mA */
				columb_before_sleep = columb_after_sleep;
				suspend_time = 0;
				bm_print(BM_LOG_CRTI, "1\n");
			} else {
				columb_before_sleep = columb_after_sleep;
				suspend_time = 0;
				bm_print(BM_LOG_CRTI, "0\n");
				return;
			}
		} else {
			return;
		}
	} else {
		return;
	}
#endif
	/* aging */
#if !defined(MD_SLEEP_CURRENT_CHECK)
if (suspend_time > OCV_RECOVER_TIME)
#endif
	{
		if (aging_ocv_1 == 0) {
			aging_ocv_1 = hw_ocv_after_sleep;
			aging_car_1 = columb_after_sleep;
			/* aging_resume_time_1 = time_after_sleep.tv_sec; */

			if (fgauge_read_d_by_v(aging_ocv_1) > DOD1_ABOVE_THRESHOLD) {
				aging_ocv_1 = 0;
				bm_print(BM_LOG_CRTI,
					 "[aging check] reset and find next aging_ocv1 for better precision\n");
			}
		} else if (aging_ocv_2 == 0) {
			aging_ocv_2 = hw_ocv_after_sleep;
			aging_car_2 = columb_after_sleep;
			/* aging_resume_time_2 = time_after_sleep.tv_sec; */

			if (fgauge_read_d_by_v(aging_ocv_2) < DOD2_BELOW_THRESHOLD) {
				aging_ocv_2 = 0;
				bm_print(BM_LOG_CRTI,
					 "[aging check] reset and find next aging_ocv2 for better precision\n");
			}
		} else {
			aging_ocv_1 = aging_ocv_2;
			aging_car_1 = aging_car_2;
			/* aging_resume_time_1 = aging_resume_time_2; */

			aging_ocv_2 = hw_ocv_after_sleep;
			aging_car_2 = columb_after_sleep;
			/* aging_resume_time_2 = time_after_sleep.tv_sec; */
		}
	}

	if (aging_ocv_2 > 0) {
		aging_dod_1 = fgauge_read_d_by_v(aging_ocv_1);
		aging_dod_2 = fgauge_read_d_by_v(aging_ocv_2);

		/* check dod region to avoid hwocv error margin */
		dod_gap = MIN_DOD_DIFF_THRESHOLD;

		/* check if DOD gap bigger than setting */
		if (aging_dod_2 > aging_dod_1 && (aging_dod_2 - aging_dod_1) >= dod_gap) {
			/* do aging calculation */
			qmax_aging =
			    (100 * (aging_car_1 - aging_car_2)) / (aging_dod_2 - aging_dod_1);

			/* update if aging over 10%. */
			if (gFG_BATT_CAPACITY > qmax_aging
			    && ((gFG_BATT_CAPACITY - qmax_aging) >
				(gFG_BATT_CAPACITY / (100 - MIN_AGING_FACTOR)))) {
				bm_print(BM_LOG_CRTI,
					 "[aging check] before apply aging, qmax_aging(%d) qmax_now(%d) ocv1(%d) dod1(%d) car1(%d) ocv2(%d) dod2(%d) car2(%d)\n",
					 qmax_aging, gFG_BATT_CAPACITY, aging_ocv_1,
					 aging_dod_1, aging_car_1, aging_ocv_2, aging_dod_2,
					 aging_car_2);

#ifdef MTK_BATTERY_LIFETIME_DATA_SUPPORT
				gFG_aging_factor =
				    ((gFG_BATT_CAPACITY - qmax_aging) * 100) / gFG_BATT_CAPACITY;
#endif

				if (gFG_BATT_CAPACITY_aging > qmax_aging) {
					bm_print(BM_LOG_CRTI,
						 "[aging check] new qmax_aging %d old qmax_aging %d\n",
						 qmax_aging, gFG_BATT_CAPACITY_aging);
					gFG_BATT_CAPACITY_aging = qmax_aging;
					gFG_DOD0 = aging_dod_2;
					gFG_DOD1 = gFG_DOD0;
					reset_parameter_car();
				} else {
					bm_print(BM_LOG_CRTI,
						 "[aging check] current qmax_aging %d is smaller than calculated qmax_aging %d\n",
						 gFG_BATT_CAPACITY_aging, qmax_aging);
				}
			} else {
				aging_ocv_2 = 0;
				bm_print(BM_LOG_CRTI,
					 "[aging check] show no degrade, qmax_aging(%d) qmax_now(%d) ocv1(%d) dod1(%d) car1(%d) ocv2(%d) dod2(%d) car2(%d)\n",
					 qmax_aging, gFG_BATT_CAPACITY, aging_ocv_1,
					 aging_dod_1, aging_car_1, aging_ocv_2, aging_dod_2,
					 aging_car_2);
				bm_print(BM_LOG_CRTI,
					 "[aging check] reset and find next aging_ocv2\n");
			}
		} else {
			aging_ocv_2 = 0;
			bm_print(BM_LOG_CRTI, "[aging check] reset and find next aging_ocv2\n");
		}
		bm_print(BM_LOG_CRTI,
			 "[aging check] qmax_aging(%d) qmax_now(%d) ocv1(%d) dod1(%d) car1(%d) ocv2(%d) dod2(%d) car2(%d)\n",
			 qmax_aging, gFG_BATT_CAPACITY, aging_ocv_1, aging_dod_1,
			 aging_car_1, aging_ocv_2, aging_dod_2, aging_car_2);
	}
#if defined(MD_SLEEP_CURRENT_CHECK)
	/* self-discharging */
	if (hw_ocv_after_sleep < vbat) {
		bm_print(BM_LOG_CRTI, "Ignore HW_OCV : smaller than VBAT\n");
	} else {

		DOD_hwocv = fgauge_read_d_by_v(hw_ocv_after_sleep);

		battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_FG_CAR, &gFG_columb);
		/* update columb counter to get DOD_now. */
		DOD_now = 100 - fgauge_read_capacity(1);

		if (DOD_hwocv > DOD_now && (DOD_hwocv - DOD_now > SELF_DISCHARGE_CHECK_THRESHOLD)) {
			gFG_DOD0 = DOD_hwocv;
			gFG_DOD1 = gFG_DOD0;
			reset_parameter_car();
			bm_print(BM_LOG_CRTI,
				 "[self-discharge check] reset to HWOCV. dod_ocv(%d) dod_now(%d)\n",
				 DOD_hwocv, DOD_now);
		}
		bm_print(BM_LOG_CRTI, "[self-discharge check] dod_ocv(%d) dod_now(%d)\n",
			 DOD_hwocv, DOD_now);
		bm_print(BM_LOG_CRTI,
			 "be_ocv=(%d), af_ocv=(%d), D0=(%d), car=(%d)\n",
			 g_hw_ocv_before_sleep, hw_ocv_after_sleep, gFG_DOD0, gFG_columb);
	}
#endif
}
#endif
#endif

static int battery_meter_resume(struct platform_device *dev)
{
#if defined(CONFIG_POWER_EXT)

#elif defined(SOC_BY_SW_FG) || defined(SOC_BY_HW_FG)
#if defined(SOC_BY_SW_FG)
	kal_int32 hw_ocv_after_sleep = -1;
	#if defined(FG_SELF_DISCHARGE_CHECK)
	kal_int32 DOD_hwocv;
	struct timespec now_time;
	#endif
#endif
	struct timespec xts, tom, rtc_time_after_sleep;
#ifdef MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return 0;
#endif

	get_xtime_and_monotonic_and_sleep_offset(&xts, &tom, &rtc_time_after_sleep);
	_g_bat_sleep_total_time += rtc_time_after_sleep.tv_sec - g_rtc_time_before_sleep.tv_sec;
	battery_log(BAT_LOG_CRTI,
		    "[battery_meter_resume] sleep time = %d, g_spm_timer = %d\n",
		    _g_bat_sleep_total_time, g_spm_timer);

#if defined(SOC_BY_HW_FG)
#ifdef MTK_ENABLE_AGING_ALGORITHM
	if (bat_is_charger_exist() == KAL_FALSE)
		battery_aging_check();

#endif
#endif

#if !defined(FG_SELF_DISCHARGE_CHECK)
	if (_g_bat_sleep_total_time < g_spm_timer)
		return 0;

	bat_spm_timeout = true;
#else
	/* trigger gauge update if accumulated sleep time more than give period */
	if (_g_bat_sleep_total_time >= g_spm_timer)
		bat_spm_timeout = true;
#endif

#if defined(SOC_BY_SW_FG)
#if !defined(FG_SELF_DISCHARGE_CHECK)
	battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_OCV, &hw_ocv_after_sleep);
	if (_g_bat_sleep_total_time > 3600) {	/* 1hr */
		if (hw_ocv_after_sleep < g_hw_ocv_before_sleep) {
			oam_d0 = fgauge_read_d_by_v(hw_ocv_after_sleep);
			oam_v_ocv_2 = oam_v_ocv_1 = hw_ocv_after_sleep;
			oam_car_1 = 0;
			oam_car_2 = 0;
		} else {
			oam_car_1 = oam_car_1 +
				(40 * (rtc_time_after_sleep.tv_sec - g_rtc_time_before_sleep.tv_sec) / 3600);
/* 0.1mAh */
			oam_car_2 = oam_car_2 +
				(40 * (rtc_time_after_sleep.tv_sec - g_rtc_time_before_sleep.tv_sec) / 3600);
/* 0.1mAh */
		}
	}
	/* FIXME */
#else
	/* trigger gauge update if oam_run() not run in the last 30s kernel active time */
	getrawmonotonic(&now_time);
	if (now_time.tv_sec - last_oam_run_time.tv_sec > 30) {
		bat_spm_timeout = true;
		bm_print(BM_LOG_CRTI, "[battery_meter] trigger oam_run() for 30s threshold.\n");
	}

	/* try to calibrate D0 by HWOCV if battery has no loading for more than 30mins */
	if (rtc_time_after_sleep.tv_sec - g_rtc_time_before_sleep.tv_sec > 1800 &&
		bat_is_charger_exist() == KAL_FALSE) {

		battery_meter_ctrl(BATTERY_METER_CMD_GET_HW_OCV, &hw_ocv_after_sleep);

		DOD_hwocv = fgauge_read_d_by_v(hw_ocv_after_sleep);

		if ((DOD_hwocv < 30 || DOD_hwocv > 70) &&
			(DOD_hwocv > oam_d_2 && DOD_hwocv - oam_d_2 > 3)) {
			oam_d0 = DOD_hwocv;
			oam_v_ocv_2 = oam_v_ocv_1 = hw_ocv_after_sleep;
			oam_car_1 = 0;
			oam_car_2 = 0;

			bm_print(BM_LOG_CRTI,
				"[self-discharge check] reset to HWOCV. dod_ocv(%d) dod_now(%d)\n",
				DOD_hwocv, oam_d_2);
		} else {
			oam_car_1 = oam_car_1 +
				(40 * (rtc_time_after_sleep.tv_sec - g_rtc_time_before_sleep.tv_sec)/3600);
/* 0.1mAh */
			oam_car_2 = oam_car_2 +
				(40 * (rtc_time_after_sleep.tv_sec - g_rtc_time_before_sleep.tv_sec)/3600);
/* 0.1mAh */
		}

		bm_print(BM_LOG_CRTI, "[self-discharge check] dod_ocv(%d) dod_now(%d)\n", DOD_hwocv, oam_d_2);
	} else {
		oam_car_1 = oam_car_1 +
			(40 * (rtc_time_after_sleep.tv_sec - g_rtc_time_before_sleep.tv_sec)/3600);
/* 0.1mAh */
		oam_car_2 = oam_car_2 +
			(40 * (rtc_time_after_sleep.tv_sec - g_rtc_time_before_sleep.tv_sec)/3600);
/* 0.1mAh */
	}
#endif

	bm_print(BM_LOG_CRTI,
		 "sleeptime=(%d)s, be_ocv=(%d), af_ocv=(%d), D0=(%d), car1=(%d), car2=(%d)\n",
		 _g_bat_sleep_total_time,
		 g_hw_ocv_before_sleep, hw_ocv_after_sleep, oam_d0, oam_car_1, oam_car_2);
#endif
#endif

#if defined(FG_BAT_INT)
#if defined(CONFIG_POWER_EXT)
#elif defined(SOC_BY_HW_FG)
	battery_meter_set_columb_interrupt(0);
#endif
#endif				/* #if defined(FG_BAT_INT) */

	bm_print(BM_LOG_CRTI, "[battery_meter_resume]\n");
	return 0;
}

/* ----------------------------------------------------- */

#ifdef CONFIG_OF
static const struct of_device_id mt_bat_meter_of_match[] = {
	{.compatible = "mediatek,bat_meter",},
	{},
};

MODULE_DEVICE_TABLE(of, mt_bat_meter_of_match);
#endif
struct platform_device battery_meter_device = {
	.name = "battery_meter",
	.id = -1,
};


static struct platform_driver battery_meter_driver = {
	.probe = battery_meter_probe,
	.remove = battery_meter_remove,
	.shutdown = battery_meter_shutdown,
	.suspend = battery_meter_suspend,
	.resume = battery_meter_resume,
	.driver = {
		   .name = "battery_meter",
		   },
};

static int battery_meter_dts_probe(struct platform_device *dev)
{
	int ret = 0;
	/* struct proc_dir_entry *entry = NULL; */

	battery_log(BAT_LOG_CRTI, "******** battery_meter_dts_probe!! ********\n");

	battery_meter_device.dev.of_node = dev->dev.of_node;
	ret = platform_device_register(&battery_meter_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI,
			    "****[battery_meter_dts_probe] Unable to register device (%d)\n", ret);
		return ret;
	}
	return 0;

}

static struct platform_driver battery_meter_dts_driver = {
	.probe = battery_meter_dts_probe,
	.remove = NULL,
	.shutdown = NULL,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		   .name = "battery_meter_dts",
#ifdef CONFIG_OF
		   .of_match_table = mt_bat_meter_of_match,
#endif
		   },
};

static int __init battery_meter_init(void)
{
	int ret;

#ifdef CONFIG_OF
	/*  */
#else
	ret = platform_device_register(&battery_meter_device);
	if (ret) {
		bm_print(BM_LOG_CRTI, "[battery_meter_driver] Unable to device register(%d)\n",
			 ret);
		return ret;
	}
#endif

	ret = platform_driver_register(&battery_meter_driver);
	if (ret) {
		bm_print(BM_LOG_CRTI, "[battery_meter_driver] Unable to register driver (%d)\n",
			 ret);
		return ret;
	}
#ifdef CONFIG_OF
	ret = platform_driver_register(&battery_meter_dts_driver);
#endif
	bm_print(BM_LOG_CRTI, "[battery_meter_driver] Initialization : DONE\n");

	return 0;

}

#ifdef BATTERY_MODULE_INIT
/* #if 0 */
/* late_initcall(battery_meter_init); */
device_initcall(battery_meter_init);
#else
static void __exit battery_meter_exit(void)
{
}
module_init(battery_meter_init);
/* module_exit(battery_meter_exit); */
#endif

MODULE_AUTHOR("James Lo");
MODULE_DESCRIPTION("Battery Meter Device Driver");
MODULE_LICENSE("GPL");
