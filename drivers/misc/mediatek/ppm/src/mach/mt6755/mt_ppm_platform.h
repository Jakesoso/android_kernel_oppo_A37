
#ifndef __MT_PPM_PLATFORM_H__
#define __MT_PPM_PLATFORM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mach/mt_ppm_api.h"

/*==============================================================*/
/* Macros							*/
/*==============================================================*/
/* ppm driver update state to MET directly  0: turn off */
#define PPM_UPDATE_STATE_DIRECT_TO_MET  	(1)

#define PPM_IC_SEGMENT_CHECK		(1)
#define PPM_VPROC_5A_LIMIT_CHECK	(1)
#ifdef PPM_VPROC_5A_LIMIT_CHECK
#define PPM_5A_LIMIT_FREQ_IDX		(1)
#endif

#define DLPT_MAX_REAL_POWER_FY	(3890)
#define DLPT_MAX_REAL_POWER_SB	(4992)

#define	LCMOFF_MIN_FREQ		(598000)
#define	PTPOD_FREQ_IDX		(3)
#define SUSPEND_FREQ_IDX_LL	(3)
#define SUSPEND_FREQ_IDX_L	(5)
#define PWRTHRO_BAT_PER_MW	(610)
#define PWRTHRO_BAT_OC_MW	(610)
#define PWRTHRO_LOW_BAT_LV1_MW	(610)
#define PWRTHRO_LOW_BAT_LV2_MW	(610)

#define NR_CLUSTERS		(2)
#define DVFS_OPP_NUM		(8)

#define PPM_DEFAULT_HOLD_TIME		(4)
#define PPM_DEFAULT_FREQ_HOLD_TIME	(4)
#define PPM_DEFAULT_DELTA		(20)
#define PPM_LOADING_UPPER		(400)
#define PPM_TLP_CRITERIA		(400)

#define get_cluster_lcmoff_min_freq(id)		LCMOFF_MIN_FREQ	/* the same for each cluster */
#define get_cluster_ptpod_fix_freq_idx(id)	PTPOD_FREQ_IDX	/* the same for each cluster */
#define get_cluster_suspend_fix_freq_idx(id)	\
	((id == 0) ? SUSPEND_FREQ_IDX_LL : SUSPEND_FREQ_IDX_L)

/*==============================================================*/
/* Enum								*/
/*==============================================================*/
enum ppm_power_state {
	PPM_POWER_STATE_LL_ONLY = 0,
	PPM_POWER_STATE_L_ONLY,
	PPM_POWER_STATE_4LL_L,
	PPM_POWER_STATE_4L_LL,	/* Need this? */

	PPM_POWER_STATE_NONE,	/* HICA disabled */
	NR_PPM_POWER_STATE = PPM_POWER_STATE_NONE,
};

/*==============================================================*/
/* Data Structures						*/
/*==============================================================*/
struct ppm_power_tbl {
	const unsigned int index;
	struct {
		int opp_lv;
		unsigned int core_num;
	} const cluster_cfg[NR_CLUSTERS];
	const unsigned int perf_idx;
	const unsigned int power_idx;
};

struct ppm_power_tbl_data {
	const struct ppm_power_tbl *power_tbl;
	const unsigned int nr_power_tbl;
};

/*==============================================================*/
/* Global Variables						*/
/*==============================================================*/

/*==============================================================*/
/* APIs								*/
/*==============================================================*/
#ifdef PPM_IC_SEGMENT_CHECK
extern enum ppm_power_state ppm_check_fix_state_by_segment(void);
#endif

#ifdef __cplusplus
}
#endif

#endif



