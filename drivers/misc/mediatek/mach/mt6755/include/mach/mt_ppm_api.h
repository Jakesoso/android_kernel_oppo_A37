
#ifndef __MT_PPM_API_H__
#define __MT_PPM_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/cpufreq.h>


/*==============================================================*/
/* Enum								*/
/*==============================================================*/
enum ppm_client {
	PPM_CLIENT_DVFS	= 0,
	PPM_CLIENT_HOTPLUG,

	NR_PPM_CLIENTS,
};

enum dvfs_table_type {
	DVFS_TABLE_TYPE_FY = 0,
	DVFS_TABLE_TYPE_SB,

	NR_DVFS_TABLE_TYPE,
};

enum ppm_sysboost_user {
	BOOST_BY_WIFI = 0,
	BOOST_BY_PERFSERV,
	BOOST_BY_UT,

	NR_PPM_SYSBOOST_USER,
};

/*==============================================================*/
/* Definition							*/
/*==============================================================*/
typedef void (*met_set_ppm_state_funcMET)(unsigned int state);

/*==============================================================*/
/* Data Structures						*/
/*==============================================================*/
struct ppm_client_req {
	unsigned int	cluster_num;
	struct ppm_client_limit {
		unsigned int cluster_id;
		unsigned int cpu_id;

		int min_cpufreq_idx;
		int max_cpufreq_idx;
		unsigned int min_cpu_core;
		unsigned int max_cpu_core;

		bool has_advise_freq;
		bool has_advise_core;
		int advise_cpufreq_idx;
		int advise_cpu_core;
	} *cpu_limit;
};

struct ppm_client_data {
	const char *name;
	enum ppm_client	client;

	/* callback */
	void (*limit_cb)(struct ppm_client_req req);
};

struct ppm_cluster_status {
	int core_num;
	int freq_idx;	/* -1 if core_num = 0 */
	int volt;
};

/*==============================================================*/
/* APIs								*/
/*==============================================================*/
#ifdef VENDOR_EDIT
//xiaocheng.li@Swdp.shanghai, 2016/3/30, Add API for enable/disable ppm
/* enable */
extern int mt_ppm_set_enabled(unsigned int enabled);
#endif

extern void mt_ppm_set_dvfs_table(unsigned int cpu, struct cpufreq_frequency_table *tbl,
	unsigned int num, enum dvfs_table_type type);
extern void mt_ppm_register_client(enum ppm_client client, void (*limit)(struct ppm_client_req req));
extern void mt_ppm_set_5A_limit_throttle(bool enable);

/* SYS boost policy */
extern void mt_ppm_sysboost_core(enum ppm_sysboost_user user, unsigned int core_num);
extern void mt_ppm_sysboost_freq(enum ppm_sysboost_user user, unsigned int freq);

/* DLPT policy */
extern void mt_ppm_dlpt_set_limit_by_pbm(unsigned int limited_power);
extern void mt_ppm_dlpt_kick_PBM(struct ppm_cluster_status *cluster_status, unsigned int cluster_num);

/* Thermal policy */
extern void mt_ppm_cpu_thermal_protect(unsigned int limited_power);
extern unsigned int mt_ppm_thermal_get_min_power(void);
extern unsigned int mt_ppm_thermal_get_max_power(void);
extern unsigned int mt_ppm_thermal_get_cur_power(void);

/* PTPOD policy */
extern void mt_ppm_ptpod_policy_activate(void);
extern void mt_ppm_ptpod_policy_deactivate(void);

/* HICA policy */
extern void mt_ppm_hica_update_algo_data(unsigned int cur_loads,
					unsigned int cur_nr_heavy_task, unsigned int cur_tlp);
extern int mt_ppm_main(void);

/* MET */
void mt_set_ppm_state_registerCB(met_set_ppm_state_funcMET pCB);

#ifdef __cplusplus
}
#endif

#endif


