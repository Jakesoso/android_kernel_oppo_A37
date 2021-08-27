#ifndef _MT_SPM_IDLE_
#define _MT_SPM_IDLE_

#include <linux/kernel.h>
#include <linux/xlog.h>
#include <mach/mt_spm.h>
#include <mach/mt_spm_sleep.h>


#define TAG     "SPM-Idle"

#define spm_idle_err(fmt, args...)		pr_emerg(TAG fmt, ##args)
#define spm_idle_warn(fmt, args...)		pr_warn(TAG fmt, ##args)
#define spm_idle_dbg(fmt, args...)		pr_debug(TAG fmt, ##args)
#define spm_idle_info(fmt, args...)		pr_debug(TAG fmt, ##args)
#define spm_idle_ver(fmt, args...)		pr_debug(TAG fmt, ##args)	/* pr_debug show nothing */

/*
 * for SPM common part
 */
extern unsigned int spm_get_cpu_pwr_status(void);

/*
 * for Deep Idle
 */
void spm_deepidle_init(void);
void spm_dpidle_before_wfi(int cpu);		 /* can be redefined */
void spm_dpidle_after_wfi(int cpu, u32 spm_debug_flag);		 /* can be redefined */
wake_reason_t spm_go_to_dpidle(u32 spm_flags, u32 spm_data, u32 dump_log);
wake_reason_t spm_go_to_sleep_dpidle(u32 spm_flags, u32 spm_data);
int spm_set_dpidle_wakesrc(u32 wakesrc, bool enable, bool replace);
bool spm_set_dpidle_pcm_init_flag(void);

#define DEEPIDLE_LOG_NONE      0
#define DEEPIDLE_LOG_REDUCED   1
#define DEEPIDLE_LOG_FULL      2

/*
 * for Screen On Deep Idle 3.0
 */
void spm_sodi3_init(void);
wake_reason_t spm_go_to_sodi3(u32 spm_flags, u32 spm_data, u32 sodi_flags);
void spm_enable_sodi3(bool);
bool spm_get_sodi3_en(void);

/*
 * for Screen On Deep Idle
 */
void spm_sodi_init(void);
wake_reason_t spm_go_to_sodi(u32 spm_flags, u32 spm_data, u32 sodi_flags);
void spm_enable_sodi(bool);
bool spm_get_sodi_en(void);

void spm_sodi_mempll_pwr_mode(bool pwr_mode);
bool spm_get_sodi_mempll(void);

/*
 * for Multi Core Deep Idle
 */
enum spm_mcdi_lock_id {
	SPM_MCDI_IDLE = 0,
	SPM_MCDI_VCORE_DVFS = 1,
	SPM_MCDI_EARLY_SUSPEND = 2,
};

void mcidle_before_wfi(int cpu);
void mcidle_after_wfi(int cpu);
void spm_mcdi_init(void);
void spm_mcdi_switch_on_off(enum spm_mcdi_lock_id id, int mcdi_en);
bool spm_mcdi_wfi(int core_id);
bool spm_mcdi_can_enter(void);
bool spm_is_cpu_irq_occur(int core_id);

bool go_to_mcidle(int cpu);

#endif
