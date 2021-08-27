#ifndef _MT_POWER_GS_H
#define _MT_POWER_GS_H

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

/*****************
* extern variable 
******************/
extern struct proc_dir_entry *mt_power_gs_dir;

/*****************
* extern function 
******************/
extern void mt_power_gs_compare(char *, const unsigned int *, unsigned int);

extern void mt_power_gs_dump_suspend(void);
#if defined (CONFIG_MTK_PMIC_CHIP_MT6353)
extern const unsigned int *MT6353_PMIC_REG_gs_flightmode_suspend_mode;
extern unsigned int MT6353_PMIC_REG_gs_flightmode_suspend_mode_len;
#else
extern const unsigned int *MT6351_PMIC_REG_gs_flightmode_suspend_mode;
extern unsigned int MT6351_PMIC_REG_gs_flightmode_suspend_mode_len;
#endif
extern void mt_power_gs_dump_dpidle(void);
#if defined (CONFIG_MTK_PMIC_CHIP_MT6353)
extern const unsigned int *MT6353_PMIC_REG_gs_early_suspend_deep_idle_mode;
extern unsigned int MT6353_PMIC_REG_gs_early_suspend_deep_idle_mode_len;
#else
extern const unsigned int *MT6351_PMIC_REG_gs_early_suspend_deep_idle__mode;
extern unsigned int MT6351_PMIC_REG_gs_early_suspend_deep_idle__mode_len;
#endif
//extern void mt_power_gs_dump_idle(void);
//extern void mt_power_gs_dump_audio_playback(void);

#endif
