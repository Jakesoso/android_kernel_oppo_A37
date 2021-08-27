#include <linux/kernel.h>
#include <linux/string.h>

#include <mach/mtk_rtc.h>
#include <mach/wd_api.h>
extern void wdt_arch_reset(char);
#ifdef VENDOR_EDIT
//rendong.shi@BSP.boot, 2015/04/27, add for reboot kernel panic mode
extern int is_kernel_panic;
#endif

void arch_reset(char mode, const char *cmd)
{
	char reboot = 0;
	int res = 0;
	struct wd_api *wd_api = NULL;
#ifdef CONFIG_FPGA_EARLY_PORTING
    return ;
#else

	res = get_wd_api(&wd_api);
	pr_warn("arch_reset: cmd = %s\n", cmd ? : "NULL");

	if (cmd && !strcmp(cmd, "charger")) {
		/* do nothing */
	} else if (cmd && !strcmp(cmd, "recovery")) {
 #ifndef CONFIG_MTK_FPGA
		rtc_mark_recovery();
 #endif
#ifndef  DISABLE_FASTBOOT_CMD
//rendong.shi@BSP.boot, 2015/07/23, add for disable fastboot for release version
	} else if (cmd && !strcmp(cmd, "bootloader")) {
 #ifndef CONFIG_MTK_FPGA
		rtc_mark_fast();
 #endif
 #endif
	}
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	else if (cmd && !strcmp(cmd, "kpoc")) {
		rtc_mark_kpoc();
	}
#endif
#ifdef VENDOR_EDIT
//rendong.shi@BSP.boot, 2015/06/03, add for silence mode
   	else if (cmd && !strcmp(cmd, "silence")){
		rtc_mark_silence();
		reboot = 1;
	}
	else if (cmd && !strcmp(cmd, "sau")){
		rtc_mark_sau();
	}
	else if (cmd && !strcmp(cmd, "mos")){
		rtc_mark_mos();
	}
	else if (cmd && !strcmp(cmd, "meta")){ //mingqiang.guo@bsp.boot 2015/10/16, add for reboot meta
		rtc_mark_meta();
	}
#endif
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2016.1.30 for sim
	else if (cmd && !strcmp(cmd, "sim")){
		reboot = 2;
	}
#endif/*VENDOR_EDIT*/	
    else {
    	reboot = 1;
    }

#ifdef VENDOR_EDIT
//rendong.shi@BSP.boot, 2015/04/27, add for reboot kernel panic mode	
	if(is_kernel_panic)
	{
	   rtc_mark_reboot_kernel();
	}
#endif	
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2016.2.17 for cmd != NULL all hardreboot
	if(cmd != NULL)
		reboot = 2;
#endif/*VENDOR_EDIT*/			
	if (res) {
		pr_warn("arch_reset, get wd api error %d\n", res);
	} else {
		wd_api->wd_sw_reset(reboot);
	}
 #endif
}
