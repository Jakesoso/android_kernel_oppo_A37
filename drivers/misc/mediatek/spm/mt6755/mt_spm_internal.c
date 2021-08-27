#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/of_fdt.h>
#include <mach/mt_spm_mtcmos_internal.h>
#include <asm/setup.h>
#include "mt_spm_internal.h"
#include <mach/upmu_common.h>
#include <linux/aee.h>
#include <primary_display.h>
#include <mach/memory.h>
/**************************************
 * Config and Parameter
 **************************************/
#define LOG_BUF_SIZE		256
#define PCM_REST_WAIT_TIMEOUT   2000

/**************************************
 * Define and Declare
 **************************************/
DEFINE_SPINLOCK(__spm_lock);
atomic_t __spm_mainpll_req = ATOMIC_INIT(0);

static u32 pcm_timer_ramp_max = 1;
static u32 pcm_timer_ramp_max_sec_loop = 1;
#ifdef VENDOR_EDIT
//Wenxian.ZhEN@Prd.BaseDrv, 2016/04/24, Add for wake up source
char wakeup_source_buf[LOG_BUF_SIZE] = { 0 };
u64  wakesrc_count[32] = { 0 };
extern int wakeup_reason_stastics_flag;
#endif /* VENDOR_EDIT */

const char *wakesrc_str[32] = {
	[0] = " R12_PCM_TIMER",
	[1] = " R12_MD32_WDT_EVENT_B",
	[2] = " R12_KP_IRQ_B",
	[3] = " R12_APWDT_EVENT_B",
	[4] = " R12_APXGPT1_EVENT_B",
	[5] = " R12_CONN2AP_SPM_WAKEUP_B",
	[6] = " R12_EINT_EVENT_B",
	[7] = " R12_CONN_WDT_IRQ_B",
	[8] = " R12_CCIF0_EVENT_B",
	[9] = " R12_LOWBATTERY_IRQ_B",
	[10] = " R12_MD32_SPM_IRQ_B",
	[11] = " R12_26M_WAKE",
	[12] = " R12_26M_SLEEP",
	[13] = " R12_PCM_WDT_WAKEUP_B",
	[14] = " R12_USB_CDSC_B",
	[15] = " R12_USB_POWERDWN_B",
	[16] = " R12_C2K_WDT_IRQ_B",
	[17] = " R12_EINT_EVENT_SECURE_B",
	[18] = " R12_CCIF1_EVENT_B",
	[19] = " R12_UART0_IRQ_B",
	[20] = " R12_AFE_IRQ_MCU_B",
	[21] = " R12_THERM_CTRL_EVENT_B",
	[22] = " R12_SYS_CIRQ_IRQ_B",
	[23] = " R12_MD2_WDT_B",
	[24] = " R12_CSYSPWREQ_B",
	[25] = " R12_MD1_WDT_B",
	[26] = " R12_CLDMA_EVENT_B",
	[27] = " R12_SEJ_WDT_GPT_B",
	[28] = " R12_ALL_MD32_WAKEUP_B",
	[29] = " R12_CPU_IRQ_B",
	[30] = " R12_APSRC_WAKE",
	[31] = " R12_APSRC_SLEEP",
};

/**************************************
 * Function and API
 **************************************/
 #ifdef VENDOR_EDIT
//Wenxian.ZhEN@Prd.BaseDrv, 2016/04/24, Add for wake up source
#define MAX_WAKEUP_REASON_IRQS 32
 void mt_clear_wakesrc_count(void)
{
	int i = 0;
	
	for (i = 0; i < MAX_WAKEUP_REASON_IRQS; i++) 
		wakesrc_count[i] = 0;
}
EXPORT_SYMBOL(mt_clear_wakesrc_count);
#endif /* VENDOR_EDIT */

void __spm_reset_and_init_pcm(const struct pcm_desc *pcmdesc)
{
	u32 con1;
	int vcorefs_retry;
	u32 save_r1, save_r15, save_pcm_sta, save_irq_sta; 
	u32 final_r15, final_pcm_sta;

	/* [Vcorefs] backup r0 to POWER_ON_VAL0 for MEM Ctrl should work during PCM reset */
	if (spm_read(PCM_REG1_DATA) == 0x1) {
		save_r1  = spm_read(PCM_REG1_DATA);
		save_r15 = spm_read(PCM_REG15_DATA);
		save_pcm_sta = spm_read(PCM_FSM_STA);
		save_irq_sta = spm_read(SPM_IRQ_STA);
		con1 = spm_read(SPM_WAKEUP_EVENT_MASK);
		spm_write(SPM_WAKEUP_EVENT_MASK, (con1 & ~(0x1)));
		spm_write(SPM_CPU_WAKEUP_EVENT, 1);

		vcorefs_retry = 0;
		while ((spm_read(SPM_IRQ_STA) & PCM_IRQ_ROOT_MASK_LSB) == 0) {
			if (vcorefs_retry > PCM_REST_WAIT_TIMEOUT) {
				pr_err("[VcoreFS] init state: r15=0x%x r1=0x%x pcmsta=0x%x irqsta=0x%x\n", save_r15, save_r1, save_pcm_sta, save_irq_sta);
				pr_err("[VcoreFS] CPU triggers wake-up event fail!(timeout=%d)\n",
					 PCM_REST_WAIT_TIMEOUT);
				pr_err("[VcoreFS] curr state: r15=0x%x r6=0x%x pcmsta=0x%x irqsta=0x%x\n", 
						spm_read(PCM_REG15_DATA), spm_read(PCM_REG6_DATA),
						spm_read(PCM_FSM_STA), spm_read(SPM_IRQ_STA));

				__check_dvfs_halt_source(__spm_vcore_dvfs.pwrctrl->dvfs_halt_src_chk);
				/* BUG(); */
				final_r15     = spm_read(PCM_REG15_DATA);
				final_pcm_sta = spm_read(PCM_FSM_STA);
				pr_err("[VcoreFS] next state: r15=0x%x r6=0x%x pcmsta=0x%x irqsta=0x%x\n", 
						final_r15, spm_read(PCM_REG6_DATA),
						final_pcm_sta, spm_read(SPM_IRQ_STA));

				if (final_r15 == 0 && (final_pcm_sta & 0xFFFF) == 0x8490)
					break;
				else
					pr_err("[VcoreFS] can not reset without idle(r15=0x%x pcm_sta=0x%x)\n", final_r15, final_pcm_sta);
			}
			udelay(1);
			vcorefs_retry++;
		}

		spm_write(SPM_CPU_WAKEUP_EVENT, 0);
		spm_write(SPM_WAKEUP_EVENT_MASK, con1);

		/* backup mem control from r0 to POWER_ON_VAL0 */
		if (spm_read(SPM_POWER_ON_VAL0) != spm_read(PCM_REG0_DATA)) {
			spm_crit("VAL0 from 0x%x to 0x%x\n",
				 spm_read(SPM_POWER_ON_VAL0), spm_read(PCM_REG0_DATA));
			spm_write(SPM_POWER_ON_VAL0, spm_read(PCM_REG0_DATA));
		}
		/* disable r0 and r7 to control power */
		spm_write(PCM_PWR_IO_EN, 0);
		/* [Vcorefs] disable pcm timer after leaving FW */
		spm_write(PCM_CON1, SPM_REGWR_CFG_KEY | (spm_read(PCM_CON1) & ~PCM_TIMER_EN_LSB));
	}

	/* reset PCM */
	spm_write(PCM_CON0, SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB | PCM_SW_RESET_LSB);
	spm_write(PCM_CON0, SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB);
	if ((spm_read(PCM_FSM_STA) & 0x7fffff) != PCM_FSM_STA_DEF) {
		spm_crit2("reset pcm(PCM_FSM_STA=0x%x\n",spm_read(PCM_FSM_STA));
		BUG(); /* PCM reset failed */
	}

	/* init PCM_CON0 (disable event vector) */
	spm_write(PCM_CON0, SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB | EN_IM_SLEEP_DVS_LSB);

	/* init PCM_CON1 (disable PCM timer but keep PCM WDT setting) */
	con1 = spm_read(PCM_CON1) & (PCM_WDT_WAKE_MODE_LSB | PCM_WDT_EN_LSB);
	spm_write(PCM_CON1, con1 | SPM_REGWR_CFG_KEY | EVENT_LOCK_EN_LSB |
		  SPM_SRAM_ISOINT_B_LSB | SPM_SRAM_SLEEP_B_LSB |
		  (pcmdesc->replace ? 0 : IM_NONRP_EN_LSB) |
		  MIF_APBEN_LSB | SCP_APB_INTERNAL_EN_LSB);
}

void __spm_kick_im_to_fetch(const struct pcm_desc *pcmdesc)
{
	u32 ptr, len, con0;

	/* tell IM where is PCM code (use slave mode if code existed) */
	if (pcmdesc->base_dma) {
		ptr = pcmdesc->base_dma;
		/* for 4GB mode */
		if (enable_4G())
			MAPPING_DRAM_ACCESS_ADDR(ptr);
	} else {
		ptr = base_va_to_pa(pcmdesc->base);
	}
	len = pcmdesc->size - 1;
	if (spm_read(PCM_IM_PTR) != ptr || spm_read(PCM_IM_LEN) != len || pcmdesc->sess > 2) {
		spm_write(PCM_IM_PTR, ptr);
		spm_write(PCM_IM_LEN, len);
	} else {
		spm_write(PCM_CON1, spm_read(PCM_CON1) | SPM_REGWR_CFG_KEY | IM_SLAVE_LSB);
	}

	/* kick IM to fetch (only toggle IM_KICK) */
	con0 = spm_read(PCM_CON0) & ~(IM_KICK_L_LSB | PCM_KICK_L_LSB);
	spm_write(PCM_CON0, con0 | SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB | IM_KICK_L_LSB);
	spm_write(PCM_CON0, con0 | SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB);
}

void __spm_init_pcm_register(void)
{
	/* init r0 with POWER_ON_VAL0 */
	spm_write(PCM_REG_DATA_INI, spm_read(SPM_POWER_ON_VAL0));
	spm_write(PCM_PWR_IO_EN, PCM_RF_SYNC_R0);
	spm_write(PCM_PWR_IO_EN, 0);

	/* init r7 with POWER_ON_VAL1 */
	spm_write(PCM_REG_DATA_INI, spm_read(SPM_POWER_ON_VAL1));
	spm_write(PCM_PWR_IO_EN, PCM_RF_SYNC_R7);
	spm_write(PCM_PWR_IO_EN, 0);
}

void __spm_init_event_vector(const struct pcm_desc *pcmdesc)
{
	/* init event vector register */
	spm_write(PCM_EVENT_VECTOR0, pcmdesc->vec0);
	spm_write(PCM_EVENT_VECTOR1, pcmdesc->vec1);
	spm_write(PCM_EVENT_VECTOR2, pcmdesc->vec2);
	spm_write(PCM_EVENT_VECTOR3, pcmdesc->vec3);
	spm_write(PCM_EVENT_VECTOR4, pcmdesc->vec4);
	spm_write(PCM_EVENT_VECTOR5, pcmdesc->vec5);
	spm_write(PCM_EVENT_VECTOR6, pcmdesc->vec6);
	spm_write(PCM_EVENT_VECTOR7, pcmdesc->vec7);
	spm_write(PCM_EVENT_VECTOR8, pcmdesc->vec8);
	spm_write(PCM_EVENT_VECTOR9, pcmdesc->vec9);
	spm_write(PCM_EVENT_VECTOR10, pcmdesc->vec10);
	spm_write(PCM_EVENT_VECTOR11, pcmdesc->vec11);
	spm_write(PCM_EVENT_VECTOR12, pcmdesc->vec12);
	spm_write(PCM_EVENT_VECTOR13, pcmdesc->vec13);
	spm_write(PCM_EVENT_VECTOR14, pcmdesc->vec14);
	spm_write(PCM_EVENT_VECTOR15, pcmdesc->vec15);

	/* event vector will be enabled by PCM itself */
}

void __spm_set_power_control(const struct pwr_ctrl *pwrctrl)
{
	/* set other SYS request mask */
	spm_write(SPM_AP_STANDBY_CON, (!!pwrctrl->conn_apsrc_sel << 27) | (!!pwrctrl->conn_mask_b << 26) | (!!pwrctrl->md_apsrc0_sel << 25) | (!!pwrctrl->md_apsrc1_sel << 24) | (spm_read(SPM_AP_STANDBY_CON) & SRCCLKENI_MASK_B_LSB) |	/* bit23 */
		  (!!pwrctrl->lte_mask_b << 22) |
		  (!!pwrctrl->scp_req_mask_b << 21) |
		  (!!pwrctrl->md2_req_mask_b << 20) |
		  (!!pwrctrl->md1_req_mask_b << 19) |
		  (!!pwrctrl->md_ddr_dbc_en << 18) |
		  (!!pwrctrl->mcusys_idle_mask << 4) |
		  (!!pwrctrl->mp1top_idle_mask << 2) |
		  (!!pwrctrl->mp0top_idle_mask << 1) | (!!pwrctrl->wfi_op << 0));

	spm_write(SPM_SRC_REQ, (!!pwrctrl->cpu_md_dvfs_sop_force_on << 16) |
		  (!!pwrctrl->spm_flag_run_common_scenario << 10) |
		  (!!pwrctrl->spm_flag_dis_vproc_vsram_dvs << 9) |
		  (!!pwrctrl->spm_flag_keep_csyspwrupack_high << 8) |
		  (!!pwrctrl->spm_ddren_req << 7) |
		  (!!pwrctrl->spm_dvfs_force_down << 6) |
		  (!!pwrctrl->spm_dvfs_req << 5) |
		  (!!pwrctrl->spm_vrf18_req << 4) |
		  (!!pwrctrl->spm_infra_req << 3) |
		  (!!pwrctrl->spm_lte_req << 2) |
		  (!!pwrctrl->spm_f26m_req << 1) | (!!pwrctrl->spm_apsrc_req << 0));

	spm_write(SPM_SRC_MASK, (!!pwrctrl->conn_srcclkena_dvfs_req_mask_b << 31) |
		  (!!pwrctrl->md_srcclkena_1_dvfs_req_mask_b << 30) |
		  (!!pwrctrl->md_srcclkena_0_dvfs_req_mask_b << 29) |
		  (!!pwrctrl->emi_bw_dvfs_req_mask << 28) |
		  (!!pwrctrl->md_vrf18_req_1_mask_b << 24) |
		  (!!pwrctrl->md_vrf18_req_0_mask_b << 23) |
		  (!!pwrctrl->md_ddr_en_1_mask_b << 22) |
		  (!!pwrctrl->md_ddr_en_0_mask_b << 21) |
		  (!!pwrctrl->md32_apsrcreq_infra_mask_b << 20) |
		  (!!pwrctrl->conn_apsrcreq_infra_mask_b << 19) |
		  (!!pwrctrl->md_apsrcreq_1_infra_mask_b << 18) |
		  (!!pwrctrl->md_apsrcreq_0_infra_mask_b << 17) |
		  (!!pwrctrl->srcclkeni_infra_mask_b << 16) |
		  (!!pwrctrl->md32_srcclkena_infra_mask_b << 15) |
		  (!!pwrctrl->conn_srcclkena_infra_mask_b << 14) |
		  (!!pwrctrl->md_srcclkena_1_infra_mask_b << 13) |
		  (!!pwrctrl->md_srcclkena_0_infra_mask_b << 12) |
		  ((pwrctrl->vsync_mask_b & 0x1f) << 7) |
		  (!!pwrctrl->ccifmd_md2_event_mask_b << 6) |
		  (!!pwrctrl->ccifmd_md1_event_mask_b << 5) |
		  (!!pwrctrl->ccif1_to_ap_mask_b << 4) |
		  (!!pwrctrl->ccif1_to_md_mask_b << 3) |
		  (!!pwrctrl->ccif0_to_ap_mask_b << 2) | (!!pwrctrl->ccif0_to_md_mask_b << 1));

	spm_write(SPM_SRC2_MASK, (!!pwrctrl->cpu_md_emi_dvfs_req_prot_dis << 26) | (!!pwrctrl->emi_boost_dvfs_req_mask_b << 25) | (!!pwrctrl->sdio_on_dvfs_req_mask_b << 24) | (!!pwrctrl->l1_c2k_rccif_wake_mask_b << 23) | (!!pwrctrl->ps_c2k_rccif_wake_mask_b << 22) | (!!pwrctrl->c2k_l1_rccif_wake_mask_b << 21) | (!!pwrctrl->c2k_ps_rccif_wake_mask_b << 20) | (!!pwrctrl->mfg_req_mask_b << 19) | (!!pwrctrl->disp1_req_mask_b << 18) | (!!pwrctrl->disp_req_mask_b << 17) | (!!pwrctrl->conn_ddr_en_mask_b << 16) | ((pwrctrl->vsync_dvfs_halt_mask_b & 0x1f) << 11) |	/* 5bit */
		  (!!pwrctrl->md2_ddr_en_dvfs_halt_mask_b << 10) | (!!pwrctrl->md1_ddr_en_dvfs_halt_mask_b << 9) | (!!pwrctrl->cpu_md_dvfs_erq_merge_mask_b << 8) | (!!pwrctrl->gce_req_mask_b << 7) | (!!pwrctrl->vdec_req_mask_b << 6) | ((pwrctrl->dvfs_halt_mask_b & 0x1f) << 0));	/* 5bit */

	spm_write(SPM_CLK_CON, (spm_read(SPM_CLK_CON) & ~CC_SRCLKENA_MASK_0) |
		  (pwrctrl->srclkenai_mask ? CC_SRCLKENA_MASK_0 : 0));

	/* set CPU WFI mask */
	spm_write(MP1_CPU0_WFI_EN, !!pwrctrl->mp1_cpu0_wfi_en);
	spm_write(MP1_CPU1_WFI_EN, !!pwrctrl->mp1_cpu1_wfi_en);
	spm_write(MP1_CPU2_WFI_EN, !!pwrctrl->mp1_cpu2_wfi_en);
	spm_write(MP1_CPU3_WFI_EN, !!pwrctrl->mp1_cpu3_wfi_en);
	spm_write(MP0_CPU0_WFI_EN, !!pwrctrl->mp0_cpu0_wfi_en);
	spm_write(MP0_CPU1_WFI_EN, !!pwrctrl->mp0_cpu1_wfi_en);
	spm_write(MP0_CPU2_WFI_EN, !!pwrctrl->mp0_cpu2_wfi_en);
	spm_write(MP0_CPU3_WFI_EN, !!pwrctrl->mp0_cpu3_wfi_en);
}

void __spm_set_wakeup_event(const struct pwr_ctrl *pwrctrl)
{
	u32 val, mask, isr;

	/* set PCM timer (set to max when disable) */
	if (pwrctrl->timer_val_ramp_en != 0) {
		val = pcm_timer_ramp_max;

		pcm_timer_ramp_max++;

		if (pcm_timer_ramp_max >= 300)
			pcm_timer_ramp_max = 1;
	} else if (pwrctrl->timer_val_ramp_en_sec != 0) {
		val = pcm_timer_ramp_max * 1600;	/* 50ms */

		pcm_timer_ramp_max += 1;
		if (pcm_timer_ramp_max >= 300)	/* max 15 sec */
			pcm_timer_ramp_max = 1;

		pcm_timer_ramp_max_sec_loop++;
		if (pcm_timer_ramp_max_sec_loop >= 50) {
			pcm_timer_ramp_max_sec_loop = 0;
			/* range 6min to 10min */
			val = (pcm_timer_ramp_max + 300) * 32000;
		}
	} else {
		if (pwrctrl->timer_val_cust == 0)
			val = pwrctrl->timer_val ? : PCM_TIMER_MAX;
		else
			val = pwrctrl->timer_val_cust;
	}

	spm_write(PCM_TIMER_VAL, val);
	spm_write(PCM_CON1, spm_read(PCM_CON1) | SPM_REGWR_CFG_KEY | PCM_TIMER_EN_LSB);

	/* unmask AP wakeup source */
	if (pwrctrl->wake_src_cust == 0)
		mask = pwrctrl->wake_src;
	else
		mask = pwrctrl->wake_src_cust;

	if (pwrctrl->syspwreq_mask)
		mask &= ~WAKE_SRC_R12_CSYSPWREQ_B;
	spm_write(SPM_WAKEUP_EVENT_MASK, ~mask);

#if 0
	/* unmask MD32 wakeup source */
	spm_write(SPM_SLEEP_MD32_WAKEUP_EVENT_MASK, ~pwrctrl->wake_src_md32);
#endif

	/* unmask SPM ISR (keep TWAM setting) */
	isr = spm_read(SPM_IRQ_MASK) & SPM_TWAM_IRQ_MASK_LSB;
	spm_write(SPM_IRQ_MASK, isr | ISRM_RET_IRQ_AUX);
}

void __spm_kick_pcm_to_run(const struct pwr_ctrl *pwrctrl)
{
	u32 con0;

	/* init register to match PCM expectation */
	spm_write(SPM_MAS_PAUSE_MASK_B, 0xffffffff);
	spm_write(SPM_MAS_PAUSE2_MASK_B, 0xffffffff);
	spm_write(PCM_REG_DATA_INI, 0);

	/* set PCM flags and data */
	spm_write(SPM_SW_FLAG, pwrctrl->pcm_flags);
	spm_write(SPM_SW_RSV_0, pwrctrl->pcm_reserve);

	/* lock Infra DCM when PCM runs */
	spm_write(SPM_CLK_CON, (spm_read(SPM_CLK_CON) & ~SPM_LOCK_INFRA_DCM_LSB) |
		  (pwrctrl->infra_dcm_lock ? SPM_LOCK_INFRA_DCM_LSB : 0));

	/* enable r0 and r7 to control power */
	spm_write(PCM_PWR_IO_EN, (pwrctrl->r0_ctrl_en ? PCM_PWRIO_EN_R0 : 0) |
		  (pwrctrl->r7_ctrl_en ? PCM_PWRIO_EN_R7 : 0));

	/* kick PCM to run (only toggle PCM_KICK) */
	con0 = spm_read(PCM_CON0) & ~(IM_KICK_L_LSB | PCM_KICK_L_LSB);
	spm_write(PCM_CON0, con0 | SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB | PCM_KICK_L_LSB);
	spm_write(PCM_CON0, con0 | SPM_REGWR_CFG_KEY | PCM_CK_EN_LSB);
}

void __spm_get_wakeup_status(struct wake_status *wakesta)
{
	/* get PC value if PCM assert (pause abort) */
	wakesta->assert_pc = spm_read(PCM_REG_DATA_INI);

	/* get wakeup event */
	wakesta->r12 = spm_read(SPM_SW_RSV_0);
	wakesta->r12_ext = spm_read(PCM_REG12_EXT_DATA);
	wakesta->raw_sta = spm_read(SPM_WAKEUP_STA);
	wakesta->raw_ext_sta = spm_read(SPM_WAKEUP_EXT_STA);
	wakesta->wake_misc = spm_read(SPM_BSI_D0_SR);	/* backup of SLEEP_WAKEUP_MISC */

	/* get sleep time */
	wakesta->timer_out = spm_read(SPM_BSI_D1_SR);	/* backup of PCM_TIMER_OUT */

	/* get other SYS and co-clock status */
	wakesta->r13 = spm_read(PCM_REG13_DATA);
	wakesta->idle_sta = spm_read(SUBSYS_IDLE_STA);

	/* get debug flag for PCM execution check */
	wakesta->debug_flag = spm_read(SPM_SW_DEBUG);

	/* get special pattern (0xf0000 or 0x10000) if sleep abort */
	wakesta->event_reg = spm_read(SPM_BSI_D2_SR);	/* PCM_EVENT_REG_STA */

	/* get ISR status */
	wakesta->isr = spm_read(SPM_IRQ_STA);
}

void __spm_clean_after_wakeup(void)
{
	/* [Vcorefs] can not switch back to POWER_ON_VAL0 here,
	   the FW stays in VCORE DVFS which use r0 to Ctrl MEM */
	/* disable r0 and r7 to control power */
	/* spm_write(PCM_PWR_IO_EN, 0); */

	/* clean CPU wakeup event */
	spm_write(SPM_CPU_WAKEUP_EVENT, 0);

	/* [Vcorefs] not disable pcm timer here, due to the
	   following vcore dvfs will use it for latency check */
	/* clean PCM timer event */
	/* spm_write(PCM_CON1, SPM_REGWR_CFG_KEY | (spm_read(PCM_CON1) & ~PCM_TIMER_EN_LSB)); */

	/* clean wakeup event raw status (for edge trigger event) */
	spm_write(SPM_WAKEUP_EVENT_MASK, ~0);

	/* clean ISR status (except TWAM) */
	spm_write(SPM_IRQ_MASK, spm_read(SPM_IRQ_MASK) | ISRM_ALL_EXC_TWAM);
	spm_write(SPM_IRQ_STA, ISRC_ALL_EXC_TWAM);
	spm_write(SPM_SW_INT_CLEAR, PCM_SW_INT_ALL);
}

#define spm_print(suspend, fmt, args...)	\
do {						\
	if (!suspend)				\
		spm_debug(fmt, ##args);		\
	else					\
		spm_crit2(fmt, ##args);		\
} while (0)

wake_reason_t __spm_output_wake_reason(const struct wake_status *wakesta,
				       const struct pcm_desc *pcmdesc, bool suspend)
{
	int i;
	char buf[LOG_BUF_SIZE] = { 0 };
	wake_reason_t wr = WR_UNKNOWN;

	if (wakesta->assert_pc != 0) {
		/* add size check for vcoredvfs */
		spm_print(suspend, "PCM ASSERT AT %u (%s%s), r13 = 0x%x, debug_flag = 0x%x\n",
			  wakesta->assert_pc, (wakesta->assert_pc > pcmdesc->size) ? "NOT " : "",
			  pcmdesc->version, wakesta->r13, wakesta->debug_flag);
		return WR_PCM_ASSERT;
	}

	if (wakesta->r12 & WAKE_SRC_R12_PCM_TIMER) {
		if (wakesta->wake_misc & WAKE_MISC_PCM_TIMER) {
			strcat(buf, " PCM_TIMER");
			wr = WR_PCM_TIMER;
		}
		if (wakesta->wake_misc & WAKE_MISC_TWAM) {
			strcat(buf, " TWAM");
			wr = WR_WAKE_SRC;
		}
		if (wakesta->wake_misc & WAKE_MISC_CPU_WAKE) {
			strcat(buf, " CPU");
			wr = WR_WAKE_SRC;
		}
	}
	for (i = 1; i < 32; i++) {
		if (wakesta->r12 & (1U << i)) {
			strcat(buf, wakesrc_str[i]);
			wr = WR_WAKE_SRC;
#ifdef VENDOR_EDIT
//Wenxian.ZhEN@Prd.BaseDrv, 2016/05/24, Add for wake up source
			if ((suspend == true)&&(!(wakesta->r12 & WAKE_SRC_R12_EINT_EVENT_B)))
			wakesrc_count[i]++;
#endif /* VENDOR_EDIT */	
		}
	}
#ifdef VENDOR_EDIT
//Wenxian.ZhEN@Prd.BaseDrv, 2016/04/15, Add for wake up source
	if ((suspend == true)&&(!(wakesta->r12 & WAKE_SRC_R12_EINT_EVENT_B)))
	strcpy(wakeup_source_buf, buf);
#endif /* VENDOR_EDIT */		
	BUG_ON(strlen(buf) >= LOG_BUF_SIZE);

	spm_print(suspend, "wake up by%s, timer_out = %u, r13 = 0x%x, debug_flag = 0x%x\n",
		  buf, wakesta->timer_out, wakesta->r13, wakesta->debug_flag);

	spm_print(suspend,
		  "r12 = 0x%x, r12_ext = 0x%x, raw_sta = 0x%x, idle_sta = 0x%x, event_reg = 0x%x, isr = 0x%x\n",
		  wakesta->r12, wakesta->r12_ext, wakesta->raw_sta, wakesta->idle_sta,
		  wakesta->event_reg, wakesta->isr);

	spm_print(suspend, "raw_ext_sta = 0x%x, wake_misc = 0x%x, pcm_flag = 0x%x",
			wakesta->raw_ext_sta,
			wakesta->wake_misc,
			spm_read(SPM_SW_FLAG));
	return wr;
}

void __spm_dbgout_md_ddr_en(bool enable)
{
	/* set TEST_MODE_CFG */
	spm_write(0xf0000230, (spm_read(0xf0000230) & ~(0x7fff << 16)) |
		  (0x3 << 26) | (0x3 << 21) | (0x3 << 16));

	/* set md_ddr_en to GPIO150 */
	spm_write(0xf0001500, 0x70e);
	spm_write(0xf00057e4, 0x7);

	/* set emi_clk_off_req to GPIO140 */
	spm_write(0xf000150c, 0x3fe);
	spm_write(0xf00057c4, 0x7);

	/* enable debug output */
	spm_write(PCM_DEBUG_CON, !!enable);
}

unsigned int spm_get_cpu_pwr_status(void)
{
	unsigned int pwr_stat[2] = { 0 };
	unsigned int stat = 0;
	unsigned int cpu_mask = (CA15_CPU3 |
				 CA15_CPU2 |
				 CA15_CPU1 | CA15_CPU0 | CA7_CPU3 | CA7_CPU2 | CA7_CPU1 | CA7_CPU0);

	pwr_stat[0] = spm_read(PWR_STATUS);
	pwr_stat[1] = spm_read(PWR_STATUS_2ND);

	stat = (pwr_stat[0] & cpu_mask) & (pwr_stat[1] & cpu_mask);

	return stat;
}

void __spm_check_md_pdn_power_control(struct pwr_ctrl *pwr_ctrl)
{
	if (is_md_c2k_conn_power_off())
		pwr_ctrl->pcm_flags |= SPM_FLAG_DIS_MD_INFRA_PDN;
	else
		pwr_ctrl->pcm_flags &= ~SPM_FLAG_DIS_MD_INFRA_PDN;
}

void __spm_sync_vcore_dvfs_power_control(struct pwr_ctrl *dest_pwr_ctrl,
					 const struct pwr_ctrl *src_pwr_ctrl)
{
	/* pwr_ctrl for mask/ctrl register */
	dest_pwr_ctrl->dvfs_halt_mask_b = src_pwr_ctrl->dvfs_halt_mask_b;
	dest_pwr_ctrl->sdio_on_dvfs_req_mask_b = src_pwr_ctrl->sdio_on_dvfs_req_mask_b;
	dest_pwr_ctrl->cpu_md_dvfs_erq_merge_mask_b = src_pwr_ctrl->cpu_md_dvfs_erq_merge_mask_b;
	dest_pwr_ctrl->md1_ddr_en_dvfs_halt_mask_b = src_pwr_ctrl->md1_ddr_en_dvfs_halt_mask_b;
	dest_pwr_ctrl->md2_ddr_en_dvfs_halt_mask_b = src_pwr_ctrl->md2_ddr_en_dvfs_halt_mask_b;
	dest_pwr_ctrl->md_srcclkena_0_dvfs_req_mask_b =
	    src_pwr_ctrl->md_srcclkena_0_dvfs_req_mask_b;
	dest_pwr_ctrl->md_srcclkena_1_dvfs_req_mask_b =
	    src_pwr_ctrl->md_srcclkena_1_dvfs_req_mask_b;
	dest_pwr_ctrl->conn_srcclkena_dvfs_req_mask_b =
	    src_pwr_ctrl->conn_srcclkena_dvfs_req_mask_b;

	dest_pwr_ctrl->vsync_dvfs_halt_mask_b = src_pwr_ctrl->vsync_dvfs_halt_mask_b;
	dest_pwr_ctrl->emi_boost_dvfs_req_mask_b = src_pwr_ctrl->emi_boost_dvfs_req_mask_b;
	dest_pwr_ctrl->emi_bw_dvfs_req_mask = src_pwr_ctrl->emi_bw_dvfs_req_mask;
	dest_pwr_ctrl->cpu_md_emi_dvfs_req_prot_dis = src_pwr_ctrl->cpu_md_emi_dvfs_req_prot_dis;
	dest_pwr_ctrl->spm_dvfs_req = src_pwr_ctrl->spm_dvfs_req;
	dest_pwr_ctrl->spm_dvfs_force_down = src_pwr_ctrl->spm_dvfs_force_down;
	dest_pwr_ctrl->cpu_md_dvfs_sop_force_on = src_pwr_ctrl->cpu_md_dvfs_sop_force_on;
	dest_pwr_ctrl->dvfs_halt_src_chk = src_pwr_ctrl->dvfs_halt_src_chk;

	/* pwr_ctrl pcm_flag */
	if (src_pwr_ctrl->pcm_flags_cust != 0) {
		if ((src_pwr_ctrl->pcm_flags_cust & SPM_FLAG_DIS_VCORE_DVS) != 0)
			dest_pwr_ctrl->pcm_flags |= SPM_FLAG_DIS_VCORE_DVS;

		if ((src_pwr_ctrl->pcm_flags_cust & SPM_FLAG_DIS_VCORE_DFS) != 0)
			dest_pwr_ctrl->pcm_flags |= SPM_FLAG_DIS_VCORE_DFS;

		if ((src_pwr_ctrl->pcm_flags_cust & SPM_FLAG_EN_MET_DBG_FOR_VCORE_DVFS) != 0)
			dest_pwr_ctrl->pcm_flags |= SPM_FLAG_EN_MET_DBG_FOR_VCORE_DVFS;
	} else {
		if ((src_pwr_ctrl->pcm_flags & SPM_FLAG_DIS_VCORE_DVS) != 0)
			dest_pwr_ctrl->pcm_flags |= SPM_FLAG_DIS_VCORE_DVS;

		if ((src_pwr_ctrl->pcm_flags & SPM_FLAG_DIS_VCORE_DFS) != 0)
			dest_pwr_ctrl->pcm_flags |= SPM_FLAG_DIS_VCORE_DFS;

		if ((src_pwr_ctrl->pcm_flags & SPM_FLAG_EN_MET_DBG_FOR_VCORE_DVFS) != 0)
			dest_pwr_ctrl->pcm_flags |= SPM_FLAG_EN_MET_DBG_FOR_VCORE_DVFS;
	}

}

#define MM_DVFS_DISP1_HALT_MASK 0x1
#define MM_DVFS_DISP0_HALT_MASK 0x2
#define MM_DVFS_ISP_HALT_MASK   0x4
#define MM_DVFS_GCE_HALT_MASK   0x10

int __check_dvfs_halt_source(int enable)
{
	u32 val, orig_val;
	bool is_halt = 1;

	val = spm_read(SPM_SRC2_MASK);
	orig_val = val;

	if (enable == 0) {
		pr_err("[VcoreFS]dvfs_halt_src_chk is disabled\n");
		return 0;
	}

	pr_err("[VcoreFS]SRC2_MASK=0x%x\n",val);
	if ((spm_read(CPU_DVFS_REQ) & DVFS_HALT_LSB) == 0) {
		is_halt = 0;
		pr_err("[VcoreFS]dvfs_halt already clear!(%d, 0x%x)\n", is_halt, spm_read(CPU_DVFS_REQ));
		aee_kernel_exception_api(__FILE__, __LINE__, \
			 DB_OPT_DEFAULT | DB_OPT_MMPROFILE_BUFFER | DB_OPT_DISPLAY_HANG_DUMP | DB_OPT_DUMP_DISPLAY,
			 "DVFS_HALT_UNKNOW", "DVFS_HALT_UNKNOW");
	}
	pr_err("[VcoreFS]halt_status(1)=0x%x\n", spm_read(CPU_DVFS_REQ));
	if ((val & MM_DVFS_ISP_HALT_MASK) && (is_halt)) {
		pr_err("[VcoreFS]isp_halt[0]:src2_mask=0x%x r6=0x%x r15=0x%x\n",
				val, spm_read(PCM_REG6_DATA), spm_read(PCM_REG15_DATA));
		spm_write(SPM_SRC2_MASK, (val & ~MM_DVFS_ISP_HALT_MASK));
		udelay(50);
		pr_err("[VcoreFS]isp_halt[1]:src2_mask=0x%x r6=0x%x r15=0x%x\n",
				spm_read(SPM_SRC2_MASK), spm_read(PCM_REG6_DATA), spm_read(PCM_REG15_DATA));
		if ((spm_read(CPU_DVFS_REQ) & DVFS_HALT_LSB) == 0) {
			aee_kernel_exception_api(__FILE__, __LINE__, \
			 DB_OPT_DEFAULT, "DVFS_HALT_ISP", "DVFS_HALT_ISP");
			is_halt = 0;
			pr_err("[VcoreFS]dvfs_halt is hold by ISP (%d, 0x%x)\n", is_halt, spm_read(CPU_DVFS_REQ));
		}
	}

	pr_err("[VcoreFS]halt_status(2)=0x%x\n", spm_read(CPU_DVFS_REQ));
	val = spm_read(SPM_SRC2_MASK);
	if ((val & MM_DVFS_DISP1_HALT_MASK) && (is_halt)) {
		pr_err("[VcoreFS]disp1_halt[0]:src2_mask=0x%x r6=0x%x r15=0x%x\n",
				 val, spm_read(PCM_REG6_DATA), spm_read(PCM_REG15_DATA));
		spm_write(SPM_SRC2_MASK, (val & ~MM_DVFS_DISP1_HALT_MASK));
		udelay(50);
		pr_err("[VcoreFS]disp1_halt[1]:src2_mask=0x%x r6=0x%x r15=0x%x\n",
				spm_read(SPM_SRC2_MASK), spm_read(PCM_REG6_DATA), spm_read(PCM_REG15_DATA));
		if ((spm_read(CPU_DVFS_REQ) & DVFS_HALT_LSB) == 0) {
			aee_kernel_exception_api(__FILE__, __LINE__, \
			 DB_OPT_DEFAULT | DB_OPT_MMPROFILE_BUFFER | DB_OPT_DISPLAY_HANG_DUMP | DB_OPT_DUMP_DISPLAY,
			 "DVFS_HALT_DISP1", "DVFS_HALT_DISP1");
			primary_display_diagnose();
			is_halt = 0;
			pr_err("[VcoreFS]dvfs_halt is hold by DISP1 (%d, 0x%x)\n", is_halt, spm_read(CPU_DVFS_REQ));
		}
	}

	pr_err("[VcoreFS]halt_status(3)=0x%x\n", spm_read(CPU_DVFS_REQ));
	val = spm_read(SPM_SRC2_MASK);
	if ((val & MM_DVFS_DISP0_HALT_MASK) && (is_halt)) {
		pr_err("[VcoreFS]disp0_halt[0]:src2_mask=0x%x r6=0x%x r15=0x%x\n",
				 val, spm_read(PCM_REG6_DATA), spm_read(PCM_REG15_DATA));
		spm_write(SPM_SRC2_MASK, (val & ~MM_DVFS_DISP0_HALT_MASK));
		udelay(50);
		pr_err("[VcoreFS]disp0_halt[1]:src2_mask=0x%x r6=0x%x r15=0x%x\n",
				spm_read(SPM_SRC2_MASK), spm_read(PCM_REG6_DATA), spm_read(PCM_REG15_DATA));
		if ((spm_read(CPU_DVFS_REQ) & DVFS_HALT_LSB) == 0) {
			aee_kernel_exception_api(__FILE__, __LINE__, \
			 DB_OPT_DEFAULT | DB_OPT_MMPROFILE_BUFFER | DB_OPT_DISPLAY_HANG_DUMP | DB_OPT_DUMP_DISPLAY,
			 "DVFS_HALT_DISP0", "DVFS_HALT_DISP0");
			primary_display_diagnose();
			is_halt = 0;
			pr_err("[VcoreFS]dvfs_halt is hold by DISP0 (%d, 0x%x)\n", is_halt, spm_read(CPU_DVFS_REQ));
		}
	}

	pr_err("[VcoreFS]halt_status(4)=0x%x\n", spm_read(CPU_DVFS_REQ));
	val = spm_read(SPM_SRC2_MASK);
	if ((val & MM_DVFS_GCE_HALT_MASK) && (is_halt)) {
		pr_err("[VcoreFS]gce_halt[0]:src2_mask=0x%x r6=0x%x r15=0x%x\n",
				val, spm_read(PCM_REG6_DATA), spm_read(PCM_REG15_DATA));
		spm_write(SPM_SRC2_MASK, (val & ~MM_DVFS_GCE_HALT_MASK));
		udelay(50);
		pr_err("[VcoreFS]gce_halt[1]:src2_mask=0x%x r6=0x%x r15=0x%x\n",
				spm_read(SPM_SRC2_MASK), spm_read(PCM_REG6_DATA), spm_read(PCM_REG15_DATA));
		if ((spm_read(CPU_DVFS_REQ) & DVFS_HALT_LSB) == 0) {
			aee_kernel_exception_api(__FILE__, __LINE__, \
			 DB_OPT_DEFAULT, "DVFS_HALT_GCE", "DVFS_HALT_GCE");
			is_halt = 0;
			pr_err("[VcoreFS]dvfs_halt is hold by GCE (%d, 0x%x)\n", is_halt, spm_read(CPU_DVFS_REQ));
		}
	}

	udelay(1000); /* change to 1 ms for slow dvs slope FW */
	spm_write(SPM_SRC2_MASK, orig_val);
	pr_err("[VcoreFS]restore src_mask=0x%x, r6=0x%x r15=0x%x\n",
			spm_read(SPM_SRC2_MASK), spm_read(PCM_REG6_DATA), spm_read(PCM_REG15_DATA));

	/* BUG(); */

	return 0;
}

#ifdef CONFIG_OF
static int dt_scan_memory(unsigned long node, const char *uname, int depth, void *data)
{
	char *type = of_get_flat_dt_prop(node, "device_type", NULL);
	__be32 *reg, *endp;
	unsigned long l;
	u32 rank0_addr, rank1_addr, dram_rank_num;
	dram_info_t *dram_info = NULL;

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


	if (node) {
		/* orig_dram_info */
		dram_info = (dram_info_t *) of_get_flat_dt_prop(node, "orig_dram_info", NULL);
	}

	dram_rank_num = dram_info->rank_num;
	rank0_addr = dram_info->rank_info[0].start;
	if (dram_rank_num == 1)
		rank1_addr = rank0_addr;
	else
		rank1_addr = dram_info->rank_info[1].start;
	spm_crit("dram_rank_num: %d\n", dram_rank_num);
	spm_crit("dummy read addr: rank0=0x%x rank1=0x%x\n", rank0_addr, rank1_addr);
	spm_write(SPM_PASR_DPD_1, rank0_addr);
	spm_write(SPM_PASR_DPD_2, rank1_addr);

	return node;
}
#endif

void spm_set_dummy_read_addr(void)
{
#ifdef CONFIG_OF
	int node;

	node = of_scan_flat_dt(dt_scan_memory, NULL);
#else
	pr_err("no rank info\n");
	BUG();
	/* return false; */
#endif
}

bool is_md_c2k_conn_power_off(void)
{
	u32 md1_pwr_con = 0;
	u32 c2k_pwr_con = 0;
	u32 conn_pwr_con = 0;

	md1_pwr_con = spm_read(MD1_PWR_CON);
	c2k_pwr_con = spm_read(C2K_PWR_CON);
	conn_pwr_con = spm_read(CONN_PWR_CON);

#if 0
	pr_err("md1_pwr_con = 0x%08x, c2k_pwr_con = 0x%08x, conn_pwr_con = 0x%08x\n",
	       md1_pwr_con, c2k_pwr_con, conn_pwr_con);
#endif

	if (!((md1_pwr_con & 0x1F) == 0x12))
		return false;

	if (!((c2k_pwr_con & 0x1F) == 0x12))
		return false;

	if (!((conn_pwr_con & 0x1F) == 0x12))
		return false;

	return true;
}

static u32 pmic_rg_auxadc_ck_pdn_hwen;
static u32 pmic_rg_efuse_ck_pdn;

void __spm_backup_pmic_ck_pdn(void)
{
	/* PMIC setting 2015/07/31 by Chia-Lin/Kev */
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_read_interface_nolock(PMIC_CLK_AUXADC_CK_PDN_HWEN_ADDR,
				   &pmic_rg_auxadc_ck_pdn_hwen,
				   PMIC_CLK_AUXADC_CK_PDN_HWEN_MASK,
				   PMIC_CLK_AUXADC_CK_PDN_HWEN_SHIFT);
	pmic_config_interface_nolock(PMIC_CLK_AUXADC_CK_PDN_HWEN_ADDR,
				     0,
				     PMIC_CLK_AUXADC_CK_PDN_HWEN_MASK,
				     PMIC_CLK_AUXADC_CK_PDN_HWEN_SHIFT);

	pmic_read_interface_nolock(PMIC_CLK_EFUSE_CK_PDN_ADDR,
				   &pmic_rg_efuse_ck_pdn,
				   PMIC_CLK_EFUSE_CK_PDN_MASK,
				   PMIC_CLK_EFUSE_CK_PDN_SHIFT);
	pmic_config_interface_nolock(PMIC_CLK_EFUSE_CK_PDN_ADDR,
				     1,
				     PMIC_CLK_EFUSE_CK_PDN_MASK,
				     PMIC_CLK_EFUSE_CK_PDN_SHIFT);
#else
	pmic_read_interface_nolock(MT6351_PMIC_RG_AUXADC_CK_PDN_HWEN_ADDR,
				   &pmic_rg_auxadc_ck_pdn_hwen,
				   MT6351_PMIC_RG_AUXADC_CK_PDN_HWEN_MASK,
				   MT6351_PMIC_RG_AUXADC_CK_PDN_HWEN_SHIFT);
	pmic_config_interface_nolock(MT6351_PMIC_RG_AUXADC_CK_PDN_HWEN_ADDR,
				     0,
				     MT6351_PMIC_RG_AUXADC_CK_PDN_HWEN_MASK,
				     MT6351_PMIC_RG_AUXADC_CK_PDN_HWEN_SHIFT);

	pmic_read_interface_nolock(MT6351_PMIC_RG_EFUSE_CK_PDN_ADDR,
				   &pmic_rg_efuse_ck_pdn,
				   MT6351_PMIC_RG_EFUSE_CK_PDN_MASK,
				   MT6351_PMIC_RG_EFUSE_CK_PDN_SHIFT);
	pmic_config_interface_nolock(MT6351_PMIC_RG_EFUSE_CK_PDN_ADDR,
				     1,
				     MT6351_PMIC_RG_EFUSE_CK_PDN_MASK,
				     MT6351_PMIC_RG_EFUSE_CK_PDN_SHIFT);
#endif
}

void __spm_restore_pmic_ck_pdn(void)
{
	/* PMIC setting 2015/07/31 by Chia-Lin/Kev */
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_config_interface_nolock(PMIC_CLK_AUXADC_CK_PDN_HWEN_ADDR,
				     pmic_rg_auxadc_ck_pdn_hwen,
				     PMIC_CLK_AUXADC_CK_PDN_HWEN_MASK,
				     PMIC_CLK_AUXADC_CK_PDN_HWEN_SHIFT);

	pmic_config_interface_nolock(PMIC_CLK_EFUSE_CK_PDN_ADDR,
				     pmic_rg_efuse_ck_pdn,
				     PMIC_CLK_EFUSE_CK_PDN_MASK,
				     PMIC_CLK_EFUSE_CK_PDN_SHIFT);
#else
	pmic_config_interface_nolock(MT6351_PMIC_RG_AUXADC_CK_PDN_HWEN_ADDR,
				     pmic_rg_auxadc_ck_pdn_hwen,
				     MT6351_PMIC_RG_AUXADC_CK_PDN_HWEN_MASK,
				     MT6351_PMIC_RG_AUXADC_CK_PDN_HWEN_SHIFT);

	pmic_config_interface_nolock(MT6351_PMIC_RG_EFUSE_CK_PDN_ADDR,
				     pmic_rg_efuse_ck_pdn,
				     MT6351_PMIC_RG_EFUSE_CK_PDN_MASK,
				     MT6351_PMIC_RG_EFUSE_CK_PDN_SHIFT);
#endif
}

void __spm_bsi_top_init_setting(void)
{
		/* BSI_TOP init setting */
		spm_write(spm_bsi1cfg + 0x2004, 0x8000A824);
		spm_write(spm_bsi1cfg + 0x2010, 0x20001201);
		spm_write(spm_bsi1cfg + 0x2014, 0x150b0000);
		spm_write(spm_bsi1cfg + 0x2020, 0x0e001841);
		spm_write(spm_bsi1cfg + 0x2024, 0x150b0000);
		spm_write(spm_bsi1cfg + 0x2030, 0x1);
}

void __spm_pmic_pg_force_on(void)
{
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_config_interface_nolock(PMIC_STRUP_DIG_IO_PG_FORCE_ADDR,
			0x1,
			PMIC_STRUP_DIG_IO_PG_FORCE_MASK,
			PMIC_STRUP_DIG_IO_PG_FORCE_SHIFT);
	pmic_config_interface_nolock(PMIC_RG_STRUP_VIO18_PG_ENB_ADDR,
			0x1,
			PMIC_RG_STRUP_VIO18_PG_ENB_MASK,
			PMIC_RG_STRUP_VIO18_PG_ENB_SHIFT);
#else
	pmic_config_interface_nolock(MT6351_PMIC_STRUP_DIG_IO_PG_FORCE_ADDR,
			0x1,
			MT6351_PMIC_STRUP_DIG_IO_PG_FORCE_MASK,
			MT6351_PMIC_STRUP_DIG_IO_PG_FORCE_SHIFT);
	pmic_config_interface_nolock(MT6351_PMIC_RG_STRUP_VIO18_PG_ENB_ADDR,
			0x1,
			MT6351_PMIC_RG_STRUP_VIO18_PG_ENB_MASK,
			MT6351_PMIC_RG_STRUP_VIO18_PG_ENB_SHIFT);
#endif
}

void __spm_pmic_pg_force_off(void)
{
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_config_interface_nolock(PMIC_STRUP_DIG_IO_PG_FORCE_ADDR,
			0x0,
			PMIC_STRUP_DIG_IO_PG_FORCE_MASK,
			PMIC_STRUP_DIG_IO_PG_FORCE_SHIFT);
	pmic_config_interface_nolock(PMIC_RG_STRUP_VIO18_PG_ENB_ADDR,
			0x0,
			PMIC_RG_STRUP_VIO18_PG_ENB_MASK,
			PMIC_RG_STRUP_VIO18_PG_ENB_SHIFT);
#else
	pmic_config_interface_nolock(MT6351_PMIC_STRUP_DIG_IO_PG_FORCE_ADDR,
			0x0,
			MT6351_PMIC_STRUP_DIG_IO_PG_FORCE_MASK,
			MT6351_PMIC_STRUP_DIG_IO_PG_FORCE_SHIFT);
	pmic_config_interface_nolock(MT6351_PMIC_RG_STRUP_VIO18_PG_ENB_ADDR,
			0x0,
			MT6351_PMIC_RG_STRUP_VIO18_PG_ENB_MASK,
			MT6351_PMIC_RG_STRUP_VIO18_PG_ENB_SHIFT);
#endif
}

MODULE_DESCRIPTION("SPM-Internal Driver v0.1");
