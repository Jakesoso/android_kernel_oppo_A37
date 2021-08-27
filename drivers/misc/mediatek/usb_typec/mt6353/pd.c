#include <linux/kthread.h>

#include "typec.h"
#include "pd.h"

#if SUPPORT_PD

////////////////////////////////////////////////////////////////////////////

#define PD_INTR_EN_0_MSK (REG_PD_TX_DONE_INTR_EN | REG_PD_TX_RETRY_ERR_INTR_EN | REG_PD_TX_RCV_NEW_MSG_DISCARD_MSG_INTR_EN | \
	REG_PD_TX_PHY_LAYER_RST_DISCARD_MSG_INTR_EN | REG_PD_TX_DIS_BUS_REIDLE_INTR_EN | REG_PD_TX_CRC_RCV_TIMEOUT_INTR_EN | \
	REG_PD_TX_AUTO_SR_DONE_INTR_EN | REG_PD_TX_AUTO_SR_RETRY_ERR_INTR_EN | REG_PD_TX_AUTO_SR_RCV_NEW_MSG_DISCARD_MSG_INTR_EN | \
	REG_PD_TX_AUTO_SR_PHY_LAYER_RST_DISCARD_MSG_INTR_EN | REG_PD_RX_RCV_MSG_INTR_EN | REG_PD_RX_LENGTH_MIS_INTR_EN | \
	REG_PD_RX_DUPLICATE_INTR_EN)

#define PD_INTR_EN_1_MSK (REG_PD_HR_TRANS_CPL_TIMEOUT_INTR_EN | REG_PD_HR_TRANS_DONE_INTR_EN | REG_PD_HR_RCV_DONE_INTR_EN | \
	REG_PD_HR_TRANS_FAIL_INTR_EN | REG_PD_TIMER0_TIMEOUT_INTR_EN | REG_PD_TIMER1_TIMEOUT_INTR_EN)

void pd_int_enable(struct typec_hba *hba)
{
	typec_writew(hba, PD_INTR_EN_0_MSK, PD_INTR_EN_0);
	typec_writew(hba, PD_INTR_EN_1_MSK, PD_INTR_EN_1);
}

void pd_int_disable(struct typec_hba *hba)
{
	typec_writew(hba, 0, PD_INTR_EN_0);
	typec_writew(hba, 0, PD_INTR_EN_1);
}

void pd_set_default_mode(struct typec_hba *hba)
{
	//PD setting
	typec_set(hba, TYPE_C_SW_PD_EN, TYPE_C_CC_SW_CTRL);
}

void pd_set_default_param(struct typec_hba *hba)
{
	uint32_t val;


	//Set TX parameters
	typec_writew_msk(hba, REG_PD_TX_HALF_UI_CYCLE_CNT, (19<<REG_PD_TX_HALF_UI_CYCLE_CNT_OFST), 
		PD_TX_PARAMETER);	


	//Set RX parameters
	typec_set(hba, REG_PD_RX_EN, PD_RX_PARAMETER);	
	typec_set(hba, REG_PD_RX_SOP_PRIME_RCV_EN, PD_RX_PARAMETER);
	typec_set(hba, (REG_PD_RX_SOP_DPRIME_RCV_EN | REG_PD_RX_CABLE_RST_RCV_EN), PD_RX_PARAMETER);	

	typec_writew(hba, 12000, PD_CRC_RCV_TIMEOUT_VAL_0);
	typec_writew(hba, (12000>>16), PD_CRC_RCV_TIMEOUT_VAL_1);	
	typec_writew(hba, 60000, PD_HR_COMPLETE_TIMEOUT_VAL_0);
	typec_writew(hba, (60000>>16), PD_HR_COMPLETE_TIMEOUT_VAL_1);
	typec_writew(hba, 143, PD_IDLE_DETECTION_0);
	typec_writew(hba, 299, PD_INTERFRAMEGAP_VAL);	
}

static void pd_dump_intr(struct typec_hba *hba, uint16_t pd_is0, uint16_t pd_is1)
{
	int i;
 	
	//PD_INTR_1
	
	struct bit_mapping pd_is0_mapping[] = {
		{PD_RX_DUPLICATE_INTR, "PD_RX_DUPLICATE_INTR"},
		{PD_RX_LENGTH_MIS_INTR, "PD_RX_LENGTH_MIS_INTR"},
		{PD_RX_RCV_MSG_INTR, "PD_RX_RCV_MSG_INTR"},
		{PD_TX_AUTO_SR_PHY_LAYER_RST_DISCARD_MSG_INTR, "PD_TX_AUTO_SR_PHY_LAYER_RST_DISCARD_MSG_INTR"},
		{PD_TX_AUTO_SR_RCV_NEW_MSG_DISCARD_MSG_INTR, "PD_TX_AUTO_SR_RCV_NEW_MSG_DISCARD_MSG_INTR"},
		{PD_TX_AUTO_SR_RETRY_ERR_INTR, "PD_TX_AUTO_SR_RETRY_ERR_INTR"},
		{PD_TX_AUTO_SR_DONE_INTR, "PD_TX_AUTO_SR_DONE_INTR"},
		{PD_TX_CRC_RCV_TIMEOUT_INTR, "PD_TX_CRC_RCV_TIMEOUT_INTR"},
		{PD_TX_DIS_BUS_REIDLE_INTR, "PD_TX_DIS_BUS_REIDLE_INTR"},
		{PD_TX_PHY_LAYER_RST_DISCARD_MSG_INTR, PD_TX_PHY_LAYER_RST_DISCARD_MSG_INTR},
		{PD_TX_RCV_NEW_MSG_DISCARD_MSG_INTR, "PD_TX_RCV_NEW_MSG_DISCARD_MSG_INTR"},
		{PD_TX_RETRY_ERR_INTR, "PD_TX_RETRY_ERR_INTR"},
		{PD_TX_DONE_INTR, "PD_TX_DONE_INTR"},		
 	};
	struct bit_mapping pd_is1_mapping[] = { 		
		{PD_TIMER1_TIMEOUT_INTR, "PD_TIMER1_TIMEOUT_INTR"},
		{PD_TIMER0_TIMEOUT_INTR, "PD_TIMER0_TIMEOUT_INTR"},
		{PD_CR_TRANS_FAIL_INTR, "PD_CR_TRANS_FAIL_INTR"},
		{PD_CR_TRANS_DONE_INTR, "PD_CR_TRANS_DONE_INTR"},
		{PD_HR_TRANS_FAIL_INTR, "PD_HR_TRANS_FAIL_INTR"},
		{PD_HR_RCV_DONE_INTR, "PD_HR_RCV_DONE_INTR"},		
		{PD_HR_TRANS_DONE_INTR, "PD_HR_TRANS_DONE_INTR"},
		{PD_HR_TRANS_CPL_TIMEOUT_INTR, "PD_HR_TRANS_CPL_TIMEOUT_INTR"},		
	};
	struct bit_mapping pd_is0_mapping_dbg[] = {
	};
	struct bit_mapping pd_is1_mapping_dbg[] = {
	};
	

	for (i = 0; i < sizeof(pd_is0_mapping)/sizeof(struct bit_mapping); i++)
	{
		if (pd_is0 & pd_is0_mapping[i].mask)
			dev_err(hba->dev, "%s\n", pd_is0_mapping[i].name);
	}

	for (i = 0; i < sizeof(pd_is1_mapping)/sizeof(struct bit_mapping); i++)
	{
		if (pd_is1 & pd_is1_mapping[i].mask)
			dev_err(hba->dev, "%s\n", pd_is1_mapping[i].name);
	}

	if (hba->dbg_lvl >= TYPEC_DBG_LVL_2)
	{
		for (i = 0; i < sizeof(pd_is0_mapping_dbg)/sizeof(struct bit_mapping); i++)
		{
			if (pd_is0 & pd_is0_mapping_dbg[i].mask)
				dev_err(hba->dev, "%s\n", pd_is0_mapping_dbg[i].name);
		}

		for (i = 0; i < sizeof(pd_is1_mapping_dbg)/sizeof(struct bit_mapping); i++)
		{
			if (pd_is1 & pd_is1_mapping_dbg[i].mask)
				dev_err(hba->dev, "%s\n", pd_is1_mapping_dbg[i].name);
		}
	}
}

static void pd_dump_objs(struct typec_hba *hba, unsigned int addr)
{
	int i;
	
	for (i = 0; i < MAX_PD_OBJS; i++)
	{
		dev_err(hba->dev, "object %d: \n", typec_readw(hba, addr+i*2+1)<<16 | typec_readw(hba, addr+i*2));
	}
}

/**
 * ufshcd_pd_intr - Interrupt service routine
 * @hba: per adapter instance
 * @pd_is0: PD interrupt status 0
 * @pd_is1: PD interrupt status 1
 */
void pd_intr(struct typec_hba *hba, uint16_t pd_is0, uint16_t pd_is1)
{
	int i;


	//dump interrupt information
	if (hba->dbg_lvl >= TYPEC_DBG_LVL_1)
		pd_dump_intr(hba, pd_is0, pd_is1);


	if (pd_is0 & PD_RX_RCV_MSG_INTR)
	{
		pd_dump_objs(hba, PD_RX_DATA_OBJECT0_0);
	}
}

#endif
