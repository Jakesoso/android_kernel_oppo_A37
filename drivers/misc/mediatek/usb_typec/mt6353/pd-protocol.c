#include "typec.h"

#if SUPPORT_PD
#include "usb_pd_func.h"

////////////////////////////////////////////////////////////////////////////

extern int typec_is_vbus_present(struct typec_hba *hba, enum enum_vbus_lvl lvl);
extern void typec_vbus_present(struct typec_hba *hba, uint8_t enable);
extern void typec_vbus_det_enable(struct typec_hba *hba, uint8_t enable);
extern void typec_drive_vconn(struct typec_hba *hba, uint8_t enable);
extern void typec_int_enable(struct typec_hba *hba, uint16_t msk0, uint16_t msk2);
extern void typec_int_disable(struct typec_hba *hba, uint16_t msk0, uint16_t msk2);
int pd_task(void *data);

////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_USB_PD_DUAL_ROLE
#define DUAL_ROLE_IF_ELSE(hba, sink_clause, src_clause) \
	(hba->power_role == PD_ROLE_SINK ? (sink_clause) : (src_clause))
#else
#define DUAL_ROLE_IF_ELSE(hba, sink_clause, src_clause) (src_clause)
#endif

#define READY_RETURN_STATE(hba) DUAL_ROLE_IF_ELSE(hba, PD_STATE_SNK_READY, \
							 PD_STATE_SRC_READY)

/* Type C supply voltage (mV) */
#define TYPE_C_VOLTAGE	5000 /* mV */

/* PD counter definitions */
#define PD_MESSAGE_ID_COUNT 7
#define PD_HARD_RESET_COUNT 2
#define PD_CAPS_COUNT 50
#define PD_SNK_CAP_RETRIES 3

#ifdef CONFIG_USB_PD_DUAL_ROLE
/* Last received source cap */
static uint32_t pd_src_caps[PDO_MAX_OBJECTS];
static int pd_src_cap_cnt;
#endif

////////////////////////////////////////////////////////////////////////////

//for debug purpose
STATIC struct typec_hba *pd_hba = NULL;

static struct os_mapping {
	uint16_t type;
	char name[MAX_SIZE];
} pd_os_type_mapping[] = {
	{PD_TX_SOP, "SOP"},
	{PD_TX_SOP_PRIME, "SOP'"},
	{PD_TX_SOP_PRIME_PRIME, "SOP''"},
	{PD_TX_HARD_RESET, "HARD_RESET"},
	{PD_TX_CABLE_RESET, "CABLE_RESET"},
	{PD_TX_SOP_DEBUG_PRIME, "DBG_SOP'"},
	{PD_TX_SOP_DEBUG_PRIME_PRIME, "DBG_SPO''"},
};

static struct state_mapping {
	uint16_t state;
	char name[MAX_SIZE];
} pd_state_mapping[] = {
	{PD_STATE_DISABLED, "DISABLED"},
	{PD_STATE_SUSPENDED, "SUSPENDED"},
#ifdef CONFIG_USB_PD_DUAL_ROLE
	{PD_STATE_SNK_UNATTACH, "SNK_UNATTACH"},
	{PD_STATE_SNK_ATTACH, "SNK_ATTACH"},
	{PD_STATE_SNK_HARD_RESET_RECOVER, "SNK_HARD_RESET_RECOVER"},
	{PD_STATE_SNK_DISCOVERY, "SNK_DISCOVERY"},
	{PD_STATE_SNK_REQUESTED, "SNK_REQUESTED"},
	{PD_STATE_SNK_TRANSITION, "SNK_TRANSITION"},
	{PD_STATE_SNK_READY, "SNK_READY"},
	{PD_STATE_SNK_SWAP_INIT, "SNK_SWAP_INIT"},
	{PD_STATE_SNK_SWAP_SNK_DISABLE, "SNK_SWAP_SNK_DISABLE"},
	{PD_STATE_SNK_SWAP_SRC_DISABLE, "SNK_SWAP_SRC_DISABLE"},
	{PD_STATE_SNK_SWAP_STANDBY, "SNK_SWAP_STANDBY"},
	{PD_STATE_SNK_SWAP_COMPLETE, "SNK_SWAP_COMPLETE"},
#endif /* CONFIG_USB_PD_DUAL_ROLE */
	{PD_STATE_SRC_UNATTACH, "SRC_UNATTACH"},
	{PD_STATE_SRC_ATTACH, "SRC_ATTACH"},
	{PD_STATE_SRC_HARD_RESET_RECOVER, "SRC_HARD_RESET_RECOVER"},
	{PD_STATE_SRC_STARTUP, "SRC_STARTUP"},
	{PD_STATE_SRC_DISCOVERY, "SRC_DISCOVERY"},
	{PD_STATE_SRC_NEGOCIATE, "SRC_NEGOCIATE"},
	{PD_STATE_SRC_ACCEPTED, "SRC_ACCEPTED"},
	{PD_STATE_SRC_POWERED, "SRC_POWERED"},
	{PD_STATE_SRC_TRANSITION, "SRC_TRANSITION"},
	{PD_STATE_SRC_READY, "SRC_READY"},
	{PD_STATE_SRC_GET_SINK_CAP, "SRC_GET_SINK_CAP"},
	{PD_STATE_DR_SWAP, "DR_SWAP"},
#ifdef CONFIG_USB_PD_DUAL_ROLE
	{PD_STATE_SRC_SWAP_INIT, "SRC_SWAP_INIT"},
	{PD_STATE_SRC_SWAP_SNK_DISABLE, "SRC_SWAP_SNK_DISABLE"},
	{PD_STATE_SRC_SWAP_SRC_DISABLE, "SRC_SWAP_SRC_DISABLE"},
	{PD_STATE_SRC_SWAP_STANDBY, "SRC_SWAP_STANDBY"},
#ifdef CONFIG_USBC_VCONN_SWAP
	{PD_STATE_VCONN_SWAP_SEND, "VCONN_SWAP_SEND"},
	{PD_STATE_VCONN_SWAP_INIT, "VCONN_SWAP_INIT"},
	{PD_STATE_VCONN_SWAP_READY, "VCONN_SWAP_READY"},
#endif /* CONFIG_USBC_VCONN_SWAP */
#endif /* CONFIG_USB_PD_DUAL_ROLE */
	{PD_STATE_SOFT_RESET, "SOFT_RESET"},
	{PD_STATE_HARD_RESET_SEND, "HARD_RESET_SEND"},
	{PD_STATE_HARD_RESET_EXECUTE, "HARD_RESET_EXECUTE"},
#ifdef CONFIG_COMMON_RUNTIME
	{PD_STATE_BIST_CMD, "BIST_CMD"},
	{PD_STATE_BIST_CARRIER_MODE_2, "BIST_CARRIER_MODE_2"},
	{PD_STATE_BIST_TEST_DATA, "BIST_TEST_DATA"},
#endif
	{PD_STATE_NO_TIMEOUT, "NO_TIMEOUT"},
};

struct bit_mapping pd_is0_mapping[] =
{
	//RX events
	{PD_RX_TRANS_GCRC_FAIL_INTR, "RX_TRANS_GCRC_FAIL"},
	{PD_RX_DUPLICATE_INTR, "RX_DUPLICATE"},
	{PD_RX_LENGTH_MIS_INTR, "RX_LENGTH_MIS"},
	{PD_RX_RCV_MSG_INTR, "RX_RCV_MSG"},
	//TX events
	{PD_TX_AUTO_SR_PHY_LAYER_RST_DISCARD_MSG_INTR, "TX_AUTO_SR_PHY_LAYER_RST_DISCARD_MSG"},
	{PD_TX_AUTO_SR_RCV_NEW_MSG_DISCARD_MSG_INTR, "TX_AUTO_SR_RCV_NEW_MSG_DISCARD_MSG"},
	{PD_TX_AUTO_SR_RETRY_ERR_INTR, "TX_AUTO_SR_RETRY_ERR"},
	{PD_TX_AUTO_SR_DONE_INTR, "TX_AUTO_SR_DONE"},
	{PD_TX_CRC_RCV_TIMEOUT_INTR, "TX_CRC_RCV_TIMEOUT"},
	{PD_TX_DIS_BUS_REIDLE_INTR, "TX_DIS_BUS_REIDLE"},
	{PD_TX_PHY_LAYER_RST_DISCARD_MSG_INTR, "TX_PHY_LAYER_RST_DISCARD_MSG"},
	{PD_TX_RCV_NEW_MSG_DISCARD_MSG_INTR, "TX_RCV_NEW_MSG_DISCARD_MSG"},
	{PD_TX_RETRY_ERR_INTR, "TX_RETRY_ERR"},
	{PD_TX_DONE_INTR, "TX_DONE"},
};

struct bit_mapping pd_is1_mapping[] =
{
	//{PD_TIMER1_TIMEOUT_INTR, "TIMER1_TIMEOUT"},
	//{PD_TIMER0_TIMEOUT_INTR, "TIMER0_TIMEOUT"},
	{PD_AD_PD_CC2_OVP_INTR, "AD_PD_CC2_OVP"},
	{PD_AD_PD_CC1_OVP_INTR, "AD_PD_CC1_OVP"},
	{PD_AD_PD_VCONN_UVP_INTR, "AD_PD_VCONN_UVP"},
	{PD_HR_TRANS_FAIL_INTR, "HR_TRANS_FAIL"},
	//{PD_HR_RCV_DONE_INTR, "HR_RCV_DONE"},
	//{PD_HR_TRANS_DONE_INTR, "HR_TRANS_DONE"},
	{PD_HR_TRANS_CPL_TIMEOUT_INTR, "HR_TRANS_CPL_TIMEOUT"},
};

struct bit_mapping pd_is0_mapping_dbg[] =
{
	//{PD_RX_RCV_MSG_INTR, "RX_RCV_MSG"},
	//{PD_TX_DONE_INTR, "TX_DONE"},
};

struct bit_mapping pd_is1_mapping_dbg[] =
{
	//{PD_TIMER1_TIMEOUT_INTR, "TIMER1_TIMEOUT"},
	{PD_TIMER0_TIMEOUT_INTR, "TIMER0_TIMEOUT"},
	{PD_HR_RCV_DONE_INTR, "HR_RCV_DONE"},
	{PD_HR_TRANS_DONE_INTR, "HR_TRANS_DONE"},
};

////////////////////////////////////////////////////////////////////////////

STATIC void pd_dump_intr(struct typec_hba *hba, uint16_t pd_is0, uint16_t pd_is1)
{
	int i;


	if (hba->dbg_lvl >= TYPEC_DBG_LVL_1)
	{
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
	}

	if (hba->dbg_lvl >= TYPEC_DBG_LVL_3)
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

////////////////////////////////////////////////////////////////////////////

//4.1x
#define PD_HZ_FACTOR_DIVIDEND 10
#define PD_HZ_FACTOR_DIVIDER 41

unsigned long pd_msecs_to_jiffies(unsigned long ms_timeout)
{
	unsigned long ret;
	unsigned long dividend = HZ * ms_timeout * PD_HZ_FACTOR_DIVIDEND;
	unsigned long divider = 1000 * PD_HZ_FACTOR_DIVIDER;


	//round to the nearest with minimum value 1
	ret = ((dividend + divider / 2) / divider);
	if (ret == 0)
		ret = 1;


	return ret;
}

#if PD_SW_WORKAROUND2
STATIC void pd_timer0_start(struct typec_hba *hba, unsigned long ms_timeout)
{
	uint32_t val = PD_REF_CK_MS * ms_timeout;

	typec_writew(hba, val, PD_TIMER0_VAL_0);
	typec_writew(hba, (val>>16), PD_TIMER0_VAL_1);

	//ONLY 0->1 transition triggers timer to work
	typec_set(hba, PD_TIMER0_EN, PD_TIMER0_ENABLE);
}

STATIC void pd_timer0_stop(struct typec_hba *hba)
{
	//disable timer because ONLY 0->1 transition triggers timer
	typec_clear(hba, PD_TIMER0_EN, PD_TIMER0_ENABLE);

	//clear the corresponding interrupt status
	typec_set(hba, PD_TIMER0_TIMEOUT_INTR, PD_INTR_1);
}
#endif

#if PD_SW_WORKAROUND3
#define time_after_16(a,b)     \
    (typecheck(uint16_t, a) && \
     typecheck(uint16_t, b) && \
     ((int16_t)(b) - (int16_t)(a) < 0))

STATIC uint16_t pd_get_jiffies(struct typec_hba *hba)
{
	return typec_readw(hba, PD_TIMER1_TICK_CNT);
}
#endif

////////////////////////////////////////////////////////////////////////////

STATIC void pd_rx_enable(struct typec_hba *hba, uint8_t enable)
{
	if ((typec_readw(hba, PD_RX_PARAMETER) & REG_PD_RX_EN) ^ enable)
	{
		typec_writew_msk(hba, REG_PD_RX_EN, (enable ? REG_PD_RX_EN : 0), PD_RX_PARAMETER);

		if (hba->dbg_lvl >= TYPEC_DBG_LVL_2)
			dev_err(hba->dev, "RX %s\n", (enable ? "ON" : "OFF"));
	}
}

STATIC void pd_bist_enable(struct typec_hba *hba, uint8_t enable, uint32_t mode)
{
	if (enable)
	{
		if (mode == BDO_MODE_CARRIER2)
			typec_set(hba, PD_TX_BIST_CARRIER_MODE2_START, PD_TX_CTRL);
		hba->bist_mode = mode;
	}
	else
	{
		typec_clear(hba, PD_TX_BIST_CARRIER_MODE2_START, PD_TX_CTRL);
		hba->bist_mode = 0;
	}
}

////////////////////////////////////////////////////////////////////////////

STATIC void pd_int_enable(struct typec_hba *hba, uint8_t enable)
{
	if (enable)
	{
		typec_writew(hba, PD_INTR_EN_0_MSK, PD_INTR_EN_0);
		typec_writew(hba, PD_INTR_EN_1_MSK, PD_INTR_EN_1);
	}
	else
	{
		typec_writew(hba, 0, PD_INTR_EN_0);
		typec_writew(hba, 0, PD_INTR_EN_1);
	}
}

STATIC void pd_set_default_param(struct typec_hba *hba)
{
	//set TX options
	typec_clear(hba, (REG_PD_TX_AUTO_SEND_SR_EN | REG_PD_TX_AUTO_SEND_HR_EN | REG_PD_TX_AUTO_SEND_CR_EN),
		PD_TX_PARAMETER);

	//set TX parameters
	typec_writew_msk(hba, REG_PD_TX_HALF_UI_CYCLE_CNT, (19<<REG_PD_TX_HALF_UI_CYCLE_CNT_OFST),
		PD_TX_PARAMETER);


	//set RX parameters
	typec_writew_msk(hba, REG_PD_RX_PRE_PROTECT_HALF_UI_CYCLE_CNT_MIN,
		(9<<REG_PD_RX_PRE_PROTECT_HALF_UI_CYCLE_CNT_MIN_OFST), PD_RX_PREAMBLE_PROTECT_PARAMETER_0);
	typec_writew_msk(hba, REG_PD_RX_PRE_PROTECT_UI_CYCLE_CNT_MAX,
		(54<<REG_PD_RX_PRE_PROTECT_UI_CYCLE_CNT_MAX_OFST), PD_RX_PREAMBLE_PROTECT_PARAMETER_2);
	typec_writew(hba, 10800, PD_CRC_RCV_TIMEOUT_VAL_0);
	typec_writew(hba, (10800>>16), PD_CRC_RCV_TIMEOUT_VAL_1);
	typec_writew(hba, 60000, PD_HR_COMPLETE_TIMEOUT_VAL_0);
	typec_writew(hba, (60000>>16), PD_HR_COMPLETE_TIMEOUT_VAL_1);
	typec_writew(hba, 143, PD_IDLE_DETECTION_0);
	typec_writew(hba, 480, PD_INTERFRAMEGAP_VAL);
	typec_writew(hba, 8, PD_RX_GLITCH_MASK_WINDOW);


	//set timer parameters
	#if PD_SW_WORKAROUND2
	typec_writew(hba, PD_REF_CK_MS, PD_TIMER0_VAL_0);
	typec_writew(hba, (PD_REF_CK_MS>>16), PD_TIMER0_VAL_1);
	#endif
	#if PD_SW_WORKAROUND3
	typec_writew(hba, PD_REF_CK_MS, PD_TIMER1_VAL);
	typec_set(hba, PD_TIMER1_EN, PD_TIMER1_ENABLE);
	#endif
}

void pd_init(struct typec_hba *hba)
{
	//SW
	pd_hba = hba;
	hba->pd_comm_enabled = 1;


	//HW
	pd_set_default_param(hba);
	//turn on PD module but leave PD-RX disabled
	typec_set(hba, TYPE_C_SW_PD_EN, TYPE_C_CC_SW_CTRL);
	pd_rx_enable(hba, 0);
	pd_int_enable(hba, 1);


	//start PD task
	kthread_run(pd_task, (void *)hba, "main_thread\n");
}

////////////////////////////////////////////////////////////////////////////

STATIC void pd_set_event(struct typec_hba *hba, uint16_t pd_is0, uint16_t pd_is1, uint16_t cc_is0)
{
	hba->pd_is0 |= pd_is0;
	hba->pd_is1 |= pd_is1;
	hba->cc_is0 |= cc_is0;
}

STATIC void pd_clear_event(struct typec_hba *hba, uint16_t pd_is0, uint16_t pd_is1, uint16_t cc_is0)
{
	hba->pd_is0 &= ~pd_is0;
	hba->pd_is1 &= ~pd_is1;
	hba->cc_is0 &= ~cc_is0;
}

/**
 * ufshcd_pd_intr - Interrupt service routine
 * @hba: per adapter instance
 * @pd_is0: PD interrupt status 0
 * @pd_is1: PD interrupt status 1
 * @cc_is0: CC interrupt status 0
 * @cc_is2: CC interrupt status 2
 */
#if PD_SW_WORKAROUND1_1
extern void pd_get_message(struct typec_hba *hba, uint16_t *header, uint32_t *payload);
#endif
void pd_intr(struct typec_hba *hba, uint16_t pd_is0, uint16_t pd_is1, uint16_t cc_is0, uint16_t cc_is2)
{
	int i;
	uint16_t cc_event;
	uint16_t tx_event;
	uint16_t rx_event;
	uint16_t timer_event;


	//dump interrupt information
	pd_dump_intr(hba, pd_is0, pd_is1);

  	//leave handling to main loop
	pd_set_event(hba, pd_is0, pd_is1, cc_is0);


#if PD_SW_WORKAROUND1_1
	if (pd_is0 & PD_RX_RCV_MSG_INTR)
		pd_get_message(hba, &hba->header, hba->payload);
#endif


	//event type
	tx_event = ((pd_is0 & PD_TX_EVENTS0_LISTEN) || (pd_is1 & PD_TX_EVENTS1_LISTEN));
	rx_event = ((pd_is0 & PD_RX_EVENTS0_LISTEN) || (pd_is1 & PD_RX_EVENTS1_LISTEN));
	cc_event = (cc_is0 & PD_INTR_IS0_LISTEN);
	timer_event = (pd_is1 & PD_TIMER0_TIMEOUT_INTR);
	typec_sw_probe(hba, DBG_INTR_TX_EVENT, (tx_event ? DBG_INTR_TX_EVENT : 0));
	typec_sw_probe(hba, DBG_INTR_RX_EVENT, (rx_event ? DBG_INTR_RX_EVENT : 0));
	typec_sw_probe(hba, DBG_INTR_CC_EVENT, (cc_event ? DBG_INTR_CC_EVENT : 0));
	typec_sw_probe(hba, DBG_INTR_TIMER_EVENT, (timer_event ? DBG_INTR_TIMER_EVENT : 0));

	//TX events
	if (tx_event)
		complete(&hba->tx_event);

	//CC & RX & timer events
	if (cc_event || rx_event || timer_event)
		complete(&hba->event);
}

////////////////////////////////////////////////////////////////////////////

STATIC inline void pd_complete_hr(struct typec_hba *hba)
{
	typec_set(hba, W1_PD_PE_HR_CPL, PD_HR_CTRL);
}

STATIC void pd_set_msg_id(struct typec_hba *hba, uint8_t msg_id)
{
	int i;


	for (i = PD_TX_SOP; i <= PD_TX_SOP_PRIME_PRIME; i++)
	{
		typec_writew_msk(hba, REG_PD_TX_OS, (i<<REG_PD_TX_OS_OFST), PD_TX_CTRL);
		typec_writew_msk(hba, REG_PD_SW_MSG_ID, msg_id, PD_MSG_ID_SW_MODE);
		typec_set(hba, PD_SW_MSG_ID_SYNC, PD_MSG_ID_SW_MODE);
	}
}

STATIC inline void set_state_timeout(struct typec_hba *hba,
	unsigned long ms_timeout, enum pd_states timeout_state)
{
	#if PD_SW_WORKAROUND3
	hba->timeout_jiffies = pd_get_jiffies(hba) + ms_timeout;
	#else
	hba->timeout_jiffies = jiffies + pd_msecs_to_jiffies(ms_timeout);
	#endif
	hba->timeout_ms = ms_timeout;
	hba->timeout_state = timeout_state;

	if ((hba->dbg_lvl >= TYPEC_DBG_LVL_1) && (timeout_state != PD_STATE_NO_TIMEOUT))
	{
		#if PD_SW_WORKAROUND3
		dev_err(hba->dev, "[TIMEOUT2 %dms] %s", ms_timeout, pd_state_mapping[timeout_state].name);
		#else
		dev_err(hba->dev, "[TIMEOUT2 %d(%d)ms] %s",
			(pd_msecs_to_jiffies(ms_timeout) * PD_HZ_FACTOR_DIVIDER), ms_timeout,
			pd_state_mapping[timeout_state].name);
		#endif
	}
}

STATIC inline int state_changed(struct typec_hba *hba)
{
	return (hba->last_state != hba->task_state);
}

STATIC void set_state(struct typec_hba *hba, enum pd_states next_state)
{
	set_state_timeout(hba, 0, PD_STATE_NO_TIMEOUT);
	hba->task_state = next_state;

	typec_sw_probe(hba, DBG_PD_STATE, (next_state<<DBG_PD_STATE_OFST));

#if 0
#ifdef CONFIG_USB_PD_DUAL_ROLE
	if (next_state == PD_STATE_SRC_UNATTACH ||
		next_state == PD_STATE_SNK_UNATTACH ||
		next_state == PD_STATE_DISABLED)
	{
		/* Clear the input current limit */
		pd_set_input_current_limit(hba, 0, 0);
#ifdef CONFIG_CHARGE_MANAGER
		typec_set_input_current_limit(hba, 0, 0);
		charge_manager_set_ceil(hba,
					CEIL_REQUESTOR_PD,
					CHARGE_CEIL_NONE);
#endif
#ifdef CONFIG_USBC_VCONN
		typec_drive_vconn(hba, 0);
#endif
#else /* CONFIG_USB_PD_DUAL_ROLE */
	if (next_state == PD_STATE_SRC_UNATTACH ||
		next_state == PD_STATE_DISABLED)
	{
#endif
		//hba->dev_id = 0;
		hba->flags &= ~PD_FLAGS_RESET_ON_DISCONNECT_MASK;
#ifdef CONFIG_CHARGE_MANAGER
		charge_manager_update_dualrole(hba, CAP_UNKNOWN);
#endif
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
		pd_dfp_exit_mode(hba, 0, 0);
#endif
#ifdef CONFIG_USBC_SS_MUX
		usb_mux_set(hba, TYPEC_MUX_NONE, USB_SWITCH_DISCONNECT,
			    hba->polarity);
#endif

		/* Disable TCPC RX */
		pd_rx_enable(hba, 0);
	}
#endif
}

STATIC int pd_transmit(struct typec_hba *hba, enum pd_transmit_type type,
		       uint16_t header, const uint32_t *data)
{
	int i;
	int ret = 1;
	uint8_t cnt;
	uint16_t pd_is0, pd_is1;
	unsigned long flags;


	#if !PD_DVT
	//if comms are disabled, do not transmit, return error
	if (!hba->pd_comm_enabled)
		return 1;
	#endif


	//reception ordered set
	typec_writew_msk(hba, REG_PD_TX_OS, (type<<REG_PD_TX_OS_OFST), PD_TX_CTRL);

	//prepare header
	cnt = PD_HEADER_CNT(header);
	typec_writew(hba, header, PD_TX_HEADER);

	//mask off message id because it's controlled by HW
	if (hba->dbg_lvl >= TYPEC_DBG_LVL_3)
		dev_err(hba->dev, "TX (%s) %04x/%d ",
			pd_os_type_mapping[type].name, (header &~ PD_HEADER_ID_MSK), cnt);

	//prepare data
	for (i = 0; i < cnt; i++)
	{
		typec_writew(hba, data[i], (PD_TX_DATA_OBJECT0_0+i*4));
		typec_writew(hba, (data[i]>>16), (PD_TX_DATA_OBJECT0_1+i*4));

		if (hba->dbg_lvl >= TYPEC_DBG_LVL_3)
			dev_err(hba->dev, "[%d]%08x ", i, data[i]);
	}


	//report unhandled TX events before sending
	if (hba->pd_is0 & PD_TX_EVENTS0 || hba->pd_is1 & PD_TX_EVENTS1)
		dev_err(hba->dev, "%s unhandled events pd_is0: %x, pd_is1: %x", __func__,
			hba->pd_is0 & PD_TX_EVENTS0, hba->pd_is1 & PD_TX_EVENTS1);


	//send message
	init_completion(&hba->tx_event);
	typec_set(hba, PD_TX_START, PD_TX_CTRL);


	if (wait_for_completion_timeout(&hba->tx_event, pd_msecs_to_jiffies(PD_TX_TIMEOUT)))
	{
		//clean up TX events
		spin_lock_irqsave(&hba->typec_lock, flags);
		pd_is0 = hba->pd_is0;
		pd_is1 = hba->pd_is1;

		pd_clear_event(hba, (pd_is0 & PD_TX_EVENTS0), (pd_is1 & PD_TX_EVENTS1), 0);
		spin_unlock_irqrestore(&hba->typec_lock, flags);


		if (pd_is1 & PD_TX_HR_SUCCESS)
			pd_complete_hr(hba);

		//no problem
		if ((pd_is0 & PD_TX_SUCCESS0) | (pd_is1 & PD_TX_SUCCESS1))
			ret = 0;
		else if ((pd_is0 & PD_TX_FAIL0) | (pd_is1 & PD_TX_FAIL1))
			ret = 1;
	}
	else //timeout
	{
		ret = 1;
	}


	return ret;
}

STATIC void pd_update_roles(struct typec_hba *hba)
{
	typec_writew_msk(hba, (REG_PD_TX_HDR_PORT_POWER_ROLE | REG_PD_TX_HDR_PORT_DATA_ROLE),
		(hba->power_role<<REG_PD_TX_HDR_PORT_POWER_ROLE_OFST | hba->data_role<<REG_PD_TX_HDR_PORT_DATA_ROLE_OFST),
		PD_TX_HEADER);
}

STATIC int send_control(struct typec_hba *hba, enum pd_ctrl_msg_type type)
{
	int ret;
	uint16_t header;

	header = PD_HEADER(type, hba->power_role, hba->data_role, 0);
	ret = pd_transmit(hba, PD_TX_SOP, header, NULL);

	return ret;
}

STATIC int send_source_cap(struct typec_hba *hba)
{
	int i;
	int ret;
	uint16_t header;

	//flag for DRP
	for (i = 0; i < pd_src_pdo_cnt; i++)
	{
		if ((pd_src_pdo[i] & PDO_TYPE_MASK) == PDO_TYPE_FIXED)
		{
			if (hba->support_role == TYPEC_ROLE_DRP)
				pd_src_pdo[i] |= PDO_FIXED_DUAL_ROLE;
			else
				pd_src_pdo[i] &= ~PDO_FIXED_DUAL_ROLE;
		}
	}

	if (pd_src_pdo_cnt == 0)
		/* No source capabilities defined, sink only */
		header = PD_HEADER(PD_CTRL_REJECT, hba->power_role, hba->data_role, 0);
	else
		header = PD_HEADER(PD_DATA_SOURCE_CAP, hba->power_role, hba->data_role, pd_src_pdo_cnt);
	ret = pd_transmit(hba, PD_TX_SOP, header, pd_src_pdo);


	return ret;
}

#ifdef CONFIG_USB_PD_DUAL_ROLE
STATIC void send_sink_cap(struct typec_hba *hba)
{
	int i;
	int ret;
	uint16_t header;


	//flag for DRP
	for (i = 0; i < pd_snk_pdo_cnt; i++)
	{
		if ((pd_snk_pdo[i] & PDO_TYPE_MASK) == PDO_TYPE_FIXED)
		{
			if (hba->support_role == TYPEC_ROLE_DRP)
				pd_snk_pdo[i] |= PDO_FIXED_DUAL_ROLE;
			else
				pd_snk_pdo[i] &= ~PDO_FIXED_DUAL_ROLE;
		}
	}


	header = PD_HEADER(PD_DATA_SINK_CAP, hba->power_role, hba->data_role, pd_snk_pdo_cnt);
	ret = pd_transmit(hba, PD_TX_SOP, header, pd_snk_pdo);


	return ret;
}

STATIC int send_request(struct typec_hba *hba, uint32_t rdo)
{
	int ret;
	uint16_t header;


	header = PD_HEADER(PD_DATA_REQUEST, hba->power_role, hba->data_role, 1);
	ret = pd_transmit(hba, PD_TX_SOP, header, &rdo);


	return ret;
}
#endif /* CONFIG_USB_PD_DUAL_ROLE */

#ifdef CONFIG_COMMON_RUNTIME
STATIC int send_bist_cmd(struct typec_hba *hba, uint32_t mode)
{
	uint32_t bdo;
	int ret;
	uint16_t header;


	if (mode == BDO_MODE_CARRIER2 || mode == BDO_MODE_TEST_DATA)
		bdo = BDO(mode, 0);
	else
		return 1;

	header = PD_HEADER(PD_DATA_BIST, hba->power_role, hba->data_role, 1);
	ret = pd_transmit(hba, PD_TX_SOP, header, &bdo);


	return ret;
}
#endif

#if 0
STATIC void queue_vdm(struct typec_hba *hba, uint32_t *header, const uint32_t *data,
			     int data_cnt)
{
	hba->vdo_count = data_cnt + 1;
	hba->vdo_data[0] = header[0];
	memcpy(&hba->vdo_data[1], data, sizeof(uint32_t) * data_cnt);
	/* Set ready, pd task will actually send */
	hba->vdm_state = VDM_STATE_READY;
}

STATIC void handle_vdm_request(struct typec_hba *hba, int cnt, uint32_t *payload)
{
	int rlen = 0;
	uint32_t *rdata;

	if (hba->vdm_state == VDM_STATE_BUSY) {
		/* If UFP responded busy retry after timeout */
		if (PD_VDO_CMDT(payload[0]) == CMDT_RSP_BUSY) {
			hba->vdm_timeout.val = get_time().val +
				PD_T_VDM_BUSY;
			hba->vdm_state = VDM_STATE_WAIT_RSP_BUSY;
			hba->vdo_retry = (payload[0] & ~VDO_CMDT_MASK) |
				CMDT_INIT;
			return;
		} else {
			hba->vdm_state = VDM_STATE_DONE;
		}
	}

	if (PD_VDO_SVDM(payload[0]))
		rlen = pd_svdm(hba, cnt, payload, &rdata);
	else
		rlen = pd_custom_vdm(hba, cnt, payload, &rdata);

	if (rlen > 0) {
		queue_vdm(hba, rdata, &rdata[1], rlen - 1);
		return;
	}
	if (debug_level >= 1)
		CPRINTF("Unhandled VDM VID %04x CMD %04x\n",
			PD_VDO_VID(payload[0]), payload[0] & 0xFFFF);
}
#endif

#ifdef CONFIG_USB_PD_DUAL_ROLE
STATIC void pd_set_power_role(struct typec_hba *hba, uint8_t role, uint8_t update_hdr)
{
	uint8_t cur_role;
	uint16_t tmp;


	/* header */
	if (update_hdr)
	{
		hba->power_role = role;
		pd_update_roles(hba);

		//Ignores PING for SNK
		//related compliance test items: TD.PD.LL.E6 Ping & TD.PD.SRC.E14 Atomic Message Sequence
		if (role == PD_ROLE_SINK)
			typec_clear(hba, REG_PD_RX_PING_MSG_RCV_EN, PD_RX_PARAMETER);
		else
			typec_set(hba, REG_PD_RX_PING_MSG_RCV_EN, PD_RX_PARAMETER);
	}

	/* termination */
	//this register ONLY updates when entering SRC_ATTACH or SNK_ATTACH state; 0 SNK, 1 SRC
	cur_role = ((typec_readw(hba, TYPE_C_PWR_STATUS) & RO_TYPE_C_CC_PWR_ROLE) >> 4);

	//toggle PR when necessary
	if (cur_role != role)
	{
		//setting this changes termination and generates Attached.SNK->Attached.SRC or Attached.SRC->Attached.SNK event
		//we don't have to serve generated event in the middle of power role swap
		typec_set(hba, W1_TYPE_C_SW_PR_SWAP_INDICATE_CMD, TYPE_C_CC_SW_CTRL);

		//TYPEC controller syncs register with Khz frequency that is much smaller than PD controller (MHz)
		//Wait a while for this to take effect, before proceeding to PD actions
		tmp = (role == TYPEC_ROLE_SOURCE) ? TYPEC_STATE_ATTACHED_SRC : TYPEC_STATE_ATTACHED_SNK;
		while ((typec_readw(hba, TYPE_C_CC_STATUS) & RO_TYPE_C_CC_ST) != tmp);
		msleep(1);

		if (hba->dbg_lvl >= TYPEC_DBG_LVL_2)
			dev_err(hba->dev, "Switch to %s", (role == TYPEC_ROLE_SOURCE) ? "Rp" : "Rd");
	}
}
#endif

void pd_execute_hard_reset(struct typec_hba *hba)
{
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	pd_dfp_exit_mode(hba, 0, 0);
#endif

	/*
	 * Fake set last state to hard reset to make sure that the next
	 * state to run knows that we just did a hard reset.
	 */
	hba->last_state = PD_STATE_HARD_RESET_EXECUTE;

#ifdef CONFIG_USB_PD_DUAL_ROLE
	/*
	 * If we are swapping to a source and have changed to Rp, restore back
	 * to Rd and turn off vbus to match our power_role.
	 */
	if (hba->task_state == PD_STATE_SNK_SWAP_STANDBY ||
	    hba->task_state == PD_STATE_SNK_SWAP_COMPLETE)
	{
		pd_set_power_role(hba, PD_ROLE_SINK, 1);
		pd_power_supply_reset(hba);
	}

	if (hba->power_role == PD_ROLE_SINK)
	{
		/* Clear the input current limit */
		pd_set_input_current_limit(hba, 0, 0);
#ifdef CONFIG_CHARGE_MANAGER
		charge_manager_set_ceil(hba,
					CEIL_REQUESTOR_PD,
					CHARGE_CEIL_NONE);
#endif /* CONFIG_CHARGE_MANAGER */

		typec_vbus_det_enable(hba, 0);
		set_state(hba, PD_STATE_SNK_HARD_RESET_RECOVER);

		return;
	}
#endif /* CONFIG_USB_PD_DUAL_ROLE */

	/* We are a source, cut power */
	pd_power_supply_reset(hba);
	#if PD_SW_WORKAROUND3
	hba->src_recover = pd_get_jiffies(hba) + PD_T_SRC_RECOVER;
	#else
	hba->src_recover = jiffies + pd_msecs_to_jiffies(PD_T_SRC_RECOVER);
	#endif
	set_state(hba, PD_STATE_SRC_HARD_RESET_RECOVER);
}

STATIC void execute_soft_reset(struct typec_hba *hba)
{
	//go back to DISCOVERY state for explicit contract negotiation
	hba->flags &=~ PD_FLAGS_EXPLICIT_CONTRACT;
	set_state(hba, DUAL_ROLE_IF_ELSE(hba, PD_STATE_SNK_DISCOVERY,
						PD_STATE_SRC_DISCOVERY));

#if 0//def CONFIG_COMMON_RUNTIME
	/* if flag to disable PD comms after soft reset, then disable comms */
	if (hba->flags & PD_FLAGS_SFT_RST_DIS_COMM)
		pd_rx_enable(hba, 0);
#endif
}

#if 0
void pd_soft_reset(void)
{
	int i;

	for (i = 0; i < CONFIG_USB_PD_PORT_COUNT; ++i)
		if (pd_is_connected(i)) {
			set_state(i, PD_STATE_SOFT_RESET);
			task_wake(PD_PORT_TO_TASK_ID(i));
		}
}

void pd_prepare_reset(void)
{
	int i;

	/*
	 * On reset, we are most definitely going to drop pings (if any)
	 * and lose all of our PD state. Instead of trying to remember all
	 * the states and deal with on-going transmission, let's send soft
	 * reset here and then disable PD communication until after sysjump
	 * is complete so that the communication starts over without dropping
	 * power.
	 */
	for (i = 0; i < CONFIG_USB_PD_PORT_COUNT; ++i)
		if (pd_is_connected(i))
			pd[i].flags |= PD_FLAGS_SFT_RST_DIS_COMM;

	pd_soft_reset();

	/* Give time for soft reset to be sent */
	usleep(pd_msecs_to_jiffies(8));
}
#endif

#ifdef CONFIG_USB_PD_DUAL_ROLE
STATIC void pd_store_src_cap(struct typec_hba *hba, int cnt, uint32_t *src_caps)
{
	int i;

	//cap
	pd_src_cap_cnt = cnt;
	for (i = 0; i < cnt; i++)
	{
		pd_src_caps[i] = *src_caps;
		src_caps++;
	}
}

STATIC void pd_send_request_msg(struct typec_hba *hba, int always_send_request)
{
	uint32_t rdo, curr_limit, supply_voltage;
	int ret;

#ifdef CONFIG_CHARGE_MANAGER
	int charging = (charge_manager_get_active_charge_port() == port);
#else
	const int charging = 1;
#endif

#ifdef CONFIG_USB_PD_CHECK_MAX_REQUEST_ALLOWED
	int max_request_allowed = pd_is_max_request_allowed();
#else
	//const int max_request_allowed = 1; //try VSAFE5V first, CC
	const int max_request_allowed = 0;
#endif

	/* Clear new power request */
	hba->new_power_request = 0;

	/* Build and send request RDO */
	/*
	 * If this port is not actively charging or we are not allowed to
	 * request the max voltage, then select vSafe5V
	 */
	ret = pd_build_request(pd_src_cap_cnt, pd_src_caps,
			       &rdo, &curr_limit, &supply_voltage,
			       charging && max_request_allowed ?
					PD_REQUEST_MAX : PD_REQUEST_VSAFE5V);

	if (ret)
		/*
		 * If fail to choose voltage, do nothing, let source re-send
		 * source cap
		 */
		return;

	/* Don't re-request the same voltage */
	if (!always_send_request && hba->prev_request_mv == supply_voltage)
		return;


	if (hba->dbg_lvl >= TYPEC_DBG_LVL_3)
	{
		dev_err(hba->dev, "Req [%d] %dmV %dmA", RDO_POS(rdo), supply_voltage, curr_limit);
		if (rdo & RDO_CAP_MISMATCH)
			dev_err(hba->dev, " Mismatch");
	}

	hba->curr_limit = curr_limit;
	hba->supply_voltage = supply_voltage;
	hba->prev_request_mv = supply_voltage;
	ret = send_request(hba, rdo);
	if (!ret)
		set_state(hba, PD_STATE_SNK_REQUESTED);
	/* If fail send request, do nothing, let source re-send source cap */
}
#endif

STATIC void pd_update_pdo_flags(struct typec_hba *hba, uint32_t pdo)
{
#ifdef CONFIG_CHARGE_MANAGER
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	int charge_whitelisted =
		(hba->power_role == PD_ROLE_SINK &&
		 pd_charge_from_device(pd_get_identity_vid(hba),
				       pd_get_identity_pid(hba)));
#else
	const int charge_whitelisted = 0;
#endif
#endif

	/* can only parse PDO flags if type is fixed */
	if ((pdo & PDO_TYPE_MASK) != PDO_TYPE_FIXED)
		return;

#ifdef CONFIG_USB_PD_DUAL_ROLE
	if (pdo & PDO_FIXED_DUAL_ROLE)
		hba->flags |= PD_FLAGS_PARTNER_DR_POWER;
	else
		hba->flags &= ~PD_FLAGS_PARTNER_DR_POWER;

	if (pdo & PDO_FIXED_EXTERNAL)
		hba->flags |= PD_FLAGS_PARTNER_EXTPOWER;
	else
		hba->flags &= ~PD_FLAGS_PARTNER_EXTPOWER;
#endif

	if (pdo & PDO_FIXED_DATA_SWAP)
		hba->flags |= PD_FLAGS_PARTNER_DR_DATA;
	else
		hba->flags &= ~PD_FLAGS_PARTNER_DR_DATA;

#ifdef CONFIG_CHARGE_MANAGER
	/*
	 * Treat device as a dedicated charger (meaning we should charge
	 * from it) if it does not support power swap, or if it is externally
	 * powered, or if we are a sink and the device identity matches a
	 * charging white-list.
	 */
	if (!(hba->flags & PD_FLAGS_PARTNER_DR_POWER) ||
	    (hba->flags & PD_FLAGS_PARTNER_EXTPOWER) ||
	    charge_whitelisted)
		charge_manager_update_dualrole(hba, CAP_DEDICATED);
	else
		charge_manager_update_dualrole(hba, CAP_DUALROLE);
#endif
}

STATIC void handle_data_request(struct typec_hba *hba, uint16_t head,
		uint32_t *payload)
{
	int type = PD_HEADER_TYPE(head);
	int cnt = PD_HEADER_CNT(head);

	switch (type) {
#ifdef CONFIG_USB_PD_DUAL_ROLE
		case PD_DATA_SOURCE_CAP:
			if ((hba->task_state == PD_STATE_SNK_DISCOVERY)
				|| (hba->task_state == PD_STATE_SNK_TRANSITION)
#ifdef CONFIG_USB_PD_NO_VBUS_DETECT
				|| (hba->task_state ==
				    PD_STATE_SNK_HARD_RESET_RECOVER)
#endif
				|| (hba->task_state == PD_STATE_SNK_READY))
			{
				/* Port partner is now known to be PD capable */
				hba->flags |= PD_FLAGS_PREVIOUS_PD_CONN;

				pd_store_src_cap(hba, cnt, payload);
				/* src cap 0 should be fixed PDO */
				pd_update_pdo_flags(hba, payload[0]);

				//pd_process_source_cap(hba, pd_src_cap_cnt, pd_src_caps);
				pd_send_request_msg(hba, 1);
			}
			break;
#endif /* CONFIG_USB_PD_DUAL_ROLE */

		case PD_DATA_REQUEST:
			if ((hba->power_role == PD_ROLE_SOURCE) && (cnt == 1))
			{
				if (!pd_check_requested_voltage(payload[0]))
				{
					if (send_control(hba, PD_CTRL_ACCEPT))
						/*
						 * if we fail to send accept, do
						 * nothing and let sink timeout and
						 * send hard reset
						 */
						return;

					/* explicit contract is now in place */
					hba->flags |= PD_FLAGS_EXPLICIT_CONTRACT;
					hba->requested_idx = payload[0] >> 28;
					set_state(hba, PD_STATE_SRC_ACCEPTED);
					return;
				}
			}
			/* the message was incorrect or cannot be satisfied */
			send_control(hba, PD_CTRL_REJECT);
			/* keep last contract in place (whether implicit or explicit) */
			set_state(hba, PD_STATE_SRC_READY);
			break;

		case PD_DATA_BIST:
			/* If not in READY state, then don't start BIST */
			if (DUAL_ROLE_IF_ELSE(hba,
					hba->task_state == PD_STATE_SNK_READY, hba->task_state == PD_STATE_SRC_READY))
			{
				/* currently only support sending bist carrier mode 2 */
				if ((payload[0] >> 28) == 5)
					set_state(hba, PD_STATE_BIST_CARRIER_MODE_2);
				else if (payload[0] >> 28 == 8)
					set_state(hba, PD_STATE_BIST_TEST_DATA);
			}
			break;

		case PD_DATA_SINK_CAP:
			hba->flags |= PD_FLAGS_SNK_CAP_RECVD;
			/* snk cap 0 should be fixed PDO */
			pd_update_pdo_flags(hba, payload[0]);
			if (hba->task_state == PD_STATE_SRC_GET_SINK_CAP)
				set_state(hba, PD_STATE_SRC_READY);
			break;

		#if 0
		case PD_DATA_VENDOR_DEF:
			handle_vdm_request(hba, cnt, payload);
			break;
		#endif

		default:
			dev_err(hba->dev, "Unhandled data message type %d\n", type);
	}
}

#ifdef CONFIG_USB_PD_DUAL_ROLE
void pd_request_power_swap(struct typec_hba *hba)
{
	if (hba->task_state == PD_STATE_SRC_READY)
		set_state(hba, PD_STATE_SRC_SWAP_INIT);
	else if (hba->task_state == PD_STATE_SNK_READY)
		set_state(hba, PD_STATE_SNK_SWAP_INIT);

	//task_wake(PD_PORT_TO_TASK_ID(hba)); //TODO
}

#ifdef CONFIG_USBC_VCONN_SWAP
STATIC void pd_request_vconn_swap(struct typec_hba *hba)
{
	if (DUAL_ROLE_IF_ELSE(hba,
		(hba->task_state == PD_STATE_SNK_READY), (hba->task_state == PD_STATE_SRC_READY)))
		set_state(hba, PD_STATE_VCONN_SWAP_SEND);

	//task_wake(PD_PORT_TO_TASK_ID(hba)); //TODO
}
#endif
#endif /* CONFIG_USB_PD_DUAL_ROLE */

void pd_request_data_swap(struct typec_hba *hba)
{
	if (DUAL_ROLE_IF_ELSE(hba, (hba->task_state == PD_STATE_SNK_READY), (hba->task_state == PD_STATE_SRC_READY)))
		set_state(hba, PD_STATE_DR_SWAP);

	//task_wake(PD_PORT_TO_TASK_ID(hba)); //TODO
}

STATIC void pd_set_data_role(struct typec_hba *hba, int role)
{
	hba->data_role = role;
	pd_execute_data_swap(hba, role);

#ifdef CONFIG_USBC_SS_MUX
#ifdef CONFIG_USBC_SS_MUX_DFP_ONLY
	/*
	 * Need to connect SS mux for if new data role is DFP.
	 * If new data role is UFP, then disconnect the SS mux.
	 */
	if (role == PD_ROLE_DFP)
		usb_mux_set(hba, TYPEC_MUX_USB, USB_SWITCH_CONNECT, hba->polarity);
	else
		usb_mux_set(hba, TYPEC_MUX_NONE, USB_SWITCH_DISCONNECT, hba->polarity);
#else
	usb_mux_set(hba, TYPEC_MUX_USB, USB_SWITCH_CONNECT, hba->polarity);
#endif
#endif

	//Only DFP receives SOP' and SOP''
	if (hba->data_role == PD_ROLE_DFP)
		typec_set(hba, (REG_PD_RX_SOP_PRIME_RCV_EN | REG_PD_RX_SOP_DPRIME_RCV_EN), PD_RX_PARAMETER);
	else
		typec_clear(hba, (REG_PD_RX_SOP_PRIME_RCV_EN | REG_PD_RX_SOP_DPRIME_RCV_EN), PD_RX_PARAMETER);

	pd_update_roles(hba);
}

STATIC void pd_dr_swap(struct typec_hba *hba)
{
	pd_set_data_role(hba, (hba->data_role == PD_ROLE_DFP ? PD_ROLE_UFP : PD_ROLE_DFP));
	hba->flags |= PD_FLAGS_DATA_SWAPPED;
}

STATIC void handle_ctrl_request(struct typec_hba *hba, uint16_t head,
		uint32_t *payload)
{
	int type = PD_HEADER_TYPE(head);
	int ret;

	struct type_mapping {
		uint16_t type;
		char name[MAX_SIZE];
	} pd_ctrl_type_mapping[] = {
		{PD_CTRL_RESERVED, "CTRL_RESERVED"},
		{PD_CTRL_GOOD_CRC, "CTRL_GOOD_CRC"},
		{PD_CTRL_GOTO_MIN, "CTRL_GOTO_MIN"},
		{PD_CTRL_ACCEPT, "CTRL_ACCEPT"},
		{PD_CTRL_REJECT, "CTRL_REJECT"},
		{PD_CTRL_PING, "CTRL_PING"},
		{PD_CTRL_PS_RDY, "CTRL_PS_RDY"},
		{PD_CTRL_GET_SOURCE_CAP, "CTRL_GET_SOURCE_CAP"},
		{PD_CTRL_GET_SINK_CAP, "CTRL_GET_SINK_CAP"},
		{PD_CTRL_DR_SWAP, "CTRL_DR_SWAP"},
		{PD_CTRL_PR_SWAP, "CTRL_PR_SWAP"},
		{PD_CTRL_VCONN_SWAP, "CTRL_VCONN_SWAP"},
		{PD_CTRL_WAIT, "CTRL_WAIT"},
		{PD_CTRL_SOFT_RESET, "CTRL_SOFT_RESET"},
	};


	if (hba->dbg_lvl >= TYPEC_DBG_LVL_2)
		dev_err(hba->dev, "RX %s\n", pd_ctrl_type_mapping[type].name);


	switch (type)
	{
		case PD_CTRL_GOOD_CRC:
			//GCRC is consumed by our controller; should not get it
			dev_err(hba->dev, "GOOD_CRC\n");
			break;

		case PD_CTRL_PING:
			//PING is ONLY from source; should not get it
			if (hba->power_role == PD_ROLE_SOURCE)
			{
				set_state(hba, PD_STATE_SOFT_RESET);
				dev_err(hba->dev, "PING\n");
			}
			break;

		case PD_CTRL_GET_SOURCE_CAP:
			ret = send_source_cap(hba);
			if (!ret && (hba->task_state == PD_STATE_SRC_DISCOVERY))
				set_state(hba, PD_STATE_SRC_NEGOCIATE);
			break;

		case PD_CTRL_GET_SINK_CAP:
			if (hba->task_state == PD_STATE_SNK_READY)
			send_sink_cap(hba);
			else
				set_state(hba, PD_STATE_SOFT_RESET);
			break;

#ifdef CONFIG_USB_PD_DUAL_ROLE
		case PD_CTRL_GOTO_MIN:
			break;

		case PD_CTRL_PS_RDY:
			if (hba->task_state == PD_STATE_SNK_SWAP_SRC_DISABLE)
			{
				set_state(hba, PD_STATE_SNK_SWAP_STANDBY);
			}
			else if (hba->task_state == PD_STATE_SRC_SWAP_STANDBY)
			{
				/* reset message ID and swap roles */
				pd_set_msg_id(hba, 0);
				pd_set_power_role(hba, PD_ROLE_SINK, 1);
				typec_vbus_det_enable(hba, 1);

				set_state(hba, PD_STATE_SNK_DISCOVERY);
			}
#ifdef CONFIG_USBC_VCONN_SWAP
			else if (hba->task_state == PD_STATE_VCONN_SWAP_INIT)
			{
				/*
				 * If VCONN is on, then this PS_RDY tells us it's
				 * ok to turn VCONN off
				 */
				if (hba->flags & PD_FLAGS_VCONN_ON)
					set_state(hba, PD_STATE_VCONN_SWAP_READY);
			}
#endif
			else if (hba->task_state == PD_STATE_SNK_DISCOVERY)
			{
				/* Don't know what power source is ready. Reset. */
				set_state(hba, PD_STATE_HARD_RESET_SEND);
			}
			else if (hba->task_state == PD_STATE_SNK_SWAP_STANDBY)
			{
				/* Do nothing, assume this is a redundant PD_RDY */
			}
			else if (hba->power_role == PD_ROLE_SINK)
			{
				set_state(hba, PD_STATE_SNK_READY);
#ifdef CONFIG_CHARGE_MANAGER
				/* Set ceiling based on what's negotiated */
				charge_manager_set_ceil(hba,
							CEIL_REQUESTOR_PD,
							hba->curr_limit);
#else
				#if 0 //marked by CC temporarily
				pd_set_input_current_limit(hba, hba->curr_limit,
							   hba->supply_voltage);
				#endif
#endif
			}
			break;
#endif

		case PD_CTRL_REJECT:
		case PD_CTRL_WAIT:
			if (hba->task_state == PD_STATE_DR_SWAP)
				set_state(hba, READY_RETURN_STATE(hba));
#ifdef CONFIG_USB_PD_DUAL_ROLE
#ifdef CONFIG_USBC_VCONN_SWAP
			else if (hba->task_state == PD_STATE_VCONN_SWAP_SEND)
				set_state(hba, READY_RETURN_STATE(hba));
#endif
			else if (hba->task_state == PD_STATE_SRC_SWAP_INIT)
				set_state(hba, PD_STATE_SRC_READY);
			else if (hba->task_state == PD_STATE_SNK_SWAP_INIT)
				set_state(hba, PD_STATE_SNK_READY);
			else if (hba->task_state == PD_STATE_SNK_REQUESTED)
				/* no explicit contract */
				set_state(hba, PD_STATE_SNK_READY); //??
#endif
			break;

		case PD_CTRL_ACCEPT:
			if (hba->task_state == PD_STATE_SOFT_RESET)
			{
				execute_soft_reset(hba);
			}
			else if (hba->task_state == PD_STATE_DR_SWAP)
			{
				/* switch data role */
				pd_dr_swap(hba);
				set_state(hba, READY_RETURN_STATE(hba));
			}
			#ifdef CONFIG_USB_PD_DUAL_ROLE
			#ifdef CONFIG_USBC_VCONN_SWAP
			else if (hba->task_state == PD_STATE_VCONN_SWAP_SEND)
			{
				/* switch vconn */
				set_state(hba, PD_STATE_VCONN_SWAP_INIT);
			}
			#endif
			else if (hba->task_state == PD_STATE_SRC_SWAP_INIT)
			{
				/* explicit contract goes away for power swap */
				hba->flags &= ~PD_FLAGS_EXPLICIT_CONTRACT;
				set_state(hba, PD_STATE_SRC_SWAP_SNK_DISABLE);
			}
			else if (hba->task_state == PD_STATE_SNK_SWAP_INIT)
			{
				/* explicit contract goes away for power swap */
				hba->flags &= ~PD_FLAGS_EXPLICIT_CONTRACT;
				set_state(hba, PD_STATE_SNK_SWAP_SNK_DISABLE);
			}
			#endif
			else if (hba->task_state == PD_STATE_SNK_REQUESTED)
			{
				/* explicit contract is now in place */
				hba->flags |= PD_FLAGS_EXPLICIT_CONTRACT;
				set_state(hba, PD_STATE_SNK_TRANSITION);
			}
			break;

		case PD_CTRL_SOFT_RESET:
			execute_soft_reset(hba);
			/* We are done, acknowledge with an Accept packet */
			send_control(hba, PD_CTRL_ACCEPT);
			break;

		case PD_CTRL_PR_SWAP:
#ifdef CONFIG_USB_PD_DUAL_ROLE
			if (pd_check_power_swap(hba))
			{
				send_control(hba, PD_CTRL_ACCEPT);

				/*
				 * Clear flag for checking power role to avoid
				 * immediately requesting another swap.
				 */
				hba->flags &= ~PD_FLAGS_CHECK_PR_ROLE;
				set_state(hba,
					  DUAL_ROLE_IF_ELSE(hba, PD_STATE_SNK_SWAP_SNK_DISABLE, PD_STATE_SRC_SWAP_SNK_DISABLE));
			}
			else
			{
				send_control(hba, PD_CTRL_REJECT);
			}
#else
			send_control(hba, PD_CTRL_REJECT);
#endif
			break;

		case PD_CTRL_DR_SWAP:
			if (pd_check_data_swap(hba))
			{
				/*
				 * Accept switch and perform data swap. Clear
				 * flag for checking data role to avoid
				 * immediately requesting another swap.
				 */
				hba->flags &= ~PD_FLAGS_CHECK_DR_ROLE;
				if (!send_control(hba, PD_CTRL_ACCEPT))
					pd_dr_swap(hba);
			}
			else
			{
				send_control(hba, PD_CTRL_REJECT);
			}
			break;

		case PD_CTRL_VCONN_SWAP:
#ifdef CONFIG_USBC_VCONN_SWAP
			if (hba->task_state == PD_STATE_SRC_READY || hba->task_state == PD_STATE_SNK_READY)
			{
				if (pd_check_vconn_swap(hba))
				{
					if (!send_control(hba, PD_CTRL_ACCEPT))
						set_state(hba, PD_STATE_VCONN_SWAP_INIT);
				}
				else
				{
					send_control(hba, PD_CTRL_REJECT);
				}
			}
#else
			send_control(hba, PD_CTRL_REJECT);
#endif
			break;

		default:
			dev_err(hba->dev, "WARNING: unknown control message\n");
			break;
	}
}

STATIC void handle_request(struct typec_hba *hba, uint16_t header,
		uint32_t *payload)
{
	#if 0
	/*
	* If we are in disconnected state, we shouldn't get a request. Do
	* a hard reset if we get one.
	*/
	if (!pd_is_connected(hba))
	set_state(hba, PD_STATE_HARD_RESET_SEND);
	#endif

	if (PD_HEADER_CNT(header))
		handle_data_request(hba, header, payload); //more than 1 objects
	else
		handle_ctrl_request(hba, header, payload); //no object
}

#if 0
void pd_send_vdm(struct typec_hba *hba, uint32_t vid, int cmd, const uint32_t *data,
		 int count)
{
	if (count > VDO_MAX_SIZE - 1) {
		CPRINTF("VDM over max size\n");
		return;
	}

	/* set VDM header with VID & CMD */
	hba->vdo_data[0] = VDO(vid, ((vid & USB_SID_PD) == USB_SID_PD) ?
				   1 : (PD_VDO_CMD(cmd) <= CMD_ATTENTION), cmd);
	queue_vdm(hba, hba->vdo_data, data, count);

	task_wake(PD_PORT_TO_TASK_ID(hba));
}

STATIC inline int pdo_busy(struct typec_hba *hba)
{
	/*
	 * Note, main PDO state machine (pd_task) uses READY state exclusively
	 * to denote port partners have successfully negociated a contract.  All
	 * other protocol actions force state transitions.
	 */
	int rv = (hba->task_state != PD_STATE_SRC_READY);
#ifdef CONFIG_USB_PD_DUAL_ROLE
	rv &= (hba->task_state != PD_STATE_SNK_READY);
#endif
	return rv;
}

STATIC uint64_t vdm_get_ready_timeout(uint32_t vdm_hdr)
{
	uint64_t timeout;
	int cmd = PD_VDO_CMD(vdm_hdr);

	/* its not a structured VDM command */
	if (!PD_VDO_SVDM(vdm_hdr))
		return 500;

	switch (PD_VDO_CMDT(vdm_hdr)) {
	case CMDT_INIT:
		if ((cmd == CMD_ENTER_MODE) || (cmd == CMD_EXIT_MODE))
			timeout = PD_T_VDM_WAIT_MODE_E;
		else
			timeout = PD_T_VDM_SNDR_RSP;
		break;
	default:
		if ((cmd == CMD_ENTER_MODE) || (cmd == CMD_EXIT_MODE))
			timeout = PD_T_VDM_E_MODE;
		else
			timeout = PD_T_VDM_RCVR_RSP;
		break;
	}
	return timeout;
}

STATIC void pd_vdm_send_state_machine(struct typec_hba *hba)
{
	int res;
	uint16_t header;

	switch (hba->vdm_state) {
	case VDM_STATE_READY:
		/* Only transmit VDM if connected. */
		if (!pd_is_connected(hba)) {
			hba->vdm_state = VDM_STATE_ERR_BUSY;
			break;
		}

		/*
		 * if there's traffic or we're not in PDO ready state don't send
		 * a VDM.
		 */
		if (pdo_busy(hba))
			break;

		/* Prepare and send VDM */
		header = PD_HEADER(PD_DATA_VENDOR_DEF, hba->power_role,
				   hba->data_role, (int)hba->vdo_count);
		res = pd_transmit(hba, TCPC_TX_SOP, header,
				  hba->vdo_data);
		if (res < 0) {
			hba->vdm_state = VDM_STATE_ERR_SEND;
		} else {
			hba->vdm_state = VDM_STATE_BUSY;
			hba->vdm_timeout.val = get_time().val +
				vdm_get_ready_timeout(hba->vdo_data[0]);
		}
		break;
	case VDM_STATE_WAIT_RSP_BUSY:
		/* wait and then initiate request again */
		if (get_time().val > hba->vdm_timeout.val) {
			hba->vdo_data[0] = hba->vdo_retry;
			hba->vdo_count = 1;
			hba->vdm_state = VDM_STATE_READY;
		}
		break;
	case VDM_STATE_BUSY:
		/* Wait for VDM response or timeout */
		if (hba->vdm_timeout.val &&
		    (get_time().val > hba->vdm_timeout.val)) {
			hba->vdm_state = VDM_STATE_ERR_TMOUT;
		}
		break;
	default:
		break;
	}
}
#endif

#ifdef CONFIG_USB_PD_DUAL_ROLE
static int pd_is_power_swapping(struct typec_hba *hba)
{
	/* return true if in the act of swapping power roles */
	return (hba->task_state == PD_STATE_SNK_SWAP_SNK_DISABLE ||
		hba->task_state == PD_STATE_SNK_SWAP_SRC_DISABLE ||
		hba->task_state == PD_STATE_SNK_SWAP_STANDBY ||
		hba->task_state == PD_STATE_SNK_SWAP_COMPLETE ||
		hba->task_state == PD_STATE_SRC_SWAP_SNK_DISABLE ||
		hba->task_state == PD_STATE_SRC_SWAP_SRC_DISABLE ||
		hba->task_state == PD_STATE_SRC_SWAP_STANDBY);
}
#endif /* CONFIG_USB_PD_DUAL_ROLE */

#ifdef CONFIG_COMMON_RUNTIME
STATIC void pd_comm_enable(struct typec_hba *hba, int enable)
{
	hba->pd_comm_enabled = enable;
	pd_rx_enable(hba, enable);
}
#endif

STATIC void pd_ping_enable(struct typec_hba *hba, int enable)
{
	if (enable)
		hba->flags |= PD_FLAGS_PING_ENABLED;
	else
		hba->flags &= ~PD_FLAGS_PING_ENABLED;
}


#if PD_SW_WORKAROUND1_1
void pd_get_message(struct typec_hba *hba, uint16_t *header, uint32_t *payload)
#else
STATIC void pd_get_message(struct typec_hba *hba, uint16_t *header, uint32_t *payload)
#endif
{
	int i;
	uint8_t cnt;
	unsigned long flags;

	//header
	*header = typec_readw(hba, PD_RX_HEADER);

	//data
	cnt = PD_HEADER_CNT(*header);
	for (i = 0; i < cnt; i++)
	{
		payload[i] = (typec_readw(hba, (PD_RX_DATA_OBJECT0_1+i*4)) << 16);
		payload[i] |= typec_readw(hba, (PD_RX_DATA_OBJECT0_0+i*4));
	}

	#if PD_SW_WORKAROUND1_2
	spin_lock_irqsave(&hba->typec_lock, flags);
	typec_writew(hba, PD_RX_RCV_MSG_INTR, PD_INTR_0);
	typec_set(hba, REG_PD_RX_RCV_MSG_INTR_EN, PD_INTR_EN_0);
	spin_unlock_irqrestore(&hba->typec_lock, flags);
	#endif
}

int pd_task(void *data)
{
	int i;
	uint16_t header;
	uint32_t payload[7];
	unsigned long timeout = 10;
	int ret;
	int incoming_packet = 0;
	int hard_reset_count = 0;
#ifdef CONFIG_USB_PD_DUAL_ROLE
	//uint64_t next_role_swap = PD_T_DRP_SNK;
#ifndef CONFIG_USB_PD_NO_VBUS_DETECT
	int snk_hard_reset_vbus_off = 0;
#endif
#ifdef CONFIG_CHARGE_MANAGER
	int typec_curr = 0, typec_curr_change = 0;
#endif /* CONFIG_CHARGE_MANAGER */
#endif /* CONFIG_USB_PD_DUAL_ROLE */
	enum pd_states curr_state;
	int caps_count = 0, hard_reset_sent = 0;
	int snk_cap_count;
	struct typec_hba *hba = (struct typec_hba *)data;
	unsigned long flags;
	uint16_t pd_is0, pd_is1, cc_is0;
	uint8_t missing_event;


	/* Disable RX until connection is established */
	pd_rx_enable(hba, 0);

	/* Ensure the power supply is in the default state */
	pd_power_supply_reset(hba);

	#if 0
#ifdef CONFIG_USBC_SS_MUX
	/* Initialize USB mux to its default state */
	usb_mux_init(hba);
#endif
	#endif

	/* Initialize PD protocol state variables for each port. */
	hba->power_role = 0;
	hba->data_role = 0;
	hba->flags = 0;
	hba->vdm_state = VDM_STATE_DONE;

	hba->timeout_user = 0;
	set_state(hba, PD_STATE_DISABLED);


	/* Initialize completion for stress test */
	init_completion(&hba->ready);


	#if 0
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	/* Initialize PD Policy engine */
	pd_dfp_pe_init(hba);
#endif

#ifdef CONFIG_CHARGE_MANAGER
	/* Initialize PD and type-C supplier current limits to 0 */
	pd_set_input_current_limit(hba, 0, 0);
	typec_set_input_current_limit(hba, 0, 0);
	charge_manager_update_dualrole(hba, CAP_UNKNOWN);
#endif
	#endif


	while (1)
	{
		#if 0
		/* process VDM messages last */
		pd_vdm_send_state_machine(hba);

		/* Verify board specific health status : current, voltages... */
		res = pd_board_checks();
		if (res != EC_SUCCESS) {
			/* cut the power */
			pd_execute_hard_reset(hba);
			/* notify the other side of the issue */
			pd_transmit(hba, TCPC_TX_HARD_RESET, 0, NULL);
		}
		#endif


		/*
			RX & CC events may happen
				1) in the middle of init_completion
				2) after their event handlers finished last time
			if this is case, jump directly into event handlers; otherwise, wait for timeout, or CC/RX/timer events
		*/
		typec_sw_probe(hba, DBG_LOOP_STATE, (DBG_LOOP_WAIT<<DBG_LOOP_STATE_OFST));
		typec_sw_probe(hba, DBG_TIMER0_VAL, timeout<<DBG_TIMER0_VAL_OFST);

		spin_lock_irqsave(&hba->typec_lock, flags);
		missing_event = (hba->pd_is0 & PD_RX_SUCCESS0) || (hba->pd_is1 & PD_RX_HR_SUCCESS) || (hba->cc_is0);
		spin_unlock_irqrestore(&hba->typec_lock, flags);
		if (!missing_event)
		{
			init_completion(&hba->event);
		#if PD_SW_WORKAROUND2
			pd_timer0_start(hba, timeout);
			wait_for_completion(&hba->event); //timer event terminates wait when timeouts
		#else
			ret = wait_for_completion_timeout(&hba->event, pd_msecs_to_jiffies(timeout));
		#endif
		}


		/* latch events */
		typec_sw_probe(hba, DBG_LOOP_STATE, (DBG_LOOP_CHK_EVENT<<DBG_LOOP_STATE_OFST));
		spin_lock_irqsave(&hba->typec_lock, flags);
		pd_is0 = hba->pd_is0;
		pd_is1 = hba->pd_is1;
		cc_is0 = hba->cc_is0;

		pd_clear_event(hba, (pd_is0 & PD_EVENTS0), (pd_is1 & PD_EVENTS1), cc_is0);
		spin_unlock_irqrestore(&hba->typec_lock, flags);


		/* dump timeout information */
		typec_sw_probe(hba, DBG_TIMER0_VAL, 0);
		#if PD_SW_WORKAROUND2
		//no matter triggered or not, timer0 work has finished
		pd_timer0_stop(hba);
		if ((hba->dbg_lvl >= TYPEC_DBG_LVL_3) && (pd_is1 & PD_TIMER0_TIMEOUT_INTR))
			dev_err(hba->dev, "[TIMEOUT1 %dms]\n", timeout);
		#else
		if ((hba->dbg_lvl >= TYPEC_DBG_LVL_3) && !ret)
			dev_err(hba->dev, "[TIMEOUT1 %dms]\n", (pd_msecs_to_jiffies(timeout)*PD_HZ_FACTOR_DIVIDER));
		#endif

		//report events, for debug purpose
		if ((hba->dbg_lvl >= TYPEC_DBG_LVL_3) && (pd_is0 | pd_is1 | cc_is0))
			dev_err(hba->dev, "pd_is0: %x, pd_is1: %x, cc_is0: %x\n", pd_is0, pd_is1, cc_is0);


		/* process RX events */
		if (pd_is0 & PD_RX_SUCCESS0)
		{
			typec_sw_probe(hba, DBG_LOOP_STATE, (DBG_LOOP_RX<<DBG_LOOP_STATE_OFST));
			#if PD_SW_WORKAROUND1_1
			if (hba->header && (hba->bist_mode != BDO_MODE_TEST_DATA))
				handle_request(hba, hba->header, hba->payload);
			#else
			pd_get_message(hba, &header, payload);
			if (header && (hba->bist_mode != BDO_MODE_TEST_DATA))
				handle_request(hba, header, payload);
			#endif
		}


		/* process hard reset */
		if (pd_is1 & PD_RX_HR_SUCCESS)
		{
			typec_sw_probe(hba, DBG_LOOP_STATE, (DBG_LOOP_HARD_RESET<<DBG_LOOP_STATE_OFST));
			pd_complete_hr(hba);
			pd_execute_hard_reset(hba);
		}


		/* process CC events */
		//change state according to Type-C controller status
		typec_sw_probe(hba, DBG_LOOP_STATE, (DBG_LOOP_CHK_CC_EVENT<<DBG_LOOP_STATE_OFST));
		if ((cc_is0 & TYPE_C_CC_ENT_ATTACH_SRC_INTR) && !pd_is_power_swapping(hba))
			set_state(hba, PD_STATE_SRC_ATTACH);
		if ((cc_is0 & TYPE_C_CC_ENT_ATTACH_SNK_INTR) && !pd_is_power_swapping(hba))
			set_state(hba, PD_STATE_SNK_ATTACH);
		if (cc_is0 & TYPE_C_CC_ENT_DISABLE_INTR)
			set_state(hba, PD_STATE_DISABLED);
		if (cc_is0 & TYPE_C_CC_ENT_UNATTACH_SRC_INTR)
			set_state(hba, PD_STATE_SRC_UNATTACH);
		if (cc_is0 & TYPE_C_CC_ENT_UNATTACH_SNK_INTR)
			set_state(hba, PD_STATE_SNK_UNATTACH);


		/* if nothing to do, verify the state of the world in 500ms */
		curr_state = hba->task_state;
		timeout = (hba->timeout_user ? hba->timeout_user : 500);

		if ((hba->dbg_lvl >= TYPEC_DBG_LVL_1)
			&& state_changed(hba)
			&& !(hba->last_state == PD_STATE_SRC_UNATTACH && hba->task_state == PD_STATE_SNK_UNATTACH)
			&& !(hba->last_state == PD_STATE_SNK_UNATTACH && hba->task_state == PD_STATE_SRC_UNATTACH)) //skip DRP toggle cases
		{
			dev_err(hba->dev, "%s->%s\n",
				pd_state_mapping[hba->last_state].name, pd_state_mapping[hba->task_state].name);
		}


		/* state machine */
		typec_sw_probe(hba, DBG_LOOP_STATE, (DBG_LOOP_STATE_MACHINE<<DBG_LOOP_STATE_OFST));
		switch (curr_state)
		{
			case PD_STATE_DISABLED:
			case PD_STATE_SRC_UNATTACH:
#ifdef CONFIG_USB_PD_DUAL_ROLE
			case PD_STATE_SNK_UNATTACH:
				/* Clear the input current limit */
				//pd_set_input_current_limit(hba, 0, 0);
#ifdef CONFIG_CHARGE_MANAGER
				typec_set_input_current_limit(hba, 0, 0);
				charge_manager_set_ceil(hba,
							CEIL_REQUESTOR_PD,
							CHARGE_CEIL_NONE);
#endif
#ifdef CONFIG_USBC_VCONN
				typec_drive_vconn(hba, 0);
#endif
#endif
				//hba->dev_id = 0;
				hba->flags &= ~PD_FLAGS_RESET_ON_DISCONNECT_MASK;
#ifdef CONFIG_CHARGE_MANAGER
				charge_manager_update_dualrole(hba, CAP_UNKNOWN);
#endif
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
				pd_dfp_exit_mode(hba, 0, 0);
#endif
#ifdef CONFIG_USBC_SS_MUX
				usb_mux_set(hba, TYPEC_MUX_NONE, USB_SWITCH_DISCONNECT,
						hba->polarity);
#endif

				/* be aware of Vbus */
				typec_vbus_det_enable(hba, 1);

				/* disable RX */
				pd_rx_enable(hba, 0);

				/* disable BIST */
				pd_bist_enable(hba, 0, 0);
				break;

			#if 0
			case PD_STATE_SRC_DISCONNECTED:
				timeout = 10;
				tcpm_get_cc(hba, &cc1, &cc2);

				/* Vnc monitoring */
				if ((cc1 == TYPEC_CC_VOLT_RD ||
				     cc2 == TYPEC_CC_VOLT_RD) ||
				    (cc1 == TYPEC_CC_VOLT_RA &&
				     cc2 == TYPEC_CC_VOLT_RA)) {
#ifdef CONFIG_USBC_BACKWARDS_COMPATIBLE_DFP
					/* Enable VBUS */
					if (pd_set_power_supply_ready(hba))
						break;
#endif
					hba->cc_state = PD_CC_NONE;
					set_state(hba,
						PD_STATE_SRC_DISCONNECTED_DEBOUNCE);
				}
#ifdef CONFIG_USB_PD_DUAL_ROLE
				/*
				 * Try.SRC state is embedded here. Wait for SNK
				 * detect, or if timer expires, transition to
				 * SNK_DISCONNETED.
				 *
				 * If Try.SRC state is not active, then this block
				 * handles the normal DRP toggle from SRC->SNK
				 */
				else if ((hba->flags & PD_FLAGS_TRY_SRC &&
					 get_time().val >= hba->try_src_marker) ||
					 (!(hba->flags & PD_FLAGS_TRY_SRC) &&
					  drp_state != PD_DRP_FORCE_SOURCE &&
					 get_time().val >= next_role_swap)) {
					hba->power_role = PD_ROLE_SINK;
					set_state(hba, PD_STATE_SNK_DISCONNECTED);
					tcpm_set_cc(hba, TYPEC_CC_RD);
					next_role_swap = get_time().val + PD_T_DRP_SNK;
					hba->try_src_marker = get_time().val
						+ PD_T_TRY_WAIT;

					/* Swap states quickly */
					timeout = 2;
				}
#endif
				break;

			case PD_STATE_SRC_DISCONNECTED_DEBOUNCE:
				timeout = 20;
				tcpm_get_cc(hba, &cc1, &cc2);

				if (cc1 == TYPEC_CC_VOLT_RD &&
				    cc2 == TYPEC_CC_VOLT_RD) {
					/* Debug accessory */
					new_cc_state = PD_CC_DEBUG_ACC;
				} else if (cc1 == TYPEC_CC_VOLT_RD ||
					   cc2 == TYPEC_CC_VOLT_RD) {
					/* UFP attached */
					new_cc_state = PD_CC_UFP_ATTACHED;
				} else if (cc1 == TYPEC_CC_VOLT_RA &&
					   cc2 == TYPEC_CC_VOLT_RA) {
					/* Audio accessory */
					new_cc_state = PD_CC_AUDIO_ACC;
				} else {
					/* No UFP */
#ifdef CONFIG_USBC_BACKWARDS_COMPATIBLE_DFP
					/* No connection any more, remove VBUS */
					pd_power_supply_reset(hba);
#endif
					set_state(hba, PD_STATE_SRC_DISCONNECTED);
					timeout = 5;
					break;
				}
				/* If in Try.SRC state, then don't need to debounce */
				if (!(hba->flags & PD_FLAGS_TRY_SRC)) {
					/* Debounce the cc state */
					if (new_cc_state != hba->cc_state) {
						hba->cc_debounce = get_time().val +
							PD_T_CC_DEBOUNCE;
						hba->cc_state = new_cc_state;
						break;
					} else if (get_time().val <
						   hba->cc_debounce) {
						break;
					}
				}

				/* Debounce complete */
				/* UFP is attached */
				if (new_cc_state == PD_CC_UFP_ATTACHED) {
					hba->polarity = (cc2 == TYPEC_CC_VOLT_RD);
					tcpm_set_polarity(hba, hba->polarity);

					/* initial data role for source is DFP */
					pd_set_data_role(hba, PD_ROLE_DFP);

#ifndef CONFIG_USBC_BACKWARDS_COMPATIBLE_DFP
					/* Enable VBUS */
					if (pd_set_power_supply_ready(hba)) {
#ifdef CONFIG_USBC_SS_MUX
						usb_mux_set(hba, TYPEC_MUX_NONE,
							    USB_SWITCH_DISCONNECT,
							    hba->polarity);
#endif
						break;
					}
#endif
					/* If PD comm is enabled, enable TCPC RX */
					if (pd_comm_enabled)
						tcpm_set_rx_enable(hba, 1);

#ifdef CONFIG_USBC_VCONN
					typec_drive_vconn(hba, 1);
					hba->flags |= PD_FLAGS_VCONN_ON;
#endif

					hba->flags |= PD_FLAGS_CHECK_PR_ROLE |
							  PD_FLAGS_CHECK_DR_ROLE;
					hard_reset_count = 0;
					timeout = 5;
					set_state(hba, PD_STATE_SRC_STARTUP);
				}
				/* Accessory is attached */
				else if (new_cc_state == PD_CC_AUDIO_ACC ||
					 new_cc_state == PD_CC_DEBUG_ACC) {
#ifdef CONFIG_USBC_BACKWARDS_COMPATIBLE_DFP
					/* Remove VBUS */
					pd_power_supply_reset(hba);
#endif

					/* Set the USB muxes and the default USB role */
					pd_set_data_role(hba, CONFIG_USB_PD_DEBUG_DR);

#ifdef CONFIG_CASE_CLOSED_DEBUG
					if (new_cc_state == PD_CC_DEBUG_ACC) {
						ccd_set_mode(system_is_locked() ?
							     CCD_MODE_PARTIAL :
							     CCD_MODE_ENABLED);
						typec_set_input_current_limit(
							port, 3000, TYPE_C_VOLTAGE);
						charge_manager_update_dualrole(
							port, CAP_DEDICATED);
					}
#endif
					set_state(hba, PD_STATE_SRC_ACCESSORY);
				}
				break;
			#endif

			case PD_STATE_SRC_HARD_RESET_RECOVER:
				/* Do not continue until hard reset recovery time */
				#if PD_SW_WORKAROUND3
				if (!time_after_16(pd_get_jiffies(hba), hba->src_recover))
				#else
				if (!time_after(jiffies, hba->src_recover))
				#endif
				{
					timeout = 50;

					break;
				}

				/* Enable VBUS */
				timeout = 10;
				if (!pd_set_power_supply_ready(hba))
				{
					set_state(hba, PD_STATE_SRC_UNATTACH);
				}
				else
				{
					set_state(hba, PD_STATE_SRC_STARTUP);
					pd_complete_hr(hba);

					pd_bist_enable(hba, 0, 0);
				}
				break;

			case PD_STATE_SRC_ATTACH:
				#if PD_DVT
				if (hba->pd_comm_enabled)
				{
					pd_rx_enable(hba, 1);

					pd_set_power_role(hba, PD_ROLE_SOURCE, 1);
					pd_set_data_role(hba, PD_ROLE_DFP);

#ifdef CONFIG_USBC_VCONN
					//drive Vconn ONLY when there is Ra
					if (typec_readw(hba, TYPE_C_PWR_STATUS) & RO_TYPE_C_DRIVE_VCONN_CAPABLE)
					{
						typec_drive_vconn(hba, 1);
						hba->flags |= PD_FLAGS_VCONN_ON;
					}
#endif
					hba->flags |= (PD_FLAGS_CHECK_PR_ROLE | PD_FLAGS_CHECK_DR_ROLE);
					hard_reset_count = 0;
					timeout = 5;
					set_state(hba, PD_STATE_SRC_STARTUP);
				}
				#else
				if (hba->pd_comm_enabled)
					pd_rx_enable(hba, 1);

				pd_set_power_role(hba, PD_ROLE_SOURCE, 1);
				pd_set_data_role(hba, PD_ROLE_DFP);

#ifdef CONFIG_USBC_VCONN
				//drive Vconn ONLY when there is Ra
				if (typec_readw(TYPE_C_PWR_STATUS) & RO_TYPE_C_DRIVE_VCONN_CAPABLE)
				{
					typec_drive_vconn(hba, 1);
					hba->flags |= PD_FLAGS_VCONN_ON;
				}
#endif
				hba->flags |= (PD_FLAGS_CHECK_PR_ROLE | PD_FLAGS_CHECK_DR_ROLE);
				hard_reset_count = 0;
				timeout = 5;
				set_state(hba, PD_STATE_SRC_STARTUP);
				#endif
				break;

			case PD_STATE_SRC_STARTUP:
				/* Wait for power source to enable */
				if (state_changed(hba))
				{
					/*
					 * fake set data role swapped flag so we send
					 * discover identity when we enter SRC_READY
					 */
					hba->flags |= PD_FLAGS_DATA_SWAPPED;
					/* reset various counters */
					caps_count = 0;
					snk_cap_count = 0;
					pd_set_msg_id(hba, 0);
					set_state_timeout(hba, PD_POWER_SUPPLY_TURN_ON_DELAY, PD_STATE_SRC_DISCOVERY);
				}
				break;

			case PD_STATE_SRC_DISCOVERY:
				if (state_changed(hba))
				{
					/*
					 * If we have had PD connection with this port
					 * partner, then start NoResponseTimer.
					 */
					if (hba->flags & PD_FLAGS_PREVIOUS_PD_CONN)
						set_state_timeout(hba, PD_T_NO_RESPONSE,
							((hard_reset_count < PD_HARD_RESET_COUNT) ?
								PD_STATE_HARD_RESET_SEND : PD_STATE_SRC_UNATTACH));
				}

				/* Send source cap some minimum number of times */
				if (caps_count < PD_CAPS_COUNT)
				{
					/* Query capabilites of the other side */
					ret = send_source_cap(hba);
					/* packet was acked => PD capable device) */
					if (ret)
					{
						timeout = PD_T_SEND_SOURCE_CAP;
						caps_count++;
					}
					else
					{
						set_state(hba, PD_STATE_SRC_NEGOCIATE);

						timeout = 10;
						hard_reset_count = 0;
						caps_count = 0;
						/* Port partner is PD capable */
						hba->flags |=
							PD_FLAGS_PREVIOUS_PD_CONN;
					}
				}
				break;

			case PD_STATE_SRC_NEGOCIATE:
				/* wait for a "Request" message */
				if (state_changed(hba))
					set_state_timeout(hba, PD_T_SENDER_RESPONSE, PD_STATE_HARD_RESET_SEND);
				break;

			case PD_STATE_SRC_ACCEPTED:
				/* Accept sent, wait for enabling the new voltage */
				if (state_changed(hba))
					set_state_timeout(hba, PD_T_SINK_TRANSITION, PD_STATE_SRC_POWERED);
				break;

			case PD_STATE_SRC_POWERED:
				/* Switch to the new requested voltage */
				if (state_changed(hba))
				{
					pd_transition_voltage(hba->requested_idx);
					set_state_timeout(hba, PD_POWER_SUPPLY_TURN_ON_DELAY, PD_STATE_SRC_TRANSITION);
				}
				break;

			case PD_STATE_SRC_TRANSITION:
				/* the voltage output is good, notify the source */
				ret = send_control(hba, PD_CTRL_PS_RDY);
				if (ret)
				{
					/* The sink did not ack, cut the power... */
					//pd_power_supply_reset(hba);
					set_state(hba, PD_STATE_SRC_UNATTACH);
				}
				else
				{
					timeout = 10;
					/* it'a time to ping regularly the sink */
					set_state(hba, PD_STATE_SRC_READY);
				}
				break;

			case PD_STATE_SRC_READY:
				complete(&hba->ready);

				timeout = PD_T_SOURCE_ACTIVITY;

				/*
				 * Don't send any PD traffic if we woke up due to
				 * incoming packet or if VDO response pending to avoid
				 * collisions.
				 */
				if (incoming_packet ||
				    (hba->vdm_state == VDM_STATE_BUSY))
					break;

				/* Send get sink cap if haven't received it yet */
				if (state_changed(hba) && !(hba->flags & PD_FLAGS_SNK_CAP_RECVD))
				{
					if (++snk_cap_count <= PD_SNK_CAP_RETRIES)
					{
						/* Get sink cap to know if dual-role device */
						send_control(hba, PD_CTRL_GET_SINK_CAP);
						set_state(hba, PD_STATE_SRC_GET_SINK_CAP);
						break;
					}
					else if (snk_cap_count == PD_SNK_CAP_RETRIES+1)
					{
						dev_err(hba->dev, "ERR SNK_CAP\n");
					}
				}

				/* Check power role policy, which may trigger a swap */
				if (hba->flags & PD_FLAGS_CHECK_PR_ROLE)
				{
					pd_check_pr_role(hba, PD_ROLE_SOURCE, hba->flags);
					hba->flags &= ~PD_FLAGS_CHECK_PR_ROLE;
					break;
				}

				/* Check data role policy, which may trigger a swap */
				if (hba->flags & PD_FLAGS_CHECK_DR_ROLE)
				{
					pd_check_dr_role(hba, hba->data_role, hba->flags);
					hba->flags &= ~PD_FLAGS_CHECK_DR_ROLE;
					break;
				}

				#if 0
				/* Send discovery SVDMs last */
				if (hba->data_role == PD_ROLE_DFP &&
				    (hba->flags & PD_FLAGS_DATA_SWAPPED)) {
#ifndef CONFIG_USB_PD_SIMPLE_DFP
					pd_send_vdm(hba, USB_SID_PD,
						    CMD_DISCOVER_IDENT, NULL, 0);
#endif
					hba->flags &= ~PD_FLAGS_DATA_SWAPPED;
					break;
				}
				#endif

				if (!(hba->flags & PD_FLAGS_PING_ENABLED))
					break;

				ret = send_control(hba, PD_CTRL_PING);
				#if 0 //TODO
				/* Verify that the sink is alive */
				if (ret)
				{
					/* Ping dropped. Try soft reset. */
					set_state(hba, PD_STATE_SOFT_RESET);
					timeout = 10;
				}
				#endif

				break;

			case PD_STATE_SRC_GET_SINK_CAP:
				if (state_changed(hba))
					set_state_timeout(hba, PD_T_SENDER_RESPONSE, PD_STATE_SRC_READY);
				break;

			case PD_STATE_DR_SWAP:
				if (state_changed(hba))
				{
					if (send_control(hba, PD_CTRL_DR_SWAP))
					{
						timeout = 10;

						#if 0
						/*
						 * If failed to get goodCRC, send
						 * soft reset, otherwise ignore
						 * failure.
						 */
						set_state(hba, res == -1 ?
							   PD_STATE_SOFT_RESET :
							   READY_RETURN_STATE(port));
						#else
						set_state(hba, PD_STATE_SOFT_RESET);
						#endif

						break;
					}

					/* Wait for accept or reject */
					set_state_timeout(hba, PD_T_SENDER_RESPONSE, READY_RETURN_STATE(hba));
				}
				break;

#ifdef CONFIG_USB_PD_DUAL_ROLE
			case PD_STATE_SRC_SWAP_INIT:
				if (state_changed(hba))
				{
					if (send_control(hba, PD_CTRL_PR_SWAP))
					{
						timeout = 10;

						#if 0
						/*
						 * If failed to get goodCRC, send
						 * soft reset, otherwise ignore
						 * failure.
						 */
						set_state(hba, res == -1 ?
							   PD_STATE_SOFT_RESET :
							   PD_STATE_SRC_READY);
						#else
						set_state(hba, PD_STATE_SOFT_RESET);
						#endif

						break;
					}

					/* Wait for accept or reject */
					set_state_timeout(hba, PD_T_SENDER_RESPONSE, PD_STATE_SRC_READY);
				}
				break;

			case PD_STATE_SRC_SWAP_SNK_DISABLE:
				/* Give time for sink to stop drawing current */
				if (state_changed(hba))
				{
					set_state_timeout(hba, PD_T_SINK_TRANSITION, PD_STATE_SRC_SWAP_SRC_DISABLE);
				}
				break;

			case PD_STATE_SRC_SWAP_SRC_DISABLE:
				/* Turn power off */
				if (state_changed(hba))
				{
					//cut off power and do not detect vbus during the swap
					typec_vbus_det_enable(hba, 0);
					pd_power_supply_reset(hba);

					set_state_timeout(hba, PD_POWER_SUPPLY_TURN_OFF_DELAY, PD_STATE_SRC_SWAP_STANDBY);
				}
				break;

			case PD_STATE_SRC_SWAP_STANDBY:
				/* Send PS_RDY to let sink know our power is off */
				if (state_changed(hba))
				{
					//Turn VBUS_PRESENT on because PR is changed to SNK later
					typec_vbus_present(hba, 1);

					//Switch to Rd but update message header later
					pd_set_power_role(hba, PD_ROLE_SINK, 0);

					//Send PD_RDY
					if (send_control(hba, PD_CTRL_PS_RDY))
					{
						timeout = 10;
						set_state(hba, PD_STATE_SRC_UNATTACH);

						break;
					}

					hba->flags |= PD_FLAGS_POWER_SWAPPED; //instruct driver not to update PD on entry to SNK attach

					//Wait for PS_RDY from new source
					set_state_timeout(hba, PD_T_PS_SOURCE_ON, PD_STATE_SNK_UNATTACH);
				}
				break;

			#if 0
			case PD_STATE_SUSPENDED:
				/*
				 * TODO: Suspend state only supported if we are also
				 * the TCPC.
				 */
#ifdef CONFIG_USB_PD_TCPC
				pd_rx_disable_monitoring(hba);
				pd_hw_release(hba);
				pd_power_supply_reset(hba);

				/* Wait for resume */
				while (hba->task_state == PD_STATE_SUSPENDED)
					task_wait_event(-1);

				pd_hw_init(hba, PD_ROLE_DEFAULT);
#endif
				break;

			case PD_STATE_SNK_DISCONNECTED:
#ifdef CONFIG_USB_PD_LOW_POWER
				timeout = drp_state == PD_DRP_TOGGLE_OFF ? MINUTE
									 : 10;
#else
				timeout = 10;
#endif
				tcpm_get_cc(hba, &cc1, &cc2);

				/* Source connection monitoring */
				if (cc1 != TYPEC_CC_VOLT_OPEN ||
				    cc2 != TYPEC_CC_VOLT_OPEN) {
					hba->cc_state = PD_CC_NONE;
					hard_reset_count = 0;
					new_cc_state = PD_CC_DFP_ATTACHED;
					hba->cc_debounce = get_time().val +
								PD_T_CC_DEBOUNCE;
					set_state(hba,
						PD_STATE_SNK_DISCONNECTED_DEBOUNCE);
					timeout = 10;
					break;
				}

				/*
				 * If Try.SRC is active and failed to detect a SNK,
				 * then it transitions to TryWait.SNK. Need to prevent
				 * normal dual role toggle until tDRPTryWait timer
				 * expires.
				 */
				if (hba->flags & PD_FLAGS_TRY_SRC) {
					if (get_time().val > hba->try_src_marker)
						hba->flags &= ~PD_FLAGS_TRY_SRC;
					break;
				}

				/*
				 * If no source detected, check for role toggle.
				 * If VBUS is detected, and we are in the debug
				 * accessory toggle state, then allow toggling.
				 */
				if ((drp_state == PD_DRP_TOGGLE_ON &&
				     get_time().val >= next_role_swap) ||
				    pd_snk_debug_acc_toggle(hba)) {
					/* Swap roles to source */
					hba->power_role = PD_ROLE_SOURCE;
					set_state(hba, PD_STATE_SRC_DISCONNECTED);
					tcpm_set_cc(hba, TYPEC_CC_RP);
					next_role_swap = get_time().val + PD_T_DRP_SRC;

					/* Swap states quickly */
					timeout = 2;
				}
				break;

			case PD_STATE_SNK_DISCONNECTED_DEBOUNCE:
				tcpm_get_cc(hba, &cc1, &cc2);
				if (cc1 == TYPEC_CC_VOLT_OPEN &&
				    cc2 == TYPEC_CC_VOLT_OPEN) {
					/* No connection any more */
					set_state(hba, PD_STATE_SNK_DISCONNECTED);
					timeout = 5;
					break;
				}

				timeout = 20;

				/* Wait for CC debounce and VBUS present */
				if (get_time().val < hba->cc_debounce ||
				    !typec_is_vbus_present(hba))
					break;

				if (pd_try_src_enable &&
				    !(hba->flags & PD_FLAGS_TRY_SRC)) {
					/*
					 * If TRY_SRC is enabled, but not active,
					 * then force attempt to connect as source.
					 */
					hba->try_src_marker = get_time().val
						+ PD_T_TRY_SRC;
					/* Swap roles to source */
					hba->power_role = PD_ROLE_SOURCE;
					tcpm_set_cc(hba, TYPEC_CC_RP);
					timeout = 2;
					set_state(hba, PD_STATE_SRC_DISCONNECTED);
					/* Set flag after the state change */
					hba->flags |= PD_FLAGS_TRY_SRC;
					break;
				}

				/* We are attached */
				hba->polarity = (cc2 != TYPEC_CC_VOLT_OPEN);
				tcpm_set_polarity(hba, hba->polarity);
				/* initial data role for sink is UFP */
				pd_set_data_role(hba, PD_ROLE_UFP);
#ifdef CONFIG_CHARGE_MANAGER
				typec_curr = get_typec_current_limit(
					hba->polarity ? cc2 : cc1);
				typec_set_input_current_limit(
					port, typec_curr, TYPE_C_VOLTAGE);
#endif
				/* If PD comm is enabled, enable TCPC RX */
				if (pd_comm_enabled)
					tcpm_set_rx_enable(hba, 1);

				/*
				 * fake set data role swapped flag so we send
				 * discover identity when we enter SRC_READY
				 */
				hba->flags |= PD_FLAGS_CHECK_PR_ROLE |
						  PD_FLAGS_CHECK_DR_ROLE |
						  PD_FLAGS_DATA_SWAPPED;
				set_state(hba, PD_STATE_SNK_DISCOVERY);
				timeout = 10;
				hook_call_deferred(
					pd_usb_billboard_deferred,
					PD_T_AME);
				break;
			#endif

			case PD_STATE_SNK_HARD_RESET_RECOVER:
				if (state_changed(hba))
					hba->flags |= PD_FLAGS_DATA_SWAPPED;

#ifdef CONFIG_USB_PD_NO_VBUS_DETECT
				/*
				 * Can't measure vbus state so this is the maximum
				 * recovery time for the source.
				 */
				if (state_changed(hba))
					set_state_timeout(hba, get_time().val +
							  PD_T_SAFE_0V +
							  PD_T_SRC_RECOVER_MAX +
							  PD_T_SRC_TURN_ON,
							  PD_STATE_SNK_DISCONNECTED);
#else
				/* Wait for VBUS to go low and then high*/
				if (state_changed(hba))
				{
					snk_hard_reset_vbus_off = 0;
					set_state_timeout(hba, PD_T_SAFE_0V,
						(hard_reset_count < PD_HARD_RESET_COUNT) ?
						PD_STATE_HARD_RESET_SEND : PD_STATE_SNK_DISCOVERY);
				}

				timeout = 50;
				if (typec_is_vbus_present(hba, TYPEC_VSAFE_0V) && !snk_hard_reset_vbus_off) {
					/* VBUS has gone low, reset timeout */
					snk_hard_reset_vbus_off = 1;
					set_state_timeout(hba, (PD_T_SRC_RECOVER_MAX + PD_T_SRC_TURN_ON),
							  PD_STATE_SNK_UNATTACH);
				}

				if (typec_is_vbus_present(hba, TYPEC_VSAFE_5V) && snk_hard_reset_vbus_off) {
					/* VBUS went high again */
					set_state(hba, PD_STATE_SNK_DISCOVERY);
					timeout = 10;

					pd_complete_hr(hba);

					pd_bist_enable(hba, 0, 0);
				}

				/*
				 * Don't need to set timeout because VBUS changing
				 * will trigger an interrupt and wake us up.
				 */
#endif
				break;

			case PD_STATE_SNK_ATTACH:
				/* reset message ID  on connection */
				pd_set_msg_id(hba, 0);
				/* initial data role for sink is UFP */
				pd_set_power_role(hba, PD_ROLE_SINK, 1);
				if (!(hba->flags & PD_FLAGS_POWER_SWAPPED)) //don't update DR after SRC->SNK power role swap
					pd_set_data_role(hba, PD_ROLE_UFP);
				else
					hba->flags &= ~PD_FLAGS_POWER_SWAPPED; //PR SWAP path 1
#ifdef CONFIG_CHARGE_MANAGER
				typec_curr = get_typec_current_limit(
					pd[port].polarity ? cc2 : cc1);
				typec_set_input_current_limit(
					port, typec_curr, TYPE_C_VOLTAGE);
#endif

				#if !PD_DVT
				if (hba->pd_comm_enabled)
				#endif
					pd_rx_enable(hba, 1);

				/*
				 * fake set data role swapped flag so we send
				 * discover identity when we enter SRC_READY
				 */
				hba->flags |= (PD_FLAGS_CHECK_PR_ROLE | PD_FLAGS_CHECK_DR_ROLE | PD_FLAGS_DATA_SWAPPED);
				hard_reset_count = 0;
				timeout = 10;
				set_state(hba, PD_STATE_SNK_DISCOVERY);
				break;

			case PD_STATE_SNK_DISCOVERY:
				/* Wait for source cap expired only if we are enabled */
				#if PD_DVT
				if (state_changed(hba))
				#else
				if (state_changed(hba) && hba->pd_comm_enabled)
				#endif
				{
					hba->flags &= ~PD_FLAGS_POWER_SWAPPED; //PR SWAP path 2

					/*
					 * If we haven't passed hard reset counter,
					 * start SinkWaitCapTimer, otherwise start
					 * NoResponseTimer.
					 */
					if (hard_reset_count < PD_HARD_RESET_COUNT)
						set_state_timeout(hba, PD_T_SINK_WAIT_CAP, PD_STATE_HARD_RESET_SEND);
					else if (hba->flags & PD_FLAGS_PREVIOUS_PD_CONN)
						/* ErrorRecovery */
						set_state_timeout(hba, PD_T_NO_RESPONSE, PD_STATE_SNK_UNATTACH);
#ifdef CONFIG_CHARGE_MANAGER
					/*
					 * If we didn't come from disconnected, must
					 * have come from some path that did not set
					 * typec current limit. So, set to 0 so that
					 * we guarantee this is revised below.
					 */
					if (hba->last_state !=
					    PD_STATE_SNK_UNATTACH)
						typec_curr = 0;
#endif
				}

#ifdef CONFIG_CHARGE_MANAGER
				timeout = PD_T_SINK_ADJ - PD_T_DEBOUNCE;

				/* Check if CC pull-up has changed */
				tcpm_get_cc(hba, &cc1, &cc2);
				if (hba->polarity)
					cc1 = cc2;
				if (typec_curr != get_typec_current_limit(cc1)) {
					/* debounce signal by requiring two reads */
					if (typec_curr_change) {
						/* set new input current limit */
						typec_curr = get_typec_current_limit(
								cc1);
						typec_set_input_current_limit(
						  port, typec_curr, TYPE_C_VOLTAGE);
					} else {
						/* delay for debounce */
						timeout = PD_T_DEBOUNCE;
					}
					typec_curr_change = !typec_curr_change;
				} else {
					typec_curr_change = 0;
				}
#endif
				break;

			case PD_STATE_SNK_REQUESTED:
				/* Wait for ACCEPT or REJECT */
				if (state_changed(hba))
				{
					hard_reset_count = 0;
					set_state_timeout(hba, PD_T_SENDER_RESPONSE, PD_STATE_HARD_RESET_SEND);
				}
				break;

			case PD_STATE_SNK_TRANSITION:
				/* Wait for PS_RDY */
				if (state_changed(hba))
					set_state_timeout(hba, PD_T_PS_TRANSITION, PD_STATE_HARD_RESET_SEND);
				break;

			case PD_STATE_SNK_READY:
				complete(&hba->ready);

				timeout = 20;

				/*
				 * Don't send any PD traffic if we woke up due to
				 * incoming packet or if VDO response pending to avoid
				 * collisions.
				 */
				if (incoming_packet || (hba->vdm_state == VDM_STATE_BUSY))
					break;

				/* Check for new power to request */
				if (hba->new_power_request)
				{
					pd_send_request_msg(hba, 0);
					break;
				}

				/* Check power role policy, which may trigger a swap */
				if (hba->flags & PD_FLAGS_CHECK_PR_ROLE)
				{
					pd_check_pr_role(hba, PD_ROLE_SINK, hba->flags);
					hba->flags &= ~PD_FLAGS_CHECK_PR_ROLE;
					break;
				}

				/* Check data role policy, which may trigger a swap */
				if (hba->flags & PD_FLAGS_CHECK_DR_ROLE)
				{
					pd_check_dr_role(hba, hba->data_role, hba->flags);
					hba->flags &= ~PD_FLAGS_CHECK_DR_ROLE;
					break;
				}

				#if 0
				/* If DFP, send discovery SVDMs */
				if (hba->data_role == PD_ROLE_DFP &&
				     (hba->flags & PD_FLAGS_DATA_SWAPPED)) {
					pd_send_vdm(hba, USB_SID_PD,
						    CMD_DISCOVER_IDENT, NULL, 0);
					hba->flags &= ~PD_FLAGS_DATA_SWAPPED;
					break;
				}
				#endif

				/* Sent all messages, don't need to wake very often */
				timeout = 200;
				break;

			case PD_STATE_SNK_SWAP_INIT:
				if (state_changed(hba))
				{
					if (send_control(hba, PD_CTRL_PR_SWAP))
					{
						timeout = 10;

						#if 0
						/*
						 * If failed to get goodCRC, send
						 * soft reset, otherwise ignore
						 * failure.
						 */
						set_state(hba, res == -1 ?
							   PD_STATE_SOFT_RESET :
							   PD_STATE_SNK_READY);
						#else
						set_state(hba, PD_STATE_SOFT_RESET);
						#endif

						break;
					}

					/* Wait for accept or reject */
					set_state_timeout(hba, PD_T_SENDER_RESPONSE, PD_STATE_SNK_READY);
				}
				break;

			case PD_STATE_SNK_SWAP_SNK_DISABLE:
				#if 0
				/* Stop drawing power */
				pd_set_input_current_limit(hba, 0, 0);
#ifdef CONFIG_CHARGE_MANAGER
				typec_set_input_current_limit(hba, 0, 0);
				charge_manager_set_ceil(hba,
							CEIL_REQUESTOR_PD,
							CHARGE_CEIL_NONE);
#endif
				#endif

				typec_vbus_det_enable(hba, 0);

				timeout = 10;
				set_state(hba, PD_STATE_SNK_SWAP_SRC_DISABLE);
				break;

			case PD_STATE_SNK_SWAP_SRC_DISABLE:
				/* Wait for PS_RDY */
				if (state_changed(hba))
				{
					set_state_timeout(hba,
						PD_T_PS_SOURCE_OFF, PD_STATE_HARD_RESET_SEND);
				}
				break;

			case PD_STATE_SNK_SWAP_STANDBY:
				if (state_changed(hba))
				{
					//Switch to Rp
					pd_set_power_role(hba, PD_ROLE_SOURCE, 0);
					typec_vbus_det_enable(hba, 1);

					if (!pd_set_power_supply_ready(hba))
					{
						//Switch back to Rd
						pd_set_power_role(hba, PD_ROLE_SINK, 0);

						timeout = 10;
						set_state(hba, PD_STATE_SNK_UNATTACH);

						break;
					}

					/* Wait for power supply to turn on */
					#if PD_SW_WORKAROUND4
					set_state_timeout(hba,
						PD_POWER_SUPPLY_TURN_ON_DELAY1, PD_STATE_SNK_SWAP_COMPLETE);
					#else
					set_state_timeout(hba,
						PD_POWER_SUPPLY_TURN_ON_DELAY, PD_STATE_SNK_SWAP_COMPLETE);
					#endif
				}
				break;

			case PD_STATE_SNK_SWAP_COMPLETE:
				/* Send PS_RDY and change to source role */
				ret = send_control(hba, PD_CTRL_PS_RDY);
				if (ret)
				{
					//Switch back to Rd
					pd_set_power_role(hba, PD_ROLE_SINK, 0);
					pd_power_supply_reset(hba);

					timeout = 10;
					set_state(hba, PD_STATE_SNK_UNATTACH);

					break;
				}

				/* Don't send GET_SINK_CAP on swap */
				snk_cap_count = PD_SNK_CAP_RETRIES+1;
				caps_count = 0;
				pd_set_msg_id(hba, 0);
				typec_vbus_present(hba, 0); //successful switch to SRC. vbus_present is no longer needed
				pd_set_power_role(hba, PD_ROLE_SOURCE, 1); //update message header

				timeout = 10;
				set_state(hba, PD_STATE_SRC_DISCOVERY);

				break;

#ifdef CONFIG_USBC_VCONN_SWAP
			case PD_STATE_VCONN_SWAP_SEND:
				if (state_changed(hba))
				{
					if (send_control(hba, PD_CTRL_VCONN_SWAP))
					{
						timeout = 10;

						#if 0
						/*
						 * If failed to get goodCRC, send
						 * soft reset, otherwise ignore
						 * failure.
						 */
						set_state(hba, res == -1 ?
							   PD_STATE_SOFT_RESET :
							   READY_RETURN_STATE(hba));
						#else
						set_state(hba, PD_STATE_SOFT_RESET);
						#endif

						break;
					}

					/* Wait for accept or reject */
					set_state_timeout(hba, PD_T_SENDER_RESPONSE, READY_RETURN_STATE(hba));
				}
				break;

			case PD_STATE_VCONN_SWAP_INIT:
				if (state_changed(hba))
				{
					if (!(hba->flags & PD_FLAGS_VCONN_ON))
					{
						/* Turn VCONN on and wait for it */
						typec_drive_vconn(hba, 1);
						set_state_timeout(hba, PD_VCONN_SWAP_DELAY, PD_STATE_VCONN_SWAP_READY);
					}
					else
					{
						set_state_timeout(hba,
							PD_T_VCONN_SOURCE_ON, READY_RETURN_STATE(hba));
					}
				}
				break;

			case PD_STATE_VCONN_SWAP_READY:
				if (state_changed(hba))
				{
					if (!(hba->flags & PD_FLAGS_VCONN_ON))
					{
						/* VCONN is now on, send PS_RDY */
						hba->flags |= PD_FLAGS_VCONN_ON;

						if (send_control(hba, PD_CTRL_PS_RDY))
						{
							timeout = 10;

							/*
							 * If failed to get goodCRC,
							 * send soft reset
							 */
							set_state(hba, PD_STATE_SOFT_RESET);

							break;
						}

						set_state(hba, READY_RETURN_STATE(hba));
					}
					else
					{
						/* Turn VCONN off and wait for it */
						typec_drive_vconn(hba, 0);
						hba->flags &= ~PD_FLAGS_VCONN_ON;

						set_state_timeout(hba, PD_VCONN_SWAP_DELAY, READY_RETURN_STATE(hba));
					}
				}
				break;
#endif /* CONFIG_USBC_VCONN_SWAP */
#endif /* CONFIG_USB_PD_DUAL_ROLE */

			case PD_STATE_SOFT_RESET:
				if (state_changed(hba))
				{
					/* if soft reset failed, try hard reset. */
					if (send_control(hba, PD_CTRL_SOFT_RESET))
					{
						timeout = 5;
						set_state(hba, PD_STATE_HARD_RESET_SEND);

						break;
					}

					set_state_timeout(hba,
						PD_T_SENDER_RESPONSE, PD_STATE_HARD_RESET_SEND);
				}
				break;

			case PD_STATE_HARD_RESET_SEND:
				if (state_changed(hba))
					hard_reset_sent = 0;

#ifdef CONFIG_CHARGE_MANAGER
				if (hba->last_state == PD_STATE_SNK_DISCOVERY)
				{
					/*
					 * If discovery timed out, assume that we
					 * have a dedicated charger attached. This
					 * may not be a correct assumption according
					 * to the specification, but it generally
					 * works in practice and the harmful
					 * effects of a wrong assumption here
					 * are minimal.
					 */
					charge_manager_update_dualrole(hba,
								       CAP_DEDICATED);
				}
#endif

				/* try sending hard reset until it succeeds */
				if (!hard_reset_sent)
				{
					hard_reset_count++;

					if (pd_transmit(hba, PD_TX_HARD_RESET, 0, NULL))
					{
						timeout = 10;

						break;
					}

					/* successfully sent hard reset */
					hard_reset_sent = 1;

					/*
					 * If we are source, delay before cutting power
					 * to allow sink time to get hard reset.
					 */
					if (hba->power_role == PD_ROLE_SOURCE)
					{
						set_state_timeout(hba,
							PD_T_PS_HARD_RESET, PD_STATE_HARD_RESET_EXECUTE);
					}
					else
					{
						set_state(hba,
							PD_STATE_HARD_RESET_EXECUTE);
						timeout = 10;
					}
				}
				break;

			case PD_STATE_HARD_RESET_EXECUTE:
				#if 0 //covered by pd_execute_hard_reset
#ifdef CONFIG_USB_PD_DUAL_ROLE
				/*
				 * If hard reset while in the last stages of power
				 * swap, then we need to restore our CC resistor.
				 */
				if (hba->last_state == PD_STATE_SNK_SWAP_STANDBY)
					pd_set_power_role(hba, PD_ROLE_SINK);
#endif
				#endif

				/* reset our own state machine */
				pd_execute_hard_reset(hba);
				timeout = 10;
				break;

#ifdef CONFIG_COMMON_RUNTIME
			case PD_STATE_BIST_CMD:
				if (state_changed(hba))
				{
					send_bist_cmd(hba, hba->bist_mode);
					/* Delay at least enough for partner to finish BIST */
					timeout = PD_T_BIST_RECEIVE + 20;

					#if 0
					/* Set to appropriate port disconnected state */
					set_state(hba, DUAL_ROLE_IF_ELSE(hba, PD_STATE_SNK_UNATTACH, PD_STATE_SRC_UNATTACH));
					#else
					set_state(hba, DUAL_ROLE_IF_ELSE(hba, PD_STATE_SNK_READY, PD_STATE_SRC_READY));
					#endif
				}
				break;

			case PD_STATE_BIST_CARRIER_MODE_2:
				if (state_changed(hba))
				{
					/* Start BIST_CARRIER_MODE2 */
					pd_bist_enable(hba, 1, BDO_MODE_CARRIER2);

					/* Delay at least enough to finish sending BIST */
					timeout = PD_T_BIST_TRANSMIT + 20;

					/* Set to appropriate port disconnected state */
					set_state(hba, DUAL_ROLE_IF_ELSE(hba, PD_STATE_SNK_UNATTACH, PD_STATE_SRC_UNATTACH));
				}
				break;

			case PD_STATE_BIST_TEST_DATA:
				//leave this state by hard reset
				if (state_changed(hba))
					pd_bist_enable(hba, 1, BDO_MODE_TEST_DATA);
				break;

#endif
			default:
				break;
		}

		hba->last_state = curr_state;


		//timeout check
		typec_sw_probe(hba, DBG_LOOP_STATE, (DBG_LOOP_CHK_TIMEOUT<<DBG_LOOP_STATE_OFST));
		if (hba->timeout_state != PD_STATE_NO_TIMEOUT)
		{
			 //already timeout, run next state directly
			#if PD_SW_WORKAROUND3
			if (time_after_16(pd_get_jiffies(hba), hba->timeout_jiffies))
			{
				set_state(hba, hba->timeout_state);
				timeout = 0;
			}
			#else
			if (time_after(jiffies, hba->timeout_jiffies))
			{
				set_state(hba, hba->timeout_state);
				timeout = 0;
			}
			#endif
			else if (timeout > hba->timeout_ms) //not yet timeout, but very soon
			{
				timeout = hba->timeout_ms;
			}
		}

		typec_sw_probe(hba, DBG_LOOP_STATE, (DBG_LOOP_CHK_TIMEOUT<<DBG_LOOP_STATE_OFST));
	}
}

////////////////////////////////////////////////////////////////////////////

#define STRESS_DELAY 2500

int ts_pd(struct file *file, int argc, char** argv)
{
	int ret, cnt, i;
	unsigned long ms_timeout;
	enum pd_states state;
	struct typec_hba *hba = file->private_data;


	if (argc == 2)
	{
		if (!strncasecmp(argv[1], "dump", 4))
			ret = 0;
		else
			ret = 1;
	}
	else if (argc == 3)
	{
		ret = 0;

		if (!strncasecmp(argv[1], "comm", 4))
		{
			if (!strncasecmp(argv[2], "on", 2))
				pd_rx_enable(hba, 1);
			else if (!strncasecmp(argv[2], "off", 3))
				pd_rx_enable(hba, 0);
			else
				ret = 1;
		}
		else if (!strncasecmp(argv[1], "ping", 4))
		{
			if (!strncasecmp(argv[2], "on", 2))
				pd_ping_enable(hba, 1);
			else if (!strncasecmp(argv[2], "off", 3))
				pd_ping_enable(hba, 0);
			else
				ret = 1;
		}
		else if (!strncasecmp(argv[1], "bist", 4))
		{
			ret = 0;
			cnt = simple_strtol(argv[2], &argv[2], 10);

			if (cnt == 0)
			{
				hba->bist_mode = BDO_MODE_CARRIER2;
				set_state(hba, PD_STATE_BIST_CMD);
			}
			else if (cnt == 1)
			{
				hba->bist_mode = BDO_MODE_TEST_DATA;
				set_state(hba, PD_STATE_BIST_CMD);
			}
			else
			{
				ret = 1;
			}
		}
		else if (!strncasecmp(argv[1], "timeout", 7))
		{
			hba->timeout_user = simple_strtol(argv[2], &argv[2], 10);
			dev_err(hba->dev, "set timeout to %lu\n", hba->timeout_user);
		}
		else if (!strncasecmp(argv[1], "soft", 4))
		{
			cnt = simple_strtol(argv[2], &argv[2], 10);
			for (i = 0; i < cnt; i++)
			{
				dev_err(hba->dev, "\n\n%d/%d\n\n", (i+1), cnt);
				init_completion(&hba->ready);

				set_state(hba, PD_STATE_SOFT_RESET);

				if (!wait_for_completion_timeout(&hba->ready, pd_msecs_to_jiffies(STRESS_DELAY)))
					return 1;
			}
		}
		else if (!strncasecmp(argv[1], "hard", 4))
		{
			cnt = simple_strtol(argv[2], &argv[2], 10);
			for (i = 0; i < cnt; i++)
			{
				dev_err(hba->dev, "\n\n%d/%d\n\n", (i+1), cnt);
				init_completion(&hba->ready);

				set_state(hba, PD_STATE_HARD_RESET_SEND);

				if (!wait_for_completion_timeout(&hba->ready, pd_msecs_to_jiffies(STRESS_DELAY)))
					return 1;
			}
		}
		else
		{
			ret = 1;
		}
	}
	else if (argc == 4)
	{
		ret = 0;

		if (!strncasecmp(argv[1], "swap", 4))
		{
			cnt = simple_strtol(argv[3], &argv[3], 10);
			for (i = 0; i < cnt; i++)
			{
				dev_err(hba->dev, "\n\n%d/%d\n\n", (i+1), cnt);
				init_completion(&hba->ready);

				if (!strncasecmp(argv[2], "power", 5))
					pd_request_power_swap(hba);
				else if (!strncasecmp(argv[2], "data", 4))
					pd_request_data_swap(hba);
				else if (!strncasecmp(argv[2], "vconn", 5))
					pd_request_vconn_swap(hba);
				else
					return 1;

				if (!wait_for_completion_timeout(&hba->ready, pd_msecs_to_jiffies(STRESS_DELAY)))
					return 1;
			}
		}
		else
		{
			ret = 1;
		}
	}
	else
	{
		ret = 1;
	}


err_handle:
	return ret;
}

#endif
