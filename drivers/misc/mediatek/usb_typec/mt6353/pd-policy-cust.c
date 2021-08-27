#include "typec.h"

#if SUPPORT_PD

#include "usb_pd_func.h"

////////////////////////////////////////////////////////////////////////////

extern void typec_drive_vbus(struct typec_hba *hba, uint8_t on);

////////////////////////////////////////////////////////////////////////////

#define PDO_FIXED_FLAGS (PDO_FIXED_DATA_SWAP | PDO_FIXED_EXTERNAL)

/* TODO: fill in correct source and sink capabilities */
uint32_t pd_src_pdo[] = {
		PDO_FIXED(5000, 3000, PDO_FIXED_FLAGS),
};
int pd_src_pdo_cnt = ARRAY_SIZE(pd_src_pdo);

uint32_t pd_snk_pdo[] = {
		PDO_FIXED(5000, 500, PDO_FIXED_FLAGS),
		#if 0
		PDO_BATT(5000, 20000, 15000),
		PDO_VAR(5000, 20000, 3000),
		#endif
};
int pd_snk_pdo_cnt = ARRAY_SIZE(pd_snk_pdo);

////////////////////////////////////////////////////////////////////////////

int pd_is_valid_input_voltage(int mv)
{
	return 1;
}

int pd_check_requested_voltage(uint32_t rdo)
{
	int max_ma = rdo & 0x3FF;
	int op_ma = (rdo >> 10) & 0x3FF;
	int idx = rdo >> 28;
	uint32_t pdo;
	uint32_t pdo_ma;

	if (!idx || idx > pd_src_pdo_cnt)
		return 1; //EC_ERROR_INVAL; /* Invalid index */

	/* check current ... */
	pdo = pd_src_pdo[idx - 1];
	pdo_ma = (pdo & 0x3ff);
	if (op_ma > pdo_ma)
		return 1; //EC_ERROR_INVAL; /* too much op current */
	if (max_ma > pdo_ma)
		return 1; //EC_ERROR_INVAL; /* too much max current */

	#if 0
	dev_err(hba->dev, "Requested %d V %d mA (for %d/%d mA)\n",
		 ((pdo >> 10) & 0x3ff) * 50, (pdo & 0x3ff) * 10,
		 ((rdo >> 10) & 0x3ff) * 10, (rdo & 0x3ff) * 10);
	#endif

	return 0; //EC_SUCCESS;
}

void pd_transition_voltage(int idx)
{
	/* No-operation: we are always 5V */
}

int pd_set_power_supply_ready(struct typec_hba *hba)
{
	typec_drive_vbus(hba, 1);

	return 1;

	#if 0
	/* Disable charging */
	gpio_set_level(port ? GPIO_USB_C1_CHARGE_L :
			      GPIO_USB_C0_CHARGE_L, 1);
	/* Provide VBUS */
	gpio_set_level(port ? GPIO_USB_C1_5V_EN :
			      GPIO_USB_C0_5V_EN, 1);

	return EC_SUCCESS; /* we are ready */
	#endif
}

void pd_power_supply_reset(struct typec_hba *hba)
{
	/* Disable VBUS */
	typec_drive_vbus(hba, 0);

	#if 0
	pd_send_host_event(PD_EVENT_POWER_CHANGE);
	#endif
}

void pd_set_input_current_limit(struct typec_hba *hba, uint32_t max_ma,
				uint32_t supply_voltage)
{
#ifdef CONFIG_CHARGE_MANAGER
	struct charge_port_info charge;
	charge.current = max_ma;
	charge.voltage = supply_voltage;
	charge_manager_update_charge(CHARGE_SUPPLIER_PD, port, &charge);
#endif
	/* notify host of power info change */
}

void typec_set_input_current_limit(struct typec_hba *hba, uint32_t max_ma,
				   uint32_t supply_voltage)
{
#ifdef CONFIG_CHARGE_MANAGER
	struct charge_port_info charge;
	charge.current = max_ma;
	charge.voltage = supply_voltage;
	charge_manager_update_charge(CHARGE_SUPPLIER_TYPEC, port, &charge);
#endif

	/* notify host of power info change */
}

#if 0
int pd_board_checks(void)
{
#if BOARD_REV <= OAK_REV3
	/* wake up VBUS task to check vbus change */
	task_wake(TASK_ID_VBUS);
#endif
	return EC_SUCCESS;
}
#endif

int pd_check_power_swap(struct typec_hba *hba)
{
	/*
	 * Allow power swap as long as we are acting as a dual role device,
	 * otherwise assume our role is fixed (not in S0 or console command
	 * to fix our role).
	 */
	return ((hba->support_role == TYPEC_ROLE_DRP) ? 1 : 0);
}

int pd_check_vconn_swap(struct typec_hba *hba)
{
	return 1;
}

int pd_check_data_swap(struct typec_hba *hba)
{
	#if 0
	/* Allow data swap if we are a UFP, otherwise don't allow */
	return ((hba->data_role == PD_ROLE_UFP) ? 1 : 0);
	#else
	return 1;
	#endif
}

void pd_execute_data_swap(struct typec_hba *hba, int data_role)
{
}

void pd_check_pr_role(struct typec_hba *hba, int pr_role, int flags)
{
	#if PD_DVT
	pd_request_power_swap(hba);
	#else
	/*
	 * If partner is dual-role power and dualrole toggling is on, consider
	 * if a power swap is necessary.
	 */
	if ((flags & PD_FLAGS_PARTNER_DR_POWER) &&
		(hba->support_role == TYPEC_ROLE_DRP))
	{
		/*
		 * If we are a sink and partner is not externally powered, then
		 * swap to become a source. If we are source and partner is
		 * externally powered, swap to become a sink.
		 */
		int partner_extpower = flags & PD_FLAGS_PARTNER_EXTPOWER;
		if ((!partner_extpower && pr_role == PD_ROLE_SINK) ||
		     (partner_extpower && pr_role == PD_ROLE_SOURCE))
			pd_request_power_swap(hba);
	}
	#endif
}

void pd_check_dr_role(struct typec_hba *hba, int dr_role, int flags)
{
	#if PD_DVT
	#else
	/* If UFP, try to switch to DFP */
	if ((flags & PD_FLAGS_PARTNER_DR_DATA) && dr_role == PD_ROLE_UFP)
		pd_request_data_swap(hba);
	#endif
}

#if 0
/* ----------------- Vendor Defined Messages ------------------ */
const struct svdm_response svdm_rsp = {
	.identity = NULL,
	.svids = NULL,
	.modes = NULL,
};

int pd_custom_vdm(struct typec_hba *hba, int cnt, uint32_t *payload,
		  uint32_t **rpayload)
{
	int cmd = PD_VDO_CMD(payload[0]);
	uint16_t dev_id = 0;
	int is_rw;

	/* make sure we have some payload */
	if (cnt == 0)
		return 0;

	switch (cmd) {
	case VDO_CMD_VERSION:
		/* guarantee last byte of payload is null character */
		*(payload + cnt - 1) = 0;
		CPRINTF("version: %s\n", (char *)(payload+1));
		break;
	case VDO_CMD_READ_INFO:
	case VDO_CMD_SEND_INFO:
		/* copy hash */
		if (cnt == 7) {
			dev_id = VDO_INFO_HW_DEV_ID(payload[6]);
			is_rw = VDO_INFO_IS_RW(payload[6]);

			CPRINTF("DevId:%d.%d SW:%d RW:%d\n",
				HW_DEV_ID_MAJ(dev_id),
				HW_DEV_ID_MIN(dev_id),
				VDO_INFO_SW_DBG_VER(payload[6]),
				is_rw);
		} else if (cnt == 6) {
			/* really old devices don't have last byte */
			pd_dev_store_rw_hash(port, dev_id, payload + 1,
					     SYSTEM_IMAGE_UNKNOWN);
		}
		break;
	case VDO_CMD_CURRENT:
		CPRINTF("Current: %dmA\n", payload[1]);
		break;
	case VDO_CMD_FLIP:
		/* board_flip_usb_mux(port); */
		break;
#ifdef CONFIG_USB_PD_LOGGING
	case VDO_CMD_GET_LOG:
		pd_log_recv_vdm(port, cnt, payload);
		break;
#endif /* CONFIG_USB_PD_LOGGING */
	}

	return 0;
}

#ifdef CONFIG_USB_PD_ALT_MODE_DFP
static int dp_flags[CONFIG_USB_PD_PORT_COUNT];
/* DP Status VDM as returned by UFP */
static uint32_t dp_status[CONFIG_USB_PD_PORT_COUNT];

static void svdm_safe_dp_mode(struct typec_hba *hba)
{
	const char *dp_str, *usb_str;
	enum typec_mux typec_mux_setting;

	/* make DP interface safe until configure */
	dp_flags[port] = 0;
	dp_status[port] = 0;

	/*
	 * Check current status, due to the mux may be switched to SS
	 * and SS device was attached before (for example: Type-C dock).
	 * To avoid broken the SS connection,
	 * keep the current setting if SS connection is enabled already.
	 */
	typec_mux_setting = (usb_mux_get(port, &dp_str, &usb_str) && usb_str) ?
			    TYPEC_MUX_USB : TYPEC_MUX_NONE;
	usb_mux_set(port, typec_mux_setting,
		    USB_SWITCH_CONNECT, pd_get_polarity(port));
}

static int svdm_enter_dp_mode(struct typec_hba *hba, uint32_t mode_caps)
{
	/* Only enter mode if device is DFP_D capable */
	if (mode_caps & MODE_DP_SNK) {
		svdm_safe_dp_mode(port);
		return 0;
	}

	return -1;
}

static int svdm_dp_status(struct typec_hba *hba, uint32_t *payload)
{
	int opos = pd_alt_mode(port, USB_SID_DISPLAYPORT);
	payload[0] = VDO(USB_SID_DISPLAYPORT, 1,
			 CMD_DP_STATUS | VDO_OPOS(opos));
	payload[1] = VDO_DP_STATUS(0, /* HPD IRQ  ... not applicable */
				   0, /* HPD level ... not applicable */
				   0, /* exit DP? ... no */
				   0, /* usb mode? ... no */
				   0, /* multi-function ... no */
				   (!!(dp_flags[port] & DP_FLAGS_DP_ON)),
				   0, /* power low? ... no */
				   (!!(dp_flags[port] & DP_FLAGS_DP_ON)));
	return 2;
};

static int svdm_dp_config(struct typec_hba *hba, uint32_t *payload)
{
	int opos = pd_alt_mode(port, USB_SID_DISPLAYPORT);
	int mf_pref = PD_VDO_DPSTS_MF_PREF(dp_status[port]);
	int pin_mode = pd_dfp_dp_get_pin_mode(port, dp_status[port]);

	if (!pin_mode)
		return 0;

	usb_mux_set(port, mf_pref ? TYPEC_MUX_DOCK : TYPEC_MUX_DP,
			  USB_SWITCH_CONNECT, pd_get_polarity(port));

	payload[0] = VDO(USB_SID_DISPLAYPORT, 1,
			 CMD_DP_CONFIG | VDO_OPOS(opos));

	payload[1] = VDO_DP_CFG(pin_mode,      /* pin mode */
				1,             /* DPv1.3 signaling */
				2);            /* UFP_U connected as UFP_D */
	return 2;
};

static void svdm_dp_post_config(struct typec_hba *hba)
{
	dp_flags[port] |= DP_FLAGS_DP_ON;
	if (!(dp_flags[port] & DP_FLAGS_HPD_HI_PENDING))
		return;
	board_typec_dp_set(port, 1);
}

static int svdm_dp_attention(struct typec_hba *hba, uint32_t *payload)
{
	int cur_lvl;
	int lvl = PD_VDO_DPSTS_HPD_LVL(payload[1]);
	int irq = PD_VDO_DPSTS_HPD_IRQ(payload[1]);

	dp_status[port] = payload[1];
	cur_lvl = gpio_get_level(GPIO_USB_DP_HPD);

	/* Its initial DP status message prior to config */
	if (!(dp_flags[port] & DP_FLAGS_DP_ON)) {
		if (lvl)
			dp_flags[port] |= DP_FLAGS_HPD_HI_PENDING;
		return 1;
	}

	if (irq & cur_lvl) {
		board_typec_dp_on(port);
	} else if (irq & !cur_lvl) {
		CPRINTF("ERR:HPD:IRQ&LOW\n");
		return 0; /* nak */
	} else {
		board_typec_dp_set(port, lvl);
	}
	/* ack */
	return 1;
}

static void svdm_exit_dp_mode(struct typec_hba *hba)
{
	svdm_safe_dp_mode(port);
	board_typec_dp_off(port, dp_flags);
}

static int svdm_enter_gfu_mode(struct typec_hba *hba, uint32_t mode_caps)
{
	/* Always enter GFU mode */
	return 0;
}

static void svdm_exit_gfu_mode(struct typec_hba *hba)
{
}

static int svdm_gfu_status(struct typec_hba *hba, uint32_t *payload)
{
	/*
	 * This is called after enter mode is successful, send unstructured
	 * VDM to read info.
	 */
	pd_send_vdm(port, USB_VID_GOOGLE, VDO_CMD_READ_INFO, NULL, 0);
	return 0;
}

static int svdm_gfu_config(struct typec_hba *hba, uint32_t *payload)
{
	return 0;
}

static int svdm_gfu_attention(struct typec_hba *hba, uint32_t *payload)
{
	return 0;
}

const struct svdm_amode_fx supported_modes[] = {
	{
		.svid = USB_SID_DISPLAYPORT,
		.enter = &svdm_enter_dp_mode,
		.status = &svdm_dp_status,
		.config = &svdm_dp_config,
		.post_config = &svdm_dp_post_config,
		.attention = &svdm_dp_attention,
		.exit = &svdm_exit_dp_mode,
	},
	{
		.svid = USB_VID_GOOGLE,
		.enter = &svdm_enter_gfu_mode,
		.status = &svdm_gfu_status,
		.config = &svdm_gfu_config,
		.attention = &svdm_gfu_attention,
		.exit = &svdm_exit_gfu_mode,
	}
};
const int supported_modes_cnt = ARRAY_SIZE(supported_modes);
#endif /* CONFIG_USB_PD_ALT_MODE_DFP */
#endif

#endif
