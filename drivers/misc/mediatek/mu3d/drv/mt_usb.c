/*
 * MUSB OTG controller driver for Blackfin Processors
 *
 * Copyright 2006-2008 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/usb/gadget.h>
/*#ifdef CONFIG_HAS_WAKELOCK*/
#include <linux/wakelock.h>
/*#endif*/
#include "mach/emi_mpu.h"

#include <linux/mu3d/hal/mu3d_hal_osal.h>
#include "musb_core.h"
#if defined(CONFIG_MTK_UART_USB_SWITCH) || defined(CONFIG_MTK_SIB_USB_SWITCH)
#include <linux/mu3phy/mtk-phy-asic.h>
#include <mach/mt_typedefs.h>
#endif

extern struct musb *_mu3d_musb;

#ifdef CONFIG_MTK_UART_USB_SWITCH
typedef enum {
	PORT_MODE_USB = 0,
	PORT_MODE_UART,

	PORT_MODE_MAX
} PORT_MODE;

extern bool in_uart_mode;
extern void uart_usb_switch_dump_register(void);
extern bool usb_phy_check_in_uart_mode(void);
extern void usb_phy_switch_to_usb(void);
extern void usb_phy_switch_to_uart(void);
extern void __iomem *ap_uart0_base;
#endif

#ifndef FOR_BRING_UP
#ifndef CONFIG_MTK_FPGA
extern void wake_up_bat(void);
#endif
#endif

#ifdef FOR_BRING_UP

static inline void BATTERY_SetUSBState(int usb_state)
{
};

static inline CHARGER_TYPE  mt_get_charger_type(void)
{
	return STANDARD_HOST;
};

static inline bool upmu_is_chr_det(void)
{
	return true;
};

static inline u32 upmu_get_rgs_chrdet(void)
{
	return 1;
};

#else				/* NOT CONFIG_ARM64 */

extern bool upmu_is_chr_det(void);
extern void BATTERY_SetUSBState(int usb_state);
extern u32 upmu_get_rgs_chrdet(void);

#endif


unsigned int cable_mode = CABLE_MODE_NORMAL;
#ifdef CONFIG_MTK_UART_USB_SWITCH
u32 port_mode = PORT_MODE_USB;
u32 sw_tx = 0;
u32 sw_rx = 0;
u32 sw_uart_path = 0;
#endif

/* ================================ */
/* connect and disconnect functions */
/* ================================ */
bool mt_usb_is_device(void)
{
#ifndef CONFIG_MTK_FPGA
	bool tmp = mtk_is_host_mode();
	os_printk(K_INFO, "%s mode\n", tmp ? "HOST" : "DEV");

	return !tmp;
#else
	return true;
#endif
}

enum status { INIT, ON, OFF };
#ifdef CONFIG_USBIF_COMPLIANCE
static enum status connection_work_dev_status = INIT;
void init_connection_work(void)
{
	connection_work_dev_status = INIT;
}
#endif

#ifndef CONFIG_USBIF_COMPLIANCE

struct timespec connect_timestamp = { 0, 0 };

void set_connect_timestamp(void)
{
	connect_timestamp = CURRENT_TIME;
	pr_debug("set timestamp = %llu\n", timespec_to_ns(&connect_timestamp));
}

void clr_connect_timestamp(void)
{
	connect_timestamp.tv_sec = 0;
	connect_timestamp.tv_nsec = 0;
	pr_debug("clr timestamp = %llu\n", timespec_to_ns(&connect_timestamp));
}

struct timespec get_connect_timestamp(void)
{
	pr_debug("get timestamp = %llu\n", timespec_to_ns(&connect_timestamp));
	return connect_timestamp;
}
#endif

void connection_work(struct work_struct *data)
{
	struct musb *musb = container_of(to_delayed_work(data), struct musb, connection_work);
#ifndef CONFIG_USBIF_COMPLIANCE
	static enum status connection_work_dev_status = INIT;
#endif
#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (!usb_phy_check_in_uart_mode()) {
#endif
		bool is_usb_cable = usb_cable_connected();
		bool cmode_effect_on = false;
		CHARGER_TYPE chg_type = mt_get_charger_type();
		if (fake_CDP && chg_type == STANDARD_HOST) {
			os_printk(K_INFO, "%s, fake to type 2\n", __func__);
			chg_type = CHARGING_HOST;
		}
		os_printk(K_NOTICE, "%s type=%d\n", __func__, chg_type);
		if ((musb->usb_mode == CABLE_MODE_HOST_ONLY && chg_type == STANDARD_HOST)
				|| musb->usb_mode == CABLE_MODE_CHRG_ONLY)
			cmode_effect_on = true;
/*
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
		if (g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
			if (chg_type == STANDARD_HOST)
				cmode_effect_on = true;

		}
#endif
*/
		os_printk(K_INFO, "%s type=%d, cmode_effect_on=%d, usb_mode:%d\n",
				__func__, chg_type, cmode_effect_on, musb->usb_mode);

#ifndef CONFIG_MTK_FPGA
		if (!mt_usb_is_device()) {
			connection_work_dev_status = OFF;
			usb_fake_powerdown(musb->is_clk_on);
			musb->is_clk_on = 0;
			os_printk(K_INFO, "%s, Host mode. directly return\n", __func__);
			return;
		}
#endif

		os_printk(K_INFO, "%s musb %s, cable %s\n", __func__,
			  ((connection_work_dev_status ==
			    0) ? "INIT" : ((connection_work_dev_status == 1) ? "ON" : "OFF")),
			  (is_usb_cable ? "IN" : "OUT"));

		if ((is_usb_cable == true) && (connection_work_dev_status != ON)
		    && (!cmode_effect_on)) {

			connection_work_dev_status = ON;
#ifndef CONFIG_USBIF_COMPLIANCE
			set_connect_timestamp();
#endif

#ifdef CONFIG_HAS_WAKELOCK
			if (!wake_lock_active(&musb->usb_wakelock))
				wake_lock(&musb->usb_wakelock);
#else
			if (!wake_lock_active(&musb->usb_wakelock))
				__pm_stay_awake(&musb->usb_wakelock);
#endif

			/* FIXME: Should use usb_udc_start() & usb_gadget_connect(), like usb_udc_softconn_store().
			 * But have no time to think how to handle. However i think it is the correct way.*/
			musb_start(musb);

			os_printk(K_INFO, "%s ----Connect----\n", __func__);
		} else if (((is_usb_cable == false) && (connection_work_dev_status != OFF))
			   || (cmode_effect_on)) {

			connection_work_dev_status = OFF;
#ifndef CONFIG_USBIF_COMPLIANCE
			clr_connect_timestamp();
#endif

			/*FIXME: we should use usb_gadget_disconnect() & usb_udc_stop().  like usb_udc_softconn_store().
			 * But have no time to think how to handle. However i think it is the correct way.*/
			musb_stop(musb);

#ifdef CONFIG_HAS_WAKELOCK
			if (wake_lock_active(&musb->usb_wakelock))
				wake_unlock(&musb->usb_wakelock);
#else
			if (wake_lock_active(&musb->usb_wakelock))
				__pm_relax(&musb->usb_wakelock);
#endif

			os_printk(K_INFO, "%s ----Disconnect----\n", __func__);
		} else {
			/* This if-elseif is to set wakelock when booting with USB cable.
			 * Because battery driver does _NOT_ notify at this codition.*/
			/* if( (is_usb_cable == true) && !wake_lock_active(&musb->usb_wakelock)) { */
			/* os_printk(K_INFO, "%s Boot wakelock\n", __func__); */
			/* wake_lock(&musb->usb_wakelock); */
			/* } else if( (is_usb_cable == false) && wake_lock_active(&musb->usb_wakelock)) { */
			/* os_printk(K_INFO, "%s Boot unwakelock\n", __func__); */
			/* wake_unlock(&musb->usb_wakelock); */
			/* } */

			os_printk(K_INFO, "%s directly return\n", __func__);
		}
#ifdef CONFIG_MTK_UART_USB_SWITCH
	} else {
#if 0
		usb_fake_powerdown(musb->is_clk_on);
		musb->is_clk_on = 0;
#else
		os_printk(K_INFO, "%s, in UART MODE!!!\n", __func__);
#endif
	}
#endif
}

bool mt_usb_is_ready(void)
{
	os_printk(K_INFO, "USB is ready or not\n");
#ifdef NEVER
	if (!mtk_musb || !mtk_musb->is_ready)
		return false;
	else
		return true;
#endif				/* NEVER */
	return true;
}

void mt_usb_connect(void)
{
	os_printk(K_INFO, "%s+\n", __func__);
	if (_mu3d_musb) {
		struct delayed_work *work;

		work = &_mu3d_musb->connection_work;

		schedule_delayed_work_on(/*0*/ WORK_CPU_UNBOUND, work, 0);
	} else {
		os_printk(K_INFO, "%s musb_musb not ready\n", __func__);
	}
	os_printk(K_INFO, "%s-\n", __func__);
}
EXPORT_SYMBOL_GPL(mt_usb_connect);

void mt_usb_disconnect(void)
{
	os_printk(K_INFO, "%s+\n", __func__);

	if (_mu3d_musb) {
		struct delayed_work *work;

		work = &_mu3d_musb->connection_work;

		schedule_delayed_work_on(/*0*/ WORK_CPU_UNBOUND, work, 0);
	} else {
		os_printk(K_INFO, "%s musb_musb not ready\n", __func__);
	}
	os_printk(K_INFO, "%s-\n", __func__);
}
EXPORT_SYMBOL_GPL(mt_usb_disconnect);

#ifdef CONFIG_MTK_TYPEC_SWITCH
int typec_switch_usb_connect(void *data)
{
	struct musb *musb = data;

	os_printk(K_INFO, "%s+\n", __func__);

	if (musb && musb->gadget_driver) {
		struct delayed_work *work;

		work = &musb->connection_work;

		schedule_delayed_work_on(0, work, 0);
	} else {
		os_printk(K_INFO, "%s musb_musb not ready\n", __func__);
	}
	os_printk(K_INFO, "%s-\n", __func__);

	return 0;
}

int typec_switch_usb_disconnect(void *data)
{
	struct musb *musb = data;

	os_printk(K_INFO, "%s+\n", __func__);

	if (musb && musb->gadget_driver) {
		struct delayed_work *work;

		work = &musb->connection_work;

		schedule_delayed_work_on(0, work, 0);
	} else {
		os_printk(K_INFO, "%s musb_musb not ready\n", __func__);
	}
	os_printk(K_INFO, "%s-\n", __func__);

	return 0;
}
#endif
bool usb_cable_connected(void)
{
#ifndef CONFIG_MTK_FPGA
	CHARGER_TYPE chg_type = CHARGER_UNKNOWN;
#ifdef CONFIG_POWER_EXT
	chg_type = mt_get_charger_type();
	os_printk(K_INFO, "%s ext-chrdet=%d type=%d\n", __func__, upmu_get_rgs_chrdet(), chg_type);
	if (upmu_get_rgs_chrdet() && (chg_type == STANDARD_HOST))
		return true;
#else
	/* IPO shutdown, disable USB and send HWDISCONNECT uevent */
	if (cable_mode == CABLE_MODE_CHRG_ONLY)
		return false;

	if (upmu_is_chr_det()) {
		chg_type = mt_get_charger_type();
		os_printk(K_INFO, "%s type=%d\n", __func__, chg_type);
		if (chg_type == STANDARD_HOST || chg_type == CHARGING_HOST)
			return true;
	}
#endif
	os_printk(K_INFO, "%s no USB Host detect!\n", __func__);
	return false;
#else
	os_printk(K_INFO, "%s [FPGA] always true\n", __func__);
	return true;
#endif
}
EXPORT_SYMBOL_GPL(usb_cable_connected);

#ifdef NEVER
void musb_platform_reset(struct musb *musb)
{
	u16 swrst = 0;
	void __iomem *mbase = musb->mregs;
	swrst = musb_readw(mbase, MUSB_SWRST);
	swrst |= (MUSB_SWRST_DISUSBRESET | MUSB_SWRST_SWRST);
	musb_writew(mbase, MUSB_SWRST, swrst);
}
#endif				/* NEVER */

void usb_check_connect(void)
{
	os_printk(K_INFO, "usb_check_connect\n");

#ifndef CONFIG_MTK_FPGA
	if (usb_cable_connected())
		mt_usb_connect();
#endif

}

void musb_sync_with_bat(struct musb *musb, int usb_state)
{
	os_printk(K_INFO, "musb_sync_with_bat\n");

#ifndef CONFIG_MTK_FPGA
	BATTERY_SetUSBState(usb_state);
#ifndef FOR_BRING_UP
	wake_up_bat();
#endif
#endif

}
EXPORT_SYMBOL_GPL(musb_sync_with_bat);


#ifdef CONFIG_USB_MTK_DUALMODE
bool musb_check_ipo_state(void)
{
	bool ipo_off;
	down(&_mu3d_musb->musb_lock);
	ipo_off = _mu3d_musb->in_ipo_off;
	os_printk(K_INFO, "IPO State is %s\n", (ipo_off ? "true" : "false"));
	up(&_mu3d_musb->musb_lock);
	return ipo_off;
}
#endif

/*--FOR INSTANT POWER ON USAGE--------------------------------------------------*/
static inline struct musb *dev_to_musb(struct device *dev)
{
	return dev_get_drvdata(dev);
}

const char *const usb_mode_str[CABLE_MODE_MAX] = { "CHRG_ONLY", "NORMAL", "HOST_ONLY" };

ssize_t musb_cmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (!dev) {
		os_printk(K_ERR, "dev is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", cable_mode);
}

ssize_t musb_cmode_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned int cmode;
	struct musb *musb = dev_to_musb(dev);

	if (!dev) {
		os_printk(K_ERR, "dev is null!!\n");
		return count;
	} else if (1 == sscanf(buf, "%d", &cmode)) {
		os_printk(K_INFO, "%s %s --> %s\n", __func__, usb_mode_str[cable_mode],
			  usb_mode_str[cmode]);

		if (cmode >= CABLE_MODE_MAX)
			cmode = CABLE_MODE_NORMAL;

		if (cable_mode != cmode) {
			if (_mu3d_musb) {
				if (down_interruptible(&_mu3d_musb->musb_lock))
					os_printk(K_INFO, "%s: busy, Couldn't get musb_lock\n", __func__);
			}
			if (cmode == CABLE_MODE_CHRG_ONLY) {	/* IPO shutdown, disable USB */
				if (_mu3d_musb)
					_mu3d_musb->in_ipo_off = true;
			} else {	/* IPO bootup, enable USB */
				if (_mu3d_musb)
					_mu3d_musb->in_ipo_off = false;
			}

			if (cmode == CABLE_MODE_CHRG_ONLY) {	/* IPO shutdown, disable USB */
				if (musb) {
					musb->usb_mode = CABLE_MODE_CHRG_ONLY;
					mt_usb_disconnect();
				}
			} else if (cmode == CABLE_MODE_HOST_ONLY) {
				if (musb) {
					musb->usb_mode = CABLE_MODE_HOST_ONLY;
					mt_usb_disconnect();
				}
			} else {	/* IPO bootup, enable USB */
				if (musb) {
					musb->usb_mode = CABLE_MODE_NORMAL;
#ifndef CONFIG_MTK_TYPEC_SWITCH
					mt_usb_connect();
#else
					typec_switch_usb_connect(musb);
#endif
				}
			}
			cable_mode = cmode;
#ifdef CONFIG_USB_MTK_DUALMODE
			if (cmode == CABLE_MODE_CHRG_ONLY) {
#ifdef CONFIG_MTK_TYPEC_SWITCH
				;
#else
				/* mask ID pin interrupt even if A-cable is not plugged in */
				switch_int_to_host_and_mask();
				if (mtk_is_host_mode() == true)
					mtk_unload_xhci_on_ipo();
#endif
			} else {
#ifdef CONFIG_MTK_TYPEC_SWITCH
				;
#else
				switch_int_to_host();	/* resotre ID pin interrupt */
#endif
			}
#endif
			if (_mu3d_musb)
				up(&_mu3d_musb->musb_lock);
		}
	}
	return count;
}

#ifdef CONFIG_MTK_UART_USB_SWITCH
ssize_t musb_portmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (!dev) {
		pr_debug("dev is null!!\n");
		return 0;
	}

	if (usb_phy_check_in_uart_mode())
		port_mode = PORT_MODE_UART;
	else
		port_mode = PORT_MODE_USB;

	if (port_mode == PORT_MODE_USB)
		pr_debug("\nUSB Port mode -> USB\n");
	else if (port_mode == PORT_MODE_UART)
		pr_debug("\nUSB Port mode -> UART\n");

	uart_usb_switch_dump_register();

	return scnprintf(buf, PAGE_SIZE, "%d\n", port_mode);
}

ssize_t musb_portmode_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int portmode;

	if (!dev) {
		pr_debug("dev is null!!\n");
		return count;
	} else if (1 == sscanf(buf, "%d", &portmode)) {
		pr_debug("\nUSB Port mode: current => %d (port_mode), change to => %d (portmode)\n",
		       port_mode, portmode);
		if (portmode >= PORT_MODE_MAX)
			portmode = PORT_MODE_USB;

		if (port_mode != portmode) {
			if (portmode == PORT_MODE_USB) {	/* Changing to USB Mode */
				pr_debug("USB Port mode -> USB\n");
				usb_phy_switch_to_usb();
			} else if (portmode == PORT_MODE_UART) {	/* Changing to UART Mode */
				pr_debug("USB Port mode -> UART\n");
				usb_phy_switch_to_uart();
			}
			uart_usb_switch_dump_register();
			port_mode = portmode;
		}
	}
	return count;
}

ssize_t musb_tx_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 var;
	u8 var2;

	if (!dev) {
		pr_debug("dev is null!!\n");
		return 0;
	}

	var = U3PhyReadReg8((u3phy_addr_t) (U3D_U2PHYDTM1 + 0x2));
	var2 = (var >> 3) & ~0xFE;
	pr_debug("[MUSB]addr: 0x6E (TX), value: %x - %x\n", var, var2);

	sw_tx = var;

	return scnprintf(buf, PAGE_SIZE, "%x\n", var2);
}

ssize_t musb_tx_store(struct device *dev, struct device_attribute *attr,
		      const char *buf, size_t count)
{
	unsigned int val;
	u8 var;
	u8 var2;

	if (!dev) {
		pr_debug("dev is null!!\n");
		return count;
	} else if (1 == sscanf(buf, "%d", &val)) {
		pr_debug("\n Write TX : %d\n", val);

#ifdef CONFIG_MTK_FPGA
		var = USB_PHY_Read_Register8(U3D_U2PHYDTM1 + 0x2);
#else
		var = U3PhyReadReg8((u3phy_addr_t) (U3D_U2PHYDTM1 + 0x2));
#endif

		if (val == 0)
			var2 = var & ~(1 << 3);
		else
			var2 = var | (1 << 3);

#ifdef CONFIG_MTK_FPGA
		USB_PHY_Write_Register8(var2, U3D_U2PHYDTM1 + 0x2);
		var = USB_PHY_Read_Register8(U3D_U2PHYDTM1 + 0x2);
#else
		/* U3PhyWriteField32(U3D_USBPHYDTM1+0x2,
			E60802_RG_USB20_BC11_SW_EN_OFST, E60802_RG_USB20_BC11_SW_EN, 0); */
		/* Jeremy TODO 0320 */
		var = U3PhyReadReg8((u3phy_addr_t) (U3D_U2PHYDTM1 + 0x2));
#endif

		var2 = (var >> 3) & ~0xFE;

		pr_debug("[MUSB]addr: U3D_U2PHYDTM1 (0x6E) TX [AFTER WRITE], value after: %x - %x\n",
		       var, var2);
		sw_tx = var;
	}
	return count;
}

ssize_t musb_rx_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 var;
	u8 var2;

	if (!dev) {
		pr_debug("dev is null!!\n");
		return 0;
	}
#ifdef CONFIG_MTK_FPGA
	var = USB_PHY_Read_Register8(U3D_U2PHYDMON1 + 0x3);
#else
	var = U3PhyReadReg8((u3phy_addr_t) (U3D_U2PHYDMON1 + 0x3));
#endif
	var2 = (var >> 7) & ~0xFE;
	pr_debug("[MUSB]addr: U3D_U2PHYDMON1 (0x77) (RX), value: %x - %x\n", var, var2);
	sw_rx = var;

	return scnprintf(buf, PAGE_SIZE, "%x\n", var2);
}

ssize_t musb_uart_path_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 var = 0;

	if (!dev) {
		pr_debug("dev is null!!\n");
		return 0;
	}

	var = DRV_Reg8(ap_uart0_base + 0xB0);
	pr_debug("[MUSB]addr: (UART0) 0xB0, value: %x\n\n", DRV_Reg8(ap_uart0_base + 0xB0));
	sw_uart_path = var;

	return scnprintf(buf, PAGE_SIZE, "%x\n", var);
}
#endif

#ifdef CONFIG_MTK_SIB_USB_SWITCH
ssize_t musb_sib_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;

	if (!dev) {
		pr_debug("dev is null!!\n");
		return 0;
	}
	ret = usb_phy_sib_enable_switch_status();
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

ssize_t musb_sib_enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int mode;

	if (!dev) {
		pr_debug("dev is null!!\n");
		return count;
	} else if (1 == sscanf(buf, "%d", &mode)) {
		pr_debug("\nUSB sib_enable: %d)\n", mode);
		usb_phy_sib_enable_switch(mode);
	}
	return count;
}
#endif

#ifdef VENDOR_EDIT
//Fuchun.Liao@Mobile.BSP.CHG 2015-12-22 add for otg_switch
static bool force_id_polling_on = false;
static bool id_polling_state = false;
static unsigned int id_polling_timeout=30000;
static bool start_id_polling= false;
extern void mtk_xhci_eint_iddig_gpio_mode(void);
extern int mtk_xhci_eint_iddig_init(void);
extern void mtk_xhci_switch_init(void);
extern void mtk_xhci_eint_iddig_deinit(void);
extern int stop_polling_hcd_cleanup();

void somc_chg_usbid_start_polling_delay_work(struct work_struct *work)
{
	struct somc_usb_id *usb_id;
	unsigned long flags;

	os_printk(K_ERR, "usbid_start_work\n");
	usb_id = container_of(work, struct somc_usb_id, start_polling_delay.work);
	os_printk(K_ERR, "switch gpio to iddig irq\n");
	spin_lock(&usb_id->change_irq_lock);
	if (id_polling_state) {
	    spin_unlock(&usb_id->change_irq_lock);
	    return;
	}
	id_polling_state = true;
#if 1
	mtk_xhci_eint_iddig_init();
#endif
	spin_unlock(&usb_id->change_irq_lock);
	os_printk(K_ERR, "usbid_start_work --\n");
}

extern void mtk_xhci_hcd_cleanup(void);
void somc_chg_usbid_stop_polling_delay_work(struct work_struct *work)
{
	struct somc_usb_id *usb_id;
	unsigned long flags;

	os_printk(K_ERR, "usbid_stop_work\n");
	usb_id = container_of(work, struct somc_usb_id,stop_polling_delay.work);
/*
	if(!id_polling_state){
		os_printk(K_WARNIN, "Does need to stop polling\n");
		goto out;
	}
*/
	if (force_id_polling_on) {
	    os_printk(K_ERR, "Force ID polling. Does not stop polling\n");
	    goto out;
	}
	/*
	if (mtk_is_host_mode()) {
	    os_printk(K_ERR, "USB device is connecting. Wait 3s and re-check\n");
	    schedule_delayed_work_on(0, &usb_id->stop_polling_delay,msecs_to_jiffies(3000));
	    goto out;
	}

	if (somc_chg_is_usb_present(params->dev)) {
	    //"Now charging. does not stop polling\n"
	    goto out;
	}
	*/
	os_printk(K_ERR, "switch iddig irq to gpio\n");
	// disable usbid irq
	cancel_delayed_work_sync(&usb_id->start_polling_delay);
	spin_lock(&usb_id->change_irq_lock);
#if 1
	mtk_xhci_eint_iddig_deinit();
	mtk_xhci_eint_iddig_gpio_mode();
#endif
	id_polling_state = false;
	spin_unlock(&usb_id->change_irq_lock);
	stop_polling_hcd_cleanup();	
out:
	usb_id->user_request_polling = false;
	if (usb_id->wakeup_source_id_polling.active)
	    __pm_relax(&usb_id->wakeup_source_id_polling);
	os_printk(K_ERR, " usbid_stop_work --\n");
}

static void somc_chg_usbid_start_polling(struct somc_usb_id *usb_id)
{
	os_printk(K_ERR, "somc_chg_usbid_start_polling !!\n");
	//schedule_delayed_work_on(0, &usb_id->start_polling_delay, 0);
	schedule_delayed_work(&usb_id->start_polling_delay, 0);
}

static void somc_chg_usbid_stop_polling(struct somc_usb_id *usb_id)
{
	os_printk(K_ERR, " somc_chg_usbid_stop_polling--------2 !!\n");
	//stop_polling_hcd_cleanup();
	cancel_delayed_work_sync(&usb_id->stop_polling_delay);
	//schedule_delayed_work_on(0, &usb_id->stop_polling_delay, 0);
	schedule_delayed_work(&usb_id->stop_polling_delay, 0);
}

#ifdef VENDOR_EDIT
// Jingchun.Wang@Phone.Bsp.Driver, 2016/03/28  Add for avoid switch otg fast 
extern bool is_switch_done();
#endif /*CONFIG_VENDOR_EDIT*/

static int set_start_id_polling(void)
{
	int ret=1;
	struct somc_usb_id *usb_id;

	os_printk(K_ERR, "[MU3D]%s \n", __func__);
	if(_mu3d_musb) {
		usb_id = &_mu3d_musb->usb_id;
	} else {
		os_printk(K_ERR, "not yet initialized\n");
		return -ENODEV;
	}

#ifdef VENDOR_EDIT
// Jingchun.Wang@Phone.Bsp.Driver, 2016/03/28  Add for avoid switch otg fast 
	if(is_switch_done() == false)
		return -EBUSY;
#endif /*CONFIG_VENDOR_EDIT*/

	if(start_id_polling) {
#if 0
	/* 
	if(id_polling_timeout > ID_POLLING_TIMEOUT_MAX)
		id_polling_timeout = ID_POLLING_TIMEOUT_MAX;
	os_printk(K_ERR, "user request polling start\n");
	cancel_delayed_work_sync(&usb_id->stop_polling_delay);
	schedule_delayed_work_on(0,&usb_id->stop_polling_delay,msecs_to_jiffies(id_polling_timeout));
	__pm_wakeup_event(&usb_id->wakeup_source_id_polling,
			(id_polling_timeout + ID_POLLING_WAKE_LOCK_TIMEOUT_EXT));
	*/                                                                           
#endif
		usb_id->user_request_polling = true;
		somc_chg_usbid_start_polling(usb_id);
	} else {
		os_printk(K_ERR, " user request polling stop !!\n");
		usb_id->user_request_polling = false;
		somc_chg_usbid_stop_polling(usb_id);
	}

	return ret;
}

ssize_t musb_id_forceon_show(struct device* dev, struct device_attribute *attr, char *buf)
{
	os_printk(K_ERR, "[MU3D]%s :%d \n", __func__,force_id_polling_on);
	if (!dev) {
		os_printk(K_ERR, "dev is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", force_id_polling_on);
}

ssize_t musb_id_forceon_store(struct device* dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
	int value;

	os_printk(K_ERR, "[MU3D]%s :%d \n", __func__,force_id_polling_on);
	if (!dev) {
		os_printk(K_ERR, "dev is null!!\n");
		return count;
	} else {
		sscanf(buf, "%d", &value);
		force_id_polling_on = value;
	}
	return count;
}

ssize_t musb_id_state_show(struct device* dev, struct device_attribute *attr, char *buf)
{
	os_printk(K_ERR, "[MU3D]%s :%d \n", __func__,id_polling_state);
	if (!dev) {
		os_printk(K_ERR, "dev is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", id_polling_state);
}

ssize_t musb_id_state_store(struct device* dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
	int value;

	os_printk(K_ERR, "[MU3D]%s :%d \n", __func__,id_polling_state);
	if (!dev) {
		os_printk(K_ERR, "dev is null!!\n");
		return count;
	} else {
		sscanf(buf, "%d", &value);
		id_polling_state = value;
	}
	return count;
}

ssize_t musb_id_timeout_show(struct device* dev, struct device_attribute *attr, char *buf)
{
	os_printk(K_ERR, "[MU3D]%s :%d \n", __func__,id_polling_timeout);
	if (!dev) {
		os_printk(K_ERR, "dev is null!!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", id_polling_timeout);
}

ssize_t musb_id_timeout_store(struct device* dev, struct device_attribute *attr,
										const char *buf, size_t count)
{
	int value;
	
	os_printk(K_ERR, "[MU3D]%s :%d \n", __func__,id_polling_timeout);
	if (!dev) {
		os_printk(K_ERR, "dev is null!!\n");
		return count;
	} else {
		sscanf(buf, "%d", &value);
		id_polling_timeout = value;
		os_printk(K_ERR, "[MU3D]%s :%d %d\n", __func__,id_polling_timeout,value);
	}
	return count;
}

ssize_t start_id_polling_show(struct device* dev, struct device_attribute *attr, char *buf)
{
	os_printk(K_ERR, "[MU3D]%s :%d \n", __func__,start_id_polling);
	if (!dev) {
	    os_printk(K_ERR, "dev is null!!\n");
	    return 0;
	}
	return sprintf(buf, "%d\n", start_id_polling);
}

ssize_t start_id_polling_store(struct device* dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
	int value;

	os_printk(K_ERR, "[MU3D]%s :%d \n", __func__,start_id_polling);
	if (!dev) {
		os_printk(K_ERR, "dev is null!!\n");
		return count;
	} else {
		sscanf(buf, "%d", &value);
		start_id_polling = value;
		os_printk(K_ERR, "[MU3D]%s :%d %d\n", __func__,start_id_polling,value);
		set_start_id_polling();
	}
	return count;
}
#endif /*VENDOR_EDIT*/

#ifdef NEVER
#ifdef CONFIG_MTK_FPGA
static struct i2c_client *usb_i2c_client;
static const struct i2c_device_id usb_i2c_id[] = { {"mtk-usb", 0}, {} };

static struct i2c_board_info usb_i2c_dev __initdata = { I2C_BOARD_INFO("mtk-usb", 0x60) };


void USB_PHY_Write_Register8(UINT8 var, UINT8 addr)
{
	char buffer[2];
	buffer[0] = addr;
	buffer[1] = var;
	i2c_master_send(usb_i2c_client, &buffer, 2);
}

UINT8 USB_PHY_Read_Register8(UINT8 addr)
{
	UINT8 var;
	i2c_master_send(usb_i2c_client, &addr, 1);
	i2c_master_recv(usb_i2c_client, &var, 1);
	return var;
}

static int usb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	pr_debug("[MUSB]usb_i2c_probe, start\n");

	usb_i2c_client = client;

	/* disable usb mac suspend */
	DRV_WriteReg8(USB_SIF_BASE + 0x86a, 0x00);

	/* usb phy initial sequence */
	USB_PHY_Write_Register8(0x00, 0xFF);
	USB_PHY_Write_Register8(0x04, 0x61);
	USB_PHY_Write_Register8(0x00, 0x68);
	USB_PHY_Write_Register8(0x00, 0x6a);
	USB_PHY_Write_Register8(0x6e, 0x00);
	USB_PHY_Write_Register8(0x0c, 0x1b);
	USB_PHY_Write_Register8(0x44, 0x08);
	USB_PHY_Write_Register8(0x55, 0x11);
	USB_PHY_Write_Register8(0x68, 0x1a);


	pr_debug("[MUSB]addr: 0xFF, value: %x\n", USB_PHY_Read_Register8(0xFF));
	pr_debug("[MUSB]addr: 0x61, value: %x\n", USB_PHY_Read_Register8(0x61));
	pr_debug("[MUSB]addr: 0x68, value: %x\n", USB_PHY_Read_Register8(0x68));
	pr_debug("[MUSB]addr: 0x6a, value: %x\n", USB_PHY_Read_Register8(0x6a));
	pr_debug("[MUSB]addr: 0x00, value: %x\n", USB_PHY_Read_Register8(0x00));
	pr_debug("[MUSB]addr: 0x1b, value: %x\n", USB_PHY_Read_Register8(0x1b));
	pr_debug("[MUSB]addr: 0x08, value: %x\n", USB_PHY_Read_Register8(0x08));
	pr_debug("[MUSB]addr: 0x11, value: %x\n", USB_PHY_Read_Register8(0x11));
	pr_debug("[MUSB]addr: 0x1a, value: %x\n", USB_PHY_Read_Register8(0x1a));


	pr_debug("[MUSB]usb_i2c_probe, end\n");
	return 0;

}

static int usb_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, "mtk-usb");
	return 0;
}

static int usb_i2c_remove(struct i2c_client *client)
{
	return 0;
}


struct i2c_driver usb_i2c_driver = {
	.probe = usb_i2c_probe,
	.remove = usb_i2c_remove,
	.detect = usb_i2c_detect,
	.driver = {
		   .name = "mtk-usb",
		   },
	.id_table = usb_i2c_id,
};

int add_usb_i2c_driver(void)
{
	i2c_register_board_info(0, &usb_i2c_dev, 1);
	if (i2c_add_driver(&usb_i2c_driver) != 0) {
		pr_debug("[MUSB]usb_i2c_driver initialization failed!!\n");
		return -1;
	} else {
		pr_debug("[MUSB]usb_i2c_driver initialization succeed!!\n");
	}
	return 0;
}
#endif				/* End of CONFIG_MTK_FPGA */
#endif				/* NEVER */
