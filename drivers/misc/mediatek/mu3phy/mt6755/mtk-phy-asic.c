#include <linux/mu3phy/mtk-phy.h>

#ifdef CONFIG_PROJECT_PHY
#include <mach/mt_pm_ldo.h>
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#else
#include <mach/mt_clkmgr.h>
#include <linux/clk.h>
#endif
#include <asm/io.h>
#include <linux/mu3phy/mtk-phy-asic.h>
#include <linux/mu3d/hal/mu3d_hal_osal.h>
#ifdef CONFIG_MTK_UART_USB_SWITCH
#include <linux/mu3d/hal/mu3d_hal_usb_drv.h>
#include <mach/mt_gpio_base.h>
#endif

#ifdef FOR_BRING_UP
#define enable_clock(x, y)
#define disable_clock(x, y)
#define hwPowerOn(x, y, z)
#define hwPowerDown(x, y)
#define set_ada_ssusb_xtal_ck(x)
#endif

#include <mach/upmu_common.h>

#ifdef CONFIG_OF
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,add 2015/6/15 for OTG and usb
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>

extern bool get_otg_switch();
int iddig_state = 1;//kong
#endif/*VENDOR_EDIT*/

bool sib_mode = false;

#ifdef USB_CLK_DEBUG
void __iomem *usb_debug_clk_infracfg_base;
#define MODULE_SW_CG_2_SET	(usb_debug_clk_infracfg_base + 0xa4)
#define MODULE_SW_CG_2_CLR	(usb_debug_clk_infracfg_base + 0xa8)
#define MODULE_SW_CG_2_STA	(usb_debug_clk_infracfg_base + 0xac)
static bool get_clk_io = true;
#endif
static bool usb_enable_clock(bool enable)
{
	if (enable) {
#ifdef CONFIG_MTK_CLKMGR
		enable_clock(MT_CG_PERI_USB0, "USB30");
#else
		clk_enable(musb_clk);
#endif
	} else {
#ifdef CONFIG_MTK_CLKMGR
		disable_clock(MT_CG_PERI_USB0, "USB30");
#else
		clk_disable(musb_clk);
#endif
	}

#ifdef USB_CLK_DEBUG
	if (get_clk_io) {
		struct device_node *node;

		get_clk_io = false;
		node = of_find_compatible_node(NULL, NULL, "mediatek,mt6755-infrasys");
		usb_debug_clk_infracfg_base = of_iomap(node, 0);
		if (!usb_debug_clk_infracfg_base)
			pr_err("[CLK_INFRACFG_AO] base failed\n");
	}
	if (!IS_ERR(musb_clk))
		pr_err("SSUSB musb clock is okay, enabel: %d\n", enable);
	else
		pr_err("SSUSB musb clock is fail, enabel: %d\n", enable);
	/*bit1: ssusb_top_cg_sta  (0: clock enable  1: clock disable)*/
	pr_err("SSUSB MODULE_SW_CG_2_STA  = 0x%08x\n", DRV_Reg32(MODULE_SW_CG_2_STA));
#endif
	return 1;
}


#ifdef NEVER
/*Turn on/off ADA_SSUSB_XTAL_CK 26MHz*/
void enable_ssusb_xtal_clock(bool enable)
{
	if (enable) {
		/*
		 * 1 *AP_PLL_CON0 =| 0x1 [0]=1: RG_LTECLKSQ_EN
		 * 2 Wait PLL stable (100us)
		 * 3 *AP_PLL_CON0 =| 0x2 [1]=1: RG_LTECLKSQ_LPF_EN
		 * 4 *AP_PLL_CON2 =| 0x1 [0]=1: DA_REF2USB_TX_EN
		 * 5 Wait PLL stable (100us)
		 * 6 *AP_PLL_CON2 =| 0x2 [1]=1: DA_REF2USB_TX_LPF_EN
		 * 7 *AP_PLL_CON2 =| 0x4 [2]=1: DA_REF2USB_TX_OUT_EN
		 */
		writel(readl((void __iomem *)AP_PLL_CON0) | (0x00000001),
		       (void __iomem *)AP_PLL_CON0);
		/*Wait 100 usec */
		udelay(100);

		writel(readl((void __iomem *)AP_PLL_CON0) | (0x00000002),
		       (void __iomem *)AP_PLL_CON0);

		writel(readl((void __iomem *)AP_PLL_CON2) | (0x00000001),
		       (void __iomem *)AP_PLL_CON2);

		/*Wait 100 usec */
		udelay(100);

		writel(readl((void __iomem *)AP_PLL_CON2) | (0x00000002),
		       (void __iomem *)AP_PLL_CON2);

		writel(readl((void __iomem *)AP_PLL_CON2) | (0x00000004),
		       (void __iomem *)AP_PLL_CON2);
	} else {
		/*
		 * AP_PLL_CON2 &= 0xFFFFFFF8        [2]=0: DA_REF2USB_TX_OUT_EN
		 *                                  [1]=0: DA_REF2USB_TX_LPF_EN
		 *                                  [0]=0: DA_REF2USB_TX_EN
		 */
		/* writel(readl((void __iomem *)AP_PLL_CON2)&~(0x00000007), */
		/* (void __iomem *)AP_PLL_CON2); */
	}
}

/*Turn on/off AD_LTEPLL_SSUSB26M_CK 26MHz*/
void enable_ssusb26m_ck(bool enable)
{
	if (enable) {
		/*
		 * 1 *AP_PLL_CON0 =| 0x1 [0]=1: RG_LTECLKSQ_EN
		 * 2 Wait PLL stable (100us)
		 * 3 *AP_PLL_CON0 =| 0x2 [1]=1: RG_LTECLKSQ_LPF_EN
		 */
		writel(readl((void __iomem *)AP_PLL_CON0) | (0x00000001),
		       (void __iomem *)AP_PLL_CON0);
		/*Wait 100 usec */
		udelay(100);

		writel(readl((void __iomem *)AP_PLL_CON0) | (0x00000002),
		       (void __iomem *)AP_PLL_CON0);

	} else {
		/*
		 * AP_PLL_CON2 &= 0xFFFFFFF8        [2]=0: DA_REF2USB_TX_OUT_EN
		 *                                  [1]=0: DA_REF2USB_TX_LPF_EN
		 *                                  [0]=0: DA_REF2USB_TX_EN
		 */
		/* writel(readl((void __iomem *)AP_PLL_CON2)&~(0x00000007), */
		/* (void __iomem *)AP_PLL_CON2); */
	}
}
#endif				/* NEVER */

void usb20_pll_settings(bool host, bool forceOn)
{
	if (host) {
		if (forceOn) {
			os_printk(K_INFO, "%s-%d - Set USBPLL_FORCE_ON.\n", __func__, __LINE__);
			/* Set RG_SUSPENDM to 1 */
			U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_SUSPENDM_OFST, E60802_RG_SUSPENDM, 1);
			/* force suspendm = 1 */
			U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 1);
#ifndef CONFIG_MTK_TYPEC_SWITCH
			U3PhyWriteField32(U3D_PHYA_REG6, E60802_RG_SSUSB_RESERVE22_OFST,
				E60802_RG_SSUSB_RESERVE22, 0x1);
#endif

		} else {
			os_printk(K_INFO, "%s-%d - Clear USBPLL_FORCE_ON.\n", __func__, __LINE__);
			/* Set RG_SUSPENDM to 1 */
			U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_SUSPENDM_OFST, E60802_RG_SUSPENDM, 0);
			/* force suspendm = 1 */
			U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 0);
#ifndef CONFIG_MTK_TYPEC_SWITCH
				U3PhyWriteField32(U3D_PHYA_REG6, E60802_RG_SSUSB_RESERVE22_OFST,
					E60802_RG_SSUSB_RESERVE22, 0x0);
#endif
			return;
		}
	}

	os_printk(K_INFO, "%s-%d - Set PLL_FORCE_MODE and SIFSLV PLL_FORCE_ON.\n", __func__,
		  __LINE__);
	U3PhyWriteField32(U3D_USBPHYACR2_0, E60802_RG_SIFSLV_USB20_PLL_FORCE_MODE_OFST,
			  E60802_RG_SIFSLV_USB20_PLL_FORCE_MODE, 0x1);
	U3PhyWriteField32(U3D_U2PHYDCR0, E60802_RG_SIFSLV_USB20_PLL_FORCE_ON_OFST,
			  E60802_RG_SIFSLV_USB20_PLL_FORCE_ON, 0x0);
}

#ifdef CONFIG_MTK_UART_USB_SWITCH
bool in_uart_mode = false;
void uart_usb_switch_dump_register(void)
{
	/* ADA_SSUSB_XTAL_CK:26MHz */
	set_ada_ssusb_xtal_ck(1);
	/* f_fusb30_ck:125MHz */
	usb_enable_clock(true);
	udelay(50);

#ifdef CONFIG_MTK_FPGA
	pr_debug("[MUSB]addr: 0x6B, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYDTM0 + 0x3));
	pr_debug("[MUSB]addr: 0x6E, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYDTM1 + 0x2));
	pr_debug("[MUSB]addr: 0x22, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYACR4 + 0x2));
	pr_debug("[MUSB]addr: 0x68, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYDTM0));
	pr_debug("[MUSB]addr: 0x6A, value: %x\n", USB_PHY_Read_Register8(U3D_U2PHYDTM0 + 0x2));
	pr_debug("[MUSB]addr: 0x1A, value: %x\n", USB_PHY_Read_Register8(U3D_USBPHYACR6 + 0x2));
#else
#if 0
	os_printk(K_INFO, "[MUSB]addr: 0x6B, value: %x\n", U3PhyReadReg8(U3D_U2PHYDTM0 + 0x3));
	os_printk(K_INFO, "[MUSB]addr: 0x6E, value: %x\n", U3PhyReadReg8(U3D_U2PHYDTM1 + 0x2));
	os_printk(K_INFO, "[MUSB]addr: 0x22, value: %x\n", U3PhyReadReg8(U3D_U2PHYACR4 + 0x2));
	os_printk(K_INFO, "[MUSB]addr: 0x68, value: %x\n", U3PhyReadReg8(U3D_U2PHYDTM0));
	os_printk(K_INFO, "[MUSB]addr: 0x6A, value: %x\n", U3PhyReadReg8(U3D_U2PHYDTM0 + 0x2));
	os_printk(K_INFO, "[MUSB]addr: 0x1A, value: %x\n", U3PhyReadReg8(U3D_USBPHYACR6 + 0x2));
#else
	os_printk(K_INFO, "[MUSB]addr: 0x18, value: 0x%x\n", U3PhyReadReg32(U3D_USBPHYACR6));
	os_printk(K_INFO, "[MUSB]addr: 0x20, value: 0x%x\n", U3PhyReadReg32(U3D_U2PHYACR4));
	os_printk(K_INFO, "[MUSB]addr: 0x68, value: 0x%x\n", U3PhyReadReg32(U3D_U2PHYDTM0));
	os_printk(K_INFO, "[MUSB]addr: 0x6C, value: 0x%x\n", U3PhyReadReg32(U3D_U2PHYDTM1));
#endif
#endif

	/* ADA_SSUSB_XTAL_CK:26MHz */
	set_ada_ssusb_xtal_ck(0);
	/* f_fusb30_ck:125MHz */
	usb_enable_clock(false);

	os_printk(K_INFO, "[MUSB]addr: 0x110020B0 (UART0), value: %x\n\n",
		  DRV_Reg8(ap_uart0_base + 0xB0));
}

bool usb_phy_check_in_uart_mode(void)
{
	PHY_INT32 usb_port_mode;

	/* ADA_SSUSB_XTAL_CK:26MHz */
	set_ada_ssusb_xtal_ck(1);
	/* f_fusb30_ck:125MHz */
	usb_enable_clock(true);

	udelay(50);
	usb_port_mode = U3PhyReadReg32(U3D_U2PHYDTM0) >> E60802_RG_UART_MODE_OFST;

	/* ADA_SSUSB_XTAL_CK:26MHz */
	set_ada_ssusb_xtal_ck(0);
	/* f_fusb30_ck:125MHz */
	usb_enable_clock(false);

	os_printk(K_INFO, "%s+ usb_port_mode = %d\n", __func__, usb_port_mode);

	if (usb_port_mode == 0x1)
		return true;
	else
		return false;
}

void usb_phy_switch_to_uart(void)
{
	PHY_INT32 ret;
	PHY_INT32 val;

	if (usb_phy_check_in_uart_mode()) {
		os_printk(K_INFO, "%s+ UART_MODE\n", __func__);
		return;
	} else {
		os_printk(K_INFO, "%s+ USB_MODE\n", __func__);
	}

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	/*---POWER-----*/
	/*AVDD18_USB_P0 is always turned on. The driver does _NOT_ need to control it. */
	/*hwPowerOn(MT6332_POWER_LDO_VUSB33, VOL_3300, "VDD33_USB_P0");*/
	ret = pmic_set_register_value(PMIC_LDO_VUSB33_EN, 0x01);
	if (ret)
		pr_debug("VUSB33 enable FAIL!!!\n");
#else
	/*---POWER-----*/
	/*AVDD18_USB_P0 is always turned on. The driver does _NOT_ need to control it. */
	/*hwPowerOn(MT6332_POWER_LDO_VUSB33, VOL_3300, "VDD33_USB_P0");*/
	ret = pmic_set_register_value(MT6351_PMIC_RG_VUSB33_EN, 0x01);
	if (ret)
		pr_debug("VUSB33 enable FAIL!!!\n");

	/* Set RG_VUSB10_ON as 1 after VDD10 Ready */
	/*hwPowerOn(MT6331_POWER_LDO_VUSB10, VOL_1000, "VDD10_USB_P0");*/
	ret = pmic_set_register_value(MT6351_PMIC_RG_VA10_EN, 0x01);
	if (ret)
		pr_debug("VA10 enable FAIL!!!\n");

	ret = pmic_set_register_value(MT6351_PMIC_RG_VA10_VOSEL, 0x02);
	if (ret)
		pr_debug("VA10 output selection to 1.0v FAIL!!!\n");
#endif
	/* ADA_SSUSB_XTAL_CK:26MHz */
	set_ada_ssusb_xtal_ck(1);

	/* f_fusb30_ck:125MHz */
	usb_enable_clock(true);
	udelay(50);

	/* RG_USB20_BC11_SW_EN = 1'b0 */
	U3PhyWriteField32(U3D_USBPHYACR2, E60802_RG_SIFSLV_MAC_BANDGAP_EN_OFST,
			  E60802_RG_SIFSLV_MAC_BANDGAP_EN, 0);

	/* Set RG_SUSPENDM to 1 */
	U3PhyWriteField32(U3D_USBPHYACR0, E60802_RG_SIFSLV_BGR_EN_OFST, E60802_RG_SIFSLV_BGR_EN, 1);

	/* force suspendm = 1 */
		U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_UART_MODE_OFST, E60802_RG_UART_MODE, 1);

	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_BIAS_EN_OFST, E60802_FORCE_UART_BIAS_EN, 1);

	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 1);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 1);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_BIAS_EN_OFST, E60802_RG_UART_BIAS_EN, 1);

	/* Set RG_USB20_DM_100K_EN to 1 */
	U3PhyWriteField32(U3D_U2PHYACR4_0, E60802_RG_USB20_DM_100K_EN_OFST, E60802_RG_USB20_DM_100K_EN, 1);

	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST, E60802_RG_USB20_BC11_SW_EN, 0);

	/* ADA_SSUSB_XTAL_CK:26MHz */
	set_ada_ssusb_xtal_ck(0);
	/* f_fusb30_ck:125MHz */
	usb_enable_clock(false);

	/* GPIO Selection */
		val = USB_RD32(0x6E0);
		val = val | 0x00080000;
    USB_WR32(0x6E0, val);

	in_uart_mode = true;
}


void usb_phy_switch_to_usb(void)
{
	in_uart_mode = false;
PHY_INT32 val;
	/* GPIO Selection */
		val = IOCFG_RD32(0x6E0);
		val = val & 0xFFF7FFFF;
		IOCFG_WR32(0x6E0, val);	/* set */

	/* clear force_uart_en */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	#if 1
	#ifdef CONFIG_MTK_UART_USB_SWITCH
	U3PhyWriteField32(U3D_USBPHYACR2, E60802_RG_SIFSLV_MAC_BANDGAP_EN_OFST,
			  E60802_RG_SIFSLV_MAC_BANDGAP_EN, 1);

	/*  */
	U3PhyWriteField32(U3D_USBPHYACR0, E60802_RG_SIFSLV_BGR_EN_OFST, E60802_RG_SIFSLV_BGR_EN, 0);

	#endif
	#endif



	phy_init_soc(u3phy);

	/* disable the USB clock turned on in phy_init_soc() */
	/* ADA_SSUSB_XTAL_CK:26MHz */
	set_ada_ssusb_xtal_ck(0);
	/* f_fusb30_ck:125MHz */
	usb_enable_clock(false);

}
#endif
#define RG_SSUSB_VUSB10_ON (1<<5)
#define RG_SSUSB_VUSB10_ON_OFST (5)

#ifdef CONFIG_MTK_SIB_USB_SWITCH
void usb_phy_sib_enable_switch(bool enable)
{
	/*
	 * It's MD debug usage. No need to care low power.
	 * Thus, no power off BULK and Clock at the end of function.
	 * MD SIB still needs these power and clock source.
	 */
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_set_register_value(PMIC_LDO_VUSB33_EN, 0x01);
#else	 
	pmic_set_register_value(MT6351_PMIC_RG_VUSB33_EN, 0x01);
	pmic_set_register_value(MT6351_PMIC_RG_VA10_EN, 0x01);
	pmic_set_register_value(MT6351_PMIC_RG_VA10_VOSEL, 0x02);
#endif

	usb_enable_clock(true);
	udelay(50);
#if !defined(CONFIG_USB_MU3D_ONLY_U2_MODE)	
	U3PhyWriteField32(U3D_USB30_PHYA_REG0, RG_SSUSB_VUSB10_ON_OFST, RG_SSUSB_VUSB10_ON, 1);
#endif
	writel(0x00031000, (u3_sif_base + 0x700));  /* SSUSB_IP_SW_RST = 0    */
	writel(0x00000000, (u3_sif_base + 0x704));  /* SSUSB_IP_HOST_PDN = 0  */
	writel(0x00000000, (u3_sif_base + 0x708));  /* SSUSB_IP_DEV_PDN = 0   */
	writel(0x00000000, (u3_sif_base + 0x70C));  /* SSUSB_IP_PCIE_PDN = 0  */
	writel(0x0000000C, (u3_sif_base + 0x730));  /* SSUSB_U3_PORT_DIS/SSUSB_U3_PORT_PDN = 0*/

	/*
	 * USBMAC mode is 0x62910002 (bit 1)
	 * MDSIB  mode is 0x62910008 (bit 3)
	 * 0x0629 just likes a signature. Can't be removed.
	 */
	if (enable) {
		U3PhyWriteReg32((u3_sif2_base+0x300), 0x62910008);
		sib_mode = true;
	} else {
		U3PhyWriteReg32((u3_sif2_base+0x300), 0x62910002);
		sib_mode = false;
	}
}

bool usb_phy_sib_enable_switch_status(void)
{
	int reg;
	bool ret;
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	pmic_set_register_value(PMIC_LDO_VUSB33_EN, 0x01);
#else	 
	pmic_set_register_value(MT6351_PMIC_RG_VUSB33_EN, 0x01);
	pmic_set_register_value(MT6351_PMIC_RG_VA10_EN, 0x01);
	pmic_set_register_value(MT6351_PMIC_RG_VA10_VOSEL, 0x02);
#endif

	usb_enable_clock(true);
	udelay(50);
#if !defined(CONFIG_USB_MU3D_ONLY_U2_MODE)		
	U3PhyWriteField32(U3D_USB30_PHYA_REG0, RG_SSUSB_VUSB10_ON_OFST, RG_SSUSB_VUSB10_ON, 1);
#endif

	reg = U3PhyReadReg32(u3_sif2_base+0x300);
	if (reg == 0x62910008)
		ret = true;
	else
		ret = false;

	return ret;
}
#endif


/*This "power on/initial" sequence refer to "6593_USB_PORT0_PWR Sequence 20130729.xls"*/
PHY_INT32 phy_init_soc(struct u3phy_info *info)
{
	PHY_INT32 ret;
	os_printk(K_INFO, "%s+\n", __func__);
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,add 2015/6/15 for OTG and usb
	iddig_state = mt_get_gpio_in(GPIO_OTG_IDDIG_EINT_PIN);//kong
#endif/*VENDOR_EDIT*/

	/*This power on sequence refers to Sheet .1 of "6593_USB_PORT0_PWR Sequence 20130729.xls" */
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	/*---POWER-----*/
	/*AVDD18_USB_P0 is always turned on. The driver does _NOT_ need to control it. */
	ret = pmic_set_register_value(PMIC_LDO_VUSB33_EN, 0x01);
	if (ret)
		pr_debug("VUSB33 enable FAIL!!!\n");
#else
	/*---POWER-----*/
	/*AVDD18_USB_P0 is always turned on. The driver does _NOT_ need to control it. */
	/*hwPowerOn(MT6332_POWER_LDO_VUSB33, VOL_3300, "VDD33_USB_P0");*/
	ret = pmic_set_register_value(MT6351_PMIC_RG_VUSB33_EN, 0x01);
	if (ret)
		pr_debug("VUSB33 enable FAIL!!!\n");

	/* Set RG_VUSB10_ON as 1 after VDD10 Ready */
	/*hwPowerOn(MT6331_POWER_LDO_VUSB10, VOL_1000, "VDD10_USB_P0");*/
	ret = pmic_set_register_value(MT6351_PMIC_RG_VA10_EN, 0x01);
	if (ret)
			pr_debug("VA10 enable FAIL!!!\n");

	ret = pmic_set_register_value(MT6351_PMIC_RG_VA10_VOSEL, 0x02);
	if (ret)
		pr_debug("VA10 output selection to 1.0v FAIL!!!\n");
#endif
	/*---CLOCK-----*/
	/* ADA_SSUSB_XTAL_CK:26MHz */
	set_ada_ssusb_xtal_ck(1);

	/* AD_LTEPLL_SSUSB26M_CK:26MHz always on */
	/* It seems that when turning on ADA_SSUSB_XTAL_CK, AD_LTEPLL_SSUSB26M_CK will also turn on. */
	/* enable_ssusb26m_ck(true); */

	/* f_fusb30_ck:125MHz */
	usb_enable_clock(true);

	/* AD_SSUSB_48M_CK:48MHz */
	/* It seems that when turning on f_fusb30_ck, AD_SSUSB_48M_CK will also turn on. */

	/*Wait 50 usec */
	udelay(50);
#if !defined(CONFIG_USB_MU3D_ONLY_U2_MODE)
	/* Set RG_SSUSB_VUSB10_ON as 1 after VUSB10 ready */
	U3PhyWriteField32(U3D_USB30_PHYA_REG0, RG_SSUSB_VUSB10_ON_OFST, RG_SSUSB_VUSB10_ON, 1);
#endif
	/*power domain iso disable */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_ISO_EN_OFST, E60802_RG_USB20_ISO_EN, 0);

#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (!in_uart_mode) {
		/*switch to USB function. (system register, force ip into usb mode) */
		U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN,
				  0);
		U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);
		U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST,
				  E60802_RG_USB20_GPIO_CTL, 0);
		U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST,
				  E60802_USB20_GPIO_MODE, 0);
		/* Set ru_uart_mode to 2'b00 */
		U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_UART_MODE_OFST, E60802_RG_UART_MODE, 0);
		/*dm_100k disable */
		U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_DM_100K_EN_OFST,
				  E60802_RG_USB20_DM_100K_EN, 0);
		/*Release force suspendm.  (force_suspendm=0) (let suspendm=1, enable usb 480MHz pll) */
		U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM,
				  0);
	}
	/*DP/DM BC1.1 path Disable */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST,
			  E60802_RG_USB20_BC11_SW_EN, 0);
	/*dp_100k disable */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_DP_100K_MODE_OFST,
			  E60802_RG_USB20_DP_100K_MODE, 1);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_DP_100K_EN_OFST, E60802_USB20_DP_100K_EN, 0);
#if defined(CONFIG_MTK_HDMI_SUPPORT) || defined(MTK_USB_MODE1)
	os_printk(K_INFO, "%s- USB PHY Driving Tuning Mode 1 Settings.\n", __func__);
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST,
			  E60802_RG_USB20_HS_100U_U3_EN, 0);
	U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_VRT_VREF_SEL_OFST,
			  E60802_RG_USB20_VRT_VREF_SEL, 5);
	U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_TERM_VREF_SEL_OFST,
			  E60802_RG_USB20_TERM_VREF_SEL, 5);
#else
#if !defined(CONFIG_USB_MU3D_ONLY_U2_MODE)
	/*Change 100uA current switch to SSUSB */
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST,
			  E60802_RG_USB20_HS_100U_U3_EN, 1);
#endif
#endif
	/*OTG Enable */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_OTG_VBUSCMP_EN_OFST,
			  E60802_RG_USB20_OTG_VBUSCMP_EN, 1);
	/*Pass RX sensitivity HQA requirement */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH, 0x2);
#else
	/*switch to USB function. (system register, force ip into usb mode) */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL,
			  0);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);
	/*DP/DM BC1.1 path Disable */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST,
			  E60802_RG_USB20_BC11_SW_EN, 0);
	/*dp_100k disable */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_DP_100K_MODE_OFST,
			  E60802_RG_USB20_DP_100K_MODE, 1);
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_DP_100K_EN_OFST, E60802_USB20_DP_100K_EN, 0);
	/*dm_100k disable */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_DM_100K_EN_OFST,
			  E60802_RG_USB20_DM_100K_EN, 0);
#if !defined(CONFIG_MTK_HDMI_SUPPORT) && !defined(MTK_USB_MODE1)
#if !defined(CONFIG_USB_MU3D_ONLY_U2_MODE)
	/*Change 100uA current switch to SSUSB */
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST,
			  E60802_RG_USB20_HS_100U_U3_EN, 1);
#endif
#endif
	/*OTG Enable */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_OTG_VBUSCMP_EN_OFST,
			  E60802_RG_USB20_OTG_VBUSCMP_EN, 1);
	/*Pass RX sensitivity HQA requirement */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH, 0x2);
	/*Release force suspendm.  (force_suspendm=0) (let suspendm=1, enable usb 480MHz pll) */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 0);
#endif

	#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,add 2015/5/4 for eyesight with vref internal R
	if((iddig_state == 0) &&(get_otg_switch() == 1 ))
	//if(iddig_state == 0)
	{
		U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_TERM_VREF_SEL_OFST, E60802_RG_USB20_TERM_VREF_SEL, 1);
		U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_VRT_VREF_SEL_OFST, E60802_RG_USB20_VRT_VREF_SEL, 7);
		printk("phy_init_soc, iddig_state = %d,get_otg_switch() = %d\r\n", iddig_state,get_otg_switch());
	}
	else
	{
		U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_TERM_VREF_SEL_OFST, E60802_RG_USB20_TERM_VREF_SEL, 5);
		U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_VRT_VREF_SEL_OFST, E60802_RG_USB20_VRT_VREF_SEL, 7);
		printk("phy_init_soc, iddig_state = %d,get_otg_switch() = %d\r\n", iddig_state,get_otg_switch());
	}

	#endif/*VENDOR_EDIT*/

	/*Wait 800 usec */
	udelay(800);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_FORCE_VBUSVALID_OFST, E60802_FORCE_VBUSVALID, 1);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_FORCE_AVALID_OFST, E60802_FORCE_AVALID, 1);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_FORCE_SESSEND_OFST, E60802_FORCE_SESSEND, 1);

	/* USB PLL Force settings */
	usb20_pll_settings(false, false);

	os_printk(K_INFO, "%s-\n", __func__);

	return PHY_TRUE;
}

PHY_INT32 u2_slew_rate_calibration(struct u3phy_info *info)
{
	PHY_INT32 i = 0;
	PHY_INT32 fgRet = 0;
	PHY_INT32 u4FmOut = 0;
	PHY_INT32 u4Tmp = 0;

	os_printk(K_INFO, "%s\n", __func__);

	/* => RG_USB20_HSTX_SRCAL_EN = 1 */
	/* enable USB ring oscillator */
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HSTX_SRCAL_EN_OFST,
			  E60802_RG_USB20_HSTX_SRCAL_EN, 1);

	/* wait 1us */
	udelay(1);

	/* => USBPHY base address + 0x110 = 1 */
	/* Enable free run clock */
	U3PhyWriteField32((u3_sif2_base + 0x110)
			  , E60802_RG_FRCK_EN_OFST, E60802_RG_FRCK_EN, 0x1);

	/* => USBPHY base address + 0x100 = 0x04 */
	/* Setting cyclecnt */
	U3PhyWriteField32((u3_sif2_base + 0x100)
			  , E60802_RG_CYCLECNT_OFST, E60802_RG_CYCLECNT, 0x400);

	/* => USBPHY base address + 0x100 = 0x01 */
	/* Enable frequency meter */
	U3PhyWriteField32((u3_sif2_base + 0x100)
			  , E60802_RG_FREQDET_EN_OFST, E60802_RG_FREQDET_EN, 0x1);

	os_printk(K_INFO, "Freq_Valid=(0x%08X)\n", U3PhyReadReg32(u3_sif2_base + 0x110));

	mdelay(1);

	/* wait for FM detection done, set 10ms timeout */
	for (i = 0; i < 10; i++) {
		/* => USBPHY base address + 0x10C = FM_OUT */
		/* Read result */
		u4FmOut = U3PhyReadReg32(u3_sif2_base + 0x10C);
		os_printk(K_INFO, "FM_OUT value: u4FmOut = %d(0x%08X)\n", u4FmOut, u4FmOut);

		/* check if FM detection done */
		if (u4FmOut != 0) {
			fgRet = 0;
			os_printk(K_INFO, "FM detection done! loop = %d\n", i);
			break;
		}
		fgRet = 1;
		mdelay(1);
	}
	/* => USBPHY base address + 0x100 = 0x00 */
	/* Disable Frequency meter */
	U3PhyWriteField32((u3_sif2_base + 0x100)
			  , E60802_RG_FREQDET_EN_OFST, E60802_RG_FREQDET_EN, 0);

	/* => USBPHY base address + 0x110 = 0x00 */
	/* Disable free run clock */
	U3PhyWriteField32((u3_sif2_base + 0x110)
			  , E60802_RG_FRCK_EN_OFST, E60802_RG_FRCK_EN, 0);

	/* RG_USB20_HSTX_SRCTRL[2:0] = (1024/FM_OUT) * reference clock frequency * 0.028 */
	if (u4FmOut == 0) {
		U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HSTX_SRCTRL_OFST,
				  E60802_RG_USB20_HSTX_SRCTRL, 0x4);
		fgRet = 1;
	} else {
		/* set reg = (1024/FM_OUT) * REF_CK * U2_SR_COEF_E60802 / 1000 (round to the nearest digits) */
		/* u4Tmp = (((1024 * REF_CK * U2_SR_COEF_E60802) / u4FmOut) + 500) / 1000; */
		u4Tmp = (1024 * REF_CK * U2_SR_COEF_E60802) / (u4FmOut * 1000);
		os_printk(K_INFO, "SR calibration value u1SrCalVal = %d\n", (PHY_UINT8) u4Tmp);
		U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HSTX_SRCTRL_OFST,
				  E60802_RG_USB20_HSTX_SRCTRL, u4Tmp);
	}

	/* => RG_USB20_HSTX_SRCAL_EN = 0 */
	/* disable USB ring oscillator */
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HSTX_SRCAL_EN_OFST,
			  E60802_RG_USB20_HSTX_SRCAL_EN, 0);

	return fgRet;
}

/*This "save current" sequence refers to "6593_USB_PORT0_PWR Sequence 20130729.xls"*/
void usb_phy_savecurrent(unsigned int clk_on)
{
	PHY_INT32 ret;
	os_printk(K_INFO, "%s clk_on=%d+\n", __func__, clk_on);

	if (sib_mode) {
		pr_err("%s sib_mode can't savecurrent\n", __func__);
		return;
	}

#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (!in_uart_mode) {
		/*switch to USB function. (system register, force ip into usb mode) */
		/* force_uart_en      1'b0 */
		/* U3D_U2PHYDTM0 E60802_FORCE_UART_EN */
		U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN,
				  0);
		/* RG_UART_EN         1'b0 */
		/* U3D_U2PHYDTM1 E60802_RG_UART_EN */
		U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);

		/*let suspendm=1, enable usb 480MHz pll */
		/* RG_SUSPENDM 1'b1 */
		/* U3D_U2PHYDTM0 E60802_RG_SUSPENDM */
		U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_SUSPENDM_OFST, E60802_RG_SUSPENDM, 1);

		/*force_suspendm=1 */
		/* force_suspendm        1'b1 */
		/* U3D_U2PHYDTM0 E60802_FORCE_SUSPENDM */
		U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM,
				  1);
	}
	/* rg_usb20_gpio_ctl  1'b0 */
	/* U3D_U2PHYACR4 E60802_RG_USB20_GPIO_CTL */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL,
			  0);
	/* usb20_gpio_mode       1'b0 */
	/* U3D_U2PHYACR4 E60802_USB20_GPIO_MODE */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);
#else
	/*switch to USB function. (system register, force ip into usb mode) */
	/* force_uart_en      1'b0 */
	/* U3D_U2PHYDTM0 E60802_FORCE_UART_EN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	/* RG_UART_EN         1'b0 */
	/* U3D_U2PHYDTM1 E60802_RG_UART_EN */
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);
	/* rg_usb20_gpio_ctl  1'b0 */
	/* U3D_U2PHYACR4 E60802_RG_USB20_GPIO_CTL */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL,
			  0);
	/* usb20_gpio_mode       1'b0 */
	/* U3D_U2PHYACR4 E60802_USB20_GPIO_MODE */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);

	/*let suspendm=1, enable usb 480MHz pll */
	/* RG_SUSPENDM 1'b1 */
	/* U3D_U2PHYDTM0 E60802_RG_SUSPENDM */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_SUSPENDM_OFST, E60802_RG_SUSPENDM, 1);

	/*force_suspendm=1 */
	/* force_suspendm        1'b1 */
	/* U3D_U2PHYDTM0 E60802_FORCE_SUSPENDM */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 1);
#endif
	/*Wait USBPLL stable. */
	/* Wait 2 ms. */
	udelay(2000);

	/* RG_DPPULLDOWN 1'b1 */
	/* U3D_U2PHYDTM0 E60802_RG_DPPULLDOWN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DPPULLDOWN_OFST, E60802_RG_DPPULLDOWN, 1);

	/* RG_DMPULLDOWN 1'b1 */
	/* U3D_U2PHYDTM0 E60802_RG_DMPULLDOWN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DMPULLDOWN_OFST, E60802_RG_DMPULLDOWN, 1);

	/* RG_XCVRSEL[1:0] 2'b01 */
	/* U3D_U2PHYDTM0 E60802_RG_XCVRSEL */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_XCVRSEL_OFST, E60802_RG_XCVRSEL, 0x1);

	/* RG_TERMSEL    1'b1 */
	/* U3D_U2PHYDTM0 E60802_RG_TERMSEL */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_TERMSEL_OFST, E60802_RG_TERMSEL, 1);

	/* RG_DATAIN[3:0]        4'b0000 */
	/* U3D_U2PHYDTM0 E60802_RG_DATAIN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DATAIN_OFST, E60802_RG_DATAIN, 0);

	/* force_dp_pulldown     1'b1 */
	/* U3D_U2PHYDTM0 E60802_FORCE_DP_PULLDOWN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DP_PULLDOWN_OFST, E60802_FORCE_DP_PULLDOWN,
			  1);

	/* force_dm_pulldown     1'b1 */
	/* U3D_U2PHYDTM0 E60802_FORCE_DM_PULLDOWN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DM_PULLDOWN_OFST, E60802_FORCE_DM_PULLDOWN,
			  1);

	/* force_xcversel        1'b1 */
	/* U3D_U2PHYDTM0 E60802_FORCE_XCVRSEL */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_XCVRSEL_OFST, E60802_FORCE_XCVRSEL, 1);

	/* force_termsel 1'b1 */
	/* U3D_U2PHYDTM0 E60802_FORCE_TERMSEL */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_TERMSEL_OFST, E60802_FORCE_TERMSEL, 1);

	/* force_datain  1'b1 */
	/* U3D_U2PHYDTM0 E60802_FORCE_DATAIN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DATAIN_OFST, E60802_FORCE_DATAIN, 1);

	/*DP/DM BC1.1 path Disable */
	/* RG_USB20_BC11_SW_EN 1'b0 */
	/* U3D_USBPHYACR6 E60802_RG_USB20_BC11_SW_EN */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST,
			  E60802_RG_USB20_BC11_SW_EN, 1); /* switch to 1 for power saving */

	/*OTG Disable */
	/* RG_USB20_OTG_VBUSCMP_EN 1b0 */
	/* U3D_USBPHYACR6 E60802_RG_USB20_OTG_VBUSCMP_EN */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_OTG_VBUSCMP_EN_OFST,
			  E60802_RG_USB20_OTG_VBUSCMP_EN, 0);

	/*Change 100uA current switch to USB2.0 */
	/* RG_USB20_HS_100U_U3_EN        1'b0 */
	/* U3D_USBPHYACR5 E60802_RG_USB20_HS_100U_U3_EN */
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST,
			  E60802_RG_USB20_HS_100U_U3_EN, 0);

	/* wait 800us */
	udelay(800);

	/*let suspendm=0, set utmi into analog power down */
	/* RG_SUSPENDM 1'b0 */
	/* U3D_U2PHYDTM0 E60802_RG_SUSPENDM */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_SUSPENDM_OFST, E60802_RG_SUSPENDM, 0);

	/* wait 1us */
	udelay(1);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_VBUSVALID_OFST, E60802_RG_VBUSVALID, 0);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_AVALID_OFST, E60802_RG_AVALID, 0);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_SESSEND_OFST, E60802_RG_SESSEND, 1);

	/* USB PLL Force settings */
	usb20_pll_settings(false, false);

	/* TODO:
	 * Turn off internal 48Mhz PLL if there is no other hardware module is
	 * using the 48Mhz clock -the control register is in clock document
	 * Turn off SSUSB reference clock (26MHz)
	 */
	if (clk_on) {
#if !defined(CONFIG_USB_MU3D_ONLY_U2_MODE)		
		/*---CLOCK-----*/
		/* Set RG_SSUSB_VUSB10_ON as 1 after VUSB10 ready */
		U3PhyWriteField32(U3D_USB30_PHYA_REG0, RG_SSUSB_VUSB10_ON_OFST, RG_SSUSB_VUSB10_ON,
				  0);
#endif
		/* Wait 10 usec. */
		udelay(10);

		/* f_fusb30_ck:125MHz */
		usb_enable_clock(false);

		/* ADA_SSUSB_XTAL_CK:26MHz */
		set_ada_ssusb_xtal_ck(0);
#if !defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		/*---POWER-----*/
		/* Set RG_VUSB10_ON as 1 after VDD10 Ready */
		/*hwPowerDown(MT6331_POWER_LDO_VUSB10, "VDD10_USB_P0");*/
		ret = pmic_set_register_value(MT6351_PMIC_RG_VA10_EN, 0x00);
#endif
	}

	os_printk(K_INFO, "%s-\n", __func__);
}

/*This "recovery" sequence refers to "6593_USB_PORT0_PWR Sequence 20130729.xls"*/
void usb_phy_recover(unsigned int clk_on)
{
	PHY_INT32 ret;
	os_printk(K_INFO, "%s clk_on=%d+\n", __func__, clk_on);
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,add 2015/6/15 for OTG and usb
	iddig_state = mt_get_gpio_in(GPIO_OTG_IDDIG_EINT_PIN);//kong
#endif/*VENDOR_EDIT*/

	if (!clk_on) {
#if defined(CONFIG_MTK_PMIC_CHIP_MT6353)
		/*---POWER-----*/
		/*AVDD18_USB_P0 is always turned on. The driver does _NOT_ need to control it. */
		ret = pmic_set_register_value(PMIC_LDO_VUSB33_EN, 0x01);
		if (ret)
			pr_debug("VUSB33 enable FAIL!!!\n");
#else	
		/*---POWER-----*/
		/*AVDD18_USB_P0 is always turned on. The driver does _NOT_ need to control it. */
		/*hwPowerOn(MT6332_POWER_LDO_VUSB33, VOL_3300, "VDD33_USB_P0");*/
		ret = pmic_set_register_value(MT6351_PMIC_RG_VUSB33_EN, 0x01);
		if (ret)
			pr_debug("VUSB33 enable FAIL!!!\n");

		/* Set RG_VUSB10_ON as 1 after VDD10 Ready */
		/*hwPowerOn(MT6331_POWER_LDO_VUSB10, VOL_1000, "VDD10_USB_P0");*/
		ret = pmic_set_register_value(MT6351_PMIC_RG_VA10_EN, 0x01);
		if (ret)
			pr_debug("VA10 enable FAIL!!!\n");

		ret = pmic_set_register_value(MT6351_PMIC_RG_VA10_VOSEL, 0x02);
		if (ret)
			pr_debug("VA10 output selection to 1.0v FAIL!!!\n");
#endif
		/*---CLOCK-----*/
		/* ADA_SSUSB_XTAL_CK:26MHz */
		set_ada_ssusb_xtal_ck(1);

		/* AD_LTEPLL_SSUSB26M_CK:26MHz always on */
		/* It seems that when turning on ADA_SSUSB_XTAL_CK, AD_LTEPLL_SSUSB26M_CK will also turn on. */
		/* enable_ssusb26m_ck(true); */

		/* f_fusb30_ck:125MHz */
		usb_enable_clock(true);

		/* AD_SSUSB_48M_CK:48MHz */
		/* It seems that when turning on f_fusb30_ck, AD_SSUSB_48M_CK will also turn on. */

		/* Wait 50 usec. (PHY 3.3v & 1.8v power stable time) */
		udelay(50);
#if !defined(CONFIG_USB_MU3D_ONLY_U2_MODE)
		/* Set RG_SSUSB_VUSB10_ON as 1 after VUSB10 ready */
	U3PhyWriteField32(U3D_USB30_PHYA_REG0, RG_SSUSB_VUSB10_ON_OFST, RG_SSUSB_VUSB10_ON,
			  1);
#endif
	}

	/*[MT6593 only]power domain iso disable */
	/* RG_USB20_ISO_EN       1'b0 */
	/* U3D_USBPHYACR6 E60802_RG_USB20_ISO_EN */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_ISO_EN_OFST, E60802_RG_USB20_ISO_EN, 0);

#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (!in_uart_mode) {
		/*switch to USB function. (system register, force ip into usb mode) */
		/* force_uart_en      1'b0 */
		/* U3D_U2PHYDTM0 E60802_FORCE_UART_EN */
		U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN,
				  0);
		/* RG_UART_EN         1'b0 */
		/* U3D_U2PHYDTM1 E60802_RG_UART_EN */
		U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);

		/*Release force suspendm. (force_suspendm=0) (let suspendm=1, enable usb 480MHz pll) */
		/*force_suspendm        1'b0 */
		/* U3D_U2PHYDTM0 E60802_FORCE_SUSPENDM */
		U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM,
				  0);
	}
	/* rg_usb20_gpio_ctl  1'b0 */
	/* U3D_U2PHYACR4 E60802_RG_USB20_GPIO_CTL */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL,
			  0);
	/* usb20_gpio_mode       1'b0 */
	/* U3D_U2PHYACR4 E60802_USB20_GPIO_MODE */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);
#else
	/*switch to USB function. (system register, force ip into usb mode) */
	/* force_uart_en      1'b0 */
	/* U3D_U2PHYDTM0 E60802_FORCE_UART_EN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_UART_EN_OFST, E60802_FORCE_UART_EN, 0);
	/* RG_UART_EN         1'b0 */
	/* U3D_U2PHYDTM1 E60802_RG_UART_EN */
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_UART_EN_OFST, E60802_RG_UART_EN, 0);
	/* rg_usb20_gpio_ctl  1'b0 */
	/* U3D_U2PHYACR4 E60802_RG_USB20_GPIO_CTL */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_RG_USB20_GPIO_CTL_OFST, E60802_RG_USB20_GPIO_CTL,
			  0);
	/* usb20_gpio_mode       1'b0 */
	/* U3D_U2PHYACR4 E60802_USB20_GPIO_MODE */
	U3PhyWriteField32(U3D_U2PHYACR4, E60802_USB20_GPIO_MODE_OFST, E60802_USB20_GPIO_MODE, 0);

	/*Release force suspendm. (force_suspendm=0) (let suspendm=1, enable usb 480MHz pll) */
	/*force_suspendm        1'b0 */
	/* U3D_U2PHYDTM0 E60802_FORCE_SUSPENDM */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_SUSPENDM_OFST, E60802_FORCE_SUSPENDM, 0);
#endif

	/* RG_DPPULLDOWN 1'b0 */
	/* U3D_U2PHYDTM0 E60802_RG_DPPULLDOWN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DPPULLDOWN_OFST, E60802_RG_DPPULLDOWN, 0);

	/* RG_DMPULLDOWN 1'b0 */
	/* U3D_U2PHYDTM0 E60802_RG_DMPULLDOWN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DMPULLDOWN_OFST, E60802_RG_DMPULLDOWN, 0);

	/* RG_XCVRSEL[1:0]       2'b00 */
	/* U3D_U2PHYDTM0 E60802_RG_XCVRSEL */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_XCVRSEL_OFST, E60802_RG_XCVRSEL, 0);

	/* RG_TERMSEL    1'b0 */
	/* U3D_U2PHYDTM0 E60802_RG_TERMSEL */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_TERMSEL_OFST, E60802_RG_TERMSEL, 0);

	/* RG_DATAIN[3:0]        4'b0000 */
	/* U3D_U2PHYDTM0 E60802_RG_DATAIN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_RG_DATAIN_OFST, E60802_RG_DATAIN, 0);

	/* force_dp_pulldown     1'b0 */
	/* U3D_U2PHYDTM0 E60802_FORCE_DP_PULLDOWN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DP_PULLDOWN_OFST, E60802_FORCE_DP_PULLDOWN,
			  0);

	/* force_dm_pulldown     1'b0 */
	/* U3D_U2PHYDTM0 E60802_FORCE_DM_PULLDOWN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DM_PULLDOWN_OFST, E60802_FORCE_DM_PULLDOWN,
			  0);

	/* force_xcversel        1'b0 */
	/* U3D_U2PHYDTM0 E60802_FORCE_XCVRSEL */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_XCVRSEL_OFST, E60802_FORCE_XCVRSEL, 0);

	/* force_termsel 1'b0 */
	/* U3D_U2PHYDTM0 E60802_FORCE_TERMSEL */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_TERMSEL_OFST, E60802_FORCE_TERMSEL, 0);

	/* force_datain  1'b0 */
	/* U3D_U2PHYDTM0 E60802_FORCE_DATAIN */
	U3PhyWriteField32(U3D_U2PHYDTM0, E60802_FORCE_DATAIN_OFST, E60802_FORCE_DATAIN, 0);

	/*DP/DM BC1.1 path Disable */
	/* RG_USB20_BC11_SW_EN   1'b0 */
	/* U3D_USBPHYACR6 E60802_RG_USB20_BC11_SW_EN */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST,
			  E60802_RG_USB20_BC11_SW_EN, 0);

	/*OTG Enable */
	/* RG_USB20_OTG_VBUSCMP_EN       1b1 */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_OTG_VBUSCMP_EN_OFST,
			  E60802_RG_USB20_OTG_VBUSCMP_EN, 1);
	/*Pass RX sensitivity HQA requirement */
	U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_SQTH_OFST, E60802_RG_USB20_SQTH, 0x2);

#if defined(CONFIG_MTK_HDMI_SUPPORT) || defined(MTK_USB_MODE1)
	os_printk(K_INFO, "%s- USB PHY Driving Tuning Mode 1 Settings.\n", __func__);
	U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST,
			  E60802_RG_USB20_HS_100U_U3_EN, 0);
#else
#if !defined(CONFIG_USB_MU3D_ONLY_U2_MODE)
	/*Change 100uA current switch to SSUSB */
	/* RG_USB20_HS_100U_U3_EN        1'b1 */
	/* revmoe in mt6755. use defaut value or eFuse data */
	/*U3PhyWriteField32(U3D_USBPHYACR5, E60802_RG_USB20_HS_100U_U3_EN_OFST,
			  E60802_RG_USB20_HS_100U_U3_EN, 1); */
#endif			  
#endif
	#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,add 2015/5/4 for eyesight with vref internal R

	if((iddig_state == 0) &&(get_otg_switch() == 1 ))
	//if(iddig_state == 0)
	{
		U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_TERM_VREF_SEL_OFST, E60802_RG_USB20_TERM_VREF_SEL, 1);
		U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_VRT_VREF_SEL_OFST, E60802_RG_USB20_VRT_VREF_SEL, 7);
		printk("phy_init_soc, iddig_state = %d,get_otg_switch() = %d\r\n", iddig_state,get_otg_switch());
	}
	else
	{
		U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_TERM_VREF_SEL_OFST, E60802_RG_USB20_TERM_VREF_SEL, 5);
		U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_VRT_VREF_SEL_OFST, E60802_RG_USB20_VRT_VREF_SEL, 7);
		printk("phy_init_soc, iddig_state = %d,get_otg_switch() = %d\r\n", iddig_state,get_otg_switch());
	}
	#endif/*VENDOR_EDIT*/

	/*
	 * 1 RG_SSUSB_TX_EIDLE_CM<3:0> / 1100-->1110 / low-power
	 *   E-idle common mode(650mV to 600mV) - 0x11290b18 bit [31:28]
	 * 2 RG_SSUSB_CDR_BIR_LTD0[4:0] / 5'b01000-->5'b01100 / Increase BW - 0x1128095c bit [12:8]
	 * 3 RG_XXX_CDR_BIR_LTD1[4:0] / 5'b00010-->5'b00011 / Increase BW - 0x1128095c bit [28:24]
	 */
	U3PhyWriteField32(U3D_USB30_PHYA_REG6, E60802_RG_SSUSB_TX_EIDLE_CM_OFST,
			  E60802_RG_SSUSB_TX_EIDLE_CM, 0xE);
	U3PhyWriteField32(U3D_PHYD_CDR1, E60802_RG_SSUSB_CDR_BIR_LTD0_OFST,
			  E60802_RG_SSUSB_CDR_BIR_LTD0, 0xC);
	U3PhyWriteField32(U3D_PHYD_CDR1, E60802_RG_SSUSB_CDR_BIR_LTD1_OFST,
			  E60802_RG_SSUSB_CDR_BIR_LTD1, 0x3);

	/*
	 * 1.DA_SSUSB_XTAL_EXT_EN[1:0]  2'b01-->2'b10 - 0x11290c00 bit[11:10]
	 * 2.DA_SSUSB_XTAL_RX_PWD[9:9]  -->1'b1 - 0x11280018 bit[9]
	 */
	U3PhyWriteField32(U3D_U3PHYA_DA_REG0, E60802_RG_SSUSB_XTAL_EXT_EN_U3_OFST,
			  E60802_RG_SSUSB_XTAL_EXT_EN_U3, 2);
	U3PhyWriteField32(U3D_SPLLC_XTALCTL3, E60802_RG_SSUSB_XTAL_RX_PWD_OFST,
			  E60802_RG_SSUSB_XTAL_RX_PWD, 1);

	/* Wait 800 usec */
	udelay(800);

	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_VBUSVALID_OFST, E60802_RG_VBUSVALID, 1);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_AVALID_OFST, E60802_RG_AVALID, 1);
	U3PhyWriteField32(U3D_U2PHYDTM1, E60802_RG_SESSEND_OFST, E60802_RG_SESSEND, 0);

#ifdef CONFIG_MTK_UART_USB_SWITCH
	if (in_uart_mode) {
		os_printk(K_INFO,
			  "%s- Switch to UART mode when UART cable in inserted before boot.\n",
			  __func__);
		usb_phy_switch_to_uart();
	}
#endif
	if (get_devinfo_with_index(9) & 0x1F) {
		os_printk(K_INFO, "USB HW reg: index9=0x%x\n", get_devinfo_with_index(9));
		/*PORT0 11290804[23:19]*/
		U3PhyWriteField32(U3D_USBPHYACR1, E60802_RG_USB20_INTR_CAL_OFST,  E60802_RG_USB20_INTR_CAL,
			get_devinfo_with_index(9) & (0x1F));
	}

	/* USB PLL Force settings */
	usb20_pll_settings(false, false);

	os_printk(K_INFO, "%s-\n", __func__);
}

/*
 * This function is to solve the condition described below.
 * The system boot has 3 situations.
 * 1. Booting without cable, so connection work called by musb_gadget_start()
 *    would turn off pwr/clk by musb_stop(). [REF CNT = 0]
 * 2. Booting with normal cable, the pwr/clk has already turned on at initial stage.
 *    and also set the flag (musb->is_clk_on=1).
 *    So musb_start() would not turn on again. [REF CNT = 1]
 * 3. Booting with OTG cable, the pwr/clk would be turned on by host one more time.[REF CNT=2]
 *    So device should turn off pwr/clk which are turned on during the initial stage.
 *    However, does _NOT_ touch the PHY registers. So we need this fake function to keep the REF CNT correct.
 *    NOT FOR TURN OFF PWR/CLK.
 */
void usb_fake_powerdown(unsigned int clk_on)
{
	PHY_INT32 ret;
	os_printk(K_INFO, "%s clk_on=%d+\n", __func__, clk_on);

	if (clk_on) {
		/*---CLOCK-----*/
		/* f_fusb30_ck:125MHz */
		usb_enable_clock(false);

		/*---POWER-----*/
		/* Set RG_VUSB10_ON as 1 after VDD10 Ready */
		/*hwPowerDown(MT6331_POWER_LDO_VUSB10, "VDD10_USB_P0");*/
		/* ret = pmic_set_register_value(MT6351_PMIC_RG_VA10_EN, 0x00);*/
	}

	os_printk(K_INFO, "%s-\n", __func__);
}

#ifdef CONFIG_USBIF_COMPLIANCE
static bool charger_det_en = true;

void Charger_Detect_En(bool enable)
{
	charger_det_en = enable;
}
#endif


/* BC1.2 */
void Charger_Detect_Init(void)
{
	os_printk(K_INFO, "%s+\n", __func__);

#ifdef CONFIG_USBIF_COMPLIANCE
	if (charger_det_en == true) {
#endif
		/* turn on USB reference clock. */
		usb_enable_clock(true);

		/* wait 50 usec. */
		udelay(50);

		/* RG_USB20_BC11_SW_EN = 1'b1 */
		U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST,
				  E60802_RG_USB20_BC11_SW_EN, 1);

		udelay(1);

		/* 4 14. turn off internal 48Mhz PLL. */
		usb_enable_clock(false);

#ifdef CONFIG_USBIF_COMPLIANCE
	} else {
		os_printk(K_INFO, "%s do not init detection as charger_det_en is false\n",
			  __func__);
	}
#endif

	os_printk(K_INFO, "%s-\n", __func__);
}

void Charger_Detect_Release(void)
{
	os_printk(K_INFO, "%s+\n", __func__);

#ifdef CONFIG_USBIF_COMPLIANCE
	if (charger_det_en == true) {
#endif
		/* turn on USB reference clock. */
		usb_enable_clock(true);

		/* wait 50 usec. */
		udelay(50);

		/* RG_USB20_BC11_SW_EN = 1'b0 */
		U3PhyWriteField32(U3D_USBPHYACR6, E60802_RG_USB20_BC11_SW_EN_OFST,
				  E60802_RG_USB20_BC11_SW_EN, 0);

		udelay(1);

		/* 4 14. turn off internal 48Mhz PLL. */
		usb_enable_clock(false);

#ifdef CONFIG_USBIF_COMPLIANCE
	} else {
		os_printk(K_INFO, "%s do not release detection as charger_det_en is false\n",
			  __func__);
	}
#endif

	os_printk(K_INFO, "%s-\n", __func__);
}
#endif
