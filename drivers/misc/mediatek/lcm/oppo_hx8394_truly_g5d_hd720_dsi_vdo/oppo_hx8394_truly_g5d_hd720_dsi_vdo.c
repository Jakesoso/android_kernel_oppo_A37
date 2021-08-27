/*****************************************************************
* Copyright (c) 2004-2014 OPPO Mobile communication Corp.ltd.,
* VENDOR_EDIT
* Description: Source file for LCM driver IC.
* Version   : 1.0
* Date      : 2015-12-23
* Author    : liping-m@PhoneSW.Multimedia
*----------------------Revision History--------------------------
* None.
*****************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
//#include <linux/oppo_devices_list.h>
#include <soc/oppo/device_info.h>
#include <soc/oppo/oppo_project.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/upmu_common.h>
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h>
	#include <platform/mt_pmic.h>
	#include <string.h>
    #include <platform/boot_mode.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_pm_ldo.h>
    #include <mach/mt_gpio.h>
    #include <mach/upmu_common.h>
    #include <mach/mt_boot_common.h>
#endif
#include <cust_gpio_usage.h>
#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_i2c.h>
#endif
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif


static const unsigned int BL_MIN_LEVEL =20;
static LCM_UTIL_FUNCS lcm_util;

static const unsigned char LCD_MODULE_ID = 0x00;

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))
#define UDELAY(n) 											(lcm_util.udelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update)

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE	0
#define FRAME_WIDTH  	    (720)
#define FRAME_HEIGHT 		(1280)
#define PHYSICAL_WIDTH      (68)
#define PHYSICAL_HEIGHT     (122)

//#ifndef FPGA_EARLY_PORTING
//#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
//#endif

#define GPIO_65132_ENP (GPIO12 | 0x80000000)
#define GPIO_65132_ENN (GPIO5 | 0x80000000)
#define LCD_BL_EN (GPIO11 | 0x80000000)
#define GPIO_LCM_RESET_PIN  (GPIO158| 0x80000000)


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif


extern int LM3630A_Write_Byte(kal_uint8 addr,  kal_uint8 value);
extern int LCMPower_Write_Byte(kal_uint8 addr,  kal_uint8 value);

#ifdef VENDOR_EDIT/* OPPO 2016-01-29 sjc Modify for charging */
extern void oppo_chg_set_led_status(bool val);
#endif /* VENDOR_EDIT */


#define REGFLAG_DELAY      0xFC
#define REGFLAG_UDELAY     0xFB

#define REGFLAG_END_OF_TABLE   0xFD   // END OF REGISTERS MARKER
#define REGFLAG_RESET_LOW  0xFE
#define REGFLAG_RESET_HIGH  0xFF


struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

//update initial param for IC hx8394
static struct LCM_setting_table lcm_initialization_video_setting[] = {

	{0xB9,3,{0xFF,0x83,0x94}},
	{0xBA,6,{0x63,0x03,0x68,0x6b,0xB2,0xC0}},
	{0xB1,10,{0x50,0x12,0x72,0x09,0x32,0x44,0x71,0x31,0x4F,0x35}},
	{0xB2,12,{0x60,0x80,0x64,0x10,0x07,0x2F,0x00,0x00,0x00,0x00,0xC0,0x1B}},
	{0xB4,21,{0x26,0x76,0x26,0x76,0x26,0x26,0x05,0x05,0x86,0x75,0x00,0x3F,0x26,0x76,0x26,0x76,0x26,0x26,0x05,0x05,0x86}},
	{0xD3,33,{0x00,0x00,0x04,0x04,0x01,0x01,0x10,0x10,0x32,0x10,0x00,0x00,0x00,0x32,0x15,0x04,0x05,0x04,0x32,0x15,0x14,0x05,0x14,0x37,0x33,0x04,0x04,0x37,0x00,0x00,0x47,0x05,0x40}},
	{0xD5,44,{0x18,0x18,0x25,0x24,0x27,0x26,0x11,0x10,0x15,0x14,0x13,0x12,0x17,0x16,0x01,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x05,0x04,0x03,0x02,0x07,0x06,0x18,0x18,0x18,0x18,0x21,0x20,0x23,0x22,0x18,0x18,0x18,0x18}},
	{0xD6,44,{0x18,0x18,0x22,0x23,0x20,0x21,0x12,0x13,0x16,0x17,0x10,0x11,0x14,0x15,0x06,0x07,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x02,0x03,0x04,0x05,0x00,0x01,0x18,0x18,0x18,0x18,0x26,0x27,0x24,0x25,0x18,0x18,0x18,0x18}},
	{0xE0,58,{0x00,0x06,0x10,0x17,0x1A,0x1E,0x22,0x20,0x43,0x54,0x65,0x65,0x70,0x84,0x8A,0x90,0x9C,0x9F,0x9B,0xA8,0xB7,0x5A,0x59,0x5D,0x61,0x65,0x6C,0x79,0x7F,0x00,0x06,0x10,0x17,0x1A,0x1E,0x22,0x20,0x43,0x54,0x65,0x65,0x70,0x84,0x8A,0x90,0x9C,0x9F,0x9B,0xA8,0xB7,0x5A,0x59,0x5D,0x61,0x65,0x6C,0x79,0x7F}},
	{0xC0,2,{0x1F,0x31}},
	{0xC7,2,{0x00,0xC0}},
	{0xB6,2,{0x4D,0x4D}},
	{0xCC,1,{0x07}},
	{0x36,1,{0x00}},
	{0xD2,1,{0x88}},
	{0xD4,1,{0x02}},
	{0xBD,1,{0x01}},
	{0xB1,1,{0x00}},
	{0xBD,1,{0x00}},
	{0xBC,1,{0x00}},
	{0xBD,1,{0x00}},
	{0xC1,43,{0x01,0x00,0x06,0x0D,0x13,0x1C,0x23,0x2A,0x31,0x38,0x3F,0x46,0x4D,0x54,0x5D,0x64,0x6C,0x76,0x7D,0x84,0x8C,0x93,0x9A,0xA1,0xA9,0xB2,0xB9,0xC1,0xC9,0xD2,0xDA,0xE3,0xEC,0xF6,0x22,0x00,0x05,0xC8,0x2C,0x6B,0x13,0xA3,0x40}},
    {0xBD,1,{0x01}},
    {0xC1,42,{0x00,0x06,0x0D,0x14,0x1C,0x23,0x2B,0x32,0x39,0x40,0x48,0x4F,0x57,0x5F,0x67,0x6F,0x79,0x80,0x88,0x8F,0x97,0x9E,0xA6,0xAE,0xB6,0xBE,0xC6,0xCE,0xD7,0xE0,0xE8,0xF1,0xFA,0x24,0xB1,0xB2,0x69,0x22,0x2A,0x5B,0x4D,0x00}},
    {0xBD,1,{0x02}},
    {0xC1,42,{0x00,0x07,0x0E,0x15,0x1E,0x26,0x2D,0x35,0x3D,0x44,0x4C,0x54,0x5D,0x65,0x6D,0x75,0x7F,0x87,0x8F,0x97,0x9F,0xA7,0xB0,0xB8,0xC0,0xC8,0xD1,0xDA,0xE2,0xEA,0xF2,0xF9,0xFF,0x16,0x8E,0x36,0x2D,0xFF,0xF0,0x68,0x23,0xC0}},
    {0xBD,1,{0x00}},
	{0x51,1,{0xFF}},
	{0x53,1,{0x24}},
	{0x55,1,{0x10}},
	{0x35,1,{0x00}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0xC9,7,{0x13,0x00,0x0A,0x1E,0xB1,0x1E,0x00}},
	//{0xCA,9,{0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22}},
	{REGFLAG_DELAY, 5, {}},
	{0xB2,12,{0x60,0x80,0x64,0x10,0x07,0x2F,0x00,0x00,0x00,0x00,0xC0,0x1B}},
	{REGFLAG_DELAY, 10, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
//update initial param for IC hx8394 open cabc
static struct LCM_setting_table lcm_initialization_video_setting_cabc[] = {
	{0xB9,3,{0xFF,0x83,0x94}},
	{0xBA,6,{0x63,0x03,0x68,0x6b,0xB2,0xC0}},
	{0xB1,10,{0x50,0x12,0x72,0x09,0x32,0x44,0x71,0x31,0x4F,0x35}},
	{0xB2,12,{0x60,0x80,0x64,0x10,0x07,0x2F,0x00,0x00,0x00,0x00,0xC0,0x1B}},
	{0xB4,21,{0x26,0x76,0x26,0x76,0x26,0x26,0x05,0x05,0x86,0x75,0x00,0x3F,0x26,0x76,0x26,0x76,0x26,0x26,0x05,0x05,0x86}},
	{0xD3,33,{0x00,0x00,0x04,0x04,0x01,0x01,0x10,0x10,0x32,0x10,0x00,0x00,0x00,0x32,0x15,0x04,0x05,0x04,0x32,0x15,0x14,0x05,0x14,0x37,0x33,0x04,0x04,0x37,0x00,0x00,0x47,0x05,0x40}},
	{0xD5,44,{0x18,0x18,0x25,0x24,0x27,0x26,0x11,0x10,0x15,0x14,0x13,0x12,0x17,0x16,0x01,0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x05,0x04,0x03,0x02,0x07,0x06,0x18,0x18,0x18,0x18,0x21,0x20,0x23,0x22,0x18,0x18,0x18,0x18}},
	{0xD6,44,{0x18,0x18,0x22,0x23,0x20,0x21,0x12,0x13,0x16,0x17,0x10,0x11,0x14,0x15,0x06,0x07,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x02,0x03,0x04,0x05,0x00,0x01,0x18,0x18,0x18,0x18,0x26,0x27,0x24,0x25,0x18,0x18,0x18,0x18}},
	{0xE0,58,{0x00,0x06,0x10,0x17,0x1A,0x1E,0x22,0x20,0x43,0x54,0x65,0x65,0x70,0x84,0x8A,0x90,0x9C,0x9F,0x9B,0xA8,0xB7,0x5A,0x59,0x5D,0x61,0x65,0x6C,0x79,0x7F,0x00,0x06,0x10,0x17,0x1A,0x1E,0x22,0x20,0x43,0x54,0x65,0x65,0x70,0x84,0x8A,0x90,0x9C,0x9F,0x9B,0xA8,0xB7,0x5A,0x59,0x5D,0x61,0x65,0x6C,0x79,0x7F}},
	{0xC0,2,{0x1F,0x31}},
	{0xC7,2,{0x00,0xC0}},
	{0xB6,2,{0x4D,0x4D}},
	{0xCC,1,{0x07}},
	{0x36,1,{0x00}},
	{0xD2,1,{0x88}},
	{0xD4,1,{0x02}},
	{0xBD,1,{0x01}},
	{0xB1,1,{0x00}},
	{0xBD,1,{0x00}},
	{0xBC,1,{0x00}},
	{0xBD,1,{0x00}},
	{0xC1,43,{0x01,0x00,0x06,0x0D,0x13,0x1C,0x23,0x2A,0x31,0x38,0x3F,0x46,0x4D,0x54,0x5D,0x64,0x6C,0x76,0x7D,0x84,0x8C,0x93,0x9A,0xA1,0xA9,0xB2,0xB9,0xC1,0xC9,0xD2,0xDA,0xE3,0xEC,0xF6,0x22,0x00,0x05,0xC8,0x2C,0x6B,0x13,0xA3,0x40}},
    {0xBD,1,{0x01}},
    {0xC1,42,{0x00,0x06,0x0D,0x14,0x1C,0x23,0x2B,0x32,0x39,0x40,0x48,0x4F,0x57,0x5F,0x67,0x6F,0x79,0x80,0x88,0x8F,0x97,0x9E,0xA6,0xAE,0xB6,0xBE,0xC6,0xCE,0xD7,0xE0,0xE8,0xF1,0xFA,0x24,0xB1,0xB2,0x69,0x22,0x2A,0x5B,0x4D,0x00}},
    {0xBD,1,{0x02}},
    {0xC1,42,{0x00,0x07,0x0E,0x15,0x1E,0x26,0x2D,0x35,0x3D,0x44,0x4C,0x54,0x5D,0x65,0x6D,0x75,0x7F,0x87,0x8F,0x97,0x9F,0xA7,0xB0,0xB8,0xC0,0xC8,0xD1,0xDA,0xE2,0xEA,0xF2,0xF9,0xFF,0x16,0x8E,0x36,0x2D,0xFF,0xF0,0x68,0x23,0xC0}},
    {0xBD,1,{0x00}},
	{0x51,1,{0xFF}},
	{0x53,1,{0x24}},
	{0x55,1,{0x11}},
	{0x35,1,{0x00}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0xC9,7,{0x13,0x00,0x0A,0x1E,0xB1,0x1E,0x00}},
	//{0xCA,9,{0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22}},
	{REGFLAG_DELAY, 5, {}},
	{0xB2,12,{0x60,0x80,0x64,0x10,0x07,0x2F,0x00,0x00,0x00,0x00,0xC0,0x1B}},
	{REGFLAG_DELAY, 10, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 2, {}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}},
};

/*
static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;

			case REGFLAG_UDELAY :
				UDELAY(table[i].count);
				break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

#ifndef BUILD_LK
//extern LCD_DEV lcd_dev;
#endif

static void lcm_get_params(LCM_PARAMS *params)
{
    int boot_mode = 0;
    //hw_version_identify();

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

    params->physical_width = PHYSICAL_WIDTH;
    params->physical_height = PHYSICAL_HEIGHT;


       // params->dsi.mode   = CMD_MODE;// NON IPC;
	   params->dsi.mode   = BURST_VDO_MODE; //BURST_VDO_MODE;
       // params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
       // params->dbi.te_edge_polarity = LCM_POLARITY_RISING;


	/* Command mode setting */
	params->dsi.LANE_NUM				=LCM_FOUR_LANE; //LCM_THREE_LANE; //LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      		= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 20;
	params->dsi.vertical_frontporch					= 20;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 2;
	params->dsi.horizontal_backporch				= 125;
	params->dsi.horizontal_frontporch				= 125;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

 	params->dsi.LPX=5;
	params->dsi.PLL_CLOCK = 240; //327; //247;  //220; //465;
    params->dsi.ssc_disable=1;//MUST Disable SSC

	//clk continuous video mode
	params->dsi.cont_clock=0;

	params->dsi.clk_lp_per_line_enable =0;
    if (g_boot_mode == META_BOOT) {
        boot_mode++;
        LCD_DEBUG("META_BOOT\n");
    }
    if (g_boot_mode == ADVMETA_BOOT) {
        boot_mode++;
        LCD_DEBUG("ADVMETA_BOOT\n");
    }
    if (g_boot_mode == ATE_FACTORY_BOOT) {
        boot_mode++;
        LCD_DEBUG("ATE_FACTORY_BOOT\n");
    }
    if (g_boot_mode == FACTORY_BOOT) {
        boot_mode++;
        LCD_DEBUG("FACTORY_BOOT\n");
    }
    if (boot_mode == 0) {
        LCD_DEBUG("neither META_BOOT or FACTORY_BOOT\n");
		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd			= 0x09;
		params->dsi.lcm_esd_check_table[0].count		= 2;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
		params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
    }

#ifndef BUILD_LK
    //lcd_dev = LCD_JDI_R63417_VIDEO;
	register_device_proc("lcd", "hx8394", "truly g5d video mode");
#endif
}



static void lcm_init_power(void)
{
    LCD_DEBUG("[soso] lcm_init_power\n");
	//For LCM 5V Power
	LCMPower_Write_Byte(0x00,0x0F);//+5.5V
	LCMPower_Write_Byte(0x03,0x0F);//DIS Address
	LCMPower_Write_Byte(0x01,0x0F);//-5.5V
	LCMPower_Write_Byte(0xFF,0xF0);//DIS Address
	MDELAY(1);
    // FOR LCM +5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
	MDELAY(7);
	// FOR LCM -5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
	MDELAY(10); //> 5ms
    //LCD RESET
    mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);
    MDELAY(1);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(2);
	mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(52);
}

static void lcm_suspend_power(void)
{
    LCD_DEBUG("[soso] lcm_suspend_power\n");
    //For BL
	mt_set_gpio_mode(LCD_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BL_EN, GPIO_OUT_ZERO);
    LCD_DEBUG(" lcm_suspend_power BL has disabled\n");
    //LCD RESET
    mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(2);
	// FOR LCM -5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ZERO);
	MDELAY(5);
    // FOR LCM +5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ZERO);
    MDELAY(5);
}

static void lcm_resume_power(void)
{
    LCD_DEBUG("[soso] lcm_resume_power \n");
	lcm_init_power();
}


static void lcm_init(void)
{
	//unsigned char cmd = 0x0;
	//unsigned char data = 0xFF;
	//int ret=0;

    LCD_DEBUG("[soso]lcm_initialization_setting\n");


    // when phone initial , config output high, enable backlight drv chip
   // push_table(lcm_initialization_cmd_setting, sizeof(lcm_initialization_cmd_setting) / sizeof(struct LCM_setting_table), 1);
    if(get_PCB_Version() >= HW_VERSION__13){
        push_table(lcm_initialization_video_setting_cabc, sizeof(lcm_initialization_video_setting_cabc) / sizeof(struct LCM_setting_table), 1);
    }else{
        push_table(lcm_initialization_video_setting, sizeof(lcm_initialization_video_setting) / sizeof(struct LCM_setting_table), 1);
    }

    //Enable BL
    mt_set_gpio_mode(LCD_BL_EN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BL_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BL_EN, GPIO_OUT_ONE);
	MDELAY(2); //> 1ms

    if(get_PCB_Version() >= HW_VERSION__13){
        LM3630A_Write_Byte(0x50,0x03);
        LM3630A_Write_Byte(0x01,0x19);  //open PWM
    }else{
        LM3630A_Write_Byte(0x01,0x18);  //close PWM
    }

    LM3630A_Write_Byte(0x00,0x1D);  //linear mode
	LM3630A_Write_Byte(0x05,0x14);
	LM3630A_Write_Byte(0x06,0x14);

	//LM3630A_Write_Byte(0x03,0xFF);//Bank Level
	//LM3630A_Write_Byte(0x04,0xFF);//Bank Level
}


static void lcm_suspend(void)
{


	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);

	//printk(KERN_ERR,"[SOSO]lcm_suspend IPC_I2C_Init\n");
	LCD_DEBUG("[soso] lcm_suspend \n");

#ifdef VENDOR_EDIT/* OPPO 2016-01-29 sjc Modify for charging */
	oppo_chg_set_led_status(false);
#endif /* VENDOR_EDIT */

}

static void lcm_resume(void)
{
    LCD_DEBUG("[soso] lcm_resume\n");
	lcm_init();

#ifdef VENDOR_EDIT/* OPPO 2016-01-29 sjc Modify for charging */
	oppo_chg_set_led_status(true);
#endif /* VENDOR_EDIT */

}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
#if 1
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
#endif
}

#if 0
static unsigned int lcm_compare_id(void)
{

    unsigned char LCD_ID_value = 0;
    LCD_ID_value = which_lcd_module_triple();
    if(LCD_MODULE_ID == LCD_ID_value)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}
#endif
int  truly_g5d_himax_set_cabc_mode(void *handle,unsigned int level)
{
	// Refresh value of backlight level.

    int mapped_level = 0;
    unsigned char value = 0;
    unsigned int cmd = 0x55;
    unsigned char count =1;
    if (level==1) {
        mapped_level = 0x11;
    } else if(level ==2) {
        mapped_level = 0x12;
    }else if(level ==3){
        mapped_level = 0x13;
    }else{
        mapped_level = 0x10;
    }
    if(get_PCB_Version() >= HW_VERSION__13){
        printk("%s = %d\n",__func__,level);
        value = mapped_level;
        dsi_set_cmdq_V22(handle, cmd, count, &value, 1);
    }
    return 0;

}

// return TRUE: need recovery
// return FALSE: No need recovery
/*
static unsigned int lcm_esd_check(void)
{



	return FALSE;//No ESD
}
*/
static void lcm_setbacklight(unsigned int level)
{
	// Refresh value of backlight level.
#ifdef BUILD_LK
	dprintf(CRITICAL, "%s [soso] level is %d\n", __func__, level);
#else
    printk("%s [soso] level is %d\n", __func__, level);
#endif

    if (level == 0) {
        LM3630A_Write_Byte(0x03,level);
	    LM3630A_Write_Byte(0x04,level);
        LM3630A_Write_Byte(0x00,0x9D);
    } else {
    	LM3630A_Write_Byte(0x03,level);//Bank Level
    	LM3630A_Write_Byte(0x04,level);//Bank Level
    	LM3630A_Write_Byte(0x00,0x1D);
    }
}


LCM_DRIVER oppo_hx8394_truly_g5d_hd720_dsi_vdo_lcm_drv=
{
    .name           	= "oppo_hx8394_truly_g5d_hd720_dsi_vdo",
    .set_util_funcs 	= lcm_set_util_funcs,
    .get_params     	= lcm_get_params,
    .init           	= lcm_init,
    .suspend        	= lcm_suspend,
    .resume         	= lcm_resume,
//     .compare_id     	= lcm_compare_id,
     .init_power		= lcm_init_power,
     .resume_power = lcm_resume_power,
     .suspend_power = lcm_suspend_power,
//     .esd_check = lcm_esd_check,
     .set_backlight = lcm_setbacklight,
//	 .ata_check		= lcm_ata_check,
//#if (LCM_DSI_CMD_MODE)
	 .update         = lcm_update,
//#endif
};
