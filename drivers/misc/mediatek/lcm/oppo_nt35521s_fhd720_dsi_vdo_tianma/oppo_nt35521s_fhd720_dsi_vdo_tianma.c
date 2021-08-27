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
#include <linux/oppo_devices_list.h>
#include <soc/oppo/device_info.h>
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


static const unsigned char LCD_MODULE_ID = 0x01;

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)                                             (lcm_util.mdelay(n))
#define UDELAY(n)                                             (lcm_util.udelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                        lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                    lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                        lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V22(cmdq,cmd, count, ppara, force_update)

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE    0
#define FRAME_WIDTH          (720)
#define FRAME_HEIGHT         (1280)
#define PHYSICAL_WIDTH      (62)
#define PHYSICAL_HEIGHT     (110)

#ifndef FPGA_EARLY_PORTING
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#endif

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


extern int lm3630a_setbacklight(unsigned int level);
extern int LM3630A_Write_Byte(kal_uint8 addr,  kal_uint8 value);
extern int LCMPower_Write_Byte(kal_uint8 addr,  kal_uint8 value);


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

//update initial param for IC nt35520 0.01
static struct LCM_setting_table lcm_initialization_video_setting[] ={

        //add 3 lane settings
        {0XFF,4,{0XAA,0X55,0X25,0X01}},
        {0X6F,1,{0X16}},
        {0XF7,1,{0X10}},
        {0XFF,4,{0XAA,0X55,0X25,0X00}},

        {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
        {0x6F,1,{0x02}},
        {0xB8,1,{0x01}},
        {0xEC,1,{0x05}}, // DVD
        {0xBB,2,{0x11,0x11}},
        {0xBC,2,{0x00,0x00}},
        {0xC8,1,{0x80}},
        {0xB6,1,{0x06}},
        {0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
        {0xB0,2,{0x0F,0x0F}},
        {0xB1,2,{0x09,0x09}},
        {0xBC,2,{0x80,0x00}},
        {0xBD,2,{0x80,0x00}},
        {0xCA,1,{0x00}},
        {0xC0,1,{0x04}},
        {0xB5,2,{0x03,0x03}},
        //{0xBE,1,{0x48}},
        {0xB3,2,{0x19,0x19}},
        {0xB4,2,{0x0F,0x0F}},
        {0xB9,2,{0x34,0x34}},
        {0xBA,2,{0x15,0x15}},

        {0xF0,  5,    {0x55,0xaa,0x52,0x08,0x02}},
        {0xEE,  1,    {0x03}},
        //R(+) MCR cmd
        {0xB0,16,{0x00,0x00,0x00,0x16,0x00,0x3A,0x00,0x56,0x00,0x6D,0x00,0x94,0x00,0xB3,0x00,0xE6}},
        {0xB1,16,{0x01,0x0F,0x01,0x4F,0x01,0x82,0x01,0xD5,0x02,0x1A,0x02,0x1C,0x02,0x5D,0x02,0xA2}},
        {0xB2,16,{0x02,0xCB,0x03,0x00,0x03,0x22,0x03,0x50,0x03,0x6D,0x03,0x92,0x03,0xAB,0x03,0xC1}},
        {0xB3,4,{0x03,0xD1,0x03,0xD3}},
        //G(+) MCR cmd
        {0xB4,16,{0x00,0x9F,0x00,0xA7,0x00,0xB5,0x00,0xC2,0x00,0xCE,0x00,0xE4,0x00,0xF9,0x01,0x22}},
        {0xB5,16,{0x01,0x42,0x01,0x72,0x01,0x9D,0x01,0xE9,0x02,0x26,0x02,0x28,0x02,0x65,0x02,0xA8}},
        {0xB6,16,{0x02,0xD0,0x03,0x03,0x03,0x26,0x03,0x50,0x03,0x6D,0x03,0x91,0x03,0xA8,0x03,0xBF}},
        {0xB7,4,{0x03,0xD0,0x03,0xD3}},
        //B(+) MCR cmd
        {0xB8,16,{0x00,0x5C,0x00,0x66,0x00,0x78,0x00,0x88,0x00,0x97,0x00,0xB2,0x00,0xCB,0x00,0xFB}},
        {0xB9,16,{0x01,0x20,0x01,0x57,0x01,0x87,0x01,0xDB,0x02,0x1C,0x02,0x1E,0x02,0x5E,0x02,0xA4}},
        {0xBA,16,{0x02,0xCD,0x03,0x01,0x03,0x26,0x03,0x5A,0x03,0x83,0x03,0xA9,0x03,0xB0,0x03,0xBF}},
        {0xBB,4,{0x03,0xD1,0x03,0xD3}},

        //{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},
        //{0xEE,1,{0x01}},
        //{0xB0,16,{0x00,0x00,0x00,0x6E,0x00,0x9D,0x00,0xBB,0x00,0xD3,0x00,0xF7,0x01,0x14,0x01,0x42}},
        //{0xB1,16,{0x01,0x66,0x01,0x9F,0x01,0xCC,0x02,0x12,0x02,0x4C,0x02,0x4D,0x02,0x82,0x02,0xBB}},
        //{0xB2,16,{0x02,0xDD,0x03,0x09,0x03,0x27,0x03,0x4E,0x03,0x67,0x03,0x85,0x03,0x99,0x03,0xB1}},
        //{0xB3,4,{0x03,0xD0,0x03,0xFF}},


        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x06}},
        {0xB0, 2,{0x08,0x34}},
        {0xB1, 2,{0x31,0x34}},
        {0xB2, 2,{0x34,0x2E}},
        {0xB3, 2,{0x2D,0x34}},
        {0xB4, 2,{0x00,0x02}},
        {0xB5, 2,{0x34,0x12}},
        {0xB6, 2,{0x10,0x34}},
        {0xB7, 2,{0x34,0x18}},
        {0xB8, 2,{0x16,0x34}},
        {0xB9, 2,{0x34,0x34}},
        {0xBA, 2,{0x34,0x34}},
        {0xBB, 2,{0x34,0x17}},
        {0xBC, 2,{0x19,0x34}},
        {0xBD, 2,{0x34,0x11}},
        {0xBE, 2,{0x13,0x34}},
        {0xBF, 2,{0x03,0x01}},
        {0xC0, 2,{0x34,0x2D}},
        {0xC1, 2,{0x2E,0x34}},
        {0xC2, 2,{0x34,0x31}},
        {0xC3, 2,{0x34,0x09}},
        {0xE5, 2,{0x34,0x34}},
        {0xC4, 2,{0x09,0x34}},
        {0xC5, 2,{0x31,0x34}},
        {0xC6, 2,{0x34,0x2D}},
        {0xC7, 2,{0x2E,0x34}},
        {0xC8, 2,{0x03,0x01}},
        {0xC9, 2,{0x34,0x17}},
        {0xCA, 2,{0x19,0x34}},
        {0xCB, 2,{0x34,0x11}},
        {0xCC, 2,{0x13,0x34}},
        {0xCD, 2,{0x34,0x34}},
        {0xCE, 2,{0x34,0x34}},
        {0xCF, 2,{0x34,0x12}},
        {0xD0, 2,{0x10,0x34}},
        {0xD1, 2,{0x34,0x18}},
        {0xD2, 2,{0x16,0x34}},
        {0xD3, 2,{0x00,0x02}},
        {0xD4, 2,{0x34,0x2E}},
        {0xD5, 2,{0x2D,0x34}},
        {0xD6, 2,{0x34,0x31}},
        {0xD7, 2,{0x34,0x08}},
        {0xE6, 2,{0x34,0x34}},
        {0xD8, 5,{0x00,0x00,0x00,0x00,0x00}},
        {0xD9, 5,{0x00,0x00,0x00,0x00,0x00}},
        {0xE7, 1,{0x00}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
        {0xED, 1,{0x30}},
        {0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},
        {0xB1,2,{0x20,0x00}},
        {0xB0, 2,{0x20,0x00}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
        {0xE5, 1,{0x00}},
        {0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},
        {0xB0,2,{0x17,0x06}},
        {0xB8,1,{0x00}},
        {0xBD, 5,{0x03,0x03,0x01,0x00,0x03}},
        {0xB1,2,{0x17,0x06}},
        {0xB9, 2,{0x00,0x03}},
        {0xB2,2,{0x17,0x06}},
        {0xBA, 2,{0x00,0x00}},
        {0xB3,2,{0x17,0x06}},
        {0xBB, 2,{0x00,0x00}},
        {0xB4,2,{0x17,0x06}},
        {0xB5,2,{0x17,0x06}},
        {0xB6,2,{0x17,0x06}},
        {0xB7,2,{0x17,0x06}},
        {0xBC, 2,{0x00,0x03}},
        {0xE5,1,{0x06}},
        {0xE6,1,{0x06}},
        {0xE7,1,{0x06}},
        {0xE8,1,{0x06}},
        {0xE9, 1,{0x06}},
        {0xEA,1,{0x06}},
        {0xEB,1,{0x06}},
        {0xEC,1,{0x06}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
        {0xC0, 1,{0x0B}},
        {0xC1, 1,{0x09}},
        {0xC2, 1,{0x0B}},
        {0xC3, 1,{0x09}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x03}},
        {0xB2, 5,{0x05,0x00,0x00,0x00,0x90}},
        {0xB3, 5,{0x05,0x00,0x00,0x00,0x90}},
        {0xB4, 5,{0x05,0x00,0x00,0x00,0x90}},
        {0xB5, 5,{0x05,0x00,0x00,0x00,0x90}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
        {0xC4, 1,{0x10}},
        {0xC5, 1,{0x10}},
        {0xC6, 1,{0x10}},
        {0xC7, 1,{0x10}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x03}},
        {0xB6, 5,{0x05,0x00,0x00,0x00,0x90}},
        {0xB7, 5,{0x05,0x00,0x00,0x00,0x90}},
        {0xB8, 5,{0x05,0x00,0x00,0x00,0x90}},
        {0xB9, 5,{0x05,0x00,0x00,0x00,0x90}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
        {0xC8, 2,{0x07,0x20}},
        {0xC9, 2,{0x03,0x20}},
        {0xCA, 2,{0x07,0x00}},
        {0xCB, 2,{0x03,0x00}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x03}},
        {0xBA, 5,{0x53,0x00,0x00,0x00,0x00}},
        {0xBB, 5,{0x53,0x00,0x00,0x00,0x00}},
        {0xBC, 5,{0x53,0x00,0x00,0x00,0x00}},
        {0xBD, 5,{0x53,0x00,0x00,0x00,0x00}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
        {0xD1, 5,{0x00,0x05,0x00,0x07,0x10}},
        {0xD2, 5,{0x00,0x05,0x04,0x07,0x10}},
        {0xD3, 5,{0x00,0x00,0x0A,0x07,0x10}},
        {0xD4, 5,{0x00,0x00,0x0A,0x07,0x10}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
        {0xD0, 7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
        {0xD5, 11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
        {0xD6, 11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
        {0xD7, 11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
        {0xD8,5,{0x00,0x00,0x00,0x00,0x00}},
        {0xF0, 5,{0x55,0xAA,0x52,0x08,0x03}},
        {0xC4, 1,{0x60}},
        {0xC5, 1,{0x40}},
        {0xC6, 1,{0x60}},
        {0xC7, 1,{0x40}},
        //{0x6F, 1,{0x01}},
        //{0xF9, 1,{0x46}},


        //// OSC Spread  open SSC
        //{0XF0,5,{0X55,0XAA,0X52,0X08,0X04}},
        //{0XC2,3,{0X80,0X60,0X00}},
        //{0XC0,1,{0X00}},
        {0XF0,5,{0X55,0XAA,0X52,0X08,0X04}},
        {0XC3,1,{0X83}},

        {0XF0,5,{0X55,0XAA,0X52,0X08,0X00}},
        {0XC8,1,{0X80}},

        // PWM 30K HZ
        //{0XF0,5,{0X55,0XAA,0X52,0X08,0X00}},
        {0xD9,5,{0x01,0x01,0x4D}},

        //CABC settings
        {0X51,1,{0XFF}},
        {0X53,1,{0X24}},
        {0X55,1,{0X81}},

        //CE
        {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
        {0xCC,6,{0x40,0x36,0x87,0x00,0x00,0x00}},
        {0xCE,12,{0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10}},
        {0xD1,16,{0x00,0x02,0x03,0x04,0x06,0x08,0x0a,0x0C,0x0E,0x0E,0x0C,0x0A,0x08,0x04,0x02,0x00}},

        {0x11,    1,    {0x00}},
        {REGFLAG_DELAY, 120, {}},

        {0x29,    1,    {0x00}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
    {0x28,0,{}},
    {REGFLAG_DELAY, 50, {}},
    {0x10,0,{}},
    {REGFLAG_DELAY, 120, {}},
};

static struct LCM_setting_table lcm_cabc_level_setting[] = {
    {0x55, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

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
extern LCD_DEV lcd_dev;
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
    params->dsi.LANE_NUM                =LCM_THREE_LANE; //LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;


    // Highly depends on LCD driver capability.
    params->dsi.packet_size=256;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;


    params->dsi.vertical_sync_active                = 2;
    params->dsi.vertical_backporch                    = 14;
    params->dsi.vertical_frontporch                    = 14;
    params->dsi.vertical_active_line                = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active                = 4;
    params->dsi.horizontal_backporch                = 92;
    params->dsi.horizontal_frontporch                = 92;
    params->dsi.horizontal_active_pixel         = FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 276; //465;
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
        params->dsi.lcm_esd_check_table[0].cmd            = 0x0A;
        params->dsi.lcm_esd_check_table[0].count        = 1;
        params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
        params->dsi.lcm_esd_check_table[1].cmd            = 0x7F;
        params->dsi.lcm_esd_check_table[1].count        = 1;
        params->dsi.lcm_esd_check_table[1].para_list[0] = 0x01;
    }

#ifndef BUILD_LK
    lcd_dev = LCD_TIANMA_NT35521S_VIDEO;
    register_device_proc("lcd", "NT35521S", "Tianma TFT video mode");
#endif
}



static void lcm_init_power(void)
{
    LCD_DEBUG("[lcd] lcm_init_power\n");

    // FOR LCM +5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
    MDELAY(5); //> 1ms
    // FOR LCM -5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
    MDELAY(5); //> 1ms

    //For LCM 5V Power
    LCMPower_Write_Byte(0x00,0x0A);//+5V
    LCMPower_Write_Byte(0x01,0x0A);//-5V
    LCMPower_Write_Byte(0x03,0x0F);//DIS Address
    LCMPower_Write_Byte(0xFF,0xF0);//DIS Address
     MDELAY(1); //> 1ms


    //LCD RESET
    mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);
    MDELAY(1);
    mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ONE);// TODO:
    MDELAY(10);
}

static void lcm_suspend_power(void)
{
    LCD_DEBUG("[lcd] lcm_suspend_power\n");

    //For BL
    mt_set_gpio_mode(LCD_BL_EN, GPIO_MODE_00);
    mt_set_gpio_dir(LCD_BL_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(LCD_BL_EN, GPIO_OUT_ZERO);
    LCD_DEBUG("[lcd] lcm_suspend_power BL has disabled\n");

    //LCD RESET
    mt_set_gpio_mode(GPIO_LCM_RESET_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RESET_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RESET_PIN, GPIO_OUT_ZERO);
    MDELAY(5);

    // FOR LCM -5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ZERO);
    MDELAY(5);

    // FOR LCM +5V Power Enable
    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ZERO);
    MDELAY(10);

    LCD_DEBUG("[lcd]  lcm_suspend_power end\n");

}

static void lcm_resume_power(void)
{
    LCD_DEBUG("[lcd] lcm_resume_power \n");
    lcm_init_power();
}


static void lcm_init(void)
{
    LCD_DEBUG("[lcd]lcm_initialization_setting\n");


    // when phone initial , config output high, enable backlight drv chip
   // push_table(lcm_initialization_cmd_setting, sizeof(lcm_initialization_cmd_setting) / sizeof(struct LCM_setting_table), 1);
     push_table(lcm_initialization_video_setting, sizeof(lcm_initialization_video_setting) / sizeof(struct LCM_setting_table), 1);

    //Enable BL
    mt_set_gpio_mode(LCD_BL_EN, GPIO_MODE_00);
    mt_set_gpio_dir(LCD_BL_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(LCD_BL_EN, GPIO_OUT_ONE);
    MDELAY(2); //> 1ms

    //LM3630A_Write_Byte(0x01,0x18);  //Disable PWM
    LM3630A_Write_Byte(0x01,0x19);  //open PWM

    LM3630A_Write_Byte(0x00,0x1F);  //linear mode
    LM3630A_Write_Byte(0x05,0x14);  //0x16 22ma 0x14 20ma
    LM3630A_Write_Byte(0x06,0x14);  //0x16 22ma 0x14 20ma

    //LM3630A_Write_Byte(0x03,0xFF);//Bank Level
    //LM3630A_Write_Byte(0x04,0xFF);//Bank Level
}


static void lcm_suspend(void)
{

    LCD_DEBUG("[lcd] lcm_suspend \n");
    push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);

}

static void lcm_resume(void)
{
    LCD_DEBUG("[lcd] lcm_resume\n");
    lcm_init();

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


int lcm_set_CABC_mode_tianma(int mode)
{
    int mapped_level = 0;

    if (mode==1) {
        mapped_level = 0x81;
    } else if(mode==2){
        mapped_level = 0x82;
    }else if(mode==3){
        mapped_level = 0x83;
    }else{
        mapped_level = 0x80;
    }

    lcm_cabc_level_setting[0].para_list[0] = mapped_level;
    push_table(lcm_cabc_level_setting,sizeof(lcm_cabc_level_setting)/sizeof(lcm_cabc_level_setting[0]),1);

    return 0;
}

int tianma_lcm_set_cabc_mode(void *handle,unsigned int level)
{
    // Refresh value of backlight level.
    int mapped_level = 0;
    //unsigned int value = 0;
    //unsigned int cmd = 0x55;
    //unsigned int count =1;
    if (level==1) {
        mapped_level = 0x81;
    } else if(level ==2) {
        mapped_level = 0x82;
    }else if(level ==3){
        mapped_level = 0x83;
    }else{
        mapped_level = 0x80;
    }

    lcm_cabc_level_setting[0].para_list[0] = mapped_level;
    push_table(lcm_cabc_level_setting,sizeof(lcm_cabc_level_setting)/sizeof(lcm_cabc_level_setting[0]),1);

    return 0;

}

static void lcm_setbacklight(unsigned int level)
{
    // Refresh value of backlight level.
#ifdef BUILD_LK
    dprintf(CRITICAL, "%s [lcd] level is %d\n", __func__, level);
#else
    printk("%s [lcd] level is %d\n", __func__, level);
#endif
    lm3630a_setbacklight(level);
}



LCM_DRIVER oppo_nt35521s_fhd720_dsi_vdo_tianma_lcm_drv=
{
    .name               = "oppo_nt35521s_fhd720_dsi_vdo_tianma",
    .set_util_funcs     = lcm_set_util_funcs,
    .get_params         = lcm_get_params,
    .init               = lcm_init,
    .suspend            = lcm_suspend,
    .resume             = lcm_resume,
//     .compare_id         = lcm_compare_id_tianma,
     .init_power        = lcm_init_power,
     .resume_power      = lcm_resume_power,
     .suspend_power     = lcm_suspend_power,
//     .esd_check       = lcm_esd_check,
     .set_backlight     = lcm_setbacklight,
//#if (LCM_DSI_CMD_MODE)
      .update            = lcm_update,
//#endif
};
