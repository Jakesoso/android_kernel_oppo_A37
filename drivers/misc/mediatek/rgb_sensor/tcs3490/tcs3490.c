/*
 * Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), and Beam functionality within the
 * AMS-TAOS TCS family of devices.
 *
 * Copyright (c) 2014, AMS-TAOS USA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <tcs3490.h>

#include <mach/mt_pm_ldo.h>
#include <cust_i2c.h>
#include "pmic_drv.h"
#include <rgb_core.h>

#define TAG_NAME "[tcs3490]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE 
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_ERROR
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

#define TCS3490_CMD_ALS_INT_CLR  0xE6
#define TCS3490_CMD_ALL_INT_CLR	0xE7

#define INTEGRATION_CYCLE 278

#define I2C_ADDR_OFFSET	0X80
#define I2C_SLAVE_ADDRESS   0x72
#define I2C_BUSNUM               4
#define PLATFORM_DRIVER_NAME "rgb_sensor"
#define TCS3490_DRVNAME           "tcs3490"
static struct i2c_board_info tcs3490_dev __initdata = { I2C_BOARD_INFO("tcs3490", 0x39) };
struct tcs3490_chip *chip_func;


enum tcs3490_regs {
	TCS3490_CONTROL,
	TCS3490_ALS_TIME,                  // 0x81
	TCS3490_RESV_1,
	TCS3490_WAIT_TIME,               // 0x83
	TCS3490_ALS_MINTHRESHLO,   // 0x84
	TCS3490_ALS_MINTHRESHHI,   // 0x85
	TCS3490_ALS_MAXTHRESHLO,  // 0x86
	TCS3490_ALS_MAXTHRESHHI,  // 0x87
	TCS3490_RESV_2,                     // 0x88
	TCS3490_PRX_MINTHRESHLO,  // 0x89 -> Not used for TCS3490 

	TCS3490_RESV_3,                    // 0x8A 
	TCS3490_PRX_MAXTHRESHHI, // 0x8B  -> Not used for TCS3490 
	TCS3490_PERSISTENCE,          // 0x8C   
	TCS3490_CONFIG,                    // 0x8D
	TCS3490_PRX_PULSE_COUNT,  // 0x8E  -> Not used for TCS3490
	TCS3490_GAIN,                        // 0x8F  : Gain Control Register  
	TCS3490_AUX,                          // 0x90  
	TCS3490_REVID,
	TCS3490_CHIPID,
	TCS3490_STATUS,                    // 0x93

	TCS3490_CLR_CHANLO,            // 0x94
	TCS3490_CLR_CHANHI,            // 0x95
	TCS3490_RED_CHANLO,           // 0x96
	TCS3490_RED_CHANHI,           // 0x97
	TCS3490_GRN_CHANLO,           // 0x98
	TCS3490_GRN_CHANHI,           // 0x99 
	TCS3490_BLU_CHANLO,           // 0x9A
	TCS3490_BLU_CHANHI,           // 0x9B
	TCS3490_PRX_HI,                    // 0x9C
	TCS3490_PRX_LO,                    // 0x9D

	TCS3490_PRX_OFFSET,            // 0x9E
	TCS3490_RESV_4,                    // 0x9F
	TCS3490_IRBEAM_CFG,            // 0xA0  
	TCS3490_IRBEAM_CARR,          // 0xA1   
	TCS3490_IRBEAM_NS,              // 0xA2 
	TCS3490_IRBEAM_ISD,            // 0xA3 
	TCS3490_IRBEAM_NP,              // 0xA4
	TCS3490_IRBEAM_IPD,            // 0xA5
	TCS3490_IRBEAM_DIV,            // 0xA6
	TCS3490_IRBEAM_LEN,            // 0xA7 

	TCS3490_IRBEAM_STAT,         // 0xA8
	TCS3490_REG_MAX,

};

enum tcs3490_en_reg {
	TCS3490_EN_PWR_ON   = (1 << 0),
	TCS3490_EN_ALS      = (1 << 1),
	TCS3490_EN_PRX      = (1 << 2),
	TCS3490_EN_WAIT     = (1 << 3),
	TCS3490_EN_ALS_IRQ  = (1 << 4),
	TCS3490_EN_PRX_IRQ  = (1 << 5),
	TCS3490_EN_IRQ_PWRDN = (1 << 6),
	TCS3490_EN_BEAM     = (1 << 7),
};

enum tcs3490_status {
	TCS3490_ST_ALS_VALID  = (1 << 0),
	TCS3490_ST_PRX_VALID  = (1 << 1),
	TCS3490_ST_BEAM_IRQ   = (1 << 3),
	TCS3490_ST_ALS_IRQ    = (1 << 4),
	TCS3490_ST_PRX_IRQ    = (1 << 5),
	TCS3490_ST_PRX_SAT    = (1 << 6),
};

enum {
	TCS3490_ALS_GAIN_MASK = (3 << 0),
	TCS3490_PRX_GAIN_MASK = (3 << 2),
	TCS3490_ALS_AGL_MASK  = (1 << 2),
	TCS3490_ALS_AGL_SHIFT = 2,
	TCS3490_ATIME_PER_100 = 273,
	TCS3490_ATIME_DEFAULT_MS = 50,
	SCALE_SHIFT = 11,
	RATIO_SHIFT = 10,
	MAX_ALS_VALUE = 0xffff,
	MIN_ALS_VALUE = 10,
	GAIN_SWITCH_LEVEL = 100,
	GAIN_AUTO_INIT_VALUE = AGAIN_16,
};

static u8 const tcs3490_ids[] = {
	0x84,		//tcs34901&tcs34905
	0x87,		//tcs34903&tcs34907
};

static char const *tcs3490_names[] = {
	"tcs34901",
	"tcs34903",
};

static u8 const restorable_regs[] = {
	TCS3490_ALS_TIME,
	TCS3490_WAIT_TIME,
	TCS3490_PERSISTENCE,
	TCS3490_PRX_PULSE_COUNT,
	TCS3490_GAIN,
	TCS3490_PRX_OFFSET,
};

static u8 const als_gains[] = {
	1,
	4,
	16,
	64
};

struct tcs3490_als_info {
	u32 cpl;
	u32 saturation;
	u16 clear_raw;
	u16 red_raw;
	u16 green_raw;
	u16 blue_raw;
	u16 realIR_raw;
	u16 lux;
	u16 cct;
	s16 ir;
};

static struct lux_segment segment_default[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};

struct tcs3490_chip {
	struct mutex lock;
	struct i2c_client *client;
	struct tcs3490_als_info als_inf;
	struct tcs3490_parameters params;
	struct tcs3490_i2c_platform_data *pdata;
	u8 shadow[42];

	struct input_dev *a_idev;
	int in_suspend;
	int wake_irq;
	int irq_pending;
	bool unpowered;
	bool als_enabled;

	struct lux_segment *segment;
	int segment_num;
	int seg_num_max;
	bool als_gain_auto;
	u8 device_index;
	u8 bc_symbol_table[128];
	u16 bc_nibbles;
	u16 hop_count;
	u8 hop_next_slot;
	u8 hop_index;
};

static bool realIR = false;

/*------------------------i2c function for 89-------------------------------------*/
int tcs3490_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;
	switch(i2c_flag){	
	case I2C_FLAG_WRITE:
	client->addr &=I2C_MASK_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	
	case I2C_FLAG_READ:
	client->addr &=I2C_MASK_FLAG;
	client->addr |=I2C_WR_FLAG;
	client->addr |=I2C_RS_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	default:
	//APS_LOG("APDS9930_i2c_master_operate i2c_flag command not support!\n");
	break;
	}
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	return res;
	EXIT_ERR:
	PK_ERR("TCS3490_i2c_transfer fail\n");
	return res;
}

static int tcs3490_i2c_read(struct tcs3490_chip *chip, u8 reg, u8 *val)
{
	u8 pdata[2] = {(u8)(reg+I2C_ADDR_OFFSET)};
	chip->client->addr = (I2C_SLAVE_ADDRESS)>>1;
	tcs3490_i2c_master_operate(chip->client, pdata, 0x101, I2C_FLAG_READ);
	*val = pdata[0];
	PK_DBG("reg = 0x%x, val = 0x%x\n", reg+I2C_ADDR_OFFSET, pdata[0]);

    return 0;
}

static int tcs3490_i2c_write(struct tcs3490_chip *chip, u8 reg, u8 val)
{
    int res = 0;
    u8 puSendCmd[2] = {	(u8)(reg+I2C_ADDR_OFFSET), 
                       			(u8)( val &0xFF)};
	
    chip->client->addr = (I2C_SLAVE_ADDRESS)>>1;
    res = tcs3490_i2c_master_operate(chip->client, puSendCmd, 0x2, I2C_FLAG_WRITE);
    if (res<=0)
    {
        PK_ERR("I2C write failed!! \n");
        return -1;
    }
	
    //PK_DBG("I2C write addr 0x%x data 0x%x\n", addr, data );
	
    return 0;
}

static int tcs3490_i2c_reg_blk_write(struct tcs3490_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;
	reg = reg+I2C_ADDR_OFFSET;
	ret =  i2c_smbus_write_i2c_block_data(client,
			reg, size, val);
	if (ret < 0) {
		PK_ERR("%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}
#if 0

static int tcs3490_i2c_ram_blk_write(struct tcs3490_chip *chip,
		u8 reg, u8 *val, int size)
{
	s32 ret;
	struct i2c_client *client = chip->client;
	reg = reg+I2C_ADDR_OFFSET;
	ret =  i2c_smbus_write_i2c_block_data(client,
			reg, size, val);
	if (ret < 0)
		PK_ERR("%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);
	return ret;
}
#endif

static int tcs3490_flush_regs(struct tcs3490_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	PK_DBG("%s\n", __func__);


	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = tcs3490_i2c_write(chip, reg, chip->shadow[reg]);
		if (rc) {
			PK_ERR("%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}


	return rc;
}

static int tcs3490_update_enable_reg(struct tcs3490_chip *chip)
{
	return	tcs3490_i2c_write(chip, TCS3490_CONTROL,
			chip->shadow[TCS3490_CONTROL]);
}


static void tcs3490_calc_cpl(struct tcs3490_chip *chip)
{
	u32 cpl;
	u32 sat;
	u8 atime = chip->shadow[TCS3490_ALS_TIME];

	cpl = 256 - chip->shadow[TCS3490_ALS_TIME];
	cpl *= INTEGRATION_CYCLE;
	cpl /= 100;
	cpl *= als_gains[chip->params.als_gain];

	sat = min_t(u32, MAX_ALS_VALUE, (u32)(256 - atime) << 10);
	sat = sat * 8 / 10;
	chip->als_inf.cpl = cpl;
	chip->als_inf.saturation = sat;
}

static int tcs3490_set_als_gain(struct tcs3490_chip *chip, int gain)
{
	int rc;
	u8 ctrl_reg  = chip->shadow[TCS3490_GAIN] & ~TCS3490_ALS_GAIN_MASK;

	switch (gain) {
	case 1:
		ctrl_reg |= AGAIN_1;
		break;
	case 4:
		ctrl_reg |= AGAIN_4;
		break;
	case 16:
		ctrl_reg |= AGAIN_16;
		break;
	case 64:
		ctrl_reg |= AGAIN_64;
		break;
	default:
		PK_ERR("%s: wrong als gain %d\n",
				__func__, gain);
		return -EINVAL;
	}

	rc = tcs3490_i2c_write(chip, TCS3490_GAIN, ctrl_reg);
	if (!rc) {
		chip->shadow[TCS3490_GAIN] = ctrl_reg;
		chip->params.als_gain = ctrl_reg & TCS3490_ALS_GAIN_MASK;
		PK_DBG("%s: new als gain %d\n",
				__func__, gain);
	}
	return rc;
}

static int tcs3490_set_segment_table(struct tcs3490_chip *chip,
		struct lux_segment *segment, int seg_num)
{
	int i;
	struct device *dev = &chip->client->dev;

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);

	if (!chip->segment) {
		PK_DBG("%s: allocating segment table\n", __func__);
		chip->segment = kzalloc(sizeof(*chip->segment) *
				chip->seg_num_max, GFP_KERNEL);
		if (!chip->segment) {
			PK_ERR("%s: no memory!\n", __func__);
			return -ENOMEM;
		}
	}
	if (seg_num > chip->seg_num_max) {
		dev_warn(dev, "%s: %d segment requested, %d applied\n",
				__func__, seg_num, chip->seg_num_max);
		chip->segment_num = chip->seg_num_max;
	} else {
		chip->segment_num = seg_num;
	}
	memcpy(chip->segment, segment,
			chip->segment_num * sizeof(*chip->segment));
	PK_DBG("%s: %d segment requested, %d applied\n", __func__,
			seg_num, chip->seg_num_max);
	for (i = 0; i < chip->segment_num; i++)
		PK_DBG("seg %d: d_factor %d, r_coef %d, g_coef %d, b_coef %d, ct_coef %d ct_offset %d\n",
		i, chip->segment[i].d_factor, chip->segment[i].r_coef,
		chip->segment[i].g_coef, chip->segment[i].b_coef,
		chip->segment[i].ct_coef, chip->segment[i].ct_offset);
	return 0;
}


static int tcs3490_set_param(struct tcs3490_chip *chip, unsigned char flag)
{
	if (flag) {
		chip->segment[chip->device_index].d_factor = D_Factor;
		chip->segment[chip->device_index].r_coef = R_Coef;
		chip->segment[chip->device_index].g_coef = G_Coef;
		chip->segment[chip->device_index].b_coef = B_Coef;
		chip->segment[chip->device_index].ct_coef = CT_Coef;
		chip->segment[chip->device_index].ct_offset = CT_Offset;

	} else {
		chip->segment[chip->device_index].d_factor = D_Factor1;
		chip->segment[chip->device_index].r_coef = R_Coef1;
		chip->segment[chip->device_index].g_coef = G_Coef1;
		chip->segment[chip->device_index].b_coef = B_Coef1;
		chip->segment[chip->device_index].ct_coef = CT_Coef1;
		chip->segment[chip->device_index].ct_offset = CT_Offset1;
	}
	return 0;
}

static int tcs3490_get_lux(struct tcs3490_chip *chip)
{
	u32 rp1,bp1;
	u32 lux = 0;
	u32 cct;
	int ret;
	u32 sat = chip->als_inf.saturation;
	u32 sf;

	/* use time in ms get scaling factor */
	tcs3490_calc_cpl(chip);

	if (!chip->als_gain_auto) {
		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) {
			PK_DBG("%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			PK_DBG("%s: saturation, keep lux & cct\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	} else {
		u8 gain = als_gains[chip->params.als_gain];
		int rc = -EIO;

		if (gain == 16 && chip->als_inf.clear_raw >= sat) {
				rc = tcs3490_set_als_gain(chip, 1);
		} else if (gain == 16 && chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL) {
				rc = tcs3490_set_als_gain(chip, 64);
		} else if ((gain == 64 && chip->als_inf.clear_raw >= (sat - GAIN_SWITCH_LEVEL)) ||
			(gain == 1 && chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL)) {
				rc = tcs3490_set_als_gain(chip, 16);
		}
		if (!rc) {
			PK_DBG("%s: gain adjusted, skip\n",
					__func__);
			tcs3490_calc_cpl(chip);
			ret = -EAGAIN;
			lux = chip->als_inf.lux;
			goto exit;
		}

		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) {
			PK_DBG("%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			PK_DBG("%s: saturation, keep lux\n",
					__func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	}

	if (5*chip->als_inf.realIR_raw > chip->als_inf.clear_raw) {//FL/LED
		printk(KERN_INFO ">=0.2: IR/Clear=%d\n",
				chip->als_inf.realIR_raw*100/chip->als_inf.clear_raw);
		tcs3490_set_param(chip, 1);
	} else {//Solar/Inc
		printk(KERN_INFO "<0.2: IR/Clear=%d\n",
				chip->als_inf.realIR_raw*100/chip->als_inf.clear_raw);
		tcs3490_set_param(chip, 0);
	}

	if (!chip->als_inf.cpl) {
		PK_DBG("%s: zero cpl. Setting to 1\n",
				__func__);
		chip->als_inf.cpl = 1;
	}

	lux = chip->segment[chip->device_index].c_coef * chip->als_inf.clear_raw;
	lux -= chip->segment[chip->device_index].r_coef * chip->als_inf.red_raw;
	lux += chip->segment[chip->device_index].g_coef * chip->als_inf.green_raw;
	lux -= chip->segment[chip->device_index].b_coef * chip->als_inf.blue_raw;

	sf = chip->als_inf.cpl;

	if (sf > 131072)
		goto error;

	lux /= sf;
	lux *= chip->segment[chip->device_index].d_factor;
	lux >>= 10;
	chip->als_inf.lux = (u16) lux;

	/* remove ir from counts*/
	rp1 = chip->als_inf.red_raw - chip->als_inf.ir;
	bp1 = chip->als_inf.blue_raw - chip->als_inf.ir;

	if ((bp1 > 0) && (rp1 > 0)){
		cct = ((chip->segment[chip->device_index].ct_coef * bp1) / rp1) +
		chip->segment[chip->device_index].ct_offset;
		chip->als_inf.cct = (u16) cct;
	}else{
		chip->als_inf.cct = 0;
		PK_ERR("%s: cct error bp1 = %d,rp1 = %d\n",__func__,bp1,rp1);
	}

exit:
return 0;

error:
	PK_ERR("ERROR Scale factor = %d", sf);

return 1;

}



static int tcs3490_irq_clr(struct tcs3490_chip *chip, u8 int2clr)
{
	int ret, ret2;

	ret = i2c_smbus_write_byte(chip->client, int2clr);
	if (ret < 0) {
		mdelay(3);
		ret2 = i2c_smbus_write_byte(chip->client, int2clr);
		if (ret2 < 0) {
			PK_ERR("%s: failed 2x, int to clr %02x\n",
					__func__, int2clr);
		}
		return ret2;
	}

	return ret;
}

static void tcs3490_get_als(struct tcs3490_chip *chip)
{
	u8 *buf = &chip->shadow[TCS3490_CLR_CHANLO];

	/* extract raw channel data */
	if (realIR == true)
	{
		chip->als_inf.realIR_raw = le16_to_cpup((const __le16 *)&buf[0]);
	}
	else
	{
		chip->als_inf.clear_raw = le16_to_cpup((const __le16 *)&buf[0]);
		chip->als_inf.red_raw = le16_to_cpup((const __le16 *)&buf[2]);
		chip->als_inf.green_raw = le16_to_cpup((const __le16 *)&buf[4]);
		chip->als_inf.blue_raw = le16_to_cpup((const __le16 *)&buf[6]);
		chip->als_inf.ir =
			(chip->als_inf.red_raw + chip->als_inf.green_raw +
			chip->als_inf.blue_raw - chip->als_inf.clear_raw + 1) >> 1;
		if (chip->als_inf.ir < 0)
			chip->als_inf.ir = 0;
	}
}

static int tcs3490_read_all(struct tcs3490_chip *chip)
{
	int ret = 0;

	tcs3490_i2c_read(chip, TCS3490_STATUS,
			&chip->shadow[TCS3490_STATUS]);

	tcs3490_i2c_read(chip, TCS3490_CLR_CHANLO,
			&chip->shadow[TCS3490_CLR_CHANLO]);
	tcs3490_i2c_read(chip, TCS3490_CLR_CHANHI,
			&chip->shadow[TCS3490_CLR_CHANHI]);

	tcs3490_i2c_read(chip, TCS3490_RED_CHANLO,
			&chip->shadow[TCS3490_RED_CHANLO]);
	tcs3490_i2c_read(chip, TCS3490_RED_CHANHI,
			&chip->shadow[TCS3490_RED_CHANHI]);

	tcs3490_i2c_read(chip, TCS3490_GRN_CHANLO,
			&chip->shadow[TCS3490_GRN_CHANLO]);
	tcs3490_i2c_read(chip, TCS3490_GRN_CHANHI,
			&chip->shadow[TCS3490_GRN_CHANHI]);

	tcs3490_i2c_read(chip, TCS3490_BLU_CHANLO,
			&chip->shadow[TCS3490_BLU_CHANLO]);
	ret = tcs3490_i2c_read(chip, TCS3490_BLU_CHANHI,
			&chip->shadow[TCS3490_BLU_CHANHI]);

	return (ret < 0) ? ret : 0;
}

static int tcs3490_update_als_thres(struct tcs3490_chip *chip, bool on_enable)
{
	s32 ret;
	u8 *buf = &chip->shadow[TCS3490_ALS_MINTHRESHLO];
	u16 deltaP = chip->params.als_deltaP;
	u16 from, to, cur;
	u16 saturation = chip->als_inf.saturation;

	cur = chip->als_inf.clear_raw;

	if (on_enable) {
		/* move deltaP far away from current position to force an irq */
		from = to = cur > saturation / 2 ? 0 : saturation;
	} else {
		deltaP = cur * deltaP / 100;
		if (!deltaP)
			deltaP = 1;

		if (cur > deltaP)
			from = cur - deltaP;
		else
			from = 0;

		if (cur < (saturation - deltaP))
			to = cur + deltaP;
		else
			to = saturation;

	}

	*buf++ = from & 0xff;
	*buf++ = from >> 8;
	*buf++ = to & 0xff;
	*buf++ = to >> 8;
	ret = tcs3490_i2c_reg_blk_write(chip, TCS3490_ALS_MINTHRESHLO,
			&chip->shadow[TCS3490_ALS_MINTHRESHLO],
			TCS3490_ALS_MAXTHRESHHI - TCS3490_ALS_MINTHRESHLO + 1);

	return (ret < 0) ? ret : 0;
}

#if 0
static void tcs3490_report_als(struct tcs3490_chip *chip)
{
	if (chip->a_idev) {
		int rc = tcs3490_get_lux(chip);
		if (!rc) {
			int lux = chip->als_inf.lux;
			input_report_abs(chip->a_idev, ABS_MISC, lux);
			input_sync(chip->a_idev);
			tcs3490_update_als_thres(chip, 0);
		} else {
			tcs3490_update_als_thres(chip, 1);
		}
	}
}

static int tcs3490_check_and_report(struct tcs3490_chip *chip)
{
	u8 status;
	u8 saturation;

	int ret = tcs3490_read_all(chip);
	if (ret)
		goto exit_clr;

	status = chip->shadow[TCS3490_STATUS];

	saturation = chip->als_inf.saturation;
	
	if ((status & (TCS3490_ST_ALS_VALID | TCS3490_ST_ALS_IRQ)) ==
			(TCS3490_ST_ALS_VALID | TCS3490_ST_ALS_IRQ)) {
		if (realIR == false)
		{
			tcs3490_get_als(chip);
			realIR = true;
			tcs3490_i2c_write(chip, 0x40, 0x80);
			tcs3490_report_als(chip);
		}
		else
		{
			tcs3490_get_als(chip);
			realIR = false;
			tcs3490_i2c_write(chip, 0x40, 0x00);
		}
		tcs3490_irq_clr(chip, TCS3490_CMD_ALS_INT_CLR);
	}

exit_clr:
	tcs3490_irq_clr(chip, TCS3490_CMD_ALL_INT_CLR);

	return ret;
}


static irqreturn_t tcs3490_irq(int irq, void *handle)
{
	struct tcs3490_chip *chip = handle;

	mutex_lock(&chip->lock);
	if (chip->in_suspend) {
		PK_DBG("%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		disable_irq_nosync(chip->client->irq);
		goto bypass;
	}
	(void)tcs3490_check_and_report(chip);
bypass:
	mutex_unlock(&chip->lock);
	return IRQ_HANDLED;
}
#endif

static void tcs3490_set_defaults(struct tcs3490_chip *chip)
{
	u8 *sh = chip->shadow;

	if (chip->pdata) {
		PK_DBG("%s: Loading pltform data\n", __func__);
		chip->params.als_time = chip->pdata->parameters.als_time;
		chip->params.als_deltaP = chip->pdata->parameters.als_deltaP;
	
		chip->params.persist = chip->pdata->parameters.persist;
		
		chip->params.als_gain = chip->pdata->parameters.als_gain;
	
	} else {
		PK_DBG("%s: use defaults\n", __func__);
		sh[TCS3490_ALS_TIME] = 0xEF; /* 47ms */
		sh[TCS3490_PERSISTENCE] = ALS_PERSIST(0);
		sh[TCS3490_PRX_PULSE_COUNT] = 8;
		sh[TCS3490_GAIN] = AGAIN_16;
		sh[TCS3490_WAIT_TIME] = 0xFF; /* 3ms */
	}

	chip->als_gain_auto = true;

	
	chip->shadow[TCS3490_PERSISTENCE]     = chip->params.persist;
	chip->shadow[TCS3490_ALS_TIME]        = chip->params.als_time;
	
	chip->shadow[TCS3490_GAIN]            = chip->params.als_gain; 
	chip->shadow[TCS3490_WAIT_TIME]        = chip->params.wait_time;
	
	tcs3490_flush_regs(chip);

}


static int tcs3490_als_enable(struct tcs3490_chip *chip, int on)
{
	int rc;

	PK_DBG("%s: on = %d\n", __func__, on);
	if (on) {
		realIR = false;
		tcs3490_irq_clr(chip, TCS3490_CMD_ALS_INT_CLR);
		tcs3490_update_als_thres(chip, 1);
		chip->shadow[TCS3490_CONTROL] |=
				(TCS3490_EN_PWR_ON | TCS3490_EN_ALS |
				TCS3490_EN_ALS_IRQ);

		rc = tcs3490_update_enable_reg(chip);
		if (rc)
			return rc;
		mdelay(3);
	} else {
		chip->shadow[TCS3490_CONTROL] &=
			~(TCS3490_EN_ALS_IRQ);

		if (!(chip->shadow[TCS3490_CONTROL] & TCS3490_EN_PRX))
			chip->shadow[TCS3490_CONTROL] &= ~TCS3490_EN_PWR_ON;
		rc = tcs3490_update_enable_reg(chip);
		if (rc)
			return rc;
		tcs3490_irq_clr(chip, TCS3490_CMD_ALS_INT_CLR);
	}
	if (!rc)
		chip->als_enabled = on;

	return rc;
}

int tcs3490_set_enable_rgb(int en)
{
	return tcs3490_als_enable(chip_func,en);
}

int tcs3490_get_enable_rgb(void)
{
	return chip_func->als_enabled;
}

int tcs3490_get_data_lux(u32 *value)
{
	tcs3490_read_all(chip_func);
	tcs3490_get_als(chip_func);
	tcs3490_get_lux(chip_func);
	*value = (u32)chip_func->als_inf.lux;
	return 0;
}
int tcs3490_get_data_r(u16 *value)
{
	*value = (u16)chip_func->als_inf.red_raw;
	return 0;
}
int tcs3490_get_data_g(u16 *value)
{
	*value = (u16)chip_func->als_inf.green_raw;
	return 0;
}
int tcs3490_get_data_b(u16 *value)
{
	*value = (u16)chip_func->als_inf.blue_raw;
	return 0;
}

int tcs3490_get_data_w(u16 *value)
{
	*value = (u16)chip_func->als_inf.clear_raw;
	return 0;
}

int tcs3490_get_data_cct(u32 *value)
{
	*value = (u32)chip_func->als_inf.cct;
	return 0;
}

int tcs3490_rgb_do_calib(void)
{
	return 0;
}

char* tcs3490_get_chipinfo(void)
{
	return "TCS3490";
}

static ssize_t tcs3490_device_als_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	tcs3490_read_all(chip);
	tcs3490_get_als(chip);
	tcs3490_get_lux(chip);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tcs3490_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	struct lux_segment *s = chip->segment;
	int i, k;

	for (i = k = 0; i < chip->segment_num; i++)
		k += snprintf(buf + k, PAGE_SIZE - k,
				"%d:%d,%d,%d,%d,%d,%d\n", i,
				s[i].d_factor,
				s[i].r_coef,
				s[i].g_coef,
				s[i].b_coef,
				s[i].ct_coef,
				s[i].ct_offset
				);
	return k;
}



static ssize_t tcs3490_lux_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int i;
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	u32 d_factor, r_coef, g_coef, b_coef, ct_coef, ct_offset;

	if (7 != sscanf(buf, "%10d:%10d,%10d,%10d,%10d,%10d,%10d",
		&i, &d_factor, &r_coef, &g_coef, &b_coef, &ct_coef, &ct_offset))
		return -EINVAL;
	if (i >= chip->segment_num)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->segment[i].d_factor = d_factor;
	chip->segment[i].r_coef = r_coef;
	chip->segment[i].g_coef = g_coef;
	chip->segment[i].b_coef = b_coef;
	chip->segment[i].ct_coef = ct_coef;
	chip->segment[i].ct_offset = ct_offset;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tcs3490_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tcs3490_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		tcs3490_als_enable(chip, 1);
	else
		tcs3490_als_enable(chip, 0);

	return size;
}



static ssize_t tcs3490_auto_gain_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
				chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tcs3490_auto_gain_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		chip->als_gain_auto = true;
	else
		chip->als_gain_auto = false;

	return size;
}

static ssize_t tcs3490_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[chip->params.als_gain],
			chip->als_gain_auto ? "auto" : "manual");
}



static ssize_t tcs3490_als_red_show(struct device *dev,
	struct device_attribute *attr, char *buf)
		{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.red_raw);
}

static ssize_t tcs3490_als_green_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.green_raw);
}

static ssize_t tcs3490_als_blue_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.blue_raw);
}

static ssize_t tcs3490_als_clear_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.clear_raw);
}

static ssize_t tcs3490_als_cct_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	tcs3490_read_all(chip);
	tcs3490_get_als(chip);
	tcs3490_get_lux(chip);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
}

static ssize_t tcs3490_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gain;
	int i = 0;
	int rc;
	struct tcs3490_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &gain);

	if (rc)
		return -EINVAL;
	if (gain != 0 && gain != 1 && gain != 4 && gain != 16 && gain != 64)
		return -EINVAL;

	while (i < sizeof(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (i > 3) {
		PK_ERR("%s: wrong als gain %d\n",
				__func__, (int)gain);
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	if (gain) {
		chip->als_gain_auto = false;
		rc = tcs3490_set_als_gain(chip, als_gains[i]);
		if (!rc)
			tcs3490_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	tcs3490_flush_regs(chip);
	mutex_unlock(&chip->lock);
	return rc ? rc : size;
}



static ssize_t tcs3490_als_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(((chip->shadow[TCS3490_PERSISTENCE]) & 0x0f)));
}

static ssize_t tcs3490_als_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	long persist;
	int rc;
	struct tcs3490_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &persist);
	if (rc)
		return -EINVAL;

	mutex_lock(&chip->lock);
	chip->shadow[TCS3490_PERSISTENCE] &= 0xF0;
	chip->shadow[TCS3490_PERSISTENCE] |= ((u8)persist & 0x0F);

	tcs3490_flush_regs(chip);

	mutex_unlock(&chip->lock);
	return size;
}


static ssize_t tcs3490_als_itime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->shadow[TCS3490_ALS_TIME];
	t *= INTEGRATION_CYCLE;
	t /= 100;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tcs3490_als_itime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	long itime;
	int rc;
	struct tcs3490_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &itime);
	if (rc)
		return -EINVAL;
	itime *= 100;
	itime /= INTEGRATION_CYCLE;
	itime = (256 - itime);
	mutex_lock(&chip->lock);
	chip->shadow[TCS3490_ALS_TIME] = (u8)itime;
	tcs3490_flush_regs(chip);

	mutex_unlock(&chip->lock);
	return size;
}



static ssize_t tcs3490_als_deltaP_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tcs3490_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE,
			"%d (in %%)\n", chip->params.als_deltaP);
}

static ssize_t tcs3490_als_deltaP_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long deltaP;
	int rc;
	struct tcs3490_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &deltaP);
	if (rc || deltaP > 100)
		return -EINVAL;
	mutex_lock(&chip->lock);
	chip->params.als_deltaP = deltaP;
	mutex_unlock(&chip->lock);
	return size;
}

static struct device_attribute als_attrs[] = {
	__ATTR(als_Itime, 0666, tcs3490_als_itime_show, tcs3490_als_itime_store),
	__ATTR(als_lux, 0444, tcs3490_device_als_lux, NULL),
	__ATTR(als_red, 0444, tcs3490_als_red_show, NULL),
	__ATTR(als_green, 0444, tcs3490_als_green_show, NULL),
	__ATTR(als_blue, 0444, tcs3490_als_blue_show, NULL),
	__ATTR(als_clear, 0444, tcs3490_als_clear_show, NULL),
	__ATTR(als_cct, 0444, tcs3490_als_cct_show, NULL),
	__ATTR(als_gain, 0666, tcs3490_als_gain_show, tcs3490_als_gain_store),
	__ATTR(als_thresh_deltaP, 0666, tcs3490_als_deltaP_show, tcs3490_als_deltaP_store),
	__ATTR(als_auto_gain, 0666, tcs3490_auto_gain_enable_show, tcs3490_auto_gain_enable_store),
	__ATTR(lux_table, 0666, tcs3490_lux_table_show, tcs3490_lux_table_store),
	__ATTR(als_power_state, 0666, tcs3490_als_enable_show, tcs3490_als_enable_store),
	__ATTR(als_persist, 0666, tcs3490_als_persist_show, tcs3490_als_persist_store),

};

static int tcs3490_add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	PK_ERR("%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void tcs3490_remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int tcs3490_get_id(struct tcs3490_chip *chip, u8 *id, u8 *rev)
{
	tcs3490_i2c_read(chip, TCS3490_REVID, rev);
	return tcs3490_i2c_read(chip, TCS3490_CHIPID, id);
}

static int tcs3490_als_idev_open(struct input_dev *idev)
{
	struct tcs3490_chip *chip = dev_get_drvdata(&idev->dev);
	int rc;
	//bool prox = chip->p_idev && chip->p_idev->users;

	PK_DBG("%s\n", __func__);
	mutex_lock(&chip->lock);

	rc = tcs3490_als_enable(chip, 1);
	//if (rc && !prox)
	//	tcs3490_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
	return rc;
}

static void tcs3490_als_idev_close(struct input_dev *idev)
{
	struct tcs3490_chip *chip = dev_get_drvdata(&idev->dev);
	PK_DBG("%s\n", __func__);
	mutex_lock(&chip->lock);
	tcs3490_als_enable(chip, 0);

	//if (!chip->p_idev || !chip->p_idev->users)
	mutex_unlock(&chip->lock);
}

static int tcs3490_i2c_probe(struct i2c_client *client,	const struct i2c_device_id *idp);
static int tcs3490_i2c_remove(struct i2c_client *client);
static struct i2c_device_id tcs3490_idtable[] = {
	{ TCS3490_DRVNAME, 0 },
	{}
};

struct i2c_driver TCS3490_i2c_driver = {
	.probe = tcs3490_i2c_probe,
	.remove = tcs3490_i2c_remove,
	.driver.name = TCS3490_DRVNAME,
	.id_table = tcs3490_idtable,
};
static int tcs3490_i2c_remove(struct i2c_client *client)
{
	return 0;
}
static int tcs3490_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct tcs3490_chip *chip;
	struct tcs3490_i2c_platform_data *pdata = dev->platform_data;
	struct rgb_control_func ctl_func = {};


	PK_DBG("%s: client->irq = %d\n", __func__, client->irq);
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		PK_ERR("%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		PK_ERR("%s: platform data required\n", __func__);
		pdata = kzalloc(sizeof(struct tcs3490_i2c_platform_data), GFP_KERNEL);
	}
	pdata->als_name = "taos_als";
	pdata->parameters.persist = ALS_PERSIST(0);	
	pdata->parameters.als_deltaP = 10;
	pdata->parameters.als_time = 0x6B; /* 5.6ms */
	pdata->parameters.als_gain = AGAIN_16;
	pdata->als_can_wake = false;
	pdata->segment = (struct lux_segment *) tcs3490_segment;
	pdata->segment_num = ARRAY_SIZE(tcs3490_segment);
	chip = kzalloc(sizeof(struct tcs3490_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}
	chip->client = client;
	chip->pdata = pdata;
	chip_func =chip;
	i2c_set_clientdata(client, chip);

	chip->seg_num_max = chip->pdata->segment_num ? chip->pdata->segment_num : ARRAY_SIZE(segment_default);
	if (chip->pdata->segment)
		ret = tcs3490_set_segment_table(chip, chip->pdata->segment, chip->pdata->segment_num);
	else
		ret =  tcs3490_set_segment_table(chip, segment_default, ARRAY_SIZE(segment_default));
	if (ret)
		goto set_segment_failed;

	ret = tcs3490_get_id(chip, &id, &rev);

	PK_DBG("%s: device id:%02x device rev:%02x\n", __func__, id, rev);

	for (i = 0; i < ARRAY_SIZE(tcs3490_ids); i++) {
		if (id == tcs3490_ids[i])
			break;
	}
	if (i < ARRAY_SIZE(tcs3490_names)) {
		PK_DBG("%s: '%s rev. %d' detected\n", __func__, tcs3490_names[i], rev);
		chip->device_index = i;
	} else {
		PK_ERR("%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}

	mutex_init(&chip->lock);
	tcs3490_set_defaults(chip);
	ret = tcs3490_flush_regs(chip);
	if (ret)
		goto flush_regs_failed;

	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		PK_ERR("%s: no memory for input_dev '%s'\n",
				__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	chip->a_idev->open = tcs3490_als_idev_open;
	chip->a_idev->close = tcs3490_als_idev_close;
	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		goto input_a_alloc_failed;
	}
	ret = tcs3490_add_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;
bypass_als_idev:
/*
	ret = request_threaded_irq(client->irq, NULL, &tcs3490_irq,
		      IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		      dev_name(dev), chip);
	if (ret) {
		dev_info(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}*/
	chip->shadow[TCS3490_CONTROL] =
			(TCS3490_EN_PWR_ON | TCS3490_EN_ALS | TCS3490_EN_WAIT);
	chip->shadow[TCS3490_CONTROL] = 0x13;//hufeng for test

	tcs3490_update_enable_reg(chip);

	ctl_func.set_enable_rgb = tcs3490_set_enable_rgb;
	ctl_func.get_enable_rgb = tcs3490_get_enable_rgb;
	ctl_func.get_data_lux = tcs3490_get_data_lux;
	ctl_func.get_data_r = tcs3490_get_data_r;
	ctl_func.get_data_g = tcs3490_get_data_g;
	ctl_func.get_data_b = tcs3490_get_data_b;
	ctl_func.get_data_w = tcs3490_get_data_w;
	ctl_func.get_data_cct = tcs3490_get_data_cct;
	ctl_func.get_chipinfo = tcs3490_get_chipinfo;
	ctl_func.rgb_do_calib = tcs3490_rgb_do_calib;

	if(rgb_misc_init(&ctl_func)){
		printk("[rgb_sensor]misc device init fail\n");
	}

	PK_DBG("Probe ok.\n");
	return 0;

//irq_register_fail:
	if (chip->a_idev) {
		tcs3490_remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
	}

input_a_alloc_failed:
flush_regs_failed:
id_failed:
	kfree(chip->segment);
set_segment_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
malloc_failed:

	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	PK_ERR("Probe failed.\n");
	return ret;
}


static int TCS3490_platform_probe(struct platform_device *pdev)
{
	PK_DBG("TCS3490_platform_probe\n");
	return i2c_add_driver(&TCS3490_i2c_driver);
}

static int TCS3490_platform_remove(struct platform_device *pdev)
{
	i2c_del_driver(&TCS3490_i2c_driver);
	return 0;
}

static int TCS3490_platform_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int TCS3490_platform_resume(struct platform_device *pdev)
{
	return 0;
}
/* platform structure */

static struct platform_device g_TCS3490_device = {
	.name = PLATFORM_DRIVER_NAME,
	.id = 0,
	.dev = {}
};
static struct platform_driver g_TCS3490_Driver = {
	.probe = TCS3490_platform_probe,
	.remove = TCS3490_platform_remove,
	.suspend = TCS3490_platform_suspend,
	.resume = TCS3490_platform_resume,
	.driver = {
		   .name = PLATFORM_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   }
};
static int __init tcs3490_init(void)
{
	PK_DBG("tcs3490_init enter\n");
	i2c_register_board_info(I2C_BUSNUM, &tcs3490_dev, 1);
	PK_DBG("after tcs3490 board infro register");
	if (platform_device_register(&g_TCS3490_device)) {
		PK_ERR("failed to register tcs3490 device\n");
		return -ENODEV;
	}
	if (platform_driver_register(&g_TCS3490_Driver)) {
		PK_ERR("failed to register tcs3490 driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit tcs3490_exit(void)
{
	platform_driver_unregister(&g_TCS3490_Driver);
}

module_init(tcs3490_init);
module_exit(tcs3490_exit);

MODULE_AUTHOR("Byron Shi<byron.shi@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tcs3490 ambient, proximity, Beam sensor driver");
MODULE_LICENSE("GPL");
