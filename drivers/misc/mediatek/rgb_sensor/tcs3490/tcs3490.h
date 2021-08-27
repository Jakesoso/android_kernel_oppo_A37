/*
 * Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), and Beam functionality within the
 * AMS-TAOS TCS3490 family of devices.
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

#ifndef __TCS3490_H
#define __TCS3490_H

#include <linux/types.h>

/* Max number of segments allowable in LUX table */
#define TCS3490_MAX_LUX_TABLE_SIZE		9
#define MAX_DEFAULT_TABLE_BYTES (sizeof(int) * TCS3490_MAX_LUX_TABLE_SIZE)

/* Default LUX and Color coefficients */

#define D_Factor	1127
#define C_Coef		154
#define R_Coef		236
#define G_Coef		1341
#define B_Coef		1126
#define CT_Coef		(4588)
#define CT_Offset	(1211)

#define D_Factor1	1127
#define C_Coef1		154
#define R_Coef1		236
#define G_Coef1		1341
#define B_Coef1		1126
#define CT_Coef1	(4588)
#define CT_Offset1	(1211)

struct device;

enum tcs3490_pwr_state {
	TCS3940_POWER_ON,
	TCS3490_POWER_OFF,
	TCS3490_POWER_STANDBY,
};

enum tcs3490_ctrl_reg {
	AGAIN_1        = (0 << 0),
	AGAIN_4        = (1 << 0),
	AGAIN_16       = (2 << 0),
	AGAIN_64       = (3 << 0),
};

#define ALS_PERSIST(p) (((p) & 0xf) << 3)

struct tcs3490_parameters {
	u8 als_time;
	u16 als_deltaP;
	u8 als_gain;
	u8 persist;
	u8 wait_time;

};

struct lux_segment {
	int d_factor;
	int r_coef;
	int g_coef;
	int b_coef;
	int c_coef;
	int ct_coef;
	int ct_offset;
};

static const struct lux_segment tcs3490_segment[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.c_coef = C_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.c_coef = C_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};
struct tcs3490_i2c_platform_data {
	/* The following callback for power events received and handled by
	   the driver.  Currently only for SUSPEND and RESUME */
	int (*platform_power)(struct device *dev, enum tcs3490_pwr_state state);
	int (*platform_init)(void);
	void (*platform_teardown)(struct device *dev);
	char const *als_name;
	struct tcs3490_parameters parameters;
	bool als_can_wake;
	struct lux_segment *segment;
	int segment_num;
	struct work_struct		report_als;
};

#endif /* __TCS3490_H */
