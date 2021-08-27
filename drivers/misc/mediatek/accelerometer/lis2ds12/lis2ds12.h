/* lis2ds12.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef LIS2DS12_H
#define LIS2DS12_H
	 
#include <linux/ioctl.h>

extern struct acc_hw* lis2ds12_get_cust_acc_hw(void);

#define LIS2DS12_I2C_SLAVE_ADDR			0x3C
#define LIS2DS12_FIXED_DEVID			0x43

#define AUTO_INCREMENT 					0x80
#define LIS2DS12_BUFSIZE 				64

/*Acc control registers*/
#define LIS2DS12_WHO_AM_I 				0x0F
#define LIS2DS12_CTRL1 					0x20
#define LIS2DS12_CTRL2 					0x21
#define LIS2DS12_CTRL3 					0x22
#define LIS2DS12_CTRL4 					0x23
#define LIS2DS12_CTRL5 					0x24
#define LIS2DS12_FIFO_CTRL 				0x25
#define LIS2DS12_OUT_TEMP 				0x26
#define LIS2DS12_STATUS 				0x27
#define LIS2DS12_OUTX_L_XL 				0x28
#define LIS2DS12_OUTX_H_XL			 	0x29
#define LIS2DS12_OUTY_L_XL 				0x2A
#define LIS2DS12_OUTY_H_XL 				0x2B
#define LIS2DS12_OUTZ_L_XL 				0x2C
#define LIS2DS12_OUTZ_H_XL 				0x2D
#define LIS2DS12_FIFO_THS 				0x2E
#define LIS2DS12_FIFO_SRC 				0x2F
#define LIS2DS12_FIFO_SAMPLES 			0x30
#define LIS2DS12_TAP_6D_THS 			0x31
#define LIS2DS12_INT_DUR 				0x32
#define LIS2DS12_WAKE_UP_THS 			0x33
#define LIS2DS12_WAKE_UP_DUR 			0x34
#define LIS2DS12_FREE_FALL 				0x35
#define LIS2DS12_STATUS_DUP 			0x36
#define LIS2DS12_WAKE_UP_SRC 			0x37
#define LIS2DS12_TAP_SRC 				0x38
#define LIS2DS12_6D_SRC 				0x39
#define LIS2DS12_STEP_COUNTER_MINTHS 	0x3A
#define LIS2DS12_STEP_COUNTER_L 		0x3B
#define LIS2DS12_STEP_COUNTER_H 		0x3C
#define LIS2DS12_FUNC_CK_GATE	 		0x3D
#define LIS2DS12_FUNC_SRC 				0x3E
#define LIS2DS12_FUNC_CTRL 				0x3F

/*LIS2DS12 Register Bit definitions*/
#define LIS2DS12_ACC_RANGE_MASK 		0x0C
#define LIS2DS12_ACC_RANGE_2g		    0x00
#define LIS2DS12_ACC_RANGE_4g		    0x08
#define LIS2DS12_ACC_RANGE_8g		    0x0C
#define LIS2DS12_ACC_RANGE_16g		    0x04

#define LIS2DS12_ACC_ODR_MASK			0xF0
#define LIS2DS12_ACC_ODR_POWER_DOWN		0x00
#define LIS2DS12_ACC_ODR_12_5HZ		    0x10
#define LIS2DS12_ACC_ODR_25HZ		    0x20
#define LIS2DS12_ACC_ODR_50HZ		    0x30
#define LIS2DS12_ACC_ODR_100HZ		    0x40
#define LIS2DS12_ACC_ODR_200HZ		    0x50
#define LIS2DS12_ACC_ODR_400HZ		    0x60
#define LIS2DS12_ACC_ODR_800HZ		    0x70

#define LIS2DS12_ACC_SENSITIVITY_2G		244	 /*ug/LSB*/
#define LIS2DS12_ACC_SENSITIVITY_4G		488	 /*ug/LSB*/
#define LIS2DS12_ACC_SENSITIVITY_8G		976	 /*ug/LSB*/
#define LIS2DS12_ACC_SENSITIVITY_16G	1952 /*ug/LSB*/

#define LIS2DS12_CTRL2_IF_ADD_INC		0x04

#define LIS2DS12_FUNC_CK_GATE_TILT_INT_MASK		0x80
#define LIS2DS12_FUNC_CK_GATE_SIGN_M_DET_MASK	0x10
#define LIS2DS12_FUNC_CK_GATE_RST_SIGN_M_MASK	0x08
#define LIS2DS12_FUNC_CK_GATE_RST_PEDO_MASK		0x04
#define LIS2DS12_FUNC_CK_GATE_STEP_D_MASK		0x02
#define LIS2DS12_FUNC_CK_GATE_MASK			   (LIS2DS12_FUNC_CK_GATE_TILT_INT_MASK | \
												LIS2DS12_FUNC_CK_GATE_SIGN_M_DET_MASK | \
												LIS2DS12_FUNC_CK_GATE_STEP_D_MASK)

#define LIS2DS12_INT2_ON_INT1_MASK				0x20
#define LIS2DS12_INT2_TILT_MASK					0x10
#define LIS2DS12_INT2_SIG_MOT_DET_MASK			0x08
#define LIS2DS12_INT2_STEP_DET_MASK			0x24//	0x04
#define LIS2DS12_INT2_EVENTS_MASK			   (LIS2DS12_INT2_TILT_MASK | \
												LIS2DS12_INT2_SIG_MOT_DET_MASK | \
												LIS2DS12_INT2_STEP_DET_MASK)

#define LIS2DS12_FUNC_CTRL_TILT_MASK			0x10
#define LIS2DS12_FUNC_CTRL_SIGN_MOT_MASK		0x02
#define LIS2DS12_FUNC_CTRL_STEP_CNT_MASK		0x01
#define LIS2DS12_FUNC_CTRL_EV_MASK			   (LIS2DS12_FUNC_CTRL_TILT_MASK | \
												LIS2DS12_FUNC_CTRL_SIGN_MOT_MASK | \
												LIS2DS12_FUNC_CTRL_STEP_CNT_MASK)

#define LIS2DS12_STEP_CNT_4G_MASK 				0x40
#define LIS2DS12_STEP_CNT_THRESHOLD_MASK 		0x3F
#define LIS2DS12_ACCESS_EMBED_REG_MASK 			0x10
#define LIS2DS12_STEP_COUNTER_DEBOUNCE 			0x2B
#define LIS2DS12_STEP_CNT_DEBOUNCE_MASK 		0xFF
#define LIS2DS12_STEP_CNT_RST_MASK			0x80

#define LIS2DS12_SUCCESS		       			0
#define LIS2DS12_ERR_I2C		       		   -1
#define LIS2DS12_ERR_STATUS			   		   -3
#define LIS2DS12_ERR_SETUP_FAILURE	   		   -4
#define LIS2DS12_ERR_GETGSENSORDATA    		   -5
#define LIS2DS12_ERR_IDENTIFICATION	   		   -6

typedef enum {
  	LIS2DS12_ACC_INT_ACTIVE_HIGH =0x00,
  	LIS2DS12_ACC_INT_ACTIVE_LOW	=0x02,
} LIS2DS12_ACC_INT_ACTIVE_t;
#define LIS2DS12_ACC_INT_ACTIVE_MASK 0x02

typedef enum {
  	LIS2DS12_ACC_INT_LATCH =0x04,
  	LIS2DS12_ACC_INT_NO_LATCH =0x00,
} LIS2DS12_ACC_INT_LATCH_CTL_t;
#define LIS2DS12_ACC_INT_LATCH_CTL_MASK 0x04

/*------------------------------------------------------------------*/

#endif //LIS2DS12_H
