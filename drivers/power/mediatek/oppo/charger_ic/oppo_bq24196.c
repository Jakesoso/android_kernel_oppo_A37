/************************************************************************************
** File:  \\192.168.144.3\Linux_Share\12015\ics2\development\mediatek\custom\oppo77_12015\kernel\battery\battery
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      for dc-dc sn111008 charg
** 
** Version: 1.0
** Date created: 21:03:46,05/04/2012
** Author: Fanhong.Kong@ProDrv.CHG
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
************************************************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>

//#ifdef CONFIG_OPPO_CHARGER_MTK
#ifdef CONFIG_OPPO_CHARGER_MTK


#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_boot.h>
#include <mach/mtk_rtc.h>
#include <soc/oppo/device_info.h>

extern void mt_power_off(void); 
#else

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

#include <mach/oppo_boot_mode.h>
#include <soc/oppo/device_info.h>

void (*enable_aggressive_segmentation_fn)(bool);

#endif

#include "../oppo_vooc.h"
#include "../oppo_gauge.h"
#include "oppo_bq24196.h"
#include "oppo_bq2202a.h"

static struct oppo_chg_chip *the_chip = NULL;
static int aicl_result = 500;

static DEFINE_MUTEX(bq24196_i2c_access);

static int __bq24196_read_reg(struct oppo_chg_chip *chip, int reg, int *returnData)
{
//#ifdef CONFIG_OPPO_CHARGER_MTK
#ifdef 	CONFIG_OPPO_CHARGER_MTK
	char cmd_buf[1]={0x00};
    char readData = 0;
    int ret = 0;

    //chip->client->addr = ((chip->client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    chip->client->ext_flag=((chip->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
	chip->client->timing = 300;
    cmd_buf[0] = reg;
    ret = i2c_master_send(chip->client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //chip->client->addr = chip->client->addr & I2C_MASK_FLAG;
        chip->client->ext_flag=0;
        return ret;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    // chip->client->addr = chip->client->addr & I2C_MASK_FLAG;
    chip->client->ext_flag=0;

#else
	int ret;	

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		chg_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*returnData = ret;
	}
#endif

	return 0;
}

static int bq24196_read_reg(struct oppo_chg_chip *chip, int reg, int *returnData)
{
	int ret = 0;

	mutex_lock(&bq24196_i2c_access);
	ret = __bq24196_read_reg(chip, reg, returnData);
	mutex_unlock(&bq24196_i2c_access);
	return ret;
}

static int __bq24196_write_reg(struct oppo_chg_chip *chip, int reg, int val)
{
//#ifdef CONFIG_OPPO_CHARGER_MTK
#ifdef CONFIG_OPPO_CHARGER_MTK

	char    write_data[2] = {0};
    int     ret=0;
    
    write_data[0] = reg;
    write_data[1] = val;
    
    chip->client->ext_flag=((chip->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    chip->client->timing = 300;
    ret = i2c_master_send(chip->client, write_data, 2);
    if (ret < 0) 
    {
       
        chip->client->ext_flag=0;
        return ret;
    }
    
    chip->client->ext_flag=0;

#else
	int ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		chg_err("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
#endif

	return 0;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
/*
static int bq24196_read_interface (int RegNum, int *val, int MASK, int SHIFT)
{
    int bq24196_reg = 0;
    int ret = 0;

   //chg_err("--------------------------------------------------\n");
	
    ret = bq24196_read_reg(RegNum, &bq24196_reg);
	
   //chg_err(" Reg[%x]=0x%x\n", RegNum, bq24196_reg);
	
    bq24196_reg &= (MASK << SHIFT);
    *val = (bq24196_reg >> SHIFT);
	
   //chg_err(" val=0x%x\n", *val);
	
    return ret;
}
*/

static int bq24196_config_interface (struct oppo_chg_chip *chip, int RegNum, int val, int MASK)
{
    int bq24196_reg = 0;
    int ret = 0;

	mutex_lock(&bq24196_i2c_access);
    ret = __bq24196_read_reg(chip, RegNum, &bq24196_reg);

    //chg_err(" Reg[%x]=0x%x\n", RegNum, bq24196_reg);
    
    bq24196_reg &= ~MASK;
    bq24196_reg |= val;

    ret = __bq24196_write_reg(chip, RegNum, bq24196_reg);

    //chg_err(" write Reg[%x]=0x%x\n", RegNum, bq24196_reg);

    __bq24196_read_reg(chip, RegNum, &bq24196_reg);

    //chg_err(" Check Reg[%x]=0x%x\n", RegNum, bq24196_reg);
	mutex_unlock(&bq24196_i2c_access);
	
    return ret;
}


//write one register directly
#if 0
int bq24196_reg_config_interface (struct oppo_chg_chip *chip, int RegNum, int val)
{   
    int ret = 0;

	mutex_lock(&bq24196_i2c_access);
    ret = __bq24196_write_reg(chip, RegNum, val);
	mutex_unlock(&bq24196_i2c_access);
	
    return ret;
}
#endif

static int bq24196_usbin_input_current_limit[] = {
    100,    150,    500,    900,
    1200,   1500,   2000,   3000,
};

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static int bq24196_input_current_limit_write(struct oppo_chg_chip *chip, int value)
{
	int rc = 0, i = 0, j = 0;
	int chg_vol = 0;
	int aicl_point_temp = 0;

	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	chg_debug( "usb input max current limit=%d setting %02x\n", value, i);

	aicl_point_temp = chip->sw_aicl_point;
/*
	for (j = 2; j < ARRAY_SIZE(bq24196_usbin_input_current_limit); j++) {
		if (j == 4) //We DO NOT use 1.2A here
			continue;
		else if (j == 5)
			aicl_point_temp = chip->sw_aicl_point - 30;
		else if (j == 6)
			aicl_point_temp = chip->sw_aicl_point - 50;
		bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
		msleep(90);
		chg_vol = battery_meter_get_charger_voltage();
		if (chg_vol < aicl_point_temp) {
			if (j > 2)
				j = j - 1;
		}
		if (bq24196_usbin_input_current_limit[j] >= 3000 || value < bq24196_usbin_input_current_limit[j + 1]) {
			goto aicl_end;
		}
	}
*/
	if (value < 150) {
		j = 0;
		goto aicl_end;
	} else if (value < 500) {
		j = 1;
		goto aicl_end;
	}

	j = 2; /* 500 */
	rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = 2;
		goto aicl_pre_step;
	} else if (value < 900)
		goto aicl_end;

	j = 3; /* 900 */
	rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value < 1200)
		goto aicl_end;

	j = 5; /* 1500 */
	aicl_point_temp = chip->sw_aicl_point + 55;
	rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 2; //We DO NOT use 1.2A here
		goto aicl_pre_step;
	} else if (value < 1500) {
		j = j - 1; //We use 1.2A here
		goto aicl_end;
	} else if (value < 2000)
		goto aicl_end;

	j = 6; /* 2000 */
	aicl_point_temp = chip->sw_aicl_point - 30;
	rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value < 3000)
		goto aicl_end;

	j = 7; /* 3000 */
	rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point_temp) {
		j = j - 1;
		goto aicl_pre_step;
	} else if (value >= 3000)
		goto aicl_end;

aicl_pre_step:
	if((j >= 2) && (j <= ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1))
		aicl_result = bq24196_usbin_input_current_limit[j];		
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, j, bq24196_usbin_input_current_limit[j], aicl_point_temp);
	rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	return rc;
aicl_end:
if((j >= 2) && (j <= ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1))
		aicl_result = bq24196_usbin_input_current_limit[j];		
	chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, j, bq24196_usbin_input_current_limit[j], aicl_point_temp);
	rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
	return rc;
}
#else
static int bq24196_input_current_limit_write(struct oppo_chg_chip *chip, int value)
{
	int rc = 0,i = 0,j = 0;
	int chg_vol = 0;
	
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	
	for (i = ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1; i >= 0; i--) 
	{
		if (bq24196_usbin_input_current_limit[i] <= value) {
			break;
		}
		else if (i == 0) {
		    break;
		}
	}
    chg_debug( "usb input max current limit=%d setting %02x\n", value, i);
    for(j = 2; j <= i; j++) {
        bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j<<REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
#ifdef CONFIG_OPPO_CHARGER_MTK
		msleep(90);
		chg_vol = battery_meter_get_charger_voltage();
#else
		msleep(90);
		chg_vol = qpnp_get_prop_charger_voltage_now();
#endif	
        //chg_err("usb input max current limit aicl chg_vol=%d j=%d\n", chg_vol, j);
        if(chg_vol < chip->sw_aicl_point) {
            if (j > 2) {
                j = j-1;
            }
			if((j >= 2) && (j <= ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1))
				aicl_result = bq24196_usbin_input_current_limit[j];		
            chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d sw_aicl_point:%d\n", chg_vol, j, bq24196_usbin_input_current_limit[j], chip->sw_aicl_point);
            bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
            return 0;
        }
    }
    j = i;

	if((j >= 2) && (j <= ARRAY_SIZE(bq24196_usbin_input_current_limit) - 1))
		aicl_result = bq24196_usbin_input_current_limit[j];		
    chg_debug( "usb input max current limit aicl chg_vol=%d j[%d]=%d\n", chg_vol, j, bq24196_usbin_input_current_limit[j]);
    rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, j << REG00_BQ24196_INPUT_CURRENT_LIMIT_SHIFT, REG00_BQ24196_INPUT_CURRENT_LIMIT_MASK);
    return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/


static int bq24196_charging_current_write_fast(struct oppo_chg_chip *chip, int chg_cur)
{	
	int value, ret = 0;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	chg_debug( " chg_cur = %d\r\n", chg_cur);
	if (chg_cur < BQ24196_MIN_FAST_CURRENT_MA_ALLOWED) {
		if (chg_cur > BQ24196_MIN_FAST_CURRENT_MA_20_PERCENT)
			chg_cur = BQ24196_MIN_FAST_CURRENT_MA_20_PERCENT;
	    chg_cur = chg_cur * 5;
	    value = (chg_cur - BQ24196_MIN_FAST_CURRENT_MA)/BQ24196_FAST_CURRENT_STEP_MA;
	    value <<= REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_SHIFT;
	    value = value | REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_ENABLE;
	} else {
	    value = (chg_cur - BQ24196_MIN_FAST_CURRENT_MA)/BQ24196_FAST_CURRENT_STEP_MA;
	    value <<= REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_SHIFT;
	}
	ret = bq24196_config_interface(chip, REG02_BQ24196_ADDRESS, value, 
		REG02_BQ24196_FAST_CHARGING_CURRENT_LIMIT_MASK | REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_MASK);
	return ret;
}

static int bq24196_set_vindpm_vol(struct oppo_chg_chip *chip, int vol)
{
	int rc;
	int value = 0;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	value = (vol - REG00_BQ24196_VINDPM_OFFSET) / REG00_BQ24196_VINDPM_STEP_MV;
	value <<= REG00_BQ24196_VINDPM_SHIFT;
	
    rc = bq24196_config_interface(chip,
		REG00_BQ24196_ADDRESS, value, REG00_BQ24196_VINDPM_MASK);
	return rc;
}

#ifdef CONFIG_OPPO_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int oppo_chg_get_dyna_aicl_result(struct oppo_chg_chip *chip)
{
	return aicl_result;
	
}
#endif /* CONFIG_OPPO_SHORT_C_BATT_CHECK */

static void bq24196_set_aicl_point(struct oppo_chg_chip *chip, int vbatt)
{
	if(chip->hw_aicl_point == 4440 && vbatt > 4140) {
		chip->hw_aicl_point = 4520;
		chip->sw_aicl_point = 4535;
		bq24196_set_vindpm_vol(chip, chip->hw_aicl_point);
	} else if(chip->hw_aicl_point == 4520 && vbatt < 4000) {
		chip->hw_aicl_point = 4440;
		chip->sw_aicl_point = 4500;
		bq24196_set_vindpm_vol(chip, chip->hw_aicl_point);
	}
}

static int bq24196_set_enable_volatile_writes(struct oppo_chg_chip *chip)
{
    int rc = 0;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	//need do nothing
	
    return rc;
}

static int bq24196_set_complete_charge_timeout(struct oppo_chg_chip * chip, int val)
{
    int rc = 0;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    if (val == OVERTIME_AC){
        val = REG05_BQ24196_CHARGING_SAFETY_TIME_ENABLE | REG05_BQ24196_FAST_CHARGING_TIMEOUT_8H;
    } else if (val == OVERTIME_USB){
        val = REG05_BQ24196_CHARGING_SAFETY_TIME_ENABLE | REG05_BQ24196_FAST_CHARGING_TIMEOUT_12H;
    } else {
        val = REG05_BQ24196_CHARGING_SAFETY_TIME_DISABLE | REG05_BQ24196_FAST_CHARGING_TIMEOUT_8H;
    }
    
    rc = bq24196_config_interface(chip, REG05_BQ24196_ADDRESS, 
		val, REG05_BQ24196_CHARGING_SAFETY_TIME_MASK | REG05_BQ24196_FAST_CHARGING_TIMEOUT_MASK);
    if (rc < 0) {
        chg_err("Couldn't complete charge timeout rc = %d\n", rc);
    }
    
    return rc;
}

int bq24196_float_voltage_write(struct oppo_chg_chip *chip, int vfloat_mv)
{
	int rc = 0, value;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	if(vfloat_mv < BQ24196_MIN_FLOAT_MV) {
		chg_err(" bad vfloat_mv:%d,return\n", vfloat_mv);
		return 0;
	}
	value = (vfloat_mv - BQ24196_MIN_FLOAT_MV)/BQ24196_VFLOAT_STEP_MV;
	value <<= REG04_BQ24196_CHARGING_VOL_LIMIT_SHIFT;
	chg_debug( "bq24196_set_float_voltage vfloat_mv = %d value=%d\n", vfloat_mv, value);

	rc = bq24196_config_interface(chip, REG04_BQ24196_ADDRESS, value, REG04_BQ24196_CHARGING_VOL_LIMIT_MASK);
	return rc;
}

int bq24196_set_prechg_current(struct oppo_chg_chip *chip, int ipre_mA)
{
    int value;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	value = (ipre_mA - BQ24196_MIN_PRE_CURRENT_MA)/BQ24196_PRE_CURRENT_STEP_MA;
	value <<= REG03_BQ24196_PRE_CHARGING_CURRENT_LIMIT_SHIFT;

	return bq24196_config_interface(chip, REG03_BQ24196_ADDRESS, 
		value, REG03_BQ24196_PRE_CHARGING_CURRENT_LIMIT_MASK);
}

static int bq24196_set_termchg_current(struct oppo_chg_chip *chip, int term_curr)
{
	int value;
	int rc = 0;
	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	if(term_curr < BQ24196_MIN_TERM_CURRENT_MA)
		term_curr = BQ24196_MIN_TERM_CURRENT_MA;
	value = (term_curr - BQ24196_MIN_TERM_CURRENT_MA)/BQ24196_TERM_CURRENT_STEP_MA;
	value <<= REG03_BQ24196_TERM_CHARGING_CURRENT_LIMIT_SHIFT;
	chg_debug( " value=%d\n", value);

	bq24196_config_interface(chip, REG03_BQ24196_ADDRESS, 
		value, REG03_BQ24196_TERM_CHARGING_CURRENT_LIMIT_MASK);
	return rc;
}

int bq24196_set_rechg_voltage(struct oppo_chg_chip *chip, int recharge_mv)
{
   int reg,rc = 0;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
   
   /* set recharge voltage*/
    if (recharge_mv >= 300) {
        reg = REG04_BQ24196_RECHARGING_THRESHOLD_VOL_300MV;
    } else {
        reg = REG04_BQ24196_RECHARGING_THRESHOLD_VOL_100MV;
    }
    rc = bq24196_config_interface(chip, REG04_BQ24196_ADDRESS, reg, REG04_BQ24196_RECHARGING_THRESHOLD_VOL_MASK);
    if (rc) {
        chg_err("Couldn't set recharging threshold rc = %d\n", rc);     
    }
	 return rc;
}

int bq24196_set_wdt_timer(struct oppo_chg_chip *chip, int reg)
{
    int rc = 0;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_config_interface(chip,
		REG05_BQ24196_ADDRESS, reg, REG05_BQ24196_I2C_WATCHDOG_TIME_MASK);
    
    return rc;
}

static int bq24196_set_chging_term_disable(struct oppo_chg_chip *chip)
{
	int rc = 0;
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	
	rc = bq24196_config_interface(chip, REG05_BQ24196_ADDRESS, 
		REG05_BQ24196_TERMINATION_DISABLE, REG05_BQ24196_TERMINATION_MASK);

	return rc;
}

int bq24196_kick_wdt(struct oppo_chg_chip *chip)
{
	int rc = 0;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_config_interface(chip, REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_WDT_TIMER_RESET, REG01_BQ24196_WDT_TIMER_RESET_MASK);
    
    return rc;
}

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool bq24196_check_charger_suspend_enable(struct oppo_chg_chip *chip);
int bq24196_enable_charging(struct oppo_chg_chip *chip)
{
	int rc;

#ifdef CONFIG_OPPO_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT)
		return 0;
#endif

	if (bq24196_check_charger_suspend_enable(chip)) {
		bq24196_unsuspend_charger(chip);
	}
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("Couldn't bq24196_enable_charging because charger_suspended\n");
		return 0;
	}
	rc = bq24196_config_interface(chip, REG01_BQ24196_ADDRESS, 
			REG01_BQ24196_CHARGING_ENABLE, REG01_BQ24196_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn'tbq24196_enable_charging rc = %d\n", rc);
	}
	return rc;
}

int bq24196_disable_charging(struct oppo_chg_chip *chip)
{
	int rc;

	/* Only for BATTERY_STATUS__HIGH_TEMP or BATTERY_STATUS__WARM_TEMP, */
	/* system power is from charger but NOT from battery to avoid temperature rise problem */
	if (atomic_read(&chip->charger_suspended) == 1) {
		chg_err("Couldn't bq24196_disable_charging because charger_suspended\n");
		return 0;
	}
	rc = bq24196_config_interface(chip, REG01_BQ24196_ADDRESS, 
			REG01_BQ24196_CHARGING_DISABLE, REG01_BQ24196_CHARGING_MASK);
	if (rc < 0) {
		chg_err("Couldn't bq24196_disable_charging  rc = %d\n", rc);
	} else {
		chg_err("battery HIGH_TEMP or WARM_TEMP, bq24196_disable_charging\n");
	}
	return rc;
}
#else
int bq24196_enable_charging(struct oppo_chg_chip *chip)
{
	int rc;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	
    rc = bq24196_config_interface(chip, REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_CHARGING_ENABLE, REG01_BQ24196_CHARGING_MASK);
    if (rc < 0) {
		chg_err("Couldn'tbq24196_enable_charging rc = %d\n", rc);
	}
	
	return rc;
}

int bq24196_disable_charging(struct oppo_chg_chip *chip)
{
	int rc;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_config_interface(chip, REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_CHARGING_DISABLE, REG01_BQ24196_CHARGING_MASK);
    if (rc < 0) {
		chg_err("Couldn't bq24196_disable_charging  rc = %d\n", rc);
	}
	
	return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
static bool bq24196_check_charger_suspend_enable(struct oppo_chg_chip *chip)
{
	int rc = 0;
	int reg_val = 0;
	bool charger_suspend_enable = false;

	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	
	rc = bq24196_read_reg(chip, REG00_BQ24196_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG00_BQ24196_ADDRESS rc = %d\n", rc);
		return 0;
	}
	
	charger_suspend_enable = ((reg_val & REG00_BQ24196_SUSPEND_MODE_MASK) == REG00_BQ24196_SUSPEND_MODE_ENABLE) ? true : false;
		
	return charger_suspend_enable;	
}
static int bq24196_check_charging_enable(struct oppo_chg_chip *chip)
{
	bool rc;
	rc = bq24196_check_charger_suspend_enable(chip);
	if(rc)
		return 0;
	else
		return 1;
}
#else
static int bq24196_check_charging_enable(struct oppo_chg_chip *chip)
{
	int rc = 0, reg_val = 0;
	bool charging_enable = false;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	rc = bq24196_read_reg(chip, REG01_BQ24196_ADDRESS, &reg_val);
    if (rc) {
        chg_err("Couldn't read REG01_BQ24196_ADDRESS rc = %d\n", rc);
        return 0;
    }
	
    charging_enable = ((reg_val & REG01_BQ24196_CHARGING_MASK) == REG01_BQ24196_CHARGING_ENABLE) ? 1 : 0;
	
	return charging_enable;	
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

int bq24196_registers_read_full(struct oppo_chg_chip *chip)
{
	int rc;
    int reg_full = 0,reg_ovp = 0;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_read_reg(chip, REG08_BQ24196_ADDRESS, &reg_full);
    if (rc) {
        chg_err("Couldn't read STAT_C rc = %d\n", rc);
        return 0;
    }
	
    reg_full = ((reg_full & REG08_BQ24196_CHARGING_STATUS_CHARGING_MASK) == REG08_BQ24196_CHARGING_STATUS_TERM_CHARGING) ? 1 : 0;

	rc = bq24196_read_reg(chip, REG09_BQ24196_ADDRESS, &reg_ovp);
	if (rc) {
        chg_err("Couldn't read STAT_D rc = %d\n", rc);
        return 0;
    }

	reg_ovp = ((reg_ovp & REG09_BQ24196_BATTERY_VOLATGE_MASK) == REG09_BQ24196_BATTERY_VOLATGE_HIGH_ERROR) ? 1 : 0;
	//chg_err("bq24196_registers_read_full, reg_full = %d, reg_ovp = %d\r\n", reg_full, reg_ovp);
	return (reg_full || reg_ovp);
}

int bq24196_otg_enable(void)
{
	int rc = 0;

	if (!the_chip) {
		return 0;
	}
#ifndef CONFIG_OPPO_CHARGER_MTK
	if (atomic_read(&the_chip->charger_suspended) == 1) {
		return 0;
	}
#endif	
	//bq24196_unsuspend_charger(the_chip);
	bq24196_config_interface(the_chip, REG00_BQ24196_ADDRESS, 
		REG00_BQ24196_SUSPEND_MODE_DISABLE, REG00_BQ24196_SUSPEND_MODE_MASK);
	
	rc = bq24196_config_interface(the_chip, REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_OTG_ENABLE, REG01_BQ24196_OTG_MASK);
	if (rc)
		chg_err("Couldn't enable  OTG mode rc=%d\n", rc);
	else
		chg_debug( "bq24196_otg_enable rc=%d\n", rc);
	oppo_chg_set_otg_online(true);
	return rc;
}

int bq24196_otg_disable(void)
{
	int rc = 0;

	if (!the_chip) {
		return 0;
	}
	
#ifndef CONFIG_OPPO_CHARGER_MTK	
	if (atomic_read(&the_chip->charger_suspended) == 1) {
		return 0;
	}
#endif
	rc = bq24196_config_interface(the_chip, REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_OTG_DISABLE, REG01_BQ24196_OTG_MASK);
	if (rc) 
		chg_err("Couldn't disable OTG mode rc=%d\n", rc);
	else
		chg_debug( "bq24196_otg_disable rc=%d\n", rc);
	
	oppo_chg_set_otg_online(false);
	return rc;
}

int bq24196_suspend_charger(struct oppo_chg_chip *chip)
{
	int rc;

	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
	
	if (oppo_chg_get_otg_online() == true)
		return 0;
	
	rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, 
		REG00_BQ24196_SUSPEND_MODE_ENABLE, REG00_BQ24196_SUSPEND_MODE_MASK);
	chg_debug( " rc = %d\n", rc);
	if (rc < 0) {
		chg_err("Couldn't bq24196_suspend_charger rc = %d\n", rc);
	}
	
	return rc;
}
int bq24196_unsuspend_charger(struct oppo_chg_chip *chip)
{
	int rc;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

    rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, 
		REG00_BQ24196_SUSPEND_MODE_DISABLE, REG00_BQ24196_SUSPEND_MODE_MASK);
	chg_debug( " rc = %d\n", rc);
    if (rc < 0) {
		chg_err("Couldn't bq24196_unsuspend_charger rc = %d\n", rc);
	}	
	return rc;
}

#ifdef CONFIG_MTK_PMIC_CHIP_MT6353
int bq24196_reset_charger(struct oppo_chg_chip *chip)
{
	int rc;
	
	if (atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}

	rc = bq24196_config_interface(chip, REG00_BQ24196_ADDRESS, 0x32, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG00 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(chip, REG01_BQ24196_ADDRESS, 0x1B, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG01 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(chip, REG02_BQ24196_ADDRESS, 0x60, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG02 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(chip, REG03_BQ24196_ADDRESS, 0x11, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG03 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(chip, REG04_BQ24196_ADDRESS, 0xCA, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG04 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(chip, REG05_BQ24196_ADDRESS, 0x1A, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG05 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(chip, REG06_BQ24196_ADDRESS, 0x03, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG06 rc = %d\n", rc);
	}
	rc = bq24196_config_interface(chip, REG07_BQ24196_ADDRESS, 0x4B, 0xFF);
	if (rc < 0) {
		chg_err("Couldn't reset REG07 rc = %d\n", rc);
	}

	return rc;
}
#else
int bq24196_reset_charger(struct oppo_chg_chip *chip)
{
	int rc;

	if(atomic_read(&chip->charger_suspended) == 1) {
		return 0;
	}
    rc = bq24196_config_interface(chip, REG01_BQ24196_ADDRESS, 
		REG01_BQ24196_REGISTER_RESET, REG01_BQ24196_REGISTER_RESET_MASK);
	chg_debug( " rc = %d\n", rc);
    if (rc < 0) {
		chg_err("Couldn't bq24196_reset_charger rc = %d\n", rc);
	}
	
	return rc;
}
#endif /*CONFIG_MTK_PMIC_CHIP_MT6353*/

static bool bq24196_check_charger_resume(struct oppo_chg_chip *chip)
{
	if(atomic_read(&chip->charger_suspended) == 1) {
		return false;
	} else {
		return true;
	}
}

#define DUMP_REG_LOG_CNT_30S             6

void bq24196_dump_registers(struct oppo_chg_chip *chip)
{
	int rc;
	int addr;
	unsigned int val_buf[BQ24196_REG_NUMBER] = {0x0};
	static int dump_count = 0;
	if(atomic_read(&chip->charger_suspended) == 1) {
		return ;
	}
	
	for (addr = BQ24196_FIRST_REG; addr <= BQ24196_LAST_REG; addr++) {
		rc = bq24196_read_reg(chip, addr, &val_buf[addr]);
		if (rc) {
			 chg_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		}
	}	

// wenbin.liu@SW.Bsp.Driver, 2016/02/29  Add for log tag 
	if(dump_count == DUMP_REG_LOG_CNT_30S) {			
		dump_count = 0;
		chg_debug( "bq24196_reg[0-a]:0x%02x(0),0x%02x(1),0x%02x(2),0x%02x(3),0x%02x(4),0x%02x(5),0x%02x(6),0x%02x(7),0x%02x(8),0x%02x(9),0x%02x(a)\n",
			val_buf[0],val_buf[1],val_buf[2],val_buf[3],val_buf[4],val_buf[5],val_buf[6],val_buf[7],
			val_buf[8],val_buf[9],val_buf[10]);
	}
	dump_count++;

}

static int bq24196_get_chg_current_step(struct oppo_chg_chip *chip)
{
	int rc = 0;
	int reg_val = 0;
	
	rc = bq24196_read_reg(chip, REG02_BQ24196_ADDRESS, &reg_val);
	if (rc) {
		chg_err("Couldn't read REG02_BQ24196_ADDRESS rc = %d\n", rc);
		return 0;
	}

	if(reg_val & REG02_BQ24196_FAST_CHARGING_CURRENT_FORCE20PCT_MASK)
		return 13;
	else 
		return 64;
}


int bq24196_hardware_init(struct oppo_chg_chip *chip)
{
	//must be before set_vindpm_vol and set_input_current
	chip->hw_aicl_point = 4440;
	chip->sw_aicl_point = 4500;
	
	bq24196_reset_charger(chip);

	if(get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
#ifndef CONFIG_MTK_PMIC_CHIP_MT6353
		bq24196_disable_charging(chip);
#endif
		bq24196_float_voltage_write(chip, 4400);
		msleep(100);
	}

	bq24196_float_voltage_write(chip, 4320);
	
	bq24196_set_enable_volatile_writes(chip);
	
	bq24196_set_complete_charge_timeout(chip, OVERTIME_DISABLED);

    bq24196_set_prechg_current(chip, 300);

	bq24196_charging_current_write_fast(chip, 512);
	
    bq24196_set_termchg_current(chip, 150);
    
    bq24196_set_rechg_voltage(chip, 100);

	bq24196_set_vindpm_vol(chip, chip->hw_aicl_point);
	
#ifdef CONFIG_OPPO_CHARGER_MTK
	if(get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
		|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT)
	{
		bq24196_suspend_charger(chip);
	} else {
		bq24196_unsuspend_charger(chip);
	}
#else
	bq24196_unsuspend_charger(chip);
#endif

    bq24196_enable_charging(chip);

    bq24196_set_wdt_timer(chip, REG05_BQ24196_I2C_WATCHDOG_TIME_40S);

	return true;
}
#ifdef CONFIG_OPPO_RTC_DET_SUPPORT
static int rtc_reset_check(void)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc = 0;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return 0;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	if ((tm.tm_year == 110) && (tm.tm_mon == 0) && (tm.tm_mday <= 1)) {
		chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  @@@ wday: %d, yday: %d, isdst: %d\n",
			tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
			tm.tm_wday, tm.tm_yday, tm.tm_isdst);
		rtc_class_close(rtc);
		return 1;
	}

	chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  ###  wday: %d, yday: %d, isdst: %d\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
		tm.tm_wday, tm.tm_yday, tm.tm_isdst);

close_time:
	rtc_class_close(rtc);
	return 0;
}
#endif /* CONFIG_OPPO_RTC_DET_SUPPORT */

struct oppo_chg_operations  bq24196_chg_ops = {
	.dump_registers = bq24196_dump_registers,
	.kick_wdt = bq24196_kick_wdt,
	.hardware_init = bq24196_hardware_init,
	.charging_current_write_fast = bq24196_charging_current_write_fast,
	.set_aicl_point = bq24196_set_aicl_point,
	.input_current_write = bq24196_input_current_limit_write,
	.float_voltage_write = bq24196_float_voltage_write,
	.term_current_set = bq24196_set_termchg_current,
	.charging_enable = bq24196_enable_charging,
	.charging_disable = bq24196_disable_charging,
	.get_charging_enable = bq24196_check_charging_enable,
	.charger_suspend = bq24196_suspend_charger,
	.charger_unsuspend = bq24196_unsuspend_charger,
	.set_rechg_vol = bq24196_set_rechg_voltage,
	.reset_charger = bq24196_reset_charger,
	.read_full = bq24196_registers_read_full,
	.otg_enable = bq24196_otg_enable,
	.otg_disable = bq24196_otg_disable,
	.set_charging_term_disable = bq24196_set_chging_term_disable,
	.check_charger_resume = bq24196_check_charger_resume,
#ifdef 		CONFIG_OPPO_CHARGER_MTK
	.get_charger_type = mt_power_supply_type_check,
	.get_chg_pretype = charger_pretype_get,
	.get_charger_volt = battery_meter_get_charger_voltage,
	.check_chrdet_status = (bool (*) (void)) pmic_chrdet_status,
	.get_instant_vbatt = battery_meter_get_battery_voltage,
	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = (int (*)(void))get_boot_reason,
#ifdef CONFIG_MTK_HAFG_20
	.get_rtc_soc = get_rtc_spare_oppo_fg_value,
	.set_rtc_soc = set_rtc_spare_oppo_fg_value,
#else
	.get_rtc_soc = get_rtc_spare_fg_value,
	.set_rtc_soc = set_rtc_spare_fg_value,
#endif
	.set_power_off = mt_power_off,
	.usb_connect = mt_usb_connect,
	.usb_disconnect = mt_usb_disconnect,
#else
	.get_charger_type = qpnp_charger_type_get,
	.get_charger_volt = qpnp_get_prop_charger_voltage_now,
	.check_chrdet_status = qpnp_lbc_is_usb_chg_plugged_in,
	.get_instant_vbatt = qpnp_get_prop_battery_voltage_now,
	.get_boot_mode = get_boot_mode,
	.get_rtc_soc = qpnp_get_pmic_soc_memory,
	.set_rtc_soc = qpnp_set_pmic_soc_memory,
#endif
	.get_chg_current_step = bq24196_get_chg_current_step,
#ifdef CONFIG_OPPO_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = oppo_chg_get_dyna_aicl_result,
#endif
#ifdef CONFIG_OPPO_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
#endif
};

static void register_charger_devinfo(void)
{
	int ret = 0;
	char *version;
	char *manufacture;
	
	version = "bq24196";
	manufacture = "TI";

	ret = register_device_proc("charger", version, manufacture);
	if (ret)
		chg_err("register_charger_devinfo fail\n");
}

static int bq24196_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    //int err=0; 
	int reg = 0;
	struct oppo_chg_chip	*chip;

#ifndef CONFIG_OPPO_CHARGER_MTK	
	struct power_supply *usb_psy;
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
        chg_err("USB psy not found; deferring probe\n");
        return -EPROBE_DEFER;
    }
#endif
    chg_debug( " call \n");

	chip = devm_kzalloc(&client->dev,
		sizeof(struct oppo_chg_chip), GFP_KERNEL);
	if (!chip) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}
	
	chip->client = client;
    chip->dev = &client->dev;
	chip->chg_ops = &bq24196_chg_ops;
	oppo_chg_parse_dt(chip);
	
	if(oppo_gauge_check_chip_is_null() || (chip->vooc_project && oppo_vooc_check_chip_is_null()) 
		|| oppo_pmic_check_chip_is_null())
	{
		chg_err("[oppo_chg_init] vooc || gauge || pmic not ready, will do after bettery init.\n");
		return -EPROBE_DEFER;
	}

#if 0
    err = bq24196_read_reg(chip, REG00_BQ24196_ADDRESS, &reg);	
	if (err < 0) {
        chg_err("Failed to detect bq24196, device may be absent\n");
        return -ENODEV;
    }
#endif

	atomic_set(&chip->charger_suspended, 0);
	bq24196_dump_registers(chip);
	
	bq24196_hardware_init(chip);
	
	chip->authenticate = get_oppo_high_battery_status();
	if (!chip->authenticate)
		bq24196_suspend_charger(chip);
		

	oppo_chg_init(chip);
	register_charger_devinfo();
	the_chip = chip;

    return 0;                                                                                       

}


static struct i2c_driver bq24196_i2c_driver;

static int bq24196_driver_remove(struct i2c_client *client)
{

	int ret=0;
    
	//ret = i2c_del_driver(&bq24196_i2c_driver);
	chg_debug( "  ret = %d\n", ret);
	return 0;
}

static unsigned long suspend_tm_sec = 0;
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		chg_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		chg_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		chg_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static int bq24196_resume(struct i2c_client *client)
{	
	unsigned long resume_tm_sec = 0, sleep_time = 0;
	int rc;

	if(!the_chip) {
		return 0;
	}
	atomic_set(&the_chip->charger_suspended, 0);
	rc = get_current_time(&resume_tm_sec);
	if (rc || suspend_tm_sec == -1) {
		chg_err("RTC read failed\n");
		sleep_time = 0;
	} else {
		sleep_time = resume_tm_sec - suspend_tm_sec;
	}

	if(sleep_time < 0) {
		sleep_time = 0;
	}
	oppo_chg_soc_update_when_resume(sleep_time);
	return 0;
}

static int bq24196_suspend(struct i2c_client *client, pm_message_t mesg)
{
	if(!the_chip) {
		return 0;
	}
	atomic_set(&the_chip->charger_suspended, 1);
	if (get_current_time(&suspend_tm_sec)) {
		chg_err("RTC read failed\n");
		suspend_tm_sec = -1;
	}
	return 0;
}

static void bq24196_reset(struct i2c_client *client)
{
	bq24196_otg_disable();
}


/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/

static const struct of_device_id bq24196_match[] = {
	{ .compatible = "oppo,bq24196-charger"},
	{ },
};

static const struct i2c_device_id bq24196_id[] = {
	{"bq24196-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq24196_id);


static struct i2c_driver bq24196_i2c_driver = {
	.driver		= {
		.name = "bq24196-charger",
		.owner	= THIS_MODULE,
		.of_match_table = bq24196_match,
	},
	.probe		= bq24196_driver_probe,
	.remove		= bq24196_driver_remove,
	.resume		= bq24196_resume,
	.suspend	= bq24196_suspend,
	.shutdown	= bq24196_reset,
	.id_table	= bq24196_id,
};


module_i2c_driver(bq24196_i2c_driver);
MODULE_DESCRIPTION("Driver for bq24196 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:bq24196-charger");
