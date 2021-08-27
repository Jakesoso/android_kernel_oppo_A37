/**********************************************************************************
* Copyright (c)  2008-2015  Guangdong OPPO Mobile Comm Corp., Ltd
* VENDOR_EDIT
* Description: Charger IC management module for charger system framework.
*              Manage all charger IC and define abstarct function flow.
* Version   : 1.0
* Date      : 2015-06-22
* Author    : fanhui@PhoneSW.BSP
* 			: Fanhong.Kong@ProDrv.CHG		   	
* ------------------------------ Revision History: --------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2015-06-22    fanhui@PhoneSW.BSP    			Created for new architecture
* Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
***********************************************************************************/
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include "oppo_charger.h"
#include "oppo_vooc.h"
#include "oppo_gauge.h"

#define VOOC_NOTIFY_FAST_PRESENT			0x52
#define VOOC_NOTIFY_FAST_ABSENT				0x54
#define VOOC_NOTIFY_ALLOW_READING_IIC		0x58
#define VOOC_NOTIFY_NORMAL_TEMP_FULL		0x5a
#define VOOC_NOTIFY_LOW_TEMP_FULL			0x53
#define VOOC_NOTIFY_FIRMWARE_UPDATE			0x56
#define VOOC_NOTIFY_BAD_CONNECTED			0x59
#define VOOC_NOTIFY_TEMP_OVER				0x5c
#define VOOC_NOTIFY_ADAPTER_FW_UPDATE		0x5b
#define VOOC_NOTIFY_BTB_TEMP_OVER			0x5d

extern int Charger_Abnormal_Log;
extern int Enable_CHARGER_LOG;
#define vooc_xlog_printk(num, fmt, ...) \
  do { \
    if (Enable_CHARGER_LOG >= (int)num) { \
		printk(KERN_NOTICE pr_fmt("[OPPO_CHG][%s]"fmt),__func__,##__VA_ARGS__);\
    } \
  } while (0)


static struct oppo_vooc_chip *g_vooc_chip = NULL;
static struct oppo_adapter_chip *g_adapter_chip = NULL;

bool __attribute__((weak)) oppo_get_fg_i2c_err_occured(void)
{
	return false;
}

void __attribute__((weak)) oppo_set_fg_i2c_err_occured(bool i2c_err)
{
	return;
}

void oppo_vooc_battery_update(struct oppo_vooc_chip *chip)
{
	if(!chip->batt_psy){	
		chip->batt_psy = power_supply_get_by_name("battery");
	} 
	if(chip->batt_psy) {
		power_supply_changed(chip->batt_psy);	
	}
}

void oppo_vooc_switch_mode(int mode)
{
	if(!g_vooc_chip) {
		chg_err("  g_vooc_chip is NULL\n");
	} else {
		g_vooc_chip->vops->set_switch_mode(g_vooc_chip, mode);
	}
}

static void oppo_vooc_watchdog(unsigned long data)
{
	struct oppo_vooc_chip *chip = (struct oppo_vooc_chip *)data;
	
	chg_err("watchdog bark: cannot receive mcu data\n");
	chip->allow_reading = true;
	chip->fastchg_started = false;
	chip->fastchg_ing = false;
	chip->fastchg_to_normal = false;
	chip->fastchg_to_warm = false;
	chip->fastchg_low_temp_full = false;
	chip->btb_temp_over = false;
	Charger_Abnormal_Log = CRITICAL_LOG_VOOC_WATCHDOG;
	oppo_chg_set_chargerid_switch_val(0);
	oppo_chg_clear_chargerid_info();
	chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
	oppo_chg_set_charger_type_unknown();
	wake_unlock(&chip->vooc_wake_lock);
}

static void adapter_update_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oppo_vooc_chip *chip = container_of(dwork,struct oppo_vooc_chip, adapter_update_work);
	bool update_result = false;
	int i = 0;

	if(!g_adapter_chip) {
		chg_err("g_vooc_chip NULL\n");
		return ;
	}
	
	vooc_xlog_printk(CHG_LOG_CRTI, " begin\n");
	chip->vops->set_data_active(chip);
	chip->vops->set_clock_sleep(chip);
	for(i = 0;i < 2;i++) {
		update_result = g_adapter_chip->vops->adapter_update(g_adapter_chip, 
			chip->vooc_gpio.clock_gpio,chip->vooc_gpio.data_gpio);
		if(update_result == true) {
			break;
		}
		if(i < 1) {
			msleep(1650);
		}
	}
	
	if(update_result) {
		chip->adapter_update_real = ADAPTER_FW_UPDATE_SUCCESS;
	} else {
		chip->adapter_update_real = ADAPTER_FW_UPDATE_FAIL;
		chip->adapter_update_report = chip->adapter_update_real;
	}
	msleep(20);
	chip->vops->eint_regist(chip);
	oppo_chg_set_chargerid_switch_val(0);
	chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
	chip->vops->reset_mcu(chip);
	if(update_result) {
		msleep(2000);
		chip->adapter_update_report = ADAPTER_FW_UPDATE_SUCCESS;
	}
	oppo_vooc_battery_update(chip);
	vooc_xlog_printk(CHG_LOG_CRTI, "  end update_result:%d\n",update_result);
	wake_unlock(&chip->vooc_wake_lock);
		
}

#define ADAPTER_UPDATE_DELAY              1400
static void oppo_vooc_adapter_update(struct oppo_vooc_chip *chip)
{
	vooc_xlog_printk(CHG_LOG_CRTI, " call \n");
	//schedule_delayed_work_on(7, &chip->adapter_update_work, 
	//			round_jiffies_relative(msecs_to_jiffies(ADAPTER_UPDATE_DELAY)));
	schedule_delayed_work(&chip->adapter_update_work, 
				round_jiffies_relative(msecs_to_jiffies(ADAPTER_UPDATE_DELAY)));
}

static void check_adapter_out_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oppo_vooc_chip *chip = container_of(dwork,struct oppo_vooc_chip, check_adapter_out_work);
	int chg_vol = 0;

	chg_vol = oppo_chg_get_charger_voltage();
	if(chg_vol >= 0 && chg_vol < 2000) {
		chip->vops->reset_fastchg_after_usbout(chip);
		oppo_chg_clear_chargerid_info();
		oppo_vooc_battery_update(chip);
		vooc_xlog_printk(CHG_LOG_CRTI, "adapter out,chg_vol:%d\n", chg_vol);
	}
}

static void oppo_vooc_check_adapter_out(struct oppo_vooc_chip *chip)
{
	vooc_xlog_printk(CHG_LOG_CRTI, "  call\n");
	schedule_delayed_work(&chip->check_adapter_out_work, 
				round_jiffies_relative(msecs_to_jiffies(3000)));
}

static void oppo_vooc_fastchg_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oppo_vooc_chip *chip = container_of(dwork, struct oppo_vooc_chip, fastchg_work);
	int i = 0, bit = 0, data = 0;
	int ret_info = 0;
	static bool isnot_power_on = false, fw_ver_info = false, adapter_fw_ver_info = false, data_err = false;
	int volt = 0, temp = 0, soc = 0, current_now = 0, chg_vol = 0, remain_cap = 0;
	static bool phone_mcu_updated = false, normalchg_disabled = false;

	if(!g_adapter_chip) {
		chg_err(" g_adapter_chip NULL\n");
		return ;
	}

	usleep_range(2000,2000);
	if(chip->vops->get_gpio_ap_data(chip) != 1) {
		//vooc_xlog_printk(CHG_LOG_CRTI, "  Shield fastchg irq,return\r\n");
		return;
	}


	chip->vops->eint_unregist(chip);
	for(i = 0; i < 7; i++)
	{
		bit = chip->vops->read_ap_data(chip);
		data |= bit<<(6-i);	
		if((i == 2) && (data != 0x50) && (!fw_ver_info) && (!adapter_fw_ver_info)){	//data recvd not start from "101"
			vooc_xlog_printk(CHG_LOG_CRTI, "  data err:0x%x\n",data);
 			if(chip->fastchg_started == true) {
				chip->allow_reading= true;
				chip->fastchg_started = false;
				chip->fastchg_to_normal = false;
				chip->fastchg_to_warm = false;
				chip->fastchg_ing = false;
				adapter_fw_ver_info = false;
				//chip->adapter_update_real = ADAPTER_FW_UPDATE_NONE;
				//chip->adapter_update_report = chip->adapter_update_real;
				chip->btb_temp_over = false;
				oppo_set_fg_i2c_err_occured(false);
				oppo_chg_set_chargerid_switch_val(0);
				chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
				data_err = true;
				if(chip->fastchg_dummy_started) {
					chg_vol = oppo_chg_get_charger_voltage();
					if(chg_vol >= 0 && chg_vol < 2000) {
						chip->fastchg_dummy_started = false;
						oppo_chg_clear_chargerid_info();
						vooc_xlog_printk(CHG_LOG_CRTI, "chg_vol:%d dummy_started:false\n",chg_vol);
					}
				} else {
					oppo_chg_clear_chargerid_info();
				}
			}
			goto out;
		}
	}
	
	vooc_xlog_printk(CHG_LOG_CRTI, " recv data:0x%x,fw_version = 0x%x\n", data, chip->fw_data_version);
	
	if(data == VOOC_NOTIFY_FAST_PRESENT) 
	{
		wake_lock(&chip->vooc_wake_lock);
		oppo_set_fg_i2c_err_occured(false);
		chip->need_to_up =0;
		fw_ver_info = false;
		adapter_fw_ver_info = false;
		data_err = false;
		phone_mcu_updated = false;
		normalchg_disabled = false;
		if(chip->adapter_update_real == ADAPTER_FW_UPDATE_FAIL) {
			chip->adapter_update_real = ADAPTER_FW_UPDATE_NONE;
			chip->adapter_update_report = chip->adapter_update_real;
		}
		if(oppo_vooc_get_fastchg_allow() == true){
			chip->allow_reading = false;
			chip->fastchg_started = true;
			chip->fastchg_ing = false;
			chip->fastchg_dummy_started = false;
			chip->fastchg_to_warm = false;
			chip->btb_temp_over = false;
		} else {
			chip->allow_reading = true;
			chip->fastchg_dummy_started = true;
			chip->fastchg_started = false;
			chip->fastchg_to_normal = false;
			chip->fastchg_to_warm = false;
			chip->fastchg_ing = false;
			chip->btb_temp_over = false;
			oppo_chg_set_chargerid_switch_val(0);
			chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
		}
		mod_timer(&chip->watchdog, jiffies+msecs_to_jiffies(25000));
		if(!isnot_power_on){
			isnot_power_on = true;
			ret_info = 0x1;
		} else {
			ret_info = 0x2;
		}
	}
	else if(data == VOOC_NOTIFY_FAST_ABSENT) 
	{
		chip->allow_reading = true;
		chip->fastchg_started = false;
		chip->fastchg_to_normal = false;
		chip->fastchg_to_warm = false;
		chip->fastchg_ing = false;
		chip->btb_temp_over = false;
		adapter_fw_ver_info = false;
		oppo_set_fg_i2c_err_occured(false);
		if(chip->fastchg_dummy_started) {
			chg_vol = oppo_chg_get_charger_voltage();
			if(chg_vol >= 0 && chg_vol < 2000) {
				chip->fastchg_dummy_started = false;
				oppo_chg_clear_chargerid_info();
				vooc_xlog_printk(CHG_LOG_CRTI, "chg_vol:%d dummy_started:false\n",chg_vol);
			}
		} else {
			oppo_chg_clear_chargerid_info();
		}
		vooc_xlog_printk(CHG_LOG_CRTI, "fastchg stop unexpectly, switch off fastchg\n");
		oppo_chg_set_chargerid_switch_val(0);
		chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
		del_timer(&chip->watchdog);
		ret_info = 0x2;
	}
	else if(data == VOOC_NOTIFY_ALLOW_READING_IIC) 
	{
		chip->fastchg_ing = true;
		chip->allow_reading = true;
		adapter_fw_ver_info = false;
		soc = oppo_gauge_get_batt_soc();
		if (oppo_get_fg_i2c_err_occured() == false) {
			volt = oppo_gauge_get_batt_mvolts();
		}
		if (oppo_get_fg_i2c_err_occured() == false) {
			temp = oppo_gauge_get_batt_temperature();
		}
		if (oppo_get_fg_i2c_err_occured() == false) {
			current_now = oppo_gauge_get_batt_current();
		}
		if (oppo_get_fg_i2c_err_occured() == false) {
			remain_cap = oppo_gauge_get_remaining_capacity();
		}
		oppo_chg_kick_wdt();
		if(!normalchg_disabled) {
			oppo_chg_disable_charge();
			normalchg_disabled = true;
		}
		//don't read
		chip->allow_reading = false;
		vooc_xlog_printk(CHG_LOG_CRTI, " volt:%d,temp:%d,soc:%d,current_now:%d,rm:%d, i2c_err:%d\n",
			volt, temp, soc, current_now, remain_cap, oppo_get_fg_i2c_err_occured());
		mod_timer(&chip->watchdog, jiffies+msecs_to_jiffies(25000));
		ret_info = 0x2;
	}
	else if(data == VOOC_NOTIFY_NORMAL_TEMP_FULL)
	{
		vooc_xlog_printk(CHG_LOG_CRTI, "VOOC_NOTIFY_NORMAL_TEMP_FULL\r\n");
		oppo_chg_set_chargerid_switch_val(0);
		chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
		del_timer(&chip->watchdog);
		ret_info = 0x2;
	}
	else if(data == VOOC_NOTIFY_LOW_TEMP_FULL)
	{
		vooc_xlog_printk(CHG_LOG_CRTI, " fastchg low temp full,switch NORMAL_CHARGER_MODE\n");
		oppo_chg_set_chargerid_switch_val(0);
		chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
		del_timer(&chip->watchdog);
		ret_info = 0x2;
	}
	else if(data == VOOC_NOTIFY_BAD_CONNECTED)
	{
		vooc_xlog_printk(CHG_LOG_CRTI, " fastchg bad connected,switch NORMAL_CHARGER_MODE\n");
		//usb bad connected,stop fastchg
		chip->btb_temp_over = false;	//to switch to normal mode
		oppo_chg_set_chargerid_switch_val(0);
		chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
	
		del_timer(&chip->watchdog);
		ret_info = 0x2;
		Charger_Abnormal_Log = CRITICAL_LOG_VOOC_BAD_CONNECTED;
	}
	else if(data == VOOC_NOTIFY_TEMP_OVER)
	{
		//fastchg temp over 45 or under 20
		vooc_xlog_printk(CHG_LOG_CRTI, " fastchg temp > 45 or < 20,switch NORMAL_CHARGER_MODE\n");
		oppo_chg_set_chargerid_switch_val(0);
		chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
		del_timer(&chip->watchdog);
		ret_info = 0x2;
	}
	else if(data == VOOC_NOTIFY_BTB_TEMP_OVER) {
		vooc_xlog_printk(CHG_LOG_CRTI, "  btb_temp_over\n");
		chip->fastchg_ing = false;
		chip->btb_temp_over = true;
		chip->fastchg_dummy_started = false;
		chip->fastchg_started = false;
		chip->fastchg_to_normal = false;
		chip->fastchg_to_warm = false;
		adapter_fw_ver_info = false;
		mod_timer(&chip->watchdog,
			  jiffies + msecs_to_jiffies(25000));
		ret_info = 0x2;
		Charger_Abnormal_Log = CRITICAL_LOG_VOOC_BTB;
	} 
	else if(data == VOOC_NOTIFY_FIRMWARE_UPDATE)
	{
		vooc_xlog_printk(CHG_LOG_CRTI, " firmware update,get fw_ver ready!\n");
		//ready to get fw_ver
		fw_ver_info = 1;
		ret_info = 0x2;
	}
	else if(fw_ver_info)
	{
		//get fw_ver
		//fw in local is large than mcu1503_fw_ver
		if((!chip->have_updated) && (chip->firmware_data[chip->fw_data_count- 4] != data)){
			ret_info = 0x2;
			chip->need_to_up = 1;	//need to update fw
			isnot_power_on = false;
		} else {
			ret_info = 0x1;
			chip->need_to_up = 0;	//fw is already new,needn't to up
			adapter_fw_ver_info = true;
		}
		vooc_xlog_printk(CHG_LOG_CRTI, "local_fw:0x%x,need_to_up_fw:%d\n",chip->firmware_data[chip->fw_data_count- 4],chip->need_to_up);
		fw_ver_info = 0;
	} else if(adapter_fw_ver_info) {
#ifdef CONFIG_OPPO_CHARGER_MTK	
		if(g_adapter_chip->adapter_firmware_data[g_adapter_chip->adapter_fw_data_count - 4] > data
			&& (oppo_gauge_get_batt_soc() > 2) && (chip->vops->is_power_off_charging(chip) == false)
			&& (chip->adapter_update_real != ADAPTER_FW_UPDATE_SUCCESS)) {
#else
			if(0){
#endif
			ret_info = 0x02;
			chip->adapter_update_real = ADAPTER_FW_NEED_UPDATE;
			chip->adapter_update_report = chip->adapter_update_real;
		} else {
			ret_info = 0x01;
			chip->adapter_update_real = ADAPTER_FW_UPDATE_NONE;
			chip->adapter_update_report = chip->adapter_update_real;
		}
		adapter_fw_ver_info = false;
		mod_timer(&chip->watchdog,
			  jiffies + msecs_to_jiffies(25000));
	} else if(data == VOOC_NOTIFY_ADAPTER_FW_UPDATE) {
		wake_lock(&chip->vooc_wake_lock);
		ret_info = 0x02;
		chip->adapter_update_real = ADAPTER_FW_NEED_UPDATE;
		chip->adapter_update_report = chip->adapter_update_real;
		mod_timer(&chip->watchdog,
			  jiffies + msecs_to_jiffies(25000));
	} else {
		oppo_chg_set_chargerid_switch_val(0);
		oppo_chg_clear_chargerid_info();
		chip->vops->set_switch_mode(chip, NORMAL_CHARGER_MODE);
		chip->vops->reset_mcu(chip);
		msleep(100);	//avoid i2c conflict
		chip->allow_reading = true;
		chip->fastchg_started = false;
		chip->fastchg_to_normal= false;
		chip->fastchg_to_warm= false;
		chip->fastchg_ing= false;
		chip->btb_temp_over = false;
		adapter_fw_ver_info = false;
		data_err = true;
		vooc_xlog_printk(CHG_LOG_CRTI, " data err,set 0x101,data=0x%x switch off fastchg\n", data);
		goto out;
	}

	msleep(2);
	
	chip->vops->set_data_sleep(chip);
	chip->vops->reply_mcu_data(chip, ret_info,oppo_gauge_get_device_type());
	
out:
	chip->vops->set_data_active(chip);
	
	chip->vops->set_clock_active(chip);
	usleep_range(10000,10000);
	chip->vops->set_clock_sleep(chip);
	usleep_range(25000,25000);
		
	if(data == VOOC_NOTIFY_NORMAL_TEMP_FULL || data == VOOC_NOTIFY_BAD_CONNECTED){
		usleep_range(350000,350000);
		chip->allow_reading = true;
		chip->fastchg_ing = false;
		chip->fastchg_to_normal = true;
		chip->fastchg_started = false;
		chip->fastchg_to_warm = false;
	} else if(data == VOOC_NOTIFY_LOW_TEMP_FULL){		
		usleep_range(350000,350000);
		chip->allow_reading = true;
		chip->fastchg_ing = false;
		chip->fastchg_low_temp_full = true;
		chip->fastchg_to_normal = false;
		chip->fastchg_started = false;
		chip->fastchg_to_warm = false;
	} else if(data == VOOC_NOTIFY_TEMP_OVER){
		usleep_range(350000,350000);
		chip->fastchg_ing = false;
		chip->fastchg_to_warm = true;
		chip->allow_reading = true;
		chip->fastchg_low_temp_full = false;
		chip->fastchg_to_normal = false;
		chip->fastchg_started = false;
	} 
		
	if(chip->need_to_up){
		msleep(500);
		del_timer(&chip->watchdog);
		chip->vops->fw_update(chip);
		chip->need_to_up = 0;
		phone_mcu_updated = true;
		mod_timer(&chip->watchdog, jiffies + msecs_to_jiffies(25000));
	}

	if((data == VOOC_NOTIFY_FAST_ABSENT || (data_err && !phone_mcu_updated) || data == VOOC_NOTIFY_BTB_TEMP_OVER) 
		&& (chip->fastchg_dummy_started == false)) {
		oppo_chg_set_charger_type_unknown();
		oppo_chg_wake_update_work();
	} else if(data == VOOC_NOTIFY_NORMAL_TEMP_FULL || data == VOOC_NOTIFY_TEMP_OVER || data == VOOC_NOTIFY_BAD_CONNECTED
		|| data == VOOC_NOTIFY_LOW_TEMP_FULL) {
		oppo_chg_set_charger_type_unknown();
		oppo_vooc_check_adapter_out(chip);
	} else if(data == VOOC_NOTIFY_BTB_TEMP_OVER) {
		oppo_chg_set_charger_type_unknown();
	}

	if(chip->adapter_update_real != ADAPTER_FW_NEED_UPDATE) {
		chip->vops->eint_regist(chip);
	} 
		
	if(chip->adapter_update_real == ADAPTER_FW_NEED_UPDATE) {
		chip->allow_reading = true;
		chip->fastchg_started = false;
		chip->fastchg_to_normal = false;
		chip->fastchg_low_temp_full = false;
		chip->fastchg_to_warm = false;
		chip->fastchg_ing = false;
		del_timer(&chip->watchdog);
		oppo_vooc_battery_update(chip);
		oppo_vooc_adapter_update(chip);
	} else if((data == VOOC_NOTIFY_FAST_PRESENT) || (data == VOOC_NOTIFY_ALLOW_READING_IIC)
				|| (data == VOOC_NOTIFY_BTB_TEMP_OVER)){
		oppo_vooc_battery_update(chip);
	} else if((data == VOOC_NOTIFY_LOW_TEMP_FULL) || (data == VOOC_NOTIFY_FAST_ABSENT) || (data == VOOC_NOTIFY_NORMAL_TEMP_FULL) 
				|| (data == VOOC_NOTIFY_BAD_CONNECTED) || (data == VOOC_NOTIFY_TEMP_OVER)){
		oppo_vooc_battery_update(chip);
#ifdef CHARGE_PLUG_IN_TP_AVOID_DISTURB
		charge_plug_tp_avoid_distrub(1,is_oppo_fast_charger);
#endif 
		wake_unlock(&chip->vooc_wake_lock);
	} else if(data_err) {
		data_err = false;
		oppo_vooc_battery_update(chip);
#ifdef CHARGE_PLUG_IN_TP_AVOID_DISTURB
		charge_plug_tp_avoid_distrub(1,is_oppo_fast_charger);
#endif 
		wake_unlock(&chip->vooc_wake_lock);
	}
}

void fw_update_thread(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct oppo_vooc_chip *chip = container_of(dwork,struct oppo_vooc_chip, fw_update_work);
	chip->vops->fw_check_then_recover(chip);	
}

#define FASTCHG_FW_INTERVAL_INIT              1000//1S
void oppo_vooc_fw_update_work_init(struct oppo_vooc_chip *chip)
{
    INIT_DELAYED_WORK(&chip->fw_update_work, fw_update_thread);        
    schedule_delayed_work(&chip->fw_update_work,
                            round_jiffies_relative(msecs_to_jiffies(FASTCHG_FW_INTERVAL_INIT)));
}


void oppo_vooc_shedule_fastchg_work(void)
{
	if(!g_vooc_chip) {
		chg_err(" g_vooc_chip is NULL\n");
	} else {
		schedule_delayed_work(&g_vooc_chip->fastchg_work, 0);
	}
}

void oppo_vooc_init(struct oppo_vooc_chip *chip)
{
	chip->allow_reading = true;
	chip->fastchg_started = false;
	chip->fastchg_dummy_started = false;
	chip->fastchg_ing = false;
	chip->fastchg_to_normal = false;
	chip->fastchg_to_warm = false;
	chip->fastchg_allow = false; 
	chip->fastchg_low_temp_full = false;

	chip->have_updated = false;
	chip->need_to_up = false;
	chip->btb_temp_over = false;
	chip->adapter_update_real = ADAPTER_FW_UPDATE_NONE;
	chip->adapter_update_report = chip->adapter_update_real;
	chip->mcu_update_ing = false;
	chip->mcu_boot_by_gpio = false;
	chip->dpdm_switch_mode = NORMAL_CHARGER_MODE;
	//chip->batt_psy = power_supply_get_by_name("battery");
	
	init_timer(&chip->watchdog);
	chip->watchdog.data = (unsigned long)chip;
	chip->watchdog.function = oppo_vooc_watchdog;
	wake_lock_init(&chip->vooc_wake_lock, WAKE_LOCK_SUSPEND, "vooc_wake_lock");
	
	INIT_DELAYED_WORK(&chip->fastchg_work, oppo_vooc_fastchg_func);
	INIT_DELAYED_WORK(&chip->adapter_update_work, adapter_update_work_func);
	INIT_DELAYED_WORK(&chip->check_adapter_out_work, check_adapter_out_work_func);
	g_vooc_chip = chip;
	chip->vops->eint_regist(chip);
}

void oppo_adapter_init(struct oppo_adapter_chip *chip)
{
	g_adapter_chip = chip;
}

bool oppo_vooc_wake_fastchg_work(struct oppo_vooc_chip *chip)
{
	return schedule_delayed_work(&chip->fastchg_work, 0);
}

void oppo_vooc_print_log(void)
{
	if(!g_vooc_chip) {
		return ;
	}

	vooc_xlog_printk(CHG_LOG_CRTI, "VOOC[ %d / %d / %d / %d / %d / %d]\n", 
		g_vooc_chip->fastchg_allow, g_vooc_chip->fastchg_started, g_vooc_chip->fastchg_dummy_started,
		g_vooc_chip->fastchg_to_normal, g_vooc_chip->fastchg_to_warm,g_vooc_chip->btb_temp_over);
}

bool oppo_vooc_get_allow_reading(void)
{
	if(!g_vooc_chip) 
		return true;
	else
		return g_vooc_chip->allow_reading;
}

bool oppo_vooc_get_fastchg_started(void)
{
	if(!g_vooc_chip) 
		return false;
	else
		return g_vooc_chip->fastchg_started;
}

bool oppo_vooc_get_fastchg_ing(void)
{
	if(!g_vooc_chip) 
		return false;
	else
		return g_vooc_chip->fastchg_ing;
}

bool oppo_vooc_get_fastchg_allow(void)
{
	if(!g_vooc_chip) 
		return false;
	else
		return g_vooc_chip->fastchg_allow;
}

void oppo_vooc_set_fastchg_allow(int enable)
{	
	if(!g_vooc_chip) 
		return;
	else
		g_vooc_chip->fastchg_allow = enable;
}

bool oppo_vooc_get_fastchg_to_normal(void)
{
	if(!g_vooc_chip) 
		return false;
	else
		return g_vooc_chip->fastchg_to_normal;
}

void oppo_vooc_set_fastchg_to_normal_false(void)
{
	if(!g_vooc_chip) 
		return;
	else
		g_vooc_chip->fastchg_to_normal = false;
}


bool oppo_vooc_get_fastchg_to_warm(void)
{
	if(!g_vooc_chip) 
		return false;
	else
		return g_vooc_chip->fastchg_to_warm;
}

void oppo_vooc_set_fastchg_to_warm_false(void)
{
	if(!g_vooc_chip) 
		return;
	else
		g_vooc_chip->fastchg_to_warm = false;
}

bool oppo_vooc_get_fastchg_low_temp_full()
{
	if(!g_vooc_chip) 
		return false;
	else
		return g_vooc_chip->fastchg_low_temp_full;
}

void oppo_vooc_set_fastchg_low_temp_full_false(void)
{
	if(!g_vooc_chip) 
		return ;
	else
		g_vooc_chip->fastchg_low_temp_full = false;
}

bool oppo_vooc_get_fastchg_dummy_started(void)
{
	if(!g_vooc_chip) 
		return false;
	else
		return g_vooc_chip->fastchg_dummy_started;
}

void oppo_vooc_set_fastchg_dummy_started_false(void)
{
	if(!g_vooc_chip) 
		return ;
	else
		g_vooc_chip->fastchg_dummy_started = false;
}

int oppo_vooc_get_adapter_update_status(void)
{
	if(!g_vooc_chip) 
		return ADAPTER_FW_UPDATE_NONE;
	else
		return g_vooc_chip->adapter_update_report;
}

int oppo_vooc_get_adapter_update_real_status(void)
{
	if(!g_vooc_chip) 
		return ADAPTER_FW_UPDATE_NONE;
	else
		return g_vooc_chip->adapter_update_real;
}

bool oppo_vooc_get_btb_temp_over(void)
{
	if(!g_vooc_chip) 
		return false;
	else
		return g_vooc_chip->btb_temp_over;
}

void oppo_vooc_reset_fastchg_after_usbout(void)
{
	if(!g_vooc_chip) 
		return ;
	else
		g_vooc_chip->vops->reset_fastchg_after_usbout(g_vooc_chip);
}

void oppo_vooc_switch_fast_chg(void)
{
	if(!g_vooc_chip) 
		return;
	else
		g_vooc_chip->vops->switch_fast_chg(g_vooc_chip);
}

void oppo_vooc_set_ap_clk_high(void)
{
	if(!g_vooc_chip) 
		return;
	else
		g_vooc_chip->vops->set_clock_sleep(g_vooc_chip);
}

void oppo_vooc_reset_mcu(void)
{
	if(!g_vooc_chip) 
		return;
	else
		g_vooc_chip->vops->reset_mcu(g_vooc_chip);
}

bool oppo_vooc_check_chip_is_null(void)
{
	if(!g_vooc_chip) 
		return true;
	else
		return false;
}

int oppo_vooc_get_vooc_switch_val(void)
{
	if(!g_vooc_chip)
		return 0;
	else
		return g_vooc_chip->vops->get_switch_gpio_val(g_vooc_chip);
}

