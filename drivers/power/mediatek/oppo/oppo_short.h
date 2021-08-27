/**********************************************************************************
* Copyright (c)  2017-2019  Guangdong OPPO Mobile Comm Corp., Ltd
* VENDOR_EDIT
* Description: For short circuit battery check
* Version   : 1.0
* Date      : 2017-10-01
* Author    : SJC@PhoneSW.BSP		   	
* ------------------------------ Revision History: --------------------------------
* <version>       <date>        	<author>              		<desc>
* Revision 1.0    2017-10-01  	SJC@PhoneSW.BSP    		Created for new architecture
***********************************************************************************/
#ifndef _OPPO_BATTERY_H_
#define _OPPO_BATTERY_H_

#define BAD_CONFIG_FILE		"/data/system/bad_bat_config.ini"
#define ERR_CODE_FILE		"/data/system/bad_bat_err_code.ini"
#define EXIT_CODE_FILE		"/data/system/bad_bat_exit_code.ini"
#define INVALID_DATA		-9999

typedef enum
{
	SHORT_C_BATT_STATUS__NORMAL = 0,
	SHORT_C_BATT_STATUS__CV_ERR_CODE1,
	SHORT_C_BATT_STATUS__FULL_ERR_CODE2,
	SHORT_C_BATT_STATUS__FULL_ERR_CODE3,
	SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE4,
	SHORT_C_BATT_STATUS__DYNAMIC_ERR_CODE5,
}OPPO_CHG_SHORT_BATTERY_STATUS;

typedef enum
{
	SHORT_C_BATT_SW_STATUS__OFF = 0,
	SHORT_C_BATT_SW_STATUS__ON,
}OPPO_CHG_SHORT_BATTERY_SWITCH_STATUS;

typedef enum
{
	SHORT_C_BATT_FEATURE_STATUS__OFF = 0,
	SHORT_C_BATT_FEATURE_STATUS__ON,
}OPPO_CHG_SHORT_BATTERY_FEATURE_STATUS;

struct short_c_batt_item{
	char *name;
	int value;
};

int oppo_short_c_batt_err_code_init(void);
int oppo_short_c_batt_chg_switch_init(void);
int oppo_short_c_batt_feature_status_init(void);
bool oppo_short_c_batt_is_prohibit_chg(struct oppo_chg_chip *chip);
bool oppo_short_c_batt_is_disable_rechg(struct oppo_chg_chip *chip);
bool oppo_short_c_batt_get_cv_status(struct oppo_chg_chip *chip);
void oppo_short_c_batt_update_change(struct oppo_chg_chip *chip, int update_value);
void oppo_chg_short_c_battery_check(struct oppo_chg_chip *chip);

#endif /* _OPPO_BATTERY_H_ */