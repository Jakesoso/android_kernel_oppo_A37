/************************************************************************************
** File: - mediatek\source\kernel\drivers\oppo_devices_list\oppo_devices_list.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      driver for get devices list
** 
** Version: 1.0
** Date created: 10:38:47,18/06/2012
** Author: Yixue.Ge@ProDrv.BL
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** 
************************************************************************************/


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
//#include <mach/mt6577_gpio.h>
#include <linux/platform_device.h>
#include <linux/oppo_devices_list.h>
#include <mach/mt_gpio.h> 
#include <soc/oppo/oppo_project.h>
volatile LCD_DEV lcd_dev = LCD_NONE;
volatile TP_DEV oppo_tp_dev = TP_NONE;
volatile int KEY_FW = 0;
volatile int TP_FW = 0;
volatile int TP_FW_CONFIG_ID= 0;
// lingjianing@CameraDrv, 2013/10/28, add for 13065 firmware version	
volatile int MS2R_FW = 0;
volatile OPPO_BKL_DEV oppo_bkl_dev = BKL_NONE;
volatile CAMERA_BACK_DEV camera_back_dev = CAMERA_BACK_NONE;
volatile CAMERA_FRONT_DEV camera_front_dev = CAMERA_FRONT_NONE;
volatile ALSPS_DEV alsps_dev = ALSPS_NONE;
volatile GYRO_DEV gyro_dev = GYRO_NONE;

#ifdef VENDOR_EDIT
//Fuchun.Liao@Mobile.BSP.CHG 2016-01-26 add for charger device_list
volatile CHARGER_DEV charger_dev = CHARGER_NONE;
volatile BMS_DEV bms_dev = BMS_NONE;
volatile VOOC_DEV vooc_dev = VOOC_NONE;
#endif

static ssize_t oppo_devices_list_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	/*not support write now*/
	return count;
}

static ssize_t oppo_devices_list_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	char page[512]; 
	char *p = page;
	int len = 0; 
	unsigned char id[15]={0};
	printk("oppo_devices_list_read is called\n");

	/*************************add here for customer devices list***************/
    switch(lcd_dev)
    {
        case LCD_NONE:
            p += sprintf(p, "LCD :None\n");
            break;
        case LCD_HITACHI:
            p += sprintf(p, "LCD :Hitachi\n");
            break;
        case LCD_TRULY:
            p += sprintf(p, "LCD :Truly\n");
            break;
        case LCD_BYD:
            p += sprintf(p, "LCD :BYD\n");
            break;		
	     case LCD_JDI:
            p += sprintf(p, "LCD :JDI\n");
            break;	
        case LCD_HITACHI_VIDEO:
            p += sprintf(p, "LCD :Hitachi_video\n");
            break;
		 case LCD_TRULY_OLD:
            p += sprintf(p, "LCD :TRULY_OLD\n");
            break;	
	    case LCD_TRULY_NEW:
            p += sprintf(p, "LCD :TRULY_NEW\n");
			break;
		case LCD_NOVTEK:
            p += sprintf(p, "LCD :LCD_NOVTEK\n");		
            break;
		case LCD_TIANMA:
            p += sprintf(p, "LCD :LCD_TIANMA\n");
			break;			
        case LCD_TRULY_COMMAND:
            p += sprintf(p, "LCD :TRULY_COMMAND\n");
			break;
		case LCD_TRULY_VIDEO:
            p += sprintf(p, "LCD :TRULY_VIDEO\n");
			break;
        case LCD_BYD_COMMAND:
            p += sprintf(p, "LCD :BYD_COMMAND\n");
			break;
		case LCD_BYD_VIDEO:
            p += sprintf(p, "LCD :BYD_VIDEO\n");
            break;
		case LCD_BYD_JDI_NT35521:
            p += sprintf(p, "LCD :BYD_JDI_NT35521\n");		
            break;	
		case LCD_TRULY_LG_NT35521:
            p += sprintf(p, "LCD :TRULY_LG_NT35521\n");		
            break;	
		case LCD_TRULY_LG_HX8394:
            p += sprintf(p, "LCD :TRULY_LG_HX8394\n");		
            break;	
		case LCD_TRULY_JDI_NT35521:
            p += sprintf(p, "LCD :TRULY_JDI_NT35521\n");
        /* Xinqin.Yang@PhoneSW.Multimedia, 2014/09/20  Add for 14053 JDI panel begin*/
            break;
        case LCD_JDI_R63417_VIDEO:
            p += sprintf(p, "LCD :JDI_R63417_VIDEO\n");
            break;
        /* Xinqin.Yang@PhoneSW.Multimedia, 2014/09/20  Add for 14053 JDI panel end*/
        /* Xinqin.Yang@PhoneSW.Multimedia, 2015/02/08  Add for 15011 SAMSUNG panel begin*/
        case LCD_SAMSUNG_S6E3FA3_CMD:
            p += sprintf(p, "LCD :LCD_SAMSUNG_S6E3FA3_CMD\n");
            break;
        /* Xinqin.Yang@PhoneSW.Multimedia, 2015/02/08  Add for 15011 SAMSUNG panel end*/
#ifdef VENDOR_EDIT
/* liuyan@Onlinerd.driver, 2014/11/28  Add for 14007 oled panel */
        case LCD_OLED_S6E3FA2_CMD:
            p += sprintf(p, "LCD :OLED_S6E3FA2_CMD\n");
#endif /*CONFIG_VENDOR_EDIT*/
            break;
#ifdef VENDOR_EDIT
		/* liping-m@PhoneSW.Multimedia, 2015/11/26  Add for oled panel video mode */
		case LCD_SAMSUNG_EA8064T_VIDEO:
			p += sprintf(p, "LCD :LCD_SAMSUNG_EA8064T_VIDEO\n");
		    break;
		case LCD_SAMSUNG_EA8064T_CMD:
			p += sprintf(p, "LCD :LCD_SAMSUNG_EA8064T_CMD\n");
			break;
#endif /*CONFIG_VENDOR_EDIT*/

#ifdef VENDOR_EDIT
		/* liping-m@PhoneSW.Multimedia, 2016/02/18  Add for add for BOE,TRULY,tianma new panel */
		case LCD_BOE_ILI9881C_VIDEO:
			p += sprintf(p, "LCD :LCD_BOE_ILI9881C_VIDEO\n");
			break;
		case LCD_TRULY_NT35521S_VIDEO:
			p += sprintf(p, "LCD :LCD_TRULY_NT35521S_VIDEO\n");
			break;
		case LCD_TIANMA_NT35521S_VIDEO:
			p += sprintf(p, "LCD :LCD_TIANMA_NT35521S_VIDEO\n");
			break;			
#endif /*CONFIG_VENDOR_EDIT*/
		default:
            p += sprintf(p, "LCD :unknown\n");
            break;
    }
    
    switch(oppo_tp_dev)
    {
        case TP_NONE:
            p += sprintf(p, "TP  :None\n");
            break;
        case TP_ALPS:
            p += sprintf(p, "TP  :ALPS FW:0x%x\n",TP_FW);
            break;
        case TP_TRULY:
            p += sprintf(p, "TP  :TRULY 0x%x\n",TP_FW);
            break;
		case TP_YOUNGFAST:		
            p += sprintf(p, "TP  :YOUNGFAST 0x%x\n",TP_FW);
			break;
		case TP_OFILM:		
            p += sprintf(p, "TP  :OFILM 0x%x\n",TP_FW);		
			break;
		case TP_TPK:		
            p += sprintf(p, "TP  :TPK 0x%x\n",TP_FW);	
            break;
		case TP_NITTO:
            p += sprintf(p, "TP  :NITTO 0x%x\n",TP_FW);
            break;
        case TP_OIKE:
            p += sprintf(p, "TP  :OIKE 0x%x\n",TP_FW);
            break;	
		case TP_JDI:
            p += sprintf(p, "TP  :JDI 0x%x\n",TP_FW);
            break;		
		case TP_OFILM_WHITE:
            p += sprintf(p, "TP  :OFILM_WHITE 0x%x\n",TP_FW);
			break;
		case TP_OFILM_BLACK:
            p += sprintf(p, "TP  :OFILM_BLACK 0x%x\n",TP_FW);
			break;
		case TP_TPK_WHITE:
            p += sprintf(p, "TP  :TPK_WHITE 0x%x\n",TP_FW);
			break;
		case TP_TPK_BLACK:
            p += sprintf(p, "TP  :TPK_BLACK 0x%x\n",TP_FW);
			break;	
		case TP_TPK_GOODIX:
            p += sprintf(p, "TP  :TP_TPK_GOODIX Base_ID=0x%x Config_ID=0x%x \n",TP_FW,TP_FW_CONFIG_ID);
			break;	
		case TP_TRULY_GOODIX:
            p += sprintf(p, "TP  :TP_TRULY_GOODIX Base_ID=0x%x Config_ID=0x%x \n",TP_FW,TP_FW_CONFIG_ID);
			break;
		case TP_TPK_SYNAPTICS:
            p += sprintf(p, "TP  :TP_TPK_SYNAPTICS 0x%x\n",TP_FW);
			break;
		case TP_TRULY_SYNAPTICS:
            p += sprintf(p, "TP  :TP_TRULY_SYNAPTICS 0x%x\n",TP_FW);
			break;		
		case TP_SAMSUNG_SYNAPTICS:
            p += sprintf(p, "TP  :TP_SAMSUNG_SYNAPTICS 0x%x \t 0x%x\n",TP_FW, KEY_FW);
			break;		
        default:
            p += sprintf(p, "TP  :unknown\n");
            break;
    }
    
    switch(camera_back_dev)
    {
        case CAMERA_BACK_NONE:
            p += sprintf(p, "Cam_b:None\n");
            break;
        case CAMERA_BACK_OV5650MIPI:
            p += sprintf(p, "Cam_b:OV5650MIPI\n");
            break;
        case CAMERA_BACK_OV5647:
            p += sprintf(p, "Cam_b:OV5647\n");
            break;
	 	case CAMERA_BACK_OV5647AC:
            p += sprintf(p, "Cam_b:OV5647AC\n");
            break;
        case CAMERA_BACK_S5K4E5YA:
            p += sprintf(p, "Cam_b:S5K4E5YA\n");
            break;
        case CAMERA_BACK_IMX105MIPI:
            p += sprintf(p, "Cam_b:IMX105MIPI\n");
            break;
	// lingjianing@CameraDrv, 2013/10/28, add for 13065 test mode and get firmware version	
	  case CAMERA_BACK_MS2R: 
            p += sprintf(p, "Cam_b  :MS2RMIPI 0x%x\n",MS2R_FW);
            break;
        case CAMERA_BACK_VD6803A:
		    p += sprintf(p,"Cam_b:VD6803A\n");
		    break;
		//zhengrong.zhang@CameraDrv, 2013/09/24, add for 13059 test mode
    	case CAMERA_BACK_IMX179:
		    p += sprintf(p,"Cam_b:IMX179\n");
		    break;
		//bin.liu@CameraDrv, 2014/11/18, add for 14053 test mode
		case CAMERA_BACK_IMX214:
		    p += sprintf(p,"Cam_b:IMX214\n");
		    break;
		/*oppo hufeng 20150515 add for device list*/
		case CAMERA_BACK_IMX278:
		    p += sprintf(p,"Cam_b:IMX278\n");
		    break;
		//xianglie.liu@CameraDrv, 2013/12/24, add for 13085 factory mode
    	case CAMERA_BACK_OV5648:
		    p += sprintf(p,"Cam_b:OV5648\n");
		    break;
    	//zhangkw add for 15011 factory mode			
    	case CAMERA_BACK_S5K3M2:
		    p += sprintf(p,"Cam_b:S5K3M2\n");
		    break;	
    	//hufeng add for 15111 factory mode			
    	case CAMERA_BACK_S5K3L8:
		    p += sprintf(p,"Cam_b:S5K3L8\n");
		    break;	
        default:
            p += sprintf(p, "Cam_b:unknown\n");
            break;
    }
    
    switch(camera_front_dev)
    {
        case CAMERA_FRONT_NONE:
            p += sprintf(p, "Cam_f:None\n");
            break;
        case CAMERA_FRONT_mt9d115:
            p += sprintf(p, "Cam_f:mt9d115\n");
            break;
        case CAMERA_FRONT_s5k5bbgx:
            p += sprintf(p, "Cam_f:s5k5bbgx\n");
            break;
		//lvxj@MutimediaDrv.camsensor, 2012/08/31, add for 12021 test mode	
        case CAMERA_FRONT_ov7675:
            p += sprintf(p, "Cam_f:ov7675\n");
            break;
        case CAMERA_FRONT_hi704:
            p += sprintf(p, "Cam_f:hi704\n");
            break;
        case CAMERA_FRONT_HI253:
            p += sprintf(p, "Cam_f:HI253\n");
            break;
		//zhengrong.zhang@CameraDrv, 2013/09/24, add for 13059 test mode
		case CAMERA_FRONT_HI256:
            p += sprintf(p, "Cam_f:HI256\n");
            break;
        case CAMERA_FRONT_OV5647:
            p += sprintf(p, "Cam_f:OV5647\n");
            break;
		//zhangkw@CameraDrv, 2013/06/08, add for test mode 13003		
	 	case CAMERA_FRONT_OV5693:
            p += sprintf(p, "Cam_f:OV5693\n");
            break;
	// lingjianing@CameraDrv, 2013/10/28, add for 13065 test mode	
	case CAMERA_FRONT_OV5648:  //=====
            p += sprintf(p, "Cam_f:OV5648\n");
            break;
	// zhangkw add factory mode for 15011
	case CAMERA_FRONT_OV8858: 
            p += sprintf(p, "Cam_f:OV8858\n");
            break;
    	//hufeng add for 15111 factory mode			
    	case CAMERA_BACK_S5K3P3SX:
		    p += sprintf(p,"Cam_b:S5K3P3SX\n");
		    break;
        default:
            p += sprintf(p, "Cam_f:unknown\n");
            break;
    }

/*for als ps dev*/
	switch(alsps_dev)
	{
		case ALSPS_NONE:
			p += sprintf(p, "ALS_PS:none\n");
			break;
		case ALSPS_STK31XX:
			p += sprintf(p, "ALS_PS:stk31xx\n");
			break;
		case ALSPS_STK3X1X:
			p += sprintf(p, "ALS_PS:stk3x1x\n");
			break;
		case ALSPS_TAOS_277X:
			p += sprintf(p, "ALS_PS:tmd277x\n");
			break;
		case ALSPS_LITE_558:
			p += sprintf(p, "ALS_PS:ltr558\n");
			break;
		case ALSPS_GP2AAP052A:
			p += sprintf(p, "ALS_PS:gp2ap052a\n");
			break;	
		case ALSPS_TAOS_TMG399X:
			p += sprintf(p, "ALS_PS:tmg399x\n");
			break;	
		case ALSPS_TMD_27723:
			p += sprintf(p, "ALS_PS:tmd27723\n");
			break;
		case ALSPS_CM_36686:
			p += sprintf(p, "ALS_PS:cm36686\n");
			break;
		case ALSPS_APDS_9921:
			p += sprintf(p, "ALS_PS:apds9921\n");
			break;
		case ALSPS_APDS_9922:
			p += sprintf(p, "ALS_PS:apds9922\n");
			break;
		default :
			p += sprintf(p, "ALS_PS:unknown\n");

	}

	switch(gyro_dev)
	{
		case GYRO_NONE:
			p += sprintf(p, "GYRO:None\n");
			break;
		case GYRO_MISS:
			p += sprintf(p, "GYRO:Miss\n");
			break;			
		case GYRO_MPU6050C:
			p += sprintf(p, "GYRO:MPU6050C\n");
			break;
		case GYRO_L3GD20:
			p += sprintf(p, "GYRO:L3GD20\n");
			break;
		case GYRO_LSM6DS3:
			p += sprintf(p, "GYRO:LSM6DS3\n");
			break;
		case GYRO_MMC_PG:
			p += sprintf(p, "GYRO:MMC_PG\n");
			break;			
		default :
			p += sprintf(p, "GYRO:Unknown\n");
	}

	switch(oppo_bkl_dev)
	{
		case BKL_NONE:
			p += sprintf(p, "back light:None\n");
			break;
		case BKL_LM3580:
			p += sprintf(p, "back light:LM3580\n");
			break;			
		case BKL_LM3630:
			p += sprintf(p, "back light:LM3630\n");
			break;
		default :
			p += sprintf(p, "back light:Unknown\n");
	}

	switch(charger_dev)
	{
		case CHARGER_NONE:
			p += sprintf(p, "charger:None\n");
			break;
		case CHARGER_BQ24196:
			p += sprintf(p, "charger:bq24196\n");
			break;			
		default :
			p += sprintf(p, "charger:Unknown\n");
	}

	switch(bms_dev)
	{
		case BMS_NONE:
			p += sprintf(p, "bms:None\n");
			break;
		case BMS_BQ27541:
			p += sprintf(p, "bms:bq27541\n");
			break;
		case BMS_BQ27411:
			p += sprintf(p, "bms:bq27411\n");
			break;	
		default :
			p += sprintf(p, "bms:Unknown\n");
	}

	switch(vooc_dev)
	{
		case VOOC_NONE:
			p += sprintf(p, "vooc:None\n");
			break;
		case VOOC_STM8S:
			p += sprintf(p, "vooc:stm8s\n");
			break;
		case VOOC_PIC16F:
			p += sprintf(p, "vooc:pic16f\n");
			break;	
		default :
			p += sprintf(p, "vooc:Unknown\n");
	}

	/*****************************add end**************************************/
	
	len = p - page;
	if (len > *pos)
		len -= *pos;
	else
		len = 0;
	if (copy_to_user(buf,page,len < count ? len  : count))
		return -EFAULT;
	*pos = *pos + (len < count ? len  : count);
	return len < count ? len  : count;

}

static const struct file_operations oppo_devices_list = {
	.write		= oppo_devices_list_write,
	.read		= oppo_devices_list_read,
};

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG, 2015/12/31, Add jsut for 15113
#define OPPO_SMALLBOARD_ID1  		GPIO16
#endif /*VENDOR_EDIT*/

static int oppo_smallboard = 0;


static ssize_t oppo_smallboard_id_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{

    char temp_buffer[2];
	int num_read_chars = 0;
	
    num_read_chars += sprintf(temp_buffer, "%d\n",oppo_smallboard);
	num_read_chars = simple_read_from_buffer(buf, count, pos, temp_buffer, strlen(temp_buffer));
	return num_read_chars; 
}


static ssize_t oppo_smallboard_id_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	/*not support write now*/
	return count;
}
static const struct file_operations oppo_smallboard_id = {
	.write		= oppo_smallboard_id_write,
	.read		= oppo_smallboard_id_read,
};


int operator_hw_version = OPERATOR_UNKOWN;
static void operator_version_check(void)
{	
	if(is_project(OPPO_15127)||is_project(OPPO_15129)||is_project(OPPO_15130)||is_project(OPPO_15131)||is_project(OPPO_15133)||is_project(OPPO_15134))
		return ;
	if(is_project(OPPO_15114))
		operator_hw_version = OPERATOR_ALL_MOBILE_CARRIER;//114
	else if(is_project(OPPO_15113))
		operator_hw_version = OPERATOR_ALL_TELECOM_CARRIER;//113
	else if(is_project(OPPO_16034))
		operator_hw_version = OPERATOR_ALL_PROJECT_16034;//16034 
	else if(is_project(OPPO_16031))
		operator_hw_version = OPERATOR_ALL_PROJECT_16031;
	else if(is_project(OPPO_15111))
		operator_hw_version = OPERATOR_OPEN_MARKET;
	else
		operator_hw_version = OPERATOR_UNKOWN;
	
	printk("kong operator_version_check,operator_hw_version = %d\n",operator_hw_version);
}

static ssize_t operatorName_read_proc(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;
	if(is_project(OPPO_15127)||is_project(OPPO_15129)||is_project(OPPO_15130)||is_project(OPPO_15131))
		len = sprintf(page,"%d",get_Operator_Version());
	else
		len = sprintf(page,"%d",operator_hw_version);
	printk("operatorName_read_proc,operator_hw_version = %d\n",operator_hw_version);

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

struct file_operations operatorName_proc_fops = {
	.read = operatorName_read_proc,
};

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);
static void smallboard_version_check(void)
{
	switch(get_project()) {
		case OPPO_15127:
		case OPPO_15130:	
		case OPPO_15129:
		{
			int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;
			int times = 5,channel = 14;
			int oppo_smallboard_ver = 100;
			i = times;
			while (i--) {
				 ret_value = IMM_GetOneChannelValue(channel, data, &ret_temp);
				 ret += ret_temp;
				 printk("[smallboard_version_check(channel%d)]: ret_temp=%d, ret = %d\n",channel,ret_temp,ret);
			}
			ret = ret * 1500 / 4096;
			ret = ret / times;
			if(((ret >=1050)&&(ret <= 1190)) || (ret >= 1300)) {//15127&&15128&&15130
				oppo_smallboard_ver = SMALLBOARD_VERSION__1;
				if(is_project(OPPO_15127)||is_project(OPPO_15130))
					oppo_smallboard = 1;
			} else if((ret >=870)&&(ret <= 1010))//reserve
				oppo_smallboard_ver = SMALLBOARD_VERSION__3;
			else if((ret >=700)&&(ret <= 840))//overseas2
				oppo_smallboard_ver = SMALLBOARD_VERSION__0;
			else if((ret >=530)&&(ret <= 670))//reserve
				oppo_smallboard_ver = SMALLBOARD_VERSION__4;
			else if((ret >=320)&&(ret <= 460))//overseas1
				oppo_smallboard_ver = SMALLBOARD_VERSION__0;
			else if((ret >=120)&&(ret <= 260)) {//15129
				oppo_smallboard_ver = SMALLBOARD_VERSION__2;
				if(is_project(OPPO_15129))
					oppo_smallboard = 1;
			}else
				oppo_smallboard_ver = SMALLBOARD_VERSION__UNKNOWN;			
			printk("smallboard_version_check oppo_smallboard_ver=%d oppo_smallboard= %d\n",oppo_smallboard_ver,oppo_smallboard);	
			break;	
		}
		case OPPO_15131:
		case OPPO_15133:
		case OPPO_15134:
		{
			int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;
			int times = 5,channel = 14;
			int oppo_smallboard_ver = 100;
			i = times;
			while (i--) {
				 ret_value = IMM_GetOneChannelValue(channel, data, &ret_temp);
				 ret += ret_temp;
				 printk("[smallboard_version_check(channel%d)]: ret_temp=%d, ret = %d\n",channel,ret_temp,ret);
			}
			ret = ret * 1500 / 4096;
			ret = ret / times;
			if((ret >=1050)&&(ret <= 1190)) {//15127&&15128&&15130
				oppo_smallboard_ver = SMALLBOARD_VERSION__1;
				if(is_project(OPPO_15131)||is_project(OPPO_15133))
					oppo_smallboard = 1;
			} else if((ret >=320)&&(ret <= 460)) {//15129
				oppo_smallboard_ver = SMALLBOARD_VERSION__2;
				if(is_project(OPPO_15134))
					oppo_smallboard = 1;
			}else
				oppo_smallboard_ver = SMALLBOARD_VERSION__UNKNOWN;			
			printk("smallboard_version_check oppo_smallboard_ver=%d oppo_smallboard= %d\n",oppo_smallboard_ver,oppo_smallboard);	
			break;	
		}
		case OPPO_16021:
		case OPPO_16023:
		case OPPO_16024:
		{
			int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;
			int times = 5,channel = 14;
			int oppo_smallboard_ver = 100;
			i = times;
			while (i--) {
				 ret_value = IMM_GetOneChannelValue(channel, data, &ret_temp);
				 ret += ret_temp;
				 printk("[smallboard_version_check(channel%d)]: ret_temp=%d, ret = %d\n",channel,ret_temp,ret);
			}
			ret = ret * 1500 / 4096;
			ret = ret / times;
			if((ret >=320)&&(ret <= 460)) {//16021,16023,16024
				oppo_smallboard_ver = SMALLBOARD_VERSION__1;
				oppo_smallboard = 1;
			} else
				oppo_smallboard_ver = SMALLBOARD_VERSION__UNKNOWN;			
			printk("smallboard_version_check oppo_smallboard_ver=%d oppo_smallboard= %d\n",oppo_smallboard_ver,oppo_smallboard);	
			break;	
		}
		default:
		{
			int id0 = 0;
			id0 = mt_get_gpio_in(OPPO_SMALLBOARD_ID1);
			//id1 = mt_get_gpio_in(GPIO_SUB_HW_ID1_PIN);
			
			printk("smallboard_version_check id1 = %d\n",id0);
			if(id0 == 1)
				oppo_smallboard = SMALLBOARD_VERSION__1;//OPPO_SMALLBOARD_DOMESTINC;
			else
				oppo_smallboard = SMALLBOARD_VERSION__0;//OPPO_SMALLBOARD_OVERSEAS;	
			break;
		}
	}
	printk("smallboard_version_check oppo_smallboard = %d\n",oppo_smallboard);
}

static int oppo_dev_platform_probe(struct platform_device *pdev)
{
	smallboard_version_check();
	operator_version_check();
	proc_create("oppo_devices_list", 0666, NULL, &oppo_devices_list);
	proc_create("oppo_smallboard_id", 0666, NULL, &oppo_smallboard_id);
   	proc_create("operatorName", 0666, NULL, &operatorName_proc_fops);

    return 0;
}

static struct platform_driver oppo_dev_platform_driver = {
    //.remove     = NULL,
    //.shutdown   = NULL,
    .probe      = oppo_dev_platform_probe,
    //#ifndef CONFIG_HAS_EARLYSUSPEND
    //.suspend    = NULL,
    .resume     = NULL,
   // #endif
    .driver     = {
    	.owner = THIS_MODULE,
        .name = "oppo_dev_platform_driver",
    },
};
static struct platform_device oppo_dev_platform_device = {
	.name = "oppo_dev_platform_driver",
	.id = -1
};

static int __init oppo_devices_list_init(void)
{
    int ret;
	ret = platform_device_register(&oppo_dev_platform_device);
	if (ret)
		printk("oppo_dev_platform_device:dev:E%d\n", ret);

	ret = platform_driver_register(&oppo_dev_platform_driver);

	if (ret)
	{
		printk("oppo_dev_platform_driver:drv:E%d\n", ret);
		platform_device_unregister(&oppo_dev_platform_device);
		return ret;
	}
	return 0;
}
static void __exit oppo_devices_list_exit(void)
{
	platform_driver_unregister(&oppo_dev_platform_driver);
	platform_device_unregister(&oppo_dev_platform_device);
}
module_init(oppo_devices_list_init);

