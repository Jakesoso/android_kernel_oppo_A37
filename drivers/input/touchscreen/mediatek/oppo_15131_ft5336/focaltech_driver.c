/*Chenggang.Li@BSP TP modify driver for oppo 2016.05.18 solve compile warning and cut glove mode*/
#include"ft_gesture_lib.h"
#include <../../drivers/input/touchscreen/mediatek/tpd.h>
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <soc/oppo/device_info.h>
#include "tpd_custom_fts.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/pmic_api_ldo.h>

#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#include "cust_gpio_usage.h"

#include <linux/dma-mapping.h>
#include <linux/oppo_devices_list.h>


#define SUPPORT_GESTURE
#define SUPPORT_REPORT_COORDINATE


#define FTS_GESTRUE
#define TPD_AUTO_UPGRADE	// if need upgrade CTP FW when POWER ON,pls enable this MACRO

#define FTS_CTL_IIC
#define SYSFS_DEBUG
#define FTS_APK_DEBUG

#define GPIO_CTP_RST_PIN (GPIO10 | 0x80000000)


#define GPIO_ID1 GPIO21 | 0x80000000
#define GPIO_ID2 GPIO19 | 0x80000000


#define GPIO_DIR_OUT 1
#define GPIO_DIR_IN 0

extern  TP_DEV tp_dev;

#define OFLM_TP 0
#define TRULY_TP 1
#define BIEL_TP 2
#define TRULY_NT_TP 3

static struct proc_dir_entry *prEntry_tp = NULL; 
static struct proc_dir_entry *prEntry_dtap = NULL;
static struct proc_dir_entry *prEntry_coodinate  = NULL; 
static atomic_t glove_enable;
static atomic_t double_enable;

static uint32_t gesture;
static int boot_mode;
static int init_fts_proc(void);

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int IMM_IsAdcInitReady(void);


#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

//#ifdef SYSFS_DEBUG
#include "focaltech_ex_fun.h"
//#endif


//...................chenggang.li add for dri code base on 15066....start
DEFINE_MUTEX(ft_suspend_lock);
struct device *focal_dev;
static uint32_t gesture_upload;
#define MT_PROTOCOL_B
#define DEBUG_TP_CHARGE
#define DEBUG_TP_RAWDATA
#define TPD_ERR(a, arg...)  pr_err(TPD_DEVICE ": " a, ##arg)
extern ssize_t ftxxxx_ftslibtest_sample(struct i2c_client *client);
static int usb_check_state =0;


//...................chenggang.li add for dri code base on 15066....end


#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#ifdef TPD_PROXIMITY
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_flag_one = 0; //add for tpd_proximity by wangdongfang
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
#endif

#ifdef FTS_GESTRUE
/*
#define  KEY_GESTURE_U KEY_U
#define  KEY_GESTURE_UP KEY_UP
#define  KEY_GESTURE_DOWN KEY_DOWN
#define  KEY_GESTURE_LEFT KEY_LEFT 
#define  KEY_GESTURE_RIGHT KEY_RIGHT
#define  KEY_GESTURE_O KEY_O
#define  KEY_GESTURE_E KEY_E
#define  KEY_GESTURE_M KEY_M 
#define  KEY_GESTURE_L KEY_L
#define  KEY_GESTURE_W KEY_W
#define  KEY_GESTURE_S KEY_S 
#define  KEY_GESTURE_V KEY_V
#define  KEY_GESTURE_Z KEY_Z
*/
#define UnkownGestrue       0
#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define LeftVee             4   // >
#define RightVee            5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W


#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_DOUBLELINE	0x25

#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_LEFT_V		0x51
#define GESTURE_RIGHT_V		0x52
#define GESTURE_DOWN_V		0x54
#define GESTURE_V		    0x53
//#define GESTURE_Z		    0x41
#define GESTURE_cw_O		0x57 // Clockwise O

#include "ft_gesture_lib.h"

#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4
#define SPEC_GESTURE_COORD_COMPRESS_ENABLED

unsigned int touch_irq = 0;
static void focaltech_irq_init(void);
unsigned short coordinate_x[256] = {0};
unsigned short coordinate_y[256] = {0};
#endif


unsigned short coordinate_doblelion_1_x[256] = {0};
unsigned short coordinate_doblelion_2_x[256] = {0};
unsigned short coordinate_doblelion_1_y[256] = {0};
unsigned short coordinate_doblelion_2_y[256] = {0};
unsigned short coordinate_report[14] = {0};
extern struct tpd_device *tpd;
static DEFINE_MUTEX(i2c_rw_access);
struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;

struct Upgrade_Info fts_updateinfo[] =
{
	{0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x54,"FT5x46",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 10, 2000},
};
				
struct Upgrade_Info fts_updateinfo_curr;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
void choose_tp_fw(int tp_dev);
int fts_ctpm_auto_upgrade(struct i2c_client *client);
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc);

extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern u8 fts_ctpm_get_i_file_tpid(void);
extern u8 fts_ctpm_get_i_file_ver(void);
extern int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
static int  tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int  tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static int tpd_flag = 0;
static int tpd_halt=0;

static int limit_enable = 1;

#ifdef VENDOR_EDIT
	static int tp_suspend = 0;
#endif

#ifdef VENDOR_EDIT
	/*mofei@EXP.BaseDrv.charge, 2015/12/01, add for checking usb connecting state in the charging module */
	int g_dma_alloct = 0;
#endif

#define __MSG_DMA_MODE__  //
#ifdef __MSG_DMA_MODE__
	u8 *g_dma_buff_va = NULL;    //
	dma_addr_t g_dma_buff_pa = 0;    // 
#endif

#ifdef __MSG_DMA_MODE__
static void msg_dma_alloct(void){
	g_dma_buff_va = (u8 *)dma_alloc_coherent(&focal_dev, 4096, &g_dma_buff_pa, GFP_KERNEL);
    if(!g_dma_buff_va){
        TPD_ERR("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
    }	
}

static void msg_dma_release(void){
	if(g_dma_buff_va){
     	dma_free_coherent(&focal_dev, 4096, g_dma_buff_va, g_dma_buff_pa);
        g_dma_buff_va = NULL;
        g_dma_buff_pa = 0;
		TPD_ERR("[DMA][release] Allocate DMA I2C Buffer release!\n");
    }
}
#endif


#define TPD_OK 0
#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02
#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06
#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C
#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12


#define TPD_RESET_ISSUE_WORKAROUND
#define TPD_MAX_RESET_COUNT 3


struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};


#ifdef TPD_HAVE_BUTTON 
	static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
	static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
	static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	static int tpd_calmat_local[8] = TPD_CALIBRATION_MATRIX;
	static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

//#define VELOCITY_CUSTOM_fts
#ifdef VELOCITY_CUSTOM_fts
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity

#ifndef TPD_VELOCITY_CUSTOM_X
	#define TPD_VELOCITY_CUSTOM_X 10
#endif

#ifndef TPD_VELOCITY_CUSTOM_Y
	#define TPD_VELOCITY_CUSTOM_Y 10
#endif

#define TOUCH_IOC_MAGIC 'A'
#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;

static int tpd_misc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int tpd_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{

	void __user *data;
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;
			
		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}
	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[10];
    int x[10];
    int p[10];
    int id[10];
    int count;
};
 
static const struct i2c_device_id fts_tpd_id[] = {{"mtk-tpd",0},{}};
static struct i2c_board_info __initdata fts_i2c_tpd={ I2C_BOARD_INFO("mtk-tpd", (0x70>>1))};
 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	  .name = "mtk-tpd",
	  .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = tpd_remove,
  .id_table = fts_tpd_id,
 };
 

int fts_i2c_Read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen)
{
	int ret=-1;
	
	mutex_lock(&i2c_rw_access);
	if(writelen!=0)
	{
		//DMA Write
		memcpy(g_dma_buff_va, writebuf, writelen);
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}
	
	if(readlen!=0)
	{
		//DMA Read 
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);
		memcpy(readbuf, g_dma_buff_va, readlen);
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);
	}
	mutex_unlock(&i2c_rw_access);
	return ret;
}

/*write data by i2c*/
int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	
	mutex_lock(&i2c_rw_access);
	memcpy(g_dma_buff_va, writebuf, writelen);
	client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
	if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
	client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);	
	mutex_unlock(&i2c_rw_access);
	return ret;
}

int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;
	return fts_i2c_Write(client, buf, sizeof(buf));
}

int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return fts_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

static void focaltech_irq_init(void)
{
	int ret = 0;
	struct device_node *node = NULL;
	u32 intr[2] = {0,0};
	  
	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if(node){
		of_property_read_u32_array(node , "interrupts", intr, ARRAY_SIZE(intr));
		pr_err("chenggang_fts_device_irq_init intr[0] = %d, intr[1]  = %d\r\n",intr[0] ,intr[1] );
		touch_irq = irq_of_parse_and_map(node, 0);
	}
	else{
		pr_err("synaptics_device_irq_init node not exist!\r\n");
	}
	ret = request_irq(touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, EINTF_TRIGGER_FALLING, "TOUCH_PANEL-eint",  NULL);
	if(ret){
		TPD_ERR("ret = %d\n", ret);
		TPD_ERR("focaltech_probe: failed to request_irq \n");
		return ;
	}
	//enable_irq(touch_irq);
}

void focaltech_get_upgrade_array(void)
{

	u8 chip_id;
	u32 i;

	fts_read_reg(i2c_client, FTS_REG_CHIP_ID, &chip_id);
	chip_id =0x54;
	printk("%s chip_id = %x\n", __func__, chip_id);

	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}

/*
*report the point information
*/
static int fts_read_Touchdata(struct ts_event *pinfo)
{
    u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	if (tpd_halt)
	{
		TPD_ERR( ".....tpd_touchinfo return ....\n");
		return ret;
	}

	//mutex_lock(&i2c_access);
	ret = fts_i2c_Read(i2c_client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "%s read touchdata failed.\n",__func__);
		//mutex_unlock(&i2c_access);
		return ret;
	}
	//mutex_unlock(&i2c_access);
	memset(pinfo, 0, sizeof(struct ts_event));
	
	pinfo->touch_point = 0;
	//printk("TPD_MAX_POINTS=%d chipID=%d\n", fts_updateinfo_curr.TPD_MAX_POINTS,fts_updateinfo_curr.CHIP_ID);
	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)
	{
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			continue;
		else
			pinfo->touch_point++;
		pinfo->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		pinfo->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		pinfo->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		pinfo->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}
	return 0;
}


static int report_count=0;
static void fts_report_value(struct ts_event *data)
{
	 struct ts_event *event = data;
	 int i = 0;
	 int up_point = 0;
	 static int touch_major_report_num  = 5;
	 
	 if(event->touch_point>0){ 
		 for (i = 0; i < event->touch_point; i++)
		 {
			if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
			{
				report_count++;
				input_mt_slot(tpd->dev, event->au8_finger_id[i]);
				input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,1);
				input_report_key(tpd->dev, BTN_TOOL_FINGER, 1);
				input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR,touch_major_report_num);
				if((report_count%50)==0){
					touch_major_report_num++;
					input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR,touch_major_report_num);
				}
				input_report_abs(tpd->dev, ABS_MT_POSITION_X,event->au16_x[i]);
				input_report_abs(tpd->dev, ABS_MT_POSITION_Y,event->au16_y[i]);
				if(report_count==150){
					TPD_ERR("tpd->dev::tpd_down_B:: x[%d],y[%d]=[%d   %d]\n",i,i,event->au16_x[i],event->au16_y[i]);
					report_count=0;
				}
			}else{
				up_point++;
				input_mt_slot(tpd->dev, event->au8_finger_id[i]);
				input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,0);
			}				 
		}
		
	}
	 
	 if(event->touch_point == up_point){		 
	 	input_report_key(tpd->dev, BTN_TOUCH, 0);
		touch_major_report_num=5;
	 }else{		
	 	input_report_key(tpd->dev, BTN_TOUCH, 1);
	 }
 input_sync(tpd->dev);

}


#ifdef TPD_PROXIMITY
int tpd_read_ps(void)
{
	tpd_proximity_detect;
	return 0;    
}

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;
	
	//i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
	fts_read_reg(i2c_client, 0xB0, &state);
	printk("[proxi_fts]read: 0xB0's value is 0x%02X\n", state);
	if (enable){
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is on\n");	
	}else{
		state &= 0x00;	
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is off\n");
	}
	//ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
	ret = fts_write_reg(i2c_client,0xb0,state);
	TPD_ERR("[proxi_fts]write: 0xb0's value is 0x%02X\n", state);
	return 0;
}

static int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,

		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{		
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
				}
			}
			break;
		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
				if((err = tpd_read_ps()))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = tpd_get_ps_value();
					TPD_ERR("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}					
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	return err;	
}
#endif

#ifdef FTS_GESTRUE
static void _get_coordinate(unsigned char *buf, int pointnum)
{
    int i;
	for(i = 0;i < pointnum;i++){
		coordinate_x[i] =  (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[4 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[5 + (4 * i)]) & 0xFF);
		printk("[focaltech]:pointx[%d] = %d,pointy[%d] = %d\n",i,coordinate_x[i],i,coordinate_y[i]);
	}    
}
#endif

#ifdef FTS_GESTRUE
//#ifdef SPEC_GESTURE_COORD_COMPRESS_ENABLED
#define UINT16 unsigned short
#define UINT8 unsigned char
static UINT16 usPointXMax;
static UINT16 usPointYMax;
static UINT16 usPointXMin;
static UINT16 usPointYMin;
static UINT16 usPointXMaxatY;
static UINT16 usPointYMinatX;
static UINT16 usPointXMinatY;
static UINT16 usPointYMaxatX;
#endif

#ifdef FTS_GESTRUE
static void SpecialGestureTraceMaxAndMin(UINT16 usCurPointX, UINT16 usCurPointY) 
{
    if (usCurPointX < usPointXMin)
    {
        usPointXMin = usCurPointX;
        usPointXMinatY= usCurPointY;      
    }

    if (usCurPointX > usPointXMax)
    {
        usPointXMax = usCurPointX;
        usPointXMaxatY= usCurPointY;  
    }

    if (usCurPointY < usPointYMin)
    {
        usPointYMin = usCurPointY;
        usPointYMinatX= usCurPointX;         
    }

    if (usCurPointY > usPointYMax)
    {
        usPointYMax = usCurPointY;
        usPointYMaxatX= usCurPointX;      
    }
}
#endif

#ifdef FTS_GESTRUE
static void SpecGestureTraceGet(UINT8 uiPointSum)
{
    UINT16 usCurPointX;
    UINT16 usCurPointY;
    UINT8 i = 0;

    usPointXMax = coordinate_x[0];
    usPointXMin = coordinate_x[0];
    usPointYMax = coordinate_y[0];
    usPointYMin = coordinate_y[0];

    for( i = 0; i< uiPointSum; i++)
    {
        usCurPointX = coordinate_x[i];
        usCurPointY = coordinate_y[i];
        SpecialGestureTraceMaxAndMin(usCurPointX,usCurPointY); 
    }
	printk("%s_usPointXMax =%d,usPointXMin =%d,usPointYMax=%d,usPointYMin =%d\n",__func__,usPointXMax,usPointXMin,usPointYMax,usPointYMin);
}
#endif

#ifdef SPEC_GESTURE_COORD_COMPRESS_ENABLED
static int _compress_coordinate(unsigned char *buf, int pointnum) // ??è|¨???|¨??|¨?????§oy
{
    int gestrue_id = buf[0];
    int retPointNum=pointnum;
    UINT16 uiTempPointSum = pointnum; // buf[1];
    
    usPointXMin = 0xFFFF;
    usPointYMin = 0xFFFF;
    usPointXMax = 0;
    usPointYMax = 0;
    
    SpecGestureTraceGet(pointnum);
    
    if((gestrue_id <= GESTURE_DOWN) && (gestrue_id >= GESTURE_LEFT) && pointnum>2)
    {
    // buf[1] = 2;
       retPointNum =2;
       coordinate_x[1] = coordinate_x[uiTempPointSum-1]; 
       coordinate_y[1] = coordinate_y[uiTempPointSum-1];
    }
    else if((gestrue_id==GESTURE_O)||(gestrue_id==GESTURE_cw_O))// if char is 'o',make up the PointNum of 6 
    {
   //   buf[1] = 6;
        retPointNum =6;
        coordinate_x[1] = usPointYMinatX; 
        coordinate_y[1] = usPointYMin ; 
        // Xmin
        coordinate_x[2] = usPointXMin; 
        coordinate_y[2] = usPointXMinatY; 
        // Ymax
        coordinate_x[3] = usPointYMaxatX; 
        coordinate_y[3] = usPointYMax ;
        // xmax
        coordinate_x[4] = usPointXMax; 
        coordinate_y[4] = usPointXMaxatY; 
        // end point
        coordinate_x[5] = coordinate_x[uiTempPointSum-1];  
        coordinate_y[5] = coordinate_y[uiTempPointSum-1]; 
    }
    else
    {
            if((gestrue_id>=GESTURE_LEFT_V)&&(gestrue_id<=GESTURE_DOWN_V) && pointnum!=3)
            {
            	#if 1
                    if(gestrue_id==GESTURE_LEFT_V) // '<'
                    {
                            coordinate_x[1] = usPointXMin;
                            coordinate_y[1] = usPointXMinatY;
                    }
                    else if(gestrue_id==GESTURE_RIGHT_V)// '>'
                    {
                            coordinate_x[1] = usPointXMax;
                            coordinate_y[1] = usPointXMaxatY;
                    }
                    else if(gestrue_id==GESTURE_V) // '^'
                    {
                            coordinate_x[1] = usPointYMinatX;
                            coordinate_y[1] = usPointYMin;
                    }
                    else if(gestrue_id==GESTURE_DOWN_V) // 'v'
                    {
                            coordinate_x[1] = usPointYMaxatX;
                            coordinate_y[1] = usPointYMax;
                    }
                    coordinate_x[2] = coordinate_x[uiTempPointSum-1]; // g_stSpecGesStatus
                    coordinate_y[2] = coordinate_y[uiTempPointSum-1]; // g_stSpecGesStatus
             //      buf[1] = 3;
             		retPointNum =3;
			 #endif
	      }
             else if(((gestrue_id==GESTURE_W)||(gestrue_id==GESTURE_M))&&(pointnum!=5)) //0x31: 'W'  ,0x32:'M'
            {
                //UINT8 i = 0;
               //UINT16  usCurPointY;
                UINT16  usinflectionPointNum =0;
               // UINT8  curYDirection=0xFF;
                //UINT8  usLastYDirection=0xFF;
                UINT16 stepX;
                UINT16 usStartPointX = usPointXMin;  
                UINT16 usEndPointX =  usPointXMax;   
                usinflectionPointNum=1;
                stepX = abs(usEndPointX-usStartPointX)/4;
                if(gestrue_id==GESTURE_W) //  'W'
                {
                    coordinate_x[0] = usStartPointX;
                    coordinate_y[0] = usPointYMin;
                    coordinate_x[1] = usStartPointX+stepX;
                    coordinate_y[1] = usPointYMax;
                    coordinate_x[2] = usStartPointX+2*stepX; 
                    coordinate_y[2] = usPointYMin;
                    coordinate_x[3] = usStartPointX+3*stepX;
                    coordinate_y[3] = usPointYMax;
                    coordinate_x[4] = usEndPointX; 
                    coordinate_y[4] = usPointYMin;
                }
                else  // 'M'
                {
                    coordinate_x[0] = usStartPointX; 
                    coordinate_y[0] = usPointYMax;
                    coordinate_x[1] = usStartPointX+stepX; 
                    coordinate_y[1] = usPointYMin;
                    coordinate_x[2] = usStartPointX+2*stepX; 
                    coordinate_y[2] = usPointYMax;
                    coordinate_x[3] = usStartPointX+3*stepX; 
                    coordinate_y[3] = usPointYMin;
                    coordinate_x[4] = usEndPointX; 
                    coordinate_y[4] = usPointYMax ; 
                }
        	//buf[1] = 5;
        	retPointNum =5;
        }
   }
   return retPointNum;
}
#endif

#ifdef FTS_GESTRUE
static void _get_coordinate_report(unsigned char *buf, int pointnum)
{
	int clk;
    int gestrue_id = buf[0];
    _get_coordinate(buf, pointnum);

#ifdef SPEC_GESTURE_COORD_COMPRESS_ENABLED
    pointnum=_compress_coordinate(buf, pointnum);
#endif

	if((gestrue_id != GESTURE_O)&&(gestrue_id!=GESTURE_cw_O)){	 
		coordinate_report[1] = coordinate_x[0];
		coordinate_report[2] = coordinate_y[0];
		coordinate_report[3] = coordinate_x[pointnum-1];
		coordinate_report[4] = coordinate_y[pointnum-1];
		coordinate_report[5] = coordinate_x[1];
		coordinate_report[6] = coordinate_y[1];
		coordinate_report[7] = coordinate_x[2];
		coordinate_report[8] = coordinate_y[2];
		coordinate_report[9] = coordinate_x[3];
		coordinate_report[10] = coordinate_y[3];
		coordinate_report[11] = coordinate_x[4];
		coordinate_report[12] = coordinate_y[4];
	}else{
		coordinate_report[1] = coordinate_x[0];
		coordinate_report[2] = coordinate_y[0];
		coordinate_report[3] = coordinate_x[pointnum-2];
		coordinate_report[4] = coordinate_y[pointnum-2];
		coordinate_report[5] = coordinate_x[1];
		coordinate_report[6] = coordinate_y[1];
		coordinate_report[7] = coordinate_x[2];
		coordinate_report[8] = coordinate_y[2];
		coordinate_report[9] = coordinate_x[3];
		coordinate_report[10] = coordinate_y[3];
		coordinate_report[11] = coordinate_x[4];
		coordinate_report[12] = coordinate_y[4];
		clk = coordinate_x[pointnum-1];
	}
 
    coordinate_report[13] = 2;
	
	if (gestrue_id==GESTURE_O){
    	coordinate_report[13]=0;
    }
	else if (gestrue_id==GESTURE_cw_O){
    	coordinate_report[13]=1;    
    }
    	
    
}
#endif

#ifdef FTS_GESTRUE
static int fts_read_Gestruedata(void)
{
    unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
    int ret = -1;
    int i = 0;
	u8 reg_addr = 0xd3;
    int gestrue_id = 0;
    short pointnum = 0;
	int state =0;
    buf[0] = 0xd3;

    //pointnum = 0;
    ret = fts_i2c_Read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	printk( "read gesture data..............START!\n");
    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }
	memset(coordinate_report, 0, sizeof(coordinate_report));
	if (fts_updateinfo_curr.CHIP_ID==0x54)
	{
		gestrue_id = buf[0];
		printk("read gesture data:gestrue_id =0x%x\n",gestrue_id);
		pointnum = (short)(buf[1]) & 0xff;	 
		if(gestrue_id != GESTURE_DOUBLELINE){	 
			if((pointnum * 4 + 2)<255){
				ret = fts_i2c_Read(i2c_client, &reg_addr, 1, buf, (pointnum * 4 + 2));
			}else{
				ret = fts_i2c_Read(i2c_client, &reg_addr, 1, buf, 255);
				ret = fts_i2c_Read(i2c_client, &reg_addr, 0, buf+255, (pointnum * 4 + 2) -255);
			}
			if (ret < 0){
				pr_err( "[focaltech]:%s read touchdata failed.\n", __func__);
				return ret;
			}
			
			_get_coordinate_report(buf, pointnum);
			
		}else{ // gesture DOUBLELINE
			if((pointnum * 4 + 4)<255){
				ret = fts_i2c_Read(i2c_client, &reg_addr, 1, buf, (pointnum * 4 + 4));
			}else{
				ret = fts_i2c_Read(i2c_client, &reg_addr, 1, buf, 255);
				ret = fts_i2c_Read(i2c_client, &reg_addr, 0, buf+255, (pointnum * 4 + 4) -255);
			}
			for(i = 0;i < pointnum;i++){
				coordinate_doblelion_1_x[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[3 + (4 * i)])& 0xFF);
				coordinate_doblelion_1_y[i] = (((s16) buf[4 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[5 + (4 * i)]) & 0xFF);
				printk("pointx[%d] = %d,pointy[%d] = %d\n",i,coordinate_doblelion_1_x[i],i,coordinate_doblelion_1_y[i]);
			}
			coordinate_report[5] = coordinate_doblelion_1_x[0];
			coordinate_report[6] = coordinate_doblelion_1_y[0];
			coordinate_report[7] = coordinate_doblelion_1_x[pointnum-1];
			coordinate_report[8] = coordinate_doblelion_1_y[pointnum-1];
			pointnum = buf[pointnum * 4 + 2]<<8 |buf[pointnum * 4 + 3];
			 //ret = focaltech_i2c_Read(client, buf, 0, buf, (pointnum * 4));
			if((pointnum * 4 )<255){
				ret = fts_i2c_Read(i2c_client, &reg_addr, 0, buf, (pointnum * 4));
			}else{
				ret = fts_i2c_Read(i2c_client, &reg_addr, 0, buf, 255);
				ret = fts_i2c_Read(i2c_client, &reg_addr, 0, buf+255, (pointnum * 4) -255);
			}
			for(i = 0;i < pointnum;i++){
				coordinate_doblelion_2_x[i] = (((s16) buf[0 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[1 + (4 * i)])& 0xFF);
				coordinate_doblelion_2_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
					8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
				printk("pointx[%d] = %d,pointy[%d] = %d\n",i,coordinate_doblelion_2_x[i],i,coordinate_doblelion_2_y[i]);
			}
			if (ret < 0)			{
				pr_err( "[focaltech]:%s read touchdata failed.\n", __func__);
				return ret;
			}			
			coordinate_report[1] = coordinate_doblelion_2_x[0];
			coordinate_report[2] = coordinate_doblelion_2_y[0];
			coordinate_report[3] = coordinate_doblelion_2_x[pointnum-1];
			coordinate_report[4] = coordinate_doblelion_2_y[pointnum-1];
			coordinate_report[13] = 2;	
		}		
		gesture = (gestrue_id == GESTURE_LEFT)	  ? Right2LeftSwip :
				 (gestrue_id == GESTURE_RIGHT) 	  ? Left2RightSwip :
				 (gestrue_id == GESTURE_UP)		  ? Down2UpSwip :
				 (gestrue_id == GESTURE_DOWN)	  ? Up2DownSwip :
				 (gestrue_id == GESTURE_DOUBLECLICK) ? DouTap	:
				 (gestrue_id == GESTURE_DOUBLELINE)  ? DouSwip 	:
				 (gestrue_id == GESTURE_LEFT_V)	  ? RightVee 	:
				 (gestrue_id == GESTURE_RIGHT_V)  ? LeftVee	    :
				 (gestrue_id == GESTURE_V)		  ? DownVee		 :
				 (gestrue_id == GESTURE_DOWN_V)	  ? UpVee 	     :
				 (gestrue_id == GESTURE_O) 		  ? Circle		 :
				 (gestrue_id == GESTURE_cw_O)     ? Circle       :
				 (gestrue_id == GESTURE_W) 		  ? Wgestrue	 :
				 (gestrue_id == GESTURE_M) 		  ? Mgestrue	 :
				 UnkownGestrue;

		if(gesture != UnkownGestrue ){
			TPD_ERR("report_gesture::gesture_id=0x%x,gesture=%d\n",gestrue_id,gesture);
			gesture_upload = gesture;
			input_report_key(tpd->dev, KEY_F4, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_F4, 0);
			input_sync(tpd->dev);	
		}else{
			TPD_ERR("......report_gesture::can't judge gesture.....\n");
			//fts_read_reg(i2c_client, 0xa5, &state);
			//fts_write_reg(i2c_client, 0xa5, 0x00);	   
			fts_read_reg(i2c_client, 0xa5, &state);
			msleep(10);
			fts_write_reg(i2c_client, 0xd0, 0x01);
			fts_write_reg(i2c_client, 0xd1, 0x3f);
			fts_write_reg(i2c_client, 0xd2, 0x07);
			fts_write_reg(i2c_client, 0xd6, 0xff);
		}
		//coordinate_report[0] = gesture;
		//coordinate_report[13] = (gesture == Circle)?0:2;	
	}
	return ret;
}
#endif


static int touch_event_handler(void *unused)
{
	struct ts_event pevent;
	int ret = 0;
	u8 state;

	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);
 	TPD_ERR("..............enter_touch_event_handler!...............\n");
#ifdef TPD_PROXIMITY
	int err;
	hwm_sensor_data sensor_data;
	u8 proximity_status;
#endif

	
	do
	{
		// mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag !=0 );	 
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		//mutex_lock(&ft_suspend_lock);
	#ifdef FTS_GESTRUE
			fts_read_reg(i2c_client, 0xd0, (u8 *)&state);
		    if(state ==1)
		    {
		        fts_read_Gestruedata();
				enable_irq(touch_irq);
		        continue;
		    }
	#endif

	#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			fts_read_reg(i2c_client, 0xb0, &state);
            TPD_ERR("proxi_fts 0xb0 state value is 0x%02X\n", state);
			if(!(state&0x01))
			{
				tpd_enable_ps(1);
			}
			fts_read_reg(i2c_client, 0x01, &proximity_status);
            TPD_ERR("proxi_fts 0x01 value is 0x%02X\n", proximity_status);
			if (proximity_status == 0xC0)
			{
				tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
			}
			TPD_ERR("tpd_proximity_detect = %d\n", tpd_proximity_detect);
			if ((err = tpd_read_ps()))
			{
				TPD_ERR("proxi_fts read ps data : %d\n", err);	
			}
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			//if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			//{
			//	TPD_PROXIMITY_DMESG(" proxi_5206 call hwmsen_get_interrupt_data failed= %d\n", err);	
			//}
		}  
	#endif
                                
	#ifdef MT_PROTOCOL_B
		{
			ret = fts_read_Touchdata(&pevent);
			if(ret == 0)
				fts_report_value(&pevent);
		}
	#else 
	{
		if (tpd_touchinfo(&cinfo, &pinfo)) 
		{
	    	printk("tpd point_num = %d\n",point_num);
			TPD_DEBUG_SET_TIME;
			if(point_num >0) {
				input_report_key(tpd->dev, BTN_TOUCH, 1);
			}
			for (i = 0; i < 5; i++)           
			{                
				if (cinfo.p[i] == 0 || cinfo.p[i] == 2)                    
					tpd_down(cinfo.x[i], cinfo.y[i],cinfo.p[i], cinfo.id[i]);                
				else if (cinfo.p[i] == 1)                    
					tpd_up(cinfo.x[i], cinfo.y[i],cinfo.p[i], cinfo.id[i]);            
			}
			if (point_num <= 0)
            {
                input_report_key(tpd->dev, BTN_TOUCH, 0);
             #ifdef MT_PROTOCOL_B
                tpd_clear_all_touch();
             #else
                tpd_up(0, 0, 0, 0);
             #endif
            }
        input_sync(tpd->dev);
		}
	}
	#endif
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	enable_irq(touch_irq);
	//mutex_unlock(&ft_suspend_lock);
 	}while(!kthread_should_stop());
return 0;
}
 
void fts_reset_tp(int HighOrLow)
{
	if(HighOrLow)
	{
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	    mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);  
	}
	else
	{
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	    mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
	}
}

static int irq_count=0; 
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
 { 
	 TPD_DEBUG_PRINT_INT;
	 tpd_flag = 1;
	 irq_count++;
	 if(irq_count==100){
		TPD_ERR(".....tpd_flag=%d........\n",tpd_flag);
	 	irq_count=0;
	 }
	 disable_irq_nosync(touch_irq);
	 wake_up_interruptible(&waiter);
	 return IRQ_HANDLED;
 }

static int fts_init_gpio_hw(void)
{
	int ret = 0;
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	return ret;
}

static struct manufacture_info tp_info;
static char manu_name[12];
static char TP_FW_ID[12];

bool ft_with_nt35521 = false;
static int get_tsid_through_lcd(char *lcd_id_char)
{
	TPD_ERR("TP get lcd name from cmdline=%s\n",lcd_id_char);
	if(!strncmp(lcd_id_char,"oppo_nt35521s_truly_g5d_hd720_dsi_vdo",37)){
		ft_with_nt35521 = true;
	}
	return 1;
}
__setup("lcm=1-",get_tsid_through_lcd);


static int get_tp_id(struct i2c_client *client)
{
	int ID1,ID2;
	u8 sensor_id = 0;
	
	//int uc_tp_fm_ver;
	

	ID1 = mt_get_gpio_in(GPIO_TP_ID1);
	ID2 = mt_get_gpio_in(GPIO_TP_ID2);
	TPD_ERR("ID1 =%d,ID2 =%d\n",ID1,ID2); 

	if(ID1 == 0&&ID2==0)
	{
		if(ft_with_nt35521)
		{
			TPD_ERR("15131_tp is Truly\n");
			sensor_id = TRULY_NT_TP;	 
		}else{
			TPD_ERR("15131_tp is Truly\n");
			sensor_id = TRULY_TP;	 
		}
			strcpy(manu_name, "TP_TRULY");
	}
	
	if(ID1 == 1&&ID2==0)
	{
		TPD_ERR("15131_tp is OFILM\n");
		sensor_id = OFLM_TP;
		strcpy(manu_name, "TP_OFILM");
	}
	
	if(ID1== 0&&ID2==1)
	{
		sensor_id = BIEL_TP;
		TPD_ERR("15131_tp is BIEL\n");
		strcpy(manu_name, "TP_BIEL");
	}
	
	tp_info.manufacture = manu_name;
	return sensor_id;
}
//Chenggang.Li@BSP add for oppo15131 2016.05.22
int focaltech_erase_pram(struct i2c_client * client)
{
	u8 reg_val[4] = {0};	
	u32 i = 0;		
	u8 auc_i2c_write_buf[10];		
	int i_ret;	u8 state;
	//fts_get_upgrade_info(&upgradeinfo);	
	for (i = 0; i < 30; i++) 
	{
		/*********Step 1:Reset  CTPM *****/
		//fts_ctpm_hw_reset();
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);  
		msleep(25);
		TPD_ERR(" ....fts reset....\n");
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
		msleep(6);
		//usleep_range(6000,6000);
		
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = 0x55;
		i_ret = fts_i2c_Write(client, auc_i2c_write_buf, 1);
		if(i_ret < 0)
		{
			pr_err("[FTS] failed writing  0x55 ! \n");
			continue;
		}
		

		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
		reg_val[0] = reg_val[1] = 0x00;
		
		fts_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == 0x54&& reg_val[1] == 0x22) 
		{
			fts_read_reg(client, 0xd0, &state);
			if(state == 0)
			{
				pr_err("[FTS] Step 3: READ State fail \n");
				continue;
			}
			printk("[FTS] focaltech_erase_pram: i_ret = %d \n", i_ret);			
			pr_err("[FTS] focaltech_erase_pram: READ CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);			
			break;
		} 
		else 
		{
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
			continue;
		}
	}
	
	//AE 00 00 00 00 01 00 // 00地址的内容清00
	auc_i2c_write_buf[0] = 0xAE;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	auc_i2c_write_buf[5] = 0x01;
	auc_i2c_write_buf[6] = 0x00;
	i_ret = fts_i2c_Write(client, auc_i2c_write_buf, 7);
	if(i_ret < 0)
	{
		pr_err("[FTS] failed writing  AE 00 00 00 00 01 00 ! \n");		
	}
	msleep(10);
	if (i >= 30 )
		return -EIO;	
	return 0;
}



static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	int retval = TPD_OK;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	static char FT_REG_ID=0xA3;
	u8 reg_addr;
	int tp_id,i;
	int uc_tp_fm_ver;
	int firmware_id;
#ifdef TPD_PROXIMITY
	int err;
	struct hwmsen_object obj_ps;
#endif

	TPD_ERR("......enter probe.......\n");  
	i2c_client = client;
	focal_dev = &client->dev;
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);  
	msleep(5);

	TPD_ERR(" ....fts reset....\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	msleep(200);

	msg_dma_alloct();
#ifdef VENDOR_EDIT
/*mofei@EXP.BaseDrv.charge, 2015/12/01, add for checking usb connecting state in the charging module */
    g_dma_alloct = 1;
#endif
    fts_init_gpio_hw();

#ifdef VENDOR_EDIT
//chenggang.li@bsp add for oppo15131 2016.05.22
	reg_addr = FT_REG_ID;
	retval = fts_i2c_Read(i2c_client, &reg_addr, 1, &uc_reg_value, 1);
	if(retval<0){
		TPD_ERR("focaltech version read failed,focaltech_i2c_Read error = %d\n",retval);
	}
	if(uc_reg_value != 0x54){
		TPD_ERR("focaltech device id read failed,device id = 0x%x,erase_pram and reset tp\n",uc_reg_value);
		focaltech_erase_pram(i2c_client);	
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
  		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
   		mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);  
		msleep(5);
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
		msleep(100);
		for(i = 0; i < 10; i++){
			fts_i2c_Read(i2c_client, &reg_addr, 1, &uc_reg_value, 1);
			if(uc_reg_value==0x54){
				TPD_ERR("........uc_reg_value==0x54.........\n");
				break;
			}
		msleep(50);
		}		
	}
#endif

	uc_reg_addr = FTS_REG_POINT_RATE;	
	fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	TPD_ERR("mtk_tpd[FTS] report rate is %dHz.\n",uc_reg_value * 10);

	uc_reg_addr = FTS_REG_FW_VER;
	fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	TPD_ERR("mtk_tpd[FTS] Firmware version = 0x%x\n", uc_reg_value);
        

	uc_reg_addr = FTS_REG_CHIP_ID;
	fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	retval=fts_i2c_Read(i2c_client, &uc_reg_addr, 0, &uc_reg_value, 1);
	TPD_ERR("mtk_tpd[FTS] chip id is %d.\n",uc_reg_value);
    if(retval<0)
    {
        TPD_ERR("mtk_tpd[FTS] Read I2C error! driver NOt load!! CTP chip id is %d.\n",uc_reg_value);
		TPD_ERR("......tpd probe failed!..........\n");
		return 0;
	}

	//Chenggang.Li@bsp add for tp driver for usb_check 2016.05.10
	fts_write_reg(i2c_client,0x8B,usb_check_state);
	
	tpd_load_status = 1;
	boot_mode = get_boot_mode(); 	
	TPD_ERR("INIT::boot_mode = %d ............\n",boot_mode); 	
	if((boot_mode == META_BOOT) || (boot_mode == FACTORY_BOOT  )){
		TPD_ERR("......tpd enter FACTORY_BOOT!..........\n");
		/* uc_reg_addr=0xA5;		
		retval = fts_write_reg(i2c_client,uc_reg_addr,0x03);		
		if(retval<0)			
			TPD_ERR("...WRITE 0xA5 ERROR\n");  */
		//pmic_set_register_value(PMIC_LDO_VIO28_EN, 0);
		pmic_ldo_vio28_sw_en(0);		
		return 0;			
	}
	
#ifdef VELOCITY_CUSTOM_fts
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
	}
#endif

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread))
	{ 
		retval = PTR_ERR(thread);
		TPD_ERR(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}else{
		TPD_ERR(TPD_DEVICE " successful to create kernel thread\n");
	}
	
	focaltech_irq_init();
	//enable_irq(touch_irq);
	focaltech_get_upgrade_array();
	
#ifdef SYSFS_DEBUG
    fts_create_sysfs(i2c_client);
#endif

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(i2c_client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",__func__);
#endif
	 
#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(i2c_client);
#endif

	//0:OFLM_TP;1:TRULY_TP;2:BIEL_TP
	tp_id = get_tp_id(i2c_client);
	TPD_ERR("....tp_id=%d.....\n",tp_id);
	if(tp_id==0)
		firmware_id=0x15131100;//ofilm
	if((tp_id==1)||(tp_id==3))
		firmware_id=0x15131000;//truly
	if(tp_id==2)
		firmware_id=0x15131600;//biel	
	choose_tp_fw(tp_id);
	printk("mtk_tpd[FTS] Firmware tpid = 0x%x\n", fts_ctpm_get_i_file_tpid());

#ifdef TPD_AUTO_UPGRADE
	TPD_ERR("Upgrade::boot_mode = %d ............\n",boot_mode);
	if((boot_mode != META_BOOT) && (boot_mode != FACTORY_BOOT)&&
			(boot_mode != RECOVERY_BOOT)&&(boot_mode != KERNEL_POWER_OFF_CHARGING_BOOT)){
		TPD_ERR("******************** CTP Auto Upgrade********************\n");
		fts_ctpm_auto_upgrade(i2c_client);
	}
	fts_read_reg(client, FTS_REG_FW_VER, (u8 *)&uc_tp_fm_ver);
	sprintf(TP_FW_ID,"0x%x",((uc_tp_fm_ver&0x000000ff)|firmware_id));
	tp_info.version=TP_FW_ID;
	TPD_ERR("version:....%s..........\n",tp_info.version);
	TPD_ERR("manu_name:....%s..........\n",tp_info.manufacture);
	register_device_proc("tp", tp_info.version, tp_info.manufacture);
#endif

#ifdef TPD_PROXIMITY
	{
		obj_ps.polling = 1; //0--interrupt mode;1--polling mode;
		obj_ps.sensor_operate = tpd_ps_operate;
		if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
		{
			//TPD_DEBUG("hwmsen attach fail, return:%d.", err);
		}
	}
#endif
	 
//#ifdef FTS_GESTRUE
	//init_para(480,854,60,0,0);
    init_fts_proc();
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_F4);
	__set_bit(KEY_F4 , tpd->dev->keybit);
	set_bit(BTN_TOUCH, tpd->dev->keybit);
	set_bit(ABS_MT_TOUCH_MAJOR, tpd->dev->absbit);	
	set_bit(ABS_MT_WIDTH_MAJOR,tpd->dev->absbit);
	set_bit(ABS_MT_POSITION_X, tpd->dev->absbit);	
	set_bit(ABS_MT_POSITION_Y, tpd->dev->absbit);

#ifdef MT_PROTOCOL_B
	input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS,0);
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, 720, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, 1280, 0, 0);
	
	
#endif
	atomic_set(&double_enable,0);
	printk("fts Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
	return 0;
   
 }

static int tpd_remove(struct i2c_client *client)
{
    msg_dma_release();
#ifdef FTS_CTL_IIC
    ft_rw_iic_drv_exit();
#endif
    //#ifdef SYSFS_DEBUG
    //	fts_release_sysfs(client);
    //#endif
    //#ifdef FTS_APK_DEBUG
    //	fts_release_apk_debug_channel();
    //#endif
	//TPD_DEBUG("TPD removed\n");
	return 0;
 }
 
static int tpd_local_init(void)
{
	TPD_ERR("Focaltech fts I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
	if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
        TPD_ERR("fts unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
        TPD_ERR("fts add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
	TPD_ERR("end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;
    return 0; 
}


int usb_check(int usb_state)
{
	
	printk("usb_check_usb_state =%d",usb_state);
#ifdef VENDOR_EDIT
/*mofei@EXP.BaseDrv.charge, 2015/12/01, modify for checking usb connecting state in the charging module */
	if(g_dma_alloct == 1)
	{	
		if(usb_check_state != usb_state )
		{
			if(usb_state ==1)
			{
				fts_write_reg(i2c_client,0x8B,0x01);
				TPD_ERR("usb enable\n");
				usb_check_state = usb_state;
				return 1;
			}
			else if(usb_state == 0)
			{
				fts_write_reg(i2c_client,0x8B,0x00);
				TPD_ERR("usb disable\n");
				usb_check_state = usb_state;
				return 0;
			}
			else
			return 0;
		}
		else 
		return 0;
	}
	else 
	return 0;
#endif		
}

static void tpd_resume( struct early_suspend *h )
{
	int state;
	int i;
	//unsigned char uc_reg_value;
	//unsigned char uc_reg_addr;
	
	TPD_ERR(".....tpd_resume_start.....\n");
	mutex_lock(&ft_suspend_lock);
 #ifdef TPD_PROXIMITY	
	if (tpd_proximity_flag == 1)
	{
		if(tpd_proximity_flag_one == 1)
		{
			tpd_proximity_flag_one = 0;	
			TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
			return;
		}
	}
#endif	

#ifdef VENDOR_EDIT
	/*huqiao@EXP.BasicDrv, 2015/05/21	Add for 15066 TP START */
	tp_suspend =0;
	/*huqiao@EXP.BasicDrv, 2015/05/21	Add for 15066 TP START */
#endif
	
 	if(1 == atomic_read(&double_enable))
    	fts_write_reg(i2c_client,0xD0,0x00);
	
	
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	//hwPowerOn(MT6323_POWER_LDO_VGP1,VOL_2800,"TP");
#else
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);  
    msleep(1);  
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
	msleep(150);
#endif
	
					
	for (i = 0; i < 10; i++) {
		input_mt_slot(tpd->dev, i);
		input_mt_report_slot_state(tpd->dev,MT_TOOL_FINGER, 0);
	}
	input_report_key(tpd->dev,BTN_TOUCH, 0);
	input_report_key(tpd->dev,BTN_TOOL_FINGER, 0);
	input_sync(tpd->dev);
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
	enable_irq(touch_irq);	
	msleep(50);
	//Chenggang.Li@bsp add for tp driver for usb_check 2016.05.10
	fts_write_reg(i2c_client,0x8B,usb_check_state);
	
	fts_read_reg(i2c_client, 0xA5, (u8 *)&state);
	TPD_ERR("...STATE=%d...\n",state);
	
	//fts_write_reg(i2c_client, 0xA5, 0x00);
	msleep(10);
	tpd_halt = 0;
	if(1 == atomic_read(&glove_enable))
	{
    	fts_write_reg(i2c_client,0xC0,1);
	}else{
		fts_write_reg(i2c_client,0xC0,0);
	}
	mutex_unlock(&ft_suspend_lock);
	TPD_ERR("TPD resume end!\n");

}

static void tpd_suspend( struct early_suspend *h )
{
	static char data = 0x2;
	int i;
	TPD_ERR("....tpd enter suspend....\n");

	
	mutex_lock(&ft_suspend_lock);

#ifdef VENDOR_EDIT
	/*huqiao@EXP.BasicDrv, 2015/05/21	Add for 15066 TP START */

#ifdef VENDOR_EDIT //huqiao@EXP.BasicDrv, 2015/12/9 add for 15066 TP
		for (i = 0; i < 10; i++) {
			input_mt_slot(tpd->dev, i);
			input_mt_report_slot_state(tpd->dev,MT_TOOL_FINGER, 0);
		}
		input_report_key(tpd->dev,BTN_TOUCH, 0);
		input_report_key(tpd->dev,BTN_TOOL_FINGER, 0);
		input_sync(tpd->dev); 
#endif

	tp_suspend =1;
	/*huqiao@EXP.BasicDrv, 2015/05/21	Add for 15066 TP START */
#endif

	if(1 == atomic_read(&double_enable))
	{
		fts_write_reg(i2c_client, 0xd0, 0x01);
		if (fts_updateinfo_curr.CHIP_ID==0x54)
		{
			fts_write_reg(i2c_client, 0xd1, 0x3f);
			fts_write_reg(i2c_client, 0xd2, 0x07);
			//fts_write_reg(i2c_client, 0xd5, 0xff);
			fts_write_reg(i2c_client, 0xd6, 0xff);
			//fts_write_reg(i2c_client, 0xd7, 0xff);
			//fts_write_reg(i2c_client, 0xd8, 0xff);
		}
		mutex_unlock(&ft_suspend_lock);
      	return;
	}
	

 	tpd_halt = 1;
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	disable_irq(touch_irq);
	//mutex_lock(&i2c_access);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	//hwPowerDown(MT6323_POWER_LDO_VGP1,"TP");
#else
	//i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
	fts_write_reg(i2c_client,0xa5,data);
#endif
	//mutex_unlock(&i2c_access);
	mutex_unlock(&ft_suspend_lock);
    TPD_ERR("tpd suspend end!\n");
	

} 


static struct tpd_driver_t tpd_device_driver = {
         .tpd_device_name = "mtk-tpd",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
};

 /* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	msleep(5);
    TPD_ERR("MediaTek fts touch panel driver init\n");
    i2c_register_board_info(0, &fts_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0)
        TPD_ERR("add fts driver failed\n");
	 return 0;
}
 
 
static ssize_t tp_double_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{

    int len = 0;
	int ret = 0 ;
	char page[512];
	TPD_ERR("double tap enable is: %d\n", atomic_read(&double_enable));
	len = sprintf(page, "%d\n", atomic_read(&double_enable));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_double_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0 ;
	int state =0;
	u8 gesture_reg_tmp = 0;
	int i = 0;
	char buf[10] = {0};

	
	if (count > 10) 
		return count;
	if (copy_from_user( buf, buffer, count)) {
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf,"%d",&ret);
 	TPD_ERR("tp_double_write_func:buf = %d,ret = %d\n",*buf,ret);
	mutex_lock(&ft_suspend_lock);
	if((ret == 0 )||(ret == 1))
	{
		atomic_set(&double_enable,ret);
	}
	
#ifdef VENDOR_EDIT
	if(tp_suspend ==1)
	{
		if(ret ==1)
		{
			//TPD_ERR(".....gesture will enable.....\n");
			fts_read_reg(i2c_client, 0xa5, (u8 *)&state);
			//fts_write_reg(i2c_client, 0xa5, 0x00);	
			do{
				msleep(50);
				TPD_ERR(".....gesture will enable try %d time.....\n", i);
				fts_write_reg(i2c_client, 0xd0, 0x01);
				fts_read_reg(i2c_client, 0xd0, &gesture_reg_tmp);
				i++;
			}while((gesture_reg_tmp != 0x01) &&  (i < 10));
			fts_write_reg(i2c_client, 0xd1, 0x3f);
			fts_write_reg(i2c_client, 0xd2, 0x07);
			fts_write_reg(i2c_client, 0xd6, 0xff);
			enable_irq(touch_irq);
		}
		if(ret ==0)
		{
			TPD_ERR(".....gesture will disable.....\n");
			fts_read_reg(i2c_client, 0xa5, (u8 *)&state);
			msleep(100);
			fts_write_reg(i2c_client,0xa5,0x02);
		}
	}
	
#endif
	mutex_unlock(&ft_suspend_lock);
	return count;
}


static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[512];
	ret = sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
                   coordinate_report[1], coordinate_report[2], coordinate_report[3], coordinate_report[4],
                   coordinate_report[5], coordinate_report[6], coordinate_report[7], coordinate_report[8],
                   coordinate_report[9], coordinate_report[10], coordinate_report[11], coordinate_report[12],
                   coordinate_report[13]);  
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;	
}

#ifdef DEBUG_TP_CHARGE
static ssize_t charge_control_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int ret = 0 ;
	char buf[10] = {0};
	if (count > 10) 
		return count;
	
	if (copy_from_user( buf, buffer, count)) {
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	sscanf(buf,"%d",&ret);
	switch(ret)
	{	 
		case 0:
			TPD_ERR("charge will be disable\n");
			pmic_ldo_vio28_sw_en(ret);
			break;
		case 1:
			TPD_ERR("charge will be enable\n");
			pmic_ldo_vio28_sw_en(ret);
			break;
		default:
			TPD_ERR("Please enter 0 or 1 to open or close the glove function\n");
	} 
	return count;
}
#endif

#ifdef DEBUG_TP_RAWDATA
static ssize_t fts_test_show(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{

    int ret = 0;
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	TPD_ERR("FTS::enter fts_test_show\n");
	ret = ftxxxx_ftslibtest_sample(i2c_client);
	
	if (ret == 0)
	{
		TPD_ERR("fts_test_show::successful!\n"); 
	}
	else
	{
		TPD_ERR("fts_test_show::failed!\n"); 
	}
	return ret;
}
#endif


static const struct file_operations double_tap_enable_fops = {	
	.write = tp_double_write_func,	
	.read =  tp_double_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static const struct file_operations coordinate_fops = {	
	.read =  coordinate_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#ifdef DEBUG_TP_CHARGE
static const struct file_operations proc_charge_contorl = {
	.write = charge_control_write_func,
	//.read = focaltech_update_fw_show,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif 

#ifdef DEBUG_TP_RAWDATA
static const struct file_operations proc_tp_delta_show = {
	//.write = charge_control_write_func,
	.read = fts_test_show,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif 

static ssize_t limit_enable_show(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret =0;
	char page[16];

	TPD_DEBUG("limit_enable is: %d\n", limit_enable);
	ret = sprintf(page, "%d\n", limit_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t limit_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[8] = {0};
	int temp = 0;
	int ret = 0;
	u8  limit_control_regaddr = 0x8C;
	
	if (count > 2 || i2c_client == NULL) 
		return count;
	
	if (copy_from_user(buf, buffer, count)) {
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	
	if (*buf == '1') {
		temp = 1;
	} else if (*buf == '0') {
		temp = 0;
	}
	TPD_ERR("%s temp = %d\n", __func__, temp);
	limit_enable = temp;
	
	mutex_lock(&ft_suspend_lock);
	if (tp_suspend == 0) {
		if (limit_enable == 0) {
			ret = fts_write_reg(i2c_client, limit_control_regaddr, 0x01);
		} else if(limit_enable == 1) {
			ret = fts_write_reg(i2c_client, limit_control_regaddr, 0x00);
		}
	}
	mutex_unlock(&ft_suspend_lock);	
	
	return count;
}

static const struct file_operations proc_limit_enable =
{
	.read = limit_enable_show,
	.write = limit_enable_write,
	.owner = THIS_MODULE,	
};


static int init_fts_proc(void)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_temp = NULL;
	
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if(prEntry_tp == NULL)
	{
		ret = -ENOMEM;
	  	TPD_ERR(KERN_INFO"touchpanel: Couldn't create TP proc entry\n");
	}

#ifdef SUPPORT_GESTURE
	prEntry_dtap = proc_create( "double_tap_enable", 0666, prEntry_tp ,&double_tap_enable_fops);
	if(prEntry_dtap == NULL)
	{
		ret = -ENOMEM;
	  	TPD_ERR(KERN_INFO"double_tap_enable: Couldn't create proc entry\n");
	}
#endif

#ifdef SUPPORT_REPORT_COORDINATE
	prEntry_coodinate = proc_create("coordinate", 0444, prEntry_tp, &coordinate_fops);
    if(prEntry_coodinate == NULL)
    {	   
		ret = -ENOMEM;	   
		TPD_ERR(KERN_INFO"coordinate: Couldn't create proc entry\n");
    }
#endif

#ifdef DEBUG_TP_CHARGE
	prEntry_dtap = proc_create("charge_contorl", 0644, prEntry_tp,&proc_charge_contorl);		
	if(prEntry_dtap == NULL){            				
		ret = -ENOMEM;	   				
		TPD_ERR(KERN_INFO"oppo_tp_fw_update: Couldn't create proc entry\n");		
	}
#endif

#ifdef DEBUG_TP_RAWDATA
	prEntry_dtap = proc_create("ft_tp_delta_show", 0644, prEntry_tp,&proc_tp_delta_show);		
	if(prEntry_dtap == NULL){           				
		ret = -ENOMEM;	   				
		TPD_ERR(KERN_INFO"oppo_tp_fw_update: Couldn't create proc entry\n");		
	}
#endif

	prEntry_temp = proc_create("oppo_tp_limit_enable", 0664, prEntry_tp, &proc_limit_enable);
	if (prEntry_temp == NULL) {            
		ret = -ENOMEM;	   
		TPD_ERR(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
return 0;
}




/* should never be called */
static void __exit tpd_driver_exit(void) {
     TPD_DMESG("MediaTek fts touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
}
 

/* static int get_panel_type(void)
{
	//wuyu debug
	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;
    int Channel = 0;
	int times = 3;
	int res = 0;
	
    if( IMM_IsAdcInitReady() == 0 )
    {
        TPD_ERR("wuyu---AUXADC is not ready");
        return 0;
    }

    i = times;
    while (i--)
    {
        ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
        ret += ret_temp;
        TPD_ERR("wuyu--ret_temp=%d\n",ret_temp);
    }
    
    ret = ret*1500/4096	;
    ret = ret/times;
	
    TPD_ERR("wuyu--output mV = %d\n", ret);
	
	if ((ret >= 600) && (ret <= 1200))
		res = 1;
	return res;
} */
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);


