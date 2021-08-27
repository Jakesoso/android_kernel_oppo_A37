/* drivers/input/touchscreen/gt1x_tpd.c
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * Version: 1.0   
 * Revision Record: 
 *      V1.0:  first release. 2014/09/28.
 *
 */

#include "gt1x_tpd_custom.h"
#include "gt1x_generic.h"

#ifdef GTP_CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#if TPD_SUPPORT_I2C_DMA
#include <linux/dma-mapping.h>
#endif

#if GTP_ICS_SLOT_REPORT
#include <linux/input/mt.h>
#endif
#include <linux/proc_fs.h> 
#if (GTP_HAVE_TOUCH_KEY && TPD_HAVE_BUTTON)
#error GTP_HAVE_TOUCH_KEY and TPD_HAVE_BUTTON are mutually exclusive.
#endif

#include "circle_point.h"
extern struct tpd_device *tpd;
static spinlock_t irq_lock; 
static int tpd_flag = 0;
static int tpd_irq_flag;
static int tpd_eint_mode = 1;
static int tpd_polling_time = 50;
static struct task_struct *thread = NULL;
#include <mach/pmic_api_ldo.h>

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);

#if TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT] = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#ifdef GTP_CONFIG_OF
static unsigned int tpd_touch_irq;
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc);
#else
static void tpd_eint_interrupt_handler(void);
#endif
static int tpd_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);

#ifndef MT6572
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En, kal_bool ACT_Polarity, void (EINT_FUNC_PTR) (void), kal_bool auto_umask);
#endif

#define GTP_DRIVER_NAME  "gt1x"

#define SUPPORT_GESTURE
#define SUPPORT_REPORT_COORDINATE
struct Coordinate {
    int x;
    int y;
};
static  Point Point_input[64];
static  Point Point_output[4];
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
static uint32_t clockwise=1;

#ifdef SUPPORT_GESTURE
//static atomic_t double_enable;
atomic_t double_enable;
//atomic_t glove_mode_enable;
atomic_t is_in_suspend;
static uint32_t gesture;
static uint32_t gesture_upload;

//static DEFINE_SEMAPHORE(suspend_sem);// after suspend sucsess  , can start resume
static DEFINE_MUTEX(gt_suspend_lock);

#define DTAP_DETECT          0xCC
#define UP_VEE_DETECT        0x76 
#define DOWN_VEE_DETECT      0x5e
#define LEFT_VEE_DETECT      0x63 
#define RIGHT_VEE_DETECT     0x3e
#define CIRCLE_DETECT        0x6f
#define DOUSWIP_DETECT       0x48 
#define DOUUPSWIP_DETECT     0x4E
#define RIGHT_SLIDE_DETECT   0xAA
#define LEFT_SLIDE_DETECT    0xbb
#define DOWN_SLIDE_DETECT    0xAB
#define UP_SLIDE_DETECT      0xBA
#define M_DETECT			 0x6D
#define W_DETECT			 0x77


#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define RightVee            4   // >
#define LeftVee             5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W	
#define CustomGestrue       14  //Custom
#endif


#ifdef SUPPORT_GESTURE
struct proc_dir_entry *prEntry_tp = NULL; 
static struct proc_dir_entry *prEntry_dtap = NULL;
static struct proc_dir_entry *prEntry_coodinate  = NULL; 
static int init_goodix_proc(void);
static ssize_t tp_double_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos);
static ssize_t tp_double_write_func(struct file *file, const char *buffer, size_t count,loff_t *ppos);
static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static ssize_t tp_devices_check_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos);
//static int tp_devices_check_write_func(struct file *file, const char *buffer, unsigned long count,void *data);

static const struct file_operations gt1x_gesture = {
	.owner = THIS_MODULE,
	.read  = tp_double_read_func,
	.write = tp_double_write_func,
};
static const struct file_operations gt1x_gesture_coor = {
	.owner = THIS_MODULE,
	.read  = coordinate_proc_read_func,
};
#endif

static const struct file_operations gt1x_devices_check = {
	.owner = THIS_MODULE,
	.read  = tp_devices_check_read_func,
	//.write = tp_devices_check_write_func,
};

//Chenggang.Li@BSP.TP add for 16021 2016/04/27 for gesture
/************start*************/
int firmware_id;
char manu_name[12];
int GT_TP_ID;
/************end*************/


static const struct i2c_device_id tpd_i2c_id[] = { {GTP_DRIVER_NAME, 0}, {} };
static unsigned short force[] = { 0, GTP_I2C_ADDRESS, I2C_CLIENT_END, I2C_CLIENT_END };
static const unsigned short *const forces[] = { force, NULL };


static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO(GTP_DRIVER_NAME, (GTP_I2C_ADDRESS >> 1)) };

static struct i2c_driver tpd_i2c_driver = {
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.detect = tpd_i2c_detect,
	.driver.name = GTP_DRIVER_NAME,
	.id_table = tpd_i2c_id,
	.address_list = (const unsigned short *)forces,
};

#if TPD_SUPPORT_I2C_DMA
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
struct mutex dma_mutex;

static s32 i2c_dma_write_mtk(u16 addr, u8 * buffer, s32 len)
{
	s32 ret = 0;
	s32 pos = 0;
	s32 transfer_length;
	u16 address = addr;
	struct i2c_msg msg = {
		.flags = !I2C_M_RD,
		.ext_flag = (gt1x_i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG),
		.timing = I2C_MASTER_CLOCK,
		.buf = (u8 *) gpDMABuf_pa,
	};

	mutex_lock(&dma_mutex);
	while (pos != len) {
		if (len - pos > (IIC_DMA_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH)) {
			transfer_length = IIC_DMA_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH;
		} else {
			transfer_length = len - pos;
		}
		
		gpDMABuf_va[0] = (address >> 8) & 0xFF;
		gpDMABuf_va[1] = address & 0xFF;
		memcpy(&gpDMABuf_va[GTP_ADDR_LENGTH], &buffer[pos], transfer_length);

		msg.len = transfer_length + GTP_ADDR_LENGTH;

		ret = i2c_transfer(gt1x_i2c_client->adapter, &msg, 1);
		if (ret != 1) {
			GTP_ERROR("I2c Transfer error! (%d)", ret);
			ret = ERROR_IIC;
			break;
		}
		ret = 0;
		pos += transfer_length;
		address += transfer_length;
	}
	mutex_unlock(&dma_mutex);
	return ret;
}

static s32 i2c_dma_read_mtk(u16 addr, u8 * buffer, s32 len)
{
	s32 ret = ERROR;
	s32 pos = 0;
	s32 transfer_length;
	u16 address = addr;
	u8 addr_buf[GTP_ADDR_LENGTH] = { 0 };
	struct i2c_msg msgs[2] = {
		{
		 .flags = 0,	//!I2C_M_RD,
		 .addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG),
		 .timing = I2C_MASTER_CLOCK,
		 .len = GTP_ADDR_LENGTH,
		 .buf = addr_buf,
		 },
		{
		 .flags = I2C_M_RD,
		 .ext_flag = (gt1x_i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		 .addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG),
		 .timing = I2C_MASTER_CLOCK,
		 .buf = (u8 *) gpDMABuf_pa,
		 },
	};
	mutex_lock(&dma_mutex);
	while (pos != len) {
		if (len - pos > IIC_DMA_MAX_TRANSFER_SIZE) {
			transfer_length = IIC_DMA_MAX_TRANSFER_SIZE;
		} else {
			transfer_length = len - pos;
		}

		msgs[0].buf[0] = (address >> 8) & 0xFF;
		msgs[0].buf[1] = address & 0xFF;
		msgs[1].len = transfer_length;

		ret = i2c_transfer(gt1x_i2c_client->adapter, msgs, 2);
		if (ret != 2) {
			GTP_ERROR("I2C Transfer error! (%d)", ret);
			ret = ERROR_IIC;
			break;
		}
		ret = 0;
		memcpy(&buffer[pos], gpDMABuf_va, transfer_length);
		pos += transfer_length;
		address += transfer_length;
	};
	mutex_unlock(&dma_mutex);
	return ret;
}

#else

static s32 i2c_write_mtk(u16 addr, u8 * buffer, s32 len)
{
	s32 ret;

	struct i2c_msg msg = {
		.flags = 0,
		.addr = (gt1x_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG),	//remain
		.timing = I2C_MASTER_CLOCK,
	};

	ret = _do_i2c_write(&msg, addr, buffer, len);
	return ret;
}

static s32 i2c_read_mtk(u16 addr, u8 * buffer, s32 len)
{
	int ret;
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };

	struct i2c_msg msgs[2] = {
		{
		 .addr = ((gt1x_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH,
		 .timing = I2C_MASTER_CLOCK},
		{
		 .addr = ((gt1x_i2c_client->addr & I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
		 .flags = I2C_M_RD,
		 .timing = I2C_MASTER_CLOCK},
	};

	ret = _do_i2c_read(msgs, addr, buffer, len);
	return ret;
}
#endif /* TPD_SUPPORT_I2C_DMA */

/**
 * @return: return 0 if success, otherwise return a negative number
 *          which contains the error code.
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
#if TPD_SUPPORT_I2C_DMA
	return i2c_dma_read_mtk(addr, buffer, len);
#else
	return i2c_read_mtk(addr, buffer, len);
#endif
}

/**
 * @return: return 0 if success, otherwise return a negative number
 *          which contains the error code.
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
#if TPD_SUPPORT_I2C_DMA
	return i2c_dma_write_mtk(addr, buffer, len);
#else
	return i2c_write_mtk(addr, buffer, len);
#endif
}

#ifdef TPD_REFRESH_RATE
/**
 * gt1x_set_refresh_rate - Write refresh rate
 * @rate: refresh rate N (Duration=5+N ms, N=0~15)
 * Return: 0---succeed.
 */
static u8 gt1x_set_refresh_rate(u8 rate)
{
	u8 buf[1] = { rate };

	if (rate > 0xf) {
		GTP_ERROR("Refresh rate is over range (%d)", rate);
		return ERROR_VALUE;
	}

	GTP_INFO("Refresh rate change to %d", rate);
	return gt1x_i2c_write(GTP_REG_REFRESH_RATE, buf, sizeof(buf));
}

/**
  *gt1x_get_refresh_rate - get refresh rate
  *Return: Refresh rate or error code
 */
static u8 gt1x_get_refresh_rate(void)
{
	int ret;
	u8 buf[1] = { 0x00 };
	ret = gt1x_i2c_read(GTP_REG_REFRESH_RATE, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	GTP_INFO("Refresh rate is %d", buf[0]);
	return buf[0];
}

static ssize_t show_refresh_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = gt1x_get_refresh_rate();
	if (ret < 0)
		return 0;
	else
		return sprintf(buf, "%d\n", ret);
}

static ssize_t store_refresh_rate(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	gt1x_set_refresh_rate(simple_strtoul(buf, NULL, 16));
	return size;
}

static DEVICE_ATTR(tpd_refresh_rate, 0664, show_refresh_rate, store_refresh_rate);

static struct device_attribute *gt1x_attrs[] = {
	&dev_attr_tpd_refresh_rate,
};
#endif

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, "mtk-tpd");
	return 0;
}

static int tpd_power_on(void)
{
	gt1x_power_switch(SWITCH_ON);

    gt1x_reset_guitar();

	if (gt1x_get_chip_type() != 0) {
		return -1;
	}

	if (gt1x_reset_guitar() != 0) {
		return -1;
	}
	return 0;
}

void gt1x_irq_enable(void)
{
    unsigned long flag;
    
    spin_lock_irqsave(&irq_lock, flag);
    if (!tpd_irq_flag) { // 0-disabled
        tpd_irq_flag = 1;  // 1-enabled
#ifdef GTP_CONFIG_OF
        enable_irq(tpd_touch_irq);
#else
	    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
    }
    spin_unlock_irqrestore(&irq_lock, flag);
}

void gt1x_irq_disable(void)
{
    unsigned long flag;

    spin_lock_irqsave(&irq_lock, flag);
    if (tpd_irq_flag) {
        tpd_irq_flag = 0;
#ifdef GTP_CONFIG_OF
        disable_irq_nosync(tpd_touch_irq);
#else
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
    }
    spin_unlock_irqrestore(&irq_lock, flag);
}

int gt1x_power_switch(s32 state)
{
    static int power_state = 0;
    
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(10);

	switch (state) {
	case SWITCH_ON:
        if (power_state == 0) {    
    		GTP_DEBUG("Power switch on!");
            power_state = 1;
        }
    		break;
	case SWITCH_OFF:
        if (power_state == 1) {
		    GTP_DEBUG("Power switch off!");
            power_state = 0;
        }
		break;
	default:
		GTP_ERROR("Invalid power switch command!");
		break;
	}
	return 0;
}


static int tpd_irq_registration(void)
{
#ifdef GTP_CONFIG_OF
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};
	GTP_INFO("Device Tree Tpd_irq_registration!");
	
	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if(node){
		of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		tpd_touch_irq = irq_of_parse_and_map(node, 0);
		GTP_INFO("Device gt1x_int_type = %d!", gt1x_int_type);
		if (!gt1x_int_type)	//EINTF_TRIGGER
		{
			ret = request_irq(tpd_touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, EINTF_TRIGGER_RISING, "TOUCH_PANEL-eint", NULL);
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		} else {
			ret = request_irq(tpd_touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, EINTF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		}
	}else{
		GTP_ERROR("tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}
	GTP_INFO("irq:%d, debounce:%d-%d:", tpd_touch_irq, ints[0], ints[1]);
	return ret;
    
#else

    #ifndef MT6589
	if (!gt1x_int_type) {	/*EINTF_TRIGGER */
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);
	} else {
		mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	}

    #else
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);

	if (!gt1x_int_type) {
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1);
	} else {
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, tpd_eint_interrupt_handler, 1);
	}
    #endif
    return 0;
#endif
}

static int boot_mode;
static s32 tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 err = 0;
	
#if GTP_HAVE_TOUCH_KEY
	s32 idx = 0;
#endif

  	GTP_ERROR("....enter probe 16021...\n");
	tpd_load_status = 1;
	gt1x_i2c_client = client;
	spin_lock_init(&irq_lock);
    init_goodix_proc();
	boot_mode = get_boot_mode();
    if((boot_mode == META_BOOT) || (boot_mode == FACTORY_BOOT  )){
		//gt1x_enter_sleep();
		pmic_ldo_vio28_sw_en(0);
		return 0;
    }
	if (gt1x_init()) {
		/* TP resolution == LCD resolution, no need to match resolution when initialized fail */
		gt1x_abs_x_max = 0;
		gt1x_abs_y_max = 0;
	}
	
#if GTP_ICS_SLOT_REPORT	
	input_mt_init_slots(tpd->dev, GTP_MAX_TOUCH, 0);
#endif
	set_bit(ABS_MT_WIDTH_MAJOR,tpd->dev->absbit);

	thread = kthread_run(tpd_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		GTP_ERROR(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}
#if GTP_HAVE_TOUCH_KEY
	for (idx = 0; idx < GTP_MAX_KEY_NUM; idx++) {
		input_set_capability(tpd->dev, EV_KEY, gt1x_touch_key_array[idx]);
	}
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(tpd->dev, EV_KEY, KEY_GES_CUSTOM);
    input_set_capability(tpd->dev, EV_KEY, KEY_GES_REGULAR);
#endif

	GTP_GPIO_AS_INT(GTP_INT_PORT);

	msleep(50);

/* interrupt registration */

	tpd_irq_registration();
	gt1x_irq_enable();

#if GTP_ESD_PROTECT
	/*  must before auto update */
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
	thread = kthread_run(gt1x_auto_update_proc, (void *)NULL, "gt1x auto update");
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		GTP_ERROR(TPD_DEVICE "failed to create auto-update thread: %d\n", err);
	}
#endif

#ifdef SUPPORT_GESTURE
	atomic_set(&double_enable,0); 
	//atomic_set(&glove_mode_enable,0); 
	atomic_set(&is_in_suspend,0);
#endif  
	return 0;
}

#ifdef GTP_CONFIG_OF
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
    TPD_DEBUG_PRINT_INT;
	
	tpd_flag = 1;

	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
    spin_lock(&irq_lock);
    tpd_irq_flag = 0;
	disable_irq_nosync(tpd_touch_irq);
    spin_unlock(&irq_lock);
 	wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
static void tpd_eint_interrupt_handler(void)
{
	TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
    gt1x_irq_disable();
	wake_up_interruptible(&waiter);
}
#endif



static int report_count=0;
void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
	static int touch_major_report_num  = 5;
#if GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif
	input_report_key(tpd->dev, BTN_TOUCH, 1);
#if GTP_ICS_SLOT_REPORT
	report_count++;
	
	input_mt_slot(tpd->dev, id);
	input_mt_report_slot_state(tpd->dev,MT_TOOL_FINGER, 1);
	//input_report_key(tpd->dev, BTN_TOOL_FINGER, 1);
	//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR,touch_major_report_num);
#else
//	input_report_key(tpd->dev, BTN_TOUCH, 1);
	if ((!size) && (!id)) {
		/* for virtual button */
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
	}
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
#endif
	if(report_count==50){					
		GTP_ERROR("tpd_down::x[%d]  y[%d]=[%d %d]\n", id,id,x,y);	
		touch_major_report_num++;
		report_count=0;
	}
	if(touch_major_report_num==10)
	{
		touch_major_report_num=5;	
	}
#ifdef TPD_HAVE_BUTTON
	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {
		tpd_button(x, y, 1);
	}
#endif
}

void gt1x_touch_up(s32 id)
{
	
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_report_key(tpd->dev, BTN_TOOL_FINGER, 0);
	
	
#if GTP_ICS_SLOT_REPORT
	input_mt_slot(tpd->dev, id);
	input_mt_report_slot_state(tpd->dev,MT_TOOL_FINGER, 0);
#else
	//input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
#endif
#ifdef TPD_HAVE_BUTTON
	if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode()) {
		tpd_button(0, 0, 0);
	}
#endif
}


#if GTP_CHARGER_SWITCH
#ifdef MT6573
#define CHR_CON0      (0xF7000000+0x2FA00)
#else
extern kal_bool upmu_is_chr_det(void);
#endif

u32 gt1x_get_charger_status(void)
{
	u32 chr_status = 0;
#ifdef MT6573
	chr_status = *(volatile u32 *)CHR_CON0;
	chr_status &= (1 << 13);
#else /* ( defined(MT6575) || defined(MT6577) || defined(MT6589) ) */
	chr_status = upmu_is_chr_det();
#endif
	return chr_status;
}
#endif

int ClockWise(Point *p,int n)
{

	int i,j,k;
	int count = 0;
	//double z;
	long int z;
	if (n < 3)
		return -1;
	for (i=0;i<n;i++) 
	{
		j = (i + 1) % n;
		k = (i + 2) % n;
		if( (p[i].x==p[j].x) && (p[j].x==p[j].y) )
		   continue;
		z = (p[j].x - p[i].x) * (p[k].y - p[j].y);
		z -= (p[j].y - p[i].y) * (p[k].x - p[j].x);
		if (z < 0)
			count--;
		else if (z > 0)
			count++;
	}
    
	printk("ClockWise count = %d\n",count);
	if (count > 0)
		return 1; 
	else if (count < 0)
		return 0;
	else
		return 0;
}

static int tpd_event_handler(void *unused)
{
	u8 finger = 0;
	u8 end_cmd = 0;
	s32 ret = 0;
	u8 point_data[11] = { 0 };
	struct sched_param param = {.sched_priority = RTPM_PRIO_TPD };//RTPM_PRIO_TPD
	
    u8 doze_buf[3];
	u8 clear_buf[1];
	u8 coordinate_single[260];
	u8 coordinate_size;
	int i = 0;
	int j = 0;
	printk("creat tpd_event_handler successful!\n");
	sched_setscheduler(current, SCHED_RR, &param);
	
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		if (tpd_eint_mode) {
			wait_event_interruptible(waiter, tpd_flag != 0);
			tpd_flag = 0;
		} else {
			GTP_DEBUG("Polling coordinate mode!");
			msleep(tpd_polling_time);
		}

		set_current_state(TASK_RUNNING);

		if(update_info.status){
			GTP_ERROR("ignore interrupt during fw update!\n");
			continue;
		}
		mutex_lock(&i2c_access);
		
#if GTP_GESTURE_WAKEUP
#ifdef SUPPORT_GESTURE
	 
        if (DOZE_ENABLED == gesture_doze_status)
        {
				ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE, doze_buf, 2);
				GTP_DEBUG("0x814C = 0x%02X,ret=%d\n", doze_buf[0],ret);  
				if (ret == 0 )
				{  
				   
					if(doze_buf[0] != 0)
					{
					    memset(coordinate_single, 0, 260);
					    coordinate_size=doze_buf[1];
						//GTP_ERROR("report gesture::original gesture=%d  coordinate_size=%d \n",doze_buf[0],coordinate_size);
						if(coordinate_size>64) //mingqiang.guo@phone.bsp add coordinate_size*4 can not large > 260 
							coordinate_size = 64;
				    	ret = gt1x_i2c_read(GTP_REG_WAKEUP_GESTURE_DETAIL, coordinate_single, coordinate_size*4);
							switch (doze_buf[0]) 
							{
								case DTAP_DETECT:
								gesture = DouTap;		
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x =  0; 
								Point_1st.y =  0; 
								clockwise = 0 ;
								break;

								case UP_VEE_DETECT :
								gesture = UpVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case DOWN_VEE_DETECT :
								gesture = DownVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case LEFT_VEE_DETECT:
								gesture =  LeftVee;
                                
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
								Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
								break;
								
								case RIGHT_VEE_DETECT :
								gesture =  RightVee;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[8]  |  (coordinate_single[9] << 8);
								Point_end.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;	
								
								case CIRCLE_DETECT  :
								gesture =  Circle;
								j = 0; 
								for(i = 0; i < coordinate_size;i++)
								{
									Point_input[i].x = coordinate_single[j]  |  (coordinate_single[j+1] << 8);
									Point_input[i].y = coordinate_single[j+2] |  (coordinate_single[j+3] << 8);
									j = j+4;
									GTP_INFO("Point_input[%d].x = %d,Point_input[%d].y = %d\n",i,Point_input[i].x,i,Point_input[i].y)	;					
								}

								clockwise = ClockWise(&Point_input[0],coordinate_size-2);
								GetCirclePoints(&Point_input[0], coordinate_size,Point_output);
								Point_start.x = Point_input[0].x;
								Point_start.y = Point_input[0].y;

								Point_end.x = Point_input[coordinate_size-1].x;
								Point_end.y = Point_input[coordinate_size-1].y;

								Point_1st.x = Point_output[0].x;
								Point_1st.y = Point_output[0].y;
						
								Point_2nd.x = Point_output[1].x;
								Point_2nd.y = Point_output[1].y;
						
								Point_3rd.x = Point_output[2].x;
								Point_3rd.y = Point_output[2].y;
						
								Point_4th.x = Point_output[3].x;
								Point_4th.y = Point_output[3].y;
								break;
								
								case DOUSWIP_DETECT  :
								gesture =  DouSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
								Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
								Point_2nd.x = coordinate_single[12]  |  (coordinate_single[13] << 8);
								Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								break;
							
								case DOUUPSWIP_DETECT:
								gesture =  DouSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);
								Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);
								Point_2nd.x = coordinate_single[12]  |  (coordinate_single[13] << 8);
								Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								break;
								
								case RIGHT_SLIDE_DETECT :
								gesture =  Left2RightSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
							    break;
								
								case LEFT_SLIDE_DETECT :
								gesture =  Right2LeftSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;
								
								case DOWN_SLIDE_DETECT  :
								gesture =  Up2DownSwip;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
							    break;
								
								case UP_SLIDE_DETECT :
							    gesture =  Down2UpSwip;
							    Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4]  |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								break;	
								
								case M_DETECT  :
								gesture =  Mgestrue;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
								Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_2nd.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_2nd.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_3rd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
								Point_3rd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
							    break;
								
								case W_DETECT :
								gesture =  Wgestrue;
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[16]  |  (coordinate_single[17] << 8);
								Point_end.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_1st.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_1st.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_2nd.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_2nd.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_3rd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
								Point_3rd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								break;	
								
								default:
								if((doze_buf[0]>=1)&&(doze_buf[0]<=15))
							{
								gesture =  doze_buf[0] + OPPO_CUSTOM_GESTURE_ID_BASE;  //user custom gesture 
								Point_start.x = coordinate_single[0] |  (coordinate_single[1] << 8);
								Point_start.y = coordinate_single[2] |  (coordinate_single[3] << 8);	
								Point_end.x = coordinate_single[4] |  (coordinate_single[5] << 8);
								Point_end.y = coordinate_single[6] |  (coordinate_single[7] << 8);
								Point_1st.x = coordinate_single[8] |  (coordinate_single[9] << 8);
								Point_1st.y = coordinate_single[10] |  (coordinate_single[11] << 8);	
								Point_2nd.x = coordinate_single[12] |  (coordinate_single[13] << 8);
								Point_2nd.y = coordinate_single[14] |  (coordinate_single[15] << 8);
								Point_3rd.x = coordinate_single[16] |  (coordinate_single[17] << 8);
								Point_3rd.y = coordinate_single[18] |  (coordinate_single[19] << 8);	
								Point_3rd.x = coordinate_single[20] |  (coordinate_single[21] << 8);
								Point_3rd.y = coordinate_single[22] |  (coordinate_single[23] << 8); 
							}
							else
							{
								gesture =  doze_buf[0];
							}
							    break;
					
			                }
							GTP_ERROR("report gesture::detect %s gesture\n", gesture == DouTap ? "double tap" :
                                                        gesture == UpVee ? "up vee" :
                                                        gesture == DownVee ? "down vee" :
                                                        gesture == LeftVee ? "(<)" :
                                                        gesture == RightVee ? "(>)" :
                                                        gesture == Circle ? "circle" :
														gesture == DouSwip ? "(||)" :
                                                        gesture == Left2RightSwip ? "(-->)" :
                                                        gesture == Right2LeftSwip ? "(<--)" :
                                                        gesture == Up2DownSwip ? "up to down |" :
                                                        gesture == Down2UpSwip ? "down to up |" :
                                                        gesture == Mgestrue ? "(M)" :
														gesture == Wgestrue ? "(W)" : "oppo custom gesture"); 
							if(gesture > OPPO_CUSTOM_GESTURE_ID_BASE )
							{
								ret = gesture_event_handler(tpd->dev);
								if (ret >= 0) 
								{
									gt1x_irq_enable();
									mutex_unlock(&i2c_access);
									continue;
								}
							}
							else
							{
								gesture_upload=gesture;
								input_report_key(tpd->dev, KEY_F4, 1);
								input_sync(tpd->dev);
								input_report_key(tpd->dev, KEY_F4, 0);
								input_sync(tpd->dev);
								clear_buf[0] = 0x00;
								gt1x_i2c_write(GTP_REG_WAKEUP_GESTURE, clear_buf, 1);

							}

					}
					else       
					{
					        GTP_ERROR("report gesture::Unknow gesture!!!\n");
					     	//clear_buf[0] = 0x00;
							//gt1x_i2c_write(GTP_REG_WAKEUP_GESTURE, clear_buf, 1);	
							//gesture_enter_doze();
					}	
				}
             gt1x_irq_enable();
		     mutex_unlock(&i2c_access);
             continue;
       }
#endif
#endif
		if (gt1x_halt) {
			mutex_unlock(&i2c_access);
			GTP_ERROR("Ignore interrupts after suspend.");
			continue;
		}

		/* read coordinates */
		ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
		if (ret < 0) {
			GTP_ERROR("I2C transfer error!");
#if !GTP_ESD_PROTECT
			gt1x_power_reset();
#endif
			gt1x_irq_enable();
			mutex_unlock(&i2c_access);
            continue;
		}
		
		finger = point_data[0];

		/* response to a ic request */
		if (finger == 0x00) {
			gt1x_request_event_handler();
		}

		if ((finger & 0x80) == 0) {
#if HOTKNOT_BLOCK_RW
			if (!hotknot_paired_flag)
#endif
			{
				gt1x_irq_enable();
				mutex_unlock(&i2c_access);
				//GTP_ERROR("buffer not ready:0x%02x", finger);
				continue;
			}
		}
#if HOTKNOT_BLOCK_RW
		ret = hotknot_event_handler(point_data);
		if (!ret) {
			goto exit_work_func;
		}
#endif

#if GTP_PROXIMITY
		ret = gt1x_prox_event_handler(point_data);
		if (ret > 0) {
			goto exit_work_func;
		}
#endif

#if GTP_WITH_STYLUS
		ret = gt1x_touch_event_handler(point_data, tpd->dev, pen_dev);
#else
		ret = gt1x_touch_event_handler(point_data, tpd->dev, NULL);
#endif

		/*if (ret) {
			gt1x_irq_enable();
			mutex_unlock(&i2c_access);
			continue;
		}*/

exit_work_func:
		if (!gt1x_rawdiff_mode && (ret >= 0 || ret == ERROR_VALUE)) {
			ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
			if (ret < 0) {
				GTP_INFO("I2C write end_cmd  error!");
			}
		}
		gt1x_irq_enable();
		mutex_unlock(&i2c_access);
	} while (!kthread_should_stop());
return 0;
}

int gt1x_debug_proc(u8 * buf, int count)
{
	char mode_str[50] = { 0 };
	int mode;

	sscanf(buf, "%s %d", (char *)&mode_str, &mode);

	/***********POLLING/EINT MODE switch****************/
	if (strcmp(mode_str, "polling") == 0) {
		if (mode >= 10 && mode <= 200) {
			GTP_INFO("Switch to polling mode, polling time is %d", mode);
			tpd_eint_mode = 0;
			tpd_polling_time = mode;
			tpd_flag = 1;
			wake_up_interruptible(&waiter);
		} else {
			GTP_INFO("Wrong polling time, please set between 10~200ms");
		}
		return count;
	}
	if (strcmp(mode_str, "eint") == 0) {
		GTP_INFO("Switch to eint mode");
		tpd_eint_mode = 1;
		return count;
	}
	/**********************************************/
	if (strcmp(mode_str, "switch") == 0) {
		if (mode == 0)	// turn off
			tpd_off();
		else if (mode == 1)	//turn on
			tpd_on();
		else
			GTP_ERROR("error mode :%d", mode);
		return count;
	}

	return -1;
}

static u16 convert_productname(u8 * name)
{
	int i;
	u16 product = 0;
	for (i = 0; i < 4; i++) {
		product <<= 4;
		if (name[i] < '0' || name[i] > '9') {
			product += '*';
		} else {
			product += name[i] - '0';
		}
	}
	return product;
}

static int tpd_i2c_remove(struct i2c_client *client)
{
	gt1x_deinit();
	return 0;
}

static int tpd_local_init(void)
{

#if TPD_SUPPORT_I2C_DMA
	mutex_init(&dma_mutex);
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *) dma_alloc_coherent(&tpd->dev->dev, IIC_DMA_MAX_TRANSFER_SIZE, &gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va) {
		GTP_ERROR("Allocate DMA I2C Buffer failed!");
		return -1;
	}
	memset(gpDMABuf_va, 0, IIC_DMA_MAX_TRANSFER_SIZE);
#endif
	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		GTP_ERROR("unable to add i2c driver.");
		return -1;
	}

	if (tpd_load_status == 0)	// disable auto load touch driver for linux3.0 porting
	{
		GTP_ERROR("add error touch panel driver.");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	//input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (GTP_MAX_TOUCH - 1), 0, 0);
	//input_mt_init_slots(tpd->dev, GTP_MAX_TOUCH,0);
#if TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);	// initialize tpd button data
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif

	// set vendor string
	tpd->dev->id.vendor = 0x00;
	tpd->dev->id.product = convert_productname(gt1x_version.product_id);
	tpd->dev->id.version = (gt1x_version.patch_id >> 8);

	GTP_INFO("end %s, %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

static ssize_t tp_devices_check_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos)
{
	char pagesize[512];
	int devices_check_test=0;
	devices_check_test = sprintf(pagesize, "%d\n", GT_TP_ID);
	devices_check_test = simple_read_from_buffer(page, size, ppos, pagesize, strlen(pagesize)); 
	
	return devices_check_test; 
}

/*
static ssize_t tp_devices_check_write_func(struct file *file,const char *buffer, unsigned long count,void *data)
{
    int len = 0;
	glove_check =1;
	printk("glove check is: %d\n", glove_check);
	len = sprintf(page, "%d\n", atomic_read(&glove_check));
	return len;
}
*/


#ifdef SUPPORT_GESTURE
static ssize_t tp_double_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos)
{                           
    char pagesize[512];
    int len = 0;
	GTP_INFO("double tap enable is: %d\n", atomic_read(&double_enable));
	len = sprintf(pagesize, "%d\n", atomic_read(&double_enable));
	len = simple_read_from_buffer(page, size, ppos, pagesize, strlen(pagesize)); 
	return len; 
}

static ssize_t tp_double_write_func(struct file *file, const char __user *buffer, size_t count,loff_t *ppos)
{
	int ret = 0;

	char buf[10] = {0};
	static int in_suspend_gustrue_status;
	
	if (count > 10) 
		return count;

	if (copy_from_user( buf, buffer, count)) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	 
	sscanf(buf,"%d",&ret);

	//down(&suspend_sem);
	mutex_lock(&gt_suspend_lock);
	GTP_INFO("double_write_func  %d\n",ret);
	if( atomic_read(&is_in_suspend) )
	{
		if(in_suspend_gustrue_status == ret)
		{
			GTP_INFO("do not need operate when gesture status is same\n");
			//up(&suspend_sem);
			mutex_unlock(&gt_suspend_lock);
			return count;
		}

		in_suspend_gustrue_status = ret; 
		switch(ret)
		{
			case 0:
				gt1x_wakeup_sleep_gesture();
				gt1x_enter_sleep_gesture();
				break;
			case 1:
				gt1x_wakeup_sleep_gesture();
				gesture_enter_doze_gesture();
				break;


			default:
				GTP_INFO("Please enter 0 or 1 to open or close the double-tap function\n");
		}
		//up(&suspend_sem);
		mutex_unlock(&gt_suspend_lock);
		return count;//can not save double_enable flag when is_in_suspend
		
	}
	else
	{
		switch(ret)
		{	
			case 0:
				GTP_DEBUG("tp_guesture_func will be disable\n");
				break;
			case 1:
				GTP_DEBUG("tp_guesture_func will be enable\n");
				break;
			default:
				GTP_DEBUG("Please enter 0 or 1 to open or close the double-tap function\n");
		}
	}
	//up(&suspend_sem);
	mutex_unlock(&gt_suspend_lock);

	if((ret == 0 )||(ret == 1))
	{
		atomic_set(&double_enable,ret);
		in_suspend_gustrue_status = ret;
	}

	return count;
}

#ifdef SUPPORT_REPORT_COORDINATE
static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int len = 0;
	char page[512];

	len =sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
			Point_start.x, Point_start.y, Point_end.x, Point_end.y,
			Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
			Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
			clockwise);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

	GTP_INFO("%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
			Point_start.x, Point_start.y, Point_end.x, Point_end.y,
			Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
			Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
			clockwise);
   printk("return ret=%d len=%d\n",ret,len);

	return ret;
}
#endif
#endif

static int init_goodix_proc(void)
{
	int ret = 0;
	
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if(prEntry_tp == NULL)
	{
		ret = -ENOMEM;
	  	GTP_ERROR(KERN_INFO"init_gt1x_proc: Couldn't create TP proc entry\n");
		return ret;
	}
	
	prEntry_dtap =proc_create("TP_AUTO_TEST_ID", 0777, prEntry_tp, &gt1x_devices_check);
	if(prEntry_dtap == NULL)
	{
		ret = -ENOMEM;
	  	GTP_ERROR(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		return ret;
	}
	
#ifdef SUPPORT_GESTURE
	prEntry_dtap = proc_create("double_tap_enable", 0777, prEntry_tp, &gt1x_gesture);
	if(prEntry_dtap == NULL)
	{
		ret = -ENOMEM;
	  	GTP_ERROR(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		return ret;
	}
	GTP_INFO("create gt1x gesture proc success\n");
#endif

#ifdef SUPPORT_REPORT_COORDINATE
	prEntry_coodinate =  proc_create("coordinate", 0777, prEntry_tp, &gt1x_gesture_coor);
    if(prEntry_coodinate == NULL)
    {	   
		ret = -ENOMEM;	   
		GTP_ERROR(KERN_INFO"init_gt1x_proc: Couldn't create proc entry\n");
		return ret;
    }
	GTP_INFO("create gt1x gesture_coor proc success\n");
    
#endif


return 0;
}
/* Function to manage low power suspend */
static void tpd_suspend(struct early_suspend *h)
{
	if((boot_mode == META_BOOT) || (boot_mode == FACTORY_BOOT  )){
		return ;
	}
	mutex_lock(&gt_suspend_lock);
	atomic_set(&is_in_suspend,1);
	gt1x_suspend();
	mutex_unlock(&gt_suspend_lock);
}

/* Function to manage power-on resume */
static void tpd_resume(struct early_suspend *h)
{

	if((boot_mode == META_BOOT) || (boot_mode == FACTORY_BOOT  )){
		return ;
	}
	mutex_lock(&gt_suspend_lock);
	gt1x_rawdiff_mode=0;
	atomic_set(&is_in_suspend,0);
	gt1x_resume();
	mutex_unlock(&gt_suspend_lock);
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "gt1x",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

void tpd_off(void)
{
	gt1x_power_switch(SWITCH_OFF);
	gt1x_halt = 1;
	gt1x_irq_disable();
}

void tpd_on(void)
{
	s32 ret = -1, retry = 0;

	while (retry++ < 5) {
		ret = tpd_power_on();
		if (ret < 0) {
			GTP_ERROR("I2C Power on ERROR!");
		}

		ret = gt1x_send_cfg(gt1x_config, gt1x_cfg_length);
		if (ret == 0) {
			GTP_DEBUG("Wakeup sleep send gt1x_config success.");
			break;
		}
	}
	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
	//gt1x_irq_enable();
	gt1x_halt = 0;
}

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	int ID1, ID2;
	
	GTP_ERROR("Goodix::tpd_driver_init\n");

	ID1 = mt_get_gpio_in(GPIO_TP_ID1_16021);//gpio21
	ID2 = mt_get_gpio_in(GPIO_TP_ID2_16021);//gpio19
	
	printk("ID1=%d ID2=%d\n", ID1,ID2);
	if( ID1==1&&ID2==0 ){
		printk("TP IS OFILM\n");
		firmware_id=0xBC021100;
		GT_TP_ID=0;
		strcpy(manu_name, "TP_OFILM");
	}else if(ID1==0&&ID2==1){
		printk("TP IS BIEL\n");
		firmware_id=0xBC021600;
		GT_TP_ID=1;
		strcpy(manu_name, "TP_BIEL");
	} 
	
	i2c_register_board_info(TPD_I2C_NUMBER, &i2c_tpd, 1);

	if (tpd_driver_add(&tpd_device_driver) < 0) {
		GTP_ERROR("add generic driver failed.");
	}

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	GTP_INFO("MediaTek GT1x touch panel driver exit.");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
