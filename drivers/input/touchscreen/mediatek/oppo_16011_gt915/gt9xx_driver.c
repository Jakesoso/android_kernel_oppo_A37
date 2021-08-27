/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *  
 * MediaTek Inc. (C) 2012. All rights reserved. 
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 *
 * Version: V2.5
 * Release Date: 2015/01/21
 */

#include "tpd.h"
#include "tpd_custom_gt9xx.h"
#include "circle_point.h"
#include <linux/oppo_devices_list.h>



#ifndef TPD_NO_GPIO
#include "cust_gpio_usage.h" 
#endif
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#ifdef CONFIG_OF_TOUCH
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#if GTP_SUPPORT_I2C_DMA
    #include <linux/dma-mapping.h>
#endif

#ifdef CONFIG_OF_TOUCH
static int irq_flag;
static unsigned int touch_irq = 0;
#endif

extern struct tpd_device *tpd;
extern u8 gtp_loading_fw;

static int tpd_flag = 0; 
int tpd_halt = 0;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
#define PAGESIZE  512
//chenggang.li@bsp add for oppo
/*******************************************************start************************************************/
#define TP_UNKNOWN 5
#define TP_BIEL 4
#define TP_G2Y 3
#define TP_TPK 2
#define TP_OFILM 1
#define TP_TRULY 0
extern int TP_FW;
extern TP_DEV oppo_tp_dev;

#define GT9XX_SUPPORT_APK_AUTOTEST   //add the proc for apk , tp autotest.
#ifdef GT9XX_SUPPORT_APK_AUTOTEST
static struct proc_dir_entry *prEntry_glove_check = NULL;
#endif

#ifdef GT9XX_SUPPORT_APK_AUTOTEST
static int glove_check = 0;
#endif

u8 sensor_id = 0;

/************************************************************end**************************************************/



#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if GTP_GESTURE_WAKEUP
typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct i2c_client *client);
#endif

#if GTP_CHARGER_SWITCH
    #ifdef MT6573
        #define CHR_CON0      (0xF7000000+0x2FA00)
    #else
        extern kal_bool upmu_is_chr_det(void);
    #endif
    static void gtp_charger_switch(s32 dir_update);
#endif 

#if GTP_HAVE_TOUCH_KEY
const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )
#endif


#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
//static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif


#if GTP_SUPPORT_I2C_DMA
static s32 i2c_dma_write(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len);
static s32 i2c_dma_read(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len);

static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
#endif


s32 gtp_send_cfg(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);


#ifdef CONFIG_OF_TOUCH
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc);
#else
static void tpd_eint_interrupt_handler(void);
#endif



static int touch_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);

#ifndef MT6572
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif


#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif


#if (GTP_ESD_PROTECT || GTP_COMPATIBLE_MODE)
static void force_reset_guitar(void);
#endif


#if GTP_ESD_PROTECT
static int clk_tick_cnt = 200;
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
static void gtp_esd_check_func(struct work_struct *);
void gtp_esd_switch(struct i2c_client *client, s32 on);
u8 esd_running = 0;
spinlock_t esd_lock;
#endif


#ifdef TPD_PROXIMITY
#define TPD_PROXIMITY_VALID_REG                   0x814E
#define TPD_PROXIMITY_ENABLE_REG                  0x8042
static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
#endif


//chenggang.li@bsp add for oppo
#define GT9XX_UPDATE_FILE_PATH  "/system/etc/firmware/"
char gt9xx_updata_file_path[128] = {0}; // fw patch.
#define SUPPORT_GESTURE
#define SUPPORT_REPORT_COORDINATE
#define GTP_LARGE_AREA_SLEEP            0 
#ifdef SUPPORT_GESTURE

struct proc_dir_entry *prEntry_tp = NULL; 
static struct proc_dir_entry *prEntry_dtap = NULL;
static struct proc_dir_entry *prEntry_coodinate  = NULL; 
static int init_goodix_proc(void);
//static ssize_t tp_double_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
//static ssize_t tp_double_write_func(struct file *filp, const char __user *buffer, size_t count, loff_t *off);
static int coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos);
static int glove_mode_read_func(struct file *file, char __user * page, size_t size, loff_t * ppos);
static int glove_mode_write_func(struct file *file,const char *buffer, unsigned long count,void *data);
static atomic_t double_enable;


#define DTAP_DETECT          0xCC
#define UP_VEE_DETECT        0x76
#define DOWN_VEE_DETECT      0x5e
#define LEFT_VEE_DETECT      0x3e
#define RIGHT_VEE_DETECT     0x63
#define CIRCLE_DETECT        0x6f
#define DOUSWIP_DETECT       0x48
#define RIGHT_SLIDE_DETECT   0xAA
#define LEFT_SLIDE_DETECT    0xbb
#define DOWN_SLIDE_DETECT    0xAB
#define UP_SLIDE_DETECT      0xBA
#define M_DETECT             0x6D
#define W_DETECT             0x77


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
static uint32_t gesture;

struct Coordinate
{
    int x;
    int y;
};
static  Point Point_input[64];
static  Point Point_output[5];
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
static uint32_t clockwise;



#endif

//............end...........



struct i2c_client *i2c_client_point = NULL;
static const struct i2c_device_id tpd_i2c_id[] = {{"gt9xx", 0}, {}};
static unsigned short force[] = {0, 0x28, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};
static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("gt9xx", (0x28 >> 1))};


static struct i2c_driver tpd_i2c_driver =
{
    .probe = tpd_i2c_probe,
    .remove = tpd_i2c_remove,
    .detect = tpd_i2c_detect,
    .driver.name = "gt9xx",
    .id_table = tpd_i2c_id,
    .address_list = (const unsigned short *) forces,
};



static u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
    = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
#if GTP_CHARGER_SWITCH
static u8 gtp_charger_config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
	= {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
#endif


#pragma pack(1)
typedef struct
{
    u16 pid;                 //product id   //
    u16 vid;                 //version id   //
} st_tpd_info;
#pragma pack()

st_tpd_info tpd_info;
u8 int_type = 0;
u32 abs_x_max = 0;
u32 abs_y_max = 0;
u8 gtp_rawdiff_mode = 0;
u8 cfg_len = 0;
u8 pnl_init_error = 0;

#if GTP_WITH_PEN
struct input_dev *pen_dev;
#endif

#if GTP_COMPATIBLE_MODE
u8 driver_num = 0;
u8 sensor_num = 0;
u8 gtp_ref_retries = 0;
u8 gtp_clk_retries = 0;
CHIP_TYPE_T gtp_chip_type = CHIP_TYPE_GT9;
u8 rqst_processing = 0;
u8 is_950 = 0;

extern u8 gup_check_fs_mounted(char *path_name);
extern u8 gup_clk_calibration(void);
extern s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
void gtp_get_chip_type(struct i2c_client *client);
u8 gtp_fw_startup(struct i2c_client *client);
static u8 gtp_bak_ref_proc(struct i2c_client *client, u8 mode);
static u8 gtp_main_clk_proc(struct i2c_client *client);
static void gtp_recovery_reset(struct i2c_client *client);
#endif

/* proc file system */
s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *, size_t, loff_t *);
static s8 gtp_enter_sleep(struct i2c_client *client);
static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xx_config_read_proc,
    .write = gt91xx_config_write_proc,
};

#define VELOCITY_CUSTOM
#ifdef VELOCITY_CUSTOM
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

// for magnify velocity********************************************
#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x = TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y = TPD_VELOCITY_CUSTOM_Y;
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
    //char strbuf[256];
    void __user *data;

    long err = 0;

    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if (err)
    {
        printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch (cmd)
    {
        case TPD_GET_VELOCITY_CUSTOM_X:
            data = (void __user *) arg;

            if (data == NULL)
            {
                err = -EINVAL;
                break;
            }

            if (copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
            {
                err = -EFAULT;
                break;
            }

            break;

        case TPD_GET_VELOCITY_CUSTOM_Y:
            data = (void __user *) arg;

            if (data == NULL)
            {
                err = -EINVAL;
                break;
            }

            if (copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
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


static struct file_operations tpd_fops =
{
//  .owner = THIS_MODULE,
    .open = tpd_misc_open,
    .release = tpd_misc_release,
    .unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "touch",
    .fops = &tpd_fops,
};

//**********************************************
#endif

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, "mtk-tpd");
    return 0;
}

#ifdef TPD_PROXIMITY
static s32 tpd_get_ps_value(void)
{
    return tpd_proximity_detect;
}

static s32 tpd_enable_ps(s32 enable)
{
    u8  state;
    s32 ret = -1;

    if (enable)
    {
        state = 1;
        tpd_proximity_flag = 1;
        GTP_INFO("TPD proximity function to be on.");
    }
    else
    {
        state = 0;
        tpd_proximity_flag = 0;
        GTP_INFO("TPD proximity function to be off.");
    }

    ret = i2c_write_bytes(i2c_client_point, TPD_PROXIMITY_ENABLE_REG, &state, 1);

    if (ret < 0)
    {
        GTP_ERROR("TPD %s proximity cmd failed.", state ? "enable" : "disable");
        return ret;
    }

    GTP_INFO("TPD proximity function %s success.", state ? "enable" : "disable");
    return 0;
}

s32 tpd_ps_operate(void *self, u32 command, void *buff_in, s32 size_in,
                   void *buff_out, s32 size_out, s32 *actualout)
{
    s32 err = 0;
    s32 value;
    hwm_sensor_data *sensor_data;

    switch (command)
    {
        case SENSOR_DELAY:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                GTP_ERROR("Set delay parameter error!");
                err = -EINVAL;
            }

            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                GTP_ERROR("Enable sensor parameter error!");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                err = tpd_enable_ps(value);
            }

            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data)))
            {
                GTP_ERROR("Get sensor data parameter error!");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = tpd_get_ps_value();
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }

            break;

        default:
            GTP_ERROR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}
#endif


static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    char *ptr = page;
    char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0};
    int i;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
    
    ptr += sprintf(ptr, "==== GT9XX config init value====\n");

    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", config[i + 2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");

    ptr += sprintf(ptr, "==== GT9XX config real value====\n");
    i2c_read_bytes(i2c_client_point, GTP_REG_CONFIG_DATA, temp_data, GTP_CONFIG_MAX_LENGTH);

    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", temp_data[i]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }
    *ppos += ptr - page;
    return (ptr - page);
}

static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    s32 ret = 0;

    GTP_DEBUG("write count %zd\n", count);

    if (count > GTP_CONFIG_MAX_LENGTH)
    {
        GTP_ERROR("size not match [%d:%zd]\n", GTP_CONFIG_MAX_LENGTH, count);
        return -EFAULT;
    }

    if (copy_from_user(&config[2], buffer, count))
    {
        GTP_ERROR("copy from user fail\n");
        return -EFAULT;
    }

    ret = gtp_send_cfg(i2c_client_point);
    abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
    abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
    int_type = (config[TRIGGER_LOC]) & 0x03;

    if (ret < 0)
    {
        GTP_ERROR("send config failed.");
    }

    return count;
}

#if GTP_SUPPORT_I2C_DMA
static s32 i2c_dma_read(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len)
{
    int ret;
    s32 retry = 0;
    u8 buffer[2];

    struct i2c_msg msg[2] =
    {
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .flags = 0,
            .buf = buffer,
            .len = 2,
            .timing = I2C_MASTER_CLOCK
        },
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            .buf = (u8*)gpDMABuf_pa,     
            .len = len,
            .timing = I2C_MASTER_CLOCK
        },
    };
    
    buffer[0] = (addr >> 8) & 0xFF;
    buffer[1] = addr & 0xFF;

    if (rxbuf == NULL)
        return -1;

    //GTP_DEBUG("dma i2c read: 0x%04X, %d bytes(s)", addr, len);
    for (retry = 0; retry < 5; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg[0], 2);
        if (ret < 0)
        {
            continue;
        }
        memcpy(rxbuf, gpDMABuf_va, len);
        return 0;
    }
    GTP_ERROR("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
    return ret;
}


static s32 i2c_dma_write(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len)
{
    int ret;
    s32 retry = 0;
    u8 *wr_buf = gpDMABuf_va;
    
    struct i2c_msg msg =
    {
        .addr = (client->addr & I2C_MASK_FLAG),
        .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
        .flags = 0,
        .buf = (u8*)gpDMABuf_pa,
        .len = 2 + len,
        .timing = I2C_MASTER_CLOCK
    };
    
    wr_buf[0] = (u8)((addr >> 8) & 0xFF);
    wr_buf[1] = (u8)(addr & 0xFF);

    if (txbuf == NULL)
        return -1;
    
    //GTP_DEBUG("dma i2c write: 0x%04X, %d bytes(s)", addr, len);
    memcpy(wr_buf+2, txbuf, len);
    for (retry = 0; retry < 5; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret < 0)
        {
            continue;
        }
        return 0;
    }
    GTP_ERROR("Dma I2C Write Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
    return ret;
}

static s32 i2c_read_bytes_dma(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len)
{
    s32 left = len;
    s32 read_len = 0;
    u8 *rd_buf = rxbuf;
    s32 ret = 0;    
    
    //GTP_DEBUG("Read bytes dma: 0x%04X, %d byte(s)", addr, len);
    while (left > 0)
    {
        if (left > GTP_DMA_MAX_TRANSACTION_LENGTH)
        {
            read_len = GTP_DMA_MAX_TRANSACTION_LENGTH;
        }
        else
        {
            read_len = left;
        }
        ret = i2c_dma_read(client, addr, rd_buf, read_len);
        if (ret < 0)
        {
            GTP_ERROR("dma read failed");
            return -1;
        }
        
        left -= read_len;
        addr += read_len;
        rd_buf += read_len;
    }
    return 0;
}

static s32 i2c_write_bytes_dma(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len)
{

    s32 ret = 0;
    s32 write_len = 0;
    s32 left = len;
    u8 *wr_buf = txbuf;
    
    //GTP_DEBUG("Write bytes dma: 0x%04X, %d byte(s)", addr, len);
    while (left > 0)
    {
        if (left > GTP_DMA_MAX_I2C_TRANSFER_SIZE)
        {
            write_len = GTP_DMA_MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            write_len = left;
        }
        ret = i2c_dma_write(client, addr, wr_buf, write_len);
        
        if (ret < 0)
        {
            GTP_ERROR("dma i2c write failed!");
            return -1;
        }
        
        left -= write_len;
        addr += write_len;
        wr_buf += write_len;
    }
    return 0;
}
#endif


static int i2c_read_bytes_non_dma(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buffer[GTP_ADDR_LENGTH];
    u8 retry;
    u16 left = len;
    u16 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
            .flags = 0,
            .buf = buffer,
            .len = GTP_ADDR_LENGTH,
            .timing = I2C_MASTER_CLOCK
        },
        {
            .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
            .flags = I2C_M_RD,
            .timing = I2C_MASTER_CLOCK
        },
    };

    if (rxbuf == NULL)
        return -1;

    //GTP_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        msg[1].buf = &rxbuf[offset];

        if (left > MAX_TRANSACTION_LENGTH)
        {
            msg[1].len = MAX_TRANSACTION_LENGTH;
            left -= MAX_TRANSACTION_LENGTH;
            offset += MAX_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;

        while (i2c_transfer(client->adapter, &msg[0], 2) != 2)
        {
            retry++;

            //if (retry == 20)
            if (retry == 5)
            {
                GTP_ERROR("I2C read 0x%X length=%d failed\n", addr + offset, len);
                return -1;
            }
        }
    }

    return 0;
}


int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
#if GTP_SUPPORT_I2C_DMA
    return i2c_read_bytes_dma(client, addr, rxbuf, len);
#else
    return i2c_read_bytes_non_dma(client, addr, rxbuf, len);
#endif
}

s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    s32 ret = -1;
    u16 addr = (buf[0] << 8) + buf[1];

    ret = i2c_read_bytes_non_dma(client, addr, &buf[2], len - 2);

    if (!ret)
    {
        return 2;
    }
    else
    {
    #if GTP_GESTURE_WAKEUP
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == gtp_chip_type)
        {
            gtp_recovery_reset(client);
        }
        else
    #endif
        {
            gtp_reset_guitar(client, 20);
        }
        return ret;
    }
}

s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;
    
    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, buf, len + 2);
        
        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, confirm_buf, len + 2);
        
        if (!memcmp(buf, confirm_buf, len+2))
        {
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }    
    GTP_ERROR("i2c read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}

int i2c_write_bytes_non_dma(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg =
    {
        .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
        //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
        .flags = 0,
        .buf = buffer,
        .timing = I2C_MASTER_CLOCK,
    };


    if (txbuf == NULL)
        return -1;

    //GTP_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        retry = 0;

        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        if (left > MAX_I2C_TRANSFER_SIZE)
        {
            memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], MAX_I2C_TRANSFER_SIZE);
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], left);
            msg.len = left + GTP_ADDR_LENGTH;
            left = 0;
        }

        //GTP_DEBUG("byte left %d offset %d\n", left, offset);

        while (i2c_transfer(client->adapter, &msg, 1) != 1)
        {
            retry++;

            //if (retry == 20)
            if (retry == 5)
            {
                GTP_ERROR("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return -1;
            }
        }
    }

    return 0;
}

int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
#if GTP_SUPPORT_I2C_DMA
    return i2c_write_bytes_dma(client, addr, txbuf, len);
#else
    return i2c_write_bytes_non_dma(client, addr, txbuf, len);
#endif
}

s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
    s32 ret = -1;
    u16 addr = (buf[0] << 8) + buf[1];

    ret = i2c_write_bytes_non_dma(client, addr, &buf[2], len - 2);

    if (!ret)
    {
        return 1;
    }
    else
    {
    #if GTP_GESTURE_WAKEUP
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    #endif
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == gtp_chip_type)
        {
            gtp_recovery_reset(client);
        }
        else
    #endif
        {
            gtp_reset_guitar(client, 20);
        }
        return ret;
    }
}


#if TOUCH_VIRTUAL_KEYS
//#define KEY_MENU	139
//#define KEY_HOMEPAGE	172
//#define KEY_BACK	158
//#define GTP_KEY_SEARCH  217

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	 __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":150:1370:100:50"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":360:1370:100:50"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":590:1370:100:50"
	 "\n");
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.mtk-tpd",//这里要与输入子系统一致
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};


/* gtp_ts_virtual_keys_init --  register virutal keys to system
 * @return: none
 */
static void gtp_ts_virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;
	
    GTP_DEBUG_FUNC();
	
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");    
}
#endif
/*******************************************************
Function:
    Send config Function.

Input:
    client: i2c client.

Output:
    Executive outcomes.0--success,non-0--fail.
*******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 1;

#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;

	if (pnl_init_error)
    {
        GTP_INFO("Error occurred in init_panel, no config sent!");
        return 0;
    }
    
    GTP_INFO("Driver Send Config");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);

        if (ret > 0)
        {
            break;
        }
    }
#endif
    return ret;
}
#if GTP_CHARGER_SWITCH
static int gtp_send_chr_cfg(struct i2c_client *client)
{
	s32 ret = 1;
#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;

	if (pnl_init_error) {
        GTP_INFO("Error occurred in init_panel, no config sent!");
        return 0;
    }
    
    GTP_INFO("Driver Send Config");
    for (retry = 0; retry < 5; retry++) {
        ret = gtp_i2c_write(client, gtp_charger_config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0) {
            break;
        }
    }
#endif	
	return ret;
}
#endif
/*******************************************************
Function:
    Read goodix touchscreen version function.

Input:
    client: i2c client struct.
    version:address to store version info

Output:
    Executive outcomes.0---succeed.
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16 *version)
{
    s32 ret = -1;
    s32 i;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));

    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }

    tpd_info.vid = *version;
    tpd_info.pid = 0x00;

    for (i = 0; i < 4; i++)
    {
        if (buf[i + 2] < 0x30)break;

        tpd_info.pid |= ((buf[i + 2] - 0x30) << ((3 - i) * 4));
    }

    if (buf[5] == 0x00)
    {        
        GTP_INFO("IC VERSION: %c%c%c_%02x%02x",
             buf[2], buf[3], buf[4], buf[7], buf[6]);  
    }
    else
    {
        GTP_INFO("IC VERSION:%c%c%c%c_%02x%02x",
             buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;
}

/*******************************************************
Function:
    GTP initialize function.

Input:
    client: i2c client private struct.

Output:
    Executive outcomes.0---succeed.
*******************************************************/
static s32 gtp_init_panel(struct i2c_client *client)
{
    s32 ret = 0;

#if GTP_DRIVER_SEND_CFG
    s32 i;
    u8 check_sum = 0;
    u8 opr_buf[16];
    u8 sensor_id = 0;
	 u8 drv_cfg_version;
	 u8 flash_cfg_version;

    u8 cfg_info_group0[] = CTP_CFG_GROUP0;
    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 cfg_info_group4[] = CTP_CFG_GROUP4;
    u8 cfg_info_group5[] = CTP_CFG_GROUP5;
    u8 *send_cfg_buf[] = {cfg_info_group0, cfg_info_group1, cfg_info_group2,
                        cfg_info_group3, cfg_info_group4, cfg_info_group5};
    u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group0), 
                          CFG_GROUP_LEN(cfg_info_group1),
                          CFG_GROUP_LEN(cfg_info_group2),
                          CFG_GROUP_LEN(cfg_info_group3), 
                          CFG_GROUP_LEN(cfg_info_group4),
                          CFG_GROUP_LEN(cfg_info_group5)};
#if GTP_CHARGER_SWITCH
	const u8 cfg_grp0_charger[] = GTP_CFG_GROUP0_CHARGER;
	const u8 cfg_grp1_charger[] = GTP_CFG_GROUP1_CHARGER;
	const u8 cfg_grp2_charger[] = GTP_CFG_GROUP2_CHARGER;
	const u8 cfg_grp3_charger[] = GTP_CFG_GROUP3_CHARGER;
	const u8 cfg_grp4_charger[] = GTP_CFG_GROUP4_CHARGER;
	const u8 cfg_grp5_charger[] = GTP_CFG_GROUP5_CHARGER;
	const u8 *cfgs_charger[] = {
		cfg_grp0_charger, cfg_grp1_charger, cfg_grp2_charger,
		cfg_grp3_charger, cfg_grp4_charger, cfg_grp5_charger
	};
	u8 cfg_lens_charger[] = {
						CFG_GROUP_LEN(cfg_grp0_charger),
						CFG_GROUP_LEN(cfg_grp1_charger),
						CFG_GROUP_LEN(cfg_grp2_charger),
						CFG_GROUP_LEN(cfg_grp3_charger),
						CFG_GROUP_LEN(cfg_grp4_charger),
						CFG_GROUP_LEN(cfg_grp5_charger)};
#endif

    GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d", 
        cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        cfg_info_len[4], cfg_info_len[5]);

    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) && 
        (!cfg_info_len[3]) && (!cfg_info_len[4]) && 
        (!cfg_info_len[5]))
    {
        sensor_id = 0; 
    }
    else
    {
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == gtp_chip_type)
        {
            msleep(50);
        }
    #endif
        ret = gtp_i2c_read_dbl_check(client, GTP_REG_SENSOR_ID, &sensor_id, 1);
        if (SUCCESS == ret)
        {
            if (sensor_id >= 0x06)
            {
                GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
                pnl_init_error = 1;
                return -1;
            }
        }
        else
        {
            GTP_ERROR("Failed to get sensor_id, No config sent!");
            pnl_init_error = 1;
            return -1;
        }
        GTP_INFO("Sensor_ID: %d", sensor_id);
    }
    
    cfg_len = cfg_info_len[sensor_id];
    
    GTP_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id, cfg_len);
    
    if (cfg_len < GTP_CONFIG_MIN_LENGTH)
    {
        GTP_ERROR("CTP_CONFIG_GROUP%d is INVALID CONFIG GROUP! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id);
        pnl_init_error = 1;
        return -1;
    }
    
#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F != gtp_chip_type)
#endif
	{
	    ret = gtp_i2c_read_dbl_check(client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);    
	    if (ret == SUCCESS)
	    {
	        GTP_DEBUG("CFG_CONFIG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id, 
	                    send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);
	
			flash_cfg_version = opr_buf[0];
			drv_cfg_version = send_cfg_buf[sensor_id][0];       // backup  config version
			
	        if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {
	            send_cfg_buf[sensor_id][0] = 0x00;
	        }
	    }
	    else
	    {
	        GTP_ERROR("Failed to get ic config version!No config sent!");
	        return -1;
	    }
	}  
    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], cfg_len);

#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);
    
    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe; 
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
#endif  // GTP_CUSTOM_CFG

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < cfg_len; i++)
	{
		check_sum += config[i];
	}
	config[cfg_len] = (~check_sum) + 1;

#if GTP_CHARGER_SWITCH
	GTP_DEBUG("Charger Config Groups Length: %d, %d, %d, %d, %d, %d", cfg_lens_charger[0],
		  cfg_lens_charger[1], cfg_lens_charger[2], cfg_lens_charger[3], cfg_lens_charger[4], cfg_lens_charger[5]);

	memset(&gtp_charger_config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	if (cfg_lens_charger[sensor_id] == cfg_len) 
		memcpy(&gtp_charger_config[GTP_ADDR_LENGTH], cfgs_charger[sensor_id], cfg_len);

#if GTP_CUSTOM_CFG
	gtp_charger_config[RESOLUTION_LOC] = (u8) GTP_MAX_WIDTH;
	gtp_charger_config[RESOLUTION_LOC + 1] = (u8) (GTP_MAX_WIDTH >> 8);
	gtp_charger_config[RESOLUTION_LOC + 2] = (u8) GTP_MAX_HEIGHT;
	gtp_charger_config[RESOLUTION_LOC + 3] = (u8) (GTP_MAX_HEIGHT >> 8);

	if (GTP_INT_TRIGGER == 0) 	/* RISING  */
		gtp_charger_config[TRIGGER_LOC] &= 0xfe;
	else if (GTP_INT_TRIGGER == 1) /* FALLING */
		gtp_charger_config[TRIGGER_LOC] |= 0x01;
#endif /* END GTP_CUSTOM_CFG */
	if (cfg_lens_charger[sensor_id] != cfg_len) 
		memset(&gtp_charger_config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);	
	
	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < cfg_len; i++)
	{
		check_sum += gtp_charger_config[i];
	}
	gtp_charger_config[cfg_len] = (~check_sum) + 1;

#endif /* END GTP_CHARGER_SWITCH */    
  
#else // DRIVER NOT SEND CONFIG
    cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(client, config, cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("Read Config Failed, Using DEFAULT Resolution & INT Trigger!");
        abs_x_max = GTP_MAX_WIDTH;
        abs_y_max = GTP_MAX_HEIGHT;
        int_type = GTP_INT_TRIGGER;
    }
#endif // GTP_DRIVER_SEND_CFG

    GTP_DEBUG_FUNC();
    if ((abs_x_max == 0) && (abs_y_max == 0))
    {
        abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        int_type = (config[TRIGGER_LOC]) & 0x03; 
    }
    
#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        u8 have_key = 0;
        if (is_950)
        {
            driver_num = config[GTP_REG_MATRIX_DRVNUM - GTP_REG_CONFIG_DATA + 2];
            sensor_num = config[GTP_REG_MATRIX_SENNUM - GTP_REG_CONFIG_DATA + 2];
        }
        else
        {
            driver_num = (config[CFG_LOC_DRVA_NUM]&0x1F) + (config[CFG_LOC_DRVB_NUM]&0x1F);
            sensor_num = (config[CFG_LOC_SENS_NUM]&0x0F) + ((config[CFG_LOC_SENS_NUM]>>4)&0x0F);
        }
        
        have_key = config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01;  // have key or not
        if (1 == have_key)
        {
            driver_num--;
        }
        
        GTP_INFO("Driver * Sensor: %d * %d(Key: %d), X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            driver_num, sensor_num, have_key, abs_x_max,abs_y_max,int_type);
    }
    else
#endif
    {
#if GTP_DRIVER_SEND_CFG
        ret = gtp_send_cfg(client);
        if (ret < 0)
        {
            GTP_ERROR("Send config error.");
        }
#if GTP_COMPATIBLE_MODE
  	  if (CHIP_TYPE_GT9F != gtp_chip_type)
#endif
	  {
        /* for resume to send config */
		if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {		
	        config[GTP_ADDR_LENGTH] = drv_cfg_version;
	        check_sum = 0;
	        for (i = GTP_ADDR_LENGTH; i < cfg_len; i++)
	        {
	            check_sum += config[i];
	        }
	        config[cfg_len] = (~check_sum) + 1;
		}
	  }
#endif
        GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            abs_x_max,abs_y_max,int_type);
    }
    
    msleep(10);
    return 0;
}

static s8 gtp_i2c_test(struct i2c_client *client)
{

    u8 retry = 0;
    s8 ret = -1;
    u32 hw_info = 0;

    GTP_DEBUG_FUNC();

    while (retry++ < 5)
    {
        ret = i2c_read_bytes(client, GTP_REG_HW_INFO, (u8 *)&hw_info, sizeof(hw_info));

        if ((!ret) && (hw_info == 0x00900600))              //20121212
        {
            return ret;
        }

        GTP_ERROR("GTP_REG_HW_INFO : %08X", hw_info);
        GTP_ERROR("GTP i2c test failed time %d.", retry);
        msleep(10);
    }

    return -1;
}



/*******************************************************
Function:
    Set INT pin  as input for FW sync.

Note:
  If the INT is high, It means there is pull up resistor attached on the INT pin.
  Pull low the INT pin manaully for FW sync.
*******************************************************/
void gtp_int_sync(s32 ms)
{
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(ms);
    GTP_GPIO_AS_INT(GTP_INT_PORT);
}

void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
    
	GTP_INFO("GTP RESET!\n");
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    msleep(ms);
    GTP_GPIO_OUTPUT(GTP_INT_PORT, client->addr == 0x14);

    msleep(2);
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

    msleep(6);                      //must >= 6ms

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        return;
    }
#endif

    gtp_int_sync(50); 
#if GTP_ESD_PROTECT
    gtp_init_ext_watchdog(i2c_client_point);
#endif


}

static int tpd_power_on(struct i2c_client *client)
{
    int ret = 0;
    int reset_count = 0;

reset_proc:
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);   
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(10);

#ifdef MT6573
    // power on CTP
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);

#else   // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )

    #ifdef TPD_POWER_SOURCE_CUSTOM                           
        hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");    
    #else
        hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    #endif
    #ifdef TPD_POWER_SOURCE_1800
        hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
    #endif

#endif

    gtp_reset_guitar(client, 20);

#if GTP_COMPATIBLE_MODE
    gtp_get_chip_type(client);
    
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN);
    
        if(FAIL == ret)
        {
            GTP_ERROR("[tpd_power_on]Download fw failed.");
            if(reset_count++ < TPD_MAX_RESET_COUNT)
            {
                goto reset_proc;
            }
            else
            {
                return -1;
            }
        }
        
        ret = gtp_fw_startup(client);
        if(FAIL == ret)
        {
            GTP_ERROR("[tpd_power_on]Startup fw failed.");
            if(reset_count++ < TPD_MAX_RESET_COUNT)
            {
                goto reset_proc;
            }
            else
            {
                return -1;
            }
        }
    }
    else  
#endif
    {
        ret = gtp_i2c_test(client);
    
        if (ret < 0)
        {
            GTP_ERROR("I2C communication ERROR!");
    
            if (reset_count < TPD_MAX_RESET_COUNT)
            {
                reset_count++;
                goto reset_proc;
            }
        }
    }
    return ret;
}


void gtp_irq_enable(void)
{
	if(irq_flag==0){
		irq_flag++;
#ifdef CONFIG_OF_TOUCH
		enable_irq(touch_irq);
#else
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif 
	}else{
		GTP_INFO("Touch Eint already enabled!");
	}
	//GTP_INFO("Enable irq_flag=%d",irq_flag);

}
void gtp_irq_disable(void)
{
	if(irq_flag==1){
		irq_flag--;
#ifdef CONFIG_OF_TOUCH
		disable_irq(touch_irq);
#else
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
	}else{
		GTP_INFO("Touch Eint already disabled!");
	}
	//GTP_INFO("Disable irq_flag=%d",irq_flag);
}

#ifdef CONFIG_OF_TOUCH 
static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = {0,0};
	GTP_INFO("Device Tree Tpd_irq_registration!");
	
	node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if(node){
		of_property_read_u32_array(node , "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		touch_irq = irq_of_parse_and_map(node, 0);
		GTP_INFO("Device gtp_int_type = %d!", int_type);
		if (!int_type)	//EINTF_TRIGGER
		{
			ret = request_irq(touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, EINTF_TRIGGER_RISING, "TOUCH_PANEL-eint", NULL);
            //gtp_eint_trigger_type = EINTF_TRIGGER_RISING;
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		}
		else
		{
			ret = request_irq(touch_irq, (irq_handler_t)tpd_eint_interrupt_handler, EINTF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
            //gtp_eint_trigger_type = EINTF_TRIGGER_FALLING;
			if(ret > 0){
			    ret = -1;
			    GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		}
	}else{
		GTP_ERROR("tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}
	//chenggang.li add for oppo
	//gtp_irq_enable();
	GTP_INFO("..............chenggang....add...for...oppo\n");
	GTP_INFO("[%s]irq:%d, debounce:%d-%d:", __FUNCTION__, touch_irq, ints[0], ints[1]);
	return ret;
}
#endif

//**************** For GT9XXF Start ********************//
#if GTP_COMPATIBLE_MODE


void gtp_get_chip_type(struct i2c_client *client)
{
    u8 opr_buf[10] = {0x00};
    s32 ret = 0;
    
    msleep(10);
    
    ret = gtp_i2c_read_dbl_check(client, GTP_REG_CHIP_TYPE, opr_buf, 10);
    
    if (FAIL == ret)
    {
        GTP_ERROR("Failed to get chip-type, set chip type default: GOODIX_GT9");
        gtp_chip_type = CHIP_TYPE_GT9;
        return;
    }
    
    if (!memcmp(opr_buf, "GOODIX_GT9", 10))
    {
        gtp_chip_type = CHIP_TYPE_GT9;
    }
    else // GT9XXF
    {
        gtp_chip_type = CHIP_TYPE_GT9F;
    }
    GTP_INFO("Chip Type: %s", (gtp_chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}

static u8 gtp_bak_ref_proc(struct i2c_client *client, u8 mode)
{
    s32 i = 0;
    s32 j = 0;
    s32 ret = 0;
    struct file *flp = NULL;
    u8 *refp = NULL;
    u32 ref_len = 0;
    u32 ref_seg_len = 0;
    s32 ref_grps = 0;
    s32 ref_chksum = 0;
    u16 tmp = 0;
    
    GTP_DEBUG("[gtp_bak_ref_proc]Driver:%d,Sensor:%d.", driver_num, sensor_num);

    //check file-system mounted 
    GTP_DEBUG("[gtp_bak_ref_proc]Waiting for FS %d", gtp_ref_retries);        
    if (gup_check_fs_mounted("/data") == FAIL)        
    {
        GTP_DEBUG("[gtp_bak_ref_proc]/data not mounted");
        if(gtp_ref_retries++ < GTP_CHK_FS_MNT_MAX)
        {
            return FAIL;
        }
    }
    else
    {
        GTP_DEBUG("[gtp_bak_ref_proc]/data mounted !!!!");
    }
    
    if (is_950)
    {
        ref_seg_len = (driver_num * (sensor_num - 1) + 2) * 2;
        ref_grps = 6;
        ref_len =  ref_seg_len * 6;  // for GT950, backup-reference for six segments
    }
    else
    {
        ref_len = driver_num*(sensor_num-2)*2 + 4;
        ref_seg_len = ref_len;
        ref_grps = 1;
    }
    
    refp = (u8 *)kzalloc(ref_len, GFP_KERNEL);
    if(refp == NULL)
    {
        GTP_ERROR("Failed to allocate memory for reference buffer!"); 
        return FAIL;
    }
    memset(refp, 0, ref_len);
    
    //get ref file data
    flp = filp_open(GTP_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(flp))
    {
        GTP_ERROR("Failed to open/create %s.", GTP_BAK_REF_PATH);
        if (GTP_BAK_REF_SEND == mode)
        {
            goto default_bak_ref;
        }
        else
        {
            goto exit_ref_proc;
        }
    }
    
    switch (mode)
    {
    case GTP_BAK_REF_SEND:
        {
            flp->f_op->llseek(flp, 0, SEEK_SET);
            ret = flp->f_op->read(flp, (char *)refp, ref_len, &flp->f_pos);
            if(ret < 0)
            {
                GTP_ERROR("Read ref file failed, send default bak ref.");
                goto default_bak_ref;
            }
            //checksum ref file
            for (j = 0; j < ref_grps; ++j)
            {
                ref_chksum = 0;
                for(i=0; i<ref_seg_len-2; i+=2)
                {
                    ref_chksum += ((refp[i + j * ref_seg_len]<<8) + refp[i + 1 + j * ref_seg_len]);
                }
            
                GTP_DEBUG("Reference chksum:0x%04X", ref_chksum&0xFF);
                tmp = ref_chksum + (refp[ref_seg_len + j * ref_seg_len -2]<<8) + refp[ref_seg_len + j * ref_seg_len -1];
                if(1 != tmp)
                {
                    GTP_DEBUG("Invalid checksum for reference, reset reference.");
                    memset(&refp[j * ref_seg_len], 0, ref_seg_len);
                    refp[ref_seg_len - 1 + j * ref_seg_len] = 0x01;
                }
                else
                {
                    if (j == (ref_grps - 1))
                    {
                        GTP_INFO("Reference data in %s used.", GTP_BAK_REF_PATH);
                    }
                }
              
            }
            ret = i2c_write_bytes(client, GTP_REG_BAK_REF, refp, ref_len);
            if(-1 == ret)
            {
                GTP_ERROR("Write ref i2c error.");
                ret = FAIL;
                goto exit_ref_proc;
            }
        }
        break;
        
    case GTP_BAK_REF_STORE:
        {
            ret = i2c_read_bytes(client, GTP_REG_BAK_REF, refp, ref_len);
            if(-1 == ret)
            {
                GTP_ERROR("Read ref i2c error.");
                ret = FAIL;
                goto exit_ref_proc;
            }
            flp->f_op->llseek(flp, 0, SEEK_SET);
            flp->f_op->write(flp, (char *)refp, ref_len, &flp->f_pos);
        }
        break;
        
    default:
        GTP_ERROR("Invalid Argument(%d) for backup reference", mode);
        ret = FAIL;
        goto exit_ref_proc;
    }
    
    ret = SUCCESS;
    goto exit_ref_proc;

default_bak_ref:
    for (j = 0; j < ref_grps; ++j)
    {
        memset(&refp[j * ref_seg_len], 0, ref_seg_len);
        refp[j * ref_seg_len + ref_seg_len - 1] = 0x01;  // checksum = 1
    }
    ret = i2c_write_bytes(client, GTP_REG_BAK_REF, refp, ref_len);
    if (flp && !IS_ERR(flp))
    {
        GTP_INFO("Write backup-reference data into %s", GTP_BAK_REF_PATH);
        flp->f_op->llseek(flp, 0, SEEK_SET);
        flp->f_op->write(flp, (char*)refp, ref_len, &flp->f_pos);
    }
    if (ret < 0)
    {
        GTP_ERROR("Failed to load the default backup reference");
        ret = FAIL;
    }
    else
    {
        ret = SUCCESS;
    }
exit_ref_proc:
    if (refp)
    {
        kfree(refp);
    }
    if (flp && !IS_ERR(flp))
    {
        filp_close(flp, NULL);
    }
    return ret;
}

u8 gtp_fw_startup(struct i2c_client *client)
{
    u8 wr_buf[4];
    s32 ret = 0;
    
    //init sw WDT
    wr_buf[0] = 0xAA;
    ret = i2c_write_bytes(client, 0x8041, wr_buf, 1);
    if (ret < 0)
    {
        GTP_ERROR("I2C error to firmware startup.");
        return FAIL;
    }
    //release SS51 & DSP
    wr_buf[0] = 0x00;
    i2c_write_bytes(client, 0x4180, wr_buf, 1);
    
    //int sync
    gtp_int_sync(25);
    
    //check fw run status
    i2c_read_bytes(client, 0x8041, wr_buf, 1);
    if(0xAA == wr_buf[0])
    {
        GTP_ERROR("IC works abnormally,startup failed.");
        return FAIL;
    }
    else
    {
        GTP_DEBUG("IC works normally,Startup success.");
        wr_buf[0] = 0xAA;
        i2c_write_bytes(client, 0x8041, wr_buf, 1);
        return SUCCESS;
    }
}






static void gtp_recovery_reset(struct i2c_client *client)
{
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_OFF);
#endif
    force_reset_guitar();
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif
}

static u8 gtp_check_clk_legality(u8 *p_clk_buf)
{
    u8 i = 0;
    u8 clk_chksum = p_clk_buf[5];
    
    for(i = 0; i < 5; i++)
    {
        if((p_clk_buf[i] < 50) || (p_clk_buf[i] > 120) ||
            (p_clk_buf[i] != p_clk_buf[0]))
        {
            break;
        }
        clk_chksum += p_clk_buf[i];
    }
    
    if((i == 5) && (clk_chksum == 0))
    {
        GTP_DEBUG("Valid main clock data.");
        return SUCCESS;
    }
    GTP_ERROR("Invalid main clock data.");
    return FAIL;
}

static u8 gtp_main_clk_proc(struct i2c_client *client)
{
    s32 ret = 0;
    u8  i = 0;
    u8  clk_cal_result = 0;
    u8  clk_chksum = 0;
    u8  gtp_clk_buf[6] = {0};
    struct file *flp = NULL;
    
    GTP_DEBUG("[gtp_main_clk_proc]Waiting for FS %d", gtp_ref_retries);        
    if (gup_check_fs_mounted("/data") == FAIL)        
    {            
        GTP_DEBUG("[gtp_main_clk_proc]/data not mounted");
        if(gtp_clk_retries++ < GTP_CHK_FS_MNT_MAX)
        {
            return FAIL;
        }
        else
        {
            GTP_ERROR("[gtp_main_clk_proc]Wait for file system timeout,need cal clk");
        }
    }
    else
    {
        GTP_DEBUG("[gtp_main_clk_proc]/data mounted !!!!");
        flp = filp_open(GTP_MAIN_CLK_PATH, O_RDWR | O_CREAT, 0666);
        if (!IS_ERR(flp))
        {
            flp->f_op->llseek(flp, 0, SEEK_SET);
            ret = flp->f_op->read(flp, (char *)gtp_clk_buf, 6, &flp->f_pos);
            if(ret > 0)
            {
                ret = gtp_check_clk_legality(gtp_clk_buf);
                if(SUCCESS == ret)
                {
                        GTP_DEBUG("[gtp_main_clk_proc]Open & read & check clk file success.");
                    goto send_main_clk;
                }
            }
        }
        GTP_ERROR("[gtp_main_clk_proc]Check clk file failed,need cal clk");
    }
    
    //cal clk
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_OFF);
#endif
    clk_cal_result = gup_clk_calibration();
    force_reset_guitar();
    GTP_DEBUG("&&&&&&&&&&clk cal result:%d", clk_cal_result);
    
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif  

    if(clk_cal_result < 50 || clk_cal_result > 120)
    {
        GTP_ERROR("Invalid main clock: %d", clk_cal_result);
        ret = FAIL;
        goto exit_clk_proc;
    }
    
    for(i = 0;i < 5; i++)
    {
        gtp_clk_buf[i] = clk_cal_result;
        clk_chksum += gtp_clk_buf[i];
    }
    gtp_clk_buf[5] = 0 - clk_chksum;
    
send_main_clk:
    
    ret = i2c_write_bytes(client, 0x8020, gtp_clk_buf, 6);
    
    if (flp && !IS_ERR(flp))
    {
        flp->f_op->llseek(flp, 0, SEEK_SET);
        flp->f_op->write(flp, (char *)gtp_clk_buf, 6, &flp->f_pos);
    }
    
    if(-1 == ret)
    {
        GTP_ERROR("[gtp_main_clk_proc]send main clk i2c error!");
        ret = FAIL;
    }
    else
    {
        ret = SUCCESS;
    }
    
exit_clk_proc:
    if (flp && !IS_ERR(flp))
    {
        filp_close(flp, NULL);
    }
    return ret;
}

#endif
//************* For GT9XXF End **********************//

#if GTP_WITH_PEN
static void gtp_pen_init(void)
{
    s32 ret = 0;
    
    pen_dev = input_allocate_device();
    if (pen_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device for pen/stylus.");
        return;
    }
    
    pen_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    pen_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    
    set_bit(BTN_TOOL_PEN, pen_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
    //set_bit(INPUT_PROP_POINTER, pen_dev->propbit);
    
#if GTP_PEN_HAVE_BUTTON
    input_set_capability(pen_dev, EV_KEY, BTN_STYLUS);
    input_set_capability(pen_dev, EV_KEY, BTN_STYLUS2);
#endif

    input_set_abs_params(pen_dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
    input_set_abs_params(pen_dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
    input_set_abs_params(pen_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(pen_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(pen_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
    
    pen_dev->name = "mtk-pen";
    pen_dev->phys = "input/ts";
    pen_dev->id.bustype = BUS_I2C;
    
    ret = input_register_device(pen_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", pen_dev->name);
        return;
    }
}

static void gtp_pen_down(s32 x, s32 y, s32 size, s32 id)
{
    input_report_key(pen_dev, BTN_TOOL_PEN, 1);
    input_report_key(pen_dev, BTN_TOUCH, 1);
    input_report_abs(pen_dev, ABS_MT_POSITION_X, x);
    input_report_abs(pen_dev, ABS_MT_POSITION_Y, y);
    if ((!size) && (!id))
    {
        input_report_abs(pen_dev, ABS_MT_PRESSURE, 100);
        input_report_abs(pen_dev, ABS_MT_TOUCH_MAJOR, 100);
    }
    else
    {
        input_report_abs(pen_dev, ABS_MT_PRESSURE, size);
        input_report_abs(pen_dev, ABS_MT_TOUCH_MAJOR, size);
        input_report_abs(pen_dev, ABS_MT_TRACKING_ID, id);
    }
    input_mt_sync(pen_dev);
}

static void gtp_pen_up(void)
{
    input_report_key(pen_dev, BTN_TOOL_PEN, 0);
    input_report_key(pen_dev, BTN_TOUCH, 0);
}
#endif



#ifdef GT9XX_SUPPORT_APK_AUTOTEST
static ssize_t tp_glove_check_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{

    int ret = 0;
    char page[PAGESIZE];
    printk("glove check is: %d\n", glove_check);
    ret = sprintf(page, "%d\n", glove_check);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

#if 0 /* delete it by lauson. */
static int tp_glove_check_write_func(struct file *file, const char *buffer, unsigned long count, void *data)
{
    /*
     int len = 0;
    glove_check =1;
    printk("glove check is: %d\n", glove_check);
    len = sprintf(page, "%d\n", atomic_read(&glove_check));

    return len;
    */
}
#endif

#endif


static ssize_t tp_double_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{

    int ret = 0;
    char page[PAGESIZE];
    printk("double tap enable is: %d\n", atomic_read(&double_enable));
    ret = sprintf(page, "%d\n", atomic_read(&double_enable));
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}


static ssize_t tp_double_write_func(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    int ret = 0;

    char buf[10] = {0};
    struct goodix_ts_data *ts = i2c_get_clientdata(i2c_client_point);
	if(ts==NULL)
		printk(KERN_INFO "%s: get ts error.\n", __func__);
    
    if (count > 10)
        return count;

    if (copy_from_user( buf, buffer, count))
    {
        printk(KERN_INFO "%s: read proc input error.\n", __func__);
        return count;
    }

    sscanf(buf, "%d", &ret);

    mutex_lock(&ts->io_lock);
    
    GTP_DEBUG("tp_double_write_func:buf = %d,ret = %d\n", *buf, ret);
    if (((ret == 0 ) || (ret == 1)) && atomic_read(&double_enable) != ret)
    {
        atomic_set(&double_enable, ret);
    }
    else
    {
        mutex_unlock(&ts->io_lock);
        return count;
    }
    
    if (ts->gtp_is_suspend)
    {
        switch (ret)
        {
            case 0:
                GTP_DEBUG("tp_guesture_func will be disable\n");
                doze_status = DOZE_DISABLED;
                disable_irq_wake(touch_irq);
                gtp_irq_disable();
                gtp_reset_guitar(i2c_client_point, 20);
                gtp_enter_sleep(i2c_client_point);
                msleep(58);
                break;
            case 1:
                GTP_DEBUG("tp_guesture_func will be enable\n");
                doze_status = DOZE_DISABLED;
                gtp_irq_disable();
                gtp_reset_guitar(i2c_client_point, 20);
                gtp_enter_doze(i2c_client_point);
                gtp_irq_enable();
                enable_irq_wake(touch_irq);
                msleep(58);
                break;
            default:
                GTP_DEBUG("Please enter 0 or 1 to open or close the double-tap function\n");
        }
    }

	mutex_unlock(&ts->io_lock);
    
    return count;
}



#ifdef SUPPORT_GESTURE
static const struct file_operations tp_double_proc_fops =
{
    .write = tp_double_write_func,
    .read =  tp_double_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif

#ifdef GT9XX_SUPPORT_APK_AUTOTEST
static const struct file_operations glove_check_proc_fops =
{
    //.write = tp_glove_write_func,
    .read =  tp_glove_check_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif




static unsigned int boot_mode = 1;
static s32 tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 err = 0;
    s32 ret = 0;
	int ID1, ID2;

    u16 version_info;   
#if GTP_HAVE_TOUCH_KEY
    s32 idx = 0;
#endif
#ifdef TPD_PROXIMITY
    struct hwmsen_object obj_ps;
#endif
	struct goodix_ts_data *ts;
	static int tp_probe_ok = 0;
   
	
	boot_mode = get_boot_mode();
	GTP_ERROR("..boot_mode=%d...",boot_mode);
	
   /*  if (RECOVERY_BOOT == get_boot_mode()){
		GTP_ERROR("..chenggang.li..probe...return!");
		return 0; 
	} */
            
	GTP_ERROR("..chenggang...probe.....!");
    i2c_client_point = client;
    ret = tpd_power_on(client);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
		return 0;
    }

	 ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
    if (!ts)
    {
        dev_err(&client->dev, "GTP not enough memory for ts\n");
        return -ENOMEM;
    }
	ts->client = client;
	i2c_set_clientdata(client, ts); 
	mutex_init(&ts->rst_lock);
	
//#ifdef VELOCITY_CUSTOM
#if 0
    if ((err = misc_register(&tpd_misc_device)))
    {
        printk("mtk_tpd: tpd_misc_device register failed\n");
    }

#endif
    ret = gtp_read_version(client, &version_info);

    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");
    }    

	//chenggang.li@bsp add for oppo 
	strcpy(gt9xx_updata_file_path, GT9XX_UPDATE_FILE_PATH);
	strcat(gt9xx_updata_file_path, "tp/16011/16011_GOODIX_UPDATE_TPK.bin");
	
	
    ret = gtp_init_panel(client);

    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
    }
    
    // Create proc file system
    gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
    if (gt91xx_config_proc == NULL)
    {
        GTP_ERROR("create_proc_entry %s failed\n", GT91XX_CONFIG_PROC_FILE);
    }
    else
    {
        GTP_INFO("create proc entry %s success", GT91XX_CONFIG_PROC_FILE);
    }

	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if(prEntry_tp == NULL)
	{
		ret = -ENOMEM;
	  	GTP_ERROR(KERN_INFO"init_gt1x_proc: Couldn't create TP proc entry\n");
		return ret;
	} 
#ifdef SUPPORT_GESTURE
	prEntry_dtap = proc_create("double_tap_enable", 0666, prEntry_tp, &tp_double_proc_fops);
	if(prEntry_dtap == NULL)
	{
		ret = -ENOMEM;
	  	GTP_ERROR(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		return ret;
	}
	GTP_INFO("create gt1x gesture proc success\n");

#endif	

#ifdef GT9XX_SUPPORT_APK_AUTOTEST
    prEntry_glove_check = proc_create( "glove_mode_check", 0666, prEntry_tp, &glove_check_proc_fops);
    if (prEntry_glove_check == NULL)
    {
        ret = -ENOMEM;
        printk(KERN_INFO"init_gt9xx_proc: Couldn't create proc entry\n");
    }
#endif


#if GTP_CREATE_WR_NODE
    init_wr_node(client);
#endif

	mutex_init(&ts->io_lock);

    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);

    if (IS_ERR(thread))
    {
        err = PTR_ERR(thread);
        GTP_INFO(TPD_DEVICE " failed to create kernel thread: %d\n", err);
    }
    

#if TOUCH_VIRTUAL_KEYS
	__set_bit(KEY_MENU,  tpd->dev->keybit);
	__set_bit(KEY_BACK,  tpd->dev->keybit);
	__set_bit(KEY_HOME,  tpd->dev->keybit);
	//__set_bit(KEY_SEARCH,  tpd->dev->keybit);
#endif
   
   
#if GTP_HAVE_TOUCH_KEY

    for (idx = 0; idx < GTP_MAX_KEY_NUM; idx++)
    {
        input_set_capability(tpd->dev, EV_KEY, touch_key_array[idx]);
    }

#endif
#if GTP_GESTURE_WAKEUP
    input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
#endif
    
#if TOUCH_VIRTUAL_KEYS
	gtp_ts_virtual_keys_init();
#endif

#if GTP_WITH_PEN
    gtp_pen_init();
#endif
    
	
	ID1=mt_get_gpio_in(GPIO_TP_ID1_16011);//GPIO21
	ID2=mt_get_gpio_in(GPIO_TP_ID2_16011);//GPIO19
	
	if(ID1==1&&ID2==0){		
		GTP_ERROR("........TP is........OFILM\n");		
		TP_FW = 16011001;
		oppo_tp_dev = TP_OFILM_WHITE;
		glove_check = 1;
		//strcpy(ts->fw_name,"tp/15127_FW_OFILM.img");	
		//strcpy(ts->test_limit_name,"tp/15127_Limit_OFILM.img");
	}else if(ID1==0&&ID2==0){		
		GTP_ERROR("........TP is........Truly\n");
		TP_FW = 16011001;		
		oppo_tp_dev = TP_TRULY_GOODIX;	
		glove_check = 0;		
		//strcpy(ts->fw_name,"tp/15127_FW_Truly.img");
		//strcpy(ts->test_limit_name,"tp/15127_Limit_Truly.img");
	}

    msleep(50);

#ifdef CONFIG_OF_TOUCH
	/* EINT device tree, default EINT enable */
	tpd_irq_registration();
#else

#ifdef MT6572
    if (!int_type)  //EINTF_TRIGGER
    {
        mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);
    }
    else
    {
        mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
    }
    
#else
    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);

    if (!int_type)
    {
        mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1);
    }
    else
    {
        mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, tpd_eint_interrupt_handler, 1);
    }
#endif
#endif

	gtp_irq_enable();
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
    ret = gup_init_update_proc(client);

    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.");
    }
#endif

#ifdef TPD_PROXIMITY
    //obj_ps.self = cm3623_obj;
    obj_ps.polling = 0;         //0--interrupt mode;1--polling mode;
    obj_ps.sensor_operate = tpd_ps_operate;

    if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        GTP_ERROR("hwmsen attach fail, return:%d.", err);
    }

#endif


   
    tpd_load_status = 1;

    return 0;
}

#ifdef CONFIG_OF_TOUCH
static irqreturn_t tpd_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
	TPD_DEBUG_PRINT_INT;
		
	tpd_flag = 1;
	/* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
	/* use _nosync to avoid deadlock */
	//disable_irq_nosync(touch_irq);
	//irq_flag--;
	//GTP_INFO("disable irq_flag=%d",irq_flag);
	wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
static void tpd_eint_interrupt_handler(void)
{
	TPD_DEBUG_PRINT_INT;

	tpd_flag = 1;

	wake_up_interruptible(&waiter);
}
#endif
/*
static void tpd_eint_interrupt_handler(void)
{
    TPD_DEBUG_PRINT_INT;
    
    tpd_flag = 1;
    
    wake_up_interruptible(&waiter);
}*/


static int tpd_i2c_remove(struct i2c_client *client)
{
#if GTP_CREATE_WR_NODE
    uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif

    return 0;
}
#if (GTP_ESD_PROTECT || GTP_COMPATIBLE_MODE)
static void force_reset_guitar(void)
{
    s32 i = 0;
    s32 ret = 0;

    GTP_INFO("force_reset_guitar");
    
    //mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	gtp_irq_disable();
		
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);   
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
#ifdef MT6573
    //Power off TP
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);  
    msleep(30);
    //Power on TP
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
    msleep(30);
#else           // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )
    // Power off TP
    #ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
    #else
        hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
    #endif
        msleep(30); 

    // Power on TP
    #ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
    #else
        hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    #endif
        msleep(30);

#endif

    for (i = 0; i < 5; i++)
    {
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == gtp_chip_type)
        {
            ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY);
            if(FAIL == ret)
            {
                GTP_ERROR("[force_reset_guitar]Check & repair fw failed.");
                continue;
            }
            //startup fw
            ret = gtp_fw_startup(i2c_client_point);
            if(FAIL == ret)
            {
                GTP_ERROR("[force_reset_guitar]Startup fw failed.");
                continue;
            }
            break;
        }
        else
    #endif
        {
            //Reset Guitar
            gtp_reset_guitar(i2c_client_point, 20);
            msleep(50);
            //Send config
            ret = gtp_send_cfg(i2c_client_point);
    
            if (ret < 0)
            {
                continue;
            }
        }
        break;
    }
    //mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    gtp_irq_enable();
    
    if (i >= 5)
    {
        GTP_ERROR("Failed to reset guitar.");
        return;
    }
    GTP_INFO("Esd recovery successful");
    return;
}
#endif





#ifdef GT9XX_SUPPORT_REPORT_COORDINATE
static const struct file_operations coordinate_proc_fops =
{
    .read =  coordinate_proc_read_func,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif



#if GTP_ESD_PROTECT
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[2] = {0xAA};
    GTP_DEBUG("Init external watchdog.");
    return i2c_write_bytes(client, 0x8041, opr_buffer, 1);
}

void gtp_esd_switch(struct i2c_client *client, s32 on)
{
    spin_lock(&esd_lock);     
    if (SWITCH_ON == on)     // switch on esd 
    {
        if (!esd_running)
        {
            esd_running = 1;
            spin_unlock(&esd_lock);
            GTP_INFO("Esd started");
            queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, clk_tick_cnt);
        }
        else
        {
            spin_unlock(&esd_lock);
        }
    }
    else    // switch off esd
    {
        if (esd_running)
        {
            esd_running = 0;
            spin_unlock(&esd_lock);
            GTP_INFO("Esd cancelled");
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
        else
        {
            spin_unlock(&esd_lock);
        }
    }
}


static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i = 0;
    s32 ret = -1;
    u8 esd_buf[3] = {0x00};
    if ((tpd_halt) || (gtp_loading_fw))
    {
        GTP_INFO("Esd suspended or IC update firmware!");
        return;
    }
    for (i = 0; i < 3; i++)
    {
        ret = i2c_read_bytes_non_dma(i2c_client_point, 0x8040, esd_buf, 2);
        
        GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[0], esd_buf[1]);
        if (ret < 0)
        {
            // IIC communication problem
            continue;
        }
        else 
        {
            if ((esd_buf[0] == 0xAA) || (esd_buf[1] != 0xAA))
            {
                u8 chk_buf[2] = {0x00};
                i2c_read_bytes_non_dma(i2c_client_point, 0x8040, chk_buf, 2);
                
                GTP_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[0], chk_buf[1]);
                
                if ( (chk_buf[0] == 0xAA) || (chk_buf[1] != 0xAA) )
                {
                    i = 3;          // jump to reset guitar
                    break;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                // IC works normally, Write 0x8040 0xAA, feed the watchdog
                esd_buf[0] = 0xAA;
                i2c_write_bytes_non_dma(i2c_client_point, 0x8040, esd_buf, 1);
                
                break;
            }
        }
    }

    if (i >= 3)
    {   
    #if GTP_COMPATIBLE_MODE
        if ((CHIP_TYPE_GT9F == gtp_chip_type) && (1 == rqst_processing))
        {
            GTP_INFO("Request Processing, no reset guitar.");
        }
        else
    #endif
        {
            GTP_INFO("IC works abnormally! Process reset guitar.");
            esd_buf[0] = 0x01;
            esd_buf[1] = 0x01;
            esd_buf[2] = 0x01;
            i2c_write_bytes(i2c_client_point, 0x4226, esd_buf, 3);  
            msleep(50);
            force_reset_guitar();
        }
    }

    if (!tpd_halt)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, clk_tick_cnt);
    }
    else
    {
        GTP_INFO("Esd suspended!");
    }

    return;
}
#endif

static void tpd_down(s32 x, s32 y, s32 size, s32 id)
{
    if ((!size) && (!id))
    {
        input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
    }
    else
    {
        input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
        /* track id Start 0 */
        input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
    }

    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, id, 1);

#if (defined(MT6575)||defined(MT6577))

    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(x, y, 1);
    }

#endif
}

static void tpd_up(s32 x, s32 y, s32 id)
{
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, id, 0);

#if (defined(MT6575) || defined(MT6577))

    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(x, y, 0);
    }

#endif
}
#if GTP_CHARGER_SWITCH
static void gtp_charger_switch(s32 dir_update)
{
    u32 chr_status = 0;
    u8 chr_cmd[3] = {0x80, 0x40};
    static u8 chr_pluggedin = 0;
    int ret = 0;
    
#ifdef MT6573
    chr_status = *(volatile u32 *)CHR_CON0;
    chr_status &= (1 << 13);
#else   // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )
    chr_status = upmu_is_chr_det();
#endif
    
    if (chr_status)     // charger plugged in
    {
        if (!chr_pluggedin || dir_update)
        {
            chr_cmd[2] = 6;
            ret = gtp_i2c_write(i2c_client_point, chr_cmd, 3);
            if (ret > 0)
            {
                GTP_INFO("Update status for Charger Plugin");
				if (gtp_send_chr_cfg(i2c_client_point) < 0) {
					GTP_ERROR("Send charger config failed.");
				} else {
					GTP_DEBUG("Send charger config.");
				}
            }
            chr_pluggedin = 1;
        }
    }
    else            // charger plugged out
    {
        if (chr_pluggedin || dir_update)
        {
            chr_cmd[2] = 7;
            ret = gtp_i2c_write(i2c_client_point, chr_cmd, 3);
            if (ret > 0)
            {
                GTP_INFO("Update status for Charger Plugout");
				if (gtp_send_cfg(i2c_client_point) < 0) {
					GTP_ERROR("Send normal config failed.");
				} else {
					GTP_DEBUG("Send normal config.");
				}
            }
            chr_pluggedin = 0;
        }
    }
}
#endif

static int touch_event_handler(void *unused)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u8 pre_touch = 0;
    static u8 pre_key = 0;
#if GTP_WITH_PEN
    u8 pen_active = 0;
    static u8 pre_pen = 0;
#endif
    u8  key_value = 0;
    u8 *coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    
#if GTP_COMPATIBLE_MODE
    u8  rqst_data[3] = {(u8)(GTP_REG_RQST >> 8), (u8)(GTP_REG_RQST & 0xFF), 0};
#endif

#ifdef TPD_PROXIMITY
    s32 err = 0;
    hwm_sensor_data sensor_data;
    u8 proximity_status;
#endif

#if GTP_GESTURE_WAKEUP
    u8 doze_buf[3] = {0x81, 0x4B};
#ifdef SUPPORT_GESTURE    
u8 coordinate_single[258] = {0x94, 0x20};    
u8 coordinate_size[3] = {0x81, 0x4c, 0x00};   
u8 length = 0;
#endif
#endif

    sched_setscheduler(current, SCHED_RR, &param);
    do{
        set_current_state(TASK_INTERRUPTIBLE);
        while (tpd_halt)
        {
        #if GTP_GESTURE_WAKEUP
            if (DOZE_ENABLED == doze_status)
            {
                break;
            }
        #endif
            tpd_flag = 0;
            msleep(20);
        }

        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        TPD_DEBUG_SET_TIME;
        set_current_state(TASK_RUNNING);

    #if GTP_CHARGER_SWITCH
        gtp_charger_switch(0);
    #endif

    #if GTP_GESTURE_WAKEUP
        if (DOZE_ENABLED == doze_status)
        {
            ret = gtp_i2c_read(i2c_client_point, doze_buf, 3);
            GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
#ifdef SUPPORT_GESTURE
        if (ret > 0)
        {
            if (doze_buf[2] != 0)
            {

                GTP_DEBUG(" gesture = %d\n", gesture);

                //rendong.shi add gesture
                for (i = 2; i < 66; i++)
                {
                    coordinate_single[i] = 0;
                }
                ret = gtp_i2c_read(i2c_client_point, coordinate_size, 3);
                GTP_INFO("The length of Gesture Coordinate is %d\n", coordinate_size[2]);
                if (doze_buf[2]  != DTAP_DETECT)
                    ret = gtp_i2c_read(i2c_client_point, coordinate_single, coordinate_size[2] * 4 + 2);
                length = coordinate_size[2];
                switch (doze_buf[2])
                {
                    case DTAP_DETECT:
                        gesture = DouTap;
                        break;

                    case UP_VEE_DETECT :
                        gesture = UpVee;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[10]  |  (coordinate_single[11] << 8);
                        Point_end.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case DOWN_VEE_DETECT :
                        gesture = DownVee;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[10]  |  (coordinate_single[11] << 8);
                        Point_end.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case LEFT_VEE_DETECT:
                        gesture =  LeftVee;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[10]  |  (coordinate_single[11] << 8);
                        Point_end.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case RIGHT_VEE_DETECT :
                        gesture =  RightVee;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[18]  |  (coordinate_single[19] << 8);
                        Point_end.y = coordinate_single[20] |  (coordinate_single[21] << 8);
                        Point_1st.x = coordinate_single[10] |  (coordinate_single[11] << 8);
                        Point_1st.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        break;

                    case CIRCLE_DETECT  :
                        gesture =  Circle;
#if 0
                        j = 0;
                        for (i = 0; i < length; i++)
                        {
                            Point_input[i].x = coordinate_single[j + 2]  |  (coordinate_single[j + 3] << 8);
                            Point_input[i].y = coordinate_single[j + 3] |  (coordinate_single[j + 4] << 8);
                            j = j + 4;
                            GTP_INFO("Point_input[%d].x = %d,Point_input[%d].y = %d\n", i, Point_input[i].x, i, Point_input[i].y)   ;
                        }
                        clock = cross(Point_input[0], Point_input[1], Point_input[2]);
                        GetCirclePoints(&Point_input[0], length, Point_output);
                        Point_start.x = Point_input[0].x;
                        Point_start.y = Point_input[0].y;
                        Point_end.x = Point_input[length - 1].x;
                        Point_end.y = Point_input[length - 1].y;
                        Point_1st.x = Point_output[0].x;
                        Point_1st.y = Point_output[0].y;
                        Point_2nd.x = Point_output[1].x;
                        Point_2nd.y = Point_output[1].y;
                        Point_3rd.x = Point_output[2].x;
                        Point_3rd.y = Point_output[2].y;
                        Point_4th.x = Point_output[3].x;
                        Point_4th.y = Point_output[3].y;
                        if (clock > 0)
                        {
                            clockwise = 1;
                        }
                        else
                        {
                            clockwise = 0;
                        }
#endif

#if 1
                        {
                            int j = 2;
                            //for (i = 2; i < (length * 4 + 2); i++)
                            //{
                                //coordinate_single[i] = 0;
                            //}
                            //ret = gtp_i2c_read(i2c_connect_client, coordinate_size, 3);
                            //length = coordinate_size[2];
                            //GTP_ERROR("coordinate_size[2] = %d\n ", coordinate_size[2]);
                            //ret = gtp_i2c_read(i2c_connect_client, coordinate_single, coordinate_size[2] * 4 + 2);
                            for (i = 2; i < (length + 2); i++)
                            {
                                Point_input[i].x = coordinate_single[j]  |  (coordinate_single[j + 1] << 8);
                                Point_input[i].y = coordinate_single[j + 2] |  (coordinate_single[j + 3] << 8);
                                j = j + 4;
                                printk("Point_input[%d].x = %d,Point_input[%d].y = %d\n", i, Point_input[i].x, i, Point_input[i].y) ;
                            }
                        }
                        GetCirclePoints(&Point_input[2], length , Point_output);
                        if (cross(Point_input[3], Point_input[4], Point_input[5]) > 0)
                            clockwise = 1;
                        else
                            clockwise = 0;
                        Point_start.x = Point_input[2].x;
                        Point_start.y = Point_input[2].y;
                        Point_end.x = Point_input[length + 1].x;
                        Point_end.y = Point_input[length + 1].y;
                        Point_1st.x = Point_output[0].x;
                        Point_1st.y = Point_output[0].y;
                        Point_2nd.x = Point_output[1].x;
                        Point_2nd.y = Point_output[1].y;
                        Point_3rd.x = Point_output[2].x;
                        Point_3rd.y = Point_output[2].y;
                        Point_4th.x = Point_output[3].x;
                        Point_4th.y = Point_output[3].y;
                        for (i = 0; i < 5; i++)
                        {
                            printk("Point_output[%d].x = %d,Point_output[%d].y = %d\n", i, Point_output[i].x, i, Point_output[i].y);
                        }
#else
                        Point_start.x = 0;
                        Point_start.y = 0;
                        Point_end.x = 0;
                        Point_end.y = 0;
                        Point_1st.x = 0;
                        Point_1st.y = 0;
                        Point_2nd.x = 0;
                        Point_2nd.y = 0;
                        Point_3rd.x = 0;
                        Point_3rd.y = 0;
                        Point_4th.x = 0;
                        Point_4th.y = 0;
#endif
                        break;

                    case DOUSWIP_DETECT  :
                        gesture =  DouSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        Point_1st.x = coordinate_single[10] |  (coordinate_single[11] << 8);
                        Point_1st.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_2nd.x = coordinate_single[14]  |  (coordinate_single[15] << 8);
                        Point_2nd.y = coordinate_single[16] |  (coordinate_single[17] << 8);
                        break;

                    case RIGHT_SLIDE_DETECT :
                        gesture =  Left2RightSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6]  |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case LEFT_SLIDE_DETECT :
                        gesture =  Right2LeftSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6]  |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case DOWN_SLIDE_DETECT  :
                        gesture =  Up2DownSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6]  |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case UP_SLIDE_DETECT :
                        gesture =  Down2UpSwip;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[6]  |  (coordinate_single[7] << 8);
                        Point_end.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        break;

                    case M_DETECT  :
                        gesture =  Mgestrue;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[18]  |  (coordinate_single[19] << 8);
                        Point_end.y = coordinate_single[20] |  (coordinate_single[21] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        Point_2nd.x = coordinate_single[10] |  (coordinate_single[11] << 8);
                        Point_2nd.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_3rd.x = coordinate_single[14] |  (coordinate_single[15] << 8);
                        Point_3rd.y = coordinate_single[16] |  (coordinate_single[17] << 8);
                        break;

                    case W_DETECT :
                        gesture =  Wgestrue;
                        Point_start.x = coordinate_single[2] |  (coordinate_single[3] << 8);
                        Point_start.y = coordinate_single[4] |  (coordinate_single[5] << 8);
                        Point_end.x = coordinate_single[18]  |  (coordinate_single[19] << 8);
                        Point_end.y = coordinate_single[20] |  (coordinate_single[21] << 8);
                        Point_1st.x = coordinate_single[6] |  (coordinate_single[7] << 8);
                        Point_1st.y = coordinate_single[8] |  (coordinate_single[9] << 8);
                        Point_2nd.x = coordinate_single[10] |  (coordinate_single[11] << 8);
                        Point_2nd.y = coordinate_single[12] |  (coordinate_single[13] << 8);
                        Point_3rd.x = coordinate_single[14] |  (coordinate_single[15] << 8);
                        Point_3rd.y = coordinate_single[16] |  (coordinate_single[17] << 8);
                        break;

                    default:
                        gesture =  UnkownGestrue;
                        break;


                }

                GTP_INFO("detect %s gesture\n", gesture == DouTap ? "double tap" :
                         gesture == UpVee ? "up vee" :
                         gesture == DownVee ? "down vee" :
                         gesture == LeftVee ? "(>)" :
                         gesture == RightVee ? "(<)" :
                         gesture == Circle ? "circle" :
                         gesture == DouSwip ? "(||)" :
                         gesture == Left2RightSwip ? "(-->)" :
                         gesture == Right2LeftSwip ? "(<--)" :
                         gesture == Up2DownSwip ? "up to down |" :
                         gesture == Down2UpSwip ? "down to up |" :
                         gesture == Mgestrue ? "(M)" :
                         gesture == Wgestrue ? "(W)" : "oppo custom gesture");

                 pr_err("%s %d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", __func__,  gesture,
                   Point_start.x, Point_start.y, Point_end.x, Point_end.y,
                   Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
                   Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
                   clockwise);
    
                //report Key to notify
                if (gesture != UnkownGestrue)
                {


                    GTP_INFO("F4 report!!!!!\n");
                    input_report_key(tpd->dev, KEY_F4, 1);
                    input_sync(tpd->dev);
                    input_report_key(tpd->dev, KEY_F4, 0);
                    input_sync(tpd->dev);
                    //doze_status = DOZE_WAKEUP;
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_client_point, doze_buf, 3);
                }
            }
            else
            {
                //gtp_enter_doze(i2c_connect_client);
                // clear 0x814B
                doze_buf[2] = 0x00;
                gtp_i2c_write(i2c_client_point,doze_buf, 3);
                gtp_enter_doze(i2c_client_point);
            }
        }
	//////////////////////////////////////////////	
#else
        if (ret > 0)
        {
                if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') || (doze_buf[2] == 'c') ||
                    (doze_buf[2] == 'd') || (doze_buf[2] == 'e') || (doze_buf[2] == 'g') || 
                    (doze_buf[2] == 'h') || (doze_buf[2] == 'm') || (doze_buf[2] == 'o') ||
                    (doze_buf[2] == 'q') || (doze_buf[2] == 's') || (doze_buf[2] == 'v') || 
                    (doze_buf[2] == 'w') || (doze_buf[2] == 'y') || (doze_buf[2] == 'z') ||
                    (doze_buf[2] == 0x5E) /* ^ */
                    )
                {
                    if (doze_buf[2] != 0x5E)
                    {
                        GTP_INFO("Wakeup by gesture(%c), light up the screen!", doze_buf[2]);
                    }
                    else
                    {
                        GTP_INFO("Wakeup by gesture(^), light up the screen!");
                    }
                    doze_status = DOZE_WAKEUP;
                    input_report_key(tpd->dev, KEY_POWER, 1);
                    input_sync(tpd->dev);
                    input_report_key(tpd->dev, KEY_POWER, 0);
                    input_sync(tpd->dev);
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_client_point, doze_buf, 3);
                }
                else if ( (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
                    (doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA) )
                {
                    char *direction[4] = {"Right", "Down", "Up", "Left"};
                    u8 type = ((doze_buf[2] & 0x0F) - 0x0A) + (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;
                    
                    GTP_INFO("%s slide to light up the screen!", direction[type]);
                    doze_status = DOZE_WAKEUP;
                    input_report_key(tpd->dev, KEY_POWER, 1);
                    input_sync(tpd->dev);
                    input_report_key(tpd->dev, KEY_POWER, 0);
                    input_sync(tpd->dev);
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_client_point, doze_buf, 3);
                }
                else if (0xCC == doze_buf[2])
                {
                    GTP_INFO("Double click to light up the screen!");
                    doze_status = DOZE_WAKEUP;
                    input_report_key(tpd->dev, KEY_POWER, 1);
                    input_sync(tpd->dev);
                    input_report_key(tpd->dev, KEY_POWER, 0);
                    input_sync(tpd->dev);
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_client_point, doze_buf, 3);
                }
                else
                {
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_client_point, doze_buf, 3);
                    gtp_enter_doze(i2c_client_point);
                }
            }
            continue;
    #endif
	return;
	}
#endif
        ret = gtp_i2c_read(i2c_client_point, point_data, 12);
        if (ret < 0)
        {
            GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
            continue;
        }
        finger = point_data[GTP_ADDR_LENGTH];
        
    #if GTP_COMPATIBLE_MODE
        if ((finger == 0x00) && (CHIP_TYPE_GT9F == gtp_chip_type))
        {
            ret = gtp_i2c_read(i2c_client_point, rqst_data, 3);

            if(ret < 0)
            {
                GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
                continue;
            }
            switch (rqst_data[2])
            {
                case GTP_RQST_BAK_REF:
                    GTP_INFO("Request Ref.");
                    rqst_processing = 1;
                    ret = gtp_bak_ref_proc(i2c_client_point, GTP_BAK_REF_SEND);
                    if(SUCCESS == ret)
                    {
                        GTP_INFO("Send ref success.");
                        rqst_data[2] = GTP_RQST_RESPONDED;
                        gtp_i2c_write(i2c_client_point, rqst_data, 3);
                        rqst_processing = 0;
                    }
                    goto exit_work_func;
                    
                case GTP_RQST_CONFIG:
                    GTP_INFO("Request Config.");
                    ret = gtp_send_cfg(i2c_client_point);
                    if (ret < 0)
                    {
                        GTP_ERROR("Send config error.");
                    }
                    else 
                    {
                        GTP_INFO("Send config success.");
                        rqst_data[2] = GTP_RQST_RESPONDED;
                        gtp_i2c_write(i2c_client_point, rqst_data, 3);
                    }
                    goto exit_work_func;
                    
                case GTP_RQST_MAIN_CLOCK:
                    GTP_INFO("Request main clock.");
                    rqst_processing = 1;
                    ret = gtp_main_clk_proc(i2c_client_point);
                    if(SUCCESS == ret)
                    {
                        GTP_INFO("Send main clk success.");
                        
                        rqst_data[2] = GTP_RQST_RESPONDED;
                        gtp_i2c_write(i2c_client_point, rqst_data, 3);
                        rqst_processing = 0;
                    }
                    goto exit_work_func;
                    
                case GTP_RQST_RESET:
                    GTP_INFO("Request Reset.");
                    gtp_recovery_reset(i2c_client_point);
                    goto exit_work_func;
                    
                default:
                    GTP_INFO("Undefined request code: 0x%02X", rqst_data[2]);
                    rqst_data[2] = GTP_RQST_RESPONDED;
                    gtp_i2c_write(i2c_client_point, rqst_data, 3);
                    break;
            }
        }
    #endif
    
        if (finger == 0x00)
        {
            continue;
        }
        
        if ((finger & 0x80) == 0)
        {
            goto exit_work_func;
        }
        
    #ifdef TPD_PROXIMITY
        if (tpd_proximity_flag == 1)
        {
            proximity_status = point_data[GTP_ADDR_LENGTH];
            GTP_DEBUG("REG INDEX[0x814E]:0x%02X\n", proximity_status);

            if (proximity_status & 0x60)                //proximity or large touch detect,enable hwm_sensor.
            {
                tpd_proximity_detect = 0;
                //sensor_data.values[0] = 0;
            }
            else
            {
                tpd_proximity_detect = 1;
                //sensor_data.values[0] = 1;
            }

            //get raw data
            GTP_DEBUG(" ps change\n");
            GTP_DEBUG("PROXIMITY STATUS:0x%02X\n", tpd_proximity_detect);
            //map and store data to hwm_sensor_data
            sensor_data.values[0] = tpd_get_ps_value();
            sensor_data.value_divide = 1;
            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
            //report to the up-layer
            ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);

            if (ret)
            {
                GTP_ERROR("Call hwmsen_get_interrupt_data fail = %d\n", err);
            }
        }

    #endif

        touch_num = finger & 0x0f;

        if (touch_num > GTP_MAX_TOUCH)
        {
            goto exit_work_func;
        }

        if (touch_num > 1)
        {
            u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

            ret = gtp_i2c_read(i2c_client_point, buf, 2 + 8 * (touch_num - 1));
            memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
        }

    #if (GTP_HAVE_TOUCH_KEY || GTP_PEN_HAVE_BUTTON)
        key_value = point_data[3 + 8 * touch_num];

        if (key_value || pre_key)
        {
        #if GTP_PEN_HAVE_BUTTON
            if (key_value == 0x40)
            {
                GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Down.");
                input_report_key(pen_dev, BTN_STYLUS, 1);
                input_report_key(pen_dev, BTN_STYLUS2, 1);
                pen_active = 1;
            }
            else if (key_value == 0x10)
            {
                GTP_DEBUG("BTN_STYLUS Down, BTN_STYLUS2 Up.");
                input_report_key(pen_dev, BTN_STYLUS, 1);
                input_report_key(pen_dev, BTN_STYLUS2, 0);
                pen_active = 1;
            }
            else if (key_value == 0x20)
            {
                GTP_DEBUG("BTN_STYLUS Up, BTN_STYLUS2 Down.");
                input_report_key(pen_dev, BTN_STYLUS, 0);
                input_report_key(pen_dev, BTN_STYLUS2, 1);
                pen_active = 1;
            }
            else
            {
                GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Up.");
                input_report_key(pen_dev, BTN_STYLUS, 0);
                input_report_key(pen_dev, BTN_STYLUS2, 0);
                if ( (pre_key == 0x40) || (pre_key == 0x20) ||
                     (pre_key == 0x10) 
                   )
                {
                    pen_active = 1;
                }
            }
            if (pen_active)
            {
                touch_num = 0;      // shield pen point
                //pre_touch = 0;    // clear last pen status
            }
        #endif
        #if GTP_HAVE_TOUCH_KEY
            if (!pre_touch)
            {
                for (i = 0; i < GTP_MAX_KEY_NUM; i++)
                {
                    input_report_key(tpd->dev, touch_key_array[i], key_value & (0x01 << i));
                }
                touch_num = 0;  // shiled fingers
            }
        #endif
        }
    #endif
        pre_key = key_value;

        GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);
        
        if (touch_num)
        {
            for (i = 0; i < touch_num; i++)
            {
                coor_data = &point_data[i * 8 + 3];

                id = coor_data[0] & 0x0F;      
                input_x  = coor_data[1] | coor_data[2] << 8;
                input_y  = coor_data[3] | coor_data[4] << 8;
                input_w  = coor_data[5] | coor_data[6] << 8;

                input_x = TPD_WARP_X(abs_x_max, input_x);
                input_y = TPD_WARP_Y(abs_y_max, input_y);

            #if GTP_WITH_PEN
                id = coor_data[0];
                if ((id & 0x80))      // pen/stylus is activated
                {
                    GTP_DEBUG("Pen touch DOWN!");
                    pre_pen = 1;
                    //id &= 0x7F;
                    id = 0;
                    GTP_DEBUG("(%d)(%d, %d)[%d]", id, input_x, input_y, input_w);
                    gtp_pen_down(input_x, input_y, input_w, id);
                    pen_active = 1;
                }
                else
            #endif
                {
                    GTP_DEBUG(" (%d)(%d, %d)[%d]", id, input_x, input_y, input_w);
                    tpd_down(input_x, input_y, input_w, id);
                }
            }
        }
        else
        {
            if (pre_touch)
            {
            #if GTP_WITH_PEN
                if (pre_pen)
                {   
                    GTP_DEBUG("Pen touch UP!");
                    gtp_pen_up();
                    pre_pen = 0;
                    pen_active = 1;
                }
                else
            #endif
                {
                    GTP_DEBUG("Touch Release!");
                    tpd_up(0, 0, 0);
                }
            }
        }
        pre_touch = touch_num;
 
    #if GTP_WITH_PEN
        if (pen_active)
        {
            pen_active = 0;
            input_sync(pen_dev);
        }
        else
    #endif
        {
            input_sync(tpd->dev);
        }

exit_work_func:

        if (!gtp_rawdiff_mode)
        {
            ret = gtp_i2c_write(i2c_client_point, end_cmd, 3);

            if (ret < 0)
            {
                GTP_INFO("I2C write end_cmd  error!");
            }
        }

    } while (!kthread_should_stop());

    return 0;
}




static int tpd_local_init(void)
{
	
#ifdef TPD_POWER_SOURCE_CUSTOM
#ifdef CONFIG_OF_TOUCH
#ifdef CONFIG_ARCH_MT6580
	tpd->reg=regulator_get(tpd->tpd_dev,TPD_POWER_SOURCE_CUSTOM); // get pointer to regulator structure
	if (IS_ERR(tpd->reg)) {
		GTP_ERROR("regulator_get() failed!\n");
	}
#endif
#endif
#endif
	
#if GTP_ESD_PROTECT
    clk_tick_cnt = 2 * HZ;   // HZ: clock ticks in 1 second generated by system
    GTP_DEBUG("Clock ticks for an esd cycle: %d", clk_tick_cnt);
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
    spin_lock_init(&esd_lock);          // 2.6.39 & later
    // esd_lock = SPIN_LOCK_UNLOCKED;   // 2.6.39 & before
#endif

#if GTP_SUPPORT_I2C_DMA
    tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
    gpDMABuf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
    if(!gpDMABuf_va){
        GTP_INFO("[Error] Allocate DMA I2C Buffer failed!\n");
    }
    memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
#endif
    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        GTP_INFO("unable to add i2c driver.\n");
        return -1;
    }

    if (tpd_load_status == 0) //if(tpd_load_status == 0) // disable auto load touch driver for linux3.0 porting
    {
        GTP_INFO("add error touch panel driver.\n");
        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
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
    tpd->dev->id.product = tpd_info.pid;
    tpd->dev->id.version = tpd_info.vid;

    GTP_INFO("end %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;

    return 0;
}

#if GTP_GESTURE_WAKEUP
static s8 gtp_enter_doze(struct i2c_client *client)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

    GTP_DEBUG_FUNC();

    GTP_DEBUG("Entering gesture mode...");
    while(retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(client, i2c_control_buf, 3);
        if (ret < 0)
        {
            GTP_DEBUG("Failed to set gesture flag into 0x8046, %d", retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_status = DOZE_ENABLED;
            GTP_INFO("Gesture mode enabled.");
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send gesture cmd failed.");
    return ret;
}
//chenggang.li@bsp delete #else 2016/01/21

//#else

	
static s8 gtp_enter_sleep(struct i2c_client *client)
{
#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        u8 i2c_status_buf[3] = {0x80, 0x44, 0x00};
        s32 ret = 0;
      
        ret = gtp_i2c_read(client, i2c_status_buf, 3);
        if(ret <= 0)
        {
             GTP_ERROR("[gtp_enter_sleep]Read ref status reg error.");
        }
        
        if (i2c_status_buf[2] & 0x80)
        {
            //Store bak ref
            ret = gtp_bak_ref_proc(client, GTP_BAK_REF_STORE);
            if(FAIL == ret)
            {
                GTP_ERROR("[gtp_enter_sleep]Store bak ref failed.");
            }        
        }
    }
#endif


#if GTP_POWER_CTRL_SLEEP

    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);   
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(10);

#ifdef MT6573
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);  
    msleep(30);
#else               // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )

    #ifdef TPD_POWER_SOURCE_1800
        hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
    #endif
    
    #ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
    #else
        hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
    #endif
#endif
   
    GTP_INFO("GTP enter sleep by poweroff!");
    return 0;
    
#else
    {
        s8 ret = -1;
        s8 retry = 0;
        u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};
        
        
        GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
        msleep(5);
    
        while (retry++ < 5)
        {
            ret = gtp_i2c_write(client, i2c_control_buf, 3);
    
            if (ret > 0)
            {
                GTP_INFO("GTP enter sleep!");
                    
                return ret;
            }
    
            msleep(10);
        }
    
        GTP_ERROR("GTP send sleep cmd failed.");
        return ret;
    }
#endif
}
#endif

/*******************************************************
Function:
    Wakeup from sleep mode Function.

Input:
    client:i2c_client.

Output:
    Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_wakeup_sleep(struct i2c_client *client)
{
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG("GTP wakeup begin.");

#if (GTP_POWER_CTRL_SLEEP)   

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        force_reset_guitar();
        GTP_INFO("Esd recovery wakeup.");
        return 0;
    }
#endif

    while (retry++ < 5)
    {
        ret = tpd_power_on(client);

        if (ret < 0)
        {
            GTP_ERROR("I2C Power on ERROR!");
            continue;
        }
        GTP_INFO("Ic wakeup by poweron");
        return 0;
    }
#else

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        u8 opr_buf[2] = {0};
        
        while (retry++ < 10)
        {
            GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
            msleep(5);
            
            ret = gtp_i2c_test(client);
    
            if (ret >= 0)
            {  
                // Hold ss51 & dsp
                opr_buf[0] = 0x0C;
                ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
                if (ret < 0)
                {
                    GTP_DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);
                    continue;
                }
                
                // Confirm hold
                opr_buf[0] = 0x00;
                ret = i2c_read_bytes(client, 0x4180, opr_buf, 1);
                if (ret < 0)
                {
                    GTP_DEBUG("confirm ss51 & dsp hold, I2C error,retry:%d", retry);
                    continue;
                }
                if (0x0C != opr_buf[0])
                {
                    GTP_DEBUG("ss51 & dsp not hold, val: %d, retry: %d", opr_buf[0], retry);
                    continue;
                }
                GTP_DEBUG("ss51 & dsp has been hold");
                
                ret = gtp_fw_startup(client);
                if (FAIL == ret)
                {
                    GTP_ERROR("[gtp_wakeup_sleep]Startup fw failed.");
                    continue;
                }
                GTP_INFO("flashless wakeup sleep success");
                return ret;
            }
            force_reset_guitar();
            retry = 0;
            break;
        }
        if (retry >= 10)
        {
            GTP_ERROR("wakeup retry timeout, process esd reset");
            force_reset_guitar();
        }
        GTP_ERROR("GTP wakeup sleep failed.");
        return ret;
    }
#endif
    while (retry++ < 10)
    {
    #if GTP_GESTURE_WAKEUP
        if (DOZE_WAKEUP != doze_status)
        {
            GTP_INFO("Powerkey wakeup.");
        }
        else
        {
            GTP_INFO("Gesture wakeup.");
        }
        doze_status = DOZE_DISABLED;
        
        gtp_irq_disable();
        //mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        gtp_reset_guitar(client, 20);
        //mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        gtp_irq_enable();
    #else
    
        GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
        msleep(5);
    #endif
        
        ret = gtp_i2c_test(client);

        if (ret >= 0)
        {
            GTP_INFO("GTP wakeup sleep.");
        #if (!GTP_GESTURE_WAKEUP)
            {
                gtp_int_sync(25);
            #if GTP_ESD_PROTECT
                gtp_init_ext_watchdog(client);
            #endif
            }
        #endif
            
            return ret;
        }
        gtp_reset_guitar(client, 20);
    }
#endif
    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

/* Function to manage low power suspend */
static void tpd_suspend(struct early_suspend *h)
{
    s32 ret = -1;
	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
    GTP_INFO("System suspend.");

#ifdef TPD_PROXIMITY

    if (tpd_proximity_flag == 1)
    {
        return ;
    }

#endif

	ts->gtp_is_suspend = 1;
    tpd_halt = 1;
#if GTP_ESD_PROTECT
    gtp_esd_switch(i2c_client_point, SWITCH_OFF);
#endif
    
#if GTP_GESTURE_WAKEUP
    ret = gtp_enter_doze(i2c_client_point);
#else
		
    //mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    gtp_irq_disable();
    
    ret = gtp_enter_sleep(i2c_client_point);
#endif
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }
    // to avoid waking up while not sleeping, delay 48 + 10ms to ensure reliability 
    msleep(58);
}

/* Function to manage power-on resume */
static void tpd_resume(struct early_suspend *h)
{
    s32 ret = -1;

	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
    GTP_INFO("System resume.");

	ts->gtp_is_suspend = 0;
#ifdef TPD_PROXIMITY

    if (tpd_proximity_flag == 1)
    {
        return ;
    }

#endif
    ret = gtp_wakeup_sleep(i2c_client_point);

    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }
    
#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        // do nothing
    }
    else
#endif
    {
        gtp_send_cfg(i2c_client_point);
    }
    
#if GTP_CHARGER_SWITCH
    gtp_charger_switch(1);  // force update
#endif

    tpd_halt = 0;
#if GTP_GESTURE_WAKEUP
    doze_status = DOZE_DISABLED;
#else 
    //mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    gtp_irq_enable();
#endif

#if GTP_ESD_PROTECT
    gtp_esd_switch(i2c_client_point, SWITCH_ON);
#endif

}

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "gt9xx",
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
static int __init tpd_driver_init(void)
{
    GTP_INFO("MediaTek gt91xx touch panel driver init\n");

    i2c_register_board_info(I2C_BUS_NUMBER, &i2c_tpd, 1);
    if (tpd_driver_add(&tpd_device_driver) < 0)
        GTP_INFO("add generic driver failed\n");

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    GTP_INFO("MediaTek gt91xx touch panel driver exit\n");
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

