/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_Kernelc
 *
 * Project:
 * --------
 *    Audio smart pa Function
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/
#include "AudDrv_NXP.h"
/* yuanyan@Multimedia, 2016/02/23, add for 15131*/
#if defined(OPPO_MTK_15131_PROJECT)
#define TFA_I2C_CHANNEL     (0)
#else
#define TFA_I2C_CHANNEL     (4)
#endif

#ifdef CONFIG_MTK_NXP_TFA9890
/* yuanyan@Multimedia, 2016/02/23, add for 15131*/
#if defined(OPPO_MTK_15131_PROJECT)
#define ECODEC_SLAVE_ADDR_WRITE 0x6a
#define ECODEC_SLAVE_ADDR_READ  0x6b
#else
#define ECODEC_SLAVE_ADDR_WRITE 0x6c
#define ECODEC_SLAVE_ADDR_READ  0x6d
#endif
#else
#define ECODEC_SLAVE_ADDR_WRITE 0x6c
#define ECODEC_SLAVE_ADDR_READ  0x6d
#endif

#define NXPEXTSPK_I2C_DEVNAME "i2c_TFA98XX"

#undef GPIO_TFA9890_RST_PIN
#define GPIO_TFA9890_RST_PIN (GPIO85| 0x80000000)

/*****************************************************************************
*           DEFINE AND CONSTANT
******************************************************************************
*/

#define AUDDRV_NXPSPK_NAME   "MediaTek Audio NXPSPK Driver"
#define AUDDRV_AUTHOR "MediaTek WCX"
#define RW_BUFFER_LENGTH (256)
#define I2C_RETRY_CNT (5)
/*When the size to be transfered is bigger than 8 bytes. I2C DMA way should be used.*/
#define MTK_I2C_DMA_THRES (8)
#define G_BUF_SIZE (1024)

static u8 *DMAbuffer_va = NULL;
static dma_addr_t DMAbuffer_pa = NULL;
static char *g_buf = NULL;


/*****************************************************************************
*           V A R I A B L E     D E L A R A T I O N
*******************************************************************************/

static char       auddrv_nxpspk_name[]       = "TFA98XX_DRV";
// I2C variable
static struct i2c_client *new_client = NULL;
char WriteBuffer[RW_BUFFER_LENGTH];
char ReadBuffer[RW_BUFFER_LENGTH];

// new I2C register method
static const struct i2c_device_id NXPExt_i2c_id[] = {{NXPEXTSPK_I2C_DEVNAME, 0}, {}};
static struct i2c_board_info __initdata  NXPExt_dev = {I2C_BOARD_INFO(NXPEXTSPK_I2C_DEVNAME, (ECODEC_SLAVE_ADDR_WRITE >> 1))};

static bool NxpSpkSuspendStatus = false;

//function declration
static int NXPExtSpk_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int NXPExtSpk_i2c_remove(struct i2c_client *client);
void AudDrv_NXPSpk_Init(void);
bool NXPExtSpk_Register(void);
static int NXPExtSpk_register(void);
ssize_t  NXPSpk_read_byte(u8 addr, u8 *returnData);
void AudDrv_NXPSpk_Init(void);


//i2c driver
struct i2c_driver NXPExtSpk_i2c_driver =
{
    .probe = NXPExtSpk_i2c_probe,
    .remove = NXPExtSpk_i2c_remove,
    .driver = {
        .owner	= THIS_MODULE,
        .name = NXPEXTSPK_I2C_DEVNAME,
    },
    .id_table = NXPExt_i2c_id,
};

static int NXPExtSpk_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    new_client = client;
#ifdef VENDOR_EDIT
/* yongzhi.zhang@Multimedia, 2016/01/21, modify by HW's request about avoiding the impact on GPS */
    new_client->timing = 350;
/* yongzhi.zhang@Multimedia, 2016/01/21, modify by HW's request about avoiding the impact on GPS */
#endif
    //smart pa DMA support
    if (DMAbuffer_va == NULL)
    {
        DMAbuffer_va = (u8 *)dma_alloc_coherent(&(client->dev), 1024, &DMAbuffer_pa, GFP_KERNEL);
    }
    if (!DMAbuffer_va)
    {
        printk("Allocate DMA I2C Buffer failed!\n");
        return -1;
    }


    g_buf = kmalloc(G_BUF_SIZE, GFP_KERNEL);
    if (g_buf == NULL)
    {
        printk("kmalloc for g_buf failed!");
        return -ENOMEM;
    }
    #ifndef S6E3FA3_FHD_DSI_CMD 
	//rendong.shi@Bsp.lcd ,2014/02/09, modify for 15011 vgp3 need 3.3V ....
     //hwPowerOn(MT6325_POWER_LDO_VGP3,VOL_1800,"tfa98xx");
	#else
     //  hwPowerOn(MT6325_POWER_LDO_VGP3,VOL_1800,"tfa98xx");
   #endif
#ifdef CONFIG_MTK_NXP_TFA9890
    printk("tfa9890 rst \n");
    mt_set_gpio_mode(GPIO_TFA9890_RST_PIN, GPIO_MODE_00);
   // mt_set_gpio_out(GPIO_TFA9890_RST_PIN/*GPIO130*/, GPIO_OUT_ZERO);
   // msleep(2);
    mt_set_gpio_dir(GPIO_TFA9890_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_TFA9890_RST_PIN, GPIO_OUT_ZERO);
    //msleep(2);
   // mt_set_gpio_out(GPIO_TFA9890_RST_PIN, GPIO_OUT_ZERO);
    //msleep(10);
#endif

    //printk("client new timing=%dK \n", new_client->timing);
    return 0;
}

static int NXPExtSpk_i2c_remove(struct i2c_client *client)
{

    if (DMAbuffer_va)
    {
        dma_free_coherent(&(client->dev), 4096, DMAbuffer_va, DMAbuffer_pa);
        DMAbuffer_va = NULL;
        DMAbuffer_pa = 0;
    }

    if (g_buf != NULL)
    {
        kfree(g_buf);
    }

    new_client = NULL;
    i2c_unregister_device(client);
    i2c_del_driver(&NXPExtSpk_i2c_driver);
#if 0//def CONFIG_MTK_NXP_TFA9890
    msleep(1);
    mt_set_gpio_mode(GPIO_AUD_EXTHP_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_AUD_EXTHP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_AUD_EXTHP_EN_PIN, GPIO_OUT_ZERO);
#endif
    return 0;
}

int NXPspk_i2c_dma_write(struct i2c_client *client, U8 *txbuf, U16 len)
{
    int retry, i;
    int msg_cnt = 0;
    /* xiaojun.lv@AudioDrv,2015/07/08,this log is useless at driver is stable,delete it */
    //printk("NXPspk_i2c_dma_write len = %d\n",len);
    memset(DMAbuffer_va, 0, G_BUF_SIZE);
    for (i = 0; i < len; i++)
    {
        DMAbuffer_va[i] = txbuf[i];
    }

    struct i2c_msg msg[] =
    {
        {
            .addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG,
            .flags = 0,
            .len = len,
            .buf = DMAbuffer_pa,
			.timing = client->timing,

        }
    };

    msg_cnt = sizeof(msg) / sizeof(msg[0]);
    for (retry = 0; retry < I2C_RETRY_CNT; retry++)
    {
        if (i2c_transfer(client->adapter, msg, msg_cnt) == msg_cnt)
        {
            break;
        }
        mdelay(5);
    }

    if (retry == I2C_RETRY_CNT)
    {
        printk(KERN_ERR "i2c_read_block retry over %d\n", I2C_RETRY_CNT);
        return -EIO;
    }

    return 0;
}

int NXPspk_i2c_dma_read(const struct i2c_client *client, char *buf, int count)
{	
    int ret;	
    struct i2c_msg msg;	
    /* xiaojun.lv@AudioDrv,2015/07/08,this log is useless at driver is stable,delete it */
    //printk("NXPspk_i2c_dma_read count = %d\n",count);
    msg.flags = 0;	
    msg.flags |= I2C_M_RD;	
    msg.len = count;	
    msg.ext_flag = client->ext_flag;	
    msg.buf = (char *)buf;	
    msg.addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;	
    msg.timing = client->timing;

    ret = i2c_transfer(client->adapter, &msg, 1);	
    /*	
    * If everything went ok (i.e. 1 msg received), return #bytes received,	 
    * else error code.	
    */	
    return (ret == 1) ? count : ret;
} 

// read write implementation
//read one register
ssize_t  NXPSpk_read_byte(u8 addr, u8 *returnData)
{
    char     cmd_buf[1] = {0x00};
    char     readData = 0;
    int     ret = 0;
    cmd_buf[0] = addr;

    if (!new_client)
    {
        printk("NXPSpk_read_byte I2C client not initialized!!");
        return -1;
    }
    ret = i2c_master_send(new_client, &cmd_buf[0], 1);
    if (ret < 0)
    {
        printk("NXPSpk_read_byte read sends command error!!\n");
        return -1;
    }
    ret = i2c_master_recv(new_client, &readData, 1);
    if (ret < 0)
    {
        printk("NXPSpk_read_byte reads recv data error!!\n");
        return -1;
    }
    *returnData = readData;
    printk("addr 0x%x data 0x%x \n", addr, readData);
    return 0;
}

//write register
ssize_t  NXPExt_write_byte(u8 addr, u8 writeData)
{
    if (!new_client)
    {
        printk("I2C client not initialized!!");
        return -1;
    }
    char    write_data[2] = {0};
    int    ret = 0;
    write_data[0] = addr;         // ex. 0x01
    write_data[1] = writeData;
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0)
    {
        printk("write sends command error!!");
        return -1;
    }
    printk("addr 0x%x data 0x%x \n", addr, writeData);
    return 0;
}


static int NXPExtSpk_register()
{
   int ret=0;
    printk("+NXPExtSpk_register \n");
#if 0//def CONFIG_MTK_NXP_TFA9890
    mt_set_gpio_mode(GPIO_AUD_EXTHP_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_AUD_EXTHP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_AUD_EXTHP_EN_PIN, GPIO_OUT_ONE);
    msleep(1);
#endif
    ret = i2c_register_board_info(TFA_I2C_CHANNEL, &NXPExt_dev, 1);
    printk("i2c_register_board_info,ret=%d \n",ret);
    ret = i2c_add_driver(&NXPExtSpk_i2c_driver);
    printk("i2c_add_driver ret=%d\n",ret);
    
   /* if (i2c_add_driver(&NXPExtSpk_i2c_driver))
    {
        printk("fail to add device into i2c");
        return -1;
    }*/
    return 0;
}


bool NXPExtSpk_Register(void)
{
    printk("NXPExtSpk_Register \n");
    NXPExtSpk_register();
    return true;
}

void AudDrv_NXPSpk_Init(void)
{
#if 0
    printk("Set GPIO for AFE I2S output to external DAC \n");
    mt_set_gpio_mode(GPIO_NXPSPA_I2S_LRCK_PIN , GPIO_MODE_01);
    mt_set_gpio_mode(GPIO_NXPSPA_I2S_BCK_PIN, GPIO_MODE_01);
    mt_set_gpio_mode(GPIO_NXPSPA_I2S_DATAIN_PIN, GPIO_MODE_01);
    mt_set_gpio_mode(GPIO_NXPSPA_I2S_DATAOUT_PIN, GPIO_MODE_01);
#endif
}

/*****************************************************************************
 * FILE OPERATION FUNCTION
 *  AudDrv_nxpspk_ioctl
 *
 * DESCRIPTION
 *  IOCTL Msg handle
 *
 *****************************************************************************
 */
static long AudDrv_nxpspk_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int  ret = 0;

    //printk("AudDrv_nxpspk_ioctl cmd = 0x%x arg = %lu\n", cmd, arg);

    switch (cmd)
    {
        default:
        {
            //printk("AudDrv_nxpspk_ioctl Fail command: %x \n", cmd);
            ret = -1;
            break;
        }
    }
    return 1;
}

static int AudDrv_nxpspk_probe(struct platform_device *dev)
{
    int ret = 0;
    printk("AudDrv_nxpspk_probe \n");

    if (ret < 0)
    {
        printk("AudDrv_nxpspk_probe request_irq MT6582_AP_BT_CVSD_IRQ_LINE Fail \n");
    }
    NXPExtSpk_Register();
    AudDrv_NXPSpk_Init();

    memset((void *)WriteBuffer, 0, RW_BUFFER_LENGTH);
    memset((void *)ReadBuffer, 0, RW_BUFFER_LENGTH);

    printk("-AudDrv_nxpspk_probe \n");
    return 0;
}

static int AudDrv_nxpspk_open(struct inode *inode, struct file *fp)
{
    return 0;
}

static ssize_t AudDrv_nxpspk_write(struct file *fp, const char __user *buf, size_t count, loff_t *offset)
{
    int ret;
    char *tmp;

    //printk("AudDrv_nxpspk_write  count = %d\n" , (int)count);
    if (!new_client)
    {
        printk("AudDrv_nxpspk_write I2C client not initialized!!");
        return -1;
    }

    if (count > 8192)
    {
        count = 8192;
    }

    if (g_buf == NULL)
    {
        printk("AudDrv_nxpspk_write g_buf NULL!!");
        return -ENOMEM;
    }

    tmp = g_buf;
    memset(tmp, 0, G_BUF_SIZE);

    if (copy_from_user(tmp, buf, count))
    {
        printk("AudDrv_nxpspk_write copy_from_user failed!!");
        return -EFAULT;
    }

    if (count < MTK_I2C_DMA_THRES)
    {
        //printk("AudDrv_nxpspk_write i2c_master_send( tmp, count) count = %d \n",(int)count);
        ret = i2c_master_send(new_client, tmp, count);
    }
    else
    {
        /* xiaojun.lv@AudioDrv,2015/07/08,this log is useless at driver is stable,delete it */
        //printk("AudDrv_nxpspk_write Nxpspk_i2c_dma_write( tmp, count)count = %d \n",(int)count);
        if (DMAbuffer_va == NULL)
        {
            printk("AudDrv_nxpspk_write DMAbuffer_va is NULL !!!");
            return -ENOMEM;
        }
        else
        {
            ret = NXPspk_i2c_dma_write(new_client, tmp, count);
        }
    }

    if (ret < 0)
    {
        printk("AudDrv_nxpspk_write i2c write failed!(%d)\n", ret);
    }
    return ret;

}

static ssize_t AudDrv_nxpspk_read(struct file *fp,  char __user *data, size_t count, loff_t *offset)
{
    int read_count = count;
    int ret = 0;

    //printk("AudDrv_nxpspk_read  count = %d\n", (int)count);
    if (!access_ok(VERIFY_READ, data, count))
    {
        printk("AudDrv_nxpspk_read !access_ok\n");
        return count;
    }
    else
    {
        // copy data from user space
        if (copy_from_user(ReadBuffer, data, count))
        {
            printk("AudDrv_nxpspk_read Fail copy from user \n");
            return -1;
        }
        char *Read_ptr = &ReadBuffer[0];
        //printk("data0 = 0x%x data1 = 0x%x \n",  ReadBuffer[0], ReadBuffer[1]);

        /*
        ret = i2c_master_send(new_client,  &ReadBuffer[0], 1);
        if (ret < 0)
        {
            printk("AudDrv_nxpspk_read read sends command error!!\n");
            return -1;
        }
        */

        //printk("i2c_master_recv read_count = %d \n", read_count);
        if(count < 8)
        {
            ret = i2c_master_recv(new_client, Read_ptr, read_count);
        }
        else
        {
            if (DMAbuffer_va == NULL)
            {
                printk("AudDrv_nxpspk_read DMAbuffer_va is NULL !!!");
                return -ENOMEM;
            }
            else
            {
                int i = 0;
                ret = NXPspk_i2c_dma_read(new_client, DMAbuffer_pa, read_count);
                for(i = 0; i < count; i++)
                {
                    ReadBuffer[i] = DMAbuffer_va[i];
                }
            }
        }
        if (ret < 0)
        {
            printk("AudDrv_nxpspk_read write sends command error!!");
            return 0;
        }
        //printk("AudDrv_nxpspk_read data0 = 0x%x data1 = 0x%x  \n",  (int)ReadBuffer[0], (int)ReadBuffer[1]);
        if (copy_to_user((void __user *)data, (void *)ReadBuffer, count))
        {
            printk("AudDrv_nxpspk_read Fail copy from user \n");
            return -1;
        }
    }
    return count;
}


/**************************************************************************
 * STRUCT
 *  File Operations and misc device
 *
 **************************************************************************/

static struct file_operations AudDrv_nxpspk_fops =
{
    .owner   = THIS_MODULE,
    .open    = AudDrv_nxpspk_open,
    .unlocked_ioctl   = AudDrv_nxpspk_ioctl,
    .write   = AudDrv_nxpspk_write,
    .read    = AudDrv_nxpspk_read,
};

#ifdef CONFIG_MTK_NXP_TFA9890
static struct miscdevice AudDrv_nxpspk_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "i2c0_tfa9890",//"smartpa_i2c",
    .fops = &AudDrv_nxpspk_fops,
};
#else
static struct miscdevice AudDrv_nxpspk_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "nxpspk",
    .fops = &AudDrv_nxpspk_fops,
};
#endif

static int mtk_nxp_pm_ops_suspend(struct device *device)
{
    printk("%s \n", __func__);
    if (NxpSpkSuspendStatus == false)
    {
        NxpSpkSuspendStatus = true;
    }
    return 0;
}

static int mtk_nxp_pm_ops_resume(struct device *device)
{
    printk("%s \n ", __func__);
    if (NxpSpkSuspendStatus == true)
    {
        NxpSpkSuspendStatus = false;
#if 0//def CONFIG_MTK_NXP_TFA9890
        mt_set_gpio_mode(GPIO_AUD_EXTDAC_RST_PIN/*GPIO130*/, GPIO_MODE_00);
        mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN/*GPIO130*/, GPIO_OUT_ZERO);
        msleep(1);
        mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN/*GPIO130*/, GPIO_OUT_ONE);
        msleep(1);
        mt_set_gpio_out(GPIO_AUD_EXTDAC_RST_PIN/*GPIO130*/, GPIO_OUT_ZERO);
        msleep(1);
#endif
        AudDrv_NXPSpk_Init();
    }
    return 0;
}

struct dev_pm_ops Nxp_pm_ops =
{
    .suspend = mtk_nxp_pm_ops_suspend,
    .resume = mtk_nxp_pm_ops_resume,
    .freeze = mtk_nxp_pm_ops_suspend,
    .thaw = mtk_nxp_pm_ops_resume,
    .poweroff = NULL,
    .restore = mtk_nxp_pm_ops_resume,
    .restore_noirq = NULL,
};


/***************************************************************************
 * FUNCTION
 *  AudDrv_nxpspk_mod_init / AudDrv_nxpspk_mod_exit
 *
 * DESCRIPTION
 *  Module init and de-init (only be called when system boot up)
 *
 **************************************************************************/

static struct platform_driver AudDrv_nxpspk =
{
    .probe    = AudDrv_nxpspk_probe,
    .driver   = {
        .name = auddrv_nxpspk_name,
#ifdef CONFIG_PM
        .pm     = &Nxp_pm_ops,
#endif
    },
};

static struct platform_device *AudDrv_NXPSpk_dev;

static int AudDrv_nxpspk_mod_init(void)
{
    int ret = 0;
    printk("+AudDrv_nxpspk_mod_init \n");


    printk("platform_device_alloc  \n");       
    AudDrv_NXPSpk_dev = platform_device_alloc(auddrv_nxpspk_name, -1);
    if (!AudDrv_NXPSpk_dev)
    {
        return -ENOMEM;
    }

    printk("platform_device_add  \n");

    ret = platform_device_add(AudDrv_NXPSpk_dev);
    if (ret != 0)
    {
        platform_device_put(AudDrv_NXPSpk_dev);
        return ret;
    }

    // Register platform DRIVER
    ret = platform_driver_register(&AudDrv_nxpspk);
    if (ret)
    {
        printk("AudDrv Fail:%d - Register DRIVER \n", ret);
        return ret;
    }

    // register MISC device
    if ((ret = misc_register(&AudDrv_nxpspk_device)))
    {
        printk("AudDrv_nxpspk_mod_init misc_register Fail:%d \n", ret);
        return ret;
    }

    printk("-AudDrv_nxpspk_mod_init\n");
    return 0;
}

static void  AudDrv_nxpspk_mod_exit(void)
{
    printk("+AudDrv_nxpspk_mod_exit \n");

    printk("-AudDrv_nxpspk_mod_exit \n");
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(AUDDRV_NXPSPK_NAME);
MODULE_AUTHOR(AUDDRV_AUTHOR);
//subsys_initcall(AudDrv_nxpspk_mod_init);
module_init(AudDrv_nxpspk_mod_init);
module_exit(AudDrv_nxpspk_mod_exit);

EXPORT_SYMBOL(NXPSpk_read_byte);
EXPORT_SYMBOL(NXPExt_write_byte);


