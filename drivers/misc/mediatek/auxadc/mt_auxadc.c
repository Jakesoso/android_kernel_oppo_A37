/*****************************************************************************
 *
 * Filename:
 * ---------
 *    mt_auxadc.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of AUXADC common code
 *
 * Author:
 * -------
 * Zhong Wang
 *
 ****************************************************************************/

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/mt_gpt.h>
#include <mach/sync_write.h>

#include "mt_auxadc.h"
#include <mt_auxadc_sw.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#endif

#if !defined(CONFIG_MTK_CLKMGR)
#include <linux/clk.h>
#else
#include <cust_adc.h>		/* generate by DCT Tool */
#include <mach/mt_clkmgr.h>
#endif				/* defined(CONFIG_MTK_LEGACY) */

#if !defined(CONFIG_MTK_LEGACY)
#else
#include <cust_adc.h>		/* generate by DCT Tool */
#endif

#if !defined(CONFIG_MTK_CLKMGR)
#ifdef CONFIG_OF
static struct clk *clk_auxadc;
#endif
#endif

/*****************************************************************************
 * Integrate with NVRAM
****************************************************************************/
#define AUXADC_CALI_DEVNAME    "mtk-adc-cali"

#define TEST_ADC_CALI_PRINT _IO('k', 0)
#define SET_ADC_CALI_Slop   _IOW('k', 1, int)
#define SET_ADC_CALI_Offset _IOW('k', 2, int)
#define SET_ADC_CALI_Cal    _IOW('k', 3, int)
#define ADC_CHANNEL_READ    _IOW('k', 4, int)

typedef struct adc_info {
	char channel_name[64];
	int channel_number;
	int reserve1;
	int reserve2;
	int reserve3;
} ADC_INFO;

static ADC_INFO g_adc_info[ADC_CHANNEL_MAX];
static int auxadc_cali_slop[ADC_CHANNEL_MAX] = { 0 };
static int auxadc_cali_offset[ADC_CHANNEL_MAX] = { 0 };

static kal_bool g_AUXADC_Cali = KAL_FALSE;

static int auxadc_cali_cal[1] = { 0 };
static int auxadc_in_data[2] = { 1, 1 };
static int auxadc_out_data[2] = { 1, 1 };

static DEFINE_MUTEX(auxadc_mutex);
static dev_t auxadc_cali_devno;
static int auxadc_cali_major;
static struct cdev *auxadc_cali_cdev;
static struct class *auxadc_cali_class;

static struct task_struct *thread;
static int g_start_debug_thread;

static int g_adc_init_flag;

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // fop Common API */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
int IMM_IsAdcInitReady(void)
{
	return g_adc_init_flag;
}

int IMM_get_adc_channel_num(char *channel_name, int len)
{
	unsigned int i;

	pr_debug("[ADC] name = %s\n", channel_name);
	pr_debug("[ADC] name_len = %d\n", len);
	for (i = 0; i < ADC_CHANNEL_MAX; i++) {
		if (!strncmp(channel_name, g_adc_info[i].channel_name, len)) {
			return g_adc_info[i].channel_number;
		}
	}
	pr_err("[ADC] find channel number failed\n");
	return -1;
}

int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata)
{
	return IMM_auxadc_GetOneChannelValue(dwChannel, data, rawdata);
}

/* 1v == 1000000 uv */
/* this function voltage Unit is uv */
int IMM_GetOneChannelValue_Cali(int Channel, int *voltage)
{
	return IMM_auxadc_GetOneChannelValue_Cali(Channel, voltage);
}


/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // fop API */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static long auxadc_cali_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int i = 0, ret = 0;
	long *user_data_addr;
	long *nvram_data_addr;

	mutex_lock(&auxadc_mutex);

	switch (cmd) {
	case TEST_ADC_CALI_PRINT:
		g_AUXADC_Cali = KAL_FALSE;
		break;

	case SET_ADC_CALI_Slop:
		nvram_data_addr = (long *)arg;
		ret = copy_from_user(auxadc_cali_slop, nvram_data_addr, 36);
		g_AUXADC_Cali = KAL_FALSE;
		/* Protection */
		for (i = 0; i < ADC_CHANNEL_MAX; i++) {
			if ((*(auxadc_cali_slop + i) == 0) || (*(auxadc_cali_slop + i) == 1)) {
				*(auxadc_cali_slop + i) = 1000;
			}
		}
		for (i = 0; i < ADC_CHANNEL_MAX; i++)
			pr_debug("auxadc_cali_slop[%d] = %d\n", i, *(auxadc_cali_slop + i));
		pr_debug("**** MT auxadc_cali ioctl : SET_ADC_CALI_Slop Done!\n");
		break;

	case SET_ADC_CALI_Offset:
		nvram_data_addr = (long *)arg;
		ret = copy_from_user(auxadc_cali_offset, nvram_data_addr, 36);
		g_AUXADC_Cali = KAL_FALSE;
		for (i = 0; i < ADC_CHANNEL_MAX; i++)
			pr_debug("auxadc_cali_offset[%d] = %d\n", i, *(auxadc_cali_offset + i));
		pr_debug("**** MT auxadc_cali ioctl : SET_ADC_CALI_Offset Done!\n");
		break;

	case SET_ADC_CALI_Cal:
		nvram_data_addr = (long *)arg;
		ret = copy_from_user(auxadc_cali_cal, nvram_data_addr, 4);
		g_AUXADC_Cali = KAL_TRUE;	/* enable calibration after setting AUXADC_CALI_Cal */
		if (auxadc_cali_cal[0] == 1) {
			g_AUXADC_Cali = KAL_TRUE;
		} else {
			g_AUXADC_Cali = KAL_FALSE;
		}
		for (i = 0; i < 1; i++)
			pr_debug("auxadc_cali_cal[%d] = %d\n", i, *(auxadc_cali_cal + i));
		pr_debug("**** MT auxadc_cali ioctl : SET_ADC_CALI_Cal Done!\n");
		break;

	case ADC_CHANNEL_READ:
		g_AUXADC_Cali = KAL_FALSE;	/* 20100508 Infinity */
		user_data_addr = (long *)arg;
		ret = copy_from_user(auxadc_in_data, user_data_addr, 8);	/* 2*int = 2*4 */

		pr_debug("this api is removed !!\n");
		ret = copy_to_user(user_data_addr, auxadc_out_data, 8);
		pr_debug("**** ioctl : AUXADC Channel %d * %d times = %d\n", auxadc_in_data[0],
			 auxadc_in_data[1], auxadc_out_data[0]);
		break;

	default:
		g_AUXADC_Cali = KAL_FALSE;
		break;
	}

	mutex_unlock(&auxadc_mutex);

	return 0;
}


#ifdef CONFIG_COMPAT
static long compat_auxadc_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		pr_err("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case TEST_ADC_CALI_PRINT:
	case SET_ADC_CALI_Slop:
	case SET_ADC_CALI_Offset:
	case SET_ADC_CALI_Cal:
	case ADC_CHANNEL_READ:
		return filp->f_op->unlocked_ioctl(filp, cmd,
			(unsigned long)compat_ptr(arg));
	default:
		pr_err("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif


static int auxadc_cali_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int auxadc_cali_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations auxadc_cali_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = auxadc_cali_unlocked_ioctl,
	.open = auxadc_cali_open,
	.release = auxadc_cali_release,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_auxadc_unlocked_ioctl,
#endif
};

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : AUXADC_Channel_X_Slope/Offset */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
#if ADC_CHANNEL_MAX > 0
static ssize_t show_AUXADC_Channel_0_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 0));
	pr_debug("[EM] AUXADC_Channel_0_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_0_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_0_Slope, 0664, show_AUXADC_Channel_0_Slope,
		   store_AUXADC_Channel_0_Slope);
static ssize_t show_AUXADC_Channel_0_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 0));
	pr_debug("[EM] AUXADC_Channel_0_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_0_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_0_Offset, 0664, show_AUXADC_Channel_0_Offset,
		   store_AUXADC_Channel_0_Offset);
#endif


#if ADC_CHANNEL_MAX > 1
static ssize_t show_AUXADC_Channel_1_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 1));
	pr_debug("[EM] AUXADC_Channel_1_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_1_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_1_Slope, 0664, show_AUXADC_Channel_1_Slope,
		   store_AUXADC_Channel_1_Slope);
static ssize_t show_AUXADC_Channel_1_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 1));
	pr_debug("[EM] AUXADC_Channel_1_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_1_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_1_Offset, 0664, show_AUXADC_Channel_1_Offset,
		   store_AUXADC_Channel_1_Offset);
#endif


#if ADC_CHANNEL_MAX > 2
static ssize_t show_AUXADC_Channel_2_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 2));
	pr_debug("[EM] AUXADC_Channel_2_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_2_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_2_Slope, 0664, show_AUXADC_Channel_2_Slope,
		   store_AUXADC_Channel_2_Slope);
static ssize_t show_AUXADC_Channel_2_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 2));
	pr_debug("[EM] AUXADC_Channel_2_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_2_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_2_Offset, 0664, show_AUXADC_Channel_2_Offset,
		   store_AUXADC_Channel_2_Offset);
#endif


#if ADC_CHANNEL_MAX > 3
static ssize_t show_AUXADC_Channel_3_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 3));
	pr_debug("[EM] AUXADC_Channel_3_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_3_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_3_Slope, 0664, show_AUXADC_Channel_3_Slope,
		   store_AUXADC_Channel_3_Slope);
static ssize_t show_AUXADC_Channel_3_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 3));
	pr_debug("[EM] AUXADC_Channel_3_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_3_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_3_Offset, 0664, show_AUXADC_Channel_3_Offset,
		   store_AUXADC_Channel_3_Offset);
#endif


#if ADC_CHANNEL_MAX > 4
static ssize_t show_AUXADC_Channel_4_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 4));
	pr_debug("[EM] AUXADC_Channel_4_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_4_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_4_Slope, 0664, show_AUXADC_Channel_4_Slope,
		   store_AUXADC_Channel_4_Slope);
static ssize_t show_AUXADC_Channel_4_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 4));
	pr_debug("[EM] AUXADC_Channel_4_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_4_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_4_Offset, 0664, show_AUXADC_Channel_4_Offset,
		   store_AUXADC_Channel_4_Offset);
#endif


#if ADC_CHANNEL_MAX > 5
static ssize_t show_AUXADC_Channel_5_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 5));
	pr_debug("[EM] AUXADC_Channel_5_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_5_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_5_Slope, 0664, show_AUXADC_Channel_5_Slope,
		   store_AUXADC_Channel_5_Slope);
static ssize_t show_AUXADC_Channel_5_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 5));
	pr_debug("[EM] AUXADC_Channel_5_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_5_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_5_Offset, 0664, show_AUXADC_Channel_5_Offset,
		   store_AUXADC_Channel_5_Offset);
#endif


#if ADC_CHANNEL_MAX > 6
static ssize_t show_AUXADC_Channel_6_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 6));
	pr_debug("[EM] AUXADC_Channel_6_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_6_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_6_Slope, 0664, show_AUXADC_Channel_6_Slope,
		   store_AUXADC_Channel_6_Slope);
static ssize_t show_AUXADC_Channel_6_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 6));
	pr_debug("[EM] AUXADC_Channel_6_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_6_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_6_Offset, 0664, show_AUXADC_Channel_6_Offset,
		   store_AUXADC_Channel_6_Offset);
#endif


#if ADC_CHANNEL_MAX > 7
static ssize_t show_AUXADC_Channel_7_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 7));
	pr_debug("[EM] AUXADC_Channel_7_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_7_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_7_Slope, 0664, show_AUXADC_Channel_7_Slope,
		   store_AUXADC_Channel_7_Slope);
static ssize_t show_AUXADC_Channel_7_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 7));
	pr_debug("[EM] AUXADC_Channel_7_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_7_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_7_Offset, 0664, show_AUXADC_Channel_7_Offset,
		   store_AUXADC_Channel_7_Offset);
#endif


#if ADC_CHANNEL_MAX > 8
static ssize_t show_AUXADC_Channel_8_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 8));
	pr_debug("[EM] AUXADC_Channel_8_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_8_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_8_Slope, 0664, show_AUXADC_Channel_8_Slope,
		   store_AUXADC_Channel_8_Slope);
static ssize_t show_AUXADC_Channel_8_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 8));
	pr_debug("[EM] AUXADC_Channel_8_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_8_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_8_Offset, 0664, show_AUXADC_Channel_8_Offset,
		   store_AUXADC_Channel_8_Offset);
#endif


#if ADC_CHANNEL_MAX > 9
static ssize_t show_AUXADC_Channel_9_Slope(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 9));
	pr_debug("[EM] AUXADC_Channel_9_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_9_Slope(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_9_Slope, 0664, show_AUXADC_Channel_9_Slope,
		   store_AUXADC_Channel_9_Slope);
static ssize_t show_AUXADC_Channel_9_Offset(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 9));
	pr_debug("[EM] AUXADC_Channel_9_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_9_Offset(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_9_Offset, 0664, show_AUXADC_Channel_9_Offset,
		   store_AUXADC_Channel_9_Offset);
#endif


#if ADC_CHANNEL_MAX > 10
static ssize_t show_AUXADC_Channel_10_Slope(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 10));
	pr_debug("[EM] AUXADC_Channel_10_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_10_Slope(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_10_Slope, 0664, show_AUXADC_Channel_10_Slope,
		   store_AUXADC_Channel_10_Slope);
static ssize_t show_AUXADC_Channel_10_Offset(struct device *dev, struct device_attribute *attr,
					     char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 10));
	pr_debug("[EM] AUXADC_Channel_10_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_10_Offset(struct device *dev, struct device_attribute *attr,
					      const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_10_Offset, 0664, show_AUXADC_Channel_10_Offset,
		   store_AUXADC_Channel_10_Offset);
#endif


#if ADC_CHANNEL_MAX > 11
static ssize_t show_AUXADC_Channel_11_Slope(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 11));
	pr_debug("[EM] AUXADC_Channel_11_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_11_Slope(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_11_Slope, 0664, show_AUXADC_Channel_11_Slope,
		   store_AUXADC_Channel_11_Slope);
static ssize_t show_AUXADC_Channel_11_Offset(struct device *dev, struct device_attribute *attr,
					     char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 11));
	pr_debug("[EM] AUXADC_Channel_11_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_11_Offset(struct device *dev, struct device_attribute *attr,
					      const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_11_Offset, 0664, show_AUXADC_Channel_11_Offset,
		   store_AUXADC_Channel_11_Offset);
#endif


#if ADC_CHANNEL_MAX > 12
static ssize_t show_AUXADC_Channel_12_Slope(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 12));
	pr_debug("[EM] AUXADC_Channel_12_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_12_Slope(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_12_Slope, 0664, show_AUXADC_Channel_12_Slope,
		   store_AUXADC_Channel_12_Slope);
static ssize_t show_AUXADC_Channel_12_Offset(struct device *dev, struct device_attribute *attr,
					     char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 12));
	pr_debug("[EM] AUXADC_Channel_12_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_12_Offset(struct device *dev, struct device_attribute *attr,
					      const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_12_Offset, 0664, show_AUXADC_Channel_12_Offset,
		   store_AUXADC_Channel_12_Offset);
#endif


#if ADC_CHANNEL_MAX > 13
static ssize_t show_AUXADC_Channel_13_Slope(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 13));
	pr_debug("[EM] AUXADC_Channel_13_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_13_Slope(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_13_Slope, 0664, show_AUXADC_Channel_13_Slope,
		   store_AUXADC_Channel_13_Slope);
static ssize_t show_AUXADC_Channel_13_Offset(struct device *dev, struct device_attribute *attr,
					     char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 13));
	pr_debug("[EM] AUXADC_Channel_13_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_13_Offset(struct device *dev, struct device_attribute *attr,
					      const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_13_Offset, 0664, show_AUXADC_Channel_13_Offset,
		   store_AUXADC_Channel_13_Offset);
#endif


#if ADC_CHANNEL_MAX > 14
static ssize_t show_AUXADC_Channel_14_Slope(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 14));
	pr_debug("[EM] AUXADC_Channel_14_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_14_Slope(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_14_Slope, 0664, show_AUXADC_Channel_14_Slope,
		   store_AUXADC_Channel_14_Slope);
static ssize_t show_AUXADC_Channel_14_Offset(struct device *dev, struct device_attribute *attr,
					     char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 14));
	pr_debug("[EM] AUXADC_Channel_14_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_14_Offset(struct device *dev, struct device_attribute *attr,
					      const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_14_Offset, 0664, show_AUXADC_Channel_14_Offset,
		   store_AUXADC_Channel_14_Offset);
#endif


#if ADC_CHANNEL_MAX > 15
static ssize_t show_AUXADC_Channel_15_Slope(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_slop + 15));
	pr_debug("[EM] AUXADC_Channel_15_Slope : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_15_Slope(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_15_Slope, 0664, show_AUXADC_Channel_15_Slope,
		   store_AUXADC_Channel_15_Slope);
static ssize_t show_AUXADC_Channel_15_Offset(struct device *dev, struct device_attribute *attr,
					     char *buf)
{
	int ret_value = 1;
	ret_value = (*(auxadc_cali_offset + 15));
	pr_debug("[EM] AUXADC_Channel_15_Offset : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_15_Offset(struct device *dev, struct device_attribute *attr,
					      const char *buf, size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_15_Offset, 0664, show_AUXADC_Channel_15_Offset,
		   store_AUXADC_Channel_15_Offset);
#endif

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Create File For EM : AUXADC_Channel_Is_Calibration */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_AUXADC_Channel_Is_Calibration(struct device *dev, struct device_attribute *attr,
						  char *buf)
{
	int ret_value = 2;
	ret_value = g_AUXADC_Cali;
	pr_debug("[EM] AUXADC_Channel_Is_Calibration : %d\n", ret_value);
	return sprintf(buf, "%u\n", ret_value);
}

static ssize_t store_AUXADC_Channel_Is_Calibration(struct device *dev,
						   struct device_attribute *attr, const char *buf,
						   size_t size)
{
	pr_debug("[EM] Not Support Write Function\n");
	return size;
}

static DEVICE_ATTR(AUXADC_Channel_Is_Calibration, 0664, show_AUXADC_Channel_Is_Calibration,
		   store_AUXADC_Channel_Is_Calibration);

static ssize_t show_AUXADC_register(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (buf == NULL) {
		pr_debug("[%s] Invalid input!!\n", __func__);
		return 0;
	}
	
	return mt_auxadc_dump_register(buf);
}

static ssize_t store_AUXADC_register(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	pr_debug("[EM] Not Support store_AUXADC_register\n");
	return size;
}

static DEVICE_ATTR(AUXADC_register, 0664, show_AUXADC_register, store_AUXADC_register);


static ssize_t show_AUXADC_chanel(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* read data */
	int i = 0, data[4] = { 0, 0, 0, 0 };
	char buf_temp[400];
	int res = 0;

	if (buf == NULL) {
		pr_debug("[%s] Invalid input!!\n", __func__);
		return 0;
	}
	
	for (i = 0; i < 5; i++) {
		res = IMM_auxadc_GetOneChannelValue(i, data, NULL);
		if (res < 0) {
			pr_debug("[adc_driver]: get data error\n");
			break;

		} else {
			pr_debug("[adc_driver]: channel[%d]=%d.%d \n", i, data[0], data[1]);
			sprintf(buf_temp, "channel[%d]=%d.%d \n", i, data[0], data[1]);
			strcat(buf, buf_temp);
		}

	}
	mt_auxadc_dump_register(buf_temp);
	strcat(buf, buf_temp);

	return strlen(buf);
}

static int dbug_thread(void *unused)
{
	int i = 0, data[4] = { 0, 0, 0, 0 };
	int res = 0;
	int rawdata = 0;
	int cali_voltage = 0;

	while (g_start_debug_thread) {
		for (i = 0; i < ADC_CHANNEL_MAX; i++) {
			res = IMM_auxadc_GetOneChannelValue(i, data, &rawdata);
			if (res < 0) {
				pr_debug("[adc_driver]: get data error\n");
				break;

			} else {
				pr_debug("[adc_driver]: channel[%d]raw =%d\n", i, rawdata);
				pr_debug("[adc_driver]: channel[%d]=%d.%.02d\n", i, data[0],
					 data[1]);

			}
			res = IMM_auxadc_GetOneChannelValue_Cali(i, &cali_voltage);
			if (res < 0) {
				pr_debug("[adc_driver]: get cali voltage error\n");
				break;

			} else {
				pr_debug("[adc_driver]: channel[%d] cali_voltage =%d\n", i,
					 cali_voltage);

			}
			msleep(500);

		}
		msleep(500);

	}
	return 0;
}


static ssize_t store_AUXADC_channel(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int start_flag = 0;
	int error = 0;
	int ret = 0;

	if (buf == NULL) {
		pr_debug("[%s] Invalid input!!\n", __func__);
		return 0;
	}

	ret = kstrtoint(buf, sizeof(int), &start_flag);
	if (ret < 0) {
		pr_debug("[%s] Invalid invalues!!\n", __func__);
		return 0;
	}

	pr_debug("[adc_driver] start flag =%d\n", start_flag);
	if (start_flag) {
		g_start_debug_thread = start_flag;
		thread = kthread_run(dbug_thread, 0, "AUXADC");

		if (IS_ERR(thread)) {
			error = PTR_ERR(thread);
			pr_debug("[adc_driver] failed to create kernel thread: %d\n", error);
		}
	} else {
		pr_debug("[%s] Invalid input!!\n", __func__);
	}

	return size;
}

static DEVICE_ATTR(AUXADC_read_channel, 0664, show_AUXADC_chanel, store_AUXADC_channel);

static int mt_auxadc_create_device_attr(struct device *dev)
{
	int ret = 0;
	/* For EM */
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_register)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_read_channel)) != 0)
		goto exit;
#if ADC_CHANNEL_MAX > 0
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_0_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_0_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 1
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_1_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_1_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 2
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_2_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_2_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 3
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_3_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_3_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 4
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_4_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_4_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 5
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_5_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_5_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 6
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_6_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_6_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 7
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_7_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_7_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 8
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_8_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_8_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 9
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_9_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_9_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 10
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_10_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_10_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 11
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_11_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_11_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 12
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_12_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_12_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 13
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_13_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_13_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 14
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_14_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_14_Offset)) != 0)
		goto exit;
#endif
#if ADC_CHANNEL_MAX > 15
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_15_Slope)) != 0)
		goto exit;
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_15_Offset)) != 0)
		goto exit;
#endif
	if ((ret = device_create_file(dev, &dev_attr_AUXADC_Channel_Is_Calibration)) != 0)
		goto exit;

	return 0;
exit:
	return 1;
}


static int adc_channel_info_init(void)
{
	unsigned int used_channel_counter = 0;
	used_channel_counter = 0;
#ifdef AUXADC_TEMPERATURE_CHANNEL
	/* ap_domain &= ~(1<<CUST_ADC_MD_CHANNEL); */
	sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_RFTMP");
	g_adc_info[used_channel_counter].channel_number = AUXADC_TEMPERATURE_CHANNEL;
	pr_debug("[ADC] channel_name = %s channel num=%d\n",
		 g_adc_info[used_channel_counter].channel_name,
		 g_adc_info[used_channel_counter].channel_number);
	used_channel_counter++;
#endif

#ifdef AUXADC_TEMPERATURE1_CHANNEL
	sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_APTMP");
	g_adc_info[used_channel_counter].channel_number = AUXADC_TEMPERATURE1_CHANNEL;
	pr_debug("[ADC] channel_name = %s channel num=%d\n",
		 g_adc_info[used_channel_counter].channel_name,
		 g_adc_info[used_channel_counter].channel_number);
	used_channel_counter++;
#endif

#ifdef AUXADC_ADC_FDD_RF_PARAMS_DYNAMIC_CUSTOM_CH_CHANNEL
	sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_FDD_Rf_Params_Dynamic_Custom");
	g_adc_info[used_channel_counter].channel_number =
	    AUXADC_ADC_FDD_RF_PARAMS_DYNAMIC_CUSTOM_CH_CHANNEL;
	pr_debug("[ADC] channel_name = %s channel num=%d\n",
		 g_adc_info[used_channel_counter].channel_name,
		 g_adc_info[used_channel_counter].channel_number);
	used_channel_counter++;
#endif

#ifdef AUXADC_HF_MIC_CHANNEL
	sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_MIC");
	g_adc_info[used_channel_counter].channel_number = AUXADC_HF_MIC_CHANNEL;
	pr_debug("[ADC] channel_name = %s channel num=%d\n",
		 g_adc_info[used_channel_counter].channel_name,
		 g_adc_info[used_channel_counter].channel_number);
	used_channel_counter++;
#endif

	return 0;

}

/* platform_driver API */
static int mt_auxadc_probe(struct platform_device *dev)
{
	int ret = 0;
	struct device *adc_dev = NULL;
	int used_channel_counter = 0;
	int of_value = 0;

	pr_debug("******** MT AUXADC driver probe!! ********\n");

#if !defined(CONFIG_MTK_CLKMGR)
#else
#ifndef CONFIG_MTK_FPGA
	if (clock_is_on(MT_PDN_PERI_AUXADC) == 0) {	/* PWR_DOWN */
		if (enable_clock(MT_PDN_PERI_AUXADC, "AUXADC"))
			pr_debug("hwEnableClock AUXADC failed.");
	}
#endif
#endif

	/* Integrate with NVRAM */
	ret = alloc_chrdev_region(&auxadc_cali_devno, 0, 1, AUXADC_CALI_DEVNAME);
	if (ret)
		pr_debug("[AUXADC_AP]Error: Can't Get Major number for auxadc_cali\n");

	auxadc_cali_cdev = cdev_alloc();
	auxadc_cali_cdev->owner = THIS_MODULE;
	auxadc_cali_cdev->ops = &auxadc_cali_fops;
	ret = cdev_add(auxadc_cali_cdev, auxadc_cali_devno, 1);
	if (ret)
		pr_debug("auxadc_cali Error: cdev_add\n");

	auxadc_cali_major = MAJOR(auxadc_cali_devno);
	auxadc_cali_class = class_create(THIS_MODULE, AUXADC_CALI_DEVNAME);
	adc_dev = (struct device *)device_create(auxadc_cali_class,
						 NULL, auxadc_cali_devno, NULL,
						 AUXADC_CALI_DEVNAME);
	pr_debug("[MT AUXADC_probe] NVRAM prepare : done !!\n");

	/* read calibration data from EFUSE */
	mt_auxadc_hal_init(dev);

	pr_debug("[MT AUXADC_probe2] mt_auxadc_hal_init : done !!\n");


#if !defined(CONFIG_MTK_LEGACY)
#ifdef CONFIG_OF
	pr_debug("[MT AUXADC_probe3] get device tree info : start !!\n");

	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek, AUXADC_CH");
	if (node) {
		ret = of_property_read_u32_array(node, "TEMPERATURE", &of_value, 1);
		if (ret == 0) {
			sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_RFTMP");
			g_adc_info[used_channel_counter].channel_number = of_value;
			pr_debug("[AUXADC_AP] find node TEMPERATURE:%d\n", of_value);
			used_channel_counter++;
		}
		ret = of_property_read_u32_array(node, "TEMPERATURE1", &of_value, 1);
		if (ret == 0) {
			sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_APTMP");
			g_adc_info[used_channel_counter].channel_number = of_value;
			pr_debug("[AUXADC_AP] find node TEMPERATURE1:%d\n", of_value);
			used_channel_counter++;
		}
		ret =
		    of_property_read_u32_array(node, "ADC_FDD_RF_PARAMS_DYNAMIC_CUSTOM_CH",
					       &of_value, 1);
		if (ret == 0) {
			sprintf(g_adc_info[used_channel_counter].channel_name,
				"ADC_FDD_Rf_Params_Dynamic_Custom");
			g_adc_info[used_channel_counter].channel_number = of_value;
			pr_debug("[AUXADC_AP] find node ADC_FDD_RF_PARAMS_DYNAMIC_CUSTOM_CH:%d\n",
				 of_value);
			used_channel_counter++;
		}
		ret = of_property_read_u32_array(node, "HF_MIC", &of_value, 1);
		if (ret == 0) {
			sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_MIC");
			g_adc_info[used_channel_counter].channel_number = of_value;
			pr_debug("[AUXADC_AP] find node HF_MIC:%d\n", of_value);
			used_channel_counter++;
		}
		ret = of_property_read_u32_array(node, "LCM_VOLTAGE", &of_value, 1);
		if (ret == 0) {
			sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_LCM_VOLTAGE");
			g_adc_info[used_channel_counter].channel_number = of_value;
			pr_debug("[AUXADC_AP] find node LCM_VOLTAGE:%d\n", of_value);
			used_channel_counter++;
		}
		ret = of_property_read_u32_array(node, "BATTERY_VOLTAGE", &of_value, 1);
		if (ret == 0) {
			sprintf(g_adc_info[used_channel_counter].channel_name,
				"ADC_BATTERY_VOLTAGE");
			g_adc_info[used_channel_counter].channel_number = of_value;
			pr_debug("[AUXADC_AP] find node BATTERY_VOLTAGE:%d\n", of_value);
			used_channel_counter++;
		}
		ret = of_property_read_u32_array(node, "CHARGER_VOLTAGE", &of_value, 1);
		if (ret == 0) {
			sprintf(g_adc_info[used_channel_counter].channel_name,
				"ADC_CHARGER_VOLTAGE");
			g_adc_info[used_channel_counter].channel_number = of_value;
			pr_debug("[AUXADC_AP] find node CHARGER_VOLTAGE:%d\n", of_value);
			used_channel_counter++;
		}
		ret = of_property_read_u32_array(node, "UTMS", &of_value, 1);
		if (ret == 0) {
			sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_UTMS");
			g_adc_info[used_channel_counter].channel_number = of_value;
			pr_debug("[AUXADC_AP] find node UTMS:%d\n", of_value);
			used_channel_counter++;
		}
		ret = of_property_read_u32_array(node, "REF_CURRENT", &of_value, 1);
		if (ret == 0) {
			sprintf(g_adc_info[used_channel_counter].channel_name, "ADC_REF_CURRENT");
			g_adc_info[used_channel_counter].channel_number = of_value;
			pr_debug("[AUXADC_AP] find node REF_CURRENT:%d\n", of_value);
			used_channel_counter++;
		}
	} else {
		pr_debug("[AUXADC_AP] find node failed\n");
	}
#endif
#else
	adc_channel_info_init();
	pr_debug("[MT AUXADC_AP] adc_channel_info_init : done !!\n");

#endif

#if !defined(CONFIG_MTK_CLKMGR)
#ifdef CONFIG_OF
	clk_auxadc = devm_clk_get(&dev->dev, "auxadc-main");
	if (!clk_auxadc) {
		pr_debug("[AUXADC] devm_clk_get failed\n");
		return -1;
	}
	pr_debug("[AUXADC]: auxadc CLK:0x%p\n", clk_auxadc);
	ret = clk_prepare_enable(clk_auxadc);
	if (ret) {
		pr_debug("hwEnableClock AUXADC failed.");
	}
#endif
#else
#endif

	g_adc_init_flag = 1;

	if (mt_auxadc_create_device_attr(adc_dev))
		goto exit;
exit:
	return ret;
}

static int mt_auxadc_remove(struct platform_device *dev)
{
	pr_debug("******** MT auxadc driver remove!! ********\n");
	return 0;
}

static void mt_auxadc_shutdown(struct platform_device *dev)
{
	pr_debug("******** MT auxadc driver shutdown!! ********\n");
}

static int mt_auxadc_suspend(struct platform_device *dev, pm_message_t state)
{
	/* pr_debug("******** MT auxadc driver suspend!! ********\n" ); */
	mt_auxadc_hal_suspend();
	return 0;
}

static int mt_auxadc_resume(struct platform_device *dev)
{
	/* pr_debug("******** MT auxadc driver resume!! ********\n" ); */
	mt_auxadc_hal_resume();
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_auxadc_of_match[] = {
	{.compatible = "mediatek,AUXADC",},
	{},
};
#endif

static struct platform_driver mt_auxadc_driver = {
	.probe = mt_auxadc_probe,
	.remove = mt_auxadc_remove,
	.shutdown = mt_auxadc_shutdown,
#ifdef CONFIG_PM
	.suspend = mt_auxadc_suspend,
	.resume = mt_auxadc_resume,
#endif
	.driver = {
		   .name = "mt-auxadc",
#ifdef CONFIG_OF
		   .of_match_table = mt_auxadc_of_match,
#endif
		   },
};

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2015.12.5 for hw version check
enum{
	HW_VERSION__UNKNOWN,
	HW_VERSION__T0, 	//1200mV
	HW_VERSION__EVT, 	//1030 mV
	HW_VERSION__FDD, 	//820 mV
	HW_VERSION__DVT, 	//680 mV
	HW_VERSION__PVT, 	//540mV
}; 


int pcb_hw_version = HW_VERSION__UNKNOWN;

 static int hw_version_check()
{
	int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;
	int times = 5,channel = 14;
	
	//pr_err("hw_version_check\n");

	if (IMM_IsAdcInitReady() == 0) {
		pr_err("[hw_version_check]: AUXADC is not ready\n");
		return 0;
	}

	 i = times;
	 while (i--) {
		 ret_value = IMM_GetOneChannelValue(channel, data, &ret_temp);
 		//pr_err("hw_version_check---------------ret_value = %d\n",ret_temp);
		 if (ret_value == 0) {
			 ret += ret_temp;
		 } else {
			 times = times > 1 ? times - 1 : 1;
			 pr_debug("[hw_version_check] ret_value=%d, times=%d\n",
				  ret_value, times);
		 }
	 }

	ret = ret * 1500 / 4096;
	ret = ret / times;
	if(ret >= 1250)
		pcb_hw_version = HW_VERSION__UNKNOWN;
	else if(ret >= 1100)
		pcb_hw_version = HW_VERSION__T0;
	else if(ret >= 900)
		pcb_hw_version = HW_VERSION__EVT;
	else if(ret >= 720)
		pcb_hw_version = HW_VERSION__FDD;
	else if(ret >= 590)
		pcb_hw_version = HW_VERSION__DVT;
	else
		pcb_hw_version = HW_VERSION__PVT;
	pr_err("kong---hw_version_check,ret =  %d, pcb_hw_version = %d\n",ret,pcb_hw_version);
	return ret;
}
#endif/*VENDOR_EDIT*/

static int __init mt_auxadc_init(void)
{
	int ret;

#if !defined(CONFIG_MTK_CLKMGR)
#else
#ifndef CONFIG_MTK_FPGA
	if (enable_clock(MT_PDN_PERI_AUXADC, "AUXADC"))
		pr_debug("hwEnableClock AUXADC failed.");
#endif
#endif

	ret = platform_driver_register(&mt_auxadc_driver);
	if (ret) {
		pr_debug("****[mt_auxadc_driver] Unable to register driver (%d)\n", ret);
		return ret;
	}
	pr_debug("****[mt_auxadc_driver] Initialization : DONE\n");
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2015.12.5 for hw version check
	hw_version_check();
#endif/*VENDOR_EDIT*/
	return 0;
}

static void __exit mt_auxadc_exit(void)
{
}

module_init(mt_auxadc_init);
module_exit(mt_auxadc_exit);

MODULE_AUTHOR("MTK");
MODULE_DESCRIPTION("MTK AUXADC Device Driver");
MODULE_LICENSE("GPL");
