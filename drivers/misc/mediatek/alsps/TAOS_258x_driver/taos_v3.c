/*******************************************************************************
 *                                                                              *
 *       File Name:      taos_v3.c (Version 3.5)                                *
 *       Description:    Linux device driver for Taos Ambient Light Sensor      *
 *                                                                              *
 *       Author:         TAOS Inc.                                              *
 *       								       *
 ********************************************************************************
 *       Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074        *
 *******************************************************************************/
/**
 * Copyright (C) 2011, Texas Advanced Optoelectronic Solutions (R).
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/wait.h>

/* Includes */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/string.h>
#include <linux/kmod.h>
#include <linux/ctype.h>
#include <linux/input.h>
#include "taos_v3.h"
#include <alsps.h>

#include <cust_alsps.h>


/**
 * definitions/declarations used in this code
 * u8 = unsigned char (8 bit)
 * s8 = signed   char (8 bit)
 * unsigned int= unsigned int  (16 bit)
 * s16= signed   int  (16 bit) - aka int
 */
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_DEBUG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_DEBUG APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_DEBUG fmt, ##args) 

//....................................................................................................
#define DRIVER_VERSION_ID	"3.6"

#define TAOS258x		2

//This build will be for..
#define DRIVER_TYPE		TAOS258x

//....................................................................................................
//Debug related
#define TAOS_DEBUG			1
//#define TAOS_DEBUG_SMBUS		1	//prints the data written or read from a smbus r/w
#define TAOS_IRQ_DEBUG			1	//prints special messages from interrupt BHs console
//....................................................................................................
//User notification method
//#define ASYNC_NOTIFY			1	//Interrupt generated application notification via fasync_
#define EVENT_NOTIFY			1	//Interrupt generated input_event update in /dev/input/event_
//....................................................................................................
//Module defines
/* Device name/id/address */
#define DEVICE_NAME			"taos"
#ifdef TAOS258x
#define DEVICE_ID			"skateFN"
#else
#define DEVICE_ID			"tritonFN"
#endif

#define ID_NAME_SIZE			10
#define DEVICE_ADDR1			0x29
#define DEVICE_ADDR2			0x39
#define DEVICE_ADDR3			0x49
#define MAX_NUM_DEVICES			3
#define MAX_DEVICE_REGS			32
#define I2C_MAX_ADAPTERS		8

/* Triton register offsets */
#ifdef TAOS258x
#define	TAOS_REG_MAX	8
#else
#define	TAOS_REG_MAX	16
#endif

/* Power management */
#define taos_suspend			NULL
#define taos_shutdown			NULL
#define taos_resume			NULL

/* Operational mode */
//#define POLLED_MODE			1
//#define INTERRUPT_MODE			2

#define	MAXI2C	32
#define	CMD_ADDRESS	0x80
#define	TAOS_ERROR	-1
#define TAOS_SUCCESS	0

#define	MAX_SAMPLES_CAL	200
#define GAIN_SWITCH_LEVEL 100

/* Prototypes */

extern struct i2c_adapter *i2c_get_adapter(int id);
static struct alsps_init_info taos_init_info;

static int taos_probe(struct i2c_client *client,const struct i2c_device_id *idp);

static int taos_remove(struct i2c_client *client);
static int taos_open(struct inode *inode, struct file *file);
static int taos_release(struct inode *inode, struct file *file);
static long taos_ioctl(struct file *file, unsigned int cmd,unsigned long arg);
//static int taos_lux_filter(int raw_lux);
static int taos_device_name(unsigned char *bufp, char **device_name);
static int taos_als_calibrate(unsigned long als_cal_target);
int taos_parse_config_file(char *x);
int taos_chip_on(void);
void taos_defaults(void);

//....................................................................................................
// Various /misc. emunerations
typedef enum
    {
    TAOS_CHIP_UNKNOWN = 0,
    TAOS_CHIP_WORKING = 1,
    TAOS_CHIP_SLEEP = 2
    } TAOS_CHIP_WORKING_STATUS;

//....................................................................................................

char driver_version_id[] = DRIVER_VERSION_ID;

/* Module parameter */
//static char *mode = "default";
//module_param(mode, charp, S_IRUGO);
//MODULE_PARM_DESC(mode, "Mode of operation - polled or interrupt");

//------------------------------------------------------------------------------------------------------------------


/* Module device table */
static const struct i2c_device_id taos_idtable[] ={{DEVICE_ID, 0},{}};

/* Board and address info */
static struct i2c_board_info __initdata taos_board_info = {I2C_BOARD_INFO(DEVICE_ID, DEVICE_ADDR3)};

/* Client and device */
//static struct i2c_client *my_client;
static struct device *devp;
static int device_found;

/* Driver definition */
static struct i2c_driver taos_driver ={
	.probe = taos_probe,
	.remove = taos_remove,
	.suspend = taos_suspend,
	.shutdown = taos_shutdown,
	.resume = taos_resume,
    .id_table = taos_idtable,
	.driver = {
		.name = DEVICE_NAME,
	},
};

/* Per-device data */
static struct taos_data{
	struct i2c_client *client;
	struct cdev cdev;
	struct fasync_struct *async_queue;
	unsigned int addr;
	char taos_id;
	char taos_name[ID_NAME_SIZE];
	char valid;
	unsigned long last_updated;
}*taos_datap;

/* File operations */
static struct file_operations taos_fops = {
    .owner = THIS_MODULE,
	.open = taos_open,
	.release = taos_release,
	.unlocked_ioctl = taos_ioctl,
};


static u8 init_done = 0;

// ................ Als info ...................../
struct taos_als_info als_cur_info;
EXPORT_SYMBOL( als_cur_info);

//Next two vars are also used to determine irq type - in addition/lieu status info
volatile bool als_on = 0;

static int device_released = 0;

/* Lux time scale */
struct time_scale_factor{
	unsigned int numerator;
	unsigned int denominator;
	unsigned int saturation;
};

//static int lux_history[FILTER_DEPTH] = {-ENODATA, -ENODATA, -ENODATA};


//Structs & vars
int taos_chip_status = TAOS_CHIP_UNKNOWN; //Unknown = uninitialized to start
int taos_cycle_type; // what is the type of cycle being run: ALS, PROX, or BOTH

struct taos_settings taos_settings;
EXPORT_SYMBOL( taos_settings);

/* Device configuration */
#define MAX_SETTING_MEMBERS	6

//More Prototypes
unsigned long taos_isqrt(unsigned long x);
int taos_register_dump(void);

int taos_i2c_read(u8 reg, u8 *val, unsigned int len);
int taos_i2c_write(u8 reg, u8 *val);
int taos_i2c_write_command(u8 reg);

int taos_parse_lux_file(char *x);
extern int als_only;
static int als_gain_factor = 1000;

//-------------------------- Work Queues used for bottom halves of IRQs -----------------------------

typedef struct
    {
	struct work_struct my_work;
	int x;
    } my_work_t;

my_work_t *als;

//..................................................................................................................
// Initial values for device - this values can/will be changed by driver (via chip_on and other)
// and applications as needed.
// These values are dynamic.

//UNCOMMENT BELOW FOR SKATE
#ifdef TAOS258x
u8 taos_config[8] =
    {
    0x00, 0xee, 0x00, 0x03, 0x00, 0xFF, 0xFF, 0x00
    };
//	cntrl atime intC  Athl0 Athl1 Athh0 Athh1 gain
#else

//UNCOMMENT BELOW FOR TRITON
//u8 taos_config[16] = {
0x00, 0xee, 0xff, 0xf5, 0x10, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x03, 0x30, 0x00, 0x0a, 0x20};
cntrl atime ptime wtime Athl0 Athl1 Athh0 Athh1 Pthl0 Pthl1 Pthh0 Pthh1 Prst pcfg pcnt gain

// initial lux equation table

#endif

#define LUX_TABLE_MIN_RECS 3	//Default minimum number of records in the following table
//Note table updates must always include complete table.


//This structure is intentionally large to accommodate updates via
//ioctl and proc input methods.
struct taos_lux
    {
	unsigned int ratio;
	unsigned int ch0;
	unsigned int ch1;
    } taos_device_lux[] =
    {
		{9830, 8520, 15729},
		{12452, 10807, 23344},
		{14746, 6383, 11705},
		{17695, 4063, 6554},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0}
    };

struct taos_lux taos_lux;
EXPORT_SYMBOL( taos_lux);

int als_time_scale; // computed, ratios lux due to als integration time
int als_saturation; // computed, set to 90% of full scale of als integration
//int	als_thresh_high;	// als threshold (trigger point) - when als >
//int	als_thresh_low;		// als threshold (trigger point) - when als <

#define MAX_GAIN_STAGES	4	//Index = ( 0 - 3) Used to validate the gain selection index
struct gainadj
    {
	s16 ch0;
	s16 ch1;
    } gainadj[] =
    {
		{1, 1},
		{8, 8},
		{16, 16},
		{107, 115}
    };

struct taos_lux *taos_device_luxP;
struct taos_lux *copy_of_taos_device_luxP;

/**
 * Name verification - uses default register values to identify the Taos device
 */
static int taos_device_name(unsigned char *bufp, char **device_name)
    {
    if (((bufp[0x12] & 0xf0) == 0x00) || (bufp[0x12] == 0x08))
	{
	*device_name = "tritonFN";
	return (1);
	}
    else if (((bufp[0x12] & 0xf0) == 0x90))
	{
	*device_name = "skateFN";
	return (1);
	}
    *device_name = "unknown";
    APS_ERR("Identified %s\n",*device_name);
    return (0);
    }

/**
 * Provides initial operational parameter defaults.\n
 * These defaults may be changed by the following:\n
 * - system deamon
 * - user application (via ioctl)
 * - directly writing to the taos procfs file
 * - external kernel level modules / applications
 */
void taos_defaults(void)
    {
    //Operational parameters
    taos_cycle_type = LIGHT; // default is ALS only
    taos_settings.als_time = 100; // must be a multiple of 50mS
    taos_settings.als_gain = 0; // this is actually an index into the gain table
    // assume clear glass as default
    taos_settings.als_gain_trim = 100; // default gain trim to account for aperture effects
    taos_settings.als_persistence = 3; // default number of 'out of bounds' b4 interrupt
    taos_settings.als_cal_target = 130; // Known external ALS reading used for calibration
    taos_settings.interrupts_enabled = 0; // Interrupts enabled (ALS) 0 = none

    //Initialize ALS data to defaults
    als_cur_info.als_ch0 = 0;
    als_cur_info.als_ch1 = 0;
    als_cur_info.lux = 0;

    taos_settings.als_thresh_high = ALS_THRESHOLD_HI_LIMIT;
    taos_settings.als_thresh_low = ALS_THRESHOLD_LO_LIMIT;

#ifdef TAOS_DEBUG
	APS_ERR("\nDEFAULTS LOADED\n");
#endif
    }
//EXPORT_SYMBOL( taos_defaults);
//..................................................................................................................
//@}	//End of Initial


//==================================================================================================================
/**@defgroup IOCTL User ioctl Functions
 * The section applies to 'ioctl' calls used primarily to support user-land applications.\n
 * @{
 */
/**
 * Ioctl functions - each one is documented below.
 */
static long taos_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
    {
    int ret = 0;
    int tmp;
    u8 reg_val;

    if(taos_datap==NULL){
		APS_ERR("taos_datap is null\n");
		return -1;
	}
    switch (cmd)
	{
    /**
     * - ALS_ON - called to set the device in ambient light sense mode.
     * configured values of light integration time, initial interrupt
     * filter, gain, and interrupt thresholds (if interrupt driven) are
     * initialized. Then power, adc, (and interrupt if needed) are enabled.
     */
    case TAOS_IOCTL_ALS_ON:
	taos_cycle_type = LIGHT;
	APS_ERR("ALS On\n");
	return (taos_chip_on());
	break;
	/**
	 * - ALS_OFF - called to stop ambient light sense mode of operation.
	 * Clears the filter history, and clears the control register.
	 */
    case TAOS_IOCTL_ALS_OFF:
	taos_chip_off();
	break;

	/**
	 * - ALS_DATA - request for current ambient light data. If correctly
	 * enabled and valid data is available at the device, the function
	 * for lux conversion is called, and the result returned if valid.
	 */
    case TAOS_IOCTL_ALS_DATA:
	//Are we actively updating the struct?
	if ((taos_settings.interrupts_enabled & CNTL_ALS_INT_ENBL) == 0x00)
	    return (taos_get_lux()); //No - so get fresh data
	else
	    return (als_cur_info.lux); //Yes - data is fresh as last change
	break;

	/**
	 * - ALS_CALIBRATE - called to run one calibration cycle, during assembly.
	 * The lux value under a known intensity light source is used to obtain
	 * a "gain trim" multiplier which is to be used to normalize subsequent
	 * lux data read from the device, after the calibration is completed.
	 * This 'known intensity should be the value of als_cal_target.
	 * If not - make is so!
	 */
    case TAOS_IOCTL_ALS_CALIBRATE:
	return (taos_als_calibrate(taos_settings.als_cal_target));
	break;

	/**
	 * - CONFIG-GET - returns the current device configuration values to the
	 * caller. The user mode application can display or store configuration
	 * values, for future reconfiguration of the device, as needed.
	 * Refer to structure "taos_settings" in '.h' file.
	 */
    case TAOS_IOCTL_CONFIG_GET:
	ret = copy_to_user((struct taos_settings *) arg, &taos_settings,
		sizeof(struct taos_settings));
	if (ret)
	    {
	    dev_err(devp, "copy_to_user() failed in ioctl config_get\n");
	    return (-ENODATA);
	    }
	return (ret);
	break;

	/**
	 * - GET_ALS - returns the current ALS structure values to the
	 * caller. The values returned represent the last call to
	 * taos_get_lux().  This ioctl can be used when the driver
	 * is operating in the interrupt mode.
	 * Refer to structure "als_cur_info" in '.h' file.
	 */
    case TAOS_IOCTL_GET_ALS:
	ret = copy_to_user((struct taos_als_info *) arg, &als_cur_info,
		sizeof(struct taos_als_info));
	if (ret)
	    {
	    dev_err(devp,
		    "copy_to_user() failed in ioctl to get taos_als_info\n");
	    return (-ENODATA);
	    }
	return (ret);
	break;

	/**
	 * - CONFIG-SET - used by a user mode application to set the desired
	 * values in the driver's in memory copy of the device configuration
	 * data. Light integration times are aligned optimally, and driver
	 * global variables dependent on configured values are updated here.
	 * Refer to structure "taos_settings" in '.h' file.
	 */
    case TAOS_IOCTL_CONFIG_SET:
	ret = copy_from_user(&taos_settings, (struct taos_settings *) arg,
		sizeof(struct taos_settings));
	if (ret)
	    {
	    dev_err(devp, "copy_from_user() failed in ioctl "
		"config_set\n");
	    return (-ENODATA);
	    }
	return (ret);
	break;

	/**
	 * - LUX_TABLE-GET - returns the current LUX coefficients table to the
	 * caller. The user mode application can display or store the table
	 * values, for future re-calibration of the device, as needed.
	 * Refer to structure "taos_lux" in '.h' file.
	 */
    case TAOS_IOCTL_LUX_TABLE_GET:
	ret = copy_to_user((struct taos_lux *) arg, &taos_device_lux,
		sizeof(taos_device_lux));
	if (ret)
	    {
	    dev_err(devp,
		    "copy_to_user() failed in ioctl TAOS_IOCTL_LUX_TABLE_GET\n");
	    return (-ENODATA);
	    }
	return (ret);
	break;

	/**
	 * - LUX TABLE-SET - used by a user mode application to set the desired
	 * LUX table values in the driver's in memory copy of the lux table
	 * Refer to structure "taos_lux" in '.h' file.
	 */
    case TAOS_IOCTL_LUX_TABLE_SET:
	ret = copy_from_user(&taos_lux, (struct taos_lux *) arg,
		sizeof(taos_device_lux));
	if (ret)
	    {
	    dev_err(devp, "copy_from_user() failed in ioctl "
		"TAOS_IOCTL_LUX_TABLE_SET\n");
	    return (-ENODATA);
	    }
	return (ret);
	break;

	/**
	 * - TAOS_IOCTL_ID - used to query driver name & version
	 */
    case TAOS_IOCTL_ID:
	ret = copy_to_user((char *) arg, &driver_version_id,
		sizeof(driver_version_id));
	APS_ERR("%s\n",DRIVER_VERSION_ID);
	return (ret);
	break;

	/**
	 * - REGISTER_DUMP - dumps registers to the console device.
	 */
    case TAOS_IOCTL_REG_DUMP:

	taos_register_dump();
	return (ret /*TAOS_SUCCESS*/);
	break;

	/*
	 * - Enable/Disable ALS interrupt
	 *
	 */
    case TAOS_IOCTL_INT_SET:
	get_user(tmp, (int *) arg);
	switch (tmp)
	    {
	case 1: //ALS INTERRUPT - ON
#ifdef TAOS_IRQ_DEBUG
	    APS_ERR("ALS Interrupts on\n");
#endif
	    //Make sure we start off the very 1st interrupt by setting persistence to 0
	    reg_val = 0x00;
	    reg_val |= CNTL_ALS_INT_ENBL; //Set up for Level interrupt
	    //and write it
	    if ((ret = taos_i2c_write((CMD_REG | (INTERRUPT)), &reg_val)) < 0)
		{
			APS_ERR("FAILED taos_i2c_write to update the persistance register.\n");
		}

	    taos_settings.interrupts_enabled = CNTL_ALS_INT_ENBL;
	    break;

	    case 0: //ALS - OFF
	    taos_settings.interrupts_enabled = 0x00;
	    reg_val = 0x01; //Set up for Level interrupt
	    //and write it
	    if ((ret = taos_i2c_write((CMD_REG |(INTERRUPT)), &reg_val))< 0)
		{
		APS_ERR("FAILED: taos_i2c_write to update the Interrupts OFF  register.\n");
		}
	    taos_chip_off();
	    break;
	    }
	return (TAOS_SUCCESS);
	break;

	/**
	 * - TAOS_IOCTL_SET_GAIN - used to set/change the device analog gain.
	 * The value passed into here from the user is the index into the table.
	 * Index = ( 0 - 3) Used to validate the gain selection index
	 */
	case TAOS_IOCTL_SET_GAIN:
	get_user(tmp,(int *)arg);
	if (tmp > MAX_GAIN_STAGES)
	return(-1);

	taos_settings.als_gain = tmp;
	if ((ret = taos_i2c_write(CMD_REG|GAIN, (u8 *)&tmp))< 0)
	    {
	    APS_ERR("taos_i2c_write to turn on device failed in taos_chip_on.\n");
	    return (-1);
	    }
	return (ret);
	break;

	default:
	return (-EINVAL);
	break;

	}

    return (ret);

    }
//.............................................................................................................
//@}	//end of IOCTL section


//==================================================================================================================
/**@defgroup ALS Ambient Light Sense (ALS)
 * The section applies to the ALS related functions.\n
 * Other ALS releated functions may appear elsewhere in the code.\n
 * @{
 */

//..................................................................................................................

/**
 * Reads and calculates current lux value.
 * The raw ch0 and ch1 values of the ambient light sensed in the last
 * integration cycle are read from the device.
 * Time scale factor array values are adjusted based on the integration time.
 * The raw values are multiplied by a scale factor, and device gain is obtained
 * using gain index. Limit checks are done next, then the ratio of a multiple
 * of ch1 value, to the ch0 value, is calculated. The array taos_device_luxP[]
 * declared above is then scanned to find the first ratio value that is just
 * above the ratio we just calculated. The ch0 and ch1 multiplier constants in
 * the array are then used along with the time scale factor array values, to
 * calculate the lux.
 *
 *	\param 	none
 *	\return int	-1 = Failure, Lux Value = success
 */
int taos_get_lux(void)
    {
    u32 ch0, ch1; /* separated ch0/ch1 data from device */
    s32 lux; /* raw lux calculated from device data */
//    u32 ratio;
    u8 buf[5];
	s64 lux1,lux2;
	u8 tmp;
    int i, ret;

    if (taos_chip_status != TAOS_CHIP_WORKING)
	{
	// device is not enabled
	APS_ERR("device is not enabled\n");
	return (-1);
	}

    if ((taos_cycle_type & LIGHT) == 0)
	{
	// device not in ALS mode
	APS_ERR("device not in ALS mode\n");
	return (-1);
	}

    if ((ret = taos_i2c_read((CMD_REG), &buf[0], 1)) < 0)
	{
	APS_ERR("taos_i2c_read() to CMD_REG reg failed in taos_get_lux()\n");
	return -1;
	}
    /* is data new & valid */
    if (!(buf[0] & STA_ADCINTR))
	{
	APS_ERR("Data not valid, so return LAST VALUE\n");
	return (als_cur_info.lux); // have no data, so return LAST VALUE
	}

    for (i = 0; i < 4; i++)
	{
	if ((ret = taos_i2c_read((CMD_REG | (ALS_CHAN0LO + i)), &buf[i], 1))
		< 0)
	    {
	    APS_ERR("taos_i2c_read() to (CMD_REG |(ALS_CHAN0LO + i) regs failed in taos_get_lux()\n");
	    return -1;
	    }
	}

    /* clear status, really interrupt status (interrupts are off, but we use the bit anyway */
    if ((ret = taos_i2c_write_command(CMD_REG | CMD_SPL_FN | CMD_ALS_INTCLR))
	    < 0)
	{
	APS_ERR("taos_i2c_write_command failed in taos_chip_on, err = %d\n", ret);
	return -1; // have no data, so return fail
	}

    /* extract ALS/lux data */
    ch0 = (buf[1] << 8) | buf[0];
    ch1 = (buf[3] << 8) | buf[2];
#ifdef TAOS_DEBUG
    APS_ERR(" ch0=%d/0x%x ch1=%d/0x%x\n",ch0,ch0,ch1,ch1);
#endif
//0x00=1 0x01=8 0x02=16 0x03=107

	if(taos_settings.als_gain==0x02 && ch0>=als_saturation){
		tmp = 0x00;
	}else if(taos_settings.als_gain==0x02 && ch0<GAIN_SWITCH_LEVEL){
		tmp = 0x03;
	}else if(taos_settings.als_gain==0x03 && ch0>als_saturation-GAIN_SWITCH_LEVEL){
		tmp = 0x02;
	}else if(taos_settings.als_gain==0x00 && ch0<GAIN_SWITCH_LEVEL){
		tmp = 0x02;
	}else{
		tmp = taos_settings.als_gain;
	}
	if(taos_settings.als_gain != tmp){
		if ((ret = taos_i2c_write(CMD_REG|GAIN, (u8 *)&tmp))< 0){
			APS_ERR("set gain fail\n");
			return -1;
		}
		msleep(300);
		taos_settings.als_gain = tmp;
		for (i = 0; i < 4; i++){
			if ((ret = taos_i2c_read((CMD_REG | (ALS_CHAN0LO + i)), &buf[i], 1))< 0){
				APS_ERR("taos_i2c_read() to (CMD_REG |(ALS_CHAN0LO + i) regs failed in taos_get_lux()\n");
				return -1;
			}
		}
		ch0 = (buf[1] << 8) | buf[0];
		ch1 = (buf[3] << 8) | buf[2];
	}
    als_cur_info.als_ch0 = ch0;
    als_cur_info.als_ch1 = ch1;

    if ((ch0 >= als_saturation) || (ch1 >= als_saturation)){
		goto return_max;
	}

    if (ch0 == 0){
		als_cur_info.lux = 0;
#ifdef TAOS_DEBUG
		APS_ERR("ch0==0\n");
#endif
		return (als_cur_info.lux); // have no data, so return LAST VALUE
	}
    if(ch1*100<ch0*34){
        lux1=(S64)(956*((S64)ch0*1000-(1592*(S64)ch1)))/(taos_settings.als_time * gainadj[taos_settings.als_gain].ch0)/1000;
        lux2 = (S64)(956 * ((2232 * (S64)ch0) - (6049 * (S64)ch1)) )/ (taos_settings.als_time * gainadj[taos_settings.als_gain].ch0)/1000;
        if((lux1<0)||(lux2<0)){
            goto cal2;
        }
        lux = lux1<lux2?lux1:lux2;
    }else{
cal2:
        lux1=(S64)(1665*((S64)ch0*1000-(2179*(S64)ch1)))/(taos_settings.als_time * gainadj[taos_settings.als_gain].ch0)/1000;
        lux2 = (S64)(1665 * ((300 * (S64)ch0) - (535 * (S64)ch1)) )/ (taos_settings.als_time * gainadj[taos_settings.als_gain].ch0)/1000;
        lux1 = lux1<0?0:lux1;
        lux2 = lux2<0?0:lux2;
        lux = lux1<lux2?lux2:lux1;
    }

	APS_ERR("ch0=%d,ch1=%d,als_time=%d,index=%d,als_gain=%d,lux1 = %lld,lux2 = %lld,lux=%d\n",ch0,ch1,taos_settings.als_time,taos_settings.als_gain,gainadj[taos_settings.als_gain].ch0,lux1,lux2,lux);
	lux = lux*100/138;
    APS_ERR("final lux=%d\n",lux);

    if (lux > MAX_LUX) /* check for overflow */
		goto return_max;

    als_cur_info.lux = lux; //Update the structure with the latest VALID lux.
    return (lux);
return_max: 
	als_cur_info.lux = MAX_LUX;
	return (als_cur_info.lux);
}
EXPORT_SYMBOL( taos_get_lux);

/**
 * Obtain single reading and calculate the als_gain_trim (later used to derive actual lux)
 *\param   ulong	als_cal_target	ALS target (real world)
 *\return  int		updated gain_trim value
 */
int taos_als_calibrate(unsigned long als_cal_target)
    {
    u8 reg_val;
    unsigned int gain_trim_val;
    int ret;
    int lux_val;

    //This may seem redundant but..
    //We need this next line in case we call this function from an external application who has
    //a different target value than our defaults.
    //In which case, make 'that' our new default/settings.
    taos_settings.als_cal_target = als_cal_target;

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (CMD_REG | CNTRL))))
	    < 0)
	{
	dev_err(devp,
		"i2c_smbus_write_byte to cmd reg failed in ioctl als_calibrate\n");
	return (ret);
	}
    reg_val = i2c_smbus_read_byte(taos_datap->client);

    if ((reg_val & (CNTL_ADC_ENBL | CNTL_PWRON))
	    != (CNTL_ADC_ENBL | CNTL_PWRON))
	{
	dev_err(
		devp,
		"reg_val & (CNTL_ADC_ENBL | CNTL_PWRON)) != (CNTL_ADC_ENBL | CNTL_PWRON) in ioctl als_calibrate\n");
	return (-ENODATA);
	}

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (CMD_REG | STATUS))))
	    < 0)
	{
	dev_err(devp,
		"i2c_smbus_write_byte to cmd reg failed in ioctl als_calibrate\n");
	return (ret);
	}
    reg_val = i2c_smbus_read_byte(taos_datap->client);

    if ((reg_val & STA_ADCVALID) != STA_ADCVALID)
	{
	dev_err(devp,
		"(reg_val & STA_ADCVALID) != STA_ADCVALID in ioctl als_calibrate\n");
	return (-ENODATA);
	}

    if ((lux_val = taos_get_lux()) < 0)
	{
	dev_err(
		devp,
		"taos_get_lux() returned bad value %d in ioctl als_calibrate\n",
		lux_val);
	return (lux_val);
	}
    gain_trim_val = (unsigned int) (((taos_settings.als_cal_target)
	    * taos_settings.als_gain_trim) / lux_val);

    APS_ERR("\n\ntaos_settings.als_cal_target = %d\ntaos_settings.als_gain_trim = %d\nlux_val = %d\n",
	    taos_settings.als_cal_target,taos_settings.als_gain_trim,lux_val);

    if ((gain_trim_val < 25) || (gain_trim_val > 400))
	{
	dev_err(
		devp,
		"ALS calibrate result %d out of range in ioctl als_calibrate\n",
		gain_trim_val);
	return (-ENODATA);
	}
    taos_settings.als_gain_trim = (int) gain_trim_val;

    return ((int) gain_trim_val);
    }

EXPORT_SYMBOL( taos_als_calibrate);

/**
 * Lux filter function - retains history of polled values, and returns the most
 * appropriate value of the three.
 */
/*static int taos_lux_filter(int raw_lux)
    {
    static u8 middle[] =
	{
	1, 0, 2, 0, 0, 2, 0, 1
	};
    int index = 0;

    lux_history[2] = lux_history[1];
    lux_history[1] = lux_history[0];
    lux_history[0] = raw_lux;
    if ((lux_history[2] < 0) || (lux_history[1] < 0) || (lux_history[0] < 0))
	return (-ENODATA);
    if (lux_history[0] > lux_history[1])
	index += 4;
    if (lux_history[1] > lux_history[2])
	index += 2;
    if (lux_history[0] > lux_history[2])
	index++;
    return (lux_history[middle[index]]);
    }
*/
//..................................................................................................................
// @} End of ALS section

//==================================================================================================================
/**@defgroup DEV_CTRL Device Control Functions
 * @{
 */

//..................................................................................................................
/**
 * Turn the device on.
 * Configuration and taos_cycle_type must be set before calling this function.
 * \param 	none.
 * \return 	int	0 = success, < 0 = failure
 */
int taos_chip_on(void)
    {
    int i;
    int ret = 0;
    u8 *uP;
    u8 utmp;
    int als_count;
    int als_time;
	APS_ERR("start\n");
    //make sure taos_cycle_type is set.
    if ((taos_cycle_type != LIGHT))
	return (-1);

    //and make sure we're not already on
    if (taos_chip_status == TAOS_CHIP_WORKING)
	{
	// if forcing a register update - turn off, then on
	APS_ERR("device is already enabled\n");
	return (-1);
	}

    //. . . . . . . . . . . . . . . SHADOW REGISTER INITIALIZATION . . . . . . . . . . .
    //Note:
    //If interrupts are enabled, keeping persist at 0 should prime the pump.
    //So don't set persist in shadow register. Just enable if required.
    taos_config[INTERRUPT] = (taos_settings.interrupts_enabled & 0xF0);

    // determine als integration regster
    als_count = (taos_settings.als_time * 100 + 135) / 270;
    if (als_count == 0)
	als_count = 1; // ensure at least one cycle

    //LIGHT
    // convert back to time (encompasses overrides)
    if (taos_cycle_type & LIGHT)
	{
	als_time = (als_count * 27 + 5) / 10;
	taos_config[ALS_TIME] = 256 - als_count;
	}
    else
	taos_config[ALS_TIME] = 0xff;

    //Set the gain based on taos_settings struct
    taos_config[GAIN] = taos_settings.als_gain;

    // set globals re scaling and saturation
    als_saturation = als_count * 922; // 90% of full scale
    als_time_scale = als_time / 50;

    //. . . . . . . . . . . . . . . . . . . . . . . . . . .  . . . . .  . . . . . . . . . . .
    //SKATE Specific power-on / adc enable sequence
    // Power on the device 1st.
    utmp = CNTL_PWRON;
    //
    //
    if ((ret = taos_i2c_write(CMD_REG | CNTRL, &utmp)) < 0)
	{
	APS_ERR("taos_i2c_write to turn on device failed in taos_chip_on.\n");
	return (-1);
	}

    //User the following shadow copy for our delay before enabling ADC
    //Write ALL THE REGISTERS
    for (i = 0, uP = taos_config; i < TAOS_REG_MAX; i++)
	{
	if ((ret = taos_i2c_write(CMD_REG + i, uP++)) < 0)
	    {
	    APS_ERR("taos_i2c_write to reg %d failed in taos_chip_on.\n",i);
	    return (-1);
	    }
	}


    //NOW enable the ADC
    // initialize the desired mode of operation
    utmp = CNTL_PWRON | CNTL_ADC_ENBL;
    //
    //
    if ((ret = taos_i2c_write(CMD_REG | CNTRL, &utmp)) < 0)
	{
	APS_ERR("taos_i2c_write to turn on device failed in taos_chip_on.\n");
	return (-1);
	}



    taos_chip_status = TAOS_CHIP_WORKING;

#ifdef TAOS_DEBUG
    APS_ERR("chip_on() called\n\n");
#endif
	APS_ERR("end\n");
    return (ret); // returns result of last i2cwrite
    }
//EXPORT_SYMBOL( taos_chip_on);

//..................................................................................................................
/**
 * Turn the device OFF.
 * \param	none.
 * \return	int	0 = success, < 0 = failure
 */
int taos_chip_off()
    {
    int ret;
    u8 utmp;

    // turn device off
    taos_chip_status = TAOS_CHIP_SLEEP;
    utmp = 0x00;
    ret = taos_i2c_write(CMD_REG | CNTRL, &utmp);
    als_on = FALSE;
    init_done = 0x00; //used by ALS irq as one/first shot
#ifdef TAOS_DEBUG
    APS_ERR("chip_off() called\n");
#endif

    return (ret);

    }
EXPORT_SYMBOL( taos_chip_off);
//..................................................................................................................
//@} End of Device Control Section


//==================================================================================================================

//==================================================================================================================
/**@defgroup I2C-HAL I2C/SMBus Communication Functions
 * The sections pertains to the driver/device communication functions\n
 * facilitated via SMBus in an I2C manner.\n
 * These functions represent several layers of abstraction.
 * Also note, some i2c controllers do not support block read/write.
 * @{
 */

//  Abstraction layer calls
/**
 * Read a number of bytes starting at register (reg) location.
 *
 * \param 	unsigned char 	reg - device register
 * \param 	unsigned char	*val - location to store results
 * \param 	unsigned int 	number of bytes to read
 * \return	TAOS_SUCCESS, or i2c_smbus_write_byte ERROR code
 *
 */
int taos_i2c_read(u8 reg, u8 *val, unsigned int len)
    {
    int result;
    int i;

    if (len > MAXI2C)
	len = MAXI2C;
    for (i = 0; i < len; i++)
	{
	/* select register to write */
	if (reg >= 0)
	    {
	    if ((result = (i2c_smbus_write_byte(taos_datap->client, (CMD_REG
		    | (reg + i))))) < 0)
		{
		dev_err(devp,
			"FAILED: i2c_smbus_write_byte_data in taos_io_read()\n");
		return (result);
		}
	    /* read the data */
	    *val = i2c_smbus_read_byte(taos_datap->client);
#ifdef TAOS_DEBUG_SMBUS
	    APS_ERR("READ FROM REGISTER [%02X] -->%02x<--\n",(CMD_REG | (reg + i)) ,*data);
#endif
	    val++;
	    }
	}
    return (TAOS_SUCCESS);
    }

/**
 * This function is used to send a an register address followed by a data value
 * When calling this function remember to set the register MSBit (if applicable).
 * \param  unsigned char register
 * \param  unsigned char * pointer to data value(s) to be written
 * \return TAOS_SUCCESS, or i2c_smbus_write_byte error code.
 */
int taos_i2c_write(u8 reg, u8 *val)
    {
    int retval;
    if ((retval = (i2c_smbus_write_byte_data(taos_datap->client, reg, (*val))))
	    < 0)
	{
	dev_err(devp, "FAILED: i2c_smbus_write_byte_data\n");
	return (retval);
	}

    return (TAOS_SUCCESS);
    }

/**
 * This function is used to send a command to device command/control register
 * All bytes sent using this command have their MSBit set - it's a command!
 * \param  unsigned char register to write
 * \return TAOS_SUCCESS, or i2c_smbus_write_byte error code.
 */
int taos_i2c_write_command(u8 reg)
    {
    int result;
    u8 *p;
    u8 buf;

    p = &buf;
    *p = reg |= CMD_REG;
    // write the data
    if ((result = (i2c_smbus_write_byte(taos_datap->client, (*p)))) < 0)
	{
	dev_err(devp, "FAILED: i2c_smbus_write_byte\n");
	return (result);
	}
    return (TAOS_SUCCESS);
    }

//..................................................................................................................
//@}	//end of I2C-HAL section


//==================================================================================================================
/**@defgroup group5 Ancillary Functions
 * The following functions are for use within the context of of the driver module.\n
 * These functions are not intended to called externally.  These functions are\n
 * not exported.\n
 * @{
 */

/**
 * Integer Square Root
 * We need an integer version since 1st Floating point is not allowed in driver world, 2nd, cannot
 * count on the devices having a FPU, and 3rd software FP emulation may be excessive.
 */
unsigned long taos_isqrt(unsigned long x)
    {
    register unsigned long op, res, one;
    op = x;
    res = 0;

    // "one" starts at the highest power of four <= than the argument.
    one = 1 << 30; // second-to-top bit set
    while (one > op)
	one >>= 2;

    while (one != 0)
	{
	if (op >= res + one)
	    {
	    op -= res + one;
	    res += one << 1;
	    }
	res >>= 1;
	one >>= 2;
	}
    return res;
    }

/**
 * Simple dump of device registers to console via printk.
 */
int taos_register_dump(void)
    {
    int i = 0;
    int ret = 0;
    u8 chdata[0x26];

    for (i = 0; i < 0x20; i++)
	{
	if ((ret = taos_i2c_read((CMD_REG | (CNTRL + i)), &chdata[i], 1)) < 0)
	    {
	    APS_ERR("i2c_smbus_read_byte() failed in taos_register_dump()\n"); //dev_err(devp, "");
	    return (-1);
	    }
	}

    APS_ERR("** taos register dump *** \n");
    for (i = 0; i < 0x24; i += 2)
	{
		printk(KERN_INFO "(0x%02x)0x%02x, (0x%02x)0x%02x", i, chdata[i], (i+1), chdata[(i+1)]);
	}
    APS_ERR("\n");
    return 0;
    }

//@}	//End of ancillary documentation - gp5
//==================================================================================================================

/**@defgroup group6 Sysfs Interface Functions
 * The following functions are for access via the sysfs style interface.\n
 * Use 'cat' to read, 'echo >' to write\n
 * @{
 */

//..................................................................................................................
// sysfs interface functions begin here

static ssize_t taos_device_id(struct device_driver *ddri,
	 char *buf)
    {
    return sprintf(buf, "%s %s\n", DEVICE_ID, DRIVER_VERSION_ID);
    }

static ssize_t taos_power_state_show(struct device_driver *ddri,
	 char *buf)
    {
    return sprintf(buf, "%d\n", taos_chip_status);
    return 0;
    }

static ssize_t taos_power_state_store(struct device_driver *ddri,
	 const char *buf, size_t len)
    {
    unsigned long value;

    if (strict_strtoul(buf, 0, &value))
	{
	return -EINVAL;
	}
    if (value == 1)
	{
	taos_chip_on();
	}
    else
	{
	taos_chip_off();
	}
    return len;
    }

static ssize_t taos_gain_idx_show(struct device_driver *ddri,char *buf){
		return sprintf(buf, "%d\n", taos_settings.als_gain);
    }

static ssize_t taos_gain_idx_store(struct device_driver *ddri,
	 const char *buf, size_t len)
    {
    unsigned long value;
    if (strict_strtoul(buf, 0, &value))
	{
	return -EINVAL;
	}
    if (value)
	{
	if (value > 4)
	    {
	    APS_ERR("Invalid Gain Index\n");
	    return (-1);
	    }
	else
	    {
	    taos_settings.als_gain = value;
	    }
	}
    return len;
    }

static ssize_t taos_als_time_show(struct device_driver *ddri,
	 char *buf)
    {
		return sprintf(buf, "%d\n", taos_settings.als_time);
    }

static ssize_t taos_als_time_store(struct device_driver *ddri,
	 const char *buf, size_t len)
    {
    unsigned long value;
    if (strict_strtoul(buf, 0, &value))
	{
	return -EINVAL;
	}
    if (value)
	{
	taos_settings.als_time = value;
	}
    return len;
    }

static ssize_t taos_als_trim_show(struct device_driver *ddri,
	 char *buf)
    {
    return sprintf(buf, "%d\n", taos_settings.als_gain_trim);
    return 0;
    }

static ssize_t taos_als_trim_store(struct device_driver *ddri,
	 const char *buf, size_t len)
    {
    unsigned long value;
    if (strict_strtoul(buf, 0, &value))
	{
	return -EINVAL;
	}
    if (value)
	{
	taos_settings.als_gain_trim = value;
	}
    return len;
    }

static ssize_t taos_als_persistence_show(struct device_driver *ddri,
	 char *buf)
    {
    return sprintf(buf, "%d\n", taos_settings.als_persistence);
    return 0;
    }

static ssize_t taos_als_persistence_store(struct device_driver *ddri,
	 const char *buf, size_t len)
    {
    unsigned long value;
    if (strict_strtoul(buf, 0, &value))
	{
	return -EINVAL;
	}
    if (value)
	{
	taos_settings.als_persistence = value;
	}
    return len;
    }

static ssize_t taos_als_cal_target_show(struct device_driver *ddri,
	 char *buf)
    {
    return sprintf(buf, "%d\n", taos_settings.als_cal_target);
    return 0;
    }

static ssize_t taos_als_cal_target_store(struct device_driver *ddri,
	 const char *buf, size_t len)
    {
    unsigned long value;
    if (strict_strtoul(buf, 0, &value))
	{
	return -EINVAL;
	}
    if (value)
	{
	taos_settings.als_cal_target = value;
	}
    return len;
    }

static ssize_t taos_lux_show(struct device_driver *ddri, 
	char *buf)
    {
    int lux = 0;
    if ((taos_settings.interrupts_enabled & CNTL_ALS_INT_ENBL) == 0x00)
	lux = taos_get_lux(); //get fresh data
    else
	lux = als_cur_info.lux; //data is fresh as last change

    return sprintf(buf, "%d\n", lux);
    return 0;
    }

static ssize_t taos_do_calibrate(struct device_driver *ddri,
	 const char *buf, size_t len)
    {
    unsigned long value;
    if (strict_strtoul(buf, 0, &value))
	{
	return -EINVAL;
	}
    if (value == 1)
	{
	taos_als_calibrate(taos_settings.als_cal_target);
	}
    return len;
    }

static ssize_t taos_als_thresh_low_show(struct device_driver *ddri,
	 char *buf)
    {
    return sprintf(buf, "%d\n", taos_settings.als_thresh_low);
    return 0;
    }

static ssize_t taos_als_thresh_low_store(struct device_driver *ddri,
	 const char *buf, size_t len)
    {
    unsigned long value;
    if (strict_strtoul(buf, 0, &value))
	{
	return -EINVAL;
	}
    if (value)
	{
	taos_settings.als_thresh_low = value;
	}
    return len;
    }

static ssize_t taos_als_thresh_high_show(struct device_driver *ddri,
	 char *buf)
    {
    return sprintf(buf, "%d\n", taos_settings.als_thresh_high);
    }

static ssize_t taos_als_thresh_high_store(struct device_driver *ddri,const char *buf, size_t len)
{
    unsigned long value;
    if (strict_strtoul(buf, 0, &value))
	{
	return -EINVAL;
	}
    if (value)
	{
	taos_settings.als_thresh_high = value;
	}
    return len;
}

static ssize_t taos_chipinfo_show(struct device_driver *ddri,char *buf)
{
    return sprintf(buf, "TSL2583\n");
}

static ssize_t taos_ch0_show(struct device_driver *ddri,char *buf)
{
    return sprintf(buf, "%d\n",als_cur_info.als_ch0);
}

static ssize_t taos_ch1_show(struct device_driver *ddri,char *buf)
{
    return sprintf(buf, "%d\n",als_cur_info.als_ch1);
}

static ssize_t taos_als_gain_ch0_show(struct device_driver *ddri,char *buf)
{
    return sprintf(buf, "%d\n",gainadj[taos_settings.als_gain].ch0);
}

static ssize_t taos_ch0_degain_show(struct device_driver *ddri,char *buf)
{
    return sprintf(buf, "%d\n",als_cur_info.als_ch0/gainadj[taos_settings.als_gain].ch0);
}

static ssize_t taos_als_show(struct device_driver *ddri,char *buf)
{
    return sprintf(buf, "%d\n",als_cur_info.lux);
}

static ssize_t taos_show_gain(struct device_driver *ddri, char *buf)
{
	return sprintf(buf, "%d\n", als_gain_factor);
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t taos_store_gain(struct device_driver *ddri, const char *buf, size_t count)
{
	if(1 != sscanf(buf, "%d", &als_gain_factor)){
		APS_ERR("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;
}
//sysfs - interface functions
static DRIVER_ATTR( als_device_id, S_IRUGO, taos_device_id, NULL);
static DRIVER_ATTR(als_device_state, S_IRUGO | S_IWUSR,taos_power_state_show,taos_power_state_store);
static DRIVER_ATTR(als_gain_idx, S_IRUGO | S_IWUSR,taos_gain_idx_show,taos_gain_idx_store);
static DRIVER_ATTR(als_time, S_IRUGO | S_IWUSR,taos_als_time_show,taos_als_time_store);
static DRIVER_ATTR(als_trim, S_IRUGO | S_IWUSR,taos_als_trim_show,taos_als_trim_store);
static DRIVER_ATTR(als_persistence, S_IRUGO | S_IWUSR,taos_als_persistence_show,taos_als_persistence_store);
static DRIVER_ATTR(als_target, S_IRUGO | S_IWUSR,taos_als_cal_target_show,taos_als_cal_target_store);
static DRIVER_ATTR( als_lux, S_IRUGO, taos_lux_show, NULL);
static DRIVER_ATTR( als_calibrate, S_IWUSR, NULL, taos_do_calibrate);
static DRIVER_ATTR(als_lowT, S_IRUGO | S_IWUSR,taos_als_thresh_low_show,taos_als_thresh_low_store);
static DRIVER_ATTR(als_highT, S_IRUGO | S_IWUSR,taos_als_thresh_high_show,taos_als_thresh_high_store);

static DRIVER_ATTR( als_chipinfo, S_IRUGO, taos_chipinfo_show, NULL);
static DRIVER_ATTR( als_ch0, S_IRUGO, taos_ch0_show, NULL);
static DRIVER_ATTR( als_ch1, S_IRUGO, taos_ch1_show, NULL);
static DRIVER_ATTR( als_gain_ch0, S_IRUGO, taos_als_gain_ch0_show, NULL);
static DRIVER_ATTR( als_ch0_degain, S_IRUGO, taos_ch0_degain_show, NULL);
static DRIVER_ATTR( als, S_IRUGO, taos_als_show, NULL);
static DRIVER_ATTR( gain_als, S_IWUSR | S_IRUGO, taos_show_gain,  taos_store_gain);

static struct driver_attribute *taos_attrs_list[] = {
	    &driver_attr_als_device_id,
	    &driver_attr_als_device_state,
	    &driver_attr_als_gain_idx,
	    &driver_attr_als_time,
	    &driver_attr_als_trim,
	    &driver_attr_als_persistence,
	    &driver_attr_als_target,
	    &driver_attr_als_lux,
	    &driver_attr_als_calibrate,
	    &driver_attr_als_lowT,
	    &driver_attr_als_highT,
		&driver_attr_als_chipinfo,
		&driver_attr_als_ch0,
		&driver_attr_als_ch1,
		&driver_attr_als_gain_ch0,
		&driver_attr_als_ch0_degain,
		&driver_attr_als,
		&driver_attr_gain_als,
};
static int taos_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(taos_attrs_list)/sizeof(taos_attrs_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, taos_attrs_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", taos_attrs_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static int taos_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(taos_attrs_list)/sizeof(taos_attrs_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, taos_attrs_list[idx]);
	}
	
	return err;
}

// Interrupt pin declaration - must be changed to suit H/W
#define GPIO_PS_ALS_INT_IRQ 	S5PV210_GPH0(4)
#define HW_IRQ_PIN				S5PV210_GPH0(4)
#define HW_IRQ_NO				IRQ_EINT4

//===========================================================================================================
//
//                                          FUNCTIONS BEGIN HERE
//
//===========================================================================================================

//===========================================================================================================
/**@defgroup Initialization Driver Initialization
 * The sections applies to functions for driver initialization, instantiation, _exit,\n
 * and user-land application invoking (ie. open).\n
 * Also included in this section is the initial interrupt handler (handler bottom halves are in the respective
 * sections).
 * @{
 */

//-----------------------------------------------------------------------------------------------------------
/** Driver initialization - device probe is initiated here, to identify
 * a valid device if present on any of the i2c buses and at any address.
 * \param  none
 * \return: int (0 = OK)
 * \note  	H/W Interrupt are device/product dependent.  Attention is required to the definition and
 * configuration.
 */
//-----------------------------------------------------------------------------------------------------------
static int taos_init_flag = -1;
static int taos_local_init(void){
	if (i2c_add_driver(&taos_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}

	if (-1 == taos_init_flag) {
		return -1;
	}

	return 0;
}

/**
 * Driver exit
 */
static int taos_uninit(void)
{
    i2c_del_driver(&taos_driver);
    kfree(taos_datap);
	return 0;
}

static int taos_als_open_report_data(int open)
{
	return 0;
}

static int taos_als_enable_nodata(int en)
{
	APS_ERR("%d\n",en);
	if(en){
		taos_chip_on();
	}else{
		taos_chip_off();
	}
	return 0;
}

static int taos_als_set_delay(u64 ns)
{
	return 0;
}

static int taos_calied_lux(int raw_data){
    return als_gain_factor*raw_data/1000;
}

static int taos_als_get_data(int* value, int* status)
{
	int als=-1;
	als = taos_get_lux();
	als = taos_calied_lux(als);
	if(als>=0){
		*value = als;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}else{
		return -1;
	}
	return 0;
}
static struct miscdevice taos_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "als",
	.fops  = &taos_fops,
};

/**
 * Client probe function - When a valid device is found, the driver's device
 * data structure is updated, and initialization completes successfully.
 */
static int taos_probe(struct i2c_client *client,const struct i2c_device_id *idp)
{
    int i, ret = 0;
    unsigned char buf[MAX_DEVICE_REGS];
    char *device_name;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	u8 reg;

	taos_datap = kzalloc(sizeof(struct taos_data), GFP_KERNEL);
    if (!taos_datap){
		APS_ERR("kmalloc for struct taos_data fail\n");
		ret = -ENOMEM;
		return -1;
	}
    taos_datap->client = client;
    i2c_set_clientdata(client, taos_datap);
    for (i = 0; i < MAX_DEVICE_REGS; i++){
		reg = CMD_REG|(CNTRL+i);
		if ((ret = (i2c_smbus_write_byte(client, reg)))< 0){
			APS_ERR("write to cmd reg %02x failed, err = %d\n",reg,ret);
			goto exit_init_failed;
		}
		buf[i] = i2c_smbus_read_byte(client);
	}
    if ((ret = taos_device_name(buf, &device_name)) == 0){
		APS_ERR("i2c device not found\n");
		goto exit_init_failed;
	}
    if (strcmp(device_name, DEVICE_ID)){
		APS_ERR("i2c device not found 2\n");
		goto exit_init_failed;
	}else{
		APS_ERR("i2c device found id of %s\n", device_name);
		device_found = 1;
	}
	taos_defaults();
    if ((ret = (i2c_smbus_write_byte(client, (CMD_REG | CNTRL)))) < 0){
		APS_ERR("write to cmd reg failed, err = %d\n", ret);
		goto exit_init_failed;
	}
    strlcpy(client->name, DEVICE_ID, I2C_NAME_SIZE);
    strlcpy(taos_datap->taos_name, DEVICE_ID, ID_NAME_SIZE);
    taos_datap->valid = 0;

	if ((ret = misc_register(&taos_miscdevice))) {
		APS_ERR("taos_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((ret = taos_create_attr(&taos_init_info.platform_diver_addr->driver)))		
	{
		APS_ERR("create attribute err = %d\n", ret);
		goto exit_create_attr_failed;
	}

	als_ctl.open_report_data= taos_als_open_report_data;
	als_ctl.enable_nodata = taos_als_enable_nodata;
	als_ctl.set_delay  = taos_als_set_delay;
	als_ctl.is_report_input_direct = false;
    als_ctl.is_support_batch = false;

	
	ret = als_register_control_path(&als_ctl);
	if(ret)
	{
		APS_ERR("register fail = %d\n", ret);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = taos_als_get_data;
	als_data.vender_div = 100;
	ret = als_register_data_path(&als_data);	
	if(ret)
	{
		APS_ERR("tregister fail = %d\n", ret);
		goto exit_sensor_obj_attach_fail;
	}
	taos_init_flag = 0;
	als_only = 1;
	register_device_proc("Sensor_als", "tsl258x", "AMS");
    return 0;
exit_sensor_obj_attach_fail:
	taos_delete_attr(&taos_init_info.platform_diver_addr->driver);
exit_create_attr_failed:
	misc_deregister(&taos_miscdevice);
exit_misc_device_register_failed:
exit_init_failed:
	kfree(taos_datap);
	APS_ERR("%s: err = %d\n", __func__, ret);
	taos_init_flag = -1;
	return -1;
}

/**
 * Client remove 
 */
static int taos_remove(struct i2c_client *client)
    {
    return (0);
    }

/**
 * Device open function
 */
static int taos_open(struct inode *inode, struct file *file)
    {
    int ret = 0;
    struct taos_data *taos_datap;

    taos_datap = container_of(inode->i_cdev, struct taos_data, cdev);
    if (strcmp(taos_datap->taos_name, DEVICE_ID) != 0)
	{
	dev_err(devp, "device name error in taos_open(), shows %s\n",
		taos_datap->taos_name);
	return (-ENODEV);
	}

    device_released = 0;
    return (ret);
    }


/**
 * Device release
 */
static int taos_release(struct inode *inode, struct file *file)
{
    struct taos_data *taos_datap;

    device_released = 1;
	taos_datap = NULL;
	return 0;
}



//@}	//End of Sysfs Interface Functions - gp6
//==================================================================================================================
static struct alsps_init_info taos_init_info = {
	.name   = DEVICE_NAME,
	.init   = taos_local_init,
	.uninit = taos_uninit,
};
static int __init taos_init(void)
{
	//APS_FUN();
	struct alsps_hw *hw = get_cust_alsps_hw();
	//taos_ps_adjust_para_real=hw->p_ps_adjust_para;
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &taos_board_info, 1);
	alsps_driver_add(&taos_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit taos_exit(void)
{
	APS_FUN();
}

MODULE_AUTHOR("Jon Brenner<jbrenner@taosinc.com>");
MODULE_DESCRIPTION("TAOS 258x ambient light sensor driver");
MODULE_LICENSE("GPL");

module_init( taos_init);
module_exit( taos_exit);

