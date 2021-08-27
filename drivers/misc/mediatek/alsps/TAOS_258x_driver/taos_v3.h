/*******************************************************************************
*									       *
*	File Name:	taos_skate.h (version 3.5)					       *
*	Description:	Common file for ioctl and configuration definitions.   *
*			Used by kernel driver and driver access applications.  *
*			for the Taos ALS device driver.		       *
*	Author:         TAOS Inc.                                              *
*									       *
********************************************************************************
*	Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074	       *
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

#ifndef TAOS_SKATE_H
#define TAOS_SKATE_H


/* power control */
#define ON              1
#define OFF		0


/* sensor type */
#define LIGHT           1
#define PROXIMITY	2
#define ALL		3

struct taos_als_info {
	u16 als_ch0;
	u16 als_ch1;
	u16 lux;
};
extern struct taos_als_info taos_als_info;

struct taos_settings {
	int	als_time;				// user set; specifies als integration time in mS (50,100)
	int	als_gain;				// user set; device gain setting (0,1,2,3)
	int	als_gain_trim;			// user set, lux gain ratio to adjust for aperture (100% = unity gain)
	int	als_cal_target; 		// Known external ALS reading used for calibration
	int	interrupts_enabled;		// Interrupts enabled (ALS/PROX) 0 = none
	int als_thresh_low;			// ALS interrupt LOW threshold
	int als_thresh_high;		// ALS interrupt HIGH threshold
	char als_persistence;		// Number of integration periods 'out of range' before interrupt.
};
extern struct taos_settings taos_settings;

//-------------------- Functions that can be accessed from User (application) space --------------------------------
/* ioctl numbers */
#define TAOS_IOCTL_MAGIC		0XCF
#define TAOS_IOCTL_ALS_ON		_IO(TAOS_IOCTL_MAGIC,  1)
#define TAOS_IOCTL_ALS_OFF		_IO(TAOS_IOCTL_MAGIC,  2)
#define TAOS_IOCTL_ALS_DATA		_IOR(TAOS_IOCTL_MAGIC, 3, short)
#define TAOS_IOCTL_ALS_CALIBRATE	_IO(TAOS_IOCTL_MAGIC,  4)
#define TAOS_IOCTL_CONFIG_GET		_IOR(TAOS_IOCTL_MAGIC, 5, struct taos_settings)
#define TAOS_IOCTL_CONFIG_SET		_IOW(TAOS_IOCTL_MAGIC, 6, struct taos_settings)
#define TAOS_IOCTL_LUX_TABLE_GET	_IOR(TAOS_IOCTL_MAGIC, 7, struct taos_lux)
#define TAOS_IOCTL_LUX_TABLE_SET	_IOW(TAOS_IOCTL_MAGIC, 8, struct taos_lux)
#define TAOS_IOCTL_ID			_IOR(TAOS_IOCTL_MAGIC, 12, char*)
#define TAOS_IOCTL_REG_DUMP		_IO(TAOS_IOCTL_MAGIC,  14)
#define TAOS_IOCTL_SET_GAIN		_IOW(TAOS_IOCTL_MAGIC, 15,int)
#define TAOS_IOCTL_GET_ALS		_IOR(TAOS_IOCTL_MAGIC, 16, struct taos_als_info)
#define TAOS_IOCTL_INT_SET		_IOW(TAOS_IOCTL_MAGIC, 17,int)

//----------------------------- Functions that can be accessed directly from within Kernel space --------------------------------
void taos_defaults(void);				//Loads the default operational parameters into the "taos_setting"
							//structure.
int taos_chip_on(void);					//Starts the device in the pre-selected mode of operation
int taos_chip_off(void);				//Stops the device mode of operation
int taos_get_lux(void);					//Returns current Lux reading

//========================================================================================================
// Device Registers and various Masks
#define CNTRL				0x00
#define STATUS				0x00
#define ALS_TIME			0X01
#define INTERRUPT			0x02
#define ALS_MINTHRESHLO			0X03
#define ALS_MINTHRESHHI			0X04
#define ALS_MAXTHRESHLO			0X05
#define ALS_MAXTHRESHHI			0X06
#define GAIN				0x07
#define REVID				0x11
#define CHIPID				0x12
#define SMB_4				0x13
#define ALS_CHAN0LO			0x14
#define ALS_CHAN0HI			0x15
#define ALS_CHAN1LO			0x16
#define ALS_CHAN1HI			0x17
#define TMR_LO				0x18
#define TMR_HI				0x19

/* Skate cmd reg masks */
#define CMD_REG				0x80
#define CMD_BYTE_RW			0x00
#define CMD_WORD_BLK_RW			0x20
#define CMD_SPL_FN			0x60
#define CMD_ALS_INTCLR			0X01

/* Skate cntrl reg masks */
#define CNTL_REG_CLEAR			0x00
#define CNTL_ALS_INT_ENBL		0x10
#define CNTL_WAIT_TMR_ENBL		0x08
#define CNTL_ADC_ENBL			0x02
#define CNTL_PWRON			0x01
#define CNTL_ALSPON_ENBL		0x03
#define CNTL_INTALSPON_ENBL		0x13

/* Skate status reg masks */
#define STA_ADCVALID			0x01
#define STA_ALSINTR			0x10
#define STA_ADCINTR			0x10

/* Lux constants */
#define	MAX_LUX				65535
#define FILTER_DEPTH			3

/* Thresholds */
#define ALS_THRESHOLD_LO_LIMIT		0x0010
#define ALS_THRESHOLD_HI_LIMIT		0xFFFF

/* Device default configuration */
#define ALS_TIME_PARAM			100
#define SCALE_FACTOR_PARAM		1
#define GAIN_TRIM_PARAM			512
#define GAIN_PARAM			1
#define ALS_THRSH_HI_PARAM		0xFFFF
#define ALS_THRSH_LO_PARAM		0
//========================================================================================================


#endif //TAOS_SKATE_H
