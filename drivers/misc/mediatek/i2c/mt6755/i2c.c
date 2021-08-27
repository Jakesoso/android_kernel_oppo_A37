#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/aee.h>
#ifdef CONFIG_OF
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_i2c.h>
#endif
#include <asm/scatterlist.h>
#include <asm/io.h>
/* #include <mach/dma.h> */
#include <mach/mt_reg_base.h>
#include <mach/sync_write.h>
#include <mach/mt_cpufreq_hybrid.h>
#include <mach/memory.h>
#include "mt_i2c.h"
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include "cust_gpio_usage.h"
#endif
#ifdef CONFIG_MTK_CLKMGR
#include <mach/mt_clkmgr.h>
#endif

/* define ONLY_KERNEL */
/******************************internal API********************************************************/
void i2c_writel(struct mt_i2c_t *i2c, U8 offset, U16 value)
{
	/* __raw_writew(value, (i2c->base) + (offset)); */
	mt_reg_sync_writel(value, (i2c->base) + (offset));
}

U32 i2c_readl(struct mt_i2c_t *i2c, U8 offset)
{
	return __raw_readl((void *)((i2c->base) + (offset)));
}

/***********************************declare  API**************************/
static void mt_i2c_clock_enable(struct mt_i2c_t *i2c);
static void mt_i2c_clock_disable(struct mt_i2c_t *i2c);

/***********************************I2C common Param **************************/
/* volatile U32 I2C_TIMING_REG_BACKUP[7]={0}; */
/* volatile U32 I2C_HIGHSP_REG_BACKUP[7]={0}; */
static struct i2c_dma_info g_dma_data[7];

#ifdef CONFIG_OF
static void __iomem *ap_dma_base;
#endif
/********************************I2C Param only used in kernel*****************/
/*this field is only for 3d camera*/
#ifdef I2C_DRIVER_IN_KERNEL
static struct i2c_msg g_msg[2];
static struct mt_i2c_t *g_i2c[2];
#endif
/***********************************i2c debug**********************************/
/* #define I2C_DEBUG_FS */
#ifdef I2C_DEBUG_FS
#define PORT_COUNT 7
#define MESSAGE_COUNT 16
#define I2C_T_DMA 1
#define I2C_T_TRANSFERFLOW 2
#define I2C_T_SPEED 3
/*7 ports,16 types of message */
U8 i2c_port[PORT_COUNT][MESSAGE_COUNT];
#if 0
#define I2CINFO(type, format, arg...) \
do { \
	if (type < MESSAGE_COUNT && type >= 0) { \
		if (i2c_port[i2c->id][0] != 0 && (i2c_port[i2c->id][type] != 0 \
						  || i2c_port[i2c->id][MESSAGE_COUNT - 1] != 0)) { \
			I2CLOG(format, ##arg); \
		} \
	} \
} while (0)
#endif
#define I2CINFO(type, format, arg...) I2CLOG(format, ##arg)

#ifdef I2C_DRIVER_IN_KERNEL
static ssize_t show_config(struct device *dev, struct device_attribute *attr, char *buff)
{
	S32 i = 0;
	S32 j = 0;
	char *buf = buff;

	for (i = 0; i < PORT_COUNT; i++) {
		for (j = 0; j < MESSAGE_COUNT; j++)
			i2c_port[i][j] += '0';
		strncpy(buf, (char *)i2c_port[i], MESSAGE_COUNT);
		buf += MESSAGE_COUNT;
		*buf = '\n';
		buf++;
		for (j = 0; j < MESSAGE_COUNT; j++)
			i2c_port[i][j] -= '0';
	}
	return buf - buff;
}

static ssize_t set_config(struct device *dev, struct device_attribute *attr, const char *buf,
			  size_t count)
{
	S32 port, type, status;

	if (sscanf(buf, "%d %d %d", &port, &type, &status) == 3) {
		if (port >= PORT_COUNT || port < 0 || type >= MESSAGE_COUNT || type < 0) {
			/*Invalid param */
			I2CERR("i2c debug system: Parameter overflowed!\n");
		} else {
			if (status != 0)
				i2c_port[port][type] = 1;
			else
				i2c_port[port][type] = 0;

			I2CLOG("port:%d type:%d status:%s\ni2c debug system: Parameter accepted!\n",
			       port, type, status ? "on" : "off");
		}
	} else {
		/*parameter invalid */
		I2CERR("i2c debug system: Parameter invalid!\n");
	}
	return count;
}

static DEVICE_ATTR(debug, 0600, show_config, set_config);
#endif
#else
#define I2CINFO(type, format, arg...)
#endif
/***********************************common API********************************************************/
#if !defined(CONFIG_MTK_PMIC_CHIP_MT6353)
static int i2c_get_semaphore(struct mt_i2c_t *i2c)
{
	if (i2c->id != 3)
		return 0;

	if (cpuhvfs_get_dvfsp_semaphore(SEMA_I2C_DRV) != 0) {
		I2CERR("sema time out 2ms\n");
		if (cpuhvfs_get_dvfsp_semaphore(SEMA_I2C_DRV) != 0) {
			I2CERR("sema time out 4ms\n");
			BUG_ON(1);
			return -EBUSY;
		}
	}

	return 0;
}

static int i2c_release_semaphore(struct mt_i2c_t *i2c)
{
	if (i2c->id != 3)
		return 0;

	cpuhvfs_release_dvfsp_semaphore(SEMA_I2C_DRV);

	return 0;
}
#endif

/*Set i2c port speed*/
static S32 i2c_set_speed(struct mt_i2c_t *i2c)
{
	S32 ret = 0;
	static S32 mode;
	static U32 khz;
	static U8 last_id;
	/* U32 base = i2c->base; */
	U16 step_cnt_div = 0;
	U16 sample_cnt_div = 0;
	U32 tmp, sclk, hclk = i2c->clk;
	U16 max_step_cnt_div = 0;
	U32 diff, min_diff = i2c->clk;
	U16 sample_div = MAX_SAMPLE_CNT_DIV;
	U16 step_div = 0;
	/* I2CFUC(); */
	/* I2CLOG("i2c_set_speed=================\n"); */
	/* compare the current speed with the latest mode */
	if ((mode == i2c->mode) && (khz == i2c->speed)) {
		if (i2c->id == last_id) {	/* same controller */
			I2CINFO(I2C_T_SPEED, " same controller still set sclk to %ldkhz\n",
				i2c->speed);
			ret = 0;
			goto end;
		}
	}
	mode = i2c->mode;
	khz = i2c->speed;

	max_step_cnt_div = (mode == HS_MODE) ? MAX_HS_STEP_CNT_DIV : MAX_STEP_CNT_DIV;
	step_div = max_step_cnt_div;

	if ((mode == FS_MODE && khz > MAX_FS_MODE_SPEED)
	    || (mode == HS_MODE && khz > MAX_HS_MODE_SPEED)) {
		I2CERR(" the speed is too fast for this mode.\n");
		I2C_BUG_ON((mode == FS_MODE && khz > MAX_FS_MODE_SPEED)
			   || (mode == HS_MODE && khz > MAX_HS_MODE_SPEED));
		ret = -EINVAL_I2C;
		goto end;
	}
/* I2CERR("first:khz=%d,mode=%d sclk=%d,min_diff=%d,max_step_cnt_div=%d\n",khz,mode,sclk,min_diff,max_step_cnt_div); */
	/*Find the best combination */
	for (sample_cnt_div = 1; sample_cnt_div <= MAX_SAMPLE_CNT_DIV; sample_cnt_div++) {
		for (step_cnt_div = 1; step_cnt_div <= max_step_cnt_div; step_cnt_div++) {
			sclk = (hclk >> 1) / (sample_cnt_div * step_cnt_div);
			if (sclk > khz)
				continue;
			diff = khz - sclk;
			if (diff < min_diff) {
				min_diff = diff;
				sample_div = sample_cnt_div;
				step_div = step_cnt_div;
			}
		}
	}
	sample_cnt_div = sample_div;
	step_cnt_div = step_div;
	sclk = hclk / (2 * sample_cnt_div * step_cnt_div);
	/* I2CERR("second:sclk=%d khz=%d,i2c->speed=%d hclk=%d sample_cnt_div=%d,step_cnt_div=%d\n",
			sclk,khz,i2c->speed,hclk,sample_cnt_div,step_cnt_div); */
	if (sclk > khz) {
		I2CERR("%s mode: unsupported speed (%dkhz)\n", (mode == HS_MODE) ? "HS" : "ST/FT",
		       khz);
		I2CLOG
		    ("i2c->clk=%d,sclk=%d khz=%d,i2c->speed=%d hclk=%d sample_cnt_div=%d,step_cnt_div=%d.\n",
		     i2c->clk, sclk, khz, i2c->speed, hclk, sample_cnt_div, step_cnt_div);
		I2C_BUG_ON(sclk > khz);
		ret = -ENOTSUPP_I2C;
		goto end;
	}

	step_cnt_div--;
	sample_cnt_div--;

	/* spin_lock(&i2c->lock); */

	if (mode == HS_MODE) {

		/*Set the hignspeed timing control register */
		tmp = i2c_readl(i2c, OFFSET_TIMING) & ~((0x7 << 8) | (0x3f << 0));
		tmp = (0 & 0x7) << 8 | (16 & 0x3f) << 0 | tmp;
		i2c->timing_reg = tmp;
		if (0 == i2c->timing_reg) {
			I2CLOG("hs base address 0x%p,tmp =0x%x\n", i2c->base, tmp);
			/* aee_kernel_warning(TAG, "@%s():%d,\n", __func__, __LINE__); */
		}
		/* i2c_writel(i2c, OFFSET_TIMING, tmp); */
		/* I2C_TIMING_REG_BACKUP[i2c->id]=tmp; */

		/*Set the hign speed mode register */
		tmp = i2c_readl(i2c, OFFSET_HS) & ~((0x7 << 12) | (0x7 << 8));
		tmp = (sample_cnt_div & 0x7) << 12 | (step_cnt_div & 0x7) << 8 | tmp;
		/*Enable the hign speed transaction */
		tmp |= 0x0001;
		i2c->high_speed_reg = tmp;
		/* I2C_HIGHSP_REG_BACKUP[i2c->id]=tmp; */
		/* i2c_writel(i2c, OFFSET_HS, tmp); */
	} else {
		/*Set non-highspeed timing */
		tmp = i2c_readl(i2c, OFFSET_TIMING) & ~((0x7 << 8) | (0x3f << 0));
		tmp = (sample_cnt_div & 0x7) << 8 | (step_cnt_div & 0x3f) << 0 | tmp;
		i2c->timing_reg = tmp;
		if (0 == i2c->timing_reg) {
			I2CLOG("n-hs base address 0x%p, tmp=0x%x\n", i2c->base, tmp);
			/* aee_kernel_warning(TAG, "@%s():%d,\n", __func__, __LINE__); */
		}
		/* I2C_TIMING_REG_BACKUP[i2c->id]=tmp; */
		/* i2c_writel(i2c, OFFSET_TIMING, tmp); */
		/*Disable the high speed transaction */
		/* I2CERR("NOT HS_MODE============================1\n"); */
		tmp = i2c_readl(i2c, OFFSET_HS) & ~(0x0001);
		/* I2CERR("NOT HS_MODE============================2\n"); */
		i2c->high_speed_reg = tmp;
		/* I2C_HIGHSP_REG_BACKUP[i2c->id]=tmp; */
		/* i2c_writel(i2c, OFFSET_HS, tmp); */
		/* I2CERR("NOT HS_MODE============================3\n"); */
	}
	/* spin_unlock(&i2c->lock); */
	I2CINFO(I2C_T_SPEED, " set sclk to %ldkhz(orig:%ldkhz), sample=%d,step=%d\n", sclk, khz,
		sample_cnt_div, step_cnt_div);
end:
	last_id = i2c->id;
	return ret;
}

void _i2c_dump_info(struct mt_i2c_t *i2c)
{
	/* I2CFUC(); */
	I2CLOG("I2C structure:\n"
	       I2CTAG "Clk=%d,Id=%d,Speed mode=%x,St_rs=%x,Dma_en=%x,Op=%x,Poll_en=%x,Irq_stat=%x\n"
	       I2CTAG "Trans_len=%x,Trans_num=%x,Trans_auxlen=%x,Data_size=%x,speed=%d\n"
	       I2CTAG "Trans_stop=%u,Trans_comp=%u,Trans_error=%u\n",
	       i2c->clk, i2c->id, i2c->mode, i2c->st_rs, i2c->dma_en, i2c->op, i2c->poll_en,
	       i2c->irq_stat, i2c->trans_data.trans_len, i2c->trans_data.trans_num,
	       i2c->trans_data.trans_auxlen, i2c->trans_data.data_size, i2c->speed,
	       atomic_read(&i2c->trans_stop), atomic_read(&i2c->trans_comp),
	       atomic_read(&i2c->trans_err));

	I2CLOG("base address 0x%p\n", i2c->base);
	I2CLOG("I2C register:\n"
	       I2CTAG "SLAVE_ADDR=%x,INTR_MASK=%x,INTR_STAT=%x,CONTROL=%x,TRANSFER_LEN=%x\n"
	       I2CTAG "TRANSAC_LEN=%x,DELAY_LEN=%x,TIMING=%x,START=%x,FIFO_STAT=%x\n"
	       I2CTAG "IO_CONFIG=%x,HS=%x,DCM_EN=%x,DEBUGSTAT=%x,EXT_CONF=%x,TRANSFER_LEN_AUX=%x\n",
	       (i2c_readl(i2c, OFFSET_SLAVE_ADDR)),
	       (i2c_readl(i2c, OFFSET_INTR_MASK)),
	       (i2c_readl(i2c, OFFSET_INTR_STAT)),
	       (i2c_readl(i2c, OFFSET_CONTROL)),
	       (i2c_readl(i2c, OFFSET_TRANSFER_LEN)),
	       (i2c_readl(i2c, OFFSET_TRANSAC_LEN)),
	       (i2c_readl(i2c, OFFSET_DELAY_LEN)),
	       (i2c_readl(i2c, OFFSET_TIMING)),
	       (i2c_readl(i2c, OFFSET_START)),
	       (i2c_readl(i2c, OFFSET_FIFO_STAT)),
	       (i2c_readl(i2c, OFFSET_IO_CONFIG)),
	       (i2c_readl(i2c, OFFSET_HS)),
	       (i2c_readl(i2c, OFFSET_DCM_EN)),
	       (i2c_readl(i2c, OFFSET_DEBUGSTAT)),
	       (i2c_readl(i2c, OFFSET_EXT_CONF)), (i2c_readl(i2c, OFFSET_TRANSFER_LEN_AUX)));

	I2CLOG("before enable DMA register(0x%p):\n"
	       I2CTAG "INT_FLAG=%x,INT_EN=%x,EN=%x,RST=%x,\n"
	       I2CTAG "STOP=%x,FLUSH=%x,CON=%x,TX_MEM_ADDR=%x, RX_MEM_ADDR=%x\n"
	       I2CTAG "TX_LEN=%x,RX_LEN=%x,INT_BUF_SIZE=%x,DEBUG_STATUS=%x\n"
	       I2CTAG "ARHP=%x,AWHP=%x\n",
	       g_dma_data[i2c->id].base, g_dma_data[i2c->id].int_flag,
	       g_dma_data[i2c->id].int_en, g_dma_data[i2c->id].en,
	       g_dma_data[i2c->id].rst, g_dma_data[i2c->id].stop,
	       g_dma_data[i2c->id].flush, g_dma_data[i2c->id].con,
	       g_dma_data[i2c->id].tx_mem_addr, g_dma_data[i2c->id].rx_mem_addr,
	       g_dma_data[i2c->id].tx_len, g_dma_data[i2c->id].rx_len,
	       g_dma_data[i2c->id].int_buf_size, g_dma_data[i2c->id].debug_sta,
	       g_dma_data[i2c->id].arhp, g_dma_data[i2c->id].awhp);

	if (i2c->dma_en) {
		I2CLOG("DMA register(0x%p):\n"
		       I2CTAG "INT_FLAG=%x,INT_EN=%x,EN=%x,RST=%x,CON=%x\n"
		       I2CTAG "TX_MEM_ADDR=%x,RX_MEM_ADDR=%x,TX_LEN=%x,RX_LEN=%x\n"
		       I2CTAG "INT_BUF_SIZE=%x,DEBUG_STATUS=%x,ARHP=%x,AWHP=%x\n"
		       I2CTAG "STOP=%x,FLUSH=%x\n",
		       i2c->pdmabase,
		       (__raw_readl(i2c->pdmabase + OFFSET_INT_FLAG)),
		       (__raw_readl(i2c->pdmabase + OFFSET_INT_EN)),
		       (__raw_readl(i2c->pdmabase + OFFSET_EN)),
		       (__raw_readl(i2c->pdmabase + OFFSET_RST)),
		       (__raw_readl(i2c->pdmabase + OFFSET_CON)),
		       (__raw_readl(i2c->pdmabase + OFFSET_TX_MEM_ADDR)),
		       (__raw_readl(i2c->pdmabase + OFFSET_RX_MEM_ADDR)),
		       (__raw_readl(i2c->pdmabase + OFFSET_TX_LEN)),
		       (__raw_readl(i2c->pdmabase + OFFSET_RX_LEN)),
		       (__raw_readl(i2c->pdmabase + OFFSET_INT_BUF_SIZE)),
		       (__raw_readl(i2c->pdmabase + OFFSET_DEBUG_STATUS)),
		       (__raw_readl(i2c->pdmabase + OFFSET_ARHP)),
		       (__raw_readl(i2c->pdmabase + OFFSET_AWHP)),
		       (__raw_readl(i2c->pdmabase + OFFSET_STOP)),
		       (__raw_readl(i2c->pdmabase + OFFSET_FLUSH)));
	}

	if (i2c->dma_en) {
		I2CLOG("AP_DMA global: 0x8: 0x%x, 0xC: 0x%x, 0x70: 0x%x\n",
		       __raw_readl(ap_dma_base + 0x08),
		       __raw_readl(ap_dma_base + 0x0C), __raw_readl(ap_dma_base + 0x70));

	}
#if !defined(CONFIG_MT_I2C_FPGA_ENABLE) && defined(CONFIG_MTK_LEGACY)
#if defined(GPIO_I2C0_SDA_PIN) && defined(GPIO_I2C1_SDA_PIN) && defined(GPIO_I2C2_SDA_PIN)
	I2CLOG("I2C0_SDA=%d,I2C0_SCL=%d,I2C1_SDA=%d,I2C1_SCL=%d,I2C2_SDA=%d,I2C2_SCL=%d\n",
	       mt_get_gpio_in(GPIO_I2C0_SDA_PIN), mt_get_gpio_in(GPIO_I2C0_SCA_PIN),
	       mt_get_gpio_in(GPIO_I2C1_SDA_PIN), mt_get_gpio_in(GPIO_I2C1_SCA_PIN),
	       mt_get_gpio_in(GPIO_I2C2_SDA_PIN), mt_get_gpio_in(GPIO_I2C2_SCA_PIN));
#endif
#if defined(GPIO_I2C3_SDA_PIN) && defined(GPIO_I2C_APPM_SDA_PIN)
	I2CLOG("I2C3_SDA=%d,I2C3_SCL=%d,I2C_APPM_SDA=%d,I2C_APPM_SCL=%d\n",
	       mt_get_gpio_in(GPIO_I2C3_SDA_PIN), mt_get_gpio_in(GPIO_I2C3_SCA_PIN),
	       mt_get_gpio_in(GPIO_I2C_APPM_SDA_PIN), mt_get_gpio_in(GPIO_I2C_APPM_SCA_PIN));
#endif
#endif
}

#define DMA_POLL_THRES 10
static int dma_busy_wait_ready(struct mt_i2c_t *i2c)
{
	int dma_tmo_poll = DMA_POLL_THRES;

	if (NULL == i2c) {
		I2CERR("dma_busy_wait_ready NULL pointer error\n");
		return -1;
	}

	/* Wait for EN bit back to 0 */
	while (1 == (__raw_readl(i2c->pdmabase + OFFSET_EN))) {
		/* I2CERR("wait dma transfer complet,dma_tmo_poll=%ld\n",dma_tmo_poll); */
		udelay(5);
		dma_tmo_poll--;
		if (dma_tmo_poll == 0)
			return -1;
	}
	return 0;
}

static void dma_reset(struct mt_i2c_t *i2c)
{
	int dma_tmo_poll = DMA_POLL_THRES;

	if (NULL == i2c) {
		I2CERR("dma_reset NULL pointer error\n");
		return;
	}

	/* warm reset DMA to wait for transfer done */
	mt_reg_sync_writel(0x0001, i2c->pdmabase + OFFSET_RST);

	/* Wait for RST bit back to 0 */
	while (1 == (__raw_readl(i2c->pdmabase + OFFSET_RST))) {
		/* I2CERR("Wait DMA warm reset: %d\n", dma_tmo_poll); */
		udelay(5);
		dma_tmo_poll--;
		if (dma_tmo_poll == 0) {
			I2CERR("Wait for DMA warm reset error\n");
			return;
		}
	}
}

static S32 _i2c_deal_result(struct mt_i2c_t *i2c)
{
#ifdef I2C_DRIVER_IN_KERNEL
	long tmo = i2c->adap.timeout;
#else
	long tmo = 1;
#endif
	U16 data_size = 0;
	U8 *ptr = i2c->msg_buf;
	S32 ret = i2c->msg_len;
	long tmo_poll = 0xffff;
	int dma_err = 0;
	#ifdef VENDOR_EDIT
	//zhihong.lu@BSP.sensor,2016/5/23,avoid the i2c err turn to oops
	char fifo_debug[16]= {0};
	int i=0;
	int temp_len=0;
	int len_err_flag =0;
	#endif /*VENDOR_EDIT*/
	/* I2CFUC(); */
	/* addr_reg = i2c->read_flag ? ((i2c->addr << 1) | 0x1) : ((i2c->addr << 1) & ~0x1); */

	if (i2c->poll_en) {	/*master read && poll mode */
		for (;;) {	/*check the interrupt status register */
			i2c->irq_stat = i2c_readl(i2c, OFFSET_INTR_STAT);
			/* I2CLOG("irq_stat = 0x%x\n", i2c->irq_stat); */
			if (i2c->irq_stat & (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP)) {
				atomic_set(&i2c->trans_stop, 1);
				spin_lock(&i2c->lock);
				/*Clear interrupt status,write 1 clear */
				i2c_writel(i2c, OFFSET_INTR_STAT,
					   (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
				spin_unlock(&i2c->lock);
				break;
			}
			tmo_poll--;
			if (tmo_poll == 0) {
				tmo = 0;
				break;
			}
		}
	} else {		/*Interrupt mode,wait for interrupt wake up */
		tmo = wait_event_timeout(i2c->wait, atomic_read(&i2c->trans_stop), tmo);
	}

	/*Save status register status to i2c struct */
#ifdef I2C_DRIVER_IN_KERNEL
	if (i2c->irq_stat & I2C_TRANSAC_COMP) {
		atomic_set(&i2c->trans_err, 0);
		atomic_set(&i2c->trans_comp, 1);
	}
	atomic_set(&i2c->trans_err, i2c->irq_stat & (I2C_HS_NACKERR | I2C_ACKERR));
#endif

	/*Check the transfer status */
	if (!(tmo == 0 || atomic_read(&i2c->trans_err))) {
		/*Transfer success, we need to get data from fifo */
		if ((!i2c->dma_en) && (i2c->op == I2C_MASTER_RD || i2c->op == I2C_MASTER_WRRD)) {
			/*only read mode or write_read mode and fifo mode need to get data */
			data_size = (i2c_readl(i2c, OFFSET_FIFO_STAT) >> 4) & 0x000F;
			#ifndef VENDOR_EDIT
			//zhihong.lu@BSP.sensor,2016/5/23,avoid the i2c err turn to oops
			if (i2c->op == I2C_MASTER_RD && data_size > i2c->msg_len) {
				I2CERR("data_size=%d, msg_len=%d\n", data_size, i2c->msg_len);
				_i2c_dump_info(i2c);
				BUG_ON(data_size > i2c->msg_len);
			}

			if (i2c->op == I2C_MASTER_WRRD && data_size > i2c->trans_data.trans_auxlen) {
				I2CERR("data_size=%d, msg_len=%d\n", data_size, i2c->msg_len);
				_i2c_dump_info(i2c);
				BUG_ON(data_size > i2c->trans_data.trans_auxlen);
			}

			while (data_size--) {
				*ptr = i2c_readl(i2c, OFFSET_DATA_PORT);
				/* I2CLOG("addr %x read byte = 0x%x\n", i2c->addr, *ptr); */
				ptr++;
			}
			#else
			if(i2c->op == I2C_MASTER_WRRD){
				//hw issue workaround
				if(data_size > i2c->trans_data.trans_auxlen){
					I2CERR("id=%d,addr: %x, read len error\n",i2c->id, i2c->addr);
					I2CERR("data_size=%d,trans_auxlen=%d,i2c->msg_len=0x%x\n",data_size,i2c->trans_data.trans_auxlen,i2c->msg_len);
					_i2c_dump_info(i2c);
					i=0;
					temp_len = data_size;
					while (temp_len--){
						fifo_debug[i] = i2c_readl(i2c, OFFSET_DATA_PORT);
						I2CERR("addr %x read data = 0x%x\n", i2c->addr, fifo_debug[i] );
						i++;
					}
					data_size = i2c->trans_data.trans_auxlen;
					len_err_flag =1;
					//BUG_ON(data_size > i2c->trans_data.trans_auxlen);
					WARN_ON(data_size > i2c->trans_data.trans_auxlen);
				}
			}
			if(i2c->op == I2C_MASTER_RD){
				//hw issue workaround
				if(data_size > i2c->trans_data.trans_len){
					I2CERR("id=%d,addr: %x, read len error\n",i2c->id, i2c->addr);
					I2CERR("data_size=%d,trans_len=%d,i2c->msg_len=0x%x\n",data_size,i2c->trans_data.trans_len,i2c->msg_len);
					_i2c_dump_info(i2c);
					i=0;
					temp_len = data_size;
					while (temp_len--){
						fifo_debug[i] = i2c_readl(i2c, OFFSET_DATA_PORT);
						I2CERR("addr %x read data = 0x%x\n", i2c->addr, fifo_debug[i] );
						i++;
					}
					data_size = i2c->trans_data.trans_len;
					len_err_flag =1;
					//BUG_ON(data_size > i2c->trans_data.trans_len);
					WARN_ON(data_size > i2c->trans_data.trans_len);
				}
			}

			i =0;
			while (data_size--){
				if(1==len_err_flag){
					*ptr = fifo_debug[i];
					i++;
				}else{
					*ptr = i2c_readl(i2c, OFFSET_DATA_PORT);
				}
				ptr++;
			}
			#endif /*VENDOR_EDIT*/
		}
		if (i2c->dma_en) {
			dma_err = dma_busy_wait_ready(i2c);
			if (dma_err) {
				I2CERR("i2c ok wait dma ready err\n");
				_i2c_dump_info(i2c);
				dma_reset(i2c);
			}
		}
#ifdef I2C_DEBUG_FS
		_i2c_dump_info(i2c);
#endif
	} else {
		/*Timeout or ACKERR */
		if (tmo == 0) {
			I2CERR("id=%d,addr: %x, transfer timeout\n", i2c->id, i2c->addr);
			ret = -ETIMEDOUT_I2C;
		} else {
			I2CERR("id=%d,addr: %x, transfer error\n", i2c->id, i2c->addr);
			ret = -EREMOTEIO_I2C;
		}
		if (i2c->irq_stat & I2C_HS_NACKERR)
			I2CERR("I2C_HS_NACKERR\n");
		if (i2c->irq_stat & I2C_ACKERR)
			I2CERR("I2C_ACKERR\n");
		if (i2c->filter_msg == FALSE)	/* TEST */
			_i2c_dump_info(i2c);

		spin_lock(&i2c->lock);
		/*Reset i2c port */
		i2c_writel(i2c, OFFSET_SOFTRESET, 0x0001);
		/*Set slave address */
		i2c_writel(i2c, OFFSET_SLAVE_ADDR, 0x0000);
		/*Clear interrupt status */
		i2c_writel(i2c, OFFSET_INTR_STAT, (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
		if (i2c->dma_en)
			dma_reset(i2c);
		/*Clear fifo address */
		i2c_writel(i2c, OFFSET_FIFO_ADDR_CLR, 0x0001);

		spin_unlock(&i2c->lock);
	}
	return ret;
}

static void record_i2c_dma_info(struct mt_i2c_t *i2c)
{
	g_dma_data[i2c->id].base = i2c->pdmabase;
	g_dma_data[i2c->id].int_flag = (__raw_readl(i2c->pdmabase + OFFSET_INT_FLAG));
	g_dma_data[i2c->id].int_en = (__raw_readl(i2c->pdmabase + OFFSET_INT_EN));
	g_dma_data[i2c->id].en = (__raw_readl(i2c->pdmabase + OFFSET_EN));
	g_dma_data[i2c->id].rst = (__raw_readl(i2c->pdmabase + OFFSET_RST));
	g_dma_data[i2c->id].stop = (__raw_readl(i2c->pdmabase + OFFSET_STOP));
	g_dma_data[i2c->id].flush = (__raw_readl(i2c->pdmabase + OFFSET_FLUSH));
	g_dma_data[i2c->id].con = (__raw_readl(i2c->pdmabase + OFFSET_CON));
	g_dma_data[i2c->id].tx_mem_addr = (__raw_readl(i2c->pdmabase + OFFSET_TX_MEM_ADDR));
	g_dma_data[i2c->id].rx_mem_addr = (__raw_readl(i2c->pdmabase + OFFSET_RX_MEM_ADDR));
	g_dma_data[i2c->id].tx_len = (__raw_readl(i2c->pdmabase + OFFSET_TX_LEN));
	g_dma_data[i2c->id].rx_len = (__raw_readl(i2c->pdmabase + OFFSET_RX_LEN));
	g_dma_data[i2c->id].int_buf_size = (__raw_readl(i2c->pdmabase + OFFSET_INT_BUF_SIZE));
	g_dma_data[i2c->id].debug_sta = (__raw_readl(i2c->pdmabase + OFFSET_DEBUG_STATUS));
	g_dma_data[i2c->id].arhp = (__raw_readl(i2c->pdmabase + OFFSET_ARHP));
	g_dma_data[i2c->id].awhp = (__raw_readl(i2c->pdmabase + OFFSET_AWHP));
}

static void _i2c_write_reg(struct mt_i2c_t *i2c)
{
	U8 *ptr = i2c->msg_buf;
	U32 data_size = i2c->trans_data.data_size;
	U32 addr_reg = 0;
	/* I2CFUC(); */

	i2c_writel(i2c, OFFSET_CONTROL, i2c->control_reg);

	/*set start condition */
//#ifndef VENDOR_EDIT
//wenjie.Liu@Prd.Connectivity.NFC&GPS, 2016/02/17, Modify for : NFC I2C thd.sta =0.59us less than 0.60us (sta
/*
	if (i2c->speed <= 100)
		i2c_writel(i2c, OFFSET_EXT_CONF, 0x8001);
	else
		i2c_writel(i2c, OFFSET_EXT_CONF, 0x1800);
*/
//#else /* VENDOR_EDIT */
    if (i2c->speed <= 100)
        i2c_writel(i2c, OFFSET_EXT_CONF, 0x8001);
    else if (i2c->speed > 390 && i2c->speed <= 400)
        i2c_writel(i2c, OFFSET_EXT_CONF, 0x101);
    else
       i2c_writel(i2c, OFFSET_EXT_CONF, 0x1800);    
//#endif /* VENDOR_EDIT */	


	/* set timing reg */
	i2c_writel(i2c, OFFSET_TIMING, i2c->timing_reg);
	i2c_writel(i2c, OFFSET_HS, i2c->high_speed_reg);

	if (0 == i2c->delay_len)
		i2c->delay_len = 2;
	if (~i2c->control_reg & I2C_CONTROL_RS) {	/* bit is set to 1, i.e.,use repeated stop */
		i2c_writel(i2c, OFFSET_DELAY_LEN, i2c->delay_len);
	}

	/*Set ioconfig */
	if (i2c->pushpull)
		i2c_writel(i2c, OFFSET_IO_CONFIG, 0x0000);
	else
		i2c_writel(i2c, OFFSET_IO_CONFIG, 0x0003);

	/*Set slave address */

	addr_reg = i2c->read_flag ? ((i2c->addr << 1) | 0x1) : ((i2c->addr << 1) & ~0x1);
	i2c_writel(i2c, OFFSET_SLAVE_ADDR, addr_reg);
	/*Clear interrupt status */
	i2c_writel(i2c, OFFSET_INTR_STAT, (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
	/*Clear fifo address */
	i2c_writel(i2c, OFFSET_FIFO_ADDR_CLR, 0x0001);
	/*Setup the interrupt mask flag */
	if (i2c->poll_en) {
		/*Disable interrupt */
		i2c_writel(i2c, OFFSET_INTR_MASK, i2c_readl(i2c, OFFSET_INTR_MASK)
					& ~(I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
	} else {
		/*Enable interrupt */
		i2c_writel(i2c, OFFSET_INTR_MASK, i2c_readl(i2c, OFFSET_INTR_MASK)
					| (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
	}
	/*Set transfer len */
	i2c_writel(i2c, OFFSET_TRANSFER_LEN, i2c->trans_data.trans_len & 0xFFFF);
	i2c_writel(i2c, OFFSET_TRANSFER_LEN_AUX, i2c->trans_data.trans_auxlen & 0xFFFF);
	/*Set transaction len */
	i2c_writel(i2c, OFFSET_TRANSAC_LEN, i2c->trans_data.trans_num & 0xFF);

	/*Prepare buffer data to start transfer */

	if (i2c->dma_en) {
		/* Warm reset I2C DMA status */
		dma_reset(i2c);
		if (I2C_MASTER_RD == i2c->op) {
			mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_INT_FLAG);
			mt_reg_sync_writel(0x0001, i2c->pdmabase + OFFSET_CON);
			mt_reg_sync_writel((u32) ((long)i2c->msg_buf),
					   i2c->pdmabase + OFFSET_RX_MEM_ADDR);
			mt_reg_sync_writel(i2c->trans_data.data_size,
					   i2c->pdmabase + OFFSET_RX_LEN);
			if (enable_4G())
				mt_reg_sync_writel(0x1, i2c->pdmabase + OFFSET_AWHP);
		} else if (I2C_MASTER_WR == i2c->op) {
			mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_INT_FLAG);
			mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_CON);
			mt_reg_sync_writel((u32) ((long)i2c->msg_buf),
					   i2c->pdmabase + OFFSET_TX_MEM_ADDR);
			mt_reg_sync_writel(i2c->trans_data.data_size,
					   i2c->pdmabase + OFFSET_TX_LEN);
			if (enable_4G())
				mt_reg_sync_writel(0x1, i2c->pdmabase + OFFSET_ARHP);
		} else {
			mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_INT_FLAG);
			mt_reg_sync_writel(0x0000, i2c->pdmabase + OFFSET_CON);
			mt_reg_sync_writel((u32) ((long)i2c->msg_buf),
					   i2c->pdmabase + OFFSET_TX_MEM_ADDR);
			mt_reg_sync_writel((u32) ((long)i2c->msg_buf),
					   i2c->pdmabase + OFFSET_RX_MEM_ADDR);
			mt_reg_sync_writel(i2c->trans_data.trans_len,
					   i2c->pdmabase + OFFSET_TX_LEN);
			mt_reg_sync_writel(i2c->trans_data.trans_auxlen,
					   i2c->pdmabase + OFFSET_RX_LEN);
			if (enable_4G()) {
				mt_reg_sync_writel(0x1, i2c->pdmabase + OFFSET_AWHP);
				mt_reg_sync_writel(0x1, i2c->pdmabase + OFFSET_ARHP);
			}
		}
		/* record dma info for debug */
		record_i2c_dma_info(i2c);

		/*All register must be prepared before setting the start bit [SMP] */
		mb();
		mt_reg_sync_writel(0x0001, i2c->pdmabase + OFFSET_EN);

		I2CINFO(I2C_T_DMA, "addr %.2x dma %.2X byte\n", i2c->addr,
			i2c->trans_data.data_size);
		I2CINFO(I2C_T_DMA, "DMA Register:INT_FLAG:0x%x,CON:0x%x,TX_MEM_ADDR:0x%x",
					readl(i2c->pdmabase + OFFSET_INT_FLAG),
					readl(i2c->pdmabase + OFFSET_CON),
					readl(i2c->pdmabase + OFFSET_TX_MEM_ADDR));
		I2CINFO(I2C_T_DMA, "RX_MEM_ADDR:0x%x,TX_LEN:0x%x,RX_LEN:0x%x,EN:0x%x\n",
					readl(i2c->pdmabase + OFFSET_RX_MEM_ADDR),
					readl(i2c->pdmabase + OFFSET_TX_LEN),
					readl(i2c->pdmabase + OFFSET_RX_LEN),
					readl(i2c->pdmabase + OFFSET_EN));
	} else {
		/*Set fifo mode data */
		if (I2C_MASTER_RD == i2c->op) {
			/*do not need set fifo data */
		} else {	/*both write && write_read mode */
			while (data_size--) {
				i2c_writel(i2c, OFFSET_DATA_PORT, *ptr);
				/* dev_info(i2c->dev, "addr %.2x write byte = 0x%.2X\n", addr, *ptr); */
				ptr++;
			}
		}
	}
	/*Set trans_data */
	i2c->trans_data.data_size = data_size;

	if (0x0 == (i2c_readl(i2c, OFFSET_TIMING))) {
		/* set timing reg */
		i2c_writel(i2c, OFFSET_TIMING, 0x1410);
		aee_kernel_warning(I2CTAG, "@%s():%d,\n", __func__, __LINE__);
		/* i2c_writel(i2c, OFFSET_HS, i2c->high_speed_reg); */
	}
}

static S32 _i2c_get_transfer_len(struct mt_i2c_t *i2c)
{
	S32 ret = I2C_OK;
	u16 trans_num = 0;
	u16 data_size = 0;
	u16 trans_len = 0;
	u16 trans_auxlen = 0;
	/* I2CFUC(); */
	/*Get Transfer len and transaux len */
	if (FALSE == i2c->dma_en) {	/*non-DMA mode */
		if (I2C_MASTER_WRRD != i2c->op) {
			trans_len = (i2c->msg_len) & 0xFFFF;
			trans_num = (i2c->msg_len >> 16) & 0xFF;
			if (0 == trans_num)
				trans_num = 1;
			trans_auxlen = 0;
			data_size = trans_len * trans_num;

			if (!trans_len || !trans_num || data_size > I2C_FIFO_SIZE) {
				I2CERR("non-WRRD trans_len is wrong. trans_len=%x, tans_num=%x, trans_auxlen=%x\n",
					trans_len, trans_num, trans_auxlen);
				I2C_BUG_ON(!trans_len || !trans_num || data_size > I2C_FIFO_SIZE);
				ret = -EINVAL_I2C;
			}
		} else {
			trans_len = (i2c->msg_len) & 0xFF;
			trans_auxlen = (i2c->msg_len >> 8) & 0xFF;
			trans_num = 2;
			data_size = trans_len;
			if (!trans_len || !trans_auxlen || trans_len > I2C_FIFO_SIZE
			    || trans_auxlen > I2C_FIFO_SIZE) {
				I2CERR("WRRD trans_len is wrong. trans_len=%x, tans_num=%x, trans_auxlen=%x\n",
					trans_len, trans_num, trans_auxlen);
				I2C_BUG_ON(!trans_len || !trans_auxlen || trans_len > I2C_FIFO_SIZE
					   || trans_auxlen > I2C_FIFO_SIZE);
				ret = -EINVAL_I2C;
			}
		}
	} else {		/*DMA mode */
		if (I2C_MASTER_WRRD != i2c->op) {
			trans_len = (i2c->msg_len) & 0xFFFF;
			trans_num = (i2c->msg_len >> 16) & 0xFF;
			if (0 == trans_num)
				trans_num = 1;
			trans_auxlen = 0;
			data_size = trans_len * trans_num;

			if (!trans_len || !trans_num || trans_len > MAX_DMA_TRANS_SIZE
			    || trans_num > MAX_DMA_TRANS_NUM) {
				I2CERR("DMA non-WRRD trans_len is wrong. trans_len=%x, tans_num=%x, trans_auxlen=%x\n",
				     trans_len, trans_num, trans_auxlen);
				I2C_BUG_ON(!trans_len || !trans_num
					   || trans_len > MAX_DMA_TRANS_SIZE
					   || trans_num > MAX_DMA_TRANS_NUM);
				ret = -EINVAL_I2C;
			}
			I2CINFO(I2C_T_DMA, "DMA non-WRRD mode!trans_len=%x, tans_num=%x, trans_auxlen=%x\n",
				trans_len, trans_num, trans_auxlen);
		} else {
			trans_len = (i2c->msg_len) & 0xFF;
			trans_auxlen = (i2c->msg_len >> 8) & 0xFF;
			trans_num = 2;
			data_size = trans_len;
			if (!trans_len || !trans_auxlen || trans_len > MAX_DMA_TRANS_SIZE
			    || trans_auxlen > MAX_DMA_TRANS_NUM) {
				I2CERR("DMA WRRD trans_len is wrong. trans_len=%x, tans_num=%x, trans_auxlen=%x\n",
				     trans_len, trans_num, trans_auxlen);
				I2C_BUG_ON(!trans_len || !trans_auxlen
					   || trans_len > MAX_DMA_TRANS_SIZE
					   || trans_auxlen > MAX_DMA_TRANS_NUM);
				ret = -EINVAL_I2C;
			}
			I2CINFO(I2C_T_DMA,
				"DMA WRRD mode!trans_len=%x, tans_num=%x, trans_auxlen=%x\n",
				trans_len, trans_num, trans_auxlen);
		}
	}

	i2c->trans_data.trans_num = trans_num;
	i2c->trans_data.trans_len = trans_len;
	i2c->trans_data.data_size = data_size;
	i2c->trans_data.trans_auxlen = trans_auxlen;

	return ret;
}

static S32 _i2c_transfer_interface(struct mt_i2c_t *i2c)
{
	S32 return_value = 0;
	S32 ret = 0;
	U8 *ptr = i2c->msg_buf;
	/* I2CFUC(); */

	if (i2c->dma_en) {
		I2CINFO(I2C_T_DMA, "DMA Transfer mode!\n");
		if (i2c->pdmabase == 0) {
			I2CERR(" I2C%d doesnot support DMA mode!\n", i2c->id);
			I2C_BUG_ON(i2c->pdmabase == NULL);
			ret = -EINVAL_I2C;
			goto err;
		}
		/* DMA mode shouldn't use virtual memory address */
		if (virt_addr_valid(ptr)) {
			I2CERR(" DMA mode should use physical buffer address!\n");
			I2C_BUG_ON(virt_addr_valid(ptr));
			ret = -EINVAL_I2C;
			goto err;
		}
	}
#ifdef I2C_DRIVER_IN_KERNEL
	atomic_set(&i2c->trans_stop, 0);
	atomic_set(&i2c->trans_comp, 0);
	atomic_set(&i2c->trans_err, 0);
#endif
	i2c->irq_stat = 0;

	return_value = _i2c_get_transfer_len(i2c);
	if (return_value < 0) {
		I2CERR("_i2c_get_transfer_len fail,return_value=%d\n", return_value);
		ret = -EINVAL_I2C;
		goto err;
	}
	/* get clock */
#ifdef CONFIG_MT_I2C_FPGA_ENABLE
	i2c->clk = I2C_CLK_RATE;
#else
	/* i2c->clk = mt_get_bus_freq() / I2C_CLK_DIV; */
	i2c->clk = I2C_CLK_RATE;
#endif

	return_value = i2c_set_speed(i2c);
	if (return_value < 0) {
		I2CERR("i2c_set_speed fail,return_value=%d\n", return_value);
		ret = -EINVAL_I2C;
		goto err;
	}
	/*Set Control Register */
	i2c->control_reg = I2C_CONTROL_ACKERR_DET_EN | I2C_CONTROL_CLK_EXT_EN;
	if (i2c->dma_en)
		i2c->control_reg |= I2C_CONTROL_DMA_EN;

	if (I2C_MASTER_WRRD == i2c->op)
		i2c->control_reg |= I2C_CONTROL_DIR_CHANGE;

	if (HS_MODE == i2c->mode
	    || (i2c->trans_data.trans_num > 1 && I2C_TRANS_REPEATED_START == i2c->st_rs)) {
		i2c->control_reg |= I2C_CONTROL_RS;
	}

#if !defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	/* Use HW semaphore to protect mt6311 access between AP and SPM */
	if (i2c_get_semaphore(i2c) != 0)
		return -EBUSY;
#endif

	spin_lock(&i2c->lock);
	_i2c_write_reg(i2c);

	/*All register must be prepared before setting the start bit [SMP] */
	mb();
#ifdef I2C_DRIVER_IN_KERNEL
	/*This is only for 3D CAMERA */
	if (i2c->i2c_3dcamera_flag) {
		spin_unlock(&i2c->lock);
		if (g_i2c[0] == NULL)
			g_i2c[0] = i2c;
		else
			g_i2c[1] = i2c;

		goto end;
	}
#endif
	I2CINFO(I2C_T_TRANSFERFLOW, "Before start .....\n");
#if defined(I2C_DEBUG_FS) && defined(CONFIG_MTK_LEGACY)
#if defined(GPIO_I2C0_SDA_PIN) && defined(GPIO_I2C1_SDA_PIN) && defined(GPIO_I2C2_SDA_PIN)
	I2CLOG("I2C0_SDA=%d,I2C0_SCL=%d,I2C1_SDA=%d,I2C1_SCL=%d,I2C2_SDA=%d,I2C2_SCL=%d\n",
	       mt_get_gpio_in(GPIO_I2C0_SDA_PIN), mt_get_gpio_in(GPIO_I2C0_SCA_PIN),
	       mt_get_gpio_in(GPIO_I2C1_SDA_PIN), mt_get_gpio_in(GPIO_I2C1_SCA_PIN),
	       mt_get_gpio_in(GPIO_I2C2_SDA_PIN), mt_get_gpio_in(GPIO_I2C2_SCA_PIN));
#endif
#endif

	/*Start the transfer */
	i2c_writel(i2c, OFFSET_START, 0x0001);
	spin_unlock(&i2c->lock);
	ret = _i2c_deal_result(i2c);

#if !defined(CONFIG_MTK_PMIC_CHIP_MT6353)
	/* Use HW semaphore to protect mt6311 access between AP and SPM */
	if (i2c_release_semaphore(i2c) != 0)
		ret = -EBUSY;
#endif

	I2CINFO(I2C_T_TRANSFERFLOW, "After i2c transfer .....\n");
err:
end:
	return ret;
}

/*=========API in kernel=====================================================================*/
static void _i2c_translate_msg(struct mt_i2c_t *i2c, struct i2c_msg *msg)
{
  /*-------------compatible with 77/75 driver------*/
	if (msg->addr & 0xFF00)
		msg->ext_flag |= msg->addr & 0xFF00;
	I2CINFO(I2C_T_TRANSFERFLOW, "Before i2c transfer .....\n");

	i2c->msg_buf = msg->buf;
	i2c->msg_len = msg->len;
	if (msg->ext_flag & I2C_RS_FLAG)
		i2c->st_rs = I2C_TRANS_REPEATED_START;
	else
		i2c->st_rs = I2C_TRANS_STOP;

	if (msg->ext_flag & I2C_DMA_FLAG)
		i2c->dma_en = TRUE;
	else
		i2c->dma_en = FALSE;

	if (msg->ext_flag & I2C_WR_FLAG)
		i2c->op = I2C_MASTER_WRRD;
	else {
		if (msg->flags & I2C_M_RD)
			i2c->op = I2C_MASTER_RD;
		else
			i2c->op = I2C_MASTER_WR;
	}
	if (msg->ext_flag & I2C_POLLING_FLAG)
		i2c->poll_en = TRUE;
	else
		i2c->poll_en = FALSE;

	if (msg->ext_flag & I2C_A_FILTER_MSG)
		i2c->filter_msg = TRUE;
	else
		i2c->filter_msg = FALSE;
	i2c->delay_len = (msg->timing & 0xff0000) >> 16;

	/* Set device speed,set it before set_control register */
	if (0 == (msg->timing & 0xFFFF)) {
		i2c->mode = ST_MODE;
		i2c->speed = MAX_ST_MODE_SPEED;
	} else {
		if (msg->ext_flag & I2C_HS_FLAG)
			i2c->mode = HS_MODE;
		else
			i2c->mode = FS_MODE;

		i2c->speed = msg->timing & 0xFFFF;
	}

	/*Set ioconfig */
	if (msg->ext_flag & I2C_PUSHPULL_FLAG)
		i2c->pushpull = TRUE;
	else
		i2c->pushpull = FALSE;

	if (msg->ext_flag & I2C_3DCAMERA_FLAG)
		i2c->i2c_3dcamera_flag = TRUE;
	else
		i2c->i2c_3dcamera_flag = FALSE;

}

static S32 mt_i2c_start_xfer(struct mt_i2c_t *i2c, struct i2c_msg *msg)
{
	S32 return_value = 0;
	S32 ret = msg->len;
	/* start=========================Check param valid===================================== */
	/* I2CLOG(" mt_i2c_start_xfer.\n"); */
	/* get the read/write flag */
	i2c->read_flag = (msg->flags & I2C_M_RD);
	i2c->addr = msg->addr;
	if (i2c->addr == 0) {
		I2CERR(" addr is invalid.\n");
		I2C_BUG_ON(i2c->addr == NULL);
		ret = -EINVAL_I2C;
		goto err;
	}

	if (msg->buf == NULL) {
		I2CERR(" data buffer is NULL.\n");
		I2C_BUG_ON(msg->buf == NULL);
		ret = -EINVAL_I2C;
		goto err;
	}
	if (g_i2c[0] == i2c || g_i2c[1] == i2c) {
		I2CERR("mt-i2c%d: Current I2C Adapter is busy.\n", i2c->id);
		ret = -EINVAL_I2C;
		goto err;
	}
	/* start=========================translate msg to mt_i2c=============================== */
	_i2c_translate_msg(i2c, msg);
#ifdef I2C_DRIVER_IN_KERNEL
	/*Save address infomation for 3d camera */
	if (i2c->i2c_3dcamera_flag) {
		if (g_msg[0].buf == NULL)
			memcpy((void *)&g_msg[0], msg, sizeof(struct i2c_msg));
		else
			memcpy((void *)&g_msg[1], msg, sizeof(struct i2c_msg));
	}
#endif
	/* end=========================translate msg to mt_i2c=============================== */
	mt_i2c_clock_enable(i2c);
	return_value = _i2c_transfer_interface(i2c);
	if (!(msg->ext_flag & I2C_3DCAMERA_FLAG))
		mt_i2c_clock_disable(i2c);
	if (return_value < 0) {
		ret = -EINVAL_I2C;
		goto err;
	}
err:
	return ret;
}

static S32 mt_i2c_do_transfer(struct mt_i2c_t *i2c, struct i2c_msg *msgs, S32 num)
{
	S32 ret = 0;
	S32 left_num = num;

	while (left_num--) {
		ret = mt_i2c_start_xfer(i2c, msgs++);
		if (ret < 0) {
			if (ret != -EINVAL_I2C)	/*We never try again when the param is invalid */
				return -EAGAIN;
			else
				return -EINVAL_I2C;
		}
	}
	/*the return value is number of executed messages */
	return num;
}

static S32 mt_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg msgs[], S32 num)
{
	S32 ret = 0;
	S32 retry;
	struct mt_i2c_t *i2c = i2c_get_adapdata(adap);

	for (retry = 0; retry < adap->retries; retry++) {
		ret = mt_i2c_do_transfer(i2c, msgs, num);
		if (ret != -EAGAIN)
			break;
		if (retry < adap->retries - 1)
			udelay(100);
	}

	if (ret != -EAGAIN)
		return ret;
	else
		return -EREMOTEIO;
}

#ifdef I2C_DRIVER_IN_KERNEL
static S32 _i2c_deal_result_3dcamera(struct mt_i2c_t *i2c, struct i2c_msg *msg)
{
	U16 addr = msg->addr;
	U16 read = (msg->flags & I2C_M_RD);

	i2c->msg_buf = msg->buf;
	i2c->msg_len = msg->len;
	i2c->addr = read ? ((addr << 1) | 0x1) : ((addr << 1) & ~0x1);
	return _i2c_deal_result(i2c);
}
#endif

static void mt_i2c_clock_enable(struct mt_i2c_t *i2c)
{
#if (!defined(CONFIG_MT_I2C_FPGA_ENABLE))
	if (i2c->dma_en) {
		I2CINFO(I2C_T_TRANSFERFLOW, "Before dma clock enable .....\n");
#ifdef CONFIG_MTK_CLKMGR
		enable_clock(MT_CG_APDMA_SW_CG, "i2c");
#else
		clk_prepare_enable(i2c->clk_dma);
#endif
	}
	I2CINFO(I2C_T_TRANSFERFLOW, "Before i2c clock enable .....\n");
#ifdef CONFIG_MTK_CLKMGR
	enable_clock(i2c->pdn, "i2c");
#else
	clk_prepare_enable(i2c->clk_main);
#endif
	I2CINFO(I2C_T_TRANSFERFLOW, "clock enable done.....\n");
#endif
}

static void mt_i2c_clock_disable(struct mt_i2c_t *i2c)
{
#if (!defined(CONFIG_MT_I2C_FPGA_ENABLE))
	if (i2c->dma_en) {
		I2CINFO(I2C_T_TRANSFERFLOW, "Before dma clock disable .....\n");
#ifdef CONFIG_MTK_CLKMGR
		disable_clock(MT_CG_APDMA_SW_CG, "i2c");
#else
		clk_disable_unprepare(i2c->clk_dma);
#endif
	}
	I2CINFO(I2C_T_TRANSFERFLOW, "Before i2c clock disable .....\n");
#ifdef CONFIG_MTK_CLKMGR
	disable_clock(i2c->pdn, "i2c");
#else
	clk_disable_unprepare(i2c->clk_main);
#endif
	I2CINFO(I2C_T_TRANSFERFLOW, "clock disable done .....\n");
#endif
}

#ifdef CONFIG_TRUSTONIC_TEE_SUPPORT
int i2c_tui_enable_clock(void)
{
	struct i2c_adapter *adap;
	struct mt_i2c_t *i2c;

	adap = i2c_get_adapter(0);
	if (!adap) {
		I2CERR("Cannot get adapter\n");
		return -1;
	}

	i2c = i2c_get_adapdata(adap);
	clk_prepare_enable(i2c->clk_main);
	clk_prepare_enable(i2c->clk_dma);

	return 0;
}

int i2c_tui_disable_clock(void)
{
	struct i2c_adapter *adap;
	struct mt_i2c_t *i2c;

	adap = i2c_get_adapter(0);
	if (!adap) {
		I2CERR("Cannot get adapter\n");
		return -1;
	}

	i2c = i2c_get_adapdata(adap);
	clk_disable_unprepare(i2c->clk_dma);
	clk_disable_unprepare(i2c->clk_main);

	return 0;
}
#endif

/*
static void mt_i2c_post_isr(struct mt_i2c_t *i2c)
{
  if (i2c->irq_stat & I2C_TRANSAC_COMP) {
    atomic_set(&i2c->trans_err, 0);
    atomic_set(&i2c->trans_comp, 1);
  }

  if (i2c->irq_stat & I2C_HS_NACKERR) {
    if (i2c->filter_msg==FALSE)
      I2CERR("I2C_HS_NACKERR\n");
  }

  if (i2c->irq_stat & I2C_ACKERR) {
    if (i2c->filter_msg==FALSE)
      I2CERR("I2C_ACKERR\n");
  }
  atomic_set(&i2c->trans_err, i2c->irq_stat & (I2C_HS_NACKERR | I2C_ACKERR));
}*/

/*interrupt handler function*/
static irqreturn_t mt_i2c_irq(S32 irqno, void *dev_id)
{
	struct mt_i2c_t *i2c = (struct mt_i2c_t *) dev_id;
	/* U32 base = i2c->base; */

	I2CINFO(I2C_T_TRANSFERFLOW, "i2c interrupt coming.....\n");
	/* I2CLOG("mt_i2c_irq\n"); */
	/*Clear interrupt mask */
	i2c_writel(i2c, OFFSET_INTR_MASK,
		   i2c_readl(i2c,
			     OFFSET_INTR_MASK) & ~(I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
	/*Save interrupt status */
	i2c->irq_stat = i2c_readl(i2c, OFFSET_INTR_STAT);
	/*Clear interrupt status,write 1 clear */
	i2c_writel(i2c, OFFSET_INTR_STAT, (I2C_HS_NACKERR | I2C_ACKERR | I2C_TRANSAC_COMP));
	/* dev_info(i2c->dev, "I2C interrupt status 0x%04X\n", i2c->irq_stat); */

	/*Wake up process */
	atomic_set(&i2c->trans_stop, 1);
	wake_up(&i2c->wait);
	return IRQ_HANDLED;
}

/*This function is only for 3d camera*/

S32 mt_wait4_i2c_complete(void)
{
	struct mt_i2c_t *i2c0 = g_i2c[0];
	struct mt_i2c_t *i2c1 = g_i2c[1];
	S32 result0, result1;
	S32 ret = 0;

	if ((i2c0 == NULL) || (i2c1 == NULL)) {
		/*What's wrong? */
		ret = -EINVAL_I2C;
		goto end;
	}

	result0 = _i2c_deal_result_3dcamera(i2c0, &g_msg[0]);
	result1 = _i2c_deal_result_3dcamera(i2c1, &g_msg[1]);

	if (result0 < 0 || result1 < 0)
		ret = -EINVAL_I2C;

	if (NULL != i2c0)
		mt_i2c_clock_disable(i2c0);
	if (NULL != i2c1)
		mt_i2c_clock_disable(i2c1);

end:
	g_i2c[0] = NULL;
	g_i2c[1] = NULL;

	g_msg[0].buf = NULL;
	g_msg[1].buf = NULL;

	return ret;
}

static U32 mt_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm mt_i2c_algorithm = {
	.master_xfer = mt_i2c_transfer,
	.smbus_xfer = NULL,
	.functionality = mt_i2c_functionality,
};

static inline void mt_i2c_init_hw(struct mt_i2c_t *i2c)
{
	i2c_writel(i2c, OFFSET_SOFTRESET, 0x0001);
	i2c_writel(i2c, OFFSET_DCM_EN, 0x0);
}

static void mt_i2c_free(struct mt_i2c_t *i2c)
{
	if (!i2c)
		return;

	free_irq(i2c->irqnr, i2c);
	i2c_del_adapter(&i2c->adap);
	kfree(i2c);
}

static S32 mt_i2c_probe(struct platform_device *pdev)
{
	int ret, irq = 0;
	struct mt_i2c_t *i2c = NULL;
	struct resource *res;

	/* Request platform_device IO resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENODEV;

	/* Request IO memory */
	if (!request_mem_region(res->start, resource_size(res), pdev->name))
		return -ENOMEM;

	i2c = kzalloc(sizeof(struct mt_i2c_t), GFP_KERNEL);
	if (NULL == i2c)
		return -ENOMEM;

#ifdef CONFIG_OF
	i2c->base = of_iomap(pdev->dev.of_node, 0);
	if (!i2c->base) {
		I2CERR("I2C iomap failed\n");
		return -ENODEV;
	}

	if (of_property_read_u32(pdev->dev.of_node, "cell-index", &pdev->id)) {
		I2CERR("I2C get cell-index failed\n");
		return -ENODEV;
	}
	i2c->id = pdev->id;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq) {
		I2CERR("I2C get irq failed\n");
		return -ENODEV;
	}
#else

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;

	/* initialize struct mt_i2c_t structure */
	i2c->id = pdev->id;
	i2c->base = IO_PHYS_TO_VIRT(res->start);

#endif
	i2c->irqnr = irq;

#ifdef CONFIG_MTK_CLKMGR
#if (defined(CONFIG_MT_I2C_FPGA_ENABLE))
	i2c->clk = I2C_CLK_RATE;
#else
	/* i2c->clk = mt_get_bus_freq() / I2C_CLK_DIV; */
	i2c->clk = I2C_CLK_RATE;

	switch (i2c->id) {
	case 0:
		i2c->pdn = MT_CG_I2C0_SW_CG;
		break;
	case 1:
		i2c->pdn = MT_CG_I2C1_SW_CG;
		break;
	case 2:
		i2c->pdn = MT_CG_I2C2_SW_CG;
		break;
	case 3:
		i2c->pdn = MT_CG_I2C3_SW_CG;
		break;
	case 4:
		i2c->pdn = MT_CG_I2C4_SW_CG;
		break;
	default:
		dev_err(&pdev->dev, "Error id %d\n", i2c->id);
		break;
	}
#endif
#else
	i2c->clk  = I2C_CLK_RATE;

	i2c->clk_main = devm_clk_get(&pdev->dev, "main");
	if (IS_ERR(i2c->clk_main)) {
		I2CERR("cannot get main clock, err=%ld\n", PTR_ERR(i2c->clk_main));
		return PTR_ERR(i2c->clk_main);
	}

	i2c->clk_dma = devm_clk_get(&pdev->dev, "dma");
	if (IS_ERR(i2c->clk_dma)) {
		I2CERR("cannot get dma clock, err=%ld\n", PTR_ERR(i2c->clk_dma));
		return PTR_ERR(i2c->clk_dma);
	}
#endif

	i2c->dev = &i2c->adap.dev;

	i2c->adap.dev.parent = &pdev->dev;
#ifdef CONFIG_OF
	i2c->adap.dev.of_node = pdev->dev.of_node;
#endif
	i2c->adap.nr = i2c->id;
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &mt_i2c_algorithm;
	i2c->adap.algo_data = NULL;
	i2c->adap.timeout = 2 * HZ;	/*2s */
	i2c->adap.retries = 1;	/*DO NOT TRY */

	snprintf(i2c->adap.name, sizeof(i2c->adap.name), dev_name(&pdev->dev));

#ifdef CONFIG_OF
	i2c->pdmabase = DMA_I2C_BASE(i2c->id, ap_dma_base);
#else
	i2c->pdmabase = DMA_I2C_BASE_CH(i2c->id);
#endif

	I2CLOG(" id: %d, reg: 0x%p, dma_reg: 0x%p, irq: %d\n", i2c->id, i2c->base, i2c->pdmabase,
	       i2c->irqnr);

	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);

	mt_i2c_clock_enable(i2c);
	mt_i2c_init_hw(i2c);
	mt_i2c_clock_disable(i2c);

	ret = request_irq(i2c->irqnr, mt_i2c_irq, IRQF_TRIGGER_NONE, dev_name(&pdev->dev), i2c);
	if (ret) {
		dev_err(&pdev->dev, "Can Not request I2C IRQ %d\n", i2c->irqnr);
		goto free;
	}

	i2c_set_adapdata(&i2c->adap, i2c);
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret) {
		dev_err(&pdev->dev, "failed to add i2c bus to i2c core\n");
		goto free;
	}
	platform_set_drvdata(pdev, i2c);

#ifdef CONFIG_OF
	of_i2c_register_devices(&i2c->adap);
#endif

#ifdef I2C_DEBUG_FS
	ret = device_create_file(i2c->dev, &dev_attr_debug);
#endif

	return ret;

free:
	mt_i2c_free(i2c);
	I2CERR("i2c probe fail\n");
	return ret;
}


static S32 mt_i2c_remove(struct platform_device *pdev)
{
	struct mt_i2c_t *i2c = platform_get_drvdata(pdev);

	if (i2c) {
		platform_set_drvdata(pdev, NULL);
		mt_i2c_free(i2c);
	}
	return 0;
}

#ifdef CONFIG_PM
static S32 mt_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* struct struct mt_i2c_t *i2c = platform_get_drvdata(pdev); */
	/* dev_dbg(i2c->dev,"[I2C %d] Suspend!\n", i2c->id); */
	return 0;
}

static S32 mt_i2c_resume(struct platform_device *pdev)
{
	/* struct struct mt_i2c_t *i2c = platform_get_drvdata(pdev); */
	/* dev_dbg(i2c->dev,"[I2C %d] Resume!\n", i2c->id); */
	return 0;
}
#else
#define mt_i2c_suspend	NULL
#define mt_i2c_resume	NULL
#endif

static const struct of_device_id mt_i2c_of_match[] = {
	{.compatible = "mediatek,mt6755-i2c",},
	{},
};

MODULE_DEVICE_TABLE(of, mt_i2c_of_match);

static struct platform_driver mt_i2c_driver = {
	.probe = mt_i2c_probe,
	.remove = mt_i2c_remove,
	.suspend = mt_i2c_suspend,
	.resume = mt_i2c_resume,
	.driver = {
		.name = I2C_DRV_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mt_i2c_of_match,
#endif
	},
};

static S32 __init mt_i2c_init(void)
{
#ifdef CONFIG_OF
	struct device_node *ap_dma_node;

	/* ioremap the AP_DMA base and use offset get the I2C DMA base */
	ap_dma_node = of_find_compatible_node(NULL, NULL, "mediatek,AP_DMA");
	if (!ap_dma_node) {
		I2CERR("Cannot find AP_DMA node\n");
		return -ENODEV;
	}

	ap_dma_base = of_iomap(ap_dma_node, 0);
	if (!ap_dma_base) {
		I2CERR("AP_DMA iomap failed\n");
		return -ENOMEM;
	}
#endif

	return platform_driver_register(&mt_i2c_driver);
}

static void __exit mt_i2c_exit(void)
{
	platform_driver_unregister(&mt_i2c_driver);
}
module_init(mt_i2c_init);
module_exit(mt_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek I2C Bus Driver");
MODULE_AUTHOR("Infinity Chen <infinity.chen@mediatek.com>");
