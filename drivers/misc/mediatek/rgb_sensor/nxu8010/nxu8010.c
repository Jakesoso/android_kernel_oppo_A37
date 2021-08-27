#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/sensors_io.h>

#include <linux/xlog.h>
#include <rgb_core.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/slab.h>
#include "nxu8010.h"

#define LENS_I2C_BUSNUM 4

#define R_LOW_DATA				0x20
#define R_HIGH_DATA				0x21
#define G_LOW_DATA				0x22
#define G_HIGH_DATA				0x23
#define B_LOW_DATA				0x24
#define B_HIGH_DATA				0x25
#define W_LOW_DATA				0x26
#define W_HIGH_DATA				0x27
#define CORRELATION_CODE		0x2A
#define SLEEP_ADDR				0x00
#define SLEEP_VALUE				0x22

#define NXU8010_DRVNAME "nxu8010"
#define TAG "[NXU8010] "
#define NXU8010_SLAVE_ADDR           0x44

static struct i2c_board_info __initdata nxu8010_board_info = { I2C_BOARD_INFO("nxu8010", NXU8010_SLAVE_ADDR) };
static struct platform_driver g_nxu8010_driver;
static struct mutex nxu8010_mutex;

static const struct i2c_device_id nxu8010_i2c_id[] = { {NXU8010_DRVNAME, 0}, {} };

static struct i2c_client *g_nxu8010_I2Cclient;

static int gRGBWLevel = 1;
static int gRGBWEnabled = 1;
static int gRGBWData[4] = {0,};
static int gRGBWCCT = 0, gRGBWLux = 0;

static int nxu8010_read(unsigned char addr, unsigned char *read_buf)
{
	unsigned char pBuff[2];

	int ret = 0;

	mutex_lock(&nxu8010_mutex);
	g_nxu8010_I2Cclient->addr = NXU8010_SLAVE_ADDR;

	pBuff[0] = addr;

	ret = i2c_master_send(g_nxu8010_I2Cclient, pBuff, 1);

	if (ret < 0) {
		RGBLOG(TAG"I2C send failed!!\n");
		mutex_unlock(&nxu8010_mutex);
		return -1;
	}
	ret = i2c_master_recv(g_nxu8010_I2Cclient, &pBuff[1], 1);

	if (ret < 0) {
		mutex_unlock(&nxu8010_mutex);
		RGBLOG(TAG"I2C recv failed!!\n");
		return -1;
	}
	*read_buf = pBuff[1];

	mutex_unlock(&nxu8010_mutex);

	RGBLOG(TAG"add=0x%x val=0x%x\n", pBuff[0],pBuff[1]);

	return 0;
}

static int nxu8010_write(unsigned char addr, unsigned char data)
{
	int ret = 0;

	unsigned char pBuff[2] = {0};

	mutex_lock(&nxu8010_mutex);
	g_nxu8010_I2Cclient->addr = NXU8010_SLAVE_ADDR;

	pBuff[1] = data;
	pBuff[0] = addr;

	ret = i2c_master_send(g_nxu8010_I2Cclient, pBuff, 2);

	if (ret < 0) {
		mutex_unlock(&nxu8010_mutex);
		RGBLOG(TAG"I2C send failed!!\n");
		return -1;
	}
	mutex_unlock(&nxu8010_mutex);
	return 0;
}

static int nxu8010_init(void)
{
	int i;
	int count = sizeof(NXU8010_RGBW_INIT_DATA) / sizeof(u8) / 2;
    u8 reg_addr = 0, reg_val = 0;

    for(i = 0; i < count; ++i) {
        reg_addr = NXU8010_RGBW_INIT_DATA[i*2];
        reg_val = NXU8010_RGBW_INIT_DATA[i*2+1];

        if( nxu8010_write(reg_addr, reg_val) )  {
            RGBLOG(TAG"Error RGBW init...\n");
            return -1;
        }

        RGBLOG(TAG"set:addr=0x%02x, value=0x%02x\n", reg_addr, reg_val);
    }

	gRGBWLevel = 1;
	return 0;
}

int nxu8010_set_enable_rgb(int en)
{

	RGBLOG(TAG"set_enable_rgb = %d\n", en);

	if ( en ) {
		gRGBWEnabled = 1;
		RGBLOG(TAG"enabled\n");
	}else {
		gRGBWEnabled = 0;
		gRGBWData[0] = gRGBWData[1] = gRGBWData[2] = gRGBWData[3] = 0;
		gRGBWLux = gRGBWCCT = 0;

		nxu8010_write(SLEEP_ADDR, SLEEP_VALUE);
		RGBLOG(TAG"starting sleep mode\n");
	}

	return 0;
}

int nxu8010_get_enable_rgb(void)
{
	RGBLOG(TAG"%d\n", gRGBWEnabled);
	return gRGBWEnabled;
}

static int nxu8010_read_rgb_data(u16 *value, u8 low_addr, u8 high_addr)
{
	u8 read_buf[2] = {0,};

	if ( value == 0 ) return -1;

	nxu8010_read(low_addr, &read_buf[0]);
	nxu8010_read(high_addr, &read_buf[1]);
	*value = (u16)read_buf[0] | (u16)(read_buf[1] << 8);

	return 0;
}


int nxu8010_get_data_w(u16 *value)
{
	//return nxu8010_read_rgb_data(value, W_LOW_DATA, W_HIGH_DATA);
	*value = (u16)gRGBWData[3];
	return 0;
}

int nxu8010_get_data_r(u16 *value)
{
	//return nxu8010_read_rgb_data(value, R_LOW_DATA, R_HIGH_DATA);
	*value = (u16)gRGBWData[0];
	return 0;
}
int nxu8010_get_data_g(u16 *value)
{
	//return nxu8010_read_rgb_data(value, G_LOW_DATA, G_HIGH_DATA);
	*value = (u16)gRGBWData[1];
	return 0;
}
int nxu8010_get_data_b(u16 *value)
{
	//return nxu8010_read_rgb_data(value, B_LOW_DATA, B_HIGH_DATA);
	*value = (u16)gRGBWData[2];
	return 0;
}

int nxu8010_get_data_lux(u32 *value)
{
	*value = gRGBWLux;
	RGBLOG(TAG"%d\n", *value);
	return 0;
}

int nxu8010_get_data_cct(u32 *value)
{
	*value = gRGBWCCT;
	RGBLOG(TAG"%d\n", *value);
	return 0;
}

char* nxu8010_get_chipinfo(void)
{
	static char info[] = "NXU8010";
	//return "NXU8010";
	return info;
}

int nxu8010_rgb_do_calib(void)
{
	return 0;
}

static long nxu8010_unlocked_ioctl( struct file *file, unsigned int cmd,unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	u16 value = 0;
	u8 read_buf = 0;
	int cct, lux;
	int code, enabled;
	int reg[2];

	switch (cmd)
	{
		case RGB_IOCTL_GET_ENABLED:	//nxu8010_read_rgb_data(&value, R_LOW_DATA, R_HIGH_DATA);
			enabled = nxu8010_get_enable_rgb();
			if(copy_to_user(argp, &enabled, sizeof(enabled)))
			{
				RGBLOG(TAG"RGB_IOCTL_GET_ENABLED: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case RGB_IOCTL_READ_SENSORDATA:
			nxu8010_read_rgb_data(&value, R_LOW_DATA, R_HIGH_DATA);
			gRGBWData[0] = (int)value;
			nxu8010_read_rgb_data(&value, G_LOW_DATA, G_HIGH_DATA);
			gRGBWData[1] = (int)value;
			nxu8010_read_rgb_data(&value, B_LOW_DATA, B_HIGH_DATA);
			gRGBWData[2] = (int)value;
			nxu8010_read_rgb_data(&value, W_LOW_DATA, W_HIGH_DATA);
			gRGBWData[3] = (int)value;
			if(copy_to_user(argp, gRGBWData, sizeof(gRGBWData)))
			{
				RGBLOG(TAG"RGB_IOCTL_READ_SENSORDATA: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case RGB_IOCTL_SET_LUX:
			if(argp == NULL){
				RGBLOG(TAG"IO parameter pointer is NULL!\n");
				break;
			}
			if(copy_from_user(&lux, argp, sizeof(lux))){
				RGBLOG(TAG"RGB_IOCTL_SET_LUX failed.");
				return -EFAULT;
			}else{
				RGBLOG(TAG"RGB_IOCTL_SET_LUX lux=%d!\n",lux);
				gRGBWLux = lux;
			}
			break;

		case RGB_IOCTL_SET_CCT:
			if(argp == NULL){
				RGBLOG(TAG"IO parameter pointer is NULL!\n");
				break;
			}
			if(copy_from_user(&cct, argp, sizeof(cct))){
				RGBLOG(TAG"RGB_IOCTL_SET_CCT failed.");
				return -EFAULT;
			}else{
				RGBLOG(TAG"RGB_IOCTL_SET_CCT cct=%d!\n",cct);
				gRGBWCCT = cct;
			}
			break;

		case RGB_IOCTL_SET_REG:
			if(argp == NULL){
				RGBLOG(TAG"IO parameter pointer is NULL!\n");
				break;
			}
			if(copy_from_user(reg, argp, sizeof(reg))){
				RGBLOG(TAG"RGB_IOCTL_SET_REGISTER failed.");
				return -EFAULT;
			}else{
				RGBLOG(TAG"RGB_IOCTL_SET_REGISTER addr=0x%02x, data=0x%02x\n", reg[0], reg[1]);
				nxu8010_write((u8)reg[0], (u8)reg[1]);
			}
			break;

		case RGB_IOCTL_GET_CODE:
			//nxu8010_read_rgb_data(&value, R_LOW_DATA, R_HIGH_DATA);
			if ( nxu8010_read(CORRELATION_CODE, &read_buf) ) {
				RGBLOG(TAG"RGB_IOCTL_GET_CODE i2c read failed.");
				return -EFAULT;
			}

			code = (int)read_buf;

			if(copy_to_user(argp, &code, sizeof(code)))
			{
				RGBLOG(TAG"RGB_IOCTL_GET_CODE: copy to user failed\n");
				return -EFAULT;
			}
			break;

		case RGB_IOCTL_SET_ENABLED:
			if(argp == NULL){
				RGBLOG(TAG"IO parameter pointer is NULL!\n");
				break;
			}
			if(copy_from_user(&enabled, argp, sizeof(enabled))){
				RGBLOG(TAG"RGB_IOCTL_SET_ENABLED failed.");
				return -EFAULT;
			}else{
				RGBLOG(TAG"RGB_IOCTL_SET_ENABLED enabled=%d!\n", enabled);
				nxu8010_set_enable_rgb(enabled);
			}
			break;

		default:
			RGBLOG(TAG"not supported cmd= 0x%04x", cmd);
			return -ENOIOCTLCMD;
			break;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long nxu8010_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	void __user *arg64 = compat_ptr(arg);

	if(!file->f_op || !file->f_op->unlocked_ioctl){
		printk(KERN_ERR "file->f_op OR file->f_op->unlocked_ioctl is null!\n");
		return -ENOTTY;
	}

	switch(cmd){
		case RGB_IOCTL_GET_ENABLED:
			ret = file->f_op->unlocked_ioctl(file, RGB_IOCTL_GET_ENABLED, (unsigned long)arg64);
			if(ret < 0){
				printk(KERN_ERR "RGB_IOCTL_GET_ENABLED is failed!\n");
			}
			break;

		case RGB_IOCTL_READ_SENSORDATA:
			ret = file->f_op->unlocked_ioctl(file, RGB_IOCTL_READ_SENSORDATA, (unsigned long)arg64);
			if(ret < 0){
				printk(KERN_ERR "RGB_IOCTL_READ_SENSORDATA is failed!\n");
			}
			break;

		case RGB_IOCTL_SET_LUX:
			ret = file->f_op->unlocked_ioctl(file, RGB_IOCTL_SET_LUX, (unsigned long)arg64);
			if(ret < 0){
				printk(KERN_ERR "RGB_IOCTL_SET_LUX is failed!\n");
			}
			break;

		case RGB_IOCTL_SET_CCT:
			ret = file->f_op->unlocked_ioctl(file, RGB_IOCTL_SET_CCT, (unsigned long)arg64);
			if(ret < 0){
				printk(KERN_ERR "RGB_IOCTL_SET_CCT is failed!\n");
			}
			break;

		case RGB_IOCTL_SET_REG:
			ret = file->f_op->unlocked_ioctl(file, RGB_IOCTL_SET_REG, (unsigned long)arg64);
			if(ret < 0){
				printk(KERN_ERR "RGB_IOCTL_SET_REG is failed!\n");
			}
			break;

		case RGB_IOCTL_GET_CODE:
			ret = file->f_op->unlocked_ioctl(file, RGB_IOCTL_GET_CODE, (unsigned long)arg64);
			if(ret < 0){
				printk(KERN_ERR "RGB_IOCTL_GET_CODE is failed!\n");
			}
			break;

		case RGB_IOCTL_SET_ENABLED:
			ret = file->f_op->unlocked_ioctl(file, RGB_IOCTL_SET_ENABLED, (unsigned long)arg64);
			if(ret < 0){
				printk(KERN_ERR "RGB_IOCTL_SET_ENABLED is failed!\n");
			}
			break;

		default:
			RGBLOG(TAG"not supported cmd= 0x%04x", cmd);
			return -ENOIOCTLCMD;
			break;
	}
		return 0;
}
#endif

static int nxu8010_open(struct inode *inode, struct file *file)
{
	int ret = -1;

	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int nxu8010_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t store_reg(struct device_driver *drv, const char *buf, size_t count)
{
	int reg_addr = 0x0;
	int reg_val = 0x0;

	RGBLOG(TAG"buf=%s\n", buf);

	sscanf(buf, "0x%02x,0x%02x", &reg_addr, &reg_val);

	RGBLOG(TAG"addr=0x%02x, val=0x%02x\n", reg_addr, reg_val);
	nxu8010_write((u8)reg_addr, (u8)reg_val);

	return count;
}

static ssize_t show_reg(struct device_driver *drv, char *buf)
{
	RGBLOG(TAG"start\n");

	nxu8010_init();

	return 0;
}

static ssize_t store_r_low_data(struct device_driver *drv, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_r_low_data(struct device_driver *drv, char *buf)
{
	unsigned char read_buf[1] = {0};
	int len = 0;
	nxu8010_read(R_LOW_DATA, read_buf);
	RGBLOG(TAG"buf=0x%02x\n", (int)read_buf[0]);
	len = snprintf(buf, 4, "%x\n", read_buf[0]);
	return len;
}

static ssize_t store_r_high_data(struct device_driver *drv, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_r_high_data(struct device_driver *drv, char *buf)
{
	unsigned char read_buf[1] = {0};
	int len = 0;
	nxu8010_read(R_HIGH_DATA, read_buf);
	RGBLOG(TAG"buf=0x%02x\n", (int)read_buf[0]);
	//sprintf(buf, "%d\n", read_buf[0]);
	len = snprintf(buf, 4, "%x\n", read_buf[0]);
	return len;
}

static ssize_t store_g_low_data(struct device_driver *drv, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_g_low_data(struct device_driver *drv, char *buf)
{
	unsigned char read_buf[1] = {0};
	int len = 0;
	nxu8010_read(G_LOW_DATA, read_buf);
	RGBLOG(TAG"buf=0x%02x\n", (int)read_buf[0]);
	len = snprintf(buf, 4, "%x\n", read_buf[0]);
	return len;
}

static ssize_t store_g_high_data(struct device_driver *drv, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_g_high_data(struct device_driver *drv, char *buf)
{
	unsigned char read_buf[1] = {0};
	int len = 0;
	nxu8010_read(G_HIGH_DATA, read_buf);
	RGBLOG(TAG"buf=0x%02x\n", (int)read_buf[0]);
	len = snprintf(buf, 4, "%x\n", read_buf[0]);
	return len;
}

static ssize_t store_b_low_data(struct device_driver *drv, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_b_low_data(struct device_driver *drv, char *buf)
{
	unsigned char read_buf[1] = {0};
	int len = 0;
	nxu8010_read(B_LOW_DATA, read_buf);
	RGBLOG(TAG"buf=0x%02x\n", (int)read_buf[0]);
	len = snprintf(buf, 4, "%x\n", read_buf[0]);
	return len;
}

static ssize_t store_b_high_data(struct device_driver *drv, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_b_high_data(struct device_driver *drv, char *buf)
{
	unsigned char read_buf[1] = {0};
	int len = 0;
	nxu8010_read(B_HIGH_DATA, read_buf);
	RGBLOG(TAG"buf=0x%02x\n", (int)read_buf[0]);
	len = snprintf(buf, 4, "%x\n", read_buf[0]);
	return len;
}

static ssize_t store_w_low_data(struct device_driver *drv, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_w_low_data(struct device_driver *drv, char *buf)
{
	unsigned char read_buf[1] = {0};
	int len = 0;
	nxu8010_read(W_LOW_DATA, read_buf);
	RGBLOG(TAG"buf=0x%02x\n", (int)read_buf[0]);
	len = snprintf(buf, 4, "%x\n", read_buf[0]);
	return len;
}

static ssize_t store_w_high_data(struct device_driver *drv, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_w_high_data(struct device_driver *drv, char *buf)
{
	unsigned char read_buf[1] = {0};
	int len = 0;
	nxu8010_read(W_HIGH_DATA, read_buf);
	RGBLOG(TAG"buf=0x%02x\n", (int)read_buf[0]);
	len = snprintf(buf, 4, "%x\n", read_buf[0]);
	return len;
}

static ssize_t store_correlation_code(struct device_driver *drv, const char *buf, size_t count)
{
	return count;
}

static ssize_t show_correlation_code(struct device_driver *drv, char *buf)
{
	unsigned char read_buf[1] = {0};
	int len = 0;
	nxu8010_read(CORRELATION_CODE, read_buf);
	RGBLOG(TAG"buf=0x%02x\n", (int)read_buf[0]);
	len = snprintf(buf, 4, "%x\n", read_buf[0]);
	return len;
}
static DRIVER_ATTR(reg, S_IWUSR | S_IRUGO, show_reg, store_reg);

static DRIVER_ATTR(r_low_data, S_IWUSR | S_IRUGO, show_r_low_data, store_r_low_data);
static DRIVER_ATTR(r_high_data, S_IWUSR | S_IRUGO, show_r_high_data, store_r_high_data);
static DRIVER_ATTR(g_low_data, S_IWUSR | S_IRUGO, show_g_low_data, store_g_low_data);
static DRIVER_ATTR(g_high_data, S_IWUSR | S_IRUGO, show_g_high_data, store_g_high_data);
static DRIVER_ATTR(b_low_data, S_IWUSR | S_IRUGO, show_b_low_data, store_b_low_data);
static DRIVER_ATTR(b_high_data, S_IWUSR | S_IRUGO, show_b_high_data, store_b_high_data);
static DRIVER_ATTR(w_low_data, S_IWUSR | S_IRUGO, show_w_low_data, store_w_low_data);
static DRIVER_ATTR(w_high_data, S_IWUSR | S_IRUGO, show_w_high_data, store_w_high_data);
static DRIVER_ATTR(correlation_code, S_IWUSR | S_IRUGO, show_correlation_code, store_correlation_code);

static struct file_operations nxu8010_fops = {
	//.owner = THIS_MODULE,
	.open = nxu8010_open,
	.release = nxu8010_release,
	.unlocked_ioctl = nxu8010_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nxu8010_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice nxu8010_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "nxu8010",
    .fops = &nxu8010_fops,
};

static int nxu8010_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret_driver_file = 0;
	struct rgb_control_func ctl_func = {};
	unsigned char read_buf[1] = {0};

	RGBLOG(TAG"start\n");

	g_nxu8010_I2Cclient = client;

	mutex_init(&nxu8010_mutex);

	if(nxu8010_read(CORRELATION_CODE, read_buf)){
		printk("fail\n");
		return -1;
	}

	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_reg);
	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_r_low_data);
	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_r_high_data);
	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_g_low_data);
	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_g_high_data);
	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_b_low_data);
	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_b_high_data);
	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_w_low_data);
	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_w_high_data);
	ret_driver_file = driver_create_file(&(g_nxu8010_driver.driver), &driver_attr_correlation_code);

	ctl_func.set_enable_rgb = nxu8010_set_enable_rgb;
	ctl_func.get_enable_rgb = nxu8010_get_enable_rgb;
	ctl_func.get_data_lux = nxu8010_get_data_lux;
	ctl_func.get_data_r = nxu8010_get_data_r;
	ctl_func.get_data_g = nxu8010_get_data_g;
	ctl_func.get_data_b = nxu8010_get_data_b;
	ctl_func.get_data_w = nxu8010_get_data_w;
	ctl_func.get_data_cct = nxu8010_get_data_cct;
	ctl_func.get_chipinfo = nxu8010_get_chipinfo;
	ctl_func.rgb_do_calib = nxu8010_rgb_do_calib;

	if(misc_register(&nxu8010_device))
	{
		RGBLOG(TAG "nxu8010_device register failed\n");
	}

	if(rgb_misc_init(&ctl_func)){
		RGBLOG(TAG"rgb misc device init fail\n");
	}

	RGBLOG(TAG"Attached!!\n");

	return 0;
}

static int nxu8010_i2c_remove(struct i2c_client *client)
{
	return 0;
}

struct i2c_driver nxu8010_i2c_driver = {
	.probe = nxu8010_i2c_probe,
	.remove = nxu8010_i2c_remove,
	.driver.name = NXU8010_DRVNAME,
	.id_table = nxu8010_i2c_id,
};

static int nxu8010_probe(struct platform_device *pdev)
{	
	i2c_add_driver(&nxu8010_i2c_driver);
	return 0;
}

static int nxu8010_remove(struct platform_device *pdev)
{
	i2c_del_driver(&nxu8010_i2c_driver);
	return 0;
}

static int nxu8010_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int nxu8010_resume(struct platform_device *pdev)
{
	return 0;
}

/* platform structure */
static struct platform_driver g_nxu8010_driver = {
	.probe = nxu8010_probe,
	.remove = nxu8010_remove,
	.suspend = nxu8010_suspend,
	.resume = nxu8010_resume,
	.driver = {
		   .name = "nxu8010",
		   .owner = THIS_MODULE,
		   }
};

static struct platform_device g_nxu8010_device = {
    .name = "nxu8010",
    .id = 0,
    .dev = {}
};
static int __init nxu8010_i2c_init(void)
{
	i2c_register_board_info(LENS_I2C_BUSNUM, &nxu8010_board_info, 1);

	if(platform_device_register(&g_nxu8010_device)){
		RGBLOG(TAG"failed to register platform_device\n");
		return -ENODEV;
	}

	if (platform_driver_register(&g_nxu8010_driver)) {
		RGBLOG(TAG"failed to register platform_driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit nxu8010_i2c_exit(void)
{
	platform_driver_unregister(&g_nxu8010_driver);
}
module_init(nxu8010_i2c_init);
module_exit(nxu8010_i2c_exit);

MODULE_DESCRIPTION("nxu8010 module driver");
MODULE_AUTHOR("sjkim <sjkim@nexuschips.com>");
MODULE_LICENSE("GPL");
