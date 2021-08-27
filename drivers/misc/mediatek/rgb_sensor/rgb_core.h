#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define RGBDEBUG
#ifdef RGBDEBUG
#define RGBLOG(format, args...) printk(KERN_ERR "[rgb] %s :"format,__func__,##args)
#else
#define RGBLOG(format, ...)
#endif

struct rgb_control_func
{
	int (*set_enable_rgb)(int en);
	int (*get_enable_rgb)(void);
	int (*get_data_lux)(u32 *value);
	int (*get_data_r)(u16 *value);
	int (*get_data_g)(u16 *value);
	int (*get_data_b)(u16 *value);
	int (*get_data_w)(u16 *value);
	int (*get_data_cct)(u32 *value);
	char* (*get_chipinfo)(void);
	int (*rgb_do_calib)(void);
};

int rgb_misc_init(struct rgb_control_func *func);