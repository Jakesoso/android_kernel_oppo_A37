#include <rgb_core.h>

static struct rgb_control_func rgb_func = {};

static struct miscdevice rgb_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "rgb_sensor",
};

static ssize_t enable_rgb_show(struct device* dev,
                                 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", rgb_func.get_enable_rgb());
}

static ssize_t enable_rgb_store(struct device* dev,
                                 struct device_attribute *attr,const char *buf, size_t count)
{
	int en;
	RGBLOG("enter\n");
	sscanf(buf, "%d", &en);

	rgb_func.set_enable_rgb(en);
	return count;
}

static ssize_t als_lux_show(struct device* dev,
                                 struct device_attribute *attr, char *buf)
{
	u32 lux;
	rgb_func.get_data_lux(&lux);
	return snprintf(buf, PAGE_SIZE, "%d\n", lux);
}

static ssize_t als_r_show(struct device* dev,
                                 struct device_attribute *attr, char *buf)
{
	u16 r;
	rgb_func.get_data_r(&r);
	return snprintf(buf, PAGE_SIZE, "%d\n", r);
}

static ssize_t als_g_show(struct device* dev,
                                 struct device_attribute *attr, char *buf)
{
	u16 g;
	rgb_func.get_data_g(&g);
	return snprintf(buf, PAGE_SIZE, "%d\n", g);
}

static ssize_t als_b_show(struct device* dev,
                                 struct device_attribute *attr, char *buf)
{
	u16 b;
	rgb_func.get_data_b(&b);
	return snprintf(buf, PAGE_SIZE, "%d\n", b);
}

static ssize_t als_w_show(struct device* dev,
                                 struct device_attribute *attr, char *buf)
{
	u16 w;
	rgb_func.get_data_w(&w);
	return snprintf(buf, PAGE_SIZE, "%d\n", w);
}

static ssize_t als_cct_show(struct device* dev,
                                 struct device_attribute *attr, char *buf)
{
	u32 cct;
	rgb_func.get_data_cct(&cct);
	return snprintf(buf, PAGE_SIZE, "%d\n", cct);
}

static ssize_t chip_info_show(struct device* dev,
                                 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", rgb_func.get_chipinfo());
}

static ssize_t rgb_do_calib_show(struct device* dev,
                                 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", rgb_func.rgb_do_calib());
}

DEVICE_ATTR(enable_rgb, 0666, enable_rgb_show, enable_rgb_store);
DEVICE_ATTR(als_lux, 0444, als_lux_show, NULL);
DEVICE_ATTR(als_r, 0444, als_r_show, NULL);
DEVICE_ATTR(als_g, 0444, als_g_show, NULL);
DEVICE_ATTR(als_b, 0444, als_b_show, NULL);
DEVICE_ATTR(als_w, 0444, als_w_show, NULL);
DEVICE_ATTR(als_cct, 0444, als_cct_show, NULL);
DEVICE_ATTR(chip_info, 0444, chip_info_show, NULL);
DEVICE_ATTR(rgb_do_calib, 0444, rgb_do_calib_show, NULL);

static struct attribute *rgb_attributes[] = {
	&dev_attr_enable_rgb.attr,
	&dev_attr_als_lux.attr,
	&dev_attr_als_r.attr,
	&dev_attr_als_g.attr,
	&dev_attr_als_b.attr,
	&dev_attr_als_w.attr,
	&dev_attr_als_cct.attr,
	&dev_attr_chip_info.attr,
	&dev_attr_rgb_do_calib.attr,
	NULL
};

static struct attribute_group rgb_attribute_group = {
	.attrs = rgb_attributes
};

static int rgb_check_func(struct rgb_control_func *func)
{
	rgb_func.set_enable_rgb = func->set_enable_rgb;
	rgb_func.get_enable_rgb = func->get_enable_rgb;
	rgb_func.get_data_lux = func->get_data_lux;
	rgb_func.get_data_r = func->get_data_r;
	rgb_func.get_data_g = func->get_data_g;
	rgb_func.get_data_b = func->get_data_b;
	rgb_func.get_data_w = func->get_data_w;
	rgb_func.get_data_cct = func->get_data_cct;
	rgb_func.get_chipinfo = func->get_chipinfo;
	rgb_func.rgb_do_calib = func->rgb_do_calib;

	if (rgb_func.set_enable_rgb != NULL || rgb_func.get_data_lux != NULL ||
		rgb_func.get_data_r != NULL || rgb_func.get_data_g != NULL ||
		rgb_func.get_data_b != NULL || rgb_func.get_data_cct != NULL ||
		rgb_func.get_chipinfo != NULL || rgb_func.get_enable_rgb != NULL ||
		rgb_func.rgb_do_calib != NULL || rgb_func.get_data_w != NULL){

		return 0;
	}else{
		return -1;
	}
}

int rgb_misc_init(struct rgb_control_func *func)
{

    int err=0;

	if((err = rgb_check_func(func))){
		printk("[rgb_sensor]check func fail!!\n");
		return err;
	}

	if((err = misc_register(&rgb_device)))
	{
		printk("[rgb_sensor]unable to register rgb misc device!!\n");
		return err;
	}

	err = sysfs_create_group(&rgb_device.this_device->kobj,
			&rgb_attribute_group);
	if (err < 0){
	   printk("[rgb_sensor]unable to create rgb attribute file\n");
	}
	return err;
}






