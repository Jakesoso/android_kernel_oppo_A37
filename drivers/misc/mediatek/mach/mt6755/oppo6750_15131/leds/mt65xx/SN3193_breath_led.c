#if 0
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <mach/mt_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/earlysuspend.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/mt_pm_ldo.h>
#include <linux/miscdevice.h>
#else
#include <linux/kernel.h> 
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/mt_gpio.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <asm/sizes.h>
#include <linux/leds.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/earlysuspend.h>
#include <mach/mt_pm_ldo.h>
#endif

//#define BREATH_LED_DEMO

#define SLED_IOCTL_MAGIC 'l'

#define SLED_ENABLE                  _IOW(SLED_IOCTL_MAGIC, 0, unsigned) //enable led
#define SLED_DISABLE                 _IOW(SLED_IOCTL_MAGIC, 1, unsigned) //disable led
#define SLED_CONFIG_FEATURE      	 _IOW(SLED_IOCTL_MAGIC, 2, unsigned) //stop flash or start flah
#define SLED_SET_WORKMOD      		 _IOW(SLED_IOCTL_MAGIC, 3, unsigned) //set work mod
#define SLED_SET_BTEATHTIME  		 _IOW(SLED_IOCTL_MAGIC, 4, unsigned)//set breateh time T0~T4
#define SLED_SET_RED                 _IOW(SLED_IOCTL_MAGIC, 5, unsigned) //set red
#define SLED_SET_GREEEN              _IOW(SLED_IOCTL_MAGIC, 6, unsigned)//set green
#define SLED_SET_BLUE                _IOW(SLED_IOCTL_MAGIC, 7, unsigned) //set blue//#define SLED_SET_LASTTIME       _IOW(SLED_IOCTL_MAGIC, 8, unsigned) //set flahing time




static int  SN3193_read_reg(unsigned char raddr, unsigned char *rdata);
static int  SN3193_write_reg(unsigned char raddr, unsigned char *rdata);

static int SN3193_power(int on);
static int SN3193_SetBrightness(int color,u8 brightness);
static int SN3193_TurnOffOut_sled(void);
static int SN3193_upData_sled(void);   
static int SN3193_enable_sled( int on);
static int SN3193_config_feature_sled( u8 feature);
static int SN3193_workmod_sled( int rgb);
static int SN3193_setCurrent_sled( u8 cs);
static int SN3193_SetBreathTime_sled(u8 Ch, u8 T0,u8 T1,u8 T2,u8 T3,u8 T4);
static int SN3193_TimeUpdate_sled(void);
static int SN3193_TurnOnRGB_sled(void);
static int SN3193_TurnOnOut_sled(u8 Ch);
static int SN3193_open(struct inode *inode, struct file *file);
static long SN3193_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
u8 get_register_t(int MS);
static void lcds_set_brightness(struct led_classdev *led_cdev,enum led_brightness value);

static int SN3193_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int SN3193_remove(struct i2c_client *client);
static int Breathlight_probe(struct platform_device *pdev);
static int Breathlight_remove(struct platform_device *pdev); 

static struct regulator *ldo121;
static struct regulator *lvs5;
extern int get_pcb_version(void); 

#define SN3193_DRV_NAME			"sn3193"
#define SN3193_DRV_WRITE_ADDER	0xD0
#define SN3193_DRV_READ_ADDER	0xD1
#define SN3193_ENABLE_GPIO   	77
#define SN3193_SDA_PIN 			118 	//SDA pin GPIO define
#define SN3193_SCK_PIN 			119 	//SCK pin GPIO define

#define SN3193_POWER_ON 		1	//  power on
#define SN3193_POWER_OFF 		0   //  power off

#define SN3193_SOFT_CONTROL_ENABLE	 		1		// 1 =ENABLE 	// 00 Register   
#define SN3193_SOFT_CONTROL_DISABLE 		0		// 0 =DISENABLE 

#define SN3193_CONTROL_PWM_MODE	 			0		// 0 =PWM mode	// 02 Register   
#define SN3193_CONTROL_PROGRAM_MODE	 		1		// 1 =program mode

#define SN3193_CONTROL_CURRENT_42MA	 		0x00	// 0x00 =42ma	// 03 Register   
#define SN3193_CONTROL_CURRENT_10MA	 		0x04	// 0x00 =10ma	// 03 Register     
#define SN3193_CONTROL_CURRENT_5MA	 		0x08	// 0x00 =5ma	// 03 Register     
#define SN3193_CONTROL_CURRENT_30MA	 		0x0C	// 0x00 =30ma	// 03 Register    
#define SN3193_CONTROL_CURRENT_17MA	 		0x10	// 0x00 =17.5ma	// 03 Register     

#define RED_SLED 				0	//  red
#define GREEN_SLED 				1	//  green
#define BLUE_SLED 				2	//  blue

#define SN3193_DELAY 			0x10
#define SN3193_SDA_PIN 			1 //SDA pin GPIO define
#define SN3193_SCK_PIN 			2 //SCK pin GPIO define
#define SN3193_SDB_PIN 			3 //SDB pin GPIO define



static const struct i2c_device_id Breathlight_i2c_id[] = {
	{SN3193_DRV_NAME, 0},
	{ }
};
static struct i2c_board_info __initdata Breathlight_i2c_info ={ I2C_BOARD_INFO(SN3193_DRV_NAME, (SN3193_DRV_WRITE_ADDER>>1))};

#if 0
static struct miscdevice SN3193_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= SN3193,
	.fops	= &SN3193_fops,
};
#endif

struct SN3193_sled
{
	struct miscdevice SN3193_miscdev;
	struct i2c_client *i2c_dev;
};
struct sled_dev_sate
{
	struct i2c_client *gi2c_dev;
};
static struct sled_dev_sate *SN3193_sled_dev_sate;	//定义设备
static struct i2c_client *g_sn3193_client;

static int SN3193_read_reg(unsigned char raddr, unsigned char *rdata)
{
	int32_t rc = -EIO;
	unsigned char buf[8];
	
	if(!g_sn3193_client)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = i2c_master_send(g_sn3193_client,buf,1);
	rc |= i2c_master_recv(g_sn3193_client,buf,1);
	if (rc < 0) {
		pr_err("%s : read reg addr 0x%x failed!\n", __func__, raddr);
		return rc;
	}
	*rdata = buf[0];
	
	return rc;
}

static int SN3193_write_reg(unsigned char raddr, unsigned char *rdata)
{
	int32_t rc = -EIO;
	unsigned char buf[8];
	
	if(!g_sn3193_client)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	buf[1] = rdata;
	rc = i2c_master_send(g_sn3193_client,buf,2);
	if (rc < 0)
		pr_err("%s :failed, addr = 0x%x, val = 0x%x!\n", __func__, raddr, rdata);
	return rc;
}
static int SN3193_power(int on)
{
	if(on) 
	{
		//hwPowerOn(MT65XX_POWER_LDO_VGP4,VOL_1800,"BT_LED");
		msleep(250); 
		mt_set_gpio_mode(SN3193_ENABLE_GPIO, 0);	
		mt_set_gpio_dir(SN3193_ENABLE_GPIO, 1);	
		mt_set_gpio_out(SN3193_ENABLE_GPIO, 1);			
		printk("dengnanwei sn3193_i2c_probe: poweron \n");
	
	}
	else if (!on) 
	{

		//hwPowerDown(MT65XX_POWER_LDO_VGP4,"BT_LED");
		msleep(1);

		mt_set_gpio_mode(SN3193_ENABLE_GPIO, 0);	
		mt_set_gpio_dir(SN3193_ENABLE_GPIO, 1);	
		mt_set_gpio_out(SN3193_ENABLE_GPIO, 0);			
		printk("dengnanwei sn3193_i2c_probe: poweroff  \n");
	
	}
	return 0 ;
}

static int SN3193_enable_sled( int on)
{    
	int ret;   
	
	if(on)
	{        
		ret=SN3193_write_reg(0x00,0x20);    	
	}
	else
	{       
		// ret=SN3193_write_reg(client,0x2f,0x00);        
		ret=SN3193_write_reg(0x00,0x01);	
	}
	return ret;
}
static int SN3193_config_feature_sled( u8 feature)
{
	int ret=0;
	ret=SN3193_write_reg(0x01,feature);
	return 0;
}
static int SN3193_workmod_sled( int rgb)
{    
	int ret=0;    
	if(rgb)
	{       
		ret=SN3193_write_reg(0x02,0x20);	
	}
	else
	{	
		ret=SN3193_write_reg(0x02,0x00);	
	}	
	return ret;
}
static int SN3193_setCurrent_sled( u8 cs)
{    
	int ret=0;	
	ret=SN3193_write_reg(0x03,cs);	
	return ret;
}
static int SN3193_SetBrightness(int color,u8 brightness)
{	
	int ret=0;	
	SN3193_write_reg(0x04+color, brightness);	// 04:red 05:green 06:blue
	return ret;
}
static int SN3193_upData_sled(void)   //update 04~06
{       
	int ret=0;	
	ret=SN3193_write_reg(0x07,0x00);	
	return ret;
}
static int SN3193_SetBreathTime_sled(u8 Ch, u8 T0,u8 T1,u8 T2,u8 T3,u8 T4)
{	         
	int ret=0;	  

	switch(Ch)	  
	{		
		case 1:			
			SN3193_write_reg(0x0a,T0<<4);			//T0(time of holding off)			
			SN3193_write_reg(0x10,(T1<<5)|(T2<<1));	//T1&T2(time of ramping up and hold on	
			SN3193_write_reg(0x16,(T3<<5)|(T4<<1));  //T3&T4(time of ramping down and hold off)			
			break;		
		case 2:  			
			SN3193_write_reg(0x0b,T0<<4);				
			SN3193_write_reg(0x11,(T1<<5)|(T2<<1));   			
			SN3193_write_reg(0x17,(T3<<5)|(T4<<1));			
			break;		
		case 3:			
			SN3193_write_reg(0x0c,T0<<4);	  		  	
			SN3193_write_reg(0x12,(T1<<5)|(T2<<1));	     		      
			SN3193_write_reg(0x18,(T3<<5)|(T4<<1));				
			break;			
	}         
	return ret;
}
static int SN3193_TimeUpdate_sled(void)
{      
	int ret=0; 	

	ret=SN3193_write_reg(0x1c,0x00);	
	return ret;
}
static int SN3193_TurnOffOut_sled(void)
{     
	int ret=0;	
	SN3193_write_reg(0x1D,0x00);	// close
	return ret;
}
static int SN3193_TurnOnRGB_sled(void)
{    
	int ret=0;    

	SN3193_write_reg(0x1D, 0x07);	   
	return ret;
}
static int SN3193_TurnOnOut_sled(u8 Ch)
{    
	int ret=0;     
	SN3193_write_reg(0x1D, (1<<Ch));	    
	return ret;
}


static const struct file_operations SN3193_fops = 
{	
	.owner		= THIS_MODULE,	
	.open		= SN3193_open,	
	.unlocked_ioctl	= SN3193_ioctl,
};
static int SN3193_open(struct inode *inode, struct file *file)
{      
	int ret=0;	
	struct SN3193_sled *info = container_of(file->private_data,	struct SN3193_sled, SN3193_miscdev);	

	file->private_data = info;	
	pr_info("%s:sn3193 open\n",__func__);	
	
	return ret;
}

static long SN3193_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct SN3193_sled *info = file->private_data;
    unsigned int val;
	  /*
           t[0]:channel
           t[1]:T0
           t[2]:T1
           t[3]:T2
           t[4]:T3
           t[5]:T4
	  */
      u8 t[6];
      long r=0;

	switch (cmd) {
		case SLED_ENABLE:
			SN3193_enable_sled(1);
			SN3193_setCurrent_sled(SN3193_CONTROL_CURRENT_5MA);
		  	pr_info("%s:enable sled\n",__func__);
		 	break;
		case SLED_DISABLE:
			SN3193_TurnOffOut_sled();
		 	SN3193_enable_sled(0);
	       	pr_info("%s:disable sled\n",__func__);
		 	break;
		case SLED_SET_WORKMOD:
			if (copy_from_user(&val,(void __user *)arg, sizeof(unsigned int))) {
				r = -EFAULT;
			}
			pr_info("%s;set sled work mod ,val:%d\n",__func__,val);
	       	SN3193_workmod_sled(val);  //now we should set to val to 0
			break;
		case SLED_CONFIG_FEATURE:
			if (copy_from_user(&val,(void __user *)arg, sizeof(unsigned int))) {
			r = -EFAULT;
			}
			pr_info("%s;set stagnate mod,val:%d\n",__func__,val);
			SN3193_config_feature_sled(val);
			break;
		case SLED_SET_BTEATHTIME:
			if (copy_from_user(t,(void __user *)arg, sizeof(unsigned int))) {
				r = -EFAULT;
			}
		 	pr_info("%s;set stagnate modl:%d,%d,%d,%d,%d\n",__func__,t[1],t[2],t[3],t[4],t[5]);
			SN3193_SetBreathTime_sled(t[0],t[1],t[2],t[3],t[4],t[5]);
			SN3193_TimeUpdate_sled();
			break;
		case SLED_SET_RED :
			if (copy_from_user(&val,(void __user *)arg, sizeof(unsigned int))) {
				r = -EFAULT;
			}
			SN3193_TurnOnOut_sled(RED_SLED);
			SN3193_SetBrightness(RED_SLED,val);
	        SN3193_upData_sled();
	       	pr_info("%s:set sled red,val:%d\n",__func__,val);
			break;
		case SLED_SET_GREEEN:
			if (copy_from_user(&val,(void __user *)arg, sizeof(unsigned int))) {
				r = -EFAULT;
			}
		 	SN3193_TurnOnOut_sled(GREEN_SLED);
	        SN3193_SetBrightness(GREEN_SLED,val);
	        SN3193_upData_sled();
	        pr_info("%s:set sled green,val:%d\n",__func__,val);
			break;
      	case SLED_SET_BLUE:
	  		if (copy_from_user(&val,(void __user *)arg, sizeof(unsigned int))) {
				r = -EFAULT;
			}
	        SN3193_TurnOnOut_sled(BLUE_SLED);
			SN3193_SetBrightness(BLUE_SLED,val);
	        SN3193_upData_sled();
			pr_info("%s:set sled blue,val:%d\n",__func__,val);
			break;
		default:
			dev_err(&info->i2c_dev->dev, "Unknown ioctl 0x%x\n", cmd);
			r = -ENOIOCTLCMD;
		break;
	}
     return r;
}


int totalMS, onMS;
int color_R,color_G,color_B;
u8 get_register_t(int MS)
{
	u8 t;
	static int table[] = {130,260,520,1040,2080,4160,8320,16640};
	t = 0;

	while(MS >= table[t ++] && t < 8);
	t --;
	printk("yuyi----%s:   MS = %d: t = %d\n", __func__, MS, t);
	if( (t > 0) && (MS << 1) < (table[t] + table[t - 1]))
	{
		t --;
	}
	printk("yuyi----%s:   MS = %d: t = %d\n", __func__, MS, t);
	return t;
}
static ssize_t sled_grppwm_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
	printk("yuyi----%s:   buf = %s: count = %d\n", __func__, buf, count);
	onMS = value * totalMS / 255;
	return count;
}
static ssize_t sled_grpfreq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
	printk("yuyi----%s:   buf = %s: count = %d\n", __func__, buf, count);
	totalMS = value * 50;
	if(totalMS > 35000)
		totalMS = 35000;
	return count;
}
static ssize_t sled_blink_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
	u8 t123,t4;
	SN3193_setCurrent_sled(SN3193_CONTROL_CURRENT_5MA);
	if(value == 1 && totalMS && onMS)
	{
		SN3193_enable_sled(1);
		SN3193_TurnOnRGB_sled();	//turn on the RGB color
		SN3193_workmod_sled(1);		//select the program mode, for breath effect.
		SN3193_upData_sled();		//turn on the light
		t123 = get_register_t(onMS / 3);
		t4 = get_register_t(totalMS - onMS) + 1;
		SN3193_SetBreathTime_sled(1,0,t123,t123 + 1,t123, t4);
		SN3193_SetBreathTime_sled(2,0,t123,t123 + 1,t123, t4);
		SN3193_SetBreathTime_sled(3,0,t123,t123 + 1,t123, t4);
		SN3193_TimeUpdate_sled();	//start breath		
	}
	else
	{
		printk("yuyi----%s:   color_R = %d: color_G = %d: color_B = %d,\n", __func__, color_R, color_G,color_B);
		if(color_R + color_G + color_B == 0)
		{
			SN3193_TurnOffOut_sled();
			SN3193_enable_sled(0);
		}
		else
		{
			SN3193_enable_sled(1);
			SN3193_TurnOnRGB_sled();	//turn on the RGB color
			SN3193_workmod_sled(0); 	//select the RGB mode, 
			SN3193_upData_sled();		//turn on the light
		}
	}
	return count;
}
static ssize_t sled_charged(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long value = simple_strtoul(buf, NULL, 10);
	printk("yuyi----%s:   buf = %s: count = %d\n", __func__, buf, count);

	
	if(value > 200 ) 
	{
		color_G = 255;
		color_B = 255;
		color_R = 255;
	
		SN3193_SetBrightness(RED_SLED,color_R);
		SN3193_SetBrightness(GREEN_SLED,color_G);
		SN3193_SetBrightness(BLUE_SLED,color_B);
	
		SN3193_enable_sled(1);
		SN3193_TurnOnRGB_sled();	//turn on the RGB color
		SN3193_workmod_sled(0); 	//select the RGB mode, 
		SN3193_upData_sled();		//turn on the light
	
		return count;
	} 
	
	else if((value >= 100) &&(value <= 200))
	{
	//	value = 100;
	
		color_B = 255;
		color_G = 0;
		color_R = 0;
	
		SN3193_SetBrightness(RED_SLED,color_R);
		SN3193_SetBrightness(GREEN_SLED,color_G);
		SN3193_SetBrightness(BLUE_SLED,color_B);
	
		SN3193_enable_sled(1);
		SN3193_TurnOnRGB_sled();	//turn on the RGB color
		SN3193_workmod_sled(0); 	//select the RGB mode, 
		SN3193_upData_sled();		//turn on the light
	
		return count;
	} 
	else if( (value < 100 )&&( value > 10) ) 
	{
		color_G = 255;
		color_B = 0;
		color_R = 0;
	
		SN3193_SetBrightness(RED_SLED,color_R);
		SN3193_SetBrightness(GREEN_SLED,color_G);
		SN3193_SetBrightness(BLUE_SLED,color_B);
	
		SN3193_enable_sled(1);
		SN3193_TurnOnRGB_sled();	//turn on the RGB color
		SN3193_workmod_sled(0); 	//select the RGB mode, 
		SN3193_upData_sled();		//turn on the light
	
		return count;
	} 
	
	else if (( value <=10 ) &&(value >0))
	{
		color_B = 0;
		color_G = 0;
		color_R = 255;
	
		SN3193_SetBrightness(RED_SLED,color_R);
		SN3193_SetBrightness(GREEN_SLED,color_G);
		SN3193_SetBrightness(BLUE_SLED,color_B);
	
		SN3193_enable_sled(1);
		SN3193_TurnOnRGB_sled();	//turn on the RGB color
		SN3193_workmod_sled(0); 	//select the RGB mode, 
		SN3193_upData_sled();		//turn on the light
		return count;
	} 
	else 
	{ 
		SN3193_TurnOffOut_sled();
		SN3193_enable_sled(0);	
		return count;
	}
}

static DEVICE_ATTR(charged, S_IWUSR | S_IRUGO, NULL, sled_charged);
static DEVICE_ATTR(grppwm, S_IWUSR | S_IRUGO, NULL, sled_grppwm_store);
static DEVICE_ATTR(grpfreq, S_IWUSR | S_IRUGO, NULL, sled_grpfreq_store);
static DEVICE_ATTR(blink, S_IWUSR | S_IRUGO, NULL, sled_blink_store);

static struct attribute *blink_attributes[] = {
	&dev_attr_grppwm.attr,
	&dev_attr_grpfreq.attr,
	&dev_attr_blink.attr,
	&dev_attr_charged.attr,

	NULL
};

static const struct attribute_group blink_attr_group = {
	.attrs = blink_attributes,
};
static void leds_set_brightness(struct led_classdev *led_cdev,enum led_brightness value)
{
	//printk("huyu----%s: light the lcd  ", __func__);
	if(!strcmp(led_cdev->name, "red"))
	{
		printk("yuyi----%s: light the red ,value = %d", __func__, value);
		//SN3193_TurnOnOut_sled(RED_SLED);
		SN3193_SetBrightness(RED_SLED,value);
		color_R = value;
	}
	if(!strcmp(led_cdev->name, "green"))
	{
		//printk("huyu----%s: light the green ,value = %d", __func__, value);
		//SN3193_TurnOnOut_sled(GREEN_SLED);
		SN3193_SetBrightness(GREEN_SLED,value);
		color_G = value;
	}
	if(!strcmp(led_cdev->name, "blue"))
	{
		printk("yuyi----%s: light the blue ,value = %d", __func__, value);
		//SN3193_TurnOnOut_sled(BLUE_SLED);
		SN3193_SetBrightness(BLUE_SLED,value);
		color_B = value;
	}
}
static struct led_classdev SN3193_lcds[] = {
	{
		.name		= "red",
		.brightness_set = leds_set_brightness,
	},
	{
		.name		= "green",
		.brightness_set = leds_set_brightness,
	},
	{
		.name		= "blue",
		.brightness_set = leds_set_brightness,
	},
};

static struct i2c_driver Breathlight_i2c_driver = {
	.id_table = Breathlight_i2c_id,
	.probe	= SN3193_probe,
	.remove = SN3193_remove,
	.driver = {
		.name = SN3193_DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int SN3193_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	
	int ret = 0;	
	int i;    
	struct SN3193_sled *sn3193_sled_dev;	  

	unsigned char temp;

	//========================================
	// STEP1: poweron   check I2C
	//=======================================
	SN3193_power(SN3193_POWER_ON);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{		
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);		
		return  -ENODEV;		
	}	
	
	//========================================
	// STEP2:  Application memory
	//=======================================	   
	g_sn3193_client = client;	   
	sn3193_sled_dev = kzalloc(sizeof(*sn3193_sled_dev), GFP_KERNEL);
	if(sn3193_sled_dev == NULL) 
	{		
		dev_err(&client->dev,"failed to allocate memory for module data\n");		
		ret = -ENOMEM;		
		goto err_exit;
	}		
	memset(sn3193_sled_dev,0,sizeof(*sn3193_sled_dev));
	printk("%s:sn3193 probe\n",__func__);

	//========================================
	// STEP3:  Sign up complex equipment
	//=======================================	   
	sn3193_sled_dev->i2c_dev   = client;	
	sn3193_sled_dev->SN3193_miscdev.minor = MISC_DYNAMIC_MINOR;	
	sn3193_sled_dev->SN3193_miscdev.name = "SN3193";	
	sn3193_sled_dev->SN3193_miscdev.fops = &SN3193_fops;
	ret = misc_register(&sn3193_sled_dev->SN3193_miscdev);	
	if(ret) 
	{		
		pr_err("%s : misc_register failed\n", __FILE__);		
		goto err_misc_register;	
	}

	//========================================
	// STEP4:  Sign up led Subsystem
	//=======================================	   
#if 1	
	for(i= 0; i< 3; i++)	
	{		
		if (led_classdev_register(&client->dev, &SN3193_lcds[i]))		
		{			
			printk(KERN_ERR "led_classdev_register failed of SN3193_lcds!\n");			
			goto err_classdev_register;		
		}	
	}		
	printk("dengnanwei_able to probe fops  6.\n");
#endif

	//========================================
	// STEP5:  Create the interface
	//=======================================	   
	ret = sysfs_create_group(&client->dev.kobj, &blink_attr_group);	
	if(ret) 
	{		
		pr_err( "%s : sysfs_create_group failed!\n", __func__);		
		goto err_group_register;	
	}
	i2c_set_clientdata(client, sn3193_sled_dev);

	//========================================
	// STEP6:  Register INIT
	//=======================================	   
#ifdef BREATH_LED_DEMO 
	SN3193_enable_sled(SN3193_SOFT_CONTROL_ENABLE);   // ret=SN3193_write_reg(0x00,0x20); 
	ret=SN3193_read_reg(0x00, &temp);
	printk("[kernel] dengnanwei read control reg0 =0x%x\n", temp);
	
	SN3193_config_feature_sled(0);  						// 	ret=SN3193_write_reg(0x01,0x00);
	SN3193_workmod_sled(SN3193_CONTROL_PWM_MODE); 			// 	ret=SN3193_write_reg(0x02,0x00);
	SN3193_setCurrent_sled(SN3193_CONTROL_CURRENT_17MA);   	// 	ret=SN3193_write_reg(0x03,0x10);
	SN3193_TurnOnRGB_sled();								// 	ret=SN3193_write_reg(0x1D,0x01);ret=SN3193_write_reg(0x1D,0x02);ret=SN3193_write_reg(0x1D,0x04);	
	SN3193_SetBrightness(RED_SLED,255);		//ret=SN3193_write_reg(0x04,255); 
	SN3193_SetBrightness(GREEN_SLED,255);	//ret=SN3193_write_reg(0x05,255); 
	SN3193_SetBrightness(BLUE_SLED,255);	//ret=SN3193_write_reg(0x06,255); 
	SN3193_upData_sled();					//ret=SN3193_write_reg(0x07,0x00);
#endif

#ifndef BREATH_LED_DEMO 	
	printk("[kernel] dengnanwei close led \n", temp);
	//close, light as need	
	SN3193_SetBrightness(RED_SLED,0);	
	SN3193_SetBrightness(GREEN_SLED,0);	
	SN3193_SetBrightness(BLUE_SLED,0);	
	SN3193_TurnOffOut_sled();	
	SN3193_upData_sled();	
	SN3193_enable_sled(SN3193_SOFT_CONTROL_DISABLE);	
	SN3193_config_feature_sled(4);		//dvt not breath
#endif
	/**************************test*********************/	
	//enable sled	
	//SN3193_enable_sled(1);       
	//SN3193_config_feature_sled(0x00);	
	//SN3193_workmod_sled(1);	
	//SN3193_setCurrent_sled(0x01);    
	//SN3193_enable_diff_color_sled(BLUE_SLED);        
	//mod_timer(&SN3193_sled_dev_sate->gsled_last_timer,jiffies+5*HZ);       
	/**************************test******************/	
	return 0;
	
#if 1
err_group_register:	
	for(i = 0; i < 3; i ++ )		
		led_classdev_unregister(&SN3193_lcds[i]);
err_classdev_register:	
		misc_deregister(&sn3193_sled_dev->SN3193_miscdev);
err_misc_register:	
		kfree(sn3193_sled_dev);
#endif
err_exit:	
	SN3193_power(SN3193_POWER_OFF);
	return ret;
}

static int SN3193_remove(struct i2c_client *client)
{	
	struct SN3193_sled * sn3193_sled_dev;	
	int i;	

	sn3193_sled_dev = i2c_get_clientdata(client);	
	sysfs_remove_group(&client->dev.kobj, &blink_attr_group);	
#if 1
	for(i = 0; i < 3; i ++ )		
		led_classdev_unregister(&SN3193_lcds[i]);	
	misc_deregister(&sn3193_sled_dev->SN3193_miscdev);	
#endif	
	kfree(sn3193_sled_dev);		
	kfree(SN3193_sled_dev_sate);	
	return 0;
}


static struct platform_driver Breathlight_platform_driver =
{
    .probe      = Breathlight_probe,
    .remove     = Breathlight_remove,
    .driver     = {
        .name = "sn3193-platform",
    },
};


static struct platform_device Breathlight_platform_device = {
    .name = SN3193_DRV_NAME,
    .id = 0,
    .dev = {
    }
};
static int Breathlight_probe(struct platform_device *pdev) 
{
    if(i2c_add_driver(&Breathlight_i2c_driver)!=0) {
        printk("unable to add i2c driver.\n");
        return -1;
    }	
    return 0;
}
static int Breathlight_remove(struct platform_device *pdev) 
{
   i2c_del_driver(&Breathlight_i2c_driver);
}

static int __init SN3193_dev_init(void)
{
	i2c_register_board_info(0, &Breathlight_i2c_info, 1);  // 
	if(i2c_add_driver(&Breathlight_i2c_driver)!=0) 
	{
		printk("unable to add i2c driver.\n");
		return -1;	
		}
	printk("dengnanwei_able to register Breathlight platform driver.\n");
	return 0;
}


static void __exit SN3193_dev_exit(void)
{
  
    i2c_del_driver(&Breathlight_i2c_driver);
}

/*****************************************************************************/
module_init(SN3193_dev_init);
module_exit(SN3193_dev_exit);

MODULE_AUTHOR("yanghai");
MODULE_DESCRIPTION("Breathlight control Driver");
MODULE_LICENSE("GPL");
