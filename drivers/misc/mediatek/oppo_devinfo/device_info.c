/**
 * Copyright 2008-2013 OPPO Mobile Comm Corp., Ltd, All rights reserved.
 * VENDOR_EDIT:
 * FileName:devinfo.c
 * ModuleName:devinfo
 * Author: wangjc
 * Create Date: 2013-10-23
 * Description:add interface to get device information.
 * History:
   <version >  <time>  <author>  <desc>
   1.0		2013-10-23	wangjc	init
   2.0      2015-04-13  hantong modify as platform device  to support diffrent configure in dts
*/

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <soc/oppo/device_info.h>
#include <soc/oppo/oppo_project.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "../../../../fs/proc/internal.h"
#include <mach/mt_gpio.h> 


#define DEVINFO_NAME "devinfo"

static struct of_device_id devinfo_id[] = {
	{.compatible = "oppo-devinfo",},
	{},
};

struct devinfo_data { 
	struct platform_device *devinfo;
	int hw_id1_gpio;
	int hw_id2_gpio;
	int hw_id3_gpio;
	int sub_hw_id1;
	int sub_hw_id2;
};

static struct proc_dir_entry *parent = NULL;

static void *device_seq_start(struct seq_file *s, loff_t *pos)
{    
	static unsigned long counter = 0;    
	if ( *pos == 0 ) {        
		return &counter;   
	}else{
		*pos = 0; 
		return NULL;
	}
}

static void *device_seq_next(struct seq_file *s, void *v, loff_t *pos)
{  
	return NULL;
}

static void device_seq_stop(struct seq_file *s, void *v)
{
	return;
}

static int device_seq_show(struct seq_file *s, void *v)
{
	struct proc_dir_entry *pde = s->private;
	struct manufacture_info *info = pde->data;
	if(info)
	  seq_printf(s, "Device version:\t\t%s\nDevice manufacture:\t\t%s\n",
		     info->version,	info->manufacture);	
	return 0;
}

static struct seq_operations device_seq_ops = {
	.start = device_seq_start,
	.next = device_seq_next,
	.stop = device_seq_stop,
	.show = device_seq_show
};

static int device_proc_open(struct inode *inode,struct file *file)
{
	int ret = seq_open(file,&device_seq_ops);
	pr_err("%s is called\n",__func__);
	
	if(!ret){
		struct seq_file *sf = file->private_data;
		sf->private = PDE(inode);
	}
	
	return ret;
}
static const struct file_operations device_node_fops = {
	.read =  seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
	.open = device_proc_open,
	.owner = THIS_MODULE,
};

int register_device_proc(char *name, char *version, char *manufacture)
{
	struct proc_dir_entry *d_entry;
	struct manufacture_info *info;

	if(!parent) {
		parent =  proc_mkdir ("devinfo", NULL);
		if(!parent) {
			pr_err("can't create devinfo proc\n");
			return -ENOENT;
		}
	}

	info = kzalloc(sizeof *info, GFP_KERNEL);
	info->version = version;
	info->manufacture = manufacture;
	d_entry = proc_create_data (name, S_IRUGO, parent, &device_node_fops, info);
	if(!d_entry) {
		pr_err("create %s proc failed.\n", name);
		kfree(info);
		return -ENOENT;
	}
	return 0;
}

static void dram_type_add(void)
{
	struct manufacture_info dram_info;
	#if 0
	int *p = NULL;
	if(p)
	{
		switch(*p){
			case DRAM_TYPE0:
				dram_info.version = "EDB8132B3PB-1D-F FBGA";
				dram_info.manufacture = "ELPIDA";
				break;
			case DRAM_TYPE1:
				dram_info.version = "EDB8132B3PB-1D-F FBGA";
				dram_info.manufacture = "ELPIDA";
				break;
			case DRAM_TYPE2:
				dram_info.version = "EDF8132A3PF-GD-F FBGA";
				dram_info.manufacture = "ELPIDA";
				break;
			case DRAM_TYPE3:
				dram_info.version = "K4E8E304ED-AGCC FBGA";
				dram_info.manufacture = "SAMSUNG";
				break;
			default:
				dram_info.version = "unknown";
				dram_info.manufacture = "unknown";
		}

	}else{
		dram_info.version = "unknown";
		dram_info.manufacture = "unknown";
	}
	#else
	dram_info.version = "unknown";
	dram_info.manufacture = "unknown";
	#endif
	register_device_proc("ddr", dram_info.version, dram_info.manufacture);
}


static void mainboard_verify(void)
{
	struct manufacture_info mainboard_info;
	int hw_opreator_version = 0;

	switch(get_project()) {
		case OPPO_15111:
		case OPPO_15112:
		case OPPO_15113:
			switch(get_PCB_Version()) {
				case HW_VERSION__10:		
					mainboard_info.version ="15111_DVT";
					//sprintf(mainboard_info.manufacture,"%d-SA",hw_opreator_version);
					mainboard_info.manufacture = "OPERATOR_ALL_TELECOM_CARRIER";
					break;
				case HW_VERSION__11:	
					mainboard_info.version = "15111";
					//sprintf(mainboard_info.manufacture,"%d-SB",hw_opreator_version);
					mainboard_info.manufacture = "OPERATOR_OPEN_MARKET";
					break;
				case HW_VERSION__12:
					mainboard_info.version = "15112";
					//sprintf(mainboard_info.manufacture,"%d-SC",hw_opreator_version);
					mainboard_info.manufacture = "OPERATOR_CHINA_MOBILE";
					break;
				case HW_VERSION__13:
					mainboard_info.version = "15113";
					//sprintf(mainboard_info.manufacture,"%d-SD",hw_opreator_version);
		            mainboard_info.manufacture = "OPERATOR_ALL_TELECOM_CARRIER";
					break;
				case HW_VERSION__14:
					mainboard_info.version = "15114";
					//sprintf(mainboard_info.manufacture,"%d-SE",hw_opreator_version);
					mainboard_info.manufacture = "OPERATOR_ALL_MOBILE_CARRIER";
					break;
				default:	
					mainboard_info.version = "UNKOWN";
					//sprintf(mainboard_info.manufacture,"%d-UNKOWN",hw_opreator_version);
					mainboard_info.manufacture = "UNKOWN";
					break;
			}
			break;
		default:
		mainboard_info.version = "UNKOWN";
		mainboard_info.manufacture = "UNKOWN";
		break;
	}
	
	register_device_proc("mainboard", mainboard_info.version, mainboard_info.manufacture);
}

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2014.4.13 for 14027
static void pa_verify(void)
{
	struct manufacture_info pa_info;

	switch(get_Modem_Version()) {
		case 0:		
			pa_info.version = "0";
			pa_info.manufacture = "RFMD PA";
			break;
		case 1:	
			pa_info.version = "1";
			pa_info.manufacture = "SKY PA";
			break;
		case 3:
			pa_info.version = "3";
			pa_info.manufacture = "AVAGO PA";
			break;
		default:	
			pa_info.version = "UNKOWN";
			pa_info.manufacture = "UNKOWN";
	}
			
	register_device_proc("pa", pa_info.version, pa_info.manufacture);

}
#endif /*VENDOR_EDIT*/	

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG, 2015/12/31, Add jsut for 15113
#define OPPO_SMALLBOARD_ID1  		GPIO16
static void smallboard_verify(void)
{
	int id1 = 0;
  	char *oppo_smallboard = "demestinc";
	struct manufacture_info smallboard_info;
	
	id1 = mt_get_gpio_in(OPPO_SMALLBOARD_ID1);
	
	printk("id1 = %d\n",id1);
	if(id1 == 1)
		oppo_smallboard = "demestinc";
	else
		oppo_smallboard = "overseas";

	
	smallboard_info.manufacture = oppo_smallboard;
	smallboard_info.version ="MTK";
	register_device_proc("smallboard", smallboard_info.version, smallboard_info.manufacture);
}

#endif /*VENDOR_EDIT*/

static int devinfo_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct devinfo_data *devinfo_data = NULL;
	devinfo_data = kzalloc(sizeof(struct devinfo_data), GFP_KERNEL);
	if( devinfo_data == NULL ) {
		pr_err("devinfo_data kzalloc failed\n");
		ret = -ENOMEM;
		return ret;
	}
	 
	/*parse_dts*/
	devinfo_data->devinfo = pdev; 
	/*end of parse_dts*/
	
	if(!parent) {
		parent =  proc_mkdir ("devinfo", NULL);
		if(!parent) {
			pr_err("can't create devinfo proc\n");
			ret = -ENOENT;
		}
	}
	
	/*Add devinfo for some devices*/
	pa_verify();
	//dram_type_add();
	mainboard_verify();
	smallboard_verify();
	/*end of Adding devinfo for some devices*/
	return ret;
}

static int devinfo_remove(struct platform_device *dev)
{
	remove_proc_entry(DEVINFO_NAME, NULL);
	return 0;
}

static struct platform_driver devinfo_platform_driver = {
	.probe = devinfo_probe,
	.remove = devinfo_remove,
	.driver = {
		.name = DEVINFO_NAME,
		.of_match_table = devinfo_id,
	},
};

module_platform_driver(devinfo_platform_driver);

MODULE_DESCRIPTION("OPPO device info");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wangjc <wjc@oppo.com>");
