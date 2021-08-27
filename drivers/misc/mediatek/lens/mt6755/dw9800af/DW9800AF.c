/*
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "DW9800AF.h"
#include "../camera/kd_camera_hw.h"
#include <linux/xlog.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

// in K2, main=3, sub=main2=1
#define LENS_I2C_BUSNUM 2
static struct i2c_board_info kd_lens_dev __initdata = { I2C_BOARD_INFO("DW9800AF", 0x18) };


#define DW9800AF_DRVNAME "DW9800AF"
#define DW9800AF_VCM_WRITE_ID           0x18

#define DW9800AF_DEBUG
#ifdef DW9800AF_DEBUG
#define DW9800AFDB printk
#else
#define DW9800AFDB(x, ...)
#endif
typedef enum manu_id
{
 IC_9718 = 0,
 IC_9800,
 IC_MAX,
}MID;
static MID ic_diver = IC_MAX;

static spinlock_t g_DW9800AF_SpinLock;

static struct i2c_client * g_pstDW9800AF_I2Cclient = NULL;

static dev_t g_DW9800AF_devno;
static struct cdev * g_pDW9800AF_CharDrv = NULL;
static struct class *actuator_class = NULL;

static int  g_s4DW9800AF_Opened = 0;
static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4DW9800AF_INF = 0;
static unsigned long g_u4DW9800AF_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition   = 0;

static int g_sr = 3;

static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[1] = { (char)(a_u2Addr) };
	i4RetValue = i2c_master_send(g_pstDW9800AF_I2Cclient, puReadCmd, 1);
	if (i4RetValue != 1) {
	    DW9800AFDB(" I2C write failed!!%d\n",i4RetValue);
	    return -1;
	}
	/*  */
	i4RetValue = i2c_master_recv(g_pstDW9800AF_I2Cclient, (char *)a_puBuff, 1);
	if (i4RetValue != 1) {
	    DW9800AFDB(" I2C read failed!! \n");
	    return -1;
	}

	return 0;
}

u8 read_data(u8 addr)
{
	u8 get_byte=0;
    i2c_read( addr ,&get_byte);
    DW9800AFDB("[DW9800AF]  get_byte %d \n",  get_byte);
    return get_byte;
}


static int s4DW9800AF_ReadReg(unsigned short * a_pu2Result)
{
    //int  i4RetValue = 0;
    //char pBuff[2];
    //9800
    //*a_pu2Result = (read_data(0x02) << 8) + (read_data(0x03)&0xff);
    //9800
	*a_pu2Result = (read_data(0x03) << 8) + (read_data(0x04)&0xff);
    DW9800AFDB("[DW9800AF]  s4DW9800AF_ReadReg %d \n",  *a_pu2Result);
    return 0;
}


static int s4DW9800AF_WriteReg(u16 a_u2Data)
{
    int  i4RetValue = 0;
     //9800
    //char puSendCmd[3] = {0x02,(char)(a_u2Data >> 8) , (char)(a_u2Data & 0xFF)};
	 //9800
    char puSendCmd[3] = {0x03,(char)(a_u2Data >> 8) , (char)(a_u2Data & 0xFF)};
    DW9800AFDB("[DW9800AF]  write %d \n",  a_u2Data);
	
	g_pstDW9800AF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstDW9800AF_I2Cclient, puSendCmd, 3);
	
    if (i4RetValue < 0) 
    {
        DW9800AFDB("[DW9800AF] I2C send failed!! \n");
        return -1;
    }

	return 0;
}

inline static int getDW9800AFInfo(__user stDW9800AF_MotorInfo * pstMotorInfo)
{
	stDW9800AF_MotorInfo stMotorInfo;
	stMotorInfo.u4MacroPosition = g_u4DW9800AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4DW9800AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = TRUE;

	if (g_i4MotorStatus == 1)	{stMotorInfo.bIsMotorMoving = 1;}
	else						{stMotorInfo.bIsMotorMoving = 0;}

	if (g_s4DW9800AF_Opened >= 1)	{stMotorInfo.bIsMotorOpen = 1;}
	else						{stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stDW9800AF_MotorInfo)))
    {
        DW9800AFDB("[DW9800AF] copy to user failed when getting motor information \n");
    }

    return 0;
}

inline static int moveDW9800AF(unsigned long a_u4Position)
{
    int ret = 0;
    
    if((a_u4Position > g_u4DW9800AF_MACRO) || (a_u4Position < g_u4DW9800AF_INF))
    {
        DW9800AFDB("[DW9800AF] out of range \n");
        return -EINVAL;
    }

    if (g_s4DW9800AF_Opened == 1)
    {
        unsigned short InitPos;
        ret = s4DW9800AF_ReadReg(&InitPos);
	    
        if(ret == 0)
        {
            DW9800AFDB("[DW9800AF] Init Pos %6d \n", InitPos);
			
			spin_lock(&g_DW9800AF_SpinLock);
            g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(&g_DW9800AF_SpinLock);
        }
        else
        {		
			spin_lock(&g_DW9800AF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(&g_DW9800AF_SpinLock);
		}

		spin_lock(&g_DW9800AF_SpinLock);
		g_s4DW9800AF_Opened = 2;
		spin_unlock(&g_DW9800AF_SpinLock);

	}

    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(&g_DW9800AF_SpinLock);	
        g_i4Dir = 1;
        spin_unlock(&g_DW9800AF_SpinLock);	
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(&g_DW9800AF_SpinLock);	
        g_i4Dir = -1;
        spin_unlock(&g_DW9800AF_SpinLock);			
    }
    else	
	{return 0;}

	spin_lock(&g_DW9800AF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(&g_DW9800AF_SpinLock);

	/* DW9800AFDB("[DW9800AF] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */

            spin_lock(&g_DW9800AF_SpinLock);
            g_sr = 3;
            g_i4MotorStatus = 0;
            spin_unlock(&g_DW9800AF_SpinLock);	
		
            if(s4DW9800AF_WriteReg((unsigned short)g_u4TargetPosition) == 0)
            {
                spin_lock(&g_DW9800AF_SpinLock);		
                g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
                spin_unlock(&g_DW9800AF_SpinLock);				
            }
            else
            {
                DW9800AFDB("[DW9800AF] set I2C failed when moving the motor \n");			
                spin_lock(&g_DW9800AF_SpinLock);
                g_i4MotorStatus = -1;
                spin_unlock(&g_DW9800AF_SpinLock);				
            }

	return 0;
}

inline static int setDW9800AFInf(unsigned long a_u4Position)
{
	spin_lock(&g_DW9800AF_SpinLock);
	g_u4DW9800AF_INF = a_u4Position;
	spin_unlock(&g_DW9800AF_SpinLock);
	return 0;
}

inline static int setDW9800AFMacro(unsigned long a_u4Position)
{
	spin_lock(&g_DW9800AF_SpinLock);
	g_u4DW9800AF_MACRO = a_u4Position;
	spin_unlock(&g_DW9800AF_SpinLock);
	return 0;
}

////////////////////////////////////////////////////////////////
static long DW9800AF_Ioctl(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch(a_u4Command)
    {
        case DW9800AFIOC_G_MOTORINFO :
            i4RetValue = getDW9800AFInfo((__user stDW9800AF_MotorInfo *)(a_u4Param));
        break;

        case DW9800AFIOC_T_MOVETO :
			i4RetValue = moveDW9800AF(a_u4Param);
        break;
 
        case DW9800AFIOC_T_SETINFPOS :
            i4RetValue = setDW9800AFInf(a_u4Param);
        break;

        case DW9800AFIOC_T_SETMACROPOS :
            i4RetValue = setDW9800AFMacro(a_u4Param);
        break;
		
        default :
      	    DW9800AFDB("[DW9800AF] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

	return i4RetValue;
}


/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
/* 3.Update f_op pointer. */
/* 4.Fill data structures into private_data */
/* CAM_RESET */
static int DW9800AF_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
    long i4RetValue = 0;
	u8 vcm_id = 0xAA;
	
    DW9800AFDB("[DW9800AF] DW9800AF_Open - Start\n");

    //vcm_id = read_data(0x00);

    if(g_s4DW9800AF_Opened)
    {
        DW9800AFDB("[DW9800AF] the device is opened \n");
        return -EBUSY;
    }
    spin_lock(&g_DW9800AF_SpinLock);
    g_s4DW9800AF_Opened = 1;
    spin_unlock(&g_DW9800AF_SpinLock);


        char puSendCmd2[2] = {0x02,0x02};
        char puSendCmd3[2] = {0x06,0x40};
        char puSendCmd4[2] = {0x07,0x66};

		ic_diver = IC_9800;
	   	i4RetValue = i2c_master_send(g_pstDW9800AF_I2Cclient, puSendCmd2, 2);
	   	i4RetValue = i2c_master_send(g_pstDW9800AF_I2Cclient, puSendCmd3, 2);
	   	i4RetValue = i2c_master_send(g_pstDW9800AF_I2Cclient, puSendCmd4, 2);
	
    DW9800AFDB("[DW9800AF] DW9800AF_Open - End\n");

    return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int DW9800AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	DW9800AFDB("[DW9800AF] DW9800AF_Release - Start\n");

    if (g_s4DW9800AF_Opened)
    {
        DW9800AFDB("[DW9800AF] feee \n");
        g_sr = 5;
        #ifndef VENDOR_EDIT
        //OPPO zhangkw modify to speed Release
        s4DW9800AF_WriteReg(200);
        msleep(10);
	    s4DW9800AF_WriteReg(100);
        msleep(10);
        #else
        if(g_u4CurrPosition > g_u4DW9800AF_INF*11/10)
        {
            s4DW9800AF_WriteReg(g_u4DW9800AF_INF*11/10);
            //msleep(10);
        }  	  
        #endif
            	            	    	    
        spin_lock(&g_DW9800AF_SpinLock);
        g_s4DW9800AF_Opened = 0;
        spin_unlock(&g_DW9800AF_SpinLock);

	}
	DW9800AFDB("[DW9800AF] DW9800AF_Release - End\n");

	return 0;
}

static const struct file_operations g_stDW9800AF_fops = 
{
    .owner = THIS_MODULE,
    .open = DW9800AF_Open,
    .release = DW9800AF_Release,
    .unlocked_ioctl = DW9800AF_Ioctl,
    .compat_ioctl = DW9800AF_Ioctl,
};

inline static int Register_DW9800AF_CharDrv(void)
{
	struct device *vcm_device = NULL;

	DW9800AFDB("[DW9800AF] Register_DW9800AF_CharDrv - Start\n");

    //Allocate char driver no.
    if( alloc_chrdev_region(&g_DW9800AF_devno, 0, 1,DW9800AF_DRVNAME) )
    {
        DW9800AFDB("[DW9800AF] Allocate device no failed\n");

		return -EAGAIN;
	}
	/* Allocate driver */
	g_pDW9800AF_CharDrv = cdev_alloc();

    if(NULL == g_pDW9800AF_CharDrv)
    {
        unregister_chrdev_region(g_DW9800AF_devno, 1);

		DW9800AFDB("[DW9800AF] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(g_pDW9800AF_CharDrv, &g_stDW9800AF_fops);

	g_pDW9800AF_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pDW9800AF_CharDrv, g_DW9800AF_devno, 1))
    {
        DW9800AFDB("[DW9800AF] Attatch file operation failed\n");

		unregister_chrdev_region(g_DW9800AF_devno, 1);

		return -EAGAIN;
	}

    actuator_class = class_create(THIS_MODULE, "actuatordrv");
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        DW9800AFDB("Unable to create class, err = %d\n", ret);
        return ret;            
    }

	vcm_device = device_create(actuator_class, NULL, g_DW9800AF_devno, NULL, DW9800AF_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }
    
    DW9800AFDB("[DW9800AF] Register_DW9800AF_CharDrv - End\n");    
    return 0;
}

inline static void Unregister_DW9800AF_CharDrv(void)
{
	DW9800AFDB("[DW9800AF] Unregister_DW9800AF_CharDrv - Start\n");

	/* Release char driver */
	cdev_del(g_pDW9800AF_CharDrv);

	unregister_chrdev_region(g_DW9800AF_devno, 1);

	device_destroy(actuator_class, g_DW9800AF_devno);

	class_destroy(actuator_class);

	DW9800AFDB("[DW9800AF] Unregister_DW9800AF_CharDrv - End\n");
}

/* //////////////////////////////////////////////////////////////////// */

static int DW9800AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int DW9800AF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id DW9800AF_i2c_id[] = { {DW9800AF_DRVNAME, 0}, {} };

struct i2c_driver DW9800AF_i2c_driver = {
	.probe = DW9800AF_i2c_probe,
	.remove = DW9800AF_i2c_remove,
	.driver.name = DW9800AF_DRVNAME,
	.id_table = DW9800AF_i2c_id,
};

#if 0 
static int DW9800AF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, DW9800AF_DRVNAME);                                                         
    return 0;                                                                                       
}      
#endif 
static int DW9800AF_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int DW9800AF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	DW9800AFDB("[DW9800AF] DW9800AF_i2c_probe\n");

	/* Kirby: add new-style driver { */
	g_pstDW9800AF_I2Cclient = client;

	g_pstDW9800AF_I2Cclient->addr = g_pstDW9800AF_I2Cclient->addr >> 1;

	/* Register char driver */
	i4RetValue = Register_DW9800AF_CharDrv();

	if (i4RetValue) {

		DW9800AFDB("[DW9800AF] register char device failed!\n");

		return i4RetValue;
	}

	spin_lock_init(&g_DW9800AF_SpinLock);

    DW9800AFDB("[DW9800AF] Attached!! \n");

	return 0;
}

static int DW9800AF_probe(struct platform_device *pdev)
{
	return i2c_add_driver(&DW9800AF_i2c_driver);
}

static int DW9800AF_remove(struct platform_device *pdev)
{
	i2c_del_driver(&DW9800AF_i2c_driver);
	return 0;
}

static int DW9800AF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int DW9800AF_resume(struct platform_device *pdev)
{
	return 0;
}

//platform device
static struct platform_device g_stDW9800AF_device = {
    .name = "lens_actuator",
    .id = 0,
    .dev = {}
};

// platform structure
static struct platform_driver g_stDW9800AF_Driver = {
    .probe		= DW9800AF_probe,
    .remove	= DW9800AF_remove,
    .suspend	= DW9800AF_suspend,
    .resume	= DW9800AF_resume,
    .driver		= {
        .name	= "lens_actuator",
        .owner	= THIS_MODULE,
    }
};

static int __init DW9800AF_i2C_init(void)
{

	 printk("DW9800AF2 init E");

    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);

     if(platform_device_register(&g_stDW9800AF_device)){
        DW9800AFDB("failed to register DW9800 device\n");
        return -ENODEV;
    }	
	
    if(platform_driver_register(&g_stDW9800AF_Driver)){
        DW9800AFDB("failed to register DW9800AF driver\n");
        return -ENODEV;
    }
    DW9800AFDB("DW9800AF init exit");
    return 0;
}

static void __exit DW9800AF_i2C_exit(void)
{
	platform_driver_unregister(&g_stDW9800AF_Driver);
}
module_init(DW9800AF_i2C_init);
module_exit(DW9800AF_i2C_exit);

MODULE_DESCRIPTION("DW9800AF lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");
