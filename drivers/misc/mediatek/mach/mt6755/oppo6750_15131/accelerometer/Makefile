
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_MPU6515A),y)
obj-y	+=  mpu6515/
endif

#ifdef VENDOR_EDIT//zhihong.lu@BSP.sensor
ifeq ($(CONFIG_MTK_LIS3DH),y)
obj-y   +=  lis3dh-new/
endif

ifeq ($(CONFIG_MTK_KX023_NEW),y)
obj-y   +=  kx023-new/
endif

ifeq ($(CONFIG_MTK_LIS2DS12),y)
obj-y   +=  lis2ds12/
endif
ifeq ($(CONFIG_MTK_LSM6DS3_NEW),y)
obj-y   +=  lsm6ds3_new/
endif
ifeq ($(CONFIG_MTK_BMI160),y)
obj-y   +=  bmi160/
endif
#endif//VENDOR_EDIT
