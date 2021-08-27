
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_MPU6515G),y)
obj-y	+=  mpu6515/
endif

#ifdef VENDOR_EDIT//zhihong.lu@BSP.sensor
ifeq ($(CONFIG_MTK_LSM6DS3_NEW),y)
obj-y	+=  lsm6ds3_new/
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_GYROSCOPE),y)
obj-y	+=  cust_gyro/
endif
ifeq ($(CONFIG_MTK_BMI160),y)
obj-y	+=  bmi160/
endif
#endif

