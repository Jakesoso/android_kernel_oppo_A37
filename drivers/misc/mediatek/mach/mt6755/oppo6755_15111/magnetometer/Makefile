
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part
subdir-ccflags-y += -Werror

ifeq ($(CONFIG_MTK_YAS532),y)
obj-y	+=  yamaha532/
endif

ifeq ($(CONFIG_MTK_YAS532_NEW),y)
obj-y	+=  yamaha532-new/
endif

#ifdef VENDOR_EDIT
#//zhihong.lu@Prd.BSP.sensor,add for custom msensor
ifeq ($(CONFIG_MTK_YAS533_NEW),y)
obj-y	+=  yamaha533-new/
endif

ifeq ($(CONFIG_MTK_MMC3416X_NEW),y)
obj-y	+=  mmc3416x-new/
endif

ifeq ($(CONFIG_MTK_AKM8963_NEW),y)
obj-y	+=  akm8963-new/
endif

ifeq ($(CONFIG_MTK_AKM8963_NEW_OPPO),y)
obj-y	+=  akm8963-new_oppo/
endif
#endif /*VENDOR_EDIT*/
