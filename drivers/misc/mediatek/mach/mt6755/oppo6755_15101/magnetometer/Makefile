
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_YAS532),y)
obj-y	+=  yamaha532/
endif

ifeq ($(CONFIG_MTK_YAS532_NEW),y)
obj-y	+=  yamaha532-new/
endif
#VENDOR_EDIT
ifeq ($(CONFIG_MTK_YAS533_NEW),y)
obj-y	+=  yamaha533-new/
endif

ifeq ($(CONFIG_MTK_MMC3416X_NEW),y)
obj-y	+=  mmc3416x-new/
endif
