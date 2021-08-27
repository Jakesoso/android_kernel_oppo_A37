include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MTK_PMIC_NEW_ARCH),y)
	obj-$(CONFIG_MTK_PMIC_WRAP) +=  pwrap_hal_v1.o
else
	obj-$(CONFIG_MTK_PMIC_WRAP) +=  pwrap_hal.o
endif
