include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_USB_DEBUG),y)
	EXTRA_CFLAGS += -DDEBUG
endif

#For USB HQA Driving Tuning Mode 1 Settings
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,add 2015/5/4 for eyesight with vref internal R
EXTRA_CFLAGS += -DMTK_USB_MODE1
#endif/*VENDOR_EDIT*/

obj-$(CONFIG_PROJECT_PHY) += mtk-phy-asic.o
