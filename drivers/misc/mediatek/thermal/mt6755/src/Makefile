include $(srctree)/drivers/misc/mediatek/Makefile.custom

THERMAL_CHIP_DRIVER_DIR := $(srctree)/drivers/misc/mediatek/thermal/$(MTK_PLATFORM)
ccflags-y  += -I$(THERMAL_CHIP_DRIVER_DIR)/inc

obj-$(CONFIG_THERMAL) += mtk_tc.o

ifeq ($(CONFIG_MTK_PMIC_CHIP_MT6353),y)
obj-$(CONFIG_THERMAL) += mtk_pmic_mt6353_efuse.o
else
obj-$(CONFIG_THERMAL) += mtk_pmic_efuse.o
endif

obj-$(CONFIG_THERMAL) += mtk_ta.o
