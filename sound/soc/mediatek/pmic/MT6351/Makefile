# MTK Platform Support

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MT_SND_SOC_6755),y)
CONFIG_MTK_PMIC_CHIP_MT6351=y
endif

COMMON_CFLAGS := -I$(src)/../../common_define
ccflags-y  += $(COMMON_CFLAGS)
obj-$(CONFIG_MTK_PMIC_CHIP_MT6351) += mt_soc_codec_speaker_63xx.o mt_soc_codec_63xx.o AudDrv_Ana.o
