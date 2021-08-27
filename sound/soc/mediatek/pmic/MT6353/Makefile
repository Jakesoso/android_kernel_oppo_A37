# MTK Platform Support

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MT_SND_SOC_6750),y)
CONFIG_MTK_PMIC_CHIP_MT6353=y
endif

COMMON_CFLAGS := -I$(src)/../../common_define
ccflags-y  += $(COMMON_CFLAGS)
obj-$(CONFIG_MTK_PMIC_CHIP_MT6353) += mt_soc_codec_speaker_6353.o mt_soc_codec_6353.o AudDrv_Ana.o