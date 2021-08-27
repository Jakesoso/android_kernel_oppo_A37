
# Makefile for the power golden setting.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y += mt_power_gs.o
#obj-y += mt_power_gs_cg_array.o
#obj-y += mt_power_gs_dcm_array.o
#obj-y += mt_power_gs_6331_array.o
#obj-y += mt_power_gs_6332_array.o
#obj-y += mt_power_gs_6325_array.o
ifdef CONFIG_MTK_PMIC_CHIP_MT6353
obj-y += mt_power_gs_6353_array.o
else
obj-y += mt_power_gs_6351_array.o
endif
obj-y += mt_power_gs_suspend.o
obj-y += mt_power_gs_dpidle.o
#obj-y += mt_power_gs_idle.o
#obj-y += mt_power_gs_audio_playback.o
