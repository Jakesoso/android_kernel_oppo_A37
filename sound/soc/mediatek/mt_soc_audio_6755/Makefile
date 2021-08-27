# MTK Platform Support

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MT_SND_SOC_6750),y)
CONFIG_MTK_PMIC_CHIP_MT6353=y
CONFIG_MTK_AP_CHIP_MT6755=y
endif
ifeq ($(CONFIG_MT_SND_SOC_6755),y)
CONFIG_MTK_PMIC_CHIP_MT6351=y
CONFIG_MTK_AP_CHIP_MT6755=y
endif

TMP_CFLAGS := -I$(src)/../common
COMMON_CFLAGS := -I$(src)/../common_define
6351_CFLAGS := -I$(src)/../pmic/MT6351
6353_CFLAGS := -I$(src)/../pmic/MT6353
ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6351) += $(6351_CFLAGS)
ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6353) += $(6353_CFLAGS)
ccflags-y  += $(TMP_CFLAGS)
ccflags-y  += $(COMMON_CFLAGS)
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += AudDrv_Afe.o AudDrv_Clk.o AudDrv_Gpio.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_afe_control.o mt_soc_afe_connection.o mt_soc_pcm_common.o mt_soc_offload_gdma.o mt_soc_offload_common.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_pcm_capture.o mt_soc_pcm_dl1.o mt_soc_pcm_dummy.o mt_soc_pcm_routing.o mt_soc_pcm_capture2.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_pcm_voice_md1.o mt_soc_pcm_voice_md2.o mt_soc_pcm_voice_md1_bt.o mt_soc_pcm_voice_md2_bt.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_pcm_hdmi.o mt_soc_pcm_dl1_i2s0.o mt_soc_pcm_dl1_i2s0Dl1.o mt_soc_pcm_i2s0_awb.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_pcm_uldlloopback.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_pcm_mrgrx.o mt_soc_pcm_mrgrx_awb.o mt_soc_pcm_fm_i2s.o mt_soc_pcm_fm_i2s_awb.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_pcm_dl1_awb.o mt_soc_pcm_dl1_bt.o mt_soc_pcm_bt_dai.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_dai_stub.o mt_soc_dai_routing.o mt_soc_codec_dummy.o mt_soc_pcm_fmtx.o mt_soc_tdm_capture.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_pcm_hp_impedance.o
# yongzhi.zhang@Multimedia, 2016/01/23, fix alloc empty buffer bug in probe process which results in the bug when using BT voice
obj-$(CONFIG_MTK_BTCVSD_ALSA) += mt_soc_pcm_btcvsd.o mt_soc_pcm_btcvsd_rx.o mt_soc_pcm_btcvsd_tx.o
obj-$(CONFIG_MTK_AP_CHIP_MT6755) += mt_soc_machine.o

