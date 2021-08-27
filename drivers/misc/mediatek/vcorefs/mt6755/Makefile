include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += \
	-I$(MTK_CUSTOM_PATH)/videox/$(CONFIG_MTK_PLATFORM) \
	-I$(srctree)/drivers/misc/mediatek/spm/$(CONFIG_MTK_PLATFORM) \
	-I$(srctree)/drivers/misc/mediatek/smi/$(CONFIG_MTK_PLATFORM) \
	-I$(srctree)/drivers/misc/mediatek/mach/$(CONFIG_MTK_PLATFORM)/include

obj-y += mt_vcorefs_governor.o
obj-y += mt_vcorefs_manager.o


