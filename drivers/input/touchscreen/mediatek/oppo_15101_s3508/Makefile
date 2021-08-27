include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/oppo_s3508/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/oppo_s3508/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/oppo_s3508/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  synaptics_s3508.o
obj-y	+=  synaptics_redremote.o
obj-y	+=  synaptics_s1302.o
obj-y	+=  synaptics_s1302_redremote.o

