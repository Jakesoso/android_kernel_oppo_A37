
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part
subdir-ccflags-y += -Werror

ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  cm36652-new/
endif
# ifdef VENDOR_EDIT
obj-y   +=  cust_alsps/

ifeq ($(CONFIG_MTK_APDS9921_NEW),y)
obj-y	+=  apds9921-new/
endif

ifeq ($(CONFIG_MTK_APDS9110_NEW),y)
obj-y	+=  apds9110-new/
endif

ifeq ($(CONFIG_MTK_CM36686_NEW),y)
obj-y	+=  cm36686-new/
endif
#endif /*VENDOR_EDIT*/
