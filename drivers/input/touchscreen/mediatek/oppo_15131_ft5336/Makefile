#include $(MTK_PATH_BUILD)/common.mk
include $(srctree)/drivers/misc/mediatek/Makefile.custom
#path := $(if $(filter yes,$(KBUILD_OUTPUT_SUPPORT)),$(srctree)/$(obj),$(obj))
#obj-y := $(patsubst %.c,%.o,$(subst $(path)/,,$(call wildcard2,$(path)/*.c)))
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/oppo_s3203/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y += ft_gesture_lib.a
obj-y += focaltech_driver.o
obj-y	+=  focaltech_ex_fun.o
obj-y	+=  focaltech_ctl.o

obj-y	+=  Config_FT3C47.o
obj-y	+=  Config_FT5X46.o
obj-y	+=  Config_FT6X36.o
obj-y	+=  Config_FT5822.o
obj-y	+=  Config_FT8606.o
obj-y	+=  DetailThreshold.o
obj-y	+=  Global.o
obj-y	+=  ini.o
obj-y	+=  Test_FT3C47.o
obj-y	+=  Test_FT5X46.o
obj-y	+=  Test_FT6X36.o
obj-y	+=  Test_FT5822.o
obj-y	+=  Test_FT8606.o
obj-y	+=  test_lib.o
obj-y	+=  testlib_Call_Sample.o
