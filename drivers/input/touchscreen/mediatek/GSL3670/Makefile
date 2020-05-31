include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GSL3670/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GSL3670/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  gsl_ts_driver.o
obj-y	+=  gsl_ReadArrayData.o


