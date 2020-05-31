include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y   +=  mt65xx_lcm_list.o

ifeq ($(CONFIG_CUSTOM_KERNEL_LCM),"hx8394d_mipi_dsi_vdo_boe")
obj-y   +=  hx8394d_mipi_dsi_vdo_boe.o
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_LCM),"nt51021b_mipi_dsi_vdo_qc")
obj-y   +=  nt51021b_mipi_dsi_vdo_qc.o
endif