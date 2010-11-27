#
# Makefile for the Xilinx Tri-mode ethernet driver
#

EXTRA_CFLAGS		+= -Idrivers/xilinx_common

# The Linux adapter for the Xilinx driver code.

# the following is a split version with PHY control moved to separate files
# (but not using PHY Abstraction Layer yet)

xilinx_temac-objs	:= xlltemac_main.o xlltemac.o xlltemac_control.o xlltemac_phy_common.o xlltemac_phy_marvell.o

# the following uses old monolithic source file
#xilinx_temac-objs	:= xlltemac_main_monolithic.o xlltemac.o xlltemac_control.o

obj-$(CONFIG_XILINX_LLTEMAC) := xilinx_temac.o
