#
# Makefile for the Lab X Technologies Tri-mode ethernet driver
#

EXTRA_CFLAGS		+= -Idrivers/xilinx_common

# Incorporate modules into a multi-part driver
labx_eth_llink-objs	:= labx_eth_locallink_main.o labx_eth_locallink.o \
					   labx_eth_locallink_control.o

obj-$(CONFIG_LABX_ETH_LOCALLINK) += labx_eth_llink.o