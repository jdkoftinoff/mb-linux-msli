#
# Makefile for the Lab X Technologies Direct 10/100/1000
# Ethernet driver
#

EXTRA_CFLAGS		+= -Idrivers/xilinx_common

# Incorporate modules into a multi-part driver
labx_direct_ethernet-objs	:= labx_ethernet_main.o    \
				   labx_ethernet.o         \
				   labx_ethernet_control.o \
				   labx_ethernet_mdio.o

obj-$(CONFIG_LABX_ETHERNET) += labx_direct_ethernet.o

