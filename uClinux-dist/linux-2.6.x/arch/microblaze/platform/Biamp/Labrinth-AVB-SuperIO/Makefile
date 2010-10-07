#
# Makefile for the Labrinth-AVB-SuperIO board
#

# Include Xilinx common definitions and Lab X AVB headers
EXTRA_CFLAGS += -I$(TOPDIR)/drivers/xilinx_common
EXTRA_CFLAGS += -I$(TOPDIR)/drivers/net/labx_avb

# Include the Labrinth-specific drivers which layer
# functionality atop the Lab X AVB Audio drivers
obj-y += labrinth_avb_packetizer.o

