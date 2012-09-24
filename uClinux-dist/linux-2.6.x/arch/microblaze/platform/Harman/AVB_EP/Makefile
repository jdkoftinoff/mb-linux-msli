#
# Makefile for the Harman Studer AVB EP64 board
#

# Include Xilinx common definitions and Lab X AVB headers
EXTRA_CFLAGS += -I$(TOPDIR)/drivers/xilinx_common
EXTRA_CFLAGS += -I$(TOPDIR)/drivers/net/labx_avb

# Include the Studer-specific drivers which layer
# functionality atop the Lab X AVB Audio drivers

obj-y += avb_ep64_tdm_output.o
obj-y += avb_ep64_packetizer.o
obj-y += studer_legacy_bridge.o
