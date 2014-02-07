#
# Include Xilinx common definitions and Lab X AVB headers
#
EXTRA_CFLAGS += -I$(TOPDIR)/drivers/xilinx_common
EXTRA_CFLAGS += -I$(TOPDIR)/drivers/net/labx_avb
EXTRA_CFLAGS += -DCAL_ICS

# Include the CAL-specific drivers
obj-y += CAL_ICS_setup.o
obj-y += cal_sample_pointers.o
obj-y += mv88e6350R.o
