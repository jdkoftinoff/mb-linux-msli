#
# Makefile for the Lab X Technologies PTP device driver
#

# Include Xilinx common definitions
EXTRA_CFLAGS += -I$(TOPDIR)/drivers/xilinx_common

# Define a multi-part driver
obj-$(CONFIG_LABX_PTP) += labx_ptp.o

labx_ptp-objs	:= labx_ptp_main.o labx_ptp_messages.o labx_ptp_state.o \
	               labx_ptp_rtc.o labx_ptp_arithmetic.o
