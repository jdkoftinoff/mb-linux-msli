#
# Makefile for the Lab X Technologies PTP device driver
#

# Define a multi-part driver
obj-$(CONFIG_LABX_PTP) += labx_ptp.o

labx_ptp-objs := labx_ptp_main.o labx_ptp_messages.o \
                 labx_ptp_pdelay_state.o labx_ptp_state.o \
                 labx_ptp_rtc.o labx_ptp_arithmetic.o \
                 labx_ptp_netlink.o labx_ptp_fpga.o \
                 labx_ptp_bmca_announce_state.o
