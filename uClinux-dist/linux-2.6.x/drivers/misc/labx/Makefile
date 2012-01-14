#
# Makefile for LABX mailbox
#

# Include Xilinx common definitions
EXTRA_CFLAGS += -I$(TOPDIR)/drivers/xilinx_common

obj-$(CONFIG_LABX_MAILBOX) += labx_mailbox_.o
labx_mailbox_-objs = labx_mailbox.o
labx_mailbox_-objs+= labx_mailbox_netlink.o
