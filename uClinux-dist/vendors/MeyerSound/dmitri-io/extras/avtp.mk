PWD:=$(shell pwd)
include $(PWD)/../../vendors/MeyerSound/dmitri-io/extras/common.mk
STACK_SIZE=32768

US_CONFIG=$(UCLIBC_LIB_DIR)/microsupport/microsupport-config
include $(PROJECT_TOP_DIR)/project.mk

romfs : 

include $(PROJECT_TOP_DIR)/magic.mk

