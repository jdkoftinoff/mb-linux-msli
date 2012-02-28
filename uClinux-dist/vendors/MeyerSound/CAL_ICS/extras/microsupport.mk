PWD:=$(shell pwd)
include $(PWD)/../../vendors/MeyerSound/dmitri-io/extras/common.mk
STACK_SIZE=32768

include $(PROJECT_TOP_DIR)/project.mk

romfs : 

include $(PROJECT_TOP_DIR)/magic.mk

