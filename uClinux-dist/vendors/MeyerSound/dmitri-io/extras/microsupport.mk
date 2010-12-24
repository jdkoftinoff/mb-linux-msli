PWD:=$(shell pwd)
include $(PWD)/../../vendors/MeyerSound/dmitri-io/extras/common.mk

include $(PROJECT_TOP_DIR)/project.mk

romfs : 

include $(PROJECT_TOP_DIR)/magic.mk

