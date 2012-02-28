PWD:=$(shell pwd)
include $(PWD)/../../vendors/MeyerSound/CAL_ICS/extras/common.mk
STACK_SIZE=65536

US_CONFIG=$(UCLIBC_LIB_DIR)/microsupport/microsupport-config

include $(PROJECT_TOP_DIR)/project.mk

romfs : tools
	mkdir -p $(ROMFSDIR)/bin $(ROMFSDIR)/etc
	(for i in $(LIB_TOOLS_EXE_FILES); do $(ROMFSINST) $${i} /bin/$$(basename "$${i}"); done)
	$(ROMFSINST) -d $(PROJECT_TOP_DIR)/scripts/cal-dspd-loop /bin/cal-dspd-loop
	$(ROMFSINST) -d $(PROJECT_TOP_DIR)/etc/$(VARIANT)/init.d/. /etc/init.d/.
	$(ROMFSINST) -s /etc/init.d/cal-dspd /etc/rc.d/S92-cal-dspd

include $(PROJECT_TOP_DIR)/magic.mk

