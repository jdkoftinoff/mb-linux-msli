PWD:=$(shell pwd)
include $(PWD)/../../vendors/MeyerSound/dmitri-io/extras/common.mk

US_CONFIG=$(UCLIBC_LIB_DIR)/microsupport/microsupport-config

include $(PROJECT_TOP_DIR)/project.mk

romfs : tools
	mkdir -p $(ROMFSDIR)/bin $(ROMFSDIR)/etc
	(for i in $(LIB_TOOLS_EXE_FILES); do $(ROMFSINST) $${i} /bin/$$(basename "$${i}"); done)
	$(ROMFSINST) -d $(PROJECT_TOP_DIR)/scripts/io-oscd-loop /bin/io-oscd-loop
	$(ROMFSINST) -d $(PROJECT_TOP_DIR)/etc/$(VARIANT)/init.d/. /etc/init.d/.
	$(ROMFSINST) -s /etc/init.d/S91-io-oscd /etc/rc.d/S91-io-oscd

include $(PROJECT_TOP_DIR)/magic.mk

