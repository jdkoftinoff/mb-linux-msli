PWD:=$(shell pwd)
include $(PWD)/../../vendors/MeyerSound/dmitri-io/extras/common.mk

US_CONFIG=$(UCLIBC_LIB_DIR)/microsupport/microsupport-config


romfs : tools
	mkdir -p $(ROMFSDIR)/bin $(ROMFSDIR)/etc
	(for i in $(LIB_TOOLS_EXE_FILES); do $(ROMFSINST) -r $(ROMFSDIR) $${i} /bin/$$(basename "$${i}"); done)
	$(ROMFSINST) -r $(ROMFSDIR) -d $(PROJECT_TOP_DIR)/scripts/midi-gateway-loop /bin/midi-gateway-loop
	$(ROMFSINST) -r $(ROMFSDIR) -d $(PROJECT_TOP_DIR)/etc/$(VARIANT)/init.d/. /etc/init.d/.
	$(ROMFSINST) -r $(ROMFSDIR) -s /etc/init.d/midi-gateway /etc/rc.d/S90midi-gateway
	$(ROMFSINST) -r $(ROMFSDIR) -d $(PROJECT_TOP_DIR)/etc/$(VARIANT)/avahi/services/. /etc/avahi.orig/services/.


