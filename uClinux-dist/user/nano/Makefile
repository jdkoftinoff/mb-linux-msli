VER = nano-2.1.9

FLTFLAGS += -s 65536

CONF_OPTS =
#CONF_OPTS = \
#	--enable-tiny

include $(ROOTDIR)/tools/autotools.mk

romfs:
	$(ROMFSINST) -d build-nano-2.1.9/src/nano /usr/bin/nano
