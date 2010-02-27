ifndef ROOTDIR
# Not running under the uClinux build environment,
# so build a host version for debugging.

	export ROOTDIR = $(shell pwd)/../..
	export ROMFSINST = $(ROOTDIR)/tools/romfs-inst.sh
	export ROMFSDIR = $(ROOTDIR)/romfs
	export IMAGEDIR = $(ROOTDIR)/image

ifndef UCLINUX_BUILD_LIB
	UCLINUX_BUILD_USER=1
endif
	UCLINUX_BUILD_LIB=1

	# Can't convince PAM to use a different path
	CONFIG_USER_FNORD_NOAUTH=y

	# This one runs in server mode so that we don't need tcpserver
	CFLAGS += -g -DHOSTBUILD -DDEBUG -DLOG_TO_SYSLOG -DSERVER_MODE #-DCHECK_STR_COPY
endif
