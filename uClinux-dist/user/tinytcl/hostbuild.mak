ifndef ROOTDIR
	ROOTDIR = $(shell pwd)/../..
	ROMFSINST = $(ROOTDIR)/tools/romfs-inst.sh
	ROMFSDIR = $(ROOTDIR)/romfs
	IMAGEDIR = $(ROOTDIR)/image

	-include hostbuild.import

ifndef UCLINUX_BUILD_LIB
	UCLINUX_BUILD_USER=1
endif
	UCLINUX_BUILD_LIB=1
	CFLAGS += -fPIC
endif
