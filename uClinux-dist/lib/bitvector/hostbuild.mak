ifndef ROOTDIR
	export ROOTDIR = $(shell pwd)/../..
	export ROMFSINST = $(ROOTDIR)/tools/romfs-inst.sh
	export ROMFSDIR = $(ROOTDIR)/romfs
	export IMAGEDIR = $(ROOTDIR)/image
endif
