#############################################################################
#
#	Makefile -- Build instructions for McAfee SGSupport
#
#############################################################################

BUILD_START_STRING ?= $(shell cat $(ROMFSDIR)/etc/version | sed 's/^.* --  //')

ZIMAGE    = $(IMAGEDIR)/bzImage
ROMFSIMG  = $(IMAGEDIR)/initrd
ROMFSIMGZ = $(IMAGEDIR)/initrd.gz
IMGBASE   = $(CONFIG_PRODUCT)
ISODIR    = $(IMAGEDIR)/iso
EXT3DIR   = $(IMAGEDIR)/ext3
EXTIMG    = $(IMAGEDIR)/ext.img


# the number of 512 byte blocks in config partition (4096 = 2Mb)
VENDDIR   = $(ROOTDIR)/vendors/$(CONFIG_VENDOR)/$(CONFIG_PRODUCT)

SQUASH_ENDIAN = -le

ROMFS_DIRS = $(DEFAULT_ROMFS_DIRS) boot

DEVICES += $(DEVICE_PTY_64)

FLASH_DEVICES =

#############################################################################

all:

clean: image.clean

# We have our own network.init script which does not add a static eth0:0 
# alias. Default eth0 setting is DHCP. Also install our custom console shell.
# We need a dummy /bin/firewall so that network programs that call it don't fail.
romfs: romfs_dev romfs.dirs romfs.symlinks romfs.default romfs.rc romfs.version
	$(ROMFSINST) network.init /etc/network.init
	$(ROMFSINST) product.init /etc/product.init
	$(ROMFSINST) -p 755 /bin/cmdline.tcl
ifdef CONFIG_USER_UDEV
	$(ROMFSINST) -s /var/dev/config /lib/udev/devices/flash/config
	$(ROMFSINST) -s /var/dev/image /lib/udev/devices/flash/image
else
	$(ROMFSINST) -s /var/dev/config /dev/flash/config
	$(ROMFSINST) -s /var/dev/image /dev/flash/image
endif
ifdef CONFIG_USER_NETFLASH_CRYPTO
	if [ -f $(SGKEY) ] ; then \
		openssl rsa -in $(SGKEY) -pubout > $(ROMFSDIR)/etc/publickey.pem ; \
	fi
endif
	$(ROMFSINST) -p 755 firewall.dummy /bin/firewall

# Link index.cgi to cgix-load so that our default page is the Support Report Emulator.
# Do it here so it overwrites anything else that may have created an index.cgi link.
romfs.post:: image.dir image.i386.zimage
	$(ROMFSINST) -s /home/httpd/cgi-bin/cgix-load /home/httpd/cgi-bin/index.cgi
	$(ROMFSINST) $(ZIMAGE) /boot/vmlinuz
	$(ROMFSINST) /boot/message
	( \
	cd $(ROOTDIR)/romfs; \
	grep -v setmac < etc/rc > etc/rc.new; \
	mv etc/rc.new etc/rc; \
	)

image: image.configs image.dir image.i386.zimage hdd-images iso-images supportemu-check

supportemu-check:
	$(MAKE) -C $(ROOTDIR)/prop/supportemu supportemu-check

#############################################################################
.PHONY: qemu-version-check

QEMU_VER_1 := 0
QEMU_VER_2 := 9
QEMU_VER_3 := 1
qemu-version-check:
	qemu --help | grep version | cut -f5 -d" " | tr ., '  ' | \
		( read a b c t; \
		  if [ "$$a" -gt $(QEMU_VER_1) -o "$$b" -gt $(QEMU_VER_2) -o "$$c" -ge $(QEMU_VER_3) ]; \
		  then echo "qemu version okay"; \
		  else echo "*** qemu version needs to be at least version" \
			    "$(QEMU_VER_1).$(QEMU_VER_2).$(QEMU_VER_3)" \
			    "to build hdd image successfully."; \
			exit 1; \
		  fi; \
		)


#############################################################################

iso-images: image.squashfs7z
	gzip -9 -f $(ROMFSIMG)
	$(MAKE) ISOBASE=$(IMGBASE)-vga-iso DEFAULT=vga iso-image
	$(MAKE) ISOBASE=$(IMGBASE)-com1-iso DEFAULT=com1 iso-image

iso-image:
	rm -rf $(IMAGEDIR)/$(ISOBASE)
	mkdir $(IMAGEDIR)/$(ISOBASE)
	$(MAKE) IMAGE=$(IMAGEDIR)/$(ISOBASE)/$(IMGBASE).iso iso-image-2
	$(MAKE) HEADS=255 SECTORS=63 CYLS=1 IMAGE=$(IMAGEDIR)/$(ISOBASE)/$(IMGBASE).cfg cfg-image
	$(MAKE) HEADS=255 SECTORS=63 CYLS=20 IMAGE=$(IMAGEDIR)/$(ISOBASE)/$(IMGBASE).reports hdd-image-blank
	cp SGSupport-iso.vmx $(IMAGEDIR)/$(ISOBASE)/$(IMGBASE).vmx
	cp nvram $(IMAGEDIR)/$(ISOBASE)/
	(cd $(IMAGEDIR)/$(ISOBASE); zip -r ../$(ISOBASE).zip .)

iso-image-2:
	rm -rf $(ISODIR)
	mkdir -p $(ISODIR)/boot/isolinux
	cp isolinux.bin $(ISODIR)/boot/isolinux/
	cp boot.msg $(ISODIR)/boot/isolinux/
	echo "default $(DEFAULT)" > $(ISODIR)/boot/isolinux/isolinux.cfg
	cat isolinux.cfg >> $(ISODIR)/boot/isolinux/isolinux.cfg
	cp $(ZIMAGE) $(ISODIR)/boot/isolinux/
	cp $(ROMFSIMGZ) $(ISODIR)/boot/isolinux/initrd.gz
	mkisofs -J -o $(IMAGE) -b boot/isolinux/isolinux.bin \
		-c boot/isolinux/boot.cat -no-emul-boot \
		-boot-load-size 4 -boot-info-table $(ISODIR)
	rm -rf $(ISODIR)

#############################################################################

hdd-images:
	$(MAKE) HDDBASE=$(IMGBASE)-vga-scsi-hdd CONFIG=4 LILOCONF=lilo-scsi.conf VMX=SGSupport-scsi-hdd.vmx HDD_IF=buslogic APPEND= hdd-image
	$(MAKE) HDDBASE=$(IMGBASE)-vga-scsi-hdd ovf
	$(MAKE) HDDBASE=$(IMGBASE)-vga-ide-hdd CONFIG=4 LILOCONF=lilo-ide.conf VMX=SGSupport-ide-hdd.vmx HDD_IF=ide APPEND= hdd-image
	$(MAKE) HDDBASE=$(IMGBASE)-com1-ide-hdd CONFIG=4 LILOCONF=lilo-ide.conf VMX=SGSupport-ide-hdd.vmx HDD_IF=ide APPEND="console=ttyS0,9600" hdd-image

hdd-image:
	rm -rf $(IMAGEDIR)/$(HDDBASE)
	mkdir $(IMAGEDIR)/$(HDDBASE)
	$(MAKE) IMAGE=$(IMAGEDIR)/$(HDDBASE)/$(IMGBASE).hdd NFI=$(IMAGEDIR)/$(HDDBASE).sgu CYLS=30 HEADS=70 SECTORS=63 hdd-image-fs
	cp $(VMX) $(IMAGEDIR)/$(HDDBASE)/$(IMGBASE).vmx
	rm -f $(IMAGEDIR)/$(HDDBASE).tar.gz
	(cd $(IMAGEDIR)/$(HDDBASE); zip -r ../$(HDDBASE).zip .)

ifdef SECTORS
TOTAL_SECTORS := $(shell expr $(CYLS) \* $(HEADS) \* $(SECTORS))
TOTAL_CONFIG  := $(shell expr $(CONFIG) \* 1024 \* 1024 / 512)
TOTAL_EXTSIZE := $(shell expr $(TOTAL_SECTORS) - 1 - $(TOTAL_CONFIG))
EXT_BLOCKS    := $(shell expr $(TOTAL_EXTSIZE) / 2)

cfg-image:
	dd if=/dev/zero bs=512 count=$(TOTAL_SECTORS) of=$(IMAGE)
	printf "1,,61,-\n" | \
		/sbin/sfdisk --force -uS -C $(CYLS) -H $(HEADS) -S $(SECTORS) $(IMAGE)
	$(MAKE) hdd-vmdk

hdd-image-blank:
	dd if=/dev/zero bs=512 count=$(TOTAL_SECTORS) of=$(IMAGE)
	printf "1,,L,-\n" | \
		/sbin/sfdisk --force -uS -C $(CYLS) -H $(HEADS) -S $(SECTORS) $(IMAGE)
	$(MAKE) hdd-vmdk

hdd-image-fs: qemu-version-check
	rm -f $(EXTIMG) $(IMAGEDIR)/device.spec $(IMAGEDIR)/qemu.pid
	rm -rf $(EXT3DIR)
	mkdir $(EXT3DIR)
	(cd $(ROMFSDIR);tar cf - .)|(cd $(EXT3DIR);tar xf -)
	cp $(LILOCONF) $(EXT3DIR)/etc/lilo.conf
	[ -z "$(APPEND)" ] || \
		echo "	append = \"$(APPEND)\"" >> $(EXT3DIR)/etc/lilo.conf
	cd $(EXT3DIR); find dev -type f -name '@*' -print | \
		sed 's/[@,]/ /g' | while read path file type maj min; do \
			rm -f $(EXT3DIR)/$$path/@$$file,$$type,$$maj,$$min; \
			echo "/$$path$$file	$$type 666 0 0 $$maj $$min - - -"; \
		done > $(IMAGEDIR)/device.spec
	/usr/bin/genext2fs -U -d $(EXT3DIR) -D $(IMAGEDIR)/device.spec \
		-b $(EXT_BLOCKS) $(EXTIMG)
	/sbin/tune2fs -i 0 -j $(EXTIMG)
	dd if=/dev/zero bs=512 count=1 of=$(IMAGE)
	printf "1,$(TOTAL_EXTSIZE),L,*\n,,61,-\n" | \
		/sbin/sfdisk --force -uS -C $(CYLS) -H $(HEADS) -S $(SECTORS) $(IMAGE)
	dd if=$(EXTIMG) conv=sync bs=`expr 512 \\* $(TOTAL_EXTSIZE)` count=1 >> $(IMAGE)
	dd if=/dev/zero bs=`expr 512 \\* $(TOTAL_CONFIG)` count=1 >> $(IMAGE)
	yes "" | \
		qemu -net none -nographic -pidfile $(IMAGEDIR)/qemu.pid \
			-append "rw root=301 console=ttyS0,9600 init=/bin/lilo" \
			-kernel $(ZIMAGE) \
			-drive file=$(IMAGE),if=ide \
			-drive file=$(IMAGE),if=scsi | ( \
			s=`date +%s`; \
			while read t; do \
				echo "$$t"; \
				case "$$t" in \
				*panic*) kill -INT `cat $(IMAGEDIR)/qemu.pid` ;; \
				esac; \
				n=`date +%s`; \
				d=`expr $$n - $$s`; \
				if [ "$$d" -gt 30 ]; then \
					kill -INT `cat $(IMAGEDIR)/qemu.pid`; \
					s=$$n; \
				fi; \
			done; \
			echo "Finished running lilo live"; \
		)
	rm -f $(EXTIMG) $(IMAGEDIR)/device.spec $(IMAGEDIR)/qemu.pid
	rm -rf $(EXT3DIR)
	dd if=$(IMAGE) conv=sync bs=`expr 512 \\* $(TOTAL_EXTSIZE) + 512` count=1 |\
		gzip -9  > $(NFI)
	printf '\0%s\0%s\0%s' $(VERSIONPKG) $(HW_VENDOR) $(HW_PRODUCT) >>$(NFI)
ifdef CONFIG_USER_NETFLASH_CRYPTO
	if [ -f $(SGKEY) ] ; then \
		$(ROOTDIR)/user/netflash/cryptimage -k $(SGKEY) -f $(NFI); \
		printf '\0%s\0%s\0%s' $(VERSIONPKG) $(HW_VENDOR) $(HW_PRODUCT) >>$(NFI) ; \
	fi
endif
	$(ROOTDIR)/tools/cksum -b -o 2 $(NFI) >> $(NFI)
	$(MAKE) hdd-vmdk IMAGE=$(IMAGE)

hdd-vmdk:
	( \
		echo "# VMWare Disk descriptor file"; \
		echo "version=1"; \
		echo "CID=ffffffff"; \
		echo "parentCID=ffffffff"; \
		echo "createType=\"monolithicFlat\""; \
		echo "# Extent description"; \
		echo "RW $(TOTAL_SECTORS) FLAT \"`basename $(IMAGE)`\" 0"; \
		echo "ddb.virtualHWVersion = \"4\""; \
		echo "ddb.adapterType = \"$(HDD_IF)\""; \
		echo "ddb.geometry.cylinders = \"$(CYLS)\""; \
		echo "ddb.geometry.heads = \"$(HEADS)\""; \
		echo "ddb.geometry.sectors = \"$(SECTORS)\""; \
	) > $(IMAGE).vmdk
endif

#############################################################################

include $(ROOTDIR)/vendors/config/config.dev
include ../vendor.mak

