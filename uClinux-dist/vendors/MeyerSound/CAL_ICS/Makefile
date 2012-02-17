#
# Makefile -- Build instructions for Meyer Sound CAL ICS MicroBlaze system
#

#default RELEASE_VERSION to current date if RELEASE_VERSION is not set
ifndef RELEASE_VERSION
	RELEASE_VERSION := $(shell date +%Y%m%d)
endif

include $(LINUX_CONFIG)
include $(CONFIG_CONFIG)
include $(ARCH_CONFIG)

LINUX	 = $(ROOTDIR)/$(LINUXDIR)/linux 
LINUXBIN = $(IMAGEDIR)/linux.bin
ROMFSIMG = $(IMAGEDIR)/romfs.img
JFFS2IMG = $(IMAGEDIR)/jffs2.img
IMAGE    = $(IMAGEDIR)/image.bin
ELFIMAGE = $(IMAGEDIR)/image.elf
CKSUM	 = $(ROOTDIR)/tools/cksum
SREC_IMAGE= $(IMAGEDIR)/image.srec

COMMON=$(VEND)/$(CONFIG_VENDOR)/$(CONFIG_PRODUCT)/

DIRS = 

ROMFS_DIRS = bin dev dev/flash etc home lib mnt mnt/apps mnt/config proc usr var


ROMFS_DIRS-$(CONFIG_SYSFS) += sys

ROMFS_DIRS-$(CONFIG_DEBUG_FS) += debugfs

ROMFS_DIRS += $(ROMFS_DIRS-y)

ifdef CONFIG_SYSTEM_ROOTFS_JFFS2
    ROMFS_DIRS += tmp var/run var/log var/tmp /var/lock var/empty
	FSIMG = $(JFFS2IMG)
else
ifdef CONFIG_SYSTEM_ROOTFS_INITRAMFS
	FSIMG = $(INITRAMFSIMG)
else
	FSIMG = $(ROMFSIMG)
endif
endif

ifdef CONFIG_USER_FLATFSD_FLATFSD
	ETCDIR = /etc/default
else
	ETCDIR = /etc
endif

DEVICES =								      \
	tty,c,5,0      console,c,5,1   mem,c,1,1     kmem,c,1,2    null,c,1,3 \
									      \
	ttyS0,c,4,64   ttyS1,c,4,65    ttyS2,c,4,66  ttyS3,c,4,67	      \
	ttyUL0,c,204,187	ttyUL1,c,204,188			      \
	ttyUL2,c,204,189	ttyUL3,c,204,190			      \
	rtc,c,10,135   nvram,c,10,144					      \
									      \
	zero,c,1,5     random,c,1,8    urandom,c,1,9			      \
									      \
        fb0,c,29,0                                                            \
                                                                              \
        ptmx,c,5,2                                                            \

DEVICES +=   \
	mtd0,c,90,0	mtdr0,c,90,1	mtdblock0,b,31,0	\
	mtd1,c,90,2	mtdr1,c,90,3	mtdblock1,b,31,1	\
	mtd2,c,90,4	mtdr2,c,90,5	mtdblock2,b,31,2	\
	mtd3,c,90,6	mtdr3,c,90,7	mtdblock3,b,31,3	\
	mtd4,c,90,8 	mtdr4,c,90,9	mtdblock4,b,31,4	\
	mtd5,c,90,10 	mtdr5,c,90,11	mtdblock5,b,31,5	\
	mtd6,c,90,12 	mtdr6,c,90,13	mtdblock6,b,31,6	\
	mtd7,c,90,14 	mtdr7,c,90,15	mtdblock7,b,31,7	\
	mtd8,c,90,16 	mtdr8,c,90,17	mtdblock8,b,31,8	\
	mtd9,c,90,18 	mtdr9,c,90,19	mtdblock9,b,31,9	\
	mtd10,c,90,20 	mtdr10,c,90,21	mtdblock10,b,31,10  \
	mtd11,c,90,22 	mtdr11,c,90,23	mtdblock11,b,31,11	\
	mtd12,c,90,24 	mtdr12,c,90,25	mtdblock12,b,31,12	\
	mtd13,c,90,26 	mtdr13,c,90,27	mtdblock13,b,31,13	\
	mtd14,c,90,26 	mtdr14,c,90,29	mtdblock14,b,31,14

ifdef CONFIG_I2C_SERIAL
DEVICES += 	\
    dsp1,c,254,0 \
    dsp1_amp1,c,254,1 \
    dsp1_amp2,c,254,2 \
    dsp2,c,254,3 \
    dsp2_amp1,c,254,4 \
    dsp2_amp2,c,254,5 \
    dsp3,c,254,6 \
    dsp3_amp1,c,254,7 \
    dsp3_amp2,c,254,8
endif


ifdef CONFIG_MICROBLAZE_FSLFIFO
DEVICES += 	\
	fslfifo0,c,10,192 fslfifo1,c,10,193 fslfifo2,c,10,194		\
	fslfifo3,c,10,195 fslfifo4,c,10,196 fslfifo5,c,10,197		\
	fslfifo6,c,10,198 fslfifo7,c,10,199
endif

ifdef CONFIG_UIO
DEVICES +=	\
	uio0,c,254,0	uio1,c,254,1	uio2,c,254,2	uio3,c,254,3	\
	uio4,c,254,4	uio5,c,254,5	uio6,c,254,6	uio7,c,254,7	\
	uio8,c,254,8	uio9,c,254,9	uio10,c,254,10	uio11,c,254,11
endif

ifdef CONFIG_XILINX_SYSACE                                                  
DEVICES +=    \
	xsysacea,b,254,0        xsysacea1,b,254,1       xsysacea2,b,254,2  \
	xsysacea3,b,254,3       xsysacea4,b,254,4       xsysacea5,b,254,5  \
	xsysacea6,b,254,6       xsysacea7,b,254,7       xsysacea8,b,254,8  \
	xsysacea9,b,254,9       xsysacea10,b,254,10     xsysacea11,b,254,11 \
	xsysacea12,b,254,12     xsysacea12,b,254,13     xsysacea14,b,254,14 \
	xsysacea15,b,254,15 
endif 

# sd? dev nodes if SCSI is enabled
ifdef CONFIG_SCSI
DEVICES +=	sda,b,8,0	\
	sda1,b,8,1	sda2,b,8,2	sda3,b,8,3	sda4,b,8,4	\
	sda5,b,8,5	sda6,b,8,6	sda7,b,8,7	sda8,b,8,8	\
	sda9,b,8,9	sda10,b,8,10	sda11,b,8,11	sda12,b,8,12	\
	sda13,b,8,13	sda14,b,8,14	sda15,b,8,15			\
	sdb,b,8,16							\
	sdb1,b,8,17	sdb2,b,8,18	sdb3,b,8,19	sdb4,b,8,20	\
	sdb5,b,8,21	sdb6,b,8,22	sdb7,b,8,23	sdb8,b,8,24	\
	sdb9,b,8,25	sdb10,b,8,26	sdb11,b,8,27	sdb12,b,8,28	\
	sdb13,b,8,29	sdb14,b,8,30	sdb15,b,8,31
endif

ifdef CONFIG_XILINX_GPIO
DEVICES +=      \
        gpio,c,10,185	gpio0,c,10,185	gpio1,c,10,186,	gpio2,c,10,187	\
	gpio3,c,10,188	gpio4,c,10,189
endif

ifdef CONFIG_INPUT_EVDEV
DEVICES +=      \
event0,c,13,64  event1,c,13,65  event2,c,13,66  event3,c,13,67 \
event4,c,13,68  event5,c,13,69  event6,c,13,70  event7,c,13,71 \
event8,c,13,72  event9,c,13,73  event10,c,13,74  event11,c,13,75 \
event12,c,13,76  event13,c,13,77  event14,c,13,78  event15,c,13,79
endif

ifdef CONFIG_VIDEO_DEV
DEVICES +=	\
	video0,c,81,0
endif

ifdef CONFIG_SPI_SPIDEV
DEVICES +=	\
	spidev_scarf_dspblock,c,153,0 \
	spidev_scarf_global,c,153,1 \
	spidev_scarf_local,c,153,2 \
	spidev_aes,c,153,2 \
	spidev_oled,c,153,3
endif

# Add the AVB packet engine device nodes
DEVICES += packetizer0,c,230,0
DEVICES += depacketizer0,c,232,0
DEVICES += depacketizer1,c,232,1
DEVICES += redswitch0,c,248,0
DEVICES += ptp0,c,233,0

# Add the CAL sample_pointers peripheral device node
DEVICES += samplepointers0,c,251,0

# Add the Marvell 88E6350R Ethernet Switch peripheral device node
DEVICES += mvEthSwitch0,c,255,0

I2C_MAJOR = 89

ifndef CONFIG_SYSTEM_ROOT_PASSWD
CONFIG_SYSTEM_ROOT_PASSWD := "root"
endif
CRYPT_PASSWD       := $(shell perl $(ROOTDIR)/tools/crypt.pl $(CONFIG_SYSTEM_ROOT_PASSWD))

all:
	dirs=$(DIRS) ; \
	for i in $$dirs ; do  $(MAKE) -C $$i || exit $? ; done

clean:
	-dirs=$(DIRS) ; \
	for i in $$dirs; do [ ! -d $$i ] || $(MAKE) -C $$i clean; done
	-rm -f mkcramfs

romfs:
	[ -d $(ROMFSDIR)/$$i ] || mkdir -p $(ROMFSDIR)
	for i in $(ROMFS_DIRS); do \
		[ -d $(ROMFSDIR)/$$i ] || mkdir -p $(ROMFSDIR)/$$i; \
	done
	for i in $(DIRS); do $(MAKE) -C $$i romfs || exit $? ; done

	mkdir -p $(ROMFSDIR)/etc
	echo "version = $(RELEASE_VERSION)" > $(ROMFSDIR)/etc/firmware_version
	echo -n "date = " >> $(ROMFSDIR)/etc/firmware_version
	date >> $(ROMFSDIR)/etc/firmware_version

	mkdir -p $(ROMFSDIR)/etc/init.d
	mkdir -p $(ROMFSDIR)/etc/rc.d
	mkdir -p $(ROMFSDIR)/dev/pts
	mkdir -p $(ROMFSDIR)/etc/avahi.orig
	mkdir -p $(ROMFSDIR)/etc/dmitripaneld

	$(ROMFSINST) -s /dev/ttyUL2 /dev/ttyMIDI
	$(ROMFSINST) -s /var/tmp/dhcpc /etc/dhcpc
	$(ROMFSINST) -s /var/tmp/dhcpc/resolv.conf /etc/resolv.conf
	$(ROMFSINST) -s /var/tmp/avahi /etc/avahi
	$(ROMFSINST) -s /var/tmp/identity /etc/identity
	$(ROMFSINST) -s /var/tmp/persist /etc/persist
	$(ROMFSINST) -s /proc/mounts /etc/mtab
	$(ROMFSINST) $(COMMON)/etc/hosts /etc/hosts
	$(ROMFSINST) $(COMMON)/etc/inetd.conf /etc/inetd.conf
	$(ROMFSINST) $(COMMON)/etc/rc/hostname /etc/init.d/hostname
	$(ROMFSINST) -s /etc/init.d/hostname /etc/rc.d/S10hostname
	$(ROMFSINST) $(COMMON)/etc/rc/mountfs /etc/init.d/mountfs
	$(ROMFSINST) -s /etc/init.d/mountfs /etc/rc.d/S20mountfs
	$(ROMFSINST) $(COMMON)/etc/rc/network /etc/init.d/network
	$(ROMFSINST) -s /etc/init.d/network /etc/rc.d/S30network
	$(ROMFSINST) $(COMMON)/etc/avahi-daemon.conf /etc/avahi.orig/avahi-daemon.conf
	$(ROMFSINST) $(COMMON)/etc/avahi-autoipd.action /etc/avahi.orig/avahi-autoipd.action
	$(ROMFSINST) $(COMMON)/etc/rc/avahi-autoipd-wrapper /bin/avahi-autoipd-wrapper
	$(ROMFSINST) $(COMMON)/etc/logo-1.bin /etc/dmitripaneld/logo-1.bin
	$(ROMFSINST) $(COMMON)/etc/nixie-16x24-font.bin /etc/dmitripaneld/nixie-16x24-font.bin
	$(ROMFSINST) $(COMMON)/etc/nixie-alternative-bold-8x12-font.bin /etc/dmitripaneld/nixie-alternative-bold-8x12-font.bin
	$(ROMFSINST) $(COMMON)/etc/nixie-bold-16x24-font.bin /etc/dmitripaneld/nixie-bold-16x24-font.bin
	$(ROMFSINST) $(COMMON)/etc/nixie-bold-7x11-font.bin /etc/dmitripaneld/nixie-bold-7x11-font.bin
	$(ROMFSINST) $(COMMON)/etc/nixie-bold-8x12-font.bin /etc/dmitripaneld/nixie-bold-8x12-font.bin
	$(ROMFSINST) $(COMMON)/etc/bitstream-vera-sans-mono-8x12-font.bin /etc/dmitripaneld/bitstream-vera-sans-mono-8x12-font.bin

	# Symbolic link for fstab into configurable area
	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD \
		-s /etc/config/fstab /etc/fstab

	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD \
		-s /etc/config/passwd /etc/passwd

ifndef CONFIG_SYSTEM_ROOTFS_JFFS2
	for i in $(DEVICES); do \
		touch $(ROMFSDIR)/dev/@$$i; \
	done
	for i in $(FLASH_DEVICES); do \
		touch $(ROMFSDIR)/dev/flash/@$$i; \
	done
	$(ROMFSINST) -s /var/tmp /tmp

	# make I2C nodes if necessary 
ifdef CONFIG_I2C
	for i in 0 1 2 3 4 5 6 7; do \
		touch $(ROMFSDIR)/dev/@i2c-$$i,c,$(I2C_MAJOR),$$i; \
	done
endif #IIC

else
	$(ROMFSINST) $(COMMON)/etc/rc/checkroot /etc/init.d/checkroot
	$(ROMFSINST) -s /etc/init.d/checkroot /etc/rc.d/S01checkroot
endif

ifdef CONFIG_DEBUG_FS
	$(ROMFSINST) $(COMMON)/etc/rc/debugfs /etc/init.d/debugfs
	$(ROMFSINST) -s /etc/init.d/debugfs /etc/rc.d/S02debufs
endif

ifdef CONFIG_USB_DEVICEFS
	$(ROMFSINST) $(COMMON)/etc/rc/usbfs /etc/init.d/usbfs
	$(ROMFSINST) -s /etc/init.d/usbfs /etc/rc.d/S02usbfs
endif

	$(ROMFSINST) -A "^root:" -a "root:${CRYPT_PASSWD}:0:0::/:/bin/sh" $(ETCDIR)/passwd
	$(ROMFSINST) -A "^root:" -a "root:x:0:root" $(ETCDIR)/group

	$(ROMFSINST) -A "^avahi:" -a "avahi:x:200:200:Avahi Daemon:/:/bin/false" $(ETCDIR)/passwd
	$(ROMFSINST) -A "^avahi:" -a "avahi:x:200:avahi" $(ETCDIR)/group

	$(ROMFSINST) -A "^avahi-autoipd:" -a "avahi-autoipd:x:201:201:Avahi Auto IP Daemon:/:/bin/false" $(ETCDIR)/passwd
	$(ROMFSINST) -A "^avahi-autoipd:" -a "avahi-autoipd:x:201:avahi-autoipd" $(ETCDIR)/group

ifdef CONFIG_USER_INIT_INIT
ifdef CONFIG_SERIAL_8250
	$(ROMFSINST) -e CONFIG_USER_AGETTY_AGETTY \
			-a "ttyS0:vt100:/bin/agetty 115200 ttyS0 " /etc/inittab
endif
ifdef CONFIG_SERIAL_UARTLITE
	$(ROMFSINST) -e CONFIG_USER_AGETTY_AGETTY \
			-a "ttyUL0:vt100:/bin/agetty 115200 ttyUL0 " /etc/inittab
endif
endif

ifdef CONFIG_USER_BUSYBOX_INIT
	$(ROMFSINST) $(COMMON)/etc/rc/sysinit /etc/rc.sysinit
	$(ROMFSINST) $(COMMON)/etc/rc/reboot /etc/rc.reboot
	chmod +x $(ROMFSDIR)/etc/rc.sysinit || true
	chmod +x $(ROMFSDIR)/etc/rc.reboot || true
ifdef CONFIG_USER_AGETTY_AGETTY 
	$(ROMFSINST) $(COMMON)/etc/inittab.agetty /etc/inittab
else
	$(ROMFSINST) $(COMMON)/etc/inittab.sh /etc/inittab
endif
endif


############################################

ifdef CONFIG_NET

# setup /etc/init.d/network and other network-related files
#	$(ROMFSINST) -s /etc/init.d/network /etc/rc.d/S40network
	$(ROMFSINST) ../../Generic/romfs/etc/services /etc/services
endif
	# Ensure init.d scripts are executable
	chmod +x $(ROMFSDIR)/etc/init.d/* || true

$(LINUXBIN): $(ROOTDIR)/$(LINUXDIR)/linux
	[ -d $(IMAGEDIR) ] || mkdir -p $(IMAGEDIR)
	$(CROSS)objcopy -O binary $(LINUX) $@

$(ROMFSIMG): $(LINUXBIN)
ifdef CONFIG_BINFMT_ZFLAT
	$(CROSS)flthdr -z $(ROMFSDIR)/bin/*
endif
ifdef USE_CRAMFS
	./mkcramfs -z -r $(ROMFSDIR) $(ROMFSIMG)
else
	genromfs -v -V "ROMdisk" -x *~ -f $(ROMFSIMG) -d $(ROMFSDIR)
endif
	cat $(LINUXBIN) $(ROMFSIMG) > $(IMAGE)

$(JFFS2IMG):
# current mkfs.jffs2 in uclinux dist doesn't work.
# assume we have good working mkfs.jffs2 installed on host system
	mkfs.jffs2 --big-endian --pad=1966080 --squash --eraseblock=65536 \
                   --devtable jffs2_devtable.txt --root=$(ROMFSDIR) --output=$(JFFS2IMG)
	cp $(LINUXBIN) $(IMAGE)
	dd if=$(JFFS2IMG) of=$(IMAGE) bs=1024 seek=1472

mkcramfs: $(ROOTDIR)/user/cramfs/mkcramfs.c
	$(HOSTCC) -o $@ -I$(ROOTDIR)/$(LINUXDIR)/include $< -lz

