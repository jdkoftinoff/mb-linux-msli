#
# Makefile -- Build instructions for Harman AVB_EP MicroBlaze system
#

include $(LINUX_CONFIG)
include $(CONFIG_CONFIG)
include $(ARCH_CONFIG)

LINUX	 = $(ROOTDIR)/$(LINUXDIR)/linux 
SQFS_DEVTABLE = $(IMAGEDIR)/sqfs_devtab

COMMON=$(VEND)/$(CONFIG_VENDOR)/$(CONFIG_PRODUCT)/

ROMFS_DIRS = bin dev dev/flash etc home lib mnt mnt/apps mnt/config proc usr var

ROMFS_DIRS-$(CONFIG_SYSFS) += sys

ROMFS_DIRS += $(ROMFS_DIRS-y)

ifdef CONFIG_USER_FLATFSD_FLATFSD
	ETCDIR = /etc/default
else
	ETCDIR = /etc
endif

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

romfs:  $(SQFS_DEVTABLE)
	[ -d $(ROMFSDIR)/$$i ] || mkdir -p $(ROMFSDIR)
	for i in $(ROMFS_DIRS); do \
		[ -d $(ROMFSDIR)/$$i ] || mkdir -p $(ROMFSDIR)/$$i; \
	done

	mkdir -p $(ROMFSDIR)/etc/init.d
	mkdir -p $(ROMFSDIR)/etc/rc.d
	mkdir -p $(ROMFSDIR)/dev/pts
	mkdir -p $(ROMFSDIR)/etc/avahi.orig

	$(ROMFSINST) -s /dev/ttyUL2 /dev/ttyMIDI
	$(ROMFSINST) -s /var/tmp/dhcpc /etc/dhcpc
	$(ROMFSINST) -s /var/tmp/dhcpc/resolv.conf /etc/resolv.conf
	$(ROMFSINST) -s /var/tmp/avahi /etc/avahi
	$(ROMFSINST) -s /proc/mounts /etc/mtab
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

	# Symbolic link for fstab into configurable area
	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD \
		-s /etc/config/fstab /etc/fstab

	$(ROMFSINST) -e CONFIG_USER_FLATFSD_FLATFSD \
		-s /etc/config/passwd /etc/passwd

	$(ROMFSINST) -s /var/tmp /tmp

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

ifdef CONFIG_NET
# setup /etc/init.d/network and other network-related files
#	$(ROMFSINST) -s /etc/init.d/network /etc/rc.d/S40network
	$(ROMFSINST) ../../Generic/romfs/etc/services /etc/services
endif
	# Ensure init.d scripts are executable
	chmod +x $(ROMFSDIR)/etc/init.d/* || true

# Dev table file for squashfs, which is used to
# create dev nodes within the file system, as opposed
# to creating specially-named files within /dev, such
# as "/dev/@tty,c,5,0", which other FS tools convert
# to dev nodes, but squashfs doesn't support.
#
# Format:
#    filename type permissions uid gid major minor
$(SQFS_DEVTABLE): Makefile
	[ -d $(IMAGEDIR) ] || mkdir -p $(IMAGEDIR)
	@echo "/dev/tty           c 666 0 0    5   0" >  $(SQFS_DEVTABLE)
	@echo "/dev/console       c 666 0 0    5   1" >> $(SQFS_DEVTABLE)
	@echo "/dev/mem           c 666 0 0    1   1" >> $(SQFS_DEVTABLE)
	@echo "/dev/kmem          c 666 0 0    1   2" >> $(SQFS_DEVTABLE)
	@echo "/dev/null          c 666 0 0    1   3" >> $(SQFS_DEVTABLE)
	@echo "/dev/zero          c 666 0 0    1   5" >> $(SQFS_DEVTABLE)
	@echo "/dev/random        c 666 0 0    1   8" >> $(SQFS_DEVTABLE)
	@echo "/dev/urandom       c 666 0 0    1   9" >> $(SQFS_DEVTABLE)
	@echo "/dev/ttyS0         c 666 0 0    4  64" >> $(SQFS_DEVTABLE)
	@echo "/dev/ttyS1         c 666 0 0    4  65" >> $(SQFS_DEVTABLE)
	@echo "/dev/ttyS2         c 666 0 0    4  66" >> $(SQFS_DEVTABLE)
	@echo "/dev/ttyS3         c 666 0 0    4  67" >> $(SQFS_DEVTABLE)
	@echo "/dev/ptmx          c 666 0 0    5   2" >> $(SQFS_DEVTABLE)
	@echo "/dev/rtc           c 666 0 0   10 135" >> $(SQFS_DEVTABLE)
	@echo "/dev/nvram         c 666 0 0   10 144" >> $(SQFS_DEVTABLE)
ifdef CONFIG_XILINX_GPIO
	@echo "/dev/gpio          c 666 0 0   10 185" >> $(SQFS_DEVTABLE)
	@echo "/dev/gpio0         c 666 0 0   10 185" >> $(SQFS_DEVTABLE)
	@echo "/dev/gpio1         c 666 0 0   10 186" >> $(SQFS_DEVTABLE)
	@echo "/dev/gpio2         c 666 0 0   10 187" >> $(SQFS_DEVTABLE)
	@echo "/dev/gpio3         c 666 0 0   10 188" >> $(SQFS_DEVTABLE)
	@echo "/dev/gpio4         c 666 0 0   10 189" >> $(SQFS_DEVTABLE)
endif
ifdef CONFIG_MICROBLAZE_FSLFIFO
	@echo "/dev/fslfifo0      c 666 0 0   10 192" >> $(SQFS_DEVTABLE)
	@echo "/dev/fslfifo1      c 666 0 0   10 193" >> $(SQFS_DEVTABLE)
	@echo "/dev/fslfifo2      c 666 0 0   10 194" >> $(SQFS_DEVTABLE)
	@echo "/dev/fslfifo3      c 666 0 0   10 195" >> $(SQFS_DEVTABLE)
	@echo "/dev/fslfifo4      c 666 0 0   10 196" >> $(SQFS_DEVTABLE)
	@echo "/dev/fslfifo5      c 666 0 0   10 197" >> $(SQFS_DEVTABLE)
	@echo "/dev/fslfifo6      c 666 0 0   10 198" >> $(SQFS_DEVTABLE)
	@echo "/dev/fslfifo7      c 666 0 0   10 199" >> $(SQFS_DEVTABLE)
endif
ifdef CONFIG_INPUT_EVDEV
	@echo "/dev/event0        c 666 0 0   13  64" >> $(SQFS_DEVTABLE)
	@echo "/dev/event1        c 666 0 0   13  65" >> $(SQFS_DEVTABLE)
	@echo "/dev/event2        c 666 0 0   13  66" >> $(SQFS_DEVTABLE)
	@echo "/dev/event3        c 666 0 0   13  67" >> $(SQFS_DEVTABLE)
	@echo "/dev/event4        c 666 0 0   13  68" >> $(SQFS_DEVTABLE)
	@echo "/dev/event5        c 666 0 0   13  69" >> $(SQFS_DEVTABLE)
	@echo "/dev/event6        c 666 0 0   13  70" >> $(SQFS_DEVTABLE)
	@echo "/dev/event7        c 666 0 0   13  71" >> $(SQFS_DEVTABLE)
	@echo "/dev/event8        c 666 0 0   13  72" >> $(SQFS_DEVTABLE)
	@echo "/dev/event9        c 666 0 0   13  73" >> $(SQFS_DEVTABLE)
	@echo "/dev/event10       c 666 0 0   13  74" >> $(SQFS_DEVTABLE)
	@echo "/dev/event11       c 666 0 0   13  75" >> $(SQFS_DEVTABLE)
	@echo "/dev/event12       c 666 0 0   13  76" >> $(SQFS_DEVTABLE)
	@echo "/dev/event13       c 666 0 0   13  77" >> $(SQFS_DEVTABLE)
	@echo "/dev/event14       c 666 0 0   13  78" >> $(SQFS_DEVTABLE)
	@echo "/dev/event15       c 666 0 0   13  79" >> $(SQFS_DEVTABLE)
endif
	@echo "/dev/fb0           c 666 0 0   29   0" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock0     b 666 0 0   31   0" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock1     b 666 0 0   31   1" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock2     b 666 0 0   31   2" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock3     b 666 0 0   31   3" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock4     b 666 0 0   31   4" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock5     b 666 0 0   31   5" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock6     b 666 0 0   31   6" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock7     b 666 0 0   31   7" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock8     b 666 0 0   31   8" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock9     b 666 0 0   31   9" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock10    b 666 0 0   31  10" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock11    b 666 0 0   31  11" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock12    b 666 0 0   31  12" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock13    b 666 0 0   31  13" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock14    b 666 0 0   31  14" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdblock15    b 666 0 0   31  15" >> $(SQFS_DEVTABLE)
ifdef CONFIG_VIDEO_DEV
	@echo "/dev/video0        c 666 0 0   81   0" >> $(SQFS_DEVTABLE)
endif
	@echo "/dev/mtd0          c 666 0 0   90   0" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd1          c 666 0 0   90   2" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd2          c 666 0 0   90   4" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd3          c 666 0 0   90   6" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd4          c 666 0 0   90   8" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd5          c 666 0 0   90  10" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd6          c 666 0 0   90  12" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd7          c 666 0 0   90  14" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd8          c 666 0 0   90  16" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd9          c 666 0 0   90  18" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd10         c 666 0 0   90  20" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd11         c 666 0 0   90  22" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd12         c 666 0 0   90  24" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd13         c 666 0 0   90  26" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd14         c 666 0 0   90  28" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtd15         c 666 0 0   90  30" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr0         c 666 0 0   90   1" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr1         c 666 0 0   90   3" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr2         c 666 0 0   90   5" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr3         c 666 0 0   90   7" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr4         c 666 0 0   90   9" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr5         c 666 0 0   90  11" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr6         c 666 0 0   90  13" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr7         c 666 0 0   90  15" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr8         c 666 0 0   90  17" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr9         c 666 0 0   90  19" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr10        c 666 0 0   90  21" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr11        c 666 0 0   90  23" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr12        c 666 0 0   90  25" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr13        c 666 0 0   90  27" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr14        c 666 0 0   90  29" >> $(SQFS_DEVTABLE)
	@echo "/dev/mtdr15        c 666 0 0   90  31" >> $(SQFS_DEVTABLE)
	@echo "/dev/ttyUL0        c 666 0 0  204 187" >> $(SQFS_DEVTABLE)
	@echo "/dev/ttyUL1        c 666 0 0  204 188" >> $(SQFS_DEVTABLE)
	@echo "/dev/ttyUL2        c 666 0 0  204 189" >> $(SQFS_DEVTABLE)
	@echo "/dev/ttyUL3        c 666 0 0  204 190" >> $(SQFS_DEVTABLE)
ifdef CONFIG_UIO
	@echo "/dev/uio0          c 666 0 0  254   0" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio1          c 666 0 0  254   1" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio2          c 666 0 0  254   2" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio3          c 666 0 0  254   3" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio4          c 666 0 0  254   4" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio5          c 666 0 0  254   5" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio6          c 666 0 0  254   6" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio7          c 666 0 0  254   7" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio8          c 666 0 0  254   8" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio9          c 666 0 0  254   9" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio10         c 666 0 0  254  10" >> $(SQFS_DEVTABLE)
	@echo "/dev/uio11         c 666 0 0  254  11" >> $(SQFS_DEVTABLE)
endif
ifdef CONFIG_XILINX_SYSACE                                                  
	@echo "/dev/xsysacea      b 666 0 0  254   0" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea0     b 666 0 0  254   0" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea1     b 666 0 0  254   1" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea2     b 666 0 0  254   2" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea3     b 666 0 0  254   3" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea4     b 666 0 0  254   4" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea5     b 666 0 0  254   5" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea6     b 666 0 0  254   6" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea7     b 666 0 0  254   7" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea8     b 666 0 0  254   8" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea9     b 666 0 0  254   9" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea10    b 666 0 0  254  10" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea11    b 666 0 0  254  11" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea12    b 666 0 0  254  12" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea13    b 666 0 0  254  13" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea14    b 666 0 0  254  14" >> $(SQFS_DEVTABLE)
	@echo "/dev/xsysacea15    b 666 0 0  254  15" >> $(SQFS_DEVTABLE)
endif
	@echo "/dev/packetizer0   c 666 0 0  250   0" >> $(SQFS_DEVTABLE)
	@echo "/dev/depacketizer0 c 666 0 0  252   0" >> $(SQFS_DEVTABLE)
	@echo "/dev/depacketizer1 c 666 0 0  252   1" >> $(SQFS_DEVTABLE)
	@echo "/dev/ptp0          c 666 0 0  253   0" >> $(SQFS_DEVTABLE)

