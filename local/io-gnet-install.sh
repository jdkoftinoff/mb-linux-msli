#!/bin/sh

if [ "$1" != "MAINT" -a "$1" != "REGULAR" -a "$2" != "MAINT" -a "$2" != "REGULAR" ]
    then
    if [ "x$1" = "x" -a "x$2" = "x" ]
	then
	echo "No operation requested"
    else
	echo "Invalid operation"
    fi
    echo "Usage: $0 [REGULAR] [MAINT]"
    exit 1
fi

cd /tmp
if [ ! -f firmware-gnet.tar.gz ]
    then
    echo "Firmware tar file /tmp/firmware-gnet.tar.gz not found"
    exit 1
fi

echo "Extracting firmware from tar file"
/bin/tar xvzf firmware-gnet.tar.gz

if [ ! -d update ]
    then
    echo "Failed to extract firmware from tar file"
    exit 1
fi

cd update
/bin/mbbl-imagetool -s 0 -I /dev/mtd0 -i identity.txt

if [ "$1" = "MAINT" -o "$2" = "MAINT" ]
    then
    echo "Press <Enter>, edit identity file, exit from vi editor with <Esc>:wq"
    read
    vi identity.txt
fi

if [ "$1" = "MAINT" -o "$2" = "MAINT" ]
    then
    echo "Installing maintenance mode firmware"
    /bin/mbbl-imagetool -s 0 -o /dev/mtd0 -b download.bit -d dt.dtb \
	-e mbbl.elf -k linux.bin.gz -r romfs.bin.gz -l logo-1.bin.gz \
	-f 8x12-font.bin.gz -f 16x24-font.bin.gz -i identity.txt
fi

if [ "$1" = "REGULAR" -o "$2" = "REGULAR" ]
    then
    echo "Installing regular mode firmware"
    /bin/mbbl-imagetool -s 0x800000 -o /dev/mtd0 -b download.bit -d dt.dtb \
	-e mbbl.elf -k linux.bin.gz -r romfs.bin.gz -l logo-1.bin.gz \
	-f 8x12-font.bin.gz -f 16x24-font.bin.gz -i identity.txt
fi

if [ -x /bin/mtd-storage ]
    then
    echo "Updating persistent user storage"
    cd /
    if [ "$1" = "MAINT" -o "$2" = "MAINT" ]
	then
	/bin/mtd-storage -F -o /dev/mtd0 -s 0 -e 0x7fffff
    fi
    if [ "$1" = "REGULAR" -o "$2" = "REGULAR" ]
	then
	/bin/mtd-storage -F -o /dev/mtd0 -s 0x800000 -e 0xffffff var/tmp/persist
    fi
fi
