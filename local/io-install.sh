#!/bin/sh

if [ "x$EDITOR" = "x" ]
    then
    if [ -x /usr/bin/nano ]
        then
        EDITOR="nano -w"
        else
        EDITOR="vi"
    fi
fi

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
if [ ! -f firmware.tar.gz ]
    then
    echo "Firmware tar file /tmp/firmware.tar.gz not found"
    exit 1
fi

echo "Extracting firmware from tar file"
/bin/tar xvzf firmware.tar.gz

if [ ! -d update ]
    then
    echo "Failed to extract firmware from tar file"
    exit 1
fi

cd update
/bin/mbbl-imagetool -s 0 -I /dev/mtd0 -i identity.txt

if [ -f mbbl.elf ]
    then
    MBBL_ELF="-e mbbl.elf"
    else
    MBBL_ELF=""
fi

if [ "$1" = "MAINT" -o "$2" = "MAINT" ]
    then
    echo -en "\x1b[0mPress \x1b[7;1m<Enter>\x1b[0m, edit identity file, " 
    echo -en "exit from "

    if [ "$EDITOR" = "vi" ]
        then
        echo -e "vi editor with \x1b[7;1m<Esc>\x1b[0;1m :wq \x1b[7;1m<Enter>\x1b[0m"
        else
        echo -e "nano editor with \x1b[7;1m<Ctrl> X\x1b[0m"
    fi
    stty -echo
    read
    stty sane
    $EDITOR identity.txt
fi

if [ "$1" = "MAINT" -o "$2" = "MAINT" ]
    then
    echo "Installing maintenance mode firmware"
    /bin/mbbl-imagetool -s 0 -o /dev/mtd0 -b download.bit -d dt.dtb \
	${MBBL_ELF} -k linux.bin.gz -r romfs.bin.gz -l logo-1.bin.gz \
	-f 8x12-font.bin.gz -f 16x24-font.bin.gz -i identity.txt
fi

if [ "$1" = "REGULAR" -o "$2" = "REGULAR" ]
    then
    echo "Installing regular mode firmware"
    /bin/mbbl-imagetool -s 0x800000 -o /dev/mtd0 -b download.bit -d dt.dtb \
	${MBBL_ELF} -k linux.bin.gz -r romfs.bin.gz -l logo-1.bin.gz \
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
