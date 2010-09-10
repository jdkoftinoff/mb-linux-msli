cd /tmp/update
/bin/mbbl-imagetool -s 0x800000 -I /dev/mtd0 -i identity.txt
/bin/mbbl-imagetool -s 0x800000 -o /dev/mtd0 -b download.bit -d dt.dtb -k linux.bin.gz -r romfs.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -i identity.txt

