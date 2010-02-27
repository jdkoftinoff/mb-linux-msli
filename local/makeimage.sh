#/bin/sh
echo -ne "device tree..." \
&& ../dtc/dtc -o dt.dtb -O dtb xilinx.dts 2>/dev/null \
&& echo -ne "\x1b[3D, kernel..." \
&& gzip -9c ../uClinux-dist/linux-2.6.x/arch/microblaze/boot/linux.bin  > linux.bin.gz \
&& echo -ne "\x1b[3D, ramdisk..." \
&& genromfs -f romfs.bin -d ../uClinux-dist/romfs && rm -f romfs.bin.gz && gzip -9 romfs.bin \
&& echo -ne "\x1b[3D, final binary image..." \
&& ../mbbl-mkbootimage/mkbootimage -o bootimage.bin -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt.dtb \
&& echo -ne "\x1b[3D, final MCS image..." \
&& ../mcsbin/mcsbin -m -o 3145728 -y bootimage.bin bootimage.mcs \
&& echo -e "\x1b[3D done" || echo -e "\x1b[3D failed"
