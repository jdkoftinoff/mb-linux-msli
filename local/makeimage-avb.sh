#/bin/sh
echo -ne "device tree..." \
&& ../dtc/dtc -f -o dt-avb.dtb -O dtb linux-avb.dts 2>/dev/null \
&& echo -ne "\x1b[3D, kernel..." \
&& gzip -9c ../uClinux-dist/linux-2.6.x/arch/microblaze/boot/linux.bin  > linux.bin.gz \
&& echo -ne "\x1b[3D, ramdisk..." \
&& genromfs -f romfs.bin -d ../uClinux-dist/romfs && rm -f romfs.bin.gz && gzip -9 romfs.bin \
&& echo -ne "\x1b[3D, final binary image..." \
&& ../mbbl-mkbootimage/mkbootimage -o bootimage-avb.bin -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt-avb.dtb \
&& if [ -f "${HOME}"/labx-git/labx-ip/IO_Link/FPGA/Synthesis/IO_Link_top_download.bit ]
then 
echo -ne "\x1b[3D,   \nextended final binary image 0..." \
&& ../mbbl-mkbootimage/mbbl-imagetool -o firmware-avb0.bin -b "${HOME}"/labx-git/labx-ip/IO_Link/FPGA/Synthesis/IO_Link_top_download.bit -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt-avb.dtb -i identity.txt \
&& echo -ne "\x1b[3D,   \nextended final binary image 1..." \
&& ../mbbl-mkbootimage/mbbl-imagetool -o firmware-avb1.bin -s 0x800000 -b "${HOME}"/labx-git/labx-ip/IO_Link/FPGA/Synthesis/IO_Link_top_download.bit -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt-avb.dtb -i identity.txt \
&& echo -ne "\x1b[3D,   \nfinal MCS image..." \
&& ../mcsbin/mcsbin -m -o 3145728 -y bootimage-avb.bin bootimage-avb.mcs \
&& echo -ne "\x1b[3D,   \nextended final MCS image 0..." \
&& ../mcsbin/mcsbin -m -o 0 -y firmware-avb0.bin firmware-avb0.mcs \
&& echo -ne "\x1b[3D,   \nextended final MCS image 1..." \
&& ../mcsbin/mcsbin -m -o 8388608 -y firmware-avb1.bin firmware-avb1.mcs \
&& echo -ne "\x1b[3D,   \ntar file..." \
&& (mkdir update 2>/dev/null ; true ) && cp "${HOME}"/labx-git/labx-ip/IO_Link/FPGA/Synthesis/IO_Link_top_download.bit linux.bin.gz logo-1.bin.gz 8x12-font.bin.gz 16x24-font.bin.gz romfs.bin.gz dt-avb.dtb identity.txt update && mv update/IO_Link_top_download.bit update/download.bit && mv update/dt-avb.dtb update/dt.dtb && tar czf firmware-avb.tar.gz update \
&& echo -e "\x1b[3D done" || echo -e "\x1b[3D failed"
else
echo -e "\x1b[3D done"
echo "***********************************************************************"
echo " Boot image bootimage.bin and bootimage.mcs build succeeded, however"
echo " FPGA bitstream file"
echo " ${HOME}/labx-git/labx-ip/IO_Link/FPGA/Synthesis/IO_Link_top_download.bit is not found."
echo " To build firmware-avb0.bin, firmware-avb0.mcs,
echo " firmware-avb1.bin, firmware-avb1.mcs and firmware-avb.tar.gz,
echo " please generate this file from XPS and re-run"
echo " $0"
echo "***********************************************************************"
fi
