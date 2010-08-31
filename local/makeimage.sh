#/bin/sh
echo -ne "device tree..." \
&& ../dtc/dtc -o dt.dtb -O dtb xilinx.dts 2>/dev/null \
&& echo -ne "\x1b[3D, kernel..." \
&& gzip -9c ../uClinux-dist/linux-2.6.x/arch/microblaze/boot/linux.bin  > linux.bin.gz \
&& echo -ne "\x1b[3D, ramdisk..." \
&& genromfs -f romfs.bin -d ../uClinux-dist/romfs && rm -f romfs.bin.gz && gzip -9 romfs.bin \
&& echo -ne "\x1b[3D, final binary image..." \
&& ../mbbl-mkbootimage/mkbootimage -o bootimage.bin -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt.dtb \
&& if [ -f "${HOME}"/dmitri_io_0/implementation/download.bit ]
then 
echo -ne "\x1b[3D,   \nextended final binary image 0..." \
&& ../mbbl-mkbootimage/mbbl-imagetool -o firmware0.bin -b "${HOME}"/dmitri_io_0/implementation/download.bit -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt.dtb -i identity.txt \
&& echo -ne "\x1b[3D,   \nextended final binary image 1..." \
&& ../mbbl-mkbootimage/mbbl-imagetool -o firmware1.bin -s 0x800000 -b "${HOME}"/dmitri_io_0/implementation/download.bit -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt.dtb -i identity.txt \
&& echo -ne "\x1b[3D,   \nfinal MCS image..." \
&& ../mcsbin/mcsbin -m -o 3145728 -y bootimage.bin bootimage.mcs \
&& echo -ne "\x1b[3D,   \nextended final MCS image 0..." \
&& ../mcsbin/mcsbin -m -o 0 -y firmware0.bin firmware0.mcs \
&& echo -ne "\x1b[3D,   \nextended final MCS image 1..." \
&& ../mcsbin/mcsbin -m -o 8388608 -y firmware1.bin firmware1.mcs \
&& echo -ne "\x1b[3D,   \ntar file..." \
&& (mkdir update 2>/dev/null ; true ) && cp "${HOME}"/dmitri_io_0/implementation/download.bit linux.bin.gz logo-1.bin.gz 8x12-font.bin.gz 16x24-font.bin.gz romfs.bin.gz dt.dtb identity.txt update && tar czf firmware.tar.gz update \
&& echo -e "\x1b[3D done" || echo -e "\x1b[3D failed"
else
echo -e "\x1b[3D done"
echo "***********************************************************************"
echo " Boot image bootimage.bin and bootimage.mcs build succeeded, however"
echo " FPGA bitstream file"
echo " ${HOME}/dmitri_io_0/implementation/download.bit is not found."
echo " To build firmware0.bin, firmware0.mcs, firmware1.bin, firmware1.mcs"
echo " and firmware.tar.gz, please generate this file from XPS and re-run"
echo " $0"
echo "***********************************************************************"
fi
