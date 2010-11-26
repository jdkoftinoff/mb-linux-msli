#/bin/sh
echo -ne "device tree..." \
&& ../dtc/dtc -o dt-gnet.dtb -O dtb linux-gnet.dts 2>/dev/null \
&& echo -ne "\x1b[3D, kernel..." \
&& gzip -9c ../uClinux-dist/linux-2.6.x/arch/microblaze/boot/linux.bin  > linux.bin.gz \
&& echo -ne "\x1b[3D, ramdisk..." \
&& genromfs -f romfs.bin -d ../uClinux-dist/romfs && rm -f romfs.bin.gz && gzip -9 romfs.bin \
&& echo -ne "\x1b[3D, final binary image..." \
&& ../mbbl-mkbootimage/mkbootimage -o bootimage-gnet.bin -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt-gnet.dtb \
&& if [ -f "${HOME}"/dmitri_io_1_GNET/mblaze_download.bit ]
then 
echo -ne "\x1b[3D,   \nextended final binary image 0..." \
&& ../mbbl-mkbootimage/mbbl-imagetool -o firmware-gnet0.bin -b "${HOME}"/dmitri_io_1_GNET/mblaze_download.bit -e "${HOME}"/dmitri_io_1_GNET/mblaze/SDK/SDK_Workspace/mbbl/Release/mbbl.elf -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt-gnet.dtb -i identity.txt \
&& echo -ne "\x1b[3D,   \nextended final binary image 1..." \
&& ../mbbl-mkbootimage/mbbl-imagetool -o firmware-gnet1.bin -s 0x800000 -b "${HOME}"/dmitri_io_1_GNET/mblaze_download.bit -e "${HOME}"/dmitri_io_1_GNET/mblaze/SDK/SDK_Workspace/mbbl/Release/mbbl.elf -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt-gnet.dtb -i identity.txt \
&& echo -ne "\x1b[3D,   \nfinal MCS image..." \
&& ../mcsbin/mcsbin -m -o 3145728 -y bootimage-gnet.bin bootimage-gnet.mcs \
&& echo -ne "\x1b[3D,   \nextended final MCS image 0..." \
&& ../mcsbin/mcsbin -m -o 0 -y firmware-gnet0.bin firmware-gnet0.mcs \
&& echo -ne "\x1b[3D,   \nextended final MCS image 1..." \
&& ../mcsbin/mcsbin -m -o 8388608 -y firmware-gnet1.bin firmware-gnet1.mcs \
&& echo -ne "\x1b[3D,   \ntar file..." \
&& (mkdir update 2>/dev/null ; true ) && cp "${HOME}"/dmitri_io_1_GNET/mblaze_download.bit "${HOME}"/dmitri_io_1_GNET/mblaze/SDK/SDK_Workspace/mbbl/Release/mbbl.elf linux.bin.gz logo-1.bin.gz 8x12-font.bin.gz 16x24-font.bin.gz romfs.bin.gz dt-gnet.dtb identity.txt update && mv update/mblaze_download.bit update/download.bit && mv update/dt-gnet.dtb update/dt.dtb && tar czf firmware-gnet.tar.gz update \
&& echo -e "\x1b[3D done" || echo -e "\x1b[3D failed"
else
echo -e "\x1b[3D done"
echo "***********************************************************************"
echo " Boot image bootimage.bin and bootimage.mcs build succeeded, however"
echo " FPGA bitstream file"
echo " ${HOME}/dmitri_io_1_GNET/mblaze_download.bit is not found."
echo " To build firmware-gnet0.bin, firmware-gnet0.mcs,
echo " firmware-gnet1.bin, firmware-gnet1.mcs and firmware-gnet.tar.gz,
echo " please generate this file from XPS and re-run"
echo " $0"
echo "***********************************************************************"
fi
