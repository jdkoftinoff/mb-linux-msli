#!/bin/bash -e

if [ -z "${DMITRI_IO_MAINT_DOWNLOAD_BIT}" ]; then
  DMITRI_IO_MAINT_DOWNLOAD_BIT="${HOME}/dmitri_io_0/implementation/download.bit"
fi

if [ -z "${DMITRI_IO_GNET_DOWNLOAD_BIT}" ]; then
  DMITRI_IO_GNET_DOWNLOAD_BIT="${HOME}/dmitri_io_1_GNET/mblaze/implementation/download.bit"
fi

if [ -z "${DMITRI_IO_AVB_DOWNLOAD_BIT}" ]; then
  DMITRI_IO_AVB_DOWNLOAD_BIT="${HOME}/labx-ip/IO_Link/FPGA/Synthesis/IO_Link_top.bit"
fi

if [ -z "${DMITRI_DGPIO_AVB_DOWNLOAD_BIT}" ]; then
  DMITRI_DGPIO_AVB_DOWNLOAD_BIT="${HOME}/labx-ip/IO_Link/FPGA_DGPIO/Synthesis/IO_Link_top.bit"
fi

if [ -z "${DMITRI_IO_MAINT_DTS}" ]; then
  DMITRI_IO_MAINT_DTS="xilinx.dts"
fi

if [ -z "${DMITRI_IO_GNET_DTS}" ]; then
  DMITRI_IO_GNET_DTS="xilinx-gnet.dts"
fi

if [ -z "${DMITRI_IO_AVB_DTS}" ]; then
  DMITRI_IO_AVB_DTS="xilinx-avb.dts"
fi

if [ -z "${DMITRI_DGPIO_AVB_DTS}" ]; then
  DMITRI_DGPIO_AVB_DTS="xilinx-avb-dgpio.dts"
fi

if [ -z "${MBBL_ELF}" ]; then
  MBBL_ELF="${HOME}/dmitri_io_1_GNET/mblaze/SDK/SDK_Workspace/mbbl/Release/mbbl.elf"
fi

echo "Maintenance device tree (${DMITRI_IO_MAINT_DTS})..."
../dtc/dtc -f -o dt-maint.dtb -O dtb "${DMITRI_IO_MAINT_DTS}" 2>/dev/null
echo "GNET device tree (${DMITRI_IO_GNET_DTS})..."
../dtc/dtc -f -o dt-gnet.dtb -O dtb "${DMITRI_IO_GNET_DTS}" 2>/dev/null
echo "AVB device tree (${DMITRI_IO_AVB_DTS})..."
../dtc/dtc -f -o dt-avb.dtb -O dtb "${DMITRI_IO_AVB_DTS}" 2>/dev/null
echo "AVB device tree (${DMITRI_DGPIO_AVB_DTS})..."
../dtc/dtc -f -o dt-avb-dgpio.dtb -O dtb "${DMITRI_DGPIO_AVB_DTS}" 2>/dev/null

echo "Kernel..."
gzip -9c ../uClinux-dist/linux-2.6.x/arch/microblaze/boot/linux.bin  > linux.bin.gz

echo "Ramdisk..."
genromfs -f romfs.bin -d ../uClinux-dist/romfs
rm -f romfs.bin.gz
gzip -9 romfs.bin

../../mbbl/mbbl-mkbootimage/pad-file -b 256 -s 8 linux.bin.gz
../../mbbl/mbbl-mkbootimage/pad-file -b 256 -s 8 logo-1.bin.gz
../../mbbl/mbbl-mkbootimage/pad-file -b 256 -s 8 8x12-font.bin.gz
../../mbbl/mbbl-mkbootimage/pad-file -b 256 -s 8 16x24-font.bin.gz
../../mbbl/mbbl-mkbootimage/pad-file -b 256 -s 8 romfs.bin.gz
../../mbbl/mbbl-mkbootimage/pad-file -b 256 -s 8 dt-maint.dtb

if [ -f "${DMITRI_IO_MAINT_DOWNLOAD_BIT}" ]
    then 
    ../../mbbl/mbbl-mkbootimage/pad-file -x -b 256 -s 32 "${DMITRI_IO_MAINT_DOWNLOAD_BIT}"
    echo "Maintenance mode binary image..."
    ../../mbbl/mbbl-mkbootimage/mbbl-imagetool -o firmware0.bin -s 0 -b "${DMITRI_IO_MAINT_DOWNLOAD_BIT}" -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt-maint.dtb -i identity.txt
    echo "Maintenance mode MCS image..." 
    ../mcsbin/mcsbin -m -o 0 -y firmware0.bin firmware0.mcs 
 
    echo "Maintenance mode and DGPIO tar file..."
    if [ -d update ]
	then
	rm -rf update
    fi
    mkdir -p update
    cp "${DMITRI_IO_MAINT_DOWNLOAD_BIT}" update/download.bit
    cp dt-maint.dtb update/dt.dtb
    cp linux.bin.gz logo-1.bin.gz 8x12-font.bin.gz 16x24-font.bin.gz romfs.bin.gz  identity.txt update
    tar czf firmware-maint.tar.gz update
    echo "done: Output file at $PWD/firmware-maint.tar.gz"
else
    echo "***********************************************************************"
    echo " FPGA bitstream file"
    echo " ${DMITRI_IO_MAINT_DOWNLOAD_BIT} is not found."
    echo " To build firmware-maint.tar.gz, please generate this file from XPS"
    echo " and re-run $0"
    echo "***********************************************************************"
fi

if [ -f "${DMITRI_IO_GNET_DOWNLOAD_BIT}" -a -f "${MBBL_ELF}" ]
    then
    ../../mbbl/mbbl-mkbootimage/pad-file -x -b 256 -s 32 "${DMITRI_IO_GNET_DOWNLOAD_BIT}"
    ../../mbbl/mbbl-mkbootimage/pad-file -b 256 -s 8 "${MBBL_ELF}"
    echo "GNET tar file..."
    if [ -d update ]
	then
	rm -rf update
    fi
    mkdir -p update
    cp "${DMITRI_IO_GNET_DOWNLOAD_BIT}" update/download.bit
    cp "${MBBL_ELF}" update/mbbl.elf
    cp dt-gnet.dtb update/dt.dtb
    cp linux.bin.gz logo-1.bin.gz 8x12-font.bin.gz 16x24-font.bin.gz romfs.bin.gz  identity.txt update
    tar czf firmware-gnet.tar.gz update
    echo "done: Output file at $PWD/firmware-gnet.tar.gz"
else
    echo "***********************************************************************"
    echo " FPGA bitstream file"
    echo " ${DMITRI_IO_GNET_DOWNLOAD_BIT} or ${MBBL_ELF} is not found."
    echo " To build firmware-gnet.tar.gz, please generate file(s) from XPS"
    echo " and re-run $0"
    echo "***********************************************************************"
fi

if [ -f "${DMITRI_IO_AVB_DOWNLOAD_BIT}" ]
    then
    ../../mbbl/mbbl-mkbootimage/pad-file -x -b 256 -s 32 "${DMITRI_IO_AVB_DOWNLOAD_BIT}"
    echo "AVB tar file..." "${DMITRI_IO_AVB_DOWNLOAD_BIT}"
    if [ -d update ]
	then
	rm -rf update
    fi
    mkdir -p update
    cp "${DMITRI_IO_AVB_DOWNLOAD_BIT}" update/download.bit
    cp dt-avb.dtb update/dt.dtb
    cp linux.bin.gz logo-1.bin.gz 8x12-font.bin.gz 16x24-font.bin.gz romfs.bin.gz  identity.txt update
    tar czf firmware-avb.tar.gz update
    echo "done: Output file at $PWD/firmware-avb.tar.gz"
else
    echo "***********************************************************************"
    echo " FPGA bitstream file"
    echo " ${DMITRI_IO_AVB_DOWNLOAD_BIT} is not found."
    echo " To build firmware-avb.tar.gz, please generate this file from XPS"
    echo " and re-run $0"
    echo "***********************************************************************"
fi

if [ -f "${DMITRI_DGPIO_AVB_DOWNLOAD_BIT}" ]
    then
    ../../mbbl/mbbl-mkbootimage/pad-file -x -b 256 -s 32 "${DMITRI_DGPIO_AVB_DOWNLOAD_BIT}"
    echo "AVB tar file... " ${DMITRI_DGPIO_AVB_DOWNLOAD_BIT}
    if [ -d update ]
	then
	rm -rf update
    fi
    mkdir -p update
    cp "${DMITRI_DGPIO_AVB_DOWNLOAD_BIT}" update/download.bit
    cp dt-avb-dgpio.dtb update/dt.dtb
    cp linux.bin.gz logo-1.bin.gz 8x12-font.bin.gz 16x24-font.bin.gz romfs.bin.gz  identity.txt update
    tar czf firmware-dgpio-avb.tar.gz update
    echo "done: Output file at $PWD/firmware-dgpio-avb.tar.gz"
else
    echo "***********************************************************************"
    echo " FPGA bitstream file"
    echo " ${DMITRI_DGPIO_AVB_DOWNLOAD_BIT} is not found."
    echo " To build firmware-dgpio-avb.tar.gz, please generate this file from XPS"
    echo " and re-run $0"
    echo "***********************************************************************"
fi

