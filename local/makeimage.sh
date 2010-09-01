#! /bin/bash -e

if [ -z "${DMITRI_IO_DOWNLOAD_BIT}" ]; then
  DMITRI_IO_DOWNLOAD_BIT="${HOME}/dmitri_io_0/implementation/download.bit"
fi

echo "device tree..." 
../dtc/dtc -o dt.dtb -O dtb xilinx.dts 2>/dev/null 
echo "kernel..." 
gzip -9c ../uClinux-dist/linux-2.6.x/arch/microblaze/boot/linux.bin  > linux.bin.gz 
echo "ramdisk..." 
genromfs -f romfs.bin -d ../uClinux-dist/romfs
rm -f romfs.bin.gz
gzip -9 romfs.bin 
echo "final binary image..." 
mkbootimage -o bootimage.bin -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt.dtb 
if [ -f "${DMITRI_IO_DOWNLOAD_BIT}" ]
then 
  echo "extended final binary image 0..." 
  ../mbbl-mkbootimage/mbbl-imagetool -o firmware0.bin -b "${DMITRI_IO_DOWNLOAD_BIT}" -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt.dtb -i identity.txt 
  echo "extended final binary image 1..." 
  ../mbbl-mkbootimage/mbbl-imagetool -o firmware1.bin -s 0x800000 -b "${DMITRI_IO_DOWNLOAD_BIT}" -k linux.bin.gz -l logo-1.bin.gz -f 8x12-font.bin.gz -f 16x24-font.bin.gz -r romfs.bin.gz -d dt.dtb -i identity.txt 
  echo "final MCS image..." 
  ../mcsbin/mcsbin -m -o 3145728 -y bootimage.bin bootimage.mcs 
  echo "extended final MCS image 0..." 
  ../mcsbin/mcsbin -m -o 0 -y firmware0.bin firmware0.mcs 
  echo "extended final MCS image 1..." 
  ../mcsbin/mcsbin -m -o 8388608 -y firmware1.bin firmware1.mcs 
  echo "tar file..." 
  mkdir -p update
  cp "${DMITRI_IO_DOWNLOAD_BIT}" linux.bin.gz logo-1.bin.gz 8x12-font.bin.gz 16x24-font.bin.gz romfs.bin.gz dt.dtb identity.txt update
  tar czf firmware.tar.gz update 
  echo "done" 
else
  echo -e "done"
  echo "***********************************************************************"
  echo " Boot image bootimage.bin and bootimage.mcs build succeeded, however"
  echo " FPGA bitstream file"
  echo " ${DMITRI_IO_DOWNLOAD_BIT} is not found."
  echo " To build firmware0.bin, firmware0.mcs, firmware1.bin, firmware1.mcs"
  echo " and firmware.tar.gz, please generate this file from XPS and re-run"
  echo " $0"
  echo "***********************************************************************"
fi

