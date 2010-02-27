#! /bin/bash -e 

MODULE_ADDRESS="$1"

. prepare.sh
(cd uClinux-dist && make)
(cd local && ./makeimage.sh)

if [ -n "$MODULE_ADDRESS" ]; then 
  curl -T local/bootimage.bin ftp://root:slipknot@$MODULE_ADDRESS/tmp/
  echo "file transferred, now telnet to $MODULE_ADDRESS and run: "
  echo "flashw -u -o 3145728 -f /tmp/bootimage.bin /dev/mtd0"
fi

