#!/bin/bash

# Check for most likely missing packages and executables

if ! which gcc > /dev/null 2>&1
 then
 echo "This script requires GCC to be present"
 exit 1
fi

if ! which awk > /dev/null 2>&1
 then
 echo "This script requires awk to be present"
 exit 1
fi

if ! which makeinfo > /dev/null 2>&1
 then
 echo "This script requires texinfo to be present"
 echo "(for GDB compilation procedure)"
 exit 1
fi

# Build xilinx toolchain
if [ ! -d mb_gnu ]
  then
  echo "This script requires mb_gnu from mb-gcc4-msli to be"
  echo "copied or linked under current directory"
  exit 1
fi

echo "Building Binutils / GCC4 / GDB toolchain (Xilinx tree)"
(
cd mb_gnu \
&& bash build_binutils.sh && bash build_gcc.sh && bash build_gdb.sh \
&& bash build_elf2flt.sh && bash build_genromfs.sh
)||(
echo "Build failed, see mb_gnu/build directory for logs"
exit 1
)

echo "Building MCS file converter"
(cd mcsbin \
&& make
) || (
echo "Build failed"
exit 1
)

echo "Building device tree compiler"
(cd dtc \
&& make
) || (
echo "Build failed"
exit 1
)

# Move tools to some convenient location

echo "Installing both toolchains in tools/"

mkdir tools 2>/dev/null

rm -rf tools/gcc4
if [ "`uname -m`" = "x86_64" ]
 then
 LOCALPLATFORM="lin64"
 else
 LOCALPLATFORM="lin"
fi

mv "mb_gnu/release/${LOCALPLATFORM}" tools/gcc4

# Preparing new PATH

CURRDIR=`pwd`

NEWPATH=`(
echo "${PATH}" | tr ':' '\n' | grep -v "${CURRDIR}"
echo -n "${CURRDIR}/tools/gcc4/bin"
) | tr '\n' ':'`

echo "PATH=${NEWPATH}" > prepare.sh

echo "Run \". prepare.sh\" from this directory before cross-compiling for MicroBlaze"
