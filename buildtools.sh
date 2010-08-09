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

if [ ! -x /bin/tcsh ]
 then
 echo "This script requires /bin/tcsh to be present"
 echo "(for Petalogix tools compilation procedure)"
 exit 1
fi

if ! which makeinfo > /dev/null 2>&1
 then
 echo "This script requires texinfo to be present"
 echo "(for GDB compilation procedure)"
 exit 1
fi

if ! which flex > /dev/null 2>&1
 then
 echo "This script requires flex to be present"
 exit 1
fi

if ! which bison > /dev/null 2>&1
 then
 echo "This script requires bison to be present"
 exit 1
fi

if ! which gmake > /dev/null 2>&1
 then
 echo "This script requires gmake to be present (if make is gnu make make a symlink for gmake)"
 exit 1
fi

GCC3_DIR="microblaze-toolchain-sources"
GCC4_DIR="mb_gnu"

if [ ! -d "$GCC3_DIR" ]; then
 echo "$GCC3_DIR should link to the gcc 3 toolchain source"
 exit 1;
fi

if [ ! -d "$GCC4_DIR" ]; then
 echo "$GCC4_DIR should link to the gcc 4 toolchain source"
 exit 1;
fi

# Build xilinx toolchain

echo "Building Binutils / GCC4 / GDB toolchain (Xilinx tree)"
(
cd "$GCC4_DIR" \
&& bash build_binutils.sh && bash build_elf2flt.sh && bash build_gcc.sh && bash build_gdb.sh
)||(
echo "Build failed, see mb_gnu/build directory for logs"
exit 1
)

# Build Petalogix toolchain

echo "Building Binutils / GCC3 / GDB / uCLinux toolchain (Petalogix tree)"
(
cd "$GCC3_DIR" \
&& csh do_everything.csh
) || (
echo "Build failed"
exit 1
)

echo "Building boot image maker"
(cd mbbl-mkbootimage \
&& make
) || (
echo "Build failed"
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

rm -rf tools/gcc4 tools/gcc3
if [ "`uname -m`" = "x86_64" ]
 then
 LOCALPLATFORM="lin64"
 else
 LOCALPLATFORM="lin"
fi

mv "mb_gnu/release/${LOCALPLATFORM}" tools/gcc4
mv "microblaze-toolchain-sources/release/${LOCALPLATFORM}/microblaze/" tools/gcc3
(
cd tools/gcc4/bin \
&& ls mb-* | awk -F- '{print "ln -s " $1 "-" $2 " " $1 "-xilinx-elf-" $2 }' | /bin/sh
)
(cd tools/gcc3/microblaze \
&& ln -s ../include .
)

# Preparing new PATH

CURRDIR=`pwd`

NEWPATH=`(
echo "${PATH}" | tr ':' '\n' | grep -v "${CURRDIR}"
echo "${CURRDIR}/tools/gcc3/bin"
echo -n "${CURRDIR}/tools/gcc4/bin"
) | tr '\n' ':'`

echo "PATH=${NEWPATH}" > prepare.sh

echo "Run \". prepare.sh\" from this directory before cross-compiling for MicroBlaze"
