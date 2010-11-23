#
# Vendor/Product Selection
#

#
# Select the Vendor you wish to target
#
CONFIG_DEFAULTS_BIAMP=y

#
# Select the Product you wish to target
#
CONFIG_DEFAULTS_BIAMP_LABRINTH_AVB_SUPERIO=y

#
# Kernel/Library/Defaults Selection
#
# CONFIG_DEFAULTS_KERNEL_2_4 is not set
CONFIG_DEFAULTS_KERNEL_2_6=y
CONFIG_DEFAULTS_LIBC_UCLIBC=y

CONFIG_VENDOR=Biamp
CONFIG_PRODUCT=Labrinth-AVB-SuperIO
CONFIG_LINUXDIR=linux-2.6.x
CONFIG_LIBCDIR=uClibc
