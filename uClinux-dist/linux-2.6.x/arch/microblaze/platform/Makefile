#
# Makefile for arch/microblaze/platform directory
#

obj-y	+= platform.o

# Recurse into a vendor directory for vendor-specific drivers,
# etc. if one exists.
PRODUCT_DIR = arch/microblaze/platform/$(CONFIG_VENDOR)/$(CONFIG_PRODUCT)
ifeq (exists, $(shell [ -d $(PRODUCT_DIR) ] && echo exists))
  obj-y	+= $(CONFIG_VENDOR)/$(CONFIG_PRODUCT)/
endif
