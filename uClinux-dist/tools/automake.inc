# Makefile to auto build tarball's with patches etc
# see Documentation/automake.txt for more info on using this file
# Copyright (C) 2009 David McCullough <davidm@uclinux.org>

.EXPORT_ALL_VARIABLES:

#
# these config options apply to all things we build
#
CONFOPTS += \
    --prefix= \
    --sysconfdir=/etc/config

CONFOPTS += --with-vendor-name=$(CONFIG_VENDOR)
CONFOPTS += --with-vendor-name-short=$(CONFIG_VENDOR)
CONFOPTS += --with-os-name=uClinux-dist
CONFOPTS += --with-os-vendor=uClinux
CONFOPTS += --disable-static

#
# for uClibc without glibc compat
#
CONFOPTS += --enable-malloc0returnsnull

#
# make sure LD does the right thing during configure if invoked directly
#
LDFLAGS  += -L$(ROOTDIR)/lib -Wl,-rpath-link -Wl,$(ROOTDIR)/lib

#
# where our pkgconfig stuff will go
#
PKG_CONFIG_LIBDIR=$(ROOTDIR)/lib/pkgconfig

#
# its generally a bad idea to fail on linker warnings for packages we
# aren't fully controlling,  though we could put this back conditionally
#
LDFLAGS := $(LDFLAGS:-Wl,--fatal-warnings=)

BUILD_CC = $(HOSTCC)
HOST_CC      = gcc
HOST_CXX     = g++
HOST_RANLIB  = $(shell which ranlib || echo :)

#
# we use wget to download
#
WGET ?= wget

#
# force using our special libtool
#
MAKEVARS     = LIBTOOL="$(LIBTOOL)"

#
# remap directories in a build to ensure that only uClinux-dist files
# are used
#
UCFRONT_MAPDIRS  = /lib/=$(ROOTDIR)/lib/
UCFRONT_MAPDIRS += /usr/lib=$(ROOTDIR)/lib
UCFRONT_MAPDIRS += /include/=$(ROOTDIR)/include/
UCFRONT_MAPDIRS += /usr/include=$(ROOTDIR)/include

#
# some short cut config,  just set URL = blah
# and we work the rest out
#
ifdef URL
  PKG := $(shell basename $(URL))
  PKG := $(PKG:.tar.gz=)
  PKG := $(PKG:.tar.bz2=)
  PKG := $(PKG:.tgz=)
  PKG := $(PKG:.tbz2=)
  $(PKG)_URL ?= $(URL)
  $(PKG)_INSTALL ?= $(INSTALL)
  $(PKG)_NODEFCONF ?= $(NODEFCONF)
  $(PKG)_CONFIGURE ?= $(CONFIGURE)
  $(PKG)_BUILDDIR ?= $(BUILDDIR)
  AUTOMAKE_y += $(PKG)
endif

#
# the main target
#
all: $(foreach pkg,$(AUTOMAKE_y),build/$(pkg)-lndir)

.PHONY: force_target
force_target:

.PRECIOUS: $(addprefix build/,$(addsuffix -extracted,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix build/,$(addsuffix -configured,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix build/,$(addsuffix -built,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix build/,$(addsuffix -installed,$(AUTOMAKE_y)))
.PRECIOUS: $(addprefix build/,$(addsuffix -patched,$(AUTOMAKE_y)))

downloads/%.tar.bz2 downloads/%.tar.gz:
	echo "Downloading $* ..."
	[ -d "$$HOME/.downloads" ] || mkdir ~/.downloads
	if [ ! -f "$$HOME/.downloads/`basename $@`" ]; then \
		(cd ~/.downloads; $(WGET) "$($*_URL)"); \
	fi
	mkdir -p `dirname $@`
	if [ -f "$$HOME/.downloads/`basename $@`" ]; then \
			ln -fs "$$HOME/.downloads/`basename $@`" $@; \
	fi
	if [ ! -f "$@" ]; then \
		echo "Cannot find download for $@" >&2 ; \
		exit 1; \
	fi

define download_dep
.PRECIOUS: downloads/$(shell basename "$($(1)_URL)")
build/$(1)-extracted: downloads/$(shell basename "$($(1)_URL)")
	mkdir -p build
	case "$$^" in \
	*bz2) bunzip2 < $$^ | (cd build; tar xf -);; \
	*) gunzip < $$^ | (cd build; tar xf -);; \
	esac
	touch $$@
endef

$(foreach pkg,$(AUTOMAKE_y),$(eval $(call download_dep,$(pkg))))

build/%-patched: build/%-extracted
	@if [ -f patches/series ]; then \
		while read t; do \
			echo "patching with patches/$$t"; \
			[ ! -f patches/$$t ] || (cd build/$(if $($*_BUILDDIR),$($*_BUILDDIR),$*); patch -E -p1) < patches/$$t; \
		done < patches/series; \
	else \
		for i in patches/$**.patch; do \
			echo "patching with $$i"; \
			[ ! -f $$i ] || (cd build/$(if $($*_BUILDDIR),$($*_BUILDDIR),$*); patch -E -p1) < $$i; \
		done; \
	fi
	touch $@

build/%-configured: build/%-patched
	cd build/$(if $($*_BUILDDIR),$($*_BUILDDIR),$*); \
		if [ -f $(if $($*_CONFIGURE),$($*_CONFIGURE),configure) ]; then \
			$(CONFVARS) \
			$($*_CONFVARS) \
			CFLAGS="$(CFLAGS) $($*_CFLAGS)" \
			sh $(if $($*_CONFIGURE),$($*_CONFIGURE),configure) $(if $($*_NODEFCONF),,$(CONFIGURE_OPTS) $(CONFOPTS)) $($*_CONFOPTS); \
		fi
	touch $@

build/%-built: build/%-configured
	$(BUILDVARS) \
	$($*_BUILDVARS) \
		$(MAKE) -C build/$(if $($*_BUILDDIR),$($*_BUILDDIR),$*) $(MAKEVARS) $($*_MAKEVARS)
	touch $@

build/%-installed: build/%-built
	mkdir -p build/$*-install
	$(BUILDVARS) \
	$($*_BUILDVARS) \
		$(MAKE) -C build/$(if $($*_BUILDDIR),$($*_BUILDDIR),$*) $(if $($*_INSTALL),$($*_INSTALL),install) DESTDIR=`pwd`/build/$*-install $(MAKEVARS) $($*_MAKEVARS)
	touch $@

build/%-lndir: build/%-installed force_target
	@cd build/$*-install; find . \( -type f -o -type l \) | egrep -v '.la$$' | \
	while read t; do \
		case "$$t" in \
		./bin/*) continue;; \
		./sbin/*) continue;; \
		esac; \
		rm -f $(ROOTDIR)/$$t; \
		mkdir -p "`dirname \"$(ROOTDIR)/$$t\"`"; \
		ln -s "`pwd`/$$t" "$(ROOTDIR)/$$t"; \
		echo "ln -s \"`pwd`/$$t\" \"$(ROOTDIR)/$$t\""; \
	done
	touch $@

clean:
	rm -rf build downloads

romfs:
	@for i in build/*-install; do \
		echo "Installing `basename $$i -install` ..."; \
		[ ! -d $$i/bin ] || $(ROMFSINST) -v -f $$i/bin /bin; \
		[ ! -d $$i/sbin ] || $(ROMFSINST) -v -f $$i/sbin /sbin; \
		[ ! -d $$i/lib ] || $(ROMFSINST) -v -f $$i/lib /lib; \
		rm -rf $$i/etc/default; \
		[ ! -d $$i/etc/config ] || mv $$i/etc/config $$i/etc/default; \
		[ ! -d $$i/etc ] || $(ROMFSINST) -v -f $$i/etc /etc; \
		[ ! -d $$i/share ] || $(ROMFSINST) -v -f $$i/share /share; \
	done
	rm -f $(ROMFSDIR)/lib/*.a
	rm -f $(ROMFSDIR)/lib/*.la

