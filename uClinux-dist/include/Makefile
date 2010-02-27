#
# build a smart "symlink" header tree so the compile lines are
# much shorter and the problems with include paths are much smaller
#
# David McCullough <davidm@snapgear.com>
#

#
# At least one architecture now uses a common include directory in the
# MMU and non-MMU kernel. Both m68k and m68knommu use the m68k includes.
# And i386 has now become x86 (from linux-2.6.29 onwards).
#
ALTARCH=$(subst i386,x86,$(subst nommu,,$(ARCH)))

#
# glibc contains lots of nasty headers in a bad location,  so we need
# to cheat a little here
#

LINKHDRS = \
	$(ROOTDIR)/lib/libxml2/include/libxml/*.h,libxml \
	$(ROOTDIR)/lib/libxml2/build/include/libxml/*.h,libxml \
	$(ROOTDIR)/lib/STLport/stlport/*,STLport \
	$(ROOTDIR)/lib/libgmp/build/install/include/*.h,. \
	$(ROOTDIR)/lib/pcre/builddir/pcre.h,. \
	$(ROOTDIR)/lib/pcre/pcreposix.h,. \
	$(ROOTDIR)/lib/bitvector/*.h,. \
	$(ROOTDIR)/lib/libjpeg/*.h,. \
	$(ROOTDIR)/lib/libpng/*.h,. \
	$(ROOTDIR)/lib/libm/*.h,. \
	$(ROOTDIR)/lib/libnet/*.h,. \
	$(ROOTDIR)/lib/libpam/*/include/security/*.h,security \
	$(ROOTDIR)/lib/libpcap/pcap*.h,. \
	$(ROOTDIR)/lib/libares/ares.h,. \
	$(ROOTDIR)/lib/libidn/lib/*.h,. \
	$(ROOTDIR)/lib/libcurl/include/curl/*.h,curl \
	$(ROOTDIR)/lib/libpcap/net/bpf.h,net \
	$(ROOTDIR)/lib/libssl/build/include/*,. \
	$(ROOTDIR)/lib/zlib/*.h,. \
	$(ROOTDIR)/lib/libbzip2/bzlib.h,. \
	$(ROOTDIR)/lib/libkrb5/build/include/com_err.h,. \
	$(ROOTDIR)/lib/libkrb5/build/include/krb5/*.h,. \
	$(ROOTDIR)/lib/libkrb5/build/include/gssapi/*.h,. \
	$(ROOTDIR)/lib/libcyrussasl/include/*.h,. \
	$(ROOTDIR)/lib/libcyrussasl/build/include/*.h,. \
	$(ROOTDIR)/prop/libsnapgear/include/*.h,. \
	$(ROOTDIR)/prop/libsnapgear++/include/*,. \
	$(ROOTDIR)/prop/acld/sg_acld.h,. \
	$(ROOTDIR)/lib/libcrypt_old/*.h,. \
	$(ROOTDIR)/lib/libldap/include/*,. \
	$(ROOTDIR)/lib/libldap/builddir/include/*,. \
	$(ROOTDIR)/lib/libupnp/upnp/inc/upnp,. \
	$(ROOTDIR)/lib/libatm/src/include/atm*.h,. \
	$(ROOTDIR)/lib/flex/build/FlexLexer.h,. \
	$(ROOTDIR)/lib/lzo/include/lzo,. \
	$(ROOTDIR)/lib/termcap/termcap.h,. \
	$(ROOTDIR)/lib/sysfsutils/include/*.h,sysfs \
	$(ROOTDIR)/$(LINUXDIR)/include/linux,. \
	$(ROOTDIR)/$(LINUXDIR)/arch/$(ARCH)/include/asm-*,. \
	$(ROOTDIR)/$(LINUXDIR)/include/asm-*,. \
	$(ROOTDIR)/$(LINUXDIR)/arch/$(ARCH)/include/asm,. \
	$(ROOTDIR)/$(LINUXDIR)/arch/$(ALTARCH)/include/asm,. \
	$(ROOTDIR)/$(LINUXDIR)/include/asm,. \
	$(ROOTDIR)/$(LINUXDIR)/include/mtd,. \
	$(ROOTDIR)/autoconf.h,vendor \
	$(ROOTDIR)/config,. \
	$(ROOTDIR)/lib/expat/lib/expat.h,. \
	$(ROOTDIR)/user/tinytcl/*.h,. \
	$(ROOTDIR)/prop/config/authd_status.h,. \
	$(ROOTDIR)/prop/config/config*.h,. \
	$(ROOTDIR)/prop/config/if_*.h,. \
	$(ROOTDIR)/prop/config/libconfig.h,. \
	$(ROOTDIR)/prop/config/meta*.h,. \
	$(ROOTDIR)/prop/config/saverestoreall.h,. \
	$(ROOTDIR)/prop/config/vpnclient_utils.h,. \
	$(ROOTDIR)/prop/config/staticbuf.h,. \
	$(ROOTDIR)/prop/setkey/key.h,key \
	$(ROOTDIR)/prop/gccmapping/ValidateUpdate.h,. \
	$(ROOTDIR)/modules/autoconf.h,modules \
	$(ROOTDIR)/modules/ocf/cryptodev.h,crypto \
	$(ROOTDIR)/prop/libsscep/*.h,. \
	$(ROOTDIR)/prop/statsd/libstatsd.h,. \
	$(ROOTDIR)/lib/libnl/include/netlink*,. \
	$(ROOTDIR)/lib/libnfnetlink/include/libnfnetlink,. \
	$(ROOTDIR)/lib/libnetfilter_conntrack/include/libnetfilter_conntrack,. \
	$(ROOTDIR)/lib/libnetfilter_log/include/libnetfilter_log,. \
	$(ROOTDIR)/lib/bfd/build/bfd.h,. \
	$(ROOTDIR)/user/gdb/include/ansidecl.h,. \
	$(ROOTDIR)/user/gdb/include/symcat.h,. \
	$(ROOTDIR)/lib/popt/popt.h,. \
	$(ROOTDIR)/lib/osip2/build/install/include/osip*2,. \
	$(ROOTDIR)/prop/libtrustedsource/librep/*.h,. \
	$(ROOTDIR)/prop/libtrustedsource/librep/common/query.h,common \
	$(ROOTDIR)/prop/initdhcpd/initdhcpd.h,. \
	$(ROOTDIR)/lib/libffi/install/include/libffi,.


.PHONY: all romfs clean


#
# link in the files,  avoid missing or CVS files,  existing files take
# precedance so we do not try to overwrite them
#

all: clean
	@echo "Making symlinks in include/"
	@for p in $(LINKHDRS); do \
		src=`expr "$$p" : '\(.*\),.*'` ; \
		dst=`expr "$$p" : '.*,\(.*\)'` ; \
		for i in $$src; do \
			[ ! -e "$$i" ] && continue; \
			[ "`basename $$i`" = CVS ] && continue; \
			[ -L "$$dst/`basename $$i`" ] && continue; \
			[ -d $$dst ] || mkdir $$dst; \
			ln -s $$i $$dst/.; \
		done ; \
	done
	@rm -rf include-linux
	@mkdir -p include-linux
	@ln -s $(ROOTDIR)/$(LINUXDIR)/include/linux include-linux/linux
ifndef CONFIG_DEFAULTS_LIBC_NONE
	@rm -f gcc
	@ln -s `$(CC) -print-file-name=include` gcc
	@echo "Making include/c++ symlink to compiler c++ includes"
	@rm -f c++
	@ln -s /usr/local/include/g++-v3 c++
	@for i in `$(CXX) -v -E -xc++ /dev/null 2>&1 | sed -e '/^Reading specs/,/^#include <\.\.\.>/d' -e '/End of search list/Q'`; do \
		if [ -f $$i/new ]; then rm -f c++; ln -s $$i c++; break; fi; \
	done
	@ln -s c++ `$(CC) --version | head -1 | cut -d " " -f3`
	@ln -s c++ g++-v3
	@rm -f bits ; \
	 GCCVER=`$(CC) --version | head -1 | cut -d " " -f3` ; \
	 GCCMACH=`$(CC) -dumpmachine` ; \
	 GCCMULTI=`$(CC) -print-multi-directory` ; \
	 GCCPATH=`$(CC) -print-libgcc-file-name | sed -e 's?/lib/gcc/.*$$??g'` ; \
	 GPLUSBITS=$$GCCPATH/$$GCCMACH/include/c++/$$GCCVER/$$GCCMACH/$$GCCMULTI/bits ; \
	 echo "Checking for modern c++ bits, $$GPLUSBITS" ; \
	 if [ -d $$GPLUSBITS ] ; then \
		ln -s $$GPLUSBITS bits ; \
	fi
endif

romfs:

clean:
	-find . -depth -type l -a ! -name Makefile | xargs rm > /dev/null 2>&1 || exit 0
	-find . -depth -type d | grep -v .svn | xargs rmdir > /dev/null 2>&1 || exit 0
