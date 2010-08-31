# $Id$
#
# This file is part of avahi.
# 
# avahi is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version.
#
# avahi is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
# License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with avahi; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
# USA.


LD=$(CC)

AUTOMAKE = true

pkgdatadir = $(datadir)/avahi
pkglibdir = $(libdir)/avahi
pkgincludedir = $(includedir)/avahi
install_sh_DATA = $(install_sh) -c -m 644
install_sh_PROGRAM = $(install_sh) -c
install_sh_SCRIPT = $(install_sh) -c
INSTALL_HEADER = $(INSTALL_DATA)
NORMAL_INSTALL = :
PRE_INSTALL = :
POST_INSTALL = :
NORMAL_UNINSTALL = :
PRE_UNINSTALL = :
POST_UNINSTALL = :
#noinst_PROGRAMS = prioq-test \
#	avahi-test \
#	conformance-test \
#	avahi-reflector dns-test \
#	dns-spin-test \
#	timeeventq-test \
#	hashmap-test querier-test \
#	update-test
#TESTS = dns-spin-test dns-test \
#	hashmap-test
am__append_1 = \
       iface-linux.c iface-linux.h \
       netlink.c netlink.h

#am__append_2 = \
#	iface-pfroute.c iface-pfroute.h

##am__append_3 = \
##	iface-none.c

subdir = avahi-core
DIST_COMMON = $(avahiinclude_HEADERS) $(srcdir)/Makefile.am \
	$(srcdir)/Makefile.in
CONFIG_HEADER = $(top_builddir)/config.h
CONFIG_CLEAN_FILES =
am__vpath_adj_setup = srcdirstrip=`echo "$(srcdir)" | sed 's|.|.|g'`;
am__vpath_adj = case $$p in \
    $(srcdir)/*) f=`echo "$$p" | sed "s|^$$srcdirstrip/||"`;; \
    *) f=$$p;; \
  esac;
am__strip_dir = `echo $$p | sed -e 's|^.*/||'`;
am__installdirs = "$(DESTDIR)$(libdir)" "$(DESTDIR)$(avahiincludedir)"
libLTLIBRARIES_INSTALL = $(INSTALL)
LTLIBRARIES = $(lib_LTLIBRARIES)
libavahi_core_a_DEPENDENCIES = ../avahi-common/libavahi-common.a
am__libavahi_core_a_SOURCES_DIST = timeeventq.c timeeventq.h iface.c \
	iface.h server.c internal.h entry.c prioq.c prioq.h cache.c \
	cache.h socket.c socket.h response-sched.c response-sched.h \
	query-sched.c query-sched.h probe-sched.c probe-sched.h \
	announce.c announce.h browse.c browse.h rrlist.c rrlist.h \
	resolve-host-name.c resolve-address.c browse-domain.c \
	browse-service-type.c browse-service.c resolve-service.c dns.c \
	dns.h rr.c rr.h rr-util.h core.h lookup.h publish.h log.c \
	log.h browse-dns-server.c fdutil.h fdutil.c util.c util.h \
	hashmap.c hashmap.h wide-area.c wide-area.h multicast-lookup.c \
	multicast-lookup.h querier.c querier.h addr-util.h addr-util.c \
	domain-util.h domain-util.c dns-srv-rr.h iface-linux.c \
	iface-linux.h netlink.c netlink.h iface-pfroute.c \
	iface-pfroute.h iface-none.c
am__objects_1 = libavahi_core_a-iface-linux.o \
	libavahi_core_a-netlink.o
#am__objects_2 = libavahi_core_a-iface-pfroute.o
##am__objects_3 = libavahi_core_a-iface-none.o
am_libavahi_core_a_OBJECTS = libavahi_core_a-timeeventq.o \
	libavahi_core_a-iface.o libavahi_core_a-server.o \
	libavahi_core_a-entry.o libavahi_core_a-prioq.o \
	libavahi_core_a-cache.o libavahi_core_a-socket.o \
	libavahi_core_a-response-sched.o \
	libavahi_core_a-query-sched.o \
	libavahi_core_a-probe-sched.o libavahi_core_a-announce.o \
	libavahi_core_a-browse.o libavahi_core_a-rrlist.o \
	libavahi_core_a-resolve-host-name.o \
	libavahi_core_a-resolve-address.o \
	libavahi_core_a-browse-domain.o \
	libavahi_core_a-browse-service-type.o \
	libavahi_core_a-browse-service.o \
	libavahi_core_a-resolve-service.o libavahi_core_a-dns.o \
	libavahi_core_a-rr.o libavahi_core_a-log.o \
	libavahi_core_a-browse-dns-server.o \
	libavahi_core_a-fdutil.o libavahi_core_a-util.o \
	libavahi_core_a-hashmap.o libavahi_core_a-wide-area.o \
	libavahi_core_a-multicast-lookup.o \
	libavahi_core_a-querier.o libavahi_core_a-addr-util.o \
	libavahi_core_a-domain-util.o $(am__objects_1) \
	$(am__objects_2) $(am__objects_3)
libavahi_core_a_OBJECTS = $(am_libavahi_core_a_OBJECTS)
PROGRAMS = $(noinst_PROGRAMS)
am_avahi_reflector_OBJECTS =  \
	avahi_reflector-avahi-reflector.o
avahi_reflector_OBJECTS = $(am_avahi_reflector_OBJECTS)
avahi_reflector_DEPENDENCIES = ../avahi-common/libavahi-common.a \
	libavahi-core.a
avahi_reflector_LINK = $(LD) $(avahi_reflector_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_avahi_test_OBJECTS = avahi_test-avahi-test.o
avahi_test_OBJECTS = $(am_avahi_test_OBJECTS)
avahi_test_DEPENDENCIES = ../avahi-common/libavahi-common.a \
	libavahi-core.a
avahi_test_LINK = $(LD) $(avahi_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_conformance_test_OBJECTS =  \
	conformance_test-conformance-test.o
conformance_test_OBJECTS = $(am_conformance_test_OBJECTS)
conformance_test_DEPENDENCIES = ../avahi-common/libavahi-common.a \
	libavahi-core.a
conformance_test_LINK = $(LD) $(conformance_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_dns_spin_test_OBJECTS = dns_spin_test-dns-spin-test.o
dns_spin_test_OBJECTS = $(am_dns_spin_test_OBJECTS)
dns_spin_test_DEPENDENCIES = libavahi-core.a
dns_spin_test_LINK = $(LD) $(dns_spin_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_dns_test_OBJECTS = dns_test-dns.o \
	dns_test-dns-test.o dns_test-log.o \
	dns_test-util.o dns_test-rr.o \
	dns_test-hashmap.o dns_test-domain-util.o
dns_test_OBJECTS = $(am_dns_test_OBJECTS)
dns_test_DEPENDENCIES = ../avahi-common/libavahi-common.a
dns_test_LINK = $(LD) $(dns_test_CFLAGS) $(CFLAGS) $(AM_LDFLAGS) \
	$(LDFLAGS) -o $@
am_hashmap_test_OBJECTS = hashmap_test-hashmap-test.o \
	hashmap_test-hashmap.o hashmap_test-util.o
hashmap_test_OBJECTS = $(am_hashmap_test_OBJECTS)
hashmap_test_DEPENDENCIES = ../avahi-common/libavahi-common.a
hashmap_test_LINK = $(LD) $(hashmap_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_prioq_test_OBJECTS = prioq_test-prioq-test.o \
	prioq_test-prioq.o
prioq_test_OBJECTS = $(am_prioq_test_OBJECTS)
prioq_test_DEPENDENCIES = ../avahi-common/libavahi-common.a
prioq_test_LINK = $(LD) $(prioq_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_querier_test_OBJECTS = querier_test-querier-test.o
querier_test_OBJECTS = $(am_querier_test_OBJECTS)
querier_test_DEPENDENCIES = ../avahi-common/libavahi-common.a \
	libavahi-core.a
querier_test_LINK = $(LD) $(querier_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_timeeventq_test_OBJECTS =  \
	timeeventq_test-timeeventq-test.o \
	timeeventq_test-timeeventq.o \
	timeeventq_test-prioq.o timeeventq_test-log.o
timeeventq_test_OBJECTS = $(am_timeeventq_test_OBJECTS)
timeeventq_test_DEPENDENCIES = ../avahi-common/libavahi-common.a
timeeventq_test_LINK = $(LD) $(timeeventq_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_update_test_OBJECTS = update_test-update-test.o
update_test_OBJECTS = $(am_update_test_OBJECTS)
update_test_DEPENDENCIES = ../avahi-common/libavahi-common.a \
	libavahi-core.a
update_test_LINK = $(LD) $(update_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
DEFAULT_INCLUDES = -I. -I$(top_builddir)
depcomp = $(SHELL) $(top_srcdir)/depcomp
am__depfiles_maybe = depfiles
COMPILE = $(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) \
	$(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
LTCOMPILE = $(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) \
	$(AM_CPPFLAGS) $(CPPFLAGS) $(AM_CFLAGS) $(CFLAGS)
CCLD = $(CC)
LINK = $(LD) $(AM_CFLAGS) $(CFLAGS) $(AM_LDFLAGS) \
	$(LDFLAGS) -o $@
SOURCES = $(libavahi_core_a_SOURCES) $(avahi_reflector_SOURCES) \
	$(avahi_test_SOURCES) $(conformance_test_SOURCES) \
	$(dns_spin_test_SOURCES) $(dns_test_SOURCES) \
	$(hashmap_test_SOURCES) $(prioq_test_SOURCES) \
	$(querier_test_SOURCES) $(timeeventq_test_SOURCES) \
	$(update_test_SOURCES)
DIST_SOURCES = $(am__libavahi_core_a_SOURCES_DIST) \
	$(avahi_reflector_SOURCES) $(avahi_test_SOURCES) \
	$(conformance_test_SOURCES) $(dns_spin_test_SOURCES) \
	$(dns_test_SOURCES) $(hashmap_test_SOURCES) \
	$(prioq_test_SOURCES) $(querier_test_SOURCES) \
	$(timeeventq_test_SOURCES) $(update_test_SOURCES)
avahiincludeHEADERS_INSTALL = $(INSTALL_HEADER)
HEADERS = $(avahiinclude_HEADERS)
ETAGS = etags
CTAGS = ctags
DISTFILES = $(DIST_COMMON) $(DIST_SOURCES) $(TEXINFOS) $(EXTRA_DIST)
ALL_LINGUAS = 
AVAHI_AUTOIPD_GROUP = avahi-autoipd
AVAHI_AUTOIPD_USER = avahi-autoipd
AVAHI_GROUP = avahi
AVAHI_PRIV_ACCESS_GROUP = netdev
AVAHI_USER = avahi
AWK = gawk
CATALOGS = 
CATOBJEXT = .gmo
CYGPATH_W = echo
DATADIRNAME = share
DBUS_CFLAGS = 
DBUS_LIBS = 
DBUS_SYSTEM_BUS_DEFAULT_ADDRESS = Disabled
DBUS_SYS_DIR = Disabled
DEFS = -DHAVE_CONFIG_H
DEPDIR = .deps
DOXYGEN_PAPER_SIZE = 
DSYMUTIL = 
DUMPBIN = 
DX_CONFIG = doxygen.cfg
DX_DOCDIR = doxygen
DX_DOT = 
DX_DOXYGEN = 
DX_DVIPS = 
DX_EGREP = 
DX_ENV =  SRCDIR='.' PROJECT='avahi' DOCDIR='doxygen' VERSION='0.6.25' HAVE_DOT='NO' GENERATE_MAN='NO' GENERATE_RTF='NO' GENERATE_XML='NO' GENERATE_HTMLHELP='NO' GENERATE_CHI='NO' GENERATE_HTML='NO' GENERATE_LATEX='NO'
DX_FLAG_chi = 0
DX_FLAG_chm = 0
DX_FLAG_doc = 0
DX_FLAG_dot = 0
DX_FLAG_html = 0
DX_FLAG_man = 0
DX_FLAG_pdf = 0
DX_FLAG_ps = 0
DX_FLAG_rtf = 0
DX_FLAG_xml = 0
DX_HHC = 
DX_LATEX = 
DX_MAKEINDEX = 
DX_PDFLATEX = 
DX_PERL = 
DX_PROJECT = avahi
ECHO_C = 
ECHO_N = -n
ECHO_T = 
EGREP = /bin/grep -E
FGREP = /bin/grep -F
GACUTIL = 
GETTEXT_PACKAGE = avahi
GLADE20_CFLAGS = 
GLADE20_LIBS = 
GLIB20_CFLAGS = 
GLIB20_LIBS = 
GMOFILES = 
GMSGFMT = /usr/bin/msgfmt
GOBJECT_CFLAGS = 
GOBJECT_LIBS = 
GREP = /bin/grep
GTK20_CFLAGS = 
GTK20_LIBS = 
HOWL_COMPAT_VERSION = 0.9.8
INSTALL = /usr/bin/install -c
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_PROGRAM = ${INSTALL}
INSTALL_SCRIPT = ${INSTALL}
INSTALL_STRIP_PROGRAM = $(install_sh) -c -s
INSTOBJEXT = .mo
INTLLIBS = 
INTLTOOL_EXTRACT = /usr/bin/intltool-extract
INTLTOOL_MERGE = /usr/bin/intltool-merge
INTLTOOL_PERL = /usr/bin/perl
INTLTOOL_UPDATE = /usr/bin/intltool-update
LIBAVAHI_CLIENT_VERSION_INFO = 5:5:2
LIBAVAHI_COMMON_VERSION_INFO = 8:1:5
LIBAVAHI_COMPAT_HOWL_VERSION_INFO = 0:0:0
LIBAVAHI_COMPAT_LIBDNS_SD_VERSION_INFO = 1:0:0
LIBAVAHI_CORE_VERSION_INFO = 6:1:0
LIBAVAHI_GLIB_VERSION_INFO = 1:1:0
LIBAVAHI_GOBJECT_VERSION_INFO = 0:1:0
LIBAVAHI_QT3_VERSION_INFO = 1:2:0
LIBAVAHI_QT4_VERSION_INFO = 1:2:0
LIBAVAHI_UI_VERSION_INFO = 1:1:1
LIBDAEMON_CFLAGS = -I../../include/
LIBDAEMON_LIBS = ../../lib/libdaemon.a
LIBOBJS = 
LIBS = 
LIPO = 
LN_S = ln -s
LTLIBOBJS = 
MAKEINFO = ${SHELL} $(ROOTDIR)/user/avahi/missing --run makeinfo
MCS = 
MDASSEMBLER = 
MKDIR_P = /bin/mkdir -p
MKINSTALLDIRS = ./mkinstalldirs
MOC_QT3 = 
MOC_QT4 = 
MONODOCER = 
MONODOC_CFLAGS = 
MONODOC_DIR = 
MONODOC_LIBS = 
MSGFMT = /usr/bin/msgfmt
MSGFMT_OPTS = -c
MSGMERGE = /usr/bin/msgmerge
NM = /usr/bin/nm -B
NMEDIT = 
OBJDUMP = objdump
OBJEXT = o
OTOOL = 
OTOOL64 = 
PACKAGE = avahi
PACKAGE_BUGREPORT = avahi (at) lists (dot) freedesktop (dot) org
PACKAGE_NAME = avahi
PACKAGE_STRING = avahi 0.6.25
PACKAGE_TARNAME = avahi
PACKAGE_URL = http://avahi.org/
PACKAGE_VERSION = 0.6.25
PATH_SEPARATOR = :
PKG_CONFIG = /usr/bin/pkg-config
POFILES = 
POSUB = po
PO_IN_DATADIR_FALSE = 
PO_IN_DATADIR_TRUE = 
PTHREAD_CC = $(CC)
PTHREAD_CFLAGS = 
PTHREAD_LIBS = -lpthread
PYTHON = 
PYTHON_EXEC_PREFIX = 
PYTHON_PLATFORM = 
PYTHON_PREFIX = 
PYTHON_VERSION = 
QT3_CFLAGS = 
QT3_LIBS = 
QT4_CFLAGS = 
QT4_LIBS = 
RANLIB = ranlib
SED = /bin/sed
SET_MAKE = 
SHELL = /bin/bash
STRIP = strip
USE_NLS = yes
VERSION = 0.6.25
VISIBILITY_HIDDEN_CFLAGS = -fvisibility=hidden -DHAVE_VISIBILITY_HIDDEN
XGETTEXT = /usr/bin/xgettext
XML_CFLAGS = -DUSE_EXPAT_H
XML_LIBS = -lexpat
abs_builddir = $(ROOTDIR)/user/avahi/avahi-core
abs_srcdir = $(ROOTDIR)/user/avahi/avahi-core
abs_top_builddir = $(ROOTDIR)/user/avahi
abs_top_srcdir = $(ROOTDIR)/user/avahi
ac_ct_CC = $(CC)
ac_ct_CXX = $(CXX)
ac_ct_DUMPBIN = 
acx_pthread_config = 
am__include = -include
am__leading_dot = .
am__quote = 
am__tar = ${AMTAR} chof - "$$tardir"
am__untar = ${AMTAR} xf -
avahi_runtime_dir = /var/run
avahi_socket = /var/run/avahi-daemon/socket
avahilocaledir = ${prefix}/${DATADIRNAME}/locale
bindir = /bin
build = i686-pc-linux-gnu
build_alias = 
build_cpu = i686
build_os = linux-gnu
build_vendor = pc
builddir = .
datadir = ${datarootdir}
datarootdir = ${prefix}/share
docdir = ${datarootdir}/doc/${PACKAGE_TARNAME}
dvidir = ${docdir}
exec_prefix = ${prefix}
have_pkg_config = yes
have_xmltoman = no
host = i686-pc-linux-gnu
host_alias = 
host_cpu = i686
host_os = linux-gnu
host_vendor = pc
htmldir = ${docdir}
includedir = ${prefix}/include
infodir = ${datarootdir}/info
install_sh = $(SHELL) $(ROOTDIR)/user/avahi/install-sh
interfacesdir = 
libdir = ${exec_prefix}/lib
libexecdir = ${exec_prefix}/libexec
localedir = ${datarootdir}/locale
localstatedir = /var
lt_ECHO = echo
mandir = ${datarootdir}/man
mkdir_p = /bin/mkdir -p
oldincludedir = /usr/include
pdfdir = ${docdir}
pkgpyexecdir = 
pkgpythondir = 
prefix = /usr
psdir = ${docdir}
pyexecdir = 
pythondir = 
sbindir = /bin
sharedstatedir = ${prefix}/com
srcdir = .
sysconfdir = /etc
target_alias = 
top_build_prefix = ../
top_builddir = ..
top_srcdir = ..

# This cool debug trap works on i386/gcc only
AM_CFLAGS = -I$(top_srcdir) '-DDEBUG_TRAP=__asm__("int $$3")'
avahiincludedir = $(includedir)/avahi-core
avahiinclude_HEADERS = \
	core.h \
	log.h \
	rr.h \
	publish.h \
	lookup.h

lib_LTLIBRARIES = \
	libavahi-core.a

libavahi_core_a_SOURCES = timeeventq.c timeeventq.h iface.c iface.h \
	server.c internal.h entry.c prioq.c prioq.h cache.c cache.h \
	socket.c socket.h response-sched.c response-sched.h \
	query-sched.c query-sched.h probe-sched.c probe-sched.h \
	announce.c announce.h browse.c browse.h rrlist.c rrlist.h \
	resolve-host-name.c resolve-address.c browse-domain.c \
	browse-service-type.c browse-service.c resolve-service.c dns.c \
	dns.h rr.c rr.h rr-util.h core.h lookup.h publish.h log.c \
	log.h browse-dns-server.c fdutil.h fdutil.c util.c util.h \
	hashmap.c hashmap.h wide-area.c wide-area.h multicast-lookup.c \
	multicast-lookup.h querier.c querier.h addr-util.h addr-util.c \
	domain-util.h domain-util.c dns-srv-rr.h $(am__append_1) \
	$(am__append_2) $(am__append_3)
libavahi_core_a_CFLAGS = $(AM_CFLAGS)
libavahi_core_a_LIBADD = $(AM_LDADD) ../avahi-common/libavahi-common.a
libavahi_core_a_LDFLAGS = $(AM_LDFLAGS)  -version-info $(LIBAVAHI_CORE_VERSION_INFO)
prioq_test_SOURCES = \
	prioq-test.c  \
	prioq.c prioq.h

prioq_test_CFLAGS = $(AM_CFLAGS)
prioq_test_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a
avahi_test_SOURCES = \
	avahi-test.c

avahi_test_CFLAGS = $(AM_CFLAGS)
avahi_test_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a libavahi-core.a
update_test_SOURCES = \
	update-test.c

update_test_CFLAGS = $(AM_CFLAGS)
update_test_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a libavahi-core.a
querier_test_SOURCES = \
	querier-test.c

querier_test_CFLAGS = $(AM_CFLAGS)
querier_test_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a libavahi-core.a
conformance_test_SOURCES = \
	conformance-test.c

conformance_test_CFLAGS = $(AM_CFLAGS)
conformance_test_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a libavahi-core.a
avahi_reflector_SOURCES = \
	avahi-reflector.c

avahi_reflector_CFLAGS = $(AM_CFLAGS)
avahi_reflector_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a libavahi-core.a
dns_test_SOURCES = \
	dns.c dns.h \
	dns-test.c \
	log.c log.h \
	util.c util.h \
	rr.c rr.h \
	hashmap.c hashmap.h \
	domain-util.c domain-util.h

dns_test_CFLAGS = $(AM_CFLAGS)
dns_test_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a
dns_spin_test_SOURCES = \
	dns-spin-test.c

dns_spin_test_CFLAGS = $(AM_CFLAGS)
dns_spin_test_LDADD = $(AM_LDADD) libavahi-core.a
timeeventq_test_SOURCES = \
	timeeventq-test.c \
	timeeventq.h timeeventq.c \
	prioq.h prioq.c \
	log.c log.h

timeeventq_test_CFLAGS = $(AM_CFLAGS)
timeeventq_test_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a
hashmap_test_SOURCES = \
	hashmap-test.c \
	hashmap.h hashmap.c \
	util.h util.c

hashmap_test_CFLAGS = $(AM_CFLAGS)
hashmap_test_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a
all: all-am

.SUFFIXES:
.SUFFIXES: .c .o
$(srcdir)/Makefile.in:  $(srcdir)/Makefile.am  $(am__configure_deps)
	@for dep in $?; do \
	  case '$(am__configure_deps)' in \
	    *$$dep*) \
	      ( cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh ) \
	        && { if test -f $@; then exit 0; else break; fi; }; \
	      exit 1;; \
	  esac; \
	done; \
	echo ' cd $(top_srcdir) && $(AUTOMAKE) --foreign  avahi-core/Makefile'; \
	cd $(top_srcdir) && \
	  $(AUTOMAKE) --foreign  avahi-core/Makefile
.PRECIOUS: Makefile
Makefile: $(srcdir)/Makefile.in $(top_builddir)/config.status
	@case '$?' in \
	  *config.status*) \
	    cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh;; \
	  *) \
	    echo ' cd $(top_builddir) && $(SHELL) ./config.status $(subdir)/$@ $(am__depfiles_maybe)'; \
	    cd $(top_builddir) && $(SHELL) ./config.status $(subdir)/$@ $(am__depfiles_maybe);; \
	esac;

$(top_builddir)/config.status: $(top_srcdir)/configure $(CONFIG_STATUS_DEPENDENCIES)
	cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh

$(top_srcdir)/configure:  $(am__configure_deps)
	cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh
$(ACLOCAL_M4):  $(am__aclocal_m4_deps)
	cd $(top_builddir) && $(MAKE) $(AM_MAKEFLAGS) am--refresh
install-libLTLIBRARIES: $(lib_LTLIBRARIES)
	@$(NORMAL_INSTALL)
	test -z "$(libdir)" || $(MKDIR_P) "$(DESTDIR)$(libdir)"
	@list='$(lib_LTLIBRARIES)'; for p in $$list; do \
	  if test -f $$p; then \
	    f=$(am__strip_dir) \
	    echo " $(libLTLIBRARIES_INSTALL) $(INSTALL_STRIP_FLAG) '$$p' '$(DESTDIR)$(libdir)/$$f'"; \
	    $(libLTLIBRARIES_INSTALL) $(INSTALL_STRIP_FLAG) "$$p" "$(DESTDIR)$(libdir)/$$f"; \
	  else :; fi; \
	done

uninstall-libLTLIBRARIES:
	@$(NORMAL_UNINSTALL)
	@list='$(lib_LTLIBRARIES)'; for p in $$list; do \
	  p=$(am__strip_dir) \
	  echo " rm -f '$(DESTDIR)$(libdir)/$$p'"; \
	  rm -f "$(DESTDIR)$(libdir)/$$p"; \
	done

clean-libLTLIBRARIES:
	-test -z "$(lib_LTLIBRARIES)" || rm -f $(lib_LTLIBRARIES)
	@list='$(lib_LTLIBRARIES)'; for p in $$list; do \
	  dir="`echo $$p | sed -e 's|/[^/]*$$||'`"; \
	  test "$$dir" != "$$p" || dir=.; \
	  echo "rm -f \"$${dir}/so_locations\""; \
	  rm -f "$${dir}/so_locations"; \
	done

libavahi-core.a: $(libavahi_core_a_OBJECTS) $(libavahi_core_a_DEPENDENCIES) 
	$(AR) -crs $@ $(libavahi_core_a_OBJECTS)

clean-noinstPROGRAMS:
	@list='$(noinst_PROGRAMS)'; for p in $$list; do \
	  echo " rm -f $$p"; \
	  rm -f $$p ; \
	done
avahi-reflector: $(avahi_reflector_OBJECTS) $(avahi_reflector_DEPENDENCIES) 
	@rm -f avahi-reflector
	$(avahi_reflector_LINK) $(avahi_reflector_OBJECTS) $(avahi_reflector_LDADD) $(LIBS)
avahi-test: $(avahi_test_OBJECTS) $(avahi_test_DEPENDENCIES) 
	@rm -f avahi-test
	$(avahi_test_LINK) $(avahi_test_OBJECTS) $(avahi_test_LDADD) $(LIBS)
conformance-test: $(conformance_test_OBJECTS) $(conformance_test_DEPENDENCIES) 
	@rm -f conformance-test
	$(conformance_test_LINK) $(conformance_test_OBJECTS) $(conformance_test_LDADD) $(LIBS)
dns-spin-test: $(dns_spin_test_OBJECTS) $(dns_spin_test_DEPENDENCIES) 
	@rm -f dns-spin-test
	$(dns_spin_test_LINK) $(dns_spin_test_OBJECTS) $(dns_spin_test_LDADD) $(LIBS)
dns-test: $(dns_test_OBJECTS) $(dns_test_DEPENDENCIES) 
	@rm -f dns-test
	$(dns_test_LINK) $(dns_test_OBJECTS) $(dns_test_LDADD) $(LIBS)
hashmap-test: $(hashmap_test_OBJECTS) $(hashmap_test_DEPENDENCIES) 
	@rm -f hashmap-test
	$(hashmap_test_LINK) $(hashmap_test_OBJECTS) $(hashmap_test_LDADD) $(LIBS)
prioq-test: $(prioq_test_OBJECTS) $(prioq_test_DEPENDENCIES) 
	@rm -f prioq-test
	$(prioq_test_LINK) $(prioq_test_OBJECTS) $(prioq_test_LDADD) $(LIBS)
querier-test: $(querier_test_OBJECTS) $(querier_test_DEPENDENCIES) 
	@rm -f querier-test
	$(querier_test_LINK) $(querier_test_OBJECTS) $(querier_test_LDADD) $(LIBS)
timeeventq-test: $(timeeventq_test_OBJECTS) $(timeeventq_test_DEPENDENCIES) 
	@rm -f timeeventq-test
	$(timeeventq_test_LINK) $(timeeventq_test_OBJECTS) $(timeeventq_test_LDADD) $(LIBS)
update-test: $(update_test_OBJECTS) $(update_test_DEPENDENCIES) 
	@rm -f update-test
	$(update_test_LINK) $(update_test_OBJECTS) $(update_test_LDADD) $(LIBS)

mostlyclean-compile:
	-rm -f *.o

distclean-compile:
	-rm -f *.tab.c

-include ./$(DEPDIR)/avahi_reflector-avahi-reflector.Po
-include ./$(DEPDIR)/avahi_test-avahi-test.Po
-include ./$(DEPDIR)/conformance_test-conformance-test.Po
-include ./$(DEPDIR)/dns_spin_test-dns-spin-test.Po
-include ./$(DEPDIR)/dns_test-dns-test.Po
-include ./$(DEPDIR)/dns_test-dns.Po
-include ./$(DEPDIR)/dns_test-domain-util.Po
-include ./$(DEPDIR)/dns_test-hashmap.Po
-include ./$(DEPDIR)/dns_test-log.Po
-include ./$(DEPDIR)/dns_test-rr.Po
-include ./$(DEPDIR)/dns_test-util.Po
-include ./$(DEPDIR)/hashmap_test-hashmap-test.Po
-include ./$(DEPDIR)/hashmap_test-hashmap.Po
-include ./$(DEPDIR)/hashmap_test-util.Po
-include ./$(DEPDIR)/libavahi_core_la-addr-util.Plo
-include ./$(DEPDIR)/libavahi_core_la-announce.Plo
-include ./$(DEPDIR)/libavahi_core_la-browse-dns-server.Plo
-include ./$(DEPDIR)/libavahi_core_la-browse-domain.Plo
-include ./$(DEPDIR)/libavahi_core_la-browse-service-type.Plo
-include ./$(DEPDIR)/libavahi_core_la-browse-service.Plo
-include ./$(DEPDIR)/libavahi_core_la-browse.Plo
-include ./$(DEPDIR)/libavahi_core_la-cache.Plo
-include ./$(DEPDIR)/libavahi_core_la-dns.Plo
-include ./$(DEPDIR)/libavahi_core_la-domain-util.Plo
-include ./$(DEPDIR)/libavahi_core_la-entry.Plo
-include ./$(DEPDIR)/libavahi_core_la-fdutil.Plo
-include ./$(DEPDIR)/libavahi_core_la-hashmap.Plo
-include ./$(DEPDIR)/libavahi_core_la-iface-linux.Plo
-include ./$(DEPDIR)/libavahi_core_la-iface-none.Plo
-include ./$(DEPDIR)/libavahi_core_la-iface-pfroute.Plo
-include ./$(DEPDIR)/libavahi_core_la-iface.Plo
-include ./$(DEPDIR)/libavahi_core_la-log.Plo
-include ./$(DEPDIR)/libavahi_core_la-multicast-lookup.Plo
-include ./$(DEPDIR)/libavahi_core_la-netlink.Plo
-include ./$(DEPDIR)/libavahi_core_la-prioq.Plo
-include ./$(DEPDIR)/libavahi_core_la-probe-sched.Plo
-include ./$(DEPDIR)/libavahi_core_la-querier.Plo
-include ./$(DEPDIR)/libavahi_core_la-query-sched.Plo
-include ./$(DEPDIR)/libavahi_core_la-resolve-address.Plo
-include ./$(DEPDIR)/libavahi_core_la-resolve-host-name.Plo
-include ./$(DEPDIR)/libavahi_core_la-resolve-service.Plo
-include ./$(DEPDIR)/libavahi_core_la-response-sched.Plo
-include ./$(DEPDIR)/libavahi_core_la-rr.Plo
-include ./$(DEPDIR)/libavahi_core_la-rrlist.Plo
-include ./$(DEPDIR)/libavahi_core_la-server.Plo
-include ./$(DEPDIR)/libavahi_core_la-socket.Plo
-include ./$(DEPDIR)/libavahi_core_la-timeeventq.Plo
-include ./$(DEPDIR)/libavahi_core_la-util.Plo
-include ./$(DEPDIR)/libavahi_core_la-wide-area.Plo
-include ./$(DEPDIR)/prioq_test-prioq-test.Po
-include ./$(DEPDIR)/prioq_test-prioq.Po
-include ./$(DEPDIR)/querier_test-querier-test.Po
-include ./$(DEPDIR)/timeeventq_test-log.Po
-include ./$(DEPDIR)/timeeventq_test-prioq.Po
-include ./$(DEPDIR)/timeeventq_test-timeeventq-test.Po
-include ./$(DEPDIR)/timeeventq_test-timeeventq.Po
-include ./$(DEPDIR)/update_test-update-test.Po

.c.o:
	$(COMPILE) -MT $@ -MD -MP -MF $(DEPDIR)/$*.Tpo -c -o $@ $<
	mv -f $(DEPDIR)/$*.Tpo $(DEPDIR)/$*.Po
#	source='$<' object='$@' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(COMPILE) -c $<

libavahi_core_a-timeeventq.o: timeeventq.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-timeeventq.Tpo -c -o libavahi_core_a-timeeventq.o `test -f 'timeeventq.c' || echo '$(srcdir)/'`timeeventq.c
	mv -f $(DEPDIR)/libavahi_core_la-timeeventq.Tpo $(DEPDIR)/libavahi_core_la-timeeventq.Plo
#	source='timeeventq.c' object='libavahi_core_a-timeeventq.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-timeeventq.o `test -f 'timeeventq.c' || echo '$(srcdir)/'`timeeventq.c

libavahi_core_a-iface.o: iface.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-iface.Tpo -c -o libavahi_core_a-iface.o `test -f 'iface.c' || echo '$(srcdir)/'`iface.c
	mv -f $(DEPDIR)/libavahi_core_la-iface.Tpo $(DEPDIR)/libavahi_core_la-iface.Plo
#	source='iface.c' object='libavahi_core_a-iface.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-iface.o `test -f 'iface.c' || echo '$(srcdir)/'`iface.c

libavahi_core_a-server.o: server.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-server.Tpo -c -o libavahi_core_a-server.o `test -f 'server.c' || echo '$(srcdir)/'`server.c
	mv -f $(DEPDIR)/libavahi_core_la-server.Tpo $(DEPDIR)/libavahi_core_la-server.Plo
#	source='server.c' object='libavahi_core_a-server.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-server.o `test -f 'server.c' || echo '$(srcdir)/'`server.c

libavahi_core_a-entry.o: entry.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-entry.Tpo -c -o libavahi_core_a-entry.o `test -f 'entry.c' || echo '$(srcdir)/'`entry.c
	mv -f $(DEPDIR)/libavahi_core_la-entry.Tpo $(DEPDIR)/libavahi_core_la-entry.Plo
#	source='entry.c' object='libavahi_core_a-entry.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-entry.o `test -f 'entry.c' || echo '$(srcdir)/'`entry.c

libavahi_core_a-prioq.o: prioq.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-prioq.Tpo -c -o libavahi_core_a-prioq.o `test -f 'prioq.c' || echo '$(srcdir)/'`prioq.c
	mv -f $(DEPDIR)/libavahi_core_la-prioq.Tpo $(DEPDIR)/libavahi_core_la-prioq.Plo
#	source='prioq.c' object='libavahi_core_a-prioq.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-prioq.o `test -f 'prioq.c' || echo '$(srcdir)/'`prioq.c

libavahi_core_a-cache.o: cache.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-cache.Tpo -c -o libavahi_core_a-cache.o `test -f 'cache.c' || echo '$(srcdir)/'`cache.c
	mv -f $(DEPDIR)/libavahi_core_la-cache.Tpo $(DEPDIR)/libavahi_core_la-cache.Plo
#	source='cache.c' object='libavahi_core_a-cache.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-cache.o `test -f 'cache.c' || echo '$(srcdir)/'`cache.c

libavahi_core_a-socket.o: socket.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-socket.Tpo -c -o libavahi_core_a-socket.o `test -f 'socket.c' || echo '$(srcdir)/'`socket.c
	mv -f $(DEPDIR)/libavahi_core_la-socket.Tpo $(DEPDIR)/libavahi_core_la-socket.Plo
#	source='socket.c' object='libavahi_core_a-socket.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-socket.o `test -f 'socket.c' || echo '$(srcdir)/'`socket.c

libavahi_core_a-response-sched.o: response-sched.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-response-sched.Tpo -c -o libavahi_core_a-response-sched.o `test -f 'response-sched.c' || echo '$(srcdir)/'`response-sched.c
	mv -f $(DEPDIR)/libavahi_core_la-response-sched.Tpo $(DEPDIR)/libavahi_core_la-response-sched.Plo
#	source='response-sched.c' object='libavahi_core_a-response-sched.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-response-sched.o `test -f 'response-sched.c' || echo '$(srcdir)/'`response-sched.c

libavahi_core_a-query-sched.o: query-sched.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-query-sched.Tpo -c -o libavahi_core_a-query-sched.o `test -f 'query-sched.c' || echo '$(srcdir)/'`query-sched.c
	mv -f $(DEPDIR)/libavahi_core_la-query-sched.Tpo $(DEPDIR)/libavahi_core_la-query-sched.Plo
#	source='query-sched.c' object='libavahi_core_a-query-sched.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-query-sched.o `test -f 'query-sched.c' || echo '$(srcdir)/'`query-sched.c

libavahi_core_a-probe-sched.o: probe-sched.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-probe-sched.Tpo -c -o libavahi_core_a-probe-sched.o `test -f 'probe-sched.c' || echo '$(srcdir)/'`probe-sched.c
	mv -f $(DEPDIR)/libavahi_core_la-probe-sched.Tpo $(DEPDIR)/libavahi_core_la-probe-sched.Plo
#	source='probe-sched.c' object='libavahi_core_a-probe-sched.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-probe-sched.o `test -f 'probe-sched.c' || echo '$(srcdir)/'`probe-sched.c

libavahi_core_a-announce.o: announce.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-announce.Tpo -c -o libavahi_core_a-announce.o `test -f 'announce.c' || echo '$(srcdir)/'`announce.c
	mv -f $(DEPDIR)/libavahi_core_la-announce.Tpo $(DEPDIR)/libavahi_core_la-announce.Plo
#	source='announce.c' object='libavahi_core_a-announce.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-announce.o `test -f 'announce.c' || echo '$(srcdir)/'`announce.c

libavahi_core_a-browse.o: browse.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-browse.Tpo -c -o libavahi_core_a-browse.o `test -f 'browse.c' || echo '$(srcdir)/'`browse.c
	mv -f $(DEPDIR)/libavahi_core_la-browse.Tpo $(DEPDIR)/libavahi_core_la-browse.Plo
#	source='browse.c' object='libavahi_core_a-browse.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-browse.o `test -f 'browse.c' || echo '$(srcdir)/'`browse.c

libavahi_core_a-rrlist.o: rrlist.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-rrlist.Tpo -c -o libavahi_core_a-rrlist.o `test -f 'rrlist.c' || echo '$(srcdir)/'`rrlist.c
	mv -f $(DEPDIR)/libavahi_core_la-rrlist.Tpo $(DEPDIR)/libavahi_core_la-rrlist.Plo
#	source='rrlist.c' object='libavahi_core_a-rrlist.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-rrlist.o `test -f 'rrlist.c' || echo '$(srcdir)/'`rrlist.c

libavahi_core_a-resolve-host-name.o: resolve-host-name.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-resolve-host-name.Tpo -c -o libavahi_core_a-resolve-host-name.o `test -f 'resolve-host-name.c' || echo '$(srcdir)/'`resolve-host-name.c
	mv -f $(DEPDIR)/libavahi_core_la-resolve-host-name.Tpo $(DEPDIR)/libavahi_core_la-resolve-host-name.Plo
#	source='resolve-host-name.c' object='libavahi_core_a-resolve-host-name.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-resolve-host-name.o `test -f 'resolve-host-name.c' || echo '$(srcdir)/'`resolve-host-name.c

libavahi_core_a-resolve-address.o: resolve-address.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-resolve-address.Tpo -c -o libavahi_core_a-resolve-address.o `test -f 'resolve-address.c' || echo '$(srcdir)/'`resolve-address.c
	mv -f $(DEPDIR)/libavahi_core_la-resolve-address.Tpo $(DEPDIR)/libavahi_core_la-resolve-address.Plo
#	source='resolve-address.c' object='libavahi_core_a-resolve-address.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-resolve-address.o `test -f 'resolve-address.c' || echo '$(srcdir)/'`resolve-address.c

libavahi_core_a-browse-domain.o: browse-domain.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-browse-domain.Tpo -c -o libavahi_core_a-browse-domain.o `test -f 'browse-domain.c' || echo '$(srcdir)/'`browse-domain.c
	mv -f $(DEPDIR)/libavahi_core_la-browse-domain.Tpo $(DEPDIR)/libavahi_core_la-browse-domain.Plo
#	source='browse-domain.c' object='libavahi_core_a-browse-domain.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-browse-domain.o `test -f 'browse-domain.c' || echo '$(srcdir)/'`browse-domain.c

libavahi_core_a-browse-service-type.o: browse-service-type.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-browse-service-type.Tpo -c -o libavahi_core_a-browse-service-type.o `test -f 'browse-service-type.c' || echo '$(srcdir)/'`browse-service-type.c
	mv -f $(DEPDIR)/libavahi_core_la-browse-service-type.Tpo $(DEPDIR)/libavahi_core_la-browse-service-type.Plo
#	source='browse-service-type.c' object='libavahi_core_a-browse-service-type.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-browse-service-type.o `test -f 'browse-service-type.c' || echo '$(srcdir)/'`browse-service-type.c

libavahi_core_a-browse-service.o: browse-service.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-browse-service.Tpo -c -o libavahi_core_a-browse-service.o `test -f 'browse-service.c' || echo '$(srcdir)/'`browse-service.c
	mv -f $(DEPDIR)/libavahi_core_la-browse-service.Tpo $(DEPDIR)/libavahi_core_la-browse-service.Plo
#	source='browse-service.c' object='libavahi_core_a-browse-service.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-browse-service.o `test -f 'browse-service.c' || echo '$(srcdir)/'`browse-service.c

libavahi_core_a-resolve-service.o: resolve-service.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-resolve-service.Tpo -c -o libavahi_core_a-resolve-service.o `test -f 'resolve-service.c' || echo '$(srcdir)/'`resolve-service.c
	mv -f $(DEPDIR)/libavahi_core_la-resolve-service.Tpo $(DEPDIR)/libavahi_core_la-resolve-service.Plo
#	source='resolve-service.c' object='libavahi_core_a-resolve-service.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-resolve-service.o `test -f 'resolve-service.c' || echo '$(srcdir)/'`resolve-service.c

libavahi_core_a-dns.o: dns.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-dns.Tpo -c -o libavahi_core_a-dns.o `test -f 'dns.c' || echo '$(srcdir)/'`dns.c
	mv -f $(DEPDIR)/libavahi_core_la-dns.Tpo $(DEPDIR)/libavahi_core_la-dns.Plo
#	source='dns.c' object='libavahi_core_a-dns.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-dns.o `test -f 'dns.c' || echo '$(srcdir)/'`dns.c

libavahi_core_a-rr.o: rr.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-rr.Tpo -c -o libavahi_core_a-rr.o `test -f 'rr.c' || echo '$(srcdir)/'`rr.c
	mv -f $(DEPDIR)/libavahi_core_la-rr.Tpo $(DEPDIR)/libavahi_core_la-rr.Plo
#	source='rr.c' object='libavahi_core_a-rr.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-rr.o `test -f 'rr.c' || echo '$(srcdir)/'`rr.c

libavahi_core_a-log.o: log.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-log.Tpo -c -o libavahi_core_a-log.o `test -f 'log.c' || echo '$(srcdir)/'`log.c
	mv -f $(DEPDIR)/libavahi_core_la-log.Tpo $(DEPDIR)/libavahi_core_la-log.Plo
#	source='log.c' object='libavahi_core_a-log.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-log.o `test -f 'log.c' || echo '$(srcdir)/'`log.c

libavahi_core_a-browse-dns-server.o: browse-dns-server.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-browse-dns-server.Tpo -c -o libavahi_core_a-browse-dns-server.o `test -f 'browse-dns-server.c' || echo '$(srcdir)/'`browse-dns-server.c
	mv -f $(DEPDIR)/libavahi_core_la-browse-dns-server.Tpo $(DEPDIR)/libavahi_core_la-browse-dns-server.Plo
#	source='browse-dns-server.c' object='libavahi_core_a-browse-dns-server.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-browse-dns-server.o `test -f 'browse-dns-server.c' || echo '$(srcdir)/'`browse-dns-server.c

libavahi_core_a-fdutil.o: fdutil.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-fdutil.Tpo -c -o libavahi_core_a-fdutil.o `test -f 'fdutil.c' || echo '$(srcdir)/'`fdutil.c
	mv -f $(DEPDIR)/libavahi_core_la-fdutil.Tpo $(DEPDIR)/libavahi_core_la-fdutil.Plo
#	source='fdutil.c' object='libavahi_core_a-fdutil.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-fdutil.o `test -f 'fdutil.c' || echo '$(srcdir)/'`fdutil.c

libavahi_core_a-util.o: util.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-util.Tpo -c -o libavahi_core_a-util.o `test -f 'util.c' || echo '$(srcdir)/'`util.c
	mv -f $(DEPDIR)/libavahi_core_la-util.Tpo $(DEPDIR)/libavahi_core_la-util.Plo
#	source='util.c' object='libavahi_core_a-util.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-util.o `test -f 'util.c' || echo '$(srcdir)/'`util.c

libavahi_core_a-hashmap.o: hashmap.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-hashmap.Tpo -c -o libavahi_core_a-hashmap.o `test -f 'hashmap.c' || echo '$(srcdir)/'`hashmap.c
	mv -f $(DEPDIR)/libavahi_core_la-hashmap.Tpo $(DEPDIR)/libavahi_core_la-hashmap.Plo
#	source='hashmap.c' object='libavahi_core_a-hashmap.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-hashmap.o `test -f 'hashmap.c' || echo '$(srcdir)/'`hashmap.c

libavahi_core_a-wide-area.o: wide-area.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-wide-area.Tpo -c -o libavahi_core_a-wide-area.o `test -f 'wide-area.c' || echo '$(srcdir)/'`wide-area.c
	mv -f $(DEPDIR)/libavahi_core_la-wide-area.Tpo $(DEPDIR)/libavahi_core_la-wide-area.Plo
#	source='wide-area.c' object='libavahi_core_a-wide-area.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-wide-area.o `test -f 'wide-area.c' || echo '$(srcdir)/'`wide-area.c

libavahi_core_a-multicast-lookup.o: multicast-lookup.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-multicast-lookup.Tpo -c -o libavahi_core_a-multicast-lookup.o `test -f 'multicast-lookup.c' || echo '$(srcdir)/'`multicast-lookup.c
	mv -f $(DEPDIR)/libavahi_core_la-multicast-lookup.Tpo $(DEPDIR)/libavahi_core_la-multicast-lookup.Plo
#	source='multicast-lookup.c' object='libavahi_core_a-multicast-lookup.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-multicast-lookup.o `test -f 'multicast-lookup.c' || echo '$(srcdir)/'`multicast-lookup.c

libavahi_core_a-querier.o: querier.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-querier.Tpo -c -o libavahi_core_a-querier.o `test -f 'querier.c' || echo '$(srcdir)/'`querier.c
	mv -f $(DEPDIR)/libavahi_core_la-querier.Tpo $(DEPDIR)/libavahi_core_la-querier.Plo
#	source='querier.c' object='libavahi_core_a-querier.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-querier.o `test -f 'querier.c' || echo '$(srcdir)/'`querier.c

libavahi_core_a-addr-util.o: addr-util.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-addr-util.Tpo -c -o libavahi_core_a-addr-util.o `test -f 'addr-util.c' || echo '$(srcdir)/'`addr-util.c
	mv -f $(DEPDIR)/libavahi_core_la-addr-util.Tpo $(DEPDIR)/libavahi_core_la-addr-util.Plo
#	source='addr-util.c' object='libavahi_core_a-addr-util.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-addr-util.o `test -f 'addr-util.c' || echo '$(srcdir)/'`addr-util.c

libavahi_core_a-domain-util.o: domain-util.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-domain-util.Tpo -c -o libavahi_core_a-domain-util.o `test -f 'domain-util.c' || echo '$(srcdir)/'`domain-util.c
	mv -f $(DEPDIR)/libavahi_core_la-domain-util.Tpo $(DEPDIR)/libavahi_core_la-domain-util.Plo
#	source='domain-util.c' object='libavahi_core_a-domain-util.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-domain-util.o `test -f 'domain-util.c' || echo '$(srcdir)/'`domain-util.c

libavahi_core_a-iface-linux.o: iface-linux.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-iface-linux.Tpo -c -o libavahi_core_a-iface-linux.o `test -f 'iface-linux.c' || echo '$(srcdir)/'`iface-linux.c
	mv -f $(DEPDIR)/libavahi_core_la-iface-linux.Tpo $(DEPDIR)/libavahi_core_la-iface-linux.Plo
#	source='iface-linux.c' object='libavahi_core_a-iface-linux.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-iface-linux.o `test -f 'iface-linux.c' || echo '$(srcdir)/'`iface-linux.c

libavahi_core_a-netlink.o: netlink.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-netlink.Tpo -c -o libavahi_core_a-netlink.o `test -f 'netlink.c' || echo '$(srcdir)/'`netlink.c
	mv -f $(DEPDIR)/libavahi_core_la-netlink.Tpo $(DEPDIR)/libavahi_core_la-netlink.Plo
#	source='netlink.c' object='libavahi_core_a-netlink.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-netlink.o `test -f 'netlink.c' || echo '$(srcdir)/'`netlink.c

libavahi_core_a-iface-pfroute.o: iface-pfroute.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-iface-pfroute.Tpo -c -o libavahi_core_a-iface-pfroute.o `test -f 'iface-pfroute.c' || echo '$(srcdir)/'`iface-pfroute.c
	mv -f $(DEPDIR)/libavahi_core_la-iface-pfroute.Tpo $(DEPDIR)/libavahi_core_la-iface-pfroute.Plo
#	source='iface-pfroute.c' object='libavahi_core_a-iface-pfroute.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-iface-pfroute.o `test -f 'iface-pfroute.c' || echo '$(srcdir)/'`iface-pfroute.c

libavahi_core_a-iface-none.o: iface-none.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/libavahi_core_la-iface-none.Tpo -c -o libavahi_core_a-iface-none.o `test -f 'iface-none.c' || echo '$(srcdir)/'`iface-none.c
	mv -f $(DEPDIR)/libavahi_core_la-iface-none.Tpo $(DEPDIR)/libavahi_core_la-iface-none.Plo
#	source='iface-none.c' object='libavahi_core_a-iface-none.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_core_a_CFLAGS) $(CFLAGS) -c -o libavahi_core_a-iface-none.o `test -f 'iface-none.c' || echo '$(srcdir)/'`iface-none.c

avahi_reflector-avahi-reflector.o: avahi-reflector.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_reflector_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/avahi_reflector-avahi-reflector.Tpo -c -o avahi_reflector-avahi-reflector.o `test -f 'avahi-reflector.c' || echo '$(srcdir)/'`avahi-reflector.c
	mv -f $(DEPDIR)/avahi_reflector-avahi-reflector.Tpo $(DEPDIR)/avahi_reflector-avahi-reflector.Po
#	source='avahi-reflector.c' object='avahi_reflector-avahi-reflector.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_reflector_CFLAGS) $(CFLAGS) -c -o avahi_reflector-avahi-reflector.o `test -f 'avahi-reflector.c' || echo '$(srcdir)/'`avahi-reflector.c

avahi_test-avahi-test.o: avahi-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/avahi_test-avahi-test.Tpo -c -o avahi_test-avahi-test.o `test -f 'avahi-test.c' || echo '$(srcdir)/'`avahi-test.c
	mv -f $(DEPDIR)/avahi_test-avahi-test.Tpo $(DEPDIR)/avahi_test-avahi-test.Po
#	source='avahi-test.c' object='avahi_test-avahi-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_test_CFLAGS) $(CFLAGS) -c -o avahi_test-avahi-test.o `test -f 'avahi-test.c' || echo '$(srcdir)/'`avahi-test.c

conformance_test-conformance-test.o: conformance-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(conformance_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/conformance_test-conformance-test.Tpo -c -o conformance_test-conformance-test.o `test -f 'conformance-test.c' || echo '$(srcdir)/'`conformance-test.c
	mv -f $(DEPDIR)/conformance_test-conformance-test.Tpo $(DEPDIR)/conformance_test-conformance-test.Po
#	source='conformance-test.c' object='conformance_test-conformance-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(conformance_test_CFLAGS) $(CFLAGS) -c -o conformance_test-conformance-test.o `test -f 'conformance-test.c' || echo '$(srcdir)/'`conformance-test.c

dns_spin_test-dns-spin-test.o: dns-spin-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_spin_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/dns_spin_test-dns-spin-test.Tpo -c -o dns_spin_test-dns-spin-test.o `test -f 'dns-spin-test.c' || echo '$(srcdir)/'`dns-spin-test.c
	mv -f $(DEPDIR)/dns_spin_test-dns-spin-test.Tpo $(DEPDIR)/dns_spin_test-dns-spin-test.Po
#	source='dns-spin-test.c' object='dns_spin_test-dns-spin-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_spin_test_CFLAGS) $(CFLAGS) -c -o dns_spin_test-dns-spin-test.o `test -f 'dns-spin-test.c' || echo '$(srcdir)/'`dns-spin-test.c

dns_test-dns.o: dns.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/dns_test-dns.Tpo -c -o dns_test-dns.o `test -f 'dns.c' || echo '$(srcdir)/'`dns.c
	mv -f $(DEPDIR)/dns_test-dns.Tpo $(DEPDIR)/dns_test-dns.Po
#	source='dns.c' object='dns_test-dns.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -c -o dns_test-dns.o `test -f 'dns.c' || echo '$(srcdir)/'`dns.c

dns_test-dns-test.o: dns-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/dns_test-dns-test.Tpo -c -o dns_test-dns-test.o `test -f 'dns-test.c' || echo '$(srcdir)/'`dns-test.c
	mv -f $(DEPDIR)/dns_test-dns-test.Tpo $(DEPDIR)/dns_test-dns-test.Po
#	source='dns-test.c' object='dns_test-dns-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -c -o dns_test-dns-test.o `test -f 'dns-test.c' || echo '$(srcdir)/'`dns-test.c

dns_test-log.o: log.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/dns_test-log.Tpo -c -o dns_test-log.o `test -f 'log.c' || echo '$(srcdir)/'`log.c
	mv -f $(DEPDIR)/dns_test-log.Tpo $(DEPDIR)/dns_test-log.Po
#	source='log.c' object='dns_test-log.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -c -o dns_test-log.o `test -f 'log.c' || echo '$(srcdir)/'`log.c

dns_test-util.o: util.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/dns_test-util.Tpo -c -o dns_test-util.o `test -f 'util.c' || echo '$(srcdir)/'`util.c
	mv -f $(DEPDIR)/dns_test-util.Tpo $(DEPDIR)/dns_test-util.Po
#	source='util.c' object='dns_test-util.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -c -o dns_test-util.o `test -f 'util.c' || echo '$(srcdir)/'`util.c

dns_test-rr.o: rr.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/dns_test-rr.Tpo -c -o dns_test-rr.o `test -f 'rr.c' || echo '$(srcdir)/'`rr.c
	mv -f $(DEPDIR)/dns_test-rr.Tpo $(DEPDIR)/dns_test-rr.Po
#	source='rr.c' object='dns_test-rr.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -c -o dns_test-rr.o `test -f 'rr.c' || echo '$(srcdir)/'`rr.c

dns_test-hashmap.o: hashmap.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/dns_test-hashmap.Tpo -c -o dns_test-hashmap.o `test -f 'hashmap.c' || echo '$(srcdir)/'`hashmap.c
	mv -f $(DEPDIR)/dns_test-hashmap.Tpo $(DEPDIR)/dns_test-hashmap.Po
#	source='hashmap.c' object='dns_test-hashmap.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -c -o dns_test-hashmap.o `test -f 'hashmap.c' || echo '$(srcdir)/'`hashmap.c

dns_test-domain-util.o: domain-util.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/dns_test-domain-util.Tpo -c -o dns_test-domain-util.o `test -f 'domain-util.c' || echo '$(srcdir)/'`domain-util.c
	mv -f $(DEPDIR)/dns_test-domain-util.Tpo $(DEPDIR)/dns_test-domain-util.Po
#	source='domain-util.c' object='dns_test-domain-util.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(dns_test_CFLAGS) $(CFLAGS) -c -o dns_test-domain-util.o `test -f 'domain-util.c' || echo '$(srcdir)/'`domain-util.c

hashmap_test-hashmap-test.o: hashmap-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(hashmap_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/hashmap_test-hashmap-test.Tpo -c -o hashmap_test-hashmap-test.o `test -f 'hashmap-test.c' || echo '$(srcdir)/'`hashmap-test.c
	mv -f $(DEPDIR)/hashmap_test-hashmap-test.Tpo $(DEPDIR)/hashmap_test-hashmap-test.Po
#	source='hashmap-test.c' object='hashmap_test-hashmap-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(hashmap_test_CFLAGS) $(CFLAGS) -c -o hashmap_test-hashmap-test.o `test -f 'hashmap-test.c' || echo '$(srcdir)/'`hashmap-test.c

hashmap_test-hashmap.o: hashmap.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(hashmap_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/hashmap_test-hashmap.Tpo -c -o hashmap_test-hashmap.o `test -f 'hashmap.c' || echo '$(srcdir)/'`hashmap.c
	mv -f $(DEPDIR)/hashmap_test-hashmap.Tpo $(DEPDIR)/hashmap_test-hashmap.Po
#	source='hashmap.c' object='hashmap_test-hashmap.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(hashmap_test_CFLAGS) $(CFLAGS) -c -o hashmap_test-hashmap.o `test -f 'hashmap.c' || echo '$(srcdir)/'`hashmap.c

hashmap_test-util.o: util.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(hashmap_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/hashmap_test-util.Tpo -c -o hashmap_test-util.o `test -f 'util.c' || echo '$(srcdir)/'`util.c
	mv -f $(DEPDIR)/hashmap_test-util.Tpo $(DEPDIR)/hashmap_test-util.Po
#	source='util.c' object='hashmap_test-util.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(hashmap_test_CFLAGS) $(CFLAGS) -c -o hashmap_test-util.o `test -f 'util.c' || echo '$(srcdir)/'`util.c

prioq_test-prioq-test.o: prioq-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(prioq_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/prioq_test-prioq-test.Tpo -c -o prioq_test-prioq-test.o `test -f 'prioq-test.c' || echo '$(srcdir)/'`prioq-test.c
	mv -f $(DEPDIR)/prioq_test-prioq-test.Tpo $(DEPDIR)/prioq_test-prioq-test.Po
#	source='prioq-test.c' object='prioq_test-prioq-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(prioq_test_CFLAGS) $(CFLAGS) -c -o prioq_test-prioq-test.o `test -f 'prioq-test.c' || echo '$(srcdir)/'`prioq-test.c

prioq_test-prioq.o: prioq.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(prioq_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/prioq_test-prioq.Tpo -c -o prioq_test-prioq.o `test -f 'prioq.c' || echo '$(srcdir)/'`prioq.c
	mv -f $(DEPDIR)/prioq_test-prioq.Tpo $(DEPDIR)/prioq_test-prioq.Po
#	source='prioq.c' object='prioq_test-prioq.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(prioq_test_CFLAGS) $(CFLAGS) -c -o prioq_test-prioq.o `test -f 'prioq.c' || echo '$(srcdir)/'`prioq.c

querier_test-querier-test.o: querier-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(querier_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/querier_test-querier-test.Tpo -c -o querier_test-querier-test.o `test -f 'querier-test.c' || echo '$(srcdir)/'`querier-test.c
	mv -f $(DEPDIR)/querier_test-querier-test.Tpo $(DEPDIR)/querier_test-querier-test.Po
#	source='querier-test.c' object='querier_test-querier-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(querier_test_CFLAGS) $(CFLAGS) -c -o querier_test-querier-test.o `test -f 'querier-test.c' || echo '$(srcdir)/'`querier-test.c

timeeventq_test-timeeventq-test.o: timeeventq-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeeventq_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/timeeventq_test-timeeventq-test.Tpo -c -o timeeventq_test-timeeventq-test.o `test -f 'timeeventq-test.c' || echo '$(srcdir)/'`timeeventq-test.c
	mv -f $(DEPDIR)/timeeventq_test-timeeventq-test.Tpo $(DEPDIR)/timeeventq_test-timeeventq-test.Po
#	source='timeeventq-test.c' object='timeeventq_test-timeeventq-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeeventq_test_CFLAGS) $(CFLAGS) -c -o timeeventq_test-timeeventq-test.o `test -f 'timeeventq-test.c' || echo '$(srcdir)/'`timeeventq-test.c

timeeventq_test-timeeventq.o: timeeventq.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeeventq_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/timeeventq_test-timeeventq.Tpo -c -o timeeventq_test-timeeventq.o `test -f 'timeeventq.c' || echo '$(srcdir)/'`timeeventq.c
	mv -f $(DEPDIR)/timeeventq_test-timeeventq.Tpo $(DEPDIR)/timeeventq_test-timeeventq.Po
#	source='timeeventq.c' object='timeeventq_test-timeeventq.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeeventq_test_CFLAGS) $(CFLAGS) -c -o timeeventq_test-timeeventq.o `test -f 'timeeventq.c' || echo '$(srcdir)/'`timeeventq.c

timeeventq_test-prioq.o: prioq.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeeventq_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/timeeventq_test-prioq.Tpo -c -o timeeventq_test-prioq.o `test -f 'prioq.c' || echo '$(srcdir)/'`prioq.c
	mv -f $(DEPDIR)/timeeventq_test-prioq.Tpo $(DEPDIR)/timeeventq_test-prioq.Po
#	source='prioq.c' object='timeeventq_test-prioq.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeeventq_test_CFLAGS) $(CFLAGS) -c -o timeeventq_test-prioq.o `test -f 'prioq.c' || echo '$(srcdir)/'`prioq.c

timeeventq_test-log.o: log.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeeventq_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/timeeventq_test-log.Tpo -c -o timeeventq_test-log.o `test -f 'log.c' || echo '$(srcdir)/'`log.c
	mv -f $(DEPDIR)/timeeventq_test-log.Tpo $(DEPDIR)/timeeventq_test-log.Po
#	source='log.c' object='timeeventq_test-log.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeeventq_test_CFLAGS) $(CFLAGS) -c -o timeeventq_test-log.o `test -f 'log.c' || echo '$(srcdir)/'`log.c

update_test-update-test.o: update-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(update_test_CFLAGS) $(CFLAGS) -MD -MP -MF $(DEPDIR)/update_test-update-test.Tpo -c -o update_test-update-test.o `test -f 'update-test.c' || echo '$(srcdir)/'`update-test.c
	mv -f $(DEPDIR)/update_test-update-test.Tpo $(DEPDIR)/update_test-update-test.Po
#	source='update-test.c' object='update_test-update-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(update_test_CFLAGS) $(CFLAGS) -c -o update_test-update-test.o `test -f 'update-test.c' || echo '$(srcdir)/'`update-test.c

install-avahiincludeHEADERS: $(avahiinclude_HEADERS)
	@$(NORMAL_INSTALL)
	test -z "$(avahiincludedir)" || $(MKDIR_P) "$(DESTDIR)$(avahiincludedir)"
	@list='$(avahiinclude_HEADERS)'; for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(avahiincludeHEADERS_INSTALL) '$$d$$p' '$(DESTDIR)$(avahiincludedir)/$$f'"; \
	  $(avahiincludeHEADERS_INSTALL) "$$d$$p" "$(DESTDIR)$(avahiincludedir)/$$f"; \
	done

uninstall-avahiincludeHEADERS:
	@$(NORMAL_UNINSTALL)
	@list='$(avahiinclude_HEADERS)'; for p in $$list; do \
	  f=$(am__strip_dir) \
	  echo " rm -f '$(DESTDIR)$(avahiincludedir)/$$f'"; \
	  rm -f "$(DESTDIR)$(avahiincludedir)/$$f"; \
	done

ID: $(HEADERS) $(SOURCES) $(LISP) $(TAGS_FILES)
	list='$(SOURCES) $(HEADERS) $(LISP) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '{ files[$$0] = 1; nonempty = 1; } \
	      END { if (nonempty) { for (i in files) print i; }; }'`; \
	mkid -fID $$unique
tags: TAGS

TAGS:  $(HEADERS) $(SOURCES)  $(TAGS_DEPENDENCIES) \
		$(TAGS_FILES) $(LISP)
	tags=; \
	here=`pwd`; \
	list='$(SOURCES) $(HEADERS)  $(LISP) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '{ files[$$0] = 1; nonempty = 1; } \
	      END { if (nonempty) { for (i in files) print i; }; }'`; \
	if test -z "$(ETAGS_ARGS)$$tags$$unique"; then :; else \
	  test -n "$$unique" || unique=$$empty_fix; \
	  $(ETAGS) $(ETAGSFLAGS) $(AM_ETAGSFLAGS) $(ETAGS_ARGS) \
	    $$tags $$unique; \
	fi
ctags: CTAGS
CTAGS:  $(HEADERS) $(SOURCES)  $(TAGS_DEPENDENCIES) \
		$(TAGS_FILES) $(LISP)
	tags=; \
	list='$(SOURCES) $(HEADERS)  $(LISP) $(TAGS_FILES)'; \
	unique=`for i in $$list; do \
	    if test -f "$$i"; then echo $$i; else echo $(srcdir)/$$i; fi; \
	  done | \
	  $(AWK) '{ files[$$0] = 1; nonempty = 1; } \
	      END { if (nonempty) { for (i in files) print i; }; }'`; \
	test -z "$(CTAGS_ARGS)$$tags$$unique" \
	  || $(CTAGS) $(CTAGSFLAGS) $(AM_CTAGSFLAGS) $(CTAGS_ARGS) \
	     $$tags $$unique

GTAGS:
	here=`$(am__cd) $(top_builddir) && pwd` \
	  && cd $(top_srcdir) \
	  && gtags -i $(GTAGS_ARGS) $$here

distclean-tags:
	-rm -f TAGS ID GTAGS GRTAGS GSYMS GPATH tags

check-TESTS: $(TESTS)
	@failed=0; all=0; xfail=0; xpass=0; skip=0; \
	srcdir=$(srcdir); export srcdir; \
	list=' $(TESTS) '; \
	if test -n "$$list"; then \
	  for tst in $$list; do \
	    if test -f ./$$tst; then dir=./; \
	    elif test -f $$tst; then dir=; \
	    else dir="$(srcdir)/"; fi; \
	    if $(TESTS_ENVIRONMENT) $${dir}$$tst; then \
	      all=`expr $$all + 1`; \
	      case " $(XFAIL_TESTS) " in \
	      *[\ \	]$$tst[\ \	]*) \
		xpass=`expr $$xpass + 1`; \
		failed=`expr $$failed + 1`; \
		echo "XPASS: $$tst"; \
	      ;; \
	      *) \
		echo "PASS: $$tst"; \
	      ;; \
	      esac; \
	    elif test $$? -ne 77; then \
	      all=`expr $$all + 1`; \
	      case " $(XFAIL_TESTS) " in \
	      *[\ \	]$$tst[\ \	]*) \
		xfail=`expr $$xfail + 1`; \
		echo "XFAIL: $$tst"; \
	      ;; \
	      *) \
		failed=`expr $$failed + 1`; \
		echo "FAIL: $$tst"; \
	      ;; \
	      esac; \
	    else \
	      skip=`expr $$skip + 1`; \
	      echo "SKIP: $$tst"; \
	    fi; \
	  done; \
	  if test "$$all" -eq 1; then \
	    tests="test"; \
	    All=""; \
	  else \
	    tests="tests"; \
	    All="All "; \
	  fi; \
	  if test "$$failed" -eq 0; then \
	    if test "$$xfail" -eq 0; then \
	      banner="$$All$$all $$tests passed"; \
	    else \
	      if test "$$xfail" -eq 1; then failures=failure; else failures=failures; fi; \
	      banner="$$All$$all $$tests behaved as expected ($$xfail expected $$failures)"; \
	    fi; \
	  else \
	    if test "$$xpass" -eq 0; then \
	      banner="$$failed of $$all $$tests failed"; \
	    else \
	      if test "$$xpass" -eq 1; then passes=pass; else passes=passes; fi; \
	      banner="$$failed of $$all $$tests did not behave as expected ($$xpass unexpected $$passes)"; \
	    fi; \
	  fi; \
	  dashes="$$banner"; \
	  skipped=""; \
	  if test "$$skip" -ne 0; then \
	    if test "$$skip" -eq 1; then \
	      skipped="($$skip test was not run)"; \
	    else \
	      skipped="($$skip tests were not run)"; \
	    fi; \
	    test `echo "$$skipped" | wc -c` -le `echo "$$banner" | wc -c` || \
	      dashes="$$skipped"; \
	  fi; \
	  report=""; \
	  if test "$$failed" -ne 0 && test -n "$(PACKAGE_BUGREPORT)"; then \
	    report="Please report to $(PACKAGE_BUGREPORT)"; \
	    test `echo "$$report" | wc -c` -le `echo "$$banner" | wc -c` || \
	      dashes="$$report"; \
	  fi; \
	  dashes=`echo "$$dashes" | sed s/./=/g`; \
	  echo "$$dashes"; \
	  echo "$$banner"; \
	  test -z "$$skipped" || echo "$$skipped"; \
	  test -z "$$report" || echo "$$report"; \
	  echo "$$dashes"; \
	  test "$$failed" -eq 0; \
	else :; fi

distdir: $(DISTFILES)
	@srcdirstrip=`echo "$(srcdir)" | sed 's/[].[^$$\\*]/\\\\&/g'`; \
	topsrcdirstrip=`echo "$(top_srcdir)" | sed 's/[].[^$$\\*]/\\\\&/g'`; \
	list='$(DISTFILES)'; \
	  dist_files=`for file in $$list; do echo $$file; done | \
	  sed -e "s|^$$srcdirstrip/||;t" \
	      -e "s|^$$topsrcdirstrip/|$(top_builddir)/|;t"`; \
	case $$dist_files in \
	  */*) $(MKDIR_P) `echo "$$dist_files" | \
			   sed '/\//!d;s|^|$(distdir)/|;s,/[^/]*$$,,' | \
			   sort -u` ;; \
	esac; \
	for file in $$dist_files; do \
	  if test -f $$file || test -d $$file; then d=.; else d=$(srcdir); fi; \
	  if test -d $$d/$$file; then \
	    dir=`echo "/$$file" | sed -e 's,/[^/]*$$,,'`; \
	    if test -d $(srcdir)/$$file && test $$d != $(srcdir); then \
	      cp -pR $(srcdir)/$$file $(distdir)$$dir || exit 1; \
	    fi; \
	    cp -pR $$d/$$file $(distdir)$$dir || exit 1; \
	  else \
	    test -f $(distdir)/$$file \
	    || cp -p $$d/$$file $(distdir)/$$file \
	    || exit 1; \
	  fi; \
	done
check-am: all-am
	$(MAKE) $(AM_MAKEFLAGS) check-TESTS
check: check-am
all-am: $(LTLIBRARIES) $(PROGRAMS) $(HEADERS)
installdirs:
	for dir in "$(DESTDIR)$(libdir)" "$(DESTDIR)$(avahiincludedir)"; do \
	  test -z "$$dir" || $(MKDIR_P) "$$dir"; \
	done
install: install-am
install-exec: install-exec-am
install-data: install-data-am
uninstall: uninstall-am

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am

installcheck: installcheck-am
install-strip:
	$(MAKE) $(AM_MAKEFLAGS) INSTALL_PROGRAM="$(INSTALL_STRIP_PROGRAM)" \
	  install_sh_PROGRAM="$(INSTALL_STRIP_PROGRAM)" INSTALL_STRIP_FLAG=-s \
	  `test -z '$(STRIP)' || \
	    echo "INSTALL_PROGRAM_ENV=STRIPPROG='$(STRIP)'"` install
mostlyclean-generic:

clean-generic:

distclean-generic:
	-test -z "$(CONFIG_CLEAN_FILES)" || rm -f $(CONFIG_CLEAN_FILES)

maintainer-clean-generic:
	@echo "This command is intended for maintainers to use"
	@echo "it deletes files that may require special tools to rebuild."
clean: clean-am

clean-am: clean-generic clean-libLTLIBRARIES \
	clean-noinstPROGRAMS mostlyclean-am

distclean: distclean-am
	-rm -rf ./$(DEPDIR)
	-rm -f Makefile
distclean-am: clean-am distclean-compile distclean-generic \
	distclean-tags

dvi: dvi-am

dvi-am:

html: html-am

info: info-am

info-am:

install-data-am: install-avahiincludeHEADERS

install-dvi: install-dvi-am

install-exec-am: install-libLTLIBRARIES

install-html: install-html-am

install-info: install-info-am

install-man:

install-pdf: install-pdf-am

install-ps: install-ps-am

installcheck-am:

maintainer-clean: maintainer-clean-am
	-rm -rf ./$(DEPDIR)
	-rm -f Makefile
maintainer-clean-am: distclean-am maintainer-clean-generic

mostlyclean: mostlyclean-am

mostlyclean-am: mostlyclean-compile mostlyclean-generic

pdf: pdf-am

pdf-am:

ps: ps-am

ps-am:

uninstall-am: uninstall-avahiincludeHEADERS uninstall-libLTLIBRARIES

.MAKE: install-am install-strip

.PHONY: CTAGS GTAGS all all-am check check-TESTS check-am clean \
	clean-generic clean-libLTLIBRARIES \
	clean-noinstPROGRAMS ctags distclean distclean-compile \
	distclean-generic distclean-tags distdir dvi \
	dvi-am html html-am info info-am install install-am \
	install-avahiincludeHEADERS install-data install-data-am \
	install-dvi install-dvi-am install-exec install-exec-am \
	install-html install-html-am install-info install-info-am \
	install-libLTLIBRARIES install-man install-pdf install-pdf-am \
	install-ps install-ps-am install-strip installcheck \
	installcheck-am installdirs maintainer-clean \
	maintainer-clean-generic mostlyclean mostlyclean-compile \
	mostlyclean-generic pdf pdf-am ps ps-am \
	tags uninstall uninstall-am uninstall-avahiincludeHEADERS \
	uninstall-libLTLIBRARIES


valgrind: avahi-test
	valgrind ./avahi-test

gdb: avahi-test
	gdb ./avahi-test
# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
