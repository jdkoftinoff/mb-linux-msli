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
#noinst_PROGRAMS = strlst-test \
#	domain-test \
#	alternative-test \
#	timeval-test watch-test \
#	watch-test-thread \
#	utf8-test
subdir = avahi-common
DIST_COMMON = $(am__noinst_HEADERS_DIST) \
	$(avahi_commoninclude_HEADERS) $(srcdir)/Makefile.am \
	$(srcdir)/Makefile.in
CONFIG_HEADER = $(top_builddir)/config.h
CONFIG_CLEAN_FILES =
am__vpath_adj_setup = srcdirstrip=`echo "$(srcdir)" | sed 's|.|.|g'`;
am__vpath_adj = case $$p in \
    $(srcdir)/*) f=`echo "$$p" | sed "s|^$$srcdirstrip/||"`;; \
    *) f=$$p;; \
  esac;
am__strip_dir = `echo $$p | sed -e 's|^.*/||'`;
am__installdirs = "$(DESTDIR)$(libdir)" \
	"$(DESTDIR)$(avahi_commonincludedir)"
libLTLIBRARIES_INSTALL = $(INSTALL)
LTLIBRARIES = $(lib_LTLIBRARIES)
am__DEPENDENCIES_1 =
libavahi_common_a_DEPENDENCIES = $(am__DEPENDENCIES_1) \
	$(am__DEPENDENCIES_1)
am_libavahi_common_a_OBJECTS = libavahi_common_a-malloc.o \
	libavahi_common_a-address.o \
	libavahi_common_a-alternative.o libavahi_common_a-error.o \
	libavahi_common_a-strlst.o libavahi_common_a-domain.o \
	libavahi_common_a-timeval.o \
	libavahi_common_a-simple-watch.o \
	libavahi_common_a-thread-watch.o libavahi_common_a-rlist.o \
	libavahi_common_a-utf8.o libavahi_common_a-i18n.o
libavahi_common_a_OBJECTS = $(am_libavahi_common_a_OBJECTS)
PROGRAMS = $(noinst_PROGRAMS)
am_alternative_test_OBJECTS = alternative_test-alternative.o \
	alternative_test-malloc.o \
	alternative_test-domain.o \
	alternative_test-address.o \
	alternative_test-alternative-test.o \
	alternative_test-utf8.o
alternative_test_OBJECTS = $(am_alternative_test_OBJECTS)
alternative_test_LDADD = $(LDADD)
alternative_test_LINK = $(LD) $(alternative_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_domain_test_OBJECTS = domain_test-domain.o \
	domain_test-malloc.o domain_test-address.o \
	domain_test-domain-test.o domain_test-utf8.o
domain_test_OBJECTS = $(am_domain_test_OBJECTS)
domain_test_LDADD = $(LDADD)
domain_test_LINK = $(LD) $(domain_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_strlst_test_OBJECTS = strlst_test-strlst.o \
	strlst_test-malloc.o strlst_test-strlst-test.o
strlst_test_OBJECTS = $(am_strlst_test_OBJECTS)
strlst_test_LDADD = $(LDADD)
strlst_test_LINK = $(LD) $(strlst_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_timeval_test_OBJECTS = timeval_test-timeval.o \
	timeval_test-timeval-test.o
timeval_test_OBJECTS = $(am_timeval_test_OBJECTS)
timeval_test_DEPENDENCIES = $(am__DEPENDENCIES_1) \
	$(am__DEPENDENCIES_1)
timeval_test_LINK = $(LD) $(timeval_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_utf8_test_OBJECTS = utf8_test-utf8-test.o \
	utf8_test-utf8.o
utf8_test_OBJECTS = $(am_utf8_test_OBJECTS)
utf8_test_DEPENDENCIES =
utf8_test_LINK = $(LD) $(utf8_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am_watch_test_OBJECTS = watch_test-timeval.o \
	watch_test-simple-watch.o watch_test-malloc.o \
	watch_test-watch-test.o
watch_test_OBJECTS = $(am_watch_test_OBJECTS)
watch_test_DEPENDENCIES = $(am__DEPENDENCIES_1) $(am__DEPENDENCIES_1)
watch_test_LINK = $(LD) $(watch_test_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am__objects_1 = watch_test_thread-timeval.o \
	watch_test_thread-simple-watch.o \
	watch_test_thread-malloc.o \
	watch_test_thread-watch-test.o
am_watch_test_thread_OBJECTS = $(am__objects_1) \
	watch_test_thread-thread-watch.o
watch_test_thread_OBJECTS = $(am_watch_test_thread_OBJECTS)
am__DEPENDENCIES_2 = $(am__DEPENDENCIES_1) $(am__DEPENDENCIES_1)
watch_test_thread_DEPENDENCIES = $(am__DEPENDENCIES_2)
watch_test_thread_LINK = $(LD) \
	$(watch_test_thread_CFLAGS) $(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) \
	-o $@
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
SOURCES = $(libavahi_common_a_SOURCES) $(alternative_test_SOURCES) \
	$(domain_test_SOURCES) $(strlst_test_SOURCES) \
	$(timeval_test_SOURCES) $(utf8_test_SOURCES) \
	$(watch_test_SOURCES) $(watch_test_thread_SOURCES)
DIST_SOURCES = $(libavahi_common_a_SOURCES) \
	$(alternative_test_SOURCES) $(domain_test_SOURCES) \
	$(strlst_test_SOURCES) $(timeval_test_SOURCES) \
	$(utf8_test_SOURCES) $(watch_test_SOURCES) \
	$(watch_test_thread_SOURCES)
avahi_commonincludeHEADERS_INSTALL = $(INSTALL_HEADER)
am__noinst_HEADERS_DIST = dbus.h dbus-watch-glue.h
HEADERS = $(avahi_commoninclude_HEADERS) $(noinst_HEADERS)
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
abs_builddir = $(ROOTDIR)/user/avahi/avahi-common
abs_srcdir = $(ROOTDIR)/user/avahi/avahi-common
abs_top_builddir = $(ROOTDIR)/user/avahi
abs_top_srcdir = $(ROOTDIR)/user/avahi
ac_ct_CC = $(CC)
ac_ct_CXX = $(CXX)
ac_ct_DUMPBIN = 
acx_pthread_config = 
am__include = include
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
avahi_commonincludedir = $(includedir)/avahi-common
avahi_commoninclude_HEADERS = \
	strlst.h \
	address.h \
	alternative.h \
	domain.h \
	cdecl.h \
	defs.h \
	malloc.h \
	watch.h \
	timeval.h \
	simple-watch.h \
	thread-watch.h \
	gccmacro.h \
	error.h \
	llist.h \
	rlist.h

lib_LTLIBRARIES = \
	libavahi-common.a

libavahi_common_a_SOURCES = \
	malloc.c malloc.h \
	address.c address.h \
	alternative.c alternative.h \
	error.c error.h \
	strlst.c strlst.h \
	domain.c domain.h \
	timeval.c timeval.h \
	simple-watch.c simple-watch.h \
	thread-watch.c thread-watch.h \
	watch.h gccmacro.h \
	rlist.h rlist.c \
	utf8.c utf8.h \
	i18n.c i18n.h

libavahi_common_a_CFLAGS = $(AM_CFLAGS) $(PTHREAD_CFLAGS) -DAVAHI_LOCALEDIR=\"$(avahilocaledir)\"
libavahi_common_a_LIBADD = $(AM_LDADD) $(PTHREAD_CFLAGS) $(PTHREAD_LIBS) $(LIBINTL)
libavahi_common_a_LDFLAGS = $(AM_LDFLAGS)  -version-info $(LIBAVAHI_COMMON_VERSION_INFO)
strlst_test_SOURCES = \
	strlst.c strlst.h \
	malloc.c malloc.h \
	strlst-test.c

strlst_test_CFLAGS = $(AM_CFLAGS)
alternative_test_SOURCES = \
	alternative.c alternative.h \
	malloc.c malloc.h \
	domain.c domain.h \
	address.c address.h \
	alternative-test.c \
	utf8.c utf8.h

alternative_test_CFLAGS = $(AM_CFLAGS)
domain_test_SOURCES = \
	domain.c domain.h \
	malloc.c malloc.h \
	address.c address.h \
	domain-test.c \
	utf8.c utf8.h

domain_test_CFLAGS = $(AM_CFLAGS)
watch_test_SOURCES = \
	timeval.c timeval.h \
	simple-watch.c simple-watch.h \
	watch.h \
	malloc.c malloc.h \
	watch-test.c

watch_test_CFLAGS = $(AM_CFLAGS) $(PTHREAD_CFLAGS)
watch_test_LDADD = $(AM_LDADD) $(PTHREAD_LIBS) $(PTHREAD_CFLAGS)
watch_test_thread_SOURCES = $(watch_test_SOURCES) thread-watch.c thread-watch.h
watch_test_thread_CFLAGS = $(watch_test_CFLAGS) -DUSE_THREAD
watch_test_thread_LDADD = $(watch_test_LDADD)
timeval_test_SOURCES = \
	timeval.c timeval.h \
	timeval-test.c

timeval_test_CFLAGS = $(AM_CFLAGS) $(PTHREAD_CFLAGS)
timeval_test_LDADD = $(AM_LDADD) $(PTHREAD_LIBS) $(PTHREAD_CFLAGS)
utf8_test_SOURCES = \
	utf8-test.c \
	utf8.c utf8.h

utf8_test_CFLAGS = $(AM_CFLAGS)
utf8_test_LDADD = $(AM_LDADD)
#noinst_HEADERS = \
#	dbus.h \
#	dbus-watch-glue.h

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
	echo ' cd $(top_srcdir) && $(AUTOMAKE) --foreign  avahi-common/Makefile'; \
	cd $(top_srcdir) && \
	  $(AUTOMAKE) --foreign  avahi-common/Makefile
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
	  echo "  rm -f '$(DESTDIR)$(libdir)/$$p'"; \
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
libavahi-common.a: $(libavahi_common_a_OBJECTS) $(libavahi_common_a_DEPENDENCIES) 
	$(AR) crs $@ $(libavahi_common_a_OBJECTS)

clean-noinstPROGRAMS:
	@list='$(noinst_PROGRAMS)'; for p in $$list; do \
	  echo " rm -f $$p"; \
	  rm -f $$p ; \
	done
alternative-test: $(alternative_test_OBJECTS) $(alternative_test_DEPENDENCIES) 
	@rm -f alternative-test
	$(alternative_test_LINK) $(alternative_test_OBJECTS) $(alternative_test_LDADD) $(LIBS)
domain-test: $(domain_test_OBJECTS) $(domain_test_DEPENDENCIES) 
	@rm -f domain-test
	$(domain_test_LINK) $(domain_test_OBJECTS) $(domain_test_LDADD) $(LIBS)
strlst-test: $(strlst_test_OBJECTS) $(strlst_test_DEPENDENCIES) 
	@rm -f strlst-test
	$(strlst_test_LINK) $(strlst_test_OBJECTS) $(strlst_test_LDADD) $(LIBS)
timeval-test: $(timeval_test_OBJECTS) $(timeval_test_DEPENDENCIES) 
	@rm -f timeval-test
	$(timeval_test_LINK) $(timeval_test_OBJECTS) $(timeval_test_LDADD) $(LIBS)
utf8-test: $(utf8_test_OBJECTS) $(utf8_test_DEPENDENCIES) 
	@rm -f utf8-test
	$(utf8_test_LINK) $(utf8_test_OBJECTS) $(utf8_test_LDADD) $(LIBS)
watch-test: $(watch_test_OBJECTS) $(watch_test_DEPENDENCIES) 
	@rm -f watch-test
	$(watch_test_LINK) $(watch_test_OBJECTS) $(watch_test_LDADD) $(LIBS)
watch-test-thread: $(watch_test_thread_OBJECTS) $(watch_test_thread_DEPENDENCIES) 
	@rm -f watch-test-thread
	$(watch_test_thread_LINK) $(watch_test_thread_OBJECTS) $(watch_test_thread_LDADD) $(LIBS)

mostlyclean-compile:
	-rm -f *.o

distclean-compile:
	-rm -f *.tab.c

include ./$(DEPDIR)/alternative_test-address.Po
include ./$(DEPDIR)/alternative_test-alternative-test.Po
include ./$(DEPDIR)/alternative_test-alternative.Po
include ./$(DEPDIR)/alternative_test-domain.Po
include ./$(DEPDIR)/alternative_test-malloc.Po
include ./$(DEPDIR)/alternative_test-utf8.Po
include ./$(DEPDIR)/domain_test-address.Po
include ./$(DEPDIR)/domain_test-domain-test.Po
include ./$(DEPDIR)/domain_test-domain.Po
include ./$(DEPDIR)/domain_test-malloc.Po
include ./$(DEPDIR)/domain_test-utf8.Po
include ./$(DEPDIR)/libavahi_common_la-address.Plo
include ./$(DEPDIR)/libavahi_common_la-alternative.Plo
include ./$(DEPDIR)/libavahi_common_la-domain.Plo
include ./$(DEPDIR)/libavahi_common_la-error.Plo
include ./$(DEPDIR)/libavahi_common_la-i18n.Plo
include ./$(DEPDIR)/libavahi_common_la-malloc.Plo
include ./$(DEPDIR)/libavahi_common_la-rlist.Plo
include ./$(DEPDIR)/libavahi_common_la-simple-watch.Plo
include ./$(DEPDIR)/libavahi_common_la-strlst.Plo
include ./$(DEPDIR)/libavahi_common_la-thread-watch.Plo
include ./$(DEPDIR)/libavahi_common_la-timeval.Plo
include ./$(DEPDIR)/libavahi_common_la-utf8.Plo
include ./$(DEPDIR)/strlst_test-malloc.Po
include ./$(DEPDIR)/strlst_test-strlst-test.Po
include ./$(DEPDIR)/strlst_test-strlst.Po
include ./$(DEPDIR)/timeval_test-timeval-test.Po
include ./$(DEPDIR)/timeval_test-timeval.Po
include ./$(DEPDIR)/utf8_test-utf8-test.Po
include ./$(DEPDIR)/utf8_test-utf8.Po
include ./$(DEPDIR)/watch_test-malloc.Po
include ./$(DEPDIR)/watch_test-simple-watch.Po
include ./$(DEPDIR)/watch_test-timeval.Po
include ./$(DEPDIR)/watch_test-watch-test.Po
include ./$(DEPDIR)/watch_test_thread-malloc.Po
include ./$(DEPDIR)/watch_test_thread-simple-watch.Po
include ./$(DEPDIR)/watch_test_thread-thread-watch.Po
include ./$(DEPDIR)/watch_test_thread-timeval.Po
include ./$(DEPDIR)/watch_test_thread-watch-test.Po

.c.o:
	$(COMPILE) -MT $@ -MD -MP -MF $(DEPDIR)/$*.Tpo -c -o $@ $<
	mv -f $(DEPDIR)/$*.Tpo $(DEPDIR)/$*.Po
#	source='$<' object='$@' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(COMPILE) -c $<

libavahi_common_a-malloc.o: malloc.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-malloc.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-malloc.Tpo -c -o libavahi_common_a-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c
	mv -f $(DEPDIR)/libavahi_common_a-malloc.Tpo $(DEPDIR)/libavahi_common_la-malloc.Plo
#	source='malloc.c' object='libavahi_common_a-malloc.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c

libavahi_common_a-address.o: address.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-address.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-address.Tpo -c -o libavahi_common_a-address.o `test -f 'address.c' || echo '$(srcdir)/'`address.c
	mv -f $(DEPDIR)/libavahi_common_a-address.Tpo $(DEPDIR)/libavahi_common_la-address.Plo
#	source='address.c' object='libavahi_common_a-address.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-address.o `test -f 'address.c' || echo '$(srcdir)/'`address.c

libavahi_common_a-alternative.o: alternative.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-alternative.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-alternative.Tpo -c -o libavahi_common_a-alternative.o `test -f 'alternative.c' || echo '$(srcdir)/'`alternative.c
	mv -f $(DEPDIR)/libavahi_common_a-alternative.Tpo $(DEPDIR)/libavahi_common_la-alternative.Plo
#	source='alternative.c' object='libavahi_common_a-alternative.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-alternative.o `test -f 'alternative.c' || echo '$(srcdir)/'`alternative.c

libavahi_common_a-error.o: error.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-error.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-error.Tpo -c -o libavahi_common_a-error.o `test -f 'error.c' || echo '$(srcdir)/'`error.c
	mv -f $(DEPDIR)/libavahi_common_a-error.Tpo $(DEPDIR)/libavahi_common_la-error.Plo
#	source='error.c' object='libavahi_common_a-error.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-error.o `test -f 'error.c' || echo '$(srcdir)/'`error.c

libavahi_common_a-strlst.o: strlst.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-strlst.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-strlst.Tpo -c -o libavahi_common_a-strlst.o `test -f 'strlst.c' || echo '$(srcdir)/'`strlst.c
	mv -f $(DEPDIR)/libavahi_common_a-strlst.Tpo $(DEPDIR)/libavahi_common_la-strlst.Plo
#	source='strlst.c' object='libavahi_common_a-strlst.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-strlst.o `test -f 'strlst.c' || echo '$(srcdir)/'`strlst.c

libavahi_common_a-domain.o: domain.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-domain.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-domain.Tpo -c -o libavahi_common_a-domain.o `test -f 'domain.c' || echo '$(srcdir)/'`domain.c
	mv -f $(DEPDIR)/libavahi_common_a-domain.Tpo $(DEPDIR)/libavahi_common_la-domain.Plo
#	source='domain.c' object='libavahi_common_a-domain.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-domain.o `test -f 'domain.c' || echo '$(srcdir)/'`domain.c

libavahi_common_a-timeval.o: timeval.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-timeval.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-timeval.Tpo -c -o libavahi_common_a-timeval.o `test -f 'timeval.c' || echo '$(srcdir)/'`timeval.c
	mv -f $(DEPDIR)/libavahi_common_a-timeval.Tpo $(DEPDIR)/libavahi_common_la-timeval.Plo
#	source='timeval.c' object='libavahi_common_a-timeval.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-timeval.o `test -f 'timeval.c' || echo '$(srcdir)/'`timeval.c

libavahi_common_a-simple-watch.o: simple-watch.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-simple-watch.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-simple-watch.Tpo -c -o libavahi_common_a-simple-watch.o `test -f 'simple-watch.c' || echo '$(srcdir)/'`simple-watch.c
	mv -f $(DEPDIR)/libavahi_common_a-simple-watch.Tpo $(DEPDIR)/libavahi_common_la-simple-watch.Plo
#	source='simple-watch.c' object='libavahi_common_a-simple-watch.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-simple-watch.o `test -f 'simple-watch.c' || echo '$(srcdir)/'`simple-watch.c

libavahi_common_a-thread-watch.o: thread-watch.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-thread-watch.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-thread-watch.Tpo -c -o libavahi_common_a-thread-watch.o `test -f 'thread-watch.c' || echo '$(srcdir)/'`thread-watch.c
	mv -f $(DEPDIR)/libavahi_common_a-thread-watch.Tpo $(DEPDIR)/libavahi_common_la-thread-watch.Plo
#	source='thread-watch.c' object='libavahi_common_a-thread-watch.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-thread-watch.o `test -f 'thread-watch.c' || echo '$(srcdir)/'`thread-watch.c

libavahi_common_a-rlist.o: rlist.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-rlist.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-rlist.Tpo -c -o libavahi_common_a-rlist.o `test -f 'rlist.c' || echo '$(srcdir)/'`rlist.c
	mv -f $(DEPDIR)/libavahi_common_a-rlist.Tpo $(DEPDIR)/libavahi_common_la-rlist.Plo
#	source='rlist.c' object='libavahi_common_a-rlist.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-rlist.o `test -f 'rlist.c' || echo '$(srcdir)/'`rlist.c

libavahi_common_a-utf8.o: utf8.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-utf8.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-utf8.Tpo -c -o libavahi_common_a-utf8.o `test -f 'utf8.c' || echo '$(srcdir)/'`utf8.c
	mv -f $(DEPDIR)/libavahi_common_a-utf8.Tpo $(DEPDIR)/libavahi_common_la-utf8.Plo
#	source='utf8.c' object='libavahi_common_a-utf8.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-utf8.o `test -f 'utf8.c' || echo '$(srcdir)/'`utf8.c

libavahi_common_a-i18n.o: i18n.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -MT libavahi_common_a-i18n.o -MD -MP -MF $(DEPDIR)/libavahi_common_a-i18n.Tpo -c -o libavahi_common_a-i18n.o `test -f 'i18n.c' || echo '$(srcdir)/'`i18n.c
	mv -f $(DEPDIR)/libavahi_common_a-i18n.Tpo $(DEPDIR)/libavahi_common_la-i18n.Plo
#	source='i18n.c' object='libavahi_common_a-i18n.o' libtool=yes \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(libavahi_common_a_CFLAGS) $(CFLAGS) -c -o libavahi_common_a-i18n.o `test -f 'i18n.c' || echo '$(srcdir)/'`i18n.c

alternative_test-alternative.o: alternative.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -MT alternative_test-alternative.o -MD -MP -MF $(DEPDIR)/alternative_test-alternative.Tpo -c -o alternative_test-alternative.o `test -f 'alternative.c' || echo '$(srcdir)/'`alternative.c
	mv -f $(DEPDIR)/alternative_test-alternative.Tpo $(DEPDIR)/alternative_test-alternative.Po
#	source='alternative.c' object='alternative_test-alternative.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -c -o alternative_test-alternative.o `test -f 'alternative.c' || echo '$(srcdir)/'`alternative.c

alternative_test-malloc.o: malloc.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -MT alternative_test-malloc.o -MD -MP -MF $(DEPDIR)/alternative_test-malloc.Tpo -c -o alternative_test-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c
	mv -f $(DEPDIR)/alternative_test-malloc.Tpo $(DEPDIR)/alternative_test-malloc.Po
#	source='malloc.c' object='alternative_test-malloc.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -c -o alternative_test-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c

alternative_test-domain.o: domain.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -MT alternative_test-domain.o -MD -MP -MF $(DEPDIR)/alternative_test-domain.Tpo -c -o alternative_test-domain.o `test -f 'domain.c' || echo '$(srcdir)/'`domain.c
	mv -f $(DEPDIR)/alternative_test-domain.Tpo $(DEPDIR)/alternative_test-domain.Po
#	source='domain.c' object='alternative_test-domain.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -c -o alternative_test-domain.o `test -f 'domain.c' || echo '$(srcdir)/'`domain.c

alternative_test-address.o: address.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -MT alternative_test-address.o -MD -MP -MF $(DEPDIR)/alternative_test-address.Tpo -c -o alternative_test-address.o `test -f 'address.c' || echo '$(srcdir)/'`address.c
	mv -f $(DEPDIR)/alternative_test-address.Tpo $(DEPDIR)/alternative_test-address.Po
#	source='address.c' object='alternative_test-address.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -c -o alternative_test-address.o `test -f 'address.c' || echo '$(srcdir)/'`address.c

alternative_test-alternative-test.o: alternative-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -MT alternative_test-alternative-test.o -MD -MP -MF $(DEPDIR)/alternative_test-alternative-test.Tpo -c -o alternative_test-alternative-test.o `test -f 'alternative-test.c' || echo '$(srcdir)/'`alternative-test.c
	mv -f $(DEPDIR)/alternative_test-alternative-test.Tpo $(DEPDIR)/alternative_test-alternative-test.Po
#	source='alternative-test.c' object='alternative_test-alternative-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -c -o alternative_test-alternative-test.o `test -f 'alternative-test.c' || echo '$(srcdir)/'`alternative-test.c

alternative_test-utf8.o: utf8.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -MT alternative_test-utf8.o -MD -MP -MF $(DEPDIR)/alternative_test-utf8.Tpo -c -o alternative_test-utf8.o `test -f 'utf8.c' || echo '$(srcdir)/'`utf8.c
	mv -f $(DEPDIR)/alternative_test-utf8.Tpo $(DEPDIR)/alternative_test-utf8.Po
#	source='utf8.c' object='alternative_test-utf8.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(alternative_test_CFLAGS) $(CFLAGS) -c -o alternative_test-utf8.o `test -f 'utf8.c' || echo '$(srcdir)/'`utf8.c

domain_test-domain.o: domain.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -MT domain_test-domain.o -MD -MP -MF $(DEPDIR)/domain_test-domain.Tpo -c -o domain_test-domain.o `test -f 'domain.c' || echo '$(srcdir)/'`domain.c
	mv -f $(DEPDIR)/domain_test-domain.Tpo $(DEPDIR)/domain_test-domain.Po
#	source='domain.c' object='domain_test-domain.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -c -o domain_test-domain.o `test -f 'domain.c' || echo '$(srcdir)/'`domain.c

domain_test-malloc.o: malloc.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -MT domain_test-malloc.o -MD -MP -MF $(DEPDIR)/domain_test-malloc.Tpo -c -o domain_test-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c
	mv -f $(DEPDIR)/domain_test-malloc.Tpo $(DEPDIR)/domain_test-malloc.Po
#	source='malloc.c' object='domain_test-malloc.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -c -o domain_test-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c

domain_test-address.o: address.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -MT domain_test-address.o -MD -MP -MF $(DEPDIR)/domain_test-address.Tpo -c -o domain_test-address.o `test -f 'address.c' || echo '$(srcdir)/'`address.c
	mv -f $(DEPDIR)/domain_test-address.Tpo $(DEPDIR)/domain_test-address.Po
#	source='address.c' object='domain_test-address.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -c -o domain_test-address.o `test -f 'address.c' || echo '$(srcdir)/'`address.c

domain_test-domain-test.o: domain-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -MT domain_test-domain-test.o -MD -MP -MF $(DEPDIR)/domain_test-domain-test.Tpo -c -o domain_test-domain-test.o `test -f 'domain-test.c' || echo '$(srcdir)/'`domain-test.c
	mv -f $(DEPDIR)/domain_test-domain-test.Tpo $(DEPDIR)/domain_test-domain-test.Po
#	source='domain-test.c' object='domain_test-domain-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -c -o domain_test-domain-test.o `test -f 'domain-test.c' || echo '$(srcdir)/'`domain-test.c

domain_test-utf8.o: utf8.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -MT domain_test-utf8.o -MD -MP -MF $(DEPDIR)/domain_test-utf8.Tpo -c -o domain_test-utf8.o `test -f 'utf8.c' || echo '$(srcdir)/'`utf8.c
	mv -f $(DEPDIR)/domain_test-utf8.Tpo $(DEPDIR)/domain_test-utf8.Po
#	source='utf8.c' object='domain_test-utf8.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(domain_test_CFLAGS) $(CFLAGS) -c -o domain_test-utf8.o `test -f 'utf8.c' || echo '$(srcdir)/'`utf8.c

strlst_test-strlst.o: strlst.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(strlst_test_CFLAGS) $(CFLAGS) -MT strlst_test-strlst.o -MD -MP -MF $(DEPDIR)/strlst_test-strlst.Tpo -c -o strlst_test-strlst.o `test -f 'strlst.c' || echo '$(srcdir)/'`strlst.c
	mv -f $(DEPDIR)/strlst_test-strlst.Tpo $(DEPDIR)/strlst_test-strlst.Po
#	source='strlst.c' object='strlst_test-strlst.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(strlst_test_CFLAGS) $(CFLAGS) -c -o strlst_test-strlst.o `test -f 'strlst.c' || echo '$(srcdir)/'`strlst.c

strlst_test-malloc.o: malloc.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(strlst_test_CFLAGS) $(CFLAGS) -MT strlst_test-malloc.o -MD -MP -MF $(DEPDIR)/strlst_test-malloc.Tpo -c -o strlst_test-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c
	mv -f $(DEPDIR)/strlst_test-malloc.Tpo $(DEPDIR)/strlst_test-malloc.Po
#	source='malloc.c' object='strlst_test-malloc.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(strlst_test_CFLAGS) $(CFLAGS) -c -o strlst_test-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c

strlst_test-strlst-test.o: strlst-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(strlst_test_CFLAGS) $(CFLAGS) -MT strlst_test-strlst-test.o -MD -MP -MF $(DEPDIR)/strlst_test-strlst-test.Tpo -c -o strlst_test-strlst-test.o `test -f 'strlst-test.c' || echo '$(srcdir)/'`strlst-test.c
	mv -f $(DEPDIR)/strlst_test-strlst-test.Tpo $(DEPDIR)/strlst_test-strlst-test.Po
#	source='strlst-test.c' object='strlst_test-strlst-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(strlst_test_CFLAGS) $(CFLAGS) -c -o strlst_test-strlst-test.o `test -f 'strlst-test.c' || echo '$(srcdir)/'`strlst-test.c

timeval_test-timeval.o: timeval.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeval_test_CFLAGS) $(CFLAGS) -MT timeval_test-timeval.o -MD -MP -MF $(DEPDIR)/timeval_test-timeval.Tpo -c -o timeval_test-timeval.o `test -f 'timeval.c' || echo '$(srcdir)/'`timeval.c
	mv -f $(DEPDIR)/timeval_test-timeval.Tpo $(DEPDIR)/timeval_test-timeval.Po
#	source='timeval.c' object='timeval_test-timeval.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeval_test_CFLAGS) $(CFLAGS) -c -o timeval_test-timeval.o `test -f 'timeval.c' || echo '$(srcdir)/'`timeval.c

timeval_test-timeval-test.o: timeval-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeval_test_CFLAGS) $(CFLAGS) -MT timeval_test-timeval-test.o -MD -MP -MF $(DEPDIR)/timeval_test-timeval-test.Tpo -c -o timeval_test-timeval-test.o `test -f 'timeval-test.c' || echo '$(srcdir)/'`timeval-test.c
	mv -f $(DEPDIR)/timeval_test-timeval-test.Tpo $(DEPDIR)/timeval_test-timeval-test.Po
#	source='timeval-test.c' object='timeval_test-timeval-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(timeval_test_CFLAGS) $(CFLAGS) -c -o timeval_test-timeval-test.o `test -f 'timeval-test.c' || echo '$(srcdir)/'`timeval-test.c

utf8_test-utf8-test.o: utf8-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(utf8_test_CFLAGS) $(CFLAGS) -MT utf8_test-utf8-test.o -MD -MP -MF $(DEPDIR)/utf8_test-utf8-test.Tpo -c -o utf8_test-utf8-test.o `test -f 'utf8-test.c' || echo '$(srcdir)/'`utf8-test.c
	mv -f $(DEPDIR)/utf8_test-utf8-test.Tpo $(DEPDIR)/utf8_test-utf8-test.Po
#	source='utf8-test.c' object='utf8_test-utf8-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(utf8_test_CFLAGS) $(CFLAGS) -c -o utf8_test-utf8-test.o `test -f 'utf8-test.c' || echo '$(srcdir)/'`utf8-test.c

utf8_test-utf8.o: utf8.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(utf8_test_CFLAGS) $(CFLAGS) -MT utf8_test-utf8.o -MD -MP -MF $(DEPDIR)/utf8_test-utf8.Tpo -c -o utf8_test-utf8.o `test -f 'utf8.c' || echo '$(srcdir)/'`utf8.c
	mv -f $(DEPDIR)/utf8_test-utf8.Tpo $(DEPDIR)/utf8_test-utf8.Po
#	source='utf8.c' object='utf8_test-utf8.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(utf8_test_CFLAGS) $(CFLAGS) -c -o utf8_test-utf8.o `test -f 'utf8.c' || echo '$(srcdir)/'`utf8.c

watch_test-timeval.o: timeval.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_CFLAGS) $(CFLAGS) -MT watch_test-timeval.o -MD -MP -MF $(DEPDIR)/watch_test-timeval.Tpo -c -o watch_test-timeval.o `test -f 'timeval.c' || echo '$(srcdir)/'`timeval.c
	mv -f $(DEPDIR)/watch_test-timeval.Tpo $(DEPDIR)/watch_test-timeval.Po
#	source='timeval.c' object='watch_test-timeval.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_CFLAGS) $(CFLAGS) -c -o watch_test-timeval.o `test -f 'timeval.c' || echo '$(srcdir)/'`timeval.c

watch_test-simple-watch.o: simple-watch.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_CFLAGS) $(CFLAGS) -MT watch_test-simple-watch.o -MD -MP -MF $(DEPDIR)/watch_test-simple-watch.Tpo -c -o watch_test-simple-watch.o `test -f 'simple-watch.c' || echo '$(srcdir)/'`simple-watch.c
	mv -f $(DEPDIR)/watch_test-simple-watch.Tpo $(DEPDIR)/watch_test-simple-watch.Po
#	source='simple-watch.c' object='watch_test-simple-watch.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_CFLAGS) $(CFLAGS) -c -o watch_test-simple-watch.o `test -f 'simple-watch.c' || echo '$(srcdir)/'`simple-watch.c

watch_test-malloc.o: malloc.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_CFLAGS) $(CFLAGS) -MT watch_test-malloc.o -MD -MP -MF $(DEPDIR)/watch_test-malloc.Tpo -c -o watch_test-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c
	mv -f $(DEPDIR)/watch_test-malloc.Tpo $(DEPDIR)/watch_test-malloc.Po
#	source='malloc.c' object='watch_test-malloc.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_CFLAGS) $(CFLAGS) -c -o watch_test-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c

watch_test-watch-test.o: watch-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_CFLAGS) $(CFLAGS) -MT watch_test-watch-test.o -MD -MP -MF $(DEPDIR)/watch_test-watch-test.Tpo -c -o watch_test-watch-test.o `test -f 'watch-test.c' || echo '$(srcdir)/'`watch-test.c
	mv -f $(DEPDIR)/watch_test-watch-test.Tpo $(DEPDIR)/watch_test-watch-test.Po
#	source='watch-test.c' object='watch_test-watch-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_CFLAGS) $(CFLAGS) -c -o watch_test-watch-test.o `test -f 'watch-test.c' || echo '$(srcdir)/'`watch-test.c

watch_test_thread-timeval.o: timeval.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -MT watch_test_thread-timeval.o -MD -MP -MF $(DEPDIR)/watch_test_thread-timeval.Tpo -c -o watch_test_thread-timeval.o `test -f 'timeval.c' || echo '$(srcdir)/'`timeval.c
	mv -f $(DEPDIR)/watch_test_thread-timeval.Tpo $(DEPDIR)/watch_test_thread-timeval.Po
#	source='timeval.c' object='watch_test_thread-timeval.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -c -o watch_test_thread-timeval.o `test -f 'timeval.c' || echo '$(srcdir)/'`timeval.c

watch_test_thread-simple-watch.o: simple-watch.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -MT watch_test_thread-simple-watch.o -MD -MP -MF $(DEPDIR)/watch_test_thread-simple-watch.Tpo -c -o watch_test_thread-simple-watch.o `test -f 'simple-watch.c' || echo '$(srcdir)/'`simple-watch.c
	mv -f $(DEPDIR)/watch_test_thread-simple-watch.Tpo $(DEPDIR)/watch_test_thread-simple-watch.Po
#	source='simple-watch.c' object='watch_test_thread-simple-watch.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -c -o watch_test_thread-simple-watch.o `test -f 'simple-watch.c' || echo '$(srcdir)/'`simple-watch.c

watch_test_thread-malloc.o: malloc.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -MT watch_test_thread-malloc.o -MD -MP -MF $(DEPDIR)/watch_test_thread-malloc.Tpo -c -o watch_test_thread-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c
	mv -f $(DEPDIR)/watch_test_thread-malloc.Tpo $(DEPDIR)/watch_test_thread-malloc.Po
#	source='malloc.c' object='watch_test_thread-malloc.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -c -o watch_test_thread-malloc.o `test -f 'malloc.c' || echo '$(srcdir)/'`malloc.c

watch_test_thread-watch-test.o: watch-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -MT watch_test_thread-watch-test.o -MD -MP -MF $(DEPDIR)/watch_test_thread-watch-test.Tpo -c -o watch_test_thread-watch-test.o `test -f 'watch-test.c' || echo '$(srcdir)/'`watch-test.c
	mv -f $(DEPDIR)/watch_test_thread-watch-test.Tpo $(DEPDIR)/watch_test_thread-watch-test.Po
#	source='watch-test.c' object='watch_test_thread-watch-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -c -o watch_test_thread-watch-test.o `test -f 'watch-test.c' || echo '$(srcdir)/'`watch-test.c

watch_test_thread-thread-watch.o: thread-watch.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -MT watch_test_thread-thread-watch.o -MD -MP -MF $(DEPDIR)/watch_test_thread-thread-watch.Tpo -c -o watch_test_thread-thread-watch.o `test -f 'thread-watch.c' || echo '$(srcdir)/'`thread-watch.c
	mv -f $(DEPDIR)/watch_test_thread-thread-watch.Tpo $(DEPDIR)/watch_test_thread-thread-watch.Po
#	source='thread-watch.c' object='watch_test_thread-thread-watch.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(watch_test_thread_CFLAGS) $(CFLAGS) -c -o watch_test_thread-thread-watch.o `test -f 'thread-watch.c' || echo '$(srcdir)/'`thread-watch.c

install-avahi_commonincludeHEADERS: $(avahi_commoninclude_HEADERS)
	@$(NORMAL_INSTALL)
	test -z "$(avahi_commonincludedir)" || $(MKDIR_P) "$(DESTDIR)$(avahi_commonincludedir)"
	@list='$(avahi_commoninclude_HEADERS)'; for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(avahi_commonincludeHEADERS_INSTALL) '$$d$$p' '$(DESTDIR)$(avahi_commonincludedir)/$$f'"; \
	  $(avahi_commonincludeHEADERS_INSTALL) "$$d$$p" "$(DESTDIR)$(avahi_commonincludedir)/$$f"; \
	done

uninstall-avahi_commonincludeHEADERS:
	@$(NORMAL_UNINSTALL)
	@list='$(avahi_commoninclude_HEADERS)'; for p in $$list; do \
	  f=$(am__strip_dir) \
	  echo " rm -f '$(DESTDIR)$(avahi_commonincludedir)/$$f'"; \
	  rm -f "$(DESTDIR)$(avahi_commonincludedir)/$$f"; \
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
check: check-am
all-am: Makefile $(LTLIBRARIES) $(PROGRAMS) $(HEADERS)
installdirs:
	for dir in "$(DESTDIR)$(libdir)" "$(DESTDIR)$(avahi_commonincludedir)"; do \
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

install-data-am: install-avahi_commonincludeHEADERS

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

uninstall-am: uninstall-avahi_commonincludeHEADERS \
	uninstall-libLTLIBRARIES

.MAKE: install-am install-strip

.PHONY: CTAGS GTAGS all all-am check check-am clean clean-generic \
	clean-libLTLIBRARIES clean-noinstPROGRAMS ctags \
	distclean distclean-compile distclean-generic \
	distclean-tags distdir dvi dvi-am html \
	html-am info info-am install install-am \
	install-avahi_commonincludeHEADERS install-data \
	install-data-am install-dvi install-dvi-am install-exec \
	install-exec-am install-html install-html-am install-info \
	install-info-am install-libLTLIBRARIES install-man install-pdf \
	install-pdf-am install-ps install-ps-am install-strip \
	installcheck installcheck-am installdirs maintainer-clean \
	maintainer-clean-generic mostlyclean mostlyclean-compile \
	mostlyclean-generic pdf pdf-am ps ps-am \
	tags uninstall uninstall-am \
	uninstall-avahi_commonincludeHEADERS uninstall-libLTLIBRARIES

# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
