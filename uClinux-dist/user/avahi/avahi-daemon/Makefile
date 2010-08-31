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
am__append_1 = \
	-DAVAHI_DAEMON_RUNTIME_DIR=\"$(avahi_runtime_dir)/avahi-daemon/\" \
	-DAVAHI_SOCKET=\"$(avahi_socket)\" \
	-DAVAHI_SERVICE_DIR=\"$(servicedir)\" \
	-DAVAHI_CONFIG_FILE=\"$(pkgsysconfdir)/avahi-daemon.conf\" \
	-DAVAHI_HOSTS_FILE=\"$(pkgsysconfdir)/hosts\" \
	-DAVAHI_DBUS_INTROSPECTION_DIR=\"$(introspectiondir)\" \
	-DAVAHI_CONFIG_DIR=\"$(pkgsysconfdir)\"

sbin_PROGRAMS =  \
	avahi-daemon
#noinst_PROGRAMS = ini-file-parser-test
am__append_2 = \
	chroot.c chroot.h \
	caps.c caps.h

am__append_3 = -lcap
am__append_4 = 
subdir = avahi-daemon
mkinstalldirs = $(install_sh) -d
CONFIG_HEADER = $(top_builddir)/config.h
CONFIG_CLEAN_FILES = avahi-dbus.conf
am__installdirs = "$(DESTDIR)$(sbindir)" "$(DESTDIR)$(dbusservicedir)" \
	"$(DESTDIR)$(introspectiondir)" "$(DESTDIR)$(pkgdatadir)" \
	"$(DESTDIR)$(pkgsysconfdir)" "$(DESTDIR)$(servicedir)"
sbinPROGRAMS_INSTALL = $(INSTALL_PROGRAM)
PROGRAMS = $(noinst_PROGRAMS) $(sbin_PROGRAMS)
am__avahi_daemon_SOURCES_DIST = main.c main.h simple-protocol.c \
	simple-protocol.h static-services.c static-services.h \
	static-hosts.c static-hosts.h ini-file-parser.c \
	ini-file-parser.h setproctitle.c setproctitle.h \
	../avahi-client/check-nss.c chroot.c chroot.h caps.c caps.h \
	dbus-protocol.c dbus-protocol.h dbus-util.c dbus-util.h \
	dbus-internal.h dbus-async-address-resolver.c \
	dbus-async-host-name-resolver.c dbus-async-service-resolver.c \
	dbus-domain-browser.c dbus-entry-group.c \
	dbus-service-browser.c dbus-service-type-browser.c \
	dbus-sync-address-resolver.c dbus-sync-host-name-resolver.c \
	dbus-sync-service-resolver.c dbus-record-browser.c \
	../avahi-common/dbus.c ../avahi-common/dbus.h \
	../avahi-common/dbus-watch-glue.c \
	../avahi-common/dbus-watch-glue.h
am__objects_1 = avahi_daemon-chroot.o \
	avahi_daemon-caps.o
am_avahi_daemon_OBJECTS = avahi_daemon-main.o \
	avahi_daemon-simple-protocol.o \
	avahi_daemon-static-services.o \
	avahi_daemon-static-hosts.o \
	avahi_daemon-ini-file-parser.o \
	avahi_daemon-setproctitle.o \
	avahi_daemon-check-nss.o \
	$(am__objects_1) \
	$(am__objects_2)
avahi_daemon_OBJECTS = $(am_avahi_daemon_OBJECTS)
am__DEPENDENCIES_1 =
#am__DEPENDENCIES_2 = $(am__DEPENDENCIES_1)
avahi_daemon_DEPENDENCIES = ../avahi-common/libavahi-common.a \
	../avahi-core/libavahi-core.a \
	$(am__DEPENDENCIES_1) \
	$(am__DEPENDENCIES_1) \
	$(am__DEPENDENCIES_1) \
	$(am__DEPENDENCIES_1) \
	$(am__DEPENDENCIES_2)
avahi_daemon_LINK = $(LD) $(avahi_daemon_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
am__ini_file_parser_test_SOURCES_DIST = ini-file-parser.c \
	ini-file-parser.h ini-file-parser-test.c
am_ini_file_parser_test_OBJECTS = ini_file_parser_test-ini-file-parser.o \
	ini_file_parser_test-ini-file-parser-test.o
ini_file_parser_test_OBJECTS = $(am_ini_file_parser_test_OBJECTS)
ini_file_parser_test_DEPENDENCIES = ../avahi-common/libavahi-common.a \
	../avahi-core/libavahi-core.a
ini_file_parser_test_LINK = $(LD) \
	$(ini_file_parser_test_CFLAGS) $(CFLAGS) $(AM_LDFLAGS) \
	$(LDFLAGS) -o $@
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
SOURCES = $(avahi_daemon_SOURCES) $(ini_file_parser_test_SOURCES)
DIST_SOURCES = $(am__avahi_daemon_SOURCES_DIST) \
	$(am__ini_file_parser_test_SOURCES_DIST)
am__vpath_adj_setup = srcdirstrip=`echo "$(srcdir)" | sed 's|.|.|g'`;
am__vpath_adj = case $$p in \
    $(srcdir)/*) f=`echo "$$p" | sed "s|^$$srcdirstrip/||"`;; \
    *) f=$$p;; \
  esac;
am__strip_dir = `echo $$p | sed -e 's|^.*/||'`;
dbusserviceDATA_INSTALL = $(INSTALL_DATA)
introspectionDATA_INSTALL = $(INSTALL_DATA)
pkgdataDATA_INSTALL = $(INSTALL_DATA)
pkgsysconfDATA_INSTALL = $(INSTALL_DATA)
serviceDATA_INSTALL = $(INSTALL_DATA)
DATA = $(dbusservice_DATA) $(introspection_DATA) $(pkgdata_DATA) \
	$(pkgsysconf_DATA) $(service_DATA)
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
LIBDAEMON_CFLAGS = -I $(top_builddir)/../../include/
LIBDAEMON_LIBS = $(top_builddir)/../../lib/libdaemon.a
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
XML_CFLAGS = -I $(top_builddir)/../../include/ -DUSE_EXPAT_H
XML_LIBS = $(top_builddir)/../../lib/libexpat.a
abs_builddir = $(ROOTDIR)/user/avahi/avahi-daemon
abs_srcdir = $(ROOTDIR)/user/avahi/avahi-daemon
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
AM_CFLAGS = -I$(top_srcdir) '-DDEBUG_TRAP=__asm__("int $$3")' \
	$(am__append_1)
pkgsysconfdir = $(sysconfdir)/avahi
servicedir = $(pkgsysconfdir)/services
introspectiondir = $(pkgdatadir)/introspection
avahi_daemon_SOURCES = main.c \
	main.h simple-protocol.c \
	simple-protocol.h \
	static-services.c \
	static-services.h \
	static-hosts.c \
	static-hosts.h \
	ini-file-parser.c \
	ini-file-parser.h \
	setproctitle.c \
	setproctitle.h \
	../avahi-client/check-nss.c \
	$(am__append_2) \
	$(am__append_5)
avahi_daemon_CFLAGS =  \
	$(AM_CFLAGS) \
	$(LIBDAEMON_CFLAGS) \
	$(XML_CFLAGS) \
	$(am__append_7)
avahi_daemon_LDADD = $(AM_LDADD) \
	-Wl,-elf2flt \
	../avahi-core/libavahi-core.a \
	../avahi-common/libavahi-common.a \
	$(LIBDAEMON_LIBS) \
	$(XML_LIBS) \
	$(PTHREAD_LIBS) \
	$(am__append_3) \
	$(am__append_4) \
	$(am__append_6)
ini_file_parser_test_SOURCES = \
	ini-file-parser.c ini-file-parser.h \
	ini-file-parser-test.c

ini_file_parser_test_CFLAGS = $(AM_CFLAGS)
ini_file_parser_test_LDADD = $(AM_LDADD) ../avahi-common/libavahi-common.a ../avahi-core/libavahi-core.a 
pkgsysconf_DATA = \
	avahi-daemon.conf \
	hosts

service_DATA = \
	ssh.service \
	sftp-ssh.service

pkgdata_DATA = \
	avahi-service.dtd

EXTRA_DIST = \
	avahi-service.dtd \
	avahi-daemon.conf \
	example.service \
	avahi-dbus.conf \
	Server.introspect \
	EntryGroup.introspect \
	DomainBrowser.introspect \
	ServiceTypeBrowser.introspect \
	ServiceBrowser.introspect \
	ServiceResolver.introspect \
	AddressResolver.introspect \
	HostNameResolver.introspect \
	RecordBrowser.introspect \
	ssh.service \
	sftp-ssh.service \
	hosts \
	example.service \
	introspect.dtd \
	introspect.xsl

all: all-am

.SUFFIXES:
.SUFFIXES: .c .o
avahi-dbus.conf: $(top_builddir)/config.status $(srcdir)/avahi-dbus.conf.in
	cd $(top_builddir) && $(SHELL) ./config.status $(subdir)/$@

clean-noinstPROGRAMS:
	@list='$(noinst_PROGRAMS)'; for p in $$list; do \
	  echo " rm -f $$p $$p.gdb"; \
	  rm -f $$p $$p.gdb ; \
	done
install-sbinPROGRAMS: $(sbin_PROGRAMS)
	@$(NORMAL_INSTALL)
	test -z "$(sbindir)" || $(MKDIR_P) "$(DESTDIR)$(sbindir)"
	@list='$(sbin_PROGRAMS)'; for p in $$list; do \
	   echo " $(INSTALL_PROGRAM_ENV) $(INSTALL) '$$p' '$(DESTDIR)$(sbindir)/$$p'"; \
	   $(INSTALL_PROGRAM_ENV) $(INSTALL) "$$p" "$(DESTDIR)$(sbindir)/$$p" || exit 1; \
	  else :; fi; \
	done

uninstall-sbinPROGRAMS:
	@$(NORMAL_UNINSTALL)
	@list='$(sbin_PROGRAMS)'; for p in $$list; do \
	  echo " rm -f '$(DESTDIR)$(sbindir)/$$p'"; \
	  rm -f "$(DESTDIR)$(sbindir)/$$p"; \
	done

clean-sbinPROGRAMS:
	@list='$(sbin_PROGRAMS)'; for p in $$list; do \
	  echo " rm -f $$p $$p.gdb"; \
	  rm -f $$p $$p.gdb ; \
	done
avahi-daemon: $(avahi_daemon_OBJECTS) $(avahi_daemon_DEPENDENCIES) 
	@rm -f avahi-daemon avahi-daemon.gdb
	$(avahi_daemon_LINK) $(avahi_daemon_OBJECTS) $(avahi_daemon_LDADD) $(LIBS)
ini-file-parser-test: $(ini_file_parser_test_OBJECTS) $(ini_file_parser_test_DEPENDENCIES) 
	@rm -f ini-file-parser-test ini-file-parser-test.gdb
	$(ini_file_parser_test_LINK) $(ini_file_parser_test_OBJECTS) $(ini_file_parser_test_LDADD) $(LIBS)

mostlyclean-compile:
	-rm -f *.o

distclean-compile:
	-rm -f *.tab.c

-include ./$(DEPDIR)/avahi_daemon-caps.Po
-include ./$(DEPDIR)/avahi_daemon-check-nss.Po
-include ./$(DEPDIR)/avahi_daemon-chroot.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-async-address-resolver.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-async-host-name-resolver.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-async-service-resolver.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-domain-browser.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-entry-group.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-protocol.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-record-browser.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-service-browser.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-service-type-browser.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-sync-address-resolver.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-sync-host-name-resolver.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-sync-service-resolver.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-util.Po
-include ./$(DEPDIR)/avahi_daemon-dbus-watch-glue.Po
-include ./$(DEPDIR)/avahi_daemon-dbus.Po
-include ./$(DEPDIR)/avahi_daemon-ini-file-parser.Po
-include ./$(DEPDIR)/avahi_daemon-main.Po
-include ./$(DEPDIR)/avahi_daemon-setproctitle.Po
-include ./$(DEPDIR)/avahi_daemon-simple-protocol.Po
-include ./$(DEPDIR)/avahi_daemon-static-hosts.Po
-include ./$(DEPDIR)/avahi_daemon-static-services.Po
-include ./$(DEPDIR)/ini_file_parser_test-ini-file-parser-test.Po
-include ./$(DEPDIR)/ini_file_parser_test-ini-file-parser.Po

.c.o:
	$(COMPILE) -MT $@ -MD -MP -MF $(DEPDIR)/$*.Tpo -c -o $@ $<
	mv -f $(DEPDIR)/$*.Tpo $(DEPDIR)/$*.Po
#	source='$<' object='$@' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(COMPILE) -c $<

avahi_daemon-main.o: main.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-main.o -MD -MP -MF $(DEPDIR)/avahi_daemon-main.Tpo -c -o avahi_daemon-main.o `test -f 'main.c' || echo '$(srcdir)/'`main.c
	mv -f $(DEPDIR)/avahi_daemon-main.Tpo $(DEPDIR)/avahi_daemon-main.Po
#	source='main.c' object='avahi_daemon-main.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-main.o `test -f 'main.c' || echo '$(srcdir)/'`main.c

avahi_daemon-simple-protocol.o: simple-protocol.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-simple-protocol.o -MD -MP -MF $(DEPDIR)/avahi_daemon-simple-protocol.Tpo -c -o avahi_daemon-simple-protocol.o `test -f 'simple-protocol.c' || echo '$(srcdir)/'`simple-protocol.c
	mv -f $(DEPDIR)/avahi_daemon-simple-protocol.Tpo $(DEPDIR)/avahi_daemon-simple-protocol.Po
#	source='simple-protocol.c' object='avahi_daemon-simple-protocol.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-simple-protocol.o `test -f 'simple-protocol.c' || echo '$(srcdir)/'`simple-protocol.c

avahi_daemon-static-services.o: static-services.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-static-services.o -MD -MP -MF $(DEPDIR)/avahi_daemon-static-services.Tpo -c -o avahi_daemon-static-services.o `test -f 'static-services.c' || echo '$(srcdir)/'`static-services.c
	mv -f $(DEPDIR)/avahi_daemon-static-services.Tpo $(DEPDIR)/avahi_daemon-static-services.Po
#	source='static-services.c' object='avahi_daemon-static-services.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-static-services.o `test -f 'static-services.c' || echo '$(srcdir)/'`static-services.c

avahi_daemon-static-hosts.o: static-hosts.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-static-hosts.o -MD -MP -MF $(DEPDIR)/avahi_daemon-static-hosts.Tpo -c -o avahi_daemon-static-hosts.o `test -f 'static-hosts.c' || echo '$(srcdir)/'`static-hosts.c
	mv -f $(DEPDIR)/avahi_daemon-static-hosts.Tpo $(DEPDIR)/avahi_daemon-static-hosts.Po
#	source='static-hosts.c' object='avahi_daemon-static-hosts.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-static-hosts.o `test -f 'static-hosts.c' || echo '$(srcdir)/'`static-hosts.c

avahi_daemon-ini-file-parser.o: ini-file-parser.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-ini-file-parser.o -MD -MP -MF $(DEPDIR)/avahi_daemon-ini-file-parser.Tpo -c -o avahi_daemon-ini-file-parser.o `test -f 'ini-file-parser.c' || echo '$(srcdir)/'`ini-file-parser.c
	mv -f $(DEPDIR)/avahi_daemon-ini-file-parser.Tpo $(DEPDIR)/avahi_daemon-ini-file-parser.Po
#	source='ini-file-parser.c' object='avahi_daemon-ini-file-parser.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-ini-file-parser.o `test -f 'ini-file-parser.c' || echo '$(srcdir)/'`ini-file-parser.c

avahi_daemon-setproctitle.o: setproctitle.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-setproctitle.o -MD -MP -MF $(DEPDIR)/avahi_daemon-setproctitle.Tpo -c -o avahi_daemon-setproctitle.o `test -f 'setproctitle.c' || echo '$(srcdir)/'`setproctitle.c
	mv -f $(DEPDIR)/avahi_daemon-setproctitle.Tpo $(DEPDIR)/avahi_daemon-setproctitle.Po
#	source='setproctitle.c' object='avahi_daemon-setproctitle.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-setproctitle.o `test -f 'setproctitle.c' || echo '$(srcdir)/'`setproctitle.c

avahi_daemon-check-nss.o: ../avahi-client/check-nss.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-check-nss.o -MD -MP -MF $(DEPDIR)/avahi_daemon-check-nss.Tpo -c -o avahi_daemon-check-nss.o `test -f '../avahi-client/check-nss.c' || echo '$(srcdir)/'`../avahi-client/check-nss.c
	mv -f $(DEPDIR)/avahi_daemon-check-nss.Tpo $(DEPDIR)/avahi_daemon-check-nss.Po
#	source='../avahi-client/check-nss.c' object='avahi_daemon-check-nss.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-check-nss.o `test -f '../avahi-client/check-nss.c' || echo '$(srcdir)/'`../avahi-client/check-nss.c

avahi_daemon-chroot.o: chroot.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-chroot.o -MD -MP -MF $(DEPDIR)/avahi_daemon-chroot.Tpo -c -o avahi_daemon-chroot.o `test -f 'chroot.c' || echo '$(srcdir)/'`chroot.c
	mv -f $(DEPDIR)/avahi_daemon-chroot.Tpo $(DEPDIR)/avahi_daemon-chroot.Po
#	source='chroot.c' object='avahi_daemon-chroot.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-chroot.o `test -f 'chroot.c' || echo '$(srcdir)/'`chroot.c

avahi_daemon-caps.o: caps.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-caps.o -MD -MP -MF $(DEPDIR)/avahi_daemon-caps.Tpo -c -o avahi_daemon-caps.o `test -f 'caps.c' || echo '$(srcdir)/'`caps.c
	mv -f $(DEPDIR)/avahi_daemon-caps.Tpo $(DEPDIR)/avahi_daemon-caps.Po
#	source='caps.c' object='avahi_daemon-caps.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-caps.o `test -f 'caps.c' || echo '$(srcdir)/'`caps.c

avahi_daemon-dbus-protocol.o: dbus-protocol.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-protocol.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-protocol.Tpo -c -o avahi_daemon-dbus-protocol.o `test -f 'dbus-protocol.c' || echo '$(srcdir)/'`dbus-protocol.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-protocol.Tpo $(DEPDIR)/avahi_daemon-dbus-protocol.Po
#	source='dbus-protocol.c' object='avahi_daemon-dbus-protocol.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-protocol.o `test -f 'dbus-protocol.c' || echo '$(srcdir)/'`dbus-protocol.c

avahi_daemon-dbus-util.o: dbus-util.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-util.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-util.Tpo -c -o avahi_daemon-dbus-util.o `test -f 'dbus-util.c' || echo '$(srcdir)/'`dbus-util.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-util.Tpo $(DEPDIR)/avahi_daemon-dbus-util.Po
#	source='dbus-util.c' object='avahi_daemon-dbus-util.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-util.o `test -f 'dbus-util.c' || echo '$(srcdir)/'`dbus-util.c

avahi_daemon-dbus-async-address-resolver.o: dbus-async-address-resolver.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-async-address-resolver.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-async-address-resolver.Tpo -c -o avahi_daemon-dbus-async-address-resolver.o `test -f 'dbus-async-address-resolver.c' || echo '$(srcdir)/'`dbus-async-address-resolver.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-async-address-resolver.Tpo $(DEPDIR)/avahi_daemon-dbus-async-address-resolver.Po
#	source='dbus-async-address-resolver.c' object='avahi_daemon-dbus-async-address-resolver.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-async-address-resolver.o `test -f 'dbus-async-address-resolver.c' || echo '$(srcdir)/'`dbus-async-address-resolver.c

avahi_daemon-dbus-async-host-name-resolver.o: dbus-async-host-name-resolver.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-async-host-name-resolver.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-async-host-name-resolver.Tpo -c -o avahi_daemon-dbus-async-host-name-resolver.o `test -f 'dbus-async-host-name-resolver.c' || echo '$(srcdir)/'`dbus-async-host-name-resolver.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-async-host-name-resolver.Tpo $(DEPDIR)/avahi_daemon-dbus-async-host-name-resolver.Po
#	source='dbus-async-host-name-resolver.c' object='avahi_daemon-dbus-async-host-name-resolver.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-async-host-name-resolver.o `test -f 'dbus-async-host-name-resolver.c' || echo '$(srcdir)/'`dbus-async-host-name-resolver.c

avahi_daemon-dbus-async-service-resolver.o: dbus-async-service-resolver.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-async-service-resolver.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-async-service-resolver.Tpo -c -o avahi_daemon-dbus-async-service-resolver.o `test -f 'dbus-async-service-resolver.c' || echo '$(srcdir)/'`dbus-async-service-resolver.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-async-service-resolver.Tpo $(DEPDIR)/avahi_daemon-dbus-async-service-resolver.Po
#	source='dbus-async-service-resolver.c' object='avahi_daemon-dbus-async-service-resolver.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-async-service-resolver.o `test -f 'dbus-async-service-resolver.c' || echo '$(srcdir)/'`dbus-async-service-resolver.c

avahi_daemon-dbus-domain-browser.o: dbus-domain-browser.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-domain-browser.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-domain-browser.Tpo -c -o avahi_daemon-dbus-domain-browser.o `test -f 'dbus-domain-browser.c' || echo '$(srcdir)/'`dbus-domain-browser.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-domain-browser.Tpo $(DEPDIR)/avahi_daemon-dbus-domain-browser.Po
#	source='dbus-domain-browser.c' object='avahi_daemon-dbus-domain-browser.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-domain-browser.o `test -f 'dbus-domain-browser.c' || echo '$(srcdir)/'`dbus-domain-browser.c

avahi_daemon-dbus-entry-group.o: dbus-entry-group.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-entry-group.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-entry-group.Tpo -c -o avahi_daemon-dbus-entry-group.o `test -f 'dbus-entry-group.c' || echo '$(srcdir)/'`dbus-entry-group.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-entry-group.Tpo $(DEPDIR)/avahi_daemon-dbus-entry-group.Po
#	source='dbus-entry-group.c' object='avahi_daemon-dbus-entry-group.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-entry-group.o `test -f 'dbus-entry-group.c' || echo '$(srcdir)/'`dbus-entry-group.c

avahi_daemon-dbus-service-browser.o: dbus-service-browser.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-service-browser.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-service-browser.Tpo -c -o avahi_daemon-dbus-service-browser.o `test -f 'dbus-service-browser.c' || echo '$(srcdir)/'`dbus-service-browser.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-service-browser.Tpo $(DEPDIR)/avahi_daemon-dbus-service-browser.Po
#	source='dbus-service-browser.c' object='avahi_daemon-dbus-service-browser.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-service-browser.o `test -f 'dbus-service-browser.c' || echo '$(srcdir)/'`dbus-service-browser.c

avahi_daemon-dbus-service-type-browser.o: dbus-service-type-browser.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-service-type-browser.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-service-type-browser.Tpo -c -o avahi_daemon-dbus-service-type-browser.o `test -f 'dbus-service-type-browser.c' || echo '$(srcdir)/'`dbus-service-type-browser.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-service-type-browser.Tpo $(DEPDIR)/avahi_daemon-dbus-service-type-browser.Po
#	source='dbus-service-type-browser.c' object='avahi_daemon-dbus-service-type-browser.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-service-type-browser.o `test -f 'dbus-service-type-browser.c' || echo '$(srcdir)/'`dbus-service-type-browser.c

avahi_daemon-dbus-sync-address-resolver.o: dbus-sync-address-resolver.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-sync-address-resolver.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-sync-address-resolver.Tpo -c -o avahi_daemon-dbus-sync-address-resolver.o `test -f 'dbus-sync-address-resolver.c' || echo '$(srcdir)/'`dbus-sync-address-resolver.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-sync-address-resolver.Tpo $(DEPDIR)/avahi_daemon-dbus-sync-address-resolver.Po
#	source='dbus-sync-address-resolver.c' object='avahi_daemon-dbus-sync-address-resolver.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-sync-address-resolver.o `test -f 'dbus-sync-address-resolver.c' || echo '$(srcdir)/'`dbus-sync-address-resolver.c

avahi_daemon-dbus-sync-host-name-resolver.o: dbus-sync-host-name-resolver.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-sync-host-name-resolver.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-sync-host-name-resolver.Tpo -c -o avahi_daemon-dbus-sync-host-name-resolver.o `test -f 'dbus-sync-host-name-resolver.c' || echo '$(srcdir)/'`dbus-sync-host-name-resolver.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-sync-host-name-resolver.Tpo $(DEPDIR)/avahi_daemon-dbus-sync-host-name-resolver.Po
#	source='dbus-sync-host-name-resolver.c' object='avahi_daemon-dbus-sync-host-name-resolver.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-sync-host-name-resolver.o `test -f 'dbus-sync-host-name-resolver.c' || echo '$(srcdir)/'`dbus-sync-host-name-resolver.c

avahi_daemon-dbus-sync-service-resolver.o: dbus-sync-service-resolver.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-sync-service-resolver.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-sync-service-resolver.Tpo -c -o avahi_daemon-dbus-sync-service-resolver.o `test -f 'dbus-sync-service-resolver.c' || echo '$(srcdir)/'`dbus-sync-service-resolver.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-sync-service-resolver.Tpo $(DEPDIR)/avahi_daemon-dbus-sync-service-resolver.Po
#	source='dbus-sync-service-resolver.c' object='avahi_daemon-dbus-sync-service-resolver.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-sync-service-resolver.o `test -f 'dbus-sync-service-resolver.c' || echo '$(srcdir)/'`dbus-sync-service-resolver.c

avahi_daemon-dbus-record-browser.o: dbus-record-browser.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-record-browser.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-record-browser.Tpo -c -o avahi_daemon-dbus-record-browser.o `test -f 'dbus-record-browser.c' || echo '$(srcdir)/'`dbus-record-browser.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-record-browser.Tpo $(DEPDIR)/avahi_daemon-dbus-record-browser.Po
#	source='dbus-record-browser.c' object='avahi_daemon-dbus-record-browser.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-record-browser.o `test -f 'dbus-record-browser.c' || echo '$(srcdir)/'`dbus-record-browser.c

avahi_daemon-dbus.o: ../avahi-common/dbus.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus.Tpo -c -o avahi_daemon-dbus.o `test -f '../avahi-common/dbus.c' || echo '$(srcdir)/'`../avahi-common/dbus.c
	mv -f $(DEPDIR)/avahi_daemon-dbus.Tpo $(DEPDIR)/avahi_daemon-dbus.Po
#	source='../avahi-common/dbus.c' object='avahi_daemon-dbus.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus.o `test -f '../avahi-common/dbus.c' || echo '$(srcdir)/'`../avahi-common/dbus.c

avahi_daemon-dbus-watch-glue.o: ../avahi-common/dbus-watch-glue.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -MT avahi_daemon-dbus-watch-glue.o -MD -MP -MF $(DEPDIR)/avahi_daemon-dbus-watch-glue.Tpo -c -o avahi_daemon-dbus-watch-glue.o `test -f '../avahi-common/dbus-watch-glue.c' || echo '$(srcdir)/'`../avahi-common/dbus-watch-glue.c
	mv -f $(DEPDIR)/avahi_daemon-dbus-watch-glue.Tpo $(DEPDIR)/avahi_daemon-dbus-watch-glue.Po
#	source='../avahi-common/dbus-watch-glue.c' object='avahi_daemon-dbus-watch-glue.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_daemon_CFLAGS) $(CFLAGS) -c -o avahi_daemon-dbus-watch-glue.o `test -f '../avahi-common/dbus-watch-glue.c' || echo '$(srcdir)/'`../avahi-common/dbus-watch-glue.c

ini_file_parser_test-ini-file-parser.o: ini-file-parser.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(ini_file_parser_test_CFLAGS) $(CFLAGS) -MT ini_file_parser_test-ini-file-parser.o -MD -MP -MF $(DEPDIR)/ini_file_parser_test-ini-file-parser.Tpo -c -o ini_file_parser_test-ini-file-parser.o `test -f 'ini-file-parser.c' || echo '$(srcdir)/'`ini-file-parser.c
	mv -f $(DEPDIR)/ini_file_parser_test-ini-file-parser.Tpo $(DEPDIR)/ini_file_parser_test-ini-file-parser.Po
#	source='ini-file-parser.c' object='ini_file_parser_test-ini-file-parser.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(ini_file_parser_test_CFLAGS) $(CFLAGS) -c -o ini_file_parser_test-ini-file-parser.o `test -f 'ini-file-parser.c' || echo '$(srcdir)/'`ini-file-parser.c

ini_file_parser_test-ini-file-parser-test.o: ini-file-parser-test.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(ini_file_parser_test_CFLAGS) $(CFLAGS) -MT ini_file_parser_test-ini-file-parser-test.o -MD -MP -MF $(DEPDIR)/ini_file_parser_test-ini-file-parser-test.Tpo -c -o ini_file_parser_test-ini-file-parser-test.o `test -f 'ini-file-parser-test.c' || echo '$(srcdir)/'`ini-file-parser-test.c
	mv -f $(DEPDIR)/ini_file_parser_test-ini-file-parser-test.Tpo $(DEPDIR)/ini_file_parser_test-ini-file-parser-test.Po
#	source='ini-file-parser-test.c' object='ini_file_parser_test-ini-file-parser-test.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(ini_file_parser_test_CFLAGS) $(CFLAGS) -c -o ini_file_parser_test-ini-file-parser-test.o `test -f 'ini-file-parser-test.c' || echo '$(srcdir)/'`ini-file-parser-test.c

install-dbusserviceDATA: $(dbusservice_DATA)
	@$(NORMAL_INSTALL)
	test -z "$(dbusservicedir)" || $(MKDIR_P) "$(DESTDIR)$(dbusservicedir)"
	@list='$(dbusservice_DATA)'; for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(dbusserviceDATA_INSTALL) '$$d$$p' '$(DESTDIR)$(dbusservicedir)/$$f'"; \
	  $(dbusserviceDATA_INSTALL) "$$d$$p" "$(DESTDIR)$(dbusservicedir)/$$f"; \
	done

uninstall-dbusserviceDATA:
	@$(NORMAL_UNINSTALL)
	@list='$(dbusservice_DATA)'; for p in $$list; do \
	  f=$(am__strip_dir) \
	  echo " rm -f '$(DESTDIR)$(dbusservicedir)/$$f'"; \
	  rm -f "$(DESTDIR)$(dbusservicedir)/$$f"; \
	done
install-introspectionDATA: $(introspection_DATA)
	@$(NORMAL_INSTALL)
	test -z "$(introspectiondir)" || $(MKDIR_P) "$(DESTDIR)$(introspectiondir)"
	@list='$(introspection_DATA)'; for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(introspectionDATA_INSTALL) '$$d$$p' '$(DESTDIR)$(introspectiondir)/$$f'"; \
	  $(introspectionDATA_INSTALL) "$$d$$p" "$(DESTDIR)$(introspectiondir)/$$f"; \
	done

uninstall-introspectionDATA:
	@$(NORMAL_UNINSTALL)
	@list='$(introspection_DATA)'; for p in $$list; do \
	  f=$(am__strip_dir) \
	  echo " rm -f '$(DESTDIR)$(introspectiondir)/$$f'"; \
	  rm -f "$(DESTDIR)$(introspectiondir)/$$f"; \
	done
install-pkgdataDATA: $(pkgdata_DATA)
	@$(NORMAL_INSTALL)
	test -z "$(pkgdatadir)" || $(MKDIR_P) "$(DESTDIR)$(pkgdatadir)"
	@list='$(pkgdata_DATA)'; for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(pkgdataDATA_INSTALL) '$$d$$p' '$(DESTDIR)$(pkgdatadir)/$$f'"; \
	  $(pkgdataDATA_INSTALL) "$$d$$p" "$(DESTDIR)$(pkgdatadir)/$$f"; \
	done

uninstall-pkgdataDATA:
	@$(NORMAL_UNINSTALL)
	@list='$(pkgdata_DATA)'; for p in $$list; do \
	  f=$(am__strip_dir) \
	  echo " rm -f '$(DESTDIR)$(pkgdatadir)/$$f'"; \
	  rm -f "$(DESTDIR)$(pkgdatadir)/$$f"; \
	done
install-pkgsysconfDATA: $(pkgsysconf_DATA)
	@$(NORMAL_INSTALL)
	test -z "$(pkgsysconfdir)" || $(MKDIR_P) "$(DESTDIR)$(pkgsysconfdir)"
	@list='$(pkgsysconf_DATA)'; for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(pkgsysconfDATA_INSTALL) '$$d$$p' '$(DESTDIR)$(pkgsysconfdir)/$$f'"; \
	  $(pkgsysconfDATA_INSTALL) "$$d$$p" "$(DESTDIR)$(pkgsysconfdir)/$$f"; \
	done

uninstall-pkgsysconfDATA:
	@$(NORMAL_UNINSTALL)
	@list='$(pkgsysconf_DATA)'; for p in $$list; do \
	  f=$(am__strip_dir) \
	  echo " rm -f '$(DESTDIR)$(pkgsysconfdir)/$$f'"; \
	  rm -f "$(DESTDIR)$(pkgsysconfdir)/$$f"; \
	done
install-serviceDATA: $(service_DATA)
	@$(NORMAL_INSTALL)
	test -z "$(servicedir)" || $(MKDIR_P) "$(DESTDIR)$(servicedir)"
	@list='$(service_DATA)'; for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(serviceDATA_INSTALL) '$$d$$p' '$(DESTDIR)$(servicedir)/$$f'"; \
	  $(serviceDATA_INSTALL) "$$d$$p" "$(DESTDIR)$(servicedir)/$$f"; \
	done

uninstall-serviceDATA:
	@$(NORMAL_UNINSTALL)
	@list='$(service_DATA)'; for p in $$list; do \
	  f=$(am__strip_dir) \
	  echo " rm -f '$(DESTDIR)$(servicedir)/$$f'"; \
	  rm -f "$(DESTDIR)$(servicedir)/$$f"; \
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
	here=`cd $(top_builddir) && pwd` \
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
all-am: Makefile $(PROGRAMS) $(DATA)
installdirs:
	for dir in "$(DESTDIR)$(sbindir)" "$(DESTDIR)$(dbusservicedir)" "$(DESTDIR)$(introspectiondir)" "$(DESTDIR)$(pkgdatadir)" "$(DESTDIR)$(pkgsysconfdir)" "$(DESTDIR)$(servicedir)"; do \
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

clean-am: clean-generic clean-noinstPROGRAMS \
	clean-sbinPROGRAMS mostlyclean-am

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

install-data-am: install-data-local install-dbusserviceDATA \
	install-introspectionDATA install-pkgdataDATA \
	install-pkgsysconfDATA install-serviceDATA

install-dvi: install-dvi-am

install-exec-am: install-sbinPROGRAMS

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

uninstall-am: uninstall-dbusserviceDATA uninstall-introspectionDATA \
	uninstall-pkgdataDATA uninstall-pkgsysconfDATA \
	uninstall-sbinPROGRAMS uninstall-serviceDATA

.MAKE: install-am install-strip

.PHONY: CTAGS GTAGS all all-am check check-am clean clean-generic \
	clean-noinstPROGRAMS clean-sbinPROGRAMS ctags \
	distclean distclean-compile distclean-generic \
	distclean-tags distdir dvi dvi-am html \
	html-am info info-am install install-am install-data \
	install-data-am install-data-local install-dbusserviceDATA \
	install-dvi install-dvi-am install-exec install-exec-am \
	install-html install-html-am install-info install-info-am \
	install-introspectionDATA install-man install-pdf \
	install-pdf-am install-pkgdataDATA install-pkgsysconfDATA \
	install-ps install-ps-am install-sbinPROGRAMS \
	install-serviceDATA install-strip installcheck installcheck-am \
	installdirs maintainer-clean maintainer-clean-generic \
	mostlyclean mostlyclean-compile mostlyclean-generic \
	pdf pdf-am ps ps-am tags uninstall \
	uninstall-am uninstall-dbusserviceDATA \
	uninstall-introspectionDATA uninstall-pkgdataDATA \
	uninstall-pkgsysconfDATA uninstall-sbinPROGRAMS \
	uninstall-serviceDATA


xmllint:
	xmllint --noout --valid example.service
	for F in $(introspection_DATA) ; do \
		xmllint --noout --valid $$F ; \
	done

install-data-local:
	test -z "$(localstatedir)/run" || $(mkdir_p) "$(DESTDIR)$(localstatedir)/run"
# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
