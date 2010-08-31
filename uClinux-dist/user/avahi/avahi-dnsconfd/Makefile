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
sbin_PROGRAMS =  \
	avahi-dnsconfd
subdir = avahi-dnsconfd
DIST_COMMON = $(srcdir)/Makefile.am $(srcdir)/Makefile.in
mkinstalldirs = $(install_sh) -d
CONFIG_HEADER = $(top_builddir)/config.h
CONFIG_CLEAN_FILES =
am__installdirs = "$(DESTDIR)$(sbindir)" "$(DESTDIR)$(pkgsysconfdir)"
sbinPROGRAMS_INSTALL = $(INSTALL_PROGRAM)
PROGRAMS = $(sbin_PROGRAMS)
am__avahi_dnsconfd_SOURCES_DIST = main.c
am_avahi_dnsconfd_OBJECTS = avahi_dnsconfd-main.o
avahi_dnsconfd_OBJECTS = $(am_avahi_dnsconfd_OBJECTS)
am__DEPENDENCIES_1 =
avahi_dnsconfd_DEPENDENCIES = ../avahi-common/libavahi-common.a \
	$(am__DEPENDENCIES_1)
avahi_dnsconfd_LINK = $(LD) $(avahi_dnsconfd_CFLAGS) \
	$(CFLAGS) $(AM_LDFLAGS) $(LDFLAGS) -o $@
pkgsysconfSCRIPT_INSTALL = $(INSTALL_SCRIPT)
SCRIPTS = $(pkgsysconf_SCRIPTS)
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
SOURCES = $(avahi_dnsconfd_SOURCES)
DIST_SOURCES = $(am__avahi_dnsconfd_SOURCES_DIST)
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
abs_builddir = $(ROOTDIR)/user/avahi/avahi-dnsconfd
abs_srcdir = $(ROOTDIR)/user/avahi/avahi-dnsconfd
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
pkgsysconfdir = $(sysconfdir)/avahi

# This cool debug trap works on i386/gcc only
AM_CFLAGS = -I$(top_srcdir) \
	-DAVAHI_RUNTIME_DIR=\"$(avahi_runtime_dir)/\" \
	-DAVAHI_SOCKET=\"$(avahi_socket)\" \
	-DAVAHI_DNSCONF_SCRIPT=\"$(pkgsysconfdir)/avahi-dnsconfd.action\" \
	'-DDEBUG_TRAP=__asm__("int \
	$$3")'
avahi_dnsconfd_SOURCES = main.c 
avahi_dnsconfd_CFLAGS = $(AM_CFLAGS) $(LIBDAEMON_CFLAGS)
avahi_dnsconfd_LDADD = -Wl,-elf2flt $(AM_LDADD) ../avahi-common/libavahi-common.a $(LIBDAEMON_LIBS)
pkgsysconf_SCRIPTS = avahi-dnsconfd.action
EXTRA_DIST = avahi-dnsconfd.action

all: all-am

.SUFFIXES:
.SUFFIXES: .c .o
install-sbinPROGRAMS: $(sbin_PROGRAMS)
	@$(NORMAL_INSTALL)
	test -z "$(sbindir)" || $(MKDIR_P) "$(DESTDIR)$(sbindir)"
	@list='$(sbin_PROGRAMS)'; for p in $$list; do \
	   echo " $(INSTALL_PROGRAM_ENV) $(sbinPROGRAMS_INSTALL) '$$p' '$(DESTDIR)$(sbindir)/$$p'"; \
	   $(INSTALL_PROGRAM_ENV) $(sbinPROGRAMS_INSTALL) "$$p" "$(DESTDIR)$(sbindir)/$$p" || exit 1; \
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
avahi-dnsconfd: $(avahi_dnsconfd_OBJECTS) $(avahi_dnsconfd_DEPENDENCIES) 
	@rm -f avahi-dnsconfd avahi-dnsconfd.gdb
	$(avahi_dnsconfd_LINK) $(avahi_dnsconfd_OBJECTS) $(avahi_dnsconfd_LDADD) $(LIBS)
install-pkgsysconfSCRIPTS: $(pkgsysconf_SCRIPTS)
	@$(NORMAL_INSTALL)
	test -z "$(pkgsysconfdir)" || $(MKDIR_P) "$(DESTDIR)$(pkgsysconfdir)"
	@list='$(pkgsysconf_SCRIPTS)'; for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  if test -f $$d$$p; then \
	    echo " $(pkgsysconfSCRIPT_INSTALL) '$$d$$p' '$(DESTDIR)$(pkgsysconfdir)/$$p'"; \
	    $(pkgsysconfSCRIPT_INSTALL) "$$d$$p" "$(DESTDIR)$(pkgsysconfdir)/$$p"; \
	  else :; fi; \
	done

uninstall-pkgsysconfSCRIPTS:
	@$(NORMAL_UNINSTALL)
	@list='$(pkgsysconf_SCRIPTS)'; for p in $$list; do \
	  echo " rm -f '$(DESTDIR)$(pkgsysconfdir)/$$p'"; \
	  rm -f "$(DESTDIR)$(pkgsysconfdir)/$$p"; \
	done

mostlyclean-compile:
	-rm -f *.o

distclean-compile:
	-rm -f *.tab.c

-include ./$(DEPDIR)/avahi_dnsconfd-main.Po

.c.o:
	$(COMPILE) -MT $@ -MD -MP -MF $(DEPDIR)/$*.Tpo -c -o $@ $<
	mv -f $(DEPDIR)/$*.Tpo $(DEPDIR)/$*.Po
#	source='$<' object='$@' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(COMPILE) -c $<

avahi_dnsconfd-main.o: main.c
	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_dnsconfd_CFLAGS) $(CFLAGS) -MT avahi_dnsconfd-main.o -MD -MP -MF $(DEPDIR)/avahi_dnsconfd-main.Tpo -c -o avahi_dnsconfd-main.o `test -f 'main.c' || echo '$(srcdir)/'`main.c
	mv -f $(DEPDIR)/avahi_dnsconfd-main.Tpo $(DEPDIR)/avahi_dnsconfd-main.Po
#	source='main.c' object='avahi_dnsconfd-main.o' libtool=no \
#	DEPDIR=$(DEPDIR) $(CCDEPMODE) $(depcomp) \
#	$(CC) $(DEFS) $(DEFAULT_INCLUDES) $(INCLUDES) $(AM_CPPFLAGS) $(CPPFLAGS) $(avahi_dnsconfd_CFLAGS) $(CFLAGS) -c -o avahi_dnsconfd-main.o `test -f 'main.c' || echo '$(srcdir)/'`main.c

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
all-am: Makefile $(PROGRAMS) $(SCRIPTS)
installdirs:
	for dir in "$(DESTDIR)$(sbindir)" "$(DESTDIR)$(pkgsysconfdir)"; do \
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

clean-am: clean-generic clean-sbinPROGRAMS \
	mostlyclean-am

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

install-data-am: install-pkgsysconfSCRIPTS

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

uninstall-am: uninstall-pkgsysconfSCRIPTS uninstall-sbinPROGRAMS

.MAKE: install-am install-strip

.PHONY: CTAGS GTAGS all all-am check check-am clean clean-generic \
	clean-sbinPROGRAMS ctags distclean \
	distclean-compile distclean-generic \
	distclean-tags distdir dvi dvi-am html html-am info info-am \
	install install-am install-data install-data-am install-dvi \
	install-dvi-am install-exec install-exec-am install-html \
	install-html-am install-info install-info-am install-man \
	install-pdf install-pdf-am install-pkgsysconfSCRIPTS \
	install-ps install-ps-am install-sbinPROGRAMS install-strip \
	installcheck installcheck-am installdirs maintainer-clean \
	maintainer-clean-generic mostlyclean mostlyclean-compile \
	mostlyclean-generic pdf pdf-am ps ps-am \
	tags uninstall uninstall-am uninstall-pkgsysconfSCRIPTS \
	uninstall-sbinPROGRAMS

# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
