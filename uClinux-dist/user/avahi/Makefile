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

# Copyright (C) 2004 Oren Ben-Kiki
# This file is distributed under the same terms as the Automake macro files.

# Generate automatic documentation using Doxygen. Goals and variables values
# are controlled by the various DX_COND_??? conditionals set by autoconf.
#
# The provided goals are:
# doxygen-doc: Generate all doxygen documentation.
# doxygen-run: Run doxygen, which will generate some of the documentation
#              (HTML, CHM, CHI, MAN, RTF, XML) but will not do the post
#              processing required for the rest of it (PS, PDF, and some MAN).
# doxygen-man: Rename some doxygen generated man pages.
# doxygen-ps: Generate doxygen PostScript documentation.
# doxygen-pdf: Generate doxygen PDF documentation.
#
# Note that by default these are not integrated into the automake goals. If
# doxygen is used to generate man pages, you can achieve this integration by
# setting man3_MANS to the list of man pages generated and then adding the
# dependency:
#
#   $(man3_MANS): doxygen-doc
#
# This will cause make to run doxygen and generate all the documentation.
#
# The following variable is intended for use in Makefile.am:
#
# DX_CLEANFILES = everything to clean.
#
# This is usually added to MOSTLYCLEANFILES.

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
DIST_COMMON = $(am__configure_deps) $(srcdir)/Makefile.am \
	$(srcdir)/Makefile.in $(srcdir)/common/doxygen.mk \
	$(srcdir)/config.h.in $(top_srcdir)/configure ABOUT-NLS \
	ChangeLog compile config.guess config.rpath config.sub depcomp \
	install-sh ltmain.sh py-compile
#am__append_1 = \
#	$(srcdir)/avahi-qt/qt-watch.h

#am__append_2 = \
#	$(srcdir)/avahi-qt/qt-watch.h

#am__append_3 = \
#	$(srcdir)/avahi-glib/glib-watch.h \
#	$(srcdir)/avahi-glib/glib-malloc.h

###am__append_4 = \
###	$(srcdir)/avahi-gobject/ga-client.h \
###	$(srcdir)/avahi-gobject/ga-entry-group.h \
###	$(srcdir)/avahi-gobject/ga-enums.h \
###	$(srcdir)/avahi-gobject/ga-error.h \
###	$(srcdir)/avahi-gobject/ga-record-browser.h \
###	$(srcdir)/avahi-gobject/ga-service-browser.h \
###	$(srcdir)/avahi-gobject/ga-service-resolver.h

#am__append_5 = \
#	$(srcdir)/avahi-client/client.h \
#	$(srcdir)/avahi-client/lookup.h \
#	$(srcdir)/avahi-client/publish.h

##am__append_6 = \
##	$(srcdir)/avahi-ui/avahi-ui.h

#am__append_7 = \
#	$(srcdir)/avahi-core/core.h \
#	$(srcdir)/avahi-core/lookup.h \
#	$(srcdir)/avahi-core/publish.h \
#	$(srcdir)/avahi-core/rr.h \
#	$(srcdir)/avahi-core/log.h

#am__append_8 = \
#	$(srcdir)/avahi-ui/avahi-ui.h

#am__append_9 = avahi-client.pc
#am__append_10 = avahi-client.pc
##am__append_11 = avahi-compat-howl.pc
##am__append_12 = avahi-compat-howl.pc
##am__append_13 = avahi-compat-libdns_sd.pc
##am__append_14 = avahi-compat-libdns_sd.pc
##am__append_15 = avahi-sharp.pc avahi-ui-sharp.pc
##am__append_16 = avahi-sharp.pc avahi-ui-sharp.pc
#am__append_17 = avahi-glib.pc
#am__append_18 = avahi-glib.pc
##am__append_19 = avahi-gobject.pc
##am__append_20 = avahi-gobject.pc
##am__append_21 = avahi-ui.pc
##am__append_22 = avahi-ui.pc
#am__append_23 = avahi-qt3.pc
#am__append_24 = avahi-qt3.pc
#am__append_25 = avahi-qt4.pc
#am__append_26 = avahi-qt4.pc
subdir = .
CONFIG_HEADER = config.h
CONFIG_CLEAN_FILES =
SOURCES =
DIST_SOURCES =
RECURSIVE_TARGETS = all-recursive check-recursive dvi-recursive \
	html-recursive info-recursive install-data-recursive \
	install-dvi-recursive install-exec-recursive \
	install-html-recursive install-info-recursive \
	install-pdf-recursive install-ps-recursive install-recursive \
	installcheck-recursive installdirs-recursive pdf-recursive \
	ps-recursive uninstall-recursive
am__vpath_adj_setup = srcdirstrip=`echo "$(srcdir)" | sed 's|.|.|g'`;
am__vpath_adj = case $$p in \
    $(srcdir)/*) f=`echo "$$p" | sed "s|^$$srcdirstrip/||"`;; \
    *) f=$$p;; \
  esac;
am__strip_dir = `echo $$p | sed -e 's|^.*/||'`;
am__installdirs = "$(DESTDIR)$(pkgconfigdir)"
pkgconfigDATA_INSTALL = $(INSTALL_DATA)
DATA = $(pkgconfig_DATA)
RECURSIVE_CLEAN_TARGETS = mostlyclean-recursive clean-recursive	\
  distclean-recursive maintainer-clean-recursive
ETAGS = etags
CTAGS = ctags
DIST_SUBDIRS = $(SUBDIRS)
DISTFILES = $(DIST_COMMON) $(DIST_SOURCES) $(TEXINFOS) $(EXTRA_DIST)
distdir = $(PACKAGE)-$(VERSION)
top_distdir = $(distdir)
am__remove_distdir = \
  { test ! -d $(distdir) \
    || { find $(distdir) -type d ! -perm -200 -exec chmod u+w {} ';' \
         && rm -fr $(distdir); }; }
DIST_ARCHIVES = $(distdir).tar.gz
GZIP_ENV = --best
distuninstallcheck_listfiles = find . -type f -print
distcleancheck_listfiles = find . -type f -print
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
abs_builddir = $(ROOTDIR)/user/avahi
abs_srcdir = $(ROOTDIR)/user/avahi
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
top_build_prefix = 
top_builddir = .
top_srcdir = .
ACLOCAL_AMFLAGS = -I common

EXTRA_DIST = \
	autogen.sh \
	bootstrap.sh \
	LICENSE \
	$(DX_CONFIG) \
	docs/INSTALL \
	docs/TODO \
	docs/NEWS \
	docs/README \
	docs/DBUS-API \
	docs/AUTHORS \
	docs/HACKING \
	docs/API-CHANGES-0.6 \
	docs/COMPAT-LAYERS \
	docs/MALLOC \
	docs/overview.dia \
	docs/server-states.dia \
	docs/avahi-poll.dia \
	avahi-core.pc.in \
	avahi-client.pc.in \
	avahi-glib.pc.in \
	avahi-gobject.pc.in \
	avahi-qt3.pc.in \
	avahi-qt4.pc.in \
	avahi-sharp.pc.in \
	avahi-ui-sharp.pc.in \
	avahi-compat-libdns_sd.pc.in \
	avahi-compat-howl.pc.in \
	avahi-ui.pc.in \
	doxygen_to_devhelp.xsl

SUBDIRS = \
	common \
	avahi-common \
	avahi-core \
	avahi-daemon \
	initscript \
	avahi-dnsconfd \
	examples \
	man \
	tests \
	service-type-database \
	avahi-autoipd \
	po

SUBDIRS_DISABLED = \
	avahi-qt \
	avahi-client \
	avahi-glib \
	avahi-gobject \
	avahi-discover-standalone \
	avahi-sharp \
	avahi-utils \
	avahi-python \
	avahi-compat-libdns_sd \
	avahi-compat-howl \
	avahi-ui \
	avahi-ui-sharp

DX_INPUT = $(srcdir)/avahi-common/address.h \
	$(srcdir)/avahi-common/malloc.h \
	$(srcdir)/avahi-common/strlst.h \
	$(srcdir)/avahi-common/alternative.h \
	$(srcdir)/avahi-common/defs.h $(srcdir)/avahi-common/error.h \
	$(srcdir)/avahi-common/domain.h $(srcdir)/avahi-common/watch.h \
	$(srcdir)/avahi-common/simple-watch.h \
	$(srcdir)/avahi-common/thread-watch.h $(am__append_1) \
	$(am__append_2) $(am__append_3) $(am__append_4) \
	$(am__append_5) $(am__append_6) $(am__append_7) \
	$(am__append_8)
DX_EXAMPLE_PATH = $(srcdir)/examples
DX_EXAMPLE_PATTERNS = *.c
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = avahi-core.pc $(am__append_9) $(am__append_11) \
	$(am__append_13) $(am__append_15) $(am__append_17) \
	$(am__append_19) $(am__append_21) $(am__append_23) \
	$(am__append_25)
CLEANFILES = avahi-core.pc $(am__append_10) $(am__append_12) \
	$(am__append_14) $(am__append_16) $(am__append_18) \
	$(am__append_20) $(am__append_22) $(am__append_24) \
	$(am__append_26) avahi.devhelp
MOSTLYCLEANFILES = $(DX_CLEANFILES)
DISTCHECK_CONFIGURE_FLAGS = \
	--disable-monodoc

DISTCLEANFILES = \
	po/.intltool-merge-cache

all: config.h
	$(MAKE) $(AM_MAKEFLAGS) all-recursive

.SUFFIXES:

install-pkgconfigDATA: $(pkgconfig_DATA)
	@$(NORMAL_INSTALL)
	test -z "$(pkgconfigdir)" || $(MKDIR_P) "$(DESTDIR)$(pkgconfigdir)"
	@list='$(pkgconfig_DATA)'; for p in $$list; do \
	  if test -f "$$p"; then d=; else d="$(srcdir)/"; fi; \
	  f=$(am__strip_dir) \
	  echo " $(pkgconfigDATA_INSTALL) '$$d$$p' '$(DESTDIR)$(pkgconfigdir)/$$f'"; \
	  $(pkgconfigDATA_INSTALL) "$$d$$p" "$(DESTDIR)$(pkgconfigdir)/$$f"; \
	done

uninstall-pkgconfigDATA:
	@$(NORMAL_UNINSTALL)
	@list='$(pkgconfig_DATA)'; for p in $$list; do \
	  f=$(am__strip_dir) \
	  echo " rm -f '$(DESTDIR)$(pkgconfigdir)/$$f'"; \
	  rm -f "$(DESTDIR)$(pkgconfigdir)/$$f"; \
	done

# This directory's subdirectories are mostly independent; you can cd
# into them and run `make' without going through this Makefile.
# To change the values of `make' variables: instead of editing Makefiles,
# (1) if the variable is set in `config.status', edit `config.status'
#     (which will cause the Makefiles to be regenerated when you run `make');
# (2) otherwise, pass the desired values on the `make' command line.
$(RECURSIVE_TARGETS):
	@failcom='exit 1'; \
	for f in x $$MAKEFLAGS; do \
	  case $$f in \
	    *=* | --[!k]*);; \
	    *k*) failcom='fail=yes';; \
	  esac; \
	done; \
	dot_seen=no; \
	target=`echo $@ | sed s/-recursive//`; \
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  echo "Making $$target in $$subdir"; \
	  if test "$$subdir" = "."; then \
	    dot_seen=yes; \
	    local_target="$$target-am"; \
	  else \
	    local_target="$$target"; \
	  fi; \
	  (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) $$local_target) \
	  || eval $$failcom; \
	done; \
	if test "$$dot_seen" = "no"; then \
	  $(MAKE) $(AM_MAKEFLAGS) "$$target-am" || exit 1; \
	fi; test -z "$$fail"

$(RECURSIVE_CLEAN_TARGETS):
	@failcom='exit 1'; \
	for f in x $$MAKEFLAGS; do \
	  case $$f in \
	    *=* | --[!k]*);; \
	    *k*) failcom='fail=yes';; \
	  esac; \
	done; \
	dot_seen=no; \
	case "$@" in \
	  distclean-* | maintainer-clean-*) list='$(DIST_SUBDIRS)' ;; \
	  *) list='$(SUBDIRS)' ;; \
	esac; \
	rev=''; for subdir in $$list; do \
	  if test "$$subdir" = "."; then :; else \
	    rev="$$subdir $$rev"; \
	  fi; \
	done; \
	rev="$$rev ."; \
	target=`echo $@ | sed s/-recursive//`; \
	for subdir in $$rev; do \
	  echo "Making $$target in $$subdir"; \
	  if test "$$subdir" = "."; then \
	    local_target="$$target-am"; \
	  else \
	    local_target="$$target"; \
	  fi; \
	  (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) $$local_target) \
	  || eval $$failcom; \
	done && test -z "$$fail"
tags-recursive:
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  test "$$subdir" = . || (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) tags); \
	done
ctags-recursive:
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  test "$$subdir" = . || (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) ctags); \
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

TAGS: tags-recursive $(HEADERS) $(SOURCES) config.h.in $(TAGS_DEPENDENCIES) \
		$(TAGS_FILES) $(LISP)
	tags=; \
	here=`pwd`; \
	if ($(ETAGS) --etags-include --version) >/dev/null 2>&1; then \
	  include_option=--etags-include; \
	  empty_fix=.; \
	else \
	  include_option=--include; \
	  empty_fix=; \
	fi; \
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  if test "$$subdir" = .; then :; else \
	    test ! -f $$subdir/TAGS || \
	      tags="$$tags $$include_option=$$here/$$subdir/TAGS"; \
	  fi; \
	done; \
	list='$(SOURCES) $(HEADERS) config.h.in $(LISP) $(TAGS_FILES)'; \
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
CTAGS: ctags-recursive $(HEADERS) $(SOURCES) config.h.in $(TAGS_DEPENDENCIES) \
		$(TAGS_FILES) $(LISP)
	tags=; \
	list='$(SOURCES) $(HEADERS) config.h.in $(LISP) $(TAGS_FILES)'; \
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
	$(am__remove_distdir)
	test -d $(distdir) || mkdir $(distdir)
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
	list='$(DIST_SUBDIRS)'; for subdir in $$list; do \
	  if test "$$subdir" = .; then :; else \
	    test -d "$(distdir)/$$subdir" \
	    || $(MKDIR_P) "$(distdir)/$$subdir" \
	    || exit 1; \
	    distdir=`cd $(distdir) && pwd`; \
	    top_distdir=`cd $(top_distdir) && pwd`; \
	    (cd $$subdir && \
	      $(MAKE) $(AM_MAKEFLAGS) \
	        top_distdir="$$top_distdir" \
	        distdir="$$distdir/$$subdir" \
		am__remove_distdir=: \
		am__skip_length_check=: \
	        distdir) \
	      || exit 1; \
	  fi; \
	done
	$(MAKE) $(AM_MAKEFLAGS) \
	  top_distdir="$(top_distdir)" distdir="$(distdir)" \
	  dist-hook
	-find $(distdir) -type d ! -perm -777 -exec chmod a+rwx {} \; -o \
	  ! -type d ! -perm -444 -links 1 -exec chmod a+r {} \; -o \
	  ! -type d ! -perm -400 -exec chmod a+r {} \; -o \
	  ! -type d ! -perm -444 -exec $(install_sh) -c -m a+r {} {} \; \
	|| chmod -R a+r $(distdir)
dist-gzip: distdir
	tardir=$(distdir) && $(am__tar) | GZIP=$(GZIP_ENV) gzip -c >$(distdir).tar.gz
	$(am__remove_distdir)

dist-bzip2: distdir
	tardir=$(distdir) && $(am__tar) | bzip2 -9 -c >$(distdir).tar.bz2
	$(am__remove_distdir)

dist-lzma: distdir
	tardir=$(distdir) && $(am__tar) | lzma -9 -c >$(distdir).tar.lzma
	$(am__remove_distdir)

dist-tarZ: distdir
	tardir=$(distdir) && $(am__tar) | compress -c >$(distdir).tar.Z
	$(am__remove_distdir)

dist-shar: distdir
	shar $(distdir) | GZIP=$(GZIP_ENV) gzip -c >$(distdir).shar.gz
	$(am__remove_distdir)

dist-zip: distdir
	-rm -f $(distdir).zip
	zip -rq $(distdir).zip $(distdir)
	$(am__remove_distdir)

dist dist-all: distdir
	tardir=$(distdir) && $(am__tar) | GZIP=$(GZIP_ENV) gzip -c >$(distdir).tar.gz
	$(am__remove_distdir)

# This target untars the dist file and tries a VPATH configuration.  Then
# it guarantees that the distribution is self-contained by making another
# tarfile.
distcheck: dist
	case '$(DIST_ARCHIVES)' in \
	*.tar.gz*) \
	  GZIP=$(GZIP_ENV) gunzip -c $(distdir).tar.gz | $(am__untar) ;;\
	*.tar.bz2*) \
	  bunzip2 -c $(distdir).tar.bz2 | $(am__untar) ;;\
	*.tar.lzma*) \
	  unlzma -c $(distdir).tar.lzma | $(am__untar) ;;\
	*.tar.Z*) \
	  uncompress -c $(distdir).tar.Z | $(am__untar) ;;\
	*.shar.gz*) \
	  GZIP=$(GZIP_ENV) gunzip -c $(distdir).shar.gz | unshar ;;\
	*.zip*) \
	  unzip $(distdir).zip ;;\
	esac
	chmod -R a-w $(distdir); chmod a+w $(distdir)
	mkdir $(distdir)/_build
	mkdir $(distdir)/_inst
	chmod a-w $(distdir)
	dc_install_base=`cd $(distdir)/_inst && pwd | sed -e 's,^[^:\\/]:[\\/],/,'` \
	  && dc_destdir="$${TMPDIR-/tmp}/am-dc-$$$$/" \
	  && cd $(distdir)/_build \
	  && ../configure --srcdir=.. --prefix="$$dc_install_base" \
	    $(DISTCHECK_CONFIGURE_FLAGS) \
	  && $(MAKE) $(AM_MAKEFLAGS) \
	  && $(MAKE) $(AM_MAKEFLAGS) dvi \
	  && $(MAKE) $(AM_MAKEFLAGS) check \
	  && $(MAKE) $(AM_MAKEFLAGS) install \
	  && $(MAKE) $(AM_MAKEFLAGS) installcheck \
	  && $(MAKE) $(AM_MAKEFLAGS) uninstall \
	  && $(MAKE) $(AM_MAKEFLAGS) distuninstallcheck_dir="$$dc_install_base" \
	        distuninstallcheck \
	  && chmod -R a-w "$$dc_install_base" \
	  && ({ \
	       (cd ../.. && umask 077 && mkdir "$$dc_destdir") \
	       && $(MAKE) $(AM_MAKEFLAGS) DESTDIR="$$dc_destdir" install \
	       && $(MAKE) $(AM_MAKEFLAGS) DESTDIR="$$dc_destdir" uninstall \
	       && $(MAKE) $(AM_MAKEFLAGS) DESTDIR="$$dc_destdir" \
	            distuninstallcheck_dir="$$dc_destdir" distuninstallcheck; \
	      } || { rm -rf "$$dc_destdir"; exit 1; }) \
	  && rm -rf "$$dc_destdir" \
	  && $(MAKE) $(AM_MAKEFLAGS) dist \
	  && rm -rf $(DIST_ARCHIVES) \
	  && $(MAKE) $(AM_MAKEFLAGS) distcleancheck
	$(am__remove_distdir)
	@(echo "$(distdir) archives ready for distribution: "; \
	  list='$(DIST_ARCHIVES)'; for i in $$list; do echo $$i; done) | \
	  sed -e 1h -e 1s/./=/g -e 1p -e 1x -e '$$p' -e '$$x'
distuninstallcheck:
	@cd $(distuninstallcheck_dir) \
	&& test `$(distuninstallcheck_listfiles) | wc -l` -le 1 \
	   || { echo "ERROR: files left after uninstall:" ; \
	        if test -n "$(DESTDIR)"; then \
	          echo "  (check DESTDIR support)"; \
	        fi ; \
	        $(distuninstallcheck_listfiles) ; \
	        exit 1; } >&2
distcleancheck: distclean
	@if test '$(srcdir)' = . ; then \
	  echo "ERROR: distcleancheck can only run from a VPATH build" ; \
	  exit 1 ; \
	fi
	@test `$(distcleancheck_listfiles) | wc -l` -eq 0 \
	  || { echo "ERROR: files left in build directory after distclean:" ; \
	       $(distcleancheck_listfiles) ; \
	       exit 1; } >&2
check-am: all-am
check: check-recursive
all-am: Makefile $(DATA) config.h
installdirs: installdirs-recursive
installdirs-am:
	for dir in "$(DESTDIR)$(pkgconfigdir)"; do \
	  test -z "$$dir" || $(MKDIR_P) "$$dir"; \
	done
install: install-recursive
install-exec: install-exec-recursive
install-data: install-data-recursive
uninstall: uninstall-recursive

install-am: all-am
	@$(MAKE) $(AM_MAKEFLAGS) install-exec-am install-data-am

installcheck: installcheck-recursive
install-strip:
	$(MAKE) $(AM_MAKEFLAGS) INSTALL_PROGRAM="$(INSTALL_STRIP_PROGRAM)" \
	  install_sh_PROGRAM="$(INSTALL_STRIP_PROGRAM)" INSTALL_STRIP_FLAG=-s \
	  `test -z '$(STRIP)' || \
	    echo "INSTALL_PROGRAM_ENV=STRIPPROG='$(STRIP)'"` install
mostlyclean-generic:
	-test -z "$(MOSTLYCLEANFILES)" || rm -f $(MOSTLYCLEANFILES)

clean-generic:
	-test -z "$(CLEANFILES)" || rm -f $(CLEANFILES)

distclean-generic:
	-test -z "$(CONFIG_CLEAN_FILES)" || rm -f $(CONFIG_CLEAN_FILES)
	-test -z "$(DISTCLEANFILES)" || rm -f $(DISTCLEANFILES)

maintainer-clean-generic:
	@echo "This command is intended for maintainers to use"
	@echo "it deletes files that may require special tools to rebuild."
clean: clean-recursive

clean-am: clean-generic mostlyclean-am

distclean: distclean-recursive
	-rm -f $(am__CONFIG_DISTCLEAN_FILES)
	-rm -f Makefile
distclean-am: clean-am distclean-generic \
	distclean-tags

dvi: dvi-recursive

dvi-am:

html: html-recursive

info: info-recursive

info-am:

install-data-am: install-pkgconfigDATA

install-dvi: install-dvi-recursive

install-exec-am:

install-html: install-html-recursive

install-info: install-info-recursive

install-man:

install-pdf: install-pdf-recursive

install-ps: install-ps-recursive

installcheck-am:

maintainer-clean: maintainer-clean-recursive
	-rm -f $(am__CONFIG_DISTCLEAN_FILES)
	-rm -rf $(top_srcdir)/autom4te.cache
	-rm -f Makefile
maintainer-clean-am: distclean-am maintainer-clean-generic

mostlyclean: mostlyclean-recursive

mostlyclean-am: mostlyclean-generic

pdf: pdf-recursive

pdf-am:

ps: ps-recursive

ps-am:

uninstall-am: uninstall-pkgconfigDATA

.MAKE: $(RECURSIVE_CLEAN_TARGETS) $(RECURSIVE_TARGETS) install-am \
	install-strip

.PHONY: $(RECURSIVE_CLEAN_TARGETS) $(RECURSIVE_TARGETS) CTAGS GTAGS \
	all all-am am--refresh check check-am clean clean-generic \
	ctags ctags-recursive dist dist-all dist-bzip2 \
	dist-gzip dist-hook dist-lzma dist-shar dist-tarZ dist-zip \
	distcheck distclean distclean-generic \
	distclean-tags distcleancheck distdir \
	distuninstallcheck dvi dvi-am html html-am info info-am \
	install install-am install-data install-data-am install-dvi \
	install-dvi-am install-exec install-exec-am install-html \
	install-html-am install-info install-info-am install-man \
	install-pdf install-pdf-am install-pkgconfigDATA install-ps \
	install-ps-am install-strip installcheck installcheck-am \
	installdirs installdirs-am maintainer-clean \
	maintainer-clean-generic mostlyclean mostlyclean-generic \
	pdf pdf-am ps ps-am tags tags-recursive \
	uninstall uninstall-am uninstall-pkgconfigDATA


##doxygen-ps: doxygen/avahi.ps

##doxygen/avahi.ps: doxygen/avahi.tag
##	cd doxygen/latex; \
##	rm -f *.aux *.toc *.idx *.ind *.ilg *.log *.out; \
##	$(DX_LATEX) refman.tex; \
##	$(MAKEINDEX_PATH) refman.idx; \
##	$(DX_LATEX) refman.tex; \
##	countdown=5; \
##	while $(DX_EGREP) 'Rerun (LaTeX|to get cross-references right)' \
##	                  refman.log > /dev/null 2>&1 \
##	   && test $$countdown -gt 0; do \
##	    $(DX_LATEX) refman.tex; \
##	    countdown=`expr $$countdown - 1`; \
##	done; \
##	$(DX_DVIPS) -o ../avahi.ps refman.dvi

##doxygen-pdf: doxygen/avahi.pdf

##doxygen/avahi.pdf: doxygen/avahi.tag
##	cd doxygen/latex; \
##	rm -f *.aux *.toc *.idx *.ind *.ilg *.log *.out; \
##	$(DX_PDFLATEX) refman.tex; \
##	$(DX_MAKEINDEX) refman.idx; \
##	$(DX_PDFLATEX) refman.tex; \
##	countdown=5; \
##	while $(DX_EGREP) 'Rerun (LaTeX|to get cross-references right)' \
##	                  refman.log > /dev/null 2>&1 \
##	   && test $$countdown -gt 0; do \
##	    $(DX_PDFLATEX) refman.tex; \
##	    countdown=`expr $$countdown - 1`; \
##	done; \
##	mv refman.pdf ../avahi.pdf

#.PHONY: doxygen-run doxygen-doc $(DX_PS_GOAL) $(DX_PDF_GOAL)

#.INTERMEDIATE: doxygen-run $(DX_PS_GOAL) $(DX_PDF_GOAL)

#doxygen-run: doxygen/avahi.tag

#doxygen-doc: doxygen-run $(DX_PS_GOAL) $(DX_PDF_GOAL)

#doxygen/avahi.tag: $(DX_CONFIG) $(pkginclude_HEADERS) $(DX_INPUT) $(DX_EXAMPLE_PATH)/$(DX_EXAMPLE_PATTERNS)
#	rm -rf doxygen
#	INPUT='$(DX_INPUT)' EXAMPLE_PATH='$(DX_EXAMPLE_PATH)' EXAMPLE_PATTERNS='$(DX_EXAMPLE_PATTERNS)' \
#	$(DX_ENV) $(DX_DOXYGEN) $(srcdir)/$(DX_CONFIG)

avahi-core.pc: avahi-core.pc.in
	sed -e 's,@prefix\@,$(prefix),g' \
	    -e 's,@libdir\@,$(libdir),g' \
	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' $< > $@

#avahi-client.pc: avahi-client.pc.in
#	sed -e 's,@prefix\@,$(prefix),g' \
#	    -e 's,@libdir\@,$(libdir),g' \
#	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' $< > $@

##avahi-compat-howl.pc: avahi-compat-howl.pc.in
##	sed -e 's,@prefix\@,$(prefix),g' \
##	    -e 's,@libdir\@,$(libdir),g' \
##	    -e 's,@HOWL_COMPAT_VERSION\@,$(HOWL_COMPAT_VERSION),g' $< > $@

##avahi-compat-libdns_sd.pc: avahi-compat-libdns_sd.pc.in
##	sed -e 's,@prefix\@,$(prefix),g' \
##	    -e 's,@libdir\@,$(libdir),g' \
##	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' $< > $@

##avahi-sharp.pc: avahi-sharp.pc.in
##	sed -e 's,@prefix\@,$(prefix),g' \
##	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' \
##	    -e 's,@libdir\@,$(libdir),g' $< > $@

##avahi-ui-sharp.pc: avahi-ui-sharp.pc.in
##	sed -e 's,@prefix\@,$(prefix),g' \
##	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' \
##	    -e 's,@libdir\@,$(libdir),g' $< > $@

#avahi-glib.pc: avahi-glib.pc.in
#	sed -e 's,@prefix\@,$(prefix),g' \
#	    -e 's,@libdir\@,$(libdir),g' \
#	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' $< > $@

##avahi-gobject.pc: avahi-gobject.pc.in
##	sed -e 's,@prefix\@,$(prefix),g' \
##	    -e 's,@libdir\@,$(libdir),g' \
##	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' $< > $@

##avahi-ui.pc: avahi-ui.pc.in
##	sed -e 's,@prefix\@,$(prefix),g' \
##	    -e 's,@libdir\@,$(libdir),g' \
##	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' $< > $@

#avahi-qt3.pc: avahi-qt3.pc.in
#	sed -e 's,@prefix\@,$(prefix),g' \
#	    -e 's,@libdir\@,$(libdir),g' \
#	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' $< > $@

#avahi-qt4.pc: avahi-qt4.pc.in
#	sed -e 's,@prefix\@,$(prefix),g' \
#	    -e 's,@libdir\@,$(libdir),g' \
#	    -e 's,@PACKAGE_VERSION\@,$(PACKAGE_VERSION),g' $< > $@

avahi.devhelp: doxygen-run
	xsltproc -o $@ doxygen_to_devhelp.xsl doxygen/xml/index.xml

homepage:
	$(MAKE) -C man
	scp avahi-daemon/*.introspect avahi-daemon/introspect.dtd avahi-daemon/introspect.xsl\
	    man/*.xml man/xmltoman.dtd man/xmltoman.xsl \
	    tango:www/avahi.org/tree/download/
	scp avahi-$(PACKAGE_VERSION).tar.gz tango:www/avahi.org/tree/download/
	rm -rf doxygen
	$(MAKE) doxygen-run
	ssh tango rm -rf www/avahi.org/tree/download/doxygen
	scp -r doxygen/html tango:www/avahi.org/tree/download/doxygen

dist-hook:
	if test -d .svn ; then \
		svn update ; \
		chmod u+w ${distdir}/ChangeLog || true ; \
		svn2cl -o ${distdir}/ChangeLog ; \
	fi

romfs:
	 $(INSTALL) avahi-daemon/avahi-daemon avahi-autoipd/avahi-autoipd avahi-dnsconfd/avahi-dnsconfd $(ROMFSDIR)/bin/

# Tell versions [3.59,3.63) of GNU make to not export all variables.
# Otherwise a system limit (for SysV at least) may be exceeded.
.NOEXPORT:
