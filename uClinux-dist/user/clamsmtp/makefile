
CONFOPTS :=	--enable-dependency-tracking		\
		--sysconfdir=/etc/config

TS_LIBS :=

ifdef CONFIG_USER_TRUSTEDSOURCE_V2
TS_LIBS := -lrep -lsnapgear -lutil
else 
ifdef CONFIG_USER_TRUSTEDSOURCE_BLACKLIST
TS_LIBS := -lsnapgear -lutil
endif
endif

ifdef CONFIG_USER_TRUSTEDSOURCE_BLACKLIST
TS_LIBS := -lconfig $(LIBTCL) $(TS_LIBS) $(LIBCRYPT) $(LIBDL) 
endif

CONFOPTS += CFLAGS="-I$(ROOTDIR)/include" CPPFLAGS="-I$(ROOTDIR)/include"  LDFLAGS="-L$(ROOTDIR)/lib" LIBS="$(TS_LIBS)"

all: build/build
	$(MAKE) -C build

build/build: makefile
	@chmod u+x configure
	@rm -rf build
	@mkdir build
	(cd build; sh ../configure $(CONFIGURE_OPTS) $(CONFOPTS))
	@touch build/build

clean:
	rm -rf build

romfs:
	[ -d $(ROMFSDIR)/etc/clamav ] || mkdir $(ROMFSDIR)/etc/clamav
	$(ROMFSINST) build/src/clamsmtpd /bin/clamsmtpd
	$(ROMFSINST) doc/clamsmtpd.conf /etc/clamav/clamsmtpd.conf
