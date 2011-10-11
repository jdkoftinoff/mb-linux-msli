# Requirements:
# - set VER to the package directory
# - define romfs target
# Optional:
# - set CONF_OPTS
#
# Then just add to your package Makefile:
# include $(ROOTDIR)/tools/autotools.mk

BUILDDIR      = build-$(VER)
BUILDDIR_HOST = build-host-$(VER)
# relative to $(VER)/
CONFIGURE     = configure

all: $(BUILDDIR)/Makefile $(AUTOTOOLS_ALL_DEPS)
	$(MAKE) pre-build
	$(MAKE) -C $(BUILDDIR) install DESTDIR=$(STAGEDIR)
	$(MAKE) post-build

	$(ROOTDIR)/tools/cross-fix-root

pre-build::
post-build::

include $(ROOTDIR)/tools/download.mk

ifneq ($(findstring s,$(MAKEFLAGS)),)
echo-cmd = :
else
echo-cmd = printf
endif

$(ROOTDIR)/tools/autotools-cache/build/$(CONFIGURE_HOST):
	set -e; \
	mkdir -p $(dir $@)/$$$$; \
	cd $(dir $@)/$$$$; \
	gt=`../../create-target-cache.sh $(CONFIGURE_HOST)`; \
	cp $$gt $(ROOTDIR)/vendors/config/$$gt; \
	touch $@
$(ROOTDIR)/tools/autotools-cache/build/config.cache:
	set -e; \
	mkdir -p $(dir $@)/$$$$; \
	cd $(dir $@)/$$$$; \
	CONFIG_SITE="" ../../configure -C; \
	mv config.cache $@
$(ROOTDIR)/vendors/config/config.site.build: $(ROOTDIR)/tools/autotools-cache/build/config.cache
	grep -v ^ac_cv_env_ $< > $@.$$$$ && mv $@.$$$$ $@
autotools-cache: $(ROOTDIR)/vendors/config/config.site.build $(ROOTDIR)/tools/autotools-cache/build/$(CONFIGURE_HOST)
.PHONY: autotools-cache

if_changed = \
	settings="$(3)/.dist.settings" ; \
	echo $(2) $(CFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) > .new.settings ; \
	if ! cmp -s .new.settings $$settings ; then \
		$(echo-cmd) "%s\n" '$(cmd_$(1))' ; \
		( $(cmd_$(1)) ) || exit $$? ; \
	fi ; \
	mv .new.settings $$settings

cmd_configure = \
	set -e ; \
	conf="$$PWD/$(VER)/$(CONFIGURE)" ; \
	chmod a+rx "$$conf" ; \
	find $(VER) -type f -print0 | xargs -0 touch -r "$$conf" ; \
	rm -rf $(3) ; \
	mkdir -p $(3) ; \
	cd $(3) ; \
	"$$conf" $(2)
$(BUILDDIR)/Makefile: $(BUILDDIR_HOST)/Makefile autotools-cache FORCE
	@$(call if_changed,configure,$(CONFIGURE_OPTS) $(CONF_OPTS),$(BUILDDIR))

$(BUILDDIR_HOST)/Makefile: autotools-cache FORCE
ifeq ($(AUTOTOOLS_BUILD_HOST),true)
	@export AR="" CC=$(HOSTCC) CXX="" LD="" RANLIB="" \
		CPPFLAGS="" CFLAGS="-O2 -g" CXXFLAGS="-O2 -g" LDFLAGS="" CONFIG_SITE="" \
	$(call if_changed,configure,$(BUILD_CONFIGURE_OPTS) $(BUILD_CONF_OPTS),$(BUILDDIR_HOST))
	$(MAKE) host-build
endif

clean:
	rm -rf build* .build*

.PHONY: all clean pre-build post-build romfs FORCE

#
# Helper functions
#

# $(call _USE_CONF,enable,disable,LIB_FFMPEG,video,blah) -> --enable-video=blah if LIB_FFMPEG
# $(call _USE_CONF,with,without,LIB_FFMPEG,video)        -> --with-video if LIB_FFMPEG
_USE_CONF = $(shell \
	opt="$(5)"; test "$${opt:+set}" = "set" && opt="=$${opt}"; \
	test "$(CONFIG_$(3))" = "y" \
		&& echo "--$(1)-$(4)$${opt}" \
		|| echo "--$(2)-$(4)")

# $(call USE_ENABLE,LIB_FFMPEG,video) => --enable-video if LIB_FFMPEG is set
USE_ENABLE = $(call _USE_CONF,enable,disable,$(1),$(2),$(3))

# $(call USE_WITH,LIB_FFMPEG,video) => --with-video if LIB_FFMPEG is set
USE_WITH = $(call _USE_CONF,with,without,$(1),$(2),$(3))
