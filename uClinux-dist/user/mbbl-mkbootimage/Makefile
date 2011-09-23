#
# Overall rules
#
ifdef V
VECHO = :
else
VECHO = echo "  "
ARFLAGS = rc
.SILENT:
endif

VERSION = 1
PATCHLEVEL = 2
SUBLEVEL = 0
EXTRAVERSION =

DTC_VERSION = $(VERSION).$(PATCHLEVEL).$(SUBLEVEL)$(EXTRAVERSION)
CFLAGS = -Wall -g -Os -Wpointer-arith -Wcast-qual -I$(LIBFDT_srcdir)

FLTFLAGS += -s 65536

all: mbbl-imagetool mtd-lock

romfs:
	$(ROMFSINST) /bin/mbbl-imagetool
	$(ROMFSINST) /bin/mtd-lock

#
# Rules for libfdt
#
LIBFDT_objdir = libfdt
LIBFDT_srcdir = libfdt
LIBFDT_archive = $(LIBFDT_objdir)/libfdt.a
LIBFDT_include = $(addprefix $(LIBFDT_srcdir)/,$(LIBFDT_INCLUDES))
LIBFDT_version = $(addprefix $(LIBFDT_srcdir)/,$(LIBFDT_VERSION))

include $(LIBFDT_srcdir)/Makefile.libfdt

.PHONY: libfdt
libfdt: $(LIBFDT_archive)

$(LIBFDT_archive): $(addprefix $(LIBFDT_objdir)/,$(LIBFDT_OBJS))

libfdt_clean:
	@$(VECHO) CLEAN "(libfdt)"
	$(RM) -f $(addprefix $(LIBFDT_objdir)/,$(STD_CLEANFILES))
	$(RM) -f $(LIBFDT_objdir)/*.a
	$(RM) -f $(LIBFDT_objdir)/*.so

#
# Clean rules
#
STD_CLEANFILES = *~ *.o *.so *.d *.a *.i *.s core a.out vgcore.* \
	*.tab.[ch] *.lex.c *.output

#
# Generic compile rules
#
%: %.o
	@$(VECHO) LD $@
	$(LINK.c) -o $@ $^

%.o: %.c
	@$(VECHO) CC $@
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ -c $<

%.o: %.S
	@$(VECHO) AS $@
	$(CC) $(CPPFLAGS) $(AFLAGS) -D__ASSEMBLY__ -o $@ -c $<

%.d: %.c
	@$(VECHO) DEP $<
	$(CC) $(CPPFLAGS) -MM -MG -MT "$*.o $@" $< > $@

%.d: %.S
	@$(VECHO) DEP $<
	$(CC) $(CPPFLAGS) -MM -MG -MT "$*.o $@" $< > $@

%.i:    %.c
	@$(VECHO) CPP $@
	$(CC) $(CPPFLAGS) -E $< > $@

%.s:    %.c
	@$(VECHO) CC -S $@
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ -S $<

%.a:
	@$(VECHO) AR $@
	$(AR) $(ARFLAGS) $@ $^

%.lex.c: %.l
	@$(VECHO) LEX $@
	$(LEX) -o$@ $<

%.tab.c %.tab.h %.output: %.y
	@$(VECHO) BISON $@
	$(BISON) -d $<

mbbl-imagetool: mbbl-imagetool.o $(LIBFDT_archive)
	@$(VECHO) LD "mbbl-imagetool"
	$(CC) $(LDFLAGS) -o $@ $^

mtd-lock: mtd-lock.o
	@$(VECHO) LD "mtd-lock"
	$(CC) $(LDFLAGS) -o $@ $^

clean: libfdt_clean
	@$(VECHO) CLEAN "(mbbl-imagetool mtd-lock)"
	$(RM) mbbl-imagetool mtd-lock mbbl-imagetool.gdb mtd-lock.gdb $(STD_CLEANFILES)
