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
CFLAGS = -Wall -g -Os -Wpointer-arith -Wcast-qual

FLTFLAGS += -s 65536

all: env mtd-storage

env:
	env > env

romfs:
	$(ROMFSINST) /bin/mtd-storage
	$(ROMFSINST) /bin/persistent-save
	$(ROMFSINST) /bin/persistent-format

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

mtd-storage: mtd-storage.o crc32.o
	@$(VECHO) LD "mtd-storage"
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	@$(VECHO) CLEAN "(mtd-storage)"
	$(RM) mtd-storage mtd-storage.gdb $(STD_CLEANFILES)
