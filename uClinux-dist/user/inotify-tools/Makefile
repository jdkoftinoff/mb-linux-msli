
ifeq (y,$(CONFIG_USER_INOTIFYTOOLS_WAIT))
PROGS  += inotifywait
endif
ifeq (y,$(CONFIG_USER_INOTIFYTOOLS_WAIT))
PROGS  += inotifywatch
endif

LIBS    = libinotifytools/libinotifytools.a

# Things missing because of port to ucLinux.
CFLAGS  = -DPACKAGE_BUGREPORT=\"yuriy.dragunov@labxtechnologies.com\" -DPACKAGE_VERSION=\"3.14\"

all: $(PROGS)

libinotifytools/libinotifytools.a:
	$(MAKE) -C libinotifytools

inotifywait: inotifywait.c common.o libinotifytools/libinotifytools.a
inotifywatch: inotifywatch.c common.o libinotifytools/libinotifytools.a

romfs:
ifeq (y,$(CONFIG_USER_INOTIFYTOOLS_WAIT))
	$(ROMFSINST) /bin/inotifywait
else
	-rm -f $(ROMFSDIR)/bin/inotifywait
endif
ifeq (y,$(CONFIG_USER_INOTIFYTOOLS_WATCH))
	$(ROMFSINST) /bin/inotifywatch
else
	-rm -f $(ROMFSDIR)/bin/inotifywatch
endif

clean:
	$(MAKE) -C libinotifytools clean
	-rm -f $(PROGS) *.elf *.gdb *.o

distclean:
	$(MAKE) -C libinotifytools distclean
	-rm -f $(PROGS) *.elf *.gdb *.o

