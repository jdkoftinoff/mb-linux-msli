
PROGS   = inotifywait inotifywatch
LIBS    = libinotifytools/libinotifytools.a

# Things missing because of port to ucLinux.
CFLAGS  = -DPACKAGE_BUGREPORT=\"yuriy.dragunov@labxtechnologies.com\" -DPACKAGE_VERSION=\"3.14\"

all: $(LIBS) $(PROGS)

libinotifytools/libinotifytools.a:
	$(MAKE) -C libinotifytools

inotifywait: inotifywait.c common.o
ifeq (y,$(CONFIG_USER_INOTIFYTOOLS_WAIT))
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ inotifywait.c common.o $(LIBS)
endif

inotifywatch: inotifywatch.c common.o
ifeq (y,$(CONFIG_USER_INOTIFYTOOLS_WATCH))
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ inotifywatch.c common.o $(LIBS)
endif

romfs:
ifeq (y,$(CONFIG_USER_INOTIFYTOOLS_WAIT))
	$(ROMFSINST) /bin/inotifywait
endif
ifeq (y,$(CONFIG_USER_INOTIFYTOOLS_WATCH))
	$(ROMFSINST) /bin/inotifywatch
endif

clean:
	$(MAKE) -C libinotifytools clean
	-rm -f $(PROGS) *.elf *.gdb *.o

distclean:
	$(MAKE) -C libinotifytools distclean
	-rm -f $(PROGS) *.elf *.gdb *.o

