CFLAGS += -DHAVE_CONFIG_H -I.. -I.

libdaemon_HEADERS = \
	dlog.h \
	dfork.h \
	dsignal.h \
	dnonblock.h \
	dpid.h \
	dexec.h \
	daemon.h

libdaemon_SOURCES = \
	dlog.c \
	dfork.c \
	dsignal.c \
	dnonblock.c \
	dpid.c \
	dexec.c \
	daemon.h \
	$(libdaemon_HEADERS)

libdaemon_OBJ = \
	dlog.o \
	dfork.o \
	dsignal.o \
	dnonblock.o \
	dpid.o \
	dexec.o


all: libdaemon.a

libdaemon.a: $(libdaemon_OBJ)
	$(AR) crs $@ $(libdaemon_OBJ)

romfs:

clean:
	$(RM) libdaemon.a *.o *.gdb *~