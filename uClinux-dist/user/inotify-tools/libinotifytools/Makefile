LIBS    = libinotifytools.a
CFLAGS += -Iinotifytools

all: $(LIBS)

libinotifytools.a: inotifytools.o redblack.o
	$(AR) $(ARFLAGS) $@ $^

%.c: %.o
	$(CC) $(CFLAGS) -o $@ $^

clean:
	-rm -f $(LIBS) *.o

distclean:
	-rm -f $(LIBS) *.o

