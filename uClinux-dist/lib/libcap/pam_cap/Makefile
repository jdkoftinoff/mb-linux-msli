# simple make file for the pam_cap module

topdir=$(shell pwd)/..
include ../Make.Rules

LDLIBS += -lcap
CFLAGS += -fPIC

all: pam_cap.so
	$(MAKE) testcompile

install: all
	mkdir -p -m 0755 $(LIBDIR)/security
	install -m 0755 pam_cap.so $(LIBDIR)/security

pam_cap.so: pam_cap.o
	$(LD) $(LDFLAGS) -o pam_cap.so $< $(LDLIBS)

pam_cap.o: pam_cap.c
	$(CC) $(CFLAGS) -c $< -o $@

testcompile: test.c pam_cap.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $+ -lpam -ldl $(LDLIBS)

clean:
	rm -f *.o *.so testcompile *~
