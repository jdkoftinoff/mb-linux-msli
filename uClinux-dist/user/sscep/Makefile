#
# $Id: Makefile,v 1.0 2003/01/12 13:17:37 jt Exp $
#

BINDIR = /usr/local/bin
MANDIR = /usr/local/man/man8

#CC	= gcc
#WITH_DEBUG   = -g
CFLAGS	+= $(WITH_DEBUG)

MAN	= sscep.8
PROG	= sscep
OBJS    = sscep.o init.o net.o sceputils.o pkcs7.o ias.o fileutils.o vars.o

$(PROG): $(OBJS)
	$(CC) $(CFLAGS) -lcrypto -o $(PROG) $(OBJS) 

clean:
	rm -f $(PROG) $(OBJS) $(MAN) core

install:
	./install-sh $(PROG) $(BINDIR)
	./install-sh $(MAN) $(MANDIR)

romfs:
	$(ROMFSINST) /bin/$(PROG)
