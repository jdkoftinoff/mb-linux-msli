INSTALL=install
libdir=$(ROOTDIR)/lib
includedir=$(ROOTDIR)/include

all:
	make -C libdaemon
	$(INSTALL) libdaemon/libdaemon.a $(libdir)
	$(INSTALL) libdaemon/daemon.h $(includedir)
	$(INSTALL) -d $(includedir)/libdaemon/
	$(INSTALL) libdaemon/*.h $(includedir)/libdaemon/

romfs:
	make -C libdaemon romfs

clean:
	make -C libdaemon clean
