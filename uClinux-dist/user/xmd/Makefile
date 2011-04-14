##########################

all:	mrd mwr

clean:
	@rm -f *.o mrd mwr

mrd:	mrd.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)
	$(STRIP) $@
	
mrd:	mrd.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)
	$(STRIP) $@
	
romfs:
	$(ROMFSINST) /bin/mrd
	$(ROMFSINST) /bin/mwr

