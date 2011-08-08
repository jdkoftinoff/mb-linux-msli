##########################

all:	mrd mwr

clean:
	@rm -f *.o mrd mwr

mrd:	mrd.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

mrd:	mrd.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

romfs:
	$(ROMFSINST) /bin/mrd
	$(ROMFSINST) /bin/mwr

