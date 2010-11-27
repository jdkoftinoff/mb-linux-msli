EXEC = scpinstall
OBJS = scpinstall.o

FLTFLAGS += -s 65536

all: $(EXEC) installmonitor scpinstall-helper

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $< $(LDLIBS)

installmonitor: installmonitor.o
	$(CC) $(LDFLAGS) -o $@ $< $(LDLIBS)

scpinstall-helper: scpinstall-helper.o
	$(CC) $(LDFLAGS) -o $@ $< $(LDLIBS)

romfs:
	$(ROMFSINST) /bin/$(EXEC)
	$(ROMFSINST) -s /bin/scpinstall /bin/scp
	$(ROMFSINST) /bin/installmonitor
	$(ROMFSINST) /bin/scpinstall-helper

clean:
	-rm -f $(EXEC) installmonitor scpinstall-helper *.elf *.gdb *.o

scpinstall.o: scpinstall.c

installmonitor.o:installmonitor.c

scpinstall-helper.o: scpinstall-helper.c
