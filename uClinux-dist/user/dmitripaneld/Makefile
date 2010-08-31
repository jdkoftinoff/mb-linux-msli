EXEC = dmitripaneld
OBJS = dmitripaneld.o
FLTFLAGS += -s 8192

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

romfs:
	$(ROMFSINST) /bin/$(EXEC)
	$(ROMFSINST) etc/rc/dmitripaneld /etc/init.d/dmitripaneld
	$(ROMFSINST) -s /etc/init.d/dmitripaneld /etc/rc.d/S31dmitripaneld
clean:
	-rm -f $(EXEC) *.elf *.gdb *.o

dmitripaneld.o: dmitripaneld.c

