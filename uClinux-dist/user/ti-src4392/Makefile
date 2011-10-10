EXEC = src4392
OBJS = src4392.o
FLTFLAGS += -s 8192

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

romfs:
	$(ROMFSINST) /bin/$(EXEC)
	$(ROMFSINST) etc/rc/src4392 /etc/init.d/src4392
	$(ROMFSINST) -s /etc/init.d/src4392 /etc/rc.d/S32src4392
clean:
	-rm -f $(EXEC) *.elf *.gdb *.o

src4392.o: src4392.c

