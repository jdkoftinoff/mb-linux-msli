EXEC = cal_oled
OBJS = cal_oled.o
FLTFLAGS += -s 8192

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

romfs:
	$(ROMFSINST) /bin/$(EXEC)
	$(ROMFSINST) etc/rc/cal_oled /etc/init.d/cal_oled
	$(ROMFSINST) -s /etc/init.d/cal_oled /etc/rc.d/S31cal_oled
clean:
	-rm -f $(EXEC) *.elf *.gdb *.o

cal_oled.o: cal_oled.c

