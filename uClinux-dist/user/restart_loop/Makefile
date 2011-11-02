EXEC = restart_loop
OBJS = restart_loop.o

FLTFLAGS += -s 32768

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $< $(LDLIBS)

romfs:
	$(ROMFSINST) /bin/$(EXEC)

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o

restart_loop.o: restart_loop.c

