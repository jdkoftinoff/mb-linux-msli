#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>

void usage(void) {
	fprintf(stderr, "Usage: mwr <address> <value> [<words to write>]\n");
	return;
}

#define PAGESIZE 4096
#define PAGEMASK (PAGESIZE-1)

int main(int argc, char *argv[])
{
	int fd;
	uint32_t address;
	uint32_t val;
	char *endp;
	unsigned long int count;
	void *base;
	uint32_t *pmem;

	if (argc < 3) {
		usage();
		return 1;
	}
	address = strtoul(argv[1], &endp, 0);
	if (address == 0 || endp == argv[1]) {
		usage();
		return 1;
	}
	val = strtoul(argv[2], &endp, 0);
	if (endp == argv[1]) {
		usage();
		return 1;
	}
	if (argc < 4 || (count = strtoul(argv[3], &endp, 0)) == 0 || endp == argv[3]) {
		count = 1;
	}
	if ((fd = open("/dev/mem", O_WRONLY)) <= 0) {
		perror("Unable to open /dev/mem");
		exit errno;
	}
    base = mmap(0, PAGESIZE, PROT_WRITE, MAP_SHARED, fd, address & ~PAGEMASK);
    if (base == (void *)-1) {
    	perror("Unable to map memory");
    	close(fd);
    	return errno;
    }
    pmem = (uint32_t *)base + ((address & PAGEMASK) >> 2);
	while (count-- > 0) {
		*pmem++ = val;;
	}
	munmap(base, PAGESIZE);
	close(fd);
	return errno;
}
