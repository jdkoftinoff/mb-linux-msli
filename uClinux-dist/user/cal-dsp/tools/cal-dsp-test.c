#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <strings.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

int main( int US_UNUSED(argc), const char **argv )
{
    int file;
    bool r=false;
    int address;
    const char dev_file[] = "/dev/i2c-0";
    char buffer[0x100];
    file = open(dev_file, O_RDWR);
    if(file < 0) {
        printf("could not open %s\n",dev_file);
        exit(1);
    }
    address=0x42; /* write to third slot */
    if(ioctl(file,I2C_SLAVE,address) < 0) {
        printf("could not set slave address\n");
        close(file);
        exit(1);
    }
    strcpy(buffer,"j");
    if(write(file,buffer,strlen(buffer)) != strlen(buffer)) {
        printf("could not write to i2c\n");
        close(file);
        exit(1);
    }
#if 0
#endif
    printf("jiffy rules\n");
    close(file);
    return r;
}

