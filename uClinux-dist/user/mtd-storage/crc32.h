#ifndef CRC32_H
#define CRC32_H
#include <sys/types.h>
unsigned long crc32(unsigned long crc,const unsigned char *buf,unsigned len);
unsigned long crc32_combine(unsigned long crc1,unsigned long crc2,off_t len2);
#endif
