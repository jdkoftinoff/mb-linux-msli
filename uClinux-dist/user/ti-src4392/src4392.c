#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <asm/setup.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

/* SPI output -- data is sent to the OLED button */
#define AES_DEVICE_SPI "/dev/spidev_aes"

//CONVERTER_RSTn
#define AES_DEVICE_GPIO_RESET "/sys/class/gpio/gpio234/value"

/* persistent file handles */
static int spi_handle=-1;


/* send reset pulse */
int src4392_reset(void)
{
  int h;
  h=open(AES_DEVICE_GPIO_RESET,O_WRONLY);
  if(h < 0)
      return -1;

  write(h,"1\n",2);
  millisleep(500);
  write(h,"0\n",2);
  close(h);
  return 0;
}


/* sleep for a time given in milliseconds */
void millisleep(int t)
{
  struct timeval tv;
  tv.tv_sec=t/1000;
  tv.tv_usec=(t%1000)*1000;
  select(0,NULL,NULL,NULL,&tv);
}


/* close persistent file handles */
void src4392_close(void)
{
  if(spi_handle>=0)
    {
      close(spi_handle);
      spi_handle=-1;
    }
}





/* send data */
int src_spi_write(unsigned char *data, size_t len)
{
  static struct spi_ioc_transfer spi_transfer={
    .rx_buf=(unsigned long)NULL,
    .speed_hz=15000000, /* supposed to be redundant, but may be used
			  for delays */
    .delay_usecs=0, /* no delay between transfers */
    .bits_per_word=8
  };

  if(spi_handle < 0)
    {
      spi_handle=open(AES_DEVICE_SPI,O_RDWR);
      if(spi_handle < 0)
	      return -1;

    }
  spi_transfer.tx_buf=(unsigned long)data;
  while(len>0)
    {
      spi_transfer.len=(len>2048)?2048:len;
      if(ioctl(spi_handle,SPI_IOC_MESSAGE(1),&spi_transfer) <0)
	      return -1;

      spi_transfer.tx_buf+=spi_transfer.len;
      len-=spi_transfer.len;
    }
  return 0;
}

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* Yi: read data */
int src_spi_read(uint8_t reg )
{
  static uint8_t mode = 3;
  int ret = 0;
  uint8_t tx[] = {
		0xFF, 0xFF,0xFF
	};
	tx[0] = reg | 0x80;
	
	uint8_t rx[ARRAY_SIZE(tx)] = {0xFF, 0xFF,0xFF};
  static struct spi_ioc_transfer spi_transfer={
    .tx_buf=(unsigned long)NULL,
    .speed_hz=15000000, /* supposed to be redundant, but may be used
			  for delays */
    .delay_usecs=0, /* no delay between transfers */
    .bits_per_word=8
  };

  if(spi_handle < 0)
    {
      spi_handle=open(AES_DEVICE_SPI,O_RDWR);
      if(spi_handle < 0)
	  return -1;

    }
  ret = ioctl(spi_handle, SPI_IOC_WR_MODE, &mode);
	if (ret < 0)
		printf("can't set spi mode");
  
  spi_transfer.rx_buf=(unsigned long)rx;
  spi_transfer.tx_buf=(unsigned long)tx;
  spi_transfer.len = ARRAY_SIZE(tx);
  
  if(ioctl(spi_handle,SPI_IOC_MESSAGE(1),&spi_transfer) <0)
	  return -1;

	//printf("Yi Cao: register %x has value %x\n", reg, rx[2]);
  return 0;
}


/* initialize button (if bootloader didn't do it already) */
int src4392_init()
{
  
  if(src4392_reset())
	{
  	printf("Error: src4392 reset failed!\n");
	  src4392_close();
	  return -1;
	}
  
  /* Power down and reset, Power up sections DIR, DIT, SRC, Port A*/
  unsigned char data[3] = {0x01, 0x00, 0x37};
  src_spi_read(0x01);
  if (src_spi_write(data, 3))
  {
    printf("Error: src4392 spi write error!\n");
    return 1;  
  }
  millisleep(40);
  src_spi_read(0x01);
  
  /* Port A control 1,Select I2S, Slave Clock Mode, Set audio output source for SRC out */
  data[0] = 0x03;
  data[2] = 0x31;
  src_spi_read(0x03);
  if (src_spi_write(data, 3))
  {
    printf("Error: src4392 spi write error!\n");
    return 1;  
  }
  millisleep(40);
  src_spi_read(0x03);
  
  /* Transmitter control 1, Set MCLK Divider ration for DIT frame rate */
  data[0] = 0x07;
  data[2] = 0x20;
  src_spi_read(0x07);
  if (src_spi_write(data, 3))
  {
    printf("Error: src4392 spi write error!\n");
    return 1;  
  }
  millisleep(40);
  src_spi_read(0x07);
  
  /* PLL clock source */
  data[0] = 0x0D;
  data[2] = 0x08;
  src_spi_read(0x0D);
  if (src_spi_write(data, 3))
  {
    printf("Error: src4392 spi write error!\n");
    return 1;  
  }
  millisleep(40);
  src_spi_read(0x0D);
  
  /* SRC control 1, Set SRC input to DIR out */
  data[0] = 0x2D;
  data[2] = 0x02;
  src_spi_read(0x2D);
  if (src_spi_write(data, 3))
  {
    printf("Error: src4392 spi write error!\n");
    return 1;  
  }
  millisleep(40);
  src_spi_read(0x2D);
  
  /* Receiver PLL1 Config,Set Parameters for the DIR PLL */
  data[0] = 0x0F;
  data[2] = 0x12;
  src_spi_read(0x0F);
  if (src_spi_write(data, 3))
  {
    printf("Error: src4392 spi write error!\n");
    return 1;  
  }
  millisleep(40);
  src_spi_read(0x0F);
  
  /* Receiver PLL1 Config,(MCLK * J.D) / P = 98.304 MHz */
  data[0] = 0x10;
  data[2] = 0x00;
  src_spi_read(0x10);
  if (src_spi_write(data, 3))
  {
    printf("Error: src4392 spi write error!\n");
    return 1;  
  }
  millisleep(40);
  src_spi_read(0x10);
  
  /* Receiver PLL1 Config,MCLK - 12.288 MHz, J = 8, D = 0, P = 1 */
  data[0] = 0x11;
  data[2] = 0x00;
  src_spi_read(0x11);
  if (src_spi_write(data, 3))
  {
    printf("Error: src4392 spi write error!\n");
    return 1;  
  }
  millisleep(40);
  src_spi_read(0x11);
  
  
  printf("CAL AES set up complete!\n"); 
  
  return 0;
}


/* main() */

int main(int argc, char **argv)
{
  
  if(src4392_init())
    return 1;

  src4392_close();
  return 0;
}
