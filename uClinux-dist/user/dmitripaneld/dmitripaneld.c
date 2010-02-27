#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
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
#define OLED_BUTTON_DEVICE_SPI "/dev/spidev0"

/*
  GPIO bit: input reads button, output 0 resets OLED,
  output 1 enables normal operation
*/
#define OLED_BUTTON_DEVICE_GPIO_RESET_INPUT "/sys/class/gpio/gpio255/value"

/* GPIO bit: output 0 switches to command mode, 1 switches to data mode */
#define OLED_BUTTON_DEVICE_GPIO_DATA_COMMAND "/sys/class/gpio/gpio254/value"

/* kernel command line */
#define PROC_CMDLINE "/proc/cmdline"

/* logo image and font files */
#define OLED_BUTTON_LOGO_IMAGE "/etc/dmitripaneld/logo-1.bin"

#define OLED_FONT_8X12 \
  "/etc/dmitripaneld/bitstream-vera-sans-mono-8x12-font.bin"
#define OLED_FONT_16X24 "/etc/dmitripaneld/nixie-16x24-font.bin"
#define OLED_FONT_BOLD_16X24 "/etc/dmitripaneld/nixie-bold-16x24-font.bin"


/* data/command modes */
typedef enum {
  OLED_BUTTON_COMMAND = 0,
  OLED_BUTTON_DATA = 1,
  OLED_BUTTON_INVALID = 2
} oled_button_mode_t;


/* OLED button dimensions -- memory and screen area */
#define OLED_MEM_HEIGHT (64)
#define OLED_MEM_WIDTH (96)
#define OLED_MEM_DEPTH (2)
#define OLED_MEM_SIZE (OLED_MEM_HEIGHT \
		       * OLED_MEM_WIDTH \
		       * OLED_MEM_DEPTH)

#define OLED_SCREEN_HEIGHT (48)
#define OLED_SCREEN_WIDTH (64)
#define OLED_SCREEN_MEM_OFFSET (16)
#define OLED_SCREEN_SIZE (OLED_SCREEN_HEIGHT \
			  * OLED_SCREEN_WIDTH \
			  * OLED_MEM_DEPTH )

/* maximum image width for images created from text */
#define MAX_IMAGE_WIDTH (4096)

#define DISPLAY_OFF_TIMEOUT (60000)

/* fonts */
typedef struct
{
  int width;
  int height;
  int n;
  unsigned char data[0];
} oled_font_t;

/* image format */
typedef struct
{
  int width;
  int height;
  unsigned char data[0];
} oled_image_t;

/* fonts */
oled_font_t *font_16x24=NULL, *font_bold_16x24=NULL, *font_8x12=NULL;

/* persistent file handles */
static int spi_handle=-1, gpio_input_handle=-1, gpio_data_command_handle=-1;

/* OLED button init sequence */
static unsigned char oled_init_sequence[]={
  0x81, 0x19, 0x82, 0x14, 0x83, 0x24, 0x87, 0x0f,
  0xa0, 0x70, 0xa1, 0x00, 0xa2, 0x10, 0xa4, 0xa8,
  0x2f, 0xab, 0x00, 0x12, 0x0c, 0x14, 0x12, 0xad,
  0x8e, 0xb0, 0x0b, 0xb1, 0x44, 0xb3, 0xa0, 0xb9,
  0xbb, 0x12, 0xbe, 0x28, 0xae
};

/* colors */
/* background */
static unsigned char glob_rb=0x00, glob_gb=0x00, glob_bb=0x00;
/* error message */
static unsigned char err_r=0xff,err_g=0,err_b=0;
/* state indicator */
static unsigned char state_r=0xff,state_g=0xff,state_b=0xff;
/* message */
static unsigned char msg_r=0x20,msg_g=0xff,msg_b=0x10;

/* color type for progress bars */
typedef enum {
  PROGRESS_COLOR_GREEN,
  PROGRESS_COLOR_RED,
  PROGRESS_COLOR_YELLOW
} progress_color_t;

/*
  progress bar segments offsets for D-Mitri boot procedure:

| 0  0 0  0 1     1 1          3 3     3 4     4 4     5 5     6 |
| 1  4 6  9 1     7 9          0 2     8 0     6 8     4 6     2 |
| ==== ==== ======= ============ ======= ======= ======= ======= |

  percentage scale:

 0    10   20      30           50      60      70      80      100
 */

static unsigned char bar_offsets[]=
  {1,4,6,9,11,17,19,30,32,38,40,46,48,54,56,62};

/* load fixed-width grayscale font from a file */
oled_font_t *oled_load_font(char *name)
{
  int h,l,n;
  unsigned char font_dimensions[2];
  struct stat stbuf;
  oled_font_t *retval;
  
  h=open(name,O_RDONLY);
  if(h<0)
      return NULL;

  if(fstat(h,&stbuf)||stbuf.st_size<=0)
    {
      close(h);
      return NULL;
    }
  if(read(h,&font_dimensions,sizeof(font_dimensions))!=sizeof(font_dimensions))
    {
      close(h);
      return NULL;
    }
  l=(unsigned int)font_dimensions[0]*(unsigned int)font_dimensions[1];
  n=(stbuf.st_size-2)/l;
  if(l*n+2!=stbuf.st_size)
    {
      close(h);
      return NULL;
    }
  retval=(oled_font_t*)malloc(sizeof(oled_font_t)+l*n);
  if(retval==NULL)
    {
      close(h);
      return NULL;
    }
  retval->width=font_dimensions[0];
  retval->height=font_dimensions[1];
  retval->n=n;
  if(read(h,&retval->data[0],l*n)!=l*n)
    {
      free(retval);
      retval=NULL;
    }
  close(h);
  return retval;
}

/* sleep for a time given in milliseconds */
void millisleep(int t)
{
  struct timeval tv;
  tv.tv_sec=t/1000;
  tv.tv_usec=(t%1000)*1000;
  select(0,NULL,NULL,NULL,&tv);
}

/* start timer */
void start_timer(struct timeval *timer,int msec)
{
  unsigned long usec;
  gettimeofday(timer,NULL);
  usec=(unsigned long)timer->tv_usec+(unsigned long)msec*1000;
  timer->tv_sec+=usec/1000000; /* all timers expire on overflow */
  timer->tv_usec=usec%1000000;
}

/* check if timer expired */
int check_timer(struct timeval *timer)
{
  struct timeval currtime;
  gettimeofday(&currtime,NULL);
  return (currtime.tv_sec>timer->tv_sec)
    ||(currtime.tv_sec==timer->tv_sec && currtime.tv_usec>=timer->tv_usec);
}

/* wait for timer to expire */
void wait_for_timer(struct timeval *timer)
{
  struct timeval currtime,tv;
  long usecdiff,carry;
  gettimeofday(&currtime,NULL);
  if((currtime.tv_sec>timer->tv_sec)
     ||(currtime.tv_sec==timer->tv_sec && currtime.tv_usec>=timer->tv_usec))
    return;
  usecdiff=timer->tv_usec-currtime.tv_usec;
  if(usecdiff>=0)
    carry=usecdiff/1000000;
  else
    carry=usecdiff/1000000-1;

  tv.tv_usec=usecdiff-carry*1000000;
  tv.tv_sec=timer->tv_sec-currtime.tv_sec+carry;
  select(0,NULL,NULL,NULL,&tv);
}

/* close persistent file handles */
void oled_button_close(void)
{
  if(spi_handle>=0)
    {
      close(spi_handle);
      spi_handle=-1;
    }
  if(gpio_data_command_handle>=0)
    {
      close(gpio_data_command_handle);
      gpio_data_command_handle=-1;
    }
  if(gpio_input_handle>=0)
    {
      close(gpio_input_handle);
      gpio_input_handle=-1;
    }
}

/* send reset pulse */
int oled_button_reset(void)
{
  int h;
  h=open(OLED_BUTTON_DEVICE_GPIO_RESET_INPUT,O_WRONLY);
  if(h < 0)
      return -1;

  write(h,"0\n",2);
  millisleep(40);
  write(h,"1\n",2);
  close(h);
  return 0;
}


/* poll button state */
int oled_button_poll(void)
{
  char c='1';
  if(gpio_input_handle < 0)
    {
      gpio_input_handle=open(OLED_BUTTON_DEVICE_GPIO_RESET_INPUT,
			     O_RDONLY);
      if(gpio_input_handle < 0)
	  return -1;
    }
  if(read(gpio_input_handle,&c,1)!=1)
    {
      close(gpio_input_handle);
      gpio_input_handle=-1;
      return -1;
    }
  lseek(gpio_input_handle,0,SEEK_SET);
  return c=='0';
}


/* set command or data mode */
int oled_button_mode(oled_button_mode_t mode)
{
  static oled_button_mode_t current_mode = OLED_BUTTON_INVALID;
  static unsigned char command[2]="0\n";
  if(mode == current_mode)
      return 0;

  if(gpio_data_command_handle < 0)
    {
      gpio_data_command_handle=open(OLED_BUTTON_DEVICE_GPIO_DATA_COMMAND,
				    O_WRONLY);
      if(gpio_data_command_handle < 0)
	  return -1;

    }
  command[0]=mode?'1':'0';
  lseek(gpio_input_handle,0,SEEK_SET);
  if(write(gpio_data_command_handle,command,1)!=1)
    {
      close(gpio_data_command_handle);
      gpio_data_command_handle=-1;
      return -1;
    }
  current_mode=mode;
  return 0;
}


/* send data */
int oled_spi_write(unsigned char *data, size_t len)
{
  static struct spi_ioc_transfer spi_transfer={
    .rx_buf=(unsigned long)NULL,
    .speed_hz=4000000, /* supposed to be redundant, but may be used
			  for delays */
    .delay_usecs=0, /* no delay between transfers */
    .bits_per_word=8
  };

  if(spi_handle < 0)
    {
      spi_handle=open(OLED_BUTTON_DEVICE_SPI,O_RDWR);
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


/* initialize button (if bootloader didn't do it already) */
int oled_button_init(int clear)
{
  if(oled_button_mode(OLED_BUTTON_COMMAND))
    {
      oled_button_close();
      return -1;
    }
  if(clear)
    {
      if(oled_button_reset())
	{
	  oled_button_close();
	  return -1;
	}
      oled_init_sequence[sizeof(oled_init_sequence)-1]=0xae;
    }
  else
    oled_init_sequence[sizeof(oled_init_sequence)-1]=0xaf;

  if(oled_spi_write(oled_init_sequence,sizeof(oled_init_sequence)))
    {
      oled_button_close();
      return -1;
    }
  return 0;
}


/*
  clear button memory with given color
  (including buffer beyond displayed area)
*/
void oled_button_clear(unsigned char red,
		       unsigned char green,
		       unsigned char blue)
{
  unsigned char command_buffer[]=
    {
      0x26, 0x01, 0x22, 0x00, 0x00, 0x5f, 0x3f, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00
    };

  command_buffer[7]=command_buffer[10]=(blue>>2)&0x3e;
  command_buffer[8]=command_buffer[11]=(green>>2)&0x3f;
  command_buffer[9]=command_buffer[12]=(red>>2)&0x3e;

  /* if a handle to SPI device is not open yet, it means that
     the device is not initialized by now */

  if(spi_handle < 0)
    {
      if(oled_button_init(0))
	return;

    }
  oled_button_mode(OLED_BUTTON_COMMAND);
  oled_spi_write(command_buffer,sizeof(command_buffer));
}

/* draw filled box */
void oled_draw_box(int x, int y, int width, int height,
		   unsigned char r,
		   unsigned char g,
		   unsigned char b)
{
  unsigned char command[]=
    {
      0x26, 0x01,
      0x22, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
  
  command[3]=x;
  command[4]=y;
  command[5]=x+width-1;
  command[6]=y+height-1;
  command[7]=command[10]=(b>>2)&0x3e;
  command[8]=command[11]=(g>>2)&0x3f;
  command[9]=command[12]=(r>>2)&0x3e;
  oled_spi_write(command,sizeof(command));
}

/* copy rectangle */
void oled_copy_rectangle(int x_dst, int y_dst, int x_src, int y_src,
			 int width, int height)
{
  unsigned char command[]=
    {
      0x26, 0x01,
      0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
  
  command[3]=x_src;
  command[4]=y_src;
  command[5]=x_src+width-1;
  command[6]=y_src+height-1;
  command[7]=x_dst;
  command[8]=y_dst;
  oled_spi_write(command,sizeof(command));
}

/*
  send a rectangle filled with image data
  (all coordinates are  relative to the memory area)
 */
int oled_send_rectangle(unsigned char *src, int src_line_length,
			int dst_x, int dst_y, int dst_width, int dst_height)
{
  unsigned char command_buffer[]={ 0x15, 0x10, 0x4f, 0x75, 0x00, 0x2f };
  
  /* if a handle to SPI device is not open yet, it means that
     the device is not initialized by now */
  int i;
  if(spi_handle < 0)
    {
      if(oled_button_init(0))
	return 1;

    }
  command_buffer[1]=dst_x;
  command_buffer[2]=dst_x+dst_width-1;
  command_buffer[4]=dst_y;
  command_buffer[5]=dst_y+dst_height-1;

  oled_button_mode(OLED_BUTTON_COMMAND);
  oled_spi_write(command_buffer,sizeof(command_buffer));
  oled_button_mode(OLED_BUTTON_DATA);

  if(dst_width == src_line_length)
    oled_spi_write(src,src_line_length*dst_height*OLED_MEM_DEPTH);
  else
    for(i=0;i<dst_height;i++)
      {
	oled_spi_write(src,dst_width*OLED_MEM_DEPTH);
	src+=src_line_length*OLED_MEM_DEPTH;
      }
  
  oled_button_mode(OLED_BUTTON_COMMAND);
  return 0;
}


/* send image (with clipping, all coordinates are relative to screen) */
int oled_send_image(oled_image_t *image,int screen_x,int screen_y)
{
  int mem_offset_x,mem_offset_y,image_offset_x,image_offset_y,
    copy_width,copy_height;
  
  /* determine offsets for copying */
  mem_offset_x=screen_x+OLED_SCREEN_MEM_OFFSET;
  mem_offset_y=screen_y;
  image_offset_x=0;
  image_offset_y=0;
  
  if(mem_offset_x>=OLED_MEM_WIDTH || mem_offset_y>=OLED_MEM_HEIGHT)
    return 0;
  
  if(mem_offset_x<0)
    {
      image_offset_x-=mem_offset_x;
      mem_offset_x=0;
    }

  if(image_offset_x>=image->width)
    return 0;

  if(mem_offset_y<0)
    {
      image_offset_y-=mem_offset_y;
      mem_offset_y=0;
    }

  if(image_offset_y>=image->height)
      return 0;

  copy_width=image->width-image_offset_x;
  if(mem_offset_x+copy_width>OLED_MEM_WIDTH)
    copy_width=OLED_MEM_WIDTH-mem_offset_x;

  copy_height=image->height-image_offset_y;
  if(mem_offset_y+copy_height>OLED_MEM_HEIGHT)
    copy_height=OLED_MEM_HEIGHT-mem_offset_y;

  return oled_send_rectangle((&image->data[0])
			     +(image_offset_x
			       +image_offset_y*image->width)*OLED_MEM_DEPTH,
			     image->width,
			     mem_offset_x,mem_offset_y,copy_width,copy_height);
}

/* 
   send image_fragment (with clipping, all coordinates are
   relative to screen)
*/
int oled_send_image_fragment(oled_image_t *image,
			     int image_x,int image_y,
			     int image_width,int image_height,
			     int screen_x,int screen_y)
{
  int mem_offset_x,mem_offset_y,image_offset_x,image_offset_y;
  
  if(image_x<0||image_x>=image->width||image_y<0||image_y>=image_height)
    return 0;

  /* determine offsets for copying */
  mem_offset_x=screen_x+OLED_SCREEN_MEM_OFFSET;
  mem_offset_y=screen_y;
  image_offset_x=image_x;
  image_offset_y=image_y;

  if(mem_offset_x>=OLED_MEM_WIDTH || mem_offset_y>=OLED_MEM_HEIGHT)
    return 0;
  
  if(image_width>image->width-image_x)
    image_width=image->width-image_x;

  if(image_height>image->height-image_y)
    image_height=image->height-image_y;

  if(mem_offset_x<0)
    {
      image_offset_x-=mem_offset_x;
      image_width+=mem_offset_x;
      mem_offset_x=0;
    }

  if(image_offset_x>=image->width)
      return 0;

  if(mem_offset_y<0)
    {
      image_offset_y-=mem_offset_y;
      image_height+=mem_offset_y;
      mem_offset_y=0;
    }

  if(image_offset_y>=image->height)
      return 0;

  if(mem_offset_x+image_width>OLED_MEM_WIDTH)
    image_width=OLED_MEM_WIDTH-mem_offset_x;

  if(mem_offset_y+image_height>OLED_MEM_HEIGHT)
    image_height=OLED_MEM_HEIGHT-mem_offset_y;

  return oled_send_rectangle((&image->data[0])
			     +(image_offset_x
			       +image_offset_y*image->width)*OLED_MEM_DEPTH,
			     image->width,
			     mem_offset_x,mem_offset_y,
			     image_width,image_height);
}


/*
  draw character in the image buffer
  (no clipping, only display characters that fit into the image completely)
*/
static inline int oled_draw_character(oled_image_t *image,
				      int x, int y,
				      oled_font_t *font,
				      unsigned char c,
				      unsigned char r,
				      unsigned char g,
				      unsigned char b,
				      unsigned char rb,
				      unsigned char gb,
				      unsigned char bb)
{
  int ff;
  unsigned char *fontptr, *imageptr, r1, g1, b1;
  unsigned int i,j;
  /* no clipping, only display characters that fit into the image */
  if(x<0||y<0
     ||x+font->width>image->width
     ||y+font->height>image->height)
    return -1;
  
  /* printable characters present in the given font */
  if(c<' ')
    c=0;
  else
    c-=' ';
  
  if(c>=font->n)
    c=0;
  
  fontptr=(&font->data[0])+font->height*font->width*(unsigned int)c;
  imageptr=(&image->data[0])+(x+image->width*y)*OLED_MEM_DEPTH;

  for(i=0;i<font->height;i++)
    {
      for(j=0;j<font->width;j++)
	{
	  ff=(int)*fontptr;
	  r1=(unsigned char)((int)rb+((((int)r-(int)rb)*ff)>>8));
	  g1=(unsigned char)((int)gb+((((int)g-(int)gb)*ff)>>8));
	  b1=(unsigned char)((int)bb+((((int)b-(int)bb)*ff)>>8));
	  imageptr[1]=(r1>>3)|(g1>>2<<5);
	  imageptr[0]=(g1>>5)|(b1>>3<<3);
	  imageptr+=OLED_MEM_DEPTH;
	  fontptr++;
	}
      imageptr+=(image->width-font->width)*OLED_MEM_DEPTH;
    }
  return 0;
}

/* draw text in the image buffer */
static int oled_draw_text(oled_image_t *image,
			  int x, int y,
			  oled_font_t *font,
			  char *s,
			  unsigned char r,
			  unsigned char g,
			  unsigned char b,
			  unsigned char rb,
			  unsigned char gb,
			  unsigned char bb)
{
  int retval=0;
  while(y<image->height&&*s)
    {
      retval|=oled_draw_character(image,x,y,font,(unsigned char)*s,r,g,b,
				  rb,gb,bb);
      x+=font->width;
      s++;
      if(x>=image->width)
	{
	  x=0;
	  y+=font->height;
	}
    }
  return retval;
}

/* allocate and clear memory for an image */
oled_image_t *oled_create_image(int width,int height,
				unsigned char rb,
				unsigned char gb,
				unsigned char bb)
{
  oled_image_t *retval;
  unsigned char byte0,byte1,*ptr,*endptr;
  retval=(oled_image_t*)malloc(sizeof(oled_image_t)
			       +width*height*OLED_MEM_DEPTH);
  if(!retval)
      return NULL;

  byte1=(rb>>3)|(gb>>2<<5);
  byte0=(gb>>5)|(bb>>3<<3);

  retval->width=width;
  retval->height=height;
  ptr=&retval->data[0];
  endptr=ptr+width*height*OLED_MEM_DEPTH;
  while(ptr<endptr)
    {
      *(ptr++)=byte0;
      *(ptr++)=byte1;
    }
  return retval;
}

/* create image that contains text */
oled_image_t *oled_create_text_image(oled_font_t *font,
				     char *s,
				     unsigned char r,
				     unsigned char g,
				     unsigned char b,
				     unsigned char rb,
				     unsigned char gb,
				     unsigned char bb)
{
  int l;
  oled_image_t *image;
  l=strlen(s)*font->width;
  if(!l)
    return NULL;
  if(l>MAX_IMAGE_WIDTH) l=MAX_IMAGE_WIDTH;
  image=(oled_image_t*)malloc(sizeof(oled_image_t)
			      +l*font->height*OLED_MEM_DEPTH);
  if(!image)
    return NULL;
  image->width=l;
  image->height=font->height;
  oled_draw_text(image,0,0,font,s,r,g,b,rb,gb,bb);
  return image;
}



/*
  start scrolling
  (all coordinates are  relative to the memory area)
*/
int oled_scroll_start(int y, int height, int scroll_x, int scroll_y)
{
  unsigned char command_buffer[]={ 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2f };
  
  command_buffer[1]=scroll_x;
  command_buffer[2]=y;
  command_buffer[3]=height;
  command_buffer[4]=scroll_y;

  oled_button_mode(OLED_BUTTON_COMMAND);
  oled_spi_write(command_buffer,sizeof(command_buffer));
  return 0;
}

/* stop scrolling */

void oled_scroll_stop(void)
{
  unsigned char cmd;
  oled_button_mode(OLED_BUTTON_COMMAND);
  cmd=0x2e;
  oled_spi_write(&cmd,1);
}

/* display image from file in raw format */
int oled_display_image(unsigned char x, unsigned char y, char *name)
{
  int h,l;
  unsigned char command_buffer[]={ 0x15, 0x10, 0x4f, 0x75, 0x00, 0x2f };
  static unsigned char filebuffer[1024];

  /* if a handle to SPI device is not open yet, it means that
     the device is not initialized by now */

  if(spi_handle < 0 && oled_button_init(0))
    return 1;

  h=open(name,O_RDONLY);
  if(h<0)
      return -1;

  /* write into a visible screen area */
  l=read(h,filebuffer,sizeof(filebuffer));
  if(l>3)
    {
      /* set dimensions */
      command_buffer[1]=x;
      command_buffer[2]=x+filebuffer[0]-1;
      command_buffer[4]=y;
      command_buffer[5]=y+filebuffer[1]-1;
      oled_button_mode(OLED_BUTTON_COMMAND);
      oled_spi_write(command_buffer,6);
      
      /* write remaining data from the first buffer */
      oled_button_mode(OLED_BUTTON_DATA);
      oled_spi_write(filebuffer+3,l-3);
      
      /* copy all data from the file */
      while((l=read(h,filebuffer,sizeof(filebuffer)))>0)
	oled_spi_write(filebuffer,l);

      oled_button_mode(OLED_BUTTON_COMMAND);
    }
  close(h);
  return (l<0);
}

/* display a progress bar segment */
int oled_button_display_progress(int n,progress_color_t color)
{
  unsigned char command[]={
#ifdef OLD_SCREEN
    0x26, 0x01,
    0x22, 0x00, 0x00, 0x00, 0x00,
    0xe0, 0xe0, 0xe0, 0x40, 0xf0, 0x40
#else
    0x26, 0x01,
    0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3f, 0x00, 0x00, 0x3f, 0x00
  };
#endif
  
  unsigned char err_cmd[]={
    0x26, 0x01,
    0x22, 0x10, 0x00, 0x4f, 0x0b,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  
  if(n<1) n=1;
  else if(n>8) n=8;
  
  command[3]=16+bar_offsets[n*2-2];
  command[4]=38;
  command[5]=16+bar_offsets[n*2-1];
  command[6]=43;
  
#if 0	
  command[7]=0xe0; /*b*/
  command[8]=0xe0; /*g*/
  command[9]=0xe0; /*r*/
  command[10]=0x40; /*b*/
  command[11]=0xf0; /*g*/
  command[12]=0x40; /*r*/
#endif
  
  switch(color)
    {
    case PROGRESS_COLOR_RED:
      command[7]=0x00; /*b*/
      command[8]=0x00; /*g*/
      command[9]=0x3e; /*r*/
      command[10]=0x00; /*b*/
      command[11]=0x00; /*g*/
      command[12]=0x3e; /*r*/
      break;
      
    case PROGRESS_COLOR_YELLOW:
      command[7]=0x00; /*b*/
      command[8]=0x3f; /*g*/
      command[9]=0x3e; /*r*/
      command[10]=0x00; /*b*/
      command[11]=0x3f; /*g*/
      command[12]=0x3e; /*r*/
      break;
      
    default:
      break;
    }
  
  /* if a handle to SPI device is not open yet, it means that
     the device is not initialized by now */
  
  if(spi_handle < 0 && oled_button_init(0))
    return 1;
  
  oled_button_mode(OLED_BUTTON_COMMAND);
  oled_scroll_stop();
  if(color!=PROGRESS_COLOR_GREEN)
    oled_spi_write(err_cmd,sizeof(err_cmd));
  
  oled_spi_write(command,sizeof(command));
  
  /* this version does not update the percentage indicator */
  
  return 0;
}

static oled_image_t *text_image_err=NULL;

/* display short error message */
int platform_display_error(char *msg)
{
  oled_scroll_stop();
  if(!text_image_err)
      text_image_err=oled_create_image(64,12,glob_rb,glob_gb,glob_bb);

  if(text_image_err && font_8x12)
    {
      oled_draw_text(text_image_err,0,0,font_8x12,msg,
		     err_r,err_g,err_b,
		     glob_rb,glob_gb,glob_bb);
      oled_send_image(text_image_err,0,0);
    }
  return 0;
}

/* turn on the button screen */

void oled_button_on(void)
{
  unsigned char cmd;
  oled_button_mode(OLED_BUTTON_COMMAND);
  cmd=0xaf;
  oled_spi_write(&cmd,1);
}


/* turn off the button screen */

void oled_button_off(void)
{
  unsigned char cmd;
  oled_button_mode(OLED_BUTTON_COMMAND);
  cmd=0xae;
  oled_spi_write(&cmd,1);
}

/* flash the button bright white */

void oled_button_flash(void)
{
  unsigned char cmd;
  oled_button_mode(OLED_BUTTON_COMMAND);
  cmd=0xa5;
  oled_spi_write(&cmd,1);
  millisleep(150);
  cmd=0xa4;
  oled_spi_write(&cmd,1);
}
 

static inline unsigned char unhex(unsigned char c)
{
  if(c>'9')
    {
      c&=0xff-('a'-'A');
      c-='A'-'9'-1;
    }
  return c-'0';
}
 
/* get eth0 ip address */
 
char *my_eth_address(char *interface,int ipversion)
{
  static char name[47]="";
  char ifname[20];
  unsigned char *addrptr;
  int h,i,addr[16],param1,param2,param3,param4;
  static FILE *f=NULL;
  struct ifreq ifr;

  name[0]='\0';
  if(ipversion!=4&&ipversion!=6)
    return name;
  
  if(ipversion==4)
    {
      h=socket(AF_INET,SOCK_DGRAM,0);
      
      if(h<0)
	return name;
      
      for(i=-1;i<16;i++)
	{
	  if(i<0)
	    snprintf(ifr.ifr_name,IFNAMSIZ,"%s",interface);
	  else
	    snprintf(ifr.ifr_name,IFNAMSIZ,"%s:%d",interface,i);
	  if(!ioctl(h,SIOCGIFADDR,(caddr_t)&ifr,sizeof(ifr)))
	    {
	      inet_ntop(AF_INET,(struct in_addr*)
			&(((struct sockaddr_in*)&(ifr.ifr_addr))->sin_addr),
			name,sizeof(name)-1);
	      if(*name)
		{
		  close(h);
		  return name;
		}
	    }
	}
      close(h);
    }
  else
    {
      if(!f)
	f=fopen("/proc/net/if_inet6","r");
      if(!f)
	return name;
      fseek(f,0,SEEK_SET);
      addrptr=(unsigned char*)
	&((struct sockaddr_in6*)&(ifr.ifr_addr))->sin6_addr;
      while(fscanf(f,"%02x%02x%02x%02x%02x%02x%02x%02x"
		   "%02x%02x%02x%02x%02x%02x%02x%02x "
		   "%08x %02x %02x %02x %20s\n",
		   addr+0,addr+1,addr+2,addr+3,
		   addr+4,addr+5,addr+6,addr+7,
		   addr+8,addr+9,addr+10,addr+11,
		   addr+12,addr+13,addr+14,addr+15,
		   &param1,&param2,&param3,&param4,ifname)==21)
	{
	  if(!strcmp(ifname,interface))
	    {
	      for(i=0;i<16;i++)
		addrptr[i]=(unsigned char)addr[i];
	      
	      inet_ntop(AF_INET6,(struct in6_addr*)
			&(((struct sockaddr_in6*)&(ifr.ifr_addr))->sin6_addr),
			name,sizeof(name)-1);
	      if(*name)
		{
		  return name;
		}
	    }
	}
    }
  return name;
}

int read_buttons_cmdline(void)
{
  int h,l,n=0;
  unsigned char buffer[COMMAND_LINE_SIZE+1],*ptr0,*ptr1;
   h=open(PROC_CMDLINE,O_RDONLY);
   if(h<0)
       return 0;

   l=read(h,buffer,COMMAND_LINE_SIZE);
   close(h);
   if(l<=0)
     return 0;

   buffer[l]='\0';
   ptr0=buffer;
   while(*ptr0)
     {
       for(;*ptr0&&*ptr0<=' ';ptr0++);
       for(ptr1=ptr0;*ptr1>' '&&*ptr1!='=';ptr1++);
       if(ptr1-ptr0==7&&!memcmp(ptr0,"BUTTONS",7))
	 {
	   for(;*ptr1>' '&&(*ptr1<'0'||*ptr1>'9');ptr1++);
	   sscanf(ptr1,"%i",&n);
	   return n;
	 }
       else
	 for(ptr0=ptr1;*ptr0&&*ptr0>' ';ptr0++);
     }
   return 0;
 }

static int vert_shift=0,horiz_shift=0;

void shift_change(void)
{
  static int vert_shift_direction=1,horiz_shift_direction=1;

  if(vert_shift_direction<0 && vert_shift==0)
    vert_shift_direction=1;
  else
    if(vert_shift_direction>0 && vert_shift==3)
      vert_shift_direction=-1;

  if(horiz_shift_direction<0 && horiz_shift==0)
    horiz_shift_direction=1;
  else
    if(horiz_shift_direction>0 && horiz_shift==4)
      horiz_shift_direction=-1;

  vert_shift+=vert_shift_direction;
  horiz_shift+=horiz_shift_direction;

  oled_draw_box(16+64,0, 16, 64,
		glob_rb,glob_gb,glob_bb);
}

/* main() */

int main(int argc, char **argv)
{
  oled_image_t *text_image,*ipaddr_label_0=NULL,*ipaddr_label_1=NULL;
  int i,n0,n1,network_on=0,maint_flag=0,counter,
    label_width,label_width1,
    ipaddr_update_flag_0=0,ipaddr_update_flag_1=0,
    display_off_flag=0,
    ipaddr_label_offset=0;
  char *ipaddr;
  char strbuffer_line1[256],strbuffer_line2[256],
    ipv4addr_0[47],ipv6addr_0[47],
    ipv4addr_1[47],ipv6addr_1[47];
  struct timeval loop_timer,display_off_timer;

  enum{
    DISPLAY_ETH0,
    DISPLAY_ETH1,
    DISPLAY_STATE_INVALID
  } display_state=DISPLAY_ETH0,prev_display_state=DISPLAY_STATE_INVALID;

  if(oled_button_init(0))
    return 1;

  maint_flag=read_buttons_cmdline()&1;
  if(maint_flag)
    {
      /* change colors to acknowledge */
      glob_rb=0x20;
      glob_gb=0x00;
      glob_bb=0x00;
      msg_r=0xff;
      msg_g=0xff;
      msg_b=0x40;
    }

  /*
  oled_scroll_stop();
  oled_button_clear(255,255,255);
  oled_display_image(0x10,0,OLED_BUTTON_LOGO_IMAGE);
  */

  font_8x12=oled_load_font(OLED_FONT_8X12);
  font_16x24=oled_load_font(OLED_FONT_16X24);
  font_bold_16x24=oled_load_font(OLED_FONT_BOLD_16X24);
  for(i=1;i<=7;i++)
    oled_button_display_progress(i,PROGRESS_COLOR_GREEN);
  
  text_image=oled_create_image(96,38,glob_rb,glob_gb,glob_bb);
  
  if(font_16x24&&font_bold_16x24&&font_8x12&&text_image)
    {
      oled_draw_text(text_image,16+0,0,font_8x12,"NETWORK:",
		     state_r,state_g,state_b,
		     glob_rb,glob_gb,glob_bb);
      
      oled_draw_text(text_image,16+8,13,font_bold_16x24,"80%",
		     msg_r,msg_g,msg_b,
		     glob_rb,glob_gb,glob_bb);
		     
      oled_send_image(text_image,-16,0);
    }
  
  oled_scroll_start(0,12,0x5e,0);
  oled_button_on();

  display_off_flag=0;
  start_timer(&display_off_timer,DISPLAY_OFF_TIMEOUT);

  counter=0;
  n0=2;
  while(1)
    {
      start_timer(&loop_timer,40);
      if(!display_off_flag)
	{
	  if(check_timer(&display_off_timer))
	    {
	      display_off_flag=1;
	      oled_button_off();
	    }
	}

      /* process button */
      n1=oled_button_poll();
      if(n0!=n1)
	{
	  n0=n1;
	  if(n1)
	    {
	      if(display_off_flag)
		oled_button_on();
	      
	      oled_button_flash();
	      display_off_flag=0;
	      start_timer(&display_off_timer,DISPLAY_OFF_TIMEOUT);
	    }
	}

      if(!network_on)
	{
	  /* check if any network interface got an ipv4 address */
	  ipaddr=my_eth_address("eth0",4);
	  if(!*ipaddr)
	    ipaddr=my_eth_address("eth1",4);
	  if(*ipaddr)
	    {
	      ipv4addr_0[0]='\0';
	      ipv6addr_0[0]='\0';
	      ipv4addr_1[0]='\0';
	      ipv6addr_1[0]='\0';
	      network_on=1;
	      oled_scroll_stop();
	      oled_button_display_progress(8,PROGRESS_COLOR_GREEN);
	      if(text_image && font_bold_16x24)
		{
		  oled_draw_text(text_image,16+0,13,font_bold_16x24,"100%",
				 msg_r,msg_g,msg_b,
				 glob_rb,glob_gb,glob_bb);
		  
		  oled_send_image(text_image,-16,0);
		}
	    }
	}
      else
	{
	  /* 
	     once network is up, check for interface addresses changes
	     and display them
	  */
	  ipaddr_update_flag_0=0;
	  ipaddr_update_flag_1=0;
	  if(ipaddr_label_offset==64||counter<=0)
	    {
	      counter=128;
	      
	      ipaddr=my_eth_address("eth0",4);
	      if(strcmp(ipaddr,ipv4addr_0))
		{
		  ipaddr_update_flag_0=1;
		  strncpy(ipv4addr_0,ipaddr,47);
		  ipv4addr_0[46]='\0';
		}
	      ipaddr=my_eth_address("eth0",6);
	      if(strcmp(ipaddr,ipv6addr_0))
		{
		  ipaddr_update_flag_0=1;
		  strncpy(ipv6addr_0,ipaddr,47);
		  ipv6addr_0[46]='\0';
		}
	      ipaddr=my_eth_address("eth1",4);
	      if(strcmp(ipaddr,ipv4addr_1))
		{
		  ipaddr_update_flag_1=1;
		  strncpy(ipv4addr_1,ipaddr,47);
		  ipv4addr_1[46]='\0';
		}
	      ipaddr=my_eth_address("eth1",6);
	      if(strcmp(ipaddr,ipv6addr_1))
		{
		  ipaddr_update_flag_1=1;
		  strncpy(ipv6addr_1,ipaddr,47);
		  ipv6addr_1[46]='\0';
		}
	      
	      if(ipaddr_update_flag_0&&ipaddr_label_0)
		{
		  free(ipaddr_label_0);
		  ipaddr_label_0=NULL;
		}
	      if(ipaddr_update_flag_1&&ipaddr_label_1)
		{
		  free(ipaddr_label_1);
		  ipaddr_label_1=NULL;
		}
	      
	      if(ipaddr_update_flag_0||ipaddr_update_flag_1)
		{
		  if(display_off_flag)
		    oled_button_on();
		  display_off_flag=0;
		  start_timer(&display_off_timer,DISPLAY_OFF_TIMEOUT);
		}
	    }
	  counter--;

	  if(!ipaddr_label_0)
	    {
	      if(font_bold_16x24&&font_8x12)
		{
		  if(*ipv6addr_0)
		    {
		      snprintf(strbuffer_line1,sizeof(strbuffer_line1),
			       " IPv6: %s  ",ipv6addr_0);
		    }
		  else
		    {
		      strbuffer_line1[0]='\0';
		    }
		  if(*ipv4addr_0)
		    {
		      snprintf(strbuffer_line2,sizeof(strbuffer_line2),
			       "IP:%s ",ipv4addr_0);
		    }
		  else
		    {
		      strcpy(strbuffer_line2,"NO IP ADDRESS ");
		    }
		  label_width=strlen(strbuffer_line1)*font_8x12->width;
		  label_width1=strlen(strbuffer_line2)*font_bold_16x24->width;
		  if(label_width<label_width1)
		    label_width=label_width1;

		  ipaddr_label_0=oled_create_image(label_width,
						 font_8x12->height
						 +font_bold_16x24->height+2,
						 glob_rb,glob_gb,glob_bb);
		  if(ipaddr_label_0)
		    {
		      oled_draw_text(ipaddr_label_0,0,0,
				     font_8x12,
				     strbuffer_line1,msg_r,msg_g,msg_b,
				     glob_rb,glob_gb,glob_bb);
		      oled_draw_text(ipaddr_label_0,0,font_8x12->height+2,
				     font_bold_16x24,
				     strbuffer_line2,msg_r,msg_g,msg_b,
				     glob_rb,glob_gb,glob_bb);
		    }
		  ipaddr_label_offset=64;
		}

	      oled_button_clear(glob_rb,glob_gb,glob_bb);
	      prev_display_state=DISPLAY_STATE_INVALID;
	    }

	  if(!ipaddr_label_1)
	    {
	      if(font_bold_16x24&&font_8x12)
		{
		  if(*ipv6addr_1)
		    {
		      snprintf(strbuffer_line1,sizeof(strbuffer_line1),
			       " IPv6: %s  ",ipv6addr_1);
		    }
		  else
		    {
		      strbuffer_line1[0]='\0';
		    }
		  if(*ipv4addr_1)
		    {
		      snprintf(strbuffer_line2,sizeof(strbuffer_line2),
			       "IP:%s ",ipv4addr_1);
		    }
		  else
		    {
		      strcpy(strbuffer_line2,"NO IP ADDRESS ");
		    }
		  label_width=strlen(strbuffer_line1)*font_8x12->width;
		  label_width1=strlen(strbuffer_line2)*font_bold_16x24->width;
		  if(label_width<label_width1)
		    label_width=label_width1;

		  ipaddr_label_1=oled_create_image(label_width,
						 font_8x12->height
						 +font_bold_16x24->height+2,
						 glob_rb,glob_gb,glob_bb);
		  if(ipaddr_label_1)
		    {
		      oled_draw_text(ipaddr_label_1,0,0,
				     font_8x12,
				     strbuffer_line1,msg_r,msg_g,msg_b,
				     glob_rb,glob_gb,glob_bb);
		      oled_draw_text(ipaddr_label_1,0,font_8x12->height+2,
				     font_bold_16x24,
				     strbuffer_line2,msg_r,msg_g,msg_b,
				     glob_rb,glob_gb,glob_bb);
		    }
		  ipaddr_label_offset=64;
		}

	      oled_button_clear(glob_rb,glob_gb,glob_bb);
	      prev_display_state=DISPLAY_STATE_INVALID;
	    }

	  switch(display_state)
	    {
	    case DISPLAY_ETH0:
	      if(ipaddr_label_0)
		{

		  if(vert_shift==0)
		    oled_copy_rectangle(16,0,16+2,0,
					64+16-2,ipaddr_label_1->height+1);
		  else
		    oled_copy_rectangle(16,vert_shift-1,16+2,vert_shift-1,
					64+16-2,ipaddr_label_1->height+2);
		  if((ipaddr_label_offset&0x0f)==0)
		    oled_send_image_fragment(ipaddr_label_0,
					     64-ipaddr_label_offset,0,
					     16,ipaddr_label_0->height,
					     64,vert_shift);
		  ipaddr_label_offset-=2;
		  if(ipaddr_label_offset+ipaddr_label_0->width<64)
		    {
		      ipaddr_label_offset=64;
		      shift_change();
		      display_state=DISPLAY_ETH1;
		    }
		  
		}
	      else
		display_state=DISPLAY_ETH1;
	      
	      break;
	    case DISPLAY_ETH1:
	      if(ipaddr_label_1)
		{
		  if(vert_shift==0)
		    oled_copy_rectangle(16,0,16+2,0,
					64+16-2,ipaddr_label_1->height+1);
		  else
		    oled_copy_rectangle(16,vert_shift-1,16+2,vert_shift-1,
					64+16-2,ipaddr_label_1->height+2);
		  if((ipaddr_label_offset&0x0f)==0)
		    oled_send_image_fragment(ipaddr_label_1,
					     64-ipaddr_label_offset,0,
					     16,ipaddr_label_1->height,
					     64,vert_shift);
		  ipaddr_label_offset-=2;
		  if(ipaddr_label_offset+ipaddr_label_1->width<64)
		    {
		      ipaddr_label_offset=64;
		      shift_change();
		      display_state=DISPLAY_ETH0;
		    }
		  
		}
	      else
		display_state=DISPLAY_ETH0;
	      
	      break;
	    default:
	      display_state=DISPLAY_ETH0;

	    }
	  if(prev_display_state!=display_state)
	    {
	      switch(display_state)
		{
		case DISPLAY_ETH0:
		  oled_draw_box(0,38+1+vert_shift, 96, 10,
				glob_rb,glob_gb,glob_bb);
		  oled_draw_box(16+60+horiz_shift-10, 40+vert_shift, 10, 1,
				msg_r,msg_g,msg_b);
		  oled_draw_box(16+60+horiz_shift-8, 40+1+vert_shift, 8, 1,
				msg_r,msg_g,msg_b);
		  oled_draw_box(16+60+horiz_shift-6, 40+2+vert_shift, 6, 1,
				msg_r,msg_g,msg_b);
		  oled_draw_box(16+60+horiz_shift-4, 40+3+vert_shift, 4, 1,
				msg_r,msg_g,msg_b);
		  oled_draw_box(16+60+horiz_shift-2, 40+4+vert_shift, 2, 1,
				msg_r,msg_g,msg_b);
		  break;
		case DISPLAY_ETH1:
		  oled_draw_box(0, 38+1+vert_shift, 96, 10,
				glob_rb,glob_gb,glob_bb);
		  oled_draw_box(16+horiz_shift, 40+vert_shift, 10, 1,
				msg_r,msg_g,msg_b);
		  oled_draw_box(16+horiz_shift, 40+1+vert_shift, 8, 1,
				msg_r,msg_g,msg_b);
		  oled_draw_box(16+horiz_shift, 40+2+vert_shift, 6, 1,
				msg_r,msg_g,msg_b);
		  oled_draw_box(16+horiz_shift, 40+3+vert_shift, 4, 1,
				msg_r,msg_g,msg_b);
		  oled_draw_box(16+horiz_shift, 40+4+vert_shift, 2, 1,
				msg_r,msg_g,msg_b);
		  break;
		}
	      prev_display_state=display_state;
	    }
	}
      wait_for_timer(&loop_timer);
    }
  oled_button_close();
  return 0;
}
