/*
  mbbl-imagetool.c -- MBBL image writer utility.
*/
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <errno.h>
#include <arpa/inet.h>
#include <getopt.h>

#include <mtd/mtd-user.h>
/* linux MTD character device */
#ifndef MTD_CHAR_MAJOR
#define MTD_CHAR_MAJOR 90
#endif

#include "libfdt_env.h"
#include "libfdt.h"
#include "fdt.h"

#define min(x,y) ((x)<(y))?(x):(y)
#define max(x,y) ((x)>(y))?(x):(y)

#include <linux/types.h>

typedef __u32 u32;
typedef __u16 u16;
typedef __u8 u8;

/* limit imposed by the kernel boot procedure */
#define FDT_SIZE 16384

/* headers */
#define BOOT_IMAGE_HEADER \
  "MBBL FIRMWARE IMAGE V1.0\0\0\0\0\0\0\0\0"
#define BITSTREAM_IMAGE_HEADER \
  "MBBL BITSTREAM\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"
#define BOOTLOADER_IMAGE_HEADER \
  "MBBL BOOTLOADER\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"
#define BOOT_IMAGE_HEADER_SIZE 32
#define BITSTREAM_IMAGE_HEADER_SIZE 32
#define BOOTLOADER_IMAGE_HEADER_SIZE 32

/* default output file */
#define DEFAULT_BOOTIMAGE_NAME "bootimage.bin"

/* all segment types */
enum{
  REC_TYPE_FDT,
  REC_TYPE_KERNEL,
  REC_TYPE_RDISK,
  REC_TYPE_BOOT_SCREEN,
  REC_TYPE_FONT,
  REC_TYPE_END,
  REC_TYPE_EMPTY,
  REC_TYPE_BAD,
  REC_TYPE_IDENTITY,
  REC_TYPE_USER,
  REC_TYPE_BITSTREAM,
  REC_TYPE_BOOTLOADER
};

/* output types */
enum{
  OUTPUT_TYPE_FILE,
  OUTPUT_TYPE_MTD
};

/* segment header -- uses network byte order */
typedef struct
{
  u32 image_segment_type;
  u32 size_padded;
} __attribute__((packed)) image_segment_header;

/* internal representation of an image segment -- uses host byte order */
typedef struct
{
  u32 image_segment_type;
  u32 size_padded;
  u32 offset;
  u32 size_actual;
  u32 align;
  int h;
  union
  {
    u8 *ptr;
    u32 fileoffset;
  } data;
} image_segment;

/* xilinx .bit file has its own headers -- uses network byte order */
typedef struct
{
  u8 fieldtype;
  u16 fieldlen_n;
} __attribute__((packed)) xil_bitstream_fieldheader;

/* global variables -- segments list */
u32 nsegments=0;
image_segment *segments;
u32 start_address=0,end_address=0;

/* this message is very common */
void errmsg_mem(void)
{
  fprintf(stderr,"Insufficient memory\n");
}

/* remove all segments, close files and free memory */
void remove_all_segments(void)
{
  u32 i;
  if(nsegments)
    {
      for(i=0;i<nsegments;i++)
	{
	  if(segments[i].h>=0)
	    close(segments[i].h);
	  else
	    /* 
	       only free data if h<-1, what means, data was allocated
	       specifically for this segment
	    */
	    if(segments[i].h<-1)
	      free(segments[i].data.ptr);
	}
      nsegments=0;
      free(segments);
    }
  end_address=start_address;
}

int skip_block_range(unsigned char *dst, u32 size)
{
  /* for file-based I/O, fill buffers with 0xff */
  memset(dst,0xff,size);
  return size;
}

int copy_block_range(unsigned char *dst,image_segment *segment,
		     u32 skip,u32 count){
  long curr_offset;
  if(segment->h>=0)
    {
      curr_offset=lseek(segment->h,0,SEEK_CUR);
      if(curr_offset!=skip)
	{
	  fprintf(stderr,"Read offset doesn't match -- expected %lu, got %lu\n",
		  skip,curr_offset);
	  return -1;
	}
      return read(segment->h,dst,count);
    }
  else
    {
      if(segment->image_segment_type==REC_TYPE_BAD)
	memset(dst,0xff,count);
      else
	memcpy(dst,segment->data.ptr+skip,count);
      return count;
    }
}

int read_buffer_block(int h,unsigned char *buffer,u32 offset,u32 size,
		      int outputtype)
{
  off_t curr_offset;
  long l;
  u32 size_left;
  unsigned char *buffer_read;
  if(outputtype==OUTPUT_TYPE_MTD)
    {
      /* for mtd, read the block */
      curr_offset=lseek(h,offset,SEEK_SET);
      if(curr_offset!=offset)
	{
	  printf("\n%08lx SEEK ERROR\n",offset);
	  return -1;
	}
      printf(
       "%08lx - %08lx read...                                   \x1b[35D",
       offset,offset+size-1);
      fflush(stdout);
      size_left=size;
      buffer_read=buffer;
      while(size_left>512)
	{
	  l=read(h,buffer_read,512);
	  if(l!=512)
	    {
	      printf("\x1b[3D FAILED\n");
	      if(buffer==buffer_read)
		{
		  return l;
		}
	      else
		{
		  if(l>0)
		    return buffer_read-buffer+l;
		  else
		    return buffer_read-buffer;
		}
	    }
	  buffer_read+=512;
	  size_left-=512;
	 }
      if(size_left>0)
	{
	  l=read(h,buffer_read,size_left);
	  if(l!=size_left)
	    {
	      printf("\x1b[3D FAILED\n");
	      if(buffer==buffer_read)
		{
		  return l;
		}
	      else
		{
		  if(l>0)
		    return buffer_read-buffer+l;
		  else
		    return buffer_read-buffer;
		}
	    }
	}
      printf("\x1b[3D done\r");
      return size;
    }
  /* for file-based I/O, do nothing */
  return size;
}

int write_buffer_block(int h,unsigned char *buffer,u32 offset,u32 size,
		       int outputtype)
{
  off_t curr_offset;
  long l;
  u32 size_left;
  unsigned char *buffer_write;
  erase_info_t erase;

  if(outputtype==OUTPUT_TYPE_MTD)
    {
      curr_offset=lseek(h,offset,SEEK_SET);
      if(curr_offset!=offset)
	{
	  printf("\n%08lx SEEK ERROR\n",offset);
	  return -1;
	}
      
      erase.start=offset;
      erase.length=size;
      ioctl(h,MEMUNLOCK,&erase);

      printf("%08lx - %08lx erase... \x1b[1D",offset,offset+size-1);
      fflush(stdout);
      erase.start=offset;
      erase.length=size;
      if(ioctl(h,MEMERASE,&erase)<0)
	{
	  printf("\x1b[3D FAILED\n");
	  return -1;
	}
      printf("\x1b[3D done, ");
      size_left=size;
      buffer_write=buffer;
      while(size_left>512)
	{
	  printf("%08lx - %08lx write...",
		 offset+(buffer_write-buffer),
		 offset+(buffer_write-buffer)+512-1);
	  fflush(stdout);
	  l=write(h,buffer_write,512);
	  if(l!=512)
	    {
	      printf("\x1b[3D FAILED\n");
	      if(buffer==buffer_write)
		{
		  return l;
		}
	      else
		{
		  if(l>0)
		    return buffer_write-buffer+l;
		  else
		    return buffer_write-buffer;
		}
	    }
	  printf("\x1b[28D");
	  buffer_write+=512;
	  size_left-=512;
	}
      if(size_left>0)
	{
	  printf("%08lx - %08lx write...",
		 offset+(buffer_write-buffer),
		 offset+(buffer_write-buffer)+size_left-1);
	  fflush(stdout);
	  l=write(h,buffer_write,size_left);
	  if(l!=size_left)
	    {
	      printf("\x1b[3D FAILED\n");
	      if(buffer==buffer_write)
		{
		  return l;
		}
	      else
		{
		  if(l>0)
		    return buffer_write-buffer+l;
		  else
		    return buffer_write-buffer;
		}
	    }
	}
      printf("\x1b[28D%08lx - %08lx write done\r",offset,offset+size-1);
      erase.start=offset;
      erase.length=size;
      ioctl(h,MEMLOCK,&erase);
      return size;
    }
  else
    {
      curr_offset=lseek(h,0,SEEK_CUR);
      if(curr_offset!=offset-start_address)
	{
	  fprintf(stderr,
		  "Write offset doesn't match -- expected %lu, got %lu\n",
		  offset-start_address,curr_offset);
	  return -1;
	}
      return write(h,buffer,size);
    }
}

/* 
   write a block to the file (file position is assumed to be at the
   beginning of this block)
*/
int write_block(int h_out,u32 block_start,u32 block_size,int outputtype)
{
  /* buffer */
  static unsigned char *write_buffer=NULL;
  static u32 write_buffer_allocated=0;
  image_segment_header segmentheader;

  u32 i,alloc_size,current_write_offset,skip,count;

  int block_read_flag=0;

  /* for mtd devices,buffer contains two copies */
  if(outputtype==OUTPUT_TYPE_MTD)
    alloc_size=block_size*2;
  else
    alloc_size=block_size*2;

  /* allocate/reallocate buffer if necessary */
  if(write_buffer_allocated<alloc_size)
    {
      if(write_buffer_allocated!=0)
	{
	  free(write_buffer);
	  write_buffer_allocated=0;
	}
      write_buffer=(unsigned char*)malloc(alloc_size);
      if(!write_buffer)
	return -1;
      write_buffer_allocated=alloc_size;
    }

  current_write_offset=block_start;

  for(i=0;i<nsegments;i++)
    {
      /* overlap? */
      if((segments[i].offset<block_start+block_size)
	 &&(segments[i].offset+segments[i].size_padded
	    +((i+1>=nsegments)?0:sizeof(image_segment_header))>block_start))
	{
	  /* overlap with data? */
	  if((segments[i].offset<block_start+block_size)
	     &&(segments[i].offset+segments[i].size_actual>block_start))
	    {
	      /* 
		 if the block isn't read yet, and the overlapping data is not
		 a bad segment, read it and mark the block as read

		 if the block isn't read, and overlapping data is a bad
		 segment, fill it with 0xff
	      */
	      if(!block_read_flag)
		{
		  if(segments[i].image_segment_type!=REC_TYPE_BAD)
		    {
		      if(read_buffer_block(h_out,write_buffer,
					   block_start,block_size,outputtype)
			 !=block_size)
			return -2;

		      /* for mtd devices,buffer contains two copies */
		      if(outputtype==OUTPUT_TYPE_MTD)
			memcpy(write_buffer+block_size,write_buffer,
			       block_size);
		      block_read_flag=1;
		    }
		  else
		    {
		      memset(write_buffer,0xff,block_size);
		    }
		}
	      
	      /* data starts after current offset? */
	      if(current_write_offset<segments[i].offset)
		{
		  skip_block_range(write_buffer+
				   (current_write_offset-block_start),
				   segments[i].offset-current_write_offset);
		  current_write_offset=segments[i].offset;
		}
	      skip=current_write_offset-segments[i].offset;
	      count=segments[i].size_actual-skip;
	      if(current_write_offset+count>block_start+block_size)
		count=block_start+block_size-current_write_offset;
	      
	      copy_block_range(write_buffer
			       +(current_write_offset-block_start),
			       &segments[i],
			       skip,count);
	      current_write_offset+=count;
	    }
	  /* overlap with the next header? */
	  if((i+1<nsegments)
	     &&(segments[i].offset+segments[i].size_padded
		<block_start+block_size)
	     &&(segments[i].offset+segments[i].size_padded
		+sizeof(image_segment_header)
		>block_start))
	    {
	      /* 
		 if the block isn't read yet, read the segment and mark
		 it as read
	      */
	      if(!block_read_flag)
		{
		  if(read_buffer_block(h_out,write_buffer,
				       block_start,block_size,outputtype)
		     !=block_size)
		    return -2;

		  /* for mtd devices,buffer contains two copies */
		  if(outputtype==OUTPUT_TYPE_MTD)
		    memcpy(write_buffer+block_size,write_buffer,
			   block_size);
		  block_read_flag=1;
		}
	      
	      /* next header starts after current offset? */
	      if(current_write_offset<segments[i].offset
		 +segments[i].size_padded)
		{
		  skip_block_range(write_buffer+
				   (current_write_offset-block_start),
				   segments[i].offset+segments[i].size_padded
				   -current_write_offset);
		  current_write_offset=segments[i].offset
		    +segments[i].size_padded;		  
		}
	      skip=current_write_offset-(segments[i].offset
					 +segments[i].size_padded);
	      count=sizeof(image_segment_header)-skip;
	      if(current_write_offset+count>block_start+block_size)
		count=block_start+block_size-current_write_offset;

	      segmentheader.image_segment_type=
		htonl(segments[i+1].image_segment_type);
	      segmentheader.size_padded=
		htonl(segments[i+1].size_padded);
	      memcpy(write_buffer
		     +(current_write_offset-block_start),
		     ((char*)&segmentheader)+skip,count);
	      current_write_offset+=count;
	    }
	}
    }
  if(current_write_offset!=block_start+block_size)
    {
      skip_block_range(write_buffer+(current_write_offset-block_start),
		       block_start+block_size-current_write_offset);
    }
  /* for mtd devices, skip write if content is unchanged */
  if((outputtype==OUTPUT_TYPE_MTD)
     &&(!block_read_flag||!memcmp(write_buffer+block_size,write_buffer,
				  block_size)))
    return 0;

  if(write_buffer_block(h_out,write_buffer,block_start,block_size,outputtype)
     !=block_size)
    return -3;
  else
    return 0;
}

#ifdef OLD_WRITE_ALL_SEGMENTS
/* write all segments to a file */
int write_all_segments(int h_out)
{
  u32 i,write_buffer_size=0;
  long l;
  char *write_buffer=NULL,*write_buffer_new;
  image_segment_header segmentheader;

  for(i=0;i<nsegments;i++)
    {
      if(segments[i].size_padded+sizeof(image_segment_header)
	 >write_buffer_size)
	{
	  if(write_buffer_size==0)
	    {
	      write_buffer=(char*)malloc(segments[i].size_padded
					 +sizeof(image_segment_header));
	      if(write_buffer==NULL)
		{
		  errmsg_mem();
		  return 1;
		}
	      else
		write_buffer_size=segments[i].size_padded
		  +sizeof(image_segment_header);
	    }
	  else
	    {
	      write_buffer_new=(char*)realloc(write_buffer,
					      segments[i].size_padded
					      +sizeof(image_segment_header));
	      if(write_buffer_new==NULL)
		{
		  free(write_buffer);
		  errmsg_mem();
		  return 1;
		}
	      else
		{
		  write_buffer=write_buffer_new;
		  write_buffer_size=segments[i].size_padded
		    +sizeof(image_segment_header);
		}
	    }
	}
      memset(write_buffer,0xff,segments[i].size_padded);
      if(segments[i].h>=0)
	{
	  l=lseek(segments[i].h,segments[i].data.fileoffset,SEEK_SET);
	  if(l!=segments[i].data.fileoffset)
	    {
	      free(write_buffer);
	      fprintf(stderr,"Seek Error\n");
	      return 1;
	    }
	  l=read(segments[i].h,write_buffer,segments[i].size_actual);
	  if(l<segments[i].size_actual)
	    {
	      free(write_buffer);
	      fprintf(stderr,"Read Error\n");
	      return 1;
	    }
	}
      else
	{
	  memcpy(write_buffer,segments[i].data.ptr,segments[i].size_actual);
	}
      if(i+1<nsegments)
	{
	  segmentheader.image_segment_type=
	    htonl(segments[i+1].image_segment_type);
	  segmentheader.size_padded=
	    htonl(segments[i+1].size_padded);
	  memcpy(write_buffer+segments[i].size_padded,&segmentheader,
		 sizeof(image_segment_header));
	}
      l=write(h_out,write_buffer,segments[i].size_padded
	      +((i+1>=nsegments)?0:sizeof(image_segment_header)));
      if(l<segments[i].size_padded
	 +((i+1>=nsegments)?0:sizeof(image_segment_header)))
	{
	  free(write_buffer);
	  fprintf(stderr,"Write Error\n");
	  return 1;
	}
    }
  free(write_buffer);
  return 0;
}
#endif

/* add a new segment (used internally) */
int add_segment(u32 image_segment_type,u32 size,
		u32 align)
{
  u32 unaligned,offset;
  image_segment *newsegments;

  if(nsegments==0)
    {
      segments=(image_segment*)malloc(sizeof(image_segment));
      if(!segments) return -1;
      nsegments=1;
    }
  else
    {
      newsegments=(image_segment*)realloc(segments,
					  sizeof(image_segment)*(nsegments+1));
      if(!newsegments) return -1;
      segments=newsegments;
      nsegments++;
    }
  segments[nsegments-1].image_segment_type=image_segment_type;
  segments[nsegments-1].align=align;
  segments[nsegments-1].size_actual=size;
  segments[nsegments-1].h=-1;
  segments[nsegments-1].data.ptr=NULL;
  offset=end_address;

  unaligned=offset%align;
  if(unaligned)
    offset+=align-unaligned;

  segments[nsegments-1].offset=offset;

  if(nsegments>1)
    segments[nsegments-2].size_padded=offset-segments[nsegments-2].offset
      -sizeof(image_segment_header);

  end_address=offset+size+sizeof(image_segment_header);
  return nsegments-1;
}

/* add a file, with some initial offset, as a segment */
int add_file_segment(u32 image_segment_type,char *name,
		     u32 fileoffset,u32 align)
{
  int h,index;
  struct stat statbuf;

  h=open(name,O_RDONLY);
  if(h<0)
    return -1;
  if(fstat(h,&statbuf))
    {
      close(h);
      return -1;
    }
  if(fileoffset>statbuf.st_size)
    {
      close(0);
      return -1;
    }
  index=add_segment(image_segment_type,statbuf.st_size-fileoffset,align);
  if(index<0)
    {
      close(h);
      return -1;
    }
  segments[index].h=h;
  segments[index].data.fileoffset=fileoffset;

  /* file remains open */
  return index;
}

/* add data in memory as a segment */
int add_memory_segment(u32 image_segment_type,u8 *src,int alloc,
		       u32 size,u32 align)
{
  int index;
  index=add_segment(image_segment_type,size,align);
  if(index<0)
    return -1;

  segments[index].h=alloc?-2:-1;
  segments[index].data.ptr=src;
  return index;
}

/* invert bits for xilinx/intel formats conversion */
static inline u8 invertbits(u8 c)
{
  return ((c&0x01)<<7)|((c&0x02)<<5)|((c&0x04)<<3)|((c&0x08)<<1)
    |((c&0x10)>>1)|((c&0x20)>>3)|((c&0x40)>>5)|((c&0x80)>>7);
}

/* buffer and message used in xilinx bitstream parsing */
static unsigned char fieldbuffer[256];
static char *msg_invalid_format="Invalid Xilinx bitstream file format: %s\n";

/* add a bitstream file, as a segment */
int add_bit_file_segment(char *name,int invert_bitstream)
{
  int h,data_found;
  size_t l,datalen;
  unsigned int i;
  unsigned char *data;
  xil_bitstream_fieldheader header;

  h=open(name,O_RDONLY);
  if(h<0)
    {
      perror("Opening bitstream file");
      return -1;
    }

  /* file starts with a 16-bit field length in network byte order */
  l=read(h,&header.fieldlen_n,sizeof(header.fieldlen_n));
  if(l<sizeof(header.fieldlen_n))
    {
      fprintf(stderr,msg_invalid_format,"incomplete header length field");
      close(h);
      return -1;
    }
  if(htons(header.fieldlen_n)>sizeof(fieldbuffer))
    {
      fprintf(stderr,msg_invalid_format,"file header is too long");
      close(h);
      return -1;
    }
  /* header -- no particular known meaning */
  l=read(h,fieldbuffer,htons(header.fieldlen_n));
  if(l<htons(header.fieldlen_n))
    {
      fprintf(stderr,msg_invalid_format,"incomplete header");
      close(h);
      return -1;
    }
  /* 
     another length field is always 1, preceding the next field,
     apparently character 'a' that follows is the field, however same
     character seems to identify the following length and data, so
     apparently that was not the original purpose of those bytes
  */
  l=read(h,&header.fieldlen_n,sizeof(header.fieldlen_n));
  if(l<sizeof(header.fieldlen_n))
    {
      fprintf(stderr,msg_invalid_format,"incomplete record length field");
      close(h);
      return -1;
    }
  if(htons(header.fieldlen_n)!=1)
    {
      fprintf(stderr,msg_invalid_format,"first record length is not 1");
      close(h);
      return -1;
    }

  data_found=0;
  /* scan through fields until the last field with bitstream data */
  do
    {
      /* 
	 from this point each field header contains a single-character
	 type and length
      */
      l=read(h,&header,sizeof(header));
      if(l<sizeof(header.fieldlen_n))
	{
	  fprintf(stderr,msg_invalid_format,"incomplete header record");
	  close(h);
	  return -1;
	}

      if(header.fieldtype=='e')
	{
	  /* bitstream data uses 32-bit length field */
	  memcpy(fieldbuffer,&header.fieldlen_n,sizeof(header.fieldlen_n));
	  l=read(h,fieldbuffer+sizeof(header.fieldlen_n),
		 sizeof(u32)-sizeof(header.fieldlen_n));
	  if(l<(sizeof(u32)-sizeof(header.fieldlen_n)))
	    {
	      fprintf(stderr,msg_invalid_format,
		      "incomplete bitstream length record");
	      close(h);
	      return -1;
	    }
	}
      else
	{
	  if(htons(header.fieldlen_n)<1)
	    {
	      fprintf(stderr,msg_invalid_format,"zero-length field");
	      close(h);
	      return -1;
	    }
	  if(htons(header.fieldlen_n)>sizeof(fieldbuffer))
	    {
	      fprintf(stderr,msg_invalid_format,"field is too long");
	      close(h);
	      return -1;
	    }
	  l=read(h,fieldbuffer,htons(header.fieldlen_n));
	  if(l<htons(header.fieldlen_n))
	    {
	      fprintf(stderr,msg_invalid_format,"incomplete field");
	      close(h);
	      return -1;
	    }
	}
      /*
	all field types other than bitstream data, contain zero-terminated
	text strings with human-readable metadata
      */
      switch(header.fieldtype)
	{
	case 'a':
	  /* 
	     original design file name (and possibly other
	     parameters after ';')
	  */
	  if(fieldbuffer[htons(header.fieldlen_n)-1]!='\0')
	    {
	      fprintf(stderr,msg_invalid_format,"invalid design file name");
	      close(h);
	      return -1;
	    }
#ifdef DEBUG
	  printf("Design file: %s\n",fieldbuffer);
#endif
	  break;
	case 'b':
	  /* target part name */
	  if(fieldbuffer[htons(header.fieldlen_n)-1]!='\0')
	    {
	      fprintf(stderr,msg_invalid_format,"invalid part name");
	      close(h);
	      return -1;
	    }
#ifdef DEBUG
	  printf("Part name  : %s\n",fieldbuffer);
#endif
	  break;
	case 'c':
	  /* date */
	  if(fieldbuffer[htons(header.fieldlen_n)-1]!='\0')
	    {
	      fprintf(stderr,msg_invalid_format,"invalid date");
	      close(h);
	      return -1;
	    }
#ifdef DEBUG
	  printf("Date       : %s\n",fieldbuffer);
#endif
	  break;
	case 'd':
	  /* time */
	  if(fieldbuffer[htons(header.fieldlen_n)-1]!='\0')
	    {
	      fprintf(stderr,msg_invalid_format,"invalid time");
	      close(h);
	      return -1;
	    }
#ifdef DEBUG
	  printf("Time       : %s\n",fieldbuffer);
#endif
	  break;
	case 'e':
	  /* bitstream data (length is a 32-bit field, in network byte order */
	  datalen=(((u32)fieldbuffer[0])<<24)|(((u32)fieldbuffer[1])<<16)
	    |(((u32)fieldbuffer[2])<<8)|((u32)fieldbuffer[3]);
#ifdef DEBUG
	  printf("Length     : %u bytes\n",datalen);
#endif
	  data_found=1;
	  break;
	default:
	  fprintf(stderr,"Unknown field type 0x%02x, ignored\n",
		  header.fieldtype);
	  break;
	}
    }
  while(!data_found);

  if(datalen<=8)
    {
      fprintf(stderr,msg_invalid_format,"bitstream is too short");
      close(h);
      return -1;
    }
  /* allocate memory for bitstream data and MBBL bitstream header */
  data=(unsigned char*)malloc(datalen+BITSTREAM_IMAGE_HEADER_SIZE);
  if(!data)
    {
      fprintf(stderr,"Insufficient memory to load %u bytes of bitstream\n",
	      datalen);
      close(h);
      return -1;
    }

  /* initialize header, append bitstream data */
  memcpy(data,BITSTREAM_IMAGE_HEADER,BITSTREAM_IMAGE_HEADER_SIZE);
  l=read(h,data+BITSTREAM_IMAGE_HEADER_SIZE,datalen);
  if(l<datalen)
    {
      fprintf(stderr,msg_invalid_format,"incomplete bitstream");
      close(h);
      return -1;
    }

  /* there should be no data in the file after the end of bitstream */
  l=read(h,fieldbuffer,1);
  if(l>0)
    {
      fprintf(stderr,msg_invalid_format,"file is longer than bitstream");
      close(h);
      return -1;
    }
  close(h);

  if(invert_bitstream)
    {
      /* invert bits in bitstream */
      for(i=BITSTREAM_IMAGE_HEADER_SIZE;
	  i<datalen+BITSTREAM_IMAGE_HEADER_SIZE;i++)
	{
	  data[i]=invertbits(data[i]);
	}
    }

  /*
    add the segment, mark it as allocated, so it will be de-allocated
    in the end or before repeating this procedure
  */
  return add_memory_segment(REC_TYPE_BITSTREAM,data,1,
			    datalen+BITSTREAM_IMAGE_HEADER_SIZE,1);
}

/* add an empty bitstream segment */
int add_empty_bit_segment(void)
{
  unsigned char *data;

  /* allocate memory for MBBL bootloader header */
  data=(unsigned char*)malloc(BOOTLOADER_IMAGE_HEADER_SIZE);
  if(!data)
    {
      fprintf(stderr,"Insufficient memory for bootloader header\n");
      return -1;
    }

  /* initialize header */
  memcpy(data,BOOTLOADER_IMAGE_HEADER,BOOTLOADER_IMAGE_HEADER_SIZE);

  /*
    add the segment, mark it as allocated, so it will be de-allocated
    in the end or before repeating this procedure
  */
  return add_memory_segment(REC_TYPE_BITSTREAM,data,1,
			    BOOTLOADER_IMAGE_HEADER_SIZE,1);
}

/* add segment with identity information, update device tree */
int add_identity_segment(unsigned char *identity_data,unsigned char *fdt_buf)
{
  unsigned char *identity_data_copy,*identity_ptr,
    *identity_key,*identity_key_end,
    *identity_value,identity_fdt_value[6],identity_new_value[6];
  unsigned identity_new_value_u[6],val;
  const char *identity_fdt_path,*identity_fdt_ptr;
  char *identity_fdt_full_path;

  int n,identity_offset,identity_fdt_value_len,identity_fdt_path_len;

  const struct fdt_property *identity_property;  
  identity_data_copy=(unsigned char*)strdup((char*)identity_data);
  if(!identity_data_copy)
    {
      errmsg_mem();
      return -1;
    }
  /* parse buffer as a text file containing key=value pairs */
  identity_ptr=identity_data_copy;
  while(*identity_ptr)
    {
      while(*identity_ptr
	    &&((*identity_ptr<' ')||isspace(*identity_ptr)))
	identity_ptr++;
      if(*identity_ptr)
	{
	  identity_key=identity_ptr;
	  while(*identity_ptr&&(*identity_ptr!='=')
		&&(*identity_ptr>=' '))
	    identity_ptr++;
	  if(*identity_ptr=='=')
	    {
	      identity_key_end=identity_ptr;
	      while((identity_key_end>identity_key)
		    &&((*identity_key_end<' ')
		       ||(*identity_key_end=='=')
		       ||isspace(*identity_key_end)))
		{
		  *identity_key_end='\0';
		  identity_key_end--;
		}
	      identity_ptr++;
	      while(*identity_ptr&&isspace(*identity_ptr))
		identity_ptr++;

	      if(*identity_ptr)
		{
		  identity_value=identity_ptr;
		  while(*identity_ptr
			&&(*identity_ptr>=' ')
			&&!isspace(*identity_ptr))
		    identity_ptr++;

		  if(*identity_ptr)
		    {
		      *identity_ptr='\0';
		      identity_ptr++;
		    }
#ifdef DEBUG		  
		  printf("Identity information: \"%s\" = \"%s\"\n",
			 identity_key,identity_value);
#endif
		  n=sscanf((const char*)identity_value,"%x:%x:%x:%x:%x:%x",
			   &identity_new_value_u[0],
			   &identity_new_value_u[1],
			   &identity_new_value_u[2],
			   &identity_new_value_u[3],
			   &identity_new_value_u[4],
			   &identity_new_value_u[5]);
		  if(n==6
		     &&(identity_new_value_u[0]
			==(identity_new_value_u[0]&0xff))
		     &&(identity_new_value_u[1]
			==(identity_new_value_u[1]&0xff))
		     &&(identity_new_value_u[2]
			==(identity_new_value_u[2]&0xff))
		     &&(identity_new_value_u[3]
			==(identity_new_value_u[3]&0xff))
		     &&(identity_new_value_u[4]
			==(identity_new_value_u[4]&0xff))
		     &&(identity_new_value_u[5]
			==(identity_new_value_u[5]&0xff)))
		    {
		      identity_new_value[0]=
			(unsigned char)identity_new_value_u[0]&0xff;
		      identity_new_value[1]=
			(unsigned char)identity_new_value_u[1]&0xff;
		      identity_new_value[2]=
			(unsigned char)identity_new_value_u[2]&0xff;
		      identity_new_value[3]=
			(unsigned char)identity_new_value_u[3]&0xff;
		      identity_new_value[4]=
			(unsigned char)identity_new_value_u[4]&0xff;
		      identity_new_value[5]=
			(unsigned char)identity_new_value_u[5]&0xff;

		      identity_fdt_path=fdt_get_alias(fdt_buf,
						      (char*)identity_key);
		      if(identity_fdt_path)
			{
			  identity_fdt_ptr=strrchr(identity_fdt_path,'@');
			  if(identity_fdt_ptr)
			    {
			      identity_fdt_path_len=
				strlen(identity_fdt_path)
				+strlen(identity_fdt_ptr)
				+10; 
			      /* 9 characters in "/ethernet" plus '\0' */
			  
			      identity_fdt_full_path=
				(char*)malloc(identity_fdt_path_len);
			      
			      if(identity_fdt_full_path)
				{
				  strcpy(identity_fdt_full_path,
					 identity_fdt_path);
				  /* first check if device has local-mac-address */
				  identity_offset=
				    fdt_path_offset(fdt_buf,
						    identity_fdt_full_path);
				  
				  if(identity_offset>=0)
				    {
				      identity_property=
					fdt_get_property(fdt_buf,
							 identity_offset,
							 "local-mac-address",
							 &identity_fdt_value_len);
				      if(identity_fdt_value_len<0)
					{
					  /* 
					     no local-mac-address here, look for
					     "ethernet" under it

					     strcat() is valid here because space
					     allocated matches the length of
					     concatenated strings
					  */
					  strcat(identity_fdt_full_path,"/ethernet");
					  strcat(identity_fdt_full_path,
						 identity_fdt_ptr);
					  identity_offset=
					    fdt_path_offset(fdt_buf,
							    identity_fdt_full_path);
					  if(identity_offset>=0)
					    {
					      identity_property=
						fdt_get_property(fdt_buf,
								 identity_offset,
								 "local-mac-address",
								 &identity_fdt_value_len);
					    }
					}
				      if(identity_property
					 &&identity_fdt_value_len==6)
					/* 
					   MAC address is always
					   6 bytes long
					*/
					{
					  memcpy(identity_fdt_value,
						 (const char*)identity_property
						 +sizeof(struct fdt_property),
						 6);
#ifdef DEBUG
	  printf("Identity path: \"%s\" = %02x:%02x:%02x:%02x:%02x:%02x\n",
		 identity_fdt_path,
		 identity_fdt_value[0],identity_fdt_value[1],
		 identity_fdt_value[2],identity_fdt_value[3],
		 identity_fdt_value[4],identity_fdt_value[5]);
#endif
					  fdt_setprop_inplace(fdt_buf,
							identity_offset,
							"local-mac-address",
							&identity_new_value,6);
					}
				    }
				  free(identity_fdt_full_path);
				}
			    }
			}
		    }
		  else
		    {
		      if(!strcmp(identity_key,"serial"))
			{
			  n=sscanf((const char*)identity_value,"%i",&val);
			  if(n==1)
			    {
			      identity_offset=fdt_path_offset(fdt_buf,
							      "/chosen");
			      if(identity_offset>=0)
				{
				  identity_property=
				    fdt_get_property(fdt_buf,
						     identity_offset,
						     "serial-number",
						     &identity_fdt_value_len);
				  if(identity_property
				     &&identity_fdt_value_len==4)
				    /* 
				       serial number is always
				       4 bytes long
				    */
				    {
				      identity_new_value[0]=(val>>24)&0xff;
				      identity_new_value[1]=(val>>16)&0xff;
				      identity_new_value[2]=(val>>8)&0xff;
				      identity_new_value[3]=val&0xff;
				      fdt_setprop_inplace(fdt_buf,
							  identity_offset,
							  "serial-number",
							  &identity_new_value,
							  4);
				    }
				}
			    }
			}
		    }
		}
	    }
	}
    }
  free(identity_data_copy);
  return add_memory_segment(REC_TYPE_IDENTITY,identity_data,1,
			    strlen((char*)identity_data)+1,4);
}

/* copy data from file to file */
int copydata(int h_dst,int h_src,u32 size,int invert,int istext)
{
  u32 size_left,slen,i;
  long l;

  static char *buffer=NULL;
  if(!buffer)
    buffer=(char*)malloc(4096);
  if(!buffer)
    return -1;
  size_left=size;
  while(size_left>4096)
    {
      if(read(h_src,buffer,4096)==4096)
	{
	  if(istext)
	    {
	      for(i=0;i<4096&&buffer[i];i++);
	      slen=i;
	      if(slen!=4096)
		size_left=slen;
	    }
	  else
	    {
	      slen=4096;
	    }
	  if(invert) for(i=0;i<4096;i++) buffer[i]=invertbits(buffer[i]);
	  l=write(h_dst,buffer,slen);
	  if(l!=slen)
	    {
	      if(l<0)
		return size-size_left;
	      else
		return size-size_left+l;
	    }
	}
      else
	return size-size_left;
      
      size_left-=slen;
    }
  if(size_left>0)
    {
      if(read(h_src,buffer,size_left)==size_left)
	{
	  if(istext)
	    {
	      for(i=0;i<size_left&&buffer[i];i++);
	      size_left=i;
	    }
	  if(invert) for(i=0;i<size_left;i++) buffer[i]=invertbits(buffer[i]);
	  l=write(h_dst,buffer,size_left);
	  if(l!=size_left)
	    {
	      if(l<0)
		return size-size_left;
	      else
		return size-size_left+l;
	    }
	}
      else
	return size-size_left;
    }
  return size;
}

/* extract current segment into a file */
void read_extract_segment(int h,int size_padded,int istext,
			  char *file_name,char *file_desc)
{
  int h_out,save_errno;
  if(file_name)
    {
      h_out=open(file_name,O_WRONLY|O_CREAT|O_TRUNC,
		 S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);
      if(h_out<0)
	{
	  save_errno=errno;
	  fprintf(stderr,"Can't open output %s file",file_desc);
	  errno=save_errno;
	  perror("");
	}
      else
	{
	  if(copydata(h_out,h,size_padded,0,istext)!=size_padded)
	    {
	      save_errno=errno;
	      fprintf(stderr,"Can't write output %s file",file_desc);
	      errno=save_errno;
	      perror("");
	      close(h_out);
	      unlink(file_name);
	      h_out=-1;
	    }
	  if(h_out>=0)
	    close(h_out);
	}
    }
}

/*
  insert bad segment -- should only be called in the order of increasing
  offsets after all other segments are added

  all segments should be removed and procedure should be restarted if
  a new bad segment is discovered before the end of known bad segments
*/
int insert_bad_segment(u32 start,u32 end)
{
  int first_overlap,i;
  u32 unaligned,offset;
  image_segment *newsegments;

  /* do not add bad segments if no other segments are defined */
  if(nsegments<1)
    return -1;
  /* do not add segment if it ends before the image */
  if(end<=start_address)
    return -1;
  /* append the segment if it starts after the end of the image */
  if(start>=end_address)
    return add_segment(REC_TYPE_BAD,end-end_address,1);
  
  /* linear search for the first overlapping segment */
  for(first_overlap=-1,i=0;i<nsegments;i++)
    {
      if(segments[i].offset>=start)
	{
	  if(segments[i].offset==start)
	    first_overlap=i;
	  else
	    first_overlap=i-1;
	  i=nsegments;
	}
    }
  /*
    if no segments start on or after the start of the bad segments,
    and bad segment starts in the used area, it overlaps with the
    last segment
  */
  if(first_overlap<0)
    first_overlap=nsegments-1;

  /* can't have bad segment as the first segment */
  if(first_overlap==0)
    return -2;

  /* check if existing bad segment can be expanded */
  if(segments[first_overlap].image_segment_type==REC_TYPE_BAD)
    {
      if(end>segments[first_overlap].offset
	 +segments[first_overlap].size_actual)
	segments[first_overlap].size_actual=end
	  -segments[first_overlap].offset;
      else
	/* new bad segmemt is entirely within the old bad segment */
	return first_overlap;
    }
  else
    {
      /* check if preceding segment is bad and can be expanded */
      if(segments[first_overlap-1].image_segment_type==REC_TYPE_BAD)
	{
	  if(end>segments[first_overlap-1].offset
	     +segments[first_overlap-1].size_actual)
	    {
	      first_overlap--;
	      segments[first_overlap].size_actual=end
		-segments[first_overlap].offset;
	    }
	  else 
	    /* new bad segmemt is entirely within the old bad segment */
	    return first_overlap-1;
	}
      else
	{
	  /* create a new segment */
	  newsegments=(image_segment*)realloc(segments,
					      sizeof(image_segment)
					      *(nsegments+1));
	  if(!newsegments)
	    return -3;
	  segments=newsegments;
	  nsegments++;
	  memmove(&segments[first_overlap+1],&segments[first_overlap],
		  sizeof(image_segment)*(nsegments-first_overlap-1));
	  /*
	    keep the starting offset, change the size based on the end of the
	    bad segment
	  */
	  segments[first_overlap].image_segment_type=REC_TYPE_BAD;
	  segments[first_overlap].h=-1;
	  segments[first_overlap].data.ptr=NULL;
	  segments[first_overlap].align=1;
	  segments[first_overlap].size_actual=end
	    -segments[first_overlap].offset;
	  
	}
    }
  
  /* recalculate addresses for shifted segments */
  for(offset=end+sizeof(image_segment_header),i=first_overlap+1;
      i<nsegments;i++)
    {
      unaligned=offset%segments[i].align;
      if(unaligned)
	offset+=segments[i].align-unaligned;

      segments[i].offset=offset;

      segments[i-1].size_padded=offset-segments[i-1].offset
	-sizeof(image_segment_header);
      offset+=segments[i].size_actual+sizeof(image_segment_header);
    }
  end_address=offset;
  return first_overlap;
}

/* common messages */
void usage(void)
{
  fprintf(stderr,
"mbbl-imagetool -- boot image maker for MBBL\n"
"Usage:\n"
"Write image from files:\n"
"mbbl-imagetool [-N] [-s <address>] \\\n"
"               [-b <bitstream>] [-e <bootloader>] -d <fdt> \\\n"
"               [-i <identity>] -k <kernel> -r <ramdisk> \\\n"
"               [-l <logo>] [-f <font>] -o <image> [options]\n"
"or\n"
"mbbl-imagetool [--noinvert] [--start=<address>] \\\n"
"               [--bit=<bitstream>] [--exec=<bootloader>] --dtb=<fdt> \\\n"
"               [--id=<identity>] --kernel=<kernel> --ramdisk=<ramdisk> \\\n"
"               [--logo=<logo>] [--font=<font>] --out=<image>\n\n"
"Read image, extract files:\n"
"mbbl-imagetool [-N] [-s <address>] \\\n"
"               [-b <bitstream>] [-e <bootloader>] [-d <fdt>] \\\n"
"               [-i <identity>] [-k <kernel>] [-r <ramdisk>] \\\n"
"               [-l <logo>] [-f <font>] -I <image> [options]\n"
"or\n"
"mbbl-imagetool [--noinvert] [--start=<address>] \\\n"
"               [--bit=<bitstream>] [--exec=<bootloader>] [--dtb=<fdt>] \\\n"
"               [--id=<identity>] [--kernel=<kernel>] [--ramdisk=<ramdisk>] "
"\\\n"
"               [--logo=<logo>] [--font=<font>] --input=<image>\n"
	  );
}

#define MAX_FONTS (32)

/* names and buffers */
int fonts_count=0;
char *bitstream_name=NULL,*fdt_name=NULL,*bootloader_name=NULL,
  *identity_name=NULL,
  *kernel_name=NULL,*ramdisk_name=NULL,*logo_name=NULL,*bootimage_name=NULL,
  *font_names[MAX_FONTS];
unsigned char *fdt_buf;

unsigned int font_offsets[MAX_FONTS], font_sizes[MAX_FONTS];

/* cleanup (just in case this will be used in another program */
void cleanup_mem(void)
{
  int i;
  for(i=0;i<fonts_count;i++)
    {
      free(font_names[i]);
    }
  fonts_count=0;
  if(bitstream_name) free(bitstream_name);
  if(fdt_buf) free(fdt_buf);
  if(fdt_name) free(fdt_name);
  if(bootloader_name) free(bootloader_name);
  if(identity_name) free(identity_name);
  if(kernel_name) free(kernel_name);
  if(ramdisk_name) free(ramdisk_name);
  if(logo_name) free(logo_name);
  if(bootimage_name) free(bootimage_name);
  fdt_buf=NULL;
  bitstream_name=fdt_name=bootloader_name=identity_name=kernel_name
    =ramdisk_name=logo_name=bootimage_name=NULL;
}

struct bad_desc
{
  u32 start;
  u32 size;
};

int badblock_cmp(const void *arg1,const void *arg2)
{
  const struct bad_desc *b1,*b2;
  b1=(const struct bad_desc*)arg1;
  b2=(const struct bad_desc*)arg2;
  return b1->start>b2->start;
}

int nbadblocks=0;
struct bad_desc *badblocks=NULL;

int add_bad_descriptor(u32 bad_start,u32 bad_end)
{
  int retval=0;
  struct bad_desc *badblocks_new;

  if(nbadblocks==0)
    {
      badblocks=(struct bad_desc*)malloc(sizeof(struct bad_desc));
      if(!badblocks)
	{
	  fprintf(stderr,"WARNING: can't allocate bad block, "
		  "record ignored\n");
	  retval=-1;
	}
      else
	{
	  if(bad_end>=bad_start)
	    {
	      badblocks[0].start=bad_start;
	      badblocks[0].size=bad_end+1-bad_start;
	    }
	  else
	    {
	      badblocks[0].start=bad_end;
	      badblocks[0].size=bad_start+1-bad_end;
	    }
	  nbadblocks=1;
	}
    }
  else
    {
      badblocks_new=(struct bad_desc*)
	realloc(badblocks,sizeof(struct bad_desc)*nbadblocks+1);
      if(!badblocks_new)
	{
	  fprintf(stderr,"WARNING: can't allocate bad block, "
		  "record ignored\n");
	  retval=-1;
	}
      else
	{
	  badblocks=badblocks_new;
	  if(bad_end>=bad_start)
	    {
	      badblocks[nbadblocks].start=bad_start;
	      badblocks[nbadblocks].size=bad_end+1-bad_start;
	    }
	  else
	    {
	      badblocks[nbadblocks].start=bad_end;
	      badblocks[nbadblocks].size=bad_start+1-bad_end;
	    }
	  nbadblocks++;
	}
    }
  return retval;
}

/* main */
int main(int argc,char **argv)
{
  int segment_index;
  int h,opt,optindex,n,i,j,k,h_fdt,h_out,err,
    chosen_offset,bootargslen,restart,found_index,retval,
    outputtype=OUTPUT_TYPE_FILE,invert_bitstream=1;
  long fdt_len,fdt_old_size,identity_len;
  u32 bad_start,bad_end,ramdisk_new_offset,ramdisk_new_offset_kb,
    bootargsstring_new_len;
  const struct fdt_property *bootargs;
  char *bootargsstring,*bootargsstring_new,*ramdisk_start_ptr,
    *ramdisk_start_continuation_ptr,ramdisk_offset_text_string[32];
  unsigned char *identity_buf;

  struct stat statbuf;

  struct option options[]={
    {"start",1,NULL,'s'},
    {"bad",1,NULL,'B'},
    {"bit",1,NULL,'b'},
    {"dtb",1,NULL,'d'},
    {"exec",1,NULL,'e'},
    {"id",1,NULL,'i'},
    {"kernel",1,NULL,'k'},
    {"ramdisk",1,NULL,'r'},
    {"logo",1,NULL,'l'},
    {"font",1,NULL,'f'},
    {"input",1,NULL,'I'},
    {"out",1,NULL,'o'},
    {"noinvert",0,NULL,'N'},
    {NULL,0,NULL,'\0'}
  };

  char *optstring="s:B:b:d:e:i:k:r:l:f:o:I:N";

  enum{
    OPER_OUTPUT,
    OPER_INPUT
  }image_oper=OPER_OUTPUT;

  /* process command line options */
  while((opt=getopt_long(argc,argv,optstring,options,&optindex))!=-1)
    {
      switch(opt)
	{
	case 's':
	  n=sscanf(optarg,"%li",&start_address);
	  if(n<1)
	    {
	      fprintf(stderr,"WARNING: invalid start address, set to 0\n");
	      start_address=0;
	    }
	  end_address=start_address;
	  break;

	case 'B':
	  n=sscanf(optarg,"%li-%li",&bad_start,&bad_end);
	  switch(n)
	    {
	    case 0:
	      fprintf(stderr,"WARNING: invalid bad block record ignored\n");
	      break;
	    case 1:
	      fprintf(stderr,"WARNING: no bad block record end address, "
		      "assumed to be 4k long\n");
	      
	      bad_end=bad_start+4095;
	      /* fall through */
	    default:
	      add_bad_descriptor(bad_start,bad_end);
	    }
	  break;

	case 'b':
	  if(bitstream_name)
	    {
	      fprintf(stderr,"WARNING: bitstream file redefined, "
		      "from \"%s\" to \"%s\"\n", bitstream_name,optarg);
	      free(bitstream_name);
	    }
	  bitstream_name=strdup(optarg);
	  break;

	case 'd':
	  if(fdt_name)
	    {
	      fprintf(stderr,"WARNING: fdt file redefined, "
		      "from \"%s\" to \"%s\"\n", fdt_name,optarg);
	      free(fdt_name);
	    }
	  fdt_name=strdup(optarg);
	  break;

	case 'e':
	  if(bootloader_name)
	    {
	      fprintf(stderr,"WARNING: bootloader file redefined, "
		      "from \"%s\" to \"%s\"\n", bootloader_name,optarg);
	      free(bootloader_name);
	    }
	  bootloader_name=strdup(optarg);
	  break;

	case 'i':
	  if(identity_name)
	    {
	      fprintf(stderr,"WARNING: identity file redefined, "
		      "from \"%s\" to \"%s\"\n", identity_name,optarg);
	      free(identity_name);
	    }
	  identity_name=strdup(optarg);
	  break;

	case 'k':
	  if(kernel_name)
	    {
	      fprintf(stderr,"WARNING: kernel file redefined, "
		      "from \"%s\" to \"%s\"\n", kernel_name,optarg);
	      free(kernel_name);
	    }
	  kernel_name=strdup(optarg);
	  break;
	case 'r':
	  if(ramdisk_name)
	    {
	      fprintf(stderr,"WARNING: ramdisk file redefined, "
		      "from \"%s\" to \"%s\"\n", ramdisk_name,optarg);
	      free(ramdisk_name);
	    }
	  ramdisk_name=strdup(optarg);
	  break;

	case 'l':
	  if(logo_name)
	    {
	      fprintf(stderr,"WARNING: boot logo file redefined, "
		      "from \"%s\" to \"%s\"\n", logo_name,optarg);
	      free(logo_name);
	    }
	  logo_name=strdup(optarg);
	  break;

	case 'f':
	  if(fonts_count>=MAX_FONTS)
	    {
	      fprintf(stderr,"WARNING: too many fonts, font \"%s\" ignored\n",
		      optarg);
	    }
	  font_names[fonts_count]=strdup(optarg);
	  if(font_names[fonts_count])
	    {
	      fonts_count++;
	    }
	  break;

	case 'I':
	case 'o':
	  if(bootimage_name)
	    {
	      if((opt=='I'&&!image_oper==OPER_OUTPUT)
		 ||(opt=='o'&&!image_oper==OPER_INPUT))
		{
		  fprintf(stderr,"Both input and output images are specified, "
			  "choose one\n");
		  cleanup_mem();
		  return 1;
		}
	      fprintf(stderr,"WARNING: image file redefined, "
		      "from \"%s\" to \"%s\"\n", bootimage_name,optarg);
	      free(bootimage_name);
	    }
	  image_oper=(opt=='o')?OPER_OUTPUT:OPER_INPUT;
	  bootimage_name=strdup(optarg);
	  break;
	case 'N':
	  invert_bitstream=0;
	  break;
	default:
	  usage();
	  cleanup_mem();
	  return 1;
	}
    }

  if(!bootimage_name)
    {
      bootimage_name=strdup(DEFAULT_BOOTIMAGE_NAME);
    }
  
  if(!bootimage_name)
    {
      errmsg_mem();
      cleanup_mem();
      return 1;
    }
  
  
  /* if image is an input, parse it and write the files */
  if(image_oper==OPER_INPUT)
    {
      int use_header_offset,fonts_counter,file_finished;
      u32 initial_offset,initial_offset_bootloader,initial_offset_chain,
	curr_offset,curr_offset_new,bitstream_size,bootloader_size;
      unsigned char imageheaderbuffer[BOOT_IMAGE_HEADER_SIZE];
      unsigned char bitstream_fixed_header[]=
	"\x00\x09\x0f\xf0\x0f\xf0\x0f\xf0\x0f\xf0\x00\x00"
	"\x01" "a" "\x00\x08"
	"unknown" "\x00"
	"b" "\x00\x08"
	"unknown" "\x00"
	"c" "\x00\x0b"
	"1970/01/01" "\x00"
	"d" "\x00\x09"
	"00:00:00" "\x00"
	"e" "\x00\x00\x00";
      image_segment_header segmentheader;

      h=open(bootimage_name,O_RDONLY);
	if(h<0)
	  {
	    perror("Can't open input image file");
	    cleanup_mem();
	    return 1;
	  }
      if(fstat(h,&statbuf))
	{
	  perror("Can't determine input file type");
	  close(h);
	  cleanup_mem();
	  return 1;
	}
      
      if(S_ISCHR(statbuf.st_mode)
	 &&major(statbuf.st_rdev)==MTD_CHAR_MAJOR)
	{
	  initial_offset=start_address;
	  if(lseek(h,start_address,SEEK_SET)!=start_address)
	    {
	      perror("Can't seek to the start address");
	      close(h);
	      cleanup_mem();
	      return 1;
	    }
	}
      else
	{
	  initial_offset=0;
	}

      
      if(read(h,imageheaderbuffer,BOOT_IMAGE_HEADER_SIZE)
	 !=BOOT_IMAGE_HEADER_SIZE)
	{
	  fprintf(stderr,"Can't read boot image header\n");
	  close(h);
	  cleanup_mem();
	  return 1;
	}
      use_header_offset=0;
      if(!memcmp(imageheaderbuffer,BOOT_IMAGE_HEADER,
		 BOOT_IMAGE_HEADER_SIZE)
	 ||(use_header_offset=!memcmp(imageheaderbuffer,
				       BITSTREAM_IMAGE_HEADER,
				       16))
	 ||(use_header_offset=!memcmp(imageheaderbuffer,
				       BOOTLOADER_IMAGE_HEADER,
				       16)))
	{
	  if(use_header_offset)
	    {
	      initial_offset_bootloader=
		(((u32)imageheaderbuffer[17])<<24)
		+(((u32)imageheaderbuffer[19])<<16)
		+(((u32)imageheaderbuffer[21])<<8)
		+((u32)imageheaderbuffer[23]);
	      initial_offset_chain=
		(((u32)imageheaderbuffer[24])<<24)
		+(((u32)imageheaderbuffer[26])<<16)
		+(((u32)imageheaderbuffer[28])<<8)
		+((u32)imageheaderbuffer[30]);

	      if(initial_offset_chain!=0)
		initial_offset_chain-=sizeof(image_segment_header);

	      if(start_address!=initial_offset)
		{
		  if(initial_offset_bootloader!=0)
		    initial_offset_bootloader-=start_address-initial_offset;
		  if(initial_offset_chain!=0)
		    initial_offset_chain-=start_address-initial_offset;
		}

	      if(initial_offset_chain==initial_offset)
		{
		  fprintf(stderr,"Empty boot image\n");
		  close(h);
		  cleanup_mem();
		  return 1;
		}
	      if(initial_offset_bootloader!=initial_offset)
		{
		  bitstream_size=initial_offset_bootloader
		    -initial_offset
		    -sizeof(image_segment_header)
		    -BOOT_IMAGE_HEADER_SIZE;
		  bootloader_size=
		    initial_offset_chain-initial_offset_bootloader;
		}
	      else
		{
		  bitstream_size=initial_offset_chain
		    -initial_offset
		    -BOOT_IMAGE_HEADER_SIZE;
		  bootloader_size=0;
		}
	    }
	  else
	    {
	      initial_offset_bootloader=0;
	      initial_offset_chain=initial_offset;
	      bitstream_size=0;
	      bootloader_size=0;
	    }
	}
      else
	{
	  fprintf(stderr,"Unknown boot image format\n");
	  close(h);
	  cleanup_mem();
	  return 1;
	}

      if(bitstream_name)
	{
	  if(bitstream_size>0)
	    {
	      bitstream_fixed_header[sizeof(bitstream_fixed_header)-4]=
		(u8)((bitstream_size>>24)&0xff);
	      bitstream_fixed_header[sizeof(bitstream_fixed_header)-3]=
		(u8)((bitstream_size>>16)&0xff);
	      bitstream_fixed_header[sizeof(bitstream_fixed_header)-2]=
		(u8)((bitstream_size>>8)&0xff);
	      bitstream_fixed_header[sizeof(bitstream_fixed_header)-1]=
		(u8)(bitstream_size&0xff);
	      h_out=open(bitstream_name,O_WRONLY|O_CREAT|O_TRUNC,
			 S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);
	      if(h_out<0)
		{
		  perror("Can't open output bitstream file");
		}
	      else
		{
		  if(write(h_out,bitstream_fixed_header,
			   sizeof(bitstream_fixed_header))
		     !=sizeof(bitstream_fixed_header))
		    {
		      fprintf(stderr,"Can't write bitstream file header\n");
		      close(h_out);
		      unlink(bitstream_name);
		      h_out=-1;
		    }
		  if(h_out>=0
		     &&(copydata(h_out,h,bitstream_size,invert_bitstream,0)
			!=bitstream_size))
		    {
		      fprintf(stderr,"Can't write bitstream file\n");
		      close(h_out);
		      unlink(bitstream_name);
		      h_out=-1;
		    }
		  if(h_out>=0)
		    close(h_out);
		}
	    }
	  else
	    {
	      fprintf(stderr,"Bitstream is not present, file %s "
		      "is not written\n",bitstream_name);
	    }
	}
      if(bootloader_name)
	{
	  if(bootloader_size>0)
	    {
	      if(lseek(h,initial_offset_bootloader,SEEK_SET)
		 !=initial_offset_bootloader)
		perror("Can't seek to the start of bootloader");
	      else
		{
		  h_out=open(bootloader_name,O_WRONLY|O_CREAT|O_TRUNC,
			     S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);
		  if(h_out<0)
		    {
		      perror("Can't open output bootloader file");
		    }
		  else
		    {
		      if(copydata(h_out,h,bootloader_size,0,0)
			 !=bootloader_size)
			{
			  fprintf(stderr,"Can't write bootloader file\n");
			  close(h_out);
			  unlink(bootloader_name);
			  h_out=-1;
			}
		      if(h_out>=0)
			close(h_out);
		    }
		}
	    }
	  else
	    {
	      fprintf(stderr,"Bootloader is not present, file %s "
		      "is not written\n",bootloader_name);
	    }
	}

      if(lseek(h,initial_offset_chain,SEEK_SET)
	 !=initial_offset_chain)
	{
	  perror("Can't seek to the start of the images segments");
	  close(h);
	  cleanup_mem();
	  return 1;
	}
      fonts_counter=0;
      file_finished=0;
      curr_offset=initial_offset_chain;
      while(!file_finished
	    &&(read(h,&segmentheader,sizeof(image_segment_header))
	       ==sizeof(image_segment_header)))
	{
	  switch(htonl(segmentheader.image_segment_type))
	    {
	    case REC_TYPE_FDT:
	      read_extract_segment(h,htonl(segmentheader.size_padded),0,
				   fdt_name,"flat device tree");
	      break;
	    case REC_TYPE_KERNEL:
	      read_extract_segment(h,htonl(segmentheader.size_padded),0,
				   kernel_name,"kernel");
	      break;
	    case REC_TYPE_RDISK:
	      read_extract_segment(h,htonl(segmentheader.size_padded),0,
				   ramdisk_name,"ramdisk");
	      file_finished=1;
	      break;
	    case REC_TYPE_BOOT_SCREEN:
	      read_extract_segment(h,htonl(segmentheader.size_padded),0,
				   logo_name,"boot screen");
	      break;
	    case REC_TYPE_FONT:
	      if(fonts_counter<fonts_count)
		{
		  read_extract_segment(h,htonl(segmentheader.size_padded),0,
				       font_names[fonts_counter],"font");
		  fonts_counter++;
		}
	      break;
	    case REC_TYPE_END:
	      file_finished=1;
	      break;
	    case REC_TYPE_IDENTITY:
	      read_extract_segment(h,htonl(segmentheader.size_padded),1,
				   identity_name,"identity");
	      break;
	    case REC_TYPE_USER:
	      break;
	    }
	  curr_offset_new=curr_offset+sizeof(image_segment_header)
	    +htonl(segmentheader.size_padded);

	  /*
	    verify that no wraparound or zero-offset happens
	    with or without the segment header size being added
	  */
	  if((curr_offset_new<=curr_offset)
	     ||(curr_offset_new<=(curr_offset+sizeof(image_segment_header))))
	    {
	      file_finished=1;
	    }
	  else
	    {
	      curr_offset=lseek(h,curr_offset_new,SEEK_SET);
	      if(curr_offset!=curr_offset_new)
		{
		  file_finished=1;
		}
	    }
	}
      cleanup_mem();
      return 0;
    }

  /* image is an output, write it with content from files */

  /* check for mandatory arguments */
  if(!fdt_name)
    {
      fprintf(stderr,"Device tree file is not defined, "
	      "use -d or --dtb option\n");
    }

  if(!kernel_name)
    {
      fprintf(stderr,"Kernel file is not defined, "
	      "use -k or --kernel option\n");
    }
  
  if(!ramdisk_name)
    {
      fprintf(stderr,"Ramdisk file is not defined, "
	      "use -r or --ramdisk option\n");
    }
  
  if(!fdt_name||!kernel_name||!ramdisk_name)
    {
      cleanup_mem();
      return 1;
    }
  
  /* allocate buffer */
  fdt_buf=(unsigned char*)malloc(FDT_SIZE);
  
  if(!fdt_buf)
    {
      errmsg_mem();
      cleanup_mem();
      return 1;
    }

  /* device tree handling */
  /* FDT_SIZE is the limit imposed by the kernel boot procedure, enforce it */
  memset(fdt_buf,0,FDT_SIZE);
  h_fdt=open(fdt_name,O_RDONLY);
  if(h_fdt<0)
    {
      perror("Can't open flat device tree file");
      cleanup_mem();
      return 1;
    }
  fdt_len=read(h_fdt,fdt_buf,FDT_SIZE);
  if(fdt_len<0)
    {
      perror("Can't read flat device tree file");
      close(h_fdt);
      cleanup_mem();
      return 1;
    }

  if(fdt_len==FDT_SIZE){
    if(read(h_fdt,&i,1)!=0)
      {
	fprintf(stderr,"Device tree file is too long\n");
	close(h_fdt);
	cleanup_mem();
	return 1;
      }
  }
  close(h_fdt);

  err=fdt_open_into(fdt_buf,fdt_buf,FDT_SIZE);
  if(err)
    {
      fprintf(stderr,"Can't open flat device tree: %s\n",fdt_strerror(err));
	cleanup_mem();
	return 1;
    }

  fdt_pack(fdt_buf);

  /* prepare to edit the command line in the device tree */
  chosen_offset=fdt_path_offset(fdt_buf,"/chosen");
  if(chosen_offset<0)
    {
      fprintf(stderr,"Can't find \"/chosen\" node in the device tree: %s\n",
	      fdt_strerror(chosen_offset));
	cleanup_mem();
	return 1;
    }
  bootargs=fdt_get_property(fdt_buf,chosen_offset,"bootargs",&bootargslen);
  if(bootargslen<0)
    {
      fprintf(stderr,"Can't find bootargs in the device tree: %s\n",
	      fdt_strerror(bootargslen));
      cleanup_mem();
      return 1;
    }
  bootargsstring=(char*)malloc(bootargslen);
  if(!bootargsstring)
    {
      errmsg_mem();
      cleanup_mem();
      return 1;
    }
  
  memcpy(bootargsstring,(const char*)bootargs+sizeof(struct fdt_property),
	 bootargslen-1);
  bootargsstring[bootargslen-1]='\0';
  
  
  /* find ramdisk_start argument */
  ramdisk_start_ptr=strstr(bootargsstring,"ramdisk_start=");
  if(ramdisk_start_ptr)
    {
      /* find the end of the start argument string */
      for(ramdisk_start_continuation_ptr=ramdisk_start_ptr+14;
	  *ramdisk_start_continuation_ptr
	    &&!isspace(*ramdisk_start_continuation_ptr);
	  ramdisk_start_continuation_ptr++);
    }
  else
    {
      ramdisk_start_continuation_ptr=NULL;
    }

  /*
    add all segments to the image, determine ramdisk position, edit
    device tree, repeat the same procedure with new (possibly resized)
    device tree, until ramdisk position stops moving
  */
  restart=0;
  do /* loop restarts for every flash write error, if any */
    {
      /* combine and sort bad blocks, if any */
      for(i=0;i<nbadblocks-1;i++)
	{
	  for(j=i+1;j<nbadblocks;j++)
	    {
	      if(((badblocks[j].start<=(badblocks[i].start+badblocks[i].size))
		  &&((badblocks[j].start+badblocks[j].size)
		     >=badblocks[i].start))
		 ||
		 ((badblocks[i].start<=(badblocks[j].start+badblocks[j].size))
		  &&((badblocks[i].start+badblocks[i].size)
		     >=badblocks[j].start)))
		{
		  bad_start=min(badblocks[i].start,badblocks[j].start);
		  bad_end=max(badblocks[i].start+badblocks[i].size,
			      badblocks[j].start+badblocks[j].size);
		  badblocks[i].start=bad_start;
		  badblocks[i].size=bad_end-bad_start;
		  for(k=j;k<nbadblocks-1;k++)
		    {
		      badblocks[k].start=badblocks[k+1].start;
		      badblocks[k].size=badblocks[k+1].size;
		    }
		  j=i;
		  nbadblocks--;
		}
	    }
	}

      qsort(badblocks,nbadblocks,sizeof(struct bad_desc),badblock_cmp);
      
      for(i=0;i<nbadblocks;i++)
	{
	  printf("Bad block: 0x%08lx-0x%08lx\n",badblocks[i].start,
		 badblocks[i].start+badblocks[i].size-1);
	}

      restart=0;
      do /* loop restarts for every resize */
	{
	  /* remove all segments that were added on previous iterations */
	  remove_all_segments();
	  
	  /* add all segments: */
	  
	  /* bitstream (optional) */
	  if(bitstream_name)
	    {
	      segment_index=add_bit_file_segment(bitstream_name,
						 invert_bitstream);
	      /* add_bit_file_segment() reports its errors by itself */
	      if(segment_index<0)
		{
		  free(bootargsstring);
		  cleanup_mem();
		  return 1;
		}
	    }
	  else
	    {
	      /* add empty bitstream if bootloader segment is present */
	      if(bootloader_name)
		{
		  segment_index=add_empty_bit_segment();
		  /* add_empty_bit_segment() reports its errors by itself */
		  if(segment_index<0)
		    {
		      free(bootargsstring);
		      cleanup_mem();
		      return 1;
		    }
		}
	    }
	  
	  /* bootloader (optional, requires bitstream or empty bitstream) */
	  if(bootloader_name)
	    {
	      segment_index=add_file_segment(REC_TYPE_BOOTLOADER,
					     bootloader_name,0,4);
	      if(segment_index<0)
		{
		  fprintf(stderr,"Can't add bootloader file \"%s\"\n",
			  bootloader_name);
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;
		}
	    }
	  
	  /* boot image header (mandatory) */
	  segment_index=add_memory_segment(REC_TYPE_EMPTY,
					   (u8*)BOOT_IMAGE_HEADER,0,
					   BOOT_IMAGE_HEADER_SIZE,4);
	  if(segment_index<0)
	    {
	      fprintf(stderr,"Can't add image header\n");
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }
	  
	  /* fdt (mandatory) */
	  segment_index=add_memory_segment(REC_TYPE_FDT,fdt_buf,0,
					   fdt_totalsize(fdt_buf),4);
	  if(segment_index<0)
	    {
	      fprintf(stderr,"Can't add flat device tree\n");
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }
	  
	  /* kernel (mandatory) */
	  segment_index=add_file_segment(REC_TYPE_KERNEL,kernel_name,0,4);
	  if(segment_index<0)
	    {
	      fprintf(stderr,"Can't add kernel file \"%s\"\n",
		      kernel_name);
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }
	  
	  /* boot logo screen (optional) */
	  if(logo_name)
	    {
	      segment_index=add_file_segment(REC_TYPE_BOOT_SCREEN,logo_name,
					     0,4);
	      if(segment_index<0)
		{
		  fprintf(stderr,"Can't add boot logo image file \"%s\"\n",
			  logo_name);
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;
		}
	    }
	  
	  /* fonts (optional and variable number of them) */
	  for(i=0;i<fonts_count;i++)
	    {
	      segment_index=add_file_segment(REC_TYPE_FONT,font_names[i],0,4);
	      if(segment_index<0)
		{
		  fprintf(stderr,"Can't add font file \"%s\"\n",
			  font_names[i]);
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;
		}
	    }
	  
	  /* identity (optional) */
	  if(identity_name)
	    {
	      h=open(identity_name,O_RDONLY);
	      if(h<0)
		{
		  perror("Opening identity file");
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;
		}
	      if(fstat(h,&statbuf))
		{
		  perror("Determining identity file size");
		  close(h);
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;
		}
	      /* allocate buffer for identity information */
	      identity_len=statbuf.st_size;
	      identity_buf=(unsigned char*)malloc(identity_len+1);
	      if(!identity_buf)
		{
		  errmsg_mem();
		  close(h);
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;
		}
	      if(read(h,identity_buf,identity_len)!=identity_len)
		{
		  perror("Reading identity file");
		  close(h);
		  free(identity_buf);
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;
		}
	      close(h);
	      
	      identity_buf[identity_len]='\0';
	      segment_index=add_identity_segment(identity_buf,fdt_buf);
	      if(segment_index<0)
		{
		  fprintf(stderr,"Can't add identity file \"%s\"\n",
			  identity_name);
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;	      
		}
	    }
	  
	  /* ramdisk (mandatory) */
	  segment_index=add_file_segment(REC_TYPE_RDISK,ramdisk_name,0,1024);
	  if(segment_index<0)
	    {
	      fprintf(stderr,"Can't add ramdisk file \"%s\"\n",
		      ramdisk_name);
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }
	  
	  /* insert all bad blocks */
	  for(i=0;i<nbadblocks;i++)
	    {
	      segment_index=insert_bad_segment(badblocks[i].start,
					       badblocks[i].start
					       +badblocks[i].size);
	      switch(segment_index)
		{
		case -2:
		  fprintf(stderr,
			  "Bad block overlaps with the first segment\n");
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;
		case -3:
		  errmsg_mem();
		  free(bootargsstring);
		  remove_all_segments();
		  cleanup_mem();
		  return 1;
		}
	    }
	  

	  /* find rmadisk segment again (bad blocks could have moved it) */
	  for(i=0,found_index=-1;i<nsegments;i++)
	    {
	      if(segments[i].image_segment_type==REC_TYPE_RDISK)
		{
		  found_index=i;
		  i=nsegments;
		}
	    }

	  /* 
	     check if ramdisk is found -- if not, something is wrong because
	     it's always included in the image
	  */
	  if(found_index<0)
	    {
	      fprintf(stderr,"Can't find ramdisk segment\n");
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }
	  
	  /* edit the device tree */
	  fdt_old_size=fdt_totalsize(fdt_buf);
	  ramdisk_new_offset=segments[found_index].offset;
	  ramdisk_new_offset_kb=ramdisk_new_offset/0x400;
	  
	  /*
	    text must be produced when the image is built by this program,
	    bootloader is not supposed to understand numbers or resize strings
	  */
	  snprintf(ramdisk_offset_text_string,
		   sizeof(ramdisk_offset_text_string),
		   "ramdisk_start=%lu",ramdisk_new_offset_kb);
	  
	  if(ramdisk_start_ptr)
	    {
	      /* reassemble the string */
	      bootargsstring_new_len=ramdisk_start_ptr-bootargsstring
		+strlen(ramdisk_offset_text_string)
		+strlen(ramdisk_start_continuation_ptr)+1;
	    }
	  else
	    {
	      /* append the new ramdisk_start parameter */
	      bootargsstring_new_len=strlen(bootargsstring)
		+1+strlen(ramdisk_offset_text_string)+1;
	      
	    }
	  
	  bootargsstring_new=(char*)malloc(bootargsstring_new_len);
	  if(!bootargsstring_new)
	    {
	      errmsg_mem();
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }
	  
	  if(ramdisk_start_ptr)
	    {
	      /* reassemble the string */
	      memcpy(bootargsstring_new,
		     bootargsstring,
		     ramdisk_start_ptr-bootargsstring);
	      memcpy(bootargsstring_new+(ramdisk_start_ptr-bootargsstring),
		     ramdisk_offset_text_string,
		     strlen(ramdisk_offset_text_string));
	      memcpy(bootargsstring_new+(ramdisk_start_ptr-bootargsstring)
		     +strlen(ramdisk_offset_text_string),
		     ramdisk_start_continuation_ptr,
		     strlen(ramdisk_start_continuation_ptr)+1);
	    }
	  else
	    {
	      /* append the new ramdisk_start parameter */
	      strcpy(bootargsstring_new,bootargsstring);
	      /* 
		 The following is a perfectly valid application of strcat(),
		 with no potential problems because buffers are already
		 allocated based on the strings' length.
		 
		 If some "security" tool or procedure flags it as insecure, the
		 problem is in that tool or procedure.
	      */
	      strcat(bootargsstring_new," ");
	      strcat(bootargsstring_new,ramdisk_offset_text_string);
	    }
	  /* update bootargs in the device tree */
	  
	  err=fdt_open_into(fdt_buf,fdt_buf,FDT_SIZE);
	  if(err)
	    {
	      fprintf(stderr,"Can't allocate device tree: %s\n",
		      fdt_strerror(chosen_offset));
	      free(bootargsstring_new);
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }

	  chosen_offset=fdt_path_offset(fdt_buf,"/chosen");
	  if(chosen_offset<0)
	    {
	      fprintf(stderr,"Can't find \"/chosen\" node in the "
		      "device tree: %s\n",
		      fdt_strerror(chosen_offset));
	      free(bootargsstring_new);
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }

	  err=fdt_setprop(fdt_buf,chosen_offset,"bootargs",
			  bootargsstring_new,strlen(bootargsstring_new)+1);
	  free(bootargsstring_new);
	  
	  if(err)
	    {
	      fprintf(stderr,
		      "Can't update bootargs in the flat device tree: %s\n",
		      fdt_strerror(err));
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }
	  
	  fdt_pack(fdt_buf);
	  
	  chosen_offset=fdt_path_offset(fdt_buf,"/chosen");
	  if(chosen_offset<0)
	    {
	      fprintf(stderr,
	       "Can't find \"/chosen\" node in the updated device tree: %s\n",
		      fdt_strerror(chosen_offset));
	      free(bootargsstring);
	      remove_all_segments();
	      cleanup_mem();
	      return 1;
	    }
	  /* device tree updated */
	  restart=(fdt_old_size!=fdt_totalsize(fdt_buf));
	}
      while(restart);
      /* if there are no segments by now, something is seriously wrong */
      if(nsegments==0)
	{
	  fprintf(stderr,"No segments\n");
	  free(bootargsstring);
	  remove_all_segments();
	  cleanup_mem();
	  return 1;
	}
      
      /* last segment is not padded */
      segments[nsegments-1].size_padded=segments[nsegments-1].size_actual;
      /* last segment does not include a header for the next segment */
      end_address-=sizeof(image_segment_header);
      
      /* 
	 bitstream segment (if it exists at all) contains a 32-byte header
	 before the actual bitstream:
	 
	 0   "MBBL BITSTREAM"
	 14   0,0,0
	 17   bootloader address byte 0,
	 18   0,
	 19   bootloader address byte 1,
	 20   0,
	 21   bootloader address byte 2,
	 22   0,
	 23   bootloader address byte 3,
	 24   header address byte 0,
	 25   0,
	 26   header address byte 1,
	 27   0,
	 28   header address byte 2,
	 29   0,
	 30   header address byte 3,
	 31   0
	 
	 fill that header with address values for bootloader and header,
	 so bootloader will find them.
	 
	 zero bytes between address bytes, and use of odd and even addresses
	 will prevent accidental mimicry of bitstream preamble (AA 99 55 66)
	 or any of its permutations

	 if bitstream is absent, bytes 0-16 are
	 0    "MBBL BOOTLOADER"
	 15   0,0

	 with the rest of the format unchanged
      */
      if(segments[0].image_segment_type==REC_TYPE_BITSTREAM)
	{
	  for(i=0,found_index=-1;i<nsegments;i++)
	    {
	      if(segments[i].image_segment_type==REC_TYPE_BOOTLOADER)
		{
		  found_index=i;
		  i=nsegments;
		}
	    }
	  
	  if(found_index>=0)
	    {
	      segments[0].data.ptr[17]=
		(u8)((segments[found_index].offset>>24)&0xff);
	      segments[0].data.ptr[19]=
		(u8)((segments[found_index].offset>>16)&0xff);
	      segments[0].data.ptr[21]=
		(u8)((segments[found_index].offset>>8)&0xff);
	      segments[0].data.ptr[23]=
		(u8)(segments[found_index].offset&0xff);
	    }
	  
	  for(i=0,found_index=-1;i<nsegments;i++)
	    {
	      if(segments[i].image_segment_type==REC_TYPE_EMPTY)
		{
		  found_index=i;
		  i=nsegments;
		}
	    }
	  
	  if(found_index>=0)
	    {
	      segments[0].data.ptr[24]=
		(u8)((segments[found_index].offset>>24)&0xff);
	      segments[0].data.ptr[26]=
		(u8)((segments[found_index].offset>>16)&0xff);
	      segments[0].data.ptr[28]=
		(u8)((segments[found_index].offset>>8)&0xff);
	      segments[0].data.ptr[30]=
		(u8)(segments[found_index].offset&0xff);
	    }
	}
      
      
      retval=0;
      outputtype=OUTPUT_TYPE_FILE;
      
      /* output image file */
      h_out=open(bootimage_name,O_RDWR);
      if(h_out>=0)
	{
	  if(fstat(h_out,&statbuf))
	    {
	      perror("Can't determine output file type");
	      close(h_out);
	      retval=1;
	    }
	  else
	    {
	      if(S_ISCHR(statbuf.st_mode)
		 &&major(statbuf.st_rdev)==MTD_CHAR_MAJOR)
		{
		  outputtype=OUTPUT_TYPE_MTD;
		}
	      else
		{
		  close(h_out);
		  h_out=-1;
		}
	    }
	}
      
      if(h_out<0 && retval==0)
	{
	  h_out=open(bootimage_name,O_WRONLY|O_CREAT|O_TRUNC,
		     S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);
	}
      
      if(h_out<0)
	{
	  perror("Can't create output image file");
	  retval=1;
	}
      else
	{
	  if(outputtype==OUTPUT_TYPE_MTD)
	    {
	      mtd_info_t mtdinfo;
	      region_info_t *reginfo=NULL;
	      int nregions,first_block,last_block,write_result;
#ifdef DEBUG
	      printf("Output file is an MTD device\n");
#endif
	      if(ioctl(h_out,MEMGETREGIONCOUNT,&nregions))
		{
		  nregions=0;
		}
	      if(nregions)
		{
		  reginfo=(region_info_t*)malloc(sizeof(region_info_t)
						 *nregions);
		  if(!reginfo)
		    {
		      fprintf(stderr,
			      "Can't allocate data for %d regions of "
			      "MTD flash, fallback to non-region mode\n",
			      nregions);
		      nregions=0;
		    }
		}
	      if(nregions)
		{
		  for(i=0;i<nregions;i++)
		    {
		      reginfo[i].regionindex=i;
		      if(ioctl(h_out,MEMGETREGIONINFO,reginfo+i))
			{
			  fprintf(stderr,
				  "Can't read data for region %d of "
				  "MTD flash, fallback to non-region mode\n",
				  i);
			  free(reginfo);
			  nregions=0;
			}
		    }
		}
	      
	      if(nregions)
		{
		  for(i=0;i<nregions&&restart==0;i++)
		    {
#ifdef DEBUG
		      printf("Region %u, start %u, %u bytes, erase size %u\n",
			     i,reginfo[i].offset,
			     reginfo[i].numblocks*reginfo[i].erasesize,
			     reginfo[i].erasesize);
#endif
		      if((start_address<reginfo[i].offset
			 +reginfo[i].numblocks*reginfo[i].erasesize)
			 &&end_address>reginfo[i].offset)
			{
			  if(start_address>=reginfo[i].offset)
			    {
			      first_block=(start_address-reginfo[i].offset)
				/reginfo[i].erasesize;
			    }
			  else
			    first_block=0;

			  if(end_address<
			     reginfo[i].numblocks*reginfo[i].erasesize)
			    {
			      last_block=(end_address-1-reginfo[i].offset)
				/reginfo[i].erasesize;
			    }
			  else
			    last_block=reginfo[i].numblocks-1;
			  for(j=first_block;j<=last_block&&restart==0;j++)
			    {
			      write_result=write_block(h_out,
						       reginfo[i].offset
						       +reginfo[i].erasesize*j,
						       reginfo[i].erasesize,
						       outputtype);
			      if(write_result==-1)
				{
				  close(h_out);
				  free(reginfo);
				  errmsg_mem();
				  free(bootargsstring);
				  remove_all_segments();
				  cleanup_mem();
				  sync();
				  return 1;
				}
			      if(write_result<0)
				{
				  add_bad_descriptor(reginfo[i].offset
					        +reginfo[i].erasesize*j,
					        reginfo[i].offset
						+reginfo[i].erasesize*(j+1)-1);
				  restart=1;
				}
			    }
			}
		    }
		  free(reginfo);
		}
	      else
		{
		  if(ioctl(h_out,MEMGETINFO,&mtdinfo)<0)
		    {
		      fprintf(stderr,"Can't get MTD information\n");
		      retval=1;
		    }
		  else
		    {
#if DEBUG
		      printf("Device: %u bytes, erase size %u bytes\n",
			     mtdinfo.size,mtdinfo.erasesize);
#endif
		      last_block=(end_address-1)/mtdinfo.erasesize;
		      for(j=0;j<=last_block&&restart==0;j++)
			{
			  write_result=write_block(h_out,
						   mtdinfo.erasesize*j,
						   mtdinfo.erasesize,
						   outputtype);
			  if(write_result==-1)
			    {
			      close(h_out);
			      free(reginfo);
			      errmsg_mem();
			      free(bootargsstring);
			      remove_all_segments();
			      cleanup_mem();
			      sync();
			      return 1;
			    }
			  if(write_result<0)
			    {
			      add_bad_descriptor(mtdinfo.erasesize*j,
						 mtdinfo.erasesize*(j+1)-1);
			      restart=1;
			    }
			}
		    }
		}
	      if(restart)
		printf("Recovering from error");
	      sync();
	      printf("\n");
	    }
	  else
	    {
#ifdef OLD_WRITE_ALL_SEGMENTS
	      if(write_all_segments(h_out))
		{
		  unlink(bootimage_name);
		  fprintf(stderr,"Output file is not written\n");
		  retval=1;
		}
#else
	      u32 write_addr,write_end_addr;
	      write_addr=start_address;
	      do
		{
		  write_end_addr=write_addr+4096;
		  if(write_end_addr>end_address)
		    write_end_addr=end_address;
		  write_block(h_out,write_addr,write_end_addr-write_addr,
			      outputtype);
		  write_addr=write_end_addr;
		}
	      while(write_addr<end_address);
#endif
	    }
	  close(h_out);
	}
    }
  while(restart);

  /* final cleanup */
  free(bootargsstring);
  remove_all_segments();
  cleanup_mem();
  return retval;
}
