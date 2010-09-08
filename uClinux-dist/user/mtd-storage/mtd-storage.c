#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <ctype.h>
#include <errno.h>
#include <arpa/inet.h>
#include <string.h>
#include <getopt.h>

#include <linux/types.h>

#include "crc32.h"

//#define DEBUG 1

#include <mtd/mtd-user.h>
/* linux MTD character device */
#ifndef MTD_CHAR_MAJOR
#define MTD_CHAR_MAJOR 90
#endif

#define FORMAT_VERSION (htonl(1))

typedef __u32 u32;
typedef __u16 u16;
typedef __u8 u8;


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


typedef struct
{
  u32 version;
  u32 chain_start;
  u32 block_size;
  u32 next_block;
  u32 crc32;
} __attribute__((packed)) erase_block_header;

typedef struct
{
  u32 serial;
  u32 size;
  u32 npart;
  u32 data_crc32;
  u32 crc32;
} __attribute__((packed)) data_block_header;

/* common messages */
void usage(void)
{
  fprintf(stderr,
	  "mtd-storage -- user data storage read/write program\n"
	  );
}

u32 glob_user_area_start=0, glob_user_area_end=0, glob_chain_start=0;
mtd_info_t mtdinfo;
region_info_t *reginfo=NULL;
int nregions=0,region_first=0,region_end=0;


/*
  return mtd region index by offset within user area
  (assuming that user area is already found)
*/

int region_by_offset(u32 offset)
{
  int i;
  if(!nregions)
    return 0;

  for(i=0;i<nregions;i++)
    {
      if(offset+glob_user_area_start<reginfo[i].offset
	 +reginfo[i].numblocks*reginfo[i].erasesize)
	return i;
    }
  return -1;
}

/*
  determine erase block by offser within user area
  (assuming that user area is already found)
*/


int eraseblock_by_offset(erase_info_t *erase,u32 offset)
{
  int i;
  if(nregions)
    {
      i=region_by_offset(offset);
      if(i<0) return -1;
      erase->start=reginfo[i].offset+
	(offset+glob_user_area_start-reginfo[i].offset)
	/reginfo[i].erasesize
	*reginfo[i].erasesize;
      erase->length=reginfo[i].erasesize;
    }
  else
    {
      erase->start=(offset+glob_user_area_start)
	/mtdinfo.erasesize*mtdinfo.erasesize;
      erase->length=mtdinfo.erasesize;
    }
  return 0;
}


/* 
   format user area (assuming that user area is found, and global chain start
   position is filled with actual value)
*/

int format_user_area(int h)
{
  erase_block_header header;
  erase_info_t erase;
  u32 curr_offset,next_offset;
  int restart;


  /* format will restart every time global chain start position is changed */
  do
    {
      restart=0;
      
      /*
	start from the end and go to the start of the user area,
	so bad blocks found in the process will be skipped after
	they are found
      */
      curr_offset=glob_user_area_end-glob_user_area_start-1;
      next_offset=glob_chain_start;
      while(curr_offset>glob_chain_start)
	{
	  if(eraseblock_by_offset(&erase,curr_offset)<0)
	    {
	      printf("FATAL ERROR: User area not found\n");
	      sync();
	      return -1;
	    }
	  if((erase.start-glob_user_area_start)==next_offset)
	    {
	      printf("FATAL ERROR: User area has one block left\n");
	      sync();
	      return -1;
	    }
	  ioctl(h,MEMUNLOCK,&erase);
	  printf("%08x - %08x format...  \x1b[2D",erase.start,
		 erase.start+erase.length-1);
	  fflush(stdout);
	  if(ioctl(h,MEMERASE,&erase)<0)
	    {
	      printf("\x1b[3D FAILED\n");
	      if(erase.start-glob_user_area_start==glob_chain_start)
		{
		  glob_chain_start+=erase.length;
		  if(glob_chain_start>=glob_user_area_end)
		    {
		      printf("FATAL ERROR: No writable space for user area\n");
		      sync();
		      return -1;
		    }
		  else
		    {
		      if(glob_chain_start+erase.length>=glob_user_area_end)
			{
			  printf("FATAL ERROR: Only one erase block left "
				 "for user area\n");
			  return -1;
			}
		    }
		  printf("Can't erase the start of the user area\n");
		  restart=1;
		}
	      else
		{
		  printf("Skipping bad block\n");
		}
	    }
	  else
	    {
	      header.chain_start=htonl(glob_chain_start);
	      header.block_size=htonl(erase.length);
	      header.next_block=htonl(next_offset);
	      header.version=FORMAT_VERSION;
	      header.crc32=htonl(crc32(0,(unsigned char *)&header,
				       sizeof(u32)*4));
	      if((lseek(h,erase.start,SEEK_SET)==erase.start)
		 &&(write(h,&header,sizeof(header))==sizeof(header)))
		{
		  printf("\x1b[3D done\r");
		  next_offset=erase.start-glob_user_area_start;
		  ioctl(h,MEMLOCK,&erase);
		}
	      else
		{
		  printf("\x1b[3D FAILED\n");
		  if(erase.start-glob_user_area_start==glob_chain_start)
		    {
		      glob_chain_start+=erase.length;
		      if(glob_chain_start>=glob_user_area_end)
			{
			  printf("FATAL ERROR: No writable space "
				 "for user area\n");
			  return -1;
			}
		      else
			{
			  if(glob_chain_start+erase.length>=glob_user_area_end)
			    {
			      printf("FATAL ERROR: Only one erase block left "
				     "for user area\n");
			      return -1;
			    }
			}
		      printf("Can't write to the start of the user area\n");
		      restart=1;
		    }
		  else
		    {
		      printf("Skipping bad block\n");
		    }
		}
	      fflush(stdout);
	      
	    }
	  if(curr_offset>erase.length)
	    curr_offset-=erase.length;
	  else
	    curr_offset=0;
	}
      if(restart)
	{
	  printf("Recovering from error");
	}
      sync();
      printf("\n");
    }while(restart);
  return 0;
}


/*
  exclude block containing given offset, erase block that points to it
  (assuming that user area is found, and global chain start position is
  filled with actual value)
*/

int exclude_erase_block(int h,u32 offset)
{
  erase_info_t exclude,erase;
  u32 curr_offset;
  erase_block_header header;

  if(eraseblock_by_offset(&exclude,offset)<0)
    return -1;

  offset=exclude.start-glob_user_area_start;

  curr_offset=glob_chain_start;
  
  while(1)
    {
      /* find erase block that corresponds to the current offset */
      if(eraseblock_by_offset(&erase,curr_offset)<0)
	return -1;

      /* check if we are excluding the start of the chain */
      if((curr_offset==glob_chain_start) && (exclude.start==erase.start))
	{
	  /* 
	     format user area starting from the next block
	     (all other bad block information will be erased at this point,
	     and will have to be re-discovered again)
	  */
	  glob_chain_start+=exclude.length;
	  return format_user_area(h);
	}
      /* read the header */
      if((lseek(h,erase.start,SEEK_SET)==erase.start)
	 &&(read(h,&header,sizeof(header))==sizeof(header))
	 &&(header.version==FORMAT_VERSION)
	 &&(header.crc32==htonl(crc32(0,(unsigned char *)&header,
				      sizeof(u32)*4))))
	{
	  /* 
	     if next block wrapped around or contains invalid value,
	     return error
	  */
	  if((htonl(header.next_block)<=curr_offset)
	     ||(htonl(header.next_block)
		>=(glob_user_area_end-glob_user_area_start)))
	    return -1;

	  /* if next block matches, skip the excluded block */
	  if(htonl(header.next_block)==offset)
	    {
	      /* 
		 check if this is the last block, so next pointer
		 should be set to the start of the chain
	      */
	      if((htonl(header.next_block)+exclude.length)
		 >=(glob_user_area_end-glob_user_area_start))
		{
		  header.next_block=htonl(glob_chain_start);
		  /*
		    if start of the chain pointed at the last 
		    block we are excluding, then there will be
		    only one block left, return an error
		   */
		  if(curr_offset==glob_chain_start)
		    return -1;
		}
	      else
		header.next_block=htonl(htonl(header.next_block)
				      +exclude.length);
	      header.version=FORMAT_VERSION;
	      header.crc32=htonl(crc32(0,(unsigned char *)&header,
				       sizeof(u32)*4));
	      ioctl(h,MEMUNLOCK,&erase);
	      if((ioctl(h,MEMERASE,&erase)==0)
		 &&(lseek(h,erase.start,SEEK_SET)==erase.start)
		 &&(write(h,&header,sizeof(header))==sizeof(header)))
		{
		  ioctl(h,MEMLOCK,&erase);
		  sync();
		  return 0;
		}
	      else
		{
		  sync();
		  return -1;
		}
	    }
	  curr_offset=htonl(header.next_block);
	}
      else
	return -1;
    }
  return -1; /* not reached */
}

/* structure used to describe possible chain start addresses */
struct chain_start_candidate
{
  u32 offset;
  int counter;
};

/* comparison function for qsort() */
static int chain_start_cmp(const void *a, const void *b)
{
  return ((const struct chain_start_candidate*)a)->counter
    <((const struct chain_start_candidate*)b)->counter;
}


/* find start of the erase blocks chain in the user data area */

int find_chain_start(int h)
{
  int i,j,k,blockcount=0,nsizes=0;
  u32 curr_offset;
  struct chain_start_candidate *start_array;
  erase_block_header header;

  /* determine the total number of erase blocks in user area */
  if(nregions)
    {
      for(i=region_first;i<region_end;i++)
	{
	  /*
	    only count regions that have user data, assume that
	    user area starts and ends on erase blocks boundaries
	   */
	  if(reginfo[i].offset<glob_user_area_start)
	    {
	      if((reginfo[i].offset+reginfo[i].numblocks*reginfo[i].erasesize)
		 <=glob_user_area_end)
		blockcount+=reginfo[i].numblocks-
		  (glob_user_area_start-reginfo[i].offset)
		  /reginfo[i].erasesize;
	      else
		blockcount+=(glob_user_area_end-glob_user_area_start)
		  /reginfo[i].erasesize;
	    }
	  else
	    {
	      if((reginfo[i].offset+reginfo[i].numblocks*reginfo[i].erasesize)
		 >glob_user_area_end)
		  blockcount+=(glob_user_area_end-reginfo[i].offset)
		    /reginfo[i].erasesize;
	      else
		blockcount+=reginfo[i].numblocks;
	    }
	}
    }
  else
    {
      blockcount=(glob_user_area_end-glob_user_area_start)
		  /mtdinfo.erasesize;
    }

  nsizes=0;
  start_array=(struct chain_start_candidate*)
    malloc(sizeof(struct chain_start_candidate)*blockcount);
  if(start_array)
    {
      if(nregions)
	{
	  for(i=region_first;i<region_end;i++)
	    {
	      for(j=0;j<reginfo[i].numblocks;j++)
		{
		  curr_offset=reginfo[i].offset+reginfo[i].erasesize*j;
		  if((curr_offset>=glob_user_area_start)
		     &&((curr_offset+reginfo[i].erasesize)
			<=glob_user_area_end)
		     &&(lseek(h,curr_offset,SEEK_SET)==curr_offset)
		     &&(read(h,&header,sizeof(header))==sizeof(header))
		     &&(header.version==FORMAT_VERSION)
		     &&(header.crc32==htonl(crc32(0,
						 (unsigned char *)&header,
						  sizeof(u32)*4))))
		    {
		      for(k=0;k<nsizes;k++)
			{
			  if(header.chain_start==start_array[k].offset)
			    {
			      start_array[k].counter++;
			      k=nsizes+1;
			    }
			}
		      if((k==nsizes)&&(nsizes<blockcount))
			{
			  start_array[nsizes].offset=header.chain_start;
			  start_array[nsizes].counter=0;
			  nsizes++;
			}
		    }
		}
	    }
	}
      else
	{
	  for(curr_offset=glob_user_area_start;
	      curr_offset<glob_user_area_end;
	      curr_offset+=mtdinfo.erasesize)
	    {
	      if((lseek(h,curr_offset,SEEK_SET)==curr_offset)
		 &&(read(h,&header,sizeof(header))==sizeof(header))
		 &&(header.version==FORMAT_VERSION)
		 &&(header.crc32==htonl(crc32(0,
					     (unsigned char *)&header,
					      sizeof(u32)*4))))
		{
		  for(k=0;k<nsizes;k++)
		    {
		      if(header.chain_start==start_array[k].offset)
			{
			  start_array[k].counter++;
			  k=nsizes+1;
			}
		    }
		  if((k==nsizes)&&(nsizes<blockcount))
		    {
		      start_array[nsizes].offset=header.chain_start;
		      start_array[nsizes].counter=0;
		      nsizes++;
		    }
		}
	    }
	}

      if(nsizes>1)
	qsort(start_array,nsizes,sizeof(struct chain_start_candidate),
	      chain_start_cmp);

      if(nsizes>0)
	glob_chain_start=htonl(start_array[0].offset);

      free(start_array);
    }
  return nsizes?0:-1;
}

u32 expect_serial=0xffffffff,write_serial=0xffffffff,
  last_record_readable=0xffffffff,curr_record_readable=0xffffffff,
  read_ptr=0xffffffff,read_end=0xffffffff,
  write_ptr=0xffffffff,erase_needed_flag=0;
u32 curr_part=0;
int start_part_flag=0;
u32 offset_dont_overwrite=0xffffffff;


/* find the end of written data */

int find_data_end(int h)
{
  u32 curr_offset,data_offset;
  erase_block_header header;
  data_block_header data_header;
  int loopcount,found_valid;

  curr_offset=glob_chain_start;
  loopcount=0;
  found_valid=0;
  erase_needed_flag=0;

  while(loopcount<2)
    {
#ifdef DEBUG
      printf("curr_offset=%08x\n",curr_offset);
#endif
      /* read and check erase block header */
      if((lseek(h,curr_offset+glob_user_area_start,SEEK_SET)
	  ==curr_offset+glob_user_area_start)
	 &&(read(h,&header,sizeof(header))==sizeof(header))
	 &&(header.version==FORMAT_VERSION)
	 &&(header.crc32==
	    htonl(crc32(0,(unsigned char *)&header,sizeof(u32)*4))))
	{
	  /* read and check first data header in erase block */
	  if((read(h,&data_header,sizeof(data_header))==sizeof(data_header))
	     &&(data_header.crc32==htonl(crc32(0,(unsigned char*)&data_header,
					       sizeof(u32)*4))))
	    {
#ifdef DEBUG
	      printf("valid data_header\n");
#endif	      
	      if(expect_serial==0xffffffff)
		expect_serial=htonl(data_header.serial);
	      else
		{
		  expect_serial++;
		  if((expect_serial==0)||(expect_serial==0xffffffff))
		    expect_serial=1;
#ifdef DEBUG
		  printf("expect_serial incremented to %08x\n",expect_serial);
#endif
		}

	      found_valid=1;
	      
	      if(htonl(data_header.serial)!=expect_serial)
		{
#ifdef DEBUG
		  printf("serial number mismatch, expected %08x, got %08x\n",
			 expect_serial,htonl(data_header.serial));
#endif
		  /* 
		     found a record out of sequence,
		     this is the end of records
		  */
		  if(last_record_readable!=0xffffffff)
		    {
#ifdef DEBUG
		      printf("last record position known, %08x\n",last_record_readable);
#endif
		      /* only exit if the start of the last record is found */
		      
		      /* block has to be erased before writing */
		      erase_needed_flag=1;
		      write_ptr=curr_offset+sizeof(header);
		      read_end=write_ptr;
		      write_serial=expect_serial;
#ifdef DEBUG
		      printf("1 write offset %08x, serial %08x (erase needed)\n",write_ptr,write_serial);
#endif
		      return 0;
		    }
		  else
		    {
#ifdef DEBUG
		      printf("** last record position not known, looping **\n");
#endif
		      /* switch to the new serial, look through the chain */
		      expect_serial=htonl(data_header.serial);
		    }
		}
	      else
		{
		  /* 
		     if this is the beginning of the record,
		     record it as "last start"
		  */
		  if(htonl(data_header.npart)==0)
		    {
		      last_record_readable=curr_offset+sizeof(header);
		      curr_record_readable=last_record_readable;
		      read_ptr=last_record_readable;
		      start_part_flag=1;
#ifdef DEBUG
		      printf("record start at %08x\n",last_record_readable);
#endif
		    }
		  
		}

	      /* read the rest of the records */
	      data_offset=curr_offset+sizeof(header)+sizeof(data_header)
		+htonl(data_header.size);
#ifdef DEBUG
	      printf("reading records after the first, offset %08x\n",data_offset);
#endif
	      while(data_offset<(curr_offset+htonl(header.block_size)
				 -sizeof(data_header)))
		{
#ifdef DEBUG
		  printf("reading record, offset %08x\n",data_offset);
#endif
		  /* read and check  data header */
		  if((lseek(h,data_offset+glob_user_area_start,SEEK_SET)
		      ==data_offset+glob_user_area_start)
		     &&(read(h,&data_header,sizeof(data_header))
			==sizeof(data_header))
		     &&(data_header.crc32==htonl(crc32(0,(unsigned char*)
						       &data_header,
						       sizeof(u32)*4))))
		    {
#ifdef DEBUG
		      printf("record found\n");
#endif
		      expect_serial++;
		      if((expect_serial==0)||(expect_serial==0xffffffff))
			expect_serial=1;
#ifdef DEBUG
		      printf("expect_serial incremented to %08x\n",expect_serial);
#endif
		      /* 
			 if this is the beginning of the record,
			 record it as "last start"
		      */
		      if(htonl(data_header.npart)==0)
			{
			  last_record_readable=data_offset;
			  curr_record_readable=last_record_readable;
			  read_ptr=last_record_readable;
			  start_part_flag=1;
#ifdef DEBUG
			  printf("record start at %08x\n",last_record_readable);
#endif
			}

		      data_offset+=sizeof(data_header)+htonl(data_header.size);

		    }
		  else
		    {
		      /* no data, writable area */
#ifdef DEBUG
		      printf("record not found\n");
#endif
		      if(last_record_readable!=0xffffffff)
			{
#ifdef DEBUG
			  printf("last record position known, %08x\n",last_record_readable);
#endif
			  /* 
			     only exit if the start of the last record
			     is found
			  */
			  write_ptr=data_offset;
			  read_end=write_ptr;

			  expect_serial++;
			  if((expect_serial==0)||(expect_serial==0xffffffff))
			    expect_serial=1;

#ifdef DEBUG
			  printf("expect_serial incremented to %08x\n",expect_serial);
#endif
			  write_serial=expect_serial;
#ifdef DEBUG
			  printf("write offset %08x, serial %08x\n",write_ptr,write_serial);
#endif
			  return 0;
			}
		      else
			{
#ifdef DEBUG
			  printf("** last record position not known, skipping to the end of the block and looping **\n");
#endif
			  /* skip to the end of erase block */
			  data_offset=curr_offset+htonl(header.block_size);
			}
		      
		    }
		  
		}
	      
	    }
	  else
	    {
              /* 
                 no data in this erase block,
                 assume that this is the end of records, prepare to write
              */
              write_ptr=curr_offset+sizeof(header);
	      if(last_record_readable!=0xffffffff)
		{
		  read_end=write_ptr;
		}

	      expect_serial++;
	      if((expect_serial==0)||(expect_serial==0xffffffff))
		expect_serial=1;
#ifdef DEBUG
	      printf("expect_serial incremented to %08x\n",expect_serial);
#endif
              write_serial=expect_serial;

#ifdef DEBUG
              printf("empty block -- write offset %08x, serial %08x\n",write_ptr,write_serial);
#endif
              return 0;
	    }
	}
      else
	{
	  /* flash has to be formatted, can't be used in this condition */
#ifdef DEBUG
	  printf("not formatted\n");
#endif
	  return -1;
	}
      
      /* check if next block is valid */
      if((htonl(header.next_block)>=(glob_user_area_end-glob_user_area_start))
	 ||(htonl(header.next_block)<glob_chain_start))
	{
	  return -1;
	}

      /* check if returning to the start of the user flash area */
      if(htonl(header.next_block)<=curr_offset)
	loopcount++;

      curr_offset=htonl(header.next_block);
#ifdef DEBUG
      printf("next block, offset %08x\n",curr_offset);
#endif
    }
#ifdef DEBUG
  printf("exceeded loop count\n");
#endif
  return -1;
}


/* read data from the last found data record */

size_t read_data(int h,unsigned char *buffer,size_t size)
{
  erase_info_t erase;
  erase_block_header header;
  data_block_header data_header;
  size_t orig_size=size;
  u32 data_left,read_size;

  if(curr_record_readable==0xffffffff)
    return 0;

  /* find the erase block that contains current record */
  if(eraseblock_by_offset(&erase,curr_record_readable)<0)
    return 0;

  /* read current erase block header */
  if((lseek(h,erase.start,SEEK_SET)==erase.start)
     &&(read(h,&header,sizeof(header))==sizeof(header))
     &&(header.version==FORMAT_VERSION)
     &&(header.crc32==
	htonl(crc32(0,(unsigned char *)&header,sizeof(u32)*4))))
    {
#ifdef DEBUG
      printf("read erase block header\n");
#endif
      while(1)
	{
	  /* 
	     if we are not at the end of readable area, read
	     current data header
	     
	     it is possible that start and end are the same, so if
	     start_part_flag is set, this does not apply
	  */
#ifdef DEBUG
	  printf("curr_record_readable=%08x, read_end=%08x\n",curr_record_readable,read_end);
#endif
	  if((start_part_flag||(curr_record_readable!=read_end))
	     &&(lseek(h,curr_record_readable+glob_user_area_start,SEEK_SET)
	      ==curr_record_readable+glob_user_area_start)
	     &&(read(h,&data_header,sizeof(data_header))
		==sizeof(data_header))
	     &&(data_header.crc32==htonl(crc32(0,(unsigned char*)
					       &data_header,
					       sizeof(u32)*4))))
	    {
#ifdef DEBUG
	      printf("read data block header\n");
#endif
	      /* 
		 check for repeating part 0 -- if reached, this is the end
		 of the new sequence
	      */
	      if(!start_part_flag&&data_header.npart==0)
		{
#ifdef DEBUG
		  printf("read part 0 again\n");
#endif
		  return orig_size-size;
		}

	      if(read_ptr<(curr_record_readable+sizeof(data_header)))
		read_ptr=curr_record_readable+sizeof(data_header);
	      data_left=curr_record_readable
		+sizeof(data_header)
		+htonl(data_header.size)
		-read_ptr;
	      if(data_left>size)
		read_size=size;
	      else
		read_size=data_left;
	      
#ifdef DEBUG
	      printf("read_size=%d\n",read_size);
#endif
	      /* seek if necessary, read data */
	      if(((read_ptr==(curr_record_readable+sizeof(data_header)))
		  ||(lseek(h,read_ptr+glob_user_area_start,SEEK_SET)
		     ==read_ptr+glob_user_area_start))
		 &&(read(h,buffer,read_size)==read_size))
		{
		  size-=read_size;
		  buffer+=read_size;
		  read_ptr+=read_size;
		  data_left-=read_size;
#ifdef DEBUG
		  printf("%ld bytes left\n",size);
#endif

		  if(data_left==0)
		    {
#ifdef DEBUG
		      printf("end of record\n");
#endif
		      /* at this point we are at the end of the record */
		      curr_record_readable=read_ptr;

		      /* 
			 reset start part flag -- any subsequent part 0
			 will be a start of a new sequence
		      */
		      start_part_flag=0;

		      /* is this the end on the block? */
		      if((erase.start+erase.length
			  -curr_record_readable
			  -glob_user_area_start)<=sizeof(data_header))
			{
#ifdef DEBUG
			  printf("end of erase block\n");
#endif
			  /* if so, move to another erase block */

			  /* check if next block is valid */
			  if((htonl(header.next_block)
			      >=(glob_user_area_end-glob_user_area_start))
			     ||(htonl(header.next_block)<glob_chain_start))
			    {
			      return orig_size-size;
			    }

			  erase.start=htonl(header.next_block)
			    +glob_user_area_start;
			  if((lseek(h,erase.start,SEEK_SET)==erase.start)
			     &&(read(h,&header,sizeof(header))==sizeof(header))
			     &&(header.version==FORMAT_VERSION)
			     &&(header.crc32==
				htonl(crc32(0,(unsigned char *)&header,
					    sizeof(u32)*4))))
			    {
#ifdef DEBUG
			      printf("start of block\n");
#endif
			      erase.length=htonl(header.block_size);
			      curr_record_readable=erase.start
				-glob_user_area_start+sizeof(header);
			      read_ptr=curr_record_readable;
			    }
			  else
			    return orig_size-size;
			}
		    }

		  if(size==0)
		    return orig_size;
		}
	      else
		return orig_size-size;
	    }
	  else
	    return orig_size-size;
	}
    }
  else
    return 0;
}


/* write data into a new record or continue writing the current one */

size_t record_data(int h,unsigned char *buffer, size_t size,int continue_flag)
{
  u32 offset_in_block,space_left,write_size;
  erase_block_header header;
  data_block_header data_header;
  erase_info_t erase;
  size_t orig_size=size;

  if(write_ptr==0xffffffff)
    find_data_end(h);
  if(write_ptr==0xffffffff)
    return -1;
  if(eraseblock_by_offset(&erase,write_ptr)<0)
    return -1;

  if(continue_flag)
    {
      /* if continuing, check if overwriting our own start offset */
      if(((erase.start+erase.length)>offset_dont_overwrite)
	 &&(write_ptr<=offset_dont_overwrite))
	{
#ifdef DEBUG
	  printf("*** short write ***\n");
#endif
	  return 0;
	}
    }
  else
    {
      /* record start offset and part 0 */
      offset_dont_overwrite=write_ptr;
      curr_part=0;
    }

  data_header.npart=curr_part;

  /* read current erase block header */
  if((lseek(h,erase.start,SEEK_SET)==erase.start)
     &&(read(h,&header,sizeof(header))==sizeof(header))
     &&(header.version==FORMAT_VERSION)
     &&(header.crc32==
	htonl(crc32(0,(unsigned char *)&header,sizeof(u32)*4))))
    {
      offset_in_block=write_ptr+glob_user_area_start-erase.start;
      space_left=erase.length-offset_in_block;

      /* 
	 add data into a block if erase is not requested, and current
	 block can hold more than a data header
      */
      if(!erase_needed_flag
	 &&(space_left>sizeof(data_header)))
	{
	  space_left-=sizeof(data_header);
	  if(size>=space_left)
	    write_size=space_left;
	  else
	    write_size=size;
#ifdef DEBUG
	  printf("writing %d bytes into current block\n",write_size);
#endif
	  data_header.serial=htonl(write_serial);
	  data_header.size=htonl(write_size);
	  data_header.data_crc32=htonl(crc32(0,buffer,write_size));
	  data_header.crc32=htonl(crc32(0,(unsigned char *)&data_header,
					sizeof(u32)*4));
	  ioctl(h,MEMUNLOCK,&erase);
	  if((lseek(h,erase.start+offset_in_block,SEEK_SET)
	      ==erase.start+offset_in_block)
	     &&(write(h,&data_header,sizeof(data_header))==sizeof(data_header))
	     &&(write(h,buffer,write_size)==write_size))
	    {
	      /* increment part, move write pointer */
	      curr_part++;
	      data_header.npart=htonl(curr_part);
	      buffer+=write_size;
	      size-=write_size;
	      write_ptr+=write_size+sizeof(data_header);
	      /* increment serial number */
	      write_serial++;
	      if(write_serial==0||write_serial==0xffffffff)
		write_serial=1;
	      ioctl(h,MEMLOCK,&erase);
	      sync();
	    }
	  else 
	    {
	      sync();
	      return -1;
	    }
	}
      while(size)
	{
	  /*
	    if erase is needed, it means that we are already at
	    the start of the new block -- if not, move to the next block first
	  */
	  if(!erase_needed_flag)
	    {
	      /* check if next block is valid */
	      if((htonl(header.next_block)
		  >=(glob_user_area_end-glob_user_area_start))
		 ||(htonl(header.next_block)<glob_chain_start))
		{
		  sync();
		  return -1;
		}
	      erase.start=htonl(header.next_block)+glob_user_area_start;
	    }

	  if((lseek(h,erase.start,SEEK_SET)==erase.start)
	     &&(read(h,&header,sizeof(header))==sizeof(header))
	     &&(header.version==FORMAT_VERSION)
	     &&(header.crc32==
		htonl(crc32(0,(unsigned char *)&header,sizeof(u32)*4))))
	    {
	      /* erase and write */
	      erase.length=htonl(header.block_size);

	      /* 
		 if we are trying to overwrite our own start of the record,
		 bail with short write

		 if erase_needed_flag is in effect, we have already checked,
		 and possibly set the new value we are overwriting now
	      */
		 
	      if(!erase_needed_flag)
		{
		  if(((erase.start+erase.length)>offset_dont_overwrite)
		     &&(erase.start<=offset_dont_overwrite))
		    {
#ifdef DEBUG
		      printf("*** short write ***\n");
#endif
		      return orig_size-size;
		    }
		}

	      space_left=erase.length-sizeof(header)-sizeof(data_header);
	      if(size>=space_left)
		write_size=space_left;
	      else
		write_size=size;
#ifdef DEBUG
	      printf("writing %d bytes into erased block\n",write_size);
#endif
	      data_header.serial=htonl(write_serial);
	      data_header.size=htonl(write_size);
	      data_header.data_crc32=htonl(crc32(0,buffer,write_size));
	      data_header.crc32=htonl(crc32(0,(unsigned char *)&data_header,
					    sizeof(u32)*4));
	      ioctl(h,MEMUNLOCK,&erase);
	      if((ioctl(h,MEMERASE,&erase)==0)
		 &&(lseek(h,erase.start,SEEK_SET)==erase.start)
		 &&(write(h,&header,sizeof(header))==sizeof(header))
		 &&(write(h,&data_header,sizeof(data_header))
		    ==sizeof(data_header))
		 &&(write(h,buffer,write_size)==write_size))
		{
		  /* block is now erased and contains data */
		  erase_needed_flag=0;
		  /* increment part, move write pointer */
		  curr_part++;
		  data_header.npart=htonl(curr_part);
		  buffer+=write_size;
		  size-=write_size;
		  write_ptr=erase.start-glob_user_area_start
		    +sizeof(header)+sizeof(data_header)+write_size;
		  /* increment serial number */
		  write_serial++;
		  if(write_serial==0||write_serial==0xffffffff)
		    write_serial=1;
		  ioctl(h,MEMLOCK,&erase);
		  sync();
		}
	      else
		{
		  sync();
		  return -1;
		}
	    }
	  else
	    return -1;
	}
      /* success */
      return orig_size;
    }
  else
    return -1;
}

/* main */
int main(int argc,char **argv,char **env)
{
  int h,opt,optindex,n,i,reg_scan_status,
    format_device_flag=0,format_if_needed_flag=0;
  struct stat statbuf;

  char *dev_name=NULL;
  u32 start_address=0,end_offset=(u32)-1,device_size=0;


  u32 curr_offset,curr_offset_new;

  struct option options[]={
    {"start",1,NULL,'s'},
    {"end",1,NULL,'e'},
    {"input",1,NULL,'I'},
    {"out",1,NULL,'o'},
    {"format",0,NULL,'F'},
    {"format-if-needed",0,NULL,'f'},
    {NULL,0,NULL,'\0'}
  };

  char *optstring="s:e:o:I:Ff";

  enum{
    OPER_OUTPUT,
    OPER_INPUT
  }image_oper=OPER_INPUT;

  while((opt=getopt_long(argc,argv,optstring,options,&optindex))!=-1)
    {
      switch(opt)
	{
	case 's':
	  n=sscanf(optarg,"%i",&start_address);
	  if(n<1)
	    {
	      fprintf(stderr,"WARNING: invalid start address, set to 0\n");
	      start_address=0;
	    }
	  break;
	case 'e':
	  n=sscanf(optarg,"%i",&end_offset);
	  if(n<1)
	    {
	      fprintf(stderr,"WARNING: invalid end address, "
		      "set to end of device\n");
	      end_offset=(u32)-1;
	    }
	  else
	    {
	      end_offset++;
	    }
	  break;
	case 'F':
	  format_device_flag=1;
	  break;
	case 'f':
	  format_if_needed_flag=1;
	  break;
	case 'I':
	case 'o':
	  if(dev_name)
	    {
	      if((opt=='I'&&!image_oper==OPER_OUTPUT)
		 ||(opt=='o'&&!image_oper==OPER_INPUT))
		{
		  fprintf(stderr,"Both input and output devices are "
			  "specified, choose one\n");
		  return 1;
		}
	      fprintf(stderr,"WARNING: device name redefined, "
		      "from \"%s\" to \"%s\"\n", dev_name,optarg);
	      free(dev_name);
	    }
	  image_oper=(opt=='o')?OPER_OUTPUT:OPER_INPUT;
	  dev_name=strdup(optarg);
	  break;
	default:
	  usage();
	  return 1;
	}
    }

  if(!dev_name)
    {
      usage();
      return 1;
    }

  if(image_oper==OPER_INPUT&&format_device_flag)
    {
      fprintf(stderr,"Format is a write operation, specify output device\n");
      return 1;
    }

  if(image_oper==OPER_INPUT)
    h=open(dev_name,O_RDONLY);
  else
    h=open(dev_name,O_RDWR);
    
  if(h<0)
    {
      perror("Can't open device");
      return 1;
    }
  if(fstat(h,&statbuf))
    {
      perror("Can't determine device or file type");
      close(h);
      return 1;
    }
  else
    {
      if(!S_ISCHR(statbuf.st_mode)
	 ||major(statbuf.st_rdev)!=MTD_CHAR_MAJOR)
	{
	  fprintf(stderr,"Not an MTD device\n");
	  close(h);
	  return 1;
	}
    }

  if(ioctl(h,MEMGETREGIONCOUNT,&nregions))
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
	  if(ioctl(h,MEMGETREGIONINFO,reginfo+i))
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
      for(i=0;i<nregions;i++)
	{
#ifdef DEBUG
	  printf("Region %u, start %u, %u bytes, erase size %u\n",
		 i,reginfo[i].offset,
		 reginfo[i].numblocks*reginfo[i].erasesize,
		 reginfo[i].erasesize);
#endif
	  device_size=reginfo[i].offset
	    +reginfo[i].numblocks*reginfo[i].erasesize;
	}
    }
  else
    {
      if(ioctl(h,MEMGETINFO,&mtdinfo)<0)
	{
	  fprintf(stderr,"Can't get MTD information\n");
	  close(h);
	  return 1;
	}
      else
	{
	  device_size=mtdinfo.size;
#if DEBUG
	  printf("Device: %u bytes, erase size %u bytes\n",
		 mtdinfo.size,mtdinfo.erasesize);
#endif
	}
    }

  if(end_offset==(u32)-1)
    end_offset=device_size;


  if(end_offset<=start_address)
    {
      fprintf(stderr,"User area ends before it starts\n");
      close(h);
      return 1;
    }

  curr_offset=lseek(h,start_address,SEEK_SET);
  if(curr_offset!=start_address)
    {
      fprintf(stderr,"Can't seek to %08x on device %s\n",
	      start_address,dev_name);
      close(h);
      return 1;
    }

  /* we are at the beginning of the MBBL image, find the chain */

  int use_header_offset;
  u32 initial_offset_chain,offset_within_region;
  unsigned char imageheaderbuffer[BOOT_IMAGE_HEADER_SIZE];
  image_segment_header segment_header;

  if(read(h,imageheaderbuffer,BOOT_IMAGE_HEADER_SIZE)
     !=BOOT_IMAGE_HEADER_SIZE)
    {
      fprintf(stderr,"Can't read boot image header\n");
      close(h);
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
	  initial_offset_chain=
	    (((u32)imageheaderbuffer[24])<<24)
	    +(((u32)imageheaderbuffer[26])<<16)
	    +(((u32)imageheaderbuffer[28])<<8)
	    +((u32)imageheaderbuffer[30]);
	  
	  if(initial_offset_chain!=0)
	    initial_offset_chain-=sizeof(segment_header);
	}
      else
	{
	  initial_offset_chain=start_address;
	}
    }
  else
    {
      fprintf(stderr,"Unknown boot image format\n");
      close(h);
      return 1;
    }
  curr_offset=lseek(h,initial_offset_chain,SEEK_SET);
  if(curr_offset!=initial_offset_chain)
    {
      fprintf(stderr,"Can't seek to 0x%08x on device %s\n",
	      start_address,dev_name);
      close(h);
      return 1;
    }
 
  /* we are at the beginning of the MBBL chain, find its end */

  do
    {
      if(read(h,&segment_header,sizeof(segment_header))
	      !=sizeof(segment_header))
	{
	  fprintf(stderr,"Can't read segment header at 0x%08x on device %s\n",
		  curr_offset,dev_name);
	  close(h);
	  return 1;
	}
      curr_offset_new=curr_offset+sizeof(segment_header)
	+htonl(segment_header.size_padded);
      if(curr_offset_new<=curr_offset)
	{
	  fprintf(stderr,"Invalid segment header at 0x%08x on device %s\n",
		  curr_offset,dev_name);
	  close(h);
	  return 1;
	}
      curr_offset=lseek(h,curr_offset_new,SEEK_SET);
      if(curr_offset!=curr_offset_new)
	{
	  fprintf(stderr,"Can't seek to 0x%08x on device %s\n",
		  start_address,dev_name);
	  close(h);
	  return 1;
	}
    }
  while((segment_header.image_segment_type!=REC_TYPE_RDISK)
	&&(segment_header.image_segment_type!=REC_TYPE_END));

#ifdef DEBUG
  printf("Chain ends at 0x%08x\n",curr_offset);
#endif

  if(nregions)
    {

      /* find regions for start and end of user area */

      for(i=0,reg_scan_status=0;i<nregions&&reg_scan_status!=2;i++)
	{
	  if(reg_scan_status==0)
	    {
	      if(curr_offset<reginfo[i].offset
		 +reginfo[i].numblocks*reginfo[i].erasesize)
		{
		  region_first=i;
		  reg_scan_status=1;
		}
	    }
	  if(reg_scan_status==1)
	    {
	      if(end_offset<=reginfo[i].offset
		 +reginfo[i].numblocks*reginfo[i].erasesize)
		{
		  region_end=i+1;
		  reg_scan_status=2;
		}
	    }
	}
      if(reg_scan_status==0)
	{
	  fprintf(stderr,"Start address does not exist on the device\n");
	  close(h);
	  return 1;
	}
      if(reg_scan_status==1)
	{
	  region_end=nregions;
	  end_offset=reginfo[nregions-1].offset
	    +reginfo[nregions-1].numblocks*reginfo[nregions-1].erasesize;
	}

      /* find start of the first erase block */

      offset_within_region=curr_offset-reginfo[region_first].offset;
      if(offset_within_region%reginfo[region_first].erasesize)
	{
	  offset_within_region=(offset_within_region
				/reginfo[region_first].erasesize+1)
	    *reginfo[region_first].erasesize;
	  curr_offset=reginfo[region_first].offset+offset_within_region;
	  if(curr_offset>=reginfo[region_first].offset
	     +reginfo[region_first].numblocks*reginfo[region_first].erasesize)
	    {
	      region_first++;
	      curr_offset=reginfo[region_first].offset;
	    }
	}

      /* find end of the last erase block */
      
      offset_within_region=end_offset-reginfo[region_end-1].offset;
      if(offset_within_region%reginfo[region_end-1].erasesize)
	{
	  offset_within_region=(offset_within_region
				/reginfo[region_end-1].erasesize)
	    *reginfo[region_end-1].erasesize;
	  end_offset=reginfo[region_end-1].offset+offset_within_region;
	  if(end_offset<=reginfo[region_end-1].offset)
	    {
	      if(region_end==0)
		{
		  fprintf(stderr,"End address does not exist on device\n");
		  close(h);
		  return 1;
		}
	      region_end--;
	      end_offset=reginfo[region_end-1].offset
	      +reginfo[region_end-1].numblocks*reginfo[region_end-1].erasesize;
	    }
	  
	}
    }
  else
    {
      /* set erase block boundaries for devices without regions */

      /* find start of the first erase block */

      if(curr_offset%mtdinfo.erasesize)
	{
	  curr_offset=(curr_offset/mtdinfo.erasesize+1)*mtdinfo.erasesize;
	}

      /* find start of the first erase block */

      if(end_offset%mtdinfo.erasesize)
	{
	  end_offset=(end_offset/mtdinfo.erasesize)*mtdinfo.erasesize;
	}

    }
  
  if(end_offset<=curr_offset)
    {
      fprintf(stderr,"User area ends before it starts\n");
      close(h);
      return 1;
    }
#ifdef DEBUG
  printf("User area 0x%08x-0x%08x\n",curr_offset,end_offset-1);
#endif
  glob_user_area_start=curr_offset;
  glob_user_area_end=end_offset;

  if(format_device_flag)
    {
      printf("Formatting user area\n");
      if(format_user_area(h)==0)
	{
	  printf("Format completed successfully\n");
	}
      else
	return 1;
    }
  if(find_chain_start(h)==0)
    {
#ifdef DEBUG
      printf("Chain start %08x\n",glob_chain_start);
#endif
    }
  else
    {
      if((image_oper==OPER_OUTPUT)
	 &&format_if_needed_flag)
	{
	  printf("User data area not formatted, formatting\n");
	  if(format_user_area(h)==0)
	    {
	      printf("Format completed successfully\n");
	    }
	  else
	    return 1;
	  
	  if(find_chain_start(h)!=0)
	    {
	      fprintf(stderr,"FATAL ERROR: Can't format user area\n");
	      return 1;
	    }
	}
      else 
	return 1;
    }
  
  if(find_data_end(h))
    {
      if((image_oper==OPER_OUTPUT)
	 &&format_if_needed_flag)
	{
	  printf("User data area not formatted, formatting\n");
	  if(format_user_area(h)==0)
	    {
	      printf("Format completed successfully\n");
	    }
	  else
	    return 1;
	  
	  if(find_chain_start(h)!=0)
	    {
	      fprintf(stderr,"FATAL ERROR: Can't format user area\n");
	      return 1;
	    }
	  if(find_data_end(h)!=0)
	    {
	      fprintf(stderr,"FATAL ERROR: Can't format user area\n");
	      return 1;
	    }
	}
      else 
	return 1;
      fprintf(stderr,"FATAL ERROR: Can't find format in user area\n");
      return 1;
    }

  /* if output is requested yet no arguments are supplied, exit */
  if((image_oper==OPER_OUTPUT)&&(argc==optind))
    return 0;
  
  char **tar_argv;
  static unsigned char buffer[4096];
  int tar_pipe[2];
  int tar_argc,j,l,status;
  pid_t pid;


  tar_argc=argc-optind+2;
  tar_argv=malloc(sizeof(char*)*(tar_argc+1));
  if(!tar_argv)
    {
      fprintf(stderr,"FATAL ERROR: Insufficient memory\n");
      return 1;
    }

  tar_argv[0]="/bin/tar";
  for(i=2,j=optind;j<argc;i++,j++)
    tar_argv[i]=argv[j];
  tar_argv[tar_argc]=NULL;
  if(pipe(tar_pipe))
    {
      perror("pipe");
      free(tar_argv);
      close(h);
      return 1;
    }

  if(image_oper==OPER_INPUT)
    {
      tar_argv[1]="xvz";
      pid=vfork();
      if(pid<0)
	{
	  perror("vfork");
	  free(tar_argv);
	  close(h);
	  return 1;
	}
      if(pid)
	{
	  close(tar_pipe[0]);
	  while((l=read_data(h,buffer,sizeof(buffer)))>0)
	    {
	      if(write(tar_pipe[1],buffer,l)!=l)
		{
		  perror("write");
		  free(tar_argv);
		  close(tar_pipe[1]);
		  waitpid(pid,&status,0);
		  close(h);
		  return 1;
		}
	    }
	  close(tar_pipe[1]);
	  waitpid(pid,&status,0);
	  return 0;
	  
	}
      else
	{
	  close(tar_pipe[1]);
	  if(tar_pipe[0]!=0)
	    dup2(tar_pipe[0],0);
	  execve(tar_argv[0],tar_argv,env);
	  perror("exec");
	  _exit(1);
	}
    }
  else
    {
      tar_argv[1]="cvz";
      pid=vfork();
      if(pid<0)
	{
	  perror("vfork");
	  free(tar_argv);
	  close(h);
	  return 1;
	}
      if(pid)
	{
	  int cont=0;
	  close(tar_pipe[1]);
	  while((l=read(tar_pipe[0],buffer,sizeof(buffer)))>0)
	    {
	      if(record_data(h,buffer,l,cont)!=l)
		{
		  cont=1;
		  fprintf(stderr,"FATAL ERROR: flash write error\n");
		  free(tar_argv);
		  close(tar_pipe[0]);
		  waitpid(pid,&status,0);
		  close(h);
		  return 1;
		}
	    }
	  close(tar_pipe[0]);
	  waitpid(pid,&status,0);
	  return 0;
	}
      else
	{
	  close(tar_pipe[0]);
	  if(tar_pipe[1]!=1)
	    dup2(tar_pipe[1],1);
	  signal(SIGCLD,SIG_DFL);
	  execve(tar_argv[0],tar_argv,env);
	  perror("exec");
	  _exit(1);
	}
    }

  close(h);
  return 0;
}
