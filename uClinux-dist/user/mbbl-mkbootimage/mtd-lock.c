/*
  mtd-lock.c -- MTD flash lock/unlock utility.
*/
#include <stdio.h>
#include <string.h>
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

#define DEFAULT_BOOTIMAGE_NAME "/dev/mtd0"

typedef __u32 u32;
typedef __u16 u16;
typedef __u8 u8;

/* common messages */
void usage(void)
{
  fprintf(stderr,
	  "mtd-lock -- MTD flash lock/unlock utility\n\n"
	  "Usage:\n"
	  "Lock:\n"
	  "mtd-lock [-l] [-d <device>]\n"
	  "or\n"
	  "mtd-lock [--lock] [--device=<device>]\n"
	  "Unlock:\n"
	  "mtd-lock -u [-d <device>]\n"
	  "or\n"
	  "mtd-lock --unlock [--device=<device>]\n\n"
	  "Default device is /dev/mtd0\n"
	  );
}
/* this message is very common */
void errmsg_mem(void)
{
  fprintf(stderr,"Insufficient memory\n");
}

int main(int argc,char **argv)
{
  int opt,optindex,i,j;
  int h_flash;
  char *bootimage_name=NULL;
  struct stat statbuf;
  mtd_info_t mtdinfo;
  region_info_t *reginfo=NULL;
  erase_info_t erase;
  int nregions,first_block,last_block,op_result;

  struct option options[]={
    {"lock",0,NULL,'l'},
    {"unlock",0,NULL,'u'},
    {"device",1,NULL,'d'},
    {NULL,0,NULL,'\0'}
  };

  char *optstring="lud:";

  enum{
    OPER_LOCK,
    OPER_UNLOCK
  }oper=OPER_LOCK;

  /* process command line options */
  while((opt=getopt_long(argc,argv,optstring,options,&optindex))!=-1)
    {
      switch(opt)
	{
	case 'l':
	  oper=OPER_LOCK;
	  break;

	case 'u':
	  oper=OPER_UNLOCK;
	  break;

	case 'd':
	  if(bootimage_name)
	    {
	      fprintf(stderr,"WARNING: image file redefined, "
		      "from \"%s\" to \"%s\"\n", bootimage_name,optarg);
	      free(bootimage_name);
	    }
	  bootimage_name=strdup(optarg);
	  if(!bootimage_name)
	    {
	      errmsg_mem();
	      return 1;
	    }
	  break;
	default:
	  usage();
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
      return 1;
    }

  /* output image file */
  h_flash=open(bootimage_name,O_RDWR);
  if(h_flash<0)
    {
      perror("Can't open flash device");
      return 1;
    }
  if(fstat(h_flash,&statbuf))
    {
      perror("Can't determine file/device type");
      close(h_flash);
      return 1;
    }
  
  if(!S_ISCHR(statbuf.st_mode)
     ||major(statbuf.st_rdev)!=MTD_CHAR_MAJOR)
    {
      close(h_flash);
      fprintf(stderr,"Not a flash device\n");
      return 1;
    }
  
  if(ioctl(h_flash,MEMGETREGIONCOUNT,&nregions))
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
	  if(ioctl(h_flash,MEMGETREGIONINFO,reginfo+i))
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
	    {
	      first_block=0;
	      last_block=reginfo[i].numblocks-1;

	      for(j=first_block;j<=last_block;j++)
		{
		  erase.start=reginfo[i].offset
		    +reginfo[i].erasesize*j;
		  erase.length=reginfo[i].erasesize;
		  op_result=ioctl(h_flash,
				  (oper==OPER_LOCK)?MEMLOCK:MEMUNLOCK,
				  &erase);
		  /*
		  op_result=write_block(h_flash,
					   reginfo[i].offset
					   +reginfo[i].erasesize*j,
					   reginfo[i].erasesize,
					   outputtype);
		  */
		  if(op_result<0)
		    {
		      fprintf(stderr,"Bad block: 0x%08lx-0x%08lx\n",
			      (long unsigned)(reginfo[i].offset
					      +reginfo[i].erasesize*j),
			      (long unsigned)(reginfo[i].offset
					      +reginfo[i].erasesize*(j+1)-1));
		    }
		}
	    }
	}
      free(reginfo);
    }
  else
    {
      if(ioctl(h_flash,MEMGETINFO,&mtdinfo)<0)
	{
	  fprintf(stderr,"Can't get MTD information\n");
	  return 1;
	}
      else
	{
#if DEBUG
	  printf("Device: %u bytes, erase size %u bytes\n",
		 mtdinfo.size,mtdinfo.erasesize);
#endif
	  last_block=(mtdinfo.size-1)/mtdinfo.erasesize;
	  for(j=0;j<=last_block;j++)
	    {
	      erase.start=mtdinfo.erasesize*j;
	      erase.length=mtdinfo.erasesize;
	      op_result=ioctl(h_flash,
			      (oper==OPER_LOCK)?MEMLOCK:MEMUNLOCK,
			      &erase);

	      /*
	      op_result=write_block(h_flash,
						   mtdinfo.erasesize*j,
						   mtdinfo.erasesize,
						   outputtype);
	      */
	      if(op_result<0)
		{
		  fprintf(stderr,"Bad block: 0x%08lx-0x%08lx\n",
			  (long unsigned)(mtdinfo.erasesize*j),
			  (long unsigned)(mtdinfo.erasesize*(j+1)-1));
		}
	    }
	}
    }
  sync();
  return 0;
}
