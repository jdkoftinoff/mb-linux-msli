#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

#define VERSION_MAJOR 0
#define VERSION_MINOR 1

void usage(void)
{
  printf("mcsbin version %d.%d\n\n",VERSION_MAJOR,VERSION_MINOR);
  puts(
"Usage:\n"
"       mcsbin -b [options] <mcsfile.mcs> <binaryfile.bin>\n"
"       mcsbin --to-binary <mcsfile.mcs> <binaryfile.bin>\n"
"\n"
"       mcsbin -m [-o <offset>] [options] <binaryfile.bin> <mcsfile.mcs>\n"
"       mcsbin --to-mcs [--offset <offset>] [options] <binaryfile.bin> <mcsfile.mcs>\n"
"\n"
"options:\n"
"-y|--swap16\n"
"       swap bytes for x16 mode\n"
);
}


/* swap bytes in an array */

void swapbytes(unsigned char *buf, unsigned int size)
{
  unsigned int i,j;
  unsigned char c;

  size&=~1;

  for(i=0,j=1;i<size;i+=2,j+=2)
    {
      c=buf[j];
      buf[j]=buf[i];
      buf[i]=c;
    }
}


/* parse MCS file, write binary image */

int mcs2bin(int dsth,int srch,int swapbytesflag)
{
  unsigned char srcbuf[4096];
  unsigned char dstbuf[4096];
  const size_t srcbuflen=sizeof(srcbuf);
  const size_t dstbuflen=sizeof(dstbuf);
  enum {READ_COLON,READ_BYTE_H,READ_BYTE_L,READ_EOL} mcsstate=READ_COLON;
  unsigned char c,b=0,mcscsum=0,mcsrecordtype=-1;
  unsigned int mcsposition=0,mcscount=0,srcparseoffset=0,dstoffset=0;
  unsigned long mcsaddress=0,addoffset=0,writeaddress=0,skipbuffer;
  int l,written,eofflag=0,datawritten=0;

  while(1)
    {
      /* read the buffer */
      l=read(srch,srcbuf,srcbuflen);
      if(l<=0) return l<0;
      
      while(srcparseoffset<l)
	{
	  c=srcbuf[srcparseoffset];
	  /* parse the record */

	  /*
	    Record format:

	    ":BCAAAATTDD ... DDCC\n"
	    BC   -- byte count (hex)
	    AAAA -- address (hex)
	    TT   -- record type (hex, 0-5)
	    DD   -- data (optional)
	    CC   -- checksum

	    Supported record types:
	    00   -- data

	    01   -- end of file
	            no data

	    02   -- extended segment address
	            data contains bits 4-19 of the segment address

	    04   -- extended linear address
	            data contains bits 16-31 of the address
	  */

	  switch(mcsstate)
	    {
	    case READ_EOL:
	      if(c!='\r'&&c!='\n')
		{
		  fprintf(stderr,"Not an End-Of-Line: '%c'\n",c);
		  return -1;
		}
	      if(c=='\n')
		{
		  if(mcscsum!=0)
		    {
		      fprintf(stderr,"Checksum error\n");
		      return -1;
		    }
		  mcsstate=READ_COLON;
		  mcsposition=0;
		}
	      break;

	    case READ_COLON:
	      if(c!=':')
		{
		  fprintf(stderr,"Not a ':': '%c'\n",c);
		  return -1;
		}
	      mcsposition=0;
	      mcscsum=0;
	      mcsstate=READ_BYTE_H;
	      break;

	    case READ_BYTE_H:
	      if((c<'0'||c>'9')&&(c<'A'||c>'F')&&(c<'a'||c>'f'))
		{
		  fprintf(stderr,"Not a Hex Digit: '%c'\n",c);
		  return -1;
		}
	      if(c>='a') c-=('a'-10);
	      else
		if(c>='A') c-=('A'-10);
		else
		  c-='0';
	      b=c<<4;
	      mcsstate=READ_BYTE_L;
	      break;
	    
	    case READ_BYTE_L:
	      if((c<'0'||c>'9')&&(c<'A'||c>'F')&&(c<'a'||c>'f'))
		{
		  fprintf(stderr,"Not a Hex Digit: '%c'\n",c);
		  return -1;
		}
	      if(c>='a') c-=('a'-10);
	      else
		if(c>='A') c-=('A'-10);
		else
		  c-='0';

	      /* get the byte value, update checksum */
	      b|=c;
	      mcscsum+=b;
	      
	      /* positions in the MCS record header */
	      switch(mcsposition)
		{
		case 0:
		  /* byte count */
		  mcscount=b+1;
		  break;

		case 1:
		  /* high address byte */
		  mcsaddress=((unsigned long)b)<<8;
		  break;

		case 2:
		  /* low address byte */
		  mcsaddress|=(unsigned long)b;
		  break;

		case 3:
		  /* record type */
		  mcsrecordtype=b;
		  if(mcsrecordtype>5)
		    {
		      fprintf(stderr,"Unknown record type %d\n",b);
		      return -1;
		    }
		  break;

		default:
		  /* data byte */
		  mcscount--;
		  switch(mcsrecordtype)
		    {
		    case 0:
		      /* data */
		      if(mcscount>0)
			{
			  if(!datawritten)
			    {
			      /* no data was written, set start address */
			      writeaddress=mcsaddress+addoffset;
			      datawritten=1;
			      printf("Binary file start address: %lu\n",
				     writeaddress);
			    }
			  else
			    {
			      if(writeaddress!=mcsaddress+addoffset)
				{
				  /* address jumped forward or backward */
				  if(writeaddress>mcsaddress+addoffset)
				    {
	      /* can't handle backward seeking when converting files */
				      fprintf(stderr,
				       "Lower address follows higher address, "
				       "can't write file in this sequence\n");
				      return -1;
				    }
				  else
				    {
	      /* jump forward, flush buffer and fill the gap with 0xff */
				      if(swapbytesflag)
					{
					  if(dstoffset&1)
					    {
					      fprintf(stderr,
			      "Odd number of bytes in swap-bytes mode\n");
					      return -2;
					    }
					  swapbytes(dstbuf,dstoffset);
					}
				      written=write(dsth,dstbuf,dstoffset);
				      if(written!=dstoffset)
					{
					  fprintf(stderr,
						  "Write error\n");
					  return -2;
					}
				      dstoffset=0;
				      skipbuffer=mcsaddress+
					addoffset-writeaddress;
				      fprintf(stderr,
				       "WARNING: skipping a gap, %lu bytes\n",
					      skipbuffer);
				      memset(dstbuf,0xff,dstbuflen);
				      while(skipbuffer)
					{
					  if(skipbuffer<=dstbuflen)
					    {
					      written=write(dsth,dstbuf,
							    skipbuffer);
					      if(written!=skipbuffer)
						{
						  fprintf(stderr,
							  "Write error\n");
						  return -2;
						}
					      skipbuffer=0;
					    }
					  else
					    {
					      written=write(dsth,dstbuf,
							    dstbuflen);
					      if(written!=dstbuflen)
						{
						  fprintf(stderr,
							  "Write error\n");
						  return -2;
						}
					      skipbuffer-=dstbuflen;
					    }
					}
				      writeaddress=mcsaddress+addoffset;
				    }
				}
			    }
			  /* place byte into the buffer */
			  dstbuf[dstoffset++]=b;
			  if(dstoffset>=dstbuflen)
			    {
			      if(swapbytesflag)
				{
				  swapbytes(dstbuf,dstbuflen);
				}
			      written=write(dsth,dstbuf,dstbuflen);
			      if(written!=dstbuflen)
				{
				  fprintf(stderr,"Write error\n");
				  return -2;
				}
			      dstoffset=0;
			    }
			  writeaddress++;
			  mcsaddress++;
			}
		      break;

		    case 1:
		      /* end of file */
		      eofflag=1;
		      break;

		    case 2:
		      /* segment address */
		      if(mcscount>0)
			{
			  addoffset<<=8;
			  addoffset+=(unsigned long)b;
			}
		      else
			{
			  addoffset<<=4;
			}
		      break;

		    case 4:
		      /* linear address */
		      if(mcscount>0)
			{
			  addoffset<<=8;
			  addoffset+=(unsigned long)b;
			}
		      else
			{
			  addoffset<<=16;
			}
		      break;
		    }
		}
	      mcsposition++;
	      if(mcscount>0)
		{
		  mcsstate=READ_BYTE_H;
		}
	      else
		{
		  if(eofflag)
		    {
		      if(mcscsum!=0)
			{
			  fprintf(stderr,"Checksum error\n");
			  return -1;
			}
		      else
			{
			  if(dstoffset)
			    {
			      if(swapbytesflag)
				{
				  if(dstoffset&1)
				    {
				      fprintf(stderr,
			      "Odd number of bytes in swap-bytes mode\n");
				      return -2;
				    }
				  swapbytes(dstbuf,dstoffset);
				}
			      written=write(dsth,dstbuf,dstoffset);
			      if(written!=dstoffset)
				{
				  fprintf(stderr,"Write error\n");
				  return -2;
				}
			    }
			  return 0;
			}
		    }
		  mcsstate=READ_EOL;
		}
	      break;
	    }
	  srcparseoffset++;
	}
      srcparseoffset=0;
    }
}


/* flush the remaining data in the write buffer */

static inline int flush_buffer(int h,unsigned char *dstbuf,
				unsigned int *dstoffset,size_t dstbuflen)
{
  int retval=0;
  if(*dstoffset)
    {
      retval=(write(h,dstbuf,*dstoffset)!=*dstoffset);
      *dstoffset=0;
    }
  return retval;
}


/* write one byte */

static inline int write_byte(int h,unsigned char *dstbuf,
			     unsigned int *dstoffset,size_t dstbuflen,
			     unsigned char c)
{
  int retval=0;
  if(*dstoffset>=dstbuflen)
    {
      retval=(write(h,dstbuf,dstbuflen)!=dstbuflen);
      *dstoffset=0;
    }
  dstbuf[*dstoffset]=c;
  (*dstoffset)++;
  return retval;
}


/* 0-16 -> 0-9,A-F */

static inline unsigned char hex_digit(unsigned char c)
{
  if(c<=9) return c+'0';
  else return c-10+'A';
}


/* write MCS record 1, 2 or 4 */

static inline int write_record(int h,unsigned char *dstbuf,
			       unsigned int *dstoffset,size_t dstbuflen,
			       unsigned int address,
			       unsigned char recordtype,
			       unsigned int intdata)
{
  int retval=0,count;
  unsigned char csum=0;
  count=(recordtype==1)?0:2;

  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,':');

  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(count>>4));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(count&0x0f));

  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit((address>>12)&0x0f));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit((address>>8)&0x0f));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit((address>>4)&0x0f));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(address&0x0f));

  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(recordtype>>4));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(recordtype&0x0f));

  csum=count+recordtype+((address>>8)&0xff)+(address&0xff);

  if(count>0)
    {
      retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
			 hex_digit((intdata>>12)&0x0f));
      retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
			 hex_digit((intdata>>8)&0x0f));
      retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
			 hex_digit((intdata>>4)&0x0f));
      retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
			 hex_digit(intdata&0x0f));
      csum+=((intdata>>8)&0xff)+(intdata&0xff);
    }

  csum=0-csum;

  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(csum>>4));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(csum&0x0f));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,'\n');

  return retval;
}


/* write MCS record 0 */

static inline int write_data_record(int h,unsigned char *dstbuf,
				    unsigned int *dstoffset,size_t dstbuflen,
				    size_t count,unsigned int address,
				    unsigned char *data)
{
  int retval=0;
  unsigned int i;
  unsigned char csum=0,c;
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,':');

  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(count>>4));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(count&0x0f));

  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit((address>>12)&0x0f));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit((address>>8)&0x0f));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit((address>>4)&0x0f));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(address&0x0f));

  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,'0');
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,'0');

  csum=count+((address>>8)&0xff)+(address&0xff);

  for(i=0;i<count;i++)
    {
      c=data[i];
      retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
			 hex_digit((c>>4)&0x0f));
      retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
			 hex_digit(c&0x0f));
      csum+=c;
    }

  csum=0-csum;

  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(csum>>4));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,
		     hex_digit(csum&0x0f));
  retval|=write_byte(h,dstbuf,dstoffset,dstbuflen,'\n');
  return 0;
}


/* read binary image, write MCS file with this image and given offset */

int bin2mcs(int dsth,int srch,unsigned long offset,int swapbytesflag)
{
  unsigned char recorddata[0x10000];
  unsigned char dstbuf[4096];
  const size_t dstbuflen=sizeof(dstbuf);
  unsigned int dstoffset=0;
  unsigned long rec4addr;
  unsigned int rec0startaddr,padaddr,blocksize,off;
  int l,retval=0;

  /* record 4 contains the initial address */
  rec4addr=offset>>16;
  
  /*
    MCS format, in theory, can be used with a wider address range,
    however it is impractical for flash files and therefore unsupported
  */
  if(rec4addr>0xffff)
    {
      fprintf(stderr,"Address overflow, this format can't be used "
	      "with >4G address space\n");
      return -1;
    }

  /*
    record 0 starts on 16-byte boundary (at least I have never seen
    otherwise), so if the initial offset does not, pad the record.
  */

  rec0startaddr=(unsigned int)(offset&0xfff0);
  padaddr=(unsigned int)(offset&0xf);

  /* start address */
  retval|=write_record(dsth,dstbuf,&dstoffset,dstbuflen,0,4,rec4addr);

  /*
    if the initial address needs padding, write the first record
    with 0xff at the beginning
  */
  if(padaddr)
    {
      memset(recorddata,0xff,padaddr);
      l=read(srch,recorddata+padaddr,16-padaddr);
      if(l<0)
	{
	  perror("read");
	  return l;
	}
      if(l<16-padaddr)
	{
	  /* if it's a very short file, finish it here */
	  if(swapbytesflag)
	    {
	      if((l+padaddr)&1)
		{
		  recorddata[l+padaddr]=0xff;
		  l++;
		}
	      swapbytes(recorddata,l+padaddr);
	    }

	  retval|=write_data_record(dsth,dstbuf,&dstoffset,dstbuflen,
				    l+padaddr,rec0startaddr,recorddata);
	  retval|=write_record(dsth,dstbuf,&dstoffset,dstbuflen,0,1,0);
	  retval|=flush_buffer(dsth,dstbuf,&dstoffset,dstbuflen);
	  return retval;
	}
      if(swapbytesflag)
	{
	  swapbytes(recorddata,16);
	}

      /* write the first 16-bytes record */
      retval|=write_data_record(dsth,dstbuf,&dstoffset,dstbuflen,
				16,rec0startaddr,recorddata);
      rec0startaddr+=16;
    }

  /* to the nearest 64k boundary */
  blocksize=0x10000-rec0startaddr;
  if(blocksize)
    {
      off=0;
      l=read(srch,recorddata,blocksize);
      if(l<0)
	{
	  perror("read");
	  return -1;
	}

      if(swapbytesflag)
	{
	  if(l<blocksize)
	    {
	      if(l&1)
		{
		  recorddata[l]=0xff;
		  l++;
		}
	    }
	  swapbytes(recorddata,blocksize);
	}

      while(l>=16)
	{
	  /* record 0, 16 bytes */
	  retval|=write_data_record(dsth,dstbuf,&dstoffset,dstbuflen,
				    16,rec0startaddr,recorddata+off);
	  l-=16;
	  off+=16;
	  rec0startaddr+=16;
	}
      if(l>0)
	{
	  /* record 0, less than 16 bytes */
	  retval|=write_data_record(dsth,dstbuf,&dstoffset,dstbuflen,
				    l,rec0startaddr,recorddata+off);
	  off+=l;
	  rec0startaddr+=l;
	}
    }

  /* read 64k blocks, write blocks preceded with record 4 */
  while((l=read(srch,recorddata,0x10000))>0)
    {
      off=0;
      rec0startaddr=0;
      rec4addr++;

      /*
	MCS format, in theory, can be used with a wider address range,
	however it is impractical for flash files and therefore unsupported
      */
      if(rec4addr>0xffff)
	{
	  fprintf(stderr,"Address overflow, this format can't be used "
		  "with >4G address space\n");
	  return -1;
	}
      /* record 4 */
      retval|=write_record(dsth,dstbuf,&dstoffset,dstbuflen,0,4,rec4addr);

      if(swapbytesflag)
	{
	  if(l<0x10000)
	    {
	      if(l&1)
		{
		  recorddata[l]=0xff;
		  l++;
		}
	    }
	  swapbytes(recorddata,0x10000);
	}

      while(l>=16)
	{
	  /* record 0, 16 bytes */
	  retval|=write_data_record(dsth,dstbuf,&dstoffset,dstbuflen,
				    16,rec0startaddr,recorddata+off);
	  l-=16;
	  off+=16;
	  rec0startaddr+=16;
	}
      if(l>0)
	{
	  /* record 0, less than 16 bytes */
	  retval|=write_data_record(dsth,dstbuf,&dstoffset,dstbuflen,
				    l,rec0startaddr,recorddata+off);
	  off+=l;
	  rec0startaddr+=l;
	}
    }
  if(l<0)
    {
      perror("read");
      return -1;
    }
  retval|=write_record(dsth,dstbuf,&dstoffset,dstbuflen,0,1,0);
  retval|=flush_buffer(dsth,dstbuf,&dstoffset,dstbuflen);
  return retval;
}


/*
  get a number from a string in one of the following formats:

  <number>   -- x1
  <number>k  -- x1024
  <number>K  -- x1024
  <number>m  -- x1024x1024
  <number>M  -- x1024x1024

  where <number> is in any format supported by %li
   <decimal>
   0x<hexadecimal>
   0X<hexadecimal>
   0<octal>

   negative values indicate an error.

*/

long getnumber(char *s)
{
  int l;
  long retval,mult=1;
  char *s1;

  l=strlen(s);
  if(l==0) return -1;
  s1=strdup(s);
  if(!s1) return -1;

  switch(s1[l-1])
    {
    case 'm':
    case 'M':
      mult=1024*1024;
      s1[l-1]='\0';
      break;
    case 'k':
    case 'K':
      mult=1024;
      s1[l-1]='\0';
    }
  if(sscanf(s1,"%li",&retval)!=1)
    {
      free(s1);
      return -1;
    }
  free(s1);
  return retval*mult;
}

/* main */

int main(int argc,char **argv)
{
  enum { UNDEFINED,MCS_TO_BIN,BIN_TO_MCS} direction=UNDEFINED;

  /* command line options */
  struct option options[]={
    {"to-binary",0,NULL,'b'},
    {"to-mcs",0,NULL,'m'},
    {"offset",1,NULL,'o'},
    {"swap16",0,NULL,'y'},
    {NULL,0,NULL,'\0'}
  };

  char *optstring="bmo:y";
  int opt,optindex,srcfd,dstfd,retval,offset_specified=0,swapbytesflag=0;
  long offset=0;

  while ((opt = getopt_long(argc, argv,optstring,options,&optindex)) != -1)
    {
      switch(opt)
	{
	case 'b':
	  direction=MCS_TO_BIN;
	  break;
	case 'm':
	  direction=BIN_TO_MCS;
	  break;
	case 'o':
	  offset_specified=1;
	  offset=getnumber(optarg);
	  if(offset<0)
	    {
	      fprintf(stderr,"Invalid offset\n");
	      return 1;
	    }
	  break;
	case 'y':
	  swapbytesflag=1;
	  break;
	default:
	  usage();
	  return 1;
	}
    }

  if(argc<optind+2)
    {
      usage();
      return 1;
    }

  srcfd=open(argv[optind],O_RDONLY);
  if(srcfd<0)
    {
      perror("Opening source file");
      return 1;
    }
  dstfd=open(argv[optind+1],O_WRONLY|O_CREAT|O_TRUNC,0666);
  if(dstfd<0)
    {
      perror("Opening destination file");
      close(srcfd);
      return 1;
    }
  switch(direction)
    {
    case MCS_TO_BIN:
      if(offset_specified)
	{
	  fprintf(stderr,
		  "WARNING: offset option from the command line "
		  "is ignored for MCS files\n");
	}
      retval=mcs2bin(dstfd,srcfd,swapbytesflag);
      break;
    case BIN_TO_MCS:
      retval=bin2mcs(dstfd,srcfd,offset,swapbytesflag);
      break;
    default:
      usage();
      retval=1;
    }
  close(srcfd);
  close(dstfd);
  if(retval)
    {
      fprintf(stderr,"Output file deleted\n");
      unlink(argv[optind+1]);
    }
  return retval;
}
