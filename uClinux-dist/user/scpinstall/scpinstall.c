#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <getopt.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <dirent.h>
#include <errno.h>
#include <signal.h>

/* default ssh program */
#define SSH_PROGRAM "/usr/bin/ssh"

/* support up to 256 levels of directories */
#define MAXDEPTH 256

/* flags */
int flag_preserve=0,flag_quiet=0,flag_recursive=0,flag_to=0,flag_from=0,
  flag_directory=0,flag_verbose=0;

/* input buffer (also re-used for output) */
typedef struct 
{
  size_t allocated;
  size_t inbuffer;
  unsigned read_offset;
  unsigned char *buffer;
} input_buffer;

/* initialize and allocate buffer */
static inline int init_buffer(input_buffer *inbuf,size_t size)
{
  inbuf->read_offset=0;
  inbuf->inbuffer=0;
  inbuf->buffer=(unsigned char*)malloc(size);
  if(inbuf->buffer!=NULL)
    {
      inbuf->allocated=size;
      return 0;
    }
  else
    {
      inbuf->allocated=0;
      return -1;
    }
}

/* free buffer */
static inline void free_buffer(input_buffer *inbuf)
{
  free(inbuf->buffer);
  inbuf->buffer=NULL;
  inbuf->allocated=0;
  inbuf->read_offset=0;
  inbuf->inbuffer=0;
}

/* read data into buffer */
static inline int read_buffer(int h,input_buffer *inbuf)
{
  int l;
  //  fprintf(file_cons,"Reading buffer\r\n");
  do
    l=read(h,inbuf->buffer+inbuf->inbuffer,inbuf->allocated-inbuf->inbuffer);
  while(l<0&&errno==EINTR);
  if(l<0)
    {
      //      fprintf(file_cons,"Reading buffer, got error %d\r\n",l);
      return l;
    }
  inbuf->inbuffer+=l;
  //  fprintf(file_cons,"Reading buffer, got %d bytes\r\n",l);
  return l;
}

/* find end of newline-terminated or zero-terminated line */
int line_end(input_buffer *inbuf)
{
  int index;
  for(index=inbuf->read_offset;index<inbuf->inbuffer;index++)
    if((inbuf->buffer[index]=='\n')||(inbuf->buffer[index]=='\0'))
      return index;
  return -1;
}

/* shift unread data to the start of the input buffer */
void shift_buffer(input_buffer *inbuf)
{
  int l;
  l=inbuf->inbuffer-inbuf->read_offset;
  if(l)
    memmove(inbuf->buffer,inbuf->buffer+inbuf->read_offset,l);
  inbuf->inbuffer-=inbuf->read_offset;
  inbuf->read_offset=0;
}

/* buffered input, read a newline-terminated or zero-terminated line */
unsigned char *get_line_buffer(int h,input_buffer *inbuf)
{
  int l;
  unsigned char *retval;
  l=line_end(inbuf);
  if(l>=0)
    {
      inbuf->buffer[l]='\0';
      retval=inbuf->buffer+inbuf->read_offset;
      inbuf->read_offset=l+1;
      //     fprintf(file_cons,"got line, offset %d\r\n",retval-inbuf->buffer);
      return retval;
    }
  
  if(inbuf->allocated>inbuf->inbuffer)
    {
      //      fprintf(file_cons,"Reading line buffer, 1\r\n");
      if(read_buffer(h,inbuf)<=0)
	{
	  //	  fprintf(file_cons,"got nothing, 1\r\n");
	  return NULL;
	}
      //      fprintf(file_cons,"got something, 1\r\n");

      l=line_end(inbuf);
      if(l>=0)
	{
	  inbuf->buffer[l]='\0';
	  retval=inbuf->buffer+inbuf->read_offset;
	  inbuf->read_offset=l+1;
	  // fprintf(file_cons,"got line, offset %d\r\n",retval-inbuf->buffer);
	  return retval;
	}
    }

  shift_buffer(inbuf);

  if(inbuf->allocated>inbuf->inbuffer)
    {
      //      fprintf(file_cons,"Reading line buffer, 2\r\n");
      if(read_buffer(h,inbuf)<=0)
	{
	  //	  fprintf(file_cons,"got nothing, 2\r\n");
	  return NULL;
	}
      
      l=line_end(inbuf);
      if(l>=0)
	{
	  inbuf->buffer[l]='\0';
	  retval=inbuf->buffer+inbuf->read_offset;
	  inbuf->read_offset=l+1;
	  // fprintf(file_cons,"got line, offset %d\r\n",retval-inbuf->buffer);
	  return retval;
	}
    }
  //  fprintf(file_cons,"got nothing, 3\r\n");
  return NULL;
}

/* write to a blocking file descriptor, retrying after signals */
int write_persist_blocking(int h,unsigned char *buffer,size_t size)
{
  ssize_t l,writesize=(ssize_t)size;
  do
    {
      do
	l=write(h,buffer,writesize);
      while(l<0&&errno==EINTR);
      if(l<0)
	{
	  if(size==writesize)
	    return -1;
	  else 
	    return size-writesize;
	}
      writesize-=l;
      buffer+=l;
    }
  while(writesize>0);
  return size;
}

/*
  read rcp/scp response format from stdin (one byte, then if the byte
  is not 0, a newline-terminated string)
*/
int get_response(void)
{
  unsigned char c;
  int retval;
  ssize_t l;
  do
    l=read(0,&c,1);
  while(l<0&&errno==EINTR);
  if(l<1)
    return -1;
  retval=c;
  switch(c)
    {
    case 0:
      return 0;
    case 1:
    case 2:
      do
	{
	  do
	    l=read(0,&c,1);
	  while(l<0&&errno==EINTR);
	  if(l<1)
	    return -1;
	}
      while(c!='\n');
      return retval;
      
    default:
      return -1;
    }
}

/* print response to stdout */
static inline int print(char *string)
{
  return write_persist_blocking(1,(unsigned char*)string,strlen(string));
}

/* send rcp/scp confirmation (byte 0) */
static inline int rcp_ok(void)
{
  return write_persist_blocking(1,(unsigned char*)"\x00",1);
}

/* send a file with rcp/scp protocol */
int send_file(char *name,input_buffer *buf)
{
  int h,response;
  ssize_t l,l0,dataleft;
  struct stat statbuf;

  /* open file */
  h=open(name,O_RDONLY);
  /*
    not being able to open or stat the file is a special error,
    it is not fatal for directory copy operations
  */
  if(h<0) return -2;
  if(fstat(h,&statbuf))
    {
      close(h);
      return -2;
    }

  /* send "C" line */
  snprintf((char*)buf->buffer,buf->allocated,"C%04o %llu ",
	   statbuf.st_mode&07777,
	   (long long)statbuf.st_size);
  write_persist_blocking(1,buf->buffer,strlen((char*)buf->buffer));
  write_persist_blocking(1,(unsigned char*)name,strlen(name));
  write_persist_blocking(1,(unsigned char*)"\n",1);

  /* receive response */
  response=get_response();
  if(response<0||response==2)
    {
      close(h);
      return -1;
    }

  /* re-use input buffer for sending */
  dataleft=statbuf.st_size;

  while(dataleft>0)
    {
      if(dataleft>buf->allocated)
	l0=buf->allocated;
      else
	l0=dataleft;
      l=read(h,(char*)buf->buffer,l0);
      if(l<=0)
	{
	  close(h);
	  h=-1;
	  if(l<0)
	    return -1;
	  /* truncated file */
	  l=l0;
	  memset((char*)buf->buffer,0,l0);
	}
      if(l>dataleft)
	l=dataleft;
      write_persist_blocking(1,buf->buffer,l);
      dataleft-=l;
    }
  close(h);

  /* send zero byte after the end of transfer */
  rcp_ok();

  /* read response, ignore it */
  get_response();
  return 0;
}

/* directory tree sending, flat implementation to avoid using deep stack */
int send_directory(char *name,input_buffer *buf)
{
  static DIR *directories[MAXDEPTH];
  int depth=0,errflag=0,response;
  struct dirent *entry;
  struct stat statbuf;

  /* if no directory name is specified, use "." */
  if(*name=='\0')
    name=".";

  /* get status */
  if(lstat(name,&statbuf))
    return -1;

  /* open a directory */
  directories[0]=opendir(name);

  /* return value -2 means, this is not a directory */
  if(directories[0]==NULL)
      return (errno==ENOTDIR)?-2:-1;

  /* send "D" line (same format as "C", except size is always 0 */
  snprintf((char*)buf->buffer,buf->allocated,"D%04o 0 ",statbuf.st_mode&07777);
  write_persist_blocking(1,buf->buffer,strlen((char*)buf->buffer));
  write_persist_blocking(1,(unsigned char*)name,strlen(name));
  write_persist_blocking(1,(unsigned char*)"\n",1);

  /* receive response */
  response=get_response();
  if(response<0||response==2)
    {
      closedir(directories[0]);
      return -1;
    }
  
  /*
    traverse directory tree, use file descriptor in DIR to move
    between directories
   */
  while(depth>=0&&!errflag&&!fchdir(dirfd(directories[depth])))
    {
      /* read entry */
      entry=readdir(directories[depth]);
      if(entry)
	{
	  if(!lstat(entry->d_name,&statbuf))
	    {
	      if(S_ISREG(statbuf.st_mode))
		{
		  /* regular file, send it */
		  if(send_file(entry->d_name,buf)==-1)
		    errflag=1;
		}
	      else
		{
		  if(S_ISDIR(statbuf.st_mode)
		     &&(depth+1<MAXDEPTH)
		     &&strcmp(entry->d_name,"..")
		     &&strcmp(entry->d_name,"."))
		    {
		      /* 
			 directory, not .. or ., and within MAXDEPTH,
			 open it
		      */
		      depth++;
		      directories[depth]=opendir(entry->d_name);
		      /* ignore directories we can not open */
		      if(directories[depth]==NULL)
			depth--;
		      else
			{
			  /* send "D" line for this directory */
			  snprintf((char*)buf->buffer,buf->allocated,
				   "D%04o 0 ",
				   statbuf.st_mode&07777);
			  write_persist_blocking(1,buf->buffer,
						 strlen((char*)buf->buffer));
			  write_persist_blocking(1,
						 (unsigned char*)entry->d_name,
						 strlen(entry->d_name));
			  write_persist_blocking(1,"\n",1);
			  response=get_response();
			  /* read response */
			  if(response<0||response==2)
			    {
			      closedir(directories[depth]);
			      depth--;
			      errflag=1;
			    }
			}
		    }
		}
	    }
	}
      else
	{
	  /* end of directory, moving up */
	  /* send "E" line */
	  write_persist_blocking(1,(unsigned char*)"E\n",2);
	  closedir(directories[depth]);
	  depth--;
	  response=get_response();
	}
    }
  /* cleanup */
  while(depth>=0)
    closedir(directories[depth--]);
  
  return errflag;
}

/* usage (only server options are implemented) */
void usage(void)
{
  fputs(
"usage: scp [-1246BCpqrv] [-c cipher] [-F ssh_config] [-i identity_file]\n"
"           [-l limit] [-o ssh_option] [-P port] [-S program]\n"
"           [[user@]host1:]file1 ... [[user@]host2:]file2\n",stderr);
}

/* main */
int main(int argc,char **argv,char **env)
{
  int ssh_argc,opt,i;
  input_buffer inbuf;
  char **ssh_argv,*ssh_arg_string;

  //  file_cons=fopen("/dev/ttyUL0","w");

  /* ssh arguments -- not implemented yet */
  ssh_argv=(char**)malloc(sizeof(char*)*(argc+1));

  if(!ssh_argv||init_buffer(&inbuf,8192))
    {
      fprintf(stderr,"Insufficient memory\n");
      exit(1);
    }

  ssh_argv[0]=strdup(SSH_PROGRAM);
  if(!ssh_argv[0])
    {
      fprintf(stderr,"Insufficient memory\n");
      exit(1);
    }
  ssh_argc=1;
  
  /* process command line options */
  while((opt=getopt(argc,argv,"1246BCpqrtfdvc:F:i:l:o:P:S:"))!=-1)
    {
      switch(opt)
	{
	case '1':
	case '2':
	case '4':
	case '6':
	case 'B':
	case 'C':
	  ssh_arg_string=malloc(3);
	  if(!ssh_arg_string)
	    {
	      fprintf(stderr,"Insufficient memory\n");
	      exit(1);
	    }
	  ssh_arg_string[0]='-';
	  ssh_arg_string[1]=opt;
	  ssh_arg_string[2]='\0';
	  if(ssh_argc<argc)
	    ssh_argv[ssh_argc++]=ssh_arg_string;
	  else
	    {
	      fprintf(stderr,"Not enough space for arguments\n");
	      exit(1);
	    }
	  /* pass to ssh */
	  break;
	  
	case 'P':
	  /* pass -p with the argument */
	  opt='p';
	  /* fall through */

	case 'c':
	case 'F':
	case 'i':
	case 'o':
	  /* pass to ssh with argument */
	  ssh_arg_string=malloc(3);
	  if(!ssh_arg_string)
	    {
	      fprintf(stderr,"Insufficient memory\n");
	      exit(1);
	    }
	  ssh_arg_string[0]='-';
	  ssh_arg_string[1]=opt;
	  ssh_arg_string[2]='\0';
	  if(ssh_argc+1<argc)
	    {
	      ssh_argv[ssh_argc++]=ssh_arg_string;
	      ssh_arg_string=strdup(optarg);
	      if(!ssh_arg_string)
		{
		  fprintf(stderr,"Insufficient memory\n");
		  exit(1);
		}
	      ssh_argv[ssh_argc++]=ssh_arg_string;
	    }
	  else
	    {
	      fprintf(stderr,"Not enough space for arguments\n");
	      exit(1);
	    }
	  break;
	  
	case 'S':
	  free(ssh_argv[0]);
	  ssh_arg_string=strdup(optarg);
	  if(!ssh_arg_string)
	    {
	      fprintf(stderr,"Insufficient memory\n");
	      exit(1);
	    }
	  ssh_argv[0]=ssh_arg_string;
	  /* run argument instead of ssh */
	  break;
	  
	case 'p':
	  flag_preserve=1;
	  /* preserve */
	  break;
	case 'q':
	  /* quiet */
	  flag_quiet=1;
	  break;
	case 'r':
	  /* recursive */
	  flag_recursive=1;
	  break;
	case 't':
	  /* server -- copy to */
	  flag_to=1;
	  break;
	case 'f':
	  /* server -- copy from */
	  flag_from=1;
	  break;
	case 'd':
	  /* server -- directory */
	  flag_directory=1;
	  break;
	case 'l':
	  /* bandwidth limit */
	  //FIXME -- unimplemented
	  break;
	case 'v':
	  /* verbose */
	  flag_verbose=1;
	  break;
	default:
	  /* invalid */
	  free_buffer(&inbuf);
	  free(ssh_argv);
	  usage();
	  exit(1);
	}  
    }

  if(ssh_argc<=argc)
    ssh_argv[ssh_argc]=NULL;
  else
    {
      fprintf(stderr,"Not enough space for arguments\n");
      exit(1);
    }

  if(!flag_from&&!flag_to)
    {
      fprintf(stderr,"Client mode is not implemented yet\n");
      exit(1);
    }
  if(flag_from&&flag_to)
    {
      fprintf(stderr,"Can't have -f and -t flags at the same time\n");
      exit(1);
    }
  free(ssh_argv);
  ssh_argv=NULL;


  /* file name arguments */
  char *argdirname,*argfname,*p;
  argdirname="";
  argfname="";

  /* argument processing (only one argument after options is used) */
  if(optind!=argc)
    {
      argdirname=strdup(argv[optind]);
      if(!argdirname)
	{
	  print("\x02"
		"Insufficient memory\n");
	  return 1;
	}
      
      
      /* initial preparations -- current directory and file names */
      if(flag_to)
	{
	  /* destination */

	  /*
	    target is an existing directory?
	    alternatively, destination is a file and therefore
	    we should not attempt using it as a directory name
	  */
	  
	  if(!flag_directory||chdir(argdirname))
	    {
	      /* split into directory and file (or new directory) name? */
	      p=strrchr(argdirname,'/');
	      if(!p)
		/*
		  no, new directory is taken relatively to the unchanged
		  current directory
		*/
		argfname=argdirname;
	      else
		{
		  /*
		    yes, determine both components, change to the
		    existing directory
		  */
		  *p='\0';
		  argfname=p+1;
		  /* 
		     if the first character was '/', and now is '\0',
		     original directory is root, and new directory
		     may be under it
		  */
		  if(*argdirname=='\0')
		    chdir("/");
		  else
		    {
		      if(chdir(argdirname))
			{
			  /* directory does not exist or is inaccessible */
			  print("\x02"
				"Can't change into directory\n");
			  return 1;
			}
		    }
		}
	    }
	  else
	    {
	      /* entered target directory, do nothing */
	    }
	}
      else
	{
	  if(flag_from)
	    {
	      /* source */
	      
	      /* split into directory and file (or source directory) name? */
	      p=strrchr(argdirname,'/');
	      if(!p)
		/*
		  no, file or directory is taken relatively to the unchanged
		  current directory
		*/
		argfname=argdirname;
	      else
		{
		  /*
		    yes, determine both components, change to the
		    existing directory
		  */
		  *p='\0';
		  argfname=p+1;
		  /* 
		     if the first character was '/', and now is '\0',
		     original directory is root
		  */
		  if(*argdirname=='\0')
		    chdir("/");
		  else
		    {
		      if(chdir(argdirname))
			{
			  /* directory does not exist or is inaccessible */
			  print("\x02"
				"Can't change into directory\n");
			  return 1;
			}
		    }
		}
	      
	    }
	}

    }

  /* allocate and fill current directory buffer */
  char *dirbuffer,*dirbuffer_new;
  size_t dirbuffer_size,newdirsize;
  dirbuffer=(char*)malloc(PATH_MAX+1);
  if(!dirbuffer)
    {
      print("\x02"
	    "Insufficient memory\n");
      return 1;
    }
  if(!getcwd(dirbuffer,PATH_MAX+1))
    {
      print("\x02"
	    "Can't get current directory\n");
      return 1;
    }
  dirbuffer_size=PATH_MAX;

  if(flag_to)
    {
      /* destination */
      int h=-1;
      int dirflag=0;
      mode_t fileperm=0;
      off_t filelen=0,dataleft;
      static char *filename=NULL;
      unsigned char *p,*cmd,*zero_response=NULL;
      rcp_ok();
      while((cmd=get_line_buffer(0,&inbuf)))
	{
	  //	  fprintf(file_cons,"Command received: %d, %s\r\n",cmd[0],cmd);
	  switch(*cmd)
	    {
	    case 'C':
	      /* file */
	      /* fall through */
	    case 'D':
	      /* directory start */
	      fileperm=0;
	      filelen=0;
	      p=cmd+1;

	      /* permissions in octal */
	      while((*p>='0')&&(*p<='7'))
		{
		  fileperm<<=3;
		  fileperm|=*p-'0';
		  p++;
		}

	      /* we only support 07777 */
	      fileperm&=07777;

	      /*
		just in case, multiple spaces are accepted, though only
		one should be here
	      */
	      while(*p==' ') p++;

	      /* size in decimal */
	      while((*p>='0')&&(*p<='9'))
		{
		  filelen*=10;
		  filelen+=*p-'0';
		  p++;
		}
	      /*
		only one space here -- a leading space in file name
		is perfectly valid, just a very stupid thing to make
	      */
	      if(*p==' ') p++;

	      /* 
		 replace file or directory name wth the argument if it
		 is still set
	      */
	      if(argfname[0])
		  p=argfname;

	      /* enforcing NAME_MAX */
	      if(strlen((char*)p)>NAME_MAX)
		p[NAME_MAX]=0;

	      if(filename)
		free(filename);
	      filename=strdup((char*)p);
	      if(!filename)
		{
		  print("\x02"
			"Insufficient memory\n");
		  return 1;
		}

	      /* this line describes a directory? */
	      dirflag=*cmd=='D';

	      /* fail on invalid names */
	      if(strchr(filename,'/'))
		{
		  /* root directory becomes "" */
		  if(filename[1]=='\0')
		    filename[0]=='\0';
		  else
		    {
		      /* any other use of slash is invalid */
		      print("\x02"
			    "Invalid use of path separator\n");
		      return 1;
		    }
		}
	      /*
		".." should not be used as a directory or file
		as a destination in copy operations
	      */
	      if((filename[0]=='.')
		 &&(filename[1]=='.')
		 &&(filename[2]=='\0'))
		{
		  print("\x02"
			"Invalid use of ..\n");
		  return 1;
		}
	      
	      if(dirflag)
		{
		  /* directory */
		  newdirsize=strlen(dirbuffer)+1+strlen(filename);
		  if(newdirsize>dirbuffer_size)
		    {
		      dirbuffer_new=(char*)realloc(dirbuffer,newdirsize+1);
		      if(!dirbuffer_new)
			{
			  print("\x02"
				"Insufficient memory\n");
			  return 1;
      			}
		      dirbuffer=dirbuffer_new;
		      dirbuffer_size=newdirsize;
		    }
		  strcat(dirbuffer,"/");
		  strcat(dirbuffer,filename);
		  if(chdir(dirbuffer))
		    {
		      if(mkdir(dirbuffer,fileperm)||chdir(dirbuffer))
			{
			  print("\x02"
				"Can't create directory\n");
			  return 1;
			}
		    }
		  /*
		    once directory is started, directory or file name argument
		    for all subsequent operations is ""
		  */
		  argfname[0]='\0';
		  /* confirm "D" line */
		  rcp_ok();
		}
	      else
		{
		  /* file */
		  int writing_firmware=0,writing_config=0;
		  int tar_pipe[2];
		  pid_t pid;
		  int status;
		  char tmpdir[20];
		  char *exec_argv[3];

		  /*
		    check if target file name is one of the predefined
		    "magic" names used for flash update
		  */
		  if(!strcmp(filename,"firmware.tar.gz"))
		    writing_firmware=1;
		  else
		    if(!strcmp(filename,"config.tar.gz"))
		      writing_config=1;
		  if(writing_firmware||writing_config)
		    {
		      char dirbuffer_tmp[2];
		      if(!getcwd(dirbuffer_tmp,2)
			 ||dirbuffer_tmp[0]!='/'
			 ||dirbuffer_tmp[1]!='\0')
			{
			  writing_firmware=0;
			  writing_config=0;
			}
		    }
		  if(writing_firmware||writing_config)
		    {
		      h=-1;
		      strcpy(tmpdir,"/tmp/scpinst-XXXXXX");
		      if(mkdtemp(tmpdir)&&!chdir(tmpdir))
			{
			  if(!pipe(tar_pipe))
			    {
			      exec_argv[0]="/bin/tar";
			      exec_argv[1]="xz";
			      exec_argv[2]=NULL;
			      pid=vfork();
			      if(pid<0)
				{
				  /* can't create new process */
				  close(tar_pipe[0]);
				  close(tar_pipe[1]);
				  chdir("/");
				  rmdir(tmpdir);
				}
			      else
				{
				  if(pid==0)
				    {
				      /* tar process */
				      close(tar_pipe[1]);
				      if(tar_pipe[0]!=0)
					dup2(tar_pipe[0],0);
				      close(1);
				      open("/dev/null",O_WRONLY);
				      execve(exec_argv[0],exec_argv,env);
				      _exit(1);
				    }
				  else
				    {
				      /* parent process */
				      close(tar_pipe[0]);
				      h=tar_pipe[1];
				    }
				}
			    }
			  else
			    {
			      chdir("/");
			      rmdir(tmpdir);
			    }
			}
		    }
		  else
		    {
		      h=open(filename,
			     O_WRONLY|O_CREAT|O_TRUNC,
			     fileperm);
		    }
		  if(h<0)
		    {
		      print("\x02"
			    "File access error\n");
		    }
		  else
		    {
		      /* confirm "C" line */
		      rcp_ok();
		      while(filelen>0)
			{
			  dataleft=inbuf.inbuffer-inbuf.read_offset;
			  if(filelen<dataleft)
			    dataleft=filelen;
			  if((h>=0)
			     &&(write(h,inbuf.buffer+inbuf.read_offset,
				      dataleft)!=dataleft))
			    {
			      /* write error, clean up */
			      close(h);
			      unlink(filename);
			      h=-1;
			      /*
				respond to the next zero byte with
				a warning message
			      */
			      zero_response="\x01"
				    "File write error\n";
			    }
			  inbuf.read_offset+=dataleft;
			  filelen-=dataleft;
			  shift_buffer(&inbuf);
			  if(filelen>0)
			    {
			      if(read_buffer(0,&inbuf)<=0)
				{
				  close(h);
				  unlink(filename);
				  return 1;
				}
			    }
			}
		      if(h>=0)
			close(h);
		      h=-1;
		      
		      if(writing_firmware||writing_config)
			{
			  if((waitpid(pid,&status,0)==pid)
			     &&(WIFEXITED(status))
			     &&(WEXITSTATUS(status)==0))
			    {
			      signal(SIGCHLD,SIG_IGN);
			      pid=vfork();
			      if(pid<0)
				{
				  print("\x02"
					"Can't start installer\n");
				  return 1;
				}
			      else
				{
				  if(pid==0)
				    {
				      /* installer process */
				      signal(SIGCHLD,SIG_DFL);
				      for(i=0;i<256;i++)
					close(i);
				      exec_argv[0]=writing_firmware?
					"FIRMWARE INSTALLER":
					"CONFIGURATION INSTALLER";
				      exec_argv[1]=NULL;
				      execve("/bin/scpinstall-helper",
					     exec_argv,env);
				      _exit(1);
				    }
				  rcp_ok();
				}
			      signal(SIGCHLD,SIG_DFL);
			    }
			  else
			    {
			      print("\x02"
				    "tar file unpacking error\n");
			      return 1;
			    }
			  chdir("/");
			}
		      else
			{
			  /*
			    regular file transfer completed,
			    confirm the next zero byte
			  */
			  zero_response="\x00";
			}
		    }
		}
	      free(filename);
	      filename=NULL;
	      break;

	    case 'E':
	      /* end of directory */
	      p=(unsigned char*)strrchr(dirbuffer,'/');
	      if(p)
		*p=0;
	      if(*dirbuffer=='\0')
		chdir("/");
	      else
		chdir(dirbuffer);
	      rcp_ok();
	      break;

	    case 'T':
	      /* time (ignore) */
	      rcp_ok();
	      break;

	    case 0:
	      /*
		zero byte received after the transfer is treated
		like a separate command line here
	      */
	      if(zero_response==NULL)
		zero_response="\x02" "Unexpected zero byte\n";
	      if(*zero_response)
		print(zero_response);
	      else
		rcp_ok();
	      zero_response=NULL;
	      break;

	    default:
	      /* ignore other lines */
	      rcp_ok();
	      break;
	    }
	}
      return 0;
    }
  else
    {
      /* source */
      if(get_response())
	return 1;
      if(flag_recursive)
	{
	  int retval;
	  /* try to send as a directory */
	  retval=send_directory(argfname,&inbuf);
	  /* don't exit if it's not a directory */
	  if(retval!=-2)
	    return retval;
	}
      /* send file */
      return send_file(argfname,&inbuf);
    }

  /* finished */
  return 0;
}
