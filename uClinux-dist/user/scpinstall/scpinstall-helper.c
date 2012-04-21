#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define LOG_FILENAME "/tmp/flash-update.log"
#define LOCK_FILENAME "/tmp/flash-update.lock"
#define LOG_BUFFER_SIZE 256

void usage(void)
{
  fprintf(stderr,"This program should be only started from scpinstall\n");
}


int write_log_message(int h,char *text,int percentage)
{
  char buffer[LOG_BUFFER_SIZE];
  memset(buffer,0,LOG_BUFFER_SIZE);
  snprintf(buffer,LOG_BUFFER_SIZE,"%s:(%d%%)\n",text,percentage);
  return write(h,buffer,strlen(buffer));
}

int main(int argc,char **argv,char **env)
{
  static char dirbuffer[24];
  static char *exec_argv[25];
  static char link_buf[128];
  pid_t pid;
  int status;
  int i,log_file,lock_file,link_len,install_firmware=0;
  struct stat statbuf;

  if(argc!=1)
    {
      usage();
      return 1;
    }
  if(!strcmp(argv[0],"FIRMWARE INSTALLER"))
    install_firmware=1;
  else
    {
      if(strcmp(argv[0],"CONFIGURATION INSTALLER"))
	{
	  usage();
	  return 1;
	}
    }
  if(!getcwd(dirbuffer,24)
     ||strncmp(dirbuffer,"/var/tmp/scpinst-",17))
    {
      usage();
      return 1;
    }
  signal(SIGCHLD,SIG_DFL);
  setsid();
  close(0);
  close(1);
  close(2);
  open("/dev/null",O_WRONLY);
  open("/dev/null",O_WRONLY);
  open("/dev/null",O_WRONLY);
  if(install_firmware)
    {
      lock_file=open(LOCK_FILENAME,O_RDWR|O_CREAT|O_EXCL,0600);
      if(lock_file<0)
	return 1;
      close(lock_file);
      log_file=open(LOG_FILENAME,O_RDWR|O_CREAT|O_EXCL,0600);
      if(log_file<0)
	return 1;
      write_log_message(log_file,"Starting update",0);
      /* firmware installation */

      /* check for mandatory files */
      if(chdir("update"))
	{
	  unlink(LOG_FILENAME);
	  unlink(LOCK_FILENAME);
	  return 1;
	}

      DIR *currdir;
      struct dirent *currfile;
      int cal_dmcu_found,cal_amcu_found,ver_major,ver_minor,
	fname_len,flash_write;
      static char cal_dmcu_name[256],cal_amcu_name[256],
	cal_dmcu_ver[8],cal_amcu_ver[8];

      cal_dmcu_found=0;
      cal_amcu_found=0;
      cal_dmcu_name[0]='\0';
      cal_amcu_name[0]='\0';
      cal_dmcu_ver[0]='\0';
      cal_amcu_ver[0]='\0';
      currdir=opendir(".");
      if(currdir)
	{
	  while((currfile=readdir(currdir)))
	    {
	      if((currfile->d_type==DT_REG)
		 &&((fname_len=strlen(currfile->d_name))>5)
		 &&(fname_len<=(sizeof(cal_dmcu_name)-1)))
		{
		  if(!strcmp(currfile->d_name+fname_len-4,".hex"))
		    {
		      if(!strncmp(currfile->d_name,"cal_dmcu",8))
			{
			  cal_dmcu_found=1;
			  strcpy(cal_dmcu_name,currfile->d_name);
			  if(sscanf(currfile->d_name+8,"_%d.%d",
				    &ver_major,&ver_minor)==2)
			    {
			      if(ver_major<0||ver_major>255
				 ||ver_minor<0||ver_minor>255)
				{
				  ver_major=0;
				  ver_minor=0;
				}
			    }
			  else
			    {
			      ver_major=0;
			      ver_minor=0;
			    }
			  sprintf(cal_dmcu_ver,"%d.%d",
				  ver_major,ver_minor);
			}
		      else
			if(!strncmp(currfile->d_name,"cal_amcu",8))
			  {
			    cal_amcu_found=1;
			    strcpy(cal_amcu_name,currfile->d_name);
			    if(sscanf(currfile->d_name+8,"_%d.%d",
				      &ver_major,&ver_minor)==2)
			      {
				if(ver_major<0||ver_major>255
				   ||ver_minor<0||ver_minor>255)
				  {
				    ver_major=0;
				    ver_minor=0;
				  }
			      }
			    else
			      {
				ver_major=0;
				ver_minor=0;
			      }
			    sprintf(cal_amcu_ver,"%d.%d",
				    ver_major,ver_minor);
			  }
		    }
		}
	    }
	  closedir(currdir);
	}
      if(cal_dmcu_found||cal_amcu_found)
	{
	  exec_argv[0]="/bin/avr-install";
	  exec_argv[1]="-h";
	  exec_argv[2]="127.0.0.1";
	  i=3;
	  if(cal_dmcu_found)
	    {
	      exec_argv[i++]="-d";
	      exec_argv[i++]=cal_dmcu_name;
	      if(strcmp(cal_dmcu_ver,"0.0"))
		{
		  exec_argv[i++]="-D";
		  exec_argv[i++]=cal_dmcu_ver;
		}
	    }
	  if(cal_amcu_found)
	    {
	      exec_argv[i++]="-a";
	      exec_argv[i++]=cal_amcu_name;
	      if(strcmp(cal_amcu_ver,"0.0"))
		{
		  exec_argv[i++]="-A";
		  exec_argv[i++]=cal_amcu_ver;
		}
	    }

	  exec_argv[i]=NULL;

	  write_log_message(log_file,"Updating MCUs",2);

	  pid=vfork();
	  if(pid<0)
	    {
	      unlink(LOG_FILENAME);
	      unlink(LOCK_FILENAME);
	      return 1;
	    }
	  if(pid==0)
	    {
	      execve(exec_argv[0],exec_argv,env);
	      _exit(1);
	    }
	  if(waitpid(pid,&status,0)!=pid)
	    {
	      unlink(LOG_FILENAME);
	      unlink(LOCK_FILENAME);
	      return 1;
	    }
	  if(cal_dmcu_found)
	    unlink(cal_dmcu_name);
	  if(cal_dmcu_found)
	    unlink(cal_amcu_name);
	}
      flash_write=1;
      if(stat("download.bit",&statbuf)||(statbuf.st_size==0))
	flash_write=0;
      if(stat("dt.dtb",&statbuf)||(statbuf.st_size==0))
	flash_write=0;
      if(stat("linux.bin.gz",&statbuf)||(statbuf.st_size==0))
	flash_write=0;
      if(stat("romfs.bin.gz",&statbuf)||(statbuf.st_size==0))
	flash_write=0;
      if(flash_write)
	{
	  /* extract identity information */
	  exec_argv[0]="/bin/mbbl-imagetool";
	  exec_argv[1]="-s";
	  exec_argv[2]="-0";
	  exec_argv[3]="-I";
	  exec_argv[4]="/dev/mtd0";
	  exec_argv[5]="-i";
	  exec_argv[6]="identity.txt";
	  exec_argv[7]=NULL;
	  
	  write_log_message(log_file,"Reading identity",3);
	  
	  pid=vfork();
	  if(pid<0)
	    {
	      unlink(LOG_FILENAME);
	      unlink(LOCK_FILENAME);
	      return 1;
	    }
	  if(pid==0)
	    {
	      execve(exec_argv[0],exec_argv,env);
	      _exit(1);
	    }
	  if((waitpid(pid,&status,0)!=pid)
	     ||!WIFEXITED(status)
	     ||(WEXITSTATUS(status)!=0))
	    {
	      unlink(LOG_FILENAME);
	      unlink(LOCK_FILENAME);
	      return 1;
	    }
	  
	  if(stat("identity.txt",&statbuf)||(statbuf.st_size==0))
	    {
	      unlink(LOG_FILENAME);
	      unlink(LOCK_FILENAME);
	      return 1;
	    }

	  /* all mandatory files present, build the command line for installer */

	  exec_argv[0]="/bin/mbbl-imagetool";
	  exec_argv[1]="-s";
	  exec_argv[2]="0x800000";
	  exec_argv[3]="-o";
	  exec_argv[4]="/dev/mtd0";
	  exec_argv[5]="-b";
	  exec_argv[6]="download.bit";
	  exec_argv[7]="-d";
	  exec_argv[8]="dt.dtb";
	  exec_argv[9]="-k";
	  exec_argv[10]="linux.bin.gz";
	  exec_argv[11]="-r";
	  exec_argv[12]="romfs.bin.gz";
	  exec_argv[13]="-i";
	  exec_argv[14]="identity.txt";
	  i=15;

	  /* optional files */
	  if(!stat("mbbl.elf",&statbuf)&&(statbuf.st_size!=0))
	    {
	      exec_argv[i++]="-e";
	      exec_argv[i++]="mbbl.elf";
	    }
	  if(!stat("logo-1.bin.gz",&statbuf)&&(statbuf.st_size!=0))
	    {
	      exec_argv[i++]="-l";
	      exec_argv[i++]="logo-1.bin.gz";
	    }
	  
	  if(!stat("8x12-font.bin.gz",&statbuf)&&(statbuf.st_size!=0))
	    {
	      exec_argv[i++]="-f";
	      exec_argv[i++]="8x12-font.bin.gz";
	      if(!stat("16x24-font.bin.gz",&statbuf)&&(statbuf.st_size!=0))
		{
		  exec_argv[i++]="-f";
		  exec_argv[i++]="16x24-font.bin.gz";
		}
	    }

	  /* check if /dev/mtd0 is an SPI flash device */
	  link_len=readlink("/sys/class/mtd/mtd0",link_buf,sizeof(link_buf)-1);
	  if(link_len>0)
	    {
	      link_buf[link_len]=0;
	      if(strstr(link_buf,".xps-spi/"))
		{
		  exec_argv[i++]="-N";
		}
	    }
	  
	  exec_argv[i]=NULL;

	  write_log_message(log_file,"Writing",5);
	  
	  pid=vfork();
	  if(pid<0)
	    {
	      unlink(LOG_FILENAME);
	      unlink(LOCK_FILENAME);
	      return 1;
	    }
	  if(pid==0)
	    {
	      execve(exec_argv[0],exec_argv,env);
	      _exit(1);
	    }
	  if((waitpid(pid,&status,0)!=pid)
	     ||!WIFEXITED(status)
	     ||(WEXITSTATUS(status)!=0))
	    {
	      unlink(LOG_FILENAME);
	      unlink(LOCK_FILENAME);
	      return 1;
	    }
	}
      chdir("/");
      exec_argv[0]="/bin/mtd-storage";
      exec_argv[1]="-F";
      exec_argv[2]="-o";
      exec_argv[3]="/dev/mtd0";
      exec_argv[4]="-s";
      exec_argv[5]="0x800000";
      exec_argv[6]="-e";
      exec_argv[7]="0xffffff";
      exec_argv[8]="var/tmp/persist";
      exec_argv[9]=NULL;

      write_log_message(log_file,"Updating user area",80);

      pid=vfork();
      if(pid<0)
	{
	  unlink(LOG_FILENAME);
	  unlink(LOCK_FILENAME);
	  return 1;
	}
      if(pid==0)
	{
	  execve(exec_argv[0],exec_argv,env);
	  _exit(1);
	}
      if((waitpid(pid,&status,0)!=pid)
	 ||!WIFEXITED(status)
	 ||(WEXITSTATUS(status)!=0))
	{
	  unlink(LOG_FILENAME);
	  unlink(LOCK_FILENAME);
	  return 1;
	}

      write_log_message(log_file,"Finished",100);

      exec_argv[0]="/bin/avahi-daemon";
      exec_argv[1]="-k";
      exec_argv[2]=NULL;

      pid=vfork();
      if(pid<0)
	{
	  unlink(LOG_FILENAME);
	  unlink(LOCK_FILENAME);
	  return 1;
	}
      if(pid==0)
	{
	  execve(exec_argv[0],exec_argv,env);
	  _exit(1);
	}
      waitpid(pid,&status,0);

      sleep(3);
      sync();
      int h;
      struct sockaddr_in serveraddr;
      FILE *f;

      serveraddr.sin_family=AF_INET;
      inet_aton("127.0.0.1",&serveraddr.sin_addr);
      serveraddr.sin_port=htons(2001);
      h=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
      if(h<0)
	{
	  unlink(LOG_FILENAME);
	  unlink(LOCK_FILENAME);
	  return 1;
	}

      if(connect(h,(struct sockaddr *)&serveraddr,
		 sizeof(serveraddr)))
	{
	  close(h);
	  unlink(LOG_FILENAME);
	  unlink(LOCK_FILENAME);
	  return 1;
	}
      f=fdopen(h,"r+");
      if(!f)
	{
	  close(h);
	  unlink(LOG_FILENAME);
	  unlink(LOCK_FILENAME);
	  return 1;
	}
      while(getc(f)!='\n');
      fputs("REBOOT REGULAR\n",f);
      fflush(f);
      while(getc(f)!='\n');
      unlink(LOG_FILENAME);
      fclose(f);
    }
  return 0;
}
