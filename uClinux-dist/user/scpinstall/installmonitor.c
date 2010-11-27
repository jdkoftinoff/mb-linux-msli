#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/inotify.h>
#include <fcntl.h>
#include <errno.h>

#define LOG_FILENAME "/tmp/flash-update.log"
#define LOG_BUFFER_SIZE 256
#define NOTIFY_BUFFER_SIZE (1024 * sizeof(struct inotify_event) + 16)

char buffer[LOG_BUFFER_SIZE];
char notify_buffer[NOTIFY_BUFFER_SIZE];
struct stat statbuf;

int main(int argc,char **argv)
{
  int h_notify=-1,wd=-1,h_file=-1,l,lnotify;
  struct inotify_event *notify=NULL;
  do
    {
      h_file=open(LOG_FILENAME,O_RDONLY);
    }
  while((h_file<0)&&(sleep(1),1));
  
  if(((h_notify=inotify_init())<0)
     ||((wd=inotify_add_watch(h_notify,LOG_FILENAME,
			      IN_ATTRIB|IN_MODIFY|IN_DELETE_SELF))<0))
    {
      if(h_notify>=0)
	{
	  if(wd>=0)
	    inotify_rm_watch(h_notify,wd);
	  close(h_notify);
	}
      write(1,":(100%)\n",8);
      return 1;
    }

  while((l=read(h_file,buffer,LOG_BUFFER_SIZE))>0)
    write(1,buffer,l);
  while((lnotify=read(h_notify,notify_buffer,sizeof(notify_buffer)))>=0
	||errno==EINTR)
    {
      if(lnotify>0)
	{
	  for(notify=(struct inotify_event*)notify_buffer;
	      (char*)notify<notify_buffer+lnotify;
	      notify=(struct inotify_event*)((char*)(notify+1)+notify->len))
	    {
	      if(stat(LOG_FILENAME,&statbuf))
		{
		  write(1,":(100%)\n",8);
		  close(h_file);
		  close(wd);
		  close(h_notify);
		  return 0;
		}
	      while(((l=read(h_file,buffer,LOG_BUFFER_SIZE))>0)
		    ||errno==EINTR)
		write(1,buffer,l);
	    }
	}
    }
  write(1,":(100%)\n",8);
  inotify_rm_watch(h_notify,wd);
  close(h_notify);
  close(h_file);
  return 0;
}
