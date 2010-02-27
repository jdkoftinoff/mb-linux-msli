/*
 * Semi-interactive MIDI loopback
 */

#include <stdio.h>
#include <termios.h>
#include <termio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <errno.h>
#include <string.h>

static unsigned char midibuf[1024],tmpstr[3]="   ";

int main(int argc,char **argv)
{
  struct termios term,stdinterm;
  fd_set readfd,writefd;
  struct timeval tv;
  unsigned char *readptr,*readend,*writeptr,*writeend;
  int i,h,l;


  /* save the original settings for the console */
  if(ioctl(0,TCGETS,&stdinterm)<0)
    {
      perror("ioctl TCGETS failed");
      return 1;
    }

  /* switch console into the raw mode */
  memcpy(&term,&stdinterm,sizeof(struct termios));

  term.c_cc[VMIN]=1;
  term.c_cc[VTIME]=0;
  term.c_iflag &= ~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);
  term.c_iflag |= (IGNBRK|IGNPAR);
  term.c_oflag &= ~OPOST;
  term.c_lflag &= ~(ECHO|ICANON|IEXTEN|ISIG);  
  term.c_cflag &= ~(CSIZE|PARENB);
  term.c_cflag |= (CS8|CREAD|CLOCAL);

  if(ioctl(0,TCSETS, &term)<0)
    {
      ioctl(0,TCSETS, &stdinterm);
      perror("ioctl TCSETS failed");
      return 1;
    }


  /* do the same for the MIDI port, plus use nonblocking mode */
  h=open("/dev/ttyMIDI",O_RDWR);
  if(h<0)
    {
      ioctl(0,TCSETS, &stdinterm);
      perror("opening /dev/ttyMIDI");
      return 1;
    }

  if(ioctl(h,TCGETS,&term)<0)
    {
      ioctl(0,TCSETS, &stdinterm);
      perror("ioctl TCGETS failed");
      close(h);
      return 1;
    }
  
  term.c_cc[VMIN]=1;
  term.c_cc[VTIME]=0;
  term.c_iflag &= ~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);
  term.c_iflag |= (IGNBRK|IGNPAR);
  term.c_oflag &= ~OPOST;
  term.c_lflag &= ~(ECHO|ICANON|IEXTEN|ISIG);  
  term.c_cflag &= ~(CSIZE|PARENB);
  term.c_cflag |= (CS8|CREAD|CLOCAL);

  if(ioctl(h,TCSETS, &term)<0)
    {
      ioctl(0,TCSETS, &stdinterm);
      perror("ioctl TCSETS failed");
      close(h);
      return 1;
    }
  
  if(fcntl(h,F_SETFL,O_NONBLOCK)<0)
    {
      ioctl(0,TCSETS, &stdinterm);
      perror("fcntl F_SETFL O_NONBLOCK failed");
      close(h);
      return 1;
    }
  
  write(1,"MIDI loopback, press any key to exit",
	sizeof("MIDI loopback, press any key to exit"));

  /*
    nonblocking loop with circular buffer for MIDI terminal device,
    console I/O is blocking
  */
  readptr=midibuf;
  readend=midibuf+sizeof(midibuf);
  writeptr=midibuf;
  writeend=midibuf;
  while(1)
    {
      FD_ZERO(&readfd);
      FD_ZERO(&writefd);
      tv.tv_sec=10;
      tv.tv_usec=0;

      /* check for the console input */
      FD_SET(0,&readfd);

      /* can read MIDI? */
      if(readend!=readptr)
	{
	  FD_SET(h,&readfd);
	}

      /* any MIDI data to write? */
      if(writeend!=writeptr)
	{
	  FD_SET(h,&writefd);
	}

      /* check for all activity */
      if(select(h+1,&readfd,&writefd,NULL,&tv)<0 && errno!=EAGAIN && errno!=ERESTART)
	{
	  /* failure */
	  ioctl(0,TCSETS, &stdinterm);
	  perror("select");
	  close(h);
	  return 1;
	}

      /* console input received, read character and exit */
      if(FD_ISSET(0,&readfd))
	{
	  read(0,midibuf,1);
	  ioctl(0,TCSETS, &stdinterm);
	  close(h);
	  printf("\nExit\n");
	  return 0;
	}

      /* MIDI data arrived */
      if(FD_ISSET(h,&readfd))
	{
	  l=read(h,readptr,readend-readptr);
	  if(l>=0 || errno==EAGAIN || errno==ERESTART)
	    {
	      if(l>0)
		{
		  for(i=0;i<l;i++)
		    {
		      /* print MIDI data in a semi-human-readable format */
		      if(readptr[i]&0x80)
			{
			  write(1,"\r\n",sizeof("\r\n"));
			}
		      /* hex output */
		      tmpstr[0]=(readptr[i]>>4)+'0';
		      if(tmpstr[0]>'9') tmpstr[0]+='a'-'9'-1;
		      tmpstr[1]=(readptr[i]&0x0f)+'0';
		      if(tmpstr[1]>'9') tmpstr[1]+='a'-'9'-1;
		      write(1,tmpstr,sizeof(tmpstr));
		    }

		  readptr+=l;
		  if(readptr>writeend)
		    {
		      writeend=readptr;
		    }
		  if(readptr==midibuf+sizeof(midibuf))
		    {
		      readptr=midibuf;
		      readend=writeptr;
		    }
		}
	    }
	  else
	    {
	      ioctl(0,TCSETS, &stdinterm);
	      perror("read failed");
	      close(h);
	      return 1;
	    }
	}

      /* 
	 if either file descriptor became available, or we just read some data, 
	 try to write it 
      */
      if(FD_ISSET(h,&writefd) || (FD_ISSET(h,&readfd) && writeend!=writeptr))
	{
	  l=write(h,writeptr,writeend-writeptr);
	  if(l>=0 || errno==EAGAIN || errno==ERESTART)
	    {
	      if(l>0)
		{
		  writeptr+=l;
		  if(writeptr>readend)
		    {
		      readend=writeptr;
		    }
		  if(writeptr==midibuf+sizeof(midibuf))
		    {
		      writeptr=midibuf;
		      writeend=readptr;
		    }
		}
	    }
	  else
	    {
	      ioctl(0,TCSETS, &stdinterm);
	      perror("write failed");
	      close(h);
	      return 1;
	    }
	}
    }
  /* not reached */
}
