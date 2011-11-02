#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>
/* Believe it or not, but some arches have more than 32 SIGs!
 * HPPA: SIGSTKFLT == 36. */

static const char signals[][7] = {
	// SUSv3 says kill must support these, and specifies the numerical values,
	// http://www.opengroup.org/onlinepubs/009695399/utilities/kill.html
	// {0, "EXIT"}, {1, "HUP"}, {2, "INT"}, {3, "QUIT"},
	// {6, "ABRT"}, {9, "KILL"}, {14, "ALRM"}, {15, "TERM"}
	// And Posix adds the following:
	// {SIGILL, "ILL"}, {SIGTRAP, "TRAP"}, {SIGFPE, "FPE"}, {SIGUSR1, "USR1"},
	// {SIGSEGV, "SEGV"}, {SIGUSR2, "USR2"}, {SIGPIPE, "PIPE"}, {SIGCHLD, "CHLD"},
	// {SIGCONT, "CONT"}, {SIGSTOP, "STOP"}, {SIGTSTP, "TSTP"}, {SIGTTIN, "TTIN"},
	// {SIGTTOU, "TTOU"}

	[0] = "EXIT",
#ifdef SIGHUP
	[SIGHUP   ] = "HUP",
#endif
#ifdef SIGINT
	[SIGINT   ] = "INT",
#endif
#ifdef SIGQUIT
	[SIGQUIT  ] = "QUIT",
#endif
#ifdef SIGILL
	[SIGILL   ] = "ILL",
#endif
#ifdef SIGTRAP
	[SIGTRAP  ] = "TRAP",
#endif
#ifdef SIGABRT
	[SIGABRT  ] = "ABRT",
#endif
#ifdef SIGBUS
	[SIGBUS   ] = "BUS",
#endif
#ifdef SIGFPE
	[SIGFPE   ] = "FPE",
#endif
#ifdef SIGKILL
	[SIGKILL  ] = "KILL",
#endif
#ifdef SIGUSR1
	[SIGUSR1  ] = "USR1",
#endif
#ifdef SIGSEGV
	[SIGSEGV  ] = "SEGV",
#endif
#ifdef SIGUSR2
	[SIGUSR2  ] = "USR2",
#endif
#ifdef SIGPIPE
	[SIGPIPE  ] = "PIPE",
#endif
#ifdef SIGALRM
	[SIGALRM  ] = "ALRM",
#endif
#ifdef SIGTERM
	[SIGTERM  ] = "TERM",
#endif
#ifdef SIGSTKFLT
	[SIGSTKFLT] = "STKFLT",
#endif
#ifdef SIGCHLD
	[SIGCHLD  ] = "CHLD",
#endif
#ifdef SIGCONT
	[SIGCONT  ] = "CONT",
#endif
#ifdef SIGSTOP
	[SIGSTOP  ] = "STOP",
#endif
#ifdef SIGTSTP
	[SIGTSTP  ] = "TSTP",
#endif
#ifdef SIGTTIN
	[SIGTTIN  ] = "TTIN",
#endif
#ifdef SIGTTOU
	[SIGTTOU  ] = "TTOU",
#endif
#ifdef SIGURG
	[SIGURG   ] = "URG",
#endif
#ifdef SIGXCPU
	[SIGXCPU  ] = "XCPU",
#endif
#ifdef SIGXFSZ
	[SIGXFSZ  ] = "XFSZ",
#endif
#ifdef SIGVTALRM
	[SIGVTALRM] = "VTALRM",
#endif
#ifdef SIGPROF
	[SIGPROF  ] = "PROF",
#endif
#ifdef SIGWINCH
	[SIGWINCH ] = "WINCH",
#endif
#ifdef SIGPOLL
	[SIGPOLL  ] = "POLL",
#endif
#ifdef SIGPWR
	[SIGPWR   ] = "PWR",
#endif
#ifdef SIGSYS
	[SIGSYS   ] = "SYS",
#endif
};

void restart_myself(char **argv,char **env)
{
  fputs("restart_loop: Restarting myself\r\n",
	stderr);
  execve(argv[0],argv,env);
  perror("restart_loop: Can'r restart myself, giving up");
  fputs("\r\n",stderr);
  _exit(1);
}

int main(int argc, char **argv, char **env)
{
  pid_t p;
  int retval,ret;

  if(argc<2)
    {
      fprintf(stderr, "Usage: restart_loop <executable> <arguments>\n");
      return 1;
    }

  do
    {
      p=vfork();
      if(p<0)
	{
	  perror("vfork");
	  fputs("\r\n",stderr);
	  sleep(3);
	  restart_myself(argv,env);
	}
      if(p==0)
	{
	  /* child process */
	  execve(argv[1],&argv[1],env);
	  _exit(127);
	}
      else
	{
	  if(waitpid(p,&retval,0)==p)
	    {
	      if(WIFEXITED(retval))
		{
		  ret=WEXITSTATUS(retval);
		  if(ret==127)
		    {
		      fprintf(stderr,
			      "restart_loop: %s failed to run or exited "
			      "with status 127\r\n",
			      argv[1]);
		    }
		  else
		    {
		      fprintf(stderr,
			      "restart_loop: %s exited with status %d\r\n",
			      argv[1],ret);
		    }
		}
	      else
		{
		  
		  if(WIFSIGNALED(retval))
		    {
		      ret=WTERMSIG(retval);
		      if((ret>0)
			 &&(ret<(sizeof(signals)/sizeof(signals[0]))))
			{
			  fprintf(stderr,
				  "restart_loop: %s terminated by SIG%s\r\n",
				  argv[1],signals[ret]);
			}
		      else
			{
			  fprintf(stderr,
				  "restart_loop: %s terminated by unknown "
				  "signal %d\r\n",
				  argv[1],ret);
			}
		    }
		  else
		    {
		      fprintf(stderr,
			      "restart_loop: %s terminated for unknown "
			      "reason\r\n",
			      argv[1]);
		    }
		}
	    }
	  else
	    {
	      perror("restart_loop: Unknown result");
	      fputs("\r\n",stderr);
	      sleep(3);
	      restart_myself(argv,env);
	    }
	}
      sleep(2);
      fprintf(stderr, "restart_loop: Restarting %s\n",argv[1]);
    }
  while(1);
}
