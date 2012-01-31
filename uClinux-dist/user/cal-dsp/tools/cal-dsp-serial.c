#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>

enum {
    DSP1,
    DSP_DEV_COUNT,
    DSP1_AMP1,DSP1_AMP2,
    DSP2,DSP2_AMP1,DSP2_AMP2,
    DSP3,DSP3_AMP1,DSP3_AMP2
};

int main( int US_UNUSED(argc), const char **argv )
{
    struct timeval t;
    int fd[DSP_DEV_COUNT];
    fd_set read_fds;
    fd_set write_fds;
    uint8_t count_in[DSP_DEV_COUNT];
    uint8_t count_out[DSP_DEV_COUNT];
    uint8_t count_error[DSP_DEV_COUNT];
    uint8_t in;
    int rv,event_count,i;
    const char * dsp_dev[DSP_DEV_COUNT] = {
            "/dev/dsp1",
#if 0
            "/dev/dsp1_amp1",
            "/dev/dsp1_amp2",
            "/dev/dsp2",
            "/dev/dsp2_amp1",
            "/dev/dsp2_amp2",
            "/dev/dsp3",
            "/dev/dsp3_amp1",
            "/dev/dsp3_amp2"
#endif
        };
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    for(i=0;i<DSP_DEV_COUNT;i++) {
        fd[i] = open(dsp_dev[i], O_RDWR);
        if(fd[i]<0) {
          printf("could not open %s\n",dsp_dev[i]);
          while(--i>=0) {
            close(fd[i]);
          }
          exit(1);
        }
        FD_SET(fd[i],&read_fds);
        FD_SET(fd[i],&write_fds);
        count_in[i]=0;
        count_out[i]=0;
        count_error[i]=0;
    }

    event_count=0;
    
    while(event_count<10) {
        printf("waiting for event\n");
        t.tv_sec=4;
        t.tv_usec=0;
        rv = select(fd[DSP_DEV_COUNT-1]+1,&read_fds,&write_fds,NULL,&t);
        printf("got event\n");
        event_count+=rv;
        if(rv==-1)
        {
            printf("An error occured: %d\n",rv);
            return(rv);
        }
        for(i=0;i<DSP_DEV_COUNT;i++) {
            if(FD_ISSET(fd[i],&read_fds)) {
                printf("readable\n");
                if(read(fd[i],&in,sizeof(uint8_t)) != sizeof(uint8_t)) {
                    printf("could not read from %s:%d\n",dsp_dev[i],fd[i]);
                    exit(1);
                }
                if(in!=count_in[i]) {
                    count_error[i]++;
                    count_in[i]=in;
                }
                count_in[i]++;
            } else {
                FD_SET(fd[i],&read_fds);
            }
            if(FD_ISSET(fd[i],&write_fds)) {
                printf("writeable\n");
                if(write(fd[i],&count_out[i],sizeof(uint8_t)) != sizeof(uint8_t)) {
                    printf("could not write to %s:%d\n",dsp_dev[i],fd[i]);
                    exit(1);
                }
                count_out[i]++;
            } else {
                FD_SET(fd[i],&write_fds);
            }
        }
    }

    for(i=0;i<DSP_DEV_COUNT;i++) {
        printf("[%d]\tout:%05d\tin:%05d\terror:%05d\n",count_out[i],count_in[i],count_error[i]);
    }

    for(i=0;i<DSP_DEV_COUNT;i++) {
        close(fd[i]);
    }
    printf("jiffy rules\n");

    return 0;
}

