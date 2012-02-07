#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <signal.h>

#define DEVICES_COUNT 9
#define NCLIENTS_MAX 20
#define INPUT_BUFFER_SIZE 4096
#define OUTPUT_BUFFER_SIZE 256

static const char * device_names[DEVICES_COUNT] = {
    "/dev/dsp1",
    "/dev/dsp1_amp1",
    "/dev/dsp1_amp2",
    "/dev/dsp2",
    "/dev/dsp2_amp1",
    "/dev/dsp2_amp2",
    "/dev/dsp3",
    "/dev/dsp3_amp1",
    "/dev/dsp3_amp2"
};
static const int server_port[DEVICES_COUNT] = {
    1310,
    1311,
    1312,
    1320,
    1321,
    1322,
    1330,
    1331,
    1332
};

/* do nothing on alarm */
void alarmfunc(int n)
{
}

static bool cleanexit;
void cleanexitfunc(int n) {
    cleanexit=true;
}

struct network_client
{
//  char inputbuffer[INPUT_BUFFER_SIZE+1];
  int output_buffer_size;
  char *outputbuffer;
  int client_socket;
  int inputbufferlen;
  int outputbufferlen;
  int disconnectflag;
//  struct client_fifo rx_fifo;
//  struct client_fifo tx_fifo;
//  short int gsb_status;
//  short int gnet_status;
//  unsigned char *gnet_write_buffer;
//  int gnet_write_buffer_len;
//  int gnet_write_buffer_alloc;
//  int gnet_write_offset;
//  int reboot_device;
//  struct resources_list read_resources;
//  struct resources_list write_resources;
//  struct resources_list monitored_resources;
};
static int dev_fd[DEVICES_COUNT];
static int socket_fd[DEVICES_COUNT];
static struct network_client clients[DEVICES_COUNT][NCLIENTS_MAX];
static int nclients_total;
static int dev_nclients[DEVICES_COUNT];

int main( int argc, const char **argv )
{
    struct timeval t;
    int i,j;
    bool r=false;
    struct sockaddr_in client_addr;
    struct sockaddr_in server_addr;
    fd_set read_fds;
    fd_set write_fds;
    int fl;
    int max_fd;
    uint8_t in[1024];
    int rv;
    socklen_t addrlen;
    int client_socket;
    int read_count;
    int x;

    max_fd=0;
    nclients_total=0;
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    /* disable SIGPIPE handler, network errors are not fatal */
    signal(SIGPIPE,SIG_IGN);
  
    /* call empty handler on alarm (and interrupt pending blocking i/o) */
    signal(SIGALRM,alarmfunc);
    signal(SIGINT,cleanexitfunc);

    for(i=0;i<DEVICES_COUNT;i++) {
        dev_fd[i]=open(device_names[i],O_RDWR|O_NONBLOCK);
        socket_fd[i]=socket(AF_INET,SOCK_STREAM,0);
        if(socket_fd[i]<0) {
          perror("Can't create server socket");
          close(dev_fd[i]);
          i--;
          break;
        }
        memset(&server_addr,0,sizeof(server_addr));
        server_addr.sin_family=PF_INET;
        server_addr.sin_addr.s_addr=htonl(INADDR_ANY);
        server_addr.sin_port=htons(server_port[i]);
        if(bind(socket_fd[i],(struct sockaddr*)&server_addr, sizeof(server_addr)))
          {
            perror("Can't bind server socket");
            break;
          }
        if(listen(socket_fd[i],10))
          {
            perror("Can't listen on server socket");
            break;
          }
        fl=fcntl(socket_fd[i],F_GETFL,0);
        fcntl(socket_fd[i],F_SETFL,fl|O_NONBLOCK);
        dev_nclients[i]=0;

        if(socket_fd[i]>max_fd) {
            max_fd=socket_fd[i];
        }
        if(dev_fd[i]>max_fd) {
            max_fd=dev_fd[i];
        }
        FD_SET(socket_fd[i],&read_fds);
        //FD_SET(socket_fd[i],&write_fds);
        FD_SET(dev_fd[i],&read_fds);
        //FD_SET(dev_fd[i],&write_fds);
    }
    if(i<DEVICES_COUNT) {
        for(;i>=0;i--) {
            close(dev_fd[i]);
            close(socket_fd[i]);
        }
        return(1);
    }
    while(!cleanexit) {
        //printf("waiting for event\n");
        t.tv_sec=4;
        t.tv_usec=0;
        rv = select(max_fd+nclients_total+1+5,&read_fds,&write_fds,NULL,&t);
        printf(".");
        if((x++&0x1f)==0)
            printf("\n");
        for(i=0;i<DEVICES_COUNT;i++) {
            for(j=0;j<dev_nclients[i];j++) {
                if(FD_ISSET(clients[i][j].client_socket,&read_fds) /* && FD_ISSET(dev_fd[i],&write_fds) */) {
                    if((read_count=read(clients[i][j].client_socket,in,sizeof(in))) <= 0) {
                        FD_CLR(clients[i][j].client_socket,&read_fds);
                        printf("could not read from client %d for %s\n",j,device_names[i]);
                        close(clients[i][j].client_socket);
                        dev_nclients[i]--;
                        nclients_total--;
                        printf("%s now has %d clients, total clients is %d\n",device_names[i],dev_nclients[i],nclients_total);
                        break;
                    }
                    if(write(dev_fd[i],in,read_count) != read_count) {
                        printf("could not write to device %s:%d\n",device_names[i],dev_fd[i]);
                        break;
                    }
                }
                FD_SET(clients[i][j].client_socket,&read_fds);
            }

            if(FD_ISSET(dev_fd[i],&read_fds)) {
                if((read_count=read(dev_fd[i],&in,sizeof(in))) <= 0) {
                    printf("could not read from device %d for %s\n",dev_fd[i],device_names[i]);
                }
                for(j=0;j<dev_nclients[i];j++) {
                    if(write(clients[i][j].client_socket,&in,read_count) != read_count) {
                        FD_CLR(dev_fd[i],&read_fds);
                        printf("could not write to client %d %s\n",j,device_names[i]);
                        close(clients[i][j].client_socket);
                        dev_nclients[i]--;
                        nclients_total--;
                        printf("%s now has %d clients, total clients is %d\n",device_names[i],dev_nclients[i],nclients_total);
                        break;
                    }
                }
            }
            FD_SET(dev_fd[i],&read_fds);

            if(FD_ISSET(socket_fd[i],&read_fds)) {
                addrlen=sizeof(client_addr);
                client_socket=accept(socket_fd[i], (struct sockaddr*)&client_addr, &addrlen);
                if(client_socket>=0) {
                    if(nclients_total<NCLIENTS_MAX)
                    {
                      fl=fcntl(client_socket,F_GETFL,0);
                      fcntl(client_socket,F_SETFL,fl|O_NONBLOCK);
                      memset(&clients[i][dev_nclients[i]],0, sizeof(struct network_client));
                      clients[i][dev_nclients[i]].client_socket=client_socket;
                      printf("client connected to %s\n",device_names[i]);
                      if(clients[i][dev_nclients[i]].client_socket>max_fd) {
                        max_fd=clients[i][dev_nclients[i]].client_socket;
                      }
                      FD_SET(clients[i][dev_nclients[i]].client_socket,&read_fds);
                      dev_nclients[i]++;
                      nclients_total++;
                    }
                  else
                    {
                    printf("could not connect to client");
                    close(client_socket);
                    }
                }
            } else {
                FD_SET(socket_fd[i],&read_fds);
            }
        }
    }
    for(i=0;i<DEVICES_COUNT;i++) {
        for(j=0;j<dev_nclients[i];j++) {
            close(clients[i][j].client_socket);
        }
        close(socket_fd[i]);
        close(dev_fd[i]);
    }
    return 0;
}

