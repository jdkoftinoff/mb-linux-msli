#include "io-osc_world.h"
#include "us_reactor.h"
#include "io-osc_gsbd_client.h"
#include "io-osc_app.h"

static bool io_osc_example_gsbd_read_callback( struct io_osc_gsbd_command_s *self, const char *response_string )
{
    bool r=false;
    if( response_string[0] == '2' && response_string[1] == '0' && response_string[2] == '0' )
    {
        char *endptr;
        int value = strtol(&response_string[4], &endptr, 10);
        if( *endptr=='\0' )
        {
            fprintf( stderr, "io_osc_server got value %d for command %s\n", value, self->m_cmdstring );
        }
        r=true;
    }
    if ( response_string[0] == '4' || response_string[0] == '5' )
    {
        /* error occurred */
        r=true;
    }
    return r;
}

static bool io_osc_example_gsbd_write_callback( struct io_osc_gsbd_command_s *self, const char *response_string )
{
    bool r=false;
    if( response_string[0] == '2' && response_string[1] == '0' && response_string[2] == '0' )
    {
        fprintf( stderr, "io_osc_server completed command %s\n", self->m_cmdstring );
        r=true;
    }
    if ( response_string[0] == '4' || response_string[0] == '5' )
    {
        /* error occurred */
        r=true;
    }
    return r;
}

static bool io_osc_example_gsbd_list_callback( struct io_osc_gsbd_command_s *self, const char *response_string )
{
    bool r=false;
    if( strcmp(response_string,"210 End of list" )== 0 )
    {
        r=true;
    }
    else
    {
        fprintf( stdout, "%s\n", response_string );
    }
    if ( response_string[0] == '4' || response_string[0] == '5' )
    {
        /* error occurred */
        r=true;
    }
    return r;
}


static bool
io_osc_example_gsbd_client(
    const char *gsbd_cmd,
    const char *gsbd_host,
    const char *gsbd_port,
    bool trace_mode
)
{
    bool r=false;
    us_reactor_t reactor;
    io_osc_gsbd_client_control_t client_control;
    r = us_reactor_init( &reactor, io_osc_sys_allocator, 4 );
    if ( r )
    {
        r = us_reactor_create_tcp_client(
                &reactor,
                io_osc_sys_allocator,
                (void *)&client_control,
                8192,
                gsbd_host, gsbd_port,
                true,
                io_osc_gsbd_client_create,
                io_osc_gsbd_client_init
            );
        if(r)
        {
            client_control.m_trace_mode = trace_mode;
            client_control.send_command( &client_control, gsbd_cmd, io_osc_example_gsbd_list_callback, 0 );
            client_control.send_command( &client_control, "READ BOARD1/RELAY_4", io_osc_example_gsbd_read_callback, 0 );
            client_control.send_command( &client_control, "READ BOARD0/TEMP_REMOTE", io_osc_example_gsbd_read_callback, 0 );
            client_control.send_command( &client_control, "READ BOARD0/TEMP_LOCAL", io_osc_example_gsbd_read_callback, 0 );
            client_control.send_command( &client_control, "WRITE BOARD1/RELAY_4=1", io_osc_example_gsbd_write_callback, 0 );
        }
    }
    if ( r )
    {
        while ( reactor.poll ( &reactor, 2000 ) )
        {
            /* do nothing until all work is done */
        }
        reactor.destroy ( &reactor );
        r = true;
    }
    return r;
}

int main( int argc, char **argv )
{
    bool r=false;
    const char * gsbd_host = "127.0.0.1";
    const char * gsbd_port = "2001";
    const char * gsbd_cmd = "LIST";
    bool trace_mode = false;
    r=io_osc_app_start( 32768 );
    if ( !r )
    {
        exit(1);
    }
    if ( argc==2 && strcmp("--help",argv[1])==0 )
    {
        fprintf(
            stdout,
            "usage:\n\t%s {gsbd command} {gsbd host} {gsbd port}\n\n"
            "With no params defauts to:\n\t%s '%s' '%s' '%s'\n",
            argv[0],
            argv[0],
            gsbd_cmd,
            gsbd_host,
            gsbd_port
        );
        io_osc_app_finish();
        exit(1);
    }
    if ( argc>1 )
    {
        gsbd_cmd = argv[1];
    }
    if ( argc>2 )
    {
        gsbd_host = argv[2];
    }
    if( argc>3 )
    {
        gsbd_port = argv[3];
    }
    if ( argc>4 )
    {
        if ( argv[4][0] == '-' && argv[4][1] == 't' )
        {
            trace_mode = true;
        }
    }
    r= io_osc_example_gsbd_client(
           gsbd_cmd,
           gsbd_host,
           gsbd_port,
           trace_mode
       );
    if (!r )
    {
        us_stderr->printf( us_stderr,
                           "Unable to run gsbd client\n"
                         );
    }
    io_osc_app_finish();
    return r!=true;
}

