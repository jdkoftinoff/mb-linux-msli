/*
 * Copyright (c) 2004, Nate Nielsen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 * 
 *     * Redistributions of source code must retain the above 
 *       copyright notice, this list of conditions and the 
 *       following disclaimer.
 *     * Redistributions in binary form must reproduce the 
 *       above copyright notice, this list of conditions and 
 *       the following disclaimer in the documentation and/or 
 *       other materials provided with the distribution.
 *     * The names of contributors to this software may not be 
 *       used to endorse or promote products derived from this 
 *       software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
 * DAMAGE.
 * 
 *
 * CONTRIBUTORS
 *  Nate Nielsen <nielsen@memberwebs.com>
 */ 

#include <config/autoconf.h>

#include <sys/types.h>
#include <sys/param.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>

#include "usuals.h"

#include "compat.h"
#include "sock_any.h"
#include "stringx.h"

#define SP_LEGACY_OPTIONS
#include "smtppass.h"

#ifdef CONFIG_USER_TRUSTEDSOURCE_BLACKLIST
#include "libconfig.h"
#include "sg_ipset.h"
#include "sg_ipr.h"
#endif

#ifdef CONFIG_USER_TRUSTEDSOURCE_V2
#include "librep.h"
#endif

/* -----------------------------------------------------------------------
 *  STRUCTURES
 */

typedef struct clstate
{
    /* Settings ------------------------------- */  
    struct sockaddr_any clamaddr;   /* Address for connecting to clamd */
    const char* clamname;   
    const char* directory;          /* The directory for temp files */
    const char* virusaction;        /* Program to run when event occurs */
    int bounce;                     /* Send back a reject line */
    int quarantine;                 /* Leave virus files in temp dir */
    int debug_files;                /* Leave all files in temp dir */
    int passthrough;                /* if the server is running in passthrough (no AV */
}
clstate_t;

typedef struct clctx
{ 
    spctx_t sp;             /* The main sp context */
    spio_t clam;            /* Connection to clamd */
}
clctx_t;

/* -----------------------------------------------------------------------
 *  STRINGS
 */

#define CRLF                "\r\n"

#define SMTP_DATAVIRUSOK    "250 Virus Detected; Discarded Email" CRLF
#define SMTP_DATAVIRUS      "550 Virus Detected; Content Rejected" CRLF

#define CLAM_OK             "OK"
#define CLAM_ERROR          "ERROR"
#define CLAM_FOUND          "FOUND"

#define CLAM_SCAN           "SCAN "

#ifdef USE_CLAM_SESSION
#define CONNECT_RSP         "PONG"
#define CLAM_CONNECT        "SESSION\nPING\n"
#define CLAM_DISCONNECT     "END\n"
#endif

#define DEFAULT_CONFIG      CONF_PREFIX "/clamsmtpd.conf"
#define DEFAULT_CLAMAV      "/var/run/clamav/clamd"
#define DEFAULT_HEADER      "X-Virus-Scanned: ClamAV using ClamSMTP"

/* -----------------------------------------------------------------------
 *  CONFIGURATION OPTIONS
 * 
 * - Be sure your option is relevant to this file. Certain options
 *   should go into smtppass.c
 * - Add field to clstate_t structure (above)
 * - Add default (above) and set in main (below). Required options 
 *   are difficult to implement under the current structure. It's 
 *   better to have a sane default.
 * - Add config keyword (below)
 * - Parsing and validation of option in cb_parse_option (below)
 * - Document in the sample doc/clamsmtpd.conf
 * - Document in doc/clamsmtpd.conf.5 
 */
 
#define CFG_CLAMADDR    "ClamAddress"
#define CFG_DIRECTORY   "TempDirectory"
#define CFG_HEADER      "Header"
#define CFG_SCANHEADER  "ScanHeader"
#define CFG_BOUNCE      "Bounce"
#define CFG_QUARANTINE  "Quarantine"
#define CFG_DEBUGFILES  "DebugFiles"
#define CFG_VIRUSACTION "VirusAction"
#define CFG_TRUSTEDSOURCE "TrustedSourceServer"
#define CFG_SPAMTHRESHOLD "SpamThreshold"
#define CFG_TRUSTEDSOURCE_ENABLED "TrustedSourceEnabled"
#define CFG_SERIALKEY    "SerialKey"
#define CFG_VENDORSTRING "VendorString"
#define CFG_TRUSTEDSOURCEV2_ENABLED "TrustedSourcev2Enabled"
#define CFG_TRUSTEDSOURCEV2_SERVER "TrustedSourcev2Server"
#define CFG_TRUSTEDSOURCEV2_THRESHOLD "TrustedSourcev2Threshold"
#define CFG_TRUSTEDSOURCE_BLACKLIST "TrustedSourceBlacklist"
#define CFG_PASSTHROUGH  "Passthrough"


/* -----------------------------------------------------------------------
 *  GLOBALS
 */
 
clstate_t g_clstate;

/*
 * Anti-spam globals
 */
char *g_trustedsource_server = NULL;        /* server address */
char *g_trustedsourcev2_server = NULL;        /* server address */
char *g_serial = NULL;                      /* serial/key */
char *g_vendor = NULL;                      /* vendor string */
int g_threshold = 0;                        /* reject threshold */
int g_thresholdv2 = 0;                        /* v2 reject threshold */
int g_trustedsource_enabled = 0;            /* lookups enabled or disabled */
int g_trustedsourcev2_enabled = 0;          /* ver 2 lookups enabled or disabled */
int g_trustedsource_bwlist = 0;		    /* trustedsource blacklisting */ 
#ifdef CONFIG_USER_TRUSTEDSOURCE_BLACKLIST
#ifdef CONFIG_USER_TRUSTEDSOURCE
ipset white_list; 	    /* trustedsource whitelist */
ipset black_list; 	    /* trustedsource blacklist */
#endif
#ifdef CONFIG_USER_TRUSTEDSOURCE_V2
ipset white_listv2; 	    /* trustedsourcev2 whitelist */
ipset black_listv2; 	    /* trustedsourcev2 blacklist */
#endif
#endif

extern int h_errno;

/* -----------------------------------------------------------------------
 *  FORWARD DECLARATIONS
 */

static void usage();
static int connect_clam(clctx_t* ctx);
static int disconnect_clam(clctx_t* ctx);
static int virus_action(clctx_t* ctx, const char* virus);
static int clam_scan_file(clctx_t* ctx, const char** virus);

/* -----------------------------------------------------------------------
 *  SIMPLE MACROS
 */

/* ----------------------------------------------------------------------------------
 *  STARTUP ETC...
 */

#ifndef HAVE___ARGV
char** __argv;
#endif


#ifdef CONFIG_USER_TRUSTEDSOURCE_BLACKLIST
/* -------------------------------------------
 * Snapgear - specific config parsing routines 
 */

/* Resolve and add the given hostname to the passed IP address set.
 */
static void add_hostname_to_set(const char *hostname, ipset addrs) {
        int               j;
        struct hostent   *h = gethostbyname(hostname);
        if (h == NULL)
                return;
        struct in_addr  **al = (struct in_addr **)h->h_addr_list;

        for (j=0; al[j] != NULL; j++)
                ipset_add_address(addrs, ntohl(al[j]->s_addr));
}


/* Routine for decoding a firewall address group into its raw IP addresses.
 *
 * The second two arguments are IP address sets.  The first is used for adding
 * ranges to and the second is for adding singleton addresses to.  Neither can
 * be NULL but they can be the same list.
 */
static void add_all_fwaddress(config_handle cfg, config_ref r, ipset addrs) {
        if (config_ref_null(r))
                return;

        /* Decode the address record */
        if (config_provides(r, "firewall.fwaddress")) {
                s_iprange       ipr;
                struct in_addr  ip;

                inet_aton(config_get(cfg, r, "lower"), &ip);
                ipr.lower = ntohl(ip.s_addr);
                inet_aton(config_get(cfg, r, "upper"), &ip);
                ipr.upper = ntohl(ip.s_addr);
                ipset_add_range(addrs, ipr);
        } else if (config_provides(r, "firewall.fwhostname")) {
                add_hostname_to_set(config_get(cfg, r, "hostname"), addrs);
        } else if (config_provides(r, "firewall.fwaddress_group")) {
                size_t           count;
                int              i;
                config_ref      *refs = config_find_all(cfg,
                                        config_subtype(cfg, r, "address"),
                                        &count, CONFIG_FIND_NORMAL);
                for (i=0; i<count; i++)
                        add_all_fwaddress(cfg, config_parseref(config_get(cfg, refs[i], "address")), addrs);
                free(refs);
        }
}
/* 
 * -------------------------------------------
 */

#endif


int main(int argc, char* argv[])
{
    const char* configfile = DEFAULT_CONFIG;
    const char* pidfile = NULL;
    int dbg_level = -1;
    int warnargs = 0;
    int ch = 0;
    int r;
    char* t;
    
#ifndef HAVE___ARGV
    __argv = argv;
#endif

    /* Configuration defaults */
    memset(&g_clstate, 0, sizeof(g_clstate));
    g_clstate.directory = _PATH_TMP;

    /* We need the default to parse into a useable form, so we do this: */
    r = cb_parse_option(CFG_CLAMADDR, DEFAULT_CLAMAV);
    ASSERT(r == 1);
    
    sp_init("clamsmtpd");

    /* COMPAT: Setup a default header */
    sp_parse_option(CFG_HEADER, DEFAULT_HEADER);
    
    /* 
     * We still accept our old arguments for compatibility reasons.
     * We fill them into the spstate structure directly 
     */

    /* Parse the arguments nicely */
    while((ch = getopt(argc, argv, "bc:d:D:f:h:l:m:p:qt:v")) != -1)
    {
        switch(ch)
        {
        /* COMPAT: Actively reject messages */
        case 'b':
            if((r = cb_parse_option(CFG_BOUNCE, "on")) < 0)
                usage();
            ASSERT(r == 1);
            warnargs = 1;
            break;

        /* COMPAT: Change the CLAM socket */
        case 'c':
            if((r = cb_parse_option(CFG_CLAMADDR, "on")) < 0)
                usage();
            ASSERT(r == 1);
            warnargs = 1;
            break;

		/*  Don't daemonize  */
        case 'd':
            dbg_level = strtol(optarg, &t, 10);
            if(*t) /* parse error */
                errx(1, "invalid debug log level");
            dbg_level += LOG_ERR;
            break;
            
        /* COMPAT: The directory for the files */
        case 'D':
            if((r = sp_parse_option(CFG_DIRECTORY, optarg)) < 0)
                usage();
            ASSERT(r == 1);
            warnargs = 1;
            break;
            
        /* The configuration file */
        case 'f':
            configfile = optarg;
            break;
            
        /* COMPAT: The header to add */
        case 'h':
            if((r = cb_parse_option(CFG_HEADER, optarg)) < 0)
                usage();
            ASSERT(r == 1);
            warnargs = 1;
            break;

        /* COMPAT: Change our listening port */
        case 'l':
            if((r = sp_parse_option("Listen", optarg)) < 0)
                usage();
            ASSERT(r == 1);
            warnargs = 1;
            break;

        /* COMPAT: The maximum number of threads */
        case 'm':
            if((r = sp_parse_option("MaxConnections", optarg)) < 0)
                usage();
            ASSERT(r == 1);
            warnargs = 1;
            break;

        /* Write out a pid file */
        case 'p':
            pidfile = optarg;
            break;    

        /* COMPAT: The timeout */
		case 't':
            if((r = sp_parse_option("TimeOut", optarg)) < 0)
                usage();
            ASSERT(r == 1);
            warnargs = 1;
			break;
          
        /* COMPAT: Leave virus files in directory */
        case 'q':
            if((r = cb_parse_option(CFG_QUARANTINE, "on")) < 0)
                usage();
            ASSERT(r == 1);
            warnargs = 1;
            break;
            
        /* Print version number */
        case 'v':
            printf("clamsmtpd (version %s)\n", VERSION);
            printf("          (config: %s)\n", DEFAULT_CONFIG);
            exit(0);
            break;

        /* COMPAT: Leave all files in the tmp directory */
        case 'X':
            if((r = cb_parse_option(CFG_DEBUGFILES, "on")) < 0)
                usage();
            ASSERT(r == 1);
            warnargs = 1;
            break;
            
        /* Usage information */
        case '?':
        default:
            usage();
            break;
		}
    }
    
	argc -= optind;
	argv += optind;
 
    if(argc > 1)
        usage();
    if(argc == 1)
    {
        /* COMPAT: The out address */
        if((r = sp_parse_option("OutAddress", argv[0])) < 0)
            usage();
        ASSERT(r == 1);
        warnargs = 1;
    }

    if(warnargs)
        warnx("please use configuration file instead of command-line flags: %s", configfile);
        
    r = sp_run(configfile, pidfile, dbg_level);
    
    sp_done();
    
    return r;
}

static void usage()
{
    fprintf(stderr, "usage: clamsmtpd [-d debuglevel] [-f configfile] [-p pidfile]\n");
    fprintf(stderr, "       clamsmtpd -v\n");
    exit(2);
}

/* ----------------------------------------------------------------------------------
 *  SP CALLBACKS
 */
 
int cb_check_data(spctx_t* sp)
{
    int r = 0;
    const char* virus;
    clctx_t* ctx = (clctx_t*)sp;

    /* if it's configured as a passthrough, then no av */
    /* shouldn't ever get here anyway */
    if(g_clstate.passthrough) {
        return sp_done_data(sp);
    }

#ifdef CONFIG_USER_CLAMAV_CLAMAV
    /* ClamAV doesn't like empty files */
    if((r = sp_cache_data(sp)) > 0)
    {    
        /* Connect to clamav */
        if(!spio_valid(&(ctx->clam)))
            r = connect_clam(ctx);
    
        if(r != -1)
            r = clam_scan_file(ctx, &virus);
    }

    switch(r)
    {
      
    /* 
     * There was an error tell the client. We haven't notified 
     * the server about any of this yet 
     */
    case -1:
        if(sp_fail_data(sp, NULL) == -1)
            return -1;
        break;
        
    /*
     * No virus was found. Now we initiate a connection to the server
     * and transfer the file to it.
     */ 
    case 0:
#ifdef CONFIG_PROP_STATSD_STATSD
		system("statsd -a incr clamav-smtp total");
#endif
        if(sp_done_data(sp) == -1)
            return -1;
        break;

    /*
     * A virus was found, normally we just drop the email. But if 
     * requested we can send a simple message back to our client.
     * The server doesn't know data was ever sent, and the client can
     * choose to reset the connection to reuse it if it wants.
     */
    case 1:
        /* Any special post operation actions on the virus */
        virus_action(ctx, virus);
        
#ifdef CONFIG_PROP_STATSD_STATSD
		system("statsd -a incr clamav-smtp infected \\;"
			            " incr clamav-smtp total");
#endif
        if(sp_fail_data(sp, g_clstate.bounce ? 
                            SMTP_DATAVIRUS : SMTP_DATAVIRUSOK) == -1)
            return -1;
        break;
        
    default:
        ASSERT(0 && "Invalid clam_scan_file return value");
        break;
    };
    
    return 0;
#else
    return 0;
#endif
}

int cb_parse_option(const char* name, const char* value)
{
    char *t;
    if(strcasecmp(CFG_CLAMADDR, name) == 0)
    {
        if(sock_any_pton(value, &(g_clstate.clamaddr), SANY_OPT_DEFLOCAL) == -1)
            errx(2, "invalid " CFG_CLAMADDR " socket name: %s", value);               
        g_clstate.clamname = value;        
        return 1;
    }
            
	/* COMPAT: Parse old header option */
    else if(strcasecmp(CFG_SCANHEADER, name) == 0)
    {
        warnx("please use \"Header\" option instead of \"ScanHeader\"");
        return sp_parse_option(CFG_HEADER, value);
    }
                        
    else if(strcasecmp(CFG_DIRECTORY, name) == 0)
    {
        g_clstate.directory = value;
        return 1;
    }
            
    else if(strcasecmp(CFG_BOUNCE, name) == 0)
    {
        if((g_clstate.bounce = strtob(value)) == -1)
            errx(2, "invalid value for " CFG_BOUNCE);
        return 1;
    }
        
    else if(strcasecmp(CFG_QUARANTINE, name) == 0)
    {
        if((g_clstate.quarantine = strtob(value)) == -1)
            errx(2, "invalid value for " CFG_BOUNCE);
        return 1;
    }
    
    else if(strcasecmp(CFG_DEBUGFILES, name) == 0)
    {
        if((g_clstate.debug_files = strtob(value)) == -1)
            errx(2, "invalid value for " CFG_DEBUGFILES);
        return 1;
    }
    
    else if(strcasecmp(CFG_VIRUSACTION, name) == 0)
    {
        g_clstate.virusaction = value;
        return 1;
    }

#ifdef CONFIG_USER_TRUSTEDSOURCE
    else if(strcasecmp(CFG_TRUSTEDSOURCE, name) == 0)
    {
        g_trustedsource_server = (char *)value;
        return 1;
    }

    else if(strcasecmp(CFG_SPAMTHRESHOLD, name) == 0)
    {
        g_threshold = strtol(value, &t, 10);
        if(*t)
            errx(2, "invalid setting for " CFG_SPAMTHRESHOLD);
        return 1;
    }

    else if(strcasecmp(CFG_TRUSTEDSOURCE_ENABLED, name) == 0)
    {
        if((g_trustedsource_enabled = strtob(value)) == -1)
            errx(2, "invalid value for " CFG_TRUSTEDSOURCE_ENABLED);
        return 1;
    }

    else if(strcasecmp(CFG_SERIALKEY, name) == 0)
    {
        g_serial = (char *)value;
        return 1;
    }

    else if(strcasecmp(CFG_VENDORSTRING, name) == 0)
    {
        g_vendor = (char *)value;
        return 1;
    }


#endif
#ifdef CONFIG_USER_TRUSTEDSOURCE_V2
    else if(strcasecmp(CFG_TRUSTEDSOURCEV2_ENABLED, name) == 0)
    {
        if((g_trustedsourcev2_enabled = strtob(value)) == -1)
            errx(2, "invalid value for " CFG_TRUSTEDSOURCEV2_ENABLED);
        return 1;
    }

    else if(strcasecmp(CFG_TRUSTEDSOURCEV2_SERVER, name) == 0)
    {
        g_trustedsourcev2_server = (char *)value;
        return 1;
    }

    else if(strcasecmp(CFG_TRUSTEDSOURCEV2_THRESHOLD, name) == 0)
    {
        g_thresholdv2 = strtol(value, &t, 10);
        if(*t)
            errx(2, "invalid setting for " CFG_TRUSTEDSOURCEV2_THRESHOLD);
        return 1;
    }

#endif

#ifdef CONFIG_USER_TRUSTEDSOURCE_BLACKLIST
    else if(strcasecmp(CFG_TRUSTEDSOURCE_BLACKLIST, name) == 0)
    {
	if((g_trustedsource_bwlist = strtob(value)) == -1) {
		errx(2, "invalid value for " CFG_TRUSTEDSOURCE_BLACKLIST);
	}

	if(g_trustedsource_bwlist) {
		/* load the black and white lists from the SG metaconfig */
		config_handle cfg = config_open();
		config_ref cref;
#ifdef CONFIG_USER_TRUSTEDSOURCE
		white_list = ipset_new(20);
		black_list = ipset_new(20);
		cref = config_parseref("trustedsource");
		add_all_fwaddress(cfg, config_parseref(config_get(cfg, cref, "white_list")), white_list);
		add_all_fwaddress(cfg, config_parseref(config_get(cfg, cref, "black_list")), black_list);
#endif
#ifdef CONFIG_USER_TRUSTEDSOURCE_V2
		white_listv2 = ipset_new(20);
		black_listv2 = ipset_new(20);
		cref = config_parseref("trustedsourcev2");
		add_all_fwaddress(cfg, config_parseref(config_get(cfg, cref, "white_list")), white_listv2);
		add_all_fwaddress(cfg, config_parseref(config_get(cfg, cref, "black_list")), black_listv2);
#endif
		/* ... and we have our lists, time to close the config */
		config_abort(cfg); 
	}
        return 1;
    }
#endif

   else if(strcasecmp(CFG_PASSTHROUGH, name) == 0)
    {
        if((g_clstate.passthrough = strtob(value)) == -1)
            errx(2, "invalid value for " CFG_PASSTHROUGH);
        return 1;
    }

    return 0;
}

spctx_t* cb_new_context()
{
    clctx_t* ctx = (clctx_t*)calloc(1, sizeof(clctx_t));
    if(!ctx)
    {
        sp_messagex(NULL, LOG_CRIT, "out of memory");
        return NULL;
    }
    
    /* Initial preparation of the structure */
    spio_init(&(ctx->clam), "CLAMAV");
    return &(ctx->sp);
}  

void cb_del_context(spctx_t* sp)
{
    clctx_t* ctx = (clctx_t*)sp;   
    int x; 
    ASSERT(sp);
    
    disconnect_clam(ctx);
    free(ctx);
    
    if(g_clstate.virusaction)
    {
        /* Cleanup any old actions */
        while(waitpid(-1, &x, WNOHANG) > 0)
            ;
    }
}

#ifdef CONFIG_USER_TRUSTEDSOURCE_BLACKLIST
/* returns 1 if the address is whitelisted
 * returns -1 if the address is blacklisted
 * returns 0 if the address isn't in either list
 * This is so that it lines up with what is expected from cb_check_client()
 */
int check_bwlist_access(ipset white, ipset black, const unsigned long srcip) {
	struct in_addr addr;
	addr.s_addr = htonl(srcip);

        if (ipset_contains_address(white, srcip)) {
		sp_messagex(NULL, LOG_ALERT, "client connection from %s is whitelisted", inet_ntoa(addr));
                return 1;
	} else if (ipset_contains_address(black, srcip)) {
		sp_messagex(NULL, LOG_ALERT, "client connection from %s is blacklisted", inet_ntoa(addr));
		return -1;
        } else {
                return 0;
	}
}
#endif

int cb_check_client(spctx_t* sp, struct sockaddr_any* peeraddr)
{
#ifdef CONFIG_USER_TRUSTEDSOURCE_V2
    struct rep_network *rn;
    struct rep_query *rq;
    rep_response rr;
#endif

#ifdef CONFIG_USER_TRUSTEDSOURCE
    char* namebuf = NULL;
    struct hostent *response = NULL;
    int x = 0, z = 0; 
    int score = 0;
    unsigned long my_ip;
    int bwlist;

    my_ip = ntohl(peeraddr->s.in.sin_addr.s_addr);

    if (!g_trustedsource_enabled) goto trustedsourcev2;

    sp_messagex(sp, LOG_DEBUG, "checking client");

#ifdef CONFIG_USER_TRUSTEDSOURCE_BLACKLIST
    if (g_trustedsource_bwlist && (bwlist = check_bwlist_access(white_list, black_list, my_ip))) {
	/* I decided that TSv1 takes priority over TSv2 static listing. */
	return bwlist;
    } /* else: do the ts v1 or 2 lookup as normal */
#endif

    /* piece together the 'domain name' to lookup */
    namebuf = malloc(sizeof(char) * 256);
    if (namebuf == NULL) {
        sp_message(sp, LOG_ERR, "error creating anti-spam lookup");
        return 1; 
    }

    /* b.<SERIAL>-<VENDOR>.d.c.b.a.ts-api.ciphertrust.net */
    sprintf(namebuf, "b.%s-%s.%d.%d.%d.%d.%s",
        g_serial,
        g_vendor,
        (my_ip >> 0) & 0xff,
        (my_ip >> 8) & 0xff,
        (my_ip >> 16) & 0xff,
        (my_ip >> 24) & 0xff,
        g_trustedsource_server
        );

    sp_messagex(sp, LOG_DEBUG, "doing trustedsource lookup for %s", namebuf);

    /* do the gethostbyname lookup */
    response = gethostbyname(namebuf);
    free(namebuf);

    if (response == NULL) {
        /* on error, log it ... */
        sp_messagex(NULL, LOG_ALERT, "error %d getting trusted source rating", h_errno);
        /* ... and allow it through */
        return 1; 
    }
    my_ip = ntohl(*(unsigned long *)(response->h_addr));

    sp_messagex(sp, LOG_DEBUG, "trustedsource response: %s\n", inet_ntoa(*((struct in_addr *)response->h_addr)));

    /* on success, decode the response and allow or deny appropriately:
     * if response is w.x.y.z then the score is as follows:
     * if (x & 1) == 1 then score = -z
     * else score = z
     */

    x = (my_ip >> 16) & 0x01;
    z = (my_ip >> 0) & 0xff;

    if (x) {
        score = -z;
    } else {
        score = z;
    }

    /* check the score against the threshold and decide to drop or allow */
    /* The higher the score the 'worse' the client's reputation */
    if (score >= g_threshold) {
        sp_messagex(NULL, LOG_ALERT, "client connection from %s is untrusted (score: %d)", inet_ntoa(peeraddr->s.in.sin_addr), score);
#ifdef CONFIG_PROP_STATSD_STATSD
		system("statsd -a incr trustedsource spam \\;"
			            " incr trustedsource total");
#endif
        return -1;
    } else {
#ifdef CONFIG_PROP_STATSD_STATSD
		system("statsd -a incr trustedsource total");
#endif
        sp_messagex(NULL, LOG_INFO, "client connection from %s is trusted (score: %d)", inet_ntoa(peeraddr->s.in.sin_addr), score);

        return 1;
    }
trustedsourcev2:

#endif

#ifdef CONFIG_USER_TRUSTEDSOURCE_V2
    /* If there are any failures in performing the query, we return success */

    /*
     * DEBUG
     * sp_messagex(sp, LOG_ERR, "DEBUG - Trusted Source v2:");
     * sp_messagex(sp, LOG_ERR, "DEBUG -   enabled:   %d", g_trustedsourcev2_enabled);
     * sp_messagex(sp, LOG_ERR, "DEBUG -   server:    %s", g_trustedsource_server);
     * sp_messagex(sp, LOG_ERR, "DEBUG -   threshold: %d", g_threshold);
     */

    if (!g_trustedsourcev2_enabled)
    {
        return 1;
    }

#ifdef CONFIG_USER_TRUSTEDSOURCE_BLACKLIST
    if (g_trustedsource_bwlist && (bwlist = check_bwlist_access(white_listv2, black_listv2, my_ip))) {
	return bwlist;
    } /* else: do the ts v1 or 2 lookup as normal */
#endif
    rn = repnet_init(3, 5, g_trustedsourcev2_server, NULL);
    if (!rn) {
        sp_messagex(sp, LOG_ERR, "error initialising Trusted Source v2");
        return 1;
    }
    rq = repquery_init(rn);
    if(!rq) {
	repnet_destroy(rn);
        sp_messagex(sp, LOG_ERR, "error creating Trusted Source v2 query");
	return 1;
    }
    repquery_set_serial(rq, "clamsmtp");
    repquery_newquery(rq);
    repquery_newchunk_meta(rq);
    repquery_newchunk(rq, C_IM);
    repquery_adddeviceserial(rq, g_serial);
    repquery_addip(rq, peeraddr->s.in.sin_addr.s_addr);
    
    if (!repquery_doquery(rq, &rr))
    {
	repnet_destroy(rn);
	repquery_destroy(rq);
        sp_message(sp, LOG_ERR, "error invalid or no response from Trusted Source v2 server");
        return 1;
    }

    repnet_destroy(rn);
    repquery_destroy(rq);

    if (rr.has_error) {
        sp_messagex(sp, LOG_ERR, "error querying Trusted Source v2 server, code: %d", rr.error);
        return 1;
    } else if (rr.has_pr_ip) {
        /* DEBUG sp_message(sp, LOG_ERR, "IP Trusted Source v2 reputation: %d", VALUE_GET(resp->pr_ip)); */

        if (rr.pr_ip >= g_thresholdv2) {
            sp_messagex(sp, LOG_ERR, "client connection from %s is untrusted (score: %d is above threshold of %d)", inet_ntoa(peeraddr->s.in.sin_addr), rr.pr_ip, g_thresholdv2);
#ifdef CONFIG_PROP_STATSD_STATSD
			system("statsd -a incr trustedsource spam \\;"
				            " incr trustedsource total");
#endif
            return -1;
        } else {
            sp_messagex(sp, LOG_DEBUG, "client connection from %s is acceptable (score: %d is below threshold of %d)\n", 
                        inet_ntoa(peeraddr->s.in.sin_addr), rr.pr_ip, g_thresholdv2);
#ifdef CONFIG_PROP_STATSD_STATSD
			system("statsd -a incr trustedsource total");
#endif
            return 1;
        }
    } else {
            sp_messagex(sp, LOG_DEBUG, "client connection from %s is unknown\n", 
                        inet_ntoa(peeraddr->s.in.sin_addr));
            return 1;
    }
#endif

#if !(defined(CONFIG_USER_TRUSTEDSOURCE_V2))
    return 1;
#endif
}


/* ----------------------------------------------------------------------------------
 *  CLAM AV
 */

static int connect_clam(clctx_t* ctx)
{
    int ret = 0;
    spctx_t* sp = &(ctx->sp);

    ASSERT(ctx);
    ASSERT(!spio_valid(&(ctx->clam)));

    if(spio_connect(sp, &(ctx->clam), NULL, &(g_clstate.clamaddr), g_clstate.clamname) == -1)
       RETURN(-1);
    
    spio_read_junk(sp, &(ctx->clam));

#ifdef USE_CLAM_SESSION
    /* Send a session and a check header to ClamAV */
    if(spio_write_data(sp, &(ctx->clam), "SESSION\n") == -1)
        RETURN(-1);
        
    spio_read_junk(sp, &(ctx->clam));

/*  
    if(spio_write_data(sp, &(ctx->clam), "PING\n") == -1 ||
       spio_read_line(sp, &(ctx->clam), CLIO_DISCARD | CLIO_TRIM) == -1)
        RETURN(-1);

    if(strcmp(sp->line, CONNECT_RESPONSE) != 0)
    {
        sp_message(sp, LOG_ERR, "clamd sent an unexpected response: %s", ctx->line);
        RETURN(-1);
    }
*/
#endif

cleanup:

    if(ret < 0 && spio_valid(&(ctx->clam)))
        spio_disconnect(sp, &(ctx->clam));

    return ret;
}

static int disconnect_clam(clctx_t* ctx)
{
    spctx_t* sp = &(ctx->sp);
    
    if(!spio_valid(&(ctx->clam)))
        return 0;
        
#ifdef USE_CLAM_SESSION        
    if(spio_write_data(sp, &(ctx->clam), CLAM_DISCONNECT) != -1)
        spio_read_junk(sp, &(ctx->clam));
#endif

    spio_disconnect(sp, &(ctx->clam));
    return 0;
}

static int clam_scan_file(clctx_t* ctx, const char** virus)
{
    int len, x;
    int ret = 0;
    char* line;
    spctx_t* sp = &(ctx->sp);
    
    /* Connect to clamav */
    if(!spio_valid(&(ctx->clam)))
    {
        if(connect_clam(ctx) == -1)
            RETURN(-1);
    }
    
    ASSERT(ctx && virus);
        
    *virus = NULL;
    
    /* Needs to be long enough to hold path names */
    ASSERT(SP_LINE_LENGTH > MAXPATHLEN + 32);

    line = ctx->clam.line;
    strcpy(line, CLAM_SCAN);
    strcat(line, sp->cachename);
    strcat(line, "\n");
    
    if(spio_write_data(sp, &(ctx->clam), line) == -1)
        RETURN(-1);

    len = spio_read_line(sp, &(ctx->clam), SPIO_DISCARD | SPIO_TRIM);
    if(len == 0)
    {
        sp_messagex(sp, LOG_ERR, "clamd disconnected unexpectedly");
        RETURN(-1);
    }
    
    if(is_last_word(line, CLAM_OK, KL(CLAM_OK)))
    {
        sp_add_log(sp, "status=", "CLEAN");
        sp_messagex(sp, LOG_DEBUG, "no virus");
        RETURN(0);
    }
        
    /*
     * When a virus is found the returned line from 
     * clamd looks something like this:
     * 
     * /path/to/virus: Virus.XXXX FOUND
     */
    if(is_last_word(line, CLAM_FOUND, KL(CLAM_FOUND)))
    {
        x = strlen(sp->cachename);
        
        /* A little sanity check ... */
        if(len > x + KL(CLAM_FOUND))
        {
            /* Remove the "FOUND" from the end */
            line[len - KL(CLAM_FOUND)] = 0;
            
            /* Skip the filename returned, and colon */
            line += x + 1;
            
            line = trim_space(line);

            sp_messagex(sp, LOG_DEBUG, "found virus: %s", line);
            sp_add_log(sp, "status=VIRUS:", line);
            *virus = line;
        }
        
        else
        {
            sp_messagex(sp, LOG_WARNING, "couldn't parse virus name from clamd response: %s", line);
            sp_add_log(sp, "status=", "VIRUS");
            *virus = "Unparsable.Virus.Name";
        }

        RETURN(1);
    }
            
    if(is_last_word(line, CLAM_ERROR, KL(CLAM_ERROR)))
    {
        sp_messagex(sp, LOG_ERR, "clamav error: %s", line);
        sp_add_log(sp, "status=", "CLAMAV-ERROR");
        RETURN(-1);
    }
    
    sp_add_log(sp, "status=", "CLAMAV-ERROR");
    sp_messagex(sp, LOG_ERR, "unexepected response from clamd: %s", line);
    RETURN(-1);
    
cleanup:
#ifndef USE_CLAM_SESSION
    disconnect_clam(ctx);
#endif

    return ret;
}

/* ----------------------------------------------------------------------------------
 *  TEMP FILE HANDLING
 */

static int virus_action(clctx_t* ctx, const char* virus)
{
    char qfilename[MAXPATHLEN];
    spctx_t* sp = &(ctx->sp);
    char* t;
    int i;
    pid_t pid;
               
    if(g_clstate.quarantine)
    {
        strlcpy(qfilename, g_clstate.directory, MAXPATHLEN);
        strlcat(qfilename, "/virus.", MAXPATHLEN);
    
        /* Points to null terminator */
        t = qfilename + strlen(qfilename);
        
        /* 
         * Yes, I know we're using mktemp. And yet we're doing it in
         * a safe manner due to the link command below not overwriting
         * existing files.
         */
        for(;;)
        {
            /* Null terminate off the ending, and replace with X's for mktemp */
            *t = 0;
            strlcat(qfilename, "XXXXXX", MAXPATHLEN);
            
            if(!mkstemp(qfilename))
            {
                sp_message(sp, LOG_ERR, "couldn't create quarantine file name");
                return -1;
            }
            
            /* Try to link the file over to the temp */
            if(link(sp->cachename, qfilename) == -1)
            {
                /* We don't want to allow race conditions */
                if(errno == EEXIST)
                {
                    sp_message(sp, LOG_WARNING, "race condition when quarantining virus file: %s", qfilename);
                    continue;
                }
                    
                sp_message(sp, LOG_ERR, "couldn't quarantine virus file");
                return -1;
            }
            
            break;
        }
        
        sp_messagex(sp, LOG_INFO, "quarantined virus file as: %s", qfilename);
    }
    
    if(g_clstate.virusaction != NULL)
    {
        /* Cleanup any old actions */
        while(waitpid(-1, &i, WNOHANG) > 0)
            ;
            
        sp_messagex(sp, LOG_DEBUG, "executing virus action: %s", g_clstate.virusaction);

        switch(pid = fork())
        {
        case -1:
            sp_message(sp, LOG_ERR, "couldn't fork for virus action");
            return -1;
       
        /* The child */
        case 0:
            /* Close std descriptors */
            for(i = 0; i <= 2; i++)
                close(i);

            /* Set the environment variables */
            sp_setup_forked(sp, 0);

            /* When quarantining we can hand the file name off */
            if(g_clstate.quarantine)
                setenv("EMAIL", qfilename, 1);
                
            if(virus)
                setenv("VIRUS", virus, 1);
                
            /* And execute the program */
            execl("/bin/sh", "sh", "-c", g_clstate.virusaction, NULL);            
            
            /* If that returned then there was an error, but there's
             * not much we can do about it. */
            _exit(1);
            break;
        };
    }
    
    return 0;
}


