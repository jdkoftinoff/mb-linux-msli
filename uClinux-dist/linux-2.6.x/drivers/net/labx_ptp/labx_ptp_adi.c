/*
 *  labx_ptp_main_adi.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
 *
 *  Written by Chris Pane(chris.pane@labxtechnologies.com)
 *
 *  Copyright (C) 2011 Lab X Technologies LLC, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

 #include <stdio.h>
 #include <linux/types.h>
 #include <linux/platform_device.h>
 #include <labx_ptp.h>
 #include "PTP_TSYNC.h"
 #include "vdk.h"
 #include "ptp_access_vdk.h"

extern float g_TickPeriodC;

static uint8_t ptp_tx_announce_buf       [1500];
static uint8_t ptp_tx_sync_buf           [1500];
static uint8_t ptp_tx_fup_buf            [1500];
static uint8_t ptp_tx_delay_req_buf      [1500];
static uint8_t ptp_tx_delay_resp_buf     [1500];
static uint8_t ptp_tx_pdelay_req_buf     [1500];
static uint8_t ptp_tx_pdelay_resp_buf    [1500];
static uint8_t ptp_tx_pdelay_resp_fup_buf[1500];
 

 
int write_ethernet_frame(int interface, uint8_t *buf, uint16_t size);
            
 /* Interrupt service routine for the instance */
static irqreturn_t labx_ptp_interrupt(int irq, void *dev_id)
{
  return(IRQ_HANDLED);
}

void ptp_enable_port(struct ptp_device *ptp,int port)
{
	
}

void ptp_disable_port(struct ptp_device *ptp,int port)
{
	
}

void ptp_disable_irqs(struct ptp_device *ptp, int port)
{
}

void ptp_enable_irqs(struct ptp_device *ptp, int port)
{
}


uint32_t ptp_get_version(struct ptp_device *ptp)
{
    return 0x00000012;
}


void ptp_setup_event_timer(struct ptp_device *ptp,int port)
{
}

uint32_t ptp_setup_interrupt(struct ptp_device *ptp)
{
    return 0;
}

void write_packet(uint8_t *bufferBase, uint32_t *wordOffset,uint32_t writeWord) 
{
    if (*wordOffset == 0)
    {
        /* Stuff size in little endian */
        bufferBase[2] = writeWord & 0xFF;
        bufferBase[3] = (writeWord >> 8)& 0xFF;
        *wordOffset+=4;
    }
    else
    {
        bufferBase[(*wordOffset)++] = (uint8_t)(writeWord >> 24);
        bufferBase[(*wordOffset)++] = (uint8_t)(writeWord >> 16);
        bufferBase[(*wordOffset)++] = (uint8_t)(writeWord >> 8);
        bufferBase[(*wordOffset)++] = (uint8_t)(writeWord >> 0);
    }
    
}

/* Reads a word from a packet buffer at the passed offset.  The offset is
 * advanced to the next word.
 */
uint32_t read_packet(uint8_t *bufferBase, uint32_t *wordOffset) 
{
  uint32_t readWord=0;

  readWord = (bufferBase[(*wordOffset)++] << 24) | 
             (bufferBase[(*wordOffset)++] << 16) |
             (bufferBase[(*wordOffset)++] <<  8) |
             (bufferBase[(*wordOffset)++]);
  return(readWord);
}

platform_driver_unregister(struct platform_driver *driver)
{
    /* Do nothing*/
}


uint32_t ptp_get_increment()
{
    /* TODO - Note: This supports debug , not critical to correct PTP operation  */
    return 0;
}

static void get_seconds_upper(struct ptp_device *ptp, uint32_t port, PtpTime *timestamp)
{
	if ((ptp->ports[port].syncTimestampsValid) && (ptp->presentRole != PTP_MASTER))
	{
	    timestamp->secondsUpper = ptp->ports[port].syncTxTimestamp.secondsUpper;
	    
	    if ((ptp->ports[port].syncTxTimestamp.secondsLower < 0x00001000) &&
	        (timestamp->secondsLower > 0xFFFFF000))
	    {
	    	// The timestamp hasn't rolled over seconds, but the master has.
	    	timestamp->secondsUpper--;
	    }
	    else if ((ptp->ports[port].syncTxTimestamp.secondsLower > 0xFFFFF000) &&
	             (timestamp->secondsLower < 0x00001000))
	    {
	    	// The timestamp has rolled over seconds, but the master hasn't.
	    	timestamp->secondsUpper++;
	    }
	}
	else
	{
		// If we are master or have no valid sync timestamps from the master, just go with zero
		timestamp->secondsUpper = 0;
	}
}

/* Gets the hardware timestamp located within the passed packet buffer.
 * It is up to the client code to ascertain that a valid timestamp exists; that is,
 * a packet has either been transmitted from or received into the buffer.
 */
void get_hardware_timestamp(struct ptp_device *ptp, 
                            uint32_t port, 
                            PacketDirection bufferDirection,
                            uint8_t * packetBuffer, 
                            PtpTime *timestamp) {

    uint32_t seconds = 0;
    uint32_t nanoseconds = 0;
    uint32_t overflowCount = 0;
    uint32_t messageType = 0;
    uint32_t sourceId;
    uint32_t *timestampArea;
//    uint32_t localSeconds = 0;
//    uint32_t localNanoseconds = 0;

    unsigned short length;

        if (bufferDirection == TRANSMITTED_PACKET)
    {
    	length = *((unsigned short *)(packetBuffer+2))+4;
    }
    else
    {
    	length = *((unsigned short *)(packetBuffer-2))-4;
    }
    timestampArea = (uint32_t *)((((uint32_t)packetBuffer)+length+3) & ~3);

//    localSeconds     = timestampArea[0];
//    localNanoseconds = timestampArea[1];
    seconds          = timestampArea[2];
    nanoseconds      = timestampArea[3];

  //  printf("get_hardware_timestamp: localSeconds: %08X localNanoseconds: %08X seconds: %08X nanoseconds %08X\n",
  //         localSeconds, localNanoseconds, seconds, nanoseconds);

    timestamp->secondsLower = seconds;
    timestamp->nanoseconds = nanoseconds;
    get_seconds_upper(ptp,port,timestamp);
}

/* Gets the local hardware timestamp located within the passed packet buffer.
 * This is the same as get_hardware_timestamp except it is from a local clock
 * that is running at a fixed rate unmodified by PTP.
 */
void get_local_hardware_timestamp(struct ptp_device *ptp, 
                                  uint32_t port, 
                                  PacketDirection bufferDirection,
                                  uint8_t *packetBuffer, 
                                  PtpTime *timestamp) 
{
    unsigned short length;

    if (bufferDirection == TRANSMITTED_PACKET)
    {
    	length = *((unsigned short *)(packetBuffer+2))+4;
    }
    else
    {
    	length   = *((unsigned short *)(packetBuffer-2))-4;
    }
    uint32_t *timestampArea = (uint32_t *)((((uint32_t)packetBuffer)+length+3) & ~3);
    uint32_t localSeconds     = timestampArea[0];
    uint32_t localNanoseconds = timestampArea[1];

    timestamp->secondsLower = localSeconds;
    timestamp->nanoseconds  = localNanoseconds;
    timestamp->secondsUpper = 0;

}
void transmit_packet(struct ptp_device *ptp, uint32_t port, uint8_t * txBuffer) 
{
    uint16_t size;
    size = *(uint16_t *)&txBuffer[2];
    /* We add 1 to size since PTP code assumes size-1. See original
       FPGA implementation this code was ported from for more details. */
       
    // Claim the transmit/aux timestamping registers for this transmitted packet.
    // The lock will be released when we get the timestamp.
    lockTx(NULL);
    
    write_ethernet_frame(port,&txBuffer[2],size+1);
}

uint8_t * get_output_buffer(struct ptp_device *ptp,uint32_t port,uint32_t bufType)
{
    uint8_t *buf = NULL;

    switch (bufType)
    {
        case PTP_TX_ANNOUNCE_BUFFER:
            buf = ptp_tx_announce_buf;
            break;
        case PTP_TX_SYNC_BUFFER:             
            buf = ptp_tx_sync_buf;
            break;
        case PTP_TX_FUP_BUFFER:              
            buf =  ptp_tx_fup_buf;
            break;
        case PTP_TX_DELAY_REQ_BUFFER:        
            buf =  ptp_tx_delay_req_buf;
            break;
        case PTP_TX_DELAY_RESP_BUFFER:       
            buf =  ptp_tx_delay_resp_buf;
            break;
        case PTP_TX_PDELAY_REQ_BUFFER:       
            buf =  ptp_tx_pdelay_req_buf;
            break;
        case PTP_TX_PDELAY_RESP_BUFFER:      
            buf =  ptp_tx_pdelay_resp_buf;
            break;
        case PTP_TX_PDELAY_RESP_FUP_BUFFER:  
            buf =  ptp_tx_pdelay_resp_fup_buf;
            break;
        default:
            buf =  NULL;
    }
    return buf;
}
/* Disables the RTC */
void disable_rtc(struct ptp_device *ptp) 
{
    printf("disable_rtc: TODO - Implement \n");
}

/* Sets the RTC increment, simultaneously enabling the RTC */
void set_rtc_increment(struct ptp_device *ptp, RtcIncrement *increment) 
{
    unsigned long long adj = (increment->mantissa << 27) | (increment->fraction);
    unsigned long long nominal = (ptp->nominalIncrement.mantissa << 27) | (ptp->nominalIncrement.fraction);
    unsigned long long fraction = ((adj << 32) / nominal) >> 1;
    adjtimex(fraction);
    //printf("set_rtc_increment: mantissa %08X, fraction %08X, Adjustment: %08X\n",increment->mantissa, increment->fraction, fraction);
}

/* Captures the present RTC time, returning it into the passed structure */
void get_rtc_time(struct ptp_device *ptp, PtpTime *time) 
{
    TimeInternal tsTime;
    gettimeofday_BF(&tsTime);

    time->nanoseconds  = tsTime.nanoseconds;
    time->secondsLower = tsTime.seconds;
    /** TODO - We need to paramaterize port, for now hard code to
     *  0 since ADI only has 1 */
    get_seconds_upper(ptp,0,time);
}

/* Sets a new RTC time from the passed structure */
void set_rtc_time(struct ptp_device *ptp, PtpTime *time) {

    TimeInternal newTime;

    newTime.nanoseconds = time->nanoseconds;
    newTime.seconds     = time->secondsLower;
    settimeofday_BF(&newTime);
}

/* Sets a new RTC time from the passed structure */
void set_rtc_time_adjusted(struct ptp_device *ptp, PtpTime *time, PtpTime *entryTime) {

    TimeInternal newTime;
    TimeInternal newEntryTime;

    newTime.nanoseconds = time->nanoseconds;
    newTime.seconds     = time->secondsLower;

    newEntryTime.nanoseconds = entryTime->nanoseconds;
    newEntryTime.seconds     = entryTime->secondsLower;
    
    settimeofday_adjusted_BF(&newTime, &newEntryTime);
}

void get_local_time(struct ptp_device *ptp, PtpTime *timestamp) 
{	
    int flags;
    unsigned int currentTicks;
    uint32_t tcount1;
    uint32_t ipend;
    uint32_t tcount2;
    uint32_t tperiod;

    flags = cli();

    /* Get Local Time in ticks */
    currentTicks = VDK_GetUptime();
    tcount1 = *pTCOUNT;
    ipend   = *pIPEND;
    tcount2 = *pTCOUNT;
    tperiod = *pTPERIOD;
    
    sti(flags);
    
    /* Core timer counts down. If the timer has rolled over we need to
       increment the current ticks before adding the count */
    if (tcount1 >= tcount2)
    {
    	if (ipend & EVT_IVTMR)
    	{
    		/* Rollover was before the read of tcount1 */
    		currentTicks++;
    	}
    	/* Else, there was no rollover */
    }
    /* Else, the rollover happend after the read of tcount1 */
    
    uint64_t localTime = (uint64_t)currentTicks*(uint64_t)(g_TickPeriodC*1000000);
    localTime += ((tperiod - tcount1) * 25)/10; // TODO: This assumes a 400MHz clock (2.5ns)!
		
    timestamp->secondsLower = localTime / GIGA_MAGNITUDE;
    timestamp->nanoseconds  = localTime % GIGA_MAGNITUDE;
    timestamp->secondsUpper = 0;
}

int ptp_events_tx_heartbeat(struct ptp_device *ptp) {
  ptp->eventFlags |= EVENT_FLAG_HEARTBEAT;  
  return 0;
}

int ptp_events_tx_gm_change(struct ptp_device *ptp) {
//  ptp->eventFlags |= EVENT_FLAG_GM_CHANGE;
  ack_grandmaster_change(ptp);

  return 0;
}

int ptp_events_tx_rtc_change(struct ptp_device *ptp) {
  ptp->eventFlags |= EVENT_FLAG_RTC_CHANGE;
  return 0;
}

int register_ptp_netlink(void) {
  return 0;
}

void unregister_ptp_netlink(void) {
}
