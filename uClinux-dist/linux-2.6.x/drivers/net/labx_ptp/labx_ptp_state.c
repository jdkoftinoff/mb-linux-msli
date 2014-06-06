/*
 *  linux/drivers/net/labx_ptp_state.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
 *  PTP state machine processing
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2009 Lab X Technologies LLC, All Rights Reserved.
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

#include "labx_ptp.h"
#include <linux/of_platform.h>

/* Define these to get some extra debug on sync/follow-up messages */
/* #define SYNC_DEBUG */
/* #define DEBUG_INCREMENT */

/* Parameters governing message rates, etc. measuring time in msec. */
#define HEARTBEAT_INTERVAL        (5000)
#define DELAY_REQ_INTERVAL        (1000)

/* Maximum error, in nanoseconds, tolerated before the time is reset */
#define RESET_THRESHOLD_NS  (100000)

/* Function Prototypes */
uint8_t * get_output_buffer(struct ptp_device *ptp,uint32_t port,uint32_t bufType);
void tasklet_init(struct tasklet_struct *t, void (*func)(unsigned long),unsigned long data);
void ptp_platform_init(struct ptp_device *ptp, int port);

/* Tasklet function for responding to timer interrupts */
void labx_ptp_timer_state_task(unsigned long data) {
  struct ptp_device *ptp = (struct ptp_device*) data;
  unsigned long flags;
  uint32_t newMaster;
  int i;
  int8_t reselect = 0;
  bool localMaster = true;

  uint32_t timerTicks = 0;

  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  timerTicks = ptp->timerTicks;
  ptp->timerTicks = 0;
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();

  /* Update port roles whenever any port is flagged for reselect */
  for (i=0; i<ptp->numPorts; i++) {
    reselect |= ptp->ports[i].reselect;
  }
  if (reselect) {
    PortRoleSelection_StateMachine(ptp);
  }

  for (i=0; i<ptp->numPorts; i++) {
    if (ptp->ports[i].selectedRole == PTP_SLAVE) {
      localMaster = false;
      break;
    }
  }

  for (i=0; i<ptp->numPorts; i++) {
    switch(ptp->ports[i].selectedRole) {
    case PTP_MASTER:
      /* Send ANNOUNCE and SYNC messages at their rate for a master port */
      ptp->ports[i].announceCounter += timerTicks;
      if(ptp->ports[i].announceCounter >= ANNOUNCE_INTERVAL_TICKS(ptp, i)) {
        ptp->ports[i].announceCounter = 0;
        ptp->ports[i].newInfo = FALSE;
        transmit_announce(ptp, i);
      }

      ptp->ports[i].syncCounter += timerTicks;
      if(ptp->ports[i].syncCounter >= SYNC_INTERVAL_TICKS(ptp, i)) {
        ptp->ports[i].syncCounter = 0;

        /* If we are the grandmaster send sync messages as long as it is after an announce. If we are not,
           we will forward sync/fup messages when we receive them from the GM. */
        if (localMaster && ptp->ports[i].firstAnnounceSent==TRUE) {
          /* Set the source port ID back to this node when we are the GM */
          memcpy(&ptp->ports[i].syncSourcePortId[0], &ptp->properties.grandmasterIdentity[0], 8);
          ptp->ports[i].syncSourcePortId[8] = (i+1) >> 8;
          ptp->ports[i].syncSourcePortId[9] = (i+1);

          transmit_sync(ptp, i);
          if (ptp->rtcChangesAllowed) {
            /* Periodically update the RTC to get update listeners to
               notice (IE when they are not coasting) */
            set_rtc_increment(ptp, &ptp->nominalIncrement);
          }
        }
      }
      break;

    case PTP_SLAVE:
    {
#ifdef DEBUG_INCREMENT
      uint32_t timeoutTicks = 8;
#endif

      /* Increment and test the announce receipt timeout counter */
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      ptp->ports[i].announceTimeoutCounter += timerTicks;
      ptp->ports[i].syncTimeoutCounter += timerTicks;
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();

#ifdef DEBUG_INCREMENT
      /* Periodically print out the increment we're using */
      if(++ptp->slaveDebugCounter >= timeoutTicks) {
        ptp->slaveDebugCounter = 0;

        printk("PTP increment: 0x%08X\n",ptp_get_increment());
      }
#endif

      /* Transmit an ANNOUNCE immediately to speed things along if we've switched our
       * port to the master state.
       */
      if(ptp->ports[i].selectedRole == PTP_MASTER) {
        printk("PTP master (port %d)\n", i);
        for (i=0; i<ptp->numPorts; i++) {
          ptp->ports[i].announceCounter    = 0;
          ptp->ports[i].announceSequenceId = 0x0000;
          transmit_announce(ptp, i);
        }
      } else {
        /* Still a slave; determine whether we are using the end-to-end or peer-to-peer
         * delay mechanism
         */
        if(ptp->properties.delayMechanism == PTP_DELAY_MECHANISM_E2E) {
          /* Increment the delay request counter and see if it's time to
           * send one to the master.
           */
          if(++ptp->ports[i].delayReqCounter >= (DELAY_REQ_INTERVAL / PTP_TIMER_TICK_MS)) {
            ptp->ports[i].delayReqCounter = 0;
            transmit_delay_request(ptp, i);
          }
        }
      } /* if(still a slave) */
    }
    break;

    default:
      /* "Passive"; do nothing */
      break;
    }
  }

  if (ptp->gmPriority == &ptp->systemPriority) {
    /* Always flag the RTC offset as valid, and zero since we're the master */
    preempt_disable();
    spin_lock_irqsave(&ptp->mutex, flags);
    ptp->rtcLastOffsetValid    = PTP_RTC_OFFSET_VALID;
    ptp->rtcLastOffset         = 0;
    ptp->rtcLastIncrementDelta = 0;
    spin_unlock_irqrestore(&ptp->mutex, flags);
    preempt_enable();
  }

  for (i=0; i<ptp->numPorts; i++)
  {
    LinkDelaySyncIntervalSetting_StateMachine(ptp, i);

    /* Regardless of whether we are a master or slave, increment the peer delay request
     * counter and see if it's time to send one to our link peer.
     */
    ptp->ports[i].pdelayIntervalTimer += timerTicks;
    ptp->ports[i].mdPdelayReq_LastTime += timerTicks;

    MDPdelayReq_StateMachine(ptp, i);

    /* Update the PortAnnounceInformation state machine */
    PortAnnounceInformation_StateMachine(ptp, i);

    /* Track consecutive multiple pdelay responses for AVnu_PTP-5 PICS,
       re-enable the port after five minutes */
    if(ptp->ports[i].multiplePdelayTimer > 0) {
      if(ptp->ports[i].multiplePdelayTimer >= timerTicks) {
        ptp->ports[i].multiplePdelayTimer -= timerTicks;
      }
      else {
        ptp->ports[i].multiplePdelayTimer = 0;
        ptp->ports[i].portEnabled = TRUE;
        printk("Re-enabled ptp on port %d after 5 minutes\n", i+1);
      }
    }
  }

  /* Test to see if the master is new from the last time we checked; if so,
   * propagate a Netlink message reporting the new Grandmaster
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  newMaster      = ptp->newMaster;
  ptp->newMaster = FALSE;
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();

  if(newMaster) ptp_events_tx_gm_change(ptp);

  /* Also regardless of mode, send a Generic Netlink message periodically to
   * serve as a "heartbeat" for other interested drivers and userspace daemons
   * to monitor.
   */
  if(++ptp->heartbeatCounter >= (HEARTBEAT_INTERVAL / PTP_TIMER_TICK_MS)) {
    /* Reset the counter and send a Netlink event */
    ptp->heartbeatCounter = 0;
    ptp_events_tx_heartbeat(ptp);
  }

  /* Update the RTC lock detection state, and send a Netlink message if
   * there is a change in state.
   */
  update_rtc_lock_detect(ptp);
  if(ptp->rtcLastLockState != ptp->rtcLockState) ptp_events_tx_rtc_change(ptp);
  ptp->rtcLastLockState = ptp->rtcLockState;
}

/* Processes a newly-received ANNOUNCE packet for the passed instance */
static void process_rx_announce(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {

  ptp->ports[port].stats.rxAnnounceCount++;

  ptp->ports[port].rcvdAnnouncePtr = rxBuffer;

  PortAnnounceReceive_StateMachine(ptp, port);
}

/* Processes a newly-received SYNC packet for the passed instance */
static void process_rx_sync(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {
  unsigned long flags;
  PtpPortIdentity rxIdentity;
  int i;
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
  PtpTime diff;
  PtpTime timeTest;
  switch_timestamp_t *switch_t1;
  switch_timestamp_t *switch_t2;
  switch_timestamp_t switch_t1a,switch_t2a;
  switch_timestamp_t switch_t1b,switch_t2b;
  int32_t t1,t2;
  int cnt1,cnt2;
  uint16_t sequence_id;
#endif

  ptp->ports[port].stats.rxSyncCount++;

  /* Only process this packet if we are a slave and it has come from the master
   * we're presently respecting.  If we're the master, spanning tree should prevent
   * us from ever seeing our own SYNC packets, but better safe than sorry.
   */
  get_source_port_id(ptp, port, RECEIVED_PACKET, rxBuffer, (uint8_t*)&rxIdentity);
  if((ptp->ports[port].selectedRole == PTP_SLAVE) &&
     (0 == memcmp(&rxIdentity, &ptp->gmPriority->sourcePortIdentity, sizeof(PtpPortIdentity)))) {
    PtpTime tempTimestamp;
    PtpTime correctionField;
    PtpTime correctedTimestamp;

    ptp->ports[port].syncReceiptTimeoutTime = SYNC_INTERVAL_TICKS(ptp, port) * ptp->ports[port].syncReceiptTimeout;

    /* This is indeed a SYNC from the present master.  Capture the hardware timestamp
     * at which we received it, and hang on to its sequence ID for matching to the
     * followup that should follow.
     */
    get_hardware_timestamp(ptp, port, RECEIVED_PACKET, rxBuffer, &tempTimestamp);

    get_correction_field(ptp, port, rxBuffer, &correctionField);
    timestamp_difference(&tempTimestamp, &correctionField, &correctedTimestamp);

    preempt_disable();
    spin_lock_irqsave(&ptp->mutex, flags);
    timestamp_copy(&ptp->ports[port].syncRxTimestampTemp, &correctedTimestamp);
#ifdef DEBUG_MISSED_TIMESTAMP
    if(((ptp->ports[port].syncSequenceId+1)&0xffff) != get_sequence_id(ptp, port, RECEIVED_PACKET, rxBuffer)) {
        printk("missed sync %d (%d)\r\n",ptp->ports[port].syncSequenceId,get_sequence_id(ptp, port, RECEIVED_PACKET, rxBuffer)-(ptp->ports[port].syncSequenceId+1));
    }
#endif
    ptp->ports[port].syncSequenceId = get_sequence_id(ptp, port, RECEIVED_PACKET, rxBuffer);
    ptp->ports[port].syncSequenceIdValid = 1;
    spin_unlock_irqrestore(&ptp->mutex, flags);
    preempt_enable();

#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
      {
        sequence_id=ptp->ports[port].syncSequenceId;
        cnt1=0;
        cnt2=0;
        if(port==0) {
          switch_timestamp(TIMESTAMP_AVB2_INCOMMING_SYNC,&switch_t1, &switch_t2,sequence_id);
          if(switch_t1==NULL) {
              do {
                block_read_avb_ptp(&switch_t1a,0,0x0c);
                block_read_avb_ptp(&switch_t1b,0,0x0c);
                cnt1++;
              } while(
                    (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
                    (switch_t1a.low!=switch_t1b.low)||
                    (switch_t1a.high!=switch_t1b.high));
          } else {
                memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
          }
          if(switch_t2==NULL) {
              do {
                block_read_avb_ptp(&switch_t2a,5,0x10);
                block_read_avb_ptp(&switch_t2b,5,0x10);
                cnt2++;
              } while(
                    (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
                    (switch_t2a.low!=switch_t2b.low)||
                    (switch_t2a.high!=switch_t2b.high));
          } else {
              memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
          }
        } else {
          switch_timestamp(TIMESTAMP_AVB1_INCOMMING_SYNC,&switch_t1, &switch_t2,sequence_id);
          if(switch_t1==NULL) {
              do {
                block_read_avb_ptp(&switch_t1a,1,0x0c);
                block_read_avb_ptp(&switch_t1b,1,0x0c);
                cnt1++;
              } while(
                    (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
                    (switch_t1a.low!=switch_t1b.low)||
                    (switch_t1a.high!=switch_t1b.high));
          } else {
              memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
          }
          if(switch_t2==NULL) {
              do {
                block_read_avb_ptp(&switch_t2a,6,0x10);
                block_read_avb_ptp(&switch_t2b,6,0x10);
                cnt2++;
              } while(
                    (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
                    (switch_t2a.low!=switch_t2b.low)||
                    (switch_t2a.high!=switch_t2b.high));
          } else {
              memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
          }
        }
        if((cnt1!=0)||(cnt2!=0)) {
            DEBUG_TIMESTAMP_PRINTF("E retry %d,%d\r\n",cnt1,cnt2);
        }
        if((sequence_id==switch_t1a.sequence_id) &&
           (sequence_id==switch_t2a.sequence_id)) {
          t1=(switch_t1a.high<<16)|switch_t1a.low;
          t2=(switch_t2a.high<<16)|switch_t2a.low;
          diff.secondsUpper=0;
          diff.secondsLower=((uint32_t)t2*8ULL)/1000000000ULL;
          diff.nanoseconds=((uint32_t)t2*8ULL)%1000000000ULL;
          timestamp_difference(&ptp->ports[port].syncRxTimestampTemp,&diff,&ptp->switchDelta2);
          ptp->t2_prev=t2;

          diff.secondsUpper=0;
          diff.secondsLower=0;
          diff.nanoseconds=(t2-t1)*8;
          if((uint32_t)t2<(uint32_t)t1) {
            WARN_TIMESTAMP_PRINTF("E t2 wrapped %04x\r\n",diff.nanoseconds); 
          }
          timestamp_difference(&ptp->ports[port].syncRxTimestampTemp,&diff,&timeTest);
          if((uint32_t)t2<(uint32_t)t1) {
            WARN_TIMESTAMP_PRINTF("E t1 wrapped around\r\n");
            diff.secondsUpper=0;
            diff.secondsLower=34;
            diff.nanoseconds=359738368;
            timestamp_difference(&ptp->switchDelta2,&diff,&ptp->switchDelta2);
            ptp->t2_prev=t1;
          }
          diff.secondsUpper=0;
          diff.secondsLower=((uint32_t)t1*8ULL)/1000000000ULL;
          diff.nanoseconds=((uint32_t)t1*8ULL)%1000000000ULL;
          timestamp_sum(&ptp->switchDelta2,&diff,&ptp->ports[port].syncRxTimestampTemp);

          timestamp_difference(&ptp->ports[port].syncRxTimestampTemp,&timeTest,&diff);
          if((diff.secondsUpper!=0x00000000) ||
             (diff.secondsLower!=0x00000000) ||
             ((diff.nanoseconds&0xffffff00)!=0x00000000)) {
                if((diff.secondsUpper!=0xffffffff) ||
                   (diff.secondsLower!=0xffffffff) ||
                   ((diff.nanoseconds&0xffffff00)!=0xffffff00)) {
                        ERROR_TIMESTAMP_PRINTF("E    !!!     bad math %08x%08x.%08x\r\n",diff.secondsUpper,diff.secondsLower,diff.nanoseconds);
                }
          }
          if(ptp->ports[port].recoveringE==1) {
                DEBUG_TIMESTAMP_PRINTF("E recovered rx sync %08x%08x.%08x\r\n",ptp->ports[port].syncRxTimestampTemp.secondsUpper,ptp->ports[port].syncRxTimestampTemp.secondsLower,ptp->ports[port].syncRxTimestampTemp.nanoseconds);
                ptp->ports[port].recoveringE=0;
          }
        } else if(sequence_id==switch_t1a.sequence_id) {
          t1=(switch_t1a.high<<16)|switch_t1a.low;
          if(((uint32_t)t1<0x70000000) &&
             ((uint32_t)ptp->t2_prev>0x90000000)) {
            WARN_TIMESTAMP_PRINTF("E t1 wrapped around\r\n");
            diff.secondsUpper=0;
            diff.secondsLower=34;
            diff.nanoseconds=359738368;
            timestamp_sum(&ptp->switchDelta2,&diff,&ptp->switchDelta2);
          }
          if(((uint32_t)ptp->t2_prev<0x70000000) &&
             ((uint32_t)t1>0x90000000)) {
            WARN_TIMESTAMP_PRINTF("E t2 wrapped around\r\n");
            diff.secondsUpper=0;
            diff.secondsLower=34;
            diff.nanoseconds=359738368;
            timestamp_difference(&ptp->switchDelta2,&diff,&ptp->switchDelta2);
          }
          diff.secondsUpper=0;
          diff.secondsLower=((uint32_t)t1*8ULL)/1000000000ULL;
          diff.nanoseconds=((uint32_t)t1*8ULL)%1000000000ULL;
          timestamp_sum(&ptp->switchDelta2,&diff,&ptp->ports[port].syncRxTimestampTemp);
          DEBUG_TIMESTAMP_PRINTF("E recovered rx sync %d t1:%08x, t2:%08x, delta:%08x%08x.%08x\r\n",port,(uint32_t)t1,(uint32_t)ptp->t2_prev,ptp->switchDelta.secondsUpper,ptp->switchDelta.secondsLower,ptp->switchDelta.nanoseconds);
          ptp->ports[port].recoveringE=1;
        } else {
          ptp->ports[port].recoveringE=1;
          ptp->ports[port].syncSequenceIdValid = 0;
          DEBUG_TIMESTAMP_PRINTF("E missed rx sync %04x:%04x instead of %04x\r\n",switch_t1a.sequence_id,switch_t2a.sequence_id,sequence_id);
        }
      }
#endif
    /* Forward the sync to any master ports if the sync is coming in on a slave port */
    if (ptp->ports[port].selectedRole == PTP_SLAVE) {
      PtpTime syncRxTimestamp;
      PtpTime linkDelay;
      get_local_hardware_timestamp(ptp, port, RECEIVED_PACKET, rxBuffer, &syncRxTimestamp);
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
      {
        if((sequence_id==switch_t1a.sequence_id) &&
           (sequence_id==switch_t2a.sequence_id)) {
          t1=(switch_t1a.high<<16)|switch_t1a.low;
          t2=(switch_t2a.high<<16)|switch_t2a.low;
          diff.secondsUpper=0;
          diff.secondsLower=((uint32_t)t2*8ULL)/1000000000ULL;
          diff.nanoseconds=((uint32_t)t2*8ULL)%1000000000ULL;
          timestamp_difference(&syncRxTimestamp,&diff,&ptp->switchDelta);
          ptp->t2_prev=t2;

          diff.secondsUpper=0;
          diff.secondsLower=0;
          diff.nanoseconds=(t2-t1)*8;
          if((uint32_t)t2<(uint32_t)t1) {
            WARN_TIMESTAMP_PRINTF("E t2 wrapped %04x\r\n",diff.nanoseconds);
          }
          timestamp_difference(&syncRxTimestamp,&diff,&timeTest);
          if((uint32_t)t2<(uint32_t)t1) {
            WARN_TIMESTAMP_PRINTF("E t1 wrapped around\r\n");
            diff.secondsUpper=0;
            diff.secondsLower=34;
            diff.nanoseconds=359738368;
            timestamp_difference(&ptp->switchDelta,&diff,&ptp->switchDelta);
            ptp->t2_prev=t1;
          }
          diff.secondsUpper=0;
          diff.secondsLower=((uint32_t)t1*8ULL)/1000000000ULL;
          diff.nanoseconds=((uint32_t)t1*8ULL)%1000000000ULL;
          timestamp_sum(&ptp->switchDelta,&diff,&syncRxTimestamp);

          timestamp_difference(&syncRxTimestamp,&timeTest,&diff);
          if((diff.secondsUpper!=0x00000000) ||
             (diff.secondsLower!=0x00000000) ||
             ((diff.nanoseconds&0xffffff00)!=0x00000000)) {
                if((diff.secondsUpper!=0xffffffff) ||
                   (diff.secondsLower!=0xffffffff) ||
                   ((diff.nanoseconds&0xffffff00)!=0xffffff00)) {
                        ERROR_TIMESTAMP_PRINTF("E    !!!     bad math %08x%08x.%08x\r\n",diff.secondsUpper,diff.secondsLower,diff.nanoseconds);
                }
          }
          if(ptp->ports[port].recoveringE==1) {
                DEBUG_TIMESTAMP_PRINTF("E recovered rx sync %08x%08x.%08x\r\n",syncRxTimestamp.secondsUpper,syncRxTimestamp.secondsLower,syncRxTimestamp.nanoseconds);
                ptp->ports[port].recoveringE=0;
          }
        } else if(sequence_id==switch_t1a.sequence_id) {
          t1=(switch_t1a.high<<16)|switch_t1a.low;
          if(((uint32_t)t1<0x70000000) &&
             ((uint32_t)ptp->t2_prev>0x90000000)) {
            WARN_TIMESTAMP_PRINTF("E t1 wrapped around\r\n");
            diff.secondsUpper=0;
            diff.secondsLower=34;
            diff.nanoseconds=359738368;
            timestamp_sum(&ptp->switchDelta,&diff,&ptp->switchDelta);
          }
          if(((uint32_t)ptp->t2_prev<0x70000000) &&
             ((uint32_t)t1>0x90000000)) {
            WARN_TIMESTAMP_PRINTF("E t2 wrapped around\r\n");
            diff.secondsUpper=0;
            diff.secondsLower=34;
            diff.nanoseconds=359738368;
            timestamp_difference(&ptp->switchDelta,&diff,&ptp->switchDelta);
          }
          diff.secondsUpper=0;
          diff.secondsLower=((uint32_t)t1*8ULL)/1000000000ULL;
          diff.nanoseconds=((uint32_t)t1*8ULL)%1000000000ULL;
          timestamp_sum(&ptp->switchDelta,&diff,&syncRxTimestamp);
          DEBUG_TIMESTAMP_PRINTF("E recovered rx sync %d t1:%08x, t2:%08x, delta:%08x%08x.%08x\r\n",port,(uint32_t)t1,(uint32_t)ptp->t2_prev,ptp->switchDelta.secondsUpper,ptp->switchDelta.secondsLower,ptp->switchDelta.nanoseconds);
          ptp->ports[port].recoveringE=1;
        } else {
          ptp->ports[port].recoveringE=1;
          ptp->ports[port].syncSequenceIdValid = 0;
          DEBUG_TIMESTAMP_PRINTF("E missed rx sync %04x:%04x instead of %04x\r\n",switch_t1a.sequence_id,switch_t2a.sequence_id,sequence_id);
        }
      }
#endif

      if(ptp->ports[port].syncSequenceIdValid) {
        linkDelay.secondsUpper = 0;
        linkDelay.secondsLower = 0;
        linkDelay.nanoseconds = ptp->ports[port].neighborPropDelay;
        for (i=0; i<ptp->numPorts; i++) {
          if (ptp->ports[i].selectedRole == PTP_MASTER ) {
            // Save the received time (with link delay) for later calculation of residency time
            timestamp_difference(&syncRxTimestamp, &linkDelay, &ptp->ports[i].syncRxTimestamp);
            get_source_port_id(ptp, port, RECEIVED_PACKET, rxBuffer, &ptp->ports[i].syncSourcePortId[0]);
            ptp->ports[i].syncSequenceId = ptp->ports[port].syncSequenceId;
            /* actually send the sync only if an announce has been sent */
            if( ptp->ports[port].firstAnnounceSent ) {
                transmit_sync(ptp, i);
            }
          }
        }
      }
    }
  }
}


void labx_ptp_signal_gm_change(struct ptp_device *ptp) {

  ptp->newMaster          = TRUE;
  ptp->rtcReset           = TRUE;

  /* Do not permit the RTC to change until userspace permits it, and also
   * reset the lock state
   */
  ptp->acquiring             = PTP_RTC_ACQUIRING;
  ptp->rtcLockState          = PTP_RTC_UNLOCKED;
  ptp->rtcLockCounter        = 0;
  ptp->rtcChangesAllowed     = FALSE;
  ptp->rtcLastOffsetValid    = PTP_RTC_OFFSET_VALID;
  ptp->rtcLastOffset         = 0;
  ptp->rtcLastIncrementDelta = 0;
}

/* Processes a newly-received FUP packet for the passed instance */
static void process_rx_fup(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {
  unsigned long flags;
  PtpPortIdentity rxIdentity;
  int i;

  ptp->ports[port].stats.rxFollowupCount++;

  /* Make certain of the following:
   * - We are a slave
   * - This is from our master
   * - The sequence ID matches the last valid SYNC message
   */
  get_source_port_id(ptp, port, RECEIVED_PACKET, rxBuffer, (uint8_t*)&rxIdentity);
  if((ptp->ports[port].selectedRole == PTP_SLAVE) &&
     (0 == memcmp(&rxIdentity, &ptp->gmPriority->sourcePortIdentity, sizeof(PtpPortIdentity))) &&
     ptp->ports[port].syncSequenceIdValid &&
     (get_sequence_id(ptp, port, RECEIVED_PACKET, rxBuffer) == ptp->ports[port].syncSequenceId)) {
    PtpTime syncTxTimestamp;
    PtpTime correctionField;
    PtpTime correctedTimestamp;
    PtpTime difference;
    PtpTime absDifference;

     ptp->ports[port].syncTimeoutCounter = 0;

    /* Everything matches; obtain the preciseOriginTimestamp from the packet.
     * This is the time at which the master captured its transmit of the preceding
     * SYNC, which we also timestamped reception for.
     */
    get_timestamp(ptp, port, RECEIVED_PACKET, rxBuffer, &syncTxTimestamp);

    /* Correct the Tx timestamp with the received correction field */
    get_correction_field(ptp, port, rxBuffer, &correctionField);

    /* Save off the timestamp and correction field info for any ports that need to forward it */
    for (i=0; i<ptp->numPorts; i++) {
      if (ptp->ports[i].selectedRole == PTP_MASTER) {
        timestamp_copy(&ptp->ports[i].lastPreciseOriginTimestamp, &syncTxTimestamp);
        timestamp_copy(&ptp->ports[i].lastFollowUpCorrectionField, &correctionField);
        ptp->ports[i].fupPreciseOriginTimestampReceived = TRUE;
      }
    }

    timestamp_sum(&syncTxTimestamp, &correctionField, &correctedTimestamp);

    preempt_disable();
    spin_lock_irqsave(&ptp->mutex, flags);
    timestamp_copy(&ptp->ports[port].syncRxTimestamp, &ptp->ports[port].syncRxTimestampTemp);
    timestamp_copy(&ptp->ports[port].syncTxTimestamp, &correctedTimestamp);
    ptp->ports[port].syncTimestampsValid = 1;
    spin_unlock_irqrestore(&ptp->mutex, flags);
    preempt_enable();

    /* Retrieve the scaled rate offset */
    ptp->ports[port].cumulativeScaledRateOffset = get_cumulative_scaled_rate_offset_field(rxBuffer);

    /* Treat GM phase change just like a GM change */
    if (get_gm_time_base_indicator_field(rxBuffer) != ptp->lastGmTimeBaseIndicator) {
      printk("GM Phase Change\n");
      labx_ptp_signal_gm_change(ptp);

      ptp->lastGmTimeBaseIndicator = get_gm_time_base_indicator_field(rxBuffer);
      get_gm_phase_change_field(rxBuffer, &ptp->lastGmPhaseChange);
      ptp->lastGmFreqChange = get_gm_freq_change_field(rxBuffer);
    }

    /* Compare the timestamps; if the one-way offset plus delay is greater than
     * the reset threshold, we need to reset our RTC before beginning to servo.  Regardless
     * of what we do, we need to invalidate the sync sequence ID, it's been "used up."
     */
    ptp->ports[port].syncSequenceIdValid = 0;
    timestamp_difference(&ptp->ports[port].syncRxTimestampTemp, &correctedTimestamp, &difference);
    timestamp_abs(&difference, &absDifference);
    if((absDifference.secondsUpper > 0) || (absDifference.secondsLower > 0) ||
       (absDifference.nanoseconds > RESET_THRESHOLD_NS) || ptp->rtcReset) {

      /* Treat clock changes just like a GM change when we think we are locked */
      if (ptp->rtcLockState == PTP_RTC_LOCKED) {
        printk("RTC Change while locked\n");
        labx_ptp_signal_gm_change(ptp);
      }

      /* Reset the time using the corrected timestamp adjusted by the time it has been
       * resident locally since it was received; also re-load the nominal
       * RTC increment in advance in order to always have a known starting point
       * for convergence.  Suppress this if the userspace controller hasn't acknowledged
       * a Grandmaster change yet.
       */
      if(ptp->rtcChangesAllowed) {
        printk("Resetting RTC!\n");
        set_rtc_increment(ptp, &ptp->nominalIncrement);
        set_rtc_time_adjusted(ptp, &correctedTimestamp, &ptp->ports[port].syncRxTimestampTemp);
        ptp->rtcReset = FALSE;
      }
    } else {
      /* Less than a second, leave these timestamps and update the servo */
#ifdef SYNC_DEBUG
      printk("Sync Rx: %08X%08X.%08X\n", ptp->ports[port].syncRxTimestampTemp.secondsUpper,
        ptp->ports[port].syncRxTimestampTemp.secondsLower, ptp->ports[port].syncRxTimestampTemp.nanoseconds);
      printk("Sync Tx: %08X%08X.%08X (corrected: %08X%08X.%08X\n", syncTxTimestamp.secondsUpper,
        syncTxTimestamp.secondsLower, syncTxTimestamp.nanoseconds, correctedTimestamp.secondsUpper,
        correctedTimestamp.secondsLower, correctedTimestamp.nanoseconds);
      printk("Correction: %08X%08X.%08X\n", correctionField.secondsUpper,
        correctionField.secondsLower, correctionField.nanoseconds);
#endif
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      rtc_update_servo(ptp, port);
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
    }

    /* Forward the follow-up to any master ports */
    for (i=0; i<ptp->numPorts; i++) {
      if (ptp->ports[i].selectedRole == PTP_MASTER) {
        if (ptp->ports[i].syncTxLocalTimestampValid) {
          transmit_fup(ptp, i);
        }
      }
    }
  }
}

/* Processes a newly-received signaling packet for the passed instance */
static void process_rx_signaling(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {

  ptp->ports[port].rcvdSignalingPtr = rxBuffer;
  ptp->ports[port].rcvdSignalingMsg1 = TRUE;

  LinkDelaySyncIntervalSetting_StateMachine(ptp, port);
}

/* Processes a newly-received DELAY_REQ packet for the passed instance */
static void process_rx_delay_req(struct ptp_device *ptp, uint32_t port, uint8_t * rxBuffer) {
  /* Only react to these messages if we are the master */
  if(ptp->ports[port].selectedRole == PTP_MASTER) {
    /* React to the reception of a delay request by simply transmitting a delay
     * response back to the slave.
     */
    transmit_delay_response(ptp, port, rxBuffer);
  }
};

/* Processes a newly-received DELAY_RESP packet for the passed instance */
static void process_rx_delay_resp(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {
  unsigned long flags;
  uint8_t rxMacAddress[MAC_ADDRESS_BYTES];
  uint8_t rxRequestingPortId[PORT_ID_BYTES];
  uint8_t txRequestingPortId[PORT_ID_BYTES];
  uint8_t *txBuffer;

  /* Make certain of the following:
   * - We are a slave
   * - This is from our master
   * - The response's source port ID matches our own
   * - The sequence ID matches the last valid DELAY_REQ message
   */
  get_rx_mac_address(ptp, port, rxBuffer, rxMacAddress);
  get_rx_requesting_port_id(ptp, port, rxBuffer, rxRequestingPortId);
  txBuffer = get_output_buffer(ptp,port,PTP_TX_PDELAY_REQ_BUFFER);
  get_source_port_id(ptp, port, TRANSMITTED_PACKET, txBuffer,
                     txRequestingPortId);

  txBuffer = get_output_buffer(ptp,port,PTP_TX_DELAY_REQ_BUFFER);
  if((ptp->ports[port].selectedRole == PTP_SLAVE) &&
     (compare_port_ids(rxRequestingPortId, txRequestingPortId) == 0) &&
     (get_sequence_id(ptp, port, RECEIVED_PACKET, rxBuffer) ==
      get_sequence_id(ptp, port, TRANSMITTED_PACKET, txBuffer))) {
    PtpTime delayReqRxTimestamp;
    PtpTime difference;
    PtpTime absDifference;

    /* Everything matches; obtain the requestReceiptTimestamp from the packet.
     * This is the time at which the master captured its receive of the delay request
     * packet we sent, which we also timestamped reception for.
     */
    get_timestamp(ptp, port, RECEIVED_PACKET, rxBuffer, &delayReqRxTimestamp);

    /* Make certain the one-way delay calculates to less than one second; if so,
     * discard the information and wait for the time to be reset via the SYNC
     * and FUP mechanism.
     */
    timestamp_difference(&delayReqRxTimestamp, &ptp->ports[port].delayReqTxTimestampTemp, &difference);
    timestamp_abs(&difference, &absDifference);
    if((absDifference.secondsUpper == 0) && (absDifference.secondsLower == 0)) {
      /* Less than a second, leave these timestamps */
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      timestamp_copy(&ptp->ports[port].delayReqTxTimestamp, &ptp->ports[port].delayReqTxTimestampTemp);
      timestamp_copy(&ptp->ports[port].delayReqTxLocalTimestamp, &ptp->ports[port].delayReqTxLocalTimestampTemp);
      timestamp_copy(&ptp->ports[port].delayReqRxTimestamp, &delayReqRxTimestamp);
      ptp->ports[port].delayReqTimestampsValid = 1;
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
    }
  }
}

/* Processes a newly-received PDELAY_REQ packet for the passed instance */
static void process_rx_pdelay_req(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {

  PtpPortIdentity rxIdentity;

  ptp->ports[port].stats.rxPDelayRequestCount++;

  /* React to peer delay requests no matter what, even if we're not using the
   * peer-to-peer delay mechanism or if we're a slave or master.  Transmit
   * a peer delay response back - we will also transmit a peer delay response
   * followup once this message is on the wire. The only exception is if the
   * request came from any port on this system we should discard it.
   */
  get_source_port_id(ptp, port, RECEIVED_PACKET, rxBuffer, (uint8_t*)&rxIdentity);
  if (0 != compare_clock_identity(rxIdentity.clockIdentity, ptp->systemPriority.rootSystemIdentity.clockIdentity)) {
      transmit_pdelay_response(ptp, port, rxBuffer);
   } else {
    uint16_t rxPortNumber = get_port_number(rxIdentity.portNumber);
    printk("Disabling AS on ports %d and %d due to receipt of our own pdelay.\n", port+1, rxPortNumber);
    ptp->ports[port].portEnabled = FALSE;
    if ((rxPortNumber >= 1) && (rxPortNumber <= ptp->numPorts)) {
      ptp->ports[rxPortNumber-1].portEnabled = FALSE;
    }
  }
};

/* Processes a newly-received PDELAY_RESP packet for the passed instance */
static void process_rx_pdelay_resp(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {

  ptp->ports[port].stats.rxPDelayResponseCount++;

  ptp->ports[port].rcvdPdelayResp = TRUE;
  ptp->ports[port].rcvdPdelayRespPtr = rxBuffer;

  /* AVnu_PTP-5 from AVnu Combined Endpoint PICS D.0.0.1
     Cease pDelay_Req transmissions if more than one
     pDelay_Resp messages have been received for each of
     three successive pDelay_Req messages. */
  ptp->ports[port].pdelayResponses++;

  MDPdelayReq_StateMachine(ptp, port);
}

/* Processes a newly-received PDELAY_RESP_FUP packet for the passed instance */
static void process_rx_pdelay_resp_fup(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {

  ptp->ports[port].stats.rxPDelayResponseFollowupCount++;

  ptp->ports[port].rcvdPdelayRespFollowUp = TRUE;
  ptp->ports[port].rcvdPdelayRespFollowUpPtr = rxBuffer;

  MDPdelayReq_StateMachine(ptp, port);
}

/* Tasklet function for PTP Rx packets */
void labx_ptp_rx_state_task(unsigned long data) {
  struct ptp_device *ptp = (struct ptp_device *) data;
  int i;

  for(i=0; i<ptp->numPorts; i++) {
    /* Make sure any pending Tx operations are completed. Tasklets aren't run in any particular order */
    if (ptp->ports[i].pendingTxFlags != PTP_TX_BUFFER_NONE)
    {
      labx_ptp_tx_state_task(data);
    }
    ptp_process_rx(ptp,i);
  }
}

void process_rx_buffer(struct ptp_device *ptp, int port, uint8_t *buffer)
{
  if(TRANSPORT_PTP == get_transport_specific(ptp, port, buffer)) {
       /* Determine which message to process */
      switch(get_message_type(ptp, port, buffer)) {
      case MSG_ANNOUNCE:
        process_rx_announce(ptp, port, buffer);
        break;

      case MSG_SYNC:
        process_rx_sync(ptp, port, buffer);
        break;

      case MSG_FUP:
        process_rx_fup(ptp, port, buffer);
        break;

      case MSG_DELAY_REQ:
        process_rx_delay_req(ptp, port, buffer);
        break;
      case MSG_DELAY_RESP:
        process_rx_delay_resp(ptp, port, buffer);
        break;

      case MSG_PDELAY_REQ:
        process_rx_pdelay_req(ptp, port, buffer);
        break;

      case MSG_PDELAY_RESP:
        process_rx_pdelay_resp(ptp, port, buffer);
        break;

      case MSG_PDELAY_RESP_FUP:
        process_rx_pdelay_resp_fup(ptp, port, buffer);
        break;

      case MSG_SIGNALING:
        process_rx_signaling(ptp, port, buffer);
        break;

      default:
        break;
      } /* switch(messageType) */
  } else printk("WARNING: Unrecognized transportSpecific rejected\n");
}

/* Tasklet function for PTP Tx packets */
void labx_ptp_tx_state_task(unsigned long data) {
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
  PtpTime diff;
  switch_timestamp_t *switch_t1;
  switch_timestamp_t *switch_t2;
  switch_timestamp_t switch_t1a,switch_t2a;
  switch_timestamp_t switch_t1b,switch_t2b;
  int32_t t1,t2;
  uint16_t sequence_id;
  int cnt1,cnt2;
#endif

  struct ptp_device *ptp = (struct ptp_device *) data;
  uint32_t pendingTxFlags;
  uint32_t whichBuffer;
  uint32_t bufferMask;
  unsigned long flags;
  uint8_t *txBuffer;
  int i;
  bool localMaster = true;

  for (i=0; i<ptp->numPorts; i++) {
    if (ptp->ports[i].selectedRole == PTP_SLAVE) {
      localMaster = false;
      break;
    }
  }

  for(i=0; i<ptp->numPorts; i++) {
    /* A packet has been transmitted; examine the pending flags to to see which one(s).
     * Lock the mutex to avoid a race condition with the Tx IRQ.
     */
    preempt_disable();
    spin_lock_irqsave(&ptp->mutex, flags);

    /* Clear the pending flags in the device structure once we've cached them; we're
     * going to "consume" them here.
     */
    pendingTxFlags = ptp->ports[i].pendingTxFlags;
    ptp->ports[i].pendingTxFlags = PTP_TX_BUFFER_NONE;
    spin_unlock_irqrestore(&ptp->mutex, flags);
    preempt_enable();

    /* Loop until the flags have been cleared */
    for(whichBuffer = 0, bufferMask = PTP_TX_BUFFER(0);
        pendingTxFlags != PTP_TX_BUFFER_NONE;
        whichBuffer++, bufferMask <<= 1) {
      if((pendingTxFlags & bufferMask) != PTP_TX_BUFFER_NONE) {
        /* Clear the pending flag bit and then check for action to be performed */
        pendingTxFlags &= ~bufferMask;
        switch(whichBuffer) {
        case PTP_TX_SYNC_BUFFER: {
          txBuffer = get_output_buffer(ptp, i, PTP_TX_SYNC_BUFFER);

#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
            sequence_id=get_sequence_id(ptp, i, TRANSMITTED_PACKET, txBuffer);
            cnt1=0;
            cnt2=0;
            if(i==0) {
              switch_timestamp(TIMESTAMP_AVB2_OUTGOING_SYNC,&switch_t1, &switch_t2,sequence_id);
              if(switch_t1==NULL) {
                  do {
                    block_read_avb_ptp(&switch_t1a,5,0x0c);
                    block_read_avb_ptp(&switch_t1b,5,0x0c);
                    cnt1++;
                  } while(
                        (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
                        (switch_t1a.low!=switch_t1b.low)||
                        (switch_t1a.high!=switch_t1b.high));
              } else {
                  memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
              }
              if(switch_t2==NULL) {
                do {
                    block_read_avb_ptp(&switch_t2a,0,0x10);
                    block_read_avb_ptp(&switch_t2b,0,0x10);
                    cnt2++;
                  } while(
                        (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
                        (switch_t2a.low!=switch_t2b.low)||
                        (switch_t2a.high!=switch_t2b.high));
              } else {
                  memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
              }
            } else {
              switch_timestamp(TIMESTAMP_AVB1_OUTGOING_SYNC,&switch_t1, &switch_t2,sequence_id);
              if(switch_t1==NULL) {
                  do {
                    block_read_avb_ptp(&switch_t1a,6,0x0c);
                    block_read_avb_ptp(&switch_t1b,6,0x0c);
                    cnt1++;
                  } while(
                        (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
                        (switch_t1a.low!=switch_t1b.low)||
                        (switch_t1a.high!=switch_t1b.high));
              } else {
                  memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
              }
              if(switch_t2==NULL) {
                  do {
                    block_read_avb_ptp(&switch_t2a,1,0x10);
                    block_read_avb_ptp(&switch_t2b,1,0x10);
                    cnt2++;
                  } while(
                        (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
                        (switch_t2a.low!=switch_t2b.low)||
                        (switch_t2a.high!=switch_t2b.high));
              } else {
                  memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
              }
            }
            if((cnt1!=0)||(cnt2!=0)) {
                DEBUG_TIMESTAMP_PRINTF("G retry%d,%d\r\n",cnt1,cnt2);
            }
#endif
          if (localMaster) {
            /* A sync message was just transmitted; send a followup message containing the
             * hardware-timestamped transmit time of the SYNC.
             */
            get_hardware_timestamp(ptp, i, TRANSMITTED_PACKET, txBuffer, &ptp->ports[i].syncTxTimestamp);
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
            if((sequence_id==switch_t1a.sequence_id) &&
               (sequence_id==switch_t2a.sequence_id)) {
              t1=(switch_t1a.high<<16)|switch_t1a.low;
              t2=(switch_t2a.high<<16)|switch_t2a.low;

              diff.secondsUpper=0;
              diff.secondsLower=0;
              diff.nanoseconds=(t2-t1)*8;
              if((uint32_t)t2<(uint32_t)t1) {
                WARN_TIMESTAMP_PRINTF("GG t2 wrapped %04x\r\n",diff.nanoseconds); 
              }
              timestamp_sum(&ptp->ports[i].syncTxTimestamp,&diff,&ptp->ports[i].syncTxTimestamp);
              transmit_fup(ptp, i);
            } else {
              ERROR_TIMESTAMP_PRINTF("GG missed tx sync %04x:%04x instead of %04x\r\n",switch_t1a.sequence_id,switch_t2a.sequence_id,sequence_id);
            }
#else
            transmit_fup(ptp, i);
#endif
          } else {
            /* Save the sync transmit time to calculate residency time once the follow-up comes in. */
            get_local_hardware_timestamp(ptp, i, TRANSMITTED_PACKET, txBuffer,
                                         &ptp->ports[i].syncTxTimestamp);
            ptp->ports[i].syncTxLocalTimestampValid = TRUE;
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
            if((sequence_id==switch_t1a.sequence_id) &&
               (sequence_id==switch_t2a.sequence_id)) {
              t1=(switch_t1a.high<<16)|switch_t1a.low;
              t2=(switch_t2a.high<<16)|switch_t2a.low;

              diff.secondsUpper=0;
              diff.secondsLower=0;
              diff.nanoseconds=(t2-t1)*8;
              if((uint32_t)t2<(uint32_t)t1) {
                WARN_TIMESTAMP_PRINTF("G t2 wrapped %04x\r\n",diff.nanoseconds);
              }
              timestamp_sum(&ptp->ports[i].syncTxTimestamp,&diff,&ptp->ports[i].syncTxTimestamp);
            } else {
              ptp->ports[i].syncTxLocalTimestampValid = FALSE;
              ERROR_TIMESTAMP_PRINTF("G missed tx sync %04x:%04x instead of %04x\r\n",switch_t1a.sequence_id,switch_t2a.sequence_id,sequence_id);
            }
#endif
            /* If the follow-up already arrived, forward it now. */
            if (ptp->ports[i].fupPreciseOriginTimestampReceived) {
              transmit_fup(ptp, i);
            }
          }
        } break;

        case PTP_TX_DELAY_REQ_BUFFER: {
          /* A delay request message has just been sent; capture and store the
           * transmission timestamp for later use.
           */
          txBuffer = get_output_buffer(ptp,i,PTP_TX_DELAY_REQ_BUFFER);
          get_hardware_timestamp(ptp, i, TRANSMITTED_PACKET, txBuffer,
                                 &ptp->ports[i].delayReqTxTimestampTemp);
          get_local_hardware_timestamp(ptp, i, TRANSMITTED_PACKET, txBuffer,
                                       &ptp->ports[i].delayReqTxLocalTimestampTemp);

        } break;

        case PTP_TX_PDELAY_REQ_BUFFER: {
          /* A peer delay request message has just been sent; capture and store the
           * transmission timestamp for later use. (Treq1 - our local clock)
           */
          uint8_t *txBuffer = get_output_buffer(ptp,i,PTP_TX_PDELAY_REQ_BUFFER);
          get_local_hardware_timestamp(ptp, i, TRANSMITTED_PACKET, txBuffer ,
                                       &ptp->ports[i].pdelayReqTxTimestamp);
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
          sequence_id=get_sequence_id(ptp, i, TRANSMITTED_PACKET, txBuffer);
          cnt1=0;
          cnt2=0;
          if(i==0) {
            switch_timestamp(TIMESTAMP_AVB2_OUTGOING_REQUEST,&switch_t1, &switch_t2,sequence_id);
            if(switch_t1==NULL) {
                do {
                  block_read_avb_ptp(&switch_t1a,5,0x08);
                  block_read_avb_ptp(&switch_t1b,5,0x08);
                  cnt1++;
                } while(
                      (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
                      (switch_t1a.low!=switch_t1b.low)||
                      (switch_t1a.high!=switch_t1b.high));
            } else {
                memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
            }
            if(switch_t2==NULL) {
                do {
                  block_read_avb_ptp(&switch_t2a,0,0x10);
                  block_read_avb_ptp(&switch_t2b,0,0x10);
                  cnt2++;
                } while(
                      (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
                      (switch_t2a.low!=switch_t2b.low)||
                      (switch_t2a.high!=switch_t2b.high));
            } else {
                memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
            }
          } else {
            switch_timestamp(TIMESTAMP_AVB1_OUTGOING_REQUEST,&switch_t1, &switch_t2,sequence_id);
            if(switch_t1==NULL) {
                do {
                  block_read_avb_ptp(&switch_t1a,6,0x08);
                  block_read_avb_ptp(&switch_t1b,6,0x08);
                  cnt1++;
                } while(
                      (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
                      (switch_t1a.low!=switch_t1b.low)||
                      (switch_t1a.high!=switch_t1b.high));
            } else {
                memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
            }
            if(switch_t2==NULL) {
                do {
                  block_read_avb_ptp(&switch_t2a,1,0x10);
                  block_read_avb_ptp(&switch_t2b,1,0x10);
                  cnt2++;
                } while(
                      (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
                      (switch_t2a.low!=switch_t2b.low)||
                      (switch_t2a.high!=switch_t2b.high));
            } else {
                memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
            }
          }
          if((cnt1!=0)||(cnt2!=0)) {
              DEBUG_TIMESTAMP_PRINTF("K retry%d,%d\r\n",cnt1,cnt2);
          }
          if((sequence_id==switch_t1a.sequence_id) &&
             (sequence_id==switch_t2a.sequence_id)) {
            t1=(switch_t1a.high<<16)|switch_t1a.low;
            t2=(switch_t2a.high<<16)|switch_t2a.low;
            diff.secondsUpper=0;
            diff.secondsLower=0;
            diff.nanoseconds=(t2-t1)*8;
            if((uint32_t)t2<(uint32_t)t1) {
              WARN_TIMESTAMP_PRINTF("K t2 wrapped %04x\r\n",diff.nanoseconds);
            }
          } else {
            ERROR_TIMESTAMP_PRINTF("K missed tx request %04x:%04x instead of %04x\r\n",switch_t1a.sequence_id,switch_t2a.sequence_id,sequence_id);
            break;
          }
          timestamp_sum(&ptp->ports[i].pdelayReqTxTimestamp,&diff,&ptp->ports[i].pdelayReqTxTimestamp);
#endif
          ptp->ports[i].rcvdMDTimestampReceive = TRUE;
          MDPdelayReq_StateMachine(ptp, i);
        } break;

        case PTP_TX_PDELAY_RESP_BUFFER: {
          /* A peer delay response message was just transmitted; send a peer delay
           * response followup message containing the hardware-timestamped transmit
           * time of the response.
           */
          transmit_pdelay_response_fup(ptp, i);
        } break;

        default:
          /* A message was sent that requires no follow-up action */
          break;
        } /* case(whichBuffer) */
      } /* if(whichBuffer was transmitted) */
    } /* for(all Tx buffers) */
  } /* for(all ports) */
}

/* Acknowledges a Grandmaster change, freeing the instance to begin slewing
 * (or hard-setting) its RTC increment again.
 */
void ack_grandmaster_change(struct ptp_device *ptp) {
  unsigned long flags;

  printk("GM ACK\n");
  /* Re-enable the changing of RTC parameters */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  ptp->rtcChangesAllowed = TRUE;
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();

  /* Set the RTC back to its nominal increment; even if we're going to
   * operate as a slave, this is the point we wish to start at.
   */
  set_rtc_increment(ptp, &ptp->nominalIncrement);
}

/* Initializes all of the state machines */
void init_state_machines(struct ptp_device *ptp) {
#ifdef CONFIG_OF
  struct of_device* interfaceDev;
#endif
  struct net_device *ndev = NULL;
  int i;

  /* Initialize the timer state machine */
#ifndef CONFIG_LABX_PTP_NO_TASKLET
  tasklet_init(&ptp->timerTasklet, &labx_ptp_timer_state_task, (unsigned long) ptp);
#endif
  ptp->heartbeatCounter = 0;
  ptp->netlinkSequence  = 0;

  for(i=0; i<ptp->numPorts; i++) {
    struct ptp_port *pPort = &ptp->ports[i];

    pPort->firstAnnounceSent   = FALSE;
    pPort->announceCounter     = 0;
    pPort->announceSequenceId  = 0x0000;
    pPort->syncCounter         = 0;
    pPort->syncSequenceId      = 0x0000;
    pPort->syncSequenceIdValid = 0;
    pPort->delayReqCounter     = 0;
    pPort->delayReqSequenceId  = 0x0000;

    pPort->currentLogSyncInterval = -3;
    pPort->initialLogSyncInterval = -3;

#ifdef CONFIG_OF
    interfaceDev = of_find_device_by_node(pPort->interfaceNode);
    ndev = platform_get_drvdata(to_platform_device(&interfaceDev->dev));
#else
    ndev = dev_get_by_name(&init_net, pPort->interfaceName);
#endif
    if((ndev!=NULL) && netif_carrier_ok(ndev)) {
      pPort->portEnabled = TRUE;
    } else {
      pPort->portEnabled = FALSE;
    }
    pPort->pttPortEnabled = TRUE;

    pPort->currentLogAnnounceInterval = 0;
    pPort->initialLogAnnounceInterval = 0;

    pPort->syncReceiptTimeout = 3;
    pPort->announceReceiptTimeout = 3;

    /* peer delay request state machine initialization */
    pPort->mdPdelayReq_State    = MDPdelayReq_NOT_ENABLED;
    pPort->mdPdelayReq_LastTime = 0;
    pPort->allowedLostResponses = 3;
    /* Recommended value in 802.1AS/COR1 is 800ns for copper. Add 400ns to allow
       use of the nTap and still come up as asCapable. */
    pPort->neighborPropDelayThresh = 1200;

    /* PortAnnounceInformation state machine initializetion */
    pPort->portAnnounceInformation_State = PortAnnounceInformation_BEGIN;
  }

  /* Initialize the Rx state machine, presuming we are a master; set the nominal
   * RTC increment, enabling the counter.
   */
  ptp->portRoleSelection_State = PortRoleSelection_INIT_BRIDGE;
  PortRoleSelection_StateMachine(ptp);
  ptp->newMaster              = TRUE;
  ptp->rtcChangesAllowed      = TRUE;

  ptp->masterRateRatio = 0x80000000;
  ptp->masterRateRatioValid = FALSE;
  ptp->prevBaseRtcIncrement = 0;
  ptp->prevAppliedRtcIncrement = 0;

  ptp->lastGmTimeBaseIndicator = 0;

#ifndef CONFIG_LABX_PTP_NO_TASKLET
  tasklet_init(&ptp->rxTasklet, &labx_ptp_rx_state_task, (unsigned long) ptp);
#endif

  for(i=0; i<ptp->numPorts; i++) {
    ptp_platform_init(ptp,i);
    ptp->ports[i].syncTimestampsValid     = 0;
    ptp->ports[i].delayReqTimestampsValid = 0;
    ptp->ports[i].neighborPropDelay       = 0;
    ptp->ports[i].announceTimeoutCounter  = 0;
    ptp->ports[i].syncTimeoutCounter      = 0;
  }

  ptp->integral             = 0;
  ptp->zeroCrossingIntegral = 0;
  ptp->derivative           = 0;
  ptp->previousOffset       = 0;
  set_rtc_increment(ptp, &ptp->nominalIncrement);

  /* Declare the RTC as initially unlocked, but put a valid, zero, offset
   * in so that it will lock shortly after the lock detection state machine
   * has run for a little bit.
   */
  ptp->acquiring             = PTP_RTC_ACQUIRING;
  ptp->rtcLastLockState      = PTP_RTC_UNLOCKED;
  ptp->rtcLockState          = PTP_RTC_UNLOCKED;
  ptp->rtcLockCounter        = 0;
  ptp->rtcLastOffsetValid    = PTP_RTC_OFFSET_VALID;
  ptp->rtcLastOffset         = 0;
  ptp->rtcLastIncrementDelta = 0;

  printk("PTP master\n");

#ifndef CONFIG_LABX_PTP_NO_TASKLET
  /* Initialize the Tx state tasklet */
  tasklet_init(&ptp->txTasklet, &labx_ptp_tx_state_task, (unsigned long) ptp);
#endif
}

