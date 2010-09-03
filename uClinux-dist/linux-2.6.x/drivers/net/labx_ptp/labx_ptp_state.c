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
#include <xio.h>

/* Define this to get some extra debug on path delay messages */
#define PATH_DELAY_DEBUG

/* Define this to get some extra debug on sync/follow-up messages */
/* #define SYNC_DEBUG */

/* Timer tick period */
#define TIMER_TICK_MS         (10)

/* Parameters governing message rates, etc.
 * TODO: Make these ioctl()-configurable!  This will help us tune the control loop.
 */
#define ANNOUNCE_INTERVAL         (1000)
#define ANNOUNCE_RECEIPT_TIMEOUT     (3)
#define SYNC_INTERVAL              (100)
#define DELAY_REQ_INTERVAL        (1000)
#define PDELAY_REQ_INTERVAL       (1000)

/* Enumerated type identifying the results of a BMCA comparison */
typedef enum {
  IS_PRESENT_MASTER,
  RETAIN_PRESENT_MASTER,
  REPLACE_PRESENT_MASTER
} BmcaResult;

/* Tasklet function for responding to timer interrupts */
static void timer_state_task(unsigned long data) {
  struct ptp_device *ptp = (struct ptp_device*) data;
  unsigned long flags;

  /* We behave differently as a master than as a slave */
  switch(ptp->presentRole) {
  case PTP_MASTER:
    {
      /* Send ANNOUNCE and SYNC messages at their rate if we're a master */
      ptp->announceCounter++;
      if(ptp->announceCounter >= (ANNOUNCE_INTERVAL / TIMER_TICK_MS)) {
        ptp->announceCounter = 0;
        transmit_announce(ptp);
      }

      ptp->syncCounter++;
      if(ptp->syncCounter >= (SYNC_INTERVAL / TIMER_TICK_MS)) {
        ptp->syncCounter = 0;
        transmit_sync(ptp);
      }
    }
    break;

  case PTP_SLAVE:
    {
      uint32_t timeoutTicks;

      /* Increment and test the announce receipt timeout counter */
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      timeoutTicks = ((ANNOUNCE_INTERVAL * ANNOUNCE_RECEIPT_TIMEOUT) / TIMER_TICK_MS);
      if(++ptp->announceTimeoutCounter >= timeoutTicks) {
        /* We haven't received an ANNOUNCE message from our master in too long, presume
         * we've become a master so we participate in BMCA again.
         */
        ptp->presentRole = PTP_MASTER;
        copy_ptp_properties(&ptp->presentMaster, &ptp->properties);
        ptp->announceTimeoutCounter = 0;

        /* Set the RTC back to its nominal increment */
        set_rtc_increment(ptp, &ptp->nominalIncrement);
      }
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();

      /* Transmit an ANNOUNCE immediately to speed things along if we've switched our
       * port to the master state.
       */
      if(ptp->presentRole == PTP_MASTER) {
        printk("PTP master\n");
        ptp->announceCounter    = 0;
        ptp->announceSequenceId = 0x0000;
        transmit_announce(ptp);
      } else {
        /* Still a slave; determine whether we are using the end-to-end or peer-to-peer
         * delay mechanism
         */
        if(ptp->properties.delayMechanism == PTP_DELAY_MECHANISM_E2E) {
          /* Increment the delay request counter and see if it's time to
           * send one to the master.
           */
          if(++ptp->delayReqCounter >= (DELAY_REQ_INTERVAL / TIMER_TICK_MS)) {
            ptp->delayReqCounter = 0;
            transmit_delay_request(ptp);
          }
        }
      } /* if(still a slave) */
    }
    break;

  default:
    /* "Passive"; do nothing */
    ;
  }

  /* Regardless of whether we are a master or slave, increment the peer delay request
   * counter and see if it's time to send one to our link peer.
   */
  ptp->pdelayIntervalTimer++;
  MDPdelayReq_StateMachine(ptp);
}

/* Runs the Best Master Clock Algorithm (BMCA) between the passed master and challenger.
 * Returns nonzero if the challenger should become the new master.
 *
 * NOTE - This implementation does not (yet) handle multiple ports and the accompanying
 *        logic to determine which port is better by topology with respect to the master.
 */
static BmcaResult bmca_comparison(PtpProperties *presentMaster, PtpProperties *challenger) {
  uint32_t returnValue;
  int32_t macComparison;

  /* Short-circuit the comparison if this is an announce message from the present master,
   * which should usually be the case during normal operation of a PTP network.
   */
  macComparison = 
    compare_mac_addresses(challenger->sourceMacAddress, presentMaster->sourceMacAddress);
  if(macComparison == 0) {
    /* This is an announce message from the present master */
    returnValue = IS_PRESENT_MASTER;
  } else {
    /* Not a message from the present master, we need to run the BMCA.  Presume we will
     * retain our present master unless proven otherwise.
     */
    returnValue = RETAIN_PRESENT_MASTER;

    /* Begin by comparing grandmaster priority 1; lower value is higher priority */
    if(challenger->grandmasterPriority1 < presentMaster->grandmasterPriority1) {
      returnValue = REPLACE_PRESENT_MASTER;
    } else if(challenger->grandmasterPriority1 == presentMaster->grandmasterPriority1) {
      PtpClockQuality *challengerQuality = &challenger->grandmasterClockQuality;
      PtpClockQuality *presentQuality = &challenger->grandmasterClockQuality;

      /* Priority 1 identical, compare clock quality - again, lower is "better". */
      if(challengerQuality->clockClass < presentQuality->clockClass) {
        returnValue = REPLACE_PRESENT_MASTER;
      } else if(challengerQuality->clockClass == presentQuality->clockClass) {
        /* Clock class equal, go to accuracy */
        if(challengerQuality->clockAccuracy < presentQuality->clockAccuracy) {
          returnValue = REPLACE_PRESENT_MASTER;
        } else if(challengerQuality->clockAccuracy == presentQuality->clockAccuracy) {
          /* Accuracy identical, go to offset scaled log variance */
          if(challengerQuality->offsetScaledLogVariance < presentQuality->offsetScaledLogVariance) {
            returnValue = REPLACE_PRESENT_MASTER;
          } else if(challengerQuality->offsetScaledLogVariance == presentQuality->offsetScaledLogVariance) {
            /* Clock settings completely identical, compare MAC addresses as a tie-breaker */
            if(compare_clock_identity(challenger->grandmasterIdentity,presentMaster->grandmasterIdentity) < 0) {
              /* The new announce message has a lower MAC address, it becomes the master */
              returnValue = REPLACE_PRESENT_MASTER;
            } /* macAddress (clockIdentity) */
          } /* offsetScaledLogVariance */
        } /* clockAccuracy */
      } /* clockClass */
    } /* priority1 */
  } /* if(present master) */
    
  return(returnValue);
}

/* Processes a newly-received ANNOUNCE packet for the passed instance */
static void process_rx_announce(struct ptp_device *ptp, uint32_t rxBuffer) {
  PtpProperties portProperties;
  unsigned long flags;
  uint32_t byteIndex;

  /* Extract the properties of the port which sent the message, and compare 
   * them to those of the present master to determine what to do.
   */
  extract_announce(ptp, rxBuffer, &portProperties);
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  switch(bmca_comparison(&ptp->presentMaster, &portProperties)) {
  case IS_PRESENT_MASTER: {
    /* A message from our fearless leader; reset its timeout counter */
    ptp->announceTimeoutCounter = 0;
  } break;
    
  case REPLACE_PRESENT_MASTER: {
    /* Replace the present master's properties, and ensure that we're a slave.
     */
    ptp->presentRole = PTP_SLAVE;
    copy_ptp_properties(&ptp->presentMaster, &portProperties);
    ptp->announceTimeoutCounter = 0;
    
    /* Invalidate all the slave flags */
    ptp->syncTimestampsValid       = 0;
    ptp->delayReqTimestampsValid   = 0;
    ptp->syncSequenceIdValid       = 0;
    ptp->delayReqCounter           = 0;
    ptp->delayReqSequenceId        = 0x0000;
    ptp->integral                  = 0;
    ptp->derivative                = 0;
    ptp->previousOffset            = 0;
    
    /* Announce the new slave */
    printk("PTP slaved to ");
    for(byteIndex = 0; byteIndex < MAC_ADDRESS_BYTES; byteIndex++) {
      printk("%02X", portProperties.sourceMacAddress[byteIndex]);
      if(byteIndex < (MAC_ADDRESS_BYTES - 1)) printk(":");
    }
    printk("\n");
  } break;

  default:
    /* Retain the present master, but do not reset its timeout counter */
    ;
  } /* switch(BMCA comparison) */
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}

/* Processes a newly-received SYNC packet for the passed instance */
static void process_rx_sync(struct ptp_device *ptp, uint32_t rxBuffer) {
  unsigned long flags;
  uint8_t rxMacAddress[MAC_ADDRESS_BYTES];

  /* Only process this packet if we are a slave and it has come from the master
   * we're presently respecting.  If we're the master, spanning tree should prevent
   * us from ever seeing our own SYNC packets, but better safe than sorry.
   */
  get_rx_mac_address(ptp, rxBuffer, rxMacAddress);
  if((ptp->presentRole == PTP_SLAVE) && 
     (compare_mac_addresses(rxMacAddress, ptp->presentMaster.sourceMacAddress) == 0)) {
    PtpTime tempTimestamp;
    PtpTime correctionField;
    PtpTime correctedTimestamp;

    /* This is indeed a SYNC from the present master.  Capture the hardware timestamp
     * at which we received it, and hang on to its sequence ID for matching to the
     * followup that should follow.
     */
    get_hardware_timestamp(ptp, RECEIVED_PACKET, rxBuffer, &tempTimestamp);
    get_correction_field(ptp, rxBuffer, &correctionField);
    timestamp_difference(&tempTimestamp, &correctionField, &correctedTimestamp);

    preempt_disable();
    spin_lock_irqsave(&ptp->mutex, flags);
    timestamp_copy(&ptp->syncRxTimestampTemp, &correctedTimestamp);
    ptp->syncSequenceId = get_sequence_id(ptp, RECEIVED_PACKET, rxBuffer);
    ptp->syncSequenceIdValid = 1;
    spin_unlock_irqrestore(&ptp->mutex, flags);
    preempt_enable();
  }
}

/* Processes a newly-received FUP packet for the passed instance */
static void process_rx_fup(struct ptp_device *ptp, uint32_t rxBuffer) {
  unsigned long flags;
  uint8_t rxMacAddress[MAC_ADDRESS_BYTES];

  /* Make certain of the following:
   * - We are a slave
   * - This is from our master
   * - The sequence ID matches the last valid SYNC message
   */
  get_rx_mac_address(ptp, rxBuffer, rxMacAddress);
  if((ptp->presentRole == PTP_SLAVE) && 
     (compare_mac_addresses(rxMacAddress, ptp->presentMaster.sourceMacAddress) == 0) &&
     ptp->syncSequenceIdValid && 
     (get_sequence_id(ptp, RECEIVED_PACKET, rxBuffer) == ptp->syncSequenceId)) {
    PtpTime syncTxTimestamp;
    PtpTime correctionField;
    PtpTime correctedTimestamp;
    PtpTime difference;
    PtpTime absDifference;

    /* Everything matches; obtain the preciseOriginTimestamp from the packet.
     * This is the time at which the master captured its transmit of the preceding
     * SYNC, which we also timestamped reception for.
     */
    get_timestamp(ptp, RECEIVED_PACKET, rxBuffer, &syncTxTimestamp);

    /* Correct the Tx timestamp with the received correction field */
    get_correction_field(ptp, rxBuffer, &correctionField);
    timestamp_sum(&syncTxTimestamp, &correctionField, &correctedTimestamp);

    /* Compare the timestamps; if the one-way offset plus delay is greater than
     * one second, we need to reset our RTC before beginning to servo.  Regardless
     * of what we do, we need to invalidate the sync sequence ID, it's been "used up."
     */
    ptp->syncSequenceIdValid = 0;
    timestamp_difference(&ptp->syncRxTimestampTemp, &correctedTimestamp, &difference);
    timestamp_abs(&difference, &absDifference);
    if((absDifference.secondsUpper > 0) || (absDifference.secondsLower > 0)) {
      /* Reset the time using the uncorrected timestamp, and invalidate the SYNC */
      printk("Resetting time!\n");
      set_rtc_time(ptp, &syncTxTimestamp);
   } else {
      /* Less than a second, leave these timestamps and update the servo */
#ifdef SYNC_DEBUG
      printk("Sync Rx: %08X%08X.%08X\n", ptp->syncRxTimestampTemp.secondsUpper,
        ptp->syncRxTimestampTemp.secondsLower, ptp->syncRxTimestampTemp.nanoseconds);
      printk("Sync Tx: %08X%08X.%08X (corrected: %08X%08X.%08X\n", syncTxTimestamp.secondsUpper,
        syncTxTimestamp.secondsLower, syncTxTimestamp.nanoseconds, correctedTimestamp.secondsUpper,
        correctedTimestamp.secondsLower, correctedTimestamp.nanoseconds);
      printk("Correction: %08X%08X.%08X\n", correctionField.secondsUpper,
        correctionField.secondsLower, correctionField.nanoseconds);
#endif
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      timestamp_copy(&ptp->syncRxTimestamp, &ptp->syncRxTimestampTemp);
      timestamp_copy(&ptp->syncTxTimestamp, &correctedTimestamp);
      ptp->syncTimestampsValid = 1;
      rtc_update_servo(ptp);
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
    }
  }
}

/* Processes a newly-received DELAY_REQ packet for the passed instance */
static void process_rx_delay_req(struct ptp_device *ptp, uint32_t rxBuffer) {
  /* Only react to these messages if we are the master */
  if(ptp->presentRole == PTP_MASTER) {
    /* React to the reception of a delay request by simply transmitting a delay
     * response back to the slave.
     */
    transmit_delay_response(ptp, rxBuffer);
  }
};

/* Processes a newly-received DELAY_RESP packet for the passed instance */
static void process_rx_delay_resp(struct ptp_device *ptp, uint32_t rxBuffer) {
  unsigned long flags;
  uint8_t rxMacAddress[MAC_ADDRESS_BYTES];
  uint8_t rxRequestingPortId[PORT_ID_BYTES];
  uint8_t txRequestingPortId[PORT_ID_BYTES];

  /* Make certain of the following:
   * - We are a slave
   * - This is from our master
   * - The response's source port ID matches our own
   * - The sequence ID matches the last valid DELAY_REQ message
   */
  get_rx_mac_address(ptp, rxBuffer, rxMacAddress);
  get_rx_requesting_port_id(ptp, rxBuffer, rxRequestingPortId);
  get_source_port_id(ptp, TRANSMITTED_PACKET, PTP_TX_PDELAY_REQ_BUFFER, 
                     txRequestingPortId);
  if((ptp->presentRole == PTP_SLAVE) && 
     (compare_mac_addresses(rxMacAddress, ptp->presentMaster.sourceMacAddress) == 0) &&
     (compare_port_ids(rxRequestingPortId, txRequestingPortId) == 0) &&
     (get_sequence_id(ptp, RECEIVED_PACKET, rxBuffer) == 
      get_sequence_id(ptp, TRANSMITTED_PACKET, PTP_TX_DELAY_REQ_BUFFER))) {
    PtpTime delayReqRxTimestamp;
    PtpTime difference;
    PtpTime absDifference;

    /* Everything matches; obtain the requestReceiptTimestamp from the packet.
     * This is the time at which the master captured its receive of the delay request
     * packet we sent, which we also timestamped reception for.
     */
    get_timestamp(ptp, RECEIVED_PACKET, rxBuffer, &delayReqRxTimestamp);

    /* Make certain the one-way delay calculates to less than one second; if so,
     * discard the information and wait for the time to be reset via the SYNC
     * and FUP mechanism.
     */
    timestamp_difference(&delayReqRxTimestamp, &ptp->delayReqTxTimestampTemp, &difference);
    timestamp_abs(&difference, &absDifference);
    if((absDifference.secondsUpper == 0) && (absDifference.secondsLower == 0)) {
      /* Less than a second, leave these timestamps */
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      timestamp_copy(&ptp->delayReqTxTimestamp, &ptp->delayReqTxTimestampTemp);
      timestamp_copy(&ptp->delayReqRxTimestamp, &delayReqRxTimestamp);
      ptp->delayReqTimestampsValid = 1;
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
    }
  }
}

/* Processes a newly-received PDELAY_REQ packet for the passed instance */
static void process_rx_pdelay_req(struct ptp_device *ptp, uint32_t rxBuffer) {
  /* React to peer delay requests no matter what, even if we're not using the
   * peer-to-peer delay mechanism or if we're a slave or master.  Transmit
   * a peer delay response back - we will also transmit a peer delay response
   * followup once this message is on the wire.
   */
  transmit_pdelay_response(ptp, rxBuffer);
};

/* Processes a newly-received PDELAY_RESP packet for the passed instance */
static void process_rx_pdelay_resp(struct ptp_device *ptp, uint32_t rxBuffer) {

  ptp->rcvdPdelayResp = TRUE;
  ptp->rcvdPdelayRespPtr = rxBuffer;

  MDPdelayReq_StateMachine(ptp);
}

/* Processes a newly-received PDELAY_RESP_FUP packet for the passed instance */
static void process_rx_pdelay_resp_fup(struct ptp_device *ptp, uint32_t rxBuffer) {

  ptp->rcvdPdelayRespFollowUp = TRUE;
  ptp->rcvdPdelayRespFollowUpPtr = rxBuffer;

  MDPdelayReq_StateMachine(ptp);
}

static void tx_state_task(unsigned long data);

/* Tasklet function for PTP Rx packets */
static void rx_state_task(unsigned long data) {
  struct ptp_device *ptp = (struct ptp_device *) data;
  uint32_t newRxBuffer;

  /* Make sure any pending Tx operations are completed. Tasklets aren't run in any particular order */
  if (ptp->pendingTxFlags != PTP_TX_BUFFER_NONE)
  {
    tx_state_task(data);
  }

  /* Process all messages received since the last time we ran */
  newRxBuffer = (XIo_In32(REGISTER_ADDRESS(ptp, PTP_RX_REG)) & PTP_RX_BUFFER_MASK);
  while(ptp->lastRxBuffer != newRxBuffer) {
    /* Advance the last buffer circularly around the available Rx buffers */
    ptp->lastRxBuffer = ((ptp->lastRxBuffer + 1) & PTP_RX_BUFFER_MASK);

    /* Determine which message to process */
    switch(get_message_type(ptp, ptp->lastRxBuffer)) {
    case MSG_ANNOUNCE:
      process_rx_announce(ptp, ptp->lastRxBuffer);
      break;

    case MSG_SYNC:
      process_rx_sync(ptp, ptp->lastRxBuffer);
      break;

    case MSG_FUP:
      process_rx_fup(ptp, ptp->lastRxBuffer);
      break;

    case MSG_DELAY_REQ:
      process_rx_delay_req(ptp, ptp->lastRxBuffer);
      break;

    case MSG_DELAY_RESP:
      process_rx_delay_resp(ptp, ptp->lastRxBuffer);
      break;

    case MSG_PDELAY_REQ:
      process_rx_pdelay_req(ptp, ptp->lastRxBuffer);
      break;

    case MSG_PDELAY_RESP:
      process_rx_pdelay_resp(ptp, ptp->lastRxBuffer);
      break;

    case MSG_PDELAY_RESP_FUP:
      process_rx_pdelay_resp_fup(ptp, ptp->lastRxBuffer);
      break;

    default:
      ;
    } /* switch(messageType) */
  }
}

/* Tasklet function for PTP Tx packets */
static void tx_state_task(unsigned long data) {
  struct ptp_device *ptp = (struct ptp_device *) data;
  uint32_t pendingTxFlags;
  uint32_t whichBuffer;
  uint32_t bufferMask;
  unsigned long flags;

  /* A packet has been transmitted; examine the pending flags to to see which one(s).
   * Lock the mutex to avoid a race condition with the Tx IRQ.
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);

  /* Clear the pending flags in the device structure once we've cached them; we're
   * going to "consume" them here.
   */
  pendingTxFlags = ptp->pendingTxFlags;
  ptp->pendingTxFlags = PTP_TX_BUFFER_NONE;
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
        /* A sync message was just transmitted; send a followup message containing the 
         * hardware-timestamped transmit time of the SYNC.
         */
        transmit_fup(ptp);
      } break;
        
      case PTP_TX_DELAY_REQ_BUFFER: {
        /* A delay request message has just been sent; capture and store the 
         * transmission timestamp for later use.
         */
        get_hardware_timestamp(ptp, TRANSMITTED_PACKET, PTP_TX_DELAY_REQ_BUFFER, 
                               &ptp->delayReqTxTimestampTemp);
      } break;
        
      case PTP_TX_PDELAY_REQ_BUFFER: {
        /* A peer delay request message has just been sent; capture and store the
         * transmission timestamp for later use. (Treq1 - our local clock)
         */
        get_local_hardware_timestamp(ptp, TRANSMITTED_PACKET, PTP_TX_PDELAY_REQ_BUFFER, 
                                     &ptp->pdelayReqTxTimestamp);

        ptp->rcvdMDTimestampReceive = TRUE;
        MDPdelayReq_StateMachine(ptp);
      } break;
        
      case PTP_TX_PDELAY_RESP_BUFFER: {
        /* A peer delay response message was just transmitted; send a peer delay 
         * response followup message containing the hardware-timestamped transmit 
         * time of the response.
         */
        transmit_pdelay_response_fup(ptp);
      } break;
        
      default:
        /* A message was sent that requires no follow-up action */
        ;
      } /* case(whichBuffer) */
    } /* if(whichBuffer was transmitted) */
  } /* for(all Tx buffers) */
}

/* Initializes all of the state machines */
void init_state_machines(struct ptp_device *ptp) {
  /* Initialize the timer state machine */
  tasklet_init(&ptp->timerTasklet, &timer_state_task, (unsigned long) ptp);

  ptp->announceCounter     = 0;
  ptp->announceSequenceId  = 0x0000;
  ptp->syncCounter         = 0;
  ptp->syncSequenceId      = 0x0000;
  ptp->syncSequenceIdValid = 0;
  ptp->delayReqCounter     = 0;
  ptp->delayReqSequenceId  = 0x0000;

  /* TODO: check the ethernet port for link-up here to determine if it should be enabled */
  ptp->portEnabled = TRUE;
  ptp->pttPortEnabled = TRUE;

  /* peer delay request state machine initialization */
  ptp->mdPdelayReq_State    = MDPdelayReq_NOT_ENABLED;
  ptp->pdelayReqInterval    = (PDELAY_REQ_INTERVAL / TIMER_TICK_MS);
  ptp->allowedLostResponses = 3;
  ptp->neighborPropDelayThresh = 10000; /* TODO: This number was randomly selected. Is it ok? */

  /* Initialize the Rx state machine, presuming we are a master; set the nominal
   * RTC increment, enabling the counter.
   */
  ptp->presentRole = PTP_MASTER;
  copy_ptp_properties(&ptp->presentMaster, &ptp->properties);
  ptp->announceTimeoutCounter = 0;
  tasklet_init(&ptp->rxTasklet, &rx_state_task, (unsigned long) ptp);
  ptp->lastRxBuffer = (XIo_In32(REGISTER_ADDRESS(ptp, PTP_RX_REG)) & PTP_RX_BUFFER_MASK);
  ptp->syncTimestampsValid       = 0;
  ptp->delayReqTimestampsValid   = 0;
  ptp->neighborPropDelay         = 0;
  ptp->integral                  = 0;
  ptp->derivative                = 0;
  ptp->previousOffset            = 0;
  set_rtc_increment(ptp, &ptp->nominalIncrement);
  printk("PTP master\n");

  /* Initialize the Tx state machine */
  tasklet_init(&ptp->txTasklet, &tx_state_task, (unsigned long) ptp);
}

