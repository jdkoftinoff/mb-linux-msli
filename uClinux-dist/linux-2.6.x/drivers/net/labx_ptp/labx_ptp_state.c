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
  int i;

  /* We behave differently as a master than as a slave */
  switch(ptp->presentRole) {
  case PTP_MASTER:
    {
      for (i=0; i<ptp->numPorts; i++) {
        /* Send ANNOUNCE and SYNC messages at their rate if we're a master */
        ptp->ports[i].announceCounter++;
        if(ptp->ports[i].announceCounter >= (ANNOUNCE_INTERVAL / TIMER_TICK_MS)) {
          ptp->ports[i].announceCounter = 0;
          transmit_announce(ptp, i);
	}

        ptp->ports[i].syncCounter++;
        if(ptp->ports[i].syncCounter >= (SYNC_INTERVAL / TIMER_TICK_MS)) {
          ptp->ports[i].syncCounter = 0;
          transmit_sync(ptp, i);
	}
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

        /* Update stats */
        for (i=0; i<ptp->numPorts; i++) {
          ptp->ports[i].stats.announceReceiptTimeoutCount++;
        }
      }
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();

      /* Transmit an ANNOUNCE immediately to speed things along if we've switched our
       * port to the master state.
       */
      if(ptp->presentRole == PTP_MASTER) {
        printk("PTP master\n");
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
	  for (i=0; i<ptp->numPorts; i++) {
            /* Increment the delay request counter and see if it's time to
             * send one to the master.
             */
            if(++ptp->ports[i].delayReqCounter >= (DELAY_REQ_INTERVAL / TIMER_TICK_MS)) {
              ptp->ports[i].delayReqCounter = 0;
              transmit_delay_request(ptp, i);
	    }
          }
        }
      } /* if(still a slave) */
    }
    break;

  default:
    /* "Passive"; do nothing */
    break;
  }

  for (i=0; i<ptp->numPorts; i++)
  {
    LinkDelaySyncIntervalSetting_StateMachine(ptp, i);

    /* Regardless of whether we are a master or slave, increment the peer delay request
     * counter and see if it's time to send one to our link peer.
     */
    ptp->ports[i].pdelayIntervalTimer++;
    MDPdelayReq_StateMachine(ptp, i);
  }
}

/* Runs the Best Master Clock Algorithm (BMCA) between the passed master and challenger.
 * Returns nonzero if the challenger should become the new master.
 *
 * NOTE - This implementation does not (yet) handle multiple ports and the accompanying
 *        logic to determine which port is better by topology with respect to the master.
 */
static BmcaResult bmca_comparison(PtpProperties *presentMaster,
  PtpPortProperties *presentMasterPort, PtpProperties *challenger,
  PtpPortProperties *challengerPort) {

  PtpClockQuality *challengerQuality = &challenger->grandmasterClockQuality;
  PtpClockQuality *presentQuality = &challenger->grandmasterClockQuality;

  int32_t identityComparison;

#if 0
  printk("BMCA: CHAL: P1 %d, CC %d, CA %d, LV %d, P2 %d, GMID %02X%02X%02X%02X%02X%02X%02X%02X, SR %d, PN %d\n",
    challenger->grandmasterPriority1, challengerQuality->clockClass, challengerQuality->clockAccuracy,
    challengerQuality->offsetScaledLogVariance, challenger->grandmasterPriority2,
    challenger->grandmasterIdentity[0], challenger->grandmasterIdentity[1], challenger->grandmasterIdentity[2],
    challenger->grandmasterIdentity[3], challenger->grandmasterIdentity[4], challenger->grandmasterIdentity[5],
    challenger->grandmasterIdentity[6], challenger->grandmasterIdentity[7], challengerPort->stepsRemoved,
    challengerPort->portNumber);
  printk("BMCA: PRES: P1 %d, CC %d, CA %d, LV %d, P2 %d, GMID %02X%02X%02X%02X%02X%02X%02X%02X, SR %d, PN %d\n",
    presentMaster->grandmasterPriority1, presentQuality->clockClass, presentQuality->clockAccuracy,
    presentQuality->offsetScaledLogVariance, presentMaster->grandmasterPriority2,
    presentMaster->grandmasterIdentity[0], presentMaster->grandmasterIdentity[1], presentMaster->grandmasterIdentity[2],
    presentMaster->grandmasterIdentity[3], presentMaster->grandmasterIdentity[4], presentMaster->grandmasterIdentity[5],
    presentMaster->grandmasterIdentity[6], presentMaster->grandmasterIdentity[7], presentMasterPort->stepsRemoved,
    presentMasterPort->portNumber);
#endif

  /* Begin by comparing grandmaster priority 1; lower value is higher priority */
  if(challenger->grandmasterPriority1 < presentMaster->grandmasterPriority1) {
    return REPLACE_PRESENT_MASTER;
  } else if(challenger->grandmasterPriority1 > presentMaster->grandmasterPriority1) {
    return RETAIN_PRESENT_MASTER;
  }

  /* Priority 1 identical, compare clock quality - again, lower is "better". */
  if(challengerQuality->clockClass < presentQuality->clockClass) {
    return REPLACE_PRESENT_MASTER;
  } else if(challengerQuality->clockClass > presentQuality->clockClass) {
    return RETAIN_PRESENT_MASTER;
  }

  /* Clock class equal, go to accuracy */
  if(challengerQuality->clockAccuracy < presentQuality->clockAccuracy) {
    return REPLACE_PRESENT_MASTER;
  } else if(challengerQuality->clockAccuracy > presentQuality->clockAccuracy) {
    return RETAIN_PRESENT_MASTER;
  }

  /* Accuracy identical, go to offset scaled log variance */
  if(challengerQuality->offsetScaledLogVariance < presentQuality->offsetScaledLogVariance) {
    return REPLACE_PRESENT_MASTER;
  } else if(challengerQuality->offsetScaledLogVariance > presentQuality->offsetScaledLogVariance) {
    return RETAIN_PRESENT_MASTER;
  }

  /* Log variance identical, compare priority 2 - again, lower is "better". */
  if(challenger->grandmasterPriority2 < presentMaster->grandmasterPriority2) {
    return REPLACE_PRESENT_MASTER;
  } else if(challenger->grandmasterPriority2 > presentMaster->grandmasterPriority2) {
    return RETAIN_PRESENT_MASTER;
  }

  /* Clock settings completely identical, compare MAC addresses as a tie-breaker */
  identityComparison =
    compare_clock_identity(challenger->grandmasterIdentity, presentMaster->grandmasterIdentity);
  if (identityComparison < 0) {
    /* The new announce message has a lower MAC address, it becomes the master */
    return REPLACE_PRESENT_MASTER;
  } else if (identityComparison > 0) {
    return RETAIN_PRESENT_MASTER;
  }

  /* If we are still tied here, then we are down to picking between our local ports */
  /* First check hops to master - lower is better */
  if (challengerPort->stepsRemoved < presentMasterPort->stepsRemoved) {
    return REPLACE_PRESENT_MASTER;
  } else if (challengerPort->stepsRemoved > presentMasterPort->stepsRemoved) {
    return RETAIN_PRESENT_MASTER;
  }

  /* Hops identical. Select lowest port number */
  if (challengerPort->portNumber < presentMasterPort->portNumber) {
    return REPLACE_PRESENT_MASTER;
  } else if (challengerPort->portNumber > presentMasterPort->portNumber) {
    return RETAIN_PRESENT_MASTER;
  }
  
  /* This is the same master on the same port. No change. */
  return IS_PRESENT_MASTER;
}

/* Processes a newly-received ANNOUNCE packet for the passed instance */
static void process_rx_announce(struct ptp_device *ptp, uint32_t port, uint32_t rxBuffer) {
  PtpProperties properties;
  PtpPortProperties portProperties;
  unsigned long flags;
  uint32_t byteIndex;

  ptp->ports[port].stats.rxAnnounceCount++;

  /* Extract the properties of the port which sent the message, and compare 
   * them to those of the present master to determine what to do.
   */
  extract_announce(ptp, port, rxBuffer, &properties, &portProperties);
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  switch(bmca_comparison(&ptp->presentMaster, &ptp->presentMasterPort, &properties, &portProperties)) {
  case IS_PRESENT_MASTER: {
    //printk("BMCA: Is present master.\n");
    /* A message from our fearless leader; reset its timeout counter */
    ptp->announceTimeoutCounter = 0;
  } break;
    
  case REPLACE_PRESENT_MASTER: {
    //printk("BMCA: Replace master.\n");
    /* Replace the present master's properties, and ensure that we're a slave.
     */
    ptp->presentRole = PTP_SLAVE;
    copy_ptp_properties(&ptp->presentMaster, &properties);
    copy_ptp_port_properties(&ptp->presentMasterPort, &portProperties);
    ptp->announceTimeoutCounter = 0;
    
    /* Invalidate all the slave flags */
    ptp->ports[port].syncTimestampsValid       = 0;
    ptp->ports[port].delayReqTimestampsValid   = 0;
    ptp->ports[port].syncSequenceIdValid       = 0;
    ptp->ports[port].delayReqCounter           = 0;
    ptp->ports[port].delayReqSequenceId        = 0x0000;

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
    //printk("BMCA: Keep present master.\n");
    /* Retain the present master, but do not reset its timeout counter */
    break;
  } /* switch(BMCA comparison) */
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}

/* Processes a newly-received SYNC packet for the passed instance */
static void process_rx_sync(struct ptp_device *ptp, uint32_t port, uint32_t rxBuffer) {
  unsigned long flags;
  uint8_t rxMacAddress[MAC_ADDRESS_BYTES];

  ptp->ports[port].stats.rxSyncCount++;

  /* Only process this packet if we are a slave and it has come from the master
   * we're presently respecting.  If we're the master, spanning tree should prevent
   * us from ever seeing our own SYNC packets, but better safe than sorry.
   */
  get_rx_mac_address(ptp, port, rxBuffer, rxMacAddress);
  if((ptp->presentRole == PTP_SLAVE) && 
     (compare_mac_addresses(rxMacAddress, ptp->presentMasterPort.sourceMacAddress) == 0) &&
     (ptp->presentMasterPort.portNumber == (port + 1))) {
    PtpTime tempTimestamp;
    PtpTime correctionField;
    PtpTime correctedTimestamp;

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
    ptp->ports[port].syncSequenceId = get_sequence_id(ptp, port, RECEIVED_PACKET, rxBuffer);
    ptp->ports[port].syncSequenceIdValid = 1;
    spin_unlock_irqrestore(&ptp->mutex, flags);
    preempt_enable();
  }
}

/* Processes a newly-received FUP packet for the passed instance */
static void process_rx_fup(struct ptp_device *ptp, uint32_t port, uint32_t rxBuffer) {
  unsigned long flags;
  uint8_t rxMacAddress[MAC_ADDRESS_BYTES];

  ptp->ports[port].stats.rxFollowupCount++;

  /* Make certain of the following:
   * - We are a slave
   * - This is from our master
   * - The sequence ID matches the last valid SYNC message
   */
  get_rx_mac_address(ptp, port, rxBuffer, rxMacAddress);
  if((ptp->presentRole == PTP_SLAVE) && 
     (compare_mac_addresses(rxMacAddress, ptp->presentMasterPort.sourceMacAddress) == 0) &&
     (ptp->presentMasterPort.portNumber == (port + 1)) &&
     ptp->ports[port].syncSequenceIdValid && 
     (get_sequence_id(ptp, port, RECEIVED_PACKET, rxBuffer) == ptp->ports[port].syncSequenceId)) {
    PtpTime syncTxTimestamp;
    PtpTime correctionField;
    PtpTime correctedTimestamp;
    PtpTime difference;
    PtpTime absDifference;

    /* Everything matches; obtain the preciseOriginTimestamp from the packet.
     * This is the time at which the master captured its transmit of the preceding
     * SYNC, which we also timestamped reception for.
     */
    get_timestamp(ptp, port, RECEIVED_PACKET, rxBuffer, &syncTxTimestamp);

    /* Correct the Tx timestamp with the received correction field */
    get_correction_field(ptp, port, rxBuffer, &correctionField);
    timestamp_sum(&syncTxTimestamp, &correctionField, &correctedTimestamp);

    /* Compare the timestamps; if the one-way offset plus delay is greater than
     * one second, we need to reset our RTC before beginning to servo.  Regardless
     * of what we do, we need to invalidate the sync sequence ID, it's been "used up."
     */
    ptp->ports[port].syncSequenceIdValid = 0;
    timestamp_difference(&ptp->ports[port].syncRxTimestampTemp, &correctedTimestamp, &difference);
    timestamp_abs(&difference, &absDifference);
    if((absDifference.secondsUpper > 0) || (absDifference.secondsLower > 0)) {
      /* Reset the time using the uncorrected timestamp, and invalidate the SYNC */
      printk("Resetting time!\n");
      set_rtc_time(ptp, &syncTxTimestamp);
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
      timestamp_copy(&ptp->ports[port].syncRxTimestamp, &ptp->ports[port].syncRxTimestampTemp);
      timestamp_copy(&ptp->ports[port].syncTxTimestamp, &correctedTimestamp);
      ptp->ports[port].syncTimestampsValid = 1;
      rtc_update_servo(ptp, port);
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
    }
  }
}

/* Processes a newly-received DELAY_REQ packet for the passed instance */
static void process_rx_delay_req(struct ptp_device *ptp, uint32_t port, uint32_t rxBuffer) {
  /* Only react to these messages if we are the master */
  if(ptp->presentRole == PTP_MASTER) {
    /* React to the reception of a delay request by simply transmitting a delay
     * response back to the slave.
     */
    transmit_delay_response(ptp, port, rxBuffer);
  }
};

/* Processes a newly-received DELAY_RESP packet for the passed instance */
static void process_rx_delay_resp(struct ptp_device *ptp, uint32_t port, uint32_t rxBuffer) {
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
  get_rx_mac_address(ptp, port, rxBuffer, rxMacAddress);
  get_rx_requesting_port_id(ptp, port, rxBuffer, rxRequestingPortId);
  get_source_port_id(ptp, port, TRANSMITTED_PACKET, PTP_TX_PDELAY_REQ_BUFFER, 
                     txRequestingPortId);
  if((ptp->presentRole == PTP_SLAVE) && 
     (compare_mac_addresses(rxMacAddress, ptp->presentMasterPort.sourceMacAddress) == 0) &&
     (compare_port_ids(rxRequestingPortId, txRequestingPortId) == 0) &&
     (get_sequence_id(ptp, port, RECEIVED_PACKET, rxBuffer) == 
      get_sequence_id(ptp, port, TRANSMITTED_PACKET, PTP_TX_DELAY_REQ_BUFFER))) {
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
      timestamp_copy(&ptp->ports[port].delayReqRxTimestamp, &delayReqRxTimestamp);
      ptp->ports[port].delayReqTimestampsValid = 1;
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
    }
  }
}

/* Processes a newly-received PDELAY_REQ packet for the passed instance */
static void process_rx_pdelay_req(struct ptp_device *ptp, uint32_t port, uint32_t rxBuffer) {

  ptp->ports[port].stats.rxPDelayRequestCount++;

  /* React to peer delay requests no matter what, even if we're not using the
   * peer-to-peer delay mechanism or if we're a slave or master.  Transmit
   * a peer delay response back - we will also transmit a peer delay response
   * followup once this message is on the wire.
   */
  transmit_pdelay_response(ptp, port, rxBuffer);
};

/* Processes a newly-received PDELAY_RESP packet for the passed instance */
static void process_rx_pdelay_resp(struct ptp_device *ptp, uint32_t port, uint32_t rxBuffer) {

  ptp->ports[port].stats.rxPDelayResponseCount++;

  ptp->ports[port].rcvdPdelayResp = TRUE;
  ptp->ports[port].rcvdPdelayRespPtr = rxBuffer;

  MDPdelayReq_StateMachine(ptp, port);
}

/* Processes a newly-received PDELAY_RESP_FUP packet for the passed instance */
static void process_rx_pdelay_resp_fup(struct ptp_device *ptp, uint32_t port, uint32_t rxBuffer) {

  ptp->ports[port].stats.rxPDelayResponseFollowupCount++;

  ptp->ports[port].rcvdPdelayRespFollowUp = TRUE;
  ptp->ports[port].rcvdPdelayRespFollowUpPtr = rxBuffer;

  MDPdelayReq_StateMachine(ptp, port);
}

static void tx_state_task(unsigned long data);

/* Tasklet function for PTP Rx packets */
static void rx_state_task(unsigned long data) {
  struct ptp_device *ptp = (struct ptp_device *) data;
  uint32_t newRxBuffer;
  int i;

  for(i=0; i<ptp->numPorts; i++) {
    /* Make sure any pending Tx operations are completed. Tasklets aren't run in any particular order */
    if (ptp->ports[i].pendingTxFlags != PTP_TX_BUFFER_NONE)
    {
      tx_state_task(data);
    }

    /* Process all messages received since the last time we ran */
    newRxBuffer = (XIo_In32(REGISTER_ADDRESS(ptp, i, PTP_RX_REG)) & PTP_RX_BUFFER_MASK);
    while(ptp->ports[i].lastRxBuffer != newRxBuffer) {
      /* Advance the last buffer circularly around the available Rx buffers */
      ptp->ports[i].lastRxBuffer = ((ptp->ports[i].lastRxBuffer + 1) & PTP_RX_BUFFER_MASK);

      /* Determine which message to process */
      switch(get_message_type(ptp, i, ptp->ports[i].lastRxBuffer)) {
      case MSG_ANNOUNCE:
        process_rx_announce(ptp, i, ptp->ports[i].lastRxBuffer);
        break;

      case MSG_SYNC:
        process_rx_sync(ptp, i, ptp->ports[i].lastRxBuffer);
        break;

      case MSG_FUP:
        process_rx_fup(ptp, i, ptp->ports[i].lastRxBuffer);
        break;

      case MSG_DELAY_REQ:
        process_rx_delay_req(ptp, i, ptp->ports[i].lastRxBuffer);
        break;

      case MSG_DELAY_RESP:
        process_rx_delay_resp(ptp, i, ptp->ports[i].lastRxBuffer);
        break;

      case MSG_PDELAY_REQ:
        process_rx_pdelay_req(ptp, i, ptp->ports[i].lastRxBuffer);
        break;

      case MSG_PDELAY_RESP:
        process_rx_pdelay_resp(ptp, i, ptp->ports[i].lastRxBuffer);
        break;

      case MSG_PDELAY_RESP_FUP:
        process_rx_pdelay_resp_fup(ptp, i, ptp->ports[i].lastRxBuffer);
        break;

      default:
        break;
      } /* switch(messageType) */
    }
  }
}

/* Tasklet function for PTP Tx packets */
static void tx_state_task(unsigned long data) {
  struct ptp_device *ptp = (struct ptp_device *) data;
  uint32_t pendingTxFlags;
  uint32_t whichBuffer;
  uint32_t bufferMask;
  unsigned long flags;
  int i;

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
          /* A sync message was just transmitted; send a followup message containing the 
           * hardware-timestamped transmit time of the SYNC.
           */
          transmit_fup(ptp, i);
        } break;
        
        case PTP_TX_DELAY_REQ_BUFFER: {
          /* A delay request message has just been sent; capture and store the 
           * transmission timestamp for later use.
           */
          get_hardware_timestamp(ptp, i, TRANSMITTED_PACKET, PTP_TX_DELAY_REQ_BUFFER, 
                                 &ptp->ports[i].delayReqTxTimestampTemp);
        } break;
        
        case PTP_TX_PDELAY_REQ_BUFFER: {
          /* A peer delay request message has just been sent; capture and store the
           * transmission timestamp for later use. (Treq1 - our local clock)
           */
          get_local_hardware_timestamp(ptp, i, TRANSMITTED_PACKET, PTP_TX_PDELAY_REQ_BUFFER, 
                                       &ptp->ports[i].pdelayReqTxTimestamp);

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

/* Initializes all of the state machines */
void init_state_machines(struct ptp_device *ptp) {
  int i;

  /* Initialize the timer state machine */
  tasklet_init(&ptp->timerTasklet, &timer_state_task, (unsigned long) ptp);

  for(i=0; i<ptp->numPorts; i++) {
    ptp->ports[i].announceCounter     = 0;
    ptp->ports[i].announceSequenceId  = 0x0000;
    ptp->ports[i].syncCounter         = 0;
    ptp->ports[i].syncSequenceId      = 0x0000;
    ptp->ports[i].syncSequenceIdValid = 0;
    ptp->ports[i].delayReqCounter     = 0;
    ptp->ports[i].delayReqSequenceId  = 0x0000;

    /* TODO: check the ethernet port for link-up here to determine if it should be enabled */
    ptp->ports[i].portEnabled = TRUE;
    ptp->ports[i].pttPortEnabled = TRUE;

    /* peer delay request state machine initialization */
    ptp->ports[i].mdPdelayReq_State    = MDPdelayReq_NOT_ENABLED;
    ptp->ports[i].pdelayReqInterval    = (PDELAY_REQ_INTERVAL / TIMER_TICK_MS);
    ptp->ports[i].allowedLostResponses = 3;
    ptp->ports[i].neighborPropDelayThresh = 10000; /* TODO: This number was randomly selected. Is it ok? */
  }

  /* Initialize the Rx state machine, presuming we are a master; set the nominal
   * RTC increment, enabling the counter.
   */
  ptp->presentRole = PTP_MASTER;
  copy_ptp_properties(&ptp->presentMaster, &ptp->properties);
  copy_ptp_port_properties(&ptp->presentMasterPort, &ptp->ports[0].portProperties);
  ptp->announceTimeoutCounter = 0;
  tasklet_init(&ptp->rxTasklet, &rx_state_task, (unsigned long) ptp);

  for(i=0; i<ptp->numPorts; i++) {
    ptp->ports[i].lastRxBuffer = (XIo_In32(REGISTER_ADDRESS(ptp, i, PTP_RX_REG)) & PTP_RX_BUFFER_MASK);
    ptp->ports[i].syncTimestampsValid       = 0;
    ptp->ports[i].delayReqTimestampsValid   = 0;
    ptp->ports[i].neighborPropDelay         = 0;
  }

  ptp->integral                  = 0;
  ptp->derivative                = 0;
  ptp->previousOffset            = 0;
  set_rtc_increment(ptp, &ptp->nominalIncrement);

  printk("PTP master\n");

  /* Initialize the Tx state machine */
  tasklet_init(&ptp->txTasklet, &tx_state_task, (unsigned long) ptp);
}

