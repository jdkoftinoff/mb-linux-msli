/*
 *  linux/drivers/net/labx_ptp_pdelay_state.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
 *  PTP peer delay state machine processing
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
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
uint8_t * get_output_buffer(struct ptp_device *ptp,uint32_t port,uint32_t bufType);

/* Define this to get some extra debug on path delay messages */
/* #define PATH_DELAY_DEBUG */

static void computePdelayRateRatio(struct ptp_device *ptp, uint32_t port)
{
  if (ptp->ports[port].initPdelayRespReceived == FALSE)
  {
    /* Capture the initial PDELAY response */
    ptp->ports[port].initPdelayRespReceived = TRUE;
    ptp->ports[port].pdelayRespTxTimestampI = ptp->ports[port].pdelayRespTxTimestamp;
    ptp->ports[port].pdelayRespRxTimestampI = ptp->ports[port].pdelayRespRxTimestamp;
  }
  else
  {
    PtpTime difference;
    PtpTime difference2;
    uint64_t nsResponder;
    uint64_t nsRequester;
    uint64_t rateRatio;
    int shift;

    timestamp_difference(&ptp->ports[port].pdelayRespTxTimestamp, &ptp->ports[port].pdelayRespTxTimestampI, &difference);
    timestamp_difference(&ptp->ports[port].pdelayRespRxTimestamp, &ptp->ports[port].pdelayRespRxTimestampI, &difference2);

    /* The raw differences have been computed; sanity-check the peer delay timestamps; if the 
     * initial Tx or Rx timestamp is later than the present one, the initial ones are bogus and
     * must be replaced.
     */
    if((difference.secondsUpper & 0x80000000) |
       (difference2.secondsUpper & 0x80000000)) {
      ptp->ports[port].initPdelayRespReceived = FALSE;
      ptp->ports[port].neighborRateRatioValid = FALSE;
    } else {
      nsResponder = ((uint64_t)difference.secondsLower) * 1000000000ULL + (uint64_t)difference.nanoseconds;
      nsRequester = ((uint64_t)difference2.secondsLower) * 1000000000ULL + (uint64_t)difference2.nanoseconds;

      for (shift = 0; shift < 31; shift++)
        {
          if (nsResponder & (1ULL<<(63-shift))) break;
        }

      if ((nsRequester >> (31-shift)) != 0) {
        rateRatio = (nsResponder << shift) / (nsRequester >> (31-shift));
        ptp->ports[port].neighborRateRatio = (uint32_t)rateRatio;

        ptp->ports[port].neighborRateRatioValid = TRUE;
      }

#ifdef PATH_DELAY_DEBUG
      printk("Responder delta: %08X%08X.%08X (%llu ns)\n", difference.secondsUpper,
             difference.secondsLower, difference.nanoseconds, nsResponder);
      printk("Requester delta: %08X%08X.%08X (%llu ns)\n", difference2.secondsUpper,
             difference2.secondsLower, difference2.nanoseconds, nsRequester);
      printk("Rate ratio: %08X (shift %d)\n", ptp->ports[port].neighborRateRatio, shift);
#endif
    } /* if(differences are sane) */
  }
}

static void computePropTime(struct ptp_device *ptp, uint32_t port)
{
  if (ptp->ports[port].neighborRateRatioValid)
  {
    PtpTime difference;
    PtpTime difference2;
    uint64_t nsResponder;
    uint64_t nsRequester;

    timestamp_difference(&ptp->ports[port].pdelayRespTxTimestamp, &ptp->ports[port].pdelayReqRxTimestamp, &difference);
    timestamp_difference(&ptp->ports[port].pdelayRespRxTimestamp, &ptp->ports[port].pdelayReqTxTimestamp, &difference2);

    nsResponder = ((uint64_t)difference.secondsLower) * 1000000000ULL + (uint64_t)difference.nanoseconds;
    nsRequester = ((uint64_t)difference2.secondsLower) * 1000000000ULL + (uint64_t)difference2.nanoseconds;

    ptp->ports[port].neighborPropDelay = (((((uint64_t)ptp->ports[port].neighborRateRatio) * nsRequester) >> 31) - nsResponder) >> 1;

#ifdef PATH_DELAY_DEBUG
    printk("Responder delta: %08X%08X.%08X (%llu ns)\n", difference.secondsUpper,
      difference.secondsLower, difference.nanoseconds, nsResponder);
    printk("Requester delta: %08X%08X.%08X (%llu ns)\n", difference2.secondsUpper,
      difference2.secondsLower, difference2.nanoseconds, nsRequester);
    printk("Prop Delay: %08X\n", ptp->ports[port].neighborPropDelay);
#endif
  }
}

/* 802.1AS MDPdelayReq state machine (11.2.15.3) entry actions */
static void MDPdelayReq_StateMachine_SetState(struct ptp_device *ptp, uint32_t port, MDPdelayReq_State_t newState)
{
  uint8_t rxSourcePortId[PORT_ID_BYTES];

#ifdef PATH_DELAY_DEBUG
  printk("MDPdelayReq: Set State %d (port index %d)\n", newState, port);
#endif

  ptp->ports[port].mdPdelayReq_State = newState;

  switch (newState)
  {
    default:
    case MDPdelayReq_NOT_ENABLED:
      break;

    case MDPdelayReq_INITIAL_SEND_PDELAY_REQ:
      ptp->ports[port].initPdelayRespReceived = FALSE;
      ptp->ports[port].neighborRateRatio = 0x80000000; // 1.0 fixed point 1.31
      ptp->ports[port].rcvdMDTimestampReceive = FALSE;
      ptp->ports[port].pdelayReqSequenceId = 0x0000; // TODO: spec says random()
      ptp->ports[port].rcvdPdelayResp = FALSE;
      ptp->ports[port].rcvdPdelayRespFollowUp = FALSE;
      transmit_pdelay_request(ptp, port);
      ptp->ports[port].pdelayIntervalTimer = 0; // currentTime ("now" is zero ticks)
      ptp->ports[port].lostResponses = 0;
      ptp->ports[port].isMeasuringDelay = FALSE;
      ptp->ports[port].asCapable = FALSE;
      ptp->ports[port].neighborRateRatioValid = FALSE;
      break;

    case MDPdelayReq_RESET:
      ptp->ports[port].initPdelayRespReceived = FALSE;
      ptp->ports[port].rcvdPdelayResp = FALSE;
      ptp->ports[port].rcvdPdelayRespFollowUp = FALSE;
      if (ptp->ports[port].lostResponses <= ptp->ports[port].allowedLostResponses)
      {
        ptp->ports[port].lostResponses++;
      }
      else
      {
        ptp->ports[port].isMeasuringDelay = FALSE;
        ptp->ports[port].asCapable = FALSE;
      }
      break;

    case MDPdelayReq_SEND_PDELAY_REQ:
      ptp->ports[port].pdelayReqSequenceId++;
      transmit_pdelay_request(ptp, port);
      ptp->ports[port].pdelayIntervalTimer = 0; // currentTime ("now" is zero ticks)
      break;

    case MDPdelayReq_WAITING_FOR_PDELAY_RESP:
      ptp->ports[port].rcvdMDTimestampReceive = FALSE;
      break;

    case MDPdelayReq_WAITING_FOR_PDELAY_RESP_FOLLOW_UP:
      ptp->ports[port].rcvdPdelayResp = FALSE;

      /* Obtain the peer delay request receive timestamp that our peer has just sent.
       * (Trsp2 - responder local clock) */
      get_timestamp(ptp, port, RECEIVED_PACKET, ptp->ports[port].rcvdPdelayRespPtr,
        &ptp->ports[port].pdelayReqRxTimestamp);

      /* Capture the hardware timestamp at which we received this packet, and hang on to 
       * it for delay and rate calculation. (Trsp4 - our local clock) */
      get_local_hardware_timestamp(ptp, port, RECEIVED_PACKET,
        ptp->ports[port].rcvdPdelayRespPtr, &ptp->ports[port].pdelayRespRxTimestamp);
      break;

    case MDPdelayReq_WAITING_FOR_PDELAY_INTERVAL_TIMER:
      ptp->ports[port].rcvdPdelayRespFollowUp = FALSE;
 
      /* Obtain the follow up timestamp for delay and rate calculation.
       * (Trsp3 - responder local clock) */
      get_timestamp(ptp, port, RECEIVED_PACKET, ptp->ports[port].rcvdPdelayRespFollowUpPtr,
        &ptp->ports[port].pdelayRespTxTimestamp);

      if (ptp->ports[port].computeNeighborRateRatio)
      {
        computePdelayRateRatio(ptp, port);
      }
      if (ptp->ports[port].computeNeighborPropDelay)
      {
        computePropTime(ptp, port);
      }
      ptp->ports[port].lostResponses = 0;
      ptp->ports[port].isMeasuringDelay = TRUE;
  
      get_source_port_id(ptp, port, RECEIVED_PACKET, ptp->ports[port].rcvdPdelayRespPtr, rxSourcePortId);

#ifdef PATH_DELAY_DEBUG
      {
        int i;
        printk("AS CHECK: pd %d, pdt %d, pidc %d, nrrv %d\n", ptp->ports[port].neighborPropDelay,
          ptp->ports[port].neighborPropDelayThresh, compare_clock_identity(rxSourcePortId, ptp->properties.grandmasterIdentity),
          ptp->ports[port].neighborRateRatioValid);

        printk("AS CHECK: rxSourcePortID:");
        for (i=0; i<PORT_ID_BYTES; i++) printk("%02X", rxSourcePortId[i]);
        printk("\n");
        printk("AS CHECK: thisClock:     ");
        for (i=0; i<PTP_CLOCK_IDENTITY_CHARS; i++) printk("%02X", ptp->properties.grandmasterIdentity[i]);
        printk("\n");
      }
#endif

      /* AS capable if the delay is low enough, the pdelay response is not from us, and we have a valid ratio */
      if ((ptp->ports[port].neighborPropDelay <= ptp->ports[port].neighborPropDelayThresh) &&
          (compare_clock_identity(rxSourcePortId, ptp->properties.grandmasterIdentity) != 0) &&
          ptp->ports[port].neighborRateRatioValid)
      {
        ptp->ports[port].asCapable = TRUE;
      }
      else
      {
        ptp->ports[port].asCapable = FALSE;
      }
      break;
  } 
}

/* 802.1AS MDPdelayReq state machine (11.2.15.3) transitions */
void MDPdelayReq_StateMachine(struct ptp_device *ptp, uint32_t port)
{
  if(ptp->properties.delayMechanism != PTP_DELAY_MECHANISM_P2P) {
    /* The PDELAY state machine should only be active in P2P mode */
    return;
  }

//  printk("PTP IDX %d, PE %d, PTTE %d, PDIT %d, PDRI %d\n", port, ptp->ports[port].portEnabled,
//    ptp->ports[port].pttPortEnabled, ptp->ports[port].pdelayIntervalTimer, ptp->ports[port].pdelayReqInterval);

  if (!ptp->ports[port].portEnabled || !ptp->ports[port].pttPortEnabled)
  {
    if (ptp->ports[port].mdPdelayReq_State != MDPdelayReq_NOT_ENABLED)
    {
      /* Disabling the port immediately forces the state machine into the disabled state */
      MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_NOT_ENABLED);
    }
  }
  else
  {
    uint8_t rxRequestingPortId[PORT_ID_BYTES];
    uint8_t txRequestingPortId[PORT_ID_BYTES];
    uint32_t rxSequenceId = 0;
    uint32_t txSequenceId = 0;
    uint8_t rxFUPRequestingPortId[PORT_ID_BYTES];
    uint8_t txFUPRequestingPortId[PORT_ID_BYTES];
    uint32_t rxFUPSequenceId = 0;
    uint32_t txFUPSequenceId = 0;
    MDPdelayReq_State_t prevState;
    uint8_t *txBuffer;

    memset(rxRequestingPortId, 0, PORT_ID_BYTES);
    memset(txRequestingPortId, 0, PORT_ID_BYTES);
    memset(rxFUPRequestingPortId, 0, PORT_ID_BYTES);
    memset(txFUPRequestingPortId, 0, PORT_ID_BYTES);

    /* Grab some inforomation needed for comparisons if we got a PDelay Response */
    if (ptp->ports[port].rcvdPdelayResp)
    {
      get_rx_requesting_port_id(ptp, port, ptp->ports[port].rcvdPdelayRespPtr, rxRequestingPortId);
      txBuffer = get_output_buffer(ptp,port,PTP_TX_PDELAY_REQ_BUFFER);
      get_source_port_id(ptp, port, TRANSMITTED_PACKET, txBuffer, txRequestingPortId);
      rxSequenceId = get_sequence_id(ptp, port, RECEIVED_PACKET, ptp->ports[port].rcvdPdelayRespPtr);
      txSequenceId = get_sequence_id(ptp, port, TRANSMITTED_PACKET, txBuffer);
    }
    if (ptp->ports[port].rcvdPdelayRespFollowUp)
    {
      get_rx_requesting_port_id(ptp, port, ptp->ports[port].rcvdPdelayRespFollowUpPtr, rxFUPRequestingPortId);
      txBuffer = get_output_buffer(ptp,port,PTP_TX_PDELAY_REQ_BUFFER);
      get_source_port_id(ptp, port, TRANSMITTED_PACKET, txBuffer, txFUPRequestingPortId);
      rxFUPSequenceId = get_sequence_id(ptp, port, RECEIVED_PACKET, ptp->ports[port].rcvdPdelayRespFollowUpPtr);
      txFUPSequenceId = get_sequence_id(ptp, port, TRANSMITTED_PACKET, txBuffer);
    }

    do
    {
      prevState = ptp->ports[port].mdPdelayReq_State;

      switch (ptp->ports[port].mdPdelayReq_State)
      {
        default:
        case MDPdelayReq_NOT_ENABLED:
          if (ptp->ports[port].portEnabled && ptp->ports[port].pttPortEnabled)
          {
#ifdef PATH_DELAY_DEBUG
            printk("Port index %d enabled\n", port);
#endif

            /* Port (and timesync on it) became enabled */
            MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_INITIAL_SEND_PDELAY_REQ);
          }
          else
          {
            /* Don't time when this port is not enabled */
            ptp->ports[port].pdelayIntervalTimer = 0;
          }
          break;

        case MDPdelayReq_RESET:
#ifdef PATH_DELAY_DEBUG
          printk("Resetting port index %d\n", port);
#endif
          MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_SEND_PDELAY_REQ);
          break;

        case MDPdelayReq_INITIAL_SEND_PDELAY_REQ:
        case MDPdelayReq_SEND_PDELAY_REQ:
          if (ptp->ports[port].rcvdMDTimestampReceive)
          {
#ifdef PATH_DELAY_DEBUG
            printk("PDelay Request Tx Timestamp available (port index %d).\n", port);
#endif

            /* The transmit timestamp for the request is available */
            MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_WAITING_FOR_PDELAY_RESP);
          }
          else if (ptp->ports[port].pdelayIntervalTimer >= ptp->ports[port].pdelayReqInterval)
          {
            /* We didn't see a timestamp for some reason (this can happen on startup sometimes) */
            MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_RESET);
          }
          break;

        case MDPdelayReq_WAITING_FOR_PDELAY_RESP:
          if ((ptp->ports[port].pdelayIntervalTimer >= ptp->ports[port].pdelayReqInterval) ||
              (ptp->ports[port].rcvdPdelayResp &&
               ((compare_port_ids(rxRequestingPortId, txRequestingPortId) != 0) ||
                (rxSequenceId != txSequenceId))))
          {
#ifdef PATH_DELAY_DEBUG
            int i;
            printk("Resetting %d: intervalTimer %d, reqInterval %d, rcvdPdelayResp %d, rcvdPdelayRespPtr %d, rxSequence %d, txSequence %d\n",
              port, ptp->ports[port].pdelayIntervalTimer, ptp->ports[port].pdelayReqInterval, ptp->ports[port].rcvdPdelayResp,
              ptp->ports[port].rcvdPdelayRespPtr, rxSequenceId, txSequenceId);
            printk("rxRequestingPortID:");
            for (i=0; i<PORT_ID_BYTES; i++) printk("%02X", rxRequestingPortId[i]);
            printk("\n");
            printk("txRequestingPortID:");
            for (i=0; i<PORT_ID_BYTES; i++) printk("%02X", txRequestingPortId[i]);
            printk("\n");
#endif
 
            /* Timeout or a non-matching response was received */
            MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_RESET);
          }
          else if (ptp->ports[port].rcvdPdelayResp &&
                   (rxSequenceId == txSequenceId) &&
                   (compare_port_ids(rxRequestingPortId, txRequestingPortId) == 0))
          {
            /* A matching response was received */
            MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_WAITING_FOR_PDELAY_RESP_FOLLOW_UP);
          }
          break;

        case MDPdelayReq_WAITING_FOR_PDELAY_RESP_FOLLOW_UP:
          if ((ptp->ports[port].pdelayIntervalTimer >= ptp->ports[port].pdelayReqInterval) ||
              (ptp->ports[port].rcvdPdelayResp &&
               (rxSequenceId == txSequenceId)))
          {
            /* Timeout or another response was received while waiting for the follow-up */
            MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_RESET);
          }
          else if (ptp->ports[port].rcvdPdelayRespFollowUp &&
                   (rxFUPSequenceId == txFUPSequenceId) &&
                   (compare_port_ids(rxFUPRequestingPortId, txFUPRequestingPortId) == 0))
          {
            /* Matching follow-up received */
            MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_WAITING_FOR_PDELAY_INTERVAL_TIMER);
          }
          break;

        case MDPdelayReq_WAITING_FOR_PDELAY_INTERVAL_TIMER:
          if (ptp->ports[port].pdelayIntervalTimer >= ptp->ports[port].pdelayReqInterval)
          {
            /* Request interval timer expired */
            MDPdelayReq_StateMachine_SetState(ptp, port, MDPdelayReq_SEND_PDELAY_REQ);
          }
          break;
      }

    } while (prevState != ptp->ports[port].mdPdelayReq_State);
  }
}

/* 802.1AS LinkDelaySyncIntervalSetting state machine (11.2.17.2) entry actions */
static void LinkDelaySyncIntervalSetting_StateMachine_SetState(struct ptp_device *ptp, uint32_t port, LinkDelaySyncIntervalSetting_State_t newState)
{
#ifdef PATH_DELAY_DEBUG
  printk("LinkDelaySyncIntervalSetting: Set State %d (port index %d)\n", newState, port);
#endif

  ptp->ports[port].linkDelaySyncIntervalSetting_State = newState;

  switch (newState)
  {
    default:
    case LinkDelaySyncIntervalSetting_NOT_ENABLED:
      break;

    case LinkDelaySyncIntervalSetting_INITIALIZE:
      ptp->ports[port].computeNeighborRateRatio = TRUE;
      ptp->ports[port].computeNeighborPropDelay = TRUE;
      /* TODO: Setup of interval values specified in the state machine */
      break;

    case LinkDelaySyncIntervalSetting_SET_INTERVALS:
      /* TODO: We don't actually process this TLV yet... */
      break;
  }
}

/* 802.1AS LinkDelaySyncIntervalSetting state machine (11.2.17.2) transitions */
void LinkDelaySyncIntervalSetting_StateMachine(struct ptp_device *ptp, uint32_t port)
{
  if (!ptp->ports[port].portEnabled || !ptp->ports[port].pttPortEnabled)
  {
    if (ptp->ports[port].linkDelaySyncIntervalSetting_State != LinkDelaySyncIntervalSetting_NOT_ENABLED)
    {
      /* Disabling the port immediately forces the state machine into the disabled state */
      LinkDelaySyncIntervalSetting_StateMachine_SetState(ptp, port, LinkDelaySyncIntervalSetting_NOT_ENABLED);
    }
  }
  else
  {
    LinkDelaySyncIntervalSetting_State_t prevState;
    do
    {
      prevState = ptp->ports[port].linkDelaySyncIntervalSetting_State;

      switch (ptp->ports[port].linkDelaySyncIntervalSetting_State)
      {
        default:
        case LinkDelaySyncIntervalSetting_NOT_ENABLED:
          if (ptp->ports[port].portEnabled && ptp->ports[port].pttPortEnabled)
          {
            /* Port (and timesync on it) became enabled */
            LinkDelaySyncIntervalSetting_StateMachine_SetState(ptp, port, LinkDelaySyncIntervalSetting_INITIALIZE);
          }
          break;

        case LinkDelaySyncIntervalSetting_INITIALIZE:
        case LinkDelaySyncIntervalSetting_SET_INTERVALS:
          if (0 /* TODO - Got signaling TLV */)
          {
            LinkDelaySyncIntervalSetting_StateMachine_SetState(ptp, port, LinkDelaySyncIntervalSetting_SET_INTERVALS);
          }
          break;
      }

    } while (prevState != ptp->ports[port].linkDelaySyncIntervalSetting_State);
  }
}


