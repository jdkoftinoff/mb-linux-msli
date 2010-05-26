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

/* Define this to get some extra debug on path delay messages */
/* #define PATH_DELAY_DEBUG */

static void computePdelayRateRatio(struct ptp_device *ptp)
{
  if (ptp->initPdelayRespReceived == FALSE)
  {
    ptp->initPdelayRespReceived = TRUE;
    ptp->pdelayRespTxTimestampI = ptp->pdelayRespTxTimestamp;
    ptp->pdelayRespRxTimestampI = ptp->pdelayRespRxTimestamp;
  }
  else
  {
    PtpTime difference;
    PtpTime difference2;
    uint64_t nsResponder;
    uint64_t nsRequester;
    uint64_t rateRatio;
    int shift;

    timestamp_difference(&ptp->pdelayRespTxTimestamp, &ptp->pdelayRespTxTimestampI, &difference);
    timestamp_difference(&ptp->pdelayRespRxTimestamp, &ptp->pdelayRespRxTimestampI, &difference2);

    nsResponder = ((uint64_t)difference.secondsLower) * 1000000000ULL + (uint64_t)difference.nanoseconds;
    nsRequester = ((uint64_t)difference2.secondsLower) * 1000000000ULL + (uint64_t)difference2.nanoseconds;

    for (shift = 0; shift < 31; shift++)
    {
      if (nsResponder & (1ULL<<(63-shift))) break;
    }

    rateRatio = (nsResponder << shift) / (nsRequester >> (31-shift));
    ptp->neighborRateRatio = (uint32_t)rateRatio;

    ptp->neighborRateRatioValid = TRUE;

#ifdef PATH_DELAY_DEBUG
    printk("Responder delta: %08X%08X.%08X (%llu ns)\n", difference.secondsUpper,
      difference.secondsLower, difference.nanoseconds, nsResponder);
    printk("Requester delta: %08X%08X.%08X (%llu ns)\n", difference2.secondsUpper,
      difference2.secondsLower, difference2.nanoseconds, nsRequester);
    printk("Rate ratio: %08X (shift %d)\n", ptp->neighborRateRatio, shift);
#endif
  }
}

static void computePropTime(struct ptp_device *ptp)
{
  if (ptp->neighborRateRatioValid)
  {
    PtpTime difference;
    PtpTime difference2;
    uint64_t nsResponder;
    uint64_t nsRequester;

    timestamp_difference(&ptp->pdelayRespTxTimestamp, &ptp->pdelayReqRxTimestamp, &difference);
    timestamp_difference(&ptp->pdelayRespRxTimestamp, &ptp->pdelayReqTxTimestamp, &difference2);

    nsResponder = ((uint64_t)difference.secondsLower) * 1000000000ULL + (uint64_t)difference.nanoseconds;
    nsRequester = ((uint64_t)difference2.secondsLower) * 1000000000ULL + (uint64_t)difference2.nanoseconds;

    ptp->neighborPropDelay = (((((uint64_t)ptp->neighborRateRatio) * nsRequester) >> 31) - nsResponder) >> 1;

#ifdef PATH_DELAY_DEBUG
    printk("Responder delta: %08X%08X.%08X (%llu ns)\n", difference.secondsUpper,
      difference.secondsLower, difference.nanoseconds, nsResponder);
    printk("Requester delta: %08X%08X.%08X (%llu ns)\n", difference2.secondsUpper,
      difference2.secondsLower, difference2.nanoseconds, nsRequester);
    printk("Prop Delay: %08X\n", ptp->neighborPropDelay);
#endif
  }
}

/* 802.1AS MDPdelayReq state machine (11.2.15.3) entry actions */
static void MDPdelayReq_StateMachine_SetState(struct ptp_device *ptp, MDPdelayReq_State_t newState)
{
  uint8_t rxRequestingPortId[PORT_ID_BYTES];
  uint8_t txRequestingPortId[PORT_ID_BYTES];

#ifdef PATH_DELAY_DEBUG
  printk("MDPdelayReq: Set State %d\n", newState);
#endif

  ptp->mdPdelayReq_State = newState;

  switch (newState)
  {
    default:
    case MDPdelayReq_NOT_ENABLED:
      break;

    case MDPdelayReq_INITIAL_SEND_PDELAY_REQ:
      ptp->initPdelayRespReceived = FALSE;
      ptp->neighborRateRatio = 0x80000000; // 1.0 fixed point 1.31
      ptp->rcvdMDTimestampReceive = FALSE;
      ptp->pdelayReqSequenceId = 0x0000; // TODO: spec says random()
      ptp->rcvdPdelayResp = FALSE;
      ptp->rcvdPdelayRespFollowUp = FALSE;
      transmit_pdelay_request(ptp);
      ptp->pdelayIntervalTimer = 0; // currentTime ("now" is zero ticks)
      ptp->lostResponses = 0;
      ptp->isMeasuringDelay = FALSE;
      ptp->asCapable = FALSE;
      ptp->neighborRateRatioValid = FALSE;
      break;

    case MDPdelayReq_RESET:
      ptp->initPdelayRespReceived = FALSE;
      ptp->rcvdPdelayResp = FALSE;
      ptp->rcvdPdelayRespFollowUp = FALSE;
      if (ptp->lostResponses <= ptp->allowedLostResponses)
      {
        ptp->lostResponses++;
      }
      else
      {
        ptp->isMeasuringDelay = FALSE;
        ptp->asCapable = FALSE;
      }
      break;

    case MDPdelayReq_SEND_PDELAY_REQ:
      ptp->pdelayReqSequenceId++;
      transmit_pdelay_request(ptp);
      ptp->pdelayIntervalTimer = 0; // currentTime ("now" is zero ticks)
      break;

    case MDPdelayReq_WAITING_FOR_PDELAY_RESP:
      ptp->rcvdMDTimestampReceive = FALSE;
      break;

    case MDPdelayReq_WAITING_FOR_PDELAY_RESP_FOLLOW_UP:
      ptp->rcvdPdelayResp = FALSE;

      /* Obtain the peer delay request receive timestamp that our peer has just sent.
       * (Trsp2 - responder local clock) */
      get_timestamp(ptp, RECEIVED_PACKET, ptp->rcvdPdelayRespPtr,
        &ptp->pdelayReqRxTimestamp);

      /* Capture the hardware timestamp at which we received this packet, and hang on to 
       * it for delay and rate calculation. (Trsp4 - our local clock) */
      get_local_hardware_timestamp(ptp, RECEIVED_PACKET,
        ptp->rcvdPdelayRespPtr, &ptp->pdelayRespRxTimestamp);
      break;

    case MDPdelayReq_WAITING_FOR_PDELAY_INTERVAL_TIMER:
      ptp->rcvdPdelayRespFollowUp = FALSE;
 
      /* Obtain the follow up timestamp for delay and rate calculation.
       * (Trsp3 - responder local clock) */
      get_timestamp(ptp, RECEIVED_PACKET, ptp->rcvdPdelayRespFollowUpPtr,
        &ptp->pdelayRespTxTimestamp);

      if (ptp->computeNeighborRateRatio)
      {
        computePdelayRateRatio(ptp);
      }
      if (ptp->computeNeighborPropDelay)
      {
        computePropTime(ptp);
      }
      ptp->lostResponses = 0;
      ptp->isMeasuringDelay = TRUE;
  
      get_rx_requesting_port_id(ptp, ptp->rcvdPdelayRespPtr, rxRequestingPortId);
      get_source_port_id(ptp, TRANSMITTED_PACKET, PTP_TX_PDELAY_REQ_BUFFER, txRequestingPortId);

      if ((ptp->neighborPropDelay <= ptp->neighborPropDelayThresh) &&
          (compare_port_ids(rxRequestingPortId, txRequestingPortId) != 0) && // TODO: this may be supposed to just compare the clockIdentity and not port
          ptp->neighborRateRatioValid)
      {
        ptp->asCapable = TRUE;
      }
      else
      {
        ptp->asCapable = FALSE;
      }
      break;
  } 
}

/* 802.1AS MDPdelayReq state machine (11.2.15.3) transitions */
void MDPdelayReq_StateMachine(struct ptp_device *ptp)
{
  if (!ptp->portEnabled || !ptp->pttPortEnabled)
  {
    if (ptp->mdPdelayReq_State != MDPdelayReq_NOT_ENABLED)
    {
      /* Disabling the port immediately forces the state machine into the disabled state */
      MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_NOT_ENABLED);
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
    int i;
    MDPdelayReq_State_t prevState;

    /* Grab some inforomation needed for comparisons if we got a PDelay Response */
    if (ptp->rcvdPdelayResp)
    {
      get_rx_requesting_port_id(ptp, ptp->rcvdPdelayRespPtr, rxRequestingPortId);
      get_source_port_id(ptp, TRANSMITTED_PACKET, PTP_TX_PDELAY_REQ_BUFFER, txRequestingPortId);
      rxSequenceId = get_sequence_id(ptp, RECEIVED_PACKET, ptp->rcvdPdelayRespPtr);
      txSequenceId = get_sequence_id(ptp, TRANSMITTED_PACKET, PTP_TX_PDELAY_REQ_BUFFER);
    }
    if (ptp->rcvdPdelayRespFollowUp)
    {
      get_rx_requesting_port_id(ptp, ptp->rcvdPdelayRespFollowUpPtr, rxFUPRequestingPortId);
      get_source_port_id(ptp, TRANSMITTED_PACKET, PTP_TX_PDELAY_REQ_BUFFER, txFUPRequestingPortId);
      rxFUPSequenceId = get_sequence_id(ptp, RECEIVED_PACKET, ptp->rcvdPdelayRespFollowUpPtr);
      txFUPSequenceId = get_sequence_id(ptp, TRANSMITTED_PACKET, PTP_TX_PDELAY_REQ_BUFFER);
    }

    do
    {
      prevState = ptp->mdPdelayReq_State;

      switch (ptp->mdPdelayReq_State)
      {
        default:
        case MDPdelayReq_NOT_ENABLED:
          if (ptp->portEnabled && ptp->pttPortEnabled)
          {
#ifdef PATH_DELAY_DEBUG
            printk("Port enabled\n");
#endif

            /* Port (and timesync on it) became enabled */
            MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_INITIAL_SEND_PDELAY_REQ);
          }
          break;

        case MDPdelayReq_INITIAL_SEND_PDELAY_REQ:
          if (ptp->rcvdMDTimestampReceive)
          {
#ifdef PATH_DELAY_DEBUG
            printk("PDelay Request Tx Timestamp available.\n");
#endif

            /* The transmit timestamp for the request is available */
            MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_WAITING_FOR_PDELAY_RESP);
          }
          break;

        case MDPdelayReq_RESET:
#ifdef PATH_DELAY_DEBUG
          printk("Resetting\n");
#endif
          MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_SEND_PDELAY_REQ);
          break;

        case MDPdelayReq_SEND_PDELAY_REQ:
          if (ptp->rcvdMDTimestampReceive)
          {
#ifdef PATH_DELAY_DEBUG
            printk("PDelay Request Tx Timestamp available.\n");
#endif

            /* The transmit timestamp for the request is available */
            MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_WAITING_FOR_PDELAY_RESP);
          }
          break;

        case MDPdelayReq_WAITING_FOR_PDELAY_RESP:
          if ((ptp->pdelayIntervalTimer >= ptp->pdelayReqInterval) ||
              (ptp->rcvdPdelayResp &&
               ((compare_port_ids(rxRequestingPortId, txRequestingPortId) != 0) ||
                (rxSequenceId != txSequenceId))))
          {
#ifdef PATH_DELAY_DEBUG
            printk("Resetting: intervalTimer %d, reqInterval %d, rcvdPdelayResp %d, rcvdPdelayRespPtr %d, rxSequence %d, txSequence %d\n",
              ptp->pdelayIntervalTimer, ptp->pdelayReqInterval, ptp->rcvdPdelayResp, ptp->rcvdPdelayRespPtr, rxSequenceId, txSequenceId);
            printk("rxRequestingPortID:");
            for (i=0; i<PORT_ID_BYTES; i++) printk("%02X", rxRequestingPortId[i]);
            printk("\n");
            printk("txRequestingPortID:");
            for (i=0; i<PORT_ID_BYTES; i++) printk("%02X", txRequestingPortId[i]);
            printk("\n");
#endif
 
            /* Timeout or a non-matching response was received */
            MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_RESET);
          }
          else if (ptp->rcvdPdelayResp &&
                   (rxSequenceId == txSequenceId) &&
                   (compare_port_ids(rxRequestingPortId, txRequestingPortId) == 0))
          {
            /* A matching response was received */
            MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_WAITING_FOR_PDELAY_RESP_FOLLOW_UP);
          }
          break;

        case MDPdelayReq_WAITING_FOR_PDELAY_RESP_FOLLOW_UP:
          if ((ptp->pdelayIntervalTimer >= ptp->pdelayReqInterval) ||
              (ptp->rcvdPdelayResp &&
               (rxSequenceId == txSequenceId)))
          {
            /* Timeout or another response was received while waiting for the follow-up */
            MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_RESET);
          }
          else if (ptp->rcvdPdelayRespFollowUp &&
                   (rxFUPSequenceId == txFUPSequenceId) &&
                   (compare_port_ids(rxFUPRequestingPortId, txFUPRequestingPortId) == 0))
          {
            /* Matching follow-up received */
            MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_WAITING_FOR_PDELAY_INTERVAL_TIMER);
          }
          break;

        case MDPdelayReq_WAITING_FOR_PDELAY_INTERVAL_TIMER:
          if (ptp->pdelayIntervalTimer >= ptp->pdelayReqInterval)
          {
            /* Request interval timer expired */
            MDPdelayReq_StateMachine_SetState(ptp, MDPdelayReq_SEND_PDELAY_REQ);
          }
          break;
      }

    } while (prevState != ptp->mdPdelayReq_State);
  }
}

