/*
 *  linux/drivers/net/labx_ptp_bmca_announce_state.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
 *  PTP peer delay state machine processing
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Lab X Technologies LLC, All Rights Reserved.
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

// Set this to one for BMCA debug messages
#define BMCA_DEBUG 1

#if BMCA_DEBUG
#define BMCA_DBG(fmt, args...) printk(fmt, ##args)
#else
#define BMCA_DBG(fmt, args...)
#endif

#if BMCA_DEBUG
static void print_priority_vector(const char* str, const PtpPriorityVector* pv) {
  printk("%s: P1 %d, CC %d, CA %d, LV %d, P2 %d, GMID %02X%02X%02X%02X%02X%02X%02X%02X, SR %d, SPID %02X%02X%02X%02X%02X%02X%02X%02X SPN %d, PN %d\n",
    str,
    pv->rootSystemIdentity.priority1,
    pv->rootSystemIdentity.clockQuality.clockClass,
    pv->rootSystemIdentity.clockQuality.clockAccuracy,
    be16_to_cpu(pv->rootSystemIdentity.clockQuality.offsetScaledLogVariance),
    pv->rootSystemIdentity.priority2,
    pv->rootSystemIdentity.clockIdentity[0], pv->rootSystemIdentity.clockIdentity[1], pv->rootSystemIdentity.clockIdentity[2],
    pv->rootSystemIdentity.clockIdentity[3], pv->rootSystemIdentity.clockIdentity[4], pv->rootSystemIdentity.clockIdentity[5],
    pv->rootSystemIdentity.clockIdentity[6], pv->rootSystemIdentity.clockIdentity[7],
    be16_to_cpu(pv->stepsRemoved),
    pv->sourcePortIdentity.clockIdentity[0], pv->sourcePortIdentity.clockIdentity[1], pv->sourcePortIdentity.clockIdentity[2],
    pv->sourcePortIdentity.clockIdentity[3], pv->sourcePortIdentity.clockIdentity[4], pv->sourcePortIdentity.clockIdentity[5],
    pv->sourcePortIdentity.clockIdentity[6], pv->sourcePortIdentity.clockIdentity[7],
    be16_to_cpu(pv->sourcePortIdentity.portNumber),
    be16_to_cpu(pv->portNumber));
}
#endif

/* Enumerated type identifying the results of a BMCA comparison */
typedef enum {
  IS_PRESENT_MASTER,
  RETAIN_PRESENT_MASTER,
  REPLACE_PRESENT_MASTER
} BmcaResult;

static BmcaResult bmca_comparison(PtpPriorityVector* presentMaster, PtpPriorityVector* challenger) {

  int32_t comparison = memcmp(presentMaster, challenger, sizeof(PtpPriorityVector));

#if BMCA_DEBUG
  print_priority_vector("BMCA: CHAL", challenger);
  print_priority_vector("BMCA: PRES", presentMaster);
  printk("BMCA: Result %d\n", comparison);
#endif

  if(comparison > 0) {
    return REPLACE_PRESENT_MASTER;
  } else if (comparison == 0) {
    return IS_PRESENT_MASTER;
  } else {
    return RETAIN_PRESENT_MASTER;
  }
}

int8_t qualifyAnnounce(struct ptp_device *ptp, uint32_t port) {
  PtpPortIdentity  sourcePortId;
  uint16_t         stepsRemoved;
  uint32_t         pathTraceLength;
  PtpClockIdentity pathTrace[PTP_MAX_PATH_TRACE];
  uint32_t         i;

  get_source_port_id(ptp, port, RECEIVED_PACKET, ptp->ports[port].rcvdAnnouncePtr, (uint8_t*)&sourcePortId);

  BMCA_DBG("QA: port %d, Source port ID %02X%02X%02X%02X%02X%02X%02X%02X, PN %d\n", port,
    sourcePortId.clockIdentity[0], sourcePortId.clockIdentity[1], sourcePortId.clockIdentity[2],
    sourcePortId.clockIdentity[3], sourcePortId.clockIdentity[4], sourcePortId.clockIdentity[5],
    sourcePortId.clockIdentity[6], sourcePortId.clockIdentity[7], __be16_to_cpu(sourcePortId.portNumber));

  if (0 == memcmp(sourcePortId.clockIdentity, ptp->systemPriority.sourcePortIdentity.clockIdentity, sizeof(PtpClockIdentity))) {
    return FALSE;
  }

  stepsRemoved = get_rx_announce_steps_removed(ptp, port, ptp->ports[port].rcvdAnnouncePtr);

  BMCA_DBG("QA: SR %d\n", stepsRemoved);
  
  if (stepsRemoved > 255) {
    return FALSE;
  }

  pathTraceLength = get_rx_announce_path_trace(ptp, port, ptp->ports[port].rcvdAnnouncePtr, pathTrace);

  BMCA_DBG("QA: PTL %d\n", pathTraceLength);

  for (i=0; i<pathTraceLength; i++) {
    if (0 == memcmp(pathTrace[i], ptp->systemPriority.sourcePortIdentity.clockIdentity, sizeof(PtpClockIdentity))) {
      BMCA_DBG("QA: PT includes our clock\n");
      return FALSE;
    }
  }

  if (ptp->ports[port].selectedRole == PTP_SLAVE) {
    memcpy(ptp->pathTrace, pathTrace, sizeof(PtpClockIdentity)*pathTraceLength);
    ptp->pathTraceLength = pathTraceLength;
  }

  return TRUE;
}

/* 802.1AS PortAnnounceReceive state machine (10.3.10) */
void PortAnnounceReceive_StateMachine(struct ptp_device *ptp, uint32_t port)
{
  struct ptp_port *pPort = &ptp->ports[port];

  if (!pPort->portEnabled || !pPort->pttPortEnabled || !pPort->asCapable) {
    BMCA_DBG("PAR: rejected %d %d %d (port %d)\n", pPort->portEnabled, pPort->pttPortEnabled, pPort->asCapable, port);
    pPort->rcvdMsg = FALSE;
  } else {
    pPort->rcvdMsg = qualifyAnnounce(ptp, port);
  }
}

/* ------------------------------------------------------------------------ */
static AnnounceReceiveInfo rcvInfo(struct ptp_device *ptp, uint32_t port) {
  int result;
  extract_announce(ptp, port, ptp->ports[port].rcvdAnnouncePtr, &ptp->ports[port].messagePriority);
  result = memcmp(&ptp->ports[port].portPriority, &ptp->ports[port].messagePriority, sizeof(PtpPriorityVector));

  // TODO: if (!"conveys master port") return OtherInfo
  if (result > 0) {
    return SuperiorMasterInfo;
  } else if (result == 0) {
    return RepeatedMasterInfo;
  } else {
    return InferiorMasterInfo;
  }
}

static void recordOtherAnnounceInfo(struct ptp_device *ptp, uint32_t port) {
  /* TODO */
}

/* 802.1AS PortAnnounceInformation state machine (10.3.11) entry actions */
static void PortAnnounceInformation_StateMachine_SetState(struct ptp_device *ptp, uint32_t port, PortAnnounceInformation_State_t newState)
{
  struct ptp_port *pPort = &ptp->ports[port];

  BMCA_DBG("PAI: Set State %d (port index %d)\n", newState, port);

  pPort->portAnnounceInformation_State = newState;

  switch (newState)
  {
    default:
    case PortAnnounceInformation_BEGIN:
    case PortAnnounceInformation_DISABLED:
      pPort->rcvdMsg                = FALSE;
      pPort->announceTimeoutCounter = 0;
      pPort->infoIs                 = InfoIs_Disabled;
      pPort->reselect               = TRUE;
      pPort->selected               = FALSE;
      memset(&pPort->portPriority, 0xFF, sizeof(PtpPriorityVector));
      break;

    case PortAnnounceInformation_AGED:
      pPort->infoIs   = InfoIs_Aged;
      pPort->reselect = TRUE;
      pPort->selected = FALSE;
      break;

    case PortAnnounceInformation_UPDATE:
      memcpy(&pPort->portPriority, &pPort->masterPriority, sizeof(PtpPriorityVector));
      pPort->portStepsRemoved = ptp->masterStepsRemoved;
      pPort->updtInfo         = FALSE;
      pPort->infoIs           = InfoIs_Mine;
      pPort->newInfo          = TRUE;
      break;

    case PortAnnounceInformation_SUPERIOR_MASTER_PORT:
      memcpy(&pPort->portPriority, &pPort->messagePriority, sizeof(PtpPriorityVector));
      //pPort->portStepsRemoved = rcvdAnnouncePtr->stepsRemoved; ...
      recordOtherAnnounceInfo(ptp, port);
      pPort->announceTimeoutCounter = 0;
      pPort->infoIs                 = InfoIs_Received;
      pPort->reselect               = TRUE;
      pPort->selected               = FALSE;
      pPort->rcvdMsg                = FALSE;
      break;

    case PortAnnounceInformation_REPEATED_MASTER_PORT:
      pPort->announceTimeoutCounter = 0;
      pPort->rcvdMsg = FALSE;
      break;

    case PortAnnounceInformation_INFERIOR_MASTER_OR_OTHER_PORT:
      pPort->rcvdMsg = FALSE;
      break;

    case PortAnnounceInformation_CURRENT:
      break;

    case PortAnnounceInformation_RECEIVE:
      pPort->rcvdInfo = rcvInfo(ptp, port);
      break;
  }
}

/* 802.1AS PortAnnounceInformation state machine (10.3.11) transitions*/
void PortAnnounceInformation_StateMachine(struct ptp_device *ptp, uint32_t port)
{
  struct ptp_port *pPort = &ptp->ports[port];

  PortAnnounceInformation_State_t prevState;
  do
  {
    prevState = pPort->portAnnounceInformation_State;

    if (!pPort->portEnabled || !pPort->pttPortEnabled || !pPort->asCapable)
    {
      if (pPort->portAnnounceInformation_State != PortAnnounceInformation_DISABLED)
      {
        /* Disabling the port immediately forces the state machine into the disabled state */
        PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_DISABLED);
      }
    }
    else
    {
      switch (pPort->portAnnounceInformation_State)
      {
        default:
        case PortAnnounceInformation_BEGIN:
          PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_DISABLED);
          break;
            
        case PortAnnounceInformation_DISABLED:
          if (pPort->portEnabled && pPort->pttPortEnabled && pPort->asCapable) {
            PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_AGED);
          } else if (ptp->ports[port].rcvdMsg) {
            PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_DISABLED);
          }
          break;

        case PortAnnounceInformation_AGED:
          if (pPort->selected && pPort->updtInfo) {
            PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_UPDATE);
          }
          break;

        case PortAnnounceInformation_UPDATE:
        case PortAnnounceInformation_SUPERIOR_MASTER_PORT:
        case PortAnnounceInformation_REPEATED_MASTER_PORT:
        case PortAnnounceInformation_INFERIOR_MASTER_OR_OTHER_PORT:
          PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_CURRENT);
          break;

        case PortAnnounceInformation_CURRENT:
          if (pPort->selected && pPort->updtInfo) {
            PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_UPDATE);
          } else if (pPort->rcvdMsg && !pPort->updtInfo) {
            PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_RECEIVE);
          } else if ((pPort->infoIs == InfoIs_Received) &&
                     ((pPort->announceTimeoutCounter >= ANNOUNCE_INTERVAL_TICKS(ptp, port) * pPort->announceReceiptTimeout) ||
                      ((pPort->syncTimeoutCounter >= SYNC_INTERVAL_TICKS(ptp, port)) && ptp->gmPresent)) &&
                     !pPort->updtInfo && !pPort->rcvdMsg) {
            PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_AGED);

            /* Update stats */
            if (pPort->announceTimeoutCounter >= ANNOUNCE_INTERVAL_TICKS(ptp, port) * pPort->announceReceiptTimeout) {
              pPort->stats.announceReceiptTimeoutCount++;
            }
          }
          break;

        case PortAnnounceInformation_RECEIVE:
          if (pPort->rcvdInfo == SuperiorMasterInfo) {
            PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_SUPERIOR_MASTER_PORT);
          } else if (pPort->rcvdInfo == RepeatedMasterInfo) {
            PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_REPEATED_MASTER_PORT);
          } else { /* InferiorMasterInfo or OtherInfo */
            PortAnnounceInformation_StateMachine_SetState(ptp, port, PortAnnounceInformation_INFERIOR_MASTER_OR_OTHER_PORT);
          }
          break;
      }
    }
  } while (prevState != pPort->portAnnounceInformation_State);
}

/* ------------------------------------------------------------------------ */

static void updtRoleDisabledTree(struct ptp_device *ptp)
{
  uint32_t i;

  /* Set all port roles to disabled */
  for (i = 0; i<ptp->numPorts; i++) {
    ptp->ports[i].selectedRole = PTP_DISABLED;
  }

  /* Set the lastGmPriority to all 1's */
  memset(&ptp->lastGmPriority, 0xFF, sizeof(PtpPriorityVector));
  ptp->gmPriority = &ptp->lastGmPriority;

  /* Init the path array with thisClock */
  ptp->pathTraceLength = 1;
  memcpy(&ptp->pathTrace[0], ptp->systemPriority.rootSystemIdentity.clockIdentity, sizeof(PtpClockIdentity));
}

static void clearReselectTree(struct ptp_device *ptp)
{
  uint32_t i;
  for (i = 0; i<ptp->numPorts; i++) {
    ptp->ports[i].reselect = FALSE;
  }
}

static void updtRolesTree(struct ptp_device *ptp)
{
  uint32_t i;

  /* Save a copy of the last gm priority before we change any of the path priority vectors */
  memcpy(&ptp->lastGmPriority, ptp->gmPriority, sizeof(PtpPriorityVector));

  /* Compute gmPathPriority vectors */
  for (i = 0; i<ptp->numPorts; i++) {
    struct ptp_port *pPort = &ptp->ports[i];
    if (!(pPort->announceTimeoutCounter >= ANNOUNCE_INTERVAL_TICKS(ptp, i) * pPort->announceReceiptTimeout) &
        (!ptp->gmPresent || !(pPort->syncTimeoutCounter >= SYNC_INTERVAL_TICKS(ptp, i)))) {
      memcpy(&pPort->gmPathPriority, &pPort->portPriority, sizeof(PtpPriorityVector));
      pPort->gmPathPriority.stepsRemoved = cpu_to_be16(be16_to_cpu(pPort->gmPathPriority.stepsRemoved) + 1);
    }
  }

  /* Update the gmPriority vector */
  ptp->gmPriority = &ptp->systemPriority;
  ptp->masterStepsRemoved = 0;
  for (i = 0; i<ptp->numPorts; i++) {
    if (0 != compare_clock_identity(ptp->systemPriority.rootSystemIdentity.clockIdentity,
                                    ptp->ports[i].gmPathPriority.rootSystemIdentity.clockIdentity)) {
      if (REPLACE_PRESENT_MASTER == bmca_comparison(ptp->gmPriority, &ptp->ports[i].gmPathPriority)) {
        ptp->gmPriority = &ptp->ports[i].gmPathPriority;
        ptp->masterStepsRemoved = ptp->ports[i].messagePriority.stepsRemoved + 1;
      }
    }
  }

  /* TODO: set up leap/utc offset/etc. */

  /* Compute masterPriority vectors and assign port roles*/
  for (i = 0; i<ptp->numPorts; i++) {
    struct ptp_port *pPort = &ptp->ports[i];

    /* masterPriority */
    memcpy(&pPort->masterPriority, ptp->gmPriority, sizeof(PtpPriorityVector));
    memcpy(pPort->masterPriority.sourcePortIdentity.clockIdentity,
           ptp->systemPriority.sourcePortIdentity.clockIdentity, sizeof(PtpClockIdentity));
    pPort->masterPriority.sourcePortIdentity.portNumber = cpu_to_be16(i+1);
    pPort->masterPriority.portNumber = cpu_to_be16(i+1);

    /* selectedRole */
    switch (pPort->infoIs) {
      default:
      case InfoIs_Disabled:
        pPort->selectedRole = PTP_DISABLED;
        break;

      case InfoIs_Aged:
        pPort->selectedRole = PTP_MASTER;
        pPort->updtInfo = TRUE;
        break;

      case InfoIs_Mine:
        pPort->selectedRole = PTP_MASTER;
        if ((pPort->portStepsRemoved != ptp->masterStepsRemoved) ||
            (IS_PRESENT_MASTER != bmca_comparison(&pPort->portPriority, &pPort->masterPriority))) {
          pPort->updtInfo = TRUE;
        }
        break;

      case InfoIs_Received:
        if (ptp->gmPriority == &pPort->gmPathPriority) {
          pPort->selectedRole = PTP_SLAVE;
          pPort->updtInfo = FALSE;
        } else if (REPLACE_PRESENT_MASTER == bmca_comparison(&pPort->portPriority, &pPort->masterPriority)) {
          pPort->selectedRole = PTP_MASTER;
          pPort->updtInfo = TRUE;
        } else {
          pPort->selectedRole = PTP_PASSIVE;
          pPort->updtInfo = FALSE;
        }
    }
  }

  /* Update gmPresent */
  ptp->gmPresent = (ptp->gmPriority->rootSystemIdentity.priority1 == 255) ? 0 : 1;

  /* TODO: Do we need selectedRole[0]? */

  /* Update pathTrace */
  if (ptp->gmPriority == &ptp->systemPriority) {
    /* Init the path array with thisClock */
    ptp->pathTraceLength = 1;
    memcpy(&ptp->pathTrace[0], ptp->systemPriority.rootSystemIdentity.clockIdentity, sizeof(PtpClockIdentity));
  }

  if (0 != memcmp(ptp->gmPriority, &ptp->lastGmPriority, sizeof(PtpPriorityVector))) {
    ptp->newMaster              = TRUE;

    /* Do not permit the RTC to change until userspace permits it, and also
     * reset the lock state
     */
    ptp->acquiring          = PTP_RTC_ACQUIRING;
    ptp->rtcLockState       = PTP_RTC_UNLOCKED;
    ptp->rtcLockCounter     = 0;
    ptp->rtcChangesAllowed  = FALSE;
    ptp->rtcLastOffsetValid = PTP_RTC_OFFSET_VALID;
    ptp->rtcLastOffset      = 0;
#if BMCA_DEBUG
    {
      print_priority_vector("PRS: Previous Master ", &ptp->lastGmPriority);
      print_priority_vector("PRS: Current  Master ", ptp->gmPriority);
      int i;
      for (i=0; i<ptp->numPorts; i++) {
        printk("PRS: Port %d, Role %d\n", i, ptp->ports[i].selectedRole);
      }
    }
#endif
  }
}

static void setSelectedTree(struct ptp_device *ptp)
{
  uint32_t i;
  for (i = 0; i<ptp->numPorts; i++) {
    ptp->ports[i].selected = 1;
  }
}

/* 802.1AS PortRoleSelection state machine (10.3.12) transitions*/
void PortRoleSelection_StateMachine(struct ptp_device *ptp)
{
  uint32_t i;

  PortRoleSelection_State_t prevState;
  do
  {
    prevState = ptp->portRoleSelection_State;

    BMCA_DBG("PRS: Current State %d\n", ptp->portRoleSelection_State);

    switch (ptp->portRoleSelection_State)
    {
      default:
      case PortRoleSelection_INIT_BRIDGE:
        updtRoleDisabledTree(ptp);
        ptp->portRoleSelection_State = PortRoleSelection_ROLE_SELECTION;
        break;

      case PortRoleSelection_ROLE_SELECTION:
        for (i = 0; i<ptp->numPorts; i++) {
          if (ptp->ports[i].reselect) {
            clearReselectTree(ptp);
            updtRolesTree(ptp);
            setSelectedTree(ptp);
            break;
          }
        }
        break;
    }
  } while (prevState != ptp->portRoleSelection_State);
}

