/*
 *  linux/drivers/net/labx_ptp_netlink.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
 *  PTP Generic Netlink interface
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
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

/* System headers */
#include <net/genetlink.h>
#include <net/netlink.h>

#include "labx_ptp.h"

/* Per-attribute policy list for events we can *receive* */
static struct nla_policy ptp_events_genl_policy[PTP_EVENTS_A_MAX + 1] = {
  [PTP_EVENTS_A_DOMAIN]   = { .type = NLA_U32 },
  [PTP_EVENTS_A_VALUEMAP] = { .type = NLA_NESTED },
};

/* Family definition */
static struct genl_family ptp_events_genl_family = {
  .id      = GENL_ID_GENERATE,
  .hdrsize = 0,
  .name    = PTP_EVENTS_FAMILY_NAME,
  .version = PTP_EVENTS_FAMILY_VERSION,
  .maxattr = PTP_EVENTS_A_MAX,
};

/* Multicast groups */
static struct genl_multicast_group rtc_mcast = {
  .name = PTP_EVENTS_RTC_GROUP,
};

/* "Heartbeat" command - this is only sent by the driver; I have this in here
 * along with its policy to serve as a temporary example
 */
static int ptp_events_rx_heartbeat(struct sk_buff *skb, struct genl_info *info) {
  return(0);
}

int ptp_events_tx_heartbeat(struct ptp_device *ptp) {
  struct sk_buff *skb;
  void *msgHead;
  int returnValue = 0;

  skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if(skb == NULL) return(-ENOMEM);

  /* Create the message headers */
  msgHead = genlmsg_put(skb, 
                        0, 
                        ptp->netlinkSequence++, 
                        &ptp_events_genl_family, 
                        0, 
                        PTP_EVENTS_C_HEARTBEAT);
  if(msgHead == NULL) {
    returnValue = -ENOMEM;
    goto heartbeat_fail;
  }

  /* Write the PTP domain identifier to the message */
  returnValue = nla_put_u8(skb, PTP_EVENTS_A_DOMAIN, ptp->properties.domainNumber);
  if(returnValue != 0) goto heartbeat_fail;

  /* Finalize the message and multicast it */
  genlmsg_end(skb, msgHead);
  returnValue = genlmsg_multicast(skb, 0, rtc_mcast.id, GFP_ATOMIC);
  switch(returnValue) {
  case 0:
  case -ESRCH:
    // Success or no process was listening, simply break
    break;

  default:
    // This is an actual error, print the return code
    printk(KERN_INFO DRIVER_NAME ": Failure delivering multicast Netlink message: %d\n",
           returnValue);
    goto heartbeat_fail;
  }

 heartbeat_fail: 
  return(returnValue);
}

/* Size of a character string buffer, in bytes; this allocates enough
 * bytes to format each ID byte as a hexadecimal pair, put a delimiter
 * between each, and have a NULL terminator.
 */
#define CLOCK_ID_STRING_SIZE (PTP_CLOCK_IDENTITY_BYTES * 3)
#define NEW_GM_KEY_STRING    ("NewGrandmaster")
#define NEW_GM_BUF_SIZE      (strlen(NEW_GM_KEY_STRING) + CLOCK_ID_STRING_SIZE + 1)

int ptp_events_tx_gm_change(struct ptp_device *ptp) {
  struct sk_buff *skb;
  struct nlattr *valueMap;
  int32_t pairIndex;
  uint8_t *gmIdentity;
  char gmIdentityString[NEW_GM_BUF_SIZE];
  void *msgHead;
  int returnValue = 0;

  skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if(skb == NULL) return(-ENOMEM);

  /* Create the message headers */
  msgHead = genlmsg_put(skb, 
                        0, 
                        ptp->netlinkSequence++, 
                        &ptp_events_genl_family, 
                        0, 
                        PTP_EVENTS_C_GM_CHANGE);
  if(msgHead == NULL) {
    returnValue = -ENOMEM;
    goto gm_change_fail;
  }

  /* Write the PTP domain identifier to the message */
  returnValue = nla_put_u8(skb, PTP_EVENTS_A_DOMAIN, ptp->properties.domainNumber);
  if(returnValue != 0) goto gm_change_fail;

  /* Put a single entry into a key / value map to communicate the new Grandmaster */
  valueMap = nla_nest_start(skb, PTP_EVENTS_A_VALUEMAP);
  if(valueMap == NULL) goto gm_change_fail;

  /* Place the length of the map, then the key / value strings */
  nla_put_u32(skb, PTP_VALUEMAP_A_LENGTH, 1);
  pairIndex = PTP_VALUEMAP_A_PAIRS;

  /* Capture the Grandmaster identity as a string value */
  gmIdentity = ptp->presentMaster.grandmasterIdentity;
  sprintf(gmIdentityString, "%s:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
          NEW_GM_KEY_STRING,
          gmIdentity[0], gmIdentity[1], gmIdentity[2], gmIdentity[3], 
          gmIdentity[4], gmIdentity[5], gmIdentity[6], gmIdentity[7]);
  nla_put_string(skb, pairIndex++, gmIdentityString);

  /* End the value map nesting */
  nla_nest_end(skb, valueMap);

  /* Finalize the message and multicast it */
  genlmsg_end(skb, msgHead);
  returnValue = genlmsg_multicast(skb, 0, rtc_mcast.id, GFP_ATOMIC);
  switch(returnValue) {
  case 0:
  case -ESRCH:
    // Success or no process was listening, simply break
    break;

  default:
    // This is an actual error, print the return code
    printk(KERN_INFO DRIVER_NAME ": Failure delivering multicast Netlink message: %d\n",
           returnValue);
    goto gm_change_fail;
  }

 gm_change_fail: 
  return(returnValue);
}

/* Transmits a Netlink packet indicating a change in the RTC status */
int ptp_events_tx_rtc_change(struct ptp_device *ptp) {
  struct sk_buff *skb;
  uint8_t commandByte;
  void *msgHead;
  int returnValue = 0;

  skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if(skb == NULL) return(-ENOMEM);

  /* Decide which command byte to send with the message based upon the lock state */
  if(ptp->rtcLockState == PTP_RTC_LOCKED) {
    commandByte = PTP_EVENTS_C_RTC_LOCK;
  } else {
    commandByte = PTP_EVENTS_C_RTC_UNLOCK;
  }

  /* Create the message headers */
  msgHead = genlmsg_put(skb, 
                        0, 
                        ptp->netlinkSequence++, 
                        &ptp_events_genl_family, 
                        0, 
                        commandByte);
  if(msgHead == NULL) {
    returnValue = -ENOMEM;
    goto rtc_change_fail;
  }

  /* Write the PTP domain identifier to the message */
  returnValue = nla_put_u8(skb, PTP_EVENTS_A_DOMAIN, ptp->properties.domainNumber);
  if(returnValue != 0) goto rtc_change_fail;

  /* Finalize the message and multicast it */
  genlmsg_end(skb, msgHead);
  returnValue = genlmsg_multicast(skb, 0, rtc_mcast.id, GFP_ATOMIC);
  switch(returnValue) {
  case 0:
  case -ESRCH:
    // Success or no process was listening, simply break
    break;

  default:
    // This is an actual error, print the return code
    printk(KERN_INFO DRIVER_NAME ": Failure delivering multicast Netlink message: %d\n",
           returnValue);
    goto rtc_change_fail;
  }

 rtc_change_fail: 
  return(returnValue);
}

/* foo */

static struct genl_ops ptp_events_gnl_ops_heartbeat = {
  .cmd    = PTP_EVENTS_C_HEARTBEAT,
  .flags  = 0,
  .policy = ptp_events_genl_policy,
  .doit   = ptp_events_rx_heartbeat,
  .dumpit = NULL,
};

int register_ptp_netlink(void) {
  int returnValue;

  printk(KERN_INFO DRIVER_NAME ": Registering PTP Generic Netlink family\n");
  
  /* Register the Generic Netlink family for use */
  returnValue = genl_register_family(&ptp_events_genl_family);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register Generic Netlink family\n");
    goto register_failure;
  }

  /* Register operations */
  returnValue = genl_register_ops(&ptp_events_genl_family, &ptp_events_gnl_ops_heartbeat);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register Generic Netlink operation\n");
    genl_unregister_family(&ptp_events_genl_family);
    goto register_failure;
  }

  /* Register multicast groups */
  returnValue = genl_register_mc_group(&ptp_events_genl_family, &rtc_mcast);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register Generic Netlink multicast group\n");
    genl_unregister_ops(&ptp_events_genl_family, &ptp_events_gnl_ops_heartbeat);
    genl_unregister_family(&ptp_events_genl_family);
    goto register_failure;
  }

 register_failure:
  return(returnValue);
}

void unregister_ptp_netlink(void) {
  /* Unregister operations and the family */
  genl_unregister_ops(&ptp_events_genl_family, &ptp_events_gnl_ops_heartbeat);
  genl_unregister_family(&ptp_events_genl_family);
}
