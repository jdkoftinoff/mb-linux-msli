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

#include <net/genetlink.h>
#include <net/netlink.h>

#include "labx_ptp.h"

/* Family attributes */
enum {
  PTP_EVENTS_A_UNSPEC,
  PTP_EVENTS_A_MSG,
  __PTP_EVENTS_A_MAX,
};
#define PTP_EVENTS_A_MAX (__PTP_EVENTS_A_MAX - 1)

/* Per-attribute policy list */
static struct nla_policy ptp_events_genl_policy[PTP_EVENTS_A_MAX + 1] = {
  [PTP_EVENTS_A_MSG] = { .type = NLA_NUL_STRING },
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

/* Family commands */
enum {
  PTP_EVENTS_C_UNSPEC,
  PTP_EVENTS_C_HEARTBEAT,
  __PTP_EVENTS_C_MAX,
};
#define PTP_EVENTS_C_MAX (__PTP_EVENTS_C_MAX - 1)

/* "Heartbeat" command - this is only sent by the driver */
static int ptp_events_rx_heartbeat(struct sk_buff *skb, struct genl_info *info) {
  printk("Rx heartbeat!\n");
  return(0);
}

int ptp_events_tx_heartbeat(struct ptp_device *ptp) {
  struct sk_buff *skb;
  void *msgHead;
  int returnValue = 0;

  printk("Transmitting heartbeat event...\n");
  skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if(skb == NULL) return(-ENOMEM);

  /* Create the message headers */
  msgHead = genlmsg_put(skb, 0, ptp->netlinkSequence++, &ptp_events_genl_family, 0, PTP_EVENTS_C_HEARTBEAT);
  if(msgHead == NULL) {
    returnValue = -ENOMEM;
    goto heartbeat_fail;
  }

  /* Add a "message" attribute */
  returnValue = nla_put_string(skb, PTP_EVENTS_A_MSG, "<<< PTP Heartbeat! >>>");
  if(returnValue != 0) goto heartbeat_fail;

  /* Finalize the message and multicast it */
  genlmsg_end(skb, msgHead);
  returnValue = genlmsg_multicast(skb, 0, rtc_mcast.id, GFP_ATOMIC);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failure multicasting Netlink message\n");
    goto heartbeat_fail;
  }

 heartbeat_fail: 
  return(returnValue);
}

static struct genl_ops ptp_events_gnl_ops_heartbeat = {
  .cmd    = PTP_EVENTS_C_HEARTBEAT,
  .flags  = 0,
  .policy = ptp_events_genl_policy,
  .doit   = ptp_events_rx_heartbeat,
  .dumpit = NULL,
};

int register_ptp_netlink(void) {
  int returnValue;

  printk(KERN_INFO DRIVER_NAME "Registering PTP Generic Netlink family\n");
  
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

  printk(KERN_INFO DRIVER_NAME ": PTP Generic Netlink family ID %d\n",
         ptp_events_genl_family.id);

 register_failure:
  return(returnValue);
}

void unregister_ptp_netlink(void) {
  /* Unregister operations and the family */
  genl_unregister_ops(&ptp_events_genl_family, &ptp_events_gnl_ops_heartbeat);
  genl_unregister_family(&ptp_events_genl_family);
}
