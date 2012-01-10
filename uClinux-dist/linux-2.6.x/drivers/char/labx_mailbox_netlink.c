/*
 *  linux/drivers/char/labx_mailbox_netlink.c
 *
 *  Lab X Technologies Mailbox driver
 *  Mailbox Generic Netlink interface
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *             Chris Wulff (chris.wulff@labxtechnologies.com)
 *	       Albert M. Hajjar (albert.hajjar@labxtechnologies.com
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
#include <xio.h>

#include "labx_mailbox.h"

#define DRIVER_NAME "labx_mailbox"

/* Family definition */
static struct genl_family events_genl_family = {
  .id      = GENL_ID_GENERATE,
  .hdrsize = 0,
  .name    = MAILBOX_EVENTS_FAMILY_NAME,
  .version = MAILBOX_EVENTS_FAMILY_VERSION,
  .maxattr = MAILBOX_EVENTS_A_MAX,
};

/* Multicast groups */
static struct genl_multicast_group mailbox_mcast = {
  .name = MAILBOX_EVENTS_GROUP,
};

int mailbox_event_rcv_mesg(struct labx_mailbox *mailbox) {
  struct sk_buff *skb;
  void *msgHead;
  int returnValue = 0;
  uint32_t messageLength;
  int i;

  skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if(skb == NULL) return(-ENOMEM);

  /* Create the message headers */
  msgHead = genlmsg_put(skb, 
                        0, 
                        mailbox->netlinkSequence++, 
                        &events_genl_family, 
                        0, 
                        MAILBOX_EVENTS_C_RECEIVE_MESSAGE);
  if(msgHead == NULL) {
    returnValue = -ENOMEM;
    goto fail;
  }

  /* Write the depacketizer minor number to identify the message source */
  returnValue = nla_put_u8(skb, MAILBOX_EVENTS_A_MINOR, MINOR(mailbox->cdev.dev));
  if(returnValue != 0) goto fail;
  
  messageLength = XIo_In32(REGISTER_ADDRESS(mailbox, SUPRV_MSG_LEN_REG));
  returnValue = nla_put_u32(skb, MAILBOX_EVENTS_A_MESSAGE_LENGTH, messageLength);
  if(returnValue != 0) goto fail;
  
  for(i=0; i < ((messageLength + 3)/4); i++) {
    returnValue = nla_put_u32(skb, MAILBOX_EVENTS_A_RECV_MESSAGE, XIo_In32(MSG_RAM_BASE(mailbox)+(i*4)));
    if(returnValue != 0) goto fail;
  }

  /* Finalize the message and multicast it */
  genlmsg_end(skb, msgHead);
  returnValue = genlmsg_multicast(skb, 0, mailbox_mcast.id, GFP_ATOMIC);
  switch(returnValue) {
  case 0:
  case -ESRCH:
    // Success or no process was listening, simply break
    break;

  default:
    // This is an actual error, print the return code
    printk(KERN_INFO DRIVER_NAME ": Failure delivering multicast Netlink message: %d\n",
           returnValue);
    goto fail;
  }

fail: 
  return(returnValue);
}

int register_mailbox_netlink(void) {
  int returnValue;

  printk(KERN_INFO DRIVER_NAME ": Registering Mailbox Generic Netlink family\n");
  
  /* Register the Generic Netlink family for use */
  returnValue = genl_register_family(&events_genl_family);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register Generic Netlink family\n");
    goto register_failure;
  }

  /* Register multicast groups */
  returnValue = genl_register_mc_group(&events_genl_family, &mailbox_mcast);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register Generic Netlink multicast group\n");
    genl_unregister_family(&events_genl_family);
    goto register_failure;
  }

 register_failure:
  return(returnValue);
}

void unregister_mailbox_netlink(void) {
  /* Unregister the family */
  genl_unregister_family(&events_genl_family);
}

