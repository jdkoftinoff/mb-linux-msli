
/*
 *  linux/drivers/misc/labx/labx_mailbox_netlink.c
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
#include <net/sock.h>
#include <xio.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <net/sock.h>
#include <linux/socket.h>
#include <linux/net.h>
#include <asm/types.h>
#include <linux/skbuff.h>
#include "labx_mailbox.h"

#define DRIVER_NAME "labx_mailbox"

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Socket to receive messages */ 
struct sock *nf_sock = NULL;

/* Family definition */
static struct genl_family events_genl_family = {
  .id      = GENL_ID_GENERATE,
  .hdrsize = 0,
  .name    = LABX_MAILBOX_EVENTS_FAMILY_NAME,
  .version = LABX_MAILBOX_EVENTS_FAMILY_VERSION,
  .maxattr = LABX_MAILBOX_EVENTS_A_MAX,
};

/* Multicast groups */
static struct genl_multicast_group mailbox_mcast = {
  .name = LABX_MAILBOX_EVENTS_GROUP,
};

struct labx_mailbox* get_instance(const char name[]) {
  int i;
  struct labx_mailbox *targetMailbox = NULL;

  for (i = 0; i<MAX_MAILBOX_DEVICES; i++) {
    if ((labx_mailboxes[i] != NULL) && (!strcmp(labx_mailboxes[i]->name, name)))
    {
      targetMailbox = labx_mailboxes[i];
      break;
    }
  }
  return (targetMailbox);
}

int mailbox_event_send_request(struct labx_mailbox *mailbox) {
  struct sk_buff *skb;
  void *msgHead;
  int returnValue = 0;
  uint32_t msgLengthBytes, msgLengthWords;
  struct nlattr *packetNesting;
  uint32_t wordIndex;
  int32_t nestIndex;

  skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if(skb == NULL) return(-ENOMEM);

  /* Create the message headers */
  msgHead = genlmsg_put(skb, 
                        0, 
                        mailbox->netlinkSequence++, 
                        &events_genl_family, 
                        0, 
                        LABX_MAILBOX_EVENTS_C_REQUEST_MESSAGE);
  if(msgHead == NULL) {
    returnValue = -ENOMEM;
    goto fail;
  }

  /* Write the mailbox device to identify the message source */
  returnValue = nla_put_string(skb, LABX_MAILBOX_EVENTS_A_MAILBOX_DEVICE, mailbox->name);
  if(returnValue != 0) goto fail;

  /* Read the mailbox message length */ 
  msgLengthBytes = XIo_In32(REGISTER_ADDRESS(mailbox, SUPRV_MSG_LEN_REG));
  msgLengthWords = (msgLengthBytes+3)/4;

  /* Each packet is itself a nested table of words; begin the nesting */
  packetNesting = nla_nest_start(skb, LABX_MAILBOX_MESSAGE_A_PACKET);
  if(packetNesting == NULL) goto fail;
      
  /* Write the length of the packet and then its raw words */
  returnValue = nla_put_u32(skb, 
                            LABX_MAILBOX_MESSAGE_PACKET_A_LENGTH, 
                            msgLengthBytes);
  if(returnValue != 0) goto fail;

  nestIndex = LABX_MAILBOX_MESSAGE_PACKET_A_WORDS;
  for(wordIndex = 0; wordIndex < msgLengthWords; wordIndex++) {
    returnValue = nla_put_u32(skb, nestIndex++, XIo_In32(MSG_RAM_BASE(mailbox)+(wordIndex*4)));
    DBG("Read %08X from 0x%08X\n", XIo_In32(MSG_RAM_BASE(mailbox)+(wordIndex*4)),
      (MSG_RAM_BASE(mailbox) + (wordIndex*4)));
    if(returnValue != 0) goto fail;
  }
  DBG("\n");
     
  /* Close the nesting for the message packet */
  nla_nest_end(skb, packetNesting);

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

int mailbox_event_send_device(struct labx_mailbox *mailbox) {
  struct sk_buff *skb;
  void *msgHead;
  int returnValue = 0;

  skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if(skb == NULL) return(-ENOMEM);

  /* Create the message headers */
  msgHead = genlmsg_put(skb, 
                        0, 
                        mailbox->netlinkSequence++, 
                        &events_genl_family, 
                        0, 
                        LABX_MAILBOX_EVENTS_C_ANNOUNCE_MESSAGE);
  if(msgHead == NULL) {
    returnValue = -ENOMEM;
    goto fail;
  }

  /* Write the mailbox device to identify the message source */
  returnValue = nla_put_string(skb, LABX_MAILBOX_EVENTS_A_MAILBOX_DEVICE, mailbox->name);
  if(returnValue != 0) goto fail;

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

static int mailbox_event_announce_cb(struct sk_buff *skb, struct genl_info *info) {
  int i;

  for (i = 0; i<MAX_MAILBOX_DEVICES; i++) {
    if(labx_mailboxes[i] != NULL) {
      enable_mailbox(labx_mailboxes[i]);
      mailbox_event_send_device(labx_mailboxes[i]);
    }
  }
  return 0;
}

static int mailbox_event_shutdown_cb(struct sk_buff *skb, struct genl_info *info) {
  char targetName[NAME_MAX_SIZE];
  struct labx_mailbox *targetMailbox;

  nla_strlcpy(targetName, info->attrs[LABX_MAILBOX_EVENTS_A_MAILBOX_DEVICE], nla_len(info->attrs[LABX_MAILBOX_EVENTS_A_MAILBOX_DEVICE]));
  targetMailbox = get_instance(targetName);
  disable_mailbox(targetMailbox);

  return 0;
}

static int mailbox_event_queue_cb(struct sk_buff *skb, struct genl_info *info) {
  char targetName[NAME_MAX_SIZE];
  struct labx_mailbox *targetMailbox;

  nla_strlcpy(targetName, info->attrs[LABX_MAILBOX_EVENTS_A_MAILBOX_DEVICE], nla_len(info->attrs[LABX_MAILBOX_EVENTS_A_MAILBOX_DEVICE]));
  targetMailbox = get_instance(targetName);

  XIo_Out32(REGISTER_ADDRESS(targetMailbox, SUPRV_TRIG_ASYNC_REG), ENABLE);
  return 0;
}

static int mailbox_event_response_cb(struct sk_buff *skb, struct genl_info *info) {
  MessageData mailboxMessage; 
  struct labx_mailbox *targetMailbox;
  uint32_t wordIndex;
  uint32_t msgPayloadSize;
  uint32_t *msgPayloadPtr;
  char targetName[NAME_MAX_SIZE];
 
  //uint32_t i;
  //uint8_t *buf = (uint8_t*)nlmsg_data(info->nlhdr);
  //for(i = 0; i < info->nlhdr->nlmsg_len; i++) {
  //  printk("0x%02X ", *(buf++));
  //}

  nla_strlcpy(targetName, info->attrs[LABX_MAILBOX_EVENTS_A_MAILBOX_DEVICE], nla_len(info->attrs[LABX_MAILBOX_EVENTS_A_MAILBOX_DEVICE]));
  targetMailbox = get_instance(targetName);
 
  mailboxMessage.length = nla_get_u32(info->attrs[LABX_MAILBOX_MESSAGE_PACKET_A_LENGTH]);

  if(mailboxMessage.length <= MAX_MESSAGE_PACKET_WORDS) { 
    msgPayloadSize = nla_len(info->attrs[LABX_MAILBOX_MESSAGE_PACKET_A_WORDS]);
    msgPayloadPtr = (uint32_t *)nla_data(info->attrs[LABX_MAILBOX_MESSAGE_PACKET_A_WORDS]);
    memcpy(mailboxMessage.messageContent, msgPayloadPtr, msgPayloadSize);

    DBG("Writing IDL response to mailbox\n");
    for(wordIndex = 0; wordIndex < ((mailboxMessage.length+3)/4); wordIndex++) {
      DBG("Message: 0x%08X\n", mailboxMessage.messageContent[wordIndex]);
      XIo_Out32(MSG_RAM_BASE(targetMailbox)+(wordIndex*4), ((uint32_t*)mailboxMessage.messageContent)[wordIndex]);
    }
    DBG("Message response length: 0x%04X\n", mailboxMessage.length);
    XIo_Out32(REGISTER_ADDRESS(targetMailbox, HOST_MSG_LEN_REG), mailboxMessage.length);
  }
  return 0;
}

static struct genl_ops mailbox_nl_ops[] = {
	{
        	.cmd = LABX_MAILBOX_EVENTS_C_ANNOUNCE_MESSAGE,
		.doit = mailbox_event_announce_cb, 
	},
	{
		.cmd = LABX_MAILBOX_EVENTS_C_SHUTDOWN_MESSAGE,
		.doit = mailbox_event_shutdown_cb,
	},
	{
		.cmd = LABX_MAILBOX_EVENTS_C_RESPONSE_MESSAGE,
		.doit = mailbox_event_response_cb,
	},
	{
		.cmd = LABX_MAILBOX_EVENTS_C_EVENT_QUEUE_READY,
		.doit = mailbox_event_queue_cb,
	},
};

int register_mailbox_netlink(void) {
  int returnValue, i;

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

  /* Register generic netlink operations */
  for (i = 0; i < ARRAY_SIZE(mailbox_nl_ops); i++) {
    returnValue = genl_register_ops(&events_genl_family, &mailbox_nl_ops[i]);
    if (returnValue != 0) {
      printk(KERN_INFO DRIVER_NAME ": Failed to register Generic netlink operations\n");
      goto register_failure;
    }
  }

 register_failure:
  return(returnValue);
}

void unregister_mailbox_netlink(void) {
  /* Unregister the family */
  genl_unregister_family(&events_genl_family);
}

