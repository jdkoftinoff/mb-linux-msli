/*
 *  linux/drivers/net/labx_avb/labx_audio_depacketizer_netlink.c
 *
 *  Lab X Technologies Audio Depacketizer driver
 *  Audio Depacketizer Generic Netlink interface
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *             Chris Wulff (chris.wulff@labxtechnologies.com)
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

#include "labx_audio_depacketizer.h"

#define DRIVER_NAME "labx_audio_depacketizer"

/* Family definition */
static struct genl_family events_genl_family = {
  .id      = GENL_ID_GENERATE,
  .hdrsize = 0,
  .name    = AUDIO_DEPACKETIZER_EVENTS_FAMILY_NAME,
  .version = AUDIO_DEPACKETIZER_EVENTS_FAMILY_VERSION,
  .maxattr = AUDIO_DEPACKETIZER_EVENTS_A_MAX,
};

/* Multicast groups */
static struct genl_multicast_group depacketizer_mcast = {
  .name = AUDIO_DEPACKETIZER_EVENTS_STREAM_GROUP,
};

int audio_depacketizer_stream_event(struct audio_depacketizer *depacketizer) {
  struct sk_buff *skb;
  void *msgHead;
  int returnValue = 0;

  skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if(skb == NULL) return(-ENOMEM);

  /* Create the message headers */
  msgHead = genlmsg_put(skb, 
                        0, 
                        depacketizer->netlinkSequence++, 
                        &events_genl_family, 
                        0, 
                        AUDIO_DEPACKETIZER_EVENTS_C_STREAM_STATUS);
  if(msgHead == NULL) {
    returnValue = -ENOMEM;
    goto fail;
  }

  /* Write the depacketizer minor number to identify the message source */
  returnValue = nla_put_u8(skb, AUDIO_DEPACKETIZER_EVENTS_A_MINOR, MINOR(depacketizer->cdev.dev));
  if(returnValue != 0) goto fail;

  returnValue = nla_put_u32(skb, AUDIO_DEPACKETIZER_EVENTS_A_STREAM_STATUS0, XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_0_REG)));
  if(returnValue != 0) goto fail;

  returnValue = nla_put_u32(skb, AUDIO_DEPACKETIZER_EVENTS_A_STREAM_STATUS1, XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_1_REG)));
  if(returnValue != 0) goto fail;

  returnValue = nla_put_u32(skb, AUDIO_DEPACKETIZER_EVENTS_A_STREAM_STATUS2, XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_2_REG)));
  if(returnValue != 0) goto fail;

  returnValue = nla_put_u32(skb, AUDIO_DEPACKETIZER_EVENTS_A_STREAM_STATUS3, XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_3_REG)));
  if(returnValue != 0) goto fail;

  /* Write an attribute identifying the stream which encountered a sequence error,
   * if there was one.
   */
  if(depacketizer->streamSeqError) {
    /* Write the identity of the stream 
     * NOTE - No register returns this yet!!!
     */
    returnValue = nla_put_u32(skb, AUDIO_DEPACKETIZER_EVENTS_A_STREAM_SEQ_ERROR, 0x00000000);
    
    /* Clear the "stream sequence error" flag.  There is a race condition inherent here
     * with the ISR; however, the delivery of any one sequence error event is already
     * unreliable due to the way the depacketizer hardware operates.
     */
    depacketizer->streamSeqError = 0;

    /* Make sure the clear occurs even if there was a write failure */
    if(returnValue != 0) goto fail;
  }

  /* Finalize the message and multicast it */
  genlmsg_end(skb, msgHead);
  returnValue = genlmsg_multicast(skb, 0, depacketizer_mcast.id, GFP_ATOMIC);
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

int register_audio_depacketizer_netlink(void) {
  int returnValue;

  printk(KERN_INFO DRIVER_NAME ": Registering Audio Depacketizer Generic Netlink family\n");
  
  /* Register the Generic Netlink family for use */
  returnValue = genl_register_family(&events_genl_family);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register Generic Netlink family\n");
    goto register_failure;
  }

  /* Register multicast groups */
  returnValue = genl_register_mc_group(&events_genl_family, &depacketizer_mcast);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register Generic Netlink multicast group\n");
    genl_unregister_family(&events_genl_family);
    goto register_failure;
  }

 register_failure:
  return(returnValue);
}

void unregister_audio_depacketizer_netlink(void) {
  /* Unregister the family */
  genl_unregister_family(&events_genl_family);
}

