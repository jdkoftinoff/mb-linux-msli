/*
 * Common framework for low-level network console, dump, and debugger code
 *
 * Sep 8 2003  Matt Mackall <mpm@selenic.com>
 *
 * based on the netconsole code from:
 *
 * Copyright (C) 2001  Ingo Molnar <mingo@redhat.com>
 * Copyright (C) 2002  Red Hat, Inc.
 */

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/string.h>
#include <linux/if_arp.h>
#include <linux/inetdevice.h>
#include <linux/inet.h>
#include <linux/interrupt.h>
#include <linux/netpoll.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/rcupdate.h>
#include <linux/workqueue.h>
#include <net/tcp.h>
#include <net/udp.h>
#include <asm/unaligned.h>

/*
 * We maintain a small pool of fully-sized skbs, to make sure the
 * message gets out even in extreme OOM situations.
 */

#define MAX_UDP_CHUNK 1460
#define MAX_SKBS 32
#define MAX_QUEUE_DEPTH (MAX_SKBS / 2)

static struct sk_buff_head skb_pool;

static atomic_t trapped;

#define USEC_PER_POLL 50
#define NETPOLL_RX_ENABLED 1
#define NETPOLL_RX_DROP 2

#define MAX_SKB_SIZE \
		(MAX_UDP_CHUNK + sizeof(struct udphdr) + \
				sizeof(struct iphdr) + sizeof(struct ethhdr))

static void zap_completion_queue(void);

static void queue_process(struct work_struct *work)
{
	struct netpoll_info *npinfo =
		container_of(work, struct netpoll_info, tx_work.work);
	struct sk_buff *skb;
	unsigned long flags;

	while ((skb = skb_dequeue(&npinfo->txq))) {
		struct net_device *dev = skb->dev;
		const struct net_device_ops *ops = dev->netdev_ops;
		struct netdev_queue *txq;

		if (!netif_device_present(dev) || !netif_running(dev)) {
			__kfree_skb(skb);
			continue;
		}

		txq = netdev_get_tx_queue(dev, skb_get_queue_mapping(skb));

		local_irq_save(flags);
		__netif_tx_lock(txq, smp_processor_id());
		if (netif_tx_queue_stopped(txq) ||
		    netif_tx_queue_frozen(txq) ||
		    ops->ndo_start_xmit(skb, dev) != NETDEV_TX_OK) {
			skb_queue_head(&npinfo->txq, skb);
			__netif_tx_unlock(txq);
			local_irq_restore(flags);

			schedule_delayed_work(&npinfo->tx_work, HZ/10);
			return;
		}
		__netif_tx_unlock(txq);
		local_irq_restore(flags);
	}
}

/*
 * Check whether delayed processing was scheduled for our NIC. If so,
 * we attempt to grab the poll lock and use ->poll() to pump the card.
 * If this fails, either we've recursed in ->poll() or it's already
 * running on another CPU.
 *
 * Note: we don't mask interrupts with this lock because we're using
 * trylock here and interrupts are already disabled in the softirq
 * case. Further, we test the poll_owner to avoid recursion on UP
 * systems where the lock doesn't exist.
 *
 * In cases where there is bi-directional communications, reading only
 * one message at a time can lead to packets being dropped by the
 * network adapter, forcing superfluous retries and possibly timeouts.
 * Thus, we set our budget to greater than 1.
 */
static int poll_one_napi(struct netpoll_info *npinfo,
			 struct napi_struct *napi, int budget)
{
	int work;

	/* net_rx_action's ->poll() invocations and our's are
	 * synchronized by this test which is only made while
	 * holding the napi->poll_lock.
	 */
	if (!test_bit(NAPI_STATE_SCHED, &napi->state))
		return budget;

	npinfo->rx_flags |= NETPOLL_RX_DROP;
	atomic_inc(&trapped);
	set_bit(NAPI_STATE_NPSVC, &napi->state);

	work = napi->poll(napi, budget);

	clear_bit(NAPI_STATE_NPSVC, &napi->state);
	atomic_dec(&trapped);
	npinfo->rx_flags &= ~NETPOLL_RX_DROP;

	return budget - work;
}

static void poll_napi(struct net_device *dev)
{
	struct napi_struct *napi;
	int budget = 16;

	list_for_each_entry(napi, &dev->napi_list, dev_list) {
		if (napi->poll_owner != smp_processor_id() &&
		    spin_trylock(&napi->poll_lock)) {
			budget = poll_one_napi(dev->npinfo, napi, budget);
			spin_unlock(&napi->poll_lock);

			if (!budget)
				break;
		}
	}
}

void netpoll_poll(struct netpoll *np)
{
	struct net_device *dev;
	const struct net_device_ops *ops;

	dev=np->dev;
	if (!dev || !netif_running(dev))
		return;

	ops = dev->netdev_ops;
	if (!ops->ndo_poll_controller)
		return;

	/* Process pending work on NIC */
	ops->ndo_poll_controller(dev);

	poll_napi(dev);

	zap_completion_queue();
}

static void refill_skbs(void)
{
	struct sk_buff *skb;
	unsigned long flags;

	spin_lock_irqsave(&skb_pool.lock, flags);
	while (skb_pool.qlen < MAX_SKBS) {
		skb = alloc_skb(MAX_SKB_SIZE, GFP_ATOMIC);
		if (!skb)
			break;

		__skb_queue_tail(&skb_pool, skb);
	}
	spin_unlock_irqrestore(&skb_pool.lock, flags);
}

static void zap_completion_queue(void)
{
	unsigned long flags;
	struct softnet_data *sd = &get_cpu_var(softnet_data);

	if (sd->completion_queue) {
		struct sk_buff *clist;

		local_irq_save(flags);
		clist = sd->completion_queue;
		sd->completion_queue = NULL;
		local_irq_restore(flags);

		while (clist != NULL) {
			struct sk_buff *skb = clist;
			clist = clist->next;
			if (skb->destructor) {
				atomic_inc(&skb->users);
				dev_kfree_skb_any(skb); /* put this one back */
			} else {
				__kfree_skb(skb);
			}
		}
	}

	put_cpu_var(softnet_data);
}

static struct sk_buff *find_skb(struct netpoll *np, int len, int reserve)
{
	int count = 0;
	struct sk_buff *skb;

	zap_completion_queue();
	refill_skbs();
repeat:

	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb)
		skb = skb_dequeue(&skb_pool);

	if (!skb) {
		if (++count < 10) {
			netpoll_poll(np);
			goto repeat;
		}
		return NULL;
	}

	atomic_set(&skb->users, 1);
	skb_reserve(skb, reserve);
	return skb;
}

static int netpoll_owner_active(struct net_device *dev)
{
	struct napi_struct *napi;

	list_for_each_entry(napi, &dev->napi_list, dev_list) {
		if (napi->poll_owner == smp_processor_id())
			return 1;
	}
	return 0;
}

static void netpoll_send_skb(struct netpoll *np, struct sk_buff *skb)
{
	int status = NETDEV_TX_BUSY;
	unsigned long tries;
	struct net_device *dev;
	const struct net_device_ops *ops;
	struct netpoll_info *npinfo;

	dev = np->dev;
	ops = dev->netdev_ops;
	npinfo = dev->npinfo;
	if (!npinfo || !netif_running(dev) || !netif_device_present(dev)) {
		__kfree_skb(skb);
		return;
	}

	/* don't get messages out of order, and no recursion */
	if (skb_queue_len(&npinfo->txq) == 0 && !netpoll_owner_active(dev)) {
		struct netdev_queue *txq;
		unsigned long flags;

		txq = netdev_get_tx_queue(dev, skb_get_queue_mapping(skb));

		local_irq_save(flags);
		/* try until next clock tick */
		for (tries = jiffies_to_usecs(1)/USEC_PER_POLL;
		     tries > 0; --tries) {
			if (__netif_tx_trylock(txq)) {
				if (!netif_tx_queue_stopped(txq))
					status = ops->ndo_start_xmit(skb, dev);
				__netif_tx_unlock(txq);

				if (status == NETDEV_TX_OK)
					break;

			}

			/* tickle device maybe there is some cleanup */
			netpoll_poll(np);

			udelay(USEC_PER_POLL);
		}
		local_irq_restore(flags);
	}

	if (status != NETDEV_TX_OK) {
		skb_queue_tail(&npinfo->txq, skb);
		schedule_delayed_work(&npinfo->tx_work,0);
	}
}

#define JDKS_NOTIFICATIONS_CONTROLLER_ENTITY_ID 0x70, 0xb3, 0xd5, 0xff, 0xff, 0xed, 0xcf, 0xff
#define JDKS_AEM_CONTROL_LOG_TEXT  0x70, 0xb3, 0xd5, 0xed, 0xc0, 0x00, 0x00, 0x00
#define AVDECC_AVTP_ETHERTYPE (0x22f0) /// See IEEE Std 1722-2011 Clause 5.1.2

static const uint8_t frame_template_17221[] = {
	(1<<7) | 0x7b,
	(0<<4) | 0x01, /* AEM response */
	0x00,0x00, /* status,length */
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* target guid */
	0x70,0xb3,0xd5,0xff,0xff,0xed,0xcf,0xff, /* controller guid */
	0x00,0x00, /* sequence id */
	(1<<7),0x18, /* unsolicited SET_CONTROL_VALUE */
	0x00,0x1a, /* CONTROL descriptor type */
	0x00, 0x00, /* descriptor index */
	0x70, 0xb3, 0xd5, 0xed, 0xc0, 0x00, 0x00, 0x00, /* JDKS_AEM_CONTROL_LOG_TEXT */
	0x00, 0x00, 0x00, 0x00, /* blob size */
	0xff, /* console log level */
	0x00, /* reserved */
};

void netpoll_send_udp(struct netpoll *np, const char *msg, int len)
{
	struct sk_buff *skb;
	struct ethhdr *eth;
	int log_level=0;
	static const uint8_t dst[6] = { 0x91, 0xe0, 0xf0, 0x01, 0x00, 0x00 };


	skb = find_skb(np,NET_IP_ALIGN+ETH_HLEN+sizeof(frame_template_17221)+len,NET_IP_ALIGN+ETH_HLEN+sizeof(frame_template_17221));
	if (!skb)
		return;

	skb_copy_to_linear_data(skb, msg, len);
	skb->len += len;

	skb_push(skb,sizeof(frame_template_17221));
	skb_reset_transport_header(skb);
	skb_reset_network_header(skb);

	memcpy(skb->data,&frame_template_17221[0],sizeof(frame_template_17221));
	*(skb->data+2)=(len+sizeof(frame_template_17221))>>(8*1);
	*(skb->data+3)=(len+sizeof(frame_template_17221))>>(8*0);
	memcpy(skb->data+4,&np->entity_id[0],sizeof(np->entity_id));
	memcpy(skb->data+20,&np->sequence_id,sizeof(np->sequence_id));
	memcpy(skb->data+26,&np->descriptor_index,sizeof(np->descriptor_index));
	*(skb->data+38)=(len+2)>>(8*1);
	*(skb->data+39)=(len+2)>>(8*0);

	np->sequence_id++;

	eth = (struct ethhdr *) skb_push(skb, ETH_HLEN);
	skb_reset_mac_header(skb);
	skb->protocol = eth->h_proto = htons(AVDECC_AVTP_ETHERTYPE);
	memcpy(eth->h_source, &np->dev->dev_addr[0], ETH_ALEN);
	memcpy(eth->h_dest, &dst[0], ETH_ALEN);

	skb->dev = np->dev;

	netpoll_send_skb(np, skb);
}

int __netpoll_rx(struct sk_buff *skb)
{
#if 0
	int proto, len, ulen;
	struct iphdr *iph;
	struct udphdr *uh;
	struct netpoll_info *npi = skb->dev->npinfo;
	struct netpoll *np = npi->rx_np;

	if (!np)
		goto out;
	if (skb->dev->type != ARPHRD_ETHER)
		goto out;

	/* check if netpoll clients need ARP */
	if (skb->protocol == htons(ETH_P_ARP) &&
//	    atomic_read(&trapped)) {
//		skb_queue_tail(&npi->arp_tx, skb);
		return 1;
	}

	proto = ntohs(eth_hdr(skb)->h_proto);
	if (proto != ETH_P_IP)
		goto out;
	if (skb->pkt_type == PACKET_OTHERHOST)
		goto out;
	if (skb_shared(skb))
		goto out;

	iph = (struct iphdr *)skb->data;
	if (!pskb_may_pull(skb, sizeof(struct iphdr)))
		goto out;
	if (iph->ihl < 5 || iph->version != 4)
		goto out;
	if (!pskb_may_pull(skb, iph->ihl*4))
		goto out;
	if (ip_fast_csum((u8 *)iph, iph->ihl) != 0)
		goto out;

	len = ntohs(iph->tot_len);
	if (skb->len < len || len < iph->ihl*4)
		goto out;

	/*
	 * Our transport medium may have padded the buffer out.
	 * Now We trim to the true length of the frame.
	 */
	if (pskb_trim_rcsum(skb, len))
		goto out;

	if (iph->protocol != IPPROTO_UDP)
		goto out;

	len -= iph->ihl*4;
	uh = (struct udphdr *)(((char *)iph) + iph->ihl*4);
	ulen = ntohs(uh->len);

	if (ulen != len)
		goto out;
	if (checksum_udp(skb, uh, ulen, iph->saddr, iph->daddr))
		goto out;
	if (np->local_ip && np->local_ip != iph->daddr)
		goto out;
	if (np->remote_ip && np->remote_ip != iph->saddr)
		goto out;
	if (np->local_port && np->local_port != ntohs(uh->dest))
		goto out;

	np->rx_hook(np, ntohs(uh->source),
		    (char *)(uh+1),
		    ulen - sizeof(struct udphdr));

	kfree_skb(skb);
	return 1;

out:
#endif
	if (atomic_read(&trapped)) {
		kfree_skb(skb);
		return 1;
	}

	return 0;
}

void netpoll_print_options(struct netpoll *np)
{
	printk(KERN_INFO "%s: interface %s\n", np->name, np->dev_name);
	printk(KERN_INFO "%s: descriptor index %d\n", np->name, np->descriptor_index);
	printk(KERN_INFO "%s: entity id %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\n",
		np->name,
		((unsigned int)np->entity_id[0])&0xff,
		((unsigned int)np->entity_id[1])&0xff,
		((unsigned int)np->entity_id[2])&0xff,
		((unsigned int)np->entity_id[3])&0xff,
		((unsigned int)np->entity_id[4])&0xff,
		((unsigned int)np->entity_id[5])&0xff,
		((unsigned int)np->entity_id[6])&0xff,
		((unsigned int)np->entity_id[7])&0xff);
}

int netpoll_parse_options(struct netpoll *np, char *opt)
{
	struct net_device *ndev = NULL;
	char *cur=opt, *delim;
	if (*cur != ',') {
		if ((delim = strchr(cur, ',')) == NULL)
			goto parse_failed;
		*delim = 0;
		np->descriptor_index = simple_strtol(cur, NULL, 10);
		cur = delim;
	}
	cur++;

	if (*cur != 0) {
		if ((delim = strchr(cur, '/')) != NULL) {
			*delim = 0;
		} else {
			delim = cur+strlen(cur);
		}
		strlcpy(np->dev_name, cur, sizeof(np->dev_name));
		cur = delim;
	}
	cur++;
	if (*cur != 0) {
		ndev = dev_get_by_name(&init_net, cur);
	} else {
		ndev = dev_get_by_name(&init_net, np->dev_name);
	}
	if (!ndev) {
		goto parse_failed;
	}

	np->entity_id[0] = ndev->dev_addr[0];
	np->entity_id[1] = ndev->dev_addr[1];
	np->entity_id[2] = ndev->dev_addr[2];
	np->entity_id[3] = 0xff;
	np->entity_id[4] = 0xfe;
	np->entity_id[5] = ndev->dev_addr[3];
	np->entity_id[6] = ndev->dev_addr[4];
	np->entity_id[7] = ndev->dev_addr[5];

	netpoll_print_options(np);

	return 0;

 parse_failed:
	printk(KERN_INFO "%s: couldn't parse config at %s!\n",
	       np->name, cur);
	return -1;
}

int netpoll_setup(struct netpoll *np)
{
	struct net_device *ndev = NULL;
#if 0
	struct in_device *in_dev;
#endif
	struct netpoll_info *npinfo;
	unsigned long flags;
	int err;

	if (np->dev_name)
		ndev = dev_get_by_name(&init_net, np->dev_name);
	if (!ndev) {
		printk(KERN_ERR "%s: %s doesn't exist, aborting.\n",
		       np->name, np->dev_name);
		return -ENODEV;
	}

	np->dev = ndev;
	if (!ndev->npinfo) {
		npinfo = kmalloc(sizeof(*npinfo), GFP_KERNEL);
		if (!npinfo) {
			err = -ENOMEM;
			goto release;
		}

		npinfo->rx_flags = 0;
		npinfo->rx_np = NULL;

		spin_lock_init(&npinfo->rx_lock);
		skb_queue_head_init(&npinfo->txq);
		INIT_DELAYED_WORK(&npinfo->tx_work, queue_process);

		atomic_set(&npinfo->refcnt, 1);
	} else {
		npinfo = ndev->npinfo;
		atomic_inc(&npinfo->refcnt);
	}

	if (!ndev->netdev_ops->ndo_poll_controller) {
		printk(KERN_ERR "%s: %s doesn't support polling, aborting.\n",
		       np->name, np->dev_name);
		err = -ENOTSUPP;
		goto release;
	}

	if (!netif_running(ndev)) {
		unsigned long atmost, atleast;

		printk(KERN_INFO "%s: device %s not up yet, forcing it\n",
		       np->name, np->dev_name);

		rtnl_lock();
		err = dev_open(ndev);
		rtnl_unlock();

		if (err) {
			printk(KERN_ERR "%s: failed to open %s\n",
			       np->name, ndev->name);
			goto release;
		}

		atleast = jiffies + HZ/10;
		atmost = jiffies + 4*HZ;
		while (!netif_carrier_ok(ndev)) {
			if (time_after(jiffies, atmost)) {
				printk(KERN_NOTICE
				       "%s: timeout waiting for carrier\n",
				       np->name);
				break;
			}
			cond_resched();
		}

		/* If carrier appears to come up instantly, we don't
		 * trust it and pause so that we don't pump all our
		 * queued console messages into the bitbucket.
		 */

		if (time_before(jiffies, atleast)) {
			printk(KERN_NOTICE "%s: carrier detect appears"
			       " untrustworthy, waiting 4 seconds\n",
			       np->name);
			msleep(4000);
		}
	}

	if (np->rx_hook) {
		spin_lock_irqsave(&npinfo->rx_lock, flags);
		npinfo->rx_flags |= NETPOLL_RX_ENABLED;
		npinfo->rx_np = np;
		spin_unlock_irqrestore(&npinfo->rx_lock, flags);
	}

	/* fill up the skb queue */
	refill_skbs();

	/* last thing to do is link it to the net device structure */
	ndev->npinfo = npinfo;

	/* avoid racing with NAPI reading npinfo */
	synchronize_rcu();

	return 0;

 release:
	if (!ndev->npinfo)
		kfree(npinfo);
	np->dev = NULL;
	dev_put(ndev);
	return err;
}

static int __init netpoll_init(void)
{
	skb_queue_head_init(&skb_pool);
	return 0;
}
core_initcall(netpoll_init);

void netpoll_cleanup(struct netpoll *np)
{
	struct netpoll_info *npinfo;
	unsigned long flags;

	if (np->dev) {
		npinfo = np->dev->npinfo;
		if (npinfo) {
			if (npinfo->rx_np == np) {
				spin_lock_irqsave(&npinfo->rx_lock, flags);
				npinfo->rx_np = NULL;
				npinfo->rx_flags &= ~NETPOLL_RX_ENABLED;
				spin_unlock_irqrestore(&npinfo->rx_lock, flags);
			}

			if (atomic_dec_and_test(&npinfo->refcnt)) {
				//skb_queue_purge(&npinfo->arp_tx);
				skb_queue_purge(&npinfo->txq);
				cancel_rearming_delayed_work(&npinfo->tx_work);

				/* clean after last, unfinished work */
				__skb_queue_purge(&npinfo->txq);
				kfree(npinfo);
				np->dev->npinfo = NULL;
			}
		}

		dev_put(np->dev);
	}

	np->dev = NULL;
}

int netpoll_trap(void)
{
	return atomic_read(&trapped);
}

void netpoll_set_trap(int trap)
{
	if (trap)
		atomic_inc(&trapped);
	else
		atomic_dec(&trapped);
}

EXPORT_SYMBOL(netpoll_set_trap);
EXPORT_SYMBOL(netpoll_trap);
EXPORT_SYMBOL(netpoll_print_options);
EXPORT_SYMBOL(netpoll_parse_options);
EXPORT_SYMBOL(netpoll_setup);
EXPORT_SYMBOL(netpoll_cleanup);
EXPORT_SYMBOL(netpoll_send_udp);
EXPORT_SYMBOL(netpoll_poll);
