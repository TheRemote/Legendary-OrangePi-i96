
/*
 * Copyright (c) 2014 Rdamicro Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <linuxver.h>
#include <linux_osl.h>
#include <linux/kernel.h>
#include <linux/if_arp.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <linux/kthread.h>
#include <linux/printk.h>
#include <linux/export.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/scatterlist.h>
#include <linux/semaphore.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/vmalloc.h>
#include <asm/unaligned.h>
#include <net/cfg80211.h>
#include <net/rtnetlink.h>

#include <wland_defs.h>
#include <wland_utils.h>
#include <wland_fweh.h>
#include <wland_dev.h>
#include <wland_dbg.h>
#include <wland_wid.h>
#include <wland_bus.h>
#include <wland_sdmmc.h>
#include <wland_p2p.h>
#include <wland_cfg80211.h>

void wland_txflowcontrol(struct device *dev, bool state)
{
	int i = 0;
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_private *drvr = bus_if->drvr;

	//struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
	struct wland_fws_info *fws = drvr->fws;

	//struct wland_sdio     *bus     = sdiodev->bus;

	WLAND_DBG(BUS, TRACE, "Enter\n");
	if (state) {
		fws->stats.bus_flow_block++;
	}

	for (i = 0; i < WLAND_MAX_IFS; i++) {
		if (drvr->iflist[i]) {
			if (state)
				netif_stop_queue(drvr->iflist[i]->ndev);
			else
				netif_wake_queue(drvr->iflist[i]->ndev);
		}
	}
}

int wland_bus_start(struct device *dev)
{
	int ret = -EINVAL;
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_private *drvr = bus_if->drvr;
	struct wland_if *ifp = NULL;
	u8 device_role = check_role_mode();

#ifdef WLAND_P2P_SUPPORT
	struct wland_if *p2p_ifp = NULL;
#endif /* WLAND_P2P_SUPPORT */

#ifdef USE_MAC_FROM_RDA_NVRAM
	u8 mac_addr[ETH_ALEN];
#else
	u8 mac_addr[ETH_ALEN] = { 0x59, 0x95, 0x4c, 0x33, 0x22, 0x11 };
#endif /* USE_MAC_FROM_RDA_NVRAM */

	WLAND_DBG(BUS, TRACE, "Enter\n");

	if (!bus_if) {
		WLAND_ERR("bus if empty!\n");
		return -EINVAL;
	}
#if defined(USE_MAC_FROM_RDA_NVRAM)
	if (wland_get_mac_address(mac_addr) != ETH_ALEN) {
		WLAND_ERR("nvram:get a random ether address\n");
		random_ether_addr(mac_addr);
		if (ret == -EINVAL)
			wlan_write_mac_to_nvram(mac_addr);
	} else {
		if (!is_valid_ether_addr(mac_addr)) {
			mac_addr[0] &= 0xfe;	/* clear multicast bit */
			mac_addr[0] |= 0x02;	/* set local assignment bit (IEEE802) */
			WLAND_ERR("nvram:get an invalid ether addr\n");
		}
	}
#elif defined(WLAND_MACADDR_DYNAMIC)
	if (wland_get_mac_address(mac_addr) != ETH_ALEN) {
		random_ether_addr(mac_addr);
		if (wland_set_mac_address(mac_addr) < 0) {
			WLAND_ERR("set cur_etheraddr failed!\n");
			return -ENODEV;
		}
	}
#endif

	/*
	 * add primary networking interface
	 */
	ifp = wland_add_if(drvr, 0, 0, "wlan%d", mac_addr);

	if (IS_ERR(ifp)) {
		WLAND_ERR("wland_add_if failed!\n");
		return PTR_ERR(ifp);
	}

#ifdef WLAND_P2P_SUPPORT
	u8 temp_addr[ETH_ALEN];
	temp_addr[0] |= 0x02;
	temp_addr[4] ^= 0x80;

	if (drvr->p2p_enable)
		p2p_ifp = wland_add_if(drvr, 1, 0, "p2p%d", temp_addr);
	else
		p2p_ifp = NULL;

	if (IS_ERR(p2p_ifp))
		p2p_ifp = NULL;
#endif /* WLAND_P2P_SUPPORT */

	ret = wland_fws_init(drvr);
	if (ret < 0)
		goto out;

	wland_fws_add_interface(ifp);

#if defined(WLAND_CFG80211_SUPPORT)
	drvr->config = cfg80211_attach(drvr, bus_if->dev);
	if (!drvr->config) {
		WLAND_ERR("cfg80211_attach failed\n");
		ret = -ENOMEM;
		goto out;
	}
#elif defined(WLAND_WEXT_SUPPORT)
	/* Attach and link in the iw */
	ret = iwext_attach(drvr, bus_if->dev);
	if (ret != 0) {
		WLAND_ERR("iwext_attach failed\n");
		ret = -ENOMEM;
		goto out;
	}
#endif /* WLAND_CFG80211_SUPPORT */

	ret = netdev_attach(ifp);
	if (ret < 0) {
		WLAND_ERR("netdev attach,failed:%d\n", ret);
		goto out;
	}

#ifdef WLAND_P2P_SUPPORT
	if (drvr->p2p_enable && p2p_ifp) {
		if (netdev_p2p_attach(p2p_ifp) < 0) {
			WLAND_ERR("p2p attach failed: %d.\n", ret);
			drvr->p2p_enable = false;
			ret = -EBADE;
			goto netdev_p2p_attach_fail;
		}
	}
#endif /* WLAND_P2P_SUPPORT */

	ret = wland_start_chip(ifp, device_role);
	if (ret < 0) {
		WLAND_ERR("failed to start up chip\n");
		goto start_chip_fail;
	}



start_chip_fail:

#ifdef WLAND_P2P_SUPPORT
	if (ret<0 && p2p_ifp && p2p_ifp->ndev) {
		WLAND_ERR("free_netdev p2p_ifp->ndev\n");
		unregister_netdev(p2p_ifp->ndev);
		drvr->iflist[1] = NULL;
	}
netdev_p2p_attach_fail:
#endif /* WLAND_P2P_SUPPORT */

	if (ret<0 && ifp && ifp->ndev) {
		WLAND_ERR("unregister netdev.\n");
		unregister_netdev(ifp->ndev);
		drvr->iflist[0] = NULL;
	}

out:
	if (ret < 0) {
		bus_if->state = WLAND_BUS_DOWN;
		if (drvr->config) {
			WLAND_ERR("cfg80211_detach\n");
			cfg80211_detach(drvr->config);
		}

		if (drvr->fws) {
			WLAND_ERR("wland_fws_deinit\n");
			wland_fws_del_interface(ifp);
			wland_fws_deinit(drvr);
		}


		if (drvr->iflist[0]) {
			free_netdev(drvr->iflist[0]->ndev);
			drvr->iflist[0] = NULL;
		}
#ifdef WLAND_P2P_SUPPORT
		if (drvr->iflist[1]) {
			free_netdev(drvr->iflist[1]->ndev);
			drvr->iflist[1] = NULL;
		}
#endif /* WLAND_P2P_SUPPORT */

		wland_registration_sem_up(false);
	} else {
		/* notify insmod ko ok */
		wland_registration_sem_up(true);
	}

	WLAND_DBG(BUS, TRACE, "Done.(ret=%d)\n", ret);
	return ret;
}

void wland_rx_frames(struct device *dev, struct sk_buff *skb)
{
	s32 ifidx = 0;
	struct wland_bus *bus_if;
	struct wland_private *drvr;
	struct wland_if *ifp;

	/*
	 * process and remove protocol-specific header
	 */
	int ret;

	WLAND_DBG(BUS, TRACE, "Enter,%s,count:%u\n", dev_name(dev), skb->len);

	/*
	 * setup receive data
	 */
	bus_if = dev_get_drvdata(dev);
	drvr = bus_if->drvr;
	ret = wland_proto_hdrpull(drvr, &ifidx, skb);
	ifp = drvr->iflist[ifidx];

	if (ret || !ifp || !ifp->ndev) {
		if ((ret != -ENODATA) && ifp)
			ifp->stats.rx_errors++;
		WLAND_ERR("RX error!\n");
		wland_pkt_buf_free_skb(skb);
	} else {
		wland_netif_rx(ifp, skb);
	}

	WLAND_DBG(BUS, TRACE, "Done,%s: count:%u\n", dev_name(dev), skb->len);
}

void wland_txcomplete(struct device *dev, struct sk_buff *txp, bool success)
{
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_private *drvr = bus_if->drvr;
	struct wland_if *ifp;
	struct ethhdr *eh;
	s32 ifidx = 0;
	u16 type;
	int res;

	WLAND_DBG(BUS, TRACE, "Enter,success:%d\n", success);

	res = wland_proto_hdrpull(drvr, &ifidx, txp);
	ifp = drvr->iflist[ifidx];
	if (!ifp)
		goto done;

	if (res == 0) {
		eh = (struct ethhdr *) (txp->data);
		type = ntohs(eh->h_proto);

		WLAND_DBG(BUS, TRACE, "type:%d\n", type);

		if (type == ETH_P_PAE) {
			atomic_dec(&ifp->pend_8021x_cnt);
			if (waitqueue_active(&ifp->pend_8021x_wait))
				wake_up(&ifp->pend_8021x_wait);
		}
	}
	if (!success)
		ifp->stats.tx_errors++;
done:
	wland_pkt_buf_free_skb(txp);

	WLAND_DBG(BUS, TRACE, "Done\n");
}

int wland_bus_active(struct device *dev)
{
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	int ret = -1;

	/*
	 * Bring up the bus
	 */
	ret = wland_bus_init(bus_if);
	if (ret < 0) {
		WLAND_ERR("bus init failed %d\n", ret);
		return ret;
	}

	return ret;
}

int wland_bus_attach(uint bus_hdrlen, struct device *dev)
{
	int ret = 0;
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_private *drvr = NULL;

	WLAND_DBG(BUS, TRACE, "Enter\n");

	/*
	 * Allocate primary wland_info
	 */
	drvr = osl_malloc(bus_if->osh, sizeof(struct wland_private));
	if (!drvr) {
		WLAND_ERR("Drvr Malloc Failed!\n");
		return -ENOMEM;
	}

	memset(drvr, '\0', sizeof(struct wland_private));

	mutex_init(&drvr->proto_block);

	/*
	 * Link to bus module
	 */
	drvr->hdrlen = bus_hdrlen;
	drvr->bus_if = bus_if;

#ifdef WLAND_P2P_SUPPORT
	drvr->p2p_enable = true;
#endif /*WLAND_P2P_SUPPORT */

	/*
	 * setup chip sleep flag
	 */
#ifdef WLAND_POWER_MANAGER
	drvr->sleep_flags = WLAND_SLEEP_ENABLE | WLAND_SLEEP_PREASSO;
#endif /*WLAND_POWER_MANAGER */

	bus_if->drvr = drvr;

	/*
	 * create device debugfs folder
	 */
	wland_debugfs_attach(drvr);

	/*
	 * Attach and link in the protocol
	 */
	ret = wland_proto_attach(drvr);
	if (ret < 0) {
		WLAND_ERR("proto_attach failed\n");
		goto fail;
	}

	/*
	 * attach firmware event handler
	 */
	wland_fweh_attach(drvr);

	WLAND_DBG(BUS, TRACE, "Done\n");

	return ret;

fail:
	wland_bus_detach(dev);

	return ret;
}

void wland_bus_detach(struct device *dev)
{
	s32 i;
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_private *drvr = bus_if->drvr;

	WLAND_DBG(BUS, TRACE, "Enter\n");

	if (!dev) {
		WLAND_ERR("Not Found Dev!\n");
		return;
	}

	if (!drvr) {
		WLAND_ERR("Not Found Private Val!\n");
		return;
	}

	/*
	 * stop firmware event handling
	 */
	wland_fweh_detach(drvr);

	/*
	 * make sure primary interface removed last
	 */
	for (i = WLAND_MAX_IFS - 1; i > -1; i--) {
		if (drvr->iflist[i]) {
			wland_fws_del_interface(drvr->iflist[i]);
			wland_del_if(drvr, i);
		}
	}

	/*
	 * Stop the bus module
	 */
	if (drvr)
		wland_bus_stop(drvr->bus_if);

	if (drvr->prot)
		wland_proto_detach(drvr);

	wland_fws_deinit(drvr);

	wland_debugfs_detach(drvr);

	bus_if->drvr = NULL;

	osl_free(bus_if->osh, drvr, sizeof(struct wland_private));

	WLAND_DBG(BUS, TRACE, "Done\n");
}
