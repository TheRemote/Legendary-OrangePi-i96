
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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <linux/debugfs.h>
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
#include <wland_android.h>

/* Version string to report */
#ifndef SRCBASE
#define SRCBASE        "drivers/net/wireless/rdaw80211"
#endif

static char wland_ver[] =
	"Compiled in " SRCBASE " on " __DATE__ " at " __TIME__;

#define MAX_WAIT_FOR_8021X_TX		        50	/* msecs */

char *wland_ifname(struct wland_private *drvr, int ifidx)
{
	if (ifidx < 0 || ifidx >= WLAND_MAX_IFS) {
		WLAND_ERR("ifidx %d out of range\n", ifidx);
		return "<if_bad>";
	}

	if (drvr->iflist[ifidx] == NULL) {
		WLAND_ERR("null i/f %d\n", ifidx);
		return "<if_null>";
	}

	return ((drvr->iflist[ifidx]->ndev) ? drvr->iflist[ifidx]->ndev->
		name : "<if_none>");
}

struct net_device *dhd_idx2net(void *pub, s32 ifidx)
{
	struct wland_private *priv = (struct wland_private *) pub;

	if (!pub || ifidx < 0 || ifidx >= WLAND_MAX_IFS)
		return NULL;

	if (priv && priv->iflist[ifidx])
		return priv->iflist[ifidx]->ndev;

	return NULL;
}

s32 dhd_net2idx(struct wland_private * driv, struct net_device * net)
{
	int i = 0;

	ASSERT(driv);

	while (i < WLAND_MAX_IFS) {
		if (driv->iflist[i] && (driv->iflist[i]->ndev == net))
			return i;
		i++;
	}

	return ALL_INTERFACES;
}

static void _wland_set_multicast_list(struct work_struct *work)
{
	struct wland_if *ifp =
		container_of(work, struct wland_if, multicast_work);
	struct net_device *ndev = ifp->ndev;

	struct netdev_hw_addr *ha;
	u32 cmd_value, cnt, buflen;
	__le32 cnt_le;
	char *buf, *bufp;
	s32 err;

	/*
	 * Determine initial value of allmulti flag
	 */
	cmd_value = (ndev->flags & IFF_ALLMULTI) ? true : false;

	/*
	 * Send down the multicast list first.
	 */
	netif_addr_lock_bh(ndev);
	cnt = netdev_mc_count(ndev);
	netif_addr_unlock_bh(ndev);

	buflen = sizeof(cnt) + (cnt * ETH_ALEN);
	buf = kmalloc(buflen, GFP_ATOMIC);

	WLAND_DBG(DEFAULT, TRACE, "Enter(idx:%d,cmd_value:%d,cnt:%d)\n",
		ifp->bssidx, cmd_value, cnt);

	if (!buf)
		return;

	bufp = buf;
	cnt_le = cpu_to_le32(cnt);
	memcpy(bufp, &cnt_le, sizeof(cnt_le));
	bufp += sizeof(cnt_le);

	netif_addr_lock_bh(ndev);
	netdev_for_each_mc_addr(ha, ndev) {
		if (!cnt)
			break;
		memcpy(bufp, ha->addr, ETH_ALEN);
		bufp += ETH_ALEN;
		cnt--;
	}

	netif_addr_unlock_bh(ndev);

	err = wland_fil_iovar_data_set(ifp, "mcast_list", buf, buflen);
	if (err < 0) {
		WLAND_ERR("Setting mcast_list failed, %d\n", err);
		cmd_value = cnt ? true : cmd_value;
	}

	kfree(buf);

	/*
	 * Now send the allmulti setting.  This is based on the setting in the
	 * net_device flags, but might be modified above to be turned on if we
	 * were trying to set some addresses and dongle rejected it...
	 */
	err = wland_fil_iovar_data_set(ifp, "allmulti", &cmd_value,
		sizeof(cmd_value));
	if (err < 0)
		WLAND_ERR("Setting allmulti failed, %d\n", err);

	/*
	 * Finally, pick up the PROMISC flag
	 */
	cmd_value = (ndev->flags & IFF_PROMISC) ? true : false;
	err = wland_fil_iovar_data_set(ifp, "promisc", &cmd_value,
		sizeof(cmd_value));
	if (err < 0)
		WLAND_ERR("Setting failed,err:%d\n", err);
}

static void _wland_set_mac_address(struct work_struct *work)
{
	s32 err;
	struct wland_if *ifp =
		container_of(work, struct wland_if, setmacaddr_work);

	WLAND_DBG(DEFAULT, TRACE, "Enter, idx=%d\n", ifp->bssidx);

	err = wland_fil_iovar_data_set(ifp, "cur_etheraddr", ifp->mac_addr,
		ETH_ALEN);
	if (err < 0) {
		WLAND_ERR("Setting cur_etheraddr failed, %d\n", err);
	} else {
		WLAND_DBG(DEFAULT, TRACE, "MAC address updated to %pM\n",
			ifp->mac_addr);
		memcpy(ifp->ndev->dev_addr, ifp->mac_addr, ETH_ALEN);
	}
}

static int netdev_set_mac_address(struct net_device *ndev, void *addr)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct sockaddr *sa = (struct sockaddr *) addr;

	memcpy(&ifp->mac_addr, sa->sa_data, ETH_ALEN);

	WLAND_DBG(DEFAULT, TRACE, "Enter %pM\n", sa->sa_data);

	schedule_work(&ifp->setmacaddr_work);
	return 0;
}

static void netdev_set_multicast_list(struct net_device *ndev)
{
	struct wland_if *ifp = netdev_priv(ndev);

	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	schedule_work(&ifp->multicast_work);
}

int netdev_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	int ret = NETDEV_TX_OK;
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_private *drvr = ifp->drvr;
	struct ethhdr *eh;
	struct wland_sdio_dev *sdiodev = drvr->bus_if->bus_priv.sdio;
	struct wland_sdio *bus = sdiodev->bus;

	WLAND_DBG(DEFAULT, TRACE, "Enter, ifp->bssidx=%d, skb->len=%d\n",
		ifp->bssidx, skb->len);

	/*
	 * Can the device send data?
	 */
	if ((drvr->bus_if->state != WLAND_BUS_DATA) || bus->hang_was_sent) {
		WLAND_ERR("xmit rejected state=%d\n", drvr->bus_if->state);
		netif_stop_queue(ndev);
		dev_kfree_skb(skb);
		ret = NETDEV_TX_BUSY;
		goto done;
	}

	if (!drvr->iflist[ifp->bssidx]) {
		WLAND_ERR("bad ifidx %d\n", ifp->bssidx);
		netif_stop_queue(ndev);
		dev_kfree_skb(skb);
		ret = NETDEV_TX_BUSY;
		goto done;
	}
	if (check_test_mode()) {
		WLAND_DBG(DEFAULT, INFO, "WIFI in test mode.\n");
		dev_kfree_skb(skb);
		goto done;
	}

	/*
	 * Make sure there's enough room for any header
	 */
	if (skb_headroom(skb) < drvr->hdrlen) {
		struct sk_buff *skb2;

		WLAND_DBG(DEFAULT, INFO,
			"%s: insufficient headroom and realloc skb.\n",
			wland_ifname(drvr, ifp->bssidx));
		skb2 = skb_realloc_headroom(skb, drvr->hdrlen);
		dev_kfree_skb(skb);
		skb = skb2;
		if (skb == NULL) {
			WLAND_ERR("%s: skb_realloc_headroom failed\n",
				wland_ifname(drvr, ifp->bssidx));
			ret = -ENOMEM;
			goto done;
		}
	}

	/*
	 * validate length for ether packet
	 */
	if (skb->len < sizeof(*eh)) {
		WLAND_ERR("validate length for ether packet!\n");
		ret = -EINVAL;
		dev_kfree_skb(skb);
		goto done;
	}

	ret = wland_sendpkt(ifp, skb);

done:
	if (ret < 0) {
		ifp->stats.tx_dropped++;
	} else {
		ifp->stats.tx_packets++;
		ifp->stats.tx_bytes += skb->len;
	}

	/*
	 * Return ok: we always eat the packet
	 */
	return NETDEV_TX_OK;
}

static struct net_device_stats *netdev_get_stats(struct net_device *ndev)
{
	struct wland_if *ifp = netdev_priv(ndev);

	if (ifp == NULL) {
		WLAND_ERR("BAD_IF\n");
		return NULL;
	}

	WLAND_DBG(DEFAULT, TRACE, "Done, idx:%d\n", ifp->bssidx);

	return &ifp->stats;
}

static void ethtool_get_drvinfo(struct net_device *ndev,
	struct ethtool_drvinfo *info)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_private *drvr = ifp->drvr;

	strlcpy(info->driver, KBUILD_MODNAME, sizeof(info->driver));
	snprintf(info->version, sizeof(info->version), "%d", drvr->drv_version);
	strlcpy(info->bus_info, dev_name(drvr->bus_if->dev),
		sizeof(info->bus_info));
}

static const struct ethtool_ops wland_ethtool_ops = {
	.get_drvinfo = ethtool_get_drvinfo,
};

static int wland_ethtool(struct wland_if *ifp, void __user * uaddr)
{
	struct wland_private *drvr = ifp->drvr;
	struct ethtool_drvinfo info;
	char drvname[sizeof(info.driver)];
	u32 cmd;
	struct ethtool_value edata;
	u32 toe_cmpnt, csum_dir;
	int ret;

	WLAND_DBG(DEFAULT, TRACE, "Enter, idx=%d\n", ifp->bssidx);

	/*
	 * all ethtool calls start with a cmd word
	 */
	if (copy_from_user(&cmd, uaddr, sizeof(u32)))
		return -EFAULT;

	switch (cmd) {
	case ETHTOOL_GDRVINFO:
		/*
		 * Copy out any request driver name
		 */
		if (copy_from_user(&info, uaddr, sizeof(info)))
			return -EFAULT;
		strncpy(drvname, info.driver, sizeof(info.driver));
		drvname[sizeof(info.driver) - 1] = '\0';

		/*
		 * clear struct for return
		 */
		memset(&info, 0, sizeof(info));
		info.cmd = cmd;

		/*
		 * if requested, identify ourselves
		 */
		if (strcmp(drvname, "?dhd") == 0) {
			sprintf(info.driver, "dhd");
			strcpy(info.version, WLAND_VERSION_STR);
		}
		/*
		 * report dongle driver type
		 */
		else {
			sprintf(info.driver, "wl");
		}

		sprintf(info.version, "%d", drvr->drv_version);

		if (copy_to_user(uaddr, &info, sizeof(info)))
			return -EFAULT;
		WLAND_DBG(DEFAULT, TRACE, "given %*s, returning %s\n",
			(int) sizeof(drvname), drvname, info.driver);
		break;

		/*
		 * Get toe offload components from dongle
		 */
	case ETHTOOL_GRXCSUM:
	case ETHTOOL_GTXCSUM:
		ret = wland_fil_iovar_data_get(ifp, "toe_ol", &toe_cmpnt,
			sizeof(toe_cmpnt));
		if (ret < 0)
			return ret;

		csum_dir = (cmd == ETHTOOL_GTXCSUM) ?
			TOE_TX_CSUM_OL : TOE_RX_CSUM_OL;

		edata.cmd = cmd;
		edata.data = (toe_cmpnt & csum_dir) ? 1 : 0;

		if (copy_to_user(uaddr, &edata, sizeof(edata)))
			return -EFAULT;
		break;

		/*
		 * Set toe offload components in dongle
		 */
	case ETHTOOL_SRXCSUM:
	case ETHTOOL_STXCSUM:
		if (copy_from_user(&edata, uaddr, sizeof(edata)))
			return -EFAULT;

		/*
		 * Read the current settings, update and write back
		 */
		ret = wland_fil_iovar_data_get(ifp, "toe_ol", &toe_cmpnt,
			sizeof(toe_cmpnt));
		if (ret < 0)
			return ret;

		csum_dir = (cmd == ETHTOOL_STXCSUM) ?
			TOE_TX_CSUM_OL : TOE_RX_CSUM_OL;

		if (edata.data != 0)
			toe_cmpnt |= csum_dir;
		else
			toe_cmpnt &= ~csum_dir;

		/*
		 * If setting TX checksum mode, tell Linux the new mode
		 */
		if (cmd == ETHTOOL_STXCSUM) {
			if (edata.data)
				ifp->ndev->features |= NETIF_F_IP_CSUM;
			else
				ifp->ndev->features &= ~NETIF_F_IP_CSUM;
		}
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int netdev_ioctl_entry(struct net_device *ndev, struct ifreq *ifr,
	int cmd)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_private *drvr = ifp->drvr;
	int ret = 0;

	/*
	 * bt wifi coexist
	 */
	static int bt_state = 0;
	int state = 0, old_state = 0;
	struct pta_param_s pta_param;

	/*
	 * bt wifi coexist
	 */

	WLAND_DBG(DEFAULT, TRACE, "Enter, idx=%d, cmd=0x%x\n", ifp->bssidx,
		cmd);

#ifdef WLAND_WEXT_SUPPORT
	/*
	 * linux wireless extensions
	 */
	if ((cmd >= SIOCIWFIRST) && (cmd <= SIOCIWLAST)) {
		/*
		 * may recurse, do NOT lock
		 */
		ret = wland_iw_ioctl(ndev, ifr, cmd);
		return ret;
	}
#endif /* WLAND_WEXT_SUPPORT */

	if (cmd == SIOCETHTOOL) {
		ret = wland_ethtool(ifp, ifr->ifr_data);
		return ret;
	}

	/*
	 * linux wireless extensions
	 */
	if (cmd == SIOCDEVPRIVATE + 1) {
		ret = wland_android_priv_cmd(ndev, ifr, cmd);
		//dhd_check_hang(net, &dhd->pub, ret);
		return ret;
	}
#if 0
	if (cmd != SIOCDEVPRIVATE) {
		dhd_os_wake_unlock(bus);
		return -EOPNOTSUPP;
	}
#endif

	if ((drvr->bus_if->chip != WLAND_VER_91_E) && (drvr->bus_if->chip != WLAND_VER_91_G))
		goto out;

	if (cmd == BT_COEXIST) {
		state = ifr->ifr_metric;
		ret = 0;

		pta_param.prot_mode = PTA_NONE_PROTECT;
		pta_param.mac_rate = 0x0;
		pta_param.hw_retry = 0x7;
		pta_param.sw_retry = 0x3;
		pta_param.cca_bypass = TRUE;
		pta_param.active_time = 500;	/* Unit is 100us */
		pta_param.thresh_time = 20;	/* Unit is 100us */
		pta_param.auto_prot_thresh_time = 200;	/* Unit is 100us */
		pta_param.flags = BIT0 | BIT1 | BIT5;
		pta_param.listen_interval = 0x06;

		if (state == BT_STATE_SCO_ONGOING) {
			state = BT_STATE_SCO_ON;
		}

		old_state = bt_state;

		if (state & (BT_STATE_SCO_ON | BT_STATE_SCO_ONGOING)) {
			bt_state |= BT_STATE_SCO_ON;
		}
		if (state & BT_STATE_A2DP_PLAYING) {
			bt_state |= BT_STATE_A2DP_PLAYING;
		}
		if (state & BT_STATE_CONNECTION_ON)
			bt_state |= BT_STATE_CONNECTION_ON;

		if (state == BT_STATE_SCO_OFF) {
			bt_state &= ~BT_STATE_SCO_ON;
		} else if (state == BT_STATE_A2DP_NO_PLAYING) {
			bt_state &= ~BT_STATE_A2DP_PLAYING;
		} else if (state == BT_STATE_CONNECTION_OFF)
			bt_state &= ~BT_STATE_CONNECTION_ON;

		if (old_state == bt_state)
			goto out;

		if (bt_state) {
			if (bt_state & BT_STATE_SCO_ON) {
				if (old_state)	//should clear pta proc before to set a new pta protec
					ret = wland_fil_set_cmd_data(ifp,
						WID_PTA_PARAMETER, &pta_param,
						sizeof(struct pta_param_s));
				pta_param.prot_mode = PTA_PS_POLL_PROTECT;
				pta_param.mac_rate = 0x4;
				pta_param.hw_retry = 0x1;
				pta_param.sw_retry = 0x1;
				pta_param.active_time = 25;
				pta_param.thresh_time = 5;
				pta_param.auto_prot_thresh_time = 15;
				pta_param.flags = BIT0 | BIT1 | BIT5;
				pta_param.listen_interval = 0x14;
			} else if (bt_state & BT_STATE_A2DP_PLAYING) {
				if (old_state)
					ret = wland_fil_set_cmd_data(ifp,
						WID_PTA_PARAMETER, &pta_param,
						sizeof(struct pta_param_s));
				pta_param.prot_mode = PTA_NULL_DATA_PROTECT;
				pta_param.active_time = 800;
				pta_param.thresh_time = 50;
				pta_param.auto_prot_thresh_time = 100;
				pta_param.sw_retry = 0x2;
			} else if (bt_state & BT_STATE_CONNECTION_ON) {
				if (old_state)
					ret = wland_fil_set_cmd_data(ifp,
						WID_PTA_PARAMETER, &pta_param,
						sizeof(struct pta_param_s));
				if(drvr->bus_if->chip == WLAND_VER_91_E)
					pta_param.prot_mode = PTA_SELF_CTS_PROTECT;
				else if(drvr->bus_if->chip == WLAND_VER_91_G)
					pta_param.prot_mode = PTA_SELF_CTS_PROTECT;
				pta_param.active_time = 1000;
				pta_param.thresh_time = 100;
				pta_param.auto_prot_thresh_time = 200;
			}
		} else {
			pta_param.prot_mode = PTA_NONE_PROTECT;
		}
		ret = wland_fil_set_cmd_data(ifp, WID_PTA_PARAMETER, &pta_param,
			sizeof(struct pta_param_s));
		WLAND_DBG(DEFAULT, INFO, "***BT_COEXIST state:%x \n", bt_state);

	}

out:
	WLAND_DBG(DEFAULT, TRACE, "Done.\n");

	return ret;
}

static int netdev_stop(struct net_device *ndev)
{
	struct wland_if *ifp = netdev_priv(ndev);

	WLAND_DBG(DEFAULT, TRACE, "Enter, idx=%d\n", ifp->bssidx);

	/*
	 * Set state and stop OS transmissions
	 */
	if (!netif_queue_stopped(ndev)) {
		netif_stop_queue(ndev);
		WLAND_DBG(DEFAULT, TRACE, "netif_stop_queue(ndev)\n");
	}
	if (netif_carrier_ok(ndev)) {
		netif_carrier_off(ndev);
		WLAND_DBG(DEFAULT, TRACE, "netif_carrier_off(ndev)\n");
	}

	wland_cfg80211_down(ndev);

	return 0;
}

static int netdev_open(struct net_device *ndev)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_private *drvr = ifp->drvr;
	struct wland_bus *bus_if = drvr->bus_if;

#ifdef WLAND_TBD_SUPPORT
	u32 toe_ol;
#endif /*WLAND_TBD_SUPPORT */
	s32 ret = 0;

	WLAND_DBG(DEFAULT, TRACE, "Enter, idx=%d\n", ifp->bssidx);

	/*
	 * If bus is not ready, can't continue
	 */
	if (bus_if->state != WLAND_BUS_DATA) {
		WLAND_ERR("failed bus is not ready\n");
		return -EAGAIN;
	}

	atomic_set(&ifp->pend_8021x_cnt, 0);

#ifdef WLAND_TBD_SUPPORT
	/*
	 * Get current TOE mode from dongle
	 */
	if (wland_fil_iovar_data_get(ifp, "toe_ol", &toe_ol,
			sizeof(toe_ol)) >= 0 && (toe_ol & TOE_TX_CSUM_OL) != 0)
		ndev->features |= NETIF_F_IP_CSUM;
	else
		ndev->features &= ~NETIF_F_IP_CSUM;
#endif /*WLAND_TBD_SUPPORT */

	if (wland_cfg80211_up(ndev) < 0) {
		WLAND_ERR("failed to bring up cfg80211\n");
		ret = -ENODEV;
	}

	/*
	 * Allow transmit calls
	 */
	if (!ret) {
		netif_carrier_on(ndev);
		WLAND_DBG(DEFAULT, TRACE, "netif_carrier_on(ndev)\n");
		netif_start_queue(ndev);
		WLAND_DBG(DEFAULT, TRACE, "netif_start_queue(ndev)\n");
	}

	return ret;
}

static void netdev_tx_timeout(struct net_device *dev)
{
	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	dev->trans_start = jiffies;	/* prevent tx timeout */
	netif_wake_queue(dev);
	dev->stats.tx_errors++;

	WLAND_DBG(DEFAULT, TRACE, "Done\n");
}

static const struct net_device_ops wland_netdev_ops_pri = {
	.ndo_open = netdev_open,
	.ndo_stop = netdev_stop,
	.ndo_get_stats = netdev_get_stats,
	.ndo_do_ioctl = netdev_ioctl_entry,
	.ndo_start_xmit = netdev_start_xmit,
	.ndo_tx_timeout = netdev_tx_timeout,
	.ndo_set_mac_address = netdev_set_mac_address,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
	.ndo_set_rx_mode = netdev_set_multicast_list,
#else /*(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)) */
	.ndo_set_multicast_list = netdev_set_multicast_list,
#endif /*(LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)) */
};

int netdev_attach(struct wland_if *ifp)
{
	struct wland_private *drvr = ifp->drvr;
	struct net_device *ndev = ifp->ndev;
	s32 err = 0;

	/*
	 * set appropriate operations
	 */
	ndev->netdev_ops = &wland_netdev_ops_pri;

	ndev->hard_header_len += drvr->hdrlen;
	ndev->flags |= IFF_BROADCAST | IFF_MULTICAST;

	ndev->ethtool_ops = &wland_ethtool_ops;

#ifdef WLAND_WEXT_SUPPORT
#if WIRELESS_EXT < 19
	ndev->get_wireless_stats = wland_get_wireless_stats;
#endif /* WIRELESS_EXT < 19 */
#if WIRELESS_EXT > 12
	ndev->wireless_handlers =
		(struct iw_handler_def *) &wland_iw_handler_def;
#endif /* WIRELESS_EXT > 12 */
#endif /* WLAND_WEXT_SUPPORT */

	/*
	 * set the mac address
	 */
	memcpy(ndev->dev_addr, ifp->mac_addr, ETH_ALEN);

	WLAND_DBG(DEFAULT, TRACE, "Enter,(%s:idx:%d,ifidx:0x%x)\n", ndev->name,
		ifp->bssidx, ifp->ifidx);

	err = register_netdev(ndev);
	if (err != 0) {
		WLAND_ERR("couldn't register the net device\n");
		goto fail;
	}

	WLAND_DBG(DEFAULT, TRACE,
		"%s: Rdamicro Host Driver(mac:%pM,ndevmtu:0x%x)\n", ndev->name,
		ndev->dev_addr, ndev->mtu);

	INIT_WORK(&ifp->setmacaddr_work, _wland_set_mac_address);
	INIT_WORK(&ifp->multicast_work, _wland_set_multicast_list);

	ndev->destructor = free_netdev;
	return 0;

fail:
	drvr->iflist[ifp->bssidx] = NULL;
	ndev->netdev_ops = NULL;
	free_netdev(ndev);
	return -EBADE;
}

#ifdef WLAND_P2P_SUPPORT
static int netdev_p2p_open(struct net_device *ndev)
{
	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	return wland_cfg80211_up(ndev);
}

static int netdev_p2p_stop(struct net_device *ndev)
{
	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	return wland_cfg80211_down(ndev);
}

static int netdev_p2p_do_ioctl(struct net_device *ndev, struct ifreq *ifr,
	int cmd)
{
	int ret = 0;

	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	/*
	 * There is no ifidx corresponding to p2p0 in our firmware. So we should
	 * * not Handle any IOCTL cmds on p2p0 other than ANDROID PRIVATE CMDs.
	 * * For Android PRIV CMD handling map it to primary I/F
	 */
	if (cmd == SIOCDEVPRIVATE + 1) {
		ret = wland_android_priv_cmd(ndev, ifr, cmd);
	} else {
		WLAND_ERR("IOCTL req 0x%x on p2p0 I/F. Ignoring. \n", cmd);
		ret = -1;
	}

	return ret;
}

static netdev_tx_t netdev_p2p_start_xmit(struct sk_buff *skb,
	struct net_device *ndev)
{
	if (skb)
		dev_kfree_skb_any(skb);

	WLAND_DBG(DEFAULT, TRACE,
		"(%s) is not used for data operations.Droping the packet.\n",
		ndev->name);

	/*
	 * Return ok: we always eat the packet
	 */
	return NETDEV_TX_OK;
}

static const struct net_device_ops wland_netdev_ops_p2p = {
	.ndo_open = netdev_p2p_open,
	.ndo_stop = netdev_p2p_stop,
	.ndo_do_ioctl = netdev_p2p_do_ioctl,
	.ndo_start_xmit = netdev_p2p_start_xmit
};

static void cfgp2p_ethtool_get_drvinfo(struct net_device *net,
	struct ethtool_drvinfo *info)
{
	snprintf(info->driver, sizeof(info->driver), "p2p");
	snprintf(info->version, sizeof(info->version), "%lu", (ulong) (0));
}

struct ethtool_ops cfgp2p_ethtool_ops = {
	.get_drvinfo = cfgp2p_ethtool_get_drvinfo
};

/* register "p2p0" interface */
int netdev_p2p_attach(struct wland_if *ifp)
{
	struct net_device *ndev = ifp->ndev;

	if (!ndev) {
		WLAND_ERR("p2p net device is empty\n");
		return -EBADE;
	}
	ndev->netdev_ops = &wland_netdev_ops_p2p;
	ndev->ethtool_ops = &cfgp2p_ethtool_ops;

	/*
	 * set the mac address
	 */
	memcpy(ndev->dev_addr, ifp->mac_addr, ETH_ALEN);

	WLAND_DBG(DEFAULT, TRACE, "Enter(idx:%d,mac:%pM)\n", ifp->bssidx,
		ifp->mac_addr);

	if (register_netdev(ndev)) {
		WLAND_ERR("couldn't register the p2p net device\n");
		goto fail;
	}
	WLAND_DBG(DEFAULT, TRACE, "Done(%s: Rdamicro Host Driver For P2P0)\n",
		ndev->name);

	return 0;
fail:
	ifp->drvr->iflist[ifp->bssidx] = NULL;
	ndev->netdev_ops = NULL;
	free_netdev(ndev);
	return -EBADE;
}
#endif /* WLAND_P2P_SUPPORT */

struct wland_if *wland_add_if(struct wland_private *drvr, s32 bssidx, s32 ifidx,
	char *name, u8 * mac_addr)
{
	struct net_device *ndev;
	struct wland_if *ifp = drvr->iflist[bssidx];

	WLAND_DBG(DEFAULT, TRACE, "Enter, idx:%d, ifidx:%d.\n", bssidx, ifidx);

	if (!(drvr && (bssidx < WLAND_MAX_IFS))) {
		WLAND_ERR("private not setup!\n");
		return ERR_PTR(-EINVAL);
	}

	/*
	 * Delete the existing interface before overwriting it in case we missed the WLAND_E_IF_DEL event.
	 */
	if (ifp) {
		WLAND_ERR("netname:%s,netdev:%p,ifidx:%d,already exists\n",
			ifp->ndev->name, ifp->ndev, ifidx);

		if (ifidx) {
			if (ifp->ndev) {
				netif_stop_queue(ifp->ndev);
				unregister_netdev(ifp->ndev);
				free_netdev(ifp->ndev);
				drvr->iflist[bssidx] = NULL;
			}
		} else {
			WLAND_ERR("ignore IF event\n");
			return ERR_PTR(-EINVAL);
		}
	}

	WLAND_DBG(DEFAULT, TRACE, "drvr->p2p_enable:%d,bssidx:%d\n",
		drvr->p2p_enable, bssidx);

	if (!drvr->p2p_enable && bssidx == 1) {
		/*
		 * this is P2P_DEVICE interface
		 */
		WLAND_DBG(DEFAULT, TRACE, "allocate non-netdev interface\n");
		ifp = kzalloc(sizeof(struct wland_if), GFP_KERNEL);
		if (!ifp)
			return ERR_PTR(-ENOMEM);
		memset(ifp, '\0', sizeof(struct wland_if));
	} else {
		WLAND_DBG(DEFAULT, TRACE, "allocate netdev interface\n");
		/*
		 * Allocate netdev, including space for private structure
		 */
		ndev = alloc_netdev(sizeof(struct wland_if), name, ether_setup);
		if (!ndev)
			return ERR_PTR(-ENOMEM);

		ndev->netdev_ops = NULL;

		ifp = netdev_priv(ndev);
		ifp->ndev = ndev;
	}

	ifp->drvr = drvr;
	ifp->ifidx = ifidx;
	ifp->bssidx = bssidx;

	drvr->iflist[bssidx] = ifp;

	init_waitqueue_head(&ifp->pend_8021x_wait);

	spin_lock_init(&ifp->netif_stop_lock);

	if (mac_addr)
		memcpy(ifp->mac_addr, mac_addr, ETH_ALEN);

	WLAND_DBG(DEFAULT, TRACE, "Done, pid:%x, if:%s (%pM) created ===\n",
		current->pid, ifp->ndev->name, ifp->mac_addr);

	return ifp;
}

void wland_del_if(struct wland_private *drvr, s32 bssidx)
{
	struct wland_if *ifp = drvr->iflist[bssidx];

	if (!ifp) {
		WLAND_ERR("Null interface,idx:%d\n", bssidx);
		return;
	}

	WLAND_DBG(DEFAULT, TRACE, "Enter,idx:%d,ifidx:%d,ndev:%p.\n", bssidx,
		ifp->ifidx, ifp->ndev);

	if (ifp->ndev) {
		if (bssidx == 0) {
			if (ifp->ndev->netdev_ops == &wland_netdev_ops_pri) {
				WLAND_DBG(DEFAULT, TRACE,
					"wlan0 interface ops.\n");

				if (!rtnl_is_locked())
					rtnl_lock();
				netdev_stop(ifp->ndev);
				if (rtnl_is_locked())
					rtnl_unlock();
			}
		} else {
			WLAND_DBG(DEFAULT, TRACE, "stop netdev:%p.\n",
				ifp->ndev);
			netif_stop_queue(ifp->ndev);
		}

		if (ifp->ndev->netdev_ops == &wland_netdev_ops_pri) {
			cancel_work_sync(&ifp->setmacaddr_work);
			cancel_work_sync(&ifp->multicast_work);
		}

		/*
		 * unregister will take care of freeing it
		 */
		WLAND_DBG(DEFAULT, TRACE, "detach netdev:%p.\n", ifp->ndev);

		unregister_netdev(ifp->ndev);
		drvr->iflist[bssidx] = NULL;
		if (bssidx == 0 && drvr->config && !IS_ERR(drvr->config))
			cfg80211_detach(drvr->config);
	} else {
		drvr->iflist[bssidx] = NULL;
		kfree(ifp);
	}
}

int wland_netdev_wait_pend8021x(struct net_device *ndev)
{
	struct wland_if *ifp = netdev_priv(ndev);
	int err = wait_event_timeout(ifp->pend_8021x_wait,
		!atomic_read(&ifp->pend_8021x_cnt),
		msecs_to_jiffies(MAX_WAIT_FOR_8021X_TX));

	WARN_ON(!err);

	return !err;
}

/* Module Entery For Linux OS */
static void wland_driver_init(struct work_struct *work)
{
#ifdef WLAND_SDIO_SUPPORT
	wland_sdio_register();
#endif /* WLAND_SDIO_SUPPORT */
#ifdef WLAND_USB_SUPPORT
	wland_usb_register();
#endif /* WLAND_USB_SUPPORT  */
}

static DECLARE_WORK(wland_driver_work, wland_driver_init);

struct semaphore registration_sem;
bool registration_check = false;

/* msec : allowed time to finished dhd registration */
#define REGISTRATION_TIMEOUT                     9000

void wland_registration_sem_up(bool check_flag)
{
	registration_check = check_flag;
	up(&registration_sem);
}

#define INSMOD_TEST 1

static int wlanfmac_module_init(void)
{
	WLAND_DBG(DEFAULT, TRACE, "%s.\n", wland_ver);
	WLAND_DBG(DEFAULT, TRACE, "Ver: %d.%d.%d.\n", WLAND_VER_MAJ,
		WLAND_VER_MIN, WLAND_VER_BLD);

	sema_init(&registration_sem, 0);

#ifdef INSMOD_TEST
	rda_wifi_power_on();
#endif /*INSMOD_TEST */

	wland_debugfs_init();

	if (!schedule_work(&wland_driver_work))
		return -EBUSY;

	/*
	 * Wait till MMC sdio_register_driver callback called and made driver attach.
	 * It's needed to make sync up exit from dhd insmod and Kernel MMC sdio device callback registration
	 */
	if ((down_timeout(&registration_sem,
				msecs_to_jiffies(REGISTRATION_TIMEOUT)) != 0)
		|| (!registration_check )) {
		WLAND_ERR("sdio_register_driver timeout or error\n");
		cancel_work_sync(&wland_driver_work);

#ifdef WLAND_SDIO_SUPPORT
		wland_sdio_exit();
#endif /* WLAND_SDIO_SUPPORT */

#ifdef WLAND_USB_SUPPORT
		wland_usb_exit();
#endif /*WLAND_USB_SUPPORT */

		wland_debugfs_exit();

#ifdef INSMOD_TEST
		rda_wifi_power_off();
#endif /* INSMOD_TEST */
		return -ENODEV;
	}
	return 0;
}

static void __exit wlanfmac_module_exit(void)
{
	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	cancel_work_sync(&wland_driver_work);

#ifdef WLAND_SDIO_SUPPORT
	wland_sdio_exit();
#endif /* WLAND_SDIO_SUPPORT */

#ifdef WLAND_USB_SUPPORT
	wland_usb_exit();
#endif /*WLAND_USB_SUPPORT */

	wland_debugfs_exit();

#ifdef INSMOD_TEST
	rda_wifi_power_off();
#endif /* INSMOD_TEST */
	WLAND_DBG(DEFAULT, TRACE, "Done\n");
}

void rda_wland_shutdown(struct device *dev)
{
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;

	WLAND_DBG(DEFAULT, TRACE, "Enter\n");
	cancel_work_sync(&wland_driver_work);
	wland_sdio_release(sdiodev->bus);
	wland_debugfs_exit();
	WLAND_DBG(DEFAULT, TRACE, "Done\n");
}

late_initcall(wlanfmac_module_init);
module_exit(wlanfmac_module_exit);

MODULE_AUTHOR("RdaMicro Corporation,BoChen");
MODULE_DESCRIPTION("RdaMicro 802.11 Wireless LAN FullMac Driver.");
MODULE_LICENSE("Dual BSD/GPL");
