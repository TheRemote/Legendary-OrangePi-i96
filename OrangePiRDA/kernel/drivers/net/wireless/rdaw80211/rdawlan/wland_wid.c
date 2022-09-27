
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
#include <linux/module.h>
#include <linux/if_ether.h>
#include <linux/spinlock.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <linux/debugfs.h>
#include <net/cfg80211.h>
#include <net/rtnetlink.h>
#include <linux/mmc/sdio_func.h>
#include <wland_defs.h>
#include <wland_utils.h>
#include <wland_fweh.h>
#include <wland_dev.h>
#include <wland_dbg.h>
#include <wland_wid.h>
#include <wland_bus.h>
#include <wland_sdmmc.h>
#include <wland_trap.h>
#include <wland_p2p.h>
#include <wland_cfg80211.h>

int wland_proto_hdrpush(struct wland_private *drvr, s32 ifidx,
	struct sk_buff *pktbuf)
{
	WLAND_DBG(DCMD, TRACE, "ifidx:%d,Enter\n", ifidx);

#if 0
	struct bdc_header *h;

	/*
	 * Push BDC header used to convey priority for buses that don't
	 */
	PKTPUSH(dhd->osh, pktbuf, BDC_HEADER_LEN);

	h = (struct bdc_header *) PKTDATA(dhd->osh, pktbuf);

	h->flags = (BDC_PROTO_VER << BDC_FLAG_VER_SHIFT);
	if (PKTSUMNEEDED(pktbuf))
		h->flags |= BDC_FLAG_SUM_NEEDED;

	h->priority = (PKTPRIO(pktbuf) & BDC_PRIORITY_MASK);
	h->flags2 = 0;
	h->dataOffset = 0;

	BDC_SET_IF_IDX(h, ifidx);
#endif /* BDC */

	return 0;
}

int wland_proto_hdrpull(struct wland_private *drvr, s32 * ifidx,
	struct sk_buff *pktbuf)
{
	WLAND_DBG(EVENT, TRACE, "Enter(pktbuf->len:%d)\n", pktbuf->len);

	/*
	 * Pop BDC header used to convey priority for buses that don't
	 */
	if (PKTLEN(drvr->bus_if->osh, pktbuf) <= FMW_HEADER_LEN) {
		WLAND_ERR("rx data too short (%d <= %d)\n", pktbuf->len,
			FMW_HEADER_LEN);
		return -EBADE;
	}

	if (ifidx)
		*ifidx = 0;
#if 0
	*ifidx = pktbuf->data[3];
	if (*ifidx >= WLAND_MAX_IFS) {
		WLAND_ERR("rx data ifnum out of range (%d)\n", *ifidx);
		return -EBADE;
	}
	/*
	 * The ifidx is the idx to map to matching netdev/ifp. When receiving
	 * * events this is easy because it contains the bssidx which maps
	 * * 1-on-1 to the netdev/ifp. But for data frames the ifidx is rcvd.
	 * * bssidx 1 is used for p2p0 and no data can be received or
	 * * transmitted on it. Therefor bssidx is ifidx + 1 if ifidx > 0
	 */
	if (*ifidx)
		(*ifidx)++;

	//////////////////////////////////////////////
	struct bdc_header *h;
	u8 data_offset = 0;

	h = (struct bdc_header *) PKTDATA(dhd->osh, pktbuf);

	if (!ifidx) {
		/*
		 * for tx packet, skip the analysis
		 */
		data_offset = h->dataOffset;
		PKTPULL(dhd->osh, pktbuf, BDC_HEADER_LEN);
		goto exit;
	}

	if ((*ifidx = BDC_GET_IF_IDX(h)) >= WLAND_MAX_IFS) {
		DHD_ERROR(("%s: rx data ifnum out of range (%d)\n",
				__FUNCTION__, *ifidx));
		return BCME_ERROR;
	}

	if (((h->flags & BDC_FLAG_VER_MASK) >> BDC_FLAG_VER_SHIFT) !=
		BDC_PROTO_VER) {
		DHD_ERROR(("%s: non-BDC packet received, flags = 0x%x\n",
				wland_ifname(dhd, *ifidx), h->flags));
		if (((h->flags & BDC_FLAG_VER_MASK) >> BDC_FLAG_VER_SHIFT) ==
			BDC_PROTO_VER_1)
			h->dataOffset = 0;
		else
			return BCME_ERROR;
	}

	if (h->flags & BDC_FLAG_SUM_GOOD) {
		DHD_ERROR(
			("%s: BDC packet received with good rx-csum, flags 0x%x\n",
				wland_ifname(dhd, *ifidx), h->flags));
		PKTSETSUMGOOD(pktbuf, true);
	}

	PKTSETPRIO(pktbuf, (h->priority & BDC_PRIORITY_MASK));
	data_offset = h->dataOffset;
	PKTPULL(dhd->osh, pktbuf, BDC_HEADER_LEN);

	if (PKTLEN(dhd->osh, pktbuf) < (u32) (data_offset << 2)) {
		DHD_ERROR(("%s: rx data too short (%d < %d)\n", __FUNCTION__,
				PKTLEN(dhd->osh, pktbuf), (data_offset * 4)));
		return BCME_ERROR;
	}
#endif /* BDC */

	if (pktbuf->len == 0)
		return -ENODATA;

	return 0;
}

int wland_sendpkt(struct wland_if *ifp, struct sk_buff *pktbuf)
{
	struct wland_private *drvr = ifp->drvr;
	struct ethhdr *eh = (struct ethhdr *) (pktbuf->data);
	bool multicast = is_multicast_ether_addr(eh->h_dest);
	bool pae = eh->h_proto == htons(ETH_P_PAE);

	WLAND_DBG(DCMD, TRACE,
		"Enter(tx_proto:0x%X, is_multicast:%d, pae:%d)\n",
		ntohs(eh->h_proto), multicast, pae);

	/*
	 * Update multicast statistic
	 */
	drvr->tx_multicast += ! !multicast;
	if (pae)
		atomic_inc(&ifp->pend_8021x_cnt);

	/*
	 * If the protocol uses a data header, apply it
	 */
	wland_proto_hdrpush(drvr, ifp->bssidx, pktbuf);

	/*
	 * Update multicast statistic
	 */
	if (pktbuf->len >= ETH_ALEN) {
		struct ethhdr *eh = (struct ethhdr *) (pktbuf->data);

		WLAND_DBG(DCMD, TRACE, "dest: %pM\n", eh->h_dest);
		WLAND_DBG(DCMD, TRACE, "source: %pM\n", eh->h_source);
		if (is_multicast_ether_addr(eh->h_dest)) {
			WLAND_DBG(DCMD, TRACE, "%pM is multicast ether addr\n",
				eh->h_dest);
		}
		if (ntohs(eh->h_proto) == ETH_P_PAE) {
			WLAND_DBG(DCMD, TRACE, "eh->h_proto == ETH_P_PAE\n");
		}
	}

	/*
	 * Use bus module to send data frame
	 */
	return wland_bus_txdata(drvr->bus_if, pktbuf);
}

/* setup chip */
int wland_preinit_cmds(struct wland_if *ifp)
{
	int err = 0;
	u32 u32Val = 0;
	u8 u8Val = 0;

	/*
	 * add for qos
	 */
	struct wland_private *drvr = ifp->drvr;
	struct wland_bus *bus_if = drvr->bus_if;

#ifdef WLAND_POWER_MANAGER
	struct wland_sdio_dev *sdiodev = drvr->bus_if->bus_priv.sdio;
#endif

	if (!ifp) {
		WLAND_ERR("ifp Empty!\n");
		return -ENODEV;
	}

	WLAND_DBG(DCMD, DEBUG, "Enter\n");

	if (check_test_mode()) {
		WLAND_DBG(DCMD, INFO, "In test mode and do nothing.\n");
		goto done;
	}
	err = wland_fil_get_cmd_data(ifp, WID_SYS_FW_VER, &u32Val,
		sizeof(u32Val));
	if (err < 0) {
		WLAND_ERR("Retreiving version information failed!\n");
		return -EINVAL;
	}
	WLAND_DBG(DCMD, DEBUG, "FirmWareVer:0x%x \n", u32Val);

	err = wland_fil_set_cmd_data(ifp, WID_MAC_ADDR, ifp->mac_addr,
		ETH_ALEN);
	if (err < 0) {
		WLAND_ERR("set cur_etheraddr failed\n");
		goto done;
	}

	WLAND_DBG(DCMD, DEBUG, "#########Set MAC Address(" MACDBG ")\n",
		MAC2STRDBG(ifp->mac_addr));

	err = wland_fil_get_cmd_data(ifp, WID_MAC_ADDR, ifp->mac_addr,
		ETH_ALEN);
	if (err < 0) {
		WLAND_ERR("Retreiving cur_etheraddr failed, %d\n", err);
		goto done;
	}

	WLAND_DBG(DCMD, DEBUG, "#########Get MAC Address(" MACDBG ")\n",
		MAC2STRDBG(ifp->mac_addr));

	memcpy(ifp->drvr->mac, ifp->mac_addr, ETH_ALEN);

	u8Val = G_AUTO_PREAMBLE;
	err = wland_fil_set_cmd_data(ifp, WID_PREAMBLE, &u8Val, sizeof(u8Val));
	if (err < 0) {
		WLAND_ERR("set preamble failed, %d\n", err);
		goto done;
	}
#if 0
	u8Val = 0;
	err = wland_fil_set_cmd_data(ifp, WID_PTA_MODE, &u8Val, sizeof(u8Val));
	if (err < 0) {
		WLAND_ERR("set pta mode failed, %d\n", err);
		goto done;
	}

	err = wland_fil_set_cmd_data(ifp, WID_PTA_BLOCK_BT, &u8Val,
		sizeof(u8Val));
	if (err < 0) {
		WLAND_ERR("set pta block bt failed, %d\n", err);
		goto done;
	}
#endif

	err = wland_set_scan_timeout(ifp);
	if (err < 0) {
		WLAND_ERR("set_scan_timeout failed, %d\n", err);
		goto done;
	}

	u8Val = WIFI_LISTEN_INTERVAL;
	err = wland_fil_set_cmd_data(ifp, WID_LISTEN_INTERVAL, &u8Val,
		sizeof(u8Val));
	if (err < 0) {
		WLAND_ERR("set_listen_interval failed, %d\n", err);
		goto done;
	}

	if (bus_if->chip == WLAND_VER_90_D || bus_if->chip == WLAND_VER_90_E) {
		u8Val = WIFI_LINK_LOSS_THRESHOLD_90;
		err = wland_fil_set_cmd_data(ifp, WID_LINK_LOSS_THRESHOLD,
			&u8Val, sizeof(u8Val));
		if (err < 0) {
			WLAND_ERR("set_link_loss_threshold failed, %d\n", err);
			goto done;
		}
	} else if (bus_if->chip == WLAND_VER_91) {
		u8Val = WIFI_LINK_LOSS_THRESHOLD_91;
		err = wland_fil_set_cmd_data(ifp, WID_LINK_LOSS_THRESHOLD,
			&u8Val, sizeof(u8Val));
		if (err < 0) {
			WLAND_ERR("set_link_loss_threshold failed, %d\n", err);
			goto done;
		}
		u8Val = 0x30;
		err = wland_fil_set_cmd_data(ifp, WID_POWER_SAVE, &u8Val,
			sizeof(u8Val));
		if (err < 0) {
			WLAND_ERR("Set WID_POWER_SAVE failed, %d\n", err);
			goto done;
		}

	} else if (bus_if->chip == WLAND_VER_91_E) {
		u8Val = WIFI_LINK_LOSS_THRESHOLD_91;
		err = wland_fil_set_cmd_data(ifp, WID_LINK_LOSS_THRESHOLD,
			&u8Val, sizeof(u8Val));
		if (err < 0) {
			WLAND_ERR("set_link_loss_threshold failed, %d\n", err);
			goto done;
		}
		u8Val = 0x30;
		err = wland_fil_set_cmd_data(ifp, WID_POWER_SAVE, &u8Val,
			sizeof(u8Val));
		if (err < 0) {
			WLAND_ERR("Set WID_POWER_SAVE failed, %d\n", err);
			goto done;
		}
	} else if (bus_if->chip == WLAND_VER_91_F) {
		u8Val = WIFI_LINK_LOSS_THRESHOLD_91;
		err = wland_fil_set_cmd_data(ifp, WID_LINK_LOSS_THRESHOLD,
			&u8Val, sizeof(u8Val));
		if (err < 0) {
			WLAND_ERR("set_link_loss_threshold failed, %d\n", err);
			goto done;
		}
		u8Val = 0x30;
		err = wland_fil_set_cmd_data(ifp, WID_POWER_SAVE, &u8Val,
			sizeof(u8Val));
		if (err < 0) {
			WLAND_ERR("Set WID_POWER_SAVE failed, %d\n", err);
			goto done;
		}
		u8Val = 0;
		err = wland_fil_set_cmd_data(ifp, WID_QOS_ENABLE, &u8Val,
			sizeof(u8Val));
		if (err) {
			WLAND_ERR("Set WID_QOS_ENABLE failed! \n");
			goto done;
		}
	} else if (bus_if->chip == WLAND_VER_91_G) {
		u8Val = WIFI_LINK_LOSS_THRESHOLD_91;
		err = wland_fil_set_cmd_data(ifp, WID_LINK_LOSS_THRESHOLD,
			&u8Val, sizeof(u8Val));
		if (err < 0) {
			WLAND_ERR("set_link_loss_threshold failed, %d\n", err);
			goto done;
		}
		u8Val = 0x30;
		err = wland_fil_set_cmd_data(ifp, WID_POWER_SAVE, &u8Val,
			sizeof(u8Val));
		if (err < 0) {
			WLAND_ERR("Set WID_POWER_SAVE failed, %d\n", err);
			goto done;
		}
		u8Val = 0;
		err = wland_fil_set_cmd_data(ifp, WID_QOS_ENABLE, &u8Val,
			sizeof(u8Val));
		if (err) {
			WLAND_ERR("wland_set_qos_enable failed! \n");
			goto done;
		}
	}
#ifdef WLAND_POWER_MANAGER
	if (ifp->drvr->sleep_flags & WLAND_SLEEP_ENABLE) {
		u8Val = MAX_FAST_PS;
		err = wland_fil_set_cmd_data(ifp, WID_POWER_MANAGEMENT, &u8Val,
			sizeof(u8Val));
		if (err < 0)
			goto done;
	}
	if (ifp->drvr->sleep_flags & WLAND_SLEEP_PREASSO) {
		u32Val = WIFI_PREASSO_SLEEP;
		err = wland_fil_set_cmd_data(ifp, WID_PREASSO_SLEEP, &u32Val,
			sizeof(u32Val));
		if (err < 0)
			goto done;
	}
	u8Val = 1;
	sdio_claim_host(sdiodev->func);
	err = sdioh_request_byte(sdiodev, SDIOH_WRITE,
		URSDIO_FUNC1_INT_TO_DEVICE, &u8Val);
	sdio_release_host(sdiodev->func);
	if (err) {
		WLAND_ERR("Write URSDIO_FUNC1_INT_TO_DEVICE failed!\n");
	}
#endif /* WLAND_POWER_MANAGER */
done:
	WLAND_DBG(DCMD, DEBUG, "Done(err:%d)\n", err);
	return err;
}

int wland_start_chip(struct wland_if *ifp, u8 device_role)
{
	struct wland_private *drvr = ifp->drvr;
	struct wland_sdio_dev *sdiodev = drvr->bus_if->bus_priv.sdio;
	struct wland_sdio *bus = sdiodev->bus;
	int err = -ENODEV;

	WLAND_DBG(DCMD, DEBUG, "Enter\n");
	if (!ifp) {
		WLAND_ERR("ifp Empty!\n");
		goto fail;
	}

	err = wland_bus_active(sdiodev->dev);
	if (err < 0) {
		WLAND_ERR("active bus failed!\n");
		goto fail;
	}

	/*
	 * sta mode
	 */
	if ((device_role == 0) || (device_role == 0xA0)) {
		drvr->dev_mode = false;
		/*
		 * set up
		 */
		if (device_role & 0xA0)
			drvr->p2p_enable = true;
		else
			drvr->p2p_enable = false;
	} else {		/* softap mode */

		u8 val = MAC_ROLE_AP;

		drvr->dev_mode = true;
		/*
		 * set up
		 */
		if (device_role & 0xA0)
			drvr->p2p_enable = true;
		else
			drvr->p2p_enable = false;

		err = wland_fil_set_cmd_data(ifp, WID_MAC_ROLE, &val,
			sizeof(u8));
		if (err < 0) {
			WLAND_ERR("set preamble failed, %d\n", err);
			goto fail;
		}
	}

	/*
	 * soft ap mac different with sta mac address
	 */
	if (drvr->dev_mode) {
		/*
		 * revert six bit of the byte
		 */
		ifp->mac_addr[5] ^= (1 << (5));
		memcpy(ifp->ndev->dev_addr, ifp->mac_addr, ETH_ALEN);
	}

	WLAND_DBG(DCMD, DEBUG, "Enter(device_role:0x%x)\n", device_role);

#ifdef WLAND_FPGA_SUPPORT
	WLAND_DBG(DCMD, TRACE, "FPGA Mode Enter\n");
#else /*WLAND_FPGA_SUPPORT */
	WLAND_DBG(DCMD, DEBUG, "%s PATCH Enter\n",
		check_test_mode()? "Test_mode" : "Nomal_mode");
	err = wland_sdio_trap_attach(drvr);
	if (err < 0) {
		WLAND_ERR("sdio_trap_attach failed!\n");
		goto fail;
	}
#endif /* WLAND_FPGA_SUPPORT */

	if (!check_test_mode()) {
		bus->intr = true;
		bus->poll = false;
#ifdef WLAND_SDIO_SUPPORT
		rda_mmc_set_sdio_irq(1, true);
#endif /*WLAND_SDIO_SUPPORT */
	}

	err = wland_preinit_cmds(ifp);
	if (err < 0)
		WLAND_ERR("preinit cmds failed!\n");

fail:
	WLAND_DBG(DCMD, TRACE, "Done(err:%d)\n", err);

	return err;
}
