
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
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
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

static const char wland_version_string[WLAND_VER_MAX][15] = {
	"Unknow chip",
	"RDA5990_D",
	"RDA5990_E",
	"RDA5991_D",
	"RDA5991_E",
	"RDA5991_F",
	"RDA5991_G",
};

static void wland_pkt_word_align(struct sk_buff *p)
{
	uint datalign = ALIGNMENT;
	uint offset = ((unsigned long) (p->data) & (datalign - 1));

	if (offset) {
		skb_reserve(p, (datalign - offset));
	}
}

/* Turn backplane clock on or off */
static int sdio_htclk(struct wland_sdio *bus, bool on)
{

	WLAND_DBG(SDIO, TRACE, "(%s): Enter\n", on ? " Open" : "Stop");

	if (on) {

		/*
		 * Mark clock available
		 */
		bus->clkstate = CLK_AVAIL;
		bus->activity = true;
		BUS_WAKE(bus);
		WLAND_DBG(SDIO, TRACE, "CLKCTL: turned ON\n");
	} else {

		bus->clkstate = CLK_NONE;
		bus->activity = false;
		WLAND_DBG(SDIO, TRACE, "CLKCTL: turned OFF\n");

	}

	WLAND_DBG(SDIO, TRACE, "(%s): Done.\n", on ? " Open" : "Stop");

	return 0;
}

int wland_chip_wake_up(struct wland_sdio *bus)
{
	int ret = 0;

#ifdef	WLAND_POWER_MANAGER
	u8 u8Val = 1;
#endif

	WLAND_DBG(SDIO, TRACE, "Enter\n");

	//Tx complete, check whether go to sleep.
	if (!bus->activity && (bus->sdiodev->card_sleep)) {
#ifdef	WLAND_POWER_MANAGER
		if (!check_test_mode() ) {
			WLAND_DBG(SDIO, TRACE,
				"WIFI chip wake up <<<<<<<<< \n");
			sdio_claim_host(bus->sdiodev->func);
			ret = sdioh_request_byte(bus->sdiodev, SDIOH_WRITE,
				URSDIO_FUNC1_INT_TO_DEVICE, &u8Val);
			sdio_release_host(bus->sdiodev->func);
			if (ret) {
				WLAND_ERR
					("Write URSDIO_FUNC1_INT_TO_DEVICE failed!\n");
			}
			bus->sdiodev->card_sleep = false;
			wland_sched_timeout(10);
		}
#endif
		if (!sdio_htclk(bus, true)) {
			WLAND_DBG(SDIO, TRACE, "WIFI chip waked up and MOD timer. \n");
			wland_sdio_wd_timer(bus, bus->save_ms);
		}
	}
	WLAND_DBG(SDIO, TRACE, "Done.\n");
	return ret;
}

int wland_chip_goto_sleep(struct wland_sdio *bus)
{
	int ret = 0;

#ifdef	WLAND_POWER_MANAGER
	u8 u8Val = URSDIO_FUNC1_HOST_TX_FLAG;
#endif
	struct wland_sdio_dev *sdiodev = bus->sdiodev;
	struct wland_bus *bus_if = sdiodev->bus_if;
	struct wland_private *drvr = bus_if->drvr;
	struct wland_if *ifp = drvr->iflist[0];
	struct wland_cfg80211_info *cfg;
	struct wland_cfg80211_connect_info *conn_info;

	WLAND_DBG(SDIO, TRACE, "Enter\n");

	if (bus_if->state == WLAND_BUS_DOWN) {
#ifdef	WLAND_POWER_MANAGER
		sdio_claim_host(bus->sdiodev->func);
		ret = sdioh_request_byte(bus->sdiodev, SDIOH_WRITE,
			URSDIO_FUNC1_INT_PENDING, &u8Val);
		sdio_release_host(bus->sdiodev->func);
		if (ret) {
			WLAND_ERR("Write URSDIO_FUNC1_INT_PENDING failed!\n");
		}
		bus->sdiodev->card_sleep = true;
#endif
		wland_sdio_wd_timer(bus, 0);
		return ret;
	}
	cfg = wiphy_priv(ifp->vif->wdev.wiphy);
	conn_info = &cfg->conn_info;

	//Tx complete, check whether go to sleep.
	if (bus->activity &&
		!check_test_mode() &&
		!test_bit(SCAN_STATUS_BUSY, &cfg->scan_status) &&
		!test_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state) &&
		!timer_pending(&conn_info->connect_restorework_timeout)) {
#ifdef	WLAND_POWER_MANAGER
		if (!check_test_mode()){
			WLAND_DBG(SDIO, TRACE,
				"WIFI chip enter sleep. >>>>>>>>>>> \n");
			sdio_claim_host(bus->sdiodev->func);
			ret = sdioh_request_byte(bus->sdiodev, SDIOH_WRITE,
				URSDIO_FUNC1_INT_PENDING, &u8Val);
			sdio_release_host(bus->sdiodev->func);
			if (ret) {
				WLAND_ERR
					("Write URSDIO_FUNC1_INT_PENDING failed!\n");
			}
			bus->sdiodev->card_sleep = true;
		}
#endif
		WLAND_DBG(SDIO, TRACE, "turn OFF clock and delete wd timer.\n");
		sdio_htclk(bus, false);
		wland_sdio_wd_timer(bus, 0);
	}else{
		BUS_WAKE(bus);
		wland_sdio_wd_timer(bus, bus->save_ms);
	}

	WLAND_DBG(SDIO, TRACE, "Done.\n");
	return ret;
}

/* Change idle/active SD state,Transition SD and backplane clock readiness */
int wland_sdio_clkctl(struct wland_sdio *bus, uint target)
{
	int ret = 0;
	uint oldstate = bus->clkstate;

	WLAND_DBG(SDIO, TRACE, "=========>OldState(%d),ExpectState(%d),Enter\n",
		bus->clkstate, target);
	if (bus->sdiodev->bus_if->state == WLAND_BUS_DOWN) {
		WLAND_ERR("bus_if is down, can't operate sdio clock!\n");
		ret = -1;
		return ret;
	}

	/*
	 * Early exit if we're already there
	 */
	if (bus->clkstate == target) {
		if (target == CLK_AVAIL) {
			BUS_WAKE(bus);
			wland_sdio_wd_timer(bus, bus->save_ms);
		}
		return ret;
	}

	switch (target) {
	case CLK_AVAIL:
		//Request SDIO clock
		/*
		 * Make sure SD clock is available
		 */
		WLAND_DBG(SDIO, TRACE, "CLK_NONE --> CLK_AVAIL\n");
		wland_chip_wake_up(bus);
		break;

	case CLK_NONE:
		//Remove SDIO clock
		/*
		 * Now remove the SD clock
		 */
		WLAND_DBG(SDIO, TRACE, "CLK_AVAIL --> CLK_NONE\n");
		wland_chip_goto_sleep(bus);
		break;
	}

	WLAND_DBG(SDIO, TRACE, "=========>OldState(%d)--->NewState(%d),Done\n",
		oldstate, bus->clkstate);

	return ret;
}

static struct sk_buff *wland_pkt_buf_get_suitable_skb(struct wland_sdio *bus,
	struct sk_buff *skb, u16 * len)
{
	struct sk_buff *skb2 = skb;
	u16 base_len = *len;
	u16 size = *len;
	int ret = 0;

	WLAND_DBG(SDIO, TRACE, "Enter, size=%d\n", size);

	//for sdio must 4 bytes align
	if (size & (ALIGNMENT - 1))
		size = roundup(size, ALIGNMENT);

	size = wland_get_align_size(bus, size);

	if (!skb) {		// just get a suitable skb.
		skb2 = wland_pkt_buf_get_skb(size + ALIGNMENT - 1);
		if (!skb2) {
			WLAND_ERR("couldn't allocate new %d-byte packet\n",
				size + ALIGNMENT - 1);
			ret = -ENOMEM;
		} else {
			wland_pkt_word_align(skb2);
		}
		goto done;
	} else if (size - base_len >= 3) {
		skb2 = wland_pkt_buf_get_skb(size);
		if (!skb2) {
			WLAND_ERR("couldn't allocate new %d-byte packet\n",
				size);
			ret = -ENOMEM;
			goto done;
		} else {
			wland_pkt_word_align(skb2);
		}
	} else if ((size - base_len < 3)
		&& !IS_ALIGNED((int) skb->data, ALIGNMENT)) {
		skb2 = wland_pkt_buf_get_skb(size + ALIGNMENT - 1);
		if (!skb2) {
			WLAND_ERR("couldn't allocate new %d-byte packet\n",
				size + ALIGNMENT - 1);
			ret = -ENOMEM;
			goto done;
		} else {
			wland_pkt_word_align(skb2);
		}

	}

done:
        if (skb2 ) {
	       *len = size;
		skb2->len = size;
		WLAND_DBG(SDIO, TRACE, "Done.\n");
		return skb2;
        } else {
                WLAND_DBG(SDIO, TRACE, "Done.\n");
                return NULL;
        }
}

/* Writes a HW/SW header into the packet and sends it. */

/* Assumes: (a) header space already there, (b) caller holds lock */
static int wland_sdio_txpkt(struct wland_sdio *bus, struct sk_buff *pkt)
{
	int ret = 0;
	u8 *frame;
	u16 len = 0, real_len = 0;
	struct wland_bus *bus_if = dev_get_drvdata(bus->sdiodev->dev);

	WLAND_DBG(SDIO, TRACE, "Enter(bus_state:%d)\n", bus_if->state);

	if (bus_if->state == WLAND_BUS_DOWN) {
		WLAND_ERR("Bus state is down and reject the pkt!\n");
		return -EINVAL;
	}

	/*
	 * Add Header
	 */
	PKTPUSH(bus_if->osh, pkt, WID_HEADER_LEN);

	/*
	 * Hardware tag: 2 byte len followed by 2 byte ~len check (all LE)
	 */
	frame = (u8 *) (pkt->data);
	len = pkt->len & CDC_DCMD_LEN_MASK;
	if (pkt->len > CDC_DCMD_LEN_MASK) {
		WLAND_ERR("pkt->len is over flow!\n");
		wland_pkt_buf_free_skb(pkt);
		return -EINVAL;
	}
	real_len = len;
	len |= (PKT_TYPE_REQ << CDC_DCMD_LEN_SHIFT);
	*(__le16 *) frame = cpu_to_le16(len);
	len = real_len;

	WLAND_DBG(SDIO, TRACE, "pkt->len=%x, frame:%x, addr(pkt->data)=%p\n", len,
		*(__le16 *) frame, pkt->data);

	if (len & (ALIGNMENT - 1))
		len = roundup(len, ALIGNMENT);

	len = wland_get_align_size(bus, len);

/*	 skb2 = wland_pkt_buf_get_suitable_skb(bus, pkt, &len);
	 if(skb2 != pkt){
	      memcpy(skb2->data, pkt->data, real_len);
	      wland_pkt_buf_free_skb(pkt);
	      pkt = skb2;
	      WLAND_DBG(SDIO, TRACE, "Get new pkt!  pkt->len=%d, addr(pkt->data)=%p\n", len, pkt->data);
	 }
*/

	WLAND_DBG(SDIO, TRACE,
		"real_len=%d, len=%d, pkt->len=%d, pkt->data:0x%p\n", real_len,
		len, pkt->len, pkt->data);
	WLAND_DUMP(TX_MSDU, pkt->data, real_len, "MSDU len:%Zu\n", real_len);
	ret = wland_sdio_send_pkt(bus, pkt, len);

	/*
	 * Add Header
	 */
	PKTPULL(bus_if->osh, pkt, WID_HEADER_LEN);

	wland_txcomplete(bus->sdiodev->dev, pkt, (ret == 0));
	return ret;
}

static uint wland_sdio_sendfromq(struct wland_sdio *bus)
{
	struct sk_buff *pkt;
	uint cnt = 0;
	unsigned long flags = 0;

	WLAND_DBG(BUS, TRACE, "Enter\n");

	/*
	 * Send frames until the limit or some other event
	 */
	dhd_os_sdlock_txq(bus, &flags);
	pkt = wland_pktq_mdeq(&bus->txq);
	dhd_os_sdunlock_txq(bus, &flags);
	if (pkt == NULL) {
		WLAND_ERR("pkt == NULL and go out.\n");
		goto done;
	}
	atomic_dec(&bus->tx_dpc_tskcnt);

	wland_sdio_txpkt(bus, pkt);
	WLAND_DBG(BUS, TRACE,
		"After wland_sdio_txpkt(), pktq len=%d, bus->tx_dpc_tskcnt=%d\n",
		bus->txq.len, atomic_read(&bus->tx_dpc_tskcnt));

	/*
	 * Deflow-control stack if needed
	 */
	if ((bus->sdiodev->bus_if->state == WLAND_BUS_DATA) && bus->txoff
		&& (bus->txq.len < TXLOW)) {
		bus->txoff = false;

		wland_txflowcontrol(bus->sdiodev->dev, false);
	}

done:
	WLAND_DBG(BUS, TRACE, "Done\n");
	return cnt;
}

static int wland_sdio_intr_set(struct wland_sdio_dev *sdiodev, bool enable)
{
	u8 val;
	int ret;

	WLAND_DBG(SDIO, TRACE, "Enter(interrupt %s)\n",
		enable ? "enable" : "disable");

	if (enable)
		val = 0x07;
	else
		val = 0x00;

	/*
	 * set chip interrupt
	 */
	ret = sdioh_request_byte(sdiodev, SDIOH_WRITE,
		URSDIO_FUNC1_REGISTER_MASK, &val);
	return ret;
}

static int wland_sdio_intr_get(struct wland_sdio_dev *sdiodev, u8 * intrstatus)
{
	int ret = 0;

	if (!intrstatus)
		return -EBADE;

	if (sdiodev->bus_if->state == WLAND_BUS_DOWN) {
		/*
		 * disable interrupt
		 */
		*intrstatus = 0;
		WLAND_ERR("Bus is down!\n");
	} else {
		ret = sdioh_request_byte(sdiodev, SDIOH_READ,
			URSDIO_FUNC1_INT_STATUS, intrstatus);
	}

	WLAND_DBG(SDIO, TRACE, "Enter(interrupt status: 0x%x)\n",
		(uint) * intrstatus);

	return ret;
}

static void wland_sdio_bus_stop(struct device *dev)
{
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
	struct wland_sdio *bus = sdiodev->bus;
	int ret;

	WLAND_DBG(SDIO, TRACE, "Enter\n");

	if (bus->watchdog_tsk) {
		send_sig(SIGTERM, bus->watchdog_tsk, 1);
		kthread_stop(bus->watchdog_tsk);
		bus->watchdog_tsk = NULL;
	}

	bus_if->state = WLAND_BUS_DOWN;
	/*
	 * Turn off the backplane clock (only)
	 */
	ret = down_interruptible(&bus->txclk_sem);
	if (ret)
		WLAND_ERR("Can not request bus->txclk_sem.wland_sdio_bus_stop\n");

	wland_sdio_clkctl(bus, CLK_NONE);
	if (!ret)
		up(&bus->txclk_sem);
	/*
	 * Clear the data packet queues
	 */
	wland_pktq_flush(&bus->txq, true, NULL, NULL);
	wland_pktq_flush(&bus->rxq, true, NULL, NULL);

	/*
	 * Clear rx control and wake any waiters
	 */
	spin_lock_bh(&bus->rxctl_lock);
	bus->rxlen = 0;
	spin_unlock_bh(&bus->rxctl_lock);

	dhd_os_ioctl_resp_wake(bus);
	WLAND_DBG(SDIO, TRACE, "Done.\n");
}

static s32 wland_handle_mac_status(struct wland_private *drvr,
	struct wland_event_msg *event_packet, u8 * pu8Buffer)
{
	u8 u8MsgType = 0, u8WidLen = 0, u8MacStatus, u8MsgID = 0;
	u16 u16MsgLen = 0, u16WidID = WID_NIL;

	/*
	 * parse type
	 */
	u8MsgType = pu8Buffer[0];

	/*
	 * Check whether the received message type is 'I'
	 */
	if (WLAND_WID_MSG_MAC_STATUS != u8MsgType) {
		WLAND_ERR("Received Message type incorrect.\n");
		return -EBADE;
	}

	/*
	 * Extract message ID
	 */
	u8MsgID = pu8Buffer[1];

	/*
	 * Extract message Length
	 */
	u16MsgLen = MAKE_WORD16(pu8Buffer[2], pu8Buffer[3]);

	/*
	 * Extract WID ID [expected to be = WID_STATUS]
	 */
	u16WidID = MAKE_WORD16(pu8Buffer[4], pu8Buffer[5]);

	if (u16WidID != WID_STATUS) {
		WLAND_ERR("Received Message wid incorrect.\n");
		return -EBADE;
	}

	/*
	 * Extract WID Length [expected to be = 1]
	 */
	u8WidLen = pu8Buffer[6];

	/*
	 * get the WID value [expected to be one of two values: either MAC_CONNECTED = (1) or MAC_DISCONNECTED = (0)]
	 */
	u8MacStatus = pu8Buffer[7];

	WLAND_DBG(EVENT, TRACE,
		"Received(u8MsgID:0x%x,u16MsgLen:0x%x,u16WidID:0x%x,u8WidLen:0x%x,u8MacStatus:0x%x)\n",
		u8MsgID, u16MsgLen, u16WidID, u8WidLen, u8MacStatus);

	event_packet->status = STATUS_SUCCESS;

	if (u8MacStatus == MAC_CONNECTED) {
		event_packet->event_code = WLAND_E_CONNECT_IND;
		WLAND_DBG(EVENT, DEBUG, "MAC CONNECTED\n");
	} else if (u8MacStatus == MAC_DISCONNECTED) {
		WLAND_DBG(EVENT, DEBUG, "MAC_DISCONNECTED\n");
		event_packet->event_code = WLAND_E_DISCONNECT_IND;
	} else {
		WLAND_ERR("Invalid MAC Status 0x%02x\n", u8MacStatus);
		return -EBADE;
	}

	firmweh_push_event(drvr, event_packet, pu8Buffer);

	return 0;
}

static s32 wland_handle_network_link_event(struct wland_private *drvr,
	struct wland_event_msg *event_packet, u8 * pu8Buffer)
{
	u8 u8MsgType = 0, u8EventType = 0;
	u16 u16EventLen = 0;
	s32 ret = 0;

	/*
	 * parse type
	 */
	u8MsgType = pu8Buffer[0];

	/*
	 * Check whether the received message type is 'I'
	 */
	if (WLAND_WID_MSG_EVENT != u8MsgType) {
		WLAND_ERR("Received Message type incorrect.\n");
		return -EBADE;
	}

	/*
	 * Extract event Type
	 */
	u8EventType = pu8Buffer[1];

	/*
	 * Extract event Length
	 */
	u16EventLen = MAKE_WORD16(pu8Buffer[2], pu8Buffer[3]);

	WLAND_DBG(EVENT, DEBUG,
		"Received(u8MsgType:0x%x, u8EventType:%d, u16EventLen:%d \n",
		u8MsgType, u8EventType, u16EventLen);

	event_packet->action = u8EventType;
	switch (u8EventType) {
		WLAND_DBG(EVENT, DEBUG, "u8EventType=%d\n", u8EventType);
	case EVENT_AUTH_IND:
		WLAND_DBG(EVENT, DEBUG, "EVENT_AUTH_IND\n");
		event_packet->event_code = WLAND_E_CONNECT_IND;
		event_packet->status = STATUS_SUCCESS;
		break;
	case EVENT_DEAUTH_IND:
		WLAND_DBG(EVENT, DEBUG, "EVENT_DEAUTH_IND\n");
		event_packet->event_code = WLAND_E_DISCONNECT_IND;
		break;
	case EVENT_ASSOC_IND:
		WLAND_DBG(EVENT, DEBUG, "EVENT_ASSOC_IND\n");
		event_packet->event_code = WLAND_E_CONNECT_IND;
		event_packet->status = STATUS_SUCCESS;
		memcpy(event_packet->addr, &pu8Buffer[16], ETH_ALEN);
		break;
	case EVENT_REASSOC_IND:
		WLAND_DBG(EVENT, DEBUG, "EVENT_REASSOC_IND\n");
		event_packet->event_code = WLAND_E_CONNECT_IND;
		event_packet->status = STATUS_SUCCESS;
		break;
	case EVENT_DISASSOC_IND:
		WLAND_DBG(EVENT, DEBUG, "EVENT_DISASSOC_IND\n");
		event_packet->event_code = WLAND_E_DISCONNECT_IND;
		break;
	default: {
		ret = -EBADE;
		WLAND_ERR("Receive invalid event type!\n");
		break;
	}
	}

	firmweh_push_event(drvr, event_packet, pu8Buffer);

	return ret;
}

/*
 * wland_sdio_readframes() - just process skb as firmware event.
 *
 * If the packet buffer contains a firmware event message it will
 * dispatch the event to a registered handler (using worker).
 */
struct sk_buff *wland_sdio_readframes(struct wland_sdio *bus)
{
	int ret = 0;
	u8 size_l = 0, size_h = 0;
	u16 rcv_len = 0;
	u16 size = 0;
	struct sk_buff *skb = NULL;
	struct wland_bus *bus_if = dev_get_drvdata(bus->sdiodev->dev);

	WLAND_DBG(DEFAULT, TRACE, "Enter\n");
	if (bus_if->state == WLAND_BUS_DOWN) {
		WLAND_ERR("Bus is down and go out!\n");
		goto done;
	}

	ret = sdioh_request_byte(bus->sdiodev, SDIOH_READ,
		URSDIO_FUNC1_RPKTLEN_LO, &size_l);
	if (ret) {
		WLAND_ERR("Read SDIO_AHB2SDIO_PKTLEN_L failed!\n");
		goto done;
	}
	ret = sdioh_request_byte(bus->sdiodev, SDIOH_READ,
				URSDIO_FUNC1_RPKTLEN_HI, &size_h);
	if (ret) {
		WLAND_ERR("Read SDIO_AHB2SDIO_PKTLEN_H failed!\n");
		goto done;
	}
	size = (size_l | ((size_h & 0x7F) << 8)) * 4;

	if ((size > WLAND_MAX_BUFSZ) || (size < FMW_HEADER_LEN)) {
		WLAND_ERR("received buffer is invalid(size:%d) and go out.\n",
			size);
		goto done;
	}

	WLAND_DBG(SDIO, TRACE, "received buffer size:%d.\n", size);

	skb =  dev_alloc_skb(size + NET_IP_ALIGN + WID_HEADER_LEN + 3);
	if(!skb){
		WLAND_ERR("dev_alloc_skb alloc skb failed \n");
		goto done;
	}

	skb_reserve(skb, NET_IP_ALIGN);
	//4byte align
	wland_pkt_word_align(skb);

	ret = wland_sdio_recv_pkt(bus, skb, size);
	if (ret) {
		WLAND_ERR("receive skb failed\n");
		dev_kfree_skb(skb);
		skb = NULL;
		goto done;
	}
	rcv_len = (u16)(skb->data[0] | ((skb->data[1]&0x0f) << 8));
	if(rcv_len > size){
		WLAND_ERR("SDIO read payload_len invalid! \n");
		dev_kfree_skb(skb);
		skb = NULL;
		goto done;
	}
	skb_put(skb, rcv_len);

done:
	WLAND_DBG(DEFAULT, TRACE, "Done\n");
	return skb;
}

static int wland_sdio_process_rxframes(struct wland_sdio *bus)
{
	int ret = 0;
	u8 rx_type, msg_type, *buf;
	u16 size = 0, rx_len;
	struct sk_buff *skb = NULL;	/* Packet for event or data frames */
	struct wland_event_msg event_packet;
	struct wland_bus *bus_if = dev_get_drvdata(bus->sdiodev->dev);
	unsigned long flags = 0;

	WLAND_DBG(DEFAULT, TRACE, "Enter\n");
	if (bus_if->state == WLAND_BUS_DOWN) {
		WLAND_ERR("Bus down 3, ret\n");
		return ret;
	}

	while (wland_pktq_mlen(&bus->rxq, ~bus->flowcontrol)) {
		dhd_os_sdlock_rxq(bus, &flags);
		skb = wland_pktq_mdeq(&bus->rxq);
		dhd_os_sdunlock_rxq(bus, &flags);
		atomic_dec(&bus->rx_dpc_tskcnt);
		if (skb == NULL) {
			break;
		}

		buf = PKTDATA(bus_if->osh, skb);
		//WLAND_DUMP(SDIO, skb->data, skb->len, "RX Data, len:%Zu\n", skb->len);

		/*
		 * pull hdr from the skb
		 */
		size = skb->len;
		rx_len = (u16) (buf[0] | ((buf[1] & 0x0F) << 8));
		WLAND_DBG(SDIO, TRACE, "size=%d, rx_len=%d, addr:%x\n", size, rx_len, (unsigned int)buf);
		if (rx_len > size) {
			WLAND_ERR("SDIO read payload_len invalid! \n");
			wland_pkt_buf_free_skb(skb);
			return -EIO;
		}
		rx_type = (u8) buf[1] >> 4;
		//skb->len = rx_len;

		if (rx_type == PKT_TYPE_CFG_RSP) {
			msg_type = buf[2];

			memset(&event_packet, 0, sizeof(event_packet));

			/*
			 * offset frame hdr
			 */
			event_packet.datalen = rx_len - WID_HEADER_LEN;

			buf += WID_HEADER_LEN;

			switch (msg_type) {
			case WLAND_WID_MSG_RESP:
				WLAND_DBG(EVENT, TRACE,
					"Receive response(%s:total_len:%u,rx_len:%u,rx_type:%u)\n",
					dev_name(bus->sdiodev->dev),
					skb->len, rx_len, rx_type);
				WLAND_DUMP(RX_WIDRSP, skb->data, skb->len,
					"RX Data (WID_MSG_RESP), len:%Zu\n", skb->len);
				spin_lock_bh(&bus->rxctl_lock);
				bus->rxctl = bus->rxbuf;
				bus->rxlen = rx_len;
				memcpy(bus->rxctl, skb->data, rx_len);
				spin_unlock_bh(&bus->rxctl_lock);
				dhd_os_ioctl_resp_wake(bus);
				wland_pkt_buf_free_skb(skb);
				break;
			case WLAND_WID_MSG_NETINFO:
				WLAND_DBG(EVENT, TRACE,
					"Receive info notify(%s:total_len:%u,rx_len:%u,rx_type:%u)\n",
					dev_name(bus->sdiodev->dev),
					skb->len, rx_len, rx_type);
				WLAND_DUMP(RX_NETINFO, skb->data, skb->len,
					"RX Data (WID_MSG_NETINFO), len:%Zu\n",
					skb->len);
				event_packet.event_code = WLAND_E_ESCAN_RESULT;
				event_packet.status = STATUS_SUCCESS;
				firmweh_push_event(bus_if->drvr, &event_packet, buf);
				wland_pkt_buf_free_skb(skb);
				break;
			case WLAND_WID_MSG_EVENT:
				WLAND_DBG(EVENT, DEBUG,
					"Receive Network event(%s:total_len:%u,rx_len:%u,rx_type:%u)\n",
					dev_name(bus->sdiodev->dev),
					skb->len, rx_len, rx_type);
				WLAND_DUMP(RX_NETEVENT, skb->data, skb->len,
					"RX Data (WID_MSG_NETEVENT), len:%Zu\n",
					skb->len);
				wland_handle_network_link_event
					(bus_if->drvr, &event_packet,buf);
				wland_pkt_buf_free_skb(skb);
				break;
			case WLAND_WID_MSG_MAC_STATUS:
				WLAND_DBG(EVENT, TRACE,
					"Receive mac status notify(%s:total_len:%u,rx_len:%u,rx_type:%u)\n",
					dev_name(bus->sdiodev->dev),
					skb->len, rx_len, rx_type);
				WLAND_DUMP(RX_MACSTAT, skb->data, skb->len,
					"RX Data (WID_MSG_MAC_STATUS), len:%Zu\n",
					skb->len);
				wland_handle_mac_status(bus_if->drvr, &event_packet, buf);
				wland_pkt_buf_free_skb(skb);
				break;
			default:
				WLAND_ERR("receive invalid frames!\n");
				wland_pkt_buf_free_skb(skb);
				ret = -EBADE;
				break;
			}
		} else {
			WLAND_DBG(EVENT, TRACE, "MSDU data in.\n");
			WLAND_DUMP(RX_MSDU, skb->data, skb->len,
				"RX Data (BIN DATA), len:%Zu\n", skb->len);
			PKTPULL(bus_if->osh, skb, WID_HEADER_LEN);
			wland_rx_frames(bus->sdiodev->dev, skb);
		}

		WLAND_DBG(BUS, TRACE,
			"Process rxframes, bus->rx_dpc_tskcnt=%d\n",
			atomic_read(&bus->rx_dpc_tskcnt));
	}
	return ret;
}

static int wland_sdio_txctl_frames(struct wland_sdio *bus)
{
	int err = 0;
	u16 payloadLen, nbytes;
	u8 *payload = bus->ctrl_frame_buf;
	struct sk_buff *pkt = NULL;

	WLAND_DBG(DEFAULT, TRACE, "Enter\n");
	nbytes = bus->ctrl_frame_len;
	payloadLen = bus->ctrl_frame_len;

	WLAND_DBG(DEFAULT, TRACE, "bus->ctrl_frame_len=%d\n",
		bus->ctrl_frame_len);
	pkt = wland_pkt_buf_get_suitable_skb(bus, NULL, &nbytes);
	if (!pkt) {
		WLAND_ERR("get pkt failed,len: %d\n", nbytes);
		return -ENOMEM;
	} else {
		dhd_os_sdlock(bus);
		memcpy(pkt->data, payload, payloadLen);
		dhd_os_sdunlock(bus);
	}

	WLAND_DBG(DEFAULT, TRACE, "payloadLen:%d, nbytes:%d, pkt->data=%p\n",
		payloadLen, nbytes, pkt->data);
	WLAND_DUMP(TX_CTRL, pkt->data, nbytes, "TX ctrl nbytes:%Zu\n", nbytes);

	err = wland_sdio_send_pkt(bus, pkt, nbytes);
	wland_pkt_buf_free_skb(pkt);

	dhd_os_sdlock(bus);
	bus->ctrl_frame_stat = false;
	if (!err)
		bus->ctrl_frame_send_success= true;
	else
		bus->ctrl_frame_send_success= false;
	dhd_os_sdunlock(bus);

	WLAND_DBG(BUS, TRACE, "Done(err:%d)\n", err);

	return err;
}

static void wland_sdio_tx_dpc(struct wland_sdio *bus)
{
	int err = 0;

	WLAND_DBG(BUS, TRACE, "Enter\n");

	if (bus->sdiodev->bus_if->state == WLAND_BUS_DOWN) {
		WLAND_ERR("Bus is down and go out.\n");
		goto done;
	}

	while (wland_pktq_mlen(&bus->txq, ~bus->flowcontrol)) {
		/* Send queued frames (limit 1 if rx may still be pending) */
		WLAND_DBG(BUS, TRACE, "TXQ_len = %d, tx_dpc_tskcnt=%d\n",
			wland_pktq_mlen(&bus->txq, ~bus->flowcontrol),
			atomic_read(&bus->tx_dpc_tskcnt));
		WLAND_DBG(BUS, TRACE, "******SendData.\n");
		if (down_interruptible(&bus->txclk_sem)) {
		    WLAND_ERR("Can not request bus->txclk_sem.1 \n");
		    goto done;
		}
		wland_sdio_clkctl(bus, CLK_AVAIL);
		if (bus->clkstate != CLK_AVAIL) {
			WLAND_ERR("Can not request SDMMC clock and go out.\n");
			up(&bus->txclk_sem);
			goto done;
		}
		wland_sdio_sendfromq(bus);
		up(&bus->txclk_sem);
		}

		if (bus->ctrl_frame_stat) {
			if (down_interruptible(&bus->txclk_sem)) {
			WLAND_ERR("Can not request bus->txclk_sem.2 \n");
			goto done;
		}
		wland_sdio_clkctl(bus, CLK_AVAIL);
		if (bus->clkstate != CLK_AVAIL) {
			WLAND_ERR("Can not request clock and go out.\n");
			up(&bus->txclk_sem);
			goto done;
		}
		err = wland_sdio_txctl_frames(bus);
		up(&bus->txclk_sem);
		if (err < 0)
			bus->sdcnt.tx_sderrs++;
		dhd_os_wait_event_wakeup(bus);
		atomic_dec(&bus->tx_dpc_tskcnt);
		WLAND_DBG(BUS, TRACE,
			"Processing TXCTL. bus->tx_dpc_tskcnt=%d\n",
			atomic_read(&bus->tx_dpc_tskcnt));
	}

done:
	WLAND_DBG(BUS, TRACE, "Done(bus->tx_dpc_tskcnt:%d)\n",
		atomic_read(&bus->tx_dpc_tskcnt));
	//dhd_os_sdunlock(bus);
}

static void wland_sdio_rx_dpc(struct wland_sdio *bus)
{

	WLAND_DBG(BUS, TRACE, "Enter\n");

	if (bus->sdiodev->bus_if->state == WLAND_BUS_DOWN) {
		WLAND_ERR("Bus is down and go out.\n");
		goto done;
	}

	wland_sdio_process_rxframes(bus);

done:
	WLAND_DBG(BUS, TRACE, "Done\n");
}

static struct pktq *wland_sdio_bus_gettxq(struct device *dev)
{
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_sdio *bus = bus_if->bus_priv.sdio->bus;

	return &bus->txq;
}

/* Conversion of 802.1D priority to precedence level */
static uint prio2prec(u32 prio)
{
	return (prio == PRIO_8021D_NONE
		|| prio == PRIO_8021D_BE) ? (prio ^ 2) : prio;
}

static int wland_sdio_bus_txdata(struct device *dev, struct sk_buff *pkt)
{
	uint prec;
	int ret = -EBADE;
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
	struct wland_sdio *bus = sdiodev->bus;
	unsigned long flags = 0;

	WLAND_DBG(SDIO, TRACE, "Enter\n");

	/*
	 * precondition: IS_ALIGNED((unsigned long)(pkt->data), 2)
	 */
	prec = prio2prec((pkt->priority & PRIOMASK));

	/*
	 * Check for existing queue, current flow-control, pending event, or pending clock
	 */
	WLAND_DBG(SDIO, TRACE, "deferring pktq len:%d,prec:%d.\n", bus->txq.len,
		prec);

	/*
	 * Priority based enq
	 */
	dhd_os_sdlock_txq(bus, &flags);
	if (!wland_prec_enq(bus->sdiodev->dev, &bus->txq, pkt, prec)) {
		dhd_os_sdunlock_txq(bus, &flags);
		wland_txcomplete(bus->sdiodev->dev, pkt, false);
		WLAND_ERR("bus->txq is over flow!!!\n");
		return -ENOSR;
	} else {
		ret = 0;
	}

	if (bus_if->state != WLAND_BUS_DATA) {
		WLAND_ERR("bus has stop\n");
		dhd_os_sdunlock_txq(bus, &flags);
		return -1;
	}

	dhd_os_sdunlock_txq(bus, &flags);
	WAKE_TX_WORK(bus);

	if (bus->txq.len >= TXHI) {
		bus->txoff = true;
		wland_txflowcontrol(bus->sdiodev->dev, true);
	}

	WLAND_DBG(SDIO, TRACE, "TXDATA Wake up DPC work, pktq len:%d\n",
		bus->txq.len);
	WLAND_DBG(SDIO, TRACE,
		"TX Data Wake up TX DPC work,  bus->tx_dpc_tskcnt:%d,  pktq len:%d\n",
		atomic_read(&bus->tx_dpc_tskcnt), bus->txq.len);

	return ret;
}

static int wland_sdio_bus_txctl(struct device *dev, u8 * msg, uint msglen)
{
	int ret = -1;
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
	struct wland_sdio *bus = sdiodev->bus;

	WLAND_DBG(SDIO, TRACE, "Enter\n");

	/*
	 * Need to lock here to protect txseq and SDIO tx calls
	 */
	dhd_os_sdlock(bus);

	bus->ctrl_frame_stat = true;
	bus->ctrl_frame_send_success = false;
	/*
	 * Send from dpc
	 */
	bus->ctrl_frame_buf = msg;
	bus->ctrl_frame_len = msglen;
	dhd_os_sdunlock(bus);
	if (bus_if->state != WLAND_BUS_DATA) {
		WLAND_ERR("bus has stop\n");
		return -1;
	}
	WAKE_TX_WORK(bus);

	WLAND_DBG(BUS, TRACE,
		"TXCTL Wake up TX DPC work,  bus->tx_dpc_tskcnt:%d\n",
		atomic_read(&bus->tx_dpc_tskcnt));
	if (bus->ctrl_frame_stat)
		dhd_os_wait_for_event(bus, &bus->ctrl_frame_stat);

	if (!bus->ctrl_frame_stat && bus->ctrl_frame_send_success) {
		WLAND_DBG(SDIO, TRACE,
			"ctrl_frame_stat == false, send success\n");
		ret = 0;
	} else if(!bus->ctrl_frame_stat && !bus->ctrl_frame_send_success){
		WLAND_DBG(SDIO, INFO, "ctrl_frame_stat == true, send failed\n");
		ret = -1;
	}
	if (ret)
		bus->sdcnt.tx_ctlerrs++;
	else
		bus->sdcnt.tx_ctlpkts++;

	return ret ? -EIO : 0;
}

static int wland_sdio_bus_rxctl(struct device *dev, u8 * msg, uint msglen)
{
	int timeleft;
	uint rxlen = 0;
	bool pending = false;
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
	struct wland_sdio *bus = sdiodev->bus;

	WLAND_DBG(SDIO, TRACE, "Enter\n");

	/*
	 * Wait until control frame is available
	 */
	timeleft = dhd_os_ioctl_resp_wait(bus, &bus->rxlen, &pending);

	if (bus->rxlen > 0) {
		spin_lock_bh(&bus->rxctl_lock);
		rxlen = bus->rxlen;
		memcpy(msg, bus->rxctl, min(msglen, rxlen));
		bus->rxlen = 0;
		spin_unlock_bh(&bus->rxctl_lock);
	}

	if (rxlen) {
		WLAND_DBG(SDIO, TRACE,
			"resumed on rxctl frame, got %d expected %d\n", rxlen,
			msglen);
	} else if (timeleft == 0) {
		WLAND_ERR("resumed on timeout\n");
	} else if (pending) {
		WLAND_DBG(SDIO, DEBUG, "cancelled\n");
		return -ERESTARTSYS;
	} else {
		WLAND_DBG(SDIO, DEBUG, "resumed for unknown reason\n");
	}

	if (rxlen)
		bus->sdcnt.rx_ctlpkts++;
	else
		bus->sdcnt.rx_ctlerrs++;

	return rxlen ? (int) rxlen : -ETIMEDOUT;
}

static int wland_sdio_bus_init(struct device *dev)
{
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
	struct wland_sdio *bus = sdiodev->bus;
	int ret = 0;

	WLAND_DBG(BUS, TRACE, "Enter\n");

	/*
	 * Start the watchdog timer
	 */
	bus->sdcnt.tickcnt = 0;

	ret = wland_sdio_intr_register(bus->sdiodev);
	if (ret != 0)
		WLAND_ERR("intr register failed:%d\n", ret);

	bus_if->state = WLAND_BUS_DATA;

	WLAND_DBG(BUS, TRACE, "Done\n");
	return ret;
}

/* sdio read and write worker */
static void wland_sdio_TxWorker(struct work_struct *work)
{
	struct wland_sdio *bus = container_of(work, struct wland_sdio, TxWork);

	WLAND_DBG(BUS, TRACE, "Enter\n");

	if (atomic_read(&bus->tx_dpc_tskcnt) > 0) {
		wland_sdio_tx_dpc(bus);

	}
}

static void wland_sdio_RxWorker(struct work_struct *work)
{
	struct wland_sdio *bus = container_of(work, struct wland_sdio, RxWork);

	if (atomic_read(&bus->rx_dpc_tskcnt) > 0) {
		wland_sdio_rx_dpc(bus);

	}
}


static int wland_sdio_watchdog_thread(void *data)
{
	struct wland_sdio *bus = (struct wland_sdio *) data;
	struct sk_buff *pkt = NULL;
	unsigned long flags = 0;

	allow_signal(SIGTERM);

	/*
	 * Run until signal received
	 */
	while (1) {
		if (kthread_should_stop()) {
			wland_sdio_wd_timer(bus, 0);
			WLAND_DBG(BUS, ERROR, "watchdog thread stoped.\n");
			break;
		}
		if (!wait_for_completion_interruptible(&bus->watchdog_wait)) {
			u8 intstatus = 0;

			WLAND_DBG(BUS, TRACE,
				"(bus->poll:%d,bus->polltick:%d,bus->pollrate:%d)\n",
				bus->poll, bus->polltick, bus->pollrate);

			SMP_RD_BARRIER_DEPENDS();

			//In poll mode
			/*
			 * Poll period: check device if appropriate.
			 */
			if (bus->poll && (++bus->polltick >= bus->pollrate)) {
				/*
				 * Reset poll tick
				 */
				bus->polltick = 0;
				WLAND_DBG(BUS, TRACE,
					"(bus->intr:%d,bus->sdcnt.intrcount:%d,bus->sdcnt.lastintrs:%d)\n",
					bus->intr, bus->sdcnt.intrcount,
					bus->sdcnt.lastintrs);

				/*
				 * Check device if no interrupts
				 */
				if (!bus->intr
					|| (bus->sdcnt.intrcount ==
						bus->sdcnt.lastintrs)) {
					sdio_claim_host(bus->sdiodev->func);
					if (wland_sdio_intr_get(bus->sdiodev,
							&intstatus) < 0) {
						WLAND_ERR("read status failed!\n");
					}

					/*
					 * If there is something, make like the ISR and schedule the DPC
					 */
					if (intstatus & I_AHB2SDIO) {
						WLAND_DBG(BUS, TRACE,
							"Frame Ind!\n");
						bus->sdcnt.pollcnt++;
						pkt = wland_sdio_readframes(bus);
						sdio_release_host(bus->sdiodev->func);
						if (pkt) {
							dhd_os_sdlock_rxq(bus, &flags);
							if(!wland_prec_enq(bus->sdiodev->dev,
								&bus->rxq, pkt,
								prio2prec((pkt->priority & PRIOMASK)))){
								dhd_os_sdunlock_rxq(bus, &flags);
								WLAND_ERR("bus->rxq is over flow!!!\n");
								continue;
							}
							dhd_os_sdunlock_rxq(bus, &flags);
							WAKE_RX_WORK(bus);
							WLAND_DBG(BUS, TRACE,
								"Watch dog wake up RX Work, bus->rx_dpc_tskcnt=%d\n",
								atomic_read(&bus->rx_dpc_tskcnt));
						}

					} else
						sdio_release_host(bus->sdiodev->func);

				}

				/*
				 * Update interrupt tracking
				 */
				bus->sdcnt.lastintrs = bus->sdcnt.intrcount;
			}

			WLAND_DBG(BUS, TRACE,
				"(bus->activity:%d,bus->idlecount:%d,bus->idletime:%d,bus->clkstate:%d)\n",
				bus->activity, bus->idlecount, bus->idletime,
				bus->clkstate);
			/*
			 * On idle timeout clear activity flag and/or turn off clock
			 */
			if ((bus->idletime > 0) && (bus->clkstate == CLK_AVAIL)) {
				WLAND_DBG(BUS, TRACE,
					"bus->idletime=%d, bus->idlecount=%d\n",
					bus->idletime, bus->idlecount);
				if (++bus->idlecount >= bus->idletime) {
					if (down_interruptible(&bus->txclk_sem)) {
						WLAND_ERR("Can not request bus->txclk_sem.watchdaothread \n");
						continue;
					}
					wland_sdio_clkctl(bus, CLK_NONE);
					up(&bus->txclk_sem);
				} else {
					if(!timer_pending(&bus->timer))
						wland_sdio_wd_timer(bus, bus->save_ms);
				}
			}
			flags = dhd_os_spin_lock(bus);
			/*
			 * Count the tick for reference
			 */
			bus->sdcnt.tickcnt++;

			dhd_os_spin_unlock(bus, flags);

		} else {
			WLAND_DBG(BUS, INFO,
				"<WDOG-TRD>watchdog thread no signal.\n");
			continue;
		}
	}
	return 0;
}

static void wland_bus_watchdog(ulong data)
{
	struct wland_sdio *bus = (struct wland_sdio *) data;

	WLAND_DBG(BUS, TRACE, "=======*****=====>Enter\n");

	if (bus->sdiodev->bus_if->state == WLAND_BUS_DOWN) {
		WLAND_DBG(BUS, TRACE,
			"=======*****=====>(bus_if->state == WLAND_BUS_DOWN)\n");

		/*
		 * Clear rx control and wake any waiters
		 */
		dhd_os_ioctl_resp_wake(bus);
		return;
	}

	if (bus->watchdog_tsk){
		WLAND_DBG(BUS, TRACE, "Wake up watchdog thread!\n");
		complete(&bus->watchdog_wait);
	}
}

static void wland_sdioh_irqhandler(struct sdio_func *func)
{
	struct wland_bus *bus_if = dev_get_drvdata(&func->dev);
	struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
	struct wland_sdio *bus = sdiodev->bus;
	u8 intstatus = 0;
	uint prec;
	struct sk_buff *pkt = NULL;
	unsigned long flags = 0;

	if (!bus_if) {
		WLAND_ERR("bus is null pointer, exiting\n");
		return;
	}

	if (bus_if->state == WLAND_BUS_DOWN) {
		WLAND_ERR("bus is down. we have nothing to do\n");
		return;
	}

	/*
	 * Disable additional interrupts
	 */
	if (!bus->intr) {
		WLAND_DBG(SDIO, INFO,
			"isr w/o interrupt is disabled, so do nothing and return\n");
		return;
	}

	/*
	 * Count the interrupt call
	 */
	bus->sdcnt.intrcount++;

	bus->intdis = true;

	wland_sdio_intr_get(bus->sdiodev, &intstatus);

	atomic_set(&bus->intstatus, intstatus);

	WLAND_DBG(BUS, TRACE, "sdio_intstatus:%x\n", intstatus);

	/*
	 * On frame indication, read available frames
	 */
	if (intstatus & I_AHB2SDIO) {
		pkt = wland_sdio_readframes(bus);
	} else if (intstatus & I_ERROR) {
		u8 val = I_ERROR;

		sdioh_request_byte(bus->sdiodev, SDIOH_WRITE,
			URSDIO_FUNC1_INT_PENDING, &val);
		WLAND_ERR("int_error!\n");
	} else {
		WLAND_DBG(BUS, DEBUG,
			"No Interrupt(bus->clkstate:%d,bus->ctrl_frame_stat:%d).\n",
			bus->clkstate, bus->ctrl_frame_stat);
	}

	if (pkt) {
		prec = prio2prec((pkt->priority & PRIOMASK));
		dhd_os_sdlock_rxq(bus, &flags);
		if(!wland_prec_enq(bus->sdiodev->dev, &bus->rxq, pkt, prec)){
			dhd_os_sdunlock_rxq(bus, &flags);
			WLAND_ERR("bus->rxq is over flow!!!\n");
			wland_pkt_buf_free_skb(pkt);
			return;
		}
		dhd_os_sdunlock_rxq(bus, &flags);
		WLAND_DBG(BUS, TRACE,
				"IRQ Wake up RX Work, bus->rx_dpc_tskcnt=%d\n",
				atomic_read(&bus->rx_dpc_tskcnt));
		WLAND_DBG(BUS, TRACE,"rxq_len=%d\n", wland_pktq_mlen(&bus->rxq, ~bus->flowcontrol));
		WAKE_RX_WORK(bus);
	}
	WLAND_DBG(SDIO, TRACE,
		"IRQ schedule work,  bus->rx_dpc_tskcnt:%d, Done\n",
		atomic_read(&bus->rx_dpc_tskcnt));
}

int wland_sdio_intr_register(struct wland_sdio_dev *sdiodev)
{
	int ret;

	sdio_claim_host(sdiodev->func);
	sdio_claim_irq(sdiodev->func, wland_sdioh_irqhandler);
	ret = wland_sdio_intr_set(sdiodev, true);
	sdio_release_host(sdiodev->func);

	WLAND_DBG(SDIO, TRACE, "Enter(ret:%d)\n", ret);

	return ret;
}

int wland_sdio_intr_unregister(struct wland_sdio_dev *sdiodev)
{

#ifdef WLAND_SDIO_SUPPORT
	rda_mmc_set_sdio_irq(1, false);
#endif /* WLAND_SDIO_SUPPORT    */

	/*
	 * disable interrupt
	 */
	sdio_claim_host(sdiodev->func);
	wland_sdio_intr_set(sdiodev, false);
	sdio_release_irq(sdiodev->func);
	sdio_release_host(sdiodev->func);

	WLAND_DBG(SDIO, TRACE, "Done\n");
	return 0;
}

void wland_sdio_wd_timer(struct wland_sdio *bus, uint wdtick)
{
	ulong flags;
	uint timeout;

	WLAND_DBG(BUS, TRACE, "------------>Enter(wdtick:%d)\n", wdtick);

	if (!bus)
		return;

	if (wdtick)
		dhd_os_wd_wake_lock(bus);

	flags = dhd_os_spin_lock(bus);

	/*
	 * don't start the wd until fw is loaded
	 */
	if (bus->sdiodev->bus_if->state == WLAND_BUS_DOWN && wdtick) {
		dhd_os_spin_unlock(bus, flags);
		dhd_os_wd_wake_unlock(bus);
		WLAND_DBG(BUS, INFO,
			"------------>Done(bus_if->state == WLAND_BUS_DOWN)\n");
		return;
	}

	/*
	 * Totally stop the timer
	 */
	if (!wdtick) {
		if (timer_pending(&bus->timer)) {
			WLAND_DBG(BUS, TRACE, "delete timer bus->timer!\n");
			del_timer_sync(&bus->timer);
		}
		bus->wd_timer_valid = false;
		dhd_os_spin_unlock(bus, flags);
		dhd_os_wd_wake_unlock(bus);
		WLAND_DBG(BUS, TRACE, "Watchdog timer release!\n");
		return;
	}

	if (wdtick) {
		bus->save_ms = wdtick;
		/*
		 * Convert timeout in millsecond to jiffies
		 */
		timeout = msecs_to_jiffies(bus->save_ms);
		bus->wd_timer_valid = true;
		/*
		 * Re arm the timer, at last watchdog period
		 */
		mod_timer(&bus->timer, jiffies + timeout);
		WLAND_DBG(BUS, TRACE, "reset watch dog timer(timer bus->timer)! timeout=%d\n",
			bus->save_ms);
	}

	dhd_os_spin_unlock(bus, flags);

	WLAND_DBG(BUS, TRACE, "------------>Done(bus->save_ms:%d)\n",
		bus->save_ms);
}

static struct wland_bus_ops wland_sdio_bus_ops = {
	.stop = wland_sdio_bus_stop,
	.init = wland_sdio_bus_init,
	.txdata = wland_sdio_bus_txdata,
	.txctl = wland_sdio_bus_txctl,
	.rxctl = wland_sdio_bus_rxctl,
	.gettxq = wland_sdio_bus_gettxq,
};

/* Detach and free everything */
void wland_sdio_release(struct wland_sdio *bus)
{
	struct wland_bus *bus_if;

	WLAND_DBG(SDIO, TRACE, "Enter\n");

	if (!bus) {
		WLAND_ERR("bus empty!\n");
		return;
	}

	bus_if = dev_get_drvdata(bus->sdiodev->dev);
	bus_if->state = WLAND_BUS_DOWN;
	/*
	 * De-register interrupt handler
	 */
	wland_sdio_intr_unregister(bus->sdiodev);

	cancel_work_sync(&bus->TxWork);
	if (bus->wland_txwq)
		destroy_workqueue(bus->wland_txwq);
	cancel_work_sync(&bus->RxWork);
	if (bus->wland_rxwq)
		destroy_workqueue(bus->wland_rxwq);

	if (bus->sdiodev->dev)
		wland_bus_detach(bus->sdiodev->dev);

	if (bus->rxbuf) {
		osl_free(bus_if->osh, bus->rxbuf, bus->rxblen);
		bus->rxctl = bus->rxbuf = NULL;
		bus->rxlen = 0;
	}
#ifdef CONFIG_HAS_WAKELOCK
	bus->wakelock_counter = 0;
	bus->wakelock_wd_counter = 0;
	bus->wakelock_rx_timeout_enable = 0;
	bus->wakelock_ctrl_timeout_enable = 0;

	wake_lock_destroy(&bus->wl_wifi);
	wake_lock_destroy(&bus->wl_rxwake);
	wake_lock_destroy(&bus->wl_ctrlwake);
	wake_lock_destroy(&bus->wl_wdwake);
#endif /* CONFIG_HAS_WAKELOCK */

	osl_free(bus_if->osh, bus, sizeof(struct wland_sdio));

	WLAND_DBG(SDIO, TRACE, "Done\n");
}

void *wland_sdio_probe(struct osl_info *osh, struct wland_sdio_dev *sdiodev)
{
	int ret;
	struct wland_sdio *bus;
	struct wland_bus *bus_if;

	WLAND_DBG(SDIO, TRACE, "Enter\n");

	/*
	 * Allocate private bus interface state
	 */
	bus = osl_malloc(osh, sizeof(struct wland_sdio));
	if (!bus)
		goto fail;

	memset(bus, '\0', sizeof(struct wland_sdio));

	/*
	 * pointer each other
	 */
	bus->sdiodev = sdiodev;
	sdiodev->bus = bus;

	bus_if = sdiodev->bus_if;

	/*
	 * attempt to attach to the chip
	 */
#ifdef WLAND_FPGA_SUPPORT
	bus_if->chip = WLAND_VER_91_E;
#else /*WLAND_FPGA_SUPPORT */
	bus_if->chip = (rda_wlan_version() & CHIP_ID_MASK);
#endif /*WLAND_FPGA_SUPPORT */

	WLAND_DBG(SDIO, TRACE, "--------------- Chipid: 0x%x(%s) ---------------\n",
		bus_if->chip, wland_version_string[bus_if->chip]);

	/*
	 * Address of cores for new chips should be added here
	 */
	switch (bus_if->chip) {
	case WLAND_VER_90_D:
	case WLAND_VER_90_E:
	case WLAND_VER_91:
	case WLAND_VER_91_E:
	case WLAND_VER_91_F:
	case WLAND_VER_91_G:
		break;
	default:
		WLAND_ERR("chipid 0x%x is not supported\n", bus_if->chip);
		goto fail;
	}

	wland_pktq_init(&bus->txq, (PRIOMASK + 1), TXQLEN);
	wland_pktq_init(&bus->rxq, (PRIOMASK + 1), TXQLEN);

	/*
	 * setup bus control parameters
	 */
	bus->txbound = WLAND_TXBOUND;
	bus->rxbound = WLAND_RXBOUND;
	bus->txminmax = WLAND_TXMINMAX;
	/*
	 * default sdio bus header length for tx packet
	 */
	bus->tx_hdrlen = FMW_HEADER_LEN;
	bus->clkstate = CLK_SDONLY;
	bus->idletime = WLAND_IDLE_INTERVAL;
	bus->save_ms = WLAND_WD_POLL_MS;
	/*
	 * Set roundup accordingly
	 */
	bus->blocksize = sdiodev->func->cur_blksize;
	/*
	 * Set the poll and interrupt flags(default poll then intr)
	 */
	bus->intr = false;
	bus->poll = true;
	bus->intdis = false;
	bus->polltick = 0;
	bus->flowcontrol = 0;
	bus->activity = false;

	if (bus->poll)
		bus->pollrate = 1;

	/*
	 * Assign bus interface call back,Attach chip version to sdio device
	 */
	bus_if->dev = sdiodev->dev;
	bus_if->ops = &wland_sdio_bus_ops;
	bus_if->state = WLAND_BUS_DOWN;

	/*
	 * disable/enable host interrupt
	 */
	rda_mmc_set_sdio_irq(1, bus->intdis);

	INIT_WORK(&bus->TxWork, wland_sdio_TxWorker);
	bus->wland_txwq = create_singlethread_workqueue("wland_txwq");
	if (!bus->wland_txwq) {
		WLAND_ERR("insufficient memory to create txworkqueue.\n");
		goto fail;
	}
	INIT_WORK(&bus->RxWork, wland_sdio_RxWorker);
	bus->wland_rxwq = create_singlethread_workqueue("wland_rxwq");
	if (!bus->wland_rxwq) {
		WLAND_ERR("insufficient memory to create rxworkqueue.\n");
		goto fail;
	}
	spin_lock_init(&bus->rxctl_lock);
	spin_lock_init(&bus->txqlock);
	spin_lock_init(&bus->rxqlock);

	sema_init(&bus->txclk_sem,1);

	init_waitqueue_head(&bus->ctrl_wait);
	init_waitqueue_head(&bus->dcmd_resp_wait);

	/*
	 * Initialize Wakelock stuff
	 */
	spin_lock_init(&bus->wakelock_spinlock);

	bus->wakelock_counter = 0;
	bus->wakelock_wd_counter = 0;
	bus->wakelock_rx_timeout_enable = 0;
	bus->wakelock_ctrl_timeout_enable = 0;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&bus->wl_wifi, WAKE_LOCK_SUSPEND, "wlan_wake");
	wake_lock_init(&bus->wl_rxwake, WAKE_LOCK_SUSPEND, "wlan_rx_wake");
	wake_lock_init(&bus->wl_ctrlwake, WAKE_LOCK_SUSPEND, "wlan_ctrl_wake");
	wake_lock_init(&bus->wl_wdwake, WAKE_LOCK_SUSPEND, "wlan_wd_wake");
#endif /* CONFIG_HAS_WAKELOCK */

	/*
	 * Set up the watchdog timer
	 */
	init_timer(&bus->timer);
	bus->timer.data = (ulong) bus;
	bus->timer.function = wland_bus_watchdog;

	/*
	 * Initialize watchdog thread
	 */
	init_completion(&bus->watchdog_wait);
	bus->watchdog_tsk =
		kthread_run(wland_sdio_watchdog_thread, bus, "wland_watchdog");
	if (IS_ERR(bus->watchdog_tsk)) {
		WLAND_ERR("watchdog thread failed to create!\n");
		bus->watchdog_tsk = NULL;
	}
	/*
	 * Initialize thread based operation and lock
	 */
	sema_init(&bus->sdsem, 1);

	bus->threads_only = true;

	/*
	 * Initialize DPC thread
	 */
	atomic_set(&bus->tx_dpc_tskcnt, 0);
	atomic_set(&bus->rx_dpc_tskcnt, 0);

	/*
	 * Attach to the common layer, reserve hdr space
	 */
	ret = wland_bus_attach(0, sdiodev->dev);
	if (ret < 0) {
		WLAND_ERR("bus_attach failed\n");
		goto fail;
	}

	/*
	 * Allocate buffers
	 */
	if (bus_if->drvr->maxctl) {
		bus->rxblen =
			roundup((bus_if->drvr->maxctl),
			ALIGNMENT) + WLAND_SDALIGN;
		bus->rxbuf = osl_malloc(osh, bus->rxblen);
		if (!bus->rxbuf) {
			WLAND_ERR("rxbuf malloc failed.\n");
			goto fail;
		}
		memset(bus->rxbuf, '\0', bus->rxblen);
	}

	WLAND_DBG(SDIO, DEBUG, "(maxctl:%d)<====>(rxblen:%d)\n",
		bus_if->drvr->maxctl, bus->rxblen);

	wland_sdio_debugfs_create(bus_if->drvr);

	/*
	 * if firmware path present try to download and bring up bus
	 */
	ret = wland_bus_start(sdiodev->dev);
	if (ret < 0) {
		WLAND_ERR("Bus Start Failed\n");
		goto fail;
	}

	WLAND_DBG(SDIO, TRACE, "SuccessFull Probe Done.\n");

	return bus;

fail:
	wland_sdio_release(bus);

	return NULL;
}
