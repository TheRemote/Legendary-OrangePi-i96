
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
#include <wland_p2p.h>
#include <wland_cfg80211.h>

/* Max valid buffer size that can be sent to the dongle */
#define CDC_MAX_MSG_SIZE	                (ETH_FRAME_LEN+ETH_FCS_LEN)

/* retries to retrieve matching dcmd response */
#define RETRIES                             2

#define wl_getrssi_err_max                  3

static int wl_getrssi_err_counter = 0;

static int wland_wid_hdrpush(struct wland_private *drvr, u16 wid, bool rw,
	u8 * val, u16 val_len)
{
	enum wid_type type = wland_get_wid_type(wid);
	u16 size = wland_get_wid_size(type, val_len);
	u16 wid_msg_len = FMW_HEADER_LEN, wid_pkt_len = 0;
	struct wland_proto *prot = drvr->prot;
	u8 *wid_msg = prot->buf;

	if (drvr->bus_if->state != WLAND_BUS_DATA) {
		WLAND_ERR("bus is down. we have nothing to do.\n");
		return -EINVAL;
	}

	prot->cmd = wid;
	prot->offset = 0;
	prot->rsplen = 0;

	/*
	 * Fill MsgID
	 */
	prot->msg.wid_msg_id = wland_get_seqidx(drvr);

	if (rw) {
		prot->msg.wid_msg_type = WLAND_WID_MSG_WRITE;

		wid_msg[0] = (u8) wid;
		wid_msg[1] = (u8) (wid >> 8);
		if(type == WID_BIN){
			wid_msg[2] = size & 0xff;
			wid_msg[3] = (size & 0xff00) >> 8;
			memcpy(&wid_msg[4], val, size);
			wid_msg_len           += (size + 4);
			wid_msg               += (size + 4);
			wid_msg_len           += 1;
		}else{
			wid_msg[2]             = (u8)(size);
			memcpy(&wid_msg[3], val, size);
			wid_msg_len           += (size + 3);
			wid_msg               += (size + 3);
		}
	} else {
		prot->msg.wid_msg_type = WLAND_WID_MSG_QUERY;

		wid_msg[0] = (u8) wid;
		wid_msg[1] = (u8) (wid >> 8);
		wid_msg_len += 2;
		wid_msg += 2;
	}

	wid_pkt_len = wid_msg_len + WID_HEADER_LEN;

	prot->msg.wid_msg_length = cpu_to_le16(wid_msg_len);
	prot->msg.wid_pkg_length =
		cpu_to_le16((wid_pkt_len & CDC_DCMD_LEN_MASK) |
		(PKT_TYPE_CFG_REQ << CDC_DCMD_LEN_SHIFT));

	WLAND_DBG(DCMD, TRACE,
		"Done(wid:0x%x,type:%d,size:%d,wid_msg_len:%d,wid_pkt_len:%d)\n",
		wid, type, size, wid_msg_len, wid_pkt_len);

	return (wid_msg_len - FMW_HEADER_LEN);
}

static int wland_wid_hdrpull(struct wland_private *drvr, u8 * val, u16 val_len)
{
	int ret = -EBADE;
	u8 flag = 0;
	struct wland_proto *prot = drvr->prot;
	u8 *wid = prot->buf;
	enum wid_type      type        = WID_UNKNOW;

	if ((drvr->bus_if->state != WLAND_BUS_DATA)
		|| (val_len < FMWID_HEADER_LEN)) {
		WLAND_ERR("invalid. we have nothing to do.\n");
		return -EINVAL;
	}

	prot->rsplen = 0;
	prot->offset = 0;
	prot->msg.wid_pkg_length = cpu_to_le16(prot->msg.wid_pkg_length);
	prot->msg.wid_msg_length = cpu_to_le16(prot->msg.wid_msg_length);

	flag = (prot->msg.
		wid_pkg_length & ~CDC_DCMD_LEN_MASK) >> CDC_DCMD_LEN_SHIFT;

	if (PKT_TYPE_CFG_RSP == flag) {
		u16 rsp = (u16) (wid[0] | (wid[1] << 8));
		type        = wland_get_wid_type(rsp);

		if (WLAND_WID_MSG_RESP == prot->msg.wid_msg_type) {
			if (rsp == WID_STATUS) {
				ret = (wid[3] != STATUS_SUCCESS) ? -EINVAL : 0;
			} else {
				ret = 0;
				if(type == WID_BIN){
					prot->rsplen = wid[2] | wid[3] << 8;
				}else{
					prot->rsplen = wid[2];
				}
				prot->offset =
					FMWID_HEADER_LEN - FMW_HEADER_LEN;
			}
		}

		WLAND_DBG(EVENT, TRACE,
			"cfgrsp_len:0x%x,cmd:0x%x,rsp:0x%x,rsplen:0x%x,status:0x%x,prot->rsplen:%d,prot->offset:%d\n",
			val_len, prot->cmd, rsp, wid[2], wid[3], prot->rsplen,
			prot->offset);
	} else if (PKT_TYPE_IND == flag) {
		WLAND_DBG(EVENT, TRACE, "data indication val_len:%d\n",
			val_len);
	} else if (PKT_TYPE_ASYNC == flag) {
		WLAND_DBG(EVENT, TRACE, "sync frame indication val_len:%d\n",
			val_len);
	}

	return ret;
}

static int wland_proto_cdc_msg(struct wland_private *drvr)
{
	struct wland_proto *prot = drvr->prot;
	uint len = le16_to_cpu(prot->msg.wid_pkg_length) & CDC_DCMD_LEN_MASK;

	WLAND_DBG(DCMD, TRACE, "Enter(real_pkt_len:%d)\n", len);

	/*
	 * NOTE : cdc->msg.len holds the desired length of the buffer to be
	 * *        returned. Only up to CDC_MAX_MSG_SIZE of this buffer area
	 * *        is actually sent to the dongle
	 */
	if (len > CDC_MAX_MSG_SIZE)
		len = CDC_MAX_MSG_SIZE;

	/*
	 * Send request
	 */
	return wland_bus_txctl(drvr->bus_if, (u8 *) & prot->msg, len);
}

static int wland_proto_cdc_cmplt(struct wland_private *drvr, u8 id, u16 len)
{
	int ret;
	struct wland_proto *prot = drvr->prot;

	do {
		ret = wland_bus_rxctl(drvr->bus_if, (u8 *) & prot->msg, len);
		if (ret < 0) {
			WLAND_ERR("***response failed***\n");
			break;
		}
	} while (prot->msg.wid_msg_id != id);

	WLAND_DBG(EVENT, TRACE,
		"Done(SendMsgId:%d, ReceivedMsgId:%d, RespLen:%d)\n", id,
		prot->msg.wid_msg_id, ret);
	return ret;
}

int wland_proto_cdc_data(struct wland_private *drvr, u16 wid_msg_len)
{
	struct wland_proto *prot = drvr->prot;
	int err, retries = 0;
	u16 wid_pkg_len = 0;
	u8 wid_msg_id = wland_get_seqidx(drvr);

	if (drvr->bus_if->state != WLAND_BUS_DATA) {
		WLAND_ERR("bus is down. we have nothing to do.\n");
		return -EINVAL;
	}

	prot->msg.wid_msg_type = WLAND_WID_MSG_WRITE;
	prot->msg.wid_msg_id = wid_msg_id;

	wid_pkg_len = wid_msg_len + WID_HEADER_LEN;
	prot->msg.wid_msg_length = cpu_to_le16(wid_msg_len);
	prot->msg.wid_pkg_length =
		cpu_to_le16((wid_pkg_len & CDC_DCMD_LEN_MASK) |
		(PKT_TYPE_CFG_REQ << CDC_DCMD_LEN_SHIFT));

	err = wland_proto_cdc_msg(drvr);
	if (err < 0) {
		WLAND_ERR("set_dcmd failed status: %d\n", err);
		goto done;
	}

retry:
	/*
	 * wait for interrupt and get first fragment
	 */
	err = wland_proto_cdc_cmplt(drvr, wid_msg_id, WLAND_DCMD_MEDLEN);
	if (err < 0) {
		WLAND_ERR("cdc_cmplt fail.\n");
		goto done;
	}

	if ((prot->msg.wid_msg_id < wid_msg_id) && (++retries < RETRIES))
		goto retry;

	if (prot->msg.wid_msg_id != wid_msg_id) {
		WLAND_ERR("unexpected request id %d (expected %d)\n",
			prot->msg.wid_msg_id, wid_msg_id);
		err = -EINVAL;
	}

	if (err > 0)
		err = wland_wid_hdrpull(drvr, (u8 *) & prot->msg, (u16) err);

	WLAND_DBG(DCMD, TRACE, "Write_MsgIdx:%d, Read_MsgIdx:%d.\n", wid_msg_id,
		prot->msg.wid_msg_id);
done:
	WLAND_DBG(DCMD, TRACE, "Done(err:%d)\n", err);
	return err;
}

/* The format of the message is:                                         */

/* +-------------------------------------------------------------------+ */

/* | Message Type | Message ID |  Message Length |Message body         | */

/* +-------------------------------------------------------------------+ */

/* |     1 Byte   |   1 Byte   |     2 Bytes     | Message Length      | */

/* +-------------------------------------------------------------------+ */

int wland_fil_set_cmd_data(struct wland_if *ifp, u16 cmd, void *data, u16 len)
{
	int err, retries = 0;
	u8 wid_msg_id = 0;
	struct wland_private *drvr = ifp->drvr;
	struct wland_proto *prot = drvr->prot;

	if (!(drvr && prot)) {
		WLAND_ERR("some init failed.\n");
		return -EIO;
	}

	if (drvr->bus_if->state != WLAND_BUS_DATA) {
		WLAND_ERR("bus is down. we have nothing to do.\n");
		return -EIO;
	}

	mutex_lock(&drvr->proto_block);

	memset(prot->buf, '\0', sizeof(prot->buf));

	err = wland_wid_hdrpush(drvr, cmd, true, data, len);
	if (err < 0) {
		WLAND_ERR("set_dcmd failed status: %d\n", err);
		goto done;
	}

	wid_msg_id = prot->msg.wid_msg_id;

	err = wland_proto_cdc_msg(drvr);
	if (err < 0) {
		WLAND_ERR("set_dcmd failed status: %d\n", err);
		goto done;
	}

retry:
	/*
	 * wait for interrupt and get first fragment
	 */
	err = wland_proto_cdc_cmplt(drvr, wid_msg_id, WLAND_DCMD_MEDLEN);
	if (err < 0) {
		WLAND_ERR("cdc_cmplt fail.\n");
		goto done;
	}

	if ((prot->msg.wid_msg_id < wid_msg_id) && (++retries < RETRIES)) {
		WLAND_ERR("MisMatch(Write_MsgIdx:%d,Read_MsgIdx:%d)\n",
			wid_msg_id, prot->msg.wid_msg_id);
		goto retry;
	}

	if (prot->msg.wid_msg_id != wid_msg_id) {
		WLAND_ERR("unexpected request id:%d (expected:%d)\n",
			prot->msg.wid_msg_id, wid_msg_id);
		err = -EINVAL;
	}
	WLAND_DBG(DCMD, TRACE, "Write_MsgIdx:%d, Read_MsgIdx:%d.\n", wid_msg_id,
		prot->msg.wid_msg_id);
done:
	mutex_unlock(&drvr->proto_block);

	WLAND_DBG(DCMD, TRACE,
		"Done(cmd:0x%x,len:%d,rsplen:%d,widx:%d,ridx:%d)\n", cmd, len,
		err, wid_msg_id, prot->msg.wid_msg_id);

	return (err >= 0) ? 0 : err;
}

int wland_fil_get_cmd_data(struct wland_if *ifp, u16 cmd, void *data, u16 len)
{
	struct wland_private *drvr = ifp->drvr;
	struct wland_proto *prot = drvr->prot;
	int err = 0, retries = 0;
	u8 wid_msg_id = 0;

	if (!(drvr && prot)) {
		WLAND_ERR("some init failed.\n");
		return -EIO;
	}

	if (drvr->bus_if->state != WLAND_BUS_DATA) {
		WLAND_ERR("bus is down. we have nothing to do.\n");
		return -EIO;
	}

	WLAND_DBG(DCMD, TRACE, "(cmd:0x%x,len:%d),Enter\n", cmd, len);

	mutex_lock(&drvr->proto_block);

	memset(prot->buf, '\0', sizeof(prot->buf));

	err = wland_wid_hdrpush(drvr, cmd, false, data, len);
	if (err < 0) {
		WLAND_ERR("set_dcmd failed status: %d\n", err);
		goto done;
	}

	wid_msg_id = prot->msg.wid_msg_id;

	/*
	 * send msg to chip
	 */
	err = wland_proto_cdc_msg(drvr);
	if (err < 0) {
		WLAND_ERR("query_dcmd failed w/status %d\n", err);
		goto done;
	}

retry:
	/*
	 * wait for interrupt and get first fragment
	 */
	err = wland_proto_cdc_cmplt(drvr, wid_msg_id, WLAND_DCMD_MEDLEN);
	if (err < 0) {
		WLAND_ERR("query_dcmd failed.\n");
		goto done;
	}

	if ((prot->msg.wid_msg_id < wid_msg_id) && (++retries < RETRIES))
		goto retry;

	if (prot->msg.wid_msg_id != wid_msg_id) {
		WLAND_ERR("%s: unexpected request id:%d(expected:%d)\n",
			wland_ifname(drvr, ifp->ifidx), prot->msg.wid_msg_id,
			wid_msg_id);
		err = -EINVAL;
		goto done;
	}

	/*
	 * Copy info buffer
	 */
	if (data) {
		if (err > 0)
			err = wland_wid_hdrpull(drvr, (u8 *) & prot->msg,
				(u16) err);

		if (!err) {
			len = (prot->rsplen > len) ? len : prot->rsplen;
			memcpy(data, &prot->buf[prot->offset], len);
		}
	}
	WLAND_DBG(DCMD, TRACE, "Write_MsgIdx:%d, Read_MsgIdx:%d.\n", wid_msg_id,
		prot->msg.wid_msg_id);
done:
	mutex_unlock(&drvr->proto_block);

	WLAND_DBG(DCMD, TRACE, "(cmd:0x%x,len:%d),Done.\n", cmd, len);

	return (err >= 0) ? 0 : err;
}

s32 wland_fil_iovar_data_set(struct wland_if * ifp, char *name, void *data,
	u16 len)
{
	s32 err = 0;

#if 0
	struct wland_private *drvr = ifp->drvr;

	mutex_lock(&drvr->proto_block);

	WLAND_DBG(DCMD, TRACE, "name=%s, len=%d\n", name, len);

	memcpy(drvr->proto_buf, data, len);
	err = wland_proto_cdc_set_dcmd(drvr, ifp->ifidx, drvr->proto_buf, len);
	mutex_unlock(&drvr->proto_block);
#endif
	return err;
}

s32 wland_fil_iovar_data_get(struct wland_if * ifp, char *name, void *data,
	u16 len)
{
	s32 err = 0;

#if 0
	struct wland_private *drvr = ifp->drvr;

	mutex_lock(&drvr->proto_block);

	memcpy(drvr->proto_buf, data, len);

	err = wland_proto_cdc_query_dcmd(drvr, ifp->ifidx, drvr->proto_buf,
		len);
	if (err == 0)
		memcpy(data, drvr->proto_buf, len);

	WLAND_DBG(DCMD, TRACE, "name=%s, len=%d\n", name, len);

	mutex_unlock(&drvr->proto_block);
#endif
	return err;
}

bool wland_prec_enq(struct device * dev, struct pktq * q, struct sk_buff * pkt,
	int prec)
{
	struct sk_buff *p;
	int eprec = -1;		/* precedence to evict from */
	bool discard_oldest = false;

	//WLAND_DUMP(DCMD, pkt->data, pkt->len, "TxData,prec:%d,TxDatalen:%Zu\n", prec, pkt->len);
	prec = 0;
	/*
	 * Fast case, precedence queue is not full and we are also not exceeding total queue length
	 */
	if (!pktq_pfull(q, prec) && !pktq_full(q)) {
		wland_pktq_penq(q, prec, pkt);
		return true;
	}
	WLAND_ERR("PKT queue is over flow!\n");

	/*
	 * Determine precedence from which to evict packet, if any
	 */
	if (pktq_pfull(q, prec)) {
		eprec = prec;
	} else if (pktq_full(q)) {
		p = wland_pktq_peek_tail(q, &eprec);
		if (eprec > prec)
			return false;
	}

	/*
	 * Evict if needed
	 */
	if (eprec >= 0) {
		/*
		 * refuse newer (incoming) packet
		 */
		if (eprec == prec && !discard_oldest)
			return false;

		/*
		 * Evict packet according to discard policy
		 */
		p = discard_oldest ? wland_pktq_pdeq(q,
			eprec) : wland_pktq_pdeq_tail(q, eprec);

		if (p == NULL)
			WLAND_ERR("failed, oldest %d\n", discard_oldest);

		wland_pkt_buf_free_skb(p);
	}

	/*
	 * Enqueue
	 */
	p = wland_pktq_penq(q, prec, pkt);
	if (p == NULL)
		WLAND_ERR("failed\n");

	WLAND_DBG(DCMD, TRACE, "Done\n");

	return p != NULL;
}

s32 wland_set_scan_timeout(struct wland_if * ifp)
{
	struct wland_private *drvr = ifp->drvr;
	struct wland_proto *prot = drvr->prot;
	u8 *buf = prot->buf;
	s32 ret = 0;
	u16 wid_msg_len = FMW_HEADER_LEN;
	enum wland_firmw_wid wid;

	if (drvr->dev_mode) {
		WLAND_DBG(DCMD, TRACE, "SoftAp: No Need To Set\n");
		return ret;
	}

	mutex_lock(&drvr->proto_block);

	memset(prot->buf, '\0', sizeof(prot->buf));

	/*
	 * wid body
	 */
	wid = WID_SITE_SURVEY_SCAN_TIME;
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = 2;
	buf[3] = SCAN_CHANNEL_TIME;	//50 ms one channel
	buf[4] = 0;
	/*
	 * offset
	 */
	buf += 5;
	wid_msg_len += 5;

	/*
	 * wid body
	 */
	wid = WID_ACTIVE_SCAN_TIME;
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = 2;
	buf[3] = SCAN_ACTIVE_TIME;	//50 ms one channel
	buf[4] = 0;
	/*
	 * offset
	 */
	buf += 5;
	wid_msg_len += 5;

	/*
	 * wid body
	 */
	wid = WID_PASSIVE_SCAN_TIME;
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = 2;
	buf[3] = SCAN_PASSIVE_TIME;	//50 ms one channel
	buf[4] = 0;
	/*
	 * offset
	 */
	buf += 5;
	wid_msg_len += 5;

	ret = wland_proto_cdc_data(drvr, wid_msg_len);

	mutex_unlock(&drvr->proto_block);

	WLAND_DBG(DCMD, TRACE, "Done(ret:%d,wid_pkg_len:%d,wid_msg_len:%d)\n",
		ret, (wid_msg_len + WID_HEADER_LEN), wid_msg_len);

	return ret;
}

s32 wland_start_ap_set(struct wland_if * ifp,
	struct wland_cfg80211_profile * profile, bool is_p2p)
{
	struct wland_private *drvr = ifp->drvr;
	struct wland_proto *prot = drvr->prot;
	u8 *buf = prot->buf;
	s32 ret = 0;
	u16 wid_msg_len = FMW_HEADER_LEN;
	enum wid_type type;
	enum wland_firmw_wid wid;
	u8 size;
	u32 u32Value;

	WLAND_DBG(DCMD, DEBUG, "Enter\n");
	mutex_lock(&drvr->proto_block);

	memset(prot->buf, '\0', sizeof(prot->buf));

	/*
	 * wid body
	 */
	wid = WID_802_11I_MODE;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;		/* size */
#if 0
	if (profile->sec.security == 0x9
		&& (drvr->bus_if->chip == WLAND_VER_91_E
			|| drvr->bus_if->chip == WLAND_VER_91_F)) {
		//huanglei add for wps
		buf[3] = 0x49;
	} else {
		//for wep104 need set imode 0x07 firmware problem
		buf[3] = (profile->sec.security ==
			0x05) ? 0x07 : profile->sec.security;
	}
#else
	buf[3] = profile->sec.security;
	WLAND_DBG(DCMD, TRACE, "profile->sec.security=0x%x\n",
		profile->sec.security);
#endif
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	/*
	 * wid body
	 */
	wid = WID_AUTH_TYPE;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;		/* size */
	buf[3] = profile->sec.firmware_autype;
	WLAND_DBG(DCMD, TRACE, "profile->sec.firmware_autype=%d\n",
		profile->sec.firmware_autype);
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	/*
	 * wid body
	 */
	wid = WID_11I_PTKSA_REPLAY_COUNTER;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;		/* size */
	buf[3] = profile->rsn_cap;
	WLAND_DBG(DCMD, DEBUG, "profile->rsn_cap=0x%02x\n", profile->rsn_cap);
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	/*
	 * wid body
	 */
	wid = WID_BEACON_INTERVAL;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	WLAND_DBG(DCMD, TRACE, "WID_BEACON_INTERVAL size:%d, value:0x%x\n",
		size, profile->beacon);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	buf[3] = (u8) (profile->beacon & 0x00FF);
	buf[4] = (u8) ((profile->beacon & 0xFF00) >> 8);
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	/*
	 * wid body
	 */
	wid = WID_DTIM_PERIOD;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	buf[3] = (u8) (profile->dtim & 0x00FF);
	buf[4] = (u8) ((profile->dtim & 0xFF00) >> 8);
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);
#if 0
	/*
	 * bssid
	 */
	wid = WID_BSSID;
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = 6;
	memcpy(buf + 3, profile->bssid, ETH_ALEN);
	wid_msg_len += 9;
	buf += 9;
#endif
	/*
	 * ssid
	 */
	wid = WID_SSID;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, profile->ssid.SSID_len);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	memcpy(buf + 3, profile->ssid.SSID, size);
	wid_msg_len += (size + 3);
	buf += (size + 3);
#if 1
	/*
	 * Network Event enable
	 */
	wid = WID_NETWORK_EVENT_EN;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	u32Value = 0x8000F800;
	buf[0] = (char) (wid & 0x00FF);
	buf[1] = (char) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	memcpy(buf + 3, &u32Value, size);
	wid_msg_len += (size + 3);
	buf += (size + 3);
#endif
	WLAND_DBG(DCMD, DEBUG, "Start SoftAp(SSID:%s, SSIDlen:%d)\n",
		profile->ssid.SSID, profile->ssid.SSID_len);

	ret = wland_proto_cdc_data(drvr, wid_msg_len);

	mutex_unlock(&drvr->proto_block);
	WLAND_DBG(DCMD, DEBUG, "Done(ret=%d).\n", ret);

	return ret;
}

s32 wland_start_scan_set(struct wland_if * ifp,
	struct wland_ssid_le * scan_ssid, bool enable)
{
	struct wland_private *drvr = ifp->drvr;
	struct wland_proto *prot = drvr->prot;
	u8 *buf = prot->buf;
	s32 ret = 0;
	u16 wid_msg_len = FMW_HEADER_LEN;
	enum wland_firmw_wid wid;
	enum wid_type type;
	u8 size;

	WLAND_DBG(DCMD, TRACE, "Enter %s scan\n", enable ? "start" : "stop");
	mutex_lock(&drvr->proto_block);

	memset(prot->buf, '\0', sizeof(prot->buf));

	/*
	 * wid body
	 */
	wid = WID_SITE_SURVEY;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	buf[3] = SITE_SURVEY_ALL_CH;
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	/*
	 * wid body
	 */
	wid = WID_START_SCAN_REQ;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	buf[3] = USER_SCAN;
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	/*
	 * wid body
	 */
	wid = WID_NETWORK_INFO_EN;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	if (enable)
		buf[3] = 1;	// 0x01:enable scan network info
	else
		buf[3] = 0;	// 0x01:disable scan network info
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	if (drvr->bus_if->chip == WLAND_VER_90_D
		|| drvr->bus_if->chip == WLAND_VER_90_E) {
		s32 i = 0;

		while (i <= scan_ssid->SSID_len) {
			/*
			 * wid body
			 */
			wid = WID_MEMORY_ADDRESS;
			type = wland_get_wid_type(wid);
			size = wland_get_wid_size(type, 1);
			buf[0] = (u8) (wid & 0x00FF);
			buf[1] = (u8) ((wid & 0xFF00) >> 8);
			buf[2] = size;
			buf[3] = 0x80 + i;
			buf[4] = 0x81;
			buf[5] = 0x10;
			buf[6] = 0x00;
			/*
			 * offset
			 */
			buf += (size + 3);
			wid_msg_len += (size + 3);

			/*
			 * wid body
			 */
			wid = WID_MEMORY_ACCESS_32BIT;
			type = wland_get_wid_type(wid);
			size = wland_get_wid_size(type, 1);
			buf[0] = (u8) (wid & 0x00FF);
			buf[1] = (u8) ((wid & 0xFF00) >> 8);
			buf[2] = size;
			if (scan_ssid->SSID_len > 0) {
				buf[3] = scan_ssid->SSID[i + 0];
				buf[4] = scan_ssid->SSID[i + 1];
				buf[5] = scan_ssid->SSID[i + 2];
				buf[6] = scan_ssid->SSID[i + 3];
			} else {
				memset(&buf[3], 0x00, 4);
			}
			/*
			 * offset
			 */
			buf += (size + 3);
			wid_msg_len += (size + 3);
			i += 4;
		}
	} else if (drvr->bus_if->chip == WLAND_VER_91
		|| drvr->bus_if->chip == WLAND_VER_91_E
		|| drvr->bus_if->chip == WLAND_VER_91_F
		|| drvr->bus_if->chip == WLAND_VER_91_G) {
		/*
		 * wid body
		 */
		wid = WID_HIDE_SSID;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, scan_ssid->SSID_len);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		if (scan_ssid->SSID_len > 0) {
			buf[2] = size;
			memcpy(buf + 3, scan_ssid->SSID, size);
		} else {
			size = 1;
			buf[2] = size;
			buf[3] = 0x00;
		}
		/*
		 * offset
		 */
		wid_msg_len += (size + 3);
		buf += (size + 3);
	}

	//WLAND_DUMP(DCMD, prot->buf, wid_msg_len, "Start Scan(SSID: %s,SSID_len:%d)widlen: %Zu\n", scan_ssid->SSID, scan_ssid->SSID_len, wid_msg_len);

	ret = wland_proto_cdc_data(drvr, wid_msg_len);

	mutex_unlock(&drvr->proto_block);
	WLAND_DBG(DCMD, DEBUG, "Done %s scan\n", enable ? "start" : "stop");

	//if the chip version is 91G ,return -ENOMEM while sending wid error to inform android reopen WIFI, otherwise it always return 0.
	if ((ret < 0) && (drvr->bus_if->chip == WLAND_VER_91_G))
		ret = -ENOMEM;
	else
		ret = 0;
	return ret;
}

s32 wland_start_join(struct wland_if * ifp,
	struct wland_cfg80211_profile * profile)
{
	struct wland_private *drvr = ifp->drvr;
	struct wland_proto *prot = drvr->prot;
	u8 *buf = prot->buf;
	s32 ret = 0;
	u16 wid_msg_len = FMW_HEADER_LEN;
	enum wland_firmw_wid wid;
	enum wid_type type;
	u8 size;
	u8 char_val;

	WLAND_DBG(DCMD, TRACE,
		"imode:0x%x, authtype:%d, ssid:%s, SSID_len:%d\n",
		profile->sec.security, profile->sec.firmware_autype,
		profile->ssid.SSID, profile->ssid.SSID_len);
	WLAND_DBG(DCMD, INFO, "Connecting to " MACDBG "\n",
		MAC2STRDBG(profile->bssid));
	mutex_lock(&drvr->proto_block);

	memset(prot->buf, '\0', sizeof(prot->buf));

	if (drvr->bus_if->chip == WLAND_VER_91_F) {
		/*
		 * wid body
		 */
		wid = WID_11G_OPERATING_MODE;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		char_val = 0;
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = char_val;
		/*
		 * offset
		 */
		buf += (size + 3);
		wid_msg_len += (size + 3);
	}

	/*
	 * wid body
	 */
	wid = WID_802_11I_MODE;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	if (profile->sec.security == 0x9
		&& (drvr->bus_if->chip == WLAND_VER_91_E
			|| drvr->bus_if->chip == WLAND_VER_91_F
			|| drvr->bus_if->chip == WLAND_VER_91_G)) {
		//huanglei add for wps
		char_val = 0x49;
	} else {
		//for wep104 need set imode 0x07 firmware problem

		char_val =
			(profile->sec.security ==
			0x05) ? 0x07 : profile->sec.security;
	}
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	buf[3] = char_val;
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	/*
	 * wid body
	 */
	wid = WID_AUTH_TYPE;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	char_val = profile->sec.firmware_autype;
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	buf[3] = char_val;
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	/*
	 * wid body
	 */
	wid = WID_NETWORK_INFO_EN;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	char_val = 0;
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	buf[3] = char_val;
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	/*
	 * wid body
	 */
	wid = WID_CURRENT_TX_RATE;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	char_val = 1;
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	buf[3] = char_val;
	/*
	 * offset
	 */
	buf += (size + 3);
	wid_msg_len += (size + 3);

	if ((drvr->bus_if->chip == WLAND_VER_90_D)
		|| (drvr->bus_if->chip == WLAND_VER_90_E)) {
		/*
		 * set bssid
		 */
		wid = WID_MEMORY_ADDRESS;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0xec;
		buf[4] = 0x81;
		buf[5] = 0x10;
		buf[6] = 0x00;
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ACCESS_32BIT;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = profile->bssid[0];
		buf[4] = profile->bssid[1];
		buf[5] = profile->bssid[2];
		buf[6] = profile->bssid[3];
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ADDRESS;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0xf0;
		buf[4] = 0x81;
		buf[5] = 0x10;
		buf[6] = 0x00;
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ACCESS_32BIT;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = profile->bssid[4];
		buf[4] = profile->bssid[5];
		buf[5] = 0;
		buf[6] = 0;
		wid_msg_len += (size + 3);
		buf += (size + 3);
	}

	if ((drvr->bus_if->chip == WLAND_VER_90_D)
		|| (drvr->bus_if->chip == WLAND_VER_90_E)) {
		/*
		 * huanglei add begin
		 */
		wid = WID_MEMORY_ADDRESS;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0x04;
		buf[4] = 0x01;
		buf[5] = 0x00;
		buf[6] = 0x50;
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ACCESS_16BIT;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0x1;	//(cmax << 4) | (cmin) 00010001
		buf[4] = 0x1;
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ADDRESS;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0x08;
		buf[4] = 0x01;
		buf[5] = 0x00;
		buf[6] = 0x50;
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ACCESS_16BIT;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0x1;	//(cmax << 4) | (cmin) 00010001
		buf[4] = 0x1;
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ADDRESS;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0x0C;
		buf[4] = 0x01;
		buf[5] = 0x00;
		buf[6] = 0x50;
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ACCESS_16BIT;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0x1;	//(cmax << 4) | (cmin) 00010001
		buf[4] = 0x1;
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ADDRESS;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0x10;
		buf[4] = 0x01;
		buf[5] = 0x00;
		buf[6] = 0x50;
		wid_msg_len += (size + 3);
		buf += (size + 3);

		wid = WID_MEMORY_ACCESS_16BIT;
		type = wland_get_wid_type(wid);
		size = wland_get_wid_size(type, 1);
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = size;
		buf[3] = 0x1;	//(cmax << 4) | (cmin) 00010001
		buf[4] = 0x1;
		wid_msg_len += (size + 3);
		buf += (size + 3);
		//huanglei add end
	}

	/*
	 * bssid
	 */
	wid = WID_BSSID;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, sizeof(profile->bssid));
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	memcpy(buf + 3, profile->bssid, size);
	wid_msg_len += (size + 3);
	buf += (size + 3);

	/*
	 * ssid
	 */
	wid = WID_SSID;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, profile->ssid.SSID_len);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	memcpy(buf + 3, profile->ssid.SSID, size);
	wid_msg_len += (size + 3);
	buf += (size + 3);

	wid = WID_START_SCAN_REQ;
	type = wland_get_wid_type(wid);
	size = wland_get_wid_size(type, 1);
	buf[0] = (u8) (wid & 0x00FF);
	buf[1] = (u8) ((wid & 0xFF00) >> 8);
	buf[2] = size;
	buf[3] = 0;
	wid_msg_len += (size + 3);
	buf += (size + 3);

	wid = WID_WEP_KEY_VALUE0;
	type = wland_get_wid_type(wid);

	//write wep key
	if (profile->sec.security == 3 || profile->sec.security == 5) {
		s32 i;
		u8 *key, key_str_len, key_str[WLAN_MAX_KEY_LEN];

		for (i = 0; i < MAX_WSEC_KEY; i++) {
			key = profile->wepkeys[i].data;

			if (profile->wepkeys[i].len == 0)
				continue;

			if (profile->wepkeys[i].len == KEY_LEN_WEP_40) {
				sprintf(key_str, "%02x%02x%02x%02x%02x\n",
					key[0], key[1], key[2], key[3], key[4]);
				key_str_len = 10;
				key_str[key_str_len] = '\0';
			} else if (profile->wepkeys[i].len == KEY_LEN_WEP_104) {
				sprintf(key_str,
					"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
					key[0], key[1], key[2], key[3], key[4],
					key[5], key[6], key[7], key[8], key[9],
					key[10], key[11], key[12]);
				key_str_len = 26;
				key_str[key_str_len] = '\0';
			} else {
				continue;
			}
			size = wland_get_wid_size(type, key_str_len);
			buf[0] = (u8) ((wid + i) & 0x00FF);
			buf[1] = (u8) (((wid + i) & 0xFF00) >> 8);
			buf[2] = size;

			memcpy(buf + 3, key_str, size);

			buf += (size + 3);
			wid_msg_len += (size + 3);
		}
	}
	//WLAND_DUMP(DCMD, prot->buf, wid_msg_len, "Start Join:%Zu\n", wid_msg_len);

	ret = wland_proto_cdc_data(drvr, wid_msg_len);

	mutex_unlock(&drvr->proto_block);

	return ret;
}

s32 wland_disconnect_bss(struct wland_if * ifp,
	struct wland_scb_val_le * scbval)
{
	struct wland_private *drvr = ifp->drvr;
	struct wland_proto *prot = drvr->prot;
	u8 *buf = prot->buf;
	s32 ret = 0;
	u16 wid_msg_len = FMW_HEADER_LEN;
	enum wland_firmw_wid wid;

	WLAND_DBG(DCMD, TRACE, "Enter(" MACDBG ")\n", MAC2STRDBG(scbval->ea));

	if (drvr->bus_if->chip == WLAND_VER_90_D
		|| drvr->bus_if->chip == WLAND_VER_90_E) {
		memset(scbval->ea, '\0', ETH_ALEN);

		ret = wland_fil_set_cmd_data(ifp, WID_SSID, scbval->ea,
			ETH_ALEN);
	} else {

		mutex_lock(&drvr->proto_block);

		memset(prot->buf, '\0', sizeof(prot->buf));

		/*
		 * wid body
		 */
		wid = WID_BSSID;
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = 6;
		wid_msg_len += 9;
		buf += 9;

		wid = WID_SSID;
		buf[0] = (u8) (wid & 0x00FF);
		buf[1] = (u8) ((wid & 0xFF00) >> 8);
		buf[2] = 6;
		wid_msg_len += 9;
		buf += 9;

		ret = wland_proto_cdc_data(drvr, wid_msg_len);

		mutex_unlock(&drvr->proto_block);
	}

	WLAND_DBG(DCMD, TRACE, "Done(disconnect reason:%d).\n", scbval->val);

	return ret;
}

s32 wland_add_wep_key_bss_sta(struct wland_if * ifp, u8 * key, u8 wep_len,
	u8 key_id)
{
	s32 err = 0;
	u8 *buf = kzalloc(wep_len + 2, GFP_KERNEL);

	if (buf == NULL) {
		WLAND_ERR("No Memory.\n");
		return -ENOMEM;
	}

	buf[0] = key_id;
	buf[1] = wep_len;

	memcpy(buf + 2, key, wep_len);

	err = wland_fil_set_cmd_data(ifp, WID_ADD_WEP_KEY, buf, (wep_len + 2));

	kfree(buf);

	WLAND_DBG(DCMD, TRACE, "Done(err:%d)\n", err);

	return err;
}

s32 wland_fil_set_mgmt_ie(struct wland_if * ifp, u8 * vndr_ie_buf,
	u16 vndr_ie_len)
{
	s32 ret = 0;
	struct wland_private *drvr = ifp->drvr;

	vndr_ie_len = vndr_ie_buf[1] + 2;
	WLAND_DBG(DCMD, TRACE,"Enter vndr_ie_len=%d, vndr_ie_buf=%p\n", vndr_ie_len, vndr_ie_buf);
	WLAND_DUMP(TX_CTRL, vndr_ie_buf, vndr_ie_len, "mgmt_le_len:%Zu\n", vndr_ie_len);

	if (drvr->bus_if->chip == WLAND_VER_91_E
		|| drvr->bus_if->chip == WLAND_VER_91_F
		|| drvr->bus_if->chip == WLAND_VER_91_G)
		ret = wland_fil_set_cmd_data(ifp, WID_GEN_ASSOC_IE, vndr_ie_buf,
			vndr_ie_len);

	/*
	 * wapi ies
	 */
	if (vndr_ie_buf[0] == 0x44)
		ret = wland_fil_set_cmd_data(ifp, WID_WAPI_ASSOC_IE,
			vndr_ie_buf, vndr_ie_len);

	WLAND_DBG(DCMD, TRACE, "Enter(vndr_ie_buf:0x%x,ret:%d)\n",
		vndr_ie_buf[0], ret);

	return ret;
}

s32 wland_set_txrate(struct wland_if * ifp, u8 mbps)
{
	s32 ret = 0;
	struct wland_private *drvr = ifp->drvr;
	u32 u32Value = 0;
	u16 u16Value = 0;

	WLAND_DBG(DCMD, TRACE, "Enter\n");
	if (wland_fil_set_cmd_data(ifp, WID_CURRENT_TX_RATE, &mbps,
			sizeof(mbps))) {
		WLAND_ERR("Set WID_CURRENT_TX_RATE value=%d failed \n", mbps);
		goto out;
	}
	if (drvr->bus_if->chip == WLAND_VER_90_D
		|| drvr->bus_if->chip == WLAND_VER_90_E) {
		u32Value = 0x50000104;
		if (wland_fil_set_cmd_data(ifp, WID_MEMORY_ADDRESS, &u32Value,
				sizeof(u32Value))) {
			WLAND_ERR("Set WID_MEMORY_ADDRESS value=%d failed \n",
				u32Value);
			goto out;
		}
		u16Value = 0x0101;
		if (wland_fil_set_cmd_data(ifp, WID_MEMORY_ACCESS_16BIT,
				&u16Value, sizeof(u16Value))) {
			WLAND_ERR
				("Set WID_MEMORY_ACCESS_16BIT value=%d failed \n",
				u16Value);
			goto out;
		}
		u32Value = 0x50000108;
		if (wland_fil_set_cmd_data(ifp, WID_MEMORY_ADDRESS, &u32Value,
				sizeof(u32Value))) {
			WLAND_ERR("Set WID_MEMORY_ADDRESS value=%d failed \n",
				u32Value);
			goto out;
		}
		u16Value = 0x0101;
		if (wland_fil_set_cmd_data(ifp, WID_MEMORY_ACCESS_16BIT,
				&u16Value, sizeof(u16Value))) {
			WLAND_ERR
				("Set WID_MEMORY_ACCESS_16BIT value=%d failed \n",
				u16Value);
			goto out;
		}
		u32Value = 0x5000010C;
		if (wland_fil_set_cmd_data(ifp, WID_MEMORY_ADDRESS, &u32Value,
				sizeof(u32Value))) {
			WLAND_ERR("Set WID_MEMORY_ADDRESS value=%d failed \n",
				u32Value);
			goto out;
		}
		u16Value = 0x0101;
		if (wland_fil_set_cmd_data(ifp, WID_MEMORY_ACCESS_16BIT,
				&u16Value, sizeof(u16Value))) {
			WLAND_ERR
				("Set WID_MEMORY_ACCESS_16BIT value=%d failed \n",
				u16Value);
			goto out;
		}
		u32Value = 0x50000110;
		if (wland_fil_set_cmd_data(ifp, WID_MEMORY_ADDRESS, &u32Value,
				sizeof(u32Value))) {
			WLAND_ERR("Set WID_MEMORY_ADDRESS value=%d failed \n",
				u32Value);
			goto out;
		}
		u16Value = 0x0101;
		if (wland_fil_set_cmd_data(ifp, WID_MEMORY_ACCESS_16BIT,
				&u16Value, sizeof(u16Value))) {
			WLAND_ERR
				("Set WID_MEMORY_ACCESS_16BIT value=%d failed \n",
				u16Value);
			goto out;
		}
	}

	WLAND_DBG(DCMD, TRACE, "Done.\n");

out:
	return ret;
}

int wland_dev_get_rssi(struct net_device *ndev, s16 * pRssi)
{
	struct wland_if *ifp = netdev_priv(ndev);
	u8 rssi = 0;
	int error = 0;
	static int count = 0;
	static u8 preVal = 0;
	int mod = 2;

	if (!pRssi)
		return -EINVAL;
	if ((preVal == 0) || !((count++) % mod)) {
		WLAND_DBG(DCMD, TRACE, "Get RSSI!\n");
		error = wland_fil_get_cmd_data(ifp, WID_RSSI, &rssi,
			sizeof(rssi));

	//error < 0 appear wl_getrssi_err_max times continuously return -ENOMEM to inform WIFI reopen.
		if (error == 0)
			wl_getrssi_err_counter = 0;
		if (error < 0) {
			WLAND_ERR("Get RSSI failed!\n");

			if (ifp->drvr->bus_if->chip == WLAND_VER_91_G) {
				wl_getrssi_err_counter++;

				if (wl_getrssi_err_counter >= wl_getrssi_err_max) {
					wl_getrssi_err_counter = 0;
					return -ENOMEM;
				}
			}
			return error;
		}

		preVal = rssi;
#ifdef WLAND_RSSIOFFSET_SUPPORT
		if(rssi < WLAND_RSSI_MAXVAL_FOR_OFFSET)
			*pRssi = (signed char)(rssi + WLAND_RSSI_OFFSET);
		else
			*pRssi = (signed char)(rssi);
#else
		*pRssi = (signed char)(rssi);
#endif
	} else {
#ifdef WLAND_RSSIOFFSET_SUPPORT
		if(preVal < WLAND_RSSI_MAXVAL_FOR_OFFSET)
			*pRssi = (signed char)(preVal + WLAND_RSSI_OFFSET);
		else
			*pRssi = (signed char)(preVal);
#else
		*pRssi = (signed char)(preVal);
#endif
	}
	WLAND_DBG(DCMD, TRACE, "*pRssi =%d\n", *pRssi);
	return error;
}

#if 0
s32 wland_set_ptk(struct wland_if * ifp, u8 * key, u8 key_len)
{
	s32 ret;
	u8 key_str[32 + ETH_ALEN + 1];
	u8 key_str_len = key_len + ETH_ALEN + 1;

	WLAND_DBG(DCMD, TRACE, "Set PTK: len = %d\n", key_len);

	if (priv->connect_status != MAC_CONNECTED) {
		WLAND_ERR("Adding PTK while not connected\n");
		ret = -EINVAL;
		goto out;
	}

	/*----------------------------------------*/
	/*
	 * STA Addr     | KeyLength |   Key
	 */

	/*----------------------------------------*/
	/*
	 * 6            |         1     |  KeyLength
	 */

	/*----------------------------------------*/

	/*---------------------------------------------------------*/
	/*
	 * key
	 */

	/*---------------------------------------------------------*/
	/*
	 * Temporal Key    | Rx Micheal Key    |   Tx Micheal Key
	 */

	/*---------------------------------------------------------*/
	/*
	 * 16 bytes        |      8 bytes          |       8 bytes
	 */

	/*---------------------------------------------------------*/

	memcpy(key_str, priv->curbssparams.bssid, ETH_ALEN);
	key_str[6] = key_len;
	memcpy(key_str + 7, key, 16);

	/*
	 * swap TX MIC and RX MIC, wlan need RX MIC to be ahead
	 */
	if (key_len > 16) {
		memcpy(key_str + 7 + 16, key + 24, 8);
		memcpy(key_str + 7 + 24, key + 16, 8);
	}

	if (priv->is_wapi)
		ret = wland_fil_set_cmd_data(ifp, WID_ADD_WAPI_PTK, key_str,
			key_str_len);
	else
		ret = wland_fil_set_cmd_data(ifp, WID_ADD_PTK, key_str,
			key_str_len);

	if (ret < 0) {
		goto out;
	}

	WLAND_DBG(DEFAULT, TRACE, "Set PTK Done\n");

out:
	return ret;
}

s32 wland_set_gtk(struct wland_if * ifp, u8 key_id, u8 * key_rsc,
	u8 key_rsc_len, u8 * key, u8 key_len)
{
	s32 ret;
	u8 key_str[32 + ETH_ALEN + 8 + 2];
	u8 key_str_len = key_len + ETH_ALEN + 8 + 2;

	/*---------------------------------------------------------*/
	/*
	 * STA Addr     | KeyRSC | KeyID | KeyLength |   Key
	 */

	/*---------------------------------------------------------*/
	/*
	 * 6            |       8        |       1       |         1     |      KeyLength
	 */

	/*---------------------------------------------------------*/

	/*-------------------------------------*/
	/*
	 * key
	 */

	/*-------------------------------------*/
	/*
	 * Temporal Key    | Rx Micheal Key
	 */

	/*-------------------------------------*/
	/*
	 * 16 bytes        |      8 bytes
	 */

	/*-------------------------------------*/

	WLAND_DBG(DCMD, TRACE, "Set GTK: len = %d\n", key_len);

	if (priv->connect_status != MAC_CONNECTED) {
		WLAND_ERR("Adding GTK while not connected\n");
		ret = -EINVAL;
		goto out;
	}

	memcpy(key_str, priv->curbssparams.bssid, ETH_ALEN);
	memcpy(key_str + 6, key_rsc, key_rsc_len);

	key_str[14] = key_id;
	key_str[15] = key_len;

	memcpy(key_str + 16, key, 16);

	/*
	 * swap TX MIC and RX MIC, wlan need RX MIC to be ahead
	 */
	if (key_len > 16) {
		memcpy(key_str + 16 + 16, key + 24, 8);
		memcpy(key_str + 16 + 24, key + 16, 8);
	}

	if (priv->is_wapi)
		ret = wland_fil_set_cmd_data(ifp, WID_ADD_WAPI_RX_GTK, key_str,
			key_str_len);
	else
		ret = wland_fil_set_cmd_data(ifp, WID_ADD_RX_GTK, key_str,
			key_str_len);

	WLAND_DBG(EVENT, TRACE, "Set GTK Done(ret:%d)\n", ret);

	return ret;
}
#endif

int wland_proto_attach(struct wland_private *drvr)
{
	struct wland_proto *cdc =
		osl_malloc(drvr->bus_if->osh, sizeof(struct wland_proto));

	if (!cdc) {
		WLAND_ERR("no memory for cdc!\n");
		return -ENOMEM;
	}

	memset(cdc, '\0', sizeof(struct wland_proto));

	/*
	 * ensure that the msg buf directly follows the cdc msg struct
	 */
	if ((ulong) (&cdc->msg + 1) != (ulong) cdc->buf) {
		WLAND_ERR("struct wland_proto is not correctly defined\n");
		if (cdc)
			osl_free(drvr->bus_if->osh, cdc,
				sizeof(struct wland_proto));
		return -ENOMEM;
	}

	drvr->prot = cdc;
	drvr->hdrlen += WID_HEADER_LEN;
	drvr->maxctl =
		WLAND_DCMD_MEDLEN + sizeof(struct wland_dcmd) + ROUND_UP_MARGIN;

	WLAND_DBG(DCMD, TRACE, "Done(drvr->hdrlen:%d,drvr->maxctl:%d)\n",
		drvr->hdrlen, drvr->maxctl);

	return 0;
}

void wland_proto_detach(struct wland_private *drvr)
{
	if (drvr) {
		osl_free(drvr->bus_if->osh, drvr->prot,
			sizeof(struct wland_proto));
		drvr->prot = NULL;
	}
	WLAND_DBG(DCMD, TRACE, "Done\n");
}
