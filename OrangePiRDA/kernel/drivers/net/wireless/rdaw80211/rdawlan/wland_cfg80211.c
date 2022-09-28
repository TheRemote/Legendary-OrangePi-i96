
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
#include <net/netlink.h>

#include <wland_defs.h>
#include <wland_utils.h>
#include <wland_fweh.h>
#include <wland_dev.h>
#include <wland_dbg.h>
#include <wland_wid.h>
#include <wland_bus.h>
#include <wland_trap.h>
#include <wland_p2p.h>
#include <wland_cfg80211.h>
#include <wland_android.h>

#define CHAN2G(_channel, _freq, _flags) {	    \
	.band			  = IEEE80211_BAND_2GHZ,    \
	.center_freq	  = (_freq),			    \
	.hw_value		  = (_channel),			    \
	.flags			  = (_flags),			    \
	.max_antenna_gain = 0,				        \
	.max_power		  = 30,				        \
}

#ifdef WLAND_5GRF_SUPPORT
#define CHAN5G(_channel, _flags) {				\
	.band			  = IEEE80211_BAND_5GHZ,	\
	.center_freq	  = 5000 + (5 * (_channel)),\
	.hw_value		  = (_channel),			    \
	.flags			  = (_flags),			    \
	.max_antenna_gain = 0,				        \
	.max_power		  = 30,				        \
}
#endif /* WLAND_5GRF_SUPPORT */

#define RATE_TO_BASE100KBPS(rate)               (((rate) * 10) / 2)

#define RATETAB_ENT(_rateid, _flags){            \
	.bitrate      = RATE_TO_BASE100KBPS(_rateid),\
	.hw_value     = (_rateid),                   \
	.flags        = (_flags),                    \
}

static struct ieee80211_rate __wl_rates[] = {
	RATETAB_ENT(WLAND_RATE_1M, 0),
	RATETAB_ENT(WLAND_RATE_2M, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(WLAND_RATE_5M5, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(WLAND_RATE_11M, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(WLAND_RATE_6M, 0),
	RATETAB_ENT(WLAND_RATE_9M, 0),
	RATETAB_ENT(WLAND_RATE_12M, 0),
	RATETAB_ENT(WLAND_RATE_18M, 0),
	RATETAB_ENT(WLAND_RATE_24M, 0),
	RATETAB_ENT(WLAND_RATE_36M, 0),
	RATETAB_ENT(WLAND_RATE_48M, 0),
	RATETAB_ENT(WLAND_RATE_54M, 0),
};

#define wl_a_rates		(__wl_rates + 4)
#define wl_a_rates_size	8
#define wl_g_rates		(__wl_rates + 0)
#define wl_g_rates_size	12

static struct ieee80211_channel __wl_2ghz_channels[] = {
	CHAN2G(1, 2412, 0),
	CHAN2G(2, 2417, 0),
	CHAN2G(3, 2422, 0),
	CHAN2G(4, 2427, 0),
	CHAN2G(5, 2432, 0),
	CHAN2G(6, 2437, 0),
	CHAN2G(7, 2442, 0),
	CHAN2G(8, 2447, 0),
	CHAN2G(9, 2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0),
};

#ifdef WLAND_5GRF_SUPPORT
static struct ieee80211_channel __wl_5ghz_a_channels[] = {
	CHAN5G(34, 0), CHAN5G(36, 0),
	CHAN5G(38, 0), CHAN5G(40, 0),
	CHAN5G(42, 0), CHAN5G(44, 0),
	CHAN5G(46, 0), CHAN5G(48, 0),
	CHAN5G(52, 0), CHAN5G(56, 0),
	CHAN5G(60, 0), CHAN5G(64, 0),
	CHAN5G(100, 0), CHAN5G(104, 0),
	CHAN5G(108, 0), CHAN5G(112, 0),
	CHAN5G(116, 0), CHAN5G(120, 0),
	CHAN5G(124, 0), CHAN5G(128, 0),
	CHAN5G(132, 0), CHAN5G(136, 0),
	CHAN5G(140, 0), CHAN5G(149, 0),
	CHAN5G(153, 0), CHAN5G(157, 0),
	CHAN5G(161, 0), CHAN5G(165, 0),
	CHAN5G(184, 0), CHAN5G(188, 0),
	CHAN5G(192, 0), CHAN5G(196, 0),
	CHAN5G(200, 0), CHAN5G(204, 0),
	CHAN5G(208, 0), CHAN5G(212, 0),
	CHAN5G(216, 0),
};
#endif /* WLAND_5GRF_SUPPORT */

static struct ieee80211_supported_band __wl_band_2ghz = {
	.band = IEEE80211_BAND_2GHZ,
	.channels = __wl_2ghz_channels,
	.n_channels = ARRAY_SIZE(__wl_2ghz_channels),
	.bitrates = wl_g_rates,
	.n_bitrates = wl_g_rates_size,
};

#ifdef WLAND_5GRF_SUPPORT
static struct ieee80211_supported_band __wl_band_5ghz_a = {
	.band = IEEE80211_BAND_5GHZ,
	.channels = __wl_5ghz_a_channels,
	.n_channels = ARRAY_SIZE(__wl_5ghz_a_channels),
	.bitrates = wl_a_rates,
	.n_bitrates = wl_a_rates_size,
};
#endif /* WLAND_5GRF_SUPPORT */

/* This is to override regulatory domains defined in cfg80211 module (reg.c)
 * By default world regulatory domain defined in reg.c puts the flags
 * NL80211_RRF_PASSIVE_SCAN and NL80211_RRF_NO_IBSS for 5GHz channels (for
 * 36..48 and 149..165). With respect to these flags, wpa_supplicant doesn't
 * start p2p operations on 5GHz channels. All the changes in world regulatory
 * domain are to be done here.
 */
static const struct ieee80211_regdomain wland_regdom = {
	.n_reg_rules = 4,
	.alpha2 = "99",
	.reg_rules = {
		/*
		* IEEE 802.11b/g, channels 1..11
		*/
		REG_RULE(2412 - 10, 2472 + 10, 40, 6, 20, 0),
		/*
		* IEEE 802.11 channel 14 - Only JP enables this and for 802.11b only
		*/
		REG_RULE(2484 - 10, 2484 + 10, 20, 6, 20, 0),
		/*
		* IEEE 802.11a, channel 36..64
		*/
		REG_RULE(5150 - 10, 5350 + 10, 40, 6, 20, 0),
		/*
		* IEEE 802.11a, channel 100..165
		*/
		REG_RULE(5470 - 10, 5850 + 10, 40, 6, 20, 0),
	}
};

static const u32 __wl_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC,
#ifdef WLAND_WAPI_SUPPORT
	WLAN_CIPHER_SUITE_SMS4
#endif /*WLAND_WAPI_SUPPORT */
};

/* Vendor specific ie. id = 221, oui and type defines exact ie */
struct wland_vs_tlv {
	u8 id;
	u8 len;
	u8 oui[3];
	u8 oui_type;
};

struct parsed_vndr_ie_info {
	u8 *ie_ptr;
	u32 ie_len;		/* total length including id & length field */
	struct wland_vs_tlv vndrie;
};

struct parsed_vndr_ies {
	u32 count;
	struct parsed_vndr_ie_info ie_info[VNDR_IE_PARSE_LIMIT];
};

/* Quarter dBm units to mW
 * Table starts at QDBM_OFFSET, so the first entry is mW for qdBm=153
 * Table is offset so the last entry is largest mW value that fits in
 * a u16.
 */

#define QDBM_OFFSET         153	/* Offset for first entry */
#define QDBM_TABLE_LEN      40	/* Table size */

/* Smallest mW value that will round up to the first table entry, QDBM_OFFSET.
 * Value is ( mW(QDBM_OFFSET - 1) + mW(QDBM_OFFSET) ) / 2
 */
#define QDBM_TABLE_LOW_BOUND 42170	/* Low bound */

/* Largest mW value that will round down to the last table entry,
 * QDBM_OFFSET + QDBM_TABLE_LEN-1.
 * Value is ( mW(QDBM_OFFSET + QDBM_TABLE_LEN - 1) +
 * mW(QDBM_OFFSET + QDBM_TABLE_LEN) ) / 2.
 */
#define QDBM_TABLE_HIGH_BOUND 64938	/* High bound */

static const u16 nqdBm_to_mW_map[QDBM_TABLE_LEN] = {

/* qdBm:	+0	+1	+2	+3	+4	+5	+6	+7 */

/* 153: */ 6683, 7079, 7499, 7943, 8414, 8913, 9441, 10000,

/* 161: */ 10593, 11220, 11885, 12589, 13335, 14125, 14962, 15849,

/* 169: */ 16788, 17783, 18836, 19953, 21135, 22387, 23714, 25119,

/* 177: */ 26607, 28184, 29854, 31623, 33497, 35481, 37584, 39811,

/* 185: */ 42170, 44668, 47315, 50119, 53088, 56234, 59566, 63096
};

#ifdef WLAND_RSSIAVG_SUPPORT
static struct wland_rssi_cache_ctrl g_rssi_cache_ctrl;

//static struct wland_rssi_cache_ctrl g_rssi2_cache_ctrl;
#endif /* WLAND_RSSIAVG_SUPPORT */
#ifdef WLAND_BSSCACHE_SUPPORT
static struct wland_bss_cache_ctrl g_bss_cache_ctrl;
#endif /* WLAND_BSSCACHE_SUPPORT */

static u16 wland_qdbm_to_mw(u8 qdbm)
{
	uint factor = 1;
	int idx = qdbm - QDBM_OFFSET;

	if (idx >= QDBM_TABLE_LEN)
		/*
		 * clamp to max u16 mW value
		 */
		return 0xFFFF;

	/*
	 * scale the qdBm index up to the range of the table 0-40
	 * * where an offset of 40 qdBm equals a factor of 10 mW.
	 */
	while (idx < 0) {
		idx += 40;
		factor *= 10;
	}

	/*
	 * return the mW value scaled down to the correct factor of 10,
	 * * adding in factor/2 to get proper rounding.
	 */
	return (nqdBm_to_mW_map[idx] + factor / 2) / factor;
}

static u8 wland_mw_to_qdbm(u16 mw)
{
	u8 qdbm;
	int offset;
	uint mw_uint = mw;
	uint boundary;

	/*
	 * handle boundary case
	 */
	if (mw_uint <= 1)
		return 0;

	offset = QDBM_OFFSET;

	/*
	 * move mw into the range of the table
	 */
	while (mw_uint < QDBM_TABLE_LOW_BOUND) {
		mw_uint *= 10;
		offset -= 40;
	}

	for (qdbm = 0; qdbm < QDBM_TABLE_LEN - 1; qdbm++) {
		boundary =
			nqdBm_to_mW_map[qdbm] + (nqdBm_to_mW_map[qdbm + 1] -
			nqdBm_to_mW_map[qdbm]) / 2;
		if (mw_uint < boundary)
			break;
	}

	qdbm += (u8) offset;

	return qdbm;
}

u16 channel_to_chanspec(struct wland_d11inf * d11inf,
	struct ieee80211_channel * ch)
{
	struct wland_chan ch_inf;

	ch_inf.chnum = ieee80211_frequency_to_channel(ch->center_freq);
	ch_inf.bw = CHAN_BW_20;

	d11inf->encchspec(&ch_inf);

	return ch_inf.chspec;
}

static __always_inline void wland_delay(u32 ms)
{
	if (ms < 1000 / HZ) {
		cond_resched();
		mdelay(ms);
	} else {
		msleep(ms);
	}
}

/* Traverse a string of 1-byte tag/1-byte length/variable-length value
 * triples, returning a pointer to the substring whose first element matches tag
 */
struct wland_tlv *wland_parse_tlvs(void *buf, int buflen, uint key)
{
	struct wland_tlv *elt = (struct wland_tlv *) buf;
	int totlen = buflen;

	/*
	 * find tagged parameter
	 */
	while (totlen >= TLV_HDR_LEN) {
		int len = elt->len;

		/*
		 * validate remaining totlen
		 */
		if ((elt->id == key) && (totlen >= (len + TLV_HDR_LEN)))
			return elt;

		elt = (struct wland_tlv *) ((u8 *) elt + (len + TLV_HDR_LEN));
		totlen -= (len + TLV_HDR_LEN);
	}

	return NULL;
}

/* Is any of the tlvs the expected entry? If not update the tlvs buffer pointer/length. */
static bool wland_tlv_has_ie(u8 * ie, u8 ** tlvs, u32 * tlvs_len, u8 * oui,
	u32 oui_len, u8 type)
{
	/*
	 * If the contents match the OUI and the type
	 */
	if (ie[TLV_LEN_OFF] >= oui_len + 1
		&& !memcmp(&ie[TLV_BODY_OFF], oui, oui_len)
		&& type == ie[TLV_BODY_OFF + oui_len]) {
		return true;
	}

	if (tlvs == NULL)
		return false;
	/*
	 * point to the next ie
	 */
	ie += ie[TLV_LEN_OFF] + TLV_HDR_LEN;
	/*
	 * calculate the length of the rest of the buffer
	 */
	*tlvs_len -= (int) (ie - *tlvs);
	/*
	 * update the pointer to the start of the buffer
	 */
	*tlvs = ie;

	return false;
}

static struct wland_vs_tlv *wland_find_wpaie(u8 * parse, u32 len)
{
	struct wland_tlv *ie;

	while ((ie = wland_parse_tlvs(parse, len, WLAN_EID_VENDOR_SPECIFIC))) {
		if (wland_tlv_has_ie((u8 *) ie, &parse, &len, WPA_OUI,
				TLV_OUI_LEN, WPA_OUI_TYPE))
			return (struct wland_vs_tlv *) ie;
	}
	return NULL;
}

static struct wland_vs_tlv *wland_find_wpsie(u8 * parse, u32 len)
{
	struct wland_tlv *ie;

	while ((ie = wland_parse_tlvs(parse, len, WLAN_EID_VENDOR_SPECIFIC))) {
		if (wland_tlv_has_ie((u8 *) ie, &parse, &len, WPA_OUI,
				TLV_OUI_LEN, WPS_OUI_TYPE))
			return (struct wland_vs_tlv *) ie;
	}
	return NULL;
}

static int send_key_to_chip(struct net_device *ndev, struct wland_wsec_key *key)
{
	int err;

	wland_netdev_wait_pend8021x(ndev);

	err = wland_fil_iovar_data_set(netdev_priv(ndev), "wsec_key", key,
		sizeof(struct wland_wsec_key));
	if (err)
		WLAND_ERR("wsec_key error (%d)\n", err);

	return err;
}

static bool check_vif_up(struct wland_cfg80211_vif *vif)
{
	if (!test_bit(VIF_STATUS_READY, &vif->sme_state)) {
		WLAND_DBG(CFG80211, DEBUG,
			"device is not ready : status (%lu)\n", vif->sme_state);
		return false;
	}

	if (test_bit(VIF_STATUS_TESTING, &vif->sme_state)) {
		WLAND_DBG(CFG80211, DEBUG,
			"device is enter testing : status (%lu)\n",
			vif->sme_state);
		return false;
	}

	return true;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
static struct wireless_dev *cfg80211_add_virtual_iface(struct wiphy *wiphy,
	const char *name,
	enum nl80211_iftype type, u32 * flags, struct vif_params *params)
{
	struct wireless_dev *ndev = NULL;

	WLAND_DBG(CFG80211, TRACE, "enter: %s, type %d\n", name, type);

	if (!name) {
		WLAND_ERR("name is NULL\n");
		return ndev;
	}

	switch (type) {
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MESH_POINT:
		WLAND_ERR("Unsupported interface type\n");
		return ndev;
	case NL80211_IFTYPE_MONITOR:
#ifdef WLAND_P2P_SUPPORT
		WLAND_ERR("No more support monitor interface\n");
		return ndev;
#else /* WLAND_P2P_SUPPORT */
		WLAND_ERR("No more support monitor interface\n");
		return ndev;
#endif /* WLAND_P2P_SUPPORT */
	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_P2P_GO:
	case NL80211_IFTYPE_P2P_DEVICE:
		return ndev;
	case NL80211_IFTYPE_UNSPECIFIED:
	default:
		return ndev;
	}
}

#else
struct net_device *cfg80211_add_virtual_iface(struct wiphy *wiphy,
	char *name,
	enum nl80211_iftype type, u32 * flags, struct vif_params *params)
{

	struct net_device *ndev = NULL;

	WLAND_DBG(CFG80211, TRACE, "enter: %s, type %d\n", name, type);

	if (!name) {
		WLAND_ERR("name is NULL\n");
		return ndev;
	}

	switch (type) {
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MESH_POINT:
		WLAND_ERR("Unsupported interface type\n");
		return ndev;
	case NL80211_IFTYPE_MONITOR:
#ifdef WLAND_MONITOR_SUPPORT
		wland_add_monitor(name, &ndev);
		return ndev;
#else
		return ndev;
#endif /*WLAND_MONITOR_SUPPORT */

	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_P2P_GO:
#ifdef WLAND_P2P_SUPPORT
		//return wland_p2p_add_vif(wiphy, name, type, flags, params);
		return ndev;
#else
		return ndev;
#endif /* WLAND_P2P_SUPPORT */
	case NL80211_IFTYPE_UNSPECIFIED:
	default:
		return ndev;
	}
}

#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */

s32 wland_notify_escan_complete(struct wland_cfg80211_info * cfg,
	struct wland_if * ifp, bool aborted, bool fw_abort)
{
	struct cfg80211_scan_request *scan_request;
	struct wland_ssid_le ssid_le;
	s32 err = 0;

	memset(&ssid_le, 0, sizeof(ssid_le));

	/*
	 * clear scan request, because the FW abort can cause a second call
	 */
	/*
	 * to this functon and might cause a double cfg80211_scan_done
	 */
	scan_request = cfg->scan_request;

	cfg->scan_request = NULL;

	if (timer_pending(&cfg->scan_timeout))
		del_timer_sync(&cfg->scan_timeout);

	if (fw_abort) {
		/*
		 * Do a scan abort to stop the driver's scan engine
		 */
		WLAND_DBG(CFG80211, TRACE, "ABORT scan in firmware\n");
#if 0
		/*
		 * E-Scan (or anyother type) can be aborted by SCAN
		 */
		err = wland_start_scan_set(ifp, &ssid_le, false);
		if (err < 0)
			WLAND_ERR("Scan abort failed\n");
#endif
	}

	/*
	 * e-scan can be initiated by scheduled scan which takes precedence.
	 */
	if (cfg->sched_escan) {
		WLAND_DBG(CFG80211, DEBUG, "scheduled scan completed\n");
		cfg->sched_escan = false;
		if (!aborted)
			cfg80211_sched_scan_results(cfg_to_wiphy(cfg));
	} else if (scan_request) {
		WLAND_DBG(CFG80211, DEBUG, "ESCAN Completed scan: %s\n",
			aborted ? "Aborted" : "Done");
		cfg80211_scan_done(scan_request, aborted);
	}

	WLAND_DBG(CFG80211, TRACE, "Done(aborted:%d,fw_abort:%d)\n", aborted,
		fw_abort);

	return err;
}

static s32 cfg80211_del_virtual_iface(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct wireless_dev *wdev
#else				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct net_device *ndev
#endif				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	)
{
	struct wland_cfg80211_info *cfg = wiphy_priv(wiphy);

#if   LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct net_device *ndev = wdev->netdev;
#else /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct wland_if *ifp = netdev_priv(ndev);
	struct wireless_dev *wdev = &(ifp->vif->wdev);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */

	/*
	 * vif event pending in firmware
	 */
	if (wland_cfg80211_vif_event_armed(cfg))
		return -EBUSY;

	if (ndev) {
		if (test_bit(SCAN_STATUS_BUSY, &cfg->scan_status)
			&& cfg->scan_info.ifp == netdev_priv(ndev))
			wland_notify_escan_complete(cfg, netdev_priv(ndev),
				true, true);
	}

	switch (wdev->iftype) {
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_MESH_POINT:
		return -EOPNOTSUPP;
	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_P2P_GO:
#if    LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	case NL80211_IFTYPE_P2P_DEVICE:
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
#ifdef WLAND_P2P_SUPPORT
		return wland_p2p_del_vif(wiphy, wdev);
#else /* WLAND_P2P_SUPPORT */
		return -EOPNOTSUPP;
#endif /* WLAND_P2P_SUPPORT */
	case NL80211_IFTYPE_UNSPECIFIED:
	default:
		return -EINVAL;
	}
	return -EOPNOTSUPP;
}

static s32 cfg80211_change_virtual_iface(struct wiphy *wiphy,
	struct net_device *ndev,
	enum nl80211_iftype type, u32 * flags, struct vif_params *params)
{
#ifdef WLAND_P2P_SUPPORT
	struct wland_cfg80211_info *cfg = wiphy_priv(wiphy);
#endif /* WLAND_P2P_SUPPORT */
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_vif *vif = ifp->vif;
	s32 infra = 0, ap = 0, err = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter, type:%d\n", type);
	switch (type) {
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MESH_POINT:
		ap = 1;
		WLAND_ERR("type (%d) : currently we do not support this type\n",
			type);
		break;
		return -EOPNOTSUPP;
	case NL80211_IFTYPE_ADHOC:
		vif->mode = WL_MODE_IBSS;
		infra = 0;
		break;
	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_STATION:
		/*
		 * Ignore change for p2p IF. Unclear why supplicant does this
		 */
		if ((vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT) ||
			(vif->wdev.iftype == NL80211_IFTYPE_P2P_GO)) {
			WLAND_DBG(CFG80211, TRACE, "Ignoring cmd for p2p if\n");
			/*
			 * WAR: It is unexpected to get a change of VIF for P2P IF, but it happens.
			 * * The request can not be handled but returning EPERM causes a crash. Returning 0
			 * * without setting ieee80211_ptr->iftype causes trace (WARN_ON) but it works with wpa_supplicant
			 */
			return 0;
		}
		vif->mode = WL_MODE_BSS;
		infra = 1;
		break;

	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_P2P_GO:
		vif->mode = WL_MODE_AP;
		ap = 1;
		break;
	default:
		err = -EINVAL;
		goto done;
	}

	WLAND_DBG(CFG80211, TRACE,
		"Enter,ndev:%p,type:%d,ap:%d,infra:%d,vif->mode:%d\n", ndev,
		type, ap, infra, vif->mode);

	if (ap) {
		if (type == NL80211_IFTYPE_P2P_GO) {
			WLAND_DBG(CFG80211, TRACE, "IF Type = P2P GO\n");
#ifdef WLAND_P2P_SUPPORT
			err = wland_p2p_ifchange(cfg, FIL_P2P_IF_GO);
#else /* WLAND_P2P_SUPPORT */
			err = 0;
#endif /* WLAND_P2P_SUPPORT */
		}
		if (!err) {
			set_bit(VIF_STATUS_AP_CREATING, &vif->sme_state);
			WLAND_DBG(CFG80211, TRACE, "IF Type = AP\n");
		}
	} else {
		err = wland_fil_iovar_data_set(ifp, "set_infra", &infra,
			sizeof(infra));
		if (err < 0) {
			WLAND_ERR("SET_INFRA error (%d)\n", err);
			err = -EAGAIN;
			goto done;
		}
		WLAND_DBG(CFG80211, TRACE, "IF Type = %s\n",
			(vif->mode == WL_MODE_IBSS) ? "Adhoc" : "Infra");
	}
	ndev->ieee80211_ptr->iftype = type;
done:
	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);
	return err;
}

static s32 wland_run_escan(struct wland_cfg80211_info *cfg,
	struct wland_if *ifp, struct cfg80211_scan_request *request, u16 action)
{
	struct wland_ssid_le ssid_le;
	s32 err = 0;

	WLAND_DBG(CFG80211, DEBUG, "E-SCAN START(request->n_ssids:%d), Enter\n",
		request->n_ssids);

	memset(&ssid_le, 0, sizeof(ssid_le));

	if (request->ssids && request->n_ssids) {
		ssid_le.SSID_len = cpu_to_le32(request->ssids[0].ssid_len);

		memcpy(ssid_le.SSID, request->ssids[0].ssid,
			request->ssids[0].ssid_len);
	}

	/*
	 * in our platform just only support one hid ssid
	 */
	err = wland_start_scan_set(ifp, &ssid_le, true);
	if (err < 0)
		WLAND_ERR("SCAN error (%d)\n", err);

	/*
	 * Arm scan timeout timer
	 */
	if(ifp->drvr->bus_if->chip == WLAND_VER_91_F)
		mod_timer(&cfg->scan_timeout,
			jiffies + msecs_to_jiffies(SCAN_TIMER_INTERVAL_MS_91F));
	else
		mod_timer(&cfg->scan_timeout,
			jiffies + msecs_to_jiffies(SCAN_TIMER_INTERVAL_MS));

	WLAND_DBG(CFG80211, TRACE, "E-SCAN START, Done\n");
	return err;
}

static s32 wland_do_escan(struct wland_cfg80211_info *cfg, struct wiphy *wiphy,
	struct wland_if *ifp, struct cfg80211_scan_request *request)
{
	struct escan_info *escan = &cfg->scan_info;

	escan->ifp = ifp;
	escan->wiphy = wiphy;
	escan->escan_state = SCAN_STATE_SCANNING;

	return escan->run(cfg, ifp, request, SCAN_ACTION_START);
}

static s32 cfg80211_scan(struct wiphy *wiphy,
#if    LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
	struct net_device *ndev,
#endif				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct cfg80211_scan_request *request)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);

#if    LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct wland_cfg80211_vif *vif =
		container_of(request->wdev, struct wland_cfg80211_vif, wdev);
#else /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct wland_cfg80211_vif *vif =
		((struct wland_if *) netdev_priv(ndev))->vif;
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct wland_if *ifp = vif->ifp;
	struct cfg80211_ssid *ssids = NULL;
	struct wland_cfg80211_scan_req *sr = &cfg->scan_req_int;
	s32 err = 0, SSID_len;
	bool escan_req;

	if (!check_vif_up(vif))
		return -EIO;

	while (test_bit(SCAN_STATUS_BUSY, &cfg->scan_status)) {
		WLAND_DBG(CFG80211, DEBUG,
			"Scanning already: status (%lu)\n", cfg->scan_status);
		msleep(100);
	}
	if (test_bit(SCAN_STATUS_ABORT, &cfg->scan_status)) {
		WLAND_ERR("Scanning being aborted: status (%lu)\n",
			cfg->scan_status);
		return -EAGAIN;
	}
	if (test_bit(SCAN_STATUS_SUPPRESS, &cfg->scan_status)) {
		WLAND_ERR("Scanning suppressed: status (%lu)\n",
			cfg->scan_status);
		return -EAGAIN;
	}
	if (test_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state)) {
		WLAND_ERR("In Connecting(status %lu), scan return.\n",
			ifp->vif->sme_state);
		return -EAGAIN;
	}
#ifdef WLAND_P2P_SUPPORT
	/*
	 * If scan req comes for p2p0, send it over primary I/F
	 */
	if (vif == cfg->p2p.bss_idx[P2PAPI_BSSCFG_DEVICE].vif)
		vif = cfg->p2p.bss_idx[P2PAPI_BSSCFG_PRIMARY].vif;
#endif /* WLAND_P2P_SUPPORT */
	escan_req = false;

	if (request) {
		/*
		 * scan bss
		 */
		ssids = request->ssids;
		escan_req = true;
	} else {
		/*
		 * scan in ibss
		 */
		/*
		 * we don't do escan in ibss
		 */
	}

	WLAND_DBG(CFG80211, DEBUG,
		"START scan bss:%d,ssids->ssid_len:%d,request->n_ssids:%d, ssids->ssid=%s\n",
		escan_req, ssids->ssid_len, request->n_ssids, ssids->ssid);
#ifdef WLAND_INIT_SCAN_SUPPORT
	if (atomic_read(&cfg->init_scan) == 0) {
		// init scan
		atomic_set(&cfg->init_scan, 1);
	} else if (atomic_read(&cfg->init_scan) == 1) {
		// first scan from wpa_supplicant
		atomic_set(&cfg->init_scan, 2);
		WLAND_DBG(CFG80211, DEBUG, "first time scan report\n");
		cfg->scan_request = request;
		schedule_work(&cfg->scan_report_work);
#ifdef WLAND_P2P_SUPPORT
		if (wland_p2p_scan_finding_common_channel(cfg, NULL))
			return 0;
#endif /* WLAND_P2P_SUPPORT */
		return 0;
	} else {
		cfg->scan_request = request;
	}
#else
		cfg->scan_request = request;
#endif
	set_bit(SCAN_STATUS_BUSY, &cfg->scan_status);

	if (escan_req) {
		cfg->scan_info.run = wland_run_escan;
#ifdef WLAND_P2P_SUPPORT
		err = wland_p2p_scan_prep(wiphy, request, vif);
		if (err < 0) {
			clear_bit(SCAN_STATUS_BUSY, &cfg->scan_status);
			if (timer_pending(&cfg->scan_timeout))
				del_timer_sync(&cfg->scan_timeout);
			cfg->scan_request = NULL;
			return err;
		}
#endif /* WLAND_P2P_SUPPORT */
	} else {
		WLAND_DBG(CFG80211, TRACE, "ssid \"%s\", ssid_len (%d)\n",
			ssids->ssid, ssids->ssid_len);

		memset(&sr->ssid_le, 0, sizeof(sr->ssid_le));
		SSID_len = min_t(u8, sizeof(sr->ssid_le.SSID), ssids->ssid_len);
		sr->ssid_le.SSID_len = cpu_to_le32(0);

		if (SSID_len) {
			memcpy(sr->ssid_le.SSID, ssids->ssid, SSID_len);
			sr->ssid_le.SSID_len = cpu_to_le32(SSID_len);
		} else {
			WLAND_DBG(CFG80211, TRACE, "Broadcast scan\n");
		}
	}

	err = wland_do_escan(cfg, wiphy, vif->ifp, request);
	if (err < 0){
//		err = 0;
		//goto scan_out;
		clear_bit(SCAN_STATUS_BUSY, &cfg->scan_status);
		if (timer_pending(&cfg->scan_timeout))
			del_timer_sync(&cfg->scan_timeout);
		cfg->scan_request = NULL;
		return err;
	}

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return 0;
}

static s32 cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct net_device *ndev = cfg_to_ndev(cfg);
	struct wland_if *ifp = netdev_priv(ndev);
	s32 err = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (!check_vif_up(ifp->vif))
		return -EIO;

	if (changed & WIPHY_PARAM_RTS_THRESHOLD
		&& (cfg->conf->rts_threshold != wiphy->rts_threshold)) {
		cfg->conf->rts_threshold = wiphy->rts_threshold;
		err = wland_fil_set_cmd_data(ifp, WID_RTS_THRESHOLD,
			&cfg->conf->rts_threshold,
			sizeof(cfg->conf->rts_threshold));
		if (err < 0)
			goto done;
	}

	if (changed & WIPHY_PARAM_FRAG_THRESHOLD
		&& (cfg->conf->frag_threshold != wiphy->frag_threshold)) {
		cfg->conf->frag_threshold = wiphy->frag_threshold;
		err = wland_fil_set_cmd_data(ifp, WID_FRAG_THRESHOLD,
			&cfg->conf->frag_threshold,
			sizeof(cfg->conf->frag_threshold));
		if (err < 0)
			goto done;
	}

	if (changed & WIPHY_PARAM_RETRY_LONG
		&& (cfg->conf->retry_long != wiphy->retry_long)) {
		cfg->conf->retry_long = wiphy->retry_long;
		err = wland_fil_set_cmd_data(ifp, WID_LONG_RETRY_LIMIT,
			&cfg->conf->retry_long, sizeof(cfg->conf->retry_long));
		if (err < 0)
			goto done;
	}

	if (changed & WIPHY_PARAM_RETRY_SHORT
		&& (cfg->conf->retry_short != wiphy->retry_short)) {
		cfg->conf->retry_short = wiphy->retry_short;
		err = wland_fil_set_cmd_data(ifp, WID_SHORT_RETRY_LIMIT,
			&cfg->conf->retry_short,
			sizeof(cfg->conf->retry_short));
		if (err < 0)
			goto done;
	}

done:
	WLAND_DBG(CFG80211, TRACE, "Done\n");
	return err;
}

static void wland_link_down(struct wland_cfg80211_vif *vif)
{
	s32 err = 0;
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(vif->wdev.wiphy);
	struct wland_scb_val_le scbval;
	struct wland_cfg80211_profile *profile = ndev_to_prof(cfg->conn_info.ndev);

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (test_bit(VIF_STATUS_CONNECTED, &vif->sme_state)) {
		WLAND_DBG(CFG80211, TRACE,
			"Call WLC_DISASSOC to stop excess roaming\n ");

		memset(&scbval, '\0', sizeof(scbval));
		memcpy(&scbval.ea, &profile->bssid, ETH_ALEN);
		err = wland_disconnect_bss(vif->ifp, &scbval);
		if (!err) {
			WLAND_ERR("DISASSOC from AP success!\n");
			cfg80211_disconnected(vif->wdev.netdev, 0, NULL, 0,
				GFP_KERNEL);
		}

		clear_bit(VIF_STATUS_CONNECTED, &vif->sme_state);
	}
	clear_bit(VIF_STATUS_CONNECTING, &vif->sme_state);
	clear_bit(SCAN_STATUS_SUPPRESS, &cfg->scan_status);

#ifdef WLAND_BTCOEX_SUPPORT
	wland_btcoex_set_mode(vif, BTCOEX_ENABLED, 0);
#endif /* WLAND_BTCOEX_SUPPORT */

	WLAND_DBG(CFG80211, TRACE, "Done\n");
}

static s32 cfg80211_join_ibss(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_ibss_params *params)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	struct wland_join_params join_params;
	size_t join_params_size = 0;
	s32 err = 0, wsec = 0, bcnprd;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (!check_vif_up(ifp->vif))
		return -EIO;

	if (params->ssid) {
		WLAND_DBG(CFG80211, TRACE, "SSID: %s\n", params->ssid);
	} else {
		WLAND_DBG(CFG80211, TRACE, "SSID: NULL, Not supported\n");
		return -EOPNOTSUPP;
	}

	set_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state);

	if (params->bssid)
		WLAND_DBG(CFG80211, TRACE, "BSSID: %pM\n", params->bssid);
	else
		WLAND_DBG(CFG80211, TRACE, "No BSSID specified\n");

#if     LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	if (params->chandef.chan)
		WLAND_DBG(CFG80211, TRACE, "channel: %d\n",
			params->chandef.chan->center_freq);
	else
		WLAND_DBG(CFG80211, TRACE, "no channel specified\n");
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */

	if (params->channel_fixed)
		WLAND_DBG(CFG80211, TRACE, "fixed channel required\n");
	else
		WLAND_DBG(CFG80211, TRACE, "no fixed channel required\n");

	if (params->ie && params->ie_len)
		WLAND_DBG(CFG80211, TRACE, "ie len: %d\n", params->ie_len);
	else
		WLAND_DBG(CFG80211, TRACE, "no ie specified\n");

	if (params->beacon_interval)
		WLAND_DBG(CFG80211, TRACE, "beacon interval: %d\n",
			params->beacon_interval);
	else
		WLAND_DBG(CFG80211, TRACE, "no beacon interval specified\n");

	if (params->basic_rates)
		WLAND_DBG(CFG80211, TRACE, "basic rates: %08X\n",
			params->basic_rates);
	else
		WLAND_DBG(CFG80211, TRACE, "no basic rates specified\n");

	if (params->privacy)
		WLAND_DBG(CFG80211, TRACE, "privacy required\n");
	else
		WLAND_DBG(CFG80211, TRACE, "no privacy required\n");

	/*
	 * Configure Privacy for starter
	 */
	if (params->privacy)
		wsec |= WEP_ENABLED;

	err = wland_fil_iovar_data_set(ifp, "wsec", &wsec, sizeof(wsec));
	if (err) {
		WLAND_ERR("wsec failed (%d)\n", err);
		goto done;
	}

	/*
	 * Configure Beacon Interval for starter
	 */
	if (params->beacon_interval)
		bcnprd = params->beacon_interval;
	else
		bcnprd = 100;

	err = wland_fil_iovar_data_set(ifp, "set_bcnprd", &bcnprd,
		sizeof(bcnprd));
	if (err < 0) {
		WLAND_ERR("WLC_SET_BCNPRD failed (%d)\n", err);
		goto done;
	}

	/*
	 * Configure required join parameter
	 */
	memset(&join_params, 0, sizeof(struct wland_join_params));

	/*
	 * SSID
	 */
	profile->ssid.SSID_len = min_t(u32, params->ssid_len, 32);

	memcpy(profile->ssid.SSID, params->ssid, profile->ssid.SSID_len);
	memcpy(join_params.ssid_le.SSID, params->ssid, profile->ssid.SSID_len);

	join_params.ssid_le.SSID_len = cpu_to_le32(profile->ssid.SSID_len);
	join_params_size = sizeof(join_params.ssid_le);

	/*
	 * BSSID
	 */
	if (params->bssid) {
		memcpy(join_params.params_le.bssid, params->bssid, ETH_ALEN);
		join_params_size =
			sizeof(join_params.ssid_le) +
			(sizeof(struct wland_assoc_params_le) - sizeof(u16));
		memcpy(profile->bssid, params->bssid, ETH_ALEN);
	} else {
		memset(join_params.params_le.bssid, 0xFF, ETH_ALEN);
		memset(profile->bssid, 0, ETH_ALEN);
	}

#if     LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	/*
	 * Channel
	 */
	if (params->chandef.chan) {
		u32 target_channel;

		cfg->channel =
			ieee80211_frequency_to_channel(params->chandef.
			chan->center_freq);

		if (params->channel_fixed) {
			/*
			 * adding chanspec
			 */
			u16 chanspec = channel_to_chanspec(&cfg->d11inf,
				params->chandef.chan);

			join_params.params_le.chanspec_list[0] =
				cpu_to_le16(chanspec);
			join_params.params_le.chanspec_num = cpu_to_le32(1);
			join_params_size += sizeof(join_params.params_le);
		}

		/*
		 * set channel for starter
		 */
		target_channel = cfg->channel;
		err = wland_fil_iovar_data_set(ifp, "set_channel",
			&target_channel, sizeof(target_channel));
		if (err < 0) {
			WLAND_ERR("WLC_SET_CHANNEL failed (%d)\n", err);
			goto done;
		}
	} else {
		cfg->channel = 0;
	}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	cfg->ibss_starter = false;

	err = wland_fil_iovar_data_set(ifp, "set_ssid", &join_params,
		join_params_size);
	if (err < 0) {
		WLAND_ERR("WLC_SET_SSID failed (%d)\n", err);
		goto done;
	}

done:
	if (err)
		clear_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state);
	WLAND_DBG(CFG80211, TRACE, "Done\n");
	return err;
}

static s32 cfg80211_leave_ibss(struct wiphy *wiphy, struct net_device *ndev)
{
	struct wland_if *ifp = netdev_priv(ndev);

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (!check_vif_up(ifp->vif))
		return -EIO;

	wland_link_down(ifp->vif);

	WLAND_DBG(CFG80211, TRACE, "Done\n");

	return 0;
}

static s32 wland_wakeup_connect_worker(struct wland_cfg80211_connect_info
	*conn_info);
static s32 cfg80211_connect(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_connect_params *sme)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	struct wland_cfg80211_connect_info *conn_info = &cfg->conn_info;
	struct ieee80211_channel *chan = sme->channel;
	u16 chanspec;
	s32 err = 0;
	u8 *pcgroup_encrypt_val = NULL, *pccipher_group = NULL, *pcwpa_version =
		NULL;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");
	if (!check_vif_up(ifp->vif))
		return -EIO;

	if (!sme->ssid) {
		WLAND_ERR("Invalid ssid\n");
		return -EOPNOTSUPP;
	}

	while (test_bit(SCAN_STATUS_BUSY, &cfg->scan_status)) {
		WLAND_DBG(CFG80211, DEBUG,
			"Scanning : status (%lu)\n", cfg->scan_status);
		msleep(200);
	}

	if (cfg->in_disconnecting) {
		cfg->in_waiting = true;
		wait_for_completion_timeout(&cfg->disconnecting_wait,
			msecs_to_jiffies(2 * 1000));
	}
	cfg->in_waiting = false;

	memset(profile, '\0', sizeof(struct wland_cfg80211_profile));
	memcpy(profile->bssid, sme->bssid, ETH_ALEN);
	profile->valid_bssid = true;

	WLAND_DBG(CFG80211, TRACE,
		"sme->ssid:%s, sme->bssid:%pM, sme->channel:%d\n", sme->ssid,
		sme->bssid,
		ieee80211_frequency_to_channel(sme->channel->center_freq));
	WLAND_DBG(CFG80211, TRACE,
		"sme->auth_type:%d, sme->ie_len:%d, sme->key_len:%d\n",
		sme->auth_type, sme->ie_len, sme->key_len);
	WLAND_DBG(CFG80211, TRACE, "############# begin connect......\n");
	set_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state);

	if (timer_pending(&conn_info->connect_restorework_timeout)) {
		del_timer_sync(&conn_info->connect_restorework_timeout);
		WLAND_DBG(CFG80211, TRACE, "###### delete connect restorework timer\n");
	}

#ifdef WLAND_P2P_SUPPORT
	if (ifp->vif == cfg->p2p.bss_idx[P2PAPI_BSSCFG_PRIMARY].vif) {
		struct wland_tlv *rsn_ie;
		struct wland_vs_tlv *wpa_ie;
		void *ie;
		u32 ie_len;

		WLAND_DBG(CFG80211, TRACE,
			"A normal (non P2P) connection request setup\n");
		/*
		 * A normal (non P2P) connection request setup.
		 */
		ie = NULL;
		ie_len = 0;
		/*
		 * find the WPA_IE
		 */
		wpa_ie = wland_find_wpaie((u8 *) sme->ie, sme->ie_len);
		if (wpa_ie) {
			ie = wpa_ie;
			ie_len = wpa_ie->len + TLV_HDR_LEN;
		} else {
			/*
			 * find the RSN_IE
			 */
			rsn_ie = wland_parse_tlvs((u8 *) sme->ie, sme->ie_len,
				WLAN_EID_RSN);
			if (rsn_ie) {
				ie = rsn_ie;
				ie_len = rsn_ie->len + TLV_HDR_LEN;
			}
		}
		wland_fil_iovar_data_set(ifp, "wpaie", ie, ie_len);
	}
#endif /* WLAND_P2P_SUPPORT */

	err = wland_fil_set_mgmt_ie(ifp, sme->ie, sme->ie_len);
	if (err < 0) {
		WLAND_ERR("Set Assoc REQ IE Failed\n");
		goto done;
	}

	if (chan) {
		cfg->channel =
			ieee80211_frequency_to_channel(chan->center_freq);
		chanspec = channel_to_chanspec(&cfg->d11inf, chan);
	} else {
		cfg->channel = 0;
		chanspec = 0;
	}

	WLAND_DBG(CFG80211, TRACE,
		"channel:%d, center_req:%d, chanspec:0x%04x\n", cfg->channel,
		chan->center_freq, chanspec);

	/*
	 * 1.set wpa version
	 */
	profile->sec.wpa_versions = sme->crypto.wpa_versions;
	/*
	 * 2.set auth type
	 */
	profile->sec.auth_type = sme->auth_type;

	WLAND_DBG(CFG80211, TRACE,
		"setting wpa_version to 0x%0x, auth_type to 0x%0x.\n",
		profile->sec.wpa_versions, profile->sec.auth_type);

	/*
	 * 3.set cipher
	 */
	profile->sec.cipher_pairwise = sme->crypto.ciphers_pairwise[0];
	profile->sec.cipher_group = sme->crypto.cipher_group;

	/*
	 * In case of privacy, but no security and WPS then simulate setting AES. WPS-2.0 allows no security
	 */
	if (wland_find_wpsie(sme->ie, sme->ie_len) && sme->privacy)
		WLAND_DBG(CFG80211, DEBUG, "privacy 0x%0x\n", sme->privacy);

	WLAND_DBG(CFG80211, TRACE,
		"setting cipher(cipher_pairwise:0x%0x, cipher_group:0x%x)\n",
		profile->sec.cipher_pairwise, profile->sec.cipher_group);

	/*
	 * 4.set key_mgmt
	 */
	profile->sec.wpa_auth = sme->crypto.akm_suites[0];

	WLAND_DBG(CFG80211, TRACE,
		"setting key_mgm(n_akm_suites:0x%0x, wpa_auth:0x%x)\n",
		sme->crypto.n_akm_suites, profile->sec.wpa_auth);

	profile->sec.security = NO_ENCRYPT;
	profile->sec.firmware_autype = OPEN_SYSTEM;

	if ((sme->crypto.n_ciphers_pairwise)
		&& (profile->sec.cipher_pairwise != NO_ENCRYPT)) {
		/*
		 * To determine the u8security value, first we check the group cipher suite then {in case of WPA or WPA2}
		 * we will add to it the pairwise cipher suite(s)
		 */
		if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2) {
			if (profile->sec.cipher_pairwise ==
				WLAN_CIPHER_SUITE_TKIP) {
				profile->sec.security =
					ENCRYPT_ENABLED | WPA2 | TKIP;
				pcgroup_encrypt_val = "WPA2_TKIP";
				pccipher_group = "TKIP";
			} else if (profile->sec.cipher_pairwise == WLAN_CIPHER_SUITE_CCMP)	//TODO: mostafa: here we assume that any other encryption type is AES
			{
				profile->sec.security =
					ENCRYPT_ENABLED | WPA2 | AES;
				pcgroup_encrypt_val = "WPA2_AES";
				pccipher_group = "AES";
			} else if (profile->sec.cipher_pairwise ==
				WLAN_CIPHER_SUITE_AES_CMAC) {
				profile->sec.security =
					ENCRYPT_ENABLED | WPA2 | AES;
				pcgroup_encrypt_val = "WPA2_AES";
				pccipher_group = "AES";
			}
			pcwpa_version = "WPA_VERSION_2";
		} else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_1) {
			if (profile->sec.cipher_pairwise ==
				WLAN_CIPHER_SUITE_TKIP) {
				profile->sec.security =
					ENCRYPT_ENABLED | WPA | TKIP;
				pcgroup_encrypt_val = "WPA_TKIP";
				pccipher_group = "TKIP";
			} else if (profile->sec.cipher_pairwise == WLAN_CIPHER_SUITE_CCMP)	//TODO: mostafa: here we assume that any other encryption type is AES
			{
				profile->sec.security =
					ENCRYPT_ENABLED | WPA | AES;
				pcgroup_encrypt_val = "WPA_AES";
				pccipher_group = "AES";
			} else if (profile->sec.cipher_pairwise ==
				WLAN_CIPHER_SUITE_AES_CMAC) {
				profile->sec.security =
					ENCRYPT_ENABLED | WPA | AES;
				pcgroup_encrypt_val = "WPA_AES";
				pccipher_group = "AES";
			}
			pcwpa_version = "WPA_VERSION_1";
		} else {
			pcwpa_version = "Default";
			if (sme->crypto.cipher_group == WLAN_CIPHER_SUITE_WEP40) {
				profile->sec.security = ENCRYPT_ENABLED | WEP;
				pcgroup_encrypt_val = "WEP40";
				pccipher_group = "WLAN_CIPHER_SUITE_WEP40";
			} else if (sme->crypto.cipher_group ==
				WLAN_CIPHER_SUITE_WEP104) {
				profile->sec.security =
					ENCRYPT_ENABLED | WEP | WEP_EXTENDED;
				pcgroup_encrypt_val = "WEP104";
				pccipher_group = "WLAN_CIPHER_SUITE_WEP104";
			} else {
				WLAND_ERR("Not supported cipher\n");
				err = -ENOTSUPP;
				goto done;
			}

			/*
			 * 5.process wep key
			 */
			profile->wepkey_idx = sme->key_idx;
			profile->wepkeys[sme->key_idx].len = (u32) sme->key_len;
			profile->wepkeys[sme->key_idx].index =
				(u32) sme->key_idx;

			if (profile->wepkeys[sme->key_idx].len >
				sizeof(profile->wepkeys[sme->key_idx].data)) {
				WLAND_ERR("Too long key length (%u)\n",
					profile->wepkeys[sme->key_idx].len);
				err = -EINVAL;
				goto done;
			}

			memcpy(profile->wepkeys[sme->key_idx].data, sme->key,
				profile->wepkeys[sme->key_idx].len);

			profile->wepkeys[sme->key_idx].flags = WL_PRIMARY_KEY;

			switch (profile->sec.cipher_pairwise) {
			case WLAN_CIPHER_SUITE_WEP40:
				profile->wepkeys[sme->key_idx].algo =
					CRYPTO_ALGO_WEP1;
				break;
			case WLAN_CIPHER_SUITE_WEP104:
				profile->wepkeys[sme->key_idx].algo =
					CRYPTO_ALGO_WEP128;
				break;
			default:
				WLAND_ERR("Invalid algorithm (%d)\n",
					sme->crypto.ciphers_pairwise[0]);
				err = -EINVAL;
				goto done;
			}

			/*
			 * Set the new key/index
			 */
			WLAND_DBG(CFG80211, DEBUG,
				"key length (%d),key index (%d),algo (%d),key \"%s\"\n",
				profile->wepkeys[sme->key_idx].len,
				profile->wepkeys[sme->key_idx].index,
				profile->wepkeys[sme->key_idx].algo,
				profile->wepkeys[sme->key_idx].data);

			err = wland_fil_set_cmd_data(ifp, WID_KEY_ID,
				&profile->wepkeys[sme->key_idx].index,
				sizeof(u8));

			if (err < 0) {
				WLAND_ERR("WID_KEY_ID failed (%d)\n", err);
				goto done;
			}

			err = wland_add_wep_key_bss_sta(ifp, (u8 *) sme->key,
				(u8) sme->key_len, (u8) sme->key_idx);
			if (err < 0) {
				WLAND_ERR("add_wep_key failed (%d)\n", err);
				goto done;
			}
		}
	}

	switch (sme->auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
		profile->sec.firmware_autype = OPEN_SYSTEM;
		break;
	case NL80211_AUTHTYPE_SHARED_KEY:
		profile->sec.firmware_autype = SHARED_KEY;
		break;
	default:
		WLAND_DBG(CFG80211, DEBUG, "Automatic Authentation type = %d\n",
			sme->auth_type);
		break;
	}

	WLAND_DBG(CFG80211, TRACE,
		"Group encryption value:%s, Cipher Group:%s, WPA version:%s\n",
		pcgroup_encrypt_val, pccipher_group, pcwpa_version);

	profile->ssid.SSID_len =
		min_t(u32, (u32) sizeof(profile->ssid.SSID),
		(u32) sme->ssid_len);

	memcpy(&profile->ssid.SSID, sme->ssid, profile->ssid.SSID_len);

	if (profile->ssid.SSID_len < IEEE80211_MAX_SSID_LEN)
		profile->ssid.SSID[profile->ssid.SSID_len] = 0;

	WLAND_DBG(CFG80211, TRACE,
		"imode:0x%x, authtype:%d, bssid:%pM, SSID:\"%s\", len (%d)\n",
		profile->sec.security, profile->sec.firmware_autype,
		profile->bssid, profile->ssid.SSID, profile->ssid.SSID_len);

	wland_wakeup_connect_worker(conn_info);
	mod_timer(&conn_info->timer, jiffies + msecs_to_jiffies(CONNECT_TIMER_INTERVAL_MS));
	WLAND_DBG(CFG80211, TRACE, "###### Set connect timer!\n");
	conn_info->timer_on = true;
	conn_info->retry_times = 0;

done:
	if (err < 0) {
		WLAND_ERR
			("Connect error(%d) and clear VIF_STATUS_CONNECTING.\n",
			err);
		clear_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state);
		if (timer_pending(&conn_info->timer)) {
			del_timer_sync(&conn_info->timer);
			WLAND_DBG(CFG80211, TRACE,
				"###### delete conn_info->timer\n");
		}
	}
	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);
	return err;
}

static s32 cfg80211_disconnect(struct wiphy *wiphy, struct net_device *ndev,
	u16 reason_code)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct wland_cfg80211_connect_info *conn_info = cfg_to_conn(cfg);
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	struct wland_scb_val_le scbval;
	s32 err = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter(Reason code:%d)\n", reason_code);

	if (!check_vif_up(ifp->vif)) {
		WLAND_ERR("check_vif_up(ifp->vif) fail and go out.\n");
		return -EIO;
	}

	while (test_bit(SCAN_STATUS_BUSY, &cfg->scan_status)) {
		WLAND_DBG(CFG80211, DEBUG,
			"Scanning: status (%lu)\n", cfg->scan_status);
		msleep(100);
	}

	cancel_work_sync(&conn_info->work);
	if (timer_pending(&conn_info->timer)) {
		del_timer_sync(&conn_info->timer);
		msleep(2000);
		WLAND_DBG(CFG80211, DEBUG, "###### delete conn_info->timer\n");
	}
	clear_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state);

	if (timer_pending(&conn_info->connect_restorework_timeout)) {
		del_timer_sync(&conn_info->connect_restorework_timeout);
		WLAND_DBG(CFG80211, DEBUG, "###### delete connect restoreworktimer\n");
	}

	cfg->in_disconnecting = true;
	memcpy(&scbval.ea, &profile->bssid, ETH_ALEN);

	scbval.val = (u8) (reason_code);

	err = wland_disconnect_bss(ifp, &scbval);

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return err;
}

static s32 cfg80211_set_tx_power(struct wiphy *wiphy,
	struct wireless_dev *wdev,
	enum nl80211_tx_power_setting type, s32 mbm)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct net_device *ndev = cfg_to_ndev(cfg);
	struct wland_if *ifp = netdev_priv(ndev);
	u16 txpwrmw;
	s32 err = 0, disable = 0;

	s32 dbm = MBM_TO_DBM(mbm);
	WLAND_DBG(CFG80211, INFO, "dbm:%d\n", dbm);

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (!check_vif_up(ifp->vif))
		return -EIO;

	WLAND_DBG(CFG80211, INFO, "type:%d\n", type);
	switch (type) {
	case NL80211_TX_POWER_AUTOMATIC:
		break;
	case NL80211_TX_POWER_LIMITED:
	case NL80211_TX_POWER_FIXED:
		if (dbm < 0) {
			WLAND_ERR("TX_POWER_FIXED - dbm is negative\n");
			err = -EINVAL;
			goto done;
		}
		break;
	default:
		err = -EINVAL;
		goto done;
	}
	/*
	 * Make sure radio is off or on as far as software is concerned
	 */
	disable = RADIO_SW_DISABLE << 16;

#if 0
	err = wland_fil_iovar_data_set(ifp, "set_radio", &disable,
		sizeof(disable));
	if (err < 0)
		WLAND_ERR("SET_RADIO error (%d)\n", err);
#endif

	if (dbm > 20)
		dbm = 20;
	if (dbm < 6)
		dbm = 6;

	txpwrmw = (u16) dbm;

	WLAND_DBG(CFG80211, DEBUG, "new tx_power:%d\n", dbm);
	WLAND_DBG(CFG80211, DEBUG, "old tx_power:%d\n", cfg->conf->tx_power);
	cfg->conf->tx_power = dbm;

	WLAND_DBG(CFG80211, DEBUG, "txpwrmw:%d\n", txpwrmw);
	dbm = wland_mw_to_qdbm(txpwrmw);
	WLAND_DBG(CFG80211, DEBUG, "new dbm:%d\n", dbm);

	err = wland_fil_iovar_data_set(ifp, "qtxpower", &dbm, sizeof(dbm));

done:
	WLAND_DBG(CFG80211, INFO, "Done(err:%d, tx_power:%d)\n", err, cfg->conf->tx_power);
	return err;
}

static s32 cfg80211_get_tx_power(struct wiphy *wiphy,
	struct wireless_dev *wdev,
	s32 * dbm)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct wland_if *ifp = netdev_priv(cfg_to_ndev(cfg));
	u8 result;
	s32 err = 0, txpwrdbm = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter.\n");

	if (!check_vif_up(ifp->vif))
		return -EIO;

	err = wland_fil_iovar_data_get(ifp, "qtxpower", &txpwrdbm, sizeof(s32));
	if (err < 0) {
		WLAND_ERR("error (%d)\n", err);
		goto done;
	}

	WLAND_DBG(CFG80211, DEBUG, "txpwrdbm:%d\n", txpwrdbm);
	WLAND_DBG(CFG80211, DEBUG, "cfg tx_power:%d\n", cfg->conf->tx_power);
	result = (u8) (txpwrdbm & ~TXPWR_OVERRIDE);
	WLAND_DBG(CFG80211, DEBUG, "result:%d\n", result);
	*dbm = (s32) wland_qdbm_to_mw(result);
	if (cfg->conf->tx_power > 20)
		cfg->conf->tx_power = 20;
	if (cfg->conf->tx_power == 0)
		cfg->conf->tx_power = 20;
	if (cfg->conf->tx_power < 6)
		cfg->conf->tx_power = 6;
	*dbm = cfg->conf->tx_power;
	WLAND_DBG(CFG80211, DEBUG, "dbm:%d\n", result);
done:
	WLAND_DBG(CFG80211, INFO, "Done(tx_power:%d)\n", cfg->conf->tx_power);
	return err;
}

static s32 cfg80211_config_default_key(struct wiphy *wiphy,
	struct net_device *ndev, u8 key_idx, bool unicast, bool multicast)
{
	u32 index, wsec;
	s32 err = 0;
	struct wland_if *ifp = netdev_priv(ndev);

	WLAND_DBG(CFG80211, TRACE, "key index (%d), Enter\n", key_idx);

	if (!check_vif_up(ifp->vif))
		return -EIO;

	err = wland_fil_iovar_data_get(ifp, "wsec", &wsec, sizeof(u32));
	if (err < 0) {
		WLAND_ERR("WLC_GET_WSEC error (%d)\n", err);
		goto done;
	}

	if (wsec & WEP_ENABLED) {
		/*
		 * Just select a new current key
		 */
		index = key_idx;
		err = wland_fil_iovar_data_set(ifp, "set_key_primary", &index,
			sizeof(u32));
		if (err < 0)
			WLAND_ERR("error (%d)\n", err);
	}
done:
	WLAND_DBG(CFG80211, TRACE, "Done\n");
	return err;
}

static s32 wland_add_ptkey(struct wiphy *wiphy, struct net_device *ndev,
	u8 key_idx, const u8 * mac_addr, struct key_params *params)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_ptkey ptkey;
	s32 err = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter key_idx=%d, params->key_len=%d, params->cipher=0x%x\n", key_idx, params->key_len, params->cipher);

	memset(&ptkey, 0, sizeof(ptkey));
	if (!is_multicast_ether_addr(mac_addr))
		memcpy(ptkey.ea, mac_addr, ETH_ALEN);

	/*
	 * check for key index change
	 */
	if (params->key_len > 0) {
		if (params->key_len > WLAN_MAX_KEY_LEN) {
			WLAND_ERR("Invalid key length (%d)\n", params->key_len);
			return -EINVAL;
		}

		WLAND_DBG(CFG80211, TRACE, "Setting the key index %d\n",
			key_idx);
		ptkey.keyLen = params->key_len;
		memcpy(ptkey.key, params->key, 16);

		if(params->key_len > 16) {
			WLAND_DBG(CFG80211, TRACE, "params->key_len > 16\n");
			memcpy(ptkey.key + 16, params->key + 24, 8);
			memcpy(ptkey.key + 24, params->key + 16, 8);
		}
		err = wland_fil_set_cmd_data(ifp, WID_ADD_PTK, &ptkey,
			MIN(sizeof(ptkey), params->key_len+ETH_ALEN+1));
		if (err < 0) {
			WLAND_ERR("Set WID_ADD_PTK error (%d)\n", err);
			return err;
		}

	}
	WLAND_DBG(CFG80211, TRACE, "Done.\n");
	return err;
}

static s32 wland_add_rx_gtkey(struct wiphy *wiphy, struct net_device *ndev,
	u8 key_idx, struct key_params *params)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	struct wland_rx_gtkey gtkey;
	s32 err = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter key_idx=%d, params->key_len=%d, params->cipher=0x%x\n", key_idx, params->key_len, params->cipher);

	memset(&gtkey, 0, sizeof(gtkey));
	memcpy(gtkey.ea, profile->bssid, ETH_ALEN);
	memcpy(gtkey.keyRSC, params->seq, params->seq_len);
	gtkey.keyIdx = key_idx;
	gtkey.keyLen = params->key_len;
	memcpy(gtkey.key, params->key, 16);
	if (params->key_len > 16) {
		WLAND_DBG(CFG80211, TRACE, "params->key_len > 16\n");
		memcpy(gtkey.key + 16, params->key + 24, 8);
		memcpy(gtkey.key + 24, params->key + 16, 8);
	}
	err = wland_fil_set_cmd_data(ifp, WID_ADD_RX_GTK, &gtkey,
		MIN(sizeof(gtkey), params->key_len+ETH_ALEN+10));
	if (err < 0) {
		WLAND_ERR("Set WID_ADD_RX_GTK error (%d)\n", err);
		return err;
	}

	WLAND_DBG(CFG80211, TRACE, "Done.\n");
	return err;
}

#ifdef STRUCT_REF

/**
 * struct key_params - key information
 *
 * Information about a key
 *
 * @key: key material
 * @key_len: length of key material
 * @cipher: cipher suite selector
 * @seq: sequence counter (IV/PN) for TKIP and CCMP keys, only used
 *	with the get_key() callback, must be in little endian,
 *	length given by @seq_len.
 * @seq_len: length of @seq.
 */
struct key_params {
	u8 *key;
	u8 *seq;
	int key_len;
	int seq_len;
	u32 cipher;
};
#endif /* STRUCT_REF */

static s32 cfg80211_add_key(struct wiphy *wiphy, struct net_device *ndev,
	u8 key_idx, bool pairwise, const u8 * mac_addr,
	struct key_params *params)
{
	struct wland_if *ifp = netdev_priv(ndev);
	s32 err = 0;

#if 0
	struct wland_wsec_key key;
	s32 val, wsec;
	u8 keybuf[8] = { 0 };
#endif
	WLAND_DBG(CFG80211, DEBUG, "key index (%d),Enter\n", key_idx);
	WLAND_DBG(CFG80211, DEBUG,
		"Add Key(%s):\n"
		"Pairwise: %u\n"
		"KeyIndex: %u\n"
		"Cipher  : %02X-%02X-%02X-%02X\n"
		"KeyLen  : %u\n"
		"SeqLen  : %u\n",
		mac_addr ? "ptkey":"gtkey",
		(unsigned int) pairwise, (unsigned int) key_idx,
		(params->cipher >> 24) & 0xFF,
		(params->cipher >> 16) & 0xFF,
		(params->cipher >> 8) & 0xFF,
		params->cipher & 0xFF, params->key_len, params->seq_len);

	if (mac_addr)
		WLAND_DUMP(TX_CTRL, mac_addr, 6, "MAddr:\n");

	if (params->key_len)
		WLAND_DUMP(TX_CTRL, params->key, params->key_len, "Key:\n");

	if (params->seq_len)
		WLAND_DUMP(TX_CTRL, params->seq, params->seq_len, "Seq:\n");


	if (!check_vif_up(ifp->vif))
		return -EIO;

	if (mac_addr)
		return wland_add_ptkey(wiphy, ndev, key_idx, mac_addr, params);
	else
		return wland_add_rx_gtkey(wiphy, ndev, key_idx, params);

#if 0
	memset(&key, 0, sizeof(key));

	key.len = (u32) params->key_len;
	key.index = (u32) key_idx;

	switch (params->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		key.algo = CRYPTO_ALGO_WEP1;
		val = WEP_ENABLED;
		WLAND_DBG(CFG80211, DEBUG, "WLAN_CIPHER_SUITE_WEP40\n");
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		key.algo = CRYPTO_ALGO_WEP128;
		val = WEP_ENABLED;
		WLAND_DBG(CFG80211, DEBUG, "WLAN_CIPHER_SUITE_WEP104\n");
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		if (ifp->vif->mode != WL_MODE_AP) {
			WLAND_DBG(CFG80211, TRACE, "Swapping RX/TX MIC key\n");
			memcpy(keybuf, &key.data[24], sizeof(keybuf));
			memcpy(&key.data[24], &key.data[16], sizeof(keybuf));
			memcpy(&key.data[16], keybuf, sizeof(keybuf));
		}
		key.algo = CRYPTO_ALGO_TKIP;
		val = TKIP_ENABLED;
		WLAND_DBG(CFG80211, DEBUG, "WLAN_CIPHER_SUITE_TKIP\n");
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
		key.algo = CRYPTO_ALGO_AES_CCM;
		val = AES_ENABLED;
		WLAND_DBG(CFG80211, DEBUG, "WLAN_CIPHER_SUITE_AES_CMAC\n");
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		key.algo = CRYPTO_ALGO_AES_CCM;
		val = AES_ENABLED;
		WLAND_DBG(CFG80211, DEBUG, "WLAN_CIPHER_SUITE_CCMP\n");
		break;
#ifdef WLAND_WAPI_SUPPORT
	case WLAN_CIPHER_SUITE_SMS4:
		key.algo = CRYPTO_ALGO_SMS4;
		val = SMS4_ENABLED;
		WLAND_DBG(CFG80211, TRACE,
			"et key to WLAN_CIPHER_SUITE_SMS4\n");
		break;
#endif /*WLAND_WAPI_SUPPORT */
	default:
		WLAND_ERR("Invalid cipher (0x%x)\n", params->cipher);
		err = -EINVAL;
		goto done;
	}

	if (mac_addr) {
		WLAND_DBG(CFG80211, DEBUG, "mac_addr not empty, Done\n");

		/*
		 * Instead of bcast for ea address for default wep keys,driver needs it to be Null
		 */
		if (!is_multicast_ether_addr(mac_addr))
			memcpy((char *) &key.ea, (void *) mac_addr, ETH_ALEN);

		/*
		 * check for key index change
		 */
		if (key.len == 0) {
			/*
			 * key delete
			 */
			err = send_key_to_chip(ndev, &key);
			if (err)
				WLAND_ERR("key delete error (%d)\n", err);
		} else {
			if (key.len > sizeof(key.data)) {
				WLAND_ERR("Invalid key length (%d)\n", key.len);
				return -EINVAL;
			}

			WLAND_DBG(CFG80211, DEBUG, "Setting the key index %d\n",
				key.index);
			memcpy(key.data, params->key, key.len);

			if ((ifp->vif->mode != WL_MODE_AP)
				&& (params->cipher == WLAN_CIPHER_SUITE_TKIP)) {
				WLAND_DBG(CFG80211, DEBUG,
					"Swapping RX/TX MIC key\n");
				memcpy(keybuf, &key.data[24], sizeof(keybuf));
				memcpy(&key.data[24], &key.data[16],
					sizeof(keybuf));
				memcpy(&key.data[16], keybuf, sizeof(keybuf));
			}

			/*
			 * if IW_ENCODE_EXT_RX_SEQ_VALID set
			 */
			if (params->seq && params->seq_len == 6) {
				/*
				 * rx iv
				 */
				u8 *ivptr;

				ivptr = (u8 *) params->seq;
				key.rxiv.hi =
					(ivptr[5] << 24) | (ivptr[4] << 16) |
					(ivptr[3] << 8) | ivptr[2];
				key.rxiv.lo = (ivptr[1] << 8) | ivptr[0];
				key.iv_initialized = true;
			}

			err = send_key_to_chip(ndev, &key);
			if (err)
				WLAND_ERR("wsec_key error (%d)\n", err);
		}
		return err;
	}

	if (key.len > sizeof(key.data)) {
		WLAND_ERR("Too long key length (%u)\n", key.len);
		err = -EINVAL;
		goto done;
	}
	memcpy(key.data, params->key, key.len);

	key.flags = WL_PRIMARY_KEY;

	err = send_key_to_chip(ndev, &key);
	if (err)
		goto done;

	err = wland_fil_iovar_data_get(ifp, "wsec", &wsec, sizeof(wsec));
	if (err < 0) {
		WLAND_ERR("get wsec error (%d)\n", err);
		goto done;
	}
	wsec |= val;
	err = wland_fil_iovar_data_set(ifp, "wsec", &wsec, sizeof(wsec));
	if (err < 0) {
		WLAND_ERR("set wsec error (%d)\n", err);
		goto done;
	}

done:
#endif
	WLAND_DBG(CFG80211, DEBUG, "Done(err:%d)\n", err);
	return err;
}

static s32 cfg80211_del_key(struct wiphy *wiphy, struct net_device *ndev,
	u8 key_idx, bool pairwise, const u8 * mac_addr)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_wsec_key key;
	s32 err = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (!check_vif_up(ifp->vif))
		return -EIO;

	return err;

	if (key_idx >= DOT11_MAX_DEFAULT_KEYS) {
		/*
		 * we ignore this key index in this case
		 */
		WLAND_ERR("invalid key index (%d)\n", key_idx);
		return -EINVAL;
	}

	memset(&key, 0, sizeof(key));

	key.index = (u32) key_idx;
	key.flags = WL_PRIMARY_KEY;
	key.algo = CRYPTO_ALGO_OFF;

	WLAND_DBG(CFG80211, TRACE, "key index (%d)\n", key_idx);

	/*
	 * Set the new key/index
	 */
	err = send_key_to_chip(ndev, &key);

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);
	return err;
}

static s32 cfg80211_get_key(struct wiphy *wiphy, struct net_device *ndev,
	u8 key_idx, bool pairwise, const u8 * mac_addr, void *cookie,
	void (*callback) (void *cookie, struct key_params * params))
{
	struct key_params params;
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	struct wland_cfg80211_security *sec;
	s32 wsec, err = 0;

	WLAND_DBG(CFG80211, TRACE, "key index (%d),Enter\n", key_idx);

	if (!check_vif_up(ifp->vif))
		return -EIO;

	memset(&params, 0, sizeof(params));

	err = wland_fil_iovar_data_get(ifp, "wsec", &wsec, sizeof(wsec));
	if (err) {
		WLAND_ERR("WLC_GET_WSEC error (%d)\n", err);
		/*
		 * Ignore this error, may happen during DISASSOC
		 */
		err = -EAGAIN;
		goto done;
	}

	if (wsec & WEP_ENABLED) {
		sec = &profile->sec;
		if (sec->cipher_pairwise & WLAN_CIPHER_SUITE_WEP40) {
			params.cipher = WLAN_CIPHER_SUITE_WEP40;
			WLAND_DBG(CFG80211, TRACE, "WLAN_CIPHER_SUITE_WEP40\n");
		} else if (sec->cipher_pairwise & WLAN_CIPHER_SUITE_WEP104) {
			params.cipher = WLAN_CIPHER_SUITE_WEP104;
			WLAND_DBG(CFG80211, TRACE,
				"WLAN_CIPHER_SUITE_WEP104\n");
		}
	} else if (wsec & TKIP_ENABLED) {
		params.cipher = WLAN_CIPHER_SUITE_TKIP;
		WLAND_DBG(CFG80211, TRACE, "WLAN_CIPHER_SUITE_TKIP\n");
	} else if (wsec & AES_ENABLED) {
		params.cipher = WLAN_CIPHER_SUITE_AES_CMAC;
		WLAND_DBG(CFG80211, TRACE, "WLAN_CIPHER_SUITE_AES_CMAC\n");
	}
#ifdef WLAND_WAPI_SUPPORT
	else if (wsec & SMS4_ENABLED) {
		params.cipher = WLAN_CIPHER_SUITE_SMS4;
		WLAND_DBG(CFG80211, TRACE, "WLAN_CIPHER_SUITE_SMS4\n");
	}
#endif /* WLAND_WAPI_SUPPORT */
	else {
		WLAND_ERR("Invalid algo (0x%x)\n", wsec);
		err = -EINVAL;
		goto done;
	}
	callback(cookie, &params);

done:
	WLAND_DBG(CFG80211, TRACE, "Done\n");
	return err;
}

static s32 cfg80211_config_default_mgmt_key(struct wiphy *wiphy,
	struct net_device *ndev, u8 key_idx)
{
	WLAND_DBG(CFG80211, TRACE, "Not supported\n");

	return -EOPNOTSUPP;
}

static s32 cfg80211_get_station(struct wiphy *wiphy, struct net_device *ndev,
	u8 * mac, struct station_info *sinfo)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	struct wland_scb_val_le scb_val;
	s32 rate, err = 0;
	u8 *bssid = profile->bssid;

	WLAND_DBG(CFG80211, DEBUG, "Enter, MAC %pM\n", mac);

	if (!check_vif_up(ifp->vif))
		return -EIO;

	if (ifp->vif->mode == WL_MODE_AP) {
		struct wland_sta_info_le *sta_info_le;
		sta_info_le =
			kmalloc(sizeof(struct wland_sta_info_le), GFP_ATOMIC);
		if (!sta_info_le) {
			WLAND_ERR("kmalloc sta_info_le failed\n");
			return -1;
		}

		memcpy(sta_info_le, mac, ETH_ALEN);

		err = wland_fil_iovar_data_get(ifp, "sta_info", sta_info_le,
			sizeof(struct wland_sta_info_le));
		if (err < 0) {
			WLAND_ERR("GET STA INFO failed, %d\n", err);
			kfree(sta_info_le);
			goto done;
		}

		sinfo->filled = STATION_INFO_INACTIVE_TIME;

		sinfo->inactive_time = le32_to_cpu(sta_info_le->idle) * 1000;

		if (le32_to_cpu(sta_info_le->flags) & WLAND_STA_ASSOC) {
			sinfo->filled |= STATION_INFO_CONNECTED_TIME;
			sinfo->connected_time = le32_to_cpu(sta_info_le->in);
		}
		WLAND_DBG(CFG80211, TRACE,
			"STA idle time : %d ms, connected time :%d sec\n",
			sinfo->inactive_time, sinfo->connected_time);
		kfree(sta_info_le);
	} else if (ifp->vif->mode == WL_MODE_BSS) {
		if (!test_bit(VIF_STATUS_CONNECTED, &ifp->vif->sme_state)) {
			WLAND_ERR
				("Network is not connected and go out(sme_state=%x)\n",
				(u32) ifp->vif->sme_state);
			goto done;
		}
		if (memcmp(mac, bssid, ETH_ALEN)) {
			WLAND_ERR
				("Wrong Mac address cfg_mac-%pM wl_bssid-%pM\n",
				mac, bssid);
			goto done;
		}

		/*
		 * Report the current tx rate
		 */
		err = wland_fil_iovar_data_get(ifp, "get_rate", &rate,
			sizeof(rate));
		if (err < 0) {
			WLAND_ERR("Could not get rate (%d)\n", err);
			goto done;
		} else {
			rate = 108;
			sinfo->filled |= STATION_INFO_TX_BITRATE;
			sinfo->txrate.legacy = rate * 5;
			WLAND_DBG(CFG80211, TRACE, "Rate %d Mbps\n", rate / 2);
		}
		if (test_bit(VIF_STATUS_CONNECTED, &ifp->vif->sme_state)) {
			memset(&scb_val, 0, sizeof(scb_val));
#if defined(WLAND_RSSIAVG_SUPPORT)
			err = wl_update_connected_rssi_cache(ndev,
				&g_rssi_cache_ctrl, &scb_val.val);
			if (err) {
				WLAND_ERR("Could not get rssi (%d)\n", err);
				goto done;
			}
			wl_delete_dirty_rssi_cache(&g_rssi_cache_ctrl);
			wl_reset_rssi_cache(&g_rssi_cache_ctrl);
#else
			err = wland_dev_get_rssi(ndev, &scb_val.val);
#endif
			if (err < 0) {
				WLAND_ERR("Could not get rssi (%d)\n", err);
				goto done;
			} else {
				sinfo->filled |= STATION_INFO_SIGNAL;
				sinfo->signal = scb_val.val;
				WLAND_DBG(CFG80211, TRACE, "RSSI %d dBm\n",
					sinfo->signal);
			}
		}
	} else {
		err = -EPERM;
	}
done:
	WLAND_DBG(CFG80211, TRACE, "Done\n");
	return err;
}

static s32 cfg80211_set_power_mgmt(struct wiphy *wiphy, struct net_device *ndev,
	bool enabled, s32 timeout)
{
	s32 pm, err = 0;
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct wland_if *ifp = netdev_priv(ndev);

	WLAND_DBG(CFG80211, TRACE, "Enter(enabled:%d,timeout:%d).\n", enabled,
		timeout);
	/*
	 * Powersave enable/disable request is coming from the cfg80211 even before the interface is up.
	 * In that scenario, driver will be storing the power save preference in cfg struct to apply this
	 * to FW later while initializing the dongle
	 */
	cfg->pwr_save = enabled;

	if (!check_vif_up(ifp->vif)) {
		WLAND_DBG(CFG80211, TRACE,
			"Device is not ready, storing the value in cfg_info struct\n");
		goto done;
	}

	pm = enabled ? MIN_FAST_PS : NO_POWERSAVE;

	/*
	 * Do not enable the power save after assoc if it is a p2p interface
	 */
	if (ifp->vif->wdev.iftype == NL80211_IFTYPE_P2P_CLIENT) {
		WLAND_DBG(CFG80211, TRACE,
			"Do not enable power save for P2P clients\n");
		pm = NO_POWERSAVE;
	}

	WLAND_DBG(CFG80211, TRACE, "power save %s\n",
		(pm ? "enabled" : "disabled"));
#if 0
	err = wland_fil_set_cmd_data(ifp, WID_POWER_MANAGEMENT, &pm,
		sizeof(u8));
	if (err < 0)
		WLAND_ERR("error (%d)\n", err);
#endif
done:
	WLAND_DBG(CFG80211, TRACE, "Done(ret:%d).\n", err);
	return err;
}

static struct wland_bss_info_le *wland_alloc_bss(
	struct wland_cfg80211_info *cfg, struct wland_bss_info_le *old)
{
	struct wland_bss_info_le *bss;

	WLAND_DBG(CFG80211, TRACE, "allocating bss\n");

        bss = kmemdup(old, sizeof(struct wland_bss_info_le), GFP_KERNEL);
	if (!bss)
		return ERR_PTR(-ENOMEM);

	list_add_tail(&bss->list, &cfg->scan_result_list);
	cfg->scan_results.count ++;

	WLAND_DBG(CFG80211, TRACE, "Done\n");
	return bss;
}

static void wland_free_bss(struct wland_cfg80211_info *cfg,
	struct wland_bss_info_le *bss)
{
	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	list_del(&bss->list);
	if(cfg->scan_results.count == 0)
		WLAND_ERR("bss count error\n");
	cfg->scan_results.count --;

	if(bss->ie)
		kfree(bss->ie);
	kfree(bss);

}

static s32 wland_inform_single_bss(struct wland_cfg80211_info *cfg,
	struct wland_bss_info_le *bi)
{
	struct wiphy *wiphy = cfg_to_wiphy(cfg);
	struct ieee80211_channel *notify_channel;
	struct cfg80211_bss *bss;
	struct ieee80211_supported_band *band =
		wiphy->bands[IEEE80211_BAND_2GHZ];
	struct wland_chan ch;
	s32 notify_signal;
	u16 channel;
	u32 freq;
	u8 *notify_ie;

	struct wland_cfg80211_profile *profile = ndev_to_prof(cfg->conn_info.ndev);

	if (bi->length > WLAND_BSS_INFO_MAX) {
		WLAND_ERR("Bss info is larger than buffer. Discarding\n");
		return 0;
	}

	if (!bi->ctl_ch) {
		ch.chspec = bi->chanspec;
		cfg->d11inf.decchspec(&ch);
		bi->ctl_ch = ch.chnum;
	}
	channel = bi->ctl_ch;

	if (channel <= CH_MAX_2G_CHANNEL)
		band = wiphy->bands[IEEE80211_BAND_2GHZ];
#ifdef WLAND_5GRF_SUPPORT
	else
		band = wiphy->bands[IEEE80211_BAND_5GHZ];
#endif /* WLAND_5GRF_SUPPORT */

	freq = ieee80211_channel_to_frequency(channel, band->band);
	notify_channel = ieee80211_get_channel(wiphy, freq);
	notify_ie = bi->ie;
	if ((profile->valid_bssid == true)		//only to BSS of current connection, we use average RSSI in scan_result
		&& !memcmp(profile->bssid, bi->BSSID, ETH_ALEN))
		notify_signal = wl_get_avg_rssi(&g_rssi_cache_ctrl, bi->BSSID) * 100;
	else
		notify_signal = bi->RSSI * 100;

	WLAND_DBG(CFG80211, TRACE,
		"ssid:%s,Bssid:%pM,Channel:%d(%d),Capability:%X,Beacon interval:%d,Signal:%d\n",
		bi->SSID, bi->BSSID, channel, freq, bi->capability,
		bi->beacon_period, notify_signal);

	bss = cfg80211_inform_bss(wiphy,
		notify_channel,
		(const u8 *) bi->BSSID,
		0,
		bi->capability,
		bi->beacon_period,
		notify_ie, (size_t) bi->ie_length, notify_signal, GFP_KERNEL);

	if (!bss)
		return -ENOMEM;
#if    LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	cfg80211_put_bss(wiphy, bss);
#else /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	cfg80211_put_bss(bss);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */

	return 0;
}

static s32 wland_inform_bss(struct wland_cfg80211_info *cfg)
{
	struct wland_bss_info_le *bss = NULL;	/* must be initialized */
	struct wland_bss_info_le *bss_temp = NULL;
	s32 err = 0;

#if defined(WLAND_BSSCACHE_SUPPORT)
	int i;
	struct wland_bss_cache *node;
#endif

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	mutex_lock(&cfg->scan_result_lock);

#if defined(WLAND_BSSCACHE_SUPPORT)
	wl_update_bss_cache(&g_bss_cache_ctrl, &cfg->scan_result_list);
	wl_delete_dirty_bss_cache(&g_bss_cache_ctrl);
	wl_reset_bss_cache(&g_bss_cache_ctrl);
#endif

#if defined(WLAND_RSSIAVG_SUPPORT)
#if defined(WLAND_BSSCACHE_SUPPORT)
	node = g_bss_cache_ctrl.m_cache_head;
	for (; node;) {
		wl_update_rssi_cache(&g_rssi_cache_ctrl, &node->bss);
		node = node->next;
	}
#else
	list_for_each_entry(bss, cfg->scan_result_list, list)
		wl_update_rssi_cache(&g_rssi_cache_ctrl, bss);
#endif

	wl_delete_dirty_rssi_cache(&g_rssi_cache_ctrl);
	wl_reset_rssi_cache(&g_rssi_cache_ctrl);
#endif

#if defined(WLAND_BSSCACHE_SUPPORT)
#if 0
	if (p2p_disconnected > 0) {
		// terence 20130703: Fix for wrong group_capab (timing issue)
		wl_delete_disconnected_bss_cache(&g_bss_cache_ctrl,
			(u8 *) & p2p_disconnected_bssid);
#if defined(WLAND_RSSIAVG_SUPPORT)
		wl_delete_disconnected_rssi_cache(&g_rssi_cache_ctrl,
			(u8 *) & p2p_disconnected_bssid);
#endif
	}
#endif
	WLAND_DBG(CFG80211, TRACE, "Inform cached AP list\n");
	node = g_bss_cache_ctrl.m_cache_head;
	for (i = 0; node; i++) {
		if (node->dirty > 1) {
			// just inform dirty bss
			bss = &node->bss;
			err = wland_inform_single_bss(cfg, bss);
		}
		node = node->next;
	}
	bss = NULL;
#endif

	WLAND_DBG(CFG80211, TRACE, "scanned AP count (%d)\n", cfg->scan_results.count);
	list_for_each_entry_safe(bss, bss_temp, &cfg->scan_result_list, list) {
		err = wland_inform_single_bss(cfg, bss);
		if (err < 0) {
			WLAND_ERR
				("wland_inform_single_bss ERROR!  (err=%d)\n",
				err);
			break;
		}
		if(time_before(bss->time,
			jiffies - msecs_to_jiffies(SCAN_TIMER2_INTERVAL_MS))) {
			WLAND_DBG(CFG80211, DEBUG,
				"del bss:%s.(timeout for %d msecond)\n",
				bss->SSID, SCAN_TIMER2_INTERVAL_MS);
			wland_free_bss(cfg, bss);
			continue;
		}
	}
	WLAND_DBG(CFG80211, TRACE, "Report scanned AP list Done.\n");

#if 0
	if (p2p_disconnected > 0) {
		// terence 20130703: Fix for wrong group_capab (timing issue)
		p2p_disconnected++;
		if (p2p_disconnected >= REPEATED_SCAN_RESULT_CNT + 1)
			p2p_disconnected = 0;
	}
#endif
	mutex_unlock(&cfg->scan_result_lock);

	return err;
}

static s32 wland_inform_ibss(struct wland_cfg80211_info *cfg,
	struct net_device *ndev, const u8 * bssid)
{
	struct wiphy *wiphy = cfg_to_wiphy(cfg);
	struct ieee80211_channel *notify_channel;
	struct wland_bss_info_le *bi = NULL;
	struct ieee80211_supported_band *band;
	struct cfg80211_bss *bss;
	struct wland_chan ch;
	u8 *buf = NULL;
	s32 err = 0;
	u32 freq;
	u8 *notify_ie;
	s32 notify_signal;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	buf = kzalloc(WLAND_BSS_INFO_MAX, GFP_KERNEL);
	if (buf == NULL) {
		err = -ENOMEM;
		goto out;
	}

	*(__le32 *) buf = cpu_to_le32(WLAND_BSS_INFO_MAX);

	err = wland_fil_iovar_data_get(netdev_priv(ndev), "get_bss_info", buf,
		WLAND_BSS_INFO_MAX);
	if (err < 0) {
		WLAND_ERR("WLC_GET_BSS_INFO failed: %d\n", err);
		goto CleanUp;
	}

	bi = (struct wland_bss_info_le *) (buf + 4);

	ch.chspec = le16_to_cpu(bi->chanspec);
	cfg->d11inf.decchspec(&ch);

	band = wiphy->bands[IEEE80211_BAND_2GHZ];
#ifdef WLAND_5GRF_SUPPORT
	if (ch.band == CHAN_BAND_5G)
		band = wiphy->bands[IEEE80211_BAND_5GHZ];
#endif /* WLAND_5GRF_SUPPORT */

	freq = ieee80211_channel_to_frequency(ch.chnum, band->band);
	notify_channel = ieee80211_get_channel(wiphy, freq);
	notify_ie = bi->ie;
	notify_signal = bi->RSSI * 100;

	WLAND_DBG(CFG80211, TRACE, "channel: %d(%d)\n", ch.chnum, freq);
	WLAND_DBG(CFG80211, TRACE, "capability: %X\n", bi->capability);
	WLAND_DBG(CFG80211, TRACE, "beacon interval: %d\n", bi->beacon_period);
	WLAND_DBG(CFG80211, TRACE, "signal: %d\n", notify_signal);

	bss = cfg80211_inform_bss(wiphy,
		notify_channel,
		bssid,
		0,
		bi->capability,
		bi->beacon_period,
		notify_ie, (size_t) bi->ie_length, notify_signal, GFP_KERNEL);

	if (!bss) {
		err = -ENOMEM;
		goto CleanUp;
	}
#if    LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	cfg80211_put_bss(wiphy, bss);
#else /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	cfg80211_put_bss(bss);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */

CleanUp:
	kfree(buf);
out:
	WLAND_DBG(CFG80211, TRACE, "Done\n");

	return err;
}

void wland_abort_scanning(struct wland_cfg80211_info *cfg)
{
	struct escan_info *escan = &cfg->scan_info;

	WLAND_DBG(CFG80211, DEBUG, "Enter\n");

	if (timer_pending(&cfg->scan_timeout)) {
		WLAND_DBG(CFG80211, TRACE, "del timer scan_timerout\n");
		del_timer_sync(&cfg->scan_timeout);
	}

	cancel_work_sync(&cfg->scan_report_work);

	set_bit(SCAN_STATUS_ABORT, &cfg->scan_status);

	if (cfg->scan_request) {
		escan->escan_state = SCAN_STATE_IDLE;
		wland_notify_escan_complete(cfg, escan->ifp, true, true);
	}

	clear_bit(SCAN_STATUS_BUSY, &cfg->scan_status);
	clear_bit(SCAN_STATUS_ABORT, &cfg->scan_status);

	WLAND_DBG(CFG80211, DEBUG, "Done\n");
}

static void cfg80211_scan_report_worker(struct work_struct *work)
{
	struct wland_cfg80211_info *cfg =
		container_of(work, struct wland_cfg80211_info,
		scan_report_work);

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	wland_notify_escan_complete(cfg, cfg->scan_info.ifp, false, true);
}

static void wland_scan_timeout(ulong data)
{
	struct wland_cfg80211_info *cfg = (struct wland_cfg80211_info *) data;
	struct wland_event_msg event;

	memset(&event, '\0', sizeof(struct wland_event_msg));

	event.event_code = WLAND_E_ESCAN_RESULT;
	event.status = STATUS_TIMEOUT;

	firmweh_push_event(cfg->pub, &event, NULL);
	if (cfg->scan_request) {
		WLAND_DBG(CFG80211, DEBUG, "timer expired, schedule_work scan_report_work\n");
		schedule_work(&cfg->scan_report_work);
#ifdef WLAND_P2P_SUPPORT
		if (wland_p2p_scan_finding_common_channel(cfg, NULL))
			return 0;
#endif /* WLAND_P2P_SUPPORT */
	}
}

static s32 wland_wakeup_connect_worker(struct wland_cfg80211_connect_info
	*conn_info)
{
	WLAND_DBG(CFG80211, DEBUG, "Enter\n");
	WLAND_DBG(CFG80211, DEBUG, "wake up connect\n");
	queue_work(conn_info->connect_wq, &conn_info->work);
	WLAND_DBG(CFG80211, DEBUG, "Done.\n");
	return 0;
}

static void wland_connect_timer(unsigned long data)
{
	struct wland_cfg80211_connect_info *conn_info =
		(struct wland_cfg80211_connect_info *) data;
	struct wland_cfg80211_info *cfg =
		container_of(conn_info, struct wland_cfg80211_info, conn_info);
	struct net_device *ndev = cfg_to_ndev(cfg);
	struct wland_if *ifp = netdev_priv(ndev);

	WLAND_DBG(CFG80211, DEBUG, "Enter\n");
	if (test_bit(VIF_STATUS_CONNECTED, &ifp->vif->sme_state)) {
		WLAND_DBG(CFG80211, ERROR, "VIF_STATUS_CONNECTED\n");
		return ;
	}

	if (conn_info) {
		conn_info->timer_on = false;
		conn_info->retry_times++;

		if (conn_info->retry_times < CONNECT_RETRY_TIMES_MAX) {
			WLAND_DBG(CFG80211, TRACE,
				"conn_info->timer expired and mod another timer! retry_times=%d\n",
				conn_info->retry_times);
			wland_wakeup_connect_worker(conn_info);
			mod_timer(&conn_info->timer,
				jiffies + msecs_to_jiffies(CONNECT_TIMER_INTERVAL_MS));
			conn_info->timer_on = true;
		} else {
			WLAND_DBG(CFG80211, INFO,
				"conn_info->timer expired reached max times %d!! leave connecting\n",
				conn_info->retry_times);
			test_and_clear_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state);
		}
	}
	WLAND_DBG(CFG80211, TRACE, "Done.\n");
}

static void wland_cfg80211_connect_worker(struct work_struct *work)
{
	struct wland_cfg80211_connect_info *conn_info =
		container_of(work, struct wland_cfg80211_connect_info, work);
	//struct wland_cfg80211_info *cfg = (struct wland_cfg80211_info *)conn_info->data;
	int err = -1;

	struct wland_if *ifp = netdev_priv(conn_info->ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(conn_info->ndev);

	WLAND_DBG(CFG80211, DEBUG, "Enter conn_ctrl->timer_on=%d\n",
		conn_info->timer_on);

	WLAND_DBG(CFG80211, DEBUG, "Really do start join AP!\n");

	//try to connect WEP_AP with shared_key or open_system
	if (profile->sec.security & (WEP | WEP_EXTENDED)) {
		if (profile->sec.firmware_autype == OPEN_SYSTEM)
			profile->sec.firmware_autype = SHARED_KEY;
		else
			profile->sec.firmware_autype = OPEN_SYSTEM;
	}

	err = wland_start_join(ifp, profile);
	if (err)
		WLAND_ERR("wland_start_join failed (%d)\n", err);

	WLAND_DBG(CFG80211, DEBUG, "Done.\n");
}

static void wland_connect_restorework_timeout(ulong data)
{
	struct wland_cfg80211_connect_info *conn_info =
		(struct wland_cfg80211_connect_info *) data;

	WLAND_DBG(CFG80211, TRACE, "restorework timer expired\n");
	schedule_work(&conn_info->connect_restorework_timeout_work);
}

static void cfg80211_connect_restorework_timeout_worker(struct work_struct *work)
{
	struct wland_cfg80211_connect_info *conn_info =
		container_of(work, struct wland_cfg80211_connect_info,
		connect_restorework_timeout_work);
	struct wland_if *ifp = netdev_priv(conn_info->ndev);

	WLAND_DBG(CFG80211, DEBUG, "Enter\n");
	if (timer_pending(&conn_info->connect_restorework_timeout)) {
		del_timer_sync(&conn_info->connect_restorework_timeout);
		WLAND_DBG(CFG80211, TRACE, "###### delete connect restoreworktimer\n");
	}

	wland_assoc_power_save(ifp->drvr);
	//restore tx rate
	wland_set_txrate(ifp, 0);

	WLAND_DBG(CFG80211, TRACE, "Done.\n");
}

static s32 wland_init_connect_info(struct wland_cfg80211_info *cfg)
{
	struct wland_cfg80211_connect_info *conn_info = cfg_to_conn(cfg);
	int err = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	conn_info->ndev = cfg_to_ndev(cfg);
	init_timer(&conn_info->timer);
	conn_info->timer.data = (unsigned long) conn_info;
	conn_info->timer.function = wland_connect_timer;
	conn_info->timer_on = false;
	INIT_WORK(&conn_info->work, wland_cfg80211_connect_worker);

	init_timer(&conn_info->connect_restorework_timeout);
	conn_info->connect_restorework_timeout.data = (unsigned long) conn_info;
	conn_info->connect_restorework_timeout.function =
		wland_connect_restorework_timeout;
	INIT_WORK(&conn_info->connect_restorework_timeout_work,
		cfg80211_connect_restorework_timeout_worker);

	conn_info->data = cfg;

	conn_info->connect_wq = create_singlethread_workqueue("connect_wq");
	if (!conn_info->connect_wq) {
		WLAND_ERR("insufficient memory to create txworkqueue.\n");
		err = -1;
	}

	WLAND_DBG(CFG80211, TRACE, "Exit\n");
	return err;
}

static s32 wland_deinit_connect_info(struct wland_cfg80211_info *cfg)
{
	struct wland_cfg80211_connect_info *conn_info = cfg_to_conn(cfg);
	int err = 0;

	WLAND_DBG(CFG80211, DEBUG, "Enter\n");

	if (timer_pending(&conn_info->timer)) {
		del_timer_sync(&conn_info->timer);
		WLAND_DBG(CFG80211, TRACE, "###### delete conn_info->timer\n");
	}
	if (timer_pending(&conn_info->connect_restorework_timeout)) {
		del_timer_sync(&conn_info->connect_restorework_timeout);
		WLAND_DBG(CFG80211, TRACE, "###### delete conn_info->connect_restorework_timeout\n");
	}
	cancel_work_sync(&conn_info->work);
	if (conn_info->connect_wq)
		destroy_workqueue(conn_info->connect_wq);

	WLAND_DBG(CFG80211, DEBUG, "Exit\n");
	return err;
}

static bool compare_update_same_bss(struct wland_cfg80211_info *cfg,
	struct wland_bss_info_le *bss, struct wland_bss_info_le *new_bss)
{
	struct list_head *p_next, *p_prev;
#ifdef WLAND_TBD_SUPPORT
	struct wland_chan ch_bss, ch_bss_info_le;
	ch_bss.chspec = bss->chanspec;
	cfg->d11inf.decchspec(&ch_bss);
	ch_bss_info_le.chspec = new_bss->chanspec;
	cfg->d11inf.decchspec(&ch_bss_info_le);

	/*
	 * A network is only a duplicate if the channel, BSSID, and ESSID
	 * * all match.  We treat all <hidden> with the same BSSID and channel
	 * * as one network
	 */
	if (!compare_ether_addr(bss->BSSID, new_bss->BSSID) &&
		ch_bss.band == ch_bss_info_le.band &&
		new_bss->SSID_len == bss->SSID_len &&
		!memcmp(new_bss->SSID, bss->SSID, new_bss->SSID_len))
#else /*WLAND_TBD_SUPPORT */
	if (((bss->ctl_ch == new_bss->ctl_ch) &&
			!compare_ether_addr(bss->BSSID, new_bss->BSSID) &&
			new_bss->SSID_len == bss->SSID_len &&
			!memcmp(bss->SSID, new_bss->SSID, new_bss->SSID_len)))
#endif /*WLAND_TBD_SUPPORT */
	{
		//If the network in list, then update it.
		/*
		 * preserve max RSSI if the measurements are both on-channel or both off-channel
		 */
		if(bss->ie)
			kfree(bss->ie);

		p_next = bss->list.next;
		p_prev= bss->list.prev;
		memcpy(bss, new_bss, sizeof(struct wland_bss_info_le));
		bss->list.next = p_next;
		bss->list.prev= p_prev;

		return true;
	}
	return false;
}

void ParseNetworkInfo(struct wland_bss_info_le *bi, u8 * pu8MsgBuffer)
{
	u8 u8MsgType = 0, u8MsgID = 0, u8index = 0;
	u16 u16MsgLen = 0, u16WidID = 0, u16WidLen = 0, u16MsaLen;
	u8 *pu8WidVal, *pbuf;

	u8MsgType = pu8MsgBuffer[0];

	/*
	 * Check whether the received message type is 'N'
	 */
	if (('N' != u8MsgType) || (!bi)) {
		WLAND_ERR("Received Message format incorrect.\n");
		return;
	}

	/*
	 * Extract message ID
	 */
	u8MsgID = pu8MsgBuffer[1];

	/*
	 * Extract message Length
	 */
	u16MsgLen = MAKE_WORD16(pu8MsgBuffer[2], pu8MsgBuffer[3]);

	/*
	 * Extract WID ID
	 */
	u16WidID = MAKE_WORD16(pu8MsgBuffer[4], pu8MsgBuffer[5]);

	/*
	 * Extract WID Length
	 */
	u16WidLen = MAKE_WORD16(pu8MsgBuffer[6], pu8MsgBuffer[7]);

	WLAND_DBG(CFG80211, TRACE,
		"u8MsgID:%d, u16MsgLen:%d, u16WidID:0x%04x, u16WidLen:%d\n",
		u8MsgID, u16MsgLen, u16WidID, u16WidLen);

	/*
	 * first byte rssi
	 */
#ifdef WLAND_RSSIOFFSET_SUPPORT
	if(pu8MsgBuffer[8] < WLAND_RSSI_MAXVAL_FOR_OFFSET)
		bi->RSSI = (signed char)(pu8MsgBuffer[8] + WLAND_RSSI_OFFSET);
	else
		bi->RSSI = (signed char)(pu8MsgBuffer[8]);
#else
	bi->RSSI = (signed char)(pu8MsgBuffer[8]);
#endif
	/*
	 * Assign a pointer to the WID value
	 */
	pu8WidVal = &pu8MsgBuffer[9];

	u16MsaLen = u16WidLen - 1;

	/*
	 * Get the cap_info
	 */
	bi->capability = get_cap_info(pu8WidVal);

	/*
	 * Get SSID
	 */
	get_ssid(pu8WidVal, bi->SSID, &(bi->SSID_len));

	/*
	 * Get BSSID
	 */
	get_BSSID(pu8WidVal, bi->BSSID);

	/*
	 * Get the current channel
	 */
	bi->ctl_ch = get_current_channel(pu8WidVal, (u16MsaLen + FCS_LEN));

	pbuf = get_data_rate(pu8WidVal, (u16MsaLen + FCS_LEN),
		WLAN_EID_SUPP_RATES, &u8index);
	if (pbuf) {
		bi->rateset.count = min_t(u8, MAX_RATES, u8index);
		memcpy(bi->rateset.rates, pbuf, bi->rateset.count);
	}

	/*
	 * Get beacon period
	 */
	u8index = (MAC_HDR_LEN + TIME_STAMP_LEN);

	bi->beacon_period = get_beacon_period(pu8WidVal + u8index);

	u8index += BEACON_INTERVAL_LEN + CAP_INFO_LEN;

	/*
	 * Get DTIM Period
	 */
	pbuf = get_tim_elm(pu8WidVal, (u16MsaLen + FCS_LEN), u8index);
	if (pbuf) {
		bi->dtim_period = pbuf[3];
	}

	pbuf = get_data_rate(pu8WidVal, (u16MsaLen + FCS_LEN),
		WLAN_EID_EXT_SUPP_RATES, &u8index);
	if (pbuf) {
		if ((u8index + bi->rateset.count) < (MAX_RATES + 2)) {
			memcpy(&bi->rateset.rates[bi->rateset.count + 1], pbuf,
				u8index);
			bi->rateset.count += u8index;
		} else {
			memcpy(&bi->rateset.rates[bi->rateset.count + 1], pbuf,
				(MAX_RATES + 1 - bi->rateset.count));
			bi->rateset.count = MAX_RATES + 1;
		}
	}

	bi->ie_length = (u16MsaLen - (MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN + CAP_INFO_LEN));
	bi->length = sizeof(struct wland_bss_info_le);
	bi->ie = kmalloc(bi->ie_length, GFP_KERNEL);
	if (!(bi->ie)) {
		WLAND_ERR("failed to allocate memory\n");
		return;
	}
	memcpy(bi->ie, &pu8WidVal[MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN + CAP_INFO_LEN], bi->ie_length);

	WLAND_DBG(CFG80211, TRACE,
		"bi->SSID(%s), bi->ie_length:%d, bi->ie:%p, bi->length:%d\n",
		bi->SSID, bi->ie_length, bi->ie, bi->length);

}

static s32 notify_escan_handler(struct wland_if *ifp,
	const struct wland_event_msg *e, void *data)
{
	struct wland_cfg80211_info *cfg = ifp->drvr->config;
	struct wland_bss_info_le *bss = NULL;
	struct wland_bss_info_le bss_info_le;
	struct wland_bss_info_le *new_bss;
	s32 err = 0;

	if (!check_vif_up(ifp->vif)) {
		WLAND_DBG(CFG80211, ERROR, "Device is not ready\n");
		goto exit;
	}

	if (e->status == STATUS_SUCCESS) {
		if (!data) {
			WLAND_ERR("Invalid escan result (NULL pointer)\n");
			goto exit;
		}
		memset(&bss_info_le, '\0', sizeof(struct wland_bss_info_le));
		ParseNetworkInfo(&bss_info_le, (u8 *) data);
		if (bss_info_le.ctl_ch == 0) {
			WLAND_ERR("failed to get bss channel number\n");
			goto exit;
		}

		/*
		 * Skip channel-14
		 */
#ifdef SKIP_REPORT_CHANNEL_14
		if (bss_info_le.ctl_ch == 14) {
			WLAND_DBG(CFG80211, DEBUG, "Skip Report Channel 14 \n");
			memset(&bss_info_le, '\0',
				sizeof(struct wland_bss_info_le));
			goto exit;
		}
#endif /*SKIP_REPORT_CHANNEL_14 */
#ifdef WLAND_P2P_SUPPORT
		if (wland_p2p_scan_finding_common_channel(cfg, &bss_info_le))
			goto exit;
#endif /* WLAND_P2P_SUPPORT */

		if (!(cfg_to_wiphy(cfg)->interface_modes &
				BIT(NL80211_IFTYPE_ADHOC))) {
			if (bss_info_le.capability & CAPABILITY_IBSS) {
				WLAND_DBG(CFG80211, DEBUG,
					"Ignoring IBSS result(SSID: %s).\n",
					bss_info_le.SSID);
				goto exit;
			}
		}
		WLAND_DBG(CFG80211, DEBUG,
			"Get BSS(BSSID:%pM, RSSI:%d, SSID:\"%s\")\n",
			bss_info_le.BSSID, bss_info_le.RSSI, bss_info_le.SSID);

		mutex_lock(&cfg->scan_result_lock);
		list_for_each_entry(bss, &cfg->scan_result_list, list) {
			if (compare_update_same_bss(cfg, bss, &bss_info_le)) {
				bss->time = jiffies;
				mutex_unlock(&cfg->scan_result_lock);
				goto exit;
			}
		}

		WLAND_DBG(CFG80211, DEBUG,
			"list->count:%u, NewBSS(BSSID:%pM, RSSI:%d, SSID:\"%s\")\n",
			cfg->scan_results.count, bss_info_le.BSSID,
			bss_info_le.RSSI, bss_info_le.SSID);

		bss_info_le.time = jiffies;

		/*
		 * store bss header to buf
		 */
		new_bss = wland_alloc_bss(cfg, &bss_info_le);
		if (IS_ERR(new_bss)) {
			WLAND_ERR("wland_alloc_bss error\n");
			err = PTR_ERR(new_bss);
		}
		mutex_unlock(&cfg->scan_result_lock);

	} else {
		WLAND_DBG(CFG80211, TRACE,
			"Scan Done(timeout or other reason)\n");

		cfg->scan_info.escan_state = SCAN_STATE_IDLE;

		if (cfg->scan_request)
			wland_inform_bss(cfg);
		else
			WLAND_DBG(CFG80211, DEBUG,
				"Ignored scan complete result 0x%x\n", e->status);

		if (test_bit(SCAN_STATUS_BUSY, &cfg->scan_status)) {
			clear_bit(SCAN_STATUS_BUSY, &cfg->scan_status);
		}
	}

exit:
	return err;
}

static s32 cfg80211_resume(struct wiphy *wiphy)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);

	if (test_bit(SCAN_STATUS_BUSY, &cfg->scan_status))
		WLAND_DBG(CFG80211, TRACE, "scanning......\n");

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	return 0;
}

static s32 cfg80211_suspend(struct wiphy *wiphy
	, struct cfg80211_wowlan *wow
	)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct net_device *ndev = cfg_to_ndev(cfg);
	struct wland_cfg80211_vif *vif = ndev_to_vif(ndev);

	/*
	 * if the primary net_device is not READY there is nothing we can do but pray resume goes smoothly.
	 */
	if (!check_vif_up(vif))
		goto exit;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	list_for_each_entry(vif, &cfg->vif_list, list) {
		if (!test_bit(VIF_STATUS_READY, &vif->sme_state))
			continue;
		/*
		 * While going to suspend if associated with AP disassociate
		 * from AP to save power while system is in suspended state
		 */
		//wland_link_down(vif);

		/*
		 * Make sure WPA_Supplicant receives all the event
		 * * generated due to DISASSOC call to the fw to keep
		 * * the state fw and WPA_Supplicant state consistent
		 */
		wland_delay(200);
	}
	/*
	 * end any scanning
	 */ if (test_bit(SCAN_STATUS_BUSY, &cfg->scan_status))
		wland_abort_scanning(cfg);

exit:
	WLAND_DBG(CFG80211, TRACE, "Done\n");
	/*
	 * clear any scanning activity
	 */
	cfg->scan_status = 0;
	return 0;
}

static __used s32 wland_update_pmklist(struct net_device *ndev,
	struct wland_cfg80211_pmk_list *pmk_list, s32 err)
{
	int i, j;
	int pmkid_len = le32_to_cpu(pmk_list->pmkids.npmkid);

	WLAND_DBG(CFG80211, TRACE, "No of elements %d\n", pmkid_len);

	for (i = 0; i < pmkid_len; i++) {
		WLAND_DBG(CFG80211, TRACE, "PMKID[%d]: %pM =\n", i,
			&pmk_list->pmkids.pmkid[i].BSSID);

		for (j = 0; j < WLAN_PMKID_LEN; j++)
			WLAND_DBG(CFG80211, TRACE, "%02x\n",
				pmk_list->pmkids.pmkid[i].PMKID[j]);
	}
	if (!err)
		wland_fil_iovar_data_set(netdev_priv(ndev), "pmkid_info",
			(char *) pmk_list, sizeof(*pmk_list));

	return err;
}

static s32 cfg80211_set_pmksa(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_pmksa *pmksa)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct wland_if *ifp = netdev_priv(ndev);
	struct pmkid_list *pmkids = &cfg->pmk_list->pmkids;
	s32 err = 0;
	int i, pmkid_len;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (!check_vif_up(ifp->vif))
		return -EIO;

	pmkid_len = le32_to_cpu(pmkids->npmkid);

	for (i = 0; i < pmkid_len; i++) {
		if (!memcmp(pmksa->bssid, pmkids->pmkid[i].BSSID, ETH_ALEN))
			break;
	}
	if (i < MAXPMKID) {
		memcpy(pmkids->pmkid[i].BSSID, pmksa->bssid, ETH_ALEN);
		memcpy(pmkids->pmkid[i].PMKID, pmksa->pmkid, WLAN_PMKID_LEN);

		if (i == pmkid_len) {
			pmkid_len++;
			pmkids->npmkid = cpu_to_le32(pmkid_len);
		}
	} else {
		err = -EINVAL;
	}

	WLAND_DBG(CFG80211, TRACE, "set_pmksa,IW_PMKSA_ADD - PMKID: %pM =\n",
		pmkids->pmkid[pmkid_len].BSSID);

	for (i = 0; i < WLAN_PMKID_LEN; i++)
		WLAND_DBG(CFG80211, TRACE, "%02x\n",
			pmkids->pmkid[pmkid_len].PMKID[i]);

	err = wland_update_pmklist(ndev, cfg->pmk_list, err);

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);
	return err;
}

static s32 cfg80211_del_pmksa(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_pmksa *pmksa)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct wland_if *ifp = netdev_priv(ndev);
	struct pmkid_list pmkid;
	s32 err = 0;
	int i, pmkid_len;

	if (!check_vif_up(ifp->vif))
		return -EIO;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	memcpy(&pmkid.pmkid[0].BSSID, pmksa->bssid, ETH_ALEN);
	memcpy(&pmkid.pmkid[0].PMKID, pmksa->pmkid, WLAN_PMKID_LEN);

	WLAND_DBG(CFG80211, TRACE,
		"del_pmksa,IW_PMKSA_REMOVE - PMKID: %pM =\n",
		&pmkid.pmkid[0].BSSID);

	for (i = 0; i < WLAN_PMKID_LEN; i++)
		WLAND_DBG(CFG80211, TRACE, "%02x\n", pmkid.pmkid[0].PMKID[i]);

	pmkid_len = le32_to_cpu(cfg->pmk_list->pmkids.npmkid);

	for (i = 0; i < pmkid_len; i++)
		if (!memcmp(pmksa->bssid, &cfg->pmk_list->pmkids.pmkid[i].BSSID,
				ETH_ALEN))
			break;

	if ((pmkid_len > 0) && (i < pmkid_len)) {
		memset(&cfg->pmk_list->pmkids.pmkid[i], 0,
			sizeof(struct pmkid));

		for (; i < (pmkid_len - 1); i++) {
			memcpy(&cfg->pmk_list->pmkids.pmkid[i].BSSID,
				&cfg->pmk_list->pmkids.pmkid[i + 1].BSSID,
				ETH_ALEN);
			memcpy(&cfg->pmk_list->pmkids.pmkid[i].PMKID,
				&cfg->pmk_list->pmkids.pmkid[i + 1].PMKID,
				WLAN_PMKID_LEN);
		}
		cfg->pmk_list->pmkids.npmkid = cpu_to_le32(pmkid_len - 1);
	} else {
		err = -EINVAL;
	}

	err = wland_update_pmklist(ndev, cfg->pmk_list, err);

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return err;
}

static s32 cfg80211_flush_pmksa(struct wiphy *wiphy, struct net_device *ndev)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct wland_if *ifp = netdev_priv(ndev);
	s32 err = 0;

	if (!check_vif_up(ifp->vif))
		return -EIO;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	memset(cfg->pmk_list, 0, sizeof(struct wland_cfg80211_pmk_list));

	err = wland_update_pmklist(ndev, cfg->pmk_list, err);

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return err;
}

/*
 * PFN result doesn't have all the info which are
 * required by the supplicant
 * (For e.g IEs) Do a target Escan so that sched scan results are reported
 * via wl_inform_single_bss in the required format. Escan does require the
 * scan request in the form of cfg80211_scan_request. For timebeing, create
 * cfg80211_scan_request one out of the received PNO event.
 */
 static s32 notify_sched_scan_results(struct wland_if *ifp,
	const struct wland_event_msg *e, void *data)
{
	struct wland_cfg80211_info *cfg = ifp->drvr->config;
	struct wland_pno_net_info_le *netinfo, *netinfo_start;
	struct cfg80211_scan_request *request = NULL;
	struct cfg80211_ssid *ssid = NULL;
	struct ieee80211_channel *channel = NULL;
	struct wiphy *wiphy = cfg_to_wiphy(cfg);
	struct wland_pno_scanresults_le *pfn_result =
		(struct wland_pno_scanresults_le *) data;
	s32 err = 0, channel_req = 0, band = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	/*
	 * PFN event is limited to fit 512 bytes so we may get multiple NET_FOUND events. For now place a warning here.
	 */
	WARN_ON(pfn_result->status != WLAND_PNO_SCAN_COMPLETE);

	WLAND_DBG(CFG80211, TRACE, "PFN NET FOUND event. count: %d\n",
		pfn_result->count);

	if (pfn_result->count > 0) {
		int i;

		request = kzalloc(sizeof(*request), GFP_KERNEL);
		ssid = kcalloc(pfn_result->count, sizeof(*ssid), GFP_KERNEL);
		channel =
			kcalloc(pfn_result->count, sizeof(*channel),
			GFP_KERNEL);
		if (!request || !ssid || !channel) {
			err = -ENOMEM;
			goto out_err;
		}

		request->wiphy = wiphy;

		data += sizeof(struct wland_pno_scanresults_le);
		netinfo_start = (struct wland_pno_net_info_le *) data;

		for (i = 0; i < pfn_result->count; i++) {
			netinfo = &netinfo_start[i];
			if (!netinfo) {
				WLAND_ERR("Invalid netinfo ptr. index: %d\n",
					i);
				err = -EINVAL;
				goto out_err;
			}

			WLAND_DBG(CFG80211, TRACE, "SSID:%s Channel:%d\n",
				netinfo->SSID, netinfo->channel);
			memcpy(ssid[i].ssid, netinfo->SSID, netinfo->SSID_len);
			ssid[i].ssid_len = netinfo->SSID_len;
			request->n_ssids++;

			channel_req = netinfo->channel;
			if (channel_req <= CH_MAX_2G_CHANNEL)
				band = NL80211_BAND_2GHZ;
			else
				band = NL80211_BAND_5GHZ;
			channel[i].center_freq =
				ieee80211_channel_to_frequency(channel_req,
				band);
			channel[i].band = band;
			channel[i].flags |= IEEE80211_CHAN_NO_HT40;

			request->channels[i] = &channel[i];
			request->n_channels++;
		}

		/*
		 * assign parsed ssid array
		 */
		if (request->n_ssids)
			request->ssids = &ssid[0];

		if (test_bit(SCAN_STATUS_BUSY, &cfg->scan_status))
			/*
			 * Abort any on-going scan
			 */
			wland_abort_scanning(cfg);

		set_bit(SCAN_STATUS_BUSY, &cfg->scan_status);

		err = wland_do_escan(cfg, wiphy, ifp, request);
		if (err < 0) {
			clear_bit(SCAN_STATUS_BUSY, &cfg->scan_status);
			goto out_err;
		}
		cfg->sched_escan = true;
		cfg->scan_request = request;
	} else {
		WLAND_ERR("false PNO Event. (pfn_count == 0)\n");
		goto out_err;
	}

	WLAND_DBG(CFG80211, TRACE, "Done\n");

	kfree(ssid);
	kfree(channel);
	kfree(request);
	return 0;

out_err:
	kfree(ssid);
	kfree(channel);
	kfree(request);
	cfg80211_sched_scan_stopped(wiphy);

	WLAND_DBG(CFG80211, TRACE, "Done(error)\n");
	return err;
}

static int cfg80211_sched_scan_start(struct wiphy *wiphy,
	struct net_device *ndev, struct cfg80211_sched_scan_request *request)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_info *cfg = wiphy_priv(wiphy);
	struct wland_pno_net_param_le pfn;
	int i, enable = 1, ret = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter n_match_sets:%d n_ssids:%d\n",
		request->n_match_sets, request->n_ssids);

	if (test_bit(SCAN_STATUS_BUSY, &cfg->scan_status)) {
		WLAND_ERR("Scanning already: status (%lu)\n", cfg->scan_status);
		return -EAGAIN;
	}

	if (test_bit(SCAN_STATUS_SUPPRESS, &cfg->scan_status)) {
		WLAND_ERR("Scanning suppressed: status (%lu)\n",
			cfg->scan_status);
		return -EAGAIN;
	}

	if (!request->n_ssids || !request->n_match_sets) {
		WLAND_ERR("Invalid sched scan req!! n_ssids:%d\n",
			request->n_ssids);
		return -EINVAL;
	}

	if (request->n_ssids > 0) {
		for (i = 0; i < request->n_ssids; i++) {
			/*
			 * Active scan req for ssids
			 */
			WLAND_DBG(CFG80211, TRACE,
				">>> Active scan req for ssid (%s)\n",
				request->ssids[i].ssid);
			/*
			 * match_set ssids is a supert set of n_ssid list,so we need not add these set seperately.
			 */
		}
	}

	if (request->n_match_sets > 0) {
		/*
		 * configure pno
		 */
		struct wland_pno_param_le pfn_param;

		memset(&pfn_param, 0, sizeof(pfn_param));

		pfn_param.version = cpu_to_le32(WLAND_PNO_VERSION);

		/*
		 * set extra pno params
		 */
		pfn_param.flags =
			cpu_to_le16(1 << WLAND_PNO_ENABLE_ADAPTSCAN_BIT);
		pfn_param.repeat = WLAND_PNO_REPEAT;
		pfn_param.exp = WLAND_PNO_FREQ_EXPO_MAX;

		/*
		 * set up pno scan fr
		 */
		pfn_param.scan_freq = cpu_to_le32(WLAND_PNO_TIME);

		ret = wland_fil_iovar_data_set(netdev_priv(ndev), "pfn_set",
			&pfn_param, sizeof(pfn_param));
		if (ret < 0) {
			WLAND_ERR("PNO setup failed!! ret=%d\n", ret);
			return -EINVAL;
		}

		/*
		 * configure each match set
		 */
		for (i = 0; i < request->n_match_sets; i++) {
			struct cfg80211_ssid *ssid =
				&request->match_sets[i].ssid;
			u32 ssid_len = ssid->ssid_len;

			if (!ssid_len) {
				WLAND_ERR("skip broadcast ssid\n");
				continue;
			}
			pfn.auth = cpu_to_le32(AUTH_OPEN);
			pfn.wpa_auth = cpu_to_le32(WLAND_PNO_WPA_AUTH_ANY);
			pfn.wsec = cpu_to_le32(0);
			pfn.infra = cpu_to_le32(1);
			pfn.flags = cpu_to_le32(1 << WLAND_PNO_HIDDEN_BIT);
			pfn.ssid.SSID_len = cpu_to_le32(ssid_len);

			memcpy(pfn.ssid.SSID, ssid->ssid, ssid_len);

			ret = wland_fil_iovar_data_set(ifp, "pfn_add", &pfn,
				sizeof(pfn));

			WLAND_DBG(CFG80211, TRACE,
				">>> PNO filter %s for ssid (%s)\n",
				ret == 0 ? "set" : "failed", ssid->ssid);
		}

		ret = wland_fil_iovar_data_set(ifp, "pfn", &enable, sizeof(u8));

		/*
		 * Enable the PNO
		 */
		if (ret < 0) {
			WLAND_ERR("PNO enable failed!! ret=%d\n", ret);
			ret = -EINVAL;
		}
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int cfg80211_sched_scan_stop(struct wiphy *wiphy,
	struct net_device *ndev)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);

	WLAND_DBG(CFG80211, TRACE, "enter\n");

	if (cfg->sched_escan)
		wland_notify_escan_complete(cfg, netdev_priv(ndev), true, true);

	return 0;
}

static bool wland_valid_wpa_oui(u8 * oui, bool is_rsn_ie)
{
	if (is_rsn_ie)
		return (memcmp(oui, RSN_OUI, TLV_OUI_LEN) == 0);

	return (memcmp(oui, WPA_OUI, TLV_OUI_LEN) == 0);
}

static s32 wland_configure_wpaie(struct net_device *ndev,
	struct wland_vs_tlv *wpa_ie, bool is_rsn_ie)
{
	//struct wland_if               *ifp     = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	u16 count, rsn_cap;
	s32 err = 0, len = 0;
	u32 i, wsec = 0, pval = 0, gval = 0, wpa_auth =
		0, offset, wme_bss_disable, auth = OPEN_SYSTEM;
	u8 *data;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (wpa_ie == NULL)
		goto exit;

	len = wpa_ie->len + TLV_HDR_LEN;
	data = (u8 *) wpa_ie;
	offset = TLV_HDR_LEN;

	if (!is_rsn_ie)
		offset += VS_IE_FIXED_HDR_LEN;
	else
		offset += WPA_IE_VERSION_LEN;

	/*
	 * check for multicast cipher suite
	 */
	if (offset + WPA_IE_MIN_OUI_LEN > len) {
		err = -EINVAL;
		WLAND_ERR("no multicast cipher suite\n");
		goto exit;
	}

	if (!wland_valid_wpa_oui(&data[offset], is_rsn_ie)) {
		err = -EINVAL;
		WLAND_ERR("ivalid OUI\n");
		goto exit;
	}
	offset += TLV_OUI_LEN;

	/*
	 * pick up multicast cipher
	 */
	switch (data[offset]) {
	case WPA_CIPHER_NONE:
		WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_NONE\n");
		gval = 0;
		break;
	case WPA_CIPHER_WEP_40:
		WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_WEP_40\n");
	case WPA_CIPHER_WEP_104:
		WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_WEP_104\n");
		gval = WEP_ENABLED;
		break;
	case WPA_CIPHER_TKIP:
		WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_TKIP\n");
		gval = TKIP_ENABLED;
		break;
	case WPA_CIPHER_AES_CCM:
		WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_AES_CCM\n");
		gval = AES_ENABLED;
		break;
#ifdef WLAND_WAPI_SUPPORT
	case WAPI_CIPHER_SMS4:
		WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_NONE\n");
		gval = SMS4_ENABLED;
		break;
#endif /*WLAND_WAPI_SUPPORT */
	default:
		err = -EINVAL;
		WLAND_ERR("Invalid multi cast cipher info\n");
		goto exit;
	}

	offset++;
	/*
	 * walk thru unicast cipher list and pick up what we recognize
	 */
	count = data[offset] + (data[offset + 1] << 8);
	offset += WPA_IE_SUITE_COUNT_LEN;
	/*
	 * Check for unicast suite(s)
	 */
	if (offset + (WPA_IE_MIN_OUI_LEN * count) > len) {
		err = -EINVAL;
		WLAND_ERR("no unicast cipher suite\n");
		goto exit;
	}

	for (i = 0; i < count; i++) {
		if (!wland_valid_wpa_oui(&data[offset], is_rsn_ie)) {
			err = -EINVAL;
			WLAND_ERR("ivalid OUI\n");
			goto exit;
		}
		offset += TLV_OUI_LEN;
		switch (data[offset]) {
		case WPA_CIPHER_NONE:
			WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_NONE\n");
			break;
		case WPA_CIPHER_WEP_40:
			WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_WEP_40\n");
		case WPA_CIPHER_WEP_104:
			WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_WEP_104\n");
			pval |= WEP_ENABLED;
			break;
		case WPA_CIPHER_TKIP:
			WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_TKIP\n");
			pval |= TKIP_ENABLED;
			break;
		case WPA_CIPHER_AES_CCM:
			WLAND_DBG(CFG80211, TRACE, "WPA_CIPHER_AES_CCM\n");
			pval |= AES_ENABLED;
			break;
#ifdef WLAND_WAPI_SUPPORT
		case WAPI_CIPHER_SMS4:
			WLAND_DBG(CFG80211, TRACE, "WAPI_CIPHER_SMS4\n");
			pval |= SMS4_ENABLED;
			break;
#endif /*WLAND_WAPI_SUPPORT */
		default:
			WLAND_ERR("Ivalid unicast security info\n");
		}
		offset++;
	}
	/*
	 * walk thru auth management suite list and pick up what we recognize
	 */
	count = data[offset] + (data[offset + 1] << 8);
	offset += WPA_IE_SUITE_COUNT_LEN;

	/*
	 * Check for auth key management suite(s)
	 */
	if (offset + (WPA_IE_MIN_OUI_LEN * count) > len) {
		err = -EINVAL;
		WLAND_ERR("no auth key mgmt suite\n");
		goto exit;
	}

	for (i = 0; i < count; i++) {
		if (!wland_valid_wpa_oui(&data[offset], is_rsn_ie)) {
			err = -EINVAL;
			WLAND_ERR("ivalid OUI\n");
			goto exit;
		}
		offset += TLV_OUI_LEN;
		switch (data[offset]) {
		case RSN_AKM_NONE:
			WLAND_DBG(CFG80211, TRACE, "RSN_AKM_NONE\n");
			wpa_auth |= WPA_AUTH_NONE;
			break;
		case RSN_AKM_UNSPECIFIED:
			WLAND_DBG(CFG80211, TRACE, "RSN_AKM_UNSPECIFIED\n");
			is_rsn_ie ? (wpa_auth |=
				WPA2_AUTH_UNSPECIFIED) : (wpa_auth |=
				WPA_AUTH_UNSPECIFIED);
			break;
		case RSN_AKM_PSK:
			WLAND_DBG(CFG80211, TRACE, "RSN_AKM_PSK\n");
			is_rsn_ie ? (wpa_auth |= WPA2_AUTH_PSK) : (wpa_auth |=
				WPA_AUTH_PSK);
			break;
		default:
			WLAND_ERR("Ivalid key mgmt info\n");
		}
		offset++;
	}

	if (is_rsn_ie) {
		wme_bss_disable = 1;
		if ((offset + RSN_CAP_LEN) <= len) {
			rsn_cap = data[offset] + (data[offset + 1] << 8);
			profile->rsn_cap = (rsn_cap >> 2) & 0x03;
			WLAND_DBG(CFG80211, DEBUG, "profile->rsn_cap: 0x%02x\n",
				profile->rsn_cap);
			if (rsn_cap & RSN_CAP_PTK_REPLAY_CNTR_MASK)
				wme_bss_disable = 0;
		}
#if 0
		/*
		 * set wme_bss_disable to sync RSN Capabilities
		 */
		err = wland_fil_iovar_data_set(ifp, "wme_bss_disable",
			&wme_bss_disable, sizeof(wme_bss_disable));
		if (err < 0) {
			WLAND_ERR("wme_bss_disable error %d\n", err);
			goto exit;
		}
#endif
	}
	/*
	 * FOR WPS , set SES_OW_ENABLED
	 */
	wsec = (pval | gval | SES_OW_ENABLED);

	//profile->sec.firmware_autype= auth;
	//profile->sec.wpa_auth       = wpa_auth;
	//profile->sec.auth_type      = wsec;

exit:
	WLAND_DBG(CFG80211, TRACE, "Done(auth:%d,wpa_auth:%d,wsec:%d,err:%d)\n",
		auth, wpa_auth, wsec, err);
	return err;
}

static s32 wland_parse_vndr_ies(const u8 * vndr_ie_buf, u32 vndr_ie_len,
	struct parsed_vndr_ies *vndr_ies)
{
	s32 err = 0, remaining_len;
	struct wland_vs_tlv *vndrie;
	struct wland_tlv *ie;
	struct parsed_vndr_ie_info *parsed_info;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	remaining_len = (s32) vndr_ie_len;

	memset(vndr_ies, 0, sizeof(*vndr_ies));

	ie = (struct wland_tlv *) vndr_ie_buf;

	while (ie) {
		if (ie->id != WLAN_EID_VENDOR_SPECIFIC)
			goto next;
		vndrie = (struct wland_vs_tlv *) ie;
		/*
		 * len should be bigger than OUI length + one
		 */
		if (vndrie->len < (VS_IE_FIXED_HDR_LEN - TLV_HDR_LEN + 1)) {
			WLAND_ERR("invalid vndr ie. length is too small %d\n",
				vndrie->len);
			goto next;
		}

		/*
		 * if wpa or wme ie, do not add ie
		 */
		if (!memcmp(vndrie->oui, (u8 *) WPA_OUI, TLV_OUI_LEN) &&
			((vndrie->oui_type == WPA_OUI_TYPE)
				|| (vndrie->oui_type == WME_OUI_TYPE))) {
			WLAND_DBG(CFG80211, TRACE,
				"Found WPA/WME oui. Do not add it\n");
			goto next;
		}

		parsed_info = &vndr_ies->ie_info[vndr_ies->count];

		/*
		 * save vndr ie information
		 */
		parsed_info->ie_ptr = (char *) vndrie;
		parsed_info->ie_len = vndrie->len + TLV_HDR_LEN;

		memcpy(&parsed_info->vndrie, vndrie, sizeof(*vndrie));

		vndr_ies->count++;

		WLAND_DBG(CFG80211, TRACE,
			"** OUI %02x %02x %02x, type 0x%02x\n",
			parsed_info->vndrie.oui[0], parsed_info->vndrie.oui[1],
			parsed_info->vndrie.oui[2],
			parsed_info->vndrie.oui_type);

		if (vndr_ies->count >= VNDR_IE_PARSE_LIMIT)
			break;
next:
		remaining_len -= (ie->len + TLV_HDR_LEN);

		if (remaining_len <= TLV_HDR_LEN)
			ie = NULL;
		else
			ie = (struct wland_tlv *) (((u8 *) ie) + ie->len +
				TLV_HDR_LEN);
	}
	WLAND_DBG(CFG80211, TRACE, "Done\n");

	return err;
}

static u32 wland_vndr_ie(u8 * iebuf, s32 pktflag, u8 * ie_ptr, u32 ie_len,
	s8 * add_del_cmd)
{
	__le32 iecount_le;
	__le32 pktflag_le;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	strncpy(iebuf, add_del_cmd, VNDR_IE_CMD_LEN - 1);
	iebuf[VNDR_IE_CMD_LEN - 1] = '\0';

	iecount_le = cpu_to_le32(1);

	memcpy(&iebuf[VNDR_IE_COUNT_OFFSET], &iecount_le, sizeof(iecount_le));

	pktflag_le = cpu_to_le32(pktflag);

	memcpy(&iebuf[VNDR_IE_PKTFLAG_OFFSET], &pktflag_le, sizeof(pktflag_le));

	memcpy(&iebuf[VNDR_IE_VSIE_OFFSET], ie_ptr, ie_len);

	return ie_len + VNDR_IE_HDR_SIZE;
}

s32 wland_vif_set_mgmt_ie(struct wland_cfg80211_vif * vif, s32 pktflag,
	const u8 * vndr_ie_buf, u32 vndr_ie_len)
{
	struct parsed_vndr_ies old_vndr_ies;
	struct parsed_vndr_ies new_vndr_ies;
	struct parsed_vndr_ie_info *vndrie_info;
	struct wland_if *ifp;
	struct vif_saved_ie *saved_ie;
	s32 err = 0, i;
	u8 *iovar_ie_buf;
	u8 *curr_ie_buf;
	u8 *mgmt_ie_buf = NULL;
	int mgmt_ie_buf_len, remained_buf_len;
	u32 *mgmt_ie_len;
	u32 del_add_ie_buf_len = 0, total_ie_buf_len = 0, parsed_ie_buf_len = 0;
	u8 *ptr;

	if (!vif)
		return -ENODEV;

	ifp = vif->ifp;
	saved_ie = &vif->saved_ie;

	WLAND_DBG(CFG80211, TRACE, "Enter(bssidx:%d,pktflag:0x%02X)\n",
		ifp->bssidx, pktflag);

	iovar_ie_buf = kzalloc(WLAND_EXTRA_BUF_MAX, GFP_KERNEL);
	if (!iovar_ie_buf)
		return -ENOMEM;
	curr_ie_buf = iovar_ie_buf;

	switch (pktflag) {
	case WLAND_VNDR_IE_PRBREQ_FLAG:
		mgmt_ie_buf = saved_ie->probe_req_ie;
		mgmt_ie_len = &saved_ie->probe_req_ie_len;
		mgmt_ie_buf_len = sizeof(saved_ie->probe_req_ie);
		break;
	case WLAND_VNDR_IE_PRBRSP_FLAG:
		mgmt_ie_buf = saved_ie->probe_res_ie;
		mgmt_ie_len = &saved_ie->probe_res_ie_len;
		mgmt_ie_buf_len = sizeof(saved_ie->probe_res_ie);
		break;
	case WLAND_VNDR_IE_BEACON_FLAG:
		mgmt_ie_buf = saved_ie->beacon_ie;
		mgmt_ie_len = &saved_ie->beacon_ie_len;
		mgmt_ie_buf_len = sizeof(saved_ie->beacon_ie);
		break;
	case WLAND_VNDR_IE_ASSOCREQ_FLAG:
		mgmt_ie_buf = saved_ie->assoc_req_ie;
		mgmt_ie_len = &saved_ie->assoc_req_ie_len;
		mgmt_ie_buf_len = sizeof(saved_ie->assoc_req_ie);
		break;
	default:
		err = -EPERM;
		WLAND_ERR("not suitable type\n");
		goto exit;
	}

	if (vndr_ie_len > mgmt_ie_buf_len) {
		err = -ENOMEM;
		WLAND_ERR("extra IE size too big\n");
		goto exit;
	}

	/*
	 * parse and save new vndr_ie in curr_ie_buff before comparing it
	 */
	if (vndr_ie_buf && vndr_ie_len && curr_ie_buf) {
		ptr = curr_ie_buf;
		wland_parse_vndr_ies(vndr_ie_buf, vndr_ie_len, &new_vndr_ies);

		for (i = 0; i < new_vndr_ies.count; i++) {
			vndrie_info = &new_vndr_ies.ie_info[i];
			memcpy(ptr + parsed_ie_buf_len, vndrie_info->ie_ptr,
				vndrie_info->ie_len);
			parsed_ie_buf_len += vndrie_info->ie_len;
		}
	}

	if (mgmt_ie_buf && *mgmt_ie_len) {
		if (parsed_ie_buf_len && (parsed_ie_buf_len == *mgmt_ie_len)
			&& (memcmp(mgmt_ie_buf, curr_ie_buf,
					parsed_ie_buf_len) == 0)) {
			WLAND_DBG(CFG80211, TRACE,
				"Previous mgmt IE equals to current IE\n");
			goto exit;
		}

		/*
		 * parse old vndr_ie
		 */
		wland_parse_vndr_ies(mgmt_ie_buf, *mgmt_ie_len, &old_vndr_ies);

		/*
		 * make a command to delete old ie
		 */
		for (i = 0; i < old_vndr_ies.count; i++) {
			vndrie_info = &old_vndr_ies.ie_info[i];

			WLAND_DBG(CFG80211, TRACE,
				"DEL ID : %d, Len: %d , OUI:%02x:%02x:%02x\n",
				vndrie_info->vndrie.id, vndrie_info->vndrie.len,
				vndrie_info->vndrie.oui[0],
				vndrie_info->vndrie.oui[1],
				vndrie_info->vndrie.oui[2]);

			del_add_ie_buf_len =
				wland_vndr_ie(curr_ie_buf, pktflag,
				vndrie_info->ie_ptr, vndrie_info->ie_len,
				"del");
			curr_ie_buf += del_add_ie_buf_len;
			total_ie_buf_len += del_add_ie_buf_len;
		}
	}

	*mgmt_ie_len = 0;

	/*
	 * Add if there is any extra IE
	 */
	if (mgmt_ie_buf && parsed_ie_buf_len) {
		ptr = mgmt_ie_buf;

		remained_buf_len = mgmt_ie_buf_len;

		/*
		 * make a command to add new ie
		 */
		for (i = 0; i < new_vndr_ies.count; i++) {
			vndrie_info = &new_vndr_ies.ie_info[i];

			/*
			 * verify remained buf size before copy data
			 */
			if (remained_buf_len <
				(vndrie_info->vndrie.len +
					VNDR_IE_VSIE_OFFSET)) {
				WLAND_ERR
					("no space in mgmt_ie_buf: len left %d",
					remained_buf_len);
				break;
			}
			remained_buf_len -=
				(vndrie_info->ie_len + VNDR_IE_VSIE_OFFSET);

			WLAND_DBG(CFG80211, TRACE,
				"ADDED ID : %d, Len: %d, OUI:%02x:%02x:%02x\n",
				vndrie_info->vndrie.id, vndrie_info->vndrie.len,
				vndrie_info->vndrie.oui[0],
				vndrie_info->vndrie.oui[1],
				vndrie_info->vndrie.oui[2]);

			del_add_ie_buf_len =
				wland_vndr_ie(curr_ie_buf, pktflag,
				vndrie_info->ie_ptr, vndrie_info->ie_len,
				"add");

			/*
			 * save the parsed IE in wl struct
			 */
			memcpy(ptr + (*mgmt_ie_len), vndrie_info->ie_ptr,
				vndrie_info->ie_len);
			*mgmt_ie_len += vndrie_info->ie_len;

			curr_ie_buf += del_add_ie_buf_len;
			total_ie_buf_len += del_add_ie_buf_len;
		}
	}

	if (total_ie_buf_len) {
		err = wland_fil_iovar_data_set(ifp, "vndr_ie", iovar_ie_buf,
			total_ie_buf_len);
		if (err < 0)
			WLAND_ERR("vndr ie set error : %d\n", err);
	}

exit:
	kfree(iovar_ie_buf);
	return err;
}

s32 wland_vif_clear_mgmt_ies(struct wland_cfg80211_vif * vif)
{
	s32 i, pktflags[] = {
		WLAND_VNDR_IE_PRBREQ_FLAG,
		WLAND_VNDR_IE_PRBRSP_FLAG, WLAND_VNDR_IE_BEACON_FLAG
	};

	for (i = 0; i < ARRAY_SIZE(pktflags); i++) {
		wland_vif_set_mgmt_ie(vif, pktflags[i], NULL, 0);
	}

	memset(&vif->saved_ie, 0, sizeof(vif->saved_ie));
	return 0;
}

#if  LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
static s32 cfg80211_modify_beacon(struct wiphy *wiphy, struct net_device *ndev,
	struct beacon_parameters *info)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_tlv *ssid_ie;
	struct wland_ssid_le ssid_le;
	struct wland_tlv *rsn_ie;
	struct wland_vs_tlv *wpa_ie;
	struct wland_join_params join_params;
	enum nl80211_iftype dev_role;
	struct wland_fil_bss_enable_le bss_enable;
	s32 ie_offset, err = -EPERM;

	dev_role = ifp->vif->wdev.iftype;

	memset(&ssid_le, 0, sizeof(ssid_le));

	WLAND_DBG(CFG80211, TRACE,
		"interval:(%d),dtim_period:(%d),head_len:(%d),tail_len:(%d)\n",
		info->interval, info->dtim_period, info->head_len,
		info->tail_len);

	//WLAND_DUMP(CFG80211, info->head, 128, "beacon_parameters,len:%Zu\n", (info->tail - info->head));

	/*
	 * find the SSID
	 */
	ie_offset = DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
	ssid_ie =
		wland_parse_tlvs((u8 *) & info->head[ie_offset],
		info->head_len - ie_offset, WLAN_EID_SSID);
	if (!ssid_ie)
		return -EINVAL;

	memcpy(ssid_le.SSID, ssid_ie->data, ssid_ie->len);
	ssid_le.SSID_len = cpu_to_le32(ssid_ie->len);

	WLAND_DBG(CFG80211, TRACE, "SSID is (%s) in Head\n", ssid_le.SSID);

	/*
	 * find the RSN_IE
	 */
	rsn_ie = wland_parse_tlvs((u8 *) info->tail, info->tail_len,
		WLAN_EID_RSN);

	/*
	 * find the WPA_IE
	 */
	wpa_ie = wland_find_wpaie((u8 *) info->tail, info->tail_len);

	if ((wpa_ie != NULL || rsn_ie != NULL)) {
		WLAND_DBG(CFG80211, TRACE, "WPA(2) IE is found\n");

		if (wpa_ie != NULL) {
			/*
			 * WPA IE
			 */
			err = wland_configure_wpaie(ndev, wpa_ie, false);
			if (err < 0)
				goto exit;
		} else {
			/*
			 * RSN IE
			 */
			err = wland_configure_wpaie(ndev,
				(struct wland_vs_tlv *) rsn_ie, true);
			if (err < 0)
				goto exit;
		}
	} else {
		WLAND_DBG(CFG80211, TRACE, "No WPA(2) IEs found\n");
	}

	/*
	 * Set BI and DTIM period
	 */
	if (info->interval) {
		err = wland_fil_set_cmd_data(ifp, WID_BEACON_INTERVAL,
			&info->interval, sizeof(info->interval));
		if (err < 0) {
			WLAND_ERR("Beacon Interval Set Error, %d\n", err);
			goto exit;
		}
	}
	if (info->dtim_period) {
		err = wland_fil_set_cmd_data(ifp, WID_DTIM_PERIOD,
			&info->dtim_period, sizeof(info->dtim_period));
		if (err < 0) {
			WLAND_ERR("DTIM Interval Set Error, %d\n", err);
			goto exit;
		}
	}

exit:
	WLAND_DBG(CFG80211, TRACE, "ADD/SET beacon done(err:%d)\n", err);

	return err;
}
#else /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0) */

#ifdef STRUCT_REF

/**
 * struct cfg80211_beacon_data - beacon data
 * @head: head portion of beacon (before TIM IE)
 *     or %NULL if not changed
 * @tail: tail portion of beacon (after TIM IE)
 *     or %NULL if not changed
 * @head_len: length of @head
 * @tail_len: length of @tail
 * @beacon_ies: extra information element(s) to add into Beacon frames or %NULL
 * @beacon_ies_len: length of beacon_ies in octets
 * @proberesp_ies: extra information element(s) to add into Probe Response
 *	frames or %NULL
 * @proberesp_ies_len: length of proberesp_ies in octets
 * @assocresp_ies: extra information element(s) to add into (Re)Association
 *	Response frames or %NULL
 * @assocresp_ies_len: length of assocresp_ies in octets
 * @probe_resp_len: length of probe response template (@probe_resp)
 * @probe_resp: probe response template (AP mode only)
 */
struct cfg80211_beacon_data {
	const u8 *head, *tail;
	const u8 *beacon_ies;
	const u8 *proberesp_ies;
	const u8 *assocresp_ies;
	const u8 *probe_resp;

	size_t head_len, tail_len;
	size_t beacon_ies_len;
	size_t proberesp_ies_len;
	size_t assocresp_ies_len;
	size_t probe_resp_len;
};

/**
 * struct cfg80211_crypto_settings - Crypto settings
 * @wpa_versions: indicates which, if any, WPA versions are enabled
 *	(from enum nl80211_wpa_versions)
 * @cipher_group: group key cipher suite (or 0 if unset)
 * @n_ciphers_pairwise: number of AP supported unicast ciphers
 * @ciphers_pairwise: unicast key cipher suites
 * @n_akm_suites: number of AKM suites
 * @akm_suites: AKM suites
 * @control_port: Whether user space controls IEEE 802.1X port, i.e.,
 *	sets/clears %NL80211_STA_FLAG_AUTHORIZED. If true, the driver is
 *	required to assume that the port is unauthorized until authorized by
 *	user space. Otherwise, port is marked authorized by default.
 * @control_port_ethertype: the control port protocol that should be
 *	allowed through even on unauthorized ports
 * @control_port_no_encrypt: TRUE to prevent encryption of control port
 *	protocol frames.
 */
struct cfg80211_crypto_settings {
	u32 wpa_versions;
	u32 cipher_group;
	int n_ciphers_pairwise;
	u32 ciphers_pairwise[NL80211_MAX_NR_CIPHER_SUITES];
	int n_akm_suites;
	u32 akm_suites[NL80211_MAX_NR_AKM_SUITES];
	bool control_port;
	__be16 control_port_ethertype;
	bool control_port_no_encrypt;
};

/**
 * enum nl80211_hidden_ssid - values for %NL80211_ATTR_HIDDEN_SSID
 * @NL80211_HIDDEN_SSID_NOT_IN_USE: do not hide SSID (i.e., broadcast it in
 *	Beacon frames)
 * @NL80211_HIDDEN_SSID_ZERO_LEN: hide SSID by using zero-length SSID element
 *	in Beacon frames
 * @NL80211_HIDDEN_SSID_ZERO_CONTENTS: hide SSID by using correct length of SSID
 *	element in Beacon frames but zero out each byte in the SSID
 */
enum nl80211_hidden_ssid {
	NL80211_HIDDEN_SSID_NOT_IN_USE,
	NL80211_HIDDEN_SSID_ZERO_LEN,
	NL80211_HIDDEN_SSID_ZERO_CONTENTS
};

/**
 * enum nl80211_auth_type - AuthenticationType
 *
 * @NL80211_AUTHTYPE_OPEN_SYSTEM: Open System authentication
 * @NL80211_AUTHTYPE_SHARED_KEY: Shared Key authentication (WEP only)
 * @NL80211_AUTHTYPE_FT: Fast BSS Transition (IEEE 802.11r)
 * @NL80211_AUTHTYPE_NETWORK_EAP: Network EAP (some Cisco APs and mainly LEAP)
 * @__NL80211_AUTHTYPE_NUM: internal
 * @NL80211_AUTHTYPE_MAX: maximum valid auth algorithm
 * @NL80211_AUTHTYPE_AUTOMATIC: determine automatically (if necessary by
 *	trying multiple times); this is invalid in netlink -- leave out
 *	the attribute for this on CONNECT commands.
 */
enum nl80211_auth_type {
	NL80211_AUTHTYPE_OPEN_SYSTEM,
	NL80211_AUTHTYPE_SHARED_KEY,
	NL80211_AUTHTYPE_FT,
	NL80211_AUTHTYPE_NETWORK_EAP,

	/*
	 * keep last
	 */
	__NL80211_AUTHTYPE_NUM,
	NL80211_AUTHTYPE_MAX = __NL80211_AUTHTYPE_NUM - 1,
	NL80211_AUTHTYPE_AUTOMATIC
};

/**
 * struct cfg80211_ap_settings - AP configuration
 *
 * Used to configure an AP interface.
 *
 * @beacon: beacon data
 * @beacon_interval: beacon interval
 * @dtim_period: DTIM period
 * @ssid: SSID to be used in the BSS (note: may be %NULL if not provided from
 *	user space)
 * @ssid_len: length of @ssid
 * @hidden_ssid: whether to hide the SSID in Beacon/Probe Response frames
 * @crypto: crypto settings
 * @privacy: the BSS uses privacy
 * @auth_type: Authentication type (algorithm)
 * @inactivity_timeout: time in seconds to determine station's inactivity.
 */
struct cfg80211_ap_settings {
	struct cfg80211_beacon_data beacon;

	int beacon_interval, dtim_period;
	const u8 *ssid;
	size_t ssid_len;
	enum nl80211_hidden_ssid hidden_ssid;
	struct cfg80211_crypto_settings crypto;
	bool privacy;
	enum nl80211_auth_type auth_type;
	int inactivity_timeout;
};
#endif /* STRUCT_REF */

s32 cfg80211_parse_ap_settings(struct net_device *ndev,
	struct cfg80211_ap_settings *settings)
{
	s32 err = 0;
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);

	WLAND_DBG(CFG80211, DEBUG, "Enter\n");
	profile->sec.security = NO_ENCRYPT;
	profile->sec.firmware_autype = ANY;

	if (settings->crypto.cipher_group != NO_ENCRYPT) {
		int i;

		WLAND_DBG(CFG80211, DEBUG, "ENCRYPT\n");

		profile->sec.security = ENCRYPT_ENABLED;

		/*
		 * To determine the u8security value, first we check the group cipher suite then {in case of WPA or WPA2}
		 * we will add to it the pairwise cipher suite(s)
		 */
		if (settings->crypto.wpa_versions & NL80211_WPA_VERSION_2) {
			WLAND_DBG(CFG80211, DEBUG, "WPA_VERSION_2\n");
			profile->sec.firmware_autype = OPEN_SYSTEM;
			profile->sec.security |= WPA2;
		} else if (settings->
			crypto.wpa_versions & NL80211_WPA_VERSION_1) {
			WLAND_DBG(CFG80211, DEBUG, "WPA_VERSION_1\n");

			profile->sec.firmware_autype = OPEN_SYSTEM;
			profile->sec.security |= WPA;
		} else {
			profile->sec.firmware_autype = SHARED_KEY;
			WLAND_DBG(CFG80211, DEBUG, "Default\n");
			goto done;
		}

		for (i = 0; i < settings->crypto.n_ciphers_pairwise; i++) {
			if (settings->crypto.ciphers_pairwise[i] ==
				WLAN_CIPHER_SUITE_TKIP) {
				profile->sec.security |= TKIP;
				WLAND_DBG(CFG80211, DEBUG, "TKIP\n");
			} else if (settings->crypto.ciphers_pairwise[i] ==
				WLAN_CIPHER_SUITE_CCMP) {
				profile->sec.security |= AES;
				WLAND_DBG(CFG80211, DEBUG, "AES\n");
			}
		}
	} else {
		WLAND_DBG(CFG80211, DEBUG, "NO_ENCRYPT\n");
		profile->sec.firmware_autype = OPEN_SYSTEM;
	}

done:
	WLAND_DBG(CFG80211, DEBUG, "Done(err:%d)\n", err);

	return err;
}

static s32 cfg80211_start_ap(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_ap_settings *settings)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	struct wland_tlv *ssid_ie;
	struct wland_tlv *rsn_ie;
	struct wland_vs_tlv *wpa_ie;
	enum nl80211_iftype dev_role;
	s32 ie_offset, err = -EPERM;

	WLAND_DBG(CFG80211, DEBUG, "ssid=%s(%zu), "
		"auth_type=%d, "
		"inactivity_timeout=%d, "
		"beacon_interval=%d, "
		"dtim_period=%d, "
		"hidden_ssid=%d, "
		"auth_type=%d "
		"crypto->wpa_versions=%d\n",
		settings->ssid,
		settings->ssid_len,
		settings->auth_type,
		settings->inactivity_timeout,
		settings->beacon_interval,
		settings->dtim_period,
		settings->hidden_ssid,
		settings->auth_type, settings->crypto.wpa_versions);

	/*
	 * Show AP settings
	 */
	WLAND_DBG(CFG80211, DEBUG,
		"\n"
		"BeaconIEs  (%p) BeaconIEsLen  (%u)\n"
		"ProbeRspIEs(%p) ProbeRspIEsLen(%u)\n"
		"AssocRspIEs(%p) AssocRspIEs   (%u)\n"
		"ProbeResp  (%p) ProbeResp     (%u)\n"
		"Head       (%p) HeadLen       (%u)\n"
		"Tail       (%p) TailLen       (%u)\n"
		"BeaconInterval: %dTU\n"
		"DTIM Period   : %d\n"
		"SSID          : '%s' Len(%uB)\n"
		"Hidden SSID   : %u\n"
		"Crypto:\n"
		"  wpa_versions = %u\n"
		"  cipher_group = %02X-%02X-%02X-%02X\n"
		"  n_ciphers_pairwise = %d %02X-%02X-%02X-%02X\n"
		"  n_akm_suites = %d %02X-%02X-%02X-%02X\n"
		"Privacy       : %u\n"
		"Auth Type     : %u\n"
		"InactiveTime  : %d\n",
		settings->beacon.beacon_ies, settings->beacon.beacon_ies_len,
		settings->beacon.proberesp_ies,
		settings->beacon.proberesp_ies_len,
		settings->beacon.assocresp_ies,
		settings->beacon.assocresp_ies_len, settings->beacon.probe_resp,
		settings->beacon.probe_resp_len, settings->beacon.head,
		settings->beacon.head_len, settings->beacon.tail,
		settings->beacon.tail_len, settings->beacon_interval,
		settings->dtim_period, (const char *) settings->ssid,
		settings->ssid_len, settings->hidden_ssid,
		settings->crypto.wpa_versions,
		(settings->crypto.cipher_group >> 24) & 0xFF,
		(settings->crypto.cipher_group >> 16) & 0xFF,
		(settings->crypto.cipher_group >> 8) & 0xFF,
		settings->crypto.cipher_group & 0xFF,
		settings->crypto.n_ciphers_pairwise,
		(settings->crypto.ciphers_pairwise[0] >> 24) & 0xFF,
		(settings->crypto.ciphers_pairwise[0] >> 16) & 0xFF,
		(settings->crypto.ciphers_pairwise[0] >> 8) & 0xFF,
		settings->crypto.ciphers_pairwise[0] & 0xFF,
		settings->crypto.n_akm_suites,
		(settings->crypto.akm_suites[0] >> 24) & 0xFF,
		(settings->crypto.akm_suites[0] >> 16) & 0xFF,
		(settings->crypto.akm_suites[0] >> 8) & 0xFF,
		settings->crypto.akm_suites[0] & 0xFF, settings->privacy,
		settings->auth_type, settings->inactivity_timeout);

	if (settings->beacon.beacon_ies && settings->beacon.beacon_ies_len) {
		WLAND_DUMP(RX_NETEVENT,
			settings->beacon.beacon_ies,
			settings->beacon.beacon_ies_len,
			"Beacon IEs(len %u):\n",
			settings->beacon.beacon_ies_len);
	}

	if (settings->beacon.proberesp_ies
		&& settings->beacon.proberesp_ies_len) {
		WLAND_DUMP(RX_NETEVENT, settings->beacon.proberesp_ies,
			settings->beacon.proberesp_ies_len,
			"Probe Response IEs(len %u):\n",
			settings->beacon.proberesp_ies_len);
	}

	if (settings->beacon.assocresp_ies
		&& settings->beacon.assocresp_ies_len) {
		WLAND_DUMP(RX_NETEVENT, settings->beacon.assocresp_ies,
			settings->beacon.assocresp_ies_len,
			"Associate Response IEs(len %u):\n",
			settings->beacon.assocresp_ies_len);
	}

	if (settings->beacon.probe_resp && settings->beacon.probe_resp_len) {
		WLAND_DUMP(RX_NETEVENT,
			settings->beacon.probe_resp,
			settings->beacon.probe_resp_len,
			"Probe Response (len %u):\n",
			settings->beacon.probe_resp_len);
	}

	if (settings->beacon.head && settings->beacon.head_len) {
		WLAND_DUMP(RX_NETEVENT,
			settings->beacon.head, settings->beacon.head_len,
			"Head (len %u):\n", settings->beacon.head_len);
	}

	if (settings->beacon.tail && settings->beacon.tail_len) {
		WLAND_DUMP(RX_NETEVENT,
			settings->beacon.tail, settings->beacon.tail_len,
			"Tail (len %u):\n", settings->beacon.tail_len);
	}

	dev_role = ifp->vif->wdev.iftype;

	memset(&profile->ssid, '\0', sizeof(struct wland_ssid));

	//SSID and SSID_len
	if (settings->ssid == NULL || settings->ssid_len == 0) {
		ie_offset = DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
		ssid_ie =
			wland_parse_tlvs((u8 *) & settings->
			beacon.head[ie_offset],
			settings->beacon.head_len - ie_offset, WLAN_EID_SSID);
		if (!ssid_ie)
			return -EINVAL;

		memcpy(profile->ssid.SSID, ssid_ie->data, ssid_ie->len);
		profile->ssid.SSID_len = ssid_ie->len;

	} else {
		memcpy(profile->ssid.SSID, settings->ssid, settings->ssid_len);
		profile->ssid.SSID_len = settings->ssid_len;
	}

	//beacon_interval and dtim_period
	/*
	 * store beacon interval and dtim period
	 */
	profile->beacon = settings->beacon_interval;
	profile->dtim = settings->dtim_period;

	WLAND_DBG(CFG80211, TRACE, "SSID is (%s,SSIDLEN:%d) in Head\n",
		profile->ssid.SSID, profile->ssid.SSID_len);

	/*
	 * find the RSN_IE
	 */
	rsn_ie = wland_parse_tlvs((u8 *) settings->beacon.tail,
		settings->beacon.tail_len, WLAN_EID_RSN);

	/*
	 * find the WPA_IE
	 */
	wpa_ie = wland_find_wpaie((u8 *) settings->beacon.tail,
		settings->beacon.tail_len);

	if ((wpa_ie != NULL || rsn_ie != NULL)) {
		WLAND_DBG(CFG80211, TRACE, "WPA(2) IE is found\n");

		if (wpa_ie != NULL) {
			/*
			 * WPA IE
			 */
			WLAND_DBG(CFG80211, TRACE, "WPA IE\n");
			err = wland_configure_wpaie(ndev, wpa_ie, false);
			if (err < 0)
				goto exit;
		} else {
			/*
			 * RSN IE
			 */
			WLAND_DBG(CFG80211, TRACE, "RSN IE\n");
			err = wland_configure_wpaie(ndev,
				(struct wland_vs_tlv *) rsn_ie, true);
			if (err < 0)
				goto exit;
		}
	} else {
		WLAND_DBG(CFG80211, TRACE, "No WPA(2) IEs found\n");
	}
#if 0
	/*
	 * Set Beacon IEs to FW
	 */
	err = wland_vif_set_mgmt_ie(ifp->vif, WLAND_VNDR_IE_BEACON_FLAG,
		settings->beacon.tail, settings->beacon.tail_len);
	if (err) {
		WLAND_ERR("Set Beacon IE Failed\n");
		return err;
	}

	WLAND_DBG(CFG80211, TRACE, "Applied Vndr IEs for Beacon\n");

	/*
	 * Set Probe Response IEs to FW
	 */
	err = wland_vif_set_mgmt_ie(ifp->vif, WLAND_VNDR_IE_PRBRSP_FLAG,
		settings->beacon.proberesp_ies,
		settings->beacon.proberesp_ies_len);
	if (err)
		WLAND_ERR("Set Probe Resp IE Failed\n");
	else
		WLAND_DBG(CFG80211, TRACE, "Applied Vndr IEs for Probe Resp\n");
#endif

	if (cfg80211_parse_ap_settings(ndev, settings)) {
		WLAND_ERR("cfg80211_parse_ap_settings error!\n");
		goto exit;
	}

	if (dev_role == NL80211_IFTYPE_AP) {
		err = wland_start_ap_set(ifp, profile, false);

		WLAND_DBG(CFG80211, TRACE, "AP mode configuration complete\n");
	} else {
		err = wland_start_ap_set(ifp, profile, true);

		WLAND_DBG(CFG80211, TRACE, "GO mode configuration complete\n");
	}

	clear_bit(VIF_STATUS_AP_CREATING, &ifp->vif->sme_state);
	set_bit(VIF_STATUS_AP_CREATED, &ifp->vif->sme_state);

exit:
	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return err;
}

static int cfg80211_stop_ap(struct wiphy *wiphy, struct net_device *ndev)
{
	struct wland_if *ifp = netdev_priv(ndev);
	s32 err = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (ifp->vif->wdev.iftype == NL80211_IFTYPE_AP) {
		struct wland_join_params join_params;

		/*
		 * Due to most likely deauths outstanding we sleep first to make sure they get processed by fw.
		 */
		memset(&join_params, 0, sizeof(join_params));

		err = wland_fil_iovar_data_set(ifp, "set_ssid", &join_params,
			sizeof(join_params));
		if (err < 0)
			WLAND_ERR("SET SSID error:%d\n", err);
	} else {
		struct wland_fil_bss_enable_le bss_enable;

		bss_enable.bsscfg_idx = cpu_to_le32(ifp->bssidx);
		bss_enable.enable = cpu_to_le32(0);

		err = wland_fil_iovar_data_set(ifp, "bss", &bss_enable,
			sizeof(bss_enable));
		if (err < 0)
			WLAND_ERR("bss_enable config failed:%d\n", err);
	}

	set_bit(VIF_STATUS_AP_CREATING, &ifp->vif->sme_state);
	clear_bit(VIF_STATUS_AP_CREATED, &ifp->vif->sme_state);

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);
	return err;
}

static s32 cfg80211_change_beacon(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_beacon_data *info)
{
	s32 err = 0;
	struct wland_if *ifp = netdev_priv(ndev);

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	/*
	 * Set Beacon IEs to FW
	 */
	err = wland_vif_set_mgmt_ie(ifp->vif, WLAND_VNDR_IE_BEACON_FLAG,
		info->tail, info->tail_len);
	if (err) {
		WLAND_ERR("Set Beacon IE Failed\n");
		return err;
	}

	WLAND_DBG(CFG80211, TRACE, "Applied Vndr IEs for Beacon\n");

	/*
	 * Set Probe Response IEs to FW
	 */
	err = wland_vif_set_mgmt_ie(ifp->vif, WLAND_VNDR_IE_PRBRSP_FLAG,
		info->proberesp_ies, info->proberesp_ies_len);
	if (err)
		WLAND_ERR("Set Probe Resp IE Failed\n");
	else
		WLAND_DBG(CFG80211, TRACE, "Applied Vndr IEs for Probe Resp\n");

	return err;
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
static int cfg80211_del_station(struct wiphy *wiphy, struct net_device *ndev,
	u8 * mac)
{
#ifdef WLAND_P2P_SUPPORT
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
#endif /* WLAND_P2P_SUPPORT */
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_scb_val_le scbval;
	s32 err = 0;

	if (!mac) {
		WLAND_ERR("mac addr is NULL ignore it\n");
		return err;
	}

	WLAND_DBG(CFG80211, TRACE, "Enter(%pM)\n", mac);

#ifdef WLAND_P2P_SUPPORT
	if (ifp->vif == cfg->p2p.bss_idx[P2PAPI_BSSCFG_DEVICE].vif)
		ifp = cfg->p2p.bss_idx[P2PAPI_BSSCFG_PRIMARY].vif->ifp;
#endif /* WLAND_P2P_SUPPORT */

	if (!check_vif_up(ifp->vif))
		return -EIO;

	memcpy(&scbval.ea, mac, ETH_ALEN);

	scbval.val = cpu_to_le32(WLAN_REASON_DEAUTH_LEAVING);

	err = wland_fil_iovar_data_set(ifp, "scb_default_for_reason", &scbval,
		sizeof(scbval));

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return err;
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0) */

static void cfg80211_mgmt_frame_register(struct wiphy *wiphy,
#if  LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct wireless_dev *wdev,
#else				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct net_device *dev,
#endif				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	u16 frame_type, bool reg)
{
#if    LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct wland_cfg80211_vif *vif =
		container_of(wdev, struct wland_cfg80211_vif, wdev);
#else /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct wland_cfg80211_vif *vif = ndev_to_vif(dev);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	u16 mgmt_type = (frame_type & IEEE80211_FCTL_STYPE) >> 4;

	WLAND_DBG(CFG80211, TRACE,
		"(frame_type: 0x%04x,reg:%x,mgmt_type:0x%x),Enter\n",
		frame_type, reg, mgmt_type);

	if (reg)
		vif->mgmt_rx_reg |= BIT(mgmt_type);
	else
		vif->mgmt_rx_reg &= ~BIT(mgmt_type);
}

static s32 cfg80211_change_bss(struct wiphy *wiphy, struct net_device *dev,
	struct bss_parameters *params)
{
	WLAND_DBG(CFG80211, DEBUG,
		"Enter, change_bss cts:%x,preamble:%x,short_slot_time:%x.\n",
		params->use_cts_prot, params->use_short_preamble,
		params->use_short_slot_time);

	return 0;
}

static s32 cfg80211_mgmt_tx(struct wiphy *wiphy,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct wireless_dev *wdev,
#else				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct net_device *ndev,
#endif				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct ieee80211_channel *channel, bool offchan,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
	enum nl80211_channel_type channel_type, bool channel_type_vaild,
#endif
	unsigned int wait, const u8 * buf, size_t len,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	bool no_cck,
#endif				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	bool dont_wait_for_ack,
#endif				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0) */
	u64 * cookie)
{
#ifdef WLAND_P2P_SUPPORT
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
#endif /* WLAND_P2P_SUPPORT */
	const struct ieee80211_mgmt *mgmt = (const struct ieee80211_mgmt *) buf;
	struct wland_fil_action_frame_le *action_frame;
	struct wland_fil_af_params_le *af_params;
	bool ack;
	s32 chan_nr, err = 0, ie_offset, ie_len;
	u32 freq;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct wland_cfg80211_vif *vif =
		container_of(wdev, struct wland_cfg80211_vif, wdev);
#else /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct wland_cfg80211_vif *vif = ndev_to_vif(ndev);;
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	*cookie = 0;

	if (!ieee80211_is_mgmt(mgmt->frame_control)) {
		WLAND_ERR("Driver only allows MGMT packet type\n");
		return -EPERM;
	}

	if (ieee80211_is_probe_resp(mgmt->frame_control)) {
		/*
		 * Right now the only reason to get a probe response
		 * is for p2p listen response or for p2p GO from
		 * wpa_supplicant. Unfortunately the probe is send
		 * on primary ndev, while dongle wants it on the p2p
		 * vif. Since this is only reason for a probe
		 * response to be sent, the vif is taken from cfg.
		 * If ever desired to send proberesp for non p2p
		 * response then data should be checked for
		 * "DIRECT-". Note in future supplicant will take
		 * dedicated p2p wdev to do this and then this 'hack'
		 * is not needed anymore.
		 */
		ie_offset = DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
		ie_len = len - ie_offset;

#ifdef WLAND_P2P_SUPPORT
		if (vif == cfg->p2p.bss_idx[P2PAPI_BSSCFG_PRIMARY].vif)
			vif = cfg->p2p.bss_idx[P2PAPI_BSSCFG_DEVICE].vif;
#endif /* WLAND_P2P_SUPPORT */
		err = wland_vif_set_mgmt_ie(vif, WLAND_VNDR_IE_PRBRSP_FLAG,
			&buf[ie_offset], ie_len);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
		cfg80211_mgmt_tx_status(wdev, *cookie, buf, len, true,
			GFP_KERNEL);
#else /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
		cfg80211_mgmt_tx_status(ndev, *cookie, buf, len, true,
			GFP_KERNEL);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	} else if (ieee80211_is_action(mgmt->frame_control)) {
		af_params = kzalloc(sizeof(*af_params), GFP_KERNEL);
		if (af_params == NULL) {
			WLAND_ERR("unable to allocate frame\n");
			err = -ENOMEM;
			goto exit;
		}
		action_frame = &af_params->action_frame;
		/*
		 * Add the packet Id
		 */
		action_frame->packet_id = cpu_to_le32(*cookie);
		/*
		 * Add BSSID
		 */
		memcpy(&action_frame->da[0], &mgmt->da[0], ETH_ALEN);
		memcpy(&af_params->bssid[0], &mgmt->bssid[0], ETH_ALEN);
		/*
		 * Add the length exepted for 802.11 header
		 */
		action_frame->len = cpu_to_le16(len - DOT11_MGMT_HDR_LEN);
		/*
		 * Add the channel. Use the one specified as parameter if any or
		 * * the current one (got from the firmware) otherwise
		 */
		if (channel)
			freq = channel->center_freq;
		else
			wland_fil_iovar_data_get(vif->ifp, "get_channel", &freq,
				sizeof(freq));

		chan_nr = ieee80211_frequency_to_channel(freq);
		af_params->channel = cpu_to_le32(chan_nr);

		memcpy(action_frame->data, &buf[DOT11_MGMT_HDR_LEN],
			le16_to_cpu(action_frame->len));

		WLAND_DBG(CFG80211, TRACE,
			"Action frame, cookie=%lld, len=%d, freq=%d\n", *cookie,
			le16_to_cpu(action_frame->len), freq);

#ifdef WLAND_P2P_SUPPORT
		ack = wland_p2p_send_action_frame(cfg, cfg_to_ndev(cfg),
			af_params);
#else /* WLAND_P2P_SUPPORT */
		ack = false;
#endif /* WLAND_P2P_SUPPORT */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
		cfg80211_mgmt_tx_status(wdev, *cookie, buf, len, ack,
			GFP_KERNEL);
#else /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
		cfg80211_mgmt_tx_status(ndev, *cookie, buf, len, ack,
			GFP_KERNEL);
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
		kfree(af_params);
	} else {
		WLAND_DBG(CFG80211, TRACE, "Unhandled, fc=%04x\n",
			mgmt->frame_control);
		//WLAND_DUMP(CFG80211, buf, len, "payload, len=%Zu\n", len);
	}

exit:
	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return err;
}

#if 0
static s32 cfg80211_remain_on_channel(struct wiphy *wiphy,
#if    LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct wireless_dev *wdev,
#else				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct net_device *dev,
#endif				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct ieee80211_channel *channel, unsigned int duration, u64 * cookie)
{
	WLAND_DBG(CFG80211, TRACE, "Enter p2p remain_on_channel\n");

#ifdef WLAND_P2P_SUPPORT
	return cfg80211_p2p_remain_on_channel(wiphy, dev, channel, channel_type,
		duration, cookie);
#else /*WLAND_P2P_SUPPORT */
	return -EOPNOTSUPP;
#endif /*WLAND_P2P_SUPPORT */
}
#endif
static s32 cfg80211_cancel_remain_on_channel(struct wiphy *wiphy,
#if    LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct wireless_dev *wdev,
#else				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct net_device *dev,
#endif				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	u64 cookie)
{
	s32 err = 0;

#ifdef WLAND_P2P_SUPPORT
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);
	struct wland_cfg80211_vif *vif =
		cfg->p2p.bss_idx[P2PAPI_BSSCFG_DEVICE].vif;
#else /* WLAND_P2P_SUPPORT */
	struct wland_cfg80211_vif *vif = NULL;
#endif /* WLAND_P2P_SUPPORT */

	WLAND_DBG(CFG80211, TRACE, "Enter p2p listen cancel\n");

	if (vif == NULL) {
		WLAND_ERR("No p2p device available for probe response\n");
		err = -ENODEV;
		goto exit;
	}
#ifdef WLAND_P2P_SUPPORT
	wland_p2p_cancel_remain_on_channel(vif->ifp);
#endif /*WLAND_P2P_SUPPORT */
exit:
	WLAND_DBG(CFG80211, TRACE, "Enter p2p listen cancel(err:%d)\n", err);

	return err;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
static int cfg80211_crit_proto_start(struct wiphy *wiphy,
	struct wireless_dev *wdev, enum nl80211_crit_proto_id proto,
	u16 duration)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);

#ifdef WLAND_BTCOEX_SUPPORT
	struct wland_cfg80211_vif *vif =
		container_of(wdev, struct wland_cfg80211_vif, wdev);
#endif /* WLAND_BTCOEX_SUPPORT */

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	/*
	 * only DHCP support for now
	 */
	if (proto != NL80211_CRIT_PROTO_DHCP)
		return -EINVAL;

	/*
	 * suppress and abort scanning
	 */
	set_bit(SCAN_STATUS_SUPPRESS, &cfg->scan_status);

	wland_abort_scanning(cfg);

#ifdef WLAND_BTCOEX_SUPPORT
	return wland_btcoex_set_mode(vif, BTCOEX_DISABLED, duration);
#else /* WLAND_BTCOEX_SUPPORT */
	return 0;
#endif /* WLAND_BTCOEX_SUPPORT */
}

static void cfg80211_crit_proto_stop(struct wiphy *wiphy,
	struct wireless_dev *wdev)
{
	struct wland_cfg80211_info *cfg = wiphy_to_cfg(wiphy);

#ifdef WLAND_BTCOEX_SUPPORT
	struct wland_cfg80211_vif *vif =
		container_of(wdev, struct wland_cfg80211_vif, wdev);
#endif /* WLAND_BTCOEX_SUPPORT */

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

#ifdef WLAND_BTCOEX_SUPPORT
	wland_btcoex_set_mode(vif, BTCOEX_ENABLED, 0);
#endif /* WLAND_BTCOEX_SUPPORT */

	clear_bit(SCAN_STATUS_SUPPRESS, &cfg->scan_status);
} static int cfg80211_tdls_oper(struct wiphy *wiphy, struct net_device *ndev,
	u8 * peer, enum nl80211_tdls_operation oper)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_tdls_iovar_le info;
	int ret = 0;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	switch (oper) {
	case NL80211_TDLS_DISCOVERY_REQ:
		ret = TDLS_MANUAL_EP_DISCOVERY;
		break;
	case NL80211_TDLS_SETUP:
		ret = TDLS_MANUAL_EP_CREATE;
		break;
	case NL80211_TDLS_TEARDOWN:
		ret = TDLS_MANUAL_EP_DELETE;
		break;
	default:
		WLAND_ERR("unsupported operation: %d\n", oper);
		ret = -EOPNOTSUPP;
		break;
	}
	if (ret < 0)
		return ret;

	memset(&info, 0, sizeof(info));

	info.mode = (u8) ret;

	if (peer)
		memcpy(info.ea, peer, ETH_ALEN);

	ret = wland_fil_iovar_data_set(ifp, "tdls_endpoint", &info,
		sizeof(info));

	WLAND_DBG(CFG80211, TRACE, "Done(ret:%d)\n", ret);

	return ret;
}
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
static s32 cfg80211_mgmt_tx_cancel_wait(struct wiphy *wiphy,
	struct wireless_dev *wdev, u64 cookie)
{
	/*
	 * CFG80211 checks for tx_cancel_wait callback when ATTR_DURATION is passed with CMD_FRAME.
	 * * This callback is supposed to cancel the OFFCHANNEL Wait.
	 * * Since we are already taking care of that with the tx_mgmt logic, do nothing here.
	 */
	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	return 0;
}
#else
static s32 cfg80211_mgmt_tx_cancel_wait(struct wiphy *wiphy,
	struct net_device *ndev, u64 cookie)
{
	/*
	 * CFG80211 checks for tx_cancel_wait callback when ATTR_DURATION is passed with CMD_FRAME.
	 * * This callback is supposed to cancel the OFFCHANNEL Wait.
	 * * Since we are already taking care of that with the tx_mgmt logic, do nothing here.
	 */
	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	return 0;
}
#endif
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0) */
#if 0
static s32 cfg80211_set_channel(struct wiphy *wiphy,
	struct net_device *dev,
	struct ieee80211_channel *chan, enum nl80211_channel_type channel_type)
{
	struct wland_if *ifp = netdev_priv(dev);

	s32 err = -EOPNOTSUPP, _chan;

	if (!check_vif_up(ifp->vif))
		return -EIO;

	_chan = ieee80211_frequency_to_channel(chan->center_freq);

	WLAND_DBG(CFG80211, TRACE, "Enter(chan:%d)\n", _chan);

#ifdef WLAND_5GRF_SUPPORT
	if (chan->band == IEEE80211_BAND_5GHZ) {
		WLAND_DBG(CFG80211, TRACE, "(BAND_5GHZ---->chan:%d),Enter\n",
			_chan);
	}
#endif /* WLAND_5GRF_SUPPORT */
	if (chan->band == IEEE80211_BAND_2GHZ) {
		WLAND_DBG(CFG80211, TRACE, "(BAND_2.4GHZ---->chan:%d),Enter\n",
			_chan);
	}

	err = wland_fil_set_cmd_data(ifp, WID_CURRENT_CHANNEL, &_chan,
		sizeof(_chan));

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return err;
}
#endif

/*
*  @brief 	    cfg80211_set_cqm_rssi_config
*  @details 	Configure connection quality monitor RSSI threshold.
*  @param[in]   struct wiphy *wiphy:
*  @param[in]	struct net_device *dev:
*  @param[in]  	s32 rssi_thold:
*  @param[in]	u32 rssi_hyst:
*  @return 	int : Return 0 on Success
*/
static s32 cfg80211_set_cqm_rssi_config(struct wiphy *wiphy,
	struct net_device *dev, s32 rssi_thold, u32 rssi_hyst)
{
	WLAND_DBG(CFG80211, TRACE, "Setting CQM RSSI Function\n");
	return 0;
}

static struct cfg80211_ops wl_cfg80211_ops = {
	.add_virtual_intf = cfg80211_add_virtual_iface,
	.del_virtual_intf = cfg80211_del_virtual_iface,
	.change_virtual_intf = cfg80211_change_virtual_iface,
	.scan =	cfg80211_scan,
	.set_wiphy_params = cfg80211_set_wiphy_params,
	.join_ibss = cfg80211_join_ibss,
	.leave_ibss = cfg80211_leave_ibss,
	.get_station = cfg80211_get_station,
	.set_tx_power = cfg80211_set_tx_power,
	.get_tx_power = cfg80211_get_tx_power,
	.add_key = cfg80211_add_key,
	.del_key = cfg80211_del_key,
	.get_key = cfg80211_get_key,
	.set_default_key = cfg80211_config_default_key,
	.set_default_mgmt_key =	cfg80211_config_default_mgmt_key,
	.set_power_mgmt = cfg80211_set_power_mgmt,
	.connect = cfg80211_connect,
	.disconnect = cfg80211_disconnect,
	.suspend = cfg80211_suspend,
	.resume = cfg80211_resume,
	.set_pmksa = cfg80211_set_pmksa,
	.del_pmksa = cfg80211_del_pmksa,
	.flush_pmksa = cfg80211_flush_pmksa,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
	.set_beacon = cfg80211_modify_beacon,
	.add_beacon = cfg80211_modify_beacon,
	.del_beacon = cfg80211_modify_beacon,
#else /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0) */
	.start_ap = cfg80211_start_ap,
	.stop_ap = cfg80211_stop_ap,
	.change_beacon = cfg80211_change_beacon,
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	.del_station = cfg80211_del_station,
	.mgmt_tx_cancel_wait = cfg80211_mgmt_tx_cancel_wait,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0) */
	.sched_scan_start = cfg80211_sched_scan_start,
	.sched_scan_stop = cfg80211_sched_scan_stop,
	.mgmt_frame_register = cfg80211_mgmt_frame_register,
	.change_bss = cfg80211_change_bss,
	.mgmt_tx = cfg80211_mgmt_tx,
#if 0
	.set_channel = cfg80211_set_channel,
	.remain_on_channel = cfg80211_remain_on_channel,
#endif
	.cancel_remain_on_channel = cfg80211_cancel_remain_on_channel,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
#ifdef WLAND_P2P_SUPPORT
	.start_p2p_device = cfg80211_p2p_start_device,
	.stop_p2p_device = cfg80211_p2p_stop_device,
#endif /* WLAND_P2P_SUPPORT */
	.crit_proto_start = cfg80211_crit_proto_start,
	.crit_proto_stop = cfg80211_crit_proto_stop,
	.tdls_oper = cfg80211_tdls_oper,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	.set_cqm_rssi_config = cfg80211_set_cqm_rssi_config,
};

static s32 nl80211_iftype_to_mode(enum nl80211_iftype type)
{
	switch (type) {
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_MESH_POINT:
		return -ENOTSUPP;
	case NL80211_IFTYPE_ADHOC:
		return WL_MODE_IBSS;
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
		return WL_MODE_BSS;
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
		return WL_MODE_AP;
	case NL80211_IFTYPE_UNSPECIFIED:
	default:
		break;
	}
	return -EINVAL;
}

#ifdef WLAND_TBD_SUPPORT
static const struct ieee80211_iface_limit wland_iface_limits[] = {
	{
		.max = 2,.types = BIT(NL80211_IFTYPE_STATION) |
		BIT(NL80211_IFTYPE_ADHOC) | BIT(NL80211_IFTYPE_AP)
	},
	{
		.max = 1,.types = BIT(NL80211_IFTYPE_P2P_CLIENT) |
		BIT(NL80211_IFTYPE_P2P_GO)
	},
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	{
		.max = 1,.types = BIT(NL80211_IFTYPE_P2P_DEVICE)
	}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
};

static const struct ieee80211_iface_combination wland_iface_combos[] = {
	{
		.max_interfaces =
			WLAND_IFACE_MAX_CNT,.num_different_channels =
			2,.n_limits =
			ARRAY_SIZE(wland_iface_limits),.limits =
			wland_iface_limits}
};
#endif /* WLAND_TBD_SUPPORT */

/* There isn't a lot of sense in it, but you can transmit anything you like */
static const struct ieee80211_txrx_stypes
	cfg80211_default_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_ADHOC] = {
			.tx = 0xFFFF,.rx = BIT(IEEE80211_STYPE_ACTION >> 4)
		},[NL80211_IFTYPE_STATION] = {
			.tx = 0xFFFF,.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
		},[NL80211_IFTYPE_AP] = {
			.tx = 0xFFFF,.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			BIT(IEEE80211_STYPE_DISASSOC >> 4) |
			BIT(IEEE80211_STYPE_AUTH >> 4) |
			BIT(IEEE80211_STYPE_DEAUTH >> 4) |
			BIT(IEEE80211_STYPE_ACTION >> 4)
		},[NL80211_IFTYPE_AP_VLAN] = {
			/*
			 * copy AP
			 */
			.tx = 0xFFFF,.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			BIT(IEEE80211_STYPE_DISASSOC >> 4) |
			BIT(IEEE80211_STYPE_AUTH >> 4) |
			BIT(IEEE80211_STYPE_DEAUTH >> 4) |
			BIT(IEEE80211_STYPE_ACTION >> 4)
		},[NL80211_IFTYPE_P2P_CLIENT] = {
			.tx = 0xFFFF,.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
		},[NL80211_IFTYPE_P2P_GO] = {
			.tx = 0xFFFF,.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
			BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
			BIT(IEEE80211_STYPE_DISASSOC >> 4) |
			BIT(IEEE80211_STYPE_AUTH >> 4) |
			BIT(IEEE80211_STYPE_DEAUTH >> 4) |
			BIT(IEEE80211_STYPE_ACTION >> 4)
		}
};

void cfg80211_reg_notifier(struct wiphy *wiphy,
	struct regulatory_request *request)
{
	struct wland_cfg80211_info *cfg =
		(struct wland_cfg80211_info *) wiphy_priv(wiphy);
	struct wland_country cspec = { {
		0}, 0, {
		0}
	};

	if (!request || !cfg) {
		WLAND_ERR("Invalid arg\n");
		return;
	}

	WLAND_DBG(CFG80211, TRACE, "ccode: %c%c Initiator: %d\n",
		request->alpha2[0], request->alpha2[1], request->initiator);

	/*
	 * We support only REGDOM_SET_BY_USER as of now
	 */
	if (request->initiator != NL80211_REGDOM_SET_BY_USER) {
		WLAND_ERR("reg_notifier for intiator:%d not supported \n",
			request->initiator);
		return;
	}

	if (request->alpha2[0] == '0' && request->alpha2[1] == '0') {
		/*
		 * world domain
		 */
		WLAND_ERR("World domain. Setting XY/4 \n");
		strncpy(cspec.country_abbrev, "XY", strlen("XY"));
		cspec.rev = 4;
	} else {
		memcpy(cspec.country_abbrev, request->alpha2, 2);
		cspec.country_abbrev[3] = '\0';
		cspec.rev = -1;	/* Unspecified */
	}

	WLAND_DBG(CFG80211, TRACE, "set country '%s/%d' done\n",
		cspec.country_abbrev, cspec.rev);
	return;
}

static struct wiphy *wland_setup_wiphy(struct device *phydev)
{
	s32 err = 0;
	struct wiphy *wiphy = NULL;

	WLAND_DBG(CFG80211, TRACE, "Enter.\n sizeof_priv = %d\n",
		sizeof(struct wland_cfg80211_info));
	wiphy = wiphy_new(&wl_cfg80211_ops, sizeof(struct wland_cfg80211_info));
	if (!wiphy) {
		WLAND_ERR("Could not allocate wiphy device\n");
		return ERR_PTR(-ENOMEM);
	}

	set_wiphy_dev(wiphy, phydev);

	//maximum number of SSIDs the device can scan for in any given scan
	wiphy->max_scan_ssids = SCAN_NUM_MAX;

	//maximum length of user-controlled IEs device can add to probe request frames transmitted
	//during a scan, must not include fixed IEs like supported rates
	wiphy->max_scan_ie_len = SCAN_IE_LEN_MAX;

	//maximum number of PMKIDs supported by device
	wiphy->max_num_pmkids = MAXPMKID;

#ifdef WLAND_SCHED_SCAN_SUPPORT
	/*
	 * scheduled scan settings
	 */
	wiphy->max_sched_scan_ssids = WLAND_PNO_MAX_PFN_COUNT;
	wiphy->max_match_sets = WLAND_PNO_MAX_PFN_COUNT;
	wiphy->max_sched_scan_ie_len = SCAN_IE_LEN_MAX;
	wiphy->flags |= WIPHY_FLAG_SUPPORTS_SCHED_SCAN;
#endif /* WLAND_SCHED_SCAN_SUPPORT */

	//bitmask of interfaces types valid for this wiphy, must be set by driver
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION)
#ifdef WLAND_MONITOR_SUPPORT
		| BIT(NL80211_IFTYPE_MONITOR)
#endif /* WLAND_MONITOR_SUPPORT */
		| BIT(NL80211_IFTYPE_AP)
#ifdef WLAND_TBD_SUPPORT
		| BIT(NL80211_IFTYPE_ADHOC)
#endif /* WLAND_TBD_SUPPORT */
#ifdef WLAND_P2P_SUPPORT
		| BIT(NL80211_IFTYPE_P2P_CLIENT)
		| BIT(NL80211_IFTYPE_P2P_GO)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
		| BIT(NL80211_IFTYPE_P2P_DEVICE)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
#endif /* WLAND_P2P_SUPPORT */
		;
#ifdef WLAND_TBD_SUPPORT
	//Valid interface combinations array, should not list single interface types.
	wiphy->iface_combinations = wland_iface_combos;
	wiphy->n_iface_combinations = ARRAY_SIZE(wland_iface_combos);
#endif /* WLAND_TBD_SUPPORT */

	//information about bands/channels supported by this device
	wiphy->bands[IEEE80211_BAND_2GHZ] = &__wl_band_2ghz;

	/*
	 * signal type reported in &struct cfg80211_bss.
	 * *@CFG80211_SIGNAL_TYPE_NONE: no signal strength information available
	 * *@CFG80211_SIGNAL_TYPE_MBM: signal strength in mBm (100*dBm)
	 * *@CFG80211_SIGNAL_TYPE_UNSPEC: signal strength, increasing from 0 through 100
	 */
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	//supported cipher suites
	wiphy->cipher_suites = __wl_cipher_suites;
	wiphy->n_cipher_suites = ARRAY_SIZE(__wl_cipher_suites);

	//Maximum time a remain-on-channel operation may request, if implemented.
	wiphy->max_remain_on_channel_duration = 5000;

	//bitmasks of frame subtypes that can be subscribed to or transmitted through nl80211,
	//points to an array indexed by interface type
	wiphy->mgmt_stypes = cfg80211_default_mgmt_stypes;

	/*
	 * initial flags
	 */
	//if set to true, powersave will be enabled.
#ifdef WLAND_POWER_MANAGER
	wiphy->flags |= WIPHY_FLAG_PS_ON_BY_DEFAULT;
#else /*WLAND_POWER_MANAGER */
	wiphy->flags &= ~WIPHY_FLAG_PS_ON_BY_DEFAULT;
#endif /*WLAND_POWER_MANAGER */

	/*
	 *@WIPHY_FLAG_NETNS_OK: if not set, do not allow changing the netns of this wiphy at all
	 *@WIPHY_FLAG_4ADDR_AP: supports 4addr mode even on AP (with a single station on a VLAN interface)
	 *@WIPHY_FLAG_4ADDR_STATION: supports 4addr mode even as a station
	 */
	wiphy->flags |= WIPHY_FLAG_NETNS_OK | WIPHY_FLAG_4ADDR_AP |
		WIPHY_FLAG_4ADDR_STATION;
	/*
	 * If driver advertises FW_ROAM, the supplicant wouldn't
	 * * send the BSSID & Freq in the connect command allowing the
	 * * the driver to choose the AP to connect to. But unless we
	 * * support ROAM_CACHE in firware this will delay the ASSOC as
	 * * as the FW need to do a full scan before attempting to connect
	 * * So that feature will just increase assoc. The better approach
	 * * to let Supplicant to provide channel info and FW letter may roam
	 * * if needed so DON'T advertise that featur eto Supplicant.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	/*
	 * wiphy->flags                       |= WIPHY_FLAG_SUPPORTS_FW_ROAM;
	 */
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0) */

	//@WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL: Device supports remain-on-channel call.
	//@WIPHY_FLAG_OFFCHAN_TX: Device supports direct off-channel TX.
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	wiphy->flags |=
		WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL | WIPHY_FLAG_OFFCHAN_TX;
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0) */

	//@WIPHY_FLAG_HAVE_AP_SME: Device integrates AP SME.
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
	/*
	 * From 3.4 kernel ownards AP_SME flag can be advertised to remove the patch from supplicant
	 */
	wiphy->flags |= WIPHY_FLAG_HAVE_AP_SME;
#endif /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0) */

	//@WIPHY_FLAG_CUSTOM_REGULATORY: tell us the driver for this device
	//has its own custom regulatory domain and cannot identify the
	//ISO /IEC 3166 alpha2 it belongs to. When this is enabled
	//we will disregard the first regulatory hint (when the initiator is %REGDOM_SET_BY_CORE).
	wiphy->flags |= WIPHY_FLAG_CUSTOM_REGULATORY;

	WLAND_DBG(CFG80211, INFO, "Registering custom regulatory.\n");

	/*
	 *the driver's regulatory notification callback,
	 *note that if your driver uses wiphy_apply_custom_regulatory()
	 *the reg_nodifier's request can be passed as NULL
	 */
	wiphy->reg_notifier = cfg80211_reg_notifier;

	wiphy_apply_custom_regulatory(wiphy, &wland_regdom);

	err = wiphy_register(wiphy);
	if (unlikely(err < 0)) {
		WLAND_ERR("Could not register wiphy device (%d)\n", err);
		wiphy_free(wiphy);
		return ERR_PTR(err);
	}
	return wiphy;
}

struct wland_cfg80211_vif *wland_alloc_vif(struct wland_cfg80211_info *cfg,
	enum nl80211_iftype type, bool pm_block)
{
	struct wland_cfg80211_vif *vif;

	if (cfg->vif_cnt == WLAND_IFACE_MAX_CNT) {
		WLAND_ERR("vif_cnt(%d)\n", cfg->vif_cnt);
		return ERR_PTR(-ENOSPC);
	}

	WLAND_DBG(CFG80211, TRACE, "allocating virtual interface (size=%zu)\n",
		sizeof(*vif));

	vif = kzalloc(sizeof(struct wland_cfg80211_vif), GFP_KERNEL);
	if (!vif)
		return ERR_PTR(-ENOMEM);

	memset(vif, 0, sizeof(struct wland_cfg80211_vif));

	vif->wdev.wiphy = cfg->wiphy;
	vif->wdev.iftype = type;

	vif->mode = nl80211_iftype_to_mode(type);
	vif->pm_block = pm_block;
	vif->roam_off = -1;

	list_add_tail(&vif->list, &cfg->vif_list);
	cfg->vif_cnt++;
	return vif;
}

void wland_free_vif(struct wland_cfg80211_info *cfg,
	struct wland_cfg80211_vif *vif)
{
	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	list_del(&vif->list);
	cfg->vif_cnt--;

	kfree(vif);

	if (!cfg->vif_cnt) {
		wiphy_unregister(cfg->wiphy);
		wiphy_free(cfg->wiphy);
	}
}

static bool wland_is_nonetwork(struct wland_cfg80211_info *cfg,
	const struct wland_event_msg *e)
{
	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (e->status == STATUS_NO_NETWORKS) {
		WLAND_DBG(CFG80211, TRACE,
			"Processing Link no network found\n");
		return true;
	}

	if (e->status != STATUS_SUCCESS) {
		WLAND_DBG(CFG80211, TRACE,
			"Processing connecting & no network found\n");
		return true;
	}

	return false;
}

static s32 wland_get_assoc_ies(struct wland_cfg80211_info *cfg,
	struct wland_if *ifp)
{
	struct wland_cfg80211_connect_info *conn_info = cfg_to_conn(cfg);
	u8 req_len, resp_len;
	s32 err = 0;

	WLAND_DBG(CFG80211, DEBUG, "Enter\n");

	if (conn_info->req_ie)
		kfree(conn_info->req_ie);
	conn_info->req_ie = NULL;
	conn_info->req_ie_len = 0;
	if (conn_info->resp_ie)
		kfree(conn_info->resp_ie);
	conn_info->resp_ie = NULL;
	conn_info->resp_ie_len = 0;

	err = wland_fil_get_cmd_data(ifp, WID_ASSOC_REQ_INFO, cfg->extra_buf,
		WLAND_ASSOC_INFO_MAX);
	if (err < 0) {
		WLAND_ERR("could not get assoc info:%d\n", err);
		return err;
	}

	req_len = cfg->extra_buf[0];

	if (req_len) {
		//WLAND_DUMP(CFG80211, cfg->extra_buf, (req_len + 1), "assocreq,len:%Zu\n", req_len);
		conn_info->req_ie_len = req_len;
		conn_info->req_ie =
			kmemdup(&cfg->extra_buf[1], conn_info->req_ie_len,
			GFP_KERNEL);
	} else {
		conn_info->req_ie_len = 0;
		conn_info->req_ie = NULL;
	}

	err = wland_fil_get_cmd_data(ifp, WID_ASSOC_RES_INFO, cfg->extra_buf,
		WLAND_ASSOC_INFO_MAX);
	if (err < 0) {
		WLAND_ERR("could not get assoc resp:%d\n", err);
		return err;
	}

	resp_len = cfg->extra_buf[0];
	if (resp_len) {
		//WLAND_DUMP(CFG80211, cfg->extra_buf, (resp_len + 1), "assocres,len:%Zu\n", resp_len);
		conn_info->resp_ie_len = resp_len;
		conn_info->resp_ie =
			kmemdup(&cfg->extra_buf[1], conn_info->resp_ie_len,
			GFP_KERNEL);
	} else {
		conn_info->resp_ie_len = 0;
		conn_info->resp_ie = NULL;
	}

	WLAND_DBG(CFG80211, DEBUG, "Done(req len:%d,resp len:%d)\n",
		conn_info->req_ie_len, conn_info->resp_ie_len);

	return err;
}

static s32 wland_bss_roaming_done(struct wland_cfg80211_info *cfg,
	struct net_device *ndev, const struct wland_event_msg *e)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	struct wland_cfg80211_connect_info *conn_info = cfg_to_conn(cfg);
	struct wiphy *wiphy = cfg_to_wiphy(cfg);
	struct ieee80211_channel *notify_channel = NULL;
	struct ieee80211_supported_band *band =
		wiphy->bands[IEEE80211_BAND_2GHZ];
	struct wland_bss_info_le *bi;
	struct wland_chan ch;
	u32 freq;
	s32 err = 0;
	u8 *buf;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	wland_get_assoc_ies(cfg, ifp);

	buf = kzalloc(WLAND_BSS_INFO_MAX, GFP_KERNEL);
	if (buf == NULL) {
		err = -ENOMEM;
		goto out;
	}

	/*
	 * data sent to dongle has to be little endian
	 */
	*(__le32 *) buf = cpu_to_le32(WLAND_BSS_INFO_MAX);

	err = wland_fil_iovar_data_get(ifp, "get_bss_info", buf,
		WLAND_BSS_INFO_MAX);
	if (err < 0)
		goto done;

	bi = (struct wland_bss_info_le *) (buf + 4);
	ch.chspec = le16_to_cpu(bi->chanspec);
	cfg->d11inf.decchspec(&ch);

	if (ch.band == CHAN_BAND_2G)
		band = wiphy->bands[IEEE80211_BAND_2GHZ];
#ifdef WLAND_5GRF_SUPPORT
	else
		band = wiphy->bands[IEEE80211_BAND_5GHZ];
#endif /*WLAND_5GRF_SUPPORT */

	freq = ieee80211_channel_to_frequency(ch.chnum, band->band);
	notify_channel = ieee80211_get_channel(wiphy, freq);

done:
	kfree(buf);
out:
	cfg80211_roamed(ndev,
		notify_channel,
		(u8 *) profile->bssid,
		conn_info->req_ie,
		conn_info->req_ie_len,
		conn_info->resp_ie, conn_info->resp_ie_len, GFP_KERNEL);
	set_bit(VIF_STATUS_CONNECTED, &ifp->vif->sme_state);
	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);
	return err;
}

static s32 wland_bss_connect_done(struct wland_cfg80211_info *cfg,
	struct net_device *ndev, const struct wland_event_msg *e,
	bool completed)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	struct wland_cfg80211_connect_info *conn_info = cfg_to_conn(cfg);

	WLAND_DBG(CFG80211, DEBUG, "Enter(completed:%d)\n", completed);
	if (test_and_clear_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state)) {
		if (completed) {
			//wland_get_assoc_ies(cfg, ifp);
			WLAND_DBG(CFG80211, TRACE,
				"########## connected and clear VIF_STATUS_CONNECTING\n");
			set_bit(VIF_STATUS_CONNECTED, &ifp->vif->sme_state);
			wland_set_phy_timeout(ifp->drvr, profile->sec.cipher_pairwise);
		}

		cfg80211_connect_result(ndev,
			(u8 *) profile->bssid,
			conn_info->req_ie,
			conn_info->req_ie_len,
			conn_info->resp_ie,
			conn_info->resp_ie_len,
			completed ? WLAN_STATUS_SUCCESS :
			WLAN_STATUS_AUTH_TIMEOUT, GFP_KERNEL);
		WLAND_DBG(CFG80211, INFO,
			"Report connect result - connection %s\n",
			completed ? "succeeded" : "failed");
	}
	WLAND_DBG(CFG80211, TRACE, "Done\n");
	return 0;
}

static s32 notify_connect_status(struct wland_if *ifp,
	const struct wland_event_msg *e, void *data)
{
	struct wland_cfg80211_info *cfg = ifp->drvr->config;
	struct wland_cfg80211_connect_info *conn_info = &cfg->conn_info;
	struct net_device *ndev = ifp->ndev;
	struct wland_cfg80211_profile *profile = ndev_to_prof(ndev);
	s32 err = 0;
	u16 eventLen = 0;
	u8 *pU8Buffer = (u8 *) data;

	WLAND_DBG(CFG80211, TRACE, "Enter(ifp->vif->mode:%d)\n",
		ifp->vif->mode);

	if (!check_vif_up(ifp->vif)) {
		WLAND_DBG(CFG80211, TRACE, "Device is not ready - vif not set up (event:%d,reason:%d)\n", e->event_code,
			e->reason);
		err = -EINVAL;
		return err;
	}

	cancel_work_sync(&conn_info->work);
	if (timer_pending(&conn_info->timer)) {
		del_timer_sync(&conn_info->timer);
		WLAND_DBG(CFG80211, TRACE, "###### delete conn_info->timer\n");
	}

	if (ifp->vif->mode == WL_MODE_AP) {
		static int generation;
		struct station_info sinfo;

		WLAND_DBG(CFG80211, TRACE, "SoftAp,event_code:%d\n",
			e->event_code);

		if (ndev != cfg_to_ndev(cfg)) {
			WLAND_DBG(CFG80211, TRACE, "AP mode link down\n");
			complete(&cfg->vif_disabled);
			goto fail;
		}

		if ((e->event_code == WLAND_E_CONNECT_IND)
			&& (e->action == EVENT_ASSOC_IND
				|| e->action == EVENT_REASSOC_IND)
			&& (e->status == STATUS_SUCCESS)) {
			/*
			 * Extract network event length
			 */
			eventLen = MAKE_WORD16(pU8Buffer[2], pU8Buffer[3]);

			WLAND_DBG(CFG80211, DEBUG,
				"WLAND_E_CONNECT_IND, e->addr=%pM\n", e->addr);
			WLAND_DUMP(RX_NETEVENT, pU8Buffer, eventLen + 4,
				"Event len:%Zu\n", eventLen);
			memset(&sinfo, 0, sizeof(sinfo));
			sinfo.filled = STATION_INFO_ASSOC_REQ_IES;
			if (!pU8Buffer) {
				WLAND_ERR
					("No IEs present in ASSOC/REASSOC_IND");
				err = -EINVAL;
				goto fail;
			}
			sinfo.assoc_req_ies = pU8Buffer + 6 + 24 + 2 + 2;
			sinfo.assoc_req_ies_len = eventLen - 24 - 2 - 2 - 4;
			generation++;
			sinfo.generation = generation;
			cfg80211_new_sta(ndev, e->addr, &sinfo, GFP_KERNEL);
		} else if ((e->event_code == WLAND_E_DISCONNECT_IND)
			&& (e->action == EVENT_DISASSOC_IND)) {
			WLAND_DBG(CFG80211, DEBUG,
				"WLAND_E_DISCONNECT_IND, e->addr=%pM\n",
				e->addr);
			cfg80211_del_sta(ndev, e->addr, GFP_KERNEL);
		}
	} else if ((e->event_code == WLAND_E_CONNECT_IND)
		&& (e->status == STATUS_SUCCESS)) {
		WLAND_DBG(CFG80211, DEBUG, "Linkup\n");

		/*
		 * start restore work timer
		 */
		mod_timer(&conn_info->connect_restorework_timeout,
			jiffies + msecs_to_jiffies(CONNECT_RESTOREWORK_TIMER_MS));
		WLAND_DBG(CFG80211, TRACE, "###### Set restore work timer(%d s)!\n",
			CONNECT_RESTOREWORK_TIMER_MS);

		if (ifp->vif->mode == WL_MODE_IBSS) {
			memcpy(profile->bssid, e->addr, ETH_ALEN);
			wland_inform_ibss(cfg, ndev, e->addr);
			cfg80211_ibss_joined(ndev, e->addr, GFP_KERNEL);
			clear_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state);
			set_bit(VIF_STATUS_CONNECTED, &ifp->vif->sme_state);
		} else {
			WLAND_DBG(CFG80211, INFO,
				"connected. wland_bss_connect_done\n");
			if (!netif_carrier_ok(ndev)) {
				netif_carrier_on(ndev);
				WLAND_DBG(CFG80211, TRACE,
					"netif_carrier_on(ndev)\n");
			}
			if (netif_queue_stopped(ndev)) {
				netif_wake_queue(ndev);
				WLAND_DBG(CFG80211, TRACE,
					"netif_wake_queue(ndev)\n");
			}
			wland_bss_connect_done(cfg, ndev, e, true);
		}
	} else if (e->event_code == WLAND_E_DISCONNECT_IND) {
		WLAND_DBG(CFG80211, TRACE,
			"Linkdown ifp->vif->mode=%d, sme_state=%x\n",
			ifp->vif->mode, (u32) ifp->vif->sme_state);

		if (timer_pending(&conn_info->connect_restorework_timeout)) {
			del_timer_sync(&conn_info->connect_restorework_timeout);
			WLAND_DBG(CFG80211, TRACE, "###### delete connect restorework timer\n");
		}

		if (ifp->vif->mode != WL_MODE_IBSS) {
			if (!netif_queue_stopped(ndev)) {
				netif_stop_queue(ndev);
				WLAND_DBG(CFG80211, TRACE,
					"netif_stop_queue(ndev)\n");
			}
			if (netif_carrier_ok(ndev)) {
				netif_carrier_off(ndev);
				WLAND_DBG(CFG80211, TRACE,
					"netif_carrier_off(ndev)\n");
			}
			wland_bss_connect_done(cfg, ndev, e, false);

			if (test_and_clear_bit(VIF_STATUS_CONNECTED,
					&ifp->vif->sme_state)) {
				cfg80211_disconnected(ndev, 0, NULL, 0,
					GFP_KERNEL);
				WLAND_DBG(CFG80211, INFO, "cfg80211_disconnected\n");
			}
		}

		wland_link_down(ifp->vif);

		memset(profile, '\0', sizeof(struct wland_cfg80211_profile));
		if (cfg->in_disconnecting) {
			if (cfg->in_waiting) {
				complete(&cfg->disconnecting_wait);
			}
			cfg->in_disconnecting = false;
		}

		if (ndev != cfg_to_ndev(cfg))
			complete(&cfg->vif_disabled);
	} else if (wland_is_nonetwork(cfg, e)) {
		if (ifp->vif->mode != WL_MODE_IBSS)
			clear_bit(VIF_STATUS_CONNECTING, &ifp->vif->sme_state);
		else
			wland_bss_connect_done(cfg, ndev, e, false);
	}

fail:
	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);
	return err;
}

static s32 notify_roaming_status(struct wland_if *ifp,
	const struct wland_event_msg *e, void *data)
{
	struct wland_cfg80211_info *cfg = ifp->drvr->config;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	if (e->event_code == WLAND_E_ROAM && e->status == STATUS_SUCCESS) {
		if (test_bit(VIF_STATUS_CONNECTED, &ifp->vif->sme_state))
			wland_bss_roaming_done(cfg, ifp->ndev, e);
		else
			wland_bss_connect_done(cfg, ifp->ndev, e, true);
	}

	return 0;
}

static s32 notify_vif_event(struct wland_if *ifp,
	const struct wland_event_msg *e, void *data)
{
	struct wland_cfg80211_info *cfg = ifp->drvr->config;
	struct wland_cfg80211_vif_event *event = &cfg->vif_event;
	struct wland_cfg80211_vif *vif = event->vif;

	WLAND_DBG(CFG80211, TRACE, "Enter: action %u ifidx %u bsscfg %u\n",
		e->action, e->ifidx, e->bsscfgidx);

	mutex_lock(&event->vif_event_lock);
	event->action = e->action;

	switch (e->action) {
	case WLAND_E_IF_ADD:
		/*
		 * waiting process may have timed out
		 */
		if (!cfg->vif_event.vif) {
			mutex_unlock(&event->vif_event_lock);
			return -EBADF;
		}

		ifp->vif = vif;

		vif->ifp = ifp;
		if (ifp->ndev) {
			vif->wdev.netdev = ifp->ndev;
			ifp->ndev->ieee80211_ptr = &vif->wdev;

			SET_NETDEV_DEV(ifp->ndev, wiphy_dev(cfg->wiphy));
		}
		mutex_unlock(&event->vif_event_lock);
		wake_up(&event->vif_wq);
		return 0;

	case WLAND_E_IF_DEL:
		mutex_unlock(&event->vif_event_lock);
		/*
		 * event may not be upon user request
		 */
		if (wland_cfg80211_vif_event_armed(cfg))
			wake_up(&event->vif_wq);
		return 0;

	case WLAND_E_IF_CHANGE:
		mutex_unlock(&event->vif_event_lock);
		wake_up(&event->vif_wq);
		return 0;

	default:
		mutex_unlock(&event->vif_event_lock);
		break;
	}

	return -EINVAL;
}

static void wland_register_event_handlers(struct wland_cfg80211_info *cfg)
{
	firmweh_register(cfg->pub, WLAND_E_ESCAN_RESULT, notify_escan_handler);
	firmweh_register(cfg->pub, WLAND_E_DISCONNECT_IND,
		notify_connect_status);
	firmweh_register(cfg->pub, WLAND_E_CONNECT_IND, notify_connect_status);

	firmweh_register(cfg->pub, WLAND_E_ROAM, notify_roaming_status);
	firmweh_register(cfg->pub, WLAND_E_PFN_NET_FOUND,
		notify_sched_scan_results);

	firmweh_register(cfg->pub, WLAND_E_IF_ADD, notify_vif_event);
	firmweh_register(cfg->pub, WLAND_E_IF_DEL, notify_vif_event);
	firmweh_register(cfg->pub, WLAND_E_IF_CHANGE, notify_vif_event);
#ifdef WLAND_P2P_SUPPORT
	firmweh_register(cfg->pub, WLAND_E_P2P_PROBEREQ_MSG,
		notify_p2p_rx_mgmt_probereq);
	firmweh_register(cfg->pub, WLAND_E_P2P_DISC_LISTEN_COMPLETE,
		notify_p2p_listen_complete);
	firmweh_register(cfg->pub, WLAND_E_ACTION_FRAME_RX,
		notify_p2p_action_frame_rx);
	firmweh_register(cfg->pub, WLAND_E_ACT_FRAME_COMPLETE,
		notify_p2p_action_tx_complete);
	firmweh_register(cfg->pub, WLAND_E_ACT_FRAME_OFF_CHAN_COMPLETE,
		notify_p2p_action_tx_complete);
#endif /* WLAND_P2P_SUPPORT */
}

static void wland_deinit_priv_mem(struct wland_cfg80211_info *cfg)
{
	WLAND_DBG(CFG80211, DEBUG, "Enter\n");

	if (cfg->conf)
		kfree(cfg->conf);
	cfg->conf = NULL;

	if (cfg->extra_buf)
		kfree(cfg->extra_buf);
	cfg->extra_buf = NULL;

	if (cfg->pmk_list)
		kfree(cfg->pmk_list);
	cfg->pmk_list = NULL;

	WLAND_DBG(CFG80211, DEBUG, "Done\n");
}

static s32 wland_init_priv_mem(struct wland_cfg80211_info *cfg)
{
	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	cfg->conf = NULL;
	cfg->extra_buf = NULL;
	cfg->pmk_list = NULL;

	cfg->conf = kzalloc(sizeof(struct wland_cfg80211_conf), GFP_KERNEL);
	if (!cfg->conf)
		goto init_priv_mem_out;

	cfg->extra_buf = kzalloc(WLAND_EXTRA_BUF_MAX, GFP_KERNEL);
	if (!cfg->extra_buf)
		goto init_priv_mem_out;

	cfg->pmk_list =
		kzalloc(sizeof(struct wland_cfg80211_pmk_list), GFP_KERNEL);
	if (!cfg->pmk_list)
		goto init_priv_mem_out;

	WLAND_DBG(CFG80211, TRACE, "Success Done\n");
	return 0;

init_priv_mem_out:wland_deinit_priv_mem(cfg);

	WLAND_DBG(CFG80211, TRACE, "Failed Done\n");
	return -ENOMEM;
}

static s32 wland_dongle_roam(struct wland_if *ifp, u32 roamvar, u32 bcn_timeout)
{
	s32 err = 0;
	__le32 roamtrigger[2];
	__le32 roam_delta[2];

	/*
	 * Setup timeout if Beacons are lost and roam is off to report link down
	 */
	if (roamvar) {
		err = wland_fil_iovar_data_set(ifp, "bcn_timeout", &bcn_timeout,
			sizeof(bcn_timeout));
		if (err < 0) {
			WLAND_ERR("bcn_timeout error (%d)\n", err);
			goto dongle_rom_out;
		}
	}

	/*
	 * Enable/Disable built-in roaming to allow supplicant to take care of roaming
	 */
	WLAND_DBG(CFG80211, TRACE, "Internal Roaming = %s\n",
		roamvar ? "Off" : "On");

	err = wland_fil_iovar_data_set(ifp, "roam_off", &roamvar,
		sizeof(roamvar));
	if (err < 0) {
		WLAND_ERR("roam_off error (%d)\n", err);
		goto dongle_rom_out;
	}

	roamtrigger[0] = cpu_to_le32(ROAM_TRIGGER_LEVEL);
	roamtrigger[1] = cpu_to_le32(WLAND_BAND_ALL);
	err = wland_fil_iovar_data_set(ifp, "roam_trigger", roamtrigger,
		sizeof(roamtrigger));
	if (err < 0) {
		WLAND_ERR("WLC_SET_ROAM_TRIGGER error (%d)\n", err);
		goto dongle_rom_out;
	}

	roam_delta[0] = cpu_to_le32(ROAM_DELTA);
	roam_delta[1] = cpu_to_le32(WLAND_BAND_ALL);
	err = wland_fil_iovar_data_set(ifp, "roam_delta", roam_delta,
		sizeof(roam_delta));
	if (err < 0) {
		WLAND_ERR("WLC_SET_ROAM_DELTA error (%d)\n", err);
		goto dongle_rom_out;
	}

dongle_rom_out:
	return err;
}

static s32 wland_construct_reginfo(struct wland_cfg80211_info *cfg, u32 bw_cap)
{
	struct wland_if *ifp = netdev_priv(cfg_to_ndev(cfg));
	struct ieee80211_channel *band_chan_arr;
	struct wland_chanspec_list *list;
	struct wland_chan ch;
	u8 *pbuf;
	s32 err;
	u32 i, j, total, channel, index, ht40_flag, array_size;
	enum ieee80211_band band;
	u32 *n_cnt;
	bool ht40_allowed, update;

	return 0;

	pbuf = kzalloc(WLAND_DCMD_MEDLEN, GFP_KERNEL);
	if (pbuf == NULL)
		return -ENOMEM;

	list = (struct wland_chanspec_list *) pbuf;
	err = wland_fil_iovar_data_get(ifp, "chanspecs", pbuf,
		WLAND_DCMD_MEDLEN);
	if (err < 0) {
		WLAND_ERR("get chanspecs error (%d)\n", err);
		goto exit;
	}

	__wl_band_2ghz.n_channels = 0;

#ifdef WLAND_5GRF_SUPPORT
	__wl_band_5ghz_a.n_channels = 0;
#endif /* WLAND_5GRF_SUPPORT */
	total = le32_to_cpu(list->count);

	for (i = 0; i < total; i++) {
		ch.chspec = (u16) le32_to_cpu(list->element[i]);
		cfg->d11inf.decchspec(&ch);

		if (ch.band == CHAN_BAND_2G) {
			band_chan_arr = __wl_2ghz_channels;
			array_size = ARRAY_SIZE(__wl_2ghz_channels);
			n_cnt = &__wl_band_2ghz.n_channels;
			band = IEEE80211_BAND_2GHZ;
			ht40_allowed = (bw_cap == WLC_N_BW_40ALL);
		}
#ifdef WLAND_5GRF_SUPPORT
		else if (ch.band == CHAN_BAND_5G) {
			band_chan_arr = __wl_5ghz_a_channels;
			array_size = ARRAY_SIZE(__wl_5ghz_a_channels);
			n_cnt = &__wl_band_5ghz_a.n_channels;
			band = IEEE80211_BAND_5GHZ;
			ht40_allowed = !(bw_cap == WLC_N_BW_20ALL);
		}
#endif /* WLAND_5GRF_SUPPORT */
		else {
			WLAND_ERR("Invalid channel Sepc. 0x%x.\n", ch.chspec);
			continue;
		}

		if (!ht40_allowed && ch.bw == CHAN_BW_40)
			continue;

		update = false;

		for (j = 0; (j < *n_cnt && (*n_cnt < array_size)); j++) {
			if (band_chan_arr[j].hw_value == ch.chnum) {
				update = true;
				break;
			}
		}

		if (update)
			index = j;
		else
			index = *n_cnt;

		if (index < array_size) {
			band_chan_arr[index].center_freq =
				ieee80211_channel_to_frequency(ch.chnum, band);
			band_chan_arr[index].hw_value = ch.chnum;

			if (ch.bw == CHAN_BW_40 && ht40_allowed) {
				/*
				 * assuming the order is HT20, HT40 Upper, HT40 lower from chanspecs
				 */
				ht40_flag =
					band_chan_arr[index].flags &
					IEEE80211_CHAN_NO_HT40;
				if (ch.sb == WLAND_CHAN_SB_U) {
					if (ht40_flag == IEEE80211_CHAN_NO_HT40)
						band_chan_arr[index].flags &=
							~IEEE80211_CHAN_NO_HT40;
					band_chan_arr[index].flags |=
						IEEE80211_CHAN_NO_HT40PLUS;
				} else {
					/*
					 * It should be one of IEEE80211_CHAN_NO_HT40 or IEEE80211_CHAN_NO_HT40PLUS
					 */
					band_chan_arr[index].flags &=
						~IEEE80211_CHAN_NO_HT40;
					if (ht40_flag == IEEE80211_CHAN_NO_HT40)
						band_chan_arr[index].flags |=
							IEEE80211_CHAN_NO_HT40MINUS;
				}
			} else {
				band_chan_arr[index].flags =
					IEEE80211_CHAN_NO_HT40;
				ch.bw = CHAN_BW_20;
				cfg->d11inf.encchspec(&ch);
				channel = ch.chspec;
				err = wland_fil_iovar_data_get(ifp,
					"per_chan_info", &channel,
					sizeof(channel));
				if (!err) {
					if (channel & WL_CHAN_RADAR)
						band_chan_arr[index].flags |=
							(IEEE80211_CHAN_RADAR |
							IEEE80211_CHAN_NO_IBSS);
					if (channel & WL_CHAN_PASSIVE)
						band_chan_arr[index].flags |=
							IEEE80211_CHAN_PASSIVE_SCAN;
				}
			}
			if (!update)
				(*n_cnt)++;
		}
	}
exit:
	kfree(pbuf);
	return err;
}

static s32 wland_update_wiphybands(struct wland_cfg80211_info *cfg, bool notify)
{
	struct wland_if *ifp = netdev_priv(cfg_to_ndev(cfg));
	struct wiphy *wiphy = cfg_to_wiphy(cfg);
	struct ieee80211_supported_band *bands[IEEE80211_NUM_BANDS];
	s32 phy_list, err, i, index;
	u32 band_list[3], nmode, nband, bw_cap = 0;
	s8 phy;

	err = wland_fil_iovar_data_get(ifp, "get_phylist", &phy_list,
		sizeof(phy_list));
	if (err < 0) {
		WLAND_ERR("WLAND_C_GET_PHYLIST error (%d)\n", err);
		return err;
	}

	phy = ((char *) &phy_list)[0];

	WLAND_DBG(CFG80211, TRACE, "GET_PHYLIST reported: %c phy\n", phy);

	err = wland_fil_iovar_data_get(ifp, "nmode", &nmode, sizeof(nmode));
	if (err < 0) {
		WLAND_ERR("nmode error (%d)\n", err);
	} else {
		err = wland_fil_iovar_data_get(ifp, "mimo_bw_cap", &bw_cap,
			sizeof(bw_cap));
		/*
		 * set default value
		 */
		bw_cap = 0, nmode = 1;
		band_list[0] = 0x01;
		band_list[1] = 0x02;
		band_list[2] = 0x00;
		if (err < 0)
			WLAND_ERR("mimo_bw_cap error (%d)\n", err);
	}
	WLAND_DBG(CFG80211, TRACE, "nmode=%d, mimo_bw_cap=%d\n", nmode, bw_cap);

	err = wland_construct_reginfo(cfg, bw_cap);
	if (err < 0) {
		WLAND_ERR("construct reginfo failed (%d)\n", err);
		return err;
	}

	nband = band_list[0];
	memset(bands, 0, sizeof(bands));

	for (i = 1; i <= nband && i < ARRAY_SIZE(band_list); i++) {
		index = -1;
#ifdef WLAND_5GRF_SUPPORT
		if ((band_list[i] == WLC_BAND_5G)
			&& (__wl_band_5ghz_a.n_channels > 0)) {
			index = IEEE80211_BAND_5GHZ;
			bands[index] = &__wl_band_5ghz_a;

			if ((bw_cap == WLC_N_BW_40ALL)
				|| (bw_cap == WLC_N_BW_20IN2G_40IN5G))
				bands[index]->ht_cap.cap |=
					IEEE80211_HT_CAP_SGI_40;
		}
#endif /*WLAND_5GRF_SUPPORT */
		if ((band_list[i] == WLC_BAND_2G)
			&& (__wl_band_2ghz.n_channels > 0)) {
			index = IEEE80211_BAND_2GHZ;
			bands[index] = &__wl_band_2ghz;

			if (bw_cap == WLC_N_BW_40ALL)
				bands[index]->ht_cap.cap |=
					IEEE80211_HT_CAP_SGI_40;
		}

		WLAND_DBG(CFG80211, TRACE, "index:%d,nmode:%d\n", index, nmode);

		/*
		 * setup 802.11n
		 */
		if ((index >= 0) && nmode) {
			bands[index]->ht_cap.ht_supported = true;
			bands[index]->ht_cap.cap |=
				(1 << IEEE80211_HT_CAP_RX_STBC_SHIFT);
			bands[index]->ht_cap.mcs.rx_mask[0] = 0xFF;
			bands[index]->ht_cap.ampdu_factor =
				IEEE80211_HT_MAX_AMPDU_8K;
			bands[index]->ht_cap.ampdu_density =
				IEEE80211_HT_MPDU_DENSITY_NONE;
#ifdef WLAND_TBD_SUPPORT
			bands[index]->ht_cap.cap |= IEEE80211_HT_CAP_SGI_20;
			bands[index]->ht_cap.cap |= IEEE80211_HT_CAP_DSSSCCK40;
			bands[index]->ht_cap.ht_supported = true;
			bands[index]->ht_cap.ampdu_factor =
				IEEE80211_HT_MAX_AMPDU_64K;
			bands[index]->ht_cap.ampdu_density =
				IEEE80211_HT_MPDU_DENSITY_16;
			/*
			 * An HT shall support all EQM rates for one spatial stream
			 */
			bands[index]->ht_cap.mcs.rx_mask[0] = 0xFF;
#endif /*WLAND_TBD_SUPPORT */
		}
	}

	wiphy->bands[IEEE80211_BAND_2GHZ] = bands[IEEE80211_BAND_2GHZ];
#ifdef WLAND_5GRF_SUPPORT
	wiphy->bands[IEEE80211_BAND_5GHZ] = bands[IEEE80211_BAND_5GHZ];
#endif /*WLAND_5GRF_SUPPORT */
	if (notify)
		wiphy_apply_custom_regulatory(wiphy, &wland_regdom);

	return err;
}

#ifdef WLAND_INIT_SCAN_SUPPORT
static inline void wland_init_scan(struct wland_cfg80211_info *cfg,
	struct net_device * ndev, struct wireless_dev *wdev)
{
	//init scan
	struct cfg80211_scan_request request;
	struct cfg80211_ssid ssids;
	memset(&request, 0, sizeof(request));
#if   LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	request.wdev = wdev;
#endif
	request.n_ssids = 1;

	ssids.ssid_len = 0;
	request.ssids = &ssids;

	//request.n_channels = 14;
	//request.n_ssids = 1;
	//request.ssids = NULL;

	cfg80211_scan(cfg->wiphy,
#if    LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
	ndev,
#endif/*LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0) */
	&request);
}
#endif

s32 wland_cfg80211_up(struct net_device * ndev)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_info *cfg = ndev_to_cfg(ndev);
	struct wireless_dev *wdev = ndev->ieee80211_ptr;
	s32 power_mode, err = 0;

	mutex_lock(&cfg->usr_sync);

	set_bit(VIF_STATUS_READY, &ifp->vif->sme_state);

	if (cfg->dongle_up) {
		WLAND_DBG(CFG80211, TRACE, "dongle is already up and received another up request\n");
		//WLAND_ERR("dongle up\n");
		goto up_exit;
	}

	power_mode = cfg->pwr_save ? MIN_FAST_PS : NO_POWERSAVE;

	WLAND_DBG(CFG80211, TRACE, "power save set to %s\n",
		(power_mode ? "enabled" : "disabled"));

	err = wland_dongle_roam(ifp, (cfg->roam_on ? 0 : 1),
		WLAND_BEACON_TIMEOUT);
	if (err < 0)
		goto up_exit;

	err = cfg80211_change_virtual_iface(wdev->wiphy, ndev, wdev->iftype,
		NULL, NULL);
	if (err < 0)
		goto up_exit;

	/*
	 * according chip cap to update wiphybands
	 */
	err = wland_update_wiphybands(cfg, true);
	if (err)
		goto up_exit;

	cfg->dongle_up = true;
#ifdef WLAND_INIT_SCAN_SUPPORT
	wland_init_scan(cfg, ndev, wdev);
#endif
up_exit:
	mutex_unlock(&cfg->usr_sync);

	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return err;
}

s32 wland_cfg80211_down(struct net_device * ndev)
{
	struct wland_if *ifp = netdev_priv(ndev);
	struct wland_cfg80211_info *cfg = ndev_to_cfg(ndev);
	s32 err = 0;

	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	if (!cfg || !ifp) {
		WLAND_ERR("Invalid Parameter\n");
		goto fail;
	}

	/*
	 * While going down, if associated with AP disassociate from AP to save power
	 */
	mutex_lock(&cfg->usr_sync);
	if (check_vif_up(ifp->vif)) {
		wland_link_down(ifp->vif);

		/*
		 * Make sure WPA_Supplicant receives all the event generated due to DISASSOC
		 * call to the fw to keep the state fw and WPA_Supplicant state consistent
		 */
		//wland_delay(200);
	}
	wland_abort_scanning(cfg);
	clear_bit(VIF_STATUS_READY, &ifp->vif->sme_state);
	mutex_unlock(&cfg->usr_sync);

fail:
	WLAND_DBG(CFG80211, TRACE, "Done(err:%d)\n", err);

	return err;
}

bool wland_vif_get_state_all(struct wland_cfg80211_info * cfg, ulong state)
{
	struct wland_cfg80211_vif *vif;
	u8 result = 0;

	list_for_each_entry(vif, &cfg->vif_list, list) {
		if (test_bit(state, &vif->sme_state))
			result++;
	}

	WLAND_DBG(CFG80211, TRACE, "vif state(result:%d)\n", result);

	return result;
}

static inline bool vif_event_equals(struct wland_cfg80211_vif_event *event,
	u8 action)
{
	u8 evt_action;

	mutex_lock(&event->vif_event_lock);
	evt_action = event->action;
	mutex_unlock(&event->vif_event_lock);

	WLAND_DBG(CFG80211, TRACE, "Enter(evt_action:%d)\n", evt_action);

	return evt_action == action;
}

void wland_cfg80211_arm_vif_event(struct wland_cfg80211_info *cfg,
	struct wland_cfg80211_vif *vif)
{
	struct wland_cfg80211_vif_event *event = &cfg->vif_event;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	mutex_lock(&event->vif_event_lock);
	event->vif = vif;
	event->action = 0;
	mutex_unlock(&event->vif_event_lock);
}

bool wland_cfg80211_vif_event_armed(struct wland_cfg80211_info *cfg)
{
	struct wland_cfg80211_vif_event *event = &cfg->vif_event;
	bool armed;

	mutex_lock(&event->vif_event_lock);
	armed = event->vif != NULL;
	mutex_unlock(&event->vif_event_lock);

	WLAND_DBG(CFG80211, TRACE, "Enter(armed:%d)\n", armed);

	return armed;
}

int wland_cfg80211_wait_vif_event_timeout(struct wland_cfg80211_info *cfg,
	u8 action, ulong timeout)
{
	struct wland_cfg80211_vif_event *event = &cfg->vif_event;

	WLAND_DBG(CFG80211, TRACE, "Enter\n");

	return wait_event_timeout(event->vif_wq, vif_event_equals(event,
			action), timeout);
}

/* attach to cfg80211 mode */
struct wland_cfg80211_info *cfg80211_attach(struct wland_private *drvr,
	struct device *busdev)
{
	struct net_device *ndev = drvr->iflist[0]->ndev;
	struct wland_cfg80211_info *cfg;
	struct wiphy *wiphy;
	struct wland_cfg80211_vif *vif;
	struct wland_if *ifp = netdev_priv(ndev);
	s32 err = 0;

	if (!ndev) {
		WLAND_ERR("ndev is invalid\n");
		return NULL;
	}

	WLAND_DBG(CFG80211, TRACE, "Enter,ifp:%p,drvr->iflist[0]:%p.\n", ifp,
		drvr->iflist[0]);

	wiphy = wland_setup_wiphy(busdev);
	if (IS_ERR(wiphy)) {
		WLAND_ERR("setup wiphy failed!\n");
		return NULL;
	}

	/*
	 * setup cfg structure
	 */
	cfg = wiphy_priv(wiphy);
	cfg->wiphy = wiphy;
	cfg->pub = drvr;

	init_waitqueue_head(&cfg->vif_event.vif_wq);

	mutex_init(&cfg->vif_event.vif_event_lock);

	INIT_LIST_HEAD(&cfg->vif_list);

	vif = wland_alloc_vif(cfg, NL80211_IFTYPE_STATION, false);
	if (IS_ERR(vif)) {
		wiphy_free(wiphy);
		WLAND_ERR("vif malloc failed\n");
		return NULL;
	}

	vif->ifp = ifp;
	vif->wdev.netdev = ndev;
	ndev->ieee80211_ptr = &vif->wdev;

	SET_NETDEV_DEV(ndev, wiphy_dev(cfg->wiphy));

	cfg->scan_request = NULL;
	cfg->pwr_save = true;
	cfg->roam_on = true;	/* roam on & off switch. we enable roam per default */
	cfg->active_scan = true;	/* we do active scan for specific scan per default */
	cfg->dongle_up = false;	/* chip is not up yet */

	mutex_init(&cfg->usr_sync);

#ifdef WLAND_INIT_SCAN_SUPPORT
	atomic_set(&cfg->init_scan, 0);
#endif
	mutex_init(&cfg->scan_result_lock);
	INIT_LIST_HEAD(&cfg->scan_result_list);
	cfg->scan_results.version = SCAN_VERSION_NUM;
	cfg->scan_results.count = 0;

	err = wland_init_priv_mem(cfg);
	if (err < 0) {
		WLAND_ERR("Failed to init iwm_priv (%d)\n", err);
		goto cfg80211_attach_out;
	}

	/*
	 * register event cb when received from chip
	 */
	wland_register_event_handlers(cfg);

	/*
	 * init escan
	 */
	cfg->scan_info.escan_state = SCAN_STATE_IDLE;

	/*
	 * Init scan_timeout timer
	 */
	init_timer(&cfg->scan_timeout);
	cfg->scan_timeout.data = (ulong) cfg;
	cfg->scan_timeout.function = wland_scan_timeout;

	INIT_WORK(&cfg->scan_report_work, cfg80211_scan_report_worker);

	err = wland_init_connect_info(cfg);
	if (err < 0) {
		WLAND_ERR("Failed to init connect_info (err=%d)\n", err);
		goto cfg80211_attach_out;
	}

	/*
	 * init config
	 */
	if (cfg->conf)
		memset(cfg->conf, 0, sizeof(struct wland_cfg80211_conf));

	init_completion(&cfg->vif_disabled);
	cfg->in_disconnecting = false;
	cfg->in_waiting = false;
	init_completion(&cfg->disconnecting_wait);

	ifp->vif = vif;

#ifdef WLAND_P2P_SUPPORT
	err = cfg80211_p2p_attach(cfg);
	if (err < 0) {
		WLAND_ERR("P2P initilisation failed (%d)\n", err);
		goto cfg80211_p2p_attach_out;
	}
#endif /* WLAND_P2P_SUPPORT */

#ifdef WLAND_BTCOEX_SUPPORT
	err = wland_btcoex_attach(cfg);
	if (err < 0) {
		WLAND_ERR("BT-coex initialisation failed (%d)\n", err);
#ifdef WLAND_P2P_SUPPORT
		cfg80211_p2p_detach(&cfg->p2p);
#endif /* WLAND_P2P_SUPPORT */
		goto cfg80211_p2p_attach_out;
	}
#endif /* WLAND_BTCOEX_SUPPORT */

	cfg->d11inf.io_type = WLAND_D11N_IOTYPE;

	wland_d11_attach(&cfg->d11inf);

#ifdef WLAND_MONITOR_SUPPORT
	wland_monitor_init(drvr);
#endif /*WLAND_MONITOR_SUPPORT */
	WLAND_DBG(CFG80211, TRACE, "Success Done\n");

	return cfg;

#ifdef WLAND_P2P_SUPPORT
cfg80211_p2p_attach_out:
	cfg->dongle_up = false;	/* dongle down */
	wland_deinit_connect_info(cfg);
	wland_abort_scanning(cfg);
	wland_deinit_priv_mem(cfg);
#endif /* WLAND_P2P_SUPPORT */

cfg80211_attach_out:
	wland_free_vif(cfg, vif);
	WLAND_DBG(CFG80211, TRACE, "Failed Done\n");
	return NULL;
}

/* dettach from annother mode */
void cfg80211_detach(struct wland_cfg80211_info *cfg)
{
	struct wland_cfg80211_vif *vif;
	struct wland_cfg80211_vif *tmp;
	struct wland_bss_info_le *bss;
	struct wland_bss_info_le *bss_tmp;
	WLAND_DBG(CFG80211, DEBUG, "Enter\n");

	if (!cfg || IS_ERR(cfg)) {
		WLAND_ERR("cfg is NULL or IS_ERR!\n");
		return;
	}

	cfg->dongle_up = false;	/* dongle down */

	wland_deinit_connect_info(cfg);
	wland_abort_scanning(cfg);
	wland_deinit_priv_mem(cfg);

#ifdef WLAND_BTCOEX_SUPPORT
	wland_btcoex_detach(cfg);
#endif /* WLAND_BTCOEX_SUPPORT */

#ifdef WLAND_MONITOR_SUPPORT
	wland_monitor_deinit();
#endif /*WLAND_MONITOR_SUPPORT */
#if defined(WLAND_RSSIAVG_SUPPORT)
	wl_free_rssi_cache(&g_rssi_cache_ctrl);
#endif
#if defined(WLAND_BSSCACHE_SUPPORT)
	wl_release_bss_cache_ctrl(&g_bss_cache_ctrl);
#endif
	list_for_each_entry_safe(bss, bss_tmp, &cfg->scan_result_list, list)
		wland_free_bss(cfg, bss);

	list_for_each_entry_safe(vif, tmp, &cfg->vif_list, list) {
		wland_free_vif(cfg, vif);
	}
}
