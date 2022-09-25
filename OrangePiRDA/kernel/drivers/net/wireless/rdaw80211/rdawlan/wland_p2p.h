
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
#ifndef _WLAND_P2P_H_
#define _WLAND_P2P_H_

#include <net/cfg80211.h>

struct wland_cfg80211_info;

/*
 * enum p2p_bss_type - different type of BSS configurations.
 *
 * @P2PAPI_BSSCFG_PRIMARY: maps to driver's primary bsscfg.
 * @P2PAPI_BSSCFG_DEVICE:  maps to driver's P2P device discovery bsscfg.
 * @P2PAPI_BSSCFG_CONNECTION: maps to driver's P2P connection bsscfg.
 * @P2PAPI_BSSCFG_MAX: used for range checking.
 */
enum p2p_bss_type {
	P2PAPI_BSSCFG_PRIMARY,	/* maps to driver's primary bsscfg */
	P2PAPI_BSSCFG_DEVICE,	/* maps to driver's P2P device discovery bsscfg */
	P2PAPI_BSSCFG_CONNECTION,	/* maps to driver's P2P connection bsscfg */
	P2PAPI_BSSCFG_MAX
};

/*
 * struct p2p_bss - peer-to-peer bss related information.
 *
 * @vif         : virtual interface of this P2P bss.
 * @private_data: TBD
 */
struct p2p_bss {
	struct wland_cfg80211_vif *vif;
	void *private_data;
};

/*
 * enum wland_p2p_status - P2P specific dongle status.
 *
 * @P2P_STATUS_IF_ADD                : peer-to-peer vif add sent to dongle.
 * @P2P_STATUS_IF_DEL                : NOT-USED?
 * @P2P_STATUS_IF_DELETING           : peer-to-peer vif delete sent to dongle.
 * @P2P_STATUS_IF_CHANGING           : peer-to-peer vif change sent to dongle.
 * @P2P_STATUS_IF_CHANGED            : peer-to-peer vif change completed on dongle.
 * @P2P_STATUS_ACTION_TX_COMPLETED   : action frame tx completed.
 * @P2P_STATUS_ACTION_TX_NOACK       : action frame tx not acked.
 * @P2P_STATUS_GO_NEG_PHASE          : P2P GO negotiation ongoing.
 * @P2P_STATUS_DISCOVER_LISTEN       : P2P listen, remaining on channel.
 * @P2P_STATUS_SENDING_ACT_FRAME     : In the process of sending action frame.
 * @P2P_STATUS_WAITING_NEXT_AF_LISTEN: extra listen time for af tx.
 * @P2P_STATUS_WAITING_NEXT_ACT_FRAME: waiting for action frame response.
 * @P2P_STATUS_FINDING_COMMON_CHANNEL: search channel for AF active.
 */
enum wland_p2p_status {
	P2P_STATUS_ENABLED,
	P2P_STATUS_IF_ADD,
	P2P_STATUS_IF_DEL,
	P2P_STATUS_IF_DELETING,
	P2P_STATUS_IF_CHANGING,
	P2P_STATUS_IF_CHANGED,
	P2P_STATUS_ACTION_TX_COMPLETED,
	P2P_STATUS_ACTION_TX_NOACK,
	P2P_STATUS_GO_NEG_PHASE,
	P2P_STATUS_DISCOVER_LISTEN,
	P2P_STATUS_SENDING_ACT_FRAME,
	P2P_STATUS_WAITING_NEXT_AF_LISTEN,
	P2P_STATUS_WAITING_NEXT_ACT_FRAME,
	P2P_STATUS_FINDING_COMMON_CHANNEL
};

/*
 * struct afx_hdl - action frame off channel storage.
 *
 * @afx_work        : worker thread for searching channel
 * @act_frm_scan    : thread synchronizing struct.
 * @is_active       : channel searching active.
 * @peer_chan       : current channel.
 * @is_listen       : sets mode for afx worker.
 * @my_listen_chan  : this peers listen channel.
 * @peer_listen_chan: remote peers listen channel.
 * @tx_dst_addr     : mac address where tx af should be sent to.
 */
struct afx_hdl {
	struct work_struct afx_work;
	struct completion act_frm_scan;
	bool is_active;
	s32 peer_chan;
	bool is_listen;
	u16 my_listen_chan;
	u16 peer_listen_chan;
	u8 tx_dst_addr[ETH_ALEN];
};

/*
 * struct wland_p2p_info - p2p specific driver information.
 *
 * @cfg                     : driver private data for cfg80211 interface.
 * @status                  : status of P2P (see enum wland_p2p_status).
 * @dev_addr                : P2P device address.
 * @int_addr                : P2P interface address.
 * @bss_idx                 : informate for P2P bss types.
 * @listen_timer            : timer for @WL_P2P_DISC_ST_LISTEN discover state.
 * @ssid                    : ssid for P2P GO.
 * @listen_channel          : channel for @WL_P2P_DISC_ST_LISTEN discover state.
 * @remain_on_channel       : contains copy of struct used by cfg80211.
 * @remain_on_channel_cookie: cookie counter for remain on channel cmd
 * @next_af_subtype         : expected action frame subtype.
 * @send_af_done            : indication that action frame tx is complete.
 * @afx_hdl                 : action frame search handler info.
 * @af_sent_channel         : channel action frame is sent.
 * @af_tx_sent_jiffies      : jiffies time when af tx was transmitted.
 * @wait_next_af            : thread synchronizing struct.
 * @gon_req_action          : about to send go negotiation requets frame.
 * @block_gon_req_tx        : drop tx go negotiation requets frame.
 */
struct wland_p2p_info {
	struct wland_cfg80211_info *cfg;
	ulong status;
	u8 dev_addr[ETH_ALEN];
	u8 int_addr[ETH_ALEN];
	struct p2p_bss bss_idx[P2PAPI_BSSCFG_MAX];
	struct timer_list listen_timer;
	struct wland_ssid ssid;
	u8 listen_channel;
	struct ieee80211_channel remain_on_channel;
	enum nl80211_channel_type remain_on_chan_type;
	u32 remain_on_channel_cookie;
	u8 next_af_subtype;
	struct completion send_af_done;
	struct afx_hdl afx_hdl;
	u32 af_sent_channel;
	u32 af_tx_sent_jiffies;
	struct completion wait_next_af;
	bool gon_req_action;
	bool block_gon_req_tx;
};

#ifdef WLAND_P2P_SUPPORT
extern s32 cfg80211_p2p_attach(struct wland_cfg80211_info *cfg);
extern void cfg80211_p2p_detach(struct wland_p2p_info *p2p);

/*
 * wland_p2p_add_vif() - create a new P2P virtual interface.
 *
 * @wiphy   : wiphy device of new interface.
 * @name    : name of the new interface.
 * @type    : nl80211 interface type.
 * @flags   : not used.
 * @params  : contains mac address for P2P device.
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0)
extern struct wireless_dev *
#else /*LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0) */
extern struct net_device *
#endif				/*LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0) */

wland_p2p_add_vif(struct wiphy *wiphy,
	const char *name,
	enum nl80211_iftype type, u32 * flags, struct vif_params *params);

extern s32 wland_p2p_del_vif(struct wiphy *wiphy, struct wireless_dev *wdev);
extern s32 wland_p2p_ifchange(struct wland_cfg80211_info *cfg,
	enum wland_fil_p2p_if_types if_type);
extern s32 wland_p2p_scan_prep(struct wiphy *wiphy,
	struct cfg80211_scan_request *request, struct wland_cfg80211_vif *vif);

extern void wland_p2p_cancel_remain_on_channel(struct wland_if *ifp);

extern s32 cfg80211_p2p_start_device(struct wiphy *wiphy,
	struct wireless_dev *wdev);
extern void cfg80211_p2p_stop_device(struct wiphy *wiphy,
	struct wireless_dev *wdev);

extern int cfg80211_p2p_remain_on_channel(struct wiphy *wiphy,
#if    LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	struct wireless_dev *wdev,
#else				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct net_device *dev,
#endif				/*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */
	struct ieee80211_channel *channel,
	enum nl80211_channel_type channel_type,
	unsigned int duration, u64 * cookie);

/* p2p event callback */
extern s32 notify_p2p_listen_complete(struct wland_if *ifp,
	const struct wland_event_msg *e, void *data);
extern s32 notify_p2p_rx_mgmt_probereq(struct wland_if *ifp,
	const struct wland_event_msg *e, void *data);
extern s32 notify_p2p_action_frame_rx(struct wland_if *ifp,
	const struct wland_event_msg *e, void *data);
extern s32 notify_p2p_action_tx_complete(struct wland_if *ifp,
	const struct wland_event_msg *e, void *data);

/* p2p action */
extern bool wland_p2p_send_action_frame(struct wland_cfg80211_info *cfg,
	struct net_device *ndev, struct wland_fil_af_params_le *af_params);
extern bool wland_p2p_scan_finding_common_channel(struct wland_cfg80211_info
	*cfg, struct wland_bss_info_le *bi);

#endif /* WLAND_P2P_SUPPORT */
#endif /* _WLAND_P2P_H_     */
