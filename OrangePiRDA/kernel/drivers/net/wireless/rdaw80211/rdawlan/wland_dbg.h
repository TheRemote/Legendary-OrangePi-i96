
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

#ifndef _WLAND_DBG_H_
#define _WLAND_DBG_H_

enum {
	WLAND_ERROR_LEVEL = 0,
	WLAND_WARNING_LEVEL = 1,
	WLAND_NOTICE_LEVEL = 2,
	WLAND_INFO_LEVEL = 3,
	WLAND_DEBUG_LEVEL = 4,
	WLAND_TRACE_LEVEL = 5,
};

#define WLAND_TRAP_VAL	            BIT0
#define WLAND_EVENT_VAL	            BIT1
#define WLAND_DCMD_VAL	            BIT2
#define WLAND_WEXT_VAL	            BIT3
#define WLAND_DEFAULT_VAL           BIT4
#define WLAND_SDIO_VAL              BIT5
#define WLAND_USB_VAL               BIT6
#define WLAND_CFG80211_VAL          BIT7
#define WLAND_BUS_VAL               BIT8
#define WLAND_DATA_VAL	            BIT9
#define WLAND_UNMASK_VAL            ((int)(0xFFFF))

#define WLAND_TX_CTRL_AREA				BIT0
#define WLAND_TX_MSDU_AREA				BIT1
#define WLAND_RX_WIDRSP_AREA			BIT2
#define WLAND_RX_MACSTAT_AREA			BIT3
#define WLAND_RX_NETINFO_AREA			BIT4
#define WLAND_RX_MSDU_AREA				BIT5
#define WLAND_RX_NETEVENT_AREA			BIT6
#define WLAND_ALL_AREA					(0xFFFFFFFF)
#define WLAND_NONE_AREA					(0)

#define TX_CTRL					(wland_dump_area & WLAND_TX_CTRL_AREA)
#define TX_MSDU					(wland_dump_area & WLAND_TX_MSDU_AREA)
#define RX_WIDRSP				(wland_dump_area & WLAND_RX_WIDRSP_AREA)
#define RX_MACSTAT				(wland_dump_area & WLAND_RX_MACSTAT_AREA)
#define RX_NETINFO				(wland_dump_area & WLAND_RX_NETINFO_AREA)
#define RX_NETEVENT				(wland_dump_area & WLAND_RX_NETEVENT_AREA)
#define RX_MSDU					(wland_dump_area & WLAND_RX_MSDU_AREA)

#define MACDBG                      "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STRDBG(ea)              (ea)[0], (ea)[1], (ea)[2], (ea)[3], (ea)[4], (ea)[5]

/* Macro for error messages. net_ratelimit() is used when driver
 * debugging is not selected. When debugging the driver error
 * messages are as important as other tracing or even more so.
 */
#ifdef DEBUG
#define WLAND_ERR(fmt, ...)	        pr_err("[RDAWLAN_ERR]:<%s,%d>: " fmt, __func__, __LINE__, ##__VA_ARGS__)
#else /* defined(DEBUG) */
#define WLAND_ERR(fmt, ...)         do {\
                    					if (net_ratelimit()){\
                    						pr_err("[RDAWLAN_ERR]:<%s,%d>: " fmt, __func__, __LINE__, ##__VA_ARGS__);\
                    					}                                                                            \
                    				} while (0)
#endif /* defined(DEBUG) */

#ifdef DEBUG
#define WLAND_DBG(area, level, fmt, ...)  do {\
										int dgb_area = WLAND_##area##_VAL & wland_dbg_area;\
                                        int dbg_level = WLAND_##level##_LEVEL;\
                                        if(dgb_area && (dbg_level <= wland_dbg_level)){\
                                    	    pr_err("%s:<%s,%d>  " fmt, wland_dbgarea(dgb_area), __func__, __LINE__, ##__VA_ARGS__);\
                                    	}                                                                           \
                                    } while (0)
#else /* defined(DEBUG) */
#define WLAND_DBG(area, level, fmt, ...)  no_printk(fmt, ##__VA_ARGS__)
#endif /* defined(DEBUG) */

#define WLAND_DUMP(area, data, len, fmt, ...)\
                                    do {\
                                        if (area){\
                                            pr_err("[RDAWLAN_DUMP]:<%s,%d>: " fmt, __func__, __LINE__, ##__VA_ARGS__);\
                                            wland_dbg_hex_dump(area, data, len, fmt, ##__VA_ARGS__);\
                                        }\
                                    } while (0)

/* hold counter variables used in wlanfmac sdio driver. */
struct wland_sdio_count {
	uint intrcount;		/* Count of device interrupt callbacks */
	uint lastintrs;		/* Count as of last watchdog timer */
	uint pollcnt;		/* Count of active polls */
	uint regfails;		/* Count of R_REG failures */
	uint tx_sderrs;		/* Count of tx attempts with sd errors */
	uint fcqueued;		/* Tx packets that got queued */
	uint rxrtx;		/* Count of rtx requests (NAK to dongle) */
	uint rx_toolong;	/* Receive frames too long to receive */
	uint rxc_errors;	/* SDIO errors when reading control frames */
	uint rx_hdrfail;	/* SDIO errors on header reads */
	uint rx_badhdr;		/* Bad received headers (roosync?) */
	uint rx_badseq;		/* Mismatched rx sequence number */
	uint fc_rcvd;		/* Number of flow-control events received */
	uint fc_xoff;		/* Number which turned on flow-control */
	uint fc_xon;		/* Number which turned off flow-control */
	uint f2rxhdrs;		/* Number of header reads */
	uint f2rxdata;		/* Number of frame data reads */
	uint f2txdata;		/* Number of f2 frame writes */
	uint f1regdata;		/* Number of f1 register accesses */
	uint tickcnt;		/* Number of watchdog been schedule */
	ulong tx_ctlerrs;	/* Err of sending ctrl frames */
	ulong tx_ctlpkts;	/* Ctrl frames sent to dongle */
	ulong rx_ctlerrs;	/* Err of processing rx ctrl frames */
	ulong rx_ctlpkts;	/* Ctrl frames processed from dongle */
	ulong rx_readahead_cnt;	/* packets where header read-ahead was used */
};

struct wland_fws_stats {
	u32 tlv_parse_failed;
	u32 tlv_invalid_type;
	u32 header_only_pkt;
	u32 header_pulls;
	u32 pkt2bus;
	u32 send_pkts[5];
	u32 requested_sent[5];
	u32 generic_error;
	u32 mac_update_failed;
	u32 mac_ps_update_failed;
	u32 if_update_failed;
	u32 packet_request_failed;
	u32 credit_request_failed;
	u32 rollback_success;
	u32 rollback_failed;
	u32 delayq_full_error;
	u32 supprq_full_error;
	u32 txs_indicate;
	u32 txs_discard;
	u32 txs_supp_core;
	u32 txs_supp_ps;
	u32 txs_tossed;
	u32 txs_host_tossed;
	u32 bus_flow_block;
	u32 fws_flow_block;
};

struct wland_sdio;
struct wland_private;

extern int wland_dbg_area;
extern int wland_dbg_level;
extern int wland_dump_area;

extern char *wland_dbgarea(int dbg_flags);

/* setup bug info dir */
extern void wland_debugfs_init(void);
extern void wland_debugfs_exit(void);

extern void wland_sdio_debugfs_create(struct wland_private *drvr);
extern int wland_debugfs_attach(struct wland_private *drvr);
extern void wland_debugfs_detach(struct wland_private *drvr);

#endif /* _WLAND_DBG_H_ */
