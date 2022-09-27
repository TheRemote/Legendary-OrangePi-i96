
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
#ifndef _WLAND_DEFS_H_
#define _WLAND_DEFS_H_

#include <linux/types.h>
#include <linux/if_ether.h>

/****************************************************************************
                        Wlan Const Defines
 ****************************************************************************/

/* init_scan support */
//#define WLAND_INIT_SCAN_SUPPORT

/* Driver Version Sync With Source Server */
#define WLAND_VER_MAJ                           5
#define WLAND_VER_MIN                           9
#define WLAND_VER_BLD                           95

/* define for chip version */
enum WLAND_CHIP_VERSION {
	WLAND_VER_90_D = 1,
	WLAND_VER_90_E = 2,
	WLAND_VER_91 = 3,
	WLAND_VER_91_E = 4,
	WLAND_VER_91_F = 5,
	WLAND_VER_91_G = 6,
	WLAND_VER_MAX = 10
};

#define WLAND_VERSION_STR		                "9.59.95.5"
#define CHIP_ID_MASK                            (0x1F)

/* SDIO Device ID */
#define SDIO_VENDOR_ID_RDAWLAN		            0x5449
#define SDIO_DEVICE_ID_RDA599X      	        0x0145

/* USB  Device ID */
#define USB_VENDOR_ID_RDAMICRO	                0x1E04
#define USB_DEVICE_ID_RDA599X	                0x8888
#define USB_DEVICE_ID_BCMFW	                    0x0BDC

#define WIFI_MAC_ACTIVATED_FLAG    				0x5990

#ifndef TRUE
#define TRUE            (1)
#endif

/*
#ifndef FALSE
#define FALSE           (0)
#endif
*/
#ifndef NULL
#define NULL                                    ((void*)0)
#endif

/* Support BUS TYPE */
#define SDIO_BUS		                        1	/* SDIO target */
#define USB_BUS			                        2	/* USB  target */

/* bit mask */
#define BIT9                                    (1 << 9)
#define BIT8                                    (1 << 8)
#define BIT7                                    (1 << 7)
#define BIT6                                    (1 << 6)
#define BIT5                                    (1 << 5)
#define BIT4                                    (1 << 4)
#define BIT3                                    (1 << 3)
#define BIT2                                    (1 << 2)
#define BIT1                                    (1 << 1)
#define BIT0                                    (1 << 0)

#define CLEAR_BIT(X , Y)                        (X) &= (~(Y))
#define SET_BIT(X , Y)                          (X) |= (Y)

/* Values for PM */
#define	OFF	                                    0
#define	ON	                                    1	/* ON = 1    */
#define	AUTO	                                (-1)	/* Auto = -1 */

#define ACTIVE_SCAN_TIME			            10
#define PASSIVE_SCAN_TIME			            1200
#define MIN_SCAN_TIME				            10
#define MAX_SCAN_TIME				            1200
#define DEFAULT_SCAN				            0
#define USER_SCAN					            BIT0

/* Return Results */
#define STATUS_SUCCESS                          (1)
#define STATUS_TIMEOUT			                (2)
#define STATUS_ABORTED      	                (3)
#define STATUS_FAILED                           (4)
#define STATUS_NO_NETWORKS                      (5)

#define MAC_CONNECTED                           (1)
#define MAC_DISCONNECTED                        (0)

/* Priority definitions according 802.1D */
#define	PRIO_8021D_NONE		                    2
#define	PRIO_8021D_BK		                    1
#define	PRIO_8021D_BE		                    0
#define	PRIO_8021D_EE		                    3
#define	PRIO_8021D_CL		                    4
#define	PRIO_8021D_VI		                    5
#define	PRIO_8021D_VO		                    6
#define	PRIO_8021D_NC		                    7

#define	MAXPRIO			                        7
#define NUMPRIO			                        (MAXPRIO + 1)

/* Bit masks for radio disabled status - returned by WL_GET_RADIO */
#define RADIO_SW_DISABLE		                (1<<0)
#define RADIO_HW_DISABLE		                (1<<1)

/* some countries don't support any channel */
#define RADIO_COUNTRY_DISABLE	                (1<<3)

/* Override bit for SET_TXPWR.  if set, ignore other level limits */
#define TXPWR_OVERRIDE	                        (1U<<31)

/* band types */
#define	WLAND_BAND_AUTO		                    0	/* auto-select */
#define	WLAND_BAND_5G		                    1	/* 5 Ghz */
#define	WLAND_BAND_2G		                    2	/* 2.4 Ghz */
#define	WLAND_BAND_ALL		                    3	/* all bands */

#ifndef WLAN_EID_GENERIC
#define WLAN_EID_GENERIC                        0xDD
#endif

/* define for debug information */
#define MAX_HEX_DUMP_LEN	                    64

#define ALL_INTERFACES	                        0xFF

#define IOCTL_RESP_TIMEOUT                      (5*1000)

/* scan relation timeout  */
#define SCAN_CHANNEL_TIME		                102	/* ms */
#define SCAN_ACTIVE_TIME		                102	/* ms */
#define SCAN_PASSIVE_TIME		                102	/* ms */

#define SKIP_REPORT_CHANNEL_14

// Low snr agc setting
// #define CHINA_VERSION

/* CDC flag definitions */
#define CDC_DCMD_LEN_MASK	                    0x0FFF	/* id an cmd pairing */
#define CDC_DCMD_LEN_SHIFT	                    12	/* ID Mask shift bits */

/****************************************************************************
                        Wlan Features Support
 ****************************************************************************/
#ifndef DEBUG
#define DEBUG
#endif

/*
 * WLAND_BSSCACHE_SUPPORT     : Cache bss list
 * WLAND_RSSIAVG_SUPPORT      : Average RSSI of BSS list
 * WLAND_RSSIOFFSET_SUPPORT   : RSSI offset
 */
#define WLAND_BSSCACHE_SUPPORT
#define WLAND_RSSIAVG_SUPPORT
#define WLAND_RSSIOFFSET_SUPPORT

#ifdef WLAND_RSSIOFFSET_SUPPORT
#define WLAND_RSSI_MAXVAL_FOR_OFFSET	236
#define WLAND_RSSI_OFFSET	                12
#endif

/* define support FPGA mode */
//#define WLAND_FPGA_SUPPORT

/* define for use random mac address  */
#define WLAND_MACADDR_DYNAMIC
#define USE_MAC_FROM_RDA_NVRAM

/* define support cfg80211 or wext mode */
#define WLAND_CFG80211_SUPPORT
//#define WLAND_WEXT_SUPPORT

/* define support wapi sec mode */
//#define WLAND_WAPI_SUPPORT

#define WLAND_SDIO_SUPPORT

/* define support p2p mode */
//#define WLAND_P2P_SUPPORT

/*define for power manager*/
#define WLAND_POWER_MANAGER

/*define for flow ctrl*/
#define WLAND_SDIO_FC_SUPPORT

/* define for chip patch */
//#define NORMAL_FIXED
#define WLAN_BIG_CURRENT_90E

/* rda platform sdio should 2^n alligen */
#define WLAND_RDAPLATFORM_SUPPORT

/* define for support 5G rf,default 2.4G */
//#define WLAND_5GRF_SUPPORT

#define CARD_ENTER_SLEEP_TIMER                  (200)
#define FLOW_CTRL_INT_SLEEP_RETRY_COUNT_91      (25)
#define FLOW_CTRL_RXCMPL_RETRY_COUNT_91         (30)
#define FLOW_CTRL_RXCMPL_RETRY_COUNT_90         (2000)

#define DEFAULT_MAX_SCAN_AGE                    (15*HZ)

#define WID_HEADER_LEN                          (2)

/* Space for header read, limit for data packets */
#ifdef WLAND_SDIO_SUPPORT
#define WLAND_MAX_BUFSZ                         2048	/* Maximum size of a sdio dma buffer */
#else /*WLAND_SDIO_SUPPORT */
#define WLAND_MAX_BUFSZ                         1660	/* Maximum size of a sdio dma buffer */
#endif /*WLAND_SDIO_SUPPORT */

/* Driver Features Config */
#define WLAND_SLEEP_ENABLE                      BIT0
#define WLAND_SLEEP_PREASSO                     BIT1

/* Mac Listen Interval */
#define WIFI_LISTEN_INTERVAL                    0x06

/* Link Loss Threshold */
#define WIFI_LINK_LOSS_THRESHOLD_90                0x20
#define WIFI_LINK_LOSS_THRESHOLD_91                0x40

/* Link Sleep Threashold,old Value: 0x00A00080 */
#define WIFI_PREASSO_SLEEP                      0x000500FF

/* max sequential rxcntl timeouts to set HANG event */
#ifndef MAX_CNTL_TIMEOUT
#define MAX_CNTL_TIMEOUT                        2
#endif

/*BT WIFI CONEXIST*/
#define BT_COEXIST  SIOCDEVPRIVATE + 2
#define BT_STATE_SCO_ON  0x01
#define BT_STATE_SCO_OFF  0x02
#define BT_STATE_SCO_ONGOING 0x04
#define BT_STATE_A2DP_PLAYING  0x08
#define BT_STATE_A2DP_NO_PLAYING 0x10
#define BT_STATE_CONNECTION_ON 0x20
#define BT_STATE_CONNECTION_OFF 0x40

/*BT WIFI CONEXIST*/

/*get mac from rda nvram*/
struct wlan_mac_info {
	u16 activated;
	u8 mac_addr[ETH_ALEN];
};

#endif /* _WLAND_DEFS_H_ */
