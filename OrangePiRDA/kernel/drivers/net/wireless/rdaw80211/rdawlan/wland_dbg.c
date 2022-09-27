
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
#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <linux/debugfs.h>
#include <net/cfg80211.h>

#include <wland_defs.h>
#include <wland_fweh.h>
#include <wland_dev.h>
#include <wland_bus.h>
#include <wland_dbg.h>

/* Error Debug Area Bit Map */
int wland_dbg_area = WLAND_DATA_VAL |
	WLAND_TRAP_VAL |
	WLAND_DCMD_VAL |
	WLAND_EVENT_VAL |
	WLAND_BUS_VAL |
	WLAND_WEXT_VAL |
	WLAND_DEFAULT_VAL | WLAND_SDIO_VAL | WLAND_CFG80211_VAL;

int wland_dbg_level = WLAND_INFO_LEVEL;
int wland_dump_area =
	//WLAND_TX_CTRL_AREA                    |
	//WLAND_TX_MSDU_AREA                   |
	//WLAND_RX_WIDRSP_AREA          |
	//WLAND_RX_MACSTAT_AREA         |
	//WLAND_RX_NETINFO_AREA         |
	//WLAND_RX_MSDU_AREA                   |
	//WLAND_RX_NETEVENT_AREA          |
	WLAND_NONE_AREA;

/* Set Default Debug Dir */
static struct dentry *root_folder = NULL;

static ssize_t wland_debugfs_sdio_counter_read(struct file *f,
	char __user * data, size_t count, loff_t * ppos)
{
	struct wland_sdio_count *sdcnt = f->private_data;
	int buf_size = 750;
	int res;
	int ret;
	char *buf;

	/*
	 * only allow read from start
	 */
	if (*ppos > 0)
		return 0;

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (buf == NULL) {
		WLAND_ERR("kmalloc buf failed\n");
		return -ENOMEM;
	}


	res = scnprintf(buf, buf_size,
		"intrcount:    %u\nlastintrs:    %u\n"
		"pollcnt:      %u\nregfails:     %u\n"
		"tx_sderrs:    %u\nfcqueued:     %u\n"
		"rxrtx:        %u\nrx_toolong:   %u\n"
		"rxc_errors:   %u\nrx_hdrfail:   %u\n"
		"rx_badhdr:    %u\nrx_badseq:    %u\n"
		"fc_rcvd:      %u\nfc_xoff:      %u\n"
		"fc_xon:       %u\n                  "
		"f2rxhdrs:     %u\nf2rxdata:     %u\n"
		"f2txdata:     %u\nf1regdata:    %u\n"
		"tickcnt:      %u\ntx_ctlerrs:   %lu\n"
		"tx_ctlpkts:   %lu\nrx_ctlerrs:   %lu\n"
		"rx_ctlpkts:   %lu\nrx_readahead: %lu\n",
		sdcnt->intrcount, sdcnt->lastintrs,
		sdcnt->pollcnt, sdcnt->regfails,
		sdcnt->tx_sderrs, sdcnt->fcqueued,
		sdcnt->rxrtx, sdcnt->rx_toolong,
		sdcnt->rxc_errors, sdcnt->rx_hdrfail,
		sdcnt->rx_badhdr, sdcnt->rx_badseq,
		sdcnt->fc_rcvd, sdcnt->fc_xoff,
		sdcnt->fc_xon,
		sdcnt->f2rxhdrs, sdcnt->f2rxdata,
		sdcnt->f2txdata, sdcnt->f1regdata,
		sdcnt->tickcnt, sdcnt->tx_ctlerrs,
		sdcnt->tx_ctlpkts, sdcnt->rx_ctlerrs,
		sdcnt->rx_ctlpkts, sdcnt->rx_readahead_cnt);

	ret = simple_read_from_buffer(data, count, ppos, buf, res);
	kfree(buf);
	return ret;
}

static const struct file_operations wland_debugfs_sdio_counter_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = wland_debugfs_sdio_counter_read
};

static ssize_t wland_debugarea_read(struct file *file, char __user * userbuf,
	size_t count, loff_t * ppos)
{
	size_t pos = 0;
	u32 addr = get_zeroed_page(GFP_KERNEL);
	char *buf = (char *) addr;
	ssize_t res;

	WLAND_DBG(DEFAULT, TRACE, "get debug_area:0x%x\n", wland_dbg_area);

	pos += snprintf(buf + pos, PAGE_SIZE - pos, "%x\n", wland_dbg_area);

	res = simple_read_from_buffer(userbuf, count, ppos, buf, pos);

	free_page(addr);
	return res;
}

static ssize_t wland_debugarea_write(struct file *file,
	const char __user * user_buf, size_t count, loff_t * ppos)
{
	ssize_t ret;
	int debug_area;
	u32 addr = get_zeroed_page(GFP_KERNEL);
	char *buf = (char *) addr;

	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	if (copy_from_user(buf, user_buf, count)) {
		ret = -EFAULT;
		goto out_unlock;
	}

	ret = sscanf(buf, "%x", &debug_area);
	if (ret != 1) {
		ret = -EINVAL;
		goto out_unlock;
	}

	wland_dbg_area = debug_area;

	WLAND_DBG(DEFAULT, TRACE, "set debug_area = 0x%x\n", wland_dbg_area);

	ret = count;
out_unlock:
	free_page(addr);
	return ret;
}

static const struct file_operations wland_dbgarea_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = wland_debugarea_read,
	.write = wland_debugarea_write
};

static ssize_t wland_debuglevel_read(struct file *file, char __user * userbuf,
	size_t count, loff_t * ppos)
{
	size_t pos = 0;
	u32 addr = get_zeroed_page(GFP_KERNEL);
	char *buf = (char *) addr;
	ssize_t res;

	WLAND_DBG(DEFAULT, TRACE, "get debug_level:%d\n", wland_dbg_level);

	pos += snprintf(buf + pos, PAGE_SIZE - pos, "%d\n", wland_dbg_level);

	res = simple_read_from_buffer(userbuf, count, ppos, buf, pos);

	free_page(addr);
	return res;
}

static ssize_t wland_debuglevel_write(struct file *file,
	const char __user * user_buf, size_t count, loff_t * ppos)
{
	ssize_t ret;
	int debug_level;
	u32 addr = get_zeroed_page(GFP_KERNEL);
	char *buf = (char *) addr;

	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	if (copy_from_user(buf, user_buf, count)) {
		ret = -EFAULT;
		goto out_unlock;
	}

	ret = sscanf(buf, "%d", &debug_level);
	if (ret != 1) {
		ret = -EINVAL;
		goto out_unlock;
	}

	wland_dbg_level = debug_level;

	WLAND_DBG(DEFAULT, TRACE, "set debug_level = %d\n", wland_dbg_level);

	ret = count;
out_unlock:
	free_page(addr);
	return ret;
}

static const struct file_operations wland_dbglevel_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = wland_debuglevel_read,
	.write = wland_debuglevel_write
};

#ifdef DEBUG
static ssize_t wland_sdio_forensic_read(struct file *f, char __user * data,
	size_t count, loff_t * ppos)
{
	//wland_private *drvr = f->private_data;
	int res = 0;

	//res = brcmf_sdio_trap_info(bus, &sh, data, count);

	if (res > 0)
		*ppos += res;
	return (ssize_t) res;
}

static const struct file_operations sdio_forensic_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = wland_sdio_forensic_read
};
#endif /* DEBUG */

void wland_sdio_debugfs_create(struct wland_private *drvr)
{
#ifdef DEBUG
	struct dentry *dentry = drvr->dbgfs_dir;;

	if (IS_ERR_OR_NULL(dentry))
		return;

	debugfs_create_file("forensics", S_IRUGO, dentry, drvr,
		&sdio_forensic_ops);

	debugfs_create_file("counters", S_IRUGO, dentry, drvr,
		&wland_debugfs_sdio_counter_ops);

	debugfs_create_file("dbglevel", S_IRUGO, dentry, drvr,
		&wland_dbglevel_ops);

	debugfs_create_file("dbgarea", S_IRUGO, dentry, drvr,
		&wland_dbgarea_ops);
#endif /* DEBUG */
}

char *wland_dbgarea(int dbg_flags)
{
	switch (dbg_flags) {
	case WLAND_TRAP_VAL:
		return "[RDAWLAN_TRAP]";
	case WLAND_EVENT_VAL:
		return "[RDAWLAN_EVENT]";
	case WLAND_DCMD_VAL:
		return "[RDAWLAN_DCMD]";
	case WLAND_WEXT_VAL:
		return "[RDAWLAN_WEXT]";
	case WLAND_DEFAULT_VAL:
		return "[RDAWLAN_DEFAULT]";
	case WLAND_SDIO_VAL:
		return "[RDAWLAN_SDIO]";
	case WLAND_USB_VAL:
		return "[RDAWLAN_USB]";
	case WLAND_CFG80211_VAL:
		return "[RDAWLAN_CFG80211]";
	case WLAND_BUS_VAL:
		return "[RDAWLAN_BUS]";
	case WLAND_DATA_VAL:
		return "[RDAWLAN_DATA]";
	default:
		return "[RDAWLAN_UNKNOW]";
	}
}

/* dbg attach */
int wland_debugfs_attach(struct wland_private *drvr)
{
	struct device *dev = drvr->bus_if->dev;

	WLAND_DBG(DEFAULT, TRACE, "Enter.\n");

	if (!root_folder)
		return -ENODEV;

	drvr->dbgfs_dir = debugfs_create_dir(dev_name(dev), root_folder);

	if (!IS_ERR_OR_NULL(drvr->dbgfs_dir))
		return -ENODEV;

	WLAND_DBG(DEFAULT, TRACE, "Done.\n");

	return 0;
}

/* dbg dettach */
void wland_debugfs_detach(struct wland_private *drvr)
{
	WLAND_DBG(DEFAULT, TRACE, "Enter\n");

	if (!IS_ERR_OR_NULL(drvr->dbgfs_dir))
		debugfs_remove_recursive(drvr->dbgfs_dir);

	WLAND_DBG(DEFAULT, TRACE, "Done\n");
}

/* dbg dir init */
void wland_debugfs_init(void)
{
	WLAND_DBG(DEFAULT, TRACE, "Enter\n");
	root_folder = debugfs_create_dir(KBUILD_MODNAME, NULL);

	if (IS_ERR(root_folder))
		root_folder = NULL;
	WLAND_DBG(DEFAULT, TRACE, "Done\n");
}

/* dbg dir exit */
void wland_debugfs_exit(void)
{
	WLAND_DBG(DEFAULT, TRACE, "Enter\n");
	if (!root_folder)
		return;

	debugfs_remove_recursive(root_folder);

	root_folder = NULL;
	WLAND_DBG(DEFAULT, TRACE, "Done\n");
}
