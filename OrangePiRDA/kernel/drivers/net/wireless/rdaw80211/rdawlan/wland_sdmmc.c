
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
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/rtnetlink.h>
#include <linux/etherdevice.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/ethtool.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/ieee80211.h>
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
#include <wland_trap.h>
#include <wland_cfg80211.h>

/* devices we support, null terminated */
static const struct sdio_device_id wland_sdmmc_ids[] = {
	{SDIO_DEVICE(SDIO_VENDOR_ID_RDAWLAN, SDIO_DEVICE_ID_RDA599X)},
	{ /* end: all zeroes */ },
};

MODULE_DEVICE_TABLE(sdio, wland_sdmmc_ids);

bool wland_pm_resume_error(struct wland_sdio_dev *sdiodev)
{
	bool is_err = false;

#if 0
#ifdef CONFIG_PM_SLEEP
	is_err = atomic_read(&sdiodev->suspend);
#endif /* CONFIG_PM_SLEEP */
#endif
	return is_err;
}

void wland_pm_resume_wait(struct wland_sdio_dev *sdiodev,
	wait_queue_head_t * wq)
{
#if 0
#ifdef CONFIG_PM_SLEEP
	int retry = 0;

	while (atomic_read(&sdiodev->suspend) && retry++ != 30)
		wait_event_timeout(*wq, false, HZ / 100);
#endif /* CONFIG_PM_SLEEP */
#endif
}

int sdioh_request_byte(struct wland_sdio_dev *sdiodev, uint rw, uint regaddr,
	u8 * byte)
{
	int err_ret;

	WLAND_DBG(SDIO, TRACE, "rw=%d,addr=0x%05x\n", rw, regaddr);

	wland_pm_resume_wait(sdiodev, &sdiodev->request_byte_wait);

	if (wland_pm_resume_error(sdiodev))
		return -EIO;

	//sdio_claim_host(sdiodev->func);
	if (SDIOH_WRITE == rw)	/* CMD52 Write */
		sdio_writeb(sdiodev->func, *byte, regaddr, &err_ret);
	else
		*byte = sdio_readb(sdiodev->func, regaddr, &err_ret);
	//sdio_release_host(sdiodev->func);

	if (err_ret)
		WLAND_ERR("Failed to %s :@0x%05x=%02x,Err: %d.\n",
			rw ? "write" : "read", regaddr, *byte, err_ret);

	return err_ret;
}

int sdioh_request_word(struct wland_sdio_dev *sdiodev, uint rw, uint addr,
	u32 * word, uint nbytes)
{
	int err_ret = -EIO;

	WLAND_DBG(SDIO, TRACE, "rw=%d, addr=0x%05x, nbytes=%d\n", rw, addr,
		nbytes);

	wland_pm_resume_wait(sdiodev, &sdiodev->request_word_wait);

	if (wland_pm_resume_error(sdiodev))
		return -EIO;

	sdio_claim_host(sdiodev->func);
	if (SDIOH_WRITE == rw) {	/* CMD52 Write */
		if (nbytes == 4)
			sdio_writel(sdiodev->func, *word, addr, &err_ret);
		else if (nbytes == 2)
			sdio_writew(sdiodev->func, (*word & 0xFFFF), addr,
				&err_ret);
		else
			WLAND_ERR("Invalid nbytes: %d\n", nbytes);
	} else {		/* CMD52 Read */
		if (nbytes == 4)
			*word = sdio_readl(sdiodev->func, addr, &err_ret);
		else if (nbytes == 2)
			*word = sdio_readw(sdiodev->func, addr,
				&err_ret) & 0xFFFF;
		else
			WLAND_ERR("Invalid nbytes: %d\n", nbytes);
	}
	sdio_release_host(sdiodev->func);

	if (err_ret)
		WLAND_ERR("Failed to %s word, Err: 0x%08x\n",
			rw ? "write" : "read", err_ret);

	return err_ret;
}

int sdioh_request_bytes(struct wland_sdio_dev *sdiodev, uint rw, uint addr,
	u8 * byte, uint nbyte)
{
	int err_ret = 0;
	int bytes_left = 0, offset = 0, batch = 0;

	WLAND_DBG(SDIO, TRACE, "%s: addr=0x%05x, lenght=%d\n",
		rw ? "WRITE" : "READ", addr, nbyte);

	wland_pm_resume_wait(sdiodev, &sdiodev->request_buffer_wait);

	if (wland_pm_resume_error(sdiodev))
		return -EIO;

	//sdio_claim_host(sdiodev->func);
	if (SDIOH_WRITE == rw) {
		bytes_left = nbyte;
		while (bytes_left > 0 && err_ret == 0) {
			batch = (bytes_left >
				sdiodev->func->cur_blksize) ? sdiodev->func->
				cur_blksize : bytes_left;
#ifdef WLAND_RDAPLATFORM_SUPPORT
			{
				u8 *packet_to_send = NULL;
				struct page *pg = NULL;

				packet_to_send = byte + offset;
				if (((u32) packet_to_send >> PAGE_SHIFT) !=
					(((u32) packet_to_send + batch - 1) >> PAGE_SHIFT) ||
					(u32)packet_to_send & (ALIGNMENT -1)) {

					pg = alloc_page(GFP_KERNEL);
					if (!pg) {
						err_ret = -1;
						break;
					}
					memcpy(page_address(pg), packet_to_send, batch);
					packet_to_send = page_address(pg);
					WLAND_DBG(SDIO, TRACE, "wlan data cross page boundary addr:%x size:%x \n",
						(u32)(packet_to_send), batch);
					err_ret = sdio_writesb(sdiodev->func, addr, packet_to_send, batch);
					__free_page(pg);
				} else
					err_ret = sdio_writesb(sdiodev->func, addr, packet_to_send, batch);
			}

#else
			err_ret =
				sdio_writesb(sdiodev->func, addr, byte + offset,
				batch);
#endif
			offset += batch;
			bytes_left -= batch;
		}
	} else {
		err_ret = sdio_readsb(sdiodev->func, byte, addr, nbyte);
	}
	//sdio_release_host(sdiodev->func);

	if (err_ret)
		WLAND_ERR("Failed to %s bytes, Err: 0x%08x\n",
			(SDIOH_WRITE == rw) ? "write" : "read", err_ret);

	return err_ret;
}

#ifdef WLAND_SDIO_FC_SUPPORT
static int wland_sdio_flow_ctrl_90(struct wland_sdio_dev *sdiodev)
{
	int ret = 0;
	u8 status = 0;
	s32 int_sleep_count = 0, check_num = FLOW_CTRL_RXCMPL_RETRY_COUNT_90;

	WLAND_DBG(SDIO, TRACE, "Enter\n");
	if (sdiodev->bus_if->chip != WLAND_VER_90_D
		&& sdiodev->bus_if->chip != WLAND_VER_90_E) {
		ret = -1;
		WLAND_ERR
			("WIFI chip version not match(sdiodev->bus_if->chip=%d)\n",
			sdiodev->bus_if->chip);
		goto out;
	}

	while (true) {
		sdio_claim_host(sdiodev->func);
		ret = sdioh_request_byte(sdiodev, SDIOH_READ,
			URSDIO_FUNC1_INT_PENDING, &status);
		sdio_release_host(sdiodev->func);
		if (status & I_RXCMPL)
			WLAND_DBG(SDIO, TRACE,
				"URSDIO_FUNC1_INT_PENDING:int_sleep_count=%d, status=%x \n",
				int_sleep_count, status);

		if (ret) {
			WLAND_ERR("wland read URSDIO_FUNC1_INT_PENDING failed......ret = %d\n", ret);
			return ret;
		}

		if ((status & I_RXCMPL) == 0) {
			if (int_sleep_count >= check_num) {
				status = I_RXCMPL;
				sdio_claim_host(sdiodev->func);
				ret = sdioh_request_byte(sdiodev, SDIOH_WRITE,
					URSDIO_FUNC1_INT_PENDING, &status);
				sdio_release_host(sdiodev->func);
				msleep(100);
				WLAND_ERR
					("flows ctrl RXCMPL failed, count:%d over, return back \n",
					check_num);
				break;
			} else {
				int_sleep_count++;
				schedule();
			}
		} else {
			status = I_RXCMPL;
			sdio_claim_host(sdiodev->func);
			ret = sdioh_request_byte(sdiodev, SDIOH_WRITE,
				URSDIO_FUNC1_INT_PENDING, &status);
			sdio_release_host(sdiodev->func);
			WLAND_DBG(SDIO, TRACE,
				"clear flowctrl flag, int_sleep_count=%d\n",
				int_sleep_count);
			if (!ret)
				break;
		}
	}

out:
	WLAND_DBG(SDIO, TRACE, "Done(ret:%d)\n", ret);
	return ret;
}

static int wland_sdio_flow_ctrl_91(struct wland_sdio_dev *sdiodev)
{
	int ret = 0;
	u8 status = 0;
	s32 int_sleep_count = 0, check_num = FLOW_CTRL_INT_SLEEP_RETRY_COUNT_91;

	WLAND_DBG(SDIO, TRACE, "Enter\n");
	if (sdiodev->bus_if->chip != WLAND_VER_91) {
		ret = -1;
		WLAND_ERR("WIFI chip version not match(sdiodev->bus_if->chip=%d)\n",
			sdiodev->bus_if->chip);
		goto out;
	}

	while (true) {
		sdio_claim_host(sdiodev->func);
		ret = sdioh_request_byte(sdiodev, SDIOH_READ,
			URSDIO_FUNC1_INT_PENDING, &status);
		sdio_release_host(sdiodev->func);
		if (status & I_RXCMPL)
			WLAND_DBG(SDIO, TRACE,
				"URSDIO_FUNC1_INT_PENDING:int_sleep_count=%d, status=%x \n",
				int_sleep_count, status);

		if (ret) {
			WLAND_ERR("wland read URSDIO_FUNC1_INT_PENDING failed......ret = %d\n", ret);
			return ret;
		}

		if ((status & I_RXCMPL) == 0) {
			if (int_sleep_count >= check_num) {
				break;
			} else {
				int_sleep_count++;
			}
		} else {
			status = I_SLEEP;
			sdio_claim_host(sdiodev->func);
			ret = sdioh_request_byte(sdiodev, SDIOH_WRITE,
				URSDIO_FUNC1_INT_PENDING, &status);
			sdio_release_host(sdiodev->func);
			WLAND_DBG(SDIO, TRACE,
				"clear flowctrl flag, int_sleep_count=%d\n",
				int_sleep_count);
			if (!ret)
				break;
		}
		if (int_sleep_count < 20)
			udelay(10);
		else
			msleep(1);
	}

out:
	WLAND_DBG(SDIO, TRACE, "Done(ret:%d)\n", ret);
	return ret;
}

static int wland_sdio_flow_ctrl_91e(struct wland_sdio_dev *sdiodev)
{
	int ret = 0;
	u8 status = 0;
	s32 int_sleep_count = 0, check_num = FLOW_CTRL_RXCMPL_RETRY_COUNT_91;

	WLAND_DBG(SDIO, TRACE, "Enter\n");
	if (sdiodev->bus_if->chip != WLAND_VER_91_E
		&& sdiodev->bus_if->chip != WLAND_VER_91_F
		&& sdiodev->bus_if->chip != WLAND_VER_91_G) {
		ret = -1;
		WLAND_ERR
			("WIFI chip version not match(sdiodev->bus_if->chip=%d)\n",
			sdiodev->bus_if->chip);
		goto out;
	}

	while (true) {
		sdio_claim_host(sdiodev->func);
		ret = sdioh_request_byte(sdiodev, SDIOH_READ,
			URSDIO_FUNC1_INT_PENDING, &status);
		sdio_release_host(sdiodev->func);
		if (status & I_RXCMPL)
			WLAND_DBG(SDIO, TRACE,
				"URSDIO_FUNC1_INT_PENDING:int_sleep_count=%d, status=%x \n",
				int_sleep_count, status);

		if (!ret) {
			if ((status & I_RXCMPL) == 0) {
				if (int_sleep_count >= check_num) {
					status = I_RXCMPL;
					sdio_claim_host(sdiodev->func);
					ret = sdioh_request_byte(sdiodev,
						SDIOH_WRITE,
						URSDIO_FUNC1_INT_PENDING,
						&status);
					sdio_release_host(sdiodev->func);
					WLAND_ERR
						("flows ctrl RXCMPL failed, count:%d over, return back \n",
						check_num);
					ret = -1;
					break;
				} else {
					int_sleep_count++;
				}
				if (int_sleep_count < 20) {
					udelay(2);
				} else {
					WLAND_DBG(SDIO, DEBUG,"msleep(1)\n");
					mdelay(1);
				}
			} else {
				status = I_RXCMPL;
				sdio_claim_host(sdiodev->func);
				ret = sdioh_request_byte(sdiodev, SDIOH_WRITE,
					URSDIO_FUNC1_INT_PENDING, &status);
				sdio_release_host(sdiodev->func);
				WLAND_DBG(SDIO, TRACE,
					"clear flowctrl flag, int_sleep_count=%d\n",
					int_sleep_count);
				if (!ret)
					break;
			}

		} else {
			WLAND_ERR("wland read URSDIO_FUNC1_INT_PENDING failed......ret = %d\n", ret);
			return ret;
		}
	}

out:
	WLAND_DBG(SDIO, TRACE, "Done(ret:%d)\n", ret);
	return ret;
}

static int wland_sdio_flow_ctrl(struct wland_sdio_dev *sdiodev)
{
	int ret = 0;

	WLAND_DBG(SDIO, TRACE, "Enter\n");

	if (sdiodev->bus_if->chip == WLAND_VER_90_D
		|| sdiodev->bus_if->chip == WLAND_VER_90_E) {
		if ((ret = wland_sdio_flow_ctrl_90(sdiodev))) {
			WLAND_ERR("wland_sdio_flow_ctrl_90 failed! \n");
			goto out;
		}
	} else if (sdiodev->bus_if->chip == WLAND_VER_91) {
		if ((ret = wland_sdio_flow_ctrl_91(sdiodev))) {
			WLAND_ERR("wland_sdio_flow_ctrl_91 failed! \n");
			goto out;
		}
	} else if (sdiodev->bus_if->chip == WLAND_VER_91_E
		|| sdiodev->bus_if->chip == WLAND_VER_91_F
		|| sdiodev->bus_if->chip == WLAND_VER_91_G) {
		if ((ret = wland_sdio_flow_ctrl_91e(sdiodev))) {
			WLAND_ERR("wland_sdio_flow_ctrl_91e failed! \n");
			goto out;
		}
	} else {
		ret = -1;
		WLAND_ERR("wlan_sdio_flow_ctrl unkown version:%d\n",
			sdiodev->bus_if->chip);
	}

out:
	WLAND_DBG(SDIO, TRACE, "Done(ret:%d)\n", ret);
	return ret;
}
#endif /*WLAND_SDIO_FC_SUPPORT */

int wland_sdio_send_pkt(struct wland_sdio *bus, struct sk_buff *pkt, uint count)
{
	int ret = 0;
	u8 size_l = 0, size_h = 0;
	u16 size = 0;
	u8 *buf = pkt->data;

	WLAND_DBG(SDIO, TRACE, "blockSize=%d, count=%d, pkt->len=%d, pkt->data=%p\n",
		bus->blocksize, count, pkt->len, pkt->data);
	//WLAND_DUMP(SDIO, pkt->data, count, "TX Data, len:%Zu\n", count);
	if (check_test_mode() && check_sdio_init()) {
		WLAND_DBG(SDIO, INFO, "In Test Mode and do not send pkt!\n");
		return ret;
	}
#ifdef WLAND_SDIO_FC_SUPPORT
	if ((ret = wland_sdio_flow_ctrl(bus->sdiodev))) {
		WLAND_ERR("wland_sdio_flow_ctrl failed!\n");
		goto out;
	}
#endif /*WLAND_SDIO_FC_SUPPORT */

	size = count / 4;

	size_l = size & 0xFF;
	size_h = ((size >> 8) & 0x7F) | 0x80;	//0x80 flags means lenght higer bytes

	sdio_claim_host(bus->sdiodev->func);
	ret = sdioh_request_byte(bus->sdiodev, SDIOH_WRITE,
		URSDIO_FUNC1_SPKTLEN_LO, &size_l);
	ret |= sdioh_request_byte(bus->sdiodev, SDIOH_WRITE,
		URSDIO_FUNC1_SPKTLEN_HI, &size_h);
	if (ret) {
		WLAND_ERR(" sdioh_request_byte failed!\n");
		sdio_release_host(bus->sdiodev->func);
		goto out;
	}
	ret = sdioh_request_bytes(bus->sdiodev, SDIOH_WRITE,
		URSDIO_FUNC1_FIFO_WR, buf, count);
	sdio_release_host(bus->sdiodev->func);

out:
	WLAND_DBG(SDIO, TRACE, "Done(ret:%d)\n", ret);
	return ret;
}

int wland_sdio_recv_pkt(struct wland_sdio *bus, struct sk_buff *skbbuf,
	uint size)
{
	int ret;		/* Return code from calls */

	if ((!skbbuf) || (!size)) {
		WLAND_ERR("skb empty!\n");
		return -EINVAL;;
	}

	ret = sdioh_request_bytes(bus->sdiodev, SDIOH_READ,
		URSDIO_FUNC1_FIFO_RD, skbbuf->data, size);
	if (ret < 0) {
		WLAND_ERR("SDIO read data failed! \n");
		return ret;
	}

	//skbbuf->len = size;

	WLAND_DBG(SDIO, TRACE, "Done(ret:%d,RxData,len:%d)\n", ret, size);
	return ret;
}

static int wland_ops_sdio_probe(struct sdio_func *func,
	const struct sdio_device_id *id)
{
	struct wland_sdio_dev *sdiodev;
	struct wland_bus *bus_if;
	struct osl_info *osh;
	int err = -ENODEV;

	WLAND_DBG(SDIO, TRACE, "Enter\n");
	WLAND_DBG(SDIO, INFO, "Class=%x\n", func->class);
	WLAND_DBG(SDIO, INFO, "sdio vendor ID: 0x%04x\n", func->vendor);
	WLAND_DBG(SDIO, INFO, "sdio device ID: 0x%04x\n", func->device);
	WLAND_DBG(SDIO, INFO, "Function#: %d\n", func->num);

	if (id->vendor != SDIO_VENDOR_ID_RDAWLAN) {
		WLAND_ERR("Unmatch Vendor ID: 0x%x.\n", id->vendor);
		return -ENODEV;
	}

	osh = osl_attach(func, SDIO_BUS, true);
	if (!osh)
		return -ENOMEM;

	bus_if = osl_malloc(osh, sizeof(struct wland_bus));
	if (!bus_if) {
		osl_detach(osh);
		return -ENOMEM;
	}
	memset(bus_if, '\0', sizeof(struct wland_bus));

	sdiodev = osl_malloc(osh, sizeof(struct wland_sdio_dev));
	if (!sdiodev) {
		osl_free(osh, bus_if, sizeof(struct wland_bus));
		osl_detach(osh);
		return -ENOMEM;
	}
	memset(sdiodev, '\0', sizeof(struct wland_sdio_dev));

	/*
	 * initial sdiodev func parameters
	 */
	sdiodev->func = func;
	sdiodev->bus_if = bus_if;

	bus_if->bus_priv.sdio = sdiodev;
	bus_if->osh = osh;

	dev_set_drvdata(&func->dev, bus_if);

	sdiodev->dev = &func->dev;

	atomic_set(&sdiodev->suspend, false);
	sdiodev->card_sleep = true;

	init_waitqueue_head(&sdiodev->request_byte_wait);
	init_waitqueue_head(&sdiodev->request_word_wait);
	init_waitqueue_head(&sdiodev->request_buffer_wait);

	WLAND_DBG(SDIO, TRACE, "F1 found, calling real sdio probe...\n");

	err = wland_sdioh_attach(sdiodev);
	if (err < 0)
		goto fail;

	/*
	 * try to attach to the target device
	 */
	sdiodev->bus = wland_sdio_probe(osh, sdiodev);
	if (!sdiodev->bus) {
		WLAND_ERR("device attach failed\n");
		goto fail;
	}
	WLAND_DBG(SDIO, TRACE, "Done,init completed success...\n");
	return 0;
fail:
	wland_sdioh_detach(sdiodev);
	dev_set_drvdata(&func->dev, NULL);
	osl_free(osh, sdiodev, sizeof(struct wland_sdio_dev));
	osl_free(osh, bus_if, sizeof(struct wland_bus));
	osl_detach(osh);
	return err;
}

static void wland_ops_sdio_remove(struct sdio_func *func)
{
	struct wland_bus *bus_if = NULL;
	struct wland_sdio_dev *sdiodev = NULL;
	struct osl_info *osh = NULL;

	WLAND_DBG(SDIO, TRACE, "Enter\n");
	bus_if = dev_get_drvdata(&func->dev);
	if (bus_if) {
		sdiodev = bus_if->bus_priv.sdio;
		osh = bus_if->osh;
	} else {
		WLAND_ERR("bus_if == NULL and go out.\n");
		goto out;
	}
	if (sdiodev == NULL || osh == NULL) {
		WLAND_ERR("sdiodev == NULL || osh == NULL and go out.\n");
		goto out;
	}
	WLAND_DBG(SDIO, TRACE, "SDIO-VID:0x%04x,SDIO-DID:0x%04x,Function:%d\n",
		func->vendor, func->device, func->num);

	sdiodev->bus_if->state = WLAND_BUS_DOWN;

	if (sdiodev->bus) {
		wland_sdio_release(sdiodev->bus);
		sdiodev->bus = NULL;
	}

	wland_sdioh_detach(sdiodev);

	dev_set_drvdata(&sdiodev->func->dev, NULL);

	osl_free(osh, sdiodev, sizeof(struct wland_sdio_dev));
	osl_free(osh, bus_if, sizeof(struct wland_bus));
	osl_detach(osh);

out:
	WLAND_DBG(SDIO, TRACE, "Done\n");
}

#ifdef CONFIG_PM
static int wland_sdio_suspend(struct device *dev)
{
	int ret = 0;
	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;
	mmc_pm_flag_t sdio_flags;

	WLAND_DBG(SDIO, TRACE, "Enter.\n");
	netif_device_detach(bus_if->drvr->iflist[0]->ndev);

	sdio_flags = sdio_get_host_pm_caps(sdiodev->func);
	if (!(sdio_flags & MMC_PM_KEEP_POWER)) {
		WLAND_ERR("Host can't keep power while suspended\n");
		return -EINVAL;
	}

	ret = sdio_set_host_pm_flags(sdiodev->func, MMC_PM_KEEP_POWER);
	if (ret) {
		WLAND_ERR("Failed to set pm_flags\n");
		return ret;
	}

	atomic_set(&sdiodev->suspend, true);

	/*
	 * Watchdog timer interface for pm ops
	 */
	while (sdiodev->card_sleep != true) {
		if (down_interruptible(&sdiodev->bus->txclk_sem)) {
			WLAND_ERR("Can not request bus->txclk_sem.wland_sdio_suspend\n");
			continue;
		}
		wland_sdio_clkctl(sdiodev->bus, CLK_NONE);
		up(&sdiodev->bus->txclk_sem);
		wland_sched_timeout(50);
	}

	WLAND_DBG(SDIO, TRACE, "Done.\n");
	return 0;
}

static int wland_sdio_resume(struct device *dev)
{

	struct wland_bus *bus_if = dev_get_drvdata(dev);
	struct wland_sdio_dev *sdiodev = bus_if->bus_priv.sdio;

	WLAND_DBG(SDIO, TRACE, "Enter\n");
	netif_device_attach(bus_if->drvr->iflist[0]->ndev);

	atomic_set(&sdiodev->suspend, false);

	WLAND_DBG(SDIO, TRACE, "Done\n");
	return 0;
}

static const struct dev_pm_ops wland_sdio_pm_ops = {
	.suspend = wland_sdio_suspend,
	.resume = wland_sdio_resume,
};
#endif /* ifdef CONFIG_PM */

static struct sdio_driver wland_sdmmc_driver = {
	.probe = wland_ops_sdio_probe,
	.remove = wland_ops_sdio_remove,
	.name = WLAND_SDIO_NAME,
	.id_table = wland_sdmmc_ids,
	.drv = {
#ifdef CONFIG_PM
		.pm = &wland_sdio_pm_ops,
#endif
		.shutdown = &rda_wland_shutdown,
	},
};

/*	Public entry points & extern's */
int wland_sdioh_attach(struct wland_sdio_dev *sdiodev)
{
	int err_ret = 0;

	WLAND_DBG(SDIO, TRACE, "Enter.\n");
	sdio_claim_host(sdiodev->func);
	err_ret = sdio_set_block_size(sdiodev->func, SDIO_FUNC1_BLOCKSIZE);
	if (err_ret < 0) {
		WLAND_ERR("Failed to set F1 blocksize.\n");
		goto out;
	}
	/*
	 * Enable Function 1
	 */
	err_ret = sdio_enable_func(sdiodev->func);
	if (err_ret < 0)
		WLAND_ERR("Failed to enable F1 Err: 0x%08x.\n", err_ret);
out:
	sdio_release_host(sdiodev->func);
	WLAND_DBG(SDIO, TRACE, "Done.\n");
	return err_ret;
}

void wland_sdioh_detach(struct wland_sdio_dev *sdiodev)
{
	WLAND_DBG(SDIO, TRACE, "Enter\n");

	/*
	 * Disable Function 1
	 */
	sdio_claim_host(sdiodev->func);
	sdio_disable_func(sdiodev->func);
	sdio_release_host(sdiodev->func);

	WLAND_DBG(SDIO, TRACE, "Done\n");
}

void wland_sdio_register(void)
{
	WLAND_DBG(SDIO, TRACE, "Enter\n");

	if (sdio_register_driver(&wland_sdmmc_driver)) {
		WLAND_ERR("sdio_register_driver failed\n");
		wland_registration_sem_up(false);
	} else {
		/*
		 * disable sdio interrupt
		 */
		rda_mmc_set_sdio_irq(1, false);

		/*
		 * trigger sdio bus scan device
		 */
		rda_mmc_bus_scan(1);
	}

	WLAND_DBG(SDIO, TRACE, "Done\n");
}

void wland_sdio_exit(void)
{
	WLAND_DBG(SDIO, TRACE, "Enter\n");
	sdio_unregister_driver(&wland_sdmmc_driver);
	WLAND_DBG(SDIO, TRACE, "Done\n");
}

//////////////////////////////////////////////////////////////////////////////////
//                                                                              //
//                     Linux OSI Relations Area                                 //
//                                                                              //
//////////////////////////////////////////////////////////////////////////////////

void dhd_os_sdlock(struct wland_sdio *bus)
{
	if (bus->threads_only)
		down(&bus->sdsem);
}

void dhd_os_sdunlock(struct wland_sdio *bus)
{
	if (bus->threads_only)
		up(&bus->sdsem);
}

void dhd_os_sdlock_txq(struct wland_sdio *bus, unsigned long *flags)
{
	if (bus)
		spin_lock_irqsave(&bus->txqlock, *flags);
}

void dhd_os_sdunlock_txq(struct wland_sdio *bus, unsigned long *flags)
{
	if (bus)
		spin_unlock_irqrestore(&bus->txqlock, *flags);
}

void dhd_os_sdlock_rxq(struct wland_sdio *bus, unsigned long *flags)
{
	if (bus)
		spin_lock_irqsave(&bus->rxqlock, *flags);
}

void dhd_os_sdunlock_rxq(struct wland_sdio *bus, unsigned long *flags)
{
	if (bus)
		spin_unlock_irqrestore(&bus->rxqlock, *flags);
}

int dhd_os_ioctl_resp_wait(struct wland_sdio *bus, uint * condition,
	bool * pending)
{
	DECLARE_WAITQUEUE(wait, current);

	/*
	 * Convert timeout in millsecond to jiffies
	 */
	int timeout = msecs_to_jiffies(IOCTL_RESP_TIMEOUT);

	/*
	 * Wait until control frame is available
	 */
	add_wait_queue(&bus->dcmd_resp_wait, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	while (!(*condition) && (!signal_pending(current) && timeout))
		timeout = schedule_timeout(timeout);

	if (signal_pending(current))
		*pending = true;

	set_current_state(TASK_RUNNING);
	remove_wait_queue(&bus->dcmd_resp_wait, &wait);

	return timeout;
}

void dhd_os_ioctl_resp_wake(struct wland_sdio *bus)
{
	if (waitqueue_active(&bus->dcmd_resp_wait))
		wake_up(&bus->dcmd_resp_wait);
}

void dhd_os_wait_for_event(struct wland_sdio *bus, bool * lockvar)
{
	int timeout = msecs_to_jiffies(IOCTL_RESP_TIMEOUT);
	wait_event_interruptible_timeout(bus->ctrl_wait, !(*lockvar),
		timeout);
}

void dhd_os_wait_event_wakeup(struct wland_sdio *bus)
{
	if (waitqueue_active(&bus->ctrl_wait))
		wake_up(&bus->ctrl_wait);
}

int dhd_os_wake_lock(struct wland_sdio *bus)
{
	ulong flags;
	int ret = 0;

	if (bus) {
		spin_lock_irqsave(&bus->wakelock_spinlock, flags);
#ifdef CONFIG_HAS_WAKELOCK
		if (!bus->wakelock_counter) {
			//wake_lock(&bus->wl_wifi);
			WLAND_DBG(SDIO, TRACE, "wl_wifi locked.\n");
		}
#endif /*CONFIG_HAS_WAKELOCK */
		bus->wakelock_counter++;
		ret = bus->wakelock_counter;
		spin_unlock_irqrestore(&bus->wakelock_spinlock, flags);
	}
	return ret;
}

int dhd_os_wake_lock_timeout(struct wland_sdio *bus)
{
	ulong flags;
	int ret = 0;

	if (bus) {
		spin_lock_irqsave(&bus->wakelock_spinlock, flags);
		ret = bus->wakelock_rx_timeout_enable >
			bus->wakelock_ctrl_timeout_enable ?
			bus->wakelock_rx_timeout_enable :
			bus->wakelock_ctrl_timeout_enable;
#ifdef CONFIG_HAS_WAKELOCK
		if (bus->wakelock_rx_timeout_enable)
			wake_lock_timeout(&bus->wl_rxwake,
				msecs_to_jiffies
				(bus->wakelock_rx_timeout_enable));
		if (bus->wakelock_ctrl_timeout_enable)
			wake_lock_timeout(&bus->wl_ctrlwake,
				msecs_to_jiffies
				(bus->wakelock_ctrl_timeout_enable));
#endif /*CONFIG_HAS_WAKELOCK */
		bus->wakelock_rx_timeout_enable = 0;
		bus->wakelock_ctrl_timeout_enable = 0;
		spin_unlock_irqrestore(&bus->wakelock_spinlock, flags);
	}
	return ret;
}

int dhd_os_wake_unlock(struct wland_sdio *bus)
{
	ulong flags;
	int ret = 0;

	dhd_os_wake_lock_timeout(bus);
	if (bus) {
		spin_lock_irqsave(&bus->wakelock_spinlock, flags);
		if (bus->wakelock_counter) {
			bus->wakelock_counter--;
#ifdef CONFIG_HAS_WAKELOCK
			if (!bus->wakelock_counter) {
				//wake_unlock(&bus->wl_wifi);
				WLAND_DBG(SDIO, TRACE, "wl_wifi unlock.\n");
			}
#endif /*CONFIG_HAS_WAKELOCK */
			ret = bus->wakelock_counter;
		}
		spin_unlock_irqrestore(&bus->wakelock_spinlock, flags);
	}
	return ret;
}

int dhd_os_check_wakelock(struct wland_sdio *bus)
{
#ifdef CONFIG_HAS_WAKELOCK
	if (!bus)
		return 0;

	/*
	 * Indicate to the SD Host to avoid going to suspend if internal locks are up
	 */
	if (wake_lock_active(&bus->wl_wifi)
		|| wake_lock_active(&bus->wl_wdwake))
		return 1;
#endif /*CONFIG_HAS_WAKELOCK */
	return 0;
}

int dhd_os_wd_wake_lock(struct wland_sdio *bus)
{
	ulong flags;
	int ret = 0;

	if (bus) {
		spin_lock_irqsave(&bus->wakelock_spinlock, flags);
#ifdef CONFIG_HAS_WAKELOCK
		/*
		 * if wakelock_wd_counter was never used : lock it at once
		 */
		if (!bus->wakelock_wd_counter) {
			wake_lock(&bus->wl_wdwake);
			WLAND_DBG(SDIO, TRACE, "wl_wdwake lock.\n");
		}
#endif /*CONFIG_HAS_WAKELOCK */
		bus->wakelock_wd_counter++;
		ret = bus->wakelock_wd_counter;
		spin_unlock_irqrestore(&bus->wakelock_spinlock, flags);
	}
	return ret;
}

int dhd_os_wd_wake_unlock(struct wland_sdio *bus)
{
	ulong flags;
	int ret = 0;

	if (bus) {
		spin_lock_irqsave(&bus->wakelock_spinlock, flags);
		if (bus->wakelock_wd_counter) {
			bus->wakelock_wd_counter = 0;
#ifdef CONFIG_HAS_WAKELOCK
			wake_unlock(&bus->wl_wdwake);
			WLAND_DBG(SDIO, TRACE, "wl_wdwake unlock.\n");
#endif /*CONFIG_HAS_WAKELOCK */
		}
		spin_unlock_irqrestore(&bus->wakelock_spinlock, flags);
	}
	return ret;
}

ulong dhd_os_spin_lock(struct wland_sdio * bus)
{
	ulong flags = 0;

	if (bus)
		spin_lock_irqsave(&bus->wakelock_spinlock, flags);

	return flags;
}

void dhd_os_spin_unlock(struct wland_sdio *bus, ulong flags)
{
	if (bus)
		spin_unlock_irqrestore(&bus->wakelock_spinlock, flags);
}

//////////////////////////////////////////////////////////////////////////////////
