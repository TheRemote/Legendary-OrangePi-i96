
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
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux_osl.h>

#include <wland_defs.h>
#include <wland_dbg.h>

#ifdef USE_MAC_FROM_RDA_NVRAM
#include <plat/md_sys.h>
#endif

#define WIFI_NVRAM_FILE_NAME "/data/misc/wifi/WLANMAC"

static int nvram_read(char *filename, char *buf, ssize_t len, int offset)
{
	struct file *fd;
	int retLen = -1;

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	fd = filp_open(filename, O_RDWR|O_CREAT, 0666);

	if (IS_ERR(fd)) {
		WLAND_ERR("[nvram_read] : failed to open!\n");
		return -1;
	}

	do {
		if ((fd->f_op == NULL) || (fd->f_op->read == NULL)) {
			WLAND_ERR("[wlan][nvram_read] : file can not be read!!\n");
			break;
		}

		if (fd->f_pos != offset) {
			if (fd->f_op->llseek) {
				if (fd->f_op->llseek(fd, offset, 0) != offset) {
					WLAND_ERR("[wlan][nvram_read] : failed to seek!!\n");
					break;
				}
			} else {
				fd->f_pos = offset;
			}
		}
		retLen = fd->f_op->read(fd, buf, len, &fd->f_pos);
	} while (false);

	filp_close(fd, NULL);
	set_fs(old_fs);

	return retLen;
}

static int nvram_write(const char *filename, char *buf, ssize_t len, int offset)
{
	struct file *fd;
	int retLen = -1;

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	fd = filp_open(filename, O_RDWR|O_CREAT, 0666);

	if (IS_ERR(fd)) {
		WLAND_ERR("[nvram_write] : failed to open!\n");
		return -1;
	}

	do {
		if ((fd->f_op == NULL) || (fd->f_op->write == NULL)) {
			WLAND_ERR("[nvram_write] : file can not be write!\n");
			break;
		}

		if (fd->f_pos != offset) {
			if (fd->f_op->llseek) {
				if (fd->f_op->llseek(fd, offset, 0) != offset) {
					WLAND_ERR("[nvram_write] : failed to seek!\n");
					break;
				}
			} else {
				fd->f_pos = offset;
			}
		}

		retLen = fd->f_op->write(fd, buf, len, &fd->f_pos);

	} while (false);

	filp_close(fd, NULL);
	set_fs(old_fs);

	return retLen;
}

int wland_get_mac_address(char *buf)
{
	return nvram_read(WIFI_NVRAM_FILE_NAME, buf, 6, 0);
}

int wland_set_mac_address(char *buf)
{
	return nvram_write(WIFI_NVRAM_FILE_NAME, buf, 6, 0);
}

#ifdef USE_MAC_FROM_RDA_NVRAM
int wlan_read_mac_from_nvram(char *buf)
{
	return nvram_read(WIFI_NVRAM_FILE_NAME, buf, 6, 0);
}

int wlan_write_mac_to_nvram(char *buf)
{
	return nvram_write(WIFI_NVRAM_FILE_NAME, buf, 6, 0);
}
#endif
