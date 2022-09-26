/*
 * rda_bl.c - A driver for controlling backlight of RDA
 *
 * Copyright (C) 2013-2014 RDA Microelectronics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/mutex.h>

#include <plat/md_sys.h>
#include <plat/boot_mode.h>

#include <linux/regulator/consumer.h>
#include <mach/regulator.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */


extern int rda_fb_register_client(struct notifier_block * nb);
extern int rda_fb_unregister_client(struct notifier_block *nb);

#define RDA_FB_TIMEOUT	1000

struct rda_bl {
	struct backlight_device *bldev;
	struct platform_device *pdev;

	struct regulator *bl_reg;

	struct rda_bl_device_data *pdata;

	int sleep_level;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_ops;
#endif /* CONFIG_HAS_EARLYSUSPEND */
	struct mutex pending_lock;
	int fb_count;

	struct notifier_block fb_notifier;
	struct completion fb_done;

	struct work_struct bl_work;
};

/*
 * The current table of backlight is a sub-table of its regulator.
 * It can be as same as table of its regulator.
 */
static int bl_table[POWER_BL_NUM] = {0};

static int rda_bl_notifier_cb(struct notifier_block *self, unsigned long event, void *data)
{
	struct rda_bl *rda_bl = container_of(self, struct rda_bl, fb_notifier);

	if (event) {
		mutex_lock(&rda_bl->pending_lock);

		if (rda_bl->fb_count < 0) {
			mutex_unlock(&rda_bl->pending_lock);
			return 0;
		}
		++rda_bl->fb_count;
		if (rda_bl->fb_count == 1) {
			int boot_mode = rda_get_boot_mode();

			if (boot_mode == BM_NORMAL) {
				/* Waiting for LCD is ready. */
				msleep(200);
			}
			mutex_unlock(&rda_bl->pending_lock);
			complete(&rda_bl->fb_done);

			return 0;
		}
		mutex_unlock(&rda_bl->pending_lock);
	} else {
		INIT_COMPLETION(rda_bl->fb_done);
		rda_bl->fb_count = 0;
	}

	return 0;
}

static int rda_bl_set_intensity(struct backlight_device *bd)
{
	struct rda_bl *rda_bl = bl_get_data(bd);
	int intensity = bd->props.brightness;
	int tsize = ARRAY_SIZE(bl_table);
	int index;
	int ret;
	unsigned long fb_timeout = 0;
	unsigned long timeout_ret = 0;
	int boot_mode = rda_get_boot_mode();

	if (intensity < 0) {
		index = 0;
	} else {
		if (rda_bl->pdata->min + intensity > rda_bl->pdata->max) {
			index = rda_bl->pdata->max;
		} else {
			index = rda_bl->pdata->min + intensity;
		}

		if (bd->props.state & BL_CORE_FBBLANK) {
			index = 0;
		}
	}

	mutex_lock(&rda_bl->pending_lock);
	if (rda_bl->fb_count == 0) {
		mutex_unlock(&rda_bl->pending_lock);
		if (boot_mode == BM_CHARGER) {
			schedule_work(&rda_bl->bl_work);
			return 0;
		}

		fb_timeout = RDA_FB_TIMEOUT;
		timeout_ret = wait_for_completion_timeout(&rda_bl->fb_done,
			msecs_to_jiffies(fb_timeout));
		if (timeout_ret == 0) {
			pr_debug("<rda-bl> : fb doesn't transit frame, timeout(%ld ms)\n", fb_timeout);
		}
		mutex_lock(&rda_bl->pending_lock);
	} else if (rda_bl->fb_count < 0) {
		/* Do not enable backlight after suspend. */
		mutex_unlock(&rda_bl->pending_lock);
		return 0;
	}
	mutex_unlock(&rda_bl->pending_lock);

	pr_debug("%s : write val = %d\n", __func__, index);

	if (boot_mode == BM_CHARGER) {
		/*
		 * We need to wait for ready of lcd as charger mode.
		 */
		msleep(30);
	}

	ret = regulator_set_voltage(rda_bl->bl_reg, bl_table[index], bl_table[tsize - 1]);

	return ret;
}

static int rda_bl_get_intensity(struct backlight_device *bd)
{
	struct rda_bl *rda_bl = bl_get_data(bd);
	int intensity;
	int i;
	int tsize = ARRAY_SIZE(bl_table);
	int value;

	value = regulator_get_voltage(rda_bl->bl_reg);

	for (i = 0; i < tsize - 1; i++) {
		if (bl_table[i] == value) {
			break;
		}
	}

	intensity =  i - rda_bl->pdata->min;

	pr_debug("%s : read val = %d\n", __func__, value);

	return intensity;
}

static void rda_bl_work(struct work_struct *work)
{
	struct rda_bl *rda_bl = container_of(work, struct rda_bl, bl_work);

	rda_bl_set_intensity(rda_bl->bldev);
}

static const struct backlight_ops rda_bl_ops = {
	.get_brightness = rda_bl_get_intensity,
	.update_status  = rda_bl_set_intensity,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rda_bl_early_suspend(struct early_suspend *h);
static void rda_bl_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int rda_bl_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bldev;
	struct rda_bl *rda_bl;
	struct rda_bl_device_data *pdata;
	int retval;
	int i;

	pr_debug("call %s\n", __func__);

	/* Init table of voltage of backlight */
	for (i = 0; i < POWER_BL_NUM; i++) {
		bl_table[i] = (i + 1) * 10000;
	}

	rda_bl = kzalloc(sizeof(struct rda_bl), GFP_KERNEL);
	if (!rda_bl) {
		return -ENOMEM;
	}

	rda_bl->pdev = pdev;

	pdata = (struct rda_bl_device_data *)pdev->dev.platform_data;
	if (!pdata) {
		retval = -ENODEV;
		goto err_free_mem;
	}
	rda_bl->pdata = pdata;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = pdata->max - pdata->min;

	bldev = backlight_device_register(RDA_BL_DRV_NAME,
		&pdev->dev,
		rda_bl,
		&rda_bl_ops,
		&props);
	if (IS_ERR(bldev)) {
		retval = PTR_ERR(bldev);
		goto err_free_mem;
	}

	init_completion(&rda_bl->fb_done);
	INIT_WORK(&rda_bl->bl_work, rda_bl_work);

	rda_bl->bldev = bldev;

	rda_bl->bl_reg = regulator_get(NULL, LDO_BACKLIGHT);
	if (IS_ERR(rda_bl->bl_reg)) {
		dev_err(&pdev->dev, "could not find regulator devices\n");
		retval = PTR_ERR(rda_bl->bl_reg);
		goto err_free_bl_dev;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	rda_bl->early_ops.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 3;
	rda_bl->early_ops.suspend = rda_bl_early_suspend;
	rda_bl->early_ops.resume = rda_bl_late_resume;

	register_early_suspend(&rda_bl->early_ops);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	rda_bl->fb_notifier.notifier_call = rda_bl_notifier_cb;
	rda_fb_register_client(&rda_bl->fb_notifier);

	platform_set_drvdata(pdev, rda_bl);

	/* Power up the backlight by default at middle intesity. */
	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.brightness = bldev->props.max_brightness / 2;

	mutex_init(&rda_bl->pending_lock);
	rda_bl->fb_count = 0;

	/* Deferred to turn on backlight */
	schedule_work(&rda_bl->bl_work);

	return 0;

err_free_bl_dev:
	platform_set_drvdata(pdev, NULL);
	backlight_device_unregister(bldev);

err_free_mem:
	if (rda_bl) {
		kfree(rda_bl);
	}

	return retval;
}

static int __exit rda_bl_remove(struct platform_device *pdev)
{
	struct rda_bl *rda_bl = platform_get_drvdata(pdev);

	if (!rda_bl) {
		return -EINVAL;
	}

	regulator_put(rda_bl->bl_reg);

	rda_fb_unregister_client(&rda_bl->fb_notifier);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rda_bl->early_ops);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	backlight_device_unregister(rda_bl->bldev);
	platform_set_drvdata(pdev, NULL);

	kfree(rda_bl);

	return 0;
}

static void rda_bl_shutdown(struct platform_device *pdev)
{
	struct rda_bl *rda_bl = platform_get_drvdata(pdev);
	int tsize = ARRAY_SIZE(bl_table);

	if (!rda_bl) {
		return;
	}

	regulator_set_voltage(rda_bl->bl_reg, bl_table[0], bl_table[tsize - 1]);

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rda_bl_early_suspend(struct early_suspend *h)
{
	struct rda_bl *rda_bl = container_of(h, struct rda_bl, early_ops);
	int tsize = ARRAY_SIZE(bl_table);

	if (!rda_bl) {
		return;
	}

	mutex_lock(&rda_bl->pending_lock);
	rda_bl->fb_count = -1;
	mutex_unlock(&rda_bl->pending_lock);

	regulator_set_voltage(rda_bl->bl_reg, bl_table[0], bl_table[tsize - 1]);
}

static void rda_bl_late_resume(struct early_suspend *h)
{
	return;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static struct platform_driver rda_bl_driver = {
	.driver = {
		.name = RDA_BL_DRV_NAME,
	},
	.remove = __exit_p(rda_bl_remove),
	.shutdown = rda_bl_shutdown,
};

static int __init rda_bl_init(void)
{
	return platform_driver_probe(&rda_bl_driver, rda_bl_probe);
}

module_init(rda_bl_init);

MODULE_AUTHOR("Tao Lei <leitao@rdamicro.com>");
MODULE_DESCRIPTION("RDA backlight driver");
MODULE_LICENSE("GPL");

