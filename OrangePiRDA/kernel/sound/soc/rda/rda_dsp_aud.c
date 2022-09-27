/*
 * =====================================================================================
 *
 *       Filename:  rda_dsp_aud.c
 *
 *    Description:  RDA DSP_AUD  Receiver driver for linux.
 *
 *        Version:  1.0
 *        Created:  06/12/2013 04:19:05 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Naiquan Hu, 
 *   Organization:  RDA Microelectronics Inc.
 *
  * Copyright (C) 2013 RDA Microelectronics Inc.
 * =====================================================================================
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h> // udelay()
#include <linux/device.h> // device_create()
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/fs.h>
//#include <linux/version.h>      /* constant of kernel version */
#include <asm/uaccess.h> // get_user()
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/audiocontrol.h>
#include <linux/gpio.h>
#include <plat/md_sys.h>
#include <linux/leds.h>

#include "rda_dsp_aud.h"
#include <rda/tgt_ap_board_config.h>

struct msys_device *bp_gpioc_msys = NULL;

/* rda_gpioc_data driver data */
struct rda_gpioc_data {
	struct msys_device *gpioc_msys;
};

typedef struct
{
	u8 id;
	u8 value;
	u8 default_value1;
	u8 default_value2;
} rda_gpioc_op;

#ifdef _TGT_AP_LED_RED_FLASH
#define LED_CAM_FLASH	"red-flash"
#elif defined(_TGT_AP_LED_GREEN_FLASH)
#define LED_CAM_FLASH	"green-flash"
#elif defined(_TGT_AP_LED_BLUE_FLASH)
#define LED_CAM_FLASH	"blue-flash"
#endif

#ifdef LED_CAM_FLASH
DEFINE_LED_TRIGGER(rda_sensor_led);
#endif

static int rda_modem_gpioc_notify(struct notifier_block *nb, unsigned long mesg, void *data)
{
	struct msys_device *pmsys_dev = container_of(nb, struct msys_device, notifier);
	struct client_mesg *pmesg = (struct client_mesg *)data;


	if (pmesg->mod_id != SYS_GEN_MOD) {
		return NOTIFY_DONE;
	}

	if (mesg != SYS_GEN_MESG_RTC_TRIGGER) {
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

int rda_gpioc_operation(rda_gpioc_op *gpioc_op)
{
	int enable;
	int ret, value;
	u8 data[sizeof(rda_gpioc_op)] = { 0 };
	struct client_cmd gpioc_cmd;

	value = sizeof(rda_gpioc_op);

	memcpy(data, gpioc_op, sizeof(rda_gpioc_op));
	memset(&gpioc_cmd, 0, sizeof(gpioc_cmd));
	gpioc_cmd.pmsys_dev = bp_gpioc_msys;
	gpioc_cmd.mod_id = SYS_GPIO_MOD;
	gpioc_cmd.mesg_id = SYS_GPIO_CMD_OPERATION;
	gpioc_cmd.pdata = (void *)&data;
	gpioc_cmd.data_size = sizeof(data);
	ret = rda_msys_send_cmd(&gpioc_cmd);

	printk( ">>>> [%s], ret [%d] \n", __func__, ret);

	return ret;
}

static ssize_t rdabp_gpio_open_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value;

	printk(KERN_DEBUG "%s, buf: %s\n", __func__, buf);

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	printk("The SYSTEM value %d\n", value);
	
	return count;
}

/*
 * GPO set.
 */
static ssize_t OrangePi_2G_IOT_gpio_set_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int gpionum;
	rda_gpioc_op gpioc_op;

	if (sscanf(buf, "%u", &gpionum) != 1)
		return -EINVAL;

	gpioc_op.id = gpionum;
	gpioc_op.value = 1;
	gpioc_op.default_value1 = 0;
	gpioc_op.default_value2 = 0;
	rda_gpioc_operation(&gpioc_op);

	return 1;
}

/*
 * GPO clear.
 */
static ssize_t OrangePi_2G_IOT_gpio_clear_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int gpionum;
	rda_gpioc_op gpioc_op;

	if (sscanf(buf, "%u", &gpionum) != 1)
		return -EINVAL;

	gpioc_op.id = gpionum;
	gpioc_op.value = 0;
	gpioc_op.default_value1 = 0;
	gpioc_op.default_value2 = 0;
	rda_gpioc_operation(&gpioc_op);

	return 1;
}
static ssize_t rdabp_gpio_close_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	printk(KERN_DEBUG "%s, buf: %s\n", __func__, buf);

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;
	
	return count;
}


static ssize_t rdabp_gpio_set_io_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	rda_gpioc_op gpioc_op;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;
    gpioc_op.id = 8;
	gpioc_op.value = 1;
	gpioc_op.default_value1 = 0;
	gpioc_op.default_value2 = 0;
	rda_gpioc_operation(&gpioc_op);
	
	return 1;
}


static ssize_t rdabp_gpio_get_value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	return count;
}


static ssize_t rdabp_gpio_set_value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	return count;
}

static ssize_t rdabp_gpio_enable_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	printk(KERN_DEBUG "%s, buf: %s\n", __func__, buf);

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;


	return count;
}

static DEVICE_ATTR(gpio_open, 0333,
		NULL, rdabp_gpio_open_store);
static DEVICE_ATTR(gpio_close,0333,
		NULL, rdabp_gpio_close_store);
static DEVICE_ATTR(gpio_set_io, 0333,
		NULL,rdabp_gpio_set_io_store);
static DEVICE_ATTR(gpio_get_value, 0333,
		NULL,rdabp_gpio_get_value_store);
static DEVICE_ATTR(gpio_set_value,  0333,
		NULL,rdabp_gpio_set_value_store);
static DEVICE_ATTR(gpio_enable_irq, 0333,
		NULL,rdabp_gpio_enable_irq_store);
static DEVICE_ATTR(gpo_set, 0333,
		NULL, OrangePi_2G_IOT_gpio_set_store);
static DEVICE_ATTR(gpo_clear, 0333,
		NULL, OrangePi_2G_IOT_gpio_clear_store);


static int rda_gpioc_platform_probe(struct platform_device *pdev)
{
	struct rda_gpioc_data *gpioc_data = NULL;
	int ret = 0;

	gpioc_data = devm_kzalloc(&pdev->dev, sizeof(struct rda_gpioc_data), GFP_KERNEL);

	if (gpioc_data == NULL) {
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, gpioc_data);

	// ap <---> modem gpioc
	gpioc_data->gpioc_msys = rda_msys_alloc_device();
	if (!gpioc_data->gpioc_msys) {
		ret = -ENOMEM;
	}

	gpioc_data->gpioc_msys->module = SYS_GPIO_MOD;
	gpioc_data->gpioc_msys->name = "rda-gpioc";
	gpioc_data->gpioc_msys->notifier.notifier_call = rda_modem_gpioc_notify;

	rda_msys_register_device(gpioc_data->gpioc_msys);
	bp_gpioc_msys = gpioc_data->gpioc_msys;

	device_create_file(&pdev->dev, &dev_attr_gpio_open);
	device_create_file(&pdev->dev, &dev_attr_gpio_close);
	device_create_file(&pdev->dev, &dev_attr_gpio_set_io);
	device_create_file(&pdev->dev, &dev_attr_gpio_get_value);
	device_create_file(&pdev->dev, &dev_attr_gpio_set_value);
	device_create_file(&pdev->dev, &dev_attr_gpio_enable_irq);
	device_create_file(&pdev->dev, &dev_attr_gpo_set);
	device_create_file(&pdev->dev, &dev_attr_gpo_clear);

	#ifdef LED_CAM_FLASH
	led_trigger_register_simple(LED_CAM_FLASH, &rda_sensor_led);
	mdelay(5);
	led_trigger_event(rda_sensor_led, LED_HALF);
	printk(KERN_INFO "rda_gpioc_platform_probe 2\n");
	#endif

	return ret;
}

static int __exit rda_gpioc_platform_remove(struct platform_device *pdev)
{
	struct rda_gpioc_data *gpioc_data = platform_get_drvdata(pdev);

	rda_msys_unregister_device(gpioc_data->gpioc_msys);
	rda_msys_free_device(gpioc_data->gpioc_msys);

	platform_set_drvdata(pdev, NULL);

	#ifdef LED_CAM_FLASH
	led_trigger_unregister_simple(rda_sensor_led);
	#endif

	return 0;
}

static struct platform_driver rda_gpioc_driver = {
	.driver = {
		.name = "rda-gpioc",
		.owner = THIS_MODULE,
	},

	.probe = rda_gpioc_platform_probe,
	.remove = __exit_p(rda_gpioc_platform_remove),
};

static int __init rda_gpioc_modinit(void)
{
	return platform_driver_register(&rda_gpioc_driver);
}

static void __exit rda_gpioc_modexit(void)
{
	platform_driver_unregister(&rda_gpioc_driver);
}

module_init(rda_gpioc_modinit);
module_exit(rda_gpioc_modexit);


