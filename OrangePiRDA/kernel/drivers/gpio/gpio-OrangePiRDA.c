#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/rwsem.h>

#include <linux/gpio.h>

#define CLASS_NAME  "gpio_OrangePi"
#define GPIO_NR     1

struct OrangePi_gpio_pd {
	char name[16];
	char link[16];
};

int GPIO_array[GPIO_NR]  = { GPIO_B24 };
char *GPIO_name[GPIO_NR] = { "B24" };
static int namecount;

struct OrangePi_gpio_classdev {
	const char       *name;
	struct mutex     class_mutex;
	int gpio;
	int (*OrangePi_gpio_data_set)(struct OrangePi_gpio_classdev *, int);
	int (*OrangePi_gpio_data_get)(struct OrangePi_gpio_classdev *);
	struct device    *dev;
	struct list_head node;
};

struct OrangePi_gpio {
	struct OrangePi_gpio_pd    *pdata;
	spinlock_t                 lock;
	struct OrangePi_gpio_classdev class;
};

DECLARE_RWSEM(OrangePi_gpio_list_lock);
LIST_HEAD(OrangePi_list);

static struct class *OrangePi_gpio_class;
static struct OrangePi_gpio_pd *OrangePi_pdata[256];
static struct platform_device *OrangePi_gpio_dev[256];

static int OrangePi_gpio_suspend(struct device *dev, pm_message_t state)
{
	return 0;
}

static int OrangePi_gpio_resume(struct device *dev)
{
	return 0;
}

static ssize_t data_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct OrangePi_gpio_classdev *cdev = dev_get_drvdata(dev);
	int length;
	int data;

	mutex_lock(&cdev->class_mutex);
	data = cdev->OrangePi_gpio_data_get(cdev);
	length = sprintf(buf, "%u\n", data);
	mutex_unlock(&cdev->class_mutex);

	return length;
}

static ssize_t data_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct OrangePi_gpio_classdev *cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	int data = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
	
	if (isspace(*after))
		count++;
	if (count == size) {
		ret = count;
		cdev->OrangePi_gpio_data_set(cdev, data);
	}

	return ret;
}

static struct device_attribute OrangePi_gpio_attrs[] = {
	__ATTR(data, 0664, data_show, data_store),
	__ATTR_NULL,
};

static void OrangePi_gpio_release(struct device *dev)
{
	printk("[%s %d]\n", __func__, __LINE__);
}

static int OrangePi_gpio_data_get(struct OrangePi_gpio_classdev *cdev)
{
	return __gpio_get_value(cdev->gpio);
}

static int OrangePi_gpio_data_set(struct OrangePi_gpio_classdev *cdev, int data)
{
	__gpio_set_value(cdev->gpio, data);
	return 0;
}

/*
 * cfg: 0 - input
 *      1 - output
 */
static int OrangePi_gpio_cfg_set(struct OrangePi_gpio_classdev *cdev, int cfg)
{
	if (cfg == 0)
		gpio_direction_input(cdev->gpio);
	else
		gpio_direction_output(cdev->gpio, 0);
	return 0;
}

int OrangePi_gpio_classdev_register(struct device *parent,
		struct OrangePi_gpio_classdev *cdev)
{
	cdev->dev = device_create(OrangePi_gpio_class, parent, 0, cdev,
			"%s", cdev->name);
	if (IS_ERR(cdev->dev))
		return PTR_ERR(cdev->dev);
	down_write(&OrangePi_gpio_list_lock);
	list_add_tail(&cdev->node, &OrangePi_list);
	up_write(&OrangePi_gpio_list_lock);
	mutex_init(&cdev->class_mutex);

	return 0;
}


static int OrangePi_gpio_probe(struct platform_device *dev)
{
	struct OrangePi_gpio        *OrangePi_gpio_entry;
	struct OrangePi_gpio_pd     *pdata = dev->dev.platform_data;
	int                         ret;
	unsigned long               flags;
	char io_area[16];

	OrangePi_gpio_entry = kzalloc(sizeof(struct OrangePi_gpio), GFP_KERNEL);
	if (!OrangePi_gpio_entry) {
		printk(KERN_ERR "Don't have enough memory!\n");
		return -ENOMEM;
	}

	OrangePi_gpio_entry->class.gpio = GPIO_array[namecount];

	sprintf(io_area, "%s", GPIO_name[namecount++]);
	printk(KERN_INFO "GPIO name %s\n", io_area);

	platform_set_drvdata(dev, OrangePi_gpio_entry);
	spin_lock_init(&OrangePi_gpio_entry->lock);
	spin_lock_irqsave(&OrangePi_gpio_entry->lock, flags);
	OrangePi_gpio_entry->pdata = pdata;

	OrangePi_gpio_entry->class.name = io_area;

	OrangePi_gpio_entry->class.OrangePi_gpio_data_set = OrangePi_gpio_data_set; 
	OrangePi_gpio_entry->class.OrangePi_gpio_data_get = OrangePi_gpio_data_get;
		
	/* default output */
	OrangePi_gpio_cfg_set(&OrangePi_gpio_entry->class, 1);
	OrangePi_gpio_data_set(&OrangePi_gpio_entry->class, 0);

	spin_unlock_irqrestore(&OrangePi_gpio_entry->lock, flags);
	
	ret = OrangePi_gpio_classdev_register(&dev->dev, &OrangePi_gpio_entry->class);
	if (ret < 0) {
		printk(KERN_ERR "OrangePi_gpio_classdev_register failed!\n");
		kfree(OrangePi_gpio_entry);
		return -1;
	} 

	return 0;
}

static int OrangePi_gpio_remove(struct platform_device *dev)
{
	printk(KERN_INFO "GPIO remove\n");
	return 0;
}

static struct platform_driver OrangePi_gpio_driver = {
	.probe     = OrangePi_gpio_probe,
	.remove    = OrangePi_gpio_remove,
	.suspend   = OrangePi_gpio_suspend,
	.resume    = OrangePi_gpio_resume,
	.driver    = {
		.name  = CLASS_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init OrangePi_gpio_init(void)
{
	int i;

	/* create debug dir: /sys/class/CLASS_NAME */
	OrangePi_gpio_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(OrangePi_gpio_class))
		return PTR_ERR(OrangePi_gpio_class);

	OrangePi_gpio_class->suspend     = OrangePi_gpio_suspend;
	OrangePi_gpio_class->resume      = OrangePi_gpio_resume;
	OrangePi_gpio_class->dev_attrs   = OrangePi_gpio_attrs;

	for (i = 0; i < GPIO_NR; i++) {
		/* GPIO Request */
		if (gpio_request(GPIO_array[i], NULL)) {
			printk("Request %s fail!\n", GPIO_name[i]);
			goto INIT_CLASS;
		}
		OrangePi_pdata[i] = kzalloc(sizeof(struct OrangePi_gpio_pd), GFP_KERNEL);
		if (!OrangePi_pdata[i]) {
			printk(KERN_ERR "No free space to pdata\n");
			goto GPIO_REQ;
		}

		OrangePi_gpio_dev[i] = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
		if (!OrangePi_gpio_dev[i]) {
			printk(KERN_ERR "No free space to dev!\n");
			goto MEMORY1;
		}
	
		sprintf(OrangePi_pdata[i]->name, "%s", GPIO_name[i]);

		OrangePi_gpio_dev[i]->name = CLASS_NAME;
		OrangePi_gpio_dev[i]->id   = i;
		OrangePi_gpio_dev[i]->dev.platform_data = OrangePi_pdata[i];
		OrangePi_gpio_dev[i]->dev.release       = OrangePi_gpio_release;

		if (platform_device_register(OrangePi_gpio_dev[i])) {
			printk(KERN_ERR "%s platform_device_register fail\n", OrangePi_pdata[i]->name);
			goto MEMORY2;
		}
	}

	if (platform_driver_register(&OrangePi_gpio_driver)) {
		printk(KERN_ERR "GPIO user paltform_driver_register fail\n");
		goto INIT_ERR_FREE;
	}

	printk(KERN_INFO "GPIO_INIT finish used!\n");
	return 0;

INIT_ERR_FREE:
	platform_device_unregister(OrangePi_gpio_dev[0]);
MEMORY2:
	kfree(OrangePi_gpio_dev[0]);
MEMORY1:
	kfree(OrangePi_pdata[0]);
GPIO_REQ:
INIT_CLASS:
	return -1;
}

static void __exit OrangePi_gpio_exit(void)
{
	printk("Exit Modules\n");
}

module_init(OrangePi_gpio_init);
module_exit(OrangePi_gpio_exit);

MODULE_AUTHOR("Buddy.Zhang");
MODULE_DESCRIPTION("OrangePi GPIO USER driver");
MODULE_LICENSE("GPL");
