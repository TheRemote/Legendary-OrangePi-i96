/*
 * V4L2 driver for RDA camera sensor
 *
 * Copyright (C) 2014 Rda electronics, Inc.
 *
 * Contact: Xing Wei <xingwei@rdamicro.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/v4l2-mediabus.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>

#include <media/soc_camera.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>

#include <mach/board.h>
#include <plat/devices.h>
#include <plat/rda_debug.h>

#include <rda/tgt_ap_board_config.h>

#include "rda_isp_reg.h"
#include "rda_sensor.h"

#define ARRAY_NUM(a)	(sizeof(a) / sizeof(a[0]))
#define to_priv(sd)	container_of(sd, struct rda_sensor_priv, subdev)

#if 0
#ifdef _TGT_AP_LED_RED_FLASH
#define LED_CAM_FLASH	"red-flash"
#elif defined(_TGT_AP_LED_GREEN_FLASH)
#define LED_CAM_FLASH	"green-flash"
#elif defined(_TGT_AP_LED_BLUE_FLASH)
#define LED_CAM_FLASH	"blue-flash"
#endif
#endif

extern void rcam_pdn(bool pdn, bool acth);
extern void rcam_rst(bool pdn, bool acth);
extern void rcam_clk(bool out, int freq);

#ifdef LED_CAM_FLASH
DEFINE_LED_TRIGGER(rda_sensor_led);
#endif


static DEFINE_MUTEX(rda_sensor_mutex);

static bool sensor_test_mode = false;

static struct i2c_client *rda_sensor_i2c_client[] = {
	NULL, //back sensor i2c client
	NULL, //front sensor i2c client
};

static struct sensor_win_size def_win_size[] = {
	WIN_SIZE("VGA", W_VGA, H_VGA, NULL),
};

struct rda_sensor_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

static struct rda_sensor_datafmt def_datafmt[] = {
	{ V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG },
};

struct rda_sensor_priv {
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_mbus_framefmt cur_mf;

	int model;
	int dev_id;
	struct sensor_dev *cur_sdev;
	struct sensor_win_size *cur_win_size;
};


/* Private Function ------------------------------------------------*/
static struct rda_sensor_priv *to_rda_sensor(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client),
			struct rda_sensor_priv, subdev);
}

static struct rda_sensor_datafmt *find_datafmt(enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_NUM(def_datafmt); i++)
		if (def_datafmt[i].code == code)
			return def_datafmt + i;
	return NULL;
}

static struct sensor_win_size *select_window(struct rda_sensor_priv *p,
		int width, int height)
{
	struct sensor_win_cfg *cfg = NULL;
	struct sensor_win_size *tmp = NULL;
	int i;

	if (!p->cur_sdev)
		return NULL;

	cfg = p->cur_sdev->info->win_cfg;
	if (!cfg || (cfg->num == 0))
		return NULL;

	for (i = 0; i < cfg->num; i++) {
		tmp = cfg->win_size + i;
		if ((tmp->width == width) && (tmp->height == height)) {
			p->cur_win_size = tmp;
			return tmp;
		}
	}
	return NULL;
}

static int sensor_i2c_read(const struct i2c_client *client,
		const u16 addr, u8 *data, u8 bits)
{
	unsigned char tmp[2];
	int len, ret;

	if (!client)
		return -ENODEV;

	if (bits == 8) {
		tmp[0] = addr & 0xff;
		len = 1;
	} else if (bits == 16) {
		tmp[0] = addr >> 8;
		tmp[1] = addr & 0xff;
		len = 2;
	} else {
		printk(KERN_ERR "%s: bits %d not support\n",
				__func__, bits);
		return -EINVAL;
	}

	ret = i2c_master_send(client, tmp, len);
	if (ret < len) {
		printk(KERN_ERR "%s: i2c read error, reg: 0x%x\n",
				__func__, addr);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(client, data, 1);
	if (ret < 1) {
		printk(KERN_ERR "%s: i2c read error, reg: 0x%x\n",
				__func__, addr);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int sensor_i2c_write(const struct i2c_client *client,
		const u16 addr, const u8 data, u8 bits)
{
	unsigned char tmp[3];
	int len, ret;

	if (!client)
		return -ENODEV;

	if (bits == 8) {
		tmp[0] = addr & 0xff;
		tmp[1] = data;
		len = 2;
	} else if (bits == 16) {
		tmp[0] = addr >> 8;
		tmp[1] = addr & 0xff;
		tmp[2] = data;
		len = 3;
	} else {
		printk(KERN_ERR "%s: bits %d not support\n",
				__func__, bits);
		return -EINVAL;
	}

	ret = i2c_master_send(client, tmp, len);
	if (ret < len) {
		printk(KERN_ERR "%s: i2c write error, reg: 0x%x\n",
				__func__, addr);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int sensor_write_array(const struct i2c_client *client,
		struct sensor_reg_list *regs)
{
	int i, res;
	struct sensor_reg *tmp = NULL;

	for (i = 0; i < regs->size; i++) {
		tmp = regs->val + i;
		res = sensor_i2c_write(client,
				tmp->addr, tmp->data, tmp->bits);
		if (res != 0)
			return res;
		if (tmp->wait)
			mdelay(tmp->wait);
	}

	return 0;
}

/* Callback Function for sensor ------------------------------------*/
#define DEF_SENSOR_READ(n) \
static void rda_sensor##n##_read(const u16 addr, u8 *data, u8 bits) \
{ \
	sensor_i2c_read(rda_sensor_i2c_client[n], addr, data, bits); \
}
DEF_SENSOR_READ(0);
DEF_SENSOR_READ(1);

#define SENSOR_READ(n) \
	((n == 0) ? \
	 (void*)rda_sensor0_read : \
	 ((n == 1) ? \
	  (void*)rda_sensor1_read : \
	  NULL))

#define DEF_SENSOR_WRITE(n) \
static void rda_sensor##n##_write(const u16 addr, const u8 data, u8 bits) \
{ \
	sensor_i2c_write(rda_sensor_i2c_client[n], addr, data, bits); \
}
DEF_SENSOR_WRITE(0);
DEF_SENSOR_WRITE(1);

#define SENSOR_WRITE(n) \
	((n == 0) ? \
	 (void*)rda_sensor0_write : \
	 ((n == 1) ? \
	  (void*)rda_sensor1_write : \
	  NULL))

#ifdef _TGT_AP_CAM_ISP_ENABLE
/* Public Function for camera adjust AE-----------------------------*/
void rda_sensor_reg_w(struct v4l2_subdev *sd, const u16 addr, const u8 data)
{
	struct rda_sensor_priv *priv = to_priv(sd);
	const struct i2c_client *client = NULL;
	unsigned int sensor_num = ARRAY_NUM(rda_sensor_i2c_client);

	if (priv && (priv->dev_id > -1) && (priv->dev_id < sensor_num)) {
		client = rda_sensor_i2c_client[priv->dev_id];
		sensor_i2c_write(client, addr, data, 8);
	} else if (priv){
		rda_dbg_camera("%s: priv=%p, id=%d\n",
				__func__, priv, priv->dev_id);
	}
}

/* update gain by ISP request for auto exposure*/
void rda_sensor_upd_gain_isp(struct v4l2_subdev *sd, int val)
{
	struct rda_sensor_priv *priv = to_priv(sd);
	struct sensor_ops *ops;
	if (priv && (priv->cur_sdev)) {
		ops = priv->cur_sdev->ops;
		if (ops->upd_gain_isp){
			ops->upd_gain_isp(val);
		}
	}
}

void rda_sensor_upd_exp_isp(struct v4l2_subdev *sd, int exp)
{
	struct rda_sensor_priv *priv = to_priv(sd);
	struct sensor_ops *ops;
	if (priv && (priv->cur_sdev)) {
		ops = priv->cur_sdev->ops;
		if (ops->upd_exp_isp){
			ops->upd_exp_isp(exp);
		}
	}
}

struct  raw_sensor_info_data* rda_sensor_get_raw_sensor_info(struct v4l2_subdev *sd)
{
	struct rda_sensor_priv *priv = to_priv(sd);
	struct sensor_dev *tmp = NULL;

	if (priv && (priv->cur_sdev)) {
		tmp = (priv->cur_sdev);
		if (tmp->info->type_sensor == RAW)
			return tmp->info->raw_sensor;
		else
			return NULL;

	}
	return NULL;
}

#endif

static void rda_sensor_pdn(bool pdn, bool acth, int id)
{
	if (id) {
#ifdef GPIO_CAM_PWDN1
		gpio_request(GPIO_CAM_PWDN1, "camera pwdn1");
		if (pdn) {
			if (acth)
				gpio_direction_output(GPIO_CAM_PWDN1, 1);
			else
				gpio_direction_output(GPIO_CAM_PWDN1, 0);
		} else {
			if (acth)
				gpio_direction_output(GPIO_CAM_PWDN1, 0);
			else
				gpio_direction_output(GPIO_CAM_PWDN1, 1);
		}
		gpio_free(GPIO_CAM_PWDN1);
#else
		rcam_pdn(pdn, acth);
#endif
	} else {
#ifdef GPIO_CAM_PWDN0
		gpio_request(GPIO_CAM_PWDN0, "camera pwdn0");
		if (pdn) {
			if (acth)
				gpio_direction_output(GPIO_CAM_PWDN0, 1);
			else
				gpio_direction_output(GPIO_CAM_PWDN0, 0);
		} else {
			if (acth)
				gpio_direction_output(GPIO_CAM_PWDN0, 0);
			else
				gpio_direction_output(GPIO_CAM_PWDN0, 1);
		}
		gpio_free(GPIO_CAM_PWDN0);
#else
		rcam_pdn(pdn, acth);
#endif
	}
}

static void rda_sensor_rst(bool rst, bool acth)
{
#ifdef GPIO_CAM_RESET
	gpio_request(GPIO_CAM_RESET, "camera reset");
	if (rst) {
		if (acth)
			gpio_direction_output(GPIO_CAM_RESET, 1);
		else
			gpio_direction_output(GPIO_CAM_RESET, 0);
	} else {
		if (acth)
			gpio_direction_output(GPIO_CAM_RESET, 0);
		else
			gpio_direction_output(GPIO_CAM_RESET, 1);
	}
	gpio_free(GPIO_CAM_RESET);
#else
	rcam_rst(rst, acth);
#endif
}

static void rda_sensor_clk(bool out, int mclk)
{
	rcam_clk(out, mclk);
}

static void rda_sensor_s_flash(bool on)
{
#ifdef GPIO_CAM_FLASH
	gpio_request(GPIO_CAM_FLASH, "camera flash");
	if (on)
		gpio_direction_output(GPIO_CAM_FLASH, 1);
	else
		gpio_direction_output(GPIO_CAM_FLASH, 0);
	gpio_free(GPIO_CAM_FLASH);
#elif defined(LED_CAM_FLASH)
	if (on)
		led_trigger_event(rda_sensor_led, LED_HALF);
	else
		led_trigger_event(rda_sensor_led, LED_OFF);
#endif
}

/* Export Function for camera sensor module in vendor --------------*/
static int rda_sensor_power(struct v4l2_subdev *sd, int on);
int rda_sensor_adapt(struct sensor_callback_ops *callback,
		struct list_head *sdl, u32 id)
{
	struct rda_sensor_priv *priv;
	struct sensor_dev *tmp = NULL;
	struct i2c_client *client = rda_sensor_i2c_client[id];
	bool adapted = false;
	u32 cid;

	printk(KERN_ALERT "%s: camera id=%d\n", __func__, id);

	if (!client || !sdl || list_empty(sdl)) {
		printk(KERN_ERR "%s: failed i2c client %p sdl %p!\n",
				__func__, client, sdl);
		return -EINVAL;
	}

	callback->cb_pdn = rda_sensor_pdn;
	callback->cb_rst = rda_sensor_rst;
	callback->cb_clk = rda_sensor_clk;
	callback->cb_i2c_r = SENSOR_READ(id);
	callback->cb_i2c_w = SENSOR_WRITE(id);

	mutex_lock(&rda_sensor_mutex);
	priv = to_rda_sensor(client);

	rda_sensor_power(&priv->subdev, 1);
	list_for_each_entry(tmp, sdl, list) {
		client->addr = tmp->info->i2c_addr;
		cid = tmp->info->chip_id;
		tmp->ops->power(id,
				tmp->info->mclk,
				tmp->info->rst_act_h,
				tmp->info->pdn_act_h);
		printk(KERN_ALERT "%s: try %s, chip_id=0x%x\n",
				__func__, tmp->info->name, cid);
		if (cid == tmp->ops->get_chipid()) {
			rda_dbg_camera(KERN_ERR "%s: name=%s, success!\n",
					__func__, tmp->info->name);
			tmp->cb = callback;
			priv->cur_sdev = tmp;
			priv->dev_id = id;
			adapted = true;
			break;
		}
	}
	tmp = NULL;
	rda_sensor_power(&priv->subdev, 0);

	mutex_unlock(&rda_sensor_mutex);
	if (!adapted) {
		printk(KERN_ERR "%s: failed!\n", __func__);
		return -ENODEV;
	}
	return 0;
}
EXPORT_SYMBOL(rda_sensor_adapt);


/* V4L2 SUBDEV VIDEO OPS -------------------------------------------*/
static int rda_sensor_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_NUM(def_datafmt))
		return -EINVAL;

	*code = def_datafmt[index].code;

	return 0;
}

static int rda_sensor_try_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	struct rda_sensor_priv *priv = to_priv(sd);
	const struct rda_sensor_datafmt *fmt = find_datafmt(mf->code);
	struct sensor_win_size *tmp = NULL;

	tmp = select_window(priv, mf->width, mf->height);
	if (!tmp) {
		priv->cur_win_size = def_win_size;
		mf->width = def_win_size[0].width;
		mf->height = def_win_size[0].height;
	}
	rda_dbg_camera("%s: window size %s ok\n",
			__func__, priv->cur_win_size->name);

	if (!fmt) {
		mf->code = def_datafmt[0].code;
		mf->colorspace = def_datafmt[0].colorspace;
	}
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int rda_sensor_s_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct rda_sensor_priv *priv = to_priv(sd);
	struct sensor_reg_list *tmp = NULL;
	int ret = 0;

	if (!find_datafmt(mf->code))
		return -EINVAL;

	if (!priv->cur_sdev)
		return -ENODEV;

	/* Set window size if already get */
	tmp = priv->cur_win_size->win_val;
	if (tmp) {
		ret = sensor_write_array(client, tmp);
		rda_dbg_camera("%s: set window size ok\n", __func__);
	} else
		rda_dbg_camera("%s: no window size to set\n", __func__);

	priv->cur_mf = *mf;

	return ret;
}

static int rda_sensor_g_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *mf)
{
	struct rda_sensor_priv *priv = to_priv(sd);

	*mf = priv->cur_mf;

	return 0;
}

static int rda_sensor_cropcap(struct v4l2_subdev *sd,
		struct v4l2_cropcap *a)
{
	struct rda_sensor_priv *priv = to_priv(sd);
	struct sensor_win_size *s = priv->cur_win_size;

	a->bounds.left = 0;
	a->bounds.top = 0;
	a->bounds.width = s->width;
	a->bounds.height = s->height;
	a->defrect = a->bounds;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator = 1;
	a->pixelaspect.denominator = 1;

	return 0;
}

static int rda_sensor_g_mbus_config(struct v4l2_subdev *sd,
		struct v4l2_mbus_config *cfg)
{
	struct rda_sensor_priv *priv = to_priv(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct sensor_csi_cfg *csi = NULL;

	if (!priv->cur_sdev)
		return -ENOIOCTLCMD;

	csi = priv->cur_sdev->info->csi_cfg;
	if (csi->csi_en) {
		cfg->type = V4L2_MBUS_CSI2;
		cfg->flags = V4L2_MBUS_CSI2_1_LANE |
			V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else {
		cfg->type = V4L2_MBUS_PARALLEL;
		cfg->flags = V4L2_MBUS_MASTER |
			V4L2_MBUS_PCLK_SAMPLE_RISING |
			V4L2_MBUS_VSYNC_ACTIVE_HIGH |
			V4L2_MBUS_HSYNC_ACTIVE_HIGH |
			V4L2_MBUS_DATA_ACTIVE_HIGH;
		cfg->flags = soc_camera_apply_board_flags(ssdd, cfg);
	}

	return 0;
};

extern void rcam_config_csi(unsigned int d, unsigned int c,
		unsigned int line, unsigned int flag);
static int rda_sensor_s_mbus_config(struct v4l2_subdev *sd,
		const struct v4l2_mbus_config *cfg)
{
	struct rda_sensor_priv *priv = to_priv(sd);
	struct sensor_csi_cfg *csi = NULL;
	unsigned int d, c, line, flag = 0;

	if (!priv->cur_sdev)
		return -ENOIOCTLCMD;

	csi = priv->cur_sdev->info->csi_cfg;
	if (csi->csi_en) {
		d = (csi->d_term_en << 16) | csi->dhs_settle;
		c = (csi->c_term_en << 16) | csi->chs_settle;
		line = priv->cur_win_size->height;

#ifdef _TGT_AP_CAM0_CSI_CH_SEL
		if (!priv->dev_id) {
			flag |= _TGT_AP_CAM0_CSI_CH_SEL;
		}
#endif

#ifdef _TGT_AP_CAM1_CSI_CH_SEL
		if (priv->dev_id) {
			flag |= _TGT_AP_CAM1_CSI_CH_SEL;
		}
#endif

#ifdef _TGT_AP_CAM_CSI_AVDD
		flag |= _TGT_AP_CAM_CSI_AVDD << 1;
#endif

#ifdef _TGT_AP_CAM0_LANE2_ENABLE
		if (!priv->dev_id) {
			flag |= _TGT_AP_CAM0_LANE2_ENABLE << 2;
#if (_TGT_AP_CAM0_LANE2_ENABLE)
			flag |= (0x1 << 3); /* set clk_edge_sel. */
#endif
		}
#endif

#ifdef _TGT_AP_CAM1_LANE2_ENABLE
		if (priv->dev_id) {
			flag |= _TGT_AP_CAM1_LANE2_ENABLE << 2;
#if (_TGT_AP_CAM1_LANE2_ENABLE)
			flag |= (0x1 << 3); /* set clk_edge_sel. */
#endif
		}
#endif

#ifdef _TGT_AP_CAM_ISP_ENABLE
//		line = rda_sensor_get_sensor_line_number(sd);
        if (priv->cur_sdev->info->raw_sensor)
			line = priv->cur_sdev->info->raw_sensor->frame_line_num;
#endif

		rcam_config_csi(d, c, line, flag);
	}

	return 0;
}

static int rda_sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rda_sensor_priv *priv = to_priv(sd);
	struct sensor_ops *ops = NULL;

	if (!priv->cur_sdev)
		return -ENODEV;

	priv->cur_sdev->cb->cb_i2c_r = SENSOR_READ(priv->dev_id);
	priv->cur_sdev->cb->cb_i2c_w = SENSOR_WRITE(priv->dev_id);
	ops = priv->cur_sdev->ops;
	if (enable) {
		if (!sensor_test_mode) {
			/* Set exp & awb */
			if (ops->set_exp)
				ops->set_exp(priv->cur_sdev->info->exp_def);
			if (ops->set_awb)
				ops->set_awb(priv->cur_sdev->info->awb_def);
		}
		if (ops->start)
			ops->start();
	} else {
		sensor_test_mode = false;
		if (ops->stop)
			ops->stop();
	}

	return 0;
}

static int rda_sensor_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	return 0;
}

static struct v4l2_subdev_video_ops rda_sensor_subdev_video_ops = {
	.s_stream	= rda_sensor_s_stream,
	.s_mbus_fmt	= rda_sensor_s_fmt,
	.g_mbus_fmt	= rda_sensor_g_fmt,
	.try_mbus_fmt	= rda_sensor_try_fmt,
	.enum_mbus_fmt	= rda_sensor_enum_fmt,
	.cropcap	= rda_sensor_cropcap,
	.g_mbus_config	= rda_sensor_g_mbus_config,
	.s_mbus_config	= rda_sensor_s_mbus_config,
	.g_crop		= rda_sensor_g_crop,
};

/* V4L2 SUBDEV CORE OPS --------------------------------------------*/
static int rda_sensor_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *id)
{
	struct rda_sensor_priv *priv = to_priv(sd);

	id->ident = priv->model;
	id->revision = 0;

	return 0;
}

static int rda_sensor_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct rda_sensor_priv *priv = to_priv(sd);
	struct sensor_dev *sdev = NULL;
	int retry = 2;
	int ret = 0;

	rda_dbg_camera("%s: power %d\n", __func__, on);
#ifdef GPIO_CAM_EN
	gpio_request(GPIO_CAM_EN, "camera enable");
#endif

	while (on && retry--) {
		/* power on the sensor */
#ifdef GPIO_CAM_EN
		gpio_direction_output(GPIO_CAM_EN, 1);
#endif
		ret = soc_camera_set_power(&client->dev, ssdd, 1);
		if ((ret < 0) || (priv->dev_id == -1))
			break; //power on failed or first time power on

		/* Init sensor to default */
		if (!priv->cur_sdev) {
			ret = -ENODEV;
			on = 0;
			break;
		}
		sdev = priv->cur_sdev;
		if (sdev->ops->power) {
			sdev->ops->power(priv->dev_id,
					sdev->info->mclk,
					sdev->info->rst_act_h,
					sdev->info->pdn_act_h);
			rda_dbg_camera("%s: id=%d, mclk=%d\n",
					__func__,
					priv->dev_id,
					sdev->info->mclk);
		}
		ret = sensor_write_array(client, sdev->info->init);
		mdelay(10);
		if (ret) {
			rda_dbg_camera("%s: init sensor failed ret=%d\n",
			__func__, ret);
			ret = -EIO;
		} else {
			rda_dbg_camera("%s: init sensor ok\n", __func__);
			break;
		}
		/* power off the sensor */
		rda_sensor_clk(false, 0);
		soc_camera_set_power(&client->dev, ssdd, 0);
#ifdef GPIO_CAM_EN
		gpio_direction_output(GPIO_CAM_EN, 0);
#endif
		mdelay(10);
	}

	if (!on) {
		/* power off the sensor */
		rda_sensor_clk(false, 0);
		soc_camera_set_power(&client->dev, ssdd, 1);
#ifdef GPIO_CAM_EN
		gpio_direction_output(GPIO_CAM_EN, 0);
#endif

#ifdef _TGT_AP_CAM_RDA2201_ENABLE
		/* reset rda2201 sensor when power off camera. */
		sensor_i2c_write(client, 0x0, 0x0, 8);
#endif
	}

#ifdef GPIO_CAM_EN
	gpio_free(GPIO_CAM_EN);
#endif
	return ret;
}

static struct v4l2_subdev_core_ops rda_sensor_subdev_core_ops = {
	.g_chip_ident = rda_sensor_g_chip_ident,
	.s_power = rda_sensor_power,
};

/* V4L2 SUBDEV OPS -------------------------------------------------*/
static struct v4l2_subdev_ops rda_sensor_subdev_ops = {
	.core	= &rda_sensor_subdev_core_ops,
	.video	= &rda_sensor_subdev_video_ops,
};

/* V4L2 CTRL -------------------------------------------------------*/
static int rda_sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rda_sensor_priv *priv;
	struct sensor_ops *ops;

	if (!ctrl)
		return -EINVAL;

	priv = container_of(ctrl->handler, struct rda_sensor_priv, hdl);
	if (!priv->cur_sdev)
		return -ENODEV;

	priv->cur_sdev->cb->cb_i2c_r = SENSOR_READ(priv->dev_id);
	priv->cur_sdev->cb->cb_i2c_w = SENSOR_WRITE(priv->dev_id);
	ops = priv->cur_sdev->ops;

	switch (ctrl->id) {
	case V4L2_CID_BLACK_LEVEL:
		if (ops->get_lum)
			ctrl->val = ops->get_lum();
		break;
	default:
		rda_dbg_camera("%s: ctrl->%d not support!\n",
				__func__, ctrl->id);
	}
	return 0;
}

static int rda_sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct rda_sensor_priv *priv;
	struct sensor_ops *ops;

	if (!ctrl)
		return -EINVAL;

	priv = container_of(ctrl->handler, struct rda_sensor_priv, hdl);
	if (priv->dev_id == -1)
		return 0;
	if (!priv->cur_sdev)
		return -ENODEV;

	priv->cur_sdev->cb->cb_i2c_r = SENSOR_READ(priv->dev_id);
	priv->cur_sdev->cb->cb_i2c_w = SENSOR_WRITE(priv->dev_id);
	ops = priv->cur_sdev->ops;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		rda_dbg_camera("%s: set hflip\n", __func__);
		if (ops->set_flip)
			ops->set_flip(0, (int)ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		rda_dbg_camera("%s: set vflip\n", __func__);
		if (ops->set_flip)
			ops->set_flip(1, (int)ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		rda_dbg_camera("%s: set exposure\n", __func__);
		if (ops->set_exp) {
			ops->set_exp((int)ctrl->val);
			priv->cur_sdev->info->exp_def = (int)ctrl->val;
		}
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		rda_dbg_camera("%s: set white balance\n", __func__);
		if (ops->set_awb) {
			ops->set_awb((int)ctrl->val);
			priv->cur_sdev->info->awb_def = (int)ctrl->val;
		}
		break;
	case V4L2_CID_BRIGHTNESS:
		rda_dbg_camera("%s: set brightness\n", __func__);
		if (ops->set_bri) {
			ops->set_bri((int)ctrl->val);
			priv->cur_sdev->info->bri_def = (int)ctrl->val;
		}
		break;
	case V4L2_CID_CONTRAST:
		rda_dbg_camera("%s: set contrast\n", __func__);
		if (ops->set_con) {
			ops->set_con((int)ctrl->val);
			priv->cur_sdev->info->con_def = (int)ctrl->val;
		}
		break;
	case V4L2_CID_SHARPNESS:
		rda_dbg_camera("%s: set sharpness\n", __func__);
		if (ops->set_sha) {
			ops->set_sha((int)ctrl->val);
			priv->cur_sdev->info->sha_def = (int)ctrl->val;
		}
		break;
	case V4L2_CID_SATURATION:
		rda_dbg_camera("%s: set saturation\n", __func__);
		if (ops->set_sat) {
			ops->set_sat((int)ctrl->val);
			priv->cur_sdev->info->sat_def = (int)ctrl->val;
		}
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		rda_dbg_camera("%s: anti flicker\n",__func__);
		if (ops->set_af) {
		        ops->set_af((int)ctrl->val);
			priv->cur_sdev->info->af_def = (int)ctrl->val;
		}
		break;
	case V4L2_CID_FLASH_LED_MODE:
		switch (ctrl->val) {
		case V4L2_FLASH_LED_MODE_NONE:
			rda_dbg_camera("%s: flash off\n",
					__func__);
			rda_sensor_s_flash(false);
			break;
		case V4L2_FLASH_LED_MODE_FLASH:
			rda_dbg_camera("%s: flash on\n",
					__func__);
		case V4L2_FLASH_LED_MODE_TORCH:
			rda_dbg_camera("%s: flash torch\n",
					__func__);
			rda_sensor_s_flash(true);
			break;
		case 4:
			if (ops->sensor_test) {
				sensor_test_mode = true;
				ops->sensor_test();
				rda_dbg_camera("set test mode\n");
			}
			break;
		default:
			BUG();
		}
		break;
	default:
		rda_dbg_camera("%s: id[%d] not support\n",
				__func__, ctrl->id);
		break;
	}
	return 0;
}

static const struct v4l2_ctrl_ops rda_sensor_ctrl_ops = {
	.g_volatile_ctrl = rda_sensor_g_ctrl,
	.s_ctrl = rda_sensor_s_ctrl,
};

static struct v4l2_ctrl_config rda_sensor_ctrls[] = {
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Horizontal Flip",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Vertical Flip",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id		= V4L2_CID_BLACK_LEVEL,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Black Level",
		.min		= 0,
		.max		= 1,
		.step		= 1,
		.def		= 0,
		.flags		= V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id		= V4L2_CID_FLASH_LED_MODE,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "LED Mode",
		.max		= 8,
		.menu_skip_mask	= ((1 << 2) | (1 << 3)),
		.def		= 0,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.min		= -3,
		.max		= 3,
		.step		= 1,
		.def		= 0,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id		= V4L2_CID_POWER_LINE_FREQUENCY,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Anti Flicker",
		.min		= 0,
		.max		= 3,
		.menu_skip_mask	= (1 << 0),
		.def		= 1,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id		= V4L2_CID_DO_WHITE_BALANCE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Do White Balance",
		.min		= 0,
		.max		= 6,
		.step		= 1,
		.def		= 1,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id 	= V4L2_CID_BRIGHTNESS,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "brightness",
		.min		= 0,
		.max		= 6,
		.step		= 1,
		.def		= 3,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id 	= V4L2_CID_CONTRAST,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "contrast",
		.min		= 0,
		.max		= 6,
		.step		= 1,
		.def		= 3,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id 	= V4L2_CID_SHARPNESS,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "sharpness",
		.min		= 0,
		.max		= 6,
		.step		= 1,
		.def		= 3,
	},
	{
		.ops		= &rda_sensor_ctrl_ops,
		.id 	= V4L2_CID_SATURATION,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "saturation",
		.min		= 0,
		.max		= 6,
		.step		= 1,
		.def		= 3,
	},
};

static int rda_sensor_video_probe(struct i2c_client *client)
{
	struct rda_sensor_priv *priv = to_rda_sensor(client);
	printk(KERN_INFO "%s: priv=%p\n", __func__, priv);
	if (!priv)
		return -EINVAL;
	priv->model = 0x2518;

	return v4l2_ctrl_handler_setup(&priv->hdl);
}

/* probe & remove --------------------------------------------------*/
static int rda_sensor_probe(struct i2c_client *client,
		const struct i2c_device_id *did)
{
	struct rda_sensor_priv *priv;
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct v4l2_ctrl *ctrl = NULL;
	int num = ARRAY_NUM(rda_sensor_ctrls);
	int ret;
	printk(KERN_INFO "%s: i2c_client %p\n", __func__, client);
	printk(KERN_INFO "%s: i2c_dev_id %ld\n", __func__, did->driver_data);
	printk(KERN_INFO "%s: i2c_addr %x\n", __func__, client->addr);

	if (!ssdd) {
		printk(KERN_ERR "%s: missing platform data!\n", __func__);
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct rda_sensor_priv), GFP_KERNEL);
	if (!priv) {
		printk(KERN_ERR "%s: Failed to allocate private data!\n",
				__func__);
		return -ENOMEM;
	}

	if (!did) {
		printk(KERN_ERR "%s: i2c device id error!\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&rda_sensor_mutex);
	while (rda_sensor_i2c_client[did->driver_data] != NULL)
		did++;
	rda_sensor_i2c_client[did->driver_data] = client;
	priv->dev_id = -1;
	mutex_unlock(&rda_sensor_mutex);

	v4l2_i2c_subdev_init(&priv->subdev, client, &rda_sensor_subdev_ops);
	printk(KERN_INFO "%s: sd->grp_id %x\n", __func__, priv->subdev.grp_id);
	v4l2_ctrl_handler_init(&priv->hdl, num);
	while (0 < num--) {
		if (rda_sensor_ctrls[num].type == V4L2_CTRL_TYPE_MENU)
			ctrl = v4l2_ctrl_new_std_menu(&priv->hdl,
					rda_sensor_ctrls[num].ops,
					rda_sensor_ctrls[num].id,
					rda_sensor_ctrls[num].max,
					rda_sensor_ctrls[num].menu_skip_mask,
					rda_sensor_ctrls[num].def);
		else
			ctrl = v4l2_ctrl_new_std(&priv->hdl,
					rda_sensor_ctrls[num].ops,
					rda_sensor_ctrls[num].id,
					rda_sensor_ctrls[num].min,
					rda_sensor_ctrls[num].max,
					rda_sensor_ctrls[num].step,
					rda_sensor_ctrls[num].def);

		ctrl->flags |= rda_sensor_ctrls[num].flags;
	}
	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		printk(KERN_ERR "%s: hdl error!\n", __func__);
		return priv->hdl.error;
	}

	ret = rda_sensor_video_probe(client);
	if (ret < 0) {
		printk(KERN_ERR "%s: video probe error!\n", __func__);
		v4l2_ctrl_handler_free(&priv->hdl);
		return ret;
	}

#ifdef LED_CAM_FLASH
	if (did->driver_data == 0)
		led_trigger_register_simple(LED_CAM_FLASH, &rda_sensor_led);
#endif
	printk(KERN_INFO "%s: i2c_client %p\n", __func__, client);

	return 0;
}

static int rda_sensor_remove(struct i2c_client *client)
{
	struct rda_sensor_priv *priv = to_rda_sensor(client);
	unsigned long id = client->driver->id_table->driver_data;

#ifdef LED_CAM_FLASH
	led_trigger_unregister_simple(rda_sensor_led);
#endif
	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	mutex_lock(&rda_sensor_mutex);
	rda_sensor_i2c_client[id] = NULL;
	priv->dev_id = -1;
	mutex_unlock(&rda_sensor_mutex);

	return 0;
}

static const struct i2c_device_id rda_sensor_id[] = {
	{ RDA_SENSOR_DRV_NAME, 0 },
	{ RDA_SENSOR_DRV_NAME, 1 },
	{},
};

MODULE_DEVICE_TABLE(i2c, rda_sensor_id);

static struct i2c_driver rda_sensor_i2c_driver = {
	.driver = {
		.name = RDA_SENSOR_DRV_NAME,
	},
	.probe		= rda_sensor_probe,
	.remove		= rda_sensor_remove,
	.id_table	= rda_sensor_id,
};

module_i2c_driver(rda_sensor_i2c_driver);

MODULE_DESCRIPTION("RDA Camera Sensor driver");
MODULE_AUTHOR("Wei Xing <xingwei@rdamicro.com>");
MODULE_LICENSE("GPL v2");
