/*
 * ALSA SoC RDA codec driver
 *
 * Author:      Arun KS, <arunks@mistralsolutions.com>
 * Copyright:   (C) 2008 Mistral Solutions Pvt Ltd.,
 *
 * Based on sound/soc/codecs/wm8731.c by Richard Purdie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 *  rda codec
 *
 *  The machine layer should disable unsupported inputs/outputs by
 *  snd_soc_dapm_disable_pin(codec, "LHPOUT"), etc.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <mach/iomap.h>
#include <asm/io.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <plat/reg_spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <plat/rda_debug.h>
#include <plat/md_sys.h>

#include <linux/gpio.h>

#include "rda_codec.h"
#include "rda_codec_adp.h"

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

static int gpio_audio_extpa = -1;
static int gpio_audio_extpa_1 = -1;//add for external PA

//#define DEBUG 1

/* rda_codec_data driver data */
struct rda_codec_data {
	void __iomem *io_base;
	struct msys_device *codec_msys;
	struct snd_soc_codec* codec;
	u32 in_sample_rate;
	u32 in_channel_nb;
	u32 out_sample_rate;
	u32 out_channel_nb;
	SND_ITF_T itf;
	HAL_AIF_STREAM_T stream;
	AUD_LEVEL_T cfg;
	AUD_APP_MODE_T CodecAppMode;
	u8 codec_is_open;

	// loop mode
	SND_ITF_T loop_mode_itf;
};

#ifdef DEBUG
static void aud_Dump(SND_ITF_T itf, HAL_AIF_STREAM_T *stream, AUD_LEVEL_T *cfg)
{
	printk(KERN_INFO "############################### \n");
	printk(KERN_INFO "itf is [%d] \n", itf);
	if(stream != NULL) {
		printk(KERN_INFO "stream->sampleRate is [%d] \n", stream->sampleRate);
		printk(KERN_INFO "stream->channelNb is [%d] \n", stream->channelNb);
	}
	else {
		printk(KERN_INFO "stream is NULL!!! \n");
	}
	if(cfg != NULL) {
		printk(KERN_INFO "cfg->spkLevel is [%d] \n", cfg->spkLevel);
		printk(KERN_INFO "cfg->micLevel is [%d] \n", cfg->micLevel);
	}
	else {
		printk(KERN_INFO "cfg is NULL!!! \n");
	}

	printk(KERN_INFO "############################### \n");
}
#endif

static int aud_StreamStart(SND_ITF_T itf, HAL_AIF_STREAM_T *stream, AUD_LEVEL_T* cfg, 
		struct rda_codec_data* codec_data)
{
	int ret = 0;
	u8 __dat[sizeof(SND_ITF_T) + sizeof(HAL_AIF_STREAM_T) + sizeof(AUD_LEVEL_T)] = {0};
	struct client_cmd codec_cmd;

#ifdef DEBUG
	aud_Dump(itf, stream, cfg);
#endif

	memcpy((u8 *)&__dat, &itf,
			sizeof(SND_ITF_T));
	memcpy((u8 *)&__dat + sizeof(SND_ITF_T), stream,
			sizeof(HAL_AIF_STREAM_T));
	memcpy((u8* )&__dat + sizeof(SND_ITF_T) + sizeof(HAL_AIF_STREAM_T), cfg,
			sizeof(AUD_LEVEL_T));

	memset(&codec_cmd, 0, sizeof(codec_cmd));
	codec_cmd.pmsys_dev = codec_data->codec_msys;
	codec_cmd.mod_id = SYS_AUDIO_MOD;
	codec_cmd.mesg_id = SYS_AUDIO_CMD_AUD_STREAM_START;
	codec_cmd.pdata = (void *)&__dat;
	codec_cmd.data_size = sizeof(__dat);
	ret = rda_msys_send_cmd(&codec_cmd);
	if ( ret )
		printk(KERN_INFO ">>>> [%s], ret [%d] \n", __func__, ret);
	else
		rda_dbg_audio(">>>> [%s], ret [%d] \n", __func__, ret);

	return ret;
}

static int aud_StreamRecord(SND_ITF_T itf, HAL_AIF_STREAM_T *stream, AUD_LEVEL_T* cfg,
		struct rda_codec_data* codec_data)
{
	int ret = 0;
	u8 __dat[sizeof(SND_ITF_T) + sizeof(HAL_AIF_STREAM_T) + sizeof(AUD_LEVEL_T)] = {0};
	struct client_cmd codec_cmd;

#ifdef DEBUG
	aud_Dump(itf, stream, cfg);
#endif

	memcpy((u8 *)&__dat, &itf, sizeof(SND_ITF_T));
	memcpy((u8 *)&__dat + sizeof(SND_ITF_T), stream,
			sizeof(HAL_AIF_STREAM_T));
	memcpy((u8 *)&__dat + sizeof(SND_ITF_T) + sizeof(HAL_AIF_STREAM_T), cfg,
			sizeof(AUD_LEVEL_T));

	memset(&codec_cmd, 0, sizeof(codec_cmd));
	codec_cmd.pmsys_dev = codec_data->codec_msys;
	codec_cmd.mod_id = SYS_AUDIO_MOD;
	codec_cmd.mesg_id = SYS_AUDIO_CMD_AUD_STREAM_RECORD;
	codec_cmd.pdata = (void *)&__dat;
	codec_cmd.data_size = sizeof(__dat);
	ret = rda_msys_send_cmd(&codec_cmd);
	if ( ret )
		printk(KERN_INFO ">>>> [%s], ret [%d] \n", __func__, ret);
	else
		rda_dbg_audio(">>>> [%s], ret [%d] \n", __func__, ret);


	return ret;
}

static int aud_StreamStop(SND_ITF_T itf,
		struct rda_codec_data* codec_data)
{
	int ret = 0;
	u8 __dat[sizeof(SND_ITF_T)] = {0};
	struct client_cmd codec_cmd;

	memcpy((u8 *)&__dat, &itf, sizeof(SND_ITF_T));

	memset(&codec_cmd, 0, sizeof(codec_cmd));
	codec_cmd.pmsys_dev = codec_data->codec_msys;
	codec_cmd.mod_id = SYS_AUDIO_MOD;
	codec_cmd.mesg_id = SYS_AUDIO_CMD_AUD_STREAM_STOP;
	codec_cmd.pdata = (void *)&__dat;
	codec_cmd.data_size = sizeof(__dat);
	ret = rda_msys_send_cmd(&codec_cmd);
	if ( ret )
		printk(KERN_INFO ">>>> [%s], ret [%d] \n", __func__, ret);
	else
		rda_dbg_audio(">>>> [%s], ret [%d] \n", __func__, ret);

	return ret;
}

static int aud_Setup(SND_ITF_T itf, AUD_LEVEL_T* cfg,
		struct rda_codec_data* codec_data)
{
	int ret = 0;
	u8 __dat[sizeof(SND_ITF_T) + sizeof(AUD_LEVEL_T)] = {0};
	struct client_cmd codec_cmd;

#ifdef DEBUG
	aud_Dump(itf, NULL, cfg);
#endif

	memcpy((u8 *)&__dat, &itf, sizeof(SND_ITF_T));
	memcpy((u8 *)&__dat + sizeof(SND_ITF_T), cfg, sizeof(AUD_LEVEL_T));

	memset(&codec_cmd, 0, sizeof(codec_cmd));
	codec_cmd.pmsys_dev = codec_data->codec_msys;
	codec_cmd.mod_id = SYS_AUDIO_MOD;
	codec_cmd.mesg_id = SYS_AUDIO_CMD_AUD_SETUP;
	codec_cmd.pdata = (void *)&__dat;
	codec_cmd.data_size = sizeof(__dat);
	ret = rda_msys_send_cmd(&codec_cmd);
	if ( ret )
		printk(KERN_DEBUG ">>>> [%s], ret [%d] \n", __func__, ret);
	else
		rda_dbg_audio(">>>> [%s], ret [%d] \n", __func__, ret);


	return ret;
}

static int aud_LoudspeakerWithEarpiece(u8 on,
		struct rda_codec_data* codec_data)
{
	int ret = 0;
	u32 __dat = 0;
	struct client_cmd codec_cmd;

	__dat = on;

	memset(&codec_cmd, 0, sizeof(codec_cmd));
	codec_cmd.pmsys_dev = codec_data->codec_msys;
	codec_cmd.mod_id = SYS_AUDIO_MOD;
	codec_cmd.mesg_id = SYS_AUDIO_CMD_AUD_LOUDSPEAKER_WITH_EARPIECE;
	codec_cmd.pdata = (void *)&__dat;
	codec_cmd.data_size = sizeof(__dat);
	ret = rda_msys_send_cmd(&codec_cmd);
	if ( ret )
		printk(KERN_INFO ">>>> [%s], ret [%d] \n", __func__, ret);
	else
		rda_dbg_audio(">>>> [%s], ret [%d] \n", __func__, ret);

	return ret;
}

static int aud_ForceReceiverMicSelection(u8 on,
		struct rda_codec_data* codec_data)
{
	int ret = 0;
	u32 __dat = 0;
	struct client_cmd codec_cmd;

	__dat = on;

	memset(&codec_cmd, 0, sizeof(codec_cmd));
	codec_cmd.pmsys_dev = codec_data->codec_msys;
	codec_cmd.mod_id = SYS_AUDIO_MOD;
	codec_cmd.mesg_id = SYS_AUDIO_CMD_AUD_FORCE_RECEIVER_MIC_SELECTION;
	codec_cmd.pdata = (void *)&__dat;
	codec_cmd.data_size = sizeof(__dat);
	ret = rda_msys_send_cmd(&codec_cmd);
	if ( ret )
		printk(KERN_INFO ">>>> [%s], ret [%d] \n", __func__, ret);
	else
		rda_dbg_audio(">>>> [%s], ret [%d] \n", __func__, ret);

	return ret;
}

static int aud_CodecAppMode(u32 mode,
		struct rda_codec_data* codec_data)
{
	int ret = 0;
	u32 __dat = 0;
	struct client_cmd codec_cmd;
	
	__dat = mode;
	memset(&codec_cmd, 0, sizeof(codec_cmd));
	codec_cmd.pmsys_dev = codec_data->codec_msys;
	codec_cmd.mod_id = SYS_AUDIO_MOD;
	codec_cmd.mesg_id = SYS_AUDIO_CMD_AUD_CODEC_APP_MODE;
	codec_cmd.pdata = (void *)&__dat;
	codec_cmd.data_size = sizeof(__dat);
	ret = rda_msys_send_cmd(&codec_cmd);
	if ( ret )
		printk(KERN_INFO ">>>> [%s], ret [%d] \n", __func__, ret);
	else
		rda_dbg_audio(">>>> [%s], ret [%d] \n", __func__, ret);

	return ret;
		
}

static int aud_TestModeSetup(SND_ITF_T itf, HAL_AIF_STREAM_T *stream, AUD_LEVEL_T* cfg,
		AUD_TEST_T mode, struct rda_codec_data* codec_data)
{
	int ret = 0;
	u8 __dat[sizeof(SND_ITF_T) + sizeof(HAL_AIF_STREAM_T) + sizeof(AUD_LEVEL_T) + sizeof(AUD_TEST_T)] = {0};
	struct client_cmd codec_cmd;

#ifdef DEBUG
	aud_Dump(itf, stream, cfg);
#endif

	memcpy((u8 *)&__dat, &itf, sizeof(SND_ITF_T));
	memcpy((u8 *)&__dat + sizeof(SND_ITF_T), stream,
			sizeof(HAL_AIF_STREAM_T));
	memcpy((u8 *)&__dat + sizeof(SND_ITF_T) + sizeof(HAL_AIF_STREAM_T), cfg,
			sizeof(AUD_LEVEL_T));
	memcpy((u8 *)&__dat + sizeof(SND_ITF_T) + sizeof(HAL_AIF_STREAM_T) + sizeof(AUD_LEVEL_T), &mode, sizeof(AUD_TEST_T));

	memset(&codec_cmd, 0, sizeof(codec_cmd));
	codec_cmd.pmsys_dev = codec_data->codec_msys;
	codec_cmd.mod_id = SYS_AUDIO_MOD;
	codec_cmd.mesg_id = SYS_AUDIO_CMD_AUD_TEST_MODE_SETUP;
	codec_cmd.pdata = (void *)&__dat;
	codec_cmd.data_size = sizeof(__dat);
	ret = rda_msys_send_cmd(&codec_cmd);
	if ( ret )
		printk(KERN_INFO ">>>> [%s], ret [%d] \n", __func__, ret);
	else
		rda_dbg_audio(">>>> [%s], ret [%d] \n", __func__, ret);

	return ret;
}
static int ctrl_ext_get_reg(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
static int rda_codec_get_open_status(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	printk(KERN_INFO"rda codec : codec_data->codec_is_open %d \n", codec_data->codec_is_open);

	ucontrol->value.integer.value[0] = codec_data->codec_is_open;

	return 0;
}
static int ctrl_ext_set_reg(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
static int rda_codec_set_playback_volume(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u32 volume = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	volume = ucontrol->value.integer.value[0];

	codec_data->cfg.spkLevel = volume;

	return 0;
}
static int rda_codec_set_capture_volume(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u32 volume = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	volume = ucontrol->value.integer.value[0];

	codec_data->cfg.micLevel = volume;

	return 0;
}

static int rda_codec_set_itf(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u32 itf = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	itf = ucontrol->value.integer.value[0];

	codec_data->itf = itf;

	return 0;
}

static int rda_codec_codec_app_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	u32 mode = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	mode = ucontrol->value.integer.value[0];

	codec_data->CodecAppMode = mode;

	ret = aud_CodecAppMode(mode, codec_data);

	return (ret!=0?-1:0);
}

static int rda_codec_start_play(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	if (codec_data->cfg.spkLevel <= SND_SPK_MUTE
		|| codec_data->cfg.spkLevel >= SND_SPK_VOL_QTY) {
		printk(KERN_WARNING "%s: Invalid playback volume\n", __func__);
		ret = -1;
	}

	if (codec_data->out_sample_rate == 0) {
		printk(KERN_WARNING "%s: Invalid sample rate\n", __func__);
		ret = -1;
	}

	if (codec_data->out_channel_nb != HAL_AIF_MONO
		&& codec_data->out_channel_nb != HAL_AIF_STEREO) {
		printk(KERN_WARNING "%s: Invalid channel number\n", __func__);
		ret = -1;
	}

	if (!ret) {
		codec_data->stream.sampleRate = codec_data->out_sample_rate;
		codec_data->stream.channelNb  = codec_data->out_channel_nb;
		ret = aud_StreamStart(codec_data->itf, &(codec_data->stream), 
				&(codec_data->cfg), codec_data);
	}

	if(!ret) {
		codec_data->codec_is_open = TRUE;
	}

	return (ret!=0?-1:0);
}

static int rda_codec_stop(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	ret = aud_StreamStop(codec_data->itf, codec_data);

	if(!ret) {
		codec_data->codec_is_open = FALSE;
	}

	return (ret!=0?-1:0);
}

static int rda_codec_start_record(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	if (codec_data->cfg.micLevel < SND_MIC_MUTE
		|| codec_data->cfg.micLevel >= SND_MIC_VOL_QTY) {
		printk(KERN_WARNING "%s: Invalid capture volume\n", __func__);
		ret = -1;
	}

	if (codec_data->in_sample_rate == 0) {
		printk(KERN_WARNING "%s: Invalid sample rate\n", __func__);
		ret = -1;
	}

	if (codec_data->in_channel_nb != HAL_AIF_MONO
		&& codec_data->in_channel_nb != HAL_AIF_STEREO) {
		printk(KERN_WARNING "%s: Invalid channel number\n", __func__);
		ret = -1;
	}

	if (!ret) {
		codec_data->stream.sampleRate = codec_data->in_sample_rate;
		codec_data->stream.channelNb  = codec_data->in_channel_nb;
		ret = aud_StreamRecord(codec_data->itf, &(codec_data->stream), 
				&(codec_data->cfg), codec_data);
	}

	if(!ret) {
		codec_data->codec_is_open = TRUE;
	}

	return (ret!=0?-1:0);
}

static int rda_codec_set_in_channel_number(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u32 val = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	val = ucontrol->value.integer.value[0];

	codec_data->in_channel_nb = val;

	return 0;
}

static int rda_codec_set_in_sample_rate(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u32 val = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	val = ucontrol->value.integer.value[0];

	codec_data->in_sample_rate = val;

	return 0;
}

static int rda_codec_set_out_channel_number(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u32 val = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	val = ucontrol->value.integer.value[0];

	codec_data->out_channel_nb = val;

	return 0;
}

static int rda_codec_set_out_sample_rate(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u32 val = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	val = ucontrol->value.integer.value[0];

	codec_data->out_sample_rate = val;

	return 0;
}

static int rda_codec_set_spksel(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	AUD_SPK_T spk = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	spk = ucontrol->value.integer.value[0];
	rda_dbg_audio("SPK = %x\n",spk);
	if (spk == AUD_SPK_DISABLE) {
		if (gpio_is_valid(gpio_audio_extpa))
			gpio_set_value(gpio_audio_extpa,0);//turn off
		else
			rda_dbg_audio("disable, gpio extpa is invalid !\n");
		if (gpio_is_valid(gpio_audio_extpa_1)) {
			gpio_set_value(gpio_audio_extpa_1,0);
			rda_dbg_audio("== external PA disable ==\n");
		} else
			rda_dbg_audio("disable, gpio extpa 1 is invalid!\n");
	} else if ((spk == AUD_SPK_LOUD_SPEAKER_EAR_PIECE) || (spk == AUD_SPK_LOUD_SPEAKER)) {
		if (gpio_is_valid(gpio_audio_extpa))
			gpio_set_value(gpio_audio_extpa,1);//turn on
		else
			rda_dbg_audio("gpio extpa is invalid !\n");
		if (gpio_is_valid(gpio_audio_extpa_1)) {
			gpio_set_value(gpio_audio_extpa_1,1);
			rda_dbg_audio("== external PA enable ==\n");
		} else
			rda_dbg_audio("gpio extpa 1 is invalid !\n");
	}

	if(spk == AUD_SPK_LOUD_SPEAKER_EAR_PIECE) {
		ret = aud_LoudspeakerWithEarpiece(TRUE, codec_data);
	} else {
		ret = aud_LoudspeakerWithEarpiece(FALSE, codec_data);
	}

	return (ret!=0?-1:0);
}
static int rda_codec_force_mainmic(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0, on = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	on = ucontrol->value.integer.value[0];

	ret = aud_ForceReceiverMicSelection(on, codec_data);

	return (ret!=0?-1:0);
}
static int rda_codec_mute_mic(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u32 mute = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	mute = ucontrol->value.integer.value[0];

	codec_data->cfg.micLevel = mute;

	return 0;
}
static int rda_codec_set_commit_setup(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	ret = aud_Setup(codec_data->itf, &(codec_data->cfg), codec_data);

	return (ret!=0?-1:0);
}

static int rda_audio_loop_mode(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	AUD_TEST_T mode = AUD_TEST_NO;
	SND_ITF_T itf = SND_ITF_LOUD_SPEAKER;
	HAL_AIF_STREAM_T stream;
	AUD_LEVEL_T cfg;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	// 1. mode
	mode = ucontrol->value.integer.value[0];
	// 2. itf & cfg
	switch(mode) {
		case AUD_TEST_HEADSETMIC_IN_HEADSET_OUT:
			itf = SND_ITF_EAR_PIECE;
			mode = AUD_TEST_SIDE_TEST;
			cfg.micLevel = SND_MIC_ENABLE;
			cfg.spkLevel = SND_SPK_VOL_6;
			cfg.toneLevel = 0;
			cfg.sideLevel = 0;
			break;
		case AUD_TEST_RECVMIC_IN_EARPIECE_OUT:
			itf = SND_ITF_EAR_PIECE;
			mode = AUD_TEST_SIDE_TEST;
			cfg.micLevel = SND_MIC_ENABLE;
			cfg.spkLevel = SND_SPK_VOL_6;
			cfg.toneLevel = 0;
			cfg.sideLevel = 0;
			break;
		case AUD_TEST_MAINMIC_IN_RECEIVER_OUT:
			itf = SND_ITF_RECEIVER;
			mode = AUD_TEST_SIDE_TEST;
			cfg.micLevel = SND_MIC_ENABLE;
			cfg.spkLevel = SND_SPK_VOL_6;
			cfg.toneLevel = 0;
			cfg.sideLevel = 0;
			break;
		case AUD_TEST_SIDE_TEST:
			itf = SND_ITF_LOUD_SPEAKER;
			mode = AUD_TEST_SIDE_TEST;
			cfg.micLevel = SND_MIC_ENABLE;
			cfg.spkLevel = SND_SPK_VOL_1;
			cfg.toneLevel = 0;
			cfg.sideLevel = 0;
			break;
		case AUD_TEST_NO:
		default:
			itf = codec_data->loop_mode_itf;
			cfg.micLevel = SND_MIC_MUTE;
			cfg.spkLevel = SND_SPK_MUTE;
			cfg.toneLevel = 0;
			cfg.sideLevel = 0;
			break;
	}

	codec_data->loop_mode_itf = itf;
	// 3. stream
	stream.startAddress = NULL;
	stream.length       = 0;
	stream.sampleRate   = HAL_AIF_FREQ_8000HZ;
	stream.channelNb    = HAL_AIF_MONO;
	stream.voiceQuality = FALSE;
	stream.playSyncWithRecord = FALSE;
	stream.halfHandler  = NULL;
	stream.endHandler   = NULL;

	ret = aud_TestModeSetup(itf, &stream, &cfg, mode, codec_data);

	return (ret!=0?-1:0);
}

/*
 * rda_codec_data register cache
 */
static const u16 rda_codec_reg[] = {
	0x0097, 0x0097, 0x00F9, 0x00F9,	/* 0 */
	0x001A, 0x0004, 0x0007, 0x0001,	/* 4 */
	0x0020, 0x0000, 0x0000, 0x0000,	/* 8 */
	0x0000, 0x0000, 0x0000, 0x0000,	/* 12 */
};

static const struct snd_kcontrol_new rda_codec_snd_controls[] = {
	// volumes ctrls
	SOC_SINGLE_EXT(MIXER_PLAYBACK_VOLUME, 0, 0, 0xFFFF, 0, 
			ctrl_ext_get_reg, rda_codec_set_playback_volume),
	SOC_SINGLE_EXT(MIXER_CAPTURE_VOLUME, 0, 0, 0xFFFF, 0, 
			ctrl_ext_get_reg, rda_codec_set_capture_volume),
	// devices ctrls
	SOC_SINGLE_EXT(MIXER_ITF, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_set_itf),
	SOC_SINGLE_EXT(MIXER_SPK_SEL, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_set_spksel),
	SOC_SINGLE_EXT(MIXER_FORCE_MAINMIC, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_force_mainmic),
	SOC_SINGLE_EXT(MIXER_CODEC_APP_MODE, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_codec_app_mode),
	SOC_SINGLE_EXT(MIXER_START_PLAY, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_start_play),
	SOC_SINGLE_EXT(MIXER_START_RECORD, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_start_record),
	SOC_SINGLE_EXT(MIXER_STOP, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_stop),
	SOC_SINGLE_EXT(MIXER_OUT_SAMPLE_RATE, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_set_out_sample_rate),
	SOC_SINGLE_EXT(MIXER_OUT_CHANNEL_NB, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_set_out_channel_number),
	SOC_SINGLE_EXT(MIXER_IN_SAMPLE_RATE, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_set_in_sample_rate),
	SOC_SINGLE_EXT(MIXER_IN_CHANNEL_NB, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_set_in_channel_number),
	SOC_SINGLE_EXT(MIXER_MUTE_MIC, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_mute_mic),
	SOC_SINGLE_EXT(MIXER_COMMIT_SETUP, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_codec_set_commit_setup),
	// status
	SOC_SINGLE_EXT(MIXER_CODEC_OPEN_STATUS, 0, 0, 0xFFFF, 0,
			rda_codec_get_open_status, ctrl_ext_set_reg),
	// factory mode ctrls
	SOC_SINGLE_EXT(MIXER_LOOP_MODE, 0, 0, 0xFFFF, 0,
			ctrl_ext_get_reg, rda_audio_loop_mode),
};


static int rda_codec_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int ret = 0;
	struct rda_codec_data *codec_data = snd_soc_codec_get_drvdata(codec);

	u32 sample_rate = params_rate(params);

	codec_data->stream.sampleRate = sample_rate;

	return ret;
}

static int rda_codec_dai_pcm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	// do this in user space
	return 0;
}

static int rda_codec_dai_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			break;
		case SNDRV_PCM_TRIGGER_RESUME:
			break;
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			break;

		case SNDRV_PCM_TRIGGER_STOP:
			break;
		case SNDRV_PCM_TRIGGER_SUSPEND:
			break;
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}

static int rda_codec_dai_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{

	return 0;
}

static void rda_codec_dai_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
}

static int rda_codec_dai_dig_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int rda_codec_dai_set_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}

static int rda_codec_dai_set_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int rda_codec_set_bias_level(struct snd_soc_codec *codec,
		enum snd_soc_bias_level level)
{

	switch (level) {
		case SND_SOC_BIAS_ON:
			break;
		case SND_SOC_BIAS_PREPARE:
			break;
		case SND_SOC_BIAS_STANDBY:
			break;
		case SND_SOC_BIAS_OFF:
			break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

#define RDA_CODEC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
		SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops rda_codec_dai_ops = {
	.startup = rda_codec_dai_startup,
	.hw_params = rda_codec_dai_hw_params,
	.prepare = rda_codec_dai_pcm_prepare,
	.trigger = rda_codec_dai_trigger,
	.shutdown = rda_codec_dai_shutdown,
	.digital_mute = rda_codec_dai_dig_mute,
	.set_fmt = rda_codec_dai_set_fmt,
	.set_sysclk = rda_codec_dai_set_sysclk,
};

static struct snd_soc_dai_driver rda_codec_dai_driver = {
	.name = "rda-codec-dai",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = RDA_CODEC_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = RDA_CODEC_FORMATS,
	},
	.ops = &rda_codec_dai_ops,
};

static int rda_codec_suspend(struct snd_soc_codec *codec)
{
	rda_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int rda_codec_resume(struct snd_soc_codec *codec)
{
	snd_soc_cache_sync(codec);
	rda_codec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

// just like hw_write in soc-io.c
static int rda_codec_hw_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	return 0;
}

// just like hw_write in soc-io.c
static unsigned int rda_codec_hw_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	return 0;
}

static int rda_modem_codec_notify(struct notifier_block *nb, unsigned long mesg, void *data)
{
	struct msys_device *pmsys_dev = container_of(nb, struct msys_device, notifier);
	struct rda_codec_data *codec_data = (struct rda_codec_data *)pmsys_dev->private;
	struct client_mesg *pmesg = (struct client_mesg *)data;
	struct snd_soc_codec* codec = NULL;

	if(codec_data != NULL)
		codec = codec_data->codec;

	if (pmesg->mod_id != SYS_GEN_MOD) {
		return NOTIFY_DONE;
	}

	if (mesg != SYS_GEN_MESG_RTC_TRIGGER) {
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static int rda_codec_probe(struct snd_soc_codec *codec)
{
	struct rda_codec_data *codec_data = NULL;

	codec_data = snd_soc_codec_get_drvdata(codec);

	if(codec_data == NULL) {
		printk(KERN_INFO"NULL codec_data is when probe, error \n");
		return -1;
	}

	codec_data->codec = codec;

	snd_soc_add_codec_controls(codec, rda_codec_snd_controls,
			ARRAY_SIZE(rda_codec_snd_controls));

	return 0;
}

static int rda_codec_remove(struct snd_soc_codec *codec)
{
	struct rda_codec_data* codec_data = NULL;

	codec_data = snd_soc_codec_get_drvdata(codec);

	rda_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_rda_codec_driver = {
	.reg_cache_size = ARRAY_SIZE(rda_codec_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = rda_codec_reg,
	.read = rda_codec_hw_read,
	.write = rda_codec_hw_write,
	.probe = rda_codec_probe,
	.remove = rda_codec_remove,
	.suspend = rda_codec_suspend,
	.resume = rda_codec_resume,
	.set_bias_level = rda_codec_set_bias_level,
	.dapm_widgets = NULL,
	.num_dapm_widgets = 0,
	.dapm_routes = NULL,
	.num_dapm_routes = 0,
};

static int rda_codec_platform_probe(struct platform_device *pdev)
{
	struct rda_codec_data *codec_data = NULL;
	int ret = 0;
	struct resource *extpa_res,*extpa1_res;

	codec_data = devm_kzalloc(&pdev->dev, sizeof(struct rda_codec_data), GFP_KERNEL);

	if (codec_data == NULL) {
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, codec_data);

	// ap <---> modem codec
	codec_data->codec_msys = rda_msys_alloc_device();
	if (!codec_data->codec_msys) {
		ret = -ENOMEM;
	}

	codec_data->codec_msys->module = SYS_AUDIO_MOD;
	codec_data->codec_msys->name = "rda-codec";
	codec_data->codec_msys->notifier.notifier_call = rda_modem_codec_notify;
	codec_data->codec_msys->private = (void *)codec_data;

	rda_msys_register_device(codec_data->codec_msys);

	ret = snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_rda_codec_driver,
			&rda_codec_dai_driver, 1);
	extpa_res = platform_get_resource_byname(pdev,IORESOURCE_MEM,"audio-extpa");
	if (extpa_res->start > 0) {
		gpio_audio_extpa = extpa_res->start;
		printk(KERN_INFO"get extpa resource succeed %d\n",gpio_audio_extpa);
		ret = gpio_request(gpio_audio_extpa,"audio-extpa");
		if (ret < 0) {
			printk(KERN_ERR"rda codec : gpio_request fail. ");
			goto err_request_gpio;
		}
		ret = gpio_direction_output(gpio_audio_extpa,1);
		if (ret < 0) {
			printk(KERN_ERR"rda codec : gpio_direction_output fail.");
			goto err_request_gpio;
		}
	} else
		printk(KERN_INFO"FAILED TO GET EXTPA RESOURCE !");

	extpa1_res = platform_get_resource_byname(pdev,IORESOURCE_MEM,"audio-extpa-1");
	if (extpa1_res->start > 0) {
		gpio_audio_extpa_1 = extpa1_res->start;
		printk(KERN_INFO"get extpa_1 resource succeed %d\n",gpio_audio_extpa_1);
		ret = gpio_request(gpio_audio_extpa_1,"audio-extpa-1");
		if (ret < 0) {
			printk(KERN_ERR"rda codec : gpio_request 1 fail. ");
			goto err_request_gpio1;
		}
		ret = gpio_direction_output(gpio_audio_extpa_1,1);
		if (ret < 0) {
			printk(KERN_ERR"rda codec : gpio_direction_output 1 fail.");
			goto err_request_gpio1;
		}
	} else
		printk(KERN_INFO"FAILED TO GET EXTPA_1 RESOURCE !");

	return ret;
err_request_gpio:
	gpio_free(gpio_audio_extpa);
err_request_gpio1:
	gpio_free(gpio_audio_extpa);
	gpio_free(gpio_audio_extpa_1);

	return ret;
}

static int __exit rda_codec_platform_remove(struct platform_device *pdev)
{
	struct rda_codec_data *codec_data = platform_get_drvdata(pdev);

	snd_soc_unregister_codec(&pdev->dev);

	rda_msys_unregister_device(codec_data->codec_msys);
	rda_msys_free_device(codec_data->codec_msys);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver rda_codec_driver = {
	.driver = {
		.name = "rda-codec",
		.owner = THIS_MODULE,
	},

	.probe = rda_codec_platform_probe,
	.remove = __exit_p(rda_codec_platform_remove),
};

static int __init rda_codec_modinit(void)
{
	return platform_driver_register(&rda_codec_driver);
}

static void __exit rda_codec_modexit(void)
{
	platform_driver_unregister(&rda_codec_driver);
}

static void __exit rdafpag_pcm_modexit(void)
{
	platform_driver_unregister(&rda_codec_driver);
}

module_init(rda_codec_modinit);
module_exit(rda_codec_modexit);

MODULE_DESCRIPTION("ASoC RDA codec driver");
MODULE_AUTHOR("Arun KS <arunks@mistralsolutions.com>");
MODULE_LICENSE("GPL");
