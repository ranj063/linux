// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// Copyright 2025 Intel Corporation. All rights reserved.
//
#include <sound/soc.h>
#include <sound/sof.h>
#include <sound/compress_driver.h>
#include <sound/pcm_params.h>
#include "sof-audio.h"
#include "sof-priv.h"
#include "sof-utils.h"
#include "ops.h"

static int sof_ipc4_compr_open(struct snd_soc_component *component,
                               struct snd_compr_stream *cstream)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_compr_runtime *crtd = cstream->runtime;
	struct sof_compr_stream *sstream;
	struct snd_sof_pcm *spcm;
	int dir, ret;

	sstream = kzalloc(sizeof(*sstream), GFP_KERNEL);
	if (!sstream)
		return -ENOMEM;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm) {
		kfree(sstream);
		return -EINVAL;
	}

	dir = cstream->direction;

	if (spcm->stream[dir].cstream) {
		kfree(sstream);
		return -EBUSY;
	}

	spcm->stream[dir].cstream = cstream;
	spcm->stream[dir].posn.host_posn = 0;
	spcm->stream[dir].posn.dai_posn = 0;
	spcm->prepared[dir] = false;

	crtd->private_data = sstream;

	ret = snd_sof_compress_platform_open(sdev, cstream);
	if (ret < 0) {
		dev_err(component->dev, "platform compress open failed %d\n", ret);
		return ret;
	}

	return 0;
}

static int sof_ipc4_compress_stream_free(struct snd_sof_dev *sdev,
                                         struct snd_sof_pcm *spcm, int dir,
                                         bool free_widget_list)
{
	const struct sof_ipc_pcm_ops *pcm_ops = sof_ipc_get_ops(sdev, pcm);
	int ret = 0;
	int err = 0;

	if (pcm_ops && pcm_ops->hw_free) {
		err = pcm_ops->hw_free(sdev->component, NULL, spcm, dir);
		if (err < 0)
			dev_err(sdev->dev, "pcm_ops->hw_free failed %d\n", ret);
	}

	/* free widget list */
	if (free_widget_list) {
		ret = sof_widget_list_free(sdev, spcm, dir);
		if (ret < 0 && err == 0) {
			dev_err(sdev->dev, "sof_widget_list_free failed %d\n", ret);
			err = ret;
		}
	}

	spcm->prepared[dir] = false;
	spcm->stream[dir].cstream = NULL;

	/* unprepare and free the list of DAPM widgets */
        sof_widget_list_unprepare(sdev, spcm, dir);

        cancel_work_sync(&spcm->stream[dir].period_elapsed_work);

	return err;
}

static int sof_ipc4_compr_free(struct snd_soc_component *component,
                               struct snd_compr_stream *cstream)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_compr_runtime *crtd = cstream->runtime;
	struct snd_sof_pcm *spcm;
	int ret, err;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm)
		return -EINVAL;

	ret = sof_ipc4_compress_stream_free(sdev, spcm, cstream->direction, true);

	err = snd_sof_compress_platform_close(sdev, cstream);
	if (err < 0) {
		dev_err(component->dev, "platform compress close failed %d\n", ret);
		if (!ret)
			ret = err;
	}
	kfree(crtd->private_data);
	crtd->private_data = NULL;

	return err;
}

static int sof_ipc4_compr_set_params(struct snd_soc_component *component,
                                     struct snd_compr_stream *cstream, struct snd_compr_params *params)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_platform_stream_params *platform_params;
	const struct sof_ipc_tplg_ops *tplg_ops = sof_ipc_get_ops(sdev, tplg);
	struct snd_compr_runtime *crtd = cstream->runtime;
	struct snd_interval *channels_interval;
	struct snd_soc_dapm_widget_list *list;
	struct snd_interval *rate_interval;
	struct snd_sof_widget *host_widget;
	struct snd_pcm_hw_params p = {0};
	struct snd_sof_pcm *spcm;
	struct snd_mask *fmt;
	int host_comp_id;
	int ret;

	/*
	 * Force format, rate and channels and use PCM hw_params structure to
	 * set up the pipelines. TODO: Should come from the codec params
	 */

	fmt = hw_param_mask(&p, SNDRV_PCM_HW_PARAM_FORMAT);
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S32_LE);

	channels_interval = hw_param_interval(&p, SNDRV_PCM_HW_PARAM_CHANNELS);
	channels_interval->min = 2;
	channels_interval->max = 2;

	rate_interval = hw_param_interval(&p, SNDRV_PCM_HW_PARAM_RATE);
	rate_interval->min = rate_interval->max = 48000;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm)
		return -EINVAL;

	/* save the compress params */
	memcpy(&spcm->compress_params[cstream->direction], params, sizeof(*params));

	cstream->dma_buffer.dev.type = SNDRV_DMA_TYPE_DEV_SG;
	cstream->dma_buffer.dev.dev = sdev->dev;

	ret = snd_compr_malloc_pages(cstream, crtd->buffer_size);
	if (ret < 0)
		return ret;

	ret = snd_sof_compr_create_page_table(component, cstream, crtd->dma_area, crtd->dma_bytes);
	if (ret < 0)
		return ret;

	platform_params = &spcm->platform_params[cstream->direction];
	ret = snd_sof_compress_platform_hw_params(sdev, cstream, params, platform_params);
	if (ret < 0) {
		dev_err(component->dev, "platform compress hw params failed\n");
		return ret;
	}

	/*
	 * Make sure that the DSP is booted up, which might not be the
	 * case if the on-demand DSP boot is used
	 */
	ret = snd_sof_boot_dsp_firmware(sdev);
	if (ret)
		return ret;

	/* set up the list of DAPM widgets if not already done */
	if (!spcm->stream[cstream->direction].list) {
		ret = sof_pcm_setup_connected_widgets(sdev, rtd, spcm, &p, platform_params,
						      cstream->direction);
		if (ret < 0)
			return ret;
	}

	/* set the host DMA ID */
	host_comp_id = spcm->stream[cstream->direction].comp_id;
	host_widget = snd_sof_find_swidget_by_comp_id(sdev, host_comp_id);
	if (!host_widget) {
		dev_err(component->dev, "failed to find host widget with comp_id %d\n", host_comp_id);
		return -EINVAL;
	}

	if (tplg_ops && tplg_ops->host_config)
		tplg_ops->host_config(sdev, host_widget, platform_params);

	/* set up the widgets and pipelines in the DSP */
	list = spcm->stream[cstream->direction].list;
	ret = sof_widget_list_setup(sdev, spcm, &p, platform_params, cstream->direction);
	if (ret < 0) {
		dev_err(sdev->dev, "failed widget list set up for compress streaM: %d dir: %d\n",
			spcm->pcm.pcm_id, cstream->direction);
		spcm->stream[cstream->direction].list = NULL;
		snd_soc_dapm_dai_free_widgets(&list);
		return ret;
	}

	memcpy(&spcm->params[cstream->direction], &p, sizeof(p));
	spcm->prepared[cstream->direction] = true;

	return 0;
}

static int sof_ipc4_compr_get_params(struct snd_soc_component *component,
                                     struct snd_compr_stream *cstream, struct snd_codec *params)
{
	/* TODO: we don't query the supported codecs for now, if the
	 * application asks for an unsupported codec the set_params() will fail.
	 */
	return 0;
}

static int sof_ipc4_compr_trigger(struct snd_soc_component *component,
                                  struct snd_compr_stream *cstream, int cmd)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	const struct sof_ipc_pcm_ops *pcm_ops = sof_ipc_get_ops(sdev, pcm);
	struct snd_sof_pcm *spcm;
	int ret;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm) {
		dev_dbg(sdev->dev, "sof_compr_trigger: can't find spcm\n");
		return -EINVAL;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (pcm_ops && pcm_ops->trigger) {
			ret = pcm_ops->trigger(component, NULL, spcm, cmd, cstream->direction);
			if (ret < 0) {
				dev_err(sdev->dev, "pcm_ops->trigger failed for cmd %d\n", cmd);
				return ret;
			}
		}

		ret = snd_sof_compress_platform_trigger(sdev, cstream, cmd);
		if (ret < 0) {
			dev_err(sdev->dev, "platform compress trigger start failed %d\n", ret);
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ret = snd_sof_compress_platform_trigger(sdev, cstream, cmd);
		if (ret < 0) {
			dev_err(sdev->dev, "platform compress trigger stop failed %d\n", ret);
			return ret;
		}

		if (pcm_ops && pcm_ops->trigger) {
			ret = pcm_ops->trigger(component, NULL, spcm, cmd, cstream->direction);
			if (ret < 0) {
				dev_err(sdev->dev, "pcm_ops->trigger failed for cmd %d\n", cmd);
				return ret;
			}
		}
		break;
	default:
		break;
	}

	return 0;
}

static int sof_ipc4_compr_copy_playback(struct snd_soc_component *component,
                                        struct snd_compr_stream *cstream,
                                        char __user *buf, size_t count)
{
	struct snd_compr_runtime *rtd = cstream->runtime;
	void *ptr;
	unsigned int offset, n;
	int ret;

	div_u64_rem(rtd->total_bytes_available, rtd->buffer_size, &offset);
	ptr = rtd->dma_area + offset;
	n = rtd->buffer_size - offset;

	if (count < n) {
		ret = copy_from_user(ptr, buf, count);
	} else {
		ret = copy_from_user(ptr, buf, n);
		ret += copy_from_user(rtd->dma_area, buf + n, count - n);
	}

	return count - ret;
}

static int sof_ipc4_compr_copy_capture(struct snd_compr_runtime *rtd,
                                       char __user *buf, size_t count)
{
	void *ptr;
	unsigned int offset, n;
	int ret;

	div_u64_rem(rtd->total_bytes_transferred, rtd->buffer_size, &offset);
	ptr = rtd->dma_area + offset;
	n = rtd->buffer_size - offset;

	if (count < n) {
		ret = copy_to_user(buf, ptr, count);
	} else {
		ret = copy_to_user(buf, ptr, n);
		ret += copy_to_user(buf + n, rtd->dma_area, count - n);
	}

	return count - ret;
}

static int sof_ipc4_compr_copy(struct snd_soc_component *component,
			       struct snd_compr_stream *cstream,
			       char __user *buf, size_t count)
{
	struct snd_compr_runtime *rtd = cstream->runtime;

	if (count > rtd->buffer_size)
		count = rtd->buffer_size;

	if (cstream->direction == SND_COMPRESS_PLAYBACK)
		return sof_ipc4_compr_copy_playback(component, cstream, buf, count);
	else
		return sof_ipc4_compr_copy_capture(rtd, buf, count);
}

static int sof_ipc4_compr_pointer(struct snd_soc_component *component,
                                  struct snd_compr_stream *cstream,
                                  struct snd_compr_tstamp64 *tstamp)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_sof_pcm *spcm;
	int ret;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm)
		return -EINVAL;

	struct snd_pcm_hw_params *params = &spcm->params[cstream->direction];

	ret = snd_sof_compress_platform_pointer(sdev, cstream, tstamp);
	if (ret < 0) {
		dev_err(sdev->dev, "platform compress pointer failed %d\n", ret);
		return ret;
	}

	tstamp->sampling_rate = params_rate(params);

	/* TODO: Get DAI position from Link DMA */
	tstamp->pcm_io_frames = div_u64(spcm->stream[cstream->direction].posn.dai_posn,
					params_channels(params) * params_physical_width(params));

	return 0;
}

const struct snd_compress_ops sof_ipc4_compressed_ops = {
	.open		= sof_ipc4_compr_open,
	.free		= sof_ipc4_compr_free,
	.set_params	= sof_ipc4_compr_set_params,
	.get_params	= sof_ipc4_compr_get_params,
	.trigger	= sof_ipc4_compr_trigger,
	.pointer	= sof_ipc4_compr_pointer,
	.copy		= sof_ipc4_compr_copy,
};