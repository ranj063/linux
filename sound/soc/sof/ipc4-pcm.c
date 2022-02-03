// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2022 Intel Corporation. All rights reserved.
//

#include <sound/pcm_params.h>
#include <sound/sof/ipc4/header.h>
#include "sof-audio.h"
#include "sof-priv.h"
#include "ipc4-priv.h"
#include "ipc4-ops.h"
#include "ipc4-topology.h"

static int sof_ipc4_set_pipeline_status(struct snd_sof_dev *sdev, u32 id, u32 status)
{
	struct sof_ipc4_msg msg = {{ 0 }};
	u32 primary;

	dev_dbg(sdev->dev, "ipc4 set pipeline %d status %d", id, status);

	primary = status;
	primary |= SOF_IPC4_GL_PIPE_STATE_ID(id);
	primary |= SOF_IPC4_GLB_MSG_TYPE(SOF_IPC4_GLB_SET_PIPELINE_STATE);
	primary |= SOF_IPC4_GLB_MSG_DIR(SOF_IPC4_MSG_REQUEST);
	primary |= SOF_IPC4_GLB_MSG_TARGET(SOF_IPC4_FW_GEN_MSG);

	msg.primary = primary;

	return sof_ipc_tx_message(sdev->ipc, &msg, 0, NULL, 0);
}

static int sof_ipc4_pcm_trigger(struct snd_soc_component *component,
				struct snd_pcm_substream *substream, int cmd)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_sof_widget *pipeline_widget;
	struct snd_soc_dapm_widget_list *list;
	struct snd_soc_dapm_widget *widget;
	struct sof_ipc4_pipeline *pipeline;
	struct snd_sof_pcm *spcm;
	int num_widgets, state, ret;

	spcm = snd_sof_find_spcm_dai(component, rtd);
	if (!spcm)
		return -EINVAL;

	list = spcm->stream[substream->stream].list;

	/* set the pipeline state */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		state = SOF_IPC4_PIPE_PAUSED;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_START:
		state = SOF_IPC4_PIPE_RUNNING;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		state = SOF_IPC4_PIPE_RESET;
		break;
	default:
		dev_err(sdev->dev, "%s: unhandled trigger cmd %d\n", __func__, cmd);
		return -EINVAL;
	}

	for_each_dapm_widgets(list, num_widgets, widget) {
		struct snd_sof_widget *swidget = widget->dobj.private;

		if (!swidget)
			continue;

		/* find pipeline widget for the pipeline that this widget belongs to */
		pipeline_widget = swidget->pipe_widget;
		pipeline = (struct sof_ipc4_pipeline *)pipeline_widget->private;

		if (pipeline->state == state)
			continue;

		/* first set the pipeline state to PAUSED */
		if (pipeline->state == SOF_IPC4_PIPE_PAUSED)
			goto set;

		ret = sof_ipc4_set_pipeline_status(sdev, swidget->pipeline_id,
						   SOF_IPC4_PIPE_PAUSED);
		if (ret < 0) {
			dev_err(sdev->dev,
				"failed to set SOF_IPC4_PIPE_PAUSED state for pipeline %d\n",
				swidget->pipeline_id);
			break;
		}

		pipeline->state = SOF_IPC4_PIPE_PAUSED;
set:
		/* then set the final state */
		if (pipeline->state == state)
			continue;

		ret = sof_ipc4_set_pipeline_status(sdev, swidget->pipeline_id, state);
		if (ret < 0) {
			dev_err(sdev->dev, "failed to set state %d for pipeline %d\n",
				state, swidget->pipeline_id);
			break;
		}

		pipeline->state = state;
	}

	return 0;
}

static int sof_ipc4_pcm_dai_link_fixup(struct snd_soc_pcm_runtime *rtd,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_component *component = snd_soc_rtdcom_lookup(rtd, SOF_AUDIO_PCM_DRV_NAME);
	struct snd_sof_dai *dai = snd_sof_find_dai(component, (char *)rtd->dai_link->name);
	struct sof_ipc4_copier *copier = dai->private;
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	switch (copier->frame_fmt) {
	case SOF_IPC_FRAME_S16_LE:
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
		break;
	case SOF_IPC_FRAME_S24_4LE:
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S24_LE);
		break;
	case SOF_IPC_FRAME_S32_LE:
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S32_LE);
		break;
	default:
		dev_err(component->dev, "No available DAI format!\n");
		return -EINVAL;
	}

	return 0;
}

const struct ipc_pcm_ops ipc4_pcm_ops = {
	.trigger = sof_ipc4_pcm_trigger,
	.dai_link_fixup = sof_ipc4_pcm_dai_link_fixup,
};
