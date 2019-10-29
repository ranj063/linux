// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// Copyright(c) 2019 Intel Corporation. All rights reserved.
//
// Author: Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
//

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <sound/sof.h>
#include "sof-priv.h"
#include "sof-client.h"
#include "sof-audio.h"
#include "ops.h"

/*
 * SOF Driver enumeration.
 */
static int sof_machine_check(struct snd_sof_dev *sdev)
{
	struct snd_sof_pdata *plat_data = sdev->pdata;
#if IS_ENABLED(CONFIG_SND_SOC_SOF_NOCODEC)
	struct snd_soc_acpi_mach *machine;
	int ret;
#endif

	if (plat_data->machine)
		return 0;

#if !IS_ENABLED(CONFIG_SND_SOC_SOF_NOCODEC)
	dev_err(sdev->dev, "error: no matching ASoC machine driver found - aborting probe\n");
	return -ENODEV;
#else
	/* fallback to nocodec mode */
	dev_warn(sdev->dev, "No ASoC machine driver found - using nocodec\n");
	machine = devm_kzalloc(sdev->dev, sizeof(*machine), GFP_KERNEL);
	if (!machine)
		return -ENOMEM;

	ret = sof_nocodec_setup(sdev->dev, plat_data, machine,
				plat_data->desc, plat_data->desc->ops);
	if (ret < 0)
		return ret;

	plat_data->machine = machine;

	return 0;
#endif
}

static int sof_set_hw_params_upon_resume(struct device *dev)
{
	struct snd_sof_dev *sdev = dev_get_drvdata(dev->parent);
	struct snd_pcm_substream *substream;
	struct snd_sof_pcm *spcm;
	snd_pcm_state_t state;
	int dir;

	/*
	 * SOF requires hw_params to be set-up internally upon resume.
	 * So, set the flag to indicate this for those streams that
	 * have been suspended.
	 */
	list_for_each_entry(spcm, &sdev->pcm_list, list) {
		for (dir = 0; dir <= SNDRV_PCM_STREAM_CAPTURE; dir++) {
			substream = spcm->stream[dir].substream;
			if (!substream || !substream->runtime)
				continue;

			state = substream->runtime->status->state;
			if (state == SNDRV_PCM_STATE_SUSPENDED)
				spcm->prepared[dir] = false;
		}
	}

	/* set internal flag for BE */
	return snd_sof_dsp_hw_params_upon_resume(sdev);
}

static int sof_restore_kcontrols(struct device *dev)
{
	struct sof_audio_dev *sof_audio = sof_get_client_data(dev);
	struct snd_sof_dev *sdev = dev_get_drvdata(dev->parent);
	struct snd_sof_control *scontrol;
	int ipc_cmd, ctrl_type;
	int ret = 0;

	/* restore kcontrol values */
	list_for_each_entry(scontrol, &sof_audio->kcontrol_list, list) {
		/* reset readback offset for scontrol after resuming */
		scontrol->readback_offset = 0;

		/* notify DSP of kcontrol values */
		switch (scontrol->cmd) {
		case SOF_CTRL_CMD_VOLUME:
		case SOF_CTRL_CMD_ENUM:
		case SOF_CTRL_CMD_SWITCH:
			ipc_cmd = SOF_IPC_COMP_SET_VALUE;
			ctrl_type = SOF_CTRL_TYPE_VALUE_CHAN_SET;
			ret = snd_sof_ipc_set_get_comp_data(scontrol,
							    ipc_cmd, ctrl_type,
							    scontrol->cmd,
							    true);
			break;
		case SOF_CTRL_CMD_BINARY:
			ipc_cmd = SOF_IPC_COMP_SET_DATA;
			ctrl_type = SOF_CTRL_TYPE_DATA_SET;
			ret = snd_sof_ipc_set_get_comp_data(scontrol,
							    ipc_cmd, ctrl_type,
							    scontrol->cmd,
							    true);
			break;

		default:
			break;
		}

		if (ret < 0) {
			dev_err(dev,
				"error: failed kcontrol value set for widget: %d\n",
				scontrol->comp_id);

			return ret;
		}
	}

	return 0;
}

static int sof_restore_pipelines(struct device *dev)
{
	struct sof_audio_dev *sof_audio = sof_get_client_data(dev);
	struct snd_sof_dev *sdev = dev_get_drvdata(dev->parent);
	struct snd_sof_widget *swidget;
	struct snd_sof_route *sroute;
	struct sof_ipc_pipe_new *pipeline;
	struct snd_sof_dai *dai;
	struct sof_ipc_comp_dai *comp_dai;
	struct sof_ipc_cmd_hdr *hdr;
	int ret;

	/* restore pipeline components */
	list_for_each_entry_reverse(swidget, &sdev->widget_list, list) {
		struct sof_ipc_comp_reply r;

		/* skip if there is no private data */
		if (!swidget->private)
			continue;

		switch (swidget->id) {
		case snd_soc_dapm_dai_in:
		case snd_soc_dapm_dai_out:
			dai = swidget->private;
			comp_dai = &dai->comp_dai;
			ret = sof_client_tx_message(dev,
						    comp_dai->comp.hdr.cmd,
						    comp_dai, sizeof(*comp_dai),
						    &r, sizeof(r));
			break;
		case snd_soc_dapm_scheduler:

			/*
			 * During suspend, all DSP cores are powered off.
			 * Therefore upon resume, create the pipeline comp
			 * and power up the core that the pipeline is
			 * scheduled on.
			 */
			pipeline = swidget->private;
			ret = sof_load_pipeline_ipc(sdev, pipeline, &r);
			break;
		default:
			hdr = swidget->private;
			ret = sof_client_tx_message(dev, hdr->cmd,
						    swidget->private, hdr->size,
						    &r, sizeof(r));
			break;
		}
		if (ret < 0) {
			dev_err(dev,
				"error: failed to load widget type %d with ID: %d\n",
				swidget->widget->id, swidget->comp_id);

			return ret;
		}
	}

	/* restore pipeline connections */
	list_for_each_entry_reverse(sroute, &sof_audio->route_list, list) {
		struct sof_ipc_pipe_comp_connect *connect;
		struct sof_ipc_reply reply;

		/* skip if there's no private data */
		if (!sroute->private)
			continue;

		connect = sroute->private;

		/* send ipc */
		ret = sof_client_tx_message(dev,
					    connect->hdr.cmd,
					    connect, sizeof(*connect),
					    &reply, sizeof(reply));
		if (ret < 0) {
			dev_err(dev,
				"error: failed to load route sink %s control %s source %s\n",
				sroute->route->sink,
				sroute->route->control ? sroute->route->control
					: "none",
				sroute->route->source);

			return ret;
		}
	}

	/* restore dai links */
	list_for_each_entry_reverse(dai, &sdev->dai_list, list) {
		struct sof_ipc_reply reply;
		struct sof_ipc_dai_config *config = dai->dai_config;

		if (!config) {
			dev_err(dev, "error: no config for DAI %s\n",
				dai->name);
			continue;
		}

		/*
		 * The link DMA channel would be invalidated for running
		 * streams but not for streams that were in the PAUSED
		 * state during suspend. So invalidate it here before setting
		 * the dai config in the DSP.
		 */
		if (config->type == SOF_DAI_INTEL_HDA)
			config->hda.link_dma_ch = DMA_CHAN_INVALID;

		ret = sof_client_tx_message(dev,
					    config->hdr.cmd, config,
					    config->hdr.size,
					    &reply, sizeof(reply));

		if (ret < 0) {
			dev_err(dev,
				"error: failed to set dai config for %s\n",
				dai->name);

			return ret;
		}
	}

	/* complete pipeline */
	list_for_each_entry(swidget, &sdev->widget_list, list) {
		switch (swidget->id) {
		case snd_soc_dapm_scheduler:
			swidget->complete =
				snd_sof_complete_pipeline(sdev, swidget);
			break;
		default:
			break;
		}
	}

	/* restore pipeline kcontrols */
	ret = sof_restore_kcontrols(dev);
	if (ret < 0)
		dev_err(dev,
			"error: restoring kcontrols after resume\n");

	return ret;
}

static int sof_destroy_pipelines(struct device *dev)
{
	struct snd_sof_dev *sdev = dev_get_drvdata(dev->parent);
	struct snd_sof_widget *swidget;
	struct sof_ipc_free ipc_free;
	struct sof_ipc_reply reply;
	int ret = 0;

	list_for_each_entry_reverse(swidget, &sdev->widget_list, list) {
		memset(&ipc_free, 0, sizeof(ipc_free));

		/* skip if there is no private data */
		if (!swidget->private)
			continue;

		/* configure ipc free message */
		ipc_free.hdr.size = sizeof(ipc_free);
		ipc_free.hdr.cmd = SOF_IPC_GLB_TPLG_MSG;
		ipc_free.id = swidget->comp_id;

		switch (swidget->id) {
		case snd_soc_dapm_scheduler:
			ipc_free.hdr.cmd |= SOF_IPC_TPLG_PIPE_FREE;
			break;
		case snd_soc_dapm_buffer:
			ipc_free.hdr.cmd |= SOF_IPC_TPLG_BUFFER_FREE;
			break;
		default:
			ipc_free.hdr.cmd |= SOF_IPC_TPLG_COMP_FREE;
			break;
		}
		ret = sof_client_tx_message(dev, ipc_free.hdr.cmd,
					    &ipc_free, sizeof(ipc_free), &reply,
					    sizeof(reply));
		if (ret < 0) {
			dev_err(dev,
				"error: failed to free widget type %d with ID: %d\n",
				swidget->widget->id, swidget->comp_id);

			return ret;
		}
	}

	return ret;
}

static int sof_audio_resume(struct device *dev)
{
	return sof_restore_pipelines(dev);
}

static int sof_audio_suspend(struct device *dev)
{
	return sof_set_hw_params_upon_resume(dev);
}

static int sof_audio_runtime_suspend(struct device *dev)
{
	return sof_destroy_pipelines(dev);
}

static const struct dev_pm_ops sof_audio_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(sof_audio_suspend, sof_audio_resume)
	SET_RUNTIME_PM_OPS(sof_audio_runtime_suspend, sof_audio_resume, NULL)
};

static int sof_audio_probe(struct platform_device *pdev)
{
	struct snd_sof_client *audio_client = dev_get_platdata(&pdev->dev);
	struct snd_sof_dev *sdev = dev_get_drvdata(pdev->dev.parent);
	struct snd_sof_pdata *plat_data = sdev->pdata;
	struct snd_soc_acpi_mach *machine =
		(struct snd_soc_acpi_mach *)plat_data->machine;
	struct sof_audio_dev *sof_audio;
	const char *drv_name;
	int size;
	int ret;

	/* create SOF audio device */
	sof_audio = devm_kzalloc(&pdev->dev, sizeof(*sof_audio), GFP_KERNEL);
	if (!sof_audio)
		return -ENOMEM;

	INIT_LIST_HEAD(&sof_audio->pcm_list);
	INIT_LIST_HEAD(&sof_audio->kcontrol_list);
	INIT_LIST_HEAD(&sof_audio->widget_list);
	INIT_LIST_HEAD(&sof_audio->dai_list);
	INIT_LIST_HEAD(&sof_audio->route_list);

	audio_client->client_data = sof_audio;

	/* check machine info */
	ret = sof_machine_check(sdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "error: failed to get machine info %d\n",
			ret);
		return ret;
	}

	/* set platform name */
	machine->mach_params.platform = dev_name(&pdev->dev);
	plat_data->platform = dev_name(&pdev->dev);

	/* set up platform component driver */
	snd_sof_new_platform_drv(sdev);

	/* now register audio DSP platform driver and dai */
	ret = devm_snd_soc_register_component(&pdev->dev, &sdev->plat_drv,
					      sof_ops(sdev)->drv,
					      sof_ops(sdev)->num_drv);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"error: failed to register DSP DAI driver %d\n", ret);
		return ret;
	}

	drv_name = plat_data->machine->drv_name;
	size = sizeof(*plat_data->machine);

	/* register machine driver, pass machine info as pdata */
	plat_data->pdev_mach =
		platform_device_register_data(&pdev->dev, drv_name,
					      PLATFORM_DEVID_NONE,
					      (const void *)machine, size);

	if (IS_ERR(plat_data->pdev_mach)) {
		ret = PTR_ERR(plat_data->pdev_mach);
		return ret;
	}

	dev_dbg(&pdev->dev, "created machine %s\n",
		dev_name(&plat_data->pdev_mach->dev));

	/* enable runtime PM */
	pm_runtime_set_autosuspend_delay(&pdev->dev, SND_SOF_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	return 0;
}

static int sof_audio_remove(struct platform_device *pdev)
{
	struct snd_sof_dev *sdev = dev_get_drvdata(pdev->dev.parent);
	struct snd_sof_pdata *pdata = sdev->pdata;

	pm_runtime_disable(&pdev->dev);

	if (!IS_ERR_OR_NULL(pdata->pdev_mach))
		platform_device_unregister(pdata->pdev_mach);

	return 0;
}

static struct platform_driver sof_audio_driver = {
	.driver = {
		.name = "sof-audio",
		.pm = &sof_audio_pm,
	},

	.probe = sof_audio_probe,
	.remove = sof_audio_remove,
};

module_platform_driver(sof_audio_driver);

MODULE_DESCRIPTION("SOF Audio Client Platform Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:sof-audio");
