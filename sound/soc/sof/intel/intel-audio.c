// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2019 Intel Corporation. All rights reserved.
//
// Authors: Ranjani Sridharan <ranjani.sridharan@linux.intel.com>

/* Intel-specific SOF audio client code */
#include "../sof-mfd.h"
#include "../sof-priv.h"
#include "shim.h"
#include "intel-audio.h"

int intel_register_audio_clients(struct snd_sof_dev *sdev)
{
	struct snd_sof_pdata *plat_data = sdev->pdata;
	const struct sof_intel_dsp_desc *chip = get_chip_info(plat_data);
	const struct sof_dev_desc *desc = plat_data->desc;
	size_t size;
	int i = 0;

	/* number of audio clients to register */
	if (chip->num_ssp_drv)
		sdev->sof_num_clients++;

	if (chip->num_dmic_drv)
		sdev->sof_num_clients++;

	if (chip->num_hda_drv)
		sdev->sof_num_clients++;

	size = sdev->sof_num_clients * sizeof(*sdev->sof_clients);

	sdev->sof_clients = devm_kzalloc(sdev->dev, size, GFP_KERNEL);
	if (!sdev->sof_clients)
		return -ENOMEM;

	/* register clients. Any of these can fail but the core keeps going */
	if (chip->num_ssp_drv && i < sdev->sof_num_clients) {
		sdev->sof_clients[i] =
			sof_client_dev_register(sdev, "sof-ssp-audio");
		if (!sdev->sof_clients[i])
			dev_warn(sdev->dev,
				 "sof-ssp-audio client failed to register\n");
	}
	i++;

	if (chip->num_dmic_drv && i < sdev->sof_num_clients) {
		sdev->sof_clients[i] =
			sof_client_dev_register(sdev, "sof-dmic-audio");
		if (!sdev->sof_clients[i])
			dev_warn(sdev->dev,
				 "sof-dmic-audio client failed to register\n");
	}
	i++;

	if (chip->num_hda_drv && i < sdev->sof_num_clients) {
		sdev->sof_clients[i] =
			sof_client_dev_register(sdev, "sof-hda-audio");
		if (!sdev->sof_clients[i])
			dev_warn(sdev->dev,
				 "sof-hda-audio client failed to register\n");
	}

	return 0;
}
EXPORT_SYMBOL(intel_register_audio_clients);

int intel_unregister_audio_clients(struct snd_sof_dev *sdev)
{
	int i;

	for (i = 0; i < sdev->sof_num_clients; i++)
		sof_client_dev_unregister(sdev->sof_clients[i]);

	return 0;
}
EXPORT_SYMBOL(intel_unregister_audio_clients);

const struct sof_intel_dsp_desc *get_chip_info(struct snd_sof_pdata *pdata)
{
	return pdata->desc->chip_info;
}
EXPORT_SYMBOL(get_chip_info);

MODULE_LICENSE("Dual BSD/GPL");
