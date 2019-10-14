/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright(c) 2019 Intel Corporation. All rights reserved.
 *
 * Author: Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
 */
#ifndef __SOUND_SOC_SOF_INTEL_AUDIO_H
#define __SOUND_SOC_SOF_INTEL_AUDIO_H

int intel_register_audio_clients(struct snd_sof_dev *sdev);
int intel_unregister_audio_clients(struct snd_sof_dev *sdev);

const struct sof_intel_dsp_desc *get_chip_info(struct snd_sof_pdata *pdata);

#endif
