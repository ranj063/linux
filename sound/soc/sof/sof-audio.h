/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright(c) 2019 Intel Corporation. All rights reserved.
 *
 * Author: Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
 */

#ifndef __SOUND_SOC_SOF_AUDIO_H
#define __SOUND_SOC_SOF_AUDIO_H

struct snd_sof_led_control {
	unsigned int use_led;
	unsigned int direction;
	unsigned int led_value;
};

/* ALSA SOF Kcontrol device */
struct snd_sof_control {
	struct snd_soc_component *scomp;
	int comp_id;
	int min_volume_step; /* min volume step for volume_table */
	int max_volume_step; /* max volume step for volume_table */
	int num_channels;
	u32 readback_offset; /* offset to mmaped data if used */
	struct sof_ipc_ctrl_data *control_data;
	u32 size;	/* cdata size */
	enum sof_ipc_ctrl_cmd cmd;
	u32 *volume_table; /* volume table computed from tlv data*/

	struct list_head list;	/* list in sof_audio_dev control list */

	struct snd_sof_led_control led_ctl;
};

/* ASoC SOF DAPM widget */
struct snd_sof_widget {
	struct snd_soc_component *scomp;
	int comp_id;
	int pipeline_id;
	int complete;
	int id;

	struct snd_soc_dapm_widget *widget;
	struct list_head list;	/* list in sof_audio_dev widget list */

	void *private;		/* core does not touch this */
};

/* SOF audio device */
struct sof_audio_dev {
	/*
	 * ASoC components. plat_drv fields are set dynamically so
	 * can't use const
	 */
	struct snd_soc_component_driver plat_drv;

	/* topology */
	struct list_head pcm_list;
	struct list_head kcontrol_list;
	struct list_head widget_list;
	struct list_head dai_list;
	struct list_head route_list;
	struct snd_soc_component *component;

	void *private;
};

/*
 * Mixer IPC
 */
int snd_sof_ipc_set_get_comp_data(struct snd_sof_control *scontrol,
				  u32 ipc_cmd,
				  enum sof_ipc_ctrl_type ctrl_type,
				  enum sof_ipc_ctrl_cmd ctrl_cmd,
				  bool send);

struct snd_sof_widget *snd_sof_find_swidget(struct snd_soc_component *scomp,
					    const char *name);
struct snd_sof_widget *
snd_sof_find_swidget_sname(struct snd_soc_component *scomp,
			   const char *pcm_name, int dir);

struct snd_sof_dai *snd_sof_find_dai(struct snd_soc_component *scomp,
				     const char *name);

#endif
