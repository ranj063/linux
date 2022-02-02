/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright(c) 2022 Intel Corporation. All rights reserved.
 */

#ifndef __SOUND_SOC_SOF_IPC3_PRIV_H
#define __SOUND_SOC_SOF_IPC3_PRIV_H

#include <sound/sof/ext_manifest4.h>

struct sof_ipc4_fw_module {
	struct sof_man4_module man4_module_entry;
	struct ida m_ida;
	u32 bss_size;
	void *private;
};

extern const struct sof_ipc_fw_loader_ops ipc4_loader_ops;
extern const struct ipc_tplg_ops ipc4_tplg_ops;
extern const struct ipc_pcm_ops ipc4_pcm_ops;

#endif
