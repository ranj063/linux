/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright(c) 2022 Intel Corporation. All rights reserved.
 */

#ifndef __SOUND_SOC_SOF_IPC4_PRIV_H
#define __SOUND_SOC_SOF_IPC4_PRIV_H

#include <sound/sof/ext_manifest4.h>

#define SOF_IPC4_FW_PAGE_SIZE BIT(12)
#define SOF_IPC4_FW_PAGE(x) ((((x) + BIT(12) - 1) & ~(BIT(12) - 1)) >> 12)
#define SOF_IPC4_FW_ROUNDUP(x) (((x) + BIT(6) - 1) & (~(BIT(6) - 1)))

#define SOF_IPC4_MODULE_LL      BIT(5)
#define SOF_IPC4_MODULE_INSTANCE_LIST_ITEM_SIZE 12
#define SOF_IPC4_PIPELINE_OBJECT_SIZE 448
#define SOF_IPC4_DATA_QUEUE_OBJECT_SIZE 128
#define SOF_IPC4_LL_TASK_OBJECT_SIZE 72
#define SOF_IPC4_DP_TASK_OBJECT_SIZE 104
#define SOF_IPC4_DP_TASK_LIST_SIZE (12 + 8)
#define SOF_IPC4_LL_TASK_LIST_ITEM_SIZE 12
#define SOF_IPC4_FW_MAX_PAGE_COUNT 20
#define SOF_IPC4_FW_MAX_QUEUE_COUNT 8

#include <sound/sof/ext_manifest4.h>

struct sof_ipc4_fw_module {
	struct sof_man4_module man4_module_entry;
	struct ida m_ida;
	u32 bss_size;
	void *private;
};

/**
 * struct sof_gtw_attributes: Gateway attributes
 * @lp_buffer_alloc: Gateway data requested in low power memory
 * @alloc_from_reg_file: Gateway data requested in register file memory
 * @rsvd: reserved for future use
 */
struct sof_ipc4_gtw_attributes {
	uint32_t lp_buffer_alloc : 1;
	uint32_t alloc_from_reg_file : 1;
	uint32_t rsvd : 30;
};

extern const struct sof_ipc_fw_loader_ops ipc4_loader_ops;
extern const struct sof_ipc_tplg_ops ipc4_tplg_ops;
extern const struct sof_ipc_pcm_ops ipc4_pcm_ops;

#endif
