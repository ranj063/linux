// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// Copyright(c) 2020 Intel Corporation. All rights reserved.
//
// Authors: Rander Wang <rander.wang@linux.intel.com>
//

/*
 * FW loader for Meteorlake.
 */

#include <linux/firmware.h>
#include <sound/sof.h>
#include <sound/sof/cavs_ext_manifest.h>
#include "../sof-audio.h"
#include "hda.h"

/*********************************************************************
 *     css_manifest hdr
 *-------------------
 *     offset reserved for future
 *-------------------
 *     fw_hdr
 *-------------------
 *     module_entry[0]
 *-------------------
 *     module_entry[1]
 *-------------------
 *     ...
 *-------------------
 *     module_entry[n]
 *-------------------
 *     FW content
 *-------------------
 *********************************************************************/
int snd_sof_fw_ext_man_parse_cavs(struct snd_sof_dev *sdev,
                                     const struct firmware *fw)
{
	struct sof_intel_hda_dev *hdev = sdev->pdata->hw_pdata;
	struct CavsFwBinaryHeader *fw_header;
	struct cavs_ext_manifest_hdr *hdr;
	struct ModuleEntry *module_entry;
	struct ModuleEntry *fm_entry;
	int fw_offset;
	int i;

	if (fw->size < sizeof(hdr)) {
		dev_err(sdev->dev, "Invalid fw size\n");
		return -EINVAL;
	}

	hdr = (struct cavs_ext_manifest_hdr *)fw->data;

	if (hdr->id == CAVS_EXT_MAN_MAGIC_NUMBER) {
		fw_offset = hdr->len;
	} else {
		dev_err(sdev->dev, "invalid cavs FW");
		return -EINVAL;
	}

	fw_header = (struct CavsFwBinaryHeader *)(fw->data + fw_offset + CAVS18_FW_HDR_OFFSET);
	dev_dbg(sdev->dev, " fw %s: header length %x, module num %d", fw_header->name,
	fw_header->len, fw_header->num_module_entries);

	fm_entry = (struct ModuleEntry *)((void *)fw_header + fw_header->len);
	module_entry = devm_kmalloc_array(sdev->dev, fw_header->num_module_entries,
	                         sizeof(*module_entry), GFP_KERNEL);
	if (!module_entry)
		return -ENOMEM;


	hdev->fw_module_num = fw_header->num_module_entries;
	hdev->cavs_fw_module_entry = module_entry;

	for (i = 0; i < fw_header->num_module_entries; i++) {
		dev_dbg(sdev->dev, "module %s : UUID %pUL, ", fm_entry->name,
		       fm_entry->uuid);
		memcpy(module_entry, fm_entry++, sizeof(*fm_entry));

		/* bringup fw starts at zero */
		module_entry->id = i;
		module_entry++;
	}

	return fw_offset;
}

