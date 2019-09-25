/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Copyright(c) 2019 Intel Corporation. All rights reserved.
 *
 * Author: Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
 */
#include <linux/platform_device.h>
#include "sof-priv.h"

#ifndef __SOUND_SOC_SOF_MFD_H
#define __SOUND_SOC_SOF_MFD_H

/* client register/unregister */
struct sof_mfd_client *sof_client_dev_register(struct snd_sof_dev *sdev,
					       const char *name);
void sof_client_dev_unregister(struct sof_mfd_client *client);

/* IPC RX/TX */
int sof_client_tx_message(struct device *dev, u32 header,
			  void *msg_data, size_t msg_bytes, void *reply_data,
			  size_t reply_bytes);

void *sof_mfd_get_client_data(struct device *dev);

#endif
