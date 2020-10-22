/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright(c) 2020 Intel Corporation. All rights reserved.
 *
 * Author: Author: Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
 */

/* DSP Registers */
#define MTL_HfDSSCS			0x1000
#define MTL_HfDSSCS_SPA_MASK		BIT(16)
#define MTL_HfDSSCS_CPA_MASK		BIT(24)
#define MTL_HfPWRCTL			0x1D18
#define MTL_HfPWRCTL_WPDSPHPxPG	BIT(0)
#define MTL_HfPWRSTS			0x1D1C
#define MTL_HfPWRSTS_DSPHPxPGS_MASK	BIT(0)
#define MTL_HfINTIPPTR			0x1108
#define MTL_IRQ_INTEN_L_HOST_IPC_MASK	BIT(0)
#define MTL_HfINTIPPTR_PTR_MASK	GENMASK(20, 0)

#define MTL_DSP2CxCAP_PRIMARY_CORE	0x178D00
#define MTL_DSP2CxCTL_PRIMARY_CORE	0x178D04
#define MTL_DSP2CxCTL_PRIMARY_CORE_SPA_MASK BIT(0)
#define MTL_DSP2CxCTL_PRIMARY_CORE_CPA_MASK BIT(8)
#define MTL_DSP2CxCTL_PRIMARY_CORE_OSEL GENMASK(25, 24)
#define MTL_DSP2CxCTL_PRIMARY_CORE_OSEL_SHIFT 24

/* IPC Registers */
#define MTL_DSP_REG_HfIPCxTDR		0x73200
#define MTL_DSP_REG_HfIPCxTDR_BUSY	BIT(31)
#define MTL_DSP_REG_HfIPCxTDR_MSG_MASK GENMASK(30, 0)
#define MTL_DSP_REG_HfIPCxTDA		0x73204
#define MTL_DSP_REG_HfIPCxTDA_BUSY	BIT(31)
#define MTL_DSP_REG_HfIPCxIDR		0x73210
#define MTL_DSP_REG_HfIPCxIDR_BUSY	BIT(31)
#define MTL_DSP_REG_HfIPCxIDR_MSG_MASK GENMASK(30, 0)
#define MTL_DSP_REG_HfIPCxIDA		0x73214
#define MTL_DSP_REG_HfIPCxIDA_DONE	BIT(31)
#define MTL_DSP_REG_HfIPCxIDA_MSG_MASK GENMASK(30, 0)
#define MTL_DSP_REG_HfIPCxCTL		0x73228
#define MTL_DSP_REG_HfIPCxCTL_BUSY	BIT(0)
#define MTL_DSP_REG_HfIPCxCTL_DONE	BIT(1)
#define MTL_DSP_REG_HfIPCxTDDy	0x73300
#define MTL_DSP_REG_HfIPCxIDDy	0x73380
#define MTL_DSP_REG_HfHIPCIE		0x1140
#define MTL_DSP_REG_HfHIPCIE_IE_MASK	BIT(0)

#define MTL_DSP_IRQSTS			0x20
#define MTL_DSP_IRQSTS_IPC		BIT(0)

#define MTL_DSP_PURGE_TIMEOUT_US	20000000 /* 20s */
#define MTL_DSP_REG_POLL_INTERVAL_US	10	/* 10 msec */

/* FW registers */
#define MTL_DSP_ROM_STS		0x180000 /* ROM status */
#define MTL_DSP_ROM_ERROR		0x180004 /* ROM error code */
#define MTL_HfFLGPxQWy			0x163200 /* ROM debug status */
#define MTL_HfFLGPxQWy_ERROR		0x163204 /* ROM debug error code */

/* Memory windows */
#define MTL_SRAM_WINDOW_OFFSET(x)	(0x180000 + 0x8000 * (x))

#define MTL_DSP_MBOX_UPLINK_OFFSET	MTL_SRAM_WINDOW_OFFSET(0) + 0x1000 
#define MTL_DSP_MBOX_UPLINK_SIZE	0x1000
#define MTL_DSP_MBOX_DOWNLINK_OFFSET	MTL_SRAM_WINDOW_OFFSET(1)
#define MTL_DSP_MBOX_DOWNLINK_SIZE	0x1000

#define MTL_DSP_REG_HfIMRIS1		0x162088
#define MTL_DSP_REG_HfIMRIS1_IU_MASK	BIT(0)

int snd_sof_fw_ext_man_parse_cavs(struct snd_sof_dev *sdev,
				  const struct firmware *fw);
