// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// Copyright(c) 2020 Intel Corporation. All rights reserved.
//
// Authors: Ranjani Sridharan <ranjani.sridharan@linux.intel.com>
//

/*
 * Hardware interface for audio DSP on Meteorlake.
 */

#include "../ops.h"
#include "hda.h"
#include "hda-ipc.h"
#include "../sof-audio.h"
#include "mtl.h"

static const struct snd_sof_debugfs_map mtl_dsp_debugfs[] = {
	{"hda", HDA_DSP_HDA_BAR, 0, 0x4000, SOF_DEBUGFS_ACCESS_ALWAYS},
	{"pp", HDA_DSP_PP_BAR,  0, 0x1000, SOF_DEBUGFS_ACCESS_ALWAYS},
	{"dsp", HDA_DSP_BAR,  0, 0x10000, SOF_DEBUGFS_ACCESS_ALWAYS},
};

static void mtl_ipc_host_done(struct snd_sof_dev *sdev);
static void mtl_ipc_dsp_done(struct snd_sof_dev *sdev);

/* Check if an IPC IRQ occurred */
bool mtl_dsp_check_ipc_irq(struct snd_sof_dev *sdev)
{
	bool ret = false;
	u32 irq_status;
	u32 hfintipptr;

	/* read Interrupt IP Pointer */
	hfintipptr = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_HfINTIPPTR) & MTL_HfINTIPPTR_PTR_MASK;
	irq_status = snd_sof_dsp_read(sdev, HDA_DSP_BAR, hfintipptr + MTL_DSP_IRQSTS);

	dev_vdbg(sdev->dev, "irq handler: irq_status:0x%x\n", irq_status);

	/* invalid message ? */
	if (irq_status == 0xffffffff)
		goto out;

	/* IPC message ? */
	if (irq_status & MTL_DSP_IRQSTS_IPC)
		ret = true;

out:
	return ret;
}

int mtl_ipc_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	struct sof_intel_hda_dev *hdev = sdev->pdata->hw_pdata;
	struct sof_ipc_cmd_hdr *hdr;
	u32 dr = 0;
	u32 dd = 0;

/* TODO: Fix compact IPC later */
#if 0
	/*
	 * Currently the only compact IPC supported is the PM_GATE
	 * IPC which is used for transitioning the DSP between the
	 * D0I0 and D0I3 states. And these are sent only during the
	 * set_power_state() op. Therefore, there will never be a case
	 * that a compact IPC results in the DSP exiting D0I3 without
	 * the host and FW being in sync.
	 */
	if (cnl_compact_ipc_compress(msg, &dr, &dd)) {
		/* send the message via IPC registers */
		snd_sof_dsp_write(sdev, HDA_DSP_BAR, CNL_DSP_REG_HIPCIDD,
				  dd);
		snd_sof_dsp_write(sdev, HDA_DSP_BAR, CNL_DSP_REG_HIPCIDR,
				  CNL_DSP_REG_HIPCIDR_BUSY | dr);
		return 0;
	}
#endif

	/* send the message via mailbox */
	sof_mailbox_write(sdev, sdev->host_box.offset, msg->msg_data, msg->msg_size);
	snd_sof_dsp_write(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxIDR, MTL_DSP_REG_HfIPCxIDR_BUSY);

/* TODO: Fix D0i3 in S0 later */
#if 0
	hdr = msg->msg_data;

	/*
	 * Use mod_delayed_work() to schedule the delayed work
	 * to avoid scheduling multiple workqueue items when
	 * IPCs are sent at a high-rate. mod_delayed_work()
	 * modifies the timer if the work is pending.
	 * Also, a new delayed work should not be queued after the
	 * CTX_SAVE IPC, which is sent before the DSP enters D3.
	 */
	if (hdr->cmd != (SOF_IPC_GLB_PM_MSG | SOF_IPC_PM_CTX_SAVE))
		mod_delayed_work(system_wq, &hdev->d0i3_work,
				 msecs_to_jiffies(SOF_HDA_D0I3_WORK_DELAY_MS));
#endif
	return 0;
}

static void mtl_enable_ipc_interrupts(struct snd_sof_dev *sdev)
{
	struct sof_intel_hda_dev *hda = sdev->pdata->hw_pdata;
	const struct sof_intel_dsp_desc *chip = hda->desc;

	/* enable IPC DONE and BUSY interrupts */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, chip->ipc_ctl,
			MTL_DSP_REG_HfIPCxCTL_BUSY | MTL_DSP_REG_HfIPCxCTL_DONE,
			MTL_DSP_REG_HfIPCxCTL_BUSY | MTL_DSP_REG_HfIPCxCTL_DONE);
}

static int mtl_enable_interrupts(struct snd_sof_dev *sdev)
{
	u32 hfintipptr, irqinten, host_ipc;
	u32 hipcie;
	int ret;

	/* read Interrupt IP Pointer */
	hfintipptr = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_HfINTIPPTR) & MTL_HfINTIPPTR_PTR_MASK;

	/* Enable Host IPC */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, hfintipptr,
				MTL_IRQ_INTEN_L_HOST_IPC_MASK, MTL_IRQ_INTEN_L_HOST_IPC_MASK);

	/* check if operation was successful */
	host_ipc = MTL_IRQ_INTEN_L_HOST_IPC_MASK;
	irqinten = snd_sof_dsp_read(sdev, HDA_DSP_BAR, hfintipptr);
	ret = snd_sof_dsp_read_poll_timeout(sdev, HDA_DSP_BAR, hfintipptr, irqinten,
					    (irqinten & host_ipc) == host_ipc,
					    HDA_DSP_REG_POLL_INTERVAL_US, HDA_DSP_RESET_TIMEOUT_US);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to enable Host IPC\n");
		return ret;
	}

	/* Set Host IPC interrupt enable */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfHIPCIE,
				MTL_DSP_REG_HfHIPCIE_IE_MASK, MTL_DSP_REG_HfHIPCIE_IE_MASK);

	/* check if operation was successful */
	host_ipc = MTL_DSP_REG_HfHIPCIE_IE_MASK;
	hipcie = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfHIPCIE);
	ret = snd_sof_dsp_read_poll_timeout(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfHIPCIE, hipcie,
					    (hipcie & host_ipc) == host_ipc,
					    HDA_DSP_REG_POLL_INTERVAL_US, HDA_DSP_RESET_TIMEOUT_US);
	if (ret < 0)
		dev_err(sdev->dev, "error: failed to set Host IPC interrupt enable\n");

	return ret;
}

/* pre fw run operations */
static int mtl_dsp_pre_fw_run(struct snd_sof_dev *sdev)
{
	u32 dsphfdsscs, dsphfpwrsts;
	u32 cpa, pgs;
	int ret;

	/* Set the DSP subsystem power on */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, MTL_HfDSSCS,
				MTL_HfDSSCS_SPA_MASK, MTL_HfDSSCS_SPA_MASK);

	/* Wait for unstable CPA read (1 then 0 then 1) just after setting SPA bit */
	usleep_range(1000, 1010);

	/* poll with timeout to check if operation successful */
	cpa = MTL_HfDSSCS_CPA_MASK;
	dsphfdsscs = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_HfDSSCS);
	ret = snd_sof_dsp_read_poll_timeout(sdev, HDA_DSP_BAR, MTL_HfDSSCS, dsphfdsscs,
					    (dsphfdsscs & cpa) == cpa, HDA_DSP_REG_POLL_INTERVAL_US,
					    HDA_DSP_RESET_TIMEOUT_US);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to enable DSP subsystem\n");
		return ret;
	}

	/* Power up gated-DSP-0 domain in order to access the DSP shim register block. */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, MTL_HfPWRCTL,
				MTL_HfPWRCTL_WPDSPHPxPG, MTL_HfPWRCTL_WPDSPHPxPG);

	usleep_range(1000, 1010);

	/* poll with timeout to check if operation successful */
	pgs = MTL_HfPWRSTS_DSPHPxPGS_MASK;
	dsphfpwrsts = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_HfPWRSTS);
	ret = snd_sof_dsp_read_poll_timeout(sdev, HDA_DSP_BAR, MTL_HfPWRSTS, dsphfpwrsts,
					    (dsphfpwrsts & pgs) == pgs,
					    HDA_DSP_REG_POLL_INTERVAL_US,
					    HDA_DSP_RESET_TIMEOUT_US);
	if (ret < 0)
		dev_err(sdev->dev, "error: failed to power up gated DSP domain\n");

	return ret;
}

static int mtl_dsp_post_fw_run(struct snd_sof_dev *sdev)
{
	/* TODO */
	return 0;
}

static void mtl_dsp_dump(struct snd_sof_dev *sdev, u32 flags)
{
	u32 fwsts, fwlec, romdbgsts, romdbgerr;

	fwsts = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_DSP_ROM_STS);
	fwlec = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_DSP_ROM_ERROR);
	romdbgsts = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_HfFLGPxQWy);
	romdbgerr = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_HfFLGPxQWy_ERROR);

	dev_err(sdev->dev, "error: ROM status: 0x%x, ROM error: 0x%x\n", fwsts, fwlec);
	dev_err(sdev->dev, "error: ROM debug status: 0x%x, ROM debug error: 0x%x\n", romdbgsts,
		romdbgerr);
}

int mtl_dsp_cl_init(struct snd_sof_dev *sdev, int stream_tag)
{
	struct sof_intel_hda_dev *hda = sdev->pdata->hw_pdata;
	const struct sof_intel_dsp_desc *chip = hda->desc;
	unsigned int status;
	int ret;
	int i;

	/* step 1: purge FW request */
	snd_sof_dsp_write(sdev, HDA_DSP_BAR, chip->ipc_req,
			  chip->ipc_req_mask | (HDA_DSP_IPC_PURGE_FW | ((stream_tag - 1) << 9)));


	/* step 2: power up primary core */
	ret = snd_sof_dsp_core_power_up(sdev, BIT(0));
	if (ret < 0) {
		if (hda->boot_iteration == HDA_FW_BOOT_ATTEMPTS)
			dev_err(sdev->dev, "error: dsp core 0/1 power up failed\n");
		goto err;
	}

	dev_dbg(sdev->dev, "Primary core power up successful\n");

	/* TODO: set SSP slave mode */

	/* step 4: wait for IPC DONE bit from ROM */
	ret = snd_sof_dsp_read_poll_timeout(sdev, HDA_DSP_BAR, chip->ipc_ack, status,
					    ((status & chip->ipc_ack_mask) == chip->ipc_ack_mask),
					    HDA_DSP_REG_POLL_INTERVAL_US, MTL_DSP_PURGE_TIMEOUT_US);
	if (ret < 0) {
		if (hda->boot_iteration == HDA_FW_BOOT_ATTEMPTS)
			dev_err(sdev->dev, "error: timeout waiting for purge IPC done\n");
		goto err;
	}

	/* set DONE bit to clear the reply IPC message */
	snd_sof_dsp_update_bits_forced(sdev, HDA_DSP_BAR, chip->ipc_ack, chip->ipc_ack_mask,
				       chip->ipc_ack_mask);

	/* step 5: enable interrupts */
	ret = mtl_enable_interrupts(sdev);
	if (ret < 0) {
		if (hda->boot_iteration == HDA_FW_BOOT_ATTEMPTS)
			dev_err(sdev->dev, "error: %s: failed to enable interrupts\n", __func__);
		return ret;
	}

	mtl_enable_ipc_interrupts(sdev);

	/* step 6: wait for ROM_INIT_DONE */
	/*
	 * ACE workaround as in Windows driver.
	 * "ACE platform cannot catch FSR_ROM_INIT_DONE, because this status is setting by short
	 * time. This "for" loop is redundant for us and we get timeout almost everytime by this
	 * condition."
	 * So dont wait for ROM INIT.
	 */

	return 0;

err:
	mtl_dsp_dump(sdev, 0);
	snd_sof_dsp_core_power_down(sdev, BIT(0));
	return ret;
}

irqreturn_t mtl_ipc_irq_thread(int irq, void *context)
{
	struct snd_sof_dev *sdev = context;
	u32 hipci;
	u32 hipcida;
	u32 hipctdr;
	u32 hipctdd;
	u32 hipcidd;
	u32 msg;
	u32 msg_ext;
	bool ipc_irq = false;

	hipcida = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxIDA);
	hipctdr = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxTDR);
	hipctdd = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxTDDy);
	hipci = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxIDR);

	/* reply message from DSP */
	if (hipcida & MTL_DSP_REG_HfIPCxIDA_DONE) {
		msg = hipcida & MTL_DSP_REG_HfIPCxIDA_MSG_MASK;

		dev_vdbg(sdev->dev, "ipc: firmware response, msg:0x%x\n", msg);

		/* mask Done interrupt */
		snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxCTL,
					MTL_DSP_REG_HfIPCxCTL_DONE, 0);

		spin_lock_irq(&sdev->ipc_lock);

		/* handle immediate reply from DSP core */
		hda_dsp_ipc_get_reply(sdev);
		snd_sof_ipc_reply(sdev, msg);

		mtl_ipc_dsp_done(sdev);

		spin_unlock_irq(&sdev->ipc_lock);

		ipc_irq = true;
	}

	/* new message from DSP */
	if (hipctdr & MTL_DSP_REG_HfIPCxTDR_BUSY) {
		msg = hipctdr & MTL_DSP_REG_HfIPCxTDR_MSG_MASK;
		msg_ext = hipctdd;

		dev_dbg(sdev->dev, "ipc: firmware initiated, msg:0x%x, msg_ext:0x%x\n",
			msg, msg_ext);

		/* handle messages from DSP */
//		if ((hipctdr & SOF_IPC_PANIC_MAGIC_MASK) == SOF_IPC_PANIC_MAGIC)
//			snd_sof_dsp_panic(sdev, HDA_DSP_PANIC_OFFSET(msg_ext));
//		else
		snd_sof_ipc2_msgs_rx(sdev, msg, msg_ext);

		mtl_ipc_host_done(sdev);

		ipc_irq = true;
	}

	if (!ipc_irq) {
		/* This interrupt is not shared so no need to return IRQ_NONE. */
		dev_dbg_ratelimited(sdev->dev, "nothing to do in IPC IRQ thread\n");
	}

	return IRQ_HANDLED;
}

static void mtl_ipc_host_done(struct snd_sof_dev *sdev)
{
	/*
	 * clear busy interrupt to tell dsp controller this interrupt has been accepted,
	 * not trigger it again
	 */
	snd_sof_dsp_update_bits_forced(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxTDR,
				       MTL_DSP_REG_HfIPCxTDR_BUSY, MTL_DSP_REG_HfIPCxTDR_BUSY);
	/*
	 * set busy bit to ack dsp the msg has been processed and send reply msg to dsp
	 */
	snd_sof_dsp_update_bits_forced(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxTDA,
				       MTL_DSP_REG_HfIPCxTDA_BUSY, MTL_DSP_REG_HfIPCxTDA_BUSY);
}

static void mtl_ipc_dsp_done(struct snd_sof_dev *sdev)
{
	/*
	 * set DONE bit - tell DSP we have received the reply msg from DSP, and processed it,
	 * don't send more reply to host
	 */
	snd_sof_dsp_update_bits_forced(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxIDA,
				       MTL_DSP_REG_HfIPCxIDA_DONE, MTL_DSP_REG_HfIPCxIDA_DONE);

	/* unmask Done interrupt */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, MTL_DSP_REG_HfIPCxCTL,
				MTL_DSP_REG_HfIPCxCTL_DONE, MTL_DSP_REG_HfIPCxCTL_DONE);
}

int mtl_dsp_core_power_up(struct snd_sof_dev *sdev, unsigned int core_mask)
{
	unsigned int cpa;
	u32 dspcxctl;
	int ret;

	/* Only the primary core can be powered up by the host */
	if (core_mask != BIT(0))
		return 0;

	/* Program the owner of the IP & shim registers (10: Host CPU) */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, MTL_DSP2CxCTL_PRIMARY_CORE,
				MTL_DSP2CxCTL_PRIMARY_CORE_OSEL,
				0x2 << MTL_DSP2CxCTL_PRIMARY_CORE_OSEL_SHIFT);

	/* enable SPA bit */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, MTL_DSP2CxCTL_PRIMARY_CORE,
				MTL_DSP2CxCTL_PRIMARY_CORE_SPA_MASK,
				MTL_DSP2CxCTL_PRIMARY_CORE_SPA_MASK);

	/* Wait for unstable CPA read (1 then 0 then 1) just after setting SPA bit */
	usleep_range(1000, 1010);

	/* poll with timeout to check if operation successful */
	cpa = MTL_DSP2CxCTL_PRIMARY_CORE_CPA_MASK;
	ret = snd_sof_dsp_read_poll_timeout(sdev, HDA_DSP_BAR, MTL_DSP2CxCTL_PRIMARY_CORE, dspcxctl,
					    (dspcxctl & cpa) == cpa, HDA_DSP_REG_POLL_INTERVAL_US,
					    HDA_DSP_RESET_TIMEOUT_US);
	if (ret < 0) {
		dev_err(sdev->dev, "error: %s: timeout on MTL_DSP2CxCTL_PRIMARY_CORE read\n",
			__func__);
		return ret;
	}

	/* did core power up ? */
	dspcxctl = snd_sof_dsp_read(sdev, HDA_DSP_BAR, MTL_DSP2CxCTL_PRIMARY_CORE);
	if ((dspcxctl & MTL_DSP2CxCTL_PRIMARY_CORE_CPA_MASK)
		!= MTL_DSP2CxCTL_PRIMARY_CORE_CPA_MASK) {
		dev_err(sdev->dev, "error: power up core failed core_mask %xadspcs 0x%x\n",
			core_mask, dspcxctl);
		ret = -EIO;
	}

	return ret;
}

static int mtl_dsp_core_power_down(struct snd_sof_dev *sdev, unsigned int core_mask)
{
	u32 dspcxctl;
	int ret;

	/* Only the primary core can be powered down by the host */
	if (core_mask != BIT(0))
		return 0;

	/* disable SPA bit */
	snd_sof_dsp_update_bits(sdev, HDA_DSP_BAR, MTL_DSP2CxCTL_PRIMARY_CORE,
				MTL_DSP2CxCTL_PRIMARY_CORE_SPA_MASK, 0);

	/* Wait for unstable CPA read (1 then 0 then 1) just after setting SPA bit */
	usleep_range(1000, 1010);

	ret = snd_sof_dsp_read_poll_timeout(sdev, HDA_DSP_BAR, MTL_DSP2CxCTL_PRIMARY_CORE, dspcxctl,
					    !(dspcxctl & MTL_DSP2CxCTL_PRIMARY_CORE_CPA_MASK),
					    HDA_DSP_REG_POLL_INTERVAL_US,
					    HDA_DSP_PD_TIMEOUT * USEC_PER_MSEC);
	if (ret < 0)
		dev_err(sdev->dev, "error: failed to power down primary core\n");

	return ret;
}

int mtl_dsp_ipc_get_mailbox_offset(struct snd_sof_dev *sdev)
{
	return MTL_DSP_MBOX_UPLINK_OFFSET;
}

int mtl_dsp_ipc_get_window_offset(struct snd_sof_dev *sdev, u32 id)
{
	return MTL_SRAM_WINDOW_OFFSET(id);
}

int mtl_fw_ready(struct snd_sof_dev *sdev, u32 msg_id)
{
	int inbox_offset, inbox_size;
	int outbox_offset, outbox_size;

	/* mailbox must be on 4k boundary */
	inbox_offset = snd_sof_dsp_get_mailbox_offset(sdev);
	if (inbox_offset < 0) {
		dev_err(sdev->dev, "error: have no mailbox offset\n");
		return inbox_offset;
	}

	dev_dbg(sdev->dev, "ipc: DSP is ready 0x%8.8x offset 0x%x\n",
		msg_id, inbox_offset);

	/* no need to re-check version/ABI for subsequent boots */
	if (!sdev->first_boot)
		return 0;

	inbox_size = MTL_DSP_MBOX_UPLINK_SIZE;
	outbox_offset = MTL_DSP_MBOX_DOWNLINK_OFFSET;
	outbox_size = MTL_DSP_MBOX_DOWNLINK_SIZE;

	snd_sof_dsp_mailbox_init(sdev, inbox_offset, inbox_size,
			 outbox_offset, outbox_size);

	dev_dbg(sdev->dev, " mailbox upstream 0x%x - size 0x%x\n",
		inbox_offset, inbox_size);
	dev_dbg(sdev->dev, " mailbox downstream 0x%x - size 0x%x\n",
		outbox_offset, outbox_size);


	return 0;
}

/* Meteorlake ops */
const struct snd_sof_dsp_ops sof_mtl_ops = {
	/* probe and remove */
	.probe		= hda_dsp_probe,
	.remove		= hda_dsp_remove,

	/* Register IO */
	.write		= sof_io_write,
	.read		= sof_io_read,
	.write64	= sof_io_write64,
	.read64		= sof_io_read64,

	/* Block IO */
	.block_read	= sof_block_read,
	.block_write	= sof_block_write,

	/* doorbell */
	.irq_thread	= mtl_ipc_irq_thread,

	/* ipc */
	.send_msg	= mtl_ipc_send_msg,
	.fw_ready	= mtl_fw_ready,
	.get_mailbox_offset = mtl_dsp_ipc_get_mailbox_offset,
	.get_window_offset = mtl_dsp_ipc_get_window_offset,
	.check_ipc_irq	= mtl_dsp_check_ipc_irq,

	.ipc_msg_data	= hda_ipc_msg_data,
	.ipc_pcm_params	= hda_ipc_pcm_params,

	/* machine driver */
	.machine_select = hda_machine_select,
	.machine_register = sof_machine_register,
	.machine_unregister = sof_machine_unregister,
	.set_mach_params = hda_set_mach_params,

	/* debug */
	.debug_map	= mtl_dsp_debugfs,
	.debug_map_count	= ARRAY_SIZE(mtl_dsp_debugfs),
	.dbg_dump	= mtl_dsp_dump,
	.ipc_dump	= cnl_ipc_dump,

	/* stream callbacks */
	.pcm_open	= hda_dsp_pcm_open,
	.pcm_close	= hda_dsp_pcm_close,
	.pcm_hw_params	= hda_dsp_pcm_hw_params,
	.pcm_hw_free	= hda_dsp_stream_hw_free,
	.pcm_trigger	= hda_dsp_pcm_trigger,
	.pcm_pointer	= hda_dsp_pcm_pointer,

#if IS_ENABLED(CONFIG_SND_SOC_SOF_HDA_PROBES)
	/* probe callbacks */
	.probe_assign	= hda_probe_compr_assign,
	.probe_free	= hda_probe_compr_free,
	.probe_set_params	= hda_probe_compr_set_params,
	.probe_trigger	= hda_probe_compr_trigger,
	.probe_pointer	= hda_probe_compr_pointer,
#endif

	.fw_ext_man_parse = snd_sof_fw_ext_man_parse_cavs,

	/* firmware loading */
	.load_firmware = snd_sof_load_firmware_raw,

	/* pre/post fw run */
	.pre_fw_run = mtl_dsp_pre_fw_run,
	.post_fw_run = mtl_dsp_post_fw_run,

	/* dsp core power up/down */
	.core_power_up = mtl_dsp_core_power_up,
	.core_power_down = mtl_dsp_core_power_down,

	/* firmware run */
	.cl_dsp_init = mtl_dsp_cl_init,
	.run = hda_dsp_cl_boot_firmware,

	/* trace callback */
	.trace_init = hda_dsp_trace_init,
	.trace_release = hda_dsp_trace_release,
	.trace_trigger = hda_dsp_trace_trigger,

	/* DAI drivers */
	.drv		= skl_dai,
	.num_drv	= SOF_SKL_NUM_DAIS,

	/* PM */
	.suspend		= hda_dsp_suspend,
	.resume			= hda_dsp_resume,
	.runtime_suspend	= hda_dsp_runtime_suspend,
	.runtime_resume		= hda_dsp_runtime_resume,
	.runtime_idle		= hda_dsp_runtime_idle,
	.set_hw_params_upon_resume = hda_dsp_set_hw_params_upon_resume,
	.set_power_state	= hda_dsp_set_power_state,

	/* ALSA HW info flags */
	.hw_info =	SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,

	.arch_ops = &sof_xtensa_arch_ops,
};
EXPORT_SYMBOL_NS(sof_mtl_ops, SND_SOC_SOF_INTEL_HDA_COMMON);

const struct sof_intel_dsp_desc mtl_chip_info = {
	.cores_num = 3,
	.init_core_mask = 1,
	.host_managed_cores_mask = BIT(0),
	.ipc_req = MTL_DSP_REG_HfIPCxIDR,
	.ipc_req_mask = MTL_DSP_REG_HfIPCxIDR_BUSY,
	.ipc_ack = MTL_DSP_REG_HfIPCxIDA,
	.ipc_ack_mask = MTL_DSP_REG_HfIPCxIDA_DONE,
	.ipc_ctl = MTL_DSP_REG_HfIPCxCTL,
	.rom_status_reg = MTL_DSP_ROM_STS,
	.rom_init_timeout	= 300,
	.ssp_count = ICL_SSP_COUNT,
	.ssp_base_offset = CNL_SSP_BASE_OFFSET,
};
EXPORT_SYMBOL_NS(mtl_chip_info, SND_SOC_SOF_INTEL_HDA_COMMON);
