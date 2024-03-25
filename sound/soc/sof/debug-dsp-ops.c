// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright(c) 2024 Intel Corporation. All rights reserved.
//

#include <linux/debugfs.h>
#include <sound/sof/debug.h>
#include "sof-priv.h"
#include "ops.h"

static int sof_dsp_ops_boot_firmware(struct snd_sof_dev *sdev)
{
	const char *fw_filename;
	int ret;

	sdev->first_boot = true;

	if (!sdev->test_profile.fw_path || !sdev->test_profile.fw_name) {
		dev_dbg(sdev->dev, "Invalid firmware path or filename\n");
		return -EINVAL;
	}

	fw_filename = kasprintf(GFP_KERNEL, "%s/%s", sdev->test_profile.fw_path,
				sdev->test_profile.fw_name);

	/* load firmware */
	ret = snd_sof_load_firmware(sdev, fw_filename);
	kfree(fw_filename);
	if (ret < 0)
		return ret;

	/* boot firmware */
	sof_set_fw_state(sdev, SOF_FW_BOOT_IN_PROGRESS);

	return snd_sof_run_firmware(sdev);
}

static ssize_t sof_dsp_ops_tester_dfs_read(struct file *file, char __user *buffer,
					   size_t count, loff_t *ppos)
{
	struct snd_sof_dfsentry *dfse = file->private_data;
	struct snd_sof_dev *sdev = dfse->sdev;
	struct dentry *dentry;
	const char *string;
	size_t size_ret;

	/* return the FW filename or path */
	dentry = file->f_path.dentry;
	if (!strcmp(dentry->d_name.name, "fw_filename"))
		string = sdev->test_profile.fw_name;
	else if (!strcmp(dentry->d_name.name, "fw_path"))
		string = sdev->test_profile.fw_path;
	else
		return 0;

	if (*ppos || !string)
		return 0;

	count = min_t(size_t, count, strlen(string));
	size_ret = copy_to_user(buffer, string, count);
	if (size_ret)
		return -EFAULT;

	*ppos += count;

	return count;
}


static ssize_t sof_dsp_ops_tester_dfs_write(struct file *file, const char __user *buffer,
					    size_t count, loff_t *ppos)
{
	struct snd_sof_dfsentry *dfse = file->private_data;
	struct dentry *dentry = file->f_path.dentry;
	struct snd_sof_dev *sdev = dfse->sdev;
	size_t size;
	char *string;

	if (!strcmp(dentry->d_name.name, "boot_fw"))
		return sof_dsp_ops_boot_firmware(sdev);

	if (strcmp(dentry->d_name.name, "fw_filename") &&
	    strcmp(dentry->d_name.name, "fw_path"))
		return 0;

	string = devm_kzalloc(sdev->dev, count + 1, GFP_KERNEL);
	if (!string)
		return -ENOMEM;

	size = simple_write_to_buffer(string, count, ppos, buffer, count);

	/* truncate the \n at the end */
	string[count - 1] = '\0';

	if (!strcmp(dentry->d_name.name, "fw_filename")) {
		if (sdev->test_profile.fw_name)
			devm_kfree(sdev->test_profile.fw_name);
		sdev->test_profile.fw_name = string;

		return size;
	}

	if (sdev->test_profile.fw_path)
		devm_kfree(sdev->test_profile.fw_path);
	sdev->test_profile.fw_path = string;

	return size;
}

static const struct file_operations sof_dsp_ops_tester_fops = {
	.open = simple_open,
	.write = sof_dsp_ops_tester_dfs_write,
	.read = sof_dsp_ops_tester_dfs_read,
};

static int sof_dsp_dsp_ops_create_dfse(struct snd_sof_dev *sdev, const char *name,
				       struct dentry *parent, umode_t mode)
{
	struct snd_sof_dfsentry *dfse;

	/* create debugfs entry for FW filename */
	dfse = devm_kzalloc(sdev->dev, sizeof(*dfse), GFP_KERNEL);
	if (!dfse)
		return -ENOMEM;

	dfse->type = SOF_DFSENTRY_TYPE_BUF;
	dfse->sdev = sdev;
	debugfs_create_file(name, mode, parent, dfse, &sof_dsp_ops_tester_fops);
	list_add(&dfse->list, &sdev->dfsentry_list);

	return 0;
}

int sof_dbg_dsp_ops_test_init(struct snd_sof_dev *sdev)
{
	struct dentry *dsp_ops_debugfs;
	int ret;

	/* debugfs root directory for DSP ops debug */
	dsp_ops_debugfs = debugfs_create_dir("dsp_ops", sdev->debugfs_root);

	/* create debugfs entry for FW filename */
	ret = sof_dsp_dsp_ops_create_dfse(sdev, "fw_filename", dsp_ops_debugfs, 0666);
	if (ret < 0)
		return ret;

	/* create debugfs entry for FW path */
	ret = sof_dsp_dsp_ops_create_dfse(sdev, "fw_path", dsp_ops_debugfs, 0666);
	if (ret < 0)
		return ret;

	return sof_dsp_dsp_ops_create_dfse(sdev, "boot_fw", dsp_ops_debugfs, 0222);
}
