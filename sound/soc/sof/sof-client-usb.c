// SPDX-License-Identifier: GPL-2.0-only
/*
 * SOF USB Audio Offload - Auxiliary Client Driver
 * Hooks into snd-usb-audio for discovery and enumeration
 */

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <sound/soc-usb.h>
#include <sound/soc.h>
#include "../usb/usbaudio.h"
#include "../usb/card.h"
#include "sof-priv.h"
#include "sof-client.h"

struct sof_usb_priv {
    struct auxiliary_device *auxdev;
    struct snd_sof_dev *sdev;
    struct snd_usb_platform_ops ops;
    struct snd_soc_usb *usb_port;
    
    /* Track if machine client is registered */
    bool mach_registered;
    struct mutex lock;
};

/* Helper to enumerate PCM devices from a USB audio chip */
static void sof_usb_enumerate_pcm(struct snd_usb_audio *chip)
{
    struct snd_usb_stream *stream;
    struct device *dev = chip->dev;
    
    dev_info(dev,
         "SOF USB: Enumerating USB audio chip %d\n",
         chip->index);
    
    /* Walk through all PCM streams */
    list_for_each_entry(stream, &chip->pcm_list, list) {
        struct snd_usb_substream *subs;
        int direction;
        
        dev_info(dev,
             "  PCM device %d: %s\n",
             stream->pcm->device,
             stream->pcm->name);
        
        /* Check both playback and capture */
        for (direction = 0; direction < 2; direction++) {
            subs = &stream->substream[direction];
            
            if (!subs->num_formats)
                continue;
            
            dev_info(dev,
                 "    Direction: %s\n",
                 direction == SNDRV_PCM_STREAM_PLAYBACK ? 
                 "Playback" : "Capture");
            dev_info(dev,
                 "      Formats: %d\n", subs->num_formats);
            
            if (subs->data_endpoint) {
                dev_info(dev,
                     "      Data EP: 0x%02x (max packet: %d)\n",
                     subs->data_endpoint->ep_num,
                     subs->data_endpoint->maxpacksize);
            }
            
            if (subs->sync_endpoint) {
                dev_info(dev,
                     "      Sync EP: 0x%02x\n",
                     subs->sync_endpoint->ep_num);
            }
        }
    }
}

/* Register machine driver client device */
static int sof_usb_register_machine(struct sof_usb_priv *priv,
                    struct snd_usb_audio *chip)
{
    int ret;
    
    /* Store chip pointer for machine driver to access */
    /* We'll use auxiliary device's platform data for this */
    dev_set_drvdata(&priv->auxdev->dev, chip);
    
    ret = sof_client_dev_register(priv->sdev, "usb-mach", 0, NULL, 0);
    if (ret < 0) {
        dev_err(&priv->auxdev->dev,
            "Failed to register USB machine client: %d\n", ret);
        return ret;
    }
    
    priv->mach_registered = true;
    
    dev_info(&priv->auxdev->dev,
         "Registered USB offload machine client device\n");
    
    return 0;
}

/* Unregister machine driver client device */
static void sof_usb_unregister_machine(struct sof_usb_priv *priv)
{
    if (!priv->mach_registered)
        return;
    
    dev_info(&priv->auxdev->dev,
         "Unregistering USB offload machine client device\n");
    
    sof_client_dev_unregister(priv->sdev, "usb-mach", 0);
    priv->mach_registered = false;
}

/* Platform ops callback: USB audio device connected */
static int sof_usb_connect_cb(struct snd_usb_audio *chip)
{
    struct sof_usb_priv *priv;
    struct device *dev = chip->dev;
    int ret;
    
    /* Get priv from platform ops */
    priv = container_of(chip->platform_ops, struct sof_usb_priv, ops);
    
    dev_info(dev,
         "========================================\n");
    dev_info(dev,
         "SOF USB: Device connected\n");
    dev_info(dev,
         "========================================\n");
    
    sof_usb_enumerate_pcm(chip);
    
    mutex_lock(&priv->lock);
    
    /* Register machine driver for this USB device */
    ret = sof_usb_register_machine(priv, chip);
    if (ret < 0)
        dev_err(dev, "Failed to register machine driver: %d\n", ret);
    
    mutex_unlock(&priv->lock);
    
    dev_info(dev,
         "========================================\n");
    
    return ret;
}

/* Platform ops callback: USB audio device disconnected */
static void sof_usb_disconnect_cb(struct snd_usb_audio *chip)
{
    struct sof_usb_priv *priv;
    
    priv = container_of(chip->platform_ops, struct sof_usb_priv, ops);
    
    dev_info(chip->dev,
         "SOF USB: Device disconnected (chip index %d)\n",
         chip->index);
    
    mutex_lock(&priv->lock);
    
    /* Unregister machine driver */
    sof_usb_unregister_machine(priv);
    
    mutex_unlock(&priv->lock);
}

/* Platform ops callback: USB audio device suspended */
static void sof_usb_suspend_cb(struct snd_usb_audio *chip,
                   pm_message_t state)
{
    dev_dbg(chip->dev,
        "SOF USB: Device suspended (chip index %d)\n",
        chip->index);
}

/* Auxiliary driver probe */
static int sof_usb_offload_probe(struct auxiliary_device *auxdev,
                 const struct auxiliary_device_id *id)
{
    struct sof_usb_priv *priv;
    struct snd_sof_dev *sdev;
    struct snd_soc_usb *usb_port;
    int ret;
    
    dev_info(&auxdev->dev, "SOF USB offload probing...\n");
    
    /* Get parent SOF device */
    sdev = dev_get_drvdata(auxdev->dev.parent);
    if (!sdev) {
        dev_err(&auxdev->dev, "No SOF device found\n");
        return -ENODEV;
    }
    
    priv = devm_kzalloc(&auxdev->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;
    
    priv->auxdev = auxdev;
    priv->sdev = sdev;
    mutex_init(&priv->lock);
    dev_set_drvdata(&auxdev->dev, priv);
    
    /* Set up platform ops for snd-usb-audio */
    priv->ops.connect_cb = sof_usb_connect_cb;
    priv->ops.disconnect_cb = sof_usb_disconnect_cb;
    priv->ops.suspend_cb = sof_usb_suspend_cb;
    
    /* Register with snd-usb-audio */
    ret = snd_usb_register_platform_ops(&priv->ops);
    if (ret < 0) {
        dev_err(&auxdev->dev,
            "Failed to register platform ops: %d\n", ret);
        return ret;
    }
    
    /* Register as a snd-soc-usb backend/port */
    usb_port = snd_soc_usb_allocate_port(NULL, priv);
    if (!usb_port) {
        dev_err(&auxdev->dev, "Failed to allocate snd-soc-usb port\n");
        ret = -ENOMEM;
        goto err_unreg_ops;
    }
    
    ret = snd_soc_usb_add_port(usb_port);
    if (ret < 0) {
        dev_err(&auxdev->dev, "Failed to add snd-soc-usb port: %d\n", ret);
        snd_soc_usb_free_port(usb_port);
        goto err_unreg_ops;
    }
    
    priv->usb_port = usb_port;
    
    /* Trigger re-discovery of already-connected USB audio devices */
    snd_usb_rediscover_devices();
    
    dev_info(&auxdev->dev, "SOF USB offload initialized\n");
    
    return 0;

err_unreg_ops:
    snd_usb_unregister_platform_ops(&priv->ops);
    return ret;
}

/* Auxiliary driver remove */
static void sof_usb_offload_remove(struct auxiliary_device *auxdev)
{
    struct sof_usb_priv *priv = dev_get_drvdata(&auxdev->dev);
    
    dev_info(&auxdev->dev, "SOF USB offload removing...\n");
    
    mutex_lock(&priv->lock);
    
    /* Unregister any active machine driver */
    sof_usb_unregister_machine(priv);
    
    mutex_unlock(&priv->lock);
    
    /* Remove snd-soc-usb port */
    if (priv->usb_port) {
        snd_soc_usb_remove_port(priv->usb_port);
        snd_soc_usb_free_port(priv->usb_port);
    }
    
    /* Unregister platform ops */
    snd_usb_unregister_platform_ops(&priv->ops);
    
    dev_info(&auxdev->dev, "SOF USB offload removed\n");
}

static const struct auxiliary_device_id sof_usb_offload_table[] = {
    { .name = "snd_sof.usb-offload" },
    {},
};
MODULE_DEVICE_TABLE(auxiliary, sof_usb_offload_table);

static struct auxiliary_driver sof_usb_offload_driver = {
    .name = "usb-offload",
    .probe = sof_usb_offload_probe,
    .remove = sof_usb_offload_remove,
    .id_table = sof_usb_offload_table,
};

module_auxiliary_driver(sof_usb_offload_driver);

MODULE_DESCRIPTION("SOF USB Audio Offload Client");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("SND_SOC_SOF_CLIENT");
MODULE_AUTHOR("Intel Corporation");