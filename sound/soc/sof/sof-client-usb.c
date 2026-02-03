// SPDX-License-Identifier: GPL-2.0-only
/*
 * SOF USB Audio Offload - Auxiliary Client Driver
 * Hooks into snd-usb-audio for discovery and enumeration only
 */

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <sound/soc-usb.h>
#include <sound/soc.h>
#include "../usb/usbaudio.h"
#include "../usb/card.h"
#include "sof-priv.h"

struct sof_usb_priv {
    struct auxiliary_device *auxdev;
    struct snd_sof_dev *sdev;
    struct snd_usb_platform_ops ops;
    struct snd_soc_usb *usb_port; /* Added for snd-soc-usb port */
};

/* Helper to enumerate PCM devices from a USB audio chip */
static void sof_usb_enumerate_pcm(struct snd_usb_audio *chip)
{
    struct snd_usb_stream *stream;
    struct device *dev = chip->dev; /* Use USB device, not card */
    
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

/* Platform ops callback: USB audio device connected */
static int sof_usb_connect_cb(struct snd_usb_audio *chip)
{
    struct device *dev = chip->dev; /* Use USB device */
    
    dev_info(dev,
         "========================================\n");
    dev_info(dev,
         "SOF USB: Device connected\n");
    dev_info(dev,
         "========================================\n");
    
    sof_usb_enumerate_pcm(chip);
    
    dev_info(dev,
         "========================================\n");
    
    return 0;
}

/* Platform ops callback: USB audio device disconnected */
static void sof_usb_disconnect_cb(struct snd_usb_audio *chip)
{
    dev_info(chip->dev, /* Use chip->dev */
         "SOF USB: Device disconnected (chip index %d)\n",
         chip->index);
}

/* Platform ops callback: USB audio device suspended */
static void sof_usb_suspend_cb(struct snd_usb_audio *chip,
                   pm_message_t state)
{
    dev_dbg(chip->dev, /* Use chip->dev */
        "SOF USB: Device suspended (chip index %d)\n",
        chip->index);
}

/* Auxiliary driver probe */
static int sof_usb_offload_probe(struct auxiliary_device *auxdev,
                 const struct auxiliary_device_id *id)
{
    struct sof_usb_priv *priv;
    struct snd_sof_dev *sdev;
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
    struct snd_soc_usb *usb_port;

    usb_port = snd_soc_usb_allocate_port(NULL, priv); /* Pass your priv as priv_data */
    if (!usb_port) {
        dev_err(&auxdev->dev, "Failed to allocate snd-soc-usb port\n");
        return -ENOMEM;
    }

    /* Set up callbacks if needed (optional for now) */
    // usb_port->connection_status_cb = sof_usb_connection_status_cb;

    ret = snd_soc_usb_add_port(usb_port);
    if (ret < 0) {
        dev_err(&auxdev->dev, "Failed to add snd-soc-usb port: %d\n", ret);
        snd_soc_usb_free_port(usb_port);
        return ret;
    }

    priv->usb_port = usb_port; /* Store for cleanup */
    
    /* Trigger re-discovery of already-connected USB audio devices */
    snd_usb_rediscover_devices();
    
    dev_info(&auxdev->dev,
         "SOF USB offload initialized (discovery only)\n");
    
    return 0;
}

/* Auxiliary driver remove */
static void sof_usb_offload_remove(struct auxiliary_device *auxdev)
{
    struct sof_usb_priv *priv = dev_get_drvdata(&auxdev->dev);
    
    dev_info(&auxdev->dev, "SOF USB offload removing...\n");
    
    /* Unregister platform ops */
    snd_usb_unregister_platform_ops(&priv->ops);
    
    /* Free snd-soc-usb port if allocated */
    if (priv->usb_port)
        snd_soc_usb_free_port(priv->usb_port);
    
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

MODULE_DESCRIPTION("SOF USB Audio Offload");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("SND_SOC_SOF_CLIENT");
MODULE_AUTHOR("Intel Corporation");
