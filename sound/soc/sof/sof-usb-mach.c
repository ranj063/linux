// SPDX-License-Identifier: GPL-2.0-only
/*
 * SOF USB Audio Offload - Machine Driver
 * Creates ASoC card with DAI links for USB audio streams
 */

#include <linux/auxiliary_bus.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include "../usb/usbaudio.h"
#include "../usb/card.h"

#define MAX_USB_STREAMS 8

struct sof_usb_stream_data {
    struct snd_usb_substream *usb_substream;
    int pcm_index;
    int direction;
};

struct sof_usb_mach_priv {
    struct snd_usb_audio *chip;
    struct snd_soc_card *card;
    struct sof_usb_stream_data streams[MAX_USB_STREAMS];
    int num_streams;
};

/* Build DAI links dynamically based on USB PCM streams */
static int sof_usb_create_dai_links(struct device *dev,
                    struct snd_usb_audio *chip,
                    struct sof_usb_mach_priv *priv,
                    struct snd_soc_dai_link **links_out,
                    int *num_links_out)
{
    struct snd_soc_dai_link *links;
    struct snd_usb_stream *usb_stream;
    int link_idx = 0;
    int stream_idx = 0;
    int num_streams = 0;
    
    /* Count streams first */
    list_for_each_entry(usb_stream, &chip->pcm_list, list) {
        if (usb_stream->substream[SNDRV_PCM_STREAM_PLAYBACK].num_formats)
            num_streams++;
        if (usb_stream->substream[SNDRV_PCM_STREAM_CAPTURE].num_formats)
            num_streams++;
    }
    
    if (num_streams == 0)
        return -EINVAL;
    
    links = devm_kcalloc(dev, num_streams, sizeof(*links), GFP_KERNEL);
    if (!links)
        return -ENOMEM;
    
    /* Create DAI links for each stream */
    list_for_each_entry(usb_stream, &chip->pcm_list, list) {
        int direction;
        
        for (direction = 0; direction < 2; direction++) {
            struct snd_usb_substream *subs = &usb_stream->substream[direction];
            struct snd_soc_dai_link *link;
            struct snd_soc_dai_link_component *cpus, *codecs, *platforms;
            char *name;
            
            if (!subs->num_formats)
                continue;
            
            if (link_idx >= num_streams)
                break;
            
            link = &links[link_idx];
            
            /* Allocate link name */
            name = devm_kasprintf(dev, GFP_KERNEL, "USB-%s-%d",
                         direction == SNDRV_PCM_STREAM_PLAYBACK ? 
                         "Playback" : "Capture",
                         usb_stream->pcm->device);
            if (!name)
                continue;
            
            link->name = name;
            link->stream_name = name;
            link->id = link_idx;
            
            /* CPU DAI - reference the one registered by sof-client-usb */
            cpus = devm_kzalloc(dev, sizeof(*cpus), GFP_KERNEL);
            if (!cpus)
                continue;
            cpus->dai_name = "USB-Offload";
            link->cpus = cpus;
            link->num_cpus = 1;
            
            /* Codec DAI (dummy for now) */
            codecs = devm_kzalloc(dev, sizeof(*codecs), GFP_KERNEL);
            if (!codecs)
                continue;
            codecs->dai_name = "snd-soc-dummy-dai";
            codecs->name = "snd-soc-dummy";
            link->codecs = codecs;
            link->num_codecs = 1;
            
            /* Platform (DSP) - should match SOF platform component name */
            platforms = devm_kzalloc(dev, sizeof(*platforms), GFP_KERNEL);
            if (!platforms)
                continue;
            platforms->name = "sof-audio-component";
            link->platforms = platforms;
            link->num_platforms = 1;
            
            /* This is a DPCM backend - no PCM device created */
            link->no_pcm = 1;
            if (direction == SNDRV_PCM_STREAM_PLAYBACK)
                link->dpcm_playback = 1;
            else
                link->dpcm_capture = 1;
            
            /* Store stream data */
            priv->streams[stream_idx].usb_substream = subs;
            priv->streams[stream_idx].pcm_index = usb_stream->pcm->device;
            priv->streams[stream_idx].direction = direction;
            link->drvdata = &priv->streams[stream_idx];
            
            stream_idx++;
            link_idx++;
            
            dev_dbg(dev, "Created DAI link: %s\n", name);
        }
    }
    
    priv->num_streams = stream_idx;
    *links_out = links;
    *num_links_out = link_idx;
    
    return 0;
}

/* Auxiliary driver probe */
static int sof_usb_mach_probe(struct auxiliary_device *auxdev,
                  const struct auxiliary_device_id *id)
{
    struct snd_usb_audio *chip = dev_get_drvdata(&auxdev->dev);
    struct sof_usb_mach_priv *priv;
    struct snd_soc_card *card;
    struct snd_soc_dai_link *links;
    int num_links;
    int ret;
    
    dev_info(&auxdev->dev, "SOF USB machine driver probing...\n");
    
    if (!chip) {
        dev_err(&auxdev->dev, "No USB audio chip found\n");
        return -ENODEV;
    }
    
    priv = devm_kzalloc(&auxdev->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;
    
    priv->chip = chip;
    
    /* Create ASoC card */
    card = devm_kzalloc(&auxdev->dev, sizeof(*card), GFP_KERNEL);
    if (!card)
        return -ENOMEM;
    
    /* Build DAI links from USB streams */
    ret = sof_usb_create_dai_links(&auxdev->dev, chip, priv, &links, &num_links);
    if (ret < 0) {
        dev_err(&auxdev->dev, "Failed to create DAI links: %d\n", ret);
        return ret;
    }
    
    card->name = "sof-usb-offload";
    card->dev = &auxdev->dev;
    card->owner = THIS_MODULE;
    card->dai_link = links;
    card->num_links = num_links;
    
    priv->card = card;
    snd_soc_card_set_drvdata(card, priv);
    
    /* Register card */
    ret = devm_snd_soc_register_card(&auxdev->dev, card);
    if (ret < 0) {
        dev_err(&auxdev->dev, "Failed to register card: %d\n", ret);
        return ret;
    }
    
    dev_info(&auxdev->dev,
         "SOF USB machine driver registered with %d DAI links\n",
         num_links);
    
    return 0;
}

static const struct auxiliary_device_id sof_usb_mach_table[] = {
    { .name = "snd_sof_usb_offload.usb-mach" },
    {},
};
MODULE_DEVICE_TABLE(auxiliary, sof_usb_mach_table);

static struct auxiliary_driver sof_usb_mach_driver = {
    .name = "usb-mach",
    .probe = sof_usb_mach_probe,
    .id_table = sof_usb_mach_table,
};

module_auxiliary_driver(sof_usb_mach_driver);

MODULE_DESCRIPTION("SOF USB Audio Offload Machine Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Corporation");