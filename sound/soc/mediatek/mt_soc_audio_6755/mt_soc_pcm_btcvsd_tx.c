/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mt_soc_pcm_btcvsd_tx.c
 *
 * Project:
 * --------
 *    Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio btcvsd playback
 *
 *
 *------------------------------------------------------------------------------
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/dma-mapping.h>
#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_digital_type.h"
#include "mt_soc_pcm_common.h"
#include "mt_soc_pcm_btcvsd.h"
#include <linux/time.h>



#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

static bool mPrepareDone;

int prev_sec; /* define 0 @ open */
long prev_usec;
long diff_msec;

static struct snd_pcm_hardware mtk_btcvsd_tx_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
	SNDRV_PCM_INFO_INTERLEAVED |
	SNDRV_PCM_INFO_RESUME |
	SNDRV_PCM_INFO_MMAP_VALID),
	.formats =   SND_SOC_ADV_MT_FMTS,
	.rates =        SOC_HIGH_USE_RATE,
	.rate_min =     SOC_HIGH_USE_RATE_MIN,
	.rate_max =     SOC_HIGH_USE_RATE_MAX,
	.channels_min =     SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max =     SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = SOC_NORMAL_USE_BUFFERSIZE_MAX,
	.period_bytes_max = SOC_NORMAL_USE_BUFFERSIZE_MAX,
	.periods_min =      SOC_NORMAL_USE_PERIODS_MIN,
	.periods_max =     SOC_NORMAL_USE_PERIODS_MAX,
	.fifo_size =        0,
};

static int mtk_pcm_btcvsd_tx_stop(struct snd_pcm_substream *substream)
{
	pr_warn("%s\n", __func__);

	return 0;
}

static snd_pcm_uframes_t prev_frame;
static kal_int32 prev_iPacket_r;

static snd_pcm_uframes_t mtk_pcm_btcvsd_tx_pointer(struct snd_pcm_substream
						 *substream)
{
	snd_pcm_uframes_t frame = 0;
	kal_uint32 byte = 0;
	static kal_int32 packet_diff;

	unsigned long flags;

	LOGBT("%s\n", __func__);

	spin_lock_irqsave(&auddrv_btcvsd_tx_lock, flags);

	/* get packet diff from last time */
	LOGBT("%s(), btsco.pTX->iPacket_r = %d, prev_iPacket_r = %d\n",
	      __func__,
	      btsco.pTX->iPacket_r,
	      prev_iPacket_r);
	if (btsco.pTX->iPacket_r >= prev_iPacket_r) {
		packet_diff = btsco.pTX->iPacket_r - prev_iPacket_r;
	} else {
		/* integer overflow */
		packet_diff = (INT_MAX - prev_iPacket_r) +
			      (btsco.pTX->iPacket_r - INT_MIN) + 1;
	}
	prev_iPacket_r = btsco.pTX->iPacket_r;

	/* increased bytes */
	byte = packet_diff * SCO_TX_ENCODE_SIZE;

	frame = audio_bytes_to_frame(substream , byte);
	frame += prev_frame;
	frame %= substream->runtime->buffer_size;

	prev_frame = frame;
	LOGBT("%s(), frame %lu, byte=%d, btsco.pTX->iPacket_r=%d\n",
	      __func__,
	      frame,
	      byte,
	      btsco.pTX->iPacket_r);

	spin_unlock_irqrestore(&auddrv_btcvsd_tx_lock, flags);

	return frame;
}

static int mtk_pcm_btcvsd_tx_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *hw_params)
{
	int ret = 0;
	struct snd_dma_buffer *dma_buf = &substream->dma_buffer;

	LOGBT("%s\n", __func__);

	if (params_period_size(hw_params) % SCO_TX_ENCODE_SIZE != 0) {
		pr_err("%s(), error, period size %d not valid\n",
		       __func__,
		       params_period_size(hw_params));
		return -EINVAL;
	}

	dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
	dma_buf->dev.dev = substream->pcm->card->dev;
	dma_buf->private_data = NULL;

	if (BT_CVSD_Mem.TX_btcvsd_dma_buf->area) {
		substream->runtime->dma_bytes = params_buffer_bytes(hw_params);
		substream->runtime->dma_area = BT_CVSD_Mem.TX_btcvsd_dma_buf->area;
		substream->runtime->dma_addr = BT_CVSD_Mem.TX_btcvsd_dma_buf->addr;
	}

	pr_warn("%s, 1 dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n", __func__,
	       substream->runtime->dma_bytes, substream->runtime->dma_area,
		   (long)substream->runtime->dma_addr);

	return ret;

}

static int mtk_pcm_btcvsd_tx_hw_free(struct snd_pcm_substream *substream)
{
	LOGBT("%s\n", __func__);

	if (BT_CVSD_Mem.TX_btcvsd_dma_buf->area)
		return 0;
	else
		return snd_pcm_lib_free_pages(substream);

	return 0;

}

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(soc_high_supported_sample_rates),
	.list = soc_high_supported_sample_rates,
	.mask = 0,
};

static int mtk_pcm_btcvsd_tx_close(struct snd_pcm_substream *substream)
{
	pr_warn("%s\n", __func__);

	int ret = 0;

	Set_BTCVSD_State(BT_SCO_TXSTATE_ENDING);
	Set_BTCVSD_State(BT_SCO_TXSTATE_IDLE);
	ret = AudDrv_btcvsd_Free_Buffer(0);

	BT_CVSD_Mem.TX_substream = NULL;

	if (mPrepareDone == true)
		mPrepareDone = false;

	return 0;
}

static int mtk_pcm_btcvsd_tx_open(struct snd_pcm_substream *substream)
{
	pr_warn("%s\n", __func__);

	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;

	ret = AudDrv_btcvsd_Allocate_Buffer(0);

	runtime->hw = mtk_btcvsd_tx_hardware;

	memcpy((void *)(&(runtime->hw)), (void *)&mtk_btcvsd_tx_hardware ,
		   sizeof(struct snd_pcm_hardware));

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
					 &constraints_sample_rates);

	BT_CVSD_Mem.TX_substream = substream;

	if (ret < 0)
		pr_warn("snd_pcm_hw_constraint_integer failed\n");

	if (ret < 0) {
		pr_err("ret < 0 mtk_pcm_btcvsd_tx_close\n");
		mtk_pcm_btcvsd_tx_close(substream);
		return ret;
	}

	return 0;
}

static int mtk_pcm_btcvsd_tx_prepare(struct snd_pcm_substream *substream)
{
	pr_warn("%s\n", __func__);

	struct snd_pcm_runtime *runtime = substream->runtime;

	Set_BTCVSD_State(BT_SCO_TXSTATE_RUNNING);

	return 0;
}

static int mtk_pcm_btcvsd_tx_start(struct snd_pcm_substream *substream)
{
	LOGBT("%s\n", __func__);

	prev_frame = 0;
	prev_iPacket_r = btsco.pTX->iPacket_r;

	return 0;
}

static int mtk_pcm_btcvsd_tx_trigger(struct snd_pcm_substream *substream, int cmd)
{
	LOGBT("%s, cmd=%d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_pcm_btcvsd_tx_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_pcm_btcvsd_tx_stop(substream);
	}
	return -EINVAL;
}

static int mtk_pcm_btcvsd_tx_copy(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos,
				void __user *dst, snd_pcm_uframes_t count)
{
	/* get total bytes to copy */
	count = audio_frame_to_bytes(substream , count);
	char *data_w_ptr = (char *)dst;
	AudDrv_btcvsd_write(data_w_ptr, count);

	LOGBT("pcm_copy return\n");
	return 0;
}

static int mtk_pcm_btcvsd_tx_silence(struct snd_pcm_substream *substream,
				   int channel, snd_pcm_uframes_t pos,
				   snd_pcm_uframes_t count)
{
	return 0; /* do nothing */
}

static int mtk_asoc_pcm_btcvsd_tx_new(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	LOGBT("%s\n", __func__);
	return ret;
}

static struct snd_pcm_ops mtk_btcvsd_tx_ops = {
	.open =     mtk_pcm_btcvsd_tx_open,
	.close =    mtk_pcm_btcvsd_tx_close,
	.ioctl =    snd_pcm_lib_ioctl,
	.hw_params =    mtk_pcm_btcvsd_tx_hw_params,
	.hw_free =  mtk_pcm_btcvsd_tx_hw_free,
	.prepare =  mtk_pcm_btcvsd_tx_prepare,
	.trigger =  mtk_pcm_btcvsd_tx_trigger,
	.pointer =  mtk_pcm_btcvsd_tx_pointer,
	.copy =     mtk_pcm_btcvsd_tx_copy,
	.silence =  mtk_pcm_btcvsd_tx_silence,
};

static struct snd_soc_platform_driver mtk_btcvsd_tx_soc_platform = {
	.ops        = &mtk_btcvsd_tx_ops,
	.pcm_new    = mtk_asoc_pcm_btcvsd_tx_new,
};

static int mtk_btcvsd_tx_probe(struct platform_device *pdev)
{
	int ret = 0;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_BTCVSD_TX_PCM);

	pr_warn("%s: dev name %s\n", __func__, dev_name(&pdev->dev));

	mDev_btcvsd_tx = &pdev->dev;

	/* init */
	if (!isProbeDone) {
		memset((void *)&BT_CVSD_Mem, 0, sizeof(CVSD_MEMBLOCK_T));
		isProbeDone = 1;
	}

	/* allocate dram */
	AudDrv_Allocate_mem_Buffer(mDev_btcvsd_tx, Soc_Aud_Digital_Block_MEM_BTCVSD_TX, sizeof(BT_SCO_TX_T));
	BT_CVSD_Mem.TX_btcvsd_dma_buf =  Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_BTCVSD_TX);

	return snd_soc_register_platform(&pdev->dev, &mtk_btcvsd_tx_soc_platform);
}

static int mtk_btcvsd_tx_remove(struct platform_device *pdev)
{
	LOGBT("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

/***************************************************************************
 * FUNCTION
 *  mtk_btcvsd_soc_platform_init / mtk_btcvsd_soc_platform_exit
 *
 * DESCRIPTION
 *  Module init and de-init (only be called when system boot up)
 *
 **************************************************************************/
#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_btcvsd_tx_of_ids[] = {
	{ .compatible = "mediatek,mt_soc_btcvsd_tx_pcm", },
	{}
};
#endif

static struct platform_driver mtk_btcvsd_tx_driver = {
	.driver = {
		.name = MT_SOC_BTCVSD_TX_PCM,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mt_soc_pcm_btcvsd_tx_of_ids,
#endif
	},
	.probe = mtk_btcvsd_tx_probe,
	.remove = mtk_btcvsd_tx_remove,
};


#ifndef CONFIG_OF
static struct platform_device *soc_mtk_btcvsd_tx_dev;
#endif

static int __init mtk_btcvsd_tx_soc_platform_init(void)
{
	int ret;
	pr_warn("+%s\n", __func__);
#ifndef CONFIG_OF
	soc_mtk_btcvsd_tx_dev = platform_device_alloc(MT_SOC_BTCVSD_TX_PCM, -1);
	if (!soc_mtk_btcvsd_tx_dev) {
		pr_warn("-%s, platform_device_alloc() fail, return\n", __func__);
		return -ENOMEM;
	}


	ret = platform_device_add(soc_mtk_btcvsd_tx_dev);
	if (ret != 0) {
		pr_warn("-%s, platform_device_add() fail, return\n", __func__);
		platform_device_put(soc_mtk_btcvsd_tx_dev);
		return ret;
	}
#endif

	/* Register platform DRIVER */
	ret = platform_driver_register(&mtk_btcvsd_tx_driver);
	if (ret) {
		pr_warn("-%s platform_driver_register Fail:%d\n", __func__, ret);
		return ret;
	}

	return ret;

}
module_init(mtk_btcvsd_tx_soc_platform_init);

static void __exit mtk_btcvsd_tx_soc_platform_exit(void)
{
	LOGBT("%s\n", __func__);

	platform_driver_unregister(&mtk_btcvsd_tx_driver);
}
module_exit(mtk_btcvsd_tx_soc_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");


