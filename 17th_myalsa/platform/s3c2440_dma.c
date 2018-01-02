#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/dma.h>


/* 参考 sound\soc\samsung\dma.c
 */

/* 1. 分配DMA BUFFER
 * 2. 从BUFFER取出period
 * 3. 启动DMA传输
 * 4. 传输完毕,更新状态(hw_ptr)
 *    2,3,4这部分主要有: request_irq, 触发DMA传输, 中断处理
 */

#define DMA0_BASE_ADDR  0x4B000000
#define DMA1_BASE_ADDR  0x4B000040
#define DMA2_BASE_ADDR  0x4B000080
#define DMA3_BASE_ADDR  0x4B0000C0

struct s3c_dma_regs {
	unsigned long disrc;
	unsigned long disrcc;
	unsigned long didst;
	unsigned long didstc;
	unsigned long dcon;
	unsigned long dstat;
	unsigned long dcsrc;
	unsigned long dcdst;
	unsigned long dmasktrig;
};

static volatile struct s3c_dma_regs *dma_regs;

struct s3c2440_dma_info {
    unsigned int buf_max_size;
    unsigned int buffer_size;
    unsigned int period_size;
    unsigned int phy_addr;
    unsigned int virt_addr;
    unsigned int dma_ofs;
    unsigned int be_running;
};

static struct s3c2440_dma_info playback_dma_info;

static const struct snd_pcm_hardware s3c2440_dma_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				    SNDRV_PCM_INFO_BLOCK_TRANSFER |
				    SNDRV_PCM_INFO_MMAP |
				    SNDRV_PCM_INFO_MMAP_VALID |
				    SNDRV_PCM_INFO_PAUSE |
				    SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_U16_LE |
				    SNDRV_PCM_FMTBIT_U8 |
				    SNDRV_PCM_FMTBIT_S8,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 128*1024,
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= PAGE_SIZE*2,
	.periods_min		= 2,
	.periods_max		= 128,
	.fifo_size		= 32,
};

/* 数据传输: 源,目的,长度 */
static void load_dma_period(void)
{       
	/* 把源,目的,长度告诉DMA */
	dma_regs->disrc      = playback_dma_info.phy_addr + playback_dma_info.dma_ofs;        /* 源的物理地址 */
	dma_regs->disrcc     = (0<<1) | (0<<0); /* 源位于AHB总线, 源地址递增 */
	dma_regs->didst      = 0x55000010;        /* 目的的物理地址 */
	dma_regs->didstc     = (0<<2) | (1<<1) | (1<<0); /* 目的位于APB总线, 目的地址不变 */

    /* bit22: 1-noreload */
	dma_regs->dcon       = (1<<31)|(0<<30)|(1<<29)|(0<<28)|(0<<27)|(0<<24)|(1<<23)|(1<<22)|(1<<20)|(playback_dma_info.period_size/2);  /* 使能中断,单个传输,硬件触发 */
}

static void s3c2440_dma_start(void)
{
	/* 启动DMA */
	dma_regs->dmasktrig  = (1<<1);
}

static void s3c2440_dma_stop(void)
{
	/* 启动DMA */
	dma_regs->dmasktrig  &= ~(1<<1);
}

static irqreturn_t s3c2440_dma2_irq(int irq, void *devid)
{
    struct snd_pcm_substream *substream = devid;
        
    /* 更新状态信息 */
    playback_dma_info.dma_ofs += playback_dma_info.period_size;
    if (playback_dma_info.dma_ofs >= playback_dma_info.buffer_size)
        playback_dma_info.dma_ofs = 0;
    
    /* 更新hw_ptr等信息,
     * 并且判断:如果buffer里没有数据了,则调用trigger来停止DMA 
     */
    snd_pcm_period_elapsed(substream);  

    if (playback_dma_info.be_running)
    {
        /* 如果还有数据
         * 1. 加载下一个period 
         * 2. 再次启动DMA传输
         */
        load_dma_period();
        s3c2440_dma_start();
    }

    return IRQ_HANDLED;
}

static int s3c2440_dma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
    int ret;

    /* 设置属性 */
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	snd_soc_set_runtime_hwparams(substream, &s3c2440_dma_hardware);

    /* 注册中断 */
    ret = request_irq(IRQ_DMA2, s3c2440_dma2_irq, IRQF_DISABLED, "myalsa for playback", substream);
    if (ret)
    {
        printk("request_irq error!\n");
        return -EIO;
    }

	return 0;
}


static int s3c2440_dma_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long totbytes = params_buffer_bytes(params);
    
    /* 根据params设置DMA */
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

    /* s3c2440_dma_new分配了很大的DMA BUFFER
     * params决定使用多大
     */
	runtime->dma_bytes            = totbytes;
    playback_dma_info.buffer_size = totbytes;
    playback_dma_info.period_size = params_period_bytes(params);

    return 0;
}


static int s3c2440_dma_prepare(struct snd_pcm_substream *substream)
{
    /* 准备DMA传输 */

    /* 复位各种状态信息 */
    playback_dma_info.dma_ofs = 0;
    playback_dma_info.be_running = 0;
    
    /* 加载第1个period */
    load_dma_period();

	return 0;
}


static int s3c2440_dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

    /* 根据cmd启动或停止DMA传输 */


	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        /* 启动DMA传输 */
        playback_dma_info.be_running = 1;
        s3c2440_dma_start();
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        /* 停止DMA传输 */
        playback_dma_info.be_running = 0;
        s3c2440_dma_stop();
		break;

	default:
		ret = -EINVAL;
		break;
	}


	return ret;
}


/* 返回结果是frame */
static snd_pcm_uframes_t s3c2440_dma_pointer(struct snd_pcm_substream *substream)
{
	return bytes_to_frames(substream->runtime, playback_dma_info.dma_ofs);
}


static int s3c2440_dma_close(struct snd_pcm_substream *substream)
{
    free_irq(IRQ_DMA2, substream);
    return 0;
}

static struct snd_pcm_ops s3c2440_dma_ops = {
	.open		= s3c2440_dma_open,
	.close		= s3c2440_dma_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= s3c2440_dma_hw_params,
	.prepare    = s3c2440_dma_prepare,
	.trigger	= s3c2440_dma_trigger,
	.pointer	= s3c2440_dma_pointer,	
};

static u64 dma_mask = DMA_BIT_MASK(32);

static int s3c2440_dma_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_pcm_substream *substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	int ret = 0;

    /* 1. 分配DMA BUFFER */
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &dma_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
	    playback_dma_info.virt_addr = (unsigned int)dma_alloc_writecombine(pcm->card->dev, s3c2440_dma_hardware.buffer_bytes_max,
					   &playback_dma_info.phy_addr, GFP_KERNEL);
        if (!playback_dma_info.virt_addr)
        {
            return -ENOMEM;
        }
        playback_dma_info.buf_max_size = s3c2440_dma_hardware.buffer_bytes_max;

    	buf->dev.type = SNDRV_DMA_TYPE_DEV;
    	buf->dev.dev = pcm->card->dev;
    	buf->private_data = NULL;
        buf->area = playback_dma_info.virt_addr;
        buf->bytes = playback_dma_info.buf_max_size;
	}

	return ret;
}


static void s3c2440_dma_free(struct snd_pcm *pcm)
{
	dma_free_writecombine(pcm->card->dev, playback_dma_info.buf_max_size,
			      (void *)playback_dma_info.virt_addr, playback_dma_info.phy_addr);
}

static struct snd_soc_platform_driver s3c2440_dma_platform = {
	.ops		= &s3c2440_dma_ops,
	.pcm_new	= s3c2440_dma_new,
	.pcm_free	= s3c2440_dma_free,
};

static int s3c2440_dma_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &s3c2440_dma_platform);
}
static int s3c2440_dma_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

static void s3c2440_dma_release(struct device * dev)
{
}

static struct platform_device s3c2440_dma_dev = {
    .name         = "s3c2440-dma",
    .id       = -1,
    .dev = { 
    	.release = s3c2440_dma_release, 
	},
};
struct platform_driver s3c2440_dma_drv = {
	.probe		= s3c2440_dma_probe,
	.remove		= s3c2440_dma_remove,
	.driver		= {
		.name	= "s3c2440-dma",
	}
};

static int s3c2440_dma_init(void)
{
    dma_regs = ioremap(DMA2_BASE_ADDR, sizeof(struct s3c_dma_regs));
    platform_device_register(&s3c2440_dma_dev);
    platform_driver_register(&s3c2440_dma_drv);
    return 0;
}

static void s3c2440_dma_exit(void)
{
    platform_device_unregister(&s3c2440_dma_dev);
    platform_driver_unregister(&s3c2440_dma_drv);
    iounmap(dma_regs);
}

module_init(s3c2440_dma_init);
module_exit(s3c2440_dma_exit);

MODULE_LICENSE("GPL");

