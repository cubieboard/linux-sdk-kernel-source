/* 
 * sunxi-ss.c - hardware cryptographic accelerator for Allwinner A20 SoC 
 * 
 * Copyright (C) 2013-2014 Corentin LABBE <clabbe....@gmail.com> 
 * 
 * Support AES cipher with 128,192,256 bits keysize. 
 * Support MD5 and SHA1 hash algorithms. 
 * Support PRNG 
 * 
 * You could find the datasheet at 
 * http://dl.linux-sunxi.org/A20/A20%20User%20Manual%202013-03-22.pdf 
 * 
 * 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation version 2 of the License 
 * 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA. 
 */ 
 
#include <linux/clk.h> 
#include <linux/crypto.h> 
#include <linux/io.h> 
#include <linux/module.h> 
#include <linux/of.h> 
#include <linux/platform_device.h> 
#include <crypto/hash.h> 
#include <crypto/internal/hash.h> 
#include <crypto/scatterwalk.h> 
#include <linux/scatterlist.h> 
#include <linux/interrupt.h> 
#include <linux/delay.h> 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_MD5 
#include <crypto/md5.h> 
#define SUNXI_SS_HASH_COMMON 
#endif 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_SHA1 
#include <crypto/sha.h> 
#define SUNXI_SS_HASH_COMMON 
#endif 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_AES 
#include <crypto/aes.h> 
#endif 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_PRNG 
#include <crypto/internal/rng.h> 
 
struct prng_context { 
        u8 seed[192/8]; 
        unsigned int slen; 
}; 
#endif 
#include <plat/dma_compat.h> 
 
#include "sunxi-ss.h" 
 
/* TODO DMA notes */ 
/* DMA arch/arm/mach-sun7i/include/mach/dma.h 
 * function declared in arch/arm/plat-sunxi/include/plat/dma_compat.h 
 * example drivers/spi/spi_sunxi.c 
 * example drivers/net/ethernet/allwinner/sunxi_emac.c 
 * example sound/soc/sunxi/spdif/sunxi_spdma.c 
 * example drivers/usb/sunxi_usb/udc/sw_udc_dma.c 
 * example drivers/usb/sunxi_usb/hcd/core/sw_hcd_dma.c 
 * */ 
 
/* General notes: 
 * I cannot use a key/IV cache because each time one of these change ALL stuff 
 * need to be re-writed. 
 * And for example, with dm-crypt IV changes on each request. 
 * 
 * After each request the device must be disabled. 
 * 
 * */ 
 
/*#define DEBUG_SS_DMA*/ 
/*#define DEBUG_DMA_P*/ 
 
#define MD5_BLOCK_SIZE    64 
#define AES_KEY_MAX_LENGTH 32 
 
#define DMA_MAGIC_RX 0x03030300 
#define DMA_MAGIC_TX 0x03030303 
/*#define SUNXI_SS_WAIT_QUEUE*/ 
 
static int use_dma; 
module_param_named(use_dma, use_dma, int, S_IRUGO | S_IWUSR); 
/* We use DMA only if we are forced to do so (use_dma=1) or if we think DMA 
 * will produce better performance (use_dma=2) 
 * The final driver certainly use only the last mode */ 
 
/* TODO check for alignment ? */ 
#define USE_DMA(len) ((use_dma == 1) || (use_dma == 2 && len > 1024)) 
 
static struct sunxi_ss_ctx { 
        void *base; 
        int irq; 
        struct clk *busclk; 
        struct clk *ssclk; 
        struct device *dev; 
        struct resource *res; 
        u64 byte_count; /* number of bytes "uploaded" to the device */ 
        u32 waiting; /* a word waiting to be uploaded to the device */ 
        unsigned int nbwait; /* number of bytes to be uploaded in the waiting word */ 
        void *buf_in; /* pointer to data to be uploaded to the device */ 
        size_t buf_in_size; 
        void *buf_out; 
        size_t buf_out_size; 
        u32 method;/* MD5/SHA1/AES*/ 
        int rxdma_init, txdma_init, rxdma_start, txdma_start; 
} _ss_ctx, *ss_ctx = &_ss_ctx; 
 
static DEFINE_MUTEX(lock); 
static DEFINE_MUTEX(bufout_lock); 
#ifdef SUNXI_SS_WAIT_QUEUE 
DECLARE_WAIT_QUEUE_HEAD(dma_queue); 
#endif 
 
struct sunxi_req_ctx { 
        u8 key[AES_KEY_MAX_LENGTH * 8]; 
        u32 keylen; 
        u32 mode; 
}; 
 
/*============================================================================*/ 
/*============================================================================*/ 
static struct sunxi_dma_params sstx_dma = { 
        .client.name    = "SSTX_DMA", 
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
        .channel        = DMACH_TSST, 
#endif 
        .dma_addr       = SUNXI_SS_REG_BASE + SUNXI_SS_TXFIFO, 
}; 
 
static struct sunxi_dma_params ssrx_dma = { 
        .client.name    = "SSRX_DMA", 
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
        .channel        = DMACH_TSSR, 
#endif 
        .dma_addr       = SUNXI_SS_REG_BASE + SUNXI_SS_RXFIFO, 
}; 
 
/*============================================================================*/ 
/*============================================================================*/ 
static void ssrx_dma_buffdone(struct sunxi_dma_params *dma, void *arg) 
{ 
#ifdef DEBUG_SS_DMA 
        dev_info(ss_ctx->dev, "DMA RX done\n"); 
#endif 
        ss_ctx->rxdma_start = 0; 
        /*atomic_set(&ss_ctx->rxdma_start, 0);*/ 
#ifdef SUNXI_SS_WAIT_QUEUE 
        wake_up_interruptible(&dma_queue); 
#endif 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static void sstx_dma_buffdone(struct sunxi_dma_params *dma, void *arg) 
{ 
#ifdef DEBUG_SS_DMA 
        dev_info(ss_ctx->dev, "DMA TX done\n"); 
#endif 
        ss_ctx->txdma_start = 0; 
        /*atomic_set(&ss_ctx->txdma_start, 0);*/ 
#ifdef SUNXI_SS_WAIT_QUEUE 
        wake_up_interruptible(&dma_queue); 
#endif 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* copied from drivers/net/ethernet/allwinner/sunxi_emac.c line 1534 */ 
/* use the same define for direction as DMA direction */ 
static int ss_sunxi_prepare_dma(int direction) 
{ 
        int ret; 
        if (direction == DMA_FROM_DEVICE) { 
                ret = sunxi_dma_request(&sstx_dma, 1);/* 1 for dedicated */ 
                if (ret < 0) { 
                        dev_err(ss_ctx->dev, "request DMA TX failed\n"); 
                        return -1; 
                } 
                ret = sunxi_dma_set_callback(&sstx_dma, sstx_dma_buffdone, 
                                ss_ctx); 
                if (ret != 0) { 
                        dev_err(ss_ctx->dev, "set_callback() error\n"); 
                        return ret; 
                } 
                ss_ctx->txdma_init = 1; 
                dev_info(ss_ctx->dev, "TX DMA is init\n"); 
                return 0; 
        } 
        ret = sunxi_dma_request(&ssrx_dma, 1);/* 1 for dedicated */ 
        /* TODO do a define for that value (dedicated)*/ 
        if (ret < 0) { 
                dev_err(ss_ctx->dev, "request DMA RX failed\n"); 
                return -1; 
        } 
        sunxi_dma_set_callback(&ssrx_dma, ssrx_dma_buffdone, ss_ctx); 
        ss_ctx->rxdma_init = 1; 
/*#ifdef DEBUG_SS_DMA*/ 
        dev_info(ss_ctx->dev, "RX DMA is init\n"); 
/*#endif*/ 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* release the DMA 
 * TODO check if it is currently running */ 
static int ss_sunxi_release_dma(struct sunxi_dma_params *sdma) 
{ 
        sunxi_dma_flush(sdma); 
        sunxi_dma_stop(sdma); 
        sunxi_dma_release(sdma); 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* prepare the DMA and send data */ 
/* notice the fact D_DST_SS_TX is an error, we send on RX 
 * same comment for the ss_dma_recv() who read from TX 
 * This is due to a typo/error in user manual 
 * TODO change the DMA header for correcting this */ 
static int ss_dma_send(dma_addr_t buff_addr, __u32 len, int start) 
{ 
        int ret; 
        dma_config_t ss_hwconf = { 
                .xfer_type = { 
                        .src_data_width = DATA_WIDTH_32BIT, 
                        .src_bst_len    = DATA_BRST_1, 
                        .dst_data_width = DATA_WIDTH_32BIT, 
                        .dst_bst_len    = DATA_BRST_1 
                }, 
                .address_type = { 
                        .src_addr_mode  = DDMA_ADDR_LINEAR, 
                        .dst_addr_mode  = DDMA_ADDR_IO 
                }, 
                .bconti_mode    = false, 
                .src_drq_type   = D_SRC_SDRAM, 
                .dst_drq_type   = D_DST_SS_TX, 
                .irq_spt        = CHAN_IRQ_FD 
        }; 
        /* value sended for DMA_OP_SET_PARA_REG 
         * 0x7f077f07 for nand and 0x03030303 for emac 
         * For the moment I put the same value of emac, must investigate more 
         * */ 
        ret = sunxi_dma_config(&ssrx_dma, &ss_hwconf, DMA_MAGIC_RX); 
        if (ret != 0) 
                return ret; 
        ret = sunxi_dma_enqueue(&ssrx_dma, buff_addr, len, 0); 
        if (ret != 0) 
                return ret; 
        if (start == 0) 
                return 0; 
#ifdef DEBUG_SS_DMA 
        dev_info(ss_ctx->dev, "RX DMA started len=%u\n", len); 
#endif 
        ss_ctx->rxdma_start = 1; 
        ret = sunxi_dma_start(&ssrx_dma); 
        if (ret != 0) { 
                dev_err(ss_ctx->dev, "ERROR sunxi_dma_start\n"); 
                ss_ctx->rxdma_start = 0; 
                return ret; 
        } 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* prepare the DMA for receiving data from the device */ 
static int ss_dma_recv(dma_addr_t buff_addr, __u32 len) 
{ 
        int ret; 
        dma_config_t ss_rx_hwconf = { 
                .xfer_type = { 
                        .src_data_width = DATA_WIDTH_32BIT, 
                        .src_bst_len    = DATA_BRST_1, 
                        .dst_data_width = DATA_WIDTH_32BIT, 
                        .dst_bst_len    = DATA_BRST_1 
                }, 
                .address_type = { 
                        .src_addr_mode  = DDMA_ADDR_IO, 
                        .dst_addr_mode  = DDMA_ADDR_LINEAR 
                }, 
                .bconti_mode    = false, 
                .src_drq_type   = D_SRC_SS_RX, 
                .dst_drq_type   = D_DST_SDRAM, 
                .irq_spt        = CHAN_IRQ_FD 
        }; 
        ret = sunxi_dma_config(&sstx_dma, &ss_rx_hwconf, DMA_MAGIC_TX); 
        if (ret != 0) { 
                dev_err(ss_ctx->dev, "sunxi_dma_config() error\n"); 
                return ret; 
        } 
        /* TODO a DEFINE must be done for this 1 */ 
        ret = sunxi_dma_enqueue(&sstx_dma, buff_addr, len, 1); 
        if (ret != 0) { 
                dev_err(ss_ctx->dev, "sunxi_dma_enqueue() error\n"); 
                return ret; 
        } 
#ifdef DEBUG_SS_DMA 
        dev_info(ss_ctx->dev, "TX DMA started from=%x len=%u\n", 
                        buff_addr, len); 
#endif 
        ss_ctx->txdma_start = 1; 
        ret = sunxi_dma_start(&sstx_dma); 
        if (ret != 0) { 
                dev_err(ss_ctx->dev, "ERROR sunxi_dma_start()\n"); 
                ss_ctx->txdma_start = 0; 
                return ret; 
        } 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* derive number of elements in scatterlist 
 * Taken from drivers/crypto/talitos.c 
 * TODO Need to be replaced by sg_nents() when mainlining for linux > 3.7 
 */ 
static int sg_count(struct scatterlist *sg_list, int nbytes) 
{ 
        struct scatterlist *sg = sg_list; 
        int sg_nents = 0; 
        while (nbytes > 0) { 
                sg_nents++; 
                nbytes -= sg->length; 
                sg = scatterwalk_sg_next(sg); 
        } 
        return sg_nents; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* This function try to calculate the good value to be set on trigger register 
 * Work not finished, need lots of test/bench */ 
#define CHANGE_TRIG_RX 1 
#define CHANGE_TRIG_TX 2 
static void sunxi_ss_trigger(const size_t len_r, const size_t len_t, 
                char *trig_rx, char *trig_tx, const int flag) 
{ 
        char cur_trig_rx = *trig_rx; 
        char cur_trig_tx = *trig_tx; 
        char apply_change = 0; 
        if ((flag & CHANGE_TRIG_RX) > 0) { 
                *trig_rx = 0x03; 
                if (len_r > 7 && (len_r / 4) % 8 == 0) 
                        *trig_rx = 0x07; 
                if (len_r > 15 && (len_r / 4) % 16 == 0) 
                        *trig_rx = 0x0F; 
                if (len_r > 31 && (len_r / 4) % 32 == 0) 
                        *trig_rx = 0x1F; 
                if (cur_trig_rx != *trig_rx) 
                        apply_change = 1; 
        } 
        if ((flag & CHANGE_TRIG_TX) > 0) { 
                *trig_tx = 0x03; 
                if (len_t > 7 && (len_t / 4) % 8 == 0) 
                        *trig_tx = 0x03; 
                if (len_t > 15 && (len_t / 4) % 16 == 0) 
                        *trig_tx = 0x03; 
                if (len_t > 31 && (len_t / 4) % 32 == 0) 
                        *trig_tx = 0x03; 
                if (cur_trig_tx != *trig_tx) 
                        apply_change = 1; 
        } 
/*        dev_info(ss_ctx->dev, "DEBUG %u %u choose %x %x %x\n", len_r, len_t, 
                        *trig_rx, *trig_tx, flag);*/ 
        if (apply_change == 1) 
                iowrite32(((*trig_rx) << 8) | *trig_tx, 
                                ss_ctx->base + SUNXI_SS_FCSR); 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* sunxi_ss_send: sends 32 bits through the FIFO 
 * 
 * It maintains a count of the number of free spaces on the queue 
 * to keep a decent speed. 
 * TODO reset the value after a DMA ? 
 */ 
static void sunxi_ss_send(struct sunxi_ss_ctx *ctx, u32 value) 
{ 
        static int spaces; 
        u32 tmp; 
 
/*        dev_info(ss_ctx->dev, "DEBUG %s space=%d value=0x%08x\n", __func__, spaces, value);*/ 
 
        if (spaces < 1) { 
                /* Wait until there are free spaces, then take note of 
                 * how many of them there are */ 
                do { 
                        tmp = ioread32(ctx->base + SUNXI_SS_FCSR); 
                } while ((tmp & SUNXI_RXFIFO_FREE) == 0); 
                spaces = SUNXI_RXFIFO_SPACES(tmp); 
                if (spaces < 1) { 
                        dev_info(ss_ctx->dev, "DEBUG ERROR %s space=%d\n", __func__, spaces); 
                        return; 
                } 
        } 
 
        iowrite32(value, ctx->base + SUNXI_SS_RXFIFO); 
        spaces--; 
} 
 
#ifdef SUNXI_SS_HASH_COMMON 
/*============================================================================*/ 
/*============================================================================*/ 
/* sunxi_hash_init: initialize request context 
 * Activate the SS, and configure it for MD5 or SHA1 
 */ 
static int sunxi_hash_init(struct ahash_request *areq) 
{ 
        const char *hash_type; 
        u32 tmp = 0; 
        mutex_lock(&lock); 
 
        hash_type = crypto_tfm_alg_name(areq->base.tfm); 
 
        ss_ctx->byte_count = 0; 
        ss_ctx->nbwait = 0; 
        ss_ctx->waiting = 0; 
 
        /* Enable and configure SS for MD5 or SHA1, using IVs on hardware */ 
        ss_ctx->method = SUNXI_OP_MD5; 
        tmp |= SUNXI_OP_MD5; 
        if (strcmp(hash_type, "sha1") == 0) { 
                tmp = SUNXI_OP_SHA1; 
                ss_ctx->method = SUNXI_OP_SHA1; 
        } 
        tmp |= SUNXI_IV_CONSTANTS; 
        tmp |= SUNXI_SS_ENABLED; 
 
        iowrite32(tmp, ss_ctx->base + SUNXI_SS_CTL); 
 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* 
 * sunxi_hash_update: update hash engine 
 * 
 * Could be used for both SHA1 and MD5 
 * This function take data from scatterlist (areq->src) and copy it in buf_in 
 * Then we get by step of 32bits the data and put then in the SS. 
 * The remaining data is stored in ss_ctx->waiting 
 */ 
static int sunxi_hash_update(struct ahash_request *areq) 
{ 
        size_t i; 
        size_t bufcnt; 
        size_t nb_in_sg; 
        size_t off; 
        unsigned int total; 
        int count; 
        struct scatterlist *sg; 
        int ret; 
        u32 v; 
 
        u8 *waiting = (u8 *)&(ss_ctx->waiting); 
 
        if (areq->nbytes == 0) 
                return 0; 
 
        total = areq->nbytes; 
        nb_in_sg = sg_count(areq->src, areq->nbytes); 
 
/*        dev_info(ss_ctx->dev, "DEBUG total=%d nbsg=%d\n", total, nb_in_sg);*/ 
 
        /* this a try to do a direct access of data from SG in CPU mode 
         * strange it works... */ 
        if (total % 4 == 0 && USE_DMA(total) == 0 && ss_ctx->nbwait == 0) { 
                /* taken from code of sg_copy_buffer at lib/scatterlist.c */ 
                unsigned int offset = 0; 
                struct sg_mapping_iter miter; 
                unsigned long flags; 
                unsigned int sg_flags = SG_MITER_ATOMIC; 
 
                sg_flags |= SG_MITER_FROM_SG; 
 
                sg_miter_start(&miter, areq->src, nb_in_sg, sg_flags); 
 
                local_irq_save(flags); 
                while (sg_miter_next(&miter) && offset < total) { 
                        unsigned int len, toff; 
                        len = min(miter.length, total - offset); 
                        toff = 0; 
                        while (toff < len) { 
                                v = *(u32 *)(miter.addr + toff); 
                                sunxi_ss_send(ss_ctx, v); 
                                toff += 4; 
                        } 
                        ss_ctx->byte_count += len; 
                        offset += len; 
                } 
                sg_miter_stop(&miter); 
                local_irq_restore(flags); 
                return 0; 
        } 
 
        off = 0; 
        /* DMA stuff */ 
        /* for the moment only use DMA if we could send all data in one DMA */ 
        /* TODO move all this to another function for solving lines > 80 column:) */ 
        if (total > 3 && total % 4 == 0 && USE_DMA(total) && 
                        ss_ctx->nbwait == 0) { 
#ifdef DEBUG_SS_DMA 
                dev_info(ss_ctx->dev, "Lets go for DMA\n"); 
#endif 
                if (ss_ctx->rxdma_init == 0) 
                        ss_sunxi_prepare_dma(DMA_TO_DEVICE); 
                if (ss_ctx->rxdma_init == 1) { 
                        /* enable DRQ */ 
                        iowrite32(SUNXI_SS_ICS_DRA_ENABLE, 
                                        ss_ctx->base + SUNXI_SS_ICSR); 
 
                        /* TODO better error handling */ 
                        count = dma_map_sg(ss_ctx->dev, areq->src, nb_in_sg, DMA_TO_DEVICE); 
                        if (count < 0) { 
                                dev_err(ss_ctx->dev, "dma_map_sg error\n"); 
                                iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                                mutex_unlock(&lock); 
                                return -1; 
                        } 
/*                        dev_info(ss_ctx->dev, "DMA count=%d nbsg=%u\n", count, nb_in_sg);*/ 
                        for_each_sg(areq->src, sg, count, i) { 
                                if (sg_dma_len(sg) % 4 != 0) { 
                                        /* TODO */ 
                                        dev_err(ss_ctx->dev, "DMA TODO dma_len non 4\n"); 
                                        iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                                        mutex_unlock(&lock); 
                                        return -EINVAL; 
                                } 
                                ret = ss_dma_send(sg_dma_address(sg), sg_dma_len(sg), 1); 
                                if (ret != 0) { 
                                        dev_err(ss_ctx->dev, "DMA ss_dma_send error\n"); 
                                        iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                                        mutex_unlock(&lock); 
                                        return ret; 
                                } 
#ifdef SUNXI_SS_WAIT_QUEUE 
                                wait_event_interruptible_timeout(dma_queue, 
                                                (ss_ctx->rxdma_start != 1), 
                                                HZ * 10); 
#endif 
#define SUNXI_SS_DMA_TIMEOUT (1000 * 1000) 
                                for (ret = 0; ret < SUNXI_SS_DMA_TIMEOUT; ret++) { 
                                        if (ss_ctx->rxdma_start == 0) 
                                                break; 
                                        usleep_range(1, 2); 
                                } 
                                if (ret >= SUNXI_SS_DMA_TIMEOUT) 
                                        dev_warn(ss_ctx->dev, 
                                                        "DMA wait timeout=\n"); 
#ifdef DEBUG_SS_DMA 
                                dev_info(ss_ctx->dev, "DMA ended\n"); 
#endif 
                                ret = sunxi_dma_stop(&ssrx_dma); 
                                if (ret != 0) 
                                        dev_err(ss_ctx->dev, "DMA could not be stopped\n"); 
#ifdef DEBUG_SS_DMA 
                                else 
                                        dev_info(ss_ctx->dev, "DMA stopped\n"); 
#endif 
                        } 
                        off = total; 
                        ss_ctx->byte_count += total; 
                        dma_unmap_sg(ss_ctx->dev, areq->src, nb_in_sg, DMA_TO_DEVICE); 
                        iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                } 
                return 0; 
        } 
        /* end DMA */ 
 
        if (ss_ctx->buf_in == NULL) { 
                ss_ctx->buf_in = kmalloc(areq->nbytes, GFP_KERNEL); 
                ss_ctx->buf_in_size = areq->nbytes; 
        } else { 
                if (areq->nbytes > ss_ctx->buf_in_size) { 
                        kfree(ss_ctx->buf_in); 
                        ss_ctx->buf_in = kmalloc(areq->nbytes, GFP_KERNEL); 
                        ss_ctx->buf_in_size = areq->nbytes; 
                } 
        } 
        if (ss_ctx->buf_in == NULL) { 
                dev_err(ss_ctx->dev, "Unable to allocate pages.\n"); 
                iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
                mutex_unlock(&lock); 
                return -ENOMEM; 
        } 
 
        bufcnt = sg_copy_to_buffer(areq->src, nb_in_sg, ss_ctx->buf_in, total); 
        if (!bufcnt) { 
                mutex_unlock(&lock); 
                return -EINVAL; 
        } 
 
        while (bufcnt - off > 3) { 
                /* TODO memcpy ? */ 
                /*waiting[0] = ((u8 *)ss_ctx->buf_in)[off]; 
                waiting[1] = ((u8 *)ss_ctx->buf_in)[off + 1]; 
                waiting[2] = ((u8 *)ss_ctx->buf_in)[off + 2]; 
                waiting[3] = ((u8 *)ss_ctx->buf_in)[off + 3];*/ 
                ss_ctx->waiting = *(u32 *)(ss_ctx->buf_in + off); 
                sunxi_ss_send(ss_ctx, ss_ctx->waiting); 
                /*sunxi_ss_send(ss_ctx, *(u32 *)(ss_ctx->buf_in + off));*/ 
                off += 4; 
                ss_ctx->byte_count += 4; 
                ss_ctx->waiting = 0; 
        } 
        if (bufcnt == off) 
                return 0; 
        if (bufcnt - off < 4) { 
                /* copy it in the waiting buffer */ 
                for (i = 0; i < bufcnt - off; i++) { 
                        waiting[ss_ctx->nbwait] = ((u8 *)ss_ctx->buf_in)[off + i]; 
                        ss_ctx->nbwait++; 
                        if (ss_ctx->nbwait == 4) { 
                                /* now we have 4 bytes to send */ 
                                ss_ctx->nbwait = 0; 
                                /*dev_info(ss_ctx->dev, "Sending %x\n", ss_ctx->waiting);*/ 
                                sunxi_ss_send(ss_ctx, ss_ctx->waiting); 
                                ss_ctx->waiting = 0; 
                                ss_ctx->byte_count += 4; 
                        } 
                } 
                /*dev_info(ss_ctx->dev, "DEBUG %s remains %d bytes\n", __func__, 
                                ss_ctx->nbwait);*/ 
        } 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* 
 * sunxi_hash_final: finalize hashing operation 
 * 
 * If we have some remaining bytes, send it. 
 * Then ask the SS for finalizing the hash 
 */ 
static int sunxi_hash_final(struct ahash_request *areq) 
{ 
        u32 v; 
        unsigned int i; 
        int zeros; 
        unsigned int index, padlen; 
        __be64 bits; 
 
/*        dev_info(ss_ctx->dev, "DEBUG %s base=%p buf_in=%p nbwait=%d\n", 
                        __func__, ss_ctx->base, ss_ctx->buf_in, ss_ctx->nbwait); 
*/ 
        if (ss_ctx->nbwait > 0) { 
                /*dev_info(ss_ctx->dev, "We have %d remaining bytes to send\n", 
                                ss_ctx->nbwait);*/ 
                ss_ctx->waiting |= ((1 << 7) << (ss_ctx->nbwait * 8)); 
                sunxi_ss_send(ss_ctx, ss_ctx->waiting); 
        } else 
                sunxi_ss_send(ss_ctx, (1 << 7)); 
 
        /* number of space to pad to obtain 64o minus 8(size) minus 4 (final 1) 
         * example len=0 
         * example len=56 
         * */ 
 
        /* we have already send 4 more byte of which nbwait data */ 
        index = (ss_ctx->byte_count + 4) & 0x3f; 
        ss_ctx->byte_count += ss_ctx->nbwait; 
        if (index > 56) 
                zeros = (120 - index) / 4; 
        else 
                zeros = (56 - index) / 4; 
/*        dev_info(ss_ctx->dev, "DEBUG zeros=%d index=%u\n", zeros, index);*/ 
        if (ss_ctx->method == SUNXI_OP_SHA1) { 
                index = ss_ctx->byte_count & 0x3f; 
                padlen = (index < 56) ? (56 - index) : ((64+56) - index); 
                zeros = (padlen - 1) / 4; 
        } 
        /* This should not happen, TODO set a unlikely() ? */ 
        if (zeros > 64 || zeros < 0) { 
                dev_err(ss_ctx->dev, "ERROR DEBUG too many zeros len=%u\n", 
                                ss_ctx->byte_count); 
                zeros = 0; 
        } 
        for (i = 0; i < zeros; i++) 
                sunxi_ss_send(ss_ctx, 0); 
 
        /* send the lenght */ 
        if (ss_ctx->method == SUNXI_OP_SHA1) { 
                bits = cpu_to_be64(ss_ctx->byte_count << 3); 
                sunxi_ss_send(ss_ctx, bits & 0xffffffff); 
                sunxi_ss_send(ss_ctx, (bits >> 32) & 0xffffffff); 
        } else { 
                sunxi_ss_send(ss_ctx, (ss_ctx->byte_count << 3) & 0xffffffff); 
                sunxi_ss_send(ss_ctx, (ss_ctx->byte_count >> 29) & 0xffffffff); 
        } 
 
        /* stop the hashing */ 
        v = ioread32(ss_ctx->base + SUNXI_SS_CTL); 
        v |= SUNXI_SS_DATA_END; 
        iowrite32(v, ss_ctx->base + SUNXI_SS_CTL); 
 
        /* check the end */ 
#define SUNXI_SS_TIMEOUT 100 
        i = 0; 
        do { 
                v = ioread32(ss_ctx->base + SUNXI_SS_CTL); 
                i++; 
        } while (i < SUNXI_SS_TIMEOUT && (v & SUNXI_SS_DATA_END) > 0); 
        if (i >= SUNXI_SS_TIMEOUT) 
                dev_err(ss_ctx->dev, "SUNXI_SS_TIMEOUT %d>%d\n", 
                                i, SUNXI_SS_TIMEOUT); 
 
        if (ss_ctx->method == SUNXI_OP_SHA1) { 
                for (i = 0; i < 5; i++) { 
                        v = cpu_to_be32(ioread32(ss_ctx->base + SUNXI_SS_MD0 + i * 4)); 
                        memcpy(areq->result + i * 4, &v, 4); 
                } 
        } else { 
                for (i = 0; i < 4; i++) { 
                        v = ioread32(ss_ctx->base + SUNXI_SS_MD0 + i * 4); 
                        memcpy(areq->result + i * 4, &v, 4); 
                        /*memcpy_fromio(areq->result + i * 4, 
                          ss_ctx->base + SUNXI_SS_MD0 + i * 4, 4);*/ 
                } 
        } 
        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
        mutex_unlock(&lock); 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/*sunxi_hash_finup: finalize hashing operation */ 
static int sunxi_hash_finup(struct ahash_request *areq) 
{ 
        int err; 
 
        err = sunxi_hash_update(areq); 
        if (err != 0) 
                return err; 
 
        return sunxi_hash_final(areq); 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* combo of init/update/final functions */ 
static int sunxi_hash_digest(struct ahash_request *areq) 
{ 
        int err; 
 
        err = sunxi_hash_init(areq); 
        if (err != 0) 
                return err; 
 
        err = sunxi_hash_update(areq); 
        if (err != 0) 
                return err; 
 
        return sunxi_hash_final(areq); 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static struct ahash_alg sunxi_md5_alg = { 
        .init = sunxi_hash_init, 
        .update = sunxi_hash_update, 
        .final = sunxi_hash_final, 
        .finup = sunxi_hash_finup, 
        .digest = sunxi_hash_digest, 
        .halg = { 
                .digestsize = MD5_DIGEST_SIZE, 
                .base = { 
                        .cra_name = "md5", 
                        .cra_driver_name = "sunxi-md5", 
                        .cra_priority = 100, 
                        .cra_flags = CRYPTO_ALG_TYPE_AHASH, 
                        .cra_blocksize = MD5_BLOCK_SIZE, 
                        /*.cra_ctxsize = sizeof(struct sunxi_req_ctx),*/ 
                        .cra_ctxsize = 0, 
                        .cra_module = THIS_MODULE, 
                        .cra_type = &crypto_ahash_type 
                } 
        } 
}; 
 
static struct ahash_alg sunxi_sha1_alg = { 
        .init = sunxi_hash_init, 
        .update = sunxi_hash_update, 
        .final = sunxi_hash_final, 
        .finup = sunxi_hash_finup, 
        .digest = sunxi_hash_digest, 
        .halg = { 
                .digestsize = SHA1_DIGEST_SIZE, 
                .base = { 
                        .cra_name = "sha1", 
                        .cra_driver_name = "sunxi-sha1", 
                        .cra_priority = 100, 
                        .cra_flags = CRYPTO_ALG_TYPE_AHASH, 
/*                        .cra_blocksize = SHA1_BLOCK_SIZE,*/ 
                        .cra_ctxsize = sizeof(struct sunxi_req_ctx), 
                        .cra_module = THIS_MODULE, 
                        .cra_type = &crypto_ahash_type 
                } 
        } 
}; 
#endif /* ifdef SUNXI_SS_HASH_COMMON */ 
 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_AES 
/*============================================================================*/ 
/*============================================================================*/ 
/* check and set the AES key, prepare the mode to be used */ 
static int sunxi_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key, 
                unsigned int keylen) 
{ 
        struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm); 
        if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 && 
                keylen != AES_KEYSIZE_256) { 
                dev_err(ss_ctx->dev, "Invalid keylen %u\n", keylen); 
                crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN); 
                return -EINVAL; 
        } 
        op->keylen = keylen; 
        memcpy(op->key, key, keylen); 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* Experimental function with kmap_atomic() */ 
static int sunxi_aes_poll_kmap_atomic(struct ablkcipher_request *areq, int flag) 
{ 
        u32 tmp, value; 
        size_t ir, it; 
        struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq); 
        struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm); 
        u32 tx_cnt = 0; 
        u32 rx_cnt = 0; 
        void *dst_addr; 
        void *src_addr; 
        struct scatterlist *in_sg; 
        struct scatterlist *out_sg; 
        size_t len_rx = 0, len_tx = 0; 
        unsigned long flags; 
        int antibug = 0; 
 
        tmp = 0; 
        tmp |= SUNXI_SS_CBC; 
        tmp |= SUNXI_OP_AES; 
        tmp |= SUNXI_KEYSELECT_KEYN; 
        tmp |= SUNXI_SS_ENABLED; 
        tmp |= flag; 
        switch (op->keylen) { 
        case 128 / 8: 
                tmp |= SUNXI_AES_128BITS; 
                break; 
        case 192 / 8: 
                tmp |= SUNXI_AES_192BITS; 
                break; 
        case 256 / 8: 
                tmp |= SUNXI_AES_256BITS; 
                break; 
        } 
        iowrite32(tmp, ss_ctx->base + SUNXI_SS_CTL); 
 
        in_sg = areq->src; 
        out_sg = areq->dst; 
 
        ir = 0; 
        it = 0; 
        do { 
                if (rx_cnt == 0 || tx_cnt == 0) { 
                        tmp = ioread32(ss_ctx->base + SUNXI_SS_FCSR); 
                        rx_cnt = SUNXI_RXFIFO_SPACES(tmp); 
                        tx_cnt = SUNXI_TXFIFO_SPACES(tmp); 
                } 
                if (rx_cnt > 0 && len_rx < areq->nbytes && in_sg != NULL) { 
                        antibug = 0; 
                        local_irq_save(flags); 
                        src_addr = kmap_atomic(sg_page(in_sg)); 
                        do { 
                                value = *(u32 *)(src_addr + in_sg->offset + ir); 
                                iowrite32(value, ss_ctx->base + SUNXI_SS_RXFIFO); 
                                ir += 4; 
                                len_rx += 4; 
                                rx_cnt--; 
                                antibug++; 
                        } while (rx_cnt > 0 && antibug < 6); 
                        kunmap_atomic(src_addr); 
                        local_irq_restore(flags); 
                } 
                if (tx_cnt > 0 && len_tx < areq->nbytes && out_sg != NULL && 
                                len_tx < len_rx) { 
                        antibug = 0; 
                        local_irq_save(flags); 
                        dst_addr = kmap_atomic(sg_page(out_sg)); 
                        do { 
                                value = ioread32(ss_ctx->base + SUNXI_SS_TXFIFO); 
                                *(u32 *)(dst_addr + out_sg->offset + it) = value; 
                                it += 4; 
                                len_tx += 4; 
                                tx_cnt--; 
                                antibug++; 
                        } while (tx_cnt > 0 && antibug < 5); 
                        kunmap_atomic(dst_addr); 
                        local_irq_restore(flags); 
                } 
                if (in_sg != NULL && ir >= in_sg->length) { 
                        in_sg = sg_next(in_sg); 
                        ir = 0; 
                } 
                if (out_sg != NULL && it >= out_sg->length) { 
                        out_sg = sg_next(out_sg); 
                        it = 0; 
                } 
        } while (out_sg != NULL); 
 
        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
        mutex_unlock(&lock); 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* Experimental function with kmap() */ 
static int sunxi_aes_poll_kmap(struct ablkcipher_request *areq, int flag) 
{ 
        u32 tmp, value; 
        size_t ir, it; 
        struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq); 
        struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm); 
        u32 tx_cnt = 0; 
        u32 rx_cnt = 0; 
        void *dst_addr; 
        void *src_addr; 
        struct scatterlist *in_sg; 
        struct scatterlist *out_sg; 
        size_t len_rx = 0, len_tx = 0; 
 
        tmp = 0; 
        tmp |= SUNXI_SS_CBC; 
        tmp |= SUNXI_OP_AES; 
        tmp |= SUNXI_KEYSELECT_KEYN; 
        tmp |= SUNXI_SS_ENABLED; 
        tmp |= flag; 
        switch (op->keylen) { 
        case 128 / 8: 
                tmp |= SUNXI_AES_128BITS; 
                break; 
        case 192 / 8: 
                tmp |= SUNXI_AES_192BITS; 
                break; 
        case 256 / 8: 
                tmp |= SUNXI_AES_256BITS; 
                break; 
        } 
        iowrite32(tmp, ss_ctx->base + SUNXI_SS_CTL); 
 
        in_sg = areq->src; 
        out_sg = areq->dst; 
 
        src_addr = kmap(sg_page(in_sg)); 
        dst_addr = kmap(sg_page(out_sg)); 
        dev_info(ss_ctx->dev, "DEBUG KMAP %p %p %p %p %d\n", src_addr, dst_addr, 
                        sg_page(in_sg), sg_page(out_sg), out_sg->offset); 
        if (src_addr == NULL || dst_addr == NULL) { 
                dev_err(ss_ctx->dev, "KMAP error\n"); 
                goto end; 
        } 
        ir = 0; 
        it = 0; 
        do { 
                if (rx_cnt == 0 || tx_cnt == 0) { 
                        tmp = ioread32(ss_ctx->base + SUNXI_SS_FCSR); 
                        rx_cnt = SUNXI_RXFIFO_SPACES(tmp); 
                        tx_cnt = SUNXI_TXFIFO_SPACES(tmp); 
                } 
                if (rx_cnt > 0 && len_rx < areq->nbytes && in_sg != NULL) { 
                        /*src_addr = kmap_atomic(sg_page(in_sg));*/ 
                        value = *(u32 *)(src_addr + in_sg->offset + ir); 
                        /*kunmap_atomic(src_addr);*/ 
                        iowrite32(value, ss_ctx->base + SUNXI_SS_RXFIFO); 
                        ir += 4; 
                        len_rx += 4; 
                        rx_cnt--; 
                } 
                if (tx_cnt > 0 && len_tx < areq->nbytes && out_sg != NULL && 
                                len_tx < len_rx) { 
                        value = ioread32(ss_ctx->base + SUNXI_SS_TXFIFO); 
                        /*dst_addr = kmap_atomic(sg_page(out_sg));*/ 
                        *(u32 *)(dst_addr + out_sg->offset + it) = value; 
                        /*kunmap_atomic(dst_addr);*/ 
                        it += 4; 
                        len_tx += 4; 
                        tx_cnt--; 
                } 
                if (in_sg != NULL && ir >= in_sg->length) { 
                        kunmap(src_addr); 
                        in_sg = sg_next(in_sg); 
                        ir = 0; 
                        if (in_sg != NULL) { 
                                src_addr = kmap(sg_page(in_sg)); 
                                if (src_addr == NULL) { 
                                        dev_err(ss_ctx->dev, "KMAP src error\n"); 
                                        goto end; 
                                } 
                        } 
                } 
                if (out_sg != NULL && it >= out_sg->length) { 
                        kunmap(dst_addr); 
                        out_sg = sg_next(out_sg); 
                        it = 0; 
                        if (out_sg != NULL) { 
                                dst_addr = kmap(sg_page(out_sg)); 
                                if (dst_addr == NULL) { 
                                        dev_err(ss_ctx->dev, "KMAP dst error\n"); 
                                        goto end; 
                                } 
                        } 
                } 
        } while (out_sg != NULL); 
 
end: 
        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
        mutex_unlock(&lock); 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* Pure CPU way of doing AES with SS 
 * All my tries to read/write directly SG fails, so this is the reasons of the 
 * use of two in/out buffer 
 * SGsrc -> buf_in -> SS -> buf_out -> SGdst */ 
static int sunxi_aes_poll(struct ablkcipher_request *areq, int flag) 
{ 
        u32 tmp, value; 
        size_t nb_in_sg_tx, nb_in_sg_rx; 
        size_t ir, it; 
        struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq); 
        struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm); 
        u32 tx_cnt = 0; 
        u32 rx_cnt = 0; 
 
        tmp = 0; 
        tmp |= SUNXI_SS_CBC; 
        tmp |= SUNXI_OP_AES; 
        tmp |= SUNXI_KEYSELECT_KEYN; 
        tmp |= SUNXI_SS_ENABLED; 
        tmp |= flag; 
        switch (op->keylen) { 
        case 128 / 8: 
                tmp |= SUNXI_AES_128BITS; 
                break; 
        case 192 / 8: 
                tmp |= SUNXI_AES_192BITS; 
                break; 
        case 256 / 8: 
                tmp |= SUNXI_AES_256BITS; 
                break; 
        } 
        iowrite32(tmp, ss_ctx->base + SUNXI_SS_CTL); 
 
        nb_in_sg_rx = sg_count(areq->src, areq->nbytes); 
        nb_in_sg_tx = sg_count(areq->dst, areq->nbytes); 
 
        if (ss_ctx->buf_in == NULL) { 
                ss_ctx->buf_in = kmalloc(areq->nbytes, GFP_KERNEL); 
                ss_ctx->buf_in_size = areq->nbytes; 
        } else { 
                if (areq->nbytes > ss_ctx->buf_in_size) { 
                        kfree(ss_ctx->buf_in); 
                        ss_ctx->buf_in = kmalloc(areq->nbytes, GFP_KERNEL); 
                        ss_ctx->buf_in_size = areq->nbytes; 
                } 
        } 
        if (ss_ctx->buf_in == NULL) { 
                dev_err(ss_ctx->dev, "Unable to allocate pages.\n"); 
                iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
                mutex_unlock(&lock); 
                return -ENOMEM; 
        } 
        mutex_lock(&bufout_lock); 
        if (ss_ctx->buf_out == NULL) { 
                ss_ctx->buf_out = kmalloc(areq->nbytes, GFP_KERNEL); 
                ss_ctx->buf_out_size = areq->nbytes; 
        } else { 
                if (areq->nbytes > ss_ctx->buf_out_size) { 
                        kfree(ss_ctx->buf_out); 
                        ss_ctx->buf_out = kmalloc(areq->nbytes, GFP_KERNEL); 
                        ss_ctx->buf_out_size = areq->nbytes; 
                } 
        } 
        mutex_unlock(&bufout_lock); 
        if (ss_ctx->buf_in == NULL || ss_ctx->buf_out == NULL) { 
                dev_err(ss_ctx->dev, "Unable to allocate pages.\n"); 
                iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
                mutex_unlock(&lock); 
                return -ENOMEM; 
        } 
 
        sg_copy_to_buffer(areq->src, nb_in_sg_rx, ss_ctx->buf_in, areq->nbytes); 
 
        ir = 0; 
        it = 0; 
 
        do { 
                if (rx_cnt == 0 || tx_cnt == 0) { 
                        tmp = ioread32(ss_ctx->base + SUNXI_SS_FCSR); 
                        rx_cnt = SUNXI_RXFIFO_SPACES(tmp); 
                        tx_cnt = SUNXI_TXFIFO_SPACES(tmp); 
                } 
                if (rx_cnt > 0 && ir < areq->nbytes) { 
                        do { 
                                value = *(u32 *)(ss_ctx->buf_in + ir); 
                                iowrite32(value, ss_ctx->base + SUNXI_SS_RXFIFO); 
                                ir += 4; 
                                rx_cnt--; 
                        } while (rx_cnt > 0 && ir < areq->nbytes); 
                } 
                if (tx_cnt > 0 && it < areq->nbytes) { 
                        do { 
                                if (ir <= it) 
                                        dev_info(ss_ctx->dev, "DEBUG ANORMAL %u %u\n", ir, it); 
                                value = ioread32(ss_ctx->base + SUNXI_SS_TXFIFO); 
                                *(u32 *)(ss_ctx->buf_out + it) = value; 
                                it += 4; 
                                tx_cnt--; 
                        } while (tx_cnt > 0 && it < areq->nbytes); 
                } 
        } while (it < areq->nbytes); 
 
        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
        mutex_unlock(&lock); 
 
        /* a simple optimization, since we dont need the hardware for this copy 
         * we release the lock and do the copy. With that we gain 5/10% perf 
         * TODO: do the same thing for buf_in */ 
        mutex_lock(&bufout_lock); 
        sg_copy_from_buffer(areq->dst, nb_in_sg_tx, ss_ctx->buf_out, 
                        areq->nbytes); 
 
        mutex_unlock(&bufout_lock); 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* Do the AES with DMA 
 * Sgsrc -DMARX-> SS -DMATX-> SGdst */ 
static int sunxi_aes_dma(struct ablkcipher_request *areq, int flag) 
{ 
        size_t nb_in_sg_tx, nb_in_sg_rx; 
        size_t i; 
        int txcount, rxcount, ret; 
        u32 v, tmp; 
        int antibug = 0; 
        unsigned int tx_len = 0, rx_len = 0, len; 
        struct scatterlist *sgt, *sgr; 
        int rtm = 0, ttm = 0; 
        int nbsgr = 0, nbsgt = 0; 
        dma_addr_t pos_src, pos_dst; 
        struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq); 
        struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm); 
        char trig_rx, trig_tx; 
 
#ifdef DEBUG_SS_DMA 
        dev_info(ss_ctx->dev, "%s %d bytes=%d\n", __func__, aes_mode, areq->nbytes); 
#endif 
        tmp = 0; 
        tmp |= SUNXI_SS_CBC; 
        tmp |= SUNXI_OP_AES; 
        tmp |= SUNXI_KEYSELECT_KEYN; 
        tmp |= SUNXI_SS_ENABLED; 
        tmp |= flag; 
        switch (op->keylen) { 
        case 128 / 8: 
                tmp |= SUNXI_AES_128BITS; 
                break; 
        case 192 / 8: 
                tmp |= SUNXI_AES_192BITS; 
                break; 
        case 256 / 8: 
                tmp |= SUNXI_AES_256BITS; 
                break; 
        } 
        iowrite32(tmp, ss_ctx->base + SUNXI_SS_CTL); 
 
        /* enable DRQ */ 
        iowrite32(SUNXI_SS_ICS_DRA_ENABLE, ss_ctx->base + SUNXI_SS_ICSR); 
 
        if (ss_ctx->rxdma_init == 0) 
                ss_sunxi_prepare_dma(DMA_TO_DEVICE); 
        if (ss_ctx->txdma_init == 0) 
                ss_sunxi_prepare_dma(DMA_FROM_DEVICE); 
        if (ss_ctx->txdma_init == 0 || ss_ctx->rxdma_init == 0) { 
                dev_err(ss_ctx->dev, "DMA init error\n"); 
                iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
                mutex_unlock(&lock); 
                return -1; 
        } 
 
        nb_in_sg_rx = sg_count(areq->src, areq->nbytes); 
        nb_in_sg_tx = sg_count(areq->dst, areq->nbytes); 
 
        if (areq->src == areq->dst) { 
#ifdef DEBUG_SS_DMA 
                dev_info(ss_ctx->dev, "Need bidirectionnal DMA\n"); 
#endif 
                rxcount = dma_map_sg(ss_ctx->dev, areq->src, nb_in_sg_rx, 
                                DMA_BIDIRECTIONAL); 
                txcount = rxcount; 
                if (rxcount < 0) { 
                        dev_err(ss_ctx->dev, "dma_map_sg of AES src error\n"); 
                        iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
                        mutex_unlock(&lock); 
                        return -1; 
                } 
        } else { 
                rxcount = dma_map_sg(ss_ctx->dev, areq->src, nb_in_sg_rx, 
                                DMA_TO_DEVICE); 
                if (rxcount < 0) { 
                        dev_err(ss_ctx->dev, "dma_map_sg of AES src error\n"); 
                        iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
                        mutex_unlock(&lock); 
                        return -1; 
                } 
                txcount = dma_map_sg(ss_ctx->dev, areq->dst, nb_in_sg_tx, 
                                DMA_FROM_DEVICE); 
                if (txcount < 0) { 
                        dev_err(ss_ctx->dev, "dma_map_sg of AES dst error\n"); 
                        iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
                        mutex_unlock(&lock); 
                        return -1; 
                } 
/*                dma_sync_sg_for_device(ss_ctx->dev, areq->src, nb_in_sg_rx, 
                                DMA_TO_DEVICE);*/ 
        } 
#ifdef DEBUG_SS_DMA 
        dev_info(ss_ctx->dev, "RX DMA cnt=%d nbsg=%u\n", rxcount, nb_in_sg_rx); 
        dev_info(ss_ctx->dev, "TX DMA cnt=%d nbsg=%u\n", txcount, nb_in_sg_tx); 
#endif 
 
/*#define DEBUG_DMA_P*/ 
        /* now we can start DMA 
         * We begin with the first SG of source and destination 
         * After each SG finished start the next SG 
         * */ 
 
        sgr = areq->src; 
        sgt = areq->dst; 
#ifdef DEBUG_DMA_P 
/*        for_each_sg(areq->src, sg, rxcount, i) { 
                rx_len += sg_dma_len(sg); 
                dev_info(ss_ctx->dev, "RX DMA hwaddr=%p\n", sg_dma_address(sg)); 
                dev_info(ss_ctx->dev, "RX DMA len=%d sum=%u\n", sg_dma_len(sg), 
                                rx_len); 
        } 
        for_each_sg(areq->dst, sg, txcount, i) { 
                tx_len += sg_dma_len(sg); 
                dev_info(ss_ctx->dev, "TX DMA hwaddr=%p\n", sg_dma_address(sg)); 
                dev_info(ss_ctx->dev, "TX DMA len=%d sum=%u\n", sg_dma_len(sg), 
                                tx_len); 
        } 
        dev_info(ss_ctx->dev, "SG lens nbytes=%u rx=%u tx=%u\n", areq->nbytes, 
                        rx_len, tx_len);*/ 
#endif 
 
        /* calculate trigger */ 
        sunxi_ss_trigger(sg_dma_len(sgr), sg_dma_len(sgt), &trig_rx, &trig_tx, 
                        CHANGE_TRIG_RX | CHANGE_TRIG_TX); 
 
        tx_len = 0; 
        rx_len = 0; 
        ret = ss_dma_recv(sg_dma_address(sgt), sg_dma_len(sgt)); 
        if (ret != 0) { 
                dev_err(ss_ctx->dev, "DMA ss_dma_recv error\n"); 
                goto ss_dma_end; 
        } 
        ret = ss_dma_send(sg_dma_address(sgr), sg_dma_len(sgr), 1); 
        if (ret != 0) { 
                dev_err(ss_ctx->dev, "DMA ss_dma_send error\n"); 
                goto ss_dma_end; 
        } 
        /* Now we have both DMA started, we wait for completion of any, advance to next SG and start another DMA 
         * while some SG are available. */ 
        do { 
                if (ss_ctx->txdma_start == 0) { 
                        ret = sunxi_dma_stop(&sstx_dma); 
                        if (ret != 0) 
                                dev_err(ss_ctx->dev, "TX DMA could not be stopped\n"); 
                        tx_len += sg_dma_len(sgt); 
#ifdef DEBUG_DMA_P 
                        dev_info(ss_ctx->dev, "Detected TX end, enqueue %d / %d done=%u\n", 
                                        nbsgt, nb_in_sg_tx, tx_len); 
#endif 
                        sgt = sg_next(sgt); 
                        nbsgt++; 
                        if (nbsgt < nb_in_sg_tx) { 
                                len = sg_dma_len(sgt); 
                                if (tx_len + len > areq->nbytes) { 
                                        len = areq->nbytes - tx_len; 
                                        dev_info(ss_ctx->dev, "DEBUG minimizing TX from %u to %u\n", sg_dma_len(sgt), len); 
                                } 
                                sunxi_ss_trigger(0, len, &trig_rx, &trig_tx, CHANGE_TRIG_TX); 
                                if (len <= 8) { 
#ifdef DEBUG_DMA_P 
                                        dev_info(ss_ctx->dev, "Evil hack at %d/%d len=%u\n", 
                                                        nbsgt, nb_in_sg_tx, len); 
#endif 
                                        iowrite32(0x301, ss_ctx->base + SUNXI_SS_FCSR); 
                                        trig_rx = 0x03; 
                                        trig_tx = 0x01; 
                                } 
                                ret = ss_dma_recv(sg_dma_address(sgt), len); 
                                if (ret != 0) { 
                                        dev_err(ss_ctx->dev, "DMA ss_dma_recv error\n"); 
                                        iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                                        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
                                        mutex_unlock(&lock); 
                                        return ret; 
                                } 
                                ttm = 0; 
                        } else { 
                                ss_ctx->txdma_start = -1; 
#ifdef DEBUG_DMA_P 
                                dev_info(ss_ctx->dev, "No more TX to do\n"); 
#endif 
                        } 
                } 
                if (ss_ctx->rxdma_start == 0) { 
                        ret = sunxi_dma_stop(&ssrx_dma); 
                        if (ret != 0) 
                                dev_err(ss_ctx->dev, 
                                                "RX DMA could not be stopped\n"); 
                        rx_len += sg_dma_len(sgr); 
#ifdef DEBUG_DMA_P 
                        dev_info(ss_ctx->dev, "Detected RX end, enqueue %d / %d done=%u\n", nbsgr, nb_in_sg_rx, rx_len); 
#endif 
                        sgr = sg_next(sgr); 
                        nbsgr++; 
                        if (nbsgr < nb_in_sg_rx) { 
#ifdef DEBUG_DMA_P 
                                dev_info(ss_ctx->dev, "More RX to do %p %u\n", sg_dma_address(sgr), sg_dma_len(sgr)); 
#endif 
                                len = sg_dma_len(sgr); 
                                if (rx_len + len > areq->nbytes) { 
                                        len = areq->nbytes - rx_len; 
/*                                        dev_info(ss_ctx->dev, "DEBUG minimizing from %u to %u\n", sg_dma_len(sgr), len);*/ 
                                } 
                                sunxi_ss_trigger(len, 0, &trig_rx, &trig_tx, CHANGE_TRIG_RX); 
                                ret = ss_dma_send(sg_dma_address(sgr), len, 1); 
                                if (ret != 0) { 
                                        dev_err(ss_ctx->dev, "DMA ss_dma_send error\n"); 
                                        iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
                                        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
                                        mutex_unlock(&lock); 
                                        return ret; 
                                } 
                                rtm = 0; 
                        } else { 
                                ss_ctx->rxdma_start = -1; 
#ifdef DEBUG_DMA_P 
                                dev_info(ss_ctx->dev, "No more RX to do\n"); 
#endif 
                        } 
                } 
                /* without waitqueue we use lots of CPU in this do while for nothing */ 
#ifdef SUNXI_SS_WAIT_QUEUE 
                ret = wait_event_interruptible_timeout(dma_queue, (ss_ctx->rxdma_start != 1 || ss_ctx->txdma_start != 1), HZ * 10); 
                /*dev_info(ss_ctx->dev, "wait_event_interruptible_timeout %d start=%d %d\n", ret, ss_ctx->rxdma_start, ss_ctx->txdma_start);*/ 
#endif 
/*                usleep_range(1, 2);*/ 
                rtm++; 
                ttm++; 
                /* the four if blocks that follow are here only to debug DMA problems */ 
                if (rtm > 1000 * 1000 && ss_ctx->rxdma_start == 1 && antibug == 0) { 
                        dev_info(ss_ctx->dev, "RX timeout at %d/%d rxlen=%u txlen=%u req=%u\n", nbsgr, nb_in_sg_rx, rx_len, tx_len, areq->nbytes); 
                        v = ioread32(ss_ctx->base + SUNXI_SS_FCSR); 
                        dev_info(ss_ctx->dev, "DEBUG SUNXI_SS_FCSR %x\n", v); 
                        v = ioread32(ss_ctx->base + SUNXI_SS_ICSR); 
                        dev_info(ss_ctx->dev, "DEBUG SUNXI_SS_ICSR %x\n", v); 
                        sunxi_dma_getcurposition(&ssrx_dma, &pos_src, &pos_dst); 
                        dev_info(ss_ctx->dev, "Curpos %x %x start=%x\n", pos_src, pos_dst, sg_dma_address(sgr)); 
                        iowrite32(0x101, ss_ctx->base + SUNXI_SS_FCSR); 
                        trig_rx = 0x01; 
                        trig_tx = 0x01; 
                        antibug = 1; 
                        rtm = 0; 
                        dev_info(ss_ctx->dev, "DEBUG try antibug RX\n"); 
                } 
                if (rtm > 1000 * 1000 && ss_ctx->rxdma_start == 1) { 
                        dev_info(ss_ctx->dev, "RX timeout at %d/%d rxlen=%u txlen=%u req=%u\n", nbsgr, nb_in_sg_rx, rx_len, tx_len, areq->nbytes); 
                        nbsgr = nb_in_sg_rx; 
                        nbsgt = nb_in_sg_tx; 
                        ret = sunxi_dma_stop(&ssrx_dma); 
                        if (ret != 0) 
                                dev_err(ss_ctx->dev, "RX DMA could not be stopped\n"); 
                        ret = sunxi_dma_stop(&sstx_dma); 
                        if (ret != 0) 
                                dev_err(ss_ctx->dev, "TX DMA could not be stopped\n"); 
                        break; 
                } 
                if (ttm > 1000 * 1000 * 10 && ss_ctx->txdma_start == 1 && antibug == 0) { 
                        dev_info(ss_ctx->dev, "TX timeout at %d/%d rxlen=%u txlen=%u req=%u\n", nbsgt, nb_in_sg_tx, rx_len, tx_len, areq->nbytes); 
                        v = ioread32(ss_ctx->base + SUNXI_SS_FCSR); 
                        dev_info(ss_ctx->dev, "DEBUG SUNXI_SS_FCSR %x\n", v); 
                        v = ioread32(ss_ctx->base + SUNXI_SS_ICSR); 
                        dev_info(ss_ctx->dev, "DEBUG SUNXI_SS_ICSR %x\n", v); 
                        sunxi_dma_getcurposition(&sstx_dma, &pos_src, &pos_dst); 
                        dev_info(ss_ctx->dev, "Curpos %x %x start=%x %x diff=%x\n", pos_src, 
                                        pos_dst, sg_dma_address(sgt), 
                                        ss_ctx->base + SUNXI_SS_TXFIFO, 
                                        pos_dst - sg_dma_address(sgt)); 
                        iowrite32(0x101, ss_ctx->base + SUNXI_SS_FCSR); 
                        trig_rx = 0x01; 
                        trig_tx = 0x01; 
                        antibug = 1; 
                        ttm = 0; 
                        dev_info(ss_ctx->dev, "DEBUG try antibug TX\n"); 
                } 
                if (ttm > 1000 * 1000 * 10 && ss_ctx->txdma_start == 1) { 
                        dev_info(ss_ctx->dev, "TX timeout at %d/%d rxlen=%u txlen=%u req=%u\n", nbsgt, nb_in_sg_tx, rx_len, tx_len, areq->nbytes); 
                        v = ioread32(ss_ctx->base + SUNXI_SS_FCSR); 
                        dev_info(ss_ctx->dev, "DEBUG SUNXI_SS_FCSR %x\n", v); 
                        v = ioread32(ss_ctx->base + SUNXI_SS_ICSR); 
                        dev_info(ss_ctx->dev, "DEBUG SUNXI_SS_ICSR %x\n", v); 
 
                        for (i = 0; i < 5; i++) { 
                                sunxi_dma_getcurposition(&sstx_dma, &pos_src, &pos_dst); 
                                dev_info(ss_ctx->dev, "Curpos %d %x %x start=%x %x diff=%x\n", 
                                                i, pos_src, pos_dst, 
                                                sg_dma_address(sgt), 
                                                ss_ctx->base + SUNXI_SS_TXFIFO, 
                                                pos_dst - sg_dma_address(sgt)); 
                                usleep_range(10, 15); 
                        } 
 
                        nbsgr = nb_in_sg_rx; 
                        nbsgt = nb_in_sg_tx; 
                        ret = sunxi_dma_stop(&ssrx_dma); 
                        if (ret != 0) 
                                dev_err(ss_ctx->dev, "RX DMA could not be stopped\n"); 
                        ret = sunxi_dma_stop(&sstx_dma); 
                        if (ret != 0) 
                                dev_err(ss_ctx->dev, "TX DMA could not be stopped\n"); 
                        break; 
                } 
        } while (nbsgr < nb_in_sg_rx || nbsgt < nb_in_sg_tx); 
 
        ret = 0; 
ss_dma_end: 
        if (areq->src == areq->dst) { 
                dma_sync_sg_for_cpu(ss_ctx->dev, areq->src, nb_in_sg_rx, 
                                DMA_BIDIRECTIONAL); 
                dma_unmap_sg(ss_ctx->dev, areq->src, nb_in_sg_rx, 
                                DMA_BIDIRECTIONAL); 
        } else { 
                dma_sync_sg_for_cpu(ss_ctx->dev, areq->dst, nb_in_sg_tx, 
                                DMA_FROM_DEVICE); 
                dma_unmap_sg(ss_ctx->dev, areq->src, nb_in_sg_rx, 
                                DMA_TO_DEVICE); 
                dma_unmap_sg(ss_ctx->dev, areq->dst, nb_in_sg_tx, 
                                DMA_FROM_DEVICE); 
        } 
        iowrite32(0, ss_ctx->base + SUNXI_SS_ICSR); 
        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
        mutex_unlock(&lock); 
        return ret; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static int sunxi_aes_cbc_encrypt(struct ablkcipher_request *areq) 
{ 
        u32 v; 
        int i; 
        struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq); 
        struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm); 
        unsigned int ivsize = crypto_ablkcipher_ivsize(tfm); 
 
        BUG_ON(ivsize && !areq->info); 
 
        if (ivsize < 4) 
                dev_info(ss_ctx->dev, "DEBUG IV size %d\n", ivsize); 
 
        mutex_lock(&lock); 
 
        ss_ctx->method = SUNXI_SS_ENCRYPTION; 
 
        if (areq->info == NULL) 
                dev_info(ss_ctx->dev, "DEBUG IV null\n"); 
 
        /* if the function becomes used by other method than CBC, test it here */ 
        if (areq->info != NULL) { 
                for (i = 0; i < op->keylen; i += 4) { 
                        v = *(u32 *)(op->key + i); 
                        iowrite32(v, ss_ctx->base + SUNXI_SS_KEY0 + i); 
                } 
                for (i = 0; i < 4; i++) { 
                        v = *(u32 *)(areq->info + i * 4); 
                        iowrite32(v, ss_ctx->base + SUNXI_SS_IV0 + i * 4); 
                } 
        } 
        op->mode |= SUNXI_SS_ENCRYPTION; 
 
        /* DMA */ 
        if (USE_DMA(areq->nbytes)) 
                return sunxi_aes_dma(areq, SUNXI_SS_ENCRYPTION); 
        else 
                return sunxi_aes_poll(areq, SUNXI_SS_ENCRYPTION); 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static int sunxi_aes_cbc_decrypt(struct ablkcipher_request *areq) 
{ 
        u32 v; 
        int i; 
        struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(areq); 
        struct sunxi_req_ctx *op = crypto_ablkcipher_ctx(tfm); 
        unsigned int ivsize = crypto_ablkcipher_ivsize(tfm); 
 
        BUG_ON(ivsize && !areq->info); 
 
        if (ivsize < 4) 
                dev_info(ss_ctx->dev, "DEBUG IV size %d\n", ivsize); 
 
        mutex_lock(&lock); 
        ss_ctx->method = SUNXI_SS_DECRYPTION; 
 
/*#ifdef DEBUG_DMA_P*/ 
/*        dev_info(ss_ctx->dev, "%s %d %d %p %x\n", __func__, areq->nbytes, op->keylen, op, op->key[0]);*/ 
/*#endif*/ 
        if (areq->info == NULL) 
                dev_info(ss_ctx->dev, "DEBUG IV null\n"); 
 
        if (areq->info != NULL) {/* TODO add test CBC */ 
                for (i = 0; i < op->keylen; i += 4) { 
                        v = *(u32 *)(op->key + i); 
                        iowrite32(v, ss_ctx->base + SUNXI_SS_KEY0 + i); 
                } 
                for (i = 0; i < 4; i++) { 
                        v = *(u32 *)(areq->info + i * 4); 
                        iowrite32(v, ss_ctx->base + SUNXI_SS_IV0 + i * 4); 
                } 
        } 
        op->mode |= SUNXI_SS_DECRYPTION; 
 
        if (USE_DMA(areq->nbytes)) 
                return sunxi_aes_dma(areq, SUNXI_SS_DECRYPTION); 
        else 
                return sunxi_aes_poll(areq, SUNXI_SS_DECRYPTION); 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static int sunxi_aes_init(struct crypto_tfm *tfm) 
{ 
        struct sunxi_req_ctx *op = crypto_tfm_ctx(tfm); 
        memset(op, 0, sizeof(struct sunxi_req_ctx)); 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static void sunxi_aes_exit(struct crypto_tfm *tfm) 
{ 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static struct crypto_alg sunxi_aes_alg = { 
        .cra_name = "cbc(aes)", 
        .cra_driver_name = "sunxi-cbc-aes", 
        .cra_priority = 100, 
        .cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER, 
        .cra_blocksize   = AES_BLOCK_SIZE, 
        .cra_ctxsize = sizeof(struct sunxi_req_ctx), 
        .cra_module = THIS_MODULE, 
        .cra_type = &crypto_ablkcipher_type, 
        .cra_init = sunxi_aes_init, 
        .cra_exit = sunxi_aes_exit, 
        .cra_u.ablkcipher = { 
                .min_keysize    = AES_MIN_KEY_SIZE, 
                .max_keysize    = AES_MAX_KEY_SIZE, 
                .ivsize         = AES_BLOCK_SIZE, 
                .setkey         = sunxi_aes_setkey, 
                .encrypt        = sunxi_aes_cbc_encrypt, 
                .decrypt        = sunxi_aes_cbc_decrypt, 
        } 
}; 
 
#endif /* CONFIG_CRYPTO_DEV_SUNXI_SS_AES */ 
 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_PRNG 
/*============================================================================*/ 
/*============================================================================*/ 
static int sunxi_ss_rng_get_random(struct crypto_rng *tfm, u8 *rdata, 
                unsigned int dlen) 
{ 
        struct prng_context *ctx = crypto_tfm_ctx((struct crypto_tfm *) tfm); 
        unsigned int i; 
        u32 mode = 0; 
        u32 v; 
 
        if (dlen == 0 || rdata == NULL) 
                return 0; 
 
        mode |= SUNXI_OP_PRNG; 
        mode |= SUNXI_PRNG_ONESHOT; 
        mode |= SUNXI_SS_ENABLED; 
 
        dev_info(ss_ctx->dev, "DEBUG %s dlen=%u\n", __func__, dlen); 
 
        mutex_lock(&lock); 
        iowrite32(mode, ss_ctx->base + SUNXI_SS_CTL); 
 
        for (i = 0; i < ctx->slen; i += 4) { 
                v = *(u32 *)(ctx->seed + i); 
                dev_info(ss_ctx->dev, "DEBUG Seed%d %x\n", i, v); 
        } 
 
        for (i = 0; i < ctx->slen && i < 192/8 && i < 16; i += 4) { 
                v = *(u32 *)(ctx->seed + i); 
                dev_info(ss_ctx->dev, "DEBUG Seed%d %x\n", i, v); 
                iowrite32(v, ss_ctx->base + SUNXI_SS_KEY0 + i); 
        } 
 
        mode |= SUNXI_PRNG_START; 
        iowrite32(mode, ss_ctx->base + SUNXI_SS_CTL); 
        for (i = 0; i < 4; i++) { 
                v = ioread32(ss_ctx->base + SUNXI_SS_CTL); 
                dev_info(ss_ctx->dev, "DEBUG CTL %x %x\n", mode, v); 
        } 
        for (i = 0; i < dlen && i < 160 / 8; i += 4) { 
                v = ioread32(ss_ctx->base + SUNXI_SS_MD0 + i); 
                *(u32 *)(rdata + i) = v; 
                dev_info(ss_ctx->dev, "DEBUG MD%d %x\n", i, v); 
        } 
 
        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
        mutex_unlock(&lock); 
        return dlen; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static int sunxi_ss_rng_reset(struct crypto_rng *tfm, u8 *seed, 
                unsigned int slen) 
{ 
        struct prng_context *ctx = crypto_tfm_ctx((struct crypto_tfm *)tfm); 
 
        dev_info(ss_ctx->dev, "DEBUG %s slen=%u\n", __func__, slen); 
        mutex_lock(&lock); 
        memcpy(ctx->seed, seed, slen); 
        ctx->slen = slen; 
        mutex_unlock(&lock); 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
/* TODO perhaps we could implement ansi_cprng ? */ 
static struct crypto_alg sunxi_ss_prng = { 
/*        .cra_name = "ansi_cprng",*/ 
        .cra_name = "stdrng", 
        .cra_driver_name = "sunxi-ss-rng", 
        .cra_priority = 100, 
        .cra_flags = CRYPTO_ALG_TYPE_RNG, 
        .cra_ctxsize = sizeof(struct prng_context), 
        .cra_module = THIS_MODULE, 
        .cra_type = &crypto_rng_type, 
        .cra_u.rng = { 
                .rng_make_random = sunxi_ss_rng_get_random, 
                .rng_reset = sunxi_ss_rng_reset, 
                .seedsize = 192/8 
        } 
}; 
#endif /* CRYPTO_DEV_SUNXI_SS_PRNG */ 
 
 
 
/* 
static irqreturn_t ss_handle_irq(int irq, void *dev_id) 
{ 
        u32 status; 
        status = ioread32(ss_ctx->base + SUNXI_SS_ICSR); 
        dev_info(ss_ctx->dev, "%s %x\n", __func__, status); 
*/ 
/*        status &= ~SUNXI_RXFIFO_EMP_INT_PENDING; 
        status &= ~SUNXI_TXFIFO_AVA_INT_PENDING;*//* 
        iowrite32(status, ss_ctx->base + SUNXI_SS_ICSR); 
        return IRQ_HANDLED; 
} 
*/ 
 
/*============================================================================*/ 
/*============================================================================*/ 
static int __init sunxi_ss_probe(struct platform_device *pdev) 
{ 
        struct resource *res; 
        /*u32 v;*/ 
        int err; 
        unsigned long cr; 
        struct clk *parent; 
 
        memset(ss_ctx, 0, sizeof(struct sunxi_ss_ctx)); 
 
        /* Map SS memory */ 
        res = platform_get_resource(pdev, IORESOURCE_MEM, 0); 
        if (res == NULL) { 
                dev_err(&pdev->dev, "Cannot get the IO MEMORY\n"); 
                return -ENXIO; 
        } 
        ss_ctx->res = request_mem_region(res->start, resource_size(res), 
                        res->name); 
        if (!ss_ctx->res) { 
                dev_err(&pdev->dev, "Cannot request mmio\n"); 
                return -ENXIO; 
        } 
        /* TODO nocache ? */ 
        ss_ctx->base = ioremap_nocache(res->start, resource_size(res)); 
        /* TODO mainline use devm_ioremap_resource(&pdev->dev, res); */ 
        if (IS_ERR(ss_ctx->base)) { 
                err = -ENOMEM; 
                goto label_error_mem; 
        } 
 
        /* We dont use the IRQ, so dont claim it for the moment */ 
/*        ss_ctx->irq = platform_get_irq(pdev, 0); 
        err = request_irq(ss_ctx->irq, ss_handle_irq, 0, pdev->name, pdev); 
        if (err != 0) { 
                dev_err(&pdev->dev, "Cannot get IRQ\n"); 
                return err; 
        }*/ 
 
        /* TODO Does this information could be usefull ? */ 
/*        v = ioread32(ss_ctx->base + SUNXI_SS_CTL); 
        v >>= 16; 
        v &= 0x07; 
        dev_info(&pdev->dev, "DIE ID %d\n", v);*/ 
 
        /* clock help https://www.kernel.org/doc/htmldocs/kernel-api/ */ 
        /* Acquire SS bus clk */ 
        ss_ctx->busclk = clk_get(&pdev->dev, "ahb_ss"); 
        if (IS_ERR(ss_ctx->busclk)) { 
                err = -ENODEV; 
                goto label_error_clock; 
        } 
        /*dev_info(&pdev->dev, "clock ahb_ss acquired\n");*/ 
 
        /* Acquire SS clk */ 
        ss_ctx->ssclk = clk_get(&pdev->dev, "ss"); 
        if (IS_ERR(ss_ctx->ssclk)) { 
                err = -ENODEV; 
                goto label_error_clock; 
        } 
        /*dev_info(&pdev->dev, "clock ss acquired\n");*/ 
 
        /* Enable the clocks */ 
        err = clk_prepare_enable(ss_ctx->ssclk); 
        if (err != 0) { 
                dev_err(&pdev->dev, "Cannot prepare_enable ssclk\n"); 
                goto label_error_clock; 
        } 
        err = clk_prepare_enable(ss_ctx->busclk); 
        if (err != 0) { 
                dev_err(&pdev->dev, "Cannot prepare_enable busclk\n"); 
                goto label_error_clock; 
        } 
 
#define        SUNXI_SS_CLOCK_RATE_BUS (24 * 1000 * 1000) 
#define        SUNXI_SS_CLOCK_RATE_SS (150 * 1000 * 1000) 
 
        parent = clk_get(NULL, "sata_pll"); 
        if (IS_ERR(parent)) { 
                dev_err(&pdev->dev, "Cannot get parent\n"); 
                err = -ENODEV; 
                goto label_error_clock; 
        } else { 
                cr = clk_get_rate(parent); 
                dev_info(&pdev->dev, "CLOCK parent %lu (%lu MHz) need >= %u\n", 
                                cr, cr / 1000000, SUNXI_SS_CLOCK_RATE_BUS); 
                clk_set_parent(ss_ctx->ssclk, parent); 
        } 
/* TODO for the moment I cannot made busclk to be 24MHz*/ 
/* 
        struct clk *p2; 
        p2 = clk_get(NULL, "ahb"); 
        if (IS_ERR(p2)) { 
                dev_err(&pdev->dev, "Cannot get parent2\n"); 
        } else { 
                cr = clk_get_rate(p2); 
                dev_info(&pdev->dev, 
                        "CLOCK parent %lu (%lu MHz) (must be >= 24000000)\n", 
                                cr, cr / 1000000); 
                clk_set_parent(ss_ctx->busclk, p2); 
        } 
        err = clk_set_rate(ss_ctx->busclk, SUNXI_SS_CLOCK_RATE_BUS); 
        if (err != 0) { 
                dev_err(&pdev->dev, "Cannot set clock rate to busclk\n"); 
                goto label_error_clock; 
        }*/ 
        err = clk_set_rate(ss_ctx->ssclk, SUNXI_SS_CLOCK_RATE_SS); 
        if (err != 0) { 
                dev_err(&pdev->dev, "Cannot set clock rate to ssclk\n"); 
                goto label_error_clock; 
        } 
 
        /* Check that clock have the correct rates */ 
        cr = clk_get_rate(ss_ctx->busclk); 
        dev_info(&pdev->dev, "CLOCK bus %lu (%lu MHz) (must be >= %u)\n", 
                        cr, cr / 1000000, SUNXI_SS_CLOCK_RATE_BUS); 
        cr = clk_get_rate(ss_ctx->ssclk); 
        dev_info(&pdev->dev, "CLOCK ss %lu (%lu MHz) (must be <= %u)\n", 
                        cr, cr / 1000000, SUNXI_SS_CLOCK_RATE_SS); 
 
        ss_ctx->buf_in = NULL; 
        ss_ctx->buf_in_size = 0; 
        ss_ctx->buf_out = NULL; 
        ss_ctx->buf_out_size = 0; 
        ss_ctx->rxdma_init = 0; 
        ss_ctx->txdma_init = 0; 
        ss_ctx->rxdma_start = 0; 
        ss_ctx->txdma_start = 0; 
        ss_ctx->dev = &pdev->dev; 
 
/*        TODO does I need this ?*/ 
/*        mutex_init(&lock); */ 
 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_PRNG 
        err = crypto_register_alg(&sunxi_ss_prng); 
        if (err) { 
                dev_err(&pdev->dev, "crypto_register_alg error\n"); 
                goto label_error_prng; 
        } else 
                dev_info(&pdev->dev, "Registred PRNG\n"); 
#endif 
 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_MD5 
        err = crypto_register_ahash(&sunxi_md5_alg); 
        if (err) { 
                dev_err(&pdev->dev, "Fail to register MD5\n"); 
                goto label_error_md5; 
        } else 
                dev_info(&pdev->dev, "Registred MD5\n"); 
#endif 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_SHA1 
        err = crypto_register_ahash(&sunxi_sha1_alg); 
        if (err) { 
                dev_err(&pdev->dev, "Fail to register SHA1\n"); 
                goto label_error_sha1; 
        } else 
                dev_info(&pdev->dev, "Registred SHA1\n"); 
#endif 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_AES 
        err = crypto_register_alg(&sunxi_aes_alg); 
        if (err) { 
                dev_err(&pdev->dev, "crypto_register_alg error for AES\n"); 
                goto label_error_aes; 
        } else 
                dev_info(&pdev->dev, "Registred AES\n"); 
#endif 
 
#ifdef SUNXI_SS_WAIT_QUEUE 
        dev_info(&pdev->dev, "Options: waitqueue\n"); 
#endif 
        if (use_dma > 0) 
                dev_info(&pdev->dev, "Options: DMA config RX %x TX %x\n", DMA_MAGIC_RX, DMA_MAGIC_TX); 
        if (use_dma == 0) 
                dev_info(&pdev->dev, "Options: poll mode\n"); 
        if (use_dma == 1) 
                dev_info(&pdev->dev, "Options: DMA mode\n"); 
        if (use_dma == 2) 
                dev_info(&pdev->dev, "Options: Optimal mode\n"); 
        return 0; 
 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_AES 
label_error_aes: 
        crypto_unregister_alg(&sunxi_aes_alg); 
#endif 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_SHA1 
label_error_sha1: 
        crypto_unregister_ahash(&sunxi_sha1_alg); 
#endif 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_MD5 
label_error_md5: 
        crypto_unregister_ahash(&sunxi_md5_alg); 
#endif 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_PRNG 
label_error_prng: 
        crypto_unregister_alg(&sunxi_ss_prng); 
#endif 
label_error_clock: 
        if (ss_ctx->ssclk != NULL) { 
                clk_disable_unprepare(ss_ctx->ssclk); 
                clk_put(ss_ctx->ssclk); 
        } 
        if (ss_ctx->busclk != NULL) { 
                clk_disable_unprepare(ss_ctx->busclk); 
                clk_put(ss_ctx->busclk); 
        } 
 
label_error_mem: 
        iounmap(ss_ctx->base); 
        release_mem_region(ss_ctx->res->start, resource_size(ss_ctx->res)); 
        return err; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static int __exit sunxi_ss_remove(struct platform_device *pdev) 
{ 
        dev_info(&pdev->dev, "%s\n", __func__); 
 
        crypto_unregister_ahash(&sunxi_md5_alg); 
        crypto_unregister_ahash(&sunxi_sha1_alg); 
#ifdef CONFIG_CRYPTO_DEV_SUNXI_SS_PRNG 
        crypto_unregister_alg(&sunxi_ss_prng); 
#endif 
        crypto_unregister_alg(&sunxi_aes_alg); 
        dev_info(&pdev->dev, "%s after crypto_unregister\n", __func__); 
 
        if (ss_ctx->buf_in != NULL) 
                kfree(ss_ctx->buf_in); 
                /*free_pages((unsigned long)ss_ctx->buf_in, 0);*/ 
        if (ss_ctx->buf_out != NULL) 
                kfree(ss_ctx->buf_out); 
        if (ss_ctx->rxdma_init == 1) 
                ss_sunxi_release_dma(&ssrx_dma); 
        if (ss_ctx->txdma_init == 1) 
                ss_sunxi_release_dma(&sstx_dma); 
 
        /* Disable SS */ 
        iowrite32(0, ss_ctx->base + SUNXI_SS_CTL); 
        /*free_irq(ss_ctx->irq, pdev);*/ 
        iounmap(ss_ctx->base); 
        release_mem_region(ss_ctx->res->start, resource_size(ss_ctx->res)); 
        clk_disable_unprepare(ss_ctx->busclk); 
        clk_disable_unprepare(ss_ctx->ssclk); 
        clk_put(ss_ctx->ssclk); 
        clk_put(ss_ctx->busclk); 
 
        return 0; 
} 
 
/*============================================================================*/ 
/*============================================================================*/ 
static void sunxi_ss_release(struct device *dev) 
{ 
        /* http://stackoverflow.com/questions/15532170/device-has-no-release-function-what-does-this-mean */ 
} 
 
static struct resource sunxi_ss_resource[] = { 
        { 
                .flags  = IORESOURCE_IRQ, 
                .start  = SUNXI_SS_IRQ, 
                .end    = SUNXI_SS_IRQ, 
        }, 
        { 
                .flags  = IORESOURCE_MEM, 
                .start  = SUNXI_SS_REG_BASE, 
                .end    = SUNXI_SS_REG_BASE + 0xfff, 
        }, 
}; 
 
struct platform_device sunxi_ss_device = { 
        .name           = "sunxi-ss", 
        .id                 = -1, 
        .resource        = sunxi_ss_resource, 
        .num_resources  = ARRAY_SIZE(sunxi_ss_resource), 
        .dev = { 
                .release = sunxi_ss_release, 
        }, 
}; 
 
static struct platform_driver sunxi_ss_driver = { 
        .probe          = sunxi_ss_probe, 
        .remove         = sunxi_ss_remove, 
        .driver         = { 
                .owner          = THIS_MODULE, 
                .name           = "sunxi-ss", 
        }, 
}; 
 
static int __init sunxi_ss_init(void) 
{ 
        platform_device_register(&sunxi_ss_device); 
        return platform_driver_register(&sunxi_ss_driver); 
} 
 
static void __exit sunxi_ss_exit(void) 
{ 
        platform_driver_unregister(&sunxi_ss_driver); 
        platform_device_unregister(&sunxi_ss_device); 
} 
 
module_init(sunxi_ss_init); 
module_exit(sunxi_ss_exit); 
 
MODULE_DESCRIPTION("Allwinner SUNXI Security System"); 
MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("Corentin LABBE <clabbe....@gmail.com>"); 
MODULE_PARM_DESC(use_dma, "Use DMA or not"); 
 
