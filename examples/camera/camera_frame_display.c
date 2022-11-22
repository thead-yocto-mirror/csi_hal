/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: ShenWuYi <shenwuyi.swy@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <syslog.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include <fcntl.h>

#include <csi_camera.h>
#include <stdlib.h>
#include "process_linker_types.h"
#include "process_linker.h"
#include "video_mem.h"


#define NUM_OF_BUFFERS  5

typedef struct _CsiPictureBuffer
{
  unsigned int bus_address;
  void *virtual_address;
  unsigned int size;
  int fd;
} CsiPictureBuffer;
typedef struct _CsiPlinkContext
{
    int useplink;
    int exitplink;
    CsiPictureBuffer sendbuffer[NUM_OF_BUFFERS];
    csi_frame_s  frame_buf[NUM_OF_BUFFERS];
    int sendid;
    int available_bufs;
    void *vmem;
    PlinkHandle plink;
    PlinkChannelID chnid;
    pthread_t  buf_release_thread;
    pthread_mutex_t mutex;
} CsiPlinkContext;
static int get_buffer_count(PlinkPacket *pkt)
{
    int ret = 0;
    for (int i = 0; i < pkt->num; i++)
    {
        PlinkDescHdr *hdr = (PlinkDescHdr *)(pkt->list[i]);
        if (hdr->type == PLINK_TYPE_MESSAGE)
        {
            int *data = (int *)(pkt->list[i] + DATA_HEADER_SIZE);
            if (*data == PLINK_EXIT_CODE)
            {
                ret |= 0x80000000; // set bit 31 to 1 to indicate 'exit'
            }
            else if (*data >= 0)
                ret++;
        }
    }

    return ret;
}

static void* camera_buf_release_process(CsiPlinkContext * plink_ctx)
{
	PlinkPacket pkt = {0};
    if(plink_ctx == NULL)
    {
        return NULL;
    }
    LOG_O("Process is runing.....\n");
     while(!plink_ctx->exitplink)
     {
        if (plink_ctx->plink != NULL) {
			if (PLINK_wait(plink_ctx->plink, plink_ctx->chnid, 1000) == PLINK_STATUS_OK)
			{
				if (PLINK_recv(plink_ctx->plink, plink_ctx->chnid, &pkt) != PLINK_STATUS_OK) {
					plink_ctx->exitplink = 1;
					break;
				}
				int count = get_buffer_count(&pkt);
				if (count >= 0)
                {
                    PlinkMsg *hdr;
	                int resp_id;
                    for(int i=0;i<count;i++)
                    {
                        hdr = (PlinkMsg *)(pkt.list[i]);
                        resp_id = hdr->msg;
                        if((resp_id>=NUM_OF_BUFFERS)) 
                        {
                            LOG_W("invalid resp_id:%d\n",resp_id);
                            continue;
                        }
                        if(plink_ctx->frame_buf[resp_id].img.dmabuf[0].fds!= 0)
                        {
                            csi_camera_put_frame(&plink_ctx->frame_buf[resp_id]);
                            LOG_D("release resp_id:%d, fd:%d\n",resp_id,plink_ctx->frame_buf[resp_id].img.dmabuf[0].fds);
                            plink_ctx->frame_buf[resp_id].img.dmabuf[0].fds = 0;
                        }

                    }
                    pthread_mutex_lock(&plink_ctx->mutex);
					plink_ctx->available_bufs += count;
                    pthread_mutex_unlock(&plink_ctx->mutex);
                }
				else {
					fprintf(stderr, "[SERVER] Exit!\n");
					plink_ctx->exitplink = 1;
					break;
				}
			}else
            {
                LOG_W("Plink Resp timeout\n");
            }
		}
     }
     LOG_O("Process is exit .....\n");
     return NULL;
}
static int allocate_sendbuffers(CsiPictureBuffer picbuffers[NUM_OF_BUFFERS], unsigned int size, void *vmem)
{
    unsigned int buffer_size = (size + 0xFFF) & ~0xFFF;
    VmemParams params;
    params.size = buffer_size;
    params.flags = VMEM_FLAG_CONTIGUOUS | VMEM_FLAG_4GB_ADDR;
    for (int i = 0; i < NUM_OF_BUFFERS; i++)
    {
        if (VMEM_allocate(vmem, &params) != 0) {
			return -1;
		}
        VMEM_mmap(vmem, &params);
        VMEM_export(vmem, &params);
        LOG_O("[SERVER] mmap %p from %x with size %d, dma-buf fd %d\n",
                params.vir_address, params.phy_address, params.size, params.fd);
        picbuffers[i].virtual_address = params.vir_address;
        picbuffers[i].bus_address = params.phy_address;
        picbuffers[i].size = buffer_size;
        picbuffers[i].fd = params.fd;
    }
}

void *vi_plink_create(csi_camera_channel_cfg_s *chn_cfg)
{
	int ret = 0;
    CsiPlinkContext *plink_ctx = NULL;
    char *env=NULL;
	if (chn_cfg == NULL) {
		LOG_E("%s failt to  get chn_cfg\n", __func__);
		return NULL;
	}

	plink_ctx = malloc(sizeof(CsiPlinkContext));
    if(!plink_ctx)
    {
        LOG_E("malloc faile\n");
        return NULL;
    }

	char *plinknamebase0 = "/tmp/plink.test";
	char *plinknamebase1 = "/tmp/plink_npu_rgb.test";
	char *plinknamebase2 = "/tmp/plink.test2";
	env = getenv("ISP_PLINK_NAME0");

	if (env != NULL) {
		plinknamebase0  = env;
	}

	env = getenv("ISP_PLINK_NAME1");
	if (env != NULL) {
		plinknamebase1  = env;
	}

	env = getenv("ISP_PLINK_NAME2");
	if (env != NULL) {
		plinknamebase2  = env;
	}
	char plinkname[32];
	if (chn_cfg->chn_id == CSI_CAMERA_CHANNEL_0) {
		sprintf(plinkname, "%s", plinknamebase0);
	} else if (chn_cfg->chn_id == CSI_CAMERA_CHANNEL_1) {
		sprintf(plinkname, "%s",  plinknamebase1);
	} else {
		sprintf(plinkname, "%s",  plinknamebase2);
	}
	fprintf(stderr, "%s, %d: ISP_PLINK_NAME = %s\n", __func__, __LINE__, plinkname);


    plink_ctx->exitplink = 0;
    fprintf(stderr, "%s, %d: Launching plink server...\n", __func__, __LINE__);
    memset(&plink_ctx->sendbuffer[0], 0, sizeof(plink_ctx->sendbuffer));
    if (VMEM_create(&plink_ctx->vmem) != VMEM_STATUS_OK)
    {
        fprintf(stderr, "Failed to create VMEM.");
        return NULL;
    }
    else {
        int framesize = 0;
        int stride = (chn_cfg->img_fmt.width + 127) & (~127); // align stride to 128
        switch (chn_cfg->img_fmt.pix_fmt) {
            case CSI_PIX_FMT_BGR:
            {
                framesize = stride * 304 * 3; //stride * chn_cfg->img_fmt.height * 3;
                if (allocate_sendbuffers(plink_ctx->sendbuffer, framesize, plink_ctx->vmem) != 0) {
					return NULL;
				}                
                // reset to black picture
                uint32_t lumasize = stride * chn_cfg->img_fmt.height;
                for (int i = 0; i < NUM_OF_BUFFERS; i++)
                    memset(plink_ctx->sendbuffer[i].virtual_address, 0, framesize);
                break;
            }

            case CSI_PIX_FMT_NV12:
            default:
            {
                framesize = stride * chn_cfg->img_fmt.height * 3 / 2;
                if (allocate_sendbuffers(plink_ctx->sendbuffer, framesize, plink_ctx->vmem) != 0) {
					return NULL;
				}
                // reset to black picture
                uint32_t lumasize = stride * chn_cfg->img_fmt.height;
                for (int i = 0; i < NUM_OF_BUFFERS; i++) {
                    CsiPictureBuffer *buf = &plink_ctx->sendbuffer[i];
                    memset(buf->virtual_address, 0, lumasize);
                    memset(buf->virtual_address + lumasize, 0x80, lumasize/2);
                }
                break;
            }
        }
    }

    PLINK_create(&plink_ctx->plink, plinkname, PLINK_MODE_SERVER);
    if (plink_ctx->plink) {
        PLINK_connect(plink_ctx->plink, &plink_ctx->chnid);
    }

    plink_ctx->sendid = 0;
    plink_ctx->available_bufs = NUM_OF_BUFFERS;
    memset(plink_ctx->frame_buf,0x0,sizeof(plink_ctx->frame_buf));
    pthread_mutex_init(&plink_ctx->mutex,NULL);
    pthread_create(&plink_ctx->buf_release_thread,NULL,(void *)camera_buf_release_process,plink_ctx);


	return plink_ctx;
}

void vi_plink_release(void * plink)
{
    CsiPlinkContext * plink_ctx = (CsiPlinkContext *)plink;
    if(plink_ctx)
    {
        plink_ctx->exitplink=1;
        pthread_join(plink_ctx->buf_release_thread,NULL);
        
        PLINK_close(plink_ctx->plink,plink_ctx->chnid);
        VMEM_destroy(plink_ctx->vmem);
    }
}


void display_camera_frame(void *plink, csi_frame_s *frame)
{
	
    CsiPlinkContext * plink_ctx = (CsiPlinkContext *)plink;
	if ((plink_ctx == NULL) ||  (frame == NULL)) {
		LOG_E("%s  check param fail\n", __func__);
        return;
	}


	LOG_O("fmt=%d img.strides[0] = %d\n",frame->img.pix_format, frame->img.strides[0]);

	if (!plink_ctx->exitplink) {
		struct timeval tv_start, tv_end, tv_duration;
		gettimeofday(&tv_start, 0);
		PlinkPacket pkt = {0};
		// send one buffer if there is available slot
        if (frame->img.type == CSI_IMG_TYPE_DMA_BUF && !plink_ctx->exitplink
            && plink_ctx->available_bufs > 0 && plink_ctx->plink != NULL && ((frame->img.pix_format == CSI_PIX_FMT_RAW_8BIT)|| (frame->img.pix_format == CSI_PIX_FMT_RAW_12BIT))) {
            char str[200];
            static int i = 0;
			uint32_t dstw = frame->img.width;//800;
			uint32_t dsth = frame->img.height;//1280;
			uint32_t dsts = frame->img.strides[0];//896;

			PlinkRawInfo info = {0};

			info.header.type = PLINK_TYPE_2D_RAW;
			info.header.size = DATA_SIZE(info);
			info.header.id = plink_ctx->sendid;

			info.format = PLINK_COLOR_FormatRawBayer10bit;
			// info.bus_address = vi_mem_import(frame->img.dmabuf[0].fds) + frame->img.dmabuf[0].offset;
            // vi_mem_release(info.bus_address);

			info.img_width = dstw;
			info.img_height = dsth;
			info.stride = frame->img.strides[0];
			info.offset = 0;
			pkt.list[0] = &info;
			pkt.num = 1;
			pkt.fd = frame->img.dmabuf[0].fds;
            if(plink_ctx->frame_buf[plink_ctx->sendid].img.dmabuf[0].fds!=0)
            {
                LOG_W("previous sendid :%d,fd %d not release,release it now\n",plink_ctx->sendid,plink_ctx->frame_buf[plink_ctx->sendid].img.dmabuf[0].fds);
                csi_camera_put_frame(&plink_ctx->frame_buf[plink_ctx->sendid]);
            }
            memcpy(&plink_ctx->frame_buf[plink_ctx->sendid],frame,sizeof(csi_frame_s));
            plink_ctx->sendid = (plink_ctx->sendid + 1) % NUM_OF_BUFFERS;
			if (PLINK_send(plink_ctx->plink, plink_ctx->chnid, &pkt) != PLINK_STATUS_OK)
				plink_ctx->exitplink = 1;
			gettimeofday(&tv_end, 0);
			timersub(&tv_end, &tv_start, &tv_duration);
            pthread_mutex_lock(&plink_ctx->mutex);
			plink_ctx->available_bufs -= 1;
            pthread_mutex_unlock(&plink_ctx->mutex);
		}
        else if (frame->img.type == CSI_IMG_TYPE_DMA_BUF && !plink_ctx->exitplink
            && plink_ctx->available_bufs > 0 && plink_ctx->plink != NULL && ((frame->img.pix_format == CSI_PIX_FMT_YUV_SEMIPLANAR_420)|| (frame->img.pix_format == CSI_PIX_FMT_NV12))) {
            char str[200];
            static int i = 0;
			uint32_t dstw = frame->img.width;//800;
			uint32_t dsth = frame->img.height;//1280;
			uint32_t dsts = frame->img.strides[0];//896;


			PlinkYuvInfo info = {0};

			info.header.type = PLINK_TYPE_2D_YUV;
			info.header.size = DATA_SIZE(info);
			info.header.id = plink_ctx->sendid;

			info.format = PLINK_COLOR_FormatYUV420SemiPlanar;
			// info.bus_address_y = vi_mem_import(frame->img.dmabuf[0].fds) + frame->img.dmabuf[0].offset;
			// info.bus_address_u = info.bus_address_y + frame->img.dmabuf[1].offset;
            // vi_mem_release(info.bus_address_y);

			info.pic_width = dstw;
			info.pic_height = dsth;
			info.stride_y = dsts;
			info.stride_u = dsts;
			info.stride_v = dsts;
			info.offset_y = 0;
			info.offset_u = frame->img.dmabuf[1].offset;
			info.offset_v = frame->img.dmabuf[2].offset;
			pkt.list[0] = &info;
			pkt.num = 1;
			pkt.fd = frame->img.dmabuf[0].fds;
            
            if(plink_ctx->frame_buf[plink_ctx->sendid].img.dmabuf[0].fds!=0)
            {
                LOG_W("previous sendid :%d,fd %d not release,release it now\n",plink_ctx->sendid,plink_ctx->frame_buf[plink_ctx->sendid].img.dmabuf[0].fds);
                csi_camera_put_frame(&plink_ctx->frame_buf[plink_ctx->sendid]);
            }
            memcpy(&plink_ctx->frame_buf[plink_ctx->sendid],frame,sizeof(csi_frame_s));          
			plink_ctx->sendid = (plink_ctx->sendid + 1) % NUM_OF_BUFFERS;
			if (PLINK_send(plink_ctx->plink, plink_ctx->chnid, &pkt) != PLINK_STATUS_OK)
				plink_ctx->exitplink = 1;
			gettimeofday(&tv_end, 0);
			timersub(&tv_end, &tv_start, &tv_duration);
            pthread_mutex_lock(&plink_ctx->mutex);
			plink_ctx->available_bufs -= 1;
            pthread_mutex_unlock(&plink_ctx->mutex);
		} else if (!plink_ctx->exitplink && plink_ctx->available_bufs > 0 && plink_ctx->plink != NULL && (frame->img.pix_format == CSI_PIX_FMT_BGR)) {
			CsiPictureBuffer *buf = &plink_ctx->sendbuffer[plink_ctx->sendid];
			int y = 0;
			// if (1) {
    		// 	void *pbuf[3];
			// 	void *phyaddr = vi_mem_import(frame->img.dmabuf[0].fds);
			// 	pbuf[0] = vi_mem_map(phyaddr) + frame->img.dmabuf[0].offset;
			// 	pbuf[1] = pbuf[0] + frame->img.dmabuf[1].offset;
			// 	vi_mem_release(phyaddr);
			// 	frame->img.usr_addr[0] = pbuf[0];
			// }
			uint8_t *src = frame->img.usr_addr[0];
			uint8_t *dst = buf->virtual_address;
			uint32_t srcw = frame->img.width;
			uint32_t srch = frame->img.height;
			uint32_t dstw = frame->img.width;
			uint32_t dsth = frame->img.height;
			uint32_t dsts = frame->img.width;
			switch (frame->img.pix_format ) {
				case CSI_PIX_FMT_BGR:
				{
					dstw = 300;
					dsth = 304;
					uint32_t w = dstw > srcw ? srcw : dstw;
					uint32_t h = dsth > srch ? srch : dsth;
					if (w != 300 || h != 176 || dsts != 304) {
						fprintf(stderr, "ERROR: expect 300x300 (stride 304) image while recieved %dx%d (stride %d) image\n", w, h, dsts);
						break;
					}
					for (y = 0; y < h; y++)
					{
						memcpy(dst, src, w);
						src += srcw;
						dst += dsts;
					}
					src = frame->img.usr_addr[0] + srcw*srch;
					dst = buf->virtual_address + dsts*dsth;
					for (y = 0; y < h; y++)
					{
						memcpy(dst, src, w);
						src += srcw;
						dst += dsts;
					}
					src = frame->img.usr_addr[0] + srcw*srch*2;
					dst = buf->virtual_address + dsts*dsth*2;
					for (y = 0; y < h; y++)
					{
						memcpy(dst, src, w);
						src += srcw;
						dst += dsts;
					}

					PlinkRGBInfo info = {0};

					info.header.type = PLINK_TYPE_2D_RGB;
					info.header.size = DATA_SIZE(info);
					info.header.id = plink_ctx->sendid + 1;

					info.format = PLINK_COLOR_Format24BitBGR888Planar;
					info.bus_address_b = buf->bus_address;
					info.bus_address_g = info.bus_address_b + dsts*dsth;
					info.bus_address_r = info.bus_address_g + dsts*dsth;
					info.img_width = dstw;
					info.img_height = dsth;
					info.stride_b = dsts;
					info.stride_g = dsts;
					info.stride_r = dsts;

					pkt.list[0] = &info;
					pkt.num = 1;
					pkt.fd = buf->fd;

					if (PLINK_send(plink_ctx->plink, plink_ctx->chnid, &pkt) != PLINK_STATUS_OK)
						plink_ctx->exitplink = 1;
					break;
				}

				case CSI_PIX_FMT_NV12:
				default:
				{
					uint32_t w = dstw > srcw ? srcw : dstw;
					uint32_t h = dsth > srch ? srch : dsth;
					for (y = 0; y < h; y++)
					{
						memcpy(dst, src, w);
						src += srcw;
						dst += dsts;
					}
					src = frame->img.usr_addr[0] + frame->img.strides[0] * frame->img.height;
					dst = buf->virtual_address + (dsts * dsth);
					for (y = 0; y < h/2; y++)
					{
						memcpy(dst, src, w);
						src += srcw;
						dst += dsts;
					}

					PlinkYuvInfo info = {0};

					info.header.type = PLINK_TYPE_2D_YUV;
					info.header.size = DATA_SIZE(info);
					info.header.id = plink_ctx->sendid + 1;

					info.format = PLINK_COLOR_FormatYUV420SemiPlanar;
					info.bus_address_y = buf->bus_address;
					info.bus_address_u = buf->bus_address + dsts*dsth;
					info.bus_address_v = info.bus_address_u;
					info.pic_width = dstw;
					info.pic_height = dsth;
					info.stride_y = dsts;
					info.stride_u = dsts;
					info.stride_v = dsts;

					pkt.list[0] = &info;
					pkt.num = 1;
					pkt.fd = buf->fd;
					if (PLINK_send(plink_ctx->plink, plink_ctx->chnid, &pkt) != PLINK_STATUS_OK)
						plink_ctx->exitplink = 1;
					break;
				}
			}

			gettimeofday(&tv_end, 0);
			timersub(&tv_end, &tv_start, &tv_duration);
            pthread_mutex_lock(&plink_ctx->mutex);
			plink_ctx->sendid = (plink_ctx->sendid + 1) % NUM_OF_BUFFERS;
			plink_ctx->available_bufs -= 1;
            pthread_mutex_unlock(&plink_ctx->mutex);
		}
	}
	LOG_O("%s exit \n", __func__);
}
