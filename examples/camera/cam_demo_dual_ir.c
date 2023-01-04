/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: ShenWuYi <shenwuyi.swy@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#define LOG_LEVEL 5
#define LOG_PREFIX "cam_demo_dual_ir"
#include <syslog.h>

#include <csi_frame.h>
#include <csi_camera.h>
#include <csi_camera_dev_api.h>
#include "list.h"
#include "video_mem.h"
#include "csi_dsp_api.h"
#include "csi_dsp_task_defs.h"
#include "csi_dsp_post_process_defs.h"
#ifdef PLATFORM_SIMULATOR
#include "apputilities.h"
#endif
#define  MAX_CAM_NUM    3
#define  MAX_CHANNEL_NUM 3 

#define TEST_DEVICE_NAME "/dev/video0"
#define CSI_CAMERA_TRUE  1
#define CSI_CAMERA_FALSE 0

/*************************************common data struct************************************************************************/
typedef struct msg_queue_item{
    void* payload;
    struct msg_queue_item *next;
}msg_queue_item_t;
typedef struct msg_queue{
         msg_queue_item_t *head;
         msg_queue_item_t *tail;
         int exit;
         pthread_mutex_t mutex;
         pthread_cond_t cond;
}msg_queue_ctx_t;


typedef struct cam_dsp_info{
    int master_fd;
    int slave_fd;
    /**add AE ctrl info********/
}cam_dsp_info_t;


/*************************************camera data struct*************************************************************/
typedef enum{
    CAM_SYNC_MSG_MAIN_FRAME,
    CAM_SYNC_MSG_SLAVE_FRAME,
    CAM_SYNC_MSG_AI_INFO,
    CAM_SYNC_MSG_DSP_INFO,
}cam_sync_message_type_e;

typedef struct{

}cam_ai_info_t;


typedef struct camera_sync_msg{
    cam_sync_message_type_e  type;
    union{
        csi_frame_s  frame;
        cam_ai_info_t ai_info;
        cam_dsp_info_t  dsp_info;
    };
}camera_sync_msg_t;


typedef enum _cam_type{
    CAM_TYEP_MASTER =0,
    CAM_TYEP_SLAVE,
    CAM_TYEP_INVALID,
}cam_type_e;

typedef enum _frame_mode{
    FRAME_NONE =0,
    FRAME_SAVE_IMG =1,
    FRAME_SAVE_STREAM =2,
    FRAME_SAVE_DISPLAY,
    FRAME_INVALID,
}frame_mode_t;
typedef struct _camera_param{
    int video_id;
    int channel_num;
    struct {
        int width;
        int height;
        csi_pixel_fmt_e fmt;

    }out_pic[MAX_CHANNEL_NUM];

    cam_type_e  type;
    int frames_to_stop;
}camera_param_t;

typedef struct event_queue_item{
    struct csi_camera_event evet;
    struct event_queue_item *next;
}event_queue_item_t;

typedef struct _camera_ctx{

    int cam_id;
    // pthread_t  cam_thread;
    int exit;
    msg_queue_ctx_t *cam_sync_queue;
    csi_cam_handle_t cam_handle;
    int  channel_num;
    csi_cam_event_handle_t event_handle;
    csi_camera_event_subscription_s event_subscribe;
 	csi_camera_channel_cfg_s chn_cfg[MAX_CHANNEL_NUM];
    int frame_num;
    struct timeval init_time;
    float fps;
    // frame_mode_t  frame_mode[MAX_CHANNEL_NUM];
}camera_ctx_t;

/********************************dsp data struct*********************************************************/
#define  MAX_DSP_DEVICE  2


typedef struct algo_pic_out_setting{
    enum{    
        ALGO_PIC_MODE_FULL_COPY,
        ALGO_PIC_MODE_FULL_COPY_WITH_PARAM,
        ALGO_PIC_MODE_FULL_COPY_NONE,
    }mode;
    int buf_id;
    int h_offset;
    int height;
    uint32_t  frame_id;
    uint64_t  timestap;
    uint32_t  temp_projector;   //投射器温度
}algo_pic_out_setting_t;

typedef struct algo_result{
    char lib_name[16];
    uint64_t timestap;
}algo_result_t;

typedef struct dsp_dual_ir_frame_msg{
    size_t		size;
    uint32_t	width;
	uint32_t	height;
    uint32_t    master_stride;
    uint32_t    slave_stride;
    csi_pixel_fmt_e	pix_format;
    int master_fd;
    int slave_fd;
    uint32_t  frame_id;
    uint64_t  timestap;
    uint32_t  temp_projector;   //投射器温度

}dsp_dual_ir_frame_msg_t;

typedef struct dsp_ctx{
    int    dsp_id;
    void * instance;
    void*  dsp_task;
    char*  algo_name;
}dsp_ctx_t;

typedef enum {
    DUAL_DSP_ALGO_TYPE_DSP0_COPY =0,
    DUAL_DSP_ALGO_TYPE_DSP1_COPY,
    DUAL_DSP_ALGO_TYPE_DSP01_COPY,

}dual_dsp_algo_type_e;
typedef struct dual_dsp_handle{
    dsp_ctx_t dsp_ctx[MAX_DSP_DEVICE];
    msg_queue_ctx_t dsp_prcoess_queue;
    msg_queue_ctx_t *cam_sync_queue;
    void *mem_allocor;
    pthread_t thread_dsp_process; 
    frame_mode_t  frame_mode;
    dual_dsp_algo_type_e dsp_run_type;
}dual_dsp_handle_t;


int file_id = 0;
extern void *vi_plink_create(csi_camera_channel_cfg_s *chn_cfg);
extern void vi_plink_release(void * plink);
extern void display_camera_frame(void * plink, csi_frame_s *frame);

static void *cam_frame_sync_process(void *ctx);
static void* dsp_dual_ir_porcess(void *arg);
/*******************************common func***********************************************************************/
static void usage(void)
{
	printf("    1 : IR 1 Camera id \n");
    printf("    2 : IR 2 Camera id \n");
    printf("    3 : runing Camera frame num :0 not stop ,else frame num to stop \n");
	printf("    4 : DSP frame mode :0 none ,1 dump enable,2 display \n");

}

void printUsage(char *name)
{
    printf("usage: %s [options]\n"
           "\n"
           "  Available options:\n"
           "    -m      master camera id \n"
           "    -s      slave camera device id)\n"
           "    -w      width (default: 800))\n"
           "    -h      height (default: 1280))\n"
           "    -n      runing Camera frame num :0 not stop ,else frame num to stop )\n"
           "    -M      DSP frame mode :0 none ,1 dump enable,2 display)\n"
           "    -a      DSP algo mode :0 catch master frame ,1 catch slave frame, 2 catch each half frame)\n"
           "\n", name);
}

static int  parseParams(int argc, char **argv, camera_param_t *params,frame_mode_t *mode,dual_dsp_algo_type_e *dsp_type)
{
        int index =0;
        int i = 1;
        params[index].video_id = 0;
        params[index].type = CAM_TYEP_MASTER;

        params[index].channel_num =1;
        params[index].out_pic[0].width = 1920;
        params[index].out_pic[0].height = 1088;
        params[index].out_pic[0].fmt = CSI_PIX_FMT_RAW_10BIT;

        params[index].frames_to_stop = 30;
        index++;

        params[index].video_id = 15;
        params[index].type = CAM_TYEP_SLAVE;

        params[index].channel_num = 1;
        params[index].out_pic[0].width = 1920;
        params[index].out_pic[0].height = 1088;
        params[index].out_pic[0].fmt =CSI_PIX_FMT_RAW_10BIT;

        params[index].frames_to_stop = 30;
        index++;
        *mode = FRAME_NONE;
        *dsp_type = DUAL_DSP_ALGO_TYPE_DSP01_COPY;
        while (i < argc)
        {
            if (argv[i][0] != '-' || strlen(argv[i]) < 2)
            {
                i++;
                continue;
            }
            
            if (argv[i][1] == 'm')
            {
                if (++i < argc)
                    params[0].video_id = atoi(argv[i++]);
            }
            else if (argv[i][1] == 's')
            {
                if (++i < argc)
                    params[1].video_id = atoi(argv[i++]);
            }
            else if (argv[i][1] == 'w')
            {
                if (++i < argc)
                     params[0].out_pic[0].width = atoi(argv[i++]);
                params[1].out_pic[0].width = params[0].out_pic[0].width;
            }
            else if (argv[i][1] == 'h')
            {
                if (++i < argc)
                    params[0].out_pic[0].height = atoi(argv[i++]);
                params[1].out_pic[0].height = params[0].out_pic[0].height;
            }
            else if (argv[i][1] == 'n')
            {
                if (++i < argc)
                    params[0].frames_to_stop = atoi(argv[i++]);
                 params[1].frames_to_stop = params[0].frames_to_stop;
            }
            else if (argv[i][1] == 'M')
            {
                if (++i < argc)
                    *mode = atoi(argv[i++]);
            }
            else if (argv[i][1] == 'a')
            {
                if (++i < argc)
                     *dsp_type = atoi(argv[i++]);
            }
            else if (strcmp(argv[i], "--help") == 0)
            {
                 printUsage(argv[0]);
                 return -1;
            }
        }
        printf("[DUAL IR] master camera         : %d\n", params[0].video_id);
        printf("[DUAL IR] master camera         : %d\n",params[1].video_id);
        printf("[DUAL IR] Resolution            : %dx%d\n", params[0].out_pic[0].width,params[0].out_pic[0].height);
        printf("[DUAL IR] run frame             : %d\n", params[0].frames_to_stop);
        printf("[DUAL IR] frame mode            : %d\n", *mode);
        printf("[DUAL IR] dsp run type          : %d\n", *dsp_type);

        return index;   
}

static void get_system_time(const char *func, int line_num)
{
	struct timeval cur_time;

	memset(&cur_time, 0, sizeof(cur_time));
	gettimeofday(&cur_time, 0);
	LOG_D("%s %s line_num = %d, cur_time.tv_sec = %ld, cur_time.tv_usec = %ld\n",
		__func__, func, line_num, cur_time.tv_sec, cur_time.tv_usec);
}

static  msg_queue_item_t *dequeue_msg(msg_queue_ctx_t *ctx)
{
	msg_queue_item_t *ev = ctx->head;
    pthread_mutex_lock(&ctx->mutex);
	if(!ev)
    {
        pthread_mutex_unlock(&ctx->mutex);
        return NULL;
    }

	if(ev == ctx->tail)
		ctx->tail = NULL;
	ctx->head = ev->next;
    pthread_mutex_unlock(&ctx->mutex);
    return ev;
}

static int enqueue_msg(msg_queue_ctx_t *ctx,msg_queue_item_t * item)
{

	if(!item || !ctx)
		return -1;
    item->next = NULL;
	LOG_D("%s enter \n", __func__);
	pthread_mutex_lock(&ctx->mutex);
	if (ctx->tail) {
		ctx->tail->next = item;
	} else {
		ctx->head = item;
        // pthread_cond_broadcast(&ctx->cond);
	}
	ctx->tail = item;
	pthread_mutex_unlock(&ctx->mutex);
    return 0;
}

/***********************DSP Dual-IR process *******************************/
static void dsp_dump_frame(void *frame_buf, csi_pixel_fmt_e fmt, int width,int height,int stride)
{
	char file[128];
	static int file_indx=0;
	int size;
	uint32_t indexd, j;
              
	sprintf(file,"demo_save_img_%d",file_indx++%10);
	int fd = open(file, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH);
	if(fd == -1) {
		LOG_E("%s, %d, open file error!!!!!!!!!!!!!!!\n", __func__, __LINE__);
		return;
	}


	LOG_O("save img from to %s, fmt:%d width:%d stride:%d height:%d \n",file, fmt, width, stride,height);

	switch(fmt) {
        default:
            size = width*2;
            for (j = 0; j < height; j++) {
				indexd = j*stride;
				write(fd, frame_buf + indexd, size);
			}

	}
	close(fd);
}


static void* req_DmaBuffer(void*mem_allocor,VmemParams *params)
{

    int pgsize = getpagesize();
    params->size = ((params->size+pgsize-1)/pgsize)*pgsize;
    if(VMEM_allocate(mem_allocor, params))
    {
        return NULL;
    }

    LOG_O("alloct dma buf @ phy:0x%x\n",params->phy_address);
    if(VMEM_export(mem_allocor,params))
    {
        return NULL;
    }
    LOG_D("export dma buf @fd:%x\n",params->fd);
    if(VMEM_mmap(mem_allocor,params))
    {
        return NULL;
    }
    memset(params->vir_address,0xff,params->size);
    return params->vir_address;

}  

  static int release_DmaBuffer(void*mem_allocor,VmemParams *params)
  {
      VMEM_free(mem_allocor, params);
  }

static int dsp_construct_task(dsp_ctx_t *ctx)
{
    if(ctx== NULL || ctx->dsp_id >= MAX_DSP_DEVICE || ctx->dsp_id <0 || ctx->algo_name == NULL)
    {
        LOG_E("prama check fail\n");
        goto one_err;
    }
    
    ctx->instance = csi_dsp_create_instance(ctx->dsp_id);
    if (ctx->instance==NULL) {
        LOG_E("dsp create fail\n");
        goto one_err;
    }

    ctx->dsp_task = csi_dsp_create_task(ctx->instance, CSI_DSP_TASK_SW_TO_SW);
    if (ctx->dsp_task==NULL) {
        LOG_E("dsp task create fail.\n");
        goto two_err;
    } 

    if(csi_dsp_task_acquire_algo(ctx->dsp_task,ctx->algo_name))
    {
        LOG_E("dsp task create fail.\n");
        goto three_err;
    }
    return 0;
three_err:
    csi_dsp_destroy_task(ctx->dsp_task);
two_err:
    csi_dsp_delete_instance(ctx->instance);
one_err:
    return -1;

}


static int dsp_release_task(dsp_ctx_t *ctx)
{
    if(ctx==NULL || ctx->instance || ctx->dsp_task)
    {
        return -1;
    }
    csi_dsp_destroy_task(ctx->instance);
    csi_dsp_delete_instance(ctx->instance);


}

static int dsp_send_info_to_cam(msg_queue_ctx_t* cam_msg_queue,int master_fd,int slave_fd)
{
    camera_sync_msg_t * msg_payload;
    msg_queue_item_t * msg;
    if(cam_msg_queue==NULL|| master_fd<=0|| slave_fd<=0)
    {
        LOG_E("param check fail\n");
        return -1;
    }
    msg_payload = malloc(sizeof(camera_sync_msg_t));
    if(msg_payload == NULL)
    {
        LOG_E("malloc fail\n");
        return -1;
    }
    msg = malloc(sizeof(msg_queue_item_t));
    if(msg==NULL)
    {
        LOG_E("malloc fail\n");
        free(msg_payload);
        return -1;
    }
    msg_payload->type = CAM_SYNC_MSG_DSP_INFO;
    msg_payload->dsp_info.master_fd = master_fd;
    msg_payload->dsp_info.slave_fd = slave_fd;
    msg->payload = msg_payload;

    if(enqueue_msg(cam_msg_queue,msg))
    {
        free(msg_payload);
        free(msg);
        return -1;
    }
    LOG_D("send dsp resp to cam sync for fd(%d,%d)\n",master_fd,slave_fd);
    return 0;    
}

static dual_dsp_handle_t* dsp_process_create(frame_mode_t mode,dual_dsp_algo_type_e type)
{
    dual_dsp_handle_t *dsp_hdl=NULL;
    dsp_hdl = malloc(sizeof(dual_dsp_handle_t));
    if(dsp_hdl==NULL)
    {
        return NULL;
    }
    dsp_hdl->dsp_ctx[0].dsp_id = 0;
    dsp_hdl->dsp_ctx[1].dsp_id = 1;
    dsp_hdl->dsp_ctx[0].algo_name = "dummy_ir_algo_flo_0";
    dsp_hdl->dsp_ctx[1].algo_name = "dummy_ir_algo_flo_1";
    dsp_hdl->dsp_prcoess_queue.head=NULL;
    dsp_hdl->dsp_prcoess_queue.tail=NULL;
    dsp_hdl->dsp_prcoess_queue.exit = 0;
    dsp_hdl->frame_mode = mode;
    dsp_hdl->dsp_run_type = type;
    pthread_mutex_init(&dsp_hdl->dsp_prcoess_queue.mutex,NULL);
    
    if(dsp_construct_task(&dsp_hdl->dsp_ctx[0]))
    {
        free(dsp_hdl);
        return NULL;
    }
    if(dsp_construct_task(&dsp_hdl->dsp_ctx[1]))
    {
        dsp_release_task(&dsp_hdl->dsp_ctx[0]);
        free(dsp_hdl);
        return NULL;
    }
    if( VMEM_create(&dsp_hdl->mem_allocor) <0)
    {
        dsp_release_task(&dsp_hdl->dsp_ctx[0]);
        dsp_release_task(&dsp_hdl->dsp_ctx[1]);
        free(dsp_hdl);
        return NULL;
    }
    if(pthread_create(&dsp_hdl->thread_dsp_process,NULL,dsp_dual_ir_porcess,dsp_hdl))
    {
        dsp_release_task(&dsp_hdl->dsp_ctx[0]);
        dsp_release_task(&dsp_hdl->dsp_ctx[1]);
        VMEM_destroy(dsp_hdl->mem_allocor);
        free(dsp_hdl);
        return NULL;
    }
    LOG_D("dsp process crated\n");
    return dsp_hdl;
}

void dsp_process_destroy(dual_dsp_handle_t* dsp_hdl)
{
    if(dsp_hdl==NULL)
    {
        LOG_E("NULL Ptr\n");
        return;
    }

    dsp_hdl->dsp_prcoess_queue.exit=1;
    pthread_join(dsp_hdl->thread_dsp_process,NULL);

    dsp_release_task(&dsp_hdl->dsp_ctx[0]);
    dsp_release_task(&dsp_hdl->dsp_ctx[1]);
    VMEM_destroy(dsp_hdl->mem_allocor);
    free(dsp_hdl);
    return;
}

static void* dsp_dual_ir_porcess(void *arg)
{
    msg_queue_item_t * msg=NULL;
    dsp_dual_ir_frame_msg_t *dual_ir_frame;
    VmemParams params_out;
    void* out_buf=NULL;
    algo_pic_out_setting_t setting;
    algo_result_t *dsp0_result;
    algo_result_t *dsp1_result;
    dual_dsp_handle_t *dsp_hdl =(dual_dsp_handle_t *) arg;
    if(dsp_hdl==NULL)
    {
        pthread_exit(0);
    }
    // alloc DSP out put buf from video memory ,which is dma buf
    params_out.size = 1920*1088*2;
    params_out.flags = VMEM_FLAG_CONTIGUOUS;
    out_buf = req_DmaBuffer(dsp_hdl->mem_allocor,&params_out);
    if(out_buf == NULL)
    {
        LOG_E("req dma Buf fail\n");
        pthread_exit(0);
    }

    LOG_O("Runing ...\n");
    while(dsp_hdl->dsp_prcoess_queue.exit!=1)
    {
        if(msg!=NULL)
        {
            free(msg);
        }
        msg =  dequeue_msg(&dsp_hdl->dsp_prcoess_queue);
        if(msg == NULL)
        {
            usleep(1000);
            continue;
        }
        LOG_D("get msg\n");
        dual_ir_frame = (dsp_dual_ir_frame_msg_t *)msg->payload;

        struct csi_sw_task_req* req1 = NULL;
        struct csi_sw_task_req* req2 = NULL;
        req1 = csi_dsp_task_create_request(dsp_hdl->dsp_ctx[0].dsp_task);
        if (req1 == NULL) {
            LOG_W("req create fail.\n");
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            continue;
        }
        req2 =  csi_dsp_task_create_request(dsp_hdl->dsp_ctx[1].dsp_task);
        if (req2 == NULL) {
            LOG_W("req create fail.\n");
            csi_dsp_task_release_request(req1);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            continue;
        }


        struct csi_dsp_buffer  buf1;
        struct csi_dsp_buffer  buf2;
        struct csi_dsp_buffer  buf3;
        struct csi_dsp_buffer  buf4;
      // import main camera frame
        buf1.buf_id = 0;
        buf1.dir = CSI_DSP_BUFFER_IN;
        buf1.type = CSI_DSP_BUF_TYPE_DMA_BUF_IMPORT;
        buf1.plane_count = 1;
        buf1.width = dual_ir_frame->width;
        buf1.height = dual_ir_frame->height;
        buf1.format = 2;
        buf1.planes[0].stride = dual_ir_frame->master_stride;
        buf1.planes[0].size = dual_ir_frame->size;
        buf1.planes[0].fd = dual_ir_frame->master_fd;
         // import slave cmaera  frame
        buf2.buf_id = 1;
        buf2.dir = CSI_DSP_BUFFER_IN;
        buf2.type = CSI_DSP_BUF_TYPE_DMA_BUF_IMPORT;
        buf2.plane_count = 1;
        buf2.width = dual_ir_frame->width;
        buf2.height = dual_ir_frame->height;
        buf2.format=2;
        buf2.planes[0].stride = dual_ir_frame->slave_stride;
        buf2.planes[0].size = dual_ir_frame->size;
        buf2.planes[0].fd = dual_ir_frame->slave_fd;
        // import output frame  buf 
        buf3.buf_id = 2;
        buf3.dir = CSI_DSP_BUFFER_OUT;
        buf3.type = CSI_DSP_BUF_TYPE_DMA_BUF_IMPORT;
        buf3.plane_count = 1;
        buf3.width = dual_ir_frame->width;
        buf3.height = dual_ir_frame->height;
        buf3.format=2;
        buf3.planes[0].stride = dual_ir_frame->width*2;
        buf3.planes[0].size = params_out.size;
        buf3.planes[0].fd = params_out.fd;
    // malloc algo result buf
        buf4.buf_id = 3;
        buf4.dir = CSI_DSP_BUFFER_OUT;
        buf4.type = CSI_DSP_BUF_ALLOC_DRV;
        buf4.plane_count = 1;
        buf4.planes[0].size = sizeof(algo_result_t);
    /**********************add buf for dsp0 req*****************************************/
    /* buf0:master frame; buf1:output buf; buf2: slave frame,buf3:algo result buf********/
        if(csi_dsp_request_add_buffer(req1,&buf1))
        {
            
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }
        if(csi_dsp_request_add_buffer(req1,&buf3))
        {
            
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }


        if(csi_dsp_request_add_buffer(req1,&buf2))
        {
            
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }

        if(csi_dsp_request_add_buffer(req1,&buf4))
        {
            
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }
        dsp0_result = (algo_result_t *)buf4.planes[0].buf_vir;
    /**********************add buf for dsp1 req*****************************************/
     /* buf0:salve frame; buf1:output buf; buf2: master frame,buf3:algo result buf********/
        if(csi_dsp_request_add_buffer(req2,&buf2))
        {
            
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }

        if(csi_dsp_request_add_buffer(req2,&buf3))
        {
            
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }

        if(csi_dsp_request_add_buffer(req2,&buf1))
        {
            
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }
        if(csi_dsp_request_add_buffer(req2,&buf4))
        {
            
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }
        dsp1_result = (algo_result_t *)buf4.planes[0].buf_vir;
        if(dsp_hdl->dsp_run_type == DUAL_DSP_ALGO_TYPE_DSP01_COPY )
        {
            setting.mode =   ALGO_PIC_MODE_FULL_COPY_WITH_PARAM;
            setting.buf_id=0;
            setting.h_offset = dual_ir_frame->height/2;
            setting.height =dual_ir_frame->height/2;
        }else if(dsp_hdl->dsp_run_type == DUAL_DSP_ALGO_TYPE_DSP0_COPY)
        {
            setting.mode =   ALGO_PIC_MODE_FULL_COPY;
        }
        else
        {
            setting.mode =   ALGO_PIC_MODE_FULL_COPY_NONE;
        }

        setting.timestap = dual_ir_frame->timestap;
        if(csi_dsp_request_set_property(req1,&setting,sizeof(setting)))
        {
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }

        if(dsp_hdl->dsp_run_type == DUAL_DSP_ALGO_TYPE_DSP01_COPY )
        {
            setting.mode =   ALGO_PIC_MODE_FULL_COPY_WITH_PARAM;
            setting.buf_id=0;
            setting.h_offset = 0;
            setting.height =dual_ir_frame->height/2;
        }else if(dsp_hdl->dsp_run_type == DUAL_DSP_ALGO_TYPE_DSP1_COPY)
        {
            setting.mode =   ALGO_PIC_MODE_FULL_COPY;
        }
        else
        {
            setting.mode =   ALGO_PIC_MODE_FULL_COPY_NONE;
        }        
        
        setting.timestap = dual_ir_frame->timestap;
        if(csi_dsp_request_set_property(req2,&setting,sizeof(setting)))
        {
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }  


        if(csi_dsp_request_enqueue(req1))
        {
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }

        if(csi_dsp_request_enqueue(req2))
        {
            req1 = csi_dsp_request_dequeue(dsp_hdl->dsp_ctx[0].dsp_task);
            csi_dsp_task_release_request(req1);
            csi_dsp_task_release_request(req2);
            dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);
            LOG_W("req create fail.\n");
            continue;
        }
        req1 = csi_dsp_request_dequeue(dsp_hdl->dsp_ctx[0].dsp_task);
        if(req1 == NULL)
        {
            LOG_W("dequeue fail.\n");
            
        }
        req2 =csi_dsp_request_dequeue(dsp_hdl->dsp_ctx[1].dsp_task);
        if(req2 == NULL)
        {
             LOG_W("dequeue fail.\n");
        }
        LOG_D("dsp0 lib:%s timestap:%lx,dsp1 lib:%s,timestap:%lx\n",dsp0_result->lib_name,dsp0_result->timestap,
                                                   dsp1_result->lib_name,dsp1_result->timestap);

        dsp_send_info_to_cam(dsp_hdl->cam_sync_queue, dual_ir_frame->master_fd,dual_ir_frame->slave_fd);

        switch(dsp_hdl->frame_mode)
        {
            case FRAME_NONE:
                    break;
            case FRAME_SAVE_IMG:
               dsp_dump_frame((void *)buf3.planes[0].buf_vir,0,buf3.width,buf3.height,buf3.planes[0].stride);
               break;
            default:
                LOG_W("unsupport frame mode %d\n",dsp_hdl->frame_mode);
        }
        csi_dsp_task_release_request(req1);
        csi_dsp_task_release_request(req2);
     
    }
    release_DmaBuffer(dsp_hdl->mem_allocor,&params_out);
    LOG_O("exit\n");
    pthread_exit(0);
}

/************************************Camera sync process***************************/
#define  MAX_FIFO_FACE_INFO_NUM    2
#define  MAX_FIFO_FRAME_NUM        2

  
typedef struct {
    struct list_head head;
    void* item;
}frame_item_t;

typedef struct cam_sync_process_ctx{
    msg_queue_ctx_t cam_sync_queue;
    msg_queue_ctx_t *dsp_prcoess_queue;       //DSP 消息queue
    csi_cam_handle_t master_cam_handle;
    csi_cam_handle_t slave_cam_handle;
    struct list_head  cam_master_frame_list;    // 缓存master camera 的frame
    struct list_head  cam_slave_frame_list;   // slave camera 的frame
    struct list_head  cam_busy_frame_list;   //缓存发送给DSP的frame
    struct list_head  ai_info_list;
    pthread_t thread_cam_sync;
    uint32_t  paried_frame_num;
}cam_sync_process_ctx_t;

static cam_sync_process_ctx_t *cam_sync_create()
{
    cam_sync_process_ctx_t * ctx;
    ctx = malloc(sizeof(cam_sync_process_ctx_t));
    if(ctx == NULL)
    {
        return NULL; 
    }
    ctx->cam_sync_queue.head=NULL;
    ctx->cam_sync_queue.tail = NULL;
    ctx->cam_sync_queue.exit =0;
    ctx->paried_frame_num = 0;
    pthread_mutex_init(&ctx->cam_sync_queue.mutex,NULL);
    INIT_LIST_HEAD(&ctx->cam_master_frame_list);
    INIT_LIST_HEAD(&ctx->ai_info_list);
    INIT_LIST_HEAD(&ctx->cam_busy_frame_list);
    INIT_LIST_HEAD(&ctx->cam_slave_frame_list);

    if(pthread_create(&ctx->thread_cam_sync,NULL,cam_frame_sync_process,ctx))
    {
        LOG_E("cam sync thread create fail\n");
        free(ctx);
        return NULL;
    }
    return ctx;
}


static void cam_sync_destroy(void *arg)
{
    cam_sync_process_ctx_t *ctx= (cam_sync_process_ctx_t *)arg;
    if(ctx==NULL)
    {
        return ;
    }
    ctx->cam_sync_queue.exit =1;
    pthread_join(ctx->thread_cam_sync,NULL);
    free(ctx);
    LOG_O("cam sync destroy\n");
}
static int push_new_frame(struct list_head *frame_list, csi_frame_s * new_frame)
{
    int entry_num =0;
    frame_item_t *frame_item = NULL;
    frame_item_t * new_item = NULL;
    if(frame_list == NULL || new_frame==NULL)
    {
        return -1;
    }

    list_for_each_entry(frame_item,frame_list,head){
        entry_num++;
    }
    if((entry_num+1)>MAX_FIFO_FRAME_NUM)
    {
        frame_item = list_first_entry(frame_list,frame_item_t,head);
        LOG_D("release frame fd:%d\n",((csi_frame_s *)frame_item->item)->img.fds[0]);
        csi_camera_put_frame((csi_frame_s *)frame_item->item);
        free(frame_item->item);
        list_del(&frame_item->head);
        free(frame_item);
    }
    new_item = malloc(sizeof(frame_item_t));
    new_item->item = new_frame;
    list_add_tail(&new_item->head,frame_list);
    return 0;
}

static csi_frame_s * pop_matched_frame_with_fd(struct list_head *frame_list,int fd)
{
    csi_frame_s *  frame = NULL;
    frame_item_t *frame_item = NULL;
    frame_item_t * temp;
    if(frame_list==NULL)
    {
        return NULL;
    }
    list_for_each_entry_safe(frame_item,temp,frame_list,head){
        frame = (csi_frame_s *)frame_item->item;
        if(frame->img.fds[0]==fd)
        {
            list_del(&frame_item->head);
            free(frame_item);
            LOG_D("get frame fd:%d\n",frame->img.fds[0]);
            return frame;
        }
    }
    return NULL;
}


static csi_frame_s * pop_matched_frame_with_ts(struct list_head *frame_list, csi_frame_s * target_frame,uint32_t rang_us,bool clear_early)
{
    csi_camera_meta_s *meta_data = (csi_camera_meta_s *)target_frame->meta.data;
	csi_camrea_meta_unit_s target_ts,loop_ts;
     csi_frame_s *  frame;
    frame_item_t *frame_item = NULL;
    frame_item_t * tmp;
    struct timeval  time_value;
    long long target_us,loop_us,deta_us;
    if(target_frame == NULL || frame_list==NULL)
    {
        return NULL;
    }
    if(list_empty(frame_list))
    {
        return NULL;
    }
    csi_camera_frame_get_meta_unit(
		&target_ts, meta_data, CSI_CAMERA_META_ID_TIMESTAMP);

    target_us = target_ts.time_value.tv_sec*1000000 + target_ts.time_value.tv_usec;

    list_for_each_entry_safe(frame_item,tmp,frame_list,head){
            frame = ( csi_frame_s *)frame_item->item;
            meta_data = (csi_camera_meta_s *)frame->meta.data;
            csi_camera_frame_get_meta_unit(
	    	&loop_ts, meta_data, CSI_CAMERA_META_ID_TIMESTAMP);
            loop_us =  loop_ts.time_value.tv_sec*1000000 + loop_ts.time_value.tv_usec;
            deta_us = (target_us-loop_us);
            if(deta_us <=rang_us || deta_us>=(-rang_us))
            {
                list_del(&frame_item->head);
                free(frame_item);
                return frame;
            }
            if(clear_early && deta_us < 0)
            {
               csi_camera_put_frame(frame);
               list_del(&frame_item->head);
               free(frame_item);
            }
            frame_item=NULL;
    }
    return NULL;

}

static int send_paired_frame_to_dsp(msg_queue_ctx_t *queue,csi_frame_s *master_frame,csi_frame_s *slave_frame,struct list_head *send_frames)
{
    dsp_dual_ir_frame_msg_t *dsp_msg_payload;
    msg_queue_item_t *dsp_msg;
    frame_item_t *main_item,*slave_item;
    csi_camera_meta_s *meta_data;
	csi_camrea_meta_unit_s timestap;
    if(queue==NULL || master_frame==NULL || slave_frame==NULL)
    {
        return -1;
    }
    
    if(master_frame->img.width!=slave_frame->img.width||
        master_frame->img.height!=slave_frame->img.height)       
        {
            LOG_E("paird frame not the same picture param\n");
            return -1;
        }
    dsp_msg=malloc(sizeof(msg_queue_item_t));
    if(dsp_msg==NULL){
        return -1;
    }
    dsp_msg_payload = malloc(sizeof(dsp_dual_ir_frame_msg_t));
    if(dsp_msg_payload==NULL)
    {
        free(dsp_msg);
        return -1;
    }
    meta_data = (csi_camera_meta_s *)master_frame->meta.data;
    csi_camera_frame_get_meta_unit(
		&timestap, meta_data, CSI_CAMERA_META_ID_TIMESTAMP);

    dsp_msg->payload = dsp_msg_payload;

    dsp_msg_payload->size = master_frame->img.height*master_frame->img.strides[0];
    dsp_msg_payload->width = master_frame->img.width;
    dsp_msg_payload->height = master_frame->img.height;
    dsp_msg_payload->pix_format = master_frame->img.pix_format;
    dsp_msg_payload->master_stride = master_frame->img.strides[0];
    dsp_msg_payload->master_fd =  master_frame->img.fds[0];
    dsp_msg_payload->slave_fd =  slave_frame->img.fds[0];
    dsp_msg_payload->slave_stride = slave_frame->img.strides[0];
    dsp_msg_payload->timestap = timestap.time_value.tv_sec*1000000+timestap.time_value.tv_usec;
    enqueue_msg(queue,dsp_msg);
    main_item = malloc(sizeof(frame_item_t));
    main_item->item = master_frame;
    slave_item = malloc(sizeof(frame_item_t));
    slave_item->item = slave_frame;
    list_add_tail(&main_item->head,send_frames);
    list_add_tail(&slave_item->head,send_frames);
    LOG_D("send paired fd (%d,%d)\n",dsp_msg_payload->master_fd,dsp_msg_payload->slave_fd);
    return 0;
}

static int cam_dual_frame_process(struct list_head * pair_frame_list,struct list_head * free_frame_list, struct list_head *busy_frame_list,csi_frame_s *frame, msg_queue_ctx_t *dsp_prcoess_queue,cam_sync_message_type_e type)
{
    csi_frame_s *new_frame = NULL;
    csi_frame_s *pair_frame = NULL;
    csi_frame_s *master_frame = NULL;
    csi_frame_s *slave_frame = NULL;

    if(pair_frame_list == NULL ||free_frame_list==NULL ||busy_frame_list ==NULL || 
        frame == NULL || dsp_prcoess_queue == NULL || type >CAM_SYNC_MSG_SLAVE_FRAME)
        {
            LOG_E("param check fail \n");
            return -1;
        }

    new_frame = malloc(sizeof(csi_frame_s));
    if(new_frame==NULL)
    {
        LOG_E("malloc fail\n");
        csi_camera_put_frame(frame);
        return -1;
    }
    memcpy(new_frame,frame,sizeof(csi_frame_s));
    pair_frame = pop_matched_frame_with_ts(pair_frame_list,frame,0,true);
    if(pair_frame == NULL)
    {
        if(push_new_frame(free_frame_list,new_frame))
        {
                csi_camera_put_frame(new_frame);
                free(new_frame);
                LOG_E("save new frame fail\n");
                return -1;
        }                      
    }else
    {
        if(type == CAM_SYNC_MSG_MAIN_FRAME)
        {
            master_frame = new_frame;
            slave_frame = pair_frame;
        }else
        {
            master_frame = pair_frame;
            slave_frame = new_frame;
        }
        if(send_paired_frame_to_dsp(dsp_prcoess_queue,master_frame,slave_frame,busy_frame_list))
        {
            LOG_E("fail to send paired frame fd (%d,%d) to dsp\n",new_frame->img.fds[0],pair_frame->img.fds[0]);
            csi_camera_put_frame(master_frame);
            free(master_frame);
            csi_camera_put_frame(slave_frame);
            free(slave_frame);
            return -1;
        }
    } 
    return 0;
}

static void *cam_frame_sync_process(void *arg)
{
    camera_sync_msg_t  *msg;
    csi_frame_s * master_frame=NULL;
    csi_frame_s * slave_frame=NULL;
    struct list_head *matched_list=NULL;
    msg_queue_item_t * item;
    cam_sync_process_ctx_t *ctx =(cam_sync_process_ctx_t *)arg;
    LOG_O("frame sync process running....\n");
    while(ctx->cam_sync_queue.exit!=1)
    {
        item =  dequeue_msg(&ctx->cam_sync_queue);
        if(item == NULL)
        {
            usleep(1000);
            continue;
        }
        msg = (camera_sync_msg_t  *)item->payload;
        free(item);
        LOG_D("get msg:%d\n",msg->type);
        switch(msg->type)
        {
            case CAM_SYNC_MSG_MAIN_FRAME:
                     cam_dual_frame_process(&ctx->cam_slave_frame_list,&ctx->cam_master_frame_list,&ctx->cam_busy_frame_list,&msg->frame,ctx->dsp_prcoess_queue,CAM_SYNC_MSG_MAIN_FRAME);
                     break;
            case CAM_SYNC_MSG_SLAVE_FRAME:
                     cam_dual_frame_process(&ctx->cam_master_frame_list,&ctx->cam_slave_frame_list,&ctx->cam_busy_frame_list,&msg->frame,ctx->dsp_prcoess_queue,CAM_SYNC_MSG_SLAVE_FRAME);
                     break;
            case CAM_SYNC_MSG_AI_INFO:
                    break;
            case CAM_SYNC_MSG_DSP_INFO:
                    master_frame = pop_matched_frame_with_fd(&ctx->cam_busy_frame_list,msg->dsp_info.master_fd);
                    if(master_frame == NULL)
                    {
                        LOG_E("fail to find sending fd\n");
                    }else
                    {
                        csi_camera_put_frame(master_frame);
                        free(master_frame);
                    }
                    slave_frame =pop_matched_frame_with_fd(&ctx->cam_busy_frame_list,msg->dsp_info.slave_fd);
                    if(slave_frame == NULL)
                    {
                        LOG_E("fail to find sending fd\n");
                    }else
                    {
                        csi_camera_put_frame(slave_frame);
                        free(slave_frame);
                    }
                    ctx->paried_frame_num++;
                    /***********done some thing for*********/
                    break;
            default:
                LOG_W("invalid msg type:%d\n",msg->type);
                break;
        }
        free(msg);
    }
    LOG_O("frame sync process exit....\n");
    pthread_exit(0);
}


/********************************Main prcoess *******************************************************************/
static int camera_get_chl_id(int env_type, int *chl_id)
{
	if (chl_id == NULL) {
		LOG_E("fail to check param chl_id = %p\n", chl_id);
		return -1;
	}
	switch (env_type) {
		case CSI_CAMERA_EVENT_TYPE_CHANNEL0:
			*chl_id = CSI_CAMERA_CHANNEL_0;
			break;
		case CSI_CAMERA_EVENT_TYPE_CHANNEL1:
			*chl_id = CSI_CAMERA_CHANNEL_1;
			break;
		case CSI_CAMERA_EVENT_TYPE_CHANNEL2:
			*chl_id = CSI_CAMERA_CHANNEL_2;
			break;
		default:
			LOG_D("fail to check env_type = %d unsupport\n", env_type);
			break;
	}
	return 0;
}
static int cam_send_frame_to_sync_process(msg_queue_ctx_t *queue,csi_frame_s *frame, cam_sync_message_type_e  type)
{
    camera_sync_msg_t *msg_palyload = malloc(sizeof(camera_sync_msg_t));
    msg_queue_item_t * item=NULL;
    if(msg_palyload==NULL)
    {
        return -1;
    }
    item = malloc(sizeof(msg_queue_item_t));
    if(item==NULL)
    {
        free(msg_palyload);
        return -1;
    }
    msg_palyload->type= type;
    memcpy(&msg_palyload->frame,frame,sizeof(csi_frame_s));
    item->payload = msg_palyload;
    return enqueue_msg(queue,item);

}
static int camera_event_process(void *arg)
{
	int ret = 0;
	if (arg == NULL) {
		LOG_E("NULL Ptr\n");
        return -1;
	}
	camera_ctx_t* ctx = (camera_ctx_t*)arg;
	csi_cam_event_handle_t ev_handle = ctx->event_handle;
    csi_camera_channel_cfg_s  *ch_cfg=NULL;
    cam_sync_message_type_e  type;
    type = ctx->cam_id==0?CAM_SYNC_MSG_MAIN_FRAME:CAM_SYNC_MSG_SLAVE_FRAME;
	struct timeval cur_time;
	struct csi_camera_event event;
	csi_frame_s frame;

	if (ev_handle == NULL) {
		LOG_E("fail to get ev_handle ev_handle\n");
		return -1;
	}

    if(ctx->exit!=0)
    {
        return 1;
    }

    int timeout =0; // unit: ms, -1 means wait forever, or until error occurs
    ret = csi_camera_get_event(ev_handle, &event, timeout);
    if(ret)
    {
        return -1;
    }
    LOG_D("Camera_%d event.type = %d, event.id = %d\n",ctx->cam_id ,event.type, event.id);
    switch (event.type) {
    case CSI_CAMERA_EVENT_TYPE_CAMERA:
        switch (event.id) {
        case CSI_CAMERA_EVENT_ERROR:
            // do sth.
                LOG_E("get CAMERA EVENT CSI_CAMERA_EVENT_ERROR!\n");
                break;
        case CSI_CAMERA_EVENT_WARNING:
                LOG_W("get CAMERA EVENT CSI_CAMERA_EVENT_WRN,RC: %s\n",event.bin);
                break;
        default:
            break;
        }
        break;
    case CSI_CAMERA_EVENT_TYPE_CHANNEL0:
    case CSI_CAMERA_EVENT_TYPE_CHANNEL1:
    case CSI_CAMERA_EVENT_TYPE_CHANNEL2:
    case CSI_CAMERA_EVENT_TYPE_CHANNEL3:
        switch (event.id) {
        case CSI_CAMERA_CHANNEL_EVENT_FRAME_READY: {
            int chn_id = 0;
            ret = camera_get_chl_id(event.type, &chn_id);
            if (ret) {
                LOG_E("fail to get chl_id = %d\n", chn_id);
                return -1;
            }
            ch_cfg = &ctx->chn_cfg[chn_id];
            get_system_time(__func__, __LINE__);
            int read_frame_count = csi_camera_get_frame_count(ctx->cam_handle,
                            chn_id);

            for (int i = 0; i < read_frame_count; i++) {
                csi_camera_get_frame(ctx->cam_handle, chn_id, &frame, timeout);
                ctx->frame_num++;
                if(cam_send_frame_to_sync_process(ctx->cam_sync_queue,&frame,type))
                {
                     csi_camera_put_frame(&frame);
                }
            }

            unsigned  long diff;
            if (ctx->init_time.tv_usec == 0)
            	gettimeofday(&ctx->init_time,0);//osGetTick();
            gettimeofday(&cur_time, 0);
            diff = 1000000 * (cur_time.tv_sec-ctx->init_time.tv_sec)+ cur_time.tv_usec-ctx->init_time.tv_usec;
            if (diff != 0)
            	ctx->fps = (float) ctx->frame_num / diff * 1000000.0f;
            LOG_O("cam:%d,read_frame_count = %d, frame_count = %d, fps = %.2f\n", ctx->cam_id,read_frame_count, ctx->frame_num,ctx->fps);

            break;
        }
        default:
            break;
        }
        break;
    default:
        break;
    }

	return 0;
}


static void camera_start_all(camera_ctx_t *  ctx)
{
    camera_ctx_t * cam_ctx = ctx;
    int loop_ch;
    for(loop_ch=0;loop_ch<cam_ctx->channel_num;loop_ch++)
    {
	    csi_camera_channel_start(cam_ctx->cam_handle, cam_ctx->chn_cfg[loop_ch].chn_id);
    }
}

static void camera_stop_all(camera_ctx_t *  ctx)
{
    camera_ctx_t * cam_ctx = ctx;
    int loop_ch;
    for(loop_ch=0;loop_ch<cam_ctx->channel_num;loop_ch++)
    {
	    csi_camera_channel_stop(cam_ctx->cam_handle, cam_ctx->chn_cfg[loop_ch].chn_id);
        LOG_I("Stop CAM:%d,channel:%d\n",cam_ctx->cam_id,loop_ch);
    }
}
static camera_ctx_t * camera_open(camera_param_t *params)
{
	int ret = 0;
	char dev_name[128];
    camera_ctx_t * cam_ctx = NULL;
    int loop_ch;
	LOG_O("Open Camera %d\n",params->video_id);
    if(params==NULL || params->video_id <0)
    {
        LOG_E("param err\n");
        return NULL;
    }
    if(params->channel_num> MAX_CHANNEL_NUM)
    {
        LOG_E("unsupoort channle num:%d \n",params->channel_num);
        return NULL;
    }


    cam_ctx = malloc(sizeof(camera_ctx_t));
    if(!cam_ctx)
    {
        return NULL;
    }
    memset(cam_ctx,0x0,sizeof(camera_ctx_t));

	// 打开Camera设备获取句柄，作为后续操对象
    sprintf(dev_name, "/dev/video%d", params->video_id);
	if(csi_camera_open(&cam_ctx->cam_handle, dev_name))
    {
        LOG_E("Fail to open cam :%s\n",dev_name);
        goto ONE_ERR; 
    }

    for(loop_ch= CSI_CAMERA_CHANNEL_0;loop_ch<params->channel_num;loop_ch++)
    {
        cam_ctx->chn_cfg[loop_ch].chn_id = loop_ch;
        cam_ctx->chn_cfg[loop_ch].img_fmt.pix_fmt = params->out_pic[loop_ch].fmt;
        cam_ctx->chn_cfg[loop_ch].img_fmt.width= params->out_pic[loop_ch].width;
        cam_ctx->chn_cfg[loop_ch].img_fmt.height = params->out_pic[loop_ch].height;
        cam_ctx->chn_cfg[loop_ch].img_type = CSI_IMG_TYPE_DMA_BUF;
        cam_ctx->chn_cfg[loop_ch].meta_fields = CSI_CAMERA_META_DEFAULT_FIELDS;
        if(csi_camera_channel_open(cam_ctx->cam_handle,&cam_ctx->chn_cfg[loop_ch]))
        {
            LOG_E("Fail to open cam %s,channel :%d\n",dev_name,loop_ch);
            goto TWO_ERR; 
        }

        cam_ctx->channel_num++;
    }

    if(csi_camera_create_event(&cam_ctx->event_handle,cam_ctx->cam_handle))
    {
            LOG_E("Fail to create event handler for cam %s\n",dev_name);
            goto TWO_ERR; 
    }

    cam_ctx->event_subscribe.type = CSI_CAMERA_EVENT_TYPE_CAMERA;
    cam_ctx->event_subscribe.id =  CSI_CAMERA_EVENT_WARNING | CSI_CAMERA_EVENT_ERROR;
    if(csi_camera_subscribe_event(cam_ctx->event_handle,&cam_ctx->event_subscribe))
    {
        LOG_E("Fail to subscribe eventfor cam %s\n",dev_name);
        goto TWO_ERR; 
    }


    for(loop_ch=0;loop_ch<cam_ctx->channel_num;loop_ch++)
    {
        cam_ctx->event_subscribe.type = CSI_CAMERA_EVENT_TYPE_CHANNEL0+loop_ch;
        cam_ctx->event_subscribe.id =  CSI_CAMERA_CHANNEL_EVENT_FRAME_READY;
        if(csi_camera_subscribe_event(cam_ctx->event_handle,&cam_ctx->event_subscribe))
        {
            LOG_E("Fail to subscribe eventfor cam %s\n",dev_name);
            goto TWO_ERR; 
        }
    }

	LOG_O("%s open successfully\n",dev_name);

    if(params->type == CAM_TYEP_SLAVE)
    {
    //         csi_camera_floodlight_led_set_flash_bright(cam_ctx->cam_handle, 500); //500ma
            csi_camera_projection_led_set_flash_bright(cam_ctx->cam_handle, 500); //500ma
            csi_camera_projection_led_set_mode(cam_ctx->cam_handle, LED_IR_ENABLE);
    //         csi_camera_floodlight_led_set_mode(cam_ctx->cam_handle, LED_IR_ENABLE);
            csi_camera_led_enable(cam_ctx->cam_handle, LED_PROJECTION);
            // csi_camera_led_set_switch_mode(cam_ctx->cam_handle, SWITCH_MODE_PROJECTION_ALWAYS_ON);
    }

	get_system_time(__func__, __LINE__);


    return cam_ctx;


TWO_ERR:
        for(loop_ch=0;loop_ch<cam_ctx->channel_num;loop_ch++)
        {
            csi_camera_channel_close(cam_ctx->cam_handle, cam_ctx->chn_cfg[loop_ch].chn_id);
        }
        csi_camera_close(cam_ctx->cam_handle);
ONE_ERR:
        free(cam_ctx);
        return NULL;
}

static void camera_close(camera_ctx_t *ctx)
{
    int loop_ch;
    if(ctx->cam_handle==NULL )
    {
        return;
    }

    csi_camera_unsubscribe_event(ctx->event_handle, &ctx->event_subscribe);

	csi_camera_destory_event(ctx->event_handle);

    for(loop_ch=0;loop_ch<ctx->channel_num;loop_ch++)
    {
        csi_camera_channel_close(ctx->cam_handle, ctx->chn_cfg[loop_ch].chn_id);
    }
    csi_camera_close(ctx->cam_handle);
    free(ctx);
}

cam_sync_process_ctx_t  *cam_sync_hdl;
dual_dsp_handle_t *dsp_hdl;

int main(int argc, char *argv[])
{

	char dev_name[128];
	int camera_id = 0;
	// 打印HAL接口版本号
	csi_api_version_u version;
	csi_camera_get_version(&version);
    camera_param_t params[MAX_CAM_NUM];
    camera_ctx_t * ctx[MAX_CAM_NUM]={NULL};
    frame_mode_t frame_mode ;
        dual_dsp_algo_type_e dsp_type;
    int cam_num=0;
    bool running = false;
 cam_num =parseParams(argc,argv,params,&frame_mode,&dsp_type);
    if(cam_num <=0 || cam_num>MAX_CAM_NUM)
    {
        LOG_E("not camera is active\n");
        exit(0);
    }

 
    dsp_hdl =dsp_process_create(frame_mode,dsp_type);
    if(dsp_hdl==NULL)
    {
        LOG_E("dsp create faile\n");
        exit(0);
    }
    cam_sync_hdl =cam_sync_create();
    if(cam_sync_hdl==NULL)
    {
        LOG_E("cam sync create fail\n");
        dsp_process_destroy(dsp_hdl);
        exit(0);
    }
   
    dsp_hdl->cam_sync_queue = &cam_sync_hdl->cam_sync_queue;
    cam_sync_hdl->dsp_prcoess_queue = &dsp_hdl->dsp_prcoess_queue;

    for(int i =cam_num-1;i>=0;i--)
    {
        ctx[i]=camera_open(&params[i]);
        if(ctx[i]==NULL)
        {
            LOG_E("camera %d open %dfail\n",i,params[i].type);
            goto ONR_ERR;
        }
        ctx[i]->cam_id = i;
        ctx[i]->frame_num =0;
        ctx[i]->cam_sync_queue =  &cam_sync_hdl->cam_sync_queue;
        if(params[i].type ==CAM_TYEP_MASTER )
        {
            cam_sync_hdl->master_cam_handle=ctx[i]->cam_handle;
        }
        else if(params[i].type ==CAM_TYEP_SLAVE)
        {
            cam_sync_hdl->slave_cam_handle = ctx[i]->cam_handle;
        }            
        camera_start_all(ctx[i]);
        running = true;
    }

    int all_exit=1;
    do{
        usleep(1000);
        all_exit=1;
        for(int i =cam_num-1;i>=0;i--)
        {
            
            camera_event_process(ctx[i]);
            if(ctx[i]&&
               ctx[i]->exit==0 && 
               params[i].frames_to_stop>0 && 
               cam_sync_hdl->paried_frame_num >=params[i].frames_to_stop)
            {
                camera_stop_all(ctx[i]);
                ctx[i]->exit=1;
            }
            if(ctx[i])
                all_exit &= ctx[i]->exit;
        }
        if(all_exit ==1)
        {
            LOG_O("All camera are stop\n");
            running = false;
        }

    }while(running);
    
ONR_ERR:
    for(int i =0;i<cam_num;i++)
    {
        if(ctx[i])
            camera_close(ctx[i]);
    }
    dsp_process_destroy(dsp_hdl);
    cam_sync_destroy(cam_sync_hdl);
    exit(0);
}

static void dump_camera_meta(csi_frame_s *frame)
{
	int i;
	//printf("%s\n", __func__);
	if (frame->meta.type != CSI_META_TYPE_CAMERA)
		return;

	csi_camera_meta_s *meta_data = (csi_camera_meta_s *)frame->meta.data;
	int meta_count = meta_data->count;
	csi_camrea_meta_unit_s meta_unit;


	csi_camera_frame_get_meta_unit(
		&meta_unit, meta_data, CSI_CAMERA_META_ID_FRAME_ID);
	LOG_I("meta_id=%d, meta_type=%d, meta_value=%d\n",
			meta_unit.id, meta_unit.type, meta_unit.int_value);
}


