/*
 * Copyright (C) 2021-2022 Alibaba Group Holding Limited
 * Author: LuChongzhi <chongzhi.lcz@alibaba-inc.com>; Minxiong Tian <minxiong.tmx@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <error.h>
#include <errno.h>

#include <sys/mman.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include "csi_camera_dev_api.h"

/* macros for share memory and semaphore */
#define SHMKEY (key_t)0x39126
#define SEMKEY (key_t)0x49126
#define IFLAGS (IPC_CREAT|IPC_EXCL)

#define LOG_LEVEL 2
#define LOG_PREFIX "camera_demo1"
#include <syslog.h>

#include <csi_frame_ex.h>
#include <csi_camera.h>

#ifdef PLATFORM_SIMULATOR
#include "apputilities.h"
#endif

#define PAGE_SIZE 1024
#define PAGE_ALIGN(addr)   (void *)(((int)(addr)+PAGE_SIZE-1)&~( PAGE_SIZE-1))

/* data store in share-memory */
struct databuf {
    int d_buf[2];
};

static void dump_camera_meta(csi_frame_ex_s *frame, int idx);
static void* save_camera_img(void *);
static void* save_camera_stream(void *);
static int set_properties(csi_cam_handle_t cam_handle, char *pArgs);
static enum csi_pixel_fmt parse_format(char *fmt);
// static void handle_dma_buf(csi_frame_s *framei, int cid);
//extern int csi_camera_frame_unlock(csi_cam_handle_t cam_handle, csi_frame_s *frame);
extern int csi_camera_put_frame(csi_frame_ex_s *frame);
static void i_save(csi_frame_ex_s *framei, int cid, FILE *fp, int vmode);
static void parse_fmt_res(int cindx);
static void getseg(struct databuf* *pdata);
static int getsem();
static void remove_s();
static void writer(struct databuf *buf, int index, int value);
static void reader(struct databuf *buf, int index, int * rv);
static int pr_error(char *mess);
extern int csi_camera_set_pp_path_param(csi_cam_handle_t cam_handle, uint16_t line_num,uint16_t buf_mode);

extern void *vi_plink_create(csi_camera_channel_cfg_s *chn_cfg);
extern void vi_plink_release(void * plink);
extern void display_camera_frame(void * plink, csi_frame_ex_s *frame);

#define TEST_DEVICE_NAME "/dev/video0"
#define CSI_CAMERA_TRUE  1
#define CSI_CAMERA_FALSE 0
#define chnMAX 8
#define fmtMAX 19

csi_cam_handle_t cam_handle;
char *prog_name;
int Soff = 0;
static char device[CSI_CAMERA_NAME_MAX_LEN];    /* target video device like /dev/video0 */
static char path[128] = "./";    /* output path */
static int channelIdx[chnMAX];    /* channel information like 8 */
enum csi_pixel_fmt fenum[chnMAX];
int fid[chnMAX] = {0};
int tframes = 4;
csi_camera_channel_id_e CAMERA_CHANNEL_ID[chnMAX];

/* File local Global array to store information which will be used when storig images */
static int PLANE[chnMAX] = {0};
static int PBIT[chnMAX] = {0};
static int PLNS[chnMAX][3] = {0};
static int LNN[chnMAX][3] = {0};
static int LSZ[chnMAX][3] = {0};
static int LNNV[chnMAX][3] = {0}; /* for rotation 90/270 */
static int LSZV[chnMAX][3] = {0}; /* for rotation 90/270 */
static int NOSTRD[chnMAX] = {0};

/* local global array for meta data verify */
static int cur_metaint[chnMAX] = {-1, -1, -1, -1};
static int pre_metaint[chnMAX] = {-1, -1, -1, -1};
static int cur_metasec[chnMAX] = {-1, -1, -1, -1};
static int pre_metasec[chnMAX] = {-1, -1, -1, -1};
static int cur_metausec[chnMAX] = {-1, -1, -1, -1};
static int pre_metausec[chnMAX] = {-1, -1, -1, -1};

static int shmid, semid;
struct sembuf p1 = {0, -1, 0}, v1 = {0, 1, 0};

int meta_mode = -1;
int flashMode = 0;
int fNumInc[chnMAX] = {0};
int fCount = 1;
int onoffMode = 0;
int afterOnOff[chnMAX];
int onoffDrop = 4;
int onoffFrame = 0;
int ooNum[chnMAX] = {0};
int ofTop = 30;
csi_frame_ex_s frame[chnMAX];
pthread_t tid[chnMAX]={0};
int hres[chnMAX], vres[chnMAX];    /* resolution information like 640x480 */
int idxArray[chnMAX][10]={0};
static char *dname;
static int chNum = 0;
char fmtArray[fmtMAX][50] = {
    "CSI_PIX_FMT_I420",
    "CSI_PIX_FMT_NV12",
    "CSI_PIX_FMT_BGR",
    "CSI_PIX_FMT_RAW_8BIT",
    "CSI_PIX_FMT_RAW_10BIT",
    "CSI_PIX_FMT_RAW_12BIT",
    "CSI_PIX_FMT_RAW_14BIT",
    "CSI_PIX_FMT_RAW_16BIT",
    "CSI_PIX_FMT_RGB_PLANAR_888",
    "CSI_PIX_FMT_RGB_INTEVLEAVED_888",
    "CSI_PIX_FMT_YUV_PLANAR_422",
    "CSI_PIX_FMT_YUV_PLANAR_420",
    "CSI_PIX_FMT_YUV_PLANAR_444",
    "CSI_PIX_FMT_YUV_SEMIPLANAR_422",
    "CSI_PIX_FMT_YUV_SEMIPLANAR_420",
    "CSI_PIX_FMT_YUV_SEMIPLANAR_444",
    "CSI_PIX_FMT_YUV_TEVLEAVED_422",
    "CSI_PIX_FMT_YUV_TEVLEAVED_420",
    "CSI_PIX_FMT_YUV_TEVLEAVED_444"
};

void usage()
{
	fprintf (stderr,
			"usage: %s [options]\n"
			"Options:\n"
			"\t-D dev		target video device like /dev/video0\n"
			"\t-R resolution        format is like 640x480, use 640x480/1092x1080 for multi channel output\n"
			"\t-F format		like NV12, use NV12/NV12 to support multi channel output\n"
			"\t-C channel_index		channel index defined in dts, index from 0, use 0/2 for multi channel output\n"
			"\t-M test mode		0: save as image; 1: save as stream file; 2: output to plink; 3. performace test\n"
			"\t-N number for frames		0: record forever; others: the recorded frame numbers, use 30/30 from multi channel output\n"
			"\t-n number of frames to save 	if not given, 4 is used\n"
			"\t-P Property		id:type:value, use id:type:value/id:type:value for multi property setting\n"
			"\t-p output path        store output file in the give path\n"
			"\t-S turn off stride mode        if given, turn off stride mode\n"
			"\t-t stream turn on/off test        if will turn off stream after several streams then turn on\n"
			"\t-s turn off sram mode        if given, turn off sram mode between ISP/DSP and DSP/ISP-RY\n"
			"\t-m disable DW DMA-BUF         if given, turn off dma-buf mode\n"
			"\t-f enable LED flash mode         if given, enable LED flash mode for odd and even frame feature in lightA production board\n"
			"\t-i nubmer of process which needs to do IPC, use for multi-video and mutli-process test scenarios\n"
			"\t-y meta-data test            0 - meta dump all all elements; 1 - meta verify for CAMERA_NAME;\n"
            "                                2 - meta verify for CHANNEL_ID; 3 - meta verify for FRAME_ID;\n"
            "                                4 - meta verify for TIMESTAMP.\n"
            "                                200 - meta verify for all elements\n"
			, prog_name
		);
}

int main(int argc, char *argv[])
{
	bool running = 0;
	csi_camera_info_s camera_info;

	int opt;
	char *resDelim = "xX/";
	int outMode = 0;
	int fNum[chnMAX] = {0};
	char pArgs[500] = {'\0'};    /* store property args */
	char *devDelim = "/";
    char *sCur;
    int idx = 0;
    int ret = 0;
	csi_camera_event_type_e CAMERA_CHANNEL_EVENT_TYPE[chnMAX];
	struct timeval init_time, cur_time;
	memset(&init_time, 0, sizeof(init_time));
	memset(&cur_time, 0, sizeof(cur_time));
    unsigned long video_open = 0;
    unsigned long channel_open = 0;
    unsigned long channel_start = 0;
    unsigned long first_frame = 0;
    unsigned long total_frame = 0;
    float fps = 0.0;
    int sramMode = 1;

    /* data for share-memory */
    struct databuf *buf;
    int mi = 0;
    int srv1 = 0, srv2 = 0;

	prog_name = argv[0];

	while ((opt = getopt(argc, argv, "t:D:R:F:C:M:hN:P:p:Smi:y:fsn:")) != EOF) {
		switch(opt) {
			case 'D':
				strcpy(device, optarg);
				strtok(optarg, devDelim);
				dname=strtok(NULL, devDelim);
				continue;
			case 'p':
				strcpy(path, optarg);
				continue;
			case 'S':
				Soff = 1;
				continue;
			case 't':
				onoffMode = 1;
				onoffFrame = atoi(optarg);
				continue;
			case 's':
				sramMode = 0;
			case 'f':
				flashMode = 1;
				continue;
			case 'P':
				strcpy(pArgs, optarg);
				continue;
			case 'R':
                idx = 0;
                sCur=strtok(optarg, resDelim);
                while (sCur != NULL){
                    if (idx % 2 == 0)
                        hres[idx/2] =  atoi(sCur);
                    if (idx % 2 == 1)
                        vres[idx/2] =  atoi(sCur);
                    sCur=strtok(NULL, resDelim);
                    idx++;
                }
				continue;
			case 'M':
				outMode = atoi(optarg);
				continue;
			case 'i':
				mi = atoi(optarg);
				continue;
			case 'y':
				meta_mode = atoi(optarg);
				continue;
			case 'N':
                for(int j=0; j<chnMAX; j++)
				    fNum[j] = atoi(optarg);
				continue;
			case 'n':
				tframes = atoi(optarg);
				continue;
			case 'F':
                idx = 0;
                sCur=strtok(optarg, devDelim);
                while (sCur != NULL){
                    fenum[idx] = parse_format(sCur);
                    sCur=strtok(NULL, devDelim);
                    idx++;
                }
				continue;
			case 'C':
                idx = 0;
                sCur=strtok(optarg, devDelim);
                while (sCur != NULL){
                    channelIdx[idx] =  atoi(sCur);
                    sCur=strtok(NULL, devDelim);
                    idx++;
                }
                chNum = idx;
				continue;
			case '?':
			case 'h':
			default:
				usage();
				return 1;
		}
	}

    /* get semaphore ID and share-memory ID if mi >=2 */
    if(mi >= 2){
        semid = getsem();
        getseg(&buf);
        printf("------IPC mode enable - finish-------\n");
    }

    /* init the target channel ID and channel event type array */
    for (int j=0; j<chNum; j++){
        CAMERA_CHANNEL_ID[j] =  CSI_CAMERA_CHANNEL_0 + channelIdx[j];
        CAMERA_CHANNEL_EVENT_TYPE[j] =  CSI_CAMERA_EVENT_TYPE_CHANNEL0 + channelIdx[j];
    }

	if (fNum[0] == 0){
		running = 1;
        printf("------running = 1-------\n");
	}

	// 打印HAL接口版本号
	csi_api_version_u version;
	csi_camera_get_version(&version);
	printf("Camera HAL version: %d.%d\n", version.major, version.minor);

	// 获取设备中，所有的Camera
	struct csi_camera_infos camera_infos;
	memset(&camera_infos, 0, sizeof(camera_infos));
	csi_camera_query_list(&camera_infos);

	// 打印所有设备所支持的Camera
	for (int i = 0; i < camera_infos.count; i++) {
		camera_info = camera_infos.info[i];
		printf("Camera[%d]: camera_name='%s', device_name='%s', bus_info='%s', capabilities=0x%08x\n",
				i,
				camera_info.camera_name, camera_info.device_name, camera_info.bus_info,
				camera_info.capabilities);
		printf("Camera[%d] caps are:\n",
				i); /*  The caps are: Video capture, metadata capture */
		for (int j = 1; j <= 0x08000000; j = j << 1) {
			switch (camera_info.capabilities & j) {
				case CSI_CAMERA_CAP_VIDEO_CAPTURE:
					printf("\t camera_infos.info[%d]:Video capture,\n", i);
					break;
				case CSI_CAMERA_CAP_META_CAPTURE:
					printf("\t camera_infos.info[%d] metadata capture,\n", i);
					break;
				default:
					if (camera_info.capabilities & j) {
						printf("\t camera_infos.info[%d] unknown capabilities(0x%08x)\n", i,
								camera_info.capabilities & j);
					}
					break;
			}
		}
	}


	// 打开Camera设备获取句柄，作为后续操对象
	strcpy(camera_info.device_name, device);
	gettimeofday(&init_time, 0);
	csi_camera_open(&cam_handle, camera_info.device_name);
	gettimeofday(&cur_time, 0);
	video_open = 1000000 * (cur_time.tv_sec-init_time.tv_sec)+ cur_time.tv_usec-init_time.tv_usec;
	// 获取Camera支持的工作模式
	struct csi_camera_modes camera_modes;
	csi_camera_get_modes(cam_handle, &camera_modes);

	// 打印camera所支持的所有工作模式
	printf("Camera:'%s' modes are:\n", device);
	printf("{\n");
	for (int i = 0; i < camera_modes.count; i++) {
		printf("\t mode_id=%d: description:'%s'\n",
				camera_modes.modes[i].mode_id, camera_modes.modes[i].description);
	}
	printf("}\n");


	// 设置camera的工作模式及其配置
	csi_camera_mode_cfg_s camera_cfg;
	camera_cfg.mode_id = 1;
	camera_cfg.calibriation = NULL; // 采用系统默认配置
	camera_cfg.lib3a = NULL;	// 采用系统默认配置
	csi_camera_set_mode(cam_handle, &camera_cfg);

	// 获取单个可控单元的属性
	csi_camera_property_description_s description;

	// 轮询获取所有可控制的单元

	printf("all properties are:\n");
	description.id = CSI_CAMERA_PID_HFLIP;
	while (!csi_camera_query_property(cam_handle, &description)) {
		switch (description.type) {
			case (CSI_CAMERA_PROPERTY_TYPE_INTEGER):
				printf("id=0x%08x type=%d default=%d value=%d\n",
						description.id, description.type,
						description.default_value.int_value, description.value.int_value);
				break;
			case (CSI_CAMERA_PROPERTY_TYPE_BOOLEAN):
				printf("id=0x%08x type=%d default=%d value=%d\n",
						description.id, description.type,
						description.default_value.bool_value, description.value.bool_value);
				break;
			case (CSI_CAMERA_PROPERTY_TYPE_ENUM):
				printf("id=0x%08x type=%d default=%d value=%d\n",
						description.id, description.type,
						description.default_value.enum_value, description.value.enum_value);
				break;
			case (CSI_CAMERA_PROPERTY_TYPE_STRING):
				printf("id=0x%08x type=%d default=%s value=%s\n",
						description.id, description.type,
						description.default_value.str_value, description.value.str_value);
				break;
			case (CSI_CAMERA_PROPERTY_TYPE_BITMASK):
				printf("id=0x%08x type=%d default=%x value=%x\n",
						description.id, description.type,
						description.default_value.bitmask_value, description.value.bitmask_value);
				break;
			default:
				LOG_E("error type!\n");
				break;
		}
		description.id |= CSI_CAMERA_FLAG_NEXT_CTRL;
	}

    if (pArgs[0] != '\0'){
        set_properties(cam_handle, pArgs);
    }

	// 查询输出channel
    csi_camera_channel_cfg_s chn_cfg[chnMAX];
    for (int j=0; j<chNum; j++){
        chn_cfg[j].chn_id = CAMERA_CHANNEL_ID[j];
        csi_camera_channel_query(cam_handle, &chn_cfg[j]);
        if (chn_cfg[j].status != CSI_CAMERA_CHANNEL_CLOSED) {
            printf("Can't open channel: %d\n", CAMERA_CHANNEL_ID[j]);
            exit(-1);
        }
    }

	// 打开输出channel
    if (sramMode == 1){
        csi_camera_set_pp_path_param(cam_handle,0,1);
    }
	csi_cam_event_handle_t event_handle;
	csi_camera_event_subscription_s subscribe;
    for (int j=0; j<chNum; j++){
        chn_cfg[j].chn_id = CAMERA_CHANNEL_ID[j];
        chn_cfg[j].frm_cnt = fCount;
        chn_cfg[j].img_fmt.width = hres[j];
        chn_cfg[j].img_fmt.height = vres[j];
        chn_cfg[j].img_fmt.pix_fmt = fenum[j];
        chn_cfg[j].img_type = CSI_IMG_TYPE_DMA_BUF;
        chn_cfg[j].meta_fields = CSI_CAMERA_META_DEFAULT_FIELDS;
        chn_cfg[j].capture_type = CSI_CAMERA_CHANNEL_CAPTURE_VIDEO |
            CSI_CAMERA_CHANNEL_CAPTURE_META;
        parse_fmt_res(j);
        gettimeofday(&init_time, 0);
        ret = csi_camera_channel_open(cam_handle, &chn_cfg[j]);
        if (ret) {
            exit(-1);
        }
        gettimeofday(&cur_time, 0);
        channel_open = 1000000 * (cur_time.tv_sec-init_time.tv_sec)+ cur_time.tv_usec-init_time.tv_usec;
    }

        // 订阅Event
    csi_camera_create_event(&event_handle, cam_handle);
    for (int j=0; j<chNum; j++){
        subscribe.type =
            CSI_CAMERA_EVENT_TYPE_CAMERA;      // 订阅Camera的ERROR事件
        subscribe.id = CSI_CAMERA_EVENT_WARNING | CSI_CAMERA_EVENT_ERROR;
        csi_camera_subscribe_event(event_handle, &subscribe);
        subscribe.type =
            CAMERA_CHANNEL_EVENT_TYPE[j];    // 订阅Channel0的FRAME_READY事件
        subscribe.id = CSI_CAMERA_CHANNEL_EVENT_FRAME_READY |
            CSI_CAMERA_CHANNEL_EVENT_OVERFLOW;
        csi_camera_subscribe_event(event_handle, &subscribe);
    }

    if ( flashMode == 1){
        csi_camera_floodlight_led_set_flash_bright(cam_handle, 500); //500ma
        csi_camera_projection_led_set_flash_bright(cam_handle, 500); //500ma
        csi_camera_projection_led_set_mode(cam_handle, LED_IR_ENABLE);
        csi_camera_floodlight_led_set_mode(cam_handle, LED_IR_ENABLE);
        csi_camera_led_enable(cam_handle, LED_FLOODLIGHT_PROJECTION);
    }
	// start all channels
    for (int j=0; j<chNum; j++){
        gettimeofday(&init_time, 0);
	    csi_camera_channel_start(cam_handle, CAMERA_CHANNEL_ID[j]);
        gettimeofday(&cur_time, 0);
        channel_start = 1000000 * (cur_time.tv_sec-init_time.tv_sec)+ cur_time.tv_usec-init_time.tv_usec;
    }
    gettimeofday(&init_time, 0);

	// 处理订阅的Event
	struct csi_camera_event event;
    int fnumTotal = fNum[0] * chNum;
    int frameTotal = fnumTotal;

	while (running || fnumTotal > 0 ) {
		int timeout = -1; // unit: ms, -1 means wait forever, or until error occurs
		csi_camera_get_event(event_handle, &event, timeout);
		printf("%s event.type = %d, event.id = %d\n", __func__, event.type, event.id);
		if(event.type == CSI_CAMERA_EVENT_TYPE_CAMERA){
			switch (event.id) {
				case CSI_CAMERA_EVENT_ERROR:
					// do sth.
					printf("-------get CAMERA EVENT CSI_CAMERA_EVENT_ERROR!----------\n");
					break;
                case CSI_CAMERA_EVENT_WARNING:
            		printf("-------get CAMERA EVENT CSI_CAMERA_EVENT_WRN,RC: %s-------\n",event.bin);
				    break;
				default:
					break;
			}
		}
        for (int j=0; j<chNum; j++){
            if(event.type == CAMERA_CHANNEL_EVENT_TYPE[j]){
                switch (event.id) {
                    case CSI_CAMERA_CHANNEL_EVENT_OVERFLOW:
                        printf("-------get channel EVENT CSI_CAMERA_CHANNEL_EVENT_OVERFLOW-------\n");
                        break;
                    case CSI_CAMERA_CHANNEL_EVENT_FRAME_READY: {
                           int read_frame_count = csi_camera_get_frame_count(cam_handle,
                                   CAMERA_CHANNEL_ID[j]);
                           printf("-------read_frame_count=%d-------\n", read_frame_count);
                           for (int i = 0; i < fCount; i++) {
                               if (tid[j] != 0){
                                    /* wait for previous image save thread end */
                                    pthread_join(tid[j], NULL);
                               }
                               csi_camera_get_frame(cam_handle, CAMERA_CHANNEL_ID[j], &frame[j], timeout);
                               if (afterOnOff[j] != 0 ){
                               /* drop the first event after on/off operation */
                                   afterOnOff[j]--;
                                   csi_camera_put_frame(&frame[j]);
                                //    csi_frame_release(&frame[j]);
                                   continue;
                               }
                               fNum[j]--;
                               fNumInc[j]++;
                               if (fNum[j] < 0 && running == 0 ){
                                   //csi_camera_frame_unlock(cam_handle, &frame[j]);
                                   csi_camera_put_frame(&frame[j]);
                                //    csi_frame_release(&frame[j]);
                                   continue;
                               }

    #ifdef PLATFORM_SIMULATOR
                               show_frame_image(frame[j].img.usr_addr[0], frame[j].img.height, frame.img.width);
    #endif
                               fid[j]++;
                               fnumTotal--;
                               if (outMode == 0){
                                   int * tIdx = malloc(sizeof(int));
                                   *tIdx = j;
                                   /* submit thread to save image for current channel then we can go to handle other channel */
                                   pthread_create(&tid[j], NULL, save_camera_img, (void *)tIdx);
                               }
                               else if (outMode == 1){
                                   int * tIdx = malloc(sizeof(int));
                                   *tIdx = j;
                                   pthread_create(&tid[j], NULL, save_camera_stream, (void *)tIdx);
                                }
                                else if(outMode == 3){
                                   /* mode 3 only do performance test, no actual image saving */
                                   gettimeofday(&cur_time, 0);
                                   if (fid[j] == 1){
                                       /* first frame */
                                        first_frame = 1000000 * (cur_time.tv_sec-init_time.tv_sec)+ cur_time.tv_usec-init_time.tv_usec + video_open + channel_open + channel_start;
                                   }
                                   if (fnumTotal == 0){
                                       /* last frame */
                                        total_frame = 1000000 * (cur_time.tv_sec-init_time.tv_sec)+ cur_time.tv_usec-init_time.tv_usec;
                                        fps = (float)frameTotal / total_frame * 1000000.0f;
                                        printf("\n");
                                        printf("\n");
                                        printf("Performance data:\n");
                                        printf("video_open:%lu ms\n", video_open/1000);
                                        printf("channel_open:%lu ms\n", channel_open/1000);
                                        printf("channel_start:%lu ms\n", channel_start/1000);
                                        printf("first_frame:%lu ms\n", first_frame/1000);
                                        printf("fps:%f fps\n", fps);
                                        printf("\n");
                                        printf("\n");
                                   }
                                   /* for mode 3, unlock in this branch, for mode 0,1 unlock in thread */
                                   //csi_camera_frame_unlock(cam_handle, &frame[j]);
                                   csi_camera_put_frame(&frame[j]);
                                 
                                }
                                else{
                                   chn_cfg[j].chn_id = j;
                                   csi_camera_channel_query(cam_handle, &chn_cfg[j]);
                                //    display_camera_frame(cam_handle, &chn_cfg[j], &frame[j]);
                                   //csi_camera_frame_unlock(cam_handle, &frame[j]);
                                   csi_camera_put_frame(&frame[j]);
                                 
                                }
                           }
                           break;
                       }
                    default:
                           break;
                }
            }
	    }
    }
    /* for IPC, update finish status to share-memory */
    if(mi >= 2){
        writer(buf, 0, 1);
    }
    for (int j=0; j<chNum; j++){
        /* for IPC only do stop when share memory value equals mi */
        if(mi >= 2){
            while (1){
                reader(buf, 0, &srv1);
                if (srv1 != mi){
                    sleep(2);
                }
                else{
                    /* update the second value in share-memory which is used to decide whether to remove share-memory */
                    writer(buf, 1, 1);
                    reader(buf, 1, &srv2);
                    if (srv2 == mi){
                        /* I am the last process to update the buf[1], remove the share-memory */
                        remove_s();
                        printf("------IPC mode: remove-------\n");
                    }
                    break;
                }
            }
        }
	    csi_camera_channel_stop(cam_handle, CAMERA_CHANNEL_ID[j]);
    }
    if ( flashMode == 1){
        csi_camera_led_disable(cam_handle, LED_FLOODLIGHT_PROJECTION);
    }
	usleep (1000000);
	// 取消订阅某一个event, 也可以直接调用csi_camera_destory_event，结束所有的订阅
    for (int j=0; j<chNum; j++){
        subscribe.type = CAMERA_CHANNEL_EVENT_TYPE[j];
        subscribe.id = CSI_CAMERA_CHANNEL_EVENT_FRAME_READY;
        csi_camera_unsubscribe_event(event_handle, &subscribe);
    }

	csi_camera_destory_event(event_handle);

    for (int j=0; j<chNum; j++){
	    csi_camera_channel_close(cam_handle, CAMERA_CHANNEL_ID[j]);
    }
	csi_camera_close(cam_handle);
}

int pr_error(char *mess){
    perror(mess);
    exit(1);
}

/* function: get or create share-memory*/
void getseg(struct databuf* *pdata){
    /* try to get share-memory */
    if((shmid = shmget(SHMKEY, sizeof(struct databuf), 0600 | IFLAGS)) < 0){
        if(errno == EEXIST){
            /* exist, return its ID */
            shmid = shmget(SHMKEY, sizeof(struct databuf), 0600 | IPC_CREAT);
            if(shmid < 0){
                pr_error("--------share-memory getting: error!!!!!---------");
            }
        }
        else {
            pr_error("--------share-memory getting: other error!!!!!---------");
        }
    }
    if((*pdata = (struct databuf *)(shmat(shmid, 0, 0))) < 0 ){
        /* mapping share-memory */
        pr_error("--------share-memory mapping error!!!!!---------");
    }
}

/* function: get semaphore */
int getsem(){
    /* try to get or create semaphore */
    if((semid = semget(SEMKEY, 1, 0600|IFLAGS)) < 0){
        if(errno ==  EEXIST){
            /* exist, reurn its ID */
            semid = semget(SEMKEY, 1, 0600 | IPC_CREAT);
            if(semid < 0){
                pr_error("--------semaphore getting ID: error!!!!!---------");
            }
            else{
                /* semaphore already exsit and return ID directly */
                return semid;
            }
        }
        else{
            pr_error("--------semaphore getting: error!!!!!---------");
        }
    }
    /* do semaphore init to 1 once just after semaphore creating */
    if(semctl(semid, 0, SETVAL, 1) < 0){
        pr_error("-----------semctl-----------");
    }
    return semid;
}

/* function remove share-memory and semaphore */
void remove_s(){
    if(shmctl(shmid, IPC_RMID, NULL) < 0)
        pr_error("-----------shmctl when removing-----------");
    if(semctl(semid, 0, IPC_RMID) < 0)
        pr_error("-----------semctl when removing-----------");
}

/* function writer to share-memory */
void writer(struct databuf *buf, int index, int value){
    /* get semaphore before update share-memory */
    semop(semid, &p1, 1);
    /* update share-memory */
    buf->d_buf[index] += value;
    /* release semaphore */
    semop(semid, &v1, 1);
}

/* function read to share-memory */
void reader(struct databuf *buf, int index, int *rv){
    /* get semaphore before update share-memory */
    semop(semid, &p1, 1);
    /* get share-memory value */
    *rv = buf->d_buf[index];
    /* release semaphore */
    semop(semid, &v1, 1);
}

static void dump_camera_meta(csi_frame_ex_s *frame, int idx)
{
	int i;
	//printf("%s\n", __func__);
	if (frame->frame_meta.type != CSI_META_TYPE_CAMERA)
		return;

	csi_camera_meta_s *meta_data = (csi_camera_meta_s *)frame->frame_meta.data;
	int meta_count = meta_data->count;
	csi_camera_meta_unit_s meta_unit;
    csi_camera_meta_id_e meta_id;

    if(meta_mode > 0){
        meta_id = (1<<(meta_mode - 1));
        csi_camera_frame_get_meta_unit(
                        &meta_unit, meta_data, meta_id);
    }
    if(meta_mode == 1 || meta_mode == 2){
        /* camera_name and channel_id - which should keep same during testing */
        cur_metaint[idx] = meta_unit.int_value;
        if(pre_metaint[idx] == -1){
            pre_metaint[idx] = cur_metaint[idx];
        }
        else{
            if(cur_metaint[idx] != pre_metaint[idx]){
                pr_error("-----meta test fail: camera_id or channel_id changes during test--------");
            }
            pre_metaint[idx] = cur_metaint[idx];
            printf("-----meta test camera_name or channel_id:%d--------\n", cur_metaint[idx]);
        }
    }
    else if(meta_mode == 3){
        /* frame id - which should increase from 0 */
        if(cur_metaint[idx] == -1 && meta_unit.int_value != 1){
            pr_error("-----meta test fail: first frame ID is not 1--------\n");
        }
        cur_metaint[idx] = meta_unit.int_value;
        if(pre_metaint[idx] == -1){
            pre_metaint[idx] = cur_metaint[idx];
        }
        else{
            if(cur_metaint[idx] != (pre_metaint[idx]+1)){
                pr_error("-----meta test fail: frame_id is not continuous--------\n");
            }
            printf("-----meta test: meta_id: %d, frame_id:%d, previous_id: %d--------\n", meta_unit.id, cur_metaint[idx], pre_metaint[idx]);
            pre_metaint[idx] = cur_metaint[idx];
        }
    }
    else if(meta_mode == 4){
        /* timestamps - which should be larger or equal to previous */
        cur_metasec[idx] = meta_unit.time_value.tv_sec;
        cur_metausec[idx] = meta_unit.time_value.tv_usec;
        if(pre_metasec[idx] == -1){
            pre_metasec[idx] = cur_metasec[idx];
            pre_metausec[idx] = cur_metausec[idx];
        }
        else{
            if(cur_metasec[idx] <= pre_metasec[idx] && cur_metausec[idx] <= pre_metausec[idx]){
                pr_error("-----meta test fail: timestamp decrease--------");
            }
            printf("-----meta test time - second:%d--------\n", cur_metasec[idx]);
            printf("-----meta test time - usecond:%d--------\n", cur_metausec[idx]);
            pre_metasec[idx] = cur_metasec[idx];
            pre_metausec[idx] = cur_metausec[idx];
        }
    }
    else{
        /* means meta_mode = 0 */
        for (i = 0; i < meta_count; i++) {
            meta_id = (1<<i);
            csi_camera_frame_get_meta_unit(
                    &meta_unit, meta_data, meta_id);
            printf("meta_id=%d, meta_type=%d, meta_value=%d\n",
                meta_unit.id, meta_unit.type, meta_unit.int_value);
        }
    }
}

/*
function: parse the format and resolution to generate the output information used to save img
Input args: index of channel will be tested whose order is decided when running
Below information should be included:
1. how may planars which will be store in global int PLANE[] array
2. the pix bit  which will be store in global int PBIT[] array
3. the bytes of each plane for the give channel to store in global int PLNS[][3] array
4. the line numbers of each plane for the give channel to store in global int LNN[][3] array
5. the bytes to store for each line size in each plane to store in  global int LSZ[][3] array
*/
static void parse_fmt_res(int cindx){
    /* fetch the format the resolution information */
    enum csi_pixel_fmt fmt = fenum[cindx];
    int iw = hres[cindx];
    int ih = vres[cindx];
    /* decide the pixel bit number */
	switch(fmt) {
        case CSI_PIX_FMT_RAW_10BIT:
            PBIT[cindx] = 10;
			break;
        case CSI_PIX_FMT_RAW_12BIT:
            PBIT[cindx] = 12;
			break;
        case CSI_PIX_FMT_RAW_14BIT:
            PBIT[cindx] = 14;
			break;
        case CSI_PIX_FMT_RAW_16BIT:
            PBIT[cindx] = 16;
			break;
		default:
            PBIT[cindx] = 8;
			break;
    }
    /* decide plane numbers, each plane size(byte) and line and line size*/
	switch(fmt) {
        case CSI_PIX_FMT_RAW_8BIT:
            PLNS[cindx][0] = iw * ih;
            LNN[cindx][0] = ih;
            LSZ[cindx][0] = iw * 2 ;
            LNNV[cindx][0] = iw;
            LSZV[cindx][0] = ih * 2 ;
            PLANE[cindx] = 1;
            NOSTRD[cindx] = 0;
			break;
        case CSI_PIX_FMT_RAW_10BIT:
        case CSI_PIX_FMT_RAW_12BIT:
        case CSI_PIX_FMT_RAW_14BIT:
        case CSI_PIX_FMT_RAW_16BIT:
            PLNS[cindx][0] = iw * ih * 2;
            LNN[cindx][0] = ih;
            LSZ[cindx][0] = iw * 2 ;
            LNNV[cindx][0] = iw;
            LSZV[cindx][0] = ih * 2 ;
            PLANE[cindx] = 1;
            NOSTRD[cindx] = 0;
			break;
        case CSI_PIX_FMT_RGB_INTEVLEAVED_888:
        case CSI_PIX_FMT_YUV_TEVLEAVED_444:
            PLNS[cindx][0] = (iw * ih * PBIT[cindx] / 8) * 3;
            LNN[cindx][0] = ih;
            LSZ[cindx][0] = (iw * PBIT[cindx] / 8) * 3;
            LNNV[cindx][0] = iw;
            LSZV[cindx][0] = (ih * PBIT[cindx] / 8) * 3;
            PLANE[cindx] = 1;
			break;
        case CSI_PIX_FMT_YUV_TEVLEAVED_422:
            PLNS[cindx][0] = (iw * ih * PBIT[cindx] / 8) * 2;
            LNN[cindx][0] = ih;
            LSZ[cindx][0] = (iw * PBIT[cindx] / 8) * 2;
            LNNV[cindx][0] = iw;
            LSZV[cindx][0] = (ih * PBIT[cindx] / 8) * 2;
            PLANE[cindx] = 1;
            break;
        case CSI_PIX_FMT_YUV_TEVLEAVED_420:
            PLNS[cindx][0] = ((iw * ih * PBIT[cindx] / 8) * 3) / 2;
            LNN[cindx][0] = ih;
            LSZ[cindx][0] = ((iw * PBIT[cindx] / 8) * 3) / 2;
            LNNV[cindx][0] = iw;
            LSZV[cindx][0] = ((ih * PBIT[cindx] / 8) * 3) / 2;
            PLANE[cindx] = 1;
			break;
        case CSI_PIX_FMT_BGR:
        case CSI_PIX_FMT_RGB_PLANAR_888:
        case CSI_PIX_FMT_YUV_PLANAR_444:
            PLNS[cindx][0] = (iw * ih * PBIT[cindx] / 8);
            PLNS[cindx][1] = (iw * ih * PBIT[cindx] / 8);
            PLNS[cindx][2] = (iw * ih * PBIT[cindx] / 8);
            LNN[cindx][0] = ih;
            LNN[cindx][1] = ih;
            LNN[cindx][2] = ih;
            LSZ[cindx][0] = LSZ[cindx][1] = LSZ[cindx][2] = (iw * PBIT[cindx] / 8);
            LNNV[cindx][0] = iw;
            LNNV[cindx][1] = iw;
            LNNV[cindx][2] = iw;
            LSZV[cindx][0] = LSZV[cindx][1] = LSZV[cindx][2] = (ih * PBIT[cindx] / 8);
            PLANE[cindx] = 3;
			break;
        case CSI_PIX_FMT_YUV_PLANAR_422:
            PLNS[cindx][0] = (iw * ih * PBIT[cindx] / 8);
            PLNS[cindx][1] = (iw * ih * PBIT[cindx] / 8) / 2;
            PLNS[cindx][2] = (iw * ih * PBIT[cindx] / 8) / 2;
            LNN[cindx][0] = ih;
            LNN[cindx][1] = ih / 2;
            LNN[cindx][2] = ih / 2;
            LSZ[cindx][0] = (iw * PBIT[cindx] / 8);
            LSZ[cindx][1] = LSZ[cindx][2] = (iw * PBIT[cindx] / 8) / 2;
            LNNV[cindx][0] = iw;
            LNNV[cindx][1] = iw /2;
            LNNV[cindx][2] = iw /2;
            LSZV[cindx][0] = (ih * PBIT[cindx] / 8);
            LSZV[cindx][1] = LSZV[cindx][2] = (ih * PBIT[cindx] / 8) / 2;
            PLANE[cindx] = 3;
			break;
        case CSI_PIX_FMT_YUV_PLANAR_420:
            PLNS[cindx][0] = (iw * ih * PBIT[cindx] / 8);
            PLNS[cindx][1] = PLNS[cindx][2] = (iw * ih * PBIT[cindx] / 8) / 4;
            LNN[cindx][0] = ih;
            LNN[cindx][1] = LNN[cindx][2] = ih / 4;
            LSZ[cindx][0] = (iw * PBIT[cindx] / 8);
            LSZ[cindx][1] = LSZ[cindx][2] = (iw * PBIT[cindx] / 8) / 4;
            LNNV[cindx][0] = iw;
            LNNV[cindx][1] = LNNV[cindx][2] = iw / 4;
            LSZV[cindx][0] = (ih * PBIT[cindx] / 8);
            LSZV[cindx][1] = LSZV[cindx][2] = (ih * PBIT[cindx] / 8) / 4;
            PLANE[cindx] = 3;
			break;
        case CSI_PIX_FMT_I420:
        case CSI_PIX_FMT_NV12:
        case CSI_PIX_FMT_YUV_SEMIPLANAR_420:
            PLNS[cindx][0] = (iw * ih * PBIT[cindx] / 8);
            PLNS[cindx][1] = (iw * ih * PBIT[cindx] / 8) / 2;
            LNN[cindx][0] = ih;
            LNN[cindx][1] = ih / 2;
            LSZ[cindx][0] = LSZ[cindx][1] = (iw * PBIT[cindx] / 8);
            LNNV[cindx][0] = iw;
            LNNV[cindx][1] = iw / 2;
            LSZV[cindx][0] = LSZV[cindx][1] = (ih * PBIT[cindx] / 8);
            PLANE[cindx] = 2;
			break;
        case CSI_PIX_FMT_YUV_SEMIPLANAR_422:
            PLNS[cindx][0] = PLNS[cindx][1] = (iw * ih * PBIT[cindx] / 8);
            LNN[cindx][0] = ih;
            LNN[cindx][1] = ih;
            LSZ[cindx][0] = LSZ[cindx][0] = (iw * PBIT[cindx] / 8);
            LNNV[cindx][0] = iw;
            LNNV[cindx][1] = iw;
            LSZV[cindx][0] = LSZV[cindx][0] = (ih * PBIT[cindx] / 8);
            PLANE[cindx] = 2;
			break;
        case CSI_PIX_FMT_YUV_SEMIPLANAR_444:
            PLNS[cindx][0] = (iw * ih * PBIT[cindx] / 8);
            PLNS[cindx][1] = (iw * ih * PBIT[cindx] / 8) * 2;
            LNN[cindx][0] = ih;
            LNN[cindx][1] = ih * 2;
            LSZ[cindx][0] = (iw * PBIT[cindx] / 8);
            LSZ[cindx][1] = (iw * PBIT[cindx] / 8);
            LNNV[cindx][0] = iw;
            LNNV[cindx][1] = iw * 2;
            LSZV[cindx][0] = (ih * PBIT[cindx] / 8);
            LSZV[cindx][1] = (ih * PBIT[cindx] / 8);
            PLANE[cindx] = 2;
			break;
		default:
            PLNS[cindx][0] = (iw * ih * PBIT[cindx] / 8);
            PLNS[cindx][1] = (iw * ih * PBIT[cindx] / 8) / 2;
            LNN[cindx][0] = ih;
            LNN[cindx][1] = ih / 2;
            LSZ[cindx][0] = LSZ[cindx][1] = (iw * PBIT[cindx] / 8);
            LNNV[cindx][0] = iw;
            LNNV[cindx][1] = iw / 2;
            LSZV[cindx][0] = LSZV[cindx][1] = (ih * PBIT[cindx] / 8);
            PLANE[cindx] = 2;
			break;
    }
}

/*
function: store image of each frame
input1: frame
input2: channel index
input3: target file point
input4: reverse mdoe for rotating 90 and 270 degree
*/
static void i_save(csi_frame_ex_s *framei, int cid, FILE *fp, int vmode)
{
    int lindex = 0;
    int smode = 0;
    if( vmode == 0){
        if(framei->frame_data.stride[0] != LSZ[cid][0] && NOSTRD[cid] == 0){
        //if(NOSTRD[cid] == 0){
            smode = 1;
            printf("-------------In Stride mode, framei->img.strides[0]=%d---------------\n", framei->frame_data.stride[0]);
        }
        else{
            printf("-------------No Stride, framei->img.strides[0]=%d---------------\n", framei->frame_data.stride[0]);
        }
        for (int p = 0; p < PLANE[cid]; p++){
            /* loop for each planar */
            //printf("-------------framei->img.usr_addr[%d]=0x%llx---------------\n", p, framei->img.usr_addr[p]);
            lindex = 0;
            //int size = 0;
            if(smode == 0){
                /* no stride, write plane by plane*/
            printf("-------------No Stride, length is = %d---------------\n", PLNS[cid][p]);
                //while(size < PLNS[cid][p]){
                 //   size += fwrite(framei->img.usr_addr[p] + size, sizeof(char), PLNS[cid][p] - size, fp);
               // }
                    fwrite(framei->frame_data.vir_addr[p], sizeof(char), PLNS[cid][p], fp);
            }
            else{
                /* stride mode, write line by line */
                for(int l = 0; l < LNN[cid][p]; l++){
                    if(framei->frame_data.stride[p] == 0){
                        printf("-------------Error: stride[%d] is 0---------------\n", p);
                        lindex = LSZ[cid][p] * l;
                    }
                    else{
                        /* for each plane, the strides[x] is the same */
                        lindex = framei->frame_data.stride[0] * l;
                    }
                    fwrite(framei->frame_data.vir_addr[p] + lindex, sizeof(char), LSZ[cid][p], fp);
                }
            }
        }
    }
    else{
        if(framei->frame_data.stride[0] != LSZV[cid][0] && NOSTRD[cid] == 0){
            smode = 1;
            printf("-------------In Stride mode, framei->img.strides[0]=%d---------------\n", framei->frame_data.stride[0]);
        }
        else{
            printf("-------------No Stride, framei->img.strides[0]=%d---------------\n", framei->frame_data.stride[0]);
        }
        for (int p = 0; p < PLANE[cid]; p++){
            /* loop for each planar */
            //printf("-------------framei->img.usr_addr[%d]=0x%llx---------------\n", p, framei->img.usr_addr[p]);
            lindex = 0;
            int size = 0;
            if(smode == 0){
                /* no stride, write plane by plane*/
            printf("-------------No Stride, length is = %d---------------\n", PLNS[cid][p]);
                while(size < PLNS[cid][p]){
                    size += fwrite(framei->frame_data.vir_addr[p] + size, sizeof(char), PLNS[cid][p] - size, fp);
                }
            }
            else{
                /* stride mode, write line by line */
                for(int l = 0; l < LNNV[cid][p]; l++){
                    if(framei->frame_data.stride[p] == 0){
                        printf("-------------Error: stride[%d] is 0---------------\n", p);
                        lindex = LSZV[cid][p] * l;
                    }
                    else{
                        /* for each plane, the strides[x] is the same */
                        lindex = framei->frame_data.stride[0] * l;
                    }
                    fwrite(framei->frame_data.vir_addr[p] + lindex, sizeof(char), LSZV[cid][p], fp);
                }
            }
        }
    }
}

/*
function: convert dma-buf file point to target address
input1: framei
input2: channel index
*/
// static void handle_dma_buf(csi_frame_ex_s *framei, int cid)
// {
//     if (frame->img.type == CSI_IMG_TYPE_DMA_BUF) {
//         void *p[3] = {0};
//         /*
//            The first plane which exists always.
//            All data for packed; Y data for planar or semi-planar
//         */
//         void *phyaddr = vi_mem_import(framei->img.dmabuf[0].fds);
//         p[0] = vi_mem_map(phyaddr) + framei->img.dmabuf[0].offset;
//         framei->img.usr_addr[0] = p[0];
//         /*
//            For other planes, in our solution, we only have plane[1] for
//            planar or sp; or no other plane for packed.
//         */
//         for (int i=1; i<PLANE[cid]; i++){
//             if (framei->img.dmabuf[i].offset != NULL){
//                 p[i] = framei->img.dmabuf[i].offset + p[0];
//                 framei->img.usr_addr[i] = p[i];
//             }
//             else{
//                 printf("-------------DMA ERROR, no usr_addr for plane=%d---------------\n", i);
//             }
//         }
//         vi_mem_release(phyaddr);
//     }
// }

/*
function: save data as image
input1: channel index
*/
static void* save_camera_img(void * idx)
{
    int index = *(int*)(idx);
    free(idx);
	char fname[512];
	char fidname[256];
	FILE *fp;
	char fidc[128];
    int fcount;
    int fidi = fid[index];
    int cid = channelIdx[index];
    int hresi = hres[index];
    int vresi = vres[index];
    enum csi_pixel_fmt fmt = fenum[index];
    csi_frame_ex_s *framei = &frame[index];
    int vmode = 0;

    if(framei->frame_info.width == vresi && framei->frame_info.height == hresi && vresi != hresi){
        vmode = 1;
        vresi = framei->frame_info.height;
        hresi = framei->frame_info.width;
    }

    /* generate the image name */
	fcount = fidi%tframes;
	sprintf(fname, "%s%s%s%s%d%s%d%s%d%s%d%s%d%s%s", path, "/img_", dname, "_", PLANE[index], "P_", cid, "_", hresi, "x", vresi, "_", fcount, ".", fmtArray[fmt]);
    /* dma-buf to address */
    // handle_dma_buf(framei, index);
	if((fp = fopen(fname, "wb")) == NULL){
		printf("Error: Can't open file\n");
		return NULL;
	}
    /* data saving */
    i_save(framei, index, fp, vmode);
    fclose(fp);

    /* frame ID saving */
    sprintf(fidc, "%d\n", fidi);
    sprintf(fidname, "%s%s%s%s%d%s", path, "/fid_", dname, "_chn_", cid, ".txt");
    if((fp = fopen(fidname, "a+")) == NULL){
        printf("Error: Can't open file\n");
        return NULL;
    }
    fwrite(fidc, sizeof(char), strlen(fidc), fp);
    fclose(fp);
    if(meta_mode >= 0){
        dump_camera_meta(framei, index);
    }
    //csi_camera_frame_unlock(cam_handle, framei);
    csi_camera_put_frame(framei);
    // csi_frame_release(framei);
    if (onoffMode == 1 && fNumInc[index] % onoffFrame == 0){
        afterOnOff[index] = onoffDrop;
        // stop stream, sleep, start stream
        csi_camera_channel_stop(cam_handle, CAMERA_CHANNEL_ID[index]);
        //sleep(1);
        csi_camera_channel_start(cam_handle, CAMERA_CHANNEL_ID[index]);
    }
    return NULL;
}

/*
function: save data as stream
input1: channel index
*/
static void* save_camera_stream(void *idx)
{
    int index = *(int*)(idx);

    free(idx);
    FILE *fp;
    char fname[256];
    int cid = channelIdx[index];
    int hresi = hres[index];
    int vresi = vres[index];
    enum csi_pixel_fmt fmt = fenum[index];
    csi_frame_ex_s *framei = &frame[index];
    int vmode = 0;

    if(framei->frame_info.width == vresi && framei->frame_info.height == hresi && vresi != hresi){
        vmode = 1;
        vresi = framei->frame_info.height;
        hresi = framei->frame_info.width;
    }

    /* generate stream name - for each time it is the same name */
	sprintf(fname, "%s%s%s%s%d%s%d%s%d%s%d%s%s", path, "/str_", dname, "_", PLANE[index], "P_", cid, "_", hresi, "x", vresi, ".", fmtArray[fmt]);
    // handle_dma_buf(framei, index);
    if((fp = fopen(fname, "ab+")) == NULL){
        printf("Error: Can't open file\n");
        return NULL;
    }
    i_save(framei, index, fp, vmode);
    fclose(fp);
    if(meta_mode >= 0){
        dump_camera_meta(framei, index);
    }

    //csi_camera_frame_unlock(cam_handle, framei);
    csi_camera_put_frame(framei);
    // csi_frame_release(framei);
    if (onoffMode == 1 && fNumInc[index] % onoffFrame == 0){
        afterOnOff[index] = onoffDrop;
        // stop stream, sleep, start stream
        csi_camera_channel_stop(cam_handle, CAMERA_CHANNEL_ID[index]);
        //sleep(1);
        csi_camera_channel_start(cam_handle, CAMERA_CHANNEL_ID[index]);
    }
    return NULL;
}

// 同时配置多个参数
static int set_properties(csi_cam_handle_t cam_handle, char *pArgs)
{
    csi_camera_properties_s properties;
    csi_camera_property_s property[30];

    char *pDelim=":/";
    int pid;
    int ptype;
    int pvalueInt;
    char *pvalueStr;
    char *pFirst;
    /*
    parse multi-properties which delimited by : or /
    1. / is used to seprate multi properties
    2. : is used to sperate pid:type:value
    for example: 1:2:1/2:2:1/3:1:90
    */
    pFirst = strtok(pArgs, pDelim);
    int i = 0;
    while(pFirst != NULL){
        pid = atoi(pFirst);
        property[i].id = pid + CSI_CAMERA_PID_BASE;
        /*
           property type difinition:
           CSI_CAMERA_PROPERTY_TYPE_INTEGER    = 1,
           CSI_CAMERA_PROPERTY_TYPE_BOOLEAN    = 2,
           CSI_CAMERA_PROPERTY_TYPE_ENUM       = 3,
           CSI_CAMERA_PROPERTY_TYPE_STRING     = 7,
           CSI_CAMERA_PROPERTY_TYPE_BITMASK    = 8,
         */
        ptype = atoi(strtok(NULL, pDelim));
        property[i].type = ptype;
        if (ptype == CSI_CAMERA_PROPERTY_TYPE_STRING) {
            pvalueStr = strtok(NULL, pDelim);
            strcpy(property[i].value.str_value, pvalueStr);
        }
        else {
            pvalueInt = atoi(strtok(NULL, pDelim));
            property[i].value.int_value = pvalueInt;
        }
        i++;
        pFirst = strtok(NULL, pDelim);
    }

    /* set multi-properties(here the property number is i) by calling API just one time */
    properties.count = i;
    properties.property = property;
    if (csi_camera_set_property(cam_handle, &properties) < 0) {
        printf("set_property fail!\n");
        return -1;
    }
    printf("set_property ok!\n");
    return 0;
}

static enum csi_pixel_fmt parse_format(char *fmt)
{
    for(int i=0; i<fmtMAX; i++){
        if(strstr(fmtArray[i], fmt)){
            return CSI_PIX_FMT_I420 + i;
        }
    }
    return CSI_PIX_FMT_NV12;
}
