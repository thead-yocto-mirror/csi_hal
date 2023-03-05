/*
 * Copyright (C) 2021 Alibaba Group Holding Limited
 * Author: LuChongzhi <chongzhi.lcz@alibaba-inc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CSI_META_H__
#define __CSI_META_H__

#include <stddef.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef enum csi_meta_type {
	CSI_META_TYPE_SYSTEM,
	CSI_META_TYPE_CAMERA,
	CSI_META_TYPE_VDEC,
	CSI_META_TYPE_GPU,
	CSI_META_TYPE_G2D,
} csi_meta_type_e;
typedef enum csi_meta_value_type {
	CSI_META_VALUE_TYPE_BOOL,
	CSI_META_VALUE_TYPE_INT,
	CSI_META_VALUE_TYPE_UINT,
	CSI_META_VALUE_TYPE_STR,
	CSI_META_VALUE_TYPE_TIMEVAL,
} csi_meta_value_type_e;

typedef struct csi_meta {
	csi_meta_type_e  type;
	size_t           size;
	void            *data;
} csi_meta_s;

#ifdef  __cplusplus
}
#endif

#endif /* __CSI_META_H__ */
