/*
* SPDX-License-Identifier: GPL
*
* Copyright (c) 2020 ChangSha JingJiaMicro Electronics Co., Ltd.
* All rights reserved.
*
* Author:
*      shanjinkui <shanjinkui@jingjiamicro.com>
*
* The software and information contained herein is proprietary and
* confidential to JingJiaMicro Electronics. This software can only be
* used by JingJiaMicro Electronics Corporation. Any use, reproduction,
* or disclosure without the written permission of JingJiaMicro
* Electronics Corporation is strictly prohibited.
*/
#include <drm/drm_utils.h>
#include "mwv207_drm.h"
#include "mwv207_gem.h"
#include "mwv207_bo.h"

static int mwv207_gem_create(struct mwv207_device *jdev,
		u64 size, u64 align, u32 preferred_domain,
		u32 flags, struct drm_gem_object **gobj)
{
	struct mwv207_bo *jbo;
	int ret;

retry:

	ret = mwv207_bo_create(jdev, size, align, ttm_bo_type_device,
			preferred_domain, flags, &jbo);
	if (ret) {
		if (ret != -ERESTARTSYS) {
			if (flags & (1<<0)) {
				flags &= ~(1<<0);
				goto retry;
			}
			if (preferred_domain == 0x2) {
				preferred_domain |= 0x1;
				goto retry;
			}
			DRM_DEBUG("Failed to allocate GEM object (%lld, %d, %llu, %d)\n",
				  size, preferred_domain, align, ret);
		}
		return ret;
	}

	*gobj = mwv207_gem_from_bo(jbo);
	return 0;
}

int mwv207_gem_dumb_create(struct drm_file *file, struct drm_device *dev,
		     struct drm_mode_create_dumb *args)
{
	struct mwv207_device *jdev = dev->dev_private;
	struct drm_gem_object *gobj;
	u32 handle;
	int ret;

	args->pitch = ALIGN(args->width * DIV_ROUND_UP(args->bpp, 8), 64);
	args->size = args->pitch * args->height;
	ret = mwv207_gem_create(jdev, args->size, 0x10000,
			0x2,
			(1<<0),
			&gobj);
	if (ret)
		return ret;

	ret = drm_gem_handle_create(file, gobj, &handle);
	mwv207_gem_object_put(gobj);
	if (ret)
		return ret;

	args->handle = handle;
	return 0;
}

void mwv207_gem_free_object(struct drm_gem_object *gobj)
{
	struct mwv207_bo *jbo;

	if (!gobj)
		return;

	jbo = mwv207_bo_from_gem(gobj);
	mwv207_bo_unref(jbo);
}

int  mwv207_gem_create_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *filp)
{
	struct mwv207_device *jdev = dev->dev_private;
	union drm_mwv207_gem_create *args = data;
	struct drm_gem_object *gobj;
	int ret;

	if (args->in.size == 0)
		return -EINVAL;
	if (args->in.alignment & (args->in.alignment - 1))
		return -EINVAL;
	if (args->in.preferred_domain & ~0x7)
		return -EINVAL;
	if (args->in.flags & ~((1<<0)|(1<<1)|(1<<2)))
		return -EINVAL;

	ret = mwv207_gem_create(jdev, args->in.size, args->in.alignment,
			args->in.preferred_domain,
			args->in.flags, &gobj);
	if (ret)
		return ret;

	ret = drm_gem_handle_create(filp, gobj, &args->out.handle);
	mwv207_gem_object_put(gobj);

	return ret;
}

int  mwv207_gem_mmap_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *filp)
{
	union drm_mwv207_gem_mmap *args = data;
	struct drm_gem_object *obj;

	if (args->in.pad)
		return -EINVAL;

	obj = drm_gem_object_lookup(filp, args->in.handle);
	if (!obj)
		return -ENOENT;
	args->out.offset = mwv207_bo_mmap_offset(obj);
	mwv207_gem_object_put(obj);

	return 0;
}

int  mwv207_gem_wait_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *filp)
{
	struct drm_mwv207_gem_wait *args = (struct drm_mwv207_gem_wait *)data;
	long timeout;
	bool write;
	int ret;

	if (args->op & ~(0x00000002 | 0x00000001))
		return -EINVAL;

	write = args->op & 0x00000002;
	timeout = drm_timeout_abs_to_jiffies(args->timeout);

	ret = drm_gem_dma_resv_wait(filp, args->handle, write, timeout);
	if (ret == -ETIME)
		ret = timeout ? -ETIMEDOUT : -EBUSY;

	return ret;
}
