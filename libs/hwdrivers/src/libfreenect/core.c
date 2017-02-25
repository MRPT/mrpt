/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <unistd.h>

#include "freenect_internal.h"

FREENECTAPI int freenect_init(freenect_context **ctx, freenect_usb_context *usb_ctx)
{
	*ctx = (freenect_context*)malloc(sizeof(freenect_context));
	if (!ctx)
		return -1;

	memset(*ctx, 0, sizeof(freenect_context));

	(*ctx)->log_level = LL_WARNING;
	return fnusb_init(&(*ctx)->usb, usb_ctx);
}

FREENECTAPI int freenect_shutdown(freenect_context *ctx)
{
	while (ctx->first) {
		FN_NOTICE("Device %p open during shutdown, closing...\n", (void*)ctx->first);
		freenect_close_device(ctx->first);
	}

	fnusb_shutdown(&ctx->usb);
	free(ctx);
	return 0;
}

FREENECTAPI int freenect_process_events(freenect_context *ctx)
{
	return fnusb_process_events(&ctx->usb);
}

FREENECTAPI int freenect_num_devices(freenect_context *ctx)
{
	return fnusb_num_devices(&ctx->usb);
}

FREENECTAPI int freenect_open_device(freenect_context *ctx, freenect_device **dev, int index)
{
	int res;
	freenect_device *pdev = (freenect_device*)malloc(sizeof(freenect_device));
	if (!pdev)
		return -1;

	memset(pdev, 0, sizeof(*pdev));

	pdev->parent = ctx;

	res = fnusb_open_subdevices(pdev, index);

	if (res < 0) {
		free(pdev);
		return res;
	}

	if (!ctx->first) {
		ctx->first = pdev;
	} else {
		freenect_device *prev = ctx->first;
		while (prev->next)
			prev = prev->next;
		prev->next = pdev;
	}

	*dev = pdev;
	return 0;
}

FREENECTAPI int freenect_close_device(freenect_device *dev)
{
	freenect_context *ctx = dev->parent;
	int res;

	// stop streams, if active
	freenect_stop_depth(dev);
	freenect_stop_video(dev);

	res = fnusb_close_subdevices(dev);
	if (res < 0) {
		FN_ERROR("fnusb_close_subdevices failed: %d\n", res);
		return res;
	}

	freenect_device *last = NULL;
	freenect_device *cur = ctx->first;

	while (cur && cur != dev) {
		last = cur;
		cur = cur->next;
	}

	if (!cur) {
		FN_ERROR("device %p not found in linked list for this context!\n", (void*)dev);
		return -1;
	}

	if (last)
		last->next = cur->next;
	else
		ctx->first = cur->next;

	free(dev);
	return 0;
}

FREENECTAPI void freenect_set_user(freenect_device *dev, void *user)
{
	dev->user_data = user;
}

FREENECTAPI void *freenect_get_user(freenect_device *dev)
{
	return dev->user_data;
}

FREENECTAPI void freenect_set_log_level(freenect_context *ctx, freenect_loglevel level)
{
	ctx->log_level = level;
}

FREENECTAPI void freenect_set_log_callback(freenect_context *ctx, freenect_log_cb cb)
{
	ctx->log_cb = cb;
}

void fn_log(freenect_context *ctx, freenect_loglevel level, const char *fmt, ...)
{
	va_list ap;

	if (level > ctx->log_level)
		return;

	if (ctx->log_cb) {
		char msgbuf[1024];

		va_start(ap, fmt);
		vsnprintf(msgbuf, 1024, fmt, ap);
		msgbuf[1023] = 0;
		va_end(ap);

		ctx->log_cb(ctx, level, msgbuf);
	} else {
		va_start(ap, fmt);
		vfprintf(stderr, fmt, ap);
		va_end(ap);
	}
}
