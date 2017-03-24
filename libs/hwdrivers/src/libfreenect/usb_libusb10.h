/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef USB_LIBUSB10
#define USB_LIBUSB10

#include <libusb-1.0/libusb.h>

#if defined(__APPLE__)
/*
  From Github Issue 22 by Roefer -
  https://github.com/OpenKinect/libfreenect/issues/#issue/22

  The current implementation still does not reach 30 Hz on MacOS. This
  is due to bad scheduling of USB transfers in libusb (Ed Note: libusb
  1.0.8). A fix can be found at
  http://www.informatik.uni-bremen.de/~roefer/libusb/libusb-osx-kinect.diff

  (Ed Note: patch applies to libusb repo at 7da756e09fd)

  In camera.c, I use PKTS_PER_XFER = 128, NUM_XFERS = 4. There are a
  few rules: PKTS_PER_XFER * NUM_XFERS <= 1000, PKTS_PER_XFER % 8 == 0.
*/
#define PKTS_PER_XFER 128
#define NUM_XFERS 4
#define DEPTH_PKTBUF 2048
#define VIDEO_PKTBUF 2048
#else
#ifdef _WIN32
  #define PKTS_PER_XFER 32
  #define NUM_XFERS 8
#else
  #define PKTS_PER_XFER 16
  #define NUM_XFERS 16
#endif
#define DEPTH_PKTBUF 1920
#define VIDEO_PKTBUF 1920
#endif

typedef struct {
	libusb_context *ctx;
	int should_free_ctx;
} fnusb_ctx;

typedef struct {
	freenect_device *parent; //so we can go up from the libusb userdata
	libusb_device_handle *dev;
} fnusb_dev;

typedef struct {
	fnusb_dev *parent; //so we can go up from the libusb userdata
	struct libusb_transfer **xfers;
	uint8_t *buffer;
	fnusb_iso_cb cb;
	int num_xfers;
	int pkts;
	int len;
	int dead;
	int dead_xfers;
} fnusb_isoc_stream;

int fnusb_num_devices(fnusb_ctx *ctx);

int fnusb_init(fnusb_ctx *ctx, freenect_usb_context *usb_ctx);
int fnusb_shutdown(fnusb_ctx *ctx);
int fnusb_process_events(fnusb_ctx *ctx);

int fnusb_open_subdevices(freenect_device *dev, int index);
int fnusb_close_subdevices(freenect_device *dev);

int fnusb_start_iso(fnusb_dev *dev, fnusb_isoc_stream *strm, fnusb_iso_cb cb, int ep, int xfers, int pkts, int len);
int fnusb_stop_iso(fnusb_dev *dev, fnusb_isoc_stream *strm);

int fnusb_control(fnusb_dev *dev, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint8_t *data, uint16_t wLength);


#endif
