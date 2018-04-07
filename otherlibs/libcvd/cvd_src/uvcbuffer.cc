#include <cvd/video/uvcbuffer.h>

using namespace std;

namespace CVD{

namespace Exceptions{ namespace UVCBuffer{
	Init::Init(string, string err)
	{
		what = "UVCBuffer: failed to initialize: " + err;
	}


	DeviceOpen::DeviceOpen(string dev, string err)
	{
		what = "UVCBuffer: failed to open \"" + dev + "\": "  + err;
	}

	FindDevice::FindDevice(string dev, string err)
	{
		what = "UVCBuffer: failed to find device \"" + dev + "\": "  + err;
	}

	DeviceSetup::DeviceSetup(string dev, string err)
	{
		what = "UVCBuffer: failed to setup device \"" + dev + "\": "  + err;
	}

	GetFrame::GetFrame(string dev, string err)
	{
		what = "UVCBuffer: failed to get frame on \"" + dev + "\": "  + err;
	}

	PutFrame::PutFrame(string dev, string err)
	{
		what = "UVCBuffer: failed to put frame on \"" + dev + "\": "  + err;
	}

	StreamOpen::StreamOpen(string dev, string err)
	{
		what = "UVCBuffer: failed open stream on \"" + dev + "\": "  + err;
	}
	StreamStart::StreamStart(string dev, string err)
	{
		what = "UVCBuffer: failed start stream on \"" + dev + "\": "  + err;
	}
}}


namespace UVC{
	
	ImageRef RawUVCBuffer::getSize()
	{
		return size;
	}

	double RawUVCBuffer::getRate()
	{
		return rate;
	}	

	const std::string & RawUVCBuffer::device_name() const 
	{ 
		return dev_str; 
	}

	

	void RawUVCBuffer::fill_frame(void* data)
	{
		uvc_frame_t to;
		to.data = data;

		uvc_frame_t* from=NULL;

		//This appears to get a frame which points to an 
		//internal structure. So, we duplicate the data.
		//In theory, we could make a proper ringbuffer, but
		//there seems to be no real need any more.
		//
		//Random number is a 1 second timeout, btw!
		uvc_error e = uvc_stream_get_frame(strmh, &from, 1000000);

		if(e != UVC_SUCCESS)
			throw Exceptions::UVCBuffer::GetFrame(dev_str, uvc_strerror(e));

		//If we're capturing RGB, then there might be a conversion 
		//involved (i.e. from MJPEG)
		if(format == UVC_FRAME_FORMAT_RGB)
			uvc_any2rgb(from, &to);
		else
			uvc_duplicate_frame(from, &to);
	}


	RawUVCBuffer::RawUVCBuffer(const std::string& sn_, unsigned int fmt, ImageRef size_, double frame_per_second, bool mjpeg, bool verbose)
	:dev_str(sn_),
	 size(size_),
	 format(fmt),
	 capture_mjpeg(mjpeg),
	 rate(frame_per_second)
	{
		/* Initialize a UVC service context. Libuvc will set up its own libusb
		 * context. Replace NULL with a libusb_context pointer to run libuvc
		 * from an existing libusb context. */
		uvc_error res = uvc_init(&ctx, NULL);

		if (res < 0) 
			throw Exceptions::UVCBuffer::Init(dev_str, uvc_strerror(res));
		
		const char* str = nullptr;
		if(dev_str != "NULL" && dev_str != "/dev/null" && dev_str != "")
			str = dev_str.c_str();

		res = uvc_find_device(ctx, &dev, 0, 0, str);

		if (res < 0) 
			throw Exceptions::UVCBuffer::FindDevice(dev_str, uvc_strerror(res));

		/* Try to open the device: requires exclusive access */
		res = uvc_open(dev, &devh);
		if (res < 0) 
			throw Exceptions::UVCBuffer::DeviceOpen(dev_str, uvc_strerror(res));

		if(verbose)
			uvc_print_diag(devh, stderr);
		

		if(mjpeg && fmt != UVC_FRAME_FORMAT_RGB)
			throw Exceptions::UVCBuffer::DeviceSetup(dev_str, "MJPEG capture only valid for RGB");

		int f= fmt;
		if(mjpeg)
			f= UVC_FRAME_FORMAT_MJPEG;
		
		//f = UVC_FRAME_FORMAT_MJPEG;
		/* Try to negotiate a 640x480 30 fps YUYV stream profile */
		res = uvc_get_stream_ctrl_format_size(
				devh, &ctrl, /* result stored in ctrl */
				(uvc_frame_format) f, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
				size.x, size.y, frame_per_second /* width, height, fps */
				);
		
		if(res < 0)
			throw Exceptions::UVCBuffer::DeviceSetup(dev_str, uvc_strerror(res));

		res = uvc_stream_open_ctrl(devh, &strmh, &ctrl);
		if(res < 0)
			throw Exceptions::UVCBuffer::StreamOpen(dev_str, uvc_strerror(res));

		
		/*Start stream with no callback, and no callback data */
		res = uvc_stream_start(strmh, NULL, NULL, 0);
		if(res < 0)
			throw Exceptions::UVCBuffer::StreamStart(dev_str, uvc_strerror(res));


	}





	
	RawUVCBuffer::~RawUVCBuffer()
	{
		/* Release our handle on the device */
		if(devh)
		{
			uvc_stop_streaming(devh);
			uvc_close(devh);
			devh=nullptr;
		}

		/* Release the device descriptor */
		if(dev)
		{
			uvc_unref_device(dev);
			dev=nullptr;
		}

		/* Close the UVC context. This closes and cleans up any existing device handles,
		 * and it closes the libusb context if one was not provided. */
		if(ctx)
		{
			uvc_exit(ctx);
			ctx=nullptr;
		}

	}





}}
