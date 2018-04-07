//-*- c++ -*-
#ifndef CVD_VIDEO_UVCBUFFER_H
#define CVD_VIDEO_UVCBUFFER_H

#include <libuvc/libuvc.h>
#include <cvd/videobuffer.h>
#include <cvd/localvideoframe.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/colourspaces.h>

namespace CVD {

namespace Exceptions
{
	/// @ingroup gException
	namespace UVCBuffer
	{
		/// @ingroup gException
		struct All: public CVD::Exceptions::VideoBuffer::All
		{
		};

		/// Error opening the device
		/// @ingroup gException
		struct Init: public All {Init(std::string dev, std::string err); ///< Construct from the device name
		};

		/// Error opening the device
		/// @ingroup gException
		struct DeviceOpen: public All {DeviceOpen(std::string dev, std::string err); ///< Construct from the device name
		};

		/// Error opening the device
		/// @ingroup gException
		struct StreamOpen: public All {StreamOpen(std::string dev, std::string err); ///< Construct from the device name
		};

		/// Error opening the device
		/// @ingroup gException
		struct StreamStart: public All {StreamStart(std::string dev, std::string err); ///< Construct from the device name
		};

		/// Error opening the device
		/// @ingroup gException
		struct FindDevice: public All {FindDevice(std::string dev, std::string err); ///< Construct from the device name
		};

		/// Error setting up the device
		/// @ingroup gException
		struct DeviceSetup: public All {DeviceSetup(std::string dev, std::string action);  ///< Construct from the device string and an error string
		};
		/// Error in a put_frame() call
		/// @ingroup gException
		struct PutFrame: public All {PutFrame(std::string dev, std::string msg); ///< Construct from the device name
		};
		/// Error in a get_frame() call
		/// @ingroup gException
		struct GetFrame: public All {GetFrame(std::string dev, std::string msg); ///< Construct from the device name
		};
	}
}

namespace UVC
{
#ifndef DOXYGEN_IGNORE_INTERNAL
	template<class C> struct format;

	//libUVC only supports a rather small subset of the possible
	//types, but these seem to be the only common ones anyway.

	template<> struct format<yuv422>
	{
		static const unsigned int fmt = UVC_FRAME_FORMAT_YUYV;
	};

	template<> struct format<Rgb<byte> >
	{
		static const unsigned int fmt = UVC_FRAME_FORMAT_RGB;
	};

#endif
	
	class RawUVCBuffer: public virtual RawVideoBuffer
	{
		public:

			RawUVCBuffer(const std::string& dev, unsigned int fmt, ImageRef size, double frame_per_second, bool mjpeg, bool verbose);
			ImageRef getSize();
			void fill_frame(void*);
			double getRate();
			virtual ~RawUVCBuffer();
			const std::string & device_name() const;

			RawUVCBuffer(const RawUVCBuffer&) = delete;
			void operator=(const RawUVCBuffer&) = delete;

		private:
			std::string dev_str;
			uvc_context_t *ctx=0;
			uvc_device_t *dev=0;
			uvc_device_handle_t *devh=0;
			uvc_stream_ctrl_t ctrl={};
			uvc_stream_handle_t *strmh=0;
			ImageRef size;
			int format;
			bool capture_mjpeg;
			double rate;


	};
	
	class V4L1Client;
	
};


/// A live video buffer which uses the Video for Linux 2 (V4L2) API.
/// A replacement for the (deprecated?) V4L2Buffer
/// @ingroup gVideoBuffer
template <class T> class UVCBuffer : public VideoBuffer<T>, public UVC::RawUVCBuffer
{
	public:
		UVCBuffer(const std::string & dev, ImageRef size, int frames_per_second=0, bool mjpeg=0, bool verbose=0) 
		:VideoBuffer<T>(VideoBufferType::Live), RawUVCBuffer(dev, UVC::format<T>::fmt, size, frames_per_second, mjpeg, verbose)
		{
		}

		virtual ImageRef size() 
		{ 
			return getSize();
		}


		virtual VideoFrame<T> * get_frame() {
			
			Image<T> frame(size());
			fill_frame(frame.data());
			return new UVCFrame(0.0, std::move(frame));
		}

		virtual void put_frame(VideoFrame<T>* f) {
			UVCFrame* vf = dynamic_cast<UVCFrame*>(f);
			if (vf == 0)
				throw Exceptions::UVCBuffer::PutFrame(device_name(), "Invalid VideoFrame");
			delete vf;
		}

		virtual bool frame_pending() 
		{ 
			return true;
		}

		virtual double frame_rate() 
		{ 
			return getRate();
		}

		int num_buffers()
		{
			return num_buffers();
		}

	
 private:

		struct UVCFrame : public LocalVideoFrame<T> {
			UVCFrame(double t, CVD::Image<T>&& local) 
			:LocalVideoFrame<T>(t, std::move(local))
			{}

			int id;
			friend class UVCBuffer<T>;
		};

		UVCBuffer(const UVCBuffer&) = delete;
		void operator=(const UVCBuffer&) = delete;

};

};
#endif
