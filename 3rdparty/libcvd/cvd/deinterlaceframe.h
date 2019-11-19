#ifndef __DEINTERLACE_FRAME_H__
#define __DEINTERLACE_FRAME_H__

#include <cvd/localvideoframe.h>
#include <utility>

namespace CVD
{
	template<typename T> class DeinterlaceBuffer;

	/// A frame from a DeinterlaceBuffer, representing one field from an
	/// interlaced frame.
	/// If the buffer is extracting both fields from the video frames, the
	/// time of the first field is reported as being the time of the
	/// original frame, while the time of the second field will be 
	/// 1/frame_rate() further on.
	/// @param T The pixel type of the original video buffer
	/// @ingroup gVideoFrame
	template<typename T> 
	class DeinterlaceFrame: public LocalVideoFrame<T>
	{
		friend class DeinterlaceBuffer<T>;
		
		public:
			/// Access the original (interlaced) frame
			const VideoFrame<T>* full_frame() {return real_frame;}

		private:
			~DeinterlaceFrame()
			{
			}

			DeinterlaceFrame(double time, Image<T>&& im)
			   :LocalVideoFrame<T>(time, std::move(im))
			{
			}	
			
		private:
			VideoFrame<T>* real_frame;
	};

}

#endif

