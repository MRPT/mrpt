#ifndef CVD_VIDEOFILEBUFFER_FRAME_H
#define CVD_VIDEOFILEBUFFER_FRAME_H

#include <cvd/localvideoframe.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <utility>

struct AVCodecContext;

namespace CVD
{
	namespace VFB
	{
		class RawVideoFileBufferPIMPL;
	}

	/// A frame from a VideoFileBuffer.
	/// @ingroup gVideoFrame	
	/// @param T The pixel type of the video frames. Currently only <code>CVD::Rgb<CVD::byte> ></code> and 
	/// <code>CVD::byte></code> are supported.
	template<class T> 
	class VideoFileFrame: public CVD::LocalVideoFrame<T>
	{
		friend class VFB::RawVideoFileBufferPIMPL;

	        public:
		        inline void delete_self() {  delete this; }
		protected:
			~VideoFileFrame()
			{
			}

			/// Construct a video frame from an Image and a timestamp
			/// @param time The timestamp of this frame
			/// @param local The Image to use for this frame
			VideoFileFrame(double time, CVD::Image<T>&& local)
			:LocalVideoFrame<T>(time, std::move(local))
			{
			}	

		private:
	};
}


#endif
