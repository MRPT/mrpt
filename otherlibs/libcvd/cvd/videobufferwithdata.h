#ifndef CVD_VIDEOBUFFERWITHDATA_H
#define CVD_VIDEOBUFFERWITHDATA_H

#include <cvd/videobuffer.h>
#include <memory>

namespace CVD {

/// Certain video buffers, especially the decorator classes, and buffers
/// such as ServerPushJpegBuffer have additional data 
/// with the same lifetime as the buffer. This is a tool to allow management of
/// this data. This class manages a video buffer and some data concurrently.
/// @param T The pixel type of the video frames
/// @ingroup gVideoBuffer
template <class T, class D> 
class VideoBufferWithData: public VideoBuffer<T> 
{
	public: 
		VideoBufferWithData(std::unique_ptr<VideoBuffer<T> >& buf_, std::unique_ptr<D>& d)
		:VideoBuffer<T>(buf_->type()), buf(std::move(buf_)),extra_data(move(d))
		{}

		ImageRef size()
		{
			return buf->size();
		}

		virtual RawVideoBuffer* source_buffer()
		{
			return buf.get();
		}

		VideoFrame<T>* get_frame()
		{
			return buf->get_frame();
		}

		void put_frame(VideoFrame<T>* f)
		{
			buf->put_frame(f);
		}	

		bool frame_pending()
		{
			return buf->frame_pending();
		}	

		void flush()
		{
			return buf->flush();
		}	

		double frame_rate()
		{
			return buf->frame_rate();
		}

		void seek_to(double time)
		{
			return buf->seek_to(time);
		}


	private:
		std::unique_ptr<VideoBuffer<T> > buf;
	public:
		std::unique_ptr<D> extra_data;
};

}

#endif
