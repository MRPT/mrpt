#ifndef CVD_INCLUDE_SKIPBUFFER_H
#define CVD_INCLUDE_SKIPBUFFER_H

#include <cvd/videobuffer.h>

namespace CVD
{

/// A decorator class which wraps a VideoBuffer to skip frames 
/// @param T The pixel type of the original VideoBuffer
/// @ingroup gVideoBuffer

template <typename T>
class SkipBuffer : public VideoBuffer<T>
{
	public:
		typedef DeinterlaceBufferFields Fields;
		/// Construct a DeinterlaceBuffer by wrapping it around another VideoBuffer
		/// @param buf The buffer that will provide the raw frames
		/// @param fields The fields to 
   		SkipBuffer(CVD::VideoBuffer<T>& buf, bool do_seek, double seek, int drop_)
		:VideoBuffer<T>(buf.type()), m_vidbuf(buf),drop(drop_)
		{
			if(do_seek)
				buf.seek_to(seek);
		}
 
		/// The size of the VideoFrames returns by this buffer. This will be half the 
		/// height of the original frames.
		ImageRef size()
		{
			return m_vidbuf.size();
		}
		
		CVD::VideoFrame<T>* get_frame()
		{
			CVD::VideoFrame<T>* f = m_vidbuf.get_frame();

			for(int i=0; i < drop && (m_vidbuf.type() == VideoBufferType::Flushable || m_vidbuf.frame_pending()); i++)
			{
				m_vidbuf.put_frame(m_vidbuf.get_frame());
			}

			return f;
		}
		
		virtual RawVideoBuffer* source_buffer()
		{
			return  &m_vidbuf;
		}

		void put_frame(CVD::VideoFrame<T>* f)
		{
			return m_vidbuf.put_frame(f);
		}
		
		virtual bool frame_pending()
		{
			return m_vidbuf.frame_pending();
		}
			
		virtual void seek_to(double t)
		{
			return m_vidbuf.seek_to(t);
		}
			
		virtual double frame_rate()
	  	{
			return m_vidbuf.frame_rate();
		}
      
   private:
		CVD::VideoBuffer<T>& m_vidbuf;
		int drop;
};
}

#endif
