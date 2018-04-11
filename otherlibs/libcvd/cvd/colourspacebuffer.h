// PAS 17/6/04 (revised 16/2/05)

#ifndef CVD_INCLUDE_COLOURSPACEBUFFER_H
#define CVD_INCLUDE_COLOURSPACEBUFFER_H

#include <cvd/localvideobuffer.h>
#include <cvd/image_convert.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspace_frame.h>

namespace CVD
{
/// A decorator class which wraps a VideoBuffer to perfrom colourspace
/// conversion on the incoming data. In general, the kernel video buffers
/// such as dvbuffer and v4l1buffer give access to what the device can provide
/// natively. However, the native formats may not be what is required. This 
/// buffer transparently converts incoming frames to the required type.
///
/// Not every possible conversion is available natively through the library,
/// some conversions have to be performed in several steps. For instance, to 
/// convert yuv411 to <code>Rgb<float></code>, the conversion must go via 
/// <code>Rgb<byte></code> since
/// the conversions from yuv411 are limited.
/// 
/// Provides frames of type <code>CVD::ColourspaceFrame.</code>
///
/// This class throws only generic VideoBuffer exceptions, but the underlying
/// videobuffer may throw.
/// @param From  The pixel type of the original VideoBuffer
/// @param T  The pixel type to convert in to.
/// @ingroup gVideoBuffer
template <class T, class From> class ColourspaceBuffer : public CVD::LocalVideoBuffer<T>
{
	public:
		/// Construct a ColourspaceBuffer by wrapping it around another VideoBuffer
		/// @param buf The buffer that will provide the raw frames
   		ColourspaceBuffer(CVD::VideoBuffer<From>& buf)
		:LocalVideoBuffer<T>(buf.type()),m_vidbuf(buf),m_size(buf.size())
		{
		}
		
		virtual RawVideoBuffer* source_buffer()
		{
			return &m_vidbuf;
		}
 
		/// The size of the VideoFrames returns by this buffer.
		ImageRef size()
		{
			return m_size;	
		}
		
		virtual bool frame_pending()
		{	
			return m_vidbuf.frame_pending();
		}
			
		virtual void seek_to(double t)
		{
			m_vidbuf.seek_to(t);
		}
			
		virtual double frame_rate()
	  	{
			return m_vidbuf.frame_rate();
		}

		virtual CVD::ColourspaceFrame<T>* get_frame()
		{
			VideoFrame<From>* fr = m_vidbuf.get_frame();
			Image<T> cv = convert_image<T>(*fr);
			ColourspaceFrame<T>* ret = new ColourspaceFrame<T>(fr->timestamp(), std::move(cv));
			m_vidbuf.put_frame(fr);
			return ret;
		}

		virtual void put_frame(CVD::VideoFrame<T>* f)
		{
			//Check that the type is correct...
			ColourspaceFrame<T>* csf = dynamic_cast<ColourspaceFrame<T>*>(f);

			if(csf == NULL)
				throw CVD::Exceptions::VideoBuffer::BadPutFrame();
			else 
				delete csf;
		}

   private:
		CVD::VideoBuffer<From>& m_vidbuf;
		ImageRef m_size;
};


/// This is just like ColourspaceBuffer, except it deleted the videobuffer on destruction
template <class T, class From> class ColourspaceBuffer_managed : public ColourspaceBuffer<T, From>
{
	public:
		/// Construct a ColourspaceBuffer by wrapping it around another VideoBuffer
		/// @param buf The buffer that will provide the raw frames
   		ColourspaceBuffer_managed(CVD::VideoBuffer<From>* buf)
		:ColourspaceBuffer<T,From>(*buf),vb(buf)
		{
		}

		~ColourspaceBuffer_managed()
		{
			delete  vb;
		}

	private:
		VideoBuffer<From>* vb;

};

} 
#endif
