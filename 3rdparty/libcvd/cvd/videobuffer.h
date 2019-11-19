#ifndef CVD_VIDEOBUFFER_H
#define CVD_VIDEOBUFFER_H

#include <cvd/videoframe.h>
#include <cvd/exceptions.h>
#include <memory>

namespace CVD {

///The semsntics of the videobuffer. See VideoFrame::type()
struct VideoBufferType
{
	enum Type
	{
		///The buffer does not have live semantics: frames
		///are not throttled by something external. 
		///VideoBuffer::frame_pending() is true until the last frame has 
		///been retrieved, after which is is set to false.
		NotLive, 
		///The buffer has live semantics: frames are throttled by
		///something externa, but VideoBuffer::frame_pending() always returns true.
		Live,
		///The buffer is flushable: it is live and VideoBuffer::frame_pending() returns
		///an accurate result.
		Flushable
	};
};

///Base class which provides untyped access to video grabber objects.
///This provides all of the functionality for which the type does not
///need to be known (eg size, etc).
///
///This allows one to follow chains of video buffers which change types (for instance
///<code>ColourspaceBuffer<Rgb<byte>, byte>) and get to the underlying video grabber.
///This provides access to the grabber specific controls.
///
///See progs/video_play_source.cc for a very basic example.
class RawVideoBuffer
{
	public:

		/// Which video grabber provides the source images for this
		/// video grabber.
		virtual RawVideoBuffer* source_buffer()
		{
			return this;
		}
		
		/// Follow the chain of video grabbers back as far as at will go. This will
		/// usually yield the video grabber dealing with the hardware.
		RawVideoBuffer* root_buffer()
		{
			RawVideoBuffer* b = this;
			while(b->source_buffer() != b)
				b = b->source_buffer();
			return b;
		}


		virtual ~RawVideoBuffer(){}

		/// The size of the VideoFrames returned by this buffer
		virtual ImageRef size()=0;

		/// Is there a frame waiting in the buffer? This function does not block. 
		/// See is_live and is_flushable.
		virtual bool frame_pending()=0;

		/// What is the (expected) frame rate of this video buffer, in frames per second?		
		virtual double frame_rate()=0;
		/// Go to a particular point in the video buffer (only implemented in buffers of recorded video)
		/// \param t The frame time in seconds
		virtual void seek_to(double)
		{}


		/// Flush all old frames out of the video buffer,
		/// on a flushable buffer, causing the next get_frame()
		/// to sleep until a frame arrives. On a non-flushable
		/// buffer, this does nothing.
		virtual void flush()=0;
		
};

/// Base class for objects which provide a typed video stream. A video 
/// stream is a sequence of video frames (derived from VideoFrame).
/// @param T The pixel type of the video frames
/// @ingroup gVideoBuffer
template <class T> 
class VideoBuffer: public virtual RawVideoBuffer
{
	public:
		///Construct the buffer with the known semantics
		VideoBuffer(VideoBufferType::Type _type)
		:m_type(_type)
		{}

		virtual ~VideoBuffer()
		{}

		/// Returns the next frame from the buffer. This function blocks until a frame is ready.
		virtual VideoFrame<T>* get_frame()=0;        	

		/// Tell the buffer that you are finished with this frame. Typically the VideoBuffer then destroys the frame.
		/// \param f The frame that you are finished with.
		virtual void put_frame(VideoFrame<T>* f)=0;
		
		virtual void flush()
		{
			if(type() == VideoBufferType::Flushable)
				while(frame_pending())
					put_frame(get_frame());
		}

		/// Returns the type of the video stream
		///
		/// A video with live semantics has frames fed at
		/// some externally controlled rate, such as from a 
		/// video camera. 
		///
		/// A stream with live semantics also may be flushable, in
		/// that all current frames can be removed from the stream
		/// while frame_pending() is 1, and then the next get_frame()
		/// will sleep until a frame arrives. This ensures that the latency
		/// is low by discarding any old frames. Buffers flushable in this
		/// manner have a type of VideoBuffer::Type::Flushable.
		/// 
		/// Some live streams are not flushable because it is not possible
		/// to determine the state of frame_pending(). These have the type
		/// VideoBuffer::Type::Live, and frame_pending() is always 1.
		///
		/// Otherwise, streams have a type VideoBuffer::Type::NotLive, and
		/// frame_pending is always 1
		///
		///This should be in the base class
		VideoBufferType::Type type()
		{
			return m_type;
		}

	private:
		VideoBufferType::Type m_type;
};


namespace Exceptions
{
	/// %Exceptions specific to VideoBuffer
	/// @ingroup gException
	namespace VideoBuffer
	{
		/// Base class for all VideoBuffer exceptions
		/// @ingroup gException
		struct All: public CVD::Exceptions::All
		{
		};

		/// The VideoBuffer was unable to successfully complete a VideoBuffer::put_frame() operation
		/// @ingroup gException
		struct BadPutFrame: public All
		{
			BadPutFrame();
		};
		
		/// The videobuffer was unable to successfully initialize grabbing in the 
		/// specified colourspace.
		/// @ingroup gException
		struct BadColourSpace: public All
		{
			/// @param colourspace Specify the failed colourspace.
			/// @param b Specify the failed buffer.
			BadColourSpace(const std::string& colourspace, const std::string& b); 
		};
	}
}



}

#endif
