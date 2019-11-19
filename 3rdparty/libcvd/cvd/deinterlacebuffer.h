// PAS 17/6/04 (revised 16/2/05)
#ifndef CVD_INCLUDE_DEINTERLACEBUFFER_H
#define CVD_INCLUDE_DEINTERLACEBUFFER_H

#include <cvd/videobuffer.h>
#include <cvd/deinterlaceframe.h>
#include <cvd/internal/pixel_operations.h>
#include <cvd/internal/rgb_components.h>


namespace CVD
{
////////////////
// DEINTERLACE BUFFER EXCEPTIONS
//
namespace Exceptions
{
	/// %Exceptions specific to DeinterlaceBuffer.
	/// @ingroup gException
	namespace DeinterlaceBuffer
	{	
		/// Base class for all DeinterlaceBuffer exceptions
		/// @ingroup gException
		struct All: public CVD::Exceptions::VideoBuffer::All { }; 
		
		/// The VideoBuffer that is being wrapped does not have an even number of lines (so the odd and even- fields would not be the same size)
		/// @ingroup gException
		struct OddNumberOfLines: public All { OddNumberOfLines(); }; 
	}
}


/////////////////
// DEINTERLACE BUFFER
//

/// A decorator class which wraps a VideoBuffer to return fields instead of 
/// the original frames (see also DeinterlaceFrame). The majority of commands are passed
/// straight through to the buffer that this class wraps, but get_frame() is 
/// overloaded to extract fields from the video frames.
///
/// Video intended for television use (i.e. from non-progressive scan cameras) tends
/// to be interlaced. Instead of grabbing the entire frame at one time instant, the 
/// image is grabbed in two parts (fields), made up of the odd-numbered lines and the 
/// even-numbered lines respectively (thus giving an effective <em>field-rate</em> of 
/// twice the video frame-rate. Any fast motion in frame will therefore exhibit serrated
/// distortion, with alternate lines out of step. 
///
/// This class returns individual fields from the video, which are guaranteed to 
/// represent a single time instant. The VideoFrames returned from this buffer are
/// therefore half the height of the original image, and so you might want to 
/// double the y-scale before displaying. 
///
/// Provides frames of type CVD::DeinterlaceFrame and throws exceptions of type 
/// CVD::Exceptions::DeinterlaceBuffer
/// @param T The pixel type of the original VideoBuffer
/// @ingroup gVideoBuffer

/// Used to select which fields, and in which order, to extract from the frame
struct DeinterlaceBufferFields
{
	enum Fields{
		OddOnly, ///< Odd fields only
		EvenOnly, ///< Even fields only
		OddEven, ///< Both fields, presenting the odd lines from each frame first 
		EvenOdd ///< Both fields, presenting the even lines from each frame first
	}; 
};

template <typename T>
class DeinterlaceBuffer : public VideoBuffer<T>
{
	public:
		typedef DeinterlaceBufferFields Fields;
		/// Construct a DeinterlaceBuffer by wrapping it around another VideoBuffer
		/// @param buf The buffer that will provide the raw frames
		/// @param fields The fields to 
   		DeinterlaceBuffer(CVD::VideoBuffer<T>& buf, Fields::Fields fields = Fields::OddEven, bool line_double=false);
 
		/// The size of the VideoFrames returns by this buffer. This will be half the 
		/// height of the original frames.
		ImageRef size();
		
		CVD::VideoFrame<T>* get_frame();

		virtual RawVideoBuffer* source_buffer()
		{
			return &m_vidbuf;
		}
		
		void put_frame(CVD::VideoFrame<T>* f);
		
		virtual bool frame_pending()
			{return m_vidbuf.frame_pending();}
			
		virtual void seek_to(double t)
			{return m_vidbuf.seek_to(t);}
			
		/// What is the (expected) frame rate of this video buffer, in frames per second?
		/// If OddEven or EvenOdd are selected, this will be reported as twice the original 
		/// buffer's rate.
		virtual double frame_rate()
	  	{
	  		if(m_fields == Fields::OddOnly || m_fields == Fields::EvenOnly)
	  			return m_vidbuf.frame_rate();
			else
		  		return m_vidbuf.frame_rate() * 2.0;
		}
      
   private:
		CVD::VideoFrame<T>* my_realframe;
		CVD::VideoBuffer<T>& m_vidbuf;
		Fields::Fields m_fields;
		bool m_loadnewframe;
		ImageRef m_size;
		unsigned int m_linebytes;
		bool line_double;
};

//
// CONSTRUCTOR
//
template <typename T>
DeinterlaceBuffer<T>::DeinterlaceBuffer(CVD::VideoBuffer<T>& vidbuf, Fields::Fields fields, bool l) :
	VideoBuffer<T>(vidbuf.type()),
	m_vidbuf(vidbuf),
	m_fields(fields),
	m_loadnewframe(true),
	line_double(l)
{
	// Check it has an even number of lines
	if(m_vidbuf.size().y % 2 != 0)
		throw Exceptions::DeinterlaceBuffer::OddNumberOfLines();
	
	if(line_double == false)
		m_size = ImageRef(m_vidbuf.size().x, m_vidbuf.size().y / 2);
	else
		m_size = m_vidbuf.size();

	m_linebytes = sizeof(T) * m_size.x;
}

//
// GET FRAME
//
template <typename T>
VideoFrame<T>* DeinterlaceBuffer<T>::get_frame()
{
	if(m_loadnewframe)
	{
		// Get a new frame from the real videobuffer
		my_realframe = m_vidbuf.get_frame();
	}
		
	// Now return the deinterlaced image
	// First sort out the time
	double time = my_realframe->timestamp();
	
	// If we're giving the second frame of a pair, make its time half-way to the next frame
	if(!m_loadnewframe)
		time += 0.5/frame_rate(); 
	
	DeinterlaceFrame<T>* frame = new DeinterlaceFrame<T>(time, Image<T>(size()));


	if(m_fields == Fields::OddOnly || 
		(m_fields == Fields::OddEven && m_loadnewframe) ||
		(m_fields == Fields::EvenOdd && !m_loadnewframe))
	{

		// We want the odd field
		if(line_double)
		{
			frame->zero();
			for(int y=1; y < m_size.y; y+=2)
				for(int x=0; x < m_size.x; x++)
					(*frame)[y][x] = (*my_realframe)[y][x];	

			
			for(int y=2; y < m_size.y-1; y+=2)
				for(int x=0; x < m_size.x; x++)
					for(unsigned int i=0; i < Pixel::Component<T>::count; i++)
						Pixel::Component<T>::get((*frame)[y][x],i) = (Pixel::Component<T>::get((*my_realframe)[y-1][x],i) + 
						                                              Pixel::Component<T>::get((*my_realframe)[y+1][x],i))/2;

/*
			//Copy line 0 from line 1, and copy over line 1 to line 1
			for(int y=0; y < 2; y++)
				for(int x=0; x < m_size.x; x++)
					(*frame)[y][x] = (*my_realframe)[0][x];

			//Done 0, 1. next 2, 3

			for(int y=3; y < m_size.y; y+=2)
				for(int x=0; x < m_size.x; x++)
				{
					(*frame)[y][x] = (*my_realframe)[y][x];
					(*frame)[y-1][x] = ((*my_realframe)[y][x] + (*my_realframe)[y-2][x])/2;
				}
*/
		}
		else
		{
			for(int y=0; y < m_size.y; y++)
				for(int x=0; x < m_size.x; x++)
					(*frame)[y][x] = (*my_realframe)[2*y+1][x];
		}
		
	}
	else
	{
		// We want the even field
		// We want the odd field
		if(line_double)
		{
			frame->zero();
			for(int y=0; y < m_size.y; y+=2)
				for(int x=0; x < m_size.x; x++)
					(*frame)[y][x] = (*my_realframe)[y][x];	

			for(int y=1; y < m_size.y-1; y+=2)
				for(int x=0; x < m_size.x; x++)
					for(unsigned int i=0; i < Pixel::Component<T>::count; i++)
						Pixel::Component<T>::get((*frame)[y][x],i) = (Pixel::Component<T>::get((*my_realframe)[y-1][x],i) + 
						                                              Pixel::Component<T>::get((*my_realframe)[y+1][x],i))/2;
/*
			//Copy over and double the first set of lines
			for(int y=0; y < m_size.y-2; y+=2)
				for(int x=0; x < m_size.x; x++)
				{
					(*frame)[y][x] = (*my_realframe)[y][x];
					(*frame)[y+1][x] = ((*my_realframe)[y][x] + (*my_realframe)[y+2][x])/2;
				}
			
			//Copy the last line.
			for(int y=m_size.y-2; y < m_size.y; y++)
				for(int x=0; x < m_size.x; x++)
					(*frame)[y][x] = (*my_realframe)[m_size.y-2][x];
*/
		}
		else
		{
			for(int y=0; y < m_size.y; y++)
				for(int x=0; x < m_size.x; x++)
					(*frame)[y][x] = (*my_realframe)[2*y][x];
		}
	}
	frame->real_frame = my_realframe;
	
	if(m_fields == Fields::OddEven || m_fields == Fields::EvenOdd)
	{
		// If we're taking both fields, we only load a frame every other field
		m_loadnewframe = !m_loadnewframe;
	}

  return frame;
}

//
// SIZE
//
template <typename T>
ImageRef DeinterlaceBuffer<T>::size()
{
	return m_size;
}

//
// PUT FRAME
//
template <typename T>
void DeinterlaceBuffer<T>::put_frame(CVD::VideoFrame<T>* frame)
{
	if(m_loadnewframe)
	{
		// Next time we'll be getting a new real frame, so put back the current real frame
		m_vidbuf.put_frame(my_realframe);
	}
	
	// And delete the data for my current deinterlaced frame
	delete dynamic_cast<DeinterlaceFrame<T>*>(frame);
}

} // CVD
#endif
