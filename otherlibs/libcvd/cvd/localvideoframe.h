/*******************************************************************************
                                                                  
   LocalVideoFrame - A video frame for when the data is local snd should be
   					 managed by the program instead of the system. Uses Image
					 to manage the memory. Programs which will only ever use 
					 these can be optimized by using the image() methos. These
					 can be deleted sensibly, but it is not currently allowed, 
					 to make the interface more consistent.
                                                               
   E. Rosten	   - 18 March 2005
                                                             
*******************************************************************************/


#ifndef CVD_LOCALVIDEOFRAME_FRAME_H
#define CVD_LOCALVIDEOFRAME_FRAME_H

#include <cvd/videoframe.h>
#include <string>

namespace CVD
{
	/// A frame from a LocalVideoBuffer, which manages its own data rather than wrapping
	/// data owned by the system.
	/// The data is stored internally using Image, and programs which will only ever use 
	/// LocalVideoBuffers can be optimized by using the 
	/// image() method. Being Images, these can be deleted sensibly.
	/// @param T The pixel type of the video frames
	/// @ingroup gVideoFrame
	template<class T> 
		class LocalVideoFrame: public VideoFrame<T>
	{

		public:

			virtual ~LocalVideoFrame()
			{
			}

			/// Construct a video frame from an Image and a timestamp.
			/// Note that the local video frame wants to own the image data
			/// so only a move constructor is provided.
			/// @param time The timestamp of this frame
			/// @param local The Image to use for this frame
			LocalVideoFrame(double time, CVD::Image<T>&& local)
				:VideoFrame<T>(time, nullptr, local.size()),
				im(std::move(local))
			{
				this->my_data = im.data();
			}	

			/// Returns the image. A LocalVideoFrame can be treated just like any other Image
			/// (for example it can use optimised copying)
			Image<T>& image()
			{
				return im;
			}
			const Image<T>& image() const
			{
				return im;
			}

			double& timestamp() 
			{
				return this->my_timestamp;
			}

		private:
			CVD::Image<T>		  im;
	};
}


#endif
