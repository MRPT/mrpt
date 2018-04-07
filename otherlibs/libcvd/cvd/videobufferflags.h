// Paul Smith 8 March 2005
// Flags for video buffers 

#ifndef __VIDEOBUFFERFLAGS__
#define __VIDEOBUFFERFLAGS__

namespace CVD
{
	/// Flags common to several different VideoBuffer classes
	namespace VideoBufferFlags
	{
		/// If it is a finite buffer (a video file, for example), what should happen when the 
		/// end of the buffer is reached?
		enum OnEndOfBuffer{
			RepeatLastFrame, ///< Continue to return the final frame when get_frame() is called (with the same timestamp)
			UnsetPending, ///< Set the return value of frame_pending() to false and throw an EndOfBuffer exception if get_frame() is called
			Loop ///< Loop the buffer, so that the next time get_frame() is called it returns the first frame in the buffer
		};
	}
}
		
#endif
