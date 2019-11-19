#ifndef CVD_COLOURSPACE_FRAME_H
#define CVD_COLOURSPACE_FRAME_H

#include <cvd/localvideoframe.h>
#include <utility>

namespace CVD
{
	//template<class To, class From> class ColourspaceBuffer;

	/// A frame from a ColourspaceBuffer. Can be treated as a VideoFrame
	template<class T> 
	class ColourspaceFrame : public CVD::LocalVideoFrame<T>
	{
		/// Allow ColourspaceBuffer to manage frames.
		public:
			~ColourspaceFrame()
			{
			}

			ColourspaceFrame(double time, CVD::Image<T>&& converted)
			:LocalVideoFrame<T>(time, std::move(converted))
			{
			}	
	};
}


#endif
