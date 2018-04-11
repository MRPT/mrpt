#ifndef CVD_DISKBUFFER2_FRAME_H
#define CVD_DISKBUFFER2_FRAME_H

#include <cvd/localvideoframe.h>
#include <string>
#include <utility>

namespace CVD
{
	template<class T> class DiskBuffer2;

	/// A frame from a DiskBuffer2. The frames in a DiskBuffer2 are image files loaded from disk.
	/// @param T The pixel type of the frames to provide (usually <code>CVD::Rgb<CVD::byte></code> 
	/// or <code>CVD::byte</code>. If the image files are of a different type, they will be automatically 
	/// converted (see @link gImageIO Image loading and saving, and format conversion@endlink).
	/// @ingroup gVideoFrame
	template<typename T> 
	class DiskBuffer2Frame : public CVD::LocalVideoFrame<T>
	{
			/// Allow DiskBuffer2 to create a DiskBuffer2Frame
			friend class CVD::DiskBuffer2<T>;
		public:		

			/// What is the filename for this image?
			const std::string& name() {return frame_name;};

			/// set the time stamp, required for TimedDiskBuffer2
			void timestamp( double time ) { this->my_timestamp = time; }

		private:
			~DiskBuffer2Frame()
			{
			}

			DiskBuffer2Frame(double time, CVD::Image<T>&& from_disk, const std::string& file)
			:LocalVideoFrame<T>(time, std::move(from_disk)),frame_name(file)
			{
			}	

			
		private:
			const std::string& frame_name;
	};
}


#endif
