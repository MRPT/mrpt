#ifndef CVD_DISKBUFFER2_H
#define CVD_DISKBUFFER2_H

#include <vector>
#include <string>
#include <utility>
#include <fstream>
#include <errno.h>

#include <cvd/localvideobuffer.h>
#include <cvd/videobufferflags.h>
#include <cvd/diskbuffer2_frame.h>
#include <cvd/image_io.h>

namespace CVD
{
	//
	// GLOBLIST
	/// Make a list of strings from a UNIX-style pattern 
	/// pathname expansion. Tilde expansion is done, and * ? [] and {} can all be 
	/// used as normal. The filenames are returned in alphabetical (and numerical) order.
	/// @param gl The pattern from which to generate the strings
	/// @relatesalso DiskBuffer2
	std::vector<std::string> globlist(const std::string& gl);

	namespace Exceptions
	{
		/// %Exceptions specific to DiskBuffer2
		/// @ingroup gException
		namespace DiskBuffer2
		{
			/// Base class for all DiskBuffer2 exceptions
			/// @ingroup gException
			struct All: public CVD::Exceptions::VideoBuffer::All { };
			/// An empty list of filename strings was passed to the buffer
			/// @ingroup gException
			struct NoFiles: public All { NoFiles(); };
			/// An error occurred trying to open a file
			/// @ingroup gException
			struct BadFile: public All { BadFile(const std::string&, int); ///< Construct from filename and error number
			}; 
			/// An error occurred trying to read a file as an image
			/// @ingroup gException
			struct BadImage: public All { BadImage(const std::string& file, const std::string& error); ///< Construct from filename and error string 
			};
			/// The file loaded was a different size from the first frame
			/// @ingroup gException
			struct BadImageSize: public All { BadImageSize(const std::string& file); ///< Construct from filename  string 
			};
			/// get_frame() was called when at the end of the buffer
			/// @ingroup gException
			struct EndOfBuffer: public All { EndOfBuffer(); };
			/// seek_to() was called for an invalid timestamp
			/// @ingroup gException
			struct BadSeek: public All { BadSeek(double t);///< Construct from invalid timestamp
			 }; 
			
		}
	}

	/// Play a series of image files as a video stream. 
	/// Provides frames of type CVD::DiskBuffer2Frame and throws exceptions of type 
	/// CVD::Exceptions::DiskBuffer2
	/// @param T The pixel type of the frames to provide (usually <code>CVD::Rgb<CVD::byte></code> 
	/// or <code>CVD::byte</code>. If the image files are of a different type, they will be automatically 
	/// converted (see @link gImageIO Image loading and saving, and format conversion@endlink).
	/// @ingroup gVideoBuffer
	template<typename T> 
	class DiskBuffer2: public CVD::LocalVideoBuffer<T>
	{
		public:
			/// Construct a DiskBuffer2 from a vector of filenames. 
			/// Typically the globlist() helper function is used to provide the filenames
			/// e.g. <code>DiskBuffer2 buffer(globlist("~/Images/lab*.jpg"), 25);</code>
			/// @param names The filenames to use (played in the order that they are in the vector) 
			/// @param fps The frames per second to report for this VideoBuffer
			/// @param eob What should the buffer do when it reaches the end of the list of files?
			DiskBuffer2(const std::vector<std::string>& names, double fps, VideoBufferFlags::OnEndOfBuffer eob = VideoBufferFlags::RepeatLastFrame);

 			virtual ImageRef size() {return my_size;}
			
			/// Is there another frame waiting in the buffer? By default, this always 
			/// returns true, but if the VideoBufferFlags::OnEndOfBuffer setting is VideoBufferFlags::UnsetPending, this will return
			/// false after the last frame has been returned by get_frame()
			virtual bool frame_pending() {return frame_ready;}

			virtual DiskBuffer2Frame<T>* get_frame();
			virtual void put_frame(VideoFrame<T>* f);
			virtual void seek_to(double t);
		
			/// What should the buffer do when it reaches the end of the list of files?
			/// @param eob The desired behaviour
			virtual void on_end_of_buffer(VideoBufferFlags::OnEndOfBuffer eob) 
				{end_of_buffer_behaviour = eob;}

			virtual double frame_rate() 
			{
				return frames_per_sec;
			}

		protected:
			ImageRef my_size;
			int		 next_frame;
			double   start_time;
			double	 time_per_frame, frames_per_sec;
			bool frame_ready;
			std::vector<std::string> file_names;
			VideoBufferFlags::OnEndOfBuffer end_of_buffer_behaviour;
	};

	//
	// CONSTRUCTOR
	//
	template<typename T>
	inline DiskBuffer2<T>::DiskBuffer2(const std::vector<std::string>& names, double fps, VideoBufferFlags::OnEndOfBuffer eob) 
	:LocalVideoBuffer<T>(VideoBufferType::NotLive),end_of_buffer_behaviour(eob)
	{
		frames_per_sec = fps;

		start_time = 0;
		next_frame=0;
		time_per_frame = 1/fps;	

		file_names = names;

		if(file_names.size() == 0)
			throw Exceptions::DiskBuffer2::NoFiles();

		Image<T> foo;
		std::ifstream im;
		im.open(names[0].c_str(), std::ios::in|std::ios::binary);

		if(!im.good())
			throw Exceptions::DiskBuffer2::BadFile(names[0], errno);
		
		try
		{
			img_load(foo, im);
		}
		catch(Exceptions::Image_IO::All err)
		{
			throw Exceptions::DiskBuffer2::BadImage(names[0], err.what);
		}

		my_size = foo.size();
		frame_ready = true;
	}

	//
	// GET FRAME
	//
	template<typename T>
	inline DiskBuffer2Frame<T>* DiskBuffer2<T>::get_frame()
	{
		if(next_frame < 0)
			next_frame = 0;

		if(!frame_pending())
			throw Exceptions::DiskBuffer2::EndOfBuffer();

		Image<T> foo(my_size);
		
		std::ifstream im_file(file_names[next_frame].c_str(), std::ios::in|std::ios::binary);

		if(!im_file.good())
			throw Exceptions::DiskBuffer2::BadFile(file_names[next_frame], errno);

		try{
		  img_load(foo, im_file);
		}
		catch(CVD::Exceptions::Image_IO::All err)
		{
			throw Exceptions::DiskBuffer2::BadImage(file_names[next_frame], err.what);
		}

		DiskBuffer2Frame<T>* vf = new DiskBuffer2Frame<T>(next_frame * time_per_frame + start_time, std::move(foo), file_names[next_frame]);

		next_frame++;
		
		if(next_frame > (int)file_names.size()-1)
		{
			switch(end_of_buffer_behaviour)
			{
				case VideoBufferFlags::RepeatLastFrame:
					next_frame = file_names.size()-1;
					break;
				
				case VideoBufferFlags::UnsetPending:
					frame_ready = false;
				   break;
				
				case VideoBufferFlags::Loop:
					next_frame = 0;
					break;
			}
		}

		return vf;	
	}

	//
	// PUT FRAME
	//
	template<typename T>
	inline void DiskBuffer2<T>::put_frame(VideoFrame<T>* f)
	{
		//Check that the type is correct...
		DiskBuffer2Frame<T>* db2f = dynamic_cast<DiskBuffer2Frame<T>*>(f);

		if(db2f == NULL)
			throw CVD::Exceptions::VideoBuffer::BadPutFrame();
		else 
			delete db2f;
	}

	//
	// SEEK TO
	//
	template<typename T>
	inline void DiskBuffer2<T>::seek_to(double t)
	{
		// t is in ms, but work in seconds
		// round the answer to the nearest whole frame
		int frameno = static_cast<int>((t - start_time) / time_per_frame + 0.5);
		if(frameno < 0 || static_cast<unsigned int>(frameno) > (file_names.size() - 1))
			throw Exceptions::DiskBuffer2::BadSeek(t);
		next_frame = frameno;
		frame_ready = true;
	}
}


#endif
