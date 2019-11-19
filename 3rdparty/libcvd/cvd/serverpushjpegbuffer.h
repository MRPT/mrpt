#ifndef CVD_INC_SERVERPUSHJPEGBUFFER_H
#define CVD_INC_SERVERPUSHJPEGBUFFER_H
#include <cvd/localvideobuffer.h>
#include <cvd/timer.h>
#include <cvd/serverpushjpegframe.h>
#include <iostream>

namespace CVD
{

/// Play a server push stream as a video stream.  This is a standard used by a number of
/// HTTP based security cameras. The format is as follows:
/// @code
/// --ImageSeparator\n
/// Content-Type: image/jpeg\r\n
/// Content-Length: 123456789\r\n
/// \r\n
/// <123456789 bytes of JPEG go here>\r\n
/// @endcode
/// This exact format is from the InVision IQEye series of cameras. Other cameras have
/// a different image separator, and do not miss the carriage return.
///
/// The buffer reads a number of frames from the stream on initiation (by default 10),
/// in order to flush the camera's inbuilt buffer. With some cameras, changing the video
/// size does not flush the buffer, so the first few frames will be of the incorrect size.
/// After flushing the buffer, the size of the first frame is taken to be the size of the
/// video stream. If spurious frames arrive of a different size later, these will be ignored.
///
/// WARNING: error checking is currently very minimal. The result of failure will probably 
/// result in an exception being thrown from the JPEG loader.
///
/// @param T The pixel type of the frames to provide (usually <code>CVD::Rgb<CVD::byte></code> 
/// or <code>CVD::byte</code>. If the image files are of a different type, they will be automatically 
/// converted (see @link gImageIO Image loading and saving, and format conversion@endlink).
/// @ingroup gVideoBuffer
template<class C> class ServerPushJpegBuffer: public LocalVideoBuffer<C>
{
	public:
		///Construct a ServerPushJpegBuffer from an istream. The istream 
		///
		///
		///@param i The stream to use for video.
		///@param warnings Whether to print warnings if mis-sized frames arrive. 
		///@param eat_frames Number of frames to discard on initialization.
		ServerPushJpegBuffer<C>(std::istream& i, bool warnings_=0, int eat_frames=0)
		:LocalVideoBuffer<C>(VideoBufferType::Live),is(i),warnings(warnings_)
		{
			std::string tmp;
			//Eat the first 10 frames because the camera sometimes takes a while to
			//crank out ones of the specified size
			
			for(int junk=0; junk< eat_frames; junk++)
				gimme_an_image(tmp);
			

			//Eat the first frame just to get the size
			Image<C> c = gimme_an_image(tmp);
			s = c.size();
		}
		
		virtual ImageRef size()
		{
			return s;
		}	
		
		LocalVideoFrame<C>* get_frame()
		{
			Image<C> c;
			std::string data;
			
			loop:
			c = gimme_an_image(data);

			if(c.size() != s)
			{
				if(warnings)
					std::cerr << "ServerPushJpegBuffer: video frame is " << c.size() << " not " << s << std::endl;
				goto loop;
			}
			
			return new ServerPushJpegFrame<C>(get_time_of_day(), std::move(c), data);
		}

		void put_frame(VideoFrame<C>* f)
		{
			LocalVideoFrame<C>* g = dynamic_cast<LocalVideoFrame<C>*>(f);

			if(g == NULL)
				throw CVD::Exceptions::VideoBuffer::BadPutFrame();
			else
				delete g;
		}

		bool frame_pending()
		{
			return 1;
		}

		void seek_to(double){};
		
		///This value is not currently correct.
		double frame_rate()
		{
			return 30;
		}

	private:
		std::istream& is;
		ImageRef s;
		bool warnings;

		Image<C> gimme_an_image(std::string& data)
		{

			std::string line;
			
			int length;
			getline(is, line); //Get --ImageSeparator
			getline(is, line); //Get Content-Type:
			is >> line;        //Get Content-Length:
			is >> length;	   //Get the actual content length
			getline(is, line); //Eat the rest of the line 
			getline(is, line); //Get the blank line
			
			data.resize(length);
			is.read(&data[0], length);

			std::istringstream ss(data);

			Image<C> c = img_load(ss);

			is.get();         //Eat the \r\n after the JPEG
			is.get();

			return c;
		}
};

}
#endif
