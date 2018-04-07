#ifndef CVD_INC_SERVERPUSHJPEGFRAME_H
#define CVD_INC_SERVERPUSHJPEGFRAME_H
#include <cvd/localvideoframe.h>

namespace CVD
{

template<class T> class ServerPushJpegBuffer;

template<class T> class ServerPushJpegFrame: public LocalVideoFrame<T>
{
	friend class CVD::ServerPushJpegBuffer<T>;

	public:		

		/// The underlying JPEG data.
		const std::string& jpeg() {return image_data;};

	private:
		~ServerPushJpegFrame()
		{
		}

		ServerPushJpegFrame(double time, CVD::Image<T>&& im, const std::string& data)
		:LocalVideoFrame<T>(time, std::move(im)),image_data(data)
		{
		}	
		
	private:
		std::string image_data;
};


};

#endif
