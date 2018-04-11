#include <cvd/videosource.h>
#include <cvd/videosource.h>

namespace CVD{

	template <> VideoBuffer<yuv422>* makeUVCBuffer(const std::string& dev, const ImageRef&, double, bool, bool)
	{
		throw VideoSourceException("V4LBuffer is not compiled in to libcvd.");
	}
	template <> VideoBuffer<Rgb<byte> >* makeUVCBuffer(const std::string& dev, const ImageRef&, double, bool, bool)
	{
		throw VideoSourceException("V4LBuffer is not compiled in to libcvd.");
	}

}
