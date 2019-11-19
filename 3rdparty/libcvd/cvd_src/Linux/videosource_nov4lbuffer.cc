#include <cvd/videosource.h>

namespace CVD{

	template <> VideoBuffer<byte>* makeV4LBuffer(const std::string&, const ImageRef&, int, bool, bool)
	{
		throw VideoSourceException("V4LBuffer is not compiled in to libcvd.");
	}

	template <> VideoBuffer<bayer_grbg>* makeV4LBuffer(const std::string&, const ImageRef&, int, bool, bool)
	{
		throw VideoSourceException("V4LBuffer is not compiled in to libcvd.");
	}

	template <> VideoBuffer<yuv422>* makeV4LBuffer(const std::string&, const ImageRef&, int, bool, bool)
	{
		throw VideoSourceException("V4LBuffer is not compiled in to libcvd.");
	}

	template <> VideoBuffer<vuy422>* makeV4LBuffer(const std::string&, const ImageRef&, int, bool, bool)
	{
		throw VideoSourceException("V4LBuffer is not compiled in to libcvd.");
	}

	template <> VideoBuffer<Rgb<byte> >* makeV4LBuffer(const std::string&, const ImageRef&, int, bool, bool)
	{
		throw VideoSourceException("V4LBuffer is not compiled in to libcvd.");
	}

	template <> VideoBuffer<Rgb8>* makeV4LBuffer(const std::string&, const ImageRef&, int, bool, bool)
	{
		throw VideoSourceException("V4LBuffer is not compiled in to libcvd.");
	}


	template <> VideoBuffer<yuv420p>* makeV4LBuffer(const std::string&, const ImageRef&, int, bool, bool)
	{
		throw VideoSourceException("V4LBuffer is not compiled in to libcvd.");
	}

}
