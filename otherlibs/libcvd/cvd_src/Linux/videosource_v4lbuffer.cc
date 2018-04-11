#include <cvd/videosource.h>
#include <cvd/Linux/v4lbuffer.h>
namespace CVD{

	template <> VideoBuffer<byte>* makeV4LBuffer(const std::string& dev, const ImageRef& size, int input, bool interlaced, bool verbose)
	{
		return new V4LBuffer<byte>(dev, size, input, interlaced, 0, verbose);
	}

	template <> VideoBuffer<bayer_grbg>* makeV4LBuffer(const std::string& dev, const ImageRef& size, int input, bool interlaced, bool verbose)
	{
		#ifdef V4L2_PIX_FMT_SBGGR8
			return new V4LBuffer<bayer_grbg>(dev, size, input, interlaced, 0, verbose);
		#else
			throw VideoSourceException("Bayer video grabbing is not available in this kernel version.");
		#endif
	}

	template <> VideoBuffer<yuv422>* makeV4LBuffer(const std::string& dev, const ImageRef& size, int input, bool interlaced, bool verbose)
	{
		return new V4LBuffer<yuv422>(dev, size, input, interlaced, 0, verbose);
	}
	template <> VideoBuffer<vuy422>* makeV4LBuffer(const std::string& dev, const ImageRef& size, int input, bool interlaced, bool verbose)
	{
		return new V4LBuffer<vuy422>(dev, size, input, interlaced, 0, verbose);
	}
	template <> VideoBuffer<Rgb<byte> >* makeV4LBuffer(const std::string& dev, const ImageRef& size, int input, bool interlaced, bool verbose)
	{
		return new V4LBuffer<Rgb<byte> >(dev, size, input, interlaced, 0, verbose);
	}

	template <> VideoBuffer<Rgb8>* makeV4LBuffer(const std::string& dev, const ImageRef& size, int input, bool interlaced, bool verbose)
	{
		return new V4LBuffer<Rgb8>(dev, size, input, interlaced, 0, verbose);
	}

	template <> VideoBuffer<yuv420p>* makeV4LBuffer(const std::string& dev, const ImageRef& size, int input, bool interlaced, bool verbose)
	{
		return new V4LBuffer<yuv420p>(dev, size, input, interlaced, 0, verbose);
	}
}
