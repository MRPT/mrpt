#include <cvd/videosource.h>
#include <cvd/video/uvcbuffer.h>
namespace CVD{

	template <> VideoBuffer<yuv422>* makeUVCBuffer(const std::string& dev, const ImageRef& size, double fps, bool jpeg, bool verbose)
	{
		return new UVCBuffer<yuv422>(dev, size, fps, jpeg, verbose);
	}
	template <> VideoBuffer<Rgb<byte> >* makeUVCBuffer(const std::string& dev, const ImageRef& size, double fps, bool jpeg, bool verbose)
	{
		return new UVCBuffer<Rgb<byte> >(dev, size, fps, jpeg, verbose);
	}

}
