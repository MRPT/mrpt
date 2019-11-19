#include <cvd/videosource.h>
#include <cvd/Linux/dvbuffer3.h>

namespace CVD{

	template <> VideoBuffer<byte>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<byte>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<unsigned short>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<unsigned short>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<yuv411>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<yuv411>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<yuv422>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<yuv422>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<bayer_grbg>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<bayer_grbg>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<bayer_gbrg>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<bayer_gbrg>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<bayer_rggb>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<bayer_rggb>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<bayer_bggr>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<bayer_bggr>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<bayer_grbg16be>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<bayer_grbg16be>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<bayer_gbrg16be>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<bayer_gbrg16be>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<bayer_rggb16be>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<bayer_rggb16be>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<bayer_bggr16be>* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<bayer_bggr16be>(cam,size, fps, offset, v, r, m);
	}

	template <> VideoBuffer<Rgb<byte> >* makeDVBuffer2(int cam, ImageRef size, float fps, ImageRef offset,  bool v, bool r, int m)
	{
		return new DVBuffer3<Rgb<byte> >(cam, size, fps, offset, v, r, m);
	}

}
