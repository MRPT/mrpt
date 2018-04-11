#include <cvd/videosource.h>
namespace CVD{

	template <> VideoBuffer<byte>* makeVideoFileBuffer(const std::string&, VideoBufferFlags::OnEndOfBuffer, bool, const std::string&, const std::map<std::string,std::string>&)
	{
		throw VideoSourceException("VideoFileBuffer is not compiled in to libcvd.");
	}

	template <> VideoBuffer<Rgb<byte> >* makeVideoFileBuffer(const std::string&, VideoBufferFlags::OnEndOfBuffer, bool, const std::string&, const std::map<std::string,std::string>&)
	{
		throw VideoSourceException("VideoFileBuffer is not compiled in to libcvd.");
	}
}
