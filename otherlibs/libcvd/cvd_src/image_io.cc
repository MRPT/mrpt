#include "cvd/image_io.h"
#include "cvd/internal/load_and_save.h"


#include <sstream>
#include <fstream>
#include <cstring>

using namespace std;
namespace CVD{

Exceptions::Image_IO::ImageSizeMismatch::ImageSizeMismatch(const ImageRef& src, const ImageRef& dest)
{
	ostringstream o;
	o <<
"Image load: Size mismatch when loading an image (size " << src << ") in to a non\
resizable image (size " << dest << ").";

	what = o.str();
}

Exceptions::Image_IO::OpenError::OpenError(const string& name, const string& why, int error)
{
	what = "Opening file: " + name+ " (" + why + "): " + strerror(error);
}

Exceptions::Image_IO::MalformedImage::MalformedImage(const string& why)
{
	what = "Image input: " + why;
}

Exceptions::Image_IO::UnsupportedImageType::UnsupportedImageType()
{
	what = "Image input: Unsuppported image type.";
}

Exceptions::Image_IO::IfstreamNotOpen::IfstreamNotOpen()
{
	what = "Image input: File stream has not been opened succesfully.";
}

Exceptions::Image_IO::EofBeforeImage::EofBeforeImage()
{
	what = "Image input: End of file occured before image.";
}

Exceptions::Image_IO::WriteError::WriteError(const string& s)
{
	what = "Error writing " + s;
}

Exceptions::Image_IO::WriteTypeMismatch::WriteTypeMismatch(const string& avail, const string& req)
{
	what = "Image output (CVD internal error): Attempting to write " + req + " data to a file containing " + avail;
}

Exceptions::Image_IO::ReadTypeMismatch::ReadTypeMismatch(const string& avail, const string& req)
{
	what = "Image input (CVD internal error): Attempting to read " + req + " data from a file containing " + avail;
}

Exceptions::Image_IO::ReadTypeMismatch::ReadTypeMismatch(const bool read8)
{
	what = string("Image input (CVD internal error): Attempting to read ") + (read8?"8":"16") + "bit data from " + (read8?"16":"8")  + "bit file (probably an internal error).";
}

Exceptions::Image_IO::UnseekableIstream::UnseekableIstream(const string& s)
{
	what = "Image input: Loading " + s + " images requires seekable istream.";
}

Exceptions::Image_IO::UnsupportedImageSubType::UnsupportedImageSubType(const string& i, const string& why)
{
	what = "Image input: Unsupported subtype of " + i+ " image: " + why;
}

Exceptions::Image_IO::InternalLibraryError::InternalLibraryError(const std::string& l, const std::string e)
{
	what = "Internal error in " + l + " library: " + e;
}

ImageType::ImageType string_to_image_type(const std::string& name)
{ 
	size_t dot = name.rfind('.');
	if (dot == std::string::npos)
		return  ImageType::PNM;
	
	std::string suffix = name.substr(dot+1,name.length()-dot-1);
	for (size_t i=0; i<suffix.length(); i++)
		suffix[i] = tolower(suffix[i]);


	if (suffix == "ps") 
	        return  ImageType::PS;
#ifdef CVD_HAVE_JPEG
	else if (suffix == "jpg" || suffix == "jpeg")
	         return  ImageType::JPEG;
#endif
#ifdef CVD_HAVE_PNG
	else if (suffix == "png")
	         return  ImageType::PNG;
#endif
#ifdef CVD_HAVE_TIFF
	else if (suffix == "tif" || suffix == "tiff")
	         return  ImageType::TIFF;
#endif
	else if (suffix == "eps")
		return  ImageType::EPS;
	else if (suffix == "bmp") 
		return  ImageType::BMP;
	else if (suffix == "pnm" || suffix == "ppm" || suffix == "pgm" || suffix == "pbm") 
		return  ImageType::PNM;
	else if (suffix == "txt")
		return ImageType::TXT;
	else if (suffix == "fits" || suffix == "fts")
		return ImageType::FITS;
	else if (suffix == "cvd")
		return ImageType::CVD;
	else
		return ImageType::Unknown;
}


Internal::ImagePromise<Internal::ImageLoaderIstream> img_load(std::istream& i)
{
	return Internal::ImagePromise<Internal::ImageLoaderIstream>(i);
}

Internal::ImagePromise<Internal::ImageLoaderString> img_load(const std::string &s)
{
	return Internal::ImagePromise<Internal::ImageLoaderString>(s);
}




}
