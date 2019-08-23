/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers

#include <mrpt/core/round.h>  // for round()
#include <mrpt/img/CImage.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/zip.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/fourier.h>
#include <mrpt/math/utils.h>  // for roundup()
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/memory.h>
#include <iostream>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

#include "CImage_impl.h"

#if MRPT_HAS_MATLAB
#include <mexplus/mxarray.h>
#endif

// Prototypes of SSE2/SSE3/SSSE3 optimized functions:
#include "CImage_SSEx.h"

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CImage, CSerializable, mrpt::img)

static bool DISABLE_ZIP_COMPRESSION_value = false;
static bool DISABLE_JPEG_COMPRESSION_value = true;
static int SERIALIZATION_JPEG_QUALITY_value = 95;
static std::string IMAGES_PATH_BASE(".");

void CImage::DISABLE_ZIP_COMPRESSION(bool val)
{
	DISABLE_ZIP_COMPRESSION_value = val;
}
bool CImage::DISABLE_ZIP_COMPRESSION() { return DISABLE_ZIP_COMPRESSION_value; }
void CImage::DISABLE_JPEG_COMPRESSION(bool val)
{
	DISABLE_JPEG_COMPRESSION_value = val;
}
bool CImage::DISABLE_JPEG_COMPRESSION()
{
	return DISABLE_JPEG_COMPRESSION_value;
}
void CImage::SERIALIZATION_JPEG_QUALITY(int q)
{
	SERIALIZATION_JPEG_QUALITY_value = q;
}
int CImage::SERIALIZATION_JPEG_QUALITY()
{
	return SERIALIZATION_JPEG_QUALITY_value;
}

CExceptionExternalImageNotFound::CExceptionExternalImageNotFound(
	const std::string& s)
	: std::runtime_error(s)
{
}

const std::string& CImage::getImagesPathBase() { return IMAGES_PATH_BASE; }
void CImage::setImagesPathBase(const std::string& path)
{
	IMAGES_PATH_BASE = path;
}

// Do performance time logging?
#define IMAGE_ALLOC_PERFLOG 0

#if IMAGE_ALLOC_PERFLOG
mrpt::img::CTimeLogger alloc_tims;
#endif

#if MRPT_HAS_OPENCV
static int interpolationMethod2Cv(TInterpolationMethod i)
{
	// clang-format off
	switch (i)
	{
	    case IMG_INTERP_NN:     return cv::INTER_NEAREST;
	    case IMG_INTERP_LINEAR: return cv::INTER_LINEAR;
	    case IMG_INTERP_CUBIC:  return cv::INTER_CUBIC;
	    case IMG_INTERP_AREA:   return cv::INTER_AREA;
	};
	// clang-format on
	return -1;
}

template <typename RET = uint32_t>
constexpr RET pixelDepth2CvDepth(PixelDepth d)
{
	// clang-format off
	switch (d)
	{
	    case PixelDepth::D8U:  return static_cast<RET>(CV_8U);
	    case PixelDepth::D8S:  return static_cast<RET>(CV_8S);
	    case PixelDepth::D16U: return static_cast<RET>(CV_16U);
	    case PixelDepth::D16S: return static_cast<RET>(CV_16S);
	    case PixelDepth::D32S: return static_cast<RET>(CV_32S);
	    case PixelDepth::D32F: return static_cast<RET>(CV_32F);
	    case PixelDepth::D64F: return static_cast<RET>(CV_64F);
	}
	// clang-format on
	return std::numeric_limits<RET>::max();
}
template <typename RET = uint32_t>
RET pixelDepth2IPLCvDepth(PixelDepth d)
{
	// clang-format off
	switch (d)
	{
	    case PixelDepth::D8U:  return static_cast<RET>(IPL_DEPTH_8U);
	    case PixelDepth::D8S:  return static_cast<RET>(IPL_DEPTH_8S);
	    case PixelDepth::D16U: return static_cast<RET>(IPL_DEPTH_16U);
	    case PixelDepth::D16S: return static_cast<RET>(IPL_DEPTH_16S);
	    case PixelDepth::D32S: return static_cast<RET>(IPL_DEPTH_32S);
	    case PixelDepth::D32F: return static_cast<RET>(IPL_DEPTH_32F);
	    case PixelDepth::D64F: return static_cast<RET>(IPL_DEPTH_64F);
	}
	// clang-format on
	return std::numeric_limits<RET>::max();
}

static PixelDepth cvDepth2PixelDepth(int64_t d)
{
	// clang-format off
	switch (d)
	{
	    case CV_8U:  return PixelDepth::D8U;
	    case CV_8S:  return PixelDepth::D8S;
	    case CV_16U: return PixelDepth::D16U;
	    case CV_16S: return PixelDepth::D16S;
	    case CV_32S: return PixelDepth::D32S;
	    case CV_32F: return PixelDepth::D32F;
	    case CV_64F: return PixelDepth::D64F;
	}
	// clang-format on
	return PixelDepth::D8U;
}

#endif  // MRPT_HAS_OPENCV

// Default ctor
CImage::CImage() : m_impl(mrpt::make_impl<CImage::Impl>()) {}

// Ctor with size
CImage::CImage(
	unsigned int width, unsigned int height, TImageChannels nChannels)
	: CImage()
{
	MRPT_START
	resize(width, height, nChannels);
	MRPT_END
}

void CImage::swap(CImage& o)
{
	std::swap(m_impl, o.m_impl);
	std::swap(m_imgIsExternalStorage, o.m_imgIsExternalStorage);
	std::swap(m_externalFile, o.m_externalFile);
}

void CImage::copyFromForceLoad(const CImage& o)
{
	*this = o;
	forceLoad();
}

CImage::CImage(const cv::Mat& img, copy_type_t copy_type) : CImage()
{
#if MRPT_HAS_OPENCV
	MRPT_START
	if (copy_type == DEEP_COPY)
		m_impl->img = img.clone();
	else
		m_impl->img = img;
	MRPT_END
#endif
}

CImage::CImage(const CImage& img, copy_type_t copy_type)
	:
#if MRPT_HAS_OPENCV
	  CImage(img.m_impl->img, copy_type)
#else
	  CImage()
#endif
{
}

CImage CImage::makeDeepCopy() const
{
#if MRPT_HAS_OPENCV
	CImage ret(*this);
	ret.m_impl->img = m_impl->img.clone();
	return ret;
#else
	THROW_EXCEPTION("Operation not supported: build MRPT against OpenCV!");
#endif
}

void CImage::asCvMat(cv::Mat& out_img, copy_type_t copy_type) const
{
#if MRPT_HAS_OPENCV
	if (copy_type == DEEP_COPY)
		out_img = m_impl->img.clone();
	else
		out_img = m_impl->img;
#endif
}

cv::Mat& CImage::asCvMatRef()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();
	return m_impl->img;
#else
	THROW_EXCEPTION("Operation not supported: build MRPT against OpenCV!");
#endif
}

const cv::Mat& CImage::asCvMatRef() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();
	return m_impl->img;
#else
	THROW_EXCEPTION("Operation not supported: build MRPT against OpenCV!");
#endif
}

void CImage::resize(
	std::size_t width, std::size_t height, TImageChannels nChannels,
	PixelDepth depth)
{
	MRPT_START

#if MRPT_HAS_OPENCV
	// Dont call makeSureImageIsLoaded() here,
	// since it will throw if resize() is called from a ctor, where it's
	// legit for the img to be uninitialized.

	// If we're resizing to exactly the current size, do nothing:
	{
		_IplImage ipl = m_impl->img;

		if (static_cast<unsigned>(ipl.width) == width &&
			static_cast<unsigned>(ipl.height) == height &&
			ipl.nChannels == nChannels &&
			static_cast<unsigned>(ipl.depth) == pixelDepth2IPLCvDepth(depth))
		{
			// Nothing to do:
			return;
		}
	}

#if IMAGE_ALLOC_PERFLOG
	const std::string sLog = mrpt::format("cvCreateImage %ux%u", width, height);
	alloc_tims.enter(sLog.c_str());
#endif

	static_assert(
		pixelDepth2CvDepth<int>(PixelDepth::D8U) + CV_8UC(3) == CV_8UC3);

	m_impl->img = cv::Mat(
		static_cast<int>(height), static_cast<int>(width),
		pixelDepth2CvDepth<int>(depth) + ((nChannels - 1) << CV_CN_SHIFT));

#if IMAGE_ALLOC_PERFLOG
	alloc_tims.leave(sLog.c_str());
#endif

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

PixelDepth CImage::getPixelDepth() const
{
	MRPT_START
#if MRPT_HAS_OPENCV
	return cvDepth2PixelDepth(m_impl->img.depth());
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

bool CImage::loadFromFile(const std::string& fileName, int isColor)
{
	MRPT_START

#if MRPT_HAS_OPENCV
	m_imgIsExternalStorage = false;
#ifdef HAVE_OPENCV_IMGCODECS
	MRPT_TODO("Port to cv::imdecode()?");
	MRPT_TODO("add flag to reuse current img buffer");

	m_impl->img = cv::imread(fileName, static_cast<cv::ImreadModes>(isColor));
#else
	IplImage* newImg = cvLoadImage(fileName.c_str(), isColor);
	if (!newImg) return false;
	m_impl->img = cv::cvarrToMat(newImg);
#endif
	if (m_impl->img.empty()) return false;

	return true;
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

bool CImage::saveToFile(const std::string& fileName, int jpeg_quality) const
{
	MRPT_START
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	ASSERT_(!m_impl->img.empty());

#ifdef HAVE_OPENCV_IMGCODECS
	const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
	return cv::imwrite(fileName, m_impl->img, params);
#else
	int p[3] = {CV_IMWRITE_JPEG_QUALITY, jpeg_quality, 0};
	_IplImage ipl = m_impl->img;
	return (0 != cvSaveImage(fileName.c_str(), &ipl, p));
#endif
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

void CImage::internal_fromIPL(const IplImage* iplImage, copy_type_t c)
{
	MRPT_START
#if MRPT_HAS_OPENCV
	ASSERT_(iplImage != nullptr);
	clear();
	m_impl->img =
		cv::cvarrToMat(iplImage, c == DEEP_COPY ? true : false /*copyData*/);
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

void CImage::loadFromMemoryBuffer(
	unsigned int width, unsigned int height, bool color,
	unsigned char* rawpixels, bool swapRedBlue)
{
	MRPT_START

#if MRPT_HAS_OPENCV
	resize(width, height, color ? CH_RGB : CH_GRAY);
	m_imgIsExternalStorage = false;

	_IplImage ii(m_impl->img);
	IplImage* img = &ii;

	if (color && swapRedBlue)
	{
		// Do copy & swap at once:
		unsigned char* ptr_src = rawpixels;
		auto* ptr_dest = reinterpret_cast<unsigned char*>(img->imageData);
		const int bytes_per_row_out = img->widthStep;

		for (int h = height; h--;)
		{
			for (unsigned int i = 0; i < width;
				 i++, ptr_src += 3, ptr_dest += 3)
			{
				unsigned char t0 = ptr_src[0], t1 = ptr_src[1], t2 = ptr_src[2];
				ptr_dest[2] = t0;
				ptr_dest[1] = t1;
				ptr_dest[0] = t2;
			}
			ptr_dest += bytes_per_row_out - width * 3;
		}
	}
	else
	{
		if (img->widthStep == img->width * img->nChannels)
		{
			// Copy the image data:
			memcpy(img->imageData, rawpixels, img->imageSize);
		}
		else
		{
			// Copy the image row by row:
			unsigned char* ptr_src = rawpixels;
			auto* ptr_dest = reinterpret_cast<unsigned char*>(img->imageData);
			int bytes_per_row = width * (color ? 3 : 1);
			int bytes_per_row_out = img->widthStep;
			for (unsigned int y = 0; y < height; y++)
			{
				memcpy(ptr_dest, ptr_src, bytes_per_row);
				ptr_src += bytes_per_row;
				ptr_dest += bytes_per_row_out;
			}
		}
	}
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

unsigned char* CImage::operator()(
	unsigned int ucol, unsigned int urow, unsigned int uchannel) const
{
#if MRPT_HAS_OPENCV

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_START
#endif

	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	const auto col = static_cast<int>(ucol);
	const auto row = static_cast<int>(urow);
	const auto channel = static_cast<int>(uchannel);

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	ASSERT_(m_impl && !m_impl->img.empty());
	if (row >= m_impl->img.rows || col >= m_impl->img.cols ||
		channel >= m_impl->img.channels())
	{
		THROW_EXCEPTION(format(
			"Pixel coordinates/channel out of bounds: row=%u/%u col=%u/%u "
			"chan=%u/%u",
			row, m_impl->img.rows, col, m_impl->img.cols, channel,
			m_impl->img.channels()));
	}
#endif
	auto p =
		(&m_impl->img.at<uint8_t>(row, m_impl->img.channels() * col)) + channel;
	return const_cast<unsigned char*>(p);
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_END
#endif

#else
	THROW_EXCEPTION("MRPT was compiled without OpenCV");
#endif
}

uint8_t* CImage::internal_get(int col, int row, uint8_t channel) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	auto p =
		(&m_impl->img.at<uint8_t>(row, m_impl->img.channels() * col)) + channel;
	return const_cast<uint8_t*>(p);
#else
	return nullptr;
#endif
}

uint8_t* CImage::get_unsafe(
	unsigned int col, unsigned int row, uint8_t channel) const
{
	return internal_get(col, row, channel);
}

uint8_t CImage::serializeGetVersion() const
{
#if !MRPT_HAS_OPENCV
	return 100;
#else
	return 9;
#endif
}
void CImage::serializeTo(mrpt::serialization::CArchive& out) const
{
#if !MRPT_HAS_OPENCV
	out << m_imgIsExternalStorage;
	if (m_imgIsExternalStorage) out << m_externalFile;
// Nothing else to serialize!
#else
	{
		// Added in version 6: possibility of being stored offline:
		out << m_imgIsExternalStorage;

		if (m_imgIsExternalStorage)
		{
			out << m_externalFile;
		}
		else
		{  // Normal image loaded in memory:
			ASSERT_(m_impl);

			const bool hasColor = m_impl->img.empty() ? false : isColor();

			out << hasColor;

			// Version >2: Color->JPEG, GrayScale->BYTE's array!
			const int32_t width = m_impl->img.cols;
			const int32_t height = m_impl->img.rows;
			if (!hasColor)
			{
				// GRAY-SCALE: Raw bytes:
				// Version 3: ZIP compression!
				// Version 4: Skip zip if the image size <= 16Kb
				int32_t origin = 0;  // not used mrpt v1.9.9
				uint32_t imageSize = height * m_impl->img.step[0];
				// Version 10: depth
				int32_t depth = m_impl->img.depth();

				out << width << height << origin << imageSize
					<< int32_t(cvDepth2PixelDepth(depth));

				// Version 5: Use CImage::DISABLE_ZIP_COMPRESSION
				bool imageStoredAsZip = !CImage::DISABLE_ZIP_COMPRESSION() &&
										(imageSize > 16 * 1024);

				out << imageStoredAsZip;

				// Version 4: Skip zip if the image size <= 16Kb
				if (imageStoredAsZip)
				{
					std::vector<unsigned char> tempBuf;
					mrpt::io::zip::compress(
						m_impl->img.data, imageSize, tempBuf);

					auto zipDataLen = static_cast<int32_t>(tempBuf.size());
					out << zipDataLen;

					out.WriteBuffer(&tempBuf[0], tempBuf.size());
					tempBuf.clear();
				}
				else
				{
					if (imageSize > 0 && m_impl->img.data != nullptr)
						out.WriteBuffer(m_impl->img.data, imageSize);
				}
			}
			else
			{
				// COLOR: High quality JPEG image

				// v7: If size is 0xN or Nx0, don't call
				// "saveToStreamAsJPEG"!!

				// v8: If DISABLE_JPEG_COMPRESSION
				if (!CImage::DISABLE_JPEG_COMPRESSION())
				{
					// normal behavior: compress images:
					out << width << height;

					if (width >= 1 && height >= 1)
					{
						// Save to temporary memory stream:
						mrpt::io::CMemoryStream aux;
						saveToStreamAsJPEG(
							aux, CImage::SERIALIZATION_JPEG_QUALITY());

						const auto nBytes =
							static_cast<uint32_t>(aux.getTotalBytesCount());

						out << nBytes;
						out.WriteBuffer(aux.getRawBufferData(), nBytes);
					}
				}
				else
				{  // (New in v8)
					// Don't JPEG-compress behavior:
					// Use negative image sizes to signal this behavior:
					const int32_t neg_width = -width;
					const int32_t neg_height = -height;

					out << neg_width << neg_height;

					// Dump raw image data:
					const auto bytes_per_row = width * 3;

					out.WriteBuffer(m_impl->img.data, bytes_per_row * height);
				}
			}
		}  // end m_imgIsExternalStorage=false
	}
#endif
}

void CImage::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
#if !MRPT_HAS_OPENCV
	if (version == 100)
	{
		in >> m_imgIsExternalStorage;
		if (m_imgIsExternalStorage)
			in >> m_externalFile;
		else
		{
			THROW_EXCEPTION(
				"[CImage] Cannot deserialize image since MRPT has been "
				"compiled without OpenCV");
		}
	}
#else
	// First, free current image.
	clear();

	switch (version)
	{
		case 100:  // Saved from an MRPT build without OpenCV:
		{
			in >> m_imgIsExternalStorage;
			if (m_imgIsExternalStorage) in >> m_externalFile;
		}
		break;
		case 0:
		{
			uint32_t width, height, nChannels, imgLength;
			uint8_t originTopLeft;

			in >> width >> height >> nChannels >> originTopLeft >> imgLength;

			resize(width, height, static_cast<TImageChannels>(nChannels));
			in.ReadBuffer(m_impl->img.data, imgLength);
		}
		break;
		case 1:
		{
			// Version 1: High quality JPEG image
			mrpt::io::CMemoryStream aux;
			uint32_t nBytes;
			in >> nBytes;
			aux.changeSize(nBytes + 10);
			in.ReadBuffer(aux.getRawBufferData(), nBytes);
			aux.Seek(0);
			loadFromStreamAsJPEG(aux);
		}
		break;
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		{
			// Version 6: 	m_imgIsExternalStorage ??
			if (version >= 6)
				in >> m_imgIsExternalStorage;
			else
				m_imgIsExternalStorage = false;

			if (m_imgIsExternalStorage)
			{
				// Just the file name:
				in >> m_externalFile;
			}
			else
			{  // Normal, the whole image data:

				// Version 2: Color->JPEG, GrayScale->BYTE's array!
				uint8_t hasColor;
				in >> hasColor;
				if (!hasColor)
				{
					// GRAY SCALE:
					int32_t width, height, origin, imageSize;
					in >> width >> height >> origin >> imageSize;
					PixelDepth depth(PixelDepth::D8U);
					if (version >= 9)
					{
						int32_t tempdepth;
						in >> tempdepth;
						depth = PixelDepth(tempdepth);
					}
					resize(
						static_cast<uint32_t>(width),
						static_cast<uint32_t>(height), CH_GRAY, depth);
					ASSERT_(
						static_cast<uint32_t>(imageSize) ==
						static_cast<uint32_t>(width) *
							static_cast<uint32_t>(height) *
							m_impl->img.step[0]);

					if (version == 2)
					{
						// RAW BYTES:
						in.ReadBuffer(m_impl->img.data, imageSize);
					}
					else
					{
						// Version 3: ZIP compression!
						bool imageIsZIP = true;

						// Version 4: Skip zip if the image size <= 16Kb
						// Version 5: Use CImage::DISABLE_ZIP_COMPRESSION
						if (version == 4 && imageSize <= 16 * 1024)
							imageIsZIP = false;

						if (version >= 5)
						{
							// It is stored int the stream:
							in >> imageIsZIP;
						}

						if (imageIsZIP)
						{
							uint32_t zipDataLen;
							in >> zipDataLen;

#if 0
						size_t outDataBufferSize = imageSize;
						size_t outDataActualSize;
						mrpt::io::zip::decompress(
						    in, zipDataLen, m_impl->img.data,
							outDataBufferSize, outDataActualSize);
						ASSERT_(outDataActualSize == outDataBufferSize);
#else
							THROW_EXCEPTION(
								"ZIP image deserialization not "
								"implemented");
#endif
						}
						else
						{
							// Raw bytes:
							if (imageSize)
								in.ReadBuffer(m_impl->img.data, imageSize);
						}
					}
				}
				else
				{
					bool loadJPEG = true;

					if (version >= 7)
					{
						int32_t width, height;
						in >> width >> height;

						if (width >= 1 && height >= 1)
						{
							loadJPEG = true;
						}
						else
						{
							loadJPEG = false;

							if (width < 0 && height < 0)
							{
								// v8: raw image:
								const int32_t real_w = -width;
								const int32_t real_h = -height;

								resize(real_w, real_h, CH_RGB);

								auto& img = m_impl->img;
								const size_t bytes_per_row = img.cols * 3;
								for (int y = 0; y < img.rows; y++)
								{
									const size_t nRead = in.ReadBuffer(
										img.ptr<void>(y), bytes_per_row);
									if (nRead != bytes_per_row)
										THROW_EXCEPTION(
											"Error: Truncated data stream "
											"while parsing raw image?");
								}
							}
							else
							{
								// it's a 0xN or Nx0 image: just resize and
								// load nothing:
								resize(width, height, CH_RGB);
							}
						}
					}

					// COLOR IMAGE: JPEG
					if (loadJPEG)
					{
						mrpt::io::CMemoryStream aux;
						uint32_t nBytes;
						in >> nBytes;
						aux.changeSize(nBytes + 10);
						in.ReadBuffer(aux.getRawBufferData(), nBytes);
						aux.Seek(0);
						loadFromStreamAsJPEG(aux);
					}
				}
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
#endif
}

/*---------------------------------------------------------------
Implements the writing to a mxArray for Matlab
---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM(mrpt::img::CImage)
#endif

mxArray* CImage::writeToMatlab() const
{
#if MRPT_HAS_MATLAB
	return mexplus::from(this->asCvMatRef());
#else
	THROW_EXCEPTION("MRPT built without MATLAB/Mex support");
#endif
}

void CImage::getSize(TImageSize& s) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	s.x = m_impl->img.cols;
	s.y = m_impl->img.rows;
#else
	THROW_EXCEPTION("MRPT built without OpenCV support");
#endif
}

size_t CImage::getWidth() const
{
#if MRPT_HAS_OPENCV
	if (m_imgIsExternalStorage) makeSureImageIsLoaded();
	return m_impl->img.cols;
#else
	return 0;
#endif
}

std::string CImage::getChannelsOrder() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	IplImage ipl(m_impl->img);
	return std::string(ipl.channelSeq);
#else
	THROW_EXCEPTION("MRPT built without OpenCV support");
#endif
}

size_t CImage::getRowStride() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	return m_impl->img.step[0];
#else
	THROW_EXCEPTION("MRPT built without OpenCV support");
#endif
}

size_t CImage::getHeight() const
{
#if MRPT_HAS_OPENCV
	if (m_imgIsExternalStorage) makeSureImageIsLoaded();
	return m_impl->img.rows;
#else
	return 0;
#endif
}

bool CImage::isColor() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	return m_impl->img.channels() == 3;
#else
	THROW_EXCEPTION("MRPT built without OpenCV support");
#endif
}

bool CImage::isEmpty() const
{
#if MRPT_HAS_OPENCV
	return m_imgIsExternalStorage || m_impl->img.empty();
#else
	THROW_EXCEPTION("MRPT built without OpenCV support");
#endif
}

TImageChannels CImage::getChannelCount() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	return static_cast<TImageChannels>(m_impl->img.channels());
#else
	THROW_EXCEPTION("MRPT built without OpenCV support");
#endif
}

bool CImage::isOriginTopLeft() const
{
	return true;  // As of mrpt v1.9.9
}

float CImage::getAsFloat(
	unsigned int col, unsigned int row, unsigned int channel) const
{
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	// [0,255]->[0,1]
	return (*(*this)(col, row, channel)) / 255.0f;
}

float CImage::getAsFloat(unsigned int col, unsigned int row) const
{
	// Is a RGB image??
	if (isColor())
	{
		// Luminance: Y = 0.3R + 0.59G + 0.11B
		unsigned char* pixels = (*this)(col, row, 0);
		return (pixels[0] * 0.3f + pixels[1] * 0.59f + pixels[2] * 0.11f) /
			   255.0f;
	}
	else
	{
		// [0,255]->[0,1]
		return (*(*this)(col, row, 0 /* Channel 0:Gray level */)) / 255.0f;
	}
}

/*---------------------------------------------------------------
					getMaxAsFloat
---------------------------------------------------------------*/
float CImage::getMaxAsFloat() const
{
	int x, y, cx = getWidth(), cy = getHeight();

	float maxPixel = 0;

	for (x = 0; x < cx; x++)
		for (y = 0; y < cy; y++) maxPixel = max(maxPixel, getAsFloat(x, y));

	return maxPixel;
}

CImage CImage::grayscale() const
{
	CImage ret;
	grayscale(ret);
	return ret;
}

// Auxiliary function for both ::grayscale() and ::grayscaleInPlace()
#if MRPT_HAS_OPENCV
static bool my_img_to_grayscale(const cv::Mat& src, cv::Mat& dest)
{
	if (dest.size() != src.size() || dest.type() != src.type())
		dest = cv::Mat(src.rows, src.cols, CV_8UC1);

// If possible, use SSE optimized version:
#if MRPT_HAS_SSE3
	if ((src.step[0] & 0x0f) == 0 && (dest.step[0] & 0x0f) == 0)
	{
		image_SSSE3_bgr_to_gray_8u(
			src.ptr<uint8_t>(), dest.ptr<uint8_t>(), src.cols, src.rows,
			src.step[0], dest.step[0]);
		return true;
	}
#endif
	// OpenCV Method:
	cv::cvtColor(src, dest, CV_BGR2GRAY);
	return false;
}
#endif

bool CImage::grayscale(CImage& ret) const
{
#if MRPT_HAS_OPENCV
	// The image is already grayscale??
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	if (m_impl->img.channels() == 1)
	{
		ret = *this;  // shallow copy
		return true;
	}
	else
	{
		// Convert to a single luminance channel image
		cv::Mat src = m_impl->img;
		// Detect in-place op and make deep copy:
		if (src.data == ret.m_impl->img.data) src = src.clone();

		return my_img_to_grayscale(src, ret.m_impl->img);
	}
#else
	THROW_EXCEPTION("Operation not supported: build MRPT against OpenCV!");
#endif
}

bool CImage::scaleHalf(CImage& out, TInterpolationMethod interp) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	// Get this image size:
	auto& img = m_impl->img;
	const int w = img.cols, h = img.rows;

	// Create target image:
	out.resize(w >> 1, h >> 1, getChannelCount());
	auto& img_out = out.m_impl->img;

	// If possible, use SSE optimized version:
#if MRPT_HAS_SSE3
	if (img.channels() == 3 && interp == IMG_INTERP_NN)
	{
		image_SSSE3_scale_half_3c8u(
			img.data, img_out.data, w, h, img.step[0], img_out.step[0]);
		return true;
	}
#endif
#if MRPT_HAS_SSE2
	if (img.channels() == 1)
	{
		if (interp == IMG_INTERP_NN)
		{
			image_SSE2_scale_half_1c8u(
				img.data, img_out.data, w, h, img.step[0], img_out.step[0]);
			return true;
		}
		else if (interp == IMG_INTERP_LINEAR)
		{
			image_SSE2_scale_half_smooth_1c8u(
				img.data, img_out.data, w, h, img.step[0], img_out.step[0]);
			return true;
		}
	}
#endif

	// Fall back to slow method:
	cv::resize(
		img, img_out, img_out.size(), 0, 0, interpolationMethod2Cv(interp));
	return false;
#else
	THROW_EXCEPTION("Operation not supported: build MRPT against OpenCV!");
#endif
}

void CImage::scaleDouble(CImage& out, TInterpolationMethod interp) const
{
	out = *this;
	const TImageSize siz = this->getSize();
	out.scaleImage(out, siz.x * 2, siz.y * 2, interp);
}

void CImage::loadFromMemoryBuffer(
	unsigned int width, unsigned int height, unsigned int bytesPerRow,
	unsigned char* red, unsigned char* green, unsigned char* blue)
{
#if MRPT_HAS_OPENCV
	MRPT_START

	resize(width, height, CH_RGB, PixelDepth::D8U);

	// Copy the image data:
	for (unsigned int y = 0; y < height; y++)
	{
		// The target pixels:
		auto* dest = m_impl->img.ptr<uint8_t>(y);

		// Source channels:
		unsigned char* srcR = red + bytesPerRow * y;
		unsigned char* srcG = green + bytesPerRow * y;
		unsigned char* srcB = blue + bytesPerRow * y;

		for (unsigned int x = 0; x < width; x++)
		{
			*(dest++) = *(srcB++);
			*(dest++) = *(srcG++);
			*(dest++) = *(srcR++);
		}  // end of x
	}  // end of y

	MRPT_END
#endif
}

void CImage::setPixel(int x, int y, size_t color)
{
#if MRPT_HAS_OPENCV

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_START
#endif

	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	auto& img = m_impl->img;

	ASSERT_(this->getPixelDepth() == mrpt::img::PixelDepth::D8U);

	if (x >= 0 && y >= 0 && y < img.rows && x < img.cols)
	{
		// The pixel coordinates is valid:
		if (img.channels() == 1)
		{
			img.ptr<uint8_t>(y)[x] = static_cast<uint8_t>(color);
		}
		else
		{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
			ASSERT_(img.channels() == 3);
#endif
			auto* dest = &img.ptr<uint8_t>(y)[3 * x];
			const auto* src = reinterpret_cast<uint8_t*>(&color);
			// Copy the color:
			*dest++ = *src++;  // R
			*dest++ = *src++;  // G
			*dest++ = *src++;  // B
		}
	}

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_END
#endif

#endif
}

void CImage::line(
	int x0, int y0, int x1, int y1, const mrpt::img::TColor color,
	unsigned int width, [[maybe_unused]] TPenStyle penStyle)
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally

	cv::line(
		m_impl->img, cv::Point(x0, y0), cv::Point(x1, y1),
		CV_RGB(color.R, color.G, color.B), static_cast<int>(width));
#endif
}

void CImage::drawCircle(
	int x, int y, int radius, const mrpt::img::TColor& color,
	unsigned int width)
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	cv::circle(
		m_impl->img, cv::Point(x, y), radius, CV_RGB(color.R, color.G, color.B),
		static_cast<int>(width));
#endif
}

void CImage::drawImage(int x, int y, const mrpt::img::CImage& img)
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();
	img.makeSureImageIsLoaded();

	cv::Rect roi(cv::Point(x, y), cv::Size(img.getWidth(), img.getHeight()));
	cv::Mat dest = m_impl->img(roi);
	img.m_impl->img.copyTo(dest);
#endif
}

void CImage::update_patch(
	const CImage& patch, const unsigned int col_, const unsigned int row_)
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	const auto& src = m_impl->img;
	auto& dest = patch.m_impl->img;

	src(cv::Rect(col_, row_, dest.cols, dest.rows)).copyTo(dest);
#endif
}

void CImage::extract_patch(
	CImage& patch, const unsigned int col_, const unsigned int row_,
	const unsigned int col_num, const unsigned int row_num) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	const auto& src = m_impl->img;
	auto& dest = patch.m_impl->img;

	src(cv::Rect(col_, row_, col_num, row_num)).copyTo(dest);
#endif
}

float CImage::correlate(
	const CImage& img2, int width_init, int height_init) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally

	if ((img2.getWidth() + width_init > getWidth()) |
		(img2.getHeight() + height_init > getHeight()))
		THROW_EXCEPTION("Correlation Error!, image to correlate out of bounds");

	float x1, x2;
	float syy = 0.0f, sxy = 0.0f, sxx = 0.0f, m1 = 0.0f, m2 = 0.0f,
		  n = (float)(img2.getHeight() * img2.getWidth());
	//	IplImage *ipl1 = (*this).img;
	//	IplImage *ipl2 = img2.img;

	// find the means
	for (size_t i = 0; i < img2.getHeight(); i++)
	{
		for (size_t j = 0; j < img2.getWidth(); j++)
		{
			m1 += *(*this)(
				j + width_init,
				i + height_init);  //(double)(ipl1->imageData[i*ipl1->widthStep
			//+ j ]);
			m2 += *img2(
				j, i);  //(double)(ipl2->imageData[i*ipl2->widthStep + j ]);
		}  //[ row * ipl->widthStep +  col * ipl->nChannels +  channel ];
	}
	m1 /= n;
	m2 /= n;

	for (size_t i = 0; i < img2.getHeight(); i++)
	{
		for (size_t j = 0; j < img2.getWidth(); j++)
		{
			x1 = *(*this)(j + width_init, i + height_init) -
				 m1;  //(double)(ipl1->imageData[i*ipl1->widthStep
					  //+ j]) - m1;
			x2 = *img2(j, i) - m2;  //(double)(ipl2->imageData[i*ipl2->widthStep
									//+ j]) - m2;
			sxx += x1 * x1;
			syy += x2 * x2;
			sxy += x1 * x2;
		}
	}

	return sxy / sqrt(sxx * syy);
#else
	return 0;
#endif
}

void CImage::normalize()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	cv::normalize(m_impl->img, m_impl->img, 255, 0, cv::NORM_MINMAX);
#endif
}

void CImage::getAsMatrix(
	CMatrixFloat& outMatrix, bool doResize, int x_min, int y_min, int x_max,
	int y_max, bool normalize_01) const
{
#if MRPT_HAS_OPENCV
	MRPT_START
	makeSureImageIsLoaded();  // For delayed loaded images stored externally

	const auto& img = m_impl->img;

	// Set sizes:
	if (x_max == -1) x_max = img.cols - 1;
	if (y_max == -1) y_max = img.rows - 1;

	ASSERT_(x_min >= 0 && x_min < img.cols && x_min < x_max);
	ASSERT_(y_min >= 0 && y_min < img.rows && y_min < y_max);

	int lx = (x_max - x_min + 1);
	int ly = (y_max - y_min + 1);

	if (doResize || outMatrix.rows() < ly || outMatrix.cols() < lx)
		outMatrix.setSize(y_max - y_min + 1, x_max - x_min + 1);

	if (isColor())
	{
		// Luminance: Y = 0.3R + 0.59G + 0.11B
		for (int y = 0; y < ly; y++)
		{
			const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y);
			for (int x = 0; x < lx; x++)
			{
				float aux = *pixels++ * 0.3f;
				aux += *pixels++ * 0.59f;
				aux += *pixels++ * 0.11f;
				if (normalize_01) aux *= (1.0f / 255);
				outMatrix.coeffRef(y, x) = aux;
			}
		}
	}
	else
	{
		for (int y = 0; y < ly; y++)
		{
			const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y);
			for (int x = 0; x < lx; x++)
			{
				float aux = (*pixels++);
				if (normalize_01) aux *= (1.0f / 255);
				outMatrix.coeffRef(y, x) = aux;
			}
		}
	}

	MRPT_END
#endif
}

void CImage::getAsRGBMatrices(
	mrpt::math::CMatrixFloat& R, mrpt::math::CMatrixFloat& G,
	mrpt::math::CMatrixFloat& B, bool doResize, int x_min, int y_min, int x_max,
	int y_max) const
{
#if MRPT_HAS_OPENCV
	MRPT_START

	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	const auto& img = m_impl->img;

	// Set sizes:
	if (x_max == -1) x_max = img.cols - 1;
	if (y_max == -1) y_max = img.rows - 1;

	ASSERT_(x_min >= 0 && x_min < img.cols && x_min < x_max);
	ASSERT_(y_min >= 0 && y_min < img.rows && y_min < y_max);

	int lx = (x_max - x_min + 1);
	int ly = (y_max - y_min + 1);

	if (doResize || R.rows() < ly || R.cols() < lx) R.setSize(ly, lx);
	if (doResize || G.rows() < ly || G.cols() < lx) G.setSize(ly, lx);
	if (doResize || B.rows() < ly || B.cols() < lx) B.setSize(ly, lx);

	if (isColor())
	{
		for (int y = 0; y < ly; y++)
		{
			const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y);
			for (int x = 0; x < lx; x++)
			{
				float aux = *pixels++ * (1.0f / 255);
				R.coeffRef(y, x) = aux;
				aux = *pixels++ * (1.0f / 255);
				G.coeffRef(y, x) = aux;
				aux = *pixels++ * (1.0f / 255);
				B.coeffRef(y, x) = aux;
			}
		}
	}
	else
	{
		for (int y = 0; y < ly; y++)
		{
			const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y);
			for (int x = 0; x < lx; x++)
			{
				R.coeffRef(y, x) = (*pixels) * (1.0f / 255);
				G.coeffRef(y, x) = (*pixels) * (1.0f / 255);
				B.coeffRef(y, x) = (*pixels++) * (1.0f / 255);
			}
		}
	}

	MRPT_END
#endif
}

void CImage::cross_correlation_FFT(
	const CImage& in_img, CMatrixFloat& out_corr, int u_search_ini,
	int v_search_ini, int u_search_size, int v_search_size, float biasThisImg,
	float biasInImg) const
{
	MRPT_START

	makeSureImageIsLoaded();  // For delayed loaded images stored externally

	// Set limits:
	if (u_search_ini == -1) u_search_ini = 0;
	if (v_search_ini == -1) v_search_ini = 0;
	if (u_search_size == -1) u_search_size = static_cast<int>(getWidth());
	if (v_search_size == -1) v_search_size = static_cast<int>(getHeight());

	int u_search_end = u_search_ini + u_search_size - 1;
	int v_search_end = v_search_ini + v_search_size - 1;

	ASSERT_(u_search_end < static_cast<int>(getWidth()));
	ASSERT_(v_search_end < static_cast<int>(getHeight()));

	// Find smallest valid size:
	size_t x, y;
	size_t actual_lx =
		std::max(static_cast<size_t>(u_search_size), in_img.getWidth());
	size_t actual_ly =
		std::max(static_cast<size_t>(v_search_size), in_img.getHeight());
	size_t lx = mrpt::round2up<size_t>(actual_lx);
	size_t ly = mrpt::round2up<size_t>(actual_ly);

	CMatrixF i1(ly, lx), i2(ly, lx);

	// We fill the images with the bias, such as when we substract the bias
	// later on,
	//  those pixels not really occupied by the image really becomes zero:
	i1.fill(biasInImg);
	i2.fill(biasThisImg);

	// Get as matrixes, padded with zeros up to power-of-two sizes:
	getAsMatrix(
		i2, false, u_search_ini, v_search_ini, u_search_ini + u_search_size - 1,
		v_search_ini + v_search_size - 1);
	in_img.getAsMatrix(i1, false);

	// Remove the bias now:
	i2 -= biasThisImg;
	i1 -= biasInImg;

	// FFT:
	CMatrixF I1_R, I1_I, I2_R, I2_I, ZEROS(ly, lx);
	math::dft2_complex(i1, ZEROS, I1_R, I1_I);
	math::dft2_complex(i2, ZEROS, I2_R, I2_I);

	// Compute the COMPLEX division of I2 by I1:
	for (y = 0; y < ly; y++)
		for (x = 0; x < lx; x++)
		{
			float r1 = I1_R(y, x);
			float r2 = I2_R(y, x);

			float ii1 = I1_I(y, x);
			float ii2 = I2_I(y, x);

			float den = square(r1) + square(ii1);
			I2_R(y, x) = (r1 * r2 + ii1 * ii2) / den;
			I2_I(y, x) = (ii2 * r1 - r2 * ii1) / den;
		}

	// IFFT:
	CMatrixF res_R, res_I;
	math::idft2_complex(I2_R, I2_I, res_R, res_I);

	out_corr.setSize(actual_ly, actual_lx);
	for (y = 0; y < actual_ly; y++)
		for (x = 0; x < actual_lx; x++)
			out_corr(y, x) = sqrt(square(res_R(y, x)) + square(res_I(y, x)));

	MRPT_END
}

void CImage::getAsMatrixTiled(CMatrixFloat& outMatrix) const
{
#if MRPT_HAS_OPENCV
	MRPT_START

	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	const auto& img = m_impl->img;

	// The size of the matrix:
	const auto matrix_lx = outMatrix.cols();
	const auto matrix_ly = outMatrix.rows();

	if (isColor())
	{
		// Luminance: Y = 0.3R + 0.59G + 0.11B
		for (CMatrixFloat::Index y = 0; y < matrix_ly; y++)
		{
			unsigned char* min_pixels = (*this)(0, y % img.rows, 0);
			unsigned char* max_pixels = min_pixels + img.cols * 3;
			unsigned char* pixels = min_pixels;
			float aux;
			for (CMatrixFloat::Index x = 0; x < matrix_lx; x++)
			{
				aux = *pixels++ * 0.30f;
				aux += *pixels++ * 0.59f;
				aux += *pixels++ * 0.11f;
				outMatrix(y, x) = aux;
				if (pixels >= max_pixels) pixels = min_pixels;
			}
		}
	}
	else
	{
		for (CMatrixFloat::Index y = 0; y < matrix_ly; y++)
		{
			unsigned char* min_pixels = (*this)(0, y % img.rows, 0);
			unsigned char* max_pixels = min_pixels + img.cols;
			unsigned char* pixels = min_pixels;
			for (CMatrixFloat::Index x = 0; x < matrix_lx; x++)
			{
				outMatrix(y, x) = *pixels++;
				if (pixels >= max_pixels) pixels = min_pixels;
			}
		}
	}

	MRPT_END
#endif
}

void CImage::clear()
{
	// Reset to defaults:
	*this = CImage();
}

void CImage::setExternalStorage(const std::string& fileName) noexcept
{
	clear();
	m_externalFile = fileName;
	m_imgIsExternalStorage = true;
}

void CImage::unload() const noexcept
{
#if MRPT_HAS_OPENCV
	if (m_imgIsExternalStorage) const_cast<cv::Mat&>(m_impl->img) = cv::Mat();
#endif
}

void CImage::makeSureImageIsLoaded() const
{
#if MRPT_HAS_OPENCV
	if (!m_impl->img.empty()) return;  // OK, continue
#endif

	if (m_imgIsExternalStorage)
	{
		// Load the file:
		string wholeFile;
		getExternalStorageFileAbsolutePath(wholeFile);

		const std::string tmpFile = m_externalFile;

		bool ret = const_cast<CImage*>(this)->loadFromFile(wholeFile);

		// These are removed by "loadFromFile", and that's good, just fix it
		// here and carry on.
		m_imgIsExternalStorage = true;
		m_externalFile = tmpFile;

		if (!ret)
			THROW_TYPED_EXCEPTION_FMT(
				CExceptionExternalImageNotFound,
				"Error loading externally-stored image from: %s",
				wholeFile.c_str());
	}
	else
	{
		THROW_EXCEPTION(
			"Trying to access uninitialized image in a non "
			"externally-stored "
			"image.");
	}
}

void CImage::getExternalStorageFileAbsolutePath(std::string& out_path) const
{
	ASSERT_(m_externalFile.size() > 2);

	if (m_externalFile[0] == '/' ||
		(m_externalFile[1] == ':' &&
		 (m_externalFile[2] == '\\' || m_externalFile[2] == '/')))
	{
		out_path = m_externalFile;
	}
	else
	{
		out_path = IMAGES_PATH_BASE;

		size_t N = IMAGES_PATH_BASE.size() - 1;
		if (IMAGES_PATH_BASE[N] != '/' && IMAGES_PATH_BASE[N] != '\\')
			out_path += "/";

		out_path += m_externalFile;
	}
}

void CImage::flipVertical()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();
	cv::flip(m_impl->img, m_impl->img, 0 /* x-axis */);
#endif
}

void CImage::flipHorizontal()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();
	cv::flip(m_impl->img, m_impl->img, 1 /* y-axis */);
#endif
}

void CImage::swapRB()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	cv::cvtColor(m_impl->img, m_impl->img, cv::COLOR_RGB2BGR);
#endif
}

void CImage::rectifyImageInPlace(void* mapX, void* mapY)
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally

	auto& srcImg = m_impl->img;
	cv::Mat outImg(srcImg.rows, srcImg.cols, srcImg.type());

	auto mapXm = static_cast<cv::Mat*>(mapX);
	auto mapYm = static_cast<cv::Mat*>(mapX);

	cv::remap(srcImg, outImg, *mapXm, *mapYm, cv::INTER_CUBIC);

	clear();
	srcImg = outImg;
#endif
}

void CImage::undistort(
	CImage& out_img, const mrpt::img::TCamera& cameraParams) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally

	ASSERTMSG_(
		out_img.m_impl->img.data != m_impl->img.data,
		"In-place undistort() not supported");

	auto& srcImg = const_cast<cv::Mat&>(m_impl->img);
	// This will avoid re-alloc if size already matches.
	out_img.resize(srcImg.cols, srcImg.rows, getChannelCount());

	const auto& intrMat = cameraParams.intrinsicParams;
	const auto& dist = cameraParams.dist;

	cv::Mat distM(1, 5, CV_64F, const_cast<double*>(&dist[0]));
	cv::Mat inMat(3, 3, CV_64F);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) inMat.at<double>(i, j) = intrMat(i, j);

	cv::undistort(srcImg, out_img.m_impl->img, inMat, distM);

#endif
}

void CImage::filterMedian(CImage& out_img, int W) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally

	auto srcImg = const_cast<cv::Mat&>(m_impl->img);
	if (this == &out_img)
		srcImg = srcImg.clone();
	else
		out_img.resize(srcImg.cols, srcImg.rows, getChannelCount());

	cv::medianBlur(srcImg, out_img.m_impl->img, W);
#endif
}

void CImage::filterGaussian(CImage& out_img, int W, int H, double sigma) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally
	auto srcImg = const_cast<cv::Mat&>(m_impl->img);
	if (this == &out_img)
		srcImg = srcImg.clone();
	else
		out_img.resize(srcImg.cols, srcImg.rows, getChannelCount());

	cv::GaussianBlur(srcImg, out_img.m_impl->img, cv::Size(W, H), sigma);
#endif
}

void CImage::scaleImage(
	CImage& out_img, unsigned int width, unsigned int height,
	TInterpolationMethod interp) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally

	auto srcImg = m_impl->img;
	// Detect in-place operation and make a deep copy if needed:
	if (out_img.m_impl->img.data == srcImg.data) srcImg = srcImg.clone();

	// Already done?
	if (out_img.getWidth() == width && out_img.getHeight() == height)
	{
		out_img.m_impl->img = srcImg;
		return;
	}
	out_img.resize(width, height, getChannelCount());

	// Resize:
	cv::resize(
		srcImg, out_img.m_impl->img, out_img.m_impl->img.size(), 0, 0,
		interpolationMethod2Cv(interp));
#endif
}

void CImage::rotateImage(
	CImage& out_img, double ang, unsigned int cx, unsigned int cy,
	double scale) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();  // For delayed loaded images stored externally

	auto srcImg = m_impl->img;
	// Detect in-place operation and make a deep copy if needed:
	if (out_img.m_impl->img.data == srcImg.data) srcImg = srcImg.clone();

	out_img.resize(getWidth(), getHeight(), getChannelCount());

	// Based on the blog entry:
	// http://blog.weisu.org/2007/12/opencv-image-rotate-and-zoom-rotation.html

	// Apply rotation & scale:
	double m[2 * 3] = {scale * cos(ang), -scale * sin(ang), 1.0 * cx,
					   scale * sin(ang), scale * cos(ang),  1.0 * cy};
	cv::Mat M(2, 3, CV_64F, m);

	double dx = (srcImg.cols - 1) * 0.5;
	double dy = (srcImg.rows - 1) * 0.5;
	m[2] -= m[0] * dx + m[1] * dy;
	m[5] -= m[3] * dx + m[4] * dy;

	cv::warpAffine(
		srcImg, out_img.m_impl->img, M, out_img.m_impl->img.size(),
		cv::INTER_LINEAR + cv::WARP_INVERSE_MAP, cv::BORDER_REPLICATE);
#endif
}

bool CImage::drawChessboardCorners(
	std::vector<TPixelCoordf>& cornerCoords, unsigned int check_size_x,
	unsigned int check_size_y, unsigned int lines_width, unsigned int r)
{
#if MRPT_HAS_OPENCV

	if (cornerCoords.size() != check_size_x * check_size_y) return false;

	auto& img = m_impl->img;

	unsigned int x, y, i;
	CvPoint prev_pt = cvPoint(0, 0);
	const int line_max = 8;
	CvScalar line_colors[8];

	line_colors[0] = CV_RGB(255, 0, 0);
	line_colors[1] = CV_RGB(255, 128, 0);
	line_colors[2] = CV_RGB(255, 128, 0);
	line_colors[3] = CV_RGB(200, 200, 0);
	line_colors[4] = CV_RGB(0, 255, 0);
	line_colors[5] = CV_RGB(0, 200, 200);
	line_colors[6] = CV_RGB(0, 0, 255);
	line_colors[7] = CV_RGB(255, 0, 255);

	CCanvas::selectTextFont("10x20");

	IplImage iplp(img);
	IplImage* ipl = &iplp;

	for (y = 0, i = 0; y < check_size_y; y++)
	{
		CvScalar color = line_colors[y % line_max];
		for (x = 0; x < check_size_x; x++, i++)
		{
			CvPoint pt;
			pt.x = cvRound(cornerCoords[i].x);
			pt.y = cvRound(cornerCoords[i].y);

			if (i != 0) cvLine(ipl, prev_pt, pt, color, lines_width);

			cvLine(
				ipl, cvPoint(pt.x - r, pt.y - r), cvPoint(pt.x + r, pt.y + r),
				color, lines_width);
			cvLine(
				ipl, cvPoint(pt.x - r, pt.y + r), cvPoint(pt.x + r, pt.y - r),
				color, lines_width);

			if (r > 0) cvCircle(ipl, pt, r + 1, color);
			prev_pt = pt;

			// Text label with the corner index in the first and last
			// corners:
			if (i == 0 || i == cornerCoords.size() - 1)
				CCanvas::textOut(
					pt.x + 5, pt.y - 5, mrpt::format("%u", i),
					mrpt::img::TColor::blue());
		}
	}

	return true;
#else
	return false;
#endif
}

CImage CImage::colorImage() const
{
	CImage ret;
	colorImage(ret);
	return ret;
}

void CImage::colorImage(CImage& ret) const
{
#if MRPT_HAS_OPENCV
	if (this->isColor())
	{
		if (&ret != this) ret = *this;
		return;
	}

	auto srcImg = m_impl->img;
	// Detect in-place op. and make deep copy:
	if (srcImg.data == ret.m_impl->img.data) srcImg = srcImg.clone();

	ret.resize(getWidth(), getHeight(), CH_RGB);

	cv::cvtColor(srcImg, ret.m_impl->img, cv::COLOR_GRAY2BGR);
#endif
}

void CImage::joinImagesHorz(const CImage& img1, const CImage& img2)
{
#if MRPT_HAS_OPENCV
	ASSERT_(img1.getHeight() == img2.getHeight());

	auto im1 = img1.m_impl->img, im2 = img2.m_impl->img, img = m_impl->img;
	ASSERT_(im1.type() == im2.type());

	this->resize(im1.cols + im2.cols, im1.rows, getChannelCount());

	im1.copyTo(img(cv::Rect(0, 0, im1.cols, im1.rows)));
	im2.copyTo(img(cv::Rect(im1.cols, 0, im2.cols, im2.rows)));
#endif
}  // end

void CImage::equalizeHist(CImage& out_img) const
{
#if MRPT_HAS_OPENCV
	// Convert to a single luminance channel image
	auto srcImg = m_impl->img;
	if (this != &out_img)
		out_img.resize(srcImg.cols, srcImg.rows, getChannelCount());
	auto outImg = out_img.m_impl->img;

	if (srcImg.channels() == 1)
		cv::equalizeHist(srcImg, outImg);
	else
		THROW_EXCEPTION("Operation only supported for grayscale images");
#endif
}

// See: https://github.com/MRPT/mrpt/issues/885
// This seems a bug in GCC?
#if defined(__GNUC__)
#define MRPT_DISABLE_FULL_OPTIMIZATION __attribute__((optimize("O1")))
#else
#define MRPT_DISABLE_FULL_OPTIMIZATION
#endif

template <unsigned int HALF_WIN_SIZE>
void MRPT_DISABLE_FULL_OPTIMIZATION image_KLT_response_template(
	const uint8_t* in, const int widthStep, unsigned int x, unsigned int y,
	int32_t& _gxx, int32_t& _gyy, int32_t& _gxy)
{
	const auto min_x = x - HALF_WIN_SIZE;
	const auto min_y = y - HALF_WIN_SIZE;

	int32_t gxx = 0;
	int32_t gxy = 0;
	int32_t gyy = 0;

	const unsigned int WIN_SIZE = 1 + 2 * HALF_WIN_SIZE;

	unsigned int yy = min_y;
	for (unsigned int iy = WIN_SIZE; iy; --iy, ++yy)
	{
		const uint8_t* ptr = in + widthStep * yy + min_x;
		unsigned int xx = min_x;
		for (unsigned int ix = WIN_SIZE; ix; --ix, ++xx, ++ptr)
		{
			const int32_t dx =
				static_cast<int32_t>(ptr[+1]) - static_cast<int32_t>(ptr[-1]);
			const int32_t dy = static_cast<int32_t>(ptr[+widthStep]) -
							   static_cast<int32_t>(ptr[-widthStep]);
			gxx += dx * dx;
			gxy += dx * dy;
			gyy += dy * dy;
		}
	}
	_gxx = gxx;
	_gyy = gyy;
	_gxy = gxy;
}

float MRPT_DISABLE_FULL_OPTIMIZATION CImage::KLT_response(
	const unsigned int x, const unsigned int y,
	const unsigned int half_window_size) const
{
#if MRPT_HAS_OPENCV

	const auto& im1 = m_impl->img;
	const auto img_w = static_cast<unsigned int>(im1.cols),
			   img_h = static_cast<unsigned int>(im1.rows);
	const int widthStep = im1.step[0];

	// If any of those predefined values worked, do the generic way:
	const unsigned int min_x = x - half_window_size;
	const unsigned int max_x = x + half_window_size;
	const unsigned int min_y = y - half_window_size;
	const unsigned int max_y = y + half_window_size;

	// Since min_* are "unsigned", checking "<" will detect negative
	// numbers:
	ASSERTMSG_(
		min_x < img_w && max_x < img_w && min_y < img_h && max_y < img_h,
		"Window is out of image bounds");

	// Gradient sums: Use integers since they're much faster than
	// doubles/floats!!
	int32_t gxx = 0;
	int32_t gxy = 0;
	int32_t gyy = 0;

	const auto* img_data = im1.ptr<uint8_t>(0);
	switch (half_window_size)
	{
		case 2:
			image_KLT_response_template<2>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 3:
			image_KLT_response_template<3>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 4:
			image_KLT_response_template<4>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 5:
			image_KLT_response_template<5>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 6:
			image_KLT_response_template<6>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 7:
			image_KLT_response_template<7>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 8:
			image_KLT_response_template<8>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 9:
			image_KLT_response_template<9>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 10:
			image_KLT_response_template<10>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 11:
			image_KLT_response_template<11>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 12:
			image_KLT_response_template<12>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 13:
			image_KLT_response_template<13>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 14:
			image_KLT_response_template<14>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 15:
			image_KLT_response_template<15>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 16:
			image_KLT_response_template<16>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;
		case 32:
			image_KLT_response_template<32>(
				img_data, widthStep, x, y, gxx, gyy, gxy);
			break;

		default:
			for (unsigned int yy = min_y; yy <= max_y; yy++)
			{
				const uint8_t* p = img_data + widthStep * yy + min_x;
				for (unsigned int xx = min_x; xx <= max_x; xx++)
				{
					const int32_t dx = p[+1] - p[-1];
					const int32_t dy = p[+widthStep] - p[-widthStep];
					gxx += dx * dx;
					gxy += dx * dy;
					gyy += dy * dy;
				}
			}
			break;
	}
	// Convert to float's and normalize in the way:
	const float K = 0.5f / ((max_y - min_y + 1) * (max_x - min_x + 1));
	const float Gxx = gxx * K;
	const float Gxy = gxy * K;
	const float Gyy = gyy * K;

	// Return the minimum eigenvalue of:
	//    ( gxx  gxy )
	//    ( gxy  gyy )
	// See, for example:
	// mrpt::math::detail::eigenVectorsMatrix_special_2x2():
	const float t = Gxx + Gyy;  // Trace
	const float de = Gxx * Gyy - Gxy * Gxy;  // Det
	// The smallest eigenvalue is:
	return 0.5f * (t - std::sqrt(t * t - 4.0f * de));
#else
	return 0;
#endif
}

// Load from TGA files. Used in loadFromFile()
// Contains code from
// https://github.com/tjohnman/Simple-Targa-Library/blob/master/src/simpleTGA.cpp
// (FreeBSD license)
bool CImage::loadTGA(
	const std::string& fileName, mrpt::img::CImage& out_RGB,
	mrpt::img::CImage& out_alpha)
{
#if MRPT_HAS_OPENCV
	std::fstream stream;
	stream.open(fileName.c_str(), std::fstream::in | std::fstream::binary);
	if (!stream.is_open())
	{
		std::cerr << "[CImage::loadTGA] Couldn't open file '" << fileName
				  << "'.\n";
		return false;
	}

	stream.seekg(0, std::ios_base::end);
	// long length = stream.tellg();
	stream.seekg(0, std::ios_base::beg);

	// Simple uncompressed true-color image
	char dumpBuffer[12];
	char trueColorHeader[] = "\0\0\2\0\0\0\0\0\0\0\0\0";
	stream.read(dumpBuffer, 12);
	if (memcmp(dumpBuffer, trueColorHeader, 12) != 0)
	{
		std::cerr << "[CImage::loadTGA] Unsupported format or invalid file.\n";
		return false;
	}

	unsigned short width, height;
	unsigned char bpp;

	stream.read((char*)&width, 2);
	stream.read((char*)&height, 2);
	bpp = stream.get();
	if (bpp != 32)
	{
		std::cerr << "[CImage::loadTGA] Only 32 bpp format supported!\n";
		return false;
	}

	unsigned char desc;
	desc = stream.get();
	if (desc != 8 && desc != 32)
	{
		std::cerr << "[CImage::loadTGA] Unsupported format or invalid file.\n";
		return false;
	}
	const bool origin_is_low_corner = (desc == 8);

	// Data section
	std::vector<uint8_t> bytes(width * height * 4);
	stream.read((char*)&bytes[0], width * height * 4);
	stream.close();

	// Move data to images:
	out_RGB.resize(width, height, CH_RGB);
	out_alpha.resize(width, height, CH_GRAY);

	size_t idx = 0;
	for (int r = 0; r < height; r++)
	{
		const auto actual_row = origin_is_low_corner ? (height - 1 - r) : r;
		auto& img = out_RGB.m_impl->img;
		auto data = img.ptr<uint8_t>(actual_row);

		auto& img_alpha = out_alpha.m_impl->img;
		auto data_alpha = img_alpha.ptr<uint8_t>(actual_row);

		for (unsigned int c = 0; c < width; c++)
		{
			*data++ = bytes[idx++];  // R
			*data++ = bytes[idx++];  // G
			*data++ = bytes[idx++];  // B
			*data_alpha++ = bytes[idx++];  // A
		}
	}

	return true;
#else
	return false;
#endif  // MRPT_HAS_OPENCV
}

std::ostream& mrpt::img::operator<<(std::ostream& o, const TPixelCoordf& p)
{
	o << "(" << p.x << "," << p.y << ")";
	return o;
}
std::ostream& mrpt::img::operator<<(std::ostream& o, const TPixelCoord& p)
{
	o << "(" << p.x << "," << p.y << ")";
	return o;
}
