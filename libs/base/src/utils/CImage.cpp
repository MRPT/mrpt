/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/compress/zip.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/fourier.h>
#include <mrpt/math/utils.h>  // for roundup()
#include <mrpt/utils/round.h> // for round()
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/filesystem.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

#if MRPT_HAS_MATLAB
#	include <mexplus/mxarray.h>
#endif

// Prototypes of SSE2/SSE3/SSSE3 optimized functions:
#include "CImage_SSEx.h"

#if MRPT_HAS_WXWIDGETS
#	include <wx/image.h>
#endif

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CImage, CSerializable, mrpt::utils)


bool CImage::DISABLE_ZIP_COMPRESSION  = false;
bool CImage::DISABLE_JPEG_COMPRESSION = false;
int CImage::SERIALIZATION_JPEG_QUALITY = 95;

std::string CImage::IMAGES_PATH_BASE(".");

// Do performance time logging?
#define  IMAGE_ALLOC_PERFLOG  0

#if IMAGE_ALLOC_PERFLOG
mrpt::utils::CTimeLogger alloc_tims;
#endif

/*---------------------------------------------------------------
						Constructor
 ---------------------------------------------------------------*/
CImage::CImage( unsigned int	width,
	unsigned int	height,
	TImageChannels	nChannels,
	bool			originTopLeft ) :
		img(NULL),
		m_imgIsReadOnly(false),
		m_imgIsExternalStorage(false)
{
	MRPT_START
	changeSize( width, height, nChannels, originTopLeft );
	MRPT_END
}
/*---------------------------------------------------------------
				Default	Constructor
 ---------------------------------------------------------------*/
CImage::CImage( ) :
	img(NULL),
	m_imgIsReadOnly(false),
	m_imgIsExternalStorage(false)
{
#if MRPT_HAS_OPENCV
	MRPT_START
	changeSize( 1, 1, CH_RGB, true );
	MRPT_END
#endif
}

/*---------------------------------------------------------------
						Copy constructor
 ---------------------------------------------------------------*/
CImage::CImage( const CImage &o ) :
	img(NULL),
	m_imgIsReadOnly(false),
	m_imgIsExternalStorage(false)
{
	MRPT_START
	*this = o;
	MRPT_END
}

/*---------------------------------------------------------------
						Copy operator
 ---------------------------------------------------------------*/
CImage& CImage::operator = (const CImage& o)
{
	MRPT_START
	if (this==&o) return *this;
	releaseIpl();
	m_imgIsExternalStorage = o.m_imgIsExternalStorage;
	m_imgIsReadOnly = false;

	if (!o.m_imgIsExternalStorage)
	{ 	// A normal image
#if MRPT_HAS_OPENCV
		ASSERTMSG_(o.img!=NULL,"Source image in = operator has NULL IplImage*")
		img = cvCloneImage( (IplImage*)o.img );
#endif
	}
	else
	{ 	// An externally stored image:
		m_externalFile = o.m_externalFile;
	}
	return *this;
	MRPT_END
}

/*---------------------------------------------------------------
					swap
 ---------------------------------------------------------------*/
void CImage::swap(CImage &o)
{
	std::swap( img, o.img );
	std::swap( m_imgIsReadOnly, o.m_imgIsReadOnly );
	std::swap( m_imgIsExternalStorage, o.m_imgIsExternalStorage );
	std::swap( m_externalFile, o.m_externalFile );
}

/*---------------------------------------------------------------
					copyFromForceLoad

 Copies from another image, and, if that one is externally
  stored, the image file will be actually loaded into memory
  in "this" object.
 ---------------------------------------------------------------*/
void CImage::copyFromForceLoad(const CImage &o)
{
	if (o.isExternallyStored())
	{
		// Load from that file:
		if (!this->loadFromFile( o.getExternalStorageFileAbsolutePath() ))
			THROW_TYPED_EXCEPTION_CUSTOM_MSG1("Error loading externally-stored image from: %s", o.getExternalStorageFileAbsolutePath().c_str() ,CExceptionExternalImageNotFound);
	}
	else
	{	// It's not external storage.
		*this = o;
	}
}

/*---------------------------------------------------------------
					copyFastFrom
 ---------------------------------------------------------------*/
void CImage::copyFastFrom( CImage &o )
{
	MRPT_START
	if (this==&o) return;
	if (o.m_imgIsExternalStorage)
	{
		// Just copy the reference to the ext. file:
		*this = o;
	}
	else
	{	// Normal copy
#if MRPT_HAS_OPENCV
		if (!o.img) 		THROW_EXCEPTION("Origin image is empty! (o.img==NULL)")
#endif
		// Erase current image:
		releaseIpl();

		// Make the transfer of just the pointer:
		img = o.img;
		m_imgIsReadOnly = o.m_imgIsReadOnly;
		m_imgIsExternalStorage = o.m_imgIsExternalStorage;
		m_externalFile = o.m_externalFile;

		o.img = NULL;
		o.m_imgIsReadOnly = false;
		o.m_imgIsExternalStorage=false;
	}

	MRPT_END
}

/*---------------------------------------------------------------
						Constructor from IplImage
 ---------------------------------------------------------------*/
CImage::CImage( void *iplImage ) :
	img(NULL),
	m_imgIsReadOnly(false),
	m_imgIsExternalStorage(false)
{
	MRPT_START

#if MRPT_HAS_OPENCV
	if (!iplImage)
		changeSize( 1, 1, 1, true );
	else
		img = cvCloneImage( (IplImage*) iplImage );
#endif
	MRPT_END
}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CImage::~CImage( )
{
	releaseIpl();
}

/*---------------------------------------------------------------
						changeSize
 ---------------------------------------------------------------*/
void  CImage::changeSize(
		unsigned int	width,
		unsigned int	height,
		TImageChannels	nChannels,
		bool			originTopLeft)
{
	MRPT_START

#if MRPT_HAS_OPENCV
	// If we're resizing to exactly the current size, do nothing and avoid wasting mem allocs/deallocs!
	if (img)
	{
		makeSureImageIsLoaded();   // For delayed loaded images stored externally
		IplImage *ipl = static_cast<IplImage*>(img);
		if (static_cast<unsigned int>(ipl->width)==width &&
		    static_cast<unsigned int>(ipl->height)==height &&
		    ipl->nChannels == nChannels &&
		    ipl->origin == ( originTopLeft ? 0:1)
			)
		{
			return; // nothing to do, we're already right with the current IplImage!
		}
	}

	// Delete current img
	releaseIpl();

#	if IMAGE_ALLOC_PERFLOG
	const std::string sLog = mrpt::format("cvCreateImage %ux%u",width,height);
	alloc_tims.enter(sLog.c_str());
#	endif

	img = cvCreateImage( cvSize(width,height),IPL_DEPTH_8U, nChannels );
	((IplImage*)img)->origin = originTopLeft ? 0:1;

#	if IMAGE_ALLOC_PERFLOG
	alloc_tims.leave(sLog.c_str());
#	endif

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END
}

/*---------------------------------------------------------------
						loadFromFile
 ---------------------------------------------------------------*/
bool  CImage::loadFromFile( const std::string& fileName, int isColor )
{
	MRPT_START

#if MRPT_HAS_OPENCV
	IplImage* newImg = cvLoadImage(fileName.c_str(),isColor);
	if (newImg!=NULL) {
		releaseIpl();
		img = newImg;
		return true;
	} else {
		return false;
	}
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

/*---------------------------------------------------------------
						saveToFile
 ---------------------------------------------------------------*/
bool  CImage::saveToFile( const std::string& fileName, int jpeg_quality ) const
{
    MRPT_START
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);

    #if MRPT_OPENCV_VERSION_NUM>0x110
        int p[3];
        p[0] = CV_IMWRITE_JPEG_QUALITY;
        p[1] = jpeg_quality;
        p[2] = 0;
        return (0!= cvSaveImage(fileName.c_str(),img,p) );
    #else
        return (0!= cvSaveImage(fileName.c_str(),img) );
    #endif
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
    MRPT_END
}

/*---------------------------------------------------------------
						loadFromIplImage
 ---------------------------------------------------------------*/
void  CImage::loadFromIplImage( void* iplImage )
{
	MRPT_START
	ASSERT_(iplImage!=NULL)
	releaseIpl();
	if (iplImage)
	{
#if MRPT_HAS_OPENCV
		img = cvCloneImage( (IplImage*)iplImage );
#else
		THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	}
	MRPT_END
}

/*---------------------------------------------------------------
						setFromIplImageReadOnly
 ---------------------------------------------------------------*/
void  CImage::setFromIplImageReadOnly( void* iplImage )
{
	MRPT_START
	releaseIpl();
#if MRPT_HAS_OPENCV
	ASSERT_(iplImage!=NULL)
	ASSERTMSG_(iplImage!=this->img,"Trying to assign read-only to itself.")

	img = (IplImage*)iplImage;
#else
	if (iplImage) {
		THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
	}
#endif
	m_imgIsReadOnly = true;
	m_imgIsExternalStorage=false;
	MRPT_END
}

/*---------------------------------------------------------------
						setFromIplImage
 ---------------------------------------------------------------*/
void  CImage::setFromIplImage( void* iplImage )
{
	MRPT_START

	releaseIpl();
	if (iplImage)
	{
#if MRPT_HAS_OPENCV
		img = (IplImage*)iplImage;
#else
		THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	}
	m_imgIsReadOnly = false;
	m_imgIsExternalStorage=false;

	MRPT_END
}

/*---------------------------------------------------------------
						loadFromMemoryBuffer
 ---------------------------------------------------------------*/
void  CImage::loadFromMemoryBuffer(
			unsigned int	width,
			unsigned int	height,
			bool			color,
			unsigned char	*rawpixels,
			bool 			swapRedBlue  )
{
	MRPT_START

#if MRPT_HAS_OPENCV
	resize(width,height,color ? 3:1, true);
	m_imgIsReadOnly = false;
	m_imgIsExternalStorage=false;

	if (color && swapRedBlue)
	{
		// Do copy & swap at once:
		unsigned char *ptr_src = rawpixels;
		unsigned char *ptr_dest = reinterpret_cast<unsigned char*>( ((IplImage*)img)->imageData );
		const int  bytes_per_row_out = ((IplImage*)img)->widthStep;

		for(int h=height; h--; )
		{
			for( unsigned int i = 0; i < width; i++, ptr_src += 3, ptr_dest += 3 )
			{
				unsigned char t0 = ptr_src[0], t1 = ptr_src[1], t2 = ptr_src[2];
				ptr_dest[2] = t0; ptr_dest[1] = t1; ptr_dest[0] = t2;
			}
			ptr_dest += bytes_per_row_out - width*3;
		}
	}
	else
	{
		if ( ((IplImage*)img)->widthStep == ((IplImage*)img)->width * ((IplImage*)img)->nChannels )
		{
			// Copy the image data:
			memcpy( ((IplImage*)img)->imageData,
					rawpixels,
					((IplImage*)img)->imageSize);
		}
		else
		{
			// Copy the image row by row:
			unsigned char *ptr_src = rawpixels;
			unsigned char *ptr_dest = reinterpret_cast<unsigned char*>( ((IplImage*)img)->imageData );
			int  bytes_per_row = width * (color ? 3:1);
			int  bytes_per_row_out = ((IplImage*)img)->widthStep;
			for (unsigned int y=0;y<height;y++)
			{
				memcpy( ptr_dest, ptr_src, bytes_per_row );
				ptr_src+=bytes_per_row;
				ptr_dest+=bytes_per_row_out;
			}
		}
	}
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
	MRPT_END
}

/*---------------------------------------------------------------
						operator()
 ---------------------------------------------------------------*/
unsigned char*  CImage::operator()(
			unsigned int	col,
			unsigned int	row,
			unsigned int	channel) const
{
#if MRPT_HAS_OPENCV

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_START
#endif

	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	IplImage *ipl = ((IplImage*)img);

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	ASSERT_(ipl);
	if (row>=(unsigned int)ipl->height || col>=(unsigned int)ipl->width || channel>=(unsigned int)ipl->nChannels )
	{
		THROW_EXCEPTION( format("Pixel coordinates/channel out of bounds: row=%u/%u col=%u/%u chan=%u/%u",
			row, ipl->height,
			col, ipl->width,
			channel, ipl->nChannels ) );
	}
#endif

	return (unsigned char*) &ipl->imageData	[ row * ipl->widthStep +
											  col * ipl->nChannels +
											  channel ];
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_END
#endif

#else
	THROW_EXCEPTION("MRPT was compiled without OpenCV")
#endif
}

/*---------------------------------------------------------------
						get_unsafe()
 ---------------------------------------------------------------*/
unsigned char*  CImage::get_unsafe(
			unsigned int	col,
			unsigned int	row,
			unsigned int	channel) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	IplImage *ipl = ((IplImage*)img);
	return (unsigned char*) &ipl->imageData	[ row * ipl->widthStep +
											  col * ipl->nChannels +
											  channel ];
#else
	return NULL;
#endif
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CImage::writeToStream(mrpt::utils::CStream &out, int *version) const
{
#if !MRPT_HAS_OPENCV
	if (version)
		*version = 100;
	else
	{
		out << m_imgIsExternalStorage;

		if (m_imgIsExternalStorage)
			out << m_externalFile;
		// Nothing else to serialize!
	}
#else
	if (version)
		*version = 8;
	else
	{
		// Added in version 6: possibility of being stored offline:
		out << m_imgIsExternalStorage;

		if (m_imgIsExternalStorage)
		{
			out << m_externalFile;
		}
		else
		{ // Normal image loaded in memory:
			ASSERT_(img!=NULL);

			const bool hasColor = isColor();

			out << hasColor;

			// Version >2: Color->JPEG, GrayScale->BYTE's array!
			if ( !hasColor )
			{
				// GRAY-SCALE: Raw bytes:
				// Version 3: ZIP compression!
				// Version 4: Skip zip if the image size <= 16Kb
				int32_t width = ((IplImage*)img)->width;
				int32_t height = ((IplImage*)img)->height;
				int32_t origin = ((IplImage*)img)->origin;
				int32_t imageSize = ((IplImage*)img)->imageSize;

				out << width << height << origin << imageSize;

				// Version 5: Use CImage::DISABLE_ZIP_COMPRESSION
				bool imageStoredAsZip = !CImage::DISABLE_ZIP_COMPRESSION && (imageSize>16*1024);

				out << imageStoredAsZip;

				// Version 4: Skip zip if the image size <= 16Kb
				if (imageStoredAsZip )
				{
					std::vector<unsigned char>	tempBuf;
					compress::zip::compress(
						((IplImage*)img)->imageData,		// Data
						((IplImage*)img)->imageSize,		// Size
						tempBuf);

					int32_t zipDataLen = (int32_t )tempBuf.size();

					out << zipDataLen;

					out.WriteBuffer( &tempBuf[0], tempBuf.size() );
					tempBuf.clear();
				}
				else
				{
					out.WriteBuffer( ((IplImage*)img)->imageData,((IplImage*)img)->imageSize );
				}
			}
			else
			{
				// COLOR: High quality JPEG image

				// v7: If size is 0xN or Nx0, don't call "saveToStreamAsJPEG"!!
				const int32_t width = ((IplImage*)img)->width;
				const int32_t height = ((IplImage*)img)->height;

				// v8: If DISABLE_JPEG_COMPRESSION
				if (!CImage::DISABLE_JPEG_COMPRESSION)
				{
					// normal behavior: compress images:
					out << width << height;

					if (width>=1 && height>=1)
					{
						// Save to temporary memory stream:
						CMemoryStream		aux;
						saveToStreamAsJPEG( aux, CImage::SERIALIZATION_JPEG_QUALITY );

						const uint32_t nBytes = static_cast<uint32_t>(aux.getTotalBytesCount());

						out << nBytes;
						out.WriteBuffer( aux.getRawBufferData(), nBytes );
					}
				}
				else
				{   // (New in v8)
					// Don't JPEG-compress behavior:
					// Use negative image sizes to signal this behavior:
					const int32_t neg_width = -width;
					const int32_t neg_height = -height;

					out << neg_width << neg_height;

					// Dump raw image data:
					const IplImage *ipl = static_cast<const IplImage*>(img);
					const size_t bytes_per_row = ipl->width * 3;

					out.WriteBuffer( &ipl->imageData[0], bytes_per_row*ipl->height );

				}
			}
		} // end m_imgIsExternalStorage=false
	}
#endif
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CImage::readFromStream(mrpt::utils::CStream &in, int version)
{
#if !MRPT_HAS_OPENCV
	if (version==100)
	{
		in >> m_imgIsExternalStorage;
		if (m_imgIsExternalStorage)
			in >> m_externalFile;
		else
		{
			THROW_EXCEPTION("[CImage] Cannot deserialize image since MRPT has been compiled without OpenCV")
		}
	}
#else
	releaseIpl();  // First, free current image.

	switch(version)
	{
	case 100: // Saved from an MRPT build without OpenCV:
		{
			in >> m_imgIsExternalStorage;
			if (m_imgIsExternalStorage)
				in >> m_externalFile;
		}
		break;
	case 0:
		{
			uint32_t		width, height, nChannels, imgLength;
			uint8_t			originTopLeft;

			in >> width >> height >> nChannels >> originTopLeft >> imgLength;

			changeSize(width, height, nChannels, originTopLeft !=0 );
			in.ReadBuffer( ((IplImage*)img)->imageData, imgLength );
		} break;
	case 1:
		{
			// Version 1: High quality JPEG image
			CMemoryStream		aux;
			uint32_t			nBytes;
			in >> nBytes;

			aux.changeSize( nBytes + 10 );

			in.ReadBuffer( aux.getRawBufferData(), nBytes );

			aux.Seek(0);

			loadFromStreamAsJPEG( aux );

		} break;
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
		{
			// Version 6: 	m_imgIsExternalStorage ??
			if (version>=6)
					in >> m_imgIsExternalStorage;
			else	m_imgIsExternalStorage=false;

			if (m_imgIsExternalStorage)
			{
				// Just the file name:
				in >> m_externalFile;
			}
			else
			{  // Normal, the whole image data:

				// Version 2: Color->JPEG, GrayScale->BYTE's array!
				uint8_t		hasColor;
				in >> hasColor;
				if (!hasColor)
				{
					// GRAY SCALE:
					int32_t		width,height,origin, imageSize;
					in >> width >> height >> origin >> imageSize;

					changeSize(width, height, 1, origin == 0 );
					ASSERT_( imageSize == ((IplImage*)img)->imageSize );

					if (version==2)
					{
						// RAW BYTES:
						in.ReadBuffer( ((IplImage*)img)->imageData, imageSize );
					}
					else
					{
						// Version 3: ZIP compression!
						bool	imageIsZIP = true;

						// Version 4: Skip zip if the image size <= 16Kb
						// Version 5: Use CImage::DISABLE_ZIP_COMPRESSION
						if (version==4 && imageSize<=16*1024)
							imageIsZIP = false;

						if (version>=5)
						{
							// It is stored int the stream:
							in >> imageIsZIP;
						}

						if (imageIsZIP)
						{
							uint32_t	zipDataLen;
							in >> zipDataLen;

							size_t	outDataBufferSize = imageSize;
							size_t	outDataActualSize;

							compress::zip::decompress(
								in,
								zipDataLen,
								((IplImage*)img)->imageData,
								outDataBufferSize,
								outDataActualSize );

							ASSERT_(outDataActualSize==outDataBufferSize);
						}
						else
						{
							// Raw bytes:
							in.ReadBuffer( ((IplImage*)img)->imageData,((IplImage*)img)->imageSize );
						}
					}
				}
				else
				{
					bool	loadJPEG=true;

					if (version>=7)
					{
						int32_t width, height;
						in >> width >> height;

						if (width>=1 && height>=1)
						{
							loadJPEG = true;
						}
						else
						{
							loadJPEG = false;

							if (width<0 && height<0)
							{
								// v8: raw image:
								const int32_t real_w = -width;
								const int32_t real_h = -height;

								this->changeSize(real_w,real_h,3,true);

								const IplImage *ipl = static_cast<const IplImage*>(img);
								const size_t bytes_per_row = ipl->width * 3;
								for (int y=0;y<ipl->height;y++)
								{
									const size_t nRead = in.ReadBuffer( &ipl->imageData[y*ipl->widthStep], bytes_per_row);
									if (nRead!=bytes_per_row) THROW_EXCEPTION("Error: Truncated data stream while parsing raw image?")
								}
							}
							else
							{
								// it's a 0xN or Nx0 image: just resize and load nothing:
								this->changeSize(width,height,3,true);
							}
						}
					}

					// COLOR IMAGE: JPEG
					if (loadJPEG)
					{
						CMemoryStream		aux;
						uint32_t			nBytes;
						in >> nBytes;
						aux.changeSize( nBytes + 10 );
						in.ReadBuffer( aux.getRawBufferData(), nBytes );
						aux.Seek(0);
						loadFromStreamAsJPEG( aux );
					}
				}
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
#endif
}

/*---------------------------------------------------------------
  Implements the writing to a mxArray for Matlab
 ---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM( mrpt::utils::CImage )

mxArray* CImage::writeToMatlab() const
{
    cv::Mat cvImg = cv::cvarrToMat( this->getAs<IplImage>() );
	return mexplus::from( cvImg );
}
#endif

/*---------------------------------------------------------------
						getSize
 ---------------------------------------------------------------*/
void CImage::getSize(TImageSize &s) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL);
	s.x = ((IplImage*)img)->width;
	s.y = ((IplImage*)img)->height;
#endif
}

/*---------------------------------------------------------------
						getWidth
 ---------------------------------------------------------------*/
size_t CImage::getWidth() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL);
	return ((IplImage*)img)->width;
#else
	return 0;
#endif
}

/*---------------------------------------------------------------
						getRowStride
 ---------------------------------------------------------------*/
size_t CImage::getRowStride() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL);
	return ((IplImage*)img)->widthStep;
#else
	return 0;
#endif
}


/*---------------------------------------------------------------
						getWidth
 ---------------------------------------------------------------*/
size_t CImage::getHeight() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL);
	return ((IplImage*)img)->height;
#else
	return 0;
#endif
}

/*---------------------------------------------------------------
						isColor
 ---------------------------------------------------------------*/
bool  CImage::isColor() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL);
	return ((IplImage*)img)->nChannels > 1;
#else
	return false;
#endif
}

/*---------------------------------------------------------------
						getChannelCount
 ---------------------------------------------------------------*/
TImageChannels CImage::getChannelCount() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL);
	return static_cast<unsigned int >( ((IplImage*)img)->nChannels );
#else
	return 0;
#endif
}


/*---------------------------------------------------------------
						isOriginTopLeft
 ---------------------------------------------------------------*/
bool  CImage::isOriginTopLeft() const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL);
	return ((IplImage*)img)->origin == 0;
#else
		THROW_EXCEPTION("MRPT compiled without OpenCV")
#endif
}


/*---------------------------------------------------------------
						getAsFloat
 ---------------------------------------------------------------*/
float  CImage::getAsFloat(
			unsigned int	col,
			unsigned int	row,
			unsigned int	channel) const
{
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	// [0,255]->[0,1]
	return (*(*this)(col,row,channel)) / 255.0f;
}

/*---------------------------------------------------------------
						getAsFloat
 ---------------------------------------------------------------*/
float  CImage::getAsFloat(
			unsigned int	col,
			unsigned int	row) const
{
	// Is a RGB image??
	if (isColor())
	{
		// Luminance: Y = 0.3R + 0.59G + 0.11B
		unsigned char	*pixels = (*this)(col,row,0);
		return (pixels[0] * 0.3f + pixels[1] *0.59f + pixels[2] * 0.11f)/255.0f;
	}
	else
	{
		// [0,255]->[0,1]
		return  (*(*this)(col,row,0 /* Channel 0:Gray level */)) / 255.0f;
	}
}

/*---------------------------------------------------------------
						getMaxAsFloat
 ---------------------------------------------------------------*/
float  CImage::getMaxAsFloat() const
{
	int		x,y,cx = getWidth(), cy = getHeight();

	float	maxPixel = 0;

	for (x=0;x<cx;x++)
		for (y=0;y<cy;y++)
			maxPixel = max( maxPixel, getAsFloat(x,y) );

	return maxPixel;
}

/*---------------------------------------------------------------
						grayscale
 ---------------------------------------------------------------*/
CImage  CImage::grayscale() const
{
	CImage		ret;
	grayscale(ret);
	return ret;
}

// Auxiliary function for both ::grayscale() and ::grayscaleInPlace()
#if MRPT_HAS_OPENCV
IplImage *ipl_to_grayscale(const IplImage * img_src)
{
	IplImage * img_dest = cvCreateImage( cvSize(img_src->width,img_src->height),IPL_DEPTH_8U, 1 );
	img_dest->origin = img_src->origin;

	// If possible, use SSE optimized version:
#if MRPT_HAS_SSE3
	if (is_aligned<16>(img_src->imageData) &&
		(img_src->width & 0xF) == 0 &&
		img_src->widthStep==img_src->width*img_src->nChannels &&
		img_dest->widthStep==img_dest->width*img_dest->nChannels )
	{
		ASSERT_(is_aligned<16>(img_dest->imageData))
		image_SSSE3_bgr_to_gray_8u( (const uint8_t*)img_src->imageData, (uint8_t*)img_dest->imageData, img_src->width,img_src->height);
		return img_dest;
	}
#endif

	// OpenCV Method:
	cvCvtColor( img_src, img_dest, CV_BGR2GRAY );
	return img_dest;
}
#endif

/*---------------------------------------------------------------
						grayscale
 ---------------------------------------------------------------*/
void CImage::grayscale( CImage  &ret ) const
{
#if MRPT_HAS_OPENCV
	// The image is already grayscale??
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	const IplImage *ipl = this->getAs<const IplImage>();
	ASSERT_(ipl)

	if (ipl->nChannels==1)
	{
		ret = *this;
		return;
	}
	else
	{
		// Convert to a single luminance channel image
		ret.setFromIplImage(ipl_to_grayscale(ipl));
	}
#endif
}

/*---------------------------------------------------------------
                        grayscaleInPlace
 ---------------------------------------------------------------*/
void CImage::grayscaleInPlace()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	const IplImage *ipl = this->getAs<const IplImage>();
	ASSERT_(ipl)
	if (ipl->nChannels==1) return; // Already done.

	setFromIplImage(ipl_to_grayscale(ipl));
#endif
}


/*---------------------------------------------------------------
						scaleHalf
 ---------------------------------------------------------------*/
void CImage::scaleHalf(CImage &out)const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL)

	// Get this image size:
	const IplImage * img_src = ((IplImage*)img);
	const int w = img_src->width;
	const int h = img_src->height;

	// Create target image:
	IplImage * img_dest = cvCreateImage( cvSize(w>>1,h>>1),IPL_DEPTH_8U, img_src->nChannels );
	img_dest->origin = img_src->origin;
	memcpy(img_dest->colorModel,img_src->colorModel,4);
	memcpy(img_dest->channelSeq,img_src->channelSeq,4);
	img_dest->dataOrder=img_src->dataOrder;


	// If possible, use SSE optimized version:
#if MRPT_HAS_SSE3
	if (img_src->nChannels==3 &&
		is_aligned<16>(img_src->imageData) &&
		is_aligned<16>(img_dest->imageData) &&
		(w & 0xF) == 0 &&
		img_src->widthStep==img_src->width*img_src->nChannels &&
		img_dest->widthStep==img_dest->width*img_dest->nChannels )
	{
		image_SSSE3_scale_half_3c8u( (const uint8_t*)img_src->imageData, (uint8_t*)img_dest->imageData, w,h);
		out.setFromIplImage(img_dest);
		return;
	}
#endif

#if MRPT_HAS_SSE2
	if (img_src->nChannels==1 &&
		is_aligned<16>(img_src->imageData) &&
		is_aligned<16>(img_dest->imageData) &&
		(w & 0xF) == 0 &&
		img_src->widthStep==img_src->width*img_src->nChannels &&
		img_dest->widthStep==img_dest->width*img_dest->nChannels )
	{
		image_SSE2_scale_half_1c8u( (const uint8_t*)img_src->imageData, (uint8_t*)img_dest->imageData, w,h);

		out.setFromIplImage(img_dest);
		return;
	}
#endif

	// Fall back to slow method:
	cvResize( img_src, img_dest, IMG_INTERP_NN );
	out.setFromIplImage(img_dest);
#endif
}

/*---------------------------------------------------------------
						scaleHalfSmooth
 ---------------------------------------------------------------*/
void CImage::scaleHalfSmooth(CImage &out)const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL)

	// Get this image size:
	const IplImage * img_src = ((IplImage*)img);
	const int w = img_src->width;
	const int h = img_src->height;

	// Create target image:
	IplImage * img_dest = cvCreateImage( cvSize(w>>1,h>>1),IPL_DEPTH_8U, img_src->nChannels );
	img_dest->origin = img_src->origin;
	memcpy(img_dest->colorModel,img_src->colorModel,4);
	memcpy(img_dest->channelSeq,img_src->channelSeq,4);
	img_dest->dataOrder=img_src->dataOrder;


	// If possible, use SSE optimized version:
#if MRPT_HAS_SSE2
	if (img_src->nChannels==1 &&
		is_aligned<16>(img_src->imageData) &&
		is_aligned<16>(img_dest->imageData) &&
		(w & 0xF) == 0 &&
		img_src->widthStep==img_src->width*img_src->nChannels &&
		img_dest->widthStep==img_dest->width*img_dest->nChannels )
	{
		image_SSE2_scale_half_smooth_1c8u( (const uint8_t*)img_src->imageData, (uint8_t*)img_dest->imageData, w,h);

		out.setFromIplImage(img_dest);
		return;
	}
#endif

	// Fall back to slow method:
	cvResize( img_src, img_dest, IMG_INTERP_LINEAR );
	out.setFromIplImage(img_dest);
#endif
}

/*---------------------------------------------------------------
						scaleDouble
 ---------------------------------------------------------------*/
void CImage::scaleDouble(CImage &out)const
{
	out = *this;
	const TImageSize siz = this->getSize();
	out.scaleImage(siz.x*2,siz.y*2);
}

/*---------------------------------------------------------------
					getChannelsOrder
 ---------------------------------------------------------------*/
const char *  CImage::getChannelsOrder()const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);
	return ((IplImage*)img)->channelSeq;
#else
		THROW_EXCEPTION("MRPT compiled without OpenCV")
#endif
}


/*---------------------------------------------------------------
				loadFromMemoryBuffer
 ---------------------------------------------------------------*/
void  CImage::loadFromMemoryBuffer(
			unsigned int		width,
			unsigned int		height,
			unsigned int		bytesPerRow,
			unsigned char		*red,
			unsigned char		*green,
			unsigned char		*blue )
{
#if MRPT_HAS_OPENCV
	MRPT_START

	// Fill in the IPL structure:
	changeSize( width, height, 3, true );

	// Copy the image data:
	for (unsigned int y=0;y<height;y++)
	{
		// The target pixels:
		unsigned char *dest = (unsigned char *)((IplImage*)img)->imageData + ((IplImage*)img)->widthStep * y;

		// Source channels:
		unsigned char *srcR = red + bytesPerRow * y;
		unsigned char *srcG = green + bytesPerRow * y;
		unsigned char *srcB = blue + bytesPerRow * y;

		for (unsigned int x=0;x<width;x++)
		{
            *(dest++) = *(srcB++);
            *(dest++) = *(srcG++);
            *(dest++) = *(srcR++);
		} // end of x
	} // end of y

	MRPT_END
#endif
}


/*---------------------------------------------------------------
						setOriginTopLeft
 ---------------------------------------------------------------*/
void  CImage::setOriginTopLeft(bool val)
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);
	((IplImage*)img)->origin =  val ? 0:1;
#endif
}

/*---------------------------------------------------------------
						setPixel
 ---------------------------------------------------------------*/
void  CImage::setPixel(
	int				x,
	int				y,
	size_t	color)
{
#if MRPT_HAS_OPENCV

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_START
#endif

	makeSureImageIsLoaded();   // For delayed loaded images stored externally

	IplImage *ipl = ((IplImage*)img);

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	ASSERT_(ipl);
#endif

	if (x>=0 && y>=0 && y<ipl->height && x<ipl->width)
	{
		// The pixel coordinates is valid:
		if (ipl->nChannels==1)
		{
			*( (unsigned char*) &ipl->imageData	[ y * ipl->widthStep + x ] ) = (unsigned char) color;
		}
		else
		{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
			ASSERT_( ipl->nChannels ==3 );
			if ( ipl->dataOrder!=0 )
				THROW_EXCEPTION("Please, use interleaved images like normal people!!! :-)");
#endif
			unsigned char	*dest = (unsigned char*) &ipl->imageData[ y * ipl->widthStep + 3*x ];
			unsigned char	*src  = (unsigned char*) &color;

			// Copy the color:
			*dest++ = *src++;	// R
			*dest++ = *src++;	// G
			*dest++ = *src++;	// B
		}
	}

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_END
#endif

#endif
}

/*---------------------------------------------------------------
						line
 ---------------------------------------------------------------*/
void  CImage::line(
	int				x0,
	int				y0,
	int				x1,
	int				y1,
	const mrpt::utils::TColor color,
	unsigned int	width,
	TPenStyle		penStyle)
{
#if MRPT_HAS_OPENCV
	MRPT_UNUSED_PARAM(penStyle);
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	IplImage *ipl = ((IplImage*)img);
	ASSERT_(ipl);

	cvLine(ipl, cvPoint(x0,y0), cvPoint(x1,y1), CV_RGB(color.R,color.G,color.B), width );
#endif
}

/*---------------------------------------------------------------
						drawCircle
 ---------------------------------------------------------------*/
void  CImage::drawCircle(
	int				x,
	int				y,
	int				radius,
	const mrpt::utils::TColor &color,
	unsigned int	width)
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	IplImage *ipl = ((IplImage*)img);
	ASSERT_(ipl);

	cvCircle( ipl, cvPoint(x,y), radius, CV_RGB (color.R,color.G,color.B), width  );
#endif
} // end

/*---------------------------------------------------------------
                                               update_patch
 --------------------------------------------------------------*/
void CImage::update_patch(const CImage &patch,
			  const unsigned int col_,
			  const unsigned int row_)
{
#if MRPT_HAS_OPENCV
	IplImage *ipl_int = ((IplImage*)img);
	IplImage *ipl_ext = ((IplImage*)patch.img);
	ASSERT_(ipl_int);
	ASSERT_(ipl_ext);
	// We check that patch do not jut out of the image.
	if(row_+ipl_ext->height > getHeight() || col_+ipl_ext->width > getWidth())
	{
	    THROW_EXCEPTION("Error : Patch jut out of image")
	}
	for (unsigned int i=0;i<patch.getHeight();i++)
	{
		memcpy( &ipl_int->imageData[(i+row_) * ipl_int->widthStep + col_ * ipl_int->nChannels],
			&ipl_ext->imageData[i * ipl_ext->widthStep ],
			ipl_ext->widthStep);
	}
#endif
}

/*---------------------------------------------------------------
						extract_patch
 ---------------------------------------------------------------*/
void  CImage::extract_patch(
	CImage	&patch,
    const unsigned int	col_,
	const unsigned int	row_,
	const unsigned int col_num,
	const unsigned int row_num)const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally

	IplImage *ipl_int = ((IplImage*)img);
	ASSERT_(ipl_int);

	if ((ipl_int->width<(int)(col_+col_num)) || (ipl_int->height<(int)(row_+row_num)))
	{
		THROW_EXCEPTION( format("Trying to extract patch out of image boundaries: Image size=%ix%i, Patch size=%ux%u, extraction location=(%u,%u)",ipl_int->width,ipl_int->height, col_num, row_num, col_, row_ ) )
	}

	patch.resize(col_num,row_num,((IplImage*)img)->nChannels,true);
	IplImage *ipl_ext = ((IplImage*)patch.img);
	ASSERT_(ipl_ext);

	for (unsigned int i=0;i<row_num;i++)
	{
		memcpy( &ipl_ext->imageData[i * ipl_ext->widthStep ],
				&ipl_int->imageData[(i+row_) * ipl_int->widthStep + col_ * ipl_int->nChannels],
				ipl_ext->widthStep);
	}

#endif
}

/*---------------------------------------------------------------
						correlate
 ---------------------------------------------------------------*/
float  CImage::correlate(const CImage &img2,int width_init,int height_init)const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally

	if ((img2.getWidth()+width_init>getWidth())|(img2.getHeight()+ height_init>getHeight()))
		THROW_EXCEPTION("Correlation Error!, image to correlate out of bounds");

    unsigned int i,j;
	float x1,x2;
	float syy=0.0f, sxy=0.0f, sxx=0.0f, m1=0.0f, m2=0.0f ,n=(float)(img2.getHeight()*img2.getWidth());
//	IplImage *ipl1 = (*this).img;
//	IplImage *ipl2 = img2.img;

	//find the means
	for (i=0;i<img2.getHeight();i++)
	{
		for (j=0;j<img2.getWidth();j++)
		{
			m1 += *(*this)(j+width_init,i+height_init);//(double)(ipl1->imageData[i*ipl1->widthStep + j ]);
			m2 += *img2(j,i); //(double)(ipl2->imageData[i*ipl2->widthStep + j ]);
		}//[ row * ipl->widthStep +  col * ipl->nChannels +  channel ];
	}
    m1 /= n;
	m2 /= n;

	for (i=0;i<img2.getHeight();i++)
	{
		for (j=0;j<img2.getWidth();j++)
		{
			x1 = *(*this)(j+width_init,i+height_init) - m1;//(double)(ipl1->imageData[i*ipl1->widthStep + j]) - m1;
			x2 = *img2(j,i) - m2;//(double)(ipl2->imageData[i*ipl2->widthStep + j]) - m2;
			sxx += x1*x1;
			syy += x2*x2;
			sxy += x1*x2;
		}
	}

	return sxy / sqrt(sxx * syy);
#else
	return 0;
#endif
}


/*---------------------------------------------------------------
					cross_correlation
 ---------------------------------------------------------------*/
void  CImage::cross_correlation(
	const CImage	&patch_img,
	size_t				&x_max,
	size_t				&y_max,
	double				&max_val,
	int					x_search_ini,
	int					y_search_ini,
	int					x_search_size,
	int					y_search_size,
	CImage				*out_corr_image)const
{
	MRPT_START

	makeSureImageIsLoaded();   // For delayed loaded images stored externally

#if MRPT_HAS_OPENCV
	double		mini;
	CvPoint		min_point,max_point;

	bool entireImg = (x_search_ini<0 || y_search_ini<0 || x_search_size<0 || y_search_size<0);

	const IplImage *im, *patch_im;

	if( this->isColor() && patch_img.isColor() )
	{
		const IplImage *im_ = this->getAs<IplImage>();
		const IplImage *patch_im_ = patch_img.getAs<IplImage>();

		IplImage *aux = cvCreateImage( cvGetSize( im_ ), 8, 1 );
		IplImage *aux2 = cvCreateImage( cvGetSize( patch_im_ ), 8, 1 );
		cvCvtColor( im_, aux, CV_BGR2GRAY );
		cvCvtColor( patch_im_, aux2, CV_BGR2GRAY );
		im = aux;
		patch_im = aux2;
	}
	else
	{
		im = this->getAs<IplImage>();
		patch_im = patch_img.getAs<IplImage>();
	}

	if (entireImg)
	{
		x_search_size = im->width - patch_im->width;
		y_search_size = im->height - patch_im->height;
	}

	// JLBC: Perhaps is better to raise the exception always??
	if ((x_search_ini + x_search_size  + patch_im->width-1)>im->width)
		x_search_size -= (x_search_ini + x_search_size + patch_im->width-1) - im->width;

	if ((y_search_ini + y_search_size  + patch_im->height-1)>im->height)
		y_search_size -= (y_search_ini + y_search_size  + patch_im->height-1) - im->height;

	ASSERT_( (x_search_ini + x_search_size  + patch_im->width-1)<=im->width )
	ASSERT_( (y_search_ini + y_search_size  + patch_im->height-1)<=im->height )

	IplImage *result = cvCreateImage(cvSize(x_search_size+1,y_search_size+1),IPL_DEPTH_32F, 1);

	const IplImage *ipl_ext;

	if (!entireImg)
	{
		IplImage *aux= cvCreateImage(cvSize(patch_im->width+x_search_size,patch_im->height+y_search_size),IPL_DEPTH_8U, 1);
		for (unsigned int i = 0 ; i < (unsigned int)y_search_size ; i++)
		{
			memcpy( &aux->imageData[i * aux->widthStep ],
					&im->imageData[(i+y_search_ini) * im->widthStep + x_search_ini * im->nChannels],
					aux->width * aux->nChannels ); //widthStep);  <-- JLBC: widthstep SHOULD NOT be used as the length of each row (the last one may be shorter!!)
		}
		ipl_ext = aux;
	}
	else
	{
		ipl_ext = im;
	}

	// Compute cross correlation:
	cvMatchTemplate(ipl_ext,patch_im,result,CV_TM_CCORR_NORMED);
	//cvMatchTemplate(ipl_ext,patch_im,result,CV_TM_CCOEFF_NORMED);

	// Find the max point:
	cvMinMaxLoc(result,&mini,&max_val,&min_point,&max_point,NULL);
	x_max = max_point.x+x_search_ini+(round(patch_im->width-1)/2);
	y_max = max_point.y+y_search_ini+(round(patch_im->height-1)/2);

	// Free memory:
	if (!entireImg) {
		IplImage *aux = const_cast<IplImage*>(ipl_ext);
		cvReleaseImage( &aux );
		ipl_ext=NULL;
	}

	// Leave output image?
	if (out_corr_image)
		out_corr_image->setFromIplImage(result);
	else
		cvReleaseImage( &result );
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END
}


/*---------------------------------------------------------------
						normalize
 ---------------------------------------------------------------*/
void  CImage::normalize()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    IplImage *ipl = getAs<IplImage>();	// Source Image
	ASSERT_(ipl)
	ASSERTMSG_( ipl->nChannels==1, "CImage::normalize() only defined for grayscale images.")

	uint8_t min_=255,max_=1;
	for (int y=0;y<ipl->height;y++)
	{
		const uint8_t *ptr = reinterpret_cast<const uint8_t*>( ipl->imageData + y*ipl->widthStep );
		for (int x=0;x<ipl->width;x++)
		{
			const uint8_t val = *ptr++;
			if (min_ > val) min_=val;
			if (max_ < val) max_=val;
		}
	}

	// Compute scale factor & build convert look-up-table:
	const double s=255.0/((double)max_-(double)min_);
	uint8_t lut[256];
	for (int v=0;v<256;v++) lut[v] = static_cast<uint8_t>( (v-min_)*s );

	// Apply LUT:
	for (int y=0;y<ipl->height;y++)
	{
		uint8_t *ptr = reinterpret_cast<uint8_t*>( ipl->imageData + y*ipl->widthStep );
		for (int x=0;x<ipl->width;x++)
		{
			*ptr = lut[*ptr];
			ptr++;
		}
	}
#endif
}

/*---------------------------------------------------------------
						getAsMatrix
 ---------------------------------------------------------------*/
void  CImage::getAsMatrix(
	CMatrixFloat &outMatrix,
	bool	doResize,
	int		x_min,
	int		y_min,
	int		x_max,
	int		y_max
	)  const
{
#if MRPT_HAS_OPENCV
	MRPT_START

	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);

	// Set sizes:
	if (x_max==-1) x_max=((IplImage*)img)->width-1;
	if (y_max==-1) y_max=((IplImage*)img)->height-1;

	ASSERT_(x_min>=0 && x_min<((IplImage*)img)->width && x_min<x_max);
	ASSERT_(y_min>=0 && y_min<((IplImage*)img)->height && y_min<y_max);

	int		lx = (x_max-x_min+1);
	int		ly = (y_max-y_min+1);

	if (doResize || (int)outMatrix.getRowCount()<ly || (int)outMatrix.getColCount()<lx)
		outMatrix.setSize( y_max-y_min+1,x_max-x_min+1 );

	if (isColor())
	{
		// Luminance: Y = 0.3R + 0.59G + 0.11B
		for (int y=0;y<ly;y++)
		{
			unsigned char	*pixels = this->get_unsafe(x_min,y_min+y,0);
			float           aux;
			for (int x=0;x<lx;x++)
			{
                aux  = *pixels++ * 0.3f*(1.0f/255);
                aux += *pixels++ * 0.59f*(1.0f/255);
                aux += *pixels++ * 0.11f*(1.0f/255);
				outMatrix.set_unsafe(y,x, aux);
			}
		}
	}
	else
	{
		for (int y=0;y<ly;y++)
		{
			unsigned char	*pixels = this->get_unsafe(x_min,y_min+y,0);
			for (int x=0;x<lx;x++)
				outMatrix.set_unsafe(y,x, (*pixels++)*(1.0f/255) );
		}
	}

	MRPT_END
#endif
}

/*---------------------------------------------------------------
						getAsRGBMatrices
 ---------------------------------------------------------------*/
void  CImage::getAsRGBMatrices(
	mrpt::math::CMatrixFloat	&outMatrixR,
	mrpt::math::CMatrixFloat	&outMatrixG,
	mrpt::math::CMatrixFloat	&outMatrixB,
	bool	doResize,
	int		x_min,
	int		y_min,
	int		x_max,
	int		y_max
	)  const
{
#if MRPT_HAS_OPENCV
	MRPT_START

	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);

	// Set sizes:
	if (x_max==-1) x_max=((IplImage*)img)->width-1;
	if (y_max==-1) y_max=((IplImage*)img)->height-1;

	ASSERT_(x_min>=0 && x_min<((IplImage*)img)->width && x_min<x_max);
	ASSERT_(y_min>=0 && y_min<((IplImage*)img)->height && y_min<y_max);

	int		lx = (x_max-x_min+1);
	int		ly = (y_max-y_min+1);

	if (doResize || (int)outMatrixR.getRowCount()<ly || (int)outMatrixR.getColCount()<lx)
		outMatrixR.setSize( y_max-y_min+1,x_max-x_min+1 );
	if (doResize || (int)outMatrixG.getRowCount()<ly || (int)outMatrixG.getColCount()<lx)
		outMatrixG.setSize( y_max-y_min+1,x_max-x_min+1 );
	if (doResize || (int)outMatrixB.getRowCount()<ly || (int)outMatrixB.getColCount()<lx)
		outMatrixB.setSize( y_max-y_min+1,x_max-x_min+1 );

	if (isColor())
	{
		for (int y=0;y<ly;y++)
		{
			unsigned char	*pixels = this->get_unsafe(x_min,y_min+y,0);
			float           aux;
			for (int x=0;x<lx;x++)
			{
                aux = *pixels++ * (1.0f/255);
				outMatrixR.set_unsafe(y,x, aux);
                aux = *pixels++ * (1.0f/255);
				outMatrixG.set_unsafe(y,x, aux);
                aux = *pixels++ * (1.0f/255);
				outMatrixB.set_unsafe(y,x, aux);

			}
		}
	}
	else
	{
		for (int y=0;y<ly;y++)
		{
			unsigned char	*pixels = this->get_unsafe(x_min,y_min+y,0);
			for (int x=0;x<lx;x++)
			{
				outMatrixR.set_unsafe(y,x, (*pixels)*(1.0f/255) );
				outMatrixG.set_unsafe(y,x, (*pixels)*(1.0f/255) );
				outMatrixB.set_unsafe(y,x, (*pixels++)*(1.0f/255) );
			}
		}
	}

	MRPT_END
#endif
}


/*---------------------------------------------------------------
						cross_correlation_FFT
 ---------------------------------------------------------------*/
void  CImage::cross_correlation_FFT(
	const CImage	&in_img,
	CMatrixFloat		&out_corr,
	int					u_search_ini,
	int					v_search_ini,
	int					u_search_size,
	int					v_search_size,
	float				biasThisImg,
	float				biasInImg ) const
{
	MRPT_START

	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);

	// Set limits:
	if (u_search_ini==-1)	u_search_ini=0;
	if (v_search_ini==-1)	v_search_ini=0;
	if (u_search_size==-1)	u_search_size=getWidth();
	if (v_search_size==-1)	v_search_size=getHeight();

	int	u_search_end = u_search_ini + u_search_size-1;
	int	v_search_end = v_search_ini + v_search_size-1;

	ASSERT_(u_search_end<(int)getWidth());
	ASSERT_(v_search_end<(int)getHeight());

	// Find smallest valid size:
	size_t		x,y;
	size_t		actual_lx = max( u_search_size,(int)in_img.getWidth());
	size_t		actual_ly = max( v_search_size,(int)in_img.getHeight());
	size_t		lx = math::round2up( actual_lx );
	size_t		ly = math::round2up( actual_ly );

//	printf("ly=%u lx=%u\n",ly,lx);

	CMatrix		i1(ly,lx),i2(ly,lx);

	// We fill the images with the bias, such as when we substract the bias later on,
	//  those pixels not really occupied by the image really becomes zero:
	i1.fill(biasInImg);
	i2.fill(biasThisImg);

	// Get as matrixes, padded with zeros up to power-of-two sizes:
	getAsMatrix(
		i2,
		false,
		u_search_ini,
		v_search_ini,
		u_search_ini+u_search_size-1,
		v_search_ini+v_search_size-1 );
    in_img.getAsMatrix(
		i1,
		false);

	// Remove the bias now:
	i2.array() -= biasThisImg;
	i1.array() -= biasInImg;

	// Fill the "padded zeros" with copies of the images:
//	SAVE_MATRIX(i1); SAVE_MATRIX(i2);

	// FFT:
	CMatrix		I1_R,I1_I,I2_R,I2_I,ZEROS(ly,lx);
	math::dft2_complex(i1,ZEROS,I1_R,I1_I);
	math::dft2_complex(i2,ZEROS,I2_R,I2_I);

//	SAVE_MATRIX(I1_R); SAVE_MATRIX(I1_I);
//	SAVE_MATRIX(I2_R); SAVE_MATRIX(I2_I);

	// Compute the COMPLEX division of I2 by I1:
	for (y = 0;y<ly;y++)
		for (x = 0;x<lx;x++)
		{
			float	r1 = I1_R.get_unsafe(y,x);
			float	r2 = I2_R.get_unsafe(y,x);

			float	ii1 = I1_I.get_unsafe(y,x);
			float	ii2 = I2_I.get_unsafe(y,x);

			float	den = square(r1)+square(ii1);
			I2_R.set_unsafe(y,x, (r1*r2+ii1*ii2)/den);
			I2_I.set_unsafe(y,x, (ii2*r1-r2*ii1)/den);
		}

//	I2_R.saveToTextFile("DIV_R.txt");
//	I2_I.saveToTextFile("DIV_I.txt");

	// IFFT:
	CMatrix		res_R,res_I;
	math::idft2_complex(I2_R,I2_I,res_R,res_I);

	out_corr.setSize(actual_ly,actual_lx);
	for (y = 0;y<actual_ly;y++)
		for (x = 0;x<actual_lx;x++)
			out_corr(y,x) = sqrt( square(res_R(y,x)) + square(res_I(y,x)) );

	MRPT_END
}


/*---------------------------------------------------------------
						getAsMatrixTiled
 ---------------------------------------------------------------*/
void  CImage::getAsMatrixTiled( CMatrix &outMatrix )  const
{
#if MRPT_HAS_OPENCV
	MRPT_START

	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);

	// The size of the matrix:
	size_t	matrix_lx = outMatrix.getColCount();
	size_t	matrix_ly = outMatrix.getRowCount();

	if (isColor())
	{
		// Luminance: Y = 0.3R + 0.59G + 0.11B
		for (unsigned int y=0;y<matrix_ly;y++)
		{
			unsigned char	*min_pixels = (*this)(0,y % ((IplImage*)img)->height,0);
			unsigned char	*max_pixels = min_pixels + ((IplImage*)img)->width*3;
			unsigned char	*pixels = min_pixels;
			float           aux;
			for (unsigned int x=0;x<matrix_lx;x++)
			{
			    aux  = *pixels++ * 0.30f;
			    aux += *pixels++ * 0.59f;
			    aux += *pixels++ * 0.11f;
				outMatrix.set_unsafe(y,x, aux);
				if (pixels>=max_pixels)
					pixels = min_pixels;
			}
		}
	}
	else
	{
		for (unsigned int y=0;y<matrix_ly;y++)
		{
			unsigned char	*min_pixels = (*this)(0,y % ((IplImage*)img)->height,0);
			unsigned char	*max_pixels = min_pixels + ((IplImage*)img)->width;
			unsigned char	*pixels = min_pixels;
			for (unsigned int x=0;x<matrix_lx;x++)
			{
				outMatrix.set_unsafe(y,x, *pixels++ );
				if (pixels>=max_pixels) pixels = min_pixels;
			}
		}
	}

	MRPT_END
#endif
}


/*---------------------------------------------------------------
						setExternalStorage
 ---------------------------------------------------------------*/
void CImage::setExternalStorage( const std::string &fileName ) MRPT_NO_THROWS
{
	releaseIpl();
	m_externalFile = fileName;
	m_imgIsExternalStorage = true;
}

/*---------------------------------------------------------------
						unload
 ---------------------------------------------------------------*/
void CImage::unload() const MRPT_NO_THROWS
{
	if (m_imgIsExternalStorage)
		const_cast<CImage*>(this)->releaseIpl( true ); // Do NOT mark the image as NON external
}

/*---------------------------------------------------------------
						releaseIpl
 ---------------------------------------------------------------*/
void CImage::releaseIpl(bool thisIsExternalImgUnload) MRPT_NO_THROWS
{
#if MRPT_HAS_OPENCV
	if (img && !m_imgIsReadOnly)
	{
		IplImage *ptr=(IplImage*)img;
		cvReleaseImage( &ptr );
	}
	img = NULL;
	m_imgIsReadOnly = false;
	if (!thisIsExternalImgUnload)
	{
		m_imgIsExternalStorage = false;
		m_externalFile = string();
	}
#endif
}

/*---------------------------------------------------------------
				makeSureImageIsLoaded
 ---------------------------------------------------------------*/
void CImage::makeSureImageIsLoaded() const throw (std::exception,utils::CExceptionExternalImageNotFound )
{
	if (img!=NULL) return;  // OK, continue

	if (m_imgIsExternalStorage)
	{
		// Load the file:
		string wholeFile;
		getExternalStorageFileAbsolutePath(wholeFile);

		const std::string tmpFile = m_externalFile;

		bool ret = const_cast<CImage*>(this)->loadFromFile(wholeFile);

		// These are removed by "loadFromFile", and that's good, just fix it here and carry on.
		m_imgIsExternalStorage = true;
		m_externalFile = tmpFile;

		if (!ret)
			THROW_TYPED_EXCEPTION_CUSTOM_MSG1("Error loading externally-stored image from: %s", wholeFile.c_str() ,CExceptionExternalImageNotFound);
	}
	else THROW_EXCEPTION("img is NULL in a non-externally stored image.");
}

/*---------------------------------------------------------------
				getExternalStorageFileAbsolutePath
 ---------------------------------------------------------------*/
void CImage::getExternalStorageFileAbsolutePath(std::string &out_path) const
{
	ASSERT_(m_externalFile.size()>2);

	if (m_externalFile[0]=='/' || ( m_externalFile[1]==':' && m_externalFile[2]=='\\' ) )
	{
		out_path= m_externalFile;
	}
	else
	{
		out_path = IMAGES_PATH_BASE;

		size_t N=IMAGES_PATH_BASE.size()-1;
		if (IMAGES_PATH_BASE[N]!='/' && IMAGES_PATH_BASE[N]!='\\' )
			out_path+= "/";

		out_path+= m_externalFile;
	}
}

/*---------------------------------------------------------------
				flipVertical
 ---------------------------------------------------------------*/
void CImage::flipVertical(bool also_swapRB )
{
#if MRPT_HAS_OPENCV
	IplImage *ptr=(IplImage*)img;
	int options = CV_CVTIMG_FLIP;
	if(also_swapRB) options |= CV_CVTIMG_SWAP_RB;
	cvConvertImage(ptr,ptr,options);
#endif
}

/*---------------------------------------------------------------
				swapRB
 ---------------------------------------------------------------*/
void CImage::swapRB()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	IplImage *ptr=(IplImage*)img;
	cvConvertImage(ptr,ptr,CV_CVTIMG_SWAP_RB);
#endif
}

/*---------------------------------------------------------------
                    rectifyImageInPlace
 ---------------------------------------------------------------*/
void CImage::rectifyImageInPlace( void *mapX, void *mapY )
{
#if MRPT_HAS_OPENCV
    makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);

#if MRPT_OPENCV_VERSION_NUM<0x200
	THROW_EXCEPTION("This method requires OpenCV 2.0.0 or above.")
#else

    IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

    cv::Mat *_mapX, *_mapY;
    _mapX = static_cast<cv::Mat*>(mapX);
    _mapY = static_cast<cv::Mat*>(mapY);

    IplImage _mapXX = *_mapX;
    IplImage _mapYY = *_mapY;

    cvRemap(srcImg,outImg, &_mapXX,&_mapYY,CV_INTER_CUBIC);
#endif

    releaseIpl();
    img = outImg;
#endif
}

/*---------------------------------------------------------------
                    rectifyImageInPlace
 ---------------------------------------------------------------*/
void CImage::rectifyImageInPlace( const mrpt::utils::TCamera &cameraParams  )
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	// MRPT -> OpenCV Input Transformation
	IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	double aux1[3][3], aux2[1][5];
	const CMatrixDouble33 &cameraMatrix = cameraParams.intrinsicParams;

	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			aux1[i][j] = cameraMatrix(i,j);
	for (int i=0;i<5;i++)
		aux2[0][i]=cameraParams.dist[i];

	CvMat inMat =  cvMat( cameraMatrix.getRowCount(), cameraMatrix.getColCount(), CV_64F, aux1 );
	CvMat distM =  cvMat( 1, 5, CV_64F, aux2 );

	// Remove distortion
	cvUndistort2( srcImg, outImg, &inMat, &distM );

	// Assign the output image to the IPLImage pointer within the CImage
	releaseIpl();
	img = outImg;
#endif
}

/*---------------------------------------------------------------
                        rectifyImage
 ---------------------------------------------------------------*/
void CImage::rectifyImage(
	CImage &out_img,
	const mrpt::utils::TCamera &cameraParams ) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	// MRPT -> OpenCV Input Transformation
	const IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	double aux1[3][3], aux2[1][5];
	const CMatrixDouble33 &cameraMatrix = cameraParams.intrinsicParams;

	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			aux1[i][j] = cameraMatrix(i,j);
	for (int i=0;i<5;i++)
		aux2[0][i]=cameraParams.dist[i];

	CvMat inMat =  cvMat( cameraMatrix.getRowCount(), cameraMatrix.getColCount(), CV_64F, aux1 );
	CvMat distM =  cvMat( 1, 5, CV_64F, aux2 );

	// Remove distortion
	cvUndistort2( srcImg, outImg, &inMat, &distM );

	// OpenCV -> MRPT Output Transformation
	out_img.loadFromIplImage( outImg );

	// Release Output Image
	cvReleaseImage( &outImg );
#endif
} // end CImage::rectifyImage


/*---------------------------------------------------------------
                        filterMedian
 ---------------------------------------------------------------*/
void CImage::filterMedian( CImage &out_img, int W ) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	// MRPT -> OpenCV Input Transformation
	const IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	// Filter
	cvSmooth( srcImg, outImg, CV_MEDIAN, W );

	outImg->origin = srcImg->origin;

	// OpenCV -> MRPT Output Transformation
	out_img.loadFromIplImage( outImg );

	// Release Output Image
	cvReleaseImage( &outImg );
#endif
}

/*---------------------------------------------------------------
                        filterMedian
 ---------------------------------------------------------------*/
void CImage::filterMedianInPlace( int W )
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	// MRPT -> OpenCV Input Transformation
	IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	// Filter
	cvSmooth( srcImg, outImg, CV_MEDIAN, W );

	outImg->origin = srcImg->origin;

	// Assign the output image to the IPLImage pointer within the CImage
	releaseIpl();
	img = outImg;
#endif
}

/*---------------------------------------------------------------
                        filterGaussian
 ---------------------------------------------------------------*/
void CImage::filterGaussian( CImage &out_img, int W, int H ) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	// MRPT -> OpenCV Input Transformation
	const IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	// Filter
	cvSmooth( srcImg, outImg, CV_GAUSSIAN, W, H );

	outImg->origin = srcImg->origin;

	// OpenCV -> MRPT Output Transformation
	out_img.loadFromIplImage( outImg );

	// Release Output Image
	cvReleaseImage( &outImg );
#endif
}

/*---------------------------------------------------------------
                        filterGaussianInPlace
 ---------------------------------------------------------------*/
void CImage::filterGaussianInPlace( int W, int H )
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	// MRPT -> OpenCV Input Transformation
	IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	// Filter
	cvSmooth( srcImg, outImg, CV_GAUSSIAN, W, H );

	outImg->origin = srcImg->origin;

	// Assign the output image to the IPLImage pointer within the CImage
	releaseIpl();
	img = outImg;
#endif
}


/*---------------------------------------------------------------
                        scaleImage
 ---------------------------------------------------------------*/
void CImage::scaleImage( unsigned int width, unsigned int height, TInterpolationMethod interp )
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img!=NULL);
	IplImage *srcImg = getAs<IplImage>();	// Source Image

	if( static_cast<unsigned int>(srcImg->width) == width && static_cast<unsigned int>(srcImg->height) == height )
		return;

	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvSize(width,height), srcImg->depth, srcImg->nChannels );

	// Resize:
	cvResize( srcImg, outImg, (int)interp );

	outImg->origin = srcImg->origin;

	// Assign the output image to the IPLImage pointer within the CImage
	releaseIpl();
	img = outImg;
#endif
}

/*---------------------------------------------------------------
                        scaleImage
 ---------------------------------------------------------------*/
void CImage::scaleImage( CImage &out_img, unsigned int width, unsigned int height, TInterpolationMethod interp ) const
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	const IplImage *srcImg = getAs<IplImage>();	// Source Image

	if( static_cast<unsigned int>(srcImg->width) == width && static_cast<unsigned int>(srcImg->height) == height )
	{
		// already at the required size:
		out_img = *this;
		return;
	}

	IplImage *outImg;		// Output Image
	outImg = cvCreateImage( cvSize(width,height), srcImg->depth, srcImg->nChannels );

	// Resize:
	cvResize( srcImg, outImg, (int)interp );
	outImg->origin = srcImg->origin;

	// Assign:
	out_img.setFromIplImage(outImg);
#endif
}


/*---------------------------------------------------------------
                        rotateImage
 ---------------------------------------------------------------*/
void CImage::rotateImage( double angle_radians, unsigned int center_x, unsigned int center_y, double scale )
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);

	IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	// Based on the blog entry:
	// http://blog.weisu.org/2007/12/opencv-image-rotate-and-zoom-rotation.html

	// Apply rotation & scale:
	float m[6];
	CvMat M = cvMat(2, 3, CV_32F, m);

	m[0] = (float)(scale*cos(angle_radians));
	m[1] = (float)(scale*sin(angle_radians));
	m[3] = -m[1];
	m[4] = m[0];
	m[2] = center_x;
	m[5] = center_y;

	cvGetQuadrangleSubPix( srcImg, outImg, &M );

	outImg->origin = srcImg->origin;

	// Assign the output image to the IPLImage pointer within the CImage
	releaseIpl();
	img = outImg;

#endif
}


/** Draw onto this image the detected corners of a chessboard. The length of cornerCoords must be the product of the two check_sizes.
*
* \param cornerCoords [IN] The pixel coordinates of all the corners.
* \param check_size_x [IN] The number of squares, in the X direction
* \param check_size_y [IN] The number of squares, in the Y direction
*
* \return false if the length of cornerCoords is inconsistent (nothing is drawn then).
*/
bool CImage::drawChessboardCorners(
	std::vector<TPixelCoordf> 	&cornerCoords,
	unsigned int  check_size_x,
	unsigned int  check_size_y,
	unsigned int  lines_width,
	unsigned int r)
{
#if MRPT_HAS_OPENCV

	if (cornerCoords.size()!=check_size_x*check_size_y) return false;

	IplImage* ipl = this->getAs<IplImage>();

	unsigned int x, y,i;
	CvPoint prev_pt = cvPoint(0, 0);
	const int line_max = 8;
	CvScalar line_colors[8];

	line_colors[0] = CV_RGB(255,0,0);
	line_colors[1] = CV_RGB(255,128,0);
	line_colors[2] = CV_RGB(255,128,0);
	line_colors[3] = CV_RGB(200,200,0);
	line_colors[4] = CV_RGB(0,255,0);
	line_colors[5] = CV_RGB(0,200,200);
	line_colors[6] = CV_RGB(0,0,255);
	line_colors[7] = CV_RGB(255,0,255);

	CCanvas::selectTextFont("10x20");

	for( y = 0, i = 0; y < check_size_y; y++ )
	{
		CvScalar color = line_colors[y % line_max];
		for( x = 0; x < check_size_x; x++, i++ )
		{
			CvPoint pt;
			pt.x = cvRound( cornerCoords[i].x);
			pt.y = cvRound( cornerCoords[i].y);

			if( i != 0 ) cvLine(ipl, prev_pt, pt, color, lines_width );

			cvLine(ipl,
					  cvPoint( pt.x - r, pt.y - r ),
					  cvPoint( pt.x + r, pt.y + r ), color, lines_width );
			cvLine(ipl,
					  cvPoint( pt.x - r, pt.y + r),
					  cvPoint( pt.x + r, pt.y - r), color, lines_width );

			if (r>0)
				cvCircle(ipl, pt, r+1, color );
			prev_pt = pt;

			// Text label with the corner index in the first and last corners:
			if (i==0 || i==cornerCoords.size()-1)
				CCanvas::textOut(pt.x+5,pt.y-5,mrpt::format("%u",i), mrpt::utils::TColor::blue);
		}
	}

	return true;
#else
	return false;
#endif
}


/** Replaces this grayscale image with a RGB version of it.
  * \sa grayscaleInPlace
  */
void CImage::colorImage( CImage  &ret ) const
{
#if MRPT_HAS_OPENCV
	if (this->isColor()) {
		ret = *this;
		return;
	}

	const IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, 3 );

	cvCvtColor( srcImg, outImg, CV_GRAY2BGR );

	outImg->origin = srcImg->origin;

	// Assign the output image to the IPLImage pointer within the CImage
	ret.setFromIplImage(outImg);
#endif
}

/** Replaces this grayscale image with a RGB version of it.
  * \sa grayscaleInPlace
  */
void CImage::colorImageInPlace()
{
#if MRPT_HAS_OPENCV
	if (this->isColor()) return;

	IplImage *srcImg = getAs<IplImage>();	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, 3 );

	cvCvtColor( srcImg, outImg, CV_GRAY2BGR );

	outImg->origin = srcImg->origin;

	// Assign the output image to the IPLImage pointer within the CImage
	releaseIpl();
	img = outImg;
#endif
}

/*---------------------------------------------------------------
						joinImagesHorz
 ---------------------------------------------------------------*/
void CImage::joinImagesHorz( const CImage &im1, const CImage &im2 )
{
#if MRPT_HAS_OPENCV
	ASSERT_( im1.getHeight() == im2.getHeight() );

	const IplImage* _im1 = im1.getAs<IplImage>();
	const IplImage* _im2 = im2.getAs<IplImage>();

	ASSERT_( _im1->depth == _im2->depth && _im1->nChannels == _im2->nChannels );

	IplImage *out = cvCreateImage( cvSize( _im1->width + _im2->width, _im1->height ), _im1->depth, _im1->nChannels );

	cvSetImageROI( out, cvRect( 0, 0, _im1->width, _im1->height ) );
	cvCopy( _im1, out );
	cvSetImageROI( out, cvRect( _im1->width, 0, _im2->width, _im2->height ) );
	cvCopy( _im2, out );
	cvSetImageROI( out, cvRect( 0, 0, out->width, out->height ) );

	IplImage *out2;
	if( (int)_im1->nChannels != (int)this->getChannelCount() )	// Convert the input to the output channel format
	{
		out2 = cvCreateImage( cvSize( _im1->width + _im2->width, _im1->height ), _im1->depth, this->getChannelCount() );
		cvCvtColor( out, out2, CV_GRAY2BGR );
		this->setFromIplImageReadOnly( out2 );
	}
	else	// Assign the output image to the IPLImage pointer within the CImage
		this->setFromIplImageReadOnly( out );



#endif
} // end


/*---------------------------------------------------------------
						equalizeHist
 ---------------------------------------------------------------*/
void CImage::equalizeHist( CImage  &outImg ) const
{
#if MRPT_HAS_OPENCV
	// Convert to a single luminance channel image
	const IplImage *srcImg = getAs<IplImage>();	// Source Image
    ASSERT_(srcImg!=NULL)

	outImg.changeSize( srcImg->width, srcImg->height, 1,isOriginTopLeft());

	if (srcImg->nChannels==1)
	{	// Grayscale:
		cvEqualizeHist(srcImg, outImg.getAs<IplImage>() );
	}
	else
	{	// Color:
		IplImage *hsv = cvCreateImage(cvGetSize(srcImg), 8, 3);
		IplImage *h = cvCreateImage(cvGetSize(srcImg), 8, 1);
		IplImage *s = cvCreateImage(cvGetSize(srcImg), 8, 1);
		IplImage *v = cvCreateImage(cvGetSize(srcImg), 8, 1);

		cvCvtColor(srcImg, hsv, CV_BGR2HSV);
		cvSplit(hsv, h, s, v, NULL);

		cvEqualizeHist(v, v);

		cvMerge(h, s, v, NULL, hsv);
		cvCvtColor(hsv, outImg.getAs<IplImage>(), CV_HSV2BGR);

		cvReleaseImage(&hsv);
		cvReleaseImage(&h);
		cvReleaseImage(&s);
		cvReleaseImage(&v);
	}

#endif
}

/*---------------------------------------------------------------
						equalizeHistInPlace
 ---------------------------------------------------------------*/
void CImage::equalizeHistInPlace()
{
#if MRPT_HAS_OPENCV
	// Convert to a single luminance channel image
	IplImage *srcImg = getAs<IplImage>();	// Source Image
    ASSERT_(srcImg!=NULL);

	IplImage *outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );
	outImg->origin = srcImg->origin;

	if (srcImg->nChannels==1)
	{	// Grayscale:
		cvEqualizeHist(srcImg, outImg );
	}
	else
	{	// Color:
		IplImage *hsv = cvCreateImage(cvGetSize(srcImg), 8, 3);
		IplImage *h = cvCreateImage(cvGetSize(srcImg), 8, 1);
		IplImage *s = cvCreateImage(cvGetSize(srcImg), 8, 1);
		IplImage *v = cvCreateImage(cvGetSize(srcImg), 8, 1);

		cvCvtColor(srcImg, hsv, CV_BGR2HSV);
		cvSplit(hsv, h, s, v, NULL);

		cvEqualizeHist(v, v);

		cvMerge(h, s, v, NULL, hsv);
		cvCvtColor(hsv, outImg, CV_HSV2BGR);

		cvReleaseImage(&hsv);
		cvReleaseImage(&h);
		cvReleaseImage(&s);
		cvReleaseImage(&v);
	}

	// Assign the output image to the IPLImage pointer within the CImage
	releaseIpl();
	img = outImg;

#endif
}


template <unsigned int HALF_WIN_SIZE>
void image_KLT_response_template(const uint8_t* in, const int widthStep, int x, int y, int32_t &_gxx,int32_t &_gyy,int32_t &_gxy)
{
	const unsigned int min_x = x-HALF_WIN_SIZE;
	const unsigned int min_y = y-HALF_WIN_SIZE;

	int32_t gxx = 0;
	int32_t gxy = 0;
	int32_t gyy = 0;

	const unsigned int WIN_SIZE = 1+2*HALF_WIN_SIZE;

	unsigned int yy = min_y;
	for (unsigned int iy=WIN_SIZE; iy ; --iy, ++yy)
	{
		const uint8_t* ptr = in + widthStep*yy+ min_x;
		unsigned int xx = min_x;
		for (unsigned int ix = WIN_SIZE; ix; --ix, ++xx)
		{
			const int32_t dx = ptr[+1]-ptr[-1];
			const int32_t dy = ptr[+widthStep]-ptr[-widthStep];
			gxx += dx * dx;
			gxy += dx * dy;
			gyy += dy * dy;
		}
	}
	_gxx = gxx;
	_gyy = gyy;
	_gxy = gxy;
}



float CImage::KLT_response(
	const unsigned int x,
	const unsigned int y,
	const unsigned int half_window_size ) const
{
#if MRPT_HAS_OPENCV
	const IplImage *srcImg = this->getAs<IplImage>();
    ASSERT_(srcImg!=NULL)
	ASSERTMSG_(srcImg->nChannels==1, "KLT_response only works with grayscale images.")

	const unsigned int img_w = srcImg->width;
	const unsigned int img_h = srcImg->height;
	const int widthStep = srcImg->widthStep;

	// If any of those predefined values worked, do the generic way:
	const unsigned int min_x = x-half_window_size;
	const unsigned int max_x = x+half_window_size;
	const unsigned int min_y = y-half_window_size;
	const unsigned int max_y = y+half_window_size;

	// Since min_* are "unsigned", checking "<" will detect negative numbers:
	ASSERTMSG_(min_x<img_w && max_x<img_w && min_y<img_h && max_y<img_h, "Window is out of image bounds")

	// Gradient sums: Use integers since they're much faster than doubles/floats!!
	int32_t gxx = 0;
	int32_t gxy = 0;
	int32_t gyy = 0;

	const uint8_t* img_data = reinterpret_cast<const uint8_t*>(srcImg->imageData);  //*VERY IMPORTANT*: Use unsigned
	switch (half_window_size)
	{
		case 2: image_KLT_response_template<2>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 3: image_KLT_response_template<3>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 4: image_KLT_response_template<4>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 5: image_KLT_response_template<5>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 6: image_KLT_response_template<6>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 7: image_KLT_response_template<7>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 8: image_KLT_response_template<8>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 9: image_KLT_response_template<9>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 10: image_KLT_response_template<10>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 11: image_KLT_response_template<11>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 12: image_KLT_response_template<12>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 13: image_KLT_response_template<13>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 14: image_KLT_response_template<14>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 15: image_KLT_response_template<15>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 16: image_KLT_response_template<16>(img_data,widthStep,x,y,gxx,gyy,gxy); break;
		case 32: image_KLT_response_template<32>(img_data,widthStep,x,y,gxx,gyy,gxy); break;

		default:
			for (unsigned int yy = min_y; yy<=max_y; yy++)
			{
				const uint8_t* ptr = img_data + widthStep*yy+ min_x;
				for (unsigned int xx = min_x; xx<=max_x; xx++)
				{
					const int32_t dx = ptr[+1]-ptr[-1];
					const int32_t dy = ptr[+widthStep]-ptr[-widthStep];
					gxx += dx * dx;
					gxy += dx * dy;
					gyy += dy * dy;
				}
			}
		 break;
	}
	// Convert to float's and normalize in the way:
	const float K = 0.5f/( (max_y-min_y+1) * (max_x-min_x+1) );
	const float Gxx = gxx * K;
	const float Gxy = gxy * K;
	const float Gyy = gyy * K;

	// Return the minimum eigenvalue of:
	//    ( gxx  gxy )
	//    ( gxy  gyy )
	// See, for example: mrpt::math::detail::eigenVectorsMatrix_special_2x2():
	const float t  = Gxx+Gyy;         // Trace
	const float de = Gxx*Gyy-Gxy*Gxy; // Det
	// The smallest eigenvalue is:
	return 0.5f * (t - std::sqrt( t*t - 4.0f*de ) );
#else
	return 0;
#endif
}


/** Marks the channel ordering in a color image (this doesn't actually modify the image data, just the format description)
  */
void CImage::setChannelsOrder_RGB()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);
	strcpy( ((IplImage*)img)->channelSeq, "RGB");
#else
		THROW_EXCEPTION("MRPT compiled without OpenCV")
#endif
}

void CImage::setChannelsOrder_BGR()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);
	strcpy( ((IplImage*)img)->channelSeq, "BGR");
#else
		THROW_EXCEPTION("MRPT compiled without OpenCV")
#endif
}


/** Loads the image from an XPM array, as #include'd from a ".xpm" file.
  * \sa loadFromFile
  * \return false on any error */
bool CImage::loadFromXPM( const char** xpm_array, bool swap_rb )
{
#if MRPT_HAS_OPENCV && MRPT_HAS_WXWIDGETS
	try {
		const wxImage b(xpm_array);

		const size_t lx = b.GetWidth();
		const size_t ly = b.GetHeight();

		this->loadFromMemoryBuffer(lx,ly,true,b.GetData(), swap_rb);
		return true;
	}
	catch(std::exception &e)
	{
		std::cerr << "[CImage::loadFromXPM] " << e.what() << std::endl;
		return false;
	}
#else
	MRPT_UNUSED_PARAM(xpm_array); MRPT_UNUSED_PARAM(swap_rb);
	return false;
#endif // MRPT_HAS_OPENCV && MRPT_HAS_WXWIDGETS
}


// Load from TGA files. Used in loadFromFile()
// Contains code from https://github.com/tjohnman/Simple-Targa-Library/blob/master/src/simpleTGA.cpp (FreeBSD license)
bool CImage::loadTGA(const std::string& fileName, mrpt::utils::CImage &out_RGB, mrpt::utils::CImage &out_alpha)
{
#if MRPT_HAS_OPENCV
	std::fstream stream;
	stream.open(fileName.c_str(), std::fstream::in | std::fstream::binary);
	if (!stream.is_open())
	{
		std::cerr << "[CImage::loadTGA] Couldn't open file '"<< fileName <<"'.\n";
		return false;
	}

	stream.seekg(0, std::ios_base::end);
	//long length = stream.tellg();
	stream.seekg(0, std::ios_base::beg);

	// Simple uncompressed true-color image
	char dumpBuffer[12];
	char trueColorHeader[] = "\0\0\2\0\0\0\0\0\0\0\0\0";
	stream.read(dumpBuffer, 12);
	if(memcmp(dumpBuffer, trueColorHeader, 12) != 0)
	{
		std::cerr << "[CImage::loadTGA] Unsupported format or invalid file.\n";
		return false;
	}

	unsigned short width, height;
	unsigned char bpp;

	stream.read((char *)&width, 2);
	stream.read((char *)&height, 2);
	bpp = stream.get();
	if(bpp!=32)
	{
		std::cerr << "[CImage::loadTGA] Only 32 bpp format supported!\n";
		return false;
	}

	unsigned char desc;
	desc = stream.get();
	if(desc!= 8 && desc!=32)
	{
		std::cerr << "[CImage::loadTGA] Unsupported format or invalid file.\n";
		return false;
	}
	const bool origin_is_low_corner = (desc==8);

	// Data section
	std::vector<uint8_t> bytes(width*height*4);
	stream.read((char *)&bytes[0], width*height*4);
	stream.close();

	// Move data to images:
	out_RGB.resize(width,height, CH_RGB, true );
	out_alpha.resize(width,height, CH_GRAY, true );

	size_t idx=0;
	for (unsigned int r=0;r<height;r++)
	{
		unsigned int actual_row = origin_is_low_corner ? (height-1-r) : r;
		IplImage *ipl = ((IplImage*)out_RGB.img);
		unsigned char* data= (unsigned char*) &ipl->imageData[ actual_row * ipl->widthStep ];

		IplImage *ipl_alpha = ((IplImage*)out_alpha.img);
		unsigned char* data_alpha= (unsigned char*)&ipl->imageData[ actual_row * ipl_alpha->widthStep ];

		for (unsigned int c=0;c<width;c++)
		{
			*data++ = bytes[idx++]; // R
			*data++ = bytes[idx++]; // G
			*data++ = bytes[idx++]; // B
			*data_alpha++ = bytes[idx++]; // A
		}
	}

	return true;
#else
	return false;
#endif // MRPT_HAS_OPENCV
}

