/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CImageFloat.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/compress/zip.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/fourier.h>
#include <mrpt/utils/CTicTac.h>

#if MRPT_HAS_OPENCV
	#define CV_NO_CVV_IMAGE   // Avoid CImage name crash

#	if MRPT_OPENCV_VERSION_NUM>=0x211
#		include <opencv2/core/core.hpp>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#		include <opencv2/calib3d/calib3d.hpp>
#	else
#		include <cv.h>
#		include <highgui.h>
#	endif

	#ifdef CImage	// For old OpenCV versions (<=1.0.0)
	#undef CImage
	#endif
#endif

#include <cmath>
#include <stdio.h>


using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;

#if MRPT_HAS_OPENCV
	// For compatibility with OpenCV images, we use the IPL image format as internal
	//  representation.
	#include <highgui.h>
	//#include <cxtypes.h>
#endif // MRPT_HAS_OPENCV

// ---------------------------------------------------------------------------------------
//							START OF JPEG FUNCTIONS PART
// ---------------------------------------------------------------------------------------
/* Expanded data destination object for stdio output */

//#undef INT32
#undef FAR
#define XMD_H

#include <stdio.h>

// In Windows, we HAVE TO (YES dear...) include our custom jpeglib
//  The problem is that, without .so/.dlls, all the libs have their
//  own jpeglib and runtime checks of expected type-sizes fail
//  causing asserts.... (fix: JLBC 20/OCT/2008)
#if MRPT_HAS_JPEG_SYSTEM
	// Normal: System libraries (typ. unix)
	#include <jpeglib.h>

	// Convert mrpt-names to normal ones:
	#define mrpt_jpeg_source_mgr	jpeg_source_mgr

#elif MRPT_HAS_JPEG   // Built-in version
	#include "jpeglib/mrpt_jpeglib.h"
	#define mrpt_jpeg_source_mgr	jpeg_source_mgr
#endif

typedef struct
{
	struct jpeg_destination_mgr pub; /* public fields */

	CStream * out;		/* target stream */
	JOCTET * buffer;		/* start of buffer */
} mrpt_destination_mgr;

typedef mrpt_destination_mgr * mrpt_dest_ptr;

#define OUTPUT_BUF_SIZE  4096	/* choose an efficiently fwrite'able size */

/*
 * Initialize destination --- called by jpeg_start_compress
 * before any data is actually written.
 */

METHODDEF(void)
init_destination (j_compress_ptr cinfo)
{
mrpt_dest_ptr dest = (mrpt_dest_ptr) cinfo->dest;

  /* Allocate the output buffer --- it will be released when done with image */
  dest->buffer = (JOCTET *)
	  (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE,
				  OUTPUT_BUF_SIZE * sizeof(JOCTET));

  dest->pub.next_output_byte = dest->buffer;
  dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;
}


/*
 * Empty the output buffer --- called whenever buffer fills up.
 *
 * In typical applications, this should write the entire output buffer
 * (ignoring the current state of next_output_byte & free_in_buffer),
 * reset the pointer & count to the start of the buffer, and return TRUE
 * indicating that the buffer has been dumped.
 *
 * In applications that need to be able to suspend compression due to output
 * overrun, a FALSE return indicates that the buffer cannot be emptied now.
 * In this situation, the compressor will return to its caller (possibly with
 * an indication that it has not accepted all the supplied scanlines).  The
 * application should resume compression after it has made more room in the
 * output buffer.  Note that there are substantial restrictions on the use of
 * suspension --- see the documentation.
 *
 * When suspending, the compressor will back up to a convenient restart point
 * (typically the start of the current MCU). next_output_byte & free_in_buffer
 * indicate where the restart point will be if the current call returns FALSE.
 * Data beyond this point will be regenerated after resumption, so do not
 * write it out when emptying the buffer externally.
 */

METHODDEF(boolean)
empty_output_buffer (j_compress_ptr cinfo)
{
  mrpt_dest_ptr dest = (mrpt_dest_ptr) cinfo->dest;

  dest->out->WriteBuffer( dest->buffer, OUTPUT_BUF_SIZE);

  dest->pub.next_output_byte = dest->buffer;
  dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;

  return TRUE;
}


/*
 * Terminate destination --- called by jpeg_finish_compress
 * after all data has been written.  Usually needs to flush buffer.
 *
 * NB: *not* called by jpeg_abort or jpeg_destroy; surrounding
 * application must deal with any cleanup that should happen even
 * for error exit.
 */

METHODDEF(void)
term_destination (j_compress_ptr cinfo)
{
  mrpt_dest_ptr dest = (mrpt_dest_ptr) cinfo->dest;
  size_t datacount = OUTPUT_BUF_SIZE - dest->pub.free_in_buffer;

  /* Write any data remaining in the buffer */
  if (datacount > 0)
	dest->out->WriteBuffer( dest->buffer, (int)datacount);

}

GLOBAL(void)
jpeg_stdio_dest (j_compress_ptr cinfo, CStream * out)
{
  mrpt_dest_ptr dest;

  /* The destination object is made permanent so that multiple JPEG images
   * can be written to the same file without re-executing jpeg_stdio_dest.
   * This makes it dangerous to use this manager and a different destination
   * manager serially with the same JPEG object, because their private object
   * sizes may be different.  Caveat programmer.
   */
  if (cinfo->dest == NULL) {	/* first time for this JPEG object? */
	cinfo->dest = (jpeg_destination_mgr *)
		(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				  sizeof(mrpt_destination_mgr));
  }

  dest = (mrpt_dest_ptr) cinfo->dest;
  dest->pub.init_destination = init_destination;
  dest->pub.empty_output_buffer = empty_output_buffer;
  dest->pub.term_destination = term_destination;
  dest->out = out;
}

// -------------------------------------------------------------

/* Expanded data source object for stdio input */

typedef struct
{
  mrpt_jpeg_source_mgr pub;	/* public fields */
  CStream * in;		/* source stream */
  JOCTET * buffer;		/* start of buffer */
  boolean start_of_file;	/* have we gotten any data yet? */
} my_source_mgr;

typedef my_source_mgr * my_src_ptr;

#define INPUT_BUF_SIZE  4096	/* choose an efficiently fread'able size */

/*
 * Initialize source --- called by jpeg_read_header
 * before any data is actually read.
 */

METHODDEF(void)
init_source (j_decompress_ptr cinfo)
{
  my_src_ptr src = (my_src_ptr) cinfo->src;

  /* We reset the empty-input-file flag for each image,
   * but we don't clear the input buffer.
   * This is correct behavior for reading a series of images from one source.
   */
  src->start_of_file = TRUE;
}


/*
 * Fill the input buffer --- called whenever buffer is emptied.
 *
 * In typical applications, this should read fresh data into the buffer
 * (ignoring the current state of next_input_byte & bytes_in_buffer),
 * reset the pointer & count to the start of the buffer, and return TRUE
 * indicating that the buffer has been reloaded.  It is not necessary to
 * fill the buffer entirely, only to obtain at least one more byte.
 *
 * There is no such thing as an EOF return.  If the end of the file has been
 * reached, the routine has a choice of ERREXIT() or inserting fake data into
 * the buffer.  In most cases, generating a warning message and inserting a
 * fake EOI marker is the best course of action --- this will allow the
 * decompressor to output however much of the image is there.  However,
 * the resulting error message is misleading if the real problem is an empty
 * input file, so we handle that case specially.
 *
 * In applications that need to be able to suspend compression due to input
 * not being available yet, a FALSE return indicates that no more data can be
 * obtained right now, but more may be forthcoming later.  In this situation,
 * the decompressor will return to its caller (with an indication of the
 * number of scanlines it has read, if any).  The application should resume
 * decompression after it has loaded more data into the input buffer.  Note
 * that there are substantial restrictions on the use of suspension --- see
 * the documentation.
 *
 * When suspending, the decompressor will back up to a convenient restart point
 * (typically the start of the current MCU). next_input_byte & bytes_in_buffer
 * indicate where the restart point will be if the current call returns FALSE.
 * Data beyond this point must be rescanned after resumption, so move it to
 * the front of the buffer rather than discarding it.
 */

METHODDEF(boolean)
fill_input_buffer (j_decompress_ptr cinfo)
{
  my_src_ptr src = (my_src_ptr) cinfo->src;
  size_t nbytes;

  nbytes = src->in->ReadBuffer( src->buffer, INPUT_BUF_SIZE);

  if (nbytes <= 0)
  {
	if (src->start_of_file)	/* Treat empty input file as fatal error */
	{
		THROW_EXCEPTION("Error looking for JPEG start data!")
	}

	/* Insert a fake EOI marker */
	src->buffer[0] = (JOCTET) 0xFF;
	src->buffer[1] = (JOCTET) JPEG_EOI;
	nbytes = 2;
  }

  src->pub.next_input_byte = src->buffer;
  src->pub.bytes_in_buffer = nbytes;
  src->start_of_file = FALSE;

  return TRUE;
}


/*
 * Skip data --- used to skip over a potentially large amount of
 * uninteresting data (such as an APPn marker).
 *
 * Writers of suspendable-input applications must note that skip_input_data
 * is not granted the right to give a suspension return.  If the skip extends
 * beyond the data currently in the buffer, the buffer can be marked empty so
 * that the next read will cause a fill_input_buffer call that can suspend.
 * Arranging for additional bytes to be discarded before reloading the input
 * buffer is the application writer's problem.
 */

METHODDEF(void)
skip_input_data (j_decompress_ptr cinfo, long num_bytes)
{
  my_src_ptr src = (my_src_ptr) cinfo->src;

  /* Just a dumb implementation for now.  Could use fseek() except
   * it doesn't work on pipes.  Not clear that being smart is worth
   * any trouble anyway --- large skips are infrequent.
   */
  if (num_bytes > 0) {
	while (num_bytes > (long) src->pub.bytes_in_buffer) {
	  num_bytes -= (long) src->pub.bytes_in_buffer;
	  (void) fill_input_buffer(cinfo);
	  /* note we assume that fill_input_buffer will never return FALSE,
	   * so suspension need not be handled.
	   */
	}
	src->pub.next_input_byte += (size_t) num_bytes;
	src->pub.bytes_in_buffer -= (size_t) num_bytes;
  }
}


/*
 * An additional method that can be provided by data source modules is the
 * resync_to_restart method for error recovery in the presence of RST markers.
 * For the moment, this source module just uses the default resync method
 * provided by the JPEG library.  That method assumes that no backtracking
 * is possible.
 */


/*
 * Terminate source --- called by jpeg_finish_decompress
 * after all data has been read.  Often a no-op.
 *
 * NB: *not* called by jpeg_abort or jpeg_destroy; surrounding
 * application must deal with any cleanup that should happen even
 * for error exit.
 */

METHODDEF(void)
term_source (j_decompress_ptr cinfo)
{
	MRPT_UNUSED_PARAM(cinfo);
  /* no work necessary here */
}


/*
 * Prepare for input from a stdio stream.
 * The caller must have already opened the stream, and is responsible
 * for closing it after finishing decompression.
 */

GLOBAL(void)
jpeg_stdio_src (j_decompress_ptr cinfo, CStream * in)
{
  my_src_ptr src;

  /* The source object and input buffer are made permanent so that a series
   * of JPEG images can be read from the same file by calling jpeg_stdio_src
   * only before the first one.  (If we discarded the buffer at the end of
   * one image, we'd likely lose the start of the next one.)
   * This makes it unsafe to use this manager and a different source
   * manager serially with the same JPEG object.  Caveat programmer.
   */
  if (cinfo->src == NULL)
  {	/* first time for this JPEG object? */
	cinfo->src = (mrpt_jpeg_source_mgr *)
		(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				  sizeof(my_source_mgr));
	src = (my_src_ptr) cinfo->src;
	src->buffer = (JOCTET *)
	  (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				  INPUT_BUF_SIZE * sizeof(JOCTET));
  }

  src = (my_src_ptr) cinfo->src;
  src->pub.init_source = init_source;
  src->pub.fill_input_buffer = fill_input_buffer;
  src->pub.skip_input_data = skip_input_data;
  src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
  src->pub.term_source = term_source;
  src->in = in;
  src->pub.bytes_in_buffer = 0; /* forces fill_input_buffer on first read */
  src->pub.next_input_byte = NULL; /* until buffer loaded */
}

// ---------------------------------------------------------------------------------------
//							END OF JPEG FUNCTIONS PART
// ---------------------------------------------------------------------------------------


using namespace mrpt::utils;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CImage, CSerializable, mrpt::utils)


bool CImage::DISABLE_ZIP_COMPRESSION  = false;
std::string CImage::IMAGES_PATH_BASE(".");

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
	MRPT_START;
	changeSize( width, height, nChannels, originTopLeft );
	MRPT_END;
}
/*---------------------------------------------------------------
				Default	Constructor
 ---------------------------------------------------------------*/
CImage::CImage( ) :
		img(NULL),
		m_imgIsReadOnly(false),
		m_imgIsExternalStorage(false)
{
	MRPT_START;
	changeSize( 1, 1, CH_RGB, true );
	MRPT_END;
}

/*---------------------------------------------------------------
						Copy constructor
 ---------------------------------------------------------------*/
CImage::CImage( const CImage &o ) :
	img(NULL),
	m_imgIsReadOnly(false),
	m_imgIsExternalStorage(false)
{
	MRPT_START;
	*this = o;
	MRPT_END;
}

/*---------------------------------------------------------------
						Copy/Transform constructor
 ---------------------------------------------------------------*/
CImage::CImage( const CImageFloat &o ) :
	img(NULL),
	m_imgIsReadOnly(false),
	m_imgIsExternalStorage(false)
{
	MRPT_START;
	*this = o;
	MRPT_END;
}

/*---------------------------------------------------------------
						Copy operator
 ---------------------------------------------------------------*/
CImage& CImage::operator = (const CImage& o)
{
	MRPT_START;

	if (this==&o) return *this;

	releaseIpl();

#if MRPT_HAS_OPENCV
	m_imgIsExternalStorage = o.m_imgIsExternalStorage;
	m_imgIsReadOnly = false;

	if (!o.m_imgIsExternalStorage)
	{ 	// A normal image
		ASSERTMSG_(o.img!=NULL,"Source image in = operator has NULL IplImage*")
		img = cvCloneImage( (IplImage*)o.img );
	}
	else
	{ 	// An externally stored image:
		m_externalFile = o.m_externalFile;
	}
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	return *this;

	MRPT_END;
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
					copyFastFrom
 ---------------------------------------------------------------*/
void CImage::copyFastFrom( CImage &o )
{
	MRPT_START;

	if (this==&o) return;

#if MRPT_HAS_OPENCV
	if (o.m_imgIsExternalStorage)
	{
		// Just copy the reference to the ext. file:
		*this = o;
	}
	else
	{	// Normal copy
		if (!o.img) 		THROW_EXCEPTION("Origin image is empty! (o.img==NULL)")

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

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
}

/*---------------------------------------------------------------
						Copy operator
 ---------------------------------------------------------------*/
CImage& CImage::operator = (const CImageFloat& o)
{
	MRPT_START;

#if MRPT_HAS_OPENCV
	bool	I_was_color;

	if (img)
			I_was_color = isColor();
	else	I_was_color = false;

	releaseIpl();
	m_imgIsExternalStorage=false;
	m_imgIsReadOnly=false;


	unsigned int nRows = o.m_height;
	unsigned int nCols = o.m_width;

	// New image:
	changeSize( nCols, nRows, I_was_color? 3:1, true );

	// Copy contents:
	if (!isColor())
	{
		for (unsigned int row=0;row<nRows;row++)
		{
			unsigned char	*ptrDest = (unsigned char*) & ((IplImage*)img)->imageData[ row * ((IplImage*)img)->widthStep ];
			float			*ptrSrc  = o.m_img + row * o.m_width;

			for (unsigned int col=0;col<nCols;col++)
			{
				short  s = (short)(255*(*ptrSrc++));
				(*ptrDest++) = s<0 ? ((unsigned char)(255+s)):((unsigned char)s);
			}
		}
	}
	else
	{
		for (unsigned int row=0;row<nRows;row++)
		{
			unsigned char	*ptrDest = (unsigned char*) &((IplImage*)img)->imageData[ row * ((IplImage*)img)->widthStep ];
			float			*ptrSrc  = o.m_img + row * o.m_width;

			for (unsigned int col=0;col<nCols;col++)
			{
				short  s = (short)(255*(*ptrSrc++));
				unsigned char c = s<0 ? ((unsigned char)(255+s)):((unsigned char)s);
				(*ptrDest++) = c;
				(*ptrDest++) = c;
				(*ptrDest++) = c;
			}
		}
	}
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	return *this;
	MRPT_END;
}

/*---------------------------------------------------------------
						Constructor from IplImage
 ---------------------------------------------------------------*/
CImage::CImage( void *iplImage ) :
	m_imgIsReadOnly(false),
	m_imgIsExternalStorage(false)
{
	MRPT_START;

	img = NULL; // Initialize

#if MRPT_HAS_OPENCV
	if (!iplImage)
	{
        changeSize( 1, 1, 1, true );
	}
	else
	{
		img = cvCloneImage( (IplImage*) iplImage );
	}
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
}

/*---------------------------------------------------------------
						Destructor
 ---------------------------------------------------------------*/
CImage::~CImage( )
{
	MRPT_START;
	releaseIpl();
	MRPT_END;
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
	MRPT_START;

	releaseIpl();

#if MRPT_HAS_OPENCV
    img = cvCreateImage( cvSize(width,height),IPL_DEPTH_8U, nChannels );
	((IplImage*)img)->origin = originTopLeft ? 0:1;
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
}

/*---------------------------------------------------------------
						loadFromFile
 ---------------------------------------------------------------*/
bool  CImage::loadFromFile( const std::string& fileName, int isColor )
{
    MRPT_START;

	releaseIpl();

#if MRPT_HAS_OPENCV
    return (NULL!= (img=cvLoadImage(fileName.c_str(),isColor) ));
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
    MRPT_END;
}

/*---------------------------------------------------------------
						saveToFile
 ---------------------------------------------------------------*/
bool  CImage::saveToFile( const std::string& fileName, int jpeg_quality ) const
{
    MRPT_START;
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
    MRPT_END;
}

/*---------------------------------------------------------------
						loadFromIplImage
 ---------------------------------------------------------------*/
void  CImage::loadFromIplImage( void* iplImage )
{
	MRPT_START;
	ASSERT_(iplImage!=NULL)

#if MRPT_HAS_OPENCV
	releaseIpl();

	img = cvCloneImage( (IplImage*)iplImage );
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
}

/*---------------------------------------------------------------
						setFromIplImageReadOnly
 ---------------------------------------------------------------*/
void  CImage::setFromIplImageReadOnly( void* iplImage )
{
	MRPT_START;

#if MRPT_HAS_OPENCV
	releaseIpl();
	img = (IplImage*)iplImage;
	m_imgIsReadOnly = true;
	m_imgIsExternalStorage=false;
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
}

/*---------------------------------------------------------------
						setFromIplImage
 ---------------------------------------------------------------*/
void  CImage::setFromIplImage( void* iplImage )
{
	MRPT_START;

#if MRPT_HAS_OPENCV
	releaseIpl();
	img = (IplImage*)iplImage;
	m_imgIsReadOnly = false;
	m_imgIsExternalStorage=false;
#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
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
	MRPT_START;

#if MRPT_HAS_OPENCV
	resize(width,height,color ? 3:1, true);

	m_imgIsReadOnly = false;
	m_imgIsExternalStorage=false;

	if ( ((IplImage*)img)->widthStep == ((IplImage*)img)->width)
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

	if (swapRedBlue)
	{
		swapRB();
	}

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
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
	MRPT_START;
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
	MRPT_END;
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
						getAsIplImage
 ---------------------------------------------------------------*/
void *  CImage::getAsIplImage() const
{
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	return img;
}



/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CImage::writeToStream(CStream &out, int *version) const
{
#if MRPT_HAS_OPENCV
	if (version)
		*version = 7;
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

			bool	hasColor = isColor();

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
				int32_t width = ((IplImage*)img)->width;
				int32_t height = ((IplImage*)img)->height;

				out << width << height;

				if (width>=1 && height>=1)
				{
					// Save to temporary memory stream:
					CMemoryStream		aux;
					uint32_t			nBytes;

					saveToStreamAsJPEG( aux );
					nBytes = (uint32_t)aux.getTotalBytesCount();

					out << nBytes;
					out.WriteBuffer( aux.getRawBufferData(), nBytes );
				}
				else
				{
					// No image to save!
				}
			}
		} // end m_imgIsExternalStorage=false
	}
#endif
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CImage::readFromStream(CStream &in, int version)
{
#if !MRPT_HAS_OPENCV
	THROW_EXCEPTION("[CImage] Cannot deserialize image since MRPT has been compiled without OpenCV")
#else

	releaseIpl();  // First, free current image.

	switch(version)
	{
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
						uint32_t		width, height;
						in >> width >> height;
						loadJPEG = (width>=1 && height>=1);
						if (!loadJPEG)
							changeSize(width,height,3,true);
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

/*---------------------------------------------------------------
						grayscale
 ---------------------------------------------------------------*/
void CImage::grayscale( CImage  &ret ) const
{
#if MRPT_HAS_OPENCV
	// The image is already grayscale??
	if (!isColor())
	{
		ret = *this;
		return;
	}
	else
	{
		// Convert to a single luminance channel image
		IplImage		*ipl = ((IplImage*)img);
		ASSERT_(ipl);

		int	 cx = getWidth(), cy = getHeight();
		ret.changeSize(cx,cy,1,isOriginTopLeft());

		cvCvtColor( ipl, (IplImage*)ret.img, CV_BGR2GRAY );
	}
#endif
}


/*---------------------------------------------------------------
						scaleHalf
 ---------------------------------------------------------------*/
CImage  CImage::scaleHalf()const
{
	CImage ret = *this;
	const TImageSize siz = this->getSize();
	ret.scaleImage(siz.x/2,siz.y/2);
	return ret;
}

/*---------------------------------------------------------------
						scaleDouble
 ---------------------------------------------------------------*/
CImage  CImage::scaleDouble()const
{
	CImage ret = *this;
	const TImageSize siz = this->getSize();
	ret.scaleImage(siz.x*2,siz.y*2);
	return ret;
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
	MRPT_START;

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

	MRPT_END;
#endif
}

/*---------------------------------------------------------------
					saveToStreamAsJPEG
 ---------------------------------------------------------------*/
void  CImage::saveToStreamAsJPEG( CStream		&out  )const
{
#if MRPT_HAS_OPENCV
	MRPT_START;

	makeSureImageIsLoaded();   // For delayed loaded images stored externally

	struct jpeg_compress_struct		cinfo;
	struct jpeg_error_mgr			jerr;
	unsigned int					nCols = getWidth();
	unsigned int					nRows = getHeight();
//	int								row_stride;			/* physical row width in buffer */
	IplImage						*ipl = ((IplImage*)img);

	// Some previous verification:
	ASSERT_(nCols>=1 && nRows>=1);
	ASSERT_(ipl);
	ASSERT_(ipl->nChannels == 1 || ipl->nChannels == 3);

	// 1) Initialization of the JPEG compresion object:
	// --------------------------------------------------
	cinfo.err = /*mrpt::utils::jpeglib::*/ jpeg_std_error(&jerr);
	/*mrpt::utils::jpeglib::*/ jpeg_create_compress(&cinfo);

	// 2) Set the destination of jpeg data:
	// --------------------------------------------------
	/*mrpt::utils::jpeglib::*/ jpeg_stdio_dest( &cinfo, &out );

	// 3) Set parameters for compression:
	// --------------------------------------------------
	cinfo.image_width = nCols;
	cinfo.image_height = nRows;
	cinfo.input_components = isColor() ? 3:1;
	cinfo.in_color_space = isColor() ? JCS_RGB : JCS_GRAYSCALE;

	/*mrpt::utils::jpeglib::*/ jpeg_set_defaults(&cinfo);
	/* Make optional parameter settings here */
	/* Now you can set any non-default parameters you wish to.
	* Here we just illustrate the use of quality (quantization table) scaling:
	*/
	/*mrpt::utils::jpeglib::*/ jpeg_set_quality(&cinfo, 95 /* quality per cent */, TRUE /* limit to baseline-JPEG values */);

	// 4) Start:
	// --------------------------------------------------
	/*mrpt::utils::jpeglib::*/ jpeg_start_compress(&cinfo, TRUE);

	// 5) Write scan lines:
	// --------------------------------------------------
//	row_stride = nCols * (isColor() ? 3:1);	/* JSAMPLEs per row in image_buffer */

	unsigned int col;

	if (isColor())
	{
		JSAMPROW						row_pointer[1];		/* pointer to a single row */
		row_pointer[0] = (JSAMPROW)new char[ ipl->widthStep ];

		for (unsigned int row = 0; row<nRows;row++)
		{
			// Flip RGB bytes order!
			char *src;
			if (ipl->origin == 0)
					src = &ipl->imageData[ row * ipl->widthStep ];
			else	src = &ipl->imageData[ (nRows-1-row) * ipl->widthStep ];
			char *target = (char *)row_pointer[0];
			for (col=0;col<nCols;col++)
			{
				target[0] = src[2];
				target[1] = src[1];
				target[2] = src[0];

				target+=3;
				src+=3;
			}

			if (1!= /*mrpt::utils::jpeglib::*/ jpeg_write_scanlines(&cinfo, row_pointer, 1))
			{
				THROW_EXCEPTION("jpeg_write_scanlines: didn't work!!");
			}
		}

		delete[] row_pointer[0];
	} // end "color"
	else
	{	// Is grayscale:
		JSAMPROW						row_pointer[1];		/* pointer to a single row */

		for (unsigned int row = 0; row<nRows;row++)
		{
			if (ipl->origin == 0)
					row_pointer[0] = (JSAMPROW) &ipl->imageData[ row * ipl->widthStep ];
			else	row_pointer[0] = (JSAMPROW) &ipl->imageData[ (nRows-1-row) * ipl->widthStep ];

			// Gray scale:
			if (1!= /*mrpt::utils::jpeglib::*/ jpeg_write_scanlines(&cinfo, row_pointer, 1))
			{
				THROW_EXCEPTION("jpeg_write_scanlines: didn't work!!");
			}
		}
	}


	// 6) Compress and finish:
	// --------------------------------------------------
	/*mrpt::utils::jpeglib::*/ jpeg_finish_compress(&cinfo);
	/*mrpt::utils::jpeglib::*/ jpeg_destroy_compress(&cinfo);

	// DONE!
	MRPT_END;
#endif
}


/*---------------------------------------------------------------
					saveToStreamAsJPEG
 ---------------------------------------------------------------*/
void  CImage::loadFromStreamAsJPEG( CStream &in )
{
#if MRPT_HAS_OPENCV
	MRPT_START;

	struct jpeg_decompress_struct		cinfo;
	struct jpeg_error_mgr				jerr;
	JSAMPARRAY							buffer;		/* Output row buffer */
	int									row_stride;		/* physical row width in output buffer */

	/* Step 1: allocate and initialize JPEG decompression object */

	/* We set up the normal JPEG error routines, then override error_exit. */
	cinfo.err = /*mrpt::utils::jpeglib::*/ jpeg_std_error(&jerr);

	/* Now we can initialize the JPEG decompression object. */
	/*mrpt::utils::jpeglib::*/ jpeg_create_decompress(&cinfo);

	/* Step 2: specify data source (eg, a file) */
	/*mrpt::utils::jpeglib::*/ jpeg_stdio_src(&cinfo, &in);

	/* Step 3: read file parameters with jpeg_read_header() */
	/*mrpt::utils::jpeglib::*/ jpeg_read_header(&cinfo, TRUE);

	/* Step 4: set parameters for decompression */

	/* Step 5: Start decompressor */
	/*mrpt::utils::jpeglib::*/ jpeg_start_decompress(&cinfo);

	/* We may need to do some setup of our own at this point before reading
	* the data.  After jpeg_start_decompress() we have the correct scaled
	* output image dimensions available, as well as the output colormap
	* if we asked for color quantization.
	* In this example, we need to make an output work buffer of the right size.
	*/
	/* JSAMPLEs per row in output buffer */
	row_stride = cinfo.output_width * cinfo.output_components;
	/* Make a one-row-high sample array that will go away when done with image */
	buffer = (*cinfo.mem->alloc_sarray)
		((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);


	// Resize the CImage now:
	this->changeSize( cinfo.output_width, cinfo.output_height, cinfo.out_color_components, true );
	IplImage		*ipl = ((IplImage*)img);

	/* Step 6: while (scan lines remain to be read) */
	/*           jpeg_read_scanlines(...); */

	/* Here we use the library's state variable cinfo.output_scanline as the
	* loop counter, so that we don't have to keep track ourselves.
	*/
	unsigned int	nCols = cinfo.output_width;
	unsigned int	nRows = cinfo.output_height;
	unsigned int	col, row;

	for (row = 0; row<nRows;row++)
	{
		/* jpeg_read_scanlines expects an array of pointers to scanlines.
		* Here the array is only one element long, but you could ask for
		* more than one scanline at a time if that's more convenient.
		*/
		/*mrpt::utils::jpeglib::*/ jpeg_read_scanlines(&cinfo, buffer, 1);

		/* Copy into the CImage object */
		if (isColor())
		{
			// Flip RGB bytes order!
			char *target = &ipl->imageData[ row * ipl->widthStep ];
			char *src = (char *)buffer[0];
			for (col=0;col<nCols;col++)
			{
				target[0] = src[2];
				target[1] = src[1];
				target[2] = src[0];

				target+=3;
				src+=3;
			}
		}
		else
		{
			// Gray scale:
			memcpy( &ipl->imageData[ row * ipl->widthStep ],
					buffer[0],
					row_stride );
		}

	}

	/* Step 7: Finish decompression */

	(void) jpeg_finish_decompress(&cinfo);
	/* We can ignore the return value since suspension is not possible
	* with the stdio data source.
	*/

	/* Step 8: Release JPEG decompression object */

	/* This is an important step since it will release a good deal of memory. */
	jpeg_destroy_decompress(&cinfo);

	// DONE!
	MRPT_END;
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
	MRPT_START;
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
	MRPT_END;
#endif

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

	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_START;
	#endif

	makeSureImageIsLoaded();   // For delayed loaded images stored externally

	IplImage *ipl = ((IplImage*)img);

	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	ASSERT_(ipl);
	#endif

	cvCircle( ipl, cvPoint(x,y), radius, CV_RGB (color.R,color.G,color.B), width  );

	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
	MRPT_END;
	#endif
#else
	THROW_EXCEPTION("Method not available since MRPT has been compiled without OpenCV")
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
	if(row_+ipl_ext->width > getWidth() || col_+ipl_ext->height > getHeight())
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

	IplImage *im, *patch_im;

	if( this->isColor() && patch_img.isColor() )
	{
		IplImage *im_ = (IplImage*)this->getAsIplImage();
		IplImage *patch_im_ = (IplImage*)patch_img.getAsIplImage();

		im = cvCreateImage( cvGetSize( im_ ), 8, 1 );
		patch_im = cvCreateImage( cvGetSize( patch_im_ ), 8, 1 );

		cvCvtColor( im_, im, CV_BGR2GRAY );
		cvCvtColor( patch_im_, patch_im, CV_BGR2GRAY );
	}
	else
	{
		im = (IplImage*)this->getAsIplImage();
		patch_im = (IplImage*)patch_img.getAsIplImage();
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

	IplImage *ipl_ext;

	if (!entireImg)
	{
		ipl_ext = cvCreateImage(cvSize(patch_im->width+x_search_size,patch_im->height+y_search_size),IPL_DEPTH_8U, 1);
		for (unsigned int i = 0 ; i < (unsigned int)y_search_size ; i++)
		{
			memcpy( &ipl_ext->imageData[i * ipl_ext->widthStep ],
					&im->imageData[(i+y_search_ini) * im->widthStep + x_search_ini * im->nChannels],
					ipl_ext->width * ipl_ext->nChannels ); //widthStep);  <-- JLBC: widthstep SHOULD NOT be used as the length of each row (the last one may be shorter!!)
		}
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
	if (!entireImg) cvReleaseImage( &ipl_ext );

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
	IplImage *ipl = ((IplImage*)img);
	ASSERT_(ipl);

	int			i;
	if (ipl->nChannels!=1)
		THROW_EXCEPTION("CImage Error using normalize, wrong number of channel");

	unsigned char val,min_=255,max_=1;
	for (i=0;i<ipl->height;i++)
	{
		for (int j=0;j<ipl->width;j++)
		{
			val = ipl->imageData[i*ipl->widthStep + j];
			if (min_ > val) min_=val;
			if (max_ < val) max_=val;
		}
	}

	double esc=255.0/((double)max_-(double)min_);
	for (i=0;i<ipl->height;i++)
	{
		for (int j=0;j<ipl->width;j++)
		{
			ipl->imageData[i*ipl->widthStep + j]=(unsigned char) (esc*(ipl->imageData[i*ipl->widthStep + j]-min_));
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
	MRPT_START;

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

	MRPT_END;
#endif
}

/*---------------------------------------------------------------
						setFromMatrix
 ---------------------------------------------------------------*/
void CImage::setFromMatrix(const mrpt::math::CMatrixDouble &m, bool matrix_is_normalized)
{
#if MRPT_HAS_OPENCV
	MRPT_START
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);

	const size_t lx = m.getColCount();
	const size_t ly = m.getRowCount();
	this->changeSize(lx,ly,1,true);

	if (matrix_is_normalized)
	{  // Matrix: [0,1]
		for (size_t y=0;y<ly;y++)
		{
			unsigned char	*pixels = this->get_unsafe(0,y,0);
			for (size_t x=0;x<lx;x++)
				(*pixels++) = static_cast<unsigned char>( m.get_unsafe(y,x) * 255  );
		}
	}
	else
	{  // Matrix: [0,255]
		for (size_t y=0;y<ly;y++)
		{
			unsigned char	*pixels = this->get_unsafe(0,y,0);
			for (size_t x=0;x<lx;x++)
				(*pixels++) = static_cast<unsigned char>( m.get_unsafe(y,x) );
		}
	}
	MRPT_END
#endif
}

/*---------------------------------------------------------------
						setFromMatrix
 ---------------------------------------------------------------*/
void CImage::setFromMatrix(const mrpt::math::CMatrixFloat &m, bool matrix_is_normalized)
{
#if MRPT_HAS_OPENCV
	MRPT_START
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
	ASSERT_(img);

	const size_t lx = m.getColCount();
	const size_t ly = m.getRowCount();
	this->changeSize(lx,ly,1,true);

	if (matrix_is_normalized)
	{  // Matrix: [0,1]
		for (size_t y=0;y<ly;y++)
		{
			unsigned char	*pixels = this->get_unsafe(0,y,0);
			for (size_t x=0;x<lx;x++)
				(*pixels++) = static_cast<unsigned char>( m.get_unsafe(y,x) * 255 );
		}
	}
	else
	{  // Matrix: [0,255]
		for (size_t y=0;y<ly;y++)
		{
			unsigned char	*pixels = this->get_unsafe(0,y,0);
			for (size_t x=0;x<lx;x++)
				(*pixels++) = static_cast<unsigned char>( m.get_unsafe(y,x) );
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
	MRPT_START;

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
	i2 -= biasThisImg;
	i1 -= biasInImg;

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

			float	i1 = I1_I.get_unsafe(y,x);
			float	i2 = I2_I.get_unsafe(y,x);

			float	den = square(r1)+square(i1);
			I2_R.set_unsafe(y,x, (r1*r2+i1*i2)/den);
			I2_I.set_unsafe(y,x, (i2*r1-r2*i1)/den);
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

	MRPT_END;
}


/*---------------------------------------------------------------
						getAsMatrixTiled
 ---------------------------------------------------------------*/
void  CImage::getAsMatrixTiled( CMatrix &outMatrix )  const
{
#if MRPT_HAS_OPENCV
	MRPT_START;

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

	MRPT_END;
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
void CImage::unload() MRPT_NO_THROWS
{
	if (m_imgIsExternalStorage)
		releaseIpl( true ); // Do NOT mark the image as NON external
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
void CImage::rectifyImageInPlace( const mrpt::utils::TCamera &cameraParams  )
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	// MRPT -> OpenCV Input Transformation
	IplImage *srcImg = static_cast<IplImage *>( getAsIplImage() );	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	double aux1[3][3], aux2[1][4];
	const CMatrixDouble33 &cameraMatrix = cameraParams.intrinsicParams;

	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			aux1[i][j] = cameraMatrix(i,j);
	for (int i=0;i<4;i++)
		aux2[0][i]=cameraParams.dist[i];

	CvMat inMat =  cvMat( cameraMatrix.getRowCount(), cameraMatrix.getColCount(), CV_64F, aux1 );
	CvMat distM =  cvMat( 1, 4, CV_64F, aux2 );

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
	IplImage *srcImg = static_cast<IplImage *>( getAsIplImage() );	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	double aux1[3][3], aux2[1][4];
	const CMatrixDouble33 &cameraMatrix = cameraParams.intrinsicParams;

	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			aux1[i][j] = cameraMatrix(i,j);
	for (int i=0;i<4;i++)
		aux2[0][i]=cameraParams.dist[i];

	CvMat inMat =  cvMat( cameraMatrix.getRowCount(), cameraMatrix.getColCount(), CV_64F, aux1 );
	CvMat distM =  cvMat( 1, 4, CV_64F, aux2 );

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
	IplImage *srcImg = static_cast<IplImage *>( getAsIplImage() );	// Source Image
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
	IplImage *srcImg = static_cast<IplImage *>( getAsIplImage() );	// Source Image
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
	IplImage *srcImg = static_cast<IplImage *>( getAsIplImage() );	// Source Image
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
	IplImage *srcImg = static_cast<IplImage *>( getAsIplImage() );	// Source Image
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
	IplImage *srcImg = static_cast<IplImage*>( getAsIplImage() );	// Source Image

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
	IplImage *srcImg = static_cast<IplImage*>( getAsIplImage() );	// Source Image

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
                        grayscaleInPlace
 ---------------------------------------------------------------*/
void CImage::grayscaleInPlace()
{
#if MRPT_HAS_OPENCV
	makeSureImageIsLoaded();   // For delayed loaded images stored externally
    ASSERT_(img!=NULL);
	if (!this->isColor()) return;

	IplImage *srcImg = static_cast<IplImage*>( getAsIplImage() );	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, 1 );

	cvCvtColor( srcImg, outImg, CV_BGR2GRAY );

	// Resize:
	outImg->origin = srcImg->origin;

	// Assign the output image to the IPLImage pointer within the CImage
	releaseIpl();
	img = outImg;
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

	IplImage *srcImg = static_cast<IplImage*>( getAsIplImage() );	// Source Image
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

// Declaration of auxiliary functions in checkerboard_ocamcalib_detector.cpp
#if MRPT_HAS_OPENCV
	// Return: -1: errors, 0: not found, 1: found OK
	int cvFindChessboardCorners3( const void* arr, CvSize pattern_size, CvPoint2D32f* out_corners, int* out_corner_count);
#endif


/** Look for the corners of a chessboard in the image
  * \param cornerCoords [OUT] The pixel coordinates of all the corners.
  * \param check_size_x [IN] The number of squares, in the X direction
  * \param check_size_y [IN] The number of squares, in the Y direction
  * \param normalize_image [IN] Whether to normalize the image before detection
  * \param useScaramuzzaMethod [IN] Whether to use the alternative, more robust method by M. Rufli, D. Scaramuzza, and R. Siegwart.
  *
  * \return true on success
  */
bool CImage::findChessboardCorners(
	std::vector<TPixelCoordf> 	&cornerCoords,
	unsigned int  check_size_x,
	unsigned int  check_size_y,
	bool		normalize_image,
	bool		useScaramuzzaMethod) const
{
#if MRPT_HAS_OPENCV
	MRPT_START

	ASSERT_(check_size_y>0 && check_size_x>0)

	// Grayscale version:
	CImage img(UNINITIALIZED_IMAGE);
	if (this->isColor())
			this->grayscale(img);
	else	img.setFromImageReadOnly(*this);

	// Try with expanded versions of the image if it fails to detect the checkerboard:
	int corners_count;
	bool corners_found=false;

	const CvSize check_size = cvSize(check_size_x, check_size_y);

	const size_t CORNERS_COUNT = check_size_x * check_size_y;

	vector<CvPoint2D32f> corners_list;
	corners_count = CORNERS_COUNT;
	corners_list.resize( CORNERS_COUNT );

	cornerCoords.clear();

	int find_chess_flags = CV_CALIB_CB_ADAPTIVE_THRESH;
	if (normalize_image)
		find_chess_flags |= CV_CALIB_CB_NORMALIZE_IMAGE;

	if (!useScaramuzzaMethod)
	{
		// Standard OpenCV's function:
		corners_found = 0 != cvFindChessboardCorners(
			static_cast<IplImage*>(img.getAsIplImage()),
			check_size,
			&corners_list[0],
			&corners_count,
			find_chess_flags);
	}
	else
	{
		int found_corner_count=0;
		// Return: -1: errors, 0: not found, 1: found OK
		corners_found = 1 == cvFindChessboardCorners3( 
			static_cast<IplImage*>(img.getAsIplImage()), 
			check_size, 
			&corners_list[0],
			&corners_count
			);
	}

	// Check # of corners:
	if (corners_found && corners_count!=CORNERS_COUNT) 
		corners_found=false;

	if( corners_found )
	{
		// Refine corners:
		cvFindCornerSubPix(
			static_cast<IplImage*>(img.getAsIplImage()),
			&corners_list[0],
			corners_count,
			cvSize(5,5), 	// window
			cvSize(-1,-1),
			cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.01f ));

		// save the corners in the data structure:
		int y;
		unsigned int k;
		for( y = 0, k = 0; y < check_size.height; y++ )
			for( int x = 0; x < check_size.width; x++, k++ )
				cornerCoords.push_back(  TPixelCoordf( corners_list[k].x, corners_list[k].y ) );
	}


	return corners_found;

	MRPT_END
#else
	return false;
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
	unsigned int  check_size_y )
{
#if MRPT_HAS_OPENCV

	if (cornerCoords.size()!=check_size_x*check_size_y) return false;

	IplImage* img = static_cast<IplImage*>(getAsIplImage());

    const int r = 4;

	unsigned int x, y,i;
	CvPoint prev_pt = {0, 0};
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

	for( y = 0, i = 0; y < check_size_y; y++ )
	{
		CvScalar color = line_colors[y % line_max];
		for( x = 0; x < check_size_x; x++, i++ )
		{
			CvPoint pt;
			pt.x = cvRound( cornerCoords[i].x);
			pt.y = cvRound( cornerCoords[i].y);

			if( i != 0 ) cvLine( img, prev_pt, pt, color );

			cvLine( img,
					  cvPoint( pt.x - r, pt.y - r ),
					  cvPoint( pt.x + r, pt.y + r ), color );
			cvLine( img,
					  cvPoint( pt.x - r, pt.y + r),
					  cvPoint( pt.x + r, pt.y - r), color );
			cvCircle( img, pt, r+1, color );
			prev_pt = pt;
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

	IplImage *srcImg = static_cast<IplImage*>( getAsIplImage() );	// Source Image
	IplImage *outImg;												// Output Image
	outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, 3 );

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

	IplImage *srcImg = static_cast<IplImage*>( getAsIplImage() );	// Source Image
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

	IplImage* _im1 = static_cast<IplImage*>( im1.getAsIplImage() );
	IplImage* _im2 = static_cast<IplImage*>( im2.getAsIplImage() );

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
	IplImage *ipl = static_cast<IplImage*>( getAsIplImage() );	// Source Image
    ASSERT_(ipl!=NULL);

	int	 cx = getWidth(), cy = getHeight();
	outImg.changeSize(cx,cy,1,isOriginTopLeft());

	cvEqualizeHist(ipl, (IplImage*)outImg.img );
#endif
}

/*---------------------------------------------------------------
						equalizeHistInPlace
 ---------------------------------------------------------------*/
void CImage::equalizeHistInPlace()
{
#if MRPT_HAS_OPENCV
	// Convert to a single luminance channel image
	IplImage *srcImg = static_cast<IplImage*>( getAsIplImage() );	// Source Image
    ASSERT_(srcImg!=NULL);

	IplImage *outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );

	cvEqualizeHist(srcImg, outImg );

	outImg->origin = srcImg->origin;

	// Assign the output image to the IPLImage pointer within the CImage
	releaseIpl();
	img = outImg;

#endif
}


/** Look for the corners of one or more chessboard/checkerboards in the image.
  *  This method uses an improved version of OpenCV's cvFindChessboardCorners published
  *   by M. Rufli, D. Scaramuzza, and R. Siegwart.
  *  That method has been extended in this MRPT implementation to automatically detect a
  *   number of different checkerboards in the same image.
  *
  * \param cornerCoords [OUT] A vector of N vectors of pixel coordinates, for each of the N chessboards detected.
  * \param check_size_x [IN] The number of squares, in the X direction
  * \param check_size_y [IN] The number of squares, in the Y direction
  *
  *
  * \sa mrpt::vision::checkerBoardCameraCalibration, drawChessboardCorners
  */
void CImage::findMultipleChessboardsCorners(
	std::vector<std::vector<TPixelCoordf> > 	&cornerCoords,
	unsigned int  check_size_x,
	unsigned int  check_size_y ) const
{
#if MRPT_HAS_OPENCV
	// Grayscale version:
	CImage img(UNINITIALIZED_IMAGE);
	if (this->isColor())
			this->grayscale(img);
	else	img.setFromImageReadOnly(*this);

	std::vector<CvPoint2D32f>  corners_list;
	corners_list.resize(check_size_x*check_size_y*10);

	int found_corner_count=0;
	// Return: -1: errors, 0: not found, 1: found OK
	bool corners_found = 1==cvFindChessboardCorners3( 
		static_cast<IplImage*>(img.getAsIplImage()), 
		cvSize(check_size_x,check_size_y), 
		&corners_list[0], 
		&found_corner_count);

	// Check # of corners:
	if (corners_found && found_corner_count!=check_size_x*check_size_y) 
		corners_found=false;

	if( corners_found )
	{
		// Refine corners:
		cvFindCornerSubPix(
			static_cast<IplImage*>(img.getAsIplImage()),
			&corners_list[0],
			found_corner_count,
			cvSize(5,5), 	// window
			cvSize(-1,-1),
			cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.01f ));

		// save the corners in the data structure:
		cornerCoords.resize(1);
		for( unsigned int y = 0, k = 0; y < check_size_y; y++ )
			for( unsigned int x = 0; x < check_size_x; x++, k++ )
				cornerCoords[0].push_back(  TPixelCoordf( corners_list[k].x, corners_list[k].y ) );
	}
	else
	{	// Not found.
		cornerCoords.clear();
		return;
	}

#endif
}


