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
#include <mrpt/utils/CStream.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt;
using namespace mrpt::utils;

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



/*---------------------------------------------------------------
					saveToStreamAsJPEG
 ---------------------------------------------------------------*/
void  CImage::saveToStreamAsJPEG( CStream &out, const int jpeg_quality ) const
{
#if MRPT_HAS_OPENCV
	MRPT_START

	makeSureImageIsLoaded();   // For delayed loaded images stored externally

	struct jpeg_compress_struct		cinfo;
	struct jpeg_error_mgr			jerr;

	const IplImage *ipl = static_cast<const IplImage*>(img);

	const unsigned int nCols = ipl->width;
	const unsigned int nRows = ipl->height;
	const bool is_color = (ipl->nChannels==3);


	// Some previous verification:
	ASSERT_(nCols>=1 && nRows>=1)
	ASSERT_(ipl)
	ASSERT_(ipl->nChannels == 1 || ipl->nChannels == 3)

	// 1) Initialization of the JPEG compresion object:
	// --------------------------------------------------
	cinfo.err =  jpeg_std_error(&jerr);
	 jpeg_create_compress(&cinfo);

	// 2) Set the destination of jpeg data:
	// --------------------------------------------------
	jpeg_stdio_dest( &cinfo, &out );

	// 3) Set parameters for compression:
	// --------------------------------------------------
	cinfo.image_width = nCols;
	cinfo.image_height = nRows;
	cinfo.input_components = is_color ? 3:1;
	cinfo.in_color_space = is_color ? JCS_RGB : JCS_GRAYSCALE;

	jpeg_set_defaults(&cinfo);
	/* Make optional parameter settings here */
	/* Now you can set any non-default parameters you wish to.
	* Here we just illustrate the use of quality (quantization table) scaling:
	*/
	jpeg_set_quality(&cinfo, jpeg_quality /* quality per cent */, TRUE /* limit to baseline-JPEG values */);

	// 4) Start:
	// --------------------------------------------------
	jpeg_start_compress(&cinfo, TRUE);

	// 5) Write scan lines:
	// --------------------------------------------------
	if (is_color)
	{
		JSAMPROW row_pointer[1];		/* pointer to a single row */
		row_pointer[0] = (JSAMPROW)new char[ ipl->widthStep ];

		for (unsigned int row = 0; row<nRows;row++)
		{
			// Flip RGB bytes order!
			char *src;
			if (ipl->origin == 0)
					src = &ipl->imageData[ row * ipl->widthStep ];
			else	src = &ipl->imageData[ (nRows-1-row) * ipl->widthStep ];
			char *target = (char *)row_pointer[0];
			for (unsigned int col=0;col<nCols;col++)
			{
				target[0] = src[2];
				target[1] = src[1];
				target[2] = src[0];

				target+=3;
				src+=3;
			}

			if (1!=  jpeg_write_scanlines(&cinfo, row_pointer, 1))
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
			if (1!=  jpeg_write_scanlines(&cinfo, row_pointer, 1))
			{
				THROW_EXCEPTION("jpeg_write_scanlines: didn't work!!");
			}
		}
	}

	// 6) Compress and finish:
	// --------------------------------------------------
	jpeg_finish_compress(&cinfo);
	jpeg_destroy_compress(&cinfo);

	// DONE!
	MRPT_END
#endif
}


/*---------------------------------------------------------------
					saveToStreamAsJPEG
 ---------------------------------------------------------------*/
void  CImage::loadFromStreamAsJPEG( CStream &in )
{
#if MRPT_HAS_OPENCV
	MRPT_START

	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr         jerr;

	/* Step 1: allocate and initialize JPEG decompression object */

	/* We set up the normal JPEG error routines, then override error_exit. */
	cinfo.err =  jpeg_std_error(&jerr);

	/* Now we can initialize the JPEG decompression object. */
	jpeg_create_decompress(&cinfo);

	/* Step 2: specify data source (eg, a file) */
	jpeg_stdio_src(&cinfo, &in);

	/* Step 3: read file parameters with jpeg_read_header() */
	jpeg_read_header(&cinfo, TRUE);

	/* Step 4: set parameters for decompression */

	/* Step 5: Start decompressor */
	jpeg_start_decompress(&cinfo);

	/* We may need to do some setup of our own at this point before reading
	* the data.  After jpeg_start_decompress() we have the correct scaled
	* output image dimensions available, as well as the output colormap
	* if we asked for color quantization.
	* In this example, we need to make an output work buffer of the right size.
	*/
	/* JSAMPLEs per row in output buffer */
	/* physical row width in output buffer */
	const int row_stride = cinfo.output_width * cinfo.output_components;
	/* Make a one-row-high sample array that will go away when done with image */
	/* Output row buffer */
	JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)
		((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);


	// Resize the CImage now:
	this->changeSize( cinfo.output_width, cinfo.output_height, cinfo.out_color_components, true );
	IplImage *ipl = static_cast<IplImage*>(img);

	/* Step 6: while (scan lines remain to be read) */
	/*           jpeg_read_scanlines(...); */

	/* Here we use the library's state variable cinfo.output_scanline as the
	* loop counter, so that we don't have to keep track ourselves.
	*/
	const unsigned int nCols = cinfo.output_width;
	const unsigned int nRows = cinfo.output_height;

	for (unsigned int row = 0; row<nRows;row++)
	{
		/* jpeg_read_scanlines expects an array of pointers to scanlines.
		* Here the array is only one element long, but you could ask for
		* more than one scanline at a time if that's more convenient.
		*/
		jpeg_read_scanlines(&cinfo, buffer, 1);

		/* Copy into the CImage object */
		if (isColor())
		{
			// Flip RGB bytes order!
			char *target = &ipl->imageData[ row * ipl->widthStep ];
			const char *src = (char *)buffer[0];
			for (unsigned int col=0;col<nCols;col++)
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

	jpeg_finish_decompress(&cinfo);
	/* We can ignore the return value since suspension is not possible
	* with the stdio data source.
	*/

	/* Step 8: Release JPEG decompression object */

	/* This is an important step since it will release a good deal of memory. */
	jpeg_destroy_decompress(&cinfo);

	// DONE!
	MRPT_END
#endif
}

