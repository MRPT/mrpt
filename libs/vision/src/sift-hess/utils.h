/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef UTILS_H
#define UTILS_H

//#include "cxcore.h"
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#include <stdio.h>

#ifdef __cplusplus
 extern "C" {
#endif


/* absolute value */
#ifndef ABS
#define ABS(x) ( ( x < 0 )? -x : x )
#endif


/***************************** Inline Functions ******************************/


/**
A function to get a pixel value from an 8-bit unsigned image.

@param img an image
@param r row
@param c column
@return Returns the value of the pixel at (\a r, \a c) in \a img
*/
static __inline int pixval8( IplImage* img, int r, int c )
{
	return (int)( ( (uchar*)(img->imageData + img->widthStep*r) )[c] );
}


/**
A function to set a pixel value in an 8-bit unsigned image.

@param img an image
@param r row
@param c column
@param val pixel value
*/
static __inline void setpix8( IplImage* img, int r, int c, uchar val)
{
	( (uchar*)(img->imageData + img->widthStep*r) )[c] = val;
}


/**
A function to get a pixel value from a 32-bit floating-point image.

@param img an image
@param r row
@param c column
@return Returns the value of the pixel at (\a r, \a c) in \a img
*/
static __inline float pixval32f( IplImage* img, int r, int c )
{
	return ( (float*)(img->imageData + img->widthStep*r) )[c];
}


/**
A function to set a pixel value in a 32-bit floating-point image.

@param img an image
@param r row
@param c column
@param val pixel value
*/
static __inline void setpix32f( IplImage* img, int r, int c, float val )
{
	( (float*)(img->imageData + img->widthStep*r) )[c] = val;
}


/**
A function to get a pixel value from a 64-bit floating-point image.

@param img an image
@param r row
@param c column
@return Returns the value of the pixel at (\a r, \a c) in \a img
*/
static __inline double pixval64f( IplImage* img, int r, int c )
{
	return (double)( ( (double*)(img->imageData + img->widthStep*r) )[c] );
}


/**
A function to set a pixel value in a 64-bit floating-point image.

@param img an image
@param r row
@param c column
@param val pixel value
*/
static __inline void setpix64f( IplImage* img, int r, int c, double val )
{
	( (double*)(img->imageData + img->widthStep*r) )[c] = val;
}


/**************************** Function Prototypes ****************************/


/**
Prints an error message and aborts the program.  The error message is
of the form "Error: ...", where the ... is specified by the \a format
argument

@param format an error message format string (as with \c printf(3)).
*/
extern void fatal_error( char* format, ... );


/**
Replaces a file's extension, which is assumed to be everything after the
last dot ('.') character.

@param file the name of a file

@param extn a new extension for \a file; should not include a dot (i.e.
	\c "jpg", not \c ".jpg") unless the new file extension should contain
	two dots.

@return Returns a new string formed as described above.  If \a file does
	not have an extension, this function simply adds one.
*/
extern char* replace_extension( const char* file, const char* extn );


/**
A function that removes the path from a filename.  Similar to the Unix
basename command.

@param pathname a (full) path name

@return Returns the basename of \a pathname.
*/
// JLBC: Changed for MRPT: It's already declared in string.h
//extern char* basename( const char* pathname );

/**
Displays progress in the console with a spinning pinwheel.  Every time this
function is called, the state of the pinwheel is incremented.  The pinwheel
has four states that loop indefinitely: '|', '/', '-', '\'.

@param done if 0, this function simply increments the state of the pinwheel;
	otherwise it prints "done"
*/
extern void progress( int done );


/**
Erases a specified number of characters from a stream.

@param stream the stream from which to erase characters
@param n the number of characters to erase
*/
extern void erase_from_stream( FILE* stream, int n );


/**
Doubles the size of an array with error checking

@param array pointer to an array whose size is to be doubled
@param n number of elements allocated for \a array
@param size size in bytes of elements in \a array

@return Returns the new number of elements allocated for \a array.  If no
	memory is available, returns 0 and frees array.
*/
extern int array_double( void** array, int n, int size );


/**
Calculates the squared distance between two points.

@param p1 a point
@param p2 another point
*/
extern double dist_sq_2D( CvPoint2D64f p1, CvPoint2D64f p2 );


/**
Draws an x on an image.

@param img an image
@param pt the center point of the x
@param r the x's radius
@param w the x's line weight
@param color the color of the x
*/
extern void draw_x( IplImage* img, CvPoint pt, int r, int w, CvScalar color );


/**
Combines two images by scacking one on top of the other

@param img1 top image
@param img2 bottom image

@return Returns the image resulting from stacking \a img1 on top if \a img2
*/
extern IplImage* stack_imgs( IplImage* img1, IplImage* img2 );


/**
Allows user to view an array of images as a video.  Keyboard controls
are as follows:

<ul>
<li>Space - start and pause playback</li>
<li>Page Up - skip forward 10 frames</li>
<li>Page Down - jump back 10 frames</li>
<li>Right Arrow - skip forward 1 frame</li>
<li>Left Arrow - jump back 1 frame</li>
<li>Backspace - jump back to beginning</li>
<li>Esc - exit playback</li>
<li>Closing the window also exits playback</li>
</ul>

@param imgs an array of images
@param n number of images in \a imgs
@param win_name name of window in which images are displayed
*/
extern void vid_view( IplImage** imgs, int n, char* win_name );


/**
Checks if a HighGUI window is still open or not

@param name the name of the window we're checking

@return Returns 1 if the window named \a name has been closed or 0 otherwise
*/
extern int win_closed( char* name );

#ifdef __cplusplus
 }
#endif

#endif
