///////////////////////////////////////////////////////
// General Doxygen documentation
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// The main title page
/**
@mainpage

\section Information
This C++ library is designed to be easy to use and portable for fast video saving, loading and display.
It supports OpenGL and X Windows.
It is released under the LGPL License.

\section Features
\subsection Language

    - Modern C++14 design
	- Optimized assembly code
    - Extensive use of exceptions
	- Easy access to many sources of video
	- Easy loading and saving of many kinds of image
	- Plenty of low-level comptuer vision functions
    - OpenGL wrappers 

\subsection Imaging

    - Type-safe easy to use images
    - Flexible loading and saving from a variety of types:
		- Native
			- PNM   
			- BMP
			- ASCII text
			- FITS 
			- PS   (saving only)
			- EPS  (saving only)
			- CVD (a custom type which supports fast, lossless compression of greyscale, RGB and Bayer images)
		- External libraries required
			- JPEG
			- TIFF 
			- PNG
		- 1/8/16/32 bit signed and unsigned integer and 32/64 bit floating point images in greyscale, RGB and RGBA.
		- Optimum bit depth and colour depth selected automatically
    - Image grabbing from video sources:
		  - Linux
          		- Video for Linux 2 devices
         	 	- Firewire IIDC cameras
                - Firewire over USB cameras produced by PointGrey - see http://www.ptgrey.com/products/fireflymv/index.asp \n
                  Use DVBuffer3 for that and make sure to set an image resolution
				- Whichever devices FFMPEG supports
		  - OSX
		        - AVFoundation devices via FFMPEG
		  - All UNIX platforms
		        - Live capture from HTTP server push JPEG cameras.
		  - All platforms
          		- AVI and MPEG files and other devices (i.e. whatever FFMPEG supports)
				- lists of images
				- Server push multipart JPEG streams.
				- libUVC for USB video camera streams
		  - Convenient run-time selection using a URL-like syntax
    - Colorspace conversions on images and video streams
    - Various image processing tools
	      - FAST corner detection
		  - Harris/Shi-Tomasi corner detection
		  - Connect components analysis
		  - Image interpolation and resampling
		  - Convolutions
		  - Drawing in to images
		  - Flipping, pasting, etc
		  - Interpolation, warping and resampling
		  - Integral images
		  - Distance transform
    - %Camera calibration support: Linear, Cubic, Quintic and Harris
		  - Program to calibrate cameras from video

\section Portability

  libCVD currently works well on Linux, OSX, Cygwin, MinGW (and therefore most
  likely any unix-like system), and is written in standards compliant C++14 with
  cross compilation in mind. It's tested with both GCC and LLVM.

  It's likely that it will compile under VisualStudio, but there's no current
  project file maintainer.
  
  libCVD now has a minimalist configuration system. You can simply chuck any of
  the sources into your project. Some configuration may be needed for CVD's
  interface to external libraries. 
  
  -Well tested (current):
	- Linux: x86, x86-64
	- Mac OS X: x86
		- Supports the OSX build environment including:
			- Frameworks
			- .dylib libraries
	- Cygwin: x86
	- MinGW: x86 (native and cross compile)
	- MinGW: x86_64 (cross compile)

  -Has worked on (current status unknown):
	- Linux: PPC
	- Mac OS X: PPC
	- Solaris: SPARC
	- Linux: ARM LPC3180, XScale (cross compile)
	- uCLinux: Blackfin  (cross compile)
	- FreeBSD: x86
	- OpenBSD: XScale
	- Win32: Visual Studio 2008
	- Win32: Visual Studio 2010 (via import of the VS 2008 project file)
	- iOS
		- XCode project provided
  
  -No longer supported
	- IRIX SGI O2: MIPS (configuration hacks removed)
	- Win32: Visual Studio 2005 (broken project file now removed)

\section Compiling

The normal system works:
@code
	./configure
	make 
	make install
@endcode

libCVD fully supports parallel builds (<code>make -j48</code> for instance).

\subsection OSX OSX Compilation notes

To build libCVD in 32 bit mode, use the \c configure_osx_32bit script instead of directly calling the configure script.

*/

///////////////////////////////////////////////////////
// Modules classifying classes and functions

/// @defgroup gImage Image storage and manipulation
/// Basic image functionality. The %CVD image classes provide fast and
/// flexible access to images.

/// @defgroup gImageIO Image loading and saving, and format conversion
/// Functions to support saving and loading of BasicImage and Image 
/// to and from streams. Supports a few common file formats (autodetecting on loading).
/// Also functions for perfoming type conversion as necessary.

/// @defgroup gVideo Video devices and video files
/// Classes and functions to manage video streams and present them as images.

/// @defgroup gVideoBuffer Video buffers
/// @ingroup gVideo
/// All classes and functions relating to video buffers (as opposed to video frames)

/// @defgroup gVideoFrame Video frames
/// @ingroup gVideo
/// All classes and functions relating to video frames (as opposed to video buffers)

/// @defgroup gException Exceptions 
/// Exceptions generated and thrown by %CVD classes and functions

/// @defgroup gGraphics Computer graphics
/// Classes and functions to support miscellaneous pixel operations

/// @defgroup gVision Computer Vision
/// Functions and classes to support common computer vision concepts and operations

/// @defgroup gGL GL helper functions and classes.
/// Overloaded versions of GL functions to use %CVD classes and datatypes, and
/// other helpful GL classes and functions.

/// @defgroup gMaths Mathematical operations
/// Useful mathematical classes and functions

/// @defgroup gLinAlg Linear Algebra
/// Classes and functions for common Linear Algebra concepts and operations

/// @defgroup gCPP General C++ and system helper functions
/// Classes and functions for writing better code


/// @namespace CVD
/// All classes and functions are within the CVD namespace

