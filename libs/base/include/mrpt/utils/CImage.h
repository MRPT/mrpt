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
#ifndef CImage_H
#define CImage_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/CCanvas.h>
#include <mrpt/utils/TCamera.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/exceptions.h>

namespace mrpt
{
	namespace utils
	{
		/** Interpolation methods for images.
		  *  Used for OpenCV related operations with images, but also with MRPT native classes.
		  * \sa mrpt::utils::CMappedImage, CImage::scaleImage
		  */
		enum TInterpolationMethod
		{
			IMG_INTERP_NN = 0,
			IMG_INTERP_LINEAR=1,
			IMG_INTERP_CUBIC=2,
			IMG_INTERP_AREA=3
		};

		/** For use in mrpt::utils::CImage */
		typedef int  TImageChannels;
		#define CH_GRAY  1
		#define CH_RGB   3

		/** For usage in one of the CImage constructors */
		enum TConstructorFlags_CImage
		{
			UNINITIALIZED_IMAGE = 0,
			FAST_REF_OR_CONVERT_TO_GRAY = 1
		};

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CImage, mrpt::utils::CSerializable )

		/** A class for storing images as grayscale or RGB bitmaps.
		 *  File I/O is supported in two different ways:
		 *		- Binary dump using the CSerializable interface(<< and >> operators), just as most objects
		 *          in the MRPT library. This format is not compatible with any standarized image format.
		 *		- Saving/loading from files of different formats (bmp,jpg,png,...) using the methods CImage::loadFromFile and CImage::saveToFile.
		 *
		 *  How to create color/grayscale images:
		 *  \code
		 *    CImage  img1(width, height,  CH_GRAY );  // Grayscale image (8U1C)
		 *    CImage  img2(width, height,  CH_RGB );  // RGB image (8U3C)
		 *  \endcode
		 *
		 * Additional notes:
		 *		- The OpenCV "IplImage" format is used internally for compatibility with all OpenCV functions. See CImage::getAsIplImage and CImage::getAs<>(). Example:
		 *         \code
		 *            CImage  img;
		 *            ...
		 *            // Call to OpenCV function expecting an "IplImage *" or a "void* arr":
		 *            cvFunction( img.getAs<IplImage>(), ... );
		 *         \endcode
		 *		- Only the unsigned 8-bit storage format for pixels (on each channel) is supported.
		 *		- An external storage mode can be enabled by calling CImage::setExternalStorage, useful for storing large collections of image objects in memory while loading the image data itself only for the relevant images at any time.
		 *		- To move images from one object to the another, use CImage::copyFastFrom rather than the copy operator =.
		 *		- If you are interested in a smart pointer to an image, use:
		 *  \code
		 *    CImagePtr   myImgPtr = CImagePtr( new CImage(...) );
		 *  \endcode
		 *		- To set a CImage from an OpenCV "IPLImage*", use the methods:
		 *			- CImage::loadFromIplImage
		 *			- CImage::setFromIplImage
		 *			- CImage::CImage(void *IPL)
		 *
		 * For many computer vision functions that use CImage as its image data type, see mrpt::vision.
		 *
		 * \note This class acts as a wrapper class to a small subset of OpenCV functions. IplImage is the internal storage structure.
		 *
		 * \sa mrpt::vision, mrpt::vision::CFeatureExtractor, CSerializable, CCanvas
		 */
		class BASE_IMPEXP CImage : public mrpt::utils::CSerializable, public CCanvas
		{
			friend class CImageFloat;
			DEFINE_SERIALIZABLE( CImage )
		public:

			// ================================================================
			/** @name Constructors & destructor
			    @{ */

			/** Default constructor: initialize an 1x1 RGB image. */
			CImage();

			/** Constructor for a given image size and type.
			  *  Examples:
			  *   \code
			  *    CImage  img1(width, height,  CH_GRAY );  // Grayscale image (8U1C)
			  *    CImage  img2(width, height,  CH_RGB );  // RGB image (8U3C)
			  *   \endcode
			  */
			CImage( unsigned int	width,
					unsigned int	height,
					TImageChannels	nChannels = CH_RGB,
					bool			originTopLeft = true
					);

			/** Copy constructor, makes a full copy of the original image contents */
			CImage( const CImage &o );

			/** Copy constructor */
			CImage( const CImageFloat &o );

			/** Fast constructor that leaves the image uninitialized (the internal IplImage pointer set to NULL).
			  *  Use only when you know the image will be soon be assigned another image.
			  *  Example of usage:
			  *   \code
			  *    CImage myImg(UNINITIALIZED_IMAGE);
			  *   \endcode
			  */
			inline CImage(TConstructorFlags_CImage constructor_flag) : img(NULL),m_imgIsReadOnly(false), m_imgIsExternalStorage(false)
			{ }

			/** Fast constructor of a grayscale version of another image, making a <b>reference</b> to the original image if it already was in grayscale, or otherwise creating a new grayscale image and converting the original image into it.
			  *   It's <b>very important to keep in mind</b> that the original image can't be destroyed before the new object being created with this constructor.
			  * Example of usage:
			  *   \code
			  *     void my_func(const CImage &in_img) {
			  *        const CImage gray_img(in_img, FAST_REF_OR_CONVERT_TO_GRAY);
			  *        // We can now operate on "gray_img" being sure it's in grayscale.
			  *     }
			  *   \endcode
			  */
			inline CImage(const CImage& other_img, TConstructorFlags_CImage constructor_flag) : img(NULL),m_imgIsReadOnly(false), m_imgIsExternalStorage(false)
			{
				if( other_img.isColor() ) other_img.grayscale(*this);
				else this->setFromImageReadOnly(other_img);
			}

			/** Constructor from an IPLImage*, making a copy of the image.
			  * \sa loadFromIplImage, setFromIplImage
			  */
			CImage( void *iplImage );

			/** Destructor: */
			virtual ~CImage( );

			/** @} */
			// ================================================================


			/** By default, when storing images through the CSerializable interface, grayscale images will be ZIP compressed if they are larger than 16Kb: this flag can be turn on to disable ZIP compression and gain speed versus occupied space.
			  *  The default value of this variable is "false".
			  */
			static bool DISABLE_ZIP_COMPRESSION;


			// ================================================================
			/** @name Manipulate the image contents or size, various computer-vision methods (image filters, undistortion, etc.)
			    @{ */

			/** Changes the size of the image, erasing previous contents (does NOT scale its current content, for that, see scaleImage).
			  *  - nChannels: Can be 3 for RGB images or 1 for grayscale images.
			  *  - originTopLeft: Is true if the top-left corner is (0,0). In other case, the reference is the bottom-left corner.
			  * \sa scaleImage
			  */
			inline void  resize(
					unsigned int	width,
					unsigned int	height,
					TImageChannels	nChannels,
					bool			originTopLeft )
			{
				ASSERT_(img!=NULL);
				changeSize(width,height,nChannels,originTopLeft);
			}

			/** Scales this image to a new size, interpolating as needed.
			  * \sa resize, rotateImage
			  */
			void scaleImage( unsigned int width, unsigned int height, TInterpolationMethod interp = IMG_INTERP_CUBIC );

			/** Scales this image to a new size, interpolating as needed, saving the new image in a different output object.
			  * \sa resize, rotateImage
			  */
			void scaleImage( CImage &out_img, unsigned int width, unsigned int height, TInterpolationMethod interp = IMG_INTERP_CUBIC ) const;

			/** Rotates the image by the given angle around the given center point, with an optional scale factor.
			  * \sa resize, scaleImage
			  */
			void rotateImage( double angle_radians, unsigned int center_x, unsigned int center_y, double scale = 1.0 );

			/** Changes the value of the pixel (x,y).
			  *  Pixel coordinates starts at the left-top corner of the image, and start in (0,0).
			  *  The meaning of the parameter "color" depends on the implementation: it will usually
			  *   be a 24bit RGB value (0x00RRGGBB), but it can also be just a 8bit gray level.
			  *  This method must support (x,y) values OUT of the actual image size without neither
			  *   raising exceptions, nor leading to memory access errors.
			  */
			void  setPixel(int x, int y, size_t color);

			/** Changes the property of the image stating if the top-left corner (vs. bottom-left) is the coordinate reference.
			  */
			void  setOriginTopLeft(bool val);

			/** Draws a circle of a given radius.
			  * \param x The center - x coordinate in pixels.
			  * \param y The center - y coordinate in pixels.
			  * \param radius The radius - in pixels.
			  * \param color The color of the circle.
			  * \param width The desired width of the line
			  */
			void  drawCircle(
				int				x,
				int				y,
				int				radius,
				const mrpt::utils::TColor &color = mrpt::utils::TColor(255,255,255),
				unsigned int	width = 1);

			void equalizeHistInPlace(); //!< Equalize the image histogram, replacing the original image.
			void equalizeHist( CImage  &outImg ) const; //!< Equalize the image histogram, saving the new image in the given output object.

			/** Returns a new image scaled down to half its original size.
			  * \exception std::exception On odd size
			  * \sa scaleDouble, scaleImage
			  */
			CImage  scaleHalf()const;

			/** Returns a new image scaled up to double its original size.
			  * \exception std::exception On odd size
			  * \sa scaleHalf, scaleImage
			  */
			CImage  scaleDouble()const;




			/** Update a part of this image with the "patch" given as argument.
			 * The "patch" will be "pasted" at the (col,row) coordinates of this image.
			 * \exception std::exception if patch pasted on the pixel (_row, _column) jut out
			 * of the image.
			 * \sa extract_patch
			 */
			void update_patch(const CImage &patch,
					  const unsigned int col,
					  const unsigned int row);

			/** Extract a patch from this image, saveing it into "patch" (its previous contents will be overwritten).
			  *  The patch to extract starts at (col,row) and has the given dimensions.
			  * \sa update_patch
			  */
			void  extract_patch(
				CImage	&patch,
				const unsigned int	col=0,
				const unsigned int	row=0,
				const unsigned int	width=1,
				const unsigned int	height=1 ) const;

			/** Computes the correlation coefficient (returned as val), between two images
			*	This function use grayscale images only
			*	img1, img2 must be same size
			* (by AJOGD @ DEC-2006)
			*/
			float  correlate( const CImage &img2int, int width_init=0, int height_init=0 )const;

			/**	Computes the correlation between this image and another one, encapsulating the openCV function cvMatchTemplate
			*
			* \param patch_img The "patch" image, which must be equal, or smaller than "this" image. This function supports gray-scale (1 channel only) images.
			* \param u_search_ini The "x" coordinate of the search window.
			* \param v_search_ini The "y" coordinate of the search window.
			* \param u_search_size The width of the search window.
			* \param v_search_size The height of the search window.
			* \param u_max The u coordinate where find the maximun cross correlation value.
			* \param v_max The v coordinate where find the maximun cross correlation value
			* \param max_val The maximun value of cross correlation which we can find
			* \param out_corr_image  If a !=NULL pointer is provided, it will be saved here the correlation image. The size of the output image is (this_width-patch_width+1, this_height-patch_height+1 )
			*  Note: By default, the search area is the whole (this) image.
			* (by AJOGD @ MAR-2007)
			*/
			void  cross_correlation(
				const CImage	&patch_img,
				size_t				&u_max,
				size_t				&v_max,
				double				&max_val,
				int					u_search_ini=-1,
				int					v_search_ini=-1,
				int					u_search_size=-1,
				int					v_search_size=-1,
				CImage				*out_corr_image = NULL
				)const;

			/**	Computes the correlation matrix between this image and another one.
			*   This implementation uses the 2D FFT for achieving reduced computation time.
			* \param in_img The "patch" image, which must be equal, or smaller than "this" image. This function supports gray-scale (1 channel only) images.
			* \param u_search_ini The "x" coordinate of the search window.
			* \param v_search_ini The "y" coordinate of the search window.
			* \param u_search_size The width of the search window.
			* \param v_search_size The height of the search window.
			* \param out_corr The output for the correlation matrix, which will be "u_search_size" x "v_search_size"
			* \param biasThisImg This optional parameter is a fixed "bias" value to be substracted to the pixels of "this" image before performing correlation.
			* \param biasInImg This optional parameter is a fixed "bias" value to be substracted to the pixels of "in_img" image before performing correlation.
			*  Note: By default, the search area is the whole (this) image.
			* (by JLBC @ JAN-2006)
			* \sa cross_correlation
			*/
			void  cross_correlation_FFT(
				const CImage	&in_img,
				math::CMatrixFloat		&out_corr,
				int					u_search_ini=-1,
				int					v_search_ini=-1,
				int					u_search_size=-1,
				int					v_search_size=-1,
				float				biasThisImg = 0,
				float				biasInImg = 0
				) const;


			/** Optimize de brightness range of a image without using histogram
			* Only for one channel images.
			*/
			void  normalize();

			/** Flips vertically the image.
			  * \sa swapRB
			  */
			void flipVertical(bool also_swapRB = false);

			/** Swaps red and blue channels.
			  * \sa flipVertical
			  */
			void swapRB();

			/** Rectify (un-distort) the image according to some camera parameters, and returns an output un-distorted image.
			  * \param out_img The output rectified image
			  * \param cameraParams The input camera params (containing the intrinsic and distortion parameters of the camera)
			  */
			void rectifyImage( CImage &out_img, const mrpt::utils::TCamera &cameraParams) const;

			/** Rectify (un-distort) the image according to a certain camera matrix and vector of distortion coefficients and returns an output rectified image
			  * \param out_img The output rectified image
			  * \param cameraMatrix The input camera matrix (containing the intrinsic parameters of the camera): [fx 0 cx; 0 fy cy; 0 0 1]: (fx,fy)  focal length and (cx,cy) principal point coordinates
			  * \param distCoeff The (input) distortion coefficients: [k1, k2, p1, p2]:  k1 and k2 (radial) and p1 and p2 (tangential)
			  */
			inline void rectifyImage( CImage &out_img, const math::CMatrixDouble33 &cameraMatrix, const vector_double &distCoeff ) const
			{
				mrpt::utils::TCamera  cam;
				cam.intrinsicParams = cameraMatrix;
				cam.setDistortionParamsVector(distCoeff);
				rectifyImage(out_img,cam);
			}

			/** Rectify (un-distort) the image according to a certain camera matrix and vector of distortion coefficients, replacing "this" with the rectified image
			  * \param cameraParams The input camera params (containing the intrinsic and distortion parameters of the camera)
			  */
			void rectifyImageInPlace(const mrpt::utils::TCamera &cameraParams );

			/** Rectify (un-distort) the image according to a certain camera matrix and vector of distortion coefficients, replacing "this" with the rectified image
			  * \param cameraMatrix The input camera matrix (containing the intrinsic parameters of the camera): [fx 0 cx; 0 fy cy; 0 0 1]: (fx,fy)  focal length and (cx,cy) principal point coordinates
			  * \param distCoeff The (input) distortion coefficients: [k1, k2, p1, p2]:  k1 and k2 (radial) and p1 and p2 (tangential)
			  */
			inline void rectifyImageInPlace( const math::CMatrixDouble33 &cameraMatrix, const vector_double &distCoeff )
			{
				mrpt::utils::TCamera  cam;
				cam.intrinsicParams = cameraMatrix;
				cam.setDistortionParamsVector(distCoeff);
				rectifyImageInPlace(cam);
			}

			/** Filter the image with a Median filter with a window size WxW, returning the filtered image in out_img  */
			void filterMedian( CImage &out_img, int W=3 ) const;

			/** Filter the image with a Median filter with a window size WxH, replacing "this" image by the filtered one. */
			void filterMedianInPlace( int W=3 );

			/** Filter the image with a Gaussian filter with a window size WxH, returning the filtered image in out_img  */
			void filterGaussianInPlace( int W = 3, int H = 3 );

			/** Filter the image with a Gaussian filter with a window size WxH, replacing "this" image by the filtered one. */
			void filterGaussian( CImage &out_img, int W = 3, int H = 3) const;

			/** Draw onto this image the detected corners of a chessboard. The length of cornerCoords must be the product of the two check_sizes.
			  *
			  * \param cornerCoords [IN] The pixel coordinates of all the corners.
			  * \param check_size_x [IN] The number of squares, in the X direction
			  * \param check_size_y [IN] The number of squares, in the Y direction
			  *
			  * \return false if the length of cornerCoords is inconsistent (nothing is drawn then).
			  *
			  * \sa mrpt::vision::findChessboardCorners
			  */
			bool drawChessboardCorners(
				std::vector<TPixelCoordf> 	&cornerCoords,
				unsigned int  check_size_x,
				unsigned int  check_size_y );

			/** Joins two images side-by-side horizontally. Both images must have the same number of rows and be of the same type (i.e. depth and color mode)
			  *
			  * \param im1 [IN] The first image.
			  * \param im2 [IN] The other image.
			  */
			void joinImagesHorz(
				const CImage &im1,
				const CImage &im2 );

			/** Compute the KLT response at a given pixel (x,y) - Only for grayscale images (for efficiency it avoids converting to grayscale internally).
			  */
			float KLT_response(
				const unsigned int x,
				const unsigned int y,
				const unsigned int half_window_size ) const;

			/** @} */
			// ================================================================



			// ================================================================
			/** @name Copy, move & swap  operations
			    @{ */

			/** Copy operator
			  * \sa copyFastFrom
			  */
			CImage& operator = (const CImage& o);

			/** Copy operator from a gray-scale, float image:
			  * \sa copyFastFrom
			  */
			CImage& operator = (const CImageFloat& o);

			/** Moves an image from another object, erasing the origin image in the process (this is much faster than copying)
			  * \sa operator =
			  */
			void copyFastFrom( CImage &o );

			void swap(CImage &o); //!< Very efficient swap of two images (just swap the internal pointers)

			/** @} */
			// ================================================================


			// ================================================================
			/** @name Access to image contents (IplImage structure and raw pixels).
			    @{ */

			/** Returns a pointer to an T* containing the image - the idea is to call like "img.getAs<IplImage>()" so we can avoid here including OpenCV's headers. \sa getAsIplImage */
			template <typename T> inline const T* getAs() const {
				makeSureImageIsLoaded();
				return reinterpret_cast<const T*>(img);
			}
			/** Returns a pointer to an T* containing the image - the idea is to call like "img.getAs<IplImage>()" so we can avoid here including OpenCV's headers. \sa getAsIplImage */
			template <typename T> inline T* getAs(){
				makeSureImageIsLoaded();
				return reinterpret_cast<T*>(img);
			}

			/** Returns a pointer to an OpenCV's IplImage struct containing the image, which is linked to this class: free neigther that pointer nor this class until they are not required anymore, since this class is in charge of freeing the memory buffers inside of the returned image.  \sa getAs */
			inline void*  getAsIplImage() const {
				makeSureImageIsLoaded();
				return img;
			}

			/**  Access to pixels without checking boundaries - Use normally the () operator better, which checks the coordinates.
			  \sa CImage::operator()
			  */
			unsigned char*  get_unsafe(
						unsigned int	col,
						unsigned int	row,
						unsigned int	channel=0) const;

			/** Returns the contents of a given pixel at the desired channel, in float format: [0,255]->[0,1]
			  *   The coordinate origin is pixel(0,0)=top-left corner of the image.
			  * \exception std::exception On pixel coordinates out of bounds
			  * \sa operator()
			  */
			float  getAsFloat(unsigned int col, unsigned int row, unsigned int channel) const;

			/** Returns the contents of a given pixel (for gray-scale images, in color images the gray scale equivalent is computed for the pixel), in float format: [0,255]->[0,1]
			  *   The coordinate origin is pixel(0,0)=top-left corner of the image.
			  * \exception std::exception On pixel coordinates out of bounds
			  * \sa operator()
			  */
			float  getAsFloat(unsigned int col, unsigned int row) const;

			/** Returns a pointer to a given pixel information.
			 *   The coordinate origin is pixel(0,0)=top-left corner of the image.
			 * \exception std::exception On pixel coordinates out of bounds
			 */
			unsigned char*  operator()(unsigned int col, unsigned int row, unsigned int channel = 0) const;

			/** @} */
			// ================================================================



			// ================================================================
			/** @name Query image properties
			    @{ */

			/** Returns the width of the image in pixels
			  * \sa getSize
			  */
			size_t getWidth() const;

			/** Returns the height of the image in pixels
			  * \sa getSize
			  */
			size_t getHeight() const;

			/** Return the size of the image
			  * \sa getWidth, getHeight
			  */
			void getSize(TImageSize &s) const;

			/** Return the size of the image
			  * \sa getWidth, getHeight
			  */
			inline TImageSize getSize() const {
				TImageSize  ret;
				getSize(ret);
				return ret;
			}

			/** Return the maximum pixel value of the image, as a float value in the range [0,1]
			  * \sa getAsFloat
			  */
			float  getMaxAsFloat() const;

			/** Returns true if the image is RGB, false if it is grayscale */
			bool  isColor() const;

			/** Returns true if the coordinates origin is top-left, or false if it is bottom-left  */
			bool  isOriginTopLeft() const;

			/** Returns a string of the form "BGR","RGB" or "GRAY" indicating the channels ordering. */
			const char *  getChannelsOrder()const;

			/** Returns the number of channels, typically 1 (GRAY) or 3 (RGB)
			  * \sa isColor
			  */
			TImageChannels getChannelCount() const;

			/**	Returns the image as a matrix with pixel grayscale values in the range [0,1]
			  *  \param doResize If set to true (default), the output matrix will be always the size of the image at output. If set to false, the matrix will be enlarged to the size of the image, but it will not be cropped if it has room enough (useful for FFT2D,...)
			  *  \param x_min The starting "x" coordinate to extract (default=0=the first column)
			  *  \param y_min The starting "y" coordinate to extract (default=0=the first row)
			  *  \param x_max The final "x" coordinate (inclusive) to extract (default=-1=the last column)
			  *  \param y_max The final "y" coordinate (inclusive) to extract (default=-1=the last row)
			  * \sa setFromMatrix
			  */
			void  getAsMatrix(
				mrpt::math::CMatrixFloat	&outMatrix,
				bool		doResize = true,
				int			x_min = 0,
				int			y_min = 0,
				int			x_max = -1,
				int			y_max = -1
				)  const;

			/**	Returns the image as a matrix, where the image is "tiled" (repeated) the required number of times to fill the entire size of the matrix on input.
			  */
			void  getAsMatrixTiled( math::CMatrix &outMatrix )  const;

			/** @} */
			// ================================================================


			// ================================================================
			/** @name External storage-mode methods
			    @{  */

			/**  By using this method the image is marked as referenced to an external file, which will be loaded only under demand.
			  *   A CImage with external storage does not consume memory until some method trying to access the image is invoked (e.g. getWidth(), isColor(),...)
			  *   At any moment, the image can be unloaded from memory again by invoking unload.
			  *   An image becomes of type "external storage" only through calling setExternalStorage. This property remains after serializing the object.
			  *   File names can be absolute, or relative to the CImage::IMAGES_PATH_BASE directory. Filenames staring with "X:\" or "/" are considered absolute paths.
			  *   By calling this method the current contents of the image are NOT saved to that file, because this method can be also called
			  *    to let the object know where to load the image in case its contents are required. Thus, for saving images in this format (not when loading)
			  *    the proper order of commands should be:
			  *   \code
			  *   img.saveToFile( fileName );
			  *   img.setExternalStorage( fileName );
			  *   \endcode
			  *
			  *   \note Modifications to the memory copy of the image are not automatically saved to disk.
			  *  \sa unload, isExternallyStored
			  */
			void setExternalStorage( const std::string &fileName ) MRPT_NO_THROWS;

			static std::string IMAGES_PATH_BASE;		//!< By default, "."  \sa setExternalStorage

			/** See setExternalStorage(). */
			bool isExternallyStored() const MRPT_NO_THROWS { return m_imgIsExternalStorage; }

			inline std::string  getExternalStorageFile() const MRPT_NO_THROWS //!< Only if isExternallyStored() returns true. \sa getExternalStorageFileAbsolutePath
			{
				return m_externalFile;
			}

			/** Only if isExternallyStored() returns true. \sa getExternalStorageFile */
			void getExternalStorageFileAbsolutePath(std::string &out_path) const;

			/** Only if isExternallyStored() returns true. \sa getExternalStorageFile */
			inline std::string getExternalStorageFileAbsolutePath() const {
					std::string tmp;
					getExternalStorageFileAbsolutePath(tmp);
					return tmp;
			}

			/** For external storage image objects only, this method unloads the image from memory (or does nothing if already unloaded).
			  *  It does not need to be called explicitly, unless the user wants to save memory for images that will not be used often.
			  *  If called for an image without the flag "external storage", it is simply ignored.
			  * \sa setExternalStorage
			  */
			void unload() MRPT_NO_THROWS;

			/** @}  */
			// ================================================================


			// ================================================================
			/** @name Set, load & save methods
			    @{  */

			/** Reads the image from raw pixels buffer in memory.
			  */
			void  loadFromMemoryBuffer( unsigned int width, unsigned int height, bool color, unsigned char *rawpixels, bool swapRedBlue = false );

			/** Reads a color image from three raw pixels buffers in memory.
			  * bytesPerRow is the number of bytes per row per channel, i.e. the row increment.
			  */
			void  loadFromMemoryBuffer( unsigned int width, unsigned int height, unsigned int bytesPerRow, unsigned char *red, unsigned char *green, unsigned char *blue );

			/** Reads the image from a OpenCV IplImage object (making a copy).
			  */
			void  loadFromIplImage( void* iplImage );

			/** Reads the image from a OpenCV IplImage object (WITHOUT making a copy).
			  *   This method provides a fast method to grab images from a camera without making a copy of every frame.
			  */
			void  setFromIplImage( void* iplImage );

			/** Reads the image from a OpenCV IplImage object (WITHOUT making a copy) and from now on the image cannot be modified, just read.
			  *  When assigning an IPLImage to this object with this method, the IPLImage will NOT be released/freed at this object destructor.
			  *   This method provides a fast method to grab images from a camera without making a copy of every frame.
			  *  \sa setFromImageReadOnly
			  */
			void  setFromIplImageReadOnly( void* iplImage );

			/** Sets the internal IplImage pointer to that of another given image, WITHOUT making a copy, and from now on the image cannot be modified in this object (it will be neither freed, so the memory responsibility will still be of the original image object).
			  *  When assigning an IPLImage to this object with this method, the IPLImage will NOT be released/freed at this object destructor.
			  *  \sa setFromIplImageReadOnly
			  */
			inline void setFromImageReadOnly( const CImage &other_img ) { setFromIplImageReadOnly(other_img.getAsIplImage() ); }

			/** Set the image from a matrix, interpreted as grayscale intensity values, in the range [0,1] (normalized=true) or [0,255] (normalized=false)
			  * \sa getAsMatrix
			  */
			void setFromMatrix(const mrpt::math::CMatrixFloat &m, bool matrix_is_normalized=true);

			/** Set the image from a matrix, interpreted as grayscale intensity values, in the range [0,1] (normalized=true) or [0,255] (normalized=false)
			  * \sa getAsMatrix
			  */
			void setFromMatrix(const mrpt::math::CMatrixDouble &m, bool matrix_is_normalized=true);


			/** Reads the image from a binary stream containing a binary jpeg file.
			 * \exception std::exception On pixel coordinates out of bounds
			  */
			void  loadFromStreamAsJPEG( CStream &in );

			/** Load image from a file, whose format is determined from the extension (internally uses OpenCV).
			 * \param fileName The file to read from.
			 * \param isColor Specifies colorness of the loaded image:
			 *  - if >0, the loaded image is forced to be color 3-channel image;
			 *  - if 0, the loaded image is forced to be grayscale;
			 *  - if <0, the loaded image will be loaded as is (with number of channels depends on the file).
			 * The supported formats are:
			 *
			 * - Windows bitmaps - BMP, DIB;
			 * - JPEG files - JPEG, JPG, JPE;
			 * - Portable Network Graphics - PNG;
			 * - Portable image format - PBM, PGM, PPM;
			 * - Sun rasters - SR, RAS;
			 * - TIFF files - TIFF, TIF.
			 *
			 * \return False on any error
			 * \sa saveToFile, setExternalStorage
			 */
			bool  loadFromFile( const std::string& fileName, int isColor = -1  );

			/** Save the image to a file, whose format is determined from the extension (internally uses OpenCV).
			 * \param fileName The file to write to.
			 *
			 * The supported formats are:
			 *
			 * - Windows bitmaps - BMP, DIB;
			 * - JPEG files - JPEG, JPG, JPE;
			 * - Portable Network Graphics - PNG;
			 * - Portable image format - PBM, PGM, PPM;
			 * - Sun rasters - SR, RAS;
			 * - TIFF files - TIFF, TIF.
			 *
			 * \param jpeg_quality Only for JPEG files, the quality of the compression in the range [0-100]. Larger is better quality but slower.
			 * \note jpeg_quality is only effective if MRPT is compiled against OpenCV 1.1.0 or newer.
			 * \return False on any error
			 * \sa loadFromFile
			 */
			bool  saveToFile( const std::string& fileName, int jpeg_quality = 95 ) const;

			/** Save image to binary stream as a JPEG (.jpg) compresed format.
			 * \exception std::exception On number of rows or cols equal to zero, or other errors.
			 * \sa saveToJPEG
			 */
			void  saveToStreamAsJPEG( CStream		&out  )const;

			/** @}  */
			// ================================================================


			// ================================================================
			/** @name Color/Grayscale conversion
			    @{ */

			/** Returns a grayscale version of the image, or itself if it is already a grayscale image.
			  */
			CImage  grayscale() const;

			/** Returns a grayscale version of the image, or itself if it is already a grayscale image.
			  * \sa colorImage
			  */
			void grayscale( CImage  &ret ) const;

			/** Returns a RGB version of the grayscale image, or itself if it is already a RGB image.
			  * \sa grayscale
			  */
			void colorImage( CImage  &ret ) const;

			/** Replaces this grayscale image with a RGB version of it.
			  * \sa grayscaleInPlace
			  */
			void colorImageInPlace();


			/** Replaces the image with a grayscale version of it.
			  * \sa colorImageInPlace
			  */
			void grayscaleInPlace();

			/** @} */
			// ================================================================


		protected:
			/** @name Data members
				@{ */

			void 	*img;  //!< The internal IplImage pointer to the actual image content.

			/**  Set to true only when using setFromIplImageReadOnly.
			  * \sa setFromIplImageReadOnly  */
			bool	m_imgIsReadOnly;
			/**  Set to true only when using setExternalStorage.
			  * \sa setExternalStorage
			  */
			mutable bool	m_imgIsExternalStorage;
			mutable std::string 	m_externalFile;		//!< The file name of a external storage image.

			/** @} */

			/**  Resize the buffers in "img" to accomodate a new image size and/or format.
			  */
			void  changeSize(
					unsigned int	width,
					unsigned int	height,
					TImageChannels	nChannels,
					bool			originTopLeft );

			/** Release the internal IPL image, if not NULL or read-only. */
			void releaseIpl(bool thisIsExternalImgUnload = false) MRPT_NO_THROWS;

			/** Checks if the image is of type "external storage", and if so and not loaded yet, load it. */
			void makeSureImageIsLoaded() const throw (std::exception,utils::CExceptionExternalImageNotFound );

		}; // End of class

		typedef CImage CMRPTImage;	//!< Deprecated name.


	} // end of namespace utils

} // end of namespace mrpt

#endif
