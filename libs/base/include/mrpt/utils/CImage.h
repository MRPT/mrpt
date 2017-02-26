/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CImage_H
#define CImage_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/eigen_frwds.h>
#include <mrpt/utils/CCanvas.h>
#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/exceptions.h>

// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM( mrpt::utils::CImage )

namespace mrpt
{
	namespace utils
	{
		/** Interpolation methods for images.
		  *  Used for OpenCV related operations with images, but also with MRPT native classes.
		  * \sa mrpt::utils::CMappedImage, CImage::scaleImage
		 * \ingroup mrpt_base_grp
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
		 *  File I/O is supported as:
		 *		- Binary dump using the CSerializable interface(<< and >> operators), just as most objects
		 *          in the MRPT library. This format is not compatible with any standarized image format.
		 *		- Saving/loading from files of different formats (bmp,jpg,png,...) using the methods CImage::loadFromFile and CImage::saveToFile. 
		 *        Available formats are all those supported by OpenCV
		 *		- Importing from an XPM array (.xpm file format) using CImage::loadFromXPM
		 *		- Importing TGA images. See CImage::loadTGA()
		 *
		 *  How to create color/grayscale images:
		 *  \code
		 *    CImage  img1(width, height,  CH_GRAY );  // Grayscale image (8U1C)
		 *    CImage  img2(width, height,  CH_RGB );  // RGB image (8U3C)
		 *  \endcode
		 *
		 * Additional notes:
		 *		- The OpenCV "IplImage" format is used internally for compatibility with all OpenCV functions. Use CImage::getAs<IplImage>() to retrieve the internal structure. Example:
		 *         \code
		 *            CImage  img;
		 *            ...
		 *            // Call to OpenCV function expecting an "IplImage *" or a "void* arr":
		 *            cv::Mat cvImg = cv::cvarrToMat( img.getAs<IplImage>() );
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
		 *   Some functions are implemented in MRPT with highly optimized SSE2/SSE3 routines, in suitable platforms and compilers. To
		 *   see the list of optimizations refer to \ref sse_optimizations "this page". If optimized versions are not available in some
		 *   platform it falls back to default OpenCV methods.
		 *
		 * For many computer vision functions that use CImage as its image data type, see mrpt::vision.
		 *
		 * \note This class acts as a wrapper class to a small subset of OpenCV functions. IplImage is the internal storage structure.
		 *
		 * \sa mrpt::vision, mrpt::vision::CFeatureExtractor, mrpt::vision::CImagePyramid, CSerializable, CCanvas
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CImage : public mrpt::utils::CSerializable, public CCanvas
		{
			DEFINE_SERIALIZABLE( CImage )

            // This must be added for declaration of MEX-related functions
            DECLARE_MEX_CONVERSION



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

			/** Copy constructor, makes a full copy of the original image contents (unless it was externally stored, in that case, this new image will just point to the same image file). */
			CImage( const CImage &o );

			/** Fast constructor that leaves the image uninitialized (the internal IplImage pointer set to NULL).
			  *  Use only when you know the image will be soon be assigned another image.
			  *  Example of usage:
			  *   \code
			  *    CImage myImg(UNINITIALIZED_IMAGE);
			  *   \endcode
			  */
			inline CImage(TConstructorFlags_CImage ) : img(NULL),m_imgIsReadOnly(false), m_imgIsExternalStorage(false)
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
				MRPT_UNUSED_PARAM(constructor_flag);
				if( other_img.isColor() ) other_img.grayscale(*this);
				else this->setFromImageReadOnly(other_img);
			}

			/** Constructor from an IPLImage*, making a copy of the image.
			  * \sa loadFromIplImage, setFromIplImage
			  */
			CImage( void *iplImage );

			/** Explicit constructor from a matrix, interpreted as grayscale intensity values, in the range [0,1] (normalized=true) or [0,255] (normalized=false)
			  * \sa setFromMatrix
			  */
			template <typename Derived>
			explicit inline CImage(const Eigen::MatrixBase<Derived> &m, bool matrix_is_normalized) : img(NULL),m_imgIsReadOnly(false), m_imgIsExternalStorage(false)
			{
				this->setFromMatrix(m,matrix_is_normalized);
			}


			/** Destructor: */
			virtual ~CImage( );

			/** @} */
			// ================================================================

			/** @name Serialization format global flags
			    @{ */

			/** By default, when storing images through the CSerializable interface, grayscale images will be ZIP compressed if they are larger than 16Kb: this flag can be turn on to disable ZIP compression and gain speed versus occupied space.
			  *  (Default = false) */
			static bool DISABLE_ZIP_COMPRESSION;

			/** By default, when storing images through the CSerializable interface, RGB images are JPEG-compressed to save space. If for some reason you prefer storing RAW image data, disable this feature by setting this flag to true.
			  *  (Default = false) */
			static bool DISABLE_JPEG_COMPRESSION;

			/** Unless DISABLE_JPEG_COMPRESSION=true, this sets the JPEG quality (range 1-100) of serialized RGB images.
			  *  (Default = 95) */
			static int SERIALIZATION_JPEG_QUALITY;

			/** @} */

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
			void  setPixel(int x, int y, size_t color) MRPT_OVERRIDE;

			/** Changes the property of the image stating if the top-left corner (vs. bottom-left) is the coordinate reference */
			void  setOriginTopLeft(bool val);

			/** Draws a line.
			  * \param x0 The starting point x coordinate
			  * \param y0 The starting point y coordinate
			  * \param x1 The end point x coordinate
			  * \param y1 The end point y coordinate
			  * \param color The color of the line
			  * \param width The desired width of the line (this is IGNORED in this virtual class)
			  *  This method may be redefined in some classes implementing this interface in a more appropiate manner.
			  */
			void  line(
				int x0, int y0,
				int x1, int y1,
				const mrpt::utils::TColor color,
				unsigned int	width = 1,
				TPenStyle		penStyle = psSolid) MRPT_OVERRIDE;

			/** Draws a circle of a given radius.
			  * \param x The center - x coordinate in pixels.
			  * \param y The center - y coordinate in pixels.
			  * \param radius The radius - in pixels.
			  * \param color The color of the circle.
			  * \param width The desired width of the line
			  */
			void drawCircle(
				int				x,
				int				y,
				int				radius,
				const mrpt::utils::TColor &color = mrpt::utils::TColor(255,255,255),
				unsigned int	width = 1) MRPT_OVERRIDE;

			void equalizeHistInPlace(); //!< Equalize the image histogram, replacing the original image. \note RGB images are first converted to HSV color space, then equalized for brightness (V)
			void equalizeHist( CImage  &outImg ) const; //!< Equalize the image histogram, saving the new image in the given output object.  \note RGB images are first converted to HSV color space, then equalized for brightness (V)

			/** Returns a new image scaled down to half its original size.
			  * \exception std::exception On odd size
			  * \sa scaleDouble, scaleImage, scaleHalfSmooth
			  */
			CImage  scaleHalf()const
			{
				CImage ret(UNINITIALIZED_IMAGE);
				this->scaleHalf(ret);
				return ret;
			}

			//! \overload
			void scaleHalf(CImage &out_image) const;


			/** Returns a new image scaled down to half its original size (averaging between every two rows)
			  * \exception std::exception On odd size
			  * \sa scaleDouble, scaleImage, scaleHalf
			  */
			CImage  scaleHalfSmooth()const
			{
				CImage ret(UNINITIALIZED_IMAGE);
				this->scaleHalfSmooth(ret);
				return ret;
			}

			//! \overload
			void scaleHalfSmooth(CImage &out_image) const;


			/** Returns a new image scaled up to double its original size.
			  * \exception std::exception On odd size
			  * \sa scaleHalf, scaleImage
			  */
			CImage  scaleDouble()const
                        {
                                CImage ret(UNINITIALIZED_IMAGE);
                                this->scaleDouble(ret);
                                return ret;
                        }

                       //! \overload
                       void scaleDouble(CImage &out_image) const;


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


			/** Optimize the brightness range of an image without using histogram
			  * Only for one channel images.
			  * \sa equalizeHist
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
			  * \sa mrpt::vision::CUndistortMap
			  */
			void rectifyImage( CImage &out_img, const mrpt::utils::TCamera &cameraParams) const;

			/** Rectify (un-distort) the image according to a certain camera matrix and vector of distortion coefficients, replacing "this" with the rectified image
			  * \param cameraParams The input camera params (containing the intrinsic and distortion parameters of the camera)
			  * \sa mrpt::vision::CUndistortMap
			  */
			void rectifyImageInPlace(const mrpt::utils::TCamera &cameraParams );

			/** Rectify an image (undistorts and rectification) from a stereo pair according to a pair of precomputed rectification maps
			  * \param mapX, mapY   [IN] The pre-computed maps of the rectification (should be computed beforehand)
			  * \sa mrpt::vision::CStereoRectifyMap, mrpt::vision::computeStereoRectificationMaps
			  */
            void rectifyImageInPlace( void *mapX, void *mapY );

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
				unsigned int  check_size_y,
				unsigned int  lines_width = 1,
				unsigned int  circles_radius = 4
				);

			/** Joins two images side-by-side horizontally. Both images must have the same number of rows and be of the same type (i.e. depth and color mode)
			  *
			  * \param im1 [IN] The first image.
			  * \param im2 [IN] The other image.
			  */
			void joinImagesHorz(
				const CImage &im1,
				const CImage &im2 );

			/** Compute the KLT response at a given pixel (x,y) - Only for grayscale images (for efficiency it avoids converting to grayscale internally).
			  *  See KLT_response_optimized for more details on the internal optimizations of this method, but this graph shows a general view:
			  *  <img src="KLT_response_performance_SSE2.png" >
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

			/** Copy operator (if the image is externally stored, the writen image will be such as well).
			  * \sa copyFastFrom
			  */
			CImage& operator = (const CImage& o);

			/** Copies from another image, and, if that one is externally stored, the image file will be actually loaded into memory in "this" object.
			  * \sa operator =
			  * \exception CExceptionExternalImageNotFound If the external image couldn't be loaded.
			  */
			void copyFromForceLoad(const CImage &o);

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

			/** Returns a pointer to a const T* containing the image - the idea is to call like "img.getAs<IplImage>()" so we can avoid here including OpenCV's headers. */
			template <typename T> inline const T* getAs() const {
				makeSureImageIsLoaded();
				return static_cast<const T*>(img);
			}
			/** Returns a pointer to a T* containing the image - the idea is to call like "img.getAs<IplImage>()" so we can avoid here including OpenCV's headers. */
			template <typename T> inline T* getAs(){
				makeSureImageIsLoaded();
				return static_cast<T*>(img);
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

			size_t getWidth() const MRPT_OVERRIDE; //!< Returns the width of the image in pixels \sa getSize
			size_t getHeight() const MRPT_OVERRIDE; //!< Returns the height of the image in pixels \sa getSize

			void getSize(TImageSize &s) const; //!< Return the size of the image \sa getWidth, getHeight
			/** Return the size of the image \sa getWidth, getHeight */
			inline TImageSize getSize() const {
				TImageSize  ret;
				getSize(ret);
				return ret;
			}

			/** Returns the row stride of the image: this is the number of *bytes* between two consecutive rows. You can access the pointer to the first row with get_unsafe(0,0)
			  * \sa getSize, get_unsafe */
			size_t getRowStride() const;

			/** Return the maximum pixel value of the image, as a float value in the range [0,1]
			  * \sa getAsFloat */
			float  getMaxAsFloat() const;

			/** Returns true if the image is RGB, false if it is grayscale */
			bool  isColor() const;

			/** Returns true if the coordinates origin is top-left, or false if it is bottom-left  */
			bool  isOriginTopLeft() const;

			/** Returns a string of the form "BGR","RGB" or "GRAY" indicating the channels ordering. \sa setChannelsOrder, swapRB */
			const char *  getChannelsOrder()const;

			/** Marks the channel ordering in a color image as "RGB" (this doesn't actually modify the image data, just the format description) \sa getChannelsOrder, swapRB */
			void setChannelsOrder_RGB();
			/** Marks the channel ordering in a color image as "BGR" (this doesn't actually modify the image data, just the format description) \sa getChannelsOrder, swapRB */
			void setChannelsOrder_BGR();

			/** Returns the number of channels, typically 1 (GRAY) or 3 (RGB)
			  * \sa isColor
			  */
			TImageChannels getChannelCount() const;

			/**	Returns the image as a matrix with pixel grayscale values in the range [0,1]. Matrix indexes in this order: M(row,column)
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

			/**	Returns the image as RGB matrices with pixel values in the range [0,1]. Matrix indexes in this order: M(row,column)
			  *  \param doResize If set to true (default), the output matrix will be always the size of the image at output. If set to false, the matrix will be enlarged to the size of the image, but it will not be cropped if it has room enough (useful for FFT2D,...)
			  *  \param x_min The starting "x" coordinate to extract (default=0=the first column)
			  *  \param y_min The starting "y" coordinate to extract (default=0=the first row)
			  *  \param x_max The final "x" coordinate (inclusive) to extract (default=-1=the last column)
			  *  \param y_max The final "y" coordinate (inclusive) to extract (default=-1=the last row)
			  * \sa setFromRGBMatrices
			  */
			void  getAsRGBMatrices(
				mrpt::math::CMatrixFloat	&outMatrixR,
				mrpt::math::CMatrixFloat	&outMatrixG,
				mrpt::math::CMatrixFloat	&outMatrixB,
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

			/** For external storage image objects only, this method makes sure the image is loaded in memory. Note that usually images are loaded on-the-fly on first access and there's no need to call this.
			  * \unload
			  */
			inline void forceLoad() const {  makeSureImageIsLoaded(); }

			/** For external storage image objects only, this method unloads the image from memory (or does nothing if already unloaded).
			  *  It does not need to be called explicitly, unless the user wants to save memory for images that will not be used often.
			  *  If called for an image without the flag "external storage", it is simply ignored.
			  * \sa setExternalStorage, forceLoad
			  */
			void unload()  const MRPT_NO_THROWS;

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

			/** Reads the image from a OpenCV IplImage object (making a COPY).
			  */
			void  loadFromIplImage( void* iplImage );

			/** Reads the image from a OpenCV IplImage object (WITHOUT making a copy).
			  *   This object will own the memory of the passed object and free the IplImage upon destruction,
			  *     so the caller CAN'T free the original object.
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
			inline void setFromImageReadOnly( const CImage &other_img ) { setFromIplImageReadOnly(const_cast<void*>(other_img.getAs<void>()) ); }

			/** Set the image from a matrix, interpreted as grayscale intensity values, in the range [0,1] (normalized=true) or [0,255] (normalized=false)
			  *	Matrix indexes are assumed to be in this order: M(row,column)
			  * \sa getAsMatrix
			  */
			template <typename Derived>
			void setFromMatrix(const Eigen::MatrixBase<Derived> &m, bool matrix_is_normalized=true)
			{
				MRPT_START
				const unsigned int lx = m.cols();
				const unsigned int ly = m.rows();
				this->changeSize(lx,ly,1,true);
				if (matrix_is_normalized) {  // Matrix: [0,1]
					for (unsigned int y=0;y<ly;y++) {
						unsigned char *pixels = this->get_unsafe(0,y,0);
						for (unsigned int x=0;x<lx;x++)
							(*pixels++) = static_cast<unsigned char>( m.get_unsafe(y,x) * 255 );
					}
				}
				else {  // Matrix: [0,255]
					for (unsigned int y=0;y<ly;y++) {
						unsigned char *pixels = this->get_unsafe(0,y,0);
						for (unsigned int x=0;x<lx;x++)
							(*pixels++) = static_cast<unsigned char>( m.get_unsafe(y,x) );
					}
				}
				MRPT_END
			}

			/** Set the image from RGB matrices, given the pixels in the range [0,1] (normalized=true) or [0,255] (normalized=false)
			  * Matrix indexes are assumed to be in this order: M(row,column)
			  * \sa getAsRGBMatrices
			  */
			template <typename Derived>
			void setFromRGBMatrices(const Eigen::MatrixBase<Derived> &m_r, const Eigen::MatrixBase<Derived> &m_g, const Eigen::MatrixBase<Derived> &m_b, bool matrix_is_normalized=true)
			{
				MRPT_START
				makeSureImageIsLoaded();   // For delayed loaded images stored externally
				ASSERT_(img);
				ASSERT_((m_r.size() == m_g.size())&&(m_r.size() == m_b.size()));
				const unsigned int lx = m_r.cols();
				const unsigned int ly = m_r.rows();
				this->changeSize(lx,ly,3,true);
				this->setChannelsOrder_RGB();

				if (matrix_is_normalized) {  // Matrix: [0,1]
					for (unsigned int y=0;y<ly;y++) {
						unsigned char *pixels = this->get_unsafe(0,y,0);
						for (unsigned int x=0;x<lx;x++)
						{
							(*pixels++) = static_cast<unsigned char>( m_r.get_unsafe(y,x) * 255 );
							(*pixels++) = static_cast<unsigned char>( m_g.get_unsafe(y,x) * 255 );
							(*pixels++) = static_cast<unsigned char>( m_b.get_unsafe(y,x) * 255 );
						}
					}
				}
				else {  // Matrix: [0,255]
					for (unsigned int y=0;y<ly;y++) {
						unsigned char *pixels = this->get_unsafe(0,y,0);
						for (unsigned int x=0;x<lx;x++)
						{
							(*pixels++) = static_cast<unsigned char>( m_r.get_unsafe(y,x) );
							(*pixels++) = static_cast<unsigned char>( m_g.get_unsafe(y,x) );
							(*pixels++) = static_cast<unsigned char>( m_b.get_unsafe(y,x) );
						}
					}
				}
				MRPT_END
			}

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
			 * \sa saveToFile, setExternalStorage,loadFromXPM, loadTGA
			 */
			bool  loadFromFile( const std::string& fileName, int isColor = -1  );

			/** Loads a TGA true-color RGBA image as two CImage objects, one for the RGB channels plus a separate gray-level image with A channel.
			  * \return true on success
			  */
			static bool  loadTGA(const std::string& fileName, mrpt::utils::CImage &out_RGB, mrpt::utils::CImage &out_alpha);

			/** Loads the image from an XPM array, as #include'd from a ".xpm" file.
			  * \param[in] swap_rb Swaps red/blue channels from loaded image. *Seems* to be always needed, so it's enabled by default.
			  * \sa loadFromFile
			  * \return false on any error */
			bool loadFromXPM( const char** xpm_array, bool swap_rb = true );

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

			/** Save image to binary stream as a JPEG (.jpg) compressed format.
			 * \exception std::exception On number of rows or cols equal to zero or other errors.
			 * \sa saveToJPEG
			 */
			void  saveToStreamAsJPEG(mrpt::utils::CStream &out, const int jpeg_quality = 95 ) const;

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
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CImage, mrpt::utils::CSerializable )

	} // end of namespace utils

} // end of namespace mrpt

#endif
