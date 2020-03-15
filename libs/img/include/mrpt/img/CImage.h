/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/pimpl.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/serialization/CSerializable.h>

// Forwards decls:
// clang-format off
struct _IplImage;
using IplImage = struct _IplImage;
namespace cv { class Mat; }
namespace mrpt::io { class CStream; }
// clang-format on

// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM(mrpt::img::CImage)

namespace mrpt::img
{
enum class PixelDepth : int32_t
{
	D8U = 0,
	D8S = 1,
	D16U = 2,
	D16S = 3,
	D32S = 4,
	D32F = 5,
	D64F = 6
};

/** Interpolation methods for images.
 *  Used for OpenCV related operations with images, but also with MRPT native
 * classes.
 * \sa mrpt::img::CMappedImage, CImage::scaleImage
 * \note These are numerically compatible to cv::InterpolationFlags
 * \ingroup mrpt_img_grp
 */
enum TInterpolationMethod
{
	IMG_INTERP_NN = 0,
	IMG_INTERP_LINEAR = 1,
	IMG_INTERP_CUBIC = 2,
	IMG_INTERP_AREA = 3
};

/** For use in mrpt::img::CImage */
enum TImageChannels : uint8_t
{
	CH_GRAY = 1,
	CH_RGB = 3
};

/** For usage in one of the CImage constructors */
enum ctor_CImage_ref_or_gray
{
	FAST_REF_OR_CONVERT_TO_GRAY = 1
};

/** Define kind of copies  */
enum copy_type_t
{
	/** Shallow copy: the copied object is a reference to the original one */
	SHALLOW_COPY = 0,
	/** Deep copy: the copied object has a duplicate of all data, becoming
	   independent  */
	DEEP_COPY = 1
};

/** Used in mrpt::img::CImage */
class CExceptionExternalImageNotFound : public std::runtime_error
{
   public:
	CExceptionExternalImageNotFound(const std::string& s);
};

/** A class for storing images as grayscale or RGB bitmaps.
 * I/O is supported as:
 * - Binary dump using the CSerializable interface(<< and >> operators),
 *just as most objects in MRPT. This format is not compatible with any
 *standarized image format but it is fast.
 * - Saving/loading from files of different formats (bmp,jpg,png,...) using
 *the methods CImage::loadFromFile and CImage::saveToFile. See OpenCV for the
 *list of supported formats.
 * - Importing from an XPM array (.xpm file format) using CImage::loadFromXPM()
 * - Importing TGA images. See CImage::loadTGA()
 *
 *  How to create color/grayscale images:
 *  \code
 *    CImage  img1(width, height,  CH_GRAY );  // Grayscale image (8U1C)
 *    CImage  img2(width, height,  CH_RGB );  // RGB image (8U3C)
 *  \endcode
 *
 * Additional notes:
 * - The OpenCV `cv::Mat` format is used internally for compatibility with
 * all OpenCV functions. Use CImage::asCvMat() to retrieve it. Example:
 * \code
 * CImage  img;
 * ...
 * cv::Mat m = img.asCvMat()
 * \endcode
 * - By default, all images use unsigned 8-bit storage format for pixels (on
 *each channel), but it can be changed by flags in the constructor.
 * - An **external storage mode** can be enabled by calling
 *CImage::setExternalStorage, useful for storing large collections of image
 *objects in memory while loading the image data itself only for the relevant
 *images at any time. See CImage::forceLoad() and CImage::unload().
 * - Operator = and copy ctor make shallow copies. For deep copies, see
 * CImage::makeDeepCopy() or CImage(const CImage&, copy_type_t), e.g:
 * \code
 * CImage a(20, 10, CH_GRAY);
 * // Shallow copy ctor:
 * CImage b(a, mrpt::img::SHALLOW_COPY);
 * CImage c(a, mrpt::img::DEEP_COPY);
 * \endcode
 * - If you are interested in a smart pointer to an image, use:
 * \code
 * CImage::Ptr myImg = CImage::Create(); // optional ctor arguments
 * // or:
 * CImage::Ptr myImg = std::make_shared<CImage>(...);
 *  \endcode
 * - To set a CImage from an OpenCV `cv::Mat` use
 *CImage::CImage(cv::Mat,copy_type_t).
 *
 * Some functions are implemented in MRPT with highly optimized SSE2/SSE3
 *routines, in suitable platforms and compilers. To see the list of
 * optimizations refer to \ref sse_optimizations, falling back to default OpenCV
 *methods where unavailable.
 *
 * For computer vision functions that use CImage as its image data type,
 *see mrpt::vision.
 *
 * \sa mrpt::vision, mrpt::vision::CFeatureExtractor,
 *mrpt::vision::CImagePyramid, CSerializable, CCanvas
 * \ingroup mrpt_img_grp
 */
class CImage : public mrpt::serialization::CSerializable, public CCanvas
{
	DEFINE_SERIALIZABLE(CImage, mrpt::img)

	// This must be added for declaration of MEX-related functions
	DECLARE_MEX_CONVERSION

   public:
	/** @name Constructors & destructor
		@{ */

	/** Default constructor: initialize to empty image. It's an error trying to
	 * access the image in such a state (except reading the image width/height,
	 * which are both zero). Either call resize(), assign from another image,
	 * load from disk, deserialize from an archive, etc. to properly initialize
	 * the image.
	 */
	CImage();

	/** Constructor for a given image size and type.
	 *  Examples:
	 *   \code
	 *    CImage  img1(width, height,  CH_GRAY );  // Grayscale image (8U1C)
	 *    CImage  img2(width, height,  CH_RGB );  // RGB image (8U3C)
	 *   \endcode
	 */
	CImage(
		unsigned int width, unsigned int height,
		TImageChannels nChannels = CH_RGB);

	/** Fast constructor of a grayscale version of another image, making a
	 * **shallow copy** from the original image if it already was grayscale, or
	 * otherwise creating a new grayscale image and converting the original
	 * image into it.
	 * Example of usage:
	 *   \code
	 *     void my_func(const CImage &in_img) {
	 *        const CImage gray_img(in_img, FAST_REF_OR_CONVERT_TO_GRAY);
	 *        // We can now operate on "gray_img" being sure it's in grayscale.
	 *     }
	 *   \endcode
	 */
	inline CImage(const CImage& other_img, ctor_CImage_ref_or_gray)
	{
		if (other_img.isColor())
		{
			*this = CImage();
			other_img.grayscale(*this);
		}
		else
		{
			// shallow copy
			*this = other_img;
		}
	}

	/** Constructor from a cv::Mat image, making or not a deep copy of the data
	 */
	CImage(const cv::Mat& img, copy_type_t copy_type);

	/** Constructor from another CImage, making or not a deep copy of the data
	 */
	CImage(const CImage& img, copy_type_t copy_type);

	/** @} */

	/** @name Behavior-changing global flags
		@{ */

	/** By default, when storing images through the CSerializable interface,
	 * grayscale images will be ZIP compressed if they are larger than 16Kb:
	 * this flag can be turn on to disable ZIP compression and gain speed versus
	 * occupied space.
	 *  (Default = false) */
	static void DISABLE_ZIP_COMPRESSION(bool val);
	static bool DISABLE_ZIP_COMPRESSION();

	/** By default, when storing images through the CSerializable interface, RGB
	 * images are JPEG-compressed to save space. If for some reason you prefer
	 * storing RAW image data, disable this feature by setting this flag to
	 * true.
	 *  (Default = true) */
	static void DISABLE_JPEG_COMPRESSION(bool val);
	static bool DISABLE_JPEG_COMPRESSION();

	/** Unless DISABLE_JPEG_COMPRESSION=true, this sets the JPEG quality (range
	 * 1-100) of serialized RGB images.
	 *  (Default = 95) */
	static void SERIALIZATION_JPEG_QUALITY(int q);
	static int SERIALIZATION_JPEG_QUALITY();

	/** @} */

	/** @name Manipulate the image contents or size, various computer-vision
	   methods (image filters, undistortion, etc.)
		@{ */

	/** Resets the image to the state after a default ctor. Accessing the image
	 * after will throw an exception, unless it is formerly initialized somehow:
	 * loading an image from disk, calling rezize(), etc. */
	void clear();

	/** Changes the size of the image, erasing previous contents (does NOT scale
	 * its current content, for that, see scaleImage).
	 *  - nChannels: Can be 3 for RGB images or 1 for grayscale images.
	 * \sa scaleImage
	 */
	void resize(
		std::size_t width, std::size_t height, TImageChannels nChannels,
		PixelDepth depth = PixelDepth::D8U);

	PixelDepth getPixelDepth() const;

	/** Scales this image to a new size, interpolating as needed, saving the new
	 * image in a different output object, or operating in-place if
	 * `out_img==this`. \sa resize, rotateImage
	 */
	void scaleImage(
		CImage& out_img, unsigned int width, unsigned int height,
		TInterpolationMethod interp = IMG_INTERP_CUBIC) const;

	/** Rotates the image by the given angle around the given center point, with
	 * an optional scale factor.
	 * \sa resize, scaleImage
	 */
	void rotateImage(
		CImage& out_img, double ang, unsigned int cx, unsigned int cy,
		double scale = 1.0) const;

	/** Changes the value of the pixel (x,y).
	 *  Pixel coordinates starts at the left-top corner of the image, and start
	 * in (0,0).
	 *  The meaning of the parameter "color" depends on the implementation: it
	 * will usually
	 *   be a 24bit RGB value (0x00RRGGBB), but it can also be just a 8bit gray
	 * level.
	 *  This method must support (x,y) values OUT of the actual image size
	 * without neither
	 *   raising exceptions, nor leading to memory access errors.
	 * \sa at, ptr
	 */
	void setPixel(int x, int y, size_t color) override;

	// See CCanvas docs
	void line(
		int x0, int y0, int x1, int y1, const mrpt::img::TColor color,
		unsigned int width = 1, TPenStyle penStyle = psSolid) override;

	// See CCanvas docs
	void drawCircle(
		int x, int y, int radius,
		const mrpt::img::TColor& color = mrpt::img::TColor(255, 255, 255),
		unsigned int width = 1) override;

	// See CCanvas docs
	void drawImage(int x, int y, const mrpt::img::CImage& img) override;

	/** Equalize the image histogram, saving the new image in the given output
	 * object.  \note RGB images are first converted to HSV color space, then
	 * equalized for brightness (V) */
	void equalizeHist(CImage& out_img) const;

	/** Returns a new image scaled down to half its original size
	 * \exception std::exception On odd size
	 * \sa scaleDouble, scaleImage, scaleHalfSmooth
	 */
	inline CImage scaleHalf(TInterpolationMethod interp) const
	{
		CImage ret;
		this->scaleHalf(ret, interp);
		return ret;
	}

	/** \overload
	 *  \return true if an optimized SSE2/SSE3 version could be used. */
	bool scaleHalf(CImage& out_image, TInterpolationMethod interp) const;

	/** Returns a new image scaled up to double its original size.
	 * \exception std::exception On odd size
	 * \sa scaleHalf, scaleImage
	 */
	inline CImage scaleDouble(TInterpolationMethod interp) const
	{
		CImage ret;
		this->scaleDouble(ret, interp);
		return ret;
	}

	//! \overload
	void scaleDouble(CImage& out_image, TInterpolationMethod interp) const;

	/** Update a part of this image with the "patch" given as argument.
	 * The "patch" will be "pasted" at the (col,row) coordinates of this image.
	 * \exception std::exception if patch pasted on the pixel (_row, _column)
	 * jut out
	 * of the image.
	 * \sa extract_patch
	 */
	void update_patch(
		const CImage& patch, const unsigned int col, const unsigned int row);

	/** Extract a patch from this image, saveing it into "patch" (its previous
	 * contents will be overwritten).
	 *  The patch to extract starts at (col,row) and has the given dimensions.
	 * \sa update_patch
	 */
	void extract_patch(
		CImage& patch, const unsigned int col = 0, const unsigned int row = 0,
		const unsigned int width = 1, const unsigned int height = 1) const;

	/** Computes the correlation coefficient (returned as val), between two
	 *images
	 *	This function use grayscale images only
	 *	img1, img2 must be same size
	 * (by AJOGD @ DEC-2006)
	 */
	float correlate(
		const CImage& img2int, int width_init = 0, int height_init = 0) const;

	/**	Computes the correlation matrix between this image and another one.
	 *   This implementation uses the 2D FFT for achieving reduced computation
	 * time.
	 * \param in_img The "patch" image, which must be equal, or smaller than
	 * "this" image. This function supports gray-scale (1 channel only) images.
	 * \param u_search_ini The "x" coordinate of the search window.
	 * \param v_search_ini The "y" coordinate of the search window.
	 * \param u_search_size The width of the search window.
	 * \param v_search_size The height of the search window.
	 * \param out_corr The output for the correlation matrix, which will be
	 * "u_search_size" x "v_search_size"
	 * \param biasThisImg This optional parameter is a fixed "bias" value to be
	 * substracted to the pixels of "this" image before performing correlation.
	 * \param biasInImg This optional parameter is a fixed "bias" value to be
	 * substracted to the pixels of "in_img" image before performing
	 * correlation. Note: By default, the search area is the whole (this) image.
	 * (by JLBC @ JAN-2006)
	 * \sa cross_correlation
	 */
	void cross_correlation_FFT(
		const CImage& in_img, math::CMatrixFloat& out_corr,
		int u_search_ini = -1, int v_search_ini = -1, int u_search_size = -1,
		int v_search_size = -1, float biasThisImg = 0,
		float biasInImg = 0) const;

	/** Optimize the brightness range of an image without using histogram
	 * Only for one channel images.
	 * \sa equalizeHist
	 */
	void normalize();

	/** Flips the image vertically. \sa swapRB(), flipHorizontal() */
	void flipVertical();
	/** Flips the image horizontally \sa swapRB(), flipVertical() */
	void flipHorizontal();

	/** Swaps red and blue channels. */
	void swapRB();

	/** Undistort the image according to some camera parameters, and
	 * returns an output undistorted image.
	 * \param out_img The output undistorted image
	 * \param cameraParams The input camera params (containing the intrinsic
	 * and distortion parameters of the camera)
	 * \note The intrinsic parameters (fx,fy,cx,cy) of the output image are the
	 * same than in the input image.
	 * \sa mrpt::vision::CUndistortMap
	 */
	void undistort(
		CImage& out_img, const mrpt::img::TCamera& cameraParams) const;

	/** Rectify an image (undistorts and rectification) from a stereo pair
	 * according to a pair of precomputed rectification maps
	 * \param mapX, mapY   [IN] The pre-computed maps of the rectification
	 * (should be computed beforehand)
	 * \sa mrpt::vision::CStereoRectifyMap,
	 * mrpt::vision::computeStereoRectificationMaps
	 */
	void rectifyImageInPlace(void* mapX, void* mapY);

	/** Filter the image with a Median filter with a window size WxW, returning
	 * the filtered image in out_img. For inplace operation, set out_img to
	 * this. */
	void filterMedian(CImage& out_img, int W = 3) const;

	/** Filter the image with a Gaussian filter with a window size WxH,
	 * replacing "this" image by the filtered one. For inplace operation, set
	 * out_img to this. */
	void filterGaussian(
		CImage& out_img, int W = 3, int H = 3, double sigma = 1.0) const;

	/** Draw onto this image the detected corners of a chessboard. The length of
	 * cornerCoords must be the product of the two check_sizes.
	 *
	 * \param cornerCoords [IN] The pixel coordinates of all the corners.
	 * \param check_size_x [IN] The number of squares, in the X direction
	 * \param check_size_y [IN] The number of squares, in the Y direction
	 *
	 * \return false if the length of cornerCoords is inconsistent (nothing is
	 * drawn then).
	 *
	 * \sa mrpt::vision::findChessboardCorners
	 */
	bool drawChessboardCorners(
		std::vector<TPixelCoordf>& cornerCoords, unsigned int check_size_x,
		unsigned int check_size_y, unsigned int lines_width = 1,
		unsigned int circles_radius = 4);

	/** Joins two images side-by-side horizontally. Both images must have the
	 * same number of rows and be of the same type (i.e. depth and color mode)
	 *
	 * \param im1 [IN] The first image.
	 * \param im2 [IN] The other image.
	 */
	void joinImagesHorz(const CImage& im1, const CImage& im2);

	/** Compute the KLT response at a given pixel (x,y) - Only for grayscale
	 * images (for efficiency it avoids converting to grayscale internally).
	 *  See KLT_response() for more details on the internal
	 * optimizations of this method, but this graph shows a general view:
	 *  <img src="KLT_response_performance_SSE2.png" >
	 */
	float KLT_response(
		const unsigned int x, const unsigned int y,
		const unsigned int half_window_size) const;

	/** @} */

	/** @name Copy, move & swap operations
		@{ */
	[[deprecated("Use makeShallowCopy() instead")]] inline void
		setFromImageReadOnly(const CImage& o)
	{
		*this = o.makeShallowCopy();
	}

	/** Returns a shallow copy of the original image */
	inline CImage makeShallowCopy() const
	{
		CImage r = *this;
		return r;
	}

	/** Returns a deep copy of this image.
	 * If the image is externally-stored, there is no difference with a shallow
	 * copy. \sa makeShallowCopy() */
	CImage makeDeepCopy() const;

	/** Copies from another image (shallow copy), and, if it is externally
	 * stored, the image file will be actually loaded into memory in "this"
	 * object. \sa operator = \exception CExceptionExternalImageNotFound If the
	 * external image couldn't be loaded.
	 */
	void copyFromForceLoad(const CImage& o);

	/** Moves an image from another object, erasing the origin image in the
	 * process.
	 * \sa operator =
	 */
	[[deprecated("Use a=std::move(b); instead ")]] inline void copyFastFrom(
		CImage& o)
	{
		*this = std::move(o);
	}

	/** Assigns from an image in IplImage format */
	inline void loadFromIplImage(
		const IplImage* iplImage, copy_type_t c = DEEP_COPY)
	{
		internal_fromIPL(iplImage, c);
	}

	[[deprecated(
		"Prefer a ctor from a cv::Mat instead or use loadFromIplImage() "
		"explicitly specifying the kind of copy to be done")]] inline void
		setFromIplImageReadOnly(IplImage* iplImage)
	{
		internal_fromIPL(iplImage, SHALLOW_COPY);
	}

	/** Efficiently swap of two images */
	void swap(CImage& o);

	/** @} */

	/** @name Access to image contents (OpenCV data structure, and raw pixels)
		@{ */

	/** Makes a shallow or deep copy of this image into the provided cv::Mat.
	 * \sa asCvMatRef */
	void asCvMat(cv::Mat& out_img, copy_type_t copy_type) const;

	template <typename CV_MAT>
	CV_MAT asCvMat(copy_type_t copy_type) const
	{
		CV_MAT ret;
		asCvMat(ret, copy_type);
		return ret;
	}

	/** Get a reference to the internal cv::Mat, which can be resized, etc. and
	 * changes will be reflected in this CImage object. */
	cv::Mat& asCvMatRef();

	/** \overload  */
	const cv::Mat& asCvMatRef() const;

	/**  Access to pixels without checking boundaries - Use normally the ()
	  operator better, which checks the coordinates.
	  \sa CImage::operator()
	  */
	[[deprecated("Use at<>(), ptr<>() or ptrLine() instead ")]] uint8_t*
		get_unsafe(
			unsigned int col, unsigned int row, uint8_t channel = 0) const;

	/**  Access to pixels without checking boundaries, and doing a
	 * reinterpret_cast<> of the data as the given type.
	 *\sa The CImage::operator() which does check for coordinate limits.
	 */
	template <typename T>
	const T& at(
		unsigned int col, unsigned int row, unsigned int channel = 0) const
	{
		return *reinterpret_cast<const T*>(internal_get(col, row, channel));
	}
	/** \overload Non-const case */
	template <typename T>
	T& at(unsigned int col, unsigned int row, unsigned int channel = 0)
	{
		return *reinterpret_cast<T*>(internal_get(col, row, channel));
	}

	/** Returns a pointer to a given pixel, without checking for boundaries.
	 *\sa The CImage::operator() which does check for coordinate limits.
	 */
	template <typename T>
	const T* ptr(
		unsigned int col, unsigned int row, unsigned int channel = 0) const
	{
		return reinterpret_cast<const T*>(internal_get(col, row, channel));
	}
	/** \overload Non-const case */
	template <typename T>
	T* ptr(unsigned int col, unsigned int row, unsigned int channel = 0)
	{
		return reinterpret_cast<T*>(internal_get(col, row, channel));
	}

	/** Returns a pointer to the first pixel of the given line.\sa ptr, at */
	template <typename T>
	const T* ptrLine(unsigned int row) const
	{
		return reinterpret_cast<const T*>(internal_get(0, row, 0));
	}
	/** \overload Non-const case */
	template <typename T>
	T* ptrLine(unsigned int row)
	{
		return reinterpret_cast<T*>(internal_get(0, row, 0));
	}

	/** Returns the contents of a given pixel at the desired channel, in float
	 * format: [0,255]->[0,1]
	 *   The coordinate origin is pixel(0,0)=top-left corner of the image.
	 * \exception std::exception On pixel coordinates out of bounds
	 * \sa operator()
	 */
	float getAsFloat(
		unsigned int col, unsigned int row, unsigned int channel) const;

	/** Returns the contents of a given pixel (for gray-scale images, in color
	 * images the gray scale equivalent is computed for the pixel), in float
	 * format: [0,255]->[0,1]
	 *   The coordinate origin is pixel(0,0)=top-left corner of the image.
	 * \exception std::exception On pixel coordinates out of bounds
	 * \sa operator()
	 */
	float getAsFloat(unsigned int col, unsigned int row) const;

	/** Returns a pointer to a given pixel information.
	 *   The coordinate origin is pixel(0,0)=top-left corner of the image.
	 * \exception std::exception On pixel coordinates out of bounds
	 */
	unsigned char* operator()(
		unsigned int col, unsigned int row, unsigned int channel = 0) const;

	/** @} */

	/** @name Query image properties
		@{ */

	/** Returns the width of the image in pixels \sa getSize */
	size_t getWidth() const override;
	/** Returns the height of the image in pixels \sa getSize */
	size_t getHeight() const override;

	/** Return the size of the image \sa getWidth, getHeight */
	void getSize(TImageSize& s) const;
	/** Return the size of the image \sa getWidth, getHeight */
	inline TImageSize getSize() const
	{
		TImageSize ret;
		getSize(ret);
		return ret;
	}

	/** Returns the row stride of the image: this is the number of *bytes*
	 * between two consecutive rows. You can access the pointer to the first row
	 * with ptrLine(0)
	 * \sa getSize, as, ptr, ptrLine */
	size_t getRowStride() const;

	/** As of mrpt 2.0.0, this returns either "GRAY" or "BGR". */
	std::string getChannelsOrder() const;

	/** Return the maximum pixel value of the image, as a float value in the
	 * range [0,1]
	 * \sa getAsFloat */
	float getMaxAsFloat() const;

	/** Returns true if the image is RGB, false if it is grayscale */
	bool isColor() const;

	/** Returns true if the object is in the state after default constructor */
	bool isEmpty() const;

	/** Returns true (as of MRPT v2.0.0, it's fixed) */
	bool isOriginTopLeft() const;

	/** Returns the number of channels, typically 1 (GRAY) or 3 (RGB)
	 * \sa isColor
	 */
	TImageChannels getChannelCount() const;

	/**	Returns the image as a matrix with pixel grayscale values in the range
	 * [0,1]. Matrix indexes in this order: M(row,column)
	 *  \param doResize If set to true (default), the output matrix will be
	 * always the size of the image at output. If set to false, the matrix will
	 * be enlarged to the size of the image, but it will not be cropped if it
	 * has room enough (useful for FFT2D,...)
	 *  \param x_min The starting "x" coordinate to extract (default=0=the
	 * first column)
	 *  \param y_min The starting "y" coordinate to extract (default=0=the
	 * first row)
	 *  \param x_max The final "x" coordinate (inclusive) to extract
	 * (default=-1=the last column)
	 *  \param y_max The final "y" coordinate (inclusive) to extract
	 * (default=-1=the last row)
	 * \param normalize_01 Normalize the image values such that they fall in the
	 * range [0,1] (default: true). If set to false, the matrix will hold
	 * numbers in the range [0,255]. \sa setFromMatrix
	 */
	void getAsMatrix(
		mrpt::math::CMatrixFloat& outMatrix, bool doResize = true,
		int x_min = 0, int y_min = 0, int x_max = -1, int y_max = -1,
		bool normalize_01 = true) const;

	/** \overload For uint8_t matrices [0, 255]. */
	void getAsMatrix(
		mrpt::math::CMatrix_u8& outMatrix, bool doResize = true, int x_min = 0,
		int y_min = 0, int x_max = -1, int y_max = -1) const;

	/**	Returns the image as RGB matrices with pixel values in the range [0,1].
	 * Matrix indexes in this order: M(row,column)
	 *  \param doResize If set to true (default), the output matrix will be
	 * always the size of the image at output. If set to false, the matrix will
	 * be enlarged to the size of the image, but it will not be cropped if it
	 * has room enough (useful for FFT2D,...)
	 *  \param x_min The starting "x" coordinate to extract (default=0=the
	 * first column)
	 *  \param y_min The starting "y" coordinate to extract (default=0=the
	 * first row)
	 *  \param x_max The final "x" coordinate (inclusive) to extract
	 * (default=-1=the last column)
	 *  \param y_max The final "y" coordinate (inclusive) to extract
	 * (default=-1=the last row)
	 * \sa setFromRGBMatrices
	 */
	void getAsRGBMatrices(
		mrpt::math::CMatrixFloat& outMatrixR,
		mrpt::math::CMatrixFloat& outMatrixG,
		mrpt::math::CMatrixFloat& outMatrixB, bool doResize = true,
		int x_min = 0, int y_min = 0, int x_max = -1, int y_max = -1) const;

	/** \overload For uint8_t matrices [0, 255]. */
	void getAsRGBMatrices(
		mrpt::math::CMatrix_u8& outMatrixR, mrpt::math::CMatrix_u8& outMatrixG,
		mrpt::math::CMatrix_u8& outMatrixB, bool doResize = true, int x_min = 0,
		int y_min = 0, int x_max = -1, int y_max = -1) const;

	/**	Returns the image as a matrix, where the image is "tiled" (repeated)
	 * the required number of times to fill the entire size of the matrix on
	 * input.
	 */
	void getAsMatrixTiled(mrpt::math::CMatrixFloat& outMatrix) const;

	/** @} */

	/** @name External storage-mode methods
		@{  */

	/**  By using this method the image is marked as referenced to an external
	 * file, which will be loaded only under demand.
	 *   A CImage with external storage does not consume memory until some
	 * method trying to access the image is invoked (e.g. getWidth(),
	 * isColor(),...)
	 *   At any moment, the image can be unloaded from memory again by invoking
	 * unload.
	 *   An image becomes of type "external storage" only through calling
	 * setExternalStorage. This property remains after serializing the object.
	 *   File names can be absolute, or relative to the
	 * CImage::getImagesPathBase() directory. Filenames staring with "X:\" or
	 * "/"
	 * are considered absolute paths.
	 *   By calling this method the current contents of the image are NOT saved
	 * to that file, because this method can be also called
	 *    to let the object know where to load the image in case its contents
	 * are required. Thus, for saving images in this format (not when loading)
	 *    the proper order of commands should be:
	 *   \code
	 *   img.saveToFile( fileName );
	 *   img.setExternalStorage( fileName );
	 *   \endcode
	 *
	 *   \note Modifications to the memory copy of the image are not
	 * automatically saved to disk.
	 *  \sa unload, isExternallyStored
	 */
	void setExternalStorage(const std::string& fileName) noexcept;

	/** By default, "."  \sa setExternalStorage */
	static const std::string& getImagesPathBase();
	static void setImagesPathBase(const std::string& path);

	/** See setExternalStorage(). */
	bool isExternallyStored() const noexcept { return m_imgIsExternalStorage; }
	/** Only if isExternallyStored() returns true. \sa
	 * getExternalStorageFileAbsolutePath */
	inline std::string getExternalStorageFile() const noexcept
	{
		return m_externalFile;
	}

	/** Only if isExternallyStored() returns true. \sa getExternalStorageFile */
	void getExternalStorageFileAbsolutePath(std::string& out_path) const;

	/** Only if isExternallyStored() returns true. \sa getExternalStorageFile */
	inline std::string getExternalStorageFileAbsolutePath() const
	{
		std::string tmp;
		getExternalStorageFileAbsolutePath(tmp);
		return tmp;
	}

	/** For external storage image objects only, this method makes sure the
	 * image is loaded in memory. Note that usually images are loaded on-the-fly
	 * on first access and there's no need to call this.
	 * \unload
	 */
	inline void forceLoad() const { makeSureImageIsLoaded(); }
	/** For external storage image objects only, this method unloads the image
	 * from memory (or does nothing if already unloaded).
	 *  It does not need to be called explicitly, unless the user wants to save
	 * memory for images that will not be used often.
	 *  If called for an image without the flag "external storage", it is
	 * simply ignored.
	 * \sa setExternalStorage, forceLoad
	 */
	void unload() const noexcept;

	/** @}  */
	// ================================================================

	// ================================================================
	/** @name Set, load & save methods
		@{  */

	/** Reads the image from raw pixels buffer in memory.
	 */
	void loadFromMemoryBuffer(
		unsigned int width, unsigned int height, bool color,
		unsigned char* rawpixels, bool swapRedBlue = false);

	/** Reads a color image from three raw pixels buffers in memory.
	 * bytesPerRow is the number of bytes per row per channel, i.e. the row
	 * increment.
	 */
	void loadFromMemoryBuffer(
		unsigned int width, unsigned int height, unsigned int bytesPerRow,
		unsigned char* red, unsigned char* green, unsigned char* blue);

	/** Set the image from a matrix, interpreted as grayscale intensity values,
	 *in the range [0,1] (normalized=true) or [0,255] (normalized=false)
	 *	Matrix indexes are assumed to be in this order: M(row,column)
	 * \sa getAsMatrix
	 */
	template <typename MAT>
	void setFromMatrix(const MAT& m, bool matrix_is_normalized = true)
	{
		MRPT_START
		const unsigned int lx = m.cols();
		const unsigned int ly = m.rows();
		this->resize(lx, ly, CH_GRAY);
		if (matrix_is_normalized)
		{  // Matrix: [0,1]
			for (unsigned int y = 0; y < ly; y++)
			{
				auto* pixels = ptrLine<uint8_t>(y);
				for (unsigned int x = 0; x < lx; x++)
					(*pixels++) = static_cast<uint8_t>(m.coeff(y, x) * 255);
			}
		}
		else
		{  // Matrix: [0,255]
			for (unsigned int y = 0; y < ly; y++)
			{
				auto* pixels = ptrLine<uint8_t>(y);
				for (unsigned int x = 0; x < lx; x++)
					(*pixels++) = static_cast<uint8_t>(m.coeff(y, x));
			}
		}
		MRPT_END
	}

	/** Set the image from RGB matrices, given the pixels in the range [0,1]
	 * (normalized=true) or [0,255] (normalized=false)
	 * Matrix indexes are assumed to be in this order: M(row,column)
	 * \sa getAsRGBMatrices
	 */
	template <typename MAT>
	void setFromRGBMatrices(
		const MAT& r, const MAT& g, const MAT& b,
		bool matrix_is_normalized = true)
	{
		MRPT_START
		makeSureImageIsLoaded();  // For delayed loaded images stored externally
		ASSERT_((r.size() == g.size()) && (r.size() == b.size()));
		const unsigned int lx = r.cols(), ly = r.rows();
		this->resize(lx, ly, CH_RGB);

		if (matrix_is_normalized)
		{  // Matrix: [0,1]
			for (unsigned int y = 0; y < ly; y++)
			{
				auto* pixels = ptrLine<uint8_t>(y);
				for (unsigned int x = 0; x < lx; x++)
				{
					(*pixels++) = static_cast<uint8_t>(r.coeff(y, x) * 255);
					(*pixels++) = static_cast<uint8_t>(g.coeff(y, x) * 255);
					(*pixels++) = static_cast<uint8_t>(b.coeff(y, x) * 255);
				}
			}
		}
		else
		{  // Matrix: [0,255]
			for (unsigned int y = 0; y < ly; y++)
			{
				auto* pixels = ptrLine<uint8_t>(y);
				for (unsigned int x = 0; x < lx; x++)
				{
					(*pixels++) = static_cast<uint8_t>(r.coeff(y, x));
					(*pixels++) = static_cast<uint8_t>(g.coeff(y, x));
					(*pixels++) = static_cast<uint8_t>(b.coeff(y, x));
				}
			}
		}
		MRPT_END
	}

	/** Reads the image from a binary stream containing a binary jpeg file.
	 * \exception std::exception On pixel coordinates out of bounds
	 */
	void loadFromStreamAsJPEG(mrpt::io::CStream& in);

	/** Load image from a file, whose format is determined from the extension
	 * (internally uses OpenCV).
	 * \param fileName The file to read from.
	 * \param isColor Specifies colorness of the loaded image:
	 *  - if >0, the loaded image is forced to be color 3-channel image;
	 *  - if 0, the loaded image is forced to be grayscale;
	 *  - if <0, the loaded image will be loaded as is (with number of channels
	 * depends on the file).
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
	bool loadFromFile(const std::string& fileName, int isColor = -1);

	/** Loads a TGA true-color RGBA image as two CImage objects, one for the RGB
	 * channels plus a separate gray-level image with A channel.
	 * \return true on success
	 */
	static bool loadTGA(
		const std::string& fileName, mrpt::img::CImage& out_RGB,
		mrpt::img::CImage& out_alpha);

	/** Loads the image from an XPM array, as #include'd from a ".xpm" file.
	 * \param[in] swap_rb Swaps red/blue channels from loaded image. *Seems* to
	 * be always needed, so it's enabled by default.
	 * \sa loadFromFile
	 * \return false on any error */
	bool loadFromXPM(const char* const* xpm_array, bool swap_rb = true);

	/** Save the image to a file, whose format is determined from the extension
	 * (internally uses OpenCV).
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
	 * \param jpeg_quality Only for JPEG files, the quality of the compression
	 * in the range [0-100]. Larger is better quality but slower.
	 * \note jpeg_quality is only effective if MRPT is compiled against OpenCV
	 * 1.1.0 or newer.
	 * \return False on any error
	 * \sa loadFromFile
	 */
	bool saveToFile(const std::string& fileName, int jpeg_quality = 95) const;

	/** Save image to binary stream as a JPEG (.jpg) compressed format.
	 * \exception std::exception On number of rows or cols equal to zero or
	 * other errors.
	 * \sa saveToJPEG
	 */
	void saveToStreamAsJPEG(
		mrpt::io::CStream& out, const int jpeg_quality = 95) const;

	/** @}  */
	// ================================================================

	// ================================================================
	/** @name Color/Grayscale conversion
		@{ */

	/** Returns a grayscale version of the image, or a shallow copy of itself if
	 * it is already a grayscale image.
	 */
	CImage grayscale() const;

	/** \overload.
	 * In-place is supported by setting `ret=*this`.
	 * \return true if SSE2 version has been run (or if the image was already
	 * grayscale)
	 */
	bool grayscale(CImage& ret) const;

	/** Returns a color (RGB) version of the grayscale image, or a shallow copy
	 * of itself if it is already a color image.
	 * \sa grayscale
	 */
	CImage colorImage() const;

	/** \overload.
	 * In-place is supported by setting `ret=*this`. */
	void colorImage(CImage& ret) const;

	/** @} */

	/** (DEPRECATED, DO NOT USE - Kept here only to interface opencv 2.4) */
	void getAsIplImage(IplImage* dest) const;

   protected:
	/** @name Data members
		@{ */

	/** PIMPL to cv::Mat object actually holding the image */
	struct Impl;
	mrpt::pimpl<Impl> m_impl;

	/**  Set to true only when using setExternalStorage.
	 * \sa setExternalStorage
	 */
	mutable bool m_imgIsExternalStorage{false};

	/** The file name of a external storage image. */
	mutable std::string m_externalFile;

	/** @} */

	/** Checks if the image is of type "external storage", and if so and not
	 * loaded yet, load it.
	 * \exception CExceptionExternalImageNotFound */
	void makeSureImageIsLoaded() const;
	uint8_t* internal_get(int col, int row, uint8_t channel = 0) const;
	void internal_fromIPL(const IplImage* iplImage, copy_type_t c);
};  // End of class
}  // namespace mrpt::img
