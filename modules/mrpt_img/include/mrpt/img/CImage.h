/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/img/CCanvas.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/serialization/CSerializable.h>

#include <memory>
#include <string>
#include <vector>

// Forwards decls:
namespace mrpt::io
{
class CStream;
}

// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM(mrpt::img::CImage)

/** Classes for image storage and manipulation \ingroup mrpt_img_grp */
namespace mrpt::img
{
enum class PixelDepth : uint8_t
{
  D8U = 1,
  D16U = 2
};

/** Interpolation methods for images.
 * \sa CImage::scaleImage
 * \ingroup mrpt_img_grp
 */
enum TInterpolationMethod
{
  IMG_INTERP_NN = 0,      //!< Nearest neighbor
  IMG_INTERP_LINEAR = 1,  //!< Bilinear interpolation
  IMG_INTERP_CUBIC = 2,   //!< Bicubic interpolation
  IMG_INTERP_AREA = 3     //!< Area-based (for downsampling)
};

/** For use in mrpt::img::CImage */
enum TImageChannels : uint8_t
{
  // To define an image:
  CH_GRAY = 1,
  CH_RGB = 3,
  CH_RGBA = 4,

  // Specifiers for loading from file/stream only:
  CH_AS_IS = 0,  //!< Load as-is, no conversion
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
  /** Deep copy: the copied object has a duplicate of all data, becoming independent  */
  DEEP_COPY = 1
};

/** Used in mrpt::img::CImage */
class CExceptionExternalImageNotFound : public std::runtime_error
{
 public:
  CExceptionExternalImageNotFound(const std::string& s);
};

/** A class for storing images as grayscale, RGB, or RGBA bitmaps.
 *
 * Supported I/O are:
 * - Saving/loading from files of different formats (JPG, PNG, TGA, BMP, PSD, GIF, HDR, PIC) using
 *   the methods CImage::loadFromFile() and CImage::saveToFile(). This uses the [stb
 *   library](https://github.com/nothings/stb).
 * - Importing from an XPM array (.xpm file format) using CImage::loadFromXPM()
 * - Binary dump using the CSerializable interface (<< and >> operators), just as most objects in
 *   MRPT. This format is not compatible with any standardized image format but it is fast.
 *
 *  How to create color/grayscale images:
 *  \code
 *    CImage  img1(width, height,  CH_GRAY );  // Grayscale image (8U1C)
 *    CImage  img2(width, height,  CH_RGB );   // RGB image (8U3C)
 *  \endcode
 *
 * Additional notes:
 * - Since MRPT 3.0.0, the internal format for images is a raw memory buffer managed by the STB
 *   library. Previous versions used OpenCV's cv::Mat, but this was removed to reduce dependencies.
 *
 * - By default, all images use unsigned 8-bit storage format for pixels (on each channel), but
 *   16-bit depth is also supported via PixelDepth::D16U.
 *
 * - An **external storage mode** can be enabled by calling CImage::setExternalStorage, useful for
 *   storing large collections of image objects in memory while loading the image data itself only
 *   for the relevant images at any time. See CImage::forceLoad() and CImage::unload().
 *
 * - Operator = and copy ctor make shallow copies. For deep copies, see CImage::makeDeepCopy() or
 *   CImage(const CImage&, copy_type_t), e.g:
 * \code
 * CImage a(20, 10, CH_GRAY);
 * // Shallow copy ctor:
 * CImage b(a, mrpt::img::SHALLOW_COPY);
 * CImage c(a, mrpt::img::DEEP_COPY);
 * \endcode
 *
 * - If you are interested in a smart pointer to an image, use:
 * \code
 * CImage::Ptr myImg = CImage::Create(); // optional ctor arguments
 * // or:
 * CImage::Ptr myImg = std::make_shared<CImage>(...);
 *  \endcode
 *
 * \sa mrpt::vision, mrpt::serialization::CSerializable, mrpt::img::CCanvas
 *
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
  CImage(int32_t width, int32_t height, TImageChannels nChannels = CH_RGB);

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

  /** Constructor from another CImage, making or not a deep copy of the data.
   */
  CImage(const CImage& img, copy_type_t copy_type);

  /** @} */

  /** @name Manipulate the image contents or size, various computer-vision methods
    @{ */

  /** Resets the image to the state after a default ctor. Accessing the image
   * after will throw an exception, unless it is formerly initialized somehow:
   * loading an image from disk, calling resize(), etc. */
  void clear();

  /** Changes the size of the image, erasing previous contents (does NOT scale its current content,
   *   for that, see scaleImage).
   * \sa scaleImage
   */
  void resize(
      int32_t width, int32_t height, TImageChannels nChannels, PixelDepth depth = PixelDepth::D8U);

  PixelDepth getPixelDepth() const;

  /** Scales this image to a new size, interpolating as needed, saving the new
   * image in a different output object, or operating in-place if `out_img==this`.
   * \sa resize, rotateImage
   */
  void scaleImage(
      CImage& out_img,
      int32_t width,
      int32_t height,
      TInterpolationMethod interp = IMG_INTERP_CUBIC) const;

  /** Rotates the image by the given angle (in radians) around the given center point, with
   * an optional scale factor.
   * \note This method is marked as TODO - not yet implemented with STB.
   * \sa resize, scaleImage
   */
  void rotateImage(
      CImage& out_img, double ang, const TPixelCoord& center, double scale = 1.0) const;

  /** Changes the value of the pixel (x,y).
   *  Pixel coordinates starts at the left-top corner of the image, and start in (0,0).
   *  The meaning of the parameter "color" depends on the implementation: it will usually
   *   be a 24bit RGB value (0x00RRGGBB), but it can also be just a 8bit gray level.
   *  This method must support (x,y) values OUT of the actual image size without neither
   *   raising exceptions, nor leading to memory access errors.
   * \sa at, ptr
   */
  void setPixel(const TPixelCoord& pt, const mrpt::img::TColor& color) override;

  // See CCanvas docs
  void drawImage(const TPixelCoord& pt, const mrpt::img::CImage& img) override;

  // See CCanvas docs
  void filledRectangle(
      const TPixelCoord& pt0, const TPixelCoord& pt1, const mrpt::img::TColor& color) override;

  /** Returns a new image scaled down to half its original size
   * \exception std::exception On odd size
   * \sa scaleDouble, scaleImage
   */
  [[nodiscard]] inline CImage scaleHalf(TInterpolationMethod interp) const
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
  [[nodiscard]] inline CImage scaleDouble(TInterpolationMethod interp) const
  {
    CImage ret;
    this->scaleDouble(ret, interp);
    return ret;
  }

  //! \overload
  void scaleDouble(CImage& out_image, TInterpolationMethod interp) const;

  /** Extract a patch from this image, saving it into "patch" (its previous
   * contents will be overwritten).
   *  The patch to extract starts at (col,row) and has the given dimensions.
   */
  void extract_patch(
      CImage& patch, const TPixelCoord& top_left_corner, const TImageSize& patch_size) const;

  /** Optimize the brightness range of an image without using histogram.
   * Only for one channel images.
   * \note Marked as TODO - not yet implemented with STB.
   */
  void normalize();

  /** Flips the image vertically. \sa swapRB(), flipHorizontal() */
  void flipVertical();

  /** Flips the image horizontally \sa swapRB(), flipVertical() */
  void flipHorizontal();

  /** Swaps red and blue channels. */
  void swapRB();

  /** Undistort the image according to some camera parameters.
   * \note Marked as TODO - not yet implemented with STB.
   * \sa mrpt::vision::CUndistortMap
   */
  void undistort(CImage& out_img, const mrpt::img::TCamera& cameraParams) const;

  /** Filter the image with a Median filter with a window size WxW.
   * \note Marked as TODO - not yet implemented with STB.
   */
  void filterMedian(CImage& out_img, int W = 3) const;

  /** Filter the image with a Gaussian filter with a window size WxH.
   * \note Marked as TODO - not yet implemented with STB.
   */
  void filterGaussian(CImage& out_img, int W = 3, int H = 3, double sigma = 1.0) const;

  /** Draw onto this image the detected corners of a chessboard.
   * \note Marked as TODO - not yet implemented with STB.
   */
  bool drawChessboardCorners(
      const std::vector<TPixelCoordf>& cornerCoords,
      unsigned int check_size_x,
      unsigned int check_size_y,
      unsigned int lines_width = 1,
      unsigned int circles_radius = 4);

  /** Joins two images side-by-side horizontally. Both images must have the
   * same number of rows and be of the same type (i.e. depth and color mode)
   */
  void joinImagesHorz(const CImage& im1, const CImage& im2);

  /** Compute the KLT response at a given pixel (x,y) - Only for grayscale images.
   */
  [[nodiscard]] float KLT_response(const TPixelCoord& pt, const int32_t half_window_size) const;

  /** @} */

  /** @name Copy, move & swap operations
    @{ */

  /** Returns a shallow copy of the original image */
  [[nodiscard]] inline CImage makeShallowCopy() const
  {
    CImage r = *this;
    return r;
  }

  /** Returns a deep copy of this image.
   * If the image is externally-stored, there is no difference with a shallow copy.
   * \sa makeShallowCopy()
   */
  [[nodiscard]] CImage makeDeepCopy() const { return CImage(*this, DEEP_COPY); }

  /** Copies from another image (shallow copy), and, if it is externally stored, the image file
   * will be actually loaded into memory in "this" object.
   * \sa operator =
   * \exception CExceptionExternalImageNotFound If the external image couldn't be loaded.
   */
  void copyFromForceLoad(const CImage& o);

  /** Efficiently swap of two images */
  void swap(CImage& o);

  /** @} */

  /** @name Access to image contents
    @{ */

  /**  Access to pixels without checking boundaries, and doing a reinterpret_cast<> of the data
   * as the given type.
   * \sa The CImage::operator() which does check for coordinate limits.
   */
  template <typename T>
  [[nodiscard]] const T& at(int32_t col, int32_t row, int8_t channel = 0) const
  {
    return *reinterpret_cast<const T*>(internal_get(col, row, channel));
  }

  /** \overload Non-const case */
  template <typename T>
  [[nodiscard]] T& at(int32_t col, int32_t row, int8_t channel = 0)
  {
    return *reinterpret_cast<T*>(internal_get(col, row, channel));
  }

  /** Returns a pointer to a given pixel, without checking for boundaries.
   * \sa The CImage::operator() which does check for coordinate limits.
   */
  template <typename T>
  [[nodiscard]] const T* ptr(int32_t col, int32_t row, int8_t channel = 0) const
  {
    return reinterpret_cast<const T*>(internal_get(col, row, channel));
  }

  /** \overload Non-const case */
  template <typename T>
  [[nodiscard]] T* ptr(int32_t col, int32_t row, int8_t channel = 0)
  {
    return reinterpret_cast<T*>(internal_get(col, row, channel));
  }

  /** Returns a pointer to the first pixel of the given line.\sa ptr, at */
  template <typename T>
  [[nodiscard]] const T* ptrLine(int32_t row) const
  {
    return reinterpret_cast<const T*>(internal_get(0, row, 0));
  }

  /** \overload Non-const case */
  template <typename T>
  [[nodiscard]] T* ptrLine(int32_t row)
  {
    return reinterpret_cast<T*>(internal_get(0, row, 0));
  }

  /** Returns the contents of a given pixel at the desired channel, in float format: [0,255]->[0,1]
   *   The coordinate origin is pixel(0,0)=top-left corner of the image.
   * \exception std::exception On pixel coordinates out of bounds
   */
  [[nodiscard]] float getAsFloat(const TPixelCoord& pt, int8_t channel) const;

  /** Returns the contents of a given pixel (for gray-scale images, in color images the gray scale
   * equivalent is computed for the pixel), in float format: [0,255]->[0,1]
   *   The coordinate origin is pixel(0,0)=top-left corner of the image.
   * \exception std::exception On pixel coordinates out of bounds
   */
  [[nodiscard]] float getAsFloat(const TPixelCoord& pt) const;

  /** @} */

  /** @name Query image properties
    @{ */

  /** Returns the width of the image in pixels \sa getSize */
  [[nodiscard]] int32_t getWidth() const override;

  /** Returns the height of the image in pixels \sa getSize */
  [[nodiscard]] int32_t getHeight() const override;

  /** Return the size of the image \sa getWidth, getHeight */
  [[nodiscard]] TImageSize getSize() const;

  /** Returns the row stride of the image: this is the number of *bytes* between two consecutive
   * rows. You can access the pointer to the first row with ptrLine(0)
   * \sa getSize, as, ptr, ptrLine
   */
  [[nodiscard]] size_t getRowStride() const;

  /** As of mrpt 3.0.0, this returns either "GRAY", "RGB", or "RGBA". */
  [[nodiscard]] std::string getChannelsOrder() const;

  /** Returns 1 (grayscale), 3 (RGB) or 4 (RGBA) */
  [[nodiscard]] TImageChannels channels() const;

  /** Returns true if the image is RGB or RGBA, false if it is grayscale */
  [[nodiscard]] bool isColor() const;

  /** Returns true if the object is in the state after default constructor.
   * Returns false for delay-loaded images, disregarding whether the image is actually on disk or
   * memory.
   */
  [[nodiscard]] bool isEmpty() const;

  /** Returns true (images are always stored with origin at top-left) */
  [[nodiscard]] bool isOriginTopLeft() const;

  /** Returns the image as a matrix with pixel grayscale values in the range [0,1].
   * Matrix indexes in this order: M(row,column)
   *  \param doResize If set to true (default), the output matrix will be always the size of the
   * image at output. If set to false, the matrix will be enlarged to the size of the image, but it
   * will not be cropped if it has room enough.
   *  \param x_min The starting "x" coordinate to extract (default=0=the first column)
   *  \param y_min The starting "y" coordinate to extract (default=0=the first row)
   *  \param x_max The final "x" coordinate (inclusive) to extract (default=-1=the last column)
   *  \param y_max The final "y" coordinate (inclusive) to extract (default=-1=the last row)
   * \param normalize_01 Normalize the image values such that they fall in the range [0,1]
   * (default: true). If set to false, the matrix will hold numbers in the range [0,255].
   * \sa setFromMatrix
   */
  void getAsMatrix(
      mrpt::math::CMatrixFloat& outMatrix,
      bool doResize = true,
      int x_min = 0,
      int y_min = 0,
      int x_max = -1,
      int y_max = -1,
      bool normalize_01 = true) const;

  /** \overload For uint8_t matrices [0, 255]. */
  void getAsMatrix(
      mrpt::math::CMatrix_u8& outMatrix,
      bool doResize = true,
      int x_min = 0,
      int y_min = 0,
      int x_max = -1,
      int y_max = -1) const;

  /** Returns the image as RGB matrices with pixel values in the range [0,1].
   * Matrix indexes in this order: M(row,column)
   *  \param doResize If set to true (default), the output matrix will be always the size of the
   * image at output. If set to false, the matrix will be enlarged to the size of the image, but it
   * will not be cropped if it has room enough.
   *  \param x_min The starting "x" coordinate to extract (default=0=the first column)
   *  \param y_min The starting "y" coordinate to extract (default=0=the first row)
   *  \param x_max The final "x" coordinate (inclusive) to extract (default=-1=the last column)
   *  \param y_max The final "y" coordinate (inclusive) to extract (default=-1=the last row)
   * \sa setFromRGBMatrices
   */
  void getAsRGBMatrices(
      mrpt::math::CMatrixFloat& outMatrixR,
      mrpt::math::CMatrixFloat& outMatrixG,
      mrpt::math::CMatrixFloat& outMatrixB,
      bool doResize = true,
      int x_min = 0,
      int y_min = 0,
      int x_max = -1,
      int y_max = -1) const;

  /** \overload For uint8_t matrices [0, 255]. */
  void getAsRGBMatrices(
      mrpt::math::CMatrix_u8& outMatrixR,
      mrpt::math::CMatrix_u8& outMatrixG,
      mrpt::math::CMatrix_u8& outMatrixB,
      bool doResize = true,
      int x_min = 0,
      int y_min = 0,
      int x_max = -1,
      int y_max = -1) const;

  /** Returns the image as a matrix, where the image is "tiled" (repeated) the required number of
   * times to fill the entire size of the matrix on input.
   */
  void getAsMatrixTiled(mrpt::math::CMatrixFloat& outMatrix) const;

  /** @} */

  /** @name External storage-mode methods
    @{  */

  /**  By using this method the image is marked as referenced to an external file, which will be
   * loaded only under demand.
   *   A CImage with external storage does not consume memory until some method trying to access
   * the image is invoked (e.g. getWidth(), isColor(),...) At any moment, the image can be unloaded
   * from memory again by invoking unload. An image becomes of type "external storage" only through
   * calling setExternalStorage. This property remains after serializing the object. File names can
   * be absolute, or relative to the CImage::getImagesPathBase() directory. Filenames staring with
   * "X:\" or "/" are considered absolute paths. By calling this method the current contents of the
   * image are NOT saved to that file, because this method can be also called to let the object
   * know where to load the image in case its contents are required. Thus, for saving images in
   * this format (not when loading) the proper order of commands should be:
   *   \code
   *   img.saveToFile( fileName );
   *   img.setExternalStorage( fileName );
   *   \endcode
   *
   *   \note Modifications to the memory copy of the image are not automatically saved to disk.
   *  \sa unload, isExternallyStored
   */
  void setExternalStorage(const std::string& fileName) noexcept;

  /** By default, "."
   *  \sa setExternalStorage
   *  \note Since MRPT 2.3.3 this is a synonym with mrpt::io::getLazyLoadPathBase()
   */
  static const std::string& getImagesPathBase();

  /**  \note Since MRPT 2.3.3 this is a synonym with mrpt::io::setLazyLoadPathBase() */
  static void setImagesPathBase(const std::string& path);

  /** See setExternalStorage(). */
  [[nodiscard]] bool isExternallyStored() const noexcept { return m_state->imgIsExternalStorage; }

  /** Only if isExternallyStored() returns true. \sa getExternalStorageFileAbsolutePath */
  [[nodiscard]] inline std::string getExternalStorageFile() const noexcept
  {
    return m_state->externalFile;
  }

  /** Only if isExternallyStored() returns true. \sa getExternalStorageFile */
  void getExternalStorageFileAbsolutePath(std::string& out_path) const;

  /** Only if isExternallyStored() returns true. \sa getExternalStorageFile */
  [[nodiscard]] inline std::string getExternalStorageFileAbsolutePath() const
  {
    std::string tmp;
    getExternalStorageFileAbsolutePath(tmp);
    return tmp;
  }

  /** For external storage image objects only, this method makes sure the image is loaded in
   * memory. Note that usually images are loaded on-the-fly on first access and there's no need to
   * call this.
   * \sa unload
   */
  inline void forceLoad() const { makeSureImageIsLoaded(true); }

  /** For external storage image objects only, this method unloads the image from memory (or does
   * nothing if already unloaded). It does not need to be called explicitly, unless the user wants
   * to save memory for images that will not be used often. If called for an image without the flag
   * "external storage", it is simply ignored.
   * \sa setExternalStorage, forceLoad
   */
  void unload() const noexcept;

  /**  Computes the correlation matrix between this image and another one.
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
   * subtracted to the pixels of "this" image before performing correlation.
   * \param biasInImg This optional parameter is a fixed "bias" value to be
   * subtracted to the pixels of "in_img" image before performing
   * correlation. Note: By default, the search area is the whole (this) image.
   * (by JLBC @ JAN-2006)
   */
  void cross_correlation_FFT(
      const CImage& in_img,
      mrpt::math::CMatrixFloat& out_corr,
      std::optional<int32_t> u_search_ini = std::nullopt,
      std::optional<int32_t> v_search_ini = std::nullopt,
      std::optional<int32_t> u_search_size = std::nullopt,
      std::optional<int32_t> v_search_size = std::nullopt,
      std::optional<float> biasThisImg = std::nullopt,
      std::optional<float> biasInImg = std::nullopt) const;

  /** @}  */

  /** @name Set, load & save methods
    @{  */

  /** Reads the image from raw pixels buffer in memory.
   */
  void loadFromMemoryBuffer(
      int32_t width,
      int32_t height,
      TImageChannels color_channels,
      uint8_t* rawpixels,
      bool swapRedBlue = false);

  /** Set the image from a matrix, interpreted as grayscale intensity values, in the range [0,1]
   * (normalized=true) or [0,255] (normalized=false) Matrix indexes are assumed to be in this
   * order: M(row,column)
   * \sa getAsMatrix
   */
  template <typename MAT>
  void setFromMatrix(const MAT& m, bool matrix_is_normalized = true, bool flip_vertically = false)
  {
    MRPT_START
    const auto lx = m.cols();
    const auto ly = m.rows();
    this->resize(lx, ly, CH_GRAY);
    if (matrix_is_normalized)
    {  // Matrix: [0,1]
      for (auto y = 0; y < ly; y++)
      {
        auto* pixels = ptrLine<uint8_t>(flip_vertically ? (ly - 1 - y) : y);
        for (unsigned int x = 0; x < lx; x++)
        {
          (*pixels++) = static_cast<uint8_t>(m.coeff(y, x) * 255);
        }
      }
    }
    else
    {  // Matrix: [0,255]
      for (unsigned int y = 0; y < ly; y++)
      {
        auto* pixels = ptrLine<uint8_t>(flip_vertically ? (ly - 1 - y) : y);
        for (unsigned int x = 0; x < lx; x++)
        {
          (*pixels++) = static_cast<uint8_t>(m.coeff(y, x));
        }
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
      const MAT& r, const MAT& g, const MAT& b, bool matrix_is_normalized = true)
  {
    MRPT_START
    makeSureImageIsLoaded();  // For delayed loaded images stored externally
    ASSERT_((r.size() == g.size()) && (r.size() == b.size()));
    const unsigned int lx = r.cols(), ly = r.rows();
    this->resize(lx, ly, CH_RGB);

    if (matrix_is_normalized)
    {  // Matrix: [0,1]
      for (int y = 0; y < ly; y++)
      {
        auto* pixels = ptrLine<uint8_t>(y);
        for (int x = 0; x < lx; x++)
        {
          (*pixels++) = static_cast<uint8_t>(r.coeff(y, x) * 255);
          (*pixels++) = static_cast<uint8_t>(g.coeff(y, x) * 255);
          (*pixels++) = static_cast<uint8_t>(b.coeff(y, x) * 255);
        }
      }
    }
    else
    {  // Matrix: [0,255]
      for (int y = 0; y < ly; y++)
      {
        auto* pixels = ptrLine<uint8_t>(y);
        for (int x = 0; x < lx; x++)
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
   *
   * MRPT also provides the special loader loadFromXPM().
   *
   * \return False on any error
   * \sa saveToFile, setExternalStorage,loadFromXPM
   */
  [[nodiscard]] bool loadFromFile(
      const std::string& fileName, TImageChannels loadChannels = CH_AS_IS);

  /** Static method to construct an CImage object from a file.
   * See CImage::loadFromFile() for meaning of parameters.
   *
   * \exception std::exception On load error.
   * \note New in MRPT 2.4.2
   */
  [[nodiscard]] static mrpt::img::CImage LoadFromFile(
      const std::string& fileName, TImageChannels loadChannels = CH_AS_IS);

  /** Loads the image from an XPM array, as included from a ".xpm" file.
   * \param[in] swap_rb Swaps red/blue channels from loaded image. *Seems* to
   * be always needed, so it's enabled by default.
   * \sa loadFromFile
   * \return false on any error */
  [[nodiscard]] bool loadFromXPM(const char* const* xpm_array, bool swap_rb = true);

  /** Save the image to a file, whose format is determined from the extension (internally uses the
   * stb library).
   * \param fileName The file to write to.
   *
   * The supported formats are:
   * - `*.png`: Portable Network Graphics
   * - `*.bmp`: Windows bitmaps
   * - `*.tga`: TGA files
   * - `*.jpeg|*.jpg`: JPEG files
   *
   * \param jpeg_quality Only for JPEG files, the quality of the compression in the range [0-100].
   * Larger is better quality but slower.
   * \return False on any error
   * \sa loadFromFile
   */
  [[nodiscard]] bool saveToFile(const std::string& fileName, int jpeg_quality = 95) const;

  /** Save image to binary stream as a JPEG (.jpg) compressed format.
   * \exception std::exception On number of rows or cols equal to zero or
   * other errors.
   * \sa saveToJPEG
   */
  void saveToStreamAsJPEG(mrpt::io::CStream& out, const int jpeg_quality = 95) const;

  /** @}  */
  // ================================================================

  // ================================================================
  /** @name Color/Grayscale conversion
    @{ */

  /** Returns a grayscale version of the image, or a shallow copy of itself if
   * it is already a grayscale image.
   */
  [[nodiscard]] CImage grayscale() const;

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
  [[nodiscard]] CImage colorImage() const;

  /** \overload.
   * In-place is supported by setting `ret=*this`. */
  void colorImage(CImage& ret) const;

  /** @} */

 protected:
  /** @name Data members
    @{ */

  /** PIMPL actual struct holding the image data */
  struct Impl
  {
    // Default constructor
    Impl() = default;

    // Destructor
    ~Impl();

    // Delete copy constructor and copy assignment operator
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;

    // Define move constructor and move assignment operator
    Impl(Impl&&) noexcept = default;
    Impl& operator=(Impl&&) noexcept = default;

    // Data:
    int32_t width = 0;
    int32_t height = 0;
    TImageChannels channels = TImageChannels::CH_GRAY;
    PixelDepth depth = PixelDepth::D8U;
    uint8_t* image_data = nullptr;  //!< Pointer to the loaded image data

    /**  Set to true only when using setExternalStorage. \sa setExternalStorage  */
    mutable bool imgIsExternalStorage{false};

    /** The file name of a external storage image. */
    mutable std::string externalFile;

    [[nodiscard]] bool empty() const { return image_data == nullptr || width == 0 || height == 0; }

    /** Clears the image data, external image data, etc. To clear just the image data, use
     * clear_image_data() */
    void clear();

    void clear_image_data();

    void deep_copy(const Impl& o)
    {
      clear();

      width = o.width;
      height = o.height;
      channels = o.channels;
      depth = o.depth;
      imgIsExternalStorage = o.imgIsExternalStorage;
      externalFile = o.externalFile;

      if (o.image_data)
      {
        const auto num_bytes = image_buffer_size_bytes();
        image_data = reinterpret_cast<uint8_t*>(std::malloc(num_bytes));
        std::memcpy(image_data, o.image_data, num_bytes);
      }
    }

    /** Returns the image buffer size according to width, height, channels, and pixel depth.
     * This function just computes the size of the image buffer in bytes from the properties filled
     * in in the structure.
     */
    [[nodiscard]] std::size_t image_buffer_size_bytes() const
    {
      return static_cast<std::size_t>(width) * static_cast<std::size_t>(height) *
             static_cast<std::size_t>(channels) * static_cast<std::size_t>(depth);
    }

    [[nodiscard]] std::size_t row_stride_in_bytes() const
    {
      return static_cast<std::size_t>(width) * static_cast<std::size_t>(channels) *
             static_cast<std::size_t>(depth);
    }

    [[nodiscard]] std::size_t pixel_size_in_bytes() const
    {
      return static_cast<std::size_t>(channels) * static_cast<std::size_t>(depth);
    }
  };

  std::shared_ptr<Impl> m_state;

  /** @} */

  /** Checks if the image is of type "external storage", and if so and not
   * loaded yet, load it.
   * \exception CExceptionExternalImageNotFound */
  void makeSureImageIsLoaded(bool allowNonInitialized = false) const;

  [[nodiscard]] uint8_t* internal_get(int32_t col, int32_t row, int8_t channel = 0);
  [[nodiscard]] const uint8_t* internal_get(int32_t col, int32_t row, int8_t channel = 0) const;

};  // End of class
}  // namespace mrpt::img
