/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/config.h>  // MRPT_HAS_MATLAB
#include <mrpt/core/cpu.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/round.h>  // for round()
#include <mrpt/img/CImage.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/fourier.h>
#include <mrpt/math/utils.h>  // for roundup()
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/memory.h>

#include <iostream>

#include "CImage.SSEx.h"
#include "CImage_impl.h"

// Define STB_IMAGE_IMPLEMENTATION in exactly one .c or .cpp file
// before including the header to create the implementation.
STB_DISABLE_WARNINGS
#include "stb/stb_image.h"
#include "stb/stb_image_resize2.h"
#include "stb/stb_image_write.h"
STB_RESTORE_WARNINGS

#if MRPT_HAS_MATLAB
#include <mexplus/mxarray.h>
#endif

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CImage, CSerializable, mrpt::img)

// Do performance time logging?
#define IMAGE_ALLOC_PERFLOG 0

namespace
{
const thread_local bool MRPT_DEBUG_IMG_LAZY_LOAD =
    mrpt::get_env<bool>("MRPT_DEBUG_IMG_LAZY_LOAD", false);

#if IMAGE_ALLOC_PERFLOG
mrpt::img::CTimeLogger alloc_times;
#endif

}  // namespace

namespace mrpt::img
{

CExceptionExternalImageNotFound::CExceptionExternalImageNotFound(const std::string& s) :
    std::runtime_error(s)
{
}

const std::string& CImage::getImagesPathBase() { return mrpt::io::getLazyLoadPathBase(); }
void CImage::setImagesPathBase(const std::string& path) { mrpt::io::setLazyLoadPathBase(path); }

// Default ctor
CImage::CImage() : m_state(std::make_shared<CImage::Impl>()) {}

// Ctor with size
CImage::CImage(int32_t width, int32_t height, TImageChannels nChannels) : CImage()
{
  resize(width, height, nChannels);
}

void CImage::swap(CImage& o) { std::swap(m_state, o.m_state); }

void CImage::copyFromForceLoad(const CImage& o)
{
  *this = o;
  forceLoad();
}

CImage::CImage(const CImage& img, copy_type_t copy_type) : CImage()
{
  MRPT_START

  // this new image is *not* lazy-load.
  if (copy_type == DEEP_COPY && !img.m_state->empty())
  {
    // deep copy
    m_state->deep_copy(*img.m_state);
  }
  else
  {
    // shallow copy
    m_state = img.m_state;
  }
  MRPT_END
}

void CImage::resize(int32_t width, int32_t height, TImageChannels nChannels, PixelDepth depth)
{
  MRPT_START

  // Dont call makeSureImageIsLoaded() here,
  // since it will throw if resize() is called from a ctor, where it's
  // legit for the img to be uninitialized.

  // If we're resizing to exactly the current size, do nothing:
  if (m_state->width == width && m_state->height == height && m_state->channels == nChannels &&
      m_state->depth == depth)
  {
    // Nothing to do:
    return;
  }

#if IMAGE_ALLOC_PERFLOG
  const std::string sLog = mrpt::format("resize %zux%zu", width, height);
  alloc_times.enter(sLog.c_str());
#endif

  m_state->clear();
  m_state->width = width;
  m_state->height = height;
  m_state->channels = nChannels;
  m_state->depth = depth;
  m_state->image_data = reinterpret_cast<uint8_t*>(std::malloc(m_state->image_buffer_size_bytes()));

#if IMAGE_ALLOC_PERFLOG
  alloc_times.leave(sLog.c_str());
#endif

  MRPT_END
}

PixelDepth CImage::getPixelDepth() const
{
  MRPT_START
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  return m_state->depth;
  MRPT_END
}

mrpt::img::CImage CImage::LoadFromFile(const std::string& fileName, TImageChannels loadChannels)
{
  CImage im;
  bool ok = im.loadFromFile(fileName, loadChannels);
  if (!ok) THROW_EXCEPTION_FMT("Error loading image from '%s'", fileName.c_str());
  return im;
}

bool CImage::loadFromFile(const std::string& fileName, TImageChannels loadChannels)
{
  MRPT_START

  m_state->clear();

  // --- Loading ---
  int width = 0;
  int height = 0;
  int original_channels = 0;
  unsigned char* image_data = nullptr;

  // stbi_load loads the image from the file specified by filename.
  // It returns a pointer to the loaded image data, or nullptr on failure.
  // - filename: Path to the image file.
  // - &width: Pointer to an int to store the image width.
  // - &height: Pointer to an int to store the image height.
  // - &original_channels: Pointer to an int to store the original number of color channels.
  // - desired_channels: Forces stb_image to convert the image to have this many channels per pixel.
  //                     Common values: 3 (RGB), 4 (RGBA). 0 means load original channels.
  image_data = stbi_load(fileName.c_str(), &width, &height, &original_channels, loadChannels);

  if (image_data == nullptr)
  {
    // std::cerr << "Reason: " << stbi_failure_reason() << std::endl;
    return false;  // Indicate failure
  }

  // Store in image container:
  m_state->width = width;
  m_state->height = height;
  m_state->channels = static_cast<TImageChannels>(original_channels);
  m_state->depth = PixelDepth::D8U;
  m_state->image_data = image_data;

  return true;
  MRPT_END
}

bool CImage::saveToFile(const std::string& fileName, int jpeg_quality) const
{
  MRPT_START
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  ASSERT_(!m_state->empty());

  const auto ext = mrpt::system::lowerCase(mrpt::system::extractFileExtension(fileName));

  if (ext == "jpg" || ext == "jpeg")
  {
    return 0 != stbi_write_jpg(
                    fileName.c_str(), static_cast<int>(m_state->width),
                    static_cast<int>(m_state->height), m_state->channels, m_state->image_data,
                    jpeg_quality);
  }
  else if (ext == "png")
  {
    return 0 != stbi_write_png(
                    fileName.c_str(), static_cast<int>(m_state->width),
                    static_cast<int>(m_state->height), m_state->channels, m_state->image_data,
                    static_cast<int>(m_state->row_stride_in_bytes()));
  }
  else if (ext == "bmp")
  {
    return 0 != stbi_write_bmp(
                    fileName.c_str(), static_cast<int>(m_state->width),
                    static_cast<int>(m_state->height), m_state->channels, m_state->image_data);
  }
  else if (ext == "tga")
  {
    return 0 != stbi_write_tga(
                    fileName.c_str(), static_cast<int>(m_state->width),
                    static_cast<int>(m_state->height), m_state->channels, m_state->image_data);
  }
  else
  {
    THROW_EXCEPTION_FMT(
        "Unknown image format extension '%s' (file: '%s')", ext.c_str(), fileName.c_str());
  }

  MRPT_END
}

void CImage::loadFromMemoryBuffer(
    int32_t width,
    int32_t height,
    TImageChannels color_channels,
    uint8_t* rawpixels,
    bool swapRedBlue)
{
  MRPT_START

  resize(width, height, color_channels);

  const auto bytes_per_row_out = m_state->row_stride_in_bytes();

  if (color_channels > 1 && swapRedBlue)
  {
    // Do copy & swap at once:
    uint8_t* ptr_src = rawpixels;
    auto* ptr_dest = m_state->image_data;

    for (auto h = height; h--;)
    {
      for (int32_t i = 0; i < width; i++, ptr_src += 3, ptr_dest += 3)
      {
        uint8_t t0 = ptr_src[0], t1 = ptr_src[1], t2 = ptr_src[2];
        ptr_dest[2] = t0;
        ptr_dest[1] = t1;
        ptr_dest[0] = t2;
      }
      ptr_dest += bytes_per_row_out - static_cast<std::size_t>(width * 3);
    }
  }
  else
  {
    // Copy the image data:
    std::memcpy(m_state->image_data, rawpixels, m_state->image_buffer_size_bytes());
  }
  MRPT_END
}

uint8_t* CImage::internal_get(int32_t col, int32_t row, uint8_t channel)
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  return m_state->image_data + m_state->row_stride_in_bytes() * static_cast<std::size_t>(row) +
         m_state->pixel_size_in_bytes() * static_cast<std::size_t>(col) + channel;
}
const uint8_t* CImage::internal_get(int32_t col, int32_t row, uint8_t channel) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  return m_state->image_data + m_state->row_stride_in_bytes() * static_cast<std::size_t>(row) +
         m_state->pixel_size_in_bytes() * static_cast<std::size_t>(col) + channel;
}

uint8_t CImage::serializeGetVersion() const { return 9; }
void CImage::serializeTo(mrpt::serialization::CArchive& out) const
{
  ASSERT_(m_state);

  // Added in version 6: possibility of being stored offline:
  out << m_state->imgIsExternalStorage;

  if (m_state->imgIsExternalStorage)
  {
    out << m_state->externalFile;
    return;
  }

  const bool hasColor = m_state->empty() ? false : isColor();

  out << hasColor;

  const int32_t width = static_cast<int32_t>(m_state->width);
  const int32_t height = static_cast<int32_t>(m_state->height);

  if (!hasColor)
  {
    // GRAY-SCALE: Raw bytes:
    // Version 3: ZIP compression
    // Version 4: Skip zip if the image size <= 16Kb
    const int32_t origin = 0;  // not used mrpt v1.9.9
    const uint32_t imageSize = static_cast<uint32_t>(m_state->image_buffer_size_bytes());
    // Version 10: depth
    const int32_t depth = static_cast<int32_t>(m_state->depth);

    out << width << height << origin << imageSize << depth;

    // Version 5: Use CImage::DISABLE_ZIP_COMPRESSION
    // Dec 2019: Remove this feature since it's not worth.
    // We still spend 1 byte for this constant bool just not to
    // bump the serialization number.
    bool imageStoredAsZip = false;

    out << imageStoredAsZip;

    if (!m_state->empty())
    {
      out.WriteBuffer(m_state->image_data, m_state->image_buffer_size_bytes());
    }
  }
  else
  {
    // COLOR: Save

    // v7: If size is 0xN or Nx0, don't call
    // "saveToStreamAsJPEG"!!

    // v8: If DISABLE_JPEG_COMPRESSION
    // (feature removed in v3.0.0)
    {  // (New in v8)
      // Don't JPEG-compress behavior:
      // Use negative image sizes to signal this behavior:
      const int32_t neg_width = -width;
      const int32_t neg_height = -height;

      out << neg_width << neg_height;

      // Dump raw image data:
      if (!m_state->empty())
      {
        out.WriteBuffer(m_state->image_data, m_state->image_buffer_size_bytes());
      }
    }
  }
}

void CImage::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  // First, free current image.
  clear();

  switch (version)
  {
    case 100:  // Saved from a legacy MRPT build without OpenCV:
    {
      in >> m_state->imgIsExternalStorage;
      if (m_state->imgIsExternalStorage)
      {
        in >> m_state->externalFile;
      }
    }
    break;
    case 0:
    {
      uint32_t width, height, nChannels, imgLength;
      uint8_t originTopLeft;

      in >> width >> height >> nChannels >> originTopLeft >> imgLength;

      resize(
          static_cast<int32_t>(width), static_cast<int32_t>(height),
          static_cast<TImageChannels>(nChannels));
      in.ReadBuffer(m_state->image_data, imgLength);
    }
    break;
    case 1:
    {
      // Version 1: High quality JPEG image
      uint32_t nBytes;
      in >> nBytes;
      std::vector<uint8_t> buf(nBytes);
      in.ReadBuffer(buf.data(), nBytes);

      mrpt::io::CMemoryStream aux;
      aux.assignMemoryNotOwn(buf.data(), buf.size());
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
      // Version 6: 	m_state->imgIsExternalStorage ??
      if (version >= 6)
      {
        in >> m_state->imgIsExternalStorage;
      }
      else
      {
        m_state->imgIsExternalStorage = false;
      }

      if (m_state->imgIsExternalStorage)
      {
        // Just the file name:
        in >> m_state->externalFile;
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
          resize(static_cast<int32_t>(width), static_cast<int32_t>(height), CH_GRAY, depth);
          ASSERT_EQUAL_(
              static_cast<uint32_t>(imageSize),
              static_cast<uint32_t>(m_state->image_buffer_size_bytes()));

          if (version == 2)
          {
            if (imageSize)
            {
              in.ReadBuffer(m_state->image_data, static_cast<std::size_t>(imageSize));
            }
          }
          else
          {
            // Version 3: ZIP compression!
            bool imageIsZIP = true;

            // Version 4: Skip zip if the image size <= 16Kb
            // Version 5: Use CImage::DISABLE_ZIP_COMPRESSION
            if (version == 4 && imageSize <= 16 * 1024) imageIsZIP = false;

            if (version >= 5)
            {
              // It is stored int the stream:
              in >> imageIsZIP;
            }

            if (imageIsZIP)
            {
              uint32_t zipDataLen;
              in >> zipDataLen;
              THROW_EXCEPTION("ZIP image deserialization not supported anymore");
            }
            else
            {
              // Raw bytes:
              if (imageSize)
              {
                in.ReadBuffer(m_state->image_data, static_cast<std::size_t>(imageSize));
              }
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
                const auto real_w = static_cast<int32_t>(-width);
                const auto real_h = static_cast<int32_t>(-height);

                resize(real_w, real_h, CH_RGB);

                const size_t bytes_per_row = m_state->row_stride_in_bytes();
                for (int32_t y = 0; y < m_state->height; y++)
                {
                  const size_t nRead = in.ReadBuffer(ptrLine<void>(y), bytes_per_row);
                  if (nRead != bytes_per_row)
                  {
                    THROW_EXCEPTION("Error: Truncated data stream while parsing raw image?");
                  }
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
            uint32_t nBytes;
            in >> nBytes;

            std::vector<uint8_t> buf(nBytes);
            in.ReadBuffer(buf.data(), nBytes);

            mrpt::io::CMemoryStream aux;
            aux.assignMemoryNotOwn(buf.data(), buf.size());
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

TImageSize CImage::getSize() const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  return {static_cast<int>(m_state->width), static_cast<int>(m_state->height)};
}

int32_t CImage::getWidth() const
{
  makeSureImageIsLoaded();
  return m_state->width;
}

std::string CImage::getChannelsOrder() const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  const auto chCount = m_state->channels;
  ASSERT_GE_(chCount, 1);
  ASSERT_LE_(chCount, 4);
  const std::array<const char*, 4> orderNames = {"GRAY", "", "BGR", "BGRA"};
  return std::string(orderNames.at(chCount - 1));
}

size_t CImage::getRowStride() const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  return m_state->row_stride_in_bytes();
}

int32_t CImage::getHeight() const
{
  makeSureImageIsLoaded();
  return m_state->height;
}

bool CImage::isColor() const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  return m_state->channels >= 3;
}

TImageChannels CImage::channels() const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  return m_state->channels;
}

bool CImage::isEmpty() const { return !m_state->imgIsExternalStorage && m_state->empty(); }

bool CImage::isOriginTopLeft() const
{
  return true;  // As of mrpt v1.9.9
}

float CImage::getAsFloat(const TPixelCoord& pt, uint8_t channel) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  // [0,255]->[0,1]
  return at<uint8_t>(pt.x, pt.y, channel) / 255.0f;
}

float CImage::getAsFloat(const TPixelCoord& pt) const
{
  // Is a RGB image??
  if (isColor())
  {
    // Luminance: Y = 0.3R + 0.59G + 0.11B
    const uint8_t* pixels = ptr<uint8_t>(pt.x, pt.y, 0);
    return (pixels[0] * 0.3f + pixels[1] * 0.59f + pixels[2] * 0.11f) / 255.0f;
  }
  else
  {
    // [0,255]->[0,1]
    return at<uint8_t>(pt.x, pt.y, 0) / 255.0f;
  }
}

CImage CImage::grayscale() const
{
  CImage ret;
  grayscale(ret);
  return ret;
}

bool CImage::grayscale(CImage& ret) const
{
  // The image is already grayscale??
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  if (m_state->img.channels() == 1)
  {
    ret = *this;  // shallow copy
    return true;
  }
  else
  {
    // Convert to a single luminance channel image
    cv::Mat src = m_state->img;
    // Detect in-place op and make deep copy:
    if (src.data == ret.m_state->img.data) src = src.clone();

    return my_img_to_grayscale(src, ret.m_impl->img);

    if (dest.size() != src.size() || dest.type() != src.type())
      dest = cv::Mat(src.rows, src.cols, CV_8UC1);

    // If possible, use SSE optimized version:
#if MRPT_ARCH_INTEL_COMPATIBLE
    if ((src.step[0] & 0x0f) == 0 && (dest.step[0] & 0x0f) == 0 &&
        mrpt::cpu::supports(mrpt::cpu::feature::SSSE3))
    {
      image_SSSE3_bgr_to_gray_8u(
          src.ptr<uint8_t>(), dest.ptr<uint8_t>(), src.cols, src.rows, src.step[0], dest.step[0]);
      return true;
    }
#endif

    // OpenCV Method:
    cv::cvtColor(src, dest, cv::COLOR_BGR2GRAY);
    return false;
  }
}

bool CImage::scaleHalf(CImage& out, TInterpolationMethod interp) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  // Get this image size:
  auto& img = m_state->img;
  const int w = img.cols, h = img.rows;

  // Create target image:
  out.resize(w >> 1, h >> 1, getChannelCount());
  auto& img_out = out.m_state->img;

// If possible, use SSE optimized version:
#if MRPT_ARCH_INTEL_COMPATIBLE
  if (img.channels() == 3 && interp == IMG_INTERP_NN &&
      mrpt::cpu::supports(mrpt::cpu::feature::SSSE3))
  {
    image_SSSE3_scale_half_3c8u(img.data, img_out.data, w, h, img.step[0], img_out.step[0]);
    return true;
  }

  if (img.channels() == 1 && mrpt::cpu::supports(mrpt::cpu::feature::SSE2))
  {
    if (interp == IMG_INTERP_NN)
    {
      image_SSE2_scale_half_1c8u(img.data, img_out.data, w, h, img.step[0], img_out.step[0]);
      return true;
    }
    else if (interp == IMG_INTERP_LINEAR)
    {
      image_SSE2_scale_half_smooth_1c8u(img.data, img_out.data, w, h, img.step[0], img_out.step[0]);
      return true;
    }
  }
#endif

  // Fall back to slow method:
  cv::resize(img, img_out, img_out.size(), 0, 0, interpolationMethod2Cv(interp));
  return false;
}

void CImage::scaleDouble(CImage& out, TInterpolationMethod interp) const
{
  out = *this;
  const TImageSize siz = this->getSize();
  out.scaleImage(out, siz.x * 2, siz.y * 2, interp);
}

void CImage::setPixel(const TPixelCoord& pt, const mrpt::img::TColor& color)
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  ASSERT_(this->getPixelDepth() == mrpt::img::PixelDepth::D8U);

  if (pt.x < 0 || pt.y < 0 || pt.y >= m_state->height || x >= m_state->width)
  {
    return;  // Invalid pixel coordinates
  }

  if (img.channels() == 1)
  {
    img.ptr<uint8_t>(y)[x] = color.R;
  }
  else
  {
#if defined(_DEBUG)
    ASSERT_(img.channels() == 3);
#endif
    auto* dest = &img.ptr<uint8_t>(y)[3 * x];
    *dest++ = color.R;
    *dest++ = color.G;  // G
    *dest++ = color.B;  // B
  }
}

void CImage::filledRectangle(int x0, int y0, int x1, int y1, const mrpt::img::TColor color)
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  cv::rectangle(m_impl->img, {x0, y0}, {x1, y1}, CV_RGB(color.R, color.G, color.B), cv::FILLED);
}

void CImage::drawImage(int x, int y, const mrpt::img::CImage& img)
{
  makeSureImageIsLoaded();
  img.makeSureImageIsLoaded();

  cv::Rect roi(cv::Point(x, y), cv::Size(img.getWidth(), img.getHeight()));
  cv::Mat dest = m_impl->img(roi);
  img.m_state->img.copyTo(dest);
}

void CImage::extract_patch(
    CImage& patch, const TPixelCoord& top_left_corner, const TImageSize& patch_size) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  const auto& src = m_state->img;
  auto& dest = patch.m_state->img;

  src(cv::Rect(col_, row_, col_num, row_num)).copyTo(dest);
}

float CImage::correlate(const CImage& img2, int width_init, int height_init) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  if ((img2.getWidth() + width_init > getWidth()) | (img2.getHeight() + height_init > getHeight()))
    THROW_EXCEPTION("Correlation Error!, image to correlate out of bounds");

  float x1, x2;
  float syy = 0.0f, sxy = 0.0f, sxx = 0.0f, m1 = 0.0f, m2 = 0.0f,
        n = (float)(img2.getHeight() * img2.getWidth());

  // find the means
  for (size_t i = 0; i < img2.getHeight(); i++)
  {
    for (size_t j = 0; j < img2.getWidth(); j++)
    {
      m1 += *(*this)(j + width_init, i + height_init);
      m2 += *img2(j, i);
    }
  }
  m1 /= n;
  m2 /= n;

  for (size_t i = 0; i < img2.getHeight(); i++)
  {
    for (size_t j = 0; j < img2.getWidth(); j++)
    {
      x1 = *(*this)(j + width_init, i + height_init) - m1;
      x2 = *img2(j, i) - m2;
      sxx += x1 * x1;
      syy += x2 * x2;
      sxy += x1 * x2;
    }
  }

  return sxy / sqrt(sxx * syy);
}

void CImage::normalize()
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  cv::normalize(m_impl->img, m_impl->img, 255, 0, cv::NORM_MINMAX);
}

void CImage::getAsMatrix(
    CMatrixFloat& outMatrix,
    bool doResize,
    int x_min,
    int y_min,
    int x_max,
    int y_max,
    bool normalize_01) const
{
  MRPT_START
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  const auto& img = m_state->img;

  // Set sizes:
  if (x_max == -1) x_max = img.cols - 1;
  if (y_max == -1) y_max = img.rows - 1;

  ASSERT_(x_min >= 0 && x_min < img.cols && x_min < x_max);
  ASSERT_(y_min >= 0 && y_min < img.rows && y_min < y_max);

  int lx = (x_max - x_min + 1);
  int ly = (y_max - y_min + 1);

  if (doResize || outMatrix.rows() < ly || outMatrix.cols() < lx)
    outMatrix.setSize(y_max - y_min + 1, x_max - x_min + 1);

  const bool is_color = isColor();

  // Luminance: Y = 0.3R + 0.59G + 0.11B
  for (int y = 0; y < ly; y++)
  {
    const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y);
    for (int x = 0; x < lx; x++)
    {
      float aux;
      if (is_color)
      {
        aux = *pixels++ * 0.3f;
        aux += *pixels++ * 0.59f;
        aux += *pixels++ * 0.11f;
      }
      else
      {
        aux = (*pixels++);
      }
      outMatrix.coeffRef(y, x) = aux;
    }
  }
  if (normalize_01) outMatrix *= (1.0f / 255);

  MRPT_END
}

void CImage::getAsMatrix(
    CMatrix_u8& outMatrix, bool doResize, int x_min, int y_min, int x_max, int y_max) const
{
  MRPT_START
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  const auto& img = m_state->img;

  // Set sizes:
  if (x_max == -1) x_max = img.cols - 1;
  if (y_max == -1) y_max = img.rows - 1;

  ASSERT_(x_min >= 0 && x_min < img.cols && x_min < x_max);
  ASSERT_(y_min >= 0 && y_min < img.rows && y_min < y_max);

  int lx = (x_max - x_min + 1);
  int ly = (y_max - y_min + 1);

  if (doResize || outMatrix.rows() < ly || outMatrix.cols() < lx)
    outMatrix.setSize(y_max - y_min + 1, x_max - x_min + 1);

  const bool is_color = isColor();

  // Luminance: Y = 0.3R + 0.59G + 0.11B
  for (int y = 0; y < ly; y++)
  {
    const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y);
    for (int x = 0; x < lx; x++)
    {
      if (is_color)
      {
        unsigned int aux = *pixels++ * 3000;  // 0.3f;
        aux += *pixels++ * 5900;              // 0.59f;
        aux += *pixels++ * 1100;              // 0.11f;
        outMatrix.coeffRef(y, x) = static_cast<uint8_t>(aux / 1000);
      }
      else
      {
        outMatrix.coeffRef(y, x) = (*pixels++);
      }
    }
  }

  MRPT_END
}

void CImage::getAsRGBMatrices(
    mrpt::math::CMatrixFloat& R,
    mrpt::math::CMatrixFloat& G,
    mrpt::math::CMatrixFloat& B,
    bool doResize,
    int x_min,
    int y_min,
    int x_max,
    int y_max) const
{
  MRPT_START

  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  const auto& img = m_state->img;

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

  const bool is_color = isColor();
  for (int y = 0; y < ly; y++)
  {
    const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y);
    for (int x = 0; x < lx; x++)
    {
      if (is_color)
      {
        R.coeffRef(y, x) = u8tof(*pixels++);
        G.coeffRef(y, x) = u8tof(*pixels++);
        B.coeffRef(y, x) = u8tof(*pixels++);
      }
      else
      {
        R.coeffRef(y, x) = G.coeffRef(y, x) = B.coeffRef(y, x) = u8tof(*pixels++);
      }
    }
  }

  MRPT_END
}

void CImage::getAsRGBMatrices(
    mrpt::math::CMatrix_u8& R,
    mrpt::math::CMatrix_u8& G,
    mrpt::math::CMatrix_u8& B,
    bool doResize,
    int x_min,
    int y_min,
    int x_max,
    int y_max) const
{
  MRPT_START

  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  const auto& img = m_state->img;

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

  const bool is_color = isColor();
  for (int y = 0; y < ly; y++)
  {
    const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y);
    for (int x = 0; x < lx; x++)
    {
      if (is_color)
      {
        R.coeffRef(y, x) = *pixels++;
        G.coeffRef(y, x) = *pixels++;
        B.coeffRef(y, x) = *pixels++;
      }
      else
      {
        R.coeffRef(y, x) = G.coeffRef(y, x) = B.coeffRef(y, x) = *pixels++;
      }
    }
  }

  MRPT_END
}

void CImage::cross_correlation_FFT(
    const CImage& in_img,
    CMatrixFloat& out_corr,
    int u_search_ini,
    int v_search_ini,
    int u_search_size,
    int v_search_size,
    float biasThisImg,
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
  size_t actual_lx = std::max(static_cast<size_t>(u_search_size), in_img.getWidth());
  size_t actual_ly = std::max(static_cast<size_t>(v_search_size), in_img.getHeight());
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
  MRPT_START

  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  const auto& img = m_state->img;

  // The size of the matrix:
  const auto matrix_lx = outMatrix.cols();
  const auto matrix_ly = outMatrix.rows();

  if (isColor())
  {
    // Luminance: Y = 0.3R + 0.59G + 0.11B
    for (CMatrixFloat::Index y = 0; y < matrix_ly; y++)
    {
      uint8_t* min_pixels = (*this)(0, y % img.rows, 0);
      uint8_t* max_pixels = min_pixels + img.cols * 3;
      uint8_t* pixels = min_pixels;
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
      uint8_t* min_pixels = (*this)(0, y % img.rows, 0);
      uint8_t* max_pixels = min_pixels + img.cols;
      uint8_t* pixels = min_pixels;
      for (CMatrixFloat::Index x = 0; x < matrix_lx; x++)
      {
        outMatrix(y, x) = *pixels++;
        if (pixels >= max_pixels) pixels = min_pixels;
      }
    }
  }

  MRPT_END
}

void CImage::clear()
{
  // Reset to defaults:
  *this = CImage();
}

void CImage::setExternalStorage(const std::string& fileName) noexcept
{
  clear();
  m_state->externalFile = fileName;
  m_state->imgIsExternalStorage = true;
}

void CImage::unload() const noexcept
{
  if (m_state->imgIsExternalStorage)
  {
    if (MRPT_DEBUG_IMG_LAZY_LOAD)
      std::cout << "[CImage::unload()] Called on this=" << reinterpret_cast<const void*>(this)
                << std::endl;

    const_cast<cv::Mat&>(m_impl->img) = cv::Mat();
  }
}

void CImage::makeSureImageIsLoaded(bool allowNonInitialized) const
{
  if (!m_state->img.empty()) return;  // OK, continue

  if (m_state->imgIsExternalStorage)
  {
    // Load the file:
    string wholeFile;
    getExternalStorageFileAbsolutePath(wholeFile);

    const std::string tmpFile = m_state->externalFile;

    bool ret = const_cast<CImage*>(this)->loadFromFile(wholeFile);

    // These are removed by "loadFromFile", and that's good, just fix it
    // here and carry on.
    m_state->imgIsExternalStorage = true;
    m_state->externalFile = tmpFile;

    if (!ret)
    {
      THROW_TYPED_EXCEPTION_FMT(
          CExceptionExternalImageNotFound, "Error loading externally-stored image from: %s",
          wholeFile.c_str());
    }

    if (MRPT_DEBUG_IMG_LAZY_LOAD)
    {
      std::cout << "[CImage] Loaded lazy-load image file '" << wholeFile
                << "' on this=" << reinterpret_cast<const void*>(this) << std::endl;
    }
  }
  else if (!allowNonInitialized)
  {
    THROW_EXCEPTION("Trying to access uninitialized image in a non externally-stored image.");
  }
}

void CImage::getExternalStorageFileAbsolutePath(std::string& out_path) const
{
  out_path = mrpt::io::lazy_load_absolute_path(m_state->externalFile);
}

void CImage::flipVertical()
{
  makeSureImageIsLoaded();
  cv::flip(m_impl->img, m_impl->img, 0 /* x-axis */);
}

void CImage::flipHorizontal()
{
  makeSureImageIsLoaded();
  cv::flip(m_impl->img, m_impl->img, 1 /* y-axis */);
}

void CImage::swapRB()
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  cv::cvtColor(m_impl->img, m_impl->img, cv::COLOR_RGB2BGR);
}

void CImage::rectifyImageInPlace(void* mapX, void* mapY)
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  auto& srcImg = m_state->img;
  cv::Mat outImg(srcImg.rows, srcImg.cols, srcImg.type());

  auto mapXm = static_cast<cv::Mat*>(mapX);
  auto mapYm = static_cast<cv::Mat*>(mapX);

  cv::remap(srcImg, outImg, *mapXm, *mapYm, cv::INTER_CUBIC);

  clear();
  srcImg = outImg;
}

void CImage::undistort(CImage& out_img, const mrpt::img::TCamera& cameraParams) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  ASSERTMSG_(out_img.m_impl->img.data != m_impl->img.data, "In-place undistort() not supported");

  auto& srcImg = const_cast<cv::Mat&>(m_state->img);
  // This will avoid re-alloc if size already matches.
  out_img.resize(srcImg.cols, srcImg.rows, getChannelCount());

  const auto& intrMat = cameraParams.intrinsicParams;
  const auto& dist = cameraParams.dist;

  cv::Mat distM(1, dist.size(), CV_64F, const_cast<double*>(&dist[0]));
  cv::Mat inMat(3, 3, CV_64F);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) inMat.at<double>(i, j) = intrMat(i, j);

  cv::undistort(srcImg, out_img.m_impl->img, inMat, distM);
}

void CImage::filterMedian(CImage& out_img, int W) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  auto srcImg = const_cast<cv::Mat&>(m_state->img);
  if (this == &out_img)
    srcImg = srcImg.clone();
  else
    out_img.resize(srcImg.cols, srcImg.rows, getChannelCount());

  cv::medianBlur(srcImg, out_img.m_impl->img, W);
}

void CImage::filterGaussian(CImage& out_img, int W, int H, double sigma) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  auto srcImg = const_cast<cv::Mat&>(m_state->img);
  if (this == &out_img)
    srcImg = srcImg.clone();
  else
    out_img.resize(srcImg.cols, srcImg.rows, getChannelCount());

  cv::GaussianBlur(srcImg, out_img.m_impl->img, cv::Size(W, H), sigma);
}

void CImage::scaleImage(
    CImage& out_img, unsigned int width, unsigned int height, TInterpolationMethod interp) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  auto srcImg = m_state->img;
  // Detect in-place operation and make a deep copy if needed:
  if (out_img.m_state->img.data == srcImg.data) srcImg = srcImg.clone();

  // Already done?
  if (srcImg.cols == static_cast<int>(width) && srcImg.rows == static_cast<int>(height))
  {
    out_img.m_state->img = srcImg;
    return;
  }
  out_img.resize(width, height, getChannelCount());

  // Resize:
  cv::resize(
      srcImg, out_img.m_impl->img, out_img.m_impl->img.size(), 0, 0,
      interpolationMethod2Cv(interp));
}

void CImage::rotateImage(CImage& out_img, double ang, const TPixelCoord& center, double scale) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  auto srcImg = m_state->img;
  // Detect in-place operation and make a deep copy if needed:
  if (out_img.m_state->img.data == srcImg.data) srcImg = srcImg.clone();

  // quick rotation?
  if (std::abs(M_PI * 0.5 - std::abs(ang)) < 1e-3 || std::abs(M_PI - std::abs(ang)) < 1e-3)
  {
    int rotCode = 0;
    if (std::abs(M_PI * 0.5 - ang) < 1e-3)
      rotCode = cv::ROTATE_90_COUNTERCLOCKWISE;
    else if (std::abs(-M_PI * 0.5 - ang) < 1e-3)
      rotCode = cv::ROTATE_90_CLOCKWISE;
    else if (std::abs(M_PI - ang) < 1e-3)
      rotCode = cv::ROTATE_180;

    cv::rotate(srcImg, out_img.m_impl->img, rotCode);
    return;
  }
  // else: general rotation:

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
}

bool CImage::drawChessboardCorners(
    const std::vector<TPixelCoordf>& cornerCoords,
    unsigned int check_size_x,
    unsigned int check_size_y,
    unsigned int lines_width,
    unsigned int r)
{
  if (cornerCoords.size() != check_size_x * check_size_y) return false;

  auto& img = m_state->img;

  unsigned int x, y, i;
  cv::Point prev_pt = cvPoint(0, 0);
  const int line_max = 8;
  cv::Scalar line_colors[8];

  line_colors[0] = CV_RGB(255, 0, 0);
  line_colors[1] = CV_RGB(255, 128, 0);
  line_colors[2] = CV_RGB(255, 128, 0);
  line_colors[3] = CV_RGB(200, 200, 0);
  line_colors[4] = CV_RGB(0, 255, 0);
  line_colors[5] = CV_RGB(0, 200, 200);
  line_colors[6] = CV_RGB(0, 0, 255);
  line_colors[7] = CV_RGB(255, 0, 255);

  CCanvas::selectTextFont("10x20");

  for (y = 0, i = 0; y < check_size_y; y++)
  {
    const auto color = line_colors[y % line_max];
    for (x = 0; x < check_size_x; x++, i++)
    {
      cv::Point pt;
      pt.x = cvRound(cornerCoords[i].x);
      pt.y = cvRound(cornerCoords[i].y);

      if (i != 0) cv::line(img, prev_pt, pt, color, lines_width);

      cv::line(img, cvPoint(pt.x - r, pt.y - r), cvPoint(pt.x + r, pt.y + r), color, lines_width);
      cv::line(img, cvPoint(pt.x - r, pt.y + r), cvPoint(pt.x + r, pt.y - r), color, lines_width);

      if (r > 0) cv::circle(img, pt, r + 1, color);
      prev_pt = pt;

      // Text label with the corner index in the first and last
      // corners:
      if (i == 0 || i == cornerCoords.size() - 1)
        CCanvas::textOut(pt.x + 5, pt.y - 5, mrpt::format("%u", i), mrpt::img::TColor::blue());
    }
  }

  return true;
}

CImage CImage::colorImage() const
{
  CImage ret;
  colorImage(ret);
  return ret;
}

void CImage::colorImage(CImage& ret) const
{
  if (this->isColor())
  {
    if (&ret != this) ret = *this;
    return;
  }

  auto srcImg = m_state->img;
  // Detect in-place op. and make deep copy:
  if (srcImg.data == ret.m_state->img.data) srcImg = srcImg.clone();

  ret.resize(getWidth(), getHeight(), CH_RGB);

  cv::cvtColor(srcImg, ret.m_impl->img, cv::COLOR_GRAY2BGR);
}

void CImage::joinImagesHorz(const CImage& img1, const CImage& img2)
{
  ASSERT_(img1.getHeight() == img2.getHeight());

  auto im1 = img1.m_state->img, im2 = img2.m_state->img;
  ASSERT_(im1.type() == im2.type());

  this->resize(im1.cols + im2.cols, im1.rows, img1.getChannelCount());

  im1.copyTo(m_impl->img(cv::Rect(0, 0, im1.cols, im1.rows)));
  im2.copyTo(m_impl->img(cv::Rect(im1.cols, 0, im2.cols, im2.rows)));
}  // end

template <unsigned int HALF_WIN_SIZE>
void image_KLT_response_template(
    const cv::Mat& im, unsigned int x, unsigned int y, int32_t& _gxx, int32_t& _gyy, int32_t& _gxy)
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
    unsigned int xx = min_x;
    for (unsigned int ix = WIN_SIZE; ix; --ix, ++xx)
    {
      const int32_t dx = static_cast<int32_t>(im.at<uint8_t>(yy, xx + 1)) -
                         static_cast<int32_t>(im.at<uint8_t>(yy, xx - 1));
      const int32_t dy = static_cast<int32_t>(im.at<uint8_t>(yy + 1, xx)) -
                         static_cast<int32_t>(im.at<uint8_t>(yy - 1, xx));
      gxx += dx * dx;
      gxy += dx * dy;
      gyy += dy * dy;
    }
  }
  _gxx = gxx;
  _gyy = gyy;
  _gxy = gxy;
}

float CImage::KLT_response(const TPixelCoord& pt, const int32_t half_window_size) const
{
  const auto& im1 = m_state->img;
  const auto img_w = static_cast<unsigned int>(im1.cols),
             img_h = static_cast<unsigned int>(im1.rows);

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

  switch (half_window_size)
  {
    case 2:
      image_KLT_response_template<2>(im1, x, y, gxx, gyy, gxy);
      break;
    case 3:
      image_KLT_response_template<3>(im1, x, y, gxx, gyy, gxy);
      break;
    case 4:
      image_KLT_response_template<4>(im1, x, y, gxx, gyy, gxy);
      break;
    case 5:
      image_KLT_response_template<5>(im1, x, y, gxx, gyy, gxy);
      break;
    case 6:
      image_KLT_response_template<6>(im1, x, y, gxx, gyy, gxy);
      break;
    case 7:
      image_KLT_response_template<7>(im1, x, y, gxx, gyy, gxy);
      break;
    case 8:
      image_KLT_response_template<8>(im1, x, y, gxx, gyy, gxy);
      break;
    case 9:
      image_KLT_response_template<9>(im1, x, y, gxx, gyy, gxy);
      break;
    case 10:
      image_KLT_response_template<10>(im1, x, y, gxx, gyy, gxy);
      break;
    case 11:
      image_KLT_response_template<11>(im1, x, y, gxx, gyy, gxy);
      break;
    case 12:
      image_KLT_response_template<12>(im1, x, y, gxx, gyy, gxy);
      break;
    case 13:
      image_KLT_response_template<13>(im1, x, y, gxx, gyy, gxy);
      break;
    case 14:
      image_KLT_response_template<14>(im1, x, y, gxx, gyy, gxy);
      break;
    case 15:
      image_KLT_response_template<15>(im1, x, y, gxx, gyy, gxy);
      break;
    case 16:
      image_KLT_response_template<16>(im1, x, y, gxx, gyy, gxy);
      break;
    case 32:
      image_KLT_response_template<32>(im1, x, y, gxx, gyy, gxy);
      break;

    default:
      for (unsigned int yy = min_y; yy <= max_y; yy++)
      {
        for (unsigned int xx = min_x; xx <= max_x; xx++)
        {
          const int32_t dx = static_cast<int32_t>(im1.at<uint8_t>(yy, xx + 1)) -
                             static_cast<int32_t>(im1.at<uint8_t>(yy, xx - 1));
          const int32_t dy = static_cast<int32_t>(im1.at<uint8_t>(yy + 1, xx)) -
                             static_cast<int32_t>(im1.at<uint8_t>(yy - 1, xx));

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
  const float t = Gxx + Gyy;               // Trace
  const float de = Gxx * Gyy - Gxy * Gxy;  // Det
  const float discr = std::max<float>(.0f, t * t - 4.0f * de);
  // The smallest eigenvalue is:
  return 0.5f * (t - std::sqrt(discr));
}

std::ostream& operator<<(std::ostream& o, const TPixelCoordf& p)
{
  o << "(" << p.x << "," << p.y << ")";
  return o;
}
std::ostream& operator<<(std::ostream& o, const TPixelCoord& p)
{
  o << "(" << p.x << "," << p.y << ")";
  return o;
}

}  // namespace mrpt::img