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

#include <mrpt/core/get_env.h>
#include <mrpt/core/round.h>
#include <mrpt/img/CImage.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/fourier.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

#include "CImage_impl.h"

// STB library includes
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

namespace
{
const thread_local bool MRPT_DEBUG_IMG_LAZY_LOAD =
    mrpt::get_env<bool>("MRPT_DEBUG_IMG_LAZY_LOAD", false);

stbir_pixel_layout mrpt_image_channel_to_stbir_layout(const mrpt::img::TImageChannels channels)
{
  using mrpt::img::TImageChannels;

  switch (channels)
  {
    case TImageChannels::CH_GRAY:
      return STBIR_1CHANNEL;

    case TImageChannels::CH_RGB:
      return STBIR_RGB;

    case TImageChannels::CH_RGBA:
      return STBIR_RGBA;

    default:
      THROW_EXCEPTION("Invalid TImageChannels value");
  };
}

stbir_datatype mrpt_pixel_depth_to_stbir_type(const mrpt::img::PixelDepth depth)
{
  using mrpt::img::PixelDepth;

  switch (depth)
  {
    case PixelDepth::D8U:
      return STBIR_TYPE_UINT8;

    case PixelDepth::D16U:
      return STBIR_TYPE_UINT16;

    default:
      THROW_EXCEPTION("Invalid PixelDepth value");
  };
}

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

  // If we're resizing to exactly the current size, do nothing:
  if (m_state->width == width && m_state->height == height && m_state->channels == nChannels &&
      m_state->depth == depth)
  {
    return;
  }

  m_state->clear();
  m_state->width = width;
  m_state->height = height;
  m_state->channels = nChannels;
  m_state->depth = depth;

  const auto num_bytes = m_state->image_buffer_size_bytes();
  m_state->image_data = reinterpret_cast<uint8_t*>(std::malloc(num_bytes));

  if (!m_state->image_data)
  {
    THROW_EXCEPTION("Failed to allocate image memory");
  }

  MRPT_END
}

PixelDepth CImage::getPixelDepth() const
{
  MRPT_START
  makeSureImageIsLoaded();
  return m_state->depth;
  MRPT_END
}

mrpt::img::CImage CImage::LoadFromFile(const std::string& fileName, TImageChannels loadChannels)
{
  CImage im;
  bool ok = im.loadFromFile(fileName, loadChannels);
  if (!ok)
  {
    THROW_EXCEPTION_FMT("Error loading image from '%s'", fileName.c_str());
  }
  return im;
}

bool CImage::loadFromFile(const std::string& fileName, TImageChannels loadChannels)
{
  MRPT_START

  m_state->clear();

  int width = 0;
  int height = 0;
  int original_channels = 0;
  unsigned char* image_data = nullptr;

  // stbi_load loads the image from the file specified by filename.
  // desired_channels: 0=load original, 1=grayscale, 3=RGB, 4=RGBA
  int desired_channels = static_cast<int>(loadChannels);

  image_data = stbi_load(fileName.c_str(), &width, &height, &original_channels, desired_channels);

  if (image_data == nullptr)
  {
    // Image loading failed
    if (MRPT_DEBUG_IMG_LAZY_LOAD)
    {
      std::cerr << "[CImage::loadFromFile] Failed to load: " << fileName
                << " Reason: " << stbi_failure_reason() << std::endl;
    }
    return false;
  }

  // Store in image container:
  m_state->width = width;
  m_state->height = height;
  m_state->depth = PixelDepth::D8U;
  m_state->image_data = image_data;

  // Determine actual channels loaded
  if (loadChannels == CH_AS_IS)
  {
    m_state->channels = static_cast<TImageChannels>(original_channels);
  }
  else
  {
    m_state->channels = loadChannels;
  }

  return true;
  MRPT_END
}

bool CImage::saveToFile(const std::string& fileName, int jpeg_quality) const
{
  MRPT_START
  makeSureImageIsLoaded();
  ASSERT_(!m_state->empty());

  const auto ext = mrpt::system::lowerCase(mrpt::system::extractFileExtension(fileName));

  const int w = static_cast<int>(m_state->width);
  const int h = static_cast<int>(m_state->height);
  const int comp = static_cast<int>(m_state->channels);
  const int stride = static_cast<int>(m_state->row_stride_in_bytes());

  if (ext == "jpg" || ext == "jpeg")
  {
    return 0 != stbi_write_jpg(fileName.c_str(), w, h, comp, m_state->image_data, jpeg_quality);
  }
  if (ext == "png")
  {
    return 0 != stbi_write_png(fileName.c_str(), w, h, comp, m_state->image_data, stride);
  }
  if (ext == "bmp")
  {
    return 0 != stbi_write_bmp(fileName.c_str(), w, h, comp, m_state->image_data);
  }
  if (ext == "tga")
  {
    return 0 != stbi_write_tga(fileName.c_str(), w, h, comp, m_state->image_data);
  }

  THROW_EXCEPTION_FMT(
      "Unknown image format extension '%s' (file: '%s')", ext.c_str(), fileName.c_str());

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

  // const auto bytes_per_row = m_state->row_stride_in_bytes();

  if (color_channels >= 3 && swapRedBlue)
  {
    // Do copy & swap at once (RGB <-> BGR):
    uint8_t* ptr_src = rawpixels;
    auto* ptr_dest = m_state->image_data;

    for (int32_t h = 0; h < height; h++)
    {
      for (int32_t w = 0; w < width; w++)
      {
        const uint8_t t0 = ptr_src[0];
        const uint8_t t1 = ptr_src[1];
        const uint8_t t2 = ptr_src[2];
        ptr_dest[0] = t2;  // Swap R and B
        ptr_dest[1] = t1;
        ptr_dest[2] = t0;

        if (color_channels == 4)
        {
          ptr_dest[3] = ptr_src[3];  // Alpha channel
          ptr_src += 4;
          ptr_dest += 4;
        }
        else
        {
          ptr_src += 3;
          ptr_dest += 3;
        }
      }
    }
  }
  else
  {
    // Direct copy:
    std::memcpy(m_state->image_data, rawpixels, m_state->image_buffer_size_bytes());
  }

  MRPT_END
}

uint8_t* CImage::internal_get(int32_t col, int32_t row, int8_t channel)
{
  makeSureImageIsLoaded();
  return m_state->image_data + m_state->row_stride_in_bytes() * static_cast<std::size_t>(row) +
         m_state->pixel_size_in_bytes() * static_cast<std::size_t>(col) +
         static_cast<std::size_t>(channel) * static_cast<std::size_t>(m_state->depth);
}

const uint8_t* CImage::internal_get(int32_t col, int32_t row, int8_t channel) const
{
  makeSureImageIsLoaded();
  return m_state->image_data + m_state->row_stride_in_bytes() * static_cast<std::size_t>(row) +
         m_state->pixel_size_in_bytes() * static_cast<std::size_t>(col) +
         static_cast<std::size_t>(channel) * static_cast<std::size_t>(m_state->depth);
}

uint8_t CImage::serializeGetVersion() const { return 10; }

void CImage::serializeTo(mrpt::serialization::CArchive& out) const
{
  ASSERT_(m_state);

  // External storage mode?
  out << m_state->imgIsExternalStorage;

  if (m_state->imgIsExternalStorage)
  {
    out << m_state->externalFile;
    return;
  }

  // Write image properties
  const bool hasColor = m_state->empty() ? false : isColor();
  out << hasColor;

  const int32_t width = m_state->width;
  const int32_t height = m_state->height;
  const int32_t depth = static_cast<int32_t>(m_state->depth);

  if (!hasColor)
  {
    // GRAYSCALE
    const uint32_t imageSize = static_cast<uint32_t>(m_state->image_buffer_size_bytes());
    out << width << height << imageSize << depth;

    // Raw bytes (no compression for now in v3.0.0)
    if (!m_state->empty())
    {
      out.WriteBuffer(m_state->image_data, m_state->image_buffer_size_bytes());
    }
  }
  else
  {
    // COLOR (RGB/RGBA)
    out << width << height;

    // Raw bytes
    if (!m_state->empty())
    {
      out.WriteBuffer(m_state->image_data, m_state->image_buffer_size_bytes());
    }
  }
}

void CImage::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  clear();

  switch (version)
  {
    case 10:  // MRPT 3.0.0 format
    {
      in >> m_state->imgIsExternalStorage;

      if (m_state->imgIsExternalStorage)
      {
        in >> m_state->externalFile;
      }
      else
      {
        bool hasColor;
        in >> hasColor;

        if (!hasColor)
        {
          // GRAYSCALE
          int32_t width, height;
          uint32_t imageSize;
          int32_t depth_int;

          in >> width >> height >> imageSize >> depth_int;

          const auto depth = static_cast<PixelDepth>(depth_int);
          resize(width, height, CH_GRAY, depth);

          ASSERT_EQUAL_(imageSize, static_cast<uint32_t>(m_state->image_buffer_size_bytes()));

          if (imageSize > 0)
          {
            in.ReadBuffer(m_state->image_data, imageSize);
          }
        }
        else
        {
          // COLOR
          int32_t width, height;
          in >> width >> height;

          // For now, assume RGB (could be extended to detect RGBA)
          resize(width, height, CH_RGB);

          const auto imageSize = m_state->image_buffer_size_bytes();
          if (imageSize > 0)
          {
            in.ReadBuffer(m_state->image_data, imageSize);
          }
        }
      }
    }
    break;

    // Support older versions for backward compatibility
    case 6:
    case 7:
    case 8:
    case 9:
    {
      // Read with older format, then convert
      // This would require parsing the old OpenCV-based format
      // For now, throw an exception suggesting conversion
      THROW_EXCEPTION(
          "Reading CImage from MRPT <3.0 format not yet implemented. "
          "Please convert images using MRPT 2.x first.");
    }
    break;

    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

#if MRPT_HAS_MATLAB
IMPLEMENTS_MEXPLUS_FROM(mrpt::img::CImage)
#endif

mxArray* CImage::writeToMatlab() const
{
#if MRPT_HAS_MATLAB
  // TODO: Implement MATLAB export
  THROW_EXCEPTION("MATLAB export not yet implemented for STB-based CImage");
#else
  THROW_EXCEPTION("MRPT built without MATLAB/Mex support");
#endif
}

TImageSize CImage::getSize() const
{
  makeSureImageIsLoaded();
  return {static_cast<int>(m_state->width), static_cast<int>(m_state->height)};
}

int32_t CImage::getWidth() const
{
  makeSureImageIsLoaded();
  return m_state->width;
}

std::string CImage::getChannelsOrder() const
{
  makeSureImageIsLoaded();

  const auto chCount = m_state->channels;
  ASSERT_GE_(chCount, 1);
  ASSERT_LE_(chCount, 4);

  const std::array<const char*, 4> orderNames = {"GRAY", "", "RGB", "RGBA"};
  return std::string(orderNames.at(chCount - 1));
}

size_t CImage::getRowStride() const
{
  makeSureImageIsLoaded();
  return m_state->row_stride_in_bytes();
}

int32_t CImage::getHeight() const
{
  makeSureImageIsLoaded();
  return m_state->height;
}

bool CImage::isColor() const
{
  makeSureImageIsLoaded();
  return m_state->channels >= 3;
}

TImageChannels CImage::channels() const
{
  makeSureImageIsLoaded();
  return m_state->channels;
}

bool CImage::isEmpty() const { return !m_state->imgIsExternalStorage && m_state->empty(); }

bool CImage::isOriginTopLeft() const { return true; }

float CImage::getAsFloat(const TPixelCoord& pt, int8_t channel) const
{
  makeSureImageIsLoaded();
  return static_cast<float>(at<uint8_t>(pt.x, pt.y, channel)) / 255.0f;
}

float CImage::getAsFloat(const TPixelCoord& pt) const
{
  makeSureImageIsLoaded();

  if (isColor())
  {
    // Luminance: Y = 0.299R + 0.587G + 0.114B
    const uint8_t* pixels = ptr<uint8_t>(pt.x, pt.y, 0);
    return (static_cast<float>(pixels[0]) * 0.299f + static_cast<float>(pixels[1]) * 0.587f +
            static_cast<float>(pixels[2]) * 0.114f) /
           255.0f;
  }

  return static_cast<float>(at<uint8_t>(pt.x, pt.y, 0)) / 255.0f;
}

CImage CImage::grayscale() const
{
  CImage ret;
  grayscale(ret);
  return ret;
}

bool CImage::grayscale(CImage& ret) const
{
  makeSureImageIsLoaded();

  if (m_state->channels == 1)
  {
    ret = *this;  // shallow copy
    return true;
  }

  // Convert to grayscale
  ret.resize(m_state->width, m_state->height, CH_GRAY, m_state->depth);

  const auto* src = m_state->image_data;
  auto* dst = ret.m_state->image_data;

  for (int32_t y = 0; y < m_state->height; y++)
  {
    for (int32_t x = 0; x < m_state->width; x++)
    {
      // Luminance formula: Y = 0.299R + 0.587G + 0.114B
      const float r = src[0];
      const float g = src[1];
      const float b = src[2];

      *dst = static_cast<uint8_t>(0.299f * r + 0.587f * g + 0.114f * b);

      src += m_state->channels;
      dst++;
    }
  }

  return false;  // No SSE optimization used
}

void CImage::scaleImage(
    CImage& out_img, int32_t width, int32_t height, TInterpolationMethod interp) const
{
  MRPT_START
  makeSureImageIsLoaded();

  // Check if already the right size
  if (m_state->width == width && m_state->height == height)
  {
    if (&out_img != this)
    {
      out_img = *this;
    }
    return;
  }

  // Prepare output image
  out_img.resize(width, height, m_state->channels, m_state->depth);

  // Map interpolation method to stbir
  stbir_filter filter;
  switch (interp)
  {
    case IMG_INTERP_NN:
      filter = STBIR_FILTER_BOX;
      break;
    case IMG_INTERP_LINEAR:
      filter = STBIR_FILTER_TRIANGLE;
      break;
    case IMG_INTERP_CUBIC:
      filter = STBIR_FILTER_CUBICBSPLINE;
      break;
    case IMG_INTERP_AREA:
      filter = STBIR_FILTER_BOX;
      break;
    default:
      filter = STBIR_FILTER_DEFAULT;
  }

  const auto filterEdge = STBIR_EDGE_CLAMP;

  // Perform resize using stbir
  stbir_resize(
      m_state->image_data, m_state->width, m_state->height,
      static_cast<int>(m_state->row_stride_in_bytes()), out_img.m_state->image_data, width, height,
      static_cast<int>(out_img.m_state->row_stride_in_bytes()),
      mrpt_image_channel_to_stbir_layout(m_state->channels),
      mrpt_pixel_depth_to_stbir_type(m_state->depth), filterEdge, filter);

  MRPT_END
}

bool CImage::scaleHalf(CImage& out_image, TInterpolationMethod interp) const
{
  scaleImage(out_image, m_state->width / 2, m_state->height / 2, interp);
  return false;  // No SSE optimization
}

void CImage::scaleDouble(CImage& out_image, TInterpolationMethod interp) const
{
  scaleImage(out_image, m_state->width * 2, m_state->height * 2, interp);
}

void CImage::setPixel(const TPixelCoord& pt, const mrpt::img::TColor& color)
{
  makeSureImageIsLoaded();

  if (pt.x < 0 || pt.y < 0 || pt.y >= m_state->height || pt.x >= m_state->width)
  {
    return;  // Out of bounds
  }

  if (m_state->channels == 1)
  {
    // Grayscale: use luminance
    const int y = 77 * static_cast<int>(color.R)     // 0.299 * 256 ≈ 77
                  + 150 * static_cast<int>(color.G)  // 0.587 * 256 ≈ 150
                  + 29 * static_cast<int>(color.B);  // 0.114 * 256 ≈ 29
    at<uint8_t>(pt.x, pt.y, 0) = static_cast<uint8_t>(y >> 8);
  }
  else if (m_state->channels == 3)
  {
    // RGB
    auto* pixel = ptr<uint8_t>(pt.x, pt.y, 0);
    pixel[0] = color.R;
    pixel[1] = color.G;
    pixel[2] = color.B;
  }
  else if (m_state->channels == 4)
  {
    // RGBA
    auto* pixel = ptr<uint8_t>(pt.x, pt.y, 0);
    pixel[0] = color.R;
    pixel[1] = color.G;
    pixel[2] = color.B;
    pixel[3] = color.A;
  }
}

void CImage::filledRectangle(
    const TPixelCoord& pt0, const TPixelCoord& pt1, const mrpt::img::TColor& color)
{
  makeSureImageIsLoaded();

  const int x0 = std::max(0, std::min(pt0.x, pt1.x));
  const int y0 = std::max(0, std::min(pt0.y, pt1.y));
  const int x1 = std::min(m_state->width - 1, std::max(pt0.x, pt1.x));
  const int y1 = std::min(m_state->height - 1, std::max(pt0.y, pt1.y));

  for (int y = y0; y <= y1; y++)
  {
    for (int x = x0; x <= x1; x++)
    {
      setPixel({x, y}, color);
    }
  }
}

void CImage::drawImage(const TPixelCoord& pt, const mrpt::img::CImage& img)
{
  makeSureImageIsLoaded();
  img.makeSureImageIsLoaded();

  const int dst_x = pt.x;
  const int dst_y = pt.y;
  const int src_w = img.getWidth();
  const int src_h = img.getHeight();

  // Clip to destination bounds
  for (int y = 0; y < src_h; y++)
  {
    const int dst_row = dst_y + y;
    if (dst_row < 0 || dst_row >= m_state->height)
    {
      continue;
    }

    for (int x = 0; x < src_w; x++)
    {
      const int dst_col = dst_x + x;
      if (dst_col < 0 || dst_col >= m_state->width)
      {
        continue;
      }

      // Copy pixel
      if (m_state->channels == img.m_state->channels)
      {
        std::memcpy(
            ptr<uint8_t>(dst_col, dst_row, 0), img.ptr<uint8_t>(x, y, 0),
            m_state->pixel_size_in_bytes());
      }
      else
      {
        // TODO: Handle channel conversion
        THROW_EXCEPTION("Case not implemented");
      }
    }
  }
}

void CImage::extract_patch(
    CImage& patch, const TPixelCoord& top_left_corner, const TImageSize& patch_size) const
{
  makeSureImageIsLoaded();

  patch.resize(patch_size.x, patch_size.y, m_state->channels, m_state->depth);

  const int x0 = top_left_corner.x;
  const int y0 = top_left_corner.y;

  for (int y = 0; y < patch_size.y; y++)
  {
    if (y0 + y >= m_state->height)
    {
      break;
    }

    std::memcpy(
        patch.ptrLine<uint8_t>(y),
        ptrLine<uint8_t>(y0 + y) + static_cast<size_t>(x0) * m_state->pixel_size_in_bytes(),
        static_cast<size_t>(patch_size.x) * m_state->pixel_size_in_bytes());
  }
}

void CImage::normalize()
{
  // TODO: Implement normalization
  THROW_EXCEPTION("normalize() not yet implemented with STB library");
}

void CImage::getAsMatrix(
    mrpt::math::CMatrixFloat& outMatrix,
    bool doResize,
    int x_min,
    int y_min,
    int x_max,
    int y_max,
    bool normalize_01) const
{
  MRPT_START
  makeSureImageIsLoaded();

  if (x_max == -1)
  {
    x_max = m_state->width - 1;
  }
  if (y_max == -1)
  {
    y_max = m_state->height - 1;
  }

  ASSERT_(x_min >= 0 && x_min < m_state->width && x_min <= x_max);
  ASSERT_(y_min >= 0 && y_min < m_state->height && y_min <= y_max);

  const int lx = (x_max - x_min + 1);
  const int ly = (y_max - y_min + 1);

  if (doResize || outMatrix.rows() < ly || outMatrix.cols() < lx)
  {
    outMatrix.setSize(ly, lx);
  }

  const bool is_color = isColor();
  const float scale = normalize_01 ? (1.0f / 255.0f) : 1.0f;

  for (int y = 0; y < ly; y++)
  {
    const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y, 0);
    for (int x = 0; x < lx; x++)
    {
      float value;
      if (is_color)
      {
        // Luminance
        value = static_cast<float>(pixels[0]) * 0.299f + static_cast<float>(pixels[1]) * 0.587f +
                static_cast<float>(pixels[2]) * 0.114f;
        pixels += m_state->channels;
      }
      else
      {
        value = *pixels++;
      }
      outMatrix(y, x) = value * scale;
    }
  }

  MRPT_END
}

void CImage::getAsMatrix(
    mrpt::math::CMatrix_u8& outMatrix, bool doResize, int x_min, int y_min, int x_max, int y_max)
    const
{
  MRPT_START
  makeSureImageIsLoaded();

  if (x_max == -1)
  {
    x_max = m_state->width - 1;
  }
  if (y_max == -1)
  {
    y_max = m_state->height - 1;
  }

  ASSERT_(x_min >= 0 && x_min < m_state->width && x_min <= x_max);
  ASSERT_(y_min >= 0 && y_min < m_state->height && y_min <= y_max);

  const int lx = (x_max - x_min + 1);
  const int ly = (y_max - y_min + 1);

  if (doResize || outMatrix.rows() < ly || outMatrix.cols() < lx)
  {
    outMatrix.setSize(ly, lx);
  }

  const bool is_color = isColor();

  for (int y = 0; y < ly; y++)
  {
    const uint8_t* pixels = ptr<uint8_t>(x_min, y_min + y, 0);
    for (int x = 0; x < lx; x++)
    {
      if (is_color)
      {
        // Luminance (integer arithmetic for speed)
        const unsigned int value = pixels[0] * 299U + pixels[1] * 587U + pixels[2] * 114U;
        outMatrix(y, x) = static_cast<uint8_t>(value / 1000);
        pixels += m_state->channels;
      }
      else
      {
        outMatrix(y, x) = *pixels++;
      }
    }
  }

  MRPT_END
}

void CImage::getAsRGBMatrices(
    [[maybe_unused]] mrpt::math::CMatrixFloat& R,
    [[maybe_unused]] mrpt::math::CMatrixFloat& G,
    [[maybe_unused]] mrpt::math::CMatrixFloat& B,
    [[maybe_unused]] bool doResize,
    [[maybe_unused]] int x_min,
    [[maybe_unused]] int y_min,
    [[maybe_unused]] int x_max,
    [[maybe_unused]] int y_max) const
{
  MRPT_START

  THROW_EXCEPTION("TODO!");
#if 0
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
#endif
  MRPT_END
}

void CImage::getAsRGBMatrices(
    [[maybe_unused]] mrpt::math::CMatrix_u8& R,
    [[maybe_unused]] mrpt::math::CMatrix_u8& G,
    [[maybe_unused]] mrpt::math::CMatrix_u8& B,
    [[maybe_unused]] bool doResize,
    [[maybe_unused]] int x_min,
    [[maybe_unused]] int y_min,
    [[maybe_unused]] int x_max,
    [[maybe_unused]] int y_max) const
{
  MRPT_START

  THROW_EXCEPTION("TODO!");
#if 0
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
#endif

  MRPT_END
}

void CImage::cross_correlation_FFT(
    const CImage& in_img,
    mrpt::math::CMatrixFloat& out_corr,
    std::optional<int32_t> u_search_ini,
    std::optional<int32_t> v_search_ini,
    std::optional<int32_t> u_search_size,
    std::optional<int32_t> v_search_size,
    std::optional<float> biasThisImg,
    std::optional<float> biasInImg) const
{
  MRPT_START

  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  // Set limits:
  const auto u_ini = u_search_ini ? *u_search_ini : 0;
  const auto v_ini = v_search_ini ? *v_search_ini : 0;
  const auto u_size = u_search_size ? *u_search_size : getWidth();
  const auto v_size = v_search_size ? *v_search_size : getHeight();

  const auto u_search_end = u_ini + u_size - 1;
  const auto v_search_end = v_ini + v_size - 1;

  ASSERT_(u_search_end < getWidth());
  ASSERT_(v_search_end < getHeight());

  // Find smallest valid size:
  const auto actual_lx = std::max(u_size, in_img.getWidth());
  const auto actual_ly = std::max(v_size, in_img.getHeight());
  const auto lx = static_cast<size_t>(mrpt::round2up(actual_lx));
  const auto ly = static_cast<size_t>(mrpt::round2up(actual_ly));

  mrpt::math::CMatrixF i1(ly, lx);
  mrpt::math::CMatrixF i2(ly, lx);

  // We fill the images with the bias, such as when we substract the bias
  // later on,
  //  those pixels not really occupied by the image really becomes zero:
  const auto biasImg = biasInImg ? *biasInImg : .0f;
  const auto biasThis = biasThisImg ? *biasThisImg : .0f;
  i1.fill(biasImg);
  i2.fill(biasThis);

  // Get as matrixes, padded with zeros up to power-of-two sizes:
  getAsMatrix(i2, false, u_ini, v_ini, u_ini + u_size - 1, v_ini + v_size - 1);
  in_img.getAsMatrix(i1, false);

  // Remove the bias now:
  i2 -= biasThis;
  i1 -= biasImg;

  // FFT:
  mrpt::math::CMatrixF I1_R, I1_I, I2_R, I2_I, ZEROS(ly, lx);
  mrpt::math::dft2_complex(i1, ZEROS, I1_R, I1_I);
  mrpt::math::dft2_complex(i2, ZEROS, I2_R, I2_I);

  // Compute the COMPLEX division of I2 by I1:
  for (int y = 0; y < static_cast<int>(ly); y++)
  {
    for (int x = 0; x < static_cast<int>(lx); x++)
    {
      float r1 = I1_R(y, x);
      float r2 = I2_R(y, x);

      float ii1 = I1_I(y, x);
      float ii2 = I2_I(y, x);

      float den = square(r1) + square(ii1);
      I2_R(y, x) = (r1 * r2 + ii1 * ii2) / den;
      I2_I(y, x) = (ii2 * r1 - r2 * ii1) / den;
    }
  }

  // IFFT:
  mrpt::math::CMatrixF res_R, res_I;
  math::idft2_complex(I2_R, I2_I, res_R, res_I);

  out_corr.setSize(actual_ly, actual_lx);
  for (int y = 0; y < actual_ly; y++)
  {
    for (int x = 0; x < actual_lx; x++)
    {
      out_corr(y, x) = std::sqrt(square(res_R(y, x)) + square(res_I(y, x)));
    }
  }

  MRPT_END
}

void CImage::getAsMatrixTiled([[maybe_unused]] mrpt::math::CMatrixFloat& outMatrix) const
{
  MRPT_START

  THROW_EXCEPTION("TODO!");
#if 0
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
#endif

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
    {
      std::cout << "[CImage::unload()] Called on this=" << reinterpret_cast<const void*>(this)
                << std::endl;
    }

    m_state->clear_image_data();
  }
}

void CImage::makeSureImageIsLoaded(bool allowNonInitialized) const
{
  if (!m_state->empty())
  {
    return;  // OK, continue
  }

  if (m_state->imgIsExternalStorage)
  {
    // Load the file:
    std::string wholeFile;
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
  THROW_EXCEPTION("TODO!");
#if 0
  cv::flip(m_impl->img, m_impl->img, 0 /* x-axis */);
#endif
}

void CImage::flipHorizontal()
{
  makeSureImageIsLoaded();
  THROW_EXCEPTION("TODO!");
#if 0
  cv::flip(m_impl->img, m_impl->img, 1 /* y-axis */);
#endif
}

void CImage::swapRB()
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  THROW_EXCEPTION("TODO!");
#if 0
  cv::cvtColor(m_impl->img, m_impl->img, cv::COLOR_RGB2BGR);
#endif
}

void CImage::undistort(
    [[maybe_unused]] CImage& out_img, [[maybe_unused]] const mrpt::img::TCamera& cameraParams) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  THROW_EXCEPTION("TODO!");
#if 0

  ASSERTMSG_(out_img.m_impl->img.data != m_impl->img.data, "In-place undistort() not supported");

  auto& srcImg = const_cast<cv::Mat&>(m_state->img);
  // This will avoid re-alloc if size already matches.
  out_img.resize(srcImg.cols, srcImg.rows, channels());

  const auto& intrMat = cameraParams.intrinsicParams;
  const auto& dist = cameraParams.dist;

  cv::Mat distM(1, dist.size(), CV_64F, const_cast<double*>(&dist[0]));
  cv::Mat inMat(3, 3, CV_64F);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) inMat.at<double>(i, j) = intrMat(i, j);

  cv::undistort(srcImg, out_img.m_impl->img, inMat, distM);
#endif
}

void CImage::filterMedian([[maybe_unused]] CImage& out_img, [[maybe_unused]] int W) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  THROW_EXCEPTION("TODO!");
#if 0
  auto srcImg = const_cast<cv::Mat&>(m_state->img);
  if (this == &out_img)
    srcImg = srcImg.clone();
  else
    out_img.resize(srcImg.cols, srcImg.rows, channels());

  cv::medianBlur(srcImg, out_img.m_impl->img, W);
#endif
}

void CImage::filterGaussian(
    [[maybe_unused]] CImage& out_img,
    [[maybe_unused]] int W,
    [[maybe_unused]] int H,
    [[maybe_unused]] double sigma) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally
  THROW_EXCEPTION("TODO!");
#if 0
  auto srcImg = const_cast<cv::Mat&>(m_state->img);
  if (this == &out_img)
    srcImg = srcImg.clone();
  else
    out_img.resize(srcImg.cols, srcImg.rows, channels());

  cv::GaussianBlur(srcImg, out_img.m_impl->img, cv::Size(W, H), sigma);
#endif
}

void CImage::rotateImage(
    [[maybe_unused]] CImage& out_img,
    [[maybe_unused]] double ang,
    [[maybe_unused]] const TPixelCoord& center,
    [[maybe_unused]] double scale) const
{
  makeSureImageIsLoaded();  // For delayed loaded images stored externally

  THROW_EXCEPTION("rotateImage() not yet implemented with STB library");
#if 0

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

  out_img.resize(getWidth(), getHeight(), channels());

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
#endif
}

bool CImage::drawChessboardCorners(
    [[maybe_unused]] const std::vector<TPixelCoordf>& cornerCoords,
    [[maybe_unused]] unsigned int check_size_x,
    [[maybe_unused]] unsigned int check_size_y,
    [[maybe_unused]] unsigned int lines_width,
    [[maybe_unused]] unsigned int r)
{
  if (cornerCoords.size() != static_cast<size_t>(check_size_x) * check_size_y)
  {
    return false;
  }

  THROW_EXCEPTION("TODO!");
#if 0

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
#endif
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
    if (&ret != this)
    {
      ret = *this;
    }
    return;
  }

  THROW_EXCEPTION("Not yet implemented");
#if 0
  auto srcImg = m_state->img;
  // Detect in-place op. and make deep copy:
  if (srcImg.data == ret.m_state->img.data) srcImg = srcImg.clone();

  ret.resize(getWidth(), getHeight(), CH_RGB);

  cv::cvtColor(srcImg, ret.m_impl->img, cv::COLOR_GRAY2BGR);
#endif
}

void CImage::joinImagesHorz(const CImage& img1, const CImage& img2)
{
  ASSERT_(img1.getHeight() == img2.getHeight());

  THROW_EXCEPTION("Not yet implemented");
#if 0
  auto im1 = img1.m_state->img, im2 = img2.m_state->img;
  ASSERT_(im1.type() == im2.type());

  this->resize(im1.cols + im2.cols, im1.rows, img1.channels());

  im1.copyTo(m_impl->img(cv::Rect(0, 0, im1.cols, im1.rows)));
  im2.copyTo(m_impl->img(cv::Rect(im1.cols, 0, im2.cols, im2.rows)));
#endif
}  // end

template <int HALF_WIN_SIZE>
void image_KLT_response_template(
    const mrpt::img::CImage& im, int x, int y, int32_t& _gxx, int32_t& _gyy, int32_t& _gxy)
{
  const auto min_x = x - HALF_WIN_SIZE;
  const auto min_y = y - HALF_WIN_SIZE;

  int32_t gxx = 0;
  int32_t gxy = 0;
  int32_t gyy = 0;

  const unsigned int WIN_SIZE = 1 + 2 * HALF_WIN_SIZE;

  int yy = min_y;
  for (int iy = WIN_SIZE; iy; --iy, ++yy)
  {
    int xx = min_x;
    for (int ix = WIN_SIZE; ix; --ix, ++xx)
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
  const auto& im1 = *this;
  const auto img_w = m_state->width;
  const auto img_h = m_state->height;

  // If any of those predefined values worked, do the generic way:
  const auto min_x = pt.x - half_window_size;
  const auto max_x = pt.x + half_window_size;
  const auto min_y = pt.y - half_window_size;
  const auto max_y = pt.y + half_window_size;

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

  const auto x = pt.x;
  const auto y = pt.y;

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
      for (int yy = min_y; yy <= max_y; yy++)
      {
        for (int xx = min_x; xx <= max_x; xx++)
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
  const float K = 0.5f / static_cast<float>((max_y - min_y + 1) * (max_x - min_x + 1));
  const float Gxx = static_cast<float>(gxx) * K;
  const float Gxy = static_cast<float>(gxy) * K;
  const float Gyy = static_cast<float>(gyy) * K;

  // Return the minimum eigenvalue of:
  //    ( gxx  gxy )
  //    ( gxy  gyy )
  // See, for example:
  // mrpt::math::detail::eigenVectorsMatrix_special_2x2():
  const float t = Gxx + Gyy;               // Trace
  const float de = Gxx * Gyy - Gxy * Gxy;  // Det
  const float discriminant = std::max<float>(.0f, t * t - 4.0f * de);
  // The smallest eigenvalue is:
  return 0.5f * (t - std::sqrt(discriminant));
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