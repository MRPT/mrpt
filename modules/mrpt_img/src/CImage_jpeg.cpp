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

#include "CImage_jpeg.h"

#include <mrpt/core/get_env.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/config.h>

#include <atomic>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>

#include "CImage_impl.h"

STB_DISABLE_WARNINGS
#include "stb/stb_image.h"
STB_RESTORE_WARNINGS

#if MRPT_HAS_LIBJPEG_TURBO
#include <turbojpeg.h>
#endif

namespace
{
// Initial value from the environment, so it can be switched without
// rebuilding (e.g. to compare both decoders in a running application).
std::atomic<bool> g_use_libjpeg_turbo{mrpt::get_env<bool>("MRPT_IMG_LIBJPEG_TURBO", true)};

#if MRPT_HAS_LIBJPEG_TURBO
/** One decompressor per thread: creating it allocates, and TurboJPEG handles
 * must not be shared between threads. */
class TurboDecompressor
{
 public:
  TurboDecompressor() : m_handle(tjInitDecompress()) {}
  ~TurboDecompressor()
  {
    if (m_handle != nullptr)
    {
      tjDestroy(m_handle);
    }
  }
  TurboDecompressor(const TurboDecompressor&) = delete;
  TurboDecompressor& operator=(const TurboDecompressor&) = delete;
  TurboDecompressor(TurboDecompressor&&) = delete;
  TurboDecompressor& operator=(TurboDecompressor&&) = delete;

  [[nodiscard]] tjhandle get() const { return m_handle; }

 private:
  tjhandle m_handle;
};

/** Decodes with libjpeg-turbo. Returns nullptr, leaving the job to stb_image,
 * for anything outside the cases handled here, or on any error or warning. */
unsigned char* turbo_decode(
    const unsigned char* data,
    int length,
    int* width,
    int* height,
    int* channels_in_file,
    int desired_channels)
{
  thread_local TurboDecompressor decompressor;
  tjhandle tj = decompressor.get();
  if (tj == nullptr)
  {
    return nullptr;
  }

  int w = 0;
  int h = 0;
  int subsamp = 0;
  int colorspace = 0;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast): TurboJPEG 2.x API takes non-const
  auto* src = const_cast<unsigned char*>(data);
  if (tjDecompressHeader3(
          tj, src, static_cast<unsigned long>(length), &w, &h, &subsamp, &colorspace) != 0 ||
      w <= 0 || h <= 0)
  {
    return nullptr;
  }

  // Same channel counts stb_image reports. CMYK/YCCK JPEGs are left to it.
  int file_channels = 0;
  if (colorspace == TJCS_GRAY)
  {
    file_channels = 1;
  }
  else if (colorspace == TJCS_YCbCr || colorspace == TJCS_RGB)
  {
    file_channels = 3;
  }
  else
  {
    return nullptr;
  }

  const int out_channels = desired_channels != 0 ? desired_channels : file_channels;
  int pixel_format = 0;
  switch (out_channels)
  {
    case 1:
      pixel_format = TJPF_GRAY;
      break;
    case 3:
      pixel_format = TJPF_RGB;
      break;
    case 4:
      pixel_format = TJPF_RGBA;  // alpha = 255, as stb_image
      break;
    default:
      return nullptr;  // 2 (gray+alpha): not used by CImage
  }

  const auto pitch = static_cast<uint64_t>(w) * static_cast<uint64_t>(out_channels);
  const auto totalBytes = pitch * static_cast<uint64_t>(h);
  // Keep the stb_image size limit, and avoid size_t overflow on 32-bit targets.
  if (totalBytes > static_cast<uint64_t>(std::numeric_limits<int>::max()))
  {
    return nullptr;
  }
  // malloc(): the buffer is released with stbi_image_free(), i.e. free().
  auto* pixels = static_cast<unsigned char*>(std::malloc(static_cast<std::size_t>(totalBytes)));
  if (pixels == nullptr)
  {
    return nullptr;
  }

  // TJFLAG_STOPONWARNING: corrupt-but-recoverable data fails here and goes
  // to stb_image, which keeps the existing accept/reject behavior.
  if (tjDecompress2(
          tj, src, static_cast<unsigned long>(length), pixels, w, static_cast<int>(pitch), h,
          pixel_format, TJFLAG_STOPONWARNING) != 0)
  {
    std::free(pixels);
    return nullptr;
  }

  *width = w;
  *height = h;
  *channels_in_file = file_channels;
  return pixels;
}
#endif

}  // namespace

namespace mrpt::img
{
bool CImage::hasLibJpegTurbo() noexcept { return MRPT_HAS_LIBJPEG_TURBO != 0; }

void CImage::setUseLibJpegTurbo(bool enable) noexcept { g_use_libjpeg_turbo = enable; }

bool CImage::getUseLibJpegTurbo() noexcept { return internal::libJpegTurboActive(); }

namespace internal
{
bool libJpegTurboActive() noexcept
{
#if MRPT_HAS_LIBJPEG_TURBO
  return g_use_libjpeg_turbo.load();
#else
  return false;
#endif
}

bool looksLikeJPEG(const uint8_t* data, std::size_t length) noexcept
{
  return data != nullptr && length >= 3 && data[0] == 0xFF && data[1] == 0xD8 && data[2] == 0xFF;
}

std::optional<std::vector<uint8_t>> readFileIfJPEG(const std::string& fileName)
{
  std::ifstream f(fileName, std::ios::binary);
  if (!f)
  {
    return std::nullopt;
  }
  std::vector<uint8_t> head(3);
  if (!f.read(reinterpret_cast<char*>(head.data()), 3) || !looksLikeJPEG(head.data(), head.size()))
  {
    return std::nullopt;
  }
  f.seekg(0, std::ios::end);
  const auto size = f.tellg();
  if (size <= 0 || size > static_cast<std::streamoff>(std::numeric_limits<int>::max()))
  {
    return std::nullopt;
  }
  std::vector<uint8_t> buf(static_cast<std::size_t>(size));
  f.seekg(0, std::ios::beg);
  if (!f.read(reinterpret_cast<char*>(buf.data()), size))
  {
    return std::nullopt;
  }
  return buf;
}

unsigned char* load_8bit_from_memory(
    const unsigned char* data,
    int length,
    int* width,
    int* height,
    int* channels_in_file,
    int desired_channels)
{
#if MRPT_HAS_LIBJPEG_TURBO
  if (libJpegTurboActive() && length > 0 && looksLikeJPEG(data, static_cast<std::size_t>(length)))
  {
    if (auto* pixels =
            turbo_decode(data, length, width, height, channels_in_file, desired_channels);
        pixels != nullptr)
    {
      return pixels;
    }
  }
#endif
  return stbi_load_from_memory(data, length, width, height, channels_in_file, desired_channels);
}

}  // namespace internal
}  // namespace mrpt::img
