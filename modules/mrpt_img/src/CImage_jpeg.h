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

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

/** Internal helpers for the optional libjpeg-turbo JPEG decoding path of CImage. */
namespace mrpt::img::internal
{
/** True if MRPT was built with libjpeg-turbo and it is enabled at runtime
 * (see CImage::setUseLibJpegTurbo()). */
bool libJpegTurboActive() noexcept;

/** True if the buffer starts with the JPEG SOI marker plus the next marker's 0xFF. */
bool looksLikeJPEG(const uint8_t* data, std::size_t length) noexcept;

/** Reads the whole file if it starts like a JPEG. Returns std::nullopt if it
 * does not, or if it cannot be read, leaving those cases to stb_image. */
std::optional<std::vector<uint8_t>> readFileIfJPEG(const std::string& fileName);

/** Drop-in replacement for stbi_load_from_memory(): same arguments, same
 * return value and ownership (free the result with stbi_image_free()).
 *
 * JPEG data is decoded with libjpeg-turbo when libJpegTurboActive(). Any other
 * format, and any JPEG libjpeg-turbo rejects or warns about (corrupt data,
 * CMYK, ...) goes to stb_image, so malformed input gives the same result as
 * before and stbi_failure_reason() stays meaningful after a failure. */
unsigned char* load_8bit_from_memory(
    const unsigned char* data,
    int length,
    int* width,
    int* height,
    int* channels_in_file,
    int desired_channels);

}  // namespace mrpt::img::internal
