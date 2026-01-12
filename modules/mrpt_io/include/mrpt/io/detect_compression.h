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
#include <string>

namespace mrpt::io
{
/** \brief Enumeration of supported compression formats.
 *
 * This enum represents the compression type detected from a file's
 * leading magic bytes.
 */
enum class CompressionType : uint8_t
{
  /** Not compressed, or unsupported format */
  None = 0,

  /** Gzip-compressed stream (RFC 1952) */
  Gzip,

  /** Zstandard-compressed stream (including skippable frames) */
  Zstd
};

/** \brief Detects whether a file is gzip- or Zstandard-compressed, or none.
 *
 * This function inspects the magic bytes at the beginning of the file
 * to identify well-known compression formats:
 *
 * - **Gzip**: magic bytes `0x1F 0x8B`
 * - **Zstandard**: magic bytes `0x28 0xB5 0x2F 0xFD`
 * - **Zstandard skippable frames**: `0x2A 0x4D 0x18 0x5x`
 *
 * The detection is constant-time and does not attempt full decompression.
 *
 * \param filePath Path to the file to inspect.
 * \return The detected compression type, or CompressionType::None
 *         if the format is not recognized.
 */
CompressionType detect_compression(const std::string& filePath);

}  // namespace mrpt::io
