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

namespace mrpt::io
{
/** Compression algorithm types supported by compressed streams.
 * \ingroup mrpt_io_grp
 */
enum class CompressionType : uint8_t
{
  /** No compression */
  None = 0,
  /** Gzip compression (RFC 1952) */
  Gzip = 1,
  /** Zstandard compression */
  Zstd = 2
};

/** Compression options for output streams.
 * \ingroup mrpt_io_grp
 */
struct CompressionOptions
{
  /** Compression algorithm to use */
  CompressionType type = CompressionType::Gzip;

  /** Compression level.
   * - For Gzip: 0 = no compression, 1 = fastest, 9 = best compression
   * - For Zstd: 1 = fastest, 3 = default, 22 = maximum (ultra mode)
   * - For None: ignored
   */
  int level = 1;

  CompressionOptions() = default;
  CompressionOptions(CompressionType t, int l = 1) : type(t), level(l) {}
};

}  // namespace mrpt::io
