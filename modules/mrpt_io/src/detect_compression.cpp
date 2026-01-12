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

#include <mrpt/io/detect_compression.h>

#include <array>
#include <cstdint>
#include <fstream>

namespace mrpt::io
{

CompressionType detect_compression(const std::string& filePath)
{
  std::ifstream file(filePath, std::ios::binary);
  if (!file.is_open())
  {
    return CompressionType::None;
  }

  std::array<std::uint8_t, 4> header{};
  file.read(reinterpret_cast<char*>(header.data()), header.size());  // NOLINT

  if (file.gcount() < 2)
  {
    return CompressionType::None;
  }

  // Gzip: 1F 8B
  if (header[0] == 0x1F && header[1] == 0x8B)
  {
    return CompressionType::Gzip;
  }

  if (file.gcount() < 4)
  {
    return CompressionType::None;
  }

  // Zstandard: 28 B5 2F FD
  if (header[0] == 0x28 && header[1] == 0xB5 && header[2] == 0x2F && header[3] == 0xFD)
  {
    return CompressionType::Zstd;
  }

  // Zstandard skippable frames: 2A 4D 18 5x
  if (header[0] == 0x2A && header[1] == 0x4D && header[2] == 0x18 &&
      (header[3] >= 0x50 && header[3] <= 0x5F))
  {
    return CompressionType::Zstd;
  }

  return CompressionType::None;
}

}  // namespace mrpt::io
