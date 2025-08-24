/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/exceptions.h>
#include <mrpt/system/crc.h>

uint16_t mrpt::system::compute_CRC16(const std::vector<uint8_t>& data, const uint16_t gen_pol)
{
  ASSERT_(!data.empty());
  return compute_CRC16(&data[0], data.size(), gen_pol);
}

uint32_t mrpt::system::compute_CRC32(const std::vector<uint8_t>& data, const uint32_t gen_pol)
{
  ASSERT_(!data.empty());
  return compute_CRC32(&data[0], data.size(), gen_pol);
}

uint16_t mrpt::system::compute_CRC16(const uint8_t* data, size_t len_, const uint16_t gen_pol)
{
  uint16_t uCrc16;
  uint8_t abData[2];

  size_t len = len_;

  uCrc16 = 0;
  abData[0] = 0;

  while (len--)
  {
    abData[1] = abData[0];
    abData[0] = *data++;

    if (uCrc16 & 0x8000)
    {
      uCrc16 = (uCrc16 & 0x7fff) << 1;
      uCrc16 ^= gen_pol;
    }
    else
    {
      uCrc16 <<= 1;
    }
    uCrc16 ^= (abData[0] | (abData[1] << 8));
  }
  return uCrc16;
}

namespace
{
uint32_t CRC32Value(int i, const uint32_t CRC32_POLYNOMIAL)
{
  uint32_t ulCRC = static_cast<uint32_t>(i);
  for (int j = 8; j > 0; j--)
  {
    if (ulCRC & 1)
    {
      ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
    }
    else
    {
      ulCRC >>= 1;
    }
  }
  return ulCRC;
}
}  // namespace

uint32_t mrpt::system::compute_CRC32(const uint8_t* data, size_t len_, const uint32_t gen_pol)
{
  size_t len = len_;
  uint32_t ulCRC = 0;
  while (len-- != 0)
  {
    const uint32_t ulTemp1 = (static_cast<uint32_t>(ulCRC) >> 8) & 0x00FFFFFFUL;
    const uint32_t ulTemp2 = CRC32Value((static_cast<int>(ulCRC) ^ *data++) & 0xff, gen_pol);
    ulCRC = ulTemp1 ^ ulTemp2;
  }
  return ulCRC;
}
