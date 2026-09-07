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

#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>

#include <cstdint>
#include <functional>
#include <string>

namespace mrpt_test
{
/** Writes an MRPT object frame (class name, streaming version, payload and end
 *  flag) with an arbitrary version number, so that the backwards-compatibility
 *  branches of serializeFrom() can be exercised without shipping binary
 *  fixture files. The stream is left rewound and ready to be read back.
 */
inline void writeLegacyObjectFrame(
    mrpt::io::CMemoryStream& buf,
    const std::string& className,
    uint8_t version,
    const std::function<void(mrpt::serialization::CArchive&)>& writePayload)
{
  auto arch = mrpt::serialization::archiveFrom(buf);

  // Class name, length with the "new format" flag in its MSB:
  const auto lenAndFlag = static_cast<int8_t>(className.size() | 0x80);
  arch << lenAndFlag;
  buf.Write(className.data(), className.size());

  arch << version;
  writePayload(arch);

  const uint8_t endFlag = 0x88;
  arch << endFlag;

  buf.Seek(0);
}
}  // namespace mrpt_test
