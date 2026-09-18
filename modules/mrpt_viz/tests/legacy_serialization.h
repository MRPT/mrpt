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

#include <mrpt/img/TColor.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/TPoint3D.h>
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

/** Emits the header that every mrpt::viz class writes first via
 *  CVisualObject::writeToStreamRender(), at an arbitrary version of *that*
 *  header (0..4), which is versioned independently of the class itself.
 *
 *  Scales are always written as "all unity", the common case, so the caller
 *  need not care about the scale bits of the magic signature.
 */
inline void writeLegacyRenderHeader(
    mrpt::serialization::CArchive& arch,
    uint8_t renderVersion = 4,
    const std::string& name = std::string(),
    const mrpt::img::TColor& color = mrpt::img::TColor(0xff, 0xff, 0xff, 0xff))
{
  arch << static_cast<uint8_t>(0xFF);
  // bit7: new header format; bit6: all scales are 1.0
  arch << static_cast<uint8_t>(renderVersion | 0xC0);

  arch << static_cast<uint16_t>(name.size());
  if (!name.empty())
  {
    arch.WriteBuffer(name.data(), name.size());
  }

  arch << color.R << color.G << color.B << color.A;

  // pose: x,y,z,yaw,pitch,roll as floats
  for (int i = 0; i < 6; i++)
  {
    arch << 0.0f;
  }

  arch << true;  // show_name
  arch << true;  // visible

  if (renderVersion >= 1)
  {
    arch << mrpt::math::TPoint3Df(0, 0, 0);  // representativePoint
  }
  if (renderVersion >= 2)
  {
    arch << 0.2f;  // materialShininess
    arch << true;  // castShadows
  }
  if (renderVersion >= 3)
  {
    arch << 32.0f;  // materialSpecularExponent
  }
  if (renderVersion >= 4)
  {
    arch << 0.0f << 0.0f << 0.0f;  // materialEmissive R,G,B
  }
}
}  // namespace mrpt_test
