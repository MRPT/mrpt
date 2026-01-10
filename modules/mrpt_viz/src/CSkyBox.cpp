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

#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/viz/CSkyBox.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CSkyBox, CVisualObject, mrpt::viz)

uint8_t CSkyBox::serializeGetVersion() const { return 0; }
void CSkyBox::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_textureImages;  // <mrpt/serialization/stl_serialization.h>
}

void CSkyBox::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      readFromStreamRender(in);
      in >> m_textureImages;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

/** In this class, returns a fixed box (max,max,max), (-max,-max,-max). */
auto CSkyBox::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf { return {}; }

void CSkyBox::assignImage(const CUBE_TEXTURE_FACE face, const mrpt::img::CImage& img)
{
  const int faceIdx = static_cast<int>(face);
  ASSERT_GE_(faceIdx, 0);
  ASSERT_LT_(faceIdx, 6);

  m_textureImages[faceIdx] = img;
  CVisualObject::notifyChange();
}

void CSkyBox::assignImage(const CUBE_TEXTURE_FACE face, mrpt::img::CImage&& img)
{
  const int faceIdx = static_cast<int>(face);
  ASSERT_GE_(faceIdx, 0);
  ASSERT_LT_(faceIdx, 6);

  m_textureImages[faceIdx] = std::move(img);
  CVisualObject::notifyChange();
}

CSkyBox::~CSkyBox() = default;
