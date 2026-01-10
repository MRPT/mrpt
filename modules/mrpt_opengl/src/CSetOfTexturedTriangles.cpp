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

#include <mrpt/opengl/CSetOfTexturedTriangles.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CSetOfTexturedTriangles, CRenderizable, mrpt::opengl)

void CSetOfTexturedTriangles::onUpdateBuffers_TexturedTriangles()
{
  // Nothing else to do: all data is already in m_triangles in my base class.
}

uint8_t CSetOfTexturedTriangles::serializeGetVersion() const { return 2; }
void CSetOfTexturedTriangles::serializeTo(mrpt::serialization::CArchive& out) const
{
  std::shared_lock<std::shared_mutex> readLock(m_trianglesMtx.data);

  uint32_t n;

  writeToStreamRender(out);
  writeToStreamTexturedObject(out);

  n = (uint32_t)m_triangles.size();

  out << n;

  for (uint32_t i = 0; i < n; i++) m_triangles[i].writeTo(out);
}

void CSetOfTexturedTriangles::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  std::unique_lock<std::shared_mutex> writeLock(m_trianglesMtx.data);

  switch (version)
  {
    case 0:
    case 1:
    case 2:
    {
      readFromStreamRender(in);
      if (version >= 2)
      {
        readFromStreamTexturedObject(in);
      }
      else
      {  // Old version.
        THROW_EXCEPTION("deserializing old version not supported.");
      }

      uint32_t n;
      in >> n;
      m_triangles.resize(n);

      for (uint32_t i = 0; i < n; i++) m_triangles[i].readFrom(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CRenderizable::notifyChange();
}

bool CSetOfTexturedTriangles::traceRay(
    [[maybe_unused]] const mrpt::poses::CPose3D& o, [[maybe_unused]] double& dist) const
{
  throw std::runtime_error("TODO: TraceRay not implemented in CSetOfTexturedTriangles");
}

auto CSetOfTexturedTriangles::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return trianglesBoundingBox();
}
