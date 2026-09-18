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

#include <mrpt/core/round.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CDisk.h>

#include <cmath>

using namespace mrpt;
using namespace mrpt::viz;
using namespace std;
using mrpt::poses::CPose3D;

IMPLEMENTS_SERIALIZABLE(CDisk, CVisualObject, mrpt::viz)

void CDisk::updateBuffers() const
{
  using mrpt::math::TPoint3Df;

  std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Triangles::m_trianglesMtx.data);
  auto& tris = VisualObjectParams_Triangles::m_triangles;

  tris.clear();

  ASSERT_GT_(m_nSlices, 2U);

  const float dAng = 2 * M_PIf / static_cast<float>(m_nSlices);
  float a = 0;
  // unit circle points: cos(ang),sin(ang)
  std::vector<mrpt::math::TPoint2Df> circle(m_nSlices);
  for (unsigned int i = 0; i < m_nSlices; i++, a += dAng)
  {
    circle[i].x = std::cos(a);
    circle[i].y = std::sin(a);
  }

  const float r0 = m_radiusIn;
  const float r1 = m_radiusOut;

  if (std::abs(r0) < 1e-6f)
  {
    // a filled disk:
    for (unsigned int i = 0; i < m_nSlices; i++)
    {
      const auto ip = (i + 1) % m_nSlices;
      tris.emplace_back(
          TPoint3Df(r1 * circle[i].x, r1 * circle[i].y, .0f),
          TPoint3Df(r1 * circle[ip].x, r1 * circle[ip].y, .0f), TPoint3Df(.0f, .0f, .0f));
    }
  }
  else
  {
    // a ring:
    for (unsigned int i = 0; i < m_nSlices; i++)
    {
      const auto ip = (i + 1) % m_nSlices;
      tris.emplace_back(
          TPoint3Df(r1 * circle[i].x, r1 * circle[i].y, .0f),
          TPoint3Df(r1 * circle[ip].x, r1 * circle[ip].y, .0f),
          TPoint3Df(r0 * circle[i].x, r0 * circle[i].y, .0f));

      tris.emplace_back(
          TPoint3Df(r1 * circle[ip].x, r1 * circle[ip].y, .0f),
          TPoint3Df(r0 * circle[ip].x, r0 * circle[ip].y, .0f),
          TPoint3Df(r0 * circle[i].x, r0 * circle[i].y, .0f));
    }
  }

  // All faces, same color:
  const auto col = getColor_u8();
  for (auto& t : tris)
  {
    t.setColor(col);
  }
}

uint8_t CDisk::serializeGetVersion() const { return 2; }
void CDisk::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_radiusIn << m_radiusOut;
  out << m_nSlices;
  VisualObjectParams_Triangles::params_serialize(out);  // v2
}

void CDisk::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    {
      readFromStreamRender(in);
      in >> m_radiusIn >> m_radiusOut;
      in >> m_nSlices;
      if (version < 1)
      {
        float dummy_loops;
        in >> dummy_loops;
      }

      if (version >= 2) VisualObjectParams_Triangles::params_deserialize(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

bool CDisk::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
  // The disk is contained initially in a plane which contains (0,0,0),
  // (1,0,0) and (0,1,0)
  // These points are converted into:
  //(x,y,z)
  //( cos(w)*cos(p)+x, sin(w)*cos(p)*y, -sin(p)+z )
  //( -sin(w)*cos(r)+cos(w)*sin(p)*sin(r)+x,
  // cos(w)*cos(r)+sin(w)*sin(p)*sin(r)+y, cos(p)*sin(r)*z )
  CPose3D transf = getCPose() - o;
  double x = transf.x(), y = transf.y(), z = transf.z(), w = transf.yaw(), p = transf.pitch(),
         r = transf.roll();
  double coef = sin(w) * sin(r) + cos(w) * sin(p) * cos(r);
  // coef is the first component of the normal to the transformed Z plane. So,
  // the scalar product between
  // this normal and (1,0,0) (which happens to be the beam's vector) equals
  // coef. And if it's 0, then both
  // are orthogonal, that is, the beam is parallel to the plane.
  if (coef == 0)
  {
    return false;
  }
  // The following expression yields the collision point between the plane and
  // the beam (the y and z
  // coordinates are zero).
  dist = x + (y * (sin(p) * sin(w) * cos(r) - cos(w) * sin(r)) + z * cos(p) * cos(r)) / coef;
  if (dist < 0)
  {
    return false;
  }
  // Euclidean distance is invariant to rotations...
  double d2 = (x - dist) * (x - dist) + y * y + z * z;
  return d2 >= (m_radiusIn * m_radiusIn) && d2 <= (m_radiusOut * m_radiusOut);

  // IMPORTANT NOTICE: using geometric intersection between Z plane and
  // CPose's line intersection is SLOWER than the used method.
}

mrpt::math::TBoundingBoxf CDisk::internalBoundingBoxLocal() const
{
  const float R = std::max(m_radiusIn, m_radiusOut);
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints({-R, -R, 0}, {R, R, .0});
}
