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

#include "viz-precomp.h"  // Precompiled header
//
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CDisk.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace std;
using mrpt::poses::CPose3D;

IMPLEMENTS_SERIALIZABLE(CDisk, CVisualObject, mrpt::viz)

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
  if (coef == 0) return false;
  // The following expression yields the collision point between the plane and
  // the beam (the y and z
  // coordinates are zero).
  dist = x + (y * (sin(p) * sin(w) * cos(r) - cos(w) * sin(r)) + z * cos(p) * cos(r)) / coef;
  if (dist < 0) return false;
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
