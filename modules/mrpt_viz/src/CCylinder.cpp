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

#include <mrpt/math/TLine3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/viz/CCylinder.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CCylinder, CVisualObject, mrpt::viz)

void CCylinder::serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const
{
  SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
  out["baseRadius"] = m_baseRadius;
  out["topRadius"] = m_topRadius;
  out["height"] = m_height;
  out["slices"] = m_slices;
  out["hasBottomBase"] = m_hasBottomBase;
  out["hasTopBase"] = m_hasTopBase;
}
void CCylinder::serializeFrom(mrpt::serialization::CSchemeArchiveBase& in)
{
  uint8_t version;
  SCHEMA_DESERIALIZE_DATATYPE_VERSION();
  switch (version)
  {
    case 1:
    {
      m_baseRadius = static_cast<float>(in["baseRadius"]);
      m_topRadius = static_cast<float>(in["topRadius"]);
      m_height = static_cast<float>(in["height"]);
      m_slices = static_cast<uint32_t>(in["slices"]);
      m_hasBottomBase = static_cast<bool>(in["hasBottomBase"]);
      m_hasTopBase = static_cast<bool>(in["hasTopBase"]);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}
uint8_t CCylinder::serializeGetVersion() const { return 2; }
void CCylinder::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  // version 0
  out << m_baseRadius << m_topRadius << m_height << m_slices << m_hasBottomBase << m_hasTopBase;
  VisualObjectParams_Triangles::params_serialize(out);  // v2
}
void CCylinder::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
      readFromStreamRender(in);
      in >> m_baseRadius >> m_topRadius >> m_height >> m_slices;

      if (version < 1)
      {
        float old_mStacks;
        in >> old_mStacks;
      }

      in >> m_hasBottomBase >> m_hasTopBase;

      if (version >= 2) VisualObjectParams_Triangles::params_deserialize(in);
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

namespace
{
bool solveEqn(double a, double b, double c, double& t)
{  // Actually, the b from the quadratic equation is
   // the DOUBLE of this. But
  // this way, operations are simpler.
  if (a < 0)
  {
    a = -a;
    b = -b;
    c = -c;
  }
  if (a >= mrpt::math::getEpsilon())
  {
    double delta = square(b) - a * c;
    if (delta == 0)
      return (t = -b / a) >= 0;
    else if (delta >= 0)
    {
      delta = sqrt(delta);
      if (-b - delta > 0)
      {
        t = (-b - delta) / a;
        return true;
      }
      else if (-b + delta > 0)
      {
        t = (-b + delta) / a;
        return true;
      }  // else return false;	Both solutions are negative
    }    // else return false;	Both solutions are complex
  }
  else if (std::abs(b) >= mrpt::math::getEpsilon())
  {
    t = -c / (b + b);
    return t >= 0;
  }  // else return false;	This actually isn't an equation
  return false;
}
}  // namespace

bool CCylinder::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
  TLine3D lin;
  mrpt::math::createFromPoseX((o - getCPose()).asTPose(), lin);
  lin.unitarize();  // By adding this line, distance from any point of the

  const float zz = d2f(lin.pBase.z);

  // line to its base is exactly equal to the "t".
  if (std::abs(lin.director[2]) < getEpsilon())
  {
    if (!reachesHeight(zz))
    {
      return false;
    }
    float r;
    return getRadius(zz, r) ? solveEqn(
                                  square(lin.director[0]) + square(lin.director[1]),
                                  lin.director[0] * lin.pBase.x + lin.director[1] * lin.pBase.y,
                                  square(lin.pBase.x) + square(lin.pBase.y) - square(r), dist)
                            : false;
  }
  bool fnd = false;
  double nDist, tZ0;
  if (m_hasBottomBase && (tZ0 = -lin.pBase.z / lin.director[2]) > 0)
  {
    nDist = sqrt(
        square(lin.pBase.x + tZ0 * lin.director[0]) + square(lin.pBase.y + tZ0 * lin.director[1]));
    if (nDist <= m_baseRadius)
    {
      fnd = true;
      dist = tZ0;
    }
  }
  if (m_hasTopBase)
  {
    tZ0 = (m_height - lin.pBase.z) / lin.director[2];
    if (tZ0 > 0 && (!fnd || tZ0 < dist))
    {
      nDist = sqrt(
          square(lin.pBase.x + tZ0 * lin.director[0]) +
          square(lin.pBase.y + tZ0 * lin.director[1]));
      if (nDist <= m_topRadius)
      {
        fnd = true;
        dist = tZ0;
      }
    }
  }
  if (m_baseRadius == m_topRadius)
  {
    if (solveEqn(
            square(lin.director[0]) + square(lin.director[1]),
            lin.director[0] * lin.pBase.x + lin.director[1] * lin.pBase.y,
            square(lin.pBase.x) + square(lin.pBase.y) - square(m_baseRadius), nDist))
      if ((!fnd || nDist < dist) && reachesHeight(lin.pBase.z + nDist * lin.director[2]))
      {
        dist = nDist;
        fnd = true;
      }
  }
  else
  {
    double slope = (m_topRadius - m_baseRadius) / m_height;
    if (solveEqn(
            square(lin.director[0]) + square(lin.director[1]) - square(lin.director[2] * slope),
            lin.pBase.x * lin.director[0] + lin.pBase.y * lin.director[1] -
                (m_baseRadius + slope * lin.pBase.z) * slope * lin.director[2],
            square(lin.pBase.x) + square(lin.pBase.y) - square(m_baseRadius + slope * lin.pBase.z),
            nDist))
      if ((!fnd || nDist < dist) && reachesHeight(lin.pBase.z + nDist * lin.director[2]))
      {
        dist = nDist;
        fnd = true;
      }
  }
  return fnd;
}

auto CCylinder::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  const float R = std::max(m_baseRadius, m_topRadius);
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints({-R, -R, 0}, {R, R, m_height});
}
