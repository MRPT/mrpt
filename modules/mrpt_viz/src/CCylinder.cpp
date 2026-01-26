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

#include <cmath>

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

      if (version >= 2)
      {
        VisualObjectParams_Triangles::params_deserialize(in);
      }
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

namespace
{
bool solveEqn(double a, double b, double c, double& t)
{
  // Actually, the b from the quadratic equation is the DOUBLE of this.
  // But this way, operations are simpler.
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
    {
      return (t = -b / a) >= 0;
    }
    if (delta >= 0)
    {
      delta = sqrt(delta);
      if (-b - delta > 0)
      {
        t = (-b - delta) / a;
        return true;
      }
      if (-b + delta > 0)
      {
        t = (-b + delta) / a;
        return true;
      }
    }
  }
  else if (std::abs(b) >= mrpt::math::getEpsilon())
  {
    t = -c / (b + b);
    return t >= 0;
  }
  return false;
}
}  // namespace

bool CCylinder::traceRay(const mrpt::poses::CPose3D& o, double& dist) const
{
  TLine3D lin;
  mrpt::math::createFromPoseX((o - getCPose()).asTPose(), lin);
  lin.unitarize();

  const float zz = d2f(lin.pBase.z);

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
    {
      if ((!fnd || nDist < dist) && reachesHeight(lin.pBase.z + nDist * lin.director[2]))
      {
        dist = nDist;
        fnd = true;
      }
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
    {
      if ((!fnd || nDist < dist) && reachesHeight(lin.pBase.z + nDist * lin.director[2]))
      {
        dist = nDist;
        fnd = true;
      }
    }
  }

  return fnd;
}

auto CCylinder::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  const float R = std::max(m_baseRadius, m_topRadius);
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints({-R, -R, 0}, {R, R, m_height});
}

void CCylinder::updateBuffers() const
{
  std::unique_lock<std::shared_mutex> lck(
      VisualObjectParams_Triangles::shaderTrianglesBufferMutex().data);

  auto& tris =
      const_cast<std::vector<TTriangle>&>(VisualObjectParams_Triangles::shaderTrianglesBuffer());

  tris.clear();

  const auto color = getColor_u8();
  const auto twoPi = static_cast<float>(2.0 * M_PI);

  // Precompute sine/cosine table for efficiency
  std::vector<float> sinTable(m_slices + 1);
  std::vector<float> cosTable(m_slices + 1);
  for (uint32_t i = 0; i <= m_slices; i++)
  {
    float angle = twoPi * static_cast<float>(i) / static_cast<float>(m_slices);
    sinTable[i] = std::sin(angle);
    cosTable[i] = std::cos(angle);
  }

  // Helper to add a triangle with color
  auto addTri = [&](const TPoint3Df& p0, const TPoint3Df& p1, const TPoint3Df& p2,
                    const TVector3Df& n0, const TVector3Df& n1, const TVector3Df& n2)
  {
    TTriangle t(p0, p1, p2, n0, n1, n2);
    t.setColor(color);
    tris.push_back(t);
  };

  // Generate lateral surface
  for (uint32_t i = 0; i < m_slices; i++)
  {
    const float c0 = cosTable[i];
    const float s0 = sinTable[i];
    const float c1 = cosTable[i + 1];
    const float s1 = sinTable[i + 1];

    // Bottom ring vertices
    const TPoint3Df pb0(m_baseRadius * c0, m_baseRadius * s0, 0.0f);
    const TPoint3Df pb1(m_baseRadius * c1, m_baseRadius * s1, 0.0f);

    // Top ring vertices
    const TPoint3Df pt0(m_topRadius * c0, m_topRadius * s0, m_height);
    const TPoint3Df pt1(m_topRadius * c1, m_topRadius * s1, m_height);

    // Normals for the lateral surface
    // For a cone/cylinder, normal is perpendicular to the surface
    float nz = (m_baseRadius - m_topRadius) / m_height;
    float nLen = std::sqrt(1.0f + nz * nz);
    TVector3Df n0(c0 / nLen, s0 / nLen, nz / nLen);
    TVector3Df n1(c1 / nLen, s1 / nLen, nz / nLen);

    // Two triangles per quad
    addTri(pb0, pt0, pb1, n0, n0, n1);
    addTri(pb1, pt0, pt1, n1, n0, n1);
  }

  // Generate bottom base (if enabled)
  if (m_hasBottomBase && m_baseRadius > 0.0f)
  {
    const TVector3Df normalDown(0, 0, -1);
    const TPoint3Df center(0, 0, 0);

    for (uint32_t i = 0; i < m_slices; i++)
    {
      const TPoint3Df p0(m_baseRadius * cosTable[i], m_baseRadius * sinTable[i], 0.0f);
      const TPoint3Df p1(m_baseRadius * cosTable[i + 1], m_baseRadius * sinTable[i + 1], 0.0f);

      // Wind counter-clockwise when viewed from below (normal pointing down)
      addTri(center, p1, p0, normalDown, normalDown, normalDown);
    }
  }

  // Generate top base (if enabled)
  if (m_hasTopBase && m_topRadius > 0.0f)
  {
    const TVector3Df normalUp(0, 0, 1);
    const TPoint3Df center(0, 0, m_height);

    for (uint32_t i = 0; i < m_slices; i++)
    {
      const TPoint3Df p0(m_topRadius * cosTable[i], m_topRadius * sinTable[i], m_height);
      const TPoint3Df p1(m_topRadius * cosTable[i + 1], m_topRadius * sinTable[i + 1], m_height);

      // Wind counter-clockwise when viewed from above (normal pointing up)
      addTri(center, p0, p1, normalUp, normalUp, normalUp);
    }
  }

  clearChangedFlag();
}