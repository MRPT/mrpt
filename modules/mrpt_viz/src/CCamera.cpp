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

#include <mrpt/containers/yaml.h>
#include <mrpt/math/geometry.h>  // crossProduct3D()
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/optional_serialization.h>
#include <mrpt/viz/CCamera.h>

#include <algorithm>
#include <cmath>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CCamera, CVisualObject, mrpt::viz)

uint8_t CCamera::serializeGetVersion() const { return 5; }
void CCamera::serializeTo(mrpt::serialization::CArchive& out) const
{
  // v5: the base state, which carries the SE(3) pose used in 6DOF mode:
  writeToStreamRender(out);

  // Save data:
  out << m_pointingX << m_pointingY << m_pointingZ << m_eyeDistance << m_azimuthDeg
      << m_elevationDeg << m_projectiveModel << m_projectiveFOVdeg;
  out << m_pinholeModel;     // v2
  out << m_useNoProjection;  // v3
  out << m_eyeRollDeg;       // v4
  out << m_6DOFMode;         // v5
}

void CCamera::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    {
      if (version >= 5) readFromStreamRender(in);

      // Load data:
      in >> m_pointingX >> m_pointingY >> m_pointingZ >> m_eyeDistance >> m_azimuthDeg >>
          m_elevationDeg >> m_projectiveModel >> m_projectiveFOVdeg;
      if (version >= 2)
        in >> m_pinholeModel;
      else
        m_pinholeModel.reset();

      if (version >= 3)
        in >> m_useNoProjection;
      else
        m_useNoProjection = false;

      if (version >= 4)
        in >> m_eyeRollDeg;
      else
        m_eyeRollDeg = 0;

      if (version >= 5)
        in >> m_6DOFMode;
      else
        m_6DOFMode = false;
    }
    break;
    case 0:
    {
      in >> m_pointingX >> m_pointingY >> m_pointingZ >> m_eyeDistance >> m_azimuthDeg >>
          m_elevationDeg;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

auto CCamera::ComputeOrbitEyeAndUp(
    const mrpt::math::TPoint3D& pointingAt,
    double azimuthRad,
    double elevationRad,
    double distance,
    double rollRad) -> OrbitEyeAndUp
{
  const double ca = std::cos(azimuthRad);
  const double sa = std::sin(azimuthRad);
  const double ce = std::cos(elevationRad);
  const double se = std::sin(elevationRad);

  // Unit vector from the pointing point towards the eye:
  const TVector3D toEye(ca * ce, sa * ce, se);

  OrbitEyeAndUp ret;
  ret.eye = pointingAt + toEye * std::max(0.001, distance);

  // Up vector without roll: orthogonal to the viewing direction, in the
  // vertical plane containing it.
  const TVector3D up0(-ca * se, -sa * se, ce);
  if (rollRad == 0)
  {
    ret.up = up0;
    return ret;
  }

  // Roll: rotate the up vector around the viewing direction (forward=-toEye).
  // Since up0 is orthogonal to forward, Rodrigues' formula reduces to:
  const TVector3D forward = toEye * -1.0;
  const TVector3D right = mrpt::math::crossProduct3D(forward, up0);
  ret.up = up0 * std::cos(rollRad) + right * std::sin(rollRad);
  return ret;
}

mrpt::img::TCamera CCamera::PinholeForViewport(
    const mrpt::img::TCamera& pinhole, uint32_t viewportWidth, uint32_t viewportHeight)
{
  if (pinhole.ncols == viewportWidth && pinhole.nrows == viewportHeight)
  {
    return pinhole;
  }
  ASSERT_GT_(pinhole.nrows, 0U);

  const double s = static_cast<double>(viewportHeight) / static_cast<double>(pinhole.nrows);
  const double offsetX = 0.5 * (static_cast<double>(viewportWidth) - s * pinhole.ncols);

  mrpt::img::TCamera ret = pinhole;
  ret.ncols = viewportWidth;
  ret.nrows = viewportHeight;
  ret.fx(pinhole.fx() * s);
  ret.fy(pinhole.fy() * s);
  ret.cx(pinhole.cx() * s + offsetX);
  ret.cy(pinhole.cy() * s);
  return ret;
}

/** In this class, returns a fixed box (max,max,max), (-max,-max,-max). */
auto CCamera::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf { return {}; }

void CCamera::toYAMLMap(mrpt::containers::yaml& p) const
{
  CVisualObject::toYAMLMap(p);

  MCP_SAVE(p, m_pointingX);
  MCP_SAVE(p, m_pointingY);
  MCP_SAVE(p, m_pointingZ);
  MCP_SAVE(p, m_eyeDistance);
  MCP_SAVE(p, m_azimuthDeg);
  MCP_SAVE(p, m_elevationDeg);
  MCP_SAVE(p, m_projectiveModel);
  MCP_SAVE(p, m_projectiveFOVdeg);
  MCP_SAVE(p, m_useNoProjection);
  MCP_SAVE(p, m_eyeRollDeg);
  MCP_SAVE(p, m_6DOFMode);

  if (m_pinholeModel) p["pinholeModel"] = m_pinholeModel->asYAML();
}
