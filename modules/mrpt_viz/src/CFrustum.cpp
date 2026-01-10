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
#include <mrpt/viz/CFrustum.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CFrustum, CVisualObject, mrpt::viz)

std::array<mrpt::math::TPoint3Df, 8> CFrustum::computeFrustumCorners() const
{
  std::array<mrpt::math::TPoint3Df, 8> pts;
  for (size_t j = 0; j < 2; j++)
  {
    const float r = j == 0 ? m_min_distance : m_max_distance;
    for (size_t i = 0; i < 4; i++) pts[4 * j + i].x = r;
    pts[4 * j + 0].y = -r * tan(m_fov_horz_left);
    pts[4 * j + 1].y = -r * tan(m_fov_horz_left);
    pts[4 * j + 2].y = r * tan(m_fov_horz_right);
    pts[4 * j + 3].y = r * tan(m_fov_horz_right);
    pts[4 * j + 0].z = -r * tan(m_fov_vert_down);
    pts[4 * j + 1].z = r * tan(m_fov_vert_up);
    pts[4 * j + 2].z = -r * tan(m_fov_vert_down);
    pts[4 * j + 3].z = r * tan(m_fov_vert_up);
  }
  return pts;
}

// Ctors
CFrustum::CFrustum() :
    m_fov_horz_left(mrpt::DEG2RAD(45.0f)),
    m_fov_horz_right(mrpt::DEG2RAD(45.0f)),
    m_fov_vert_down(mrpt::DEG2RAD(30.0f)),
    m_fov_vert_up(mrpt::DEG2RAD(30.0f)),
    m_planes_color(0xE0, 0x00, 0x00, 0x50)  // RGBA
{
  keep_min(m_fov_horz_left, DEG2RAD(89.9f));
  keep_max(m_fov_horz_left, 0);
  keep_min(m_fov_horz_right, DEG2RAD(89.9f));
  keep_max(m_fov_horz_right, 0);
  keep_min(m_fov_vert_down, DEG2RAD(89.9f));
  keep_max(m_fov_vert_down, 0);
  keep_min(m_fov_vert_up, DEG2RAD(89.9f));
  keep_max(m_fov_vert_up, 0);
}

CFrustum::CFrustum(
    float near_distance,
    float far_distance,
    float horz_FOV_degrees,
    float vert_FOV_degrees,
    float lineWidth,
    bool draw_lines,
    bool draw_planes) :
    m_min_distance(near_distance),
    m_max_distance(far_distance),
    m_fov_horz_left(mrpt::DEG2RAD(.5f * horz_FOV_degrees)),
    m_fov_horz_right(mrpt::DEG2RAD(.5f * horz_FOV_degrees)),
    m_fov_vert_down(mrpt::DEG2RAD(.5f * vert_FOV_degrees)),
    m_fov_vert_up(mrpt::DEG2RAD(.5f * vert_FOV_degrees)),
    m_draw_lines(draw_lines),
    m_draw_planes(draw_planes),
    m_planes_color(0xE0, 0x00, 0x00, 0x50)  // RGBA
{
  this->setLineWidth(lineWidth);
}

uint8_t CFrustum::serializeGetVersion() const { return 2; }
void CFrustum::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  // version 0
  out << m_min_distance << m_max_distance << m_fov_horz_left << m_fov_horz_right << m_fov_vert_down
      << m_fov_vert_up << m_draw_lines << m_draw_planes;
  VisualObjectParams_Lines::params_serialize(out);  // was: m_lineWidth; // v2
  out << m_planes_color.R << m_planes_color.G << m_planes_color.B << m_planes_color.A;
  VisualObjectParams_Triangles::params_serialize(out);  // v1
}

void CFrustum::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
      readFromStreamRender(in);
      in >> m_min_distance >> m_max_distance >> m_fov_horz_left >> m_fov_horz_right >>
          m_fov_vert_down >> m_fov_vert_up >> m_draw_lines >> m_draw_planes;
      if (version < 2)
      {
        setLineWidth(in.ReadAs<float>());
      }
      else
      {
        VisualObjectParams_Lines::params_deserialize(in);
      }

      in >> m_planes_color.R >> m_planes_color.G >> m_planes_color.B >> m_planes_color.A;

      if (version >= 1) VisualObjectParams_Triangles::params_deserialize(in);

      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

bool CFrustum::traceRay(
    [[maybe_unused]] const mrpt::poses::CPose3D& o, [[maybe_unused]] double& dist) const
{
  THROW_EXCEPTION("TO DO");
}

// setters:
void CFrustum::setNearFarPlanes(const float near_distance, const float far_distance)
{
  m_min_distance = near_distance;
  m_max_distance = far_distance;
  CVisualObject::notifyChange();
}
void CFrustum::setHorzFOV(const float fov_horz_degrees)
{
  m_fov_horz_right = m_fov_horz_left = 0.5f * mrpt::DEG2RAD(fov_horz_degrees);
  keep_min(m_fov_horz_left, DEG2RAD(89.9f));
  keep_max(m_fov_horz_left, 0);
  keep_min(m_fov_horz_right, DEG2RAD(89.9f));
  keep_max(m_fov_horz_right, 0);
  CVisualObject::notifyChange();
}
void CFrustum::setVertFOV(const float fov_vert_degrees)
{
  m_fov_vert_down = m_fov_vert_up = 0.5f * mrpt::DEG2RAD(fov_vert_degrees);
  keep_min(m_fov_vert_down, DEG2RAD(89.9f));
  keep_max(m_fov_vert_down, 0);
  keep_min(m_fov_vert_up, DEG2RAD(89.9f));
  keep_max(m_fov_vert_up, 0);
  CVisualObject::notifyChange();
}
void CFrustum::setHorzFOVAsymmetric(
    const float fov_horz_left_degrees, const float fov_horz_right_degrees)
{
  m_fov_horz_left = mrpt::DEG2RAD(fov_horz_left_degrees);
  m_fov_horz_right = mrpt::DEG2RAD(fov_horz_right_degrees);
  keep_min(m_fov_horz_left, DEG2RAD(89.9f));
  keep_max(m_fov_horz_left, 0);
  keep_min(m_fov_horz_right, DEG2RAD(89.9f));
  keep_max(m_fov_horz_right, 0);
  CVisualObject::notifyChange();
}
void CFrustum::setVertFOVAsymmetric(
    const float fov_vert_down_degrees, const float fov_vert_up_degrees)
{
  m_fov_vert_down = mrpt::DEG2RAD(fov_vert_down_degrees);
  m_fov_vert_up = mrpt::DEG2RAD(fov_vert_up_degrees);
  keep_min(m_fov_vert_down, DEG2RAD(89.9f));
  keep_max(m_fov_vert_down, 0);
  keep_min(m_fov_vert_up, DEG2RAD(89.9f));
  keep_max(m_fov_vert_up, 0);
  CVisualObject::notifyChange();
}

auto CFrustum::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  std::array<mrpt::math::TPoint3Df, 8> pts = computeFrustumCorners();

  auto bb = mrpt::math::TBoundingBoxf::PlusMinusInfinity();

  for (const auto& pt : pts) bb.updateWithPoint(pt);

  return bb;
}

CFrustum::CFrustum(const mrpt::img::TCamera& i, double focalScale) :
    CFrustum(
        i.fx() * focalScale * 0.1f,
        i.fx() * focalScale,
        2 * mrpt::RAD2DEG(std::atan2(i.ncols, 2 * i.fx())),
        2 * mrpt::RAD2DEG(std::atan2(i.nrows, 2 * i.fy())),
        1.0f,
        true,
        false)
{
}
