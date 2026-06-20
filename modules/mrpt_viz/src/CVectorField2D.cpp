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
#include <mrpt/viz/CVectorField2D.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CVectorField2D, CVisualObject, mrpt::viz)

/** Constructor */
CVectorField2D::CVectorField2D() : xcomp(0, 0), ycomp(0, 0)
{
  m_point_color = getColor_u8();
  m_field_color = getColor_u8();
}

/** Constructor with a initial set of lines. */
CVectorField2D::CVectorField2D(
    [[maybe_unused]] CMatrixFloat Matrix_x,
    [[maybe_unused]] CMatrixFloat Matrix_y,
    [[maybe_unused]] float xmin,
    [[maybe_unused]] float xmax,
    [[maybe_unused]] float ymin,
    [[maybe_unused]] float ymax)
{
  m_point_color = getColor_u8();
  m_field_color = getColor_u8();
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
   CSerializable objects
  ---------------------------------------------------------------*/
uint8_t CVectorField2D::serializeGetVersion() const { return 2; }
void CVectorField2D::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << xcomp << ycomp;
  out << xMin << xMax << yMin << yMax;
  out << m_point_color;
  out << m_field_color;
  VisualObjectParams_Lines::params_serialize(out);
  VisualObjectParams_Points::params_serialize(out);
  VisualObjectParams_Triangles::params_serialize(out);
}

void CVectorField2D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
      THROW_EXCEPTION("Unsupported old serialized version");
      break;

    case 2:
      readFromStreamRender(in);

      in >> xcomp >> ycomp;
      in >> xMin >> xMax >> yMin >> yMax;
      in >> m_point_color;
      in >> m_field_color;

      VisualObjectParams_Lines::params_deserialize(in);
      VisualObjectParams_Points::params_deserialize(in);
      VisualObjectParams_Triangles::params_deserialize(in);

      break;

    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
      break;
  };
  CVisualObject::notifyChange();
}

void CVectorField2D::updateBuffers() const
{
  const int rows = static_cast<int>(xcomp.rows());
  const int cols = static_cast<int>(xcomp.cols());

  const float dx = (cols > 1) ? (xMax - xMin) / static_cast<float>(cols - 1) : 0.0f;
  const float dy = (rows > 1) ? (yMax - yMin) / static_cast<float>(rows - 1) : 0.0f;

  // Lines buffer: stem of each vector arrow
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Lines::m_linesMtx.data);
    auto& vbd = VisualObjectParams_Lines::m_vertex_buffer_data;
    auto& cbd = VisualObjectParams_Lines::m_color_buffer_data;
    vbd.clear();
    cbd.clear();
    vbd.reserve(static_cast<size_t>(rows * cols) * 2);

    for (int c = 0; c < cols; c++)
    {
      for (int r = 0; r < rows; r++)
      {
        const float px = xMin + static_cast<float>(c) * dx;
        const float py = yMin + static_cast<float>(r) * dy;
        vbd.emplace_back(px, py, 0.0f);
        vbd.emplace_back(px + xcomp(r, c), py + ycomp(r, c), 0.0f);
      }
    }
    cbd.assign(vbd.size(), m_field_color);
  }

  // Triangles buffer: arrowhead at the tip of each vector
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Triangles::m_trianglesMtx.data);
    auto& tris = VisualObjectParams_Triangles::m_triangles;
    tris.clear();
    tris.reserve(static_cast<size_t>(rows * cols));

    using P3f = mrpt::math::TPoint3Df;
    for (int c = 0; c < cols; c++)
    {
      for (int r = 0; r < rows; r++)
      {
        const float vx = xcomp(r, c), vy = ycomp(r, c);
        const float tri_side = 0.25f * std::sqrt(vx * vx + vy * vy);
        const float ang = std::atan2(vy, vx) - 1.5708f;
        const float tip_x = xMin + static_cast<float>(c) * dx + vx;
        const float tip_y = yMin + static_cast<float>(r) * dy + vy;
        TTriangle t(
            P3f(-std::sin(ang) * 0.866f * tri_side + tip_x,
                std::cos(ang) * 0.866f * tri_side + tip_y, 0.0f),
            P3f(std::cos(ang) * 0.5f * tri_side + tip_x, std::sin(ang) * 0.5f * tri_side + tip_y,
                0.0f),
            P3f(-std::cos(ang) * 0.5f * tri_side + tip_x, -std::sin(ang) * 0.5f * tri_side + tip_y,
                0.0f));
        t.setColor(m_field_color);
        tris.emplace_back(std::move(t));
      }
    }
  }

  // Points buffer: dot at each grid position
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Points::m_pointsMtx.data);
    auto& vbd = VisualObjectParams_Points::m_vertex_buffer_data;
    auto& cbd = VisualObjectParams_Points::m_color_buffer_data;
    vbd.clear();
    cbd.clear();
    vbd.reserve(static_cast<size_t>(rows * cols));

    for (int c = 0; c < cols; c++)
    {
      for (int r = 0; r < rows; r++)
      {
        vbd.emplace_back(
            xMin + static_cast<float>(c) * dx, yMin + static_cast<float>(r) * dy, 0.0f);
      }
    }
    cbd.assign(vbd.size(), m_point_color);
  }
}

auto CVectorField2D::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return verticesBoundingBox();
}

void CVectorField2D::adjustVectorFieldToGrid()
{
  ASSERT_(xcomp.size() > 0);

  const float ratio_xp = xcomp.maxCoeff() * static_cast<float>(xcomp.cols() - 1) / (xMax - xMin);
  const float ratio_xn = xcomp.minCoeff() * static_cast<float>(xcomp.cols() - 1) / (xMax - xMin);
  const float ratio_yp = ycomp.maxCoeff() * static_cast<float>(ycomp.rows() - 1) / (yMax - yMin);
  const float ratio_yn = ycomp.minCoeff() * static_cast<float>(ycomp.rows() - 1) / (yMax - yMin);
  const float norm_factor =
      0.85f / max(max(ratio_xp, std::abs(ratio_xn)), max(ratio_yp, std::abs(ratio_yn)));

  xcomp *= norm_factor;
  ycomp *= norm_factor;
  CVisualObject::notifyChange();
}
