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
#include <mrpt/viz/CVectorField3D.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CVectorField3D, CVisualObject, mrpt::viz)

/** Constructor */
CVectorField3D::CVectorField3D() :
    x_vf(0, 0), y_vf(0, 0), z_vf(0, 0), x_p(0, 0), y_p(0, 0), z_p(0, 0)
{
  m_point_color = m_field_color = m_still_color = m_maxspeed_color = getColor_u8();
  m_maxspeed = 1.f;
}

/** Constructor with a initial set of lines. */
CVectorField3D::CVectorField3D(
    CMatrixFloat x_vf_ini,
    CMatrixFloat y_vf_ini,
    CMatrixFloat z_vf_ini,
    CMatrixFloat x_p_ini,
    CMatrixFloat y_p_ini,
    CMatrixFloat z_p_ini) :
    m_colorFromModule(false), m_showPoints(true)
{
  x_vf = x_vf_ini;
  y_vf = y_vf_ini;
  z_vf = z_vf_ini;
  x_p = x_p_ini;
  y_p = y_p_ini;
  z_p = z_p_ini;
  m_point_color = m_field_color = m_still_color = m_maxspeed_color = getColor_u8();
  m_maxspeed = 1.f;
}

uint8_t CVectorField3D::serializeGetVersion() const { return 1; }
void CVectorField3D::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);

  out << x_vf << y_vf << z_vf;
  out << x_p << y_p << z_p;
  out << m_point_color;
  out << m_field_color;
  VisualObjectParams_Lines::params_serialize(out);   // v1
  VisualObjectParams_Points::params_serialize(out);  // v1
}
void CVectorField3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
      THROW_EXCEPTION("Importing from old version not supported");
      break;

    case 1:
      readFromStreamRender(in);

      in >> x_vf >> y_vf >> z_vf;
      in >> x_p >> y_p >> z_p;
      in >> m_point_color;
      in >> m_field_color;
      VisualObjectParams_Lines::params_deserialize(in);   // v1
      VisualObjectParams_Points::params_deserialize(in);  // v1
      break;

    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
      break;
  };
  CVisualObject::notifyChange();
}

void CVectorField3D::updateBuffers() const
{
  const int rows = static_cast<int>(x_vf.rows());
  const int cols = static_cast<int>(x_vf.cols());

  // Lines buffer: arrows from each point along the field vector
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Lines::m_linesMtx.data);
    auto& vbd = VisualObjectParams_Lines::m_vertex_buffer_data;
    auto& cbd = VisualObjectParams_Lines::m_color_buffer_data;
    vbd.clear();
    cbd.clear();

    for (int r = 0; r < rows; r++)
    {
      for (int c = 0; c < cols; c++)
      {
        const float px = x_p(r, c), py = y_p(r, c), pz = z_p(r, c);
        vbd.emplace_back(px, py, pz);
        vbd.emplace_back(px + x_vf(r, c), py + y_vf(r, c), pz + z_vf(r, c));

        mrpt::img::TColor col;
        if (m_colorFromModule)
        {
          const float module = std::sqrt(
              x_vf(r, c) * x_vf(r, c) + y_vf(r, c) * y_vf(r, c) + z_vf(r, c) * z_vf(r, c));
          const float t = std::min(module / m_maxspeed, 1.0f);
          col.R =
              static_cast<uint8_t>(m_still_color.R + t * (m_maxspeed_color.R - m_still_color.R));
          col.G =
              static_cast<uint8_t>(m_still_color.G + t * (m_maxspeed_color.G - m_still_color.G));
          col.B =
              static_cast<uint8_t>(m_still_color.B + t * (m_maxspeed_color.B - m_still_color.B));
          col.A =
              static_cast<uint8_t>(m_still_color.A + t * (m_maxspeed_color.A - m_still_color.A));
        }
        else
        {
          col = m_field_color;
        }
        cbd.emplace_back(col);
        cbd.emplace_back(col);
      }
    }
  }

  // Points buffer: dot at each sample position
  if (m_showPoints)
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Points::m_pointsMtx.data);
    auto& vbd = VisualObjectParams_Points::m_vertex_buffer_data;
    auto& cbd = VisualObjectParams_Points::m_color_buffer_data;
    vbd.clear();
    cbd.clear();

    for (int r = 0; r < rows; r++)
    {
      for (int c = 0; c < cols; c++)
      {
        vbd.emplace_back(x_p(r, c), y_p(r, c), z_p(r, c));
        cbd.emplace_back(m_point_color);
      }
    }
  }
}

auto CVectorField3D::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  return verticesBoundingBox();
}
