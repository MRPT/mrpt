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
#include <mrpt/viz/CColorBar.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CColorBar, CVisualObject, mrpt::viz)

CColorBar::CColorBar(
    /** The colormap to represent. */
    const mrpt::img::TColormap colormap,
    /** size of the color bar */
    double width,
    double height,
    /** limits for [0,1] colormap indices */
    float min_col,
    float max_col,
    /** limits for values associated to extreme colors */
    float min_value,
    float max_value,
    /** sprintf-like format string for values */
    const std::string& label_format,
    /** Label text font size */
    float label_font_size) :
    m_colormap(colormap),
    m_width(width),
    m_height(height),
    m_label_format(label_format),
    m_min_col(min_col),
    m_max_col(max_col),
    m_min_value(min_value),
    m_max_value(max_value),
    m_label_font_size(label_font_size)
{
}

void CColorBar::setColormap(const mrpt::img::TColormap colormap)
{
  m_colormap = colormap;
  CVisualObject::notifyChange();
}

void CColorBar::setColorAndValueLimits(
    float col_min, float col_max, float value_min, float value_max)
{
  m_min_col = col_min;
  m_max_col = col_max;
  m_min_value = value_min;
  m_max_value = value_max;
  CVisualObject::notifyChange();
}

uint8_t CColorBar::serializeGetVersion() const { return 2; }
void CColorBar::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  // version 0
  out << uint32_t(m_colormap) << m_min_col << m_max_col << m_min_value << m_max_value
      << m_label_format << m_label_font_size;
  VisualObjectParams_Triangles::params_serialize(out);  // v2
}
void CColorBar::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
      readFromStreamRender(in);

      in.ReadAsAndCastTo<uint32_t, mrpt::img::TColormap>(m_colormap);
      in >> m_min_col >> m_max_col >> m_min_value >> m_max_value >> m_label_format >>
          m_label_font_size;
      if (version == 0)
      {
        bool old_disable_depth_test;
        in >> old_disable_depth_test;
      }
      if (version >= 2) VisualObjectParams_Triangles::params_deserialize(in);

      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

void CColorBar::updateBuffers() const
{
  constexpr unsigned int NUM_DIVISIONS = 64;
  constexpr unsigned int NUM_LABELS = 4;
  constexpr unsigned int ONE_LABEL_EACH_NTH = NUM_DIVISIONS / NUM_LABELS;

  const float x0 = 0.0f, x1 = d2f(m_width), x2 = d2f(m_width * 1.3);
  const float Ay = d2f(m_height) / (NUM_DIVISIONS - 1);

  // Precompute per-division colors
  std::vector<mrpt::img::TColor> colors(NUM_DIVISIONS);
  for (unsigned int i = 0; i < NUM_DIVISIONS; i++)
  {
    const float col_idx = m_min_col + i * (m_max_col - m_min_col) / (NUM_DIVISIONS - 1);
    mrpt::img::TColorf colf = mrpt::img::colormap(m_colormap, col_idx);
    colf.A = 1.0f;
    colors[i] = colf.asTColor();
  }

  // Triangles — the colored bar itself
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Triangles::m_trianglesMtx.data);
    auto& tris = VisualObjectParams_Triangles::m_triangles;
    tris.clear();

    for (unsigned int i = 0; i < NUM_DIVISIONS - 1; i++)
    {
      const float y0 = Ay * i, y1 = Ay * (i + 1);
      const TPoint3Df pt00(x0, y0, 0), pt10(x1, y0, 0);
      const TPoint3Df pt01(x0, y1, 0), pt11(x1, y1, 0);

      // Two triangles per quad strip, with per-vertex colors for the gradient
      TTriangle t;

      t.vertices[0].xyzrgba.pt = pt00;
      t.vertices[0].setColor(colors[i]);
      t.vertices[1].xyzrgba.pt = pt10;
      t.vertices[1].setColor(colors[i]);
      t.vertices[2].xyzrgba.pt = pt11;
      t.vertices[2].setColor(colors[i + 1]);
      t.computeNormals();
      tris.emplace_back(t);

      t.vertices[0].xyzrgba.pt = pt00;
      t.vertices[0].setColor(colors[i]);
      t.vertices[1].xyzrgba.pt = pt11;
      t.vertices[1].setColor(colors[i + 1]);
      t.vertices[2].xyzrgba.pt = pt01;
      t.vertices[2].setColor(colors[i + 1]);
      t.computeNormals();
      tris.emplace_back(t);
    }
  }

  // Lines — tick marks at label positions
  {
    std::unique_lock<std::shared_mutex> lck(VisualObjectParams_Lines::m_linesMtx.data);
    auto& vbd = VisualObjectParams_Lines::m_vertex_buffer_data;
    auto& cbd = VisualObjectParams_Lines::m_color_buffer_data;
    vbd.clear();
    cbd.clear();

    const auto tickColor = mrpt::img::TColor::black();

    for (unsigned int i = 0; i < NUM_DIVISIONS; i++)
    {
      const bool draw_label = (i % ONE_LABEL_EACH_NTH) == 0 || i == (NUM_DIVISIONS - 1);
      if (draw_label)
      {
        const float y0 = Ay * i;
        vbd.emplace_back(x0, y0, 0);
        vbd.emplace_back(x2, y0, 0);
        cbd.emplace_back(tickColor);
        cbd.emplace_back(tickColor);
      }
    }
  }
}

mrpt::math::TBoundingBoxf CColorBar::internalBoundingBoxLocal() const
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints(
      {.0f, .0f, .0f}, {d2f(m_width), d2f(m_height), .0f});
}
