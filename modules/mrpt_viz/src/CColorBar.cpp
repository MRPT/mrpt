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

mrpt::math::TBoundingBoxf CColorBar::internalBoundingBoxLocal() const
{
  return mrpt::math::TBoundingBoxf::FromUnsortedPoints(
      {.0f, .0f, .0f}, {d2f(m_width), d2f(m_height), .0f});
}
