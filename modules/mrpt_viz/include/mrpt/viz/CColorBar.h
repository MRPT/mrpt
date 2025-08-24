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
#pragma once

#include <mrpt/img/color_maps.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/viz/CVisualObject.h>

namespace mrpt::viz
{
/** A colorbar indicator. This class renders a colorbar as a 3D object, in the
 * XY plane.
 * For an overlay indicator that can be easily added to any display, see
 * Scene::addColorBar()
 *
 * ![mrpt::viz::CColorBar](preview_CColorBar.png)
 *
 * \sa mrpt::viz::Scene, mrpt::viz::CRenderizable, Scene::addColorBar()
 * \ingroup mrpt_viz_grp
 */
class CColorBar : virtual public CVisualObject, public VisualObjectParams_Triangles
{
  DEFINE_SERIALIZABLE(CColorBar, mrpt::viz)

 public:
  CColorBar(/** The colormap to represent. */
            const mrpt::img::TColormap colormap = mrpt::img::cmGRAYSCALE,
            /** size of the color bar */
            double width = 0.2,
            double height = 1.0,
            /** limits for [0,1] colormap indices */
            float min_col = .0,
            float max_col = 1.0,
            /** limits for values associated to extreme colors */
            float min_value = .0,
            float max_value = 1.0,
            /** sprintf-like format string for values */
            const std::string& label_format = std::string("%7.02f"),
            /** Label text font size */
            float label_font_size = .05f);

  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  void setColormap(const mrpt::img::TColormap colormap);
  void setColorAndValueLimits(float col_min, float col_max, float value_min, float value_max);

 protected:
  mrpt::img::TColormap m_colormap;
  double m_width, m_height;
  std::string m_label_format;
  float m_min_col, m_max_col, m_min_value, m_max_value;
  float m_label_font_size;
};
}  // namespace mrpt::viz
