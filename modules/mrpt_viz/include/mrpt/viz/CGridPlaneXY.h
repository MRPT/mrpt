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

#pragma once

#include <mrpt/viz/CVisualObject.h>

namespace mrpt::viz
{
/** A grid of lines over the XY plane.
 *
 * ![mrpt::viz::CGridPlaneXY](preview_CGridPlaneXY.png)
 *
 * \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */
class CGridPlaneXY : virtual public CVisualObject, public VisualObjectParams_Lines
{
  DEFINE_SERIALIZABLE(CGridPlaneXY, mrpt::viz)

 protected:
  float m_xMin, m_xMax;
  float m_yMin, m_yMax;
  float m_plane_z;
  float m_frequency;

 public:
  void setPlaneLimits(float xmin, float xmax, float ymin, float ymax)
  {
    m_xMin = xmin;
    m_xMax = xmax;
    m_yMin = ymin;
    m_yMax = ymax;
    CVisualObject::notifyChange();
  }

  void getPlaneLimits(float& xmin, float& xmax, float& ymin, float& ymax) const
  {
    xmin = m_xMin;
    xmax = m_xMax;
    ymin = m_yMin;
    ymax = m_yMax;
  }

  void setPlaneZcoord(float z)
  {
    CVisualObject::notifyChange();
    m_plane_z = z;
  }
  float getPlaneZcoord() const { return m_plane_z; }
  void setGridFrequency(float freq)
  {
    ASSERT_(freq > 0);
    m_frequency = freq;
    CVisualObject::notifyChange();
  }
  float getGridFrequency() const { return m_frequency; }

  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  /** Constructor  */
  CGridPlaneXY(
      float xMin = -10,
      float xMax = 10,
      float yMin = -10,
      float yMax = 10,
      float z = 0,
      float frequency = 1,
      float lineWidth = 1.3f,
      bool antiAliasing = true);

  /** Private, virtual destructor: only can be deleted from smart pointers */
  ~CGridPlaneXY() override = default;
};

}  // namespace mrpt::viz
