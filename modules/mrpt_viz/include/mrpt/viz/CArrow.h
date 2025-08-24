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

#include <mrpt/viz/CVisualObject.h>

namespace mrpt::viz
{
/** A 3D arrow
 *
 *  ![mrpt::viz::CArrow](preview_CArrow.png)
 *
 * \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */
class CArrow : virtual public CVisualObject, public VisualObjectParams_Triangles
{
  DEFINE_SERIALIZABLE(CArrow, mrpt::viz)
  DEFINE_SCHEMA_SERIALIZABLE()

 public:
  void setArrowEnds(float x0, float y0, float z0, float x1, float y1, float z1)
  {
    m_x0 = x0;
    m_y0 = y0;
    m_z0 = z0;
    m_x1 = x1;
    m_y1 = y1;
    m_z1 = z1;
    CVisualObject::notifyChange();
  }
  template <typename Vector3Like>
  void setArrowEnds(const Vector3Like& start, const Vector3Like& end)
  {
    m_x0 = start[0];
    m_y0 = start[1];
    m_z0 = start[2];
    m_x1 = end[0];
    m_y1 = end[1];
    m_z1 = end[2];
    CVisualObject::notifyChange();
  }
  void setHeadRatio(float rat)
  {
    m_headRatio = rat;
    CVisualObject::notifyChange();
  }
  void setSmallRadius(float rat)
  {
    m_smallRadius = rat;
    CVisualObject::notifyChange();
  }
  void setLargeRadius(float rat)
  {
    m_largeRadius = rat;
    CVisualObject::notifyChange();
  }
  /** Number of radial divisions  */
  void setSlicesCount(uint32_t slices)
  {
    m_slices = slices;
    CVisualObject::notifyChange();
  }

  /** Number of radial divisions  */
  uint32_t getSlicesCount() const { return m_slices; }

  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  /** Constructor */
  CArrow(
      float x0 = 0,
      float y0 = 0,
      float z0 = 0,
      float x1 = 1,
      float y1 = 1,
      float z1 = 1,
      float headRatio = 0.2f,
      float smallRadius = 0.05f,
      float largeRadius = 0.2f) :
      m_x0(x0),
      m_y0(y0),
      m_z0(z0),
      m_x1(x1),
      m_y1(y1),
      m_z1(z1),
      m_headRatio(headRatio),
      m_smallRadius(smallRadius),
      m_largeRadius(largeRadius)
  {
  }

  CArrow(
      const mrpt::math::TPoint3Df& from,
      const mrpt::math::TPoint3Df& to,
      float headRatio = 0.2f,
      float smallRadius = 0.05f,
      float largeRadius = 0.2f) :
      m_x0(from.x),
      m_y0(from.y),
      m_z0(from.z),
      m_x1(to.x),
      m_y1(to.y),
      m_z1(to.z),
      m_headRatio(headRatio),
      m_smallRadius(smallRadius),
      m_largeRadius(largeRadius)
  {
  }
  ~CArrow() override = default;

 protected:
  mutable float m_x0, m_y0, m_z0;
  mutable float m_x1, m_y1, m_z1;
  float m_headRatio;
  float m_smallRadius, m_largeRadius;
  /** Number of radial divisions. */
  uint32_t m_slices = 10;
};

}  // namespace mrpt::viz
