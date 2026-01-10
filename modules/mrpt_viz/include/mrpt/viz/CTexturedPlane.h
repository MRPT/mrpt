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

#include <mrpt/math/TPolygonWithPlane.h>
#include <mrpt/viz/CVisualObject.h>

namespace mrpt::viz
{
/** A 2D plane in the XY plane with a texture image.
 *
 * Lighting is disabled by default in this class, so the plane color or texture
 * will be independent of its orientation or shadows cast on it.
 * This can be changed calling enableLighting(true)
 *
 *  \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */
class CTexturedPlane :
    virtual public CVisualObject,
    public VisualObjectParams_Triangles,
    public VisualObjectParams_TexturedTriangles
{
  DEFINE_SERIALIZABLE(CTexturedPlane, mrpt::viz)

 protected:
  float m_xMin = -1.0f, m_xMax = 1.0f;
  float m_yMin = -1.0f, m_yMax = 1.0f;

  mutable bool polygonUpToDate{false};
  /** Used for ray-tracing */
  mutable std::vector<mrpt::math::TPolygonWithPlane> tmpPoly;
  void updatePoly() const;

 public:
  CTexturedPlane(float x_min = -1, float x_max = 1, float y_min = -1, float y_max = 1);
  virtual ~CTexturedPlane() override = default;

  /** Set the coordinates of the four corners that define the plane on the XY
   * plane. */
  void setPlaneCorners(float xMin, float xMax, float yMin, float yMax)
  {
    ASSERT_NOT_EQUAL_(xMin, xMax);
    ASSERT_NOT_EQUAL_(yMin, yMax);
    m_xMin = xMin;
    m_xMax = xMax;
    m_yMin = yMin;
    m_yMax = yMax;
    polygonUpToDate = false;
    CVisualObject::notifyChange();
  }

  /** Get the coordinates of the four corners that define the plane on the XY
   * plane. */
  void getPlaneCorners(float& xMin, float& xMax, float& yMin, float& yMax) const
  {
    xMin = m_xMin;
    xMax = m_xMax;
    yMin = m_yMin;
    yMax = m_yMax;
  }

  void enableLighting(bool enable = true)
  {
    VisualObjectParams_TexturedTriangles::enableLight(enable);
    VisualObjectParams_Triangles::enableLight(enable);
  }

  bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;
};

}  // namespace mrpt::viz
