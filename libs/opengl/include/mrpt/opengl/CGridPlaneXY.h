/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

namespace mrpt::opengl
{
/** A grid of lines over the XY plane.
 *
 * ![mrpt::opengl::CGridPlaneXY](preview_CGridPlaneXY.png)
 *
 * \sa opengl::Scene
 * \ingroup mrpt_opengl_grp
 */
class CGridPlaneXY : public CRenderizableShaderWireFrame
{
  DEFINE_SERIALIZABLE(CGridPlaneXY, mrpt::opengl)

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
    CRenderizable::notifyChange();
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
    CRenderizable::notifyChange();
    m_plane_z = z;
  }
  float getPlaneZcoord() const { return m_plane_z; }
  void setGridFrequency(float freq)
  {
    ASSERT_(freq > 0);
    m_frequency = freq;
    CRenderizable::notifyChange();
  }
  float getGridFrequency() const { return m_frequency; }

  void onUpdateBuffers_Wireframe() override;

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

}  // namespace mrpt::opengl
