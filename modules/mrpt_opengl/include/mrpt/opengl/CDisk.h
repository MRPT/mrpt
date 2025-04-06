/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt::opengl
{
/** A planar disk in the XY plane.
 *
 * ![mrpt::opengl::CDisk](preview_CDisk.png)
 *
 *  \sa opengl::Scene
 * \ingroup mrpt_opengl_grp
 */
class CDisk : public CRenderizableShaderTriangles
{
  DEFINE_SERIALIZABLE(CDisk, mrpt::opengl)

 public:
  /** @name Renderizable shader API virtual methods
   * @{ */
  void onUpdateBuffers_Triangles() override;
  /** @} */

  void setDiskRadius(float outRadius, float inRadius = 0)
  {
    m_radiusIn = inRadius;
    m_radiusOut = outRadius;
    CRenderizable::notifyChange();
  }

  float getInRadius() const { return m_radiusIn; }
  float getOutRadius() const { return m_radiusOut; }

  /** Default=50 */
  void setSlicesCount(uint32_t N)
  {
    m_nSlices = N;
    CRenderizable::notifyChange();
  }

  /** Evaluates the bounding box of this object (including possible children)
   * in the coordinate frame of the object parent. */
  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  /** Ray tracing
   */
  bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

  /** Constructor
   */
  CDisk() = default;
  CDisk(float rOut, float rIn, uint32_t slices = 50) :
      m_radiusIn(rIn), m_radiusOut(rOut), m_nSlices(slices)
  {
  }

  /** Private, virtual destructor: only can be deleted from smart pointers */
  ~CDisk() override = default;

 protected:
  float m_radiusIn = 0, m_radiusOut = 1;
  uint32_t m_nSlices = 50;
};

}  // namespace mrpt::opengl
