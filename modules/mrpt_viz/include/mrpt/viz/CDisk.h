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

#include <mrpt/math/geometry.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/CVisualObject.h>

namespace mrpt::viz
{
/** A planar disk in the XY plane.
 *
 * ![mrpt::viz::CDisk](preview_CDisk.png)
 *
 *  \sa opengl::Scene
 * \ingroup mrpt_viz_grp
 */
class CDisk : virtual public CVisualObject, public VisualObjectParams_Triangles
{
  DEFINE_SERIALIZABLE(CDisk, mrpt::viz)

 public:
  void setDiskRadius(float outRadius, float inRadius = 0)
  {
    m_radiusIn = inRadius;
    m_radiusOut = outRadius;
    CVisualObject::notifyChange();
  }

  [[nodiscard]] float getInRadius() const { return m_radiusIn; }
  [[nodiscard]] float getOutRadius() const { return m_radiusOut; }

  /** Default=50 */
  void setSlicesCount(uint32_t N)
  {
    m_nSlices = N;
    CVisualObject::notifyChange();
  }

  /** Evaluates the bounding box of this object (including possible children)
   * in the coordinate frame of the object parent. */
  [[nodiscard]] mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

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

}  // namespace mrpt::viz
