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
/** A cylinder or cone whose base lies in the XY plane.
 *
 * The cylinder extends along the +Z axis from z=0 to z=height.
 * - If baseRadius == topRadius: regular cylinder
 * - If baseRadius != topRadius: truncated cone (frustum)
 * - If topRadius == 0: cone with apex at the top
 * - If baseRadius == 0: inverted cone with apex at the bottom
 *
 * ![mrpt::viz::CCylinder](preview_CCylinder.png)
 *
 * \sa mrpt::viz::Scene, mrpt::viz::CDisk
 * \ingroup mrpt_viz_grp
 */
class CCylinder : public virtual CVisualObject, public VisualObjectParams_Triangles
{
  DEFINE_SERIALIZABLE(CCylinder, mrpt::viz)
  DEFINE_SCHEMA_SERIALIZABLE()

 public:
  /** Default constructor: unit cylinder */
  CCylinder() = default;

  /** Constructor with parameters.
   * \param baseRadius Radius at z=0
   * \param topRadius Radius at z=height
   * \param height Height of the cylinder (along +Z)
   * \param slices Number of radial divisions (higher = smoother)
   */
  CCylinder(float baseRadius, float topRadius, float height = 1.0f, int slices = 20) :
      m_baseRadius(baseRadius),
      m_topRadius(topRadius),
      m_height(height),
      m_slices(slices),
      m_hasTopBase(true),
      m_hasBottomBase(true)
  {
  }

  /** Destructor */
  ~CCylinder() override = default;

  /** @name Geometry Configuration
   * @{ */

  /** Sets both radii to a single value, configuring the object as a cylinder.
   * \sa setRadii
   */
  void setRadius(float radius)
  {
    m_baseRadius = m_topRadius = radius;
    CVisualObject::notifyChange();
  }

  /** Sets both radii independently.
   * \sa setRadius
   */
  void setRadii(float bottom, float top)
  {
    m_baseRadius = bottom;
    m_topRadius = top;
    CVisualObject::notifyChange();
  }

  /** Changes cylinder's height. */
  void setHeight(float height)
  {
    m_height = height;
    CVisualObject::notifyChange();
  }

  /** Gets the bottom radius. */
  [[nodiscard]] float getBottomRadius() const { return m_baseRadius; }

  /** Gets the top radius. */
  [[nodiscard]] float getTopRadius() const { return m_topRadius; }

  /** Gets the cylinder's height. */
  [[nodiscard]] float getHeight() const { return m_height; }

  /** Number of radial divisions (affects mesh smoothness) */
  void setSlicesCount(uint32_t slices)
  {
    m_slices = slices;
    CVisualObject::notifyChange();
  }

  /** Returns the number of radial divisions */
  [[nodiscard]] uint32_t getSlicesCount() const { return m_slices; }

  /** @} */

  /** @name Base Caps Configuration
   * @{ */

  /** Configuration of the cylinder's bases display.
   * \param top Whether to draw the top cap
   * \param bottom Whether to draw the bottom cap
   */
  void setHasBases(bool top = true, bool bottom = true)
  {
    m_hasTopBase = top;
    m_hasBottomBase = bottom;
    CVisualObject::notifyChange();
  }

  /** Check whether top base is displayed.
   * \sa hasBottomBase
   */
  [[nodiscard]] bool hasTopBase() const { return m_hasTopBase; }

  /** Check whether bottom base is displayed.
   * \sa hasTopBase
   */
  [[nodiscard]] bool hasBottomBase() const { return m_hasBottomBase; }

  /** @} */

  /** @name CVisualObject Interface
   * @{ */

  bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

  [[nodiscard]] mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  /** Updates the internal triangle buffer for rendering.
   *
   * Generates the triangulated mesh for the lateral surface and optional
   * top/bottom caps.
   */
  void updateBuffers() const override;

  /** @} */

 protected:
  /** Radius at z=0 */
  float m_baseRadius = 1.0f;

  /** Radius at z=height */
  float m_topRadius = 1.0f;

  /** Cylinder height */
  float m_height = 1.0f;

  /** Number of radial divisions */
  uint32_t m_slices = 20;

  /** Whether to draw the top cap */
  bool m_hasTopBase = true;

  /** Whether to draw the bottom cap */
  bool m_hasBottomBase = true;

 private:
  /** Gets the radius at a given height Z.
   * \return false if Z is outside the cylinder's height range
   */
  [[nodiscard]] bool getRadius(float Z, float& r) const
  {
    if (!reachesHeight(Z))
    {
      return false;
    }
    r = (Z / m_height) * (m_topRadius - m_baseRadius) + m_baseRadius;
    return true;
  }

  /** Checks whether the cylinder exists at some height. */
  [[nodiscard]] bool reachesHeight(float Z) const
  {
    return (m_height < 0) ? (Z >= m_height && Z <= 0) : (Z <= m_height && Z >= 0);
  }

  [[nodiscard]] bool reachesHeight(double Z) const { return reachesHeight(d2f(Z)); }
};

}  // namespace mrpt::viz