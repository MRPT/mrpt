/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizableShaderTriangles.h>

namespace mrpt::opengl
{
class CCylinder;
/** A cylinder or cone whose base lies in the XY plane.
 *
 * ![mrpt::opengl::CCylinder](preview_CCylinder.png)
 *
 * \sa opengl::Scene,opengl::CDisk
 * \ingroup mrpt_opengl_grp
 */
class CCylinder : public CRenderizableShaderTriangles
{
  DEFINE_SERIALIZABLE(CCylinder, mrpt::opengl)
  DEFINE_SCHEMA_SERIALIZABLE()
 public:
  /** @name Renderizable shader API virtual methods
   * @{ */
  void onUpdateBuffers_Triangles() override;
  /** @} */

  bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
  /**
   * Configuration of the cylinder's bases display.
   */
  inline void setHasBases(bool top = true, bool bottom = true)
  {
    m_hasTopBase = top;
    m_hasBottomBase = bottom;
    CRenderizable::notifyChange();
  }
  /**
   * Check whether top base is displayed.
   * \sa hasBottomBase
   */
  inline bool hasTopBase() const { return m_hasTopBase; }
  /**
   * Check whether bottom base is displayed.
   * \sa hasTopBase
   */
  inline bool hasBottomBase() const { return m_hasBottomBase; }
  /**
   * Sets both radii to a single value, thus configuring the object as a
   * cylinder.
   * \sa setRadii
   */
  inline void setRadius(float radius)
  {
    m_baseRadius = m_topRadius = radius;
    CRenderizable::notifyChange();
  }
  /**
   * Sets both radii independently.
   * \sa setRadius
   */
  inline void setRadii(float bottom, float top)
  {
    m_baseRadius = bottom;
    m_topRadius = top;
    CRenderizable::notifyChange();
  }
  /**
   * Chenges cylinder's height.
   */
  inline void setHeight(float height)
  {
    m_height = height;
    CRenderizable::notifyChange();
  }
  /**
   * Gets the bottom radius.
   */
  inline float getBottomRadius() const { return m_baseRadius; }
  /**
   * Gets the top radius.
   */
  inline float getTopRadius() const { return m_topRadius; }
  /**
   * Gets the cylinder's height.
   */
  inline float getHeight() const { return m_height; }

  /** Number of radial divisions  */
  inline void setSlicesCount(uint32_t slices)
  {
    m_slices = slices;
    CRenderizable::notifyChange();
  }

  /** Number of radial divisions  */
  inline uint32_t getSlicesCount() const { return m_slices; }

  /** Evaluates the bounding box of this object (including possible children)
   * in the coordinate frame of the object parent. */
  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  CCylinder() = default;
  /**
   * Complete constructor. Allows the configuration of every parameter.
   */
  /** Constructor with two radii. Allows the construction of any cylinder. */
  CCylinder(
      const float baseRadius,
      const float topRadius,
      const float height = 1,
      const int slices = 10) :
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

 protected:
  /**
   * Cylinder's radii. If m_baseRadius==m_topRadius, then the object is an
   * actual cylinder. If both differ, it's a truncated cone. If one of the
   * radii is zero, the object is a cone.
   */
  float m_baseRadius{1}, m_topRadius{1};
  /**
   * Cylinder's height
   */
  float m_height{1};

  /** Number of radial divisions. */
  uint32_t m_slices{10};

  /**
   * Boolean parameters about including the bases in the object. If both
   * m_hasTopBase and m_hasBottomBase are set to false, only the lateral area
   * is displayed.
   */
  bool m_hasTopBase{true}, m_hasBottomBase{true};

 private:
  /**
   * Gets the radius of the circunference located at certain height,
   * returning false if the cylinder doesn't get that high.
   */
  inline bool getRadius(float Z, float& r) const
  {
    if (!reachesHeight(Z)) return false;
    r = (Z / m_height) * (m_topRadius - m_baseRadius) + m_baseRadius;
    return true;
  }
  /**
   * Checks whether the cylinder exists at some height.
   */
  inline bool reachesHeight(float Z) const
  {
    return (m_height < 0) ? (Z >= m_height && Z <= 0) : (Z <= m_height && Z >= 0);
  }
  inline bool reachesHeight(double Z) const { return reachesHeight(d2f(Z)); }
};
}  // namespace mrpt::opengl
