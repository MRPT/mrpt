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

#include <mrpt/math/TPoint3D.h>
#include <mrpt/viz/CVisualObject.h>

namespace mrpt::viz
{
/** A solid or wireframe box in 3D, defined by 6 rectangular faces parallel to
 * the planes X, Y and Z (note that the object can be translated and rotated
 * afterwards as any other CVisualObject object using the "object pose" in the
 * base class).
 *
 * Three drawing modes are possible:
 * - Wireframe: setWireframe(true). Used color is the CVisualObject color
 * - Solid box: setWireframe(false). Used color is the CVisualObject color
 * - Solid box with border: setWireframe(false) + enableBoxBorder(true). Solid
 *   color is the CVisualObject color, border line can be set with
 *   setBoxBorderColor().
 *
 * ![mrpt::viz::CBox](preview_CBox.png)
 *
 * \sa mrpt::viz::Scene, mrpt::viz::CVisualObject
 * \ingroup mrpt_viz_grp
 */
class CBox :
    public virtual CVisualObject,
    public virtual VisualObjectParams_Lines,
    public virtual VisualObjectParams_Triangles
{
  DEFINE_SERIALIZABLE(CBox, mrpt::viz)

 public:
  /** Basic empty constructor. Set all parameters to default. */
  CBox() = default;

  /** Constructor with all the parameters */
  CBox(
      const mrpt::math::TPoint3D& corner1,
      const mrpt::math::TPoint3D& corner2,
      bool is_wireframe = false,
      float lineWidth = 1.0);

  /** Destructor */
  ~CBox() override = default;

  /** @name Geometry Configuration
   * @{ */

  /** Set the position and size of the box, from two corners in 3D */
  void setBoxCorners(const mrpt::math::TPoint3D& corner1, const mrpt::math::TPoint3D& corner2);

  /** Get the current box corners */
  void getBoxCorners(mrpt::math::TPoint3D& corner1, mrpt::math::TPoint3D& corner2) const
  {
    corner1 = m_corner_min;
    corner2 = m_corner_max;
  }

  /** Returns the minimum corner */
  [[nodiscard]] const mrpt::math::TPoint3D& getCornerMin() const { return m_corner_min; }

  /** Returns the maximum corner */
  [[nodiscard]] const mrpt::math::TPoint3D& getCornerMax() const { return m_corner_max; }

  /** @} */

  /** @name Rendering Style
   * @{ */

  /** Sets wireframe rendering mode (true) or solid mode (false, default) */
  void setWireframe(bool is_wireframe = true)
  {
    m_wireframe = is_wireframe;
    CVisualObject::notifyChange();
  }

  /** Returns true if wireframe mode is enabled */
  [[nodiscard]] bool isWireframe() const { return m_wireframe; }

  /** Enable/disable drawing a border around solid boxes */
  void enableBoxBorder(bool drawBorder = true)
  {
    m_draw_border = drawBorder;
    CVisualObject::notifyChange();
  }

  /** Returns true if box border is enabled */
  [[nodiscard]] bool isBoxBorderEnabled() const { return m_draw_border; }

  /** Set the color of the box border lines */
  void setBoxBorderColor(const mrpt::img::TColor& c)
  {
    m_solidborder_color = c;
    CVisualObject::notifyChange();
  }

  /** Get the color of the box border lines */
  [[nodiscard]] mrpt::img::TColor getBoxBorderColor() const { return m_solidborder_color; }

  /** @} */

  /** @name CVisualObject Interface
   * @{ */

  [[nodiscard]] mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  /** Ray tracing.
   * \sa mrpt::viz::CVisualObject
   */
  bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

  /** Updates the internal geometry buffers for rendering.
   *
   * This is called automatically by the rendering system when hasToUpdateBuffers()
   * returns true (i.e., after notifyChange() was called).
   */
  void updateBuffers() const override;

  /** @} */

 protected:
  /** Corners coordinates */
  mrpt::math::TPoint3D m_corner_min = {-1, -1, -1};
  mrpt::math::TPoint3D m_corner_max = {1, 1, 1};

  /** true: wireframe, false (default): solid */
  bool m_wireframe = false;

  /** Draw line borders to solid box (default: true) */
  bool m_draw_border = true;

  /** Color of the solid box borders. */
  mrpt::img::TColor m_solidborder_color = {0, 0, 0};

 private:
  /** Populate m_triangles with the 12 triangles forming the 6 box faces */
  void updateTrianglesBuffer() const;

  /** Populate line vertex buffer with the 12 edges of the box */
  void updateLinesBuffer() const;
};

}  // namespace mrpt::viz