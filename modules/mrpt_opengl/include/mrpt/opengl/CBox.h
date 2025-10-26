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

#include <mrpt/math/TOrientedBox.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

namespace mrpt::opengl
{
/** A solid or wireframe box in 3D, defined by 6 rectangular faces parallel to
 *the planes X, Y and Z (note that the object can be translated and rotated
 *afterwards as any other CRenderizable object using the "object pose" in the
 *base class).
 *  Three drawing modes are possible:
 *	- Wireframe: setWireframe(true). Used color is the CRenderizable color
 *	- Solid box: setWireframe(false). Used color is the CRenderizable color
 *	- Solid box with border: setWireframe(false) + enableBoxBorder(true). Solid
 *color is the CRenderizable color, border line can be set with
 *setBoxBorderColor().
 *
 * ![mrpt::opengl::CBox](preview_CBox.png)
 *
 * \sa opengl::Scene,opengl::CRenderizable
 * \ingroup mrpt_opengl_grp
 */
class CBox : public CRenderizableShaderTriangles, public CRenderizableShaderWireFrame
{
  DEFINE_SERIALIZABLE(CBox, mrpt::opengl)

 public:
  /** @name Renderizable shader API virtual methods
   * @{ */
  void render(const RenderContext& rc) const override;
  void renderUpdateBuffers() const override;

  virtual shader_list_t requiredShaders() const override
  {
    // May use up to two shaders (triangles and lines):
    return {DefaultShaderID::WIREFRAME, DefaultShaderID::TRIANGLES_LIGHT};
  }
  void onUpdateBuffers_Wireframe() override;
  void onUpdateBuffers_Triangles() override;
  void freeOpenGLResources() override
  {
    CRenderizableShaderTriangles::freeOpenGLResources();
    CRenderizableShaderWireFrame::freeOpenGLResources();
  }
  /** @} */

  mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

  /**
   * Ray tracing.
   * \sa mrpt::opengl::CRenderizable
   */
  bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

  inline void setLineWidth(float width)
  {
    m_lineWidth = width;
    CRenderizable::notifyChange();
  }
  inline float getLineWidth() const { return m_lineWidth; }
  inline void setWireframe(bool is_wireframe = true)
  {
    m_wireframe = is_wireframe;
    CRenderizable::notifyChange();
  }
  inline bool isWireframe() const { return m_wireframe; }
  inline void enableBoxBorder(bool drawBorder = true)
  {
    m_draw_border = drawBorder;
    CRenderizable::notifyChange();
  }
  inline bool isBoxBorderEnabled() const { return m_draw_border; }
  inline void setBoxBorderColor(const mrpt::img::TColor& c)
  {
    m_solidborder_color = c;
    CRenderizable::notifyChange();
  }
  inline mrpt::img::TColor getBoxBorderColor() const { return m_solidborder_color; }

  /** Set the position and size of the box, from two corners in 3D */
  void setBoxCorners(const mrpt::math::TPoint3D& corner1, const mrpt::math::TPoint3D& corner2);
  void getBoxCorners(mrpt::math::TPoint3D& corner1, mrpt::math::TPoint3D& corner2) const
  {
    corner1 = m_corner_min;
    corner2 = m_corner_max;
  }

  /** Basic empty constructor. Set all parameters to default. */
  CBox() = default;

  /** Constructor with all the parameters  */
  CBox(
      const mrpt::math::TPoint3D& corner1,
      const mrpt::math::TPoint3D& corner2,
      bool is_wireframe = false,
      float lineWidth = 1.0);

  /** Destructor  */
  ~CBox() override = default;

  /** Set box pose and size from an mrpt::math::TOrientedBox */
  template <typename T>
  explicit CBox(const mrpt::math::TOrientedBox_<T>& ob)
  {
    setFromOrientedBox(ob);
  }

  template <typename T>
  void setFromOrientedBox(const mrpt::math::TOrientedBox_<T>& ob)
  {
    const auto size = ob.size().template cast<double>();
    m_corner_min = size * (-0.5);
    m_corner_max = size * (+0.5);
    this->setPose(ob.pose());
  }

 protected:
  /** Corners coordinates */
  mrpt::math::TPoint3D m_corner_min = {-1, -1, -1}, m_corner_max = {1, 1, 1};
  /** true: wireframe, false (default): solid */
  bool m_wireframe{false};

  /** Draw line borders to solid box with the given linewidth (default: true)
   */
  bool m_draw_border{true};

  /** Color of the solid box borders. */
  mrpt::img::TColor m_solidborder_color = {0, 0, 0};
};
}  // namespace mrpt::opengl
