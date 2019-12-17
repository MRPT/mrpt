/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CRenderizable.h>

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
 * \sa opengl::COpenGLScene,opengl::CRenderizable
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 *border-style: solid;">
 *   <tr> <td> mrpt::opengl::CBox </td> <td> \image html preview_CBox.png </td>
 *</tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CBox : public CRenderizable
{
	DEFINE_SERIALIZABLE(CBox, mrpt::opengl)

   protected:
	/** Corners coordinates */
	mrpt::math::TPoint3D m_corner_min, m_corner_max;
	/** true: wireframe, false: solid */
	bool m_wireframe{false};
	/** For wireframe only. */
	float m_lineWidth{1};
	/** Draw line borders to solid box with the given linewidth (default: true)
	 */
	bool m_draw_border{false};
	/** Color of the solid box borders. */
	mrpt::img::TColor m_solidborder_color;

   public:
	void render() const override;
	void renderUpdateBuffers() const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

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
	inline mrpt::img::TColor getBoxBorderColor() const
	{
		return m_solidborder_color;
	}

	/** Set the position and size of the box, from two corners in 3D */
	void setBoxCorners(
		const mrpt::math::TPoint3D& corner1,
		const mrpt::math::TPoint3D& corner2);
	void getBoxCorners(
		mrpt::math::TPoint3D& corner1, mrpt::math::TPoint3D& corner2) const
	{
		corner1 = m_corner_min;
		corner2 = m_corner_max;
	}

	/** Basic empty constructor. Set all parameters to default. */
	CBox();

	/** Constructor with all the parameters  */
	CBox(
		const mrpt::math::TPoint3D& corner1,
		const mrpt::math::TPoint3D& corner2, bool is_wireframe = false,
		float lineWidth = 1.0);

	/** Destructor  */
	~CBox() override = default;

   private:
};
}  // namespace mrpt::opengl
