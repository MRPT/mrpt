/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/lightweight_geom_data.h>

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
class CBox : public CRenderizableDisplayList
{
	DEFINE_SERIALIZABLE(CBox)

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
	/** Render
	 * \sa mrpt::opengl::CRenderizable
	 */
	void render_dl() const override;

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
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
		CRenderizableDisplayList::notifyChange();
	}
	inline float getLineWidth() const { return m_lineWidth; }
	inline void setWireframe(bool is_wireframe = true)
	{
		m_wireframe = is_wireframe;
		CRenderizableDisplayList::notifyChange();
	}
	inline bool isWireframe() const { return m_wireframe; }
	inline void enableBoxBorder(bool drawBorder = true)
	{
		m_draw_border = drawBorder;
		CRenderizableDisplayList::notifyChange();
	}
	inline bool isBoxBorderEnabled() const { return m_draw_border; }
	inline void setBoxBorderColor(const mrpt::img::TColor& c)
	{
		m_solidborder_color = c;
		CRenderizableDisplayList::notifyChange();
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
