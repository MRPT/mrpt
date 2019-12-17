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
/** A solid or wireframe frustum in 3D (a rectangular truncated pyramid), with
 * arbitrary (possibly assymetric) field-of-view angles.
 *
 *  You can switch whether to show only the lines, the surface of the frustum,
 * or both.
 *  By default only the lines are drawn.
 *
 *  The color of the object (via CRenderizable::setColor()) affects the color
 * of lines.
 *  To set the color of planes use \a setPlaneColor()
 *
 *  As usual in MRPT, the +X axis is assumed to by the main direction, in this
 * case of the pyramid axis.
 *
 *  The horizontal and vertical FOVs can be set directly with \a setHorzFOV()
 * and \a setVertFOV() if
 *  they are symmetric, or with \a setHorzFOVAsymmetric() and \a
 * setVertFOVAsymmetric() otherwise.
 *
 *  All FOV angles are positive numbers. FOVs must be below 90deg on each side
 * (below 180deg in total).
 *  If you try to set FOVs to larger values they'll truncated to 89.9deg.
 *
 * \sa opengl::COpenGLScene,opengl::CRenderizable
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CFrustum </td> <td> \image html
 * preview_CFrustum.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CFrustum : public CRenderizable
{
	DEFINE_SERIALIZABLE(CFrustum, mrpt::opengl)

   protected:
	/** Near and far planes */
	float m_min_distance{0.1f}, m_max_distance{1.f};
	/** Semi FOVs (in radians) */
	float m_fov_horz_left, m_fov_horz_right;
	/** Semi FOVs (in radians) */
	float m_fov_vert_down, m_fov_vert_up;
	bool m_draw_lines{true}, m_draw_planes{false};
	float m_lineWidth{1.5f};
	mrpt::img::TColor m_planes_color;

   public:
	inline void setLineWidth(float width)
	{
		m_lineWidth = width;
		CRenderizable::notifyChange();
	}
	inline float getLineWidth() const { return m_lineWidth; }
	/** Changes the color of the planes; to change color of lines, use
	 * CRenderizable base methods. */
	inline void setPlaneColor(const mrpt::img::TColor& c)
	{
		m_planes_color = c;
		CRenderizable::notifyChange();
	}
	inline const mrpt::img::TColor& getPlaneColor() const
	{
		return m_planes_color;
	}

	/** Changes distance of near & far planes */
	void setNearFarPlanes(const float near_distance, const float far_distance);

	float getNearPlaneDistance() const { return m_min_distance; }
	float getFarPlaneDistance() const { return m_max_distance; }
	/** Changes horizontal FOV (symmetric) */
	void setHorzFOV(const float fov_horz_degrees);
	/** Changes vertical FOV (symmetric) */
	void setVertFOV(const float fov_vert_degrees);
	/** Changes horizontal FOV (asymmetric) */
	void setHorzFOVAsymmetric(
		const float fov_horz_left_degrees, const float fov_horz_right_degrees);
	/** Changes vertical FOV (asymmetric) */
	void setVertFOVAsymmetric(
		const float fov_vert_down_degrees, const float fov_vert_up_degrees);

	float getHorzFOV() const
	{
		return mrpt::RAD2DEG(m_fov_horz_left + m_fov_horz_right);
	}
	float getVertFOV() const
	{
		return mrpt::RAD2DEG(m_fov_vert_down + m_fov_vert_up);
	}
	float getHorzFOVLeft() const { return mrpt::RAD2DEG(m_fov_horz_left); }
	float getHorzFOVRight() const { return mrpt::RAD2DEG(m_fov_horz_right); }
	float getVertFOVDown() const { return mrpt::RAD2DEG(m_fov_vert_down); }
	float getVertFOVUp() const { return mrpt::RAD2DEG(m_fov_vert_up); }

	void render() const override;
	void renderUpdateBuffers() const override;
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Basic empty constructor. Set all parameters to default. */
	CFrustum();
	/** Constructor with some parameters  */
	CFrustum(
		float near_distance, float far_distance, float horz_FOV_degrees,
		float vert_FOV_degrees, float lineWidth, bool draw_lines,
		bool draw_planes);

	/** Destructor  */
	~CFrustum() override = default;
};
}  // namespace mrpt::opengl
