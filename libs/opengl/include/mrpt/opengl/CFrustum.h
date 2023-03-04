/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/TCamera.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

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
 *
 * ![mrpt::opengl::CFrustum](preview_CFrustum.png)
 *
 * \sa opengl::Scene,opengl::CRenderizable
 * \ingroup mrpt_opengl_grp
 */
class CFrustum : public CRenderizableShaderTriangles,
				 public CRenderizableShaderWireFrame
{
	DEFINE_SERIALIZABLE(CFrustum, mrpt::opengl)

   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;
	void freeOpenGLResources() override
	{
		CRenderizableShaderTriangles::freeOpenGLResources();
		CRenderizableShaderWireFrame::freeOpenGLResources();
	}

	virtual shader_list_t requiredShaders() const override
	{
		// May use up to two shaders (triangles and lines):
		return {DefaultShaderID::WIREFRAME, DefaultShaderID::TRIANGLES_LIGHT};
	}
	void onUpdateBuffers_Wireframe() override;
	void onUpdateBuffers_Triangles() override;
	/** @} */

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

	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
	mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

	/** Basic empty constructor. Set all parameters to default. */
	CFrustum();
	/** Constructor with some parameters  */
	CFrustum(
		float near_distance, float far_distance, float horz_FOV_degrees,
		float vert_FOV_degrees, float lineWidth, bool draw_lines,
		bool draw_planes);

	/** Constructor from camera intrinsic parameters: creates a frustrum with
	 * the correct vertical and horizontal FOV angles for the given camera
	 * model, in wireframe.
	 *
	 * \param intrinsics Camera intrinsics. Distortion is ignored here.
	 * \param focalDistScale Scale for the far plane, in meters/pixels.
	 *
	 * \note (New in MRPT 2.1.8)
	 */
	CFrustum(
		const mrpt::img::TCamera& intrinsics, double focalDistScale = 5e-3);

	/** Destructor  */
	~CFrustum() override = default;

   protected:
	/** Near and far planes */
	float m_min_distance{0.1f}, m_max_distance{1.f};
	/** Semi FOVs (in radians) */
	float m_fov_horz_left, m_fov_horz_right;
	/** Semi FOVs (in radians) */
	float m_fov_vert_down, m_fov_vert_up;
	bool m_draw_lines{true}, m_draw_planes{false};
	mrpt::img::TColor m_planes_color;

	// Compute the 8 corners of the frustum:
	std::array<mrpt::math::TPoint3Df, 8> computeFrustumCorners() const;
};
}  // namespace mrpt::opengl
