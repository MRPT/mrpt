/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CFrustum_H
#define opengl_CFrustum_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt	{
namespace opengl	{

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CFrustum,CRenderizableDisplayList, OPENGL_IMPEXP)

	/** A solid or wireframe frustum in 3D (a rectangular truncated pyramid), with arbitrary (possibly assymetric) field-of-view angles.
	  *
	  *  You can switch whether to show only the lines, the surface of the frustum, or both.
	  *  By default only the lines are drawn.
	  *
	  *  The color of the object (via CRenderizable::setColor()) affects the color of lines.
	  *  To set the color of planes use \a setPlaneColor()
	  *
	  *  As usual in MRPT, the +X axis is assumed to by the main direction, in this case of the pyramid axis.
	  *
	  *  The horizontal and vertical FOVs can be set directly with \a setHorzFOV() and \a setVertFOV() if
	  *  they are symmetric, or with \a setHorzFOVAsymmetric() and \a setVertFOVAsymmetric() otherwise.
	  *
	  *  All FOV angles are positive numbers. FOVs must be below 90deg on each side (below 180deg in total).
	  *  If you try to set FOVs to larger values they'll truncated to 89.9deg.
	  *
	  * \sa opengl::COpenGLScene,opengl::CRenderizable
	  *
	  *  <div align="center">
	  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
	  *   <tr> <td> mrpt::opengl::CFrustum </td> <td> \image html preview_CFrustum.png </td> </tr>
	  *  </table>
	  *  </div>
	  *
	  * \ingroup mrpt_opengl_grp
	  */
	class OPENGL_IMPEXP CFrustum :public CRenderizableDisplayList
	{
		DEFINE_SERIALIZABLE(CFrustum)

	protected:
		float  m_min_distance, m_max_distance;    //!< Near and far planes
		float  m_fov_horz_left,m_fov_horz_right;  //!< Semi FOVs (in radians)
		float  m_fov_vert_down,m_fov_vert_up;     //!< Semi FOVs (in radians)
		bool   m_draw_lines, m_draw_planes;
		float  m_lineWidth;
		mrpt::utils::TColor  m_planes_color;

	public:
		/** Constructor returning a smart pointer to the newly created object. */
		static CFrustumPtr Create(float near_distance, float far_distance, float horz_FOV_degrees, float vert_FOV_degrees, float lineWidth = 1.5f, bool draw_lines = true, bool draw_planes = false );

		inline void setLineWidth(float width) { m_lineWidth = width; CRenderizableDisplayList::notifyChange(); }
		inline float getLineWidth() const { return m_lineWidth; }

		/** Changes the color of the planes; to change color of lines, use CRenderizable base methods. */
		inline void setPlaneColor(const mrpt::utils::TColor &c) { m_planes_color=c; CRenderizableDisplayList::notifyChange(); }
		inline const mrpt::utils::TColor &getPlaneColor() const { return m_planes_color; }

		/** Changes distance of near & far planes */
		void setNearFarPlanes(const float near_distance, const float far_distance);

		float getNearPlaneDistance() const { return m_min_distance; }
		float getFarPlaneDistance() const { return m_max_distance; }

		/** Changes horizontal FOV (symmetric) */
		void setHorzFOV(const float fov_horz_degrees);
		/** Changes vertical FOV (symmetric) */
		void setVertFOV(const float fov_vert_degrees);
		/** Changes horizontal FOV (asymmetric) */
		void setHorzFOVAsymmetric(const float fov_horz_left_degrees,const float fov_horz_right_degrees);
		/** Changes vertical FOV (asymmetric) */
		void setVertFOVAsymmetric(const float fov_vert_down_degrees,const float fov_vert_up_degrees);

		float getHorzFOV() const { return mrpt::utils::RAD2DEG(m_fov_horz_left+m_fov_horz_right); }
		float getVertFOV() const { return mrpt::utils::RAD2DEG(m_fov_vert_down+m_fov_vert_up); }
		float getHorzFOVLeft() const { return mrpt::utils::RAD2DEG(m_fov_horz_left); }
		float getHorzFOVRight() const { return mrpt::utils::RAD2DEG(m_fov_horz_right); }
		float getVertFOVDown() const { return mrpt::utils::RAD2DEG(m_fov_vert_down); }
		float getVertFOVUp() const { return mrpt::utils::RAD2DEG(m_fov_vert_up); }

		/** Render \sa mrpt::opengl::CRenderizable */
		void render_dl() const MRPT_OVERRIDE;

		/** Ray tracing. \sa mrpt::opengl::CRenderizable */
		bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const MRPT_OVERRIDE;

		/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
		void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

	private:
		/** Basic empty constructor. Set all parameters to default. */
		CFrustum();
		/** Constructor with some parameters  */
		CFrustum(float near_distance, float far_distance, float horz_FOV_degrees, float vert_FOV_degrees, float lineWidth, bool draw_lines, bool draw_planes);

		/** Destructor  */
		virtual ~CFrustum() { }
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CFrustum,CRenderizableDisplayList, OPENGL_IMPEXP)
}
}
#endif
