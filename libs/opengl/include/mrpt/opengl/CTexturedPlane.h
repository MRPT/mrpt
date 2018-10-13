/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CTexturedObject.h>

namespace mrpt::opengl
{
/** A 2D plane in the XY plane with a texture image.
 *  \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CTexturedPlane : public CTexturedObject
{
	DEFINE_SERIALIZABLE(CTexturedPlane)
   protected:
	mutable float m_tex_x_min, m_tex_x_max;
	mutable float m_tex_y_min, m_tex_y_max;

	float m_xMin, m_xMax;
	float m_yMin, m_yMax;

	mutable bool polygonUpToDate{false};
	/** Used for ray-tracing */
	mutable std::vector<mrpt::math::TPolygonWithPlane> tmpPoly;
	void updatePoly() const;
	void unloadTexture();

	void render_texturedobj() const override;

   public:
	/** Set the texture coordinates of the four corners (in the range 0-1). */
	void setTextureCornerCoords(
		float tex_x_min, float tex_x_max, float tex_y_min, float tex_y_max)
	{
		m_tex_x_min = tex_x_min;
		m_tex_x_max = tex_x_max;
		m_tex_y_min = tex_y_min;
		m_tex_y_max = tex_y_max;
		CRenderizableDisplayList::notifyChange();
	}

	/** Set the coordinates of the four corners that define the plane on the XY
	 * plane. */
	void setPlaneCorners(float xMin, float xMax, float yMin, float yMax)
	{
		m_xMin = xMin;
		m_xMax = xMax;
		m_yMin = yMin;
		m_yMax = yMax;
		polygonUpToDate = false;
		CRenderizableDisplayList::notifyChange();
	}

	/** Get the coordinates of the four corners that define the plane on the XY
	 * plane. */
	inline void getPlaneCorners(
		float& xMin, float& xMax, float& yMin, float& yMax) const
	{
		xMin = m_xMin;
		xMax = m_xMax;
		yMin = m_yMin;
		yMax = m_yMax;
	}

	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Constructor
	 */
	CTexturedPlane(
		float x_min = -1, float x_max = 1, float y_min = -1, float y_max = 1);

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CTexturedPlane() override;
};

}  // namespace mrpt::opengl
