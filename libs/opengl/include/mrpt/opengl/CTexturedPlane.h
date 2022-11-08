/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPolygonWithPlane.h>
#include <mrpt/opengl/CRenderizableShaderTexturedTriangles.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>

namespace mrpt::opengl
{
/** A 2D plane in the XY plane with a texture image.
 *  Lighting is disabled in this class, so the plane color or texture will be
 *  independent of its orientation.
 *
 *  \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CTexturedPlane : public CRenderizableShaderTexturedTriangles,
					   public CRenderizableShaderTriangles
{
	DEFINE_SERIALIZABLE(CTexturedPlane, mrpt::opengl)

   protected:
	float m_xMin = -1.0f, m_xMax = 1.0f;
	float m_yMin = -1.0f, m_yMax = 1.0f;

	mutable bool polygonUpToDate{false};
	/** Used for ray-tracing */
	mutable std::vector<mrpt::math::TPolygonWithPlane> tmpPoly;
	void updatePoly() const;

   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;
	virtual void onUpdateBuffers_TexturedTriangles() override;
	virtual void onUpdateBuffers_Triangles() override;
	virtual shader_list_t requiredShaders() const override
	{
		return {
			DefaultShaderID::TRIANGLES_NO_LIGHT,
			DefaultShaderID::TEXTURED_TRIANGLES_NO_LIGHT};
	}
	void freeOpenGLResources() override
	{
		CRenderizableShaderTriangles::freeOpenGLResources();
		CRenderizableShaderTexturedTriangles::freeOpenGLResources();
	}

	/** Control whether to render the FRONT, BACK, or BOTH (default) set of
	 * faces. Refer to docs for glCullFace() */
	void cullFaces(const TCullFace& cf)
	{
		CRenderizableShaderTexturedTriangles::cullFaces(cf);
		CRenderizableShaderTriangles::cullFaces(cf);
	}
	TCullFace cullFaces() const
	{
		return CRenderizableShaderTexturedTriangles::cullFaces();
	}

	/** @} */

	CTexturedPlane(
		float x_min = -1, float x_max = 1, float y_min = -1, float y_max = 1);
	virtual ~CTexturedPlane() override = default;

	/** Set the coordinates of the four corners that define the plane on the XY
	 * plane. */
	void setPlaneCorners(float xMin, float xMax, float yMin, float yMax)
	{
		ASSERT_NOT_EQUAL_(xMin, xMax);
		ASSERT_NOT_EQUAL_(yMin, yMax);
		m_xMin = xMin;
		m_xMax = xMax;
		m_yMin = yMin;
		m_yMax = yMax;
		polygonUpToDate = false;
		CRenderizable::notifyChange();
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
	mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;
};

}  // namespace mrpt::opengl
