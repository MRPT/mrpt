/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizableShaderTexturedTriangles.h>

namespace mrpt::opengl
{
/** A set of textured triangles.
 *  This class can be used to draw any solid, arbitrarily complex object with
 * textures.
 *  \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CSetOfTexturedTriangles : public CRenderizableShaderTexturedTriangles
{
	DEFINE_SERIALIZABLE(CSetOfTexturedTriangles, mrpt::opengl)

   public:
	using TVertex = mrpt::opengl::TTriangle::Vertex;
	using TTriangle = mrpt::opengl::TTriangle;

	CSetOfTexturedTriangles() = default;
	virtual ~CSetOfTexturedTriangles() override = default;

	void onUpdateBuffers_TexturedTriangles() override;

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	void clearTriangles()
	{
		m_triangles.clear();
		CRenderizable::notifyChange();
	}
	size_t getTrianglesCount() const { return m_triangles.size(); }
	const TTriangle& getTriangle(size_t idx) const
	{
		ASSERT_BELOW_(idx, m_triangles.size());
		return m_triangles[idx];
	}
	void getTriangle(size_t idx, TTriangle& t) const
	{
		ASSERT_BELOW_(idx, m_triangles.size());
		t = m_triangles[idx];
		CRenderizable::notifyChange();
	}
	void insertTriangle(const TTriangle& t)
	{
		m_triangles.push_back(t);
		CRenderizable::notifyChange();
	}

	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
};

}  // namespace mrpt::opengl
