/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CTexturedObject.h>

namespace mrpt::opengl
{
/** A set of textured triangles.
 *  This class can be used to draw any solid, arbitrarily complex object with
 * textures.
 *  \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CSetOfTexturedTriangles : public CTexturedObject
{
	DEFINE_SERIALIZABLE(CSetOfTexturedTriangles, mrpt::opengl)

   public:
	/** Triangle vertex. This structure encapsulates the vertex coordinates and
	 * the image pixels.
	 */
	struct TVertex
	{
		/** Default constructor			 */
		TVertex();
		TVertex(float x, float y, float z, uint32_t u, uint32_t v);
		/** 3D vertex coordinates. */
		float m_x{0.0}, m_y{0.0}, m_z{0.0};
		/** 2D texture coordinates. Notice that the texture coordinates are 2D
		 * pixels!!! */
		uint32_t m_u{0}, m_v{0};
		void writeToStream(mrpt::serialization::CArchive& out) const;
		void readFromStream(mrpt::serialization::CArchive& in);
	};

	/** Triangle. This structure encapsulates the triangle vertices.
	 */
	struct TTriangle
	{
		/** Default constructor */
		TTriangle();
		TTriangle(TVertex v1, TVertex v2, TVertex v3);
		/** vertices */
		TVertex m_v1, m_v2, m_v3;
		void writeToStream(mrpt::serialization::CArchive& out) const;
		void readFromStream(mrpt::serialization::CArchive& in);
	};

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

   protected:
	/** Triangle array. */
	std::vector<TTriangle> m_triangles;

	void renderUpdateBuffers() const override;
	void render_texturedobj() const override;

   public:
	void clearTriangles()
	{
		m_triangles.clear();
		CRenderizable::notifyChange();
	}
	size_t getTrianglesCount() const { return m_triangles.size(); }
	const TTriangle& getTriangle(size_t idx) const
	{
		ASSERT_(idx < m_triangles.size());
		return m_triangles[idx];
	}
	void getTriangle(size_t idx, TTriangle& t) const
	{
		ASSERT_(idx < m_triangles.size());
		t = m_triangles[idx];
		CRenderizable::notifyChange();
	}
	void insertTriangle(const TTriangle& t)
	{
		m_triangles.push_back(t);
		CRenderizable::notifyChange();
	}

	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	/** Constructor
	 */
	CSetOfTexturedTriangles() : m_triangles() {}
	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CSetOfTexturedTriangles() override;
};

}  // namespace mrpt::opengl
