/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/CRenderizableShaderTriangles.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

#include <array>

namespace mrpt::opengl
{
/** A 3D mesh composed of triangles and/or quads.
 * A typical usage example would be a 3D model of an object.
 *
 * ![mrpt::opengl::CMesh3D](preview_CMesh3D.png)
 *
 * \sa opengl::Scene,opengl::CMesh,opengl::CAssimpModel
 * \ingroup mrpt_opengl_grp
 */
class CMesh3D : public CRenderizableShaderTriangles,
				public CRenderizableShaderWireFrame,
				public CRenderizableShaderPoints
{
	DEFINE_SERIALIZABLE(CMesh3D, mrpt::opengl)

   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;
	void freeOpenGLResources() override
	{
		CRenderizableShaderTriangles::freeOpenGLResources();
		CRenderizableShaderWireFrame::freeOpenGLResources();
		CRenderizableShaderPoints::freeOpenGLResources();
	}

	virtual shader_list_t requiredShaders() const override
	{
		return {
			DefaultShaderID::WIREFRAME, DefaultShaderID::TRIANGLES_LIGHT,
			DefaultShaderID::POINTS};
	}
	void onUpdateBuffers_Wireframe() override;
	void onUpdateBuffers_Triangles() override;
	void onUpdateBuffers_Points() override;
	/** @} */

	CMesh3D() = default;
	virtual ~CMesh3D() override;

	void enableShowEdges(bool v)
	{
		m_showEdges = v;
		CRenderizable::notifyChange();
	}
	void enableShowFaces(bool v)
	{
		m_showFaces = v;
		CRenderizable::notifyChange();
	}
	void enableShowVertices(bool v)
	{
		m_showVertices = v;
		CRenderizable::notifyChange();
	}
	void enableFaceNormals(bool v)
	{
		m_computeNormals = v;
		CRenderizable::notifyChange();
	}

	/** Load a 3D mesh. The arguments indicate:
		- num_verts: Number of vertices of the mesh
		- num_faces: Number of faces of the mesh
		- verts_per_face: An array (pointer) with the number of vertices of each
	   face. The elements must be set either to 3 (triangle) or 4 (quad).
		- face_verts: An array (pointer) with the vertices of each face. The
	   vertices of each face must be consecutive in this array.
		- vert_coords: An array (pointer) with the coordinates of each vertex.
	   The xyz coordinates of each vertex must be consecutive in this array.
	  */
	void loadMesh(
		unsigned int num_verts, unsigned int num_faces, int* verts_per_face,
		int* face_verts, float* vert_coords);

	/** Load a 3D mesh. The arguments indicate:
		- num_verts: Number of vertices of the mesh
		- num_faces: Number of faces of the mesh
		- is_quad: A binary array saying whether the face is a quad (1)
	   or a triangle (0)
		- face_verts: An array with the vertices of each face. For every
	   column (face), each row contains the num of a vertex. The fourth does not
	   need
		  to be filled if the face is a triangle.
		- vert_coords: An array with the coordinates of each vertex. For
	   every column (vertex), each row contains the xyz coordinates of the
	   vertex.
	 */
	void loadMesh(
		unsigned int num_verts, unsigned int num_faces,
		const mrpt::math::CMatrixDynamic<bool>& is_quad,
		const mrpt::math::CMatrixDynamic<int>& face_verts,
		const mrpt::math::CMatrixDynamic<float>& vert_coords);

	void setEdgeColor(float r, float g, float b, float a = 1.f)
	{
		edge_color = mrpt::img::TColorf(r, g, b, a);
	}
	void setFaceColor(float r, float g, float b, float a = 1.f)
	{
		face_color = mrpt::img::TColorf(r, g, b, a);
	}
	void setVertColor(float r, float g, float b, float a = 1.f)
	{
		vert_color = mrpt::img::TColorf(r, g, b, a);
	}

	mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

   protected:
	using vertex_indices_t = mrpt::math::CMatrixFixed<uint32_t, 1, 4>;

	bool m_showEdges = true;
	bool m_showFaces = true;
	bool m_showVertices = false;
	bool m_computeNormals = true;

	/** Pointer storing whether a face is a quad (1) or a triangle (0) */
	std::vector<bool> m_is_quad;
	/** Pointer storing the vertices that compose each face. Size: 4 x num_faces
	 * (4 for the possible max number - quad) */
	std::vector<vertex_indices_t> m_face_verts;

	/** Pointer storing the coordinates of the vertices. Size: 3 x num_vertices
	 */
	std::vector<mrpt::math::TPoint3Df> m_vertices;

	/** Pointer storing the face normals. Size: 3 x num_faces */
	std::vector<mrpt::math::TPoint3Df> m_normals;

	/** Color of the edges */
	mrpt::img::TColorf edge_color = {.9f, .9f, .9f, 1.0f};

	/** Color of the faces */
	mrpt::img::TColorf face_color = {.7f, .7f, .8f, 1.0f};

	/** Color of the vertices */
	mrpt::img::TColorf vert_color = {.3f, .3f, .3f, 1.0f};
};

}  // namespace mrpt::opengl
