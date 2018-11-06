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
#include <mrpt/img/color_maps.h>
#include <Eigen/Dense>

using Eigen::Array;
using Eigen::Dynamic;

namespace mrpt::opengl
{
/** A 3D mesh composed of Triangles and/or Quads.
 * A typical usage example would be a 3D model of an object.
 *  \sa opengl::COpenGLScene,opengl::CMesh,opengl::CAssimpModel
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CMesh3D </td> <td> \image html preview_CMesh3D.png
 * </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CMesh3D : public CRenderizableDisplayList
{
	DEFINE_SERIALIZABLE(CMesh3D)

	using f_verts = int[4];
	using coord3D = float[3];

   protected:
	bool m_enableTransparency;
	bool m_antiAliasing;
	bool m_showEdges;
	bool m_showFaces;
	bool m_showVertices;
	bool m_computeNormals{true};
	float m_lineWidth{2.f};
	float m_pointSize{6.f};

	// Data
	/** Number of vertices of the mesh */
	unsigned int m_num_verts;
	/** Number of faces of the mesh */
	unsigned int m_num_faces;
	/** Pointer storing whether a face is a quad (1) or a triangle (0) */
	bool* m_is_quad;
	/** Pointer storing the vertices that compose each face. Size: 4 x num_faces
	 * (4 for the possible max number - quad) */
	f_verts* m_face_verts;
	/** Pointer storing the coordinates of the vertices. Size: 3 x num_vertices
	 */
	coord3D* m_vert_coords;
	/** Pointer storing the face normals. Size: 3 x num_faces */
	coord3D* m_normals;

	// Colors
	/** Color of the edges (when shown) */
	float edge_color[4];
	/** Color of the faces (when shown) */
	float face_color[4];
	/** Color of the vertices (when shown) */
	float vert_color[4];
	mrpt::img::TColormap m_colorMap{
		mrpt::img::cmHOT};  // Not used yet. I leave it here in case
	// I want to use it in the future

   public:
	void enableTransparency(bool v)
	{
		m_enableTransparency = v;
		CRenderizableDisplayList::notifyChange();
	}
	void enableAntiAliasing(bool v)
	{
		m_antiAliasing = v;
		CRenderizableDisplayList::notifyChange();
	}
	void enableShowEdges(bool v)
	{
		m_showEdges = v;
		CRenderizableDisplayList::notifyChange();
	}
	void enableShowFaces(bool v)
	{
		m_showFaces = v;
		CRenderizableDisplayList::notifyChange();
	}
	void enableShowVertices(bool v)
	{
		m_showVertices = v;
		CRenderizableDisplayList::notifyChange();
	}
	void enableFaceNormals(bool v)
	{
		m_computeNormals = v;
		CRenderizableDisplayList::notifyChange();
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
		- is_quad: A binary array (Eigen) saying whether the face is a quad (1)
	   or a triangle (0)
		- face_verts: An array (Eigen) with the vertices of each face. For every
	   column (face), each row contains the num of a vertex. The fourth does not
	   need
		  to be filled if the face is a triangle.
		- vert_coords: An array (Eigen) with the coordinates of each vertex. For
	   every column (vertex), each row contains the xyz coordinates of the
	   vertex.
	 */
	void loadMesh(
		unsigned int num_verts, unsigned int num_faces,
		const Array<bool, 1, Dynamic>& is_quad,
		const Array<int, 4, Dynamic>& face_verts,
		const Array<float, 3, Dynamic>& vert_coords);

	void setEdgeColor(float r, float g, float b, float a = 1.f);
	void setFaceColor(float r, float g, float b, float a = 1.f);
	void setVertColor(float r, float g, float b, float a = 1.f);
	void setLineWidth(float lw) { m_lineWidth = lw; }
	void setPointSize(float ps) { m_pointSize = ps; }
	/** Render
	 */
	void render_dl() const override;

	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent.
	 */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Constructor */
	CMesh3D(
		bool enableTransparency = false, bool antiAliasing = false,
		bool enableShowEdges = true, bool enableShowFaces = true,
		bool enableShowVertices = false);
	/** Private, virtual destructor: only can be deleted from smart pointers  */
	~CMesh3D() override;
};

}  // namespace mrpt::opengl
