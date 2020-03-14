/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/opengl/CRenderizableShaderTexturedTriangles.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>
#include <mrpt/opengl/CSetOfTriangles.h>

namespace mrpt::opengl
{
/** A planar (XY) grid where each cell has an associated height and, optionally,
 * a texture map.
 *  A typical usage example would be an elevation map or a 3D model of a
 * terrain.
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CMesh </td> <td> \image html preview_CMesh.png
 * </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CMesh : public CRenderizableShaderTexturedTriangles,
			  public CRenderizableShaderWireFrame
{
	DEFINE_SERIALIZABLE(CMesh, mrpt::opengl)
   public:
	struct TTriangleVertexIndices
	{
		size_t vind[3] = {0, 0, 0};
	};

   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	virtual shader_list_t requiredShaders() const override
	{
		// May use up to two shaders (triangles and lines):
		return {DefaultShaderID::WIREFRAME,
				DefaultShaderID::TEXTURED_TRIANGLES};
	}
	void onUpdateBuffers_Wireframe() override;
	void onUpdateBuffers_TexturedTriangles() override;
	void freeOpenGLResources() override
	{
		CRenderizableShaderTexturedTriangles::freeOpenGLResources();
		CRenderizableShaderWireFrame::freeOpenGLResources();
	}
	/** @} */

	CMesh(
		bool enableTransparency = false, float xMin = -1.0f, float xMax = 1.0f,
		float yMin = -1.0f, float yMax = 1.0f);

	virtual ~CMesh() override;

	template <typename T>
	void setGridLimits(T xMin, T xMax, T yMin, T yMax)
	{
		m_xMin = static_cast<float>(xMin);
		m_xMax = static_cast<float>(xMax);
		m_yMin = static_cast<float>(yMin);
		m_yMax = static_cast<float>(yMax);
		CRenderizable::notifyChange();
	}

	void getGridLimits(float& xMin, float& xMax, float& yMin, float& yMax) const
	{
		xMin = m_xMin;
		xMax = m_xMax;
		yMin = m_yMin;
		yMax = m_yMax;
	}

	void enableTransparency(bool v)
	{
		m_enableTransparency = v;
		CRenderizable::notifyChange();
	}
	void enableWireFrame(bool v)
	{
		m_isWireFrame = v;
		CRenderizable::notifyChange();
	}
	void enableColorFromZ(
		bool v, mrpt::img::TColormap colorMap = mrpt::img::cmHOT)
	{
		m_colorFromZ = v;
		m_colorMap = colorMap;
		CRenderizable::notifyChange();
	}

	/** This method sets the matrix of heights for each position (cell) in the
	 * mesh grid */
	void setZ(const mrpt::math::CMatrixDynamic<float>& in_Z);

	/** Returns a reference to the internal Z matrix, allowing changing it
	 * efficiently */
	inline void getZ(mrpt::math::CMatrixFloat& out) const { out = Z; }
	/** Returns a reference to the internal mask matrix, allowing changing it
	 * efficiently */
	inline void getMask(mrpt::math::CMatrixFloat& out) const { out = mask; }
	/** This method sets the boolean mask of valid heights for each position
	 * (cell) in the mesh grid */
	void setMask(const mrpt::math::CMatrixDynamic<float>& in_mask);

	inline float getxMin() const { return m_xMin; }
	inline float getxMax() const { return m_xMax; }
	inline float getyMin() const { return m_yMin; }
	inline float getyMax() const { return m_yMax; }
	inline void setxMin(const float nxm)
	{
		m_xMin = nxm;
		m_trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setxMax(const float nxm)
	{
		m_xMax = nxm;
		m_trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setyMin(const float nym)
	{
		m_yMin = nym;
		m_trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setyMax(const float nym)
	{
		m_yMax = nym;
		m_trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void getXBounds(float& min, float& max) const
	{
		min = m_xMin;
		max = m_xMax;
	}
	inline void getYBounds(float& min, float& max) const
	{
		min = m_yMin;
		max = m_yMax;
	}
	inline void setXBounds(const float min, const float max)
	{
		m_xMin = min;
		m_xMax = max;
		m_trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setYBounds(const float min, const float max)
	{
		m_yMin = min;
		m_yMax = max;
		m_trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}

	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Assigns a texture image.
	 */
	void assignImage(const mrpt::img::CImage& img);

	/** Assigns a texture image and Z simultaneously, and disable transparency.
	 */
	void assignImageAndZ(
		const mrpt::img::CImage& img,
		const mrpt::math::CMatrixDynamic<float>& in_Z);

	/** Adjust grid limits according to the image aspect ratio, maintaining the
	 * X limits and resizing in the Y direction.
	 */
	void adjustGridToImageAR();

	/** Trace ray
	 */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

   protected:
	bool m_enableTransparency;
	bool m_colorFromZ{false};
	bool m_isWireFrame{false};
	bool m_isImage{false};

	/** Z(x,y): Z-coordinate of the point (x,y) */
	math::CMatrixF Z;
	math::CMatrixF mask;

	/** Grayscale Color [0,1] for each cell, updated by updateColorsMatrix */
	mutable math::CMatrixF C;
	/** Red Component of the Color [0,1] for each cell, updated by
	 * updateColorsMatrix */
	mutable math::CMatrixF C_r;
	/** Green Component of the  Color [0,1] for each cell, updated by
	 * updateColorsMatrix */
	mutable math::CMatrixF C_g;
	/** Blue Component of the  Color [0,1] for each cell, updated by
	 * updateColorsMatrix */
	mutable math::CMatrixF C_b;

	/** Used when m_colorFromZ is true */
	mrpt::img::TColormap m_colorMap{mrpt::img::cmHOT};

	/** Whether C is not up-to-date wrt to Z */
	mutable bool m_modified_Z{true};
	/** Whether C is not up-to-date wrt to the texture image */
	mutable bool m_modified_Image{false};

	/** Called internally to assure C is updated. */
	void updateColorsMatrix() const;
	/** Called internally to assure the triangle list is updated. */
	void updateTriangles() const;
	void updatePolygons() const;  //<!Called internally to assure that the
	// polygon list is updated.

	/** Mesh bounds */
	float m_xMin, m_xMax, m_yMin, m_yMax;
	/** List of triangles in the mesh */
	mutable std::vector<
		std::pair<mrpt::opengl::TTriangle, TTriangleVertexIndices>>
		actualMesh;
	/** The accumulated normals & counts for each vertex, so normals can be
	 * averaged. */
	mutable std::vector<std::pair<mrpt::math::TPoint3D, size_t>> vertex_normals;
	/**Whether the actual mesh needs to be recalculated */
	mutable bool m_trianglesUpToDate{false};
	/**Whether the polygon mesh (auxiliary structure for ray tracing) needs to
	 * be recalculated */
	mutable bool m_polygonsUpToDate{false};
	mutable std::vector<mrpt::math::TPolygonWithPlane> tmpPolys;
};

}  // namespace mrpt::opengl
