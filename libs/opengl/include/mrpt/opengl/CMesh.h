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
#include <mrpt/opengl/CRenderizable.h>
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
class CMesh : public CRenderizable
{
	DEFINE_SERIALIZABLE(CMesh, mrpt::opengl)
   public:
	struct TTriangleVertexIndices
	{
		size_t vind[3] = {0, 0, 0};
	};

   protected:
	mrpt::img::CImage m_textureImage;

	bool m_enableTransparency;
	bool m_colorFromZ{false};
	bool m_isWireFrame{false};
	bool m_isImage{false};

	/** Z(x,y): Z-coordinate of the point (x,y) */
	math::CMatrixF Z;
	math::CMatrixF mask;
	/** Texture coordinates */
	math::CMatrixF U, V;
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
	float xMin, xMax, yMin, yMax;
	/** List of triangles in the mesh */
	mutable std::vector<
		std::pair<CSetOfTriangles::TTriangle, TTriangleVertexIndices>>
		actualMesh;
	/** The accumulated normals & counts for each vertex, so normals can be
	 * averaged. */
	mutable std::vector<std::pair<mrpt::math::TPoint3D, size_t>> vertex_normals;
	/**Whether the actual mesh needs to be recalculated */
	mutable bool trianglesUpToDate{false};
	/**Whether the polygon mesh (auxiliary structure for ray tracing) needs to
	 * be recalculated */
	mutable bool polygonsUpToDate{false};
	mutable std::vector<mrpt::math::TPolygonWithPlane> tmpPolys;

   public:
	void setGridLimits(float xmin, float xmax, float ymin, float ymax)
	{
		xMin = xmin;
		xMax = xmax;
		yMin = ymin;
		yMax = ymax;
		CRenderizable::notifyChange();
	}

	void getGridLimits(float& xmin, float& xmax, float& ymin, float& ymax) const
	{
		xmin = xMin;
		xmax = xMax;
		ymin = yMin;
		ymax = yMax;
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

	/** Sets the (u,v) texture coordinates (in range [0,1]) for each cell */
	void setUV(
		const mrpt::math::CMatrixDynamic<float>& in_U,
		const mrpt::math::CMatrixDynamic<float>& in_V);

	inline float getXMin() const { return xMin; }
	inline float getXMax() const { return xMax; }
	inline float getYMin() const { return yMin; }
	inline float getYMax() const { return yMax; }
	inline void setXMin(const float nxm)
	{
		xMin = nxm;
		trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setXMax(const float nxm)
	{
		xMax = nxm;
		trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setYMin(const float nym)
	{
		yMin = nym;
		trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setYMax(const float nym)
	{
		yMax = nym;
		trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void getXBounds(float& min, float& max) const
	{
		min = xMin;
		max = xMax;
	}
	inline void getYBounds(float& min, float& max) const
	{
		min = yMin;
		max = yMax;
	}
	inline void setXBounds(const float min, const float max)
	{
		xMin = min;
		xMax = max;
		trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setYBounds(const float min, const float max)
	{
		yMin = min;
		yMax = max;
		trianglesUpToDate = false;
		CRenderizable::notifyChange();
	}

	void render() const override;
	void renderUpdateBuffers() const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	/** Assigns a texture image, and disable transparency.
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
	inline void adjustGridToImageAR()
	{
		ASSERT_(m_isImage);
		const float ycenter = 0.5 * (yMin + yMax);
		const float xwidth = xMax - xMin;
		const float newratio = float(m_textureImage.getWidth()) /
							   float(m_textureImage.getHeight());
		yMax = ycenter + 0.5 * newratio * xwidth;
		yMin = ycenter - 0.5 * newratio * xwidth;
		CRenderizable::notifyChange();
	}

	/** Trace ray
	 */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;

	/** Constructor  */
	CMesh(
		bool enableTransparency = false, float xMin = 0.0f, float xMax = 0.0f,
		float yMin = 0.0f, float yMax = 0.0f);

	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CMesh() override;
};

}  // namespace mrpt::opengl
