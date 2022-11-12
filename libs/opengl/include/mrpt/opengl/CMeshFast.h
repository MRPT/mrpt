/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>

namespace mrpt::opengl
{
/** A planar (XY) grid where each cell has an associated height and, optionally,
 * a texture map.
 * To make it faster to render, instead of drawing lines and triangles it draws
 * a point at each gridcell.
 *  A typical usage example would be an elevation map or a 3D model of a
 * terrain.
 *
 * ![mrpt::opengl::CMeshFast](preview_CMeshFast.png)
 *
 *  \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */
class CMeshFast : public CRenderizableShaderPoints
{
	DEFINE_SERIALIZABLE(CMeshFast, mrpt::opengl)

   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void onUpdateBuffers_Points() override;
	/** @} */

	/** Constructor
	 */
	CMeshFast(
		bool enableTransparency = false, float xMin_p = -1.0f,
		float xMax_p = 1.0f, float yMin_p = -1.0f, float yMax_p = 1.0f)
		: m_textureImage(0, 0),
		  m_enableTransparency(enableTransparency),
		  X(0, 0),
		  Y(0, 0),
		  Z(0, 0),
		  C(0, 0),
		  C_r(0, 0),
		  C_g(0, 0),
		  C_b(0, 0),
		  xMin(xMin_p),
		  xMax(xMax_p),
		  yMin(yMin_p),
		  yMax(yMax_p)
	{
		m_color.A = 255;
		m_color.R = 0;
		m_color.G = 0;
		m_color.B = 150;
	}
	virtual ~CMeshFast() override = default;

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
	void enableColorFromZ(
		bool v, mrpt::img::TColormap colorMap = mrpt::img::cmJET)
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
	inline float getXMin() const { return xMin; }
	inline float getXMax() const { return xMax; }
	inline float getYMin() const { return yMin; }
	inline float getYMax() const { return yMax; }
	inline void setXMin(float nxm)
	{
		xMin = nxm;
		pointsUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setXMax(float nxm)
	{
		xMax = nxm;
		pointsUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setYMin(float nym)
	{
		yMin = nym;
		pointsUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setYMax(float nym)
	{
		yMax = nym;
		pointsUpToDate = false;
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
	inline void setXBounds(float min, float max)
	{
		xMin = min;
		xMax = max;
		pointsUpToDate = false;
		CRenderizable::notifyChange();
	}
	inline void setYBounds(float min, float max)
	{
		yMin = min;
		yMax = max;
		pointsUpToDate = false;
		CRenderizable::notifyChange();
	}

	mrpt::math::TBoundingBoxf internalBoundingBoxLocal() const override;

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
		const float ycenter = 0.5f * (yMin + yMax);
		const float xwidth = xMax - xMin;
		const float newratio = float(m_textureImage.getWidth()) /
			float(m_textureImage.getHeight());
		yMax = ycenter + 0.5f * newratio * xwidth;
		yMin = ycenter - 0.5f * newratio * xwidth;
		CRenderizable::notifyChange();
	}

   protected:
	mrpt::img::CImage m_textureImage;

	bool m_enableTransparency;
	bool m_colorFromZ{false};
	bool m_isImage{false};

	/** X(x,y): X-coordinate of the point (x,y) */
	mutable math::CMatrixF X;
	/** Y(x,y): Y-coordinate of the point (x,y) */
	mutable math::CMatrixF Y;
	/** Z(x,y): Z-coordinate of the point (x,y) */
	mutable math::CMatrixF Z;

	/** Grayscale or RGB components [0,255] for each cell, updated by
	 * updateColorsMatrix */
	mutable math::CMatrix_u8 C, C_r, C_g, C_b;

	/** Used when m_colorFromZ is true */
	mrpt::img::TColormap m_colorMap{mrpt::img::cmJET};

	/** Whether C is not up-to-date wrt to Z */
	mutable bool m_modified_Z{true};
	/** Whether C is not up-to-date wrt to the texture image */
	mutable bool m_modified_Image{false};

	/** Called internally to assure C is updated. */
	void updateColorsMatrix() const;
	void updatePoints() const;

	/** Mesh bounds */
	float xMin, xMax, yMin, yMax;

	/**Whether the coordinates of the points needs to be recalculated */
	mutable bool pointsUpToDate{false};
};

}  // namespace mrpt::opengl
