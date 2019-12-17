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

namespace mrpt::opengl
{
/** A planar (XY) grid where each cell has an associated height and, optionally,
 * a texture map.
 * To make it faster to render, instead of drawing lines and triangles it draws
 * a point at each
 * gridcell.
 *  A typical usage example would be an elevation map or a 3D model of a
 * terrain.
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CMeshFast </td> <td> \image html
 * preview_CMeshFast.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CMeshFast : public CRenderizable
{
	DEFINE_SERIALIZABLE(CMeshFast, mrpt::opengl)

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
	mrpt::img::TColormap m_colorMap{mrpt::img::cmJET};
	/** By default is 1.0 */
	float m_pointSize;
	/** Default: false */
	bool m_pointSmooth;

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

   public:
	/** By default is 1.0 */
	inline void setPointSize(float p) { m_pointSize = p; }
	inline float getPointSize() const { return m_pointSize; }
	inline void enablePointSmooth(bool enable = true)
	{
		m_pointSmooth = enable;
	}
	inline void disablePointSmooth() { m_pointSmooth = false; }
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
	/** Private, virtual destructor: only can be deleted from smart pointers */
	~CMeshFast() override = default;
};

}  // namespace mrpt::opengl
