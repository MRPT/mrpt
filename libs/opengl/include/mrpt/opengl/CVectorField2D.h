/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CMatrixF.h>
#include <mrpt/opengl/CRenderizable.h>

namespace mrpt::opengl
{
/** A 2D vector field representation, consisting of points and arrows drawn on a
 * plane (invisible grid).
 *  \sa opengl::COpenGLScene
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CVectorField2D </td> <td> \image html
 * preview_CVectorField2D.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */

class CVectorField2D : public CRenderizable
{
	DEFINE_SERIALIZABLE(CVectorField2D, mrpt::opengl)
   protected:
	/** X component of the vector field */
	mrpt::math::CMatrixF xcomp;
	/** Y component of the vector field */
	mrpt::math::CMatrixF ycomp;

	/** Grid bounds */
	float xMin{-1.0}, xMax{1.0}, yMin{-1.0}, yMax{1.0};
	/** By default is 1.0 */
	float m_LineWidth{1.0};
	/** By default is 1.0 */
	float m_pointSize{1.0};
	/** By default is true */
	bool m_antiAliasing{true};

	mrpt::img::TColor m_point_color;
	mrpt::img::TColor m_field_color;

   public:
	/**
	 * Clear the matrices
	 */
	inline void clear()
	{
		xcomp.resize(0, 0);
		ycomp.resize(0, 0);
		CRenderizable::notifyChange();
	}

	/**
	 * Set the point color in the range [0,1]
	 */
	inline void setPointColor(
		const float R, const float G, const float B, const float A = 1)
	{
		m_point_color = mrpt::img::TColor(R * 255, G * 255, B * 255, A * 255);
		CRenderizable::notifyChange();
	}

	/**
	 * Get the point color in the range [0,1]
	 */
	inline mrpt::img::TColorf getPointColor() const
	{
		return mrpt::img::TColorf(m_point_color);
	}

	/**
	 * Set the arrow color in the range [0,1]
	 */
	inline void setVectorFieldColor(
		const float R, const float G, const float B, const float A = 1)
	{
		m_field_color = mrpt::img::TColor(R * 255, G * 255, B * 255, A * 255);
		CRenderizable::notifyChange();
	}

	/**
	 * Get the arrow color in the range [0,1]
	 */
	inline mrpt::img::TColorf getVectorFieldColor() const
	{
		return mrpt::img::TColorf(m_field_color);
	}

	/**
	 * Set the size with which points will be drawn. By default 1.0
	 */
	inline void setPointSize(const float p)
	{
		m_pointSize = p;
		CRenderizable::notifyChange();
	}

	/**
	 * Get the size with which points are drawn. By default 1.0
	 */
	inline float getPointSize() const { return m_pointSize; }
	/**
	 * Set the width with which lines will be drawn.
	 */
	inline void setLineWidth(const float w)
	{
		m_LineWidth = w;
		CRenderizable::notifyChange();
	}

	/**
	 * Get the width with which lines are drawn.
	 */
	float getLineWidth() const { return m_LineWidth; }
	/**
	 * Set the coordinates of the grid on where the vector field will be drawn
	 * by setting its center and the cell size.
	 * The number of cells is marked by the content of xcomp and ycomp.
	 * \sa xcomp, ycomp
	 */
	void setGridCenterAndCellSize(
		const float center_x, const float center_y, const float cellsize_x,
		const float cellsize_y)
	{
		xMin = center_x - 0.5 * cellsize_x * (xcomp.cols() - 1);
		xMax = center_x + 0.5 * cellsize_x * (xcomp.cols() - 1);
		yMin = center_y - 0.5 * cellsize_y * (xcomp.rows() - 1);
		yMax = center_y + 0.5 * cellsize_y * (xcomp.rows() - 1);
		CRenderizable::notifyChange();
	}

	/**
	 * Set the coordinates of the grid on where the vector field will be drawn
	 * using x-y max and min values.
	 */
	void setGridLimits(
		const float xmin, const float xmax, const float ymin, const float ymax)
	{
		xMin = xmin;
		xMax = xmax;
		yMin = ymin;
		yMax = ymax;
		CRenderizable::notifyChange();
	}

	/**
	 * Get the coordinates of the grid on where the vector field is drawn using
	 * the max and min values.
	 */
	void getGridLimits(float& xmin, float& xmax, float& ymin, float& ymax) const
	{
		xmin = xMin;
		xmax = xMax;
		ymin = yMin;
		ymax = yMax;
	}

	/**
	 * Get the vector field. Matrix_x stores the "x" component and Matrix_y
	 * stores the "y" component.
	 */
	void getVectorField(
		mrpt::math::CMatrixFloat& Matrix_x,
		mrpt::math::CMatrixFloat& Matrix_y) const
	{
		Matrix_x = xcomp;
		Matrix_y = ycomp;
	}

	/** Get the "x" component of the vector field, as a matrix where each entry
	 * represents a point in the 2D grid. */
	inline const mrpt::math::CMatrixFloat& getVectorField_x() const
	{
		return xcomp;
	}
	/** \overload */
	inline mrpt::math::CMatrixFloat& getVectorField_x() { return xcomp; }
	/** Get the "y" component of the vector field, as a matrix where each entry
	 * represents a point in the 2D grid. */
	inline const mrpt::math::CMatrixFloat& getVectorField_y() const
	{
		return ycomp;
	}
	/** \overload */
	inline mrpt::math::CMatrixFloat& getVectorField_y() { return ycomp; }
	/**
	 * Set the vector field. Matrix_x contains the "x" component and Matrix_y
	 * contains the "y" component.
	 */
	void setVectorField(
		mrpt::math::CMatrixFloat& Matrix_x, mrpt::math::CMatrixFloat& Matrix_y)
	{
		ASSERT_(
			(Matrix_x.rows() == Matrix_y.rows()) &&
			(Matrix_x.cols() == Matrix_y.cols()));
		xcomp = Matrix_x;
		ycomp = Matrix_y;
		CRenderizable::notifyChange();
	}

	/**
	 * Adjust the vector field in the scene (vectors magnitude) according to
	 * the grid size.
	 */
	void adjustVectorFieldToGrid();

	/** Resizes the set.
	 */
	void resize(size_t rows, size_t cols)
	{
		xcomp.resize(rows, cols);
		ycomp.resize(rows, cols);
		CRenderizable::notifyChange();
	}

	/** Returns the total count of rows used to represent the vector field. */
	inline size_t cols() const { return xcomp.cols(); }
	/** Returns the total count of columns used to represent the vector field.
	 */
	inline size_t rows() const { return xcomp.rows(); }

	void render() const override;
	void renderUpdateBuffers() const override;
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

	void enableAntiAliasing(bool enable = true)
	{
		m_antiAliasing = enable;
		CRenderizable::notifyChange();
	}
	bool isAntiAliasingEnabled() const { return m_antiAliasing; }
	/** Constructor */
	CVectorField2D();
	/** Constructor with a initial set of lines. */
	CVectorField2D(
		mrpt::math::CMatrixFloat Matrix_x, mrpt::math::CMatrixFloat Matrix_y,
		float xmin = -1, float xmax = 1, float ymin = -1, float ymax = 1);
	/** Private, virtual destructor: only can be deleted from smart pointers. */
	~CVectorField2D() override = default;
};

}  // namespace mrpt::opengl
