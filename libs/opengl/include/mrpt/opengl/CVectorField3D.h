/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CMatrixF.h>
#include <mrpt/opengl/CRenderizableShaderPoints.h>
#include <mrpt/opengl/CRenderizableShaderWireFrame.h>

namespace mrpt::opengl
{
/** A 3D vector field representation, consisting of points and arrows drawn at
 * any spatial position.
 * This opengl object has been created to represent scene flow, and hence
 * both the vector field and
 * the coordinates of the points at which the vector field is represented
 * are stored in matrices because
 * they are computed from intensity and depth images.
 *
 * ![mrpt::opengl::CVectorField3D](preview_CVectorField3D.png)
 *
 * \sa opengl::COpenGLScene
 * \ingroup mrpt_opengl_grp
 */

class CVectorField3D : public CRenderizableShaderPoints,
					   public CRenderizableShaderWireFrame
{
	DEFINE_SERIALIZABLE(CVectorField3D, mrpt::opengl)
   protected:
	/** X component of the vector field */
	mrpt::math::CMatrixF x_vf;
	/** Y component of the vector field */
	mrpt::math::CMatrixF y_vf;
	/** Z component of the vector field */
	mrpt::math::CMatrixF z_vf;

	/** X coordinate of the points at which the vector field is plotted */
	mrpt::math::CMatrixF x_p;
	/** Y coordinate of the points at which the vector field is plotted */
	mrpt::math::CMatrixF y_p;
	/** Z coordinate of the points at which the vector field is plotted */
	mrpt::math::CMatrixF z_p;

	/** By default it is false */
	bool m_colorFromModule{false};
	/** By default it is true */
	bool m_showPoints{true};

	mrpt::img::TColor m_point_color;
	mrpt::img::TColor m_field_color;

	/** Color associated to fields with null module */
	mrpt::img::TColor m_still_color;
	/** Color associated to fields whose module is equal or larger than
	 * 'm_maxspeed' */
	mrpt::img::TColor m_maxspeed_color;
	/** Value of the module of the motion field which will correspond to
	 * 'm_maxspeed_color' */
	float m_maxspeed;

   public:
	/** @name Renderizable shader API virtual methods
	 * @{ */
	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;
	void freeOpenGLResources() override
	{
		CRenderizableShaderWireFrame::freeOpenGLResources();
		CRenderizableShaderPoints::freeOpenGLResources();
	}

	virtual shader_list_t requiredShaders() const override
	{
		return {DefaultShaderID::WIREFRAME, DefaultShaderID::POINTS};
	}
	void onUpdateBuffers_Wireframe() override;
	void onUpdateBuffers_Points() override;
	/** @} */

	/**
	 * Clear the matrices
	 */
	inline void clear()
	{
		x_vf.resize(0, 0);
		y_vf.resize(0, 0);
		z_vf.resize(0, 0);
		x_p.resize(0, 0);
		y_p.resize(0, 0);
		z_p.resize(0, 0);

		CRenderizable::notifyChange();
	}

	/**
	 * Set the point color in the range [0,1]
	 */
	inline void setPointColor(
		const float R, const float G, const float B, const float A = 1)
	{
		m_point_color = mrpt::img::TColor(f2u8(R), f2u8(G), f2u8(B), f2u8(A));
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
		m_field_color = mrpt::img::TColor(f2u8(R), f2u8(G), f2u8(B), f2u8(A));
		CRenderizable::notifyChange();
	}

	/**
	 * Get the motion field min and max colors (colormap) in the range [0,1]
	 */
	inline void getVectorFieldColor(
		mrpt::img::TColorf Cmin, mrpt::img::TColorf Cmax) const
	{
		Cmin = m_still_color / 255;
		Cmax = m_maxspeed_color / 255;
	}

	/**
	 * Set the motion field min and max colors (colormap) in the range [0,1]
	 */
	inline void setMotionFieldColormap(
		const float Rmin, const float Gmin, const float Bmin, const float Rmax,
		const float Gmax, const float Bmax, const float Amin = 1,
		const float Amax = 1)
	{
		m_still_color =
			mrpt::img::TColor(f2u8(Rmin), f2u8(Gmin), f2u8(Bmin), f2u8(Amin));
		m_maxspeed_color =
			mrpt::img::TColor(f2u8(Rmax), f2u8(Gmax), f2u8(Bmax), f2u8(Amax));
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
	 * Set the max speed associated for the color map ( m_still_color,
	 * m_maxspeed_color)
	 */
	inline void setMaxSpeedForColor(const float s)
	{
		m_maxspeed = s;
		CRenderizable::notifyChange();
	}

	/**
	 * Get the max_speed  with which lines are drawn.
	 */
	float getMaxSpeedForColor() const { return m_maxspeed; }
	/**
	 * Get the vector field in three independent matrices: Matrix_x, Matrix_y
	 * and Matrix_z.
	 */
	void getVectorField(
		mrpt::math::CMatrixFloat& Matrix_x, mrpt::math::CMatrixFloat& Matrix_y,
		mrpt::math::CMatrixFloat& Matrix_z) const
	{
		Matrix_x = x_vf;
		Matrix_y = y_vf;
		Matrix_z = z_vf;
	}

	template <class MATRIX>
	void getVectorField(
		MATRIX& Matrix_x, MATRIX& Matrix_y, MATRIX& Matrix_z) const
	{
		Matrix_x = x_vf;
		Matrix_y = y_vf;
		Matrix_z = z_vf;
	}

	/**
	 * Get the coordiantes of the points at which the vector field is
	 * plotted: Coord_x, Coord_y and Coord_z.
	 */
	void getPointCoordinates(
		mrpt::math::CMatrixFloat& Coord_x, mrpt::math::CMatrixFloat& Coord_y,
		mrpt::math::CMatrixFloat& Coord_z) const
	{
		Coord_x = x_p;
		Coord_y = y_p;
		Coord_z = z_p;
	}

	template <class MATRIX>
	void getPointCoordinates(
		MATRIX& Coord_x, MATRIX& Coord_y, MATRIX& Coord_z) const
	{
		Coord_x = x_p;
		Coord_y = y_p;
		Coord_z = z_p;
	}

	/** Get the "x" component of the vector field as a matrix. */
	inline const mrpt::math::CMatrixFloat& getVectorField_x() const
	{
		return x_vf;
	}
	/** \overload */
	inline mrpt::math::CMatrixFloat& getVectorField_x() { return x_vf; }
	/** Get the "y" component of the vector field as a matrix. */
	inline const mrpt::math::CMatrixFloat& getVectorField_y() const
	{
		return y_vf;
	}
	/** \overload */
	inline mrpt::math::CMatrixFloat& getVectorField_y() { return y_vf; }
	/** Get the "z" component of the vector field as a matrix. */
	inline const mrpt::math::CMatrixFloat& getVectorField_z() const
	{
		return z_vf;
	}
	/** \overload */
	inline mrpt::math::CMatrixFloat& getVectorField_z() { return z_vf; }
	/**
	 * Set the vector field with Matrix_x, Matrix_y and Matrix_z.
	 */
	void setVectorField(
		mrpt::math::CMatrixFloat& Matrix_x, mrpt::math::CMatrixFloat& Matrix_y,
		mrpt::math::CMatrixFloat& Matrix_z)
	{
		ASSERT_(
			(Matrix_x.rows() == Matrix_y.rows()) &&
			(Matrix_x.rows() == Matrix_z.rows()));
		ASSERT_(
			(Matrix_x.cols() == Matrix_y.cols()) &&
			(Matrix_x.cols() == Matrix_z.cols()));
		x_vf = Matrix_x;
		y_vf = Matrix_y;
		z_vf = Matrix_z;
		CRenderizable::notifyChange();
	}

	template <class MATRIX>
	void setVectorField(MATRIX& Matrix_x, MATRIX& Matrix_y, MATRIX& Matrix_z)
	{
		ASSERT_(
			(Matrix_x.rows() == Matrix_y.rows()) &&
			(Matrix_x.rows() == Matrix_z.rows()));
		ASSERT_(
			(Matrix_x.cols() == Matrix_y.cols()) &&
			(Matrix_x.cols() == Matrix_z.cols()));
		x_vf = Matrix_x;
		y_vf = Matrix_y;
		z_vf = Matrix_z;
		CRenderizable::notifyChange();
	}

	/**
	 * Set the coordinates of the points at which the vector field is plotted
	 * with Matrix_x, Matrix_y and Matrix_z.
	 */
	void setPointCoordinates(
		mrpt::math::CMatrixFloat& Matrix_x, mrpt::math::CMatrixFloat& Matrix_y,
		mrpt::math::CMatrixFloat& Matrix_z)
	{
		ASSERT_(
			(Matrix_x.rows() == Matrix_y.rows()) &&
			(Matrix_x.rows() == Matrix_z.rows()));
		ASSERT_(
			(Matrix_x.cols() == Matrix_y.cols()) &&
			(Matrix_x.cols() == Matrix_z.cols()));
		x_p = Matrix_x;
		y_p = Matrix_y;
		z_p = Matrix_z;
		CRenderizable::notifyChange();
	}

	template <class MATRIX>
	void setPointCoordinates(
		MATRIX& Matrix_x, MATRIX& Matrix_y, MATRIX& Matrix_z)
	{
		ASSERT_(
			(Matrix_x.rows() == Matrix_y.rows()) &&
			(Matrix_x.rows() == Matrix_z.rows()));
		ASSERT_(
			(Matrix_x.cols() == Matrix_y.cols()) &&
			(Matrix_x.cols() == Matrix_z.cols()));
		x_p = Matrix_x;
		y_p = Matrix_y;
		z_p = Matrix_z;
		CRenderizable::notifyChange();
	}

	/**
	 * Resizes the set.
	 */
	void resize(size_t rows, size_t cols)
	{
		x_vf.resize(rows, cols);
		y_vf.resize(rows, cols);
		z_vf.resize(rows, cols);
		x_p.resize(rows, cols);
		y_p.resize(rows, cols);
		z_p.resize(rows, cols);
		CRenderizable::notifyChange();
	}

	/** Returns the total count of rows used to represent the vector field. */
	inline size_t cols() const { return x_vf.cols(); }
	/** Returns the total count of columns used to represent the vector field.
	 */
	inline size_t rows() const { return x_vf.rows(); }

	mrpt::math::TBoundingBox getBoundingBox() const override;

	void enableColorFromModule(bool enable = true)
	{
		m_colorFromModule = enable;
		CRenderizable::notifyChange();
	}
	void enableShowPoints(bool enable = true)
	{
		m_showPoints = enable;
		CRenderizable::notifyChange();
	}
	bool isAntiAliasingEnabled() const { return m_antiAliasing; }
	bool isColorFromModuleEnabled() const { return m_colorFromModule; }
	/** Constructor */
	CVectorField3D();
	/** Constructor with a initial set of lines. */
	CVectorField3D(
		mrpt::math::CMatrixFloat x_vf_ini, mrpt::math::CMatrixFloat y_vf_ini,
		mrpt::math::CMatrixFloat z_vf_ini, mrpt::math::CMatrixFloat x_p_ini,
		mrpt::math::CMatrixFloat y_p_ini, mrpt::math::CMatrixFloat z_p_ini);
	/** Private, virtual destructor: only can be deleted from smart pointers. */
	~CVectorField3D() override = default;
};

}  // namespace mrpt::opengl
