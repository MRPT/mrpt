/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_CVectorField3D_H
#define opengl_CVectorField3D_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/stl_extensions.h>
#include <Eigen/Dense>

namespace mrpt
{
	namespace opengl
	{
		class OPENGL_IMPEXP CVectorField3D;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CVectorField3D, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A 3D vector field representation, consisting of points and arrows drawn at any spatial position.
			*  This opengl object has been created to represent scene flow, and hence both the vector field and
			*  the coordinates of the points at which the vector field is represented are stored in matrices because
			*  they are computed from intensity and depth images.
			*  \sa opengl::COpenGLScene
			*
			*  <div align="center">
			*  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
			*   <tr> <td> mrpt::opengl::CVectorField2D </td> <td> \image html preview_CVectorField2D.png </td> </tr>
			*  </table>
			*  </div>
			*
			* \ingroup mrpt_opengl_grp
			*/

		class OPENGL_IMPEXP CVectorField3D : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CVectorField3D )
		protected:
			mrpt::math::CMatrix x_vf;				//!< X component of the vector field
			mrpt::math::CMatrix y_vf;				//!< Y component of the vector field
			mrpt::math::CMatrix z_vf;               //!< Z component of the vector field

			mrpt::math::CMatrix x_p;                //!< X coordinate of the points at which the vector field is plotted
			mrpt::math::CMatrix y_p;                //!< Y coordinate of the points at which the vector field is plotted
			mrpt::math::CMatrix z_p;                //!< Z coordinate of the points at which the vector field is plotted

			float	m_LineWidth;			//!< By default it is 1.0
			float	m_pointSize;			//!< By default it is 1.0
			bool    m_antiAliasing;			//!< By default it is true
			bool    m_colorFromModule;      //!< By default it is false
			bool	m_showPoints;			//!< By default it is true

			mrpt::utils::TColor m_point_color;
			mrpt::utils::TColor m_field_color;

			mrpt::utils::TColor m_still_color;           //!< Color associated to fields with null module
			mrpt::utils::TColor m_maxspeed_color;        //!< Color associated to fields whose module is equal or larger than 'm_maxspeed'
			float  m_maxspeed;              //!< Value of the module of the motion field which will correspond to 'm_maxspeed_color'

		public:
			/**
				* Clear the matrices
				*/
			inline void clear()	{
				x_vf.resize(0,0); y_vf.resize(0,0); z_vf.resize(0,0);
				x_p.resize(0,0); y_p.resize(0,0); z_p.resize(0,0);

				CRenderizableDisplayList::notifyChange();
			}

			/**
				* Set the point color in the range [0,1]
				*/
			inline void setPointColor( const float R, const float G, const float B, const float A = 1)
			{
				m_point_color = mrpt::utils::TColor(R*255,G*255,B*255,A*255);
				CRenderizableDisplayList::notifyChange();
			}

			/**
				* Get the point color in the range [0,1]
				*/
			inline mrpt::utils::TColorf getPointColor() const { return mrpt::utils::TColorf(m_point_color); }

			/**
				* Set the arrow color in the range [0,1]
				*/
			inline void setVectorFieldColor( const float R, const float G, const float B, const float A = 1)
			{
				m_field_color = mrpt::utils::TColor(R*255,G*255,B*255,A*255);
				CRenderizableDisplayList::notifyChange();
			}

			/**
				* Get the motion field min and max colors (colormap) in the range [0,1]
				*/
			inline void getVectorFieldColor(mrpt::utils::TColorf Cmin, mrpt::utils::TColorf Cmax) const
			{
				Cmin = m_still_color/255;
				Cmax = m_maxspeed_color/255;
			}

			/**
				* Set the motion field min and max colors (colormap) in the range [0,1]
				*/
			inline void setMotionFieldColormap( const float Rmin, const float Gmin, const float Bmin, const float Rmax, const float Gmax, const float Bmax, const float Amin = 1, const float Amax = 1)
			{
				m_still_color = mrpt::utils::TColor(Rmin*255,Gmin*255,Bmin*255,Amin*255);
				m_maxspeed_color = mrpt::utils::TColor(Rmax*255,Gmax*255,Bmax*255,Amax*255);
				CRenderizableDisplayList::notifyChange();
			}

			/**
				* Get the arrow color in the range [0,1]
				*/
			inline mrpt::utils::TColorf getVectorFieldColor() const	{ return mrpt::utils::TColorf(m_field_color); }

			/**
				* Set the size with which points will be drawn. By default 1.0
				*/
			inline void setPointSize(const float p) { m_pointSize=p; CRenderizableDisplayList::notifyChange(); }

			/**
				* Get the size with which points are drawn. By default 1.0
				*/
			inline float getPointSize() const { return m_pointSize; }

			/**
				* Set the width with which lines will be drawn.
				*/
			inline void setLineWidth(const float w) { m_LineWidth = w; CRenderizableDisplayList::notifyChange(); }

			/**
				* Get the width with which lines are drawn.
				*/
			float getLineWidth() const {return m_LineWidth;	}

			/**
				* Set the max speed associated for the color map ( m_still_color, m_maxspeed_color)
				*/
			inline void setMaxSpeedForColor (const float s) { m_maxspeed = s; CRenderizableDisplayList::notifyChange(); }

			/**
				* Get the max_speed  with which lines are drawn.
				*/
			float getMaxSpeedForColor() const {return m_maxspeed;	}

			/**
				* Get the vector field in three independent matrices: Matrix_x, Matrix_y and Matrix_z.
				*/
			void getVectorField(mrpt::math::CMatrixFloat &Matrix_x, mrpt::math::CMatrixFloat &Matrix_y, mrpt::math::CMatrixFloat &Matrix_z) const {
				Matrix_x = x_vf;
				Matrix_y = y_vf;
				Matrix_z = z_vf;
			}

			void getVectorField(Eigen::MatrixXf &Matrix_x, Eigen::MatrixXf &Matrix_y, Eigen::MatrixXf &Matrix_z) const {
				Matrix_x = x_vf;
				Matrix_y = y_vf;
				Matrix_z = z_vf;
			}

			/**
				* Get the coordiantes of the points at which the vector field is plotted: Coord_x, Coord_y and Coord_z.
				*/
			void getPointCoordinates(mrpt::math::CMatrixFloat &Coord_x, mrpt::math::CMatrixFloat &Coord_y, mrpt::math::CMatrixFloat &Coord_z) const {
				Coord_x = x_p;
				Coord_y = y_p;
				Coord_z = z_p;
			}

			void getPointCoordinates(Eigen::MatrixXf &Coord_x, Eigen::MatrixXf &Coord_y, Eigen::MatrixXf &Coord_z) const {
				Coord_x = x_p;
				Coord_y = y_p;
				Coord_z = z_p;
			}

			/** Get the "x" component of the vector field as a matrix. */
			inline const mrpt::math::CMatrixFloat & getVectorField_x() const { return x_vf; }
			/** \overload */
			inline mrpt::math::CMatrixFloat & getVectorField_x() { return x_vf; }

			/** Get the "y" component of the vector field as a matrix. */
			inline const mrpt::math::CMatrixFloat & getVectorField_y() const { return y_vf; }
			/** \overload */
			inline mrpt::math::CMatrixFloat & getVectorField_y() { return y_vf; }

			/** Get the "z" component of the vector field as a matrix. */
			inline const mrpt::math::CMatrixFloat & getVectorField_z() const { return z_vf; }
			/** \overload */
			inline mrpt::math::CMatrixFloat & getVectorField_z() { return z_vf; }

			/**
				* Set the vector field with Matrix_x, Matrix_y and Matrix_z.
				*/
			void setVectorField(mrpt::math::CMatrixFloat &Matrix_x, mrpt::math::CMatrixFloat &Matrix_y, mrpt::math::CMatrixFloat &Matrix_z) {
				ASSERT_((Matrix_x.getRowCount() == Matrix_y.getRowCount())&&(Matrix_x.getRowCount() == Matrix_z.getRowCount()))
				ASSERT_((Matrix_x.getColCount() == Matrix_y.getColCount())&&(Matrix_x.getColCount() == Matrix_z.getColCount()))
				x_vf = Matrix_x;
				y_vf = Matrix_y;
				z_vf = Matrix_z;
				CRenderizableDisplayList::notifyChange();
			}

			void setVectorField(Eigen::MatrixXf &Matrix_x,Eigen::MatrixXf &Matrix_y, Eigen::MatrixXf &Matrix_z) {
				ASSERT_((Matrix_x.getRowCount() == Matrix_y.getRowCount())&&(Matrix_x.getRowCount() == Matrix_z.getRowCount()))
				ASSERT_((Matrix_x.getColCount() == Matrix_y.getColCount())&&(Matrix_x.getColCount() == Matrix_z.getColCount()))
				x_vf = Matrix_x;
				y_vf = Matrix_y;
				z_vf = Matrix_z;
				CRenderizableDisplayList::notifyChange();
			}

			/**
				* Set the coordinates of the points at which the vector field is plotted with Matrix_x, Matrix_y and Matrix_z.
				*/
			void setPointCoordinates(mrpt::math::CMatrixFloat &Matrix_x, mrpt::math::CMatrixFloat &Matrix_y, mrpt::math::CMatrixFloat &Matrix_z) {
				ASSERT_((Matrix_x.getRowCount() == Matrix_y.getRowCount())&&(Matrix_x.getRowCount() == Matrix_z.getRowCount()))
				ASSERT_((Matrix_x.getColCount() == Matrix_y.getColCount())&&(Matrix_x.getColCount() == Matrix_z.getColCount()))
				x_p = Matrix_x;
				y_p = Matrix_y;
				z_p = Matrix_z;
				CRenderizableDisplayList::notifyChange();
			}

			void setPointCoordinates(Eigen::MatrixXf &Matrix_x, Eigen::MatrixXf &Matrix_y, Eigen::MatrixXf &Matrix_z) {
				ASSERT_((Matrix_x.getRowCount() == Matrix_y.getRowCount())&&(Matrix_x.getRowCount() == Matrix_z.getRowCount()))
				ASSERT_((Matrix_x.getColCount() == Matrix_y.getColCount())&&(Matrix_x.getColCount() == Matrix_z.getColCount()))
				x_p = Matrix_x;
				y_p = Matrix_y;
				z_p = Matrix_z;
				CRenderizableDisplayList::notifyChange();
			}


			/**
				* Resizes the set.
				*/
			void resize(size_t rows, size_t cols)	{
				x_vf.resize(rows,cols); y_vf.resize(rows,cols); z_vf.resize(rows,cols);
				x_p.resize(rows,cols); y_p.resize(rows,cols); z_p.resize(rows,cols);
				CRenderizableDisplayList::notifyChange();
			}

			/** Returns the total count of rows used to represent the vector field. */
			inline size_t getColCount() const { return x_vf.getColCount(); }

			/** Returns the total count of columns used to represent the vector field. */
			inline size_t getRowCount() const { return x_vf.getRowCount(); }

			/**
				* Class factory
				*/
			static CVectorField3DPtr Create(const mrpt::math::CMatrixFloat x_vf_ini, const mrpt::math::CMatrixFloat y_vf_ini, const mrpt::math::CMatrixFloat z_vf_ini, const mrpt::math::CMatrixFloat x_p_ini, const mrpt::math::CMatrixFloat y_p_ini, const mrpt::math::CMatrixFloat z_p_ini);
			/** Render
				*/
			void  render_dl() const MRPT_OVERRIDE;


			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

			void enableAntiAliasing(bool enable=true) { m_antiAliasing = enable; CRenderizableDisplayList::notifyChange(); }
			void enableColorFromModule(bool enable=true) { m_colorFromModule = enable; CRenderizableDisplayList::notifyChange(); }
			void enableShowPoints(bool enable=true) { m_showPoints = enable; CRenderizableDisplayList::notifyChange(); }
			bool isAntiAliasingEnabled() const { return m_antiAliasing; }
			bool isColorFromModuleEnabled() const { return m_colorFromModule; }

		private:
			/** Constructor */
			CVectorField3D();
			/** Constructor with a initial set of lines. */
			CVectorField3D( mrpt::math::CMatrixFloat x_vf_ini, mrpt::math::CMatrixFloat y_vf_ini, mrpt::math::CMatrixFloat z_vf_ini, mrpt::math::CMatrixFloat x_p_ini, mrpt::math::CMatrixFloat y_p_ini, mrpt::math::CMatrixFloat z_p_ini);
			/** Private, virtual destructor: only can be deleted from smart pointers. */
			virtual ~CVectorField3D() { }
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CVectorField3D, CRenderizableDisplayList, OPENGL_IMPEXP )


	} // end namespace

} // End of namespace


#endif
