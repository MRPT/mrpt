/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_CVectorField2D_H
#define opengl_CVectorField2D_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/CMatrix.h>

namespace mrpt
{
	namespace opengl
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CVectorField2D, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A 2D vector field representation, consisting of points and arrows drawn on a plane (invisible grid).
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

		class OPENGL_IMPEXP CVectorField2D : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CVectorField2D )
		protected:
			mrpt::math::CMatrix xcomp;				//!< X component of the vector field
			mrpt::math::CMatrix ycomp;				//!< Y component of the vector field

			float	xMin,xMax,yMin,yMax;	//!< Grid bounds
			float	m_LineWidth;			//!< By default is 1.0
			float	m_pointSize;			//!< By default is 1.0
			bool    m_antiAliasing;			//!< By default is true

			mrpt::utils::TColor m_point_color;
			mrpt::utils::TColor m_field_color;

		public:
			/**
			  * Clear the matrices
			  */
			inline void clear()	{
				xcomp.resize(0,0);
				ycomp.resize(0,0);
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
			float getLineWidth() const {
				return m_LineWidth;
			}

			/**
			  * Set the coordinates of the grid on where the vector field will be drawn by setting its center and the cell size.
			  * The number of cells is marked by the content of xcomp and ycomp.
			  * \sa xcomp, ycomp
			  */
			void setGridCenterAndCellSize(const float center_x, const float center_y, const float cellsize_x, const float cellsize_y)
			{
				xMin = center_x - 0.5*cellsize_x*(xcomp.getColCount()-1);
				xMax = center_x + 0.5*cellsize_x*(xcomp.getColCount()-1);
				yMin = center_y - 0.5*cellsize_y*(xcomp.getRowCount()-1);
				yMax = center_y + 0.5*cellsize_y*(xcomp.getRowCount()-1);
				CRenderizableDisplayList::notifyChange();
			}

			/**
			  * Set the coordinates of the grid on where the vector field will be drawn using x-y max and min values.
			  */
			void setGridLimits(const float xmin,const float xmax, const float ymin, const float ymax)
			{
				xMin=xmin; xMax = xmax;
				yMin=ymin; yMax = ymax;
				CRenderizableDisplayList::notifyChange();
			}

			/**
			  * Get the coordinates of the grid on where the vector field is drawn using the max and min values.
			  */
			void getGridLimits(float &xmin,float &xmax, float &ymin, float &ymax) const
			{
				xmin=xMin; xmax=xMax;
				ymin=yMin; ymax=yMax;
			}

			/**
			  * Get the vector field. Matrix_x stores the "x" component and Matrix_y stores the "y" component.
			  */
			void getVectorField(mrpt::math::CMatrixFloat &Matrix_x, mrpt::math::CMatrixFloat &Matrix_y) const {
				Matrix_x = xcomp;
				Matrix_y = ycomp;
			}

			/** Get the "x" component of the vector field, as a matrix where each entry represents a point in the 2D grid. */
			inline const mrpt::math::CMatrixFloat & getVectorField_x() const { return xcomp; }
			/** \overload */
			inline mrpt::math::CMatrixFloat & getVectorField_x() { return xcomp; }

			/** Get the "y" component of the vector field, as a matrix where each entry represents a point in the 2D grid. */
			inline const mrpt::math::CMatrixFloat & getVectorField_y() const { return ycomp; }
			/** \overload */
			inline mrpt::math::CMatrixFloat & getVectorField_y() { return ycomp; }

			/**
			  * Set the vector field. Matrix_x contains the "x" component and Matrix_y contains the "y" component.
			  */
			void setVectorField(mrpt::math::CMatrixFloat &Matrix_x, mrpt::math::CMatrixFloat &Matrix_y) {
				ASSERT_((Matrix_x.getRowCount() == Matrix_y.getRowCount())&&(Matrix_x.getColCount() == Matrix_y.getColCount()))
				xcomp = Matrix_x;
				ycomp = Matrix_y;
				CRenderizableDisplayList::notifyChange();
			}

			/**
			  * Adjust the vector field in the scene (vectors magnitude) according to the grid size.
			  */
			void adjustVectorFieldToGrid();

			/** Resizes the set.
			  */
			void resize(size_t rows, size_t cols)	{
				xcomp.resize(rows, cols);
				ycomp.resize(rows, cols);
				CRenderizableDisplayList::notifyChange();
			}

			/** Returns the total count of rows used to represent the vector field. */
			inline size_t getColCount() const { return xcomp.getColCount(); }

			/** Returns the total count of columns used to represent the vector field. */
			inline size_t getRowCount() const { return xcomp.getRowCount(); }

			/**
			  * Class factory
			  */
			static CVectorField2DPtr Create(const mrpt::math::CMatrixFloat &Matrix_x, const mrpt::math::CMatrixFloat &Matrix_y, float	xmin=-1, float xmax=1, float ymin=-1, float ymax=1);
			/** Render
			  */
			void  render_dl() const MRPT_OVERRIDE;


			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

			void enableAntiAliasing(bool enable=true) { m_antiAliasing = enable; CRenderizableDisplayList::notifyChange(); }
			bool isAntiAliasingEnabled() const { return m_antiAliasing; }

		private:
			/** Constructor */
			CVectorField2D();
			/** Constructor with a initial set of lines. */
			CVectorField2D( mrpt::math::CMatrixFloat Matrix_x, mrpt::math::CMatrixFloat Matrix_y, float	xmin=-1, float xmax=1, float ymin=-1, float ymax=1);
			/** Private, virtual destructor: only can be deleted from smart pointers. */
			virtual ~CVectorField2D() { }
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CVectorField2D, CRenderizableDisplayList, OPENGL_IMPEXP )


	} // end namespace

} // End of namespace


#endif
