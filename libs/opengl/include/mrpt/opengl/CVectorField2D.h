/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#ifndef opengl_CVectorField2D_H
#define opengl_CVectorField2D_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/stl_extensions.h>

namespace mrpt
{
	namespace opengl
	{
		using mrpt::math::TPoint3D;
		using mrpt::math::TSegment3D;
		using mrpt::math::CMatrixFloat;
		using mrpt::utils::TColor;
		class OPENGL_IMPEXP CVectorField2D;

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
			CMatrix xcomp;				//!< X component of the vector field
			CMatrix ycomp;				//!< Y component of the vector field

			float	xMin,xMax,yMin,yMax;	//!< Grid bounds
            float	m_LineWidth;			//!< By default is 1.0
			float	m_pointSize;			//!< By default is 1.0
			bool    m_antiAliasing;			//!< By default is true

			TColor m_point_color;
			TColor m_field_color;

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
				m_point_color = TColor(R*255,G*255,B*255,A*255);
				CRenderizableDisplayList::notifyChange();
			}

			/**
			  * Get the point color in the range [0,1]
			  */			
			inline TColorf getPointColor() const { return mrpt::utils::TColorf(m_point_color); }

			/**
			  * Set the arrow color in the range [0,1]
			  */
			inline void setVectorFieldColor( const float R, const float G, const float B, const float A = 1)
			{
				m_field_color = TColor(R*255,G*255,B*255,A*255);
				CRenderizableDisplayList::notifyChange();
			}

			/**
			  * Get the arrow color in the range [0,1]
			  */			
			inline TColorf getVectorFieldColor() const	{ return mrpt::utils::TColorf(m_field_color); }

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
			void getVectorField(CMatrixFloat &Matrix_x, CMatrixFloat &Matrix_y) const {
				Matrix_x = xcomp;
				Matrix_y = ycomp;
			}

			/**
			  * Set the vector field. Matrix_x contains the "x" component and Matrix_y contains the "y" component.
			  */
			void setVectorField(CMatrixFloat &Matrix_x, CMatrixFloat &Matrix_y) {
				ASSERT_((Matrix_x.getRowCount() == Matrix_y.getRowCount())&&(Matrix_x.getColCount() == Matrix_y.getColCount()))
				xcomp = Matrix_x;
				ycomp = Matrix_y;
				CRenderizableDisplayList::notifyChange();
			}

			/**
			  * Adjust the vector field in the scene (vectors magnitude) according to the grid size.
			  */			
			void adjustVectorFieldToGrid();

			/**
			  * Resizes the set.
			  * \sa reserve
			  */
			void resize(size_t rows, size_t cols)	{
				xcomp.resize(rows, cols);
				ycomp.resize(rows, cols);
				CRenderizableDisplayList::notifyChange();
			}
			/**
			  * Reserves an amount of lines to the set. This method should be used when some known amount of lines is going to be inserted, so that only a memory allocation is needed.
			  * \sa resize
			  */
			void reserve(size_t rows, size_t cols)	{
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
			inline static CVectorField2DPtr Create(const CMatrixFloat Matrix_x, const CMatrixFloat Matrix_y, float	xmin=-1, float xmax=1, float ymin=-1, float ymax=1)	{
				return CVectorField2DPtr(new CVectorField2D( Matrix_x,  Matrix_y, xmin, xmax, ymin, ymax));
			}
			/** Render
			  */
			void  render_dl() const;


			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			virtual void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const;

			void enableAntiAliasing(bool enable=true) { m_antiAliasing = enable; CRenderizableDisplayList::notifyChange(); }
			bool isAntiAliasingEnabled() const { return m_antiAliasing; }

		private:
			/** Constructor */
			CVectorField2D();
			/** Constructor with a initial set of lines. */
			CVectorField2D( CMatrixFloat Matrix_x, CMatrixFloat Matrix_y, float	xmin=-1, float xmax=1, float ymin=-1, float ymax=1);
			/** Private, virtual destructor: only can be deleted from smart pointers. */
			virtual ~CVectorField2D() { }
		};


	} // end namespace

} // End of namespace


#endif
