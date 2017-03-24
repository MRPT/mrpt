/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CEllipsoid_H
#define opengl_CEllipsoid_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt
{
	namespace opengl
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CEllipsoid, CRenderizableDisplayList, OPENGL_IMPEXP )

		/** A 2D ellipse or 3D ellipsoid, depending on the size of the m_cov matrix (2x2 or 3x3).
		  *  The center of the ellipsoid is the "m_x,m_y,m_z" object's coordinates. In the case of
		  *   a 2D ellipse it will be drawn in the XY plane, for z=0.
		  *  The color is determined by the RGBA fields in the class "CRenderizable". Note that a
		  *   transparent ellipsoid can be drawn for "0<alpha<1" values.
		  *	 If one of the eigen value of the covariance matrix of the ellipsoid is null, ellipsoid will not be rendered.
		  *  \sa opengl::COpenGLScene
		  *
		  *
		  * Please read the documentation of CGeneralizedEllipsoidTemplate::setQuantiles() for learning
		  *  the mathematical details about setting the desired confidence interval.
		  *
		  *  <div align="center">
		  *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
		  *   <tr> <td> mrpt::opengl::CEllipsoid </td> <td> \image html preview_CEllipsoid.png </td> </tr>
		  *  </table>
		  *  </div>
		  *
		  * \ingroup mrpt_opengl_grp
		  */
		class OPENGL_IMPEXP CEllipsoid : public CRenderizableDisplayList
		{
			DEFINE_SERIALIZABLE( CEllipsoid )

		protected:
			/** Used to store computed values the first time this is rendered, and to avoid recomputing them again.
			 */
			math::CMatrixD		m_eigVal,m_eigVec,m_prevComputedCov;

			math::CMatrixD	m_cov;		//!< The 2x2 or 3x3 covariance matrix that will determine the aspect of the ellipsoid.
			bool			m_drawSolid3D;	//!< If set to true (default), a whole ellipsoid surface will be drawn, or if set to "false" it will be drawn as a "wireframe".
			float			m_quantiles;	//!< The number of "sigmas" for drawing the ellipse/ellipsoid (default=3)
			unsigned int	m_2D_segments;	//!< The number of segments of a 2D ellipse (default=20)
			unsigned int	m_3D_segments;	//!< The number of segments of a 3D ellipse (in both "axis") (default=20)
			float			m_lineWidth;	//!< The line width for 2D ellipses or 3D wireframe ellipsoids (default=1)
			mutable mrpt::math::TPoint3D m_bb_min, m_bb_max;

		public:
			void setCovMatrix( const mrpt::math::CMatrixDouble &m, int resizeToSize = -1 ); //!< Set the 2x2 or 3x3 covariance matrix that will determine the aspect of the ellipsoid (if resizeToSize>0, the matrix will be cut to the square matrix of the given size)
			void setCovMatrix( const mrpt::math::CMatrixFloat &m, int resizeToSize = -1 ); //!< Set the 2x2 or 3x3 covariance matrix that will determine the aspect of the ellipsoid (if resizeToSize>0, the matrix will be cut to the square matrix of the given size).

			/**  Set the 2x2 or 3x3 covariance matrix that will determine the aspect of the ellipsoid (if resizeToSize>0, the matrix will be cut to the square matrix of the given size)
			 */
			template <typename T>
			void setCovMatrix( const mrpt::math::CMatrixFixedNumeric<T,3,3> &m, int resizeToSize = -1 )	{
				setCovMatrix(mrpt::math::CMatrixTemplateNumeric<T>(m),resizeToSize);
			}

			/**  Set the 2x2 or 3x3 covariance matrix that will determine the aspect of the ellipsoid (if resizeToSize>0, the matrix will be cut to the square matrix of the given size)
			 */
			template <typename T>
			void setCovMatrix( const mrpt::math::CMatrixFixedNumeric<T,2,2> &m )	{
				setCovMatrix(mrpt::math::CMatrixTemplateNumeric<T>(m));
			}

			mrpt::math::CMatrixDouble getCovMatrix() const { return mrpt::math::CMatrixDouble(m_cov); }

			void enableDrawSolid3D(bool v) { m_drawSolid3D = v; CRenderizableDisplayList::notifyChange(); } //!< If set to true (default), a whole ellipsoid surface will be drawn, or if set to "false" it will be drawn as a "wireframe".
			void setQuantiles(float q) { m_quantiles=q; CRenderizableDisplayList::notifyChange(); } //!< The number of "sigmas" for drawing the ellipse/ellipsoid (default=3)
			float getQuantiles() const { return m_quantiles; }

			void set2DsegmentsCount(unsigned int N) { m_2D_segments=N; CRenderizableDisplayList::notifyChange(); }  //!< The number of segments of a 2D ellipse (default=20)
			void set3DsegmentsCount(unsigned int N) { m_3D_segments=N; CRenderizableDisplayList::notifyChange(); } //!< The number of segments of a 3D ellipse (in both "axis") (default=20)

			void setLineWidth(float w) { m_lineWidth=w; CRenderizableDisplayList::notifyChange(); } //!< The line width for 2D ellipses or 3D wireframe ellipsoids (default=1)
			float getLineWidth() const { return m_lineWidth; }


			/** Render
			  *	If one of the eigen value of the covariance matrix of the ellipsoid is null, ellipsoid will not
			  * be rendered to ensure stability in the rendering process.
			  */
			void  render_dl() const MRPT_OVERRIDE;

			/** Evaluates the bounding box of this object (including possible children) in the coordinate frame of the object parent. */
			void getBoundingBox(mrpt::math::TPoint3D &bb_min, mrpt::math::TPoint3D &bb_max) const MRPT_OVERRIDE;

			/** Ray tracing
			  */
			bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const MRPT_OVERRIDE;

		private:
			/** Constructor
			  */
			CEllipsoid() : m_eigVal(),m_eigVec(),m_prevComputedCov(),
				m_cov(2,2),
				m_drawSolid3D(true),
				m_quantiles(3),
				m_2D_segments(20),
				m_3D_segments(20),
				m_lineWidth(1.0),
				m_bb_min(0,0,0), 
				m_bb_max(0,0,0)
			{
			}
			/** Private, virtual destructor: only can be deleted from smart pointers */
			virtual ~CEllipsoid() { }
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CEllipsoid, CRenderizableDisplayList, OPENGL_IMPEXP )

	} // end namespace

} // End of namespace


#endif
