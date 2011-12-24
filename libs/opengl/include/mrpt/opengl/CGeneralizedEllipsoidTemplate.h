/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef opengl_CGeneralizedEllipsoidTemplate_H
#define opengl_CGeneralizedEllipsoidTemplate_H

#include <mrpt/opengl/CRenderizableDisplayList.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt
{
	namespace opengl
	{
		namespace detail 
		{
			template <int DIM>
			void OPENGL_IMPEXP renderGeneralizedEllipsoidTemplate(
				const std::vector<mrpt::math::CArray<float,DIM> > & pts,
				const float    lineWidth,
				const uint32_t slices, 
				const uint32_t stacks);
			template <int DIM>
			void OPENGL_IMPEXP generalizedEllipsoidPoints(
				const mrpt::math::CMatrixFixedNumeric<double,DIM,DIM> & U,
				const mrpt::math::CMatrixFixedNumeric<double,DIM,1>   & mean,
				std::vector<mrpt::math::CArray<float,DIM> >   &out_params_pts,
				const uint32_t slices, 
				const uint32_t stacks);
		}

		/** A class that generalizes the concept of an ellipsoid to arbitrary parameterizations of 
		  *  uncertainty shapes in either 2D or 3D. See derived classes for examples. 
		  *
		  *  The main method to set the modeled uncertainty is \a setCovMatrixAndMean()
		  *
		  * \tparam DIM The dimensionality of the parameter space, which must coincide with that of the rendering space (2 or 3)
		  *
		  * \ingroup mrpt_opengl_grp
		  */
		template <int DIM>
		class CGeneralizedEllipsoidTemplate : public CRenderizableDisplayList
		{
		public:
			typedef mrpt::math::CMatrixFixedNumeric<double,DIM,DIM> cov_matrix_t;   //!< The type of fixed-size covariance matrices for this representation
			typedef mrpt::math::CMatrixFixedNumeric<double,DIM,1> mean_vector_t;   //!< The type of fixed-size vector for this representation
				
			typedef mrpt::math::CArray<float,DIM> array_parameter_t;
			typedef mrpt::math::CArray<float,DIM>     array_point_t;

			/**  Set the NxN covariance matrix that will determine the aspect of the ellipsoid - Notice that the 
			  *  covariance determines the uncertainty in the parameter space, which would be transformed by derived function
			  */
			template <typename MATRIX, typename VECTOR>
			void setCovMatrixAndMean( const MATRIX &new_cov, const VECTOR &new_mean )
			{
				MRPT_START
				ASSERT_( new_cov.getColCount() == new_cov.getRowCount() && new_cov.getColCount() == DIM )
				m_cov = new_cov;
				m_mean = new_mean;
				m_needToRecomputeEigenVals = true;
				CRenderizableDisplayList::notifyChange();
				MRPT_END
			}

			/** Gets the current uncertainty covariance of parameter space */
			const cov_matrix_t &getCovMatrix() const { return m_cov; }

			/** Changes the number of "sigmas" for drawing the ellipse/ellipsoid (default=3) */
			void setQuantiles(float q) { m_quantiles=q; CRenderizableDisplayList::notifyChange(); }
			float getQuantiles() const { return m_quantiles; }

			/** The line width for 2D ellipses or 3D wireframe ellipsoids (default=1) */
			void setLineWidth(float w) { m_lineWidth=w; CRenderizableDisplayList::notifyChange(); } 
			float getLineWidth() const { return m_lineWidth; }

			/** Set the number of segments of the surface/curve (higher means with greater resolution) */
			void setNumberOfSegments(const uint32_t numSegments) { m_numSegments=numSegments; CRenderizableDisplayList::notifyChange(); }
			uint32_t getNumberOfSegments() { return m_numSegments; }

			/** Render
			  *	If one of the eigen value of the covariance matrix of the ellipsoid is null, ellipsoid will not
			  * be rendered to ensure stability in the rendering process.
			  */
			void  render_dl() const
			{
				MRPT_START
				// 1) Update eigenvectors/values:
				if (m_needToRecomputeEigenVals) 
				{
					m_needToRecomputeEigenVals = false;
					// Handle the special case of an ellipsoid of volume = 0
					if (m_cov.det()==0) {
						// All zeros:
						m_U.setZero(DIM,DIM);
					}
					else{
						// Not null matrix:
						m_cov.chol(m_U);
					}
				}

				// Only if all the eigenvalues are !=0
				bool eig_ok = true;
				for (int i=0;i<DIM;i++) 
					if (m_U.coeff(i,i)==0) 
						eig_ok=false;

				if(eig_ok)
				{
					// 2) Generate "standard" ellipsoid:
					std::vector<array_point_t> params_pts;
					const cov_matrix_t Uscaled = static_cast<double>(m_quantiles) * m_U;
					detail::generalizedEllipsoidPoints(Uscaled,m_mean, params_pts,m_numSegments,m_numSegments);
					
					// 3) Transform into 2D/3D render space:
					std::vector<array_point_t> render_pts;
					this->transformFromParameterSpace(params_pts,render_pts);

					// 4) Render them:
					mrpt::opengl::detail::renderGeneralizedEllipsoidTemplate<DIM>(render_pts,
						m_lineWidth,
						m_numSegments,m_numSegments );
				}

				MRPT_END
			}
			
			/** Ray tracing
			  */
			virtual bool traceRay(const mrpt::poses::CPose3D &o,double &dist) const { THROW_EXCEPTION("Not implemented ") }

		protected:
			/** To be implemented by derived classes: maps, using some arbitrary space transformation, a list of points 
			  *  defining an ellipsoid in parameter space into their corresponding points in 2D/3D space.
			  */
			virtual void transformFromParameterSpace(
				const std::vector<array_point_t> &params_pts,
				std::vector<array_point_t> & out_pts) const = 0;

			mutable cov_matrix_t m_cov;
			mean_vector_t   m_mean; 
			mutable bool    m_needToRecomputeEigenVals;
			float           m_quantiles;	//!< The number of "sigmas" for drawing the ellipse/ellipsoid (default=3)
			float           m_lineWidth;	//!< The line width for 2D ellipses or 3D wireframe ellipsoids (default=1)
			uint32_t        m_numSegments;  //!< Number of segments in 2D/3D ellipsoids (default=10)
			
			mutable cov_matrix_t  m_U;  //!< Cholesky U triangular matrix cache. */

			void  thisclass_writeToStream(CStream &out) const
			{
				const uint8_t version = 0;
				out << version 
					<< m_cov << m_mean
					<< m_quantiles << m_lineWidth << m_numSegments;
			}
			void  thisclass_readFromStream(CStream &in)
			{
				uint8_t version;
				in >> version;
				switch (version)
				{
				case 0:
					{
						in  >> m_cov >> m_mean
							>> m_quantiles >> m_lineWidth  >> m_numSegments;
						m_needToRecomputeEigenVals = true;
					}
					break;
				default:
					MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
				};
			}

			CGeneralizedEllipsoidTemplate() : 
				m_needToRecomputeEigenVals(true),
				m_quantiles(3.f),
				m_lineWidth(1.f),
				m_numSegments(50)
			{
			}
			virtual ~CGeneralizedEllipsoidTemplate() { }
		};

	} // end namespace

} // End of namespace


#endif
