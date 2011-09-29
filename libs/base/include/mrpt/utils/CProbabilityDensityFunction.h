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
#ifndef CProbabilityDensityFunction_H
#define CProbabilityDensityFunction_H

#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt
{
	namespace poses { class CPose3D; }

	namespace utils
	{
		using namespace mrpt::math;

		/** A generic template for probability density distributions (PDFs).
		  * This template is used as base for many classes in mrpt::poses
		  *  Any derived class must implement \a getMean() and a getCovarianceAndMean().
		  *  Other methods such as \a getMean() or \a getCovariance() are implemented here for convenience.
		  * \sa mprt::poses::CPosePDF, mprt::poses::CPose3DPDF, mprt::poses::CPointPDF
		 * \ingroup mrpt_base_grp
 		  */
		template <class TDATA, size_t STATE_LEN>
		class CProbabilityDensityFunction
		{
		public:
			static const size_t state_length = STATE_LEN;	//!< The length of the variable, for example, 3 for a 3D point, 6 for a 3D pose (x y z yaw pitch roll).
			typedef TDATA type_value;  //!< The type of the state the PDF represents

			 /** Returns the mean, or mathematical expectation of the probability density distribution (PDF).
			   * \sa getCovarianceAndMean, getInformationMatrix
			   */
			virtual void getMean(TDATA &mean_point) const = 0;

			/** Returns an estimate of the pose covariance matrix (STATE_LENxSTATE_LEN cov matrix) and the mean, both at once.
			  * \sa getMean, getInformationMatrix
			  */
			virtual void getCovarianceAndMean(CMatrixFixedNumeric<double,STATE_LEN,STATE_LEN> &cov,TDATA  &mean_point) const = 0;

			/** Returns an estimate of the pose covariance matrix (STATE_LENxSTATE_LEN cov matrix) and the mean, both at once.
			  * \sa getMean, getInformationMatrix
			  */
			inline void getCovarianceDynAndMean(CMatrixDouble &cov,TDATA  &mean_point) const
			{
				CMatrixFixedNumeric<double,STATE_LEN,STATE_LEN> C(UNINITIALIZED_MATRIX);
				this->getCovarianceAndMean(C,mean_point);
				cov = C; // Convert to dynamic size matrix
			}

			/** Returns the mean, or mathematical expectation of the probability density distribution (PDF).
			   * \sa getCovariance, getInformationMatrix
			   */
			inline TDATA getMeanVal() const
			{
				TDATA p;
				getMean(p);
				return p;
			}

			/** Returns the estimate of the covariance matrix (STATE_LEN x STATE_LEN covariance matrix)
			  * \sa getMean, getCovarianceAndMean, getInformationMatrix
			  */
			inline void getCovariance(CMatrixDouble &cov) const
			{
				TDATA p;
				this->getCovarianceDynAndMean(cov,p);
			}

			/** Returns the estimate of the covariance matrix (STATE_LEN x STATE_LEN covariance matrix)
			  * \sa getMean, getCovarianceAndMean, getInformationMatrix
			  */
			inline void getCovariance(CMatrixFixedNumeric<double,STATE_LEN,STATE_LEN> &cov) const
			{
				TDATA p;
				this->getCovarianceAndMean(cov,p);
			}

			/** Returns the estimate of the covariance matrix (STATE_LEN x STATE_LEN covariance matrix)
			  * \sa getMean, getInformationMatrix
			  */
			inline CMatrixFixedNumeric<double,STATE_LEN,STATE_LEN> getCovariance() const
			{
				CMatrixFixedNumeric<double,STATE_LEN,STATE_LEN> cov(UNINITIALIZED_MATRIX);
				TDATA p;
				this->getCovarianceAndMean(cov,p);
				return cov;
			}


			/** Returns the information (inverse covariance) matrix (a STATE_LEN x STATE_LEN matrix)
			  *  Unless reimplemented in derived classes, this method first reads the covariance, then invert it.
			  * \sa getMean, getCovarianceAndMean
			  */
			virtual void getInformationMatrix(CMatrixFixedNumeric<double,STATE_LEN,STATE_LEN> &inf) const
			{
				CMatrixFixedNumeric<double,STATE_LEN,STATE_LEN> cov(UNINITIALIZED_MATRIX);
				TDATA p;
				this->getCovarianceAndMean(cov,p);
				cov.inv_fast(inf); // Destroy source cov matrix, since we don't need it anymore.
			}

			/** Save PDF's particles to a text file. See derived classes for more information about the format of generated files.
			*/
			virtual void  saveToTextFile(const std::string &file) const = 0;

			/** Draws a single sample from the distribution
			  */
			virtual void  drawSingleSample( TDATA &outPart ) const = 0;

			/** Draws a number of samples from the distribution, and saves as a list of 1xSTATE_LEN vectors, where each row contains a (x,y,z,yaw,pitch,roll) datum.
			  * This base method just call N times to drawSingleSample, but derived classes should implemented optimized method for each particular PDF.
			  */
			virtual void  drawManySamples( size_t N, std::vector<vector_double> & outSamples ) const
			{
				outSamples.resize(N);
				TDATA	pnt;
				for (size_t i=0;i<N;i++)
				{
					this->drawSingleSample(pnt);
					pnt.getAsVector(outSamples[i]);
				}
			}

			/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
			  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
			  */
			virtual void  changeCoordinatesReference( const mrpt::poses::CPose3D &newReferenceBase ) = 0;

			/** Compute the entropy of the estimated covariance matrix.
			  * \sa http://en.wikipedia.org/wiki/Multivariate_normal_distribution#Entropy
			  */
			inline double  getCovarianceEntropy() const
			{
				static const double ln_2PI= 1.8378770664093454835606594728112;
				return 0.5*( STATE_LEN + STATE_LEN * ln_2PI + log( std::max(getCovariance().det(), std::numeric_limits<double>::epsilon() ) ) );
			}

		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
