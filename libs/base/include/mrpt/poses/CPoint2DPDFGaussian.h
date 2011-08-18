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
#ifndef CPoint2DPDFGaussian_H
#define CPoint2DPDFGaussian_H

#include <mrpt/poses/CPoint2DPDF.h>

namespace mrpt
{
namespace poses
{
	using namespace mrpt::math;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPoint2DPDFGaussian, CPoint2DPDF )

	/** A gaussian distribution for 2D points. Also a method for bayesian fusion is provided.
	 * \ingroup poses_pdf_grp
	 * \sa CPoint2DPDF
	 */
	class BASE_IMPEXP CPoint2DPDFGaussian : public CPoint2DPDF
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPoint2DPDFGaussian )

	 public:
		/** Default constructor
		  */
		CPoint2DPDFGaussian();

		/** Constructor
		  */
		CPoint2DPDFGaussian( const CPoint2D &init_Mean );

		/** Constructor
		  */
		CPoint2DPDFGaussian( const CPoint2D &init_Mean, const CMatrixDouble22 &init_Cov );

		/** The mean value
		 */
		CPoint2D	mean;

		/** The 2x2 covariance matrix
		 */
		CMatrixDouble22		cov;

		 /** Returns an estimate of the point, (the mean, or mathematical expectation of the PDF)
		  */
		void getMean(CPoint2D &p) const {
			p = this->mean;
		}

		/** Returns an estimate of the point covariance matrix (2x2 cov matrix) and the mean, both at once.
		  * \sa getMean
		  */
		void getCovarianceAndMean(CMatrixDouble22 &cov,CPoint2D &mean_point) const {
			cov = this->cov;
			mean_point = this->mean;
		}

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		void  copyFrom(const CPoint2DPDF &o);

		/** Save PDF's particles to a text file, containing the 2D pose in the first line, then the covariance matrix in next 3 lines.
		 */
		void  saveToTextFile(const std::string &file) const;

		/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. Both the mean value and the covariance matrix are updated correctly.
		  */
		void  changeCoordinatesReference( const CPose3D &newReferenceBase );

		/** Bayesian fusion of two points gauss. distributions, then save the result in this object.
		  *  The process is as follows:<br>
		  *		- (x1,S1): Mean and variance of the p1 distribution.
		  *		- (x2,S2): Mean and variance of the p2 distribution.
		  *		- (x,S): Mean and variance of the resulting distribution.
		  *
		  *    S = (S1<sup>-1</sup> + S2<sup>-1</sup>)<sup>-1</sup>;
		  *    x = S * ( S1<sup>-1</sup>*x1 + S2<sup>-1</sup>*x2 );
		  */
		void  bayesianFusion( const CPoint2DPDFGaussian &p1, const CPoint2DPDFGaussian &p2 );

		/** Computes the "correspondence likelihood" of this PDF with another one: This is implemented as the integral from -inf to +inf of the product of both PDF.
		  * The resulting number is >=0.
		  * \sa productIntegralNormalizedWith
		  * \exception std::exception On errors like covariance matrix with null determinant, etc...
		  */
		double  productIntegralWith( const CPoint2DPDFGaussian &p) const;

		/** Computes the "correspondence likelihood" of this PDF with another one: This is implemented as the integral from -inf to +inf of the product of both PDF.
		  * The resulting number is in the range [0,1].
		  *  Note that the resulting value is in fact
		  *  \f[ exp( -\frac{1}{2} D^2 ) \f]
		  *  , with \f$ D^2 \f$ being the square Mahalanobis distance between the two pdfs.
		  * \sa productIntegralWith
		  * \exception std::exception On errors like covariance matrix with null determinant, etc...
		  */
		double  productIntegralNormalizedWith( const CPoint2DPDFGaussian &p) const;

		/** Draw a sample from the pdf.
		  */
		void drawSingleSample(CPoint2D  &outSample) const;

		/** Bayesian fusion of two point distributions (product of two distributions->new distribution), then save the result in this object (WARNING: See implementing classes to see classes that can and cannot be mixtured!)
		  * \param p1 The first distribution to fuse
		  * \param p2 The second distribution to fuse
		  * \param minMahalanobisDistToDrop If set to different of 0, the result of very separate Gaussian modes (that will result in negligible components) in SOGs will be dropped to reduce the number of modes in the output.
		  */
		void  bayesianFusion( const CPoint2DPDF &p1, const CPoint2DPDF &p2, const double &minMahalanobisDistToDrop = 0);


		/** Returns the Mahalanobis distance from this PDF to another PDF, that is, it's evaluation at (0,0,0)
		  */
		double mahalanobisDistanceTo( const CPoint2DPDFGaussian & other ) const;


	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
