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
#ifndef CPosePDFGaussian_H
#define CPosePDFGaussian_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt
{
namespace poses
{
	using namespace mrpt::math;

	class CPose3DPDF;

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPosePDFGaussian, CPosePDF )

	/** Declares a class that represents a Probability Density  function (PDF) of a 2D pose \f$ p(\mathbf{x}) = [x ~ y ~ \phi ]^t \f$.
	 *
	 *   This class implements that PDF using a mono-modal Gaussian distribution. See mrpt::poses::CPosePDF for more details.
	 *
	 * \sa CPose2D, CPosePDF, CPosePDFParticles
	 * \ingroup poses_pdf_grp
	 */
	class BASE_IMPEXP CPosePDFGaussian : public CPosePDF
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPosePDFGaussian )

	protected:
		/** Assures the symmetry of the covariance matrix (eventually certain operations in the math-coprocessor lead to non-symmetric matrixes!)
		  */
		void  assureSymmetry();

	 public:
		/** @name Data fields
			@{ */

		CPose2D				mean;	//!< The mean value
		CMatrixDouble33		cov;	//!< The 3x3 covariance matrix

		/** @} */

		inline const CPose2D & getPoseMean() const { return mean; }
		inline       CPose2D & getPoseMean()       { return mean; }

		/** Default constructor
		  */
		CPosePDFGaussian();

		/** Constructor
		  */
		explicit CPosePDFGaussian( const CPose2D &init_Mean );

		/** Constructor
		  */
		CPosePDFGaussian( const CPose2D &init_Mean, const CMatrixDouble33 &init_Cov );

	    /** Copy constructor, including transformations between other PDFs */
		explicit CPosePDFGaussian( const CPosePDF &o ) { copyFrom( o ); }

		/** Copy constructor, including transformations between other PDFs */
		explicit CPosePDFGaussian( const CPose3DPDF &o ) { copyFrom( o ); }

		 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF).
		   * \sa getCovariance
		   */
		void getMean(CPose2D &mean_pose) const {
			mean_pose = mean;
		}

		/** Returns an estimate of the pose covariance matrix (3x3 cov matrix) and the mean, both at once.
		  * \sa getMean
		  */
		void getCovarianceAndMean(CMatrixDouble33 &cov,CPose2D &mean_point) const {
			mean_point = mean;
			cov = this->cov;
		}

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		void  copyFrom(const CPosePDF &o);

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		void  copyFrom(const CPose3DPDF &o);

		/** Save PDF's particles to a text file, containing the 2D pose in the first line, then the covariance matrix in next 3 lines.
		 */
		void  saveToTextFile(const std::string &file) const;

		/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference( const CPose3D &newReferenceBase );

		/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference( const CPose2D &newReferenceBase );

		/** Rotate the covariance matrix by replacing it by \f$ \mathbf{R}~\mathbf{COV}~\mathbf{R}^t \f$, where \f$ \mathbf{R} = \left[ \begin{array}{ccc} \cos\alpha & -\sin\alpha & 0 \\ \sin\alpha & \cos\alpha & 0 \\ 0 & 0 & 1 \end{array}\right] \f$.
		  */
		void  rotateCov(const double ang);

		/** Set \f$ this = x1 \ominus x0 \f$ , computing the mean using the "-" operator and the covariances through the corresponding Jacobians (For 'x0' and 'x1' being independent variables!).
		  */
		void inverseComposition( const CPosePDFGaussian &x, const CPosePDFGaussian &ref  );

		/** Set \f$ this = x1 \ominus x0 \f$ , computing the mean using the "-" operator and the covariances through the corresponding Jacobians (Given the 3x3 cross-covariance matrix of variables x0 and x1).
		  */
		void inverseComposition(
			const CPosePDFGaussian &x1,
			const CPosePDFGaussian &x0,
			const CMatrixDouble33  &COV_01
			  );

		/** Draws a single sample from the distribution
		  */
		void  drawSingleSample( CPose2D &outPart ) const;

		/** Draws a number of samples from the distribution, and saves as a list of 1x3 vectors, where each row contains a (x,y,phi) datum.
		  */
		void  drawManySamples( size_t N, std::vector<vector_double> & outSamples ) const;

		/** Bayesian fusion of two points gauss. distributions, then save the result in this object.
		  *  The process is as follows:<br>
		  *		- (x1,S1): Mean and variance of the p1 distribution.
		  *		- (x2,S2): Mean and variance of the p2 distribution.
		  *		- (x,S): Mean and variance of the resulting distribution.
		  *
		  *    S = (S1<sup>-1</sup> + S2<sup>-1</sup>)<sup>-1</sup>;
		  *    x = S * ( S1<sup>-1</sup>*x1 + S2<sup>-1</sup>*x2 );
		  */
		void  bayesianFusion(const  CPosePDF &p1,const  CPosePDF &p2, const double &minMahalanobisDistToDrop = 0 );

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
		  */
		void	 inverse(CPosePDF &o) const;

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated).
		  */
		void  operator += ( const CPose2D &Ap);

		/** Evaluates the PDF at a given point.
		  */
		double  evaluatePDF( const CPose2D &x ) const;

		/** Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1].
		  */
		double  evaluateNormalizedPDF( const CPose2D &x ) const;

		/** Computes the Mahalanobis distance between the centers of two Gaussians.
		  */
		double  mahalanobisDistanceTo( const CPosePDFGaussian& theOther );

		/** Substitutes the diagonal elements if (square) they are below some given minimum values (Use this before bayesianFusion, for example, to avoid inversion of singular matrixes, etc...)
		  */
		void  assureMinCovariance( const double & minStdXY, const double &minStdPhi );

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated) (see formulas in jacobiansPoseComposition ).
		  */
		void  operator += ( const CPosePDFGaussian &Ap);

		/** Makes: thisPDF = thisPDF - Ap, where "-" is pose inverse composition (both the mean, and the covariance matrix are updated)
		  */
		inline void operator -=( const CPosePDFGaussian &ref  ) {
			this->inverseComposition(*this,ref);
		}


	}; // End of class def.


	/** Pose compose operator: RES = A (+) B , computing both the mean and the covariance */
	inline CPosePDFGaussian operator +( const CPosePDFGaussian &a, const CPosePDFGaussian &b  ) {
		CPosePDFGaussian res(a);
		res+=b;
		return res;
	}

	/** Pose inverse compose operator: RES = A (-) B , computing both the mean and the covariance */
	inline CPosePDFGaussian operator -( const CPosePDFGaussian &a, const CPosePDFGaussian &b  ) {
		CPosePDFGaussian res;
		res.inverseComposition(a,b);
		return res;
	}

	/** Dumps the mean and covariance matrix to a text stream.
	  */
	std::ostream BASE_IMPEXP & operator << (std::ostream & out, const CPosePDFGaussian& obj);

	/** Returns the Gaussian distribution of \f$ \mathbf{C} \f$, for \f$ \mathbf{C} = \mathbf{A} \oplus \mathbf{B} \f$.
	  */
	poses::CPosePDFGaussian	BASE_IMPEXP operator + ( const mrpt::poses::CPose2D &A, const mrpt::poses::CPosePDFGaussian &B  );

	bool BASE_IMPEXP operator==(const CPosePDFGaussian &p1,const CPosePDFGaussian &p2);

	} // End of namespace
} // End of namespace

#endif
