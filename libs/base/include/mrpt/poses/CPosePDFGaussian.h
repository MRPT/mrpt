/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPosePDFGaussian_H
#define CPosePDFGaussian_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt
{
namespace poses
{
	class CPose3DPDF;
	class CPoint2DPDFGaussian;

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
		mrpt::math::CMatrixDouble33		cov;	//!< The 3x3 covariance matrix

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
		CPosePDFGaussian( const CPose2D &init_Mean, const mrpt::math::CMatrixDouble33 &init_Cov );

	    /** Copy constructor, including transformations between other PDFs */
		explicit CPosePDFGaussian( const CPosePDF &o ) { copyFrom( o ); }

		/** Copy constructor, including transformations between other PDFs */
		explicit CPosePDFGaussian( const CPose3DPDF &o ) { copyFrom( o ); }

		 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF).
		   * \sa getCovariance
		   */
		void getMean(CPose2D &mean_pose) const  MRPT_OVERRIDE{
			mean_pose = mean;
		}

		/** Returns an estimate of the pose covariance matrix (3x3 cov matrix) and the mean, both at once.
		  * \sa getMean
		  */
		void getCovarianceAndMean(mrpt::math::CMatrixDouble33 &out_cov,CPose2D &mean_point) const  MRPT_OVERRIDE{
			mean_point = mean;
			out_cov = this->cov;
		}

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations) */
		void  copyFrom(const CPosePDF &o) MRPT_OVERRIDE;

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations) */
		void  copyFrom(const CPose3DPDF &o);

		/** Save PDF's particles to a text file, containing the 2D pose in the first line, then the covariance matrix in next 3 lines. */
		void  saveToTextFile(const std::string &file) const MRPT_OVERRIDE;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference( const CPose3D &newReferenceBase ) MRPT_OVERRIDE;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference( const CPose2D &newReferenceBase );

		/** Rotate the covariance matrix by replacing it by \f$ \mathbf{R}~\mathbf{COV}~\mathbf{R}^t \f$, where \f$ \mathbf{R} = \left[ \begin{array}{ccc} \cos\alpha & -\sin\alpha & 0 \\ \sin\alpha & \cos\alpha & 0 \\ 0 & 0 & 1 \end{array}\right] \f$.
		  */
		void  rotateCov(const double ang);

		/** Set \f$ this = x1 \ominus x0 \f$ , computing the mean using the "-" operator and the covariances through the corresponding Jacobians (For 'x0' and 'x1' being independent variables!). */
		void inverseComposition( const CPosePDFGaussian &x, const CPosePDFGaussian &ref  );

		/** Set \f$ this = x1 \ominus x0 \f$ , computing the mean using the "-" operator and the covariances through the corresponding Jacobians (Given the 3x3 cross-covariance matrix of variables x0 and x1). */
		void inverseComposition(
			const CPosePDFGaussian &x1,
			const CPosePDFGaussian &x0,
			const mrpt::math::CMatrixDouble33  &COV_01
			  );

		/** Draws a single sample from the distribution
		  */
		void  drawSingleSample( CPose2D &outPart ) const MRPT_OVERRIDE;

		/** Draws a number of samples from the distribution, and saves as a list of 1x3 vectors, where each row contains a (x,y,phi) datum.
		  */
		void  drawManySamples( size_t N, std::vector<mrpt::math::CVectorDouble> & outSamples ) const MRPT_OVERRIDE;

		/** Bayesian fusion of two points gauss. distributions, then save the result in this object.
		  *  The process is as follows:<br>
		  *		- (x1,S1): Mean and variance of the p1 distribution.
		  *		- (x2,S2): Mean and variance of the p2 distribution.
		  *		- (x,S): Mean and variance of the resulting distribution.
		  *
		  *    S = (S1<sup>-1</sup> + S2<sup>-1</sup>)<sup>-1</sup>;
		  *    x = S * ( S1<sup>-1</sup>*x1 + S2<sup>-1</sup>*x2 );
		  */
		void  bayesianFusion(const  CPosePDF &p1,const  CPosePDF &p2, const double &minMahalanobisDistToDrop = 0 ) MRPT_OVERRIDE;

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
		  */
		void	 inverse(CPosePDF &o) const MRPT_OVERRIDE;

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated). */
		void  operator += ( const CPose2D &Ap);

		/** Evaluates the PDF at a given point. */
		double  evaluatePDF( const CPose2D &x ) const;

		/** Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1]. */
		double  evaluateNormalizedPDF( const CPose2D &x ) const;

		/** Computes the Mahalanobis distance between the centers of two Gaussians.  */
		double  mahalanobisDistanceTo( const CPosePDFGaussian& theOther );

		/** Substitutes the diagonal elements if (square) they are below some given minimum values (Use this before bayesianFusion, for example, to avoid inversion of singular matrixes, etc...)  */
		void  assureMinCovariance( const double & minStdXY, const double &minStdPhi );

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated) (see formulas in jacobiansPoseComposition ). */
		void  operator += ( const CPosePDFGaussian &Ap);

		/** Makes: thisPDF = thisPDF - Ap, where "-" is pose inverse composition (both the mean, and the covariance matrix are updated) */
		inline void operator -=( const CPosePDFGaussian &ref  ) {
			this->inverseComposition(*this,ref);
		}

		/** Returns the PDF of the 2D point \f$ g = q \oplus l\f$ with "q"=this pose and "l" a point without uncertainty */
		void composePoint(const mrpt::math::TPoint2D &l, CPoint2DPDFGaussian &g ) const;


	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPosePDFGaussian, CPosePDF )


	/** Pose compose operator: RES = A (+) B , computing both the mean and the covariance */
	CPosePDFGaussian BASE_IMPEXP operator +( const CPosePDFGaussian &a, const CPosePDFGaussian &b  );

	/** Pose inverse compose operator: RES = A (-) B , computing both the mean and the covariance */
	CPosePDFGaussian BASE_IMPEXP  operator -( const CPosePDFGaussian &a, const CPosePDFGaussian &b  );

	/** Dumps the mean and covariance matrix to a text stream. */
	std::ostream BASE_IMPEXP & operator << (std::ostream & out, const CPosePDFGaussian& obj);

	/** Returns the Gaussian distribution of \f$ \mathbf{C} \f$, for \f$ \mathbf{C} = \mathbf{A} \oplus \mathbf{B} \f$. */
	poses::CPosePDFGaussian	BASE_IMPEXP operator + ( const mrpt::poses::CPose2D &A, const mrpt::poses::CPosePDFGaussian &B  );

	bool BASE_IMPEXP operator==(const CPosePDFGaussian &p1,const CPosePDFGaussian &p2);

	} // End of namespace
} // End of namespace

#endif
