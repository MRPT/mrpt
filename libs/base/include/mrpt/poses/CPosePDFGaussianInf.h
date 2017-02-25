/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPosePDFGaussianInf_H
#define CPosePDFGaussianInf_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

namespace mrpt
{
namespace poses
{
	class CPose3DPDF;

	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPosePDFGaussianInf, CPosePDF )

	/** A Probability Density  function (PDF) of a 2D pose \f$ p(\mathbf{x}) = [x ~ y ~ \phi ]^t \f$ as a Gaussian with a mean and the inverse of the covariance.
	 *
	 *   This class implements a PDF as a mono-modal Gaussian distribution in its <b>information form</b>, that is,
	 *     keeping the inverse of the covariance matrix instead of the covariance matrix itself.
	 *
	 *  This class is the dual of CPosePDFGaussian.
	 *
	 * \sa CPose2D, CPosePDF, CPosePDFParticles
	 * \ingroup poses_pdf_grp
	 */
	class BASE_IMPEXP CPosePDFGaussianInf : public CPosePDF
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPosePDFGaussianInf )

	protected:
		/** Assures the symmetry of the covariance matrix (eventually certain operations in the math-coprocessor lead to non-symmetric matrixes!)
		  */
		void  assureSymmetry();

	 public:
		/** @name Data fields
			@{ */

		CPose2D		mean;	//!< The mean value
		mrpt::math::CMatrixDouble33		cov_inv;	//!< The inverse of the 3x3 covariance matrix (the "information" matrix)

		/** @} */

		inline const CPose2D & getPoseMean() const { return mean; }
		inline       CPose2D & getPoseMean()       { return mean; }

		/** Default constructor (mean=all zeros, inverse covariance=all zeros -> so be careful!) */
		CPosePDFGaussianInf();

		/** Constructor with a mean value (inverse covariance=all zeros -> so be careful!) */
		explicit CPosePDFGaussianInf( const CPose2D &init_Mean );

		/** Constructor */
		CPosePDFGaussianInf( const CPose2D &init_Mean, const mrpt::math::CMatrixDouble33 &init_CovInv );

	    /** Copy constructor, including transformations between other PDFs */
		explicit CPosePDFGaussianInf( const CPosePDF &o ) { copyFrom( o ); }

		/** Copy constructor, including transformations between other PDFs */
		explicit CPosePDFGaussianInf( const CPose3DPDF &o ) { copyFrom( o ); }

		 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF).
		   * \sa getCovariance */
		void getMean(CPose2D &mean_pose) const MRPT_OVERRIDE {
			mean_pose = mean;
		}

		/** Returns an estimate of the pose covariance matrix (3x3 cov matrix) and the mean, both at once.
		  * \sa getMean */
		void getCovarianceAndMean(mrpt::math::CMatrixDouble33 &cov,CPose2D &mean_point) const  MRPT_OVERRIDE{
			mean_point = mean;
			this->cov_inv.inv(cov);
		}

		/** Returns the information (inverse covariance) matrix (a STATE_LEN x STATE_LEN matrix) \sa getMean, getCovarianceAndMean */
		virtual void getInformationMatrix(mrpt::math::CMatrixDouble33 &inf) const  MRPT_OVERRIDE { inf=cov_inv; }

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations) */
		void  copyFrom(const CPosePDF &o) MRPT_OVERRIDE;

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations) */
		void  copyFrom(const CPose3DPDF &o);

		/** Save PDF's particles to a text file, containing the 2D pose in the first line, then the covariance matrix in next 3 lines. */
		void  saveToTextFile(const std::string &file) const MRPT_OVERRIDE;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object */
		void  changeCoordinatesReference( const CPose3D &newReferenceBase ) MRPT_OVERRIDE;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. */
		void  changeCoordinatesReference( const CPose2D &newReferenceBase );

		/** Rotate the covariance matrix by replacing it by \f$ \mathbf{R}~\mathbf{COV}~\mathbf{R}^t \f$, where \f$ \mathbf{R} = \left[ \begin{array}{ccc} \cos\alpha & -\sin\alpha & 0 \\ \sin\alpha & \cos\alpha & 0 \\ 0 & 0 & 1 \end{array}\right] \f$. */
		void  rotateCov(const double ang);

		/** Set \f$ this = x1 \ominus x0 \f$ , computing the mean using the "-" operator and the covariances through the corresponding Jacobians (For 'x0' and 'x1' being independent variables!).  */
		void inverseComposition( const CPosePDFGaussianInf &x, const CPosePDFGaussianInf &ref  );

		/** Set \f$ this = x1 \ominus x0 \f$ , computing the mean using the "-" operator and the covariances through the corresponding Jacobians (Given the 3x3 cross-covariance matrix of variables x0 and x1). */
		void inverseComposition(
			const CPosePDFGaussianInf &x1,
			const CPosePDFGaussianInf &x0,
			const mrpt::math::CMatrixDouble33  &COV_01
			);

		/** Draws a single sample from the distribution */
		void  drawSingleSample( CPose2D &outPart ) const MRPT_OVERRIDE;

		/** Draws a number of samples from the distribution, and saves as a list of 1x3 vectors, where each row contains a (x,y,phi) datum. */
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

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF */
		void	 inverse(CPosePDF &o) const MRPT_OVERRIDE;

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated). */
		void  operator += ( const CPose2D &Ap);

		/** Evaluates the PDF at a given point */
		double  evaluatePDF( const CPose2D &x ) const;

		/** Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1]. */
		double  evaluateNormalizedPDF( const CPose2D &x ) const;

		/** Computes the Mahalanobis distance between the centers of two Gaussians. */
		double  mahalanobisDistanceTo( const CPosePDFGaussianInf& theOther );

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated) (see formulas in jacobiansPoseComposition ). */
		void  operator += ( const CPosePDFGaussianInf &Ap);

		/** Makes: thisPDF = thisPDF - Ap, where "-" is pose inverse composition (both the mean, and the covariance matrix are updated) */
		inline void operator -=( const CPosePDFGaussianInf &ref  ) {
			this->inverseComposition(*this,ref);
		}

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPosePDFGaussianInf, CPosePDF )

	bool BASE_IMPEXP operator==(const CPosePDFGaussianInf &p1,const CPosePDFGaussianInf &p2);
	/** Pose compose operator: RES = A (+) B , computing both the mean and the covariance */
	CPosePDFGaussianInf BASE_IMPEXP operator +( const CPosePDFGaussianInf &a, const CPosePDFGaussianInf &b  );
	/** Pose inverse compose operator: RES = A (-) B , computing both the mean and the covariance */
	CPosePDFGaussianInf BASE_IMPEXP operator -( const CPosePDFGaussianInf &a, const CPosePDFGaussianInf &b  );
	/** Returns the Gaussian distribution of \f$ \mathbf{C} \f$, for \f$ \mathbf{C} = \mathbf{A} \oplus \mathbf{B} \f$. */
	poses::CPosePDFGaussianInf	BASE_IMPEXP operator + ( const mrpt::poses::CPose2D &A, const mrpt::poses::CPosePDFGaussianInf &B  );

	/** Dumps the mean and covariance matrix to a text stream. */
	std::ostream BASE_IMPEXP & operator << (std::ostream & out, const CPosePDFGaussianInf& obj);

	} // End of namespace
} // End of namespace

#endif
