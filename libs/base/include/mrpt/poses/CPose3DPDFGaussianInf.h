/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPose3DPDFGaussianInf_H
#define CPose3DPDFGaussianInf_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/math/CMatrixD.h>

namespace mrpt
{
namespace poses
{
	class CPosePDFGaussian;
	class CPose3DQuatPDFGaussian;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPose3DPDFGaussianInf , CPose3DPDF )

	/** Declares a class that represents a Probability Density function (PDF) of a 3D pose \f$ p(\mathbf{x}) = [x ~ y ~ z ~ yaw ~ pitch ~ roll]^t \f$ as a Gaussian described by its mean and its inverse covariance matrix.
	 *
	 *   This class implements that PDF using a mono-modal Gaussian distribution in "information" form (inverse covariance matrix).
	 *
	 *  Uncertainty of pose composition operations (\f$ y = x \oplus u \f$) is implemented in the method "CPose3DPDFGaussianInf::operator+=".
	 *
	 *  For further details on implemented methods and the theory behind them,
	 *  see <a href="http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty" >this report</a>.
	 *
	 * \sa CPose3D, CPose3DPDF, CPose3DPDFParticles, CPose3DPDFGaussian
	 * \ingroup poses_pdf_grp
	 */
	class BASE_IMPEXP CPose3DPDFGaussianInf : public CPose3DPDF
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPose3DPDFGaussianInf )

	protected:
		/** Assures the symmetry of the covariance matrix (eventually certain operations in the math-coprocessor lead to non-symmetric matrixes!)
		  */
		void  assureSymmetry();

	 public:
		/** @name Data fields
			@{   */

		CPose3D				mean;		//!< The mean value
		mrpt::math::CMatrixDouble66		cov_inv;	//!< The inverse of the 6x6 covariance matrix

		/** @} */

		inline const CPose3D & getPoseMean() const { return mean; }
		inline       CPose3D & getPoseMean()       { return mean; }

		 /** Default constructor - mean: all zeros, inverse covariance=all zeros -> so be careful! */
		CPose3DPDFGaussianInf();

		/** Constructor with a mean value, inverse covariance=all zeros -> so be careful! */
		explicit CPose3DPDFGaussianInf( const CPose3D &init_Mean );

		/** Uninitialized constructor: leave all fields uninitialized - Call with UNINITIALIZED_POSE as argument */
		CPose3DPDFGaussianInf(TConstructorFlags_Poses constructor_dummy_param);

		/** Constructor with mean and inv cov. */
		CPose3DPDFGaussianInf( const CPose3D &init_Mean, const mrpt::math::CMatrixDouble66 &init_CovInv );

		/** Constructor from a 6D pose PDF described as a Quaternion */
		explicit CPose3DPDFGaussianInf( const CPose3DQuatPDFGaussian &o);

		 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF).
		   * \sa getCovariance */
		void getMean(CPose3D &mean_pose) const MRPT_OVERRIDE {
			mean_pose = mean;
		}

		/** Returns an estimate of the pose covariance matrix (6x6 cov matrix) and the mean, both at once.
		  * \sa getMean  */
		void getCovarianceAndMean(mrpt::math::CMatrixDouble66 &cov,CPose3D &mean_point) const MRPT_OVERRIDE {
			mean_point = this->mean;
			this->cov_inv.inv(cov);
		}

		/** Returns the information (inverse covariance) matrix (a STATE_LEN x STATE_LEN matrix) \sa getMean, getCovarianceAndMean */
		virtual void getInformationMatrix(mrpt::math::CMatrixDouble66 &inf) const MRPT_OVERRIDE { inf=cov_inv; }

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations) */
		void  copyFrom(const CPose3DPDF &o) MRPT_OVERRIDE;

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations) */
		void  copyFrom(const CPosePDF &o);

		/** Copy from a 6D pose PDF described as a Quaternion
		  */
		void copyFrom( const CPose3DQuatPDFGaussian &o);

		/** Save the PDF to a text file, containing the 3D pose in the first line, then the covariance matrix in next 3 lines. */
		void  saveToTextFile(const std::string &file) const MRPT_OVERRIDE;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. */
		void  changeCoordinatesReference(  const CPose3D &newReferenceBase ) MRPT_OVERRIDE;

		/** Draws a single sample from the distribution */
		void  drawSingleSample( CPose3D &outPart ) const MRPT_OVERRIDE;

		/** Draws a number of samples from the distribution, and saves as a list of 1x6 vectors, where each row contains a (x,y,phi) datum. */
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
		void  bayesianFusion( const CPose3DPDF &p1, const CPose3DPDF &p2 ) MRPT_OVERRIDE;

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF */
		void	 inverse(CPose3DPDF &o) const MRPT_OVERRIDE;

		/** Unary - operator, returns the PDF of the inverse pose.  */
		inline CPose3DPDFGaussianInf operator -() const
		{
			CPose3DPDFGaussianInf p(UNINITIALIZED_POSE);
			this->inverse(p);
			return p;
		}


		void operator += ( const CPose3D &Ap);  //!< Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated)
		void operator += ( const CPose3DPDFGaussianInf &Ap); //!< Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated)
		void operator -= ( const CPose3DPDFGaussianInf &Ap); //!< Makes: thisPDF = thisPDF - Ap, where "-" is pose inverse composition (both the mean, and the covariance matrix are updated)
		double evaluatePDF( const CPose3D &x ) const; //!< Evaluates the PDF at a given point
		double evaluateNormalizedPDF( const CPose3D &x ) const; //!< Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1]
		void getInvCovSubmatrix2D( mrpt::math::CMatrixDouble &out_cov ) const; //!< Returns a 3x3 matrix with submatrix of the inverse covariance for the variables (x,y,yaw) only

		/** Computes the Mahalanobis distance between the centers of two Gaussians.
		  *  The variables with a variance exactly equal to 0 are not taken into account in the process, but
		  *   "infinity" is returned if the corresponding elements are not exactly equal.
		  */
		double  mahalanobisDistanceTo( const CPose3DPDFGaussianInf& theOther);

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPose3DPDFGaussianInf , CPose3DPDF )


	bool BASE_IMPEXP operator==(const CPose3DPDFGaussianInf &p1,const CPose3DPDFGaussianInf &p2);
	/** Pose composition for two 3D pose Gaussians  \sa CPose3DPDFGaussian::operator +=  */
	CPose3DPDFGaussianInf BASE_IMPEXP operator +( const CPose3DPDFGaussianInf &x, const CPose3DPDFGaussianInf &u );
	/** Pose composition for two 3D pose Gaussians  \sa CPose3DPDFGaussianInf::operator -=  */
	CPose3DPDFGaussianInf BASE_IMPEXP operator -( const CPose3DPDFGaussianInf &x, const CPose3DPDFGaussianInf &u );
	/** Dumps the mean and covariance matrix to a text stream. */
	std::ostream  BASE_IMPEXP & operator << (std::ostream & out, const CPose3DPDFGaussianInf& obj);

	} // End of namespace
} // End of namespace
#endif
