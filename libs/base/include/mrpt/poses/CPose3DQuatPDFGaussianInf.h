/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPose3DQuatPDFGaussianInf_H
#define CPose3DQuatPDFGaussianInf_H

#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/math/CMatrixD.h>

namespace mrpt
{
namespace poses
{
	class CPosePDFGaussian;
	class CPose3DPDFGaussian;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPose3DQuatPDFGaussianInf , CPose3DQuatPDF )

	/** Declares a class that represents a Probability Density function (PDF) of a 3D pose using a quaternion \f$ p(\mathbf{x}) = [x ~ y ~ z ~ qr ~ qx ~ qy ~ qz]^\top \f$.
	 *
	 *   This class implements that PDF using a mono-modal Gaussian distribution storing the information matrix instead of its inverse, the covariance matrix.
	 *     See mrpt::poses::CPose3DQuatPDF for more details, or
	 *     mrpt::poses::CPose3DPDF for classes based on Euler angles instead.
	 *
	 *  Uncertainty of pose composition operations (\f$ y = x \oplus u \f$) is implemented in the methods "CPose3DQuatPDFGaussianInf::operator+=" and "CPose3DQuatPDF::jacobiansPoseComposition".
	 *
	 *  For further details on implemented methods and the theory behind them,
	 *  see <a href="http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty" >this report</a>.
	 *
	 * \sa CPose3DQuat, CPose3DQuatPDF, CPose3DPDF, CPose3DQuatPDFGaussian
	 * \ingroup poses_pdf_grp
	 */
	class BASE_IMPEXP CPose3DQuatPDFGaussianInf : public CPose3DQuatPDF
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPose3DQuatPDFGaussianInf )

	 public:
		 /** Default constructor - set all values to zero. */
		CPose3DQuatPDFGaussianInf();

		/** Constructor which left all the member uninitialized, for using when speed is critical - as argument, use UNINITIALIZED_QUATERNION. */
		CPose3DQuatPDFGaussianInf(mrpt::math::TConstructorFlags_Quaternions constructor_dummy_param);

		/** Constructor from a default mean value, information matrix equals to zero. */
		explicit CPose3DQuatPDFGaussianInf( const CPose3DQuat &init_Mean );

		/** Constructor with mean and inverse covariance (information matrix). */
		CPose3DQuatPDFGaussianInf( const CPose3DQuat &init_Mean, const mrpt::math::CMatrixDouble77 &init_CovInv );


		CPose3DQuat                  mean;  //!< The mean value
		mrpt::math::CMatrixDouble77  cov_inv; //!< The 7x7 information matrix (the inverse of the covariance)

		inline const CPose3DQuat & getPoseMean() const { return mean; }
		inline       CPose3DQuat & getPoseMean()       { return mean; }

		 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF)  \sa getCovariance */
		void getMean(CPose3DQuat &mean_pose) const MRPT_OVERRIDE {
			mean_pose = mean;
		}

		/** Returns an estimate of the pose covariance matrix (7x7 cov matrix) and the mean, both at once. \sa getMean */
		void getCovarianceAndMean(mrpt::math::CMatrixDouble77 &cov,CPose3DQuat &mean_point) const MRPT_OVERRIDE {
			cov_inv.inv(cov);
			mean_point = mean;
		}

		/** Returns the information (inverse covariance) matrix (a STATE_LEN x STATE_LEN matrix) \sa getMean, getCovarianceAndMean */
		void getInformationMatrix(mrpt::math::CMatrixDouble77 &inf) const MRPT_OVERRIDE { inf=cov_inv; }

		void copyFrom(const CPose3DQuatPDF &o) MRPT_OVERRIDE; //!< Copy operator, translating if necesary (for example, between particles and gaussian representations)

		/** Save the PDF to a text file, containing the 3D pose in the first line (x y z qr qx qy qz), then the information matrix in the next 7 lines. */
		void saveToTextFile(const std::string &file) const MRPT_OVERRIDE;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. */
		void  changeCoordinatesReference(  const CPose3DQuat &newReferenceBase );

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. */
		void changeCoordinatesReference(  const CPose3D &newReferenceBase ) MRPT_OVERRIDE;


		void drawSingleSample( CPose3DQuat &outPart ) const MRPT_OVERRIDE; //!< Draws a single sample from the distribution
		void drawManySamples( size_t N, std::vector<mrpt::math::CVectorDouble> & outSamples ) const MRPT_OVERRIDE; //!< Draws a number of samples from the distribution, and saves as a list of 1x7 vectors, where each row contains a (x,y,z,qr,qx,qy,qz) datum
		void inverse(CPose3DQuatPDF &o) const MRPT_OVERRIDE; //!< Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF

		/** Unary - operator, returns the PDF of the inverse pose.  */
		inline CPose3DQuatPDFGaussianInf operator -() const
		{
			CPose3DQuatPDFGaussianInf p(mrpt::math::UNINITIALIZED_QUATERNION);
			this->inverse(p);
			return p;
		}

		void operator += ( const CPose3DQuat &Ap); //!< Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated).
		void operator += ( const CPose3DQuatPDFGaussianInf &Ap); //!< Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated) (see formulas in jacobiansPoseComposition ).
		void operator -= ( const CPose3DQuatPDFGaussianInf &Ap);//!< Makes: thisPDF = thisPDF - Ap, where "-" is pose inverse composition (both the mean, and the covariance matrix are updated).

		double evaluatePDF( const CPose3DQuat &x ) const; //!< Evaluates the PDF at a given point
		double  evaluateNormalizedPDF( const CPose3DQuat &x ) const; //!< Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1]

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPose3DQuatPDFGaussianInf , CPose3DQuatPDF )

	bool BASE_IMPEXP operator==(const CPose3DQuatPDFGaussianInf &p1,const CPose3DQuatPDFGaussianInf &p2);
	/** Pose composition for two 3D pose Gaussians  \sa CPose3DQuatPDFGaussianInf::operator += */
	CPose3DQuatPDFGaussianInf BASE_IMPEXP operator +( const CPose3DQuatPDFGaussianInf &x, const CPose3DQuatPDFGaussianInf &u );
	/** Inverse pose composition for two 3D pose Gaussians  \sa CPose3DQuatPDFGaussianInf::operator -= */
	CPose3DQuatPDFGaussianInf BASE_IMPEXP operator -( const CPose3DQuatPDFGaussianInf &x, const CPose3DQuatPDFGaussianInf &u );

	/** Dumps the mean and covariance matrix to a text stream. */
	std::ostream  BASE_IMPEXP & operator << (std::ostream & out, const CPose3DQuatPDFGaussianInf& obj);

	} // End of namespace
} // End of namespace
#endif
