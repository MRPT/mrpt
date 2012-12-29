/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#ifndef CPose3DPDFGaussianInf_H
#define CPose3DPDFGaussianInf_H

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
		CMatrixDouble66		cov_inv;	//!< The inverse of the 6x6 covariance matrix

		/** @} */

		inline const CPose3D & getPoseMean() const { return mean; }
		inline       CPose3D & getPoseMean()       { return mean; }

		 /** Default constructor - mean: all zeros, inverse covariance=all zeros -> so be careful!
		  */
		CPose3DPDFGaussianInf();

		/** Constructor with a mean value, inverse covariance=all zeros -> so be careful! */
		explicit CPose3DPDFGaussianInf( const CPose3D &init_Mean );

		/** Uninitialized constructor: leave all fields uninitialized - Call with UNINITIALIZED_POSE as argument
		  */
		CPose3DPDFGaussianInf(TConstructorFlags_Poses constructor_dummy_param);

		/** Constructor with mean and inv cov. */
		CPose3DPDFGaussianInf( const CPose3D &init_Mean, const CMatrixDouble66 &init_CovInv );

		/** Constructor from a 6D pose PDF described as a Quaternion
		  */
		explicit CPose3DPDFGaussianInf( const CPose3DQuatPDFGaussian &o);

		 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF).
		   * \sa getCovariance
		   */
		void getMean(CPose3D &mean_pose) const {
			mean_pose = mean;
		}

		/** Returns an estimate of the pose covariance matrix (6x6 cov matrix) and the mean, both at once.
		  * \sa getMean
		  */
		void getCovarianceAndMean(CMatrixDouble66 &cov,CPose3D &mean_point) const {
			mean_point = this->mean;
			this->cov_inv.inv(cov);
		}

		/** Returns the information (inverse covariance) matrix (a STATE_LEN x STATE_LEN matrix) \sa getMean, getCovarianceAndMean */
		virtual void getInformationMatrix(CMatrixDouble66 &inf) const { inf=cov_inv; }

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		void  copyFrom(const CPose3DPDF &o);

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		void  copyFrom(const CPosePDF &o);

		/** Copy from a 6D pose PDF described as a Quaternion
		  */
		void copyFrom( const CPose3DQuatPDFGaussian &o);

		/** Save the PDF to a text file, containing the 3D pose in the first line, then the covariance matrix in next 3 lines.
		 */
		void  saveToTextFile(const std::string &file) const;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference(  const CPose3D &newReferenceBase );

		/** Draws a single sample from the distribution
		  */
		void  drawSingleSample( CPose3D &outPart ) const;

		/** Draws a number of samples from the distribution, and saves as a list of 1x6 vectors, where each row contains a (x,y,phi) datum.
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
		void  bayesianFusion( const CPose3DPDF &p1, const CPose3DPDF &p2 );

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
		  */
		void	 inverse(CPose3DPDF &o) const;

		/** Unary - operator, returns the PDF of the inverse pose.  */
		inline CPose3DPDFGaussianInf operator -() const
		{
			CPose3DPDFGaussianInf p(UNINITIALIZED_POSE);
			this->inverse(p);
			return p;
		}

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated).
		  */
		void  operator += ( const CPose3D &Ap);

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated).
		  */
		void  operator += ( const CPose3DPDFGaussianInf &Ap);

		/** Makes: thisPDF = thisPDF - Ap, where "-" is pose inverse composition (both the mean, and the covariance matrix are updated).
		  */
		void  operator -= ( const CPose3DPDFGaussianInf &Ap);

		/** Evaluates the PDF at a given point.
		  */
		double  evaluatePDF( const CPose3D &x ) const;

		/** Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1].
		  */
		double  evaluateNormalizedPDF( const CPose3D &x ) const;

		/** Computes the Mahalanobis distance between the centers of two Gaussians.
		  *  The variables with a variance exactly equal to 0 are not taken into account in the process, but
		  *   "infinity" is returned if the corresponding elements are not exactly equal.
		  */
		double  mahalanobisDistanceTo( const CPose3DPDFGaussianInf& theOther);

		/** Returns a 3x3 matrix with submatrix of the inverse covariance for the variables (x,y,yaw) only.
		  */
		void getInvCovSubmatrix2D( CMatrixDouble &out_cov ) const;

	}; // End of class def.


	/** Pose composition for two 3D pose Gaussians  \sa CPose3DPDFGaussian::operator +=  */
	inline CPose3DPDFGaussianInf operator +( const CPose3DPDFGaussianInf &x, const CPose3DPDFGaussianInf &u )
	{
		CPose3DPDFGaussianInf 	res(x);
		res+=u;
		return res;
	}

	/** Pose composition for two 3D pose Gaussians  \sa CPose3DPDFGaussianInf::operator -=  */
	inline CPose3DPDFGaussianInf operator -( const CPose3DPDFGaussianInf &x, const CPose3DPDFGaussianInf &u )
	{
		CPose3DPDFGaussianInf 	res(x);
		res-=u;
		return res;
	}

	/** Dumps the mean and covariance matrix to a text stream.
	  */
	std::ostream  BASE_IMPEXP & operator << (std::ostream & out, const CPose3DPDFGaussianInf& obj);

	bool BASE_IMPEXP operator==(const CPose3DPDFGaussianInf &p1,const CPose3DPDFGaussianInf &p2);

	} // End of namespace
} // End of namespace

#endif
