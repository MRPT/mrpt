/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
		CPose3DQuatPDFGaussianInf(TConstructorFlags_Quaternions constructor_dummy_param);

		/** Constructor from a default mean value, information matrix equals to zero. */
		explicit CPose3DQuatPDFGaussianInf( const CPose3DQuat &init_Mean );

		/** Constructor with mean and inverse covariance (information matrix). */
		CPose3DQuatPDFGaussianInf( const CPose3DQuat &init_Mean, const CMatrixDouble77 &init_CovInv );

		/** The mean value */
		CPose3DQuat		mean;

		/** The 7x7 information matrix (the inverse of the covariance) */
		CMatrixDouble77		cov_inv;

		inline const CPose3DQuat & getPoseMean() const { return mean; }
		inline       CPose3DQuat & getPoseMean()       { return mean; }

		 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF).
		   * \sa getCovariance */
		void getMean(CPose3DQuat &mean_pose) const {
			mean_pose = mean;
		}

		/** Returns an estimate of the pose covariance matrix (7x7 cov matrix) and the mean, both at once.
		  * \sa getMean */
		void getCovarianceAndMean(CMatrixDouble77 &cov,CPose3DQuat &mean_point) const {
			cov_inv.inv(cov);
			mean_point = mean;
		}

		/** Returns the information (inverse covariance) matrix (a STATE_LEN x STATE_LEN matrix) \sa getMean, getCovarianceAndMean */
		virtual void getInformationMatrix(CMatrixDouble77 &inf) const { inf=cov_inv; }


		/** Copy operator, translating if necesary (for example, between particles and gaussian representations) */
		void  copyFrom(const CPose3DQuatPDF &o);

		/** Save the PDF to a text file, containing the 3D pose in the first line (x y z qr qx qy qz), then the information matrix in the next 7 lines. */
		void  saveToTextFile(const std::string &file) const;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference(  const CPose3DQuat &newReferenceBase );

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference(  const CPose3D &newReferenceBase );

		/** Draws a single sample from the distribution */
		void  drawSingleSample( CPose3DQuat &outPart ) const;

		/** Draws a number of samples from the distribution, and saves as a list of 1x7 vectors, where each row contains a (x,y,z,qr,qx,qy,qz) datum. */
		void  drawManySamples( size_t N, std::vector<vector_double> & outSamples ) const;

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF */
		void	 inverse(CPose3DQuatPDF &o) const;

		/** Unary - operator, returns the PDF of the inverse pose.  */
		inline CPose3DQuatPDFGaussianInf operator -() const
		{
			CPose3DQuatPDFGaussianInf p(UNINITIALIZED_QUATERNION);
			this->inverse(p);
			return p;
		}

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated).
		  */
		void  operator += ( const CPose3DQuat &Ap);

		/** Makes: thisPDF = thisPDF + Ap, where "+" is pose composition (both the mean, and the covariance matrix are updated) (see formulas in jacobiansPoseComposition ).
		  */
		void  operator += ( const CPose3DQuatPDFGaussianInf &Ap);

		/** Makes: thisPDF = thisPDF - Ap, where "-" is pose inverse composition (both the mean, and the covariance matrix are updated).
		  */
		void  operator -= ( const CPose3DQuatPDFGaussianInf &Ap);

		/** Evaluates the PDF at a given point.
		  */
		double  evaluatePDF( const CPose3DQuat &x ) const;

		/** Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in the range [0,1].
		  */
		double  evaluateNormalizedPDF( const CPose3DQuat &x ) const;

	}; // End of class def.


	/** Pose composition for two 3D pose Gaussians  \sa CPose3DQuatPDFGaussianInf::operator += */
	inline CPose3DQuatPDFGaussianInf operator +( const CPose3DQuatPDFGaussianInf &x, const CPose3DQuatPDFGaussianInf &u )
	{
		CPose3DQuatPDFGaussianInf 	res(x);
		res+=u;
		return res;
	}

	/** Inverse pose composition for two 3D pose Gaussians  \sa CPose3DQuatPDFGaussianInf::operator -= */
	inline CPose3DQuatPDFGaussianInf operator -( const CPose3DQuatPDFGaussianInf &x, const CPose3DQuatPDFGaussianInf &u )
	{
		CPose3DQuatPDFGaussianInf 	res(x);
		res-=u;
		return res;
	}

	/** Dumps the mean and covariance matrix to a text stream.
	  */
	std::ostream  BASE_IMPEXP & operator << (std::ostream & out, const CPose3DQuatPDFGaussianInf& obj);

	bool BASE_IMPEXP operator==(const CPose3DQuatPDFGaussianInf &p1,const CPose3DQuatPDFGaussianInf &p2);

	} // End of namespace

} // End of namespace

#endif
