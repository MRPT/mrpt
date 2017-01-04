/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;

using namespace std;

IMPLEMENTS_SERIALIZABLE( CPosePDFGaussianInf, CPosePDF, mrpt::poses )


/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGaussianInf::CPosePDFGaussianInf() : mean(0,0,0), cov_inv()
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGaussianInf::CPosePDFGaussianInf(
	const CPose2D	&init_Mean,
	const CMatrixDouble33	&init_CovInv ) : mean(init_Mean), cov_inv(init_CovInv)
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGaussianInf::CPosePDFGaussianInf(const CPose2D  &init_Mean ) : mean(init_Mean), cov_inv()
{
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << mean.x() << mean.y() << mean.phi();
		out << cov_inv(0,0) << cov_inv(1,1) << cov_inv(2,2);
		out << cov_inv(0,1) << cov_inv(0,2) << cov_inv(1,2);
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			TPose2D p;
			in >> p.x >> p.y >> p.phi;
			mean = p;

			in >> cov_inv(0,0) >> cov_inv(1,1) >> cov_inv(2,2);
			in >> cov_inv(0,1) >> cov_inv(0,2) >> cov_inv(1,2);
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}


/*---------------------------------------------------------------
						copyFrom
  ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::copyFrom(const CPosePDF &o)
{
	if (this == &o) return;		// It may be used sometimes

	if (IS_CLASS(&o, CPosePDFGaussianInf))
	{	// It's my same class:
		const CPosePDFGaussianInf *ptr = static_cast<const CPosePDFGaussianInf*>(&o);
		mean    = ptr->mean;
		cov_inv = ptr->cov_inv;
	}
	else
	{	// Convert to gaussian pdf:
		o.getMean(mean);

		CMatrixDouble33 o_cov(UNINITIALIZED_MATRIX);
		o.getCovariance(o_cov);
		o_cov.inv_fast(this->cov_inv);
	}
}

/*---------------------------------------------------------------
						copyFrom 3D
  ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::copyFrom(const CPose3DPDF &o)
{
	// Convert to gaussian pdf:
	mean = CPose2D(o.getMeanVal());

	if (IS_CLASS(&o, CPose3DPDFGaussianInf))
	{	// Cov is already in information form:
		const CPose3DPDFGaussianInf *ptr = static_cast<const CPose3DPDFGaussianInf*>(&o);
		cov_inv(0,0)=ptr->cov_inv(0,0);
		cov_inv(1,1)=ptr->cov_inv(1,1);
		cov_inv(2,2)=ptr->cov_inv(3,3);
		cov_inv(0,1)=cov_inv(1,0)=ptr->cov_inv(0,1);
		cov_inv(0,2)=cov_inv(2,0)=ptr->cov_inv(0,3);
		cov_inv(1,2)=cov_inv(2,1)=ptr->cov_inv(1,3);
	}
	else
	{
		CMatrixDouble66 C(UNINITIALIZED_MATRIX);
		o.getCovariance(C);

		// Clip to 3x3:
		CMatrixDouble33 o_cov(UNINITIALIZED_MATRIX);
		o_cov(0,0)=C(0,0);
		o_cov(1,1)=C(1,1);
		o_cov(2,2)=C(3,3);
		o_cov(0,1)=o_cov(1,0)=C(0,1);
		o_cov(0,2)=o_cov(2,0)=C(0,3);
		o_cov(1,2)=o_cov(2,1)=C(1,3);

		o_cov.inv_fast(this->cov_inv);
	}
}


/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::saveToTextFile(const std::string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;

	os::fprintf(f,"%f %f %f\n", mean.x(), mean.y(), mean.phi() );

	for (unsigned int i=0;i<3;i++)
		os::fprintf(f,"%f %f %f\n", cov_inv(i,0),cov_inv(i,1),cov_inv(i,2) );

	os::fclose(f);
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::changeCoordinatesReference(const CPose3D &newReferenceBase_ )
{
	const CPose2D newReferenceBase = CPose2D(newReferenceBase_);

	// The mean:
	mean.composeFrom(newReferenceBase, mean);

	// The covariance:
	rotateCov( newReferenceBase.phi() );
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::changeCoordinatesReference(const CPose2D &newReferenceBase )
{
	// The mean:
	mean.composeFrom(newReferenceBase, mean);
	// The covariance:
	rotateCov( newReferenceBase.phi() );
}


/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::rotateCov(const double ang)
{
	const double ccos = cos(ang);
	const double ssin = sin(ang);

	MRPT_ALIGN16 const double rot_vals[] = {
		ccos, -ssin, 0.,
		ssin, ccos,  0.,
		0.  ,   0.,  1. };

	const CMatrixFixedNumeric<double,3,3> rot(rot_vals);

	// NEW_COV = H C H^T
	// NEW_COV^(-1) = (H C H^T)^(-1) = (H^T)^(-1) C^(-1) H^(-1)
	// rot: Inverse of a rotation matrix is its trasposed.
	//      But we need H^t^-1 -> H !! so rot stays unchanged:
	cov_inv = (rot * cov_inv * rot.adjoint()).eval();
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::drawSingleSample( CPose2D &outPart ) const
{
	MRPT_START

	CMatrixDouble33 cov(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(cov);

	CVectorDouble	v;
	randomGenerator.drawGaussianMultivariate(v,cov);

	outPart.x(  mean.x() + v[0] );
	outPart.y(  mean.y() + v[1] );
	outPart.phi( mean.phi() + v[2] );

	// Range -pi,pi
	outPart.normalizePhi();

	MRPT_END_WITH_CLEAN_UP( \
        cov_inv.saveToTextFile("__DEBUG_EXC_DUMP_drawSingleSample_COV_INV.txt"); \
		);
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::drawManySamples(
	size_t						N,
	std::vector<CVectorDouble>	&outSamples ) const
{
	MRPT_START

	CMatrixDouble33 cov(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(cov);

	std::vector<CVectorDouble>	rndSamples;

	randomGenerator.drawGaussianMultivariateMany(rndSamples,N,cov);
	outSamples.resize( N );
	for (unsigned int i=0;i<N;i++)
	{
		outSamples[i].resize(3);
		outSamples[i][0] = mean.x() + rndSamples[i][0] ;
		outSamples[i][1] = mean.y() + rndSamples[i][1] ;
		outSamples[i][2] = mean.phi() + rndSamples[i][2] ;

		wrapToPiInPlace( outSamples[i][2] );
	}

	MRPT_END
}


/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::bayesianFusion(const  CPosePDF &p1_,const  CPosePDF &p2_,const double& minMahalanobisDistToDrop )
{
	MRPT_START

	MRPT_UNUSED_PARAM(minMahalanobisDistToDrop); // Not used in this class!

	ASSERT_( p1_.GetRuntimeClass() == CLASS_ID( CPosePDFGaussianInf )  );
	ASSERT_( p2_.GetRuntimeClass() == CLASS_ID( CPosePDFGaussianInf )  );

	const CPosePDFGaussianInf	*p1 = static_cast<const CPosePDFGaussianInf*>( &p1_ );
	const CPosePDFGaussianInf	*p2 = static_cast<const CPosePDFGaussianInf*>( &p2_ );

	const CMatrixDouble33& C1_inv = p1->cov_inv;
	const CMatrixDouble33& C2_inv = p2->cov_inv;

	CMatrixDouble31	x1 = CMatrixDouble31(p1->mean);
	CMatrixDouble31	x2 = CMatrixDouble31(p2->mean);

	this->cov_inv = C1_inv + C2_inv;

	CMatrixDouble33 cov(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(cov);

	CMatrixDouble31	x = cov * ( C1_inv*x1 + C2_inv*x2 );

	this->mean.x( x(0,0) );
	this->mean.y( x(1,0) );
	this->mean.phi( x(2,0) );
	this->mean.normalizePhi();

	MRPT_END

}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void	 CPosePDFGaussianInf::inverse(CPosePDF &o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPosePDFGaussianInf));
	CPosePDFGaussianInf	*out = static_cast<CPosePDFGaussianInf*>( &o );

	// The mean:
	out->mean = CPose2D(0,0,0) - mean;

	// The covariance:
	const double ccos = ::cos(mean.phi());
	const double ssin = ::sin(mean.phi());

	// jacobian:
	MRPT_ALIGN16 const double H_values[] = {
		-ccos, -ssin,  mean.x()*ssin-mean.y()*ccos,
		 ssin, -ccos,  mean.x()*ccos+mean.y()*ssin,
		 0   ,     0,  -1
		};
	const CMatrixFixedNumeric<double,3,3> H(H_values);

	out->cov_inv.noalias() = (H * cov_inv * H.adjoint()).eval();  // o.cov = H * cov * Ht. It's the same with inverse covariances.
}


/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::operator += ( const CPose2D &Ap)
{
	mean = mean + Ap;
	rotateCov( Ap.phi() );
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double  CPosePDFGaussianInf::evaluatePDF( const CPose2D &x ) const
{
	CMatrixDouble31	X = CMatrixDouble31(x);
	CMatrixDouble31	MU = CMatrixDouble31(mean);

	return math::normalPDF( X, MU, cov_inv.inverse() );
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double  CPosePDFGaussianInf::evaluateNormalizedPDF( const CPose2D &x ) const
{
	CMatrixDouble31	X = CMatrixDouble31(x);
	CMatrixDouble31	MU = CMatrixDouble31(mean);

	CMatrixDouble33 cov(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(cov);

	return math::normalPDF( X, MU, cov ) / math::normalPDF( MU, MU, cov );
}

/*---------------------------------------------------------------
						assureSymmetry
 ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::assureSymmetry()
{
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	cov_inv(0,1) = cov_inv(1,0);
	cov_inv(0,2) = cov_inv(2,0);
	cov_inv(1,2) = cov_inv(2,1);
}

/*---------------------------------------------------------------
						mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double  CPosePDFGaussianInf::mahalanobisDistanceTo( const CPosePDFGaussianInf& theOther )
{
	MRPT_START

	CArrayDouble<3> MU=CArrayDouble<3>(mean);
	MU-=CArrayDouble<3>(theOther.mean);

	wrapToPiInPlace(MU[2]);

	if (MU[0]==0 && MU[1]==0 && MU[2]==0)
		return 0; // This is the ONLY case where we know the result, whatever COVs are.

	CMatrixDouble33	COV_(UNINITIALIZED_MATRIX), cov2(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(COV_);
	theOther.cov_inv.inv(cov2);

	COV_+=cov2; // COV_ = cov1+cov2

	CMatrixDouble33 COV_inv(UNINITIALIZED_MATRIX);
	COV_.inv_fast(COV_inv);

	// (~MU) * (!COV_) * MU
	return std::sqrt( mrpt::math::multiply_HCHt_scalar(MU,COV_inv) );

	MRPT_END
}

/*---------------------------------------------------------------
						operator <<
 ---------------------------------------------------------------*/
std::ostream &   mrpt::poses::operator << (
	std::ostream		&out,
	const CPosePDFGaussianInf	&obj )
{
	out << "Mean: " << obj.mean << "\n";
	out << "Inverse cov:\n" << obj.cov_inv << "\n";

	return out;
}

/*---------------------------------------------------------------
						operator +
 ---------------------------------------------------------------*/
poses::CPosePDFGaussianInf	 operator + ( const mrpt::poses::CPose2D &A, const mrpt::poses::CPosePDFGaussianInf &B  )
{
	poses::CPosePDFGaussianInf	ret(B);
	ret.changeCoordinatesReference(A);
	return ret;
}

/*---------------------------------------------------------------
						inverseComposition
  Set 'this' = 'x' - 'ref', computing the mean using the "-"
    operator and the covariances through the corresponding Jacobians.
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::inverseComposition(
	const CPosePDFGaussianInf &xv,
	const CPosePDFGaussianInf &xi  )
{
	// Use implementation in CPosePDFGaussian:
	CMatrixDouble33 xv_cov(UNINITIALIZED_MATRIX), xi_cov(UNINITIALIZED_MATRIX);
	xv.cov_inv.inv(xv_cov);
	xi.cov_inv.inv(xi_cov);

	const CPosePDFGaussian  xv_(xv.mean,xv_cov);
	const CPosePDFGaussian  xi_(xi.mean,xi_cov);

	CPosePDFGaussian  RET;
	RET.inverseComposition(xv_,xi_);

	// Copy result to "this":
	this->mean = RET.mean;
	RET.cov.inv(this->cov_inv);
}

/*---------------------------------------------------------------
						inverseComposition
  Set \f$ this = x1 \ominus x0 \f$ , computing the mean using
   the "-" operator and the covariances through the corresponding
    Jacobians (Given the 3x3 cross-covariance matrix of variables x0 and x0).
 ---------------------------------------------------------------*/
void CPosePDFGaussianInf::inverseComposition(
	const CPosePDFGaussianInf &x1,
	const CPosePDFGaussianInf &x0,
	const CMatrixDouble33  &COV_01 )
{
	// Use implementation in CPosePDFGaussian:
	CMatrixDouble33 x1_cov(UNINITIALIZED_MATRIX), x0_cov(UNINITIALIZED_MATRIX);
	x1.cov_inv.inv(x1_cov);
	x0.cov_inv.inv(x0_cov);

	const CPosePDFGaussian  x1_(x1.mean,x1_cov);
	const CPosePDFGaussian  x0_(x0.mean,x0_cov);

	CPosePDFGaussian  RET;
	RET.inverseComposition(x1_,x0_,COV_01);

	// Copy result to "this":
	this->mean = RET.mean;
	RET.cov.inv(this->cov_inv);
}


/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPosePDFGaussianInf::operator += ( const CPosePDFGaussianInf &Ap)
{
	// COV:
	CMatrixDouble33  OLD_COV(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(OLD_COV);

	CMatrixDouble33  df_dx, df_du;

	CPosePDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap.mean,     // u
		df_dx,
		df_du );

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	CMatrixDouble33 cov;

	CMatrixDouble33 Ap_cov(UNINITIALIZED_MATRIX);
	Ap.cov_inv.inv(Ap_cov);

	df_dx.multiply_HCHt( OLD_COV, cov );
	df_du.multiply_HCHt( Ap_cov,  cov, true); // Accumulate result

	cov.inv_fast(this->cov_inv);

	// MEAN:
	this->mean = this->mean + Ap.mean;
}

bool mrpt::poses::operator==(const CPosePDFGaussianInf &p1,const CPosePDFGaussianInf &p2)
{
	return p1.mean==p2.mean && p1.cov_inv==p2.cov_inv;
}

/** Pose compose operator: RES = A (+) B , computing both the mean and the covariance */
CPosePDFGaussianInf mrpt::poses::operator +( const CPosePDFGaussianInf &a, const CPosePDFGaussianInf &b  ) {
	CPosePDFGaussianInf res(a);
	res+=b;
	return res;
}

/** Pose inverse compose operator: RES = A (-) B , computing both the mean and the covariance */
CPosePDFGaussianInf mrpt::poses::operator -( const CPosePDFGaussianInf &a, const CPosePDFGaussianInf &b  ) {
	CPosePDFGaussianInf res;
	res.inverseComposition(a,b);
	return res;
}

