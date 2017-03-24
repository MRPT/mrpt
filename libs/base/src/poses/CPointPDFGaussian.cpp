/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE( CPointPDFGaussian, CPointPDF, mrpt::poses )


/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPointPDFGaussian::CPointPDFGaussian() : mean(0,0,0), cov()
{

}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPointPDFGaussian::CPointPDFGaussian(
	const CPoint3D	&init_Mean,
	const CMatrixDouble33 &init_Cov ) : mean(init_Mean), cov(init_Cov)
{

}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPointPDFGaussian::CPointPDFGaussian(
	const CPoint3D	&init_Mean ) : mean(init_Mean), cov()
{
	cov.zeros();
}


/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF)
 ---------------------------------------------------------------*/
void CPointPDFGaussian::getMean(CPoint3D &p) const
{
	p=mean;
}

/*---------------------------------------------------------------
						getCovarianceAndMean
 ---------------------------------------------------------------*/
void CPointPDFGaussian::getCovarianceAndMean(CMatrixDouble33 &C,CPoint3D &p) const
{
	p=mean;
	C=cov;
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPointPDFGaussian::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << CPoint3D(mean) << cov;
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPointPDFGaussian::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			CPoint3D m;
			in >> m;
			mean = TPoint3D(m);

			CMatrix c;
			in >> c; cov = c.cast<double>();
		} break;
	case 1:
		{
			in >> mean >> cov;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


void  CPointPDFGaussian::copyFrom(const CPointPDF &o)
{
	if (this == &o) return;		// It may be used sometimes

	// Convert to gaussian pdf:
	o.getCovarianceAndMean(cov,mean);
}

/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
void  CPointPDFGaussian::saveToTextFile(const std::string &file) const
{
	MRPT_START

	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;

	os::fprintf(f,"%f %f %f\n", mean.x(), mean.y(), mean.z() );

	os::fprintf(f,"%f %f %f\n", cov(0,0),cov(0,1),cov(0,2) );
	os::fprintf(f,"%f %f %f\n", cov(1,0),cov(1,1),cov(1,2) );
	os::fprintf(f,"%f %f %f\n", cov(2,0),cov(2,1),cov(2,2) );


	os::fclose(f);

	MRPT_END
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPointPDFGaussian::changeCoordinatesReference(const CPose3D &newReferenceBase )
{
	const CMatrixDouble33 &M = newReferenceBase.getRotationMatrix();

	// The mean:
	mean = newReferenceBase + mean;

	// The covariance:
	M.multiply_HCHt(CMatrixDouble33(cov), cov); // save in cov
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPointPDFGaussian::bayesianFusion( const CPointPDFGaussian &p1, const CPointPDFGaussian &p2 )
{
	MRPT_START

	CMatrixDouble	x1(3,1),x2(3,1),x(3,1);
	CMatrixDouble	C1( p1.cov );
	CMatrixDouble	C2( p2.cov );
	CMatrixDouble	C1_inv = C1.inv();
	CMatrixDouble	C2_inv = C2.inv();
	CMatrixDouble	C;

	x1(0,0) = p1.mean.x(); x1(1,0) = p1.mean.y(); x1(2,0) = p1.mean.z();
	x2(0,0) = p2.mean.x(); x2(1,0) = p2.mean.y(); x2(2,0) = p2.mean.z();


	C = (C1_inv + C2_inv).inv();
	cov = C;

	x = C * ( C1_inv*x1 + C2_inv*x2 );

	mean.x( x(0,0) );
	mean.y( x(1,0) );
	mean.z( x(2,0) );

//	std::cout << "IN1: " << x1 << "\n" << C1 << "\n";
//	std::cout << "IN2: " << x2 << "\n" << C2 << "\n";
//	std::cout << "OUT: " << x << "\n" << C << "\n";

	MRPT_END
}

/*---------------------------------------------------------------
					productIntegralWith
 ---------------------------------------------------------------*/
double  CPointPDFGaussian::productIntegralWith( const CPointPDFGaussian &p) const
{
	MRPT_START

	// --------------------------------------------------------------
	// 12/APR/2009 - Jose Luis Blanco:
	//  The integral over all the variable space of the product of two
	//   Gaussians variables amounts to simply the evaluation of
	//   a normal PDF at (0,0), with mean=M1-M2 and COV=COV1+COV2
	// ---------------------------------------------------------------
	CMatrixDouble33 C = cov; C+=p.cov;	// Sum of covs:
	CMatrixDouble33 C_inv;
	C.inv(C_inv);

	CMatrixDouble31	MU(UNINITIALIZED_MATRIX); // Diff. of means
	MU.get_unsafe(0,0) = mean.x() - p.mean.x();
	MU.get_unsafe(1,0) = mean.y() - p.mean.y();
	MU.get_unsafe(2,0) = mean.z() - p.mean.z();

	return std::pow( M_2PI, -0.5*state_length )
		* (1.0/std::sqrt( C.det() ))
		* exp( -0.5* MU.multiply_HtCH_scalar(C_inv) );

	MRPT_END
}

/*---------------------------------------------------------------
					productIntegralWith2D
 ---------------------------------------------------------------*/
double  CPointPDFGaussian::productIntegralWith2D( const CPointPDFGaussian &p) const
{
	MRPT_START

	// --------------------------------------------------------------
	// 12/APR/2009 - Jose Luis Blanco:
	//  The integral over all the variable space of the product of two
	//   Gaussians variables amounts to simply the evaluation of
	//   a normal PDF at (0,0), with mean=M1-M2 and COV=COV1+COV2
	// ---------------------------------------------------------------
	CMatrixDouble22 C = cov.block(0,0,2,2);
	C+=p.cov.block(0,0,2,2);	// Sum of covs:

	CMatrixDouble22 C_inv;
	C.inv(C_inv);

	CMatrixDouble21	MU(UNINITIALIZED_MATRIX); // Diff. of means
	MU.get_unsafe(0,0) = mean.x() - p.mean.x();
	MU.get_unsafe(1,0) = mean.y() - p.mean.y();

	return std::pow( M_2PI, -0.5*(state_length-1) )
		* (1.0/std::sqrt( C.det() ))
		* exp( -0.5* MU.multiply_HtCH_scalar(C_inv) );

	MRPT_END
}

/*---------------------------------------------------------------
					productIntegralNormalizedWith
 ---------------------------------------------------------------*/
double  CPointPDFGaussian::productIntegralNormalizedWith( const CPointPDFGaussian &p) const
{
	return std::exp( -0.5*square(mahalanobisDistanceTo(p)) );
}

/*---------------------------------------------------------------
					productIntegralNormalizedWith
 ---------------------------------------------------------------*/
double  CPointPDFGaussian::productIntegralNormalizedWith2D( const CPointPDFGaussian &p) const
{
	return std::exp( -0.5*square(mahalanobisDistanceTo(p,true)) );
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPointPDFGaussian::drawSingleSample(CPoint3D &outSample) const
{
	MRPT_START

	CVectorDouble vec;
	randomGenerator.drawGaussianMultivariate(vec,cov);

	ASSERT_(vec.size()==3);
	outSample.x( mean.x() + vec[0] );
	outSample.y( mean.y() + vec[1] );
	outSample.z( mean.z() + vec[2] );

	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPointPDFGaussian::bayesianFusion( const CPointPDF &p1_, const CPointPDF &p2_,const double &minMahalanobisDistToDrop )
{
	MRPT_UNUSED_PARAM(minMahalanobisDistToDrop);
	MRPT_START

	// p1: CPointPDFGaussian, p2: CPosePDFGaussian:
	ASSERT_( p1_.GetRuntimeClass() == CLASS_ID(CPointPDFGaussian) );
	ASSERT_( p2_.GetRuntimeClass() == CLASS_ID(CPointPDFGaussian) );

	THROW_EXCEPTION("TODO!!!");

	MRPT_END
}

/*---------------------------------------------------------------
					mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double CPointPDFGaussian::mahalanobisDistanceTo( const CPointPDFGaussian & other, bool only_2D  ) const
{
	// The difference in means:
	CMatrixDouble13 deltaX;
	deltaX.get_unsafe(0,0) = other.mean.x() - mean.x();
	deltaX.get_unsafe(0,1) = other.mean.y() - mean.y();
	deltaX.get_unsafe(0,2) = other.mean.z() - mean.z();

	// The inverse of the combined covs:
	CMatrixDouble33 COV = other.cov;
	COV += this->cov;

	if (!only_2D)
	{
		CMatrixDouble33 COV_inv;
		COV.inv(COV_inv);
		return sqrt( deltaX.multiply_HCHt_scalar(COV_inv) );
	}
	else
	{
		CMatrixDouble22 C = COV.block(0,0,2,2);
		CMatrixDouble22 COV_inv;
		C.inv(COV_inv);
		CMatrixDouble12 deltaX2 = deltaX.block(0,0,1,2);
		return std::sqrt( deltaX2.multiply_HCHt_scalar(COV_inv) );
	}


}
