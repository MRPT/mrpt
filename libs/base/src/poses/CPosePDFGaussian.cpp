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

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/math/CMatrix.h>

#include <mrpt/math/utils.h>
#include <mrpt/math/distributions.h>

#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;

using namespace std;

IMPLEMENTS_SERIALIZABLE( CPosePDFGaussian, CPosePDF, mrpt::poses )


/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGaussian::CPosePDFGaussian() : mean(0,0,0), cov()
{

}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGaussian::CPosePDFGaussian(
	const CPose2D	&init_Mean,
	const CMatrixDouble33	&init_Cov ) : mean(init_Mean), cov(init_Cov)
{

}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFGaussian::CPosePDFGaussian(const CPose2D  &init_Mean ) : mean(init_Mean), cov()
{
	cov.zeros();
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPosePDFGaussian::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		out << mean;
		out << cov(0,0) << cov(1,1) << cov(2,2);
		out << cov(0,1) << cov(0,2) << cov(1,2);
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPosePDFGaussian::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 2:
		{
			double	x;
			in >> mean;

			in >> x; cov(0,0) = x;
			in >> x; cov(1,1) = x;
			in >> x; cov(2,2) = x;

			in >> x; cov(1,0) = x; cov(0,1) = x;
			in >> x; cov(2,0) = x; cov(0,2) = x;
			in >> x; cov(1,2) = x; cov(2,1) = x;
		} break;
	case 1:
		{
			float	x;
			in >> mean;

			in >> x; cov(0,0) = x;
			in >> x; cov(1,1) = x;
			in >> x; cov(2,2) = x;

			in >> x; cov(1,0) = x; cov(0,1) = x;
			in >> x; cov(2,0) = x; cov(0,2) = x;
			in >> x; cov(1,2) = x; cov(2,1) = x;
		} break;
	case 0:
		{
			CMatrix		auxCov;
			in >> mean >> auxCov;
			cov = auxCov.cast<double>();
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}



/*---------------------------------------------------------------
						copyFrom
  ---------------------------------------------------------------*/
void  CPosePDFGaussian::copyFrom(const CPosePDF &o)
{
	if (this == &o) return;		// It may be used sometimes

	// Convert to gaussian pdf:
	o.getMean(mean);
	o.getCovariance(cov);
}

/*---------------------------------------------------------------
						copyFrom 3D
  ---------------------------------------------------------------*/
void  CPosePDFGaussian::copyFrom(const CPose3DPDF &o)
{
	// Convert to gaussian pdf:
	mean = CPose2D(o.getMeanVal());
	CMatrixDouble66 C;
	o.getCovariance(C);

	// Clip to 3x3:
	C(2,0)=C(0,2) = C(0,3);
	C(2,1)=C(1,2) = C(1,3);
	C(2,2)=         C(3,3);
	cov = C.block(0,0,3,3);
}


/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
void  CPosePDFGaussian::saveToTextFile(const std::string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;

	os::fprintf(f,"%f %f %f\n", mean.x(), mean.y(), mean.phi() );

	os::fprintf(f,"%f %f %f\n", cov(0,0),cov(0,1),cov(0,2) );
	os::fprintf(f,"%f %f %f\n", cov(1,0),cov(1,1),cov(1,2) );
	os::fprintf(f,"%f %f %f\n", cov(2,0),cov(2,1),cov(2,2) );


	os::fclose(f);
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPosePDFGaussian::changeCoordinatesReference(const CPose3D &newReferenceBase_ )
{
	CPose2D newReferenceBase = CPose2D(newReferenceBase_);

	// The mean:
	mean = CPose2D( newReferenceBase + mean );

	// The covariance:
	rotateCov( newReferenceBase.phi() );
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPosePDFGaussian::changeCoordinatesReference(const CPose2D &newReferenceBase )
{
	// The mean:
	mean = newReferenceBase + mean;
	// The covariance:
	rotateCov( newReferenceBase.phi() );
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPosePDFGaussian::rotateCov(const double ang)
{
	const double ccos = cos(ang);
	const double ssin = sin(ang);

	EIGEN_ALIGN16 const double rot_vals[] = {
		ccos, -ssin, 0.,
		ssin, ccos,  0.,
		0.  ,   0.,  1. };

	const CMatrixFixedNumeric<double,3,3> rot(rot_vals);
	cov = (rot * cov * rot.adjoint()).eval();
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPosePDFGaussian::drawSingleSample( CPose2D &outPart ) const
{
	MRPT_START

	vector_double	v;
	randomGenerator.drawGaussianMultivariate(v,cov);

	outPart.x(  mean.x() + v[0] );
	outPart.y(  mean.y() + v[1] );
	outPart.phi( mean.phi() + v[2] );

	// Range -pi,pi
	outPart.normalizePhi();

	MRPT_END_WITH_CLEAN_UP( \
        cov.saveToTextFile("__DEBUG_EXC_DUMP_drawSingleSample_COV.txt"); \
		);
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void  CPosePDFGaussian::drawManySamples(
	size_t						N,
	std::vector<vector_double>	&outSamples ) const
{
	MRPT_START

	std::vector<vector_double>	rndSamples;

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
void  CPosePDFGaussian::bayesianFusion(const  CPosePDF &p1_,const  CPosePDF &p2_,const double& minMahalanobisDistToDrop )
{
	MRPT_START

	MRPT_UNUSED_PARAM(minMahalanobisDistToDrop); // Not used in this class!

	ASSERT_( p1_.GetRuntimeClass() == CLASS_ID( CPosePDFGaussian )  );
	ASSERT_( p2_.GetRuntimeClass() == CLASS_ID( CPosePDFGaussian )  );

	const CPosePDFGaussian	*p1 = static_cast<const CPosePDFGaussian*>( &p1_ );
	const CPosePDFGaussian	*p2 = static_cast<const CPosePDFGaussian*>( &p2_ );


	CMatrixDouble33	C1 = p1->cov;
	CMatrixDouble33	C2 = p2->cov;

	CMatrixDouble33	C1_inv;
	C1.inv(C1_inv);

	CMatrixDouble33	C2_inv;
	C2.inv(C2_inv);

	CMatrixDouble31	x1 = CMatrixDouble31(p1->mean);
	CMatrixDouble31	x2 = CMatrixDouble31(p2->mean);


	CMatrixDouble33	auxC = C1_inv + C2_inv;
	auxC.inv(this->cov);
	this->assureSymmetry();

	CMatrixDouble31	x = this->cov * ( C1_inv*x1 + C2_inv*x2 );

	this->mean.x( x(0,0) );
	this->mean.y( x(1,0) );
	this->mean.phi( x(2,0) );
	this->mean.normalizePhi();

	MRPT_END

}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void	 CPosePDFGaussian::inverse(CPosePDF &o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPosePDFGaussian));
	CPosePDFGaussian	*out = static_cast<CPosePDFGaussian*>( &o );

	// The mean:
	out->mean = CPose2D(0,0,0) - mean;

	// The covariance:
	const double ccos = ::cos(mean.phi());
	const double ssin = ::sin(mean.phi());

	// jacobian:
	EIGEN_ALIGN16 const double H_values[] = {
		-ccos, -ssin,  mean.x()*ssin-mean.y()*ccos,
		 ssin, -ccos,  mean.x()*ccos+mean.y()*ssin,
		 0   ,     0,  -1
		};
	const CMatrixFixedNumeric<double,3,3> H(H_values);

	out->cov.noalias() = (H * cov * H.adjoint()).eval();  // o.cov = H * cov * Ht
}


/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPosePDFGaussian::operator += ( const CPose2D &Ap)
{
	mean = mean + Ap;
	rotateCov( Ap.phi() );
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double  CPosePDFGaussian::evaluatePDF( const CPose2D &x ) const
{
	CMatrixDouble31	X = CMatrixDouble31	(x);
	CMatrixDouble31	MU = CMatrixDouble31(mean);

	return math::normalPDF( X, MU, this->cov );
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double  CPosePDFGaussian::evaluateNormalizedPDF( const CPose2D &x ) const
{
	CMatrixDouble31	X = CMatrixDouble31(x);
	CMatrixDouble31	MU = CMatrixDouble31(mean);

	return math::normalPDF( X, MU, this->cov ) / math::normalPDF( MU, MU, this->cov );
}

/*---------------------------------------------------------------
						assureSymmetry
 ---------------------------------------------------------------*/
void  CPosePDFGaussian::assureSymmetry()
{
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	cov(0,1) = cov(1,0);
	cov(0,2) = cov(2,0);
	cov(1,2) = cov(2,1);
}

/*---------------------------------------------------------------
						mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double  CPosePDFGaussian::mahalanobisDistanceTo( const CPosePDFGaussian& theOther )
{
	MRPT_START

	CArrayDouble<3> MU=CArrayDouble<3>(mean);
	MU-=CArrayDouble<3>(theOther.mean);

	wrapToPiInPlace(MU[2]);

	if (MU[0]==0 && MU[1]==0 && MU[2]==0)
		return 0; // This is the ONLY case where we know the result, whatever COVs are.

	CMatrixDouble33	COV_=cov;
	COV_+=theOther.cov;

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
	const CPosePDFGaussian	&obj )
{
	out << "Mean: " << obj.mean << "\n";
	out << "Covariance:\n" << obj.cov << "\n";

	return out;
}

/*---------------------------------------------------------------
						operator +
 ---------------------------------------------------------------*/
poses::CPosePDFGaussian	 operator + ( const mrpt::poses::CPose2D &A, const mrpt::poses::CPosePDFGaussian &B  )
{
	poses::CPosePDFGaussian	ret(B);
	ret.changeCoordinatesReference(A);
	return ret;
}

/*---------------------------------------------------------------
						assureMinCovariance
 ---------------------------------------------------------------*/
void  CPosePDFGaussian::assureMinCovariance( const double& minStdXY, const double&minStdPhi )
{
	cov(0,0) = max( cov(0,0), square(minStdXY) );
	cov(1,1) = max( cov(1,1), square(minStdXY) );
	cov(2,2) = max( cov(2,2), square(minStdPhi) );
}

/*---------------------------------------------------------------
						inverseComposition
  Set 'this' = 'x' - 'ref', computing the mean using the "-"
    operator and the covariances through the corresponding Jacobians.
 ---------------------------------------------------------------*/
void CPosePDFGaussian::inverseComposition(
	const CPosePDFGaussian &xv,
	const CPosePDFGaussian &xi  )
{
	// COV:
	double cpi = cos(xi.mean.phi());
	double spi = sin(xi.mean.phi());

	// jacob: dh_xv
	CMatrixDouble33 dh_xv;

	dh_xv(0,0) =  cpi; dh_xv(0,1) = spi;
	dh_xv(1,0) = -spi; dh_xv(1,1) = cpi;
	dh_xv(2,2) = 1;

	// jacob: dh_xi
	CMatrixDouble33 dh_xi;

	double xv_xi = xv.mean.x() - xi.mean.x();
	double yv_yi = xv.mean.y() - xi.mean.y();

	dh_xi(0,0) = -cpi;  dh_xi(0,1) = -spi;  dh_xi(0,2) = -xv_xi*spi+yv_yi*cpi;
	dh_xi(1,0) =  spi;  dh_xi(1,1) = -cpi;  dh_xi(1,2) = -xv_xi*cpi-yv_yi*spi;
	dh_xi(2,2) = -1;

	// Build the cov:
	//  Y = dh_xv * XV * dh_xv^T  + dh_xi * XI * dh_xi^T
	dh_xv.multiply_HCHt( xv.cov, this->cov );
	dh_xi.multiply_HCHt( xi.cov, this->cov, true ); // Accum. result

	// Mean:
	mean = xv.mean - xi.mean;
}


/*---------------------------------------------------------------
						inverseComposition
  Set \f$ this = x1 \ominus x0 \f$ , computing the mean using
   the "-" operator and the covariances through the corresponding
    Jacobians (Given the 3x3 cross-covariance matrix of variables x0 and x0).
 ---------------------------------------------------------------*/
void CPosePDFGaussian::inverseComposition(
	const CPosePDFGaussian &x1,
	const CPosePDFGaussian &x0,
	const CMatrixDouble33  &COV_01 )
{
	double cp0 = cos(x0.mean.phi());
	double sp0 = sin(x0.mean.phi());

	// jacob: dh_x1
	CMatrixDouble33 dh_x1;

	dh_x1(0,0) =  cp0; dh_x1(0,1) = sp0;
	dh_x1(1,0) = -sp0; dh_x1(1,1) = cp0;
	dh_x1(2,2) = 1;

	// jacob: dh_x0
	CMatrixDouble33 dh_x0;

	double xv_xi = x1.mean.x() - x0.mean.x();
	double yv_yi = x1.mean.y() - x0.mean.y();

	dh_x0(0,0) = -cp0;  dh_x0(0,1) = -sp0;  dh_x0(0,2) = -xv_xi*sp0+yv_yi*cp0;
	dh_x0(1,0) =  sp0;  dh_x0(1,1) = -cp0;  dh_x0(1,2) = -xv_xi*cp0-yv_yi*sp0;
	dh_x0(2,2) = -1;

	// Build the cov:
	//  Y = dh_xv * XV * dh_xv^T  + dh_xi * XI * dh_xi^T +  A + At
	//     being A = dh_dx0 * COV01 * dh_dx1^t
	dh_x0.multiply_HCHt( x0.cov, this->cov );
	dh_x1.multiply_HCHt( x1.cov, this->cov, true ); // Accum. result

	CMatrixDouble33 M;
	M.multiply_ABCt( dh_x0, COV_01, dh_x1 );

	this->cov.add_AAt(M);

	//mean
	mean = x1.mean - x0.mean;
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPosePDFGaussian::operator += ( const CPosePDFGaussian &Ap)
{
	// COV:
	const CMatrixDouble33  OLD_COV = this->cov;
	CMatrixDouble33  df_dx, df_du;

	CPosePDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap.mean,     // u
		df_dx,
		df_du );

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	df_dx.multiply_HCHt( OLD_COV, cov );
	df_du.multiply_HCHt( Ap.cov,  cov, true); // Accumulate result

	// MEAN:
	this->mean = this->mean + Ap.mean;
}

bool mrpt::poses::operator==(const CPosePDFGaussian &p1,const CPosePDFGaussian &p2)
{
	return p1.mean==p1.mean && p1.cov==p2.cov;
}
