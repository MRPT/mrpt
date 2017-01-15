/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/random.h>
#include <mrpt/math/transform_gaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CStream.h>

#include <sstream>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

bool mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION = false;


IMPLEMENTS_SERIALIZABLE( CPose3DPDFGaussian, CPose3DPDF, mrpt::poses )


/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussian::CPose3DPDFGaussian() : mean(0,0,0), cov()
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussian::CPose3DPDFGaussian(TConstructorFlags_Poses constructor_dummy_param) : mean(UNINITIALIZED_POSE), cov(UNINITIALIZED_MATRIX)
{
	MRPT_UNUSED_PARAM(constructor_dummy_param);
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussian::CPose3DPDFGaussian( const CPose3D &init_Mean, const CMatrixDouble66 &init_Cov ) :
	mean(init_Mean), cov(init_Cov)
{
}

/*---------------------------------------------------------------
	Copy Constructor from 2D PDF
  ---------------------------------------------------------------*/
CPose3DPDFGaussian::CPose3DPDFGaussian( const CPosePDFGaussian &o )
	 : mean( o.mean.x(),o.mean.y(),0,o.mean.phi(),0,0 ),
	   cov()
{
	for (size_t i=0;i<3;i++)
	{
		const size_t ii= (i==2) ? 3 : i;
		for (size_t j=0;j<3;j++)
		{
			const size_t jj= (j==2) ? 3 : j;
			cov(ii,jj) = o.cov(i,j);
		}
	}
}


/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussian::CPose3DPDFGaussian(
	const CPose3D	&init_Mean ) : mean(init_Mean), cov()
{
}


//#define DO_TEST_JACOB

#ifdef DO_TEST_JACOB
void ffff(const CVectorDouble &x,const CQuaternionDouble &Q, CVectorDouble &OUT)
{
	OUT.resize(3);
	CQuaternionDouble q(x[0],x[1],x[2],x[3]);
	q.normalize();
	q.rpy(OUT[2],OUT[1],OUT[0]);
}
#endif

void aux_posequat2poseypr(const CArrayDouble<7> &x,const double&dummy, CArrayDouble<6> &y)
{
	MRPT_UNUSED_PARAM(dummy);
	y[0]=x[0]; y[1]=x[1]; y[2]=x[2];
	CQuaternionDouble q(x[3],x[4],x[5],x[6]);
	q.normalize();
	q.rpy(y[5],y[4],y[3]);
}

/*---------------------------------------------------------------
					CPose3DPDFGaussian
 ---------------------------------------------------------------*/
CPose3DPDFGaussian::CPose3DPDFGaussian( const CPose3DQuatPDFGaussian &o)  :
	mean(UNINITIALIZED_POSE), cov(UNINITIALIZED_MATRIX)
{
	this->copyFrom(o);
}

/*---------------------------------------------------------------
                    asString
 ---------------------------------------------------------------*/
void CPose3DPDFGaussian::asString(std::string &s) const
{
    ostringstream ss;
    ss << *this;
    s = ss.str();
}

/*---------------------------------------------------------------
						copyFrom
 ---------------------------------------------------------------*/
void CPose3DPDFGaussian::copyFrom( const CPose3DQuatPDFGaussian &o)
{
	MRPT_START
	if (!mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION)
	{
		// Convert using Jacobians and first order approximation:

		//         [  I_3   |    0         ]
		// dr_dq = [ -------+------------- ]
		//         [  0     | dr_dq_angles ]
#ifdef DO_TEST_JACOB
		// Test Jacob:
		{
			CVectorDouble x(4);
			for (int i=0;i<4;i++) x[i] = o.mean.quat()[i];
			CVectorDouble Ax(4); Ax.assign(1e-7);
			CMatrixDouble H;
			jacobians::jacob_numeric_estimate(x,ffff,Ax, o.mean.quat(),H);
			cout << "num:" <<endl <<H << endl << endl;
			CMatrixDouble J;
			double a,b,c;
			o.mean.quat().rpy_and_jacobian(a,b,c,&J);
			CMatrixDouble NJ;
			o.mean.quat().normalizationJacobian(NJ);
			cout << "lin:" <<endl<< J*NJ << endl << endl;
		}
#endif

		double yaw,pitch,roll;
		CMatrixFixedNumeric<double,3,4> dr_dq_sub_aux(UNINITIALIZED_MATRIX);

		o.mean.quat().rpy_and_jacobian(roll,pitch,yaw,&dr_dq_sub_aux,false);

		CMatrixDouble44 dnorm_dq(UNINITIALIZED_MATRIX);
		o.mean.quat().normalizationJacobian(dnorm_dq);

		CMatrixFixedNumeric<double,3,4> dr_dq_sub(UNINITIALIZED_MATRIX);
		dr_dq_sub.multiply(dr_dq_sub_aux,dnorm_dq);

		// Set the mean:
		this->mean.setFromValues(o.mean.x(),o.mean.y(),o.mean.z(),yaw,pitch,roll);

		// Cov:
		CMatrixDouble44 cov_Q(UNINITIALIZED_MATRIX);
		CMatrixDouble33 cov_T(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double,3,4> cov_TQ(UNINITIALIZED_MATRIX);
		o.cov.extractMatrix(3,3,cov_Q);
		o.cov.extractMatrix(0,0,cov_T);
		o.cov.extractMatrix(0,3,cov_TQ);

		// [        S_T       |   S_TQ * H^t    ]
		// [ -----------------+---------------- ]
		// [  (S_TQ * H^t)^t  |  H * S_Q * H^t  ]

		// top-left:
		this->cov.insertMatrix(0,0,cov_T);

		// diagonals:
		CMatrixFixedNumeric<double,3,3> cov_TR(UNINITIALIZED_MATRIX);
		cov_TR.multiply_ABt(cov_TQ,dr_dq_sub);
		this->cov.insertMatrix         (0,3,cov_TR);
		this->cov.insertMatrixTranspose(3,0,cov_TR);

		// bottom-right:
		CMatrixDouble33 cov_r(UNINITIALIZED_MATRIX);
		dr_dq_sub.multiply_HCHt(cov_Q,cov_r);
		this->cov.insertMatrix(3,3,cov_r);
	}
	else
	{
		// Use UT transformation:
		//   f: R^7 => R^6
		const CArrayDouble<7> x_mean(o.mean);
		CArrayDouble<6> y_mean;
		static const bool elements_do_wrapPI[6] = {false,false,false,true,true,true};  // xyz yaw pitch roll

		static const double dummy=0;
		mrpt::math::transform_gaussian_unscented(
			x_mean, o.cov,
			aux_posequat2poseypr,
			dummy,
			y_mean,
			this->cov,
			elements_do_wrapPI
			);
		this->mean.setFromValues(y_mean[0],y_mean[1],y_mean[2],y_mean[3],y_mean[4],y_mean[5]);
	}
	MRPT_END
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << mean;
		for (size_t r=0;r<size(cov,1);r++)
			out << cov.get_unsafe(r,r);
		for (size_t r=0;r<size(cov,1);r++)
			for (size_t c=r+1;c<size(cov,2);c++)
				out << cov.get_unsafe(r,c);
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			in >> mean;

			for (size_t r=0;r<size(cov,1);r++)
				in >> cov.get_unsafe(r,r);
			for (size_t r=0;r<size(cov,1);r++)
				for (size_t c=r+1;c<size(cov,2);c++)
				{
					float x;
					in >> x;
					cov.get_unsafe(r,c) = cov.get_unsafe(c,r) = x;
				}

		} break;
	case 1:
		{
			in >> mean;

			for (size_t r=0;r<size(cov,1);r++)
				in >> cov.get_unsafe(r,r);
			for (size_t r=0;r<size(cov,1);r++)
				for (size_t c=r+1;c<size(cov,2);c++)
				{
					double x;
					in >> x;
					cov.get_unsafe(r,c) = cov.get_unsafe(c,r) = x;
				}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

void  CPose3DPDFGaussian::copyFrom(const CPose3DPDF &o)
{
	if (this == &o) return;		// It may be used sometimes

	// Convert to gaussian pdf:
	o.getCovarianceAndMean(cov,mean);
}

void  CPose3DPDFGaussian::copyFrom(const CPosePDF &o)
{
	// Convert to gaussian pdf:
	CMatrixDouble33 C;
	CPose2D p;
	o.getCovarianceAndMean(C,p);
	mean = CPose3D(p);

	cov.zeros();
	cov.get_unsafe(0,0)=   C.get_unsafe(0,0);
	cov.get_unsafe(1,1)=   C.get_unsafe(1,1);
	cov.get_unsafe(3,3)=   C.get_unsafe(2,2);

	cov.get_unsafe(0,1)=
	cov.get_unsafe(1,0)=   C.get_unsafe(0,1);

	cov.get_unsafe(0,3)=
	cov.get_unsafe(3,0)=   C.get_unsafe(0,2);

	cov.get_unsafe(1,3)=
	cov.get_unsafe(3,1)=   C.get_unsafe(1,2);
}


/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::saveToTextFile(const string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;

	os::fprintf(f,"%e %e %e %e %e %e\n", mean.x(), mean.y(), mean.z(), mean.yaw(), mean.pitch(), mean.roll() );

	for (unsigned int i=0;i<6;i++)
		os::fprintf(f,"%e %e %e %e %e %e\n", cov(i,0),cov(i,1),cov(i,2),cov(i,3),cov(i,4),cov(i,5));

	os::fclose(f);
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::changeCoordinatesReference( const CPose3D &newReferenceBase )
{
	MRPT_START
	// this = p (+) this

	// COV:
	const CMatrixDouble66  OLD_COV = this->cov;
	CMatrixDouble66  df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DPDF::jacobiansPoseComposition(
		newReferenceBase,     // x
		this->mean,  // u
		df_dx,
		df_du );

	// this->cov = H1*this->cov*~H1 + H2* 0 *~H2;
	df_du.multiply_HCHt( OLD_COV, cov );

	// MEAN:
	this->mean.composeFrom(newReferenceBase, this->mean);

	MRPT_END
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::drawSingleSample( CPose3D &outPart ) const
{
	MRPT_START

	CVectorDouble	v;
	randomGenerator.drawGaussianMultivariate(v,cov);

	outPart.setFromValues(
		mean.x() + v[0],
		mean.y() + v[1],
		mean.z() + v[2],
		mean.yaw() + v[3],
		mean.pitch() + v[4],
		mean.roll() + v[5] );

	MRPT_END_WITH_CLEAN_UP( \
        cov.saveToTextFile("__DEBUG_EXC_DUMP_drawSingleSample_COV.txt"); \
		);
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::drawManySamples(
	size_t						N,
	vector<CVectorDouble>	&outSamples ) const
{
	MRPT_START

	randomGenerator.drawGaussianMultivariateMany(outSamples,N,cov);

	for (vector<CVectorDouble>::iterator it=outSamples.begin();it!=outSamples.end();++it)
	{
		(*it)[0] += mean.x();
		(*it)[1] += mean.y();
		(*it)[2] += mean.z();
		(*it)[3] = math::wrapToPi( (*it)[3] + mean.yaw() );
		(*it)[4] = math::wrapToPi( (*it)[4] + mean.pitch() );
		(*it)[5] = math::wrapToPi( (*it)[5] + mean.roll() );
	}

	MRPT_END
}


/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::bayesianFusion( const CPose3DPDF &p1_, const CPose3DPDF &p2_ )
{
	MRPT_UNUSED_PARAM(p1_); MRPT_UNUSED_PARAM(p2_);
	MRPT_START

	THROW_EXCEPTION("TO DO!!!");

/*	ASSERT_( p1_.GetRuntimeClass() == CLASS_ID( CPose3DPDFGaussian )  );
	ASSERT_( p2_.GetRuntimeClass() == CLASS_ID( CPose3DPDFGaussian )  );

	CPose3DPDFGaussian	*p1 = (CPose3DPDFGaussian*) &p1_;
	CPose3DPDFGaussian	*p2 = (CPose3DPDFGaussian*) &p2_;


	CMatrixD	x1(3,1),x2(3,1),x(3,1);
	CMatrixD	C1( p1->cov );
	CMatrixD	C2( p2->cov );
	CMatrixD	C1_inv = C1.inv();
	CMatrixD	C2_inv = C2.inv();
	CMatrixD	C;

	x1(0,0) = p1->mean.x; x1(1,0) = p1->mean.y; x1(2,0) = p1->mean.phi;
	x2(0,0) = p2->mean.x; x2(1,0) = p2->mean.y; x2(2,0) = p2->mean.phi;

	C = !(C1_inv + C2_inv);

	this->cov = C;
	this->assureSymmetry();

	x = C * ( C1_inv*x1 + C2_inv*x2 );

	this->mean.x = x(0,0);
	this->mean.y = x(1,0);
	this->mean.phi = x(2,0);
	this->mean.normalizePhi();
*/
	MRPT_END

}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void	 CPose3DPDFGaussian::inverse(CPose3DPDF &o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPose3DPDFGaussian));
	CPose3DPDFGaussian	&out = static_cast<CPose3DPDFGaussian&>(o);

	// This is like: b=(0,0,0)
	//  OUT = b - THIS
	CPose3DPDFGaussian p_zero( CPose3D(0,0,0, 0,0,0), CMatrixDouble66() ); // COV=All zeros

	out = p_zero - *this;
}


/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::operator += ( const CPose3D &Ap)
{
	// COV:
	const CMatrixDouble66  OLD_COV = this->cov;
	CMatrixDouble66  df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DPDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap,     // u
		df_dx,
		df_du );

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	df_dx.multiply_HCHt( OLD_COV, cov );
	// df_du: Nothing to do, since COV(Ap) = zeros

	// MEAN:
	this->mean.composeFrom(this->mean,  Ap);
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::operator += ( const CPose3DPDFGaussian &Ap)
{
	// Direct equations (for the covariances) in yaw-pitch-roll are too complex.
	//  Make a way around them and consider instead this path:
	//
	//      X(6D)       U(6D)
	//        |           |
	//        v           v
	//      X(7D)       U(7D)
	//        |           |
	//        +--- (+) ---+
	//              |
	//              v
	//            RES(7D)
	//              |
	//              v
	//            RES(6D)
	//
	CPose3DQuatPDFGaussian   X7(*this);
	const CPose3DQuatPDFGaussian   U7(Ap);

	X7+=U7;

	this->copyFrom(X7);
}

/*---------------------------------------------------------------
							-=
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::operator -= ( const CPose3DPDFGaussian &Ap)
{
	// Direct equations (for the covariances) in yaw-pitch-roll are too complex.
	//  Make a way around them and consider instead this path:
	//
	//      X(6D)       U(6D)
	//        |           |
	//        v           v
	//      X(7D)       U(7D)
	//        |           |
	//        +--- (-) ---+
	//              |
	//              v
	//            RES(7D)
	//              |
	//              v
	//            RES(6D)
	//
	CPose3DQuatPDFGaussian   X7(*this);
	const CPose3DQuatPDFGaussian   U7(Ap);

	X7-=U7;

	this->copyFrom(X7);
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double  CPose3DPDFGaussian::evaluatePDF( const CPose3D &x ) const
{
	MRPT_UNUSED_PARAM(x);
	THROW_EXCEPTION("TO DO!!!");

/*	CMatrixD	X(6,1);
	X(0,0) = x.x;
	X(1,0) = x.y;
	X(2,0) = x.z;

	CMatrixD	MU(6,1);
	MU(0,0) = mean.x;
	MU(1,0) = mean.y;
	MU(2,0) = mean.z;

	return math::normalPDF( X, MU, this->cov );
*/
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double  CPose3DPDFGaussian::evaluateNormalizedPDF( const CPose3D &x ) const
{
	MRPT_UNUSED_PARAM(x);
	THROW_EXCEPTION("TO DO!!!");
/*	CMatrixD	X(3,1);
	X(0,0) = x.x;
	X(1,0) = x.y;
	X(2,0) = x.phi;

	CMatrixD	MU(3,1);
	MU(0,0) = mean.x;
	MU(1,0) = mean.y;
	MU(2,0) = mean.phi;

	return math::normalPDF( X, MU, this->cov ) / math::normalPDF( MU, MU, this->cov );
*/
}

/*---------------------------------------------------------------
						assureSymmetry
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussian::assureSymmetry()
{
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (unsigned int i=0;i<size(cov,1)-1;i++)
		for (unsigned int j=i+1;j<size(cov,1);j++)
			cov.get_unsafe(i,j) = cov.get_unsafe(j,i);
}

/*---------------------------------------------------------------
						mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double  CPose3DPDFGaussian::mahalanobisDistanceTo( const CPose3DPDFGaussian& theOther )
{
	MRPT_START

	CMatrixDouble66	COV_ = cov + theOther.cov;
	CMatrixDouble61	MU   = CMatrixDouble61(theOther.mean) - CMatrixDouble61(mean);

	for (int i=0;i<6;i++)
	{
		if (COV_.get_unsafe(i,i)==0)
		{
			if (MU.get_unsafe(i,0)!=0)
					return std::numeric_limits<double>::infinity();
			else COV_.get_unsafe(i,i) = 1;  // Any arbitrary value since MU(i)=0, and this value doesn't affect the result.
		}
	}

	return std::sqrt( MU.multiply_HtCH_scalar(COV_.inv()) );

	MRPT_END
}

/*---------------------------------------------------------------
						operator <<
 ---------------------------------------------------------------*/
ostream &   mrpt::poses::operator << (
	ostream		&out,
	const CPose3DPDFGaussian	&obj )
{
	out << "Mean: " << obj.mean << "\n";
	out << "Covariance:\n" << obj.cov << "\n";

	return out;
}

/*---------------------------------------------------------------
						getCovSubmatrix2D
 ---------------------------------------------------------------*/
void CPose3DPDFGaussian::getCovSubmatrix2D( CMatrixDouble &out_cov ) const
{
	out_cov.setSize(3,3);

	for (int i=0;i<3;i++)
	{
		int a = i==2 ? 3:i;
		for (int j=i;j<3;j++)
		{
			int b = j==2 ? 3:j;
			double f = cov.get_unsafe(a,b);
			out_cov.set_unsafe(i,j, f);
			out_cov.set_unsafe(j,i, f);
		}
	}
}

bool mrpt::poses::operator==(const CPose3DPDFGaussian &p1,const CPose3DPDFGaussian &p2)
{
	return p1.mean==p2.mean && p1.cov==p2.cov;
}
