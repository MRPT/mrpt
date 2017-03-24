/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/math/transform_gaussian.h>
#include <mrpt/math/distributions.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

bool mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION = false;


IMPLEMENTS_SERIALIZABLE( CPose3DQuatPDFGaussian, CPose3DQuatPDF, mrpt::poses )

/** Default constructor - set all values to zero. */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian() :
	mean(), cov()
{
}

// Un-initialized constructor:
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian(TConstructorFlags_Quaternions constructor_dummy_param) :
	mean(UNINITIALIZED_QUATERNION), cov(UNINITIALIZED_MATRIX)
{
	MRPT_UNUSED_PARAM(constructor_dummy_param);
}

/** Constructor from a default mean value, covariance equals to zero. */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian( const CPose3DQuat &init_Mean ) :
	mean(init_Mean), cov()
{
}

/** Constructor with mean and covariance. */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian( const CPose3DQuat &init_Mean, const CMatrixDouble77 &init_Cov ) :
	mean(init_Mean), cov(init_Cov)
{
}

/** Constructor from a Gaussian 2D pose PDF (sets to 0 the missing variables). */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian( const CPosePDFGaussian &o ) :
	mean(UNINITIALIZED_QUATERNION), cov(UNINITIALIZED_MATRIX)
{
	this->copyFrom(CPose3DPDFGaussian(o));
}

/** Constructor from an equivalent Gaussian in Euler angles representation. */
CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian( const CPose3DPDFGaussian &o ) :
	mean(UNINITIALIZED_QUATERNION), cov(UNINITIALIZED_MATRIX)
{
	this->copyFrom(o);
}

/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF)
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::getMean(CPose3DQuat &p) const
{
	p=mean;
}

/*---------------------------------------------------------------
						getCovarianceAndMean
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::getCovarianceAndMean(CMatrixDouble77 &C, CPose3DQuat &p) const
{
	C=cov;
	p=mean;
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
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
void  CPose3DQuatPDFGaussian::readFromStream(mrpt::utils::CStream &in,int version)
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
					double x;
					in >> x;
					cov.get_unsafe(r,c) = cov.get_unsafe(c,r) = x;
				}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

void  CPose3DQuatPDFGaussian::copyFrom(const CPose3DQuatPDF &o)
{
	if (this == &o) return;		// It may be used sometimes

	// Convert to gaussian pdf:
	o.getCovarianceAndMean(cov,mean);
}

void  CPose3DQuatPDFGaussian::copyFrom(const CPosePDF &o)
{
	CPose3DPDFGaussian aux;
	aux.copyFrom(o);
	this->copyFrom(aux);
}


void aux_poseypr2posequat(const CArrayDouble<6> &x,const double&dummy, CArrayDouble<7> &y)
{
	MRPT_UNUSED_PARAM(dummy);
	y[0]=x[0];
	y[1]=x[1];
	y[2]=x[2];

	CPose3D  p(0,0,0, x[3],x[4],x[5]);
	CQuaternionDouble q(UNINITIALIZED_QUATERNION );
	p.getAsQuaternion(q);
	y[3] = q[0];
	y[4] = q[1];
	y[5] = q[2];
	y[6] = q[3];
}

void  CPose3DQuatPDFGaussian::copyFrom(const CPose3DPDFGaussian &o)
{
	if (!mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION)
	{	// Use Jacobians
		CMatrixFixedNumeric<double,4,3>  dq_dr_sub(UNINITIALIZED_MATRIX);

		// Mean:
		mean.x(o.mean.x());
		mean.y(o.mean.y());
		mean.z(o.mean.z());

		o.mean.getAsQuaternion(mean.quat(), &dq_dr_sub);

		// Cov:
		#if 1
			CMatrixFixedNumeric<double,7,6>  dq_dr;
			dq_dr.get_unsafe(0,0)=dq_dr.get_unsafe(1,1)=dq_dr.get_unsafe(2,2)=1;
			dq_dr.insertMatrix(3,3,dq_dr_sub);
			// Now for the covariance:
			dq_dr.multiply_HCHt( o.cov, this->cov);
		#else
			CMatrixDouble33 cov_R(UNINITIALIZED_MATRIX);
			CMatrixDouble33 cov_T(UNINITIALIZED_MATRIX);
			CMatrixDouble33 cov_TR(UNINITIALIZED_MATRIX);
			o.cov.extractMatrix(3,3,cov_R);
			o.cov.extractMatrix(0,0,cov_T);
			o.cov.extractMatrix(0,3,cov_TR);

			// [        S_T       |   S_TR * H^t    ]
			// [ -----------------+---------------- ]
			// [  (S_TR * H^t)^t  |  H * S_R * H^t  ]

			// top-left:
			this->cov.insertMatrix(0,0,cov_T);

			// diagonals:
			CMatrixFixedNumeric<double,3,4> cov_TQ(UNINITIALIZED_MATRIX);
			cov_TQ.multiply_ABt(cov_TR,dq_dr_sub);
			this->cov.insertMatrix         (0,3,cov_TQ);
			this->cov.insertMatrixTranspose(3,0,cov_TQ);

			// bottom-right:
			CMatrixDouble44 cov_q(UNINITIALIZED_MATRIX);
			dq_dr_sub.multiply_HCHt(cov_R,cov_q);
			this->cov.insertMatrix(3,3,cov_q);
		#endif
	}
	else
	{
		// Use UT transformation:
		//   f: R^6 => R^7
		const CArrayDouble<6> x_mean(o.mean);

		static const double dummy=0;
		mrpt::math::transform_gaussian_unscented(
			x_mean, o.cov,
			aux_poseypr2posequat,
			dummy,
			this->mean,
			this->cov
			);
	}
}

/*---------------------------------------------------------------
					saveToTextFile
  ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::saveToTextFile(const string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;

	os::fprintf(f,"%e %e %e %e %e %e %e\n", mean.x(), mean.y(), mean.z(), mean.quat()[0], mean.quat()[1], mean.quat()[2], mean.quat()[3]);

	for (unsigned int i=0;i<7;i++)
		os::fprintf(f,"%e %e %e %e %e %e %e\n", cov(i,0),cov(i,1),cov(i,2),cov(i,3),cov(i,4),cov(i,5),cov(i,6));

	os::fclose(f);
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::changeCoordinatesReference( const CPose3D &newReferenceBase )
{
	MRPT_START
	changeCoordinatesReference(CPose3DQuat(newReferenceBase));
	MRPT_END
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::changeCoordinatesReference( const CPose3DQuat &newReferenceBaseQuat )
{
	MRPT_START

	// COV:
	const CMatrixDouble77  OLD_COV = this->cov;
	CMatrixDouble77  df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		newReferenceBaseQuat,  // x
		this->mean,     	// u
		df_dx,
		df_du,
		&this->mean // Output:  newReferenceBaseQuat + this->mean;
		);

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	//df_dx: not used, since its COV are all zeros... // df_dx.multiply_HCHt( OLD_COV, cov );
	df_du.multiply_HCHt( OLD_COV,  cov);

	MRPT_END
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::drawSingleSample( CPose3DQuat &outPart ) const
{
	MRPT_START
	randomGenerator.drawGaussianMultivariate(outPart,cov, &mean);
	MRPT_END
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::drawManySamples(
	size_t						N,
	vector<CVectorDouble>	&outSamples ) const
{
	MRPT_START

	randomGenerator.drawGaussianMultivariateMany(outSamples,N,cov);

	for (vector<CVectorDouble>::iterator it=outSamples.begin();it!=outSamples.end();++it)
		for (unsigned int k=0;k<7;k++)
			(*it)[k] += mean[k];

	MRPT_END
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void	 CPose3DQuatPDFGaussian::inverse(CPose3DQuatPDF &o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPose3DQuatPDFGaussian));
	CPose3DQuatPDFGaussian	&out = static_cast<CPose3DQuatPDFGaussian&>(o);

	// COV:
	CMatrixFixedNumeric<double,3,7>  df_dpose(UNINITIALIZED_MATRIX);
	double lx,ly,lz;
	mean.inverseComposePoint(0,0,0,lx,ly,lz, NULL, &df_dpose);


	CMatrixFixedNumeric<double,7,7>  jacob;
	jacob.insertMatrix(0,0, df_dpose );
	jacob.set_unsafe(3,3,  1);
	jacob.set_unsafe(4,4, -1);
	jacob.set_unsafe(5,5, -1);
	jacob.set_unsafe(6,6, -1);

	// C(0:2,0:2): H C H^t
	jacob.multiply_HCHt( this->cov, out.cov );

	// Mean:
	out.mean.x(lx);
	out.mean.y(ly);
	out.mean.z(lz);
	this->mean.quat().conj( out.mean.quat() );
}


/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::operator += ( const CPose3DQuat &Ap)
{
	// COV:
	const CMatrixDouble77  OLD_COV = this->cov;
	CMatrixDouble77  df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap,     // u
		df_dx,
		df_du,
		&this->mean // Output: this->mean + Ap;
		);

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	df_dx.multiply_HCHt( OLD_COV, cov );
	// df_du: Nothing to do, since COV(Ap) = zeros

}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::operator += ( const CPose3DQuatPDFGaussian &Ap)
{
	// COV:
	const CMatrixDouble77  OLD_COV = this->cov;
	CMatrixDouble77  df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDF::jacobiansPoseComposition(
		this->mean,  // x
		Ap.mean,     // u
		df_dx,
		df_du,
		&this->mean 	// Output:  this->mean + Ap.mean;
		);

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	df_dx.multiply_HCHt( OLD_COV, cov );
	df_du.multiply_HCHt( Ap.cov,  cov, true); // Accumulate result
}

/*---------------------------------------------------------------
							-=
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::operator -= ( const CPose3DQuatPDFGaussian &Ap)
{
	// THIS = THIS (-) Ap             -->
	// THIS = inverse(Ap) (+) THIS
	CPose3DQuatPDFGaussian inv_Ap = -Ap;
	*this = inv_Ap + *this;
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double  CPose3DQuatPDFGaussian::evaluatePDF( const CPose3DQuat &x ) const
{
	return mrpt::math::normalPDF(CMatrixDouble71(x), CMatrixDouble71(this->mean), this->cov);
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double  CPose3DQuatPDFGaussian::evaluateNormalizedPDF( const CPose3DQuat &x ) const
{
	return mrpt::math::normalPDF(CMatrixDouble71(x),CMatrixDouble71(this->mean),this->cov, true);
}

/*---------------------------------------------------------------
						assureSymmetry
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::assureSymmetry()
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
double  CPose3DQuatPDFGaussian::mahalanobisDistanceTo( const CPose3DQuatPDFGaussian& theOther )
{
	MRPT_START
	const CMatrixDouble77	COV2 = cov + theOther.cov;
	return mrpt::math::mahalanobisDistance( CMatrixDouble71(this->mean) - CMatrixDouble71(theOther.mean), COV2);
	MRPT_END
}

/*---------------------------------------------------------------
						operator <<
 ---------------------------------------------------------------*/
ostream &   mrpt::poses::operator << (
	ostream		&out,
	const CPose3DQuatPDFGaussian	&obj )
{
	out << "Mean: " << obj.mean << "\n";
	out << "Covariance:\n" << obj.cov << "\n";
	return out;
}


bool mrpt::poses::operator==(const CPose3DQuatPDFGaussian &p1,const CPose3DQuatPDFGaussian &p2)
{
	return p1.mean==p2.mean && p1.cov==p2.cov;
}

CPose3DQuatPDFGaussian mrpt::poses::operator +( const CPose3DQuatPDFGaussian &x, const CPose3DQuatPDFGaussian &u )
{
	CPose3DQuatPDFGaussian 	res(x);
	res+=u;
	return res;
}

CPose3DQuatPDFGaussian mrpt::poses::operator -( const CPose3DQuatPDFGaussian &x, const CPose3DQuatPDFGaussian &u )
{
	CPose3DQuatPDFGaussian 	res(x);
	res-=u;
	return res;
}
