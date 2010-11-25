/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/random.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/jacobians.h>
#include <mrpt/math/transform_gaussian.h>
#include <mrpt/math/distributions.h>

#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>

using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils;
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
void  CPose3DQuatPDFGaussian::writeToStream(CStream &out,int *version) const
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
void  CPose3DQuatPDFGaussian::readFromStream(CStream &in,int version)
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


/*---------------------------------------------------------------
						operator =
  ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::copyFrom(const CPose3DQuatPDF &o)
{
	if (this == &o) return;		// It may be used sometimes

	// Convert to gaussian pdf:
	o.getCovarianceAndMean(cov,mean);
}

/*---------------------------------------------------------------
						operator =
  ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::copyFrom(const CPosePDF &o)
{
	CPose3DPDFGaussian aux;
	aux.copyFrom(o);
	this->copyFrom(aux);
}


void aux_poseypr2posequat(const CArrayDouble<6> &x,const double&dummy, CArrayDouble<7> &y)
{
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
/*---------------------------------------------------------------
						operator =
  ---------------------------------------------------------------*/
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

	CPose3DQuatPDFGaussian::jacobiansPoseComposition(
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
	vector<vector_double>	&outSamples ) const
{
	MRPT_START;

	randomGenerator.drawGaussianMultivariateMany(outSamples,N,cov);

	for (vector<vector_double>::iterator it=outSamples.begin();it!=outSamples.end();++it)
		for (unsigned int k=0;k<7;k++)
			(*it)[k] += mean[k];

	MRPT_END;
}


/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::bayesianFusion(const  CPose3DQuatPDF &p1_,const  CPose3DQuatPDF &p2_ )
{
	MRPT_UNUSED_PARAM(p1_); MRPT_UNUSED_PARAM(p2_);
	MRPT_START;

	THROW_EXCEPTION("TO DO!!!");

/*	ASSERT_( p1_.GetRuntimeClass() == CLASS_ID( CPose3DQuatPDFGaussian )  );
	ASSERT_( p2_.GetRuntimeClass() == CLASS_ID( CPose3DQuatPDFGaussian )  );

	CPose3DQuatPDFGaussian	*p1 = (CPose3DQuatPDFGaussian*) &p1_;
	CPose3DQuatPDFGaussian	*p2 = (CPose3DQuatPDFGaussian*) &p2_;


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
	MRPT_END;

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

	CPose3DQuatPDFGaussian::jacobiansPoseComposition(
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
					jacobiansPoseComposition
 ---------------------------------------------------------------*/
void CPose3DQuatPDFGaussian::jacobiansPoseComposition(
	const CPose3DQuat &x,
	const CPose3DQuat &u,
	CMatrixDouble77	  &df_dx,
	CMatrixDouble77	  &df_du,
	CPose3DQuat       *out_x_oplus_u
	)
{
	// For the derivation of the formulas, see the tech. report cited in the header file.

	const double   qr = x.quat().r();
	const double   qx = x.quat().x(); const double qx2 = square(qx);
	const double   qy = x.quat().y(); const double qy2 = square(qy);
	const double   qz = x.quat().z(); const double qz2 = square(qz);

	const double   ax  = u.x();
	const double   ay  = u.y();
	const double   az  = u.z();
	const double   q2r = u.quat().r();
	const double   q2x = u.quat().x();
	const double   q2y = u.quat().y();
	const double   q2z = u.quat().z();

	const CPose3DQuat  x_plus_u = x + u;  // needed for the normalization Jacobian:
	CMatrixDouble44  norm_jacob(UNINITIALIZED_MATRIX);
	x_plus_u.quat().normalizationJacobian(norm_jacob);

	CMatrixDouble44  norm_jacob_x(UNINITIALIZED_MATRIX);
	x.quat().normalizationJacobian(norm_jacob_x);

	// df_dx ===================================================
	df_dx.zeros();

	// first part 3x7:  df_{qr} / dp
	df_dx.set_unsafe(0,0, 1);
	df_dx.set_unsafe(1,1, 1);
	df_dx.set_unsafe(2,2, 1);

	EIGEN_ALIGN16 const double vals2[3*4] = {
		2*(-qz*ay +qy*az  ),
		2*(qy*ay + qz*az  ),
		2*(-2*qy*ax + qx*ay +qr*az  ),
		2*(-2*qz*ax - qr*ay +qx*az  ),

		2*(qz*ax - qx*az   ),
		2*(qy*ax - 2*qx*ay -qr*az  ),
		2*(qx*ax +qz*az   ),
		2*(qr*ax - 2*qz*ay +qy*az ),

		2*(-qy*ax + qx*ay  ),
		2*( qz*ax + qr*ay - 2*qx*az  ),
		2*(-qr*ax + qz*ay - 2*qy*az  ),
		2*( qx*ax + qy*ay )
		};

	// df_dx(0:3,3:7) = vals2 * NORM_JACOB
	df_dx.block(0,3, 3,4).noalias() = (CMatrixFixedNumeric<double,3,4>(vals2) * norm_jacob_x).eval();
	// second part:
	{
		EIGEN_ALIGN16 const double aux44_data[4*4] = {
			q2r,-q2x,-q2y,-q2z,
			q2x, q2r, q2z,-q2y,
			q2y,-q2z, q2r, q2x,
			q2z, q2y,-q2x, q2r };

		df_dx.block(3,3, 4,4).noalias() =  (norm_jacob * CMatrixFixedNumeric<double,4,4>(aux44_data)).eval();
	}

	// df_du ===================================================
	df_du.zeros();

	// first part 3x3:  df_{qr} / da
	df_du.set_unsafe(0,0,  1-2*(qy2+qz2) );
	df_du.set_unsafe(0,1,  2*(qx*qy - qr*qz ) );
	df_du.set_unsafe(0,2,  2*(qr*qy + qx*qz ) );

	df_du.set_unsafe(1,0,  2*(qr*qz + qx*qy  ) );
	df_du.set_unsafe(1,1,  1 - 2*( qx2+qz2) );
	df_du.set_unsafe(1,2,  2*(qy*qz - qr*qx ) );

	df_du.set_unsafe(2,0,  2*(qx*qz - qr*qy ) );
	df_du.set_unsafe(2,1,  2*(qr*qx + qy*qz ) );
	df_du.set_unsafe(2,2,  1-2*(qx2+qy2) );

	// Second part:
	{
		EIGEN_ALIGN16 const double aux44_data[4*4] = {
			qr,-qx,-qy,-qz,
			qx, qr,-qz, qy,
			qy, qz, qr,-qx,
			qz,-qy, qx, qr };

//		std::cout  << "x.quat:\n" << x.quat() << std::endl;
//		std::cout  << "aux44:\n" << CMatrixFixedNumeric<double,4,4>(aux44_data) << std::endl;
		df_du.block(3,3, 4,4).noalias() = (norm_jacob * CMatrixFixedNumeric<double,4,4>(aux44_data)).eval();
	}

	if (out_x_oplus_u)
		*out_x_oplus_u = x_plus_u;
}


/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPose3DQuatPDFGaussian::operator += ( const CPose3DQuatPDFGaussian &Ap)
{
	// COV:
	const CMatrixDouble77  OLD_COV = this->cov;
	CMatrixDouble77  df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DQuatPDFGaussian::jacobiansPoseComposition(
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
	return p1.mean==p1.mean && p1.cov==p2.cov;
}
