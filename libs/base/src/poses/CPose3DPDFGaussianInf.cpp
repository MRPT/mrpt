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
#include <mrpt/math/CMatrixViews.h>
#include <mrpt/math/transform_gaussian.h>

#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace std;


IMPLEMENTS_SERIALIZABLE( CPose3DPDFGaussianInf, CPose3DPDF, mrpt::poses )


/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf() : mean(0,0,0), cov_inv()
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf(TConstructorFlags_Poses constructor_dummy_param) : mean(UNINITIALIZED_POSE), cov_inv(UNINITIALIZED_MATRIX)
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf( const CPose3D &init_Mean, const CMatrixDouble66 &init_Cov ) :
	mean(init_Mean), cov_inv(init_Cov)
{
}

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf(
	const CPose3D	&init_Mean ) : mean(init_Mean), cov_inv()
{
}

/*---------------------------------------------------------------
					CPose3DPDFGaussianInf
 ---------------------------------------------------------------*/
CPose3DPDFGaussianInf::CPose3DPDFGaussianInf( const CPose3DQuatPDFGaussian &o)  :
	mean(UNINITIALIZED_POSE), cov_inv(UNINITIALIZED_MATRIX)
{
	this->copyFrom(o);
}

/*---------------------------------------------------------------
						copyFrom
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::copyFrom( const CPose3DQuatPDFGaussian &o)
{
	const CPose3DPDFGaussian p(o);
	this->copyFrom(p);
}

/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF)
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::getMean(CPose3D &p) const
{
	p=mean;
}

/*---------------------------------------------------------------
						getCovarianceAndMean
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::getCovarianceAndMean(CMatrixDouble66 &C, CPose3D &p) const
{
	this->cov_inv.inv(C);
	p=mean;
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << mean;
		for (size_t r=0;r<size(cov_inv,1);r++)
			out << cov_inv.get_unsafe(r,r);
		for (size_t r=0;r<size(cov_inv,1);r++)
			for (size_t c=r+1;c<size(cov_inv,2);c++)
				out << cov_inv.get_unsafe(r,c);
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 1:
		{
			in >> mean;

			for (size_t r=0;r<size(cov_inv,1);r++)
				in >> cov_inv.get_unsafe(r,r);
			for (size_t r=0;r<size(cov_inv,1);r++)
				for (size_t c=r+1;c<size(cov_inv,2);c++)
				{
					double x;
					in >> x;
					cov_inv.get_unsafe(r,c) = cov_inv.get_unsafe(c,r) = x;
				}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


/*---------------------------------------------------------------
						operator =
  ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::copyFrom(const CPose3DPDF &o)
{
	if (this == &o) return;		// It may be used sometimes

	// Convert to gaussian pdf:
	CMatrixDouble66 cov(UNINITIALIZED_MATRIX);
	o.getCovarianceAndMean(cov,mean);
	cov.inv_fast(this->cov_inv);
}

/*---------------------------------------------------------------
						operator =
  ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::copyFrom(const CPosePDF &o)
{
	CPose3DPDFGaussian p(UNINITIALIZED_POSE);
	p.copyFrom(o);
	this->copyFrom(p);
}

/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::saveToTextFile(const string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;

	os::fprintf(f,"%e %e %e %e %e %e\n", mean.x(), mean.y(), mean.z(), mean.yaw(), mean.pitch(), mean.roll() );

	for (unsigned int i=0;i<6;i++)
		os::fprintf(f,"%e %e %e %e %e %e\n", cov_inv(i,0),cov_inv(i,1),cov_inv(i,2),cov_inv(i,3),cov_inv(i,4),cov_inv(i,5));

	os::fclose(f);
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::changeCoordinatesReference( const CPose3D &newReferenceBase )
{
	MRPT_START

	CMatrixDouble44	HM;
	newReferenceBase.getHomogeneousMatrix(HM);

	CMatrixDouble66   M = CMatrixDouble66(HM);	// Clip the 4x4 matrix

	// The variance in yaw,pitch & roll is unmodified:
	M.get_unsafe(3,3) = M.get_unsafe(4,4) = M.get_unsafe(5,5) = 1;
	M.get_unsafe(0,3) = M.get_unsafe(1,3) = M.get_unsafe(2,3) = 0;

	// The mean:
	mean = newReferenceBase + mean;

	// The covariance:
	// cov     =    M  *   cov   * (~M);
	// cov_inv = !(~M) * cov_inv * !M
	//         =    M  * cov_inv * (~M)

	M.multiply_HCHt( CMatrixDouble66(cov_inv), cov_inv );  // CMatrixDouble66() makes a temporary copy of the input so it can be used as output.

	MRPT_END
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::drawSingleSample( CPose3D &outPart ) const
{
	MRPT_UNUSED_PARAM(outPart);
	MRPT_START;

	CMatrixDouble66 cov(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(cov);

	vector_double	v;
	randomGenerator.drawGaussianMultivariate(v,cov);

	outPart.setFromValues(
		mean.x() + v[0],
		mean.y() + v[1],
		mean.z() + v[2],
		mean.yaw() + v[3],
		mean.pitch() + v[4],
		mean.roll() + v[5] );

	MRPT_END_WITH_CLEAN_UP( \
        cov_inv.saveToTextFile("__DEBUG_EXC_DUMP_drawSingleSample_COV_INV.txt"); \
		);
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::drawManySamples(
	size_t						N,
	vector<vector_double>	&outSamples ) const
{
	MRPT_START;

	CMatrixDouble66 cov(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(cov);

	randomGenerator.drawGaussianMultivariateMany(outSamples,N,cov);

	for (vector<vector_double>::iterator it=outSamples.begin();it!=outSamples.end();++it)
	{
		it->at(0) += mean.x();
		it->at(1) += mean.y();
		it->at(2) += mean.z();
		it->at(3) = math::wrapToPi( it->at(3) + mean.yaw() );
		it->at(4) = math::wrapToPi( it->at(4) + mean.pitch() );
		it->at(5) = math::wrapToPi( it->at(5) + mean.roll() );
	}

	MRPT_END;
}


/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::bayesianFusion( const CPose3DPDF &p1_, const CPose3DPDF &p2_ )
{
	MRPT_UNUSED_PARAM(p1_); MRPT_UNUSED_PARAM(p2_);

	THROW_EXCEPTION("TO DO!!!");
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void	 CPose3DPDFGaussianInf::inverse(CPose3DPDF &o) const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPose3DPDFGaussianInf));
	CPose3DPDFGaussianInf	&out = static_cast<CPose3DPDFGaussianInf&>(o);

	// This is like: b=(0,0,0)
	//  OUT = b - THIS
	CPose3DPDFGaussianInf b;  // Init: all zeros.
	out = b - *this;
}


/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::operator += ( const CPose3D &Ap)
{
	// COV:
	const CMatrixDouble66  OLD_COV_INV = this->cov_inv;
	CMatrixDouble66  df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);

	CPose3DPDFGaussianInf::jacobiansPoseComposition(
		this->mean,  // x
		Ap,     // u
		df_dx,
		df_du );

	// this->cov = H1*this->cov*~H1 + H2*Ap.cov*~H2;
	// cov_inv   = ... => The same than above!
	df_dx.multiply_HCHt( OLD_COV_INV, cov_inv );
	// df_du: Nothing to do, since COV(Ap) = zeros

	// MEAN:
	this->mean = this->mean + Ap;
}

/*---------------------------------------------------------------
					jacobiansPoseComposition
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::jacobiansPoseComposition(
	const CPose3D &x,
	const CPose3D &u,
	CMatrixDouble66	 &df_dx,
	CMatrixDouble66	 &df_du)
{
	// See this techical report: http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty
	CPose3DPDFGaussian::jacobiansPoseComposition(
		x,u,
		df_dx,
		df_du);
}

/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::operator += ( const CPose3DPDFGaussianInf &Ap)
{
	CPose3DPDFGaussian a(UNINITIALIZED_POSE);
	CPose3DPDFGaussian b(UNINITIALIZED_POSE);
	a.copyFrom(*this);
	b.copyFrom(Ap);

	a+=b;

	this->mean = a.mean;
	a.cov.inv( this->cov_inv );
}

/*---------------------------------------------------------------
							-=
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::operator -= ( const CPose3DPDFGaussianInf &Ap)
{
	CPose3DPDFGaussian a(UNINITIALIZED_POSE);
	CPose3DPDFGaussian b(UNINITIALIZED_POSE);
	a.copyFrom(*this);
	b.copyFrom(Ap);

	a-=b;

	this->mean = a.mean;
	a.cov.inv( this->cov_inv );
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double  CPose3DPDFGaussianInf::evaluatePDF( const CPose3D &x ) const
{
	MRPT_UNUSED_PARAM(x);
	THROW_EXCEPTION("TO DO!!!");
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double  CPose3DPDFGaussianInf::evaluateNormalizedPDF( const CPose3D &x ) const
{
	MRPT_UNUSED_PARAM(x);
	THROW_EXCEPTION("TO DO!!!");
}

/*---------------------------------------------------------------
						assureSymmetry
 ---------------------------------------------------------------*/
void  CPose3DPDFGaussianInf::assureSymmetry()
{
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (unsigned int i=0;i<size(cov_inv,1)-1;i++)
		for (unsigned int j=i+1;j<size(cov_inv,1);j++)
			cov_inv.get_unsafe(i,j) = cov_inv.get_unsafe(j,i);
}

/*---------------------------------------------------------------
						mahalanobisDistanceTo
 ---------------------------------------------------------------*/
double  CPose3DPDFGaussianInf::mahalanobisDistanceTo( const CPose3DPDFGaussianInf& theOther )
{
	MRPT_START

	CMatrixDouble66	 cov(UNINITIALIZED_MATRIX), cov2(UNINITIALIZED_MATRIX);
	this->cov_inv.inv(cov);
	theOther.cov_inv.inv(cov2);

	CMatrixDouble66	COV_ = cov+cov2;
	CMatrixDouble16	MU   = CMatrixDouble16(theOther.mean) - CMatrixDouble16(mean);

	for (int i=0;i<6;i++)
	{
		if (COV_.get_unsafe(i,i)==0)
		{
			if (MU.get_unsafe(0,i)!=0)
					return std::numeric_limits<double>::infinity();
			else COV_.get_unsafe(i,i) = 1;  // Any arbitrary value since MU(i)=0, and this value doesn't affect the result.
		}
	}

	CMatrixDouble66	COV_inv;
	COV_.inv(COV_inv);

	return std::sqrt( MU.multiply_HCHt_scalar(COV_inv) );

	MRPT_END
}

/*---------------------------------------------------------------
						operator <<
 ---------------------------------------------------------------*/
ostream &   mrpt::poses::operator << (
	ostream		&out,
	const CPose3DPDFGaussianInf	&obj )
{
	out << "Mean: " << obj.mean << "\n";
	out << "Inverse cov:\n" << obj.cov_inv << "\n";

	return out;
}

/*---------------------------------------------------------------
						getInvCovSubmatrix2D
 ---------------------------------------------------------------*/
void CPose3DPDFGaussianInf::getInvCovSubmatrix2D( CMatrixDouble &out_cov ) const
{
	out_cov.setSize(3,3);

	for (int i=0;i<3;i++)
	{
		int a = i==2 ? 3:i;
		for (int j=i;j<3;j++)
		{
			int b = j==2 ? 3:j;
			double f = cov_inv.get_unsafe(a,b);
			out_cov.set_unsafe(i,j, f);
			out_cov.set_unsafe(j,i, f);
		}
	}
}

bool mrpt::poses::operator==(const CPose3DPDFGaussianInf &p1,const CPose3DPDFGaussianInf &p2)
{
	return p1.mean==p1.mean && p1.cov_inv==p2.cov_inv;
}
