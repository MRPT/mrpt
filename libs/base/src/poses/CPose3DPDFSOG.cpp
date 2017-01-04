/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/poses/SO_SE_average.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CPose3DPDFSOG, CPose3DPDF, mrpt::poses )

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFSOG::CPose3DPDFSOG( size_t nModes ) :
	m_modes(nModes)
{
}

/*---------------------------------------------------------------
			clear
  ---------------------------------------------------------------*/
void CPose3DPDFSOG::clear()
{
	m_modes.clear();
}

/*---------------------------------------------------------------
	Resize
  ---------------------------------------------------------------*/
void CPose3DPDFSOG::resize(const size_t N)
{
	m_modes.resize(N);
}


/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF)
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::getMean(CPose3D &p) const
{
	if (!m_modes.empty())
	{
		// Calc average on SE(3)
		mrpt::poses::SE_average<3> se_averager;
		for (const_iterator	it=m_modes.begin();it!=m_modes.end();++it)
		{
			const double w  = exp(it->log_w);
			se_averager.append( (it)->val.mean,w );
		}
		se_averager.get_average(p);
	}
	else
	{
		p.setFromValues(0,0,0,0,0,0);
	}
}

/*---------------------------------------------------------------
						getCovarianceAndMean
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::getCovarianceAndMean(mrpt::math::CMatrixDouble66 &estCovOut,CPose3D &mean) const
{
	size_t		N = m_modes.size();

	getMean(mean);
	mrpt::math::CMatrixDouble66 estCov;

	if (N)
	{
		// 1) Get the mean:
		double		sumW = 0;
		mrpt::math::CMatrixDouble estMean( mean );

		mrpt::math::CMatrixDouble66		MMt;
		mrpt::math::CMatrixDouble61 estMean_i;
		for (const_iterator	it=m_modes.begin();it!=m_modes.end();++it)
		{
		    double w;
			sumW += w = exp((it)->log_w);
			estMean_i = mrpt::math::CMatrixDouble61((it)->val.mean);
			MMt.multiply_AAt(estMean_i);
			MMt+=(it)->val.cov;
			MMt*=w;
			estCov += MMt; //w * ( (it)->val.cov + ((estMean_i-estMean)*(~(estMean_i-estMean))) );
		}

		if (sumW!=0)
			estCov *= (1.0/sumW);
	}

	estCovOut = estCov;
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPose3DPDFSOG::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		uint32_t	N = m_modes.size();
		out << N;
		for (const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		{
			out << (it)->log_w;
			out << (it)->val.mean;
			out << (it)->val.cov;
		}
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPose3DPDFSOG::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			uint32_t		N;
			in >> N;
			this->resize(N);

			for (iterator it=m_modes.begin();it!=m_modes.end();++it)
			{
				in >> (it)->log_w;

				// In version 0, weights were linear!!
				if (version==0) (it)->log_w = log(max(1e-300,(it)->log_w));

				in >> (it)->val.mean;


				if (version==1) // were floats
				{
					THROW_EXCEPTION("Unsupported serialized version: too old")
				}
				else
				{
					in >> (it)->val.cov;
				}
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

void  CPose3DPDFSOG::copyFrom(const CPose3DPDF &o)
{
	MRPT_START

	if (this == &o) return;		// It may be used sometimes

	if (o.GetRuntimeClass()==CLASS_ID(CPose3DPDFSOG))
	{
		*this = *static_cast<const CPose3DPDFSOG*>(&o);
	}
	else
	{
		this->resize(1);
		m_modes[0].log_w = 0;
		mrpt::math::CMatrixDouble66 C;
		o.getCovarianceAndMean( C, m_modes[0].val.mean );
		m_modes[0].val.cov = C;
	}

	MRPT_END
}

/*---------------------------------------------------------------
						saveToTextFile
  ---------------------------------------------------------------*/
void  CPose3DPDFSOG::saveToTextFile(const std::string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;


	for (const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		os::fprintf(f,"%e %e %e %e %e %e %e %e %e %e\n",
			exp((it)->log_w),
			(it)->val.mean.x(), (it)->val.mean.y(), (it)->val.mean.z(),
			(it)->val.cov(0,0),(it)->val.cov(1,1),(it)->val.cov(2,2),
			(it)->val.cov(0,1),(it)->val.cov(0,2),(it)->val.cov(1,2) );
	os::fclose(f);
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPose3DPDFSOG::changeCoordinatesReference(const CPose3D &newReferenceBase )
{
	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
		(it)->val.changeCoordinatesReference( newReferenceBase );
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPose3DPDFSOG::bayesianFusion(const  CPose3DPDF &p1_,const  CPose3DPDF &p2_ )
{
	MRPT_START

	// p1: CPose3DPDFSOG, p2: CPosePDFGaussian:

	ASSERT_( p1_.GetRuntimeClass() == CLASS_ID(CPose3DPDFSOG) );
	ASSERT_( p2_.GetRuntimeClass() == CLASS_ID(CPose3DPDFSOG) );

	THROW_EXCEPTION("TODO!!!");
#if 0
/*
	CPose3DPDFSOG		*p1 = (CPose3DPDFSOG*)&p1_;
	CPose3DPDFSOG		*p2 = (CPose3DPDFSOG*)&p2_;

	// Compute the new kernel means, covariances, and weights after multiplying to the Gaussian "p2":
	CPosePDFGaussian	auxGaussianProduct,auxSOG_Kernel_i;
	TGaussianMode		newKernel;



	CMatrixD				covInv( p2->cov.inv() );
	CMatrixD				eta(3,1);
	eta(0,0) = p2->mean.x;
	eta(1,0) = p2->mean.y;
	eta(2,0) = p2->mean.phi;
	eta = covInv * eta;

	// Normal distribution canonical form constant:
	// See: http://www-static.cc.gatech.edu/~wujx/paper/Gaussian.pdf
	double				a = -0.5*( 3*log(M_2PI) - log( covInv.det() ) + (~eta * p2->cov * eta)(0,0) );

	this->m_modes.clear();
	for (std::deque<TGaussianMode>::iterator it =p1->m_modes.begin();it!=p1->m_modes.end();++it)
	{
		auxSOG_Kernel_i.mean = it->mean;
		auxSOG_Kernel_i.cov  = it->cov;
		auxGaussianProduct.bayesianFusion( auxSOG_Kernel_i, *p2 );

		// ----------------------------------------------------------------------
		// The new weight is given by:
		//
		//   w'_i = w_i * exp( a + a_i - a' )
		//
		//      a = -1/2 ( dimensionality * log(2pi) - log(det(Cov^-1)) + (Cov^-1 * mu)^t * Cov^-1 * (Cov^-1 * mu) )
		//
		// ----------------------------------------------------------------------
		newKernel.mean = auxGaussianProduct.mean;
		newKernel.cov  = auxGaussianProduct.cov;

		CMatrixD		covInv_i( auxSOG_Kernel_i.cov.inv() );
		CMatrixD		eta_i(3,1);
		eta_i(0,0) = auxSOG_Kernel_i.mean.x;
		eta_i(1,0) = auxSOG_Kernel_i.mean.y;
		eta_i(2,0) = auxSOG_Kernel_i.mean.phi;
		eta_i = covInv_i * eta_i;

		CMatrixD		new_covInv_i( newKernel.cov.inv() );
		CMatrixD		new_eta_i(3,1);
		new_eta_i(0,0) = newKernel.mean.x;
		new_eta_i(1,0) = newKernel.mean.y;
		new_eta_i(2,0) = newKernel.mean.phi;
		new_eta_i = new_covInv_i * new_eta_i;

		double		a_i	    = -0.5*( 3*log(M_2PI) - log( new_covInv_i.det() ) + (~eta_i * auxSOG_Kernel_i.cov * eta_i)(0,0) );
		double		new_a_i = -0.5*( 3*log(M_2PI) - log( new_covInv_i.det() ) + (~new_eta_i * newKernel.cov * new_eta_i)(0,0) );

		newKernel.w	   = it->w * exp( a + a_i - new_a_i );

		// Add to the results (in "this") the new kernel:
		this->m_modes.push_back( newKernel );
	}
*/
	normalizeWeights();
#endif
	MRPT_END
}

/*---------------------------------------------------------------
						assureSymmetry
 ---------------------------------------------------------------*/
void  CPose3DPDFSOG::assureSymmetry()
{
	MRPT_START
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
	{
		for (size_t i=0;i<6;i++)
			for (size_t j=i+1;j<6;j++)
				(it)->val.cov.get_unsafe(i,j) = (it)->val.cov.get_unsafe(j,i);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						normalizeWeights
 ---------------------------------------------------------------*/
void  CPose3DPDFSOG::normalizeWeights()
{
	MRPT_START

	if (m_modes.empty()) return;

	double		maxW = m_modes[0].log_w;
	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
		maxW = max(maxW,(it)->log_w);

	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
		(it)->log_w -= maxW;

	MRPT_END
}

/*---------------------------------------------------------------
						drawSingleSample
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::drawSingleSample( CPose3D &outPart ) const
{
	MRPT_UNUSED_PARAM(outPart);
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
						drawManySamples
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::drawManySamples( size_t N, std::vector<CVectorDouble> & outSamples ) const
{
	MRPT_UNUSED_PARAM(N); MRPT_UNUSED_PARAM(outSamples);
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
						inverse
 ---------------------------------------------------------------*/
void  CPose3DPDFSOG::inverse(CPose3DPDF &o) const
{
	MRPT_START
	ASSERT_( o.GetRuntimeClass() == CLASS_ID(CPose3DPDFSOG) );
	CPose3DPDFSOG	*out = static_cast<CPose3DPDFSOG*>( &o );

	// Prepare the output SOG:
	out->resize(m_modes.size());

	const_iterator	it;
	iterator	outIt;

	for (it=m_modes.begin(),outIt=out->m_modes.begin();it!=m_modes.end();it++,outIt++)
	{
		(it)->val.inverse( (outIt)->val );
		(outIt)->log_w = (it)->log_w;
	}


	MRPT_END
}

/*---------------------------------------------------------------
						appendFrom
 ---------------------------------------------------------------*/
void  CPose3DPDFSOG::appendFrom( const CPose3DPDFSOG &o )
{
	MRPT_START

	ASSERT_( &o != this );  // Don't be bad...
	if (o.m_modes.empty()) return;

	// Make copies:
	for (const_iterator it = o.m_modes.begin();it!=o.m_modes.end();++it)
		m_modes.push_back( *it );

	normalizeWeights();
	MRPT_END
}


/*---------------------------------------------------------------
						getMostLikelyMode
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::getMostLikelyMode( CPose3DPDFGaussian& outVal ) const
{
	if (this->empty())
	{
		outVal = CPose3DPDFGaussian();
	}
	else
	{
		const_iterator it_best = m_modes.end();
		for (const_iterator it = m_modes.begin();it!=m_modes.end();++it)
			if (it_best==m_modes.end() || it->log_w>it_best->log_w)
				it_best = it;

		outVal = it_best->val;
	}
}
