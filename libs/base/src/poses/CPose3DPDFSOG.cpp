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


#include <mrpt/poses/CPose3DPDFSOG.h>

using namespace mrpt::poses;
using namespace mrpt::utils;
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
	size_t		N = m_modes.size();

	if (N)
	{
		double			X=0,Y=0,Z=0,YAW=0,PITCH=0,ROLL=0;
		double			ang,sumW=0;

		double			W_yaw_R=0,W_yaw_L=0;
		double			yaw_R=0,yaw_L=0;
		double			W_roll_R=0,W_roll_L=0;
		double			roll_R=0,roll_L=0;


		const_iterator	it;
		double w;

		for (it=m_modes.begin();it!=m_modes.end();it++)
		{
			sumW += w = exp((it)->log_w);

			X += (it)->val.mean.x() * w;
			Y += (it)->val.mean.y() * w;
			Z += (it)->val.mean.z() * w;
			PITCH	+= w * (it)->val.mean.pitch();

			// Angles Yaw and Roll are especials!:
			ang = (it)->val.mean.yaw();
			if (fabs( ang )>1.5707963267948966192313216916398f)
			{
				// LEFT HALF: 0,2pi
				if (ang<0) ang = M_2PI + ang;
				yaw_L += ang * w;
				W_yaw_L += w;
			}
			else
			{
				// RIGHT HALF: -pi,pi
				yaw_R += ang * w;
				W_yaw_R += w;
			}

			// Angles Yaw and Roll are especials!:
			ang = (it)->val.mean.roll();
			if (fabs( ang )>1.5707963267948966192313216916398f)
			{
				// LEFT HALF: 0,2pi
				if (ang<0) ang = M_2PI + ang;
				roll_L += ang * w;
				W_roll_L += w;
			}
			else
			{
				// RIGHT HALF: -pi,pi
				roll_R += ang * w;
				W_roll_R += w;
			}
		}

		if (sumW==0)
		{
			p.setFromValues(0,0,0,0,0,0);
			return;
		}

		X /= sumW;
		Y /= sumW;
		Z /= sumW;
		PITCH /= sumW;

		// Next: Yaw and Roll:
		// -----------------------------------
		// The mean value from each side:
		if (W_yaw_L>0)	yaw_L /= W_yaw_L;  // [0,2pi]
		if (W_yaw_R>0)	yaw_R /= W_yaw_R;  // [-pi,pi]
		// Left side to [-pi,pi] again:
		if (yaw_L>M_PI) yaw_L = yaw_L - M_2PI;

		// The mean value from each side:
		if (W_roll_L>0)	roll_L /= W_roll_L;  // [0,2pi]
		if (W_roll_R>0)	roll_R /= W_roll_R;  // [-pi,pi]
		// Left side to [-pi,pi] again:
		if (roll_L>M_PI) roll_L = roll_L - M_2PI;

		// The total means:
		YAW = ((yaw_L * W_yaw_L + yaw_R * W_yaw_R )/(W_yaw_L+W_yaw_R));
		ROLL = ((roll_L * W_roll_L + roll_R * W_roll_R )/(W_roll_L+W_roll_R));

		p.setFromValues(X,Y,Z,YAW,PITCH,ROLL);
	}
	else
	{
		p.setFromValues(0,0,0,0,0,0);
	}
}

/*---------------------------------------------------------------
						getCovarianceAndMean
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::getCovarianceAndMean(CMatrixDouble66 &estCovOut,CPose3D &mean) const
{
	size_t		N = m_modes.size();

	getMean(mean);
	CMatrixDouble66 estCov;

	if (N)
	{
		// 1) Get the mean:
		double		w,sumW = 0;
		CMatrixDouble estMean( mean );

		const_iterator	it;

		CMatrixDouble66		MMt;
		CMatrixDouble61 estMean_i;
		for (it=m_modes.begin();it!=m_modes.end();it++)
		{
			sumW += w = exp((it)->log_w);
			estMean_i = CMatrixDouble61((it)->val.mean);
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
void  CPose3DPDFSOG::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		uint32_t	N = m_modes.size();
		const_iterator		it;

		out << N;

		for (it=m_modes.begin();it!=m_modes.end();it++)
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
void  CPose3DPDFSOG::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			uint32_t		N;
			iterator		it;

			in >> N;

			this->resize(N);

			for (it=m_modes.begin();it!=m_modes.end();it++)
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



/*---------------------------------------------------------------
						operator =
  ---------------------------------------------------------------*/
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
		CMatrixDouble66 C;
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

	if (!m_modes.size()) return;

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
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
						drawManySamples
 ---------------------------------------------------------------*/
void CPose3DPDFSOG::drawManySamples( size_t N, std::vector<vector_double> & outSamples ) const
{
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
