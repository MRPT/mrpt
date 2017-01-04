/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/poses/CPointPDFSOG.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/random.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>

using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::bayes;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CPointPDFSOG, CPosePDF, mrpt::poses )


/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPointPDFSOG::CPointPDFSOG( size_t nModes ) :
	m_modes(nModes)
{
}

/*---------------------------------------------------------------
			clear
  ---------------------------------------------------------------*/
void CPointPDFSOG::clear()
{
	m_modes.clear();
}

/*---------------------------------------------------------------
	Resize
  ---------------------------------------------------------------*/
void CPointPDFSOG::resize(const size_t N)
{
	m_modes.resize(N);
}


/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF)
 ---------------------------------------------------------------*/
void CPointPDFSOG::getMean(CPoint3D &p) const
{
	size_t		N = m_modes.size();
	double		X=0,Y=0,Z=0;

	if (N)
	{
		CListGaussianModes::const_iterator	it;
		double		sumW = 0;

		for (it=m_modes.begin();it!=m_modes.end();++it)
		{
		    double w;
			sumW += w = exp(it->log_w);
			X += it->val.mean.x() * w;
			Y += it->val.mean.y() * w;
			Z += it->val.mean.z() * w;
		}
		if (sumW>0)
		{
			X /= sumW;
			Y /= sumW;
			Z /= sumW;
		}
	}

	p.x(X);
	p.y(Y);
	p.z(Z);
}

/*---------------------------------------------------------------
						getCovarianceAndMean
 ---------------------------------------------------------------*/
void CPointPDFSOG::getCovarianceAndMean(CMatrixDouble33 &estCov, CPoint3D &p) const
{
	size_t		N = m_modes.size();

	getMean(p);
	estCov.zeros();

	if (N)
	{
		// 1) Get the mean:
		double		sumW = 0;
		CMatrixDouble31	estMean = CMatrixDouble31(p);

		CListGaussianModes::const_iterator	it;

		CMatrixDouble33  partCov;

		for (it=m_modes.begin();it!=m_modes.end();++it)
		{
		    double w;
			sumW += w = exp(it->log_w);

			// estCov += w * ( it->val.cov + ((estMean_i-estMean)*(~(estMean_i-estMean))) );
			CMatrixDouble31 estMean_i = CMatrixDouble31(it->val.mean);
			estMean_i -=estMean;
			partCov.multiply_AAt(estMean_i);
			partCov+=it->val.cov;
			partCov*=w;
			estCov += partCov;
		}

		if (sumW!=0)
			estCov *= (1.0/sumW);
	}
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPointPDFSOG::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		uint32_t	N = m_modes.size();
		CListGaussianModes::const_iterator		it;

		out << N;

		for (it=m_modes.begin();it!=m_modes.end();++it)
		{
			out << it->log_w;
			out << it->val.mean;
			out << it->val.cov(0,0) << it->val.cov(1,1) << it->val.cov(2,2);
			out << it->val.cov(0,1) << it->val.cov(0,2) << it->val.cov(1,2);
		}
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPointPDFSOG::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			uint32_t							N;
			CListGaussianModes::iterator		it;
			double								x;

			in >> N;

			this->resize(N);

			for (it=m_modes.begin();it!=m_modes.end();++it)
			{
				in >> it->log_w;

				// In version 0, weights were linear!!
				if (version==0) it->log_w = log(max(1e-300,it->log_w));

				in >> it->val.mean;

				in >> x; it->val.cov(0,0) = x;
				in >> x; it->val.cov(1,1) = x;
				in >> x; it->val.cov(2,2) = x;

				in >> x; it->val.cov(1,0) = x; it->val.cov(0,1) = x;
				in >> x; it->val.cov(2,0) = x; it->val.cov(0,2) = x;
				in >> x; it->val.cov(1,2) = x; it->val.cov(2,1) = x;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

void  CPointPDFSOG::copyFrom(const CPointPDF &o)
{
	MRPT_START

	if (this == &o) return;		// It may be used sometimes

	if (o.GetRuntimeClass()==CLASS_ID(CPointPDFSOG))
	{
		m_modes = static_cast<const CPointPDFSOG*>(&o)->m_modes;
	}
	else
	{
		// Approximate as a mono-modal gaussian pdf:
		this->resize(1);
		m_modes[0].log_w = 0;
		o.getCovarianceAndMean(m_modes[0].val.cov,m_modes[0].val.mean);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						saveToTextFile
  ---------------------------------------------------------------*/
void  CPointPDFSOG::saveToTextFile(const std::string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;


	for (CListGaussianModes::const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		os::fprintf(f,"%e %e %e %e %e %e %e %e %e %e\n",
			exp(it->log_w),
			it->val.mean.x(), it->val.mean.y(), it->val.mean.z(),
			it->val.cov(0,0),it->val.cov(1,1),it->val.cov(2,2),
			it->val.cov(0,1),it->val.cov(0,2),it->val.cov(1,2) );
	os::fclose(f);
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPointPDFSOG::changeCoordinatesReference(const CPose3D &newReferenceBase )
{
	for (CListGaussianModes::iterator it=m_modes.begin();it!=m_modes.end();++it)
		it->val.changeCoordinatesReference( newReferenceBase );
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void CPointPDFSOG::drawSingleSample(CPoint3D  &outSample) const
{
	MRPT_START

	ASSERT_(m_modes.size()>0);


	// 1st: Select a mode with a probability proportional to its weight:
	vector<double>				logWeights( m_modes.size() );
	vector<size_t>				outIdxs;
	vector<double>::iterator 	itW;
	CListGaussianModes::const_iterator it;
	for (it=m_modes.begin(),itW=logWeights.begin();it!=m_modes.end();++it,++itW)
		*itW = it->log_w;

	CParticleFilterCapable::computeResampling(
		CParticleFilter::prMultinomial, // Resampling algorithm
		logWeights,                     // input: log weights
		outIdxs                         // output: indexes
		);

	// we need just one: take the first (arbitrary)
	size_t   selectedIdx = outIdxs[0];
	ASSERT_(selectedIdx<m_modes.size());
	const CPointPDFGaussian* selMode = & m_modes[selectedIdx].val;


	// 2nd: Draw a position from the selected Gaussian:
	CVectorDouble vec;
	randomGenerator.drawGaussianMultivariate(vec,selMode->cov);

	ASSERT_(vec.size()==3);
	outSample.x( selMode->mean.x() + vec[0] );
	outSample.y( selMode->mean.y() + vec[1] );
	outSample.z( selMode->mean.z() + vec[2] );

	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPointPDFSOG::bayesianFusion(const  CPointPDF &p1_, const CPointPDF &p2_,const double &minMahalanobisDistToDrop )
{
	MRPT_START

	// p1: CPointPDFSOG, p2: CPosePDFGaussian:

	ASSERT_( p1_.GetRuntimeClass() == CLASS_ID(CPointPDFSOG) );
	ASSERT_( p2_.GetRuntimeClass() == CLASS_ID(CPointPDFSOG) );

	const CPointPDFSOG		*p1 = static_cast<const CPointPDFSOG*>( &p1_ );
	const CPointPDFSOG		*p2 = static_cast<const CPointPDFSOG*>( &p2_ );

	// Compute the new kernel means, covariances, and weights after multiplying to the Gaussian "p2":
	CPointPDFGaussian	auxGaussianProduct,auxSOG_Kernel_i;

	float			minMahalanobisDistToDrop2 = square(minMahalanobisDistToDrop);


	this->m_modes.clear();
	bool is2D = false; // to detect & avoid errors in 3x3 matrix inversions of range=2.

	for (CListGaussianModes::const_iterator it1 =p1->m_modes.begin();it1!=p1->m_modes.end();++it1)
	{
		CMatrixDouble33	c = it1->val.cov;

		// Is a 2D covariance??
		if (c.get_unsafe(2,2)==0)
		{
			is2D = true;
			c.set_unsafe(2,2,1);
		}

		ASSERT_( c(0,0)!=0 && c(0,0)!=0 )

		CMatrixDouble33	 covInv(UNINITIALIZED_MATRIX);
		c.inv(covInv);

		CMatrixDouble31	 eta = covInv * CMatrixDouble31(it1->val.mean);

		// Normal distribution canonical form constant:
		// See: http://www-static.cc.gatech.edu/~wujx/paper/Gaussian.pdf
		double a = -0.5*( 3*log(M_2PI) - log( covInv.det() ) +
		            eta.multiply_HtCH_scalar(c)); // (~eta * (*it1).val.cov * eta)(0,0) );

		for (CListGaussianModes::const_iterator it2 =p2->m_modes.begin();it2!=p2->m_modes.end();++it2)
		{
			auxSOG_Kernel_i = (*it2).val;
			if (auxSOG_Kernel_i.cov.get_unsafe(2,2)==0) { auxSOG_Kernel_i.cov.set_unsafe(2,2,1); is2D=true; }
			ASSERT_(auxSOG_Kernel_i.cov(0,0)>0 && auxSOG_Kernel_i.cov(1,1)>0 )


			// Should we drop this product term??
			bool reallyComputeThisOne = true;
			if (minMahalanobisDistToDrop>0)
			{
				// Approximate (fast) mahalanobis distance (square):
				float mahaDist2;

				float stdX2 = max(auxSOG_Kernel_i.cov.get_unsafe(0,0) , (*it1).val.cov.get_unsafe(0,0));
				mahaDist2 = square( auxSOG_Kernel_i.mean.x() - (*it1).val.mean.x() )/stdX2;

				float stdY2 = max(auxSOG_Kernel_i.cov.get_unsafe(1,1), (*it1).val.cov.get_unsafe(1,1));
				mahaDist2 += square( auxSOG_Kernel_i.mean.y() - (*it1).val.mean.y() )/stdY2;

				if (!is2D)
				{
					float stdZ2 = max( auxSOG_Kernel_i.cov.get_unsafe(2,2), (*it1).val.cov.get_unsafe(2,2) );
					mahaDist2 += square( auxSOG_Kernel_i.mean.z() - (*it1).val.mean.z() )/stdZ2;
				}

				reallyComputeThisOne = mahaDist2 < minMahalanobisDistToDrop2;
			}

			if (reallyComputeThisOne)
			{
				auxGaussianProduct.bayesianFusion( auxSOG_Kernel_i, (*it1).val );

				// ----------------------------------------------------------------------
				// The new weight is given by:
				//
				//   w'_i = w_i * exp( a + a_i - a' )
				//
				//      a = -1/2 ( dimensionality * log(2pi) - log(det(Cov^-1)) + (Cov^-1 * mu)^t * Cov^-1 * (Cov^-1 * mu) )
				//
				// ----------------------------------------------------------------------
				TGaussianMode	newKernel;

				newKernel.val = auxGaussianProduct; // Copy mean & cov

				CMatrixDouble33		covInv_i= auxSOG_Kernel_i.cov.inv();
				CMatrixDouble31		eta_i = CMatrixDouble31(auxSOG_Kernel_i.mean);
				eta_i = covInv_i * eta_i;

				CMatrixDouble33		new_covInv_i = newKernel.val.cov.inv();
				CMatrixDouble31		new_eta_i = CMatrixDouble31(newKernel.val.mean);
				new_eta_i = new_covInv_i * new_eta_i;

				double		a_i	    = -0.5*( 3*log(M_2PI) - log( new_covInv_i.det() ) + (eta_i.adjoint() * auxSOG_Kernel_i.cov * eta_i)(0,0) );
				double		new_a_i = -0.5*( 3*log(M_2PI) - log( new_covInv_i.det() ) + (new_eta_i.adjoint() * newKernel.val.cov * new_eta_i)(0,0) );

				newKernel.log_w	   = (it1)->log_w + (it2)->log_w + a + a_i - new_a_i ;

				// Fix 2D case:
				if (is2D) newKernel.val.cov(2,2)=0;

				// Add to the results (in "this") the new kernel:
				this->m_modes.push_back( newKernel );
			} // end if reallyComputeThisOne
		} // end for it2

	} // end for it1

	normalizeWeights();

	MRPT_END
}

/*---------------------------------------------------------------
						assureSymmetry
 ---------------------------------------------------------------*/
void  CPointPDFSOG::assureSymmetry()
{
	MRPT_START
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (CListGaussianModes::iterator it=m_modes.begin();it!=m_modes.end();++it)
	{
		it->val.cov(0,1) = it->val.cov(1,0);
		it->val.cov(0,2) = it->val.cov(2,0);
		it->val.cov(1,2) = it->val.cov(2,1);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						normalizeWeights
 ---------------------------------------------------------------*/
void  CPointPDFSOG::normalizeWeights()
{
	MRPT_START

	if (!m_modes.size()) return;

	CListGaussianModes::iterator		it;
	double		maxW = m_modes[0].log_w;
	for (it=m_modes.begin();it!=m_modes.end();++it)
		maxW = max(maxW,it->log_w);

	for (it=m_modes.begin();it!=m_modes.end();++it)
		it->log_w -= maxW;

	MRPT_END
}

/*---------------------------------------------------------------
						ESS
 ---------------------------------------------------------------*/
double CPointPDFSOG::ESS() const
{
	MRPT_START
	CListGaussianModes::const_iterator	it;
	double	cum = 0;

	/* Sum of weights: */
	double sumLinearWeights = 0;
	for (it=m_modes.begin();it!=m_modes.end();++it) sumLinearWeights += exp(it->log_w);

	/* Compute ESS: */
	for (it=m_modes.begin();it!=m_modes.end();++it)
		cum+= square( exp(it->log_w) / sumLinearWeights );

	if (cum==0)
			return 0;
	else	return 1.0/(m_modes.size()*cum);
	MRPT_END
}

/*---------------------------------------------------------------
						evaluatePDFInArea
 ---------------------------------------------------------------*/
void  CPointPDFSOG::evaluatePDFInArea(
	float		x_min,
	float		x_max,
	float		y_min,
	float		y_max,
	float		resolutionXY,
	float		z,
	CMatrixD	&outMatrix,
	bool		sumOverAllZs )
{
	MRPT_START

	ASSERT_(x_max>x_min);
	ASSERT_(y_max>y_min);
	ASSERT_(resolutionXY>0);

	const size_t Nx = (size_t)ceil((x_max-x_min)/resolutionXY);
	const size_t Ny = (size_t)ceil((y_max-y_min)/resolutionXY);
	outMatrix.setSize(Ny,Nx);

	for (size_t i=0;i<Ny;i++)
	{
		const float y = y_min + i*resolutionXY;
		for (size_t j=0;j<Nx;j++)
		{
			float x = x_min + j*resolutionXY;
			outMatrix(i,j) = evaluatePDF(CPoint3D(x,y,z),sumOverAllZs);
		}
	}


	MRPT_END
}


/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double  CPointPDFSOG::evaluatePDF(
	const	CPoint3D &x,
	bool	sumOverAllZs ) const
{
	if (!sumOverAllZs)
	{
		// Normal evaluation:
		CMatrixDouble31 X = CMatrixDouble31(x);
		double	ret = 0;

		CMatrixDouble31 MU;

		for (CListGaussianModes::const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		{
			MU = CMatrixDouble31(it->val.mean);
			ret+= exp(it->log_w) * math::normalPDF( X, MU, it->val.cov );
		}

		return ret;
	}
	else
	{
		// Only X,Y:
		CMatrixD	X(2,1), MU(2,1),COV(2,2);
		double	ret = 0;

		X(0,0) = x.x();
		X(1,0) = x.y();

		for (CListGaussianModes::const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		{
			MU(0,0) = it->val.mean.x();
			MU(1,0) = it->val.mean.y();

			COV(0,0) = it->val.cov(0,0);
			COV(1,1) = it->val.cov(1,1);
			COV(0,1) = COV(1,0) = it->val.cov(0,1);

			ret+= exp(it->log_w) * math::normalPDF( X, MU, COV );
		}

		return ret;
	}
}

/*---------------------------------------------------------------
						getMostLikelyMode
 ---------------------------------------------------------------*/
void CPointPDFSOG::getMostLikelyMode( CPointPDFGaussian& outVal ) const
{
	if (this->empty())
	{
		outVal = CPointPDFGaussian();
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

/*---------------------------------------------------------------
						getAs3DObject
 ---------------------------------------------------------------*/
//void  CPointPDFSOG::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
//{
//	// For each gaussian node
//	for (CListGaussianModes::const_iterator it = m_modes.begin(); it!= m_modes.end();++it)
//	{
//		opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();
//
//		obj->setPose( it->val.mean);
//		obj->setCovMatrix(it->val.cov,  it->val.cov(2,2)==0  ?  2:3);
//
//		obj->setQuantiles(3);
//		obj->enableDrawSolid3D(false);
//		obj->setColor(1,0,0, 0.5);
//
//		outObj->insert( obj );
//	} // end for each gaussian node
//}
