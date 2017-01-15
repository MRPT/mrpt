/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/system/os.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/poses/SO_SE_average.h>


using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CPosePDFSOG, CPosePDF, mrpt::poses )


/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFSOG::CPosePDFSOG( size_t nModes ) : m_modes(nModes)
{
}

/*---------------------------------------------------------------
			clear
  ---------------------------------------------------------------*/
void CPosePDFSOG::clear()
{
	m_modes.clear();
}

/*---------------------------------------------------------------
	Resize
  ---------------------------------------------------------------*/
void CPosePDFSOG::resize(const size_t N)
{
	m_modes.resize(N);
}


/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF)
 ---------------------------------------------------------------*/
void CPosePDFSOG::getMean(CPose2D &p) const
{
	if (!m_modes.empty())
	{
		mrpt::poses::SE_average<2> se_averager;
		for (const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		{
			const double w = exp((it)->log_w);
			se_averager.append( (it)->mean, w);
		}
		se_averager.get_average(p);
	}
	else {
		p = CPose2D();
	}
}

/*---------------------------------------------------------------
						getEstimatedCovariance
 ---------------------------------------------------------------*/
void CPosePDFSOG::getCovarianceAndMean(CMatrixDouble33 &estCov, CPose2D &estMean2D) const
{
	size_t		N = m_modes.size();

	this->getMean(estMean2D);
	estCov.zeros();

	if (N)
	{
		// 1) Get the mean:
		double sumW = 0;
		CMatrixDouble31	estMeanMat = CMatrixDouble31(estMean2D);
		CMatrixDouble33 temp;
		CMatrixDouble31 estMean_i;

		for (const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		{
		    double w;
			sumW += w = exp((it)->log_w);

			estMean_i = CMatrixDouble31( (it)->mean );
			estMean_i -= estMeanMat;

			temp.multiply_AAt(estMean_i);
			temp+= (it)->cov;
			temp*=w;

			estCov += temp;
		}

		if (sumW!=0)
			estCov *= (1.0/sumW);
	}
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPosePDFSOG::writeToStream(mrpt::utils::CStream &out,int *version) const
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
			out << (it)->mean;
			out << (it)->cov(0,0) << (it)->cov(1,1) << (it)->cov(2,2);
			out << (it)->cov(0,1) << (it)->cov(0,2) << (it)->cov(1,2);
		}
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPosePDFSOG::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			uint32_t	N;
			float		x;
			double		x0;

			in >> N;

			resize(N);

			for (iterator it=m_modes.begin();it!=m_modes.end();++it)
			{
				in >> (it)->log_w;

				// In version 0, weights were linear!!
				if (version==0) (it)->log_w = log(max(1e-300,(it)->log_w));

				in >> (it)->mean;

				if (version==1)  // float's
				{
					in >> x; (it)->cov(0,0) = x;
					in >> x; (it)->cov(1,1) = x;
					in >> x; (it)->cov(2,2) = x;

					in >> x; (it)->cov(1,0) = x; (it)->cov(0,1) = x;
					in >> x; (it)->cov(2,0) = x; (it)->cov(0,2) = x;
					in >> x; (it)->cov(1,2) = x; (it)->cov(2,1) = x;
				}
				else
				{
					in >> x0; (it)->cov(0,0) = x0;
					in >> x0; (it)->cov(1,1) = x0;
					in >> x0; (it)->cov(2,2) = x0;

					in >> x0; (it)->cov(1,0) = x0; (it)->cov(0,1) = x0;
					in >> x0; (it)->cov(2,0) = x0; (it)->cov(0,2) = x0;
					in >> x0; (it)->cov(1,2) = x0; (it)->cov(2,1) = x0;
				}

			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

void  CPosePDFSOG::copyFrom(const CPosePDF &o)
{
	MRPT_START

	if (this == &o) return;		// It may be used sometimes

	if (o.GetRuntimeClass()==CLASS_ID(CPosePDFSOG))
	{
		m_modes = static_cast<const CPosePDFSOG*>(&o)->m_modes;
	}
	else
	{
		// Approximate as a mono-modal gaussian pdf:
		m_modes.resize(1);
		m_modes[0].log_w = 0;
		o.getMean( m_modes[0].mean );
		o.getCovariance( m_modes[0].cov );
	}

	MRPT_END
}

/*---------------------------------------------------------------
						saveToTextFile
  ---------------------------------------------------------------*/
void  CPosePDFSOG::saveToTextFile(const std::string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;


	for (const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		os::fprintf(f,"%e %e %e %e %e %e %e %e %e %e\n",
			exp((it)->log_w),
			(it)->mean.x(), (it)->mean.y(), (it)->mean.phi(),
			(it)->cov(0,0),(it)->cov(1,1),(it)->cov(2,2),
			(it)->cov(0,1),(it)->cov(0,2),(it)->cov(1,2) );
	os::fclose(f);
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPosePDFSOG::changeCoordinatesReference(const CPose3D &newReferenceBase_ )
{
	const CPose2D newReferenceBase = CPose2D(newReferenceBase_);

	CMatrixDouble44 HM;
	newReferenceBase.getHomogeneousMatrix(HM);

	// Clip the 4x4 matrix
	CMatrixDouble33	M = HM.block(0,0, 3,3).eval();

	// The variance in phi is unmodified:
	M(0,2) = 0; M(1,2) = 0;
	M(2,0) = 0; M(2,1) = 0;
	M(2,2) = 1;

	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
	{
		// The mean:
		(it)->mean.composeFrom(newReferenceBase, (it)->mean);

		// The covariance:
		// NOTE: The CMatrixDouble33() is NEEDED to create a temporary copy of (it)->cov
		M.multiply_HCHt( CMatrixDouble33((it)->cov), (it)->cov );  // * (it)->cov * (~M);
	}

	assureSymmetry();
}

/*---------------------------------------------------------------
						rotateAllCovariances
 ---------------------------------------------------------------*/
void  CPosePDFSOG::rotateAllCovariances(const double & ang)
{
	CMatrixDouble33		rot;
	rot(0,0)=rot(1,1)=cos(ang);
	rot(0,1)=-sin(ang);
	rot(1,0)=sin(ang);
	rot(2,2)=1;

	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
		rot.multiply_HCHt( CMatrixDouble33((it)->cov), (it)->cov );
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPosePDFSOG::drawSingleSample( CPose2D &outPart ) const
{
	MRPT_START
	MRPT_UNUSED_PARAM(outPart);

	THROW_EXCEPTION("Not implemented yet!!");

	MRPT_END
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void  CPosePDFSOG::drawManySamples(
	size_t						N,
	std::vector<CVectorDouble>	&outSamples )  const
{
	MRPT_START
	MRPT_UNUSED_PARAM(N);
	MRPT_UNUSED_PARAM(outSamples);

	THROW_EXCEPTION("Not implemented yet!!");

	MRPT_END
}


/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPosePDFSOG::bayesianFusion(const  CPosePDF &p1_,const  CPosePDF &p2_, const double &minMahalanobisDistToDrop )
{
	MRPT_START

	MRPT_UNUSED_PARAM(minMahalanobisDistToDrop);

	// p1: CPosePDFSOG, p2: CPosePDFGaussian:

	ASSERT_( p1_.GetRuntimeClass() == CLASS_ID(CPosePDFSOG) );
	ASSERT_( p2_.GetRuntimeClass() == CLASS_ID(CPosePDFGaussian) );

	const CPosePDFSOG			*p1 = static_cast<const CPosePDFSOG*>(&p1_);
	const CPosePDFGaussian		*p2 = static_cast<const CPosePDFGaussian*>(&p2_);

	// Compute the new kernel means, covariances, and weights after multiplying to the Gaussian "p2":
	CPosePDFGaussian	auxGaussianProduct,auxSOG_Kernel_i;

	CMatrixDouble33	covInv;
	p2->cov.inv(covInv);

	CMatrixDouble31	eta = CMatrixDouble31(p2->mean);
	eta = covInv * eta;

	// Normal distribution canonical form constant:
	// See: http://www-static.cc.gatech.edu/~wujx/paper/Gaussian.pdf
	double				a = -0.5*( 3*log(M_2PI) - log( covInv.det() ) + (eta.adjoint() * p2->cov * eta)(0,0) );

	this->m_modes.clear();
	for (const_iterator it =p1->m_modes.begin();it!=p1->m_modes.end();++it)
	{
		auxSOG_Kernel_i.mean = (it)->mean;
		auxSOG_Kernel_i.cov  = CMatrixDouble( (it)->cov );
		auxGaussianProduct.bayesianFusion( auxSOG_Kernel_i, *p2 );

		// ----------------------------------------------------------------------
		// The new weight is given by:
		//
		//   w'_i = w_i * exp( a + a_i - a' )
		//
		//      a = -1/2 ( dimensionality * log(2pi) - log(det(Cov^-1)) + (Cov^-1 * mu)^t * Cov^-1 * (Cov^-1 * mu) )
		//
		// ----------------------------------------------------------------------
		TGaussianMode		newKernel;
		newKernel.mean = auxGaussianProduct.mean;
		newKernel.cov  = auxGaussianProduct.cov;

		CMatrixDouble33	covInv_i;
		auxSOG_Kernel_i.cov.inv(covInv_i);

		CMatrixDouble31	eta_i = CMatrixDouble31(auxSOG_Kernel_i.mean);
		eta_i = covInv_i * eta_i;

		CMatrixDouble33 new_covInv_i;
		newKernel.cov.inv(new_covInv_i);

		CMatrixDouble31	new_eta_i = CMatrixDouble31(newKernel.mean);
		new_eta_i = new_covInv_i * new_eta_i;

		double		a_i	    = -0.5*( 3*log(M_2PI) - log( new_covInv_i.det() ) + (eta_i.adjoint() * auxSOG_Kernel_i.cov * eta_i)(0,0) );
		double		new_a_i = -0.5*( 3*log(M_2PI) - log( new_covInv_i.det() ) + (new_eta_i.adjoint() * newKernel.cov * new_eta_i)(0,0) );

		//newKernel.w	   = (it)->w * exp( a + a_i - new_a_i );
		newKernel.log_w	   = (it)->log_w + a + a_i - new_a_i;

		// Add to the results (in "this") the new kernel:
		this->m_modes.push_back(newKernel );
	}

	normalizeWeights();

	MRPT_END
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void	 CPosePDFSOG::inverse(CPosePDF &o)  const
{
	ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPosePDFSOG));
	CPosePDFSOG	*out = static_cast<CPosePDFSOG*>( &o );

	const_iterator	itSrc;
	iterator			itDest;

	out->m_modes.resize(m_modes.size());

	for (itSrc=m_modes.begin(),itDest=out->m_modes.begin();itSrc!=m_modes.end();++itSrc,++itDest)
	{
		// The mean:
		(itDest)->mean = -(itSrc)->mean;

		// The covariance: Is the same:
		(itDest)->cov = (itSrc)->cov;
	}
}


/*---------------------------------------------------------------
							+=
 ---------------------------------------------------------------*/
void  CPosePDFSOG::operator += ( const CPose2D &Ap)
{
	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
		(it)->mean = (it)->mean + Ap;

	this->rotateAllCovariances( Ap.phi() );
}

/*---------------------------------------------------------------
						evaluatePDF
 ---------------------------------------------------------------*/
double  CPosePDFSOG::evaluatePDF(
	const	CPose2D &x,
	bool	sumOverAllPhis ) const
{
	if (!sumOverAllPhis)
	{
		// Normal evaluation:
		CMatrixDouble31	X = CMatrixDouble31(x);
		CMatrixDouble31	MU;
		double	ret = 0;

		for (const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		{
			MU = CMatrixDouble31((it)->mean);
			ret+= exp((it)->log_w) * math::normalPDF( X, MU, (it)->cov );
		}

		return ret;
	}
	else
	{
		// Only X,Y:
		CMatrixDouble	X(2,1), MU(2,1),COV(2,2);
		double	ret = 0;

		X(0,0) = x.x();
		X(1,0) = x.y();

		for (const_iterator it=m_modes.begin();it!=m_modes.end();++it)
		{
			MU(0,0) = (it)->mean.x();
			MU(1,0) = (it)->mean.y();

			COV(0,0) = (it)->cov(0,0);
			COV(1,1) = (it)->cov(1,1);
			COV(0,1) = COV(1,0) = (it)->cov(0,1);

			ret+= exp((it)->log_w) * math::normalPDF( X, MU, COV );
		}

		return ret;
	}
}

/*---------------------------------------------------------------
						evaluateNormalizedPDF
 ---------------------------------------------------------------*/
double  CPosePDFSOG::evaluateNormalizedPDF( const CPose2D &x ) const
{
	CMatrixDouble31	X = CMatrixDouble31(x);
	CMatrixDouble31	MU;
	double	ret = 0;

	for (const_iterator it=m_modes.begin();it!=m_modes.end();++it)
	{
		MU = CMatrixDouble31((it)->mean);
		ret+= exp((it)->log_w) * math::normalPDF( X, MU, (it)->cov ) / math::normalPDF( MU, MU, (it)->cov );
	}

	return ret;
}

/*---------------------------------------------------------------
						assureSymmetry
 ---------------------------------------------------------------*/
void  CPosePDFSOG::assureSymmetry()
{
	// Differences, when they exist, appear in the ~15'th significant
	//  digit, so... just take one of them arbitrarily!
	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
	{
		(it)->cov(0,1) = (it)->cov(1,0);
		(it)->cov(0,2) = (it)->cov(2,0);
		(it)->cov(1,2) = (it)->cov(2,1);
	}
}

/*---------------------------------------------------------------
						normalizeWeights
 ---------------------------------------------------------------*/
void  CPosePDFSOG::normalizeWeights()
{
	MRPT_START

	if (!m_modes.size()) return;

	double maxW = m_modes[0].log_w;
	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
		maxW = max(maxW,(it)->log_w);

	for (iterator it=m_modes.begin();it!=m_modes.end();++it)
		(it)->log_w -= maxW;

	MRPT_END
}

/*---------------------------------------------------------------
						normalizeWeights
 ---------------------------------------------------------------*/
void  CPosePDFSOG::evaluatePDFInArea(
	const double &	x_min,
	const double &		x_max,
	const double &		y_min,
	const double &		y_max,
	const double &		resolutionXY,
	const double &		phi,
	CMatrixD	&outMatrix,
	bool		sumOverAllPhis )
{
	MRPT_START

	ASSERT_(x_max>x_min);
	ASSERT_(y_max>y_min);
	ASSERT_(resolutionXY>0);

	const size_t		Nx = (size_t)ceil((x_max-x_min)/resolutionXY);
	const size_t		Ny = (size_t)ceil((y_max-y_min)/resolutionXY);

	outMatrix.setSize(Ny,Nx);

	for (size_t i=0;i<Ny;i++)
	{
		double y = y_min + i*resolutionXY;
		for (size_t j=0;j<Nx;j++)
		{
			double x = x_min + j*resolutionXY;
			outMatrix(i,j) = evaluatePDF(CPose2D(x,y,phi),sumOverAllPhis);
		}
	}


	MRPT_END
}

/*---------------------------------------------------------------
						mergeModes
 ---------------------------------------------------------------*/
void CPosePDFSOG::mergeModes( double max_KLd, bool verbose  )
{
	MRPT_START

	normalizeWeights();

	size_t N = m_modes.size();
	if (N<2) return; // Nothing to do

	// Method described in:
	// "Kullback-Leibler Approach to Gaussian Mixture Reduction", A.R. Runnalls.
	// IEEE Transactions on Aerospace and Electronic Systems, 2007.
	//  See Eq.(21) for Bij !!

	for (size_t i=0;i<(N-1); )
	{
		N = m_modes.size(); // It might have changed.
		double sumW=0;

		// For getting linear weights:
		sumW = 0;
		for (size_t j=0;j<N;j++)
			sumW += exp(m_modes[j].log_w);
		ASSERT_(sumW);

		const double Wi = exp(m_modes[i].log_w) / sumW;

		double 	min_Bij = std::numeric_limits<double>::max();

		CMatrixDouble33  min_Bij_COV;
		size_t  best_j = 0;

		CMatrixDouble31  MUi = CMatrixDouble31(m_modes[i].mean);

		// Compute B(i,j), j=[i+1,N-1]  (the discriminant)
		for (size_t j=0;j<N;j++)
		if (i!=j)
		{
			const double Wj = exp(m_modes[j].log_w) / sumW;
			const double Wij_ = 1.0/(Wi+Wj);

			CMatrixDouble33  Pij = m_modes[i].cov * (Wi*Wij_);
			Pij.add_Ac(m_modes[j].cov, Wj*Wij_ );

			CMatrixDouble31  MUij = CMatrixDouble31(m_modes[j].mean);
			MUij-=MUi;
			// Account for circular dimensions:
			mrpt::math::wrapToPiInPlace( MUij(2,0) );

			CMatrixDouble33 AUX;
			AUX.multiply_AAt( MUij ); // AUX = MUij * MUij^T

			AUX *= Wi*Wj*Wij_*Wij_;
			Pij += AUX;

			double Bij = (Wi+Wj)*log( Pij.det() ) - Wi*log(m_modes[i].cov.det()) - Wj*log(m_modes[j].cov.det());
			if (verbose)
			{
				cout << "try merge[" << i << ", " << j << "] -> Bij: " << Bij << endl;
				//cout << "AUX: " << endl << AUX;
				//cout << "Wi: " << Wi << " Wj:" << Wj << " Wij_: " << Wij_ << endl;
				cout << "Pij: " << Pij << endl << " Pi: " << m_modes[i].cov << endl << " Pj: " << m_modes[j].cov << endl;
			}

			if (Bij<min_Bij)
			{
				min_Bij = Bij;
				best_j = j;
				min_Bij_COV = Pij;
			}
		}

		// Is a good move to merge (i,j)??
		if (verbose)
			cout << "merge[" << i << ", " << best_j << "] Tempting merge: KLd = " << min_Bij;

		if (min_Bij<max_KLd)
		{
			if (verbose)
				cout << " Accepted." << endl;

			// Do the merge (i,j):
			TGaussianMode	Mij;
			TGaussianMode	&Mi = m_modes[i];
			TGaussianMode	&Mj = m_modes[best_j];

			// Weight:
			Mij.log_w = log( exp(Mi.log_w) + exp(Mj.log_w) );

			// Mean:
			const double Wj = exp(Mj.log_w) / sumW;
			const double Wij_ = 1.0/(Wi+Wj);
			const double Wi_ = Wi*Wij_;
			const double Wj_ = Wj*Wij_;

			Mij.mean = CPose2D(
				Wi_ * Mi.mean.x() + Wj_ * Mj.mean.x(),
				Wi_ * Mi.mean.y() + Wj_ * Mj.mean.y(),
				Wi_ * Mi.mean.phi() + Wj_ * Mj.mean.phi() );

			// Cov:
			Mij.cov = min_Bij_COV;

			// Replace Mi <- Mij:
			m_modes[i] = Mij;
			m_modes.erase( m_modes.begin() + best_j );	// erase Mj
		} // end merge
		else
		{
			if (verbose)
				cout << " Nope." << endl;

			i++;
		}
	} // for i

	normalizeWeights();

	MRPT_END
}

/*---------------------------------------------------------------
						getMostLikelyCovarianceAndMean
 ---------------------------------------------------------------*/
void CPosePDFSOG::getMostLikelyCovarianceAndMean(CMatrixDouble33 &cov,CPose2D &mean_point) const
{
	const_iterator it_best = end();
	double best_log_w = -std::numeric_limits<double>::max();

	for (const_iterator i=begin();i!=end();++i)
	{
		if (i->log_w>best_log_w)
		{
			best_log_w = i->log_w;
			it_best = i;
		}
	}

	if (it_best!=end())
	{
		mean_point = it_best->mean;
		cov = it_best->cov;
	}
	else
	{
		cov.unit(3,1.0);
		cov*=1e20;
		mean_point = CPose2D(0,0,0);
	}
}


/*---------------------------------------------------------------
						getAs3DObject
 ---------------------------------------------------------------*/
//void  CPosePDFSOG::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
//{
//	outObj->clear();
//
//	for (const_iterator it=m_modes.begin();it!=m_modes.end();++it)
//	{
//		opengl::CEllipsoidPtr ellip = opengl::CEllipsoid::Create();
//
//		ellip->setPose( CPose3D((it)->mean.x(), (it)->mean.y(), (it)->mean.phi()) );
//		ellip->setCovMatrix((it)->cov);
//		ellip->setColor(0,0,1,0.6);
//
//		outObj->insert(ellip);
//	}
//
//}
