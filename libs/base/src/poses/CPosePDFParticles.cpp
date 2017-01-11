/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CTicTac.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/system/os.h>
#include <mrpt/random.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/distributions.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/poses/SO_SE_average.h>

using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CPosePDFParticles, CPosePDF, mrpt::poses )

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFParticles::CPosePDFParticles( size_t M )
{
	m_particles.resize(M);

	for (auto & p : m_particles)
	{
		p.log_w = .0;
		p.d.reset(new CPose2D());
	}

	CPose2D nullPose(0,0,0);
	resetDeterministic( nullPose );
}

void  CPosePDFParticles::copyFrom(const CPosePDF &o)
{
	MRPT_START

	CParticleList::iterator			itDest;
	CParticleList::const_iterator	itSrc;

	if (this == &o) return;		// It may be used sometimes

	if (o.GetRuntimeClass()==CLASS_ID(CPosePDFParticles))
	{
		const CPosePDFParticles	*pdf = dynamic_cast<const CPosePDFParticles*>( &o );
		ASSERT_(pdf);

		// Both are m_particles:
		m_particles = pdf->m_particles;
	}
	else
	if (o.GetRuntimeClass()==CLASS_ID(CPosePDFGaussian))
	{
		const CPosePDFGaussian	*pdf = static_cast<const CPosePDFGaussian*>( &o );
		size_t M = m_particles.size();
		std::vector<CVectorDouble>			parts;
		std::vector<CVectorDouble>::iterator partsIt;

		randomGenerator.drawGaussianMultivariateMany(parts,M,pdf->cov);

		clearParticles();
		m_particles.resize(M);

		for ( itDest = m_particles.begin(),partsIt=parts.begin();itDest!=m_particles.end();++itDest,++partsIt )
		{
			itDest->log_w = 0;
			itDest->d.reset(new CPose2D(
				(pdf->mean.x() + (*partsIt)[0]),
				(pdf->mean.y() + (*partsIt)[1]),
				(pdf->mean.phi() + (*partsIt)[2]))
			);
			itDest->d->normalizePhi();
		}

	}

	MRPT_END
}

void  CPosePDFParticles::clear( )
{
	clearParticles();
}

/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF), computed
		as a weighted average over all m_particles.
 ---------------------------------------------------------------*/
void CPosePDFParticles::getMean(CPose2D &est_) const
{
	// Calc average on SE(2)
	const size_t n = m_particles.size();
	if (n)
	{
		mrpt::poses::SE_average<2> se_averager;
		for (size_t i=0;i<n;i++)
		{
			const CPose2D &p  = *m_particles[i].d;
			double w  = exp(m_particles[i].log_w);
			se_averager.append(p,w);
		}
		se_averager.get_average(est_);
	}
	else
	{
		est_ = CPose2D();
	}
}

/*---------------------------------------------------------------
						getEstimatedCovariance
  ---------------------------------------------------------------*/
void CPosePDFParticles::getCovarianceAndMean(CMatrixDouble33 &cov, CPose2D &mean) const
{
	cov.zeros();
	getMean(mean);

	size_t		i,n = m_particles.size();
	double		var_x=0,var_y=0,var_p=0,var_xy=0,var_xp=0,var_yp=0;
	double		mean_phi = mean.phi();

	if (mean_phi<0) mean_phi = M_2PI + mean_phi;

	double lin_w_sum = 0;

	for (i=0;i<n;i++) lin_w_sum+= exp( m_particles[i].log_w );
	if (lin_w_sum==0) lin_w_sum=1;

	for (i=0;i<n;i++)
	{
		double w = exp( m_particles[i].log_w ) / lin_w_sum;

		// Manage 1 PI range:
		double	err_x   = m_particles[i].d->x() - mean.x();
		double	err_y   = m_particles[i].d->y() - mean.y();
		double	err_phi = math::wrapToPi( fabs(m_particles[i].d->phi() - mean_phi) );

		var_x+= square(err_x)*w;
		var_y+= square(err_y)*w;
		var_p+= square(err_phi)*w;
		var_xy+= err_x*err_y*w;
		var_xp+= err_x*err_phi*w;
		var_yp+= err_y*err_phi*w;
	}

	if (n<2)
	{
		// Not enought information to estimate the variance:
	}
	else
	{
		// Unbiased estimation of variance:
		cov(0,0) = var_x;
		cov(1,1) = var_y;
		cov(2,2) = var_p;

		cov(1,0) = cov(0,1) = var_xy;
		cov(2,0) = cov(0,2) = var_xp;
		cov(1,2) = cov(2,1) = var_yp;

	}
}


/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPosePDFParticles::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		writeParticlesToStream( out );
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPosePDFParticles::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			readParticlesFromStream( in );
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


/*---------------------------------------------------------------
							resetDeterministic
	Reset PDF to a single point and set the number of m_particles
 ---------------------------------------------------------------*/
void  CPosePDFParticles::resetDeterministic( const CPose2D &location,
										size_t	particlesCount)
{
	if (particlesCount>0)
	{
		clear();
		m_particles.resize(particlesCount);
		for (auto &p: m_particles)
			p.d.resetDefaultCtor();
	}

	for (auto &p: m_particles)
	{
		*p.d = location;
		p.log_w = .0;
	}
}

/*---------------------------------------------------------------
						resetUniform
 ---------------------------------------------------------------*/
void  CPosePDFParticles::resetUniform(
	const double & x_min,
	const double & x_max,
	const double & y_min,
	const double & y_max,
	const double & phi_min,
	const double & phi_max,
	const int	&particlesCount)
{
	MRPT_START

	if (particlesCount>0)
	{
		clear();
		m_particles.resize(particlesCount);
		for (int i = 0; i < particlesCount; i++)
			m_particles[i].d.reset(new CPose2D());
	}

	size_t		i,M = m_particles.size();
	for (i=0;i<M;i++)
	{
		m_particles[i].d->x( randomGenerator.drawUniform( x_min, x_max ) );
		m_particles[i].d->y( randomGenerator.drawUniform( y_min, y_max ) );
		m_particles[i].d->phi( randomGenerator.drawUniform( phi_min, phi_max ) );
		m_particles[i].log_w=0;
	}

	MRPT_END
}

void  CPosePDFParticles::resetAroundSetOfPoses(
	const std::vector<mrpt::math::TPose2D> & list_poses,
	const size_t num_particles_per_pose,
	const double spread_x,
	const double spread_y,
	const double spread_phi_rad)
{
	MRPT_START
	ASSERT_(!list_poses.empty());
	ASSERT_(num_particles_per_pose>=1);

	const size_t N = list_poses.size() * num_particles_per_pose;

	clear();
	m_particles.resize(N);
	size_t i,nSpot;
	for (i=0,nSpot=0;nSpot<list_poses.size();nSpot++)
	{
		const mrpt::math::TPose2D & p = list_poses[nSpot];
		for (size_t k=0;k<num_particles_per_pose;k++,i++)
		{
			m_particles[i].d.reset(new CPose2D());
			m_particles[i].d->x( randomGenerator.drawUniform( p.x - spread_x*0.5, p.x + spread_x*0.5 ) );
			m_particles[i].d->y( randomGenerator.drawUniform( p.y - spread_y*0.5, p.y + spread_y*0.5 ) );
			m_particles[i].d->phi( randomGenerator.drawUniform( p.phi - spread_phi_rad*0.5, p.phi + spread_phi_rad*0.5 ) );
			m_particles[i].log_w=0;
		}
	}

	ASSERT_EQUAL_(i,N);

	MRPT_END
}

/*---------------------------------------------------------------
						saveToTextFile
   Save PDF's m_particles to a text file. In each line it
      will go: "x y phi weight"
 ---------------------------------------------------------------*/
void  CPosePDFParticles::saveToTextFile(const std::string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;
	os::fprintf(f,"%% x  y  yaw[rad] log_weight\n");

	for (unsigned int i=0;i<m_particles.size();i++)
		os::fprintf(f,"%f %f %f %e\n",
				m_particles[i].d->x(),
				m_particles[i].d->y(),
				m_particles[i].d->phi(),
				m_particles[i].log_w );

	os::fclose(f);
}

/*---------------------------------------------------------------
						getParticlePose
 ---------------------------------------------------------------*/
CPose2D	 CPosePDFParticles::getParticlePose(size_t i) const
{
	return *m_particles[i].d;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPosePDFParticles::changeCoordinatesReference(const CPose3D &newReferenceBase_ )
{
	const CPose2D newReferenceBase = CPose2D(newReferenceBase_);

	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
		it->d->composeFrom(newReferenceBase, *it->d);
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPosePDFParticles::drawSingleSample( CPose2D &outPart ) const
{
	const double uni = randomGenerator.drawUniform(0.0,0.9999);
	double cum = 0;

	for (CParticleList::const_iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		cum+= exp(it->log_w);
		if ( uni<= cum )
		{
			outPart= (*it->d);
			return;
		}
	}

	// Might not come here normally:
	outPart = *(m_particles.rbegin())->d;
}

/*---------------------------------------------------------------
						+=
 ---------------------------------------------------------------*/
void  CPosePDFParticles::operator += ( const CPose2D &Ap)
{
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
		it->d->composeFrom(*it->d, Ap);
}

/*---------------------------------------------------------------
					append
 ---------------------------------------------------------------*/
void  CPosePDFParticles::append( CPosePDFParticles &o )
{
	for (unsigned int i=0;i<o.m_particles.size();i++)
	{
		CParticleData		part;
		part.d.reset(new CPose2D(*o.m_particles[i].d));
		part.log_w = o.m_particles[i].log_w;
		m_particles.push_back( part );
	}

	normalizeWeights();
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void	 CPosePDFParticles::inverse(CPosePDF  &o) const
{
	MRPT_START
	ASSERT_( o.GetRuntimeClass() == CLASS_ID(CPosePDFParticles) );
	CPosePDFParticles	*out = static_cast<CPosePDFParticles*>( &o );


	out->copyFrom( *this );
	static CPose2D		nullPose(0,0,0);

	for (unsigned int i=0;i<out->m_particles.size();i++)
		(*out->m_particles[i].d) = nullPose - (*out->m_particles[i].d);

	MRPT_END
}

/*---------------------------------------------------------------
					getMostLikelyParticle
 ---------------------------------------------------------------*/
CPose2D	 CPosePDFParticles::getMostLikelyParticle() const
{
	CParticleList::const_iterator	it, itMax=m_particles.begin();
	double		max_w = -1e300;


	for (it=m_particles.begin();it!=m_particles.end();++it)
	{
		if (it->log_w > max_w)
		{
			itMax = it;
			max_w = it->log_w;
		}
	}

	return *itMax->d;
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPosePDFParticles::bayesianFusion(const  CPosePDF &p1,const  CPosePDF &p2, const double &minMahalanobisDistToDrop  )
{
	MRPT_UNUSED_PARAM(p1);MRPT_UNUSED_PARAM(p2);MRPT_UNUSED_PARAM(minMahalanobisDistToDrop);

	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					evaluatePDF_parzen
 ---------------------------------------------------------------*/
double  CPosePDFParticles::evaluatePDF_parzen(
	const double &	x,
	const double &	y,
	const double &	phi,
	const double &	stdXY,
	const double &	stdPhi ) const
{
	double	ret = 0;

	for (CParticleList::const_iterator	it=m_particles.begin();it!=m_particles.end();++it)
	{
		double difPhi = math::wrapToPi( phi - it->d->phi() );

		ret += exp(it->log_w) *
			   math::normalPDF( it->d->distance2DTo(x,y), 0, stdXY ) *
			   math::normalPDF( fabs( difPhi ), 0, stdPhi );
	}

	return ret;
}

/*---------------------------------------------------------------
					saveParzenPDFToTextFile
 ---------------------------------------------------------------*/
void  CPosePDFParticles::saveParzenPDFToTextFile(
	const char	*fileName,
	const double &		x_min,
	const double &		x_max,
	const double &		y_min,
	const double &		y_max,
	const double &		phi,
	const double &		stepSizeXY,
	const double &		stdXY,
	const double &		stdPhi ) const
{
	FILE	*f=os::fopen(fileName,"wt");
	if (!f) return;

	for (double y=y_min; y<y_max; y+=stepSizeXY)
	{
		for (double x=x_min; x<x_max; x+=stepSizeXY)
		{
			os::fprintf(f,"%f ",
				evaluatePDF_parzen(x,y,phi,stdXY,stdPhi) );
		} // y
		os::fprintf(f,"\n");
	} // x

	os::fclose(f);
}
