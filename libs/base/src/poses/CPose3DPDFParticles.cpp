/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE( CPose3DPDFParticles, CPose3DPDF, mrpt::poses )

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFParticles::CPose3DPDFParticles( size_t M )
{
	m_particles.resize(M);

	for (auto &it : m_particles)
	{
		it.log_w = .0;
		it.d.reset(new CPose3D());
	}

	CPose3D	nullPose(0,0,0);
	resetDeterministic( nullPose );
}

void  CPose3DPDFParticles::copyFrom(const CPose3DPDF &o)
{
	MRPT_START

	CPose3DPDFParticles::CParticleList::const_iterator	itSrc;
	CPose3DPDFParticles::CParticleList::iterator		itDest;

	if (this == &o) return;		// It may be used sometimes

	if (o.GetRuntimeClass()==CLASS_ID(CPose3DPDFParticles))
	{
		const CPose3DPDFParticles	*pdf = dynamic_cast<const CPose3DPDFParticles*>( &o );
		ASSERT_(pdf);

		m_particles = pdf->m_particles;
	}
	else
	if (o.GetRuntimeClass()==CLASS_ID(CPose3DPDFGaussian))
	{
		THROW_EXCEPTION("TO DO!!");
	}

	MRPT_END
}

/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF), computed
		as a weighted average over all m_particles.
 ---------------------------------------------------------------*/
void CPose3DPDFParticles::getMean(CPose3D &p) const
{
	MRPT_START

	// Default to (0,..,0)
	p = CPose3D();
	if (m_particles.empty())
		return;

	// Calc average on SE(3)
	mrpt::poses::SE_average<3> se_averager;
	for (CPose3DPDFParticles::CParticleList::const_iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		const double w  = exp(it->log_w);
		se_averager.append( *it->d,w );
	}
	se_averager.get_average(p);

	MRPT_END
}

/*---------------------------------------------------------------
						getCovarianceAndMean
  ---------------------------------------------------------------*/
void CPose3DPDFParticles::getCovarianceAndMean(CMatrixDouble66 &cov,CPose3D &mean) const
{
	MRPT_START

	getMean(mean);	// First! the mean value:

	// Now the covariance:
	cov.zeros();
	CVectorDouble	vars; vars.assign(6,0.0);	// The diagonal of the final covariance matrix

	// Elements off the diagonal of the covariance matrix:
	double											std_xy = 0,std_xz = 0,std_xya= 0,std_xp = 0,std_xr = 0;
	double											std_yz = 0,std_yya = 0,std_yp = 0,std_yr = 0;
	double											std_zya = 0,std_zp = 0,std_zr = 0;
	double											std_yap = 0,std_yar = 0;
	double											std_pr = 0;

	// Mean values in [0, 2pi] range:
	double		mean_yaw = mean.yaw();
	double		mean_pitch = mean.pitch();
	double		mean_roll = mean.roll();
	if (mean_yaw<0) mean_yaw += M_2PI;
	if (mean_pitch<0) mean_pitch += M_2PI;
	if (mean_roll<0) mean_roll += M_2PI;

	// Enought information to estimate the covariance?
	if (m_particles.size()<2)	return;


	// Sum all weight values:
	double		W = 0;
	for (CPose3DPDFParticles::CParticleList::const_iterator it=m_particles.begin();it!=m_particles.end();++it)
		W += exp(it->log_w);

	ASSERT_(W>0);

	// Compute covariance:
	for (CPose3DPDFParticles::CParticleList::const_iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		double w = exp( it->log_w ) / W;

		// Manage 1 PI range:
		double	err_yaw   = wrapToPi( fabs(it->d->yaw() - mean_yaw) );
		double	err_pitch = wrapToPi( fabs(it->d->pitch() - mean_pitch) );
		double	err_roll  = wrapToPi( fabs(it->d->roll() - mean_roll) );

		double	err_x	  = it->d->x() - mean.x();
		double	err_y	  = it->d->y() - mean.y();
		double	err_z	  = it->d->z() - mean.z();

		vars[0] += square(err_x)*w;
		vars[1] += square(err_y)*w;
		vars[2] += square(err_z)*w;
		vars[3] += square(err_yaw)*w;
		vars[4] += square(err_pitch)*w;
		vars[5] += square(err_roll)*w;

		std_xy	+= err_x*err_y * w;
		std_xz	+= err_x*err_z * w;
		std_xya	+= err_x*err_yaw * w;
		std_xp	+= err_x*err_pitch * w;
		std_xr	+= err_x*err_roll * w;

		std_yz	+= err_y*err_z * w;
		std_yya	+= err_y*err_yaw * w;
		std_yp	+= err_y*err_pitch * w;
		std_yr	+= err_y*err_roll * w;

		std_zya	+= err_z*err_yaw * w;
		std_zp	+= err_z*err_pitch * w;
		std_zr	+= err_z*err_roll * w;

		std_yap	+= err_yaw*err_pitch * w;
		std_yar	+= err_yaw*err_roll * w;

		std_pr	+= err_pitch*err_roll * w;
	} // end for it

	// Unbiased estimation of variance:
	cov(0,0) = vars[0];
	cov(1,1) = vars[1];
	cov(2,2) = vars[2];
	cov(3,3) = vars[3];
	cov(4,4) = vars[4];
	cov(5,5) = vars[5];

	cov(1,0) = cov(0,1) = std_xy;
	cov(2,0) = cov(0,2) = std_xz;
	cov(3,0) = cov(0,3) = std_xya;
	cov(4,0) = cov(0,4) = std_xp;
	cov(5,0) = cov(0,5) = std_xr;

	cov(2,1) = cov(1,2) = std_yz;
	cov(3,1) = cov(1,3) = std_yya;
	cov(4,1) = cov(1,4) = std_yp;
	cov(5,1) = cov(1,5) = std_yr;

	cov(3,2) = cov(2,3) = std_zya;
	cov(4,2) = cov(2,4) = std_zp;
	cov(5,2) = cov(2,5) = std_zr;

	cov(4,3) = cov(3,4) = std_yap;
	cov(5,3) = cov(3,5) = std_yar;

	cov(5,4) = cov(4,5) = std_pr;


	MRPT_END
}


/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPose3DPDFParticles::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		writeParticlesToStream(out);
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPose3DPDFParticles::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			readParticlesFromStream(in);
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
						saveToTextFile
   Save PDF's m_particles to a text file. In each line it
      will go: "x y phi weight"
 ---------------------------------------------------------------*/
void  CPose3DPDFParticles::saveToTextFile(const std::string &file) const
{
	using namespace mrpt::system;

	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;

	os::fprintf(f,"%% x  y  z  yaw[rad] pitch[rad] roll[rad] log_weight\n");

	for (const auto &p : m_particles)
		os::fprintf(f,"%f %f %f %f %f %f %e\n",
				p.d->x(),
				p.d->y(),
				p.d->z(),
				p.d->yaw(),
				p.d->pitch(),
				p.d->roll(),
				p.log_w );

	os::fclose(f);
}


/*---------------------------------------------------------------
						getParticlePose
 ---------------------------------------------------------------*/
CPose3D	 CPose3DPDFParticles::getParticlePose(int i) const
{
	return *m_particles[i].d;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPose3DPDFParticles::changeCoordinatesReference( const CPose3D &newReferenceBase )
{
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
		it->d->composeFrom(newReferenceBase, *it->d);
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPose3DPDFParticles::drawSingleSample( CPose3D &outPart ) const
{
	MRPT_UNUSED_PARAM(outPart);
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void  CPose3DPDFParticles::drawManySamples(
	size_t						N,
	std::vector<CVectorDouble>	&outSamples ) const
{
	MRPT_UNUSED_PARAM(N); MRPT_UNUSED_PARAM(outSamples);
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
						+=
 ---------------------------------------------------------------*/
void  CPose3DPDFParticles::operator += ( const CPose3D &Ap)
{
	MRPT_UNUSED_PARAM(Ap);
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
					append
 ---------------------------------------------------------------*/
void  CPose3DPDFParticles::append( CPose3DPDFParticles &o )
{
	MRPT_UNUSED_PARAM(o);
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void CPose3DPDFParticles::inverse(CPose3DPDF  &o) const
{
	MRPT_START
	ASSERT_( o.GetRuntimeClass() == CLASS_ID(CPose3DPDFParticles) );
	CPose3DPDFParticles * out = static_cast<CPose3DPDFParticles*> (&o);

	// Prepare the output:
	out->copyFrom(*this);

	CPose3DPDFParticles::CParticleList::iterator it;
	CPose3D   zero(0,0,0);

	for (it=out->m_particles.begin();it!=out->m_particles.end();++it)
		*it->d = zero - *it->d;

	MRPT_END
}

/*---------------------------------------------------------------
					getMostLikelyParticle
 ---------------------------------------------------------------*/
CPose3D	 CPose3DPDFParticles::getMostLikelyParticle() const
{
	CPose3DPDFParticles::CParticleList::const_iterator	it, itMax=m_particles.begin();
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
void  CPose3DPDFParticles::bayesianFusion( const CPose3DPDF &p1,const  CPose3DPDF &p2 )
{
	MRPT_UNUSED_PARAM(p1);MRPT_UNUSED_PARAM(p2);
	THROW_EXCEPTION("Not implemented yet!");
}


/*---------------------------------------------------------------
							resetDeterministic
	Reset PDF to a single point and set the number of m_particles
 ---------------------------------------------------------------*/
void  CPose3DPDFParticles::resetDeterministic( const CPose3D &location,
												size_t	particlesCount)
{
	CPose3DPDFParticles::CParticleList::iterator		it;

	if (particlesCount>0)
	{
		clearParticles();
		m_particles.resize(particlesCount);
		for (it = m_particles.begin(); it != m_particles.end(); ++it)
			it->d.reset(new CPose3D());
	}

	for (it=m_particles.begin();it!=m_particles.end();++it)
	{
		*it->d	= location;
		it->log_w	= 0;
	}
}
