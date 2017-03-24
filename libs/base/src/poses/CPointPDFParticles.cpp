/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE( CPointPDFParticles, CPointPDF, mrpt::poses )
IMPLEMENTS_SERIALIZABLE( TSimple3DPoint, CSerializable, mrpt::poses )

CPointPDFParticles::CPointPDFParticles(size_t numParticles)
{
	setSize(numParticles);
}

/** Clear all the particles (free memory) */
void CPointPDFParticles::clear() { 
	setSize(0); 
}

/*---------------------------------------------------------------
		setSize
  ---------------------------------------------------------------*/
void CPointPDFParticles::setSize(
	size_t	numberParticles,
	const	CPoint3D &defaultValue)
{
	// Free old particles: automatic via smart ptr
	m_particles.resize(numberParticles);
	for (auto &it : m_particles)
	{
		it.log_w = 0;
		it.d.reset(new TSimple3DPoint(defaultValue));
	}
}

/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF)
 ---------------------------------------------------------------*/
void CPointPDFParticles::getMean(CPoint3D &p) const
{
	MRPT_START
	if (m_particles.empty())
		THROW_EXCEPTION("Cannot compute mean since there are zero particles.")

	CParticleList::const_iterator	it;
	double		sumW=0;
	double		x=0,y=0,z=0;
	for (it=m_particles.begin();it!=m_particles.end();it++)
	{
		const double w = exp(it->log_w);
		x+=it->d->x*w;
		y+=it->d->y*w;
		z+=it->d->z*w;
		sumW+=w;
	}

	ASSERT_(sumW!=0)

	sumW = 1.0/sumW;

	p.x( x*sumW );
	p.y( y*sumW );
	p.z( z*sumW );

	MRPT_END
}

/*---------------------------------------------------------------
						getEstimatedCovariance
 ---------------------------------------------------------------*/
void CPointPDFParticles::getCovarianceAndMean(CMatrixDouble33 &cov, CPoint3D &mean) const
{
	MRPT_START

	getMean(mean);
	cov.zeros();

	size_t		i,n = m_particles.size();
	double		var_x=0,var_y=0,var_p=0,var_xy=0,var_xp=0,var_yp=0;

	double lin_w_sum = 0;

	for (i=0;i<n;i++) lin_w_sum+= exp( m_particles[i].log_w );
	if (lin_w_sum==0) lin_w_sum=1;

	for (i=0;i<n;i++)
	{
		double w = exp( m_particles[i].log_w ) / lin_w_sum;

		double	err_x   = m_particles[i].d->x - mean.x();
		double	err_y   = m_particles[i].d->y - mean.y();
		double	err_phi = m_particles[i].d->z - mean.z();

		var_x+= square(err_x)*w;
		var_y+= square(err_y)*w;
		var_p+= square(err_phi)*w;
		var_xy+= err_x*err_y*w;
		var_xp+= err_x*err_phi*w;
		var_yp+= err_y*err_phi*w;
	}

	if (n>=2)
	{
		// Unbiased estimation of variance:
		cov(0,0) = var_x;
		cov(1,1) = var_y;
		cov(2,2) = var_p;

		cov(1,0) = cov(0,1) = var_xy;
		cov(2,0) = cov(0,2) = var_xp;
		cov(1,2) = cov(2,1) = var_yp;
	}

	MRPT_END
}

/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPointPDFParticles::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t	N = size();
		out << N;

		for (CParticleList::const_iterator	it=m_particles.begin();it!=m_particles.end();++it)
			out << it->log_w << it->d->x << it->d->y << it->d->z;
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPointPDFParticles::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t	N;
			in >> N;
			setSize(N);

			for (CParticleList::iterator	it=m_particles.begin();it!=m_particles.end();++it)
				in >> it->log_w >> it->d->x >> it->d->y >> it->d->z;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

void  CPointPDFParticles::copyFrom(const CPointPDF &o)
{
	if (this == &o) return;		// It may be used sometimes

	// Convert to samples:
	THROW_EXCEPTION("NO");
}

/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
void  CPointPDFParticles::saveToTextFile(const std::string &file) const
{
	MRPT_START

	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;

	size_t i,N = m_particles.size();
	for (i=0;i<N;i++)
		os::fprintf(f,"%f %f %f %e\n", m_particles[i].d->x,m_particles[i].d->y,m_particles[i].d->z,m_particles[i].log_w);

	os::fclose(f);

	MRPT_END
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPointPDFParticles::changeCoordinatesReference(const CPose3D &newReferenceBase )
{
	TPoint3D pt;
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		newReferenceBase.composePoint(
			it->d->x, it->d->y, it->d->z,  // In
			pt.x, pt.y, pt.z // Out
			);
		it->d->x = pt.x;
		it->d->y = pt.y;
		it->d->z = pt.z;
	}
}

/*---------------------------------------------------------------
						computeKurtosis
 ---------------------------------------------------------------*/
double CPointPDFParticles::computeKurtosis()
{
	MRPT_START

	// kurtosis = \mu^4 / (\sigma^2) -3
	CVectorDouble kurts, mu4,m,var;
	kurts.assign(3,.0);
	mu4.assign(3,.0);
	m.assign(3,.0);
	var.assign(3,.0);

	// Means:
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		m[0]+=it->d->x;
		m[1]+=it->d->y;
		m[2]+=it->d->z;
	}
	m*=1.0/m_particles.size();

	// variances:
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		var[0]+=square(it->d->x-m[0]);
		var[1]+=square(it->d->y-m[1]);
		var[2]+=square(it->d->z-m[2]);
	}
	var*=1.0/m_particles.size();
	var[0]=square(var[0]);
	var[1]=square(var[1]);
	var[2]=square(var[2]);

	// Moment:
	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		mu4[0]+=pow(it->d->x-m[0],4.0);
		mu4[1]+=pow(it->d->y-m[1],4.0);
		mu4[2]+=pow(it->d->z-m[2],4.0);
	}
	mu4*=1.0/m_particles.size();

	// Kurtosis's
	kurts.array() = mu4.array() / var.array();

	return math::maximum(kurts);

	MRPT_END
}


/*---------------------------------------------------------------
					drawSingleSample
  ---------------------------------------------------------------*/
void CPointPDFParticles::drawSingleSample(CPoint3D &outSample) const
{
	MRPT_UNUSED_PARAM(outSample);
	THROW_EXCEPTION("TO DO!")
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPointPDFParticles::bayesianFusion( const  CPointPDF &p1_, const  CPointPDF &p2_,const double &minMahalanobisDistToDrop )
{
	MRPT_UNUSED_PARAM(p1_); MRPT_UNUSED_PARAM(p2_); MRPT_UNUSED_PARAM(minMahalanobisDistToDrop);
	MRPT_START

	THROW_EXCEPTION("TODO!!!");

	MRPT_END
}


/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  TSimple3DPoint::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	MRPT_UNUSED_PARAM(out); MRPT_UNUSED_PARAM(version);
	THROW_EXCEPTION("Shouldn't arrive here!");
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  TSimple3DPoint::readFromStream(mrpt::utils::CStream &in,int version)
{
	MRPT_UNUSED_PARAM(in); MRPT_UNUSED_PARAM(version);
	THROW_EXCEPTION("Shouldn't arrive here!");
}
