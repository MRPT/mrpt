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

#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/math/utils.h>

using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE( CPose3DPDFParticles, CPose3DPDF, mrpt::poses )

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPose3DPDFParticles::CPose3DPDFParticles( size_t M )
{
	m_particles.resize(M);

	for (CPose3DPDFParticles::CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		it->log_w	= 0;
		it->d		= new CPose3D();
	}

	static CPose3D	nullPose(0,0,0);
	resetDeterministic( nullPose );
}

/*---------------------------------------------------------------
						operator =
  ---------------------------------------------------------------*/
void  CPose3DPDFParticles::copyFrom(const CPose3DPDF &o)
{
	MRPT_START

	CPose3DPDFParticles::CParticleList::const_iterator	itSrc;
	CPose3DPDFParticles::CParticleList::iterator		itDest;

	if (this == &o) return;		// It may be used sometimes

	if (o.GetRuntimeClass()==CLASS_ID(CPose3DPDFParticles))
	{
		const CPose3DPDFParticles	*pdf = static_cast<const CPose3DPDFParticles*>( &o );

		// Both are m_particles:
		if (m_particles.size()==pdf->m_particles.size())
		{
			for ( itSrc=pdf->m_particles.begin(), itDest = m_particles.begin();
				   itSrc!=pdf->m_particles.end();
			      itSrc++, itDest++ )
			{
				(*itDest->d) = (*itSrc->d);
				itDest->log_w = itSrc->log_w;
			}
		}
		else
		{
			for ( itDest = m_particles.begin();itDest!=m_particles.end();itDest++ )
				delete itDest->d;

			m_particles.resize( pdf->m_particles.size() );

			for ( itSrc=pdf->m_particles.begin(), itDest = m_particles.begin();
				   itSrc!=pdf->m_particles.end();
			      itSrc++, itDest++ )
			{
				itDest->d = new CPose3D( *itSrc->d );
				itDest->log_w = itSrc->log_w;
			}
		}
	}
	else
	if (o.GetRuntimeClass()==CLASS_ID(CPose3DPDFGaussian))
	{
		THROW_EXCEPTION("TO DO!!");
/*		CPose3DPDFGaussian	*pdf = (CPosePDFGaussian*) &o;
		int					M = (int)m_particles.size();
		std::vector<vector_float>			parts;
		std::vector<vector_float>::iterator partsIt;

		mrpt::random::randomNormalMultiDimensionalMany(pdf->cov,M,parts);

		for ( itDest = m_particles.begin();itDest!=m_particles.end();itDest++ )
				delete itDest->d;

		m_particles.clear();
		m_particles.resize(M);

		for ( itDest = m_particles.begin(),partsIt=parts.begin();itDest!=m_particles.end();itDest++,partsIt++ )
		{
			itDest->log_w = 0;
            itDest->d = new CPose3D( ( pdf->mean.x + (*partsIt)[0] ),
									 ( pdf->mean.y + (*partsIt)[1] ),
                                     ( pdf->mean.phi + (*partsIt)[2] ) );
			itDest->d->normalizePhi();
		}
*/
	}

	MRPT_END
}

/*---------------------------------------------------------------
	Destructor
  ---------------------------------------------------------------*/
CPose3DPDFParticles::~CPose3DPDFParticles( )
{
	clearParticles();
}

/*---------------------------------------------------------------
						getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF), computed
		as a weighted average over all m_particles.
 ---------------------------------------------------------------*/
void CPose3DPDFParticles::getMean(CPose3D &p) const
{
	MRPT_START

	double			X=0,Y=0,Z=0,YAW=0,PITCH=0,ROLL=0;
	double			ang,w,W=0;

	double			W_yaw_R=0,W_yaw_L=0;
	double			yaw_R=0,yaw_L=0;
	double			W_roll_R=0,W_roll_L=0;
	double			roll_R=0,roll_L=0;

	// First: XYZ
	// -----------------------------------
	for (CPose3DPDFParticles::CParticleList::const_iterator it=m_particles.begin();it!=m_particles.end();++it)
	{
		w  = exp(it->log_w);
		W += w;

		X		+= w * it->d->x();
		Y		+= w * it->d->y();
		Z		+= w * it->d->z();
		PITCH	+= w * it->d->pitch();

		// Angles Yaw and Roll are especial!:
		ang = it->d->yaw();
		if (fabs( ang )>0.5*M_PI)
		{
			// LEFT HALF: 0,2pi
			if (ang<0) ang = (M_2PI + ang);
			yaw_L += ang * w;
			W_yaw_L += w;
		}
		else
		{
			// RIGHT HALF: -pi,pi
			yaw_R += ang * w;
			W_yaw_R += w;
		}

		// Angles Yaw and Roll are especial!:
		ang = it->d->roll();
		if (fabs( ang )>0.5*M_PI)
		{
			// LEFT HALF: 0,2pi
			if (ang<0) ang = (M_2PI + ang);
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


	if (W==0) {
		p.setFromValues(0,0,0,0,0,0);
		return;
	}

	X /= W;
	Y /= W;
	Z /= W;
	PITCH /= W;

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
	vector_double	vars(6,0);	// The diagonal of the final covariance matrix
	CPose3DPDFParticles::CParticleList::const_iterator		it;

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
	for (it=m_particles.begin();it!=m_particles.end();it++)
		W += exp(it->log_w);

	ASSERT_(W>0);

	// Compute covariance:
	for (it=m_particles.begin();it!=m_particles.end();it++)
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
void  CPose3DPDFParticles::writeToStream(CStream &out,int *version) const
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
void  CPose3DPDFParticles::readFromStream(CStream &in, int version)
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
	MRPT_UNUSED_PARAM(file);
	THROW_EXCEPTION("TO DO!");
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
	CParticleList::iterator	it;
	for (it=m_particles.begin();it!=m_particles.end();it++)
		(*it->d) = newReferenceBase + (*it->d);
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
	std::vector<vector_double>	&outSamples ) const
{
	MRPT_UNUSED_PARAM(N); MRPT_UNUSED_PARAM(outSamples);
	THROW_EXCEPTION("TO DO!");
}

/*---------------------------------------------------------------
						+=
 ---------------------------------------------------------------*/
void  CPose3DPDFParticles::operator += ( const CPose3D &Ap)
{
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

	for (it=out->m_particles.begin();it!=out->m_particles.end();it++)
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

	for (it=m_particles.begin();it!=m_particles.end();it++)
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
		for (it=m_particles.begin();it!=m_particles.end();it++)
			it->d = new CPose3D();
	}

	for (it=m_particles.begin();it!=m_particles.end();it++)
	{
		*it->d	= location;
		it->log_w	= 0;
	}
}
