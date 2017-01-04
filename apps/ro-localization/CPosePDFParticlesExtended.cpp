/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/random.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/system/os.h>

#include "CPosePDFParticlesExtended.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::bayes;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;


double	likelihood_acumulation=0;

IMPLEMENTS_SERIALIZABLE( CPosePDFParticlesExtended, CPosePDF, mrpt::poses )

/*---------------------------------------------------------------
	Constructor
  ---------------------------------------------------------------*/
CPosePDFParticlesExtended::CPosePDFParticlesExtended( size_t M )
{
	m_particles.resize(M);

	for (CParticleList::iterator it=m_particles.begin();it!=m_particles.end();++it)
		it->d.reset( new TExtendedCPose2D());

	static TExtendedCPose2D	nullPose;
	resetDeterministic( nullPose );

	likelihood_acumulation=0;
}

/*---------------------------------------------------------------
						operator =
  ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::copyFrom(const CPosePDF &o)
{
	MRPT_START

	CParticleList::iterator	itSrc,itDest;

	if (this == &o) return;		// It may be used sometimes

	if (o.GetRuntimeClass()==CLASS_ID(CPosePDFParticlesExtended))
	{
		CPosePDFParticlesExtended	*pdf = (CPosePDFParticlesExtended*) &o;

		// Both are particles:
		m_particles = pdf->m_particles;
	}
	else
	if (o.GetRuntimeClass()==CLASS_ID(CPosePDFGaussian))
	{
		CPosePDFGaussian	*pdf = (CPosePDFGaussian*) &o;
		int					M = (int)m_particles.size();
		std::vector<vector<double> >			parts;
		std::vector<vector<double> >::iterator partsIt;

		randomGenerator.drawGaussianMultivariateMany(parts,M,pdf->cov);

		m_particles.clear();
		m_particles.resize(M);

		for ( itDest = m_particles.begin(),partsIt=parts.begin();itDest!=m_particles.end();itDest++,partsIt++ )
		{
			itDest->log_w = 0;
			itDest->d.reset( new TExtendedCPose2D());
			itDest->d->pose.x( pdf->mean.x() + (*partsIt)[0] );
			itDest->d->pose.x( pdf->mean.y() + (*partsIt)[1] );
			itDest->d->pose.phi(  pdf->mean.phi() + (*partsIt)[2] );

			itDest->d->pose.normalizePhi();
		}

	}

	MRPT_END
}

/*---------------------------------------------------------------
	Destructor
  ---------------------------------------------------------------*/
CPosePDFParticlesExtended::~CPosePDFParticlesExtended( )
{
	clear();
}

/*---------------------------------------------------------------
			clear
  ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::clear( )
{
	m_particles.clear();
}

/*---------------------------------------------------------------
						getEstimatedPose
  Returns an estimate of the pose, i.e. a "mean value", computed
		as a weighted average over all m_particles.
 ---------------------------------------------------------------*/
void CPosePDFParticlesExtended::getMean(CPose2D  &est) const
{
	est = CPose2D(0,0,0);

	CPose2D			p;
	size_t			i,n = m_particles.size();
	double			phi,w,W=0;
	double			W_phi_R=0,W_phi_L=0;
	double			phi_R=0,phi_L=0;


	if (!n) return;

	// First: XY
	// -----------------------------------
	for (i=0;i<n;i++)
	{
		p  = m_particles[i].d->pose;
		w  = exp(m_particles[i].log_w);
		W += w;

		est.x_incr( p.x() * w );
		est.y_incr( p.y() * w );

		// PHI is special:
		phi = p.phi();
		if (fabs(phi)>1.5707963267948966192313216916398f)
		{
			// LEFT HALF: 0,2pi
			if (phi<0) phi = (M_2PI + phi);

			phi_L += phi * w;
			W_phi_L += w;
		}
		else
		{
			// RIGHT HALF: -pi,pi
			phi_R += phi * w;
			W_phi_R += w;
		}
	}

	est*=(1/W);

	// Next: PHI
	// -----------------------------------
	// The mean value from each side:
	if (W_phi_L>0)	phi_L /= W_phi_L;  // [0,2pi]
	if (W_phi_R>0)	phi_R /= W_phi_R;  // [-pi,pi]

	// Left side to [-pi,pi] again:
	if (phi_L>M_PI) phi_L = phi_L - M_2PI;

	// The total mean:
	est.phi( ((phi_L * W_phi_L + phi_R * W_phi_R )/(W_phi_L+W_phi_R)) );
}

/*---------------------------------------------------------------
						getEstimatedPoseState
 ---------------------------------------------------------------*/
TExtendedCPose2D  CPosePDFParticlesExtended::getEstimatedPoseState() const
{
	TExtendedCPose2D	est;
	CPose2D			p;
	size_t			i,n = m_particles.size();
	double			phi;
	double			w,W=0;
	double			W_phi_R=0,W_phi_L=0;
	double			phi_R=0,phi_L=0;



	if (!n) return est;

	est.state.resize( m_particles[0].d->state.size() );

	for (i=0;i<n;i++)	W += exp(m_particles[i].log_w);
	if (W==0) W=1;

	// First: XY
	// -----------------------------------
	for (i=0;i<n;i++)
	{
		p  = m_particles[i].d->pose;
		w  = exp(m_particles[i].log_w) / W;

		est.pose.x_incr( (p.x() * w));
		est.pose.y_incr( (p.y() * w));

		CVectorDouble	auxVec (m_particles[i].d->state);
		auxVec *= w;
		est.state += auxVec;

		// PHI is special:
		phi = p.phi();
		if (fabs(phi)>1.5707963267948966192313216916398f)
		{
			// LEFT HALF: 0,2pi
			if (phi<0) phi = (M_2PI + phi);

			phi_L += phi * w;
			W_phi_L += w;
		}
		else
		{
			// RIGHT HALF: -pi,pi
			phi_R += phi * w;
			W_phi_R += w;
		}
	}

	est.pose *= (1.0/W);

	// Next: PHI
	// -----------------------------------
	// The mean value from each side:
	if (W_phi_L>0)	phi_L /= W_phi_L;  // [0,2pi]
	if (W_phi_R>0)	phi_R /= W_phi_R;  // [-pi,pi]

	// Left side to [-pi,pi] again:
	if (phi_L>M_PI) phi_L = phi_L - M_2PI;

	// The total mean:
	est.pose.phi(  ((phi_L * W_phi_L + phi_R * W_phi_R )/(W_phi_L+W_phi_R)) );

	return est;
}


/*---------------------------------------------------------------
						getEstimatedCovariance
  ---------------------------------------------------------------*/
void CPosePDFParticlesExtended::getCovarianceAndMean(CMatrixDouble33 &cov,CPose2D &mean) const
{
	getMean(mean);
	cov.zeros();

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
		double	err_x   = m_particles[i].d->pose.x() - mean.x();
		double	err_y   = m_particles[i].d->pose.y() - mean.y();
		double	err_phi = wrapToPi( fabs(m_particles[i].d->pose.phi() - mean_phi) );

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
}


/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		CParticleList::const_iterator	it;
		uint32_t						n;

		// The data
		n = uint32_t( m_particles.size() );
		out << n;
		for (it=m_particles.begin();it!=m_particles.end();it++)
			out << it->log_w << it->d->pose << it->d->state;
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			CParticleList::iterator	it;
			uint32_t	n;

			// Delete previous content:
			m_particles.clear();

			// The data
			in >> n;
			m_particles.resize(n);
			for (it=m_particles.begin();it!=m_particles.end();it++)
				in >> it->log_w >> it->d->pose >> it->d->state;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
			prediction_and_update_pfStandardProposal
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::offsetTransitionModel(double &val )
{
	if ( randomGenerator.drawUniform(0.0,1.0) < options.probabilityChangingBias )
	{
		val = randomGenerator.drawUniform(-options.changingBiasUnifRange,1.0);
//		val = min( val, 0.5f*options.changingBiasUnifRange );
//		val = max( val,-0.5f*options.changingBiasUnifRange );
	}
}


/*---------------------------------------------------------------

			prediction_and_update_pfStandardProposal

 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::prediction_and_update_pfStandardProposal(
	const mrpt::obs::CActionCollection	* actions,
	const mrpt::obs::CSensoryFrame		* sf,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	MRPT_START

	size_t				i,M = m_particles.size();
	CActionRobotMovement2DPtr robotMovement;

	// ----------------------------------------------------------------------
	//						PREDICTION STAGE
	// ----------------------------------------------------------------------
	ASSERT_(actions!=NULL);

	// Find a robot movement estimation:
	robotMovement = actions->getBestMovementEstimation();

	// Assure one has been found:
	if ( !robotMovement )
		THROW_EXCEPTION("Action list does not contain any CActionRobotMovement2D derived object!");

	//  Prediction:
	// -----------------------------
	if ( !PF_options.adaptiveSampleSize )
	{
		// ---------------------
		//   FIXES SAMPLE SIZE
		// ---------------------
		// Initialize random sample generator:
		mrpt::poses::CPoseRandomSampler	poseSamplesGen;
		poseSamplesGen.setPosePDF( robotMovement->poseChange.get_ptr() );

		CPose2D		increment_i;
		for (i=0;i<M;i++)			// Update particle poses:
		{
			poseSamplesGen.drawSample(increment_i);

			// Add pose increment.
			m_particles[i].d->pose = m_particles[i].d->pose + increment_i;

			// Prediction of the BIAS "state vector":
			for (int k=0;k<m_particles[i].d->state.size();k++)
				offsetTransitionModel( m_particles[i].d->state[k] );
		}
	} // end of fixed sample size
	else
	{
		THROW_EXCEPTION("Dynamic sample size not implemented in this class");
	} // end of ADAPTIVE SAMPLE SIZE

	// ----------------------------------------------------------------------
	//						UPDATE STAGE
	// ----------------------------------------------------------------------
	M = m_particles.size();
	if (sf!=NULL)
	{
		// A map MUST be supplied!
		ASSERT_(options.metricMap || options.metricMaps.size()>0);
		if (!options.metricMap)
			ASSERT_(options.metricMaps.size() == M);

		// Update particle's likelihood using the particle's pose:
		CParticleList::iterator	it;
		for (it=m_particles.begin(),i=0;it!=m_particles.end();it++,i++)
			it->log_w += auxiliarComputeObservationLikelihood(PF_options, this,i,sf, it->d.get() ) * PF_options.powFactor;
	};

	MRPT_END
}


/*---------------------------------------------------------------

			prediction_and_update_pfAuxiliaryPFOptimal

 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::prediction_and_update_pfAuxiliaryPFOptimal(
	const mrpt::obs::CActionCollection	* actions,
	const mrpt::obs::CSensoryFrame		* sf,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options )
{
	MRPT_START

	size_t						i,k,N,M = m_particles.size();
	CActionRobotMovement2DPtr robotMovement;

	// ----------------------------------------------------------------------
	//							PRELIMINAR CHECKS:
	// ----------------------------------------------------------------------
	ASSERT_(sf!=NULL);
	ASSERT_(actions!=NULL);

	robotMovement = actions->getBestMovementEstimation();	// Find a robot movement estimation:
	if ( !robotMovement )									// Assure one has been found:
		THROW_EXCEPTION("Action list does not contain any CActionRobotMovement2D derived object!");

	ASSERT_(options.metricMap || options.metricMaps.size()>0);
	if (!options.metricMap)
	{
		ASSERT_(options.metricMaps.size() == M);
	}


	// ----------------------------------------------------------------------
	//		0) Common part:  Prepare m_particles "draw" and compute
	// ----------------------------------------------------------------------
	// Initialize random sample generator:
	mrpt::poses::CPoseRandomSampler	m_movementDrawer;
	m_movementDrawer.setPosePDF( robotMovement->poseChange.get_ptr() );

	CPose2D  mean_movement;
	robotMovement->poseChange->getMean(mean_movement);

	m_pfAuxiliaryPFOptimal_estimatedProb.resize(M);

	// Prepare data for executing "fastDrawSample"
	CTicTac		tictac;
	printf("[prepareFastDrawSample] Computing...");
	tictac.Tic();
	prepareFastDrawSample(
		PF_options,
		particlesEvaluator_AuxPFOptimal,
		&mean_movement,
		sf );
	printf("Done! in %.06f ms\n",tictac.Tac()*1e3f);

#if 1  	/** DEBUG **/
	printf("[prepareFastDrawSample] max      (log) = %10.06f\n",  math::maximum(m_pfAuxiliaryPFOptimal_estimatedProb) );
	printf("[prepareFastDrawSample] max-mean (log) = %10.06f\n", -math::mean(m_pfAuxiliaryPFOptimal_estimatedProb) + math::maximum(m_pfAuxiliaryPFOptimal_estimatedProb) );
	printf("[prepareFastDrawSample] max-min  (log) = %10.06f\n", -math::minimum(m_pfAuxiliaryPFOptimal_estimatedProb) + math::maximum(m_pfAuxiliaryPFOptimal_estimatedProb) );
#endif	/*****/

	// Now we have the vector "m_fastDrawProbability" filled out with:
	//     w[i]·p(zt|z^{t-1},x^{[i],t-1},X)
	//  where X is the robot pose prior (as implemented in
	//  the aux. function "particlesEvaluator_AuxPFOptimal"),

	// Precompute a list of "random" samples from the movement model:
	deque<TExtendedCPose2D*>	newParticles;
	vector<double>				newParticlesWeight;

	// We need the (aproximate) maximum likelihood value for each
	//  previous particle [i]:
	//
	//     max{ p( z^t | data^[i], x_(t-1)^[i], u_(t) ) }
	//
	// We store "-1" if the max. value has not been computed yet:
	vector<double>					maxLikelihood(M, -1e300 );
	CPose2D							movementDraw;
	TExtendedCPose2D 				newPose,oldPose;
	double							acceptanceProb,newPoseLikelihood,ratioLikLik;
	unsigned int					timeoutCount, MAX_TIMEOUT = 1000;
	double							bestNewPoseAccProb;
	TExtendedCPose2D				bestNewPose;

	if ( !PF_options.adaptiveSampleSize )
	{
		// ----------------------------------------------------------------------
		//						1) FIXED SAMPLE SIZE VERSION
		// ----------------------------------------------------------------------
		newParticles.resize(M);
		newParticlesWeight.resize(M);


		for (i=0;i<M;i++)
		{
			// Generate a new particle:
			//   (a) Draw a "t-1" m_particles' index:
			// ----------------------------------------------------------------
			k = fastDrawSample( PF_options );
			oldPose = *m_particles[k].d;

            //   (b) Rejection-sampling: Draw a new robot pose from x[k],
			//       and accept it with probability p(zk|x) / maxLikelihood:
			// ----------------------------------------------------------------
			// Is the maxLikelihood already computed?
			double			maxLik_k = maxLikelihood[k];
			CPose2D			drawnSample;
			TExtendedCPose2D maxLikNewPose;

			if (maxLik_k<0)
			{
				// NO -> Compute now and save it:
				for (unsigned int q=0 ; q<PF_options.pfAuxFilterOptimal_MaximumSearchSamples; q++)
				{
					// Draw new robot pose:
					newPose.pose = oldPose.pose + m_movementDrawer.drawSample(drawnSample);
					// Prediction of the BIAS "state vector":
					newPose.state = oldPose.state;
					for (size_t q=0;q<size_t(newPose.state.size());q++)
						offsetTransitionModel( newPose.state[q] );

					// Likelihood:
					double lik = auxiliarComputeObservationLikelihood(PF_options, this,k,sf,&newPose);
					if (lik>maxLik_k)
					{
						maxLik_k = lik;
						maxLikNewPose = newPose;
					}
				}
				maxLikelihood[k] = maxLik_k;
			}

			// Rejection-sampling:
			timeoutCount = 0;
			bestNewPoseAccProb = 0;
			//static unsigned int	q = PF_options.pfAuxFilterOptimal_MaximumSearchSamples;
			do
			{
				// Draw new robot pose:
				newPose.pose = oldPose.pose + m_movementDrawer.drawSample(drawnSample);
				// Prediction of the BIAS "state vector":
				newPose.state = oldPose.state;
				for (size_t q=0;q<size_t(newPose.state.size());q++)
					offsetTransitionModel( newPose.state[q] );

				// Compute acceptance probability:
				newPoseLikelihood = auxiliarComputeObservationLikelihood(PF_options,this,k,sf,&newPose);
				ratioLikLik = exp(newPoseLikelihood - maxLikelihood[k]);
				acceptanceProb = min( 1.0, ratioLikLik );

				// Save the best "newPose" for the case of having a "timeout":
				if (acceptanceProb>bestNewPoseAccProb)
				{
					bestNewPoseAccProb	= acceptanceProb;
					bestNewPose			= newPose;
				}

                if ( ratioLikLik > 1)
				{
					if (ratioLikLik>1.2)
					{
						//  DEBUG
						//printf("\n[pfAuxiliaryPFOptimal] Warning!! p(z|x)/p(z|x*)=%f\n",ratioLikLik);
					}
					maxLikelihood[k] = newPoseLikelihood; //  :'-( !!!
				}

			} while ( acceptanceProb < randomGenerator.drawUniform(0.0,0.999) && (++timeoutCount)<MAX_TIMEOUT );

			if (timeoutCount>=MAX_TIMEOUT)
				newPose = bestNewPose;

			// Insert the new particle!:
			newParticles[i] = new TExtendedCPose2D( newPose );
			// And its weight:
			newParticlesWeight[i] = 1; //newPoseLikelihood / m_pfAuxiliaryPFOptimal_estimatedProb[k];
		} // for i

	} // end fixed sample size
	else
	{
		THROW_EXCEPTION("Not implemented for this class.")
	} // end adaptive sample size

	// Substitute old by new particle set:
	// -------------------------------------------------
	N = newParticles.size();
	clear();	// Free old m_particles memory:
	m_particles.resize(N);

	CParticleList::iterator	itDest;
	deque<TExtendedCPose2D*>::const_iterator itSrc;
	vector<double>::iterator itW;

	for (itDest=m_particles.begin(), itSrc = newParticles.begin(), itW = newParticlesWeight.begin();
			itDest!=m_particles.end();
		    itDest++,itSrc++,itW++)
	{
		itDest->d.reset( *itSrc );
		itDest->log_w = *itW;
	}
	newParticles.clear();
	normalizeWeights();

	MRPT_END
}


/*---------------------------------------------------------------
							resetDeterministic
	Reset PDF to a single point and set the number of particles
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::resetDeterministic(
	TExtendedCPose2D	&location,
	int					particlesCount)
{
	CParticleList::iterator		it;

	if (particlesCount>0)
	{
		clear();
		m_particles.resize(particlesCount);
		for (it=m_particles.begin();it!=m_particles.end();it++)
			it->d.reset( new TExtendedCPose2D() );
	}

	for (it=m_particles.begin();it!=m_particles.end();it++)
	{
		*it->d	= location;
		it->log_w	= 0;
	}
}

/*---------------------------------------------------------------
						resetUniform
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::resetUniform(
			  float x_min,  float x_max,
			  float y_min,  float y_max,
			  CVectorFloat	state_min,
			  CVectorFloat	state_max,
			  float phi_min,float phi_max,
			  int	particlesCount)
{
	MRPT_START

	ASSERT_( state_min.size() == state_max.size() );

	if (particlesCount>0)
	{
		clear();
		m_particles.resize(particlesCount);
		for (int i = 0; i < particlesCount; i++)
			m_particles[i].d.reset(new TExtendedCPose2D());
	}

	size_t		i,M = m_particles.size();
	for (i=0;i<M;i++)
	{
		m_particles[i].d->pose.x(  randomGenerator.drawUniform( x_min, x_max ));
		m_particles[i].d->pose.y(  randomGenerator.drawUniform( y_min, y_max ));
		m_particles[i].d->pose.phi(randomGenerator.drawUniform( phi_min, phi_max ));
		m_particles[i].d->state.resize(state_min.size());
		m_particles[i].d->state.resize(state_max.size());

		for (int k=0;k<state_min.size();k++)
			m_particles[i].d->state[k] = randomGenerator.drawUniform( state_min[k], state_max[k] );

		m_particles[i].log_w=0;
	}

	MRPT_END
}

/*---------------------------------------------------------------
						saveToTextFile
   Save PDF's particles to a text file. In each line it
      will go: "x y phi weight"
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::saveToTextFile(const std::string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return;

	for (unsigned int i=0;i<m_particles.size();i++)
		os::fprintf(f,"%f %f %f %e\n",
				m_particles[i].d->pose.x(),
				m_particles[i].d->pose.y(),
				m_particles[i].d->pose.phi(),
				m_particles[i].log_w );

	os::fclose(f);
}


/*---------------------------------------------------------------
						getParticlePose
 ---------------------------------------------------------------*/
CPose2D	 CPosePDFParticlesExtended::getParticlePose(int i) const
{
	return m_particles[i].d->pose;
}

/*---------------------------------------------------------------
						changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::changeCoordinatesReference(const CPose3D &newReferenceBase_ )
{
	CPose2D newReferenceBase = CPose2D(newReferenceBase_);
	CParticleList::iterator	it;

	for (it=m_particles.begin();it!=m_particles.end();it++)
		it->d->pose = newReferenceBase + it->d->pose;
}

/*---------------------------------------------------------------
					drawSingleSample
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::drawSingleSample( CPose2D &outPart ) const
{
	float									uni = randomGenerator.drawUniform(0.0f,0.9999f);
	double									cum = 0;
	CParticleList::const_iterator	it;

	for (it=m_particles.begin();it!=m_particles.end();it++)
	{
		cum+= exp(it->log_w);
		if ( uni<= cum )
		{
			outPart= it->d->pose;
			return;
		}
	}

	// Might not come here normally:
	outPart = (m_particles.end()-1)->d->pose;
}

/*---------------------------------------------------------------
					drawManySamples
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::drawManySamples( size_t N, std::vector<CVectorDouble> & outSamples ) const
{
	CParticleFilter::TParticleFilterOptions  PF_options;
	PF_options.adaptiveSampleSize = true;
	PF_options.resamplingMethod = CParticleFilter::prMultinomial;

	prepareFastDrawSample( PF_options );

	outSamples.resize(N);
	for (size_t i=0;i<N;i++)
	{
		const TExtendedCPose2D *ptr = m_particles[ fastDrawSample( PF_options ) ].d.get();

		// Copy pose:
		outSamples[i].resize(3);
		outSamples[i][0] = ptr->pose.x();
		outSamples[i][1] = ptr->pose.y();
		outSamples[i][2] = ptr->pose.phi();
	}
}


/*---------------------------------------------------------------
						+=
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::operator += ( const CPose2D &Ap)
{
	CParticleList::iterator	it;

	for (it=m_particles.begin();it!=m_particles.end();it++)
		it->d->pose = it->d->pose + Ap;
}

/*---------------------------------------------------------------
					inverse
 ---------------------------------------------------------------*/
void	 CPosePDFParticlesExtended::inverse(CPosePDF  &o) const
{
	ASSERT_( o.GetRuntimeClass() == CLASS_ID(CPosePDFParticlesExtended) );
	CPosePDFParticlesExtended	*out = (CPosePDFParticlesExtended*) &o;


	out->copyFrom( *this );
	static CPose2D		nullPose(0,0,0);

	for (unsigned int i=0;i<out->m_particles.size();i++)
		out->m_particles[i].d->pose = nullPose - out->m_particles[i].d->pose;

}

/*---------------------------------------------------------------
					getMostLikelyParticle
 ---------------------------------------------------------------*/
CPose2D	 CPosePDFParticlesExtended::getMostLikelyParticle() const
{
	CParticleList::const_iterator	it, itMax=m_particles.begin();
	double		max_w = -1e300;


	for (it=m_particles.begin();it!=m_particles.end();it++)
	{
		if (it->log_w > max_w)
		{
			itMax = it;
			max_w = it->log_w;
		}
	}

	return itMax->d->pose;
}

/*---------------------------------------------------------------
					bayesianFusion
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::bayesianFusion( const CPosePDF &p1, const CPosePDF &p2, const double&minMahalanobisDistToDrop  )
{
	MRPT_UNUSED_PARAM(p1);MRPT_UNUSED_PARAM(p2);

	THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
					evaluatePDF_parzen
 ---------------------------------------------------------------*/
double  CPosePDFParticlesExtended::evaluatePDF_parzen(
	float	x,
	float	y,
	float	phi,
	float	stdXY,
	float	stdPhi ) const
{
	double	difPhi, ret = 0;

	for (CParticleList::const_iterator	it=m_particles.begin();it!=m_particles.end();++it)
	{
		difPhi = wrapToPi( phi-it->d->pose.phi() );

		ret += exp(it->log_w) *
			mrpt::math::normalPDF( sqrt( square(x-it->d->pose.x()) + square(y-it->d->pose.y())) , 0, stdXY ) *
			mrpt::math::normalPDF( fabs( difPhi ), 0, stdPhi );
	}

	return ret;
}

/*---------------------------------------------------------------
					saveParzenPDFToTextFile
 ---------------------------------------------------------------*/
void  CPosePDFParticlesExtended::saveParzenPDFToTextFile(
	const char	*fileName,
	float		x_min,
	float		x_max,
	float		y_min,
	float		y_max,
	float		phi,
	float		stepSizeXY,
	float		stdXY,
	float		stdPhi ) const
{
	FILE	*f=os::fopen(fileName,"wt");
	if (!f) return;

	for (float y=y_min; y<y_max; y+=stepSizeXY)
	{
		for (float x=x_min; x<x_max; x+=stepSizeXY)
		{
			fprintf(f,"%f ",
				evaluatePDF_parzen(x,y,phi,stdXY,stdPhi) );
		} // y
		fprintf(f,"\n");
	} // x

	os::fclose(f);
}


/*---------------------------------------------------------------
					TPredictionParams
 ---------------------------------------------------------------*/
CPosePDFParticlesExtended::TPredictionParams::TPredictionParams()
{
	metricMap				= NULL;

	KLD_minSampleSize		= 250;
	KLD_maxSampleSize		= 100000;
	KLD_binSize_XY			= 0.2f;
	KLD_binSize_PHI			= DEG2RAD(5);
	KLD_delta				= 0.01f;
	KLD_epsilon				= 0.02f;

	probabilityChangingBias = 0.1f;
	changingBiasUnifRange   = 2.0f;
	mixtureProposalRatio	= 0.10f;

	pfAuxFilterOptimal_MaximumSearchSamples	= 20;
}

/*---------------------------------------------------------------
				auxiliarComputeObservationLikelihood
 ---------------------------------------------------------------*/
double  CPosePDFParticlesExtended::auxiliarComputeObservationLikelihood(
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
	const CParticleFilterCapable	*obj,
	size_t			particleIndexForMap,
	const CSensoryFrame	*observation,
	const TExtendedCPose2D		*x )
{
	double			ret = 1;
	CMetricMap		*map;	// The map:

	const CPosePDFParticlesExtended *pdf = static_cast<const CPosePDFParticlesExtended*> (obj);

	if (pdf->options.metricMap)
			map = pdf->options.metricMap;
	else
	{
		ASSERT_( pdf->options.metricMaps.size()>particleIndexForMap );
		map = pdf->options.metricMaps[particleIndexForMap]; //->m_gridMaps[0];
	}

	// For each observation:
	for (CSensoryFrame::const_iterator it=observation->begin();it!=observation->end();++it)
	{
		const CObservation	*obser = it->pointer();
		CObservationBeaconRanges	obserDumm;

		// JLBC: 20/ABR/2007 -> UWB offset from extended state vector
		if (obser->GetRuntimeClass() == CLASS_ID(CObservationBeaconRanges) )
		{
			CObservationBeaconRanges	*obs = (CObservationBeaconRanges*) obser;
			obserDumm = *obs;

			// Introduce bias:
			ASSERT_( (int)obserDumm.sensedData.size() == (int)x->state.size() );
			for (size_t k=0;k<size_t(obserDumm.sensedData.size());k++)
				obserDumm.sensedData[k].sensedDistance -= x->state[k];

			// Substitute:
			obser = &obserDumm;
		}

		ret += map->computeObservationLikelihood( obser, x->pose );	// Compute the likelihood:
	}

	//cout << x->pose << format(": lik = %f",ret) << endl;

	// Done!
	return ret;
}


/*---------------------------------------------------------------
			particlesEvaluator_AuxPFOptimal
 ---------------------------------------------------------------*/
double  CPosePDFParticlesExtended::particlesEvaluator_AuxPFOptimal(
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
	const CParticleFilterCapable	*obj,
	size_t index,
	const void * action,
	const void * observation )
{
	MRPT_START

	// Compute the quantity:
	//     w[i]·p(zt|z^{t-1},x^{[i],t-1},X)
	// See paper: [blanco2007...]

	// Take the previous particle weight:
	// --------------------------------------------
	const CPosePDFParticlesExtended *pdf = static_cast<const CPosePDFParticlesExtended*>(obj);

	double		ret = pdf->m_particles[index].log_w;

	// , take the mean of the posterior density:
	// --------------------------------------------
	TExtendedCPose2D x_predict = *pdf->m_particles[index].d;
	x_predict.pose = x_predict.pose + *static_cast<const CPose2D*>(action);

	// and compute the obs. likelihood:
	// --------------------------------------------
	return ret + ( pdf->m_pfAuxiliaryPFOptimal_estimatedProb[index] = auxiliarComputeObservationLikelihood(
		PF_options,
		pdf,
		index,
		static_cast<const CSensoryFrame*>(observation),
		&x_predict ) );

	MRPT_END
}
