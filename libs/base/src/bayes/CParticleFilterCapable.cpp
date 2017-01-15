/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/random.h>
#include <mrpt/math/ops_vectors.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::bayes;
using namespace mrpt::random;
using namespace mrpt::math;
using namespace std;


const unsigned CParticleFilterCapable::PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS = 20;

/*---------------------------------------------------------------
					performResampling
 ---------------------------------------------------------------*/
void  CParticleFilterCapable::performResampling(
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
	size_t out_particle_count )
{
	MRPT_START

	// Make a vector with the particles' log. weights:
	const size_t in_particle_count = particlesCount();
	ASSERT_(in_particle_count>0)

	vector<size_t>	indxs;
	vector<double>	log_ws;
	log_ws.assign(in_particle_count, .0);
	for (size_t i=0;i<in_particle_count;i++)
		log_ws[i] = getW(i);

	// Compute the surviving indexes:
	computeResampling(
		PF_options.resamplingMethod,
		log_ws,
		indxs,
		out_particle_count );

	// And perform the particle replacement:
	performSubstitution( indxs );

	// Finally, equal weights:
	for (size_t i=0;i<out_particle_count;i++) setW(i, 0 /* Logarithmic weight */ );

	MRPT_END
}

/*---------------------------------------------------------------
						resample
 ---------------------------------------------------------------*/
void CParticleFilterCapable::computeResampling(
	CParticleFilter::TParticleResamplingAlgorithm	method,
	const vector<double>	&in_logWeights,
	vector<size_t>			&out_indexes,
	size_t out_particle_count )
{
	MRPT_START

	// Compute the normalized linear weights:
	//  The array "linW" will be the input to the actual
	//  resampling algorithms.
	size_t							i,j,M=in_logWeights.size();
	ASSERT_(M>0)

	if (!out_particle_count)
		out_particle_count = M;

	vector<double>					linW( M,0 );
	vector<double>::const_iterator	inIt;
	vector<double>::iterator			outIt;
	double							linW_SUM=0;

	// This is to avoid float point range problems:
	double max_log_w = math::maximum( in_logWeights );
	for (i=0,inIt=in_logWeights.begin(),outIt=linW.begin();i<M;i++,inIt++,outIt++)
		linW_SUM += ( (*outIt) = exp( (*inIt) - max_log_w ) );

	// Normalize weights:
	ASSERT_(linW_SUM>0);
	linW *= 1.0 / linW_SUM;

	switch ( method )
	{
	case CParticleFilter::prMultinomial:
		{
			// ==============================================
			//   Select with replacement
			// ==============================================
			vector<double>	Q;
			mrpt::math::cumsum_tmpl<vector<double>,vector<double>,double>(linW, Q);
			Q[M-1] = 1.1;

			vector<double>	T(M);
			randomGenerator.drawUniformVector(T,0.0, 0.999999);
			T.push_back(1.0);

			// Sort:
			// --------------------
			sort( T.begin(), T.end() );

			out_indexes.resize(out_particle_count);
			i=j=0;

			while (i < out_particle_count)
			{
				if (T[i]<Q[j])
				{
					out_indexes[i++] = (unsigned int) j;
				}
				else
				{
					j++;
					if (j>=M) j=M-1;
				}
			}
		}
		break;	// end of "Select with replacement"

	case CParticleFilter::prResidual:
		{
			// ==============================================
			//   prResidual
			// ==============================================
			// Repetition counts:
			vector_uint	N(M);
			size_t 		R=0;	// Remainder or residual count
			for (i=0;i<M;i++)
			{
				N[i] = int( M*linW[i] );
				R+= N[i];
			}
			size_t  N_rnd =  out_particle_count>=R ? (out_particle_count-R) : 0; // # of particles to be drawn randomly (the "residual" part)

			// Fillout the deterministic part of the resampling:
			out_indexes.resize(out_particle_count);
			for (i=0, j=0 ;i<out_particle_count;i++)
				for (size_t k=0;k<N[i];k++)
					out_indexes[j++] = i;

			size_t M_fixed = j;

			// Prepare a multinomial resampling with the residual part,
			//  using the modified weights:
			// ----------------------------------------------------------
			if (N_rnd)	// If there are "residual" part (should be virtually always!)
			{
				// Compute modified weights:
				vector<double>	linW_mod(M);
				const double M_R_1 = 1.0/N_rnd;
				for (i=0;i<M;i++)
					linW_mod[i] = M_R_1 * (M*linW[i]-N[i]);

				// perform resampling:
				vector<double>	Q;
				mrpt::math::cumsum_tmpl<vector<double>,vector<double>,double>(linW_mod, Q);
				Q[M-1] = 1.1;

				vector<double>	T(M);
				randomGenerator.drawUniformVector(T, 0.0 , 0.999999);
				T.push_back(1.0);

				// Sort:
				sort( T.begin(), T.end() );

				i=0;
				j=0;

				while (i < N_rnd)
				{
					if (T[i]<Q[j])
					{
						out_indexes[M_fixed + i++] = (unsigned int) j;
					}
					else
					{
						j++;
						if (j>=M) j=M-1;
					}
				}
			} // end if N_rnd!=0
		}
		break;
	case CParticleFilter::prStratified:
		{
			// ==============================================
			//   prStratified
			// ==============================================
			vector<double>	Q;
			mrpt::math::cumsum_tmpl<vector<double>,vector<double>,double>(linW, Q);
			Q[M-1] = 1.1;

			// Stratified-uniform random vector:
			vector<double>	T(M+1);
			const double	_1_M = 1.0 / M;
			const double	_1_M_eps = _1_M - 0.000001;
			double   		T_offset = 0;
			for (i=0;i<M;i++)
			{
				T[i] = T_offset + randomGenerator.drawUniform(0.0,_1_M_eps);
				T_offset+= _1_M;
			}
			T[M] = 1;

			out_indexes.resize(out_particle_count);
			i=j=0;
			while (i < out_particle_count)
			{
				if (T[i]<Q[j])
					out_indexes[i++] = (unsigned int)j;
				else
				{
					j++;
					if (j>=M) j=M-1;
				}
			}
		}
		break;
	case CParticleFilter::prSystematic:
		{
			// ==============================================
			//   prSystematic
			// ==============================================
			vector<double>	Q;
			mrpt::math::cumsum_tmpl<vector<double>,vector<double>,double>(linW, Q);
			Q[M-1] = 1.1;

			// Uniform random vector:
			vector<double>	T(M+1);
			double	_1_M = 1.0 / M;
			T[0] = randomGenerator.drawUniform(0.0,_1_M);
			for (i=1;i<M;i++)	T[i] = T[i-1] + _1_M;
			T[M] = 1;

			out_indexes.resize(out_particle_count);
			i=j=0;
			while (i < out_particle_count)
			{
				if (T[i]<Q[j])
					out_indexes[i++] = (unsigned int)j;
				else
				{
					j++;
					if (j>=M) j=M-1;
				}
			}

		}
		break;
	default:
			THROW_EXCEPTION( format("ERROR: Unknown resampling method selected: %i",method) );
	};

	MRPT_END
}

/*---------------------------------------------------------------
					prediction_and_update
 ---------------------------------------------------------------*/
void  CParticleFilterCapable::prediction_and_update(
	const mrpt::obs::CActionCollection	* action,
	const mrpt::obs::CSensoryFrame		* observation,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options  )
{
	switch ( PF_options.PF_algorithm )
	{
	case CParticleFilter::pfStandardProposal:
		prediction_and_update_pfStandardProposal( action, observation, PF_options );
		break;
	case CParticleFilter::pfAuxiliaryPFStandard:
		prediction_and_update_pfAuxiliaryPFStandard( action, observation, PF_options );
		break;
	case CParticleFilter::pfOptimalProposal:
		prediction_and_update_pfOptimalProposal( action, observation, PF_options );
		break;
	case CParticleFilter::pfAuxiliaryPFOptimal:
		prediction_and_update_pfAuxiliaryPFOptimal( action, observation, PF_options );
		break;
	default:
		{
			THROW_EXCEPTION("Invalid particle filter algorithm selection!");
		} break;

	}

}

/*---------------------------------------------------------------
					prediction_and_update_...
 ---------------------------------------------------------------*/
void  CParticleFilterCapable::prediction_and_update_pfStandardProposal(
	const mrpt::obs::CActionCollection	* action,
	const mrpt::obs::CSensoryFrame		* observation,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options   )
{
	MRPT_UNUSED_PARAM(action); MRPT_UNUSED_PARAM(observation); MRPT_UNUSED_PARAM(PF_options);
	THROW_EXCEPTION("Algorithm 'pfStandardProposal' is not implemented in inherited class!");
}
/*---------------------------------------------------------------
					prediction_and_update_...
 ---------------------------------------------------------------*/
void  CParticleFilterCapable::prediction_and_update_pfAuxiliaryPFStandard(
	const mrpt::obs::CActionCollection	* action,
	const mrpt::obs::CSensoryFrame		* observation,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options   )
{
	MRPT_UNUSED_PARAM(action); MRPT_UNUSED_PARAM(observation); MRPT_UNUSED_PARAM(PF_options);
	THROW_EXCEPTION("Algorithm 'pfAuxiliaryPFStandard' is not implemented in inherited class!");
}
/*---------------------------------------------------------------
					prediction_and_update_...
 ---------------------------------------------------------------*/
void  CParticleFilterCapable::prediction_and_update_pfOptimalProposal(
	const mrpt::obs::CActionCollection	* action,
	const mrpt::obs::CSensoryFrame		* observation,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options   )
{
	MRPT_UNUSED_PARAM(action); MRPT_UNUSED_PARAM(observation); MRPT_UNUSED_PARAM(PF_options);
	THROW_EXCEPTION("Algorithm 'pfOptimalProposal' is not implemented in inherited class!");
}
/*---------------------------------------------------------------
					prediction_and_update_...
 ---------------------------------------------------------------*/
void  CParticleFilterCapable::prediction_and_update_pfAuxiliaryPFOptimal(
	const mrpt::obs::CActionCollection	* action,
	const mrpt::obs::CSensoryFrame		* observation,
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options   )
{
	MRPT_UNUSED_PARAM(action); MRPT_UNUSED_PARAM(observation); MRPT_UNUSED_PARAM(PF_options);
	THROW_EXCEPTION("Algorithm 'pfAuxiliaryPFOptimal' is not implemented in inherited class!");
}

/*---------------------------------------------------------------
					prepareFastDrawSample
 ---------------------------------------------------------------*/
void  CParticleFilterCapable::prepareFastDrawSample(
	const bayes::CParticleFilter::TParticleFilterOptions &PF_options,
	TParticleProbabilityEvaluator partEvaluator,
	const void	* action,
	const void	* observation ) const
{
	MRPT_START

	if (PF_options.adaptiveSampleSize)
	{
		// --------------------------------------------------------
		// CASE: Dynamic number of particles:
		//  -> Use m_fastDrawAuxiliary.CDF, PDF, CDF_indexes
		// --------------------------------------------------------
		if (PF_options.resamplingMethod!=CParticleFilter::prMultinomial)
			THROW_EXCEPTION("resamplingMethod must be 'prMultinomial' for a dynamic number of particles!");

		size_t	i,j=666666,M = particlesCount();

		MRPT_START

		// Prepare buffers:
		m_fastDrawAuxiliary.CDF.resize( 1+PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS, 0);
		m_fastDrawAuxiliary.CDF_indexes.resize( PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS, 0);

		// Compute the vector of each particle's probability (usually
		//  it will be simply the weight, but there are other algorithms)
		m_fastDrawAuxiliary.PDF.resize( M, 0);

		// This is done to avoid floating point overflow!! (JLBC - SEP 2007)
		// -------------------------------------------------------------------
		double	SUM = 0;
		// Save the log likelihoods:
		for (i=0;i<M;i++)	m_fastDrawAuxiliary.PDF[i] = partEvaluator(PF_options, this,i,action,observation);
		// "Normalize":
		m_fastDrawAuxiliary.PDF += -math::maximum( m_fastDrawAuxiliary.PDF );
		for (i=0;i<M;i++)	SUM += m_fastDrawAuxiliary.PDF[i] = exp( m_fastDrawAuxiliary.PDF[i] );
		ASSERT_(SUM>=0);
		MRPT_CHECK_NORMAL_NUMBER(SUM);
		m_fastDrawAuxiliary.PDF *= 1.0/SUM;

		// Compute the CDF thresholds:
		for (i=0;i<PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS;i++)
			m_fastDrawAuxiliary.CDF[i] = ((double)i) / ((double)PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS);
		m_fastDrawAuxiliary.CDF[PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS] = 1.0;

		// Compute the CDF and save threshold indexes:
		double	CDF = 0; // Cumulative density func.
		for (i=0,j=0;i<M && j<PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS;i++)
		{
			double CDF_next = CDF + m_fastDrawAuxiliary.PDF[i];
			if (i==(M-1)) CDF_next = 1.0;	// rounds fix...
			if (CDF_next>1.0) CDF_next = 1.0;

			while ( m_fastDrawAuxiliary.CDF[j] < CDF_next )
				m_fastDrawAuxiliary.CDF_indexes[j++] = (unsigned int)i;

			CDF = CDF_next;
		}

		ASSERT_( j == PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS );

		// Done!
#if !defined(_MSC_VER) || (_MSC_VER>1400)  // <=VC2005 doesn't work with this!
		MRPT_END_WITH_CLEAN_UP( \
			/* Debug: */ \
			cout << "j=" << j << "\nm_fastDrawAuxiliary.CDF_indexes:" << m_fastDrawAuxiliary.CDF_indexes << endl; \
			cout << "m_fastDrawAuxiliary.CDF:" << m_fastDrawAuxiliary.CDF << endl; \
			);
#else
		MRPT_END
#endif

	}
	else
	{
		// ------------------------------------------------------------------------
		// CASE: Static number of particles:
		//  -> Use m_fastDrawAuxiliary.alreadyDrawnIndexes & alreadyDrawnNextOne
		// ------------------------------------------------------------------------
		// Generate the vector with the "probabilities" of each particle being selected:
		size_t	i,M = particlesCount();
		vector<double>		PDF(M,0);
		for (i=0;i<M;i++)
			PDF[i] = partEvaluator(PF_options,this,i,action,observation); // Default evaluator: takes current weight.

		vector<size_t>		idxs;

		// Generate the particle samples:
		computeResampling( PF_options.resamplingMethod, PDF, idxs );

		vector<size_t>::iterator	it;
		vector_uint::iterator			it2;
		m_fastDrawAuxiliary.alreadyDrawnIndexes.resize( idxs.size() );
		for ( it=idxs.begin(),it2=m_fastDrawAuxiliary.alreadyDrawnIndexes.begin();it!=idxs.end(); ++it, ++it2)
			*it2 = (unsigned int)(*it);

		m_fastDrawAuxiliary.alreadyDrawnNextOne = 0;
	}

	MRPT_END
}

/*---------------------------------------------------------------
					fastDrawSample
 ---------------------------------------------------------------*/
size_t  CParticleFilterCapable::fastDrawSample( const bayes::CParticleFilter::TParticleFilterOptions &PF_options  ) const
{
	MRPT_START

	if (PF_options.adaptiveSampleSize)
	{
		// --------------------------------------------------------
		// CASE: Dynamic number of particles:
		//  -> Use m_fastDrawAuxiliary.CDF, PDF, CDF_indexes
		// --------------------------------------------------------
		if (PF_options.resamplingMethod!=CParticleFilter::prMultinomial)
			THROW_EXCEPTION("resamplingMethod must be 'prMultinomial' for a dynamic number of particles!");

		double draw = randomGenerator.drawUniform(0,0.999999);
		double CDF_next=-1.;
		double CDF = -1.;

		MRPT_START

		// Use the look-up table to see the starting index we must start looking from:
		size_t j = (size_t)floor( draw * ((double)PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS-0.05) );
		CDF = m_fastDrawAuxiliary.CDF[j];
		size_t i = m_fastDrawAuxiliary.CDF_indexes[j];

		// Find the drawn particle!
		while ( draw > (CDF_next = CDF+m_fastDrawAuxiliary.PDF[i]) )
		{
			CDF = CDF_next;
			i++;
		}

		return i;

		MRPT_END_WITH_CLEAN_UP( \
			printf("\n[CParticleFilterCapable::fastDrawSample] DEBUG: draw=%f, CDF=%f CDF_next=%f\n",draw,CDF,CDF_next); \
			);
	}
	else
	{
		// --------------------------------------------------------
		// CASE: Static number of particles:
		//  -> Use m_fastDrawAuxiliary.alreadyDrawnIndexes & alreadyDrawnNextOne
		// --------------------------------------------------------
		if ( m_fastDrawAuxiliary.alreadyDrawnNextOne>=m_fastDrawAuxiliary.alreadyDrawnIndexes.size() )
			THROW_EXCEPTION("Have you called 'fastDrawSample' more times than the sample size? Did you forget calling 'prepareFastCall' before?");

		return m_fastDrawAuxiliary.alreadyDrawnIndexes[m_fastDrawAuxiliary.alreadyDrawnNextOne++];
	}

	MRPT_END
}

/*---------------------------------------------------------------
						log2linearWeights
 ---------------------------------------------------------------*/
void CParticleFilterCapable::log2linearWeights(
	const vector<double>	&in_logWeights,
	vector<double>		&out_linWeights )
{
	MRPT_START

	size_t N = in_logWeights.size();

	out_linWeights.resize( N );

	if (!N) return;

	double sumW = 0;
	size_t i;
	for (i=0;i<N;i++)
		sumW += out_linWeights[i] = exp( in_logWeights[i] );

	ASSERT_(sumW>0)

	for (i=0;i<N;i++)
		out_linWeights[i] /= sumW;

	MRPT_END
}
