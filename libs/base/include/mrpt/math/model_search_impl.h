/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef math_modelsearch_impl_h
#define math_modelsearch_impl_h

#ifndef math_modelsearch_h
#	include "model_search.h"
#endif

#include <limits>

namespace mrpt {
	namespace math {

//----------------------------------------------------------------------
//! Run the ransac algorithm searching for a single model
template<typename TModelFit>
bool ModelSearch::ransacSingleModel( const TModelFit& p_state,
									 size_t p_kernelSize,
									 const typename TModelFit::Real& p_fitnessThreshold,
									 typename TModelFit::Model& p_bestModel,
									 vector_size_t& p_inliers )
{
    size_t bestScore = std::string::npos; // npos will mean "none"
	size_t iter = 0;
	size_t softIterLimit = 1; // will be updated by the size of inliers
	size_t hardIterLimit = 100; // a fixed iteration step
	p_inliers.clear();
	size_t nSamples = p_state.getSampleCount();
	vector_size_t ind( p_kernelSize );

	while ( iter < softIterLimit && iter < hardIterLimit )
	{
		bool degenerate = true;
		typename TModelFit::Model currentModel;
		size_t i = 0;
		while ( degenerate )
		{
			pickRandomIndex( nSamples, p_kernelSize, ind );
			degenerate = !p_state.fitModel( ind, currentModel );
			i++;
			if( i > 100 )
				return false;
		}

		vector_size_t inliers;

		for( size_t i = 0; i < nSamples; i++ )
		{
			if( p_state.testSample( i, currentModel ) < p_fitnessThreshold )
				inliers.push_back( i );
		}
		ASSERT_( inliers.size() > 0 );

		// Find the number of inliers to this model.
		const size_t ninliers = inliers.size();
        bool  update_estim_num_iters = (iter==0); // Always update on the first iteration, regardless of the result (even for ninliers=0)

        if ( ninliers > bestScore || (bestScore==std::string::npos && ninliers!=0))
		{
			bestScore = ninliers;
			p_bestModel = currentModel;
			p_inliers = inliers;
			update_estim_num_iters = true;
		}

		if (update_estim_num_iters)
		{
			// Update the estimation of maxIter to pick dataset with no outliers at propability p
			double f =  ninliers / static_cast<double>( nSamples );
			double p = 1 -  pow( f, static_cast<double>( p_kernelSize ) );
			const double eps = std::numeric_limits<double>::epsilon();
			p = std::max( eps, p);	// Avoid division by -Inf
			p = std::min( 1-eps, p);	// Avoid division by 0.
			softIterLimit = log(1-p) / log(p);
		}

		iter++;
	}

	return true;
}

//----------------------------------------------------------------------
//! Run a generic programming version of ransac searching for a single model
template<typename TModelFit>
bool ModelSearch::geneticSingleModel( const TModelFit& p_state,
									  size_t p_kernelSize,
									  const typename TModelFit::Real& p_fitnessThreshold,
									  size_t p_populationSize,
									  size_t p_maxIteration,
									  typename TModelFit::Model& p_bestModel,
									  vector_size_t& p_inliers )
{
	// a single specie of the population
	typedef TSpecies<TModelFit> Species;
	std::vector<Species> storage;
	std::vector<Species*> population;
	std::vector<Species*> sortedPopulation;

	size_t sampleCount = p_state.getSampleCount();
	int elderCnt = (int)p_populationSize/3;
	int siblingCnt = (p_populationSize - elderCnt) / 2;
	int speciesAlive = 0;

	storage.resize( p_populationSize );
	population.reserve( p_populationSize );
	sortedPopulation.reserve( p_populationSize );
	for( typename std::vector<Species>::iterator it = storage.begin(); it != storage.end(); it++ )
	{
		pickRandomIndex( sampleCount, p_kernelSize, it->sample );
		population.push_back( &*it );
		sortedPopulation.push_back( &*it );
	}

	size_t iter = 0;
	while ( iter < p_maxIteration )
	{
		if( iter > 0 )
		{
			//generate new population: old, siblings,  new
			population.clear();
			int i = 0;

			//copy the best elders
			for(; i < elderCnt; i++ )
			{
				population.push_back( sortedPopulation[i] );
			}

			// mate elders to make siblings
			int se = (int)speciesAlive; //dead species cannot mate
			for( ; i < elderCnt + siblingCnt ; i++ )
			{
				Species* sibling = sortedPopulation[--se];
				population.push_back( sibling );

				//pick two parents, from the species not yet refactored
				//better elders has more chance to mate as they are removed later from the list
				int r1 = rand();
				int r2 = rand();
				int p1 = r1 % se;
				int p2 = (p1 > se / 2) ? (r2 % p1) : p1 + 1 + (r2 % (se-p1-1));
				ASSERT_( p1 != p2 && p1 < se && p2 < se );
				ASSERT_( se >= elderCnt );
				Species* a = sortedPopulation[p1];
				Species* b = sortedPopulation[p2];

				// merge the sample candidates
				std::set<size_t> sampleSet;
				sampleSet.insert( a->sample.begin(), a->sample.end() );
				sampleSet.insert( b->sample.begin(), b->sample.end() );
				//mutate - add a random sample that will be selected with some (non-zero) probability
				sampleSet.insert( rand() % sampleCount );
				pickRandomIndex( sampleSet, p_kernelSize, sibling->sample );
			}

			// generate some new random species
			for( ; i < (int)p_populationSize; i++ )
			{
				Species* s = sortedPopulation[i];
				population.push_back( s );
				pickRandomIndex( sampleCount, p_kernelSize, s->sample );
			}
		}

		//evaluate species
		speciesAlive = 0;
		for( typename std::vector<Species*>::iterator it = population.begin(); it != population.end(); it++ )
		{
			Species& s = **it;
			if( p_state.fitModel( s.sample, s.model ) )
			{
				s.fitness = 0;
				for( size_t i = 0; i < p_state.getSampleCount(); i++ )
				{
					typename TModelFit::Real f = p_state.testSample( i, s.model );
					if( f < p_fitnessThreshold )
					{
						s.fitness += f;
						s.inliers.push_back( i );
					}
				}
				ASSERT_( s.inliers.size() > 0 );

				s.fitness /= s.inliers.size();
				// scale by the number of outliers
				s.fitness *= (sampleCount - s.inliers.size());
				speciesAlive++;
			}
			else
				s.fitness = std::numeric_limits<typename TModelFit::Real>::max();
		}

		if( !speciesAlive )
		{
			// the world is dead, no model was found
			return false;
		}

		//sort by fitness (ascending)
		std::sort( sortedPopulation.begin(), sortedPopulation.end(), Species::compare );

		iter++;
	}

	p_bestModel = sortedPopulation[0]->model;
	p_inliers = sortedPopulation[0]->inliers;

	return !p_inliers.empty();
}

} // namespace math
} // namespace mrpt

#endif // math_modelsearch_h
