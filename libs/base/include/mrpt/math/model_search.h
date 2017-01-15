/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef math_modelsearch_h
#define math_modelsearch_h

#include <mrpt/utils/utils_defs.h>
#include <set>

namespace mrpt {
	namespace math {


	/** Model search implementations: RANSAC and genetic algorithm
	  *
	  *  The type  \a TModelFit is a user-supplied struct/class that implements this interface:
	  *  - Types:
	  *    - \a Real : The numeric type to use (typ: double, float)
	  *    - \a Model : The type of the model to be fitted (for example: A matrix, a TLine2D, a TPlane3D, ...)
	  *  - Methods:
	  *    - size_t getSampleCount() const : return the number of samples. This should not change during a model search.
	  *    - bool fitModel( const vector_size_t& useIndices, Model& model ) const : This function fits a model to the data selected by the indices. The return value indicates the success, hence false means a degenerate case, where no model was found.
	  *    - Real testSample( size_t index, const Model& model ) const : return some value that indicates how well a sample fits to the model. This way the thresholding is moved to the searching procedure and the model just tells how good a sample is.
	  *
	  *  There are two methods provided in this class to fit a model:
	  *    - \a ransacSingleModel (RANSAC): Just like mrpt::math::RANSAC_Template
	  *
	  *    - \a geneticSingleModel (Genetic): Provides a mixture of a genetic and the ransac algorithm.
	  *         Instead of selecting a set of data in each iteration, it takes more (ex. 10) and order these model
	  *         using some fitness function: the average error of the inliers scaled by the number of outliers (This
	  *         fitness might require some fine tuning). Than the (ex 10) new kernel for the next iteration is created as follows:
	  *             - Take the best kernels (as for the original ransac)
	  *             - Select two kernels ( with a higher probability for the better models) and let the new kernel be a subset of the two original kernels ( additionally to leave the local minimums an additional random seed might appear - mutation)
	  *             - Generate some new random samples.
	  *
	  *  For an example of usage, see "samples/model_search_test/"
	  *  \sa mrpt::math::RANSAC_Template, another RANSAC implementation where models can be matrices only.
	  *
	  *  \author Zoltar Gaal
	  * \ingroup ransac_grp
	  */
	class BASE_IMPEXP ModelSearch {
	private:
		//! Select random (unique) indices from the 0..p_size sequence
		void pickRandomIndex( size_t p_size, size_t p_pick, vector_size_t& p_ind );

		/** Select random (unique) indices from the set.
		  *  The set is destroyed during pick */
		void pickRandomIndex( std::set<size_t> p_set, size_t p_pick, vector_size_t& p_ind );

	public:
		template<typename TModelFit>
		bool	ransacSingleModel( const TModelFit& p_state,
								   size_t p_kernelSize,
								   const typename TModelFit::Real& p_fitnessThreshold,
								   typename TModelFit::Model& p_bestModel,
								   vector_size_t& p_inliers );

	private:
		template<typename TModelFit>
		struct TSpecies {
			typename TModelFit::Model	model;
			vector_size_t				sample;
			vector_size_t				inliers;
			typename TModelFit::Real	fitness;

			static bool compare( const TSpecies* p_a, const TSpecies* p_b )
			{
				return p_a->fitness < p_b->fitness;
			}
		};

	public:
		template<typename TModelFit>
		bool	geneticSingleModel( const TModelFit& p_state,
									size_t p_kernelSize,
									const typename TModelFit::Real& p_fitnessThreshold,
									size_t p_populationSize,
									size_t p_maxIteration,
									typename TModelFit::Model& p_bestModel,
									vector_size_t& p_inliers );
	}; // end of class

	} // namespace math
} // namespace mrpt

// Template implementations:
#include "model_search_impl.h"

#endif // math_modelsearch_h
