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


#include <mrpt/math/ransac.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::math;
using namespace std;


/*---------------------------------------------------------------
			ransac generic implementation
 ---------------------------------------------------------------*/
template <typename NUMTYPE>
bool RANSAC_Template<NUMTYPE>::execute(
	const CMatrixTemplateNumeric<NUMTYPE>	  &data,
	TRansacFitFunctor			fit_func,
	TRansacDistanceFunctor  	dist_func,
	TRansacDegenerateFunctor 	degen_func,
	const double   				distanceThreshold,
	const unsigned int			minimumSizeSamplesToFit,
	mrpt::vector_size_t			&out_best_inliers,
	CMatrixTemplateNumeric<NUMTYPE> &out_best_model,
	bool						verbose,
	const double                p,
	const size_t				maxIter
	)
{
	MRPT_START

	ASSERT_(minimumSizeSamplesToFit>=1)

	// Highly inspired on http://www.csse.uwa.edu.au/~pk/

	const size_t D = size(data,1);  //  dimensionality
	const size_t Npts = size(data,2);

	ASSERT_(D>=1);
	ASSERT_(Npts>1);

	const size_t maxDataTrials = 100; // Maximum number of attempts to select a non-degenerate data set.

	out_best_model.setSize(0,0);  // Sentinel value allowing detection of solution failure.

    size_t trialcount = 0;  //
    size_t bestscore =  0;
    size_t N = 1;     // Dummy initialisation for number of trials.

    vector_size_t   ind( minimumSizeSamplesToFit );

    while (N > trialcount)
    {
        // Select at random s datapoints to form a trial model, M.
        // In selecting these points we have to check that they are not in
        // a degenerate configuration.
        bool degenerate=true;
        size_t count = 1;
        std::vector< CMatrixTemplateNumeric<NUMTYPE> >  MODELS;

        while (degenerate)
        {
            // Generate s random indicies in the range 1..npts
            ind.resize( minimumSizeSamplesToFit );

			// The +0.99... is due to the floor rounding afterwards when converting from random double samples to size_t
			randomGenerator.drawUniformVector(ind,0.0, Npts-1+0.999999 );

            // Test that these points are not a degenerate configuration.
            degenerate = degen_func(data, ind);

            if (!degenerate)
            {
                // Fit model to this random selection of data points.
                // Note that M may represent a set of models that fit the data
                fit_func(data,ind,MODELS);

                // Depending on your problem it might be that the only way you
                // can determine whether a data set is degenerate or not is to
				// try to fit a model and see if it succeeds.  If it fails we
                // reset degenerate to true.
                degenerate = MODELS.empty();
			}

            // Safeguard against being stuck in this loop forever
            if (++count > maxDataTrials)
            {
                if (verbose) printf_debug("[RANSAC] Unable to select a nondegenerate data set\n");
                break;
            }
        }

        // Once we are out here we should have some kind of model...
        // Evaluate distances between points and model returning the indices
        // of elements in x that are inliers.  Additionally, if M is a cell
        // array of possible models 'distfn' will return the model that has
        // the most inliers.  After this call M will be a non-cell objec
        // representing only one model.
        unsigned int bestModelIdx = 1000;
        mrpt::vector_size_t   inliers;
        if (!degenerate)
        {
			dist_func(data,MODELS, distanceThreshold, bestModelIdx, inliers);
			ASSERT_( bestModelIdx<MODELS.size() );
        }

        // Find the number of inliers to this model.
        const size_t ninliers = inliers.size();

        if (ninliers > bestscore )
        {
            bestscore = ninliers;  // Record data for this model

            out_best_model    = MODELS[bestModelIdx];
            out_best_inliers  = inliers;

            // Update estimate of N, the number of trials to ensure we pick,
            // with probability p, a data set with no outliers.
            double fracinliers =  ninliers/static_cast<double>(Npts);
            double pNoOutliers = 1 -  pow(fracinliers,static_cast<double>(minimumSizeSamplesToFit));

            pNoOutliers = std::max( std::numeric_limits<double>::epsilon(), pNoOutliers);  // Avoid division by -Inf
            pNoOutliers = std::min(1.0 - std::numeric_limits<double>::epsilon() , pNoOutliers); // Avoid division by 0.
            // Number of
            N = log(1-p)/log(pNoOutliers);
			if (verbose)
				printf_debug("[RANSAC] Iter #%u Estimated number of iters: %u  pNoOutliers = %f  #inliers: %u\n", (unsigned)trialcount ,(unsigned)N,pNoOutliers, (unsigned)ninliers);
        }

        ++trialcount;

        if (verbose)
            printf_debug("[RANSAC] trial %u out of %u \r",(unsigned int)trialcount, (unsigned int)ceil(static_cast<double>(N)));

        // Safeguard against being stuck in this loop forever
        if (trialcount > maxIter)
        {
			printf_debug("[RANSAC] Warning: maximum number of trials (%u) reached\n", (unsigned)maxIter);
            break;
        }
    }

    if (size(out_best_model,1)>0)
    {  // We got a solution
		if (verbose)
			printf_debug("[RANSAC]  Finished in %u iterations.\n",(unsigned)trialcount );
    	return true;
    }
    else
    {
		if (verbose)
			printf_debug("[RANSAC] Warning: Finished without any proper solution.\n");
    	return false;
    }

	MRPT_END
}




// Template instantiation:
template class BASE_IMPEXP mrpt::math::RANSAC_Template<float>;
template class BASE_IMPEXP mrpt::math::RANSAC_Template<double>;

#ifdef HAVE_LONG_DOUBLE
template class BASE_IMPEXP mrpt::math::RANSAC_Template<long double>;
#endif

