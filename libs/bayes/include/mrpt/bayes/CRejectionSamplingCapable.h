/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CRejectionSamplingCapable_H
#define CRejectionSamplingCapable_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/random.h>

namespace mrpt
{
/// \ingroup mrpt_bayes_grp
namespace bayes
{
	/** A base class for implementing rejection sampling in a generic state space.
	 *   See the main method CRejectionSamplingCapable::rejectionSampling
	 *  To use this class, create your own class as a child of this one and implement the desired
	 *   virtual methods, and add any required internal data.
	 * \ingroup mrpt_bayes_grp
	 */
	template <class TStateSpace>
	class CRejectionSamplingCapable
	{
	public:
		typedef CProbabilityParticle<TStateSpace> TParticle;

        /** Virtual destructor
          */
        virtual ~CRejectionSamplingCapable()
		{
		}

		/** Generates a set of N independent samples via rejection sampling.
		  * \param desiredSamples The number of desired samples to generate
		  * \param outSamples The output samples.
		  * \param timeoutTrials The maximum number of rejection trials for each generated sample (i.e. the maximum number of iterations). This can be used to set a limit to the time complexity of the algorithm for difficult probability densities.
		  *  All will have equal importance weights (a property of rejection sampling), although those samples
		  *   generated at timeout will have a different importance weights.
		  */
		void rejectionSampling(
			size_t							desiredSamples,
			std::vector<TParticle>			&outSamples,
			size_t							timeoutTrials = 1000)
		{
			MRPT_START

			TStateSpace							x;
			typename std::vector<TParticle>::iterator	it;

			// Set output size:
			if ( outSamples.size() != desiredSamples )
			{
				// Free old memory:
				for (it = outSamples.begin();it!=outSamples.end();it++)
					delete (it->d);
				outSamples.clear();

				// Reserve new memory:
				outSamples.resize( desiredSamples );
				for (it = outSamples.begin();it!=outSamples.end();it++)
					it->d = new TStateSpace;
			}

			// Rejection sampling loop:
			double	acceptanceProb;
			for (it = outSamples.begin();it!=outSamples.end();it++)
			{
				size_t	timeoutCount = 0;
				double		bestLik = -1e250;
				TStateSpace	bestVal;
				do
				{
					RS_drawFromProposal( *it->d );
					acceptanceProb = RS_observationLikelihood( *it->d );
					ASSERT_(acceptanceProb>=0 && acceptanceProb<=1);
					if (acceptanceProb>bestLik)
					{
						bestLik = acceptanceProb;
						bestVal = *it->d;
					}
				} while (	acceptanceProb < mrpt::random::randomGenerator.drawUniform(0.0,0.999) &&
							(++timeoutCount)<timeoutTrials );

				// Save weights:
				if (timeoutCount>=timeoutTrials)
				{
					it->log_w = log(bestLik);
					*it->d    = bestVal;
				}
				else
				{
					it->log_w = 0; // log(1.0);
				}
			} // end for it

			MRPT_END
		}

	protected:
		/** Generates one sample, drawing from some proposal distribution.
		  */
		virtual void RS_drawFromProposal( TStateSpace &outSample ) = 0;

		/** Returns the NORMALIZED observation likelihood (linear, not exponential!!!) at a given point of the state space (values in the range [0,1]).
		  */
		virtual double RS_observationLikelihood( const TStateSpace &x) = 0;

	}; // End of class def.

} // End of namespace
} // End of namespace

#endif
