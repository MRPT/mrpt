/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/random.h>

#include <limits>

namespace mrpt
{
/// \ingroup mrpt_bayes_grp
namespace bayes
{
/** A base class for implementing rejection sampling in a generic state space.
 *   See the main method CRejectionSamplingCapable::rejectionSampling
 *  To use this class, create your own class as a child of this one and
 * implement the desired
 *   virtual methods, and add any required internal data.
 * \ingroup mrpt_bayes_grp
 */
template <
    class TStateSpace,
    mrpt::bayes::particle_storage_mode STORAGE = mrpt::bayes::particle_storage_mode::POINTER>
class CRejectionSamplingCapable
{
 public:
  using TParticle = CProbabilityParticle<TStateSpace, STORAGE>;

  /** Virtual destructor
   */
  virtual ~CRejectionSamplingCapable() = default;
  /** Generates a set of N independent samples via rejection sampling.
   * \param desiredSamples The number of desired samples to generate.
   * \param timeoutTrials The maximum number of rejection trials per sample.
   *   Samples that exceed the timeout are assigned a weight proportional to
   *   the best likelihood seen, rather than the uniform weight of accepted
   *   samples.
   * \return Vector of \a desiredSamples particles.
   */
  [[nodiscard]] std::vector<TParticle> rejectionSampling(
      size_t desiredSamples, size_t timeoutTrials = 1000)
  {
    MRPT_START

    std::vector<TParticle> outSamples(desiredSamples);

    if constexpr (STORAGE == particle_storage_mode::POINTER)
    {
      for (auto& p : outSamples)
      {
        p.d.reset(new TStateSpace);
      }
    }

    for (auto& p : outSamples)
    {
      size_t timeoutCount = 0;
      double bestLik = std::numeric_limits<double>::lowest();
      TStateSpace bestVal{};
      double acceptanceProb = 0.0;
      do
      {
        if constexpr (STORAGE == particle_storage_mode::POINTER)
        {
          RS_drawFromProposal(*p.d);
          acceptanceProb = RS_observationLikelihood(*p.d);
          ASSERT_(acceptanceProb >= 0 && acceptanceProb <= 1);
          if (acceptanceProb > bestLik)
          {
            bestLik = acceptanceProb;
            bestVal = *p.d;
          }
        }
        else
        {
          RS_drawFromProposal(p.d);
          acceptanceProb = RS_observationLikelihood(p.d);
          ASSERT_(acceptanceProb >= 0 && acceptanceProb <= 1);
          if (acceptanceProb > bestLik)
          {
            bestLik = acceptanceProb;
            bestVal = p.d;
          }
        }
      } while (acceptanceProb < mrpt::random::getRandomGenerator().drawUniform(0.0, 0.999) &&
               (++timeoutCount) < timeoutTrials);

      if (timeoutCount >= timeoutTrials)
      {
        p.log_w = std::max(std::log(bestLik), -50.0);
        if constexpr (STORAGE == particle_storage_mode::POINTER)
        {
          *p.d = bestVal;
        }
        else
        {
          p.d = bestVal;
        }
      }
      else
      {
        p.log_w = 0.0;  // log(1.0)
      }
    }

    return outSamples;
    MRPT_END
  }

 protected:
  /** Generates one sample, drawing from some proposal distribution.
   */
  virtual void RS_drawFromProposal(TStateSpace& outSample) = 0;

  /** Returns the NORMALIZED observation likelihood (linear, not
   * exponential!!!) at a given point of the state space (values in the range
   * [0,1]).
   */
  virtual double RS_observationLikelihood(const TStateSpace& x) = 0;

};  // End of class def.

}  // namespace bayes
}  // namespace mrpt
