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

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>

namespace mrpt
{
namespace obs
{
class CSensoryFrame;
class CActionCollection;
}  // namespace obs

/** The namespace for Bayesian filtering algorithm: different particle filters
 * and Kalman filter algorithms. \ingroup mrpt_bayes_grp
 */
namespace bayes
{
class CParticleFilterCapable;

/** This class acts as a common interface to the different interfaces (see
 *CParticleFilter::TParticleFilterAlgorithm) any bayes::CParticleFilterCapable
 *class can implement: it is the invoker of particle filter algorithms.
 *   The particle filter is executed on a probability density function (PDF)
 *described by a CParticleFilterCapable object, passed in the constructor or
 *alternatively through the CParticleFilter::executeOn method.<br>
 *
 * For a complete example and further details, see the <a
 *href="http://www.mrpt.org/Particle_Filter_Tutorial" >Particle Filter
 *tutorial</a>.
 *
 *   The basic SIR algorithm (pfStandardProposal) consists of:
 *		- Execute a prediction with the given "action".
 *		- Update the weights of the particles using the likelihood of the
 *"observation".
 *		- Normalize weights.
 *		- Perform resampling if the ESS is below the threshold options.BETA.
 *
 * \ingroup mrpt_bayes_grp
 * \sa mrpt::poses::CPoseParticlesPDF
 */
class CParticleFilter : public mrpt::system::COutputLogger
{
 public:
  /** Defines different types of particle filter algorithms.
   *  The defined SIR implementations are:
   *		- StandardProposal: Standard proposal distribution + weights
   *according to likelihood function.
   *		- AuxiliaryPFStandard: An auxiliary PF using the standard proposal
   *distribution.
   *		- OptimalProposal: Use the optimal proposal distribution (where
   *available!, usually this will perform approximations)
   *		- AuxiliaryPFOptimal: Use the optimal proposal and an auxiliary
   *particle filter (see <a
   *href="http://www.mrpt.org/Paper:An_Optimal_Filtering_Algorithm_for_Non-Parametric_Observation_Models_in_Robot_Localization_(ICRA_2008)"
   *>paper</a>).
   *
   * See the theoretical discussion in <a
   *href="http://www.mrpt.org/Resampling_Schemes" >resampling schemes</a>.
   */
  enum class TParticleFilterAlgorithm : uint8_t
  {
    StandardProposal = 0,
    AuxiliaryPFStandard,
    OptimalProposal,
    AuxiliaryPFOptimal
  };
  // Backward-compat aliases:
  static constexpr TParticleFilterAlgorithm pfStandardProposal =
      TParticleFilterAlgorithm::StandardProposal;
  static constexpr TParticleFilterAlgorithm pfAuxiliaryPFStandard =
      TParticleFilterAlgorithm::AuxiliaryPFStandard;
  static constexpr TParticleFilterAlgorithm pfOptimalProposal =
      TParticleFilterAlgorithm::OptimalProposal;
  static constexpr TParticleFilterAlgorithm pfAuxiliaryPFOptimal =
      TParticleFilterAlgorithm::AuxiliaryPFOptimal;

  /** Defines the different resampling algorithms.
   *  The implemented resampling methods are:
   *		- Multinomial (Default): Uses standard select with replacement
   *(draws M random uniform numbers). See Doucet & Johansen 2009.
   *		- Residual: The residual or "remainder" method. See Liu & Chen 1998.
   *		- Stratified: A uniform sample is drawn for each of M subdivisions
   *of the range (0,1]. See Kitagawa 1996.
   *		- Systematic: A single uniform sample is drawn in the range
   *(0,1/M]. See Carpenter et al. 1999.
   *
   * See the theoretical discussion in <a
   *href="http://www.mrpt.org/Resampling_Schemes" >resampling schemes</a>.
   */
  enum class TParticleResamplingAlgorithm : uint8_t
  {
    Multinomial = 0,
    Residual,
    Stratified,
    Systematic
  };
  // Backward-compat aliases:
  static constexpr TParticleResamplingAlgorithm prMultinomial =
      TParticleResamplingAlgorithm::Multinomial;
  static constexpr TParticleResamplingAlgorithm prResidual = TParticleResamplingAlgorithm::Residual;
  static constexpr TParticleResamplingAlgorithm prStratified =
      TParticleResamplingAlgorithm::Stratified;
  static constexpr TParticleResamplingAlgorithm prSystematic =
      TParticleResamplingAlgorithm::Systematic;

  /** \brief The configuration of a particle filter algorithm and resampling
   * parameters. Pass this to CParticleFilter::executeOn() or set
   * CParticleFilter::m_options directly.
   */
  struct TParticleFilterOptions : public mrpt::config::CLoadableOptions
  {
   public:
    // See base docs:
    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source, const std::string& section) override;
    void saveToConfigFile(
        mrpt::config::CConfigFileBase& target, const std::string& section) const override;

    /** A flag that indicates whether the CParticleFilterCapable object
     * should perform adative sample size (default=false). */
    bool adaptiveSampleSize{false};
    /** The resampling of particles will be performed when ESS (in range
     * [0,1]) < BETA (default is 0.5) */
    double BETA{0.5};
    /** The initial number of particles in the filter (it can change only if
     * adaptiveSampleSize=true) (default=1) */
    unsigned int sampleSize{1};

    /** In the algorithm "CParticleFilter::pfAuxiliaryPFOptimal" (and in
     * "CParticleFilter::pfAuxiliaryPFStandard" only if
     * pfAuxFilterStandard_FirstStageWeightsMonteCarlo = true) the number of
     * samples for searching the maximum likelihood value and also to
     * estimate the "first stage weights" (see papers!) (default=100)
     */
    unsigned int pfAuxFilterOptimal_MaximumSearchSamples{100};
    /** An optional step to "smooth" dramatic changes in the observation
     * model to affect the variance of the particle weights, eg
     * weight*=likelihood^powFactor (default=1 = no effects). */
    double powFactor{1};
    /** The PF algorithm to use (default=StandardProposal) See
     * TParticleFilterAlgorithm for the possibilities. */
    TParticleFilterAlgorithm PF_algorithm{TParticleFilterAlgorithm::StandardProposal};
    /** The resampling algorithm to use (default=Multinomial). */
    TParticleResamplingAlgorithm resamplingMethod{TParticleResamplingAlgorithm::Multinomial};

    /** Only for PF_algorithm=pfAuxiliaryPFOptimal: If a given particle has
     * a max_likelihood (from the a-priori estimate) below the maximum from
     * all the samples - max_loglikelihood_dyn_range, then the particle is
     * directly discarded.
     *  This is done to assure that the rejection sampling doesn't get
     * stuck in an infinite loop trying to get an acceptable sample.
     *  Default = 15 (in logarithmic likelihood)
     */
    double max_loglikelihood_dyn_range{15};

    /** Only for PF_algorithm==pfAuxiliaryPFStandard:
     * If false, the APF will predict the first stage weights just at the
     * mean of the prior of the next time step.
     * If true, these weights will be estimated as described in the papers
     * for the "pfAuxiliaryPFOptimal" method, i.e. through a monte carlo
     * simulation.
     *  In that case, "pfAuxFilterOptimal_MaximumSearchSamples" is the
     * number of MC samples used.
     */
    bool pfAuxFilterStandard_FirstStageWeightsMonteCarlo{false};

    /** (Default=false) In the algorithm
     * "CParticleFilter::pfAuxiliaryPFOptimal", if set to true, do not
     * perform rejection sampling, but just the most-likely (ML) particle
     * found in the preliminary weight-determination stage. */
    bool pfAuxFilterOptimal_MLE{false};
  };

  /** \brief Statistics returned from CParticleFilter::executeOn(). */
  struct TParticleFilterStats
  {
    /** \brief Effective sample size (ESS) computed before the resampling step. */
    double ESS_beforeResample{0};
    /** \brief Variance of the normalized linear weights before resampling. */
    double weightsVariance_beforeResample{0};
  };

  /** Default constructor.
   *   After creating the PF object, set the options in
   * CParticleFilter::m_options, then execute steps through
   * CParticleFilter::executeOn.
   */
  CParticleFilter();

  ~CParticleFilter() override = default;
  /** Executes a complete prediction + update step of the selected particle
   * filtering algorithm.
   *    The member CParticleFilter::m_options must be set before calling this
   * to settle the algorithm parameters.
   *
   * \param obj           The object representing the probability distribution
   * function (PDF) which apply the particle filter algorithm to.
   * \param action		A pointer to an action in the form of a
   * CActionCollection,
   * or nullptr if there is no action.
   * \param observation	A pointer to observations in the form of a
   * CSensoryFrame, or nullptr if there is no observation.
   * \param stats An output structure for gathering statistics of the particle
   * filter execution, or set to nullptr if you do not need it (see
   * CParticleFilter::TParticleFilterStats).
   *
   * \sa CParticleFilterCapable, executeOn
   */
  void executeOn(
      CParticleFilterCapable& obj,
      const mrpt::obs::CActionCollection* action,
      const mrpt::obs::CSensoryFrame* observation,
      TParticleFilterStats* stats = nullptr) const;

  /** The options to be used in the PF, must be set before executing any step
   * of the particle filter.
   */
  CParticleFilter::TParticleFilterOptions m_options;

};  // End of class def.

}  // namespace bayes
}  // namespace mrpt

MRPT_ENUM_TYPE_BEGIN(mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm)
MRPT_FILL_ENUM_CUSTOM_NAME(
    mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm::StandardProposal, "pfStandardProposal");
MRPT_FILL_ENUM_CUSTOM_NAME(
    mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm::AuxiliaryPFStandard,
    "pfAuxiliaryPFStandard");
MRPT_FILL_ENUM_CUSTOM_NAME(
    mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm::OptimalProposal, "pfOptimalProposal");
MRPT_FILL_ENUM_CUSTOM_NAME(
    mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm::AuxiliaryPFOptimal,
    "pfAuxiliaryPFOptimal");
MRPT_ENUM_TYPE_END()

MRPT_ENUM_TYPE_BEGIN(mrpt::bayes::CParticleFilter::TParticleResamplingAlgorithm)
MRPT_FILL_ENUM_CUSTOM_NAME(
    mrpt::bayes::CParticleFilter::TParticleResamplingAlgorithm::Multinomial, "prMultinomial");
MRPT_FILL_ENUM_CUSTOM_NAME(
    mrpt::bayes::CParticleFilter::TParticleResamplingAlgorithm::Residual, "prResidual");
MRPT_FILL_ENUM_CUSTOM_NAME(
    mrpt::bayes::CParticleFilter::TParticleResamplingAlgorithm::Stratified, "prStratified");
MRPT_FILL_ENUM_CUSTOM_NAME(
    mrpt::bayes::CParticleFilter::TParticleResamplingAlgorithm::Systematic, "prSystematic");
MRPT_ENUM_TYPE_END()
