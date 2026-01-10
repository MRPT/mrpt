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

#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/random.h>

#include <algorithm>

using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::random;
using namespace mrpt::math;
using namespace std;

const unsigned CParticleFilterCapable::PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS = 20;

/*---------------------------------------------------------------
          performResampling
 ---------------------------------------------------------------*/
void CParticleFilterCapable::performResampling(
    const bayes::CParticleFilter::TParticleFilterOptions& PF_options, size_t out_particle_count)
{
  MRPT_START

  // Make a vector with the particles' log weights:
  const size_t in_particle_count = particlesCount();
  ASSERT_(in_particle_count > 0);

  vector<size_t> indxs;
  vector<double> log_ws(in_particle_count, 0.0);

  for (size_t i = 0; i < in_particle_count; i++)
  {
    log_ws[i] = getW(i);
  }

  // Compute the surviving indexes:
  computeResampling(PF_options.resamplingMethod, log_ws, indxs, out_particle_count);

  // Perform the particle replacement:
  performSubstitution(indxs);

  // Finally, assign equal weights (logarithmic):
  for (size_t i = 0; i < out_particle_count; i++)
  {
    setW(i, 0.0);
  }

  MRPT_END
}

/*---------------------------------------------------------------
          computeResampling
 ---------------------------------------------------------------*/
void CParticleFilterCapable::computeResampling(
    CParticleFilter::TParticleResamplingAlgorithm method,
    const vector<double>& in_logWeights,
    vector<size_t>& out_indexes,
    size_t out_particle_count)
{
  MRPT_START

  // Compute the normalized linear weights:
  // The array "linW" will be the input to the actual resampling algorithms.
  const size_t M = in_logWeights.size();
  ASSERT_(M > 0);

  if (out_particle_count == 0)
  {
    out_particle_count = M;
  }

  vector<double> linW(M, 0.0);
  double linW_SUM = 0.0;

  // Avoid floating point range problems by subtracting the maximum log weight
  const double max_log_w = math::maximum(in_logWeights);

  for (size_t i = 0; i < M; i++)
  {
    linW[i] = exp(in_logWeights[i] - max_log_w);
    linW_SUM += linW[i];
  }

  // Normalize weights:
  ASSERT_(linW_SUM > 0);
  linW *= 1.0 / linW_SUM;

  switch (method)
  {
    case CParticleFilter::prMultinomial:
    {
      // ==============================================
      //   Multinomial resampling (select with replacement)
      // ==============================================
      vector<double> Q;
      mrpt::math::cumsum_tmpl<vector<double>, vector<double>>(linW, Q);
      Q[M - 1] = 1.1;  // Ensure last element exceeds 1.0 for comparison

      vector<double> T(M);
      getRandomGenerator().drawUniformVector(T, 0.0, 0.999999);
      T.push_back(1.0);

      // Sort random values
      sort(T.begin(), T.end());

      out_indexes.resize(out_particle_count);
      size_t i = 0;
      size_t j = 0;

      while (i < out_particle_count)
      {
        if (T[i] < Q[j])
        {
          out_indexes[i++] = j;
        }
        else
        {
          j++;
          if (j >= M)
          {
            j = M - 1;
          }
        }
      }
    }
    break;

    case CParticleFilter::prResidual:
    {
      // ==============================================
      //   Residual resampling
      // ==============================================
      // Compute repetition counts for deterministic part
      std::vector<uint32_t> N(M);
      size_t R = 0;  // Total count for deterministic part

      for (size_t i = 0; i < M; i++)
      {
        N[i] = static_cast<uint32_t>(static_cast<double>(M) * linW[i]);
        R += N[i];
      }

      // Number of particles to be drawn randomly (the "residual" part)
      const size_t N_rnd = (out_particle_count >= R) ? (out_particle_count - R) : 0;

      // Fill out the deterministic part of the resampling
      out_indexes.resize(out_particle_count);
      size_t j = 0;

      for (size_t i = 0; i < M && j < out_particle_count; i++)
      {
        for (size_t k = 0; k < N[i]; k++)
        {
          out_indexes[j++] = i;
        }
      }

      const size_t M_fixed = j;

      // Prepare multinomial resampling for the residual part using modified weights
      if (N_rnd > 0)
      {
        // Compute modified weights
        vector<double> linW_mod(M);
        const double M_R_1 = 1.0 / static_cast<double>(N_rnd);

        for (size_t i = 0; i < M; i++)
        {
          linW_mod[i] = M_R_1 * (static_cast<double>(M) * linW[i] - N[i]);
        }

        // Perform resampling on residual
        vector<double> Q;
        mrpt::math::cumsum_tmpl<vector<double>, vector<double>>(linW_mod, Q);
        Q[M - 1] = 1.1;

        vector<double> T(M);
        getRandomGenerator().drawUniformVector(T, 0.0, 0.999999);
        T.push_back(1.0);

        // Sort random values
        sort(T.begin(), T.end());

        size_t i = 0;
        j = 0;

        while (i < N_rnd)
        {
          if (T[i] < Q[j])
          {
            out_indexes[M_fixed + i++] = j;
          }
          else
          {
            j++;
            if (j >= M)
            {
              j = M - 1;
            }
          }
        }
      }
    }
    break;

    case CParticleFilter::prStratified:
    {
      // ==============================================
      //   Stratified resampling
      // ==============================================
      vector<double> Q;
      mrpt::math::cumsum_tmpl<vector<double>, vector<double>>(linW, Q);
      Q[M - 1] = 1.1;

      // Generate stratified-uniform random vector
      vector<double> T(M + 1);
      const double _1_M = 1.0 / static_cast<double>(M);
      const double _1_M_eps = _1_M - 0.000001;
      double T_offset = 0.0;

      for (size_t i = 0; i < M; i++)
      {
        T[i] = T_offset + getRandomGenerator().drawUniform(0.0, _1_M_eps);
        T_offset += _1_M;
      }
      T[M] = 1.0;

      out_indexes.resize(out_particle_count);
      size_t i = 0;
      size_t j = 0;

      while (i < out_particle_count)
      {
        if (T[i] < Q[j])
        {
          out_indexes[i++] = j;
        }
        else
        {
          j++;
          if (j >= M)
          {
            j = M - 1;
          }
        }
      }
    }
    break;

    case CParticleFilter::prSystematic:
    {
      // ==============================================
      //   Systematic resampling
      // ==============================================
      vector<double> Q;
      mrpt::math::cumsum_tmpl<vector<double>, vector<double>>(linW, Q);
      Q[M - 1] = 1.1;

      // Generate uniform random vector with fixed spacing
      vector<double> T(M + 1);
      const double _1_M = 1.0 / static_cast<double>(M);
      T[0] = getRandomGenerator().drawUniform(0.0, _1_M);

      for (size_t i = 1; i < M; i++)
      {
        T[i] = T[i - 1] + _1_M;
      }
      T[M] = 1.0;

      out_indexes.resize(out_particle_count);
      size_t i = 0;
      size_t j = 0;

      while (i < out_particle_count)
      {
        if (T[i] < Q[j])
        {
          out_indexes[i++] = j;
        }
        else
        {
          j++;
          if (j >= M)
          {
            j = M - 1;
          }
        }
      }
    }
    break;

    default:
      THROW_EXCEPTION(format("ERROR: Unknown resampling method selected: %i", method));
  }

  MRPT_END
}

/*---------------------------------------------------------------
          prediction_and_update
 ---------------------------------------------------------------*/
void CParticleFilterCapable::prediction_and_update(
    const mrpt::obs::CActionCollection* action,
    const mrpt::obs::CSensoryFrame* observation,
    const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
  switch (PF_options.PF_algorithm)
  {
    case CParticleFilter::pfStandardProposal:
      prediction_and_update_pfStandardProposal(action, observation, PF_options);
      break;

    case CParticleFilter::pfAuxiliaryPFStandard:
      prediction_and_update_pfAuxiliaryPFStandard(action, observation, PF_options);
      break;

    case CParticleFilter::pfOptimalProposal:
      prediction_and_update_pfOptimalProposal(action, observation, PF_options);
      break;

    case CParticleFilter::pfAuxiliaryPFOptimal:
      prediction_and_update_pfAuxiliaryPFOptimal(action, observation, PF_options);
      break;

    default:
      THROW_EXCEPTION("Invalid particle filter algorithm selection!");
  }
}

/*---------------------------------------------------------------
          prediction_and_update_pfStandardProposal
 ---------------------------------------------------------------*/
void CParticleFilterCapable::prediction_and_update_pfStandardProposal(
    [[maybe_unused]] const mrpt::obs::CActionCollection* action,
    [[maybe_unused]] const mrpt::obs::CSensoryFrame* observation,
    [[maybe_unused]] const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
  THROW_EXCEPTION("Algorithm 'pfStandardProposal' is not implemented in inherited class!");
}

/*---------------------------------------------------------------
          prediction_and_update_pfAuxiliaryPFStandard
 ---------------------------------------------------------------*/
void CParticleFilterCapable::prediction_and_update_pfAuxiliaryPFStandard(
    [[maybe_unused]] const mrpt::obs::CActionCollection* action,
    [[maybe_unused]] const mrpt::obs::CSensoryFrame* observation,
    [[maybe_unused]] const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
  THROW_EXCEPTION("Algorithm 'pfAuxiliaryPFStandard' is not implemented in inherited class!");
}

/*---------------------------------------------------------------
          prediction_and_update_pfOptimalProposal
 ---------------------------------------------------------------*/
void CParticleFilterCapable::prediction_and_update_pfOptimalProposal(
    [[maybe_unused]] const mrpt::obs::CActionCollection* action,
    [[maybe_unused]] const mrpt::obs::CSensoryFrame* observation,
    [[maybe_unused]] const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
  THROW_EXCEPTION("Algorithm 'pfOptimalProposal' is not implemented in inherited class!");
}

/*---------------------------------------------------------------
          prediction_and_update_pfAuxiliaryPFOptimal
 ---------------------------------------------------------------*/
void CParticleFilterCapable::prediction_and_update_pfAuxiliaryPFOptimal(
    [[maybe_unused]] const mrpt::obs::CActionCollection* action,
    [[maybe_unused]] const mrpt::obs::CSensoryFrame* observation,
    [[maybe_unused]] const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
  THROW_EXCEPTION("Algorithm 'pfAuxiliaryPFOptimal' is not implemented in inherited class!");
}

/*---------------------------------------------------------------
          prepareFastDrawSample
 ---------------------------------------------------------------*/
void CParticleFilterCapable::prepareFastDrawSample(
    const bayes::CParticleFilter::TParticleFilterOptions& PF_options,
    TParticleProbabilityEvaluator partEvaluator,
    const void* action,
    const void* observation) const
{
  MRPT_START

  if (PF_options.adaptiveSampleSize)
  {
    // --------------------------------------------------------
    // CASE: Dynamic number of particles
    //  -> Use m_fastDrawAuxiliary.CDF, PDF, CDF_indexes
    // --------------------------------------------------------
    if (PF_options.resamplingMethod != CParticleFilter::prMultinomial)
    {
      THROW_EXCEPTION(
          "resamplingMethod must be 'prMultinomial' for a dynamic number of particles!");
    }

    const size_t M = particlesCount();

    MRPT_START

    // Prepare buffers
    m_fastDrawAuxiliary.CDF.resize(1 + PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS, 0.0);
    m_fastDrawAuxiliary.CDF_indexes.resize(PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS, 0);

    // Compute the vector of each particle's probability
    // (usually it will be simply the weight, but other algorithms may differ)
    m_fastDrawAuxiliary.PDF.resize(M, 0.0);

    // Avoid floating point overflow (JLBC - SEP 2007)
    double SUM = 0.0;

    // Save the log likelihoods
    for (size_t i = 0; i < M; i++)
    {
      m_fastDrawAuxiliary.PDF[i] = partEvaluator(PF_options, this, i, action, observation);
    }

    // Normalize by subtracting maximum
    m_fastDrawAuxiliary.PDF += -math::maximum(m_fastDrawAuxiliary.PDF);

    for (size_t i = 0; i < M; i++)
    {
      m_fastDrawAuxiliary.PDF[i] = exp(m_fastDrawAuxiliary.PDF[i]);
      SUM += m_fastDrawAuxiliary.PDF[i];
    }

    ASSERT_(SUM >= 0);
    MRPT_CHECK_NORMAL_NUMBER(SUM);
    m_fastDrawAuxiliary.PDF *= 1.0 / SUM;

    // Compute the CDF thresholds
    for (size_t i = 0; i < PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS; i++)
    {
      m_fastDrawAuxiliary.CDF[i] =
          static_cast<double>(i) / static_cast<double>(PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS);
    }
    m_fastDrawAuxiliary.CDF[PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS] = 1.0;

    // Compute the CDF and save threshold indexes
    double CDF = 0.0;  // Cumulative density function
    size_t j = 0;

    for (size_t i = 0; i < M && j < PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS; i++)
    {
      double CDF_next = CDF + m_fastDrawAuxiliary.PDF[i];

      if (i == (M - 1))
      {
        CDF_next = 1.0;  // Rounding fix
      }

      if (CDF_next > 1.0)
      {
        CDF_next = 1.0;
      }

      while (m_fastDrawAuxiliary.CDF[j] < CDF_next)
      {
        m_fastDrawAuxiliary.CDF_indexes[j++] = static_cast<unsigned int>(i);
      }

      CDF = CDF_next;
    }

    ASSERT_(j == PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS);

    MRPT_END
  }
  else
  {
    // ------------------------------------------------------------------------
    // CASE: Static number of particles
    //  -> Use m_fastDrawAuxiliary.alreadyDrawnIndexes & alreadyDrawnNextOne
    // ------------------------------------------------------------------------

    // Generate the vector with the "probabilities" of each particle being selected
    const size_t M = particlesCount();
    vector<double> PDF(M, 0.0);

    for (size_t i = 0; i < M; i++)
    {
      // Default evaluator: takes current weight
      PDF[i] = partEvaluator(PF_options, this, i, action, observation);
    }

    // Generate the particle samples
    vector<size_t> idxs;
    computeResampling(PF_options.resamplingMethod, PDF, idxs);

    // Convert to uint32_t for storage
    m_fastDrawAuxiliary.alreadyDrawnIndexes.resize(idxs.size());

    for (size_t i = 0; i < idxs.size(); i++)
    {
      m_fastDrawAuxiliary.alreadyDrawnIndexes[i] = static_cast<uint32_t>(idxs[i]);
    }

    m_fastDrawAuxiliary.alreadyDrawnNextOne = 0;
  }

  MRPT_END
}

/*---------------------------------------------------------------
          fastDrawSample
 ---------------------------------------------------------------*/
size_t CParticleFilterCapable::fastDrawSample(
    const bayes::CParticleFilter::TParticleFilterOptions& PF_options) const
{
  MRPT_START

  if (PF_options.adaptiveSampleSize)
  {
    // --------------------------------------------------------
    // CASE: Dynamic number of particles
    //  -> Use m_fastDrawAuxiliary.CDF, PDF, CDF_indexes
    // --------------------------------------------------------
    if (PF_options.resamplingMethod != CParticleFilter::prMultinomial)
    {
      THROW_EXCEPTION(
          "resamplingMethod must be 'prMultinomial' for a dynamic number of particles!");
    }

    const double draw = getRandomGenerator().drawUniform(0.0, 0.999999);
    double CDF_next = -1.0;
    double CDF = -1.0;

    MRPT_START

    // Use the look-up table to find the starting index
    const size_t j = static_cast<size_t>(
        floor(draw * (static_cast<double>(PARTICLE_FILTER_CAPABLE_FAST_DRAW_BINS) - 0.05)));

    CDF = m_fastDrawAuxiliary.CDF[j];
    size_t i = m_fastDrawAuxiliary.CDF_indexes[j];

    // Find the drawn particle
    while (draw > (CDF_next = CDF + m_fastDrawAuxiliary.PDF[i]))
    {
      CDF = CDF_next;
      i++;
    }

    return i;

    MRPT_END_WITH_CLEAN_UP(
        printf(
            "\n[CParticleFilterCapable::fastDrawSample] DEBUG: draw=%f, CDF=%f CDF_next=%f\n", draw,
            CDF, CDF_next););
  }
  else
  {
    // --------------------------------------------------------
    // CASE: Static number of particles
    //  -> Use m_fastDrawAuxiliary.alreadyDrawnIndexes & alreadyDrawnNextOne
    // --------------------------------------------------------
    if (m_fastDrawAuxiliary.alreadyDrawnNextOne >= m_fastDrawAuxiliary.alreadyDrawnIndexes.size())
    {
      THROW_EXCEPTION(
          "Have you called 'fastDrawSample' more times than the sample size? "
          "Did you forget calling 'prepareFastCall' before?");
    }

    return m_fastDrawAuxiliary.alreadyDrawnIndexes[m_fastDrawAuxiliary.alreadyDrawnNextOne++];
  }

  MRPT_END
}

/*---------------------------------------------------------------
          log2linearWeights
 ---------------------------------------------------------------*/
void CParticleFilterCapable::log2linearWeights(
    const vector<double>& in_logWeights, vector<double>& out_linWeights)
{
  MRPT_START

  const size_t N = in_logWeights.size();
  out_linWeights.resize(N);

  if (N == 0)
  {
    return;
  }

  double sumW = 0.0;

  for (size_t i = 0; i < N; i++)
  {
    out_linWeights[i] = exp(in_logWeights[i]);
    sumW += out_linWeights[i];
  }

  ASSERT_(sumW > 0);

  for (size_t i = 0; i < N; i++)
  {
    out_linWeights[i] /= sumW;
  }

  MRPT_END
}