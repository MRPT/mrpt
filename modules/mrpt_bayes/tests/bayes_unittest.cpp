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

#include <gtest/gtest.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/bayes/CRejectionSamplingCapable.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/random.h>

#include <cmath>
#include <numeric>

using namespace mrpt::bayes;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Simple particle filter holding double values. */
struct SimpleParticlePDF :
    public CParticleFilterData<double, particle_storage_mode::VALUE>,
    public CParticleFilterDataImpl<
        SimpleParticlePDF,
        CParticleFilterData<double, particle_storage_mode::VALUE>::CParticleList>
{
};

// ---------------------------------------------------------------------------
// log2linearWeights
// ---------------------------------------------------------------------------

TEST(BayesTest, log2linearWeights_sum_to_one)
{
  const std::vector<double> logW = {-1.0, -2.0, -0.5, -3.0};
  std::vector<double> linW;
  CParticleFilterCapable::log2linearWeights(logW, linW);

  ASSERT_EQ(linW.size(), logW.size());

  double sum = std::accumulate(linW.begin(), linW.end(), 0.0);
  EXPECT_NEAR(sum, 1.0, 1e-10);

  // Max weight is at index 2 (log_w = -0.5)
  const size_t maxIdx =
      static_cast<size_t>(std::max_element(linW.begin(), linW.end()) - linW.begin());
  EXPECT_EQ(maxIdx, 2u);
}

TEST(BayesTest, logWeightsToLinear_matches_log2linearWeights)
{
  const std::vector<double> logW = {0.0, -1.0, -2.0};
  std::vector<double> out1;
  CParticleFilterCapable::log2linearWeights(logW, out1);
  const auto out2 = CParticleFilterCapable::logWeightsToLinear(logW);
  ASSERT_EQ(out1.size(), out2.size());
  for (size_t i = 0; i < out1.size(); i++)
  {
    EXPECT_NEAR(out1[i], out2[i], 1e-15);
  }
}

// ---------------------------------------------------------------------------
// ESS
// ---------------------------------------------------------------------------

TEST(BayesTest, ESS_equal_weights_is_one)
{
  SimpleParticlePDF pdf;
  const size_t N = 100;
  pdf.m_particles.resize(N);
  for (auto& p : pdf.m_particles)
  {
    p.log_w = 0.0;
    p.d = 0.0;
  }
  EXPECT_NEAR(pdf.ESS(), 1.0, 1e-10);
}

TEST(BayesTest, ESS_one_dominant_weight_is_near_zero)
{
  SimpleParticlePDF pdf;
  const size_t N = 100;
  pdf.m_particles.resize(N);
  for (size_t i = 0; i < N; i++)
  {
    pdf.m_particles[i].log_w = (i == 0) ? 0.0 : -100.0;
    pdf.m_particles[i].d = 0.0;
  }
  // ESS should be approximately 1/N
  EXPECT_LT(pdf.ESS(), 0.1);
}

// ---------------------------------------------------------------------------
// normalizeWeights
// ---------------------------------------------------------------------------

TEST(BayesTest, normalizeWeights_max_becomes_zero)
{
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(4);
  pdf.m_particles[0].log_w = -1.0;
  pdf.m_particles[1].log_w = -3.0;
  pdf.m_particles[2].log_w = -0.5;
  pdf.m_particles[3].log_w = -2.0;
  for (auto& p : pdf.m_particles)
  {
    p.d = 0.0;
  }

  double out_max_log_w = 0.0;
  [[maybe_unused]] auto ratio = pdf.normalizeWeights(&out_max_log_w);
  EXPECT_NEAR(out_max_log_w, -0.5, 1e-12);

  double maxW = -1e300;
  for (const auto& p : pdf.m_particles)
  {
    maxW = std::max(maxW, p.log_w);
  }
  EXPECT_NEAR(maxW, 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// computeResampling — all four methods
// ---------------------------------------------------------------------------

static void check_resampling(CParticleFilter::TParticleResamplingAlgorithm method)
{
  // Trivial: uniform weights => output indexes should cover all particles.
  const size_t N = 50;
  std::vector<double> logW(N, 0.0);
  std::vector<size_t> idx;
  CParticleFilterCapable::computeResampling(method, logW, idx);

  ASSERT_EQ(idx.size(), N);
  for (size_t i : idx)
  {
    EXPECT_LT(i, N);
  }
}

TEST(BayesTest, computeResampling_Multinomial)
{
  check_resampling(CParticleFilter::TParticleResamplingAlgorithm::Multinomial);
}
TEST(BayesTest, computeResampling_Residual)
{
  check_resampling(CParticleFilter::TParticleResamplingAlgorithm::Residual);
}
TEST(BayesTest, computeResampling_Stratified)
{
  check_resampling(CParticleFilter::TParticleResamplingAlgorithm::Stratified);
}
TEST(BayesTest, computeResampling_Systematic)
{
  check_resampling(CParticleFilter::TParticleResamplingAlgorithm::Systematic);
}

TEST(BayesTest, computeResampling_dominated_weight)
{
  // One particle has all the weight: resampling should produce mostly index 2.
  const size_t N = 20;
  std::vector<double> logW(N, -100.0);
  logW[2] = 0.0;
  std::vector<size_t> idx;
  CParticleFilterCapable::computeResampling(
      CParticleFilter::TParticleResamplingAlgorithm::Multinomial, logW, idx);

  ASSERT_EQ(idx.size(), N);
  for (size_t i : idx)
  {
    EXPECT_EQ(i, 2u);
  }
}

// ---------------------------------------------------------------------------
// performSubstitution — VALUE storage
// ---------------------------------------------------------------------------

TEST(BayesTest, performSubstitution_value_storage)
{
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(3);
  for (size_t i = 0; i < 3; i++)
  {
    pdf.m_particles[i].d = static_cast<double>(i * 10);
    pdf.m_particles[i].log_w = 0.0;
  }

  // Duplicate particle 1, drop particle 2
  pdf.performSubstitution({0, 1, 1});

  ASSERT_EQ(pdf.m_particles.size(), 3u);
  EXPECT_NEAR(pdf.m_particles[0].d, 0.0, 1e-12);
  EXPECT_NEAR(pdf.m_particles[1].d, 10.0, 1e-12);
  EXPECT_NEAR(pdf.m_particles[2].d, 10.0, 1e-12);
}

// ---------------------------------------------------------------------------
// getWeights / return-by-value overload
// ---------------------------------------------------------------------------

TEST(BayesTest, getWeights_return_by_value)
{
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(3);
  pdf.m_particles[0].log_w = -1.0;
  pdf.m_particles[1].log_w = -2.0;
  pdf.m_particles[2].log_w = -3.0;
  for (auto& p : pdf.m_particles)
  {
    p.d = 0.0;
  }

  const auto w = pdf.getWeights();
  ASSERT_EQ(w.size(), 3u);
  EXPECT_NEAR(w[0], -1.0, 1e-15);
  EXPECT_NEAR(w[1], -2.0, 1e-15);
  EXPECT_NEAR(w[2], -3.0, 1e-15);
}

// ---------------------------------------------------------------------------
// CRejectionSamplingCapable
// ---------------------------------------------------------------------------

/** 1-D Gaussian rejection sampler: proposal = Uniform[-5,5], target = N(0,1). */
struct GaussianRS : public CRejectionSamplingCapable<double, particle_storage_mode::VALUE>
{
 protected:
  void RS_drawFromProposal(double& x) override
  {
    x = mrpt::random::getRandomGenerator().drawUniform(-5.0, 5.0);
  }
  double RS_observationLikelihood(const double& x) override
  {
    // Unnormalized Gaussian, scaled to [0,1] within [-5,5]
    return std::exp(-0.5 * x * x);
  }
};

TEST(BayesTest, rejectionSampling_samples_near_zero)
{
  mrpt::random::Randomize(42);
  GaussianRS sampler;
  auto samples = sampler.rejectionSampling(200, 2000);

  ASSERT_EQ(samples.size(), 200u);

  double mean = 0.0;
  for (const auto& p : samples)
  {
    mean += p.d;
  }
  mean /= static_cast<double>(samples.size());

  // Mean of samples drawn from N(0,1) should be close to 0
  EXPECT_LT(std::abs(mean), 0.3);
}

/** A proposal/likelihood pair with a vanishingly small acceptance
 * probability, forcing the timeout branch of rejectionSampling(). */
struct AlwaysRejectRS : public CRejectionSamplingCapable<double, particle_storage_mode::VALUE>
{
 protected:
  void RS_drawFromProposal(double& x) override { x = 0.0; }
  double RS_observationLikelihood(const double&) override { return 1e-6; }
};

TEST(BayesTest, rejectionSampling_timeoutPath)
{
  mrpt::random::Randomize(1);
  AlwaysRejectRS sampler;
  // timeoutTrials=1: virtually guaranteed to hit the timeout branch since the
  // acceptance probability is tiny.
  auto samples = sampler.rejectionSampling(5, 1);
  ASSERT_EQ(samples.size(), 5u);
  for (const auto& p : samples)
  {
    EXPECT_LE(p.log_w, 0.0);
  }
}

// ---------------------------------------------------------------------------
// CParticleFilterCapable virtual dispatch (getW/setW/particlesCount) and
// normalizeWeights() edge cases.
// ---------------------------------------------------------------------------

TEST(BayesTest, virtualDispatch_getW_setW_particlesCount)
{
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(3);
  for (auto& p : pdf.m_particles) p.d = 0.0;

  CParticleFilterCapable& base = pdf;
  EXPECT_EQ(base.particlesCount(), 3u);
  base.setW(1, -2.5);
  EXPECT_NEAR(base.getW(1), -2.5, 1e-12);

  EXPECT_THROW(base.getW(10), std::exception);
  EXPECT_THROW(base.setW(10, 0.0), std::exception);
}

TEST(BayesTest, normalizeWeights_singleParticle)
{
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(1);
  pdf.m_particles[0].log_w = -3.0;
  pdf.m_particles[0].d = 0.0;

  const double ratio = pdf.normalizeWeights();
  EXPECT_NEAR(ratio, 1.0, 1e-12);
  EXPECT_NEAR(pdf.m_particles[0].log_w, 0.0, 1e-12);
}

TEST(BayesTest, normalizeWeights_empty)
{
  SimpleParticlePDF pdf;
  EXPECT_NEAR(pdf.normalizeWeights(), 0.0, 1e-12);
  EXPECT_NEAR(pdf.ESS(), 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// computeResampling — error path and the residual/random part of Residual.
// ---------------------------------------------------------------------------

TEST(BayesTest, computeResampling_InvalidMethod_Throws)
{
  std::vector<double> logW(5, 0.0);
  std::vector<size_t> idx;
  EXPECT_THROW(
      CParticleFilterCapable::computeResampling(
          static_cast<CParticleFilter::TParticleResamplingAlgorithm>(99), logW, idx),
      std::exception);
}

TEST(BayesTest, computeResampling_Residual_WithRandomPart)
{
  // Skewed weights: most particles get 0 deterministic copies, forcing the
  // random ("residual") part of the algorithm (N_rnd>0) to run.
  const size_t N = 10;
  std::vector<double> logW(N, -100.0);
  logW[0] = 0.0;
  logW[1] = -1.0;
  std::vector<size_t> idx;
  CParticleFilterCapable::computeResampling(
      CParticleFilter::TParticleResamplingAlgorithm::Residual, logW, idx);
  ASSERT_EQ(idx.size(), N);
  for (size_t i : idx) EXPECT_LT(i, N);
}

// ---------------------------------------------------------------------------
// performResampling — regression test for weights being reset to 0 (this was
// previously silently skipped when out_particle_count defaulted to 0, see
// CParticleFilterCapable::performResampling()).
// ---------------------------------------------------------------------------

TEST(BayesTest, performResampling_ResetsWeightsToZero)
{
  mrpt::random::Randomize(7);
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(10);
  for (size_t i = 0; i < pdf.m_particles.size(); i++)
  {
    pdf.m_particles[i].log_w = -static_cast<double>(i);
    pdf.m_particles[i].d = static_cast<double>(i);
  }

  CParticleFilter::TParticleFilterOptions opts;
  opts.resamplingMethod = CParticleFilter::TParticleResamplingAlgorithm::Multinomial;
  pdf.performResampling(opts);  // default out_particle_count=0: "keep the same count"

  ASSERT_EQ(pdf.m_particles.size(), 10u);
  for (const auto& p : pdf.m_particles)
  {
    EXPECT_NEAR(p.log_w, 0.0, 1e-12);
  }
}

// ---------------------------------------------------------------------------
// prediction_and_update — dispatch to the 4 algorithms (all throw in the
// base implementation, since none is overridden), plus an invalid selector.
// ---------------------------------------------------------------------------

TEST(BayesTest, predictionAndUpdate_AllAlgorithms_ThrowByDefault)
{
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(1);
  pdf.m_particles[0].d = 0.0;

  for (auto alg :
       {CParticleFilter::TParticleFilterAlgorithm::StandardProposal,
        CParticleFilter::TParticleFilterAlgorithm::AuxiliaryPFStandard,
        CParticleFilter::TParticleFilterAlgorithm::OptimalProposal,
        CParticleFilter::TParticleFilterAlgorithm::AuxiliaryPFOptimal})
  {
    CParticleFilter::TParticleFilterOptions opts;
    opts.PF_algorithm = alg;
    EXPECT_THROW(pdf.prediction_and_update(nullptr, nullptr, opts), std::exception);
  }
}

TEST(BayesTest, predictionAndUpdate_InvalidAlgorithm_Throws)
{
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(1);
  pdf.m_particles[0].d = 0.0;

  CParticleFilter::TParticleFilterOptions opts;
  opts.PF_algorithm = static_cast<CParticleFilter::TParticleFilterAlgorithm>(99);
  EXPECT_THROW(pdf.prediction_and_update(nullptr, nullptr, opts), std::exception);
}

// ---------------------------------------------------------------------------
// prepareFastDrawSample / fastDrawSample
// ---------------------------------------------------------------------------

TEST(BayesTest, fastDrawSample_staticSampleSize)
{
  mrpt::random::Randomize(3);
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(5);
  for (size_t i = 0; i < 5; i++)
  {
    pdf.m_particles[i].log_w = 0.0;
    pdf.m_particles[i].d = static_cast<double>(i);
  }

  CParticleFilter::TParticleFilterOptions opts;
  opts.adaptiveSampleSize = false;
  pdf.prepareFastDrawSample(opts);

  for (int i = 0; i < 5; i++)
  {
    const size_t idx = pdf.fastDrawSample(opts);
    EXPECT_LT(idx, 5u);
  }
  // Calling once more than prepared must throw:
  EXPECT_THROW(pdf.fastDrawSample(opts), std::exception);
}

TEST(BayesTest, fastDrawSample_adaptiveSampleSize_multinomial)
{
  mrpt::random::Randomize(4);
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(6);
  for (size_t i = 0; i < 6; i++)
  {
    pdf.m_particles[i].log_w = 0.0;
    pdf.m_particles[i].d = static_cast<double>(i);
  }

  CParticleFilter::TParticleFilterOptions opts;
  opts.adaptiveSampleSize = true;
  opts.resamplingMethod = CParticleFilter::TParticleResamplingAlgorithm::Multinomial;
  pdf.prepareFastDrawSample(opts);

  // Dynamic sample size: can be called an arbitrary number of times.
  for (int i = 0; i < 20; i++)
  {
    const size_t idx = pdf.fastDrawSample(opts);
    EXPECT_LT(idx, 6u);
  }
}

TEST(BayesTest, prepareFastDrawSample_adaptiveSampleSize_wrongMethod_Throws)
{
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(3);
  for (auto& p : pdf.m_particles) p.d = 0.0;

  CParticleFilter::TParticleFilterOptions opts;
  opts.adaptiveSampleSize = true;
  opts.resamplingMethod = CParticleFilter::TParticleResamplingAlgorithm::Residual;
  EXPECT_THROW(pdf.prepareFastDrawSample(opts), std::exception);
}

TEST(BayesTest, fastDrawSample_adaptiveSampleSize_wrongMethod_Throws)
{
  mrpt::random::Randomize(5);
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(3);
  for (auto& p : pdf.m_particles) p.d = 0.0;

  CParticleFilter::TParticleFilterOptions prepOpts;
  prepOpts.adaptiveSampleSize = false;
  pdf.prepareFastDrawSample(prepOpts);

  CParticleFilter::TParticleFilterOptions drawOpts;
  drawOpts.adaptiveSampleSize = true;
  drawOpts.resamplingMethod = CParticleFilter::TParticleResamplingAlgorithm::Residual;
  EXPECT_THROW(pdf.fastDrawSample(drawOpts), std::exception);
}

// ---------------------------------------------------------------------------
// log2linearWeights — empty-input edge case.
// ---------------------------------------------------------------------------

TEST(BayesTest, log2linearWeights_empty)
{
  std::vector<double> logW;
  std::vector<double> linW;
  CParticleFilterCapable::log2linearWeights(logW, linW);
  EXPECT_TRUE(linW.empty());

  const auto out2 = CParticleFilterCapable::logWeightsToLinear(logW);
  EXPECT_TRUE(out2.empty());
}

// ---------------------------------------------------------------------------
// CParticleFilter::executeOn — end-to-end, triggering the resampling branch.
// ---------------------------------------------------------------------------

struct ResamplingTestPDF : public SimpleParticlePDF
{
 protected:
  void prediction_and_update_pfStandardProposal(
      const mrpt::obs::CActionCollection*,
      const mrpt::obs::CSensoryFrame*,
      const CParticleFilter::TParticleFilterOptions&) override
  {
    // Concentrate almost all the weight on particle 0: low ESS triggers resampling.
    for (size_t i = 0; i < m_particles.size(); i++)
    {
      m_particles[i].log_w = (i == 0) ? 0.0 : -100.0;
    }
  }
};

TEST(BayesTest, executeOn_triggersResampling)
{
  mrpt::random::Randomize(9);
  ResamplingTestPDF pdf;
  pdf.m_particles.resize(20);
  for (size_t i = 0; i < pdf.m_particles.size(); i++)
  {
    pdf.m_particles[i].log_w = 0.0;
    pdf.m_particles[i].d = static_cast<double>(i);
  }

  CParticleFilter pf;
  pf.m_options.PF_algorithm = CParticleFilter::TParticleFilterAlgorithm::StandardProposal;
  pf.m_options.adaptiveSampleSize = false;
  pf.m_options.BETA = 0.9;  // High threshold: force resampling given the low ESS.

  CParticleFilter::TParticleFilterStats stats;
  pf.executeOn(pdf, nullptr, nullptr, &stats);

  EXPECT_GE(stats.ESS_beforeResample, 0.0);
  EXPECT_LE(stats.ESS_beforeResample, 1.0);
  // After resampling, weights must have been reset:
  for (const auto& p : pdf.m_particles)
  {
    EXPECT_NEAR(p.log_w, 0.0, 1e-12);
  }
}

// ---------------------------------------------------------------------------
// CParticleFilter::TParticleFilterOptions — save/load round-trip.
// ---------------------------------------------------------------------------

TEST(BayesTest, TParticleFilterOptions_SaveLoadRoundTrip_StandardProposal)
{
  CParticleFilter::TParticleFilterOptions opts;
  opts.adaptiveSampleSize = true;
  opts.BETA = 0.3;
  opts.sampleSize = 500;
  opts.powFactor = 2.0;
  opts.PF_algorithm = CParticleFilter::TParticleFilterAlgorithm::StandardProposal;
  opts.resamplingMethod = CParticleFilter::TParticleResamplingAlgorithm::Stratified;
  opts.max_loglikelihood_dyn_range = 20.0;
  opts.pfAuxFilterStandard_FirstStageWeightsMonteCarlo = true;
  opts.pfAuxFilterOptimal_MLE = true;

  mrpt::config::CConfigFileMemory cfg;
  opts.saveToConfigFile(cfg, "PF");

  CParticleFilter::TParticleFilterOptions opts2;
  opts2.loadFromConfigFile(cfg, "PF");

  EXPECT_EQ(opts2.adaptiveSampleSize, opts.adaptiveSampleSize);
  EXPECT_NEAR(opts2.BETA, opts.BETA, 1e-12);
  EXPECT_EQ(opts2.sampleSize, opts.sampleSize);
  EXPECT_NEAR(opts2.powFactor, opts.powFactor, 1e-12);
  EXPECT_EQ(opts2.PF_algorithm, opts.PF_algorithm);
  EXPECT_EQ(opts2.resamplingMethod, opts.resamplingMethod);
  EXPECT_NEAR(opts2.max_loglikelihood_dyn_range, opts.max_loglikelihood_dyn_range, 1e-12);
  EXPECT_EQ(
      opts2.pfAuxFilterStandard_FirstStageWeightsMonteCarlo,
      opts.pfAuxFilterStandard_FirstStageWeightsMonteCarlo);
  EXPECT_EQ(opts2.pfAuxFilterOptimal_MLE, opts.pfAuxFilterOptimal_MLE);
}

TEST(BayesTest, TParticleFilterOptions_SaveLoadRoundTrip_AuxiliaryPFOptimal)
{
  CParticleFilter::TParticleFilterOptions opts;
  opts.PF_algorithm = CParticleFilter::TParticleFilterAlgorithm::AuxiliaryPFOptimal;
  opts.pfAuxFilterOptimal_MaximumSearchSamples = 250;

  mrpt::config::CConfigFileMemory cfg;
  opts.saveToConfigFile(cfg, "PF");

  CParticleFilter::TParticleFilterOptions opts2;
  opts2.loadFromConfigFile(cfg, "PF");

  EXPECT_EQ(opts2.PF_algorithm, CParticleFilter::TParticleFilterAlgorithm::AuxiliaryPFOptimal);
  EXPECT_EQ(opts2.pfAuxFilterOptimal_MaximumSearchSamples, 250u);
}

// ---------------------------------------------------------------------------
// CParticleFilterData<>::writeParticlesToStream() / readParticlesFromStream()
// ---------------------------------------------------------------------------

namespace
{
/** Minimal stand-in for a serialization archive, just enough to exercise
 * CParticleFilterData<>::{write,read}ParticlesToStream<STREAM>(), without
 * pulling in mrpt_serialization (which itself depends on mrpt_bayes'
 * siblings, not the other way around). */
struct FakeStream
{
  std::vector<double> doubles;
  std::vector<uint32_t> uints;
  size_t d_idx = 0;
  size_t u_idx = 0;
};
FakeStream& operator<<(FakeStream& s, double v)
{
  s.doubles.push_back(v);
  return s;
}
FakeStream& operator<<(FakeStream& s, uint32_t v)
{
  s.uints.push_back(v);
  return s;
}
FakeStream& operator>>(FakeStream& s, double& v)
{
  v = s.doubles.at(s.d_idx++);
  return s;
}
FakeStream& operator>>(FakeStream& s, uint32_t& v)
{
  v = s.uints.at(s.u_idx++);
  return s;
}
}  // namespace

TEST(BayesTest, writeReadParticlesToFromStream_ValueStorage)
{
  SimpleParticlePDF pdf;
  pdf.m_particles.resize(3);
  for (size_t i = 0; i < 3; i++)
  {
    pdf.m_particles[i].log_w = -static_cast<double>(i);
    pdf.m_particles[i].d = static_cast<double>(i) * 10.0;
  }

  FakeStream stream;
  pdf.writeParticlesToStream(stream);

  SimpleParticlePDF pdf2;
  pdf2.readParticlesFromStream(stream);

  ASSERT_EQ(pdf2.m_particles.size(), 3u);
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_NEAR(pdf2.m_particles[i].log_w, -static_cast<double>(i), 1e-12);
    EXPECT_NEAR(pdf2.m_particles[i].d, static_cast<double>(i) * 10.0, 1e-12);
  }
}
