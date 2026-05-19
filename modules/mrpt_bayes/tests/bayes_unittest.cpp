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

  [[maybe_unused]] auto ratio = pdf.normalizeWeights();

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
