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

// Statistical checks of the particle filter resampling schemes and helpers.

#include <gtest/gtest.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/bayes/CRejectionSamplingCapable.h>
#include <mrpt/random.h>

#include <cmath>
#include <numeric>
#include <vector>

using namespace mrpt::bayes;
using Algo = CParticleFilter::TParticleResamplingAlgorithm;

namespace
{
struct ValuePDF :
    public CParticleFilterData<double, particle_storage_mode::VALUE>,
    public CParticleFilterDataImpl<
        ValuePDF,
        CParticleFilterData<double, particle_storage_mode::VALUE>::CParticleList>
{
};

struct PointerPDF :
    public CParticleFilterData<std::vector<double>, particle_storage_mode::POINTER>,
    public CParticleFilterDataImpl<
        PointerPDF,
        CParticleFilterData<std::vector<double>, particle_storage_mode::POINTER>::CParticleList>
{
};

// Non-uniform test weights (normalized).
std::vector<double> testWeights() { return {0.30, 0.02, 0.15, 0.08, 0.20, 0.05, 0.12, 0.08}; }

std::vector<double> toLog(const std::vector<double>& w, double offset)
{
  std::vector<double> lw(w.size());
  for (size_t i = 0; i < w.size(); i++)
  {
    lw[i] = std::log(w[i]) + offset;
  }
  return lw;
}

// Average number of copies of each particle over `trials` resamplings.
std::vector<double> meanCopies(
    Algo method, size_t outCount, size_t trials, std::vector<int>* minC, std::vector<int>* maxC)
{
  const auto w = testWeights();
  const auto lw = toLog(w, -3.0);
  const size_t M = w.size();
  std::vector<double> acc(M, 0.0);
  if (minC)
  {
    minC->assign(M, 1 << 30);
  }
  if (maxC)
  {
    maxC->assign(M, -1);
  }
  std::vector<size_t> idx;
  for (size_t t = 0; t < trials; t++)
  {
    CParticleFilterCapable::computeResampling(method, lw, idx, outCount);
    std::vector<int> cnt(M, 0);
    for (size_t i : idx)
    {
      cnt.at(i)++;
    }
    for (size_t i = 0; i < M; i++)
    {
      acc[i] += cnt[i];
      if (minC)
      {
        (*minC)[i] = std::min((*minC)[i], cnt[i]);
      }
      if (maxC)
      {
        (*maxC)[i] = std::max((*maxC)[i], cnt[i]);
      }
    }
  }
  for (auto& a : acc)
  {
    a /= static_cast<double>(trials);
  }
  return acc;
}

void checkUnbiased(Algo method, size_t outCount)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const auto w = testWeights();
  const size_t M = w.size();
  const size_t N = outCount == 0 ? M : outCount;
  const size_t trials = 20000;
  const auto mc = meanCopies(method, outCount, trials, nullptr, nullptr);
  for (size_t i = 0; i < M; i++)
  {
    // Per-trial std is at most sqrt(N/4): allow 8 sigma.
    const double tol = 8.0 * std::sqrt(static_cast<double>(N) / 4.0 / trials);
    EXPECT_NEAR(mc[i], static_cast<double>(N) * w[i], tol)
        << "method=" << static_cast<int>(method) << " particle=" << i << " N=" << N;
  }
}

class SimpleRS : public CRejectionSamplingCapable<double, particle_storage_mode::VALUE>
{
 protected:
  void RS_drawFromProposal(double& x) override
  {
    x = mrpt::random::getRandomGenerator().drawUniform(0.0, 1.0);
  }
  // Target density is proportional to x on [0,1].
  double RS_observationLikelihood(const double& x) override { return x; }
};

}  // namespace

TEST(ParticleResamplingStats, MultinomialIsUnbiased) { checkUnbiased(Algo::Multinomial, 0); }

// The random part of residual resampling must be unbiased too.
TEST(ParticleResamplingStats, ResidualIsUnbiased) { checkUnbiased(Algo::Residual, 0); }

TEST(ParticleResamplingStats, StratifiedIsUnbiased) { checkUnbiased(Algo::Stratified, 0); }

TEST(ParticleResamplingStats, SystematicIsUnbiased) { checkUnbiased(Algo::Systematic, 0); }

TEST(ParticleResamplingStats, SystematicCopiesAreFloorOrCeil)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const auto w = testWeights();
  std::vector<int> minC;
  std::vector<int> maxC;
  (void)meanCopies(Algo::Systematic, 0, 2000, &minC, &maxC);
  for (size_t i = 0; i < w.size(); i++)
  {
    const double Nw = static_cast<double>(w.size()) * w[i];
    EXPECT_GE(minC[i], static_cast<int>(std::floor(Nw))) << i;
    EXPECT_LE(maxC[i], static_cast<int>(std::ceil(Nw))) << i;
  }
}

TEST(ParticleResamplingStats, ResidualDeterministicPartIsKept)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const auto w = testWeights();
  std::vector<int> minC;
  (void)meanCopies(Algo::Residual, 0, 2000, &minC, nullptr);
  for (size_t i = 0; i < w.size(); i++)
  {
    EXPECT_GE(minC[i], static_cast<int>(std::floor(static_cast<double>(w.size()) * w[i]))) << i;
  }
}

// Asking for a different number of output particles must not bias any scheme.
TEST(ParticleResamplingStats, DifferentOutputCountIsUnbiased)
{
  for (const auto m : {Algo::Multinomial, Algo::Residual, Algo::Stratified, Algo::Systematic})
  {
    checkUnbiased(m, 4);
    checkUnbiased(m, 13);
  }
}

TEST(ParticleResamplingStats, OutputSizeMatchesRequest)
{
  for (const auto m : {Algo::Multinomial, Algo::Residual, Algo::Stratified, Algo::Systematic})
  {
    std::vector<size_t> idx;
    CParticleFilterCapable::computeResampling(m, toLog(testWeights(), 0.0), idx, 5);
    EXPECT_EQ(idx.size(), 5u);
  }
}

TEST(ParticleResamplingStats, InvariantToLogWeightOffset)
{
  // Same seed and shifted log-weights must give identical indices.
  for (const auto m : {Algo::Multinomial, Algo::Residual, Algo::Stratified, Algo::Systematic})
  {
    std::vector<size_t> a;
    std::vector<size_t> b;
    mrpt::random::getRandomGenerator().randomize(77);
    CParticleFilterCapable::computeResampling(m, toLog(testWeights(), 0.0), a);
    mrpt::random::getRandomGenerator().randomize(77);
    CParticleFilterCapable::computeResampling(m, toLog(testWeights(), -5000.0), b);
    EXPECT_EQ(a, b);
  }
}

// Very small or very large log-weights must not underflow or overflow.
TEST(ParticleResamplingStats, Log2LinearWeightsIsShiftInvariant)
{
  const auto w = testWeights();
  for (const double offset : {0.0, -1000.0, 800.0})
  {
    std::vector<double> lin;
    EXPECT_NO_THROW(CParticleFilterCapable::log2linearWeights(toLog(w, offset), lin))
        << "offset=" << offset;
    ASSERT_EQ(lin.size(), w.size());
    for (size_t i = 0; i < w.size(); i++)
    {
      EXPECT_NEAR(lin[i], w[i], 1e-12) << "offset=" << offset;
    }
  }
}

TEST(ParticleResamplingStats, ESSMatchesDefinition)
{
  const auto w = testWeights();
  double sum2 = 0;
  for (double wi : w)
  {
    sum2 += wi * wi;
  }
  const double essRef = 1.0 / (static_cast<double>(w.size()) * sum2);
  for (const double offset : {0.0, -2000.0, 700.0})
  {
    ValuePDF pdf;
    pdf.m_particles.resize(w.size());
    const auto lw = toLog(w, offset);
    for (size_t i = 0; i < w.size(); i++)
    {
      pdf.m_particles[i].log_w = lw[i];
    }
    EXPECT_NEAR(pdf.ESS(), essRef, 1e-12) << "offset=" << offset;
  }
}

TEST(ParticleResamplingStats, NormalizeWeightsReturnsDynamicRange)
{
  ValuePDF pdf;
  pdf.m_particles.resize(3);
  pdf.m_particles[0].log_w = -2.0;
  pdf.m_particles[1].log_w = 1.5;
  pdf.m_particles[2].log_w = -0.5;
  double maxLw = 0;
  const double ratio = pdf.normalizeWeights(&maxLw);
  EXPECT_NEAR(maxLw, 1.5, 1e-15);
  EXPECT_NEAR(ratio, std::exp(3.5), 1e-9);
  EXPECT_NEAR(pdf.m_particles[0].log_w, -3.5, 1e-15);
  EXPECT_NEAR(pdf.m_particles[1].log_w, 0.0, 1e-15);
}

// The CDF look-up bins of the adaptive draw must not bias the selection.
TEST(ParticleResamplingStats, AdaptiveFastDrawSampleMatchesWeights)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const auto w = testWeights();
  ValuePDF pdf;
  pdf.m_particles.resize(w.size());
  const auto lw = toLog(w, 0.0);
  for (size_t i = 0; i < w.size(); i++)
  {
    pdf.m_particles[i].log_w = lw[i];
    pdf.m_particles[i].d = static_cast<double>(i);
  }
  CParticleFilter::TParticleFilterOptions opts;
  opts.adaptiveSampleSize = true;
  opts.resamplingMethod = Algo::Multinomial;
  pdf.prepareFastDrawSample(opts);
  const size_t nDraws = 200000;
  std::vector<double> freq(w.size(), 0.0);
  for (size_t k = 0; k < nDraws; k++)
  {
    const size_t i = pdf.fastDrawSample(opts);
    ASSERT_LT(i, w.size());
    freq[i] += 1.0;
  }
  for (size_t i = 0; i < w.size(); i++)
  {
    // Binomial std < 0.0012 for these sizes.
    EXPECT_NEAR(freq[i] / nDraws, w[i], 0.006) << "particle " << i;
  }
}

TEST(ParticleResamplingStats, StaticFastDrawSampleIsUnbiased)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const auto w = testWeights();
  ValuePDF pdf;
  pdf.m_particles.resize(w.size());
  const auto lw = toLog(w, 0.0);
  for (size_t i = 0; i < w.size(); i++)
  {
    pdf.m_particles[i].log_w = lw[i];
  }
  CParticleFilter::TParticleFilterOptions opts;
  opts.adaptiveSampleSize = false;
  opts.resamplingMethod = Algo::Systematic;
  std::vector<double> acc(w.size(), 0.0);
  const size_t trials = 20000;
  for (size_t t = 0; t < trials; t++)
  {
    pdf.prepareFastDrawSample(opts);
    for (size_t k = 0; k < w.size(); k++)
    {
      acc[pdf.fastDrawSample(opts)] += 1.0;
    }
  }
  for (size_t i = 0; i < w.size(); i++)
  {
    EXPECT_NEAR(acc[i] / trials, static_cast<double>(w.size()) * w[i], 0.06) << i;
  }
}

TEST(ParticleResamplingStats, PointerSubstitutionMakesIndependentCopies)
{
  PointerPDF pdf;
  pdf.m_particles.resize(3);
  for (size_t i = 0; i < 3; i++)
  {
    pdf.m_particles[i].d.reset(new std::vector<double>(1, static_cast<double>(i)));
    pdf.m_particles[i].log_w = -static_cast<double>(i);
  }
  pdf.performSubstitution({2, 0, 2, 2});
  ASSERT_EQ(pdf.m_particles.size(), 4u);
  // Sorted order: 0,2,2,2
  EXPECT_EQ((*pdf.m_particles[0].d)[0], 0.0);
  EXPECT_EQ((*pdf.m_particles[1].d)[0], 2.0);
  EXPECT_EQ((*pdf.m_particles[3].d)[0], 2.0);
  EXPECT_NEAR(pdf.m_particles[2].log_w, -2.0, 1e-15);
  (*pdf.m_particles[1].d)[0] = 42.0;
  EXPECT_EQ((*pdf.m_particles[2].d)[0], 2.0);
  EXPECT_EQ((*pdf.m_particles[3].d)[0], 2.0);
}

TEST(ParticleResamplingStats, RejectionSamplingFollowsTarget)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  SimpleRS rs;
  const auto samples = rs.rejectionSampling(20000, 100000);
  double mean = 0;
  double mean2 = 0;
  for (const auto& p : samples)
  {
    EXPECT_EQ(p.log_w, 0.0);
    mean += p.d;
    mean2 += p.d * p.d;
  }
  mean /= static_cast<double>(samples.size());
  mean2 /= static_cast<double>(samples.size());
  // Density 2x on [0,1]: E[x]=2/3, E[x^2]=1/2.
  EXPECT_NEAR(mean, 2.0 / 3.0, 0.01);
  EXPECT_NEAR(mean2, 0.5, 0.01);
}
