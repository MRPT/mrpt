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

// End-to-end checks of CParticleFilter::executeOn() against the exact Kalman
// filter of a linear Gaussian model.

#include <gtest/gtest.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/math/distributions.h>
#include <mrpt/random.h>

#include <cmath>
#include <vector>

using namespace mrpt::bayes;

namespace
{
// 1D model: x_k = x_{k-1} + u + w, w ~ N(0, Q); z_k = x_k + v, v ~ N(0, R).
constexpr double U = 0.5;
constexpr double Q = 0.04;
constexpr double R = 0.09;
const std::vector<double> kMeasurements = {0.7, 1.1, 1.4, 2.2, 2.4, 3.1};

struct Particle1D
{
  double x = 0;
};

class PF1D :
    public CParticleFilterData<Particle1D>,
    public CParticleFilterDataImpl<PF1D, CParticleFilterData<Particle1D>::CParticleList>
{
 public:
  size_t step = 0;

  void init(size_t N, double mean, double var)
  {
    clearParticles();
    m_particles.resize(N);
    auto& rng = mrpt::random::getRandomGenerator();
    for (auto& p : m_particles)
    {
      p.d.reset(new Particle1D());
      p.d->x = mean + std::sqrt(var) * rng.drawGaussian1D_normalized();
      p.log_w = 0;
    }
  }
  void moments(double& mean, double& var) const
  {
    double sw = 0;
    double sx = 0;
    double sxx = 0;
    for (const auto& p : m_particles)
    {
      const double w = std::exp(p.log_w);
      sw += w;
      sx += w * p.d->x;
      sxx += w * p.d->x * p.d->x;
    }
    mean = sx / sw;
    var = sxx / sw - mean * mean;
  }

 protected:
  void prediction_and_update_pfStandardProposal(
      const mrpt::obs::CActionCollection*,
      const mrpt::obs::CSensoryFrame*,
      const CParticleFilter::TParticleFilterOptions&) override
  {
    auto& rng = mrpt::random::getRandomGenerator();
    const double z = kMeasurements.at(step);
    for (auto& p : m_particles)
    {
      p.d->x += U + std::sqrt(Q) * rng.drawGaussian1D_normalized();
      p.log_w += -0.5 * (z - p.d->x) * (z - p.d->x) / R;
    }
  }
};

void kfStep(double& m, double& P, double z)
{
  m += U;
  P += Q;
  const double K = P / (P + R);
  m += K * (z - m);
  P *= (1 - K);
}

void checkAgainstKF(CParticleFilter::TParticleResamplingAlgorithm resampling, double beta)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  const size_t N = 20000;
  PF1D pf;
  pf.init(N, 0.0, 0.25);
  double m = 0.0;
  double P = 0.25;

  CParticleFilter filter;
  filter.m_options.PF_algorithm = CParticleFilter::TParticleFilterAlgorithm::StandardProposal;
  filter.m_options.resamplingMethod = resampling;
  filter.m_options.BETA = beta;

  for (size_t k = 0; k < kMeasurements.size(); k++)
  {
    pf.step = k;
    CParticleFilter::TParticleFilterStats stats;
    filter.executeOn(pf, nullptr, nullptr, &stats);
    kfStep(m, P, kMeasurements[k]);

    EXPECT_GT(stats.ESS_beforeResample, 0.0);
    EXPECT_LE(stats.ESS_beforeResample, 1.0);
    EXPECT_EQ(pf.particlesCount(), N);

    double pm = 0;
    double pv = 0;
    pf.moments(pm, pv);
    // Monte Carlo tolerances, from the effective sample size:
    const double ess = stats.ESS_beforeResample * static_cast<double>(N);
    EXPECT_NEAR(pm, m, 5 * std::sqrt(P / ess)) << "step " << k;
    EXPECT_NEAR(pv, P, 5 * std::sqrt(2.0 / ess) * P) << "step " << k;
  }
}
}  // namespace

TEST(CParticleFilter, executeOnMatchesKalmanFilter_Multinomial)
{
  checkAgainstKF(CParticleFilter::TParticleResamplingAlgorithm::Multinomial, 0.5);
}

TEST(CParticleFilter, executeOnMatchesKalmanFilter_Systematic)
{
  checkAgainstKF(CParticleFilter::TParticleResamplingAlgorithm::Systematic, 0.5);
}

TEST(CParticleFilter, executeOnMatchesKalmanFilter_ResampleAlways)
{
  checkAgainstKF(CParticleFilter::TParticleResamplingAlgorithm::Residual, 1.01);
}

TEST(CParticleFilter, executeOnMatchesKalmanFilter_NeverResample)
{
  // Without resampling, weights degenerate but the estimate stays unbiased
  // over these few steps.
  checkAgainstKF(CParticleFilter::TParticleResamplingAlgorithm::Stratified, 0.0);
}
