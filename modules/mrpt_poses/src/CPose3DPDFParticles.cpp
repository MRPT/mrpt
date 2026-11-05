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

#include <mrpt/math/TPose3D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include <Eigen/Dense>
#include <limits>
#include <ostream>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;

IMPLEMENTS_SERIALIZABLE(CPose3DPDFParticles, CPose3DPDF, mrpt::poses)

CPose3DPDFParticles::CPose3DPDFParticles(size_t M)
{
  m_particles.resize(M);
  TPose3D nullPose(0, 0, 0, 0, 0, 0);
  resetDeterministic(nullPose);
}

void CPose3DPDFParticles::copyFrom(const CPose3DPDF& o)
{
  MRPT_START
  if (this == &o) return;  // It may be used sometimes
  if (o.GetRuntimeClass() == CLASS_ID(CPose3DPDFParticles))
  {
    const auto* pdf = dynamic_cast<const CPose3DPDFParticles*>(&o);
    ASSERT_(pdf);
    m_particles = pdf->m_particles;
  }
  else
  {
    // Any other PDF: draw samples from it.
    const size_t N = m_particles.empty() ? 1000 : m_particles.size();
    m_particles.resize(N);

    std::vector<CVectorDouble> samples;
    o.drawManySamples(N, samples);
    ASSERT_EQUAL_(samples.size(), N);

    const double uniformWeight = -std::log(static_cast<double>(N));
    for (size_t i = 0; i < N; i++)
    {
      m_particles[i].log_w = uniformWeight;
      m_particles[i].d = TPose3D(
          samples[i][0], samples[i][1], samples[i][2], samples[i][3], samples[i][4], samples[i][5]);
    }
  }
  MRPT_END
}

void CPose3DPDFParticles::getMean(CPose3D& p) const
{
  MRPT_START
  // Default to (0,..,0)
  p = CPose3D();
  if (m_particles.empty())
  {
    return;
  }
  // Shift log-weights by their maximum to avoid underflow:
  double maxW = m_particles[0].log_w;
  for (const auto& part : m_particles)
  {
    maxW = std::max(maxW, part.log_w);
  }
  // Calc average on SE(3)
  mrpt::poses::SE_average<3> se_averager;
  for (const auto& part : m_particles)
  {
    const double w = exp(part.log_w - maxW);
    se_averager.append(part.d, w);
  }
  se_averager.get_average(p);
  MRPT_END
}

std::tuple<CMatrixDouble66, CPose3D> CPose3DPDFParticles::getCovarianceAndMean() const
{
  MRPT_START

  CMatrixDouble66 cov;
  CPose3D mean;

  getMean(mean);  // First! the mean value:

  // Now the covariance:
  cov.setZero();
  CVectorFixedDouble<6> vars;
  vars.setZero();  // The diagonal of the final covariance matrix

  // Elements off the diagonal of the covariance matrix:
  double std_xy = 0, std_xz = 0, std_xya = 0, std_xp = 0, std_xr = 0;
  double std_yz = 0, std_yya = 0, std_yp = 0, std_yr = 0;
  double std_zya = 0, std_zp = 0, std_zr = 0;
  double std_yap = 0, std_yar = 0;
  double std_pr = 0;

  // enough information to estimate the covariance?
  if (m_particles.size() < 2) return {cov, mean};

  // Sum all weight values (shifted by their maximum to avoid underflow):
  double maxW = m_particles[0].log_w;
  for (const auto& p : m_particles)
  {
    maxW = std::max(maxW, p.log_w);
  }
  double W = 0;
  for (const auto& p : m_particles)
  {
    W += exp(p.log_w - maxW);
  }

  ASSERT_(W > 0);

  // Compute covariance:
  for (const auto& p : m_particles)
  {
    double w = exp(p.log_w - maxW) / W;

    // Signed angular errors, in the range [-pi,pi]:
    double err_yaw = wrapToPi(p.d.yaw - mean.yaw());
    double err_pitch = wrapToPi(p.d.pitch - mean.pitch());
    double err_roll = wrapToPi(p.d.roll - mean.roll());

    double err_x = p.d.x - mean.x();
    double err_y = p.d.y - mean.y();
    double err_z = p.d.z - mean.z();

    vars[0] += square(err_x) * w;
    vars[1] += square(err_y) * w;
    vars[2] += square(err_z) * w;
    vars[3] += square(err_yaw) * w;
    vars[4] += square(err_pitch) * w;
    vars[5] += square(err_roll) * w;

    std_xy += err_x * err_y * w;
    std_xz += err_x * err_z * w;
    std_xya += err_x * err_yaw * w;
    std_xp += err_x * err_pitch * w;
    std_xr += err_x * err_roll * w;

    std_yz += err_y * err_z * w;
    std_yya += err_y * err_yaw * w;
    std_yp += err_y * err_pitch * w;
    std_yr += err_y * err_roll * w;

    std_zya += err_z * err_yaw * w;
    std_zp += err_z * err_pitch * w;
    std_zr += err_z * err_roll * w;

    std_yap += err_yaw * err_pitch * w;
    std_yar += err_yaw * err_roll * w;

    std_pr += err_pitch * err_roll * w;
  }  // end for it

  // Weighted (biased) estimate of the covariance:
  cov(0, 0) = vars[0];
  cov(1, 1) = vars[1];
  cov(2, 2) = vars[2];
  cov(3, 3) = vars[3];
  cov(4, 4) = vars[4];
  cov(5, 5) = vars[5];

  cov(1, 0) = cov(0, 1) = std_xy;
  cov(2, 0) = cov(0, 2) = std_xz;
  cov(3, 0) = cov(0, 3) = std_xya;
  cov(4, 0) = cov(0, 4) = std_xp;
  cov(5, 0) = cov(0, 5) = std_xr;

  cov(2, 1) = cov(1, 2) = std_yz;
  cov(3, 1) = cov(1, 3) = std_yya;
  cov(4, 1) = cov(1, 4) = std_yp;
  cov(5, 1) = cov(1, 5) = std_yr;

  cov(3, 2) = cov(2, 3) = std_zya;
  cov(4, 2) = cov(2, 4) = std_zp;
  cov(5, 2) = cov(2, 5) = std_zr;

  cov(4, 3) = cov(3, 4) = std_yap;
  cov(5, 3) = cov(3, 5) = std_yar;

  cov(5, 4) = cov(4, 5) = std_pr;

  return {cov, mean};
  MRPT_END
}

uint8_t CPose3DPDFParticles::serializeGetVersion() const { return 1; }
void CPose3DPDFParticles::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeParticlesToStream(out);  // v1: CPose3D -> TPose3D
}
void CPose3DPDFParticles::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      mrpt::bayes::CParticleFilterData<mrpt::poses::CPose3D, PARTICLE_STORAGE> old;
      old.readParticlesFromStream(in);
      m_particles.clear();
      std::transform(
          old.m_particles.begin(), old.m_particles.end(), std::back_inserter(m_particles),
          [](const auto& p) -> CParticleData { return CParticleData(p.d.asTPose(), p.log_w); });
    }
    break;
    case 1:
    {
      readParticlesFromStream(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

bool CPose3DPDFParticles::saveToTextFile(const std::string& file) const
{
  using namespace mrpt::system;

  FILE* f = os::fopen(file.c_str(), "wt");
  if (!f)
  {
    return false;
  }

  os::fprintf(f, "%% x  y  z  yaw[rad] pitch[rad] roll[rad] log_weight\n");

  for (const auto& p : m_particles)
    os::fprintf(
        f, "%f %f %f %f %f %f %e\n", p.d.x, p.d.y, p.d.z, p.d.yaw, p.d.pitch, p.d.roll, p.log_w);

  os::fclose(f);
  return true;
}

mrpt::math::TPose3D CPose3DPDFParticles::getParticlePose(int i) const { return m_particles[i].d; }

void CPose3DPDFParticles::changeCoordinatesReference(const CPose3D& newReferenceBase)
{
  for (auto& p : m_particles) p.d = (newReferenceBase + CPose3D(p.d)).asTPose();
}

void CPose3DPDFParticles::drawSingleSample(CPose3D& outPart) const
{
  ASSERT_(!m_particles.empty());

  // Weights do not need to sum 1 (e.g. normalizeWeights() sets max log_w=0):
  double maxW = m_particles[0].log_w;
  for (const auto& p : m_particles)
  {
    maxW = std::max(maxW, p.log_w);
  }
  double sumW = 0;
  for (const auto& p : m_particles)
  {
    sumW += exp(p.log_w - maxW);
  }

  const double uni = getRandomGenerator().drawUniform(0.0, sumW);
  double cum = 0;
  for (const auto& p : m_particles)
  {
    cum += exp(p.log_w - maxW);
    if (uni <= cum)
    {
      outPart = CPose3D(p.d);
      return;
    }
  }
  // Fallback:
  outPart = CPose3D(m_particles.rbegin()->d);
}

void CPose3DPDFParticles::drawManySamples(size_t N, std::vector<CVectorDouble>& outSamples) const
{
  outSamples.resize(N);
  for (size_t i = 0; i < N; i++)
  {
    CPose3D pose;
    drawSingleSample(pose);
    outSamples[i].resize(6);
    outSamples[i][0] = pose.x();
    outSamples[i][1] = pose.y();
    outSamples[i][2] = pose.z();
    outSamples[i][3] = pose.yaw();
    outSamples[i][4] = pose.pitch();
    outSamples[i][5] = pose.roll();
  }
}

void CPose3DPDFParticles::operator+=(const CPose3D& Ap)
{
  for (auto& p : m_particles) p.d = (CPose3D(p.d) + Ap).asTPose();
}

void CPose3DPDFParticles::append(CPose3DPDFParticles& o)
{
  m_particles.insert(m_particles.end(), o.m_particles.begin(), o.m_particles.end());
}

void CPose3DPDFParticles::inverse(CPose3DPDF& o) const
{
  MRPT_START
  ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPose3DPDFParticles));
  auto* out = dynamic_cast<CPose3DPDFParticles*>(&o);
  ASSERT_(out != nullptr);
  // Prepare the output:
  out->copyFrom(*this);
  const CPose3D zero(0, 0, 0);
  for (auto& p : out->m_particles) p.d = (zero - CPose3D(p.d)).asTPose();
  MRPT_END
}

TPose3D CPose3DPDFParticles::getMostLikelyParticle() const
{
  mrpt::math::TPose3D ret{0, 0, 0, 0, 0, 0};
  double max_w = -std::numeric_limits<double>::max();
  for (const auto& p : m_particles)
  {
    if (p.log_w > max_w)
    {
      ret = p.d;
      max_w = p.log_w;
    }
  }
  return ret;
}

void CPose3DPDFParticles::bayesianFusion(const CPose3DPDF& p1, const CPose3DPDF& p2)
{
  MRPT_START

  // Start with particles from p1
  copyFrom(p1);

  const auto [cov2, mean2] = p2.getCovarianceAndMean();

  if (const auto* p2p = dynamic_cast<const CPose3DPDFParticles*>(&p2))
  {
    // Kernel density estimate of p2, with Gaussian kernels shaped as its
    // covariance and scaled with Silverman's rule of thumb:
    // H = h^2 * cov, h = (4 / ((d + 2) * N))^(1 / (d + 4)), d = 6.
    const double N2 = static_cast<double>(std::max<size_t>(p2p->m_particles.size(), 1));
    CMatrixDouble66 H = cov2;
    H *= std::pow(4.0 / (8.0 * N2), 2.0 / 10.0);
    for (int k = 0; k < 6; k++)
    {
      H(k, k) = H(k, k) * (1 + 1e-9) + 1e-12;
    }
    const CMatrixDouble66 Hinv = H.inverse_LLt();
    for (auto& p : m_particles)
    {
      // log(sum_j w_j * exp(-0.5 * d_j^T * Hinv * d_j)), without underflow:
      double maxTerm = -std::numeric_limits<double>::infinity();
      double sum = 0;
      for (const auto& q : p2p->m_particles)
      {
        const double d[6] = {
            p.d.x - q.d.x,
            p.d.y - q.d.y,
            p.d.z - q.d.z,
            wrapToPi(p.d.yaw - q.d.yaw),
            wrapToPi(p.d.pitch - q.d.pitch),
            wrapToPi(p.d.roll - q.d.roll)};
        double maha2 = 0;
        for (int r = 0; r < 6; r++)
        {
          for (int c = 0; c < 6; c++)
          {
            maha2 += d[r] * Hinv(r, c) * d[c];
          }
        }
        const double term = q.log_w - 0.5 * maha2;
        if (term > maxTerm)
        {
          sum = sum * std::exp(maxTerm - term) + 1.0;
          maxTerm = term;
        }
        else
        {
          sum += std::exp(term - maxTerm);
        }
      }
      p.log_w += maxTerm + std::log(sum);
    }
  }
  else
  {
    // Generic: use Gaussian approximation of p2 (Mahalanobis-based likelihood)
    const CMatrixDouble66 cov2inv = cov2.inverse_LLt();
    for (auto& p : m_particles)
    {
      CVectorFixedDouble<6> diff;
      diff[0] = p.d.x - mean2.x();
      diff[1] = p.d.y - mean2.y();
      diff[2] = p.d.z - mean2.z();
      diff[3] = wrapToPi(p.d.yaw - mean2.yaw());
      diff[4] = wrapToPi(p.d.pitch - mean2.pitch());
      diff[5] = wrapToPi(p.d.roll - mean2.roll());
      double maha2 = 0;
      for (int ii = 0; ii < 6; ii++)
        for (int jj = 0; jj < 6; jj++) maha2 += diff[ii] * cov2inv(ii, jj) * diff[jj];
      p.log_w += -0.5 * maha2;
    }
  }
  (void)normalizeWeights();

  MRPT_END
}

void CPose3DPDFParticles::resetDeterministic(const TPose3D& location, size_t particlesCount)
{
  if (particlesCount > 0) m_particles.resize(particlesCount);

  for (auto& p : m_particles)
  {
    p.d = location;
    p.log_w = 0;
  }
}

void CPose3DPDFParticles::resetUniform(
    const mrpt::math::TPose3D& cmin, const mrpt::math::TPose3D& cmax, const int particlesCount)
{
  MRPT_START
  if (particlesCount > 0) m_particles.resize(particlesCount);

  auto& rnd = mrpt::random::getRandomGenerator();

  for (auto& p : m_particles)
  {
    p.d.x = rnd.drawUniform(cmin.x, cmax.x);
    p.d.y = rnd.drawUniform(cmin.y, cmax.y);
    p.d.z = rnd.drawUniform(cmin.z, cmax.z);
    p.d.yaw = rnd.drawUniform(cmin.yaw, cmax.yaw);
    p.d.pitch = rnd.drawUniform(cmin.pitch, cmax.pitch);
    p.d.roll = rnd.drawUniform(cmin.roll, cmax.roll);
    p.log_w = 0;
  }
  MRPT_END
}

void CPose3DPDFParticles::printTo(std::ostream& out) const
{
  const auto [c, m] = getCovarianceAndMean();
  out << "mrpt::poses::CPose3DPDFParticles object with " << size()
      << " particles, mean=" << m.asString() << " cov=" << c.inMatlabFormat() << " ESS=" << ESS();
}
