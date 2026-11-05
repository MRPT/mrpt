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

#include <mrpt/math/distributions.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include <fstream>
#include <limits>
#include <ostream>

using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPosePDFParticles, CPosePDF, mrpt::poses)

CPosePDFParticles::CPosePDFParticles(size_t M)
{
  m_particles.resize(M);
  for (auto& p : m_particles)
  {
    p.log_w = .0;
    p.d = TPose2D();
  }
  TPose2D nullPose(0, 0, 0);
  resetDeterministic(nullPose);
}

void CPosePDFParticles::copyFrom(const CPosePDF& o)
{
  MRPT_START

  CParticleList::iterator itDest;
  CParticleList::const_iterator itSrc;

  if (this == &o) return;  // It may be used sometimes

  if (o.GetRuntimeClass() == CLASS_ID(CPosePDFParticles))
  {
    const auto* pdf = dynamic_cast<const CPosePDFParticles*>(&o);
    ASSERT_(pdf);

    // Both are m_particles:
    m_particles = pdf->m_particles;
  }
  else if (o.GetRuntimeClass() == CLASS_ID(CPosePDFGaussian))
  {
    const auto* pdf = dynamic_cast<const CPosePDFGaussian*>(&o);
    size_t M = m_particles.size();
    std::vector<CVectorDouble> parts;
    std::vector<CVectorDouble>::iterator partsIt;

    getRandomGenerator().drawGaussianMultivariateMany(parts, M, pdf->cov);

    clearParticles();
    m_particles.resize(M);

    for (itDest = m_particles.begin(), partsIt = parts.begin(); itDest != m_particles.end();
         ++itDest, ++partsIt)
    {
      itDest->log_w = 0;
      itDest->d = TPose2D(
          pdf->mean.x() + (*partsIt)[0], (pdf->mean.y() + (*partsIt)[1]),
          (pdf->mean.phi() + (*partsIt)[2]));
      itDest->d.normalizePhi();
    }
  }
  else
  {
    // Any other PDF: draw samples from it.
    const size_t M = m_particles.size();
    std::vector<CVectorDouble> parts;
    o.drawManySamples(M, parts);
    ASSERT_EQUAL_(parts.size(), M);
    for (size_t i = 0; i < M; i++)
    {
      m_particles[i].log_w = 0;
      m_particles[i].d = TPose2D(parts[i][0], parts[i][1], parts[i][2]);
      m_particles[i].d.normalizePhi();
    }
  }

  MRPT_END
}

void CPosePDFParticles::clear() { clearParticles(); }
void CPosePDFParticles::getMean(CPose2D& est_) const
{
  // Calc average on SE(2)
  const size_t n = m_particles.size();
  if (n)
  {
    // Shift log-weights by their maximum to avoid underflow:
    double maxW = m_particles[0].log_w;
    for (const auto& p : m_particles)
    {
      maxW = std::max(maxW, p.log_w);
    }
    mrpt::poses::SE_average<2> se_averager;
    for (size_t i = 0; i < n; i++)
    {
      double w = exp(m_particles[i].log_w - maxW);
      se_averager.append(m_particles[i].d, w);
    }
    se_averager.get_average(est_);
  }
  else
  {
    est_ = CPose2D();
  }
}

std::tuple<CMatrixDouble33, CPose2D> CPosePDFParticles::getCovarianceAndMean() const
{
  CMatrixDouble33 cov;
  CPose2D mean;
  cov.setZero();
  getMean(mean);

  size_t i, n = m_particles.size();
  double var_x = 0, var_y = 0, var_p = 0, var_xy = 0, var_xp = 0, var_yp = 0;

  // Shift log-weights by their maximum to avoid underflow:
  double maxW = n ? m_particles[0].log_w : 0.0;
  for (i = 0; i < n; i++)
  {
    maxW = std::max(maxW, m_particles[i].log_w);
  }

  double lin_w_sum = 0;

  for (i = 0; i < n; i++)
  {
    lin_w_sum += exp(m_particles[i].log_w - maxW);
  }
  if (lin_w_sum == 0) lin_w_sum = 1;

  for (i = 0; i < n; i++)
  {
    double w = exp(m_particles[i].log_w - maxW) / lin_w_sum;

    double err_x = m_particles[i].d.x - mean.x();
    double err_y = m_particles[i].d.y - mean.y();
    // Signed heading error, in the range [-pi,pi]:
    double err_phi = math::wrapToPi(m_particles[i].d.phi - mean.phi());

    var_x += square(err_x) * w;
    var_y += square(err_y) * w;
    var_p += square(err_phi) * w;
    var_xy += err_x * err_y * w;
    var_xp += err_x * err_phi * w;
    var_yp += err_y * err_phi * w;
  }

  if (n < 2)
  {
    // Not enough information to estimate the variance:
  }
  else
  {
    // Weighted (biased) estimate of the covariance:
    cov(0, 0) = var_x;
    cov(1, 1) = var_y;
    cov(2, 2) = var_p;

    cov(1, 0) = cov(0, 1) = var_xy;
    cov(2, 0) = cov(0, 2) = var_xp;
    cov(1, 2) = cov(2, 1) = var_yp;
  }
  return {cov, mean};
}

uint8_t CPosePDFParticles::serializeGetVersion() const { return 1; }
void CPosePDFParticles::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeParticlesToStream(out);  // v1: changed CPose2D -> TPose2D
}
void CPosePDFParticles::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      mrpt::bayes::CParticleFilterData<mrpt::poses::CPose2D, PARTICLE_STORAGE> old;
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

void CPosePDFParticles::resetDeterministic(const TPose2D& location, size_t particlesCount)
{
  if (particlesCount > 0) m_particles.resize(particlesCount);

  for (auto& p : m_particles)
  {
    p.d = location;
    p.log_w = .0;
  }
}

void CPosePDFParticles::resetUniform(
    const double x_min,
    const double x_max,
    const double y_min,
    const double y_max,
    const double phi_min,
    const double phi_max,
    const int particlesCount)
{
  MRPT_START
  if (particlesCount > 0) m_particles.resize(particlesCount);

  for (auto& p : m_particles)
  {
    p.d.x = getRandomGenerator().drawUniform(x_min, x_max);
    p.d.y = getRandomGenerator().drawUniform(y_min, y_max);
    p.d.phi = getRandomGenerator().drawUniform(phi_min, phi_max);
    p.log_w = 0;
  }
  MRPT_END
}

void CPosePDFParticles::resetAroundSetOfPoses(
    const std::vector<mrpt::math::TPose2D>& list_poses,
    const size_t num_particles_per_pose,
    const double spread_x,
    const double spread_y,
    const double spread_phi_rad)
{
  MRPT_START
  ASSERT_(!list_poses.empty());
  ASSERT_(num_particles_per_pose >= 1);

  const size_t N = list_poses.size() * num_particles_per_pose;

  clear();
  m_particles.resize(N);
  size_t i, nSpot;
  for (i = 0, nSpot = 0; nSpot < list_poses.size(); nSpot++)
  {
    const mrpt::math::TPose2D& p = list_poses[nSpot];
    for (size_t k = 0; k < num_particles_per_pose; k++, i++)
    {
      m_particles[i].d.x =
          getRandomGenerator().drawUniform(p.x - spread_x * 0.5, p.x + spread_x * 0.5);
      m_particles[i].d.y =
          getRandomGenerator().drawUniform(p.y - spread_y * 0.5, p.y + spread_y * 0.5);
      m_particles[i].d.phi = getRandomGenerator().drawUniform(
          p.phi - spread_phi_rad * 0.5, p.phi + spread_phi_rad * 0.5);
      m_particles[i].log_w = 0;
    }
  }
  ASSERT_EQUAL_(i, N);
  MRPT_END
}

bool CPosePDFParticles::saveToTextFile(const std::string& file) const
{
  using namespace mrpt::system;

  FILE* f = os::fopen(file.c_str(), "wt");
  if (!f)
  {
    return false;
  }

  os::fprintf(f, "%% x  y  yaw[rad] log_weight\n");

  for (const auto& p : m_particles) os::fprintf(f, "%f %f %f %e\n", p.d.x, p.d.y, p.d.phi, p.log_w);

  os::fclose(f);
  return true;
}

TPose2D CPosePDFParticles::getParticlePose(size_t i) const { return m_particles[i].d; }

void CPosePDFParticles::changeCoordinatesReference(const CPose3D& newReferenceBase_)
{
  const TPose2D newReferenceBase = CPose2D(newReferenceBase_).asTPose();
  for (auto& p : m_particles) p.d = newReferenceBase + p.d;
}

void CPosePDFParticles::drawSingleSample(CPose2D& outPart) const
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

  for (auto& p : m_particles)
  {
    cum += exp(p.log_w - maxW);
    if (uni <= cum)
    {
      outPart = CPose2D(p.d);
      return;
    }
  }

  // Might not come here normally:
  outPart = CPose2D(m_particles.rbegin()->d);
}

void CPosePDFParticles::operator+=(const TPose2D& Ap)
{
  for (auto& p : m_particles) p.d = p.d + Ap;
}

void CPosePDFParticles::append(CPosePDFParticles& o)
{
  for (auto& p : o.m_particles) m_particles.emplace_back(p);
  (void)normalizeWeights();
}

void CPosePDFParticles::inverse(CPosePDF& o) const
{
  MRPT_START
  ASSERT_(o.GetRuntimeClass() == CLASS_ID(CPosePDFParticles));
  auto* out = dynamic_cast<CPosePDFParticles*>(&o);

  out->copyFrom(*this);
  TPose2D nullPose(0, 0, 0);

  for (auto& p : out->m_particles) p.d = nullPose - p.d;

  MRPT_END
}

mrpt::math::TPose2D CPosePDFParticles::getMostLikelyParticle() const
{
  mrpt::math::TPose2D ret{0, 0, 0};
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

void CPosePDFParticles::bayesianFusion(
    const CPosePDF& p1, const CPosePDF& p2, [[maybe_unused]] const double minMahalanobisDistToDrop)
{
  MRPT_START

  // Start with particles from p1
  copyFrom(p1);

  // Evaluate p2 at each particle, then multiply weights
  if (const auto* p2p = dynamic_cast<const CPosePDFParticles*>(&p2))
  {
    // Kernel density estimate of p2, with Gaussian kernels shaped as its
    // covariance and scaled with Silverman's rule of thumb:
    // H = h^2 * cov, h = (4 / ((d + 2) * N))^(1 / (d + 4)), d = 3.
    const auto [cov2, mean2] = p2.getCovarianceAndMean();
    const double N2 = static_cast<double>(std::max<size_t>(p2p->m_particles.size(), 1));
    CMatrixDouble33 H = cov2;
    H *= std::pow(4.0 / (5.0 * N2), 2.0 / 7.0);
    for (int k = 0; k < 3; k++)
    {
      H(k, k) = H(k, k) * (1 + 1e-9) + 1e-12;
    }
    const CMatrixDouble33 Hinv = H.inverse_LLt();
    for (auto& p : m_particles)
    {
      // log(sum_j w_j * exp(-0.5 * d_j^T * Hinv * d_j)), without underflow:
      double maxTerm = -std::numeric_limits<double>::infinity();
      double sum = 0;
      for (const auto& q : p2p->m_particles)
      {
        const double d[3] = {p.d.x - q.d.x, p.d.y - q.d.y, math::wrapToPi(p.d.phi - q.d.phi)};
        double maha2 = 0;
        for (int r = 0; r < 3; r++)
        {
          for (int c = 0; c < 3; c++)
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
    // Generic: Gaussian approximation of p2
    const auto [cov2, mean2] = p2.getCovarianceAndMean();
    const CMatrixDouble33 cov2inv = cov2.inverse_LLt();
    for (auto& p : m_particles)
    {
      const double dx = p.d.x - mean2.x();
      const double dy = p.d.y - mean2.y();
      const double dp = math::wrapToPi(p.d.phi - mean2.phi());
      const double maha2 = cov2inv(0, 0) * dx * dx + cov2inv(1, 1) * dy * dy +
                           cov2inv(2, 2) * dp * dp + 2.0 * cov2inv(0, 1) * dx * dy +
                           2.0 * cov2inv(0, 2) * dx * dp + 2.0 * cov2inv(1, 2) * dy * dp;
      p.log_w += -0.5 * maha2;
    }
  }
  (void)normalizeWeights();

  MRPT_END
}

double CPosePDFParticles::evaluatePDF_parzen(
    const double x, const double y, const double phi, const double stdXY, const double stdPhi) const
{
  double ret = 0;
  for (const auto& p : m_particles)
  {
    double difPhi = math::wrapToPi(phi - p.d.phi);
    ret += exp(p.log_w) *
           math::normalPDF(std::sqrt(square(p.d.x - x) + square(p.d.y - y)), 0, stdXY) *
           math::normalPDF(std::abs(difPhi), 0, stdPhi);
  }
  return ret;
}

void CPosePDFParticles::saveParzenPDFToTextFile(
    const char* fileName,
    const double x_min,
    const double x_max,
    const double y_min,
    const double y_max,
    const double phi,
    const double stepSizeXY,
    const double stdXY,
    const double stdPhi) const
{
  std::string buf;

  for (double y = y_min; y < y_max; y += stepSizeXY)
    for (double x = x_min; x < x_max; x += stepSizeXY)
      buf += mrpt::format("%f ", evaluatePDF_parzen(x, y, phi, stdXY, stdPhi));
  buf += "\n";

  std::ofstream f(fileName);
  if (!f.is_open())
  {
    return;
  }
  f << buf;
}

void CPosePDFParticles::printTo(std::ostream& out) const
{
  const auto [c, m] = getCovarianceAndMean();
  out << "mrpt::poses::CPosePDFParticles object with " << size()
      << " particles, mean=" << m.asString() << " cov=" << c.inMatlabFormat() << " ESS=" << ESS();
}
