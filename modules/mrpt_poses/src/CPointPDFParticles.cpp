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

#include <mrpt/math/ops_containers.h>  // maximum()
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CPointPDFParticles, CPointPDF, mrpt::poses)

CPointPDFParticles::CPointPDFParticles(size_t numParticles) { setSize(numParticles); }

/** Clear all the particles (free memory) */
void CPointPDFParticles::clear() { setSize(0); }
/*---------------------------------------------------------------
    setSize
  ---------------------------------------------------------------*/
void CPointPDFParticles::setSize(size_t numberParticles, const mrpt::math::TPoint3Df& defaultValue)
{
  // Free old particles: automatic via smart ptr
  m_particles.resize(numberParticles);
  for (auto& it : m_particles)
  {
    it.log_w = 0;
    it.d.reset(new TPoint3Df(defaultValue));
  }
}

/*---------------------------------------------------------------
            getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the
 PDF)
 ---------------------------------------------------------------*/
void CPointPDFParticles::getMean(CPoint3D& p) const
{
  MRPT_START
  if (m_particles.empty()) THROW_EXCEPTION("Cannot compute mean since there are zero particles.");

  CParticleList::const_iterator it;
  double sumW = 0;
  double x = 0, y = 0, z = 0;
  for (it = m_particles.begin(); it != m_particles.end(); it++)
  {
    const double w = exp(it->log_w);
    x += it->d->x * w;
    y += it->d->y * w;
    z += it->d->z * w;
    sumW += w;
  }

  ASSERT_(sumW != 0);

  sumW = 1.0 / sumW;

  p.x(x * sumW);
  p.y(y * sumW);
  p.z(z * sumW);

  MRPT_END
}

std::tuple<CMatrixDouble33, CPoint3D> CPointPDFParticles::getCovarianceAndMean() const
{
  MRPT_START

  CPoint3D mean;
  CMatrixDouble33 cov;

  getMean(mean);
  cov.setZero();

  size_t i, n = m_particles.size();
  double var_x = 0, var_y = 0, var_p = 0, var_xy = 0, var_xp = 0, var_yp = 0;

  double lin_w_sum = 0;

  for (i = 0; i < n; i++) lin_w_sum += exp(m_particles[i].log_w);
  if (lin_w_sum == 0) lin_w_sum = 1;

  for (i = 0; i < n; i++)
  {
    double w = exp(m_particles[i].log_w) / lin_w_sum;

    double err_x = m_particles[i].d->x - mean.x();
    double err_y = m_particles[i].d->y - mean.y();
    double err_phi = m_particles[i].d->z - mean.z();

    var_x += square(err_x) * w;
    var_y += square(err_y) * w;
    var_p += square(err_phi) * w;
    var_xy += err_x * err_y * w;
    var_xp += err_x * err_phi * w;
    var_yp += err_y * err_phi * w;
  }

  if (n >= 2)
  {
    // Unbiased estimation of variance:
    cov(0, 0) = var_x;
    cov(1, 1) = var_y;
    cov(2, 2) = var_p;

    cov(1, 0) = cov(0, 1) = var_xy;
    cov(2, 0) = cov(0, 2) = var_xp;
    cov(1, 2) = cov(2, 1) = var_yp;
  }

  return {cov, mean};
  MRPT_END
}

uint8_t CPointPDFParticles::serializeGetVersion() const { return 0; }
void CPointPDFParticles::serializeTo(mrpt::serialization::CArchive& out) const
{
  uint32_t N = static_cast<uint32_t>(size());
  out << N;

  for (const auto& m_particle : m_particles)
    out << m_particle.log_w << m_particle.d->x << m_particle.d->y << m_particle.d->z;
}

void CPointPDFParticles::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      uint32_t N;
      in >> N;
      setSize(N);

      for (auto& m_particle : m_particles)
        in >> m_particle.log_w >> m_particle.d->x >> m_particle.d->y >> m_particle.d->z;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void CPointPDFParticles::copyFrom(const CPointPDF& o)
{
  if (this == &o) return;  // It may be used sometimes

  if (o.GetRuntimeClass() == CLASS_ID(CPointPDFParticles))
  {
    const auto& src = dynamic_cast<const CPointPDFParticles&>(o);
    setSize(src.m_particles.size());
    for (size_t i = 0; i < m_particles.size(); i++)
    {
      m_particles[i].log_w = src.m_particles[i].log_w;
      *m_particles[i].d = *src.m_particles[i].d;
    }
  }
  else if (o.GetRuntimeClass() == CLASS_ID(CPointPDFGaussian))
  {
    const auto& pdf = dynamic_cast<const CPointPDFGaussian&>(o);
    const size_t N = m_particles.empty() ? 1000 : m_particles.size();
    std::vector<CVectorDouble> samples;
    pdf.drawManySamples(N, samples);
    setSize(N);
    const double log_w = -std::log(static_cast<double>(N));
    for (size_t i = 0; i < N; i++)
    {
      m_particles[i].log_w = log_w;
      m_particles[i].d->x = static_cast<float>(samples[i][0]);
      m_particles[i].d->y = static_cast<float>(samples[i][1]);
      m_particles[i].d->z = static_cast<float>(samples[i][2]);
    }
  }
  else
  {
    THROW_EXCEPTION("CPointPDFParticles::copyFrom: unsupported source PDF type");
  }
}

/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
bool CPointPDFParticles::saveToTextFile(const std::string& file) const
{
  MRPT_START

  FILE* f = os::fopen(file.c_str(), "wt");
  if (!f)
  {
    return false;
  }

  size_t i, N = m_particles.size();
  for (i = 0; i < N; i++)
    os::fprintf(
        f, "%f %f %f %e\n", m_particles[i].d->x, m_particles[i].d->y, m_particles[i].d->z,
        m_particles[i].log_w);

  os::fclose(f);
  return true;
  MRPT_END
}

/*---------------------------------------------------------------
            changeCoordinatesReference
 ---------------------------------------------------------------*/
void CPointPDFParticles::changeCoordinatesReference(const CPose3D& newReferenceBase)
{
  TPoint3D pt;
  for (auto& m_particle : m_particles)
  {
    newReferenceBase.composePoint(
        m_particle.d->x, m_particle.d->y,
        m_particle.d->z,  // In
        pt.x, pt.y, pt.z  // Out
    );
    m_particle.d->x = d2f(pt.x);
    m_particle.d->y = d2f(pt.y);
    m_particle.d->z = d2f(pt.z);
  }
}

/*---------------------------------------------------------------
            computeKurtosis
 ---------------------------------------------------------------*/
double CPointPDFParticles::computeKurtosis()
{
  MRPT_START

  // kurtosis = \mu^4 / (\sigma^2) -3
  Eigen::Vector3d kurts, mu4, m, var;
  kurts.fill(0);
  mu4.fill(0);
  m.fill(0);
  var.fill(0);

  // Means:
  for (auto& m_particle : m_particles)
  {
    m[0] += m_particle.d->x;
    m[1] += m_particle.d->y;
    m[2] += m_particle.d->z;
  }
  m *= 1.0 / static_cast<double>(m_particles.size());

  // variances:
  for (auto& m_particle : m_particles)
  {
    var[0] += square(m_particle.d->x - m[0]);
    var[1] += square(m_particle.d->y - m[1]);
    var[2] += square(m_particle.d->z - m[2]);
  }
  var *= 1.0 / static_cast<double>(m_particles.size());
  var[0] = square(var[0]);
  var[1] = square(var[1]);
  var[2] = square(var[2]);

  // Moment:
  for (auto& m_particle : m_particles)
  {
    mu4[0] += pow(m_particle.d->x - m[0], 4.0);
    mu4[1] += pow(m_particle.d->y - m[1], 4.0);
    mu4[2] += pow(m_particle.d->z - m[2], 4.0);
  }
  mu4 *= 1.0 / static_cast<double>(m_particles.size());

  // Kurtosis's
  kurts.array() = mu4.array() / var.array();

  return math::maximum(kurts);

  MRPT_END
}

/*---------------------------------------------------------------
          drawSingleSample
  ---------------------------------------------------------------*/
void CPointPDFParticles::drawSingleSample(CPoint3D& outSample) const
{
  const double uni = getRandomGenerator().drawUniform(0.0, 0.9999);
  double cum = 0;
  for (const auto& p : m_particles)
  {
    cum += exp(p.log_w);
    if (uni <= cum)
    {
      outSample.x(p.d->x);
      outSample.y(p.d->y);
      outSample.z(p.d->z);
      return;
    }
  }
  const auto& last = m_particles.rbegin()->d;
  outSample.x(last->x);
  outSample.y(last->y);
  outSample.z(last->z);
}

/*---------------------------------------------------------------
          bayesianFusion
 ---------------------------------------------------------------*/
void CPointPDFParticles::bayesianFusion(
    const CPointPDF& p1_,
    const CPointPDF& p2_,
    [[maybe_unused]] const double minMahalanobisDistToDrop)
{
  MRPT_START

  // Start with particles from p1
  copyFrom(p1_);

  const auto [cov2, mean2] = p2_.getCovarianceAndMean();
  const double h = std::max(std::sqrt((cov2(0, 0) + cov2(1, 1) + cov2(2, 2)) / 3.0), 1e-6);

  if (const auto* p2p = dynamic_cast<const CPointPDFParticles*>(&p2_))
  {
    // Parzen-window KDE estimate of p2 at each particle from p1
    const size_t N2 = p2p->m_particles.size();
    for (auto& p : m_particles)
    {
      double lik = 0;
      for (const auto& q : p2p->m_particles)
      {
        const double dx = p.d->x - q.d->x;
        const double dy = p.d->y - q.d->y;
        const double dz = p.d->z - q.d->z;
        lik += std::exp(q.log_w) * std::exp(-0.5 * (dx * dx + dy * dy + dz * dz) / (h * h));
      }
      p.log_w += std::log(std::max(lik / static_cast<double>(N2), 1e-300));
    }
  }
  else
  {
    // Generic: Gaussian approximation of p2
    const CMatrixDouble33 cov2inv = cov2.inverse_LLt();
    for (auto& p : m_particles)
    {
      const double dx = p.d->x - mean2.x();
      const double dy = p.d->y - mean2.y();
      const double dz = p.d->z - mean2.z();
      const double maha2 = cov2inv(0, 0) * dx * dx + cov2inv(1, 1) * dy * dy +
                           cov2inv(2, 2) * dz * dz + 2.0 * cov2inv(0, 1) * dx * dy +
                           2.0 * cov2inv(0, 2) * dx * dz + 2.0 * cov2inv(1, 2) * dy * dz;
      p.log_w += -0.5 * maha2;
    }
  }
  (void)normalizeWeights();

  MRPT_END
}
