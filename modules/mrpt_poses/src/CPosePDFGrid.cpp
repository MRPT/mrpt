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

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include <fstream>
#include <ostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::random;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CPosePDFGrid, CPosePDF, mrpt::poses)

/*---------------------------------------------------------------
  Constructor
  ---------------------------------------------------------------*/
CPosePDFGrid::CPosePDFGrid(
    double xMin,
    double xMax,
    double yMin,
    double yMax,
    double resolutionXY,
    double resolutionPhi,
    double phiMin,
    double phiMax) :
    CPose2DGridTemplate<double>(xMin, xMax, yMin, yMax, resolutionXY, resolutionPhi, phiMin, phiMax)
{
  uniformDistribution();
}

void CPosePDFGrid::copyFrom(const CPosePDF& o)
{
  if (this == &o) return;  // It may be used sometimes

  if (o.GetRuntimeClass() == CLASS_ID(CPosePDFGrid))
  {
    const auto& grid = dynamic_cast<const CPosePDFGrid&>(o);
    setSize(
        grid.m_xMin, grid.m_xMax, grid.m_yMin, grid.m_yMax, grid.m_resolutionXY,
        grid.m_resolutionPhi, grid.m_phiMin, grid.m_phiMax);
    m_data = grid.m_data;
  }
  else
  {
    // Sample-based approximation: draw samples from o and accumulate into this grid's cells.
    const size_t N = m_data.size() * 10;
    std::vector<CVectorDouble> samples;
    o.drawManySamples(N, samples);
    std::fill(m_data.begin(), m_data.end(), 0.0);
    for (const auto& s : samples)
    {
      const int xi = mrpt::round((s[0] - m_xMin) / m_resolutionXY);
      const int yi = mrpt::round((s[1] - m_yMin) / m_resolutionXY);
      const int pi = mrpt::round((s[2] - m_phiMin) / m_resolutionPhi);
      if (xi >= 0 && xi < static_cast<int>(m_sizeX) && yi >= 0 && yi < static_cast<int>(m_sizeY) &&
          pi >= 0 && pi < static_cast<int>(m_sizePhi))
        *getByIndex(xi, yi, pi) += 1.0;
    }
    normalize();
  }
}

/*---------------------------------------------------------------
  Destructor
  ---------------------------------------------------------------*/
CPosePDFGrid::~CPosePDFGrid() = default;
/*---------------------------------------------------------------
            getMean
  Returns an estimate of the pose, (the mean, or mathematical expectation of the
 PDF), computed
    as a weighted average over all particles.
 ---------------------------------------------------------------*/
void CPosePDFGrid::getMean(CPose2D& p) const
{
  // Calc average on SE(2)
  mrpt::poses::SE_average<2> se_averager;
  for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
    for (size_t y = 0; y < m_sizeY; y++)
      for (size_t x = 0; x < m_sizeX; x++)
      {
        const double w = *getByIndex(x, y, phiInd);
        se_averager.append(CPose2D(idx2x(x), idx2y(y), idx2phi(phiInd)), w);
      }
  se_averager.get_average(p);
}

std::tuple<CMatrixDouble33, CPose2D> CPosePDFGrid::getCovarianceAndMean() const
{
  CPosePDFParticles auxParts;
  auxParts.resetDeterministic(TPose2D(0, 0, 0), m_sizePhi * m_sizeY * m_sizeX);
  size_t idx = 0;
  for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
  {
    for (size_t y = 0; y < m_sizeY; y++)
      for (size_t x = 0; x < m_sizeX; x++)
      {
        auxParts.m_particles[idx].log_w = log(*getByIndex(x, y, phiInd));
        auxParts.m_particles[idx].d = TPose2D(idx2x(x), idx2y(y), idx2phi(phiInd));
      }
  }
  return auxParts.getCovarianceAndMean();
}

uint8_t CPosePDFGrid::serializeGetVersion() const { return 0; }
void CPosePDFGrid::serializeTo(mrpt::serialization::CArchive& out) const
{
  // The size:
  out << m_xMin << m_xMax << m_yMin << m_yMax << m_phiMin << m_phiMax << m_resolutionXY
      << m_resolutionPhi << static_cast<int32_t>(m_sizeX) << static_cast<int32_t>(m_sizeY)
      << static_cast<int32_t>(m_sizePhi) << static_cast<int32_t>(m_sizeXY)
      << static_cast<int32_t>(m_idxLeftX) << static_cast<int32_t>(m_idxLeftY)
      << static_cast<int32_t>(m_idxLeftPhi);

  // The data:
  out << m_data;
}
void CPosePDFGrid::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      // The size:
      in >> m_xMin >> m_xMax >> m_yMin >> m_yMax >> m_phiMin >> m_phiMax >> m_resolutionXY >>
          m_resolutionPhi;

      int32_t sizeX, sizeY, sizePhi, sizeXY, idxLeftX, idxLeftY, idxLeftPhi;

      in >> sizeX >> sizeY >> sizePhi >> sizeXY >> idxLeftX >> idxLeftY >> idxLeftPhi;

      m_sizeX = sizeX;
      m_sizeY = sizeY;
      m_sizePhi = sizePhi;
      m_sizeXY = sizeXY;
      m_idxLeftX = idxLeftX;
      m_idxLeftY = idxLeftY;
      m_idxLeftPhi = idxLeftPhi;

      // The data:
      in >> m_data;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

bool CPosePDFGrid::saveToTextFile(const std::string& dataFile) const
{
  const auto dimsFile = dataFile + std::string("_dims.txt");

  std::ofstream f_d(dataFile), f_s(dimsFile);
  if (!f_d.is_open() || !f_s.is_open())
  {
    return false;
  }

  // Save dims:
  f_s << mrpt::format(
      "%u %u %u %f %f %f %f %f %f\n", static_cast<unsigned>(m_sizeX),
      static_cast<unsigned>(m_sizeY), static_cast<unsigned>(m_sizePhi), m_xMin, m_xMax, m_yMin,
      m_yMax, m_phiMin, m_phiMax);

  // Save one rectangular matrix each time:
  for (unsigned int phiInd = 0; phiInd < m_sizePhi; phiInd++)
  {
    for (unsigned int y = 0; y < m_sizeY; y++)
    {
      for (unsigned int x = 0; x < m_sizeX; x++)
        f_d << mrpt::format("%.5e ", *getByIndex(x, y, phiInd));
      f_d << "\n";
    }
  }

  return true;  // Done!
}

/*---------------------------------------------------------------
            changeCoordinatesReference
  ---------------------------------------------------------------*/
void CPosePDFGrid::changeCoordinatesReference(const CPose3D& newReferenceBase)
{
  const CPose2D newRef2D(newReferenceBase);
  auto oldData = m_data;
  std::fill(m_data.begin(), m_data.end(), 0.0);
  for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
    for (size_t y = 0; y < m_sizeY; y++)
      for (size_t x = 0; x < m_sizeX; x++)
      {
        const double w = oldData[phiInd * m_sizeXY + y * m_sizeX + x];
        if (w == 0.0) continue;
        const CPose2D newPose = newRef2D + CPose2D(idx2x(x), idx2y(y), idx2phi(phiInd));
        const int xi = mrpt::round((newPose.x() - m_xMin) / m_resolutionXY);
        const int yi = mrpt::round((newPose.y() - m_yMin) / m_resolutionXY);
        const int pi = mrpt::round((newPose.phi() - m_phiMin) / m_resolutionPhi);
        if (xi >= 0 && xi < static_cast<int>(m_sizeX) && yi >= 0 &&
            yi < static_cast<int>(m_sizeY) && pi >= 0 && pi < static_cast<int>(m_sizePhi))
          *getByIndex(xi, yi, pi) += w;
      }
  normalize();
}

/*---------------------------------------------------------------
          bayesianFusion
 ---------------------------------------------------------------*/
void CPosePDFGrid::bayesianFusion(
    const CPosePDF& p1, const CPosePDF& p2, [[maybe_unused]] const double minMahalanobisDistToDrop)
{
  // Initialise *this from p1
  copyFrom(p1);

  // Multiply element-wise by p2 evaluated at each grid cell.
  if (const auto* p2g = dynamic_cast<const CPosePDFGrid*>(&p2))
  {
    // Same-type, same dimensions: direct element-wise product.
    ASSERT_EQUAL_(p2g->m_sizeX, m_sizeX);
    ASSERT_EQUAL_(p2g->m_sizeY, m_sizeY);
    ASSERT_EQUAL_(p2g->m_sizePhi, m_sizePhi);
    for (size_t i = 0; i < m_data.size(); i++) m_data[i] *= p2g->m_data[i];
  }
  else if (const auto* p2p = dynamic_cast<const CPosePDFParticles*>(&p2))
  {
    // p2 is particle-based: use a Parzen-window estimate at each cell.
    const auto [cov2, mean2] = p2.getCovarianceAndMean();
    const double stdXY = std::max(std::sqrt((cov2(0, 0) + cov2(1, 1)) * 0.5), 1e-6);
    const double stdPhi = std::max(std::sqrt(cov2(2, 2)), 1e-6);
    for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
      for (size_t y = 0; y < m_sizeY; y++)
        for (size_t x = 0; x < m_sizeX; x++)
        {
          const double lik =
              p2p->evaluatePDF_parzen(idx2x(x), idx2y(y), idx2phi(phiInd), stdXY, stdPhi);
          *getByIndex(x, y, phiInd) *= lik;
        }
  }
  else
  {
    THROW_EXCEPTION("bayesianFusion: unsupported combination of PDF types for CPosePDFGrid");
  }
  normalize();
}

/*---------------------------------------------------------------
          inverse
 ---------------------------------------------------------------*/
void CPosePDFGrid::inverse(CPosePDF& o) const
{
  auto* out = dynamic_cast<CPosePDFGrid*>(&o);
  ASSERT_(out != nullptr);

  // Rebuild the output grid with the same bounds/resolution.
  out->setSize(m_xMin, m_xMax, m_yMin, m_yMax, m_resolutionXY, m_resolutionPhi, m_phiMin, m_phiMax);
  std::fill(out->m_data.begin(), out->m_data.end(), 0.0);

  const CPose2D zero(0, 0, 0);
  for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
    for (size_t y = 0; y < m_sizeY; y++)
      for (size_t x = 0; x < m_sizeX; x++)
      {
        const double w = *getByIndex(x, y, phiInd);
        if (w == 0.0) continue;
        // Inverse of pose p is (0 - p) in SE(2)
        const CPose2D invPose = zero - CPose2D(idx2x(x), idx2y(y), idx2phi(phiInd));
        const int xi = mrpt::round((invPose.x() - m_xMin) / m_resolutionXY);
        const int yi = mrpt::round((invPose.y() - m_yMin) / m_resolutionXY);
        const int pi = mrpt::round((invPose.phi() - m_phiMin) / m_resolutionPhi);
        if (xi >= 0 && xi < static_cast<int>(m_sizeX) && yi >= 0 &&
            yi < static_cast<int>(m_sizeY) && pi >= 0 && pi < static_cast<int>(m_sizePhi))
          *out->getByIndex(xi, yi, pi) += w;
      }
  out->normalize();
}

/*---------------------------------------------------------------
          drawSingleSample
 ---------------------------------------------------------------*/
void CPosePDFGrid::drawSingleSample(CPose2D& outPart) const
{
  MRPT_START

  // Compute CDF and sample
  double SUM = 0;
  for (const auto& v : m_data) SUM += v;
  ASSERT_(SUM > 0);

  const double uni = getRandomGenerator().drawUniform(0.0, 0.9999) * SUM;
  double cum = 0;
  for (size_t phiInd = 0; phiInd < m_sizePhi; phiInd++)
  {
    for (size_t y = 0; y < m_sizeY; y++)
    {
      for (size_t x = 0; x < m_sizeX; x++)
      {
        cum += *getByIndex(x, y, phiInd);
        if (uni <= cum)
        {
          outPart = CPose2D(idx2x(x), idx2y(y), idx2phi(phiInd));
          return;
        }
      }
    }
  }
  // Fallback
  outPart = CPose2D(0, 0, 0);

  MRPT_END
}

void CPosePDFGrid::drawManySamples(size_t N, std::vector<CVectorDouble>& outSamples) const
{
  outSamples.resize(N);
  for (size_t i = 0; i < N; i++)
  {
    CPose2D pose;
    drawSingleSample(pose);
    outSamples[i].resize(3);
    outSamples[i][0] = pose.x();
    outSamples[i][1] = pose.y();
    outSamples[i][2] = pose.phi();
  }
}

/*---------------------------------------------------------------
          CPosePDFGrid
 ---------------------------------------------------------------*/
void CPosePDFGrid::normalize()
{
  double SUM = 0;

  // SUM:
  for (auto it = m_data.begin(); it != m_data.end(); ++it) SUM += *it;

  if (SUM > 0)
  {
    // Normalize:
    for (double& it : m_data) it /= SUM;
  }
}

/*---------------------------------------------------------------
            uniformDistribution
  ---------------------------------------------------------------*/
void CPosePDFGrid::uniformDistribution()
{
  double val = 1.0 / static_cast<double>(m_data.size());

  for (double& it : m_data) it = val;
}

void CPosePDFGrid::printTo(std::ostream& out) const
{
  CPose2D m;
  getMean(m);
  out << "CPosePDFGrid: x=[" << m_xMin << "," << m_xMax << "] y=[" << m_yMin << "," << m_yMax
      << "] cells=" << m_sizeX << "x" << m_sizeY << "x" << m_sizePhi << " mean=" << m.asString();
}
