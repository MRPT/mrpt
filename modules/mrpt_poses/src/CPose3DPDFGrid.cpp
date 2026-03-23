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
#include <mrpt/poses/CPose3DPDFGrid.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
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

IMPLEMENTS_SERIALIZABLE(CPose3DPDFGrid, CPose3DPDF, mrpt::poses)

CPose3DPDFGrid::CPose3DPDFGrid(
    const mrpt::math::TPose3D& bb_min,
    const mrpt::math::TPose3D& bb_max,
    double resolution_XYZ,
    double resolution_YPR) :
    CPose3DGridTemplate<double>(bb_min, bb_max, resolution_XYZ, resolution_YPR)
{
  uniformDistribution();
}

void CPose3DPDFGrid::copyFrom(const CPose3DPDF& o)
{
  if (this == &o) return;  // It may be used sometimes

  if (o.GetRuntimeClass() == CLASS_ID(CPose3DPDFGrid))
  {
    const auto& grid = dynamic_cast<const CPose3DPDFGrid&>(o);
    setSize(grid.m_bb_min, grid.m_bb_max, grid.m_resolutionXYZ, grid.m_resolutionYPR);
    m_data = grid.m_data;
  }
  else
  {
    // Sample-based approximation: draw samples from o and accumulate into this grid's cells.
    const size_t N = m_size_xyzYPR * 10;
    std::vector<CVectorDouble> samples;
    o.drawManySamples(N, samples);
    std::fill(m_data.begin(), m_data.end(), 0.0);
    for (const auto& s : samples)
    {
      // s = [x, y, z, yaw, pitch, roll]
      const int cx = mrpt::round((s[0] - m_bb_min.x) / m_resolutionXYZ);
      const int cy = mrpt::round((s[1] - m_bb_min.y) / m_resolutionXYZ);
      const int cz = mrpt::round((s[2] - m_bb_min.z) / m_resolutionXYZ);
      const int cY = mrpt::round((s[3] - m_bb_min.yaw) / m_resolutionYPR);
      const int cP = mrpt::round((s[4] - m_bb_min.pitch) / m_resolutionYPR);
      const int cR = mrpt::round((s[5] - m_bb_min.roll) / m_resolutionYPR);
      if (cx >= 0 && cx < static_cast<int>(m_sizeX) && cy >= 0 && cy < static_cast<int>(m_sizeY) &&
          cz >= 0 && cz < static_cast<int>(m_sizeZ) && cY >= 0 &&
          cY < static_cast<int>(m_sizeYaw) && cP >= 0 && cP < static_cast<int>(m_sizePitch) &&
          cR >= 0 && cR < static_cast<int>(m_sizeRoll))
        *getByIndex(cx, cy, cz, cY, cP, cR) += 1.0;
    }
    normalize();
  }
}

void CPose3DPDFGrid::getMean(CPose3D& p) const
{
  // Calc average on SE(3)
  mrpt::poses::SE_average<3> se_averager;

  for (size_t cR = 0; cR < m_sizeRoll; cR++)
    for (size_t cP = 0; cP < m_sizePitch; cP++)
      for (size_t cY = 0; cY < m_sizeYaw; cY++)
        for (size_t cz = 0; cz < m_sizeZ; cz++)
          for (size_t cy = 0; cy < m_sizeY; cy++)
            for (size_t cx = 0; cx < m_sizeX; cx++)
            {
              const double w = *getByIndex(cx, cy, cz, cY, cP, cR);
              se_averager.append(
                  CPose3D(
                      idx2x(cx), idx2y(cy), idx2z(cz), idx2yaw(cY), idx2pitch(cP), idx2roll(cR)),
                  w);
            }
  se_averager.get_average(p);
}

std::tuple<CMatrixDouble66, CPose3D> CPose3DPDFGrid::getCovarianceAndMean() const
{
  CPose3DPDFParticles auxParts;
  auxParts.resetDeterministic(TPose3D(), m_size_xyzYPR);

  size_t idx = 0;
  for (size_t cR = 0; cR < m_sizeRoll; cR++)
    for (size_t cP = 0; cP < m_sizePitch; cP++)
      for (size_t cY = 0; cY < m_sizeYaw; cY++)
        for (size_t cz = 0; cz < m_sizeZ; cz++)
          for (size_t cy = 0; cy < m_sizeY; cy++)
            for (size_t cx = 0; cx < m_sizeX; cx++)
            {
              const double w = *getByIndex(cx, cy, cz, cY, cP, cR);
              auxParts.m_particles[idx].log_w = std::log(w);
              auxParts.m_particles[idx].d = mrpt::math::TPose3D(
                  idx2x(cx), idx2y(cy), idx2z(cz), idx2yaw(cY), idx2pitch(cP), idx2roll(cR));

              ++idx;
            }
  return auxParts.getCovarianceAndMean();
}

uint8_t CPose3DPDFGrid::serializeGetVersion() const { return 0; }
void CPose3DPDFGrid::serializeTo(mrpt::serialization::CArchive& out) const
{
  // The size:
  out << m_bb_min << m_bb_max << m_resolutionXYZ << m_resolutionYPR;
  out.WriteAs<int32_t>(m_sizeX);
  out.WriteAs<int32_t>(m_sizeY);
  out.WriteAs<int32_t>(m_sizeZ);
  out.WriteAs<int32_t>(m_sizeYaw);
  out.WriteAs<int32_t>(m_sizePitch);
  out.WriteAs<int32_t>(m_sizeRoll);
  out << m_sizeX << m_sizeY << m_sizeZ << m_sizeYaw << m_sizePitch << m_sizeRoll;
  out << m_min_cidX << m_min_cidY << m_min_cidZ << m_min_cidYaw << m_min_cidPitch << m_min_cidRoll;

  // The data:
  out << m_data;
}
void CPose3DPDFGrid::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      in >> m_bb_min >> m_bb_max >> m_resolutionXYZ >> m_resolutionYPR;
      m_sizeX = in.ReadAs<int32_t>();
      m_sizeY = in.ReadAs<int32_t>();
      m_sizeZ = in.ReadAs<int32_t>();
      m_sizeYaw = in.ReadAs<int32_t>();
      m_sizePitch = in.ReadAs<int32_t>();
      m_sizeRoll = in.ReadAs<int32_t>();
      in >> m_sizeX >> m_sizeY >> m_sizeZ >> m_sizeYaw >> m_sizePitch >> m_sizeRoll;
      in >> m_min_cidX >> m_min_cidY >> m_min_cidZ >> m_min_cidYaw >> m_min_cidPitch >>
          m_min_cidRoll;

      // The data:
      in >> m_data;

      update_cached_size_products();

      ASSERT_EQUAL_(m_data.size(), m_size_xyzYPR);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

bool CPose3DPDFGrid::saveToTextFile(const std::string& dataFile) const
{
  std::ofstream f(dataFile);
  if (!f.is_open()) return false;

  // Header with grid metadata
  f << "% CPose3DPDFGrid text dump\n"
    << "% bb_min: " << m_bb_min.x << " " << m_bb_min.y << " " << m_bb_min.z << " " << m_bb_min.yaw
    << " " << m_bb_min.pitch << " " << m_bb_min.roll << "\n"
    << "% bb_max: " << m_bb_max.x << " " << m_bb_max.y << " " << m_bb_max.z << " " << m_bb_max.yaw
    << " " << m_bb_max.pitch << " " << m_bb_max.roll << "\n"
    << "% resolution_xyz: " << m_resolutionXYZ << " resolution_ypr: " << m_resolutionYPR << "\n"
    << "% sizes: " << m_sizeX << " " << m_sizeY << " " << m_sizeZ << " " << m_sizeYaw << " "
    << m_sizePitch << " " << m_sizeRoll << "\n"
    << "% columns: cx cy cz cYaw cPitch cRoll probability\n";

  // Sparse dump: only non-zero cells
  for (uint32_t cR = 0; cR < m_sizeRoll; cR++)
    for (uint32_t cP = 0; cP < m_sizePitch; cP++)
      for (uint32_t cY = 0; cY < m_sizeYaw; cY++)
        for (uint32_t cz = 0; cz < m_sizeZ; cz++)
          for (uint32_t cy = 0; cy < m_sizeY; cy++)
            for (uint32_t cx = 0; cx < m_sizeX; cx++)
            {
              const double w = *getByIndex(cx, cy, cz, cY, cP, cR);
              if (w != 0.0)
                f << cx << " " << cy << " " << cz << " " << cY << " " << cP << " " << cR << " "
                  << mrpt::format("%.6e\n", w);
            }
  return true;
}

void CPose3DPDFGrid::changeCoordinatesReference(const CPose3D& newReferenceBase)
{
  auto oldData = m_data;
  std::fill(m_data.begin(), m_data.end(), 0.0);

  for (uint32_t cR = 0; cR < m_sizeRoll; cR++)
    for (uint32_t cP = 0; cP < m_sizePitch; cP++)
      for (uint32_t cY = 0; cY < m_sizeYaw; cY++)
        for (uint32_t cz = 0; cz < m_sizeZ; cz++)
          for (uint32_t cy = 0; cy < m_sizeY; cy++)
            for (uint32_t cx = 0; cx < m_sizeX; cx++)
            {
              const double w = oldData[idx2absidx(cx, cy, cz, cY, cP, cR)];
              if (w == 0.0) continue;

              const CPose3D newPose =
                  newReferenceBase +
                  CPose3D(
                      idx2x(cx), idx2y(cy), idx2z(cz), idx2yaw(cY), idx2pitch(cP), idx2roll(cR));

              const int nx = mrpt::round((newPose.x() - m_bb_min.x) / m_resolutionXYZ);
              const int ny = mrpt::round((newPose.y() - m_bb_min.y) / m_resolutionXYZ);
              const int nz = mrpt::round((newPose.z() - m_bb_min.z) / m_resolutionXYZ);
              const int nY = mrpt::round((newPose.yaw() - m_bb_min.yaw) / m_resolutionYPR);
              const int nP = mrpt::round((newPose.pitch() - m_bb_min.pitch) / m_resolutionYPR);
              const int nR = mrpt::round((newPose.roll() - m_bb_min.roll) / m_resolutionYPR);

              if (nx >= 0 && nx < static_cast<int>(m_sizeX) && ny >= 0 &&
                  ny < static_cast<int>(m_sizeY) && nz >= 0 && nz < static_cast<int>(m_sizeZ) &&
                  nY >= 0 && nY < static_cast<int>(m_sizeYaw) && nP >= 0 &&
                  nP < static_cast<int>(m_sizePitch) && nR >= 0 &&
                  nR < static_cast<int>(m_sizeRoll))
                *getByIndex(nx, ny, nz, nY, nP, nR) += w;
            }
  normalize();
}

void CPose3DPDFGrid::bayesianFusion(const CPose3DPDF& p1, const CPose3DPDF& p2)
{
  // Initialise *this from p1
  copyFrom(p1);

  // Multiply element-wise by p2
  if (const auto* p2g = dynamic_cast<const CPose3DPDFGrid*>(&p2))
  {
    ASSERT_EQUAL_(p2g->m_sizeX, m_sizeX);
    ASSERT_EQUAL_(p2g->m_sizeY, m_sizeY);
    ASSERT_EQUAL_(p2g->m_sizeZ, m_sizeZ);
    ASSERT_EQUAL_(p2g->m_sizeYaw, m_sizeYaw);
    ASSERT_EQUAL_(p2g->m_sizePitch, m_sizePitch);
    ASSERT_EQUAL_(p2g->m_sizeRoll, m_sizeRoll);
    for (size_t i = 0; i < m_data.size(); i++) m_data[i] *= p2g->m_data[i];
  }
  else
  {
    THROW_EXCEPTION("bayesianFusion: unsupported combination of PDF types for CPose3DPDFGrid");
  }
  normalize();
}

void CPose3DPDFGrid::inverse(CPose3DPDF& o) const
{
  auto* out = dynamic_cast<CPose3DPDFGrid*>(&o);
  ASSERT_(out != nullptr);

  out->setSize(m_bb_min, m_bb_max, m_resolutionXYZ, m_resolutionYPR);
  std::fill(out->m_data.begin(), out->m_data.end(), 0.0);

  const CPose3D zero(0, 0, 0, 0, 0, 0);
  for (uint32_t cR = 0; cR < m_sizeRoll; cR++)
    for (uint32_t cP = 0; cP < m_sizePitch; cP++)
      for (uint32_t cY = 0; cY < m_sizeYaw; cY++)
        for (uint32_t cz = 0; cz < m_sizeZ; cz++)
          for (uint32_t cy = 0; cy < m_sizeY; cy++)
            for (uint32_t cx = 0; cx < m_sizeX; cx++)
            {
              const double w = *getByIndex(cx, cy, cz, cY, cP, cR);
              if (w == 0.0) continue;

              const CPose3D inv = zero - CPose3D(
                                             idx2x(cx), idx2y(cy), idx2z(cz), idx2yaw(cY),
                                             idx2pitch(cP), idx2roll(cR));

              const int nx = mrpt::round((inv.x() - m_bb_min.x) / m_resolutionXYZ);
              const int ny = mrpt::round((inv.y() - m_bb_min.y) / m_resolutionXYZ);
              const int nz = mrpt::round((inv.z() - m_bb_min.z) / m_resolutionXYZ);
              const int nY = mrpt::round((inv.yaw() - m_bb_min.yaw) / m_resolutionYPR);
              const int nP = mrpt::round((inv.pitch() - m_bb_min.pitch) / m_resolutionYPR);
              const int nR = mrpt::round((inv.roll() - m_bb_min.roll) / m_resolutionYPR);

              if (nx >= 0 && nx < static_cast<int>(m_sizeX) && ny >= 0 &&
                  ny < static_cast<int>(m_sizeY) && nz >= 0 && nz < static_cast<int>(m_sizeZ) &&
                  nY >= 0 && nY < static_cast<int>(m_sizeYaw) && nP >= 0 &&
                  nP < static_cast<int>(m_sizePitch) && nR >= 0 &&
                  nR < static_cast<int>(m_sizeRoll))
                *out->getByIndex(nx, ny, nz, nY, nP, nR) += w;
            }
  out->normalize();
}

void CPose3DPDFGrid::drawSingleSample(CPose3D& outPart) const
{
  MRPT_START

  double SUM = 0;
  for (const auto& v : m_data) SUM += v;
  ASSERT_(SUM > 0);

  const double uni = getRandomGenerator().drawUniform(0.0, 0.9999) * SUM;
  double cum = 0;

  for (uint32_t cR = 0; cR < m_sizeRoll; cR++)
    for (uint32_t cP = 0; cP < m_sizePitch; cP++)
      for (uint32_t cY = 0; cY < m_sizeYaw; cY++)
        for (uint32_t cz = 0; cz < m_sizeZ; cz++)
          for (uint32_t cy = 0; cy < m_sizeY; cy++)
            for (uint32_t cx = 0; cx < m_sizeX; cx++)
            {
              cum += *getByIndex(cx, cy, cz, cY, cP, cR);
              if (uni <= cum)
              {
                outPart.setFromValues(
                    idx2x(cx), idx2y(cy), idx2z(cz), idx2yaw(cY), idx2pitch(cP), idx2roll(cR));
                return;
              }
            }

  outPart = CPose3D(0, 0, 0, 0, 0, 0);

  MRPT_END
}

void CPose3DPDFGrid::drawManySamples(size_t N, std::vector<CVectorDouble>& outSamples) const
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

void CPose3DPDFGrid::normalize()
{
  double SUM = 0;

  // SUM:
  for (auto it = m_data.begin(); it != m_data.end(); ++it) SUM += *it;

  if (SUM > 0)
  {
    const auto f = 1.0 / SUM;
    for (double& it : m_data) it *= f;
  }
}

void CPose3DPDFGrid::uniformDistribution()
{
  const double val = 1.0 / m_data.size();

  for (double& it : m_data) it = val;
}

void CPose3DPDFGrid::printTo(std::ostream& out) const
{
  CPose3D m;
  getMean(m);
  out << "CPose3DPDFGrid: xyz=[" << m_bb_min.x << "," << m_bb_max.x << "] cells=" << m_sizeX << "x"
      << m_sizeY << "x" << m_sizeZ << "x" << m_sizeYaw << "x" << m_sizePitch << "x" << m_sizeRoll
      << " mean=" << m.asString();
}
