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

#include <mrpt/core/round.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/serialization/CArchive.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation2DRangeScan, CObservation, mrpt::obs)

uint8_t CObservation2DRangeScan::serializeGetVersion() const { return 8; }
void CObservation2DRangeScan::serializeTo(mrpt::serialization::CArchive& out) const
{
  // The data
  out << aperture << rightToLeft << maxRange << sensorPose;
  uint32_t N = m_scan.size();
  out << N;
  ASSERT_EQUAL_(m_validRange.size(), m_scan.size());
  if (N)
  {
    out.WriteBufferFixEndianness(&m_scan[0], N);
    out.WriteBuffer(&m_validRange[0], sizeof(m_validRange[0]) * N);
  }
  out << stdError;
  out << timestamp;
  out << beamAperture;

  out << sensorLabel;

  out << deltaPitch;

  out << hasIntensity();
  if (hasIntensity()) out.WriteBufferFixEndianness(&m_intensity[0], N);

  // New in version 8:
  out << sweepDuration;
}

void CObservation2DRangeScan::truncateByDistanceAndAngle(
    float min_distance, float max_angle, float min_height, float max_height, float h)
{
  // FILTER OUT INVALID POINTS!!
  CPose3D pose;
  unsigned int k = 0;
  const auto nPts = m_scan.size();

  auto itValid = m_validRange.begin();
  for (auto itScan = m_scan.begin(); itScan != m_scan.end(); itScan++, itValid++, k++)
  {
    const auto ang = std::abs(k * aperture / nPts - aperture * 0.5f);
    float x = (*itScan) * cos(ang);

    if (min_height != 0 || max_height != 0)
    {
      ASSERT_(max_height > min_height);
      if (*itScan < min_distance || ang > max_angle || x > h - min_height || x < h - max_height)
        *itValid = false;
    }  // end if
    else if (*itScan < min_distance || ang > max_angle)
      *itValid = false;
  }
}

void CObservation2DRangeScan::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    {
      CMatrixF covSensorPose;
      in >> aperture >> rightToLeft >> maxRange >> sensorPose >> covSensorPose;
      uint32_t N, i;

      in >> N;

      this->resizeScan(N);
      if (N) in.ReadBufferFixEndianness(&m_scan[0], N);

      if (version >= 1)
      {  // Load validRange:
        if (N) in.ReadBuffer(&m_validRange[0], sizeof(m_validRange[0]) * N);
      }
      else
      {
        // validRange: Default values: If distance is not maxRange
        for (i = 0; i < N; i++) m_validRange[i] = m_scan[i] < maxRange;
      }

      if (version >= 2)
      {
        in >> stdError;
      }
      else
      {
        stdError = 0.01f;
      }

      if (version >= 3)
      {
        in >> timestamp;
      }

      // Default values for newer versions:
      beamAperture = DEG2RAD(0.25f);

      deltaPitch = 0;
      sensorLabel = "";
      sweepDuration = 0;  // Default for older versions
    }
    break;

    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    {
      uint32_t N;

      CMatrixF covSensorPose;
      in >> aperture >> rightToLeft >> maxRange >> sensorPose;

      if (version < 6)  // covSensorPose was removed in version 6
        in >> covSensorPose;

      in >> N;
      this->resizeScan(N);
      if (N)
      {
        in.ReadBufferFixEndianness(&m_scan[0], N);
        in.ReadBuffer(&m_validRange[0], sizeof(m_validRange[0]) * N);
      }
      in >> stdError;
      in.ReadBufferFixEndianness(&timestamp, 1);
      in >> beamAperture;

      if (version >= 5)
      {
        in >> sensorLabel;
        in >> deltaPitch;
      }
      else
      {
        sensorLabel = "";
        deltaPitch = 0;
      }
      if (version >= 7)
      {
        bool hasIntensity;
        in >> hasIntensity;
        setScanHasIntensity(hasIntensity);
        if (hasIntensity && N)
        {
          in.ReadBufferFixEndianness(&m_intensity[0], N);
        }
      }
      if (version >= 8)
      {
        in >> sweepDuration;
      }
      else
      {
        sweepDuration = 0;  // Default for older versions
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };

  auto lck = mrpt::lockHelper(m_cachedMapMtx.data);
  m_cachedMap.reset();
}

/*---------------------------------------------------------------
            isPlanarScan
 ---------------------------------------------------------------*/
bool CObservation2DRangeScan::isPlanarScan(const double tolerance) const
{
  return sensorPose.isHorizontal(tolerance);
}

bool CObservation2DRangeScan::hasIntensity() const { return m_has_intensity; }
void CObservation2DRangeScan::setScanHasIntensity(bool setHasIntensityFlag)
{
  m_has_intensity = setHasIntensityFlag;
}

/*---------------------------------------------------------------
            filterByExclusionAreas
 ---------------------------------------------------------------*/
void CObservation2DRangeScan::filterByExclusionAreas(const TListExclusionAreasWithRanges& areas)
{
  if (areas.empty()) return;

  MRPT_START

  float Ang, dA;
  const size_t sizeRangeScan = m_scan.size();

  ASSERT_EQUAL_(m_scan.size(), m_validRange.size());

  if (!sizeRangeScan) return;

  if (rightToLeft)
  {
    Ang = -0.5f * aperture;
    dA = aperture / (sizeRangeScan - 1);
  }
  else
  {
    Ang = +0.5f * aperture;
    dA = -aperture / (sizeRangeScan - 1);
  }

  // For each exclusion area:
  for (const auto& area : areas)
  {
    const auto& poly = area.first;
    const double z_min = area.second.first;
    const double z_max = area.second.second;

    // For each point in the scan:
    for (size_t i = 0; i < sizeRangeScan; i++, Ang += dA)
    {
      if (!m_validRange[i]) continue;  // skip already invalid

      const float R = m_scan[i];
      const float x = R * cosf(Ang);
      const float y = R * sinf(Ang);

      // Apply sensorPose:
      mrpt::math::TPoint3D pt3D;
      sensorPose.composePoint(x, y, 0, pt3D.x, pt3D.y, pt3D.z);

      if (pt3D.z >= z_min && pt3D.z <= z_max)
      {
        if (poly.contains(mrpt::math::TPoint2D(pt3D.x, pt3D.y)))
        {
          m_validRange[i] = false;
        }
      }
    }
    // Reset angle for next polygon
    if (rightToLeft)
      Ang = -0.5f * aperture;
    else
      Ang = +0.5f * aperture;
  }

  MRPT_END
}

float CObservation2DRangeScan::getScanAngle(size_t idx) const
{
  float Ang = -0.5f * aperture, dA = aperture / (m_scan.size() - 1);
  ASSERT_LT_(idx, m_scan.size());

  if (!rightToLeft)
  {
    Ang = -Ang;
    dA = -dA;
  }
  return Ang + dA * idx;
}

float CObservation2DRangeScan::getScanRelativeTimestamp(size_t idx) const
{
  ASSERT_LT_(idx, m_scan.size());
  const size_t N = m_scan.size();
  if (N <= 1 || sweepDuration <= 0.0f) return 0.0f;
  return sweepDuration * static_cast<float>(idx) / static_cast<float>(N - 1);
}

/*---------------------------------------------------------------
            filterByExclusionAreas
 ---------------------------------------------------------------*/
void CObservation2DRangeScan::filterByExclusionAreas(const std::vector<mrpt::math::CPolygon>& areas)
{
  if (areas.empty()) return;

  TListExclusionAreasWithRanges lst;
  for (const auto& area : areas)
  {
    TListExclusionAreasWithRanges::value_type dat;
    dat.first = area;
    dat.second.first = -std::numeric_limits<double>::max();
    dat.second.second = std::numeric_limits<double>::max();

    lst.push_back(dat);
  }
  filterByExclusionAreas(lst);
}

/*---------------------------------------------------------------
            filterByExclusionAngles
 ---------------------------------------------------------------*/
void CObservation2DRangeScan::filterByExclusionAngles(
    const std::vector<std::pair<double, double>>& angles)
{
  if (angles.empty()) return;

  MRPT_START

  double Ang, dA;
  const size_t sizeRangeScan = m_scan.size();

  ASSERT_EQUAL_(m_scan.size(), m_validRange.size());

  if (!sizeRangeScan) return;

  if (rightToLeft)
  {
    Ang = -0.5 * aperture;
    dA = aperture / (sizeRangeScan - 1);
  }
  else
  {
    Ang = +0.5 * aperture;
    dA = -aperture / (sizeRangeScan - 1);
  }

  // For each forbiden angle range:
  for (const auto& angle : angles)
  {
    int ap_idx_ini = mrpt::round(mrpt::math::wrapTo2Pi(angle.first - Ang) / dA);
    int ap_idx_end = mrpt::round(mrpt::math::wrapTo2Pi(angle.second - Ang) / dA);

    if (ap_idx_ini < 0) ap_idx_ini = 0;
    if (ap_idx_end < 0) ap_idx_end = 0;

    if (ap_idx_ini > static_cast<int>(sizeRangeScan)) ap_idx_ini = sizeRangeScan - 1;
    if (ap_idx_end > static_cast<int>(sizeRangeScan)) ap_idx_end = sizeRangeScan - 1;

    const size_t idx_ini = ap_idx_ini;
    const size_t idx_end = ap_idx_end;

    if (idx_end >= idx_ini)
    {
      for (size_t i = idx_ini; i <= idx_end; i++) m_validRange[i] = false;
    }
    else
    {
      for (size_t i = 0; i < idx_end; i++) m_validRange[i] = false;

      for (size_t i = idx_ini; i < sizeRangeScan; i++) m_validRange[i] = false;
    }
  }

  MRPT_END
}

namespace mrpt::obs
{
// Tricky way to call to a library that depends on us, a sort of "run-time"
// linking: ptr_internal_build_points_map_from_scan2D is a functor in
// "mrpt-obs", set by "mrpt-maps" at its startup.
using scan2pts_functor = void (*)(
    const mrpt::obs::CObservation2DRangeScan& obs,
    mrpt::maps::CMetricMap::Ptr& out_map,
    const void* insertOps);

extern scan2pts_functor ptr_internal_build_points_map_from_scan2D;
scan2pts_functor ptr_internal_build_points_map_from_scan2D = nullptr;

void internal_set_build_points_map_from_scan2D(scan2pts_functor fn)
{
  ptr_internal_build_points_map_from_scan2D = fn;
}
}  // namespace mrpt::obs

/*---------------------------------------------------------------
            internal_buildAuxPointsMap
  ---------------------------------------------------------------*/
void CObservation2DRangeScan::internal_buildAuxPointsMap(const void* options) const
{
  auto lck = mrpt::lockHelper(m_cachedMapMtx.data);

  if (!ptr_internal_build_points_map_from_scan2D)
    throw std::runtime_error(
        "[CObservation2DRangeScan::buildAuxPointsMap] ERROR: This function "
        "needs linking against mrpt-maps.\n");

  (*ptr_internal_build_points_map_from_scan2D)(*this, m_cachedMap, options);
}

/** Fill out a T2DScanProperties structure with the parameters of this scan */
void CObservation2DRangeScan::getScanProperties(T2DScanProperties& p) const
{
  p.nRays = m_scan.size();
  p.aperture = this->aperture;
  p.rightToLeft = this->rightToLeft;
}

bool mrpt::obs::operator<(const T2DScanProperties& a, const T2DScanProperties& b)
{
  if (a.nRays != b.nRays) return a.nRays < b.nRays;
  if (a.aperture != b.aperture) return a.aperture < b.aperture;
  if (a.rightToLeft != b.rightToLeft) return a.rightToLeft;
  return false;
}

void CObservation2DRangeScan::getDescriptionAsText(std::ostream& o) const
{
  CObservation::getDescriptionAsText(o);
  o << "Homogeneous matrix for the sensor's 3D pose, relative to robot "
       "base:\n";
  o << sensorPose.getHomogeneousMatrixVal<CMatrixDouble44>() << "\n" << sensorPose << "\n";

  o << format("Samples direction: %s\n", (rightToLeft) ? "Right->Left" : "Left->Right");
  o << "Points in the scan: " << m_scan.size() << "\n";
  o << format("Estimated sensor 'sigma': %f\n", stdError);
  o << format("Increment in pitch during the scan: %f deg\n", RAD2DEG(deltaPitch));
  o << format("Sweep duration: %f s\n", sweepDuration);

  size_t i, inval = 0;
  for (i = 0; i < m_scan.size(); i++)
    if (!m_validRange[i]) inval++;
  o << format("Invalid points in the scan: %u\n", (unsigned)inval);

  o << format("Sensor maximum range: %.02f m\n", maxRange);
  o << format("Sensor field-of-view (\"aperture\"): %.01f deg\n", RAD2DEG(aperture));

  o << "Raw scan values: [";
  for (i = 0; i < m_scan.size(); i++) o << format("%.03f ", m_scan[i]);
  o << "]\n";

  o << "Raw valid-scan values: [";
  for (i = 0; i < m_validRange.size(); i++) o << format("%u ", m_validRange[i] ? 1 : 0);
  o << "]\n\n";

  if (hasIntensity())
  {
    o << "Raw intensity values: [";
    for (i = 0; i < m_intensity.size(); i++) o << format("%d ", m_intensity[i]);
    o << "]\n\n";
  }
}

const float& CObservation2DRangeScan::getScanRange(size_t i) const
{
  ASSERT_LT_(i, m_scan.size());
  return m_scan[i];
}
float& CObservation2DRangeScan::getScanRange(size_t i)
{
  ASSERT_LT_(i, m_scan.size());
  return m_scan[i];
}

void CObservation2DRangeScan::setScanRange(size_t i, const float val)
{
  ASSERT_LT_(i, m_scan.size());
  m_scan[i] = val;
}

const int32_t& CObservation2DRangeScan::getScanIntensity(size_t i) const
{
  ASSERT_LT_(i, m_intensity.size());
  return m_intensity[i];
}
int32_t& CObservation2DRangeScan::getScanIntensity(size_t i)
{
  ASSERT_LT_(i, m_intensity.size());
  return m_intensity[i];
}
void CObservation2DRangeScan::setScanIntensity(size_t i, const int val)
{
  ASSERT_LT_(i, m_intensity.size());
  m_intensity[i] = val;
}

bool CObservation2DRangeScan::getScanRangeValidity(size_t i) const
{
  ASSERT_LT_(i, m_validRange.size());
  return m_validRange[i] != 0;
}
void CObservation2DRangeScan::setScanRangeValidity(size_t i, const bool val)
{
  ASSERT_LT_(i, m_validRange.size());
  m_validRange[i] = val ? 1 : 0;
}

void CObservation2DRangeScan::resizeScan(size_t len)
{
  m_scan.resize(len);
  m_intensity.resize(len);
  m_validRange.resize(len);
}

void CObservation2DRangeScan::resizeScanAndAssign(
    size_t len, const float rangeVal, const bool rangeValidity, const int32_t rangeIntensity)
{
  m_scan.assign(len, rangeVal);
  m_validRange.assign(len, rangeValidity);
  m_intensity.assign(len, rangeIntensity);
}

size_t CObservation2DRangeScan::getScanSize() const { return m_scan.size(); }
void CObservation2DRangeScan::loadFromVectors(
    size_t nRays, const float* scanRanges, const char* scanValidity)
{
  ASSERT_(scanRanges);
  ASSERT_(scanValidity);
  resizeScan(nRays);
  for (size_t i = 0; i < nRays; i++)
  {
    m_scan[i] = scanRanges[i];
    m_validRange[i] = scanValidity[i];
  }
}

// See base class docs:
std::string CObservation2DRangeScan::exportTxtHeader() const
{
  std::string ret = "RANGES[i] ... VALID[i]";
  if (hasIntensity()) ret += " ... INTENSITY[i]";
  return ret;
}

std::string CObservation2DRangeScan::exportTxtDataRow() const
{
  std::stringstream o;
  for (size_t i = 0; i < m_scan.size(); i++) o << format("%.03f ", m_scan[i]);
  o << "    ";

  for (size_t i = 0; i < m_validRange.size(); i++) o << format("%u ", m_validRange[i] ? 1 : 0);
  o << "    ";

  if (hasIntensity())
  {
    for (size_t i = 0; i < m_intensity.size(); i++) o << format("%d ", m_intensity[i]);
  }
  return o.str();
}