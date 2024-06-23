/* +------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)            |
|                          https://www.mrpt.org/                         |
|                                                                        |
| Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
| See: https://www.mrpt.org/Authors - All rights reserved.               |
| Released under BSD License. See: https://www.mrpt.org/License          |
+------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/core/bits_mem.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/serialization/aligned_serialization.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <fstream>
#include <iostream>

#include "CPointsMap_crtp_common.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::img;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::config;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("mrpt::maps::CPointsMapXYZIRT", mrpt::maps::CPointsMapXYZIRT)

CPointsMapXYZIRT::TMapDefinition::TMapDefinition() = default;

void CPointsMapXYZIRT::TMapDefinition::loadFromConfigFile_map_specific(
    const CConfigFileBase& source, const std::string& sectionNamePrefix)
{
  insertionOpts.loadFromConfigFile(source, sectionNamePrefix + string("_insertOpts"));
  likelihoodOpts.loadFromConfigFile(source, sectionNamePrefix + string("_likelihoodOpts"));
}

void CPointsMapXYZIRT::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  this->insertionOpts.dumpToTextStream(out);
  this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr CPointsMapXYZIRT::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const CPointsMapXYZIRT::TMapDefinition& def =
      *dynamic_cast<const CPointsMapXYZIRT::TMapDefinition*>(&_def);
  auto obj = CPointsMapXYZIRT::Create();
  obj->insertionOptions = def.insertionOpts;
  obj->likelihoodOptions = def.likelihoodOpts;
  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CPointsMapXYZIRT, CPointsMap, mrpt::maps)

CPointsMapXYZIRT::CPointsMapXYZIRT(const CPointsMapXYZIRT& o) : CPointsMap()
{
  CPointsMapXYZIRT::impl_copyFrom(o);
}
CPointsMapXYZIRT::CPointsMapXYZIRT(const CPointsMapXYZI& o) : CPointsMap()
{
  CPointsMapXYZIRT::impl_copyFrom(o);
}
CPointsMapXYZIRT& CPointsMapXYZIRT::operator=(const CPointsMap& o)
{
  impl_copyFrom(o);
  return *this;
}
CPointsMapXYZIRT& CPointsMapXYZIRT::operator=(const CPointsMapXYZIRT& o)
{
  impl_copyFrom(o);
  return *this;
}
CPointsMapXYZIRT& CPointsMapXYZIRT::operator=(const CPointsMapXYZI& o)
{
  impl_copyFrom(o);
  return *this;
}

void CPointsMapXYZIRT::reserve(size_t newLength)
{
  m_x.reserve(newLength);
  m_y.reserve(newLength);
  m_z.reserve(newLength);
  m_intensity.reserve(newLength);
  m_ring.reserve(newLength);
  m_time.reserve(newLength);
}

// Resizes all point buffers so they can hold the given number of points: newly
// created points are set to default values,
//  and old contents are not changed.
void CPointsMapXYZIRT::resize(size_t newLength)
{
  m_x.resize(newLength, 0);
  m_y.resize(newLength, 0);
  m_z.resize(newLength, 0);
  m_intensity.resize(newLength, 0);
  m_ring.resize(newLength, 0);
  m_time.resize(newLength, 0);
  mark_as_modified();
}

void CPointsMapXYZIRT::resize_XYZIRT(
    size_t newLength, bool hasIntensity, bool hasRing, bool hasTime)
{
  m_x.resize(newLength, 0);
  m_y.resize(newLength, 0);
  m_z.resize(newLength, 0);
  m_intensity.resize(hasIntensity ? newLength : 0, 0);
  m_ring.resize(hasRing ? newLength : 0, 0);
  m_time.resize(hasTime ? newLength : 0, 0);
  mark_as_modified();
}

void CPointsMapXYZIRT::reserve_XYZIRT(size_t n, bool hasIntensity, bool hasRing, bool hasTime)
{
  m_x.reserve(n);
  m_y.reserve(n);
  m_z.reserve(n);
  if (hasIntensity) m_intensity.reserve(n);
  if (hasRing) m_ring.reserve(n);
  if (hasTime) m_time.reserve(n);
}

// Resizes all point buffers so they can hold the given number of points,
// *erasing* all previous contents
//  and leaving all points to default values.
void CPointsMapXYZIRT::setSize(size_t newLength)
{
  m_x.assign(newLength, 0);
  m_y.assign(newLength, 0);
  m_z.assign(newLength, 0);
  m_intensity.assign(newLength, 0);
  m_ring.assign(newLength, 0);
  m_time.assign(newLength, 0);
  mark_as_modified();
}

uint8_t CPointsMapXYZIRT::serializeGetVersion() const { return 0; }
void CPointsMapXYZIRT::serializeTo(mrpt::serialization::CArchive& out) const
{
  // XYZ
  uint32_t n = m_x.size();
  out << n;
  if (n > 0)
  {
    out.WriteBufferFixEndianness(m_x.data(), n);
    out.WriteBufferFixEndianness(m_y.data(), n);
    out.WriteBufferFixEndianness(m_z.data(), n);
  }

  // I
  n = m_intensity.size();
  out << n;
  if (n > 0) out.WriteBufferFixEndianness(m_intensity.data(), n);

  // R
  n = m_ring.size();
  out << n;
  if (n > 0) out.WriteBufferFixEndianness(m_ring.data(), n);

  // T
  n = m_time.size();
  out << n;
  if (n > 0) out.WriteBufferFixEndianness(m_time.data(), n);

  insertionOptions.writeToStream(out);
  likelihoodOptions.writeToStream(out);
}

void CPointsMapXYZIRT::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      mark_as_modified();

      // XYZ
      uint32_t n;
      in >> n;
      m_x.resize(n);
      m_y.resize(n);
      m_z.resize(n);
      if (n > 0)
      {
        in.ReadBufferFixEndianness(m_x.data(), n);
        in.ReadBufferFixEndianness(m_y.data(), n);
        in.ReadBufferFixEndianness(m_z.data(), n);
      }
      // I:
      in >> n;
      m_intensity.resize(n);
      if (n > 0) in.ReadBufferFixEndianness(m_intensity.data(), n);

      // R:
      in >> n;
      m_ring.resize(n);
      if (n > 0) in.ReadBufferFixEndianness(m_ring.data(), n);

      // T:
      in >> n;
      m_time.resize(n);
      if (n > 0) in.ReadBufferFixEndianness(m_time.data(), n);

      insertionOptions.readFromStream(in);
      likelihoodOptions.readFromStream(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void CPointsMapXYZIRT::internal_clear()
{
  vector_strong_clear(m_x);
  vector_strong_clear(m_y);
  vector_strong_clear(m_z);
  vector_strong_clear(m_intensity);
  vector_strong_clear(m_ring);
  vector_strong_clear(m_time);
  mark_as_modified();
}

void CPointsMapXYZIRT::setPointRGB(
    size_t index, float x, float y, float z, float R, float G, float B)
{
  if (index >= m_x.size()) THROW_EXCEPTION("Index out of bounds");
  m_x[index] = x;
  m_y[index] = y;
  m_z[index] = z;
  m_intensity[index] = R;
  mark_as_modified();
}

void CPointsMapXYZIRT::insertPointFast(float x, float y, float z)
{
  m_x.push_back(x);
  m_y.push_back(y);
  m_z.push_back(z);
  // mark_as_modified(); Don't, this is the "XXXFast()" method
}

void CPointsMapXYZIRT::insertPointRGB(
    float x, float y, float z, float R_intensity, float G_ignored, float B_ignored)
{
  m_x.push_back(x);
  m_y.push_back(y);
  m_z.push_back(z);
  m_intensity.push_back(R_intensity);
  mark_as_modified();
}

void CPointsMapXYZIRT::getVisualizationInto(mrpt::opengl::CSetOfObjects& o) const
{
  if (!genericMapParams.enableSaveAs3DObject) return;

  auto obj = mrpt::opengl::CPointCloudColoured::Create();

  obj->loadFromPointsMap(this);
  obj->setColor(1, 1, 1, 1.0);
  obj->setPointSize(this->renderOptions.point_size);

  o.insert(obj);
}

void CPointsMapXYZIRT::getPointRGB(
    size_t index, float& x, float& y, float& z, float& R, float& G, float& B) const
{
  ASSERT_LT_(index, m_x.size());
  ASSERT_LT_(index, m_intensity.size());

  x = m_x[index];
  y = m_y[index];
  z = m_z[index];
  R = G = B = m_intensity[index];
}

bool CPointsMapXYZIRT::saveXYZIRT_to_text_file(const std::string& file) const
{
  FILE* f = os::fopen(file.c_str(), "wt");
  if (!f) return false;
  for (unsigned int i = 0; i < m_x.size(); i++)
    os::fprintf(
        f, "%f %f %f %f %i %f\n", m_x[i], m_y[i], m_z[i], getPointIntensity(i),
        static_cast<int>(getPointRing(i)), getPointTime(i));
  os::fclose(f);
  return true;
}

bool CPointsMapXYZIRT::loadXYZIRT_from_text_file(const std::string& file)
{
  MRPT_START

  // Clear current map:
  mark_as_modified();
  this->clear();

  std::ifstream f;
  f.open(file);
  if (!f.is_open()) return false;

  while (!f.eof())
  {
    std::string line;
    std::getline(f, line);

    std::stringstream ss(line);

    float x, y, z, i, t;
    uint16_t r;
    if (!(ss >> x >> y >> z >> i >> r >> t))
    {
      break;
    }

    insertPointFast(x, y, z);
    m_intensity.push_back(i);
    m_ring.push_back(r);
    m_time.push_back(t);
  }

  return true;

  MRPT_END
}

/*---------------------------------------------------------------
addFrom_classSpecific
---------------------------------------------------------------*/
void CPointsMapXYZIRT::addFrom_classSpecific(
    const CPointsMap& anotherMap, size_t nPreviousPoints, const bool filterOutPointsAtZero)
{
  const size_t nOther = anotherMap.size();

  const auto& oxs = anotherMap.getPointsBufferRef_x();
  const auto& oys = anotherMap.getPointsBufferRef_y();
  const auto& ozs = anotherMap.getPointsBufferRef_z();

  // Specific data for this class:
  if (const auto* o = dynamic_cast<const CPointsMapXYZIRT*>(&anotherMap); o)
  {
    bool any = false;
    if (o->hasIntensityField())
    {
      m_intensity.reserve(nPreviousPoints + nOther);
      any = true;
    }
    if (o->hasRingField())
    {
      m_ring.reserve(nPreviousPoints + nOther);
      any = true;
    }
    if (o->hasTimeField())
    {
      m_time.reserve(nPreviousPoints + nOther);
      any = true;
    }
    ASSERTMSG_(
        any,
        "Cannot insert a CPointsMapXYZIRT map without any of IRT fields "
        "present.");

    for (size_t i = 0; i < nOther; i++)
    {
      if (filterOutPointsAtZero && oxs[i] == 0 && oys[i] == 0 && ozs[i] == 0) continue;

      if (o->hasIntensityField()) m_intensity.push_back(o->m_intensity[i]);
      if (o->hasRingField()) m_ring.push_back(o->m_ring[i]);
      if (o->hasTimeField()) m_time.push_back(o->m_time[i]);
    }
  }
  else if (const auto* oi = dynamic_cast<const CPointsMapXYZI*>(&anotherMap); oi)
  {
    m_intensity.reserve(nPreviousPoints + nOther);

    for (size_t i = 0; i < nOther; i++)
    {
      if (filterOutPointsAtZero && oxs[i] == 0 && oys[i] == 0 && ozs[i] == 0) continue;

      m_intensity.push_back(oi->getPointIntensity_fast(i));
    }
  }
}

bool CPointsMapXYZIRT::internal_insertObservation(
    const mrpt::obs::CObservation& obs, const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  CPose3D robotPose3D;
  if (robotPose) robotPose3D = (*robotPose);

  if (IS_CLASS(obs, CObservationVelodyneScan))
  {
    /********************************************************************
          OBSERVATION TYPE: CObservationVelodyneScan
     ********************************************************************/
    mark_as_modified();

    const auto& o = static_cast<const CObservationVelodyneScan&>(obs);

    // Automatically generate pointcloud if needed:
    // make sure points have timestamps:
    if (!o.point_cloud.size() || o.point_cloud.timestamp.empty())
    {
      CObservationVelodyneScan::TGeneratePointCloudParameters p;
      p.generatePerPointTimestamp = true;

      const_cast<CObservationVelodyneScan&>(o).generatePointCloud(p);
    }

    const auto& pc = o.point_cloud;
    ASSERT_EQUAL_(pc.x.size(), pc.intensity.size());
    ASSERT_EQUAL_(pc.x.size(), pc.timestamp.size());
    ASSERT_EQUAL_(pc.x.size(), pc.laser_id.size());

    const size_t n = pc.x.size();
    if (!n) return true;

    const size_t n0 = this->size();
    resize_XYZIRT(n0 + pc.x.size(), true /*I*/, true /*R*/, true /*T*/);

    std::vector<double> ts;
    ts.reserve(n);
    std::optional<double> minT, maxT;
    for (size_t i = 0; i < n; i++)
    {
      const double t = mrpt::Clock::toDouble(pc.timestamp[i]);
      if (!minT)
      {
        minT = t;
        maxT = t;
      }
      else
      {
        mrpt::keep_min(*minT, t);
        mrpt::keep_max(*maxT, t);
      }
      ts.push_back(t);
    }

    // scale timestamps so the center is At=0:
    const double Tmid = minT ? (*minT + *maxT) * 0.5 : .0;
    for (size_t i = 0; i < n; i++) ts[i] -= Tmid;

    for (size_t i = 0; i < n; i++)
    {
      const auto gp = robotPose3D.composePoint(mrpt::math::TPoint3D(pc.x[i], pc.y[i], pc.z[i]));
      insertPointFast(gp.x, gp.y, gp.z);
      this->insertPointField_Intensity(mrpt::u8tof(pc.intensity[i]));
      this->insertPointField_Ring(pc.laser_id[i]);
      this->insertPointField_Timestamp(ts[i]);
    }

    return true;
  }

  // let my parent class to handle it:
  return CPointsMap::internal_insertObservation(obs, robotPose);
}

namespace mrpt::maps::detail
{
using mrpt::maps::CPointsMapXYZIRT;

template <>
struct pointmap_traits<CPointsMapXYZIRT>
{
  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called only once before inserting
   * points - this is the place to reserve memory in lric for extra working
   * variables. */
  inline static void internal_loadFromRangeScan2D_init(
      CPointsMapXYZIRT& me, mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
  {
    // lric.fVars: not needed
  }

  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called once per range data */
  inline static void internal_loadFromRangeScan2D_prepareOneRange(
      CPointsMapXYZIRT& me,
      [[maybe_unused]] const float gx,
      [[maybe_unused]] const float gy,
      [[maybe_unused]] const float gz,
      mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
  {
  }
  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called after each
   * "{x,y,z}.push_back(...);" */
  inline static void internal_loadFromRangeScan2D_postPushBack(
      CPointsMapXYZIRT& me, mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
  {
    float pI = 1.0f;
    me.m_intensity.push_back(pI);
  }

  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called only once before inserting
   * points - this is the place to reserve memory in lric for extra working
   * variables. */
  inline static void internal_loadFromRangeScan3D_init(
      CPointsMapXYZIRT& me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
  {
    // Not used.
  }

  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called once per range data */
  inline static void internal_loadFromRangeScan3D_prepareOneRange(
      CPointsMapXYZIRT& me,
      [[maybe_unused]] const float gx,
      [[maybe_unused]] const float gy,
      [[maybe_unused]] const float gz,
      mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
  {
  }

  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called after each
   * "{x,y,z}.push_back(...);" */
  inline static void internal_loadFromRangeScan3D_postPushBack(
      CPointsMapXYZIRT& me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
  {
    const float pI = 1.0f;
    me.m_intensity.push_back(pI);
  }

  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called once per range data, at the
   * end */
  inline static void internal_loadFromRangeScan3D_postOneRange(
      CPointsMapXYZIRT& me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
  {
  }
};
}  // namespace mrpt::maps::detail
/** See CPointsMap::loadFromRangeScan() */
void CPointsMapXYZIRT::loadFromRangeScan(
    const CObservation2DRangeScan& rangeScan,
    const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  mrpt::maps::detail::loadFromRangeImpl<CPointsMapXYZIRT>::templ_loadFromRangeScan(
      *this, rangeScan, robotPose);
}

/** See CPointsMap::loadFromRangeScan() */
void CPointsMapXYZIRT::loadFromRangeScan(
    const CObservation3DRangeScan& rangeScan,
    const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  mrpt::maps::detail::loadFromRangeImpl<CPointsMapXYZIRT>::templ_loadFromRangeScan(
      *this, rangeScan, robotPose);
}

// ====PLY files import & export virtual methods
void CPointsMapXYZIRT::PLY_import_set_vertex_count(size_t N) { this->setSize(N); }

void CPointsMapXYZIRT::PLY_import_set_vertex(
    size_t idx, const mrpt::math::TPoint3Df& pt, const TColorf* pt_color)
{
  if (pt_color)
    this->setPointRGB(idx, pt.x, pt.y, pt.z, pt_color->R, pt_color->G, pt_color->B);
  else
    this->setPoint(idx, pt.x, pt.y, pt.z);
}

void CPointsMapXYZIRT::PLY_export_get_vertex(
    size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color, TColorf& pt_color) const
{
  pt_has_color = true;

  pt.x = m_x[idx];
  pt.y = m_y[idx];
  pt.z = m_z[idx];
  pt_color.R = pt_color.G = pt_color.B = m_intensity[idx];
}
