/* +------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)            |
|                          https://www.mrpt.org/                         |
|                                                                        |
| Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
| See: https://www.mrpt.org/Authors - All rights reserved.               |
| Released under BSD License. See: https://www.mrpt.org/License          |
 ------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/core/bits_mem.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/serialization/aligned_serialization.h>
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
MAP_DEFINITION_REGISTER("mrpt::maps::CPointsMapXYZI", mrpt::maps::CPointsMapXYZI)

CPointsMapXYZI::TMapDefinition::TMapDefinition() = default;

void CPointsMapXYZI::TMapDefinition::loadFromConfigFile_map_specific(
    const CConfigFileBase& source, const std::string& sectionNamePrefix)
{
  insertionOpts.loadFromConfigFile(source, sectionNamePrefix + string("_insertOpts"));
  likelihoodOpts.loadFromConfigFile(source, sectionNamePrefix + string("_likelihoodOpts"));
}

void CPointsMapXYZI::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  this->insertionOpts.dumpToTextStream(out);
  this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr CPointsMapXYZI::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const CPointsMapXYZI::TMapDefinition& def =
      *dynamic_cast<const CPointsMapXYZI::TMapDefinition*>(&_def);
  auto obj = CPointsMapXYZI::Create();
  obj->insertionOptions = def.insertionOpts;
  obj->likelihoodOptions = def.likelihoodOpts;
  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CPointsMapXYZI, CPointsMap, mrpt::maps)

void CPointsMapXYZI::reserve(size_t newLength)
{
  m_x.reserve(newLength);
  m_y.reserve(newLength);
  m_z.reserve(newLength);
  m_intensity.reserve(newLength);
}

// Resizes all point buffers so they can hold the given number of points: newly
// created points are set to default values,
//  and old contents are not changed.
void CPointsMapXYZI::resize(size_t newLength)
{
  m_x.resize(newLength, 0);
  m_y.resize(newLength, 0);
  m_z.resize(newLength, 0);
  m_intensity.resize(newLength, 0);
  mark_as_modified();
}

// Resizes all point buffers so they can hold the given number of points,
// *erasing* all previous contents
//  and leaving all points to default values.
void CPointsMapXYZI::setSize(size_t newLength)
{
  m_x.assign(newLength, 0);
  m_y.assign(newLength, 0);
  m_z.assign(newLength, 0);
  m_intensity.assign(newLength, 0);
  mark_as_modified();
}

uint8_t CPointsMapXYZI::serializeGetVersion() const { return 0; }
void CPointsMapXYZI::serializeTo(mrpt::serialization::CArchive& out) const
{
  uint32_t n = m_x.size();

  // First, write the number of points:
  out << n;

  if (n > 0)
  {
    out.WriteBufferFixEndianness(&m_x[0], n);
    out.WriteBufferFixEndianness(&m_y[0], n);
    out.WriteBufferFixEndianness(&m_z[0], n);
    out.WriteBufferFixEndianness(&m_intensity[0], n);
  }
  insertionOptions.writeToStream(out);
  likelihoodOptions.writeToStream(out);
}

void CPointsMapXYZI::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      mark_as_modified();

      // Read the number of points:
      uint32_t n;
      in >> n;
      this->resize(n);
      if (n > 0)
      {
        in.ReadBufferFixEndianness(&m_x[0], n);
        in.ReadBufferFixEndianness(&m_y[0], n);
        in.ReadBufferFixEndianness(&m_z[0], n);
        in.ReadBufferFixEndianness(&m_intensity[0], n);
      }
      insertionOptions.readFromStream(in);
      likelihoodOptions.readFromStream(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void CPointsMapXYZI::internal_clear()
{
  vector_strong_clear(m_x);
  vector_strong_clear(m_y);
  vector_strong_clear(m_z);
  vector_strong_clear(m_intensity);
  mark_as_modified();
}

void CPointsMapXYZI::setPointIntensity(size_t index, float I)
{
  if (index >= m_intensity.size()) THROW_EXCEPTION("Index out of bounds");
  this->m_intensity[index] = I;
  // mark_as_modified();  // No need to rebuild KD-trees, etc...
}

void CPointsMapXYZI::getVisualizationInto(mrpt::opengl::CSetOfObjects& o) const
{
  if (!genericMapParams.enableSaveAs3DObject) return;

  auto obj = mrpt::opengl::CPointCloudColoured::Create();

  obj->loadFromPointsMap(this);
  obj->setColor(1, 1, 1, 1.0);
  obj->setPointSize(this->renderOptions.point_size);

  o.insert(obj);
}

float CPointsMapXYZI::getPointIntensity(size_t index) const
{
  ASSERT_LT_(index, m_intensity.size());
  return m_intensity[index];
}

bool CPointsMapXYZI::saveXYZI_to_text_file(const std::string& file) const
{
  FILE* f = os::fopen(file.c_str(), "wt");
  if (!f) return false;
  for (unsigned int i = 0; i < m_x.size(); i++)
    os::fprintf(f, "%f %f %f %f\n", m_x[i], m_y[i], m_z[i], m_intensity[i]);
  os::fclose(f);
  return true;
}

bool CPointsMapXYZI::loadXYZI_from_text_file(const std::string& file)
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

    float x, y, z, i;
    if (!(ss >> x >> y >> z >> i))
    {
      break;
    }

    insertPointFast(x, y, z);
    m_intensity.push_back(i);
  }

  return true;

  MRPT_END
}

namespace mrpt::maps::detail
{
using mrpt::maps::CPointsMapXYZI;

template <>
struct pointmap_traits<CPointsMapXYZI>
{
  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called only once before inserting
   * points - this is the place to reserve memory in lric for extra working
   * variables. */
  inline static void internal_loadFromRangeScan2D_init(
      CPointsMapXYZI& me, mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
  {
    // lric.fVars: not needed
  }

  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called once per range data */
  inline static void internal_loadFromRangeScan2D_prepareOneRange(
      CPointsMapXYZI& me,
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
      CPointsMapXYZI& me, mrpt::maps::CPointsMap::TLaserRange2DInsertContext& lric)
  {
    float pI = 1.0f;
    me.m_intensity.push_back(pI);
  }

  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called only once before inserting
   * points - this is the place to reserve memory in lric for extra working
   * variables. */
  inline static void internal_loadFromRangeScan3D_init(
      CPointsMapXYZI& me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
  {
    // Not used.
  }

  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called once per range data */
  inline static void internal_loadFromRangeScan3D_prepareOneRange(
      CPointsMapXYZI& me,
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
      CPointsMapXYZI& me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
  {
    const float pI = 1.0f;
    me.m_intensity.push_back(pI);
  }

  /** Helper method fot the generic implementation of
   * CPointsMap::loadFromRangeScan(), to be called once per range data, at the
   * end */
  inline static void internal_loadFromRangeScan3D_postOneRange(
      CPointsMapXYZI& me, mrpt::maps::CPointsMap::TLaserRange3DInsertContext& lric)
  {
  }
};
}  // namespace mrpt::maps::detail
/** See CPointsMap::loadFromRangeScan() */
void CPointsMapXYZI::loadFromRangeScan(
    const CObservation2DRangeScan& rangeScan,
    const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  mrpt::maps::detail::loadFromRangeImpl<CPointsMapXYZI>::templ_loadFromRangeScan(
      *this, rangeScan, robotPose);
}

/** See CPointsMap::loadFromRangeScan() */
void CPointsMapXYZI::loadFromRangeScan(
    const CObservation3DRangeScan& rangeScan,
    const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  mrpt::maps::detail::loadFromRangeImpl<CPointsMapXYZI>::templ_loadFromRangeScan(
      *this, rangeScan, robotPose);
}

/* ------------------------------------------------------------------
 String-keyed field access virtual interface implementation
   ------------------------------------------------------------------ */
bool CPointsMapXYZI::hasPointField(const std::string_view& fieldName) const
{
  if (fieldName == POINT_FIELD_INTENSITY) return true;
  return CPointsMap::hasPointField(fieldName);
}
std::vector<std::string_view> CPointsMapXYZI::getPointFieldNames_float() const
{
  std::vector<std::string_view> names = CPointsMap::getPointFieldNames_float();
  names.push_back(POINT_FIELD_INTENSITY);
  return names;
}

float CPointsMapXYZI::getPointField_float(size_t index, const std::string_view& fieldName) const
{
  if (fieldName == POINT_FIELD_INTENSITY)
  {
    if (!hasIntensityField()) return 0;
    ASSERT_LT_(index, m_intensity.size());
    return m_intensity[index];
  }
  return 0;
}

void CPointsMapXYZI::setPointField_float(
    size_t index, const std::string_view& fieldName, float value)
{
  if (fieldName == POINT_FIELD_INTENSITY && hasIntensityField())
  {
    setPointIntensity(index, value);
  }
  else
  {
    CPointsMap::setPointField_float(index, fieldName, value);
  }
}

void CPointsMapXYZI::insertPointField_float(const std::string_view& fieldName, float value)
{
  if (fieldName == POINT_FIELD_INTENSITY)
  {
    m_intensity.push_back(value);
  }
  else
  {
    CPointsMap::insertPointField_float(fieldName, value);
  }
}

void CPointsMapXYZI::reserveField_float(const std::string_view& fieldName, size_t n)
{
  if (fieldName == POINT_FIELD_INTENSITY) m_intensity.reserve(n);
}
void CPointsMapXYZI::resizeField_float(const std::string_view& fieldName, size_t n)
{
  if (fieldName == POINT_FIELD_INTENSITY) m_intensity.resize(n, 0);
}

// ====PLY files import & export virtual methods
void CPointsMapXYZI::PLY_import_set_vertex_count(size_t N) { this->setSize(N); }

void CPointsMapXYZI::PLY_import_set_vertex(
    size_t idx, const mrpt::math::TPoint3Df& pt, const TColorf* pt_color)
{
  if (pt_color)
  {
    setPoint(idx, pt);
    setPointIntensity(idx, pt_color->R);
  }
  else
  {
    setPoint(idx, pt.x, pt.y, pt.z);
  }
}

void CPointsMapXYZI::PLY_export_get_vertex(
    size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color, TColorf& pt_color) const
{
  pt_has_color = true;

  pt.x = m_x[idx];
  pt.y = m_y[idx];
  pt.z = m_z[idx];
  pt_color.R = pt_color.G = pt_color.B = m_intensity[idx];
}
