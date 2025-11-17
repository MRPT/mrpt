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
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/serialization/aligned_serialization.h>
#include <mrpt/serialization/stl_serialization.h>

#include <iostream>

#include "CPointsMap_crtp_common.h"
#include "mrpt/core/bits_mem.h"

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::img;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::config;

namespace
{
/// using string_view requires using a permanent storage somewhere in a place that outlives all
/// possible accesses.
thread_local std::set<std::string> fieldNamesCache;
}  // namespace

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("mrpt::maps::CGenericPointsMap", mrpt::maps::CGenericPointsMap)

CGenericPointsMap::TMapDefinition::TMapDefinition() = default;

void CGenericPointsMap::TMapDefinition::loadFromConfigFile_map_specific(
    const CConfigFileBase& source, const std::string& sectionNamePrefix)
{
  using namespace std::string_literals;

  insertionOpts.loadFromConfigFile(source, sectionNamePrefix + "_insertOpts"s);
  likelihoodOpts.loadFromConfigFile(source, sectionNamePrefix + "_likelihoodOpts"s);
}

void CGenericPointsMap::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  this->insertionOpts.dumpToTextStream(out);
  this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr CGenericPointsMap::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const CGenericPointsMap::TMapDefinition& def =
      *dynamic_cast<const CGenericPointsMap::TMapDefinition*>(&_def);
  auto obj = CGenericPointsMap::Create();
  obj->insertionOptions = def.insertionOpts;
  obj->likelihoodOptions = def.likelihoodOpts;
  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CGenericPointsMap, CPointsMap, mrpt::maps)

CGenericPointsMap::CGenericPointsMap(const CGenericPointsMap& o) : CPointsMap()
{
  impl_copyFrom(o);
}

CGenericPointsMap& CGenericPointsMap::operator=(const CGenericPointsMap& o)
{
  impl_copyFrom(o);
  return *this;
}

void CGenericPointsMap::reserve(size_t newLength)
{
  m_x.reserve(newLength);
  m_y.reserve(newLength);
  m_z.reserve(newLength);
  for (auto& field : m_float_fields) field.second.reserve(newLength);
  for (auto& field : m_double_fields) field.second.reserve(newLength);
  for (auto& field : m_uint16_fields) field.second.reserve(newLength);
}

void CGenericPointsMap::resize(size_t newLength)
{
  m_x.resize(newLength, 0);
  m_y.resize(newLength, 0);
  m_z.resize(newLength, 0);
  for (auto& field : m_float_fields) field.second.resize(newLength, 0);
  for (auto& field : m_double_fields) field.second.resize(newLength, 0);
  for (auto& field : m_uint16_fields) field.second.resize(newLength, 0);
  mark_as_modified();
}

void CGenericPointsMap::setSize(size_t newLength)
{
  m_x.assign(newLength, 0);
  m_y.assign(newLength, 0);
  m_z.assign(newLength, 0);
  for (auto& field : m_float_fields) field.second.assign(newLength, 0);
  for (auto& field : m_double_fields) field.second.assign(newLength, 0);
  for (auto& field : m_uint16_fields) field.second.assign(newLength, 0);
  mark_as_modified();
}

uint8_t CGenericPointsMap::serializeGetVersion() const { return 1; }
void CGenericPointsMap::serializeTo(mrpt::serialization::CArchive& out) const
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

  // Float fields
  out.WriteAs<uint32_t>(m_float_fields.size());
  for (const auto& [name, v] : m_float_fields)
  {
    out << std::string(name) << v;
  }

  // Double fields (v1)
  out.WriteAs<uint32_t>(m_double_fields.size());
  for (const auto& [name, v] : m_double_fields)
  {
    out << std::string(name) << v;
  }

  // Uint16 fields
  out.WriteAs<uint32_t>(m_uint16_fields.size());
  for (const auto& [name, v] : m_uint16_fields)
  {
    out << std::string(name) << v;
  }

  insertionOptions.writeToStream(out);
  likelihoodOptions.writeToStream(out);
}

void CGenericPointsMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  internal_clear();

  switch (version)
  {
    case 0:
    case 1:
    {
      mark_as_modified();

      // XYZ
      {
        const auto n = in.ReadAs<uint32_t>();
        m_x.resize(n);
        m_y.resize(n);
        m_z.resize(n);
        if (n > 0)
        {
          in.ReadBufferFixEndianness(m_x.data(), n);
          in.ReadBufferFixEndianness(m_y.data(), n);
          in.ReadBufferFixEndianness(m_z.data(), n);
        }
      }
      // Float fields
      {
        const auto n = in.ReadAs<uint32_t>();
        for (uint32_t i = 0; i < n; i++)
        {
          std::string name;
          in >> name;
          this->registerField_float(name);
          in >> m_float_fields[name];
        }
      }
      // double fields
      if (version >= 1)
      {
        const auto n = in.ReadAs<uint32_t>();
        for (uint32_t i = 0; i < n; i++)
        {
          std::string name;
          in >> name;
          this->registerField_double(name);
          in >> m_double_fields[name];
        }
      }
      // Uint16 fields
      {
        const auto n = in.ReadAs<uint32_t>();
        for (uint32_t i = 0; i < n; i++)
        {
          std::string name;
          in >> name;
          this->registerField_uint16(name);
          in >> m_uint16_fields[name];
        }
      }

      insertionOptions.readFromStream(in);
      likelihoodOptions.readFromStream(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void CGenericPointsMap::internal_clear()
{
  vector_strong_clear(m_x);
  vector_strong_clear(m_y);
  vector_strong_clear(m_z);
  m_float_fields.clear();
  m_double_fields.clear();
  m_uint16_fields.clear();
  mark_as_modified();
}

bool CGenericPointsMap::registerField_float(const std::string_view& fieldName)
{
  if (hasPointField(fieldName))
  {
    THROW_EXCEPTION_FMT(
        "Field '%.*s' already exists.", static_cast<int>(fieldName.size()), fieldName.data());
  }
  const auto [itPermanentFieldName, _] = fieldNamesCache.insert(std::string(fieldName));
  m_float_fields[*itPermanentFieldName].resize(size(), 0);
  return true;
}

bool CGenericPointsMap::registerField_double(const std::string_view& fieldName)
{
  if (hasPointField(fieldName))
  {
    THROW_EXCEPTION_FMT(
        "Field '%.*s' already exists.", static_cast<int>(fieldName.size()), fieldName.data());
  }
  const auto [itPermanentFieldName, _] = fieldNamesCache.insert(std::string(fieldName));
  m_double_fields[*itPermanentFieldName].resize(size(), 0);
  return true;
}

bool CGenericPointsMap::registerField_uint16(const std::string_view& fieldName)
{
  if (hasPointField(fieldName))
  {
    THROW_EXCEPTION_FMT(
        "Field '%.*s' already exists.", static_cast<int>(fieldName.size()), fieldName.data());
  }
  const auto [itPermanentFieldName, _] = fieldNamesCache.insert(std::string(fieldName));
  m_uint16_fields[*itPermanentFieldName].resize(size(), 0);
  return true;
}

bool CGenericPointsMap::unregisterField(const std::string_view& fieldName)
{
  if (m_float_fields.erase(fieldName) > 0) return true;
  if (m_double_fields.erase(fieldName) > 0) return true;
  if (m_uint16_fields.erase(fieldName) > 0) return true;
  return false;
}

void CGenericPointsMap::getPointAllFieldsFast(size_t index, std::vector<float>& point_data) const
{
  const size_t nFields =
      3 + m_float_fields.size() + m_uint16_fields.size() + m_double_fields.size();
  point_data.resize(nFields);
  point_data[0] = m_x[index];
  point_data[1] = m_y[index];
  point_data[2] = m_z[index];

  size_t i = 3;
  for (const auto& field : m_float_fields) point_data[i++] = field.second[index];
  for (const auto& field : m_double_fields) point_data[i++] = field.second[index];
  for (const auto& field : m_uint16_fields) point_data[i++] = field.second[index];
}

void CGenericPointsMap::setPointAllFieldsFast(size_t index, const std::vector<float>& point_data)
{
  const size_t nFields =
      3 + m_float_fields.size() + m_uint16_fields.size() + m_double_fields.size();
  ASSERT_EQUAL_(point_data.size(), nFields);
  m_x[index] = point_data[0];
  m_y[index] = point_data[1];
  m_z[index] = point_data[2];

  size_t i = 3;
  for (auto& field : m_float_fields) field.second[index] = point_data[i++];
  for (auto& field : m_double_fields) field.second[index] = point_data[i++];
  for (auto& field : m_uint16_fields) field.second[index] = static_cast<uint16_t>(point_data[i++]);
}

bool CGenericPointsMap::hasPointField(const std::string_view& fieldName) const
{
  return CPointsMap::hasPointField(fieldName) ||  //
         m_float_fields.count(fieldName) ||       //
         m_double_fields.count(fieldName) ||      //
         m_uint16_fields.count(fieldName);
}

std::vector<std::string_view> CGenericPointsMap::getPointFieldNames_float() const
{
  std::vector<std::string_view> names = CPointsMap::getPointFieldNames_float();
  for (const auto& f : m_float_fields) names.push_back(f.first);
  return names;
}
std::vector<std::string_view> CGenericPointsMap::getPointFieldNames_double() const
{
  std::vector<std::string_view> names = CPointsMap::getPointFieldNames_double();
  for (const auto& f : m_double_fields) names.push_back(f.first);
  return names;
}
std::vector<std::string_view> CGenericPointsMap::getPointFieldNames_uint16() const
{
  std::vector<std::string_view> names = CPointsMap::getPointFieldNames_uint16();
  for (const auto& f : m_uint16_fields) names.push_back(f.first);
  return names;
}

float CGenericPointsMap::getPointField_float(size_t index, const std::string_view& fieldName) const
{
  auto it = m_float_fields.find(fieldName);
  if (it == m_float_fields.end())
  {
    return CPointsMap::getPointField_float(index, fieldName);
  }
  ASSERT_LT_(index, it->second.size());
  return it->second[index];
}

double CGenericPointsMap::getPointField_double(
    size_t index, const std::string_view& fieldName) const
{
  auto it = m_double_fields.find(fieldName);
  if (it == m_double_fields.end())
  {
    return 0;
  }
  ASSERT_LT_(index, it->second.size());
  return it->second[index];
}

uint16_t CGenericPointsMap::getPointField_uint16(
    size_t index, const std::string_view& fieldName) const
{
  auto it = m_uint16_fields.find(fieldName);
  if (it == m_uint16_fields.end())
  {
    return 0;
  }
  ASSERT_LT_(index, it->second.size());
  return it->second[index];
}

void CGenericPointsMap::setPointField_float(
    size_t index, const std::string_view& fieldName, float value)
{
  auto it = m_float_fields.find(fieldName);
  if (it == m_float_fields.end())
  {
    THROW_EXCEPTION_FMT(
        "Field '%.*s' is not registered.", static_cast<int>(fieldName.size()), fieldName.data());
  }
  ASSERT_LT_(index, it->second.size());
  it->second[index] = value;
}
void CGenericPointsMap::setPointField_double(
    size_t index, const std::string_view& fieldName, double value)
{
  auto it = m_double_fields.find(fieldName);
  if (it == m_double_fields.end())
  {
    THROW_EXCEPTION_FMT(
        "Field '%.*s' is not registered.", static_cast<int>(fieldName.size()), fieldName.data());
  }
  ASSERT_LT_(index, it->second.size());
  it->second[index] = value;
}

void CGenericPointsMap::setPointField_uint16(
    size_t index, const std::string_view& fieldName, uint16_t value)
{
  auto it = m_uint16_fields.find(fieldName);
  if (it == m_uint16_fields.end())
  {
    THROW_EXCEPTION_FMT(
        "Field '%.*s' is not registered.", static_cast<int>(fieldName.size()), fieldName.data());
  }
  ASSERT_LT_(index, it->second.size());
  it->second[index] = value;
}

void CGenericPointsMap::insertPointField_float(const std::string_view& fieldName, float value)
{
  auto it = m_float_fields.find(fieldName);
  if (it == m_float_fields.end())
  {
    THROW_EXCEPTION_FMT(
        "Field '%.*s' is not registered.", static_cast<int>(fieldName.size()), fieldName.data());
  }
  auto& vec = it->second;
  if (vec.size() < m_x.size() - 1) vec.resize(m_x.size() - 1, 0);  // Pad
  ASSERT_EQUAL_(vec.size(), m_x.size() - 1);
  vec.push_back(value);
}

void CGenericPointsMap::insertPointField_double(const std::string_view& fieldName, double value)
{
  auto it = m_double_fields.find(fieldName);
  if (it == m_double_fields.end())
  {
    THROW_EXCEPTION_FMT(
        "Field '%.*s' is not registered.", static_cast<int>(fieldName.size()), fieldName.data());
  }
  auto& vec = it->second;
  if (vec.size() < m_x.size() - 1) vec.resize(m_x.size() - 1, 0);  // Pad
  ASSERT_EQUAL_(vec.size(), m_x.size() - 1);
  vec.push_back(value);
}

void CGenericPointsMap::insertPointField_uint16(const std::string_view& fieldName, uint16_t value)
{
  auto it = m_uint16_fields.find(fieldName);
  if (it == m_uint16_fields.end())
  {
    THROW_EXCEPTION_FMT(
        "Field '%.*s' is not registered.", static_cast<int>(fieldName.size()), fieldName.data());
  }
  auto& vec = it->second;
  if (vec.size() < m_x.size() - 1) vec.resize(m_x.size() - 1, 0);  // Pad
  ASSERT_EQUAL_(vec.size(), m_x.size() - 1);
  vec.push_back(value);
}

void CGenericPointsMap::reserveField_float(const std::string_view& fieldName, size_t n)
{
  m_float_fields[fieldName].reserve(n);
}
void CGenericPointsMap::reserveField_double(const std::string_view& fieldName, size_t n)
{
  m_double_fields[fieldName].reserve(n);
}
void CGenericPointsMap::reserveField_uint16(const std::string_view& fieldName, size_t n)
{
  m_uint16_fields[fieldName].reserve(n);
}

void CGenericPointsMap::resizeField_float(const std::string_view& fieldName, size_t n)
{
  m_float_fields[fieldName].resize(n, 0);
}
void CGenericPointsMap::resizeField_double(const std::string_view& fieldName, size_t n)
{
  m_double_fields[fieldName].resize(n, 0);
}
void CGenericPointsMap::resizeField_uint16(const std::string_view& fieldName, size_t n)
{
  m_uint16_fields[fieldName].resize(n, 0);
}

// See base class docs
void CGenericPointsMap::getPointRGB(
    size_t index, float& x, float& y, float& z, float& R, float& G, float& B) const
{
  ASSERT_LT_(index, m_x.size());
  x = m_x[index];
  y = m_y[index];
  z = m_z[index];

  // Try R,G,B fields
  auto itR = m_float_fields.find("R");
  auto itG = m_float_fields.find("G");
  auto itB = m_float_fields.find("B");

  if (itR != m_float_fields.end() && itG != m_float_fields.end() && itB != m_float_fields.end())
  {
    R = itR->second[index];
    G = itG->second[index];
    B = itB->second[index];
  }
  else
  {
    // Try intensity field
    auto itI = m_float_fields.find("intensity");
    if (itI != m_float_fields.end())
    {
      R = G = B = itI->second[index];
    }
    else
    {
      // Default color
      R = G = B = 1.0f;
    }
  }
}

// See base class docs
void CGenericPointsMap::setPointRGB(
    size_t index, float x, float y, float z, float R, float G, float B)
{
  setPoint(index, x, y, z);  // Sets x,y,z and mark_as_modified()

  // Try R,G,B fields
  auto itR = m_float_fields.find("R");
  auto itG = m_float_fields.find("G");
  auto itB = m_float_fields.find("B");

  if (itR != m_float_fields.end() && itG != m_float_fields.end() && itB != m_float_fields.end())
  {
    itR->second[index] = R;
    itG->second[index] = G;
    itB->second[index] = B;
  }
  else
  {
    // Try intensity field
    auto itI = m_float_fields.find("intensity");
    if (itI != m_float_fields.end())
    {
      itI->second[index] = R;  // Use R as intensity
    }
  }
}

/** Tries to insert R,G,B into fields "R","G","B" or "intensity" if they
 * exist */
void CGenericPointsMap::insertPointRGB(float x, float y, float z, float R, float G, float B)
{
  insertPointFast(x, y, z);

  // Try R,G,B fields
  auto itR = m_float_fields.find("R");
  auto itG = m_float_fields.find("G");
  auto itB = m_float_fields.find("B");

  if (itR != m_float_fields.end() && itG != m_float_fields.end() && itB != m_float_fields.end())
  {
    insertPointField_float("R", R);
    insertPointField_float("G", G);
    insertPointField_float("B", B);
  }
  else
  {
    // Try intensity field
    auto itI = m_float_fields.find("intensity");
    if (itI != m_float_fields.end())
    {
      insertPointField_float("intensity", R);  // Use R as intensity
    }
  }

  // Pad other fields
  for (auto& f : m_float_fields)
  {
    if (f.first != "R" && f.first != "G" && f.first != "B" && f.first != "intensity")
    {
      insertPointField_float(f.first, 0);
    }
  }

  for (auto& f : m_uint16_fields) insertPointField_uint16(f.first, 0);

  mark_as_modified();
}

namespace mrpt::maps::detail
{
using mrpt::maps::CGenericPointsMap;

template <>
struct pointmap_traits<CGenericPointsMap>
{
  inline static void internal_loadFromRangeScan2D_init(
      CGenericPointsMap& me, CPointsMap::TLaserRange2DInsertContext& lric)
  {
  }
  inline static void internal_loadFromRangeScan2D_prepareOneRange(
      CGenericPointsMap& me,
      [[maybe_unused]] const float gx,
      [[maybe_unused]] const float gy,
      [[maybe_unused]] const float gz,
      CPointsMap::TLaserRange2DInsertContext& lric)
  {
  }
  inline static void internal_loadFromRangeScan2D_postPushBack(
      CGenericPointsMap& me, CPointsMap::TLaserRange2DInsertContext& lric)
  {
    // Nothing
  }

  inline static void internal_loadFromRangeScan3D_init(
      CGenericPointsMap& me, CPointsMap::TLaserRange3DInsertContext& lric)
  {
  }
  inline static void internal_loadFromRangeScan3D_prepareOneRange(
      CGenericPointsMap& me,
      [[maybe_unused]] const float gx,
      [[maybe_unused]] const float gy,
      [[maybe_unused]] const float gz,
      CPointsMap::TLaserRange3DInsertContext& lric)
  {
  }
  inline static void internal_loadFromRangeScan3D_postPushBack(
      CGenericPointsMap& me, CPointsMap::TLaserRange3DInsertContext& lric)
  {
    // Anything to do?
  }
  inline static void internal_loadFromRangeScan3D_postOneRange(
      CGenericPointsMap& me, CPointsMap::TLaserRange3DInsertContext& lric)
  {
  }
};
}  // namespace mrpt::maps::detail

void CGenericPointsMap::loadFromRangeScan(
    const CObservation2DRangeScan& rangeScan,
    const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  mrpt::maps::detail::loadFromRangeImpl<CGenericPointsMap>::templ_loadFromRangeScan(
      *this, rangeScan, robotPose);
}

void CGenericPointsMap::loadFromRangeScan(
    const CObservation3DRangeScan& rangeScan,
    const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  mrpt::maps::detail::loadFromRangeImpl<CGenericPointsMap>::templ_loadFromRangeScan(
      *this, rangeScan, robotPose);
}

// ====PLY files import & export virtual methods
void CGenericPointsMap::PLY_import_set_vertex_count(size_t N)
{
  // Register R,G,B fields for PLY color
  if (!hasPointField("R")) registerField_float("R");
  if (!hasPointField("G")) registerField_float("G");
  if (!hasPointField("B")) registerField_float("B");
  this->setSize(N);
}

void CGenericPointsMap::PLY_import_set_vertex(
    size_t idx, const mrpt::math::TPoint3Df& pt, const TColorf* pt_color)
{
  this->setPointFast(idx, pt.x, pt.y, pt.z);
  if (pt_color)
  {
    this->setPointField_float(idx, "R", pt_color->R);
    this->setPointField_float(idx, "G", pt_color->G);
    this->setPointField_float(idx, "B", pt_color->B);
  }
}

void CGenericPointsMap::PLY_export_get_vertex(
    size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color, TColorf& pt_color) const
{
  pt.x = m_x[idx];
  pt.y = m_y[idx];
  pt.z = m_z[idx];

  // Use getPointRGB logic to find color
  float r, g, b;
  getPointRGB(idx, pt.x, pt.y, pt.z, r, g, b);
  pt_color.R = r;
  pt_color.G = g;
  pt_color.B = b;
  pt_has_color = true;
}
