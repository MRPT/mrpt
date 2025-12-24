/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#pragma once

#include <mrpt/core/aligned_std_vector.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/opengl/pointcloud_adapters.h>
#include <mrpt/serialization/CSerializable.h>

#include <string_view>
#include <unordered_map>

namespace mrpt::maps
{
/** A map of 3D points (X,Y,Z) plus any number of custom, string-keyed
 * per-point data channels.
 *
 * Supported channel data types are `float`, `double`, `uint16_t`, and `uint8_t`.
 *
 * Before inserting points, you must register the fields you want to use via
 * `registerField_float()`, `registerField_double()`, ...
 *
 * When inserting points, you must call `insertPointFast()` (for X,Y,Z) and
 * then `insertPointField_float()`, `insertPointField_double()`,... for
 * **each** registered field to keep data vectors synchronized.
 *
 * Alternatively, use `resize()` or `setSize()` to allocate space, then populate
 * data using `setPointFast()` and `setPointField_float()` /
 * `setPointField_uint16()` / ...
 *
 * A mechanism is provided to copy all point fields from one point map to another:
 * - `const auto ctx = CPointsMap::prepareForInsertPointsFrom(sourcePc)`, then
 * - `CPointsMap::insertPointFrom(i, ctx)`
 *
 * Although field names can be freely set by users, these names have reserved uses:
 * - `t` (float): per-point timestamp. mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP
 * - `color_{r,g,b}` (uint8_t): per point RGB color in range [0, 255].
 *    mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8, ...
 * - `color_{rf,gf,bf}` (float): per point RGB color in range [0, 1].
 *    mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Rf, ...
 *
 * For coloring a mrpt::opengl::CPointCloudColoured using fields from a
 * mrpt::maps::CGenericPointsMap object, use mrpt::obs::recolorize3Dpc()
 * or mrpt::obs::obs_to_viz() for an mrpt::obs::CObservationPointCloud
 *
 * \sa mrpt::maps::CPointsMap, mrpt::maps::CMetricMap
 * \ingroup mrpt_maps_grp
 */
class CGenericPointsMap : public CPointsMap
{
  DEFINE_SERIALIZABLE(CGenericPointsMap, mrpt::maps)

 public:
  CGenericPointsMap() = default;
  CGenericPointsMap(const CGenericPointsMap& o);
  CGenericPointsMap& operator=(const CGenericPointsMap& o);

  /** @name Register/unregister custom data fields
    @{ */

  // see docs in parent class
  bool registerField_float(const std::string_view& fieldName) override;
  bool registerField_double(const std::string_view& fieldName) override;
  bool registerField_uint16(const std::string_view& fieldName) override;
  bool registerField_uint8(const std::string_view& fieldName) override;

  /** Removes a data channel.
   * \return True if the field existed and was removed, false otherwise.
   */
  bool unregisterField(const std::string_view& fieldName);

  /** Returns the map of float fields: map<field_name, vector_of_data> */
  const std::unordered_map<std::string_view, mrpt::aligned_std_vector<float>>& float_fields() const
  {
    return m_float_fields;
  }
  /** Returns the map of double fields: map<field_name, vector_of_data> */
  const std::unordered_map<std::string_view, mrpt::aligned_std_vector<double>>& double_fields()
      const
  {
    return m_double_fields;
  }
  /** Returns the map of uint16_t fields: map<field_name, vector_of_data> */
  const std::unordered_map<std::string_view, mrpt::aligned_std_vector<uint16_t>>& uint16_fields()
      const
  {
    return m_uint16_fields;
  }
  /** Returns the map of uint8_t fields: map<field_name, vector_of_data> */
  const std::unordered_map<std::string_view, mrpt::aligned_std_vector<uint8_t>>& uint8_fields()
      const
  {
    return m_uint8_fields;
  }

  /** @} */

  /** @name CPointsMap virtual interface implementation
    @{ */

  void reserve(size_t newLength) override;
  void resize(size_t newLength) override;
  void setSize(size_t newLength) override;

  void getPointAllFieldsFast(size_t index, std::vector<float>& point_data) const override;
  void setPointAllFieldsFast(size_t index, const std::vector<float>& point_data) override;

  void loadFromRangeScan(
      const mrpt::obs::CObservation2DRangeScan& rangeScan,
      const std::optional<const mrpt::poses::CPose3D>& robotPose) override;
  void loadFromRangeScan(
      const mrpt::obs::CObservation3DRangeScan& rangeScan,
      const std::optional<const mrpt::poses::CPose3D>& robotPose) override;

  /** @} */

  /** @name String-keyed field access virtual interface implementation
    @{ */
  bool hasPointField(const std::string_view& fieldName) const override;
  std::vector<std::string_view> getPointFieldNames_float() const override;
  std::vector<std::string_view> getPointFieldNames_double() const override;
  std::vector<std::string_view> getPointFieldNames_uint16() const override;
  std::vector<std::string_view> getPointFieldNames_uint8() const override;

  float getPointField_float(size_t index, const std::string_view& fieldName) const override;
  double getPointField_double(size_t index, const std::string_view& fieldName) const override;
  uint16_t getPointField_uint16(size_t index, const std::string_view& fieldName) const override;
  uint8_t getPointField_uint8(size_t index, const std::string_view& fieldName) const override;

  void setPointField_float(size_t index, const std::string_view& fieldName, float value) override;
  void setPointField_double(size_t index, const std::string_view& fieldName, double value) override;
  void setPointField_uint16(
      size_t index, const std::string_view& fieldName, uint16_t value) override;
  void setPointField_uint8(size_t index, const std::string_view& fieldName, uint8_t value) override;

  /** Appends a value to the given field.
   * The field must be registered.
   * Asserts that the field vector's size is exactly `this->size() - 1`
   * (i.e. you just called `insertPointFast()`).
   */
  void insertPointField_float(const std::string_view& fieldName, float value) override;

  /** Appends a value to the given field.
   * The field must be registered.
   * Asserts that the field vector's size is exactly `this->size() - 1`
   * (i.e. you just called `insertPointFast()`).
   */
  void insertPointField_double(const std::string_view& fieldName, double value) override;

  /** Appends a value to the given field.
   * The field must be registered.
   * Asserts that the field vector's size is exactly `this->size() - 1`
   * (i.e. you just called `insertPointFast()`).
   */
  void insertPointField_uint16(const std::string_view& fieldName, uint16_t value) override;

  /** Appends a value to the given field.
   * The field must be registered.
   * Asserts that the field vector's size is exactly `this->size() - 1`
   * (i.e. you just called `insertPointFast()`).
   */
  void insertPointField_uint8(const std::string_view& fieldName, uint8_t value) override;

  void reserveField_float(const std::string_view& fieldName, size_t n) override;
  void reserveField_double(const std::string_view& fieldName, size_t n) override;
  void reserveField_uint16(const std::string_view& fieldName, size_t n) override;
  void reserveField_uint8(const std::string_view& fieldName, size_t n) override;

  void resizeField_float(const std::string_view& fieldName, size_t n) override;
  void resizeField_double(const std::string_view& fieldName, size_t n) override;
  void resizeField_uint16(const std::string_view& fieldName, size_t n) override;
  void resizeField_uint8(const std::string_view& fieldName, size_t n) override;

  auto getPointsBufferRef_float_field(const std::string_view& fieldName) const
      -> const mrpt::aligned_std_vector<float>* override
  {
    if (auto* f = CPointsMap::getPointsBufferRef_float_field(fieldName); f)
    {
      return f;
    }
    if (auto it = m_float_fields.find(fieldName); it != m_float_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }
  auto getPointsBufferRef_double_field(const std::string_view& fieldName) const
      -> const mrpt::aligned_std_vector<double>* override
  {
    if (auto it = m_double_fields.find(fieldName); it != m_double_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }
  auto getPointsBufferRef_uint16_field(const std::string_view& fieldName) const
      -> const mrpt::aligned_std_vector<uint16_t>* override
  {
    if (auto it = m_uint16_fields.find(fieldName); it != m_uint16_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }
  auto getPointsBufferRef_uint8_field(const std::string_view& fieldName) const
      -> const mrpt::aligned_std_vector<uint8_t>* override
  {
    if (auto it = m_uint8_fields.find(fieldName); it != m_uint8_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }

  auto getPointsBufferRef_float_field(const std::string_view& fieldName)
      -> mrpt::aligned_std_vector<float>* override
  {
    if (auto* f = CPointsMap::getPointsBufferRef_float_field(fieldName); f)
    {
      return f;
    }
    if (auto it = m_float_fields.find(fieldName); it != m_float_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }
  auto getPointsBufferRef_double_field(const std::string_view& fieldName)
      -> mrpt::aligned_std_vector<double>* override
  {
    if (auto it = m_double_fields.find(fieldName); it != m_double_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }
  auto getPointsBufferRef_uint16_field(const std::string_view& fieldName)
      -> mrpt::aligned_std_vector<uint16_t>* override
  {
    if (auto it = m_uint16_fields.find(fieldName); it != m_uint16_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }
  auto getPointsBufferRef_uint8_field(const std::string_view& fieldName)
      -> mrpt::aligned_std_vector<uint8_t>* override
  {
    if (auto it = m_uint8_fields.find(fieldName); it != m_uint8_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }

  /** @} */

 protected:
  std::unordered_map<std::string_view, mrpt::aligned_std_vector<float>> m_float_fields;
  std::unordered_map<std::string_view, mrpt::aligned_std_vector<double>> m_double_fields;
  std::unordered_map<std::string_view, mrpt::aligned_std_vector<uint16_t>> m_uint16_fields;
  std::unordered_map<std::string_view, mrpt::aligned_std_vector<uint8_t>> m_uint8_fields;

  /** Clear the map, erasing all the points and all fields */
  void internal_clear() override;

  /** @name Redefinition of PLY Import virtual methods from CPointsMap
    @{ */
  void PLY_import_set_vertex(
      size_t idx,
      const mrpt::math::TPoint3Df& pt,
      const mrpt::img::TColorf* pt_color = nullptr) override;
  void PLY_import_set_vertex_count(size_t N) override;
  /** @} */

  /** @name Redefinition of PLY Export virtual methods from CPointsMap
    @{ */
  void PLY_export_get_vertex(
      size_t idx,
      mrpt::math::TPoint3Df& pt,
      bool& pt_has_color,
      mrpt::img::TColorf& pt_color) const override;
  /** @} */

  MAP_DEFINITION_START(CGenericPointsMap)
  mrpt::maps::CPointsMap::TInsertionOptions insertionOpts;
  mrpt::maps::CPointsMap::TLikelihoodOptions likelihoodOpts;
  MAP_DEFINITION_END(CGenericPointsMap)

};  // End of class def.

}  // namespace mrpt::maps

namespace mrpt::opengl
{
template <>
class PointCloudAdapter<mrpt::maps::CGenericPointsMap> :
    public PointCloudAdapter<mrpt::maps::CPointsMap>
{
};
}  // namespace mrpt::opengl
