/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/aligned_std_vector.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/opengl/pointcloud_adapters.h>
#include <mrpt/serialization/CSerializable.h>

#include <map>

namespace mrpt::maps
{
/** A map of 3D points (X,Y,Z) plus any number of custom, string-keyed
 * per-point data channels.
 *
 * Supported channel data types are `float` and `uint16_t`.
 *
 * Before inserting points, you must register the fields you want to use via
 * `registerField_float()` or `registerField_uint16()`.
 *
 * When inserting points, you must call `insertPointFast()` (for X,Y,Z) and
 * then `insertPointField_float()` or `insertPointField_uint16()` for **each**
 * registered field to keep data vectors synchronized.
 *
 * Alternatively, use `resize()` or `setSize()` to allocate space, then populate
 * data using `setPointFast()` and `setPointField_float()` /
 * `setPointField_uint16()`.
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

  /** Registers a new data channel of type `float`.
   * If the map is not empty, the new channel is filled with default values (0)
   * to match the current point count.
   */
  void registerField_float(const std::string& fieldName);

  /** Registers a new data channel of type `uint16_t`.
   * If the map is not empty, the new channel is filled with default values (0)
   * to match the current point count.
   */
  void registerField_uint16(const std::string& fieldName);

  /** Removes a data channel.
   * \return True if the field existed and was removed, false otherwise.
   */
  bool unregisterField(const std::string& fieldName);

  /** Returns the map of float fields: map<field_name, vector_of_data> */
  const std::map<std::string, mrpt::aligned_std_vector<float>>& float_fields() const
  {
    return m_float_fields;
  }
  /** Returns the map of uint16_t fields: map<field_name, vector_of_data> */
  const std::map<std::string, mrpt::aligned_std_vector<uint16_t>>& uint16_fields() const
  {
    return m_uint16_fields;
  }

  /** @} */

  /** @name CPointsMap virtual interface implementation
    @{ */

  void reserve(size_t newLength) override;
  void resize(size_t newLength) override;
  void setSize(size_t newLength) override;

  /** Inserts a new point (X,Y,Z).
   * You **must** call `insertPointField_float()` or `insertPointField_uint16()`
   * *after* this for each registered field to keep data vectors synchronized.
   */
  void insertPointFast(float x, float y, float z = 0) override;

  void getPointAllFieldsFast(size_t index, std::vector<float>& point_data) const override;
  void setPointAllFieldsFast(size_t index, const std::vector<float>& point_data) override;

  void loadFromRangeScan(
      const mrpt::obs::CObservation2DRangeScan& rangeScan,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;
  void loadFromRangeScan(
      const mrpt::obs::CObservation3DRangeScan& rangeScan,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;

  // See base class docs
  void getPointRGB(
      size_t index, float& x, float& y, float& z, float& R, float& G, float& B) const override;

  // See base class docs
  void setPointRGB(size_t index, float x, float y, float z, float R, float G, float B) override;

  /** Tries to insert R,G,B into fields "R","G","B" or "intensity" if they
   * exist */
  void insertPointRGB(float x, float y, float z, float R, float G, float B) override;

  /** @} */

  /** @name String-keyed field access virtual interface implementation
    @{ */
  bool hasPointField(const std::string& fieldName) const override;
  std::vector<std::string> getPointFieldNames_float() const override;
  std::vector<std::string> getPointFieldNames_uint16() const override;

  float getPointField_float(size_t index, const std::string& fieldName) const override;
  uint16_t getPointField_uint16(size_t index, const std::string& fieldName) const override;

  void setPointField_float(size_t index, const std::string& fieldName, float value) override;
  void setPointField_uint16(size_t index, const std::string& fieldName, uint16_t value) override;

  /** Appends a value to the given field.
   * The field must be registered.
   * Asserts that the field vector's size is exactly `this->size() - 1`
   * (i.e. you just called `insertPointFast()`).
   */
  void insertPointField_float(const std::string& fieldName, float value) override;
  /** Appends a value to the given field.
   * The field must be registered.
   * Asserts that the field vector's size is exactly `this->size() - 1`
   * (i.e. you just called `insertPointFast()`).
   */
  void insertPointField_uint16(const std::string& fieldName, uint16_t value) override;

  void reserveField_float(const std::string& fieldName, size_t n) override;
  void reserveField_uint16(const std::string& fieldName, size_t n) override;
  void resizeField_float(const std::string& fieldName, size_t n) override;
  void resizeField_uint16(const std::string& fieldName, size_t n) override;

  auto getPointsBufferRef_float_field(const std::string& fieldName) const
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
  auto getPointsBufferRef_uint_field(const std::string& fieldName) const
      -> const mrpt::aligned_std_vector<uint16_t>* override
  {
    if (auto it = m_uint16_fields.find(fieldName); it != m_uint16_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }

  auto getPointsBufferRef_float_field(const std::string& fieldName)
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
  auto getPointsBufferRef_uint_field(const std::string& fieldName)
      -> mrpt::aligned_std_vector<uint16_t>* override
  {
    if (auto it = m_uint16_fields.find(fieldName); it != m_uint16_fields.end())
    {
      return &it->second;
    }
    return nullptr;
  }

  /** @} */

 protected:
  /** Map from field name to data vector */
  std::map<std::string, mrpt::aligned_std_vector<float>> m_float_fields;
  /** Map from field name to data vector */
  std::map<std::string, mrpt::aligned_std_vector<uint16_t>> m_uint16_fields;

  // See base class
  void addFrom_classSpecific(
      const CPointsMap& anotherMap,
      size_t nPreviousPoints,
      const bool filterOutPointsAtZero) override;

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
/** Specialization
 * mrpt::opengl::PointCloudAdapter<mrpt::maps::CGenericPointsMap>
 * \ingroup mrpt_adapters_grp */
template <>
class PointCloudAdapter<mrpt::maps::CGenericPointsMap>
{
 private:
  mrpt::maps::CGenericPointsMap& m_obj;

 public:
  /** The type of each point XYZ coordinates */
  using coords_t = float;
  /** Has any color RGB info? */
  static constexpr bool HAS_RGB = true;
  /** Has native RGB info (as floats)? */
  static constexpr bool HAS_RGBf = true;
  /** Has native RGB info (as uint8_t)? */
  static constexpr bool HAS_RGBu8 = false;

  /** Constructor (accept a const ref for convenience) */
  explicit PointCloudAdapter(const mrpt::maps::CGenericPointsMap& obj) :
      m_obj(*const_cast<mrpt::maps::CGenericPointsMap*>(&obj))
  {
  }
  /** Get number of points */
  inline size_t size() const { return m_obj.size(); }
  /** Set number of points (to uninitialized values) */
  inline void resize(size_t N) { m_obj.resize(N); }
  /** Does nothing as of now */
  inline void setDimensions(size_t /*height*/, size_t /*width*/) {}
  /** Get XYZ coordinates of i'th point */
  template <typename T>
  inline void getPointXYZ(size_t idx, T& x, T& y, T& z) const
  {
    m_obj.getPointFast(idx, x, y, z);
  }
  /** Set XYZ coordinates of i'th point */
  inline void setPointXYZ(size_t idx, const coords_t x, const coords_t y, const coords_t z)
  {
    m_obj.setPointFast(idx, x, y, z);
  }

  /** Get XYZ_RGBf coordinates of i'th point */
  template <typename T>
  inline void getPointXYZ_RGBAf(
      size_t idx, T& x, T& y, T& z, float& r, float& g, float& b, float& a) const
  {
    m_obj.getPointRGB(idx, x, y, z, r, g, b);
    a = 1.0f;
  }
  /** Set XYZ_RGBf coordinates of i'th point */
  inline void setPointXYZ_RGBAf(
      size_t idx,
      const coords_t x,
      const coords_t y,
      const coords_t z,
      const float r,
      const float g,
      const float b,
      [[maybe_unused]] const float a)
  {
    m_obj.setPointRGB(idx, x, y, z, r, g, b);
  }

  // Color getters/setters:
  // (Get) Tries to read "R","G","B" or "intensity"
  inline void getPointRGBf(size_t idx, float& r, float& g, float& b) const
  {
    float x, y, z;
    m_obj.getPointRGB(idx, x, y, z, r, g, b);
  }
  // (Set) Tries to write "R","G","B"
  inline void setPointRGBf(size_t idx, const float r, const float g, const float b)
  {
    if (m_obj.hasPointField("R")) m_obj.setPointField_float(idx, "R", r);
    if (m_obj.hasPointField("G")) m_obj.setPointField_float(idx, "G", g);
    if (m_obj.hasPointField("B")) m_obj.setPointField_float(idx, "B", b);
  }
};
}  // namespace mrpt::opengl
