/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/opengl/pointcloud_adapters.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::maps
{
/** A map of 3D points with channels: X,Y,Z,I (intensity), R (ring), T (time).
 *
 * - `ring` (`uint16_t`) holds the "ring number", or the "row index" for
 * organized point clouds.
 * - `time` (`float`) uses the convention of time offset **in seconds** since
 * the first firing of the cloud. So, for example a 10 Hz LIDAR will produce
 * clouds with XYZIRT points with `time` in the range [0, 0.1] seconds.
 *
 * All three fields I,R,T are optional. Empty vectors are used to represent that
 * any of these fields is empty, and trying to read them will silently read
 * zeros.
 *
 * \sa mrpt::maps::CPointsMap, mrpt::maps::CMetricMap
 * \ingroup mrpt_maps_grp
 */
class [[deprecated("Use CGenericPointsMap instead")]] CPointsMapXYZIRT : public CPointsMap
{
  DEFINE_SERIALIZABLE(CPointsMapXYZIRT, mrpt::maps)

 public:
  CPointsMapXYZIRT() = default;

  explicit CPointsMapXYZIRT(const CPointsMap& o) { CPointsMap::operator=(o); }
  CPointsMapXYZIRT(const CPointsMapXYZIRT& o);
  CPointsMapXYZIRT& operator=(const CPointsMap& o);
  CPointsMapXYZIRT& operator=(const CPointsMapXYZIRT& o);

  /** @name Pure virtual interfaces to be implemented by any class derived from CPointsMap
  @{ */

  // By default, these method will grow all fields XYZIRT. See other methods
  // below.
  void reserve(size_t newLength) override;  // See base class docs
  void resize(size_t newLength) override;   // See base class docs
  void setSize(size_t newLength) override;  // See base class docs

  /// Like reserve(), but allows selecting which fields are present or not:
  void reserve_XYZIRT(size_t n, bool hasIntensity, bool hasRing, bool hasTime);

  /// Like resize(), but allows selecting which fields are present or not:
  void resize_XYZIRT(size_t newLength, bool hasIntensity, bool hasRing, bool hasTime);

  bool hasIntensityField() const { return !m_intensity.empty(); }
  bool hasRingField() const { return !m_ring.empty(); }
  bool hasTimeField() const { return !m_time.empty(); }

  /** Get all the data fields for one point as a vector: [X Y Z I]
   *  Unlike getPointAllFields(), this method does not check for index out of
   * bounds
   * \sa getPointAllFields, setPointAllFields, setPointAllFieldsFast
   */
  void getPointAllFieldsFast(size_t index, std::vector<float> & point_data) const override
  {
    point_data.resize(6);
    point_data[0] = m_x[index];
    point_data[1] = m_y[index];
    point_data[2] = m_z[index];
    point_data[3] = !m_intensity.empty() ? m_intensity[index] : 0;
    point_data[4] = !m_ring.empty() ? m_ring[index] : 0;
    point_data[5] = !m_time.empty() ? m_time[index] : 0;
  }

  /** Set all the data fields for one point as a vector: [X Y Z I R T]
   *  Unlike setPointAllFields(), this method does not check for index out of
   * bounds
   * \sa setPointAllFields, getPointAllFields, getPointAllFieldsFast
   */
  void setPointAllFieldsFast(size_t index, const std::vector<float>& point_data) override
  {
    ASSERT_(point_data.size() == 6);
    m_x[index] = point_data[0];
    m_y[index] = point_data[1];
    m_z[index] = point_data[2];
    if (hasIntensityField()) m_intensity[index] = point_data[3];
    if (hasRingField()) m_ring[index] = static_cast<uint16_t>(point_data[4]);
    if (hasTimeField()) m_time[index] = point_data[5];
  }

  /** See CPointsMap::loadFromRangeScan() */
  void loadFromRangeScan(
      const mrpt::obs::CObservation2DRangeScan& rangeScan,
      const std::optional<const mrpt::poses::CPose3D>& robotPose) override;
  /** See CPointsMap::loadFromRangeScan() */
  void loadFromRangeScan(
      const mrpt::obs::CObservation3DRangeScan& rangeScan,
      const std::optional<const mrpt::poses::CPose3D>& robotPose) override;

 protected:
  // Friend methods:
  template <class Derived>
  friend struct detail::loadFromRangeImpl;
  template <class Derived>
  friend struct detail::pointmap_traits;

 public:
  /** @} */

  /** Save to a text file. In each line contains `X Y Z I R T`
   * Returns false if any error occurred, true elsewere.
   */
  bool saveXYZIRT_to_text_file(const std::string& file) const;

  /** Loads from a text file, each line having "X Y Z I", I in [0,1].
   * Returns false if any error occurred, true elsewere. */
  bool loadXYZIRT_from_text_file(const std::string& file);

  /** Changes the intensity of a given point from the map. First index is 0.
   * \exception Throws std::exception on index out of bound.
   */
  void setPointIntensity(size_t index, float intensity)
  {
    ASSERT_LT_(index, m_intensity.size());
    m_intensity[index] = intensity;
    // mark_as_modified();  // No need to rebuild KD-trees, etc...
  }

  /** Changes the ring of a given point from the map.
   * \exception Throws std::exception on index out of bound.
   */
  void setPointRing(size_t index, uint16_t ring)
  {
    ASSERT_LT_(index, m_ring.size());
    m_ring[index] = ring;
    // mark_as_modified();  // No need to rebuild KD-trees, etc...
  }

  /** Changes the time of a given point from the map. First index is 0.
   * \exception Throws std::exception on index out of bound.
   */
  void setPointTime(size_t index, float time)
  {
    ASSERT_LT_(index, m_time.size());
    m_time[index] = time;
    // mark_as_modified();  // No need to rebuild KD-trees, etc...
  }

  /** Like \c setPointColor but without checking for out-of-index errors */
  inline void setPointColor_fast(size_t index, float R, float G, float B)
  {
    m_intensity[index] = R;
  }

  /** Gets point intensity ([0,1]), or 0 if field is not present */
  float getPointIntensity(size_t index) const
  {
    if (m_intensity.empty()) return 0;
    ASSERT_LT_(index, m_intensity.size());
    return m_intensity[index];
  }
  /** Gets point ring number, or 0 if field is not present */
  uint16_t getPointRing(size_t index) const
  {
    if (m_ring.empty()) return 0;
    ASSERT_LT_(index, m_ring.size());
    return m_ring[index];
  }
  /** Gets point time, or 0 if field is not present */
  float getPointTime(size_t index) const
  {
    if (m_time.empty()) return 0;
    ASSERT_LT_(index, m_time.size());
    return m_time[index];
  }

  /** Like \c getPointColor but without checking for out-of-index errors */
  inline float getPointIntensity_fast(size_t index) const { return m_intensity[index]; }

  /** Override of the default 3D scene builder to account for the individual
   * points' color.
   */
  void getVisualizationInto(mrpt::opengl::CSetOfObjects & outObj) const override;

  /** @name String-keyed field access virtual interface implementation
      @{ */
  bool hasPointField(const std::string_view& fieldName) const override;
  std::vector<std::string_view> getPointFieldNames_float() const override;
  std::vector<std::string_view> getPointFieldNames_uint16() const override;

  float getPointField_float(size_t index, const std::string_view& fieldName) const override;
  uint16_t getPointField_uint16(size_t index, const std::string_view& fieldName) const override;

  void setPointField_float(size_t index, const std::string_view& fieldName, float value) override;
  void setPointField_uint16(size_t index, const std::string_view& fieldName, uint16_t value)
      override;

  void insertPointField_float(const std::string_view& fieldName, float value) override;
  void insertPointField_uint16(const std::string_view& fieldName, uint16_t value) override;

  void reserveField_float(const std::string_view& fieldName, size_t n) override;
  void reserveField_uint16(const std::string_view& fieldName, size_t n) override;
  void resizeField_float(const std::string_view& fieldName, size_t n) override;
  void resizeField_uint16(const std::string_view& fieldName, size_t n) override;

  auto getPointsBufferRef_float_field(const std::string_view& fieldName)
      const->const mrpt::aligned_std_vector<float>* override
  {
    if (auto* f = CPointsMap::getPointsBufferRef_float_field(fieldName); f)
    {
      return f;
    }
    if (fieldName == POINT_FIELD_INTENSITY) return &m_intensity;
    if (fieldName == POINT_FIELD_TIMESTAMP) return &m_time;
    return nullptr;
  }
  auto getPointsBufferRef_uint16_field(const std::string_view& fieldName)
      const->const mrpt::aligned_std_vector<uint16_t>* override
  {
    if (fieldName == POINT_FIELD_RING_ID) return &m_ring;
    return nullptr;
  }

  auto getPointsBufferRef_float_field(const std::string_view& fieldName)
      ->mrpt::aligned_std_vector<float>* override
  {
    if (auto* f = CPointsMap::getPointsBufferRef_float_field(fieldName); f)
    {
      return f;
    }
    if (fieldName == POINT_FIELD_INTENSITY) return &m_intensity;
    if (fieldName == POINT_FIELD_TIMESTAMP) return &m_time;
    return nullptr;
  }
  auto getPointsBufferRef_uint16_field(const std::string_view& fieldName)
      ->mrpt::aligned_std_vector<uint16_t>* override
  {
    if (fieldName == POINT_FIELD_RING_ID) return &m_ring;
    return nullptr;
  }

  /** @} */

  void saveMetricMapRepresentationToFile(const std::string& filNamePrefix) const override
  {
    std::string fil(filNamePrefix + std::string(".txt"));
    saveXYZIRT_to_text_file(fil);
  }

 protected:
  /** The intensity/reflectance data */
  mrpt::aligned_std_vector<float> m_intensity;

  /** The ring data */
  mrpt::aligned_std_vector<uint16_t> m_ring;

  /** The time data (see description at the beginning of the class) */
  mrpt::aligned_std_vector<float> m_time;

  /** Clear the map, erasing all the points */
  void internal_clear() override;

  /** Redefinition to handle Velodyne Scan observations and generate per-point timestamps */
  bool internal_insertObservation(
      const mrpt::obs::CObservation& obs,
      const std::optional<const mrpt::poses::CPose3D>& robotPose) override;

  /** @name Redefinition of PLY Import virtual methods from CPointsMap
    @{ */
  void PLY_import_set_vertex(
      size_t idx, const mrpt::math::TPoint3Df& pt, const mrpt::img::TColorf* pt_color = nullptr)
      override;

  void PLY_import_set_vertex_count(size_t N) override;

  void PLY_import_set_vertex_timestamp(size_t idx, const double unixTimestamp) override
  {
    m_time.at(idx) = unixTimestamp;
  }

  /** @} */

  /** @name Redefinition of PLY Export virtual methods from CPointsMap
    @{ */
  void PLY_export_get_vertex(
      size_t idx, mrpt::math::TPoint3Df & pt, bool& pt_has_color, mrpt::img::TColorf& pt_color)
      const override;
  /** @} */

  MAP_DEFINITION_START(CPointsMapXYZIRT)
  mrpt::maps::CPointsMap::TInsertionOptions insertionOpts;
  mrpt::maps::CPointsMap::TLikelihoodOptions likelihoodOpts;
  MAP_DEFINITION_END(CPointsMapXYZIRT)

};  // End of class def.

}  // namespace mrpt::maps

namespace mrpt::opengl
{
template <>
class PointCloudAdapter<mrpt::maps::CPointsMapXYZIRT> :
    public PointCloudAdapter<mrpt::maps::CPointsMap>
{
 public:
  explicit PointCloudAdapter(const mrpt::maps::CPointsMapXYZIRT& pts) :
      PointCloudAdapter<mrpt::maps::CPointsMap>(pts)
  {
  }
};
}  // namespace mrpt::opengl
