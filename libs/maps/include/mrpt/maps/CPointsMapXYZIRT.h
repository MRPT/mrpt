/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
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
class CPointsMapXYZI;

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
 * zeros, but you can check their validity with:
 *  - `hasIntensityField()`
 *  - `hasRingField()`
 *  - `hasTimeField()`
 *
 * \sa mrpt::maps::CPointsMap, mrpt::maps::CMetricMap
 * \ingroup mrpt_maps_grp
 */
class CPointsMapXYZIRT : public CPointsMap
{
  DEFINE_SERIALIZABLE(CPointsMapXYZIRT, mrpt::maps)

 public:
  CPointsMapXYZIRT() = default;

  CPointsMapXYZIRT(const CPointsMap& o) { CPointsMap::operator=(o); }
  CPointsMapXYZIRT(const CPointsMapXYZIRT& o);
  explicit CPointsMapXYZIRT(const CPointsMapXYZI& o);
  CPointsMapXYZIRT& operator=(const CPointsMap& o);
  CPointsMapXYZIRT& operator=(const CPointsMapXYZIRT& o);
  CPointsMapXYZIRT& operator=(const CPointsMapXYZI& o);

  /** @name Pure virtual interfaces to be implemented by any class derived
 from CPointsMap
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

  /** The virtual method for \a insertPoint() *without* calling
   * mark_as_modified()   */
  void insertPointFast(float x, float y, float z = 0) override;

  /** Get all the data fields for one point as a vector: [X Y Z I]
   *  Unlike getPointAllFields(), this method does not check for index out of
   * bounds
   * \sa getPointAllFields, setPointAllFields, setPointAllFieldsFast
   */
  void getPointAllFieldsFast(size_t index, std::vector<float>& point_data) const override
  {
    point_data.resize(6);
    point_data[0] = m_x[index];
    point_data[1] = m_y[index];
    point_data[2] = m_z[index];
    point_data[3] = hasIntensityField() ? m_intensity[index] : 0;
    point_data[4] = hasRingField() ? m_ring[index] : 0;
    point_data[5] = hasTimeField() ? m_time[index] : 0;
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
    if (hasRingField()) m_ring[index] = point_data[4];
    if (hasTimeField()) m_time[index] = point_data[5];
  }

  /** See CPointsMap::loadFromRangeScan() */
  void loadFromRangeScan(
      const mrpt::obs::CObservation2DRangeScan& rangeScan,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;
  /** See CPointsMap::loadFromRangeScan() */
  void loadFromRangeScan(
      const mrpt::obs::CObservation3DRangeScan& rangeScan,
      const std::optional<const mrpt::poses::CPose3D>& robotPose = std::nullopt) override;

 protected:
  // See base class
  void addFrom_classSpecific(
      const CPointsMap& anotherMap,
      size_t nPreviousPoints,
      const bool filterOutPointsAtZero) override;

  // Friend methods:
  template <class Derived>
  friend struct detail::loadFromRangeImpl;
  template <class Derived>
  friend struct detail::pointmap_traits;

 public:
  /** @} */

  /** Save to a text file. In each line contains `X Y Z I R T`
   * Returns false if any error occured, true elsewere.
   */
  bool saveXYZIRT_to_text_file(const std::string& file) const;

  /** Loads from a text file, each line having "X Y Z I", I in [0,1].
   * Returns false if any error occured, true elsewere. */
  bool loadXYZIRT_from_text_file(const std::string& file);

  /** Changes a given point from map. First index is 0.
   * \exception Throws std::exception on index out of bound.
   */
  void setPointRGB(
      size_t index, float x, float y, float z, float R_intensity, float G_ignored, float B_ignored)
      override;

  /** Adds a new point given its coordinates and color (colors range is [0,1])
   */
  void insertPointRGB(
      float x, float y, float z, float R_intensity, float G_ignored, float B_ignored) override;

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

  /** Like \c setPointColor but without checking for out-of-index erors */
  inline void setPointColor_fast(size_t index, float R, float G, float B)
  {
    m_intensity[index] = R;
  }

  /** Retrieves a point and its color (colors range is [0,1])
   */
  void getPointRGB(
      size_t index,
      float& x,
      float& y,
      float& z,
      float& R_intensity,
      float& G_intensity,
      float& B_intensity) const override;

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

  /** Like \c getPointColor but without checking for out-of-index erors */
  inline float getPointIntensity_fast(size_t index) const { return m_intensity[index]; }

  /** Returns true if the point map has a color field for each point */
  bool hasColorPoints() const override { return true; }

  /** Override of the default 3D scene builder to account for the individual
   * points' color.
   */
  void getVisualizationInto(mrpt::opengl::CSetOfObjects& outObj) const override;

  // clang-format off
	auto getPointsBufferRef_intensity() const  -> const mrpt::aligned_std_vector<float>* override { return &m_intensity; }
	auto getPointsBufferRef_ring() const       -> const mrpt::aligned_std_vector<uint16_t>* override { return &m_ring; }
	auto getPointsBufferRef_timestamp() const  -> const mrpt::aligned_std_vector<float>* override { return &m_time; }

	auto getPointsBufferRef_intensity()        -> mrpt::aligned_std_vector<float>* override { return &m_intensity; }
	auto getPointsBufferRef_ring()             -> mrpt::aligned_std_vector<uint16_t>* override { return &m_ring; }
	auto getPointsBufferRef_timestamp()        -> mrpt::aligned_std_vector<float>* override { return  &m_time; }

	void insertPointField_Intensity(float i) override { m_intensity.push_back(i); }
	void insertPointField_Ring(uint16_t r)   override { m_ring.push_back(r); }
	void insertPointField_Timestamp(float t) override { m_time.push_back(t); }
	/// clang-format on

	void saveMetricMapRepresentationToFile(
		const std::string& filNamePrefix) const override
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
		const std::optional<const mrpt::poses::CPose3D>& robotPose =
			std::nullopt) override;


	/** @name Redefinition of PLY Import virtual methods from CPointsMap
		@{ */
	void PLY_import_set_vertex(
		size_t idx, const mrpt::math::TPoint3Df& pt,
		const mrpt::img::TColorf* pt_color = nullptr) override;

	void PLY_import_set_vertex_count(size_t N) override;

	void PLY_import_set_vertex_timestamp(
		size_t idx, const double unixTimestamp) override
	{
		m_time.at(idx) = unixTimestamp;
	}

	/** @} */

	/** @name Redefinition of PLY Export virtual methods from CPointsMap
		@{ */
	void PLY_export_get_vertex(
		size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color,
		mrpt::img::TColorf& pt_color) const override;
	/** @} */

	MAP_DEFINITION_START(CPointsMapXYZIRT)
	mrpt::maps::CPointsMap::TInsertionOptions insertionOpts;
	mrpt::maps::CPointsMap::TLikelihoodOptions likelihoodOpts;
	MAP_DEFINITION_END(CPointsMapXYZIRT)

};	// End of class def.

}  // namespace mrpt::maps

namespace mrpt::opengl
{
/** Specialization
 * mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZIRT>
 * \ingroup mrpt_adapters_grp */
template <>
class PointCloudAdapter<mrpt::maps::CPointsMapXYZIRT>
{
   private:
	mrpt::maps::CPointsMapXYZIRT& m_obj;

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
	inline PointCloudAdapter(const mrpt::maps::CPointsMapXYZIRT& obj)
		: m_obj(*const_cast<mrpt::maps::CPointsMapXYZIRT*>(&obj))
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
	inline void setPointXYZ(
		size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		m_obj.setPointFast(idx, x, y, z);
	}

	/** Get XYZ_RGBf coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBAf(
		size_t idx, T& x, T& y, T& z, float& r, float& g, float& b,
		float& a) const
	{
		m_obj.getPointRGB(idx, x, y, z, r, g, b);
		a = 1.0f;
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointXYZ_RGBAf(
		size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const float r, const float g, const float b,
		[[maybe_unused]] const float a)
	{
		m_obj.setPointRGB(idx, x, y, z, r, g, b);
	}

	/** Get XYZ_RGBu8 coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBu8(
		size_t idx, T& x, T& y, T& z, uint8_t& r, uint8_t& g, uint8_t& b) const
	{
		float I, Gignrd, Bignrd;
		m_obj.getPoint(idx, x, y, z, I, Gignrd, Bignrd);
		r = g = b = I * 255;
	}
	/** Set XYZ_RGBu8 coordinates of i'th point */
	inline void setPointXYZ_RGBu8(
		size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const uint8_t r, const uint8_t g, const uint8_t b)
	{
		m_obj.setPointRGB(idx, x, y, z, r / 255.f, g / 255.f, b / 255.f);
	}

	/** Get RGBf color of i'th point */
	inline void getPointRGBf(size_t idx, float& r, float& g, float& b) const
	{
		r = g = b = m_obj.getPointIntensity_fast(idx);
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointRGBf(
		size_t idx, const float r, const float g, const float b)
	{
		m_obj.setPointColor_fast(idx, r, g, b);
	}

	/** Get RGBu8 color of i'th point */
	inline void getPointRGBu8(
		size_t idx, uint8_t& r, uint8_t& g, uint8_t& b) const
	{
		float i = m_obj.getPointIntensity_fast(idx);
		r = g = b = i * 255;
	}
	/** Set RGBu8 coordinates of i'th point */
	inline void setPointRGBu8(
		size_t idx, const uint8_t r, const uint8_t g, const uint8_t b)
	{
		m_obj.setPointColor_fast(idx, r / 255.f, g / 255.f, b / 255.f);
	}

};	// end of PointCloudAdapter<mrpt::maps::CPointsMapXYZIRT>
}  // namespace mrpt::opengl
