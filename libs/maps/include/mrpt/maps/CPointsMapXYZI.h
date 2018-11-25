/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mrpt
{
namespace maps
{
/** A map of 3D points with reflectance/intensity (float).
 * \sa mrpt::maps::CPointsMap, mrpt::maps::CMetricMap
 * \ingroup mrpt_maps_grp
 */
class CPointsMapXYZI : public CPointsMap
{
	DEFINE_SERIALIZABLE(CPointsMapXYZI)

   public:
	// --------------------------------------------
	/** @name Pure virtual interfaces to be implemented by any class derived
	   from CPointsMap
		@{ */

	void reserve(size_t newLength) override;  // See base class docs
	void resize(size_t newLength) override;  // See base class docs
	void setSize(size_t newLength) override;  // See base class docs

	/** The virtual method for \a insertPoint() *without* calling
	 * mark_as_modified()   */
	void insertPointFast(float x, float y, float z = 0) override;

	/** Get all the data fields for one point as a vector: [X Y Z I]
	 *  Unlike getPointAllFields(), this method does not check for index out of
	 * bounds
	 * \sa getPointAllFields, setPointAllFields, setPointAllFieldsFast
	 */
	void getPointAllFieldsFast(
		const size_t index, std::vector<float>& point_data) const override
	{
		point_data.resize(4);
		point_data[0] = m_x[index];
		point_data[1] = m_y[index];
		point_data[2] = m_z[index];
		point_data[3] = m_intensity[index];
	}

	/** Set all the data fields for one point as a vector: [X Y Z I]
	 *  Unlike setPointAllFields(), this method does not check for index out of
	 * bounds
	 * \sa setPointAllFields, getPointAllFields, getPointAllFieldsFast
	 */
	void setPointAllFieldsFast(
		const size_t index, const std::vector<float>& point_data) override
	{
		ASSERT_(point_data.size() == 4);
		m_x[index] = point_data[0];
		m_y[index] = point_data[1];
		m_z[index] = point_data[2];
		m_intensity[index] = point_data[3];
	}

	/** Loads from a Kitti dataset Velodyne scan binary file.
	 * The file can be gz compressed.
	 * \return true on success */
	bool loadFromKittiVelodyneFile(const std::string& filename);

	/** See CPointsMap::loadFromRangeScan() */
	void loadFromRangeScan(
		const mrpt::obs::CObservation2DRangeScan& rangeScan,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;
	/** See CPointsMap::loadFromRangeScan() */
	void loadFromRangeScan(
		const mrpt::obs::CObservation3DRangeScan& rangeScan,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;

   protected:
	// See base class
	void impl_copyFrom(const CPointsMap& obj) override;
	// See base class
	void addFrom_classSpecific(
		const CPointsMap& anotherMap, const size_t nPreviousPoints) override;

	// Friend methods:
	template <class Derived>
	friend struct detail::loadFromRangeImpl;
	template <class Derived>
	friend struct detail::pointmap_traits;

   public:
	/** @} */
	// --------------------------------------------

	/** Save to a text file. In each line contains X Y Z (meters) I (intensity)
	 * Returns false if any error occured, true elsewere.
	 */
	bool saveXYZI_to_text_file(const std::string& file) const;

	/** Changes a given point from map. First index is 0.
	 * \exception Throws std::exception on index out of bound.
	 */
	void setPointRGB(
		size_t index, float x, float y, float z, float R_intensity,
		float G_ignored, float B_ignored) override;

	/** Adds a new point given its coordinates and color (colors range is [0,1])
	 */
	void insertPointRGB(
		float x, float y, float z, float R_intensity, float G_ignored,
		float B_ignored) override;

	/** Changes the intensity of a given point from the map. First index is 0.
	 * \exception Throws std::exception on index out of bound.
	 */
	void setPointIntensity(size_t index, float intensity);

	/** Like \c setPointColor but without checking for out-of-index erors */
	inline void setPointColor_fast(size_t index, float R, float G, float B)
	{
		m_intensity[index] = R;
	}

	/** Retrieves a point and its color (colors range is [0,1])
	 */
	void getPointRGB(
		size_t index, float& x, float& y, float& z, float& R_intensity,
		float& G_intensity, float& B_intensity) const override;

	/** Retrieves a point intensity (range [0,1]) */
	float getPointIntensity(size_t index) const;

	/** Like \c getPointColor but without checking for out-of-index erors */
	inline float getPointIntensity_fast(size_t index) const
	{
		return m_intensity[index];
	}

	/** Returns true if the point map has a color field for each point */
	bool hasColorPoints() const override { return true; }

	/** Override of the default 3D scene builder to account for the individual
	 * points' color.
	 */
	void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

	/** @name PCL library support
		@{ */

	/** Save the point cloud as a PCL PCD file, in either ASCII or binary format
	 * \return false on any error */
	bool savePCDFile(
		const std::string& filename, bool save_as_binary) const override;

	/** Loads a PCL point cloud (WITH XYZI information) into this MRPT class.
	 *  Usage example:
	 *  \code
	 *    pcl::PointCloud<pcl::PointXYZI> cloud;
	 *    mrpt::maps::CPointsMapXYZI       pc;
	 *
	 *    pc.setFromPCLPointCloudXYZI(cloud);
	 *  \endcode
	 * \sa CPointsMap::setFromPCLPointCloud()
	 */
	template <class POINTCLOUD>
	void setFromPCLPointCloudXYZI(const POINTCLOUD& cloud)
	{
		const size_t N = cloud.points.size();
		clear();
		reserve(N);
		for (size_t i = 0; i < N; ++i)
			this->insertPoint(
				cloud.points[i].x, cloud.points[i].y, cloud.points[i].z,
				cloud.points[i].intensity);
	}

	/** Like CPointsMap::getPCLPointCloud() but for PointCloud<PointXYZI> */
	template <class POINTCLOUD>
	void getPCLPointCloudXYZI(POINTCLOUD& cloud) const
	{
		const size_t nThis = this->size();
		this->getPCLPointCloud(cloud);  // 1st: xyz data
		// 2nd: I data
		for (size_t i = 0; i < nThis; ++i)
			cloud.points[i].intensity = m_intensity[i];
	}
	/** @} */

   protected:
	/** The intensity/reflectance data */
	mrpt::aligned_std_vector<float> m_intensity;

	/** Clear the map, erasing all the points */
	void internal_clear() override;

	/** @name Redefinition of PLY Import virtual methods from CPointsMap
		@{ */
	void PLY_import_set_vertex(
		const size_t idx, const mrpt::math::TPoint3Df& pt,
		const mrpt::img::TColorf* pt_color = nullptr) override;

	void PLY_import_set_vertex_count(const size_t N) override;
	/** @} */

	/** @name Redefinition of PLY Export virtual methods from CPointsMap
		@{ */
	void PLY_export_get_vertex(
		const size_t idx, mrpt::math::TPoint3Df& pt, bool& pt_has_color,
		mrpt::img::TColorf& pt_color) const override;
	/** @} */

	MAP_DEFINITION_START(CPointsMapXYZI)
	mrpt::maps::CPointsMap::TInsertionOptions insertionOpts;
	mrpt::maps::CPointsMap::TLikelihoodOptions likelihoodOpts;
	MAP_DEFINITION_END(CPointsMapXYZI)

};  // End of class def.

}  // namespace maps

#include <mrpt/opengl/pointcloud_adapters.h>
namespace opengl
{
/** Specialization
 * mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI> \ingroup
 * mrpt_adapters_grp */
template <>
class PointCloudAdapter<mrpt::maps::CPointsMapXYZI>
{
   private:
	mrpt::maps::CPointsMapXYZI& m_obj;

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
	inline PointCloudAdapter(const mrpt::maps::CPointsMapXYZI& obj)
		: m_obj(*const_cast<mrpt::maps::CPointsMapXYZI*>(&obj))
	{
	}
	/** Get number of points */
	inline size_t size() const { return m_obj.size(); }
	/** Set number of points (to uninitialized values) */
	inline void resize(const size_t N) { m_obj.resize(N); }
	/** Does nothing as of now */
	inline void setDimensions(const size_t& height, const size_t& width) {}
	/** Get XYZ coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ(const size_t idx, T& x, T& y, T& z) const
	{
		m_obj.getPointFast(idx, x, y, z);
	}
	/** Set XYZ coordinates of i'th point */
	inline void setPointXYZ(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		m_obj.setPointFast(idx, x, y, z);
	}

	/** Get XYZ_RGBf coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBf(
		const size_t idx, T& x, T& y, T& z, float& r, float& g, float& b) const
	{
		m_obj.getPointRGB(idx, x, y, z, r, g, b);
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointXYZ_RGBf(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const float r, const float g, const float b)
	{
		m_obj.setPointRGB(idx, x, y, z, r, g, b);
	}

	/** Get XYZ_RGBu8 coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBu8(
		const size_t idx, T& x, T& y, T& z, uint8_t& r, uint8_t& g,
		uint8_t& b) const
	{
		float I, Gignrd, Bignrd;
		m_obj.getPoint(idx, x, y, z, I, Gignrd, Bignrd);
		r = g = b = I * 255;
	}
	/** Set XYZ_RGBu8 coordinates of i'th point */
	inline void setPointXYZ_RGBu8(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const uint8_t r, const uint8_t g, const uint8_t b)
	{
		m_obj.setPointRGB(idx, x, y, z, r / 255.f, g / 255.f, b / 255.f);
	}

	/** Get RGBf color of i'th point */
	inline void getPointRGBf(
		const size_t idx, float& r, float& g, float& b) const
	{
		r = g = b = m_obj.getPointIntensity_fast(idx);
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointRGBf(
		const size_t idx, const float r, const float g, const float b)
	{
		m_obj.setPointColor_fast(idx, r, g, b);
	}

	/** Get RGBu8 color of i'th point */
	inline void getPointRGBu8(
		const size_t idx, uint8_t& r, uint8_t& g, uint8_t& b) const
	{
		float i = m_obj.getPointIntensity_fast(idx);
		r = g = b = i * 255;
	}
	/** Set RGBu8 coordinates of i'th point */
	inline void setPointRGBu8(
		const size_t idx, const uint8_t r, const uint8_t g, const uint8_t b)
	{
		m_obj.setPointColor_fast(idx, r / 255.f, g / 255.f, b / 255.f);
	}

};  // end of PointCloudAdapter<mrpt::maps::CPointsMapXYZI>
}  // namespace opengl
}  // namespace mrpt
