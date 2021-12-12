/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt
{
namespace maps
{
/** A cloud of points in 2D or 3D, which can be built from a sequence of laser
 * scans.
 *    This class stores the coordinates (x,y,z) and a "weight", or counter of
 * how many times that point has been seen, used only if points fusion is
 * enabled in the options structure.
 * \sa CMetricMap, CPoint, mrpt::serialization::CSerializable, CSimplePointsMap
 * \ingroup mrpt_maps_grp
 */
class CWeightedPointsMap : public CPointsMap
{
	DEFINE_SERIALIZABLE(CWeightedPointsMap, mrpt::maps)

   public:
	/** Default constructor */
	CWeightedPointsMap() = default;
	CWeightedPointsMap(const CPointsMap& o) { impl_copyFrom(o); }
	CWeightedPointsMap(const CWeightedPointsMap& o) : CPointsMap()
	{
		impl_copyFrom(o);
	}
	CWeightedPointsMap& operator=(const CPointsMap& o)
	{
		impl_copyFrom(o);
		return *this;
	}
	CWeightedPointsMap& operator=(const CWeightedPointsMap& o)
	{
		impl_copyFrom(o);
		return *this;
	}

	// --------------------------------------------
	/** @name Pure virtual interfaces to be implemented by any class derived
	   from CPointsMap
		@{ */
	void reserve(size_t newLength) override;  // See base class docs
	void resize(size_t newLength) override;	 // See base class docs
	void setSize(size_t newLength) override;  // See base class docs

	/** The virtual method for \a insertPoint() *without* calling
	 * mark_as_modified()   */
	void insertPointFast(float x, float y, float z = 0) override;

	/** Get all the data fields for one point as a vector: [X Y Z WEIGHT]
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
		point_data[3] = pointWeight[index];
	}

	/** Set all the data fields for one point as a vector: [X Y Z WEIGHT]
	 *  Unlike setPointAllFields(), this method does not check for index out of
	 * bounds
	 * \sa setPointAllFields, getPointAllFields, getPointAllFieldsFast
	 */
	void setPointAllFieldsFast(
		const size_t index, const std::vector<float>& point_data) override
	{
		ASSERTDEB_(point_data.size() == 4);
		m_x[index] = point_data[0];
		m_y[index] = point_data[1];
		m_z[index] = point_data[2];
		pointWeight[index] = point_data[3];
	}

	/** See CPointsMap::loadFromRangeScan() */
	void loadFromRangeScan(
		const mrpt::obs::CObservation2DRangeScan& rangeScan,
		const std::optional<const mrpt::poses::CPose3D>& robotPose =
			std::nullopt) override;

	/** See CPointsMap::loadFromRangeScan() */
	void loadFromRangeScan(
		const mrpt::obs::CObservation3DRangeScan& rangeScan,
		const std::optional<const mrpt::poses::CPose3D>& robotPose =
			std::nullopt) override;

   protected:
	void impl_copyFrom(const CPointsMap& obj) override;
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

	/// Sets the point weight, which is ignored in all classes but those which
	/// actually store that field (Note: No checks are done for out-of-bounds
	/// index). \sa getPointWeight
	void setPointWeight(size_t index, unsigned long w) override
	{
		pointWeight[index] = w;
	}
	/// Gets the point weight, which is ignored in all classes (defaults to 1)
	/// but in those which actually store that field (Note: No checks are done
	/// for out-of-bounds index).  \sa setPointWeight
	unsigned int getPointWeight(size_t index) const override
	{
		return pointWeight[index];
	}

   protected:
	/** The points weights */
	mrpt::aligned_std_vector<uint32_t> pointWeight;

	/** Clear the map, erasing all the points.
	 */
	void internal_clear() override;

   protected:
	/** @name PLY Import virtual methods to implement in base classes
		@{ */
	/** In a base class, reserve memory to prepare subsequent calls to
	 * PLY_import_set_vertex */
	void PLY_import_set_vertex_count(const size_t N) override;
	/** @} */

	MAP_DEFINITION_START(CWeightedPointsMap)
	/** Observations insertion options */
	mrpt::maps::CPointsMap::TInsertionOptions insertionOpts;
	/** Probabilistic observation likelihood options */
	mrpt::maps::CPointsMap::TLikelihoodOptions likelihoodOpts;
	MAP_DEFINITION_END(CWeightedPointsMap)
};	// End of class def.
}  // namespace maps

namespace opengl
{
/** Specialization
 * mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>
 * \ingroup mrpt_adapters_grp */
template <>
class PointCloudAdapter<mrpt::maps::CWeightedPointsMap>
{
   private:
	mrpt::maps::CWeightedPointsMap& m_obj;

   public:
	/** The type of each point XYZ coordinates */
	using coords_t = float;
	/** Has any color RGB info? */
	static constexpr bool HAS_RGB = false;
	/** Has native RGB info (as floats)? */
	static constexpr bool HAS_RGBf = false;
	/** Has native RGB info (as uint8_t)? */
	static constexpr bool HAS_RGBu8 = false;

	/** Constructor (accept a const ref for convenience) */
	inline PointCloudAdapter(const mrpt::maps::CWeightedPointsMap& obj)
		: m_obj(*const_cast<mrpt::maps::CWeightedPointsMap*>(&obj))
	{
	}
	/** Get number of points */
	inline size_t size() const { return m_obj.size(); }
	/** Set number of points (to uninitialized values) */
	inline void resize(const size_t N) { m_obj.resize(N); }
	/** Does nothing as of now */
	inline void setDimensions(size_t height, size_t width) {}
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
};	// end of PointCloudAdapter<mrpt::maps::CPointsMap>
}  // namespace opengl
}  // namespace mrpt
