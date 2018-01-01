/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CSimplePointsMap_H
#define CSimplePointsMap_H

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt
{
namespace maps
{
/** A cloud of points in 2D or 3D, which can be built from a sequence of laser
 * scans.
 *    This class only stores the coordinates (x,y,z) of each point.
 *
 *  See mrpt::maps::CPointsMap and derived classes for other point cloud
 * classes.
 *
 * \sa CMetricMap, CWeightedPointsMap, CPoint, mrpt::serialization::CSerializable
 * \ingroup mrpt_maps_grp
 */
class CSimplePointsMap : public CPointsMap
{
	DEFINE_SERIALIZABLE(CSimplePointsMap)

   public:
	/** Default constructor */
	CSimplePointsMap();
	/** Destructor */
	virtual ~CSimplePointsMap();

	// --------------------------------------------
	/** @name Pure virtual interfaces to be implemented by any class derived
	   from CPointsMap
		@{ */
	virtual void reserve(size_t newLength) override;  // See base class docs
	virtual void resize(size_t newLength) override;  // See base class docs
	virtual void setSize(size_t newLength) override;  // See base class docs
	/** Changes the coordinates of the given point (0-based index), *without*
	 * checking for out-of-bounds and *without* calling mark_as_modified()  \sa
	 * setPoint */
	virtual void setPointFast(size_t index, float x, float y, float z) override;
	/** The virtual method for \a insertPoint() *without* calling
	 * mark_as_modified()   */
	virtual void insertPointFast(float x, float y, float z = 0) override;
	/** Virtual assignment operator, to be implemented in derived classes  */
	virtual void copyFrom(const CPointsMap& obj) override;
	/** Get all the data fields for one point as a vector: [X Y Z]
	  *  Unlike getPointAllFields(), this method does not check for index out of
	 * bounds
	  * \sa getPointAllFields, setPointAllFields, setPointAllFieldsFast
	  */
	virtual void getPointAllFieldsFast(
		const size_t index, std::vector<float>& point_data) const override
	{
		point_data.resize(3);
		point_data[0] = x[index];
		point_data[1] = y[index];
		point_data[2] = z[index];
	}
	/** Set all the data fields for one point as a vector: [X Y Z]
	  *  Unlike setPointAllFields(), this method does not check for index out of
	 * bounds
	  * \sa setPointAllFields, getPointAllFields, getPointAllFieldsFast
	  */
	virtual void setPointAllFieldsFast(
		const size_t index, const std::vector<float>& point_data) override
	{
		ASSERTDEB_(point_data.size() == 3)
		x[index] = point_data[0];
		y[index] = point_data[1];
		z[index] = point_data[2];
	}

	// See CPointsMap::loadFromRangeScan()
	virtual void loadFromRangeScan(
		const mrpt::obs::CObservation2DRangeScan& rangeScan,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;
	// See CPointsMap::loadFromRangeScan()
	virtual void loadFromRangeScan(
		const mrpt::obs::CObservation3DRangeScan& rangeScan,
		const mrpt::poses::CPose3D* robotPose = nullptr) override;

   protected:
	/** Auxiliary method called from within \a addFrom() automatically, to
	 * finish the copying of class-specific data  */
	virtual void addFrom_classSpecific(
		const CPointsMap& anotherMap, const size_t nPreviousPoints) override
	{
		MRPT_UNUSED_PARAM(anotherMap);
		MRPT_UNUSED_PARAM(nPreviousPoints);
		// No extra data.
	}

	// Friend methods:
	template <class Derived>
	friend struct detail::loadFromRangeImpl;
	template <class Derived>
	friend struct detail::pointmap_traits;

   public:
	/** @} */
	// --------------------------------------------

	/** If the map is a simple points map or it's a multi-metric map that
	 * contains EXACTLY one simple points map, return it.
		* Otherwise, return NULL
		*/
	virtual const mrpt::maps::CSimplePointsMap* getAsSimplePointsMap()
		const override
	{
		return this;
	}
	virtual mrpt::maps::CSimplePointsMap* getAsSimplePointsMap() override
	{
		return this;
	}

   protected:
	/** Clear the map, erasing all the points.
	 */
	virtual void internal_clear() override;

	/** @name PLY Import virtual methods to implement in base classes
		@{ */
	/** In a base class, reserve memory to prepare subsequent calls to
	 * PLY_import_set_vertex */
	virtual void PLY_import_set_vertex_count(const size_t N) override;
	/** @} */

	MAP_DEFINITION_START(CSimplePointsMap)
	/** Observations insertion options */
	mrpt::maps::CPointsMap::TInsertionOptions insertionOpts;
	/** Probabilistic observation likelihood options */
	mrpt::maps::CPointsMap::TLikelihoodOptions likelihoodOpts;
	MAP_DEFINITION_END(CSimplePointsMap, )

};  // End of class def.
}  // End of namespace

namespace utils
{
/** Specialization mrpt::opengl::PointCloudAdapter<mrpt::maps::CSimplePointsMap>
 * \ingroup mrpt_adapters_grp*/
template <>
class PointCloudAdapter<mrpt::maps::CSimplePointsMap>
	: public detail::PointCloudAdapterHelperNoRGB<mrpt::maps::CSimplePointsMap,
												  float>
{
   private:
	mrpt::maps::CSimplePointsMap& m_obj;

   public:
	/** The type of each point XYZ coordinates */
	typedef float coords_t;
	/** Has any color RGB info? */
	static const int HAS_RGB = 0;
	/** Has native RGB info (as floats)? */
	static const int HAS_RGBf = 0;
	/** Has native RGB info (as uint8_t)? */
	static const int HAS_RGBu8 = 0;

	/** Constructor (accept a const ref for convenience) */
	inline PointCloudAdapter(const mrpt::maps::CSimplePointsMap& obj)
		: m_obj(*const_cast<mrpt::maps::CSimplePointsMap*>(&obj))
	{
	}
	/** Get number of points */
	inline size_t size() const { return m_obj.size(); }
	/** Set number of points (to uninitialized values) */
	inline void resize(const size_t N) { m_obj.resize(N); }
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

	/** Set XYZ coordinates of i'th point */
	inline void setInvalidPoint(const size_t idx)
	{
		THROW_EXCEPTION("mrpt::maps::CSimplePointsMap needs to be dense");
	}
};  // end of PointCloudAdapter<mrpt::maps::CPointsMap>
}

}  // End of namespace

#endif
