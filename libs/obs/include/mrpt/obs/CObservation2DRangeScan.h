/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/containers/ContainerReadOnlyProxyAccessor.h>
#include <mrpt/core/aligned_std_vector.h>

// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM(mrpt::obs::CObservation2DRangeScan)

namespace mrpt
{
namespace obs
{
/** A "CObservation"-derived class that represents a 2D range scan measurement
 * (typically from a laser scanner).
 *  The data structures are generic enough to hold a wide variety of 2D
 * scanners and "3D" planar rotating 2D lasers.
 *
 *  These are the most important data fields:
 *    - These three fields are private data member (since MRPT 1.5.0) for
 * safety and to ensure data consistency. Read them with the
 * backwards-compatible proxies `scan`, `intensity`, `validRange` or (preferred)
 * with the new `get_*`, `set_*` and `resize()` methods:
 *      - CObservation2DRangeScan::scan -> A vector of float values with all
 * the range measurements (in meters).
 *      - CObservation2DRangeScan::validRange -> A vector (of <b>identical
 * size</b> to <i>scan<i>), has non-zeros for those ranges than are valid (i.e.
 * will be zero for non-reflected rays, etc.)
 *      - CObservation2DRangeScan::intensity -> A vector (of <b>identical
 * size</b> to <i>scan<i>) a unitless int values representing the relative
 * strength of each return. Higher values indicate a more intense return. This
 * is useful for filtering out low intensity(noisy) returns or detecting intense
 * landmarks.
 *    - CObservation2DRangeScan::aperture -> The field-of-view of the scanner,
 * in radians (typically, M_PI = 180deg).
 *    - CObservation2DRangeScan::sensorPose -> The 6D location of the sensor on
 * the robot reference frame (default=at the origin).
 *
 * \sa CObservation, CPointsMap, T2DScanProperties
 * \ingroup mrpt_obs_grp
 */
class CObservation2DRangeScan : public CObservation
{
	DEFINE_SERIALIZABLE(CObservation2DRangeScan)
	// This must be added for declaration of MEX-related functions
	DECLARE_MEX_CONVERSION
   private:
	/** The range values of the scan, in meters. Must have same length than \a
	 * validRange */
	mrpt::aligned_std_vector<float> m_scan;
	/** The intensity values of the scan. If available, must have same length
	 * than \a validRange */
	mrpt::aligned_std_vector<int32_t> m_intensity;
	/** It's false (=0) on no reflected rays, referenced to elements in \a scan
	 */
	mrpt::aligned_std_vector<char> m_validRange;
	/** Whether the intensity values are present or not. If not, space is saved
	 * during serialization. */
	bool m_has_intensity{false};

   public:
	/** Used in filterByExclusionAreas */
	using TListExclusionAreas = std::vector<mrpt::math::CPolygon>;
	/** Used in filterByExclusionAreas */
	using TListExclusionAreasWithRanges =
		std::vector<std::pair<mrpt::math::CPolygon, std::pair<double, double>>>;

	/** Default constructor */
	CObservation2DRangeScan() = default;
	/** copy ctor */
	CObservation2DRangeScan(const CObservation2DRangeScan& o);

	/** @name Scan data
		@{ */
	/** Resizes all data vectors to allocate a given number of scan rays */
	void resizeScan(const size_t len);
	/** Resizes all data vectors to allocate a given number of scan rays and
	 * assign default values. */
	void resizeScanAndAssign(
		const size_t len, const float rangeVal, const bool rangeValidity,
		const int32_t rangeIntensity = 0);
	/** Get number of scan rays */
	size_t getScanSize() const;

	/** The range values of the scan, in meters. Must have same length than \a
	 * validRange */
	mrpt::containers::ContainerReadOnlyProxyAccessor<
		mrpt::aligned_std_vector<float>>
		scan{m_scan};
	float getScanRange(const size_t i) const;
	void setScanRange(const size_t i, const float val);

	/** The intensity values of the scan. If available, must have same length
	 * than \a validRange */
	mrpt::containers::ContainerReadOnlyProxyAccessor<
		mrpt::aligned_std_vector<int32_t>>
		intensity{m_intensity};
	int32_t getScanIntensity(const size_t i) const;
	void setScanIntensity(const size_t i, const int val);

	/** It's false (=0) on no reflected rays, referenced to elements in \a scan
	 */
	mrpt::containers::ContainerReadOnlyProxyAccessor<
		mrpt::aligned_std_vector<char>>
		validRange{m_validRange};
	bool getScanRangeValidity(const size_t i) const;
	void setScanRangeValidity(const size_t i, const bool val);

	/** The "aperture" or field-of-view of the range finder, in radians
	 * (typically M_PI = 180 degrees). */
	float aperture{M_PIf};
	/** The scanning direction: true=counterclockwise; false=clockwise */
	bool rightToLeft{true};
	/** The maximum range allowed by the device, in meters (e.g. 80m, 50m,...)
	 */
	float maxRange{80.0f};
	/** The 6D pose of the sensor on the robot at the moment of starting the
	 * scan. */
	mrpt::poses::CPose3D sensorPose;
	/** The "sigma" error of the device in meters, used while inserting the scan
	 * in an occupancy grid. */
	float stdError{0.01f};
	/** The aperture of each beam, in radians, used to insert "thick" rays in
	 * the occupancy grid. */
	float beamAperture{0};
	/** If the laser gathers data by sweeping in the pitch/elevation angle, this
	 * holds the increment in "pitch" (=-"elevation") between the beginning and
	 * the end of the scan (the sensorPose member stands for the pose at the
	 * beginning of the scan). */
	double deltaPitch{0};

	/** Fill out a T2DScanProperties structure with the parameters of this scan
	 */
	void getScanProperties(T2DScanProperties& p) const;
	/** @} */

	void loadFromVectors(
		size_t nRays, const float* scanRanges, const char* scanValidity);

	/** @name Cached points map
		@{  */
   protected:
	/** A points map, build only under demand by the methods getAuxPointsMap()
	 * and buildAuxPointsMap().
	 *  It's a generic smart pointer to avoid depending here in the library
	 * mrpt-obs on classes on other libraries.
	 */
	mutable mrpt::maps::CMetricMap::Ptr m_cachedMap;
	/** Internal method, used from buildAuxPointsMap() */
	void internal_buildAuxPointsMap(const void* options = nullptr) const;

   public:
	/** Returns the cached points map representation of the scan, if already
	 * build with buildAuxPointsMap(), or nullptr otherwise.
	 * Usage:
	 *  \code
	 *    mrpt::maps::CPointsMap *map =
	 * obs->getAuxPointsMap<mrpt::maps::CPointsMap>();
	 *  \endcode
	 * \sa buildAuxPointsMap
	 */
	template <class POINTSMAP>
	inline const POINTSMAP* getAuxPointsMap() const
	{
		return static_cast<const POINTSMAP*>(m_cachedMap.get());
	}

	/** Returns a cached points map representing this laser scan, building it
	 * upon the first call.
	 * \param options Can be nullptr to use default point maps' insertion
	 * options, or a pointer to a "CPointsMap::TInsertionOptions" structure to
	 * override some params.
	 * Usage:
	 *  \code
	 *    mrpt::maps::CPointsMap *map =
	 * obs->buildAuxPointsMap<mrpt::maps::CPointsMap>(&options or nullptr);
	 *  \endcode
	 * \sa getAuxPointsMap
	 */
	template <class POINTSMAP>
	inline const POINTSMAP* buildAuxPointsMap(
		const void* options = nullptr) const
	{
		if (!m_cachedMap) internal_buildAuxPointsMap(options);
		return static_cast<const POINTSMAP*>(m_cachedMap.get());
	}

	/** @} */

	/** Return true if the laser scanner is "horizontal", so it has an absolute
	 * value of "pitch" and "roll" less or equal to the given tolerance (in
	 * rads, default=0) (with the normal vector either upwards or downwards).
	 */
	bool isPlanarScan(const double tolerance = 0) const;

	/** Return true if scan has intensity */
	bool hasIntensity() const;
	/** Marks this scan as having or not intensity data. */
	void setScanHasIntensity(bool setHasIntensityFlag);

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = sensorPose;
	}
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		sensorPose = newSensorPose;
	}
	void getDescriptionAsText(std::ostream& o) const override;

	/** A general method to truncate the scan by defining a minimum valid
	   distance and a maximum valid angle as well as minimun and maximum heights
	   (NOTE: the laser z-coordinate must be provided).
	  */
	void truncateByDistanceAndAngle(
		float min_distance, float max_angle, float min_height = 0,
		float max_height = 0, float h = 0);

	/** Mark as invalid sensed points that fall within any of a set of
	 * "exclusion areas", given in coordinates relative to the vehicle (taking
	 * into account "sensorPose").
	 * \sa C2DRangeFinderAbstract::loadExclusionAreas
	 */
	void filterByExclusionAreas(const TListExclusionAreas& areas);

	/** Mark as invalid sensed points that fall within any of a set of
	 * "exclusion areas", given in coordinates relative to the vehicle (taking
	 * into account "sensorPose"), AND such as the Z coordinate of the point
	 * falls in the range [min,max] associated to each exclusion polygon.
	 * \sa C2DRangeFinderAbstract::loadExclusionAreas
	 */
	void filterByExclusionAreas(const TListExclusionAreasWithRanges& areas);

	/** Mark as invalid the ranges in any of a given set of "forbiden angle
	 * ranges", given as pairs<min_angle,max_angle>.
	 * \sa C2DRangeFinderAbstract::loadExclusionAreas
	 */
	void filterByExclusionAngles(
		const std::vector<std::pair<double, double>>& angles);

};  // End of class def.

}  // namespace obs
namespace typemeta
{
// Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_PTR_NAMESPACE(CObservation2DRangeScan, ::mrpt::obs)
}  // namespace typemeta

}  // namespace mrpt
