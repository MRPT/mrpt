/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservation2DRangeScan_H
#define CObservation2DRangeScan_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/utils/ContainerReadOnlyProxyAccessor.h>

// Add for declaration of mexplus::from template specialization
DECLARE_MEXPLUS_FROM( mrpt::obs::CObservation2DRangeScan )

namespace mrpt
{
namespace obs
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservation2DRangeScan, CObservation, OBS_IMPEXP)

	/** A "CObservation"-derived class that represents a 2D range scan measurement (typically from a laser scanner).
	  *  The data structures are generic enough to hold a wide variety of 2D scanners and "3D" planar rotating 2D lasers.
	  *
	  *  These are the most important data fields:
	  *    - These three fields are private data member (since MRPT 1.5.0) for safety and to ensure data consistency. Read them with the backwards-compatible proxies `scan`, `intensity`, `validRange` or (preferred) with the new `get_*`, `set_*` and `resize()` methods:
	  *      - CObservation2DRangeScan::scan -> A vector of float values with all the range measurements (in meters).
	  *      - CObservation2DRangeScan::validRange -> A vector (of <b>identical size</b> to <i>scan<i>), has non-zeros for those ranges than are valid (i.e. will be zero for non-reflected rays, etc.)
	  *      - CObservation2DRangeScan::intensity -> A vector (of <b>identical size</b> to <i>scan<i>) a unitless int values representing the relative strength of each return. Higher values indicate a more intense return. This is useful for filtering out low intensity(noisy) returns or detecting intense landmarks.
	  *    - CObservation2DRangeScan::aperture -> The field-of-view of the scanner, in radians (typically, M_PI = 180deg).
	  *    - CObservation2DRangeScan::sensorPose -> The 6D location of the sensor on the robot reference frame (default=at the origin).
	  *
	  * \sa CObservation, CPointsMap, T2DScanProperties
	  * \ingroup mrpt_obs_grp
	  */
	class OBS_IMPEXP CObservation2DRangeScan : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservation2DRangeScan )
		// This must be added for declaration of MEX-related functions
		DECLARE_MEX_CONVERSION
	private:
		std::vector<float>   m_scan; //!< The range values of the scan, in meters. Must have same length than \a validRange
		std::vector<int32_t> m_intensity; //!< The intensity values of the scan. If available, must have same length than \a validRange
		std::vector<char>    m_validRange;  //!< It's false (=0) on no reflected rays, referenced to elements in \a scan
		bool                 m_has_intensity; //!< Whether the intensity values are present or not. If not, space is saved during serialization.

	public:
		typedef std::vector<mrpt::math::CPolygon> TListExclusionAreas; //!< Used in filterByExclusionAreas
		typedef std::vector<std::pair<mrpt::math::CPolygon,std::pair<double,double> > > TListExclusionAreasWithRanges; //!< Used in filterByExclusionAreas

		CObservation2DRangeScan(); //!< Default constructor
		CObservation2DRangeScan(const CObservation2DRangeScan &o); //!< copy ctor
		virtual ~CObservation2DRangeScan(); //!< Destructor

		/** @name Scan data
		    @{ */
		void resizeScan(const size_t len); //!< Resizes all data vectors to allocate a given number of scan rays
		void resizeScanAndAssign(const size_t len, const float rangeVal, const bool rangeValidity, const int32_t rangeIntensity = 0); //!< Resizes all data vectors to allocate a given number of scan rays and assign default values.
		size_t getScanSize() const; //!< Get number of scan rays

		mrpt::utils::ContainerReadOnlyProxyAccessor<std::vector<float> > scan; //!< The range values of the scan, in meters. Must have same length than \a validRange
		float getScanRange(const size_t i) const;
		void setScanRange(const size_t i, const float val);

		mrpt::utils::ContainerReadOnlyProxyAccessor<std::vector<int32_t> >   intensity; //!< The intensity values of the scan. If available, must have same length than \a validRange
		int32_t getScanIntensity(const size_t i) const;
		void setScanIntensity(const size_t i, const int val);

		mrpt::utils::ContainerReadOnlyProxyAccessor<std::vector<char> >  validRange;  //!< It's false (=0) on no reflected rays, referenced to elements in \a scan
		bool  getScanRangeValidity(const size_t i) const;
		void setScanRangeValidity(const size_t i, const bool val);

		float                aperture; //!< The "aperture" or field-of-view of the range finder, in radians (typically M_PI = 180 degrees).
		bool                 rightToLeft; //!< The scanning direction: true=counterclockwise; false=clockwise
		float                maxRange; //!< The maximum range allowed by the device, in meters (e.g. 80m, 50m,...)
		mrpt::poses::CPose3D sensorPose; //!< The 6D pose of the sensor on the robot at the moment of starting the scan.
		float                stdError; //!< The "sigma" error of the device in meters, used while inserting the scan in an occupancy grid.
		float                beamAperture; //!< The aperture of each beam, in radians, used to insert "thick" rays in the occupancy grid.
		double               deltaPitch; //!< If the laser gathers data by sweeping in the pitch/elevation angle, this holds the increment in "pitch" (=-"elevation") between the beginning and the end of the scan (the sensorPose member stands for the pose at the beginning of the scan).

		void getScanProperties(T2DScanProperties& p) const;  //!< Fill out a T2DScanProperties structure with the parameters of this scan
		/** @} */

		void loadFromVectors(size_t nRays, const float *scanRanges, const char *scanValidity );

		/** @name Cached points map
		    @{  */
	protected:
		/** A points map, build only under demand by the methods getAuxPointsMap() and buildAuxPointsMap().
		  *  It's a generic smart pointer to avoid depending here in the library mrpt-obs on classes on other libraries.
		  */
		mutable mrpt::maps::CMetricMapPtr  m_cachedMap;
		void internal_buildAuxPointsMap( const void *options = NULL ) const;  //!< Internal method, used from buildAuxPointsMap()
	public:

		/** Returns the cached points map representation of the scan, if already build with buildAuxPointsMap(), or NULL otherwise.
		  * Usage:
		  *  \code
		  *    mrpt::maps::CPointsMap *map = obs->getAuxPointsMap<mrpt::maps::CPointsMap>();
		  *  \endcode
		  * \sa buildAuxPointsMap
		  */
		template <class POINTSMAP>
		inline const POINTSMAP* getAuxPointsMap() const {
			return static_cast<const POINTSMAP*>(m_cachedMap.pointer());
		}

		/** Returns a cached points map representing this laser scan, building it upon the first call.
		  * \param options Can be NULL to use default point maps' insertion options, or a pointer to a "CPointsMap::TInsertionOptions" structure to override some params.
		  * Usage:
		  *  \code
		  *    mrpt::maps::CPointsMap *map = obs->buildAuxPointsMap<mrpt::maps::CPointsMap>(&options or NULL);
		  *  \endcode
		  * \sa getAuxPointsMap
		  */
		template <class POINTSMAP>
		inline const POINTSMAP	*buildAuxPointsMap( const void *options = NULL ) const {
			if (!m_cachedMap.present()) internal_buildAuxPointsMap(options);
			return static_cast<const POINTSMAP*>(m_cachedMap.pointer());
		}

		/** @} */



		/** Return true if the laser scanner is "horizontal", so it has an absolute value of "pitch" and "roll" less or equal to the given tolerance (in rads, default=0) (with the normal vector either upwards or downwards).
		  */
		bool isPlanarScan(const double tolerance = 0) const;

		bool hasIntensity() const; //!< Return true if scan has intensity
		void setScanHasIntensity(bool setHasIntensityFlag); //!< Marks this scan as having or not intensity data.

		// See base class docs
		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE { out_sensorPose = sensorPose; }
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) MRPT_OVERRIDE { sensorPose = newSensorPose; }
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;

		/** A general method to truncate the scan by defining a minimum valid distance and a maximum valid angle as well as minimun and maximum heights
		   (NOTE: the laser z-coordinate must be provided).
		  */
		void truncateByDistanceAndAngle(float min_distance, float max_angle, float min_height = 0, float max_height = 0, float h = 0 );

		/** Mark as invalid sensed points that fall within any of a set of "exclusion areas", given in coordinates relative to the vehicle (taking into account "sensorPose").
		  * \sa C2DRangeFinderAbstract::loadExclusionAreas
		  */
		void filterByExclusionAreas( const TListExclusionAreas &areas );

		/** Mark as invalid sensed points that fall within any of a set of "exclusion areas", given in coordinates relative to the vehicle (taking into account "sensorPose"), AND such as the Z coordinate of the point falls in the range [min,max] associated to each exclusion polygon.
		  * \sa C2DRangeFinderAbstract::loadExclusionAreas
		  */
		void filterByExclusionAreas( const TListExclusionAreasWithRanges &areas );

		/** Mark as invalid the ranges in any of a given set of "forbiden angle ranges", given as pairs<min_angle,max_angle>.
		  * \sa C2DRangeFinderAbstract::loadExclusionAreas
		  */
		void filterByExclusionAngles( const std::vector<std::pair<double,double> >  &angles );

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservation2DRangeScan, CObservation, OBS_IMPEXP)

	} // End of namespace
	namespace utils
	{
		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_PTR_NAMESPACE(CObservation2DRangeScan, ::mrpt::obs)
	}

} // End of namespace

#endif
