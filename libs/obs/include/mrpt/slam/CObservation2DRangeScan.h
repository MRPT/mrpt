/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CObservation2DRangeScan_H
#define CObservation2DRangeScan_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

#include <mrpt/slam/CMetricMap.h>

#include <mrpt/math/CPolygon.h>


namespace mrpt
{
namespace slam
{

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservation2DRangeScan, CObservation, OBS_IMPEXP)

	/** Declares a class derived from "CObservation" that
	       encapsules a 2D range scan measurement (typically from a laser scanner).
		   This is prepared for accepting 180deg,360deg or any other aperture scan,
		   as well as resolutions of 0.5deg,0.25deg or any other.
	 *
	 * \sa CObservation
	 */
	class OBS_IMPEXP CObservation2DRangeScan : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservation2DRangeScan )

	 public:
		typedef std::vector<mrpt::math::CPolygon> TListExclusionAreas; //!< Used in filterByExclusionAreas
		typedef std::vector<std::pair<mrpt::math::CPolygon,std::pair<double,double> > > TListExclusionAreasWithRanges; //!< Used in filterByExclusionAreas

		/** Default constructor */
		CObservation2DRangeScan( );

		/** Destructor */
		virtual ~CObservation2DRangeScan( );


		/** @name Scan data
		    @{ */

		/** The range values of the scan, in meters.
		  */
		vector_float	    scan;

		/** It's false (=0) on no reflected rays, referenced to elements in "scan"
		  *  (Added in the streamming version #1 of the class)
		  */
		std::vector<char>	validRange;

		/** The aperture of the range finder, in radians (typically M_PI = 180 degrees).
		  */
		float				aperture;

		/** The scanning direction
		  */
		bool				rightToLeft;

		/** The maximum range allowed by the device, in meters (e.g. 80m, 50m,...)
		  */
		float				maxRange;

		/** The 6D pose of the sensor on the robot.
		  */
		CPose3D				sensorPose;

		/** The "sigma" error of the device in meters, used while inserting the scan in an occupancy grid.
		  */
		float				stdError;

		/** The aperture of each beam, in radians, used to insert "thick" rays in the occupancy grid.
		  * (Added in the streamming version #4 of the class)
		  */
		float				beamAperture;

		/** If the laser gathers data by sweeping in the pitch/elevation angle, this holds the increment in "pitch" (=-"elevation") between the beginning and the end of the scan (the sensorPose member stands for the pose at the beginning of the scan).
		  */
		double				deltaPitch;

		/** @} */


		/** @name Cached points map
		    @{  */

	protected:
		/** A points map, build only under demand by the methods getAuxPointsMap() and buildAuxPointsMap().
		  *  It's a generic smart pointer to avoid depending here in the library mrpt-obs on classes on other libraries.
		  */
		mutable mrpt::slam::CMetricMapPtr  m_cachedMap;

		void internal_buildAuxPointsMap( const void *options = NULL ) const;  //!< Internal method, used from buildAuxPointsMap()

	public:

		/** Returns the cached points map representation of the scan, if already build with buildAuxPointsMap(), or NULL otherwise.
		  * Usage:
		  *  \code
		  *    mrpt::slam::CPointsMap *map = obs->getAuxPointsMap<mrpt::slam::CPointsMap>();
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
		  *    mrpt::slam::CPointsMap *map = obs->buildAuxPointsMap<mrpt::slam::CPointsMap>(&options or NULL);
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

		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = sensorPose; }


		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) { sensorPose = newSensorPose; }

		/** A general method to truncate the scan by defining a minimum valid distance and a maximum valid angle as well as minimun and maximum heights
		   (NOTE: the laser z-coordinate must be provided).
		  */
		void truncateByDistanceAndAngle(float min_distance, float max_angle, float min_height = 0, float max_height = 0, float h = 0 );

		/** Mark as invalid sensed points that fall within any of a set of "exclusion areas", given in coordinates relative to the vehicle (taking into account "sensorPose").
		  * \sa C2DRangeFinderAbstract::loadExclusionAreas
		  */
		void filterByExclusionAreas( const TListExclusionAreas &areas );

		/** Mark as invalid sensed points that fall within any of a set of "exclusion areas", given in coordinates relative to the vehicle (taking into account "sensorPose").
		  * \sa C2DRangeFinderAbstract::loadExclusionAreas
		  */
		void filterByExclusionAreas( const TListExclusionAreasWithRanges &areas );

		/** Mark as invalid the ranges in any of a given set of "forbiden angle ranges", given as pairs<min_angle,max_angle>.
		  * \sa C2DRangeFinderAbstract::loadExclusionAreas
		  */
		void filterByExclusionAngles( const std::vector<std::pair<double,double> >  &angles );

	}; // End of class def.


	} // End of namespace

	namespace utils
	{
		using namespace ::mrpt::slam;
		// Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_PTR(CObservation2DRangeScan)
	}

} // End of namespace

#endif
