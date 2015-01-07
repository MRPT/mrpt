/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMultiMetricMap_H
#define CMultiMetricMap_H

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/CColouredOctoMap.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/maps/CWirelessPowerGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/maps/CReflectivityGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CWeightedPointsMap.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/TEnumType.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
namespace maps
{
	class TSetOfMetricMapInitializers;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CMultiMetricMap , CMetricMap, SLAM_IMPEXP )

	/** This class stores any customizable set of metric maps.
	 *  The internal metric maps can be accessed directly by the user as smart pointers. 
	 *   The intended utility of this container is to operate on several maps simultaneously: update them by inserting observations, 
	 *    evaluate the likelihood of one observation by fusing (multiplying) the likelihoods over the different maps, etc.
	 *
	 *  <b>All these kinds of metric maps can be kept in a multi-metric map:</b>:
	 *		- mrpt::maps::CPointsMap: For laser 2D range scans, and posibly for IR ranges,... (It keeps the full 3D structure of scans)
	 *		- mrpt::maps::COccupancyGridMap2D: Exclusively for 2D, <b>horizontal</b>  laser range scans, at different altitudes.
	 *		- mrpt::maps::COctoMap: For 3D occupancy grids of variable resolution, with octrees (based on the library "octomap").
	 *		- mrpt::maps::CColouredOctoMap: The same than above, but nodes can store RGB data appart from occupancy.
	 *		- mrpt::maps::CLandmarksMap: For visual landmarks,etc...
	 *		- mrpt::maps::CGasConcentrationGridMap2D: For gas concentration maps.
	 *		- mrpt::maps::CWirelessPowerGridMap2D: For wifi power maps.
	 *		- mrpt::maps::CBeaconMap: For range-only SLAM.
	 *		- mrpt::maps::CHeightGridMap2D: For maps of height for each (x,y) location.
	 *		- mrpt::maps::CReflectivityGridMap2D: For maps of "reflectivity" for each (x,y) location.
	 *		- mrpt::maps::CColouredPointsMap: For point map with color.
	 *		- mrpt::maps::CWeightedPointsMap: For point map with weights (capable of "fusing").
	 *
	 *  See CMultiMetricMap::setListOfMaps() for the method for initializing this class programatically. 
	 *  See also TSetOfMetricMapInitializers::loadFromConfigFile for a template of ".ini"-like configuration
	 *   file that can be used to define which maps to create and all their parameters.
	 *
	 * \note [New in MRPT 1.3.0]: `likelihoodMapSelection`, which selected the map to be used when 
	 *   computing the likelihood of an observation, has been removed. Use the `enableObservationLikelihood` 
	 *   property of each individual map declaration. 
	 * 
	 * \note [New in MRPT 1.3.0]: `enableInsertion_{pointsMap,...}` have been also removed. 
	 *   Use the `enableObservationInsertion` property of each map declaration.
	 *
	 * \note This class belongs to [mrpt-slam] instead of [mrpt-maps] due to the dependency on map classes in mrpt-vision.
	 * \sa CMetricMap  \ingroup mrpt_slam_grp 
	 */
	class SLAM_IMPEXP CMultiMetricMap : public mrpt::maps::CMetricMap
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CMultiMetricMap )

	protected:
		/** Deletes all maps and clears the internal lists of maps.
		  */
		void  deleteAllMaps();

		/** Clear all elements of the map.
		  */
		virtual void  internal_clear();

		 /** Insert the observation information into this map (see options)
		  * \param obs The observation
		  * \param robotPose The 3D pose of the robot mobile base in the map reference system, or NULL (default) if you want to use CPose2D(0,0,deg)
		  *
		  * \sa CObservation::insertObservationInto
		  */
		virtual bool  internal_insertObservation( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose = NULL );

	public:
		typedef std::pair<mrpt::poses::CPoint3D,unsigned int> TPairIdBeacon;

		/** Returns true if the map is empty/no observation has been inserted.
		*/
		bool  isEmpty() const;

		/** Some options for this class:
		  */
		struct SLAM_IMPEXP TOptions : public utils::CLoadableOptions
		{
			TOptions();

			/** Load parameters from configuration source
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase	&source,
				const std::string		&section);

			/** This method must display clearly all the contents of the structure in textual form, sending it to a CStream.
			  */
			void  dumpToTextStream(mrpt::utils::CStream	&out) const;

			/** This selects the map to be used when computing the likelihood of an observation.
			 * This enum has a corresponding mrpt::utils::TEnumType<> specialization.
			 * \sa computeObservationLikelihood
			 */
			enum TMapSelectionForLikelihood
			{
				mapFuseAll = -1,
				mapGrid = 0,
				mapPoints,
				mapLandmarks,
				mapGasGrid,
				mapWifiGrid,
				mapBeacon,
				mapHeight,
				mapColourPoints,
				mapReflectivity,
				mapWeightedPoints,
				mapOctoMaps,
				mapColourOctoMaps
			} likelihoodMapSelection;

			bool	enableInsertion_pointsMap;			//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_landmarksMap;		//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_gridMaps;			//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_gasGridMaps;		//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_wifiGridMaps;		//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_beaconMap;			//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_heightMaps;			//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_reflectivityMaps;	//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_colourPointsMaps;	//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_weightedPointsMaps;	//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_octoMaps;			//!< Default = true (set to false to avoid "insertObservation" to update a given map)
			bool	enableInsertion_colourOctoMaps;		//!< Default = true (set to false to avoid "insertObservation" to update a given map)

		} options;


		/** @name Internal lists of maps
		    @{ */
		// Note: A variable number of maps may exist, depending on the initialization from TSetOfMetricMapInitializers.
		//       Not used maps are "NULL" or empty smart pointers.

		std::deque<mrpt::maps::CSimplePointsMapPtr>              m_pointsMaps;
		std::deque<mrpt::maps::COccupancyGridMap2DPtr>           m_gridMaps;
		std::deque<mrpt::maps::COctoMapPtr>                      m_octoMaps;
		std::deque<mrpt::maps::CColouredOctoMapPtr>              m_colourOctoMaps;
		std::deque<mrpt::maps::CGasConcentrationGridMap2DPtr>    m_gasGridMaps;
		std::deque<mrpt::maps::CWirelessPowerGridMap2DPtr>       m_wifiGridMaps;
		std::deque<mrpt::maps::CHeightGridMap2DPtr>              m_heightMaps;
		std::deque<mrpt::maps::CReflectivityGridMap2DPtr>        m_reflectivityMaps;
		mrpt::maps::CColouredPointsMapPtr                        m_colourPointsMap;
		mrpt::maps::CWeightedPointsMapPtr                        m_weightedPointsMap;
		mrpt::maps::CLandmarksMapPtr                             m_landmarksMap;
		mrpt::maps::CBeaconMapPtr                                m_beaconMap;

		/** @} */

		/** Constructor.
		 * \param initializers One internal map will be created for each entry in this "TSetOfMetricMapInitializers" struct, and each map will be initialized with the corresponding options.
		 * \param opts If provided (not NULL), the member "options" will be initialized with those values.
		 *  If initializers is NULL, no internal map will be created.
		 */
		CMultiMetricMap(
			const mrpt::maps::TSetOfMetricMapInitializers	*initializers = NULL,
			const TOptions		*opts		  = NULL );

		/** Sets the list of internal map according to the passed list of map initializers (Current maps' content will be deleted!)
		  */
		void  setListOfMaps( const mrpt::maps::TSetOfMetricMapInitializers	*initializers );

		/** Copy constructor */
		CMultiMetricMap(const mrpt::maps::CMultiMetricMap &other );

		/** Copy operator from "other" object.
		 */
		mrpt::maps::CMultiMetricMap &operator = ( const mrpt::maps::CMultiMetricMap &other );

		/** Destructor.
		 */
		virtual ~CMultiMetricMap( );


		// See docs in base class
		double	 computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom );

		/** Returns the ratio of points in a map which are new to the point map while falling into yet static cells of gridmap.
		  * \param points The set of points to check.
		  * \param takenFrom The pose for the reference system of points, in global coordinates of this hybrid map.
		  */
		float 	getNewStaticPointsRatio(
			mrpt::maps::CPointsMap		*points,
			mrpt::poses::CPose2D		&takenFrom );

		// See docs in base class.
		virtual void  determineMatching2D(
			const mrpt::maps::CMetricMap      * otherMap,
			const mrpt::poses::CPose2D         & otherMapPose,
			mrpt::utils::TMatchingPairList     & correspondences,
			const mrpt::maps::TMatchingParams & params,
			mrpt::maps::TMatchingExtraResults & extraResults ) const;

		/** See the definition in the base class: Calls in this class become a call to every single map in this set. */
		float  compute3DMatchingRatio(
				const mrpt::maps::CMetricMap						*otherMap,
				const mrpt::poses::CPose3D							&otherMapPose,
				float									maxDistForCorr = 0.10f,
				float									maxMahaDistForCorr = 2.0f
				) const;

		/** The implementation in this class just calls all the corresponding method of the contained metric maps.
		  */
		void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
			) const;

		/** This method is called at the end of each "prediction-update-map insertion" cycle within "mrpt::slam::CMetricMapBuilderRBPF::processActionObservation".
		  *  This method should normally do nothing, but in some cases can be used to free auxiliary cached variables.
		  */
		void  auxParticleFilterCleanUp();

		/** Returns a 3D object representing the map.
		  */
		void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

		/** Returns true if any of the inner maps is able to compute a sensible likelihood function for this observation.
		 * \param obs The observation.
		 * \sa computeObservationLikelihood
		 */
		bool canComputeObservationLikelihood( const mrpt::obs::CObservation *obs );

		/** If the map is a simple point map or it's a multi-metric map that contains EXACTLY one simple point map, return it.
			* Otherwise, return NULL
			*/
		virtual const mrpt::maps::CSimplePointsMap * getAsSimplePointsMap() const;
		virtual       mrpt::maps::CSimplePointsMap * getAsSimplePointsMap();

		/** An auxiliary variable that can be used freely by the users (this will be copied to other maps using the copy constructor, copy operator, streaming,etc) The default value is 0.
		  */
		unsigned int	m_ID;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CMultiMetricMap , mrpt::maps::CMetricMap, SLAM_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
