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
	 * The internal metric maps can be accessed directly by the user as smart pointers with CMultiMetricMap::getMapByIndex() or via `iterator`s.
	 * The utility of this container is to operate on several maps simultaneously: update them by inserting observations, 
	 * evaluate the likelihood of one observation by fusing (multiplying) the likelihoods over the different maps, etc.
	 *
	 *  <b>These kinds of metric maps can be kept inside</b> (list may be incomplete, refer to classes derived from mrpt::maps::CMetricMap):
	 *		- mrpt::maps::CSimplePointsMap: For 2D or 3D range scans, ...
	 *		- mrpt::maps::COccupancyGridMap2D: 2D, <b>horizontal</b>  laser range scans, at different altitudes.
	 *		- mrpt::maps::COctoMap: For 3D occupancy grids of variable resolution, with octrees (based on the library `octomap`).
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
	 * See CMultiMetricMap::setListOfMaps() for the method for initializing this class programatically.
	 * See also TSetOfMetricMapInitializers::loadFromConfigFile for a template of ".ini"-like configuration
	 * file that can be used to define which maps to create and all their parameters.
	 * Alternatively, the list of maps is public so it can be directly manipulated/accessed in CMultiMetricMap::maps
	 *
	 * \note [New in MRPT 1.3.0]: `likelihoodMapSelection`, which selected the map to be used when 
	 *  computing the likelihood of an observation, has been removed. Use the `enableObservationLikelihood` 
	 *  property of each individual map declaration. 
	 * 
	 * \note [New in MRPT 1.3.0]: `enableInsertion_{pointsMap,...}` have been also removed. 
	 *  Use the `enableObservationInsertion` property of each map declaration.
	 *
	 * \note [New in MRPT 1.3.0]: Plain list of maps is exposed in `maps` member. Proxies named `m_pointsMaps`,`m_gridMaps`, etc. 
	 *  are provided for backwards-compatibility and for their utility.
	 *
	 * \note This class belongs to [mrpt-slam] instead of [mrpt-maps] due to the dependency on map classes in mrpt-vision.
	 * \sa CMetricMap  \ingroup mrpt_slam_grp 
	 */
	class SLAM_IMPEXP CMultiMetricMap : public mrpt::maps::CMetricMap
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CMultiMetricMap )
	protected:
		void  deleteAllMaps(); //!< Deletes all maps and clears the internal lists of maps (with clear_unique(), so user copies remain alive)
		virtual void  internal_clear(); //!< Clear all elements of the map.
		// See base class docs
		virtual bool  internal_insertObservation( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose = NULL ) MRPT_OVERRIDE;
		/** Returns true if any of the inner maps is able to compute a sensible likelihood function for this observation.
		 * \param obs The observation.
		 * \sa computeObservationLikelihood
		 */
		bool internal_canComputeObservationLikelihood( const mrpt::obs::CObservation *obs );
		// See docs in base class
		double	 internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom );

	public:
		/** @name Access to internal list of maps: direct list, utility methods and proxies
		    @{ */
		
		typedef std::deque<mrpt::maps::CMetricMapPtr> TListMaps;
		/** The list of MRPT metric maps in this object. Use dynamic_cast or smart pointer-based downcast to access maps by their actual type.
		  * You can directly manipulate this list. Helper methods to initialize it are described in the docs of CMultiMetricMap
		  */
		TListMaps maps;

		mrpt::maps::CMetricMapPtr getMapByIndex(size_t idx) const;

		// Note: A variable number of maps may exist, depending on the initialization from TSetOfMetricMapInitializers.
		//       Not used maps are "NULL" or empty smart pointers.
		//std::deque<mrpt::maps::CSimplePointsMapPtr>              m_pointsMaps;
		//std::deque<mrpt::maps::COccupancyGridMap2DPtr>           m_gridMaps;
		//std::deque<mrpt::maps::COctoMapPtr>                      m_octoMaps;
		//std::deque<mrpt::maps::CColouredOctoMapPtr>              m_colourOctoMaps;
		//std::deque<mrpt::maps::CGasConcentrationGridMap2DPtr>    m_gasGridMaps;
		//std::deque<mrpt::maps::CWirelessPowerGridMap2DPtr>       m_wifiGridMaps;
		//std::deque<mrpt::maps::CHeightGridMap2DPtr>              m_heightMaps;
		//std::deque<mrpt::maps::CReflectivityGridMap2DPtr>        m_reflectivityMaps;
		//mrpt::maps::CColouredPointsMapPtr                        m_colourPointsMap;
		//mrpt::maps::CWeightedPointsMapPtr                        m_weightedPointsMap;
		//mrpt::maps::CLandmarksMapPtr                             m_landmarksMap;
		//mrpt::maps::CBeaconMapPtr                                m_beaconMap;

		/** @} */

		/** Constructor.
		 * \param initializers One internal map will be created for each entry in this "TSetOfMetricMapInitializers" struct.
		 *  If initializers is NULL, no internal map will be created.
		 */
		CMultiMetricMap(const mrpt::maps::TSetOfMetricMapInitializers	*initializers = NULL);
		CMultiMetricMap(const mrpt::maps::CMultiMetricMap &other );  //!< Copy constructor
		mrpt::maps::CMultiMetricMap &operator = ( const mrpt::maps::CMultiMetricMap &other ); //!< Copy operator from "other" object.
		virtual ~CMultiMetricMap( ); //!< Destructor.

		/** Sets the list of internal map according to the passed list of map initializers (Current maps' content will be deleted!) */
		void  setListOfMaps( const mrpt::maps::TSetOfMetricMapInitializers	*initializers );

		bool  isEmpty() const MRPT_OVERRIDE; //!< Returns true if all maps returns true to their isEmpty() method, which is map-dependent. Read the docs of each map class

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
