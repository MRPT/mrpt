/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#ifndef CMultiMetricMap_H
#define CMultiMetricMap_H

#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CGasConcentrationGridMap2D.h>
#include <mrpt/slam/CWirelessPowerGridMap2D.h>
#include <mrpt/slam/CHeightGridMap2D.h>
#include <mrpt/slam/CReflectivityGridMap2D.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/CColouredPointsMap.h>
#include <mrpt/slam/CWeightedPointsMap.h>
#include <mrpt/slam/CLandmarksMap.h>
#include <mrpt/slam/CBeaconMap.h>
#include <mrpt/slam/CMetricMap.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/TEnumType.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	class TSetOfMetricMapInitializers;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CMultiMetricMap , CMetricMap, SLAM_IMPEXP )

	/** This class stores any customizable set of metric maps.
	 *    The internal metric maps can be accessed directly by the user.
	 *    If some kind of map is not desired, it can be just ignored, but if this fact is specified in the
	 *    "CMultiMetricMap::mapsUsage" member some methods (as the observation insertion) will be more
	 *    efficient since it will be invoked on desired maps only.<br><br>
	 *  <b>Currently these metric maps are supported for being kept internally:</b>:
	 *		- mrpt::slam::CPointsMap: For laser 2D range scans, and posibly for IR ranges,... (It keeps the full 3D structure of scans)
	 *		- mrpt::slam::COccupancyGridMap2D: Exclusively for 2D, <b>horizontal</b>  laser range scans, at different altitudes.
	 *		- mrpt::slam::CLandmarksMap: For visual landmarks,etc...
	 *		- mrpt::slam::CGasConcentrationGridMap2D: For gas concentration maps.
	 *		- mrpt::slam::CWirelessPowerGridMap2D: For wifi power maps.
	 *		- mrpt::slam::CBeaconMap: For range-only SLAM.
	 *		- mrpt::slam::CHeightGridMap2D: For maps of height for each (x,y) location.
	 *		- mrpt::slam::CReflectivityGridMap2D: For maps of "reflectivity" for each (x,y) location.
	 *		- mrpt::slam::CColouredPointsMap: For point map with color.
	 *		- mrpt::slam::CWeightedPointsMap: For point map with weights (capable of "fusing").
	 *
	 *  See CMultiMetricMap::setListOfMaps() for the method for initializing this class, and also
	 *   see TSetOfMetricMapInitializers::loadFromConfigFile for a template of ".ini"-like configuration
	 *   file that can be used to define what maps to create and all their parameters.
	 *
	 * \sa CMetricMap  \ingroup mrpt_slam_grp 
	 */
	class SLAM_IMPEXP CMultiMetricMap : public CMetricMap
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
		virtual bool  internal_insertObservation( const CObservation *obs, const CPose3D *robotPose = NULL );

	public:
		typedef std::pair<CPoint3D,unsigned int> TPairIdBeacon;

		/** Returns true if the map is empty/no observation has been inserted.
		*/
		bool  isEmpty() const;

		/** Some options for this class:
		  */
		struct SLAM_IMPEXP TOptions : public utils::CLoadableOptions
		{
			TOptions() :	likelihoodMapSelection(mapFuseAll),
							enableInsertion_pointsMap(true),
							enableInsertion_landmarksMap(true),
							enableInsertion_gridMaps(true),
							enableInsertion_gasGridMaps(true),
							enableInsertion_wifiGridMaps(true),
							enableInsertion_beaconMap(true),
							enableInsertion_heightMaps(true),
							enableInsertion_reflectivityMaps(true),
							enableInsertion_colourPointsMaps(true),
							enableInsertion_weightedPointsMaps(true)
			{
			}

			/** Load parameters from configuration source
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase	&source,
				const std::string		&section);

			/** This method must display clearly all the contents of the structure in textual form, sending it to a CStream.
			  */
			void  dumpToTextStream(CStream	&out) const;

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
				mapWeightedPoints
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

		} options;


		/** @name Internal lists of maps
		    @{ */
		// Note: A variable number of maps may exist, depending on the initialization from TSetOfMetricMapInitializers.
		//       Not used maps are "NULL" or empty smart pointers.

		std::deque<CSimplePointsMapPtr>              m_pointsMaps;
		std::deque<COccupancyGridMap2DPtr>           m_gridMaps;
		std::deque<CGasConcentrationGridMap2DPtr>    m_gasGridMaps;
		std::deque<CWirelessPowerGridMap2DPtr>       m_wifiGridMaps;
		std::deque<CHeightGridMap2DPtr>              m_heightMaps;
		std::deque<CReflectivityGridMap2DPtr>        m_reflectivityMaps;
		CColouredPointsMapPtr                        m_colourPointsMap;
		CWeightedPointsMapPtr                        m_weightedPointsMap;
		CLandmarksMapPtr                             m_landmarksMap;
		CBeaconMapPtr                                m_beaconMap;

		/** @} */

		/** Constructor.
		 * \param initializers One internal map will be created for each entry in this "TSetOfMetricMapInitializers" struct, and each map will be initialized with the corresponding options.
		 * \param opts If provided (not NULL), the member "options" will be initialized with those values.
		 *  If initializers is NULL, no internal map will be created.
		 */
		CMultiMetricMap(
			const mrpt::slam::TSetOfMetricMapInitializers	*initializers = NULL,
			const TOptions		*opts		  = NULL );

		/** Sets the list of internal map according to the passed list of map initializers (Current maps' content will be deleted!)
		  */
		void  setListOfMaps( const mrpt::slam::TSetOfMetricMapInitializers	*initializers );

		/** Copy constructor */
		CMultiMetricMap(const mrpt::slam::CMultiMetricMap &other );

		/** Copy operator from "other" object.
		 */
		mrpt::slam::CMultiMetricMap &operator = ( const mrpt::slam::CMultiMetricMap &other );

		/** Destructor.
		 */
		virtual ~CMultiMetricMap( );


		/** Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
		 *
		 * \param takenFrom The robot's pose the observation is supposed to be taken from.
		 * \param obs The observation.
		 * \return This method returns a likelihood in the range [0,1].
		 *
		 * \sa likelihoodMapSelection, Used in particle filter algorithms, see: CMultiMetricMapPDF::update
		 */
		double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );

		/** Returns the ratio of points in a map which are new to the point map while falling into yet static cells of gridmap.
		  * \param points The set of points to check.
		  * \param takenFrom The pose for the reference system of points, in global coordinates of this hybrid map.
		  */
		float 	getNewStaticPointsRatio(
				CPointsMap		*points,
				CPose2D			&takenFrom );

		/** See the definition in the base class: In this class calls to this method are passed to the inner point map.
		 *
		 * \sa computeMatching3DWith
		 */
		void  computeMatchingWith2D(
				const CMetricMap						*otherMap,
				const CPose2D							&otherMapPose,
				float									maxDistForCorrespondence,
				float									maxAngularDistForCorrespondence,
				const CPose2D							&angularDistPivotPoint,
				TMatchingPairList						&correspondences,
				float									&correspondencesRatio,
				float									*sumSqrDist	= NULL,
				bool									onlyKeepTheClosest = false,
				bool									onlyUniqueRobust = false,
				const size_t          decimation_other_map_points = 1,
				const size_t          offset_other_map_points = 0 ) const;

		/** Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
		 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
		 * \param  otherMap					  [IN] The other map to compute the matching with.
		 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
		 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
		 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
		 *
		 * \return The matching ratio [0,1]
		 * \sa computeMatchingWith2D
		 */
		float  compute3DMatchingRatio(
				const CMetricMap						*otherMap,
				const CPose3D							&otherMapPose,
				float									minDistForCorr = 0.10f,
				float									minMahaDistForCorr = 2.0f
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

		/** Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
		 * \param obs The observation.
		 * \sa computeObservationLikelihood
		 */
		bool canComputeObservationLikelihood( const CObservation *obs );

		/** Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
		 * \param obs The observation.
		 * \sa computeObservationLikelihood
		 */
		inline bool canComputeObservationLikelihood( const CObservationPtr &obs ) { return canComputeObservationLikelihood(obs.pointer()); }

		/** If the map is a simple point map or it's a multi-metric map that contains EXACTLY one simple point map, return it.
			* Otherwise, return NULL
			*/
		virtual const CSimplePointsMap * getAsSimplePointsMap() const;
		virtual       CSimplePointsMap * getAsSimplePointsMap();

		/** An auxiliary variable that can be used freely by the users (this will be copied to other maps using the copy constructor, copy operator, streaming,etc) The default value is 0.
		  */
		unsigned int	m_ID;

	}; // End of class def.

	/** Each structure of this kind will determine the kind (and initial configuration) of one map to be build into a CMultiMetricMap object.
	  *  See "mrpt::slam::TSetOfMetricMapInitializers::loadFromConfigFile" as an easy way of initialize this object.
	  * \sa TSetOfMetricMapInitializers, CMultiMetricMap::CMultiMetricMap
	  */
	struct SLAM_IMPEXP  TMetricMapInitializer
	{
		/** Initialization (sets 'metricMapClassType' to NULL, an invalid value -> it must be set correctly before use!)
		  */
		TMetricMapInitializer();

		/** Set this to CLASS_ID(< class >) where < class > is any CMetricMap derived class.
		  */
		TRuntimeClassIdPtr	metricMapClassType;

		/** This value will be copied to the member with the same value in the map, see mrpt::slam::CMetricMap::m_disableSaveAs3DObject
		  */
		bool				m_disableSaveAs3DObject;

		/** Specific options for grid maps (mrpt::slam::COccupancyGridMap2D)
		  */
		struct SLAM_IMPEXP TOccGridMap2DOptions
		{
			TOccGridMap2DOptions();	//!< Default values loader

			float	min_x,max_x,min_y,max_y,resolution;	//!< See COccupancyGridMap2D::COccupancyGridMap2D
			COccupancyGridMap2D::TInsertionOptions	insertionOpts;	//!< Customizable initial options.
			COccupancyGridMap2D::TLikelihoodOptions	likelihoodOpts;	//!< Customizable initial options.

		} occupancyGridMap2D_options;

		/** Specific options for point maps (mrpt::slam::CPointsMap)
		  */
		struct SLAM_IMPEXP CPointsMapOptions
		{
			CPointsMapOptions();		//!< Default values loader
			CPointsMap::TInsertionOptions	insertionOpts;	//!< Customizable initial options for loading the class' own defaults.
			CPointsMap::TLikelihoodOptions  likelihoodOpts; //!< 	//!< Customizable initial likelihood options
		} pointsMapOptions_options;

		/** Specific options for gas grid maps (mrpt::slam::CGasConcentrationGridMap2D)
		  */
		struct SLAM_IMPEXP CGasConcentrationGridMap2DOptions
		{
			CGasConcentrationGridMap2DOptions();	//!< Default values loader

			float	min_x,max_x,min_y,max_y,resolution;	//!< See CGasConcentrationGridMap2D::CGasConcentrationGridMap2D
			CGasConcentrationGridMap2D::TMapRepresentation	mapType;	//!< The kind of map representation (see CGasConcentrationGridMap2D::CGasConcentrationGridMap2D)
			CGasConcentrationGridMap2D::TInsertionOptions	insertionOpts;	//!< Customizable initial options.

		} gasGridMap_options;

		/** Specific options for wifi grid maps (mrpt::slam::CWirelessPowerGridMap2D)
		  */
		struct SLAM_IMPEXP CWirelessPowerGridMap2DOptions
		{
			CWirelessPowerGridMap2DOptions();	//!< Default values loader

			float	min_x,max_x,min_y,max_y,resolution;	//!< See CWirelessPowerGridMap2D::CWirelessPowerGridMap2D
			CWirelessPowerGridMap2D::TMapRepresentation	mapType;	//!< The kind of map representation (see CWirelessPowerGridMap2D::CWirelessPowerGridMap2D)
			CWirelessPowerGridMap2D::TInsertionOptions	insertionOpts;	//!< Customizable initial options.

		} wifiGridMap_options;

		/** Specific options for landmarks maps (mrpt::slam::CLandmarksMap)
		  */
		struct SLAM_IMPEXP CLandmarksMapOptions
		{
			CLandmarksMapOptions();		//!< Default values loader

			std::deque<CMultiMetricMap::TPairIdBeacon>	initialBeacons;	//!< Initial contents of the map, especified by a set of 3D Beacons with associated IDs
			CLandmarksMap::TInsertionOptions	insertionOpts;	//!< Customizable initial options.
			CLandmarksMap::TLikelihoodOptions	likelihoodOpts;	//!< Customizable initial options.

		} landmarksMap_options;


		/** Specific options for landmarks maps (mrpt::slam::CBeaconMap)
		  */
		struct SLAM_IMPEXP CBeaconMapOptions
		{
			CBeaconMapOptions();	//!< Default values loader

			CBeaconMap::TLikelihoodOptions	likelihoodOpts;	//!< Customizable initial options.
			CBeaconMap::TInsertionOptions	insertionOpts; 	//!< Customizable initial options.

		} beaconMap_options;

		/** Specific options for height grid maps (mrpt::slam::CHeightGridMap2D)
		  */
		struct SLAM_IMPEXP CHeightGridMap2DOptions
		{
			CHeightGridMap2DOptions();	//!< Default values loader

			float	min_x,max_x,min_y,max_y,resolution;	//!< See CHeightGridMap2D::CHeightGridMap2D
			CHeightGridMap2D::TMapRepresentation	mapType;	//!< The kind of map representation (see CHeightGridMap2D::CHeightGridMap2D)
			CHeightGridMap2D::TInsertionOptions	insertionOpts;	//!< Customizable initial options.
		} heightMap_options;

		/** Specific options for height grid maps (mrpt::slam::CReflectivityGridMap2D)
		  */
		struct SLAM_IMPEXP CReflectivityGridMap2DOptions
		{
			CReflectivityGridMap2DOptions();	//!< Default values loader

			float	min_x,max_x,min_y,max_y,resolution;	//!< See CReflectivityGridMap2DOptions::CReflectivityGridMap2DOptions
			CReflectivityGridMap2D::TInsertionOptions	insertionOpts;	//!< Customizable initial options.
		} reflectivityMap_options;

		/** Specific options for coloured point maps (mrpt::slam::CPointsMap)
		  */
		struct SLAM_IMPEXP CColouredPointsMapOptions
		{
			CColouredPointsMapOptions();	//!< Default values loader
			CPointsMap::TInsertionOptions	insertionOpts;	//!< Customizable initial options for loading the class' own defaults.
			CPointsMap::TLikelihoodOptions  likelihoodOpts; //!< 	//!< Customizable initial likelihood options
			CColouredPointsMap::TColourOptions colourOpts;	//!< Customizable initial options for loading the class' own defaults. */
		} colouredPointsMapOptions_options;

		/** Specific options for coloured point maps (mrpt::slam::CPointsMap)
		  */
		struct SLAM_IMPEXP CWeightedPointsMapOptions
		{
			CWeightedPointsMapOptions();	//!< Default values loader
			CPointsMap::TInsertionOptions	insertionOpts;	//!< Customizable initial options for loading the class' own defaults.
			CPointsMap::TLikelihoodOptions  likelihoodOpts; //!< 	//!< Customizable initial likelihood options
		} weightedPointsMapOptions_options;
	};

	/** A set of TMetricMapInitializer structures, passed to the constructor CMultiMetricMap::CMultiMetricMap
	  *  See the comments for TSetOfMetricMapInitializers::loadFromConfigFile, and "CMultiMetricMap::setListOfMaps" for
	  *   effectively creating the list of desired maps.
	  * \sa CMultiMetricMap::CMultiMetricMap, utils::CLoadableOptions
	  */
	class SLAM_IMPEXP TSetOfMetricMapInitializers : public utils::CLoadableOptions
	{
	protected:
		std::deque<TMetricMapInitializer>	m_list;

	public:
		size_t size() const { return m_list.size(); }
		void push_back( const TMetricMapInitializer &o ) { m_list.push_back(o); }

		typedef std::deque<TMetricMapInitializer>::iterator  iterator;
		typedef std::deque<TMetricMapInitializer>::const_iterator  const_iterator;

		iterator begin()   { return m_list.begin(); }
		const_iterator begin() const  { return m_list.begin(); }

		iterator end()   { return m_list.end(); }
		const_iterator end() const  { return m_list.end(); }

		void clear() { m_list.clear(); }


		TSetOfMetricMapInitializers() : m_list(), options()
		{}


		/** This options will be loaded when creating the set of maps in CMultiMetricMap (See CMultiMetricMap::TOptions)
		  */
		CMultiMetricMap::TOptions	options;

		/** Loads the configuration for the set of internal maps from a textual definition in an INI-like file.
		  *  The format of the ini file is defined in utils::CConfigFile. The list of maps and their options
		  *   will be loaded from a handle of sections:
		  *
		  *  \code
		  * [<sectionName>]
		  *  // Creation of maps:
		  *  occupancyGrid_count=<Number of mrpt::slam::COccupancyGridMap2D maps>
		  *  gasGrid_count=<Number of mrpt::slam::CGasConcentrationGridMap2D maps>
		  *  wifiGrid_count=<Number of mrpt::slam::CWirelessPowerGridMap2D maps>
		  *  landmarksMap_count=<0 or 1, for creating a mrpt::slam::CLandmarksMap map>
		  *  beaconMap_count=<0 or 1, for creating a mrpt::slam::CBeaconMap map>
		  *  pointsMap_count=<Number of mrpt::slam::CSimplePointsMap map>
		  *  heightMap_count=<Number of mrpt::slam::CHeightGridMap2D maps>
		  *  reflectivityMap_count=<Number of mrpt::slam::CReflectivityGridMap2D maps>
		  *  colourPointsMap_count=<0 or 1, for creating a mrpt::slam::CColouredPointsMap map>
		  *  weightedPointsMap_count=<0 or 1, for creating a mrpt::slam::CWeightedPointsMap map>
		  *
		  *  // Selection of map for likelihood. Either a numeric value or the textual enum
		  *  //   enum value of slam::CMultiMetricMap::TOptions::TMapSelectionForLikelihood (e.g: either "-1" or "fuseAll", ect...)
		  *  likelihoodMapSelection = -1
		  *
		  *  // Enables (1 or "true") / Disables (0 or "false") insertion into specific maps (Defaults are all "true"):
		  *  enableInsertion_pointsMap=<0/1>
		  *  enableInsertion_landmarksMap=<0/1>
		  *  enableInsertion_gridMaps=<0/1>
		  *  enableInsertion_gasGridMaps=<0/1>
		  *  enableInsertion_wifiGridMaps=<0/1>
		  *  enableInsertion_beaconMap=<0/1>
		  *  enableInsertion_heightMap=<0/1>
		  *  enableInsertion_reflectivityMap=<0/1>
		  *  enableInsertion_colourPointsMap=<0/1>
		  *  enableInsertion_weightedPointsMap=<0/1>
		  *
		  * // ====================================================
		  * //  Creation Options for OccupancyGridMap ##:
		  * [<sectionName>+"_occupancyGrid_##_creationOpts"]
		  *  min_x=<value>
		  *  max_x=<value>
		  *  min_y=<value>
		  *  max_y=<value>
		  *  resolution=<value>
		  *
		  * // Insertion Options for OccupancyGridMap ##:
		  * [<sectionName>+"_occupancyGrid_##_insertOpts"]
		  *  <See COccupancyGridMap2D::TInsertionOptions>
		  *
		  * // Likelihood Options for OccupancyGridMap ##:
		  * [<sectionName>+"_occupancyGrid_##_likelihoodOpts"]
		  *  <See COccupancyGridMap2D::TLikelihoodOptions>
		  *
		  *
		  * // ====================================================
		  * // Insertion Options for CSimplePointsMap ##:
		  * [<sectionName>+"_pointsMap_##_insertOpts"]
		  *  <See CPointsMap::TInsertionOptions>
		  *
		  * // Likelihood Options for CSimplePointsMap ##:
		  * [<sectionName>+"_pointsMap_##_likelihoodOpts"]
		  *  <See CPointsMap::TLikelihoodOptions>
		  *
		  *
		  * // ====================================================
		  * // Creation Options for CGasConcentrationGridMap2D ##:
		  * [<sectionName>+"_gasGrid_##_creationOpts"]
		  *  mapType= <0-1> ; See CGasConcentrationGridMap2D::CGasConcentrationGridMap2D
		  *  min_x=<value>
		  *  max_x=<value>
		  *  min_y=<value>
		  *  max_y=<value>
		  *  resolution=<value>
		  *
		  * // Insertion Options for CGasConcentrationGridMap2D ##:
		  * [<sectionName>+"_gasGrid_##_insertOpts"]
		  *  <See CGasConcentrationGridMap2D::TInsertionOptions>




		  * // ====================================================
		  * // Creation Options for CWirelessPowerGridMap2D ##:
		  * [<sectionName>+"_wifiGrid_##_creationOpts"]
		  *  mapType= <0-1> ; See CWirelessPowerGridMap2D::CWirelessPowerGridMap2D
		  *  min_x=<value>
		  *  max_x=<value>
		  *  min_y=<value>
		  *  max_y=<value>
		  *  resolution=<value>
		  *
		  * // Insertion Options for CWirelessPowerGridMap2D ##:
		  * [<sectionName>+"_wifiGrid_##_insertOpts"]
		  *  <See CWirelessPowerGridMap2D::TInsertionOptions>


		  *
		  *
		  * // ====================================================
		  * // Creation Options for CLandmarksMap ##:
		  * [<sectionName>+"_landmarksMap_##_creationOpts"]
		  *  nBeacons=<# of beacons>
		  *  beacon_001_ID=67		; The ID and 3D coordinates of each beacon
		  *  beacon_001_X=<x>
		  *  beacon_001_Y=<x>
		  *  beacon_001_Z=<x>
		  *
		  * // Insertion Options for CLandmarksMap ##:
		  * [<sectionName>+"_landmarksMap_##_insertOpts"]
		  *  <See CLandmarksMap::TInsertionOptions>
		  *
		  * // Likelihood Options for CLandmarksMap ##:
		  * [<sectionName>+"_landmarksMap_##_likelihoodOpts"]
		  *  <See CLandmarksMap::TLikelihoodOptions>
		  *
		  *
		  * // ====================================================
		  * // Insertion Options for CBeaconMap ##:
		  * [<sectionName>+"_beaconMap_##_insertOpts"]
		  *  <See CBeaconMap::TInsertionOptions>
		  *
		  * // Likelihood Options for CBeaconMap ##:
		  * [<sectionName>+"_beaconMap_##_likelihoodOpts"]
		  *  <See CBeaconMap::TLikelihoodOptions>
		  *
		  * // ====================================================
		  * // Creation Options for HeightGridMap ##:
		  * [<sectionName>+"_heightGrid_##_creationOpts"]
		  *  mapType= <0-1> // See CHeightGridMap2D::CHeightGridMap2D
		  *  min_x=<value>
		  *  max_x=<value>
		  *  min_y=<value>
		  *  max_y=<value>
		  *  resolution=<value>
		  *
		  * // Insertion Options for HeightGridMap ##:
		  * [<sectionName>+"_heightGrid_##_insertOpts"]
		  *  <See CHeightGridMap2D::TInsertionOptions>
		  *
		  *
		  * // ====================================================
		  * // Creation Options for ReflectivityGridMap ##:
		  * [<sectionName>+"_reflectivityGrid_##_creationOpts"]
		  *  min_x=<value>  // See CReflectivityGridMap2D::CReflectivityGridMap2D
		  *  max_x=<value>
		  *  min_y=<value>
		  *  max_y=<value>
		  *  resolution=<value>
		  *
		  * // Insertion Options for HeightGridMap ##:
		  * [<sectionName>+"_reflectivityGrid_##_insertOpts"]
		  *  <See CReflectivityGridMap2D::TInsertionOptions>
		  *
		  *
		  * // ====================================================
		  * // Insertion Options for CColouredPointsMap ##:
		  * [<sectionName>+"_colourPointsMap_##_insertOpts"]
		  *  <See CPointsMap::TInsertionOptions>
		  *
		  *
		  * // Color Options for CColouredPointsMap ##:
		  * [<sectionName>+"_colourPointsMap_##_colorOpts"]
		  *  <See CColouredPointsMap::TColourOptions>
		  *
		  * // Likelihood Options for CSimplePointsMap ##:
		  * [<sectionName>+"_colourPointsMap_##_likelihoodOpts"]
		  *  <See CPointsMap::TLikelihoodOptions>
		  *
		  *
		  * // ====================================================
		  * // Insertion Options for CWeightedPointsMap ##:
		  * [<sectionName>+"_weightedPointsMap_##_insertOpts"]
		  *  <See CPointsMap::TInsertionOptions>
		  *
		  *
		  * // Likelihood Options for CWeightedPointsMap ##:
		  * [<sectionName>+"_weightedPointsMap_##_likelihoodOpts"]
		  *  <See CPointsMap::TLikelihoodOptions>
		  *
		  *  \endcode
		  *
		  *  Where:
		  *		- ##: Represents the index of the map (e.g. "00","01",...)
		  *		- By default, the variables into each "TOptions" structure of the maps are defined in textual form by the same name of the corresponding C++ variable (e.g. "float resolution;" -> "resolution=0.10")
		  *
		  * \note Examples of map definitions can be found in the '.ini' files provided in the demo directories: "share/mrpt/config-files/"
		  */
		void  loadFromConfigFile(
			const mrpt::utils::CConfigFileBase  &source,
			const std::string &sectionName);

		/** This method dumps the options of the multi-metric map AND those of every internal map.
		  */
		void  dumpToTextStream(CStream	&out) const;
	};

	} // End of namespace


	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<slam::CMultiMetricMap::TOptions::TMapSelectionForLikelihood>
		{
			typedef slam::CMultiMetricMap::TOptions::TMapSelectionForLikelihood enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(slam::CMultiMetricMap::TOptions::mapFuseAll,   "mapFuseAll");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapGrid,      "mapGrid");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapPoints,    "mrSimpleAverage");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapLandmarks, "mapLandmarks");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapGasGrid,   "mapGasGrid");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapWifiGrid,   "mapWifiGrid");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapBeacon,    "mapBeacon");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapHeight,    "mapHeight");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapColourPoints, "mapColourPoints");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapReflectivity, "mapReflectivity");
				m_map.insert(slam::CMultiMetricMap::TOptions::mapWeightedPoints, "mapWeightedPoints");
			}
		};
	} // End of namespace



} // End of namespace

#endif
