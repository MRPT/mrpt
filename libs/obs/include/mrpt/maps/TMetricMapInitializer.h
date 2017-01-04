/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CObject.h> // For TRuntimeClassId
#include <mrpt/maps/TMetricMapTypesRegistry.h>
#include <mrpt/maps/metric_map_types.h>
#include <deque>
#include <mrpt/obs/link_pragmas.h>

namespace mrpt
{
	namespace maps
	{
		class TSetOfMetricMapInitializers;

		/** Virtual base for specifying the kind and parameters of one map (normally, to be inserted into mrpt::maps::CMultiMetricMap)
		  *  See `mrpt::maps::TSetOfMetricMapInitializers::loadFromConfigFile()` as an easy way of initialize this object, or
		  *  construct with the factory methods `<metric_map_class>::MapDefinition()` and `TMetricMapInitializer::factory()`
		  * 
		  * \sa TSetOfMetricMapInitializers, mrpt::maps::CMultiMetricMap
		  * \ingroup mrpt_obs_grp
		  */
		struct OBS_IMPEXP TMetricMapInitializer : public mrpt::utils::CLoadableOptions
		{
			friend class TSetOfMetricMapInitializers;

			/** Common params for all maps: These are automatically set in TMetricMapTypesRegistry::factoryMapObjectFromDefinition()  */
			mrpt::maps::TMapGenericParams  genericMapParams;

			/** Load all params from a config file/source. For examples and format, read the docs of mrpt::maps::CMultiMetricMap 
			  * Typical section names:
			  *  - `<sectionNamePrefix>_creationOpts`
			  *  - `<sectionNamePrefix>_insertOpts`
			  *  - `<sectionNamePrefix>_likelihoodOpts`
			  */
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &sectionNamePrefix) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			/** Query the map type (C++ class), as set by the factory method MapDefinition() */
			const mrpt::utils::TRuntimeClassIdPtr & getMetricMapClassType() const { return metricMapClassType; }

			/** Looks up in the registry of known map types and call the corresponding `<metric_map_class>::MapDefinition()`. */
			static TMetricMapInitializer* factory(const std::string &mapClassName);			

		protected:
			TMetricMapInitializer(const mrpt::utils::TRuntimeClassId* classID );
			const mrpt::utils::TRuntimeClassIdPtr metricMapClassType; //!< Derived classes set this to CLASS_ID(< class >) where < class > is any CMetricMap derived class.

			/** Load all map-specific params*/
			virtual void  loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase  &source, const std::string &sectionNamePrefix) = 0;
			virtual void  dumpToTextStream_map_specific(mrpt::utils::CStream	&out) const = 0;
		}; // end TMetricMapInitializer

		typedef stlplus::smart_ptr_clone<TMetricMapInitializer> TMetricMapInitializerPtr; //!< Smart pointer to TMetricMapInitializer 

		/** A set of TMetricMapInitializer structures, passed to the constructor CMultiMetricMap::CMultiMetricMap
		  *  See the comments for TSetOfMetricMapInitializers::loadFromConfigFile, and "CMultiMetricMap::setListOfMaps" for
		  *   effectively creating the list of desired maps.
		  * \sa CMultiMetricMap::CMultiMetricMap, utils::CLoadableOptions
		  * \ingroup mrpt_obs_grp
		  */
		class OBS_IMPEXP TSetOfMetricMapInitializers : public mrpt::utils::CLoadableOptions
		{
		protected:
			std::deque<TMetricMapInitializerPtr>	m_list;

		public:
			TSetOfMetricMapInitializers() : m_list()
			{}

			template <typename MAP_DEFINITION>
			void push_back( const MAP_DEFINITION &o ) { m_list.push_back( TMetricMapInitializerPtr(new MAP_DEFINITION(o)) ); }

			void push_back( const TMetricMapInitializerPtr &o ) { m_list.push_back(o); }

			size_t size() const { return m_list.size(); }
			typedef std::deque<TMetricMapInitializerPtr>::iterator  iterator;
			typedef std::deque<TMetricMapInitializerPtr>::const_iterator  const_iterator;
			iterator begin()       { return m_list.begin(); }
			iterator end()         { return m_list.end(); }
			const_iterator begin() const { return m_list.begin(); }
			const_iterator end() const   { return m_list.end(); }
			void clear() { m_list.clear(); }


			/** Loads the configuration for the set of internal maps from a textual definition in an INI-like file.
			  *  The format of the ini file is defined in utils::CConfigFile. The list of maps and their options
			  *   will be loaded from a handle of sections:
			  *
			  *  \code
			  * [<sectionName>]
			  *  // Creation of maps:
			  *  occupancyGrid_count=<Number of mrpt::maps::COccupancyGridMap2D maps>
			  *  octoMap_count=<Number of mrpt::maps::COctoMap maps>
			  *  colourOctoMap_count=<Number of mrpt::slam::CColourOctoMap maps>
			  *  gasGrid_count=<Number of mrpt::maps::CGasConcentrationGridMap2D maps>
			  *  wifiGrid_count=<Number of mrpt::maps::CWirelessPowerGridMap2D maps>
			  *  landmarksMap_count=<0 or 1, for creating a mrpt::maps::CLandmarksMap map>
			  *  beaconMap_count=<0 or 1, for creating a mrpt::maps::CBeaconMap map>
			  *  pointsMap_count=<Number of mrpt::maps::CSimplePointsMap map>
			  *  heightMap_count=<Number of mrpt::maps::CHeightGridMap2D maps>
			  *  reflectivityMap_count=<Number of mrpt::maps::CReflectivityGridMap2D maps>
			  *  colourPointsMap_count=<0 or 1, for creating a mrpt::maps::CColouredPointsMap map>
			  *  weightedPointsMap_count=<0 or 1, for creating a mrpt::maps::CWeightedPointsMap map>
			  *
			  * // ====================================================
			  * //  Creation Options for OccupancyGridMap ##:
			  * [<sectionName>+"_occupancyGrid_##_creationOpts"]
			  *  min_x=<value>
			  *  max_x=<value>
			  *  min_y=<value>
			  *  max_y=<value>
			  *  resolution=<value>
			  *  # Common params for all maps:
			  *  #enableSaveAs3DObject        = {true|false}
			  *  #enableObservationLikelihood = {true|false}
			  *  #enableObservationInsertion  = {true|false}
			  *
			  * // Insertion Options for OccupancyGridMap ##:
			  * [<sectionName>+"_occupancyGrid_##_insertOpts"]
			  *  <See COccupancyGridMap2D::TInsertionOptions>
			  *
			  * // Likelihood Options for OccupancyGridMap ##:
			  * [<sectionName>+"_occupancyGrid_##_likelihoodOpts"]
			  *  <See COccupancyGridMap2D::TLikelihoodOptions>
			  *
			  * // ====================================================
			  * //  Creation Options for OctoMap ##:
			  * [<sectionName>+"_octoMap_##_creationOpts"]
			  *  resolution=<value>
			  *
			  * // Insertion Options for OctoMap ##:
			  * [<sectionName>+"_octoMap_##_insertOpts"]
			  *  <See COctoMap::TInsertionOptions>
			  *
			  * // Likelihood Options for OctoMap ##:
			  * [<sectionName>+"_octoMap_##_likelihoodOpts"]
			  *  <See COctoMap::TLikelihoodOptions>
			  *
			  * // ====================================================
			  * //  Creation Options for ColourOctoMap ##:
			  * [<sectionName>+"_colourOctoMap_##_creationOpts"]
			  *  resolution=<value>
			  *
			  * // Insertion Options for ColourOctoMap ##:
			  * [<sectionName>+"_colourOctoMap_##_insertOpts"]
			  *  <See CColourOctoMap::TInsertionOptions>
			  *
			  * // Likelihood Options for ColourOctoMap ##:
			  * [<sectionName>+"_colourOctoMap_##_likelihoodOpts"]
			  *  <See CColourOctoMap::TLikelihoodOptions>
			  *
			  *
			  * // ====================================================
			  * // Insertion Options for mrpt::maps::CSimplePointsMap ##:
			  * [<sectionName>+"_pointsMap_##_insertOpts"]
			  *  <See CPointsMap::TInsertionOptions>
			  *
			  * // Likelihood Options for mrpt::maps::CSimplePointsMap ##:
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
			  *
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
			  * // Likelihood Options for mrpt::maps::CSimplePointsMap ##:
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
			void  loadFromConfigFile(const mrpt::utils::CConfigFileBase  &source,const std::string &sectionName) MRPT_OVERRIDE;

			/** This method dumps the options of the multi-metric map AND those of every internal map. */
			void  dumpToTextStream(mrpt::utils::CStream	&out) const MRPT_OVERRIDE;
		};


	} // End of namespace
} // End of namespace

