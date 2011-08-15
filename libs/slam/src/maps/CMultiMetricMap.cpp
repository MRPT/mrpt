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

#include <mrpt/slam.h>   // Precompiled headers

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/utils/CStartUpClassesRegister.h>
#include <mrpt/utils/metaprogramming.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::utils::metaprogramming;

IMPLEMENTS_SERIALIZABLE( CMultiMetricMap, CMetricMap, mrpt::slam )


extern CStartUpClassesRegister  mrpt_slam_class_reg;
const int dumm = mrpt_slam_class_reg.do_nothing(); // Avoid compiler removing this class in static linking

// ------------------------------------------------------------------------
// A few words explaining how all this works:
//  The main hub for operating with all the maps in the internal list
//   if MapExecutor.
//
// All operations go thru MapExecutor::run<OP>() with OP being one of the
//  possible map operations (clear, matching, likelihood, etc.). The
//  idea is that when adding new map types to the internal list of
//  CMultiMetricMap, *only* "MapExecutor" methods must be updated.
// (The only exception are readFromStream() & writeToStream())
//
// The map-specific operations all go into template specializations of
//  other helper structures or in overloaded methods.
//                                                 JLBC (7-AUG-2011)
// ------------------------------------------------------------------------

struct MapExecutor {
	// Apply operation to maps in the same order as declared in CMultiMetricMap.h:
	template <typename OP>
	static void run(const CMultiMetricMap &_mmm, OP &op)
	{
		MRPT_START
		CMultiMetricMap &mmm = const_cast<CMultiMetricMap&>(_mmm);  // This is to avoid duplicating "::run()" for const and non-const.

		for_each( mmm.m_pointsMaps.begin(),mmm.m_pointsMaps.end(), op );
		for_each( mmm.m_gridMaps.begin(),mmm.m_gridMaps.end(),  op );
		for_each( mmm.m_gasGridMaps.begin(),mmm.m_gasGridMaps.end(),  op );
		for_each( mmm.m_wifiGridMaps.begin(),mmm.m_wifiGridMaps.end(),  op );
		for_each( mmm.m_heightMaps.begin(),mmm.m_heightMaps.end(),  op );
		for_each( mmm.m_reflectivityMaps.begin(),mmm.m_reflectivityMaps.end(), op );
		op(mmm.m_colourPointsMap);
		op(mmm.m_weightedPointsMap);
		op(mmm.m_landmarksMap);
		op(mmm.m_beaconMap);

		MRPT_END
	}
	// Apply operation to the vectors (or deques) contianing maps:
	template <typename OP>
	static void runOnVectors(const CMultiMetricMap &_mmm, OP &op)
	{
		MRPT_START
		CMultiMetricMap &mmm = const_cast<CMultiMetricMap&>(_mmm);  // This is to avoid duplicating "::run()" for const and non-const.

		op( mmm.m_pointsMaps );
		op( mmm.m_gridMaps );
		op( mmm.m_gasGridMaps );
		op( mmm.m_wifiGridMaps );
		op( mmm.m_heightMaps );
		op( mmm.m_reflectivityMaps );

		MRPT_END
	}
	// Copy all smart pointers:
	static void copyAll(const CMultiMetricMap &other, CMultiMetricMap &mmm) {
		mmm.m_pointsMaps = other.m_pointsMaps;
		mmm.m_gridMaps = other.m_gridMaps;
		mmm.m_gasGridMaps = other.m_gasGridMaps;
		mmm.m_wifiGridMaps = other.m_wifiGridMaps;
		mmm.m_heightMaps = other.m_heightMaps;
		mmm.m_reflectivityMaps = other.m_reflectivityMaps;
		mmm.m_colourPointsMap = other.m_colourPointsMap;
		mmm.m_weightedPointsMap = other.m_weightedPointsMap;
		mmm.m_landmarksMap = other.m_landmarksMap;
		mmm.m_beaconMap = other.m_beaconMap;
	}
};  // end of MapExecutor

// ------------------- Begin of map-operations helper templates -------------------
struct MapVectorClearer
{
	template<typename T>
	inline void operator()(T &container) {
		container.clear();
	}
};

// Auxiliary methods are in this base helper struct:
struct MapTraits
{
	const CMultiMetricMap & mmm;
	MapTraits(const CMultiMetricMap & m) : mmm(m) { }

	inline bool isUsedLik(CSimplePointsMapPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapPoints ) );
	}
	inline bool isUsedLik(COccupancyGridMap2DPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapGrid ) );
	}
	inline bool isUsedLik(CGasConcentrationGridMap2DPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapGasGrid ) );
	}
	inline bool isUsedLik(CWirelessPowerGridMap2DPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapWifiGrid ) );
	}
	inline bool isUsedLik(CHeightGridMap2DPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapHeight ) );
	}
	inline bool isUsedLik(CReflectivityGridMap2DPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapReflectivity ) );
	}
	inline bool isUsedLik(CColouredPointsMapPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapColourPoints ) );
	}
	inline bool isUsedLik(CWeightedPointsMapPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapWeightedPoints) );
	}
	inline bool isUsedLik(CLandmarksMapPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapLandmarks ) );
	}
	inline bool isUsedLik(CBeaconMapPtr &ptr) {
		return (ptr.present() && (mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapFuseAll ||
				mmm.options.likelihoodMapSelection==CMultiMetricMap::TOptions::mapBeacon ) );
	}

	// --------------------
	inline bool isUsedInsert(CSimplePointsMapPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_pointsMap; }
	inline bool isUsedInsert(COccupancyGridMap2DPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_gridMaps; }
	inline bool isUsedInsert(CGasConcentrationGridMap2DPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_gasGridMaps; }
	inline bool isUsedInsert(CWirelessPowerGridMap2DPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_wifiGridMaps; }
	inline bool isUsedInsert(CHeightGridMap2DPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_heightMaps; }
	inline bool isUsedInsert(CReflectivityGridMap2DPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_reflectivityMaps; }
	inline bool isUsedInsert(CColouredPointsMapPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_colourPointsMaps; }
	inline bool isUsedInsert(CWeightedPointsMapPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_weightedPointsMaps; }
	inline bool isUsedInsert(CLandmarksMapPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_landmarksMap; }
	inline bool isUsedInsert(CBeaconMapPtr &ptr) { return ptr.present() && mmm.options.enableInsertion_beaconMap; }

}; // end of MapTraits

struct MapComputeLikelihood : public MapTraits
{
	const CObservation    * obs;
	const CPose3D         & takenFrom;
	double                & total_log_lik;

	MapComputeLikelihood(const CMultiMetricMap &m,const CObservation * _obs, const CPose3D & _takenFrom, double & _total_log_lik) :
		MapTraits(m),
		obs(_obs), takenFrom(_takenFrom),
		total_log_lik(_total_log_lik)
	{
		total_log_lik=0;
	}

	template <typename PTR>
	inline void operator()(PTR &ptr) {
		if (isUsedLik(ptr))
			total_log_lik+=ptr->computeObservationLikelihood(obs,takenFrom);
	}

}; // end of MapComputeLikelihood

struct MapCanComputeLikelihood  : public MapTraits
{
	const CObservation    * obs;
	bool                  & can;

	MapCanComputeLikelihood(const CMultiMetricMap &m,const CObservation * _obs, bool & _can) :
		MapTraits(m),
		obs(_obs),
		can(_can)
	{
		can = false;
	}

	template <typename PTR>
	inline void operator()(PTR &ptr) {
		if (isUsedLik(ptr))
			can = can || ptr->canComputeObservationLikelihood(obs);
	}

}; // end of MapCanComputeLikelihood

struct MapInsertObservation : public MapTraits
{
	const CObservation    * obs;
	const CPose3D         * robot_pose;
	int                   & total_insert;

	MapInsertObservation(const CMultiMetricMap &m,const CObservation * _obs, const CPose3D * _robot_pose, int & _total_insert) :
		MapTraits(m),
		obs(_obs), robot_pose(_robot_pose),
		total_insert(_total_insert)
	{
		total_insert = 0;
	}

	template <typename PTR>
	inline void operator()(PTR &ptr) {
		if (isUsedInsert(ptr))
		{
			bool ret = ptr->insertObservation(obs,robot_pose);
			if (ret) total_insert++;
		}
	}
}; // end of MapInsertObservation

struct MapIsEmpty
{
	bool & is_empty;

	MapIsEmpty(bool & _is_empty) : is_empty(_is_empty)
	{
		is_empty = true;
	}

	template <typename PTR>
	inline void operator()(PTR &ptr) {
		if (ptr.present())
			is_empty = is_empty && ptr->isEmpty();
	}
}; // end of MapInsertObservation

// ------------------- End of map-operations helper templates -------------------



/*---------------------------------------------------------------
			Constructor
  ---------------------------------------------------------------*/
CMultiMetricMap::CMultiMetricMap(
	const TSetOfMetricMapInitializers		*initializers,
	const mrpt::slam::CMultiMetricMap::TOptions	*opts) :
		m_ID(0)
{
	MRPT_START
	MRPT_UNUSED_PARAM(dumm);

	// Create maps
	setListOfMaps(initializers);

	// Do we have initial options?
	if (opts) options = *opts;

	MRPT_END
}

/*---------------------------------------------------------------
			copy constructor
  ---------------------------------------------------------------*/
CMultiMetricMap::CMultiMetricMap(const mrpt::slam::CMultiMetricMap &other ) :
	m_ID(0)
{
	*this = other;	// Call the "=" operator
}

/*---------------------------------------------------------------
			setListOfMaps
  ---------------------------------------------------------------*/
void  CMultiMetricMap::setListOfMaps(
	const mrpt::slam::TSetOfMetricMapInitializers	*initializers )
{
	MRPT_START

	m_ID = 0;

	// Erase current list of maps:
	deleteAllMaps();

	// Do we have any initializer?
	if (initializers!=NULL)
	{
		// The set of options of this class:
		options = initializers->options;


		// Process each entry in the "initializers" and create maps accordingly:
		for (TSetOfMetricMapInitializers::const_iterator it = initializers->begin();it!=initializers->end();++it)
		{
			if ( it->metricMapClassType == CLASS_ID(COccupancyGridMap2D) )
			{
				// -------------------------------------------------------
				//						GRID MAPS
				// -------------------------------------------------------
				COccupancyGridMap2DPtr newGridmap = COccupancyGridMap2DPtr( new COccupancyGridMap2D(
					it->occupancyGridMap2D_options.min_x,
					it->occupancyGridMap2D_options.max_x,
					it->occupancyGridMap2D_options.min_y,
					it->occupancyGridMap2D_options.max_y,
					it->occupancyGridMap2D_options.resolution ) );

				newGridmap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;

				newGridmap->insertionOptions = it->occupancyGridMap2D_options.insertionOpts;
				newGridmap->likelihoodOptions= it->occupancyGridMap2D_options.likelihoodOpts;

				m_gridMaps.push_back( newGridmap );
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CGasConcentrationGridMap2D) )
			{
				// -------------------------------------------------------
				//			GAS CONCENTRATION GRID MAPS
				// -------------------------------------------------------
				CGasConcentrationGridMap2DPtr newGridmap = CGasConcentrationGridMap2DPtr( new CGasConcentrationGridMap2D(
					it->gasGridMap_options.mapType,
					it->gasGridMap_options.min_x,
					it->gasGridMap_options.max_x,
					it->gasGridMap_options.min_y,
					it->gasGridMap_options.max_y,
					it->gasGridMap_options.resolution ) );

				newGridmap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;
				newGridmap->insertionOptions = it->gasGridMap_options.insertionOpts;

				// IMPORTANT: Reinitialize the map with the new parameters:
				newGridmap->clear();

				m_gasGridMaps.push_back( newGridmap );
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CWirelessPowerGridMap2D) )
			{
				// -------------------------------------------------------
				//			WIRELESS POWER GRID MAPS
				// -------------------------------------------------------
				CWirelessPowerGridMap2DPtr newGridmap = CWirelessPowerGridMap2DPtr( new CWirelessPowerGridMap2D(
					it->wifiGridMap_options.mapType,
					it->wifiGridMap_options.min_x,
					it->wifiGridMap_options.max_x,
					it->wifiGridMap_options.min_y,
					it->wifiGridMap_options.max_y,
					it->wifiGridMap_options.resolution ) );

				newGridmap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;
				newGridmap->insertionOptions = it->wifiGridMap_options.insertionOpts;

				// IMPORTANT: Reinitialize the map with the new parameters:
				newGridmap->clear();

				m_wifiGridMaps.push_back( newGridmap );
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CHeightGridMap2D) )
			{
				// -------------------------------------------------------
				//			HEIGHT GRID MAPS
				// -------------------------------------------------------
				CHeightGridMap2DPtr newGridmap = CHeightGridMap2DPtr( new CHeightGridMap2D(
					it->heightMap_options.mapType,
					it->heightMap_options.min_x,
					it->heightMap_options.max_x,
					it->heightMap_options.min_y,
					it->heightMap_options.max_y,
					it->heightMap_options.resolution ) );

				newGridmap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;
				newGridmap->insertionOptions = it->heightMap_options.insertionOpts;

				m_heightMaps.push_back( newGridmap );
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CReflectivityGridMap2D) )
			{
				// -------------------------------------------------------
				//			REFLECTIVITY GRID MAPS
				// -------------------------------------------------------
				CReflectivityGridMap2DPtr newGridmap = CReflectivityGridMap2DPtr( new CReflectivityGridMap2D(
					it->reflectivityMap_options.min_x,
					it->reflectivityMap_options.max_x,
					it->reflectivityMap_options.min_y,
					it->reflectivityMap_options.max_y,
					it->reflectivityMap_options.resolution ) );

				newGridmap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;
				newGridmap->insertionOptions = it->reflectivityMap_options.insertionOpts;

				m_reflectivityMaps.push_back( newGridmap );
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CSimplePointsMap) )
			{
				// -------------------------------------------------------
				//						POINTS MAPS
				// -------------------------------------------------------
				CSimplePointsMapPtr newPointsMap = CSimplePointsMap::Create();
				newPointsMap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;
				newPointsMap->insertionOptions = it->pointsMapOptions_options.insertionOpts;

				m_pointsMaps.push_back( newPointsMap );
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CColouredPointsMap) )
			{
				// -------------------------------------------------------
				//						COLOURED POINTS MAPS
				// -------------------------------------------------------
				CColouredPointsMapPtr newPointsMap = CColouredPointsMap::Create();
				newPointsMap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;
				newPointsMap->insertionOptions = it->colouredPointsMapOptions_options.insertionOpts;
				newPointsMap->colorScheme	   = it->colouredPointsMapOptions_options.colourOpts;

				m_colourPointsMap = newPointsMap;
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CWeightedPointsMap) )
			{
				// -------------------------------------------------------
				//				  WEIGHTED POINTS MAPS
				// -------------------------------------------------------
				CWeightedPointsMapPtr newPointsMap = CWeightedPointsMap::Create();
				newPointsMap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;
				newPointsMap->insertionOptions = it->weightedPointsMapOptions_options.insertionOpts;

				m_weightedPointsMap = newPointsMap;
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CLandmarksMap) )
			{
				// -------------------------------------------------------
				//					LANDMARKS MAPS
				// -------------------------------------------------------
				m_landmarksMap = CLandmarksMap::Create();

				for (std::deque<CMultiMetricMap::TPairIdBeacon>::const_iterator p = it->landmarksMap_options.initialBeacons.begin();p!=it->landmarksMap_options.initialBeacons.end();++p)
				{
					CLandmark	lm;

					lm.createOneFeature();
					lm.features[0]->type = featBeacon;

					lm.features[0]->ID = p->second;
					lm.ID = p->second;

					lm.pose_mean = p->first;

					lm.pose_cov_11=
					lm.pose_cov_22=
					lm.pose_cov_33=
					lm.pose_cov_12=
					lm.pose_cov_13=
					lm.pose_cov_23=square(0.01f);

					m_landmarksMap->landmarks.push_back( lm );
				}

				m_landmarksMap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;
				m_landmarksMap->insertionOptions = it->landmarksMap_options.insertionOpts;
				m_landmarksMap->likelihoodOptions = it->landmarksMap_options.likelihoodOpts;
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CBeaconMap) )
			{
				// -------------------------------------------------------
				//					SOG LANDMARKS MAPS
				// -------------------------------------------------------
				m_beaconMap = CBeaconMapPtr ( new CBeaconMap() );

				m_beaconMap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;
				m_beaconMap->likelihoodOptions = it->beaconMap_options.likelihoodOpts;
				m_beaconMap->insertionOptions = it->beaconMap_options.insertionOpts;
			}
			else
			{
				// -------------------------------------------------------
				//							ERROR
				// -------------------------------------------------------
				THROW_EXCEPTION("Unknown class ID found into initializers (Bug, unsoported map class, or a non-map class?)!!");
			}

		} // end for each "initializers"

	} // end if initializers!=NULL

	MRPT_END
}

/*---------------------------------------------------------------
					clear
  ---------------------------------------------------------------*/
void  CMultiMetricMap::internal_clear()
{
	ObjectClear op;
	MapExecutor::run(*this, op);
}

/*---------------------------------------------------------------
		Copy constructor
  ---------------------------------------------------------------*/
mrpt::slam::CMultiMetricMap & CMultiMetricMap::operator = ( const CMultiMetricMap &other )
{
	MRPT_START

	if (this == &other) return *this;			// Who knows! :-)

	options	  = other.options;
	m_ID	  = other.m_ID;

	// Copy all maps and then make_unique() to really duplicate the objects:
	MapExecutor::copyAll(other,*this);

	// invoke make_unique() operation on each smart pointer:
	ObjectMakeUnique op;
	MapExecutor::run(*this, op);

	return *this;

	MRPT_END
}

/*---------------------------------------------------------------
		Destructor
  ---------------------------------------------------------------*/
CMultiMetricMap::~CMultiMetricMap( )
{
	deleteAllMaps();
}

/*---------------------------------------------------------------
						deleteAllMaps
  ---------------------------------------------------------------*/
void  CMultiMetricMap::deleteAllMaps( )
{
	// invoke make_unique() operation on each smart pointer:
	MapVectorClearer op_vec_clear;
	MapExecutor::runOnVectors(*this, op_vec_clear);

	ObjectMakeUnique op_make_unique;
	MapExecutor::run(*this, op_make_unique);
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CMultiMetricMap::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 8;
	else
	{
		// Version 5: The options:
		out << options.enableInsertion_pointsMap
			<< options.enableInsertion_landmarksMap
			<< options.enableInsertion_gridMaps
			<< options.enableInsertion_gasGridMaps
			<< options.enableInsertion_wifiGridMaps
			<< options.enableInsertion_beaconMap
			<< options.enableInsertion_heightMaps	// Added in v6
			<< options.enableInsertion_reflectivityMaps; // Added in v8

		// The data
		uint32_t	i,n = static_cast<uint32_t>(m_gridMaps.size());

		// grid maps:
		// ----------------------
		out << n;
		for (i=0;i<n;i++)	out << *m_gridMaps[i];

		// Points maps:
		// ----------------------
		n = m_pointsMaps.size();
		out << n;
		for (i=0;i<n;i++)	out << *m_pointsMaps[i];

		// Landmarks maps:
		// ----------------------
		n = m_landmarksMap.present() ? 1:0;
		out << n;
		if (n)	out << *m_landmarksMap;

		// gas grid maps:
		// ----------------------
		n = static_cast<uint32_t>(m_gasGridMaps.size());
		out << n;
		for (i=0;i<n;i++)	out << *m_gasGridMaps[i];

		// wifi grid maps:
		// ----------------------
		n = static_cast<uint32_t>(m_wifiGridMaps.size());
		out << n;
		for (i=0;i<n;i++)	out << *m_wifiGridMaps[i];

		// Added in version 3:
		out << static_cast<uint32_t>(m_ID);

		// Added in version 4:

		// Beacons maps:
		// ----------------------
		n = m_beaconMap.present() ? 1:0;
		out << n;
		if (n)	out << *m_beaconMap;

		// Added in version 6:
		n = static_cast<uint32_t>(m_heightMaps.size());
		out << n;
		for (i=0;i<n;i++)	out << *m_heightMaps[i];

		// Added in version 8:
		n = static_cast<uint32_t>(m_reflectivityMaps.size());
		out << n;
		for (i=0;i<n;i++)	out << *m_reflectivityMaps[i];

		// Added in version 7:
		n = m_colourPointsMap.present() ? 1:0;
		out << n;
		if (n) out << *m_colourPointsMap;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CMultiMetricMap::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
		{
			uint32_t  n;

			// Version 5: The options:
			if (version>=5)
			{
				in  >> options.enableInsertion_pointsMap
					>> options.enableInsertion_landmarksMap
					>> options.enableInsertion_gridMaps
					>> options.enableInsertion_gasGridMaps
					>> options.enableInsertion_wifiGridMaps
					>> options.enableInsertion_beaconMap;

				if (version>=6)
					in >> options.enableInsertion_heightMaps;
				else options.enableInsertion_heightMaps=true;

				if (version>=8)
					in >> options.enableInsertion_reflectivityMaps;
				else options.enableInsertion_reflectivityMaps = true;

			}
			else
			{ } // Default!


			// grid maps:
			// ----------------------
			if (version>=2)
			{
				in >> n;
			}
			else
			{
				// Compatibility: Only 1 grid map!
				n = 1;
			}

			// Free previous grid maps:
			m_gridMaps.clear();

			// Load from stream:
			m_gridMaps.resize(n);
			for_each( m_gridMaps.begin(), m_gridMaps.end(), ObjectReadFromStream(&in) );

			// Points maps:
			// ----------------------
			if (version>=2)
						in >> n;
			else		n = 1;			// Compatibility: Always there is a points map!

			m_pointsMaps.clear();
			m_pointsMaps.resize(n);
			for_each(m_pointsMaps.begin(), m_pointsMaps.end(), ObjectReadFromStream(&in) );

			// Set the parent of the points map to "this":
			//for (std::deque<CSimplePointsMapPtr>::iterator it3=m_pointsMaps.begin();it3!=m_pointsMaps.end();it3++)
			//	(*it3)->m_parent = this;

			// Landmarks maps:
			// ----------------------
			if (version>=2)
						in >> n;
			else		n = 1;			// Compatibility: Always there is a points map!

			if (n && version>=1)
				in >> m_landmarksMap;		// If the pointer were NULL, a new object would be automatically created.
			else
			{
				// We haven't this kind of map:
				m_landmarksMap.clear_unique();
			}

			// Gas grid maps:
			// ----------------------
			if (version>=2)
						in >> n;
			else		n = 0;			// Compatibility: Previously there were no gas grid maps!

			// Free previous grid maps:
			m_gasGridMaps.clear();

			// Load from stream:
			m_gasGridMaps.resize(n);
			for_each(m_gasGridMaps.begin(), m_gasGridMaps.end(), ObjectReadFromStream(&in) );

			if (version>=3)
			{
				uint32_t	ID;
				in >> ID; m_ID = ID;
			}
			else	m_ID = 0;

			// Wifi grid maps:
			// ----------------------
			if (version>=2)
						in >> n;
			else		n = 0;			// Compatibility: Previously there were no gas grid maps!

			// Free previous grid maps:
			m_wifiGridMaps.clear();

			// Load from stream:
			m_wifiGridMaps.resize(n);
			for_each(m_wifiGridMaps.begin(), m_wifiGridMaps.end(), ObjectReadFromStream(&in) );

			if (version>=3)
			{
				uint32_t	ID;
				in >> ID; m_ID = ID;
			}
			else	m_ID = 0;

			// Landmarks SOG maps:
			// ----------------------
			if (version>=4)
						in >> n;
			else		n = 0;			// Compatibility: Previously there were no SOG LM maps!

			if (n && version>=4)
				in >> m_beaconMap;		// If the pointer were NULL, a new object would be automatically created.
			else
			{
				// We haven't this kind of map:
				m_beaconMap.clear_unique();
			}

			// Height maps (added in version 6)
			// --------------------------------------
			if (version>=6)
						in >> n;
			else		n = 0;			// Compatibility: Previously there were no height maps!

			// Free previous maps:
			m_heightMaps.clear();

			// Load from stream:
			m_heightMaps.resize(n);
			for_each( m_heightMaps.begin(), m_heightMaps.end(), ObjectReadFromStream(&in) );


			// Reflectivity maps (added in version 8)
			// --------------------------------------
			if (version>=8)
						in >> n;
			else		n = 0;			// Compatibility: Previously there were no such maps!

			// Free previous maps:
			m_reflectivityMaps.clear();

			// Load from stream:
			m_reflectivityMaps.resize(n);
			for_each( m_reflectivityMaps.begin(), m_reflectivityMaps.end(), ObjectReadFromStream(&in) );


			// Colour points maps (added in version 7)
			// --------------------------------------
			if (version>=7)
					in >> n;
			else	n = 0;			// Compatibility: Previously there were no height maps!

			// Free previous maps:
			if (n)
			{
					in >> m_colourPointsMap;
					//m_colourPointsMap->m_parent = this;
			}
			else	m_colourPointsMap.clear_unique();

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}


/*---------------------------------------------------------------
 Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
	takenFrom The robot's pose the observation is supposed to be taken from.
	obs The observation.
 This method returns a likelihood in the range [0,1].
 ---------------------------------------------------------------*/
double	 CMultiMetricMap::computeObservationLikelihood(
			const CObservation		*obs,
			const CPose3D			&takenFrom )
{
	double ret_log_lik;
	MapComputeLikelihood op_likelihood(*this,obs,takenFrom,ret_log_lik);

	MapExecutor::run(*this,op_likelihood);

	MRPT_CHECK_NORMAL_NUMBER(ret_log_lik)
	return ret_log_lik;
}

/*---------------------------------------------------------------
Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
\param obs The observation.
 ---------------------------------------------------------------*/
bool CMultiMetricMap::canComputeObservationLikelihood( const CObservation *obs )
{
	bool can_comp;

	MapCanComputeLikelihood op_can_likelihood(*this,obs,can_comp);
	MapExecutor::run(*this,op_can_likelihood);
	return can_comp;
}

/*---------------------------------------------------------------
				getNewStaticPointsRatio
Returns the ratio of points in a map which are new to the points map while falling into yet static cells of gridmap.
	points The set of points to check.
	takenFrom The pose for the reference system of points, in global coordinates of this hybrid map.
 ---------------------------------------------------------------*/
float  CMultiMetricMap::getNewStaticPointsRatio(
		CPointsMap		*points,
		CPose2D			&takenFrom )
{
	size_t						    i;
	size_t							nStaticPoints = 0;
	size_t							nTotalPoints = points->getPointsCount();
	CPoint2D								g,l;
	mrpt::utils::TMatchingPairList			correspondences;
	float									correspondencesRatio;
	mrpt::utils::TMatchingPairList::iterator	corrsIt;
	static CPose2D							nullPose(0,0,0);


	ASSERT_( m_gridMaps.size()>0 );

	// There must be points!
	if ( !nTotalPoints ) return 0.0f;

	// Used below:
	float	maxDistThreshold = 0.95f*m_gridMaps[0]->insertionOptions.maxDistanceInsertion;

	// Compute matching:
	m_gridMaps[0]->computeMatchingWith2D(
				points,
				takenFrom,
				m_gridMaps[0]->getResolution() * 2,
				0,
				nullPose,
				correspondences,
				correspondencesRatio,
				NULL,
				true);

//	gridMap->saveAsBitmapFile("debug_occ_grid.bmp");
//	points->save2D_to_text_file(std::string("debug_points.txt"));
//	printf("%u corrs, %f corrsRatio\n",correspondences.size(),correspondencesRatio);

	for (i=0;i<nTotalPoints;i++)
	{
		bool	hasCoor = false;
		// Has any correspondence?
		for (corrsIt=correspondences.begin();!hasCoor && corrsIt!=correspondences.end();corrsIt++)
			if (corrsIt->other_idx==i)
				hasCoor = true;

		if ( !hasCoor )
		{
			// The distance between the point and the robot: If it is farther than the insertion max. dist.
			//   it should not be consider as an static point!!
			points->getPoint(i,l);

			CPoint2D	temp = l - takenFrom;
			if ( temp.norm() < maxDistThreshold)
			{
				// A new point
				// ------------------------------------------
				// Translate point to global coordinates:
				g = takenFrom + l;

				if ( m_gridMaps[0]->isStaticPos( g.x(), g.y() ) )
				{
					// A new, static point:
					nStaticPoints++;
				}
			}
		}
	}	// End of for

	return nStaticPoints/(static_cast<float>(nTotalPoints));
}


/*---------------------------------------------------------------
					insertObservation

Insert the observation information into this map.
 ---------------------------------------------------------------*/
bool  CMultiMetricMap::internal_insertObservation(
		const CObservation	*obs,
		const CPose3D			*robotPose)
{
	int total_insert;
	MapInsertObservation op_insert_obs(*this,obs,robotPose,total_insert);
	MapExecutor::run(*this,op_insert_obs);
	return total_insert!=0;
}

/*---------------------------------------------------------------
					computeMatchingWith2D
 ---------------------------------------------------------------*/
void  CMultiMetricMap::computeMatchingWith2D(
		const CMetricMap						*otherMap,
		const CPose2D							&otherMapPose,
		float									maxDistForCorrespondence,
		float									maxAngularDistForCorrespondence,
		const CPose2D							&angularDistPivotPoint,
		TMatchingPairList						&correspondences,
		float									&correspondencesRatio,
		float									*sumSqrDist	,
		bool									onlyKeepTheClosest,
		bool									onlyUniqueRobust,
		const size_t          decimation_other_map_points,
		const size_t          offset_other_map_points ) const
{
    MRPT_START

	ASSERTMSG_( m_pointsMaps.empty()==1, "There is not exactly 1 points maps in the multimetric map." );

	m_pointsMaps[0]->computeMatchingWith2D(
									otherMap,
									otherMapPose,
									maxDistForCorrespondence,
									maxAngularDistForCorrespondence,
									angularDistPivotPoint,
									correspondences,
									correspondencesRatio,
									sumSqrDist	,
									onlyKeepTheClosest,
									onlyUniqueRobust,
									decimation_other_map_points);
    MRPT_END
}

/*---------------------------------------------------------------
					isEmpty
 ---------------------------------------------------------------*/
bool  CMultiMetricMap::isEmpty() const
{
	bool is_empty;
	MapIsEmpty op_insert_obs(is_empty);
	MapExecutor::run(*this,op_insert_obs);
	return is_empty;
}

/*---------------------------------------------------------------
					TMetricMapInitializer
 ---------------------------------------------------------------*/
TMetricMapInitializer::TMetricMapInitializer() :
	metricMapClassType(NULL),
	m_disableSaveAs3DObject(false),
	occupancyGridMap2D_options(),
	pointsMapOptions_options(),
	gasGridMap_options(),
	wifiGridMap_options(),
	landmarksMap_options(),
	beaconMap_options(),
	heightMap_options(),
	colouredPointsMapOptions_options()
{
}

/*---------------------------------------------------------------
					TOccGridMap2DOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::TOccGridMap2DOptions::TOccGridMap2DOptions() :
	min_x(-10.0f),
	max_x(10.0f),
	min_y(-10.0f),
	max_y(10.0f),
	resolution(0.10f),
	insertionOpts(),
	likelihoodOpts()
{
}

/*---------------------------------------------------------------
					CPointsMapOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::CPointsMapOptions::CPointsMapOptions() :
	insertionOpts()
{

}

/*---------------------------------------------------------------
					CLandmarksMapOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::CLandmarksMapOptions::CLandmarksMapOptions() :
	initialBeacons(),
	insertionOpts(),
	likelihoodOpts()
{

}


/*---------------------------------------------------------------
					CBeaconMapOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::CBeaconMapOptions::CBeaconMapOptions() :
	likelihoodOpts(),
	insertionOpts()
{

}

/*---------------------------------------------------------------
					CGasConcentrationGridMap2DOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::CGasConcentrationGridMap2DOptions::CGasConcentrationGridMap2DOptions() :
	min_x(-2),
	max_x(2),
	min_y(-2),
	max_y(2),
	resolution(0.10f),
	mapType(CGasConcentrationGridMap2D::mrKernelDM),
	insertionOpts()
{
}

/*---------------------------------------------------------------
					CWirelessPowerGridMap2DOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::CWirelessPowerGridMap2DOptions::CWirelessPowerGridMap2DOptions() :
	min_x(-2),
	max_x(2),
	min_y(-2),
	max_y(2),
	resolution(0.10f),
	mapType(CWirelessPowerGridMap2D::mrKernelDM),
	insertionOpts()
{
}

/*---------------------------------------------------------------
					CHeightGridMap2DOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::CHeightGridMap2DOptions::CHeightGridMap2DOptions() :
	min_x(-2),
	max_x(2),
	min_y(-2),
	max_y(2),
	resolution(0.10f),
	mapType(CHeightGridMap2D::mrSimpleAverage),
	insertionOpts()
{
}


/*---------------------------------------------------------------
					CReflectivityGridMap2DOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::CReflectivityGridMap2DOptions::CReflectivityGridMap2DOptions() :
	min_x(-2),
	max_x(2),
	min_y(-2),
	max_y(2),
	resolution(0.10f),
	insertionOpts()
{
}

/*---------------------------------------------------------------
					CColouredPointsMapOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::CColouredPointsMapOptions::CColouredPointsMapOptions() :
	insertionOpts(),
	colourOpts()
{
}

/*---------------------------------------------------------------
					CWeightedPointsMapOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::CWeightedPointsMapOptions::CWeightedPointsMapOptions() :
	insertionOpts()
{
}

/*---------------------------------------------------------------
					CGasConcentrationGridMap2DOptions
 ---------------------------------------------------------------*/
void  CMultiMetricMap::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix
	) const
{
	MRPT_START

	unsigned int		idx;

	// grid maps:
	{
		std::deque<COccupancyGridMap2DPtr>::const_iterator	it;
		for (idx=0,it = m_gridMaps.begin();it!=m_gridMaps.end();it++,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_gridmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Gas grids maps:
	{
		std::deque<CGasConcentrationGridMap2DPtr>::const_iterator	it;
		for (idx=0,it = m_gasGridMaps.begin();it!=m_gasGridMaps.end();it++,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_gasgridmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Wifi grids maps:
	{
		std::deque<CWirelessPowerGridMap2DPtr>::const_iterator	it;
		for (idx=0,it = m_wifiGridMaps.begin();it!=m_wifiGridMaps.end();it++,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_wifigridmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Points maps:
	{
		std::deque<CSimplePointsMapPtr>::const_iterator	it;
		for (idx=0,it = m_pointsMaps.begin();it!=m_pointsMaps.end();it++,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_pointsmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Landmarks maps:
	if (m_landmarksMap.present())
	{
		std::string		fil( filNamePrefix + std::string("_landmarkMap") );
		m_landmarksMap->saveMetricMapRepresentationToFile( fil );
	}

	// Landmark SOG maps:
	if (m_beaconMap.present())
	{
		std::string		fil( filNamePrefix + std::string("_beaconMap") );
		m_beaconMap->saveMetricMapRepresentationToFile( fil );
	}

	// Height grids maps:
	{
		std::deque<CHeightGridMap2DPtr>::const_iterator	it;
		for (idx=0,it = m_heightMaps.begin();it!=m_heightMaps.end();it++,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_heightgridmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Reflexivity grids maps:
	{
		std::deque<CReflectivityGridMap2DPtr>::const_iterator	it;
		for (idx=0,it = m_reflectivityMaps.begin();it!=m_reflectivityMaps.end();it++,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_reflectivitygridmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Colour points map:
	if (m_colourPointsMap.present())
	{
		std::string		fil( filNamePrefix + std::string("_colourpointsmap") );
		m_colourPointsMap->saveMetricMapRepresentationToFile( fil );
	}

	MRPT_END
}

/*---------------------------------------------------------------
		TSetOfMetricMapInitializers::loadFromConfigFile
 ---------------------------------------------------------------*/
void  TSetOfMetricMapInitializers::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &ini,
	const std::string &sectionName )
{
	MRPT_START

	std::string subSectName;

	// Delete previous contents:
	clear();

/*
		  *  ; Creation of maps:
		  *  occupancyGrid_count=<Number of mrpt::slam::COccupancyGridMap2D maps>
		  *  gasGrid_count=<Number of mrpt::slam::CGasConcentrationGridMap2D maps>
		  *  wifiGrid_count=<Number of mrpt::slam::CWirelessPowerGridMap2D maps>
		  *  landmarksMap_count=<0 or 1, for creating a mrpt::slam::CLandmarksMap map>
		  *  beaconMap_count=<0 or 1>
		  *  pointsMap_count=<0 or 1, for creating a mrpt::slam::CSimplePointsMap map>
*/

	unsigned int n = ini.read_int(sectionName,"occupancyGrid_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( COccupancyGridMap2D );

		// [<sectionName>+"_occupancyGrid_##_creationOpts"]
		subSectName = format("%s_occupancyGrid_%02u_creationOpts",sectionName.c_str(),i);

		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.occupancyGridMap2D_options.min_x	= ini.read_float(subSectName,"min_x",init.occupancyGridMap2D_options.min_x);
		init.occupancyGridMap2D_options.max_x	= ini.read_float(subSectName,"max_x",init.occupancyGridMap2D_options.max_x);
		init.occupancyGridMap2D_options.min_y	= ini.read_float(subSectName,"min_y",init.occupancyGridMap2D_options.min_y);
		init.occupancyGridMap2D_options.max_y	= ini.read_float(subSectName,"max_y",init.occupancyGridMap2D_options.max_y);
		init.occupancyGridMap2D_options.resolution = ini.read_float(subSectName,"resolution",init.occupancyGridMap2D_options.resolution);

		// [<sectionName>+"_occupancyGrid_##_insertOpts"]
		init.occupancyGridMap2D_options.insertionOpts.loadFromConfigFile(ini,format("%s_occupancyGrid_%02u_insertOpts",sectionName.c_str(),i));

		// [<sectionName>+"_occupancyGrid_##_likelihoodOpts"]
		init.occupancyGridMap2D_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_occupancyGrid_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	n = ini.read_int(sectionName,"pointsMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CSimplePointsMap );

		// [<sectionName>+"_pointsMap_##_creationOpts"]
		subSectName = format("%s_pointsMap_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);

		// [<sectionName>+"_pointsMap_##_insertOpts"]
		init.pointsMapOptions_options.insertionOpts.loadFromConfigFile(ini,format("%s_pointsMap_%02u_insertOpts",sectionName.c_str(),i));

		// [<sectionName>+"_pointsMap_##_likelihoodOpts"]
		init.pointsMapOptions_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_pointsMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	n = ini.read_int(sectionName,"gasGrid_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CGasConcentrationGridMap2D );

		// [<sectionName>+"_gasGrid_##_creationOpts"]
		subSectName = format("%s_gasGrid_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.gasGridMap_options.mapType = ini.read_enum<CGasConcentrationGridMap2D::TMapRepresentation>(subSectName,"mapType",init.gasGridMap_options.mapType);
		init.gasGridMap_options.min_x	= ini.read_float(subSectName,"min_x",init.occupancyGridMap2D_options.min_x);
		init.gasGridMap_options.max_x	= ini.read_float(subSectName,"max_x",init.occupancyGridMap2D_options.max_x);
		init.gasGridMap_options.min_y	= ini.read_float(subSectName,"min_y",init.occupancyGridMap2D_options.min_y);
		init.gasGridMap_options.max_y	= ini.read_float(subSectName,"max_y",init.occupancyGridMap2D_options.max_y);
		init.gasGridMap_options.resolution = ini.read_float(subSectName,"resolution",init.occupancyGridMap2D_options.resolution);

		// [<sectionName>+"_gasGrid_##_insertOpts"]
		init.gasGridMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_gasGrid_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	n = ini.read_int(sectionName,"wifiGrid_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CWirelessPowerGridMap2D );

		// [<sectionName>+"_wifiGrid_##_creationOpts"]
		subSectName = format("%s_wifiGrid_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.wifiGridMap_options.mapType = ini.read_enum<CWirelessPowerGridMap2D::TMapRepresentation>(subSectName,"mapType",init.wifiGridMap_options.mapType);
		init.wifiGridMap_options.min_x	= ini.read_float(subSectName,"min_x",init.occupancyGridMap2D_options.min_x);
		init.wifiGridMap_options.max_x	= ini.read_float(subSectName,"max_x",init.occupancyGridMap2D_options.max_x);
		init.wifiGridMap_options.min_y	= ini.read_float(subSectName,"min_y",init.occupancyGridMap2D_options.min_y);
		init.wifiGridMap_options.max_y	= ini.read_float(subSectName,"max_y",init.occupancyGridMap2D_options.max_y);
		init.wifiGridMap_options.resolution = ini.read_float(subSectName,"resolution",init.occupancyGridMap2D_options.resolution);

		// [<sectionName>+"_wifiGrid_##_insertOpts"]
		init.wifiGridMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_wifiGrid_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	n = ini.read_int(sectionName,"landmarksMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer				init;
		unsigned int						nBeacons;
		std::pair<CPoint3D,unsigned int>	newPair;

		init.metricMapClassType					= CLASS_ID( CLandmarksMap );

		// [<sectionName>+"_landmarksMap_##_creationOpts"]
		subSectName = format("%s_landmarksMap_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);

		init.landmarksMap_options.initialBeacons.clear();
		nBeacons = ini.read_int(subSectName,"nBeacons",0);
		for (unsigned int q=1;q<=nBeacons;q++)
		{
			newPair.second = ini.read_int(subSectName,format("beacon_%03u_ID",q),0);

			newPair.first.x( ini.read_float(subSectName,format("beacon_%03u_x",q),0) );
			newPair.first.y( ini.read_float(subSectName,format("beacon_%03u_y",q),0) );
			newPair.first.z( ini.read_float(subSectName,format("beacon_%03u_z",q),0) );

			init.landmarksMap_options.initialBeacons.push_back(newPair);
		}

		// [<sectionName>+"_landmarksMap_##_insertOpts"]
		init.landmarksMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_landmarksMap_%02u_insertOpts",sectionName.c_str(),i));

		// [<sectionName>+"_landmarksMap_##_likelihoodOpts"]
		init.landmarksMap_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_landmarksMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	n = ini.read_int(sectionName,"beaconMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer				init;

		init.metricMapClassType				= CLASS_ID( CBeaconMap );

		// [<sectionName>+"_beaconMap_##_creationOpts"]
		subSectName = format("%s_beaconMap_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);

		// [<sectionName>+"_beaconMap_##_likelihoodOpts"]
		init.beaconMap_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_beaconMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// [<sectionName>+"_beaconMap_##_insertOpts"]
		init.beaconMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_beaconMap_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i


	// Height grid maps:
	n = ini.read_int(sectionName,"heightMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CHeightGridMap2D );

		// [<sectionName>+"_heightGrid_##_creationOpts"]
		subSectName = format("%s_heightGrid_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.heightMap_options.mapType = ini.read_enum<CHeightGridMap2D::TMapRepresentation>(subSectName,"mapType",init.heightMap_options.mapType);
		init.heightMap_options.min_x	= ini.read_float(subSectName,"min_x",init.heightMap_options.min_x);
		init.heightMap_options.max_x	= ini.read_float(subSectName,"max_x",init.heightMap_options.max_x);
		init.heightMap_options.min_y	= ini.read_float(subSectName,"min_y",init.heightMap_options.min_y);
		init.heightMap_options.max_y	= ini.read_float(subSectName,"max_y",init.heightMap_options.max_y);
		init.heightMap_options.resolution = ini.read_float(subSectName,"resolution",init.heightMap_options.resolution);

		// [<sectionName>+"_heightGrid_##_insertOpts"]
		init.heightMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_heightGrid_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	// reflectivity grid maps:
	n = ini.read_int(sectionName,"reflectivityMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CHeightGridMap2D );

		// [<sectionName>+"_reflectivityGrid_##_creationOpts"]
		subSectName = format("%s_reflectivityGrid_%02u_creationOpts",sectionName.c_str(),i);
		init.m_disableSaveAs3DObject = ini.read_bool(subSectName,"disableSaveAs3DObject",false);
		init.reflectivityMap_options.min_x	= ini.read_float(subSectName,"min_x",init.heightMap_options.min_x);
		init.reflectivityMap_options.max_x	= ini.read_float(subSectName,"max_x",init.heightMap_options.max_x);
		init.reflectivityMap_options.min_y	= ini.read_float(subSectName,"min_y",init.heightMap_options.min_y);
		init.reflectivityMap_options.max_y	= ini.read_float(subSectName,"max_y",init.heightMap_options.max_y);
		init.reflectivityMap_options.resolution = ini.read_float(subSectName,"resolution",init.heightMap_options.resolution);

		// [<sectionName>+"_reflectivityGrid_##_insertOpts"]
		init.reflectivityMap_options.insertionOpts.loadFromConfigFile(ini,format("%s_reflectivityGrid_%02u_insertOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	// Colour points map:
	n = ini.read_int(sectionName,"colourPointsMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CColouredPointsMap );

		// [<sectionName>+"_colourPointsMap_##_creationOpts"]
		init.m_disableSaveAs3DObject = ini.read_bool(format("%s_colourPointsMap_%02u_creationOpts",sectionName.c_str(),i),"disableSaveAs3DObject",false);

		// [<sectionName>+"_colourPointsMap_##_insertOpts"]
		init.colouredPointsMapOptions_options.insertionOpts.loadFromConfigFile(ini, format("%s_colourPointsMap_%02u_insertOpts",sectionName.c_str(),i) );

		// [<sectionName>+"_colourPointsMap_##_colorOpts"]
		init.colouredPointsMapOptions_options.colourOpts.loadFromConfigFile(ini, format("%s_colourPointsMap_%02u_colorOpts",sectionName.c_str(),i) );

		// [<sectionName>+"_pointsMap_##_likelihoodOpts"]
		init.colouredPointsMapOptions_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_colourPointsMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i

	// Weighted points map:
	n = ini.read_int(sectionName,"weightedPointsMap_count",0);
	for (unsigned int i=0;i<n;i++)
	{
		TMetricMapInitializer	init;

		init.metricMapClassType					= CLASS_ID( CWeightedPointsMap );

		// [<sectionName>+"_weightedPointsMap_##_creationOpts"]
		init.m_disableSaveAs3DObject = ini.read_bool(format("%s_weightedPointsMap_%02u_creationOpts",sectionName.c_str(),i),"disableSaveAs3DObject",false);

		// [<sectionName>+"_weightedPointsMap_##_insertOpts"]
		init.weightedPointsMapOptions_options.insertionOpts.loadFromConfigFile(ini, format("%s_weightedPointsMap_%02u_insertOpts",sectionName.c_str(),i) );

		// [<sectionName>+"_weightedPointsMap_##_likelihoodOpts"]
		init.weightedPointsMapOptions_options.likelihoodOpts.loadFromConfigFile(ini,format("%s_weightedPointsMap_%02u_likelihoodOpts",sectionName.c_str(),i));

		// Add the map and its params to the list of "to-create":
		this->push_back(init);
	} // end for i


/*
		  *  ; Selection of map for likelihood: (occGrid=0, points=1,landmarks=2,gasGrid=3)
		  *  likelihoodMapSelection=<0-3>
*/
	MRPT_LOAD_HERE_CONFIG_VAR_CAST(likelihoodMapSelection,int,CMultiMetricMap::TOptions::TMapSelectionForLikelihood,options.likelihoodMapSelection, ini,sectionName );

/*
		  *  ; Enables (1) / Disables (0) insertion into specific maps:
		  *  enableInsertion_pointsMap=<0/1>
		  *  enableInsertion_landmarksMap=<0/1>
		  *  enableInsertion_gridMaps=<0/1>
		  *  enableInsertion_gasGridMaps=<0/1>
		  *  enableInsertion_wifiGridMaps=<0/1>
		  *  enableInsertion_beaconMap=<0/1>
*/
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_pointsMap,bool,	options.enableInsertion_pointsMap,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_landmarksMap,bool, options.enableInsertion_landmarksMap,	ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_gridMaps,bool,		options.enableInsertion_gridMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_gasGridMaps,bool,	options.enableInsertion_gasGridMaps,	ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_wifiGridMaps,bool,	options.enableInsertion_wifiGridMaps,	ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_beaconMap,bool,	options.enableInsertion_beaconMap,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_heightMaps,bool,	options.enableInsertion_heightMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_reflectivityMaps,bool,	options.enableInsertion_reflectivityMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_colourPointsMaps,bool,	options.enableInsertion_colourPointsMaps,		ini,sectionName);
	MRPT_LOAD_HERE_CONFIG_VAR(enableInsertion_weightedPointsMaps,bool,	options.enableInsertion_weightedPointsMaps,		ini,sectionName);

	MRPT_END
}

/*---------------------------------------------------------------
		TSetOfMetricMapInitializers::dumpToTextStream
 ---------------------------------------------------------------*/
void  TSetOfMetricMapInitializers::dumpToTextStream(CStream	&out) const
{
	MRPT_START

	out.printf("====================================================================\n\n");
	out.printf("             Set of internal maps for 'CMultiMetricMap' object\n\n");
	out.printf("====================================================================\n");

	out.printf("likelihoodMapSelection                  = %s\n",
		TEnumType<CMultiMetricMap::TOptions::TMapSelectionForLikelihood>::value2name(options.likelihoodMapSelection).c_str() );

	LOADABLEOPTS_DUMP_VAR(options.enableInsertion_pointsMap		, bool)
	LOADABLEOPTS_DUMP_VAR(options.enableInsertion_landmarksMap		, bool)
	LOADABLEOPTS_DUMP_VAR(options.enableInsertion_beaconMap		, bool)
	LOADABLEOPTS_DUMP_VAR(options.enableInsertion_gridMaps		, bool)
	LOADABLEOPTS_DUMP_VAR(options.enableInsertion_gasGridMaps		, bool)
	LOADABLEOPTS_DUMP_VAR(options.enableInsertion_wifiGridMaps		, bool)
	LOADABLEOPTS_DUMP_VAR(options.enableInsertion_reflectivityMaps		, bool)
	LOADABLEOPTS_DUMP_VAR(options.enableInsertion_colourPointsMaps		, bool)
	LOADABLEOPTS_DUMP_VAR(options.enableInsertion_weightedPointsMaps	, bool)

	// Show each map:
	out.printf("Showing next the %u internal maps:\n\n", (int)size());

	int i=0;
	for (const_iterator it=begin();it!=end();++it,i++)
	{
		out.printf("------------------------- Internal map %u out of %u --------------------------\n",i+1,(int)size());

		out.printf("                 C++ Class: '%s'\n", it->metricMapClassType->className);

		if (it->metricMapClassType==CLASS_ID(COccupancyGridMap2D))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("resolution                              = %0.3f\n",it->occupancyGridMap2D_options.resolution);
			out.printf("min_x                                   = %0.3f\n",it->occupancyGridMap2D_options.min_x);
			out.printf("max_x                                   = %0.3f\n",it->occupancyGridMap2D_options.max_x);
			out.printf("min_y                                   = %0.3f\n",it->occupancyGridMap2D_options.min_y);
			out.printf("max_y                                   = %0.3f\n",it->occupancyGridMap2D_options.max_y);

			it->occupancyGridMap2D_options.insertionOpts.dumpToTextStream(out);
			it->occupancyGridMap2D_options.likelihoodOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CLandmarksMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("number of initial beacons               = %u\n",(int)it->landmarksMap_options.initialBeacons.size());

			out.printf("      ID         (X,Y,Z)\n");
			out.printf("--------------------------------------------------------\n");
			for (std::deque<CMultiMetricMap::TPairIdBeacon>::const_iterator p=it->landmarksMap_options.initialBeacons.begin();p!=it->landmarksMap_options.initialBeacons.end();++p)
				out.printf("      %03u         (%8.03f,%8.03f,%8.03f)\n", p->second,p->first.x(),p->first.y(),p->first.z());

			it->landmarksMap_options.insertionOpts.dumpToTextStream(out);
			it->landmarksMap_options.likelihoodOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CBeaconMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->beaconMap_options.likelihoodOpts.dumpToTextStream(out);
			it->beaconMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CGasConcentrationGridMap2D))
		{
			out.printf("m_disableSaveAs3DObject                   = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("MAP TYPE                                  = %s\n", mrpt::utils::TEnumType<CGasConcentrationGridMap2D::TMapRepresentation>::value2name(it->gasGridMap_options.mapType).c_str() );
			out.printf("min_x                                     = %f\n", it->gasGridMap_options.min_x );
			out.printf("max_x                                     = %f\n", it->gasGridMap_options.max_x );
			out.printf("min_y                                     = %f\n", it->gasGridMap_options.min_y );
			out.printf("max_y                                     = %f\n", it->gasGridMap_options.max_y );
			out.printf("resolution                                = %f\n", it->gasGridMap_options.resolution );

			it->gasGridMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CWirelessPowerGridMap2D))
		{
			out.printf("m_disableSaveAs3DObject                   = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("MAP TYPE                                  = %s\n", mrpt::utils::TEnumType<CWirelessPowerGridMap2D::TMapRepresentation>::value2name(it->wifiGridMap_options.mapType).c_str() );
			out.printf("min_x                                     = %f\n", it->wifiGridMap_options.min_x );
			out.printf("max_x                                     = %f\n", it->wifiGridMap_options.max_x );
			out.printf("min_y                                     = %f\n", it->wifiGridMap_options.min_y );
			out.printf("max_y                                     = %f\n", it->wifiGridMap_options.max_y );
			out.printf("resolution                                = %f\n", it->wifiGridMap_options.resolution );

			it->wifiGridMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CHeightGridMap2D))
		{
			out.printf("m_disableSaveAs3DObject                   = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("MAP TYPE                                  = %s\n", mrpt::utils::TEnumType<CHeightGridMap2D::TMapRepresentation>::value2name(it->heightMap_options.mapType).c_str() );
			out.printf("min_x                                     = %f\n", it->heightMap_options.min_x );
			out.printf("max_x                                     = %f\n", it->heightMap_options.max_x );
			out.printf("min_y                                     = %f\n", it->heightMap_options.min_y );
			out.printf("max_y                                     = %f\n", it->heightMap_options.max_y );
			out.printf("resolution                                = %f\n", it->heightMap_options.resolution );

			it->heightMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CReflectivityGridMap2D))
		{
			out.printf("m_disableSaveAs3DObject                   = %s\n",it->m_disableSaveAs3DObject ? "true":"false");
			out.printf("min_x                                     = %f\n", it->reflectivityMap_options.min_x );
			out.printf("max_x                                     = %f\n", it->reflectivityMap_options.max_x );
			out.printf("min_y                                     = %f\n", it->reflectivityMap_options.min_y );
			out.printf("max_y                                     = %f\n", it->reflectivityMap_options.max_y );
			out.printf("resolution                                = %f\n", it->reflectivityMap_options.resolution );

			it->reflectivityMap_options.insertionOpts.dumpToTextStream(out);
		}
		else
		if (it->metricMapClassType==CLASS_ID(CSimplePointsMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->pointsMapOptions_options.insertionOpts.dumpToTextStream(out);
			it->pointsMapOptions_options.likelihoodOpts.dumpToTextStream(out);
		}
		else
			if (it->metricMapClassType==CLASS_ID(CColouredPointsMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->colouredPointsMapOptions_options.insertionOpts.dumpToTextStream(out);
			it->colouredPointsMapOptions_options.likelihoodOpts.dumpToTextStream(out);
			it->colouredPointsMapOptions_options.colourOpts.dumpToTextStream(out);
		}
		else
			if (it->metricMapClassType==CLASS_ID(CWeightedPointsMap))
		{
			out.printf("m_disableSaveAs3DObject                 = %s\n",it->m_disableSaveAs3DObject ? "true":"false");

			it->weightedPointsMapOptions_options.insertionOpts.dumpToTextStream(out);
			it->weightedPointsMapOptions_options.likelihoodOpts.dumpToTextStream(out);
		}
		else
		{
			THROW_EXCEPTION_CUSTOM_MSG1("Unknown class!: '%s'",it->metricMapClassType->className);
		}
	} // for "it"

	MRPT_END
}


/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CMultiMetricMap::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	MRPT_START

	// Points maps:
	{
		std::deque<CSimplePointsMapPtr>::const_iterator	it;
		for (it = m_pointsMaps.begin();it!=m_pointsMaps.end();it++)
			(*it)->getAs3DObject( outObj );
	}

	// grid maps:
	{
		std::deque<COccupancyGridMap2DPtr>::const_iterator	it;
		for (it = m_gridMaps.begin();it!=m_gridMaps.end();it++)
			(*it)->getAs3DObject( outObj );
	}

	// Gas grids maps:
	{
		std::deque<CGasConcentrationGridMap2DPtr>::const_iterator	it;
		for (it = m_gasGridMaps.begin();it!=m_gasGridMaps.end();it++)
			(*it)->getAs3DObject( outObj );
	}

	// Wifi grids maps:
	{
		std::deque<CWirelessPowerGridMap2DPtr>::const_iterator	it;
		for (it = m_wifiGridMaps.begin();it!=m_wifiGridMaps.end();it++)
			(*it)->getAs3DObject( outObj );
	}

	// Landmarks maps:
	if (m_landmarksMap.present())
		m_landmarksMap->getAs3DObject( outObj );

	// Landmark SOG maps:
	if (m_beaconMap.present())
		m_beaconMap->getAs3DObject( outObj );

	// Height grids maps:
	{
		std::deque<CHeightGridMap2DPtr>::const_iterator	it;
		for (it = m_heightMaps.begin();it!=m_heightMaps.end();it++)
			(*it)->getAs3DObject( outObj );
	}

	// Reflexivity grids maps:
	{
		std::deque<CReflectivityGridMap2DPtr>::const_iterator	it;
		for (it = m_reflectivityMaps.begin();it!=m_reflectivityMaps.end();it++)
			(*it)->getAs3DObject( outObj );
	}

	// Colour Maps:
	if (m_colourPointsMap.present())
		m_colourPointsMap->getAs3DObject( outObj );

	MRPT_END
}


/*---------------------------------------------------------------
 Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
  ---------------------------------------------------------------*/
float  CMultiMetricMap::compute3DMatchingRatio(
		const CMetricMap								*otherMap,
		const CPose3D							&otherMapPose,
		float									minDistForCorr,
		float									minMahaDistForCorr
		) const
{
	MRPT_START

	size_t		nMapsComputed = 0;
	float		accumResult = 0;

	// grid maps: NO

	// Gas grids maps: NO

	// Wifi grids maps: NO

	// Points maps:
	if (m_pointsMaps.size()>0)
	{
		ASSERT_(m_pointsMaps.size()==1);
		accumResult += m_pointsMaps[0]->compute3DMatchingRatio( otherMap, otherMapPose,minDistForCorr,minMahaDistForCorr );
		nMapsComputed++;
	}

	// Landmarks maps:
	if (m_landmarksMap.present())
	{
		accumResult += m_landmarksMap->compute3DMatchingRatio( otherMap, otherMapPose,minDistForCorr,minMahaDistForCorr );
		nMapsComputed++;
	}

	// Landmark SOG maps:
	if (m_beaconMap.present())
	{
		accumResult += m_beaconMap->compute3DMatchingRatio( otherMap, otherMapPose,minDistForCorr,minMahaDistForCorr );
		nMapsComputed++;
	}

	// Return average:
	if (nMapsComputed) accumResult/=nMapsComputed;
	return accumResult;

	MRPT_END
}

/*---------------------------------------------------------------
					auxParticleFilterCleanUp
 ---------------------------------------------------------------*/
void  CMultiMetricMap::auxParticleFilterCleanUp()
{
	MRPT_START

	// grid maps:
	{
		std::deque<COccupancyGridMap2DPtr>::iterator	it;
		for (it = m_gridMaps.begin();it!=m_gridMaps.end();it++)
			(*it)->auxParticleFilterCleanUp( );
	}

	// Gas grids maps:
	{
		std::deque<CGasConcentrationGridMap2DPtr>::iterator	it;
		for (it = m_gasGridMaps.begin();it!=m_gasGridMaps.end();it++)
			(*it)->auxParticleFilterCleanUp( );
	}

	// Wifi grids maps:
	{
		std::deque<CWirelessPowerGridMap2DPtr>::iterator	it;
		for (it = m_wifiGridMaps.begin();it!=m_wifiGridMaps.end();it++)
			(*it)->auxParticleFilterCleanUp( );
	}

	// Points maps:
	{
		std::deque<CSimplePointsMapPtr>::iterator	it;
		for (it = m_pointsMaps.begin();it!=m_pointsMaps.end();it++)
			(*it)->auxParticleFilterCleanUp( );
	}

	// Landmarks maps:
	if (m_landmarksMap.present())
		m_landmarksMap->auxParticleFilterCleanUp( );

	// Landmark SOG maps:
	if (m_beaconMap.present())
		m_beaconMap->auxParticleFilterCleanUp( );

	MRPT_END
}



/** Load parameters from configuration source
  */
void  CMultiMetricMap::TOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string		&section)
{
	likelihoodMapSelection = source.read_enum<TMapSelectionForLikelihood>(section,"likelihoodMapSelection",likelihoodMapSelection);

	MRPT_LOAD_CONFIG_VAR(enableInsertion_pointsMap, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_landmarksMap, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_gridMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_gasGridMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_wifiGridMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_beaconMap, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_heightMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_reflectivityMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_colourPointsMaps, bool,  source, section );

}

/** This method must display clearly all the contents of the structure in textual form, sending it to a CStream.
  */
void  CMultiMetricMap::TOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CMultiMetricMap::TOptions] ------------ \n\n");

	out.printf("likelihoodMapSelection                  = %i\n",	static_cast<int>(likelihoodMapSelection) );
	out.printf("enableInsertion_pointsMap               = %c\n",	enableInsertion_pointsMap ? 'Y':'N');
	out.printf("enableInsertion_landmarksMap            = %c\n",	enableInsertion_landmarksMap ? 'Y':'N');
	out.printf("enableInsertion_gridMaps                = %c\n",	enableInsertion_gridMaps ? 'Y':'N');
	out.printf("enableInsertion_gasGridMaps             = %c\n",	enableInsertion_gasGridMaps ? 'Y':'N');
	out.printf("enableInsertion_wifiGridMaps             = %c\n",	enableInsertion_gasGridMaps ? 'Y':'N');
	out.printf("enableInsertion_beaconMap               = %c\n",	enableInsertion_beaconMap ? 'Y':'N');

	out.printf("\n");
}


/** If the map is a simple points map or it's a multi-metric map that contains EXACTLY one simple points map, return it.
* Otherwise, return NULL
*/
const CSimplePointsMap * CMultiMetricMap::getAsSimplePointsMap() const
{
	MRPT_START
	ASSERT_(m_pointsMaps.size()==1 || m_pointsMaps.size()==0)
	if (m_pointsMaps.empty()) return NULL;
	else return m_pointsMaps[0].pointer();
	MRPT_END
}
CSimplePointsMap * CMultiMetricMap::getAsSimplePointsMap()
{
	MRPT_START
	ASSERT_(m_pointsMaps.size()==1 || m_pointsMaps.size()==0)
	if (m_pointsMaps.empty()) return NULL;
	else return m_pointsMaps[0].pointer();
	MRPT_END
}

