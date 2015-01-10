/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/utils/CStartUpClassesRegister.h>
#include <mrpt/utils/metaprogramming.h>

using namespace mrpt::maps;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::utils::metaprogramming;

IMPLEMENTS_SERIALIZABLE( CMultiMetricMap, CMetricMap, mrpt::maps )


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
		for_each( mmm.maps.begin(),mmm.maps.end(), op );
		MRPT_END
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

struct MapComputeLikelihood
{
	const CObservation    * obs;
	const CPose3D         & takenFrom;
	double                & total_log_lik;

	MapComputeLikelihood(const CMultiMetricMap &m,const CObservation * _obs, const CPose3D & _takenFrom, double & _total_log_lik) :
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

struct MapCanComputeLikelihood
{
	const CObservation    * obs;
	bool                  & can;

	MapCanComputeLikelihood(const CMultiMetricMap &m,const CObservation * _obs, bool & _can) :
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


struct MapInsertObservation
{
	const CObservation    * obs;
	const CPose3D         * robot_pose;
	int                   & total_insert;

	MapInsertObservation(const CMultiMetricMap &m,const CObservation * _obs, const CPose3D * _robot_pose, int & _total_insert) :
		obs(_obs), robot_pose(_robot_pose),
		total_insert(_total_insert)
	{
		total_insert = 0;
	}

	template <typename PTR>
	inline void operator()(PTR &ptr) {
		bool ret = ptr->insertObservation(obs,robot_pose);
		if (ret) total_insert++;
	}
}; // end of MapInsertObservation

struct MapGetAs3DObject
{
	mrpt::opengl::CSetOfObjectsPtr & obj_gl;

	MapGetAs3DObject(mrpt::opengl::CSetOfObjectsPtr &_obj_gl) : obj_gl(_obj_gl)
	{
	}

	template <typename PTR>
	inline void operator()(PTR &ptr) {
		if (ptr.present()) ptr->getAs3DObject(obj_gl);
	}
}; // end of MapGetAs3DObject

struct MapAuxPFCleanup
{
	MapAuxPFCleanup() { }

	template <typename PTR>
	inline void operator()(PTR &ptr) {
		if (ptr.present()) ptr->auxParticleFilterCleanUp();
	}
}; // end of MapAuxPFCleanup


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
}; // end of MapIsEmpty

// ------------------- End of map-operations helper templates -------------------


// Ctor
CMultiMetricMap::CMultiMetricMap(const TSetOfMetricMapInitializers *initializers) :
	m_ID(0)
{
	MRPT_START
	setListOfMaps(initializers);
	MRPT_END
}

// Copy ctor
CMultiMetricMap::CMultiMetricMap(const mrpt::maps::CMultiMetricMap &other ) :
	m_ID(0)
{
	*this = other;	// Call the "=" operator
}

/*---------------------------------------------------------------
			setListOfMaps
  ---------------------------------------------------------------*/
void  CMultiMetricMap::setListOfMaps( const mrpt::maps::TSetOfMetricMapInitializers *initializers )
{
	MRPT_START

	// Erase current list of maps:
	deleteAllMaps();

	internal::TMetricMapTypesRegistry & mmr = internal::TMetricMapTypesRegistry::Instance();

	// Do we have any initializer?
	if (initializers!=NULL)
	{
		// Process each entry in the "initializers" and create maps accordingly:
		for (TSetOfMetricMapInitializers::const_iterator it = initializers->begin();it!=initializers->end();++it)
		{
			// Create map from the list of all params:
			mrpt::maps::CMetricMap *theMap = mmr.factoryMapObjectFromDefinition(*it->pointer());
			ASSERT_(theMap)

			// Add to the list of maps:
			this->maps.push_back( mrpt::maps::CMetricMapPtr(theMap) );
		}

	} // end if initializers!=NULL

#if 0
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
			if ( it->metricMapClassType == CLASS_ID(COctoMap) )
			{
				// -------------------------------------------------------
				//						OCTO MAPS
				// -------------------------------------------------------
				COctoMapPtr newOctomap = COctoMapPtr( new COctoMap(
					it->octoMap_options.resolution ) );

				newOctomap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;

				newOctomap->insertionOptions = it->octoMap_options.insertionOpts;
				newOctomap->likelihoodOptions= it->octoMap_options.likelihoodOpts;

				m_octoMaps.push_back( newOctomap );
			}
			else
			if ( it->metricMapClassType == CLASS_ID(CColouredOctoMap) )
			{
				// -------------------------------------------------------
				//						COLOURED OCTO MAPS
				// -------------------------------------------------------
				CColouredOctoMapPtr newOctomap = CColouredOctoMapPtr( new CColouredOctoMap(
					it->octoMap_options.resolution ) );

				newOctomap->m_disableSaveAs3DObject = it->m_disableSaveAs3DObject;

				newOctomap->insertionOptions = it->colourOctoMap_options.insertionOpts;
				newOctomap->likelihoodOptions= it->colourOctoMap_options.likelihoodOpts;

				m_colourOctoMaps.push_back( newOctomap );
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
#endif

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
mrpt::maps::CMultiMetricMap & CMultiMetricMap::operator = ( const CMultiMetricMap &other )
{
	MRPT_START

	if (this == &other) return *this;			// Who knows! :-)

	m_ID	  = other.m_ID;

	// Copy all maps and then make_unique() to really duplicate the objects:
	this->maps = other.maps;
	// invoke make_unique() operation on each smart pointer:
	ObjectMakeUnique op;
	MapExecutor::run(*this, op);

	return *this;
	MRPT_END
}

/*---------------------------------------------------------------
		Destructor
  ---------------------------------------------------------------*/
CMultiMetricMap::~CMultiMetricMap()
{
	deleteAllMaps();
}

// Deletes all maps and clears the internal lists of maps (with clear_unique(), so user copies remain alive)
void  CMultiMetricMap::deleteAllMaps()
{
	// Clear smart pointers:
	ObjectClearUnique op_clear_unique;
	MapExecutor::run(*this, op_clear_unique);

	// Clear list:
	maps.clear();
	m_ID = 0;
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CMultiMetricMap::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 11;
	else
	{
		// Version 11: simply the list of maps:
		out << static_cast<uint32_t>(m_ID);

		const uint32_t n = static_cast<uint32_t>(maps.size());
		for (uint32_t i=0;i<n;i++)
			out << *maps[i];
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CMultiMetricMap::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 11:
		{
			// ID: 
			{
				uint32_t	ID;
				in >> ID; m_ID = ID;
			}

			// List of maps:
			uint32_t  n;
			in >> n;
			this->maps.resize(n);
			for_each( maps.begin(), maps.end(), ObjectReadFromStream(&in) );

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

// Read docs in base class
double	 CMultiMetricMap::internal_computeObservationLikelihood(
			const CObservation		*obs,
			const CPose3D			&takenFrom )
{
	double ret_log_lik;
	MapComputeLikelihood op_likelihood(*this,obs,takenFrom,ret_log_lik);

	MapExecutor::run(*this,op_likelihood);

	MRPT_CHECK_NORMAL_NUMBER(ret_log_lik) //-V614
	return ret_log_lik;
}

// Read docs in base class
bool CMultiMetricMap::internal_canComputeObservationLikelihood( const CObservation *obs )
{
	bool can_comp;

	MapCanComputeLikelihood op_can_likelihood(*this,obs,can_comp);
	MapExecutor::run(*this,op_can_likelihood);
	return can_comp; //-V614
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
	const size_t nTotalPoints = points->size();
	ASSERT_( m_gridMaps.size()>0 );

	// There must be points!
	if ( !nTotalPoints ) return 0.0f;

	// Compute matching:
	mrpt::utils::TMatchingPairList correspondences;
	TMatchingExtraResults extraResults;
	TMatchingParams params;
	params.maxDistForCorrespondence = 0.95f*m_gridMaps[0]->insertionOptions.maxDistanceInsertion;

	m_gridMaps[0]->determineMatching2D(
		points,
		takenFrom,
		correspondences,
		params, extraResults);

	size_t nStaticPoints = 0;
	TPoint2D g,l;

	for (size_t i=0;i<nTotalPoints;i++)
	{
		bool	hasCoor = false;
		// Has any correspondence?
		for (mrpt::utils::TMatchingPairList::iterator corrsIt=correspondences.begin();!hasCoor && corrsIt!=correspondences.end();++corrsIt)
			if (corrsIt->other_idx==i)
				hasCoor = true;

		if ( !hasCoor )
		{
			// The distance between the point and the robot: If it is farther than the insertion max. dist.
			//   it should not be consider as an static point!!
			points->getPoint(i,l);

			CPoint2D	temp = CPoint2D(l) - takenFrom;
			if ( temp.norm() < params.maxDistForCorrespondence)
			{
				// A new point
				// ------------------------------------------
				// Translate point to global coordinates:
				g = takenFrom + l;

				if ( m_gridMaps[0]->isStaticPos( g.x, g.y ) )
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
	return total_insert!=0; //-V614
}

/*---------------------------------------------------------------
					computeMatchingWith2D
 ---------------------------------------------------------------*/
void CMultiMetricMap::determineMatching2D(
	const mrpt::maps::CMetricMap      * otherMap,
	const CPose2D         & otherMapPose,
	TMatchingPairList     & correspondences,
	const TMatchingParams & params,
	TMatchingExtraResults & extraResults ) const
{
    MRPT_START
	ASSERTMSG_( m_pointsMaps.empty()==1, "There is not exactly 1 points maps in the multimetric map." );
	m_pointsMaps[0]->determineMatching2D(otherMap,otherMapPose,correspondences,params,extraResults);
    MRPT_END
}

/*---------------------------------------------------------------
					isEmpty
 ---------------------------------------------------------------*/
bool  CMultiMetricMap::isEmpty() const
{
	bool is_empty;
	MapIsEmpty op_insert_obs(is_empty); //-V614
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
					TOctoMapOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::TOctoMapOptions::TOctoMapOptions() :
	resolution(0.10),
	insertionOpts(),
	likelihoodOpts()
{
}

/*---------------------------------------------------------------
					TColouredOctoMapOptions
 ---------------------------------------------------------------*/
TMetricMapInitializer::TColourOctoMapOptions::TColourOctoMapOptions() :
	resolution(0.10),
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
		for (idx=0,it = m_gridMaps.begin();it!=m_gridMaps.end();++it,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_gridmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// octo maps:
	{
		std::deque<COctoMapPtr>::const_iterator	it;
		for (idx=0,it = m_octoMaps.begin();it!=m_octoMaps.end();++it,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_octomap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// colored octo maps:
	{
		std::deque<CColouredOctoMapPtr>::const_iterator	it;
		for (idx=0,it = m_colourOctoMaps.begin();it!=m_colourOctoMaps.end();++it,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_colour_octomap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Gas grids maps:
	{
		std::deque<CGasConcentrationGridMap2DPtr>::const_iterator	it;
		for (idx=0,it = m_gasGridMaps.begin();it!=m_gasGridMaps.end();++it,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_gasgridmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Wifi grids maps:
	{
		std::deque<CWirelessPowerGridMap2DPtr>::const_iterator	it;
		for (idx=0,it = m_wifiGridMaps.begin();it!=m_wifiGridMaps.end();++it,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_wifigridmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Points maps:
	{
		std::deque<CSimplePointsMapPtr>::const_iterator	it;
		for (idx=0,it = m_pointsMaps.begin();it!=m_pointsMaps.end();++it,idx++)
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
		for (idx=0,it = m_heightMaps.begin();it!=m_heightMaps.end();++it,idx++)
		{
			std::string		fil( filNamePrefix );
			fil += format("_heightgridmap_no%02u",idx);
			(*it)->saveMetricMapRepresentationToFile( fil );
		}
	}

	// Reflexivity grids maps:
	{
		std::deque<CReflectivityGridMap2DPtr>::const_iterator	it;
		for (idx=0,it = m_reflectivityMaps.begin();it!=m_reflectivityMaps.end();++it,idx++)
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
						getAs3DObject
---------------------------------------------------------------*/
void  CMultiMetricMap::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	MRPT_START
	MapGetAs3DObject op_get_3D(outObj);
	MapExecutor::run(*this,op_get_3D);
	MRPT_END
}


/*---------------------------------------------------------------
 Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
 * \param  maxDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
 * \param  maxMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
  ---------------------------------------------------------------*/
float  CMultiMetricMap::compute3DMatchingRatio(
		const mrpt::maps::CMetricMap								*otherMap,
		const CPose3D							&otherMapPose,
		float									maxDistForCorr,
		float									maxMahaDistForCorr
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
		accumResult += m_pointsMaps[0]->compute3DMatchingRatio( otherMap, otherMapPose,maxDistForCorr,maxMahaDistForCorr );
		nMapsComputed++;
	}

	// Landmarks maps:
	if (m_landmarksMap.present())
	{
		accumResult += m_landmarksMap->compute3DMatchingRatio( otherMap, otherMapPose,maxDistForCorr,maxMahaDistForCorr );
		nMapsComputed++;
	}

	// Landmark SOG maps:
	if (m_beaconMap.present())
	{
		accumResult += m_beaconMap->compute3DMatchingRatio( otherMap, otherMapPose,maxDistForCorr,maxMahaDistForCorr );
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
	MapAuxPFCleanup op_cleanup;
	MapExecutor::run(*this,op_cleanup);
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
	MRPT_LOAD_CONFIG_VAR(enableInsertion_octoMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_colourOctoMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_gasGridMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_wifiGridMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_beaconMap, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_heightMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_reflectivityMaps, bool,  source, section );
	MRPT_LOAD_CONFIG_VAR(enableInsertion_colourPointsMaps, bool,  source, section );

}

/** This method must display clearly all the contents of the structure in textual form, sending it to a CStream.
  */
void  CMultiMetricMap::TOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [CMultiMetricMap::TOptions] ------------ \n\n");

	out.printf("likelihoodMapSelection                  = %i\n",	static_cast<int>(likelihoodMapSelection) );
	out.printf("enableInsertion_pointsMap               = %c\n",	enableInsertion_pointsMap ? 'Y':'N');
	out.printf("enableInsertion_landmarksMap            = %c\n",	enableInsertion_landmarksMap ? 'Y':'N');
	out.printf("enableInsertion_gridMaps                = %c\n",	enableInsertion_gridMaps ? 'Y':'N');
	out.printf("enableInsertion_octoMaps                = %c\n",	enableInsertion_octoMaps ? 'Y':'N');
	out.printf("enableInsertion_colourOctoMaps          = %c\n",	enableInsertion_colourOctoMaps? 'Y':'N');
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



/** Ctor: TOptions::TOptions
*/
CMultiMetricMap::TOptions::TOptions() :
	likelihoodMapSelection(mapFuseAll),
	enableInsertion_pointsMap  (true),
	enableInsertion_landmarksMap (true),
	enableInsertion_gridMaps(true),
	enableInsertion_gasGridMaps(true),
	enableInsertion_wifiGridMaps(true),
	enableInsertion_beaconMap(true),
	enableInsertion_heightMaps(true),
	enableInsertion_reflectivityMaps(true),
	enableInsertion_colourPointsMaps(true),
	enableInsertion_weightedPointsMaps(true),
	enableInsertion_octoMaps(true),
	enableInsertion_colourOctoMaps(true)
{
}
