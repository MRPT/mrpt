/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/config/CConfigFile.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/metaprogramming_serialization.h>

using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::tfest;
using namespace mrpt::serialization::metaprogramming;

IMPLEMENTS_SERIALIZABLE(CMultiMetricMap, CMetricMap, mrpt::maps)

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

struct MapExecutor
{
	// Apply operation to maps in the same order as declared in
	// CMultiMetricMap.h:
	template <typename OP>
	static void run(const CMultiMetricMap& _mmm, OP op)
	{
		MRPT_START
		auto& mmm =
			const_cast<CMultiMetricMap&>(_mmm);  // This is to avoid duplicating
		// "::run()" for const and
		// non-const.
		for_each(mmm.maps.begin(), mmm.maps.end(), op);
		MRPT_END
	}
};  // end of MapExecutor

// ------------------- Begin of map-operations helper templates
// -------------------
struct MapVectorClearer
{
	template <typename T>
	inline void operator()(T& container)
	{
		container.clear();
	}
};

struct MapComputeLikelihood
{
	const CObservation* obs;
	const CPose3D& takenFrom;
	double& total_log_lik;

	MapComputeLikelihood(
		const CMultiMetricMap& m, const CObservation* _obs,
		const CPose3D& _takenFrom, double& _total_log_lik)
		: obs(_obs), takenFrom(_takenFrom), total_log_lik(_total_log_lik)
	{
		total_log_lik = 0;
	}

	template <typename PTR>
	inline void operator()(PTR& ptr)
	{
		total_log_lik += ptr->computeObservationLikelihood(obs, takenFrom);
	}

};  // end of MapComputeLikelihood

struct MapCanComputeLikelihood
{
	const CObservation* obs;
	bool& can;

	MapCanComputeLikelihood(
		const CMultiMetricMap& m, const CObservation* _obs, bool& _can)
		: obs(_obs), can(_can)
	{
		can = false;
	}

	template <typename PTR>
	inline void operator()(PTR& ptr)
	{
		can = can || ptr->canComputeObservationLikelihood(obs);
	}

};  // end of MapCanComputeLikelihood

struct MapInsertObservation
{
	const CObservation* obs;
	const CPose3D* robot_pose;
	int& total_insert;

	MapInsertObservation(
		const CMultiMetricMap& m, const CObservation* _obs,
		const CPose3D* _robot_pose, int& _total_insert)
		: obs(_obs), robot_pose(_robot_pose), total_insert(_total_insert)
	{
		total_insert = 0;
	}

	template <typename PTR>
	inline void operator()(PTR& ptr)
	{
		bool ret = ptr->insertObservation(obs, robot_pose);
		if (ret) total_insert++;
	}
};  // end of MapInsertObservation

struct MapGetAs3DObject
{
	mrpt::opengl::CSetOfObjects::Ptr& obj_gl;

	MapGetAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& _obj_gl)
		: obj_gl(_obj_gl)
	{
	}

	template <typename PTR>
	inline void operator()(PTR& ptr)
	{
		if (ptr) ptr->getAs3DObject(obj_gl);
	}
};  // end of MapGetAs3DObject

struct MapAuxPFCleanup
{
	MapAuxPFCleanup() = default;
	template <typename PTR>
	inline void operator()(PTR& ptr)
	{
		if (ptr) ptr->auxParticleFilterCleanUp();
	}
};  // end of MapAuxPFCleanup

struct MapIsEmpty
{
	bool& is_empty;

	MapIsEmpty(bool& _is_empty) : is_empty(_is_empty) { is_empty = true; }
	template <typename PTR>
	inline void operator()(PTR& ptr)
	{
		if (ptr) is_empty = is_empty && ptr->isEmpty();
	}
};  // end of MapIsEmpty

// ------------------- End of map-operations helper templates
// -------------------

#define ALL_PROXIES_INIT                                                     \
	m_pointsMaps(maps), m_gridMaps(maps), m_octoMaps(maps),                  \
		m_colourOctoMaps(maps), m_gasGridMaps(maps), m_wifiGridMaps(maps),   \
		m_heightMaps(maps), m_heightMRFMaps(maps), m_reflectivityMaps(maps), \
		m_colourPointsMap(maps), m_weightedPointsMap(maps),                  \
		m_landmarksMap(maps), m_beaconMap(maps)

// Ctor
CMultiMetricMap::CMultiMetricMap(
	const TSetOfMetricMapInitializers* initializers)
	: maps(), ALL_PROXIES_INIT
{
	MRPT_START
	setListOfMaps(initializers);
	MRPT_END
}

CMultiMetricMap::CMultiMetricMap(const CMultiMetricMap& o)
	: maps(o.maps), ALL_PROXIES_INIT, m_ID(o.m_ID)
{
}

CMultiMetricMap& CMultiMetricMap::operator=(const CMultiMetricMap& o)
{
	maps = o.maps;
	m_ID = o.m_ID;
	return *this;
}

CMultiMetricMap::CMultiMetricMap(CMultiMetricMap&& o)
	: maps(std::move(o.maps)), ALL_PROXIES_INIT, m_ID(o.m_ID)
{
}

CMultiMetricMap& CMultiMetricMap::operator=(CMultiMetricMap&& o)
{
	maps = std::move(o.maps);
	m_ID = o.m_ID;
	return *this;
}

/*---------------------------------------------------------------
			setListOfMaps
  ---------------------------------------------------------------*/
void CMultiMetricMap::setListOfMaps(
	const mrpt::maps::TSetOfMetricMapInitializers* initializers)
{
	MRPT_START

	// Erase current list of maps:
	deleteAllMaps();

	auto& mmr = mrpt::maps::internal::TMetricMapTypesRegistry::Instance();

	// Do we have any initializer?
	if (initializers != nullptr)
	{
		// Process each entry in the "initializers" and create maps accordingly:
		for (const auto& initializer : *initializers)
		{
			// Create map from the list of all params:
			mrpt::maps::CMetricMap* theMap =
				mmr.factoryMapObjectFromDefinition(*initializer.get());
			ASSERT_(theMap);
			// Add to the list of maps:
			this->maps.push_back(mrpt::maps::CMetricMap::Ptr(theMap));
		}

	}  // end if initializers!=nullptr

	MRPT_END
}

void CMultiMetricMap::internal_clear()
{
	MapExecutor::run(*this, [](auto ptr) {
		if (ptr) ptr->clear();
	});
}

void CMultiMetricMap::deleteAllMaps()
{
	// Clear list:
	maps.clear();
	m_ID = 0;
}

uint8_t CMultiMetricMap::serializeGetVersion() const { return 11; }
void CMultiMetricMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	// Version 11: simply the list of maps:
	out << static_cast<uint32_t>(m_ID);
	const auto n = static_cast<uint32_t>(maps.size());
	for (uint32_t i = 0; i < n; i++) out << *maps[i];
}

void CMultiMetricMap::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 11:
		{
			// ID:
			{
				uint32_t ID;
				in >> ID;
				m_ID = ID;
			}

			// List of maps:
			uint32_t n;
			in >> n;
			this->maps.resize(n);
			for_each(
				maps.begin(), maps.end(),
				ObjectReadFromStreamToPtrs<mrpt::maps::CMetricMap::Ptr>(&in));
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

// Read docs in base class
double CMultiMetricMap::internal_computeObservationLikelihood(
	const CObservation* obs, const CPose3D& takenFrom)
{
	double ret_log_lik;
	MapComputeLikelihood op_likelihood(*this, obs, takenFrom, ret_log_lik);

	MapExecutor::run(*this, op_likelihood);

	MRPT_CHECK_NORMAL_NUMBER(ret_log_lik);  //-V614
	return ret_log_lik;
}

// Read docs in base class
bool CMultiMetricMap::internal_canComputeObservationLikelihood(
	const CObservation* obs) const
{
	bool can_comp;

	MapCanComputeLikelihood op_can_likelihood(*this, obs, can_comp);
	MapExecutor::run(*this, op_can_likelihood);
	return can_comp;  //-V614
}

/*---------------------------------------------------------------
					insertObservation

Insert the observation information into this map.
 ---------------------------------------------------------------*/
bool CMultiMetricMap::internal_insertObservation(
	const CObservation* obs, const CPose3D* robotPose)
{
	int total_insert;
	MapInsertObservation op_insert_obs(*this, obs, robotPose, total_insert);
	MapExecutor::run(*this, op_insert_obs);
	return total_insert != 0;  //-V614
}

/*---------------------------------------------------------------
					computeMatchingWith2D
 ---------------------------------------------------------------*/
void CMultiMetricMap::determineMatching2D(
	const mrpt::maps::CMetricMap* otherMap, const CPose2D& otherMapPose,
	TMatchingPairList& correspondences, const TMatchingParams& params,
	TMatchingExtraResults& extraResults) const
{
	MRPT_START
	ASSERTMSG_(
		m_pointsMaps.size() == 1,
		"There is not exactly 1 points maps in the multimetric map.");
	m_pointsMaps[0]->determineMatching2D(
		otherMap, otherMapPose, correspondences, params, extraResults);
	MRPT_END
}

/*---------------------------------------------------------------
					isEmpty
 ---------------------------------------------------------------*/
bool CMultiMetricMap::isEmpty() const
{
	bool is_empty;
	MapIsEmpty op_insert_obs(is_empty);  //-V614
	MapExecutor::run(*this, op_insert_obs);
	return is_empty;
}

void CMultiMetricMap::saveMetricMapRepresentationToFile(
	const std::string& filNamePrefix) const
{
	MRPT_START

	for (size_t idx = 0; idx < maps.size(); idx++)
	{
		const mrpt::maps::CMetricMap* m = maps[idx].get();
		ASSERT_(m);
		std::string fil = filNamePrefix;
		fil += format(
			"_%s_%02u", m->GetRuntimeClass()->className,
			static_cast<unsigned int>(idx));
		m->saveMetricMapRepresentationToFile(fil);
	}

	MRPT_END
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void CMultiMetricMap::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	MRPT_START
	MapGetAs3DObject op_get_3D(outObj);
	MapExecutor::run(*this, op_get_3D);
	MRPT_END
}

// See docs in base class
float CMultiMetricMap::compute3DMatchingRatio(
	const mrpt::maps::CMetricMap* otherMap,
	const mrpt::poses::CPose3D& otherMapPose,
	const TMatchingRatioParams& params) const
{
	MRPT_START

	float accumResult = 0;

	for (const auto& map : maps)
	{
		const mrpt::maps::CMetricMap* m = map.get();
		ASSERT_(m);
		accumResult +=
			m->compute3DMatchingRatio(otherMap, otherMapPose, params);
	}

	// Return average:
	const size_t nMapsComputed = maps.size();
	if (nMapsComputed) accumResult /= nMapsComputed;
	return accumResult;

	MRPT_END
}

/*---------------------------------------------------------------
					auxParticleFilterCleanUp
 ---------------------------------------------------------------*/
void CMultiMetricMap::auxParticleFilterCleanUp()
{
	MRPT_START
	MapAuxPFCleanup op_cleanup;
	MapExecutor::run(*this, op_cleanup);
	MRPT_END
}

/** If the map is a simple points map or it's a multi-metric map that contains
 * EXACTLY one simple points map, return it.
 * Otherwise, return NULL
 */
const CSimplePointsMap* CMultiMetricMap::getAsSimplePointsMap() const
{
	MRPT_START
	ASSERT_(m_pointsMaps.size() == 1 || m_pointsMaps.size() == 0);
	if (m_pointsMaps.empty())
		return nullptr;
	else
		return m_pointsMaps[0].get();
	MRPT_END
}
CSimplePointsMap* CMultiMetricMap::getAsSimplePointsMap()
{
	MRPT_START
	ASSERT_(m_pointsMaps.size() == 1 || m_pointsMaps.size() == 0);
	if (m_pointsMaps.empty())
		return nullptr;
	else
		return m_pointsMaps[0].get();
	MRPT_END
}

/** Gets the i-th map \exception std::runtime_error On out-of-bounds */
mrpt::maps::CMetricMap::Ptr CMultiMetricMap::getMapByIndex(size_t idx) const
{
	ASSERT_BELOW_(idx, maps.size());
	return maps[idx].get_ptr();
}
