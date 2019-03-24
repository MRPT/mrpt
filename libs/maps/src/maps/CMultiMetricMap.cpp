/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precompiled headers

#include <mrpt/config/CConfigFile.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPoint2D.h>
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
		// This is to avoid duplicating "::run()" for const and non-const.
		auto& mmm = const_cast<CMultiMetricMap&>(_mmm);
		std::for_each(mmm.maps.begin(), mmm.maps.end(), op);
		MRPT_END
	}
};  // end of MapExecutor

// ------------------- Begin of map-operations helper templates

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

// ---- End of map-operations helper templates

// Ctor
CMultiMetricMap::CMultiMetricMap(const TSetOfMetricMapInitializers& i)
{
	MRPT_START
	setListOfMaps(i);
	MRPT_END
}

void CMultiMetricMap::setListOfMaps(const TSetOfMetricMapInitializers& inits)
{
	MRPT_START
	// Erase current list of maps:
	maps.clear();

	auto& mmr = mrpt::maps::internal::TMetricMapTypesRegistry::Instance();

	// Process each entry in the "initializers" and create maps accordingly:
	for (const auto& i : inits)
	{
		// Create map from the list of all params:
		auto* theMap = mmr.factoryMapObjectFromDefinition(*i.get());
		ASSERT_(theMap);
		// Add to the list of maps:
		this->maps.push_back(mrpt::maps::CMetricMap::Ptr(theMap));
	}
	MRPT_END
}

void CMultiMetricMap::internal_clear()
{
	std::for_each(maps.begin(), maps.end(), [](auto ptr) {
		if (ptr) ptr->clear();
	});
}

uint8_t CMultiMetricMap::serializeGetVersion() const { return 12; }
void CMultiMetricMap::serializeTo(mrpt::serialization::CArchive& out) const
{
	const auto n = static_cast<uint32_t>(maps.size());
	out << n;
	for (uint32_t i = 0; i < n; i++) out << *maps[i];
}

void CMultiMetricMap::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 11:
		case 12:
		{
			// ID:
			if (version < 12)  // ID was removed in v12
			{
				uint32_t ID;
				in >> ID;
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
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

// Read docs in base class
double CMultiMetricMap::internal_computeObservationLikelihood(
	const CObservation* obs, const CPose3D& takenFrom)
{
	MRPT_START
	double ret_log_lik = 0;

	std::for_each(maps.begin(), maps.end(), [&](auto& ptr) {
		ret_log_lik += ptr->computeObservationLikelihood(obs, takenFrom);
	});
	return ret_log_lik;

	MRPT_END
}

// Read docs in base class
bool CMultiMetricMap::internal_canComputeObservationLikelihood(
	const CObservation* obs) const
{
	bool can_comp = false;
	std::for_each(maps.begin(), maps.end(), [&](auto& ptr) {
		can_comp = can_comp || ptr->canComputeObservationLikelihood(obs);
	});
	return can_comp;  //-V614
}

bool CMultiMetricMap::internal_insertObservation(
	const CObservation* obs, const CPose3D* robotPose)
{
	int total_insert = 0;

	std::for_each(maps.begin(), maps.end(), [&](auto& ptr) {
		const bool ret = ptr->insertObservation(obs, robotPose);
		if (ret) total_insert++;
	});
	return total_insert != 0;
}

void CMultiMetricMap::determineMatching2D(
	const mrpt::maps::CMetricMap* otherMap, const CPose2D& otherMapPose,
	TMatchingPairList& correspondences, const TMatchingParams& params,
	TMatchingExtraResults& extraResults) const
{
	MRPT_START
	const auto numPointsMaps = countMapsByClass<CSimplePointsMap>();

	ASSERTMSG_(
		numPointsMaps == 1,
		"There is not exactly 1 points maps in the multimetric map.");
	mapByClass<CSimplePointsMap>()->determineMatching2D(
		otherMap, otherMapPose, correspondences, params, extraResults);
	MRPT_END
}

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

void CMultiMetricMap::getAs3DObject(
	mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	MRPT_START
	std::for_each(maps.begin(), maps.end(), [&](auto& ptr) {
		ptr->getAs3DObject(outObj);
	});
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

void CMultiMetricMap::auxParticleFilterCleanUp()
{
	MRPT_START
	std::for_each(maps.begin(), maps.end(), [](auto& ptr) {
		ptr->auxParticleFilterCleanUp();
	});
	MRPT_END
}

const CSimplePointsMap* CMultiMetricMap::getAsSimplePointsMap() const
{
	MRPT_START
	const auto numPointsMaps = countMapsByClass<CSimplePointsMap>();
	ASSERT_(numPointsMaps == 1 || numPointsMaps == 0);
	if (!numPointsMaps)
		return nullptr;
	else
		return this->mapByClass<CSimplePointsMap>(0).get();
	MRPT_END
}

mrpt::maps::CMetricMap::Ptr CMultiMetricMap::mapByIndex(size_t idx) const
{
	MRPT_START
	return maps.at(idx).get_ptr();
	MRPT_END
}
