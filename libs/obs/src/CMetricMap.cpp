/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
//
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::tfest;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CMetricMap, CSerializable, mrpt::maps)

CMetricMap::CMetricMap() = default;
/** Erase all the contents of the map */
void CMetricMap::clear()
{
	internal_clear();
	publishEvent(mrptEventMetricMapClear(this));
}

void CMetricMap::loadFromProbabilisticPosesAndObservations(
	const mrpt::maps::CSimpleMap& sfSeq)
{
	// Erase previous contents:
	this->clear();

	// Insert new content:
	for (const auto& pair : sfSeq)
	{
		ASSERTMSG_(pair.pose, "Input map has an empty `CPose3DPDF` ptr");
		ASSERTMSG_(pair.sf, "Input map has an empty `CSensoryFrame` ptr");

		pair.sf->insertObservationsInto(*this, pair.pose->getMeanVal());
	}
}

double CMetricMap::computeObservationsLikelihood(
	const CSensoryFrame& sf, const CPose3D& takenFrom)
{
	double lik = 0;
	for (const auto& it : sf)
		lik += computeObservationLikelihood(*it, takenFrom);

	return lik;
}

bool CMetricMap::canComputeObservationsLikelihood(const CSensoryFrame& sf) const
{
	bool can = false;
	for (auto it = sf.begin(); !can && it != sf.end(); ++it)
		can = can || canComputeObservationLikelihood(**it);
	return can;
}

bool CMetricMap::insertObservation(
	const CObservation& obs,
	const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
	if (!genericMapParams.enableObservationInsertion) return false;

	bool done = internal_insertObservation(obs, robotPose);
	if (done)
	{
		OnPostSuccesfulInsertObs(obs);
		publishEvent(mrptEventMetricMapInsert(this, &obs, robotPose));
	}
	return done;
}

bool CMetricMap::insertObservationPtr(
	const CObservation::Ptr& obs,
	const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
	MRPT_START
	if (!obs) { THROW_EXCEPTION("Trying to pass a null pointer."); }
	return insertObservation(*obs, robotPose);
	MRPT_END
}

void CMetricMap::determineMatching2D(
	[[maybe_unused]] const mrpt::maps::CMetricMap* otherMap,
	[[maybe_unused]] const CPose2D& otherMapPose,
	[[maybe_unused]] TMatchingPairList& correspondences,
	[[maybe_unused]] const TMatchingParams& params,
	[[maybe_unused]] TMatchingExtraResults& extraResults) const
{
	MRPT_START
	THROW_EXCEPTION("Virtual method not implemented in derived class.");
	MRPT_END
}

void CMetricMap::determineMatching3D(
	[[maybe_unused]] const mrpt::maps::CMetricMap* otherMap,
	[[maybe_unused]] const CPose3D& otherMapPose,
	[[maybe_unused]] TMatchingPairList& correspondences,
	[[maybe_unused]] const TMatchingParams& params,
	[[maybe_unused]] TMatchingExtraResults& extraResults) const
{
	MRPT_START
	THROW_EXCEPTION("Virtual method not implemented in derived class.");
	MRPT_END
}

float CMetricMap::compute3DMatchingRatio(
	[[maybe_unused]] const mrpt::maps::CMetricMap* otherMap,
	[[maybe_unused]] const mrpt::poses::CPose3D& otherMapPose,
	[[maybe_unused]] const TMatchingRatioParams& params) const
{
	MRPT_START
	THROW_EXCEPTION("Virtual method not implemented in derived class.");
	MRPT_END
}

float CMetricMap::squareDistanceToClosestCorrespondence(
	[[maybe_unused]] float x0, [[maybe_unused]] float y0) const
{
	MRPT_START
	THROW_EXCEPTION("Virtual method not implemented in derived class.");
	MRPT_END
}

bool CMetricMap::canComputeObservationLikelihood(
	const mrpt::obs::CObservation& obs) const
{
	if (genericMapParams.enableObservationLikelihood)
		return internal_canComputeObservationLikelihood(obs);
	else
		return false;
}

double CMetricMap::computeObservationLikelihood(
	const mrpt::obs::CObservation& obs,
	const mrpt::poses::CPose3D& takenFrom) const
{
	if (genericMapParams.enableObservationLikelihood)
		return internal_computeObservationLikelihood(obs, takenFrom);
	else
		return false;
}
