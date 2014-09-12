/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/utils/CStream.h>
#include <mrpt/slam/CMetricMap.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CSimpleMap.h>

#include <mrpt/math/lightweight_geom_data.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CMetricMap, CSerializable, mrpt::slam)


/*---------------------------------------------------------------
						Virtual constructor
  ---------------------------------------------------------------*/
CMetricMap::CMetricMap() :
	m_disableSaveAs3DObject ( false )
{
}

/*---------------------------------------------------------------
						Virtual destructor
  ---------------------------------------------------------------*/
CMetricMap::~CMetricMap()
{

}

/** Erase all the contents of the map */
void  CMetricMap::clear()
{
	internal_clear();
	publishEvent( mrptEventMetricMapClear(this) );
}


/*---------------------------------------------------------------
Load the map contents from a CSensFrameProbSequence object,
erasing all previous content of the map.
This is automaticed invoking "insertObservation" for each
observation at the mean 3D robot pose as given by
the "poses::CPosePDF" in the CSensFrameProbSequence object.
  ---------------------------------------------------------------*/
void  CMetricMap::loadFromProbabilisticPosesAndObservations(const CSimpleMap &sfSeq )
{
	CPose3DPDFPtr		posePDF;
	CSensoryFramePtr	sf;
	const size_t n = sfSeq.size();

	// Erase previous contents:
	this->clear();

	// Insert new content:
	for (size_t i=0;i<n;i++)
	{
		sfSeq.get(i,posePDF, sf);

		CPose3D		robotPose;
		posePDF->getMean(robotPose);

		sf->insertObservationsInto(
				this,		// Insert into THIS map.
				&robotPose	// At this pose.
				);
	}
}

/*---------------------------------------------------------------
						computeObservationsLikelihood
  ---------------------------------------------------------------*/
double CMetricMap::computeObservationsLikelihood(
	const CSensoryFrame &sf,
	const CPose2D &takenFrom )
{
	double lik = 0;
	for (CSensoryFrame::const_iterator it=sf.begin();it!=sf.end();++it)
		lik += computeObservationLikelihood( it->pointer(), takenFrom );

	return lik;
}

double CMetricMap::computeObservationLikelihood( const CObservation *obs, const CPose2D &takenFrom )
{
	return computeObservationLikelihood(obs,CPose3D(takenFrom));
}

/*---------------------------------------------------------------
				canComputeObservationLikelihood
  ---------------------------------------------------------------*/
bool CMetricMap::canComputeObservationsLikelihood( const CSensoryFrame &sf )
{
	bool can = false;
	for (CSensoryFrame::const_iterator it=sf.begin();!can && it!=sf.end();++it)
		can = can || canComputeObservationLikelihood( it->pointer()  );
	return can;
}

bool CMetricMap::insertObservation(
	const CObservation *obs,
	const CPose3D *robotPose)
{
	bool done = internal_insertObservation(obs,robotPose);
	if (done)
	{
		OnPostSuccesfulInsertObs(obs);
		publishEvent( mrptEventMetricMapInsert(this,obs,robotPose) );
	}
	return done;
}

bool CMetricMap::insertObservationPtr(
	const CObservationPtr &obs,
	const CPose3D *robotPose)
{
	MRPT_START
	if (!obs.present()) { THROW_EXCEPTION("Trying to pass a null pointer."); }
	return insertObservation(obs.pointer(),robotPose);
	MRPT_END
}

bool CMetricMap::canComputeObservationLikelihood( const CObservationPtr &obs ) {
	return canComputeObservationLikelihood(obs.pointer());
}

void  CMetricMap::determineMatching2D(
	const CMetricMap      * otherMap,
	const CPose2D         & otherMapPose,
	TMatchingPairList     & correspondences,
	const TMatchingParams & params,
	TMatchingExtraResults & extraResults ) const
{
	MRPT_UNUSED_PARAM(otherMap);
	MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(correspondences);
	MRPT_UNUSED_PARAM(params);
	MRPT_UNUSED_PARAM(extraResults);
	MRPT_START
	THROW_EXCEPTION("Virtual method not implemented in derived class.")
	MRPT_END
}


void CMetricMap::determineMatching3D(
	const CMetricMap      * otherMap,
	const CPose3D         & otherMapPose,
	TMatchingPairList     & correspondences,
	const TMatchingParams & params,
	TMatchingExtraResults & extraResults ) const
{
	MRPT_UNUSED_PARAM(otherMap);
	MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(correspondences);
	MRPT_UNUSED_PARAM(params);
	MRPT_UNUSED_PARAM(extraResults);
	MRPT_START
	THROW_EXCEPTION("Virtual method not implemented in derived class.")
	MRPT_END
}


float  CMetricMap::compute3DMatchingRatio(
	const CMetricMap								*otherMap,
	const CPose3D							&otherMapPose,
	float									maxDistForCorr,
	float									maxMahaDistForCorr
) const
{
	MRPT_UNUSED_PARAM(otherMap);
	MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(maxDistForCorr);
	MRPT_UNUSED_PARAM(maxMahaDistForCorr);
	MRPT_START
	THROW_EXCEPTION("Virtual method not implemented in derived class.")
	MRPT_END
}

float CMetricMap::squareDistanceToClosestCorrespondence(
	float x0,
	float y0 ) const
{
	MRPT_UNUSED_PARAM(x0);
	MRPT_UNUSED_PARAM(y0);
	MRPT_START
	THROW_EXCEPTION("Virtual method not implemented in derived class.")
	MRPT_END
}