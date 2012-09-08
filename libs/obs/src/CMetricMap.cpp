/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs.h>   // Precompiled headers


#include <mrpt/slam/CMetricMap.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CSimpleMap.h>

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/utils.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;

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
