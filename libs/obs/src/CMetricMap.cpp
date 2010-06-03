/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
  Aligns an observation to its maximum likelihood pose (in 2D)
     into this map, by hill climbing in values computed
	 with "computeObservationLikelihood".
	\param obs The observation to be aligned
	\param in_initialEstimatedPose The initial input to the
			algorithm, an initial "guess" for the observation pose in the map.
	\param out_ResultingPose The resulting pose from this algorithm
  ---------------------------------------------------------------*/
void 	CMetricMap::alignBylikelihoodHillClimbing(
					CObservation		*obs,
					CPose2D				in_initialEstimatedPose,
					CPose2D				&out_ResultingPose )
{
	bool			iterate;
	TPose2D			curEstimation,est[10],estBest;
	double			likCur,lik[10],likBest;
	float			Axy,Ap;
	int				i;

	Axy = 0.01f;
	Ap  = DEG2RAD(1);

	// Initial value:
	curEstimation = in_initialEstimatedPose;

	// The main loop:
	// ---------------------------
	do
	{
		// Current pose:
		likCur=computeObservationLikelihood(obs,curEstimation);

		// Neighbourhood:
		est[0]=curEstimation;  est[0].x+=Axy;
		est[1]=curEstimation;  est[1].x-=Axy;
		est[2]=curEstimation;  est[2].y+=Axy;
		est[3]=curEstimation;  est[3].y-=Axy;
		est[4]=curEstimation;  est[4].phi+=Ap;
		est[5]=curEstimation;  est[5].phi-=Ap;

		est[6]=curEstimation;  est[6].x+=Axy;est[6].y+=Axy;
		est[7]=curEstimation;  est[7].x-=Axy;est[7].y+=Axy;
		est[8]=curEstimation;  est[8].x+=Axy;est[8].y-=Axy;
		est[9]=curEstimation;  est[9].x+=Axy;est[9].y-=Axy;


		// Evaluate and find the best:
		likBest=-1.0f;
		for (i=0;i<10;i++)
		{
			lik[i] = computeObservationLikelihood(obs,est[i]) - likCur;
			if (lik[i]>likBest || i==0)
			{
				likBest = lik[i];
				estBest = est[i];
			}
		}

		// for the next iteration:
		// -----------------------------
		if (likBest>0)
		{
			float	gradX = estBest.x - curEstimation.x;
			float	gradY = estBest.y - curEstimation.y;
			float	gradP = estBest.phi - curEstimation.phi;
			float	Alfa  = 1.0f;

			// Follow the incre. gradient of the MI:
			curEstimation.x+= gradX * Alfa;
			curEstimation.y+= gradY * Alfa;
			curEstimation.phi+= gradP * Alfa;

//			curEstimation = estBest;
			printf("MI_incr=%f\tGrad=(%.02f,%.02f,%.02f)\tNew=(%.03f,%.03f,%.02f)\n",likBest,gradX,gradY,RAD2DEG(gradP),curEstimation.x,curEstimation.y,RAD2DEG(curEstimation.phi));
		}


		// End criterion:
		// ---------------------------
		iterate = (likBest>0);

	} while (iterate);


	out_ResultingPose = curEstimation;
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
