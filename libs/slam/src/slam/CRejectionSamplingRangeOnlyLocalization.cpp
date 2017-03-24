/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers


#include <mrpt/slam/CRejectionSamplingRangeOnlyLocalization.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/maps/CLandmark.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/math/utils.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

/*---------------------------------------------------------------
						Constructor
---------------------------------------------------------------*/
CRejectionSamplingRangeOnlyLocalization::CRejectionSamplingRangeOnlyLocalization() :
	m_z_robot		( 0 ),
	m_sigmaRanges	( 0.10f ),
	m_oldPose(),
	m_drawIndex(0),
	m_dataPerBeacon()
{
}

/*---------------------------------------------------------------
					RS_drawFromProposal
---------------------------------------------------------------*/
void CRejectionSamplingRangeOnlyLocalization::RS_drawFromProposal( CPose2D &outSample )
{
	MRPT_START

	// Use the first beacon data to draw a pose:
	if (m_dataPerBeacon.size()==0)
		THROW_EXCEPTION("There is no information from which to draw samples!! Use 'setParams' with valid data!");

	ASSERT_(m_drawIndex<m_dataPerBeacon.size());

	float	ang = randomGenerator.drawUniform( m_dataPerBeacon[m_drawIndex].minAngle,m_dataPerBeacon[m_drawIndex].maxAngle);
	float	R = randomGenerator.drawGaussian1D( m_dataPerBeacon[m_drawIndex].radiusAtRobotPlane, m_sigmaRanges);

	// This is the point where the SENSOR is:
	outSample.x( m_dataPerBeacon[m_drawIndex].beaconPosition.x + cos(ang) * R );
	outSample.y( m_dataPerBeacon[m_drawIndex].beaconPosition.y + sin(ang) * R );

	outSample.phi( randomGenerator.drawGaussian1D( m_oldPose.phi(), DEG2RAD(2) ) );

	// Compute the robot pose P.
	//	  P = SAMPLE - ROT · SENSOR_ON_ROBOT
	CPoint2D  on(m_dataPerBeacon[m_drawIndex].sensorOnRobot.x,m_dataPerBeacon[m_drawIndex].sensorOnRobot.y);
	CPoint2D  S(outSample.x(),outSample.y());
	on = CPose2D(0,0,outSample.phi()) + on;
	S = S - on;

	outSample.x( S.x() );
	outSample.y( S.y() );

	MRPT_END
}

/*---------------------------------------------------------------
					RS_observationLikelihood
---------------------------------------------------------------*/
double CRejectionSamplingRangeOnlyLocalization::RS_observationLikelihood( const CPose2D &x)
{
	MRPT_START

	double	lik				= 1.0;
	double	m_sigmaRanges2	= square(m_sigmaRanges);

	// Evaluate the likelihood for all the observations but the "m_drawIndex":
	for (size_t i=0;i<m_dataPerBeacon.size();i++)
	{
		// TODO: height now includes the sensor "z"!!!...
		CPoint3D	P( x + CPoint3D(m_dataPerBeacon[i].sensorOnRobot.x,m_dataPerBeacon[i].sensorOnRobot.y,0) );

		if (i!=m_drawIndex)
			// Evalute:
			lik*=exp(-0.5* square( m_dataPerBeacon[i].radiusAtRobotPlane - P.distanceTo(m_dataPerBeacon[i].beaconPosition) ) / m_sigmaRanges2 );
	}

	return lik;

	MRPT_END
}

/*---------------------------------------------------------------
					setParams
---------------------------------------------------------------*/
bool CRejectionSamplingRangeOnlyLocalization::setParams(
	const CLandmarksMap				&beaconsMap,
	const CObservationBeaconRanges	&observation,
	float							sigmaRanges,
	const CPose2D					&oldPose,
	float							robot_z,
	bool							autoCheckAngleRanges )

{
	MRPT_START

	// Various vars:
	m_z_robot		= robot_z;
	m_oldPose		= oldPose;
	m_sigmaRanges	= sigmaRanges; // TODO: observation.stdError;

	double 	xMin=1e30,xMax=-1e30,yMin=1e30,yMax=-1e30,gridRes=2*m_sigmaRanges;
	float	R;

	// Save each observed beacon data:
	m_dataPerBeacon.clear();

	// Minimum radius:
	std::deque<mrpt::obs::CObservationBeaconRanges::TMeasurement>::const_iterator	it;
	size_t																i;

	// For each observation:
	for (i=0,it=observation.sensedData.begin();it!=observation.sensedData.end();++it,++i)
	{
		// Is in the map?
		const CLandmark	*lm = beaconsMap.landmarks.getByBeaconID( it->beaconID );
		if (lm!=NULL)
		{
			TDataPerBeacon		data;

			data.sensorOnRobot = it->sensorLocationOnRobot;

			data.beaconPosition.x = lm->pose_mean.x;
			data.beaconPosition.y = lm->pose_mean.y;

			// First compute squared:
			data.radiusAtRobotPlane = square(it->sensedDistance) - square( lm->pose_mean.z - robot_z );

			if (data.radiusAtRobotPlane>0)
			{
				data.radiusAtRobotPlane = sqrt( data.radiusAtRobotPlane );
				data.minAngle=-M_PIf;
				data.maxAngle= M_PIf;
				m_dataPerBeacon.push_back(data);

				//std::cout << "BEACON: " << m_dataPerBeacon.size() << "\n   " <<data.beaconPosition << " R=" << data.radiusAtRobotPlane << "\n";

				// Keep max/min:
				xMin=min(xMin,data.beaconPosition.x-(data.radiusAtRobotPlane+4*m_sigmaRanges+1.0));
				xMax=max(xMax,data.beaconPosition.x+(data.radiusAtRobotPlane+4*m_sigmaRanges+1.0));
				yMin=min(yMin,data.beaconPosition.y-(data.radiusAtRobotPlane+4*m_sigmaRanges+1.0));
				yMax=max(yMax,data.beaconPosition.y+(data.radiusAtRobotPlane+4*m_sigmaRanges+1.0));
			}
			else
			{
				printf("\nWARNING: Beacon range is shorter than distance between the robot and the beacon in the Z axis!!!: Ignoring this measurement\n");
			}
		}
	} // end for

	// ------------------------------------------------------------------------
	// Precompute the min/max angles for potential samples, for each beacon:
	// ------------------------------------------------------------------------
	if (autoCheckAngleRanges && m_dataPerBeacon.size()>1)
	{
		// Build the grid:
		// Each cell in the grid is a vector of bools, each one indicating whether some samples from the i'th beacon falls there.
		utils::CDynamicGrid<vector_bool>	grid(xMin,xMax,yMin,yMax,gridRes);
		grid.fill(vector_bool(m_dataPerBeacon.size(),false));
		vector_bool							*cell;

		// The ngular step size:
		float			Aa = DEG2RAD(5);

		// Fill the grid:
		for (i=0;i<m_dataPerBeacon.size();i++)
		{
			for (float	a=-M_PIf;a<=M_PIf;a+=Aa)
			{
				// Radius R - 3*SIGMA:
				R = m_dataPerBeacon[i].radiusAtRobotPlane - 3.0f*m_sigmaRanges;
				R = max(R,0.0f);
				cell=grid.cellByPos(
					m_dataPerBeacon[i].beaconPosition.x + cos(a)*R,
					m_dataPerBeacon[i].beaconPosition.y + sin(a)*R ); ASSERT_(cell!=NULL); (*cell)[i]=true;
				// Radius R :
				R = m_dataPerBeacon[i].radiusAtRobotPlane;
				cell=grid.cellByPos(
					m_dataPerBeacon[i].beaconPosition.x + cos(a)*R,
					m_dataPerBeacon[i].beaconPosition.y + sin(a)*R ); ASSERT_(cell!=NULL); (*cell)[i]=true;
				// Radius R + 3*SIGMA:
				R = m_dataPerBeacon[i].radiusAtRobotPlane + 3.0f*m_sigmaRanges;
				cell=grid.cellByPos(
					m_dataPerBeacon[i].beaconPosition.x + cos(a)*R,
					m_dataPerBeacon[i].beaconPosition.y + sin(a)*R ); ASSERT_(cell!=NULL); (*cell)[i]=true;
			} // end for a
		} // end for i


		// Check the angles:
		for (i=0;i<m_dataPerBeacon.size();i++)
		{
			float	maxA=-M_PIf, minA=M_PIf;
			for (float	a=-M_PIf;a<M_PIf;a+=Aa)
			{
				// Radius R - 3*SIGMA:
				R = m_dataPerBeacon[i].radiusAtRobotPlane - 3.0f*m_sigmaRanges;
				R = max(R,0.0f);
				cell=grid.cellByPos(
					m_dataPerBeacon[i].beaconPosition.x + cos(a)*R,
					m_dataPerBeacon[i].beaconPosition.y + sin(a)*R ); ASSERT_(cell!=NULL);
				if ( std::count(cell->begin(),cell->end(),true)>1) { maxA=max(maxA,a);minA=min(minA,a); }
				// Radius R :
				R = m_dataPerBeacon[i].radiusAtRobotPlane;
				cell=grid.cellByPos(
					m_dataPerBeacon[i].beaconPosition.x + cos(a)*R,
					m_dataPerBeacon[i].beaconPosition.y + sin(a)*R ); ASSERT_(cell!=NULL);
				if (std::count(cell->begin(),cell->end(),true)>1) { maxA=max(maxA,a);minA=min(minA,a); }
				// Radius R + 3*SIGMA:
				R = m_dataPerBeacon[i].radiusAtRobotPlane + 3.0f*m_sigmaRanges;
				cell=grid.cellByPos(
					m_dataPerBeacon[i].beaconPosition.x + cos(a)*R,
					m_dataPerBeacon[i].beaconPosition.y + sin(a)*R ); ASSERT_(cell!=NULL);
				if (std::count(cell->begin(),cell->end(),true)>1) { maxA=max(maxA,a);minA=min(minA,a); }
			} // end for a
			m_dataPerBeacon[i].minAngle=max(-M_PIf, minA-Aa );
			m_dataPerBeacon[i].maxAngle=min( M_PIf, maxA+Aa );
		} // end for i
	} // end if >1 beacons

	// Select best candidate for "m_drawIndex":
	float	minCoberture=1e30f;
	m_drawIndex = 0;
	for (i=0;i<m_dataPerBeacon.size();i++)
	{
		float c = m_dataPerBeacon[i].radiusAtRobotPlane*(m_dataPerBeacon[i].maxAngle - m_dataPerBeacon[i].minAngle);
		if (c<minCoberture)
		{
			minCoberture = c;
			m_drawIndex = i;
		}
	} // end for i

	// End!
	return m_dataPerBeacon.size()!=0;
	MRPT_END
}
