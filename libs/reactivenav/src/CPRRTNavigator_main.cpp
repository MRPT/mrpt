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

#include <mrpt/reactivenav.h>  // Precomp header

using namespace mrpt::reactivenav;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::system;
using namespace std;


const double CPRRTNavigator::INVALID_HEADING = -10;


/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CPRRTNavigator::CPRRTNavigator() :
	params			( ),
	m_initialized	(false),
	m_closingThreads(false),
	m_target_pose_time(INVALID_TIMESTAMP),
	m_last_obstacles_time(INVALID_TIMESTAMP),
	m_planned_path_time(INVALID_TIMESTAMP)
{
	// Launch threads:
 	m_thr_planner = mrpt::system::createThreadFromObjectMethod(this, &CPRRTNavigator::thread_planner);
 	m_thr_testcol = mrpt::system::createThreadFromObjectMethod(this, &CPRRTNavigator::thread_test_collision);
 	m_thr_pathtrack = mrpt::system::createThreadFromObjectMethod(this, &CPRRTNavigator::thread_path_tracking);
}

/*---------------------------------------------------------------
							Destructor
  ---------------------------------------------------------------*/
CPRRTNavigator::~CPRRTNavigator()
{
	// Stop navigation:

	// Make sure the robot is still:
	//onMotionCommand(0,0);

	// End threads.
	m_closingThreads = true;


	mrpt::system::joinThread( m_thr_planner );
	mrpt::system::joinThread( m_thr_testcol );
	mrpt::system::joinThread( m_thr_pathtrack );
}


/*---------------------------------------------------------------
							initialize
  ---------------------------------------------------------------*/
bool CPRRTNavigator::initialize()
{
	try
	{
		// Are we running? Stop first:
		m_initialized = false;

		// Check needed params:
		if (params.robot_shape.size()<3)
			THROW_EXCEPTION("robot_shape must have at least 3 vertices.");

		// Initialize PTG tables:
		// ==========================
		// Clear previous contents:
		for (TListPTGs::iterator i = m_PTGs.begin();i!=m_PTGs.end();i++) delete *i;
		m_PTGs.clear();

        TParameters<double> p;
        p["ref_distance"]     = 3;        // 3 ó 4 metros.
        p["resolution"]       = 0.02;     // 2 ó 3 cm
        p["v_max"]            = params.absolute_max_v;
        p["w_max"]            = params.absolute_max_w;

        // PTG 1: C +
        p["PTG_type"]       = 1;
		p["K"]              = 1;
        m_PTGs.push_back( CParameterizedTrajectoryGenerator::CreatePTG (p));

        // PTG 2: C -
        p["PTG_type"]       = 1;
		p["K"]              = -1;
        m_PTGs.push_back( CParameterizedTrajectoryGenerator::CreatePTG (p));

        // Create list of PTGs to use: Builds the collision grid for a given list of PTGs.
      //  build_PTG_collision_grids(m_PTGs, mrpt::math::CPolygon(params.robot_shape));

		m_initialized = true;
		return true;
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return false;
	}
}

/*---------------------------------------------------------------
							navigate
  ---------------------------------------------------------------*/
void CPRRTNavigator::navigate( const mrpt::math::TPose2D  &target_pose)
{
	CCriticalSectionLocker lock(&m_target_pose_cs);
	m_target_pose = target_pose;

	m_target_pose_time = mrpt::system::now();
}
/*---------------------------------------------------------------
							navigate
  ---------------------------------------------------------------*/
void CPRRTNavigator::navigate( const mrpt::math::TPoint2D  &target_point)
{
	CCriticalSectionLocker lock(&m_target_pose_cs);
	m_target_pose.x= target_point.x;
	m_target_pose.y= target_point.y;
	m_target_pose.phi= INVALID_HEADING;

	m_target_pose_time = mrpt::system::now();
}

/*---------------------------------------------------------------
							TOptions
  ---------------------------------------------------------------*/
CPRRTNavigator::TOptions::TOptions():
	absolute_max_v	( 1.0 ),
	absolute_max_w	( DEG2RAD(120) ),
	max_accel_v		( 0.5 ),
	max_accel_w		( DEG2RAD(180) ),
	max_age_observations (0.4)
{
	pathtrack.radius_checkpoints		= 0.15;  // meters
	planner.max_time_expend_planning	= 0.200; // sec
}

/*---------------------------------------------------------------
							loadFromConfigFile
  ---------------------------------------------------------------*/
void  CPRRTNavigator::TOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string		&section)
{
	MRPT_START

	MRPT_LOAD_CONFIG_VAR(absolute_max_v,double,  source,section);
	MRPT_LOAD_CONFIG_VAR_DEGREES(absolute_max_w, source,section);
	MRPT_LOAD_CONFIG_VAR(max_accel_v,double,  source,section);
	MRPT_LOAD_CONFIG_VAR_DEGREES(max_accel_w, source,section);

	// Robot shape is a bit special to load:
	const std::string sShape = source.read_string(section,"robot_shape","",true);
	CMatrixDouble mShape;
	if (!mShape.fromMatlabStringFormat(sShape))
		THROW_EXCEPTION_CUSTOM_MSG1("Error parsing robot_shape matrix: '%s'",sShape.c_str());
	ASSERT_(size(mShape,1)==2)
	ASSERT_(size(mShape,2)>=3)

	robot_shape.clear();
	for (size_t i=0;i<size(mShape,2);i++)
		robot_shape.push_back(TPoint2D(mShape(0,i),mShape(1,i)));


	MRPT_END
}

/*---------------------------------------------------------------
							dumpToTextStream
  ---------------------------------------------------------------*/
void  CPRRTNavigator::TOptions::dumpToTextStream( CStream	&out) const
{

}

/*---------------------------------------------------------------
							processNewLocalization
  ---------------------------------------------------------------*/
void CPRRTNavigator::processNewLocalization(
	const TPose2D &new_robot_pose,
	const CMatrixDouble33 &new_robot_cov,
	TTimeStamp timestamp )
{
	m_robotStateFilter.processUpdateNewPoseLocalization(new_robot_pose,new_robot_cov,timestamp );
}

/*---------------------------------------------------------------
						processNewOdometryInfo
  ---------------------------------------------------------------*/
void CPRRTNavigator::processNewOdometryInfo(
	const TPose2D &newOdoPose,
	TTimeStamp timestamp,
	bool hasVelocities,
	float v,
	float w )
{
	m_robotStateFilter.processUpdateNewOdometry(newOdoPose,timestamp,hasVelocities,v,w);
}

/*---------------------------------------------------------------
						processNewObstaclesData
  ---------------------------------------------------------------*/
void CPRRTNavigator::processNewObstaclesData(
	const mrpt::slam::CPointsMap* obstacles,
	TTimeStamp timestamp )
{
	CCriticalSectionLocker lock( &m_last_obstacles_cs );
	// save new obs data:
	obstacles->getAllPoints( m_last_obstacles_x, m_last_obstacles_y);
	m_last_obstacles_time = timestamp;
}

/*---------------------------------------------------------------
						setPathToFollow
  ---------------------------------------------------------------*/
void CPRRTNavigator::setPathToFollow(const std::list<TPathData> &path )
{
	CCriticalSectionLocker lock( &m_planned_path_cs );
	m_planned_path_time = now();
	m_planned_path = path;
}

/*---------------------------------------------------------------
						getCurrentPlannedPath
  ---------------------------------------------------------------*/
void CPRRTNavigator::getCurrentPlannedPath(std::list<TPathData> &path ) const
{
	CCriticalSectionLocker lock( &m_planned_path_cs );
	path = m_planned_path;
}

