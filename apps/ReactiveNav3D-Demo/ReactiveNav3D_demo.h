/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/nav/reactive/CReactiveNavigationSystem3D.h>
#include <mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h>
#include <mrpt/opengl.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/utils/CObserver.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/gui.h>
#include <mrpt/utils/round.h>
#include "map2_1.xpm"
#include "map2_2.xpm"
#include "map2_3.xpm"


using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::opengl;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::kinematics;

class MyObserver : public mrpt::utils::CObserver
{
protected:
	void OnEvent(const mrptEvent &e)
	{
		if (e.isOfType<mrptEventMouseDown>())
		{
			mouse_click = 1;
		}
	}
public:
	bool mouse_click;
};


struct TRobotLaser {
	CObservation2DRangeScan m_scan;
	int						m_level;
	int						m_segments;
};

class CRobotKinects {
public:
	CSimplePointsMap	m_points;
	float				m_xrel;
	float				m_yrel;
	float				m_zrel;
	float				m_phi;
	int					m_level;
	float				m_min_range;
	float				m_max_range;
	float				m_fov_v;
	float				m_fov_h;
	float				m_pitch_angle;
	float				m_roll_angle;
	unsigned int		m_rows;
	unsigned int		m_columns;
	float				m_std_error;

	void CorrectFloorPoints(const mrpt::poses::CPose3D &kinectrelpose)
	{
		using namespace mrpt::math;
		TSegment3D ray;
		TPoint3D p1,p2,pint(0,0,0);
		TObject3D pintobj;
		TPlane ground(0,0,1,0);
		std::vector <float> x, y, z;

		m_points.getAllPoints(x,y,z,1);
		p2.x = kinectrelpose[0];
		p2.y = kinectrelpose[1];
		p2.z = kinectrelpose[2];
		ray.point2 = p2;

		for (unsigned int i=0; i<m_points.size();i++)
		{
			if (z[i] < 0)
			{
				p1.x = x[i];
				p1.y = y[i];
				p1.z = z[i];
				ray.point1 = p1;
				intersect(ray,ground,pintobj);
				ASSERT_(pintobj.isPoint())
				pintobj.getPoint(pint);
				x[i] = pint.x;
				y[i] = pint.y;
				z[i] = pint.z;
			}
		}
		m_points.setAllPoints(x,y,z);
	}


	void CorrectCeiling(const mrpt::poses::CPose3D &kinectrelpose, float height)
	{
		using namespace mrpt::math;
		TSegment3D ray;
		TPoint3D p1,p2,pint(0,0,0);
		TObject3D pintobj;
		TPlane ceiling(0,0,1,-height);
		std::vector <float> x, y, z;

		m_points.getAllPoints(x,y,z,1);

		p2.x = kinectrelpose[0];
		p2.y = kinectrelpose[1];
		p2.z = kinectrelpose[2];
		ray.point2 = p2;

		for (unsigned int i=0; i<m_points.size();i++)
		{
			if (z[i] > height)
			{
				p1.x = x[i];
				p1.y = y[i];
				p1.z = z[i];
				ray.point1 = p1;
				intersect(ray,ceiling,pintobj);
				ASSERT_(pintobj.isPoint())
				pintobj.getPoint(pint);
				x[i] = pint.x;
				y[i] = pint.y;
				z[i] = pint.z;
			}
		}
		m_points.setAllPoints(x,y,z);
	}


	void CorrectRanges(const mrpt::poses::CPose3D &kinectrelpose)
	{
		using namespace std;
		vector <float> x, y, z;
		vector <bool> deletion;

		m_points.getAllPoints(x,y,z,1);

		for (unsigned int i=0; i<m_points.size();i++)
		{
			if ((kinectrelpose.distance3DTo(x[i],y[i],z[i]) < m_min_range)||(kinectrelpose.distance3DTo(x[i],y[i],z[i]) > m_max_range))
			{
				deletion.push_back(1);
			}
			else
			{
				deletion.push_back(0);
			}
		}
		m_points.applyDeletionMask(deletion);
	}



	void KinectScan(const std::vector<mrpt::maps::COccupancyGridMap2D> &m_maps, const std::vector<double> &heights, const mrpt::poses::CPose3D &robotpose, const mrpt::poses::CPose3D &kinectrelpose)
	{
		unsigned int acc_factor = std::max(1,mrpt::utils::round<double>(80.0/m_columns));
		float h = 0, incrz;
		CObservation2DRangeScan m_auxlaser;
		mrpt::poses::CPose2D scanpose2d;
		mrpt::math::TPoint3D point;
		CSimplePointsMap row_points;
		row_points.insertionOptions.minDistBetweenLaserPoints = 0;
		m_points.clear();

		scanpose2d.x(robotpose[0]);
		scanpose2d.y(robotpose[1]);
		scanpose2d.phi(robotpose[3]);
		m_auxlaser.setSensorPose(kinectrelpose);
		m_auxlaser.aperture = m_fov_h;
		//m_auxlaser.beamAperture = 0.01;  //Optional

		//For each map
		for (unsigned int k=0;k<m_maps.size();k++)
		{
			//acc_factor is used to get a higher resolution
			m_maps[k].laserScanSimulator( m_auxlaser, scanpose2d, 0.5f, acc_factor*m_columns, m_std_error, 1, 0);
			row_points.insertObservation(&m_auxlaser);

			for (unsigned int i=0;i<m_rows;i++)
			{
				for (unsigned int j=0;j<m_columns;j++)
				{
					if (row_points.size() > acc_factor*j)
					{
						row_points.getPoint(acc_factor*j,point.x,point.y,point.z);
						incrz = kinectrelpose.distance3DTo(point.x,point.y,point.z)*tan((float(i)/(m_rows-1)-0.5)*m_fov_v+m_pitch_angle)*cos((float(j)/(m_columns-1)-0.5)*m_fov_h);
						point.z = point.z + incrz;

						//Points which belong to their height level are inserted. Otherwise they are deleted.
						if (m_maps.size() == 1)
						{
							m_points.insertPoint(point);
						}
						else
						{
							if (k == 0)
							{
								if (point.z < heights[k]) {m_points.insertPoint(point);}
							}
							else if (k == m_maps.size() - 1)
							{
								if (point.z >= h) {m_points.insertPoint(point);}
							}
							else
							{
								if ((point.z >= h)&&(point.z < h + heights[k])) {m_points.insertPoint(point);}
							}
						}
					}
				}
			}
			row_points.clear();
			h = h + heights[k];
		}

		CorrectFloorPoints(kinectrelpose);
		CorrectCeiling(kinectrelpose, 3);  //Default: ceiling height = 3 meters
		CorrectRanges(kinectrelpose);
	}
};

class CShortTermMemory {
public:
	bool								is_active;
	std::vector <mrpt::maps::COccupancyGridMap2D>	obsgrids;
	float								vision_limit;
	float								likelihood_incr;
	float								likelihood_decr;
	float								occupancy_threshold;
	math::TPoint2D						robot_ingrid;
	CSimplePointsMap					grid_points;

	float remainder(float dividend, float divisor)
	{
		while (dividend > divisor)
			dividend -= divisor;
		return dividend;
	}

	void updateObsGrids(float incrx, float incry, float phi, const std::vector<CRobotKinects> &kinects, const std::vector<double> &heights )
	{
		using namespace std;
		using mrpt::utils::square;
		//First, move the robot respect to the grid and adjust the likelihood values in the grid according to that movement
		//-----------------------------------------------------------------------------------------------------------------

		if (( abs(robot_ingrid.x + incrx) < obsgrids[0].getResolution())&&( abs(robot_ingrid.y + incry) < obsgrids[0].getResolution()))
		// The grid doesn't have to be diplaced
		{
			robot_ingrid.x = robot_ingrid.x + incrx;
			robot_ingrid.y = robot_ingrid.y + incry;
		}
		else if (sqrt(square(incrx) + square(incry)) > 2.6*obsgrids[0].getXMax())
		// The displacement is too big so the grid is reset
		{
			for (unsigned int i=0; i < obsgrids.size(); i++)
			{
				obsgrids[i].setSize(obsgrids[0].getXMin(), obsgrids[0].getXMax(), obsgrids[0].getYMin(), obsgrids[0].getXMax(), obsgrids[0].getResolution(), 0.5);
			}
		}
		else
		// The grid is displaced according to the robot movement
		{
			int despx = obsgrids[0].x2idx(robot_ingrid.x + incrx) - obsgrids[0].x2idx(robot_ingrid.x);
			int despy = obsgrids[0].y2idx(robot_ingrid.y + incry) - obsgrids[0].y2idx(robot_ingrid.y);
			//int despxpos = abs(despx);
			//int despypos = abs(despy);
			float despxmeters = despx*obsgrids[0].getResolution();
			float despymeters = despy*obsgrids[0].getResolution();

			float xcel, ycel;
			vector <float> cells_newval;

			//For each of the "n" grids
			for (unsigned int n=0; n < obsgrids.size(); n++)
			{
				cells_newval.clear();

				//Cell values are stored
				for (unsigned int i = 0; i < obsgrids[n].getSizeX(); i++)
				{
					for (unsigned int j = 0; j < obsgrids[n].getSizeY(); j++)
					{
						xcel = obsgrids[n].idx2x(i) + despxmeters;
						ycel = obsgrids[n].idx2y(j) + despymeters;
						if ((abs(xcel) > obsgrids[n].getXMax()) || (abs(ycel) > obsgrids[n].getYMax()))
							cells_newval.push_back(-1);
						else
							cells_newval.push_back(obsgrids[n].getCell( i + despx, j + despy));
					}
				}

				//Cell values are updated in their new "positions"
				for (unsigned int i = 0; i < obsgrids[n].getSizeX(); i++)
				{
					for (unsigned int j = 0; j < obsgrids[n].getSizeY(); j++)
					{
						if (cells_newval[j + obsgrids[n].getSizeY()*i] == -1)
							obsgrids[n].setCell( i, j, 0.5);
						else
							obsgrids[n].setCell( i, j, cells_newval[j + obsgrids[n].getSizeY()*i] );
					}
				}
			}

			robot_ingrid.x = sign<float>(robot_ingrid.x + incrx)*remainder(abs(robot_ingrid.x + incrx),obsgrids[0].getResolution());
			robot_ingrid.y = sign<float>(robot_ingrid.y + incry)*remainder(abs(robot_ingrid.y + incry),obsgrids[0].getResolution());
		}


		//Second, update the likelihood values according to kinect scan
		//-------------------------------------------------------------

		float angrot = -phi;
		float aux_xpass;
		float incr_grid_reactive = 0.2/obsgrids[0].getResolution();  //This number marks distance in meters (but it's transformed into an index)
		mrpt::math::TPoint3D paux;
		unsigned int index;
		unsigned int lim_visionxn = obsgrids[0].x2idx(-vision_limit + robot_ingrid.x);
		unsigned int lim_visionxp = obsgrids[0].x2idx(vision_limit + robot_ingrid.x);
		unsigned int lim_visionyn = obsgrids[0].x2idx(-vision_limit + robot_ingrid.y);
		unsigned int lim_visionyp = obsgrids[0].x2idx(vision_limit + robot_ingrid.y);
		unsigned int num_col = obsgrids[0].getSizeX();
		float xylim = obsgrids[0].getXMax();

		float level_height = 0.0;
		vector <bool> obs_in;
		obs_in.resize(square(num_col),0);
		grid_points.clear();

		for (unsigned int n=0; n < obsgrids.size(); n++)
		{
			obs_in.assign(square(num_col),0);

			//Vector obs_in is filled with 0 or 1 depending on the presence of any obstacle at each cell (of the grid)
			for (unsigned int i=0; i<kinects[0].m_points.size(); i++)
			{
				kinects[0].m_points.getPoint(i, paux);

				//Points rotation and translation
				aux_xpass = paux.x*cos(angrot) + paux.y*sin(angrot) + robot_ingrid.x;
				paux.y = -paux.x*sin(angrot) + paux.y*cos(angrot) + robot_ingrid.y;
				paux.x = aux_xpass;

				//Set binary occupancy of the cells (1 - there is at least one point, 0 - there isn't any point)
				if ((paux.x >= -xylim)&&(paux.x <= xylim)&&(paux.y >= -xylim)&&(paux.y <= xylim)&&(paux.z < level_height + heights[n])&&(paux.z > level_height + 0.000001))
				{
					index = obsgrids[n].x2idx(paux.x) + num_col*obsgrids[n].y2idx(paux.y);
					obs_in[index] = 1;
				}
			}

			//The likelihood values of the grid are updated
			float angle_cell, dif_angle, rango = M_PIf/6 - 0.05f;

			for (unsigned int i=0; i<num_col; i++)
			{
				for (unsigned int j=0; j<num_col; j++)
				{
					if (obs_in[i + num_col*j] == 1)
						obsgrids[n].updateCell(i,j,likelihood_incr);
					else if (((i < lim_visionxn)||(i > lim_visionxp))||((j < lim_visionyn)||(j > lim_visionyp)))
					{
						//The angle between the advance direction of the robot and the cell is calculated
						angle_cell = atan2(obsgrids[n].idx2y(j)-robot_ingrid.y, obsgrids[n].idx2x(i)-robot_ingrid.x);
						dif_angle = abs(angle_cell - phi);
						if (dif_angle > M_PI)
							dif_angle = dif_angle - 2*M_PI;
						if (abs(dif_angle) < rango)
							obsgrids[n].updateCell(i,j,likelihood_decr);
					}

					//Transform the cell with high occupancy likelihood into 3D points
					if (((i >= lim_visionxn-incr_grid_reactive)&&(i <= lim_visionxp+incr_grid_reactive))&&((j >= lim_visionyn-incr_grid_reactive)&&(j <= lim_visionyp+incr_grid_reactive)))
					{
						if (obsgrids[n].getCell(i,j) > occupancy_threshold)
						{
							paux.x = (obsgrids[n].idx2x(i)-robot_ingrid.x)*cos(-angrot) + (obsgrids[n].idx2y(j)-robot_ingrid.y)*sin(-angrot);
							paux.y = -(obsgrids[n].idx2x(i)-robot_ingrid.x)*sin(-angrot) + (obsgrids[n].idx2y(j)-robot_ingrid.y)*cos(-angrot);
							paux.z = level_height + 0.5*heights[n];
							grid_points.insertPoint(paux.x, paux.y, paux.z);
							//Include points in their level for reactive navigation
							//obstacles_inlevels[n].insertPoint(paux.x, paux.y, paux.z);
						}
					}
				}
			}
			level_height += heights[n];
		}
	}

};


class CMyReactInterface : public mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven
{
public:
	CVehicleSimul_DiffDriven        robotSim;

	CMyReactInterface() : 
		CRobot2NavInterfaceForSimulator_DiffDriven(this->robotSim)
	{
	}

	CPose2D							new_pose;
	CPose2D							last_pose;
	CPose2D							target;
	mrpt::nav::TRobotShape		robotShape;
	std::vector <mrpt::maps::COccupancyGridMap2D>	maps;
	std::vector <TRobotLaser>			lasers;
	std::vector <CRobotKinects>			kinects;
	CShortTermMemory				stm;
	gui::CDisplayWindow3D			window;
	COpenGLScenePtr					scene;
	
	bool getCurrentPoseAndSpeeds( mrpt::math::TPose2D &curPose, mrpt::math::TTwist2D &curVel, mrpt::system::TTimeStamp &timestamp) MRPT_OVERRIDE
	{
		curPose = robotSim.getCurrentGTPose();
		curVel  = robotSim.getCurrentGTVel();
		timestamp = mrpt::system::now();
		return true;
	}

	bool senseObstacles( mrpt::maps::CSimplePointsMap 	&obstacles, mrpt::system::TTimeStamp &timestamp ) MRPT_OVERRIDE
	{
		last_pose = new_pose;
		new_pose = robotSim.getCurrentGTPose();
		CPose3D robotpose3d(new_pose[0], new_pose[1], 0, new_pose[2], 0, 0);
		CPose3D kinectrelpose(0,0,0,0,0,0);

		obstacles.clear();

		//Laser scans
		for (unsigned int i=0;i< lasers.size();i++)
		{
			maps[lasers[i].m_level-1].laserScanSimulator(	lasers[i].m_scan, new_pose, 0.5f, lasers[i].m_segments,
															lasers[i].m_scan.stdError, 1, 0);

			obstacles.insertObservation(&lasers[i].m_scan);
		}
		timestamp = mrpt::system::now();

		//Depth scans
		for (unsigned int i=0; i<kinects.size();i++)
		{
			kinectrelpose.x(kinects[i].m_xrel);
			kinectrelpose.y(kinects[i].m_yrel);
			kinectrelpose.z(kinects[i].m_zrel);
			kinectrelpose.setYawPitchRoll(kinects[i].m_phi, 0, 0);
			kinects[i].KinectScan(maps, robotShape.getHeights(), robotpose3d, kinectrelpose);
			obstacles.insertAnotherMap(&kinects[i].m_points, CPose3D(0,0,0,0,0,0));
		}

		//Process depth scans in the STM
		if ( stm.is_active )
		{
			float incrx = new_pose[0] - last_pose[0];
			float incry = new_pose[1] - last_pose[1];
			stm.updateObsGrids(incrx, incry, float(new_pose.phi()), kinects, robotShape.getHeights());
			obstacles.insertAnotherMap(&stm.grid_points, CPose3D(0,0,0,0,0,0));
		}

		return true;
	}


	void loadMaps( const utils::CConfigFileBase &ini )
	{
		COccupancyGridMap2D grid;
		CImage myImg;
		//int family = ini.read_int("MAP_CONFIG","FAMILY", 1, true);
		float resolution = ini.read_float("MAP_CONFIG","MAP_RESOLUTION", 0.02f, true);
		//int num_maps = ini.read_int("MAP_CONFIG","NUM_MAPS", 1, true);

		//Maps are loaded here. Different maps can be loaded changing these lines
		//and including them above (#define...)
		myImg.loadFromXPM(map2_1_xpm);
		grid.loadFromBitmap(myImg,resolution);
		maps.push_back(grid);
		myImg.loadFromXPM(map2_2_xpm);
		grid.loadFromBitmap(myImg,resolution);
		maps.push_back(grid);
		myImg.loadFromXPM(map2_3_xpm);
		grid.loadFromBitmap(myImg,resolution);
		maps.push_back(grid);

		std::cout << std::endl << "Maps have been loaded successfully.";
	}


	void loadConfiguration( const utils::CConfigFileBase &ini )
	{
		unsigned int num_lasers, num_kinects, num_levels;
		std::vector<double> lasercoord, xaux, yaux;


		//Read lasers params
		num_lasers = ini.read_int("LASER_CONFIG","N_LASERS", 1, true);
		lasers.resize(num_lasers);
		for (unsigned int i=1;i<=num_lasers;i++)
		{
			ini.read_vector("LASER_CONFIG",format("LASER%d_POSE",i), std::vector<double> (0), lasercoord , true);
			mrpt::obs::CObservation2DRangeScan &scan = lasers[i-1].m_scan;
			scan.maxRange = ini.read_float("LASER_CONFIG",format("LASER%d_MAX_RANGE",i), 50, true);
			scan.aperture = ini.read_float("LASER_CONFIG",format("LASER%d_APERTURE",i), M_PIf, true);
			scan.stdError = ini.read_float("LASER_CONFIG",format("LASER%d_STD_ERROR",i), 0.05f, true);
			scan.sensorPose.setFromValues(lasercoord[0],lasercoord[1],lasercoord[2],lasercoord[3],lasercoord[4],lasercoord[5]);
			lasers[i-1].m_level = ini.read_int("LASER_CONFIG",format("LASER%d_LEVEL",i), 1, true);
			lasers[i-1].m_segments = ini.read_int("LASER_CONFIG",format("LASER%d_SEGMENTS",i), 181, true);
		}

		//Read kinects params
		num_kinects = ini.read_int("KINECT_CONFIG","N_KINECTS", 1, true );
		kinects.resize(num_kinects);
		for (unsigned int i=1;i<=num_kinects;i++)
		{
			kinects[i-1].m_level = ini.read_int("KINECT_CONFIG",format("KINECT%d_LEVEL",i), 1, true);
			kinects[i-1].m_xrel = ini.read_float("KINECT_CONFIG",format("KINECT%d_X",i), 0, true);
			kinects[i-1].m_yrel = ini.read_float("KINECT_CONFIG",format("KINECT%d_Y",i), 0, true);
			kinects[i-1].m_zrel = ini.read_float("KINECT_CONFIG",format("KINECT%d_Z",i), 0, true);
			kinects[i-1].m_phi = DEG2RAD(ini.read_float("KINECT_CONFIG",format("KINECT%d_PHI",i), 0, true));
			kinects[i-1].m_min_range = ini.read_float("KINECT_CONFIG",format("KINECT%d_MINRANGE",i), 0, true);
			kinects[i-1].m_max_range = ini.read_float("KINECT_CONFIG",format("KINECT%d_MAXRANGE",i), 0, true);
			kinects[i-1].m_fov_v = DEG2RAD(ini.read_float("KINECT_CONFIG",format("KINECT%d_FOV_V",i), 60, true));
			kinects[i-1].m_fov_h = DEG2RAD(ini.read_float("KINECT_CONFIG",format("KINECT%d_FOV_H",i), 60, true));
			kinects[i-1].m_pitch_angle = DEG2RAD(ini.read_float("KINECT_CONFIG",format("KINECT%d_PITCH",i), 0, true));
			kinects[i-1].m_rows = ini.read_int("KINECT_CONFIG",format("KINECT%d_ROWS",i), 10, true);
			kinects[i-1].m_columns = ini.read_int("KINECT_CONFIG",format("KINECT%d_COLUMNS",i), 10, true);
			kinects[i-1].m_std_error = ini.read_float("KINECT_CONFIG",format("KINECT%d_STD_ERROR",i), 0.05f, true);
		}

		//Read config params which describe the robot shape
		num_levels = ini.read_int("ROBOT_CONFIG","HEIGHT_LEVELS", 1, true);
		robotShape.resize(num_levels);
		for (unsigned int i=1;i<=num_levels;i++)
		{
			robotShape.setHeight(i-1, ini.read_double("ROBOT_CONFIG",format("LEVEL%d_HEIGHT",i), 1.0, true) );
			robotShape.setRadius(i-1, ini.read_double("ROBOT_CONFIG",format("LEVEL%d_RADIUS",i), 0.5, false) );
			ini.read_vector("ROBOT_CONFIG",format("LEVEL%d_VECTORX",i), std::vector<double> (0), xaux, false);
			ini.read_vector("ROBOT_CONFIG",format("LEVEL%d_VECTORY",i), std::vector<double> (0), yaux, false);
			ASSERT_(xaux.size() == yaux.size());
			for (unsigned int j=0;j<xaux.size();j++)
			{
				robotShape.polygon(i-1).AddVertex(xaux[j], yaux[j]);
			}
		}

		//Read other params associated with the robot model and its navigation
		//CRobot2NavInterface_DiffDriven::loadConfigFile(ini, "ReactiveParams");
		float tau = 0.f; //ini.read_float("ReactiveParams","ROBOTMODEL_TAU", 0, true);
		float delay = 0.f; //ini.read_float("ReactiveParams","ROBOTMODEL_DELAY", 0, true);
		float x_ini = ini.read_float("ReactiveParams","X0", 0, true);
		float y_ini = ini.read_float("ReactiveParams","Y0", 0, true);
		float phi_ini = DEG2RAD(ini.read_float("ReactiveParams","PHI0", 0, true));
		robotSim.setDelayModelParams(tau, delay);
		robotSim.resetStatus();
		robotSim.setOdometryErrors(0);
		robotSim.setCurrentGTPose(mrpt::math::TPose2D(x_ini, y_ini, phi_ini));

		//Read the "short term memory" parameters
		stm.is_active = ini.read_bool("STM_CONFIG","Stm_active", 0, 1);
		float grid_length = ini.read_float("STM_CONFIG","Obs_grid_length", 0.8f, 1);
		float grid_resolution = ini.read_float("STM_CONFIG","Obs_grid_resolution", 0.1f, 1);
		stm.vision_limit = ini.read_float("STM_CONFIG","Vision_limit", 0.6f, 1);
		stm.likelihood_incr = ini.read_float("STM_CONFIG","Pos_likelihood_incr", 0.55f, 1);
		stm.likelihood_decr = ini.read_float("STM_CONFIG","Neg_likelihood_incr", 0.45f, 1);
		stm.occupancy_threshold = ini.read_float("STM_CONFIG","Occupancy_threshold", 0.8f, 1);

		if (stm.is_active)
		{
			stm.robot_ingrid.x = 0;
			stm.robot_ingrid.y = 0;
			stm.obsgrids.resize(num_levels);
			for (unsigned int i=0; i < stm.obsgrids.size(); i++)
			{
				stm.obsgrids[i].setSize(-grid_length, grid_length, -grid_length, grid_length, grid_resolution, 0.5);
			}
		}

	}


	void initializeScene()
	{
		CPose3D robotpose3d = CPose2D(robotSim.getCurrentGTPose());


		//The display window is created
		mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 10000;
		window.setWindowTitle("Reactive Navigation. Robot motion simulation");
		window.resize(1800,980);
		window.setPos(50,0);
		window.setCameraZoom(35);
		window.setCameraAzimuthDeg(20);
		window.setCameraElevationDeg(60);
		scene = window.get3DSceneAndLock();


		//Maps are inserted
		{
			CSetOfObjectsPtr gl_grid = CSetOfObjects::Create();
			for (unsigned int i=0; i<maps.size(); i++)
			{
				maps[i].getAs3DObject(gl_grid);
				scene->insert(gl_grid);
			}
		}

		//A CornerXYZ object is inserted as an absolute frame of reference
		{
			CSetOfObjectsPtr obj = opengl::stock_objects::CornerXYZ();
			obj->setLocation(0,0,0);
			scene->insert( obj );
		}

		////A reference grid is inserted
		//{
		//	CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-16,16,-16,16,0,1);
		//	obj->setColor(0.4,0.4,0.4);
		//	obj->setLocation(0,0,0);
		//	obj->setName("gridref");
		//	scene->insert( obj );
		//}

		//The target is inserted
		{
			CDiskPtr obj = opengl::CDisk::Create(0.4f, 0.3f);
			obj->setLocation(0, 0, 0);
			obj->setColor(0.2,0.3,0.9);
			scene->insert( obj );
		}

		//The robot is inserted
		{
			float h;
			for (unsigned int i=0; i<robotShape.size(); i++)
			{
				if (i == 0) {h = 0;}
				else {h = robotShape.getHeight(i-1) + h;}

				robotpose3d.z(h);
				CPolyhedronPtr obj;
				obj = opengl::CPolyhedron::CreateCustomPrism(robotShape.polygon(i), robotShape.getHeight(i));
				obj->setName(format("Level%d",i+1));
				obj->setPose(robotpose3d);
				obj->setColor(0.2,0.5,0.2,1);
				//obj->setWireframe(true);
				obj->setLineWidth(2);
				scene->insert( obj );
			}
		}

		//One scan is simulated
		CSimplePointsMap	auxpoints;
		mrpt::system::TTimeStamp auxpoints_time;
		this->senseObstacles( auxpoints, auxpoints_time);

		robotpose3d.z() = 0;
		//The laserscans are inserted
		{
			std::vector <CPlanarLaserScanPtr> gl_scan;
			CPlanarLaserScanPtr gl_scanind;

			for (unsigned int i=0; i<lasers.size(); i++)
			{
				gl_scan.push_back(gl_scanind);
				gl_scan[i] = CPlanarLaserScan::Create();

				gl_scan[i]->enableLine(true);
				gl_scan[i]->enableSurface(false);
				gl_scan[i]->enablePoints(true);
				gl_scan[i]->setName(format("Laser%d",i+1));
				gl_scan[i]->setScan(lasers[i].m_scan);
				gl_scan[i]->setPose(robotpose3d);
				scene->insert(gl_scan[i]);
			}
		}

		//The Kinectscans are inserted
		{
			robotpose3d.z(0);
			mrpt::math::TPoint3D point;
			std::vector <CPointCloudPtr> obj;
			CPointCloudPtr indobj;

			for (unsigned int i=0;i<kinects.size();i++)
			{
				obj.push_back(indobj);
				obj[i] = opengl::CPointCloud::Create();
				obj[i]->setPose(robotpose3d);
				obj[i]->setName(format("Kinect%d",i+1));
				scene->insert(obj[i]);
				obj[i]->setColor(0,0,1);
				obj[i]->setPointSize(4.0);
				obj[i]->enablePointSmooth();
				for (unsigned int j=0; j<kinects[i].m_points.size();j++)
				{
					kinects[i].m_points.getPoint(j,point);
					obj[i]->insertPoint(point.x,point.y,point.z);
				}
			}
		}

		//Virtual obstacles from STM are inserted
		{
			CPointCloudPtr obj = CPointCloud::Create();
			obj->setPose(robotpose3d);
			obj->setPointSize(5.0);
			obj->setColor(0,1,0);
			obj->enablePointSmooth();
			obj->loadFromPointsMap<CSimplePointsMap> (&stm.grid_points);
			scene->insert( obj );
		}


		window.unlockAccess3DScene();
		std::string legend;
		legend.append("--------------------------------------------\n");
		legend.append("| m - Move the target \t|\n");
		legend.append("| p - Pause navigation \t\t|\n");
		legend.append("| r - Resume navigation \t|\n");
		legend.append("| e - Exit \t\t\t\t|\n");
		legend.append("--------------------------------------------\n");
		legend.append(format("\n        %.02fFPS", window.getRenderingFPS()));

		window.addTextMessage(5,180, legend,utils::TColorf(1,1,1),"Arial",13);
		window.repaint();
	}

	void updateScene()
	{
		scene = window.get3DSceneAndLock();
		CPose3D robotpose3d = CPose2D(robotSim.getCurrentGTPose());
		CRenderizablePtr obj;
		
		//The robot pose is updated
		{
			float h=0;
			for (unsigned int i=0; i<robotShape.size();i++)
			{
				obj = scene->getByName(format("Level%d",i+1));

				if (i == 0) {h = 0;}
				else { h = robotShape.getHeight(i-1) + h;}

				robotpose3d.z(h);
				obj->setPose(robotpose3d);
			}
		}

		robotpose3d.z() = 0;
		//The laserscan is inserted
		{
			CPlanarLaserScanPtr lasobj;

			for (unsigned int i=0; i<lasers.size(); i++)
			{
				lasobj = scene->getByClass<CPlanarLaserScan> (i);
				lasobj->setScan(lasers[i].m_scan);
				lasobj->setPose(robotpose3d);
			}
		}

		//The Kinectscan is inserted
		{
			robotpose3d.z(0);
		 mrpt::math::TPoint3D point;
			CPointCloudPtr obj;

			for (unsigned int i=0; i<kinects.size(); i++)
			{
				obj = scene->getByClass<CPointCloud> (i);
				obj->clear();
				obj->setPose(robotpose3d);

				for (unsigned int j=0; j<kinects[i].m_points.size(); j++)
				{
					kinects[i].m_points.getPoint(j,point);
					obj->insertPoint(point.x,point.y,point.z);
				}
			}
		}

		//Virtual obstacles from STM are inserted
		{
			CPointCloudPtr obj;
			obj = scene->getByClass<CPointCloud> (kinects.size());
			obj->setPose(robotpose3d);
			obj->clear();
			obj->loadFromPointsMap<CSimplePointsMap> (&stm.grid_points);
		}

		window.unlockAccess3DScene();
		std::string legend;
		legend.append("--------------------------------------------\n");
		legend.append("| m - Move the target \t|\n");
		legend.append("| p - Pause navigation \t\t|\n");
		legend.append("| r - Resume navigation \t|\n");
		legend.append("| e - Exit \t\t\t\t|\n");
		legend.append("--------------------------------------------\n");
		legend.append(format("\n        %.02fFPS", window.getRenderingFPS()));

		window.addTextMessage(5,180, legend,utils::TColorf(1,1,1),"Arial",13);
		window.repaint();
	}


	CAbstractNavigator::TNavigationParams createNewTarget(float x,
					float y, float targetAllowedDistance, bool targetIsRelative = false)
	{
		CAbstractNavigator::TNavigationParams navparams;
		navparams.target = mrpt::math::TPose2D(x,y,0);
		navparams.targetAllowedDistance = targetAllowedDistance;
		navparams.targetIsRelative = targetIsRelative;
		if (!targetIsRelative)
			target = mrpt::math::TPose2D(x, y, 0);
		else
		{
			CPose2D robotpose = robotSim.getCurrentGTPose();
			target = CPose2D(x ,y, 0) + robotpose;
		}
		return navparams;
	}

};



